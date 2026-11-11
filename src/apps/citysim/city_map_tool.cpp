#include "city_map_tool.h"

#include "../../engine/camera/fly_camera_controller.h"
#include "../../engine/components.h"
#include "../../log.h"
#include "../../engine/procgen/city/city_svg.h"
#include "../../engine/player/tools.h"
#include "../../engine/waypoint.h"
#include "../../engine/world.h"
#include "bus_stop_props.h"   // routeColour: the map wears the stop signs' colours
#include "city_render.h"
#include "rail_sim.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <filesystem>
#include <memory>
#include <unistd.h>

namespace citysim {

using engine::Real;
using engine::Vec2;
using Quad = engine::Renderer::UiQuad;

namespace {

// Tiles kept on the GPU (256 KB each): beyond this the least recently wanted go (the overview never does).
constexpr std::size_t kMaxTiles = 400;
// Tiles uploaded per frame: a burst of finished tiles is spread over a few frames.
constexpr int kUploadsPerFrame = 24;
// The transit lines' width, in pixels at any zoom (times each line's own factor).
constexpr float kLinePx = 4.0f;

constexpr float kFrame[4] = {0.10f, 0.10f, 0.11f, 0.92f};
constexpr float kPaper[4] = {0.84f, 0.82f, 0.77f, 1.0f};
constexpr float kYou[4] = {0.90f, 0.20f, 0.16f, 1.0f};
constexpr float kMark[4] = {1.00f, 0.78f, 0.16f, 1.0f};   // the waypoint: the watch's dot is this colour too

// A soft round dot, white (tinted per use).
std::vector<uint8_t> makeDot(int n) {
    std::vector<uint8_t> px(static_cast<std::size_t>(n) * n * 4, 255);
    const double c = (n - 1) * 0.5, r = n * 0.5 - 1.0;
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
            const double d = std::sqrt((x - c) * (x - c) + (y - c) * (y - c));
            const double a = std::clamp(r - d + 0.5, 0.0, 1.0);
            px[(static_cast<std::size_t>(y) * n + x) * 4 + 3] = static_cast<uint8_t>(a * 255.0 + 0.5);
        }
    return px;
}

// The you-are-here chevron, pointing UP (texture -v), white, 4x supersampled.
std::vector<uint8_t> makeArrow(int n) {
    std::vector<uint8_t> px(static_cast<std::size_t>(n) * n * 4, 255);
    // In [-1,1]^2, y down: tip at the top, two feet, a notch.
    const double poly[4][2] = {{0.0, -0.95}, {0.75, 0.85}, {0.0, 0.40}, {-0.75, 0.85}};
    auto inside = [&](double x, double y) {
        bool in = false;
        for (int i = 0, j = 3; i < 4; j = i++) {
            const double xi = poly[i][0], yi = poly[i][1], xj = poly[j][0], yj = poly[j][1];
            if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) in = !in;
        }
        return in;
    };
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
            int hits = 0;
            for (int sy = 0; sy < 4; ++sy)
                for (int sx = 0; sx < 4; ++sx) {
                    const double u = ((x + (sx + 0.5) / 4.0) / n) * 2.0 - 1.0;
                    const double v = ((y + (sy + 0.5) / 4.0) / n) * 2.0 - 1.0;
                    if (inside(u, v)) ++hits;
                }
            px[(static_cast<std::size_t>(y) * n + x) * 4 + 3] = static_cast<uint8_t>(hits * 255 / 16);
        }
    return px;
}

Quad tinted(Quad q, const float c[4]) {
    q.r = c[0]; q.g = c[1]; q.b = c[2]; q.a = c[3];
    return q;
}

}  // namespace

CityMapToolSystem::~CityMapToolSystem() { stopWorker(); }

void CityMapToolSystem::onStart(engine::FrameContext& ctx) {
    ctx.actions.bindButton("slot_3", engine::KeyCode::Num3);   // the map (a tool)
    // THE MAP'S OWN CONTROLS, on its input layer (ADR-0248): up, it has the stick and the keys -- the player under
    // it stands still. Pan with the left stick or WASD, zoom with the right stick, E/Q or the wheel; mark the
    // waypoint at the crosshair with A, Space or a right click (again on it: cleared); Y or C back to you; B puts
    // the map away.
    using engine::KeyCode;
    using engine::GamepadAxis;
    using engine::GamepadButton;
    engine::InputMap& a = ctx.actions;
    a.bindAxis("map_pan_x", GamepadAxis::LeftX, 1.0);
    a.bindAxis("map_pan_x", KeyCode::D, 1.0);
    a.bindAxis("map_pan_x", KeyCode::A, -1.0);
    a.bindAxis("map_pan_y", GamepadAxis::LeftY, 1.0);   // (screen down: south, +z)
    a.bindAxis("map_pan_y", KeyCode::S, 1.0);
    a.bindAxis("map_pan_y", KeyCode::W, -1.0);
    a.bindAxis("map_zoom", GamepadAxis::RightY, -1.0);  // (stick up: closer)
    a.bindAxis("map_zoom", KeyCode::E, 1.0);    // (E and Q: the body's and the freecam's keys, dead under the map;
    a.bindAxis("map_zoom", KeyCode::Q, -1.0);   //  +/- are the sim's speed)
    a.bindButton("map_centre", KeyCode::C);
    a.bindButton("map_centre", GamepadButton::Y);
    a.bindButton("map_mark", GamepadButton::A);
    a.bindButton("map_mark", KeyCode::Space);   // (jump, dead under the map; Enter is pause)
    a.bindButton("map_mark", engine::MouseButton::Right);
    a.bindButton("map_close", GamepadButton::B);
    for (const char* n : {"map_pan_x", "map_pan_y", "map_zoom", "map_centre", "map_mark", "map_close"})
        a.setActionLayer(n, "map");
}

void CityMapToolSystem::onStop(engine::FrameContext& ctx) {
    stopWorker();
    fly_.inputSuspended = false;
    ctx.actions.popLayer("map");
    for (auto& [key, tile] : tiles_)
        if (tile.tex.valid()) ctx.renderer.removeTexture(tile.tex);
    tiles_.clear();
    if (dot_.valid()) ctx.renderer.removeTexture(dot_);
    if (arrow_.valid()) ctx.renderer.removeTexture(arrow_);
    dot_ = arrow_ = engine::TextureHandle{};
    open_ = false;
}

const float* CityMapToolSystem::youColour() { return kYou; }
const float* CityMapToolSystem::frameColour() { return kFrame; }

CityMapToolSystem::Rect CityMapToolSystem::panel(int fbW, int fbH) {
    const float w = static_cast<float>(fbW);
    const float h = static_cast<float>(fbH);
    Rect r;
    r.x0 = w * 0.07f;
    r.x1 = w * 0.93f;
    r.y0 = h * 0.07f;
    r.y1 = h * 0.93f;
    return r;
}

void CityMapToolSystem::ensureSprites(engine::Renderer& r) {
    if (!dot_.valid()) {
        const std::vector<uint8_t> d = makeDot(32);
        dot_ = r.uploadTexture(32, 32, 4, d.data());
    }
    if (!arrow_.valid()) {
        const std::vector<uint8_t> a = makeArrow(64);
        arrow_ = r.uploadTexture(64, 64, 4, a.data());
    }
}

void CityMapToolSystem::startWorker(engine::FrameContext& ctx) {
    if (loader_.joinable()) return;
    std::shared_ptr<const engine::CityMapData> data;
    ctx.world.each<engine::CityMap>([&](engine::Entity, engine::CityMap& m) {
        if (!data && m.data) data = m.data;
    });
    if (!data) return;   // not built yet (or no roads on this level): asked again next frame
    {
        std::lock_guard<std::mutex> lk(mu_);
        quit_ = false;
    }
    loader_ = std::thread([this, data]() {
        // The map IS the city SVG: written once from what the generators built, parsed once, rasterized per tile.
        std::shared_ptr<CityMapRaster> raster = std::make_shared<CityMapRaster>();
        std::error_code ec;
        const std::filesystem::path tmp =
            std::filesystem::temp_directory_path(ec) /
            ("rt_citymap_" + std::to_string(static_cast<long>(::getpid())) + ".svg");
        using Clock = std::chrono::steady_clock;
        auto msSince = [](Clock::time_point t) {
            return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
        };
        const Clock::time_point t0 = Clock::now();
        bool ok = engine::writeCityMapSvg(tmp.string(), *data, engine::CityMapLayers::fromList(kCityMapLayers));
        const double writeMs = msSince(t0);
        const long long bytes = ok ? static_cast<long long>(std::filesystem::file_size(tmp, ec)) : 0;
        const Clock::time_point t1 = Clock::now();
        ok = ok && raster->loadCity(tmp.string());
        std::filesystem::remove(tmp, ec);
        if (!ok) {
            LOG_WARN << "[map] could not build the city map picture";
            return;
        }
        LOG_INFO << "[map] city parsed: wrote the svg " << writeMs << " ms (" << bytes / 1024 << " KB), parsed "
                 << msSince(t1) << " ms; overview level " << raster->overviewLevel() << " ("
                 << CityMapRaster::levelMpp(raster->overviewLevel()) << " m/px)";
        {
            std::lock_guard<std::mutex> lk(mu_);
            raster_ = raster;
        }
        cv_.notify_all();
    });
    // The tile workers: a few, each with its own rasterizer, taking the next wanted tile.
    const unsigned hw = std::max(2u, std::thread::hardware_concurrency());
    const int n = static_cast<int>(std::clamp(hw / 4u, 2u, 6u));
    for (int w = 0; w < n; ++w)
        workers_.emplace_back([this]() {
            CityMapRaster::Rasterizer rast;
            for (;;) {
                TileKey k;
                std::shared_ptr<const CityMapRaster> r;
                {
                    std::unique_lock<std::mutex> lk(mu_);
                    cv_.wait(lk, [&] { return quit_ || (raster_ && !want_.empty()); });
                    if (quit_) return;
                    k = want_.front();
                    want_.erase(want_.begin());
                    drawing_.insert(k);
                    r = raster_;
                }
                std::vector<uint8_t> px;
                const auto tt = std::chrono::steady_clock::now();
                const int shapes = r->rasterizeTile(r->tileView(k.z, k.ix, k.iy), px, rast);
                {   // the first tiles' cost, said once (what a view waits for)
                    static std::atomic<int> count{0};
                    static std::atomic<long long> micros{0}, drawn{0};
                    micros += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - tt).count();
                    drawn += shapes;
                    const int n = ++count;
                    if (n == 40 || n == 200)
                        LOG_INFO << "[map] " << n << " tiles: " << micros.load() / 1000.0 / n << " ms each, "
                                 << drawn.load() / n << " shapes each";
                }
                {
                    std::lock_guard<std::mutex> lk(mu_);
                    drawing_.erase(k);
                    done_.emplace_back(k, std::move(px));
                }
            }
        });
}

std::vector<TransitLine> CityMapToolSystem::transitLines() const {
    std::vector<TransitLine> lines;
    if (!city_.built()) return lines;
    // THE RAIL (Glenn, 2026-10-12: "it would be nice to see the trains on the map"): each line a dark casing with its
    // colour down the middle, every casing first so a double track reads as one line
    if (const RailSim* rs = city_.sim().railRiders().rail()) {
        for (int pass = 0; pass < 2; ++pass)
            for (const RailLine& L : rs->lines()) {
                TransitLine l;
                for (std::size_t i = 0; i < L.pts.size(); i += 2) l.path.push_back({L.pts[i].x, L.pts[i].z});
                if (!L.pts.empty()) l.path.push_back({L.pts.back().x, L.pts.back().z});
                const engine::Vec3 c = pass == 0 ? engine::Vec3(0.08, 0.08, 0.09) : lineColour(L);
                l.r = static_cast<float>(c.x);
                l.g = static_cast<float>(c.y);
                l.b = static_cast<float>(c.z);
                l.width = pass == 0 ? 2.2f : 1.3f;
                l.closed = L.loop;
                lines.push_back(std::move(l));
            }
    }
    const BusNetwork& net = city_.sim().buses();
    for (int r = 0; r < net.routeCount(); ++r) {
        TransitLine l;
        for (const Vec2& p : net.route(r).path) l.path.push_back({p.x, p.y});
        const engine::Vec3 c = routeColour(net.lineOf(r));
        l.r = static_cast<float>(c.x);
        l.g = static_cast<float>(c.y);
        l.b = static_cast<float>(c.z);
        lines.push_back(std::move(l));
    }
    return lines;
}

void CityMapToolSystem::stopWorker() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        quit_ = true;
    }
    cv_.notify_all();
    if (loader_.joinable()) loader_.join();
    for (std::thread& t : workers_)
        if (t.joinable()) t.join();
    workers_.clear();
    std::lock_guard<std::mutex> lk(mu_);
    want_.clear();
    drawing_.clear();
    done_.clear();
}

void CityMapToolSystem::takeTiles(engine::Renderer& r) {
    std::vector<std::pair<TileKey, std::vector<uint8_t>>> got;
    {
        std::lock_guard<std::mutex> lk(mu_);
        const std::size_t n = std::min<std::size_t>(done_.size(), kUploadsPerFrame);
        for (std::size_t k = 0; k < n; ++k) got.push_back(std::move(done_[k]));
        done_.erase(done_.begin(), done_.begin() + static_cast<std::ptrdiff_t>(n));
    }
    for (auto& [key, px] : got) {
        const engine::TextureHandle t =
            r.uploadTexture(CityMapRaster::kTilePx, CityMapRaster::kTilePx, 4, px.data());
        if (!t.valid()) continue;
        Tile& tile = tiles_[key];
        if (tile.tex.valid()) r.removeTexture(tile.tex);
        tile.tex = t;
        tile.used = frame_;
    }
    if (tiles_.size() <= kMaxTiles) return;
    // the cache is full: the least recently wanted go, never the overview (the picture under everything)
    std::vector<std::pair<uint64_t, TileKey>> old;
    for (const auto& [key, tile] : tiles_)
        if (key.z != overviewZ_ && tile.used != frame_) old.emplace_back(tile.used, key);
    std::sort(old.begin(), old.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    for (std::size_t k = 0; k < old.size() && tiles_.size() > kMaxTiles; ++k) {
        auto it = tiles_.find(old[k].second);
        if (it->second.tex.valid()) r.removeTexture(it->second.tex);
        tiles_.erase(it);
    }
}

void CityMapToolSystem::wantTiles(double panelW, double panelH) {
    if (overviewZ_ < 0) return;
    std::vector<std::pair<double, TileKey>> wanted;   // (priority, key): lower first
    std::set<TileKey> busy;
    {
        std::lock_guard<std::mutex> lk(mu_);
        busy = drawing_;
    }
    // every tile of level z over the world rectangle, ranked by `rank` then distance from the centre
    auto cover = [&](int z, double x0, double z0, double x1, double z1, double rank) {
        const double span = CityMapRaster::kTilePx * CityMapRaster::levelMpp(z);
        const int nx = static_cast<int>(std::ceil(cityW_ / span)), ny = static_cast<int>(std::ceil(cityH_ / span));
        const int ix0 = std::max(0, static_cast<int>(std::floor((x0 - cityX_) / span)));
        const int iy0 = std::max(0, static_cast<int>(std::floor((z0 - cityZ_) / span)));
        const int ix1 = std::min(nx - 1, static_cast<int>(std::floor((x1 - cityX_) / span)));
        const int iy1 = std::min(ny - 1, static_cast<int>(std::floor((z1 - cityZ_) / span)));
        for (int iy = iy0; iy <= iy1; ++iy)
            for (int ix = ix0; ix <= ix1; ++ix) {
                const TileKey k{z, ix, iy};
                auto have = tiles_.find(k);
                if (have != tiles_.end()) {
                    have->second.used = frame_;
                    continue;
                }
                if (busy.count(k)) continue;
                const double tx = cityX_ + (ix + 0.5) * span - cx_, tz = cityZ_ + (iy + 0.5) * span - cz_;
                wanted.emplace_back(rank * 1e9 + std::hypot(tx, tz) / span, k);
            }
    };
    // the overview first, always (map open or not: the first look is never blank)
    cover(overviewZ_, cityX_, cityZ_, cityX_ + cityW_, cityZ_ + cityH_, 0);
    if (open_) {
        const int z = std::min(CityMapRaster::levelFor(mpp_), overviewZ_);
        const double hx = panelW * 0.5 * mpp_, hz = panelH * 0.5 * mpp_;
        // a level under the one shown, to show while it comes; then the one shown, a tile's margin round the panel
        if (z + 1 < overviewZ_) cover(z + 1, cx_ - hx, cz_ - hz, cx_ + hx, cz_ + hz, 1);
        const double pad = CityMapRaster::kTilePx * CityMapRaster::levelMpp(z);
        cover(z, cx_ - hx - pad, cz_ - hz - pad, cx_ + hx + pad, cz_ + hz + pad, 2);
    }
    std::sort(wanted.begin(), wanted.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    std::vector<TileKey> list;
    list.reserve(wanted.size());
    for (const auto& w : wanted) list.push_back(w.second);
    {
        std::lock_guard<std::mutex> lk(mu_);
        want_.swap(list);
    }
    cv_.notify_all();
}

void CityMapToolSystem::update(engine::FrameContext& ctx) {
    // Tool changes: 3 raises the map, 1 or 2 puts it away (hands / gun).
    if (ctx.actions.pressed("slot_3")) {
        open_ = !open_;
        if (open_) follow_ = true;
    } else if (open_ && [&] {
                   for (int k = 1; k <= engine::kToolSlots; ++k)
                       if (k != engine::slotOf(engine::Tool::Map) && ctx.actions.pressed(engine::slotAction(k))) return true;
                   return false;
               }()) {
        open_ = false;
    } else if (open_ && ctx.actions.pressed("map_close")) {
        open_ = false;
    }
    fly_.inputSuspended = open_;
    if (open_ != layered_) {   // the map's controls over everything else while it is up
        if (open_) ctx.actions.pushLayer("map");
        else ctx.actions.popLayer("map");
        layered_ = open_;
    }
    // THE TILES, map open or not: the city is parsed and the overview drawn as soon as the level is up, so the
    // first look at the map is the whole city at once, sharpening (ADR-0394).
    ++frame_;
    startWorker(ctx);
    if (overviewZ_ < 0) {
        std::shared_ptr<const CityMapRaster> r;
        {
            std::lock_guard<std::mutex> lk(mu_);
            r = raster_;
        }
        if (r) {
            overviewZ_ = r->overviewLevel();
            cityX_ = r->minX();
            cityZ_ = r->minZ();
            cityW_ = r->width();
            cityH_ = r->height();
        }
    }
    takeTiles(ctx.renderer);
    if (!open_) {
        wantTiles(0, 0);
        return;
    }

    ensureSprites(ctx.renderer);
    // the transit lines, drawn live; again once the trains are there (they start a few frames after the city)
    if (city_.built() && (!linesTaken_ || (!railInLines_ && city_.sim().railRiders().rail()))) {
        takeLines();
        railInLines_ = city_.sim().railRiders().rail() != nullptr;
    }
    const Rect p = panel(ctx.framebufferWidth, ctx.framebufferHeight);
    const double pw = p.x1 - p.x0, ph = p.y1 - p.y0;

    // Where the player is (the controlled body; on a bus, wherever the bus is).
    Vec2 me(cx_, cz_);
    bool haveMe = false;
    ctx.world.each<engine::Transform, engine::ControlledBy>(
        [&](engine::Entity, engine::Transform& t, engine::ControlledBy&) {
            if (haveMe) return;
            me = Vec2(t.position.x, t.position.z);
            haveMe = true;
        });

    if (!sized_) {
        // First sight: the neighbourhood, ~1.2 km across the panel.
        mpp_ = 1200.0 / std::max(1.0, pw);
        sized_ = true;
    }
    if (ctx.actions.pressed("map_centre")) follow_ = true;
    // Drag pans: the map moves with the mouse (the pointer is captured, so
    // these are raw deltas, in window points -> framebuffer pixels).
    const double fbPerPt = ctx.windowWidth > 0
                               ? static_cast<double>(ctx.framebufferWidth) / ctx.windowWidth
                               : 1.0;
    if (ctx.input.mouseLeftDown &&
        (ctx.input.mouseDeltaX != 0.0 || ctx.input.mouseDeltaY != 0.0)) {
        cx_ -= ctx.input.mouseDeltaX * fbPerPt * mpp_;
        cz_ -= ctx.input.mouseDeltaY * fbPerPt * mpp_;
        follow_ = false;
    }
    // The wheel zooms about the panel centre (up = closer); so does the right stick, 4x a second at full tilt.
    const double dt = std::clamp(ctx.frameDelta, 0.0, 0.1);
    const double zoom = ctx.actions.axis("map_zoom");
    if (ctx.input.scrollDelta != 0.0 || zoom != 0.0) {
        mpp_ *= std::pow(0.82, ctx.input.scrollDelta) * std::pow(0.25, zoom * dt);
        mpp_ = std::clamp(mpp_, 0.12, 12.0);
    }
    // The stick (and WASD) pans: at full tilt, half the panel's width a second, at any zoom.
    const double panX = ctx.actions.axis("map_pan_x"), panY = ctx.actions.axis("map_pan_y");
    if (panX != 0.0 || panY != 0.0) {
        cx_ += panX * 0.5 * pw * mpp_ * dt;
        cz_ += panY * 0.5 * pw * mpp_ * dt;
        follow_ = false;
    }
    // THE WAYPOINT at the crosshair (the panel centre): marked, or -- marked there already -- cleared
    if (ctx.actions.pressed("map_mark")) {
        engine::Entity player;
        ctx.world.each<engine::Transform, engine::ControlledBy>(
            [&](engine::Entity e, engine::Transform&, engine::ControlledBy&) { if (!player.valid()) player = e; });
        if (player.valid()) {
            const engine::Waypoint* w = ctx.world.get<engine::Waypoint>(player);
            if (w && std::hypot(w->at.x - cx_, w->at.z - cz_) / mpp_ < 14.0) {
                ctx.world.remove<engine::Waypoint>(player);
            } else {
                const engine::Waypoint mark{engine::Vec3(cx_, 0, cz_)};
                if (w) *ctx.world.get<engine::Waypoint>(player) = mark;
                else ctx.world.add<engine::Waypoint>(player, mark);
            }
            follow_ = false;
        }
    }
    if (follow_ && haveMe) {
        cx_ = me.x;
        cz_ = me.y;
    }
    wantTiles(pw, ph);
}

void CityMapToolSystem::render(engine::FrameContext& ctx) {
    if (!open_) return;
    ctx.renderer.submitUi(composeFrame(ctx.world, ctx.framebufferWidth, ctx.framebufferHeight));
}

std::vector<engine::Renderer::UiQuad> CityMapToolSystem::composeFrame(engine::World& world,
                                                                      int fbW, int fbH) const {
    const Rect p = panel(fbW, fbH);
    const double pcx = (p.x0 + p.x1) * 0.5, pcy = (p.y0 + p.y1) * 0.5;
    std::vector<Quad> q;
    q.reserve(200);

    // Frame and paper.
    q.push_back(tinted(Quad::rect(p.x0 - 8, p.y0 - 8, p.x1 + 8, p.y1 + 8), kFrame));
    q.push_back(tinted(Quad::rect(p.x0, p.y0, p.x1, p.y1), kPaper));

    // World <-> panel pixels for the CURRENT view.
    auto toPx = [&](double wx, double wz) {
        return Vec2(pcx + (wx - cx_) / mpp_, pcy + (wz - cz_) / mpp_);
    };
    auto onPanel = [&](const Vec2& s, float pad) {
        return s.x >= p.x0 - pad && s.x <= p.x1 + pad && s.y >= p.y0 - pad && s.y <= p.y1 + pad;
    };

    // The panel's world rectangle.
    const double vl = cx_ - (pcx - p.x0) * mpp_, vr = cx_ + (p.x1 - pcx) * mpp_;
    const double vt = cz_ - (pcy - p.y0) * mpp_, vb = cz_ + (p.y1 - pcy) * mpp_;

    // THE PICTURE: the tiles, coarsest first, each clipped to the panel -- a sharper tile covers the coarser one
    // under it, and where it has not come yet the coarser one shows (ADR-0394).
    if (overviewZ_ >= 0) {
        const int zShow = std::min(CityMapRaster::levelFor(mpp_), overviewZ_);
        for (int z = overviewZ_; z >= zShow; --z) {
            const double span = CityMapRaster::kTilePx * CityMapRaster::levelMpp(z);
            for (auto it = tiles_.lower_bound(TileKey{z, -1 << 30, -1 << 30});
                 it != tiles_.end() && it->first.z == z; ++it) {
                const double tl = cityX_ + it->first.ix * span, tt = cityZ_ + it->first.iy * span;
                const double tr = tl + span, tb = tt + span;
                const double l = std::max(tl, vl), r = std::min(tr, vr);
                const double t = std::max(tt, vt), b = std::min(tb, vb);
                if (r <= l || b <= t || !it->second.tex.valid()) continue;
                const Vec2 a = toPx(l, t), c = toPx(r, b);
                Quad m = Quad::rect(static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(c.x),
                                    static_cast<float>(c.y));
                m.texture = it->second.tex;
                const float u0 = static_cast<float>((l - tl) / span), u1 = static_cast<float>((r - tl) / span);
                const float v0 = static_cast<float>((t - tt) / span), v1 = static_cast<float>((b - tt) / span);
                m.u[0] = u0; m.u[1] = u1; m.u[2] = u1; m.u[3] = u0;
                m.v[0] = v0; m.v[1] = v0; m.v[2] = v1; m.v[3] = v1;
                q.push_back(m);
            }
        }
    }

    // THE TRANSIT LINES, live over the picture: a few pixels wide at any zoom, clipped to the panel.
    for (const TransitLine& line : lines_) {
        const float half = kLinePx * line.width * 0.5f;
        const float col[4] = {line.r, line.g, line.b, 0.9f};
        const std::size_t n = line.path.size();
        if (n < 2) continue;
        Vec2 prev = toPx(line.path[0].first, line.path[0].second);
        const std::size_t segs = line.closed ? n : n - 1;
        for (std::size_t k = 1; k <= segs; ++k) {
            const auto& w = line.path[k % n];
            const Vec2 cur = toPx(w.first, w.second);
            // a segment under a pixel and a half joins the next (a whole-island view has thousands)
            if (k < segs && std::hypot(cur.x - prev.x, cur.y - prev.y) < 1.5) continue;
            Vec2 a = prev, b = cur;
            prev = cur;
            // clip to the panel (Liang-Barsky), inset by the half width so a line never paints the frame
            double t0 = 0, t1 = 1;
            const double dx = b.x - a.x, dy = b.y - a.y;
            const double edges[4][2] = {{-dx, a.x - (p.x0 + half)}, {dx, (p.x1 - half) - a.x},
                                        {-dy, a.y - (p.y0 + half)}, {dy, (p.y1 - half) - a.y}};
            bool out = false;
            for (const auto& e : edges) {
                if (e[0] == 0) {
                    if (e[1] < 0) out = true;
                    continue;
                }
                const double t = e[1] / e[0];
                if (e[0] < 0) t0 = std::max(t0, t);
                else t1 = std::min(t1, t);
            }
            if (out || t0 >= t1) continue;
            b = Vec2(a.x + dx * t1, a.y + dy * t1);
            a = Vec2(a.x + dx * t0, a.y + dy * t0);
            const double len = std::hypot(b.x - a.x, b.y - a.y);
            if (len < 1e-6) continue;
            // along by the half width each end, so the corners between segments close
            const double ux = (b.x - a.x) / len * half, uy = (b.y - a.y) / len * half;
            Quad seg;
            seg.x[0] = static_cast<float>(a.x - ux + uy); seg.y[0] = static_cast<float>(a.y - uy - ux);
            seg.x[1] = static_cast<float>(b.x + ux + uy); seg.y[1] = static_cast<float>(b.y + uy - ux);
            seg.x[2] = static_cast<float>(b.x + ux - uy); seg.y[2] = static_cast<float>(b.y + uy + ux);
            seg.x[3] = static_cast<float>(a.x - ux - uy); seg.y[3] = static_cast<float>(a.y - uy + ux);
            q.push_back(tinted(seg, col));
        }
    }

    auto dot = [&](const Vec2& s, float radius, const float c[4]) {
        Quad d = Quad::rect(static_cast<float>(s.x) - radius, static_cast<float>(s.y) - radius,
                            static_cast<float>(s.x) + radius, static_cast<float>(s.y) + radius);
        d.texture = dot_;
        q.push_back(tinted(d, c));
    };
    const float ink[4] = {0.08f, 0.08f, 0.09f, 1.0f};
    const float white[4] = {1.0f, 1.0f, 1.0f, 1.0f};

    if (city_.built()) {
        const BusNetwork& net = city_.sim().buses();
        // STOPS: a dot in the route's colour; an interchange (a hub stop,
        // where routes meet) bigger and ringed.
        for (int r = 0; r < net.routeCount(); ++r) {
            const engine::Vec3 rc = routeColour(net.lineOf(r));
            const float col[4] = {static_cast<float>(rc.x), static_cast<float>(rc.y),
                                  static_cast<float>(rc.z), 1.0f};
            const BusRoute& route = net.route(r);
            for (std::size_t si = 0; si < route.stops.size(); ++si) {
                const Vec2 s = toPx(route.stops[si].pos.x, route.stops[si].pos.y);
                if (!onPanel(s, -4.0f)) continue;
                const bool hub = std::find(route.hubStops.begin(), route.hubStops.end(),
                                           static_cast<int>(si)) != route.hubStops.end();
                dot(s, hub ? 9.0f : 7.0f, ink);
                dot(s, hub ? 6.5f : 4.5f, hub ? white : col);
                if (hub) dot(s, 3.5f, col);
            }
        }
        // THE BUSES, live: where each one is right now.
        const auto& agents = city_.sim().agents();
        for (int i = 0; i < static_cast<int>(agents.size()); ++i) {
            const int r = city_.sim().busRouteOf(i);
            if (r < 0) continue;
            const Vec2 s = toPx(agents[static_cast<std::size_t>(i)].pos.x,
                                agents[static_cast<std::size_t>(i)].pos.y);
            if (!onPanel(s, -6.0f)) continue;
            const engine::Vec3 rc = routeColour(city_.sim().buses().lineOf(r));
            const float col[4] = {static_cast<float>(rc.x), static_cast<float>(rc.y),
                                  static_cast<float>(rc.z), 1.0f};
            Quad back = Quad::rect(static_cast<float>(s.x) - 8, static_cast<float>(s.y) - 8,
                                   static_cast<float>(s.x) + 8, static_cast<float>(s.y) + 8);
            q.push_back(tinted(back, ink));
            Quad front = Quad::rect(static_cast<float>(s.x) - 5.5f, static_cast<float>(s.y) - 5.5f,
                                    static_cast<float>(s.x) + 5.5f, static_cast<float>(s.y) + 5.5f);
            q.push_back(tinted(front, col));
        }
    }

    // THE RAIL, live: each station a ringed square, each train its cars, a dot apiece in its line's colour
    if (city_.built())
        if (const RailSim* rs = city_.sim().railRiders().rail()) {
            std::vector<std::pair<std::string, Vec2>> stations;
            for (const RailLine& L : rs->lines())
                for (const engine::RailStop& st : L.stops) {
                    const engine::Vec3 c = L.at(0.5 * (st.s0 + st.s1));
                    bool seen = false;
                    for (auto& [name, at] : stations)
                        if (name == st.name) { at = (at + Vec2(c.x, c.z)) * 0.5; seen = true; }   // the two platforms' middle
                    if (!seen) stations.push_back({st.name, Vec2(c.x, c.z)});
                }
            for (const auto& [name, at] : stations) {
                const Vec2 s = toPx(at.x, at.y);
                if (!onPanel(s, -6.0f)) continue;
                q.push_back(tinted(Quad::rect(static_cast<float>(s.x) - 9, static_cast<float>(s.y) - 9, static_cast<float>(s.x) + 9, static_cast<float>(s.y) + 9), ink));
                q.push_back(tinted(Quad::rect(static_cast<float>(s.x) - 6.5f, static_cast<float>(s.y) - 6.5f, static_cast<float>(s.x) + 6.5f, static_cast<float>(s.y) + 6.5f), white));
                q.push_back(tinted(Quad::rect(static_cast<float>(s.x) - 2.5f, static_cast<float>(s.y) - 2.5f, static_cast<float>(s.x) + 2.5f, static_cast<float>(s.y) + 2.5f), ink));
            }
            for (int ti = 0; ti < static_cast<int>(rs->trains().size()); ++ti) {
                const Train& T = rs->trains()[static_cast<std::size_t>(ti)];
                const engine::Vec3 lc = lineColour(rs->lines()[static_cast<std::size_t>(T.line)]);
                const float col[4] = {static_cast<float>(lc.x), static_cast<float>(lc.y), static_cast<float>(lc.z), 1.0f};
                const std::vector<CarPose> cars = rs->carPoses(ti, 1.0);
                for (int pass = 0; pass < 2; ++pass)
                    for (const CarPose& cp : cars) {
                        const engine::Vec3 at = cp.m.transformPoint(engine::Vec3(0, 0, 0));
                        const Vec2 s = toPx(at.x, at.z);
                        if (!onPanel(s, -8.0f)) continue;
                        // a car's length on the map, but never under a few pixels
                        const float r = std::max(pass == 0 ? 6.5f : 4.5f, static_cast<float>(cp.length * 0.5 / mpp_) + (pass == 0 ? 2.0f : 0.0f));
                        dot(s, std::min(r, pass == 0 ? 16.0f : 14.0f), pass == 0 ? ink : (cp.loco ? white : col));
                    }
            }
        }

    // THE WAYPOINT, pinned to the panel's edge when it is off it; and the crosshair it is marked at
    {
        const engine::Waypoint* mark = nullptr;
        world.each<engine::Transform, engine::ControlledBy>(
            [&](engine::Entity e, engine::Transform&, engine::ControlledBy&) {
                if (!mark) mark = world.get<engine::Waypoint>(e);
            });
        if (mark) {
            Vec2 s = toPx(mark->at.x, mark->at.z);
            s.x = std::clamp(s.x, static_cast<Real>(p.x0 + 14), static_cast<Real>(p.x1 - 14));
            s.y = std::clamp(s.y, static_cast<Real>(p.y0 + 14), static_cast<Real>(p.y1 - 14));
            dot(s, 13.0f, ink);
            dot(s, 10.0f, kMark);
            dot(s, 3.5f, ink);
        }
        const float hair[4] = {0.08f, 0.08f, 0.09f, 0.55f};
        const float cxp = static_cast<float>(pcx), cyp = static_cast<float>(pcy);
        for (const float side : {-1.0f, 1.0f}) {   // four ticks round an open centre
            q.push_back(tinted(Quad::rect(cxp + side * 6.0f - (side < 0 ? 12.0f : 0.0f), cyp - 1.0f,
                                          cxp + side * 6.0f + (side > 0 ? 12.0f : 0.0f), cyp + 1.0f), hair));
            q.push_back(tinted(Quad::rect(cxp - 1.0f, cyp + side * 6.0f - (side < 0 ? 12.0f : 0.0f),
                                          cxp + 1.0f, cyp + side * 6.0f + (side > 0 ? 12.0f : 0.0f)), hair));
        }
    }

    // YOU: the chevron at your position, pointing where you look.
    {
        Vec2 me(cx_, cz_);
        bool haveMe = false;
        world.each<engine::Transform, engine::ControlledBy>(
            [&](engine::Entity, engine::Transform& t, engine::ControlledBy&) {
                if (haveMe) return;
                me = Vec2(t.position.x, t.position.z);
                haveMe = true;
            });
        if (haveMe) {
            Vec2 s = toPx(me.x, me.y);
            // Off the panel: pinned to its edge, still pointing, so you can
            // find your way back.
            s.x = std::clamp(s.x, static_cast<Real>(p.x0 + 14), static_cast<Real>(p.x1 - 14));
            s.y = std::clamp(s.y, static_cast<Real>(p.y0 + 14), static_cast<Real>(p.y1 - 14));
            const Real yaw = fly_.yaw * 3.14159265358979 / 180.0;
            const Vec2 f(std::sin(yaw), -std::cos(yaw));   // world XZ forward = screen (x, y)
            const Vec2 rt(-f.y, f.x);
            auto arrow = [&](float size, const float c[4]) {
                const Real h = size * 0.5;
                Quad a;
                a.texture = arrow_;
                const Vec2 tl = s + (f - rt) * h, tr = s + (f + rt) * h;
                const Vec2 br = s + (rt - f) * h, bl = s - (f + rt) * h;
                a.x[0] = static_cast<float>(tl.x); a.y[0] = static_cast<float>(tl.y);
                a.x[1] = static_cast<float>(tr.x); a.y[1] = static_cast<float>(tr.y);
                a.x[2] = static_cast<float>(br.x); a.y[2] = static_cast<float>(br.y);
                a.x[3] = static_cast<float>(bl.x); a.y[3] = static_cast<float>(bl.y);
                q.push_back(tinted(a, c));
            };
            arrow(40.0f, white);
            arrow(30.0f, kYou);
        }
    }

    // THE LEGEND: each LINE's colour, in route order (the HUD names them). Twins come after every loop, so the
    // lines are the first routes.
    if (city_.built()) {
        int n = 0;
        while (n < city_.sim().buses().routeCount() && city_.sim().buses().lineOf(n) == n) ++n;
        const float x0 = p.x0 + 16, y0 = p.y0 + 16;
        if (n > 0)
            q.push_back(tinted(Quad::rect(x0 - 8, y0 - 8, x0 + 48, y0 + n * 22.0f + 2), kFrame));
        for (int r = 0; r < n; ++r) {
            const engine::Vec3 rc = routeColour(r);
            const float col[4] = {static_cast<float>(rc.x), static_cast<float>(rc.y),
                                  static_cast<float>(rc.z), 1.0f};
            const float y = y0 + r * 22.0f;
            q.push_back(tinted(Quad::rect(x0, y + 4, x0 + 40, y + 12), col));
        }
    }
    return q;
}

}  // namespace citysim
