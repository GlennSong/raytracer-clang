#include "city_map_tool.h"

#include "../../engine/camera/fly_camera_controller.h"
#include "../../engine/components.h"
#include "../../log.h"
#include "../../engine/procgen/city/city_svg.h"
#include "../../engine/world.h"
#include "bus_stop_props.h"   // routeColour: the map wears the stop signs' colours
#include "city_render.h"
#include "rail_sim.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <unistd.h>

namespace citysim {

using engine::Real;
using engine::Vec2;
using Quad = engine::Renderer::UiQuad;

namespace {

// Rasterize a margin round the panel so a short pan still has picture.
constexpr double kMargin = 1.5;

constexpr float kFrame[4] = {0.10f, 0.10f, 0.11f, 0.92f};
constexpr float kPaper[4] = {0.84f, 0.82f, 0.77f, 1.0f};
constexpr float kYou[4] = {0.90f, 0.20f, 0.16f, 1.0f};

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
    ctx.actions.bindButton("map_centre", engine::KeyCode::C);
}

void CityMapToolSystem::onStop(engine::FrameContext& ctx) {
    stopWorker();
    fly_.inputSuspended = false;
    if (tex_.valid()) ctx.renderer.removeTexture(tex_);
    if (dot_.valid()) ctx.renderer.removeTexture(dot_);
    if (arrow_.valid()) ctx.renderer.removeTexture(arrow_);
    tex_ = dot_ = arrow_ = engine::TextureHandle{};
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
    if (worker_.joinable()) return;
    std::shared_ptr<const engine::CityMapData> data;
    ctx.world.each<engine::CityMap>([&](engine::Entity, engine::CityMap& m) {
        if (!data && m.data) data = m.data;
    });
    if (!data) {
        LOG_WARN << "[map] this level has no city map (no roads?)";
        return;
    }
    // The lines, copied on the main thread: the worker never touches the sim.
    std::vector<TransitLine> lines = transitLines();
    railInOverlay_ = city_.built() && city_.sim().railRiders().rail() != nullptr;
    {
        std::lock_guard<std::mutex> lk(mu_);
        quit_ = false;
        loading_ = true;
    }
    worker_ = std::thread([this, data, lines = std::move(lines)]() {
        CityMapRaster raster;
        // The map IS the city SVG: written once from what the generators
        // built, parsed once, rasterized per view.
        std::error_code ec;
        const std::filesystem::path tmp =
            std::filesystem::temp_directory_path(ec) /
            ("rt_citymap_" + std::to_string(static_cast<long>(::getpid())) + ".svg");
        bool ok = engine::writeCityMapSvg(tmp.string(), *data,
                                          engine::CityMapLayers::fromList(kCityMapLayers)) &&
                  raster.loadCity(tmp.string());
        std::filesystem::remove(tmp, ec);
        if (ok) raster.loadTransit(transitSvg(lines, raster.minX(), raster.minZ(),
                                              raster.width(), raster.height()));
        {
            std::lock_guard<std::mutex> lk(mu_);
            loading_ = false;
            if (!ok) quit_ = true;
        }
        if (!ok) {
            LOG_WARN << "[map] could not build the city map picture";
            return;
        }
        for (;;) {
            CityMapRaster::View v;
            {
                std::unique_lock<std::mutex> lk(mu_);
                cv_.wait(lk, [&] { return quit_ || haveJob_; });
                if (quit_) return;
                if (haveLines_) {   // the overlay again (the trains arrived after the map first opened)
                    raster.loadTransit(transitSvg(newLines_, raster.minX(), raster.minZ(), raster.width(), raster.height()));
                    newLines_.clear();
                    haveLines_ = false;
                }
                v = job_;
                haveJob_ = false;
                rasterBusy_ = true;
            }
            std::vector<uint8_t> px;
            raster.rasterize(v, px, 4.0f);
            {
                std::lock_guard<std::mutex> lk(mu_);
                resultView_ = v;
                result_.swap(px);
                haveResult_ = true;
                rasterBusy_ = false;
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
    if (worker_.joinable()) worker_.join();
}

void CityMapToolSystem::requestRaster(const CityMapRaster::View& v) {
    {
        std::lock_guard<std::mutex> lk(mu_);
        job_ = v;
        haveJob_ = true;
    }
    cv_.notify_one();
    lastAsked_ = v;
    askedAny_ = true;
}

void CityMapToolSystem::update(engine::FrameContext& ctx) {
    // Tool changes: 3 raises the map, 1 or 2 puts it away (hands / gun).
    if (ctx.actions.pressed("slot_3")) {
        open_ = !open_;
        if (open_) follow_ = true;
    } else if (open_ && (ctx.actions.pressed("slot_1") || ctx.actions.pressed("slot_2") || ctx.actions.pressed("slot_4") ||
                         ctx.actions.pressed("slot_5") || ctx.actions.pressed("slot_6"))) {
        open_ = false;
    }
    fly_.inputSuspended = open_;
    if (!open_) return;

    startWorker(ctx);
    ensureSprites(ctx.renderer);
    // the trains came after the picture's overlay was drawn: draw it again with the rail
    bool railNow = false;
    if (worker_.joinable() && !railInOverlay_ && city_.built() && city_.sim().railRiders().rail()) {
        std::lock_guard<std::mutex> lk(mu_);
        newLines_ = transitLines();
        haveLines_ = true;
        railInOverlay_ = railNow = true;
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

    const double oldCx = cx_, oldCz = cz_, oldMpp = mpp_;
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
    // The wheel zooms about the panel centre (up = closer).
    if (ctx.input.scrollDelta != 0.0) {
        mpp_ *= std::pow(0.82, ctx.input.scrollDelta);
        mpp_ = std::clamp(mpp_, 0.12, 12.0);
    }
    if (follow_ && haveMe) {
        cx_ = me.x;
        cz_ = me.y;
    }
    const bool changed = cx_ != oldCx || cz_ != oldCz || mpp_ != oldMpp;
    settle_ = changed ? 0.0 : settle_ + ctx.frameDelta;

    // Collect a finished picture.
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (haveResult_) {
            const engine::TextureHandle t = ctx.renderer.uploadTexture(
                resultView_.w, resultView_.h, 4, result_.data());
            if (t.valid()) {
                if (tex_.valid()) ctx.renderer.removeTexture(tex_);
                tex_ = t;
                texView_ = resultView_;
            }
            haveResult_ = false;
        }
    }

    // Ask for a new one when the picture no longer covers the view well: the
    // zoom has drifted, or a pan has eaten into the margin. Following a walk
    // or finishing a drag counts; a drag in progress does not (the old picture
    // slides with it).
    bool busy, loading;
    {
        std::lock_guard<std::mutex> lk(mu_);
        busy = rasterBusy_ || haveJob_;
        loading = loading_;
    }
    if (!loading && !busy && !(ctx.input.mouseLeftDown && !follow_)) {
        const CityMapRaster::View& have = tex_.valid() ? texView_ : lastAsked_;
        const bool none = !tex_.valid() && !askedAny_;
        const double zoomDrift = std::fabs(have.metresPerPixel / mpp_ - 1.0);
        const double panPx = std::hypot(have.cx - cx_, have.cz - cz_) / mpp_;
        const bool stale = none || zoomDrift > 0.03 || panPx > 0.2 * std::min(pw, ph);
        const bool alreadyAsked = askedAny_ &&
            std::fabs(lastAsked_.metresPerPixel / mpp_ - 1.0) < 1e-6 &&
            std::hypot(lastAsked_.cx - cx_, lastAsked_.cz - cz_) / mpp_ < 0.5;
        if ((stale && !alreadyAsked && (settle_ > 0.12 || none || follow_)) || railNow) {
            CityMapRaster::View v;
            v.cx = cx_;
            v.cz = cz_;
            v.metresPerPixel = mpp_;
            v.w = static_cast<int>(pw * kMargin);
            v.h = static_cast<int>(ph * kMargin);
            requestRaster(v);
        }
    }
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

    // The picture, placed by the view it was drawn for and clipped to the
    // panel: while a new one is on its way the old one slides and scales.
    if (tex_.valid() && texView_.w > 0 && texView_.h > 0) {
        const double tl = texView_.cx - texView_.w * 0.5 * texView_.metresPerPixel;
        const double tt = texView_.cz - texView_.h * 0.5 * texView_.metresPerPixel;
        const double tr = tl + texView_.w * texView_.metresPerPixel;
        const double tb = tt + texView_.h * texView_.metresPerPixel;
        // The panel's world rectangle.
        const double vl = cx_ - (pcx - p.x0) * mpp_, vr = cx_ + (p.x1 - pcx) * mpp_;
        const double vt = cz_ - (pcy - p.y0) * mpp_, vb = cz_ + (p.y1 - pcy) * mpp_;
        const double l = std::max(tl, vl), r = std::min(tr, vr);
        const double t = std::max(tt, vt), b = std::min(tb, vb);
        if (r > l && b > t) {
            const Vec2 a = toPx(l, t), c = toPx(r, b);
            Quad m = Quad::rect(static_cast<float>(a.x), static_cast<float>(a.y),
                                static_cast<float>(c.x), static_cast<float>(c.y));
            m.texture = tex_;
            const float u0 = static_cast<float>((l - tl) / (tr - tl));
            const float u1 = static_cast<float>((r - tl) / (tr - tl));
            const float v0 = static_cast<float>((t - tt) / (tb - tt));
            const float v1 = static_cast<float>((b - tt) / (tb - tt));
            m.u[0] = u0; m.u[1] = u1; m.u[2] = u1; m.u[3] = u0;
            m.v[0] = v0; m.v[1] = v0; m.v[2] = v1; m.v[3] = v1;
            q.push_back(m);
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
