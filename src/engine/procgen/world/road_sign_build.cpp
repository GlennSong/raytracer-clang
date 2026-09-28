#include "road_sign_build.h"

#include "../../mesh_builder.h"
#include "../../../log.h"

#include <tinygltf/stb_image.h>
#include <tinygltf/stb_image_write.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <tuple>

namespace engine {

namespace {

void parseHex(const std::string& hex, uint8_t out[4]) {
    unsigned v = 0xffffff;
    if (hex.size() == 7 && hex[0] == '#') v = static_cast<unsigned>(std::stoul(hex.substr(1), nullptr, 16));
    out[0] = static_cast<uint8_t>((v >> 16) & 255); out[1] = static_cast<uint8_t>((v >> 8) & 255); out[2] = static_cast<uint8_t>(v & 255); out[3] = 255;
}

// Blend `c` over the image wherever `inside(x, y)` holds (pixel units), 3x3 supersampled.
void fillShape(TextImage& img, double x0, double y0, double x1, double y1, const uint8_t c[4],
               const std::function<bool(double, double)>& inside) {
    const int i0 = std::max(0, static_cast<int>(std::floor(x0))), i1 = std::min(img.w - 1, static_cast<int>(std::ceil(x1)));
    const int j0 = std::max(0, static_cast<int>(std::floor(y0))), j1 = std::min(img.h - 1, static_cast<int>(std::ceil(y1)));
    for (int j = j0; j <= j1; ++j)
        for (int i = i0; i <= i1; ++i) {
            int hits = 0;
            for (int sy = 0; sy < 3; ++sy)
                for (int sx = 0; sx < 3; ++sx) hits += inside(i + (sx + 0.5) / 3.0, j + (sy + 0.5) / 3.0) ? 1 : 0;
            if (!hits) continue;
            const unsigned a = static_cast<unsigned>(hits * 255 / 9) * c[3] / 255u;
            uint8_t* d = &img.rgba[(static_cast<std::size_t>(j) * img.w + i) * 4];
            for (int k = 0; k < 3; ++k) d[k] = static_cast<uint8_t>((c[k] * a + d[k] * (255u - a)) / 255u);
            d[3] = static_cast<uint8_t>(std::min(255u, d[3] + a * (255u - d[3]) / 255u));
        }
}
bool inRoundRect(double x, double y, double X, double Y, double W, double H, double r) {
    if (x < X || y < Y || x > X + W || y > Y + H) return false;
    const double cx = std::clamp(x, X + r, X + W - r), cy = std::clamp(y, Y + r, Y + H - r);
    return (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r;
}
bool inPolygon(const std::vector<std::pair<double, double>>& P, double x, double y) {
    bool in = false;
    for (std::size_t i = 0, j = P.size() - 1; i < P.size(); j = i++)
        if ((P[i].second > y) != (P[j].second > y) &&
            x < (P[j].first - P[i].first) * (y - P[i].second) / (P[j].second - P[i].second) + P[i].first) in = !in;
    return in;
}

}  // namespace

TextImage rasterizeSignFace(const SignFace& f, const Font& font, double k, double* tabAbove) {
    // the canvas covers the panel and any exit tab above it
    double top = 0;
    for (const SignElem& e : f.elems) if (e.kind == "tab") top = std::min(top, e.y);
    if (tabAbove) *tabAbove = -top;
    const double oy = -top;   // metres from the canvas top to the panel's
    TextImage img;
    const uint8_t clear[4] = {0, 0, 0, 0};
    img.resize(std::max(4, static_cast<int>(std::ceil(f.w * k)) + 2), std::max(4, static_cast<int>(std::ceil((f.h + oy) * k)) + 2), clear);
    auto X = [&](double m) { return 1.0 + m * k; };
    auto Y = [&](double m) { return 1.0 + (m + oy) * k; };
    const float capRatio = font.capHeight(100.0f) / 100.0f;
    auto text = [&](const std::string& t, double x, double baseline, double cap, const uint8_t c[4], const std::string& anchor) {
        const float P = static_cast<float>(cap * k / capRatio);
        const double w = font.measure(t, P);
        const double x0 = anchor == "middle" ? X(x) - w / 2 : anchor == "end" ? X(x) - w : X(x);
        font.draw(img, t, static_cast<float>(x0), static_cast<float>(Y(baseline)), P, c);
    };
    uint8_t bg[4], white[4], green[4], black[4];
    parseHex(f.background, bg);
    parseHex("#ffffff", white);
    parseHex("#00693f", green);
    parseHex("#111111", black);
    // the exit tab first: the panel overlaps its foot
    for (const SignElem& e : f.elems)
        if (e.kind == "tab") {
            fillShape(img, X(e.x), Y(e.y), X(e.x + e.w), Y(e.y + e.h + 0.08), white,
                      [&](double x, double y) { return inRoundRect(x, y, X(e.x), Y(e.y), e.w * k, (e.h + 0.08) * k, 0.1 * k); });
            fillShape(img, X(e.x), Y(e.y), X(e.x + e.w), Y(e.y + e.h + 0.08), green,
                      [&](double x, double y) { return inRoundRect(x, y, X(e.x + 0.04), Y(e.y + 0.04), (e.w - 0.08) * k, (e.h) * k, 0.08 * k); });
            text(e.text, e.x + e.w / 2, e.y + e.h * 0.5 + e.cap / 2, e.cap, white, "middle");
        }
    fillShape(img, X(0), Y(0), X(f.w), Y(f.h), bg, [&](double x, double y) { return inRoundRect(x, y, X(0), Y(0), f.w * k, f.h * k, 0.12 * k); });
    for (const SignElem& e : f.elems) {
        uint8_t c[4];
        parseHex(e.color, c);
        if (e.kind == "rect" && e.anchor == "stroke") {
            // a border: the ring between the rect and the rect inset by its width
            const double sw = e.cap;
            fillShape(img, X(e.x - sw / 2), Y(e.y - sw / 2), X(e.x + e.w + sw / 2), Y(e.y + e.h + sw / 2), c, [&](double x, double y) {
                return inRoundRect(x, y, X(e.x - sw / 2), Y(e.y - sw / 2), (e.w + sw) * k, (e.h + sw) * k, (e.radius + sw / 2) * k) &&
                       !inRoundRect(x, y, X(e.x + sw / 2), Y(e.y + sw / 2), (e.w - sw) * k, (e.h - sw) * k, std::max(0.0, e.radius - sw / 2) * k);
            });
        } else if (e.kind == "rect") {
            fillShape(img, X(e.x), Y(e.y), X(e.x + e.w), Y(e.y + e.h), c, [&](double x, double y) { return x >= X(e.x) && x <= X(e.x + e.w) && y >= Y(e.y) && y <= Y(e.y + e.h); });
        } else if (e.kind == "disc") {
            const double cx = X(e.x + e.w / 2), cy = Y(e.y + e.h / 2), r = e.w / 2 * k;
            fillShape(img, cx - r, cy - r, cx + r, cy + r, c, [&](double x, double y) { return (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r; });
        } else if (e.kind == "text") {
            text(e.text, e.x, e.y, e.cap, c, e.anchor);
        } else if (e.kind == "shield") {
            // white, a flat top and a pointed foot (the SVG's quadratic curves, sampled), a black rim
            std::vector<std::pair<double, double>> P;
            const double sx = e.x, sy = e.y, W = e.w, H = e.h;
            P.push_back({X(sx), Y(sy)});
            P.push_back({X(sx + W), Y(sy)});
            P.push_back({X(sx + W), Y(sy + H * 0.62)});
            for (int q = 1; q <= 8; ++q) {   // Q (W, .9H) -> (W/2, H)
                const double t = q / 8.0, a = (1 - t) * (1 - t), b = 2 * (1 - t) * t, cc = t * t;
                P.push_back({X(sx + a * W + b * W + cc * W / 2), Y(sy + a * H * 0.62 + b * H * 0.9 + cc * H)});
            }
            for (int q = 1; q <= 8; ++q) {   // Q (0, .9H) -> (0, .62H)
                const double t = q / 8.0, a = (1 - t) * (1 - t), b = 2 * (1 - t) * t, cc = t * t;
                P.push_back({X(sx + a * W / 2 + b * 0 + cc * 0), Y(sy + a * H + b * H * 0.9 + cc * H * 0.62)});
            }
            double bx0 = 1e9, by0 = 1e9, bx1 = -1e9, by1 = -1e9;
            for (const auto& [px, py] : P) { bx0 = std::min(bx0, px); by0 = std::min(by0, py); bx1 = std::max(bx1, px); by1 = std::max(by1, py); }
            fillShape(img, bx0 - 2, by0 - 2, bx1 + 2, by1 + 2, black, [&](double x, double y) {
                for (double dx : {-0.02 * k, 0.02 * k}) for (double dy : {-0.02 * k, 0.02 * k}) if (inPolygon(P, x + dx, y + dy)) return true;
                return false;
            });
            fillShape(img, bx0, by0, bx1, by1, white, [&](double x, double y) { return inPolygon(P, x, y); });
            text(e.text, sx + W / 2, sy + H * 0.62, H * 0.42, black, "middle");
        } else if (e.kind == "arrow") {
            // an up arrow in its box, rotated about the box's centre
            const double cx = e.x + e.w / 2, cy = e.y + e.h / 2, W = e.w, H = e.h;
            const std::vector<std::pair<double, double>> local = {
                {0, -H / 2}, {W / 2, -H * 0.05}, {W * 0.16, -H * 0.05}, {W * 0.16, H / 2}, {-W * 0.16, H / 2}, {-W * 0.16, -H * 0.05}, {-W / 2, -H * 0.05}};
            const double a = e.angle * 3.14159265358979 / 180.0, ca = std::cos(a), sa = std::sin(a);
            std::vector<std::pair<double, double>> P;
            for (const auto& [lx, ly] : local) P.push_back({X(cx + lx * ca - ly * sa), Y(cy + lx * sa + ly * ca)});
            const double r = std::max(W, H);
            fillShape(img, X(cx - r), Y(cy - r), X(cx + r), Y(cy + r), c, [&](double x, double y) { return inPolygon(P, x, y); });
        }
    }
    return img;
}

nlohmann::json roadSignsToJson(const std::vector<IslandSign>& signs) {
    nlohmann::json a = nlohmann::json::array();
    for (const IslandSign& s : signs)
        a.push_back({{"kind", s.kind}, {"at", {s.at.x, s.at.y}}, {"facing", {s.facing.x, s.facing.y}}, {"mount", s.mount}, {"legend", s.legend}});
    return a;
}

std::vector<IslandSign> roadSignsFromJson(const nlohmann::json& j) {
    std::vector<IslandSign> out;
    for (const nlohmann::json& s : j) {
        IslandSign sg;
        sg.kind = s.value("kind", std::string());
        sg.at = Vec2(s["at"][0].get<double>(), s["at"][1].get<double>());
        sg.facing = Vec2(s["facing"][0].get<double>(), s["facing"][1].get<double>());
        sg.mount = s.value("mount", std::string("roadside"));
        sg.legend = s.value("legend", nlohmann::json::object());
        out.push_back(sg);
    }
    return out;
}

RoadSignAtlas bakeRoadSignAtlas(const std::vector<IslandSign>& signs, const Font& font, const std::string& cacheDir,
                                double k, int pageSize) {
    RoadSignAtlas at;
    // the key: every face's content, the resolution, and the layout's version (bump it when layoutSign changes)
    {
        nlohmann::json faces = nlohmann::json::array();
        for (const IslandSign& s : signs) faces.push_back({s.kind, s.legend});
        const std::string blob = faces.dump() + "|k=" + std::to_string(k) + "|page=" + std::to_string(pageSize) + "|layout=2";
        uint64_t h = 1469598103934665603ull;
        for (unsigned char c : blob) { h ^= c; h *= 1099511628211ull; }
        char buf[24];
        std::snprintf(buf, sizeof buf, "%016llx", static_cast<unsigned long long>(h));
        at.key = buf;
    }
    const std::string meta = cacheDir.empty() ? std::string() : cacheDir + "/roadsigns_" + at.key + ".json";
    if (!meta.empty()) {
        std::ifstream in(meta);
        if (in) {
            try {
                nlohmann::json m;
                in >> m;
                for (const nlohmann::json& s : m["slots"])
                    at.slots.push_back({s["page"].get<int>(), s["uv"][0].get<float>(), s["uv"][1].get<float>(), s["uv"][2].get<float>(),
                                        s["uv"][3].get<float>(), s["w"].get<double>(), s["h"].get<double>(), s["tab"].get<double>(), s.value("tabW", 0.0)});
                bool ok = at.slots.size() == signs.size();
                for (int p = 0; ok && p < m["pages"].get<int>(); ++p) {
                    int w = 0, hh = 0, n = 0;
                    const std::string png = cacheDir + "/roadsigns_" + at.key + "_" + std::to_string(p) + ".png";
                    unsigned char* data = stbi_load(png.c_str(), &w, &hh, &n, 4);
                    if (!data) { ok = false; break; }
                    TextImage img;
                    img.w = w; img.h = hh;
                    img.rgba.assign(data, data + static_cast<std::size_t>(w) * hh * 4);
                    stbi_image_free(data);
                    at.pages.push_back(std::move(img));
                }
                if (ok) { at.fromCache = true; return at; }
            } catch (const std::exception& e) {
                LOG_WARN << "[roadsigns] cache " << meta << ": " << e.what();
            }
            at.slots.clear();
            at.pages.clear();
        }
    }
    // BAKE: faces shelf-packed onto pages, a pixel of air round each
    const uint8_t clear[4] = {0, 0, 0, 0};
    int x = 0, y = 0, rowH = 0;
    auto newPage = [&] { TextImage p; p.resize(pageSize, pageSize, clear); at.pages.push_back(std::move(p)); x = y = rowH = 0; };
    newPage();
    for (const IslandSign& s : signs) {
        const SignFace f = layoutSign(s);
        double tab = 0;
        const TextImage face = rasterizeSignFace(f, font, k, &tab);
        if (face.w + 2 > pageSize || face.h + 2 > pageSize) { at.slots.push_back({}); continue; }
        if (x + face.w + 2 > pageSize) { x = 0; y += rowH + 2; rowH = 0; }
        if (y + face.h + 2 > pageSize) newPage();
        TextImage& pg = at.pages.back();
        for (int j = 0; j < face.h; ++j)
            std::copy_n(&face.rgba[static_cast<std::size_t>(j) * face.w * 4], static_cast<std::size_t>(face.w) * 4,
                        &pg.rgba[(static_cast<std::size_t>(y + 1 + j) * pageSize + x + 1) * 4]);
        RoadSignAtlas::Slot sl;
        sl.page = static_cast<int>(at.pages.size()) - 1;
        // the face's panel and tab, inside the canvas's one-pixel margin
        sl.u0 = static_cast<float>(x + 2) / pageSize;
        sl.v0 = static_cast<float>(y + 2) / pageSize;
        sl.u1 = static_cast<float>(x + face.w) / pageSize;
        sl.v1 = static_cast<float>(y + face.h) / pageSize;
        sl.w = f.w;
        sl.h = f.h;
        sl.tab = tab;
        for (const SignElem& e : f.elems) if (e.kind == "tab") sl.tabW = e.w;
        at.slots.push_back(sl);
        x += face.w + 2;
        rowH = std::max(rowH, face.h);
    }
    if (!meta.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(cacheDir, ec);
        nlohmann::json m;
        m["pages"] = at.pages.size();
        m["slots"] = nlohmann::json::array();
        for (const RoadSignAtlas::Slot& s : at.slots) m["slots"].push_back({{"page", s.page}, {"uv", {s.u0, s.v0, s.u1, s.v1}}, {"w", s.w}, {"h", s.h}, {"tab", s.tab}, {"tabW", s.tabW}});
        for (std::size_t p = 0; p < at.pages.size(); ++p)
            stbi_write_png((cacheDir + "/roadsigns_" + at.key + "_" + std::to_string(p) + ".png").c_str(), at.pages[p].w, at.pages[p].h, 4,
                           at.pages[p].rgba.data(), at.pages[p].w * 4);
        std::ofstream(meta) << m.dump();
    }
    return at;
}

RoadSignMeshes buildRoadSignMeshes(const std::vector<IslandSign>& signs, const RoadSignAtlas& atlas,
                                   const std::function<double(double, double)>& ground, double carriageHalf) {
    RoadSignMeshes out;
    // grouped by 400 m cell (and page), so each group's draw distance culls it
    constexpr double kCell = 400.0;
    std::map<std::tuple<int, int, int>, std::size_t> faceGroup;
    std::map<std::pair<int, int>, std::size_t> steelGroup;
    RenderMesh* steelMesh = nullptr;
    const Vec3 steelCol(0.55f, 0.57f, 0.58f), backCol(0.45f, 0.47f, 0.48f);
    auto steelBox = [&](const Vec3& size, const Vec3& at, float yaw) {
        const std::size_t first = steelMesh->vertices.size();
        MeshBuilder::appendTransformed(*steelMesh, MeshBuilder::box(size), Mat4::trs(at, Quat::fromAxisAngle(Vec3(0, 1, 0), yaw), Vec3(1, 1, 1)));
        for (std::size_t v = first; v < steelMesh->vertices.size(); ++v) steelMesh->vertices[v].color = steelCol;
    };
    for (std::size_t i = 0; i < signs.size() && i < atlas.slots.size(); ++i) {
        const IslandSign& s = signs[i];
        const RoadSignAtlas::Slot& sl = atlas.slots[i];
        if (sl.page < 0) continue;
        // the reader travels along `facing`; the panel faces them (its normal is -forward), and its
        // texture's u runs to their RIGHT in the world, whatever the plan's mirror
        const Vec3 fwd(static_cast<Real>(s.facing.x), 0, static_cast<Real>(s.facing.y));
        const Vec3 up(0, 1, 0);
        Vec3 right = cross(fwd, up);
        right = right * (1.0 / std::max(1e-6, static_cast<double>(right.length())));
        const Vec3 normal = fwd * -1.0f;
        const double g = ground(s.at.x, s.at.y);
        const bool overhead = s.mount == "overhead";
        const bool regulatory = s.kind == "do-not-enter" || s.kind == "wrong-way";
        const double bottom = overhead ? 5.6 : regulatory ? 1.5 : 2.1;
        const double W = sl.w, H = sl.h, T = sl.tab;
        const Vec3 base(static_cast<Real>(s.at.x), static_cast<Real>(g), static_cast<Real>(s.at.y));
        const Vec3 c0 = base + up * static_cast<Real>(bottom) - right * static_cast<Real>(W / 2);   // bottom-left as the reader sees it
        auto P = [&](double u, double v) { return c0 + right * static_cast<Real>(u) + up * static_cast<Real>(v); };
        // the face (tab included): bottom-left, bottom-right, top-right, top-left; v runs down the texture
        const int cx = static_cast<int>(std::floor(s.at.x / kCell)), cz = static_cast<int>(std::floor(s.at.y / kCell));
        auto it = faceGroup.find({sl.page, cx, cz});
        if (it == faceGroup.end()) { it = faceGroup.emplace(std::make_tuple(sl.page, cx, cz), out.panels.size()).first; out.panels.push_back({sl.page, {}, {}, 0}); }
        RenderMesh& m = out.panels[it->second].mesh;
        auto st = steelGroup.find({cx, cz});
        if (st == steelGroup.end()) { st = steelGroup.emplace(std::make_pair(cx, cz), out.steel.size()).first; out.steel.push_back({}); }
        steelMesh = &out.steel[st->second].mesh;
        const Vec3 lift = normal * 0.03f;   // the face stands just proud of the backing plate
        MeshBuilder::emitQuadUV(m, P(0, 0) + lift, P(W, 0) + lift, P(W, H + T) + lift, P(0, H + T) + lift, normal, Vec3(1, 1, 1),
                                sl.u0, sl.v1, sl.u1, sl.v1, sl.u1, sl.v0, sl.u0, sl.v0);
        // the back: a plain plate behind the panel, and behind the tab (the face is alpha-cut to both)
        MeshBuilder::emitQuad(*steelMesh, P(0, 0), P(0, H), P(W, H), P(W, 0), fwd, backCol);
        if (T > 0 && sl.tabW > 0) MeshBuilder::emitQuad(*steelMesh, P(0, H), P(0, H + T), P(sl.tabW, H + T), P(sl.tabW, H), fwd, backCol);
        const float yaw = std::atan2(static_cast<float>(fwd.x), static_cast<float>(fwd.z));
        if (overhead) {
            // a gantry across the carriageway: uprights either side, a truss at the panel's middle
            const double span = carriageHalf + 2.0, trussY = bottom + H / 2;
            for (double side : {-span, span}) {
                const Vec3 foot = base + right * static_cast<Real>(side);
                const double gf = ground(foot.x, foot.z);
                const double hgt = g + trussY + 0.4 - gf;
                steelBox(Vec3(0.35f, static_cast<float>(hgt), 0.35f), Vec3(foot.x, static_cast<Real>(gf + hgt / 2), foot.z) + fwd * 0.3f, yaw);
            }
            steelBox(Vec3(static_cast<float>(2 * span), 0.25f, 0.25f), base + up * static_cast<Real>(trussY + 0.3) + fwd * 0.3f, yaw);
            steelBox(Vec3(static_cast<float>(2 * span), 0.25f, 0.25f), base + up * static_cast<Real>(trussY - 0.3) + fwd * 0.3f, yaw);
        } else {
            // one post, or two under a wide panel, from the ground to the panel's top
            const int posts = W > 1.6 ? 2 : 1;
            for (int p = 0; p < posts; ++p) {
                const double u = posts == 1 ? W / 2 : (p == 0 ? W * 0.22 : W * 0.78);
                const Vec3 foot = P(u, 0) - up * static_cast<Real>(bottom);
                const double hgt = bottom + H;
                steelBox(Vec3(0.09f, static_cast<float>(hgt), 0.09f), Vec3(foot.x, static_cast<Real>(g + hgt / 2), foot.z) + fwd * 0.06f, yaw);
            }
        }
    }
    auto bound = [](const RenderMesh& m, Vec3& centre, double& radius) {
        Vec3 lo(1e30f, 1e30f, 1e30f), hi(-1e30f, -1e30f, -1e30f);
        for (const auto& v : m.vertices) { lo = Vec3(std::min(lo.x, v.position.x), std::min(lo.y, v.position.y), std::min(lo.z, v.position.z)); hi = Vec3(std::max(hi.x, v.position.x), std::max(hi.y, v.position.y), std::max(hi.z, v.position.z)); }
        centre = (lo + hi) * 0.5f;
        radius = (hi - lo).length() * 0.5;
    };
    for (RoadSignMeshes::Panels& p : out.panels) bound(p.mesh, p.centre, p.radius);
    for (RoadSignMeshes::Steel& p : out.steel) bound(p.mesh, p.centre, p.radius);
    return out;
}

}  // namespace engine
