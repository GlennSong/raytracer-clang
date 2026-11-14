#include "street_signs.h"

#include "../../mesh_builder.h"
#include "../../asset_root.h"
#include "../../../log.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <set>

namespace engine {

namespace {


Real distToRay(Vec2 p, Vec2 o, Vec2 d, Real len) {
    const Vec2 v = p - o;
    const Real t = std::clamp(v.x * d.x + v.y * d.y, Real(0), len);
    const Vec2 c = o + d * t;
    return (p - c).length();
}

}  // namespace

void applyStreetSignData(const nlohmann::json& signs, StreetSignParams& p) {
    if (!signs.is_object()) return;
    p.bladeHeight = signs.value("bladeHeight", static_cast<double>(p.bladeHeight));
    p.minBladeWidth = signs.value("minBladeWidth", static_cast<double>(p.minBladeWidth));
    p.maxBladeWidth = signs.value("maxBladeWidth", static_cast<double>(p.maxBladeWidth));
    p.topBladeY = signs.value("topBladeY", static_cast<double>(p.topBladeY));
    p.bladeStep = signs.value("bladeStep", static_cast<double>(p.bladeStep));
    p.kerbGap = signs.value("kerbGap", static_cast<double>(p.kerbGap));
    p.bladePx = signs.value("bladePx", p.bladePx);
    p.pagePx = signs.value("pagePx", p.pagePx);
    p.capFraction = signs.value("capFraction", p.capFraction);
    p.capFloorFraction = signs.value("capFloorFraction", p.capFloorFraction);
    auto colour = [&](const char* key, uint8_t* out) {
        if (!signs.contains(key) || !signs[key].is_array()) return;
        const auto& c = signs[key];
        for (std::size_t i = 0; i < 3 && i < c.size(); ++i)
            out[i] = static_cast<uint8_t>(std::clamp(c[i].get<int>(), 0, 255));
        out[3] = 255;
    };
    colour("face", p.face);
    colour("legend", p.legend);
}

const StreetSignParams& streetSignParams() {
    static StreetSignParams p = [] {
        StreetSignParams q;
        std::ifstream in(assetPath("assets/data/streets.json"));
        if (in) {
            nlohmann::json doc;
            try {
                in >> doc;
                applyStreetSignData(doc.value("signs", nlohmann::json::object()), q);
            } catch (const std::exception& e) {
                LOG_WARN << "[streets] signs: " << e.what();
            }
        }
        return q;
    }();
    return p;
}

Real SignBlade::widthM() const {
    return hPx > 0 ? heightM * static_cast<Real>(wPx) / static_cast<Real>(hPx) : 0.0;
}

std::vector<StreetSignPost> planStreetSigns(const std::vector<SignCorner>& corners, const RoadGraph& g,
                                            const std::function<Real(Real, Real)>& ground,
                                            const StreetSignParams& p,
                                            const std::vector<Vec3>& avoid) {
    std::vector<StreetSignPost> out;
    const int N = static_cast<int>(g.nodes.size());
    struct Arm { Vec2 point, dir; Real angle; Real hw; const SignCorner::Arm* src; };
    // Every carriageway, bucketed: a corner must clear the NEIGHBOURS' roads
    // too (a parallel street, a nearby junction's arm), not only its own.
    constexpr Real kCell = 40.0;
    std::map<std::pair<int, int>, std::vector<int>> edgeCells;
    for (int e = 0; e < static_cast<int>(g.edges.size()); ++e) {
        const RoadEdge& ed = g.edges[static_cast<std::size_t>(e)];
        if (ed.a < 0 || ed.b < 0 || ed.a >= N || ed.b >= N) continue;
        const Vec2 a = g.nodes[static_cast<std::size_t>(ed.a)].pos;
        const Vec2 b = g.nodes[static_cast<std::size_t>(ed.b)].pos;
        const int x0 = static_cast<int>(std::floor((std::min(a.x, b.x) - ed.width) / kCell));
        const int x1 = static_cast<int>(std::floor((std::max(a.x, b.x) + ed.width) / kCell));
        const int z0 = static_cast<int>(std::floor((std::min(a.y, b.y) - ed.width) / kCell));
        const int z1 = static_cast<int>(std::floor((std::max(a.y, b.y) + ed.width) / kCell));
        for (int cx = x0; cx <= x1; ++cx)
            for (int cz = z0; cz <= z1; ++cz) edgeCells[{cx, cz}].push_back(e);
    }
    auto inAnyRoad = [&](Vec2 q, Real gap) {
        const auto it = edgeCells.find({static_cast<int>(std::floor(q.x / kCell)),
                                        static_cast<int>(std::floor(q.y / kCell))});
        if (it == edgeCells.end()) return false;
        for (int e : it->second) {
            const RoadEdge& ed = g.edges[static_cast<std::size_t>(e)];
            const Vec2 a = g.nodes[static_cast<std::size_t>(ed.a)].pos;
            const Vec2 b = g.nodes[static_cast<std::size_t>(ed.b)].pos;
            const Vec2 ab = b - a;
            const Real l = ab.length();
            if (l < 1e-9) continue;
            if (distToRay(q, a, ab * (1.0 / l), l) < ed.width * 0.5 + gap) return true;
        }
        return false;
    };
    for (const SignCorner& corner : corners) {
        // A JUNCTION WHERE TWO OR MORE NAMED STREETS MEET (the model says which: a carry-on is no junction, ADR-0419; a
        // ramp's landing on a street meets one named street)
        std::vector<Arm> arms;
        std::set<std::string> streets;
        for (const SignCorner::Arm& a : corner.arms) {
            if (a.name.empty() || a.dir.length() < 1e-6) continue;
            const Vec2 d = a.dir * (1.0 / a.dir.length());
            arms.push_back({a.point, d, std::atan2(d.y, d.x), a.halfWidth, &a});
            streets.insert(a.name);
        }
        if (streets.size() < 2) continue;
        std::sort(arms.begin(), arms.end(), [](const Arm& a, const Arm& b) { return a.angle < b.angle; });

        // THE CORNER: between each pair of neighbouring arms, where the two kerb lines meet (each arm's centre line
        // offset by its half-width plus kerbGap, on the other arm's side), pushed out until it clears every
        // carriageway. Prefer a true corner (60..150 degrees); the flat side of a T is the fallback.
        // Each arm runs from where it meets the junction, and a junction the plan spread over several points has its
        // arms meeting it apart: so the corner comes from the arms' own lines, never from a middle point between
        // their mouths (which, on a spread junction, is inside a block). A corner whose kerb lines do not meet near
        // both mouths (two arms leaving a spread junction side by side) is no corner.
        auto clear = [&](Vec2 q) {
            for (const Arm& a : arms)
                if (distToRay(q, a.point, a.dir, 40.0) < a.hw + p.kerbGap * 0.9) return false;
            if (inAnyRoad(q, p.kerbGap * 0.6)) return false;
            for (const Vec3& v : avoid)
                if ((Vec2(v.x, v.z) - q).length() < 1.4) return false;
            return true;
        };
        // On the sidewalk of BOTH arms: within the band past each one's kerb (measured from its mouth onward).
        auto onBothSidewalks = [&](Vec2 q, const Arm& a, const Arm& b) {
            return distToRay(q, a.point, a.dir, 40.0) <= a.hw + p.kerbGap + p.sidewalkWidth + 3.0 &&
                   distToRay(q, b.point, b.dir, 40.0) <= b.hw + p.kerbGap + p.sidewalkWidth + 3.0;
        };
        constexpr Real kReach = 40.0;   // how far along an arm its kerb corner may lie
        bool found = false;
        Vec2 best;
        Real bestScore = 1e30;
        std::set<std::string> cornerOf;   // the two streets whose corner the post stands on
        // Every pair of arms, b counter-clockwise of a -- not only neighbours by heading: on a spread junction the arm
        // between two others may leave from another of its points, and the corner of the two is still a corner. A
        // pair with a third arm between them at their own mouth has no clear corner (clear() refuses its road).
        const std::size_t K = arms.size();
        for (std::size_t pair = 0; pair < K * K; ++pair) {
            const std::size_t i = pair / K, j = (pair / K + 1 + pair % K) % K;
            if (i == j) continue;
            const Arm& a = arms[i];
            const Arm& b = arms[j];
            Real ang = b.angle - a.angle;
            if (ang <= 0) ang += 2 * PI;
            // Each arm's side facing the other (b lies counter-clockwise of a).
            const Vec2 na(-a.dir.y, a.dir.x);
            const Vec2 nb(b.dir.y, -b.dir.x);
            Vec2 out = na + nb;
            out = out.length() > 1e-6 ? out * (1.0 / out.length())
                                      : Vec2(std::cos(a.angle + ang * 0.5), std::sin(a.angle + ang * 0.5));
            const Vec2 pa = a.point + na * (a.hw + p.kerbGap);
            const Vec2 pb = b.point + nb * (b.hw + p.kerbGap);
            // pa + a.dir * t = pb + b.dir * u
            const Real den = a.dir.x * b.dir.y - a.dir.y * b.dir.x;
            Vec2 q;
            if (std::fabs(den) > 0.05) {
                const Vec2 w = pb - pa;
                const Real t = (w.x * b.dir.y - w.y * b.dir.x) / den;
                const Real u = (w.x * a.dir.y - w.y * a.dir.x) / den;
                if (t < -2.0 || u < -2.0 || t > kReach || u > kReach) continue;
                q = pa + a.dir * t;
            } else {
                // The two kerbs run on in one line (the flat side of a T) or side by side (no corner).
                if ((a.point - b.point).length() > a.hw + b.hw + 2.0 * p.kerbGap) continue;
                q = (pa + pb) * 0.5;
            }
            Real extra = 0;
            while (!clear(q) && extra < 8.0) { extra += 0.5; q = q + out * 0.5; }
            if (!clear(q) || !onBothSidewalks(q, a, b)) continue;
            const bool trueCorner = ang > 60.0 * PI / 180.0 && ang < 150.0 * PI / 180.0;
            const Real score = extra + (trueCorner ? 0.0 : 50.0);
            if (score < bestScore) { bestScore = score; best = q; found = true; cornerOf = {a.src->name, b.src->name}; }
        }
        if (!found) continue;

        StreetSignPost post;
        post.city = corner.city;
        post.junction = corner.junction;
        post.base = Vec3(best.x, ground ? ground(best.x, best.y) : 0.0, best.y);
        // One blade per street, at most three: the two streets of the corner it stands on, then the widest of the rest
        // (a junction spread over several points can have more streets than one post names); stacked widest first
        // (the main road on top; then by name, so the order is the same every run), each parallel to its street.
        std::vector<std::pair<Real, std::string>> order;
        for (const std::string& s : streets) {
            Real w = 0;
            for (const Arm& a : arms)
                if (a.src->name == s) w = std::max(w, a.hw);
            order.emplace_back(-w, s);
        }
        std::sort(order.begin(), order.end());
        std::stable_partition(order.begin(), order.end(),
                              [&](const std::pair<Real, std::string>& o) { return cornerOf.count(o.second) > 0; });
        if (order.size() > 3) order.resize(3);
        std::sort(order.begin(), order.end());
        for (std::size_t bi = 0; bi < order.size(); ++bi) {
            std::vector<const Arm*> mine;
            for (const Arm& a : arms)
                if (a.src->name == order[bi].second) mine.push_back(&a);
            Vec2 along = mine.front()->dir;
            if (mine.size() >= 2) {   // through street: the line it runs on
                const Vec2 line = mine[0]->dir - mine[1]->dir;
                if (line.length() > 1e-6) along = line * (1.0 / line.length());
            }
            post.blades.push_back({order[bi].second, mine.front()->src->arm, along, p.topBladeY - p.bladeStep * static_cast<Real>(bi)});
        }
        out.push_back(std::move(post));
    }
    (void)N;
    return out;
}

SignAtlas buildSignAtlas(const Font& font, const std::vector<StreetSignPost>& posts, const StreetSignParams& p) {
    SignAtlas atlas;
    std::set<std::string> streets;
    for (const StreetSignPost& post : posts)
        for (const StreetSignBlade& b : post.blades) streets.insert(b.name);
    const int H = p.bladePx;
    const int maxW = static_cast<int>(std::floor(H * p.maxBladeWidth / p.bladeHeight));
    const int minW = static_cast<int>(std::ceil(H * p.minBladeWidth / p.bladeHeight));
    const int padX = static_cast<int>(std::lround(H * 0.28));
    const float capUnit = font.capHeight(1.0f);
    if (capUnit <= 0) return atlas;
    const float em0 = p.capFraction * H / capUnit;
    const float emFloor = p.capFloorFraction * H / capUnit;
    const float budget = static_cast<float>(maxW - 2 * padX);

    struct Made { std::string street; SignBlade blade; TextImage img; };
    std::vector<Made> made;
    for (const std::string& s : streets) {
        if (s.empty()) continue;
        const std::string& name = s;
        SignBlade b;
        // Signs abbreviate the suffix, as real blades do ("Market St").
        b.text = abbreviateStreetName(name);
        b.abbreviated = b.text != name;
        b.heightM = p.bladeHeight;
        float em = em0, xs = 1.0f;
        float tw = font.measure(b.text, em, xs);
        if (tw > budget) { xs = std::max(0.82f, budget / tw); tw = font.measure(b.text, em, xs); }
        if (tw > budget) { em = std::max(emFloor, em * budget / tw); tw = font.measure(b.text, em, xs); }
        b.xScale = xs;
        b.textPx = tw;
        b.capPx = font.capHeight(em);
        b.hPx = H;
        b.wPx = std::clamp(static_cast<int>(std::ceil(tw)) + 2 * padX, minW, maxW);
        TextImage img;
        img.resize(b.wPx, H, p.face);
        // The white border, inset like a real blade's.
        const int in = std::max(2, H / 26), t = std::max(2, H / 30);
        img.fillRect(in, in, b.wPx - in, in + t, p.legend);
        img.fillRect(in, H - in - t, b.wPx - in, H - in, p.legend);
        img.fillRect(in, in, in + t, H - in, p.legend);
        img.fillRect(b.wPx - in - t, in, b.wPx - in, H - in, p.legend);
        const float baseline = 0.5f * (H + b.capPx);
        font.draw(img, b.text, 0.5f * (b.wPx - tw), baseline, em, p.legend, xs);
        made.push_back({s, b, std::move(img)});
    }
    // Shelf-pack, widest first; each blade padded by edge-extension so the
    // mip chain does not bleed a neighbour's green into it.
    std::stable_sort(made.begin(), made.end(), [](const Made& a, const Made& b) {
        return a.blade.wPx > b.blade.wPx;
    });
    const int pad = 6, W = p.pagePx, rowH = H + 2 * pad;
    int x = 0, y = 0;
    std::vector<int> usedH;
    auto newPage = [&]() {
        TextImage page;
        page.resize(W, p.pagePx, p.face);
        atlas.pages.push_back(std::move(page));
        usedH.push_back(0);
        x = 0;
        y = 0;
    };
    for (Made& m : made) {
        const int cw = m.blade.wPx + 2 * pad;
        if (atlas.pages.empty()) newPage();
        if (x + cw > W) { x = 0; y += rowH; }
        if (y + rowH > p.pagePx) newPage();
        TextImage& page = atlas.pages.back();
        const int ox = x + pad, oy = y + pad;
        for (int yy = -pad; yy < m.blade.hPx + pad; ++yy)
            for (int xx = -pad; xx < m.blade.wPx + pad; ++xx) {
                const int sx = std::clamp(xx, 0, m.blade.wPx - 1);
                const int sy = std::clamp(yy, 0, m.blade.hPx - 1);
                const int dx = ox + xx, dy = oy + yy;
                if (dx < 0 || dy < 0 || dx >= page.w || dy >= page.h) continue;
                const uint8_t* s = &m.img.rgba[(static_cast<std::size_t>(sy) * m.blade.wPx + sx) * 4];
                uint8_t* d = &page.rgba[(static_cast<std::size_t>(dy) * page.w + dx) * 4];
                for (int c = 0; c < 4; ++c) d[c] = s[c];
            }
        m.blade.page = static_cast<int>(atlas.pages.size()) - 1;
        m.blade.u0 = static_cast<float>(ox) / W;
        m.blade.u1 = static_cast<float>(ox + m.blade.wPx) / W;
        m.blade.v0 = static_cast<float>(oy);   // pixels for now: the page may shrink
        m.blade.v1 = static_cast<float>(oy + m.blade.hPx);
        usedH.back() = std::max(usedH.back(), y + rowH);
        x += cw;
        atlas.blades[m.street] = m.blade;
    }
    // Trim each page to the rows it used, then turn v into texture space.
    for (std::size_t pi = 0; pi < atlas.pages.size(); ++pi) {
        TextImage& page = atlas.pages[pi];
        const int h = std::max(1, usedH[pi]);
        page.rgba.resize(static_cast<std::size_t>(page.w) * h * 4);
        page.h = h;
    }
    for (auto& [s, b] : atlas.blades) {
        const float h = static_cast<float>(atlas.pages[static_cast<std::size_t>(b.page)].h);
        b.v0 /= h;
        b.v1 /= h;
    }
    return atlas;
}

StreetSignMeshes buildStreetSignMeshes(const std::vector<StreetSignPost>& posts,
                                       const SignAtlas& atlas, const StreetSignParams& p) {
    StreetSignMeshes out;
    // The post: a slim square pole, unit height (each post's transform scales
    // it). It stops UNDER the lowest blade -- the blades sit on top on their
    // bracket -- instead of running up through the lettering (the first
    // in-game look: the pole hid the middle of "Redwood").
    out.postMesh = MeshBuilder::box(Vec3(0.07, 1.0, 0.07));
    for (Vertex& v : out.postMesh.vertices) {
        v.position.y += 0.5;
        v.color = Vec3(0.42, 0.44, 0.46);
    }
    std::map<std::tuple<int, int, int>, std::size_t> cellOf;   // (cx, cz, page) -> blades index
    for (const StreetSignPost& post : posts) {
        Real lowest = p.topBladeY;
        for (const StreetSignBlade& bl : post.blades) lowest = std::min(lowest, bl.y);
        const Real postH = lowest - p.bladeHeight * 0.5 + 0.01;
        out.posts.push_back(Mat4::trs(post.base, Quat::identity(), Vec3(1, postH, 1)));
        const int cx = static_cast<int>(std::floor(post.base.x / p.cellSize));
        const int cz = static_cast<int>(std::floor(post.base.z / p.cellSize));
        for (const StreetSignBlade& bl : post.blades) {
            const std::map<std::string, SignBlade>::const_iterator it = atlas.blades.find(bl.name);
            if (it == atlas.blades.end()) continue;
            const SignBlade& sb = it->second;
            const auto key = std::make_tuple(cx, cz, sb.page);
            auto ci = cellOf.find(key);
            if (ci == cellOf.end()) {
                ci = cellOf.emplace(key, out.blades.size()).first;
                StreetSignMeshes::Cell c;
                c.page = sb.page;
                out.blades.push_back(std::move(c));
            }
            RenderMesh& m = out.blades[ci->second].mesh;
            const Vec3 A(bl.along.x, 0, bl.along.y);
            const Vec3 U(0, 1, 0);
            const Vec3 nrm(-bl.along.y, 0, bl.along.x);   // front: text reads along +A
            const Vec3 c = post.base + Vec3(0, bl.y, 0);
            const Real hw = sb.widthM() * 0.5, hh = p.bladeHeight * 0.5;
            // Both faces, a few millimetres apart, each lettered to read
            // left-to-right from its own side.
            for (int side = 0; side < 2; ++side) {
                const Real sgn = side == 0 ? 1.0 : -1.0;
                const Vec3 n = nrm * sgn;
                const Vec3 right = A * sgn;          // screen-right for a viewer on this side
                const Vec3 cc = c + n * 0.006;
                const uint32_t base = static_cast<uint32_t>(m.vertices.size());
                auto vtx = [&](Vec3 pos, float u, float v) {
                    Vertex vx(pos, n, u, v);
                    vx.tangent = right;
                    m.vertices.push_back(vx);
                };
                vtx(cc - right * hw + U * hh, sb.u0, sb.v0);   // TL
                vtx(cc + right * hw + U * hh, sb.u1, sb.v0);   // TR
                vtx(cc + right * hw - U * hh, sb.u1, sb.v1);   // BR
                vtx(cc - right * hw - U * hh, sb.u0, sb.v1);   // BL
                MeshBuilder::quadFacing(m, base, base + 1, base + 2, base + 3, n);
            }
        }
    }
    for (StreetSignMeshes::Cell& c : out.blades) {
        Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
        for (const Vertex& v : c.mesh.vertices) {
            lo = Vec3(std::min(lo.x, v.position.x), std::min(lo.y, v.position.y), std::min(lo.z, v.position.z));
            hi = Vec3(std::max(hi.x, v.position.x), std::max(hi.y, v.position.y), std::max(hi.z, v.position.z));
        }
        c.centre = (lo + hi) * 0.5;
        c.radius = (hi - lo).length() * 0.5;
    }
    return out;
}

}  // namespace engine
