#include "city_plan.h"

#include "../parcel.h"
#include "../site_plan.h"
#include "../../../ai/nav_graph.h"
#include "../../../ai/pathfind.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>

namespace engine {
namespace plan {

namespace {

constexpr Real kPi = 3.14159265358979323846;

// ---- noise -------------------------------------------------------------------
uint32_t hash3(int x, int y, uint32_t s) {
    uint32_t h = s ^ (static_cast<uint32_t>(x) * 0x8da6b343u) ^ (static_cast<uint32_t>(y) * 0xd8163841u);
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    return h;
}
Real valueNoise(Real x, Real y, uint32_t s) {   // -1..1, smooth
    const int xi = static_cast<int>(std::floor(x)), yi = static_cast<int>(std::floor(y));
    const Real fx = x - xi, fy = y - yi;
    auto r = [&](int a, int b) { return (hash3(a, b, s) & 0xFFFF) / 65535.0 * 2.0 - 1.0; };
    const Real u = fx * fx * (3 - 2 * fx), v = fy * fy * (3 - 2 * fy);
    const Real a0 = r(xi, yi) + (r(xi + 1, yi) - r(xi, yi)) * u;
    const Real a1 = r(xi, yi + 1) + (r(xi + 1, yi + 1) - r(xi, yi + 1)) * u;
    return a0 + (a1 - a0) * v;
}
// Periodic in theta: a closed ring stays closed.
Real ringNoise(Real theta, int k, uint32_t s) {
    Real v = 0, norm = 0;
    for (int h = 2; h <= 5; ++h) {
        const uint32_t hh = hash3(k, h, s);
        const Real phase = (hh & 0xFFFF) / 65535.0 * 2 * kPi;
        const Real amp = 1.0 / h;
        v += amp * std::sin(h * theta + phase);
        norm += amp;
    }
    return v / norm;
}
Real smooth01(Real t) { t = std::max(Real(0), std::min(Real(1), t)); return t * t * (3 - 2 * t); }

// ---- roads as polylines ------------------------------------------------------
struct Polyline {
    std::vector<Vec2> pts;
    RoadClass klass = RoadClass::Local;
    Real width = 12;
    bool closed = false;
};

struct Frame {
    Vec2 c, u, v;
    Vec2 toWorld(Real a, Real b) const { return c + u * a + v * b; }
};

// Grid line positions from the centre outward: denser near the middle, the block
// growing toward midtown's rim.
std::vector<std::pair<Real, int>> gridPositions(Real coreBlock, Real midBlock, Real coreR, Real midR) {
    std::vector<std::pair<Real, int>> out{{0.0, 0}};
    for (int dir = -1; dir <= 1; dir += 2) {
        Real s = 0;
        for (int i = 1; i < 200; ++i) {
            const Real t = smooth01((s - coreR * 0.6) / std::max(Real(1), midR - coreR * 0.6));
            const Real step = coreBlock + (midBlock - coreBlock) * t;
            s += step;
            if (s > midR * 1.15) break;
            out.push_back({dir * s, dir * i});
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

// ---- planarize polylines (spatially hashed) ------------------------------------
struct SegRef { int road, seg; };

RoadGraph planarizePolylines(const std::vector<Polyline>& roads, Real snap = 0.6) {
    // All segments.
    std::vector<std::vector<std::vector<Real>>> cuts(roads.size());
    std::vector<std::pair<Vec2, Vec2>> segs;
    std::vector<SegRef> refs;
    for (std::size_t r = 0; r < roads.size(); ++r) {
        const auto& P = roads[r].pts;
        const std::size_t n = P.size();
        const std::size_t ns = roads[r].closed ? n : (n > 0 ? n - 1 : 0);
        cuts[r].assign(ns, {});
        for (std::size_t i = 0; i < ns; ++i) {
            segs.push_back({P[i], P[(i + 1) % n]});
            refs.push_back({static_cast<int>(r), static_cast<int>(i)});
        }
    }
    const Real cell = 40.0;
    std::unordered_map<long long, std::vector<int>> grid;
    auto key = [](int cx, int cy) { return (static_cast<long long>(cx) << 32) ^ static_cast<uint32_t>(cy); };
    for (int si = 0; si < static_cast<int>(segs.size()); ++si) {
        const Vec2& a = segs[si].first; const Vec2& b = segs[si].second;
        const int x0 = static_cast<int>(std::floor((std::min(a.x, b.x) - snap) / cell));
        const int x1 = static_cast<int>(std::floor((std::max(a.x, b.x) + snap) / cell));
        const int y0 = static_cast<int>(std::floor((std::min(a.y, b.y) - snap) / cell));
        const int y1 = static_cast<int>(std::floor((std::max(a.y, b.y) + snap) / cell));
        for (int cx = x0; cx <= x1; ++cx)
            for (int cy = y0; cy <= y1; ++cy) grid[key(cx, cy)].push_back(si);
    }
    std::set<std::pair<int, int>> tested;
    auto nearestT = [](const Vec2& p, const Vec2& a, const Vec2& b, Real& d) {
        const Vec2 ab = b - a; const Real L2 = ab.lengthSquared();
        Real t = L2 > 1e-12 ? dot(p - a, ab) / L2 : 0.0;
        t = std::max(Real(0), std::min(Real(1), t));
        d = (p - (a + ab * t)).length();
        return t;
    };
    for (const auto& [k, list] : grid) {
        for (std::size_t i = 0; i < list.size(); ++i)
            for (std::size_t j = i + 1; j < list.size(); ++j) {
                int s0 = list[i], s1 = list[j];
                if (s0 > s1) std::swap(s0, s1);
                if (!tested.insert({s0, s1}).second) continue;
                const SegRef& A = refs[s0]; const SegRef& B = refs[s1];
                if (A.road == B.road) {
                    const int n = static_cast<int>(cuts[A.road].size());
                    const int d = std::abs(A.seg - B.seg);
                    if (d <= 1 || (roads[A.road].closed && d == n - 1)) continue;   // neighbours share a vertex
                }
                const Vec2 p = segs[s0].first, q = segs[s0].second, a = segs[s1].first, b = segs[s1].second;
                const Vec2 r = q - p, s = b - a;
                const Real den = r.x * s.y - r.y * s.x;
                if (std::fabs(den) > 1e-12) {
                    const Vec2 ap = a - p;
                    const Real t = (ap.x * s.y - ap.y * s.x) / den, u = (ap.x * r.y - ap.y * r.x) / den;
                    if (t > 1e-6 && t < 1 - 1e-6 && u > 1e-6 && u < 1 - 1e-6) {
                        cuts[A.road][A.seg].push_back(t);
                        cuts[B.road][B.seg].push_back(u);
                        continue;
                    }
                }
                // T-junctions within the snap distance: an end of one on the other.
                Real d;
                for (const Vec2* e : {&p, &q}) {
                    const Real t = nearestT(*e, a, b, d);
                    if (d < snap && t > 1e-6 && t < 1 - 1e-6) cuts[B.road][B.seg].push_back(t);
                }
                for (const Vec2* e : {&a, &b}) {
                    const Real t = nearestT(*e, p, q, d);
                    if (d < snap && t > 1e-6 && t < 1 - 1e-6) cuts[A.road][A.seg].push_back(t);
                }
            }
    }
    // Nodes, snapped on a hash.
    RoadGraph g;
    std::unordered_map<long long, std::vector<int>> nodeGrid;
    const Real ncell = 2.0;
    auto nodeAt = [&](const Vec2& p) {
        const int cx = static_cast<int>(std::floor(p.x / ncell)), cy = static_cast<int>(std::floor(p.y / ncell));
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy) {
                auto it = nodeGrid.find(key(cx + dx, cy + dy));
                if (it == nodeGrid.end()) continue;
                for (int ni : it->second)
                    if ((g.nodes[static_cast<std::size_t>(ni)].pos - p).length() < snap) return ni;
            }
        const int id = static_cast<int>(g.nodes.size());
        g.nodes.push_back(RoadNode{p});
        nodeGrid[key(cx, cy)].push_back(id);
        return id;
    };
    std::set<std::pair<int, int>> have;
    auto addEdge = [&](int a, int b, const Polyline& R) {
        if (a == b) return;
        const std::pair<int, int> k2 = {std::min(a, b), std::max(a, b)};
        if (!have.insert(k2).second) return;
        RoadEdge e{a, b, R.width, R.klass};
        g.edges.push_back(e);
    };
    for (std::size_t r = 0; r < roads.size(); ++r) {
        const auto& P = roads[r].pts;
        const std::size_t n = P.size();
        int prev = -1;
        for (std::size_t i = 0; i < cuts[r].size(); ++i) {
            const Vec2 a = P[i], b = P[(i + 1) % n];
            if (prev < 0) prev = nodeAt(a);
            std::vector<Real>& ts = cuts[r][i];
            std::sort(ts.begin(), ts.end());
            for (Real t : ts) { const int m = nodeAt(a + (b - a) * t); addEdge(prev, m, roads[r]); prev = m; }
            const int e = nodeAt(b);
            addEdge(prev, e, roads[r]);
            prev = e;
        }
    }
    return g;
}

// Remove dead-end chains shorter than `stub` (the overshoot past a crossing road).
// A LINK SHORTER THAN THE ROADS THAT MEET THERE IS NOT A LINK, it is one junction the
// planarizer found twice — two grid lines crossing a boulevard a few metres apart leave a
// triangle whose inside is narrower than the pavement around it. The plan cannot see the
// harm (the block insets to nothing and drops out of the count) but the builder paves it,
// and its lanes then sit inside each other's footprints. So the two ends become one node.
void collapseShortLinks(RoadGraph& g, Real sidewalk) {
    for (int pass = 0; pass < 6; ++pass) {
        // Only a link BETWEEN JUNCTIONS counts: the graph's edges are 15 m samples along a
        // road, and collapsing those would fold every street into a point.
        std::vector<int> degree(g.nodes.size(), 0);
        for (const RoadEdge& e : g.edges) { ++degree[static_cast<std::size_t>(e.a)]; ++degree[static_cast<std::size_t>(e.b)]; }
        std::vector<Real> widest(g.nodes.size(), 0);
        for (const RoadEdge& e : g.edges) {
            widest[static_cast<std::size_t>(e.a)] = std::max(widest[static_cast<std::size_t>(e.a)], e.width);
            widest[static_cast<std::size_t>(e.b)] = std::max(widest[static_cast<std::size_t>(e.b)], e.width);
        }
        std::vector<int> merge(g.nodes.size());
        for (std::size_t i = 0; i < merge.size(); ++i) merge[i] = static_cast<int>(i);
        std::function<int(int)> find = [&](int a) { while (merge[static_cast<std::size_t>(a)] != a) a = merge[static_cast<std::size_t>(a)] = merge[static_cast<std::size_t>(merge[static_cast<std::size_t>(a)])]; return a; };
        bool any = false;
        for (const RoadEdge& e : g.edges) {
            const Vec2 pa = g.nodes[static_cast<std::size_t>(e.a)].pos, pb = g.nodes[static_cast<std::size_t>(e.b)].pos;
            if (degree[static_cast<std::size_t>(e.a)] < 3 || degree[static_cast<std::size_t>(e.b)] < 3) continue;
            const Real want = 0.5 * (widest[static_cast<std::size_t>(e.a)] + widest[static_cast<std::size_t>(e.b)]) + 4 * sidewalk;
            if ((pb - pa).length() >= want) continue;
            const int ra = find(e.a), rb = find(e.b);
            if (ra != rb) { merge[static_cast<std::size_t>(std::max(ra, rb))] = std::min(ra, rb); any = true; }
        }
        if (!any) return;
        // Rebuild: merged nodes keep the average position, edges within a group disappear.
        std::vector<Vec2> sum(g.nodes.size(), Vec2(0, 0));
        std::vector<int> count(g.nodes.size(), 0);
        for (std::size_t i = 0; i < g.nodes.size(); ++i) {
            const int r = find(static_cast<int>(i));
            sum[static_cast<std::size_t>(r)] = sum[static_cast<std::size_t>(r)] + g.nodes[i].pos;
            ++count[static_cast<std::size_t>(r)];
        }
        RoadGraph out;
        std::vector<int> remap(g.nodes.size(), -1);
        for (std::size_t i = 0; i < g.nodes.size(); ++i) {
            const int r = find(static_cast<int>(i));
            if (remap[static_cast<std::size_t>(r)] < 0) {
                remap[static_cast<std::size_t>(r)] = static_cast<int>(out.nodes.size());
                RoadNode n = g.nodes[static_cast<std::size_t>(r)];
                n.pos = sum[static_cast<std::size_t>(r)] * (1.0 / std::max(1, count[static_cast<std::size_t>(r)]));
                out.nodes.push_back(n);
            }
            remap[i] = remap[static_cast<std::size_t>(r)];
        }
        std::set<std::pair<int, int>> seen;
        for (const RoadEdge& e : g.edges) {
            RoadEdge n = e;
            n.a = remap[static_cast<std::size_t>(e.a)];
            n.b = remap[static_cast<std::size_t>(e.b)];
            if (n.a == n.b) continue;
            if (!seen.insert({std::min(n.a, n.b), std::max(n.a, n.b)}).second) continue;
            out.edges.push_back(n);
        }
        g = std::move(out);
    }
}

void pruneStubs(RoadGraph& g, Real stub) {
    for (int pass = 0; pass < 4; ++pass) {
        std::vector<std::vector<int>> inc(g.nodes.size());
        for (int e = 0; e < static_cast<int>(g.edges.size()); ++e) {
            inc[static_cast<std::size_t>(g.edges[e].a)].push_back(e);
            inc[static_cast<std::size_t>(g.edges[e].b)].push_back(e);
        }
        std::vector<char> drop(g.edges.size(), 0);
        bool any = false;
        for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v) {
            if (inc[static_cast<std::size_t>(v)].size() != 1) continue;
            std::vector<int> chain;
            Real len = 0;
            int cur = v, e = inc[static_cast<std::size_t>(v)][0];
            while (true) {
                chain.push_back(e);
                const RoadEdge& E = g.edges[static_cast<std::size_t>(e)];
                const int nxt = E.a == cur ? E.b : E.a;
                len += (g.nodes[static_cast<std::size_t>(nxt)].pos - g.nodes[static_cast<std::size_t>(cur)].pos).length();
                if (len > stub) break;
                if (inc[static_cast<std::size_t>(nxt)].size() != 2) {   // reached a junction (or another end)
                    for (int c : chain) drop[static_cast<std::size_t>(c)] = 1;
                    any = true;
                    break;
                }
                const auto& ie = inc[static_cast<std::size_t>(nxt)];
                e = ie[0] == e ? ie[1] : ie[0];
                cur = nxt;
            }
        }
        if (!any) break;
        std::vector<RoadEdge> kept;
        for (std::size_t i = 0; i < g.edges.size(); ++i) if (!drop[i]) kept.push_back(g.edges[i]);
        g.edges.swap(kept);
    }
}

Real polylineLength(const RoadGraph& g) {
    Real L = 0;
    for (const RoadEdge& e : g.edges) L += (g.nodes[static_cast<std::size_t>(e.a)].pos - g.nodes[static_cast<std::size_t>(e.b)].pos).length();
    return L;
}

Real distToPolyline(const Vec2& p, const std::vector<Vec2>& P, bool closed) {
    Real best = 1e30;
    const std::size_t n = P.size();
    const std::size_t ns = closed ? n : (n ? n - 1 : 0);
    for (std::size_t i = 0; i < ns; ++i) {
        const Vec2 a = P[i], b = P[(i + 1) % n], ab = b - a;
        const Real L2 = ab.lengthSquared();
        Real t = L2 > 1e-12 ? dot(p - a, ab) / L2 : 0.0;
        t = std::max(Real(0), std::min(Real(1), t));
        best = std::min(best, (p - (a + ab * t)).length());
    }
    return best;
}

const char* planClassName(RoadClass k) {
    switch (k) {
        case RoadClass::Freeway: return "freeway";
        case RoadClass::Arterial: return "arterial";
        case RoadClass::Collector: return "collector";
        case RoadClass::Ramp: return "ramp";
        default: return "local";
    }
}

// The nearest point on a polyline: its distance, and the direction the polyline runs there.
// Distance alone cannot tell a CROSSING from a shared line — a street crossing a 22 m arterial
// is inside its corridor for fifty metres — so everything that asks "are these two roads on top
// of each other" asks about the angle too.
Real distToPolylineDir(const Vec2& p, const std::vector<Vec2>& P, Vec2* dirOut, bool* pastEndOut) {
    Real best = 1e30;
    for (std::size_t i = 0; i + 1 < P.size(); ++i) {
        const Vec2 a = P[i], b = P[i + 1], ab = b - a;
        const Real L2 = ab.lengthSquared();
        const Real raw = L2 > 1e-12 ? dot(p - a, ab) / L2 : 0.0;
        const Real t = std::max(Real(0), std::min(Real(1), raw));
        const Real d = (p - (a + ab * t)).length();
        if (d < best) {
            best = d;
            if (dirOut && L2 > 1e-12) *dirOut = ab * (1 / std::sqrt(L2));
            // BESIDE the road, or PAST THE END of it? A road that continues where another
            // stops — the same street on the far side of a junction — is nearest to that
            // road's last vertex, not to any point along it. That is a continuation, not two
            // roads sharing a line, and it is most of what a naive distance test reports.
            if (pastEndOut) *pastEndOut = (i == 0 && raw < 0) || (i + 2 == P.size() && raw > 1);
        }
    }
    return best;
}

// Two roads share a line when they are inside each other's corridor AND running the same way.
constexpr Real kParallelSin = 0.35;   // ~20 degrees: anything more open is a crossing
bool sharesLine(const Vec2& dirA, const Vec2& dirB) { return std::fabs(cross(dirA, dirB)) < kParallelSin; }

Real pointSegDistance(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const Real L2 = ab.lengthSquared();
    Real t = L2 > 1e-12 ? dot(p - a, ab) / L2 : 0.0;
    t = std::max(Real(0), std::min(Real(1), t));
    return (p - (a + ab * t)).length();
}

// Distance between two segments: 0 when they cross, else the nearest endpoint approach.
Real segmentDistance(const Vec2& a0, const Vec2& a1, const Vec2& b0, const Vec2& b1) {
    const Vec2 r = a1 - a0, s2 = b1 - b0;
    const Real denom = cross(r, s2);
    if (std::fabs(denom) > 1e-12) {
        const Real t = cross(b0 - a0, s2) / denom, u = cross(b0 - a0, r) / denom;
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return 0;
    }
    return std::min(std::min(pointSegDistance(a0, b0, b1), pointSegDistance(a1, b0, b1)),
                    std::min(pointSegDistance(b0, a0, a1), pointSegDistance(b1, a0, a1)));
}

std::vector<std::size_t> dpKeep(const Poly2& P, Real tol) {
    const std::size_t n = P.size();
    if (n < 4) { std::vector<std::size_t> all(n); for (std::size_t i = 0; i < n; ++i) all[i] = i; return all; }
    std::vector<char> keep(n, 0); keep[0] = keep[n - 1] = 1;
    std::vector<std::pair<std::size_t, std::size_t>> st{{0, n - 1}};
    while (!st.empty()) {
        auto [a, b] = st.back(); st.pop_back();
        if (b <= a + 1) continue;
        Real bd = -1; std::size_t bi = a;
        const Vec2 d = P[b] - P[a]; const Real L = d.length();
        for (std::size_t i = a + 1; i < b; ++i) {
            const Vec2 v = P[i] - P[a];
            const Real dist = L > 1e-9 ? std::fabs(cross(v, d)) / L : v.length();
            if (dist > bd) { bd = dist; bi = i; }
        }
        if (bd > tol) { keep[bi] = 1; st.push_back({a, bi}); st.push_back({bi, b}); }
    }
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < n; ++i) if (keep[i]) out.push_back(i);
    return out;
}

}  // namespace

// ---- brief I/O -------------------------------------------------------------------
Brief briefFromJson(const nlohmann::json& j) {
    Brief b;
    b.name = j.value("name", b.name);
    b.seed = j.value("seed", b.seed);
    b.size = j.value("size", b.size);
    if (j.contains("center") && j["center"].is_array() && j["center"].size() == 2)
        b.center = Vec2(j["center"][0].get<Real>(), j["center"][1].get<Real>());
    if (j.contains("core")) {
        const auto& c = j["core"];
        b.coreRadius = c.value("radius", b.coreRadius);
        if (c.contains("block") && c["block"].size() == 2) { b.coreBlockU = c["block"][0]; b.coreBlockV = c["block"][1]; }
        b.gridAngleDeg = c.value("angle", b.gridAngleDeg);
        b.arterialEvery = c.value("arterialEvery", b.arterialEvery);
    }
    if (j.contains("midtown")) {
        const auto& m = j["midtown"];
        b.midRadius = m.value("radius", b.midRadius);
        if (m.contains("block") && m["block"].size() == 2) { b.midBlockU = m["block"][0]; b.midBlockV = m["block"][1]; }
        b.warp = m.value("warp", b.warp);
    }
    if (j.contains("outskirts")) {
        const auto& o = j["outskirts"];
        b.ringSpacing = o.value("ringSpacing", b.ringSpacing);
        b.spokes = o.value("spokes", b.spokes);
        b.curvature = o.value("curvature", b.curvature);
        b.wedgeStreetSpacing = o.value("streetSpacing", b.wedgeStreetSpacing);
        b.outerMargin = o.value("margin", b.outerMargin);
    }
    if (j.contains("freeway")) {
        const auto& f = j["freeway"];
        b.freewayRadius = f.value("radius", b.freewayRadius);
        b.freewayRadials = f.value("radials", b.freewayRadials);
        b.freewayWobble = f.value("wobble", b.freewayWobble);
    }
    b.relief = j.value("relief", b.relief);
    if (j.contains("roads")) {
        const auto& r = j["roads"];
        b.localWidth = r.value("local", b.localWidth);
        b.collectorWidth = r.value("collector", b.collectorWidth);
        b.arterialWidth = r.value("arterial", b.arterialWidth);
        b.freewayWidth = r.value("freeway", b.freewayWidth);
        b.sidewalk = r.value("sidewalk", b.sidewalk);
    }
    return b;
}

nlohmann::json briefToJson(const Brief& b) {
    return {{"name", b.name}, {"seed", b.seed}, {"size", b.size}, {"center", {b.center.x, b.center.y}},
            {"core", {{"radius", b.coreRadius}, {"block", {b.coreBlockU, b.coreBlockV}}, {"angle", b.gridAngleDeg},
                      {"arterialEvery", b.arterialEvery}}},
            {"midtown", {{"radius", b.midRadius}, {"block", {b.midBlockU, b.midBlockV}}, {"warp", b.warp}}},
            {"outskirts", {{"ringSpacing", b.ringSpacing}, {"spokes", b.spokes}, {"curvature", b.curvature},
                           {"streetSpacing", b.wedgeStreetSpacing}, {"margin", b.outerMargin}}},
            {"freeway", {{"radius", b.freewayRadius}, {"radials", b.freewayRadials}, {"wobble", b.freewayWobble}}},
            {"relief", b.relief},
            {"roads", {{"local", b.localWidth}, {"collector", b.collectorWidth}, {"arterial", b.arterialWidth},
                       {"freeway", b.freewayWidth}, {"sidewalk", b.sidewalk}}}};
}

Brief variantOf(const Brief& b, int k) {
    if (k <= 0) return b;
    Brief v = b;
    auto u = [&](int salt) { return (hash3(k, salt, b.seed) & 0xFFFF) / 65535.0; };   // 0..1
    v.name = b.name + "_v" + std::to_string(k);
    v.seed = b.seed * 7919u + static_cast<uint32_t>(k) * 104729u;
    v.gridAngleDeg = b.gridAngleDeg + (u(1) - 0.5) * 40.0;
    const Real bs = 0.85 + 0.3 * u(2);
    v.coreBlockU *= bs; v.coreBlockV *= bs;
    v.midBlockU *= 0.85 + 0.3 * u(3); v.midBlockV *= 0.85 + 0.3 * u(3);
    v.arterialEvery = 2 + static_cast<int>(u(4) * 2.99);
    v.warp = b.warp * (0.5 + u(5));
    v.spokes = std::max(6, b.spokes + static_cast<int>((u(6) - 0.5) * 8));
    v.curvature = b.curvature * (0.5 + u(7));
    v.ringSpacing = b.ringSpacing * (0.85 + 0.3 * u(8));
    v.freewayRadius = b.freewayRadius * (0.92 + 0.12 * u(9));
    v.freewayRadials = static_cast<int>(u(10) * 3.99);
    v.coreRadius = b.coreRadius * (0.85 + 0.3 * u(11));
    return v;
}

const char* blockUseName(BlockUse u) {
    switch (u) {
        case BlockUse::Lots: return "lots";
        case BlockUse::Landmark: return "landmark";
        case BlockUse::Park: return "park";
        case BlockUse::RightOfWay: return "right_of_way";
    }
    return "?";
}

// ---- generate -----------------------------------------------------------------------
CityPlan generatePlan(const Brief& B) {
    CityPlan plan;
    plan.brief = B;
    const Real ang = B.gridAngleDeg * kPi / 180.0;
    const Frame F{B.center, Vec2(std::cos(ang), std::sin(ang)), Vec2(-std::sin(ang), std::cos(ang))};
    const Real outerR = B.size * 0.5 - B.outerMargin;
    auto midR = [&](Real theta) { return B.midRadius * (1.0 + 0.05 * ringNoise(theta, 99, B.seed)); };
    // The freeway ring and its corridor: the ring's centreline radius at an angle, and the
    // half-width out to the frontage roads either side of it.
    auto freewayR = [&](Real th) { return B.freewayRadius + B.freewayWobble * ringNoise(th, 777, B.seed); };
    const Real corridorHalf = B.freewayWidth * 0.5 + 35.0;
    const bool haveFreeway = B.freewayRadius > B.midRadius + corridorHalf && B.freewayRadius + corridorHalf < outerR;
    // The warp field: zero in the core, full at midtown's rim — and zero again across the
    // freeway's corridor. The freeway is not warped (at 260 m a 19 m warp bends it tighter
    // than a motorway can), so a warped frontage road wandered 37-69 m from it: too close
    // for a ramp beside the deck in places, and never parallel enough to land one along.
    auto warped = [&](const Vec2& p) {
        const Vec2 d = p - B.center;
        const Real r = d.length();
        Real w = B.warp * smooth01((r - B.coreRadius) / std::max(Real(1), B.midRadius - B.coreRadius));
        if (haveFreeway && w > 0) {
            const Real off = std::fabs(r - freewayR(std::atan2(d.y, d.x)));
            w *= smooth01((off - corridorHalf - 10) / 80.0);
        }
        if (w <= 0) return p;
        const Real s = 1.0 / 260.0;
        return p + Vec2(valueNoise(p.x * s, p.y * s, B.seed + 11), valueNoise(p.x * s + 17.3, p.y * s + 5.1, B.seed + 23)) * w;
    };
    std::vector<Polyline> roads;
    // --- the grid (core + midtown), clipped to midtown's (wobbly) rim, overshooting it a little ---
    const auto posU = gridPositions(B.coreBlockU, B.midBlockU, B.coreRadius, B.midRadius);
    const auto posV = gridPositions(B.coreBlockV, B.midBlockV, B.coreRadius, B.midRadius);
    auto gridLine = [&](Real fixed, bool alongV, int idx) {
        const bool arterial = idx % std::max(1, B.arterialEvery) == 0;
        Polyline cur;
        cur.klass = arterial ? RoadClass::Arterial : RoadClass::Local;
        cur.width = arterial ? B.arterialWidth : B.localWidth;
        const Real L = B.midRadius * 1.25, step = 15.0;
        // A grid line ENDS ON the rim boulevard — it does not run 25 m past it. Overshooting
        // put the last stretch of every grid line inside the boulevard's corridor, and a line
        // that leaves tangentially stayed in it for hundreds of metres: two roads on one line.
        // A grid line CROSSES the rim boulevard and stops just past it: a real crossing, which
        // the planarizer splits into a junction (an analytic "rim point" of my own landed a
        // metre off the boulevard's own vertices and made TWO nodes, which is a road running
        // beside itself). The overshoot is short enough for pruneStubs to take back.
        const Real overshoot = 10.0;
        auto flush = [&] { if (cur.pts.size() >= 2) roads.push_back(cur); cur.pts.clear(); };
        const Vec2 dirLine = alongV ? F.v : F.u;
        for (Real t = -L; t <= L; t += step) {
            const Vec2 p = alongV ? F.toWorld(fixed, t) : F.toWorld(t, fixed);
            const Vec2 d = p - B.center;
            const Real th = std::atan2(d.y, d.x);
            // A line that leaves TANGENTIALLY would run inside the boulevard's corridor for
            // hundreds of metres — two roads on one line. Within 20 degrees of parallel, it
            // stops before the corridor instead of grazing along it.
            const Real rim = midR(th), dist = d.length();
            const Vec2 radial = dist > 1e-6 ? d * (1 / dist) : Vec2(1, 0);
            const bool grazing = !std::getenv("RT_PLAN_NOGRAZE") &&
                                 std::fabs(cross(dirLine, Vec2(-radial.y, radial.x))) < 0.35 &&
                                 rim - dist < B.arterialWidth / 2 + B.sidewalk + cur.width / 2 + B.sidewalk;
            if (dist <= rim + overshoot && !grazing) cur.pts.push_back(warped(p));
            else flush();
        }
        flush();
    };
    for (const auto& [s, i] : posU) gridLine(s, true, std::abs(i));
    for (const auto& [s, i] : posV) gridLine(s, false, std::abs(i));
    // --- midtown's rim: a boulevard the grid Ts into ---
    auto ringAt = [&](std::function<Real(Real)> radius, RoadClass k, Real w) {
        Polyline ring; ring.klass = k; ring.width = w; ring.closed = true;
        const Real r0 = radius(0);
        const int n = std::max(24, static_cast<int>(2 * kPi * r0 / 15.0));
        for (int i = 0; i < n; ++i) {
            const Real th = 2 * kPi * i / n;
            const Real r = radius(th);
            ring.pts.push_back(warped(B.center + Vec2(std::cos(th), std::sin(th)) * r));
        }
        return ring;
    };
    roads.push_back(ringAt(midR, RoadClass::Arterial, B.arterialWidth));
    // --- outskirts: rings (collectors), spokes (arterials), wedge locals ---
    // The freeway runs between two FRONTAGE roads (collectors hugging its right-of-way).
    // No local street crosses that corridor, so the blocks either side stay whole — with
    // locals running across it, every block in the freeway's band straddled the freeway
    // and the whole ring of land was right-of-way.
    struct BandEdge { Real base; std::function<Real(Real)> r; bool corridorInner; };
    std::vector<BandEdge> bandEdges{{B.midRadius, midR, false}};
    if (haveFreeway) {
        auto inner = [&, freewayR, corridorHalf](Real th) { return freewayR(th) - corridorHalf; };
        auto outer = [&, freewayR, corridorHalf](Real th) { return freewayR(th) + corridorHalf; };
        roads.push_back(ringAt(inner, RoadClass::Collector, B.collectorWidth));
        roads.push_back(ringAt(outer, RoadClass::Collector, B.collectorWidth));
        bandEdges.push_back({B.freewayRadius - corridorHalf, inner, true});
        bandEdges.push_back({B.freewayRadius + corridorHalf, outer, false});
    }
    for (int k = 1; ; ++k) {
        const Real rk = B.midRadius + k * B.ringSpacing;
        if (rk > outerR) break;
        auto rf = [&, rk, k](Real th) { return rk + B.curvature * ringNoise(th, k, B.seed); };
        // Keep a ring clear of the corridor and its frontage roads (the frontage roads ARE its
        // rings there) — at every angle: both wobble, so bases 100 m apart can still touch.
        if (haveFreeway) {
            const Real want = B.collectorWidth + 2 * B.sidewalk + 10;
            bool touches = false;
            for (int i = 0; i < 72 && !touches; ++i) {
                const Real th = 2 * kPi * i / 72;
                touches = std::fabs(rf(th) - (freewayR(th) - corridorHalf)) < want ||
                          std::fabs(rf(th) - (freewayR(th) + corridorHalf)) < want;
            }
            if (touches) continue;
        }
        roads.push_back(ringAt(rf, RoadClass::Collector, B.collectorWidth));
        bandEdges.push_back({rk, rf, false});
    }
    std::sort(bandEdges.begin(), bandEdges.end(), [](const BandEdge& a, const BandEdge& b) { return a.base < b.base; });
    std::vector<Real> spokeTheta;
    std::vector<std::vector<Vec2>> spokeLines;   // a spoke MEANDERS: its angle is not where it is
    for (int s = 0; s < B.spokes; ++s) {
        const Real th0 = 2 * kPi * (s + 0.5 * ((hash3(s, 3, B.seed) & 0xFF) / 255.0 - 0.5)) / B.spokes;
        spokeTheta.push_back(th0);
        Polyline sp; sp.klass = RoadClass::Arterial; sp.width = B.arterialWidth;
        for (Real r = midR(th0) - 8.0; r <= outerR + 8.0; r += 15.0) {
            const Real th = th0 + (B.curvature / std::max(Real(200), r)) * valueNoise(r / 180.0, s * 3.1, B.seed + 41);
            sp.pts.push_back(warped(B.center + Vec2(std::cos(th), std::sin(th)) * r));
        }
        spokeLines.push_back(sp.pts);
        roads.push_back(sp);
    }
    std::sort(spokeTheta.begin(), spokeTheta.end());
    // Wedge locals: short radial streets across each band, staggered band to band.
    for (std::size_t bi = 0; bi + 1 < bandEdges.size(); ++bi) {
        if (bandEdges[bi].corridorInner) continue;   // the freeway corridor: no local crosses it
        const auto& inner = bandEdges[bi].r;
        const auto& outer = bandEdges[bi + 1].r;
        const Real rMid = 0.5 * (inner(0) + outer(0));
        const Real dTheta = B.wedgeStreetSpacing / std::max(Real(100), rMid);
        const Real stagger = (bi % 2) * 0.5 * dTheta;
        for (Real th = stagger; th < 2 * kPi; th += dTheta) {
            bool nearSpoke = false;
            for (Real st : spokeTheta) {
                Real d = std::fabs(std::remainder(th - st, 2 * kPi));
                if (d * rMid < 0.45 * B.wedgeStreetSpacing) nearSpoke = true;
            }
            if (nearSpoke) continue;
            Polyline ls; ls.klass = RoadClass::Local; ls.width = B.localWidth;
            const Real r0 = inner(th) - 8.0, r1 = outer(th) + 8.0;
            for (int i = 0; i <= 8; ++i) {
                const Real r = r0 + (r1 - r0) * i / 8.0;
                const Real bend = (B.curvature * 0.4 / rMid) * std::sin(kPi * i / 8.0) * valueNoise(th * 3.0, bi * 1.7, B.seed + 7);
                ls.pts.push_back(warped(B.center + Vec2(std::cos(th + bend), std::sin(th + bend)) * r));
            }
            // ...and the spoke it is parallel to may have MEANDERED into it since: both are
            // radial, so a spoke that wandered 40 m sideways is not a street's neighbour, it is
            // the same street twice. Measured against the spoke's line, not against its angle.
            const Real keep = B.arterialWidth / 2 + B.localWidth / 2 + 2 * B.sidewalk + 6;
            bool onSpoke = false;
            for (const std::vector<Vec2>& sl : spokeLines)
                for (const Vec2& q : ls.pts)
                    if (distToPolyline(q, sl, false) < keep) { onSpoke = true; break; }
            if (onSpoke) continue;
            roads.push_back(ls);
        }
    }
    // --- freeway: a ring and radial spurs toward downtown ---
    std::vector<Polyline> fw;
    {
        Polyline ring; ring.klass = RoadClass::Freeway; ring.width = B.freewayWidth; ring.closed = true;
        const int n = static_cast<int>(2 * kPi * B.freewayRadius / 20.0);
        for (int i = 0; i < n; ++i) {
            const Real th = 2 * kPi * i / n;
            ring.pts.push_back(B.center + Vec2(std::cos(th), std::sin(th)) * freewayR(th));
        }
        fw.push_back(ring);
        for (int i = 0; i < B.freewayRadials; ++i) {
            // Midway between two spokes, so the spur never runs down an arterial.
            Real th0 = 2 * kPi * (i + 0.25) / std::max(1, B.freewayRadials);
            if (!spokeTheta.empty()) {
                Real best = 1e9, pick = th0;
                for (std::size_t s = 0; s < spokeTheta.size(); ++s) {
                    const Real a = spokeTheta[s], b = spokeTheta[(s + 1) % spokeTheta.size()] + (s + 1 == spokeTheta.size() ? 2 * kPi : 0);
                    const Real mid = 0.5 * (a + b);
                    const Real d = std::fabs(std::remainder(mid - th0, 2 * kPi));
                    if (d < best) { best = d; pick = mid; }
                }
                th0 = pick;
            }
            Polyline spur; spur.klass = RoadClass::Freeway; spur.width = B.freewayWidth;
            const Real rEnd = midR(th0);
            for (Real r = B.freewayRadius + B.freewayWobble; r >= rEnd; r -= 20.0)
                spur.pts.push_back(B.center + Vec2(std::cos(th0), std::sin(th0)) * r);
            spur.pts.push_back(warped(B.center + Vec2(std::cos(th0), std::sin(th0)) * rEnd));
            fw.push_back(spur);
            plan.interchanges.push_back(spur.pts.back());   // the spur lands on midtown's boulevard
        }
    }
    plan.freeway = planarizePolylines(fw);
    // NO STREET RUNS DOWN THE FREEWAY'S RIGHT-OF-WAY. The ring already has its two frontage
    // roads and nothing crosses between them, but a radial SPUR had no such rule: ring roads
    // and wedge streets ran inside its right-of-way for hundreds of metres, which builds as a
    // street paved over a motorway. Crossing it is untouched — that is a bridge.
    {
        // Just the carriageway and its shoulder — the frontage roads are DESIGNED to run
        // beside the freeway at the corridor's edge, and a keep-out wide enough to catch a
        // ring road caught them too (and took a third of the city's buildings with them).
        const Real rowHalf = B.freewayWidth * 0.5 + 6.0;
        std::vector<Polyline> kept;
        for (const Polyline& p : roads) {
            const Real half = p.width / 2 + B.sidewalk;
            Polyline cur = p;
            cur.pts.clear();
            cur.closed = false;
            bool split = false;
            for (std::size_t i = 0; i < p.pts.size(); ++i) {
                const Vec2 q = p.pts[i];
                const Vec2 dir = i + 1 < p.pts.size() ? p.pts[i + 1] - q : q - p.pts[i - 1];
                const Vec2 mine = dir.lengthSquared() > 1e-12 ? normalize(dir) : Vec2(1, 0);
                bool inROW = false;
                for (const Polyline& f : fw) {
                    Vec2 theirs(1, 0);
                    bool pastEnd = false;
                    if (distToPolylineDir(q, f.pts, &theirs, &pastEnd) < rowHalf + half && !pastEnd &&
                        sharesLine(mine, theirs)) { inROW = true; break; }
                }
                if (inROW) {
                    if (cur.pts.size() >= 2) { kept.push_back(cur); split = true; }
                    cur.pts.clear();
                } else {
                    cur.pts.push_back(q);
                }
            }
            if (cur.pts.size() >= 2) kept.push_back(cur);
            else if (!split && cur.pts.size() == p.pts.size()) kept.push_back(p);
        }
        roads.swap(kept);
    }
    // --- the street graph ---
    plan.streets = planarizePolylines(roads);
    collapseShortLinks(plan.streets, B.sidewalk);
    pruneStubs(plan.streets, 45.0);
    // Interchanges: where the ring crosses an arterial (a spoke).
    for (const RoadEdge& e : plan.streets.edges) {
        if (e.klass != RoadClass::Arterial) continue;
        const Vec2 a = plan.streets.nodes[static_cast<std::size_t>(e.a)].pos, b = plan.streets.nodes[static_cast<std::size_t>(e.b)].pos;
        const Real ra = (a - B.center).length(), rb = (b - B.center).length();
        const Real thA = std::atan2(a.y - B.center.y, a.x - B.center.x);
        const Real rf = B.freewayRadius + B.freewayWobble * ringNoise(thA, 777, B.seed);
        if ((ra - rf) * (rb - rf) <= 0) {
            const Real t = std::fabs(ra - rb) > 1e-9 ? (rf - ra) / (rb - ra) : 0.5;
            const Vec2 p = a + (b - a) * t;
            bool dup = false;
            for (const Vec2& q : plan.interchanges) if ((q - p).length() < 150.0) dup = true;
            if (!dup) plan.interchanges.push_back(p);
        }
    }
    // --- blocks: faces of the street graph, inset by each street's half-width + sidewalk ---
    std::map<std::pair<long long, long long>, Real> widthOf;
    auto q = [](const Vec2& p) { return (static_cast<long long>(std::llround(p.x * 10)) << 32) ^ static_cast<uint32_t>(std::llround(p.y * 10)); };
    for (const RoadEdge& e : plan.streets.edges) {
        const long long ka = q(plan.streets.nodes[static_cast<std::size_t>(e.a)].pos), kb = q(plan.streets.nodes[static_cast<std::size_t>(e.b)].pos);
        widthOf[{std::min(ka, kb), std::max(ka, kb)}] = e.width;
    }
    for (Poly2 rawFace : extractBlocks(plan.streets, 300.0)) {
        ensureCCW(rawFace);
        const std::size_t nr = rawFace.size();
        // Each raw edge's street width (the face walks graph edges node to node).
        std::vector<Real> rawW(nr, B.localWidth);
        for (std::size_t i = 0; i < nr; ++i) {
            const long long ka = q(rawFace[i]), kb = q(rawFace[(i + 1) % nr]);
            auto it = widthOf.find({std::min(ka, kb), std::max(ka, kb)});
            if (it != widthOf.end()) rawW[i] = it->second;
        }
        // THE BLOCK'S REAL CORNERS. A face carries every 15 m sample of its streets and the
        // slivers a junction leaves; offsetting that dense outline folded little loops into
        // every corner (rectangularity read 0.74 on a plainly rectangular block). Simplify
        // the ring first, keeping a corner wherever the STREET changes too, and carry each
        // street's width onto the simplified edge it becomes.
        std::size_t start = 0;
        for (std::size_t i = 1; i < nr; ++i) if (rawFace[i].x < rawFace[start].x) start = i;   // an extreme point is a true corner
        Poly2 open;
        std::vector<Real> openW;
        for (std::size_t k = 0; k <= nr; ++k) { open.push_back(rawFace[(start + k) % nr]); openW.push_back(rawW[(start + k) % nr]); }
        std::vector<std::size_t> keep = dpKeep(open, 1.0);
        {   // also keep every vertex where the street width changes
            std::set<std::size_t> ks(keep.begin(), keep.end());
            for (std::size_t k = 1; k < nr; ++k) if (std::fabs(openW[k] - openW[k - 1]) > 0.1) ks.insert(k);
            keep.assign(ks.begin(), ks.end());
        }
        Poly2 face;
        std::vector<Real> out;
        for (std::size_t k = 0; k + 1 < keep.size(); ++k) {
            face.push_back(open[keep[k]]);
            out.push_back(-(openW[keep[k]] * 0.5 + B.sidewalk));
        }
        if (face.size() < 3) continue;
        Real mean = 0;
        for (Real o : out) mean += -o;
        mean /= static_cast<Real>(out.size());
        Poly2 bld = offsetPolygonEdges(face, out);
        const Real fa = area(face);
        if (bld.size() < 3 || signedArea(bld) <= 0 || area(bld) >= fa || area(bld) < 0.1 * fa) bld = inset(face, mean);
        if (bld.size() < 3 || area(bld) < 150.0) continue;
        PlanBlock pb;
        pb.face = std::move(face);
        pb.buildable = std::move(bld);
        const Real r = (centroid(pb.buildable) - B.center).length();
        pb.district = r < B.coreRadius ? 0 : r < B.midRadius ? 1 : 2;
        plan.blocks.push_back(std::move(pb));
    }
    return plan;
}

// ---- evaluate -------------------------------------------------------------------------

std::vector<PlanChain> chainsOf(const RoadGraph& g) {
    std::vector<std::vector<std::pair<int, int>>> adj(g.nodes.size());
    for (std::size_t e = 0; e < g.edges.size(); ++e) {
        adj[static_cast<std::size_t>(g.edges[e].a)].push_back({static_cast<int>(e), g.edges[e].b});
        adj[static_cast<std::size_t>(g.edges[e].b)].push_back({static_cast<int>(e), g.edges[e].a});
    }
    auto sameRoad = [&](int e1, int e2) {
        return g.edges[static_cast<std::size_t>(e1)].klass == g.edges[static_cast<std::size_t>(e2)].klass &&
               std::fabs(g.edges[static_cast<std::size_t>(e1)].width - g.edges[static_cast<std::size_t>(e2)].width) < 0.01;
    };
    auto through = [&](int n, int viaEdge, int& nextEdge) {
        const auto& at = adj[static_cast<std::size_t>(n)];
        if (at.size() != 2) return false;
        const int other = at[0].first == viaEdge ? at[1].first : at[0].first;
        if (!sameRoad(viaEdge, other)) return false;
        nextEdge = other;
        return true;
    };
    std::vector<bool> used(g.edges.size(), false);
    std::vector<PlanChain> out;
    auto walk = [&](int startEdge, int startNode) {
        PlanChain c;
        c.klass = g.edges[static_cast<std::size_t>(startEdge)].klass;
        c.width = g.edges[static_cast<std::size_t>(startEdge)].width;
        int e = startEdge, n = startNode;
        c.pts.push_back(g.nodes[static_cast<std::size_t>(n)].pos);
        c.nodes.push_back(n);
        while (true) {
            used[static_cast<std::size_t>(e)] = true;
            const int far = g.edges[static_cast<std::size_t>(e)].a == n ? g.edges[static_cast<std::size_t>(e)].b
                                                                       : g.edges[static_cast<std::size_t>(e)].a;
            c.pts.push_back(g.nodes[static_cast<std::size_t>(far)].pos);
            c.nodes.push_back(far);
            int next = -1;
            if (!through(far, e, next) || used[static_cast<std::size_t>(next)]) break;
            e = next;
            n = far;
        }
        if (c.pts.size() >= 2) out.push_back(std::move(c));
    };
    for (std::size_t n = 0; n < g.nodes.size(); ++n) {
        if (adj[n].size() == 2) continue;   // junctions and dead ends start a chain
        for (const auto& [e, other] : adj[n]) { (void)other; if (!used[static_cast<std::size_t>(e)]) walk(e, static_cast<int>(n)); }
    }
    for (std::size_t n = 0; n < g.nodes.size(); ++n)   // what is left is a closed loop
        for (const auto& [e, other] : adj[n]) { (void)other; if (!used[static_cast<std::size_t>(e)]) walk(e, static_cast<int>(n)); }
    return out;
}

PlanScore evaluatePlan(CityPlan& plan) {
    const Brief& B = plan.brief;
    PlanScore s;
    // Freeway geometry for right-of-way tests.
    std::vector<std::vector<Vec2>> fwLines;
    for (const RoadEdge& e : plan.freeway.edges)
        fwLines.push_back({plan.freeway.nodes[static_cast<std::size_t>(e.a)].pos, plan.freeway.nodes[static_cast<std::size_t>(e.b)].pos});
    const Real rowHalf = B.freewayWidth * 0.5 + 25.0;
    Real gridAll = 0, gridRect = 0;
    Real rectArea = 0, allArea = 0, coreRect = 0, coreAll = 0;
    int lotsInLotBlocks = 0;
    for (PlanBlock& b : plan.blocks) {
        b.area = area(b.buildable);
        const OBB2 ob = orientedBoundingBox(b.buildable);
        const Real obbArea = 4 * ob.half[0] * ob.half[1];
        b.rectangularity = obbArea > 0 ? b.area / obbArea : 0;
        b.narrow = 2 * std::min(ob.half[0], ob.half[1]);
        b.edges = static_cast<int>(dpKeep(b.buildable, 1.5).size());
        // On the freeway: any buildable vertex (or the centroid) inside its right-of-way.
        bool onFreeway = false;
        const Vec2 c = centroid(b.buildable);
        for (const auto& L : fwLines) {
            if (distToPolyline(c, L, false) < rowHalf) { onFreeway = true; break; }
            for (const Vec2& v : b.buildable) if (distToPolyline(v, L, false) < rowHalf * 0.6) { onFreeway = true; break; }
            if (onFreeway) break;
        }
        // A dry-run of the engine's parcel walk at the district grain (the lot pass's).
        ParcelParams pp;
        if (b.district == 0) { pp.frontWidth = 45; pp.lotDepth = 50; pp.targetArea = 2400; }
        else if (b.district == 1) { pp.frontWidth = 22; pp.lotDepth = 36; pp.targetArea = 700; }
        else { pp.frontWidth = 18; pp.lotDepth = 27; pp.targetArea = 400; }
        pp.seed = static_cast<uint32_t>(std::llround(c.x * 3.1 + c.y * 7.7)) ^ B.seed;
        int lots = 0;
        for (const Lot& L : subdivideBlock(b.buildable, pp, b.district))
            if (!L.court && area(L.footprint) >= pp.minArea) ++lots;
        b.lots = lots;
        if (onFreeway) b.use = BlockUse::RightOfWay;
        else if (b.area < 700.0 || b.narrow < 16.0) b.use = BlockUse::Park;
        else if (lots <= 2) b.use = BlockUse::Landmark;
        else b.use = BlockUse::Lots;
        b.predictedBuildings = b.use == BlockUse::Lots ? static_cast<int>(std::lround(lots * 0.9))
                               : b.use == BlockUse::Landmark ? 1 : 0;
        ++s.blocks;
        switch (b.use) {
            case BlockUse::Lots: ++s.lotBlocks; lotsInLotBlocks += lots; break;
            case BlockUse::Landmark: ++s.landmarkBlocks; break;
            case BlockUse::Park: ++s.parkBlocks; break;
            case BlockUse::RightOfWay: ++s.rowBlocks; break;
        }
        s.predictedBuildings += b.predictedBuildings;
        s.lots += b.use == BlockUse::Lots ? lots : 0;
        if (b.use != BlockUse::RightOfWay) {
            allArea += b.area;
            if (b.rectangularity >= 0.85) rectArea += b.area;
            if (b.district <= 1) { coreAll += b.area; if (b.rectangularity >= 0.85) coreRect += b.area; }
            if (b.district == 0) { gridAll += b.area; if (b.rectangularity >= 0.85) gridRect += b.area; }
        }
    }
    s.rectilinearShare = allArea > 0 ? rectArea / allArea : 0;
    s.coreRectilinearShare = coreAll > 0 ? coreRect / coreAll : 0;
    s.gridRectilinearShare = gridAll > 0 ? gridRect / gridAll : 0;
    s.meanLotsPerLotBlock = s.lotBlocks ? static_cast<Real>(lotsInLotBlocks) / s.lotBlocks : 0;
    s.streetKm = polylineLength(plan.streets) / 1000.0;
    s.freewayKm = polylineLength(plan.freeway) / 1000.0;
    // Street connectivity (undirected components that carry an edge).
    {
        std::vector<int> parent(plan.streets.nodes.size());
        for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = static_cast<int>(i);
        std::function<int(int)> find = [&](int a) { while (parent[a] != a) a = parent[a] = parent[parent[a]]; return a; };
        for (const RoadEdge& e : plan.streets.edges) parent[find(e.a)] = find(e.b);
        std::set<int> roots;
        for (const RoadEdge& e : plan.streets.edges) roots.insert(find(e.a));
        s.streetComponents = static_cast<int>(roots.size());
    }
    // CORRIDORS THAT OVERLAP: two ROADS planned so close that one is paved over the other
    // (what the lanes builder reports as a lane that does not own its footprint).
    //
    // The measure is a RUN, not a point. Two roads that CROSS are inside each other's
    // corridor for a few metres and that is a junction (or, over the freeway, a bridge);
    // two roads drawn along the same line are inside it for their whole length, and that is
    // the fault. Chain to chain, because consecutive edges of one curve are always within a
    // lane of each other, and away from any junction the two roads share.
    {
        struct Road { std::vector<Vec2> pts; std::set<int> nodes; Real half; Vec2 lo, hi; RoadClass klass; };
        std::vector<Road> roads;
        auto addChains = [&](const RoadGraph& g, Real sidewalk, int nodeBase) {
            for (const PlanChain& c : chainsOf(g)) {
                Road r;
                r.pts = c.pts;
                for (int n : c.nodes) r.nodes.insert(nodeBase + n);
                r.half = c.width / 2 + sidewalk;
                r.klass = c.klass;
                r.lo = r.hi = c.pts.front();
                for (const Vec2& p : c.pts) {
                    r.lo = Vec2(std::min(r.lo.x, p.x), std::min(r.lo.y, p.y));
                    r.hi = Vec2(std::max(r.hi.x, p.x), std::max(r.hi.y, p.y));
                }
                roads.push_back(std::move(r));
            }
        };
        addChains(plan.streets, plan.brief.sidewalk, 0);
        addChains(plan.freeway, 0, static_cast<int>(plan.streets.nodes.size()));
        constexpr Real kStep = 5;        // sample along the road
        constexpr Real kRun = 25;        // a run this long is two roads sharing a line
        for (std::size_t i = 0; i < roads.size(); ++i)
            for (std::size_t j = i + 1; j < roads.size(); ++j) {
                const Road& A = roads[i];
                const Road& B = roads[j];
                const Real want = A.half + B.half;
                if (A.lo.x - want > B.hi.x || B.lo.x - want > A.hi.x) continue;   // boxes apart
                if (A.lo.y - want > B.hi.y || B.lo.y - want > A.hi.y) continue;
                std::vector<Vec2> shared;
                for (int n : A.nodes)
                    if (B.nodes.count(n)) {
                        const bool street = n < static_cast<int>(plan.streets.nodes.size());
                        const RoadGraph& g = street ? plan.streets : plan.freeway;
                        const int idx = street ? n : n - static_cast<int>(plan.streets.nodes.size());
                        shared.push_back(g.nodes[static_cast<std::size_t>(idx)].pos);
                    }
                const Real junctionReach = want * 2;
                Real run = 0, best = 0, worst = 0;
                for (std::size_t p = 0; p + 1 < A.pts.size(); ++p) {
                    const Vec2 a = A.pts[p], b = A.pts[p + 1];
                    const Real L = (b - a).length();
                    const int steps = std::max(1, static_cast<int>(L / kStep));
                    const Vec2 mine = L > 1e-6 ? (b - a) * (1 / L) : Vec2(1, 0);
                    for (int k = 0; k <= steps; ++k) {
                        const Vec2 q = a + (b - a) * (static_cast<Real>(k) / steps);
                        bool atJunction = false;
                        for (const Vec2& sn : shared) atJunction |= (q - sn).length() < junctionReach;
                        Vec2 theirs(1, 0);
                        bool pastEnd = false;
                        Real d = atJunction ? want : distToPolylineDir(q, B.pts, &theirs, &pastEnd);
                        if (!atJunction && (pastEnd || !sharesLine(mine, theirs))) d = want;   // a continuation, or a crossing
                        if (d < want - 0.5) {
                            run += L / steps;
                            worst = std::max(worst, want - d);
                            best = std::max(best, run);
                        } else {
                            run = 0;
                        }
                    }
                }
                if (best >= kRun) {
                    ++s.corridorOverlaps;
                    s.worstOverlap = std::max(s.worstOverlap, worst);
                    if (const char* why = std::getenv("RT_PLAN_WHY"); why && std::string(why) == "2" && worst > 30) {
                        std::printf("[plan] WORST PAIR A:");
                        for (const Vec2& q : A.pts) std::printf(" (%.1f,%.1f)", static_cast<double>(q.x), static_cast<double>(q.y));
                        std::printf("\n[plan]            B:");
                        for (const Vec2& q : B.pts) std::printf(" (%.1f,%.1f)", static_cast<double>(q.x), static_cast<double>(q.y));
                        std::printf("\n");
                    }
                    if (std::getenv("RT_PLAN_WHY")) {
                        const Vec2 at = A.pts[A.pts.size() / 2];
                        std::printf("[plan] overlap %-9s x %-9s %5.1f m deep for %4.0f m near (%.0f, %.0f) r=%.0f "
                                    "| A (%.0f,%.0f)->(%.0f,%.0f) %zu pts, B (%.0f,%.0f)->(%.0f,%.0f) %zu pts, shared %zu\n",
                                    planClassName(A.klass), planClassName(B.klass), static_cast<double>(worst),
                                    static_cast<double>(best), static_cast<double>(at.x), static_cast<double>(at.y),
                                    static_cast<double>((at - plan.brief.center).length()),
                                    static_cast<double>(A.pts.front().x), static_cast<double>(A.pts.front().y),
                                    static_cast<double>(A.pts.back().x), static_cast<double>(A.pts.back().y), A.pts.size(),
                                    static_cast<double>(B.pts.front().x), static_cast<double>(B.pts.front().y),
                                    static_cast<double>(B.pts.back().x), static_cast<double>(B.pts.back().y), B.pts.size(),
                                    shared.size());
                    }
                }
            }
    }
    if (s.corridorOverlaps)
        s.notes.push_back(std::to_string(s.corridorOverlaps) + " road pairs overlap (worst " +
                          std::to_string(static_cast<int>(std::round(s.worstOverlap))) + " m into each other)");
    // The route-choice test: commutes from the outskirts and midtown into the core, on the
    // engine's own router (with its street-junction delay), streets + freeway + interchanges.
    {
        RoadGraph all = plan.streets;
        const int off = static_cast<int>(all.nodes.size());
        for (const RoadNode& n : plan.freeway.nodes) all.nodes.push_back(n);
        for (RoadEdge e : plan.freeway.edges) { e.a += off; e.b += off; e.layer = 1; all.edges.push_back(e); }
        auto nearest = [&](const Vec2& p, int from, int to, bool arterialOnly) {
            int best = -1; Real bd = 1e30;
            for (int i = from; i < to; ++i) {
                const Real d = (all.nodes[static_cast<std::size_t>(i)].pos - p).length();
                if (d < bd) { bd = d; best = i; }
            }
            (void)arterialOnly;
            return best;
        };
        for (const Vec2& p : plan.interchanges) {
            const int fn = nearest(p, off, static_cast<int>(all.nodes.size()), false);
            const int sn = nearest(p, 0, off, true);
            if (fn >= 0 && sn >= 0) {
                RoadEdge r{fn, sn, 8.0, RoadClass::Ramp};
                all.edges.push_back(r);
            }
        }
        NavBuildParams np;
        np.oneWayRamps = false;
        const NavGraph nav = buildNavGraph(all, np);
        std::vector<int> homes, jobs;
        for (std::size_t i = 0; i < plan.blocks.size(); ++i) {
            const PlanBlock& b = plan.blocks[i];
            if (b.use == BlockUse::RightOfWay || b.use == BlockUse::Park) continue;
            if (b.district == 0) jobs.push_back(static_cast<int>(i));
            else homes.push_back(static_cast<int>(i));
        }
        int via = 0, routed = 0;
        if (!homes.empty() && !jobs.empty() && nav.nodeCount() > 1) {
            for (int k = 0; k < 300; ++k) {
                const uint32_t h = hash3(k, 91, B.seed);
                const PlanBlock& hb = plan.blocks[static_cast<std::size_t>(homes[h % homes.size()])];
                const PlanBlock& jb = plan.blocks[static_cast<std::size_t>(jobs[(h >> 12) % jobs.size()])];
                const int hn = nav.nearestNode(centroid(hb.buildable)), jn = nav.nearestNode(centroid(jb.buildable));
                if (hn < 0 || jn < 0 || hn == jn) continue;
                const Route r = findRoute(nav, hn, jn);
                if (!r.valid()) continue;
                ++routed;
                for (int li : r.links)
                    if (nav.links[static_cast<std::size_t>(li)].klass == RoadClass::Freeway) { ++via; break; }
            }
        }
        s.commutesSampled = routed;
        s.freewayCommuteShare = routed ? static_cast<Real>(via) / routed : 0;
    }
    if (s.streetComponents != 1) s.notes.push_back("the street network is in " + std::to_string(s.streetComponents) + " pieces");
    if (s.coreRectilinearShare < 0.6) s.notes.push_back("the core and midtown are less than 60% rectilinear by area");
    return s;
}

// ---- output --------------------------------------------------------------------------------
nlohmann::json planToJson(const CityPlan& plan, const PlanScore& s) {
    nlohmann::json j;
    j["brief"] = briefToJson(plan.brief);
    auto graphJson = [](const RoadGraph& g) {
        nlohmann::json nodes = nlohmann::json::array(), edges = nlohmann::json::array();
        for (const RoadNode& n : g.nodes) nodes.push_back({n.pos.x, n.pos.y});
        for (const RoadEdge& e : g.edges) {
            const char* k = e.klass == RoadClass::Freeway ? "freeway" : e.klass == RoadClass::Arterial ? "arterial"
                          : e.klass == RoadClass::Collector ? "collector" : e.klass == RoadClass::Ramp ? "ramp" : "local";
            edges.push_back({{"a", e.a}, {"b", e.b}, {"class", k}, {"width", e.width}});
        }
        return nlohmann::json{{"nodes", nodes}, {"edges", edges}};
    };
    j["streets"] = graphJson(plan.streets);
    j["freeway"] = graphJson(plan.freeway);
    nlohmann::json ic = nlohmann::json::array();
    for (const Vec2& p : plan.interchanges) ic.push_back({p.x, p.y});
    j["interchanges"] = ic;
    nlohmann::json blocks = nlohmann::json::array();
    for (const PlanBlock& b : plan.blocks) {
        nlohmann::json poly = nlohmann::json::array();
        for (const Vec2& v : b.buildable) poly.push_back({std::round(v.x * 10) / 10, std::round(v.y * 10) / 10});
        blocks.push_back({{"polygon", poly}, {"district", b.district == 0 ? "core" : b.district == 1 ? "midtown" : "outskirts"},
                          {"use", blockUseName(b.use)}, {"area", std::round(b.area)}, {"rectangularity", std::round(b.rectangularity * 100) / 100},
                          {"narrow", std::round(b.narrow * 10) / 10}, {"lots", b.lots}, {"buildings", b.predictedBuildings}});
    }
    j["blocks"] = blocks;
    j["score"] = {{"blocks", s.blocks}, {"lotBlocks", s.lotBlocks}, {"landmarkBlocks", s.landmarkBlocks}, {"parkBlocks", s.parkBlocks},
                  {"rightOfWayBlocks", s.rowBlocks}, {"predictedBuildings", s.predictedBuildings}, {"lots", s.lots},
                  {"rectilinearShare", s.rectilinearShare}, {"coreRectilinearShare", s.coreRectilinearShare}, {"gridRectilinearShare", s.gridRectilinearShare},
                  {"meanLotsPerLotBlock", s.meanLotsPerLotBlock}, {"streetComponents", s.streetComponents},
                  {"commutesSampled", s.commutesSampled}, {"freewayCommuteShare", s.freewayCommuteShare},
                  {"streetKm", s.streetKm}, {"freewayKm", s.freewayKm},
                  {"corridorOverlaps", s.corridorOverlaps}, {"worstOverlap", s.worstOverlap}, {"notes", s.notes}};
    return j;
}

std::string planToSvg(const CityPlan& plan, const PlanScore& s) {
    const Brief& B = plan.brief;
    const Real half = B.size * 0.5;
    const Real panel = B.size * 0.36;
    std::ostringstream o;
    o.setf(std::ios::fixed); o.precision(1);
    const Real x0 = B.center.x - half, y0 = B.center.y - half;
    o << "<svg xmlns='http://www.w3.org/2000/svg' viewBox='" << x0 << " " << y0 << " " << (B.size + panel) << " " << B.size
      << "' width='" << (B.size + panel) * 0.5 << "' height='" << B.size * 0.5 << "' font-family='Helvetica, Arial, sans-serif'>\n";
    o << "<rect x='" << x0 << "' y='" << y0 << "' width='" << B.size + panel << "' height='" << B.size << "' fill='#f4f1ea'/>\n";
    auto poly = [&](const Poly2& p, const char* fill, const char* stroke, Real sw) {
        o << "<polygon points='";
        for (const Vec2& v : p) o << v.x << "," << v.y << " ";
        o << "' fill='" << fill << "' stroke='" << stroke << "' stroke-width='" << sw << "'/>\n";
    };
    for (const PlanBlock& b : plan.blocks) {
        const char* fill = "#e3dcc0";
        if (b.use == BlockUse::RightOfWay) fill = "#e2e2e2";
        else if (b.use == BlockUse::Park) fill = "#b9d4a2";
        else if (b.use == BlockUse::Landmark) fill = "#b0896a";
        else fill = b.district == 0 ? "#c9b79c" : b.district == 1 ? "#d8c9a8" : "#e6dfc4";
        poly(b.buildable, fill, "#8a7d6b", 1.2);
    }
    auto roadsSvg = [&](const RoadGraph& g, bool fwy) {
        for (const RoadEdge& e : g.edges) {
            const Vec2 a = g.nodes[static_cast<std::size_t>(e.a)].pos, b = g.nodes[static_cast<std::size_t>(e.b)].pos;
            const char* col = fwy ? "#262626" : e.klass == RoadClass::Arterial ? "#6a6a6a" : e.klass == RoadClass::Collector ? "#858585" : "#a3a3a3";
            o << "<line x1='" << a.x << "' y1='" << a.y << "' x2='" << b.x << "' y2='" << b.y << "' stroke='" << col
              << "' stroke-width='" << e.width << "' stroke-linecap='round'/>\n";
        }
    };
    roadsSvg(plan.streets, false);
    roadsSvg(plan.freeway, true);
    for (const Vec2& p : plan.interchanges)
        o << "<circle cx='" << p.x << "' cy='" << p.y << "' r='30' fill='none' stroke='#e07a1f' stroke-width='8'/>\n";
    // The scorecard.
    const Real tx = B.center.x + half + 40, ty = y0 + 90;
    const Real fs = B.size / 60.0;
    int line = 0;
    auto text = [&](const std::string& t, bool bold = false) {
        o << "<text x='" << tx << "' y='" << ty + line * fs * 1.5 << "' font-size='" << (bold ? fs * 1.3 : fs)
          << "'" << (bold ? " font-weight='bold'" : "") << ">" << t << "</text>\n";
        ++line;
    };
    char buf[256];
    text(B.name, true);
    std::snprintf(buf, sizeof buf, "%.1f x %.1f km, grid %.0f deg, core %.0fx%.0f m", B.size / 1000, B.size / 1000, B.gridAngleDeg, B.coreBlockU, B.coreBlockV); text(buf);
    ++line;
    std::snprintf(buf, sizeof buf, "blocks %d: lots %d, landmark %d, park %d, freeway %d", s.blocks, s.lotBlocks, s.landmarkBlocks, s.parkBlocks, s.rowBlocks); text(buf);
    std::snprintf(buf, sizeof buf, "predicted buildings %d (%d lots, %.1f per block)", s.predictedBuildings, s.lots, s.meanLotsPerLotBlock); text(buf);
    std::snprintf(buf, sizeof buf, "rectilinear: %.0f%% of block area (core+midtown %.0f%%)", 100 * s.rectilinearShare, 100 * s.coreRectilinearShare); text(buf);
    std::snprintf(buf, sizeof buf, "streets %.1f km, freeway %.1f km, %zu interchanges", s.streetKm, s.freewayKm, plan.interchanges.size()); text(buf);
    std::snprintf(buf, sizeof buf, "freeway share of commutes: %.0f%% (%d sampled)", 100 * s.freewayCommuteShare, s.commutesSampled); text(buf);
    std::snprintf(buf, sizeof buf, "core grid %.0f%% rectangles; roads that share a line: %d",
                  100 * static_cast<double>(s.gridRectilinearShare), s.corridorOverlaps); text(buf);
    std::snprintf(buf, sizeof buf, "street network: %d piece%s", s.streetComponents, s.streetComponents == 1 ? "" : "s"); text(buf);
    for (const std::string& n : s.notes) text("! " + n);
    ++line;
    struct Key { const char* fill; const char* label; };
    for (const Key& k : {Key{"#c9b79c", "core lots"}, Key{"#d8c9a8", "midtown lots"}, Key{"#e6dfc4", "outskirts lots"},
                         Key{"#b0896a", "landmark (one building + plaza)"}, Key{"#b9d4a2", "park"}, Key{"#e2e2e2", "freeway right-of-way"}}) {
        o << "<rect x='" << tx << "' y='" << ty + line * fs * 1.5 - fs << "' width='" << fs * 1.4 << "' height='" << fs * 1.1
          << "' fill='" << k.fill << "' stroke='#8a7d6b'/>\n";
        o << "<text x='" << tx + fs * 2 << "' y='" << ty + line * fs * 1.5 << "' font-size='" << fs << "'>" << k.label << "</text>\n";
        ++line;
    }
    o << "</svg>\n";
    return o.str();
}

}  // namespace plan
}  // namespace engine
