#include "city_plan.h"
#include "plan_scene.h"   // the freeway both sides build: what the plan clears for the scene
#include "land_shape.h"   // a city shaped by its land (world.land.shape)
#include "../roads/lanes/road_graph_spec.h"   // stations
#include "../roads/lanes/polyline_ops.h"
#include "../roads/lanes/geom2d.h"          // constrainedTriangulation: the towns' cells     // pointAt, tangentAtStation: the loop's frame

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

// A dead end whose free end satisfies `near` goes back to its junction whatever its length:
// a street cut short by a keep-out is not a cul-de-sac anyone designed.
void pruneStubsNear(RoadGraph& g, const std::function<bool(const Vec2&)>& near) {
    for (int pass = 0; pass < 8; ++pass) {
        std::vector<std::vector<int>> inc(g.nodes.size());
        for (int e = 0; e < static_cast<int>(g.edges.size()); ++e) {
            inc[static_cast<std::size_t>(g.edges[e].a)].push_back(e);
            inc[static_cast<std::size_t>(g.edges[e].b)].push_back(e);
        }
        std::vector<char> drop(g.edges.size(), 0);
        bool any = false;
        for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v) {
            if (inc[static_cast<std::size_t>(v)].size() != 1 || !near(g.nodes[static_cast<std::size_t>(v)].pos)) continue;
            int cur = v, e = inc[static_cast<std::size_t>(v)][0];
            while (true) {
                drop[static_cast<std::size_t>(e)] = 1;
                any = true;
                const RoadEdge& E = g.edges[static_cast<std::size_t>(e)];
                const int nxt = E.a == cur ? E.b : E.a;
                if (inc[static_cast<std::size_t>(nxt)].size() != 2) break;   // reached a junction (or another end)
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

// THIN AND TINY BLOCKS (Glenn, 2026-09-23: "very close and criss crossing roads ... will make weird
// blocks"). A block is judged by its own shape — its area, and 2 * area / perimeter, about the radius
// of the largest circle that fits in it — because that is what fails: a block narrower than a street's
// half-width and sidewalk each side, plus room to build, holds nothing, and its corners are the
// shallow junctions the lanes builder handles worst. Each failing block loses one side, merging it with
// its neighbour: the longest LOCAL street segment (junction to junction) on its boundary — never an
// arterial or a collector, which carry the main streets, frontage roads and the streets under the
// freeways. Dead ends that leaves are pruned. `where` limits the pass to blocks whose centroid it
// accepts. With `dryRun` it only counts. Returns the blocks that failed (before any removal).
int simplifyThinBlocks(RoadGraph& g, Real minArea, Real minRadius, const std::function<bool(const Vec2&)>& where, bool dryRun, int* removedOut) {
    std::vector<char> alive(g.edges.size(), 1);
    auto other = [&](int e, int n) { const RoadEdge& E = g.edges[static_cast<std::size_t>(e)]; return E.a == n ? E.b : E.a; };
    int firstBad = -1, removed = 0;
    std::set<long long> hopeless;
    for (int iter = 0; iter < 2000; ++iter) {
        // the faces: at each node its live edges by angle; a face turns to the next edge clockwise
        std::vector<std::vector<std::pair<Real, int>>> at(g.nodes.size());
        for (int e = 0; e < static_cast<int>(g.edges.size()); ++e) {
            if (!alive[static_cast<std::size_t>(e)]) continue;
            const RoadEdge& E = g.edges[static_cast<std::size_t>(e)];
            if (E.a == E.b) continue;
            const Vec2 pa = g.nodes[static_cast<std::size_t>(E.a)].pos, pb = g.nodes[static_cast<std::size_t>(E.b)].pos;
            at[static_cast<std::size_t>(E.a)].push_back({std::atan2(pb.y - pa.y, pb.x - pa.x), e});
            at[static_cast<std::size_t>(E.b)].push_back({std::atan2(pa.y - pb.y, pa.x - pb.x), e});
        }
        for (auto& v : at) std::sort(v.begin(), v.end());
        std::vector<char> seen(g.edges.size() * 2, 0);
        struct Face { Real area = 0, perim = 0; Vec2 centroid; std::vector<int> edges; };
        const Face* worst = nullptr; Face worstFace; Real worstScore = 1e30; int bad = 0;
        for (int h0 = 0; h0 < static_cast<int>(g.edges.size()) * 2; ++h0) {
            if (seen[static_cast<std::size_t>(h0)] || !alive[static_cast<std::size_t>(h0 / 2)]) continue;
            Face f; Real cx = 0, cy = 0; int h = h0, guard = 0;
            while (!seen[static_cast<std::size_t>(h)] && guard++ < 100000) {
                seen[static_cast<std::size_t>(h)] = 1;
                const int e = h / 2; const RoadEdge& E = g.edges[static_cast<std::size_t>(e)];
                const int u = h % 2 ? E.b : E.a, v = h % 2 ? E.a : E.b;
                const Vec2 pu = g.nodes[static_cast<std::size_t>(u)].pos, pv = g.nodes[static_cast<std::size_t>(v)].pos;
                const Real cr = pu.x * pv.y - pv.x * pu.y;
                f.area += cr; cx += (pu.x + pv.x) * cr; cy += (pu.y + pv.y) * cr;
                f.perim += (pv - pu).length();
                f.edges.push_back(e);
                // at v, the edge just clockwise of the one we came in on
                const auto& lst = at[static_cast<std::size_t>(v)];
                std::size_t k = 0;
                for (; k < lst.size(); ++k) if (lst[k].second == e && other(e, v) == u) break;
                const int nxt = lst[(k + lst.size() - 1) % lst.size()].second;
                h = 2 * nxt + (g.edges[static_cast<std::size_t>(nxt)].a == v ? 0 : 1);
            }
            f.area *= 0.5;
            if (f.area <= 1.0) continue;   // the outer face (and degenerate ones)
            f.centroid = Vec2(cx / (6 * f.area), cy / (6 * f.area));
            if (!where(f.centroid)) continue;
            const Real radius = 2 * f.area / std::max(Real(1), f.perim);
            if (f.area >= minArea && radius >= minRadius) continue;
            long long key = 0; for (int e : f.edges) key = key * 1000003 + e;
            if (hopeless.count(key)) continue;
            ++bad;
            if (radius < worstScore) { worstScore = radius; worstFace = f; worst = &worstFace; }
        }
        if (firstBad < 0) firstBad = bad;
        if (dryRun || !worst) break;
        // the longest local segment on its boundary, junction to junction
        auto degree = [&](int n) { return at[static_cast<std::size_t>(n)].size(); };
        std::vector<int> best; Real bestLen = -1;
        std::set<int> tried;
        for (int e0 : worst->edges) {
            if (tried.count(e0) || g.edges[static_cast<std::size_t>(e0)].klass != RoadClass::Local) continue;
            std::vector<int> seg{e0}; tried.insert(e0);
            for (int dir = 0; dir < 2; ++dir) {
                int e = e0, n = dir ? g.edges[static_cast<std::size_t>(e0)].b : g.edges[static_cast<std::size_t>(e0)].a;
                while (degree(n) == 2) {
                    const auto& lst = at[static_cast<std::size_t>(n)];
                    const int nx = lst[0].second == e ? lst[1].second : lst[0].second;
                    if (g.edges[static_cast<std::size_t>(nx)].klass != RoadClass::Local || tried.count(nx)) break;
                    seg.push_back(nx); tried.insert(nx); n = other(nx, n); e = nx;
                }
            }
            Real len = 0;
            for (int e : seg) len += (g.nodes[static_cast<std::size_t>(g.edges[static_cast<std::size_t>(e)].a)].pos - g.nodes[static_cast<std::size_t>(g.edges[static_cast<std::size_t>(e)].b)].pos).length();
            if (len > bestLen) { bestLen = len; best = seg; }
        }
        if (best.empty()) { long long key = 0; for (int e : worst->edges) key = key * 1000003 + e; hopeless.insert(key); continue; }
        for (int e : best) alive[static_cast<std::size_t>(e)] = 0;
        ++removed;
        // dead ends the removal left: back to their junction, local streets only
        for (int pass = 0; pass < 8; ++pass) {
            std::vector<int> deg(g.nodes.size(), 0);
            for (std::size_t e = 0; e < g.edges.size(); ++e) if (alive[e]) { ++deg[static_cast<std::size_t>(g.edges[e].a)]; ++deg[static_cast<std::size_t>(g.edges[e].b)]; }
            bool any = false;
            for (std::size_t e = 0; e < g.edges.size(); ++e) {
                if (!alive[e] || g.edges[e].klass != RoadClass::Local) continue;
                for (int end : {g.edges[e].a, g.edges[e].b})
                    if (deg[static_cast<std::size_t>(end)] == 1 && (end == best.front() || true)) {
                        // only dead ends the pass made: next to a removed segment's junctions
                        bool near = false;
                        for (int r : best) for (int q : {g.edges[static_cast<std::size_t>(r)].a, g.edges[static_cast<std::size_t>(r)].b})
                            if ((g.nodes[static_cast<std::size_t>(q)].pos - g.nodes[static_cast<std::size_t>(end)].pos).length() < 200) near = true;
                        if (near) { alive[e] = 0; any = true; break; }
                    }
            }
            if (!any) break;
        }
    }
    if (!dryRun) {
        std::vector<RoadEdge> kept;
        for (std::size_t e = 0; e < g.edges.size(); ++e) if (alive[e]) kept.push_back(g.edges[e]);
        g.edges.swap(kept);
    }
    if (removedOut) *removedOut = removed;
    return std::max(0, firstBad);
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

// Cut roads where `blocked(point, half-width)` holds -- walked at 4 m so a cut lands within a few
// metres of the edge, not a vertex away -- keeping the pieces between longer than `minLen`.
std::vector<Polyline> cutRoadsWhere(const std::vector<Polyline>& roads, Real sidewalk,
                                    const std::function<bool(const Vec2&, Real)>& blocked, Real minLen = 30.0) {
    std::vector<Polyline> kept;
    for (const Polyline& p : roads) {
        const Real half = p.width / 2 + sidewalk;
        std::vector<Vec2> dense;
        const std::size_t n = p.pts.size(), segs = p.closed ? n : n - 1;
        for (std::size_t i = 0; n >= 2 && i < segs; ++i) {
            const Vec2 a = p.pts[i], b = p.pts[(i + 1) % n];
            const int m = std::max(1, static_cast<int>(std::ceil((b - a).length() / 4.0)));
            for (int k = 0; k < m; ++k) dense.push_back(a + (b - a) * (Real(k) / m));
        }
        if (!p.closed && n) dense.push_back(p.pts.back());
        bool any = false;
        for (const Vec2& q : dense) if (blocked(q, half)) { any = true; break; }
        if (!any) { kept.push_back(p); continue; }
        Polyline cur = p;
        cur.closed = false;
        cur.pts.clear();
        auto flush = [&] {
            if (cur.pts.size() >= 2 && roads::lanes::stations(cur.pts).back() > minLen) kept.push_back(cur);
            cur.pts.clear();
        };
        for (const Vec2& q : dense) {
            if (blocked(q, half)) flush();
            else cur.pts.push_back(q);
        }
        flush();
    }
    return kept;
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
    if (j.contains("world")) b.world = j["world"];
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
    nlohmann::json j = {{"name", b.name}, {"seed", b.seed}, {"size", b.size}, {"center", {b.center.x, b.center.y}},
            {"core", {{"radius", b.coreRadius}, {"block", {b.coreBlockU, b.coreBlockV}}, {"angle", b.gridAngleDeg},
                      {"arterialEvery", b.arterialEvery}}},
            {"midtown", {{"radius", b.midRadius}, {"block", {b.midBlockU, b.midBlockV}}, {"warp", b.warp}}},
            {"outskirts", {{"ringSpacing", b.ringSpacing}, {"spokes", b.spokes}, {"curvature", b.curvature},
                           {"streetSpacing", b.wedgeStreetSpacing}, {"margin", b.outerMargin}}},
            {"freeway", {{"radius", b.freewayRadius}, {"radials", b.freewayRadials}, {"wobble", b.freewayWobble}}},
            {"relief", b.relief},
            {"roads", {{"local", b.localWidth}, {"collector", b.collectorWidth}, {"arterial", b.arterialWidth},
                       {"freeway", b.freewayWidth}, {"sidewalk", b.sidewalk}}}};
    if (!b.world.is_null()) j["world"] = b.world;
    return j;
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
    const Real corridorHalf = B.freewayWidth * 0.5 + 35.0;
    // --- LAND (world.land, ADR-0106): a city sited on an island is shaped BY the island. Glenn: "The
    // city also needs to fit the terrain. Part of the city is in the water and another part looks
    // embedded into the mountain." So the land comes first: the city's FOOTPRINT is the buildable
    // ground connected to its centre -- dry (above seaLevel + minHeight; a river is the river step's),
    // gentle (maxSlope, measured over 40 m) and not far above the centre (maxRise: a city climbs its
    // foothills, not the mountain) -- and the plan is drawn to it:
    //   * midtown's rim follows the footprint's edge (a waterfront boulevard along the coast);
    //   * the freeway ring pulls in along the foot of the slopes, and where it meets the sea it opens
    //     into a C against the coast (plan.ringArc);
    //   * every street outside the footprint is cut (below, before the river step).
    const nlohmann::json landSpec = B.world.is_object() ? B.world.value("land", nlohmann::json()) : nlohmann::json();
    struct LandMask { Vec2 o; Real cell = 10; int n = 0; std::vector<char> dry, fit, base; };
    LandMask land;
    // SHAPED (world.land.shape): the city's limits grown over its land to a target area, and the
    // streets laid in them by depth (land_shape.h) -- not a circle trimmed to fit
    const bool shaped = landSpec.is_object() && landSpec.value("shape", false);
    LandShape shape;
    std::shared_ptr<const Hydrology> landHy;   // the world's water, for the shape's inner edges and its bridges
    constexpr int kBins = 360;
    std::vector<Real> edgeR(kBins, 1e30);       // the footprint's reach along each bearing (smoothed)
    std::vector<Real> edgeSea(kBins, 0);        // ...and how much of it ends at the water (0..1)
    if (landSpec.is_object()) {
        const HeightField g = sceneGround(B);
        const std::shared_ptr<const Hydrology> hy = worldHydrology(B);
        landHy = hy;
        const Real sea = B.world.value("seaLevel", 0.0) + landSpec.value("minHeight", 1.5);
        const Real maxSlope = landSpec.value("maxSlope", 0.08), maxRise = landSpec.value("maxRise", 50.0);
        const Real half = B.size * 0.5 + 200;
        land.o = B.center - Vec2(half, half);
        land.n = static_cast<int>(std::ceil(2 * half / land.cell)) + 1;
        const int N = land.n;
        auto at = [N](int i, int j) { return static_cast<std::size_t>(j) * static_cast<std::size_t>(N) + static_cast<std::size_t>(i); };
        std::vector<Real> h(static_cast<std::size_t>(N) * N);
        std::vector<char> river(h.size(), 0);
        for (int j = 0; j < N; ++j)
            for (int i = 0; i < N; ++i) {
                const Vec2 p(land.o.x + i * land.cell, land.o.y + j * land.cell);
                h[at(i, j)] = g(p.x, p.y);
                // a RIVER is not sea: its channel is carved below the beach, and a street across it is
                // the river step's to bridge or stop -- as are its steep banks
                river[at(i, j)] = hy && hy->distanceToRiver(p.x, p.y, 60.0) < 40.0;
            }
        const Real hc = g(B.center.x, B.center.y);
        land.dry.assign(h.size(), 0);
        land.base.assign(h.size(), 0);
        std::vector<char> ok(h.size(), 0);
        for (int j = 0; j < N; ++j)
            for (int i = 0; i < N; ++i) {
                const int i0 = std::max(0, i - 2), i1 = std::min(N - 1, i + 2), j0 = std::max(0, j - 2), j1 = std::min(N - 1, j + 2);
                const Real sx = (h[at(i1, j)] - h[at(i0, j)]) / (land.cell * std::max(1, i1 - i0));
                const Real sy = (h[at(i, j1)] - h[at(i, j0)]) / (land.cell * std::max(1, j1 - j0));
                const std::size_t k = at(i, j);
                land.dry[k] = h[k] > sea || river[k];
                land.base[k] = land.dry[k] && (river[k] || std::hypot(sx, sy) < maxSlope);
                ok[k] = land.base[k] && (river[k] || h[k] - hc < maxRise);
            }
        // the footprint: what of it is connected to the centre (the nearest fit ground, if the centre isn't)
        land.fit.assign(h.size(), 0);
        int ci = N / 2, cj = N / 2;
        for (int r = 0; r < 40 && !ok[at(ci, cj)]; ++r)
            for (int dj = -r; dj <= r; ++dj)
                for (int di = -r; di <= r; ++di)
                    if (!ok[at(ci, cj)] && ok[at(N / 2 + di, N / 2 + dj)]) { ci = N / 2 + di; cj = N / 2 + dj; }
        if (ok[at(ci, cj)]) {
            std::vector<std::pair<int, int>> stack{{ci, cj}};
            land.fit[at(ci, cj)] = 1;
            while (!stack.empty()) {
                const auto [i, j] = stack.back();
                stack.pop_back();
                const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
                for (const auto& d : nb) {
                    const int u = i + d[0], v = j + d[1];
                    if (u < 0 || v < 0 || u >= N || v >= N || land.fit[at(u, v)] || !ok[at(u, v)]) continue;
                    land.fit[at(u, v)] = 1;
                    stack.push_back({u, v});
                }
            }
        }
        // its reach along each bearing, and what stops it
        std::vector<Real> raw(kBins);
        std::vector<char> rawSea(kBins, 0);
        for (int b = 0; b < kBins; ++b) {
            const Real th = 2 * kPi * b / kBins;
            const Vec2 d(std::cos(th), std::sin(th));
            Real r = 0;
            for (; r < half - 20; r += land.cell * 0.5) {
                const Vec2 p = B.center + d * r;
                const int i = static_cast<int>(std::lround((p.x - land.o.x) / land.cell)), j = static_cast<int>(std::lround((p.y - land.o.y) / land.cell));
                if (!land.fit[at(i, j)]) {
                    // the COAST, if the water is within 200 m on: a bluff over the beach is the shore,
                    // not a mountainside to pull the ring in from
                    for (Real r2 = r; r2 < r + 200 && !rawSea[b]; r2 += land.cell) {
                        const Vec2 q = B.center + d * r2;
                        const int u = static_cast<int>(std::lround((q.x - land.o.x) / land.cell)), v = static_cast<int>(std::lround((q.y - land.o.y) / land.cell));
                        rawSea[b] = u < 0 || v < 0 || u >= N || v >= N || !land.dry[at(u, v)];
                    }
                    break;
                }
            }
            raw[b] = r;
        }
        // smoothed: the least reach within 5 degrees (a gully is not a gap), then averaged over 7
        std::vector<Real> lo(kBins);
        for (int b = 0; b < kBins; ++b) {
            Real m = 1e30;
            for (int k = -5; k <= 5; ++k) m = std::min(m, raw[static_cast<std::size_t>((b + k + kBins) % kBins)]);
            lo[static_cast<std::size_t>(b)] = m;
        }
        for (int b = 0; b < kBins; ++b) {
            Real sum = 0;
            int sea = 0;
            for (int k = -7; k <= 7; ++k) { sum += lo[static_cast<std::size_t>((b + k + kBins) % kBins)]; sea += rawSea[static_cast<std::size_t>((b + k + kBins) % kBins)]; }
            edgeR[static_cast<std::size_t>(b)] = sum / 15;
            edgeSea[static_cast<std::size_t>(b)] = sea / Real(15);
        }
        if (shaped) {
            LandShapeParams sp;
            sp.cell = land.cell;
            sp.targetArea = landSpec.value("area", kPi * B.size * B.size * 0.16);
            sp.slopeWeight = landSpec.value("slopeWeight", sp.slopeWeight);
            sp.riseWeight = landSpec.value("riseWeight", sp.riseWeight);
            sp.seaLevel = B.world.value("seaLevel", -1e9);
            sp.coastPull = landSpec.value("coastPull", sp.coastPull);
            sp.coastReach = landSpec.value("coastReach", sp.coastReach);
            sp.smooth = landSpec.value("smooth", 60.0);
            if (hy && landSpec.value("waterEdges", true)) sp.water = [hy](const Vec2& q) { return hy->isWet(q.x, q.y, 0.0); };
            auto base = [&](const Vec2& q) {
                const int i = static_cast<int>(std::lround((q.x - land.o.x) / land.cell)), j = static_cast<int>(std::lround((q.y - land.o.y) / land.cell));
                return i >= 0 && j >= 0 && i < N && j < N && land.base[at(i, j)] != 0;
            };
            shape = growLandShape(g, base, B.center, half - 20, sp);
        }
    }
    auto edgeAt = [&](Real th, Real* seaOut = nullptr) {
        const int b = ((static_cast<int>(std::lround(th / (2 * kPi) * kBins)) % kBins) + kBins) % kBins;
        if (seaOut) *seaOut = edgeSea[static_cast<std::size_t>(b)];
        return edgeR[static_cast<std::size_t>(b)];
    };
    auto landAt = [&](const Vec2& p, bool streets) {
        if (land.n == 0) return true;
        const int i = static_cast<int>(std::lround((p.x - land.o.x) / land.cell)), j = static_cast<int>(std::lround((p.y - land.o.y) / land.cell));
        if (i < 0 || j < 0 || i >= land.n || j >= land.n) return false;
        const std::size_t k = static_cast<std::size_t>(j) * static_cast<std::size_t>(land.n) + static_cast<std::size_t>(i);
        // the limits, and a street's overshoot; the city's own water is the river step's to judge
        if (streets && shaped) return shape.depthAt(p) > -12.0 || shape.isWater(p);
        return streets ? land.fit[k] != 0 : land.dry[k] != 0;
    };
    // The freeway ring and its corridor: the ring's centreline radius at an angle, and the
    // half-width out to the frontage roads either side of it. On land it keeps its outer frontage
    // road off the slopes; toward the sea it holds its radius and is cut there into a C.
    auto freewayR = [&](Real th) {
        const Real r = B.freewayRadius + B.freewayWobble * ringNoise(th, 777, B.seed);
        Real sea = 0;
        const Real e = edgeAt(th, &sea);
        const Real onLand = std::min(r, e - corridorHalf - 30);
        return onLand + (r - onLand) * smooth01((sea - 0.3) / 0.4);   // blended, so a C has no kink where coast meets foothill
    };
    auto midR = [&](Real theta) {
        const Real r = B.midRadius * (1.0 + 0.05 * ringNoise(theta, 99, B.seed));
        if (land.n == 0) return r;
        // inside the ring's corridor, and inside the footprint: along the coast, a waterfront boulevard
        Real lim = edgeAt(theta) - 40;
        if (B.freewayRadius > 0) lim = std::min(lim, freewayR(theta) - corridorHalf - 40);
        return std::max(std::min(r, lim), std::min(r, Real(150)));
    };
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
    // STREETS UNDER THE RING. Only the spokes used to cross the freeway's corridor, ten crossings
    // round a 6.4 km ring; with the system interchanges taking the stretch round each expressway,
    // the diamonds that fitted all landed on one side (Glenn: "on-ramps on one side but not the
    // other"). A collector now crosses the corridor midway between each pair of spokes, frontage
    // road to frontage road, passing under the ring — twice the places a diamond can stand, evenly
    // round it. Where one meets a system interchange, its keep-out cuts it, like any street there.
    // It runs on, like a spoke, to the ring each side of the corridor: a ramp lands on it at a
    // four-way out past the band, and a street that stopped at the frontage road left no room
    // for one — every diamond on the new crossings fell back to the frontage road and failed.
    std::vector<Real> underTheta;
    if (haveFreeway && spokeTheta.size() >= 2) {
        std::size_t ci = 0;
        while (ci < bandEdges.size() && !bandEdges[ci].corridorInner) ++ci;
        for (std::size_t sI = 0; sI < spokeTheta.size() && ci < bandEdges.size(); ++sI) {
            const Real a = spokeTheta[sI], b = spokeTheta[(sI + 1) % spokeTheta.size()] + (sI + 1 == spokeTheta.size() ? 2 * kPi : 0);
            const Real th = 0.5 * (a + b);
            Polyline c; c.klass = RoadClass::Collector; c.width = B.collectorWidth;
            const Real r0 = (ci > 0 ? bandEdges[ci - 1].r(th) : freewayR(th) - corridorHalf) - 8;
            const Real r1 = (ci + 2 < bandEdges.size() ? bandEdges[ci + 2].r(th) : freewayR(th) + corridorHalf + 120) + 8;
            const int n = std::max(2, static_cast<int>((r1 - r0) / 15.0));
            for (int i = 0; i <= n; ++i) c.pts.push_back(warped(B.center + Vec2(std::cos(th), std::sin(th)) * (r0 + (r1 - r0) * i / n)));
            roads.push_back(c);
            underTheta.push_back(th);
        }
    }
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
            const bool besideCorridor = bandEdges[bi + 1].corridorInner || (bi > 0 && bandEdges[bi - 1].corridorInner);
            for (const std::vector<Real>* at : {&spokeTheta, besideCorridor ? &underTheta : &spokeTheta})
                for (Real st : *at) {
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
    // --- SHAPED: the streets laid by the land's depth instead (land_shape.h) ---
    Real shapeRim = 0, shapeCore = 0;
    if (shaped && shape.area > 0) {
        roads.clear();
        spokeTheta.clear();
        const Real edgeDepth = landSpec.value("edgeDepth", 45.0);
        shapeRim = shape.depthHolding(landSpec.value("midShare", 0.35));
        shapeCore = shape.depthHolding(landSpec.value("coreShare", 0.12));
        // THE GRID in the deep part, in the brief's frame round the heart, block sizes growing out
        // from it as they do round a centre; each line crossing midtown's rim and stopping just past
        const Frame G{shape.heart, F.u, F.v};
        Real reach = 0;
        for (int j = 0; j < shape.n; ++j)
            for (int i = 0; i < shape.n; ++i)
                if (shape.depth[static_cast<std::size_t>(j) * shape.n + i] >= shapeRim)
                    reach = std::max(reach, (shape.origin + Vec2(i * shape.cell, j * shape.cell) - shape.heart).length());
        const Real coreR = std::sqrt(std::max(Real(1), shape.area * landSpec.value("coreShare", 0.12)) / kPi);
        for (int axis = 0; axis < 2; ++axis) {
            const bool alongV = axis == 0;
            for (const auto& [off, idx] : gridPositions(alongV ? B.coreBlockU : B.coreBlockV, alongV ? B.midBlockU : B.midBlockV, coreR, reach)) {
                const bool arterial = std::abs(idx) % std::max(1, B.arterialEvery) == 0;
                Polyline cur;
                cur.klass = arterial ? RoadClass::Arterial : RoadClass::Local;
                cur.width = arterial ? B.arterialWidth : B.localWidth;
                const Vec2 dirLine = alongV ? G.v : G.u;
                auto flush = [&] { if (cur.pts.size() >= 2) roads.push_back(cur); cur.pts.clear(); };
                for (Real t = -reach - 20; t <= reach + 20; t += 15.0) {
                    const Vec2 p = alongV ? G.toWorld(off, t) : G.toWorld(t, off);
                    const Real d = shape.depthAt(p);
                    // grazing the rim (running along its contour) it stops short, as round a centre
                    const bool grazing = std::fabs(dot(dirLine, shape.gradient(p))) < 0.35 &&
                                         d - shapeRim < B.arterialWidth / 2 + B.sidewalk + cur.width / 2 + B.sidewalk;
                    if (d >= shapeRim - 10.0 && !grazing) cur.pts.push_back(p);
                    else flush();
                }
                flush();
            }
        }
        // MIDTOWN'S RIM, a boulevard along the depth contour -- and the OUTSKIRTS' rings, contours
        // further out, evenly between it and the last one near the limits
        // smoothed along their length: the outer ones follow a ragged shore, and a street there
        // should drive as a curve, not a zigzag
        const Real smoothM = landSpec.value("streetSmooth", 90.0);
        auto contourRoads = [&](Real level, RoadClass k, Real w) {
            for (const std::vector<Vec2>& c : shape.contour(level, 15.0, 250.0, smoothM)) {
                Polyline pl; pl.klass = k; pl.width = w; pl.pts = c;
                pl.closed = c.size() > 2 && (c.front() - c.back()).length() < 30.0;
                roads.push_back(pl);
            }
        };
        contourRoads(shapeRim, RoadClass::Arterial, B.arterialWidth);
        std::vector<Real> levels{shapeRim};
        const int nRings = std::max(1, static_cast<int>(std::lround((shapeRim - edgeDepth) / std::max(Real(60), B.ringSpacing))));
        for (int k = 1; k <= nRings && shapeRim - edgeDepth > 60; ++k) {
            levels.push_back(shapeRim - (shapeRim - edgeDepth) * k / nRings);
            contourRoads(levels.back(), RoadClass::Collector, B.collectorWidth);
        }
        // SPOKES: arterials from the rim down the depth to the last ring, every spokeSpacing along the rim
        std::vector<std::vector<Vec2>> spokeLines;
        const Real spokeSpacing = landSpec.value("spokeSpacing", 480.0);
        for (const std::vector<Vec2>& rim : shape.contour(shapeRim, 15.0, 250.0, smoothM)) {
            const std::vector<double> st = roads::lanes::stations(rim);
            const int k = std::max(1, static_cast<int>(std::lround(st.back() / spokeSpacing)));
            for (int i = 0; i < k; ++i) {
                const Vec2 p = roads::lanes::pointAt(rim, st, st.back() * (i + 0.5) / k);
                Polyline sp; sp.klass = RoadClass::Arterial; sp.width = B.arterialWidth;
                sp.pts = descendDepth(shape, p + shape.gradient(p) * 8.0, levels.back() - 8.0);
                if (sp.pts.size() >= 3) { spokeLines.push_back(sp.pts); roads.push_back(sp); }
            }
        }
        // WEDGE STREETS across each band, square to its contours, staggered band to band, never
        // beside a spoke or crowding the street before it where the lines converge (an inlet's shore)
        const Real keep = B.arterialWidth / 2 + B.localWidth / 2 + 2 * B.sidewalk + 6;
        for (std::size_t bi = 0; bi + 1 < levels.size(); ++bi) {
            std::vector<std::vector<Vec2>> placed;
            for (const std::vector<Vec2>& inner : shape.contour(levels[bi], 15.0, 150.0, smoothM)) {
                const std::vector<double> st = roads::lanes::stations(inner);
                const Real stagger = (bi % 2) * 0.5 * B.wedgeStreetSpacing;
                for (Real s0 = stagger + 0.5 * B.wedgeStreetSpacing; s0 < st.back(); s0 += B.wedgeStreetSpacing) {
                    const Vec2 p = roads::lanes::pointAt(inner, st, s0);
                    std::vector<Vec2> line = descendDepth(shape, p + shape.gradient(p) * 8.0, levels[bi + 1] - 8.0);
                    if (line.size() < 3) continue;
                    bool clash = false;
                    for (const auto* set : {&spokeLines, &placed})
                        for (const std::vector<Vec2>& o : *set) {
                            for (const Vec2& q : line) if (distToPolyline(q, o, false) < keep) { clash = true; break; }
                            if (clash) break;
                        }
                    if (clash) continue;
                    Polyline ls; ls.klass = RoadClass::Local; ls.width = B.localWidth; ls.pts = line;
                    placed.push_back(line);
                    roads.push_back(ls);
                }
            }
        }
        // BRIDGES. The river is an edge of the depth, so nothing crosses it by accident: each bridge
        // is placed. Where an arterial comes down to the water (a spoke, a grid avenue), it crosses --
        // no two within minBridgeGap -- and any stretch of river in the city longer than maxBridgeGap
        // without one gets one midway. Each runs square across, from past the riverside street on one
        // bank to past it on the other; the river step (below) keeps it as a bridge.
        if (landHy && !shape.water.empty()) {
            const nlohmann::json rp = B.world.value("riverPlan", nlohmann::json::object());
            const Real minGap = rp.value("minBridgeGap", 250.0), maxGap = rp.value("maxBridgeGap", 600.0);
            std::vector<Polyline> arterialsNow;
            for (const Polyline& r : roads) if (r.klass == RoadClass::Arterial && !r.closed) arterialsNow.push_back(r);
            int bridges = 0;
            for (const River& rv : landHy->rivers()) {
                std::vector<Vec2> line;
                for (const RiverNode& nd : rv.nodes) line.push_back(nd.p);
                if (line.size() < 2) continue;
                const std::vector<double> st = roads::lanes::stations(line);
                // the river's runs inside the city (its water cells in the footprint), by station
                std::vector<std::pair<double, double>> runs;
                bool in = false;
                for (double sAt = 0; sAt <= st.back(); sAt += 10.0) {
                    const bool w = shape.isWater(roads::lanes::pointAt(line, st, sAt));
                    if (w && !in) { runs.push_back({sAt, sAt}); in = true; }
                    if (w) runs.back().second = sAt;
                    if (!w) in = false;
                }
                std::vector<double> want;
                // where arterials come down to it
                for (const Polyline& r : arterialsNow)
                    for (const Vec2& end : {r.pts.front(), r.pts.back()}) {
                        if (landHy->distanceToRiver(end.x, end.y, 200.0) > edgeDepth + 25) continue;
                        const roads::lanes::Projection pr = roads::lanes::project(line, st, end);
                        if (pr.distance < edgeDepth + 80 && shape.isWater(roads::lanes::pointAt(line, st, pr.station))) want.push_back(pr.station);
                    }
                std::sort(want.begin(), want.end());
                std::vector<double> at;
                for (double w : want) if (at.empty() || w - at.back() >= minGap) at.push_back(w);
                // and the gaps, run by run
                std::vector<double> filled;
                for (const auto& [r0, r1] : runs) {
                    if (r1 - r0 < 60.0) continue;
                    std::vector<double> e{r0};
                    for (double a2 : at) if (a2 > r0 && a2 < r1) e.push_back(a2);
                    e.push_back(r1);
                    for (std::size_t k = 0; k + 1 < e.size(); ++k) {
                        const bool end0 = k == 0, end1 = k + 2 == e.size();
                        // from a run's end (the city's edge) only half the gap counts: the first bridge
                        // stands within maxGap/2 of where the river comes in
                        const double span = e[k + 1] - e[k], limit = (end0 || end1) ? maxGap * 0.5 : maxGap;
                        if (span <= limit) continue;
                        const int extra = static_cast<int>(std::ceil(span / maxGap - ((end0 || end1) ? 0.5 : 0.0)));
                        for (int q = 1; q <= std::max(1, extra); ++q) {
                            const double f = end0 && !end1 ? 1.0 - (q - 0.5) / std::max(1, extra) * 0.999
                                           : end1 && !end0 ? (q - 0.5) / std::max(1, extra)
                                                           : static_cast<double>(q) / (std::max(1, extra) + 1);
                            filled.push_back(e[k] + span * std::clamp(f, 0.0, 1.0));
                        }
                    }
                }
                at.insert(at.end(), filled.begin(), filled.end());
                for (double sAt : at) {
                    const Vec2 P = roads::lanes::pointAt(line, st, sAt);
                    const Vec2 T = roads::lanes::pointAt(line, st, std::min(st.back(), sAt + 20)) - roads::lanes::pointAt(line, st, std::max(0.0, sAt - 20));
                    if (T.length() < 1e-6) continue;
                    const Vec2 Nn = Vec2(-T.y, T.x) * (1.0 / T.length());
                    // out each way to past the riverside contour
                    auto reach = [&](const Vec2& dir) {
                        for (Real t = 5; t <= 260; t += 5) {
                            const Vec2 q = P + dir * t;
                            if (!shape.inside(q)) return Real(-1);
                            if (!shape.isWater(q) && shape.depthAt(q) >= edgeDepth + 12) return t;
                        }
                        return Real(-1);
                    };
                    const Real t0 = reach(Nn * -1.0), t1 = reach(Nn);
                    if (t0 < 0 || t1 < 0) continue;
                    Polyline br; br.klass = RoadClass::Arterial; br.width = B.arterialWidth;
                    const int m = std::max(2, static_cast<int>(std::ceil((t0 + t1) / 15.0)));
                    for (int k = 0; k <= m; ++k) br.pts.push_back(P + Nn * (-t0 + (t0 + t1) * k / m));
                    roads.push_back(br);
                    ++bridges;
                }
            }
            if (std::getenv("RT_PLAN_WHY")) std::printf("[plan] shaped: %d bridges placed\n", bridges);
        }
        plan.limits = shape.limits;
    }
    // --- the towns at the expressways' far ends ---
    // A site: the farthest point along the expressway's line, past the city by `gap` and inside the
    // ground grid, where the whole town — `depth` along the expressway, `halfWidth` either side —
    // stands 5 m clear of the sea. The coast is why it slides: the NE expressway points at it.
    const nlohmann::json townSpec = B.world.is_null() ? nlohmann::json() : B.world.value("towns", nlohmann::json());
    const HeightField townGround = townSpec.is_null() ? HeightField() : sceneGround(B);
    const Real townDepth = townSpec.is_null() ? 0 : townSpec.value("depth", 420.0), townHalf = townSpec.is_null() ? 0 : townSpec.value("halfWidth", 240.0);
    // Past the city the expressway may BEND toward its town: straight on first, then 10 degrees at a
    // time up to 50 either side, taking the smallest bend with a dry site — the NE expressway points
    // straight at the coast. The bend is one smooth curve from where it leaves the city (`leave`)
    // to the town's gate, and the whole of it must stand clear of the sea too.
    auto siteTown = [&](const Vec2& dir, std::vector<Vec2>& head) {
        CityPlan::Town t;
        if (townSpec.is_null()) return t;
        const Real sea = B.world.value("seaLevel", -1e30), gridHalf = B.world.value("grid", B.size * 0.5 + 60);
        const Vec2 leave = B.center + dir * (outerR + 60);
        auto dry = [&](const Vec2& q, Real above) {
            return std::fabs(q.x - B.center.x) <= gridHalf - 150 && std::fabs(q.y - B.center.y) <= gridHalf - 150 && townGround(q.x, q.y) >= sea + above;
        };
        for (int k = 0; k <= 10; ++k) {
            const Real bend = (k == 0 ? 0 : (k % 2 ? 1 : -1) * ((k + 1) / 2) * 10.0) * kPi / 180.0;
            const Vec2 d(dir.x * std::cos(bend) - dir.y * std::sin(bend), dir.x * std::sin(bend) + dir.y * std::cos(bend));
            const Vec2 left(-d.y, d.x);
            for (Real r = gridHalf * Real(1.41); r >= townSpec.value("gap", 500.0); r -= 20) {
                const Vec2 gate = leave + d * r;
                bool ok = true;
                for (Real u = -150; u <= townDepth + 60 && ok; u += 40)
                    for (Real v = -townHalf - 60; v <= townHalf + 60 && ok; v += 40) ok = dry(gate + d * u + left * v, 5);
                if (!ok) continue;
                // the curve from the gate back to where the expressway leaves the city
                const Real k2 = (gate - leave).length() * Real(0.4);
                std::vector<Vec2> path;
                const int n = std::max(4, static_cast<int>((gate - leave).length() / 20));
                for (int i = 0; i < n; ++i) {
                    const Real tt = Real(i) / n, uu = 1 - tt;
                    const Vec2 p0 = gate, p1 = gate - d * k2, p2 = leave + dir * k2, p3 = leave;
                    path.push_back(p0 * (uu * uu * uu) + p1 * (3 * uu * uu * tt) + p2 * (3 * uu * tt * tt) + p3 * (tt * tt * tt));
                }
                for (const Vec2& q : path) if (!dry(q, 2)) ok = false;
                if (!ok) continue;
                t.built = true; t.gate = gate; t.axis = d; t.centre = gate + d * (townDepth * Real(0.5));
                head = std::move(path);
                return t;
            }
        }
        return t;
    };
    // The streets: a MAIN STREET across the expressway's end at the gate (an arterial), cross
    // streets behind it every `block`[0], and streets running out from it every `block`[1] — but
    // none within 40 m of the expressway's line, where its boulevard pair meets the main street at
    // two Ts 40 m apart. Each street overshoots the one it ends on by 2 m, so the planariser finds
    // the T, and pruneStubs takes the overshoot back.
    std::vector<Polyline> townRoads;
    auto layTown = [&](const CityPlan::Town& t) {
        const Vec2 left(-t.axis.y, t.axis.x);
        const Real bu = townSpec.contains("block") ? townSpec["block"][0].get<double>() : 100.0;
        const Real bv = townSpec.contains("block") ? townSpec["block"][1].get<double>() : 80.0;
        auto at = [&](Real u, Real v) { return t.gate + t.axis * u + left * v; };
        auto street = [&](Vec2 a, Vec2 b, RoadClass k, Real w) {
            Polyline p; p.klass = k; p.width = w;
            const int n = std::max(1, static_cast<int>((b - a).length() / 20.0));
            for (int i = 0; i <= n; ++i) p.pts.push_back(a + (b - a) * (Real(i) / n));
            townRoads.push_back(p);
        };
        std::vector<Real> vs;
        for (Real v = 40; v <= townHalf + 1; v += bv) { vs.push_back(v); vs.push_back(-v); }
        const Real vMax = vs.empty() ? 40 : *std::max_element(vs.begin(), vs.end());
        // the town rounds off away from the city: its streets shorten toward the far corners
        auto reach = [&](Real v) { return townDepth * std::sqrt(std::max(Real(0.25), 1 - (v / (vMax + bv)) * (v / (vMax + bv)))); };
        street(at(0, -vMax - 2), at(0, vMax + 2), RoadClass::Arterial, B.arterialWidth);
        // cross streets first, each as wide as the side streets that reach it; then each side
        // street runs out to the last cross street that spans it, so every block closes and
        // nothing is left dangling past the edge of town
        std::vector<std::pair<Real, Real>> cross;   // (u, half-width)
        for (Real u = bu; u <= townDepth - 20; u += bu) {
            Real w = 0;
            for (Real v : vs) if (reach(v) >= u + 20) w = std::max(w, std::fabs(v));
            if (w > 0) { cross.push_back({u, w}); street(at(u, -w - 2), at(u, w + 2), RoadClass::Local, B.localWidth); }
        }
        for (Real v : vs) {
            Real end = 0;
            for (const auto& c : cross) if (c.second >= std::fabs(v) - 0.5) end = std::max(end, c.first);
            if (end > 0) street(at(-2, v), at(end + 2, v), RoadClass::Local, B.localWidth);
        }
    };

    // --- the outer loop and its places ---
    const nlohmann::json loopSpec = B.world.is_null() ? nlohmann::json() : B.world.value("loop", nlohmann::json());
    // A PLACE along the loop, in the loop's own frame (station along it, offset from it): its main
    // street 50 m in on the city side (an arterial), `depth` - 1 more streets parallel behind it, a
    // back road 50 m out on the mountain side, and cross streets every 110 m from the back road to
    // the last parallel, passing under the freeway. The two roads 50 m either side are where the
    // loop's diamonds land, as the ring's land on its frontage roads.
    std::vector<Polyline> placeRoads;
    auto layPlace = [&](CityPlan::Place& pl, int depth, const HeightField& g) {
        (void)g;
        const std::vector<double> st = roads::lanes::stations(plan.loop);
        auto frame = [&](Real s, Real off) {
            const Vec2 p = roads::lanes::pointAt(plan.loop, st, s), t = roads::lanes::tangentAtStation(plan.loop, st, s);
            const Vec2 n(-t.y, t.x);
            const Real in = dot(n, B.center - p) > 0 ? 1 : -1;   // + offsets toward the city
            return p + n * (in * off);
        };
        auto street = [&](Real s0, Real s1, Real off0, Real off1, RoadClass k, Real w) {
            Polyline p; p.klass = k; p.width = w;
            if (off0 == off1) {   // along the loop, curving with it
                for (Real s = s0; s < s1; s += 20) p.pts.push_back(frame(s, off0));
                p.pts.push_back(frame(s1, off0));
            } else {              // across it
                const int n = std::max(1, static_cast<int>(std::fabs(off1 - off0) / 20));
                for (int i = 0; i <= n; ++i) p.pts.push_back(frame(s0, off0 + (off1 - off0) * i / n));
            }
            placeRoads.push_back(p);
        };
        const Real bv = 90, bu = 110;
        const Real inner = 50 + (depth - 1) * bv;
        street(pl.s0 - 2, pl.s1 + 2, 50, 50, RoadClass::Arterial, B.arterialWidth);                  // main street
        for (int k = 1; k < depth; ++k) street(pl.s0 - 2, pl.s1 + 2, 50 + k * bv, 50 + k * bv, RoadClass::Local, B.localWidth);
        street(pl.s0 - 2, pl.s1 + 2, -50, -50, RoadClass::Collector, B.collectorWidth);             // back road
        // Every fifth cross street passes under the freeway to the back road; the rest stop at the
        // main street. With all of them crossing, 110 m apart, every diamond had a ramp coming down
        // across one — the generator refused all 40 — and at every fourth, a diamond's decel or aux
        // lane still reached over the next. A diamond sits on a crossing street, its ramps touching
        // down on the main street and back road between them.
        // Counted from the MIDDLE cross street, so a short town's one crossing is at its centre,
        // with main street and back road running on both ways for its ramps to land along.
        const int nCross = static_cast<int>((pl.s1 + 1 - pl.s0) / bu) + 1, mid = nCross / 2;
        int j = 0;
        for (Real s = pl.s0; s <= pl.s1 + 1; s += bu, ++j) {
            const bool under = (j - mid) % 5 == 0;
            street(s, s, under ? -52 : 48, inner + 2, under ? RoadClass::Collector : RoadClass::Local, under ? B.collectorWidth : B.localWidth);
        }
        pl.hub = frame((pl.s0 + pl.s1) / 2, 50 + (depth - 1) * bv / 2);
    };

    // AN ORGANIC TOWN on the loop (Glenn, 2026-09-23: "organic cellular lot growth — curvy residential
    // areas and gridded town centers"). What its diamond needs from a strip place — main street and back
    // road 50 m either side of the loop, one street under it at the centre — then a gridded CENTRE on the
    // city side of the main street, and around it RESIDENTIAL streets that are the edges of cells: seeds
    // scattered at least `cell` apart, their Voronoi cells (the dual of their Delaunay triangulation),
    // each edge bowed into a gentle curve. Cells meet three to a junction at near 120 degrees, which is
    // also the junction the lanes builder is happiest with. The cells stop short of the freeway, the
    // centre, steep ground (6%), the sea and the edge of the ground grid, so the town's edge is ragged.
    auto layTown2 = [&](CityPlan::Place& pl, const nlohmann::json& pj, const HeightField& g) {
        const std::vector<double> st = roads::lanes::stations(plan.loop);
        const Real sc = (pl.s0 + pl.s1) / 2;
        auto frameAt = [&](Real s, Real off, Vec2& origin, Vec2& along, Vec2& in) {
            origin = roads::lanes::pointAt(plan.loop, st, s);
            along = roads::lanes::tangentAtStation(plan.loop, st, s);
            const Vec2 n(-along.y, along.x);
            in = dot(n, B.center - origin) > 0 ? n : n * Real(-1);
            origin = origin + in * off;
        };
        auto frame = [&](Real s, Real off) { Vec2 o, a, i; frameAt(s, 0, o, a, i); return o + i * off; };
        auto street = [&](std::vector<Vec2> pts, RoadClass k, Real w) { Polyline p; p.klass = k; p.width = w; p.pts = std::move(pts); placeRoads.push_back(p); };
        auto along = [&](Real s0, Real s1, Real off, RoadClass k, Real w) {
            std::vector<Vec2> pts; for (Real s = s0; s < s1; s += 20) pts.push_back(frame(s, off)); pts.push_back(frame(s1, off)); street(pts, k, w);
        };
        auto across = [&](Real s, Real off0, Real off1, RoadClass k, Real w) {
            std::vector<Vec2> pts; const int n = std::max(1, static_cast<int>(std::fabs(off1 - off0) / 20));
            for (int i = 0; i <= n; ++i) pts.push_back(frame(s, off0 + (off1 - off0) * i / n)); street(pts, k, w);
        };
        // THE CENTRE: a grid behind the main street, `grid` blocks each way — or, for a CITY on the
        // loop, `gridAlong` blocks along it by `gridDeep` deep — its streets square to the loop and
        // curving with it. The main street is its front edge; the back road runs as far on the far
        // side. Streets pass under the freeway the least interchange spacing apart, counted from the centre (a town's one is
        // at its middle): with more, every diamond's ramps or merge lanes reached over one and it was
        // refused. A city's DOWNTOWN is its middle, a financial hub of its own: towers, not smaller
        // blocks (half-size ones fall under the three-lot minimum and the thin-block pass took them).
        const int nb = pj.value("grid", 3);
        const int nbU = pj.value("gridAlong", nb), nbV = pj.value("gridDeep", nb);
        const Real bu = pj.value("blockAlong", 90.0), bv = pj.value("blockDeep", 75.0);
        const Real half = nbU * bu / 2, depth = 50 + nbV * bv;
        const bool town = nbU == nb && nbV == nb;
        along(sc - half - 2, sc + half + 2, 50, RoadClass::Arterial, B.arterialWidth);
        // (a city's runs on past its end crossing far enough for that diamond's ramps to land along it:
        // touch-down, run along it and the shift across, 30 + 20 + 50 m)
        const Real backRun = town ? 32 : 110;
        along(sc - half - backRun, sc + half + backRun, -50, RoadClass::Collector, B.collectorWidth);
        const int underEvery = std::max(1, static_cast<int>(std::ceil(SceneOptions{}.interchangeSpacing / bu)));   // a diamond at each
        for (int i = 0; i <= nbU; ++i) {
            const int fromMid = i - nbU / 2;
            const bool under = town ? i == nbU / 2 : fromMid % underEvery == 0;
            const bool main = i == nbU / 2 || under;
            across(sc - half + i * bu, under ? -52 : 48, depth + 2, main ? RoadClass::Collector : RoadClass::Local, main ? B.collectorWidth : B.localWidth);
        }
        for (int j = 1; j <= nbV; ++j) along(sc - half - 2, sc + half + 2, 50 + j * bv, RoadClass::Local, B.localWidth);
        Vec2 o, a, in; frameAt(sc, 0, o, a, in);
        const Vec2 centre = o + in * (50 + nbV * bv / 2);
        if (town) pl.hubs.push_back({centre, pl.kind});
        else {
            // a city's quarters: downtown in the middle, shops and offices a third of the way out
            // each side, industry at the far end by the freeway, housing beyond (the cells' hub)
            pl.hubs.push_back({centre, "financial"});
            for (Real f : {-0.36, 0.36}) pl.hubs.push_back({frame(sc + f * 2 * half, 50 + nbV * bv / 2), "commercial"});
            pl.hubs.push_back({frame(sc + half * 0.9, 50 + nbV * bv * 0.4), "industrial"});
        }
        // inside the centre's grid, in the loop's own (station, offset) frame, so a grid kilometres
        // long follows the loop's curve (a town's straight frame at its middle was close enough)
        const std::vector<double> gst = roads::lanes::stations(plan.loop);
        auto inGrid = [&](const Vec2& p, Real margin) {
            const roads::lanes::Projection pr = roads::lanes::project(plan.loop, gst, p);
            const Vec2 foot = roads::lanes::pointAt(plan.loop, gst, pr.station);
            const Real u = pr.station - sc, v = dot(p - foot, in) > 0 ? pr.distance : -pr.distance;
            return u > -half - margin && u < half + margin && v > 50 - margin && v < depth + margin;
        };
        // THE CELLS: seeds at least `cell` apart, a jittered lattice thinned by distance
        const Real cell = pj.value("cell", 105.0), radius = pj.value("radius", 520.0), fringe = pj.value("fringe", 0.0);
        const Real sea = B.world.value("seaLevel", -1e30), gridHalf = B.world.value("grid", B.size * 0.5 + 60);
        const Vec2 tc = o + in * (50 + nbV * bv * Real(0.5));
        const std::vector<double> lst = roads::lanes::stations(plan.loop);
        auto open = [&](const Vec2& p) {   // may a residential street stand here?
            if (town ? (p - tc).length() > radius : !inGrid(p, fringe)) return false;
            // on the town's own side of the loop: across it, nothing reaches a cell
            const roads::lanes::Projection pr = roads::lanes::project(plan.loop, lst, p);
            if (dot(p - roads::lanes::pointAt(plan.loop, lst, pr.station), in) < 0) return false;
            if (std::fabs(p.x - B.center.x) > gridHalf - 150 || std::fabs(p.y - B.center.y) > gridHalf - 150) return false;
            if (distToPolyline(p, plan.loop, false) < 62) return false;
            if (g(p.x, p.y) < sea + 3) return false;
            const Real e = 10, sx = g(p.x + e, p.y) - g(p.x - e, p.y), sy = g(p.x, p.y + e) - g(p.x, p.y - e);
            return std::hypot(sx, sy) / (2 * e) < 0.06;
        };
        std::vector<Vec2> seeds;
        const uint32_t salt = static_cast<uint32_t>(std::hash<std::string>{}(pl.name));
        // the seed lattice spans the town's disc, or the city's grid and its fringe
        Vec2 lo = tc - Vec2(radius, radius), hi = tc + Vec2(radius, radius);
        if (!town) {
            lo = Vec2(1e30, 1e30); hi = Vec2(-1e30, -1e30);
            for (Real su = sc - half - fringe; su <= sc + half + fringe + 1; su += 40)
                for (Real sv : {Real(50) - fringe, depth + fringe}) {
                    const Vec2 q = frame(su, sv);
                    lo = Vec2(std::min(lo.x, q.x), std::min(lo.y, q.y)); hi = Vec2(std::max(hi.x, q.x), std::max(hi.y, q.y));
                }
        }
        for (Real y = lo.y - tc.y - cell; y <= hi.y - tc.y + cell; y += cell * Real(0.55))
            for (Real x = lo.x - tc.x - cell; x <= hi.x - tc.x + cell; x += cell * Real(0.55)) {
                const int ix = static_cast<int>(std::lround(x)), iy = static_cast<int>(std::lround(y));
                const Real jx = ((hash3(ix, iy, B.seed ^ salt) & 0xFFFF) / 65535.0 - 0.5) * cell * 0.5;
                const Real jy = ((hash3(iy, ix, B.seed ^ salt ^ 0x9E37u) & 0xFFFF) / 65535.0 - 0.5) * cell * 0.5;
                const Vec2 q = tc + Vec2(x + jx, y + jy);
                bool ok = true;
                for (const Vec2& s2 : seeds) if ((s2 - q).lengthSquared() < cell * cell) { ok = false; break; }
                if (ok) seeds.push_back(q);
            }
        const roads::lanes::Triangulation tri = roads::lanes::constrainedTriangulation(seeds, {});
        auto circumcentre = [&](const std::array<int, 3>& t) {
            const Vec2 A = tri.verts[static_cast<std::size_t>(t[0])], Bv = tri.verts[static_cast<std::size_t>(t[1])], C = tri.verts[static_cast<std::size_t>(t[2])];
            const Real d = 2 * (A.x * (Bv.y - C.y) + Bv.x * (C.y - A.y) + C.x * (A.y - Bv.y));
            if (std::fabs(d) < 1e-9) return (A + Bv + C) / Real(3);
            const Real a2 = A.lengthSquared(), b2 = Bv.lengthSquared(), c2 = C.lengthSquared();
            return Vec2((a2 * (Bv.y - C.y) + b2 * (C.y - A.y) + c2 * (A.y - Bv.y)) / d, (a2 * (C.x - Bv.x) + b2 * (A.x - C.x) + c2 * (Bv.x - A.x)) / d);
        };
        std::map<std::pair<int, int>, std::vector<std::size_t>> byEdge;   // Delaunay edge -> its triangles
        for (std::size_t ti = 0; ti < tri.tris.size(); ++ti)
            for (int k = 0; k < 3; ++k) {
                const int u = tri.tris[ti][static_cast<std::size_t>(k)], v = tri.tris[ti][static_cast<std::size_t>((k + 1) % 3)];
                byEdge[{std::min(u, v), std::max(u, v)}].push_back(ti);
            }
        // The cells' corners are the triangles' circumcentres. Near-cocircular seeds put two corners
        // metres apart; those MERGE into one junction (dropping the short edge between them instead
        // left every cell an island).
        std::vector<std::size_t> root(tri.tris.size());
        for (std::size_t i = 0; i < root.size(); ++i) root[i] = i;
        std::function<std::size_t(std::size_t)> find = [&](std::size_t i) { return root[i] == i ? i : root[i] = find(root[i]); };
        std::vector<Vec2> cc(tri.tris.size());
        for (std::size_t ti = 0; ti < tri.tris.size(); ++ti) cc[ti] = circumcentre(tri.tris[ti]);
        for (const auto& kv : byEdge)
            if (kv.second.size() == 2 && (cc[kv.second[0]] - cc[kv.second[1]]).length() < 30) root[find(kv.second[0])] = find(kv.second[1]);
        std::map<std::size_t, std::pair<Vec2, int>> merged;
        for (std::size_t ti = 0; ti < tri.tris.size(); ++ti) { auto& m = merged[find(ti)]; m.first = m.first + cc[ti]; ++m.second; }
        auto corner = [&](std::size_t ti) { const auto& m = merged[find(ti)]; return m.first / Real(m.second); };
        std::set<std::pair<std::size_t, std::size_t>> done;
        int streets = 0;
        // Every street already laid out here — this town's grid and main street, the loop's other
        // places — as a cell street may cross one (a junction) but not run beside one: a cell edge
        // that ends near a grid ran along its edge street 10-18 m off, two roads where one belongs.
        const std::size_t before = placeRoads.size();
        auto runsBeside = [&](const std::vector<Vec2>& pts) {
            const std::vector<double> ss = roads::lanes::stations(pts);
            for (Real sv = 10; sv < ss.back() - 10; sv += 6) {
                const Vec2 q = roads::lanes::pointAt(pts, ss, sv), t = roads::lanes::tangentAtStation(pts, ss, sv);
                for (std::size_t k = 0; k < before; ++k) {
                    const std::vector<Vec2>& o2 = placeRoads[k].pts;
                    const std::vector<double> os = roads::lanes::stations(o2);
                    const roads::lanes::Projection pr = roads::lanes::project(o2, os, q);
                    if (pr.distance < 24 && std::fabs(dot(t, roads::lanes::tangentAtStation(o2, os, pr.station))) > 0.8) return true;
                }
            }
            return false;
        };
        for (const auto& kv : byEdge) {
            if (kv.second.size() != 2) continue;   // a hull edge: no Voronoi edge of finite length
            const std::size_t ra = find(kv.second[0]), rb = find(kv.second[1]);
            if (ra == rb || !done.insert({std::min(ra, rb), std::max(ra, rb)}).second) continue;
            Vec2 p = corner(kv.second[0]), q = corner(kv.second[1]);
            // into the centre's grid, only as far as its edge (3 m on, for the planariser's T)
            const bool pIn = inGrid(p, 0), qIn = inGrid(q, 0);
            if (pIn && qIn) continue;
            if (pIn || qIn) {
                const Vec2 inside = pIn ? p : q, out = pIn ? q : p;
                Real lo = 0, hi = 1;   // bisect for the rectangle's edge
                for (int it = 0; it < 30; ++it) { const Real m = (lo + hi) / 2; if (inGrid(out + (inside - out) * m, 0)) hi = m; else lo = m; }
                const Vec2 edge = out + (inside - out) * lo;
                const Vec2 dir = normalize(inside - out);
                p = out; q = edge + dir * Real(3);
            }
            if (!open(p) && !inGrid(p, 5)) continue;
            if (!open(q) && !inGrid(q, 5)) continue;
            // bowed into a curve: the midpoint pushed aside by up to 12% of the street's length
            const Vec2 d = q - p, n(-d.y / d.length(), d.x / d.length());
            const Real bow = ((hash3(kv.first.first, kv.first.second, B.seed ^ salt) & 0xFF) / 255.0 - 0.5) * 0.24 * d.length();
            const Vec2 m = (p + q) * Real(0.5) + n * bow;
            std::vector<Vec2> pts;
            for (int i = 0; i <= 6; ++i) { const Real t = Real(i) / 6, u = 1 - t; pts.push_back(p * (u * u) + m * (2 * u * t) + q * (t * t)); }
            if (!open(m) && !inGrid(m, 5)) continue;
            if (runsBeside(pts)) continue;
            street(pts, RoadClass::Local, B.localWidth);
            ++streets;
        }
        // the residential hub: the mean of the seeds that stand in open ground
        // (a city's, one per third of its length: one mean over a city kilometres long lands downtown)
        const int bins = town ? 1 : 3;
        std::vector<Vec2> sum(static_cast<std::size_t>(bins), Vec2(0, 0)); std::vector<int> n(static_cast<std::size_t>(bins), 0);
        for (const Vec2& s2 : seeds) {
            if (!open(s2) || inGrid(s2, 20)) continue;
            const Real u = roads::lanes::project(plan.loop, gst, s2).station - sc;
            const int k = std::clamp(static_cast<int>((u + half + fringe) / (2 * (half + fringe)) * bins), 0, bins - 1);
            sum[static_cast<std::size_t>(k)] = sum[static_cast<std::size_t>(k)] + s2; ++n[static_cast<std::size_t>(k)];
        }
        for (int k = 0; k < bins; ++k) if (n[static_cast<std::size_t>(k)]) pl.hubs.push_back({sum[static_cast<std::size_t>(k)] / Real(n[static_cast<std::size_t>(k)]), "residential"});
        pl.hub = centre;
        if (std::getenv("RT_PLAN_WHY")) std::printf("[plan] %s: a %dx%d centre, %d cell streets from %zu seeds, hubs", pl.name.c_str(), nbU, nbV, streets, seeds.size());
        if (std::getenv("RT_PLAN_WHY")) { for (const auto& h : pl.hubs) std::printf(" %s", h.second.c_str()); std::printf("\n"); }
    };

    // THE MOUNTAIN FRONT, ray by ray from the centre: the first place past the city where the
    // ground turns steep (6% over 20 m). The loop keeps `setback` short of it, clamped between
    // room for the places inside and the edge of the ground grid, and is smoothed along its arc.
    // Of the two arcs between the expressways it takes the one with more front: the mountain side.
    auto buildLoop = [&](const std::vector<Real>& th, std::vector<Real>& start) {
        const HeightField g = sceneGround(B);
        const Real gridHalf = B.world.value("grid", B.size * 0.5 + 60);
        const Real setback = loopSpec.value("setback", 250.0);
        const Real rMin = outerR + loopSpec.value("inner", 650.0), rMax = gridHalf - 350;
        auto front = [&](Real a) {
            const Vec2 d(std::cos(a), std::sin(a));
            for (Real r = rMin; r <= rMax + setback; r += 20) {
                const Vec2 p = B.center + d * r, q = B.center + d * (r + 20);
                if (std::fabs(q.x - B.center.x) > gridHalf - 100 || std::fabs(q.y - B.center.y) > gridHalf - 100) return Real(1e9);
                if (g(q.x, q.y) - g(p.x, p.y) > 0.06 * 20) return r;
            }
            return Real(1e9);
        };
        const Real deg = kPi / 180;
        Real a0 = th[0], a1 = th[1];
        auto sweep = [&](Real from, Real to, Real sign) {   // angles from `from` to `to` going `sign`-wise, 1 degree apart
            std::vector<Real> out;
            Real span = std::remainder(to - from, 2 * kPi);
            if (span * sign < 0) span += sign * 2 * kPi;
            for (Real t = 0; t <= std::fabs(span) + 1e-9; t += deg) out.push_back(from + sign * t);
            return out;
        };
        int hitsUp = 0, hitsDown = 0;
        for (Real a : sweep(a0, a1, +1)) if (front(a) < 1e8) ++hitsUp;
        for (Real a : sweep(a0, a1, -1)) if (front(a) < 1e8) ++hitsDown;
        const Real sign = hitsUp >= hitsDown ? 1 : -1;
        const std::vector<Real> arc = sweep(a0, a1, sign);
        std::vector<Real> r(arc.size());
        for (std::size_t i = 0; i < arc.size(); ++i) r[i] = std::clamp(front(arc[i]) - setback, rMin, rMax);
        for (int pass = 0; pass < 4; ++pass) {   // +-8 degrees, four times: a curve a freeway can drive
            std::vector<Real> sm(r.size());
            for (std::size_t i = 0; i < r.size(); ++i) {
                Real sum = 0; int n = 0;
                for (int k = -8; k <= 8; ++k) { const long j = static_cast<long>(i) + k; if (j < 0 || j >= static_cast<long>(r.size())) continue; sum += r[static_cast<std::size_t>(j)]; ++n; }
                sm[i] = sum / n;
            }
            r.swap(sm);
        }
        // THE JOINTS: each spur runs out to 450 m short of the loop's radius at its angle and bends
        // into the loop 25 degrees along, on one cubic — a 90-degree turn at a freeway's radius
        // (350 m and 20 degrees made the NE joint's tightest curve 250 m).
        const std::size_t bend = std::min<std::size_t>(25, arc.size() / 4);
        auto at = [&](std::size_t i) { return B.center + Vec2(std::cos(arc[i]), std::sin(arc[i])) * r[i]; };
        auto tangent = [&](std::size_t i) { return normalize(at(std::min(i + 1, arc.size() - 1)) - at(i > 0 ? i - 1 : 0)); };
        const Vec2 dA(std::cos(a0), std::sin(a0)), dB(std::cos(a1), std::sin(a1));
        const Vec2 jA = B.center + dA * (r.front() - 450), jB = B.center + dB * (r.back() - 450);
        start = {r.front() - 450, r.back() - 450};
        const std::size_t iA = bend, iB = arc.size() - 1 - bend;
        auto curve = [&](const Vec2& p0, const Vec2& t0, const Vec2& p3, const Vec2& t3) {
            const Real k = (p3 - p0).length() * Real(0.45);
            const Vec2 p1 = p0 + t0 * k, p2 = p3 - t3 * k;
            std::vector<Vec2> out;
            const int n = std::max(4, static_cast<int>((p3 - p0).length() / 20));
            for (int i = 0; i < n; ++i) { const Real t = Real(i) / n, u = 1 - t; out.push_back(p0 * (u * u * u) + p1 * (3 * u * u * t) + p2 * (3 * u * t * t) + p3 * (t * t * t)); }
            return out;
        };
        plan.loop = curve(jA, dA, at(iA), tangent(iA));
        for (std::size_t i = iA; i <= iB; ++i) plan.loop.push_back(at(i));
        const std::vector<Vec2> tail = curve(at(iB), tangent(iB), jB, dB * Real(-1));
        plan.loop.insert(plan.loop.end(), tail.begin() + 1, tail.end());
        plan.loop.push_back(jB);
        plan.loopFrom = 0; plan.loopTo = 1;
        // THE PLACES: each at the loop's point nearest its angle, `length` along the loop.
        const std::vector<double> st = roads::lanes::stations(plan.loop);
        for (const nlohmann::json& pj : loopSpec.value("places", nlohmann::json::array())) {
            const Real want = pj.value("at", 180.0) * deg;
            std::size_t best = 0; Real bd = 1e9;
            for (std::size_t i = 0; i < plan.loop.size(); ++i) {
                const Vec2 d = plan.loop[i] - B.center;
                const Real e = std::fabs(std::remainder(std::atan2(d.y, d.x) - want, 2 * kPi));
                if (e < bd) { bd = e; best = i; }
            }
            CityPlan::Place pl;
            pl.name = pj.value("name", std::string("place"));
            pl.kind = pj.value("kind", std::string("oldtown"));
            const Real len = pj.value("length", 500.0);
            pl.s0 = std::max(400.0, st[best] - len / 2);
            pl.s1 = std::min(st.back() - 400.0, st[best] + len / 2);
            if (pl.s1 - pl.s0 < 200) continue;
            if (pj.value("style", std::string("strip")) == "town") layTown2(pl, pj, g);
            else { layPlace(pl, pj.value("depth", 2), g); pl.hubs.push_back({pl.hub, pl.kind}); }
            plan.places.push_back(pl);
        }
    };

    // --- freeway: a ring and radial spurs toward downtown ---
    std::vector<Polyline> fw;
    bool ringKept = B.freewayRadius > 0 && !shaped;   // a shaped city's freeways are the region's, not a ring
    {
        Polyline ring; ring.klass = RoadClass::Freeway; ring.width = B.freewayWidth; ring.closed = true;
        const int n = static_cast<int>(2 * kPi * B.freewayRadius / 20.0);
        for (int i = 0; i < n; ++i) {
            const Real th = 2 * kPi * i / n;
            ring.pts.push_back(B.center + Vec2(std::cos(th), std::sin(th)) * freewayR(th));
        }
        // AGAINST THE COAST: the ring's longest run over land, as an open C. Less than 40% of it left
        // and the city is a strip along the shore with no ring at all.
        if (land.n && n >= 8) {
            std::vector<char> ok(static_cast<std::size_t>(n));
            bool all = true;
            for (int i = 0; i < n; ++i) { ok[static_cast<std::size_t>(i)] = landAt(ring.pts[static_cast<std::size_t>(i)], false); all = all && ok[static_cast<std::size_t>(i)]; }
            if (!all) {
                int bestAt = 0, bestLen = 0;
                for (int i = 0; i < n; ++i) {
                    if (!ok[static_cast<std::size_t>(i)] || ok[static_cast<std::size_t>((i + n - 1) % n)]) continue;   // a run's start
                    int len = 0;
                    while (len < n && ok[static_cast<std::size_t>((i + len) % n)]) ++len;
                    if (len > bestLen) { bestLen = len; bestAt = i; }
                }
                std::vector<Vec2> arc;
                for (int k = 0; k < bestLen; ++k) arc.push_back(ring.pts[static_cast<std::size_t>((bestAt + k) % n)]);
                // no hook at its ends: where the C comes out over the ragged shore the radius blend
                // turns it sharply in its last few hundred metres -- it ends before the turn
                for (int side = 0; side < 2 && arc.size() > 40; ++side) {
                    std::size_t cut = 0;
                    for (std::size_t k = 1; k < 15; ++k) {
                        const Vec2 d0 = arc[k] - arc[k - 1], d1 = arc[k + 1] - arc[k];
                        const Real c = dot(d0, d1) / std::max(Real(1e-9), d0.length() * d1.length());
                        if (c < std::cos(20.0 * kPi / 180.0)) cut = k + 1;
                    }
                    arc.erase(arc.begin(), arc.begin() + static_cast<std::ptrdiff_t>(cut));
                    std::reverse(arc.begin(), arc.end());
                }
                ring.pts = arc;
                ring.closed = false;
                if (bestLen >= n * 2 / 5) plan.ringArc = arc;
                else ringKept = false;
            }
        }
        fw.push_back(ring);
        std::vector<Real> spurTheta;
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
            spurTheta.push_back(th0);
        }
        // THE OUTER LOOP: round the mountain side, from one expressway's outer end to the other's.
        std::vector<Real> spurStart(spurTheta.size(), 0);   // where each spur begins, if the loop sets it
        if (!loopSpec.is_null() && spurTheta.size() == 2) buildLoop(spurTheta, spurStart);
        for (std::size_t i = 0; i < spurTheta.size(); ++i) {
            const Real th0 = spurTheta[i];
            // A RADIAL EXPRESSWAY: in from the loop, a town or the map edge — the region's road
            // into the city — across the ring at a system interchange, to midtown's boulevard.
            Polyline spur; spur.klass = RoadClass::Freeway; spur.width = B.freewayWidth;
            const Real rEnd = midR(th0);
            const Vec2 dir(std::cos(th0), std::sin(th0));
            std::vector<Vec2> head;   // gate -> where it leaves the city, when it bends to a town
            CityPlan::Town town;
            if (spurStart[i] <= 0) town = siteTown(dir, head);
            const Real rStart = spurStart[i] > 0 ? spurStart[i]
                                : town.built ? outerR + 60
                                : B.world.is_null() ? B.size * Real(0.5) + 30 : B.world.value("grid", B.size * 0.5 + 60) - 120;
            if (town.built) layTown(town);
            plan.towns.push_back(town);
            spur.pts = head;
            for (Real r = rStart; r >= rEnd; r -= 20.0)
                spur.pts.push_back(B.center + dir * r);
            spur.pts.push_back(warped(B.center + dir * rEnd));
            fw.push_back(spur);
            plan.interchanges.push_back(spur.pts.back());   // the spur lands on midtown's boulevard
            if (town.built) plan.interchanges.push_back(town.gate);   // and on its town's main street
        }
        if (!plan.loop.empty()) {
            Polyline lp; lp.klass = RoadClass::Freeway; lp.width = B.freewayWidth; lp.pts = plan.loop;
            fw.push_back(lp);
        }
    }
    if (!ringKept || fw.front().pts.size() < 2) fw.front().pts.clear();
    {
        std::vector<Polyline> drawn;
        for (const Polyline& p : fw) if (p.pts.size() >= 2) drawn.push_back(p);
        plan.freeway = planarizePolylines(drawn);
    }
    if (plan.ringArc.empty() && ringKept) plan.ring = fw.front().pts;   // the closed ring; an open C is ringArc
    for (std::size_t i = 1; i < fw.size(); ++i) if (plan.loop.empty() || i + 1 < fw.size()) plan.spurs.push_back(fw[i].pts);   // the loop is last, not a spur
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
    // THE SYSTEM INTERCHANGES CLEAR THEIR GROUND. Where an expressway crosses the ring it runs
    // at ground level under the ring's deck, so no street may cross it there, and its four
    // ramps sweep through the quadrants round the crossing. Streets are cut back from both —
    // the same ramps the scene will build (plan_scene.h), so what is cleared is what is built
    // — and the dead ends that leaves are pruned to their last junction below.
    std::vector<std::vector<Vec2>> keepOut;
    std::vector<Real> keepReach;
    {
        const FreewaySection fs = freewaySection(B);
        const roads::lanes::SystemOptions so = systemOptions();
        for (const SystemAt& at : systemInterchanges(plan, ringChains(plan))) {
            if (!at.r.built) continue;
            for (const nlohmann::json& r : at.r.ramps) {
                std::vector<Vec2> sp;
                for (const nlohmann::json& p : r["path"]["points"]) sp.push_back(Vec2(p[0].get<double>(), p[1].get<double>()));
                keepOut.push_back(sp);
                keepReach.push_back(so.rampHalf + 3);
            }
            // the expressway at ground level: from 150 m outside the ring (where it is still
            // climbing to bridge the suburbs) to where its freeway ends inside
            const std::vector<Vec2> fwy = spurFreeway(plan, at.spur);
            const std::vector<double> st = roads::lanes::stations(fwy);
            std::vector<Vec2> low;
            for (std::size_t i = 0; i < fwy.size(); ++i) if (st[i] >= at.r.sStem - 150.0) low.push_back(fwy[i]);
            if (low.size() >= 2) { keepOut.push_back(low); keepReach.push_back(fs.edgeReach + 3); }
        }
        if (!keepOut.empty()) {
            auto blocked = [&](const Vec2& q, Real half) {
                for (std::size_t k = 0; k < keepOut.size(); ++k)
                    if (distToPolyline(q, keepOut[k], false) < keepReach[k] + half) return true;
                return false;
            };
            roads = cutRoadsWhere(roads, B.sidewalk, blocked);
        }
    }
    // --- the street graph ---
    roads.insert(roads.end(), townRoads.begin(), townRoads.end());
    roads.insert(roads.end(), placeRoads.begin(), placeRoads.end());
    roads.erase(std::remove_if(roads.begin(), roads.end(), [](const Polyline& p) { return p.pts.size() < 2; }), roads.end());
    if (land.n) {
        const std::size_t before = roads.size();
        roads = cutRoadsWhere(roads, B.sidewalk, [&](const Vec2& q, Real) { return !landAt(q, true); }, 60.0);
        if (std::getenv("RT_PLAN_WHY")) std::printf("[plan] land: %zu roads in, %zu pieces kept on land%s\n", before, roads.size(),
                                                    plan.ringArc.empty() ? (plan.ring.empty() ? ", no ring" : "") : ", the ring a C against the coast");
    }
    // --- RIVERS (ADR-0104). Glenn: "regenerate the city into two halves cut by the river and then
    // create a handful of bridges. Not every road would become a bridge. Look at Chicago." The river
    // is the world's (its hydrology). A street that crosses it is a BRIDGE only if it is a main
    // street -- an arterial -- or the river would otherwise run too far without one (a collector is
    // promoted in the middle of the gap). Every other street stops at a RIVERSIDE street that runs
    // along each bank inside the city (Chicago's Wacker Drive), in a T.
    if (const std::shared_ptr<const Hydrology> hy = worldHydrology(B)) {
        const nlohmann::json rp = B.world.value("riverPlan", nlohmann::json::object());
        const Real quay = rp.value("quay", 4.0);                         // bank to the riverside street's kerb
        const Real sideHalf = B.collectorWidth * 0.5 + B.sidewalk;       // the riverside street's half-width
        const Real sideAt = quay + sideHalf;                              // its centreline, from the bank
        const Real maxGap = rp.value("maxBridgeGap", 650.0);
        const Real maxBridge = rp.value("maxBridge", 220.0);
        // a shaped city's riverside street is its depth contour along the bank already
        const bool riverside = rp.value("riversideStreets", !shaped);
        auto bank = [&](const Vec2& q) { return hy->distanceToRiver(q.x, q.y, 300.0); };
        auto inCity = [&](const Vec2& q) { return shaped ? shape.depthAt(q) > 20.0 : (q - B.center).length() < outerR - 20.0; };
        // where each river crossing is along its river: (river, station) -> for the gap rule
        auto riverStation = [&](const Vec2& q, int* which, Vec2* tangent = nullptr) {
            double best = 1e30, bestS = 0.0;
            int bw = -1;
            Vec2 bt(1, 0);
            for (std::size_t ri = 0; ri < hy->rivers().size(); ++ri) {
                const auto& nodes = hy->rivers()[ri].nodes;
                double acc = 0.0;
                for (std::size_t k = 0; k + 1 < nodes.size(); ++k) {
                    const Vec2 a2 = nodes[k].p, ab = nodes[k + 1].p - a2;
                    const double L2 = dot(ab, ab), L = std::sqrt(L2);
                    const double t = L2 > 1e-12 ? std::clamp(dot(q - a2, ab) / L2, 0.0, 1.0) : 0.0;
                    const double d = (q - (a2 + ab * t)).length();
                    if (d < best) { best = d; bestS = acc + L * t; bw = static_cast<int>(ri); if (L > 1e-9) bt = ab / L; }
                    acc += L;
                }
            }
            if (which) *which = bw;
            if (tangent) *tangent = bt;
            return bestS;
        };
        struct Crossing { std::size_t road; int river; double station; bool bridge; };
        std::vector<Crossing> crossings;
        std::vector<std::vector<Vec2>> dense(roads.size());
        std::vector<char> wetRoad(roads.size(), 0), bridgeRoad(roads.size(), 0);
        for (std::size_t i = 0; i < roads.size(); ++i) {
            const Polyline& pl = roads[i];
            const std::size_t n = pl.pts.size(), segs = pl.closed ? n : n - 1;
            for (std::size_t k = 0; k < segs; ++k) {
                const Vec2 a2 = pl.pts[k], b2 = pl.pts[(k + 1) % n];
                const int m = std::max(1, static_cast<int>(std::ceil((b2 - a2).length() / 4.0)));
                for (int j = 0; j < m; ++j) dense[i].push_back(a2 + (b2 - a2) * (Real(j) / m));
            }
            if (!pl.closed && n) dense[i].push_back(pl.pts.back());
            const Real half = pl.width * 0.5 + B.sidewalk + quay;
            // its wet runs
            std::vector<std::pair<std::size_t, std::size_t>> runs;
            bool in = false;
            std::size_t r0 = 0;
            for (std::size_t k = 0; k < dense[i].size(); ++k) {
                const bool wet = bank(dense[i][k]) < half;
                if (wet && !in) { in = true; r0 = k; }
                if (!wet && in) { in = false; runs.push_back({r0, k}); }
            }
            if (in) runs.push_back({r0, dense[i].size() - 1});
            if (runs.empty()) continue;
            wetRoad[i] = 1;
            // a candidate bridge: every wet run inside it, short (a crossing, not a street along the bank)
            bool candidate = !pl.closed;
            for (const auto& r : runs) {
                const Real len = 4.0 * static_cast<Real>(r.second - r.first);
                if (r.first == 0 || r.second + 1 >= dense[i].size() || len > maxBridge) candidate = false;
            }
            // and it crosses, not glances: a street at a shallow angle would be a long diagonal deck
            for (const auto& r : runs) {
                const std::size_t mid = (r.first + r.second) / 2;
                const Vec2 d0 = dense[i][mid > 2 ? mid - 2 : 0], d1 = dense[i][std::min(mid + 2, dense[i].size() - 1)];
                Vec2 rt;
                riverStation(dense[i][mid], nullptr, &rt);
                const Vec2 sd = d1 - d0;
                if (sd.length() > 1e-9 && std::fabs(cross(sd / sd.length(), rt)) < std::sin(40.0 * 3.14159265358979 / 180.0)) candidate = false;
            }
            if (!candidate) continue;
            for (const auto& r : runs) {
                int which = -1;
                const double st = riverStation(dense[i][(r.first + r.second) / 2], &which);
                crossings.push_back({i, which, st, pl.klass == RoadClass::Arterial});
            }
        }
        // the gap rule: along each river, between consecutive bridges (and from the city's edge),
        // promote the collector crossing nearest the middle of any gap over maxGap
        for (std::size_t ri = 0; ri < hy->rivers().size(); ++ri) {
            for (int round = 0; round < 12; ++round) {
                std::vector<double> at;
                for (const Crossing& c : crossings) if (c.river == static_cast<int>(ri) && c.bridge) at.push_back(c.station);
                std::sort(at.begin(), at.end());
                // the gaps between the city's own crossings (candidates bound the city's extent)
                double lo = 1e30, hi = -1e30;
                for (const Crossing& c : crossings) if (c.river == static_cast<int>(ri)) { lo = std::min(lo, c.station); hi = std::max(hi, c.station); }
                if (lo > hi) break;
                std::vector<double> edges2{lo};
                edges2.insert(edges2.end(), at.begin(), at.end());
                edges2.push_back(hi);
                double worst = 0.0, mid = 0.0;
                for (std::size_t k = 0; k + 1 < edges2.size(); ++k)
                    if (edges2[k + 1] - edges2[k] > worst) { worst = edges2[k + 1] - edges2[k]; mid = 0.5 * (edges2[k] + edges2[k + 1]); }
                if (worst <= maxGap) break;
                int pick = -1;
                double pd = 1e30;
                for (std::size_t c = 0; c < crossings.size(); ++c) {
                    const Crossing& x = crossings[c];
                    if (x.river != static_cast<int>(ri) || x.bridge) continue;
                    if (roads[x.road].klass != RoadClass::Collector && roads[x.road].klass != RoadClass::Arterial) continue;
                    if (std::fabs(x.station - mid) < pd) { pd = std::fabs(x.station - mid); pick = static_cast<int>(c); }
                }
                if (pick < 0 || pd > worst * 0.5) break;
                for (Crossing& x : crossings) if (x.road == crossings[static_cast<std::size_t>(pick)].road) x.bridge = true;
            }
        }
        for (const Crossing& c : crossings) if (c.bridge) bridgeRoad[c.road] = 1;
        // cut the rest back to just past the riverside street's centreline (a T once planarized; the
        // pruner takes the overhang)
        const Real cutAt = riverside ? sideAt - 2.0 : quay + 2.0;
        std::vector<Polyline> kept;
        int bridges = 0, cut = 0;
        for (std::size_t i = 0; i < roads.size(); ++i) {
            if (!wetRoad[i] && !(riverside && roads[i].pts.size() >= 2)) { kept.push_back(roads[i]); continue; }
            if (bridgeRoad[i]) { kept.push_back(roads[i]); ++bridges; continue; }
            bool near = false;
            for (const Vec2& q : dense[i]) if (bank(q) < cutAt) { near = true; break; }
            if (!near) { kept.push_back(roads[i]); continue; }
            ++cut;
            Polyline cur = roads[i];
            cur.closed = false;
            cur.pts.clear();
            for (const Vec2& q : dense[i]) {
                if (bank(q) < cutAt) {
                    if (cur.pts.size() >= 2 && roads::lanes::stations(cur.pts).back() > 20.0) kept.push_back(cur);
                    cur.pts.clear();
                } else {
                    cur.pts.push_back(q);
                }
            }
            if (cur.pts.size() >= 2 && roads::lanes::stations(cur.pts).back() > 20.0) kept.push_back(cur);
        }
        roads.swap(kept);
        // the riverside streets: each bank of each river, where it runs through the city
        int sideRuns = 0;
        if (riverside)
            for (const River& rv : hy->rivers()) {
                for (int sgn : {-1, 1}) {
                    Polyline cur;
                    cur.klass = RoadClass::Collector;
                    cur.width = B.collectorWidth;
                    auto flush = [&] {
                        if (cur.pts.size() >= 2 && roads::lanes::stations(cur.pts).back() > 80.0) { roads.push_back(cur); ++sideRuns; }
                        cur.pts.clear();
                    };
                    double since = 1e30;
                    for (std::size_t k = 0; k < rv.nodes.size(); ++k) {
                        const Vec2 prev = rv.nodes[k > 0 ? k - 1 : 0].p, next = rv.nodes[std::min(k + 1, rv.nodes.size() - 1)].p;
                        Vec2 t = next - prev;
                        if (t.length() < 1e-9) continue;
                        t = t / t.length();
                        const Vec2 q = rv.nodes[k].p + perp(t) * (sgn * (rv.nodes[k].width * 0.5 + sideAt));
                        bool ok = inCity(q) && bank(q) > sideAt - 3.0;   // inside the city, and not over another reach
                        for (std::size_t kk = 0; ok && kk < keepOut.size(); ++kk)
                            if (distToPolyline(q, keepOut[kk], false) < keepReach[kk] + sideHalf) ok = false;
                        if (!ok) { flush(); since = 1e30; continue; }
                        if (k > 0) since += (rv.nodes[k].p - rv.nodes[k - 1].p).length();
                        if (cur.pts.empty() || since >= 12.0) { cur.pts.push_back(q); since = 0.0; }
                    }
                    flush();
                }
            }
        std::printf("[plan] rivers: %zu; %d bridges (main streets%s), %d streets stop at the water, %d riverside street runs\n",
                    hy->rivers().size(), bridges, maxGap < 1e29 ? " + gap-fillers" : "", cut, sideRuns);
    }
    plan.streets = planarizePolylines(roads);
    collapseShortLinks(plan.streets, B.sidewalk);
    pruneStubs(plan.streets, 45.0);
    {
        // thin and tiny blocks, counted over the whole map, split: the city, and the places out past it
        // Both from the street and the lot, not tuned: 2A/P is a strip's whole width, so a strip
        // narrower than a local street's pavement (carriageway + a sidewalk each side, measured
        // centreline to centreline) holds nothing; and a block whose land, once that pavement is
        // taken off, is less than three stock lots (ParcelParams: 16 x 28 m) is too small to lot.
        const ParcelParams lot;
        const Real pavement = B.localWidth + 2 * B.sidewalk;
        const Real minRadius = pavement;
        const Real minArea = std::pow(pavement + std::sqrt(3 * lot.frontWidth * lot.lotDepth), 2);
        auto inCity = [&](const Vec2& p) { return shaped ? shape.depthAt(p) > -60.0 : (p - B.center).length() < outerR + 60; };
        int removed = 0;
        const int city = simplifyThinBlocks(plan.streets, minArea, minRadius, inCity, true, nullptr);
        const int out = simplifyThinBlocks(plan.streets, minArea, minRadius, [&](const Vec2& p) { return !inCity(p); }, !(B.world.is_object() ? B.world.value("simplifyBlocks", true) : true), &removed);
        if (std::getenv("RT_PLAN_WHY"))
            std::printf("[plan] thin or tiny blocks (< %.0f m2, or 2A/P < %.0f m): %d in the city (left), %d beyond it (%d streets removed)\n",
                        minArea, minRadius, city, out, removed);
    }
    // a street cut short by the SLOPES goes back to its junction (up a hillside it is a whisker, not a
    // lane anyone laid); one cut by the sea stays -- a street that runs down to the water
    if (land.n)
        pruneStubsNear(plan.streets, [&](const Vec2& p) {
            for (const Vec2& d : {Vec2(20, 0), Vec2(-20, 0), Vec2(0, 20), Vec2(0, -20)})
                if (!landAt(p + d, true) && landAt(p + d, false)) return true;
            return false;
        });
    if (!keepOut.empty())
        pruneStubsNear(plan.streets, [&](const Vec2& p) {
            for (std::size_t k = 0; k < keepOut.size(); ++k)
                if (distToPolyline(p, keepOut[k], false) < keepReach[k] + 40.0) return true;
            return false;
        });
    // Interchanges: where the ring crosses an arterial (a spoke).
    for (const RoadEdge& e : plan.streets.edges) {
        if (e.klass != RoadClass::Arterial) continue;
        const Vec2 a = plan.streets.nodes[static_cast<std::size_t>(e.a)].pos, b = plan.streets.nodes[static_cast<std::size_t>(e.b)].pos;
        const Real ra = (a - B.center).length(), rb = (b - B.center).length();
        const Real thA = std::atan2(a.y - B.center.y, a.x - B.center.x);
        const Real rf = freewayR(thA);
        if ((ra - rf) * (rb - rf) <= 0) {
            const Real t = std::fabs(ra - rb) > 1e-9 ? (rf - ra) / (rb - ra) : 0.5;
            const Vec2 p = a + (b - a) * t;
            if (plan.ring.empty() && (plan.ringArc.empty() || distToPolyline(p, plan.ringArc, false) > 40.0)) continue;   // no ring there
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
        if (shaped && shape.area > 0) {
            const Real d = shape.depthAt(centroid(pb.buildable));
            pb.district = d >= shapeCore ? 0 : d >= shapeRim ? 1 : 2;
        } else {
            const Real r = (centroid(pb.buildable) - B.center).length();
            pb.district = r < B.coreRadius ? 0 : r < B.midRadius ? 1 : 2;
        }
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
