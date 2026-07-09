#include "metro.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace engine {
namespace {

constexpr double kTau = 6.283185307179586;

// Small deterministic PRNG (SplitMix64-ish), so a metro is reproducible per seed
// without pulling in the district Rng.
struct Lcg {
    std::uint64_t s;
    double unit() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>((s >> 11) & ((1ULL << 53) - 1)) / 9007199254740992.0;
    }
    double range(double a, double b) { return a + (b - a) * unit(); }
};

double dist2(const Vec2& a, const Vec2& b) { Vec2 d = a - b; return d.x * d.x + d.y * d.y; }

// The chord a bisecting line lays across a face (engine-faithful, from district.cpp).
bool cutSpan(const Poly2& poly, const Vec2& pt, const Vec2& dir, Vec2& a, Vec2& b) {
    Vec2 n(-dir.y, dir.x);
    double c = n.x * pt.x + n.y * pt.y;
    double tMin = 1e30, tMax = -1e30; int hits = 0;
    const int m = static_cast<int>(poly.size());
    for (int i = 0; i < m; ++i) {
        Vec2 p0 = poly[i], p1 = poly[(i + 1) % m];
        double d0 = n.x * p0.x + n.y * p0.y - c, d1 = n.x * p1.x + n.y * p1.y - c;
        if ((d0 <= 0 && d1 > 0) || (d0 > 0 && d1 <= 0)) {
            double u = d0 / (d0 - d1);
            Vec2 x = p0 + (p1 - p0) * u;
            double t = (x.x - pt.x) * dir.x + (x.y - pt.y) * dir.y;
            tMin = std::min(tMin, t); tMax = std::max(tMax, t); ++hits;
        }
    }
    if (hits < 2) return false;
    a = pt + dir * tMin; b = pt + dir * tMax;
    return true;
}

// Recursively bisect a face along its long OBB axis until block-sized, recording
// each cut as a street segment (the same recipe as district::subdivideStreets).
void subdivide(const Poly2& poly, double mn, double mx, double jitter, Lcg& rng,
               std::vector<std::pair<Vec2, Vec2>>& streets, int depth = 0) {
    if (poly.size() < 3 || depth > 22) return;
    OBB2 o = orientedBoundingBox(poly);
    int la = o.longAxis();
    double longHalf = o.half[la], shortHalf = o.half[1 - la];
    if (longHalf * 2.0 <= mx || longHalf < mn || shortHalf * 2.0 < mn) return;
    Vec2 dirLong = o.axis[la], cutDir = o.axis[1 - la];
    double off = (rng.unit() - 0.5) * 2.0 * jitter * longHalf;
    Vec2 sp = o.center + dirLong * off;
    Poly2 left, right;
    splitByLine(poly, sp, cutDir, left, right);
    if (left.size() < 3 || right.size() < 3) return;
    Vec2 a, b;
    if (cutSpan(poly, sp, cutDir, a, b)) streets.push_back({a, b});
    subdivide(left, mn, mx, jitter, rng, streets, depth + 1);
    subdivide(right, mn, mx, jitter, rng, streets, depth + 1);
}

// Connected components of a graph over its edges. Fills `comp` (−1 for isolated
// nodes) and returns the component count.
int components(const RoadGraph& g, std::vector<int>& comp) {
    std::vector<std::vector<int>> adj(g.nodes.size());
    for (const RoadEdge& e : g.edges) { adj[e.a].push_back(e.b); adj[e.b].push_back(e.a); }
    comp.assign(g.nodes.size(), -1);
    int n = 0;
    for (std::size_t s = 0; s < g.nodes.size(); ++s) {
        if (comp[s] != -1 || adj[s].empty()) continue;
        std::vector<int> st{static_cast<int>(s)}; comp[s] = n;
        while (!st.empty()) {
            int u = st.back(); st.pop_back();
            for (int v : adj[u]) if (comp[v] == -1) { comp[v] = n; st.push_back(v); }
        }
        ++n;
    }
    return n;
}

}  // namespace

RoadGraph buildMetro(const MetroParams& p) {
    Lcg rng{0x9E3779B97F4A7C15ULL ^ (static_cast<std::uint64_t>(p.seed) * 0x100000001B3ULL)};
    const double DOM = std::max(200.0, p.radius);
    const int H = std::max(2, p.hotspots);

    // --- hotspots: one central hub + the rest on a jittered ring; ~1 in 3 radial.
    // Terrain-aware gate: when a ground sampler is supplied, the city may only
    // occupy buildable land (not water, not steep mountain).
    const bool gated = static_cast<bool>(p.ground);
    auto buildable = [&](double x, double z) {
        return !gated || isBuildable(p.build, p.ground, x, z);
    };
    auto inDomain = [&](const Vec2& q) {
        return std::fabs(q.x - p.center.x) <= DOM && std::fabs(q.y - p.center.y) <= DOM;
    };
    // Nudge a point to the nearest buildable ground (spiral search); returns the
    // input unchanged if none is found within the domain.
    auto snapBuildable = [&](Vec2 s) -> Vec2 {
        if (buildable(s.x, s.y)) return s;
        for (double r = 25; r <= DOM; r += 25)
            for (int k = 0; k < 16; ++k) {
                double t = kTau * k / 16;
                Vec2 q = s + Vec2(std::cos(t), std::sin(t)) * r;
                if (inDomain(q) && buildable(q.x, q.y)) return q;
            }
        return s;
    };

    struct Hot { Vec2 pos; bool radial; };
    std::vector<Hot> hots;
    hots.push_back({snapBuildable(p.center), true});
    for (int i = 1; i < H; ++i) {
        double ang = kTau * i / (H - 1) + rng.range(-0.35, 0.35);
        double r = DOM * rng.range(0.42, 0.74);
        hots.push_back({snapBuildable(p.center + Vec2(std::cos(ang), std::sin(ang)) * r), (i % 3) == 0});
    }

    // --- attractors: dense along inter-hotspot corridors + light ambient wander.
    std::vector<Vec2> attr;
    for (std::size_t i = 0; i < hots.size(); ++i)
        for (std::size_t j = i + 1; j < hots.size(); ++j) {
            double L = (hots[j].pos - hots[i].pos).length();
            int n = static_cast<int>(L / 55.0);
            for (int k = 0; k < n; ++k) {
                double t = (k + 0.5) / std::max(1, n);
                Vec2 m = hots[i].pos * (1.0 - t) + hots[j].pos * t;
                Vec2 q{m.x + rng.range(-55, 55), m.y + rng.range(-55, 55)};
                if (buildable(q.x, q.y)) attr.push_back(q);   // corridor stays on land
            }
        }
    // A little ambient wander (kept low): too much sends arterials shooting into
    // empty land as long tendrils that add length/nodes without enclosing blocks.
    int ambient = static_cast<int>(H * 3);
    for (int k = 0; k < ambient; ++k) {
        Vec2 q{p.center.x + rng.range(-DOM, DOM), p.center.y + rng.range(-DOM, DOM)};
        if (buildable(q.x, q.y)) attr.push_back(q);
    }

    // --- multi-source space colonization: a growth tree seeded at each hotspot.
    std::vector<Vec2> node; std::vector<int> src; std::vector<std::pair<int, int>> aedge;
    auto addN = [&](const Vec2& q, int s) { node.push_back(q); src.push_back(s); return static_cast<int>(node.size()) - 1; };
    for (std::size_t h = 0; h < hots.size(); ++h) addN(hots[h].pos, static_cast<int>(h));
    const double SEG = 18, INFL = 240, KILL = 48, MERGE = 34;
    std::vector<char> alive(attr.size(), 1);
    for (int iter = 0; iter < 6000; ++iter) {
        std::vector<Vec2> pull(node.size(), Vec2(0, 0));
        std::vector<int> votes(node.size(), 0);
        int remaining = 0;
        for (std::size_t a = 0; a < attr.size(); ++a) {
            if (!alive[a]) continue; ++remaining;
            int best = -1; double bd = INFL * INFL;
            for (std::size_t n = 0; n < node.size(); ++n) { double q = dist2(attr[a], node[n]); if (q < bd) { bd = q; best = static_cast<int>(n); } }
            if (best >= 0) { pull[best] = pull[best] + normalize(attr[a] - node[best]); ++votes[best]; }
        }
        if (!remaining) break;
        int grew = 0; std::size_t base = node.size();
        for (std::size_t n = 0; n < base; ++n) {
            if (!votes[n]) continue;
            Vec2 dir = normalize(pull[n]); if (dir.length() < 1e-6) continue;
            Vec2 np = node[n] + dir * SEG;
            if (np.x < p.center.x - DOM || np.x > p.center.x + DOM ||
                np.y < p.center.y - DOM || np.y > p.center.y + DOM) continue;
            if (!buildable(np.x, np.y)) continue;   // don't grow into water / up the mountain
            int hit = -1;
            for (std::size_t m = 0; m < node.size(); ++m)
                if (src[m] != src[n] && dist2(np, node[m]) < MERGE * MERGE) { hit = static_cast<int>(m); break; }
            if (hit >= 0) { aedge.push_back({static_cast<int>(n), hit}); continue; }
            int ni = addN(np, src[n]); aedge.push_back({static_cast<int>(n), ni}); ++grew;
        }
        for (std::size_t a = 0; a < attr.size(); ++a) {
            if (!alive[a]) continue;
            for (std::size_t n = 0; n < node.size(); ++n) if (dist2(attr[a], node[n]) < KILL * KILL) { alive[a] = 0; break; }
        }
        if (!grew) break;
    }

    // --- arterial graph.
    RoadGraph Ga;
    std::vector<int> amap(node.size());
    for (std::size_t i = 0; i < node.size(); ++i) amap[i] = Ga.addNode(node[i], 6.0);
    for (const auto& e : aedge) Ga.addEdge(amap[e.first], amap[e.second], p.arteryWidth, RoadClass::Arterial);

    // --- stitch the separately-grown trees into ONE network (greedy nearest link).
    for (int guard = 0; guard < 40; ++guard) {
        std::vector<int> c; int n = components(Ga, c); if (n <= 1) break;
        double bd = 1e30; int ba = -1, bb = -1;
        std::vector<int> ar;
        for (std::size_t i = 0; i < Ga.nodes.size(); ++i) if (c[i] >= 0) ar.push_back(static_cast<int>(i));
        for (std::size_t x = 0; x < ar.size(); ++x)
            for (std::size_t y = x + 1; y < ar.size(); ++y) {
                int i = ar[x], j = ar[y]; if (c[i] == c[j]) continue;
                double q = dist2(Ga.nodes[i].pos, Ga.nodes[j].pos); if (q < bd) { bd = q; ba = i; bb = j; }
            }
        if (ba < 0) break; Ga.addEdge(ba, bb, p.arteryWidth, RoadClass::Arterial);
    }

    // --- close a few loops so the arterials enclose blocks (a tree encloses none):
    // link nodes near in space but far along the graph. Optionally a ring road too.
    {
        std::vector<std::vector<int>> adj(Ga.nodes.size());
        for (const RoadEdge& e : Ga.edges) { adj[e.a].push_back(e.b); adj[e.b].push_back(e.a); }
        const double maxLoop = 170, minLoop = 80; const int hopBar = 9;
        std::vector<std::pair<int, int>> links;
        for (std::size_t i = 0; i < Ga.nodes.size(); ++i) {
            if (adj[i].empty() || (i % 4) != 0) continue;
            std::vector<int> d(Ga.nodes.size(), -1), q{static_cast<int>(i)}; d[i] = 0; std::size_t qi = 0;
            while (qi < q.size()) { int u = q[qi++]; if (d[u] >= hopBar) continue; for (int v : adj[u]) if (d[v] < 0) { d[v] = d[u] + 1; q.push_back(v); } }
            int best = -1; double bd = maxLoop * maxLoop;
            for (std::size_t j = 0; j < Ga.nodes.size(); ++j) {
                if (j == i || adj[j].empty() || d[j] >= 0) continue;
                double dd = dist2(Ga.nodes[i].pos, Ga.nodes[j].pos);
                if (dd < bd && dd > minLoop * minLoop) { bd = dd; best = static_cast<int>(j); }
            }
            if (best >= 0 && static_cast<int>(i) < best) links.push_back({static_cast<int>(i), best});
        }
        for (const auto& l : links) Ga.addEdge(l.first, l.second, p.arteryWidth, RoadClass::Arterial);
        if (p.ringRoad) {
            double rr = DOM * 0.72; int segs = std::max(24, static_cast<int>(kTau * rr / 40.0));
            int prev = -1, first = -1;
            for (int k = 0; k <= segs; ++k) {
                double a = kTau * k / segs;
                int id = Ga.addNode(p.center + Vec2(std::cos(a), std::sin(a)) * rr, 6.0);
                if (prev >= 0) Ga.addEdge(prev, id, p.arteryWidth, RoadClass::Arterial); else first = id;
                prev = id;
            }
            if (first >= 0 && prev >= 0 && first != prev) Ga.addEdge(prev, first, p.arteryWidth, RoadClass::Arterial);
        }
    }
    Ga = planarize(Ga, 1.0);

    // --- fill each enclosed block: subdivide (grid) or rings+spokes (radial hub).
    std::vector<Poly2> faces = extractBlocks(Ga, std::max(200.0, p.blockSize * p.blockSize * 0.08));
    RoadGraph full = Ga;
    auto nearestHot = [&](const Vec2& q) {
        int b = 0; double bd = 1e30;
        for (std::size_t h = 0; h < hots.size(); ++h) { double d = dist2(q, hots[h].pos); if (d < bd) { bd = d; b = static_cast<int>(h); } }
        return b;
    };
    const double mn = p.blockSize * 0.6, mx = p.blockSize * 1.18;
    for (const Poly2& f : faces) {
        if (f.size() < 3) continue;
        Vec2 c = centroid(f);
        if (gated && !buildable(c.x, c.y)) continue;   // don't fill a block that's mostly water/steep
        int h = nearestHot(c);
        if (hots[h].radial && pointInPolygon(f, hots[h].pos)) {
            Vec2 C = hots[h].pos; int spokes = 12; double ringStep = std::max(40.0, p.blockSize * 0.8);
            double R = 1e30; for (const Vec2& v : f) R = std::min(R, (v - C).length()); R *= 0.96;
            std::vector<std::vector<int>> sn(spokes);
            for (int sd = 0; sd < spokes; ++sd) {
                double t = kTau * sd / spokes; int prev = full.addNode(C, 6.0);
                for (double r = ringStep; r <= R; r += ringStep) {
                    Vec2 q{C.x + std::cos(t) * r, C.y + std::sin(t) * r};
                    if (!pointInPolygon(f, q)) break;
                    int ni = full.addNode(q, 6.0); full.addEdge(prev, ni, p.streetWidth, RoadClass::Local); prev = ni; sn[sd].push_back(ni);
                }
            }
            int rings = 0; for (auto& v : sn) rings = std::max(rings, static_cast<int>(v.size()));
            for (int r = 0; r < rings; ++r)
                for (int sd = 0; sd < spokes; ++sd) {
                    int s2 = (sd + 1) % spokes;
                    if (r < static_cast<int>(sn[sd].size()) && r < static_cast<int>(sn[s2].size()))
                        full.addEdge(sn[sd][r], sn[s2][r], p.streetWidth, RoadClass::Local);
                }
        } else {
            std::vector<std::pair<Vec2, Vec2>> streets;
            subdivide(f, mn, mx, 0.16, rng, streets);
            for (const auto& s : streets) {
                int a = full.addNode(s.first, 6.0), b = full.addNode(s.second, 6.0);
                full.addEdge(a, b, p.streetWidth, RoadClass::Local);
            }
        }
    }
    return planarize(full, 1.0);
}

}  // namespace engine
