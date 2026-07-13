#include "metro.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine {
namespace {

constexpr double kTau = 6.283185307179586;

// Uniform hash grid over Vec2 point sets: the spatial accelerator that lifts
// the space-colonization inner loops (nearest-node, merge, kill) from O(n) per
// query to O(cell). Deterministic: queries scan cells in a fixed ring order and
// candidates in insertion order.
class PointGrid {
public:
    explicit PointGrid(double cell) : cell_(cell) {}

    void insert(const Vec2& q, int idx) {
        cells_[key(cx(q.x), cy(q.y))].push_back(idx);
    }

    // Nearest point within maxR; -1 when none. `pts` is the backing array the
    // stored indices point into; `skip` filters candidates (return true = skip).
    template <typename Skip>
    int nearest(const std::vector<Vec2>& pts, const Vec2& q, double maxR,
                Skip&& skip) const {
        const int R = static_cast<int>(std::ceil(maxR / cell_));
        const int qx = cx(q.x), qy = cy(q.y);
        int best = -1;
        double bd = maxR * maxR;
        for (int ring = 0; ring <= R; ++ring) {
            // Once a hit exists, stop after the first ring that cannot beat it.
            if (best >= 0) {
                double reach = (ring - 1) * cell_;
                if (reach > 0 && reach * reach > bd) break;
            }
            for (int dy = -ring; dy <= ring; ++dy)
                for (int dx = -ring; dx <= ring; ++dx) {
                    if (std::max(std::abs(dx), std::abs(dy)) != ring) continue;
                    auto it = cells_.find(key(qx + dx, qy + dy));
                    if (it == cells_.end()) continue;
                    for (int i : it->second) {
                        if (skip(i)) continue;
                        Vec2 d = pts[i] - q;
                        double dd = d.x * d.x + d.y * d.y;
                        if (dd < bd) { bd = dd; best = i; }
                    }
                }
        }
        return best;
    }

    // Visit every stored index within `r` of q.
    template <typename Fn>
    void each(const Vec2& q, double r, Fn&& fn) const {
        const int R = static_cast<int>(std::ceil(r / cell_));
        const int qx = cx(q.x), qy = cy(q.y);
        for (int dy = -R; dy <= R; ++dy)
            for (int dx = -R; dx <= R; ++dx) {
                auto it = cells_.find(key(qx + dx, qy + dy));
                if (it == cells_.end()) continue;
                for (int i : it->second) fn(i);
            }
    }

private:
    int cx(double x) const { return static_cast<int>(std::floor(x / cell_)); }
    int cy(double y) const { return static_cast<int>(std::floor(y / cell_)); }
    static long long key(int x, int y) {
        return (static_cast<long long>(x) << 32) ^
               (static_cast<long long>(y) & 0xffffffffLL);
    }
    double cell_;
    std::unordered_map<long long, std::vector<int>> cells_;
};

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

// One recorded subdivision cut: a street segment plus the tier it belongs to
// (a cut across a face still wider than the collector span is a COLLECTOR —
// the distributor between the arterials and the local grid).
struct Cut {
    Vec2 a, b;
    bool collector = false;
};

// Recursively bisect a face along its long OBB axis until block-sized, recording
// each cut as a street segment (the same recipe as district::subdivideStreets).
// `collectorSpan` <= 0 disables the collector tier (every cut is Local).
void subdivide(const Poly2& poly, double mn, double mx, double jitter,
               double collectorSpan, Lcg& rng, std::vector<Cut>& streets,
               int depth = 0) {
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
    if (cutSpan(poly, sp, cutDir, a, b))
        streets.push_back({a, b, collectorSpan > 0 && longHalf * 2.0 > collectorSpan});
    subdivide(left, mn, mx, jitter, collectorSpan, rng, streets, depth + 1);
    subdivide(right, mn, mx, jitter, collectorSpan, rng, streets, depth + 1);
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

RoadGraph buildMetro(const MetroParams& p,
                     std::vector<std::vector<Vec2>>* freewayPlans) {
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

    // Hub kinds mirror DistrictTag: the centre is the financial core; ring hubs
    // draw from a weighted flavor cycle (commerce-heavy near the core is the
    // architect's job — here each SECONDARY CENTRE gets one dominant flavor).
    std::vector<CityHub> hots;
    hots.push_back({snapBuildable(p.center), /*kind=*/0, /*radial=*/true});
    const int kindCycle[6] = {1, 2, 4, 1, 2, 3};   // commercial, residential,
                                                   // industrial, ..., oldtown
    // One jittered ring up to 7 hubs; bigger metros add an OUTER ring so the
    // footprint is covered by centres instead of leaving a bald mid-band.
    const int ring1 = std::min(H - 1, 6);
    for (int i = 1; i < H; ++i) {
        const bool outer = (i - 1) >= ring1;
        const int k = outer ? (i - 1 - ring1) : (i - 1);
        const int n = outer ? (H - 1 - ring1) : ring1;
        double ang = kTau * k / std::max(1, n) + rng.range(-0.35, 0.35) +
                     (outer ? kTau * 0.5 / std::max(1, n) : 0.0);
        double r = DOM * (outer ? rng.range(0.60, 0.80) : rng.range(0.34, 0.55));
        hots.push_back({snapBuildable(p.center + Vec2(std::cos(ang), std::sin(ang)) * r),
                        kindCycle[(i - 1) % 6], (i % 3) == 0});
    }
    if (p.outHubs) *p.outHubs = hots;

    // --- freeway backbone (metropolis tier): connect the hubs with an MST plus
    // a few nearest extras, each link a gently-curved polyline. Laid down FIRST
    // so everything else grows around it; interchange-spaced seeds along the
    // lines join the arterial colonization below.
    RoadGraph Ga;
    std::vector<Vec2> seedPts;                      // extra colonization sources
    if (p.freeways && H >= 2) {
        // MST over hubs (Prim) + each hub's nearest non-tree neighbour when the
        // link is under ~1.2 DOM (adds loops without a hairball).
        std::vector<std::pair<int, int>> links;
        std::vector<char> inTree(H, 0);
        inTree[0] = 1;
        for (int added = 1; added < H; ++added) {
            double bd = 1e30; int bi = -1, bj = -1;
            for (int i = 0; i < H; ++i) if (inTree[i])
                for (int j = 0; j < H; ++j) if (!inTree[j]) {
                    double d = dist2(hots[i].pos, hots[j].pos);
                    if (d < bd) { bd = d; bi = i; bj = j; }
                }
            if (bj < 0) break;
            inTree[bj] = 1;
            links.push_back({bi, bj});
        }
        auto linked = [&](int i, int j) {
            for (auto& l : links)
                if ((l.first == i && l.second == j) || (l.first == j && l.second == i))
                    return true;
            return false;
        };
        for (int i = 0; i < H; ++i) {
            double bd = 1.2 * DOM * 1.2 * DOM; int bj = -1;
            for (int j = 0; j < H; ++j) {
                if (j == i || linked(i, j)) continue;
                double d = dist2(hots[i].pos, hots[j].pos);
                if (d < bd) { bd = d; bj = j; }
            }
            if (bj >= 0 && i < bj) links.push_back({i, bj});
        }
        // Each link: anchor points every ~220 m with small perpendicular sway
        // (freeways curve gently — DesignRules Freeway minRadius is 300 m), then
        // segments every ~36 m. Unbuildable anchors slide sideways to stay on
        // land (a coastal freeway hugs the shoreline instead of wading).
        if (!p.corridorFreeways) {
            // LEGACY freeway tier: the backbone stays street edges (proven,
            // shipping) while the corridor path earns its lab proof.
            for (auto& l : links) {
                Vec2 A = hots[l.first].pos, B = hots[l.second].pos;
                Vec2 dir = B - A;
                double L = dir.length();
                if (L < 1.0) continue;
                dir = dir * (1.0 / L);
                Vec2 nrm(-dir.y, dir.x);
                int anchors = std::max(2, static_cast<int>(L / 220.0) + 1);
                std::vector<Vec2> pts;
                for (int k = 0; k <= anchors; ++k) {
                    double t = static_cast<double>(k) / anchors;
                    double sway = (k == 0 || k == anchors)
                                      ? 0.0
                                      : rng.range(-0.05, 0.05) * L;
                    Vec2 q = A + dir * (t * L) + nrm * sway;
                    if (!buildable(q.x, q.y)) {
                        bool fixed = false;
                        for (double off = 30; off <= 210 && !fixed; off += 30)
                            for (double s : {+1.0, -1.0}) {
                                Vec2 c2 = q + nrm * (off * s);
                                if (inDomain(c2) && buildable(c2.x, c2.y)) { q = c2; fixed = true; break; }
                            }
                    }
                    pts.push_back(q);
                }
                auto cr = [&](int i0) {
                    Vec2 p0 = pts[std::max(0, i0 - 1)], p1 = pts[i0],
                         p2 = pts[std::min<int>(pts.size() - 1, i0 + 1)],
                         p3 = pts[std::min<int>(pts.size() - 1, i0 + 2)];
                    return [=](double t) {
                        double t2 = t * t, t3 = t2 * t;
                        return (p1 * 2.0 + (p2 - p0) * t +
                                (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * t2 +
                                ((p1 - p2) * 3.0 + p3 - p0) * t3) * 0.5;
                    };
                };
                int prev = -1;
                double along = 0, nextSeed = p.interchangeSpacing * 0.5;
                for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
                    auto seg = cr(static_cast<int>(k));
                    double segLen = (pts[k + 1] - pts[k]).length();
                    int steps = std::max(1, static_cast<int>(segLen / 36.0));
                    for (int s = (k == 0 ? 0 : 1); s <= steps; ++s) {
                        Vec2 q = seg(static_cast<double>(s) / steps);
                        int id = Ga.addNode(q, 8.0);
                        if (prev >= 0 && id != prev)
                            Ga.addEdge(prev, id, p.freewayWidth, RoadClass::Freeway);
                        if (prev >= 0) {
                            along += segLen / steps;
                            if (along >= nextSeed) {
                                seedPts.push_back(q);
                                nextSeed += p.interchangeSpacing;
                            }
                        }
                        prev = id;
                    }
                }
            }
        } else {
        // §10.6: chain the MST links into THROUGH-ROUTES — a freeway passes
        // through hubs, it doesn't stop at each one. Walk maximal paths,
        // preferring the straightest continuation at every hub, so 6 short
        // hub-to-hub stubs become 2-3 corridors long enough for real
        // interchanges.
        std::vector<std::vector<int>> hubAdj(H);
        for (std::size_t li2 = 0; li2 < links.size(); ++li2) {
            hubAdj[links[li2].first].push_back(static_cast<int>(li2));
            hubAdj[links[li2].second].push_back(static_cast<int>(li2));
        }
        std::vector<char> usedLink(links.size(), 0);
        std::vector<std::vector<int>> routes;   // hub index sequences
        auto walk = [&](int startHub, int firstLink) {
            std::vector<int> seq{startHub};
            int hub = startHub, link = firstLink;
            while (link >= 0 && !usedLink[link]) {
                usedLink[link] = 1;
                hub = links[link].first == hub ? links[link].second
                                               : links[link].first;
                seq.push_back(hub);
                // straightest unvisited continuation
                const Vec2 inDir = normalize(hots[hub].pos -
                                             hots[seq[seq.size() - 2]].pos);
                int best = -1;
                double bestDot = 0.2;   // never turn harder than ~78 degrees
                for (int cand : hubAdj[hub]) {
                    if (usedLink[cand]) continue;
                    const int other = links[cand].first == hub
                                          ? links[cand].second
                                          : links[cand].first;
                    const double d =
                        dot(inDir, normalize(hots[other].pos - hots[hub].pos));
                    if (d > bestDot) { bestDot = d; best = cand; }
                }
                link = best;
            }
            if (seq.size() >= 2) routes.push_back(std::move(seq));
        };
        {   // start at odd-degree hubs first (route ends), then leftovers
            std::vector<int> order(H);
            for (int i = 0; i < H; ++i) order[i] = i;
            std::sort(order.begin(), order.end(), [&](int a, int b) {
                return hubAdj[a].size() < hubAdj[b].size();
            });
            for (int hub : order)
                for (int cand : hubAdj[hub])
                    if (!usedLink[cand]) walk(hub, cand);
        }
        for (const std::vector<int>& seq : routes) {
            // anchors every ~220 m along EACH leg with small sway, buildable
            // slide as before — one polyline for the whole through-route
            std::vector<Vec2> pts;
            for (std::size_t si = 0; si + 1 < seq.size(); ++si) {
                Vec2 A = hots[seq[si]].pos, B = hots[seq[si + 1]].pos;
                Vec2 dir = B - A;
                double L = dir.length();
                if (L < 1.0) continue;
                dir = dir * (1.0 / L);
                Vec2 nrm(-dir.y, dir.x);
                int anchors = std::max(2, static_cast<int>(L / 220.0) + 1);
                for (int k = (si == 0 ? 0 : 1); k <= anchors; ++k) {
                    double t = static_cast<double>(k) / anchors;
                    double sway = (k == 0 || k == anchors)
                                      ? 0.0
                                      : rng.range(-0.05, 0.05) * L;
                    Vec2 q = A + dir * (t * L) + nrm * sway;
                    if (!buildable(q.x, q.y)) {
                        bool fixed = false;
                        for (double off = 30; off <= 210 && !fixed; off += 30)
                            for (double s : {+1.0, -1.0}) {
                                Vec2 c = q + nrm * (off * s);
                                if (inDomain(c) && buildable(c.x, c.y)) { q = c; fixed = true; break; }
                            }
                    }
                    pts.push_back(q);
                }
            }
            if (pts.size() < 2) continue;
            // §10.6: the route becomes a CORRIDOR PLAN — anchor polyline out,
            // NO street edges (the freeway-as-fat-street era ends here). The
            // interchange seeds stay: streets grow toward the future ramps.
            if (freewayPlans) freewayPlans->push_back(pts);
            double along = 0, nextSeed = p.interchangeSpacing * 0.5;
            for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
                const double segLen = (pts[k + 1] - pts[k]).length();
                along += segLen;
                while (along >= nextSeed) {
                    const double back = (along - nextSeed) / std::max(1.0, segLen);
                    seedPts.push_back(pts[k + 1] - (pts[k + 1] - pts[k]) * back);
                    nextSeed += p.interchangeSpacing;
                }
            }
        }
        }   // corridorFreeways
    }

    // --- attractors: dense along inter-hotspot corridors + ambient wander that
    // scales with the FOOTPRINT AREA (a fixed count starves a 2 km domain).
    std::vector<Vec2> attr;
    for (std::size_t i = 0; i < hots.size(); ++i)
        for (std::size_t j = i + 1; j < hots.size(); ++j) {
            double L = (hots[j].pos - hots[i].pos).length();
            int n = static_cast<int>(L / p.corridorSpacing);
            for (int k = 0; k < n; ++k) {
                double t = (k + 0.5) / std::max(1, n);
                Vec2 m = hots[i].pos * (1.0 - t) + hots[j].pos * t;
                Vec2 q{m.x + rng.range(-p.corridorSpacing, p.corridorSpacing),
                       m.y + rng.range(-p.corridorSpacing, p.corridorSpacing)};
                if (buildable(q.x, q.y)) attr.push_back(q);   // corridor stays on land
            }
        }
    // Ambient wander: ~90 per (500 m)^2 of domain. Colonization STOPS where no
    // attractor sits within INFL of the growing trees, so the ambient field is
    // what carries growth across the whole footprint — sparse ambient strands
    // whole quarters (device: the southern hubs never grew streets). Tendrils
    // stay bounded by the loop-closing pass + block subdivision.
    int ambient = std::max(H * 3, static_cast<int>(p.ambientPer500 * (DOM / 500.0) * (DOM / 500.0)));
    for (int k = 0; k < ambient; ++k) {
        Vec2 q{p.center.x + rng.range(-DOM, DOM), p.center.y + rng.range(-DOM, DOM)};
        if (buildable(q.x, q.y)) attr.push_back(q);
    }

    // --- multi-source space colonization: a growth tree seeded at each hotspot
    // and at each freeway interchange. Grid-accelerated (PointGrid) so a 2 km
    // metro stays out of the old O(iterations x attractors x nodes) wall.
    std::vector<Vec2> node; std::vector<int> src; std::vector<std::pair<int, int>> aedge;
    const double SEG = p.segLength, INFL = p.influence, KILL = p.killRadius,
                 MERGE = p.mergeRadius;
    PointGrid nodeGrid(64.0);
    auto addN = [&](const Vec2& q, int s) {
        node.push_back(q); src.push_back(s);
        int id = static_cast<int>(node.size()) - 1;
        nodeGrid.insert(q, id);
        return id;
    };
    for (std::size_t h = 0; h < hots.size(); ++h) addN(hots[h].pos, static_cast<int>(h));
    for (std::size_t s = 0; s < seedPts.size(); ++s)
        addN(seedPts[s], static_cast<int>(hots.size() + s));
    PointGrid attrGrid(64.0);
    for (std::size_t a = 0; a < attr.size(); ++a) attrGrid.insert(attr[a], static_cast<int>(a));
    std::vector<char> alive(attr.size(), 1);
    // Kill attractors already inside a seed's kill radius.
    for (std::size_t n = 0; n < node.size(); ++n)
        attrGrid.each(node[n], KILL, [&](int a) {
            if (alive[a] && dist2(attr[a], node[n]) < KILL * KILL) alive[a] = 0;
        });
    for (int iter = 0; iter < 20000; ++iter) {
        std::vector<Vec2> pull(node.size(), Vec2(0, 0));
        std::vector<int> votes(node.size(), 0);
        int remaining = 0;
        for (std::size_t a = 0; a < attr.size(); ++a) {
            if (!alive[a]) continue; ++remaining;
            int best = nodeGrid.nearest(node, attr[a], INFL, [](int) { return false; });
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
            int hit = nodeGrid.nearest(node, np, MERGE,
                                       [&](int m) { return src[m] == src[n]; });
            if (hit >= 0) { aedge.push_back({static_cast<int>(n), hit}); continue; }
            int ni = addN(np, src[n]); aedge.push_back({static_cast<int>(n), ni}); ++grew;
            attrGrid.each(np, KILL, [&](int a) {
                if (alive[a] && dist2(attr[a], np) < KILL * KILL) alive[a] = 0;
            });
        }
        if (!grew) break;
    }

    // --- arterial graph (merged onto the freeway backbone laid above).
    std::vector<int> amap(node.size());
    for (std::size_t i = 0; i < node.size(); ++i) amap[i] = Ga.addNode(node[i], 6.0);
    for (const auto& e : aedge) Ga.addEdge(amap[e.first], amap[e.second], p.arteryWidth, RoadClass::Arterial);

    // --- stitch the separately-grown trees into ONE network (greedy nearest
    // link, grid-accelerated: for every node of the smallest component, query
    // the nearest node of any OTHER component).
    for (int guard = 0; guard < 40; ++guard) {
        std::vector<int> c; int n = components(Ga, c); if (n <= 1) break;
        std::vector<int> csize(n, 0);
        for (std::size_t i = 0; i < Ga.nodes.size(); ++i) if (c[i] >= 0) ++csize[c[i]];
        int small = 0;
        for (int k = 1; k < n; ++k) if (csize[k] < csize[small]) small = k;
        std::vector<Vec2> pos(Ga.nodes.size());
        PointGrid g(96.0);
        for (std::size_t i = 0; i < Ga.nodes.size(); ++i) {
            pos[i] = Ga.nodes[i].pos;
            if (c[i] >= 0 && c[i] != small) g.insert(pos[i], static_cast<int>(i));
        }
        double bd = 1e30; int ba = -1, bb = -1;
        for (std::size_t i = 0; i < Ga.nodes.size(); ++i) {
            if (c[i] != small) continue;
            int j = g.nearest(pos, pos[i], 2.0 * DOM, [](int) { return false; });
            if (j < 0) continue;
            double q = dist2(pos[i], pos[j]);
            if (q < bd) { bd = q; ba = static_cast<int>(i); bb = j; }
        }
        if (ba < 0) break; Ga.addEdge(ba, bb, p.arteryWidth, RoadClass::Arterial);
    }

    // --- close a few loops so the arterials enclose blocks (a tree encloses none):
    // link nodes near in space but far along the graph. Optionally a ring road too.
    {
        std::vector<std::vector<int>> adj(Ga.nodes.size());
        for (const RoadEdge& e : Ga.edges) { adj[e.a].push_back(e.b); adj[e.b].push_back(e.a); }
        const double maxLoop = p.loopMax, minLoop = p.loopMin; const int hopBar = 9;
        std::vector<std::pair<int, int>> links;
        for (std::size_t i = 0; i < Ga.nodes.size(); ++i) {
            if (adj[i].empty() || (i % 3) != 0) continue;
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
            // §10.6: the ring stays an ARTERIAL — an orbital freeway would be
            // another corridor (future), never freeway-class street edges.
            const RoadClass rc = RoadClass::Arterial;
            const double rw = p.arteryWidth;
            double rr = DOM * 0.72; int segs = std::max(24, static_cast<int>(kTau * rr / 40.0));
            int prev = -1, first = -1;
            for (int k = 0; k <= segs; ++k) {
                double a = kTau * k / segs;
                int id = Ga.addNode(p.center + Vec2(std::cos(a), std::sin(a)) * rr, 6.0);
                if (prev >= 0) Ga.addEdge(prev, id, rw, rc); else first = id;
                prev = id;
            }
            if (first >= 0 && prev >= 0 && first != prev) Ga.addEdge(prev, first, rw, rc);
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
    // District flavor drives the grain: financial cores parcel tight, industry
    // parcels wide; old town is small and crooked. (Indexed by CityHub::kind.)
    const double kindBlockMul[5] = {0.72, 0.95, 1.15, 0.60, 1.70};
    const double kindJitter[5]   = {0.08, 0.14, 0.16, 0.30, 0.12};
    const double collectorSpan = p.collectorSpan > 0 ? p.collectorSpan : p.blockSize * 3.0;
    for (const Poly2& f : faces) {
        if (f.size() < 3) continue;
        Vec2 c = centroid(f);
        if (gated && !buildable(c.x, c.y)) continue;   // don't fill a block that's mostly water/steep
        int h = nearestHot(c);
        const int kind = std::clamp(hots[h].kind, 0, 4);
        const double mn = p.blockSize * kindBlockMul[kind] * 0.6;
        const double mx = p.blockSize * kindBlockMul[kind] * 1.18;
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
            std::vector<Cut> streets;
            subdivide(f, mn, mx, kindJitter[kind], collectorSpan, rng, streets);
            for (const Cut& s : streets) {
                int a = full.addNode(s.a, 6.0), b = full.addNode(s.b, 6.0);
                if (s.collector)
                    full.addEdge(a, b, p.collectorWidth, RoadClass::Collector);
                else
                    full.addEdge(a, b, p.streetWidth, RoadClass::Local);
            }
        }
    }
    return planarize(full, 1.0);
}

}  // namespace engine
