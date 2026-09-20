#include "road_net_mesh.h"

// DEPRECATED (2026-09-20): the lattice road builder's mesher, conform and walls,
// lifted verbatim out of procgen/city/road_net.cpp when the roads module split
// planning (the road entity, shared) from building (a swappable RoadBuilder).
// Still the builder every shipped level uses; see city/roads/road_builder.h.

#include "road_lattice.h"                 // swept-lattice street mesher (stage 3)
#include "../../city/roads/road_net_internal.h"
#include "../../city/road_offset.h"       // ribbonOutline + polygonUnion (S5 curb/sidewalk band)
#include "../../city/road_semantics.h"    // classifyRoadGraph
#include "../../city/road_constraints.h"  // applyConstraints, RoadRules
#include "../../city/road_rules.h"        // DesignRules (clearance, deck thickness, ramp grade)
#include "../../city/road_mesh.h"         // UnionSpine, roadProfile, weldChainProfiles
#include "../../../mesh_builder.h"           // MeshBuilder::append
#include "../../../../log.h"
#include "../../../../profile.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <limits>
#include <cstdlib>
#include <unordered_map>
#include <cstdio>

namespace engine {

using namespace roadnet;


namespace {
// Trim a chain to start `rA` / end `rB` into it (arc length), so its ends stop at
// the junction boundary and its swept end rings become the arm mouths.
UnionSpine trimSpine(const UnionSpine& s, double rA, double rB) {
    const int n = static_cast<int>(s.points.size());
    if (n < 2) return s;
    std::vector<double> cum(n, 0.0);
    for (int i = 1; i < n; ++i) cum[i] = cum[i - 1] + (s.points[i] - s.points[i - 1]).length();
    const double L = cum.back();
    const double a = std::min(rA, L * 0.45);
    const double b = std::max(L - std::min(rB, L * 0.45), a + 0.5);
    const bool hasY = static_cast<int>(s.yAbs.size()) == n;
    auto at = [&](double d, Vec2& p, double& y) {
        int i = 0; while (i + 1 < n && cum[i + 1] < d) ++i;
        const double seg = cum[i + 1] - cum[i];
        const double t = seg > 1e-9 ? (d - cum[i]) / seg : 0.0;
        p = s.points[i] + (s.points[i + 1] - s.points[i]) * t;
        if (hasY) y = s.yAbs[i] + (s.yAbs[i + 1] - s.yAbs[i]) * t;
    };
    UnionSpine o; o.halfWidth = s.halfWidth; o.klass = s.klass;
    o.access = s.access; o.accessBack = s.accessBack;
    Vec2 p; double y = 0;
    at(a, p, y); o.points.push_back(p); if (hasY) o.yAbs.push_back(y);
    for (int i = 0; i < n; ++i)
        if (cum[i] > a + 1e-6 && cum[i] < b - 1e-6) {
            o.points.push_back(s.points[i]); if (hasY) o.yAbs.push_back(s.yAbs[i]);
        }
    at(b, p, y); o.points.push_back(p); if (hasY) o.yAbs.push_back(y);
    return o;
}
}  // namespace

namespace {
// Nearest-asphalt-edge height sampler for the S5 curb/sidewalk band: segments
// (chain centrelines with their reconciled profile heights + pad boundary
// loops) in a coarse grid hash; sample = height at the nearest point on the
// nearest segment. The band hugs the asphalt within ~7 m, so a small search
// window finds its segment; brute force is the (rare) fallback.
struct EdgeHeightField {
    struct Seg { Vec2 a, b; double ya, yb; };
    std::vector<Seg> segs;
    std::unordered_map<long long, std::vector<int>> cells;
    static constexpr double kCell = 12.0;
    static long long key(int cx, int cz) {
        return (static_cast<long long>(cx) << 32) ^
               (static_cast<long long>(cz) & 0xffffffffLL);
    }
    void addPolyline(const std::vector<Vec2>& pts, const std::vector<double>& ys) {
        for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
            const int si = static_cast<int>(segs.size());
            segs.push_back({ pts[i], pts[i + 1], ys[i], ys[i + 1] });
            const double x0 = std::min(pts[i].x, pts[i + 1].x) - 1.0;
            const double x1 = std::max(pts[i].x, pts[i + 1].x) + 1.0;
            const double z0 = std::min(pts[i].y, pts[i + 1].y) - 1.0;
            const double z1 = std::max(pts[i].y, pts[i + 1].y) + 1.0;
            for (int cx = (int)std::floor(x0 / kCell); cx <= (int)std::floor(x1 / kCell); ++cx)
                for (int cz = (int)std::floor(z0 / kCell); cz <= (int)std::floor(z1 / kCell); ++cz)
                    cells[key(cx, cz)].push_back(si);
        }
    }
    void addLoop(const std::vector<Vec3>& loop) {
        if (loop.size() < 2) return;
        std::vector<Vec2> pts;
        std::vector<double> ys;
        for (const Vec3& p : loop) { pts.push_back(Vec2(p.x, p.z)); ys.push_back(p.y); }
        pts.push_back(Vec2(loop.front().x, loop.front().z));
        ys.push_back(loop.front().y);
        addPolyline(pts, ys);
    }
    double sample(double x, double z) const {
        // Inverse-distance BLEND over the nearby segments, not nearest-only:
        // at a junction corner the nearest chain flips between arms meeting at
        // different heights, and a nearest-snap put that height CLIFF straight
        // into the band (steep slab quads on hills). Blending keeps the band
        // flush where it hugs one edge (that weight dominates) and smooth
        // where two arms compete.
        const int cx = (int)std::floor(x / kCell), cz = (int)std::floor(z / kCell);
        std::vector<int> cand;                   // small: the 3x3 window's lists
        for (int r = 1; r <= 3 && cand.empty(); ++r)
            for (int dx = -r; dx <= r; ++dx)
                for (int dz = -r; dz <= r; ++dz) {
                    auto it = cells.find(key(cx + dx, cz + dz));
                    if (it != cells.end())
                        cand.insert(cand.end(), it->second.begin(), it->second.end());
                }
        if (cand.empty()) {                      // fallback: brute force
            cand.resize(segs.size());
            for (std::size_t i = 0; i < segs.size(); ++i) cand[i] = (int)i;
        }
        std::sort(cand.begin(), cand.end());
        cand.erase(std::unique(cand.begin(), cand.end()), cand.end());
        double wsum = 0.0, hsum = 0.0;
        for (int si : cand) {
            const Seg& sg = segs[si];
            const Vec2 ab = sg.b - sg.a;
            const double l2 = ab.lengthSquared();
            double t = l2 > 1e-12 ? dot(Vec2(x, z) - sg.a, ab) / l2 : 0.0;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            const double d = (sg.a + ab * t - Vec2(x, z)).length();
            const double w = 1.0 / (d * d + 0.5);
            wsum += w;
            hsum += w * (sg.ya + (sg.yb - sg.ya) * t);
        }
        return wsum > 0 ? hsum / wsum : 0.0;
    }
};
}  // namespace

RenderMesh buildRoadNetLattice(const RoadGraph& gIn,
                               const std::function<Real(Real, Real)>& heightAt,
                               std::vector<std::size_t>* chainTriEndsOut,
                               double sidewalkWidth, double curbHeight,
                               bool crosswalks, CurbBandAudit* auditOut,
                               double cornerRadius,
                               RoadDeckField* deckOut, bool perClassGrade) {
    // Semantic self-heal (#17): tests and tools hand this hand-built graphs
    // that never went through a producer — classify so kind-gated styling
    // behaves identically for them. Classified inputs pass through untouched
    // (classifyRoadGraph is idempotent; hints are preserved).
    RoadGraph healed;
    const RoadGraph& g = roadGraphUnclassified(gIn)
                             ? (healed = gIn,
                                classifyRoadGraph(
                                    healed,
                                    heightAt ? GroundFn([&heightAt](double x, double y) {
                                        return static_cast<double>(heightAt(x, y));
                                    })
                                             : GroundFn()),
                                healed)
                             : gIn;
    const int N = static_cast<int>(g.nodes.size());
    // Stage 1 of the junction re-architecture (docs/junction-weld-decision.md):
    // the junction owns the FULL cross-section. Trim each body by its full
    // half-width (carriageway + sidewalk), so an arm's raised sidewalk pulls
    // back out of the junction disc instead of sweeping into the pad and
    // double-covering it — the measured 85% of the 16% overlap.
    const double kSidewalkW = sidewalkWidth;
    std::vector<int> deg(N, 0);
    std::vector<double> rad(N, 0.0);
    for (const RoadEdge& e : g.edges) {
        ++deg[e.a]; ++deg[e.b];
        rad[e.a] = std::max(rad[e.a], static_cast<double>(e.width) * 0.5 + kSidewalkW);
        rad[e.b] = std::max(rad[e.b], static_cast<double>(e.width) * 0.5 + kSidewalkW);
    }
    auto ground = [&](double x, double z) { return heightAt ? (double)heightAt(x, z) : 0.0; };
    // Position -> node, O(1): the chain endpoints ARE node positions (weldChainSpines
    // uses g.nodes[v].pos), so a 0.5 m-cell hash finds them without the O(N) scan
    // that would make this O(chains * N) on a city.
    auto key = [](const Vec2& p) {
        return (static_cast<long long>(std::llround(p.x * 2.0)) << 32) ^
               (static_cast<long long>(std::llround(p.y * 2.0)) & 0xffffffffLL);
    };
    std::unordered_map<long long, int> nodeIndex;
    nodeIndex.reserve(N * 2);
    for (int v = 0; v < N; ++v) nodeIndex.emplace(key(g.nodes[v].pos), v);
    auto nodeAt = [&](const Vec2& q) {
        auto it = nodeIndex.find(key(q));
        return it == nodeIndex.end() ? -1 : it->second;
    };

    // Dump junction WORLD positions so a camera can be aimed at each one (we know
    // where every intersection is — no coordinate-guessing). Format: "x z degree".
    if (const char* path = std::getenv("RT_DUMP_JUNCTIONS")) {
        if (std::FILE* f = std::fopen(path, "w")) {
            for (int v = 0; v < N; ++v)
                if (deg[v] >= 3)
                    std::fprintf(f, "%.3f %.3f %d\n", (double)g.nodes[v].pos.x,
                                 (double)g.nodes[v].pos.y, deg[v]);
            std::fclose(f);
        }
    }

    RenderMesh out;
    const std::function<double(double, double)> groundFn = ground;   // for strips
    std::vector<std::vector<JunctionArm>> arms(N);
    // ONE profile source (Glenn: "roads under the terrain... floating ribbons").
    // The terrain carve grades to weldChainProfiles' reconciled, grade-limited
    // heights — but the lattice was draping each ring on RAW ground, so wherever
    // smoothing changed the height the road sat under (or above) the carved
    // terrain. Ride the SAME profiles the carve uses: mesh and terrain now agree
    // by construction, exactly like the weld path (plan P3.2).
    std::vector<UnionSpine> chains = weldChainSpines(g);
    // Mixed chains carry NaN at their at-grade nodes (a baked ramp's street
    // landing). weldChainProfiles READS yAbs as its authored input, so the
    // NaNs must die HERE, before any consumer: fill them with the draped
    // street surface at that point. NaN must never reach a profile or a mesh.
    for (UnionSpine& c : chains)
        for (std::size_t i = 0; i < c.yAbs.size(); ++i)
            if (std::isnan(c.yAbs[i]))
                c.yAbs[i] = groundFn(c.points[i].x, c.points[i].y) + 0.15;
    {
        // overlapReach = sidewalk + 4.0, the SAME value the carve and the walls
        // pass — it was a hardcoded 3.0 + 4.0 here, so on a 3.5 m sidewalk
        // level a sample 7.0-7.5 m outside another corridor was min'd in the
        // carve but not in the mesh (a deck/ground disagreement by design).
        std::vector<std::vector<double>> profiles = weldChainProfiles(
            chains, groundFn, 0.0, kRoadMaxGrade, sidewalkWidth + 4.0,
            perClassGrade ? &kDesignRules : nullptr);
        for (std::size_t si = 0; si < chains.size(); ++si) {
            const bool sized = profiles[si].size() == chains[si].points.size();
            if (chains[si].yAbs.empty()) {
                if (sized) chains[si].yAbs = profiles[si];
                continue;
            }
            // Mixed chain: authored deck heights win; the at-grade holes
            // (NaN — e.g. a ramp's street landing node) ride the same drape
            // profile a street would, so the descent meets the street exactly
            // where the street's own surface sits. If the profile pass
            // resampled (size mismatch), bridge NaN runs from the finite
            // neighbours instead — NaN must NEVER reach the mesh.
            std::vector<double>& ys = chains[si].yAbs;
            for (std::size_t i = 0; i < ys.size(); ++i) {
                if (!std::isnan(ys[i])) continue;
                if (sized) { ys[i] = profiles[si][i]; continue; }
                std::size_t lo = i; while (lo > 0 && std::isnan(ys[lo - 1])) --lo;
                std::size_t hi = i; while (hi + 1 < ys.size() && std::isnan(ys[hi + 1])) ++hi;
                const double before = lo > 0 ? ys[lo - 1]
                                             : (hi + 1 < ys.size() ? ys[hi + 1] : 0.0);
                const double after = hi + 1 < ys.size() ? ys[hi + 1] : before;
                for (std::size_t k = lo; k <= hi; ++k) {
                    const double t = (hi >= lo)
                        ? (double(k - lo) + 1.0) / (double(hi - lo) + 2.0) : 0.5;
                    ys[k] = before + (after - before) * t;
                }
                i = hi;
            }
        }
    }
    // THE DECK, exported (RoadDeckField): these chains' yAbs ARE the heights
    // every body ring below is swept at, so a consumer sampling them stands on
    // the drawn asphalt by construction.
    if (deckOut) deckOut->spines = chains;
    // S4: junction pairs closer than their combined radii merge into ONE
    // COMPOUND pad. trimSpine clamps its trims at 45% of the chain, so S1 kept
    // a squeezed sub-metre body with BOTH pads stacked over it — the measured
    // pad-pad overlap (52 cells on the real graph). Union-find the too-close
    // pairs; the link body between merged nodes is skipped (the compound pad
    // owns that asphalt), and the cluster's outer arms build one pad.
    std::vector<int> uf(N);
    for (int v = 0; v < N; ++v) uf[v] = v;
    std::function<int(int)> find = [&](int v) {
        while (uf[v] != v) v = uf[v] = uf[uf[v]];
        return v;
    };
    for (const UnionSpine& s : chains) {
        if (s.points.size() < 2) continue;
        const int a = nodeAt(s.points.front()), b = nodeAt(s.points.back());
        if (a < 0 || b < 0 || a == b || deg[a] < 3 || deg[b] < 3) continue;
        double len = 0;
        for (std::size_t i = 1; i < s.points.size(); ++i)
            len += (s.points[i] - s.points[i - 1]).length();
        if (len <= rad[a] + rad[b] + 0.25) uf[find(a)] = find(b);
    }
    // S5: the curb/sidewalk band is swept along the union outline of these
    // footprints (body ribbons + pad boundaries), with heights sampled from
    // this field — asphalt and band meet flush by construction.
    EdgeHeightField bandHeights;
    std::vector<Poly2> bandRibbons;
    std::vector<std::pair<Vec2, Vec2>> bandGaps;   // non-street mouths (S6)
    std::vector<int> streetArms(N, 0);             // per cluster root
    // Which chains are REALLY elevated (deck rides above the ground — a
    // viaduct or layered bridge)? weldChainProfiles assigns yAbs to EVERY
    // chain (the one-profile-source ride), so yAbs presence alone says
    // nothing — compare against the ground. At-grade chains double as the
    // pier KEEP-OUT: no column may stand in a road below (S6).
    std::vector<char> chainElevated(chains.size(), 0);
    struct KeepOutSeg { Vec2 a, b; double hw; };
    std::vector<KeepOutSeg> pierKeepOut;
    int nElevated = 0, nElevatedStreets = 0;
    for (std::size_t ci = 0; ci < chains.size(); ++ci) {
        const UnionSpine& cs = chains[ci];
        bool up = false;
        double lift = 0.0;
        Vec2 liftAt(0, 0);
        if (cs.yAbs.size() == cs.points.size())
            for (std::size_t i = 0; i < cs.points.size(); ++i) {
                const double d = cs.yAbs[i] - ground(cs.points[i].x, cs.points[i].y);
                if (d > lift) { lift = d; liftAt = cs.points[i]; }
                if (d > 1.5) up = true;
            }
        // A BRIDGE is structural, not a height difference: authored deck
        // heights (corridor decks/ramps) or a layer above grade. A plain
        // street whose grade-limited profile rides a fill over a dip is still
        // an at-grade street — the conform pass carves the ground up to it.
        // The old test (any point > 1.5 m above the RAW terrain) turned two
        // metro locals crossing valleys into "bridges": no sidewalk band, so
        // the arterial's band swept straight across their mouths at all four
        // ends (the city map's deepest four sidewalk-on-asphalt places).
        up = up && (cs.authoredDeck || cs.layer > 0);
        chainElevated[ci] = up;
        if (up) {
            ++nElevated;
            const bool street = cs.klass != RoadClass::Freeway && cs.klass != RoadClass::Ramp;
            if (street) ++nElevatedStreets;
            if (std::getenv("RT_LATTICE_DEBUG"))
                LOG_INFO << "[lattice] chain " << ci << " class " << static_cast<int>(cs.klass)
                         << " flagged ELEVATED: rides " << lift << " m above ground at ("
                         << liftAt.x << ", " << liftAt.y << "), " << cs.points.size()
                         << " pts from (" << cs.points.front().x << ", " << cs.points.front().y
                         << ") to (" << cs.points.back().x << ", " << cs.points.back().y << ")"
                         << (street ? " — a STREET: bridge treatment, no sidewalk band" : "");
        }
        if (!up)
            for (std::size_t i = 0; i + 1 < cs.points.size(); ++i)
                pierKeepOut.push_back({ cs.points[i], cs.points[i + 1], cs.halfWidth });
    }
    if (nElevatedStreets > 0)
        LOG_INFO << "[lattice] " << nElevated << " chains elevated, " << nElevatedStreets
                 << " of them STREETS (bridge treatment: no sidewalk band; the junction "
                    "pad's mouth becomes a band boundary — RT_LATTICE_DEBUG=1 lists them)";
    auto pierBlocked = [&](const Vec2& p) {
        for (const KeepOutSeg& k : pierKeepOut) {
            const Vec2 ab = k.b - k.a;
            const double l2 = ab.lengthSquared();
            double t2 = l2 > 1e-12 ? dot(p - k.a, ab) / l2 : 0.0;
            t2 = t2 < 0 ? 0 : (t2 > 1 ? 1 : t2);
            if ((k.a + ab * t2 - p).length() < k.hw + 2.5) return true;
        }
        return false;
    };
    // ACUTE-PAIR TRIM (2c nose trim, generalized in R2): two arms leaving a
    // junction NEARLY PARALLEL — a ramp along its freeway at a gore, or two
    // streets forking at a sharp angle (drive feedback A1: "one of the roads
    // remains a flat ribbon... It floats above it") — overlap ribbons far
    // beyond the standard junction radius. Each such chain end extends its
    // trim to where its centreline actually CLEARS the sibling's ribbon
    // (half + half + margin); the junction pad then fills the wedge between
    // the real mouths. Perpendicular and opposite arms are untouched (their
    // ribbons separate within the standard radius already).
    struct ArmRef { std::size_t chain; Vec2 dir; };
    std::vector<std::vector<ArmRef>> armsAt(N);
    for (std::size_t ci2 = 0; ci2 < chains.size(); ++ci2) {
        const UnionSpine& cs2 = chains[ci2];
        if (cs2.points.size() < 2) continue;
        const int fa = nodeAt(cs2.points.front()), fb = nodeAt(cs2.points.back());
        if (fa >= 0) {
            Vec2 d = cs2.points[1] - cs2.points[0];
            const double l = d.length();
            if (l > 1e-9) armsAt[fa].push_back({ ci2, d * (1.0 / l) });
        }
        if (fb >= 0) {
            Vec2 d = cs2.points[cs2.points.size() - 2] - cs2.points.back();
            const double l = d.length();
            if (l > 1e-9) armsAt[fb].push_back({ ci2, d * (1.0 / l) });
        }
    }
    // Near-node segments per chain (for the clearance walk).
    auto chainSegsNear = [&](std::size_t ci2, int node, double reach) {
        std::vector<std::pair<Vec2, Vec2>> segs;
        const UnionSpine& cs2 = chains[ci2];
        const bool fromA = nodeAt(cs2.points.front()) == node;
        double acc = 0;
        for (std::size_t i = 1; i < cs2.points.size() && acc < reach; ++i) {
            const std::size_t k = fromA ? i : cs2.points.size() - 1 - i;
            const std::size_t kp = fromA ? k - 1 : k + 1;
            segs.push_back({ cs2.points[kp], cs2.points[k] });
            acc += (cs2.points[k] - cs2.points[kp]).length();
        }
        return segs;
    };
    struct Wedge { int node; std::size_t subCi, domCi; double subTrim; double side = 0; };
    std::vector<Wedge> wedges;
    auto pairTrim = [&](const UnionSpine& mine, std::size_t myCi, int node,
                        bool fromFront) {
        double trim = 0;
        std::size_t domBest = SIZE_MAX;
        Vec2 myDir(0, 0);
        for (const ArmRef& ar : armsAt[node])
            if (ar.chain == myCi) { myDir = ar.dir; break; }
        if (myDir.x == 0 && myDir.y == 0) return trim;
        for (const ArmRef& other : armsAt[node]) {
            if (other.chain == myCi) continue;
            if (dot(myDir, other.dir) < 0.82) continue;   // ~35 deg: not acute
            // DOMINANCE: only the narrower-or-equal arm gives way — a ramp
            // trims to clear its freeway, never the freeway to clear the
            // ramp (the symmetric rule cut the mainline's body and parapets
            // ~60 m back from every gore). Equal street forks both extend.
            if (mine.halfWidth > chains[other.chain].halfWidth + 0.1) continue;
            const double clearL = mine.halfWidth +
                                  chains[other.chain].halfWidth + 0.4;
            const auto segs = chainSegsNear(other.chain, node, 220.0);
            double acc = 0;
            const std::size_t n2 = mine.points.size();
            for (std::size_t i = 0; i < n2; ++i) {
                const std::size_t k = fromFront ? i : n2 - 1 - i;
                if (i > 0) {
                    const std::size_t kp = fromFront ? k - 1 : k + 1;
                    acc += (mine.points[k] - mine.points[kp]).length();
                }
                double dmin = 1e30;
                for (const auto& seg : segs) {
                    const Vec2 ab = seg.second - seg.first;
                    const double l2 = ab.lengthSquared();
                    double t2 = l2 > 1e-12
                        ? dot(mine.points[k] - seg.first, ab) / l2 : 0.0;
                    t2 = t2 < 0 ? 0 : (t2 > 1 ? 1 : t2);
                    dmin = std::min(dmin,
                                    (seg.first + ab * t2 - mine.points[k]).length());
                }
                if (dmin > clearL) {
                    if (acc > trim) { trim = acc; domBest = other.chain; }
                    break;
                }
            }
        }
        if (domBest != SIZE_MAX && trim > 0)
            wedges.push_back({ node, myCi, domBest, trim });
        return trim;
    };
    // TRIM PRE-PASS (R2): all trims — and therefore ALL wedge records — are
    // computed before any chain sweeps, because a DOMINANT chain's parapet
    // needs gap windows from wedges its SUBORDINATE (which may come later in
    // the loop) records.
    std::vector<double> trimA(chains.size(), 0.0), trimB(chains.size(), 0.0);
    for (std::size_t ci = 0; ci < chains.size(); ++ci) {
        const UnionSpine& s = chains[ci];
        if (s.points.size() < 2) continue;
        const int a = nodeAt(s.points.front()), b = nodeAt(s.points.back());
        trimA[ci] = (a >= 0 && deg[a] >= 3) ? rad[a] : 0.0;
        trimB[ci] = (b >= 0 && deg[b] >= 3) ? rad[b] : 0.0;
        if (a >= 0 && deg[a] >= 3)
            trimA[ci] = std::max(trimA[ci], pairTrim(s, ci, a, true));
        if (b >= 0 && deg[b] >= 3)
            trimB[ci] = std::max(trimB[ci], pairTrim(s, ci, b, false));
    }
    // WEDGE RAILS (R2): for each acute pair, the pad's corner between the
    // dominant and subordinate arms follows the DOMINANT chain's edge curve
    // (sampled from its mouth out to the subordinate's mouth arc, offset by
    // its half-width on the subordinate's side) instead of a straight chord
    // the curved arm bows off of. Keyed to the pad root; matched by arm ids.
    std::map<int, std::vector<JunctionRail>> nodeRails;
    for (Wedge& w : wedges) {
        const UnionSpine& dom = chains[w.domCi];
        const UnionSpine& sub = chains[w.subCi];
        if (dom.points.size() < 2 || sub.points.size() < 2) continue;
        if (dom.yAbs.size() != dom.points.size()) continue;
        const bool domFromA = nodeAt(dom.points.front()) == w.node;
        if (!domFromA && nodeAt(dom.points.back()) != w.node) continue;
        auto dirAt = [&](const UnionSpine& c, bool fromA) {
            const Vec2 d = fromA ? c.points[1] - c.points[0]
                                 : c.points[c.points.size() - 2] - c.points.back();
            const double l = d.length();
            return l > 1e-9 ? d * (1.0 / l) : Vec2(1, 0);
        };
        const bool subFromA = nodeAt(sub.points.front()) == w.node;
        const Vec2 dDir = dirAt(dom, domFromA);
        const Vec2 sDir = dirAt(sub, subFromA);
        const double side =
            (dDir.x * sDir.y - dDir.y * sDir.x) >= 0 ? 1.0 : -1.0;
        w.side = side;
        JunctionRail rail;
        rail.fromId = static_cast<int>(w.domCi);
        rail.toId = static_cast<int>(w.subCi);
        const double s0 = (w.node < N ? rad[w.node] : 8.0) + 1.5;
        const double s1 = w.subTrim - 1.5;
        double acc = 0;
        for (std::size_t i = 1; i < dom.points.size() && acc < s1; ++i) {
            const std::size_t k = domFromA ? i : dom.points.size() - 1 - i;
            const std::size_t kp = domFromA ? k - 1 : k + 1;
            const Vec2 seg = dom.points[k] - dom.points[kp];
            const double segL = seg.length();
            if (segL < 1e-9) continue;
            const double a0 = acc;
            acc += segL;
            for (double st = std::max(s0, std::ceil(a0 / 4.0) * 4.0); st < acc && st < s1;
                 st += 4.0) {
                const double t = (st - a0) / segL;
                const Vec2 c2 = dom.points[kp] + seg * t;
                const Vec2 dirN = seg * (1.0 / segL);
                const Vec2 leftN(-dirN.y, dirN.x);
                const double y =
                    dom.yAbs[kp] + (dom.yAbs[k] - dom.yAbs[kp]) * t;
                // seg is oriented AWAY from the node in BOTH walk
                // orientations, so leftN is already node-relative — the side
                // sign (from the away-from-node dir cross) applies directly.
                const Vec2 p = c2 + leftN * (side * dom.halfWidth);
                rail.pts.push_back(Vec3(p.x, y, p.y));
            }
        }
        if (rail.pts.size() >= 2)
            nodeRails[find(w.node)].push_back(std::move(rail));
    }
    for (std::size_t ci = 0; ci < chains.size(); ++ci) {
        const UnionSpine& s = chains[ci];
        if (s.points.size() < 2) continue;
        const int a = nodeAt(s.points.front()), b = nodeAt(s.points.back());
        if (a >= 0 && b >= 0 && a != b && find(a) == find(b))
            continue;                            // compound link: the pad owns it
        const double rA = trimA[ci], rB = trimB[ci];
        UnionSpine t = trimSpine(s, rA, rB);
        if (t.points.size() < 2) continue;

        // LANES come from the road's CLASS — the one lanesForClass() source the
        // nav lanes already use, so the painted dividers and the lanes cars
        // actually drive in agree by construction. Dividing the width by a
        // nominal 3.6 m (what this did) is a guess that disagreed with both:
        // it made every 12 m street "2 lanes per side" and every road, however
        // wide, was painted as a plain two-laner because the strip that carries
        // a divider was never emitted at all.
        const int laneTotal = std::max(1, lanesForClass(s.klass, /*perDirection=*/false));
        const int lanesPerSide = std::max(1, laneTotal / 2);
        const bool elevated = chainElevated[ci] != 0;
        const bool isFwy = s.klass == RoadClass::Freeway;
        const bool isRamp = s.klass == RoadClass::Ramp;
        std::vector<Vec3> ring0, ringN;
        if (isFwy || isRamp) {
            // S6 ONE MESHER: a Freeway/Ramp chain gets the full structure the
            // corridor kit builds — deck, viaduct underside, edge parapets,
            // (freeway) median, piers under elevated spans. Parapets stop at
            // the junction trim, so a gore's merge stays open by construction.
            const int lanes = isRamp ? 1
                : std::max(1, static_cast<int>(std::lround((s.halfWidth - 1.0) / 3.6)));
            MeshBuilder::append(out, sweepRoadLattice(t, freewayDeckProfile(lanes),
                                                      ground, 2.0, nullptr, &ring0, &ringN));
            // STRUCTURE exists only where the deck FLIES (the old corridor's
            // 35 cm rule, as gap data): at-grade stretches get no underside
            // (buried soffit + an open fascia ring at the street were the
            // zoo's landing rims) and no PARAPET either — an at-grade ramp
            // edge wants a guardrail (R6 side grammar), not 0.85 m of
            // concrete running into the junction.
            std::vector<GapWindow> ground0;
            {
                double acc2 = 0, runStart = -1;
                const bool hasY2 = t.yAbs.size() == t.points.size();
                for (std::size_t i = 0; i < t.points.size(); ++i) {
                    if (i > 0) acc2 += (t.points[i] - t.points[i - 1]).length();
                    const double clr2 = hasY2
                        ? t.yAbs[i] - ground(t.points[i].x, t.points[i].y)
                        : 0.0;
                    if (clr2 < 0.35) {
                        if (runStart < 0) runStart = std::max(0.0, acc2 - 1.0);
                    } else if (runStart >= 0) {
                        ground0.push_back({ runStart, acc2 });
                        runStart = -1;
                    }
                }
                if (runStart >= 0) ground0.push_back({ runStart, acc2 + 1.0 });
                MeshBuilder::append(out, sweepRoadLattice(
                    t, freewayUndersideProfile(0.5), ground, 2.0,
                    ground0.empty() ? nullptr : &ground0));
            }
            // Parapet WEDGE GAPS (R2): where this chain is the DOMINANT of
            // an acute pair, its wedge-side parapet opens over the whole
            // merge window (gore trim -> subordinate mouth), not just the
            // junction trim — or the wall stands across the merge (the
            // probe's blocked=1 at three gores).
            double Lt = 0;
            for (std::size_t i = 1; i < t.points.size(); ++i)
                Lt += (t.points[i] - t.points[i - 1]).length();
            std::vector<GapWindow> gapsL, gapsR;
            for (const Wedge& w : wedges) {
                if (w.domCi != ci || w.side == 0) continue;
                const bool atA = a >= 0 && nodeAt(s.points.front()) == w.node;
                const bool atB = b >= 0 && nodeAt(s.points.back()) == w.node;
                if (!atA && !atB) continue;
                const double span = w.subTrim - (atA ? rA : rB);
                if (span <= 1.0) continue;
                const GapWindow g = atA ? GapWindow{ 0.0, span }
                                        : GapWindow{ Lt - span, Lt };
                // side is in away-from-node coords; sweep runs a->b, so the
                // b-end flips.
                const double sweepSide = atA ? w.side : -w.side;
                (sweepSide > 0 ? gapsL : gapsR).push_back(g);
            }
            for (const GapWindow& g : ground0) {   // no parapets at grade
                gapsL.push_back(g);
                gapsR.push_back(g);
            }
            MeshBuilder::append(out, sweepRoadLattice(t, parapetProfile(+1), ground, 2.0,
                                                      gapsL.empty() ? nullptr : &gapsL));
            MeshBuilder::append(out, sweepRoadLattice(t, parapetProfile(-1), ground, 2.0,
                                                      gapsR.empty() ? nullptr : &gapsR));
            // R6d side grammar: box girders hang under both deck edges (the
            // flat soffit alone read as paper), gapped where the deck sits
            // at grade; and a railing POST tops each parapet run every
            // ~3.2 m — skipping exactly the windows the walls skip.
            MeshBuilder::append(out, sweepRoadLattice(t, girderProfile(+1), ground, 2.0,
                                                      ground0.empty() ? nullptr : &ground0));
            MeshBuilder::append(out, sweepRoadLattice(t, girderProfile(-1), ground, 2.0,
                                                      ground0.empty() ? nullptr : &ground0));
            {
                const Vec3 postCol(0.55, 0.56, 0.58);
                auto inGaps = [](const std::vector<GapWindow>& gs, double sArc) {
                    for (const GapWindow& g : gs)
                        if (sArc >= g.s0 - 0.4 && sArc <= g.s1 + 0.4) return true;
                    return false;
                };
                auto post = [&](const Vec3& base, const Vec2& d) {
                    const Vec2 l(-d.y, d.x);
                    const double hx = 0.06, hz = 0.06, h = 0.5;
                    const Vec3 dx(l.x * hx, 0, l.y * hx);
                    const Vec3 dz(d.x * hz, 0, d.y * hz);
                    const Vec3 top = base + Vec3(0, h, 0);
                    auto face = [&](const Vec3& A, const Vec3& B, const Vec3& C,
                                    const Vec3& D, const Vec3& n) {
                        MeshBuilder::emitQuad(out, A, B, C, D, n, postCol);
                    };
                    face(base - dx - dz, base + dx - dz, top + dx - dz,
                         top - dx - dz, Vec3(-d.x, 0, -d.y));
                    face(base + dx + dz, base - dx + dz, top - dx + dz,
                         top + dx + dz, Vec3(d.x, 0, d.y));
                    face(base - dx + dz, base - dx - dz, top - dx - dz,
                         top - dx + dz, Vec3(-l.x, 0, -l.y));
                    face(base + dx - dz, base + dx + dz, top + dx + dz,
                         top + dx - dz, Vec3(l.x, 0, l.y));
                    face(top - dx - dz, top + dx - dz, top + dx + dz,
                         top - dx + dz, Vec3(0, 1, 0));
                };
                const bool hasY = t.yAbs.size() == t.points.size();
                const bool hasW = t.hw.size() == t.points.size();
                double acc = 0, nextPost = 1.6;
                for (std::size_t i = 1; i < t.points.size() && hasY; ++i) {
                    const Vec2 A = t.points[i - 1], B = t.points[i];
                    const double seg = (B - A).length();
                    while (seg > 1e-6 && nextPost <= acc + seg) {
                        const double f = (nextPost - acc) / seg;
                        const Vec2 P = A + (B - A) * f;
                        const Vec2 d = normalize(B - A);
                        const Vec2 l(-d.y, d.x);
                        const double y =
                            t.yAbs[i - 1] + (t.yAbs[i] - t.yAbs[i - 1]) * f;
                        const double hwHere =
                            hasW ? t.hw[i - 1] + (t.hw[i] - t.hw[i - 1]) * f
                                 : t.halfWidth;
                        for (int side = -1; side <= 1; side += 2) {
                            if (inGaps(side > 0 ? gapsL : gapsR, nextPost))
                                continue;
                            const Vec2 c2 = P + l * (side * (hwHere - 0.14));
                            post(Vec3(c2.x, y + 0.85, c2.y), d);
                        }
                        nextPost += 3.2;
                    }
                    acc += seg;
                }
            }
            if (isFwy)
                MeshBuilder::append(out, sweepRoadLattice(t, medianProfile(), ground, 2.0));
            MeshBuilder::append(out, latticeChainPiers(t, groundFn, 0.5, nullptr, pierBlocked));
            // No sidewalk band along a freeway or ramp — and where this chain
            // enters a STREET junction, the band must GAP across its mouth
            // (else a curb + slab curbs the ramp entrance shut).
            if (ring0.size() >= 2)
                bandGaps.push_back({ Vec2(ring0.front().x, ring0.front().z),
                                     Vec2(ring0.back().x, ring0.back().z) });
            if (ringN.size() >= 2)
                bandGaps.push_back({ Vec2(ringN.front().x, ringN.front().z),
                                     Vec2(ringN.back().x, ringN.back().z) });
        } else {
            // S5 OWNERSHIP: street bodies sweep the CARRIAGEWAY only. The curb
            // + sidewalk band owns everything outside the asphalt — a body can
            // no longer sweep a raised sidewalk into a pad or a neighbour.
            const std::size_t v0 = out.vertices.size();
            MeshBuilder::append(out,
                                sweepRoadLattice(t,
                                                 carriagewayProfile(lanesPerSide, laneTotal,
                                                                    /*oneWay=*/false,
                                                                    s.travelEdgeFrac),
                                                 ground, 2.0, nullptr, &ring0, &ringN));
            // LANDING/gore MOUTH GAP (#20, review S4b): where a STREET body
            // meets a ramp landing, gap the sidewalk band across its mouth —
            // exactly as the freeway/ramp branch does. Otherwise the street's
            // closed-capsule ribbon caps with a CURB WALL straight across the
            // carriageway at the landing, sealing the ramp entrance ("holes
            // where it thinks it's an intersection"). The zebra is already
            // suppressed there (S3); this opens the curb too.
            auto landingEnd = [&](int nd) {
                return nd >= 0 &&
                       (g.nodes[nd].kind == JunctionKind::Landing ||
                        isGore(g.nodes[nd].kind));
            };
            if (landingEnd(a) && ring0.size() >= 2)
                bandGaps.push_back({ Vec2(ring0.front().x, ring0.front().z),
                                     Vec2(ring0.back().x, ring0.back().z) });
            if (landingEnd(b) && ringN.size() >= 2)
                bandGaps.push_back({ Vec2(ringN.front().x, ringN.front().z),
                                     Vec2(ringN.back().x, ringN.back().z) });
            // Crosswalk band UV (ADR-0062, ported from the weld): the shader
            // stripes a zebra where mv lands in the set-back window past a
            // junction mouth. The sweep bakes v = chain arc length; remap it
            // to metres-past-the-NEAREST-JUNCTION-mouth (dead ends and chain
            // interiors stay out of the window), or shift everything clear of
            // the window when crosswalks are off — the paint stays gated.
            {
                double Lc = 0;
                for (std::size_t i = 1; i < t.points.size(); ++i)
                    Lc += (t.points[i] - t.points[i - 1]).length();
                // Semantic zebra gate (#21): a crossing window opens at a
                // chain end ONLY where that end is a street INTERSECTION and
                // the edge is crossable — never at a ramp landing or a gore
                // (both are deg >= 3, so the old trim-radius test lit a zebra
                // there: "the freeways have crosswalks?"). Per-end, so a
                // street that is an Intersection at A and a Landing at B gets
                // its zebra only at A.
                const bool jA = rA > 0.0 &&
                                (s.access & road_access::kCrossable) && a >= 0 &&
                                g.nodes[a].kind == JunctionKind::Intersection;
                const bool jB = rB > 0.0 &&
                                (s.accessBack & road_access::kCrossable) && b >= 0 &&
                                g.nodes[b].kind == JunctionKind::Intersection;
                for (std::size_t vi = v0; vi < out.vertices.size(); ++vi) {
                    const double sAt = out.vertices[vi].v;
                    double d = 1e4;
                    if (jA) d = std::min(d, sAt);
                    if (jB) d = std::min(d, Lc - sAt);
                    out.vertices[vi].v = static_cast<float>(crosswalks ? d : d + 64.0);
                }
            }
            if (elevated) {
                // A layered street bridge: give the deck an underside and legs.
                // (Its sidewalk is omitted — bridge sides are barrier/grammar
                // territory, not a kerbed slab hanging over the void.)
                MeshBuilder::append(out, sweepRoadLattice(t, freewayUndersideProfile(0.4), ground, 2.0));
                MeshBuilder::append(out, latticeChainPiers(t, groundFn, 0.4, nullptr, pierBlocked));
            } else {   // footprint + edge heights for the band union
                std::vector<double> ys(t.points.size(), 0.0);
                if (t.yAbs.size() == t.points.size()) ys = t.yAbs;
                else for (std::size_t i = 0; i < t.points.size(); ++i)
                    ys[i] = ground(t.points[i].x, t.points[i].y);
                // APPROACH BAND CUT (#20): a street that climbs to a ramp
                // LANDING or gore must not carry its sidewalk up the grade
                // ("the sidewalk floats as it elevates, not attached to the
                // road"). Trim the RISING portion off each Landing/gore end,
                // keeping only the flat interior (within 0.5 m of the chain's
                // low point). A plain flat street trims nothing.
                std::vector<Vec2> bpts = t.points;
                std::vector<double> bys = ys;
                {
                    int lo = 0, hi = static_cast<int>(bpts.size()) - 1;
                    const double base =
                        *std::min_element(bys.begin(), bys.end());
                    auto isLand = [&](int nd) {
                        return nd >= 0 &&
                               (g.nodes[nd].kind == JunctionKind::Landing ||
                                isGore(g.nodes[nd].kind));
                    };
                    if (isLand(a))
                        while (lo < hi && bys[lo] > base + 0.5) ++lo;
                    if (isLand(b))
                        while (hi > lo && bys[hi] > base + 0.5) --hi;
                    if (lo > 0 || hi < static_cast<int>(bpts.size()) - 1) {
                        std::vector<Vec2> kp(bpts.begin() + lo,
                                             bpts.begin() + hi + 1);
                        std::vector<double> ky(bys.begin() + lo,
                                               bys.begin() + hi + 1);
                        bpts.swap(kp);
                        bys.swap(ky);
                    }
                }
                if (bpts.size() >= 2) {
                    bandHeights.addPolyline(bpts, bys);
                    Poly2 rib = ribbonOutline(bpts, t.halfWidth + 0.02, 2.0);
                    if (rib.size() >= 3) {
                        if (signedArea(rib) < 0)
                            std::reverse(rib.begin(), rib.end());
                        bandRibbons.push_back(std::move(rib));
                    }
                }
            }
        }
        // The mouth is the FULL profile ring (sidewalk|curb|lanes|curb|sidewalk),
        // reversed to run left -> right looking OUTWARD. Slicing only the
        // carriageway here was the boundary bug: each arm swept its raised
        // sidewalk into a pad that covered carriageway only, so sidewalks
        // double-covered at every corner. The pad now spans verge to verge and
        // its corner points sit at the sidewalk outer edge, where the kerb
        // returns will fillet (stage 2).
        // The body ring IS the carriageway (S5 ownership), so the mouth is the
        // whole ring. The contract is left->right looking OUTWARD along the
        // arm — and "outward" FLIPS between the chain's two ends: at ring0
        // outward is +sweep (the ring needs reversing), at ringN outward is
        // -sweep (it does not). S1 reversed BOTH, so every chain-END arm
        // crossed its mouth backwards and the boundary loop bowtied there
        // (found by the compound-pad area check: 149 vs 213 filled).
        auto mouth = [&](const std::vector<Vec3>& ring, bool flip) {
            return flip ? std::vector<Vec3>(ring.rbegin(), ring.rend()) : ring;
        };
        if (chainTriEndsOut) chainTriEndsOut->push_back(out.indices.size());
        const std::size_t tn = t.points.size();
        // Arms gather on the cluster ROOT: a compound pad (merged too-close
        // junctions) collects the outer arms of ALL its member nodes.
        if (a >= 0 && deg[a] >= 3 && ring0.size() >= 2) {
            arms[find(a)].push_back({ normalize(t.points[1] - t.points[0]), mouth(ring0, true), static_cast<int>(ci) });
            if (!isFwy && !isRamp && !elevated) ++streetArms[find(a)];
        }
        if (b >= 0 && deg[b] >= 3 && ringN.size() >= 2) {
            arms[find(b)].push_back({ normalize(t.points[tn - 2] - t.points[tn - 1]), mouth(ringN, false), static_cast<int>(ci) });
            if (!isFwy && !isRamp && !elevated) ++streetArms[find(b)];
        }
    }

    int nCoons = 0, nT = 0, nFan = 0, nStub = 0, nDeg2 = 0, nMismatch = 0,
        nCompound = 0;
    std::vector<int> clusterSize(N, 0);
    for (int v = 0; v < N; ++v)
        if (deg[v] >= 3) ++clusterSize[find(v)];
    // Cluster KIND (#21): the most-specific member kind over each union-find
    // cluster of too-close junctions. Landings/gores styling-differ from
    // street Intersections — the pad still fills the disc, but only an
    // Intersection cluster joins the sidewalk band and hosts zebras.
    auto kindPrio = [](JunctionKind k) {
        switch (k) {
            case JunctionKind::Landing: return 6;
            case JunctionKind::Diverge: return 5;
            case JunctionKind::Merge: return 4;
            case JunctionKind::Intersection: return 3;
            case JunctionKind::DeadEnd: return 2;
            case JunctionKind::None: return 1;
            default: return 0;
        }
    };
    std::vector<JunctionKind> clusterKind(N, JunctionKind::None);
    for (int v = 0; v < N; ++v) {
        if (deg[v] < 3) continue;
        const int r = find(v);
        if (kindPrio(g.nodes[v].kind) > kindPrio(clusterKind[r]))
            clusterKind[r] = g.nodes[v].kind;
    }
    std::vector<int> degHist(12, 0);
    for (int v = 0; v < N; ++v) {
        if (deg[v] < 12) ++degHist[deg[v]];
        if (deg[v] < 3) { if (deg[v] == 2) ++nDeg2; continue; }
        if (find(v) != v) continue;                  // compound member: root emits
        const int na = static_cast<int>(arms[v].size());
        if (na < 3) { ++nStub; continue; }          // arms not gathered (trim/lookup failed)
        if (clusterSize[v] > 1) ++nCompound;         // merged too-close junctions
        else if (na != deg[v]) ++nMismatch;          // gathered != incident: a real bug
        if (na == 3) ++nT;
        else if (na == 4) ++nCoons;
        else ++nFan;
        std::vector<Vec3> padFootprint;
        // The authored kerb return is only APPLIED where it is geometrically
        // possible. The curb/sidewalk band is an INWARD offset of this corner
        // (the sidewalk sits on the concave side of a kerb return), and a curve
        // cannot be offset inward by more than its own radius: with a 3.5 m walk
        // around a 3.0 m return the band's outer edge is degenerate before it is
        // drawn. Measured on metro_v2 (docs/curb-weld-analysis.md): rounding at
        // r=3.0 with a 3.5 m walk took folded band quads 462 -> 832 and stacked
        // band 884 -> 1084 m2, while the SAME rounding with a 2.0 m walk costs
        // almost nothing (464 folds, 254 m2). So round when the radius can carry
        // the walk, and keep the old corner when it cannot.
        const double padCorner =
            (std::getenv("RT_KERB_RETURN") && cornerRadius > sidewalkWidth * 1.05)
                ? cornerRadius : 0.0;
        RenderMesh pad = junctionPatch(arms[v], 2.0f, Vec3(0.10, 0.10, 0.11),
                                       &padFootprint,
                                       nodeRails.count(v) ? &nodeRails[v]
                                                          : nullptr,
                                       padCorner);
        // 2c: an ELEVATED pad (a gore on the deck) closes its bottom — a
        // mirrored down-facing belly at deck thickness plus a rim skirt, so
        // the viaduct has structure under the junction fill too, not a
        // floating carpet between the chains' undersides.
        if (padFootprint.size() >= 3) {
            double avgY = 0, gnd = 0;
            for (const Vec3& p : padFootprint) {
                avgY += p.y;
                gnd += ground(p.x, p.z);
            }
            avgY /= padFootprint.size();
            gnd /= padFootprint.size();
            if (avgY - gnd > 2.5) {
                const double thk = 0.5;
                RenderMesh belly = pad;
                for (auto& vtx : belly.vertices) vtx.position.y -= thk;
                // Flip winding so the belly faces DOWN.
                for (std::size_t t = 0; t + 2 < belly.indices.size(); t += 3)
                    std::swap(belly.indices[t + 1], belly.indices[t + 2]);
                MeshBuilder::append(out, belly);
                // Rim skirt: quads around the footprint loop.
                RenderMesh skirt;
                const std::size_t nfp = padFootprint.size();
                for (std::size_t i = 0; i < nfp; ++i) {
                    const Vec3& A = padFootprint[i];
                    const Vec3& B = padFootprint[(i + 1) % nfp];
                    Vec3 d = B - A;
                    const double dl = std::sqrt(d.x * d.x + d.z * d.z);
                    if (dl < 1e-6) continue;
                    const Vec3 nrm(d.z / dl, 0.0, -d.x / dl);   // outward (CCW loop)
                    MeshBuilder::emitQuad(
                        skirt, A, B, Vec3(B.x, B.y - thk, B.z),
                        Vec3(A.x, A.y - thk, A.z), nrm, Vec3(0.32, 0.32, 0.34));
                }
                MeshBuilder::append(out, skirt);
            }
        }
        // The pad's drivable faces join the deck field (RoadDeckField::pads);
        // an elevated pad's bottom closure faces down and is skipped.
        if (deckOut) {
            for (std::size_t ti = 0; ti + 2 < pad.indices.size(); ti += 3) {
                const Vec3& a = pad.vertices[pad.indices[ti]].position;
                const Vec3& b = pad.vertices[pad.indices[ti + 1]].position;
                const Vec3& c = pad.vertices[pad.indices[ti + 2]].position;
                if (pad.vertices[pad.indices[ti]].normal.y <= 0.0) continue;
                deckOut->pads.push_back({a, b, c});
            }
        }
        MeshBuilder::append(out, pad);
        // Only a street INTERSECTION's pad joins the sidewalk band union
        // (#21). A Landing/gore pad has a ramp arm; joining it wrapped the
        // sidewalk band around and ONTO the ramp mouth (the curb "sealed the
        // ramp entrance shut", and a zebra painted across it). The pad still
        // meshes as the drivable disc — it just stops recruiting sidewalk.
        if (padFootprint.size() >= 3 && streetArms[v] > 0 &&
            clusterKind[v] == JunctionKind::Intersection) {
            bandHeights.addLoop(padFootprint);
            Poly2 fp;
            for (const Vec3& p : padFootprint) fp.push_back(Vec2(p.x, p.z));
            if (signedArea(fp) < 0) std::reverse(fp.begin(), fp.end());
            bandRibbons.push_back(std::move(fp));
        }
    }
    // S5: sweep the curb + sidewalk band along every union boundary loop
    // (exterior outlines AND block-interior holes — the band rides each
    // loop's right normal, outward from the asphalt in both cases).
    if (!bandRibbons.empty()) {
        std::vector<Poly2> loops;
        int nSliverLoops = 0;
        for (Poly2& L : polygonUnion(bandRibbons)) {
            // snap-round + de-spur so band quads never fold on a hairline
            // vertex the union left behind.
            //
            // MERGE TOLERANCE > SNAP QUANTUM (docs/curb-weld-analysis.md). The
            // old pair — snap to 1 cm, then merge closer than 0.5 cm — cannot
            // converge: two union vertices a hair apart land on ADJACENT grid
            // points, exactly 1 cm apart, which is above the merge threshold, so
            // the snap MANUFACTURES the hairline pairs the merge exists to
            // remove. The kerb then reverses across a 1 cm edge and the band's
            // mitre blows that reversal out to metres. 2 cm clears the 1.41 cm
            // grid diagonal, so no adjacent-grid-point pair can survive; real
            // kerb detail is metres, never centimetres.
            const double kSnap = 0.01, kMerge = 0.02;
            Poly2 c;
            for (const Vec2& p : L) {
                const Vec2 q(std::round(p.x / kSnap) * kSnap, std::round(p.y / kSnap) * kSnap);
                if (c.empty() || (q - c.back()).length() > kMerge) c.push_back(q);
            }
            while (c.size() >= 2 && (c.front() - c.back()).length() <= kMerge) c.pop_back();
            bool changed = true;
            while (changed && c.size() > 3) {
                changed = false;
                const int nn2 = static_cast<int>(c.size());
                for (int i = 0; i < nn2; ++i) {
                    const Vec2 e0 = c[i] - c[(i + nn2 - 1) % nn2];
                    const Vec2 e1 = c[(i + 1) % nn2] - c[i];
                    const double l0 = e0.length(), l1 = e1.length();
                    if (l0 < 1e-9 || l1 < 1e-9) { c.erase(c.begin() + i); changed = true; break; }
                    const double cosT = dot(e0, e1) / (l0 * l1);
                    // A SPUR is a shape, not an angle: the kerb goes out and comes
                    // straight back, enclosing nothing. The old test fired only
                    // past ~170 deg (cosT < -0.985) — but the reversals that
                    // wreck the band measure 90-166 deg and sailed through. Judge
                    // the sliver instead: a reversal that encloses under 0.05 m2,
                    // or reverses across a sub-decimetre edge, is a union
                    // artifact. A LEGITIMATE sharp corner (an acute Y arriving at
                    // 30 deg) reverses just as hard but spans metres, so its
                    // triangle is ~1 m2 and it is kept.
                    const double sliver = 0.5 * std::fabs(cross(e0, e1));
                    // A sub-decimetre edge is noise at ANY angle: real kerb
                    // detail — an arc segment, a mouth, a corner — is metres.
                    // (Gating this on the angle too left a band of 90-120 deg
                    // reversals alive, 155 of them on metro_v2.)
                    // Judged against the BAND's scale, not centimetres (the
                    // sidewalk census, metro_v2): 24 of the 29 remaining
                    // sidewalk-on-asphalt places were reversals of 120-170 deg
                    // on 0.2-0.6 m edges enclosing 0.1-0.5 m2 — out-and-back
                    // hooks the union leaves where two roads' verges cross at a
                    // junction corner — and each one took the band a metre into
                    // the carriageway. A reversal sharper than 120 deg on an
                    // edge under 1 m, or enclosing under 0.5 m2, is noise: a
                    // real kerb never reverses inside a metre.
                    if (cosT < -0.985 || std::min(l0, l1) < 0.10 ||
                        (cosT < -0.5 && (sliver < 0.5 || std::min(l0, l1) < 1.0))) {
                        c.erase(c.begin() + i);
                        changed = true;
                        break;
                    }
                }
            }
            // A loop smaller than the band it would carry is not a kerb. The
            // union leaves a 10 cm triangle where two verges cross at a sharp
            // junction corner; the de-spur above never touches a triangle, and
            // the sweeper built a 2 m sidewalk ring around it — the concrete
            // wedge in the middle of metro's collector/local junction (census #1,
            // 4.4 m onto the deck).
            if (c.size() >= 3) {
                double perim = 0.0;
                for (std::size_t i = 0; i < c.size(); ++i)
                    perim += (c[(i + 1) % c.size()] - c[i]).length();
                if (std::fabs(signedArea(c)) < 1.0 || perim < 2.0) {
                    ++nSliverLoops;
                    continue;
                }
                loops.push_back(std::move(c));
            }
        }
        if (nSliverLoops > 0)
            LOG_INFO << "[lattice] dropped " << nSliverLoops
                     << " sliver curb loop(s) (area < 1 m2 or perimeter < 2 m)";
        if (auditOut) {
            auditOut->loops = loops;
            auditOut->mouthGaps = bandGaps;
            auditOut->sidewalkWidth = sidewalkWidth;
            auditOut->curbHeight = curbHeight;
            for (int v = 0; v < N; ++v)
                if (deg[v] >= 3) {
                    auditOut->junctions.push_back(g.nodes[v].pos);
                    auditOut->junctionDegree.push_back(deg[v]);
                    // Smallest gap between adjacent arms, from the SAME arm
                    // directions the pad was built from.
                    double minGap = -1.0;
                    if (arms[v].size() >= 2) {
                        std::vector<double> ang;
                        ang.reserve(arms[v].size());
                        for (const JunctionArm& A : arms[v])
                            ang.push_back(std::atan2(A.dir.y, A.dir.x));
                        std::sort(ang.begin(), ang.end());
                        minGap = 360.0;
                        for (std::size_t k = 0; k < ang.size(); ++k) {
                            double d = ang[(k + 1) % ang.size()] - ang[k];
                            if (k + 1 == ang.size()) d += 2.0 * PI;
                            minGap = std::min(minGap, d * 180.0 / PI);
                        }
                    }
                    auditOut->junctionMinAngle.push_back(minGap);
                }
        }
        MeshBuilder::append(out, sweepCurbSidewalkBand(
            loops, [&](double x, double z) { return bandHeights.sample(x, z); },
            sidewalkWidth, curbHeight, bandGaps.empty() ? nullptr : &bandGaps,
            heightAt ? &heightAt : nullptr));   // outer skirt drapes to terrain
    }
    if (std::getenv("RT_LATTICE_DEBUG")) {
        int degen = 0;
        for (std::size_t t = 0; t + 2 < out.indices.size(); t += 3) {
            const Vec3& a = out.vertices[out.indices[t]].position;
            const Vec3& b = out.vertices[out.indices[t + 1]].position;
            const Vec3& c = out.vertices[out.indices[t + 2]].position;
            if (cross(b - a, c - a).length() < 1e-9) ++degen;
        }
        LOG_INFO << "[lattice] nodes=" << N << " deg2chains=" << nDeg2
                 << " pad3=" << nT << " pad4=" << nCoons << " pad5+=" << nFan
                 << " compound=" << nCompound << " stub(arms<3)=" << nStub
                 << " armMismatch=" << nMismatch
                 << " | tris=" << (out.indices.size() / 3) << " degenerate=" << degen;
        std::string h;
        for (int d = 0; d < 12; ++d) if (degHist[d]) h += " d" + std::to_string(d) + "=" + std::to_string(degHist[d]);
        LOG_INFO << "[lattice] degree histogram:" << h;
    }
    return out;
}

RenderMesh buildRoadNetMesh(const RoadEntity& road, const RoadGroundFn& heightAt,
                            CurbBandAudit* auditOut, RoadDeckField* deckOut) {
    // Roads-v2.1 2e: the ONE mesher builds EVERYTHING, baked corridor edges
    // included — the corridor renderer is gone. (Conform still strips baked
    // edges: corridorAuthor's engineered flatten owns that carve.)
    // Cap centerline curvature so neither the carriageway nor the sidewalk outer rail can
    // fold: keep the turn radius above the widest offset (half-width + sidewalk) + margin.
    double minR = netMinTurnRadius(road);
    RoadGraph raw = sampleNetGraph(road, minR);
    // Does the local-constraints pass (ADR-0052) promote any node to a roundabout?
    // Honour the junction policy (ADR-0075 P0): a generated road set
    // autoRoundabout=false, so we must NOT probe/promote with default rules here
    // (that re-promoted roundabouts the generator disabled).
    RoadRules rules;
    rules.autoRoundabout = road.look.autoRoundabout;
    RoadGraph g = applyConstraints(raw, rules);   // promoted graph (= the constrained graph)
    joinDanglingEndsLogged(g, road);              // the same joins/pull-backs the other graphs got

    // Grade separations (ADR-0051/0054): an edge on a higher layer is an overpass.
    // Instead of a separate bridge mesher, STAMP an absolute deck elevation onto
    // each higher-layer chain (a flat clearing span over the roads it crosses,
    // ramping down at rampGrade to grade) and let the ONE welder build it — deck,
    // piers, and the Δz grade-separation from the roads below — the same weld that
    // meshes the streets. (Retires buildLayeredRoadNetMesh; unifies the fork.)
    {
        int maxLayer = 0;
        for (const RoadEdge& e : g.edges) maxLayer = std::max(maxLayer, e.layer);
        if (maxLayer > 0) {
            const DesignRules& dr = defaultDesign();
            const double clearance = dr.clearance, deckThk = dr.deckThickness,
                         rampGrade = dr.rampGrade, groundSurf = road.look.lift;
            auto groundFn = [&](const Vec2& q) { return heightAt ? heightAt(q.x, q.y) : 0.0; };
            const int N = static_cast<int>(g.nodes.size());
            std::vector<std::vector<int>> incL(N);
            for (int L = 1; L <= maxLayer; ++L) {
                for (auto& v : incL) v.clear();
                for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei)
                    if (g.edges[ei].layer == L) {
                        incL[g.edges[ei].a].push_back(ei); incL[g.edges[ei].b].push_back(ei);
                    }
                auto degL = [&](int v) { return static_cast<int>(incL[v].size()); };
                auto other = [&](int e, int v) { return g.edges[e].a == v ? g.edges[e].b : g.edges[e].a; };
                std::vector<char> used(g.edges.size(), 0);
                auto walk = [&](int startV, int startE) {
                    std::vector<int> ns{startV};
                    int cur = startV, ce = startE;
                    for (;;) {
                        used[ce] = 1; int nx = other(ce, cur); ns.push_back(nx);
                        if (degL(nx) != 2) break;
                        int ne = -1;
                        for (int e2 : incL[nx]) if (e2 != ce && !used[e2]) { ne = e2; break; }
                        if (ne < 0) break; cur = nx; ce = ne;
                    }
                    return ns;
                };
                std::vector<std::vector<int>> chains;
                for (int v = 0; v < N; ++v)
                    if (degL(v) != 2 && degL(v) > 0)
                        for (int e0 : incL[v]) if (!used[e0]) chains.push_back(walk(v, e0));
                for (int e0 = 0; e0 < static_cast<int>(g.edges.size()); ++e0)   // pure rings
                    if (g.edges[e0].layer == L && !used[e0]) chains.push_back(walk(g.edges[e0].a, e0));
                for (const std::vector<int>& ns : chains) {
                    const int n = static_cast<int>(ns.size());
                    if (n < 2) continue;
                    std::vector<double> s(n, 0.0), minH(n);
                    for (int i = 1; i < n; ++i)
                        s[i] = s[i - 1] + (g.nodes[ns[i]].pos - g.nodes[ns[i - 1]].pos).length();
                    // Hold the deck FLAT over each LOWER road it passes above, clearing it by
                    // `clearance`. A chain node within (lower half-width + overhang) of a lower
                    // road's centreline is over the crossing — proximity, not a strict segment
                    // cross, because the sampler puts a shared vertex exactly at the crossing.
                    for (int i = 0; i < n; ++i) {
                        const Vec2 q = g.nodes[ns[i]].pos;
                        minH[i] = groundFn(q) + groundSurf;
                        for (const RoadEdge& e : g.edges) {
                            if (e.layer >= L) continue;                        // only clear roads below
                            const Vec2 a = g.nodes[e.a].pos, ab = g.nodes[e.b].pos - a;
                            const double L2 = ab.lengthSquared();
                            const double t = L2 < 1e-12 ? 0.0
                                : std::max(0.0, std::min(1.0, dot(q - a, ab) / L2));
                            if ((q - (a + ab * t)).length() > e.width * 0.5 + 5.0) continue;
                            minH[i] = std::max(minH[i], groundFn(q) + groundSurf + clearance + deckThk);
                        }
                    }
                    std::vector<double> deckY = clearanceProfile(s, minH, rampGrade);
                    for (int i = 0; i < n; ++i) {
                        // 2e: AUTHORED deck heights win. A baked corridor
                        // node already carries its solved profile — the
                        // clearance re-derivation must not clobber it (it
                        // would flatten the engineered viaduct onto generic
                        // layer heights).
                        if (g.nodes[ns[i]].elevAbsolute) continue;
                        g.nodes[ns[i]].elev = static_cast<Real>(deckY[i]);
                        g.nodes[ns[i]].elevAbsolute = true;                     // ride it through the weld
                    }
                }
            }
        }
    }

    // Semantic layer (#17): classify AFTER the layer-elevation stamping so
    // access bits see the final elevAbsolute truth.
    classifyRoadGraph(g, heightAt);

    // ONE MESHER (roads-v2 S6): every street net goes through the swept
    // lattice — carriageway bodies, junction pads, curb/sidewalk band loops,
    // and per-class structure (freeway kit, ramp decks, layered bridges).
    // The union weld (weldSolid), the SDF grid and the analytic fallback are
    // deleted from this path; the lattice IS the road mesher.
    RenderMesh rm = buildRoadNetLattice(g, heightAt, nullptr, road.look.sidewalk,
                                        road.look.curb, road.look.crosswalks, auditOut,
                                        road.look.cornerRadius,
                                        deckOut, road.look.perClassGrade);
    if (deckOut) deckOut->buildIndex();
    // Bounds + NaN audit: one NaN vertex poisons the bounding sphere and the
    // whole road entity frustum-culls to nothing, silently.
    std::size_t nans = 0;
    Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
    for (const Vertex& v : rm.vertices) {
        if (!std::isfinite(v.position.x) || !std::isfinite(v.position.y) ||
            !std::isfinite(v.position.z)) { ++nans; continue; }
        lo.x = std::min(lo.x, v.position.x); hi.x = std::max(hi.x, v.position.x);
        lo.y = std::min(lo.y, v.position.y); hi.y = std::max(hi.y, v.position.y);
        lo.z = std::min(lo.z, v.position.z); hi.z = std::max(hi.z, v.position.z);
    }
    LOG_INFO << "[roadmesh] " << g.edges.size() << " edges -> "
             << rm.vertices.size() << " verts, " << rm.indices.size() / 3
             << " tris, nan " << nans << ", y [" << lo.y << ", " << hi.y
             << "], x [" << lo.x << ", " << hi.x << "], z [" << lo.z << ", "
             << hi.z << "]";
    // RT_DUMP_ROADMESH=<path>: write the mesh's XZ vertex cloud so coverage can
    // be plotted against the road graph (which chains actually meshed?).
    if (const char* dp = std::getenv("RT_DUMP_ROADMESH")) {
        if (FILE* f = std::fopen(dp, "w")) {
            for (std::size_t i = 0; i < rm.vertices.size(); i += 16)
                std::fprintf(f, "%.1f %.1f\n", rm.vertices[i].position.x,
                             rm.vertices[i].position.z);
            std::fclose(f);
        }
    }
    return rm;
}


std::vector<TerrainFlatten> roadNetConformRegions(const RoadEntity& roadIn,
                                                  const RoadGroundFn& heightAt,
                                                  double shoulder,
                                                  double falloff, double maxGrade) {
    RT_PROFILE_ZONE_NAMED("roadNetConformRegions");
    // 2e: the conform is the ONE consumer that still strips baked corridor
    // edges — corridorAuthor's engineered flatten (at-grade windows only,
    // viaducts fly) owns that carve; carving the baked chains here too would
    // double-grade the interchange ground.
    const RoadEntity road = roadNetStreetsOnly(roadIn);
    std::vector<TerrainFlatten> out;
    if (!heightAt) return out;                           // flat road: nothing to carve
    // Mirror the DECK's own profile computation EXACTLY — the same constrained
    // graph, the same weldChainSpines decomposition (curve-sampled points,
    // per-chain widths), the same roadProfile smoothing at the weld's grade —
    // so the carve grades the ground to where the deck actually is. The old
    // independently-densified profile diverged by metres on slopes: the
    // grade-limited deck cut through hills the carve never lowered (device:
    // "the road is being buried by the terrain — it's not conforming").
    RoadGraph g = constrainedGraph(road, heightAt);      // grade to the roundabout, not the raw spokes
    (void)maxGrade;   // superseded: the carve must use the deck's own grade
    std::vector<UnionSpine> spines = weldChainSpines(g);
    // RT_JUNCTION_DUMP prints this too, beside the report's own fingerprint,
    // so the two graphs (this streets-only one and the report's full one) can
    // be compared on the same run — on a level with baked corridors they are
    // NOT expected to match, and the dump must say so rather than hide it.
    if (std::getenv("RT_POKE_SITE") || std::getenv("RT_JUNCTION_DUMP")) {
        double fp = 0;
        for (std::size_t si = 0; si < spines.size(); ++si)
            fp += spines[si].points.front().x * (si + 1) * 1e-3;
        LOG_INFO << "[conform-fingerprint] nodes=" << g.nodes.size()
                 << " edges=" << g.edges.size() << " spines=" << spines.size()
                 << " fp=" << fp;
    }
    // ONE profile source (plan P3.2): weldChainProfiles now reconciles mid-span
    // overlaps INSIDE the shared pass — the mesher rides the same reconciled
    // heights — so deck and carve cannot disagree at junctions. The old carve-
    // only minOverlapping left the higher deck floating up to 2.6 m above the
    // ground it never lowered (road_poke_probe metropolis, 8.4% verts >1 m).
    std::vector<std::vector<double>> profiles =
        weldChainProfiles(spines, heightAt, 0.0, kRoadMaxGrade,
                          road.look.sidewalk + 4.0,
                          road.look.perClassGrade ? &kDesignRules : nullptr);
    for (std::size_t si = 0; si < spines.size(); ++si) {
        const UnionSpine& sp = spines[si];
        if (profiles[si].size() < 2) continue;
        // AUTHORED elevated decks ride on piers — the ground must NOT be graded
        // up to them, or the terrain balloons into a ridge that buries the
        // flyover (weldChainProfiles returns the +Y deck for these). Skip their
        // conform; the piers span deck-to-ground and the surface roads they fly
        // over keep their own at-grade carve. (Ramp feet on sloped ground want a
        // partial carve — a later refinement; on flat ground none is needed.)
        if (!sp.yAbs.empty()) continue;
        std::vector<double> profile = profiles[si];
        // Carve a step BELOW the drivable profile, not exactly to it: the
        // terrain grid interpolates between its samples and can overshoot the
        // carve target past the road's small lift, patchily swallowing the
        // deck. 0.22 m keeps the deck proud; the curb skirt hides the step.
        for (double& hh : profile) hh -= kRoadConformStep;
        // Flatten out to the SIDEWALK's outer edge, not just the carriageway —
        // the sidewalk band rides the same smoothed profile and needs ground
        // graded under it too.
        // +2 m margin past the sidewalk's outer edge: corner bevels and the
        // curb skirt reach slightly beyond the band, and the flatten's full
        // strength must cover them before its falloff starts.
        std::vector<TerrainFlatten> r = roadConformRegions(
            sp.points, profile, sp.halfWidth + road.look.sidewalk + 2.0, shoulder,
            falloff);
        for (TerrainFlatten& f : r) f.owner = static_cast<int>(si);
        // NOTE (ADR-0075): the DaylightBatter earthwork was tried here and REGRESSED
        // the conform on the real CDLOD city — the batter rises too steeply near the
        // road, so coarse-LOD terrain samples poke through the deck (headless
        // road_poke_probe: 0.00% -> 0.14% device poke, worst 0.44 m -> 2.15 m). The
        // smoothstep feather (roadConformRegions' default) sits the road cleanly, so
        // we keep it. The batter mode stays available on TerrainFlatten for a future
        // attempt that samples the deck the way the LOD terrain does.
        out.insert(out.end(), r.begin(), r.end());
    }
    // JUNCTION PAD CARVE (plan P3.2 round 3): the chain footprints above are
    // RECTANGLES that stop at the chain ends, but the weld adds a DISC per
    // junction node (padCenters) whose lobes between the arms sit past every
    // rectangle — on sloped ground those lobes ride natural, uncarved terrain
    // and the deck stands metres proud (or the ground swallows the pad). Carve
    // each disc to the deck's own local plane, sampled from the SAME reconciled
    // profiles the mesher rides (nearest-spine, like weldSolid's heightOf).
    {
        auto deckAt = [&](const Vec2& q) {
            double best = 1e30, h = 0.0;
            for (std::size_t si = 0; si < spines.size(); ++si) {
                if (profiles[si].size() < 2) continue;
                const auto& pts = spines[si].points;
                for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
                    Vec2 ab = pts[i + 1] - pts[i];
                    double L2 = ab.lengthSquared();
                    double t = L2 < 1e-12
                                   ? 0.0
                                   : std::max(0.0, std::min(1.0, dot(q - pts[i], ab) / L2));
                    double d2 = (q - (pts[i] + ab * t)).lengthSquared();
                    if (d2 < best) {
                        best = d2;
                        h = profiles[si][i] + (profiles[si][i + 1] - profiles[si][i]) * t;
                    }
                }
            }
            return h;
        };
        std::vector<int> deg(g.nodes.size(), 0);
        std::vector<double> jw(g.nodes.size(), 0.0);
        for (const RoadEdge& e : g.edges) {
            ++deg[e.a]; ++deg[e.b];
            jw[e.a] = std::max(jw[e.a], static_cast<double>(e.width));
            jw[e.b] = std::max(jw[e.b], static_cast<double>(e.width));
        }
        for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v) {
            if (deg[v] < 3) continue;
            const Vec2 C = g.nodes[v].pos;
            // Cover the weld disc + the sidewalk wrap + the same margin the
            // chain footprints use, so the pad's full apron sits on graded ground.
            const double r = jw[v] * 0.5 * 1.02 + road.look.sidewalk + 2.0;
            // The deck's local plane: sampled along the steepest deck direction
            // through the node (8-direction probe), carved 0.22 m under like the
            // chains so the deck stays proud of the interpolating terrain grid.
            Vec2 gdir(0, 0);
            const double hC = deckAt(C);
            for (int k = 0; k < 8; ++k) {
                const double a = 2.0 * 3.14159265358979323846 * k / 8.0;
                const Vec2 d(std::cos(a), std::sin(a));
                gdir = gdir + d * (deckAt(C + d * r) - hC);
            }
            Vec2 axis = gdir.length() > 1e-6 ? normalize(gdir) : Vec2(1, 0);
            const Vec2 A = C - axis * r, B = C + axis * r;
            out.push_back(makeFlattenRamp(Vec3(A.x, 0, A.y), Vec3(B.x, 0, B.y),
                                          deckAt(A) - kRoadConformStep, deckAt(B) - kRoadConformStep, r,
                                          falloff));
            out.back().owner = -2 - v;   // junction pad for node v
        }
    }
    // FIX A (frontage seams): road regions outrank lot/building pads wherever
    // both cover — the street owns its corridor and verge; pads own the block
    // interior. Priority 1 also lets the LOD bake identify "near a road" for
    // its corner clamp (roadPlaneNear).
    for (TerrainFlatten& f : out) f.priority = 1;
    return out;
}

StructureSet buildRoadWalls(const RoadEntity& roadIn, const RoadGroundFn& heightAt,
                            const StructureParams& p) {
    StructureSet set;
    // CO-DESIGNED with the SMOOTHSTEP conform (roads-v2.1 R6a; the old
    // version priced walls off the DaylightBatter reach the conform no
    // longer uses). roadNetConformRegions feathers deck -> natural over
    // `falloff` metres past the graded corridor; a DEEP CUT leaves that
    // feather face near-vertical and raw (hillcity's first frame). The wall
    // stands just OUTSIDE the graded corridor and rises to the natural
    // ground at the feather's END, fronting the steep face — the terrain
    // still smoothsteps behind it, hidden. Shallow cuts (drop <= minWall)
    // keep their grass; fills stay open embankments (they read as slopes).
    const RoadEntity road = roadNetStreetsOnly(roadIn);   // corridor flies; no walls
    if (!heightAt) return set;                            // flat road: no walls
    RoadGraph g = constrainedGraph(road, heightAt);
    std::vector<UnionSpine> spines = weldChainSpines(g);
    std::vector<std::vector<double>> profiles =
        weldChainProfiles(spines, heightAt, 0.0, kRoadMaxGrade,
                          road.look.sidewalk + 4.0,
                          road.look.perClassGrade ? &kDesignRules : nullptr);
    const double falloff = 8.0;   // roadNetConformRegions' feather span

    for (std::size_t si = 0; si < spines.size(); ++si) {
        const UnionSpine& sp = spines[si];
        const int n = static_cast<int>(sp.points.size());
        if (static_cast<int>(profiles[si].size()) != n || n < 2) continue;
        if (!sp.yAbs.empty()) continue;   // authored elevated: piers, not walls
        const double flatHalf = sp.halfWidth + road.look.sidewalk + 2.0;   // graded corridor
        const double wallLat = flatHalf + 0.6;    // just outside the graded ground
        // DENSIFIED stations (~6 m): the spine's own points can span a whole
        // straight edge, and a single wall quad would chord an undulating
        // hill line instead of following it.
        std::vector<Vec2> pts;
        std::vector<double> deck;
        for (int k = 0; k + 1 < n; ++k) {
            const Vec2 A = sp.points[k], B = sp.points[k + 1];
            const double len = (B - A).length();
            const int div = std::max(1, static_cast<int>(std::ceil(len / 6.0)));
            for (int t = 0; t < div; ++t) {
                const double f = static_cast<double>(t) / div;
                pts.push_back(A + (B - A) * f);
                deck.push_back(profiles[si][k] +
                               (profiles[si][k + 1] - profiles[si][k]) * f);
            }
        }
        pts.push_back(sp.points[n - 1]);
        deck.push_back(profiles[si][n - 1]);
        const int m = static_cast<int>(pts.size());
        auto stationWall = [&](int side, int k, Vec3& topOut, double& dropOut) {
            int seg = std::min(k, m - 2);
            Vec2 dir = normalize(pts[seg + 1] - pts[seg]);
            Vec2 nrm = perp(dir);
            Vec2 at = pts[k] + nrm * (static_cast<double>(side) * wallLat);
            Vec2 end =
                pts[k] + nrm * (static_cast<double>(side) * (flatHalf + falloff));
            const double naturalEnd = heightAt(end.x, end.y);
            const double drop = naturalEnd - deck[k];   // cut face height
            topOut = Vec3(at.x, deck[k] + std::max(0.0, drop), at.y);
            dropOut = std::max(0.0, drop);
        };
        for (int side = -1; side <= 1; side += 2)
            for (int k = 0; k + 1 < m; ++k) {
                Vec3 topA, topB; double dropA, dropB;
                stationWall(side, k, topA, dropA);
                stationWall(side, k + 1, topB, dropB);
                if (dropA <= p.minWall && dropB <= p.minWall) continue;
                WallSegment w{topA, topB, dropA, dropB, true};
                const int seg = std::min(k, m - 2);
                const Vec2 nrm = perp(normalize(pts[seg + 1] - pts[seg]));
                w.out = Vec3(nrm.x * side, 0, nrm.y * side);
                w.bench = falloff - 0.6 + 0.4;   // cover the feather to natural
                set.walls.push_back(w);
                set.colliderEdges.push_back({topA, topB});
            }
    }
    set.mesh = bakeWallMesh(set.walls, p.color);
    return set;
}

}  // namespace engine
