// THE FLOORPLAN CONFORMANCE CENSUS (device, after two rounds of seam fixes
// still left hovering facades: "we need better tests to prove that the entire
// floorplan of the building is conformed to the surface").
//
// The prior tests proved component invariants — winding, offsets, pad math —
// and every one of them passed while the COMPOSED result was wrong. This gate
// tests the composition, and it does it against the two truths the player
// actually experiences:
//
//   1. the DRAWN ground: the final flatten set (roads + in-pass block grades
//      + building pads) sampled the way the CDLOD mesher samples it — leaf
//      dilate, the FIX-A road clamp, and bilinear interpolation between leaf
//      grid corners (mid-cell sag is real geometry, so it is real here);
//   2. the EMITTED geometry: the census never re-derives the foundation
//      formula — it asks the grown Concrete part for its lowest vertex near
//      each perimeter sample. A broken emitter cannot agree with this test
//      by construction.
//
// Claim, per built lot, per ~1 m perimeter sample:
//   gap:    foundation geometry reaches below the drawn ground (no daylight
//           under any wall), and
//   burial: the drawn ground stays below baseY + plinth tolerance (no wall
//           buried past its plinth).
// The distant tier (appendLotMassBox) must satisfy the same gap claim.
#include "test_framework.h"

#include "../src/engine/procgen/city/city_lots.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/procgen/terrain.h"
#include "../src/engine/procgen/noise.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <unordered_map>
#include <vector>

using namespace engine;  // namespace migration (ADR-0015)

namespace {

// A believable hillside: a steady 7% grade with a low ridge across it —
// enough relief that frontage strips, cross-slopes and pad falloffs all
// occur, while staying under the relief gate for most lots.
double hillside(double x, double z) {
    return 0.07 * x + 1.8 * std::sin(z * 0.02) + 0.9 * std::sin(x * 0.013);
}

// The loader's pad-apron dilation, replicated (level_loader kPadApron):
// vertices pushed out from the centroid by the apron distance.
Poly2 dilatedPlan(const Poly2& plan, double apron) {
    Vec2 c(0, 0);
    for (const Vec2& v : plan) c = c + v;
    c = c * (1.0 / static_cast<double>(plan.size()));
    Poly2 out;
    out.reserve(plan.size());
    for (const Vec2& v : plan) {
        const Vec2 d = v - c;
        const double l = d.length();
        out.push_back(l > 1e-6 ? c + d * ((l + apron) / l) : v);
    }
    return out;
}

// The DRAWN ground: reproduce generateLodNodeMesh's leaf-vertex math over
// the final flatten set, then bilerp between grid corners like the mesh does.
struct MeshGround {
    const TerrainParams& tp;
    const Noise& noise;
    double leafStep;

    double corner(double x, double z) const {
        double y = terrainHeight(tp, noise, x, z, leafStep * 1.45);
        if (tp.flattenIndex) {
            const double rp = roadPlaneNear(*tp.flattenIndex, tp.flatten, x, z,
                                            leafStep * 1.6);
            if (rp < 1e29 && !padPlaneAbove(*tp.flattenIndex, tp.flatten, x, z, rp,
                                       leafStep * 1.45))
                y = std::min(y, rp);
        }
        return y;
    }
    double at(double x, double z) const {
        const double gx = std::floor(x / leafStep) * leafStep;
        const double gz = std::floor(z / leafStep) * leafStep;
        const double tx = (x - gx) / leafStep, tz = (z - gz) / leafStep;
        const double y00 = corner(gx, gz), y10 = corner(gx + leafStep, gz);
        const double y01 = corner(gx, gz + leafStep);
        const double y11 = corner(gx + leafStep, gz + leafStep);
        return (y00 * (1 - tx) + y10 * tx) * (1 - tz) +
               (y01 * (1 - tx) + y11 * tx) * tz;
    }
};

// Lowest emitted vertex near an XZ point — the geometry's REAL reach there.
// Grid-bucketed so the census stays O(samples).
struct LowestVertexIndex {
    std::unordered_map<long long, double> lowest;
    double cell;
    explicit LowestVertexIndex(const RenderMesh& m, double cellSize)
        : cell(cellSize) {
        for (const Vertex& v : m.vertices) {
            const long long k = key(v.position.x, v.position.z);
            auto it = lowest.find(k);
            if (it == lowest.end() || v.position.y < it->second)
                lowest[k] = v.position.y;
        }
    }
    long long key(double x, double z) const {
        const long long ix = static_cast<long long>(std::floor(x / cell));
        const long long iz = static_cast<long long>(std::floor(z / cell));
        return ix * 1000003LL + iz;
    }
    // Lowest Y among the 3x3 cells around (x,z); +inf when nothing near.
    double near(double x, double z) const {
        double best = 1e30;
        for (int dx = -1; dx <= 1; ++dx)
            for (int dz = -1; dz <= 1; ++dz) {
                auto it = lowest.find(key(x + dx * cell, z + dz * cell));
                if (it != lowest.end()) best = std::min(best, it->second);
            }
        return best;
    }
};

struct CensusResult {
    int lots = 0;
    int gapViolations = 0;      // daylight under a wall
    int burialViolations = 0;   // wall buried past plinth tolerance
    double worstGap = 0;        // metres of daylight
    double worstBurial = 0;
    Vec2 worstGapAt{0, 0};
};

CensusResult runCensus(const std::vector<LotBuilding>& lots,
                       const MeshGround& mesh,
                       const LowestVertexIndex& concrete,
                       double plinthTolerance) {
    CensusResult r;
    for (const LotBuilding& lb : lots) {
        if (lb.type == "park" || lb.type == "green") continue;
        if (lb.plan.size() < 3) continue;
        ++r.lots;
        bool gapHit = false, buryHit = false;
        for (std::size_t i = 0; i < lb.plan.size(); ++i) {
            const Vec2& a = lb.plan[i];
            const Vec2& b = lb.plan[(i + 1) % lb.plan.size()];
            const double len = (b - a).length();
            const int steps = std::max(1, static_cast<int>(std::ceil(len)));
            for (int s = 0; s <= steps; ++s) {
                const Vec2 q = a + (b - a) * (static_cast<double>(s) / steps);
                const double ground = mesh.at(q.x, q.y);
                // gap: whatever geometry exists here must reach below ground.
                const double reach = std::min(concrete.near(q.x, q.y),
                                              static_cast<double>(lb.baseY));
                const double gap = reach - ground;
                if (gap > 0.10) {
                    gapHit = true;
                    if (gap > r.worstGap) { r.worstGap = gap; r.worstGapAt = q; }
                }
                // burial: ground must not swallow the wall past its plinth.
                const double burial = ground - lb.baseY;
                if (burial > plinthTolerance) {
                    buryHit = true;
                    r.worstBurial = std::max(r.worstBurial, burial);
                }
            }
        }
        if (gapHit) ++r.gapViolations;
        if (buryHit) ++r.burialViolations;
    }
    return r;
}

}  // namespace

TEST_CASE(floorplan_conformance_census_on_a_hillside_net) {
    // A small road net across the hillside — a ring plus a through-street, so
    // the census sees frontage strips on uphill, downhill and cross-slope
    // sides, junction conform overlap, and in-pass block grading.
    RoadEntity net;
    net.look.defaultWidth = 12.0;
    net.look.sidewalk = 3.5;   // metro-like: road conform reaches well outboard
    net.look.autoRoundabout = false;
    net.graph.nodes = { RoadNode{Vec2(-80, -60)}, RoadNode{Vec2(80, -60)},
                        RoadNode{Vec2(80, 60)},   RoadNode{Vec2(-80, 60)},
                        RoadNode{Vec2(0, -60)},   RoadNode{Vec2(0, 60)} };
    net.graph.edges = { RoadEdge{0, 4, 12.0}, RoadEdge{4, 1, 12.0},
                        RoadEdge{1, 2, 12.0}, RoadEdge{2, 5, 12.0},
                        RoadEdge{5, 3, 12.0}, RoadEdge{3, 0, 12.0},
                        RoadEdge{4, 5, 12.0} };

    const RoadGroundFn natural = hillside;
    std::vector<TerrainFlatten> roadFlatten = roadNetConformRegions(net, natural);

    // The loader's lot sampler: road-carved ground (script flattens: none).
    TerrainParams lotTp;
    lotTp.erodedBase = std::make_shared<const std::function<double(double, double)>>(
        natural);
    lotTp.flatten = roadFlatten;
    rebuildFlattenIndex(lotTp);
    Noise noise(7);
    auto lotGround = [&](Real x, Real z) {
        return terrainHeight(lotTp, noise, x, z);
    };

    LotParams lp;
    lp.seed = 11;
    lp.buildChance = 1.0;      // every parcel builds — maximum census coverage
    lp.plinth = 0.45;          // the hill levels' authored value
    lp.roadMargin = 3.5;
    NetLotResult grown = growLotBuildingsOnNets({net}, lp, EdgeBlockParams{},
                                                /*roadClearance=*/4.1,
                                                lotGround);
    CHECK(!grown.lots.empty());

    // The FINAL field, assembled the loader's way: roads + in-pass grades +
    // building pads (apron-dilated).
    TerrainParams finalTp = lotTp;
    finalTp.flatten.insert(finalTp.flatten.end(), grown.gradeFlatten.begin(),
                           grown.gradeFlatten.end());
    int pads = 0;
    for (const LotBuilding& lb : grown.lots) {
        if (lb.type == "park" || lb.type == "green" || lb.plan.size() < 3)
            continue;
        std::vector<Vec3> poly;
        for (const Vec2& v : dilatedPlan(lb.plan, 2.2))
            poly.push_back(Vec3(v.x, 0, v.y));
        TerrainFlatten padF = makeFlattenPad(std::move(poly), lb.groundY, 5.0);
        padF.priority = kPadFlattenPriority;   // as the loader sets it
        finalTp.flatten.push_back(std::move(padF));
        ++pads;
    }
    CHECK(pads > 0);
    rebuildFlattenIndex(finalTp);

    // metro_v2_test's leaf step (worldHalf 1380, numLods 6, gridRes 32).
    const MeshGround mesh{finalTp, noise, 2.695};

    // The emitted concrete (foundations live in PartId::Concrete).
    const RenderMesh& concretePart =
        grown.parts[static_cast<std::size_t>(PartId::Concrete)];
    const LowestVertexIndex concrete(concretePart, 1.0);

    const CensusResult r =
        runCensus(grown.lots, mesh, concrete, /*plinthTolerance=*/0.55);
    std::printf(
        "[census] lots=%d gapViolations=%d burialViolations=%d worstGap=%.2f m "
        "at (%.1f, %.1f) worstBurial=%.2f m\n",
        r.lots, r.gapViolations, r.burialViolations, r.worstGap, r.worstGapAt.x,
        r.worstGapAt.y, r.worstBurial);

    CHECK(r.lots > 10);
    // THE GATE: no daylight under any wall, no wall buried past its plinth.
    CHECK(r.gapViolations == 0);
    CHECK(r.burialViolations == 0);
}

TEST_CASE(distant_tier_mass_boxes_reach_below_the_drawn_ground) {
    // The same claim for the far tier: past detailDistance a building IS its
    // mass box, so the box itself must reach below the drawn ground.
    RoadEntity net;
    net.look.defaultWidth = 12.0;
    net.look.sidewalk = 3.5;
    net.look.autoRoundabout = false;
    // Block sized so the frontage ring yields a real SAMPLE of mass boxes.
    // It used to be 160x100 and produced 8 — but 8 counted lots that OVERLAPPED
    // each other: the parcel backstop's cut was derived from an edge of the
    // already-placed lot, and a duplicated corner (which clipping produces
    // whenever a vertex lands on the cut line) is a zero-length edge whose
    // normal is (0,0), so `dot(0,p) <= 0` held everywhere and the "cut"
    // returned the lot WHOLE — then won the largest-area contest, because
    // removing nothing is always biggest. With the cut actually separating
    // lots, the honest yield on that ring is 5, which tripped the sample floor
    // below. The floor is not the gate — `violations == 0` is — so the sample
    // is restored by giving the walk more frontage (440x300 -> 8 boxes), never
    // by lowering the bar.
    net.graph.nodes = { RoadNode{Vec2(-220, -150)}, RoadNode{Vec2(220, -150)},
                        RoadNode{Vec2(220, 150)},  RoadNode{Vec2(-220, 150)} };
    net.graph.edges = { RoadEdge{0, 1, 12.0}, RoadEdge{1, 2, 12.0},
                        RoadEdge{2, 3, 12.0}, RoadEdge{3, 0, 12.0} };
    const RoadGroundFn natural = hillside;
    std::vector<TerrainFlatten> roadFlatten = roadNetConformRegions(net, natural);
    TerrainParams lotTp;
    lotTp.erodedBase = std::make_shared<const std::function<double(double, double)>>(
        natural);
    lotTp.flatten = roadFlatten;
    rebuildFlattenIndex(lotTp);
    Noise noise(7);
    auto lotGround = [&](Real x, Real z) {
        return terrainHeight(lotTp, noise, x, z);
    };
    LotParams lp;
    lp.seed = 5;
    lp.buildChance = 1.0;
    lp.plinth = 0.45;
    lp.roadMargin = 3.5;
    NetLotResult grown = growLotBuildingsOnNets({net}, lp, EdgeBlockParams{},
                                                4.1, lotGround);
    CHECK(!grown.lots.empty());

    TerrainParams finalTp = lotTp;
    finalTp.flatten.insert(finalTp.flatten.end(), grown.gradeFlatten.begin(),
                           grown.gradeFlatten.end());
    for (const LotBuilding& lb : grown.lots) {
        if (lb.type == "park" || lb.type == "green" || lb.plan.size() < 3)
            continue;
        std::vector<Vec3> poly;
        for (const Vec2& v : dilatedPlan(lb.plan, 2.2))
            poly.push_back(Vec3(v.x, 0, v.y));
        TerrainFlatten padF = makeFlattenPad(std::move(poly), lb.groundY, 5.0);
        padF.priority = kPadFlattenPriority;   // as the loader sets it
        finalTp.flatten.push_back(std::move(padF));
    }
    rebuildFlattenIndex(finalTp);
    const MeshGround mesh{finalTp, noise, 2.695};

    int boxes = 0, violations = 0;
    double worst = 0;
    for (const LotBuilding& lb : grown.lots) {
        if (lb.type == "park" || lb.type == "green" || lb.plan.size() < 3)
            continue;
        RenderMesh box;
        // As the loader now calls it: bottom = min perimeter ground - 0.5.
        double lo = 1e30;
        for (std::size_t i = 0; i < lb.plan.size(); ++i) {
            const Vec2& a = lb.plan[i];
            const Vec2& b = lb.plan[(i + 1) % lb.plan.size()];
            lo = std::min(lo, mesh.at(a.x, a.y));
            const Vec2 m2 = (a + b) * 0.5;
            lo = std::min(lo, mesh.at(m2.x, m2.y));
        }
        appendLotMassBox(box, lb, Vec3(0.6, 0.6, 0.6), Vec3(0.4, 0.4, 0.4),
                         static_cast<Real>(lo - 0.5));
        if (box.vertices.empty()) continue;
        ++boxes;
        double boxBottom = 1e30;
        for (const Vertex& v : box.vertices)
            boxBottom = std::min(boxBottom, static_cast<double>(v.position.y));
        double groundMin = 1e30;
        for (std::size_t i = 0; i < lb.plan.size(); ++i) {
            const Vec2& a = lb.plan[i];
            const Vec2& b = lb.plan[(i + 1) % lb.plan.size()];
            const int steps =
                std::max(1, static_cast<int>(std::ceil((b - a).length())));
            for (int s = 0; s <= steps; ++s) {
                const Vec2 q = a + (b - a) * (static_cast<double>(s) / steps);
                groundMin = std::min(groundMin, mesh.at(q.x, q.y));
            }
        }
        const double gap = boxBottom - groundMin;
        if (gap > 0.10) {
            ++violations;
            worst = std::max(worst, gap);
        }
    }
    std::printf("[census] massBoxes=%d violations=%d worstGap=%.2f m\n", boxes,
                violations, worst);
    CHECK(boxes > 5);
    CHECK(violations == 0);
}
