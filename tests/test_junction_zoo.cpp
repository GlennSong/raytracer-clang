// THE JUNCTION ZOO (roads-v2.1 R2, drive feedback A1/A2). One fixture per
// junction archetype, each verified three ways the drive probe alone is
// structurally blind to:
//   STACKED SURFACES — no two up-facing triangles cover the same ground in
//     the junction disc (the "floating ribbon above the pad" at the acute
//     4-way, the criss-crossed polygons of old);
//   CURB CLOSURE — every curb/sidewalk band loop in the disc is CLOSED
//     (open ribbon ends read as unfinished geometry at T-junctions);
//   CROSS DRIVES — every arm-to-arm path through the junction drives clean
//     (holes/steps/blocked = 0), not just the arms individually.
// The fixtures run the REAL pipeline: RoadEntity -> buildRoadNetMesh.
#include "test_framework.h"
#include "drive_probe.h"

#include "../src/engine/procgen/city/corridor_bake.h"
#include "../src/engine/procgen/city/corridor_mesh.h"
#include "../src/engine/procgen/city/road_net.h"
#include <cmath>
#include <cstdio>
#include <map>
#include <tuple>
#include <vector>

using namespace engine;

namespace {

const RoadGroundFn flatGround = [](double, double) { return 0.0; };

RoadEntity baseNet() {
    RoadEntity net;
    net.look.defaultWidth = 8.0;
    net.look.sidewalk = 2.0;
    net.look.curb = 0.15;
    net.look.autoRoundabout = false;
    return net;
}

// Up-facing coverage census on a grid over the junction disc: cells covered
// by 2+ up-facing triangles from DIFFERENT height bands are stacked surface.
int stackedCells(const RenderMesh& m, const Vec2& c, double radius) {
    const double cell = 0.8;
    const int N = static_cast<int>(std::ceil(2 * radius / cell));
    std::vector<std::vector<double>> hits(N * N);
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& A = m.vertices[m.indices[t]].position;
        const Vec3& B = m.vertices[m.indices[t + 1]].position;
        const Vec3& C = m.vertices[m.indices[t + 2]].position;
        const Vec3 n = cross(C - A, B - A);
        if (n.y <= 1e-9) continue;                     // up-facing only
        const Vec3 cen = (A + B + C) * (1.0 / 3.0);
        const double dx = cen.x - c.x, dz = cen.z - c.y;
        if (dx * dx + dz * dz > radius * radius) continue;
        const int ix = static_cast<int>((dx + radius) / cell);
        const int iz = static_cast<int>((dz + radius) / cell);
        if (ix < 0 || iz < 0 || ix >= N || iz >= N) continue;
        hits[iz * N + ix].push_back(cen.y);
    }
    int stacked = 0;
    for (int iz = 0; iz < N; ++iz)
        for (int ix = 0; ix < N; ++ix) {
            const auto& hs = hits[iz * N + ix];
            if (hs.size() < 2) continue;
            double lo = hs[0], hi = hs[0];
            for (double h : hs) { lo = std::min(lo, h); hi = std::max(hi, h); }
            // Two surfaces at materially different heights over one cell: a
            // ribbon floating over a pad. Same-height duplicates (shared
            // seams, curb top vs sidewalk at +curb) get a 0.10 m allowance.
            if (hi - lo > 0.10) {
                ++stacked;
                if (std::getenv("RT_ZOO_DEBUG"))
                    std::printf("[zoo]   stacked cell (%.1f, %.1f) y %.2f..%.2f\n",
                                c.x - radius + (ix + 0.5) * cell,
                                c.y - radius + (iz + 0.5) * cell, lo, hi);
            }
        }
    return stacked;
}

// OPEN CURB ENDS (A2: "the ribbon sometimes doesn't have a side to give it
// an enclosed shape appearance... on a T-junction"): a properly capped curb
// band has no tall vertical BOUNDARY edges (edges used by exactly one
// triangle) — an open profile end leaks a vertical rectangle's rim.
int openCurbEnds(const RenderMesh& m, const Vec2& c, double radius) {
    // Edges keyed by QUANTIZED POSITION, not vertex index: the emitters
    // duplicate vertices at every segment boundary, so coincident seam edges
    // are topologically unshared yet geometrically closed. Only an edge with
    // no coincident partner is truly OPEN.
    auto key = [](const Vec3& p) {
        return std::make_tuple(llround(p.x * 500.0), llround(p.y * 500.0),
                               llround(p.z * 500.0));
    };
    std::map<std::pair<std::tuple<long long, long long, long long>,
                       std::tuple<long long, long long, long long>>,
             int>
        uses;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
        for (int e = 0; e < 3; ++e) {
            auto a = key(m.vertices[m.indices[t + e]].position);
            auto b = key(m.vertices[m.indices[t + (e + 1) % 3]].position);
            if (b < a) std::swap(a, b);
            ++uses[{ a, b }];
        }
    int open = 0;
    for (const auto& kv : uses) {
        if (kv.second != 1) continue;
        const auto& A = kv.first.first;
        const auto& B = kv.first.second;
        const double ax = std::get<0>(A) / 500.0, ay = std::get<1>(A) / 500.0,
                     az = std::get<2>(A) / 500.0;
        const double bx = std::get<0>(B) / 500.0, by = std::get<1>(B) / 500.0,
                     bz = std::get<2>(B) / 500.0;
        const double dx = (ax + bx) * 0.5 - c.x, dz = (az + bz) * 0.5 - c.y;
        if (dx * dx + dz * dz > radius * radius) continue;
        // A true OPEN END is a VERTICAL rim: both endpoints at one xz, a
        // curb-scale height span. (Long slanted once-used edges are mm-scale
        // pad/band seam cracks — a lesser defect class, gated separately.)
        const double runXZ = std::hypot(ax - bx, az - bz);
        const double dy = std::fabs(ay - by);
        if (runXZ < 0.05 && dy > 0.10 && dy < 0.60) {
            ++open;
            if (std::getenv("RT_ZOO_DEBUG"))
                std::printf("[zoo]   open edge (%.2f,%.2f,%.2f)-(%.2f,%.2f,%.2f)\n",
                            ax, ay, az, bx, by, bz);
        }
    }
    return open;
}

// Drive every ordered arm pair straight through the junction.
int badCrossDrives(const RenderMesh& m, const RoadEntity& net, int node,
                   double reach) {
    const Vec2 c = net.graph.nodes[node].pos;
    std::vector<Vec2> arms;
    for (const auto& e : net.graph.edges) {
        int other = -1;
        if (e.a == node) other = e.b;
        if (e.b == node) other = e.a;
        if (other < 0) continue;
        Vec2 d = net.graph.nodes[other].pos - c;
        const double l = d.length();
        if (l > 1e-6) arms.push_back(d * (1.0 / l));
    }
    int bad = 0;
    for (std::size_t i = 0; i < arms.size(); ++i)
        for (std::size_t j = 0; j < arms.size(); ++j) {
            if (i == j) continue;
            std::vector<Vec3> path;
            for (double s = -reach; s <= reach; s += 2.0) {
                const Vec2 d = s < 0 ? arms[i] : arms[j];
                const Vec2 p = c + d * std::fabs(s);
                path.push_back(Vec3(p.x, 0.3, p.y));
            }
            driveprobe::Report rep;
            driveprobe::drivePath(m, path, rep);
            if (rep.holes + rep.steps + rep.blocked > 0) {
                std::printf("[zoo]   arm %zu->%zu: holes=%d steps=%d blocked=%d\n",
                            i, j, rep.holes, rep.steps, rep.blocked);
                ++bad;
            }
        }
    return bad;
}

void checkJunction(const char* tag, RoadEntity& net, int node, double radius,
                   const RoadGroundFn& ground = flatGround) {
    RenderMesh m = buildRoadNetMesh(net, ground);
    CHECK(!m.vertices.empty());
    const int stacked = stackedCells(m, net.graph.nodes[node].pos, radius);
    const int badDrives = badCrossDrives(m, net, node, radius);
    const int curbOpen = openCurbEnds(m, net.graph.nodes[node].pos, radius);
    std::printf("[zoo] %s: stacked=%d badDrives=%d curbOpen=%d\n", tag,
                stacked, badDrives, curbOpen);
    CHECK(stacked == 0);
    CHECK(badDrives == 0);
    CHECK(curbOpen == 0);
}

}  // namespace

TEST_CASE(zoo_orthogonal_four_way) {
    RoadEntity net = baseNet();
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(80, 0)},
                        RoadNode{Vec2(-80, 0)}, RoadNode{Vec2(0, 80)},
                        RoadNode{Vec2(0, -80)} };
    net.graph.edges = { RoadEdge{ 0, 1, 8.0 }, RoadEdge{ 0, 2, 8.0 },
                        RoadEdge{ 0, 3, 8.0 }, RoadEdge{ 0, 4, 8.0 } };
    checkJunction("ortho4", net, 0, 22.0);
}

TEST_CASE(zoo_acute_four_way) {
    // Glenn's A1 find: "a 4-way where two roads merge at a sharp acute angle
    // and one of the roads remains a flat ribbon... It floats above it."
    RoadEntity net = baseNet();
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(80, 0)},
                        RoadNode{Vec2(-80, 0)},
                        RoadNode{Vec2(76, 26)},  // ~19 deg off the +x arm: acute pair
                        RoadNode{Vec2(0, -80)} };
    net.graph.edges = { RoadEdge{ 0, 1, 8.0 }, RoadEdge{ 0, 2, 8.0 },
                        RoadEdge{ 0, 3, 8.0 }, RoadEdge{ 0, 4, 8.0 } };
    checkJunction("acute4", net, 0, 24.0);
}

TEST_CASE(zoo_tee) {
    RoadEntity net = baseNet();
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(80, 0)},
                        RoadNode{Vec2(-80, 0)}, RoadNode{Vec2(0, 80)} };
    net.graph.edges = { RoadEdge{ 0, 1, 8.0 }, RoadEdge{ 0, 2, 8.0 },
                        RoadEdge{ 0, 3, 8.0 } };
    checkJunction("tee", net, 0, 20.0);
}

TEST_CASE(zoo_skew_tee_mixed_widths) {
    // A2's curb failures showed at Ts with unequal arms: a wide arterial met
    // by a narrow local at a slant.
    RoadEntity net = baseNet();
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(90, 0)},
                        RoadNode{Vec2(-90, 0)}, RoadNode{Vec2(34, 74)} };
    net.graph.edges = { RoadEdge{ 0, 1, 13.0 }, RoadEdge{ 0, 2, 13.0 },
                        RoadEdge{ 0, 3, 7.0 } };
    checkJunction("skewT", net, 0, 22.0);
}

TEST_CASE(zoo_five_arm) {
    RoadEntity net = baseNet();
    net.graph.nodes = { RoadNode{Vec2(0, 0)},   RoadNode{Vec2(80, 6)},
                        RoadNode{Vec2(-78, -14)}, RoadNode{Vec2(18, 80)},
                        RoadNode{Vec2(-30, 74)},  RoadNode{Vec2(24, -78)} };
    net.graph.edges = { RoadEdge{ 0, 1, 8.0 }, RoadEdge{ 0, 2, 8.0 },
                        RoadEdge{ 0, 3, 8.0 }, RoadEdge{ 0, 4, 8.0 },
                        RoadEdge{ 0, 5, 8.0 } };
    checkJunction("five", net, 0, 24.0);
}

TEST_CASE(zoo_short_arm_pair) {
    // Two junctions closer than their combined radii: the S4 compound-pad
    // case, now under the stacked-surface lens.
    RoadEntity net = baseNet();
    net.graph.nodes = { RoadNode{Vec2(0, 0)},  RoadNode{Vec2(18, 0)},   // the too-close pair
                        RoadNode{Vec2(-80, 0)}, RoadNode{Vec2(98, 0)},
                        RoadNode{Vec2(0, 80)},  RoadNode{Vec2(18, -80)} };
    net.graph.edges = { RoadEdge{ 0, 2, 8.0 }, RoadEdge{ 0, 1, 8.0 },
                        RoadEdge{ 1, 3, 8.0 }, RoadEdge{ 0, 4, 8.0 },
                        RoadEdge{ 1, 5, 8.0 } };
    checkJunction("shortpair", net, 0, 30.0);
}

TEST_CASE(zoo_ramp_landing) {
    // A baked ramp lands on a street: the band must GAP across the ramp's
    // mouth (no curbing a ramp shut) — and the gap's CUT ENDS must be capped
    // (A2: "the ribbon sometimes doesn't have a side to give it an enclosed
    // shape appearance").
    RoadEntity net = baseNet();
    net.graph.nodes = { RoadNode{Vec2(-40, -150)}, RoadNode{Vec2(60, -150)},
                        RoadNode{Vec2(150, -150)}, RoadNode{Vec2(280, -150)} };
    net.graph.edges = { RoadEdge{ 0, 1, 8.0 }, RoadEdge{ 1, 2, 8.0 },
                        RoadEdge{ 2, 3, 8.0 } };
    CorridorDef def;
    def.horizontal = Alignment::fromPolyline({ Vec2(-100, 0), Vec2(500, 0) },
                                             300.0, 20.0);
    const Real L = def.horizontal.length();
    def.vertical.pvis = { { 0.0, 9.0, 0.0 }, { L, 9.0, 0.0 } };
    def.lanes.throughLanes = 3;
    ExitDef e;
    e.station = 250.0;
    e.upStation = true;
    e.target = Vec2(150, -150);
    e.targetY = 0.0;
    def.exits.push_back(e);
    CorridorAuthoring au =
        corridorAuthor(def, [](Real, Real) { return Real(0); }, 3.0);
    CHECK(!au.rampPaths.empty());
    CHECK(!au.rampPaths[0].pts.empty());
    bakeCorridorIntoNet(net, def, au.rampPaths, {}, flatGround);
    checkJunction("landing", net, 2, 22.0);
}

TEST_CASE(zoo_tee_on_hills) {
    // The flat zoo hides everything terrain touches (the standing lesson):
    // the same T on rolling ground.
    RoadEntity net = baseNet();
    const RoadGroundFn hills = [](double x, double z) {
        return 2.5 * std::sin(x * 0.045) + 1.8 * std::cos(z * 0.06);
    };
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(80, 0)},
                        RoadNode{Vec2(-80, 0)}, RoadNode{Vec2(0, 80)} };
    net.graph.edges = { RoadEdge{ 0, 1, 8.0 }, RoadEdge{ 0, 2, 8.0 },
                        RoadEdge{ 0, 3, 8.0 } };
    checkJunction("hillT", net, 0, 20.0, hills);
}
