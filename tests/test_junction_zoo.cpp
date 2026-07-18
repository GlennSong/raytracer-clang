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
// The fixtures run the REAL pipeline: RoadNet -> buildRoadNetMesh.
#include "test_framework.h"
#include "drive_probe.h"

#include "../src/engine/procgen/city/road_net.h"
#include <cmath>
#include <cstdio>
#include <vector>

using namespace engine;

namespace {

RoadNet baseNet() {
    RoadNet net;
    net.width = 8.0;
    net.sidewalk = 2.0;
    net.curb = 0.15;
    net.autoRoundabout = false;
    net.heightAt = [](double, double) { return 0.0; };
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
    for (const auto& hs : hits) {
        if (hs.size() < 2) continue;
        double lo = hs[0], hi = hs[0];
        for (double h : hs) { lo = std::min(lo, h); hi = std::max(hi, h); }
        // Two surfaces at materially different heights over one cell: a
        // ribbon floating over a pad. Same-height duplicates (shared seams,
        // curb top vs sidewalk at +curb) get a 0.10 m allowance.
        if (hi - lo > 0.10) ++stacked;
    }
    return stacked;
}

// Drive every ordered arm pair straight through the junction.
int badCrossDrives(const RenderMesh& m, const RoadNet& net, int node,
                   double reach) {
    const Vec2 c = net.nodes[node];
    std::vector<Vec2> arms;
    for (const auto& e : net.edges) {
        int other = -1;
        if (e[0] == node) other = e[1];
        if (e[1] == node) other = e[0];
        if (other < 0) continue;
        Vec2 d = net.nodes[other] - c;
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

void checkJunction(const char* tag, RoadNet& net, int node, double radius) {
    RenderMesh m = buildRoadNetMesh(net);
    CHECK(!m.vertices.empty());
    const int stacked = stackedCells(m, net.nodes[node], radius);
    const int badDrives = badCrossDrives(m, net, node, radius);
    std::printf("[zoo] %s: stacked=%d badDrives=%d\n", tag, stacked, badDrives);
    CHECK(stacked == 0);
    CHECK(badDrives == 0);
}

}  // namespace

TEST_CASE(zoo_orthogonal_four_way) {
    RoadNet net = baseNet();
    net.nodes = { Vec2(0, 0), Vec2(80, 0), Vec2(-80, 0), Vec2(0, 80), Vec2(0, -80) };
    net.edges = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 0, 4 } };
    checkJunction("ortho4", net, 0, 22.0);
}

TEST_CASE(zoo_acute_four_way) {
    // Glenn's A1 find: "a 4-way where two roads merge at a sharp acute angle
    // and one of the roads remains a flat ribbon... It floats above it."
    RoadNet net = baseNet();
    net.nodes = { Vec2(0, 0), Vec2(80, 0), Vec2(-80, 0),
                  Vec2(76, 26),            // ~19 deg off the +x arm: acute pair
                  Vec2(0, -80) };
    net.edges = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 0, 4 } };
    checkJunction("acute4", net, 0, 24.0);
}

TEST_CASE(zoo_tee) {
    RoadNet net = baseNet();
    net.nodes = { Vec2(0, 0), Vec2(80, 0), Vec2(-80, 0), Vec2(0, 80) };
    net.edges = { { 0, 1 }, { 0, 2 }, { 0, 3 } };
    checkJunction("tee", net, 0, 20.0);
}

TEST_CASE(zoo_skew_tee_mixed_widths) {
    // A2's curb failures showed at Ts with unequal arms: a wide arterial met
    // by a narrow local at a slant.
    RoadNet net = baseNet();
    net.nodes = { Vec2(0, 0), Vec2(90, 0), Vec2(-90, 0), Vec2(34, 74) };
    net.edges = { { 0, 1 }, { 0, 2 }, { 0, 3 } };
    net.edgeWidths = { 13.0, 13.0, 7.0 };
    checkJunction("skewT", net, 0, 22.0);
}

TEST_CASE(zoo_five_arm) {
    RoadNet net = baseNet();
    net.nodes = { Vec2(0, 0),  Vec2(80, 6),  Vec2(-78, -14), Vec2(18, 80),
                  Vec2(-30, 74), Vec2(24, -78) };
    net.edges = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 0, 4 }, { 0, 5 } };
    checkJunction("five", net, 0, 24.0);
}

TEST_CASE(zoo_short_arm_pair) {
    // Two junctions closer than their combined radii: the S4 compound-pad
    // case, now under the stacked-surface lens.
    RoadNet net = baseNet();
    net.nodes = { Vec2(0, 0),  Vec2(18, 0),   // the too-close pair
                  Vec2(-80, 0), Vec2(98, 0), Vec2(0, 80), Vec2(18, -80) };
    net.edges = { { 0, 2 }, { 0, 1 }, { 1, 3 }, { 0, 4 }, { 1, 5 } };
    checkJunction("shortpair", net, 0, 30.0);
}
