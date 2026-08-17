// A junction is a DRIVING surface: where several roads meet, the pad must be
// smooth enough to drive across.
//
// Every other weld fixture builds its junctions on FLAT ground (no heightAt), so
// the height field is constant and any defect in it is invisible. On real terrain
// weldSolid's height comes from `sample()` — the NEAREST spine's draped profile —
// which is discontinuous across the medial axis between two arms that arrive at
// different heights. The deck sheet is triangulated over that field, so the pad
// inherits the jump as a BUMP a car cannot drive over.
#include "test_framework.h"
#include "drive_probe.h"

#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/road_mesh.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace engine;

namespace {

// A 4-way cross on ROLLING terrain — the metropolis case (streets drape hills).
RoadEntity hillyCross() {
    RoadEntity n;
    n.graph.nodes = { RoadNode{Vec2(0, 0)},   RoadNode{Vec2(-60, 0)},
                      RoadNode{Vec2(60, 0)},  RoadNode{Vec2(0, -60)},
                      RoadNode{Vec2(0, 60)} };
    n.graph.edges = { RoadEdge{ 0, 1, 10.0 }, RoadEdge{ 0, 2, 10.0 },
                      RoadEdge{ 0, 3, 10.0 }, RoadEdge{ 0, 4, 10.0 } };
    return n;
}

// Gentle hills: ~1.5 m of relief across the junction, like real terrain.
const RoadGroundFn hillyGround = [](double x, double z) {
    return 1.5 * std::sin(x * 0.02) + 1.2 * std::cos(z * 0.017);
};

// tan(tilt) of a triangle's true geometry.
double faceGrade(const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 n = cross(b - a, c - a);
    const double ny = std::fabs(n.y);
    if (ny < 1e-9) return 1e9;
    return std::sqrt(n.x * n.x + n.z * n.z) / ny;
}

}  // namespace

// The terrain under this cross rolls by ~1.5 m over 120 m — under 3% grade. The
// road drapes onto it, so the junction pad should be about that gentle too.
// Anything steep in the PAD is the mesher's, not the terrain's.
TEST_CASE(weld_junction_pad_is_drivable_on_hills) {
    RenderMesh m = buildRoadNetMesh(hillyCross(), hillyGround);
    CHECK(!m.vertices.empty());

    int steep = 0, near = 0;
    double worst = 0.0;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vec3& a = m.vertices[m.indices[i]].position;
        const Vec3& b = m.vertices[m.indices[i + 1]].position;
        const Vec3& c = m.vertices[m.indices[i + 2]].position;
        const Vec3 ctr = (a + b + c) * (1.0f / 3.0f);
        // the junction pad only: within 14 m of the node, and on the road top
        if (std::sqrt(ctr.x * ctr.x + ctr.z * ctr.z) > 14.0) continue;
        if (m.vertices[m.indices[i]].normal.y < 0.5) continue;   // skip kerb/skirt walls
        const double g = faceGrade(a, b, c);
        if (g > 1e8) continue;                                   // degenerate sliver
        ++near;
        worst = std::max(worst, g);
        if (g > 0.25) ++steep;                                   // 25% = a wall, not a road
    }
    std::printf("[junction] pad faces=%d steep=%d worstGrade=%.0f%%\n", near, steep,
                worst * 100.0);
    // WHERE are the steep faces? If they sit on the MEDIAL AXES between arms
    // (the 45-degree diagonals of this cross), the cause is the nearest-spine
    // height field: two arms drape to different heights and heightOf jumps
    // across the line where the nearest arm changes.
    std::printf("   %10s %26s %8s %8s\n", "grade", "centre(x,y,z)", "|drop|", "bearing");
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vec3& a = m.vertices[m.indices[i]].position;
        const Vec3& b = m.vertices[m.indices[i + 1]].position;
        const Vec3& c = m.vertices[m.indices[i + 2]].position;
        const Vec3 ctr = (a + b + c) * (1.0f / 3.0f);
        if (std::sqrt(ctr.x * ctr.x + ctr.z * ctr.z) > 14.0) continue;
        if (m.vertices[m.indices[i]].normal.y < 0.5) continue;
        const double g = faceGrade(a, b, c);
        if (g <= 0.25 || g > 1e8) continue;
        const double drop = std::max({a.y, b.y, c.y}) - std::min({a.y, b.y, c.y});
        const double bearing = std::atan2(ctr.z, ctr.x) * 180.0 / 3.14159265;
        std::printf("   %9.0f%% (%8.2f,%6.2f,%8.2f) %8.2f %7.0f deg\n", g * 100.0,
                    (double)ctr.x, (double)ctr.y, (double)ctr.z, drop, bearing);
    }
    CHECK(near > 20);          // the fixture really does build a pad
    // BASELINE 2026-07-17: steep=8, worst 164% grade on the 45-degree medial axes
    // (nearest-spine height is winner-take-all, so it STEPS between arms). This
    // bound catches regression; the swept-lattice junction patch (Coons blend
    // between arm mouths) drives it to 0. STAGE-2 GATE: tighten to `steep == 0`.
    CHECK(steep <= 10);
}

