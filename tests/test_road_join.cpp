// INTEGRATION tests for the swept-lattice street mesher: real road graphs meshed
// by buildRoadNetLattice, validated as a DRIVING SURFACE. Isolation tests on
// synthetic arms passed while the real integration was soup — the mouths a body
// sweeps and the mouths a patch consumes did not actually coincide. These build
// small graphs (various angles, unequal widths, on hills) and assert the joins
// are AIRTIGHT: no degenerate faces, and a car drives every arm through every
// junction with no hole (a hole == a gap between a body and a patch).
#include "test_framework.h"
#include "drive_probe.h"

#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/road_mesh.h"
#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>

using namespace engine;

namespace {

using Ground = std::function<Real(Real, Real)>;

int degenerateFaces(const RenderMesh& m) {
    int d = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        if (cross(b - a, c - a).length() < 1e-9) ++d;
    }
    return d;
}

// Drive node a -> mid -> c across the mesh, following the draped surface.
void driveArm(const RenderMesh& m, const RoadGraph& g, int a, int mid, int c,
              const Ground& h, driveprobe::Report& rep) {
    std::vector<Vec3> path;
    auto seg = [&](Vec2 p0, Vec2 p1) {
        for (double t = 0; t <= 1.0001; t += 0.08) {
            const Vec2 q = p0 + (p1 - p0) * t;
            path.push_back(Vec3(q.x, h ? h(q.x, q.y) : 0.0, q.y));
        }
    };
    seg(g.nodes[a].pos, g.nodes[mid].pos);
    seg(g.nodes[mid].pos, g.nodes[c].pos);
    driveprobe::drivePath(m, path, rep);
}

// A cross: centre 0, arms 1..4 at the given offsets (lets us skew the angles).
RoadGraph cross(Vec2 e1, Vec2 e2, Vec2 e3, Vec2 e4, Real w = 8) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0), 0, false}, {e1, 0, false}, {e2, 0, false},
                {e3, 0, false}, {e4, 0, false} };
    g.edges = { {0, 1, w}, {0, 2, w}, {0, 3, w}, {0, 4, w} };
    return g;
}

const Ground kFlat = [](Real, Real) { return 0.0; };
const Ground kHills = [](Real x, Real z) {
    return 1.5f * std::sin(x * 0.02f) + 1.2f * std::cos(z * 0.017f);
};

}  // namespace

// A right-angle 4-way on FLAT ground: the simplest real join. Drive both through
// routes; the pad must connect to all four bodies with no hole and no degenerate.
TEST_CASE(join_4way_square_flat_is_airtight) {
    RoadGraph g = cross(Vec2(-60, 0), Vec2(60, 0), Vec2(0, -60), Vec2(0, 60));
    RenderMesh m = buildRoadNetLattice(g, kFlat);
    CHECK(!m.vertices.empty());
    CHECK(degenerateFaces(m) == 0);
    driveprobe::Report rep;
    driveArm(m, g, 1, 0, 2, kFlat, rep);       // W -> E
    driveArm(m, g, 3, 0, 4, kFlat, rep);       // S -> N
    rep.print("4way-square-flat");
    CHECK(rep.holes == 0);
    CHECK(rep.blocked == 0);
}

// The same 4-way on ROLLING hills — draped, so every arm meets the pad at a
// slightly different height. The join must still be airtight.
TEST_CASE(join_4way_square_hilly_is_airtight) {
    RoadGraph g = cross(Vec2(-60, 0), Vec2(60, 0), Vec2(0, -60), Vec2(0, 60));
    RenderMesh m = buildRoadNetLattice(g, kHills);
    CHECK(degenerateFaces(m) == 0);
    driveprobe::Report rep;
    driveArm(m, g, 1, 0, 2, kHills, rep);
    driveArm(m, g, 3, 0, 4, kHills, rep);
    rep.print("4way-square-hilly");
    CHECK(rep.holes == 0);
    CHECK(rep.blocked == 0);
}

// A SKEWED 4-way (arms not at right angles) — the generated city is full of
// these. The kerb-corner geometry must hold at non-90-degree angles.
TEST_CASE(join_4way_skewed_is_airtight) {
    RoadGraph g = cross(Vec2(-60, -18), Vec2(60, 12), Vec2(-14, -60), Vec2(20, 60));
    RenderMesh m = buildRoadNetLattice(g, kFlat);
    CHECK(degenerateFaces(m) == 0);
    driveprobe::Report rep;
    driveArm(m, g, 1, 0, 2, kFlat, rep);
    driveArm(m, g, 3, 0, 4, kFlat, rep);
    rep.print("4way-skewed");
    CHECK(rep.holes == 0);
    CHECK(rep.blocked == 0);
}

// A T-junction: through road W-E, branch to N. The dominant generated junction.
TEST_CASE(join_T_flat_is_airtight) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0), 0, false}, {Vec2(-60, 0), 0, false},
                {Vec2(60, 0), 0, false}, {Vec2(0, 60), 0, false} };
    g.edges = { {0, 1, 8}, {0, 2, 8}, {0, 3, 8} };
    RenderMesh m = buildRoadNetLattice(g, kFlat);
    CHECK(degenerateFaces(m) == 0);
    driveprobe::Report rep;
    driveArm(m, g, 1, 0, 2, kFlat, rep);       // through W -> E
    driveArm(m, g, 1, 0, 3, kFlat, rep);       // turn W -> N (up the branch)
    rep.print("T-flat");
    CHECK(rep.holes == 0);
    CHECK(rep.blocked == 0);
}

// A T on hills.
TEST_CASE(join_T_hilly_is_airtight) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0), 0, false}, {Vec2(-60, 0), 0, false},
                {Vec2(60, 0), 0, false}, {Vec2(0, 60), 0, false} };
    g.edges = { {0, 1, 8}, {0, 2, 8}, {0, 3, 8} };
    RenderMesh m = buildRoadNetLattice(g, kHills);
    CHECK(degenerateFaces(m) == 0);
    driveprobe::Report rep;
    driveArm(m, g, 1, 0, 2, kHills, rep);
    driveArm(m, g, 1, 0, 3, kHills, rep);
    rep.print("T-hilly");
    CHECK(rep.holes == 0);
    CHECK(rep.blocked == 0);
}

// MIXED widths: a narrow street tees into a wide arterial (the generated city
// mixes artery=13 with street=7). The two through mouths are wide, the branch is
// narrow — the patch must join mouths of different column counts.
TEST_CASE(join_mixed_width_is_airtight) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0), 0, false}, {Vec2(-60, 0), 0, false},
                {Vec2(60, 0), 0, false}, {Vec2(0, 60), 0, false} };
    g.edges = { {0, 1, 13}, {0, 2, 13}, {0, 3, 7} };   // wide W-E through, narrow N branch
    RenderMesh m = buildRoadNetLattice(g, kFlat);
    CHECK(!m.vertices.empty());
    CHECK(degenerateFaces(m) == 0);
    driveprobe::Report rep;
    driveArm(m, g, 1, 0, 2, kFlat, rep);
    driveArm(m, g, 1, 0, 3, kFlat, rep);
    rep.print("mixed-width");
    CHECK(rep.holes == 0);
    CHECK(rep.blocked == 0);
}

// A CURVED chain (deg-2 intermediate nodes) into a junction — the generated city
// is MOSTLY these (metro_hills: 102 of 120 nodes are deg-2 curve points). The
// swept body of a multi-node curved chain must still present a clean mouth.
TEST_CASE(join_curved_chain_into_junction) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0), 0, false},  {Vec2(-60, 0), 0, false},
                {Vec2(60, 0), 0, false}, {Vec2(0, -60), 0, false},
                {Vec2(28, 28), 0, false}, {Vec2(48, 58), 0, false} };  // 4,5 curve away
    g.edges = { {0, 1, 8}, {0, 2, 8}, {0, 3, 8}, {0, 4, 8}, {4, 5, 8} };  // 0-4-5 curved chain
    RenderMesh m = buildRoadNetLattice(g, kHills);
    CHECK(degenerateFaces(m) == 0);
    driveprobe::Report rep;
    driveArm(m, g, 1, 0, 2, kHills, rep);
    driveArm(m, g, 5, 4, 0, kHills, rep);      // drive the curved chain into the junction
    rep.print("curved-chain");
    CHECK(rep.holes == 0);
    CHECK(rep.blocked == 0);
}

// Two junctions joined by a shared road: drive from the far end of one, through
// BOTH junctions, to the far end of the other — the chain body between them must
// connect to both patches.
TEST_CASE(join_two_junctions_chain_between) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0), 0, false},   {Vec2(80, 0), 0, false},    // 0,1: the two junctions
                {Vec2(0, 50), 0, false},  {Vec2(0, -50), 0, false},   // 2,3: arms on node 0
                {Vec2(80, 50), 0, false}, {Vec2(80, -50), 0, false} }; // 4,5: arms on node 1
    g.edges = { {0, 1, 8}, {0, 2, 8}, {0, 3, 8}, {1, 4, 8}, {1, 5, 8} };
    RenderMesh m = buildRoadNetLattice(g, kHills);
    CHECK(degenerateFaces(m) == 0);
    driveprobe::Report rep;
    driveArm(m, g, 2, 0, 1, kHills, rep);      // arm -> junction0 -> toward junction1
    driveArm(m, g, 0, 1, 4, kHills, rep);      // junction0 -> junction1 -> arm
    rep.print("two-junctions");
    CHECK(rep.holes == 0);
    CHECK(rep.blocked == 0);
}
