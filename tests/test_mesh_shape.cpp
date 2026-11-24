// HOW WELL MADE A MESH IS (#191): mesh_audit.h's measureShape and checkMirror, the numbers the model sheet prints
// under "triangle shape" and "left against right". auditMesh says a mesh is valid; these say whether its triangles
// are a sensible shape and in order, and whether its two sides match.
//   - a closed box: no thin triangles, no open or over-shared edges;
//   - a long strip: thin and sharp, and its edge lengths reported;
//   - a grid of quads is regular, a fan from one point is not;
//   - a sheet has open edges; three sheets on one line share an edge three ways;
//   - a hard edge's doubled vertices are one point, not a gap;
//   - left against right: a centred box matches, a box with a lump on one side does not, and the box says where.
#include "test_framework.h"

#include "../src/engine/mesh_audit.h"
#include "../src/engine/mesh_builder.h"
#include "../src/renderer/renderer.h"

#include <cmath>
#include <vector>

using namespace engine;

namespace {

// Triangles over points, each point its own vertex per triangle (as a hard-edged mesh has them).
RenderMesh soup(const std::vector<Vec3>& points, const std::vector<int>& corners) {
    RenderMesh m;
    for (int c : corners) {
        Vertex v;
        v.position = points[static_cast<std::size_t>(c)];
        v.normal = Vec3(0, 1, 0);
        m.indices.push_back(static_cast<uint32_t>(m.vertices.size()));
        m.vertices.push_back(v);
    }
    return m;
}

// A flat grid of nx by nz quads, each `dx` by `dz`, cut into triangles.
RenderMesh gridOf(int nx, int nz, double dx, double dz) {
    std::vector<Vec3> p;
    for (int j = 0; j <= nz; ++j) for (int i = 0; i <= nx; ++i) p.push_back(Vec3(i * dx, 0, j * dz));
    std::vector<int> c;
    for (int j = 0; j < nz; ++j)
        for (int i = 0; i < nx; ++i) {
            const int a = j * (nx + 1) + i, b = a + 1, d = a + nx + 1, e = d + 1;
            for (int k : {a, b, e, a, e, d}) c.push_back(k);
        }
    return soup(p, c);
}

}  // namespace

TEST_CASE(a_closed_box_has_no_thin_triangles_and_no_loose_edges) {
    const MeshShape s = measureShape(MeshBuilder::box(Vec3(1, 1, 1)));
    CHECK(s.tris == 12 && s.thin == 0 && s.sharp == 0);
    CHECK(s.open == 0 && s.overShared == 0);
    CHECK(s.points == 8);                                   // each corner once, though the mesh stores it per face
    CHECK(std::fabs(s.shortest - 1.0) < 1e-6 && std::fabs(s.longest - std::sqrt(2.0)) < 1e-6);
    CHECK(s.worst < 3.0);
}

TEST_CASE(a_long_strip_is_thin_and_sharp_and_says_how_long) {
    const MeshShape s = measureShape(gridOf(1, 1, 0.01, 2.0));   // one quad, 1 cm by 2 m
    CHECK(s.tris == 2 && s.thin == 2 && s.sharp == 2);
    CHECK(std::fabs(s.shortest - 0.01) < 1e-9 && s.longest > 2.0 && s.longest < 2.001);
    CHECK(s.worst > 100.0);
    CHECK(s.thinShare() == 1.0 && s.sharpShare() == 1.0);
    const MeshShape square = measureShape(gridOf(1, 1, 1.0, 1.0));
    CHECK(square.thin == 0 && square.sharp == 0);
}

TEST_CASE(a_grid_is_regular_and_a_fan_is_not) {
    const MeshShape grid = measureShape(gridOf(20, 20, 0.1, 0.1));
    CHECK(grid.regularShare() > 0.8);                       // every point inside has 6 edges; only the rim has fewer
    std::vector<Vec3> p{Vec3(0, 0, 0)};
    std::vector<int> c;
    const int n = 40;
    for (int i = 0; i < n; ++i) p.push_back(Vec3(std::cos(i * 6.283185307 / n), 0, std::sin(i * 6.283185307 / n)));
    for (int i = 0; i < n; ++i) for (int k : {0, 1 + i, 1 + (i + 1) % n}) c.push_back(k);
    const MeshShape fan = measureShape(soup(p, c));
    CHECK(fan.tris == n && fan.regular == 0);               // the middle has 40 edges, the rim 3 each
}

TEST_CASE(open_edges_and_edges_shared_three_ways_are_counted) {
    const MeshShape sheet = measureShape(gridOf(2, 2, 1, 1));
    CHECK(sheet.open == 8 && sheet.overShared == 0);
    // three triangles on one edge, like a door jamb meeting the skin and its back
    const std::vector<Vec3> p{Vec3(0, 0, 0), Vec3(0, 0, 1), Vec3(1, 0, 0.5), Vec3(-1, 0, 0.5), Vec3(0, 1, 0.5)};
    const MeshShape fin = measureShape(soup(p, {0, 1, 2, 0, 1, 3, 0, 1, 4}));
    CHECK(fin.overShared == 1 && fin.open == 6);
    // nothing to measure
    const MeshShape none = measureShape(RenderMesh{});
    CHECK(none.tris == 0 && none.thinShare() == 0 && none.regularShare() == 0 && none.shortest == 0);
    // adding up parts: the counts add, the edge range is the widest
    MeshShape sum;
    sum.add(none);
    sum.add(measureShape(gridOf(1, 1, 0.01, 2.0)));
    sum.add(measureShape(MeshBuilder::box(Vec3(1, 1, 1))));
    CHECK(sum.tris == 14 && sum.thin == 2 && std::fabs(sum.shortest - 0.01) < 1e-9 && sum.longest > 2.0);
}

TEST_CASE(left_against_right_finds_what_one_side_has_and_the_other_lacks) {
    const RenderMesh box = MeshBuilder::box(Vec3(2, 1, 4));
    const MirrorCheck same = checkMirror(box);
    CHECK(same.points == 8 && same.missing == 0);
    RenderMesh lumpy = box;                                  // a small box on the right side only
    RenderMesh lump = MeshBuilder::box(Vec3(0.2, 0.2, 0.2));
    const uint32_t base = static_cast<uint32_t>(lumpy.vertices.size());
    for (Vertex v : lump.vertices) { v.position = v.position + Vec3(-1.3, 0.0, 1.0); lumpy.vertices.push_back(v); }
    for (uint32_t i : lump.indices) lumpy.indices.push_back(base + i);
    const MirrorCheck off = checkMirror(lumpy);
    CHECK(off.points == 16 && off.missing == 8);
    CHECK(off.hi.x < -1.1 && off.lo.x > -1.5 && off.lo.z > 0.8 && off.hi.z < 1.2);   // the box round the lump
    // a point 2 mm out of place has no mirror at 1 mm and has one at 5 mm
    RenderMesh nudged = box;
    for (Vertex& v : nudged.vertices) if (v.position.x > 0 && v.position.z > 0 && v.position.y > 0) v.position.x += 0.002;
    CHECK(checkMirror(nudged).missing == 2);                 // the moved corner and the one that was its mirror
    CHECK(checkMirror(nudged, 0.005).missing == 0);
}
