#include "test_framework.h"

#include "../src/engine/procgen/lsystem.h"
#include "../src/engine/mesh_builder.h"
#include <string>

using namespace engine;  // namespace migration (ADR-0015)

TEST_CASE(lsystem_expands_rules) {
    LSystem sys;
    sys.rule('F', "FF");
    CHECK(sys.expand("F", 0) == "F");
    CHECK(sys.expand("F", 1) == "FF");
    CHECK(sys.expand("F", 2) == "FFFF");
    CHECK(sys.expand("F", 3) == "FFFFFFFF");
}

TEST_CASE(lsystem_passes_through_symbols_without_rules) {
    LSystem sys;
    sys.rule('F', "F+F");
    CHECK(sys.expand("F+", 1) == "F+F+");   // '+' has no rule, copied verbatim
}

TEST_CASE(turtle_mesh_one_segment_per_F) {
    TurtleParams p;
    const size_t cyl = MeshBuilder::cylinder(p.radius, p.length, p.segmentSlices)
                           .vertices.size();
    CHECK(buildTurtleMesh("F", p).vertices.size() == cyl);
    CHECK(buildTurtleMesh("FF", p).vertices.size() == 2 * cyl);
    // A branch draws three segments (trunk, branch, continuation).
    CHECK(buildTurtleMesh("F[+F]F", p).vertices.size() == 3 * cyl);
}

TEST_CASE(turtle_mesh_handles_unbalanced_pop) {
    TurtleParams p;
    // A stray ']' must not crash (pop of an empty stack is a no-op).
    RenderMesh m = buildTurtleMesh("]F", p);
    CHECK(m.vertices.size() > 0);
}

TEST_CASE(generate_tree_is_deterministic) {
    LSystem sys;
    sys.rule('F', "F[+F][-F]F");
    TurtleParams p;
    RenderMesh a = generateTree(sys, "F", 3, p);
    RenderMesh b = generateTree(sys, "F", 3, p);
    CHECK(a.vertices.size() == b.vertices.size());
    CHECK(a.vertices.size() > 0);
    bool same = true;
    for (size_t i = 0; i < a.vertices.size(); i++) {
        const Vec3& pa = a.vertices[i].position;
        const Vec3& pb = b.vertices[i].position;
        if ((pa - pb).lengthSquared() > 1e-18) same = false;
    }
    CHECK(same);
}

TEST_CASE(turtle_segments_one_per_F) {
    TurtleParams p;
    CHECK(turtleSegments("F", p).size() == 1);
    CHECK(turtleSegments("FF", p).size() == 2);
    CHECK(turtleSegments("F[+F]F", p).size() == 3);
    // Each segment spans exactly one length.
    auto segs = turtleSegments("F", p);
    CHECK_APPROX((segs[0].b - segs[0].a).length(), p.length, 1e-5);
}

TEST_CASE(turtle_sdf_skin_is_one_welded_surface) {
    // A two-branch skeleton skinned as capsules: the SDF blends them into a
    // single connected, bounded surface with valid outward-ish normals.
    TurtleParams p;
    p.length = 1.0f;
    p.radius = 0.2f;
    RenderMesh m = buildTurtleMeshSdf("F[&F]", p, /*smoothness=*/0.15, /*resolution=*/28);
    CHECK(m.vertices.size() > 50);
    CHECK(m.indices.size() % 3 == 0);
    uint32_t maxIdx = 0;
    for (uint32_t i : m.indices) maxIdx = std::max(maxIdx, i);
    CHECK(maxIdx < m.vertices.size());
    for (const Vertex& v : m.vertices)
        CHECK_APPROX(v.normal.length(), 1.0, 1e-3);
}

TEST_CASE(generate_tree_sdf_is_deterministic) {
    LSystem sys;
    sys.rule('F', "F[&F][^F]");
    TurtleParams p;
    RenderMesh a = generateTreeSdf(sys, "F", 2, p, 0.1, 24);
    RenderMesh b = generateTreeSdf(sys, "F", 2, p, 0.1, 24);
    CHECK(a.vertices.size() == b.vertices.size());
    CHECK(a.vertices.size() > 0);
}

TEST_CASE(lsystem_stochastic_rules_vary_by_seed) {
    LSystem sys;
    sys.rule('F', "F[+F]");   // two productions for F -> stochastic
    sys.rule('F', "F[-F]");
    std::string a = sys.expand("F", 4, 1);
    std::string b = sys.expand("F", 4, 1);
    std::string c = sys.expand("F", 4, 2);
    CHECK(a == b);            // deterministic per seed
    CHECK(a != c);            // a different seed grows a different tree
}

TEST_CASE(lsystem_single_production_ignores_seed) {
    LSystem sys;
    sys.rule('F', "FF");
    CHECK(sys.expand("F", 3, 1) == sys.expand("F", 3, 99));   // no choice = deterministic
}

TEST_CASE(turtle_leaf_symbol_adds_blob_geometry) {
    TurtleParams p;
    p.leafRadius = 0.5f;
    // 'L' emits a leaf sphere (a==b capsule) without advancing; "FL" = one
    // branch + one leaf blob, more segments than "F" alone.
    CHECK(turtleSegments("F", p).size() == 1);
    CHECK(turtleSegments("FL", p).size() == 2);
    auto segs = turtleSegments("FL", p);
    CHECK((segs[1].a - segs[1].b).lengthSquared() < 1e-12);   // leaf is a sphere
    CHECK_APPROX(segs[1].radius, 0.5, 1e-6);
    // With leafRadius 0, L is skipped.
    p.leafRadius = 0.0f;
    CHECK(turtleSegments("FL", p).size() == 1);
}
