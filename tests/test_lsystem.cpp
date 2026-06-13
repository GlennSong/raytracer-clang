#include "test_framework.h"

#include "../src/engine/lsystem.h"
#include "../src/engine/mesh_builder.h"
#include <string>

using namespace engine;  // namespace migration (ADR-0015)

TEST_CASE(lsystem_expands_rules) {
    LSystem sys;
    sys.rules['F'] = "FF";
    CHECK(sys.expand("F", 0) == "F");
    CHECK(sys.expand("F", 1) == "FF");
    CHECK(sys.expand("F", 2) == "FFFF");
    CHECK(sys.expand("F", 3) == "FFFFFFFF");
}

TEST_CASE(lsystem_passes_through_symbols_without_rules) {
    LSystem sys;
    sys.rules['F'] = "F+F";
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
    sys.rules['F'] = "F[+F][-F]F";
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
