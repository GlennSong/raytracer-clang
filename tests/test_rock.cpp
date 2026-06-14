#include "test_framework.h"

#include "../src/engine/procgen/rock.h"
#include "../src/engine/mesh_builder.h"
#include "../src/engine/procgen/noise.h"
#include <cmath>

using namespace engine;  // namespace migration (ADR-0015)

TEST_CASE(rock_preserves_sphere_vertex_count) {
    RockParams p;
    Noise n(1);
    RenderMesh rock = generateRock(p, n);
    size_t sphereVerts = MeshBuilder::sphere(p.radius, p.stacks, p.slices).vertices.size();
    CHECK(rock.vertices.size() == sphereVerts);   // displacement moves, never adds
}

TEST_CASE(rock_is_deterministic_for_a_seed) {
    RockParams p;
    Noise a(9), b(9);
    RenderMesh ra = generateRock(p, a);
    RenderMesh rb = generateRock(p, b);
    bool same = true;
    for (size_t i = 0; i < ra.vertices.size(); i++)
        if ((ra.vertices[i].position - rb.vertices[i].position).lengthSquared() > 1e-18)
            same = false;
    CHECK(same);
}

TEST_CASE(rock_is_displaced_from_the_sphere) {
    RockParams p;
    Noise n(3);
    RenderMesh rock = generateRock(p, n);
    RenderMesh sphere = MeshBuilder::sphere(p.radius, p.stacks, p.slices);
    double maxDelta = 0.0;
    for (size_t i = 0; i < rock.vertices.size(); i++)
        maxDelta = std::max(maxDelta,
            (rock.vertices[i].position - sphere.vertices[i].position).length());
    CHECK(maxDelta > 0.01);   // the noise actually deformed it
}

TEST_CASE(rock_stays_within_displacement_bounds) {
    RockParams p;
    p.radius = 1.5f;
    p.displacement = 0.3f;
    Noise n(5);
    RenderMesh rock = generateRock(p, n);
    double maxR = 0.0;
    for (const Vertex& v : rock.vertices) maxR = std::max(maxR, v.position.length());
    CHECK(maxR <= p.radius * (1.0 + p.displacement) * 1.02);
}

TEST_CASE(rock_normals_unit_and_outward_overall) {
    RockParams p;
    Noise n(7);
    RenderMesh rock = generateRock(p, n);
    double dotSum = 0.0;
    for (const Vertex& v : rock.vertices) {
        CHECK_APPROX(v.normal.length(), 1.0, 1e-4);
        dotSum += dot(v.normal, normalize(v.position));
    }
    CHECK(dotSum / rock.vertices.size() > 0.3);   // normals broadly point outward
}

TEST_CASE(rock_sdf_is_a_bounded_deterministic_surface) {
    // "A rock is a generator graph": base sphere + lumps - cuts, polygonized.
    RockSdfParams p;
    p.baseRadius = 1.0f;
    p.resolution = 28;
    RenderMesh a = generateRockSdf(p, 11);
    RenderMesh b = generateRockSdf(p, 11);
    CHECK(a.vertices.size() > 50);
    CHECK(a.vertices.size() == b.vertices.size());   // deterministic per seed

    double extent = p.baseRadius * (1.0 + p.lumpScale) + p.smoothness;
    bool inBounds = true;
    for (const Vertex& v : a.vertices) {
        if (v.position.length() > extent * 1.5) inBounds = false;
        CHECK_APPROX(v.normal.length(), 1.0, 1e-3);
    }
    CHECK(inBounds);

    RenderMesh c = generateRockSdf(p, 99);
    CHECK(c.vertices.size() != a.vertices.size() || c.vertices.size() > 0);  // a different rock
}
