#include "test_framework.h"

#include "../src/engine/mesh_builder.h"
#include <algorithm>
#include <cstdint>

using namespace engine;  // namespace migration (ADR-0015)

namespace {

Vec3 centroid(const RenderMesh& m) {
    Vec3 sum;
    for (const Vertex& v : m.vertices) sum += v.position;
    return m.vertices.empty() ? sum
                              : sum * (1.0 / static_cast<double>(m.vertices.size()));
}

}  // namespace

TEST_CASE(mesh_append_merges_buffers_and_offsets_indices) {
    RenderMesh a = MeshBuilder::box(Vec3(1, 1, 1));
    RenderMesh b = MeshBuilder::box(Vec3(1, 1, 1));
    const size_t va = a.vertices.size(), ia = a.indices.size();

    MeshBuilder::append(a, b);
    CHECK(a.vertices.size() == 2 * va);
    CHECK(a.indices.size() == 2 * ia);

    // Every index stays in range, and the appended block is offset past a's
    // original vertices (so b's geometry isn't aliased onto a's).
    uint32_t maxIdx = 0;
    for (uint32_t i : a.indices) maxIdx = std::max(maxIdx, i);
    CHECK(maxIdx < a.vertices.size());
    CHECK(a.indices[ia] >= va);
}

TEST_CASE(mesh_transform_translates_positions) {
    RenderMesh m = MeshBuilder::box(Vec3(1, 1, 1));
    Vec3 before = centroid(m);
    MeshBuilder::transform(m, Mat4::translate(5, 2, -3));
    Vec3 after = centroid(m);
    CHECK_APPROX(after.x - before.x, 5.0, 1e-4);
    CHECK_APPROX(after.y - before.y, 2.0, 1e-4);
    CHECK_APPROX(after.z - before.z, -3.0, 1e-4);
}

TEST_CASE(mesh_transform_keeps_normals_unit_under_nonuniform_scale) {
    RenderMesh m = MeshBuilder::box(Vec3(2, 1, 3));
    // Rotation composed with a non-uniform scale — the case that breaks naive
    // normal transforms; the inverse-transpose keeps them unit after normalize.
    MeshBuilder::transform(m, Mat4::rotateZ(0.7) * Mat4::scale(1, 2, 1));
    for (const Vertex& v : m.vertices)
        CHECK_APPROX(v.normal.length(), 1.0, 1e-4);
}

TEST_CASE(mesh_append_transformed_places_geometry) {
    RenderMesh dst;
    RenderMesh box = MeshBuilder::box(Vec3(1, 1, 1));
    MeshBuilder::appendTransformed(dst, box, Mat4::translate(10, 0, 0));
    CHECK(dst.vertices.size() == box.vertices.size());
    CHECK_APPROX(centroid(dst).x, 10.0, 1e-4);
}

TEST_CASE(mesh_merged_concatenates_all_parts) {
    RenderMesh box = MeshBuilder::box(Vec3(1, 1, 1));
    RenderMesh out = MeshBuilder::merged({box, box, box});
    CHECK(out.vertices.size() == 3 * box.vertices.size());
    CHECK(out.indices.size() == 3 * box.indices.size());
}

TEST_CASE(mesh_recompute_normals_are_unit_length) {
    RenderMesh m = MeshBuilder::box(Vec3(2, 2, 2));
    MeshBuilder::recomputeNormals(m);
    for (const Vertex& v : m.vertices)
        CHECK_APPROX(v.normal.length(), 1.0, 1e-4);
}

TEST_CASE(mesh_recompute_normals_face_out_for_engine_winding) {
    // An up-facing quad wound the engine way (box top-face order: v0,v1,v2,v3
    // around the rim). recomputeNormals must yield +Y, matching how the
    // clockwise-front renderer shades it (regression guard for the terrain
    // inverted-normal bug).
    RenderMesh m;
    m.vertices = {
        Vertex(Vec3(-1, 0, -1), Vec3(0, 0, 0)),
        Vertex(Vec3( 1, 0, -1), Vec3(0, 0, 0)),
        Vertex(Vec3( 1, 0,  1), Vec3(0, 0, 0)),
        Vertex(Vec3(-1, 0,  1), Vec3(0, 0, 0)),
    };
    m.indices = {0, 1, 2, 0, 2, 3};
    MeshBuilder::recomputeNormals(m);
    for (const Vertex& v : m.vertices)
        CHECK_APPROX(v.normal.y, 1.0, 1e-4);
}

TEST_CASE(mesh_planar_uvs_project_position) {
    RenderMesh m;
    m.vertices.push_back(Vertex(Vec3(2, 5, 3), Vec3(0, 1, 0)));
    MeshBuilder::generatePlanarUVs(m, /*axis=*/1, /*scale=*/1.0f);
    CHECK_APPROX(m.vertices[0].u, 2.0, 1e-5);   // x
    CHECK_APPROX(m.vertices[0].v, 3.0, 1e-5);   // z
}
