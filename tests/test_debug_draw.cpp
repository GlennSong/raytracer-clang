#include "test_framework.h"
#include "../src/engine/debug_draw.h"

using engine::DebugDraw;
using engine::RenderMesh;
using engine::Vec3;
using engine::Quat;
using engine::Mat4;
using engine::buildDebugLineMesh;

TEST_CASE(debug_draw_shape_line_counts) {
    DebugDraw dd;
    dd.line({0, 0, 0}, {1, 0, 0}, {1, 1, 1});
    CHECK(dd.lineCount() == 1);

    dd.clear();
    dd.aabb({0, 0, 0}, {1, 1, 1}, {1, 1, 1});
    CHECK(dd.lineCount() == 12);

    dd.clear();
    dd.box({0, 0, 0}, {1, 1, 1}, Quat(), {1, 1, 1});
    CHECK(dd.lineCount() == 12);

    dd.clear();
    dd.sphere({0, 0, 0}, 1.0, {1, 1, 1}, 0, 24);
    CHECK(dd.lineCount() == 3 * 24);

    dd.clear();
    dd.circle({0, 0, 0}, {0, 1, 0}, 1.0, {1, 1, 1}, 0, 16);
    CHECK(dd.lineCount() == 16);

    dd.clear();
    dd.axes(Mat4());
    CHECK(dd.lineCount() == 3);

    dd.clear();
    dd.arrow({0, 0, 0}, {0, 0, 5}, {1, 1, 1});
    CHECK(dd.lineCount() == 5);   // shaft + four head fins
}

TEST_CASE(debug_draw_oriented_box_rotates_corners) {
    DebugDraw dd;
    // 90 degrees about Y: the +X half extent should land on -Z.
    Quat rot = Quat::fromAxisAngle(Vec3(0, 1, 0), M_PI / 2.0);
    dd.box({0, 0, 0}, {2, 1, 1}, rot, {1, 1, 1});
    // Every endpoint's |z| should reach the rotated 2-unit half extent.
    double maxAbsZ = 0;
    for (const auto& l : dd.lines()) {
        maxAbsZ = std::max(maxAbsZ, std::fabs(l.a.z));
        maxAbsZ = std::max(maxAbsZ, std::fabs(l.b.z));
    }
    CHECK_APPROX(maxAbsZ, 2.0, 1e-6);
}

TEST_CASE(debug_draw_one_frame_shapes_expire_at_end_frame) {
    DebugDraw dd;
    dd.line({0, 0, 0}, {1, 0, 0}, {1, 1, 1});   // default: this frame only
    CHECK(dd.lineCount() == 1);
    dd.endFrame();
    CHECK(dd.empty());
}

TEST_CASE(debug_draw_timed_shapes_survive_until_aged_out) {
    DebugDraw dd;
    dd.line({0, 0, 0}, {1, 0, 0}, {1, 1, 1}, 0.05);
    dd.update(0.016);
    dd.endFrame();
    CHECK(dd.lineCount() == 1);   // 0.034 s left
    dd.update(0.016);
    dd.endFrame();
    CHECK(dd.lineCount() == 1);   // 0.018 s left
    dd.update(0.05);
    dd.endFrame();
    CHECK(dd.empty());
}

TEST_CASE(debug_line_mesh_one_ribbon_quad_per_line) {
    std::vector<DebugDraw::Line> lines = {
        {{0, 0, 0}, {1, 0, 0}, {1, 0, 0}, 0},
        {{0, 1, 0}, {1, 1, 0}, {0, 1, 0}, 0},
    };
    RenderMesh mesh = buildDebugLineMesh(lines, Vec3(0.5, 0.5, 5.0));
    CHECK(mesh.vertices.size() == 8);
    CHECK(mesh.indices.size() == 12);
}

TEST_CASE(debug_line_mesh_skips_degenerate_lines) {
    std::vector<DebugDraw::Line> lines = {
        {{1, 2, 3}, {1, 2, 3}, {1, 1, 1}, 0},   // zero length
    };
    RenderMesh mesh = buildDebugLineMesh(lines, Vec3(0, 0, 5));
    CHECK(mesh.vertices.empty());
    CHECK(mesh.indices.empty());
}

TEST_CASE(debug_line_mesh_ribbon_faces_the_camera) {
    std::vector<DebugDraw::Line> lines = {
        {{-1, 0, 0}, {1, 0, 0}, {1, 1, 0}, 0},
    };
    Vec3 camera(0, 0, 5);
    RenderMesh mesh = buildDebugLineMesh(lines, camera);
    CHECK(mesh.indices.size() == 6);
    // Engine winding: the front-face geometric normal of (a,b,c) is
    // cross(c-a, b-a); it must point toward the camera.
    Vec3 a = mesh.vertices[mesh.indices[0]].position;
    Vec3 b = mesh.vertices[mesh.indices[1]].position;
    Vec3 c = mesh.vertices[mesh.indices[2]].position;
    Vec3 faceNormal = cross(c - a, b - a);
    Vec3 toCam = camera - (a + b + c) * (1.0 / 3.0);
    CHECK(dot(faceNormal, toCam) > 0);
}

TEST_CASE(debug_line_mesh_bakes_line_color_into_vertices) {
    std::vector<DebugDraw::Line> lines = {
        {{0, 0, 0}, {1, 0, 0}, {0.2, 0.9, 0.4}, 0},
    };
    RenderMesh mesh = buildDebugLineMesh(lines, Vec3(0, 0, 5));
    CHECK(mesh.vertices.size() == 4);
    for (const auto& v : mesh.vertices) {
        CHECK_APPROX(v.color.x, 0.2, 1e-6);
        CHECK_APPROX(v.color.y, 0.9, 1e-6);
        CHECK_APPROX(v.color.z, 0.4, 1e-6);
    }
}

TEST_CASE(debug_line_mesh_width_scales_with_camera_distance) {
    std::vector<DebugDraw::Line> nearLine = {
        {{-1, 0, 0}, {1, 0, 0}, {1, 1, 1}, 0},
    };
    std::vector<DebugDraw::Line> farLine = {
        {{-1, 0, -40}, {1, 0, -40}, {1, 1, 1}, 0},
    };
    Vec3 camera(0, 0, 2);
    auto ribbonWidth = [](const RenderMesh& mesh) {
        return (mesh.vertices[1].position - mesh.vertices[0].position).length();
    };
    RenderMesh nearMesh = buildDebugLineMesh(nearLine, camera);
    RenderMesh farMesh = buildDebugLineMesh(farLine, camera);
    CHECK(ribbonWidth(farMesh) > ribbonWidth(nearMesh) * 5.0);
}

TEST_CASE(debug_line_mesh_handles_line_pointing_at_camera) {
    // Line along the view ray: the cross(dir, toCam) side vector vanishes;
    // the fallback perpendicular must still make a full quad.
    std::vector<DebugDraw::Line> lines = {
        {{0, 0, 0}, {0, 0, 2}, {1, 1, 1}, 0},
    };
    RenderMesh mesh = buildDebugLineMesh(lines, Vec3(0, 0, 5));
    CHECK(mesh.vertices.size() == 4);
    CHECK(mesh.indices.size() == 6);
}
