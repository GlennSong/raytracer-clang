#include "test_framework.h"

#include "../src/engine/mesh_audit.h"
#include "../src/engine/mesh_compare.h"
#include "../src/engine/mesh_builder.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_vm.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

using namespace engine;

// THE CURVE AND SURFACE WORDS (#191, mesh_surfaces.cpp). Each surface is judged two ways: the mesh audit (every face
// wound out, none degenerate, unit normals) and its VOLUME by the divergence theorem against the shape's own formula
// -- a surface with a hole, an inside-out face or a missing cap is off by a measurable amount.

namespace {

// The enclosed volume (m^3): the engine's front faces wind clockwise, so the outward normal is cross(c - a, b - a)
double volumeOf(const RenderMesh& m) {
    double v = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3 &a = m.vertices[m.indices[t]].position, &b = m.vertices[m.indices[t + 1]].position,
                   &c = m.vertices[m.indices[t + 2]].position;
        v -= dot(a, cross(b, c)) / 6.0;
    }
    return v;
}

double areaOf(const MeshProfile& p) {
    double a = 0;
    for (std::size_t i = 0, n = p.pts.size(); i < n; ++i) {
        const Vec2 &u = p.pts[i], &w = p.pts[(i + 1) % n];
        a += u.x * w.y - w.x * u.y;
    }
    return 0.5 * a;
}

bool clean(const char* what, const RenderMesh& m) {
    const MeshAudit a = auditMesh(m);
    if (!a.clean())
        std::printf("    [%s] audit: %ld tris, %ld inward, %ld degenerate, %ld non-finite, %ld bad normals\n", what, a.tris,
                    a.inward, a.degenerate, a.nonFinite, a.badNormals);
    return a.clean() && a.tris > 0;
}

void report(const char* what, const RenderMesh& m, double want) {
    std::printf("    [%s] %zu verts, %zu tris, volume %.5f (want %.5f)\n", what, m.vertices.size(), m.indices.size() / 3,
                volumeOf(m), want);
}

}  // namespace

TEST_CASE(mesh_profiles_have_their_shapes_area) {
    CHECK_APPROX(areaOf(MeshBuilder::profileRect(2, 3)), 6.0, 1e-12);
    // a rounded rectangle loses (4 - pi) r^2 at its corners (the arcs' chords cost a little more)
    const MeshProfile rr = MeshBuilder::profileRoundedRect(2, 1, 0.25, 16);
    CHECK(std::fabs(areaOf(rr) - (2.0 - (4.0 - PI) * 0.0625)) < 2e-3);
    CHECK(areaOf(rr) > 0);   // counter-clockwise
    // a fillet of a clockwise ring comes out counter-clockwise too
    MeshProfile cw = MeshBuilder::profileRect(1, 1);
    std::reverse(cw.pts.begin(), cw.pts.end());
    CHECK(areaOf(MeshBuilder::profileFillet(cw, 0.1, 4)) > 0);
    // the offset of a rectangle by d grows each side by 2 d
    CHECK_APPROX(areaOf(MeshBuilder::profileOffset(MeshBuilder::profileRect(2, 1), 0.1)), 2.2 * 1.2, 1e-9);
    CHECK_APPROX(areaOf(MeshBuilder::profileOffset(MeshBuilder::profileRect(2, 1), -0.1)), 1.8 * 0.8, 1e-9);
    CHECK(MeshBuilder::profileOffset(MeshBuilder::profileRect(2, 1), -0.6).pts.empty());   // collapsed
    // a superellipse of n = 2 is the ellipse; larger n fills toward the rectangle
    CHECK(std::fabs(areaOf(MeshBuilder::profileSuperellipse(2, 1, 2, 256)) - PI * 2) < 1e-3);
    const double sq = areaOf(MeshBuilder::profileSuperellipse(1, 1, 8, 256));
    CHECK(sq > 3.6 && sq < 4.0);
    CHECK(MeshBuilder::profileStar(5, 1, 0.4).pts.size() == 10u);
    CHECK(MeshBuilder::profileResample(MeshBuilder::profileRect(2, 2), 16).pts.size() == 16u);
}

TEST_CASE(mesh_paths_run_where_they_say) {
    const MeshPath line = MeshBuilder::pathLine(Vec3(0, 0, 0), Vec3(3, 4, 0), 5);
    CHECK(line.pts.size() == 6u);
    CHECK_APPROX(MeshBuilder::pathLength(line), 5.0, 1e-12);
    const MeshPath ring = MeshBuilder::pathCircle(Vec3(0, 1, 0), 2.0, Vec3(0, 1, 0), 64);
    CHECK(ring.closed && ring.pts.size() == 64u);
    for (const Vec3& p : ring.pts) CHECK(std::fabs((p - Vec3(0, 1, 0)).length() - 2.0) < 1e-9);
    const MeshPath helix = MeshBuilder::pathHelix(Vec3(0, 0, 0), 1.0, 0.5, 3.0, 32);
    CHECK_APPROX(helix.pts.back().y, 1.5, 1e-9);
    // a spline through straight points is the straight line, at even spacing, ends kept
    const MeshPath sp = MeshBuilder::pathSpline({Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(4, 0, 0)}, false, 9);
    CHECK(sp.pts.size() == 9u);
    CHECK_APPROX(MeshBuilder::pathLength(sp), 4.0, 1e-6);
    CHECK((sp.pts.front() - Vec3(0, 0, 0)).length() < 1e-9 && (sp.pts.back() - Vec3(4, 0, 0)).length() < 1e-9);
    const MeshPath bz = MeshBuilder::pathBezier(Vec3(0, 0, 0), Vec3(0, 1, 0), Vec3(1, 1, 0), Vec3(1, 0, 0), 17);
    CHECK(bz.pts.size() == 17u);
    const MeshPath rs = MeshBuilder::pathResample(line, 0.5);
    CHECK(rs.pts.size() == 11u);
}

TEST_CASE(mesh_extrude_is_a_closed_prism_with_soft_edges) {
    const RenderMesh box = MeshBuilder::extrude(MeshBuilder::profileRect(2, 3), 1.0);
    report("extrude rect", box, 6.0);
    CHECK(clean("extrude rect", box));
    CHECK_APPROX(volumeOf(box), 6.0, 1e-9);
    const RenderMesh soft = MeshBuilder::extrude(MeshBuilder::profileRect(2, 3), 1.0, 0.1, 6);
    // the bevel's quarter round takes (1 - pi/4) b^2 along every edge: about 0.0215 m^2 x 10 m of top and bottom rims
    report("extrude rect, bevel 0.1", soft, 6.0 - (1 - PI / 4) * 0.01 * 20);
    CHECK(clean("extrude bevel", soft));
    CHECK(volumeOf(soft) < 6.0 && volumeOf(soft) > 5.9);
    // a star: concave, still closed and wound out
    const RenderMesh star = MeshBuilder::extrude(MeshBuilder::profileStar(5, 1.0, 0.45), 0.2, 0.02, 2);
    CHECK(clean("extrude star", star));
    CHECK(volumeOf(star) > 0.0);
}

TEST_CASE(mesh_sweep_tubes_rings_and_helices_close) {
    // a 12-sided tube 3 m long: its polygon's area times the length, capped
    const MeshProfile c12 = MeshBuilder::profileCircle(0.2, 12);
    const RenderMesh rod = MeshBuilder::sweep(c12, MeshBuilder::pathLine(Vec3(0, 0, 0), Vec3(0, 0, 3), 4));
    report("sweep rod", rod, areaOf(c12) * 3);
    CHECK(clean("sweep rod", rod));
    CHECK_APPROX(volumeOf(rod), areaOf(c12) * 3, 1e-9);
    // a square section round a closed ring of radius 2 (Pappus: area x 2 pi R, a polygonal ring a hair less)
    const MeshProfile sq = MeshBuilder::profileRect(0.3, 0.3);
    const RenderMesh ring = MeshBuilder::sweep(sq, MeshBuilder::pathCircle(Vec3(0, 0, 0), 2.0, Vec3(0, 1, 0), 96));
    report("sweep ring", ring, areaOf(sq) * TAU * 2.0);
    CHECK(clean("sweep ring", ring));
    CHECK(std::fabs(volumeOf(ring) / (areaOf(sq) * TAU * 2.0) - 1.0) < 0.01);
    // a square section keeps its edges: each side its own normals (24 per ring of 4 sides would be smooth: 4 + seam)
    // a helix: no spiral, no flip, volume the section times its length
    const MeshPath helix = MeshBuilder::pathHelix(Vec3(0, 0, 0), 1.0, 0.6, 2.5, 48);
    const RenderMesh coil = MeshBuilder::sweep(c12, helix);
    report("sweep helix", coil, areaOf(c12) * MeshBuilder::pathLength(helix));
    CHECK(clean("sweep helix", coil));
    CHECK(std::fabs(volumeOf(coil) / (areaOf(c12) * MeshBuilder::pathLength(helix)) - 1.0) < 0.02);
    // tapered and twisted: still closed and wound out
    SweepStyle st;
    st.scaleAt = [](double t) { return 1.0 - 0.7 * t; };
    st.twistAt = [](double t) { return 2.0 * t; };
    const RenderMesh horn = MeshBuilder::sweep(MeshBuilder::profileStar(6, 0.3, 0.15),
                                               MeshBuilder::pathSpline({Vec3(0, 0, 0), Vec3(0.5, 1, 0), Vec3(0, 2, 0.5)}, false, 40), st);
    CHECK(clean("sweep horn", horn));
    CHECK(volumeOf(horn) > 0.0);
}

TEST_CASE(mesh_revolve_makes_spheres_and_part_turns) {
    // a half circle of 32 points from the south pole to the north, turned: the sphere (a polygon's a hair under)
    MeshProfile half;
    half.closed = false;
    for (int k = 0; k <= 32; ++k) {
        const double a = -PI * 0.5 + PI * k / 32;
        half.pts.push_back(Vec2(std::cos(a), std::sin(a)));
    }
    half.pts.front().x = 0;
    half.pts.back().x = 0;
    const RenderMesh ball = MeshBuilder::revolve(half, 64);
    report("revolve sphere", ball, 4.0 / 3.0 * PI);
    CHECK(clean("revolve sphere", ball));
    CHECK(std::fabs(volumeOf(ball) / (4.0 / 3.0 * PI) - 1.0) < 0.01);
    // the poles' normals point along the axis
    for (const Vertex& v : ball.vertices)
        if (v.position.x * v.position.x + v.position.z * v.position.z < 1e-12) CHECK(std::fabs(std::fabs(v.normal.y) - 1.0) < 1e-6);
    // a quarter turn of a closed rectangle (r 1..2, y 0..1): a quarter of the annulus' volume, its cut faces closed
    MeshProfile rect = {{Vec2(1, 0), Vec2(2, 0), Vec2(2, 1), Vec2(1, 1)}, true};
    const RenderMesh quarter = MeshBuilder::revolve(rect, 48, 0.0, PI * 0.5);
    const double want = 0.25 * PI * (4.0 - 1.0);
    report("revolve quarter", quarter, want);
    CHECK(clean("revolve quarter", quarter));
    CHECK(std::fabs(volumeOf(quarter) / want - 1.0) < 0.01);
}

TEST_CASE(mesh_loft_and_superquadric_close) {
    // a frustum from a 2 x 2 square to a 1 x 1 one, 1 m up: h (A1 + A2 + sqrt(A1 A2)) / 3
    auto square = [](double s, double y) {
        return std::vector<Vec3>{Vec3(-s / 2, y, -s / 2), Vec3(s / 2, y, -s / 2), Vec3(s / 2, y, s / 2), Vec3(-s / 2, y, s / 2)};
    };
    const RenderMesh fr = MeshBuilder::loft({square(2, 0), square(1, 1)});
    report("loft frustum", fr, (4.0 + 1.0 + 2.0) / 3.0);
    CHECK(clean("loft frustum", fr));
    CHECK_APPROX(volumeOf(fr), 7.0 / 3.0, 1e-9);
    // a superquadric of exponents 1 is the ellipsoid; small exponents approach the box
    const RenderMesh egg = MeshBuilder::superquadric(Vec3(1, 1.5, 1), 1.0, 1.0, 48, 96);
    report("superquadric 1,1", egg, 4.0 / 3.0 * PI * 1.5);
    CHECK(clean("superquadric 1,1", egg));
    CHECK(std::fabs(volumeOf(egg) / (4.0 / 3.0 * PI * 1.5) - 1.0) < 0.01);
    const RenderMesh brick = MeshBuilder::superquadric(Vec3(1, 1, 1), 0.1, 0.1, 64, 128);
    report("superquadric 0.1", brick, 8.0);
    CHECK(clean("superquadric 0.1", brick));
    CHECK(volumeOf(brick) > 7.0 && volumeOf(brick) < 8.0);
}

TEST_CASE(mesh_curve_words_reach_lua) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> m;
    std::string err;
    const char* code = R"LUA(
        local seat = mesh.extrude(mesh.profile.rounded_rect(0.45, 0.45, 0.06), 0.05, { bevel = 0.012, bevel_segs = 3 })
        local leg = mesh.revolve(mesh.profile.open{ {0, 0}, {0.025, 0}, {0.03, 0.2}, {0.018, 0.4}, {0.022, 0.45}, {0, 0.45} },
                                 { segs = 16 })
        local rail = mesh.sweep(mesh.profile.circle(0.012, 10),
                                mesh.path.spline({ {-0.2, 0.8, 0}, {0, 0.9, -0.05}, {0.2, 0.8, 0} }, { count = 24 }),
                                { scale = { 1, 0.6 }, twist = function(t) return t * 0.5 end })
        local pebble = mesh.superquadric({ 0.1, 0.05, 0.08 }, 0.6, 0.8)
        local hull = mesh.loft({ mesh.section(mesh.profile.circle(0.3, 16), { origin = {0, 0, 0} }),
                                 mesh.section(mesh.profile.circle(0.5, 16), { origin = {0, 0, 1} }),
                                 mesh.section(mesh.profile.circle(0.2, 16), { origin = {0, 0, 2} }) })
        assert(mesh.path.length(mesh.path.line({0, 0, 0}, {0, 3, 4})) == 5)
        local hook = mesh.bend(mesh.sweep(mesh.profile.circle(0.02, 8), mesh.path.line({0, 0, 0}, {0, 0.5, 0}, 24)), math.pi, 0.4)
        local column = mesh.twist(mesh.taper(mesh.extrude(mesh.profile.star(6, 0.1, 0.07), 1.0, { bevel = 0.005, bevel_segs = 2 }), 0.7, 1.0), 2.0)
        local posts = mesh.array_along(mesh.extrude(mesh.profile.rect(0.05, 0.05), 0.8), mesh.path.circle({0, 0, 0}, 3, {0, 1, 0}, 48), 16)
        local steps = mesh.array(mesh.extrude(mesh.profile.rect(0.8, 0.25), 0.04), 12, { offset = {0, 0.18, 0}, rotate_y = 0.3 })
        local flipped = mesh.mirror(hook, {1, 0, 0}, 0)
        local warped = mesh.warp(mesh.superquadric({0.2, 0.2, 0.2}, 1, 1), function(p) return { p[1], p[2] * (1 + 0.5 * p[1]), p[3] } end)
        return mesh.merge{ mesh.translate(seat, {0, 0.45, 0}), leg, rail, pebble, hull, hook, column, posts, steps, flipped, warped }
    )LUA";
    const bool ok = runProcgenMesh(vm, code, m, &err);
    if (!ok) std::printf("    lua: %s\n", err.c_str());
    CHECK(ok);
    if (!m) return;
    CHECK(clean("lua chair parts", *m));
    std::printf("    [lua] %zu verts, %zu tris\n", m->vertices.size(), m->indices.size() / 3);
}

TEST_CASE(mesh_modifiers_bend_twist_taper_mirror_array) {
    // a 1 x 1 bar 2 m tall standing on y = 0 (an extruded rect lies in plan: its y is up)
    const RenderMesh bar = MeshBuilder::extrude(MeshBuilder::profileRect(0.2, 0.2), 2.0);
    const double v0 = volumeOf(bar);
    // TAPER to half at the top: a frustum of a square, h (A1 + A2 + sqrt(A1 A2)) / 3 -- exact on the flat faces
    // (the bar's two rings: the taper is linear between them)
    RenderMesh tp = bar;
    MeshBuilder::taper(tp, 0.5, 2.0);
    const double want = 2.0 * (0.04 + 0.01 + 0.02) / 3.0;
    report("taper", tp, want);
    CHECK(clean("taper", tp));
    CHECK_APPROX(volumeOf(tp), want, 1e-9);
    // TWIST a square column finely divided along its height: the section turns but keeps its area -> its volume (a
    // twisted face is not flat: each quad's diagonal bulges a little, an error that halves with the ring spacing --
    // 1.9 % at 40 rings over 2 m, under 0.5 % at 400)
    RenderMesh col = MeshBuilder::sweep(MeshBuilder::profileRect(0.3, 0.3), MeshBuilder::pathLine(Vec3(0, 0, 0), Vec3(0, 2, 0), 400));
    const double vc = volumeOf(col);
    MeshBuilder::twist(col, 1.2);
    report("twist", col, vc);
    CHECK(clean("twist", col));
    CHECK(std::fabs(volumeOf(col) / vc - 1.0) < 0.005);
    // BEND a finely divided bar a quarter turn: still closed and wound out, its volume nearly kept
    RenderMesh rod = MeshBuilder::sweep(MeshBuilder::profileRect(0.1, 0.1), MeshBuilder::pathLine(Vec3(0, 0, 0), Vec3(0, 2, 0), 64));
    const double vr = volumeOf(rod);
    MeshBuilder::bend(rod, PI * 0.5, 2.0);
    report("bend", rod, vr);
    CHECK(clean("bend", rod));
    CHECK(std::fabs(volumeOf(rod) / vr - 1.0) < 0.01);
    double maxX = -1e9;
    for (const Vertex& v : rod.vertices) maxX = std::max(maxX, v.position.x);
    CHECK(maxX > 1.2);   // (a quarter turn of radius 4/pi: it ends ~1.27 m over toward +x)
    // MIRROR: the same volume, wound out
    const RenderMesh mm = MeshBuilder::mirrored(tp, Vec3(1, 0, 0), 0.5);
    CHECK(clean("mirror", mm));
    CHECK_APPROX(volumeOf(mm), want, 1e-9);
    // ARRAY: five copies a metre apart -- five times the volume; along a circle, twelve, each turned to it
    const RenderMesh row = MeshBuilder::array(bar, 5, Mat4::translate(1, 0, 0));
    CHECK_APPROX(volumeOf(row), 5 * v0, 1e-9);
    CHECK(clean("array", row));
    const RenderMesh ring = MeshBuilder::arrayAlong(MeshBuilder::extrude(MeshBuilder::profileRect(0.1, 0.3), 0.1),
                                                    MeshBuilder::pathCircle(Vec3(0, 0, 0), 2.0, Vec3(0, 1, 0), 64), 12);
    CHECK(clean("array along", ring));
    CHECK(ring.indices.size() == 12 * MeshBuilder::extrude(MeshBuilder::profileRect(0.1, 0.3), 0.1).indices.size());
}

TEST_CASE(geometry_equal_ignores_order_and_catches_a_change) {
    // the same box built twice: once as it comes, once with its triangles and vertices shuffled
    RenderMesh a = MeshBuilder::extrude(MeshBuilder::profileRoundedRect(1, 1, 0.2, 4), 0.5, 0.05, 2);
    RenderMesh b = a;
    std::reverse(b.vertices.begin(), b.vertices.end());
    const uint32_t n = static_cast<uint32_t>(b.vertices.size());
    for (uint32_t& ix : b.indices) ix = n - 1 - ix;
    for (std::size_t t = 0; t + 5 < b.indices.size(); t += 6) {   // swap neighbouring triangles, rotate one's corners
        std::swap_ranges(b.indices.begin() + static_cast<long>(t), b.indices.begin() + static_cast<long>(t + 3),
                         b.indices.begin() + static_cast<long>(t + 3));
        std::rotate(b.indices.begin() + static_cast<long>(t), b.indices.begin() + static_cast<long>(t + 1),
                    b.indices.begin() + static_cast<long>(t + 3));
    }
    const MeshDiff same = geometryEqual(a, b);
    if (!same.equal) std::printf("    %s\n", same.why.c_str());
    CHECK(same.equal);
    // one triangle wound the other way: not the same surface
    RenderMesh c = a;
    std::swap(c.indices[1], c.indices[2]);
    CHECK(!geometryEqual(a, c).equal);
    // one vertex nudged a millimetre: caught
    RenderMesh d = a;
    d.vertices[3].position.x += 1e-3;
    CHECK(!geometryEqual(a, d).equal);
    // a colour changed: caught
    RenderMesh e = a;
    MeshBuilder::paint(e, Vec3(0.5, 0.5, 0.5));
    CHECK(!geometryEqual(a, e).equal);
}
