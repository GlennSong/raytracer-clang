#include "test_framework.h"

// LUA REACHES EVERY MESH WORD (#191 step 2).
//
// The toolkit that writes triangles is MeshBuilder (engine/mesh_builder.h). A recipe is a Lua script, so a toolkit
// function Lua cannot call is a function no recipe can use. Two things are held here:
//
// 1. THE LIST. Every `static` function declared in mesh_builder.h is on the list below with ONE of
//      - the Lua word that reaches it ("mesh.simplify"), or
//      - "c++ only: <why>", for a function that is on purpose not a word.
//    A function added to the header and not to the list FAILS, so the choice is made when the function is written.
//    A name on the list that is no longer in the header FAILS too, so the list does not rot. Every Lua word named
//    must exist in a fresh VM.
// 2. THE NEW WORDS WORK: weld, simplify, split_non_manifold, fill_holes, remesh, hull, tri, wedge, cube_sphere,
//    cross_cards, ribbon, project_uv, planar_uv, and mesh.quad's own uvs.
//
// WHAT THE LIST DOES NOT CATCH (said, not pretended): that the named word really calls the named function (it checks
// that the word exists, not what is inside it); PolyMesh's functions (procgen/polymesh.h, the `poly` table), which
// want the same list when `poly` folds into `mesh`; and any function declared without `static` or outside this header.

#include "../src/engine/mesh_audit.h"
#include "../src/engine/mesh_builder.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_vm.h"
#include "../src/renderer/renderer.h"

#include <fstream>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <string>

using namespace engine;

namespace {

// function in mesh_builder.h -> the Lua word that reaches it, or "c++ only: why"
const std::map<std::string, std::string> kReach = {
    {"aabb", "c++ only: a frame; a script states one with mesh.box_frame or a scope"},
    {"append", "mesh.merge"},
    {"appendTransformed", "mesh.merge"},
    {"array", "mesh.array"},
    {"arrayAlong", "mesh.array_along"},
    {"autosmooth", "mesh.autosmooth"},
    {"bakeHeightColor", "mesh.bake_height_color"},
    {"bend", "mesh.bend"},
    {"boolean", "mesh.subtract"},
    {"box", "mesh.box"},
    {"capsule", "mesh.capsule"},
    {"centred", "c++ only: a frame; a script states one with mesh.box_frame or a scope"},
    {"chunkByCell", "c++ only: the loader's split of a world mesh into render cells"},
    {"colorBy", "mesh.color_by"},
    {"computeTangents", "mesh.tangents"},
    {"cone", "mesh.cone"},
    {"convexHull", "mesh.hull"},
    {"crossCards", "mesh.cross_cards"},
    {"cubeSphere", "mesh.cube_sphere"},
    {"cutByPlane", "mesh.cut"},
    {"cylinder", "mesh.cylinder"},
    {"deform", "mesh.move_by"},
    {"displaceNoise", "mesh.displace"},
    {"drapeOnMesh", "c++ only: lays road paint on the drawn deck; needs the deck's triangle list"},
    {"dropDegenerate", "c++ only: the primitives end with it; a script gets it from mesh.weld"},
    {"emitAnnulus", "mesh.annulus"},
    {"emitBand", "mesh.band"},
    {"emitBlock", "mesh.block"},
    {"emitBox", "mesh.box_frame"},
    {"emitCylinder", "mesh.column"},
    {"emitFill", "mesh.fill"},
    {"emitLattice", "c++ only: the road mesher's shared-vertex grid"},
    {"emitPlanPrism", "mesh.prism"},
    {"emitPolygon", "mesh.polygon"},
    {"emitQuad", "mesh.quad"},
    {"emitQuadUV", "mesh.quad"},
    {"emitSkirt", "mesh.skirt"},
    {"emitTri", "mesh.tri"},
    {"emitTriUV", "mesh.tri"},
    {"extrude", "mesh.extrude"},
    {"facet", "mesh.facet"},
    {"fillHoles", "mesh.fill_holes"},
    {"flatShade", "mesh.flat_shade"},
    {"generatePlanarUVs", "mesh.planar_uv"},
    {"gridCell", "c++ only: one cell of a terrain or deck grid, by vertex index"},
    {"gridIndices", "c++ only: a terrain or deck grid's indices, by vertex index"},
    {"icosphere", "mesh.icosphere"},
    {"lathe", "mesh.lathe"},
    {"leanNormals", "mesh.lean_normals"},
    {"level", "mesh.fill"},
    {"loft", "mesh.loft"},
    {"merged", "mesh.merge"},
    {"mirrored", "mesh.mirror"},
    {"paint", "mesh.paint"},
    {"patch", "mesh.patch"},
    {"pathArc", "mesh.path.arc"},
    {"pathBezier", "mesh.path.bezier"},
    {"pathCircle", "mesh.path.circle"},
    {"pathHelix", "mesh.path.helix"},
    {"pathLength", "mesh.path.length"},
    {"pathLine", "mesh.path.line"},
    {"pathResample", "mesh.path.resample"},
    {"pathSpline", "mesh.path.spline"},
    {"plane", "mesh.plane"},
    {"profileCircle", "mesh.profile.circle"},
    {"profileEllipse", "mesh.profile.ellipse"},
    {"profileFillet", "mesh.profile.fillet"},
    {"profileOffset", "mesh.profile.offset"},
    {"profilePolygon", "mesh.profile.polygon"},
    {"profileRect", "mesh.profile.rect"},
    {"profileResample", "mesh.profile.resample"},
    {"profileRoundedRect", "mesh.profile.rounded_rect"},
    {"profileStar", "mesh.profile.star"},
    {"profileSuperellipse", "mesh.profile.superellipse"},
    {"projectUV", "mesh.project_uv"},
    {"quadFacing", "c++ only: builds by vertex index; a script emits whole faces with mesh.quad"},
    {"recomputeNormals", "mesh.recompute_normals"},
    {"remesh", "mesh.remesh"},
    {"revolve", "mesh.revolve"},
    {"ribbon", "mesh.ribbon"},
    {"roundedBox", "mesh.rounded_box"},
    {"ruled", "mesh.ruled"},
    {"shape", "c++ only: the level file's shape names; a script calls the primitive itself"},
    {"sheet", "mesh.sheet"},
    {"simplifiedSurface", "mesh.simplify"},
    {"simplify", "mesh.simplify"},
    {"sphere", "mesh.sphere"},
    {"splitNonManifold", "mesh.split_non_manifold"},
    {"superquadric", "mesh.superquadric"},
    {"sweep", "mesh.sweep"},
    {"taper", "mesh.taper"},
    {"torus", "mesh.torus"},
    {"transferNormals", "mesh.transfer_normals"},
    {"transform", "mesh.place"},
    {"triFacing", "c++ only: builds by vertex index; a script emits whole faces with mesh.tri"},
    {"triangulate", "c++ only: indices for a flat outline; a script gets the faces from mesh.polygon or mesh.fill"},
    {"tube", "mesh.tube"},
    {"twist", "mesh.twist"},
    {"vertex", "c++ only: builds by vertex index; a script emits whole faces with mesh.tri"},
    {"warp", "mesh.warp"},
    {"wedge", "mesh.wedge"},
    {"weld", "mesh.weld"},
};

std::set<std::string> staticFunctionsIn(const std::string& path) {
    std::ifstream in(path);
    std::stringstream text;
    text << in.rdbuf();
    const std::string source = text.str();
    std::set<std::string> names;
    const std::regex decl(R"((?:^|\n)\s+static [A-Za-z_:<>0-9, ]+[ &*]+([A-Za-z0-9_]+)\()");
    for (std::sregex_iterator it(source.begin(), source.end(), decl), end; it != end; ++it) names.insert((*it)[1].str());
    return names;
}

std::shared_ptr<RenderMesh> build(ScriptVM& vm, const std::string& code) {
    std::shared_ptr<RenderMesh> out;
    std::string err;
    if (!runProcgenMesh(vm, code, out, &err)) std::printf("    script error: %s\n", err.c_str());
    return out;
}

double number(ScriptVM& vm, const std::string& expr) {
    std::string err;
    if (!vm.doString("test_value = " + expr, &err)) std::printf("    script error: %s\n", err.c_str());
    double v = -1e30;
    vm.getGlobalNumber("test_value", v);
    return v;
}

}  // namespace

TEST_CASE(every_mesh_builder_function_has_a_lua_word_or_a_stated_reason) {
    const std::set<std::string> declared = staticFunctionsIn(std::string(RT_SOURCE_DIR) + "/src/engine/mesh_builder.h");
    CHECK(declared.size() >= 80);   // the header was read (95 names on 2026-11-19)
    for (const std::string& fn : declared) {
        if (kReach.count(fn) == 0) {
            std::printf("    mesh_builder.h declares `%s`, which is not on the list in tests/test_mesh_lua_words.cpp: give it a Lua "
                        "word, or list it as c++ only with the reason\n", fn.c_str());
            CHECK(false);
        }
    }
    ScriptVM vm;
    openProcgenLibrary(vm);
    for (const auto& entry : kReach) {
        if (declared.count(entry.first) == 0) {
            std::printf("    `%s` is on the list but no longer in mesh_builder.h: take it off\n", entry.first.c_str());
            CHECK(false);
        }
        if (entry.second.rfind("c++ only: ", 0) == 0) continue;
        std::string err;
        bool exists = false;
        CHECK(vm.doString("test_exists = type(" + entry.second + ") == 'function'", &err));
        vm.getGlobalBool("test_exists", exists);
        if (!exists) std::printf("    `%s` names the Lua word %s, which does not exist\n", entry.first.c_str(), entry.second.c_str());
        CHECK(exists);
    }
}

TEST_CASE(mesh_weld_joins_what_is_one_vertex_and_keeps_a_hard_edge) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    // a box has 8 corners; drawn with hard edges it has 24 vertices, three normals at each corner
    auto hard = build(vm, "return mesh.weld(mesh.box{1, 1, 1})");
    CHECK(hard && hard->vertices.size() == 24);
    auto soft = build(vm, "return mesh.weld(mesh.box{1, 1, 1}, { position_only = true })");
    CHECK(soft && soft->vertices.size() == 8);
    CHECK(soft && soft->indices.size() == 36);
}

TEST_CASE(mesh_simplify_cuts_the_count_and_keeps_a_closed_clean_surface) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    auto ball = build(vm, "return mesh.icosphere(3)");
    auto less = build(vm, "local m, left = mesh.simplify(mesh.icosphere(3), { ratio = 0.25 }) simplify_left = left return m");
    CHECK(ball && less);
    if (!ball || !less) return;
    const std::size_t before = ball->indices.size() / 3, after = less->indices.size() / 3;
    CHECK(before == 1280);
    CHECK(after <= before / 4 + 2);
    CHECK(after >= before / 8);
    double left = 0;
    CHECK(vm.getGlobalNumber("simplify_left", left));
    CHECK(static_cast<std::size_t>(left) == after);
    CHECK(auditMesh(*less).clean());
    CHECK(openEdges(*less) == 0);
    // the ball is still a ball: every vertex within 6% of the unit radius
    for (const Vertex& v : less->vertices) CHECK(std::fabs(v.position.length() - 1.0) < 0.06);
    // max_error is in metres: a hair's allowance leaves a flat-faced box alone but for nothing to remove
    auto box = build(vm, "return (mesh.simplify(mesh.box{1, 1, 1}, { tris = 2, max_error = 0.001 }))");
    CHECK(box && box->indices.size() / 3 == 12);
}

TEST_CASE(mesh_fill_holes_closes_a_box_with_no_lid) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    const std::string open =
        "local h = 0.5 "
        "local function q(a, b, c, d, n) return mesh.quad(a, b, c, d, n, { 1, 1, 1 }) end "
        "local open = mesh.merge{ q({-h,-h,-h},{h,-h,-h},{h,-h,h},{-h,-h,h},{0,-1,0}), "
        "  q({-h,-h,h},{h,-h,h},{h,h,h},{-h,h,h},{0,0,1}), q({-h,-h,-h},{h,-h,-h},{h,h,-h},{-h,h,-h},{0,0,-1}), "
        "  q({h,-h,-h},{h,-h,h},{h,h,h},{h,h,-h},{1,0,0}), q({-h,-h,-h},{-h,-h,h},{-h,h,h},{-h,h,-h},{-1,0,0}) } ";
    auto lidless = build(vm, open + "return open");
    CHECK(lidless && openEdges(*lidless) == 4);
    auto closed = build(vm, open + "local m, n = mesh.fill_holes(open, { max_edges = 8 }) holes_filled = n return m");
    CHECK(closed && openEdges(*closed) == 0);
    CHECK(closed && closed->indices.size() / 3 == 14);   // ten, and a fan of four over the lid
    CHECK(closed && auditMesh(*closed).clean());
    double filled = 0;
    CHECK(vm.getGlobalNumber("holes_filled", filled));
    CHECK(filled == 1.0);
    // a hole longer than max_edges is left alone
    auto left = build(vm, open + "return (mesh.fill_holes(open, { max_edges = 3 }))");
    CHECK(left && openEdges(*left) == 4);
}

TEST_CASE(mesh_remesh_evens_the_triangles_and_stays_closed) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    auto m = build(vm, "return mesh.remesh(mesh.icosphere(2), { edge = 0.2 })");
    CHECK(m != nullptr);
    if (!m) return;
    CHECK(auditMesh(*m).clean());
    CHECK(openEdges(*m) == 0);
    CHECK(m->indices.size() / 3 > 320);   // a unit ball at 0.2 m edges wants more triangles than the 320 it came with
    // The edges gather round the 0.2 m asked for. (Each round splits every edge over 4/3 of the target and then flips
    // and slides vertices, so a few edges end a little over 4/3; what is held is the mean and a loose ceiling.)
    double longest = 0, sum = 0;
    long edges = 0;
    for (std::size_t t = 0; t + 2 < m->indices.size(); t += 3)
        for (int e = 0; e < 3; ++e) {
            const Vec3 a = m->vertices[m->indices[t + e]].position, b = m->vertices[m->indices[t + (e + 1) % 3]].position;
            const double length = (b - a).length();
            longest = std::max(longest, length);
            sum += length;
            ++edges;
        }
    const double mean = sum / static_cast<double>(std::max(edges, 1L));
    std::printf("    remesh to 0.2 m: mean edge %.3f, longest %.3f, %zu triangles\n", mean, longest, m->indices.size() / 3);
    CHECK(mean > 0.16 && mean < 0.24);
    CHECK(longest < 0.2 * 1.6);
    std::string err;
    std::shared_ptr<RenderMesh> none;
    CHECK(!runProcgenMesh(vm, "return mesh.remesh(mesh.icosphere(1))", none, &err));   // the edge length must be given
}

TEST_CASE(mesh_hull_wraps_points_and_meshes_facing_out) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    // a cube's eight corners and a point inside it: twelve triangles, the inner point left out
    auto cube = build(vm, "return mesh.hull({ {0,0,0},{1,0,0},{1,0,1},{0,0,1},{0,1,0},{1,1,0},{1,1,1},{0,1,1},{0.5,0.5,0.5} })");
    CHECK(cube && cube->indices.size() / 3 == 12);
    CHECK(cube && auditMesh(*cube).clean());
    CHECK(cube && openEdges(*cube) == 0);
    // every face looks away from the middle
    if (cube)
        for (std::size_t t = 0; t + 2 < cube->indices.size(); t += 3) {
            const Vertex& a = cube->vertices[cube->indices[t]];
            CHECK(dot(a.normal, a.position - Vec3(0.5, 0.5, 0.5)) > 0.0);
        }
    auto ofMesh = build(vm, "return mesh.hull(mesh.torus(1.0, 0.3))");
    CHECK(ofMesh && ofMesh->indices.size() > 0 && openEdges(*ofMesh) == 0);
    auto flat = build(vm, "return mesh.hull({ {0,0,0},{1,0,0},{1,0,1},{0,0,1} })");   // all in one plane: no solid
    CHECK(flat && flat->indices.empty());
}

TEST_CASE(mesh_raw_faces_and_small_primitives_reach_lua) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    auto tri = build(vm, "return mesh.tri({0,0,0}, {1,0,0}, {0,0,1}, {0,1,0}, {1,0,0})");
    CHECK(tri && tri->indices.size() == 3 && auditMesh(*tri).clean());
    auto triUV = build(vm, "return mesh.tri({0,0,0}, {1,0,0}, {0,0,1}, {0,1,0}, {1,1,1}, { {0.25, 0.5}, {1, 0}, {0, 1} })");
    CHECK(triUV && triUV->vertices.size() == 3);
    if (triUV) {
        bool found = false;
        for (const Vertex& v : triUV->vertices) found = found || (std::fabs(v.u - 0.25f) < 1e-6f && std::fabs(v.v - 0.5f) < 1e-6f);
        CHECK(found);
    }
    auto quadUV = build(vm, "return mesh.quad({0,0,0}, {2,0,0}, {2,0,3}, {0,0,3}, {0,1,0}, {1,1,1}, { {0,0}, {2,0}, {2,3}, {0,3} })");
    CHECK(quadUV && quadUV->indices.size() == 6);
    if (quadUV) {
        float top = 0;
        for (const Vertex& v : quadUV->vertices) top = std::max(top, v.v);
        CHECK_APPROX(top, 3.0, 1e-6);
    }
    auto wedge = build(vm, "return mesh.wedge{2, 1, 3}");
    CHECK(wedge && !wedge->indices.empty() && auditMesh(*wedge).clean() && openEdges(*wedge) == 0);
    auto ball = build(vm, "return mesh.cube_sphere(2.0, 6)");
    CHECK(ball && ball->indices.size() / 3 == 6 * 6 * 6 * 2 && openEdges(*ball) == 0);
    if (ball) for (const Vertex& v : ball->vertices) CHECK_APPROX(v.position.length(), 2.0, 1e-4);
    auto cards = build(vm, "return mesh.cross_cards(1.0, 2.0, 3)");
    CHECK(cards && cards->indices.size() / 3 == 6);
    auto ribbon = build(vm, "return mesh.ribbon({ {0,0,0}, {0,0,1}, {0,0,2} }, { 0.5, 0.5, 0.25 })");
    CHECK(ribbon && ribbon->indices.size() / 3 == 4);
    // flat uvs: a 2 m plate tiled at 0.5 m runs u from -2 to 2
    auto projected = build(vm, "return mesh.project_uv(mesh.plane(2, 2), {0,0,0}, {2,0,0}, {0,0,2})");
    CHECK(projected != nullptr);
    if (projected) {
        float lo = 1e9f, hi = -1e9f;
        for (const Vertex& v : projected->vertices) { lo = std::min(lo, v.u); hi = std::max(hi, v.u); }
        CHECK_APPROX(lo, -2.0, 1e-5);
        CHECK_APPROX(hi, 2.0, 1e-5);
    }
    auto planar = build(vm, "return mesh.planar_uv(mesh.plane(2, 2), 'y', 0.5)");
    CHECK(planar != nullptr);
    if (planar) {
        float hi = -1e9f;
        for (const Vertex& v : planar->vertices) hi = std::max(hi, v.u);
        CHECK_APPROX(hi, 0.5, 1e-5);
    }
    CHECK(number(vm, "select(2, mesh.split_non_manifold(mesh.box{1, 1, 1}))") == 0.0);   // a box has no pinched vertex
}

// --- BOOLEANS (#191 step 3): MeshBuilder::boolean over the Manifold library, as mesh.union / subtract / intersect ---

namespace {

// The volume a closed mesh encloses, in cubic metres: the sum of the signed tetrahedra from the origin to each face.
// The engine winds a front face clockwise seen from outside, so the sum comes out negative for a solid; its size is
// the volume.
double volumeOf(const RenderMesh& m) {
    double six = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        six += dot(a, cross(b, c));
    }
    return std::fabs(six) / 6.0;
}

}  // namespace

TEST_CASE(mesh_booleans_give_the_volumes_arithmetic_says) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    // a 1 m cube with a 0.5 x 0.5 m bar pushed right through it: a quarter of it goes
    auto holed = build(vm, "return (mesh.subtract(mesh.box{1, 1, 1}, mesh.box{0.5, 0.5, 2}))");
    CHECK(holed != nullptr);
    if (holed) {
        CHECK_APPROX(volumeOf(*holed), 0.75, 1e-9);
        CHECK(auditMesh(*holed).clean());
        CHECK(openEdges(*holed) == 0);
    }
    // two 1 m cubes half a metre apart: joined 1.5 m3, the overlap 0.5 m3
    const std::string two = "local a, b = mesh.box{1, 1, 1}, mesh.translate(mesh.box{1, 1, 1}, {0.5, 0, 0}) ";
    auto joined = build(vm, two + "return (mesh.union(a, b))");
    auto overlap = build(vm, two + "return (mesh.intersect(a, b))");
    CHECK(joined && overlap);
    if (joined) { CHECK_APPROX(volumeOf(*joined), 1.5, 1e-9); CHECK(auditMesh(*joined).clean()); CHECK(openEdges(*joined) == 0); }
    if (overlap) { CHECK_APPROX(volumeOf(*overlap), 0.5, 1e-9); CHECK(auditMesh(*overlap).clean()); CHECK(openEdges(*overlap) == 0); }
    // solids that do not touch: nothing overlaps, and joining them keeps both
    const std::string apart = "local a, b = mesh.box{1, 1, 1}, mesh.translate(mesh.box{1, 1, 1}, {5, 0, 0}) ";
    auto nothing = build(vm, apart + "return (mesh.intersect(a, b))");
    CHECK(nothing && nothing->indices.empty());
    auto both = build(vm, apart + "return (mesh.union(a, b))");
    CHECK(both != nullptr);
    if (both) CHECK_APPROX(volumeOf(*both), 2.0, 1e-9);
    // a thin part: a 2 cm panel with a 1 cm slot through it
    auto panel = build(vm, "return (mesh.subtract(mesh.box{1.0, 0.8, 0.02}, mesh.box{0.6, 0.01, 0.2}))");
    CHECK(panel != nullptr);
    if (panel) {
        CHECK_APPROX(volumeOf(*panel), 1.0 * 0.8 * 0.02 - 0.6 * 0.01 * 0.02, 1e-9);
        CHECK(auditMesh(*panel).clean());
        CHECK(openEdges(*panel) == 0);
    }
}

TEST_CASE(mesh_booleans_keep_each_inputs_colour_uvs_and_facing) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    const std::string cut =
        "local wall = mesh.paint(mesh.box{2, 2, 0.3}, {0.8, 0.1, 0.1}) "
        "local hole = mesh.paint(mesh.box{0.5, 0.5, 1}, {0.1, 0.1, 0.8}) "
        "local whole, from_wall, from_hole = mesh.subtract(wall, hole) ";
    auto fromWall = build(vm, cut + "return from_wall");
    auto fromHole = build(vm, cut + "return from_hole");
    CHECK(fromWall && fromHole);
    if (!fromWall || !fromHole) return;
    CHECK(fromHole->indices.size() / 3 == 8);   // the opening's four walls, two triangles each
    for (const Vertex& v : fromWall->vertices) CHECK(v.color.x > 0.7 && v.color.z < 0.2);
    for (const Vertex& v : fromHole->vertices) CHECK(v.color.z > 0.7 && v.color.x < 0.2);
    // the opening's walls face into the opening (toward its middle line, x = y = 0), normals and winding both
    CHECK(auditMesh(*fromHole).clean());
    for (const Vertex& v : fromHole->vertices) CHECK(v.normal.x * v.position.x + v.normal.y * v.position.y < 0.0);
    // and stop at the wall's two faces (to a micron: the box's half thickness, 0.15, is held as a single-precision number)
    for (const Vertex& v : fromHole->vertices) CHECK(std::fabs(std::fabs(v.position.z) - 0.15) < 1e-6);
    // uvs ride along: a cut face's new corners carry uvs between its old corners' (none outside 0..1 on a box)
    for (const Vertex& v : fromWall->vertices) CHECK(v.u >= -1e-6f && v.u <= 1.0f + 1e-6f && v.v >= -1e-6f && v.v <= 1.0f + 1e-6f);
}

TEST_CASE(mesh_booleans_take_every_closed_primitive_and_refuse_an_open_sheet) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    // The round primitives close their seam with a second vertex a hair from the first (mesh_boolean.cpp kSamePlace).
    const char* solids[] = {"mesh.box{0.5, 0.5, 2}", "mesh.cylinder(0.3, 1.6, 32)", "mesh.cone(0.3, 1.6, 16)", "mesh.sphere(0.6)",
                            "mesh.scale(mesh.icosphere(2), {0.6, 0.6, 0.6})", "mesh.torus(0.5, 0.2)", "mesh.capsule(0.3, 1.2)", "mesh.rounded_box{0.6, 0.6, 2}",
                            "mesh.cube_sphere(0.6, 8)", "mesh.wedge{0.5, 0.5, 2}", "mesh.column({0, -1, 0}, 0.3, 2, 16)"};
    for (const char* solid : solids) {
        auto cut = build(vm, std::string("return (mesh.subtract(mesh.box{1, 1, 1}, ") + solid + "))");
        if (!cut || cut->indices.empty() || !auditMesh(*cut).clean() || openEdges(*cut) != 0) std::printf("    failed for %s\n", solid);
        CHECK(cut && !cut->indices.empty());
        if (cut) { CHECK(auditMesh(*cut).clean()); CHECK(openEdges(*cut) == 0); CHECK(volumeOf(*cut) < 1.0); }
    }
    std::shared_ptr<RenderMesh> none;
    std::string err;
    CHECK(!runProcgenMesh(vm, "return (mesh.subtract(mesh.plane(1, 1), mesh.box{1, 1, 1}))", none, &err));
    CHECK(err.find("the first mesh is not a closed solid (4 open edges)") != std::string::npos);
    err.clear();
    CHECK(!runProcgenMesh(vm, "return (mesh.union(mesh.box{1, 1, 1}, mesh.plane(1, 1)))", none, &err));
    CHECK(err.find("the second mesh is not a closed solid") != std::string::npos);
    // the C++ word says the same without a script
    RenderMesh box = MeshBuilder::box(Vec3(1, 1, 1)), sheet = MeshBuilder::plane(1, 1);
    const MeshBuilder::BooleanResult refused = MeshBuilder::boolean(box, sheet, MeshBuilder::BooleanOp::Subtract);
    CHECK(!refused.ok());
    CHECK(refused.fromA.indices.empty() && refused.fromB.indices.empty());
}

// --- SHEETS (#191 step 3): a grid of points, the lines between two curves, the fill of four curves ---

namespace {

double areaOf(const RenderMesh& m) {
    double twice = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        twice += cross(m.vertices[m.indices[t + 1]].position - a, m.vertices[m.indices[t + 2]].position - a).length();
    }
    return twice * 0.5;
}

}  // namespace

TEST_CASE(mesh_sheets_have_the_area_and_the_side_they_were_asked_for) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    // two level 2 m lines a metre apart: a 2 m2 floor. Asked to face up it does; asked to face down it does.
    const std::string lines = "local a, b = { {0, 0, 0}, {2, 0, 0} }, { {0, 0, -1}, {2, 0, -1} } ";
    auto up = build(vm, lines + "return mesh.ruled(a, b, { steps = 4, count = 5, toward = {0, 1, 0} })");
    auto down = build(vm, lines + "return mesh.ruled(a, b, { toward = {0, -1, 0} })");
    CHECK(up && down);
    if (up) {
        CHECK_APPROX(areaOf(*up), 2.0, 1e-9);
        CHECK(up->indices.size() / 3 == 4 * 4 * 2);
        CHECK(auditMesh(*up).clean());
        for (const Vertex& v : up->vertices) CHECK(v.normal.y > 0.99);
    }
    if (down) { CHECK(auditMesh(*down).clean()); for (const Vertex& v : down->vertices) CHECK(v.normal.y < -0.99); }
    // with no side asked for: rows running left to right (+x) and going away (-z) face the viewer above them
    auto own = build(vm, lines + "return mesh.ruled(a, b)");
    CHECK(own != nullptr);
    if (own) for (const Vertex& v : own->vertices) CHECK(v.normal.y > 0.99);
    // a grid given row by row is the same sheet
    auto grid = build(vm, "return mesh.sheet({ { {0,0,0}, {1,0,0}, {2,0,0} }, { {0,0,-1}, {1,0,-1}, {2,0,-1} } })");
    CHECK(grid != nullptr);
    if (grid) { CHECK_APPROX(areaOf(*grid), 2.0, 1e-9); for (const Vertex& v : grid->vertices) CHECK(v.normal.y > 0.99); }
    // rows of different counts are not a grid: nothing is built
    auto ragged = build(vm, "return mesh.sheet({ { {0,0,0}, {1,0,0} }, { {0,0,-1}, {1,0,-1}, {2,0,-1} } })");
    CHECK(ragged && ragged->indices.empty());
}

TEST_CASE(mesh_patch_fills_four_curves_and_passes_through_each) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    // four straight edges of a unit square: the flat square, 1 m2
    auto flat = build(vm, "return mesh.patch({ {0,0,0}, {1,0,0} }, { {0,0,-1}, {1,0,-1} }, { {0,0,0}, {0,0,-1} }, { {1,0,0}, {1,0,-1} }, "
                          "{ across = 4, up = 4, toward = {0, 1, 0} })");
    CHECK(flat != nullptr);
    if (flat) { CHECK_APPROX(areaOf(*flat), 1.0, 1e-9); CHECK(flat->indices.size() / 3 == 32); CHECK(auditMesh(*flat).clean()); }
    // the front edge arched 0.3 m up at its middle, the other three straight: the patch reaches 0.3 m at the middle
    // of its front edge, stays level along the back, and rises nowhere above the arch
    auto arched = build(vm, "local front = {} for i = 0, 16 do local t = i / 16 front[#front + 1] = { t, 0.3 * math.sin(math.pi * t), 0 } end "
                            "return mesh.patch(front, { {0,0,-1}, {1,0,-1} }, { {0,0,0}, {0,0,-1} }, { {1,0,0}, {1,0,-1} }, "
                            "{ across = 16, up = 8, toward = {0, 1, 0} })");
    CHECK(arched != nullptr);
    if (!arched) return;
    CHECK(auditMesh(*arched).clean());
    double frontMiddle = -1, backHighest = -1, highest = -1;
    for (const Vertex& v : arched->vertices) {
        highest = std::max(highest, static_cast<double>(v.position.y));
        if (std::fabs(v.position.z) < 1e-9 && std::fabs(v.position.x - 0.5) < 0.02) frontMiddle = std::max(frontMiddle, static_cast<double>(v.position.y));
        if (std::fabs(v.position.z + 1.0) < 1e-9) backHighest = std::max(backHighest, static_cast<double>(v.position.y));
    }
    CHECK(frontMiddle > 0.29 && frontMiddle < 0.301);
    CHECK(std::fabs(backHighest) < 1e-9);
    CHECK(highest < 0.301);
    CHECK(areaOf(*arched) > 1.0);   // bowed, so more than the flat square
}
