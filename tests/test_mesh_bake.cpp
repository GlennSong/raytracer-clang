// THE BAKER (engine/mesh_bake.h): a detailed surface found from a cheap one's atlas and its normals, colour and
// occlusion baked -- each against an answer worked out by hand.
#include "test_framework.h"

#include "../src/engine/mesh_bake.h"
#include "../src/engine/mesh_builder.h"
#include "../src/engine/mesh_unwrap.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_vm.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace engine;

namespace {
// a flat grid in x-z (y up), `n` cells a side, `h(x, z)` its height, normals from the faces
RenderMesh grid(int n, double size, double (*h)(double, double)) {
    RenderMesh m;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) {
            const double x = (i / static_cast<double>(n) - 0.5) * size, z = (j / static_cast<double>(n) - 0.5) * size;
            m.vertices.push_back(Vertex(Vec3(x, h(x, z), z), Vec3(0, 1, 0)));
        }
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const uint32_t a = static_cast<uint32_t>(j * (n + 1) + i), b = a + 1, c = a + static_cast<uint32_t>(n + 1), d = c + 1;
            m.indices.insert(m.indices.end(), {a, c, b, b, c, d});
        }
    MeshBuilder::recomputeNormals(m);
    if (m.vertices[0].normal.y < 0) {   // (wound the other way: turned to face up)
        for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) std::swap(m.indices[i + 1], m.indices[i + 2]);
        MeshBuilder::recomputeNormals(m);
    }
    return m;
}
double flat(double, double) { return 0; }
double bump(double x, double z) { return 0.05 * std::sin(x * 6) * std::cos(z * 6); }
}  // namespace

TEST_CASE(bake_a_smooth_sphere_onto_a_coarse_one_finds_every_texel_and_its_radial_normal) {
    RenderMesh low = MeshBuilder::icosphere(1);
    MeshBuilder::flatShade(low);
    const UnwrapReport r = unwrapMesh(low, UnwrapOptions{50, 0.92, 64, 2, 1024});
    const RenderMesh high = MeshBuilder::sphere(1.0f, 96, 192);
    BakeOptions o;
    o.size = r.size;
    o.cage = 0.12;
    const BakedMaps m = bakeMaps(low, high, o);
    double worst = 1;
    int checked = 0, frontZ = 0;
    for (std::size_t i = 0; i < m.covered.size(); ++i) {
        if (m.covered[i] != 1 || m.world[i].lengthSquared() == 0) continue;
        const double c = dot(m.world[i], normalize(m.position[i]));
        worst = std::min(worst, c);
        ++checked;
        if (m.normal[i].z > 0.9) ++frontZ;
    }
    std::printf("    sphere: %d px atlas, %d chart texels, %d found, worst cos(baked, radial) %.4f, %d of %d near-flat in tangent space\n",
                r.size, m.texels, m.hits, worst, frontZ, checked);
    CHECK(m.texels > 1000);
    CHECK(m.hits == m.texels);
    CHECK(worst > 0.995);
    CHECK(frontZ > checked * 0.9);
}

TEST_CASE(bake_a_bumped_surface_onto_a_flat_one_matches_the_bump) {
    RenderMesh low = grid(1, 1.0, flat);
    const UnwrapReport r = unwrapMesh(low, UnwrapOptions{50, 0.92, 256, 2, 1024});
    const RenderMesh high = grid(160, 1.0, bump);
    BakeOptions o;
    o.size = r.size;
    o.cage = 0.1;
    const BakedMaps m = bakeMaps(low, high, o);
    double worst = 1;
    int n = 0;
    for (std::size_t i = 0; i < m.covered.size(); ++i) {
        if (m.covered[i] != 1 || m.world[i].lengthSquared() == 0) continue;
        const Vec3& p = m.position[i];
        // the bump's analytic normal: (-dh/dx, 1, -dh/dz)
        const Vec3 want = normalize(Vec3(-0.05 * 6 * std::cos(p.x * 6) * std::cos(p.z * 6), 1, 0.05 * 6 * std::sin(p.x * 6) * std::sin(p.z * 6)));
        worst = std::min(worst, dot(m.world[i], want));
        ++n;
    }
    std::printf("    bump: %d texels, %d found, worst cos(baked, analytic) %.4f\n", m.texels, m.hits, worst);
    CHECK(n > 10000);
    CHECK(m.hits == m.texels);
    CHECK(worst > 0.995);
}

TEST_CASE(bake_colour_and_occlusion) {
    // colour: the detailed floor painted red to green across x
    RenderMesh low = grid(1, 2.0, flat);
    const UnwrapReport r = unwrapMesh(low, UnwrapOptions{50, 0.92, 64, 2, 1024});
    RenderMesh high = grid(40, 2.0, flat);
    for (Vertex& v : high.vertices) v.color = Vec3(0.5 - v.position.x * 0.5, 0.5 + v.position.x * 0.5, 0);
    // occlusion: a wall standing along the floor's -x edge
    RenderMesh wall = MeshBuilder::box(Vec3(0.1, 1.0, 2.0));
    MeshBuilder::transform(wall, Mat4::translate(-1.05, 0.5, 0));
    MeshBuilder::append(high, wall);
    BakeOptions o;
    o.size = r.size;
    o.color = true;
    o.aoRays = 64;
    o.aoDistance = 1.0;
    const BakedMaps m = bakeMaps(low, high, o);
    double colourErr = 0, aoNear = 0, aoFar = 0;
    int near = 0, far = 0, n = 0;
    for (std::size_t i = 0; i < m.covered.size(); ++i) {
        if (m.covered[i] != 1 || m.world[i].lengthSquared() == 0) continue;
        const Vec3& p = m.position[i];
        // (clear of the wall's foot: at x = -1 the floor's edge meets the wall's own faces, which a straight-down
        // ray may graze instead)
        if (p.x > -0.97) colourErr = std::max(colourErr, std::fabs(m.color[i].x - (0.5 - p.x * 0.5)) + std::fabs(m.color[i].y - (0.5 + p.x * 0.5)));
        ++n;
        if (p.x < -0.8) { aoNear += m.ao[i]; ++near; }
        if (p.x > 0.8) { aoFar += m.ao[i]; ++far; }
    }
    aoNear /= std::max(near, 1);
    aoFar /= std::max(far, 1);
    std::printf("    colour: worst error %.4f over %d texels; occlusion by the wall %.2f, across the floor %.2f\n", colourErr, n, aoNear, aoFar);
    CHECK(colourErr < 0.03);
    CHECK(aoNear < aoFar - 0.2);
    CHECK(aoFar > 0.9);
}

TEST_CASE(bake_dilates_past_the_chart_edges_and_is_deterministic) {
    RenderMesh low = MeshBuilder::box(Vec3(1, 1, 1));
    const UnwrapReport r = unwrapMesh(low, UnwrapOptions{50, 0.92, 64, 4, 1024});
    const RenderMesh high = MeshBuilder::roundedBox(Vec3(1, 1, 1), 0.1, 4);
    BakeOptions o;
    o.size = r.size;
    o.dilate = 3;
    const BakedMaps a = bakeMaps(low, high, o), b = bakeMaps(low, high, o);
    int dilated = 0;
    for (uint8_t c : a.covered) if (c == 2) ++dilated;
    bool same = a.normal.size() == b.normal.size();
    for (std::size_t i = 0; same && i < a.normal.size(); ++i)
        same = a.normal[i].x == b.normal[i].x && a.normal[i].y == b.normal[i].y && a.normal[i].z == b.normal[i].z;
    std::printf("    dilation: %d texels filled past the charts\n", dilated);
    CHECK(dilated > 0);
    CHECK(same);
}

// THE LUA WORDS: mesh.unwrap then mesh.bake from a recipe, a coarse ball under a smooth one
TEST_CASE(bake_from_lua_unwraps_and_bakes_a_normal_map) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::string err;
    CHECK(vm.doString(R"(
        local low, rep = mesh.unwrap(mesh.flat_shade(mesh.icosphere(1)), { texels_per_metre = 48 })
        local maps = mesh.bake(low, mesh.sphere(1.0), { size = rep.size, cage = 0.12, ao_rays = 8 })
        __ok = rep.ok and rep.charts > 0 and maps.texels > 0 and maps.hits == maps.texels and maps.normal ~= nil and maps.ao ~= nil
        __charts, __texels = rep.charts, maps.texels
    )", &err));
    if (!err.empty()) std::printf("    lua: %s\n", err.c_str());
    bool ok = false;
    vm.getGlobalBool("__ok", ok);
    CHECK(ok);
}

namespace {
double raise2cm(double, double) { return 0.02; }
double tilt(double x, double) { return 0.25 * x; }
double sunk(double, double) { return -0.15; }
// the mean over the chart texels that found the surface, of f(i)
template <class F> double meanOver(const BakedMaps& m, F f) {
    double s = 0; int n = 0;
    for (std::size_t i = 0; i < m.covered.size(); ++i)
        if (m.covered[i] == 1 && m.world[i].lengthSquared() > 0) { s += f(i); ++n; }
    return n ? s / n : 0;
}
}  // namespace

TEST_CASE(bake_curvature_height_and_thickness_read_their_shapes) {
    // CURVATURE: a unit sphere is 1/m convex everywhere; turned inside out, -1/m
    RenderMesh low = MeshBuilder::icosphere(2);
    const UnwrapReport r = unwrapMesh(low, UnwrapOptions{50, 0.92, 48, 2, 1024});
    BakeOptions o;
    o.size = r.size; o.cage = 0.1; o.curvature = true;
    RenderMesh high = MeshBuilder::sphere(1.0f, 64, 128);
    const BakedMaps conv = bakeMaps(low, high, o);
    const double kConv = meanOver(conv, [&](std::size_t i) { return conv.curvature[i]; });
    RenderMesh inside = high;
    for (Vertex& v : inside.vertices) v.normal = v.normal * -1.0;
    for (std::size_t i = 0; i + 2 < inside.indices.size(); i += 3) std::swap(inside.indices[i + 1], inside.indices[i + 2]);
    RenderMesh lowIn = low;
    for (Vertex& v : lowIn.vertices) v.normal = v.normal * -1.0;
    for (std::size_t i = 0; i + 2 < lowIn.indices.size(); i += 3) std::swap(lowIn.indices[i + 1], lowIn.indices[i + 2]);
    const BakedMaps conc = bakeMaps(lowIn, inside, o);
    const double kConc = meanOver(conc, [&](std::size_t i) { return conc.curvature[i]; });
    // HEIGHT: a floor 2 cm above the cheap one
    RenderMesh flatLow = grid(1, 1.0, flat);
    const UnwrapReport rf = unwrapMesh(flatLow, UnwrapOptions{50, 0.92, 128, 2, 1024});
    BakeOptions oh;
    oh.size = rf.size; oh.cage = 0.05; oh.height = true;
    const BakedMaps hm = bakeMaps(flatLow, grid(8, 1.0, raise2cm), oh);
    const double h = meanOver(hm, [&](std::size_t i) { return hm.height[i]; });
    // THICKNESS: the top of a 10 cm slab
    RenderMesh slab = MeshBuilder::box(Vec3(1, 0.1, 1));
    MeshBuilder::transform(slab, Mat4::translate(0, -0.05, 0));
    BakeOptions ot;
    ot.size = rf.size; ot.cage = 0.05; ot.thickness = true; ot.thicknessMax = 0.5;
    const BakedMaps tm = bakeMaps(flatLow, slab, ot);
    const double t = meanOver(tm, [&](std::size_t i) { return tm.thickness[i]; });
    std::printf("    curvature: sphere %.3f /m, inside out %.3f /m; height %.4f m; thickness %.4f m\n", kConv, kConc, h, t);
    CHECK(std::fabs(kConv - 1.0) < 0.1);
    CHECK(std::fabs(kConc + 1.0) < 0.1);
    CHECK(std::fabs(h - 0.02) < 0.001);
    CHECK(std::fabs(t - 0.1) < 0.005);
}

TEST_CASE(bake_by_part_takes_its_own_surface_and_samples_agree) {
    // the cheap floor (part 0) over its own flat floor (part 0) and, nearer, a tilted sheet of another part (1):
    // by part, the flat one is baked; without parts, the nearer tilted one
    RenderMesh low = grid(1, 1.0, flat);
    const UnwrapReport r = unwrapMesh(low, UnwrapOptions{50, 0.92, 96, 2, 1024});
    RenderMesh high = grid(8, 1.0, sunk);                     // its own part, 15 cm below
    const std::size_t flatFaces = high.indices.size() / 3;
    RenderMesh sheet = grid(8, 1.0, tilt);                    // another part's sheet, nearer everywhere (|y| <= 12.5 cm)
    MeshBuilder::append(high, sheet);
    std::vector<int> lowParts(low.indices.size() / 3, 0), highParts(high.indices.size() / 3, 1);
    std::fill(highParts.begin(), highParts.begin() + static_cast<long>(flatFaces), 0);
    BakeOptions o;
    o.size = r.size; o.cage = 0.2;
    const BakedMaps any = bakeMaps(low, high, o);
    o.lowParts = &lowParts; o.highParts = &highParts;
    const BakedMaps own = bakeMaps(low, high, o);
    o.samples = 3;
    const BakedMaps fine = bakeMaps(low, high, o);
    const double zAny = meanOver(any, [&](std::size_t i) { return any.normal[i].z; });
    const double zOwn = meanOver(own, [&](std::size_t i) { return own.normal[i].z; });
    const double zFine = meanOver(fine, [&](std::size_t i) { return fine.normal[i].z; });
    std::printf("    by part: tangent z %.4f any surface, %.4f its own part, %.4f with 3x3 samples\n", zAny, zOwn, zFine);
    CHECK(zOwn > 0.9999);
    CHECK(zAny < 0.99);
    CHECK(std::fabs(zFine - zOwn) < 1e-4);
}

TEST_CASE(bake_parts_atlases_the_baked_parts_and_leaves_the_glass_alone) {
    // a body baked onto one atlas beside a pane of glass that is not baked: the glass keeps its uvs, takes no texels
    BakePart body;
    body.low = MeshBuilder::icosphere(1);
    MeshBuilder::flatShade(body.low);
    body.high = MeshBuilder::sphere(1.0f, 48, 96);
    BakePart glass;
    glass.low = MeshBuilder::box(Vec3(0.6, 0.4, 0.02));
    MeshBuilder::transform(glass.low, Mat4::translate(0, 0, 1.1));
    for (Vertex& v : glass.low.vertices) { v.u = 0.25f; v.v = 0.75f; }
    glass.bake = false;
    BakeOptions bo;
    bo.cage = 0.12;
    const BakedModel m = bakeParts({body, glass}, UnwrapOptions{50, 0.92, 48, 2, 1024}, bo);
    int glassKept = 0, glassVerts = 0;
    for (std::size_t c = m.firstIndex[1]; c < m.mesh.indices.size(); ++c) {
        const Vertex& v = m.mesh.vertices[m.mesh.indices[c]];
        ++glassVerts;
        if (v.u == 0.25f && v.v == 0.75f) ++glassKept;
    }
    std::printf("    parts: %d charts at %d px, %d texels found of %d; glass corners kept %d of %d\n", m.unwrap.charts, m.unwrap.size,
                m.maps.hits, m.maps.texels, glassKept, glassVerts);
    CHECK(m.unwrap.ok);
    CHECK(m.maps.texels > 1000 && m.maps.hits == m.maps.texels);
    CHECK(glassKept == glassVerts && glassVerts > 0);
}
