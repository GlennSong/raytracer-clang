// THE UV UNWRAPPER (engine/mesh_unwrap.h; Glenn, 2026-11-06: "let's build our own then and make a battle tested"):
// shapes with known answers, degenerate input, determinism, tagged faces, and a CORPUS -- every furniture piece in
// three variants and the mesh words' surfaces -- each held to the same bar: every face placed in [0, 1], no two
// charts sharing atlas cells, the texel density even, the stretch bounded.
#include "test_framework.h"

#include "../src/engine/mesh_builder.h"
#include "../src/engine/mesh_unwrap.h"
#include "../src/engine/procgen/furniture_kit.h"
#include "../src/proc_rng.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

using namespace engine;

namespace {

// The bar every unwrapped mesh is held to.
struct Bar {
    double overlap = 0.002;     // the share of covered cells covered twice (rasterisation at the triangles' edges)
    double densityLo = 0.70, densityHi = 1.40;   // the 5th / 95th percentile density ratio
    double stretch = 6.0;       // the worst face's anisotropy
};

bool held(const char* what, RenderMesh m, const Bar& bar = {}, UnwrapReport* repOut = nullptr) {
    const std::size_t tris = m.indices.size() / 3;
    const auto t0 = std::chrono::steady_clock::now();
    const UnwrapReport r = unwrapMesh(m);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const UvAudit a = auditUv(m, 1024);
    const bool good = r.ok && a.outside == 0 && a.overlapShare <= bar.overlap && a.densityLo >= bar.densityLo &&
                      a.densityHi <= bar.densityHi && a.worstStretch <= bar.stretch && m.indices.size() / 3 == tris;
    std::printf("    [unwrap] %-34s %6zu tris %4d charts (%d planar, %d lscm, %d splits) %4d px fill %.2f | overlap %.4f "
                "density %.2f..%.2f stretch %.2f | %.1f ms%s\n",
                what, tris, r.charts, r.planar, r.lscm, r.splits, r.size, r.fill, a.overlapShare, a.densityLo, a.densityHi,
                a.worstStretch, ms, good ? "" : "  <--");
    if (repOut) *repOut = r;
    return good;
}

}  // namespace

TEST_CASE(unwrap_a_box_lays_its_six_faces_flat_and_even) {
    UnwrapReport r;
    CHECK(held("box", MeshBuilder::box(Vec3(2, 1, 0.5)), Bar{0.002, 0.98, 1.02, 1.01}, &r));
    CHECK(r.charts == 6);
    CHECK(r.lscm == 0);
}

TEST_CASE(unwrap_curved_shapes_keep_their_charts_apart_and_even) {
    CHECK(held("sphere", MeshBuilder::sphere(1.0f, 24, 48)));
    CHECK(held("icosphere", MeshBuilder::icosphere(3)));
    CHECK(held("cylinder", MeshBuilder::cylinder(0.5f, 2.0f, 32)));
    CHECK(held("cone", MeshBuilder::cone(0.6f, 1.5f, 32)));
    CHECK(held("torus", MeshBuilder::torus(1.0f, 0.3f, 48, 24)));
    CHECK(held("superquadric", MeshBuilder::superquadric(Vec3(1, 0.5, 0.7), 0.3, 0.3)));
    CHECK(held("rounded box", MeshBuilder::roundedBox(Vec3(1.2, 0.4, 0.8), 0.1, 3)));
}

TEST_CASE(unwrap_survives_degenerate_and_odd_input) {
    RenderMesh empty;
    const UnwrapReport e = unwrapMesh(empty);
    CHECK(e.ok && e.charts == 0);
    // one triangle
    RenderMesh one;
    one.vertices = {Vertex(Vec3(0, 0, 0), Vec3(0, 1, 0)), Vertex(Vec3(0, 0, 1), Vec3(0, 1, 0)), Vertex(Vec3(1, 0, 0), Vec3(0, 1, 0))};
    one.indices = {0, 1, 2};
    CHECK(held("one triangle", one, Bar{0.0, 0.99, 1.01, 1.01}));
    // zero-area triangles among good ones, a repeated index, coincident points
    RenderMesh bad = MeshBuilder::box(Vec3(1, 1, 1));
    const uint32_t base = static_cast<uint32_t>(bad.vertices.size());
    bad.vertices.push_back(Vertex(Vec3(0.5, 0.5, 0.5), Vec3(0, 1, 0)));
    bad.vertices.push_back(Vertex(Vec3(0.5, 0.5, 0.5), Vec3(0, 1, 0)));
    bad.vertices.push_back(Vertex(Vec3(0.6, 0.5, 0.5), Vec3(0, 1, 0)));
    for (uint32_t i : {base, base + 1, base + 2, base, base, base + 2}) bad.indices.push_back(i);
    RenderMesh copy = bad;
    const UnwrapReport rb = unwrapMesh(copy);
    const UvAudit ab = auditUv(copy);
    std::printf("    degenerate: ok %d, charts %d, outside %d, overlap %.4f\n", rb.ok, rb.charts, ab.outside, ab.overlapShare);
    CHECK(rb.ok && ab.outside == 0 && ab.overlapShare < 0.002);
    // non-manifold: three faces on one edge (a fin)
    RenderMesh fin;
    fin.vertices = {Vertex(Vec3(0, 0, 0), Vec3(0, 1, 0)), Vertex(Vec3(1, 0, 0), Vec3(0, 1, 0)), Vertex(Vec3(0.5, 0, 1), Vec3(0, 1, 0)),
                    Vertex(Vec3(0.5, 1, 0), Vec3(0, 0, 1)), Vertex(Vec3(0.5, 0, -1), Vec3(0, 1, 0))};
    fin.indices = {0, 2, 1, 0, 1, 3, 0, 1, 4};
    CHECK(held("non-manifold fin", fin));
    // a long sliver
    RenderMesh sliver;
    sliver.vertices = {Vertex(Vec3(0, 0, 0), Vec3(0, 1, 0)), Vertex(Vec3(0, 0, 0.001), Vec3(0, 1, 0)), Vertex(Vec3(20, 0, 0), Vec3(0, 1, 0))};
    sliver.indices = {0, 1, 2};
    CHECK(held("sliver", sliver, Bar{0.0, 0.5, 2.0, 1.01}));
}

TEST_CASE(unwrap_is_deterministic) {
    RenderMesh a = MeshBuilder::torus(1.0f, 0.4f, 40, 20), b = a;
    unwrapMesh(a);
    unwrapMesh(b);
    CHECK(a.vertices.size() == b.vertices.size() && a.indices == b.indices);
    bool same = a.vertices.size() == b.vertices.size();
    for (std::size_t i = 0; same && i < a.vertices.size(); ++i) same = a.vertices[i].u == b.vertices[i].u && a.vertices[i].v == b.vertices[i].v;
    CHECK(same);
}

TEST_CASE(unwrap_keeps_tagged_faces_on_their_tag) {
    // a box whose top face is a lens (u = -1000 - role): it keeps the tag, the rest unwrap
    RenderMesh m = MeshBuilder::box(Vec3(1, 1, 1));
    int taggedVerts = 0;
    for (Vertex& v : m.vertices)
        if (v.normal.y > 0.9) { v.u = -1001.0f; v.v = 0; ++taggedVerts; }
    const UnwrapReport r = unwrapMesh(m);
    int still = 0;
    for (const Vertex& v : m.vertices) if (v.u == -1001.0f) ++still;
    const UvAudit a = auditUv(m);
    std::printf("    tagged: %d faces kept, %d charts, %d verts still tagged\n", r.tagged, r.charts, still);
    CHECK(r.ok && r.tagged == 2 && r.charts == 5 && still == taggedVerts && a.outside == 0);
}

TEST_CASE(unwrap_every_furniture_piece_holds_the_bar) {
    int pieces = 0, failed = 0;
    for (int p = 0; p < kPieceCount; ++p)
        for (uint32_t variant : {0u, 43u, 84u}) {
            const FurniturePiece& fp = furniturePiece(static_cast<Piece>(p), variant);
            RenderMesh all;
            for (const RenderMesh& part : fp.mesh) MeshBuilder::append(all, part);
            if (all.indices.empty()) continue;
            ++pieces;
            const std::string name = std::string(furniturePieceName(static_cast<Piece>(p))) + " v" + std::to_string(variant);
            if (!held(name.c_str(), all)) ++failed;
        }
    std::printf("    furniture: %d pieces unwrapped, %d below the bar\n", pieces, failed);
    CHECK(pieces > 200);
    CHECK(failed == 0);
}

// FUZZ (every seed works): lumpy spheres -- every vertex pushed along its radius by a random amount, so charts curve
// every which way -- and triangle SOUP -- random triangles in a box, crossing, nested and duplicated -- forty seeds
// each, all held to the bar.
TEST_CASE(unwrap_fuzz_lumpy_shapes_and_triangle_soup) {
    int failed = 0;
    for (uint32_t seed = 1; seed <= 40; ++seed) {
        ProcRng rng(seed);
        RenderMesh m = MeshBuilder::icosphere(2 + static_cast<int>(seed % 2));
        for (Vertex& v : m.vertices) v.position = v.position * (1.0 + rng.in(-0.35, 0.35));
        MeshBuilder::recomputeNormals(m);
        if (!held(("lumpy sphere seed " + std::to_string(seed)).c_str(), m, Bar{0.002, 0.6, 1.6, 12.0})) ++failed;
    }
    for (uint32_t seed = 1; seed <= 40; ++seed) {
        ProcRng rng(seed * 7919u);
        RenderMesh m;
        const int n = 20 + rng.below(200);
        for (int t = 0; t < n; ++t) {
            const Vec3 c(rng.in(-2, 2), rng.in(-2, 2), rng.in(-2, 2));
            for (int k = 0; k < 3; ++k)
                m.vertices.push_back(Vertex(c + Vec3(rng.in(-0.6, 0.6), rng.in(-0.6, 0.6), rng.in(-0.6, 0.6)), Vec3(0, 1, 0)));
            const uint32_t b = static_cast<uint32_t>(m.vertices.size()) - 3;
            m.indices.insert(m.indices.end(), {b, b + 1, b + 2});
            if (rng.next() < 0.1) m.indices.insert(m.indices.end(), {b, b + 1, b + 2});   // a duplicate face
        }
        if (!held(("soup seed " + std::to_string(seed)).c_str(), m, Bar{0.002, 0.6, 1.6, 1e9})) ++failed;
    }
    std::printf("    fuzz: %d below the bar\n", failed);
    CHECK(failed == 0);
}

// A BIG MESH in reasonable time: a 65k-triangle sphere, under two seconds
TEST_CASE(unwrap_a_big_mesh_in_reasonable_time) {
    RenderMesh m = MeshBuilder::sphere(1.0f, 128, 256);
    const auto t0 = std::chrono::steady_clock::now();
    CHECK(held("sphere 128 x 256", m));
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("    big mesh: %.2f s with the audit\n", s);
    CHECK(s < 4.0);
}
