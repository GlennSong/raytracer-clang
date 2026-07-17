// The primitive the swept-lattice road mesher is built on
// (docs/road-mesher-research.md §1). A road is a swept surface — station rings
// x profile columns — so its mesh should be an INDEXED lattice with shared
// vertices, not the flat-shaded triangle soup weldSolid emits (measured V/T=2.9,
// a 457-triangle fan, 68.8% slivers). These tests pin the two properties the
// whole rewrite depends on: (1) vertices are SHARED, so V ~ T/2 and every
// vertex touches a bounded number of faces — there is real topology to
// subdivide/decimate/smooth along; (2) winding is consistent and front-facing
// no matter how the caller ordered columns or how the surface banks/curves.
#include "test_framework.h"

#include "../src/engine/mesh_builder.h"
#include "../src/renderer/renderer.h"
#include <cmath>
#include <map>
#include <vector>

using namespace engine;

namespace {

// A flat R x P lattice in the XZ plane, +Y up, laid out so columns advance +X
// and rings advance +Z. `flipCols` reverses the column order — same surface,
// opposite handedness — to prove winding is fixed from the normals, not assumed.
std::vector<Vertex> flatGrid(int R, int P, bool flipCols) {
    std::vector<Vertex> v(static_cast<std::size_t>(R) * P);
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < P; ++j) {
            const int col = flipCols ? (P - 1 - j) : j;
            Vertex& out = v[i * P + j];
            out.position = Vec3(col * 3.0, 0.0, i * 4.0);
            out.normal = Vec3(0, 1, 0);           // up-facing surface
            out.tangent = Vec3(1, 0, 0);
            out.u = static_cast<float>(col);
            out.v = static_cast<float>(i);
        }
    return v;
}

// Geometric front normal of a CW-wound triangle (engine convention:
// outward = cross(c-a, b-a)).
Vec3 faceNormal(const RenderMesh& m, std::size_t t) {
    const Vec3& a = m.vertices[m.indices[t]].position;
    const Vec3& b = m.vertices[m.indices[t + 1]].position;
    const Vec3& c = m.vertices[m.indices[t + 2]].position;
    return cross(c - a, b - a);
}

}  // namespace

// SHARING: an R x P lattice emits exactly R*P vertices (not 3*T) and
// 2*(R-1)*(P-1) triangles, so V/T ~ 0.5 — the opposite of a soup's 3.0. And
// because vertices are shared, an interior vertex is the corner of at most 6
// triangles (a soup fan puts hundreds on one point).
TEST_CASE(lattice_shares_vertices) {
    const int R = 12, P = 7;
    std::vector<Vertex> v = flatGrid(R, P, false);
    RenderMesh m;
    MeshBuilder::emitLattice(m, { R, P, v.data() });

    CHECK(m.vertices.size() == static_cast<std::size_t>(R) * P);
    CHECK(m.indices.size() == static_cast<std::size_t>(R - 1) * (P - 1) * 6);
    const std::size_t tris = m.indices.size() / 3;
    const double vPerT = static_cast<double>(m.vertices.size()) / tris;
    CHECK(vPerT < 0.7);            // shared lattice; a soup is 3.0

    // Max triangles sharing any one vertex (the "fan" metric).
    std::map<uint32_t, int> valence;
    for (uint32_t idx : m.indices) ++valence[idx];
    int worst = 0;
    for (const auto& [k, n] : valence) worst = std::max(worst, n);
    CHECK(worst <= 6);            // interior lattice vertex touches <= 6 faces
}

// WINDING: every face is front-facing (geometric normal agrees with the
// supplied vertex normal), and this holds even when the caller lists columns in
// the opposite order — the primitive reads the front from the normals ONCE for
// the whole lattice rather than assuming a column handedness.
TEST_CASE(lattice_winding_matches_supplied_normals) {
    for (bool flip : { false, true }) {
        std::vector<Vertex> v = flatGrid(9, 5, flip);
        RenderMesh m;
        MeshBuilder::emitLattice(m, { 9, 5, v.data() });
        CHECK(!m.indices.empty());
        int backfaces = 0, tris = 0;
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
            Vec3 gn = faceNormal(m, t);
            if (gn.lengthSquared() < 1e-12) continue;
            ++tris;
            if (dot(gn, Vec3(0, 1, 0)) < 0.0) ++backfaces;
        }
        CHECK(tris == 2 * 8 * 4);
        CHECK(backfaces == 0);
    }
}

// A curved, BANKED lattice (a superelevated road bending through a turn) still
// comes out consistently wound — the flip is chosen once and every face agrees
// with its own supplied normal, which is what a real road hands the primitive.
TEST_CASE(lattice_winding_holds_on_a_banked_curve) {
    const int R = 40, P = 5;
    std::vector<Vertex> v(static_cast<std::size_t>(R) * P);
    for (int i = 0; i < R; ++i) {
        const double s = i / static_cast<double>(R - 1);
        const double ang = s * 1.4;                       // sweeps through a bend
        const Vec3 c(60.0 * std::sin(ang), 3.0 * s, 60.0 * (1 - std::cos(ang)));
        const Vec3 fwd = normalize(Vec3(std::cos(ang), 0, std::sin(ang)));
        const Vec3 left = normalize(cross(Vec3(0, 1, 0), fwd));   // +t direction
        const double bank = 0.08;                          // 8% superelevation
        // banked up-normal: rotate +Y toward the outside of the turn
        const Vec3 up = normalize(Vec3(0, 1, 0) - left * bank);
        for (int j = 0; j < P; ++j) {
            const double lat = (j - (P - 1) * 0.5) * 3.5;
            Vertex& out = v[i * P + j];
            out.position = c + left * lat + up * (std::fabs(lat) * bank * 0);
            out.position.y += lat * bank;                  // bank the deck
            out.normal = up;
            out.u = static_cast<float>(j);
            out.v = static_cast<float>(i);
        }
    }
    RenderMesh m;
    MeshBuilder::emitLattice(m, { R, P, v.data() });
    int agree = 0, tris = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        Vec3 gn = faceNormal(m, t);
        if (gn.lengthSquared() < 1e-12) continue;
        ++tris;
        // each face should agree with the supplied normal at its first corner
        const Vec3& n0 = m.vertices[m.indices[t]].normal;
        if (dot(gn, n0) > 0.0) ++agree;
    }
    CHECK(tris > 0);
    CHECK(agree == tris);          // all front-facing against the banked normals
}
