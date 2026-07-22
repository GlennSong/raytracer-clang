#ifndef RAYTRACER_ENGINE_MESH_BUILDER_H
#define RAYTRACER_ENGINE_MESH_BUILDER_H

#include "../renderer/renderer.h"
#include <string>
#include <vector>

namespace engine {

struct MeshBuilder {
    // Level-format shape names ("box", "sphere", ...) -> mesh, with the same
    // size semantics as the level loader (x = radius for sphere/cylinder/...,
    // y = height where applicable). Empty mesh for unknown names.
    static RenderMesh shape(const std::string& name, Vec3 size);

    static RenderMesh box(Vec3 size);
    static RenderMesh sphere(float radius, int stacks = 32, int slices = 64);
    // A cube-sphere: the 6 faces of a cube, each a faceRes×faceRes grid, projected
    // onto the sphere and welded across the shared face seams into one watertight
    // manifold. Preferred over sphere() as a planet base (procedural-planet-plan,
    // ADR-0076) — near-uniform triangle area (no pole pinch, no longitude seam) and
    // cubemap-native: a face's texel↔direction follows cube_faces.h, so per-face
    // planet cubemaps line up with the mesh. `warp` applies the COBE / tangent
    // adjustment tan(s·π/4) that equalises the raw cube→sphere corner bunching
    // (~5:1 area ratio down to ~1.4:1). Vertex normal is the outward radial; front
    // faces wound clockwise (engine convention). Each face is a quadtree root for a
    // future LOD scheme (left dormant — ADR-0076 is from-orbit only).
    static RenderMesh cubeSphere(float radius, int faceRes = 32, bool warp = true);
    static RenderMesh cylinder(float radius, float height, int slices = 32);
    static RenderMesh plane(float width, float depth);
    static RenderMesh cone(float radius, float height, int slices = 32);
    static RenderMesh wedge(Vec3 size);
    static RenderMesh torus(float majorRadius, float minorRadius,
                            int majorSegments = 32, int minorSegments = 16);
    static RenderMesh capsule(float radius, float height,
                              int stacks = 16, int slices = 32);

    // --- Procgen-grade assembly ops (ROADMAP 3.3) ------------------------
    // These operate on RenderMesh as a value type so generators (L-systems,
    // terrain, scatter) compose geometry the same way loaded meshes are built —
    // no special procgen path. Pure data; unit-tested headless.

    // Append `src` into `dst`, offsetting src's indices so the two merge into
    // one vertex/index buffer (dst keeps its materialIndex).
    static void append(RenderMesh& dst, const RenderMesh& src);

    // Append `src` transformed by `xform`: positions by the matrix, normals by
    // its inverse-transpose (correct under non-uniform scale), renormalized.
    // The building block for kit-bashing parts — L-system segments, props.
    static void appendTransformed(RenderMesh& dst, const RenderMesh& src,
                                  const Mat4& xform);

    // Transform a mesh in place (positions + normals/tangents as above).
    static void transform(RenderMesh& mesh, const Mat4& xform);

    // Merge several meshes into one.
    static RenderMesh merged(const std::vector<RenderMesh>& parts);

    // Recompute smooth vertex normals from face geometry (area-weighted by the
    // face cross product), e.g. after displacing vertices (noise terrain).
    static void recomputeNormals(RenderMesh& mesh);

    // --- Winding-aware face emission -------------------------------------
    // The engine winds front faces CLOCKWISE: for a triangle (a,b,c) the
    // outward geometric normal is cross(c-a, b-a) (see recomputeNormals), and
    // the Metal viewer culls back faces by that rule. The offline path tracer
    // is two-sided, so it silently tolerates flipped winding — which is how
    // inside-out terrain/road meshes slip through until they reach the viewer.
    // These helpers bake the convention in once: each orients the triangle(s)
    // it appends so the geometric normal agrees with the supplied shading
    // `normal`, and flat-shades them with that normal + color. Generators
    // (terrain, roads, the shape grammar) emit through here instead of
    // hand-rolling index order, so winding can't drift per call site.

    // One triangle through the three corners, wound to face `normal`.
    static void emitTri(RenderMesh& mesh, const Vec3& a, const Vec3& b,
                        const Vec3& c, const Vec3& normal, const Vec3& color);

    // emitTri with explicit per-vertex UVs — for generators that bake a meaningful
    // parameterization (the welded carriageway's road-local u=lateral, v=arc-length,
    // read by the RoadMarkings surface). Same winding/colour handling as emitTri.
    static void emitTriUV(RenderMesh& mesh, const Vec3& a, const Vec3& b, const Vec3& c,
                          const Vec3& normal, const Vec3& color,
                          float ua, float va, float ub, float vb, float uc, float vc);

    // One quad: corners a,b,c,d in perimeter order, wound to face `normal`,
    // with 0..1 UVs (a->b is U, a->d is V) and a->b as the tangent.
    static void emitQuad(RenderMesh& mesh, const Vec3& a, const Vec3& b,
                         const Vec3& c, const Vec3& d, const Vec3& normal,
                         const Vec3& color);

    // emitQuad with explicit per-corner UVs (a..d) instead of the implicit 0..1
    // mapping — for generators that bake a meaningful parameterization (e.g. the road
    // carriageway's road-local u=lateral, v=arc-length, read by the RoadMarkings
    // surface). Same winding/colour handling as emitQuad.
    static void emitQuadUV(RenderMesh& mesh, const Vec3& a, const Vec3& b,
                           const Vec3& c, const Vec3& d, const Vec3& normal,
                           const Vec3& color, float ua, float va, float ub, float vb,
                           float uc, float vc, float ud, float vd);

    // Append clockwise-front indices for an up-facing (+Y) vertex lattice
    // already pushed row-major: vertex (i,j) lives at `base + j*cols + i`,
    // with +i = +X and +j = +Z. The shared triangulation for terrain grids
    // (heightfield bake + noise terrain), so the top always faces up.
    static void gridIndices(RenderMesh& mesh, int cols, int rows,
                            uint32_t base = 0);

    // --- Swept-lattice emission (road-mesher-research.md) -----------------
    // The primitive the road mesher needs and the engine was missing: a
    // (rings x profilePts) grid of SHARED vertices, connectivity by index
    // arithmetic — vertex (i,j) at base + i*profilePts + j, quad (i,j) over
    // {(i,j),(i,j+1),(i+1,j+1),(i+1,j)}. Row i is one station ring along a
    // swept surface; column j is one profile point across it. Unlike emitTri,
    // NOTHING is duplicated per face, so there is real topology to subdivide,
    // decimate, and smooth along — V/T ~ 0.5, not 3. Split columns (list the
    // same XZ twice with different normal/uv/color) to keep a crease flat-shaded
    // or an attribute seam sharp; the lattice stays regular.
    //
    // Winding is fixed ONCE for the whole lattice, from the supplied vertex
    // normals of the first non-degenerate cell (front = the caller's normals) —
    // NOT per face. Shared vertices cannot carry a per-face winding, which is
    // exactly the trap emitTri's per-face normal test would spring here.
    struct LatticeSpec {
        int rings = 0;         // number of station rings (rows of vertices)
        int profilePts = 0;    // number of profile points (columns of vertices)
        const Vertex* verts = nullptr;   // row-major, rings*profilePts, caller-filled
    };
    static void emitLattice(RenderMesh& mesh, const LatticeSpec& spec);

    // Planar UVs projected along an axis (0=x, 1=y, 2=z): the two perpendicular
    // position components, scaled, become (u, v). A cheap default mapping for
    // terrain (axis=1) and generated geometry that ships without UVs.
    static void generatePlanarUVs(RenderMesh& mesh, int axis = 1, float scale = 1.0f);

    // Tint per-vertex color by height: `low` at the mesh's lowest vertex, `high`
    // at its highest, linearly between. A cheap strata/canopy effect — e.g. a
    // brown trunk fading to green leaves on a tree (use a white material so the
    // vertex color shows). No-op on an empty mesh.
    static void bakeHeightColor(RenderMesh& mesh, const Vec3& low, const Vec3& high);
};

}  // namespace engine

#endif
