#ifndef RAYTRACER_ENGINE_MESH_BUILDER_H
#define RAYTRACER_ENGINE_MESH_BUILDER_H

#include "../renderer/renderer.h"
#include "procgen/city/polygon.h"   // Vec2
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace engine {

// THE MESH LIBRARY (ADR-0172, Glenn 2026-10-13: "a common library of triangulation techniques ... how to represent
// triangles in this engine, and then code to handle all the different cases for building vehicles, buildings,
// furniture, etc. And then lua should be able to access it"). Every procedural mesh in the engine is built from the
// words below; a producer that needs a shape they lack ADDS a word here rather than a private helper.
//
//   THE CONVENTION  A RenderMesh is an indexed triangle list. A triangle (a, b, c) faces the way cross(c - a, b - a)
//                   points -- CLOCKWISE seen from its front (VK_FRONT_FACE_CLOCKWISE; back faces are culled) -- and
//                   that way must agree with its vertex normals. Nobody writes index order by hand: every triangle
//                   goes through triFacing (emitTri, emitQuad, emitBox, emitBlock, emitPolygon, emitPlanPrism,
//                   emitCylinder and the shape words all do), which winds it to the outward normal it is given and
//                   drops it when it has no area (kNoArea2).
//   SOLIDS ARE CLOSED  A box, block or prism has every face unless the caller masks one off because something
//                   covers it (a sleeper on its ballast): an open bottom is a hole when the solid stands over air.
//   ONE TRIANGULATOR  Polygons (with holes) fill through triangulate (earcut, procgen/city/triangulate.h).
//   CHECKED  engine/mesh_audit.h judges any mesh by these rules: the unit tests run it over every producer
//                   (tests/test_winding.cpp), rt_bake --audit over every bake.

// AN ORIENTED FRAME: a box spans origin + [0,1] x + [0,1] y + [0,1] z -- the axes carry the size and need not be unit
// or square (a sheared scope, a pitched part). `aabb` / `centred` build the common ones.
struct MeshFrame {
    Vec3 o{0, 0, 0}, x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};
    Vec3 at(double u, double v, double w) const { return o + x * u + y * v + z * w; }
    static MeshFrame aabb(const Vec3& lo, const Vec3& hi) {
        return {lo, Vec3(hi.x - lo.x, 0, 0), Vec3(0, hi.y - lo.y, 0), Vec3(0, 0, hi.z - lo.z)};
    }
    // `size` along unit axes `right`, `up`, `fwd` (default the world's), centred on `c`
    static MeshFrame centred(const Vec3& c, const Vec3& size, const Vec3& right = Vec3(1, 0, 0), const Vec3& up = Vec3(0, 1, 0),
                             const Vec3& fwd = Vec3(0, 0, 1)) {
        const Vec3 X = right * size.x, Y = up * size.y, Z = fwd * size.z;
        return {c - X * 0.5 - Y * 0.5 - Z * 0.5, X, Y, Z};
    }
};

// Which faces of a box (its frame's axes) or a block (foot/head and the sides in ring order) to emit.
enum MeshFaces : uint8_t {
    kFaceNegX = 1, kFacePosX = 2, kFaceNegY = 4, kFacePosY = 8, kFaceNegZ = 16, kFacePosZ = 32, kFacesAll = 63,
    kFaceBottom = kFaceNegY, kFaceTop = kFacePosY, kFaceSides = kFaceNegX | kFacePosX | kFaceNegZ | kFacePosZ,
};

// A LATHE's look (MeshBuilder::lathe): smooth (normals averaged through gentle bends) or faceted (every quad flat,
// the shape grammar's columns and finials); turned about +y through `centre`; vertex colour `color`.
struct LatheStyle {
    bool faceted = false;
    Vec3 centre{0, 0, 0};
    Vec3 color{1, 1, 1};
};

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
    // recomputeNormals with CREASES (PolyMesh toParts' autosmooth, for a RenderMesh): a corner's normal averages only
    // the faces round its vertex within `creaseDeg` of its own face; where they part by more, the vertex is split, so
    // a hard edge stays hard and no face is shaded from behind (smoothing across a fold turns a small face's normals
    // away from it -- the mesh audit's "inward" triangles on the car shell's nose).
    static void autosmooth(RenderMesh& mesh, double creaseDeg = 50.0);
    // A triangle whose |cross(c - a, b - a)|^2 is under this (area under 0.1 mm^2) has NO AREA: the emitters skip it,
    // dropDegenerate removes it, and the mesh audit (tests/mesh_invariants.h) counts by the same number.
    static constexpr double kNoArea2 = 4e-14;
    // Drop the triangles of no area (a sphere's pole row, a rounded box's collapsed corner): the vertices stay, so
    // indices into them hold. The grid primitives end with it.
    static void dropDegenerate(RenderMesh& mesh);

    // Split a WORLD-SPACE mesh into grid-cell chunks by triangle centroid (cell key = floor(x / cell),
    // floor(z / cell)), in first-seen order; vertices are duplicated per chunk (a vertex is shared by few
    // triangles), materialIndex carries over. The loader's render chunking and level bundles (ADR-0084)
    // both use it, so a bundle cell IS a render cell.
    struct CellChunk { int cx = 0, cz = 0; RenderMesh mesh; };
    static std::vector<CellChunk> chunkByCell(const RenderMesh& m, double cell);

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

    // --- THE SHAPE KIT (the flora plan): organic and mineral forms from a few operations,
    // shared by trees, rocks, shrubs, hedges -- and Lua (mesh.icosphere, mesh.displace, ...).
    // A unit icosphere, welded (subdiv 0: 12 verts / 20 faces; 1: 42 / 80; 2: 162 / 320).
    static RenderMesh icosphere(int subdiv);
    // Push every vertex along its direction from `centre` by amp * noise(p * freq): lumps.
    static void displaceNoise(RenderMesh& mesh, const Vec3& centre, double amp, double freq, uint32_t seed);
    // Clamp every vertex onto the plane dot(n, p - centre) <= d: a crisp flat break.
    static void cutByPlane(RenderMesh& mesh, const Vec3& centre, const Vec3& n, double d);
    // Unweld into flat faces (each triangle its own vertices, the face normal), wound outward
    // from `centre`. The low-poly look; soften afterwards with leanNormals.
    static void facet(RenderMesh& mesh, const Vec3& centre);
    // Lean every normal by `t` toward the normal of the ellipsoid (centre, radii) at that vertex:
    // the whole shape lights as one soft volume (canopies, hedges, semi-faceted stone).
    static void leanNormals(RenderMesh& mesh, const Vec3& centre, const Vec3& radii, double t);
    // Move every vertex through a function of its position (squash, bend, taper, flatten a foot).
    static void deform(RenderMesh& mesh, const std::function<Vec3(const Vec3&)>& fn);
    // Colour every vertex by a function of it (position, normal): gradients, moss by facing, AO.
    static void colorBy(RenderMesh& mesh, const std::function<Vec3(const Vertex&)>& fn);
    // A RIBBON along a curve: a strip through `points`, `halfWidths[i]` to each side, the side
    // direction horizontal-perpendicular to the curve (level across, like water or a road) unless
    // `up` says otherwise. u runs 0..1 across, v the distance along in metres; normals `up`;
    // `colours` per point (may be empty). Rivers today; fronds and paths are its other readers.
    static RenderMesh ribbon(const std::vector<Vec3>& points, const std::vector<double>& halfWidths,
                             const Vec3& up = Vec3(0, 1, 0), const std::vector<Vec3>& colours = {});
    // Indexed building blocks for custom topology (cone tiers, fronds): append one vertex (normal
    // normalised) and return its index; append a triangle wound to face `out` -- or nothing, when it has no area.
    // Every emitter here (emitTri, emitQuad and their UV forms) winds through triFacing.
    static uint32_t vertex(RenderMesh& mesh, const Vec3& p, const Vec3& n, const Vec3& color);
    static void triFacing(RenderMesh& mesh, uint32_t a, uint32_t b, uint32_t c, const Vec3& out);
    // CROSSED CARDS: `planes` vertical quads `width` x `height` through the origin, evenly turned
    // about +y, standing on y = 0. u runs 0..1 across a card, v 0 at the foot .. 1 at the top;
    // every normal is `normal` (straight up for ground cover, so it lights like the ground it
    // stands on). White. Grass cards, flowers, reeds, distant-tree impostors.
    static RenderMesh crossCards(double width, double height, int planes, const Vec3& normal = Vec3(0, 1, 0));
    // A tapered generalized cylinder through `points` (a `sides`-gon ring at each, radial
    // normals), colour per ring (`colours` may be empty: white).
    static RenderMesh tube(const std::vector<Vec3>& points, const std::vector<double>& radii, int sides,
                           const std::vector<Vec3>& colours = {});
    // A ROUNDED BOX (buildings M4b, the furniture words): `size` overall, centred on the origin, every edge and
    // corner rounded to `radius` (clamped to half the smallest side) in `segs` steps per quarter -- cushions,
    // mattresses, worktops, drawer fronts, upholstered panels. Smooth normals; UVs box-projected in metres times
    // `uvPerMetre` (u along the face's first in-plane axis), so a baked surface tiles at its real scale.
    static RenderMesh roundedBox(const Vec3& size, double radius, int segs = 2, double uvPerMetre = 1.0);
    // A LATHE: the profile (r, y) pairs, bottom to top, turned `segs` times about +y. Radial normals from the
    // profile's slope, smooth through a gentle bend and split at a corner sharper than 40 deg (a bowl's rim, a
    // plate's edge), each band wound by its own segment; a point on the axis (r = 0) closes the shape there. u round the turn 0..1, v the profile's
    // length in metres. Lamp bases, gas lifts, basins, bowls, finials.
    static RenderMesh lathe(const std::vector<std::pair<double, double>>& profile, int segs);
    static RenderMesh lathe(const std::vector<std::pair<double, double>>& profile, int segs, const LatheStyle& style);

    // --- THE SOLIDS AND FILLS (mesh_words.cpp) ---------------------------------------------------------------
    // A BOX on a frame, every face flat with its own normal, closed unless `faces` masks some off.
    static void emitBox(RenderMesh& mesh, const MeshFrame& f, const Vec3& color, uint8_t faces = kFacesAll);
    // An axis-aligned box from its two corners.
    static void emitBox(RenderMesh& mesh, const Vec3& lo, const Vec3& hi, const Vec3& color, uint8_t faces = kFacesAll);
    // A BLOCK: any six-sided solid from its foot ring a, b, c, d and the head ring over it (head[k] above foot[k]).
    // Faces: kFaceBottom the foot, kFaceTop the head; the side from corner k to k+1 is bit kFaceNegZ (k = 0),
    // kFacePosX (1), kFacePosZ (2), kFaceNegX (3) -- a box's own faces when the foot runs (x0 z0, x1 z0, x1 z1, x0 z1).
    // Each face faces away from the block's centre, whichever way round the corners come.
    static void emitBlock(RenderMesh& mesh, const Vec3 (&foot)[4], const Vec3 (&head)[4], const Vec3& color,
                          uint8_t faces = kFacesAll);
    // A PLANAR POLYGON with holes, any plane, any winding: triangulated by the one triangulator in its own plane and
    // wound to `normal`. `uvPerMetre` > 0 maps uv from the plane in metres.
    static void emitPolygon(RenderMesh& mesh, const std::vector<Vec3>& ring, const std::vector<std::vector<Vec3>>& holes,
                            const Vec3& normal, const Vec3& color, double uvPerMetre = 0.0);
    // A PLAN PRISM: an outline in plan (x, z) with holes, extruded from y0 to y1 -- the top facing up, the bottom
    // down, a wall along every edge of the outline facing out and of every hole facing into it. A slab, a pad, a
    // lip, a slab with a stair well. `faces`: kFaceTop, kFaceBottom, kFaceSides.
    static void emitPlanPrism(RenderMesh& mesh, const std::vector<Vec2>& outline, const std::vector<std::vector<Vec2>>& holes,
                              double y0, double y1, const Vec3& color, uint8_t faces = kFacesAll);
    // A CYLINDER standing on `base`: `sides` faces round, flat-faceted or smooth, capped top and bottom by `faces`
    // (kFaceTop, kFaceBottom; the side always).
    static void emitCylinder(RenderMesh& mesh, const Vec3& base, double radius, double height, int sides, const Vec3& color,
                             bool faceted = true, uint8_t faces = kFacesAll);
    // THE ONE TRIANGULATOR: index triples into the outline's points followed by each hole's, any winding in, the
    // triples' order unspecified (wind them with triFacing). Earcut (procgen/city/triangulate.h).
    static std::vector<std::array<int, 3>> triangulate(const std::vector<Vec2>& outline,
                                                       const std::vector<std::vector<Vec2>>& holes = {});
};

}  // namespace engine

#endif
