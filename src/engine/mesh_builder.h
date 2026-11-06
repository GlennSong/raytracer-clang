#ifndef RAYTRACER_ENGINE_MESH_BUILDER_H
#define RAYTRACER_ENGINE_MESH_BUILDER_H

#include "../renderer/renderer.h"
#include "procgen/city/polygon.h"   // Vec2
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace engine {

// THE MESH LIBRARY (ADR-0173, Glenn 2026-10-13: "a common library of triangulation techniques ... how to represent
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
    // the world's plan at height y round (x, z) = `at`, metres, facing up (or down): where the flat shapes lie
    static MeshFrame level(double y, bool up = true, const Vec2& at = Vec2(0, 0)) {
        return {Vec3(at.x, y, at.y), Vec3(1, 0, 0), Vec3(0, up ? 1 : -1, 0), Vec3(0, 0, 1)};
    }
    // `size` along unit axes `right`, `up`, `fwd` (default the world's), centred on `c`
    static MeshFrame centred(const Vec3& c, const Vec3& size, const Vec3& right = Vec3(1, 0, 0), const Vec3& up = Vec3(0, 1, 0),
                             const Vec3& fwd = Vec3(0, 0, 1)) {
        const Vec3 X = right * size.x, Y = up * size.y, Z = fwd * size.z;
        return {c - X * 0.5 - Y * 0.5 - Z * 0.5, X, Y, Z};
    }
    // The frame turned by `angle` (radians, right-handed) about the line through `pivot` along unit `axis`: a limb
    // pitched at its hip, a door on its hinge.
    MeshFrame rotatedAbout(const Vec3& pivot, const Vec3& axis, double angle) const {
        const double c = std::cos(angle), s = std::sin(angle);
        auto rot = [&](const Vec3& v) { return v * c + cross(axis, v) * s + axis * (dot(axis, v) * (1 - c)); };
        return {pivot + rot(o - pivot), rot(x), rot(y), rot(z)};
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

// CURVES AND PROFILES (#191, Glenn 2026-11-06: "We need to be able to go further than a box ... curves, rounded
// objects, and more"). A PROFILE is a 2-D section: points (x, y) in the plane it is swept or extruded in, closed
// (a ring; any winding in, kept counter-clockwise) or open (a moulding's edge). A PATH is a 3-D polyline, open or closed
// (a ring: the last point is NOT the first repeated). The curve words build them from the engine's splines
// (src/curve.h); the surface words (sweep, extrude, revolve, loft, superquadric) turn them into meshes.
struct MeshProfile {
    std::vector<Vec2> pts;
    bool closed = true;
};
struct MeshPath {
    std::vector<Vec3> pts;
    bool closed = false;
};
// A SWEEP's look: the profile scaled and turned along the path (`scaleAt` / `twistAt` take t in 0..1, the share of
// the path's length; null: 1 and 0), the ends capped (a closed profile on an open path), smooth round the profile
// where it bends less than `creaseDeg` (a square tube keeps its edges, a round one is smooth), and the frame's up
// along `up` at the start (rotation-minimising after it, so a swept rail does not spiral). u runs round the profile
// 0..1, v along the path in metres (bark, rails, cables tile at their real length).
struct SweepStyle {
    std::function<double(double)> scaleAt;
    std::function<double(double)> twistAt;
    bool caps = true;
    double creaseDeg = 40.0;
    Vec3 up{0, 1, 0};
    Vec3 color{1, 1, 1};
};

struct MeshBuilder {
    // --- PROFILES (mesh_surfaces.cpp): counter-clockwise, centred on the origin unless said ----------------------
    static MeshProfile profileRect(double width, double height);
    // a rectangle with every corner rounded to `radius` (clamped to half the shorter side) in `segs` steps
    static MeshProfile profileRoundedRect(double width, double height, double radius, int segs = 4);
    static MeshProfile profileCircle(double radius, int sides = 24);
    static MeshProfile profileEllipse(double rx, double ry, int sides = 24);
    // |x/rx|^n + |y/ry|^n = 1: n = 2 an ellipse, 4 a squircle, larger ever squarer, under 1 a star-cornered diamond
    static MeshProfile profileSuperellipse(double rx, double ry, double n, int sides = 48);
    // a regular `sides`-gon of circumradius `radius`, a corner on +x
    static MeshProfile profilePolygon(int sides, double radius);
    static MeshProfile profileStar(int points, double outer, double inner);
    // every corner of `p` rounded to `radius` (each clamped to what its two edges allow) in `segs` steps
    static MeshProfile profileFillet(const MeshProfile& p, double radius, int segs = 4);
    // a closed profile grown by `d` (shrunk when negative), its edges kept parallel (the mitre offset: exact for convex
    // profiles; empty when it collapses)
    static MeshProfile profileOffset(const MeshProfile& p, double d);
    // `count` points spaced evenly along the profile's length (corners are not kept)
    static MeshProfile profileResample(const MeshProfile& p, int count);

    // --- PATHS -------------------------------------------------------------------------------------------------
    static MeshPath pathLine(const Vec3& a, const Vec3& b, int segs = 1);
    // an arc round `centre` in the plane square to unit `normal`, from `from` (a vector from the centre, its length
    // the radius) turning `angle` radians right-handed about `normal`
    static MeshPath pathArc(const Vec3& centre, const Vec3& from, const Vec3& normal, double angle, int segs = 24);
    static MeshPath pathCircle(const Vec3& centre, double radius, const Vec3& normal = Vec3(0, 1, 0), int segs = 32);
    // a helix up +y round `centre`: `turns` turns of `radius`, rising `pitch` metres a turn
    static MeshPath pathHelix(const Vec3& centre, double radius, double pitch, double turns, int segsPerTurn = 32);
    // a smooth curve THROUGH `points` (centripetal Catmull-Rom: no cusps or overshoot), `count` points at even spacing
    static MeshPath pathSpline(const std::vector<Vec3>& points, bool closed = false, int count = 64);
    // a cubic Bezier from p0 to p3 pulled toward p1 and p2, `count` points at even spacing
    static MeshPath pathBezier(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, int count = 32);
    // the path again, its points `spacing` metres apart along it (ends kept)
    static MeshPath pathResample(const MeshPath& p, double spacing);
    // a path's length in metres
    static double pathLength(const MeshPath& p);

    // --- SURFACES ----------------------------------------------------------------------------------------------
    // SWEEP the profile along the path: at each point the profile stands square to the path, x along the frame's
    // right, y along its up. Rods, pipes, cables, rails, mouldings, handles, branches, a ring (a closed path).
    static RenderMesh sweep(const MeshProfile& profile, const MeshPath& path, const SweepStyle& style = {});
    // EXTRUDE a closed profile in plan (x -> world x, y -> world z) up +y from 0 to `height`, its top and bottom edges
    // rounded to `bevel` in `bevelSegs` steps (0: sharp), its side in `heightSegs` bands (more for a warp to bend).
    // A seat, a worktop, a plinth, a sign, a slab with soft edges.
    static RenderMesh extrude(const MeshProfile& profile, double height, double bevel = 0.0, int bevelSegs = 3,
                              const Vec3& color = Vec3(1, 1, 1), int heightSegs = 1);
    // REVOLVE a profile of (r, y) points about +y from angle `from` to `to` (radians; a full turn unless less), in
    // `segs` steps round; a part turn closes its cut faces when the profile is closed. Open profiles run bottom to
    // top up the outside (as the lathe's); u runs round the turn 0..1, v along the profile in metres, as the lathe's.
    // Vases, bowls, domes, finials, a wheel's tyre, a quarter-round moulding.
    static RenderMesh revolve(const MeshProfile& profile, int segs = 32, double from = 0.0, double to = TAU,
                              const Vec3& color = Vec3(1, 1, 1), double creaseDeg = 40.0);
    // LOFT through `sections` (rings of 3-D points, the same count each, in order): a surface skinned through them,
    // closed round each ring when `closedRings`, its ends capped when `caps`. Hulls, bodies, vases, a bottle from
    // circles, a car's sections.
    static RenderMesh loft(const std::vector<std::vector<Vec3>>& sections, bool closedRings = true, bool caps = true,
                           const Vec3& color = Vec3(1, 1, 1), double creaseDeg = 40.0);
    // A SUPERQUADRIC (Barr's superellipsoid): radii `r`; `e1` shapes it top to bottom and `e2` round it (1: round,
    // under 1: squarer, 0.1 nearly a box; 2: a diamond). One word from a box with soft edges to a sphere to a cylinder
    // with round ends: pebbles, cushions, buttons, soap-bar seats, rounded car parts.
    static RenderMesh superquadric(const Vec3& r, double e1, double e2, int stacks = 24, int slices = 48,
                                   const Vec3& color = Vec3(1, 1, 1));

    // --- MODIFIERS (mesh_surfaces.cpp) --------------------------------------------------------------------------
    // WARP: every vertex moved through `fn`, its normal and tangent carried through fn's Jacobian (measured by finite
    // differences `h` apart), so a crease stays a crease and shading follows the new shape; a warp that turns the
    // shape inside out (a mirror) flips the winding with it. Bend, twist and taper are warps.
    static void warp(RenderMesh& mesh, const std::function<Vec3(const Vec3&)>& fn, double h = 1e-4);
    // BEND the mesh's +y run [0, length] into an arc of `angle` radians toward +x (a rod into a hook, a sheet into a
    // pipe); points below 0 or above `length` carry on straight from the arc's ends.
    static void bend(RenderMesh& mesh, double angle, double length);
    // TWIST about +y: `radiansPerMetre` turns per metre of height (a twisted column, a drill, a cable)
    static void twist(RenderMesh& mesh, double radiansPerMetre);
    // TAPER about +y: x and z scaled from 1 at y = 0 to `topScale` at y = `height` (a tapered leg, an obelisk)
    static void taper(RenderMesh& mesh, double topScale, double height);
    // MIRROR across the plane dot(n, p) = d (unit n): the mirrored copy, wound to face out
    static RenderMesh mirrored(const RenderMesh& mesh, const Vec3& n, double d = 0.0);
    // ARRAY: `count` copies, each the last moved by `step` (a row of posts, a stack, a spiral stair's treads)
    static RenderMesh array(const RenderMesh& mesh, int count, const Mat4& step);
    // ...along a path: `count` copies spaced evenly, each turned to the path's frame (x right, y up, z along it)
    // when `align` (else only moved): fence posts on a curve, beads on a string, the teeth of a gear
    static RenderMesh arrayAlong(const RenderMesh& mesh, const MeshPath& path, int count, bool align = true,
                                 const Vec3& up = Vec3(0, 1, 0));

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
    // NORMAL TRANSFER (Glenn, 2026-11-06: "faceted lower poly cars with baked normals"): each vertex of `dst` takes the
    // normal of the `src` vertex that best matches it -- within `radius` metres, the nearest whose normal is closest to
    // the dst vertex's own (so the two sides of a crease each find their own side). A coarse cage then shades like the
    // smooth body it was subdivided into, at the cage's triangle count. A vertex with no candidate keeps its normal.
    static void transferNormals(RenderMesh& dst, const RenderMesh& src, double radius = 0.08);
    // A triangle whose |cross(c - a, b - a)|^2 is under this (area under 0.1 mm^2) has NO AREA: the emitters skip it,
    // dropDegenerate removes it, and the mesh audit (tests/mesh_invariants.h) counts by the same number.
    static constexpr double kNoArea2 = 4e-14;
    // Drop the triangles of no area (a sphere's pole row, a rounded box's collapsed corner): the vertices stay, so
    // indices into them hold. The grid primitives end with it.
    static void dropDegenerate(RenderMesh& mesh);
    // FLAT SHADING: every triangle its own three vertices, their normal the triangle's own facing (cross(c-a, b-a)),
    // its winding kept; no-area triangles dropped. Low-poly rocks, faceted gems.
    static void flatShade(RenderMesh& mesh);
    // SIMPLIFY (quadric error edge collapse, Garland & Heckbert 1997): collapse the edge that moves the surface least
    // -- each vertex carries the sum of its faces' plane quadrics (and its open edges', weighted heavy, so a border
    // keeps its line), an edge collapses to the point that minimises their sum -- until `targetTris` triangles are left
    // or the next collapse would move the surface more than `maxError` (squared distance, m^2). A collapse that would
    // turn a face over or make the surface non-manifold (the edge's ends sharing more neighbours than faces) is skipped.
    // Winding is kept. POSITIONS ONLY: a kept vertex keeps its other attributes as they were, so the caller re-derives
    // normals, UVs and colours (an implicit surface's from its field). Returns the triangles left.
    static std::size_t simplify(RenderMesh& mesh, std::size_t targetTris, double maxError = 1e30);
    // MANIFOLD BY SPLITTING: where two sheets of a mesh touch at a vertex (an isosurface's pinch where a feature is
    // thinner than its cell -- an edge then sits on four faces), the vertex is split, one copy for each fan of faces
    // round it that are joined through edges of exactly two faces. Positions are kept (the sheets still touch); the
    // mesh is then manifold by index. Returns the vertices added.
    static std::size_t splitNonManifold(RenderMesh& mesh);
    // FILL SMALL HOLES: every loop of open edges (an edge on one face, followed head to tail) of at most `maxEdges`
    // edges is closed by a fan round a new vertex at its middle (put on the surface by `project`), wound with the
    // faces round it. (splitNonManifold can open a few where two sheets crossed at an edge.) Returns the holes filled.
    static std::size_t fillHoles(RenderMesh& mesh, std::size_t maxEdges, const std::function<Vec3(const Vec3&)>& project);
    // ISOTROPIC REMESHING (Botsch & Kobbelt 2004): even, near-equilateral triangles of about `edgeAt(p)` metres --
    // `iterations` rounds of: split every edge over 4/3 of its target, collapse every edge under 4/5 (where no edge
    // it makes would be over 4/3, no face turns over and the surface stays manifold), flip edges toward valence six,
    // and move each vertex toward its neighbours' middle within its tangent plane -- every new or moved vertex put
    // back on the surface by `project` (an implicit surface's Newton step; identity for a mesh with none). The mesh
    // must be manifold (splitNonManifold first). Positions only, as simplify: the caller re-derives the rest.
    static void remesh(RenderMesh& mesh, const std::function<double(const Vec3&)>& edgeAt,
                       const std::function<Vec3(const Vec3&)>& project, int iterations = 6);

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
    // One cell of an up-facing lattice: corners (i, j) a00, (i+1, j) a10, (i+1, j+1) a11, (i, j+1) a01, split on the
    // a00-a11 diagonal (the CDLOD leaf layout drapeOnGround follows), clockwise-front facing +Y. gridIndices is this
    // over every cell; a sparse grid (the sea, a ring of terrain) calls it per cell.
    static void gridCell(RenderMesh& mesh, uint32_t a00, uint32_t a10, uint32_t a11, uint32_t a01);
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
    // PLANAR UVs along any axes, on the vertices from index `from` on (what an emitter just added): u = dot(p - origin,
    // uAxis), v = dot(p - origin, vAxis) -- the axes carry the scale (a direction over a tile size) -- and the tangent
    // along uAxis. A floor's boards run along its grain; a plate's texture tiles at its real size.
    static void projectUV(RenderMesh& mesh, std::size_t from, const Vec3& origin, const Vec3& uAxis, const Vec3& vAxis);

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
    // PAINT: every vertex one colour (a part's flat colour; the white material shows it). (#191: the one recolour,
    // for the loops each producer wrote and Lua's bake_height_color(m, c, c).)
    static void paint(RenderMesh& mesh, const Vec3& color) {
        for (Vertex& v : mesh.vertices) v.color = color;
    }
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
    // A quad over four vertices already placed (a b c d round it): two triFacing, split on a-c.
    static void quadFacing(RenderMesh& mesh, uint32_t a, uint32_t b, uint32_t c, uint32_t d, const Vec3& out) {
        triFacing(mesh, a, b, c, out);
        triFacing(mesh, a, c, d, out);
    }
    // CROSSED CARDS: `planes` vertical quads `width` x `height` through the origin, evenly turned
    // about +y, standing on y = 0. u runs 0..1 across a card, v 0 at the foot .. 1 at the top;
    // every normal is `normal` (straight up for ground cover, so it lights like the ground it
    // stands on). White. Grass cards, flowers, reeds, distant-tree impostors.
    static RenderMesh crossCards(double width, double height, int planes, const Vec3& normal = Vec3(0, 1, 0));
    // THE CONVEX HULL of `points` as triangles (indices into `points`), each wound counter-clockwise seen from
    // outside (its right-hand normal points out). Points inside, or on a face within a hair of it, are left out.
    // Fewer than four non-coplanar points: empty. Incremental (O(n^2)) -- for the small sets of a joint, a cap, a
    // rock (the tree's branch junctions: the hull of the limbs' end rings, ADR-0181).
    static std::vector<std::array<uint32_t, 3>> convexHull(const std::vector<Vec3>& points);
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
    // A SKIRT hung from a flat outline's edge at `topY` down to `bottoms(a, b)` (the bottom's y at a and at b, per edge:
    // the ground under a pad, a metre below it), each face facing out of the outline (into it when `facingIn`: a
    // hole). A paved plate's kerb, the side of a slab on a slope, a plan prism's walls.
    static void emitSkirt(RenderMesh& mesh, const std::vector<Vec2>& outline, double topY,
                          const std::function<Vec2(const Vec2&, const Vec2&)>& bottoms, const Vec3& color,
                          bool facingIn = false);
    // A CYLINDER standing on `base`: `sides` faces round, flat-faceted or smooth, capped top and bottom by `faces`
    // (kFaceTop, kFaceBottom; the side always).
    static void emitCylinder(RenderMesh& mesh, const Vec3& base, double radius, double height, int sides, const Vec3& color,
                             bool faceted = true, uint8_t faces = kFacesAll);
    // FLAT SHAPES lie in a frame's x-z plane (a profile's (x, y) -> f.at(x, 0, y); unit axes for metres) and face
    // along f.y (turn it over to face down). uv is the plane's metres.
    // A BAND between two outlines paired point for point (outer.pts[k] with inner.pts[k]), round and closed when
    // `outer` is closed: a coping's top, a track's lane, a ring drawn as paint.
    static void emitBand(RenderMesh& mesh, const MeshFrame& f, const MeshProfile& outer, const MeshProfile& inner,
                         const Vec3& color);
    // A FILL: the profile (closed) and its holes, triangulated flat. A stadium's infield, a sign's plate.
    static void emitFill(RenderMesh& mesh, const MeshFrame& f, const MeshProfile& outline, const Vec3& color,
                         const std::vector<MeshProfile>& holes = {});
    // AN ANNULUS round the frame's origin: radii rIn..rOut from angle a0 to a1 (radians from f.x toward f.z) in
    // `sides` steps, in `rings` bands outward (more to drape it on ground). rIn 0: a DISC, fanned from its centre;
    // a part turn: a SECTOR or an arc of ring. Paved rounds, roof pads, tank lids, a centre circle, a ring marker.
    static void emitAnnulus(RenderMesh& mesh, const MeshFrame& f, double rIn, double rOut, const Vec3& color,
                            int sides = 32, double a0 = 0.0, double a1 = TAU, int rings = 1);
    // THE ONE TRIANGULATOR: index triples into the outline's points followed by each hole's, any winding in, the
    // triples' order unspecified (wind them with triFacing). Earcut (procgen/city/triangulate.h).
    static std::vector<std::array<int, 3>> triangulate(const std::vector<Vec2>& outline,
                                                       const std::vector<std::vector<Vec2>>& holes = {});
};

}  // namespace engine

#endif
