#ifndef RAYTRACER_ENGINE_PROCGEN_POLYMESH_H
#define RAYTRACER_ENGINE_PROCGEN_POLYMESH_H

// POLYMESH (fleet v2, ADR-0139; Glenn: "how can we better generate car meshes that aren't just cubes on
// wheels? ... some kind of geometry node set that allows us to build lots of different meshes"). An
// editable polygon mesh and the operations a box-modeller uses, so a vehicle -- or anything hard-surface
// -- is a short recipe instead of a bespoke C++ generator:
//
//   loft       a body from DIFFERENT cross-sections (hood -> cab -> box), each a rounded 2-D polygon
//   extrude    grow a region of faces out (bumpers, fenders, a cab roof, a mirror arm)
//   inset      a ring inside each face (window frames, grilles, door panels, lamp bezels)
//   crease     mark edges sharp (or semi-sharp) so subdivision keeps them
//   subdivide  Catmull-Clark: a coarse cage becomes a smooth body; the LEVEL is the LOD
//   mirror     build one side, weld the seam
//   toParts    one RenderMesh per material (paint, glass, trim, rubber, chrome, lamp, interior)
//
// Faces are polygons (usually quads), counter-clockwise seen from outside. Each carries a GROUP (what it
// is: "hood", "window_l"), a MATERIAL (what it is drawn with) and a colour. Edges carry a crease
// sharpness: 0 smooth, >= 1 stays sharp for that many subdivision levels, fractional blends, kInfCrease
// forever. Boundary edges are always sharp.

#include "../../renderer/renderer.h"   // RenderMesh
#include "../../rt_math.h"
#include "city/polygon.h"          // Vec2

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace engine {

struct PolyMesh {
    static constexpr float kInfCrease = 1e6f;
    struct Face {
        std::vector<int> v;
        int group = 0;
        int mat = 0;
        int panel = 0;   // the moving part it belongs to (panels[]: "body", or a door, hood, trunk ...)
        Vec3 color{1, 1, 1};
    };
    std::vector<Vec3> pts;
    std::vector<Face> faces;
    std::vector<std::string> groups{"default"};
    std::vector<std::string> mats{"body"};
    // PANELS (ADR-0352): a third face tag, beside group and material, that survives subdivision -- which part of
    // a model a face moves with. Windows keep their groups; a door's glass is still glass, on the door panel.
    std::vector<std::string> panels{"body"};
    std::map<std::pair<int, int>, float> creases;   // undirected (min, max) -> sharpness
    // VERTEX corners: a vertex with sharpness >= 1 keeps its place through that many subdivision levels
    // (the Catmull-Clark corner rule), fractional blends -- a window's corner stays a corner while the
    // outline between corners stays smooth.
    std::map<int, float> corners;

    int groupId(const std::string& name);            // adds if new
    int matId(const std::string& name);
    int findGroup(const std::string& name) const;    // -1 if absent
    int findMat(const std::string& name) const;
    int panelId(const std::string& name);            // adds if new
    int findPanel(const std::string& name) const;
    float crease(int a, int b) const;
    void setCrease(int a, int b, float s);
    float corner(int v) const;
    void setCorner(int v, float s);
    Vec3 faceCentroid(int f) const;
    Vec3 faceNormal(int f) const;                    // unit (Newell), outward for a CCW face
    void append(const PolyMesh& other);              // groups/materials matched by name
};

// A 2-D polygon (x lateral, y up; CCW seen from +z) with each corner rounded by radii[i] (one value =
// all corners) in `segs` steps. A radius is clamped to what the two adjacent edges allow.
// `segsPer` (optional, one per corner) overrides `segs` corner by corner: 0 makes that corner exactly one
// sharp point. Sections built with the same corner list and counts correspond point for point.
std::vector<Vec2> roundedPolygon(const std::vector<Vec2>& corners, const std::vector<double>& radii, int segs,
                                 const std::vector<int>& segsPer = {});

// Resample a closed polygon to `n` points evenly by arc length, starting at the point where the
// polygon crosses x = 0 at its lowest y (so every section of a symmetric body starts at the same
// place and corresponds point for point).
std::vector<Vec2> resampleClosed(const std::vector<Vec2>& poly, int n);

// A loft: ring k is sections[k] placed at z = stations[k] (each section's points in XY, all the same
// count, CCW seen from +z). Quads join consecutive rings; `capStart`/`capEnd` close the first and last
// ring with one polygon each (group "cap_start"/"cap_end"). Stations ascend (tail at the lowest z).
// `capCrease` is the sharpness of the rim where a cap meets the sides (0 = let subdivision round it).
PolyMesh loft(const std::vector<std::vector<Vec2>>& sections, const std::vector<double>& stations,
              bool capStart, bool capEnd, float capCrease = 1.0f);

// Region extrude: the faces in `sel` move out by `dist` along each vertex's averaged face normal (or
// along `dir` if it is non-zero); their boundary grows side walls (group of the face they border).
// Returns the moved faces (the same indices). Side walls inherit the adjacent face's material.
std::vector<int> extrude(PolyMesh& m, const std::vector<int>& sel, double dist, Vec3 dir = Vec3(0, 0, 0));

// Per-face inset: each face in `sel` shrinks toward its centroid by `amount` metres (along each corner's
// bisector, clamped), the ring between becomes quads with the face's group. Returns the inner faces.
std::vector<int> inset(PolyMesh& m, const std::vector<int>& sel, double amount);

// Region inset: the region `sel` as ONE panel shrinks inside its own outline by `amount` (its interior
// points stay), a ring of quads between. For grilles, doors, panels spanning several faces.
std::vector<int> insetRegion(PolyMesh& m, const std::vector<int>& sel, double amount);

// Copy the faces of region `sel` into a NEW poly, every point moved `offset` along its averaged face normal
// (negative = inward) and, with `flip`, the winding reversed so the copy faces the other way. A cabin's
// headliner and door cards are the body's own inside, offset in and flipped to face the cabin.
PolyMesh extractOffset(const PolyMesh& m, const std::vector<int>& sel, double offset, bool flip);
// THICKNESS for a skin (ADR-0352: a door, a hood, the body's side of the opening): the region's back -- its faces
// offset `thickness` inward and turned to face in, in material `mat` -- and a wall along every edge of its outline
// joining the skin to that back, all as a NEW poly to append. The walls face out of the region, so a door seen
// edge-on is a solid slab. With `against`, a wall is built only where the outline runs along one of THOSE faces: the
// body's side of a door opening wants its wall at the opening, not a second one buried in the body along the far
// side of the ring of faces that was thickened (that one shared its edge with two skin faces: three faces on one
// edge, which no closed skin has).
PolyMesh solidify(const PolyMesh& m, const std::vector<int>& sel, double thickness, const std::string& mat,
                  const std::vector<int>* against = nullptr);
// The faces outside `sel` that share a corner with it (one ring round the region).
std::vector<int> ringAround(const PolyMesh& m, const std::vector<int>& sel);

// Crease every edge on the outline of the region `sel` (edges with exactly one face in it).
void creaseBorder(PolyMesh& m, const std::vector<int>& sel, float sharpness);
// Crease every edge of every face in `sel`.
void creaseFaces(PolyMesh& m, const std::vector<int>& sel, float sharpness);
// Mark as corners the vertices where the outline of region `sel` turns by more than `angleDeg`.
void creaseCorners(PolyMesh& m, const std::vector<int>& sel, double angleDeg, float sharpness);

// Mirror across x = 0 and weld points within `eps` of the plane. Faces straddling nothing: the input is
// one half with its seam on x = 0.
void mirrorX(PolyMesh& m, double eps = 1e-4);

// Catmull-Clark, `levels` times, honouring creases (sharp, semi-sharp, corners with 3+ sharp edges).
PolyMesh subdivide(const PolyMesh& m, int levels);

// Face selection helpers.
// EDGE BEVEL (#191; the first of the modelling tools Glenn asked for, 2026-11-20): cut every sharp edge back to a
// flat strip (a chamfer), so a hard-surface part has an edge that catches the light. An edge is sharp when its two
// faces meet at more than `angleDeg` (the angle between their normals). Each face keeps its plane and is drawn back
// `width` metres from each of its sharp edges; a strip joins the two faces along the edge, and where three or more
// sharp edges meet, a small face closes the corner. A closed mesh stays closed and outward-facing.
//
// Returns the new faces (strips and corners), which are also put in the group "bevel" with the material, panel and
// colour of a face beside them. `sel`, when not empty, limits the bevel to edges between two faces of the selection
// or on its outline (so one panel's rim can be bevelled alone).
//
// What it does not do: round the strip (subdivide after it, or bevel twice, for a softer edge); stop a width wider
// than a face from folding the face over (keep `width` under half the narrowest face); keep creases on the edges it
// cuts (they are gone with the edge). An edge on the open border of a mesh is left alone.
std::vector<int> bevel(PolyMesh& m, double width, double angleDeg = 30.0, const std::vector<int>& sel = {});

// LOOP CUT (#191): a new ring of edges through a run of quads. Starting at the edge between points `a` and `b`, the
// cut crosses each quad to the edge opposite, on and on, until it comes back round, reaches the mesh's open border,
// or meets a face that is not a quad. Each crossed edge gets a new point `t` of the way along it (0.5: the middle;
// measured from a's side all the way round) and each crossed quad becomes two. A face the cut stops at gains the
// new point on its edge, so a closed mesh stays closed. A crease on a crossed edge carries on over both halves.
// Returns the new points in order; empty when a and b are not the two ends of an edge.
std::vector<int> loopCut(PolyMesh& m, int a, int b, double t = 0.5);

// SLICE: cut every face a plane passes through, along the plane. Each edge crossing the plane gets a point where it
// crosses and each face crossed from one edge to another becomes two; nothing moves, so the shape is the same with
// a ring of edges where the plane cuts it. (A loop cut follows the quads; a slice follows a plane, whatever the
// faces.) A point within `eps` of the plane counts as on it. Returns the points on the cut, new and old.
std::vector<int> slice(PolyMesh& m, Vec3 point, Vec3 normal, double eps = 1e-7);

// BRIDGE: join two regions of faces by a tube. The faces of `a` and of `b` are taken away and the two holes' rims
// joined by quads, rim point to rim point: two boxes facing each other become one solid, and a region on the top of
// a block bridged to one on its underside is a hole through it. Each region must have one rim, the two with the
// same number of points; the pairing is the one with the least twist. Returns the new quads (faces are renumbered:
// selections made before are stale). Empty, and nothing changed, when the rims do not match. Where a new quad
// would lie back to back with a face already there (a hole reaching the mesh's edge: a doorway down to the floor),
// both go and the hole is open on that side.
std::vector<int> bridge(PolyMesh& m, const std::vector<int>& a, const std::vector<int>& b);

// UN-SUBDIVIDE (#191): take one level of subdivision back out. A mesh that came from `subdivide` (or has the same
// pattern: all quads, every other point an edge's middle) loses its edge points and face points and keeps every
// second loop: a quarter of the faces, in the same order and shape, for a lower level of detail that is still an
// orderly grid (where the simplifier gives a scatter of triangles). The points that stay do not move, so the
// result is the subdivided shape sampled, a little inside the cage that made it.
//
// `keep` names one point that must stay (a mesh's pattern has two sets of points that could be "the corners"; by
// default the set holding point 0, which is the one `subdivide` wrote first). A crease that ran along both halves
// of an edge runs along the whole edge, one level sharper.
//
// Returns false, and changes nothing, when the mesh does not have the pattern: a face that is not a quad, an open
// border, or loops that do not pair off.
bool unsubdivide(PolyMesh& m, int keep = 0);

// The point nearest to `at` (for naming an edge to loopCut by where its ends are).
int nearestPoint(const PolyMesh& m, Vec3 at);

std::vector<int> selectAll(const PolyMesh& m);
std::vector<int> selectGroup(const PolyMesh& m, const std::string& group);
std::vector<int> selectWhere(const PolyMesh& m, const std::function<bool(const Vec3& centroid, const Vec3& normal)>& pred);

// Mesh quality (Glenn: "we definitely don't want wasted triangles in the runtime"): face count, triangles
// after triangulation, and the worst face's roundness 4*pi*area/perimeter^2 (1 a circle, 0.785 a square,
// -> 0 a sliver) with the count of faces under `sliver`.
struct PolyStats { int faces = 0, points = 0, triangles = 0, corners = 0, creases = 0, slivers = 0; double worst = 1.0; int worstFace = -1; };
PolyStats polyStats(const PolyMesh& m, double sliver = 0.02);

// The first surface a ray from `o` along `d` meets (faces fanned into triangles, either side). Placement
// rules use it to set parts ON the finished body -- a lamp, a mirror, a handle -- rather than on a guess
// from the profile. Returns false on a miss; `t` along d, the face's unit normal, the face index.
bool raycast(const PolyMesh& m, const Vec3& o, const Vec3& d, double& t, Vec3& normal, int& face);

// Topology checks (tests and recipe asserts).
bool isClosed(const PolyMesh& m);        // every edge in exactly two faces, in opposite directions
double signedVolume(const PolyMesh& m);  // > 0 for an outward-facing closed mesh

// Triangulate for the renderer: one RenderMesh per material, in the engine's winding. Normals are
// smoothed across edges whose faces meet within `autosmoothDeg` and share a material, split elsewhere
// (and at creases >= kInfCrease). UVs are box-projected at `uvScale` metres per tile; tangents follow.
// `panel` >= 0 emits only that panel's faces, its normals still averaged across the seam with its neighbours' so a
// closed door shades as one with the body (-1: every face).
std::map<std::string, RenderMesh> toParts(const PolyMesh& m, double autosmoothDeg = 35.0, double uvScale = 1.0,
                                          int panel = -1);

}  // namespace engine

#endif
