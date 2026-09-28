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
        Vec3 color{1, 1, 1};
    };
    std::vector<Vec3> pts;
    std::vector<Face> faces;
    std::vector<std::string> groups{"default"};
    std::vector<std::string> mats{"body"};
    std::map<std::pair<int, int>, float> creases;   // undirected (min, max) -> sharpness
    // VERTEX corners: a vertex with sharpness >= 1 keeps its place through that many subdivision levels
    // (the Catmull-Clark corner rule), fractional blends -- a window's corner stays a corner while the
    // outline between corners stays smooth.
    std::map<int, float> corners;

    int groupId(const std::string& name);            // adds if new
    int matId(const std::string& name);
    int findGroup(const std::string& name) const;    // -1 if absent
    int findMat(const std::string& name) const;
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
std::map<std::string, RenderMesh> toParts(const PolyMesh& m, double autosmoothDeg = 35.0, double uvScale = 1.0);

}  // namespace engine

#endif
