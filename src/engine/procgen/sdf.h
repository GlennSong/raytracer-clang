#ifndef RAYTRACER_ENGINE_PROCGEN_SDF_H
#define RAYTRACER_ENGINE_PROCGEN_SDF_H

#include "../../rt_math.h"
#include "../../renderer/renderer.h"   // RenderMesh
#include <functional>
#include <vector>

namespace engine {

// Signed distance field (ROADMAP 4 Phase A.1): negative inside, positive
// outside, zero on the surface. A function of position, so fields compose by
// composing functions — the building blocks of "a rock/tree is a graph of SDF
// ops" (ADR-0021/0022). The in-house, robust analog to a B-rep modeling kernel:
// booleans are min/max, blends are smooth-min, and self-intersecting parts fuse
// into one continuous surface — which is exactly how organic things (welded
// L-system branches) should be built.
using Sdf = std::function<double(const Vec3&)>;

// --- primitives ---
Sdf sdfSphere(const Vec3& center, double radius);
Sdf sdfBox(const Vec3& center, const Vec3& halfExtent);
Sdf sdfCapsule(const Vec3& a, const Vec3& b, double radius);   // segment a..b

// --- combinators ---
Sdf sdfUnion(Sdf a, Sdf b);
Sdf sdfIntersect(Sdf a, Sdf b);
Sdf sdfSubtract(Sdf a, Sdf b);                 // a minus b
Sdf sdfSmoothUnion(Sdf a, Sdf b, double k);    // blended union (welds, fillets)
Sdf sdfUnion(const std::vector<Sdf>& parts);
Sdf sdfSmoothUnion(const std::vector<Sdf>& parts, double k);

// Central-difference gradient of a field (points outward, toward +distance).
Vec3 sdfGradient(const Sdf& field, const Vec3& p, double eps = 1e-3);

// Axis-aligned sampling region for polygonization.
struct SdfBounds {
    Vec3 min;
    Vec3 max;
};

// Polygonize a field into a RenderMesh via Surface Nets: sample the field on a
// (resolution^3) grid, place one vertex per sign-changing cell at the averaged
// edge crossings, and stitch quads across sign-changing grid edges. Vertex
// normals come from the field gradient (outward); triangle winding is set to the
// engine's clockwise-front convention. Smooth, watertight-ish, organic — ideal
// for welded branches and rocks. (Marching cubes / dual contouring with sharp
// features can come later.)
RenderMesh polygonizeSdf(const Sdf& field, const SdfBounds& bounds, int resolution);

}  // namespace engine

#endif
