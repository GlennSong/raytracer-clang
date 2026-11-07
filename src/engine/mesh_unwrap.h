#ifndef RAYTRACER_ENGINE_MESH_UNWRAP_H
#define RAYTRACER_ENGINE_MESH_UNWRAP_H

// THE UV UNWRAPPER AND ATLAS (#191; Glenn, 2026-11-06: "Can we build our uv unwrapper and Atlas system? ... let's
// build our own then and make a battle tested"). Any mesh gets a UNIQUE texture layout -- every triangle its own
// place in one square atlas, none overlapping -- so maps can be baked onto it (normals, occlusion, wear, colour) and
// damage painted per car. Three stages:
//
//   CHARTS   faces grown into patches from the largest unclaimed face across shared edges, each face within
//            `chartAngleDeg` of the patch's area-weighted normal (so a chart can never wrap round on itself)
//   FLATTEN  each chart projected onto its own plane; a chart curved past `lscmBelowCos` is flattened by LSCM (least
//            squares conformal maps: angles kept, solved by conjugate gradients); any chart whose flattening folds
//            over itself is split and done again
//   PACK     every chart at one texel density, turned to its tightest box, packed bottom-left with `padding` texels
//            round each so mip levels do not bleed one chart into the next
//
// The mesh comes back with its vertices split along the chart seams and u, v the atlas coordinates (0..1). Faces
// whose vertices carry a TAGGED uv (u <= -999: a car lens's role, vehicle_lamps.h) keep it and take no atlas space.
// Deterministic: the same mesh unwraps the same way.

#include "../renderer/renderer.h"

#include <vector>

namespace engine {

struct UnwrapOptions {
    double chartAngleDeg = 50.0;   // the most a face may turn from its chart's normal
    double lscmBelowCos = 0.92;    // a chart whose worst face is turned further than this from its plane: LSCM
    double texelsPerMetre = 128.0; // the density every chart is laid out at (before the atlas cap)
    int padding = 4;               // texels round each chart
    int maxSize = 4096;            // the atlas's largest side; the density shrinks to fit
    // the PART each face belongs to (one entry per face; null: one part): a chart never grows from one part into
    // another, though the parts touch (a body and the lens set into it)
    const std::vector<int>* faceGroups = nullptr;
};

struct UnwrapReport {
    int charts = 0, planar = 0, lscm = 0, splits = 0;
    int size = 0;                  // the atlas's side in texels (square, a power of two)
    double texelsPerMetre = 0.0;   // the density it was laid out at
    double fill = 0.0;             // the share of the atlas the charts' triangles cover
    int tagged = 0;                // faces kept on their tagged uv
    bool ok = false;               // every untagged face has a place
};

// Unwrap `mesh` in place (see above). An empty mesh unwraps to an empty report.
UnwrapReport unwrapMesh(RenderMesh& mesh, const UnwrapOptions& opts = {});

// THE CHECKS (tests/test_mesh_unwrap.cpp, and any caller that wants to be sure): over the untagged faces,
// the uv area two triangles share (rasterised at `res` cells a side: 0 when none overlap), the faces outside [0, 1],
// and the area-weighted spread of texel density (each face's sqrt(uv area / 3-D area) over the mesh's mean: 1 even).
struct UvAudit {
    double overlapShare = 0.0;     // of the covered cells, the share covered twice or more
    int outside = 0;               // faces with a uv outside [0, 1]
    double densityLo = 1.0, densityHi = 1.0;   // the 5th and 95th percentile (by area) of the density ratio
    double worstStretch = 1.0;     // the worst face's anisotropy (its uv triangle's long side over short, vs 3-D)
    int faces = 0;
};
UvAudit auditUv(const RenderMesh& mesh, int res = 1024);

}  // namespace engine

#endif
