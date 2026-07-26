#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_PATCH_FABRIC_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_PATCH_FABRIC_H

// PATCH-CONFORMING FABRIC (8km-city plan P7, Glenn's design): subdivide a
// patch — a face enclosed by arterial roads — into small city blocks whose
// grid CONFORMS to the patch's own shape. Streets near the boundary run
// parallel to the arterials (inset offset rings, curving with them);
// cross-streets ("ribs") branch perpendicular to the boundary's local tangent
// every ~block length; interior cells are locally rectilinear in the boundary
// frame. Wedge cells against curved arterials and an irregular mid-patch seam
// are WANTED variety. Never a grid overlay: the city flows in the shape of
// its arterials. (Metropolis-era gridFill sliced ONE straight OBB grid per
// face — the look this module replaces.)
//
// Two generators share the interface so the fabric lab can show them side by
// side; the production recommendation (architect pressure-test) is RINGS —
// the tensor tracer is demo-grade (its streamline stops dangle by design and
// are only naively trimmed here).

#include "road_network.h"

#include <vector>

namespace engine {

struct PatchFabricParams {
    double blockLen = 150.0;     // rib spacing along rings (cell width)
    double blockDepth = 150.0;   // ring offset step (cell depth)
    double streetWidth = 10.0;   // emitted street width
    double cornerClear = 0.0;    // no station within this of a boundary corner
                                 // (0 = derive from blockLen)
    uint32_t seed = 1;           // station phase / tensor jitter
};

// A street segment to weld into the host graph. Endpoints on the patch
// boundary land exactly on it (the caller's planarize T-splits the chain).
struct FabricSegment {
    Vec2 a, b;
    Real width = 10.0;
    RoadClass klass = RoadClass::Local;
};

// Rings + ribs (production candidate). Deterministic; a pure function of
// (patch, params). Emits nothing for patches smaller than one block.
std::vector<FabricSegment> fabricRingsRibs(const Poly2& patch,
                                           const PatchFabricParams& params);

// Tensor-field per patch (lab demo): boundary-tangent basis fields decaying
// inward, traced as evenly-spaced streamlines, clipped to the patch, dangles
// naively trimmed. Deterministic.
std::vector<FabricSegment> fabricTensor(const Poly2& patch,
                                        const PatchFabricParams& params);

// The ring primitive, exposed for tests: mitered closed-loop offset of the
// patch boundary at `depth`, split into maximal VALID arcs — every sample
// inside the patch and >= depth-eps from the boundary (the medial-validity
// prune that survives concavity, lobe splits, and collapse).
std::vector<std::vector<Vec2>> insetRingArcs(const Poly2& patch, double depth);

}  // namespace engine

#endif
