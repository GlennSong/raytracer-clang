#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_PATCH_FABRIC_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_PATCH_FABRIC_H

// PATCH-CONFORMING FABRIC (8km-city plan P7, Glenn's design, rounds 7-13):
// subdivide a PATCH — a face enclosed by arterial roads — into small city
// blocks whose grid CONFORMS to the patch's own shape. The unit is the
// N-SIDED patch: corners are the face's arterial junctions, sides are the
// boundary chains between them (transfinite/Coons thinking — the same math
// road_lattice's coonsPatch uses for junction surfaces, lifted to city
// scale). Never a grid overlay: the city flows in the shape of its
// arterials. (Metropolis-era gridFill sliced ONE straight OBB grid per face
// — the look this module replaces.)
//
// The production menu (round 12): CHORDS (planned districts) / BISECT
// (grown, oldtown/industrial) / RING COURT (green heart), chosen per patch
// by the metro's district-weighted dispatch.

#include "road_network.h"

#include <vector>

namespace engine {

struct PatchFabricParams {
    double blockLen = 150.0;     // station spacing along sides (cell width)
    double blockDepth = 150.0;   // ring offset step / cell depth
    double streetWidth = 10.0;   // emitted street width
    double cornerClear = 0.0;    // no station within this of a boundary corner
                                 // (0 = derive from blockLen)
    uint32_t seed = 1;           // GLOBAL seed. Station phase/jitter hashes mix
                                 // it with side GEOMETRY (lexicographically
                                 // smallest endpoint first), so the two patches
                                 // sharing an arterial chain mint the SAME
                                 // stations — canonical chain stationing
                                 // (plan step 2.5) without a chain registry.

    // ROUND-10 dials (Glenn: loosen/tighten, rigid-square <-> Coons).
    double conform = 0.15;   // 0 = straight chord .. 1 = full Coons iso-curve,
                             // blended per line (endpoints always exact)
    double jitter = 0.12;    // seeded station jitter as a fraction of one
                             // station division (clamped to +/-0.25)
    double snapRatio = 1.3;  // side matching: opposite counts within this
                             // ratio snap to equal (extras dropped); above it
                             // the dense side's extras T-terminate instead
    // Dials present but not yet wired (round 10 lists six; these three are
    // cheap-later): pairing (orthogonal exit <-> matched-parameter fan),
    // relax (N Laplacian passes, boundary pinned), angleSnap (snap chords to
    // the patch's dominant axes). TODO(P7-followup).
    double pairing = 0.0;
    int    relax = 0;
    bool   angleSnap = false;

    // ROUND-13 bisect node hygiene: junction pairs closer than hardCollapse
    // ALWAYS fuse; pairs in (hardCollapse..blockLen) fuse with seeded
    // per-pair probability softCollapseP (deterministic randomness — the
    // per-district dial: oldtown ~0.5, residential ~0.9).
    double hardCollapse = 45.0;
    double softCollapseP = 0.8;
};

// A street segment to weld into the host graph. Endpoints on the patch
// boundary land exactly on it (the caller's planarize T-splits the chain).
struct FabricSegment {
    Vec2 a, b;
    Real width = 10.0;
    RoadClass klass = RoadClass::Local;
};

// The N-sided patch: the face polygon plus which of its vertices are true
// CORNERS (arterial junction nodes, degree >= 3 in the host skeleton). The
// chains between consecutive corners are the patch's SIDES.
struct FabricPatch {
    Poly2 boundary;              // face polygon, CCW, curve samples included
    std::vector<int> corners;    // ascending indices into boundary
};

// CHORD fabric (rounds 9-11, the planned-district look). Stations on each
// side at canonical ~blockLen spacing; each street is the straight CHORD
// between its own opposite-side stations, blended toward the Coons iso-curve
// by `conform`. Mismatched side counts: within snapRatio the counts snap
// equal; above it the sparse family runs through and the dense side's extras
// T-TERMINATE at their first crossing with the cross family (an extra that
// finds no crossing is dropped — no dead ends inside a closed face).
// N != 4 patches decompose via side midpoints -> centroid into N quad
// sub-patches (seams are streets); a patch smaller than one block emits
// nothing; a degenerate boundary (repeated vertices, < 2 corners, centroid
// outside) returns empty rather than guessing. Deterministic.
std::vector<FabricSegment> fabricChords(const FabricPatch& patch,
                                        const PatchFabricParams& params);

// BISECT fabric (round 12, the grown oldtown/industrial look): recursive
// near-midpoint bisection — cut position jittered in 0.4..0.6 of the long
// OBB axis, cut direction tilted toward seeded diagonals — recursing until
// faces drop below ~2x block area (variety over correctness). Round-13 node
// hygiene runs at emission: endpoints fuse onto existing junctions per the
// hard/soft thresholds above; a cut whose mandatory fuse would leave the
// patch is dropped whole. Every endpoint lands on the boundary or an
// earlier cut — no dead ends. Deterministic (patch-geometry seeded).
std::vector<FabricSegment> fabricBisect(const Poly2& patch,
                                        const PatchFabricParams& params);

// RING COURT (round 12's "green heart"): the rings+ribs construction with
// the interior rows OMITTED — one perimeter ring at blockDepth plus its
// boundary ribs; the middle stays a single big block (park/campus/court).
std::vector<FabricSegment> fabricCourt(const Poly2& patch,
                                       const PatchFabricParams& params);

// Rings + ribs (the P7 foundation; fabricCourt is this capped at one ring).
// Deterministic; a pure function of (patch, params). Emits nothing for
// patches smaller than one block.
std::vector<FabricSegment> fabricRingsRibs(const Poly2& patch,
                                           const PatchFabricParams& params);

// The ring primitive, exposed for tests: mitered closed-loop offset of the
// patch boundary at `depth`, split into maximal VALID arcs — every sample
// inside the patch and >= depth-eps from the boundary (the medial-validity
// prune that survives concavity, lobe splits, and collapse).
std::vector<std::vector<Vec2>> insetRingArcs(const Poly2& patch, double depth);

}  // namespace engine

#endif
