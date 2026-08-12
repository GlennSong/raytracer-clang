#ifndef ROADLAB_PAINT_BAKE_H
#define ROADLAB_PAINT_BAKE_H

// Turning a cross-section into the flat boundary array the shader evaluator
// reads (rl_paint.h).
//
// This is the bridge that has to exist for the marking shader to run on a GPU at
// all. The CPU reference asks the CrossSection for its boundaries per fragment;
// a fragment cannot walk a std::vector of strips whose widths are cubics, so
// something has to precompute them. That something has two plausible shapes, and
// this header supports both from one function:
//
//   Vertex attributes — bake at each ring of the road mesh and let the hardware
//   interpolate between them. Cheap, no new resources, and accurate because the
//   boundaries are cubics in s sampled every metre or two. Capped at whatever
//   attribute budget the vertex format has.
//
//   Profile texture — bake into a row per station bucket and texelFetch. No cap,
//   costs a fetch or two, and needs an atlas.
//
// The interpolation the first option relies on is the thing worth testing, and
// `bakeBoundaryStrip` exists so a test can drive exactly what the GPU would see.

#include "junction.h"   // Network holds Junctions by value
#include "network.h"
#include "rl_paint.h"
#include <vector>

namespace roadlab {

// How many PAINTED boundaries a station can carry into the shader.
//
// Measured, not guessed: across every demo the widest cross-section has 17
// boundaries but only nine of them carry paint. Twelve leaves headroom. The
// difference matters — 17 records is 136 floats per station, which settles the
// vertex-attribute question against itself and points at a profile texture; 9 is
// 18 RGBA texels, which is nothing.
constexpr int kMaxBakedBoundaries = 12;

float markStyleCode(MarkStyle style);
float paintColorCode(PaintColor color);

// Fill `out` with the road's PAINTED boundaries at station `s`, ordered
// most-negative t first, and return how many were written.
int bakeBoundaries(const Road& road, double s, RlBoundary* out, int maxCount);

// One bake per `step` metres along the road's active window, as a mesh ring
// would. `stationOut[i]` is the station of row i; each row is `kMaxBakedBoundaries`
// entries wide, padded with RL_MARK_NONE.
//
// The row count is fixed across the strip on purpose: a vertex format cannot
// vary its attribute count per vertex, so the bake has to pad to the worst case
// the same way the GPU would.
struct BoundaryStrip {
    std::vector<double> stations;
    std::vector<RlBoundary> rows;   // stations.size() * kMaxBakedBoundaries
    int stride = kMaxBakedBoundaries;

    // The boundary array at an arbitrary station, interpolated between rows
    // exactly as a rasteriser would interpolate vertex attributes. Returns the
    // count written.
    int sampleInterpolated(double s, RlBoundary* out, int maxCount) const;
};

BoundaryStrip bakeBoundaryStrip(const Road& road, double step);

}  // namespace roadlab

#endif
