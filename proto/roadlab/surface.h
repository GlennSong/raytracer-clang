#ifndef ROADLAB_SURFACE_H
#define ROADLAB_SURFACE_H

// THE SHADER. This is a CPU reference implementation of a fragment shader; it
// is written to be portable to GLSL/MSL almost line for line, which is why it
// takes a filter width instead of using derivatives and why it avoids anything
// the GPU cannot do per-fragment.
//
// The mesh under it is dumb: a ribbon whose vertices carry (s, t) in METRES.
// Everything visible is computed here from those two numbers plus the road's
// own parametric data:
//
//   * the material comes from which strip t falls in;
//   * lane markings are analytic SDFs about the boundary offsets t_i(s), so a
//     stripe automatically follows a lane that is tapering out — no authoring,
//     no separate marking mesh, no z-fighting, and it stays crisp at any
//     distance;
//   * glyphs (arrows, stop bars, zebras, chevrons, text) are stroke SDFs placed
//     in (s, t), so they are just more of the same;
//   * wear, wheel polish, patching and joints are functions of t relative to the
//     lane centre and of s along the road.
//
// The one thing an SDF cannot do is mip, so `filterWidth` (the fragment's
// footprint in metres) both antialiases the edges and, past the point where a
// stripe is thinner than a pixel, fades it toward its average contribution
// instead of letting it alias.

#include "network.h"
#include "profile.h"
#include <vector>

namespace roadlab {

enum class GlyphKind : uint8_t {
    ArrowThrough,
    ArrowLeft,
    ArrowRight,
    ArrowThroughLeft,
    ArrowThroughRight,
    ArrowMergeLeft,     // the lane-drop merge arrow
    ArrowMergeRight,
    StopBar,
    YieldTeeth,
    CrosswalkZebra,
    CrosswalkLadder,
    ChevronGore,
    ParkingTee,
    BikeSymbol,
    HovDiamond,
    KeepClearGrid,
    Text,               // `text` field, stroke font
};

struct Decal {
    GlyphKind kind = GlyphKind::ArrowThrough;
    double s = 0, t = 0;        // centre, in the road's own frame
    double along = 4.0;         // extent along s (m)
    double across = 2.0;        // extent along t (m)
    double rotate = 0.0;        // extra rotation in the (s,t) plane, radians
    double wear = 0.0;
    PaintColor color = PaintColor::White;
    char text[10] = {0};        // for GlyphKind::Text
    bool flip = false;          // face down-station (for -dir lanes)
};

// Per-road paint. Kept out of Road on purpose: geometry and topology are the
// truth, paint is generated from them by rules (see props.h) and can be thrown
// away and rebuilt.
struct RoadPaint {
    std::vector<Decal> decals;
    double maxAlong = 0;   // largest decal extent, for the query window

    // Sort by station and record the widest decal so the shader can binary
    // search a window instead of walking every decal per fragment.
    void finalize();
    // Index of the first decal that could touch station s.
    size_t lowerBound(double s) const;
};

struct ShadeParams {
    double wear = 0.30;         // global paint aging, 0 = fresh
    double grime = 0.35;        // asphalt discoloration and patching
    uint32_t seed = 7;
    bool markings = true;
    bool decals = true;
    bool micro = true;          // aggregate / joints / rumble bumps
    int debugMode = 0;          // 0 off, 1 strip false-colour, 2 lane ids, 3 s/t grid
};

struct SurfaceSample {
    Vec3 albedo{0.06, 0.06, 0.065};
    double roughness = 0.92;
    double metallic = 0.0;
    double bump = 0.0;          // micro height in metres, for normal perturbation
    bool paint = false;         // this fragment is painted (used by the debug view)
};

// The fragment entry point. `filterWidth` is the fragment footprint in metres
// along t (the renderer passes max(ds, dt) of the pixel).
SurfaceSample shadeRoadSurface(const Road& road, const RoadPaint* paint, double s, double t,
                               double filterWidth, const ShadeParams& params);

// Just the micro-displacement, so the caller can difference it into a normal
// without paying for the full shade three times.
double surfaceMicroHeight(const Road& road, double s, double t, const ShadeParams& params);

// Junction pads are shaded in their own planar frame rather than in (s,t) —
// there is no meaningful arc length inside an intersection. Same machinery:
// material plus decals, just in junction-local coordinates.
SurfaceSample shadeJunctionPad(const Junction& j, const std::vector<Decal>& decals, Vec2 planPoint,
                               double filterWidth, const ShadeParams& params);

// --- rule-driven paint ----------------------------------------------------

// Generate the decals a road implies: lane-use arrows before junctions, merge
// arrows at lane drops, gore chevrons, parking bay tees, bike and HOV symbols,
// stop bars and zebras at junction arms, speed numerals where the limit changes.
// Nothing here is authored — it all follows from the cross-section, the lane
// links and the junction control, which is what keeps the paint and the rules
// telling the same story.
void generateRoadPaint(const Network& net, std::vector<RoadPaint>& paint);

// Colour of a paint stripe.
Vec3 paintAlbedo(PaintColor c);

}  // namespace roadlab

#endif
