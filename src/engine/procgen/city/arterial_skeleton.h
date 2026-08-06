#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ARTERIAL_SKELETON_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ARTERIAL_SKELETON_H

// P8 FOOTPRINT-FIRST ARTERIALS (Glenn's masterplan step 2, the sector-model
// inversion): the city's arterial skeleton is CONSTRUCTED, never grown — the
// footprint polygon is recursively bisected into district-scale cells and
// the cut lines ARE the arterials. Every district is bounded by arterials by
// construction, every junction is a placed station (a cut meeting the rim or
// an earlier cut), spacing is the bisection stop size — the three
// pathologies of space colonization at this tier (reinforced parallel
// pairs, collision junctions, sliver wedges) are structurally impossible.
//
// Cut lines are Glenn's "start with a line, subdivide and shape it into a
// curve": midpoint displacement (the fractal-terrain technique) with the
// displacement biased toward buildable ground and clamped inside the cell,
// then smoothed — meander with terrain awareness at every octave, endpoints
// pinned so junctions stay planned. Cut #1 can be the SPINE: the founding
// road between the two most-opposite rim gates, through the center.

#include "city_footprint.h"
#include "metro.h"          // CityHub
#include "road_network.h"   // RoadGraph, RoadClass

#include <cstdint>
#include <functional>
#include <vector>

namespace engine {

struct SkeletonParams {
    double districtLen = 1500.0;  // bisection stop: target district cell size
    bool   rimRoad     = true;    // v1 always emits the rim (cuts must land on
                                  // roads); false logs a warning and emits it
    bool   spineRoad   = true;    // cut #1 = founding road between the two
                                  // most-opposite gates, through the center
    double sway        = 0.05;    // meander amplitude (fraction of leg length)
    double arteryWidth = 17.0;
    double continuationSnap = 0.35;  // cut endpoints snap to existing
                                     // junctions within this * districtLen
                                     // (gates pull at 2x — crossroads over
                                     // offset-Ts where geometry allows)
    std::uint32_t seed = 5u;
};

// Build one site's skeleton into `Ga`: rim (gates as exact chain vertices) +
// spine + recursive bisection to district cells. Fills `hubsOut` with the
// derived CityHubs (center hub + one per district cell, metro kind cycle) so
// polycentric zoning keeps working without the legacy hub scatter.
// Deterministic: seeded hashes per cut, fixed-order recursion.
void buildFootprintSkeleton(RoadGraph& Ga, const Footprint& f, int siteIndex,
                            int hotspots, int kindBias,
                            const SkeletonParams& params,
                            const std::function<bool(double, double)>& buildable,
                            std::vector<CityHub>& hubsOut);

// A connector road between two sites' gates (or any two points): the same
// line -> midpoint-displaced curve -> smooth construction as a cut, without
// a cell clamp. Replaces the legacy backbone router in footprint mode.
void emitConnectorRoad(RoadGraph& Ga, const Vec2& a, const Vec2& b,
                       double width, RoadClass klass,
                       const std::function<bool(double, double)>& buildable,
                       double sway, std::uint32_t seed);

}  // namespace engine

#endif
