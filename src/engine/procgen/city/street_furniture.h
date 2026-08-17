#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_STREET_FURNITURE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_STREET_FURNITURE_H

#include "../../ai/nav_graph.h"
#include "../../../rt_math.h"
#include <functional>
#include <vector>

namespace engine {

// Build-time street furniture (device: "place the stop lights when we build
// the city instead of during the simulation ... The simulation should use it
// but it shouldn't be responsible for where they are"). The city build plans
// WHERE every signal pole and street lamp stands, keyed to the deterministic
// NavGraph the sim will later derive from the same roads; the sim animates the
// lenses and reacts to the phases but never invents a pole.

// One signalled approach: the pole foot on the near-right junction corner and
// the direction the three-lamp head faces (toward its approaching traffic).
// `link` is the NavGraph link this signal governs.
struct SignalSpot {
    Vec3 base;
    Vec3 face;
    int link = -1;
};

struct StreetFurnitureParams {
    Real lampSpacing = 34.0;       // metres between lamps along one direction
    Real lampVerge = 1.2;          // beyond the kerb, matching ped verge
    Real maxLampRoadWidth = 16.0;  // no lamps on freeway-width carriageways
    Real curbGap = 0.8;            // pole stands this far beyond the kerb
    Real lampHeight = 4.6;         // = street_kit LampParams.height (head Y)
    Real junctionClear = 12.0;     // no lamps this close to a junction corner
};

struct StreetFurniturePlan {
    std::vector<SignalSpot> signals;
    std::vector<Vec3> lampBases;   // pole feet (instance translations)
    std::vector<Vec3> lampHeads;   // bulb positions (night point lights)
};

// Plan every pole from the nav graph: a signal per junction-entering link
// (same criterion SignalController uses, so sim phases and placed poles agree
// one-to-one), lamps marching along each link's right sidewalk — two-way roads
// get both sides for free, one direction per side. `ground` is the road
// surface height (a RoadEntity's heightAt), so every base sits on the deck.
StreetFurniturePlan planStreetFurniture(
    const NavGraph& nav, const std::function<Real(Real, Real)>& ground,
    const StreetFurnitureParams& params = {});

}  // namespace engine

#endif
