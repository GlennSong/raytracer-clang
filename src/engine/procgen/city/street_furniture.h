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
    // Sidewalk band width (RoadLook::sidewalk): the junction PAD spans
    // verge-to-verge, i.e. carriageway half-width + THIS, so a pole placed
    // without it stands on the pad's asphalt — the "stoplight in the middle
    // of the road". The kerb-corner pole must back off past the pad mouth —
    // and, at an ACUTE corner, further still, until it clears the
    // neighbouring arm's carriageway too (the metro's organic grid put a
    // pole inside the neighbour's ribbon at every corner under ~75 degrees).
    Real sidewalkWidth = 3.5;
    // 26 m between lamps: close enough that the 34 m pools OVERLAP rather
    // than leaving a dark gap between every pair (device: "it's pretty dark
    // in the city at night").
    Real lampSpacing = 26.0;       // metres between lamps along one direction
    Real lampVerge = 1.2;          // beyond the kerb, matching ped verge
    // The gate that left every ARTERIAL dark. Its comment said "no lamps on
    // freeway-width carriageways", but freeways and ramps are already
    // excluded by road CLASS above — so all this actually did was reject
    // arterials, whose carriageway is 17 m in the metro recipe against this
    // 16 m bar. The main roads of the city had no street lighting at all.
    // Kept as a real guard for anything genuinely wider than a city street.
    Real maxLampRoadWidth = 26.0;
    Real curbGap = 0.8;            // pole stands this far beyond the kerb
    Real lampHeight = 4.6;         // = street_kit LampParams.height (head Y)
    // 10 m, not 12: junction corners are exactly where a pedestrian wants
    // light. Still clear of the signal poles, which stand ~9.6 m out.
    Real junctionClear = 10.0;     // no lamps this close to a junction corner
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
