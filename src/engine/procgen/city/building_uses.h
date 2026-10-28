#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_BUILDING_USES_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_BUILDING_USES_H

// WHAT A BUILDING'S FLOORS ARE FOR (building-uses plan, #117 item 35 follow-on). One answer, read by the interior: the
// building record's parameters as its interior is grown -- the runtime facts no lot cache holds (officeUse, the civic
// rooms, a house) set from its recipe. The interior system, the tests and the censuses all ask here, so they plan
// the floors the game does.

#include "building_records.h"

#include <string>

namespace engine {

// The use of a recipe's upper floors (assets/scripts/building_uses.lua): a floor program's name (office, apartments)
// or a planner's (house, hotel, hospital, civic); "none" for a building with no interior. A recipe not listed takes
// its place type's default. The file is read on first use and again when it changes, so an edit while the game runs
// reaches the next interior planned.
std::string floorUseFor(const std::string& recipe, const std::string& placeType);

// Whether building_uses.lua lists the recipe by name (else its place type's default stands).
bool floorUseListed(const std::string& recipe);

// Bumped each time building_uses.lua is (re)read: an interior grown before a change can be grown again.
int buildingUsesGeneration();

// The interior's runtime facts from the use: officeUse (an office floor, and not homes) and house.
void applyFloorUse(BuildingParams& p, const std::string& recipe, const std::string& placeType);

// The record's parameters with the interior's runtime facts filled in from its recipe and use.
BuildingParams interiorParamsFor(const BuildingRecord& r);

}  // namespace engine

#endif
