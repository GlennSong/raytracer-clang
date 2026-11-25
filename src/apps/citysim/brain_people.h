#ifndef RAYTRACER_APPS_CITYSIM_BRAIN_PEOPLE_H
#define RAYTRACER_APPS_CITYSIM_BRAIN_PEOPLE_H

// THE AGENT BRAIN'S PEOPLE, TAKEN FROM THE CITY'S OWN HOUSEHOLDS (ADR-0490; one-brain plan, step 1).
//
// The city sim fills the city's real dwellings with households (households.h planHouseholds: singles, couples,
// families... each person an age, a life stage and, for a worker, a job at a real place; CitySim::assignPlaces matches
// each member to one of the sim's agents). A level that names `citysim.brainPeople` has the agent brain (city_brain.h)
// run whole households of them, in a seeded order, until that many people: each household's home is the
// door of its building on the road model, its members are its agents, its cars stand in kerb spaces near home, and a
// worker's job is their own workplace's door. The brain is handed them as a level states residents (CityMarker,
// CityResident, CityHousehold), with the agent who is each resident.
//
// Words: a PLACE'S MARKER is the brain's point beside a street for one of the city's places (its door's frontage,
// as the lot pass recorded it, else the frontage nearest its entrance); named place_<id> after the place.
//
// What people own is people.lua's `people.brain` (BrainPeopleRules): how many cars a household owns (never more than its
// members who may drive) and the travel traits. Their day is their role's (CityResident::roleDay): the brain runs the
// sim's role table for each of them on the sim's clock (city_brain.h, ADR-0491).
//
// EVERY HOUSEHOLD WHOSE HOME HAS A STREET BESIDE ITS DOOR (ADR-0493: the brain runs everyone): one whose door joins no
// walk to the rest of the town is taken all the same, and counted (its people's trips find no way, as they would for
// the old sim's). A household's cars hold free kerb spaces near home, each car naming its space; a car with none is
// not made (the household has fewer), and counted. Left out, and counted: a home with no street and sidewalk beside it.
//
// Same result for the same city and seed: one ProcRng stream, drawn in a fixed order.

#include "../../engine/components.h"   // CityMarker, CityResident, CityHousehold
#include "../../rt_math.h"
#include "places.h"   // PlaceId

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace engine {
struct RoadModel;
struct RoadFrontage;
struct KerbSpace;
}  // namespace engine

namespace citysim {

class CitySim;
class PlaceMap;

// WHAT WAS MADE, as a level states it, the agent who is each resident, and what was left out.
struct BrainPeople {
    std::vector<engine::CityMarker> markers;
    std::vector<engine::CityResident> residents;
    std::vector<engine::CityHousehold> households;
    std::vector<int> agents;   // resident r is the sim's agent agents[r]
    int cars = 0;
    int workers = 0;           // residents with a job away from home
    // left out
    int homesCutOff = 0;       // households taken whose home's door joins walks outside the largest part of them: another
                               // town of an island (its own walks), or a sliver cut off (its people's trips find no way)
    // left out
    int homesUnusable = 0;     // households whose home has no street and sidewalk beside its door
    int carsNoSpace = 0;       // cars with no free kerb space near home
    std::string describe() const;
};

// WHERE A PERSON CAN WALK FROM (the caller's answer): whether people can walk to and from a point beside a street
// (`stand`, the middle of its sidewalk; `f` its frontage) and the rest of the town.
using BrainPlaceUsable = std::function<bool(engine::Vec2 stand, const engine::RoadFrontage& f)>;
// THE KERB SPACES A HOUSEHOLD'S CARS CAN START IN at a home whose frontage is `f`, nearest first on foot (the brain's
// own list, city_brain.h kerbSpacesOffered). A household's cars hold as many of them as they are, unheld by an earlier
// household, so the brain, putting each car in the first free space of its list in the same order, always finds one.
// Empty (no function): the level has no driving, and no household has a car.
using BrainCarSpaces = std::function<std::vector<engine::KerbSpace>(const engine::RoadFrontage& f)>;

// A PLACE'S MARKER, as a level would state it: named placeMarkerName(id); by street, station and kerb when the place's
// door names a street of `model` (Place::frontage, as the lot pass recorded it), else at its entrance. The one way a
// place becomes a marker: here for homes, and in the brain for wherever a day goes (CityBrain, ADR-0491).
engine::CityMarker placeMarkerSpec(const Place& p, PlaceId id, const engine::RoadModel& model);
std::string placeMarkerName(PlaceId id);
// ...and back: the place a marker of that name stands for (kNoPlace: a marker of no place, a lab level's own).
PlaceId placeOfMarkerName(const std::string& name);

// MAKE THE BRAIN'S PEOPLE from `sim`'s households (its plan and agents, after assignPlaces) in `places`, on `model`,
// at most `most` people, seeded by `seed`. False, with the reason in `why`, when people.lua has no `people.brain`.
bool brainPeopleFromHouseholds(const CitySim& sim, const PlaceMap& places, const engine::RoadModel& model, int most,
                               const BrainPlaceUsable& usable, const BrainCarSpaces& spaces, uint32_t seed,
                               BrainPeople& out, std::string* why);

}  // namespace citysim

#endif
