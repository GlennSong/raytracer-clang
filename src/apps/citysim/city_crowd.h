#ifndef RAYTRACER_APPS_CITYSIM_CITY_CROWD_H
#define RAYTRACER_APPS_CITYSIM_CITY_CROWD_H

// A CROWD OF RESIDENTS MADE FROM THE ROADS (ADR-0446; docs/two-drivers-meeting-design.md section 12, sitting 2).
//
// A level that names a crowd (citysim.crowd = "crowd.lua") gets its residents, their households and cars, their days
// and the places they go between, all made here from that data and the road model, before the agent brain starts
// (city_brain.h). What is made is what a hand-written level states: markers (CityMarker), residents (CityResident),
// households (CityHousehold); the brain takes them as it takes brain_lab's, so nothing downstream knows they were made.
//
// Words: a PLACE is a point beside a street where a person can be (a marker of the brain); its KIND (home, work, a
// shop...) is a name from the data, which says how many places of each there are and which kind households live at.
// A DAY is a round of stops, each a kind of place (or the person's own home) and minutes spent there.
//
// HOW THE PLACES ARE SPREAD: every street with a sidewalk that people can walk to (CrowdUsable, asked at its middle),
// end to end, as one long line (in the road model's street order); the line cut into as many equal pieces as there are places, and one place a seeded way along each piece, on
// a seeded side of the road, kept `keepFromEnds` from either end of its street (a junction's mouth). So places cover
// the whole network in proportion to its streets' length, never in a heap. The kinds are then dealt to the places in a
// seeded order. The road model is asked for each place (frontageOn, sidewalkBeside); nothing is worked out from the
// road's geometry here.
//
// Same result for the same data and seed, on every host: one ProcRng stream, drawn in a fixed order.

#include "../../engine/components.h"   // CityMarker, CityResident, CityHousehold
#include "../../rt_math.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace engine {
struct RoadModel;
}

namespace citysim {

using engine::Real;

// A choice with a share (the shares of a list need not add up to 1: each is its share of their sum).
struct CrowdShare {
    int value = 0;
    Real share = 0;
};

struct CrowdPlaceKind {
    std::string kind;
    int count = 0;        // how many places of it (a kind households live at: one per household, `count` unused)
    bool homes = false;   // households live at places of this kind
    bool parking = false; // cars are parked near places of this kind (a home's always are: its household's start there)
};

struct CrowdStop {
    std::string kind;     // a kind of place (a seeded one of them, chosen once per person)...
    bool home = false;    // ...or the person's own home
    Real minutesLo = 0, minutesHi = 0;   // minutes there, seeded in this band per person
    Real beyond = 0;      // a place of the kind at least this far from home in a straight line (metres; none so far:
                          // the farthest); 0: any of them
};

struct CrowdDay {
    Real share = 0;
    std::vector<CrowdStop> stops;
};

struct CrowdTrait {
    std::string name;     // a trait of travel.lua
    Real share = 0;       // of residents who have it
};

// THE DATA (assets/scripts/crowd.lua; scripting/crowd_lua.h reads it).
struct CrowdSpec {
    int residents = 0;
    std::string intent;                      // a row of brain.lua's `intents`
    std::vector<CrowdPlaceKind> places;
    Real keepFromEnds = 0;                   // metres a place keeps from either end of its street
    std::vector<CrowdShare> householdSizes;  // value: members
    std::vector<CrowdShare> carsPerHousehold;   // value: cars (never more than its members)
    std::vector<CrowdTrait> traits;
    std::vector<CrowdDay> days;
};

// WHAT WAS MADE, as a level states it, and a count of each kind of place.
struct CrowdMade {
    std::vector<engine::CityMarker> markers;
    std::vector<engine::CityResident> residents;
    std::vector<engine::CityHousehold> households;
    std::vector<std::pair<std::string, int>> placesOfKind;
    int cars = 0;
};

// WHERE A PERSON CAN BE: the caller's answer for a point beside a street (the middle of its sidewalk), whether people
// can walk to it and away from it (the city's walking network, which the road model does not know), and when `cars`
// (a home: households' cars start beside it) whether cars can be parked near it and driven away. A street whose middle
// it refuses (without cars) has no place on it; a place it refuses is drawn again along its piece, then along the rest
// of the line, and is left out when none is found.
using CrowdUsable = std::function<bool(engine::Vec2, bool cars)>;

// MAKE THE CROWD on `model`'s streets from `spec`, seeded by `seed`. False, with the reason in `why`, when the data
// cannot be met (no kind households live at, no street to put places on, a day naming a kind with no places).
bool makeCrowd(const CrowdSpec& spec, const engine::RoadModel& model, const CrowdUsable& usable, uint32_t seed, CrowdMade& out,
               std::string* why);

}  // namespace citysim

#endif
