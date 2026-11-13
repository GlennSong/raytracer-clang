#ifndef RAYTRACER_APPS_CITYSIM_TRIP_PLAN_H
#define RAYTRACER_APPS_CITYSIM_TRIP_PLAN_H

// THE TRIP PLANNER (ADR-0415; Glenn: "The agent should decide where to go and how to get there ... Bike, car, walk
// etc. plan the trip"). A person somewhere wants to be at a door. Every way of travelling they could use becomes an
// OPTION: a list of LEGS (walk to the car, drive, walk to the door), each with its route, metres and minutes. The
// option with the fewest WEIGHTED minutes is chosen -- a minute of a way the person minds counts for more -- ties
// going to the earlier way in the table.
//
// The ways of travelling are DATA (assets/scripts/travel.lua, one row each): adding one is a row, plus, where it
// moves the person differently, its execution (city_trip.h). This module names no way of travelling.
//
// Pure: the nav graph and the table in, the plan out. No sim, no world, no Lua (the reader is
// scripting/travel_lua.h), so it is unit-tested headless.

#include "../../engine/ai/nav_graph.h"
#include "../../engine/ai/pathfind.h"

#include <string>
#include <vector>

namespace citysim {

using engine::Real;
using engine::Vec2;

// ONE WAY OF TRAVELLING (a row of travel.lua).
struct TravelMode {
    std::string name;            // "walk", "car"
    bool roads = false;          // its network: the roads (a vehicle's routing) or the footpaths (a walker's)
    Real speed = 1.4;            // m/s, door to door on its network (the table says km/h)
    Real startMinutes = 0;       // fixed minutes at the start of a leg of it (unlocking, pulling out)
    Real endMinutes = 0;         // ...and at its end (finding the space, parking, locking)
    bool ownVehicle = false;     // needs a vehicle the person owns, standing somewhere
    int access = -1;             // the way of travelling to and from that vehicle (a row index); -1: none needed
    Real weight = 1.0;           // how much the person minds a minute of it (1 = a plain minute)
    engine::Vec3 colour{1, 1, 1};   // its colour when a plan is drawn
};

// The whole table: the ways, in order (the order breaks ties), and the distance bands a person picks a destination
// from when told "somewhere near" / "somewhere far" (straight-line metres).
struct TravelCatalog {
    std::vector<TravelMode> modes;
    Real nearMin = 150, nearMax = 600;
    Real farMin = 900, farMax = 4000;
    int find(const std::string& name) const;   // a row by name; -1 when there is none
};

// One leg: a single way of travelling from one point to another.
struct TripLeg {
    int mode = -1;               // row in the catalog
    Vec2 from, to;
    engine::Route route;         // links of the leg's network (empty when from and to share a node)
    int fromNode = -1, toNode = -1;
    Real metres = 0;             // the route plus the bits off the network at each end
    Real minutes = 0;            // metres at the way's speed, plus its start and end minutes
};

// One way of making the whole trip.
struct TripOption {
    int mode = -1;               // the row this option is for
    bool possible = false;
    std::string why;             // why not, when it is not possible ("no car", "no route by road")
    std::vector<TripLeg> legs;
    Real minutes = 0;            // the legs' minutes
    Real weighted = 0;           // ...each times its way's weight: what the choice compares
};

struct TripPlan {
    Vec2 from, to;
    std::vector<TripOption> options;   // one per row of the catalog, in its order
    int chosen = -1;                   // index into options; -1: nothing possible
    std::string reason;                // one line: "walk 14.0 min, car 6.0 min (2.0 walk + 3.0 car + 1.0 walk): car"
    bool valid() const { return chosen >= 0; }
    const TripOption* choice() const { return chosen >= 0 ? &options[static_cast<std::size_t>(chosen)] : nullptr; }
};

// A vehicle the person owns: which way of travelling it is for, where it stands now, and where it would be left at
// the far end (a free space near the destination: the caller knows the parking; the planner does not).
struct OwnedVehicle {
    int mode = -1;
    Vec2 at;
    Vec2 leaveAt;
};

struct TripRequest {
    Vec2 from, to;
    std::vector<OwnedVehicle> vehicles;
    std::vector<int> ruledOut;   // rows not to use this time (a leg of it could not be done: city_trip re-plans)
};

// One leg by `mode` from `from` to `to`: routed on the way's network between the nearest nodes that connect. False
// (leg untouched) when nothing near the two ends routes.
bool planLeg(const engine::NavGraph& nav, const TravelCatalog& cat, int mode, Vec2 from, Vec2 to, TripLeg& out);

// Every option, and the choice.
TripPlan planTrip(const engine::NavGraph& nav, const TravelCatalog& cat, const TripRequest& req);

// "car 6.0 min (2.0 walk + 3.0 car + 1.0 walk)", or "car -- no car" when it is not possible.
std::string describeOption(const TravelCatalog& cat, const TripOption& o);

}  // namespace citysim

#endif
