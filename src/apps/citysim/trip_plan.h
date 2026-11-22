#ifndef RAYTRACER_APPS_CITYSIM_TRIP_PLAN_H
#define RAYTRACER_APPS_CITYSIM_TRIP_PLAN_H

// THE TRIP PLANNER (ADR-0415; Glenn: "The agent should decide where to go and how to get there ... Bike, car, walk
// etc. plan the trip"). A person somewhere wants to be at a door. Every way of travelling they could use becomes an
// OPTION: a list of LEGS (walk to the car, drive, walk to the door), each with its route, metres and minutes. The
// option with the fewest WEIGHTED minutes is chosen -- a minute of a way the person minds counts for more -- ties
// going to the earlier way in the table.
//
// The ways of travelling are DATA (assets/scripts/travel.lua, one row each): adding one is a row, plus, where it
// moves the person differently, its execution (the agent brain's skills, city_brain.h). This module names no way of
// travelling.
//
// Pure: the road model, the walker and the table in, the plan out. No sim, no world, no Lua (the reader is
// scripting/travel_lua.h), so it is unit-tested headless.
//
// THE NETWORK (#306, the street map's deletion): every leg is routed over the road model's walks by the one router
// (findWalkRoute) -- the sidewalks run along every street a person drives too. A leg on the roads is timed by the
// class of the street each walk runs along (classSpeed), a leg on foot at the way's speed. (So a drive is priced on
// the streets that have sidewalks: no freeway. The simplest port; the driving's own lane router is the one to price
// a drive by, later.)

#include "../../engine/procgen/city/road_network.h"   // RoadClass, classSpeed
#include "../../engine/procgen/city/roads/route.h"     // findWalkRoute, WalkRoute, RouteWalker

#include <cstdint>
#include <string>
#include <vector>

namespace driving {
class Driving;
}

namespace citysim {

using engine::Real;
using engine::Vec2;

// THE MOST LEGS A PLAN HOLDS (agent-brain-design.md section 12): a way of making the trip that would need more is
// refused and the next cheapest is taken, so a plan can never grow without end.
constexpr int kMaxPlanLegs = 8;
// THE MOST TRAITS a person can be dealt (TravelCatalog::traits): one bit each of a byte in a person's record.
constexpr int kMaxTraits = 8;

// ONE WAY OF TRAVELLING (a row of travel.lua).
struct TravelMode {
    std::string name;            // "walk", "car"
    bool roads = false;          // its network: the roads (a vehicle's routing) or the footpaths (a walker's)...
    bool bus = false;            // ...or the bus lines (ADR-0484): walked to a stop, waited for, ridden round the line
    // WHO HAS IT (travel.lua `requires`): the trait a person must have for this row to be priced for them
    // ("has_board"), and its place in TravelCatalog::traits. Empty and -1: anyone.
    std::string needs;
    int needsTrait = -1;
    // THE LONGEST LEG IT SUITS, in minutes (travel.lua `longest_leg_minutes`): a leg of it that would take longer is
    // not made this way. 0: no limit.
    Real longestLegMinutes = 0;
    // What a person is doing when a leg of it begins (travel.lua `begins_in`: a state of brain.lua). The planner does
    // not read it; the agent brain does (brain/brain_table.h).
    std::string beginsIn;
    // m/s, door to door, on the FOOTPATHS (the table says km/h). A way on the roads has none: its minutes are the
    // driving router's own time for the route (Driving::routeBetween: each lane and turn at its free-flow speed), the
    // time it chose the route by.
    Real speed = 1.4;
    Real startMinutes = 0;       // fixed minutes at the start of a leg of it (unlocking, pulling out)
    Real endMinutes = 0;         // ...and at its end (finding the space, parking, locking)
    // THE LONGEST A LEG OF IT WAITS TO SET OFF, in minutes (travel.lua `set_off_wait_minutes`): a car ready to leave
    // its space waits for the lane beside it to be clear; after this long it is given up as blocked and the person
    // plans again. The planner does not read it; the agent brain does. 0: it waits for as long as it takes.
    Real setOffWaitMinutes = 0;
    bool ownVehicle = false;     // needs a vehicle the person owns, standing somewhere
    // The way of travelling to and from that vehicle, WHEN THE TABLE FIXES ONE (a row index). -1, the usual: it is
    // reached and left by the cheapest way on foot the person has (walking, or a board for someone who has one).
    int access = -1;
    Real weight = 1.0;           // how much the person minds a minute of it (1 = a plain minute)
    engine::Vec3 colour{1, 1, 1};   // its colour when a plan is drawn
};

// A KIND OF ROAD a drive is told apart by (travel.lua `road_kinds`): the road classes it covers, what the plan calls
// it ("freeway"), and the colour its part of the drawn line takes (none: the way's own). A kind counted rather than
// measured (`count`: ramps) is said as how many separate stretches of it there are ("2 ramps").
struct RoadKind {
    std::string name, one;                  // "ramps", and one of them: "ramp"
    std::vector<engine::RoadClass> classes;
    bool hasColour = false;
    engine::Vec3 colour{1, 1, 1};
    bool count = false;
};

// The whole table: the ways, in order (the order breaks ties), the kinds of road, and the distance bands a person
// picks a destination from when told "somewhere near" / "somewhere far" (straight-line metres).
struct TravelCatalog {
    std::vector<TravelMode> modes;
    // THE TRAITS a way can require (travel.lua `traits`), in the table's order: a person's traits are one bit each, bit
    // i for traits[i] (TripRequest::traits). At most kMaxTraits.
    std::vector<std::string> traits;
    int trait(const std::string& name) const;   // its place in `traits`; -1 when it is not listed
    std::vector<RoadKind> roadKinds;
    int roadKindOf(engine::RoadClass k) const;   // the kind a class belongs to; -1 when none lists it
    Real nearMin = 150, nearMax = 600;
    Real farMin = 900, farMax = 4000;
    int find(const std::string& name) const;   // a row by name; -1 when there is none
};

// WHAT A LEG ENDS AT, so a reader that keeps places as handles (the agent brain) knows which handle each leg's end is
// without comparing points: the door the trip is for, the vehicle where it stands, or where the vehicle is left.
enum class LegEnd : uint8_t { Door, Vehicle, VehicleLeft, Stop };   // Stop: a bus stop (its line and index on the leg)

// One leg: a single way of travelling from one point to another.
struct TripLeg {
    int mode = -1;               // row in the catalog
    LegEnd end = LegEnd::Door;
    Vec2 from, to;
    engine::roads::WalkRoute route;   // the walks of a leg on foot (not found when from and to join the walks at one
                                      // point, nor for a leg on the roads)
    Real metres = 0;             // the route (on foot, plus the bits off the walks at each end)
    // On foot: metres at the way's speed. On the roads: the driving router's time for the route, kerb lane to kerb lane
    // (into and out of a space is the way's start and end minutes). Plus the way's start and end minutes either way.
    Real minutes = 0;
    // ON THE ROADS: metres on each of the catalog's road kinds (roadKinds' order), and how many separate stretches of
    // each
    std::vector<Real> kindMetres;
    std::vector<int> kindStretches;
    // A LEG THAT ENDS AT A BUS STOP (LegEnd::Stop; ADR-0484): the line and the stop's index in it. A bus leg also names
    // the stop it is boarded at (`busFrom`).
    int busLine = -1, busTo = -1, busFrom = -1;
};

// One way of making the whole trip.
struct TripOption {
    int mode = -1;               // the row this option is for
    bool possible = false;
    // False: the person does not have what this way requires (a board), so it was never priced for them and the
    // plan's reason leaves it out.
    bool offered = true;
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
// A HOUSEHOLD'S CAR is shared, and whoever reaches it first takes it (agent-brain-design.md section 12): the caller says
// where it stands NOW and whether it is `free` at this moment. A vehicle that is not free is not priced.
// A VEHICLE IN HAND (`inHand`, ADR-0442): the person is in it now, on the road at `at` (a driver told to be somewhere
// else). Then the trip is made in it and no other way is priced: nobody leaves a car in a lane to walk. Its option has
// no leg to reach it: drive from `at` to `leaveAt`, then on foot to the door. The drive's minutes still count the
// way's start minutes (a known over-count: the record does not say a leg began part-way).
struct OwnedVehicle {
    int mode = -1;
    Vec2 at;
    Vec2 leaveAt;
    bool free = true;
    bool inHand = false;
    uint32_t vehicleClass = 0;   // the driving's class it is driven as (its route is found for that class)
    // IN HAND ON THE ROAD: where it is on its lane (its drive is priced from there, not from the kerb lane beside `at`)
    bool onLane = false;
    engine::PathPoint lane;
};

struct TripRequest {
    Vec2 from, to;
    std::vector<OwnedVehicle> vehicles;
    std::vector<int> ruledOut;   // rows not to use this time (a leg of it could not be done: the caller re-plans)
    uint32_t traits = 0;         // what the person has: bit i for TravelCatalog::traits[i]
    int maxLegs = kMaxPlanLegs;  // a way needing more legs than this is refused (never more than kMaxPlanLegs)
};

// A PLAN IN FIXED FORM, for a reader that stores it per person with no heap (the agent brain's record): each leg's way,
// what it ends at and its planned whole seconds; no route (a route is found when its leg begins and belongs to whatever
// carries the leg out).
struct FixedLeg {
    uint8_t way = 0;             // row in the catalog
    LegEnd end = LegEnd::Door;
    uint16_t seconds = 0;        // the leg's minutes, in whole seconds
    uint8_t busLine = 0;         // LegEnd::Stop: the line ...
    uint16_t busStop = 0;        // ... and the stop the leg ends at
};
struct FixedLegs {
    FixedLeg legs[kMaxPlanLegs];
    uint8_t count = 0;
};
// The longest leg the fixed form holds, in seconds (its 16 bits: a little over 18 hours).
constexpr long kMaxLegSeconds = 65535;
// `o` in fixed form. Refused (false, `err` says why, naming the way's row of travel.lua) when `o` is not possible, has
// more than kMaxPlanLegs legs, or holds a leg longer than kMaxLegSeconds.
bool fixedLegs(const TravelCatalog& cat, const TripOption& o, FixedLegs& out, std::string* err);

// THE BUS LINES AS THE PLANNER PRICES THEM (ADR-0484): for each line of the road model, each stop's waiting place (where
// it joins the walks, RoadModel::busStopWalk) and the seconds from it to the next stop (the bus's route there at the
// free-flow speed, and its wait at a stop). Made by the host from the road model, the driving and the bus service.
struct BusTimetable {
    struct Stop {
        Vec2 wait;
        Real toNextSeconds = 0;
    };
    std::vector<std::vector<Stop>> lines;
    Real walkReach = 800;   // a stop is considered only this far (straight line) from where a trip begins or ends
};

// THE BEST BUS RIDE between two points (ADR-0484): the line and the two stops (each within the timetable's walk reach of
// its end, in a straight line) whose estimate is least -- the two walks in a straight line at `walkPace`, and the ride
// round the line by the timetable from the first stop to the second. The wait is not in it (the caller's). What the trip
// planner and the sim's own walkers choose a bus by. Not found: no line has a stop near both ends.
struct BusRide {
    int line = -1, board = -1, alight = -1;
    Real rideSeconds = 0;   // the timetable's, from `board` to `alight`
    Real estimate = 0;      // the two walks and the ride
    bool found() const { return line >= 0; }
};
BusRide bestBusRide(const BusTimetable& tt, Vec2 from, Vec2 to, Real walkPace);

// The network a plan is routed on: the road model, the walker on its walks (route.h bindWalker), the driving, whose
// router prices a leg on the roads (none: no leg on the roads can be made), and the buses' timetable (none: no bus).
struct TripNetwork {
    const engine::RoadModel* model = nullptr;
    const engine::roads::RouteWalker* walker = nullptr;
    const driving::Driving* driving = nullptr;
    const BusTimetable* buses = nullptr;
    bool valid() const;
};

// One leg by `mode` from `from` to `to`. ON FOOT: each end joined to the walks by position (RoadModel::walkJoinNear),
// routed between them by the walker. ON THE ROADS: each end at the kerb lane beside it (frontageAt, kerbLaneBeside),
// routed between them by the driving's router for a car of `vehicleClass` (Driving::routeBetween): its lanes, its
// turns and its free-flow time, freeways included; a car already on the road from where it is (`fromLane`). False (leg
// untouched) when an end joins nothing or no route leads there.
bool planLeg(const TripNetwork& net, const TravelCatalog& cat, int mode, Vec2 from, Vec2 to, TripLeg& out,
             uint32_t vehicleClass = 0, const engine::PathPoint* fromLane = nullptr);

// Every option, and the choice.
TripPlan planTrip(const TripNetwork& net, const TravelCatalog& cat, const TripRequest& req);

// "car 6.0 min (2.0 walk + 3.0 car + 1.0 walk)", or "car -- no car" when it is not possible.
std::string describeOption(const TravelCatalog& cat, const TripOption& o);

// A road leg's roads: "420 m streets, 1,350 m freeway, 2 ramps" (empty for a leg on foot, or with no kinds listed).
std::string describeRoads(const TravelCatalog& cat, const TripLeg& leg);
// Metres on the road kind named `name` in a leg (0 when the catalog has no such kind).
Real kindMetres(const TravelCatalog& cat, const TripLeg& leg, const std::string& name);
// Whole metres with the thousands marked: "1,350 m".
std::string metresText(Real m);

}  // namespace citysim

#endif
