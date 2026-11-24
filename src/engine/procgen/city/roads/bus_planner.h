#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_BUS_PLANNER_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_BUS_PLANNER_H

// LAYING A TOWN'S BUS LINES (the buses plan, step 2; ~/.claude/plans/buses-on-road-model.md; ADR-0482). The lines it lays
// are written into the road file (lanes_tool buslines --write) as the stated "bus_lines" of ADR-0481, and read from
// there by everything else: nothing lays them at load.
//
// What it does, in plain words:
//   1. HUBS: far-apart junctions a bus can reach, on streets with a sidewalk -- one for about every `hubEvery` metres of
//      such street, between `fewestHubs` and `mostHubs`; the first the junction nearest the middle of the town, each
//      next the one farthest from those already picked (the old network's farthest-point hubs).
//   2. LINES: each a loop through every hub but one, in their order round the town's middle (line k leaves out hub k).
//      Each leg from one hub to the next is the route the bus would drive (the one router, for the bus's class); a
//      street already on a line is made to look slower to the router (each use adds `reusePrice` times its time), so the
//      lines spread over the town instead of all taking the one fast road.
//   3. STOPS: along each loop, one every `spacing` metres or so, never within `junctionClear` of a junction's mouth and
//      only beside a sidewalk; each stated as the street the bus is on there, the station along it and the kerb of the
//      way it is going (the lane it is on runs along the spine: +1; against it: -1).
//   4. CHECKED: every leg between two stops routes for the bus (as Driving::busLineProblems will check it at load);
//      a stop that breaks its line is left out.
// What it reports (the old network's two numbers that mattered): the share of the town's sidewalk streets' length within
// `reach` metres of a stop in a straight line (coverage), and the share a bus drives (street share).

#include "engine/procgen/city/roads/route.h"   // RouteClass, the one router
#include "engine/procgen/city/roads/road_model.h"

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace engine::roads {

struct BusPlanParams {
    Real hubEvery = 2500;       // metres of sidewalk street for each hub
    int fewestHubs = 3, mostHubs = 6;
    int lines = 0;              // 0: one fewer than the hubs, at most 3 [~]
    Real spacing = 250;         // metres between stops along a line (the old network: 180-300 m)
    Real junctionClear = 25;    // a stop this far, at least, from either end of the lane it stands beside
    Real reusePrice = 4;        // each line on a street adds this many times its time for the next line's router
    Real reach = 220;           // coverage: a street point this near a stop is served
    std::string idPrefix = "B"; // lines "B1", "B2"...; stops "B1-01"...
};

struct BusPlan {
    std::vector<ModelBusLine> lines;
    int hubs = 0;
    Real coverage = 0;          // share of the sidewalk streets' length within `reach` of a stop
    Real streetShare = 0;       // share of the sidewalk streets' length a bus drives
    std::vector<std::string> notes;   // what was left out, and why
};

// Lays the lines over a built road model for a bus of class `bus` (its tags, turning circle and top speed as the road
// model's lane level knows them, and whether it may change lanes: the city sim's cars may not yet, so a stop stands only
// where the bus's route is already in the kerb lane, and the hubs are in the part of the roads it can go round in its
// lanes).
BusPlan planBusLines(const RoadModel& model, const RouteClass& bus, const BusPlanParams& params = {});

// The lines as the road file states them (ADR-0481): [{"id", "name", "stops": [{"id", "name", "street", "station",
// "kerb"}]}].
nlohmann::json busLinesJson(const std::vector<ModelBusLine>& lines);

}  // namespace engine::roads

#endif
