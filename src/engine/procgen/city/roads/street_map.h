#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_STREET_MAP_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_STREET_MAP_H

// THE STREET MAP, BUILT FROM THE ROAD MODEL (docs/road-model-design.md step 7, ADR-0425): the graph of nodes and one-way
// links every agent routes, drives and walks on (engine::NavGraph, the structure the city sim already reads), made from
// the road model's planned layer -- what the road file states -- and the heights of the drawn deck. It is a LOOK-UP TABLE
// (design section 4): it holds no fact of its own. Every link carries the street it runs along (or the junction it lies
// inside), its road and the stations along that road's spine at its two ends; every node at a junction carries the
// junction; lanes, class, one-way, width and kerb parking are copied from the street's stated cross-section. It is
// rebuilt from the model at load, never saved, never edited.
//
// Words used here:
//   - a NODE is a place where links meet: a junction's point, a place where one street carries on as another, a ramp's
//     gore on the road it leaves or joins, a dead end, or a point along a street where its centre line bends;
//   - a LINK is one straight piece of a street (or of a stretch of road inside a junction), one way;
//   - a SPREAD junction is one the plan made of several points (it has stretches of road inside it); the road file
//     lists its points (ModelJunction::nodes) and which roads meet at each;
//   - the GORE is where a ramp leaves or meets the road it is anchored to (its host).
//
// WHAT IS JOINED, AND BY WHAT (nothing by a distance or an angle):
//   - a junction with nothing inside it is ONE node, at its stated `at`; every street whose end names it ends there;
//   - a spread junction is a node at each of its stated points; a street ends at the point that holds its arm's road
//     and station; each stretch of road inside it runs between the points on that road, in station order;
//   - a street that carries on as another (no junction: `carriesOn`) shares one node with it;
//   - a ramp's end anchored to a host road is a node on that host at the stated gore station (`hostStation`), which
//     splits the host's street there; the ramp's own street is driven from its gore along itself (`rampStation`);
//   - a street end with none of these is a dead end.
// Two streets that cross with no stated junction (a bridge) share no node. Where the stated facts leave something
// unjoined (an arm no point holds, a stretch inside a junction whose end is at no point, a spread junction in pieces, a
// ramp anchored on no street), it is a NOTE in the report, a finding, never patched.
//
// ALONG A STREET: its centre line (the model's) kept to within kStreetMapTolerance, with a node at least every
// kStreetMapSpan metres -- the granularity the city sim's links had -- and links each way its stated lanes run.
//
// HEIGHTS: links on a freeway or a ramp (a deck class) carry their ends' heights as absolute deck heights, read off the
// drawn deck (the bake's RoadDeckField, one spine per road-file edge, in edge order) at the link end's station along its
// own road; every other link is at the ground (relative height 0) -- as the city sim resolved the old map's heights. The
// sim resolves the heights above the ground once, at load, where it knows the ground.

#include "engine/ai/nav_graph.h"
#include "engine/procgen/city/roads/road_model.h"

#include <string>
#include <vector>

namespace engine {

struct RoadDeckField;

// The centre line kept to within this (metres) at street level; and a node at least every so many metres.
constexpr Real kStreetMapTolerance = 1.5;
constexpr Real kStreetMapSpan = 60.0;

struct StreetMapReport {
    uint32_t nodes = 0, links = 0;
    uint32_t junctions = 0, junctionPoints = 0, carryOns = 0, deadEnds = 0, gores = 0, insideLinks = 0;
    // what the stated facts leave unjoined: a finding to read, never patched
    std::vector<std::string> notes;
    // a fact the map cannot be built without (a spread junction that lists no points): the city gets no street map
    std::vector<std::string> errors;
};

// APPENDS one city's street map to `map` (city `city`: its index in LevelRoadModel::cities, carried on every node and
// link). `deck`: the city's drawn deck (one spine per road-file edge); null: deck links have no heights (a note says so).
// False, with `report.errors`, when the model lacks a fact the map needs; nothing is appended then.
bool appendStreetMap(NavGraph& map, const RoadModel& m, int city, const RoadDeckField* deck, StreetMapReport* report);

// THE MODEL'S WALKWAYS ON THE STREET MAP (ADR-0476), read through the model, never joined by distance: each walkway a
// walk from end to end; an end stated on another walkway is one node with it there (cutting it part way: a `tee`); an
// end stated on a street joins the link of city `city`'s street map along that street holding its station, from the
// pavement on its stated kerb; a dead end, or an end whose street or walkway is not found, joins nothing (`unjoined`).
// Goes with the street map (#306 task 4, piece 10).
NavGraph::FootpathReport appendWalkways(NavGraph& map, const RoadModel& m, int city);

// The street map's links and nodes in words (counts by kind, the notes): what the loader logs.
std::string describeStreetMap(const StreetMapReport& report);

}  // namespace engine

#endif
