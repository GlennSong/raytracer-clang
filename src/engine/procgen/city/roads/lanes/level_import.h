#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LEVEL_IMPORT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LEVEL_IMPORT_H

// A shipped level's road recipe -> a lanelab graph (ADR-0083). Runs the engine's OWN recipe and
// terrain (applyGenerateRecipe, navRoadGraph, terrainHeight), takes the chain spines with their
// classes and widths, and optionally turns the city's perimeter loop into a two-carriageway
// freeway: other streets are cut short of it, chosen streets pass under it as landings, and
// diamond ramps join both carriageways to those landings. Nothing here authors a lane.

#include "engine/procgen/city/polygon.h"   // Vec2
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

// A freeway ROUTE: where a freeway should go, as data. This is the one part of the
// old corridor pipeline worth keeping (Glenn, 2026-09-20: the rest is legacy) — a
// polyline and nothing else, no alignment opinions, no geometry. The lanes builder
// builds the carriageways, the clearance and the ramps from it.
//
// A route is NOT the traced ring: the ring is the city's own outline, which is why
// its redraw only ever reaches a 9-12 m radius on a real city. A route arrives
// already designed, so it needs none of the ring's morphology, and none of its
// "inside" tests either — a route has a left and a right, not an inside.
struct FreewayRoute {
    std::vector<Vec2> points;
    bool closed = false;         // a bypass a level ASKED for, rather than a traced fallback
};

struct ImportOptions {
    // Routes win when present; the traced ring is what happens when a level says
    // nothing, and is on its way out (see docs/road-module-plan.md).
    std::vector<FreewayRoute> routes;
    bool freewayLoop = true;
    int diamonds = 6;                 // landings around the loop
    double gateSpacing = 650;         // min loop distance between landings
    double landingLen = 230;          // how far a landing street continues outside the loop
    double goreOffset = 120;          // loop distance from a landing to its gores
    double footInside = 230, footOutside = 170;   // ramp feet along the landing street, from the crossing
    int freewayLanes = 3; double freewayLaneW = 3.6, medianGap = 6.0, clearance = 8.0;
    // A ramp is its own road, not a freeway lane: wider than a mainline lane (a driver has one lane and a
    // barrier either side, no room to correct) and with the mainline's shoulder, which also sets how far out
    // the parapet sits. Was freewayLaneW / 1.0 m, which drove narrow (Glenn, 2026-09-08).
    double rampLaneW = 4.5, rampShoulder = 2.5;
    double freewayGMax = 0.06, freewayWindow = 200;   // this shelf has real relief: steeper and shorter than a flatland freeway
    double maxRampClimb = 18.0;   // the diamond grows to its climb; 18 m is ~340 m of ramp at the class grade                        // a diamond's ramps can climb this much over their free length
    double freewayRadius = 220.0;      // design radius of the freeway alignment: the ring is redrawn with no curve tighter than this
    bool frontage = true; double frontageSetback = 20.0;   // a collector ring inside the freeway that trimmed streets end on; 20 m leaves room for a ramp lane in the band
    bool turnPockets = true;                          // arterials get a centre gap and left-turn pockets at arterial crossings
    double terrainRes = 5.0, margin = 250.0;
};

struct ImportReport { int chains = 0, loopChains = 0, streets = 0, trimmed = 0, dropped = 0, landings = 0, ramps = 0; double loopLength = 0; std::string notes; };

// Writes <outDir>/<name>_terrain.json (the level ground baked to a grid) and returns the graph.
nlohmann::json graphFromLevel(const std::string& levelPath, const std::string& outDir, const ImportOptions& o, ImportReport& rep);

}  // namespace roads::lanes
}  // namespace engine

#endif
