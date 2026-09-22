#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_INTERCHANGE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_INTERCHANGE_H

// DIAMONDS: the way on and off a freeway, at the streets it crosses.
//
// Each carriageway drops a ramp into the band beside it and Ts onto a street out past the
// freeway's own edge — the simpler thing a ring's frontage-road diamond could not be, since
// a route has no inside, only a left and a right. The ramp is sized for its CLIMB measured
// to the terminal's ground, not the crossing's (sized against the wrong end, it arrives at
// the street too high and the profile solver drags the street up to meet it: 27% grade and
// a divergent junction solve).
//
// This is the one generator. The level importer builds its routes' diamonds with it, and so
// does the city planner, whose freeway is planned rather than traced (ADR-0092) — a second
// copy would be a second answer to "where can you get on the motorway".

#include "engine/procgen/city/roads/lanes/geom2d.h"
#include "engine/procgen/terrain_field.h"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

// A street a ramp may land on, by the id it has in the scene.
struct RampStreet {
    std::string id;
    std::vector<Vec2> xy;
};

struct DiamondOptions {
    std::string aId, bId;        // the carriageway edge ids: a runs with the route, b against it
    std::string idPrefix = "d0"; // ramp ids are <prefix>_<n>_{a,b}_{off,on}
    double carriage = 7.0;       // centreline -> a carriageway's centreline
    double edgeReach = 12.0;     // centreline -> the outer edge of the freeway's pavement
    int maxDiamonds = 8;
    double spacing = 600;        // least distance between interchanges along the route
    double decel = 80, taperOff = 72, aux = 110, taperOn = 90, approach = 60;
    double clearance = 0;        // the deck's height over the ground at a crossing
    double window = 200, gMax = 0.06;   // the freeway's own profile, for the climb
    HeightField ground;          // empty: a flat city, every climb is zero
};

struct DiamondResult {
    std::vector<nlohmann::json> ramps;
    int built = 0;
    int candidates = 0, rejectedOblique = 0, rejectedSpacing = 0, rejectedTerminal = 0, rejectedRoom = 0;
    double squarestRejected = 0;
};

// `route` is the freeway's centreline (resampled); `streets` are what its ramps may land on.
DiamondResult diamondRamps(const std::vector<Vec2>& route, const std::vector<RampStreet>& streets,
                           const DiamondOptions& o);

}  // namespace roads::lanes
}  // namespace engine

#endif
