#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_WALKWAY_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_WALKWAY_H

// A WALKWAY (ADR-0475) is a path for people on foot that runs beside no street: a park's path, a plaza's, a cut-through
// between two buildings. Whoever draws it (the lot pass, ADR-0476) STATES it, once: its line and where each of its two
// ends lands, looked up when it was drawn. The road model holds it (RoadModel::walkways, given by stateWalkways), and the
// lane level lays it as walks (its kind's, from the path users' table) joined where the ends say -- never by distance.
// A header of its own so the lot pass can state them without the whole road model.
//
// WHERE AN END LANDS: one of
//   - a street's sidewalk: the street's id, the station along its centre line and the kerb (`meets` as a StatedFrontage
//     names a street: the same three facts a door states, road_network.h, ADR-0474);
//   - another walkway: its id and the metres along it (`meets` naming a walkway);
//   - nothing: a dead end, stated (`deadEnd`), so it is no fault that nothing meets it.
// Stating neither, or both, is a fault the lane level reports.

#include "engine/procgen/city/road_network.h"   // StatedFrontage

#include <array>
#include <string>
#include <vector>

namespace engine {

struct WalkwayEnd {
    StatedFrontage meets;
    bool deadEnd = false;
};
struct ModelWalkway {
    std::string id;                  // unique among the model's walkways; what a door on it names (StatedFrontage::walkway)
    std::string kind;                // a kind of path of assets/data/path_users.json ("footway", "park_path"): who may use it
    Real width = 0;                  // metres
    std::vector<Vec2> line;          // its centre line in plan, from its end 0 to its end 1
    std::vector<Real> heights;       // ...each point's height (as many as `line`)
    std::array<WalkwayEnd, 2> ends{};
};

}  // namespace engine

#endif
