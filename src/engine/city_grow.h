#ifndef RAYTRACER_ENGINE_CITY_GROW_H
#define RAYTRACER_ENGINE_CITY_GROW_H

// GROWING THE CITY: one function, every host (Glenn, 2026-09-22: "for building the
// city should be one path right?", after 2026-09-21: "I don't know why the editor
// should end up building a procedurally generated city different than any other
// path. There should only be one path.").
//
// lotGrowSetupForLevel (ADR-0084 B) already made the PARAMETERS one derivation.
// What stayed duplicated was the CALL around them — which blocks, which streets a
// door faces, the lane-built city's margin and paving datum, whether geometry is
// wanted at all — written out three times: in LevelLoader::growCityLots, in the
// `lots` bundle producer, and again in the offline tracer's level_scene, which
// re-derived the hub list and the coreness centre by hand and so grew a different
// city than the game whenever a level authored its own districts (ADR-0090).
//
// A host still owns its CACHING (the loader and the producer read and write
// bundles; the offline tracer grows in place). It no longer owns the rules.

#include "engine/lot_grow_setup.h"
#include "engine/procgen/city/city_lots.h"
#include <functional>
#include "engine/procgen/city/polygon.h"
#include "engine/procgen/city/road_network.h"
#include "engine/procgen/terrain_field.h"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace engine {

// Two shapes of city arrive here, and a host fills exactly one side:
//
//   NETS     the lattice: blocks are the faces of the road graph, and the grow
//            samples the nets for clearance (growLotBuildingsOnNets).
//   HOLES    a builder that paved a whole city (RoadProducts): the pavement's
//            holes ARE the blocks, its twin is the streets doors face, and its
//            kerb band is the pavement a door walks out to.
struct CityGrowInputs {
    nlohmann::json citysim = nlohmann::json::object();
    std::string levelDir;
    HeightField padGround;              // what the lot pads grade off (eroded + the road carve)
    HeightField netGround;              // NETS: the ground the roads themselves drape on
    const std::vector<RoadEntity>* nets = nullptr;
    const RoadGraph* freewayROW = nullptr;   // NETS: the lot pass's keep-out
    const std::vector<Poly2>* holes = nullptr;   // HOLES: un-inset, straight from the builder
    const std::vector<Poly2>* blocks = nullptr;  // HOLES, already run through cityBlocksFromHoles
    const std::vector<std::vector<Vec2>>* water = nullptr;   // HOLES: rivers and lakes the blocks stand back from (levelWaterKeepOut)
    const RoadGraph* streets = nullptr;          // HOLES: the graph a door faces
    double pavedSidewalk = 0.0;                  // HOLES: the band that was actually paved
    LotGroundWithFn groundWith;
    double groundMeshCell = 0.0;
    const Vec2* enterableAt = nullptr;   // the spawn building grows enterable (ADR-0080)
    // The ground already carries the streets (a lane city's earthwork field): no block planes or terraces
    // -- the smooth field IS the grade (Glenn: "not a huge fan of the terraced terrain look").
    bool smoothGround = false;
    // The height of the street in front of a building (lanesStreetHeight): its pad takes it (LotParams).
    std::function<bool(Real, Real, Real*)> streetHeight;
    std::function<bool(Real, Real)> nearFreeway;   // lanesNearFreeway (LotParams)
};

// `setupOut` (optional) receives the parameters the grow ran with — the loader
// wants its resolved script paths for the watch list, tests want the rest.
NetLotResult growCity(const CityGrowInputs& in, LotGrowSetup* setupOut = nullptr);

// The style and archetype books this level would grow with, resolved but not parsed —
// a host's file-watch list, on a load that reads its lots from a bundle and grows nothing.
std::vector<std::string> cityGrowScriptFiles(const std::string& levelDir);

// The pavement's holes as city blocks: ONE spelling of the inset and the minimum
// width, which three call sites had each written out with its own constants.
std::vector<Poly2> cityBlocksFromHoles(const std::vector<Poly2>& holes,
                                       const std::vector<std::vector<Vec2>>* water = nullptr);

}  // namespace engine

#endif
