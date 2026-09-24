#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LOTS_PRODUCER_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LOTS_PRODUCER_H

// The lot pass as the second bundle producer (ADR-0084, milestone B). It reads the city producer's products
// out of the bundle being written — the LAST lanelab entity's conformed ground and pavement holes, built a
// moment ago or copied forward — grows the blocks' lots and buildings exactly as LevelLoader::growCityLots
// does (the same lotGrowSetupForLevel, the same growLotBuildings call) and writes the NetLotResult under
// `lots/` through the lot codecs. The loader reads it back and runs everything after the grow unchanged.
//
// Identity: the city producer's key (the blocks and the ground come from it), the level's citysim block,
// the authored spawn (the enterable building), the style and archetype books' bytes, sizeof(Real),
// kLotsFormatVersion and kLotsBuildTag — bumped by hand whenever the grow's output changes.
//
// Applies to a lab level only: a lanelab entity, citysim.buildLots or planOnly, no terrain block (a terrain
// level grows its lots in the terrain pre-pass with its own ground samplers) and no shape:"road" entity
// (their hubs would zone the lots).

#include "engine/bundle/bake.h"
#include "engine/bundle/codecs.h"
#include "engine/procgen/city/city_lots.h"
#include "engine/procgen/city/roads/lanes/terrain_recipe.h"   // HeightGrid
#include "engine/procgen/city/road_network.h"
#include "engine/procgen/city/roads/lanes/geom2d.h"

#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

constexpr const char* kLotsProducerName = "lots";
constexpr const char* kLotsSectionPrefix = "lots/";
extern const char* const kLotsBuildTag;

void registerLotsProducer();   // idempotent

// The city products the lot pass reads: the last lanelab entity's (its blocks are the ones the loader grows on).
struct LotsCityInputs {
    bool hasTerrain = false;
    bundle::HeightGridBlob ground;
    std::vector<Ring> holes;   // pavement holes (they stop at the back of the drawn sidewalk); cityBlocksFromHoles at grow time
    // The streets a door faces and the pavement it walks out to, from the same city products.
    // The loader hands both to the grow; baking without them made the BAKED city's doors differ
    // from the same city grown in place — the cache deciding what the buildings look like.
    RoadGraph nav;
    double pavedSidewalk = 0.0;
};

// ONE derivation: what the producer runs, exposed for tests. `report` (optional) receives the counts.
NetLotResult growLotsForLevel(const bundle::LevelInputs& in, const LotsCityInputs& city, nlohmann::json* report = nullptr);

// THE ERODED BASE A LANE-BUILT CITY'S TERRAIN READS (ADR-0095): the city's conformed grid
// inside its extent, blended over its last 60 m into the level's own terrain (which keeps
// its own eroded base, `levelEroded`) outside -- or the grid's edge value would smear over
// the whole world. The loader draws the ground from it, the lot pass and the lots producer
// grow on it: one definition, so a baked city stands on the loader's ground.
std::shared_ptr<const std::function<double(double, double)>> laneErodedBase(
    const nlohmann::json& root, std::shared_ptr<const HeightGrid> grid,
    std::shared_ptr<const std::function<double(double, double)>> levelEroded);

}  // namespace roads::lanes
}  // namespace engine

#endif
