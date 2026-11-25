#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LOTS_PRODUCER_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LOTS_PRODUCER_H

// The lot pass as the second bundle producer (ADR-0084, milestone B). It reads the city producer's products
// out of the bundle being written — the LAST lanelab entity's conformed ground and pavement holes, built a
// moment ago or copied forward — grows the blocks' lots and buildings exactly as LevelLoader::growCityLots
// does (the same lotGrowSetupForLevel, the same growLotBuildings call) and writes the NetLotResult under
// `lots/` through the lot codecs. The loader reads it back and runs everything after the grow unchanged.
//
// Identity: the city producer's key (the blocks and the ground come from it), the citysim fields the lot pass reads
// (lot_citysim.h kLotCitysimFields, read through the same LotCitysim the grow reads; ADR-0457),
// the authored spawn (the enterable building), the style and archetype books' bytes, sizeof(Real),
// kLotsFormatVersion and kLotsBuildTag — bumped by hand whenever the grow's output changes.
//
// Applies to a lab level only: a lanelab entity, citysim.buildLots or planOnly, no terrain block (a terrain
// level grows its lots in the terrain pre-pass with its own ground samplers) and no shape:"road" entity
// (their hubs would zone the lots).

#include "engine/bundle/bake.h"
#include "engine/bundle/codecs.h"
#include "engine/procgen/city/city_lots.h"
#include "engine/procgen/city/lot_cache.h"   // TileCellWriter: the lots written tile by tile
#include "engine/procgen/city/roads/lanes/terrain_recipe.h"   // HeightGrid
#include "engine/procgen/city/road_network.h"
#include "engine/procgen/city/roads/lanes/geom2d.h"
#include "engine/procgen/city/roads/road_model.h"   // the road model the lot pass's roads are read from

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
    // The pavement a door walks out to, from the same city products. (The streets a door faces are not a city
    // product: the grow reads them from the level's one road model, lotRoadInputs below.)
    double pavedSidewalk = 0.0;
    // The drawn road decks: the earthwork field is pinned to them (lanesEarthworkField), so the lots grow
    // on the ground the terrain will show -- the streets setting it, not natural ground under a skirt.
    RoadDeckField deck;
    std::vector<RailTrack> rails;   // the rail lines (none: no track) -- buildings stand clear of them (lanesTrackKeepOut)
};

// ONE derivation: what the producer runs, exposed for tests. `report` (optional) receives the counts.
// `cells` (the producer's): the lots grown tile by tile into it, unchanged tiles kept from the last bake (ADR-0504);
// none: every part in the result
NetLotResult growLotsForLevel(const bundle::LevelInputs& in, const LotsCityInputs& city, nlohmann::json* report = nullptr,
                              ::engine::lotcache::TileCellWriter* cells = nullptr);
// The same inputs from a city's products as read from its bundle (or built a moment ago).
struct CityProducts;
LotsCityInputs lotsCityInputs(const CityProducts& p);

// THE ROADS AS THE LOT PASS READS THEM (city_lots.h LotRoadInputs), from the one road description: each of the road
// model's streets, and each stretch of road inside a junction, by its id, with its centre line, its carriageway's half
// width (travel lanes, median gap and shoulder), its sidewalk and its class, copied from the model; and the drawn
// deck. Tracks and footways are left out (no building fronts one). A look-up: made on request, never saved.
LotRoadInputs lotRoadInputs(const RoadModel& model, const RoadDeckField& deck);
// ...for a level: of its last lane-built city (the one whose blocks the lots grow on), out of the level's PLANNED road
// model (road_model_cache.h levelPlannedRoadModels: the lots grow before the built model is assembled, ADR-0476). No
// city, or one whose road file could not be read: no roads.
LotRoadInputs lotRoadInputs(const std::vector<std::shared_ptr<RoadModel>>& cities, const RoadDeckField& deck);
// THE LOTS' PATHS HANDED TO THE ROAD MODEL (ADR-0476): every lot's walkways, in the lots' order, stated on the level's
// built model (road_model.h stateWalkways, the one place walkways are supplied), once the model is assembled. Returns
// what did not land, with why. The loader calls it on the model it publishes; the bake's lot producer on the model it
// obtains, to say the faults.
std::vector<WalkwayProblem> stateLotWalkways(RoadModel& built, const std::vector<LotBuilding>& lots);
// ...for a level: on the model of THE CITY THE LOTS GROW ON (lotsCity), the same one lotRoadInputs(cities, ...) reads
// their streets from, so every street end names a street of the model it is stated on. That model not built (a later
// city's planned layer: stateWalkways refuses it, NoLaneLevel) or missing: nothing is stated, and a fault says so.
std::vector<WalkwayProblem> stateLotWalkways(const std::vector<std::shared_ptr<RoadModel>>& cities, const std::vector<LotBuilding>& lots);
// WHICH OF A LEVEL'S LANE-BUILT CITIES THE LOTS GROW ON (an index into its road model's `cities`; -1: none): the last.
// A level runs one lot pass, on all its blocks, reading that city's streets (as the lots producer reads that city's
// products); its paths are stated on that city's model and the street map reads them from there.
int lotsCity(const std::vector<std::shared_ptr<RoadModel>>& cities);
// The faults in one line, counted by kind ("12 the end's station is past where ..., 3 ..."); empty when none.
std::string describeWalkwayProblems(const std::vector<WalkwayProblem>& problems);
// What of a road file the lot pass reads besides what the city's own key already holds (the roads and their classes):
// its streets' and its junctions' inside stretches' ids, roads and stations. The lots key hashes it.
std::string lotRoadsKeyText(const nlohmann::json& roadFile);

// THE ERODED BASE A LANE-BUILT CITY'S TERRAIN READS (ADR-0095, ADR-0221): the city's conformed grid in a
// corridor along its roads (every node a `deck` reaches or the build changed, plus 40 m), the level's own
// terrain's finer detail (its own eroded base, `levelEroded`) faded in over the next 60 m, all of it beyond;
// blended over the grid's last 60 m into the level's terrain outside. The road build runs at the grid's
// cell (20 m on the island), not the ground's (2.44 m): the roads keep the ground they were built on, and
// the rest of the island gets the full ground. The loader draws the ground from it, the lot pass and the
// lots producer grow on it: one definition, so a baked city stands on the loader's ground.
std::shared_ptr<const std::function<double(double, double)>> laneErodedBase(
    const nlohmann::json& root, std::shared_ptr<const HeightGrid> grid,
    std::shared_ptr<const std::function<double(double, double)>> levelEroded, const RoadDeckField* deck = nullptr);

}  // namespace roads::lanes
}  // namespace engine

#endif
