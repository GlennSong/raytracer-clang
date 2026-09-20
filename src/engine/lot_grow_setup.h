#ifndef RAYTRACER_ENGINE_LOT_GROW_SETUP_H
#define RAYTRACER_ENGINE_LOT_GROW_SETUP_H

// The lot pass's parameters from a level (ADR-0084, milestone B): what LevelLoader::growCityLots assembled
// inline — LotParams and EdgeBlockParams from the citysim block, hubs and the coreness centre from the
// nets, the ground sampler, the enterable-building spawn, the style book (Lua, kept alive for the grow)
// and the archetype book (Lua resolved to data). One derivation, so the loader and the `lots` bundle
// producer grow the same city from the same inputs.

#include "engine/procgen/city/city_lots.h"
#include "engine/procgen/city/roads/road_entity.h"
#include "engine/procgen/terrain.h"
#include "engine/procgen/terrain_field.h"

#include <nlohmann/json.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace engine {

using LotGroundWithFn = std::function<std::function<Real(Real, Real, Real)>(const std::vector<TerrainFlatten>&)>;

struct LotGrowSetup {
    LotParams lp;
    EdgeBlockParams ep;
    std::shared_ptr<void> styleVm;          // the style book's VM: the hook in lp holds it, so it must outlive the grow
    std::vector<std::string> scriptFiles;   // resolved book paths, for the loader's script watch list
    double roadClear = 4.6;                 // sidewalk + 0.6: buildings keep clear of the sampled road corridors
    bool wantFlat = false;                  // grow the LOD1 twin (facadeDistance > detailDistance)
    bool planOnly = false;                  // outlines only, no buildings
};

LotGrowSetup lotGrowSetupForLevel(const nlohmann::json& cs, const std::string& levelDir, const HeightField& ground,
                                  const std::vector<RoadEntity>& nets, LotGroundWithFn groundWith, double groundMeshCell,
                                  const Vec2* enterableAt);

// The authored player position's XZ (the spawn building grows enterable), false when the level has none.
bool authoredSpawnXZ(const nlohmann::json& root, Vec2& out);

}  // namespace engine

#endif
