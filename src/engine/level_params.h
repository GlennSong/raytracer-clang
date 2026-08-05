#ifndef RAYTRACER_ENGINE_LEVEL_PARAMS_H
#define RAYTRACER_ENGINE_LEVEL_PARAMS_H

// Level-JSON -> params readers shared by BOTH level pipelines: the engine
// loader (level_loader.cpp, viewer/editor) and the offline tracer's importer
// (level_scene.cpp). One recipe, one parse — the only legitimate divergence
// between the pipelines is the renderer (AGENTS.md, "one pipeline, one
// divergence"). Before this module each importer parsed these blocks with its
// own copy, and the copies drifted: the offline city parse never learned the
// district-road fields, so a districtRoads level silently generated a
// different city offline than in the viewer.

#include "procgen/terrain.h"          // TerrainParams
#include "procgen/tree.h"             // TreeParams
#include "procgen/city/city.h"        // CityParams
#include "procgen/city/city_lots.h"   // LotParams, EdgeBlockParams
#include "procgen/city/water_mesh.h"  // WaterMeshParams

#include <nlohmann/json.hpp>

#include <cstdint>
#include <functional>
#include <memory>

namespace engine {

// The level's top-level "terrain" block.
TerrainParams readTerrainParams(const nlohmann::json& t);

// A shape:"tree" entity's "tree" block (hero L-system tree). `seedOut`
// receives the block's seed (0 when absent).
TreeParams readTreeParams(const nlohmann::json& ent, uint32_t& seedOut);

// A shape:"city" entity: position -> center/baseY, the "city" block's fields,
// and — when onTerrain and the level has a "terrain" block — the groundAt
// sampler closure that drapes the city on that terrain (ADR-0038 §6; the
// shared_ptrs keep params/noise alive for the closure).
CityParams readCityParams(const nlohmann::json& ent, const nlohmann::json& root);

// Baked erosion (opt-in via terrain.erode): bake ONCE and share the sampler
// across every consumer — ground, roads, lots — so they all conform to the
// same eroded surface. Null when the level doesn't opt in.
std::shared_ptr<const std::function<double(double, double)>>
readErodedBase(const nlohmann::json& root);

// The lot-growing knobs of a city/metro block (edge blocks + lot params).
// Callers still supply LotParams::ground and the hub list — those depend on
// loader-side state (terrain samplers, road nets), not on the JSON.
void readLotGrowParams(const nlohmann::json& cityJson,
                       EdgeBlockParams& edges, LotParams& lots);

// A "water" block ({seaLevel, region | lo/hi, cell, foamBand}).
WaterMeshParams readWaterParams(const nlohmann::json& w);

}  // namespace engine

#endif
