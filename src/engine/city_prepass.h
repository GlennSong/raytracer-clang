#ifndef RAYTRACER_ENGINE_CITY_PREPASS_H
#define RAYTRACER_ENGINE_CITY_PREPASS_H

// The world the lot pre-pass grows in, derived from a level's JSON alone (ADR-0084, milestone C).
//
// LevelLoader builds this inline before it grows lots: the natural terrain sampler, the shape:"road"
// entities with their `generate` recipes run, the road carve regions, the earthwork field fitted to
// them, the freeway right-of-way, and the carved sampler the lots actually stand on. The `citylots`
// bundle producer needs the same world headlessly, and a second copy of that sequence is exactly the
// duplication AGENTS.md forbids — so it lives here once and both callers use it.
//
// It is deliberately NOT the whole of the loader's terrain setup. A level whose terrain is shaped by
// something the JSON does not fully determine — a Lua script pre-pass, a corridor solve — is refused
// by cityPrePassApplies() rather than approximated, because a producer that grows a DIFFERENT city
// than the loader would is worse than no cache at all.

#include "engine/lot_grow_setup.h"
#include "engine/procgen/city/roads/road_entity.h"
#include "engine/procgen/terrain.h"
#include "engine/procgen/terrain_field.h"

#include <nlohmann/json.hpp>
#include <memory>
#include <vector>

namespace engine {

struct CityPrePass {
    std::shared_ptr<TerrainParams> natural;    // the level's terrain, no road carve
    std::shared_ptr<Noise> noise;
    HeightField naturalGround;                 // the loader's `levelGround`: roads solve against this
    std::vector<RoadEntity> nets;              // one per shape:"road" entity, recipe applied
    std::vector<TerrainFlatten> roadFlatten;   // the road carve regions
    std::shared_ptr<TerrainParams> lotParams;  // natural + road carve + the earthwork field
    HeightField lotGround;                     // what the lots and pads sample
    LotGroundWithFn lotGroundWith;             // priority-correct rebind for in-pass block grades
    RoadGraph freewayROW;                      // Freeway/Ramp edges only; empty when there are none
    double lotMeshCell = 0.0;                  // finest rendered CDLOD cell, 0 without a cdlod block

    const RoadGraph* freewayROWOrNull() const { return freewayROW.edges.empty() ? nullptr : &freewayROW; }
};

// True when this level's lot pre-pass world is reproducible from its JSON alone — see the header note.
bool cityPrePassApplies(const nlohmann::json& root);

// Build it. Returns false when cityPrePassApplies() is false or the level has no road net.
bool cityPrePassForLevel(const nlohmann::json& root, CityPrePass& out);

}  // namespace engine

#endif
