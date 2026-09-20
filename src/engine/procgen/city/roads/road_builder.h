#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_BUILDER_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_BUILDER_H

#include "road_entity.h"
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <vector>

// The roads module: ONE interface, several builders (docs/road-module-plan.md).
//
// A level's road entity is PLANNED the same way whatever builds it — the
// "generate" recipe runs (applyGenerateRecipe) and leaves a RoadGraph on the
// entity — and then a builder turns that plan into the four things the rest of
// the engine reads (ADR-0085): the surface, the deck it rides, the terrain
// carve under it, and the graph everything routes on.
//
//   recipe ──applyGenerateRecipe──► RoadEntity ──RoadBuilder──► RoadProducts
//
// Which builder is DATA, not a build flag: a road entity says
// `"builder": "lattice"` (the default, and what every shipped level builds
// today) or `"builder": "lanes"`. The lattice's code lives in
// procgen/deprecated/roads — still compiled, still selectable, deleted when no
// level selects it.

namespace engine::roads {

// What a builder is given: the planned entity, the ground it drapes on, and the
// level's own road block (a builder reads its own knobs out of it, so adding one
// does not change this struct).
struct RoadBuildInput {
    const RoadEntity* road = nullptr;
    RoadGroundFn ground;                  // null = flat
    nlohmann::json options = nlohmann::json::object();
};

// What every builder owes. (Phase 3 adds the lanes builder's blocks and its
// right-of-way keep-out here; nothing fills them yet, so they are not fields.)
struct RoadProducts {
    RenderMesh mesh;              // the carriageway, kerbs, sidewalks, markings
    RoadDeckField deck;           // the surface things stand on
    CurbBandAudit bands;          // sidewalk loops — the city map draws these
};

class RoadBuilder {
public:
    virtual ~RoadBuilder() = default;
    virtual const char* name() const = 0;
    // The surface, built on `ground` (already carved).
    virtual RoadProducts build(const RoadBuildInput& in) const = 0;
    // The cut/fill the ground needs BEFORE the surface is built — the terrain
    // pre-pass runs this against the NATURAL ground, then builds on the result.
    virtual std::vector<TerrainFlatten> flattens(const RoadBuildInput& in) const = 0;
    // The centrelines everything routes on: nav, traffic, furniture, signs, map.
    virtual RoadGraph navGraph(const RoadBuildInput& in) const = 0;
};

inline constexpr const char* kDefaultRoadBuilder = "lattice";

void registerRoadBuilder(std::unique_ptr<RoadBuilder> builder);
// The builder by name, or nullptr (warned once per unknown name).
RoadBuilder* roadBuilder(std::string_view name);
std::vector<std::string> roadBuilderNames();
// The builder a road block asks for: its "builder" string, else the default.
std::string roadBuilderName(const nlohmann::json& roadBlock);
// The builder a road block asks for, never null: an unknown name warns and
// falls back to the default, so a typo in a level loses the road's shape, not
// the level.
RoadBuilder& roadBuilderFor(const nlohmann::json& roadBlock);

}  // namespace engine::roads

#endif
