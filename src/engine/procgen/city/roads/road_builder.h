#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_BUILDER_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_BUILDER_H

#include "ground_grid.h"
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

// WHAT A BUILDER DOES TO THE GROUND — asked BEFORE anything is built, because a
// road picks a smooth, grade-limited line of its own and the ground has to be
// made to meet it. There are two honest answers and a builder gives exactly one:
//
//   carve    the level's terrain stays the level's; these are the patches to
//            press into it ("inside this polygon the ground is this plane,
//            feathering back to natural over N m"). The lattice works this way.
//   replace  the builder MADE the ground: a baked grid of absolute heights with
//            the cut and fill already in it, for the terrain to use as its base
//            inside the grid's bounds. The lane builder works this way — it
//            conforms its own grid while it paves, and that grid is what CDLOD
//            renders (it rides in through TerrainParams::erodedBase, the same
//            slot a baked erosion uses).
//
// Glenn chose `replace` for lanes on 2026-09-20: "it should hand back the ground
// which is what it currently does ... handing back patches is the old way."
struct GroundPlan {
    std::vector<TerrainFlatten> carve;
    std::shared_ptr<const GroundGrid> replace;
    bool replaces() const { return replace && !replace->empty(); }
};

class RoadBuilder {
public:
    virtual ~RoadBuilder() = default;
    virtual const char* name() const = 0;
    // The surface, built on `ground` (already carved/replaced).
    virtual RoadProducts build(const RoadBuildInput& in) const = 0;
    // What the ground must become before the surface exists — see GroundPlan.
    // The terrain pre-pass asks this against the NATURAL ground.
    virtual GroundPlan ground(const RoadBuildInput& in) const = 0;
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
