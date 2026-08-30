#ifndef RAYTRACER_ENGINE_LEVEL_LOADER_H
#define RAYTRACER_ENGINE_LEVEL_LOADER_H

#include "system.h"
#include <string>
#include <vector>

namespace engine {

class AssetManager;

static constexpr int LEVEL_FORMAT_VERSION = 1;

struct LevelLoader {
    // editorMode replaces the physics player with a pickable PlayerSpawn
    // entity (green capsule gizmo) the editor can move; the game loads the
    // real player from the same "player" block as always.
    // Primitive meshes are acquired through `assets` (deduped + refcounted), so
    // the caller frees the previous level's meshes with assets.clear() before
    // loading (it pairs with world.destroyAll()).
    static bool load(const std::string& path,
                     World& world, Renderer& renderer, RenderView& view,
                     AssetManager& assets, bool editorMode = false);

    // Every .lua file the last load() actually read — entity recipes, host
    // preludes, and any module a recipe `require`d. Reset at the start of each
    // load.
    //
    // This exists for hot reload. The watcher used to re-derive the list by
    // parsing the level JSON, which cannot see a module: whether a recipe pulls
    // in vehicle_classes.lua is only knowable by RUNNING it. Recording what was
    // read is exact, where static scanning for `require(` would miss any
    // computed name. Paths are as resolved, so the watcher stats the same file
    // the loader opened.
    static const std::vector<std::string>& lastLoadedScriptFiles();

    // Verdict of the last RT_GROUND_PROBES pass (empty when the env is
    // unset): every probe compares the analytic terrainHeight against the
    // finest rendered tile's own bilinear interpolation — the surface the
    // player stands on. This exists so a test can assert the agreement on a
    // SHIPPED level through the real load path instead of re-deriving the
    // measurement (see lastLoadedScriptFiles for the precedent, and
    // test_levels_playable.cpp for why mirrors are forbidden there).
    struct GroundProbeReport {
        int total = 0;        // probes planted
        int flush = 0;        // |delta| <= 0.3 m
        int nearMiss = 0;     // 0.3 < |delta| <= 1 m
        int off = 0;          // |delta| > 1 m (cliff seams, dilation smear)
        double worst = 0.0;   // signed metres, function minus tile
        double worstX = 0.0, worstZ = 0.0;
    };
    static const GroundProbeReport& lastGroundProbeReport();

    // The RT_POKE_REPORT=1 result (the dense deck-vs-drawn-terrain poke map at
    // LOD 0/1/2, level_loader.cpp "[poke-report]"): kept here for the same
    // reason as GroundProbeReport — so a test can ASSERT it on a shipped level
    // instead of grepping a log. This is the exact number that killed the
    // DaylightBatter earthwork (0.00% -> 0.14% poke, road_net.cpp) and it had
    // no gate; the terrain-earthwork plan makes it one. Baseline on metro_v2:
    // LOD0/1/2 = 0 / 184,688 pokes.
    struct PokeReport {
        int lods = 0;                    // levels reported (0 = RT_POKE_REPORT unset)
        long samples[3] = {0, 0, 0};     // dense grid samples per LOD
        long pokes[3] = {0, 0, 0};       // samples where terrain > deck + 0.05 m
        double worst[3] = {0, 0, 0};     // metres, per LOD
        double worstX[3] = {0, 0, 0}, worstZ[3] = {0, 0, 0};
    };
    static const PokeReport& lastPokeReport();
};

}  // namespace engine

#endif
