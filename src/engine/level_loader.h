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
};

}  // namespace engine

#endif
