#ifndef RAYTRACER_ENGINE_LEVEL_LOADER_H
#define RAYTRACER_ENGINE_LEVEL_LOADER_H

#include "system.h"
#include <string>

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
};

}  // namespace engine

#endif
