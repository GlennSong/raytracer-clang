#ifndef RAYTRACER_ENGINE_LEVEL_LOADER_H
#define RAYTRACER_ENGINE_LEVEL_LOADER_H

#include "system.h"
#include <string>

namespace engine {

static constexpr int LEVEL_FORMAT_VERSION = 1;

struct LevelLoader {
    static bool load(const std::string& path,
                     World& world, Renderer& renderer, RenderView& view);
};

}  // namespace engine

#endif
