#ifndef RAYTRACER_ENGINE_UNDERWATER_SYSTEM_H
#define RAYTRACER_ENGINE_UNDERWATER_SYSTEM_H

#include "../system.h"

namespace engine {

class World;

// Is the camera under water, and whose (#58)? Each frame it looks for a water surface over the camera --
// the open sea (Sea), a lake or a river (the terrain's Hydrology) -- and, when the camera is below it,
// sets SceneLighting::underwater: that water's surface height, colour and clarity. The renderer's
// composite does the rest (absorption, caustics, the surface from below).
class UnderwaterSystem : public System {
public:
    void update(FrameContext& ctx) override;

    enum class Water { None, Sea, Lake, River };
    struct Surface { Water kind = Water::None; double level = 0.0; };
    // The water surface at (x, z), if any: a lake before a river (a river mouth inside a lake is the lake),
    // the sea last (rivers carve their channels below sea level near their mouths).
    static Surface surfaceAt(World& world, double x, double z);
};

}  // namespace engine

#endif
