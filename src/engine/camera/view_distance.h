#ifndef RAYTRACER_ENGINE_VIEW_DISTANCE_H
#define RAYTRACER_ENGINE_VIEW_DISTANCE_H

#include "../world.h"
#include "../components.h"

#include <algorithm>

namespace engine {

// World-extent-driven view distance. The far plane must cover the world seen
// corner-to-corner (a street-level view of the far edge of an 8 km map is
// ~11.3 km), so it derives from the CDLOD world half-extent rather than living
// as a per-controller constant. 8000 is the floor every pre-8km level was
// framed against; levels without CDLOD terrain keep it unchanged.
inline Real worldViewFar(World& world) {
    Real viewFar = 8000.0;
    world.each<TerrainLodConfig>([&](Entity, TerrainLodConfig& cfg) {
        viewFar = std::max(viewFar, static_cast<Real>(cfg.worldHalf) * 3.5);
    });
    return viewFar;
}

}  // namespace engine

#endif
