#ifndef RAYTRACER_ENGINE_WAYPOINT_H
#define RAYTRACER_ENGINE_WAYPOINT_H

#include "../rt_math.h"

namespace engine {

// THE WAYPOINT (ADR-0248): the one place the player has marked -- set on the map (A, Space or a right click at
// its crosshair), pointed to by the watch. On the player's entity; absent, there is none.
struct Waypoint {
    Vec3 at;   // world (the height is the ground's as far as the map knows: 0 on a flat map)
};

}  // namespace engine

#endif
