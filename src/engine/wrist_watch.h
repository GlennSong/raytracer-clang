#ifndef RAYTRACER_ENGINE_WRIST_WATCH_H
#define RAYTRACER_ENGINE_WRIST_WATCH_H

#include "../rt_math.h"

namespace engine {

// THE WATCH ON THE PLAYER'S LEFT WRIST (ADR-0251), on the player's entity. WatchSystem raises it (held Q, or down on
// the D-pad) when a hand is free; PlayerArmsSystem brings the wrist up in front of the face and says where it drew
// the watch's face, which WatchSystem draws the dial onto.
struct WristWatch {
    Real raise = 0;          // 0 down .. 1 up in front of the face (eased)
    // where the face was drawn this frame (first person): its centre, the face's right and up (world, unit), radius
    bool faceShown = false;
    Vec3 centre, right, up;
    Real radius = 0.022;
};

}  // namespace engine

#endif
