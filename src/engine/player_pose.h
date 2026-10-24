#ifndef RAYTRACER_ENGINE_PLAYER_POSE_H
#define RAYTRACER_ENGINE_PLAYER_POSE_H

#include "../rt_math.h"

#include <cstdint>

namespace engine {

// WHAT THE PLAYER'S BODY IS DOING, for whatever draws it (the first-person block arms, PlayerArmsSystem). Written
// by PlayerSystem on the player entity every fixed step; read-only everywhere else.
struct PlayerPose {
    enum class Mode : uint8_t { Walk, Air, Mantle, Climb, Swim, Ladder, Hidden };
    Mode mode = Mode::Walk;
    Real speed = 0;           // horizontal m/s
    Real stride = 0;          // walk cycles so far (the fraction is the phase)
    bool run = false;
    bool crouched = false;
    // HANDS ON SOMETHING (mantle, climb, ladder): where each hand holds (world) and the surface's normal
    Vec3 handL, handR;
    Vec3 wallNormal;
    Real progress = 0;        // a mantle's 0..1
    Real climbCycle = 0;      // climbing: hand-over-hand cycles so far
    Real swimPhase = 0;       // swimming: strokes so far
    bool underwater = false;
    Real stamina = 1;         // 0..1
    Real landed = 0;          // a landing's jolt, 1 on touchdown easing to 0
};

}  // namespace engine

#endif
