#ifndef RAYTRACER_ENGINE_PLAYER_POSE_H
#define RAYTRACER_ENGINE_PLAYER_POSE_H

#include "../rt_math.h"

#include <cstdint>

namespace engine {

// WHAT THE PLAYER'S BODY IS DOING, for whatever draws it (the first-person block arms, PlayerArmsSystem). Written
// by PlayerSystem on the player entity every fixed step; read-only everywhere else.
struct PlayerPose {
    enum class Mode : uint8_t { Walk, Air, Mantle, Climb, Swim, Ladder, Parachute, Wingsuit, Grapple, Ragdoll, Board,
                                PlaySwing, Bars, Hidden };
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
    // GLIDING: the canopy's heading (horizontal unit) and how far out it is (0..1); the body's speed through the air
    Vec3 canopyHeading{0, 0, -1};
    Real canopyOpen = 0;
    Real airspeed = 0;
    Vec3 velocity;
    // THE GRAPPLE: where the hook is (the rope runs from the right hand to it)
    Vec3 anchor;
    // ON A BOARD: which (0 skateboard, 1 scooter), which way it points, how far through a kick (0..1, 0 = none)
    int board = 0;
    Vec3 boardHeading{0, 0, -1};
    Real kick = 0;
    // GONE LIMP: the six blocks where the ragdoll's bodies are (engine/block_body.h order)
    Vec3 ragdollPos[6];
    Quat ragdollRot[6];
};

// Is the body drawn as posed blocks (PlayerBodySystem) rather than the walker mesh and its walk cycle?
inline bool drawnAsBlocks(PlayerPose::Mode m) {
    return m != PlayerPose::Mode::Walk && m != PlayerPose::Mode::Air && m != PlayerPose::Mode::Hidden;
}

}  // namespace engine

#endif
