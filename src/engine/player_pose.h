#ifndef RAYTRACER_ENGINE_PLAYER_POSE_H
#define RAYTRACER_ENGINE_PLAYER_POSE_H

#include "../rt_math.h"

#include <cstdint>

namespace engine {

// WHAT THE PLAYER'S BODY IS DOING, for whatever draws it (the first-person block arms, PlayerArmsSystem). Written
// by PlayerSystem on the player entity every fixed step; read-only everywhere else.
struct PlayerPose {
    enum class Mode : uint8_t { Walk, Air, Mantle, Climb, Swim, Ladder, Parachute, Wingsuit, Grapple, Ragdoll, Board,
                                PlaySwing, Bars, Drive, Hidden };
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
    Real swimStroke = 0;      // 0 treading water upright (still), 1 flat out in the crawl (going somewhere), eased
    Real swimClock = 0;       // s afloat: the treading kick and the hands sculling run on it
    bool underwater = false;
    Real waterLevel = 0;      // swimming: the surface's height (a bubble pops there)
    Real stamina = 1;         // 0..1
    Real landed = 0;          // a landing's jolt, 1 on touchdown easing to 0
    // GLIDING: the canopy's heading (horizontal unit) and how far out it is (0..1); the body's speed through the air
    Vec3 canopyHeading{0, 0, -1};
    Real canopyOpen = 0;
    Real airspeed = 0;
    Vec3 velocity;
    // THE GRAPPLE: where the hook is (the rope runs from the right hand to it)
    Vec3 anchor;
    // THE ROPE AS DRAWN (engine/rope_line.h), the hand's end first; ropePoints 0 = no rope out. Drawn in any mode:
    // it flies out ahead of the hook, hangs from it, and whips back in after letting go.
    static constexpr int kRopePoints = 24;
    Vec3 rope[kRopePoints];
    int ropePoints = 0;
    bool hookFlying = false;
    Real reel = 0;            // m/s the rope is being reeled: + in, - out
    // THINGS THAT HAPPENED, counted, for whatever plays their sounds (MotionAudioSystem edge-detects them)
    uint32_t hookShots = 0;   // the hook fired
    uint32_t hookBites = 0;   // ...and bit into something
    uint32_t ropeSnaps = 0;   // a slack rope came taut under the body
    Real snapSpeed = 0;       // ...this fast along the rope (m/s)
    uint32_t chuteOpens = 0;
    uint32_t ollies = 0;
    uint32_t boardLands = 0;  // the wheels back down after air
    uint32_t grabs = 0;       // the hands caught hold: a wall, a ledge, a rung, a bar
    Real landSpeed = 0;       // ...this fast down (m/s)
    bool grounded = false;
    int handsFree = 2;        // hands free for something else -- the gun (2 both, 1 one, 0 none), from the body's mode
    int tool = 1;             // the tool slot in hand (engine/player/tools.h): where the light rides
    // ON A BOARD: which (0 skateboard, 1 scooter), which way it points, how far through a kick (0..1, 0 = none)
    int board = 0;
    Vec3 boardHeading{0, 0, -1};
    Real kick = 0;
    // AT THE WHEEL (Mode::Drive; VehicleSystem writes these while you drive or ride, ADR-0252 -- the seat is the
    // vehicle's): the body's sway in the seat (engine/player/driver_seat.h SeatSway, m in the vehicle's frame), the
    // steering -1..1 and a bicycle's cranks (revolutions)
    Vec3 seatLean, seatNod;
    Real steer = 0;
    Real crank = 0;
    // GONE LIMP: the six blocks where the ragdoll's bodies are (engine/block_body.h order)
    Vec3 ragdollPos[10];   // (BlockBody::kParts)
    Quat ragdollRot[10];
};

}  // namespace engine

#endif
