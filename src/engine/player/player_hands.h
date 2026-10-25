#ifndef RAYTRACER_ENGINE_PLAYER_HANDS_H
#define RAYTRACER_ENGINE_PLAYER_HANDS_H

// WHAT THE HANDS HOLD (ADR-0246), beside whatever the body is doing: the tool slot in hand, and the grapple's hook
// -- fired, it flies to where it was aimed and bites if the trigger is still held; let go of the rope and it pops
// out and whips back to the hand. The rope you see (engine/rope_line.h) follows it in every mode.

#include "../locomotion.h"
#include "../rope_line.h"

#include <cstdint>

namespace engine {

class PlayerBody;
struct BodyStep;

struct PlayerHands {
    // THE TOOL IN HAND (the slots the gun script tracks too): 1 hands, 2 the gun, 3 the map, 4 the light,
    // 5 the parachute, 6 the wingsuit (jump in the air opens either), 7 the grapple, 8 the skateboard, 9 the scooter
    int gear = 1;
    bool holdingBoard() const { return gear == 8 || gear == 9; }
    bool holdingGlider() const { return gear == 5 || gear == 6; }
    bool holdingGrapple() const { return gear == 7; }

    // THE GRAPPLE'S HOOK
    enum class Hook { None, Flying, Hooked, Loose };
    Hook hook = Hook::None;
    GrappleAnchor aim;            // where a shot now would hook (the grapple in hand, each step)
    Vec3 hookTip, hookTarget;
    bool hookWillHold = false;    // the shot was at something it can hook
    Real looseLen = 0;            // the rope still out while it comes back in
    RopeLine rope;
    uint32_t hookShots = 0, hookBites = 0;

    // Aim, and spend a staged press of the trigger on a shot (from walking, falling, gliding...).
    void aimAndFire(PlayerBody& b, BodyStep& s);
    // The hook and the rope as drawn, every step whatever the body is doing.
    void stepHook(PlayerBody& b, BodyStep& s);

private:
    void fire(PlayerBody& b, BodyStep& s);
};

}  // namespace engine

#endif
