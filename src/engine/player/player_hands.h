#ifndef RAYTRACER_ENGINE_PLAYER_HANDS_H
#define RAYTRACER_ENGINE_PLAYER_HANDS_H

// WHAT THE HANDS HOLD (ADR-0246), beside whatever the body is doing: the tool slot in hand, and the grapple's hook
// -- fired, it flies to where it was aimed and bites if the trigger is still held; let go of the rope and it pops
// out and whips back to the hand. The rope you see (engine/rope_line.h) follows it in every mode.

#include "../locomotion.h"
#include "../rope_line.h"
#include "tools.h"

#include <cstdint>

namespace engine {

class PlayerBody;
struct BodyStep;

struct PlayerHands {
    // THE TOOL IN HAND (engine/player/tools.h; the gun script tracks the slots too). A chute or a suit opens with a
    // jump in the air; a board is ridden on the ground.
    Tool tool = Tool::Hands;
    bool holdingBoard() const { return tool == Tool::Skateboard || tool == Tool::Scooter; }
    bool holdingGlider() const { return tool == Tool::Parachute || tool == Tool::Wingsuit; }
    bool holdingGrapple() const { return tool == Tool::Grapple; }

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
