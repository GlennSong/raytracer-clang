#ifndef RAYTRACER_ENGINE_PLAYER_INTENT_H
#define RAYTRACER_ENGINE_PLAYER_INTENT_H

#include "../../rt_math.h"
#include "../world.h"

#include <cstdint>

namespace engine {

// WHAT THE PLAYER WANTS THIS STEP (ADR-0246): the stick, the buttons held, the presses the frame staged for exactly
// one step, and where the view looks. The body's modes read this and never the keys -- so a rebinding is a
// bindings change, and anything that can fill one in (a probe, a replay, a possessed agent) drives the very same
// modes. PlayerSystem fills it from the input map each fixed step.
struct PlayerIntent {
    bool active = false;          // the body is the player's (the camera rides it); false: the freecam has the keys
    Real forward = 0;             // the stick: forward (+) / back, -1..1 -- 0 while not active
    Real right = 0;               // ...right (+) / left
    bool run = false;             // the buttons as held (raw: a mode asks for `active` as well where it matters)
    bool jumpHeld = false;
    bool crouchHeld = false;
    // ONE-SHOTS staged by the frame (a press read in a fixed step would fire twice on a two-step frame). A mode that
    // spends one clears it; one nobody spends waits for the next step.
    bool jump = false;
    bool limp = false;            // go limp (G, `ragdoll`)
    bool fire = false;            // the trigger's press, with the grapple in hand...
    bool fireHeld = false;        // ...and held (let go and the rope goes)
    Entity playTarget;            // E on the swing (playKind 1) or the monkey bars (2)
    uint64_t playKind = 0;
    // THE VIEW: its eye and which way it looks (the fly camera, which is the heading in either view)
    Vec3 eye{0, 0, 0};
    Vec3 look{0, 0, -1};
    Vec3 lookRight{1, 0, 0};
};

}  // namespace engine

#endif
