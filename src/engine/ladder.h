#ifndef RAYTRACER_ENGINE_LADDER_H
#define RAYTRACER_ENGINE_LADDER_H

// LADDERS (Glenn, 2026-10-13: "We don't have ladders as interactable objects? Can we make ladders as interactable?
// And then use it on the fire engine and on buildings like auxiliary access ways to roofs?"). A ladder is data: where
// it stands and where it lets you off. The LadderSystem offers "climb" at its foot and "climb down" at its top (the
// interaction broker), and on the command puts CLIMBING on the player; the PlayerSystem then moves the body up and
// down the rails (no gravity, like swimming) and steps it off at either end. Two sources fill the list: the buildings'
// roof-access ladders (procgen/city/ladders.h, from the building records, nothing baked) and the vehicles' (a fire
// engine's rear ladder, published by the citysim each frame as `moving`).

#include "../rt_math.h"
#include "procgen/city/polygon.h"   // Vec2
#include <vector>

namespace engine {

struct LadderSpec {
    Vec3 foot;            // the bottom of the rails, their centre, where they meet the ground (world)
    Vec2 out{0, 0};       // horizontal unit normal from the ladder's wall toward the climber
    Real width = 0.5;     // between the rails
    Real topY = 0;        // where the rails end (world y)
    Real landY = 0;       // the surface you step off onto at the top (a roof, a truck's top)
    Vec3 exit;            // where you stand once off at the top (feet, world)
    bool cage = false;    // a safety cage round the climb (a tall fixed ladder)
};

// On the player while on a ladder. `feetY`: how high up it the feet are.
struct Climbing {
    LadderSpec ladder;
    Real feetY = 0;
};

// The world's ladders (one entity carries it; the LadderSystem makes it).
struct Ladders {
    std::vector<LadderSpec> fixed;    // buildings' (built once from the records)
    std::vector<LadderSpec> moving;   // vehicles' (rewritten every frame by whoever owns the vehicles)
    bool built = false;
};

}  // namespace engine

#endif
