#ifndef RAYTRACER_ENGINE_PLAYGROUND_H
#define RAYTRACER_ENGINE_PLAYGROUND_H

// PLAYGROUND EQUIPMENT THE PLAYER CAN USE (Glenn, 2026-10-23: "It would be fun if you could also interact with the
// swings, monkey bars, and gymnasium stuff in the park"). A level lists them under "playground" (the movement lab
// has a set); the frames are ordinary blocks, these are the parts that move or are held. PlayerSystem offers them
// (E: sit on the swing, grab the bars) and rides them (Move::Swing, Move::Bars).

#include "../rt_math.h"
#include "world.h"

namespace engine {

struct PlaySwing {
    Vec3 pivot;               // the middle of the beam over the seat (world)
    Real length = 2.0;        // chain length: pivot to seat
    Vec3 forward{0, 0, 1};    // horizontal unit: the way it swings out (and the rider faces)
    Entity seat;              // the chains and seat, drawn hanging from the pivot, turned by the swing
    Real theta = 0;           // the seat's angle off vertical now (rad, + toward forward)
    Real omega = 0;           // ...and its rate: an empty swing swings on and dies down
    bool ridden = false;
};

struct MonkeyBars {
    Vec3 a, b;                // the two ends of the bar line (world, at the bars' height)
};

}  // namespace engine

#endif
