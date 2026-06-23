#ifndef RAYTRACER_ENGINE_ANIMATION_PATH_H
#define RAYTRACER_ENGINE_ANIMATION_PATH_H

#include "editable_curve.h"
#include "../rt_math.h"

namespace engine {

// How the traversal repeats once it reaches the end of the curve.
enum class LoopMode { Once, Loop, PingPong };

// An animation path (ADR-0050): an EditableCurve plus how an object follows it. The
// second consumer of the curve toolkit — a path painted in the world and associated
// with an object, which a "play" / isolated-sim drives the object along. Editing
// (when not simulating) goes through the same curve handles the road editor uses.
struct AnimationPath {
    EditableCurve curve;
    double   duration = 5.0;          // seconds to traverse the whole curve once
    LoopMode loop = LoopMode::Loop;
    bool     faceTangent = true;      // orient the object's forward (+Z) along travel
    Vec3     upHint{0, 1, 0};         // up reference for the facing
};

// The object's pose at a point along the path.
struct PathPose {
    Vec3   position;
    Quat   orientation;
    double u = 0;                      // normalized progress [0,1] after the loop mode
};

// Sample the path at `t` seconds: applies the loop mode, walks the curve at CONSTANT
// speed (arc length), and — if faceTangent — orients +Z along the direction of travel.
PathPose animationPose(const AnimationPath& path, double t);

}  // namespace engine

#endif
