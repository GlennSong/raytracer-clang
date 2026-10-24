#ifndef RAYTRACER_ENGINE_ROPE_LINE_H
#define RAYTRACER_ENGINE_ROPE_LINE_H

#include "../rt_math.h"

#include <functional>
#include <vector>

namespace engine {

// THE ROPE YOU SEE (Glenn, 2026-10-24: "Is there a way to make the grapple a little more rope like?"). A chain of
// points, Verlet-stepped: gravity, a little air drag, links that pull but never push (a rope, not a rod), and
// long-range limits from each pinned end so a taut rope is straight in a few passes instead of sagging like
// elastic. Each point is swept against the world as it moves, so a slack rope lies over a ledge instead of through it.
//
// It is only the LOOK. The swing is still the one exact constraint in ropeStep (locomotion.h): a body on a chain of
// light links stretches and jitters, and the player is a character controller, not a rigid body. The start is pinned
// to the hand; the end is pinned to the hook, or free (`endFree`) when the rope has let go and is coming back in.
// The rest length can change every step (reeling), which is why this is ours and not a Jolt soft body: a soft body's
// edge lengths are fixed when it is made.
class RopeLine {
public:
    // A sweep from `from` to `to`: true and the contact point + normal if it hits something.
    using Sweep = std::function<bool(const Vec3& from, const Vec3& to, Vec3& hit, Vec3& normal)>;

    static constexpr int kPoints = 24;

    // All the points at `at`, still (a rope coming out of the hand).
    void reset(const Vec3& at);
    bool active() const { return !p_.empty(); }
    void clear() { p_.clear(); prev_.clear(); }

    // One step. `start` is where the hand is; `end` where the hook is (ignored when `endFree`). `restLength` is how
    // long the rope is: shorter than the span and the ends pull it straight, longer and it sags.
    void step(const Vec3& start, const Vec3& end, bool endFree, Real restLength, Real dt, const Sweep& sweep);

    const std::vector<Vec3>& points() const { return p_; }

private:
    std::vector<Vec3> p_, prev_;
};

}  // namespace engine

#endif
