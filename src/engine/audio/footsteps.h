#ifndef RAYTRACER_ENGINE_AUDIO_FOOTSTEPS_H
#define RAYTRACER_ENGINE_AUDIO_FOOTSTEPS_H

// When a walker's feet make a sound, and which (#62 footsteps, #63 jump and landing). Pure logic, fed one
// fixed step at a time, so the cadence and the landing rules are testable without Jolt or a device; the
// FootstepSystem (engine/systems/footstep_system.h) feeds it the player and plays what it returns.

#include "sfx.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine {

struct FootstepEvent {
    enum class Kind : uint8_t { None, Step, Jump, Land };
    Kind kind = Kind::None;
    double volume = 1.0;   // 0..1: a stroll is quieter than a run, a crouch quieter still
    double heavy = 0.0;    // Land: 0 (a hop) .. 1 (a drop that would hurt)
};

struct FootstepTracker {
    // Stride: ~0.75 m at a walk, lengthening to ~1.9 m at a run (people take longer strides, not only
    // faster ones). One step sounds per stride travelled on the ground. 1.1 m at the player's 6 m/s was
    // 5.5 steps a second -- "the footsteps feel sped up" (Glenn, #98); a runner at 6 m/s takes ~3.
    static constexpr double kWalkStride = 0.75, kRunStride = 1.9, kRunSpeed = 6.0;
    static constexpr double kMinSpeed = 0.35;        // slower than this is shuffling on the spot: no steps
    static constexpr double kJumpUp = 1.5;           // leaving the ground this fast upward is a jump
    static constexpr double kLandQuiet = 1.8;        // landing slower than this is just the next step
    static constexpr double kLandHard = 11.0;        // ~6 m of free fall: heavy = 1
    static constexpr double kMinAir = 0.12;          // shorter "air" is a bump in the ground

    bool grounded = true;
    double stride = kWalkStride * 0.5;   // distance since the last step (standing: half a stride banked)
    double airTime = 0.0;
    double fallSpeed = 0.0;    // fastest downward speed while airborne

    // horizSpeed: over the ground, m/s; vy: vertical velocity (+up); crouched: sneaking.
    FootstepEvent update(bool onGround, double horizSpeed, double vy, double dt, bool crouched = false) {
        FootstepEvent ev;
        if (!onGround) {
            if (grounded && vy > kJumpUp) {   // the moment the feet leave: a push-off
                ev.kind = FootstepEvent::Kind::Jump;
                ev.volume = 0.8;
            }
            if (grounded) { airTime = 0; fallSpeed = 0; }
            grounded = false;
            airTime += dt;
            fallSpeed = std::max(fallSpeed, -vy);
            return ev;
        }
        if (!grounded) {   // touching down
            grounded = true;
            const bool realAir = airTime >= kMinAir;
            if (realAir && fallSpeed >= kLandQuiet) {
                ev.kind = FootstepEvent::Kind::Land;
                ev.heavy = std::clamp((fallSpeed - kLandQuiet) / (kLandHard - kLandQuiet), 0.0, 1.0);
                ev.volume = std::clamp(0.5 + 0.5 * ev.heavy + 0.1 * fallSpeed / kLandHard, 0.0, 1.0);
                stride = 0;   // the next step comes a stride after the landing
                return ev;
            }
            if (realAir) {   // a small hop down: sounds as a step
                ev.kind = FootstepEvent::Kind::Step;
                ev.volume = stepVolume(horizSpeed, crouched);
                stride = 0;
                return ev;
            }
        }
        if (horizSpeed < kMinSpeed) {
            // standing: the first step after starting off comes half a stride in, not a whole one
            stride = kWalkStride * 0.5;
            return ev;
        }
        stride += horizSpeed * dt;
        if (stride >= strideFor(horizSpeed)) {
            stride -= strideFor(horizSpeed);
            ev.kind = FootstepEvent::Kind::Step;
            ev.volume = stepVolume(horizSpeed, crouched);
        }
        return ev;
    }
    static double strideFor(double speed) {
        const double t = std::clamp((speed - 1.4) / (kRunSpeed - 1.4), 0.0, 1.0);
        return kWalkStride + (kRunStride - kWalkStride) * t;
    }
    static double stepVolume(double speed, bool crouched) {
        const double v = std::clamp(0.45 + 0.55 * speed / kRunSpeed, 0.35, 1.0);
        return crouched ? v * 0.45 : v;
    }
};

// Which sound the ground makes. `surface` is the body's engine::ColliderSurface tag (components.h:
// 0 unknown, 1 terrain, then asphalt, concrete, grass, dirt, sand, rock, snow, wood, metal); terrain
// takes the ground-cover weights at the foot (grass, dirt, sand, rock, snow; `haveCover` false: the
// level has no cover map, and terrain reads as grass). Unknown -- a building's floor, a plinth, a prop
// -- reads as concrete.
inline sfx::Ground groundForSurface(uint8_t surface, bool haveCover, double grass, double dirt, double sand,
                                    double rock, double snow) {
    switch (surface) {
        case 2: return sfx::Ground::Asphalt;
        case 3: return sfx::Ground::Concrete;
        case 4: return sfx::Ground::Grass;
        case 5: return sfx::Ground::Dirt;
        case 6: return sfx::Ground::Sand;
        case 7: return sfx::Ground::Rock;
        case 8: return sfx::Ground::Snow;
        case 9: return sfx::Ground::Wood;
        case 10: return sfx::Ground::Metal;
        case 1: break;
        default: return sfx::Ground::Concrete;
    }
    if (!haveCover) return sfx::Ground::Grass;
    if (snow >= 0.5) return sfx::Ground::Snow;   // snow over anything is snow
    sfx::Ground best = sfx::Ground::Grass;
    double w = grass;
    if (dirt > w) { w = dirt; best = sfx::Ground::Dirt; }
    if (sand > w) { w = sand; best = sfx::Ground::Sand; }
    if (rock > w) { w = rock; best = sfx::Ground::Rock; }
    return best;
}

}  // namespace engine

#endif
