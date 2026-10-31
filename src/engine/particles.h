#ifndef RAYTRACER_ENGINE_PARTICLES_H
#define RAYTRACER_ENGINE_PARTICLES_H

#include "../rt_math.h"
#include "world.h"   // Entity

#include <cstdint>
#include "../proc_rng.h"
#include <string>
#include <unordered_map>
#include <vector>

namespace engine {

// PARTICLES (Glenn, 2026-10-24: "We don't have a particle system, but I wonder if we could introduce a rudimentary one
// to have some dust kick up when you kick for the skateboard, drift around a corner, or bubbles underwater ..." and
// then: "Is it very pointed to these asks or is it more generalized?"). General, in three parts:
//
//  EFFECTS are data: a named ParticleEffect says how its particles move (gravity, drag, wobble), live (life), look
//  (sprite shape, colour, glow, size and fade along the life, a stretch along their speed) and burst (how many,
//  how fast, how spread). The book is assets/scripts/particles.lua (scripting/particles_lua.h); the built-ins
//  below mirror it so a level without the script, and the tests, have the same effects.
//
//  ANY SYSTEM SETS THEM OFF by name: requestParticles(world, "spark", at, along) queues a burst that
//  ParticleSystem spends next frame. For a steady flow, a ParticleEmitter component on any entity with a Transform
//  (or placed in a level's "emitters") emits `rate` a second, which its owner can turn up, down or off.
//
//  THE SIMULATION here is points with a speed, an age and a life, stepped under the effect's gravity (negative for a
//  bubble) and drag. No collision: dust and smoke live a second or two in the air, sparks die before they would
//  bounce, and a particle with a ceiling (a bubble at the water's surface) ends there. ParticleSystem draws them.

// The sprite's shape: a soft puff (dust, smoke, spray), a ring (a bubble), a glow with a hot middle (a spark, a streak)
enum class ParticleShape : uint8_t { Puff, Ring, Glow };

struct ParticleEffect {
    std::string name;
    // moving
    Real gravity = 0;          // m/s^2 down (negative rises)
    Real drag = 0;             // 1/s: speed lost to the air
    Real wobble = 0;           // m/s of sideways wander (a bubble's)
    // living
    Real lifeMin = 1, lifeMax = 1;
    // looking
    ParticleShape shape = ParticleShape::Puff;
    Vec3 colour{1, 1, 1};      // albedo (lit by the scene)
    Vec3 glow{0, 0, 0};        // emission (a spark's heat)
    Real sizeStart = 0.1, sizeEnd = 0.1;   // m across, eased between over the life
    Real alpha = 1;            // the most it shows
    Real fadeIn = 0.1;         // fraction of the life spent appearing...
    Real fadeOut = 0.5;        // ...and the last fraction spent going
    Real stretch = 0;          // s: drawn this long along its speed relative to its `ref` (0 = round)
    // bursting (requestParticles' defaults)
    int count = 8;
    Real speed = 1;            // m/s along the request's direction
    Real up = 0;               // m/s up besides
    Real spread = 0.5;         // m/s in any direction (a ball)
    Real jitter = 0;           // m round the point they start from
};

// The effects by name. global() is what the game uses: the built-ins until particles.lua is loaded over them.
class ParticleBook {
public:
    ParticleBook();   // the built-ins
    static ParticleBook& global();

    void set(const ParticleEffect& e);   // adds, or replaces the one of that name
    int find(const std::string& name) const;   // -1: none
    const ParticleEffect& at(int index) const { return effects_[static_cast<std::size_t>(index)]; }
    std::size_t size() const { return effects_.size(); }

private:
    std::vector<ParticleEffect> effects_;
    std::unordered_map<std::string, int> byName_;
};

struct Particle {
    Vec3 pos, vel;
    Vec3 ref;                  // the velocity it is stretched against (a streak's: the body flying past it)
    float age = 0, life = 1;
    float size = 1;            // a per-particle scale on the effect's sizes (variety)
    float ceiling = 1e9f;      // it ends here (a bubble at the surface)
    float phase = 0;           // the wobble's
    uint16_t effect = 0;
};

class ParticlePool {
public:
    static constexpr std::size_t kMax = 4000;   // past this the oldest go first

    explicit ParticlePool(const ParticleBook& book = ParticleBook::global(), uint32_t seed = 0x70617274u)
        : book_(&book), rng_(seed) {}

    void emit(int effect, const Vec3& pos, const Vec3& vel, Real size = 1, Real ceiling = 1e9, const Vec3& ref = Vec3());
    // `count` (< 0: the effect's) round `pos`, moving `dir` * the effect's speed, plus its `up` and `spread`, starting
    // up to its `jitter` off the point, sized 0.75..1.25 of the effect's; `scale` multiplies the speeds and spread.
    void burst(int effect, const Vec3& pos, const Vec3& dir, int count = -1, Real scale = 1, Real ceiling = 1e9,
               const Vec3& ref = Vec3());
    void step(Real dt);
    void clear() { p_.clear(); }

    const std::vector<Particle>& particles() const { return p_; }
    std::size_t size() const { return p_.size(); }
    const ParticleBook& book() const { return *book_; }

    // Along its life: how big it is drawn (m) and how much it shows (0..alpha).
    Real drawnSize(const Particle& p) const;
    Real drawnAlpha(const Particle& p) const;

    ProcRng& rng() { return rng_; }

private:
    const ParticleBook* book_;
    std::vector<Particle> p_;
    ProcRng rng_;
};

// A burst asked for by any system, spent by ParticleSystem next frame.
struct ParticleRequest {
    std::string effect;
    Vec3 at, dir;
    int count = -1;            // < 0: the effect's
    Real scale = 1;            // on its speeds and spread (a harder landing kicks up more)
    Real ceiling = 1e9;
    Vec3 ref;
};
struct ParticleRequests {      // on one entity, made on the first request
    std::vector<ParticleRequest> queue;
};
void requestParticles(World& world, const ParticleRequest& r);
inline void requestParticles(World& world, const std::string& effect, const Vec3& at, const Vec3& dir = Vec3(),
                             int count = -1, Real scale = 1) {
    ParticleRequest r;
    r.effect = effect;
    r.at = at;
    r.dir = dir;
    r.count = count;
    r.scale = scale;
    requestParticles(world, r);
}

// A STEADY FLOW from an entity (with a Transform): `rate` a second of `effect`, from `offset` (in the entity's own
// frame) inside a box of `halfExtent`, moving `velocity` (world) plus the effect's spread. Its owner turns `rate` up,
// down or to 0. `ahead` s: start them where the entity will be (the air ahead of a falling body); `relative`: stretch
// them against the entity's velocity (`entityVelocity`, set by the owner) -- the streaks of air going past.
struct ParticleEmitter {
    std::string effect;
    Real rate = 0;
    Vec3 offset;
    Vec3 halfExtent;
    Vec3 velocity;
    Real ceiling = 1e9;
    Real ahead = 0;
    bool relative = false;
    Vec3 entityVelocity;
    Real due = 0;              // emission owed between frames (ParticleSystem's)
};

}  // namespace engine

#endif
