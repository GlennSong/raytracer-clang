#include "footstep_system.h"

#include "physics_system.h"
#include "underwater_system.h"
#include "../audio/sfx.h"
#include "../components.h"
#include "../procgen/ground_cover.h"
#include "../procgen/height_pyramid.h"
#include "../procgen/terrain.h"
#include "../world.h"
#include "../../log.h"

#include <chrono>
#include <cstdlib>

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
AudioClipHandle makeClip(AudioEngine& audio, const std::vector<float>& pcm, uint32_t rate) {
    return audio.createClip(pcm.data(), pcm.size(), 1, rate);
}
}  // namespace

void FootstepSystem::onStart(FrameContext& ctx) {
    ready_ = ctx.audio.ready();
    if (!ready_) return;
    const auto t0 = std::chrono::steady_clock::now();
    const uint32_t rate = ctx.audio.sampleRate();
    const double landHeavy[kLandLevels] = {0.15, 0.55, 1.0};
    for (int g = 0; g < kGrounds; ++g) {
        const auto ground = static_cast<sfx::Ground>(g);
        for (int v = 0; v < kStepVariants; ++v)
            steps_[g][v] = makeClip(ctx.audio, sfx::footstep(ground, rate, 101u + 17u * v + 131u * g), rate);
        jumps_[g] = makeClip(ctx.audio, sfx::jumpPush(ground, rate, 7u + g), rate);
        for (int l = 0; l < kLandLevels; ++l)
            lands_[g][l] = makeClip(ctx.audio, sfx::landing(ground, landHeavy[l], rate, 31u + 5u * l + 97u * g), rate);
    }
    for (int v = 0; v < kStepVariants; ++v) swishes_[v] = makeClip(ctx.audio, sfx::grassSwish(rate, 3u + 29u * v), rate);
    const double splashStrength[3] = {0.12, 0.35, 1.0};
    for (int k = 0; k < 3; ++k) splashes_[k] = makeClip(ctx.audio, sfx::splash(splashStrength[k], rate, 5u + k), rate);
    LOG_INFO << "[footsteps] " << kGrounds * (kStepVariants + 1 + kLandLevels) + kStepVariants << " clips synthesized in "
             << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() << " ms";
}

void FootstepSystem::onStop(FrameContext&) {
    swishLevel_ = 0;
}

double FootstepSystem::groundHeight(const TerrainLodConfig& cfg, double x, double z) {
    if (cfg.baked) return cfg.baked->height(x, z);
    if (!noise_ || noiseSeed_ != cfg.seed) { noise_ = std::make_unique<Noise>(cfg.seed); noiseSeed_ = cfg.seed; }
    return terrainHeight(cfg.params, *noise_, x, z);
}

sfx::Ground FootstepSystem::groundAt(World& world, uint8_t surface, double x, double y, double z, double* slopeCos) {
    if (slopeCos) *slopeCos = 1.0;
    if (surface != static_cast<uint8_t>(ColliderSurface::Terrain))
        return groundForSurface(surface, false, 0, 0, 0, 0, 0);
    const TerrainLodConfig* cfg = nullptr;
    world.each<TerrainLodConfig>([&](Entity, TerrainLodConfig& c) { if (!cfg) cfg = &c; });
    if (!cfg || !cfg->params.cover) return groundForSurface(surface, false, 0, 0, 0, 0, 0);
    // the slope the cover judges rock by, from the ground a step either side
    const double e = 1.0;
    const double hx0 = groundHeight(*cfg, x - e, z), hx1 = groundHeight(*cfg, x + e, z);
    const double hz0 = groundHeight(*cfg, x, z - e), hz1 = groundHeight(*cfg, x, z + e);
    const Vec3 n = normalize(Vec3(-(hx1 - hx0) / (2 * e), 1.0, -(hz1 - hz0) / (2 * e)));
    if (slopeCos) *slopeCos = n.y;
    const Cover c = cfg->params.cover->at(x, z, y, n.y, n.x, n.z);
    return groundForSurface(surface, true, c.grass, c.dirt, c.sand, c.rock, c.snow);
}

void FootstepSystem::fixedUpdate(FrameContext& ctx) {
    if (!ready_) return;
    Entity player;
    bool found = false;
    ctx.world.each<Transform, CharacterController, ControlledBy>(
        [&](Entity e, Transform&, CharacterController& cc, ControlledBy&) {
            if (!found && cc.characterId != INVALID_CHARACTER) { player = e; found = true; }
        });
    const Real dt = ctx.clock.fixedStep();
    if (!found || ctx.world.has<InVehicle>(player)) { swishLevel_ = 0; tracker_ = FootstepTracker{}; haveLastFeet_ = false; return; }
    const Transform& t = *ctx.world.get<Transform>(player);
    const CharacterController& cc = *ctx.world.get<CharacterController>(player);
    PhysicsWorld& pw = physics_.physicsWorld();
    const GroundState gs = pw.characterGroundState(cc.characterId);
    const bool onGround = gs == GroundState::OnGround || gs == GroundState::OnSteepGround;
    const Vec3 v = pw.characterVelocity(cc.characterId);
    const Vec3 feet = t.position - Vec3(0, cc.halfHeight + cc.radius, 0);
    // PACE FROM WHERE THE FEET WENT: the controller's velocity is what was asked for, and pushing into a
    // wall or up a slope too steep to climb asks for walking speed while going nowhere. A jump in position
    // (a teleport, a respawn) is not a walk either.
    double horiz = 0.0;
    if (haveLastFeet_ && dt > 0) {
        const double dx = feet.x - lastFeet_.x, dz = feet.z - lastFeet_.z;
        const double moved = std::sqrt(dx * dx + dz * dz);
        horiz = moved < 1.0 ? moved / dt : 0.0;
    }
    lastFeet_ = feet;
    haveLastFeet_ = true;
    // SWIMMING (#43): no feet on the ground -- a splash going in (bigger the harder you hit the water), then a
    // soft stroke every ~0.9 s while you swim
    if (cc.swimming) {
        if (!wasSwimming_) {
            const int k = fallBeforeWater_ > 6.0 ? 2 : 1;
            AudioPlayParams pp; pp.bus = AudioBus::Sfx; pp.volume = static_cast<float>(k == 2 ? 1.0 : 0.55);
            ctx.audio.playAt(splashes_[k], t.position, 25.0, pp);
            strokeTimer_ = 0.6;
        }
        wasSwimming_ = true;
        if (horiz > 0.4 && (strokeTimer_ -= dt) <= 0) {
            strokeTimer_ = 0.85 + 0.15 * std::uniform_real_distribution<double>(0, 1)(rng_);
            std::uniform_real_distribution<double> pitch(0.9, 1.1);
            AudioPlayParams pp; pp.bus = AudioBus::Sfx; pp.volume = 0.4f; pp.pitch = static_cast<float>(pitch(rng_));
            ctx.audio.playAt(splashes_[0], t.position, 18.0, pp);
        }
        swishLevel_ = 0;
        tracker_ = FootstepTracker{};
        tracker_.grounded = false;   // leaving the water is a touch-down, not a jump
        return;
    }
    wasSwimming_ = false;
    fallBeforeWater_ = std::max(0.0, -v.y);
    const FootstepEvent ev = tracker_.update(onGround, horiz, v.y, dt, cc.halfHeight < 0.3);

    // #64: tall grass brushing the legs -- only where the ground under the foot IS grass (or earth), asking each
    // grass field with the real slope (it thins grass on steep ground; asked as if flat it called a rock face a
    // meadow, #85), and only grass that reaches the shin: the city's mown lawns (a 0.55 m meadow cut to 0.3 of
    // that) and park lawns stay silent (2026-10-09, Glenn: "a constant low roar" walking on the city's grass)
    if (onGround && horiz > 0.3) {
        if ((hereTimer_ -= dt) <= 0) {
            hereTimer_ = 0.1;
            groundHere_ = groundAt(ctx.world, pw.bodySurface(pw.characterGroundBody(cc.characterId)), feet.x, feet.y, feet.z, &slopeHere_);
            swishLevel_ = 0.0;
            if (groundHere_ == sfx::Ground::Grass || groundHere_ == sfx::Ground::Dirt)
                ctx.world.each<GrassField>([&](Entity, GrassField& g) {
                    if (!g.density || !g.ground) return;
                    const double tall = g.bladeHeight * (g.height ? g.height(feet.x, feet.z) : 1.0);
                    const double reach = std::clamp((tall - 0.25) / 0.35, 0.0, 1.0);   // ankle-high: nothing; 0.6 m: all
                    if (reach <= 0) return;
                    const double d = std::clamp(g.density(feet.x, feet.z, g.ground(feet.x, feet.z), slopeHere_), 0.0, 1.0);
                    swishLevel_ = std::max(swishLevel_, d * reach);
                });
        }
    } else {
        hereTimer_ = 0;
    }

    // what the foot is on: this tick's ground body; a push-off, the one it just left
    const PhysicsBodyId under = pw.characterGroundBody(cc.characterId);
    if (ev.kind == FootstepEvent::Kind::None) {
        if (under != INVALID_PHYSICS_BODY) lastSurface_ = pw.bodySurface(under);
        return;
    }
    const bool leaving = ev.kind == FootstepEvent::Kind::Jump || under == INVALID_PHYSICS_BODY;
    const uint8_t surface = leaving ? lastSurface_ : pw.bodySurface(under);
    const sfx::Ground ground = leaving && ev.kind == FootstepEvent::Kind::Jump ? lastGround_ : groundAt(ctx.world, surface, feet.x, feet.y, feet.z);
    lastSurface_ = surface;
    lastGround_ = ground;
    // wading: water over the feet makes every step a slosh, whatever the bed is
    {
        const UnderwaterSystem::Surface w = UnderwaterSystem::surfaceAt(ctx.world, feet.x, feet.z);
        if (w.kind != UnderwaterSystem::Water::None && w.level > feet.y + 0.08) {
            const int gw = static_cast<int>(sfx::Ground::Water);
            AudioPlayParams wp; wp.bus = AudioBus::Sfx; wp.pitch = static_cast<float>(std::uniform_real_distribution<double>(0.94, 1.06)(rng_));
            wp.volume = static_cast<float>(ev.volume * (ev.kind == FootstepEvent::Kind::Land ? 0.9 : 0.5));
            AudioClipHandle c = ev.kind == FootstepEvent::Kind::Land ? splashes_[1]
                              : ev.kind == FootstepEvent::Kind::Jump ? jumps_[gw] : steps_[gw][std::max(0, lastVariant_) % kStepVariants];
            if (ev.kind == FootstepEvent::Kind::Step) {
                std::uniform_int_distribution<int> pick(0, kStepVariants - 1);
                c = steps_[gw][pick(rng_)];
            }
            ctx.audio.playAt(c, feet, 20.0, wp);
            return;
        }
    }
    const int g = static_cast<int>(ground);
    std::uniform_real_distribution<double> pitch(0.94, 1.06);
    AudioPlayParams pp;
    pp.bus = AudioBus::Sfx;
    pp.pitch = static_cast<float>(pitch(rng_));
    pp.volume = static_cast<float>(ev.volume);
    AudioClipHandle clip;
    switch (ev.kind) {
        case FootstepEvent::Kind::Step: {
            std::uniform_int_distribution<int> pick(0, kStepVariants - 2);
            int n = pick(rng_);
            if (n >= lastVariant_) ++n;   // never the same step twice running
            lastVariant_ = n;
            clip = steps_[g][n];
            pp.volume *= 0.55f;
            break;
        }
        case FootstepEvent::Kind::Jump: clip = jumps_[g]; pp.volume *= 0.6f; break;
        case FootstepEvent::Kind::Land:
            clip = lands_[g][ev.heavy < 0.33 ? 0 : (ev.heavy < 0.75 ? 1 : 2)];
            pp.pitch *= static_cast<float>(1.0 - 0.1 * ev.heavy);
            break;
        default: return;
    }
    ctx.audio.playAt(clip, feet, 20.0, pp);
    if (ev.kind == FootstepEvent::Kind::Step && swishLevel_ > 0.02) {
        AudioPlayParams sp;
        sp.bus = AudioBus::Sfx;
        sp.pitch = static_cast<float>(std::uniform_real_distribution<double>(0.92, 1.08)(rng_));
        sp.volume = static_cast<float>(0.4 * swishLevel_ * std::clamp(horiz / 2.5, 0.3, 1.2));
        ctx.audio.playAt(swishes_[std::max(0, lastVariant_) % kStepVariants], feet + Vec3(0, 0.4, 0), 20.0, sp);
    }
    static const bool log = std::getenv("RT_FOOTSTEP_LOG") != nullptr;   // RT_FOOTSTEP_LOG=1: every sound, and what it stood on
    if (log)
        LOG_INFO << "[footsteps] " << (ev.kind == FootstepEvent::Kind::Step ? "step" : ev.kind == FootstepEvent::Kind::Jump ? "jump" : "land")
                 << " on " << sfx::groundName(ground) << " (tag " << int(surface) << ") at (" << feet.x << ", " << feet.y << ", " << feet.z
                 << ") vol " << pp.volume << (ev.kind == FootstepEvent::Kind::Land ? " heavy " + std::to_string(ev.heavy) : std::string())
                 << " grass " << swishLevel_;
}

}  // namespace engine
