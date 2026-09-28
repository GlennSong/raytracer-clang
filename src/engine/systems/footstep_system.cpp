#include "footstep_system.h"

#include "physics_system.h"
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
    rustle_ = makeClip(ctx.audio, sfx::grassRustle(rate, 3), rate);
    LOG_INFO << "[footsteps] " << kGrounds * (kStepVariants + 1 + kLandLevels) + 1 << " clips synthesized in "
             << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() << " ms";
}

void FootstepSystem::onStop(FrameContext& ctx) {
    if (rustleVoice_.valid()) ctx.audio.stop(rustleVoice_);
    rustleVoice_ = AudioVoiceHandle{};
    rustleLevel_ = 0;
}

double FootstepSystem::groundHeight(const TerrainLodConfig& cfg, double x, double z) {
    if (cfg.baked) return cfg.baked->height(x, z);
    if (!noise_ || noiseSeed_ != cfg.seed) { noise_ = std::make_unique<Noise>(cfg.seed); noiseSeed_ = cfg.seed; }
    return terrainHeight(cfg.params, *noise_, x, z);
}

sfx::Ground FootstepSystem::groundAt(World& world, uint8_t surface, double x, double y, double z) {
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
    auto fadeRustle = [&](double target) {
        const double k = 1.0 - std::exp(-dt / 0.15);
        rustleLevel_ += (target - rustleLevel_) * k;
        if (rustleLevel_ > 0.01 && !rustleVoice_.valid()) {
            AudioPlayParams pp; pp.loop = true; pp.volume = 0.0f; pp.bus = AudioBus::Sfx;
            rustleVoice_ = ctx.audio.play(rustle_, pp);
        }
        if (rustleVoice_.valid()) {
            ctx.audio.setVoiceVolume(rustleVoice_, static_cast<float>(0.45 * rustleLevel_));
            if (rustleLevel_ < 0.003) { ctx.audio.stop(rustleVoice_); rustleVoice_ = AudioVoiceHandle{}; }
        }
    };
    if (!found || ctx.world.has<InVehicle>(player)) { fadeRustle(0.0); tracker_ = FootstepTracker{}; haveLastFeet_ = false; return; }
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
    const FootstepEvent ev = tracker_.update(onGround, horiz, v.y, dt, cc.halfHeight < 0.3);

    // #64: tall grass brushing the legs, while walking through it
    double grassTarget = 0.0;
    if (onGround && horiz > 0.3) {
        ctx.world.each<GrassField>([&](Entity, GrassField& g) {
            if (grassTarget > 0 || !g.density || !g.ground) return;
            const double gy = g.ground(feet.x, feet.z);
            grassTarget = std::clamp(g.density(feet.x, feet.z, gy, 1.0), 0.0, 1.0) * std::clamp(horiz / 2.5, 0.0, 1.2);
        });
        // only on the ground itself: a road or a floor standing over the grass field is not grass
        const uint8_t s = pw.bodySurface(pw.characterGroundBody(cc.characterId));
        if (s != static_cast<uint8_t>(ColliderSurface::Terrain) && s != static_cast<uint8_t>(ColliderSurface::Grass)) grassTarget = 0;
    }
    fadeRustle(grassTarget);

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
    static const bool log = std::getenv("RT_FOOTSTEP_LOG") != nullptr;   // RT_FOOTSTEP_LOG=1: every sound, and what it stood on
    if (log)
        LOG_INFO << "[footsteps] " << (ev.kind == FootstepEvent::Kind::Step ? "step" : ev.kind == FootstepEvent::Kind::Jump ? "jump" : "land")
                 << " on " << sfx::groundName(ground) << " (tag " << int(surface) << ") at (" << feet.x << ", " << feet.y << ", " << feet.z
                 << ") vol " << pp.volume << (ev.kind == FootstepEvent::Kind::Land ? " heavy " + std::to_string(ev.heavy) : std::string())
                 << " grass " << rustleLevel_;
}

}  // namespace engine
