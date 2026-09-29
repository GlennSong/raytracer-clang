#include "water_ambience_system.h"

#include "../audio/sfx.h"
#include "../components.h"
#include "../procgen/hydrology.h"
#include "../world.h"
#include "../../log.h"

#include <cstdlib>

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
constexpr int kDirs = 16;
const double kRadii[] = {6.0, 16.0, 32.0, 56.0, 90.0, 130.0, 170.0};
constexpr int kRings = sizeof(kRadii) / sizeof(kRadii[0]);
constexpr int kPerFrame = 16;   // one ring's worth of ground samples a frame: a whole sweep in ~7 frames
}  // namespace

void WaterAmbienceSystem::onStart(FrameContext& ctx) {
    ready_ = ctx.audio.ready();
    if (!ready_) return;
    const uint32_t rate = ctx.audio.sampleRate();
    const std::vector<float> s = sfx::surf(rate, 5), r = sfx::river(rate, 9, 0.1), big = sfx::river(rate, 11, 0.9);
    surfClip_ = ctx.audio.createClip(s.data(), s.size(), 1, rate);
    riverClip_ = ctx.audio.createClip(r.data(), r.size(), 1, rate);
    bigRiverClip_ = ctx.audio.createClip(big.data(), big.size(), 1, rate);
    const std::vector<float> lapPcm = sfx::lap(rate, 17);
    lapClip_ = ctx.audio.createClip(lapPcm.data(), lapPcm.size(), 1, rate);
    const std::vector<float> under = sfx::underwaterRiver(rate, 13);
    underClip_ = ctx.audio.createClip(under.data(), under.size(), 1, rate);
}

void WaterAmbienceSystem::onStop(FrameContext& ctx) {
    if (surfVoice_.valid()) ctx.audio.stop(surfVoice_);
    if (riverVoice_.valid()) ctx.audio.stop(riverVoice_);
    if (bigRiverVoice_.valid()) ctx.audio.stop(bigRiverVoice_);
    if (underVoice_.valid()) ctx.audio.stop(underVoice_);
    if (lapVoice_.valid()) ctx.audio.stop(lapVoice_);
    lapVoice_ = AudioVoiceHandle{};
    haveLap_ = lapFound_ = false;
    lapGain_ = 0;
    underVoice_ = AudioVoiceHandle{};
    underGain_ = 0;
    surfVoice_ = riverVoice_ = bigRiverVoice_ = AudioVoiceHandle{};
    haveSurf_ = haveRiver_ = haveBigRiver_ = false;
    surfGain_ = riverGain_ = bigRiverGain_ = 0;
}

void WaterAmbienceSystem::update(FrameContext& ctx) {
    if (!ready_) return;
    const TerrainLodConfig* cfg = nullptr;
    ctx.world.each<TerrainLodConfig>([&](Entity, TerrainLodConfig& c) { if (!cfg) cfg = &c; });
    if (!cfg) return;
    // the listener: the player on foot or driving, else the camera
    Vec3 at = ctx.view.camera.position;
    ctx.world.each<Transform, ControlledBy>([&](Entity, Transform& t, ControlledBy&) { at = t.position; });
    const Real dt = std::min<Real>(ctx.frameDelta, 0.1);
    const bool under = ctx.view.lighting.underwater.active;
    const double duck = under ? 0.1 : 1.0;   // the surface world, heard through the water: nearly gone
    {   // the muffled bubbling all around, while under
        underGain_ += ((under ? 0.7 : 0.0) - underGain_) * (1.0 - std::exp(-dt / 0.25));
        if (underGain_ > 0.01 && !underVoice_.valid()) {
            AudioPlayParams pp; pp.loop = true; pp.volume = 0.0f; pp.bus = AudioBus::Ambient;
            underVoice_ = ctx.audio.play(underClip_, pp);
        }
        if (underVoice_.valid()) {
            ctx.audio.setVoiceVolume(underVoice_, static_cast<float>(underGain_));
            if (underGain_ < 0.005) { ctx.audio.stop(underVoice_); underVoice_ = AudioVoiceHandle{}; }
        }
    }
    const double glide = 1.0 - std::exp(-dt / 0.6);

    auto drive = [&](AudioClipHandle clip, AudioVoiceHandle& voice, Vec3& pos, bool& have, double& gain,
                     bool found, const Vec3& target, double range, double level) {
        if (found) { pos = have ? pos + (target - pos) * glide : target; have = true; }
        gain += ((found ? level : 0.0) - gain) * (1.0 - std::exp(-dt / 0.8));
        if (gain > 0.01 && !voice.valid() && have) {
            AudioPlayParams pp; pp.loop = true; pp.volume = 0.0f; pp.bus = AudioBus::Ambient;
            voice = ctx.audio.playAt(clip, pos, range, pp);
        }
        if (voice.valid()) {
            ctx.audio.setVoicePosition(voice, pos);
            ctx.audio.setVoiceVolume(voice, static_cast<float>(gain));
            if (gain < 0.005) { ctx.audio.stop(voice); voice = AudioVoiceHandle{}; }
        }
    };

    // THE SEA: sweep a ring pattern round the listener, a slice a frame, for the ocean's cells.
    const Sea* seaMap = nullptr;
    ctx.world.each<Sea>([&](Entity, Sea& s) { if (!seaMap) seaMap = &s; });
    if (seaMap) {
        const double sea = seaMap->level;
        if (sweep_ == 0) { sweepCentre_ = at; bestD2_ = 1e30; }
        for (int k = 0; k < kPerFrame && sweep_ < kRings * kDirs; ++k, ++sweep_) {
            const double r = kRadii[sweep_ / kDirs];
            const double a = 6.283185307179586 * ((sweep_ % kDirs) + 0.5 * ((sweep_ / kDirs) % 2)) / kDirs;
            const double x = sweepCentre_.x + r * std::cos(a), z = sweepCentre_.z + r * std::sin(a);
            if (seaMap->contains(x, z) && r * r < bestD2_) { bestD2_ = r * r; best_ = Vec3(x, sea, z); }
        }
        if (sweep_ >= kRings * kDirs) {
            sweep_ = 0;
            // standing in the sea, the waves are all round: put them right here
            const bool inSea = seaMap->contains(sweepCentre_.x, sweepCentre_.z);
            surfFound_ = inSea || bestD2_ < 1e29;
            surfTarget_ = inSea ? Vec3(sweepCentre_.x, sea, sweepCentre_.z) : best_;
            // high on a cliff the surf is far below: thin it with height above the water
            const double above = std::max(0.0, at.y - sea);
            surfLevel_ = std::clamp(1.0 - above / 120.0, 0.15, 1.0);   // #87: the surf a notch louder
        }
        drive(surfClip_, surfVoice_, surfAt_, haveSurf_, surfGain_, surfFound_, surfTarget_, kSurfRange, surfLevel_ * duck);
    }

    // LAKES: the nearest lake edge, swept like the sea (a lake is still water: lapping, quietly)
    if (const Hydrology* hy = cfg->params.hydro.get(); hy && !hy->lakes().empty()) {
        constexpr int kLapRings = 5;
        const double lapRadii[kLapRings] = {4.0, 10.0, 20.0, 35.0, 55.0};
        if (lapSweep_ == 0) { lapCentre_ = at; lapBestD2_ = 1e30; }
        for (int k = 0; k < kPerFrame && lapSweep_ < kLapRings * kDirs; ++k, ++lapSweep_) {
            const double r = lapRadii[lapSweep_ / kDirs];
            const double a = 6.283185307179586 * ((lapSweep_ % kDirs) + 0.5 * ((lapSweep_ / kDirs) % 2)) / kDirs;
            const double x = lapCentre_.x + r * std::cos(a), z = lapCentre_.z + r * std::sin(a);
            if (r * r < lapBestD2_ && hy->inLake(x, z)) {
                const double lv = hy->lakeLevelAt(x, z);
                lapBestD2_ = r * r;
                lapBest_ = Vec3(x, std::isfinite(lv) ? lv : at.y, z);
            }
        }
        if (lapSweep_ >= kLapRings * kDirs) {
            lapSweep_ = 0;
            lapFound_ = lapBestD2_ < 1e29 || hy->inLake(lapCentre_.x, lapCentre_.z);
            if (lapBestD2_ < 1e29) lapTarget_ = lapBest_;
            else if (lapFound_) lapTarget_ = Vec3(lapCentre_.x, at.y, lapCentre_.z);
        }
        drive(lapClip_, lapVoice_, lapAt_, haveLap_, lapGain_, lapFound_, lapTarget_, 45.0, 0.35 * duck);
    }

    // RIVERS: the nearest bank, found through the distance field's slope.
    if (const Hydrology* hydro = cfg->params.hydro.get()) {
        static bool listed = false;
        if (!listed && std::getenv("RT_WATER_LOG")) {
            listed = true;
            for (const River& r : hydro->rivers())
                if (!r.nodes.empty()) {
                    const RiverNode& m = r.nodes[r.nodes.size() / 2];
                    LOG_INFO << "[water] river of " << r.nodes.size() << " nodes, middle (" << m.p.x << ", " << m.p.y << ") level " << m.level << " width " << m.width;
                }
        }
        double level = 0;
        const double d = hydro->distanceToRiver(at.x, at.z, kRiverRange, &level);
        const bool found = d < kRiverRange && std::isfinite(level);
        Vec3 target = at;
        if (found) {
            const double e = 2.0;
            const double gx = hydro->distanceToRiver(at.x + e, at.z, kRiverRange) - hydro->distanceToRiver(at.x - e, at.z, kRiverRange);
            const double gz = hydro->distanceToRiver(at.x, at.z + e, kRiverRange) - hydro->distanceToRiver(at.x, at.z - e, kRiverRange);
            const double gl = std::sqrt(gx * gx + gz * gz);
            const double step = std::max(0.0, d);
            target = gl > 1e-6 ? Vec3(at.x - gx / gl * step, level, at.z - gz / gl * step) : Vec3(at.x, level, at.z);
        }
        // how big the nearest river is: a narrow stream babbles, a broad one roars (#87)
        if (found && (riverSizeTimer_ -= dt) <= 0) {
            riverSizeTimer_ = 0.5;
            double best = 1e30, width = 20.0;
            for (const River& r : hydro->rivers())
                for (const RiverNode& n : r.nodes) {
                    const double dx = n.p.x - at.x, dz = n.p.y - at.z, d2 = dx * dx + dz * dz;
                    if (d2 < best) { best = d2; width = n.width; }
                }
            riverSize_ = std::clamp((width - 8.0) / 27.0, 0.0, 1.0);   // 8 m a stream .. 35 m a broad river
        }
        // quieter than the surf: a river murmurs (Glenn: "too intense")
        drive(riverClip_, riverVoice_, riverAt_, haveRiver_, riverGain_, found, target, kRiverRange, 0.32 * (1.0 - riverSize_) * duck);   // a narrow river: quieter
        drive(bigRiverClip_, bigRiverVoice_, bigRiverAt_, haveBigRiver_, bigRiverGain_, found, target, kRiverRange, 0.6 * riverSize_ * duck);
    }
    static const bool log = std::getenv("RT_WATER_LOG") != nullptr;   // RT_WATER_LOG=1: where each voice sits, twice a second
    static double since = 0;
    if (log && (since += dt) > 0.5) {
        since = 0;
        LOG_INFO << "[water] listener (" << at.x << ", " << at.y << ", " << at.z << ") surf " << (surfVoice_.valid() ? "on" : "off")
                 << " gain " << surfGain_ << " at (" << surfAt_.x << ", " << surfAt_.z << ") | river " << (riverVoice_.valid() ? "on" : "off")
                 << " gain " << riverGain_ << " at (" << riverAt_.x << ", " << riverAt_.z << ")";
    }
}

}  // namespace engine
