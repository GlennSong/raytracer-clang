#include "city_traffic_audio.h"

#include "../../engine/audio/sfx.h"
#include "../../engine/vehicle_audio.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace citysim {

using engine::EngineSound;
using engine::Vec2;
using engine::Vec3;

void CityTrafficAudioSystem::update(engine::FrameContext& ctx) {
    if (!ctx.audio.ready() || !city_.built()) return;

    const std::vector<Agent>& agents = city_.sim().agents();
    const Vec3 ear = ctx.view.camera.position;

    struct Candidate {
        int agent;
        Vec3 pos;
        engine::Real speed;
        engine::Real dist2;
    };
    std::vector<Candidate> heard;
    for (std::size_t i = 0; i < agents.size(); ++i) {
        const Agent& a = agents[i];
        if (a.mode != Agent::Mode::Driver) continue;
        if (a.far()) continue;        // no body, no note
        if (a.released) continue;     // the player's car — VehicleSystem's voice
        Vec3 pos;
        Vec2 heading;
        if (!city_.agentWorldPose(static_cast<int>(i), pos, heading)) continue;
        const engine::Real d2 = (pos - ear).lengthSquared();
        if (d2 > kAudibleRange * kAudibleRange) continue;
        heard.push_back({static_cast<int>(i), pos, a.speed, d2});
    }
    std::sort(heard.begin(), heard.end(),
              [](const Candidate& x, const Candidate& y) {
                  return x.dist2 < y.dist2;
              });
    if (heard.size() > static_cast<std::size_t>(kMaxVoices))
        heard.resize(static_cast<std::size_t>(kMaxVoices));

    for (Voice& v : voices_) v.wanted = false;

    const float dt = std::max(1e-4f, static_cast<float>(ctx.frameDelta));
    for (const Candidate& c : heard) {
        Voice* slot = nullptr;
        for (Voice& v : voices_)
            if (v.agent == c.agent) { slot = &v; break; }
        if (!slot) {
            if (!clip_.valid()) {
                // Seed 2: the traffic fleet's rasp differs from the player
                // car's (seed 1), so your own engine still reads as yours.
                std::vector<float> pcm = engine::sfx::engine(
                    ctx.audio.sampleRate(), 2);
                clip_ = ctx.audio.createClip(pcm.data(), pcm.size(), 1,
                                             ctx.audio.sampleRate());
            }
            if (!clip_.valid()) return;
            Voice v;
            v.agent = c.agent;
            v.prevSpeed = c.speed;
            engine::AudioPlayParams params;
            params.loop = true;
            params.volume = 0.0f;   // ramped in — no click as a car arrives
            v.voice = ctx.audio.playAt(clip_, c.pos, kAudibleRange, params);
            if (!v.voice.valid()) continue;
            voices_.push_back(v);
            slot = &voices_.back();
        }
        slot->wanted = true;

        // LOAD from acceleration: the planner gives no pedal, but a car
        // gaining speed is a car working, and that is what the ear reads as
        // "pulling away from the lights" versus "coasting past".
        const engine::Real accel = (c.speed - slot->prevSpeed) / dt;
        slot->prevSpeed = c.speed;
        const engine::Real throttle =
            std::min(engine::Real(1),
                     std::max(engine::Real(0.15), accel / engine::Real(3.0)));

        const EngineSound s = engine::engineSoundFor(c.speed, throttle, true);
        slot->gain += (s.gain - slot->gain) * std::min(1.0f, 8.0f * dt);
        // Per-car rev hunt, phased by agent id: forty cars holding one exact
        // note is a chord, not a street.
        const float wobble = engine::engineWobble(
            ctx.clock.simulatedTime(),
            static_cast<engine::Real>(c.agent % 23) * engine::Real(0.29),
            engine::engineRevFraction(c.speed));
        ctx.audio.setVoiceVolume(slot->voice, slot->gain);
        ctx.audio.setVoicePitch(slot->voice, s.pitch * wobble);
        ctx.audio.setVoicePosition(slot->voice, c.pos);
    }

    for (std::size_t i = voices_.size(); i-- > 0;) {
        Voice& v = voices_[i];
        if (!ctx.audio.isPlaying(v.voice)) {
            voices_.erase(voices_.begin() + static_cast<std::ptrdiff_t>(i));
            continue;
        }
        if (v.wanted) continue;
        v.gain += (0.0f - v.gain) * std::min(1.0f, 6.0f * dt);
        if (v.gain < 0.01f) {
            ctx.audio.stop(v.voice);
            voices_.erase(voices_.begin() + static_cast<std::ptrdiff_t>(i));
        } else {
            ctx.audio.setVoiceVolume(v.voice, v.gain);
        }
    }
}

void CityTrafficAudioSystem::onStop(engine::FrameContext& ctx) {
    if (ctx.audio.ready())
        for (Voice& v : voices_) ctx.audio.stop(v.voice);
    voices_.clear();
}

}  // namespace citysim
