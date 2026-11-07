#include "engine_voices.h"

#include "sfx.h"
#include "../vehicle_audio.h"

#include <algorithm>
#include <cstddef>

namespace engine {

void EngineVoicePool::update(AudioEngine& audio, std::vector<Engine>& heard, Real simTime, Real frameDelta) {
    // Nearest first: when there are more running cars than voices, the ones you can actually pick out win the slots.
    std::sort(heard.begin(), heard.end(), [](const Engine& a, const Engine& b) { return a.dist2 < b.dist2; });
    if (heard.size() > static_cast<std::size_t>(maxVoices_)) heard.resize(static_cast<std::size_t>(maxVoices_));

    for (Voice& v : voices_) v.wanted = false;

    const float dt = std::max(1e-4f, static_cast<float>(frameDelta));
    for (const Engine& c : heard) {
        Voice* slot = nullptr;
        for (Voice& v : voices_)
            if (v.key == c.key) { slot = &v; break; }
        if (!slot) {
            if (!clip_.valid()) {
                std::vector<float> pcm = sfx::engine(audio.sampleRate(), seed_);
                clip_ = audio.createClip(pcm.data(), pcm.size(), 1, audio.sampleRate());
            }
            if (!clip_.valid()) return;
            AudioPlayParams params;
            params.loop = true;
            params.volume = 0.0f;   // ramped in below -- no click as a car arrives
            Voice v;
            v.key = c.key;
            v.prevSpeed = c.speed;
            v.voice = audio.playAt(clip_, c.pos, range_, params);
            if (!v.voice.valid()) continue;
            voices_.push_back(v);
            slot = &voices_.back();
        }
        slot->wanted = true;
        Real throttle = c.throttle;
        if (c.estimate) {
            const Real accel = (c.speed - slot->prevSpeed) / dt;
            throttle = std::min(Real(1), std::max(Real(0.15), accel / Real(3.0)));
        }
        slot->prevSpeed = c.speed;
        const EngineSound s = engineSoundFor(c.speed, throttle, true);
        const EngineRough rough = engineRough(simTime, c.phase, c.health, c.roughBelow, c.roughWobble, c.roughMisfire);
        slot->gain += (s.gain * rough.gain - slot->gain) * std::min(1.0f, (rough.gain < 1.0f ? 40.0f : 8.0f) * dt);
        const float wobble = engineWobble(simTime, c.phase, engineRevFraction(c.speed));
        audio.setVoiceVolume(slot->voice, slot->gain);
        audio.setVoicePitch(slot->voice, s.pitch * wobble * rough.pitch);
        audio.setVoicePosition(slot->voice, c.pos);
    }

    // Fade out and reclaim: cars that drove away, lost their driver, or lost their slot to a nearer one. A voice the
    // mixer already dropped (level swap -> stopAll) is forgotten so the car can be voiced again.
    for (std::size_t i = voices_.size(); i-- > 0;) {
        Voice& v = voices_[i];
        if (!audio.isPlaying(v.voice)) {
            voices_.erase(voices_.begin() + static_cast<std::ptrdiff_t>(i));
            continue;
        }
        if (v.wanted) continue;
        v.gain += (0.0f - v.gain) * std::min(1.0f, 6.0f * dt);
        if (v.gain < 0.01f) {
            audio.stop(v.voice);
            voices_.erase(voices_.begin() + static_cast<std::ptrdiff_t>(i));
        } else {
            audio.setVoiceVolume(v.voice, v.gain);
        }
    }
}

void EngineVoicePool::stopAll(AudioEngine& audio) {
    if (audio.ready())
        for (Voice& v : voices_) audio.stop(v.voice);
    voices_.clear();
}

}  // namespace engine
