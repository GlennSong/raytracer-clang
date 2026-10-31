#ifndef RAYTRACER_ENGINE_AUDIO_ENGINE_VOICES_H
#define RAYTRACER_ENGINE_AUDIO_ENGINE_VOICES_H

#include "audio_engine.h"
#include "../../rt_math.h"

#include <cstdint>
#include <vector>

namespace engine {

// THE ENGINE VOICE POOL (ADR-0285, #117 item 28): running cars, heard. Each frame its owner hands it the engines in
// earshot; the nearest `maxVoices` get the mixer's slots, the rest are silence you would not have picked out anyway.
// A voice follows its car across frames (a car keeps its note as it passes), its gain eases in and fades out rather
// than snapping, its pitch tracks the core directly (a gear change reads as the shift it is), and each car hunts its
// revs on its own phase (forty cars holding one exact note is a chord, not a street). One procedural loop
// (sfx::engine, ADR-0071), synthesized once per pool and pitch-shifted per car; the note itself is
// vehicle_audio.h's (engineSoundFor). The physics cars (VehicleSystem) and the city's traffic (citysim's
// CityTrafficAudioSystem) each keep a pool; they differ only in where their engines come from.
class EngineVoicePool {
public:
    // One engine in earshot this frame.
    struct Engine {
        uint64_t key = 0;     // stable per car across frames
        Vec3 pos;
        Real speed = 0;       // m/s
        // The pedal, 0..1; kEstimate: the car has none (a planner car), so the pool reads the load from its
        // change of speed -- a car gaining speed is a car working: "pulling away from the lights" vs "coasting".
        Real throttle = 0;
        Real phase = 0;       // its rev hunt's phase
        Real dist2 = 0;       // to the ear
    };
    static constexpr Real kEstimate = -1;

    EngineVoicePool(int maxVoices, Real range, int clipSeed) : maxVoices_(maxVoices), range_(range), seed_(clipSeed) {}

    // Voice the nearest of `heard` (reordered), and fade and reclaim the rest.
    void update(AudioEngine& audio, std::vector<Engine>& heard, Real simTime, Real frameDelta);
    void stopAll(AudioEngine& audio);
    Real range() const { return range_; }

private:
    struct Voice {
        uint64_t key = 0;
        AudioVoiceHandle voice;
        float gain = 0.0f;      // ramped toward the core's target
        Real prevSpeed = 0;     // for the load estimate
        bool wanted = false;    // survives this frame's cull?
    };
    int maxVoices_;
    Real range_;
    int seed_;
    AudioClipHandle clip_;
    std::vector<Voice> voices_;
};

}  // namespace engine

#endif
