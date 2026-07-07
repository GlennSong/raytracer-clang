// Audio demo (ADR-0069), headless: a 12-second spatial scene mixed by the real
// AudioEngine in Manual mode and written to audio_demo.wav — listen to it.
//
// What you hear, and which system produces it:
//   0s-12s  a synthesized hum ORBITING the listener — an ECS AudioSource whose
//           entity Transform circles the AudioListener; AudioSystem::step
//           feeds the moving position to the voice every tick (pan sweeps
//           left -> front -> right -> behind, ADR-0069)
//   2s-9s   a noise "vehicle" flying past on the left — distance attenuation
//           swells in and fades out (linear falloff to its range)
//   every 1.5s  a bell ping — a PlaySound event published on the EventBus
//           (ADR-0066) and enqueued for the frame dispatch, exactly how
//           gameplay systems fire cues; alternates left / right / far ahead.
//           The ping clip round-trips through a real WAV file (decode path);
//           the hum and whoosh are procedurally synthesized PCM (createClip).
//
// Build: the `audio_demo` CMake target (needs RT_ENABLE_AUDIO=ON, the default).
#include "engine/audio/audio_engine.h"
#include "engine/components.h"
#include "engine/event_bus.h"
#include "engine/systems/audio_system.h"
#include "engine/world.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <random>
#include <vector>

using namespace engine;

namespace {

constexpr uint32_t RATE = 48000;

// Two-partial hum with integer cycles per second, so the 1 s loop is seamless.
std::vector<float> humClip() {
    std::vector<float> frames(RATE);
    for (uint32_t i = 0; i < RATE; i++) {
        double t = static_cast<double>(i) / RATE;
        frames[i] = static_cast<float>(0.30 * std::sin(2.0 * M_PI * 110.0 * t) +
                                       0.15 * std::sin(2.0 * M_PI * 165.0 * t));
    }
    return frames;
}

// Low-passed white noise, loopable: a wind/engine "whoosh" bed.
std::vector<float> whooshClip() {
    std::vector<float> frames(RATE * 2);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> noise(-1.0f, 1.0f);
    float lp = 0.0f;
    for (auto& f : frames) {
        lp += 0.08f * (noise(rng) - lp);   // one-pole low pass
        f = lp * 1.4f;
    }
    // Crossfade the tail into the head so the loop seam is silent.
    const size_t fade = RATE / 10;
    for (size_t i = 0; i < fade; i++) {
        float t = static_cast<float>(i) / fade;
        frames[i] = frames[i] * t + frames[frames.size() - fade + i] * (1.0f - t);
    }
    return frames;
}

// A decaying two-partial bell strike.
std::vector<float> pingClip() {
    std::vector<float> frames(RATE / 2);
    for (size_t i = 0; i < frames.size(); i++) {
        double t = static_cast<double>(i) / RATE;
        double envelope = std::exp(-t * 9.0);
        frames[i] = static_cast<float>(
            envelope * (0.5 * std::sin(2.0 * M_PI * 880.0 * t) +
                        0.2 * std::sin(2.0 * M_PI * 1320.0 * t)));
    }
    return frames;
}

bool writeWav16(const std::string& path, const std::vector<float>& interleaved,
                uint32_t sampleRate, uint16_t channels) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    auto w32 = [&](uint32_t v) { out.write(reinterpret_cast<char*>(&v), 4); };
    auto w16 = [&](uint16_t v) { out.write(reinterpret_cast<char*>(&v), 2); };
    uint32_t dataBytes = static_cast<uint32_t>(interleaved.size() * 2);
    out.write("RIFF", 4);
    w32(36 + dataBytes);
    out.write("WAVE", 4);
    out.write("fmt ", 4);
    w32(16);
    w16(1);
    w16(channels);
    w32(sampleRate);
    w32(sampleRate * channels * 2);
    w16(static_cast<uint16_t>(channels * 2));
    w16(16);
    out.write("data", 4);
    w32(dataBytes);
    for (float f : interleaved) {
        auto s = static_cast<int16_t>(std::max(-1.0f, std::min(f, 1.0f)) * 32767);
        w16(static_cast<uint16_t>(s));
    }
    return true;
}

}  // namespace

int main() {
    AudioEngine audio;
    if (!audio.initialize(AudioBackendMode::Manual)) {
        std::fprintf(stderr, "audio init failed (built with RT_ENABLE_AUDIO?)\n");
        return 1;
    }

    World world;
    EventBus events;
    AudioSystem system;

    // The bus -> audio chain, wired the way AudioSystem::onStart does it.
    events.subscribe<PlaySound>(
        [&](const PlaySound& cue) { system.onPlaySound(cue, audio); });

    // The ears: an AudioListener entity at the origin, facing -Z.
    Entity listener = world.create();
    world.add<AudioListener>(listener, {});
    world.add<Transform>(listener, {});

    // The orbiting hum: procedural PCM straight into the source's clipHandle —
    // the procgen path, no file involved.
    auto hum = humClip();
    Entity orbiter = world.create();
    Transform orbiterPose;
    orbiterPose.position = Vec3(0, 0, -6);
    world.add<Transform>(orbiter, orbiterPose);
    AudioSource humSource;
    humSource.clipHandle = audio.createClip(hum.data(), hum.size(), 1, RATE);
    humSource.loop = true;
    humSource.volume = 0.9f;
    humSource.range = 30.0;
    world.add<AudioSource>(orbiter, humSource);

    // The fly-by: enters at 2 s from far left, passes close, exits far right.
    auto whoosh = whooshClip();
    Entity vehicle = world.create();
    Transform vehiclePose;
    vehiclePose.position = Vec3(-80, 0, -10);   // far outside its range at start
    world.add<Transform>(vehicle, vehiclePose);
    AudioSource whooshSource;
    whooshSource.clipHandle =
        audio.createClip(whoosh.data(), whoosh.size(), 1, RATE);
    whooshSource.loop = true;
    whooshSource.volume = 1.0f;
    whooshSource.range = 30.0;
    world.add<AudioSource>(vehicle, whooshSource);

    // The ping rides a real WAV file, exercising the decode path end to end.
    const std::string pingPath = "audio_demo_ping.wav";
    writeWav16(pingPath, pingClip(), RATE, 1);

    // --- run the scene at 60 ticks/s for 12 s, pumping the mix --------------
    const double dt = 1.0 / 60.0;
    const int ticks = 12 * 60;
    const uint64_t framesPerTick = RATE / 60;
    std::vector<float> mix;
    mix.reserve(ticks * framesPerTick * 2);
    std::vector<float> block(framesPerTick * 2);

    CameraState unusedCamera;   // listener entity exists; camera is fallback
    int pingCount = 0;
    for (int tick = 0; tick < ticks; tick++) {
        double t = tick * dt;

        // Orbit: one lap every 6 s, radius 6, starting dead ahead.
        double theta = 2.0 * M_PI * t / 6.0;
        world.get<Transform>(orbiter)->position =
            Vec3(6.0 * std::sin(theta), 0, -6.0 * std::cos(theta));

        // Fly-by between 2 s and 9 s: left to right, closest at ~5.5 s.
        double u = (t - 2.0) / 7.0;
        world.get<Transform>(vehicle)->position =
            Vec3(-80.0 + 160.0 * u, 0, -8.0);

        // A gameplay system firing a cue: enqueue on the bus, dispatch with
        // the frame, alternating left / right / far ahead (the far one is
        // audibly quieter — attenuation at work).
        if (tick % 90 == 0 && tick > 0) {
            PlaySound cue;
            cue.clip = pingPath;
            cue.spatial = true;
            static const Vec3 spots[3] = {Vec3(-5, 0, -2), Vec3(5, 0, -2),
                                          Vec3(0, 0, -22)};
            cue.position = spots[pingCount++ % 3];
            cue.range = 30.0;
            events.enqueue(cue);
        }
        events.dispatchQueued();

        system.syncListener(world, audio, unusedCamera);
        system.step(world, audio);

        // Fade the master out over the final second (the mix knob, live).
        if (t > 11.0)
            audio.setMasterVolume(static_cast<float>(std::max(0.0, 12.0 - t)));

        uint64_t rendered = audio.readFrames(block.data(), framesPerTick);
        mix.insert(mix.end(), block.begin(),
                   block.begin() + rendered * 2);
    }

    writeWav16("audio_demo.wav", mix, RATE, 2);
    std::printf("Wrote audio_demo.wav: %.1f s stereo, %zu clips, %zu voices "
                "live at end, %d pings fired\n",
                mix.size() / 2.0 / RATE, audio.clipCount(),
                audio.activeVoiceCount(), pingCount);
    return 0;
}
