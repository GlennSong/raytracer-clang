#include "test_framework.h"
#include "../src/engine/audio/audio_engine.h"
#include "../src/engine/components.h"
#include "../src/engine/systems/audio_system.h"
#include "../src/engine/world.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <vector>

using namespace engine;

namespace {

// A short 440 Hz sine, mono, engine-rate.
std::vector<float> sineClip(uint32_t sampleRate, double seconds,
                            double hz = 440.0) {
    auto frameCount = static_cast<uint64_t>(sampleRate * seconds);
    std::vector<float> frames(frameCount);
    for (uint64_t i = 0; i < frameCount; i++)
        frames[i] = static_cast<float>(
            0.5 * std::sin(2.0 * M_PI * hz * i / sampleRate));
    return frames;
}

// Pump the manual-mode mix and report per-channel mean absolute amplitude.
struct StereoEnergy {
    double left = 0, right = 0;
    double total() const { return left + right; }
};
StereoEnergy pump(AudioEngine& audio, uint64_t frameCount) {
    std::vector<float> out(frameCount * 2);
    uint64_t rendered = audio.readFrames(out.data(), frameCount);
    StereoEnergy energy;
    for (uint64_t i = 0; i < rendered; i++) {
        energy.left += std::fabs(out[i * 2]);
        energy.right += std::fabs(out[i * 2 + 1]);
    }
    if (rendered > 0) {
        energy.left /= static_cast<double>(rendered);
        energy.right /= static_cast<double>(rendered);
    }
    return energy;
}

// Minimal PCM16 mono WAV writer, for the file-decode path.
bool writeWav(const std::string& path, const std::vector<float>& frames,
              uint32_t sampleRate) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    auto write32 = [&](uint32_t v) { out.write(reinterpret_cast<char*>(&v), 4); };
    auto write16 = [&](uint16_t v) { out.write(reinterpret_cast<char*>(&v), 2); };
    uint32_t dataBytes = static_cast<uint32_t>(frames.size() * 2);
    out.write("RIFF", 4);
    write32(36 + dataBytes);
    out.write("WAVE", 4);
    out.write("fmt ", 4);
    write32(16);
    write16(1);                    // PCM
    write16(1);                    // mono
    write32(sampleRate);
    write32(sampleRate * 2);       // byte rate
    write16(2);                    // block align
    write16(16);                   // bits per sample
    out.write("data", 4);
    write32(dataBytes);
    for (float f : frames) {
        auto s = static_cast<int16_t>(std::max(-1.0f, std::min(f, 1.0f)) * 32767);
        write16(static_cast<uint16_t>(s));
    }
    return true;
}

struct TempWav {
    std::string path = "test_audio_clip.wav";
    explicit TempWav(uint32_t sampleRate = 48000, double seconds = 0.05) {
        writeWav(path, sineClip(sampleRate, seconds), sampleRate);
    }
    ~TempWav() { std::remove(path.c_str()); }
};

}  // namespace

TEST_CASE(audio_engine_initializes_in_manual_mode) {
    AudioEngine audio;
    CHECK(!audio.ready());
    CHECK(audio.initialize(AudioBackendMode::Manual));
    CHECK(audio.ready());
    CHECK(audio.sampleRate() == 48000);
    CHECK(audio.backendMode() == AudioBackendMode::Manual);
    audio.shutdown();
    CHECK(!audio.ready());
}

TEST_CASE(audio_engine_rejects_bad_clip_data) {
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    std::vector<float> pcm = sineClip(48000, 0.01);
    CHECK(!audio.createClip(nullptr, 100, 1, 48000).valid());
    CHECK(!audio.createClip(pcm.data(), 0, 1, 48000).valid());
    CHECK(!audio.createClip(pcm.data(), pcm.size(), 0, 48000).valid());
    CHECK(!audio.play(AudioClipHandle{}).valid());
    CHECK(audio.clipCount() == 0);
    CHECK(audio.activeVoiceCount() == 0);
}

TEST_CASE(audio_engine_plays_a_clip_into_the_mix) {
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    auto pcm = sineClip(48000, 0.1);
    AudioClipHandle clip = audio.createClip(pcm.data(), pcm.size(), 1, 48000);
    CHECK(clip.valid());
    CHECK(audio.clipCount() == 1);

    // Silence before anything plays.
    CHECK_APPROX(pump(audio, 512).total(), 0.0, 1e-9);

    AudioVoiceHandle voice = audio.play(clip);
    CHECK(voice.valid());
    CHECK(audio.isPlaying(voice));
    CHECK(pump(audio, 1024).total() > 0.01);
}

TEST_CASE(audio_engine_one_shot_finishes_and_is_reclaimed) {
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    auto pcm = sineClip(48000, 0.02);   // 960 frames
    AudioClipHandle clip = audio.createClip(pcm.data(), pcm.size(), 1, 48000);
    AudioVoiceHandle voice = audio.play(clip);
    CHECK(audio.activeVoiceCount() == 1);

    pump(audio, 4096);   // read well past the clip's end
    audio.update();
    CHECK(!audio.isPlaying(voice));
    CHECK(audio.activeVoiceCount() == 0);
}

TEST_CASE(audio_engine_looping_voice_outlives_the_clip_length) {
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    auto pcm = sineClip(48000, 0.02);
    AudioClipHandle clip = audio.createClip(pcm.data(), pcm.size(), 1, 48000);
    AudioPlayParams params;
    params.loop = true;
    AudioVoiceHandle voice = audio.play(clip, params);

    pump(audio, 8192);   // several clip lengths
    audio.update();
    CHECK(audio.isPlaying(voice));
    CHECK(pump(audio, 1024).total() > 0.01);   // still audible

    audio.stop(voice);
    CHECK(!audio.isPlaying(voice));
    CHECK(audio.activeVoiceCount() == 0);
    CHECK_APPROX(pump(audio, 1024).total(), 0.0, 1e-9);
}

TEST_CASE(audio_engine_spatial_voice_pans_toward_its_side) {
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    // Listener at origin looking down -Z, so +X is the right ear.
    audio.setListener(Vec3(0, 0, 0), Vec3(0, 0, -1));
    auto pcm = sineClip(48000, 0.5);
    AudioClipHandle clip = audio.createClip(pcm.data(), pcm.size(), 1, 48000);

    AudioVoiceHandle left = audio.playAt(clip, Vec3(-5, 0, 0), 50.0);
    CHECK(left.valid());
    StereoEnergy energy = pump(audio, 2048);
    CHECK(energy.left > energy.right * 1.5);
    audio.stopAll();

    AudioVoiceHandle right = audio.playAt(clip, Vec3(5, 0, 0), 50.0);
    CHECK(right.valid());
    energy = pump(audio, 2048);
    CHECK(energy.right > energy.left * 1.5);
}

TEST_CASE(audio_engine_spatial_voice_attenuates_with_distance) {
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    audio.setListener(Vec3(0, 0, 0), Vec3(0, 0, -1));
    auto pcm = sineClip(48000, 0.5);
    AudioClipHandle clip = audio.createClip(pcm.data(), pcm.size(), 1, 48000);

    audio.playAt(clip, Vec3(0, 0, -2), 20.0);
    double nearEnergy = pump(audio, 2048).total();
    audio.stopAll();

    audio.playAt(clip, Vec3(0, 0, -18), 20.0);
    double farEnergy = pump(audio, 2048).total();
    audio.stopAll();

    audio.playAt(clip, Vec3(0, 0, -100), 20.0);   // beyond range: silent
    double beyondEnergy = pump(audio, 2048).total();

    CHECK(nearEnergy > farEnergy * 2.0);
    CHECK(beyondEnergy < nearEnergy * 0.01);
}

TEST_CASE(audio_engine_destroy_clip_stops_its_voices) {
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    auto pcm = sineClip(48000, 0.02);
    AudioClipHandle clip = audio.createClip(pcm.data(), pcm.size(), 1, 48000);
    AudioPlayParams params;
    params.loop = true;
    AudioVoiceHandle voice = audio.play(clip, params);
    CHECK(audio.isPlaying(voice));

    audio.destroyClip(clip);
    CHECK(!audio.isPlaying(voice));
    CHECK(audio.activeVoiceCount() == 0);
    CHECK(audio.clipCount() == 0);
    CHECK(!audio.play(clip).valid());   // stale clip handle: no new voice
}

TEST_CASE(audio_engine_loads_a_wav_file) {
    TempWav wav;
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    AudioClipHandle clip = audio.loadClip(wav.path);
    CHECK(clip.valid());
    CHECK(audio.clipCount() == 1);
    audio.play(clip);
    CHECK(pump(audio, 1024).total() > 0.01);
    CHECK(!audio.loadClip("no_such_file.wav").valid());
}

TEST_CASE(audio_engine_master_and_bus_volumes_scale_the_mix) {
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    auto pcm = sineClip(48000, 0.5);
    AudioClipHandle clip = audio.createClip(pcm.data(), pcm.size(), 1, 48000);

    AudioPlayParams params;
    params.loop = true;
    audio.play(clip, params);
    double fullEnergy = pump(audio, 2048).total();

    audio.setBusVolume(AudioBus::Sfx, 0.25f);
    double busEnergy = pump(audio, 2048).total();
    CHECK(busEnergy < fullEnergy * 0.5);
    CHECK_APPROX(audio.busVolume(AudioBus::Sfx), 0.25, 1e-6);

    audio.setBusVolume(AudioBus::Sfx, 1.0f);
    audio.setMasterVolume(0.0f);
    // The engine node smooths volume changes; skip the fade, then expect
    // silence.
    pump(audio, 4096);
    double mutedEnergy = pump(audio, 2048).total();
    CHECK(mutedEnergy < fullEnergy * 0.01);
}

TEST_CASE(audio_system_autoplays_sources_and_stops_dead_entities) {
    TempWav wav;
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    World world;
    AudioSystem system;

    Entity e = world.create();
    world.add<Transform>(e, {});
    AudioSource source;
    source.clip = wav.path;
    source.loop = true;
    world.add<AudioSource>(e, source);

    system.step(world, audio);
    CHECK(audio.activeVoiceCount() == 1);
    CHECK(system.cachedClipCount() == 1);
    CHECK(world.get<AudioSource>(e)->voice.valid());

    // A second step must not start a second voice for the same source.
    system.step(world, audio);
    CHECK(audio.activeVoiceCount() == 1);

    // Killing the entity stops its looping voice (no leak).
    world.destroy(e);
    system.step(world, audio);
    CHECK(audio.activeVoiceCount() == 0);
}

TEST_CASE(audio_system_trigger_restarts_and_missing_clip_disarms) {
    TempWav wav;
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    World world;
    AudioSystem system;

    // trigger-only source (no autoplay)
    Entity e = world.create();
    world.add<Transform>(e, {});
    AudioSource source;
    source.clip = wav.path;
    source.autoplay = false;
    world.add<AudioSource>(e, source);

    system.step(world, audio);
    CHECK(audio.activeVoiceCount() == 0);

    world.get<AudioSource>(e)->trigger = true;
    system.step(world, audio);
    CHECK(audio.activeVoiceCount() == 1);
    CHECK(!world.get<AudioSource>(e)->trigger);   // consumed

    // A source with a missing file disarms instead of retrying every frame.
    Entity bad = world.create();
    world.add<Transform>(bad, {});
    AudioSource badSource;
    badSource.clip = "no_such_file.wav";
    world.add<AudioSource>(bad, badSource);
    system.step(world, audio);
    CHECK(!world.get<AudioSource>(bad)->autoplay);
    CHECK(audio.activeVoiceCount() == 1);
}

TEST_CASE(audio_system_listener_follows_entity_then_camera) {
    TempWav wav;
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    World world;
    AudioSystem system;

    auto pcm = sineClip(48000, 0.5);
    AudioClipHandle clip = audio.createClip(pcm.data(), pcm.size(), 1, 48000);

    // Listener entity at +X of the source: sound should favor the LEFT ear
    // (entity faces -Z, so -X is its left, where the source sits).
    Entity listener = world.create();
    world.add<AudioListener>(listener, {});
    Transform t;
    t.position = Vec3(10, 0, 0);
    world.add<Transform>(listener, t);

    CameraState camera;   // parked far away, facing -Z
    camera.position = Vec3(-1000, 0, 0);
    camera.target = Vec3(-1000, 0, -1);

    system.syncListener(world, audio, camera);
    audio.playAt(clip, Vec3(0, 0, 0), 100.0);
    StereoEnergy viaEntity = pump(audio, 2048);
    CHECK(viaEntity.left > viaEntity.right * 1.5);
    audio.stopAll();

    // Without the listener entity, the camera pose drives the ears: parked
    // 1000 units out with a 100-unit range, the same source is silent.
    world.destroy(listener);
    system.syncListener(world, audio, camera);
    audio.playAt(clip, Vec3(0, 0, 0), 100.0);
    CHECK(pump(audio, 2048).total() < viaEntity.total() * 0.01);
}

TEST_CASE(audio_system_play_sound_event_fires_a_voice) {
    TempWav wav;
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    AudioSystem system;

    PlaySound cue;
    cue.clip = wav.path;
    system.onPlaySound(cue, audio);
    CHECK(audio.activeVoiceCount() == 1);
    CHECK(pump(audio, 1024).total() > 0.01);

    PlaySound missing;
    missing.clip = "no_such_file.wav";
    system.onPlaySound(missing, audio);   // must not crash or add a voice
    CHECK(audio.activeVoiceCount() == 1);
}

#include "../src/engine/audio/sfx.h"

TEST_CASE(sfx_generators_produce_bounded_deterministic_pcm) {
    auto shotA = sfx::gunshot(48000, 7);
    auto shotB = sfx::gunshot(48000, 7);
    auto shotC = sfx::gunshot(48000, 8);
    CHECK(!shotA.empty());
    CHECK(shotA == shotB);        // same seed -> identical (ADR-0021 ethos)
    CHECK(shotA != shotC);        // different seed -> different crack

    auto hit = sfx::impact(48000, 3);
    CHECK(!hit.empty());
    double energy = 0;
    for (float f : shotA) {
        CHECK(std::fabs(f) <= 1.0f);
        energy += std::fabs(f);
    }
    for (float f : hit) CHECK(std::fabs(f) <= 1.0f);
    CHECK(energy / shotA.size() > 0.005);   // audible, not near-silence
}

TEST_CASE(sfx_horn_is_loop_clean_and_seed_pitched) {
    auto a = sfx::horn(48000, 5);
    auto b = sfx::horn(48000, 5);
    auto c = sfx::horn(48000, 6);
    CHECK(!a.empty());
    CHECK(a == b);                 // deterministic
    CHECK(a != c);                 // seeds pick different horn pitches
    double energy = 0;
    for (float f : a) {
        CHECK(std::fabs(f) <= 1.0f);
        energy += std::fabs(f);
    }
    CHECK(energy / a.size() > 0.1);   // a steady tone, not a transient
    // LOOP-CLEAN (the horn's contract): the buffer is one exact period, so the
    // wrap seam a looping voice plays — last sample back to the first — is just
    // another sample step, no bigger than the steepest step inside the buffer.
    // The first sample is the period's zero phase (sines starting at 0).
    auto seamIsClean = [](const std::vector<float>& f) {
        float maxStep = 0.0f;
        for (size_t i = 1; i < f.size(); i++)
            maxStep = std::max(maxStep, std::fabs(f[i] - f[i - 1]));
        return std::fabs(f.front() - f.back()) <= maxStep * 1.5f + 1e-4f;
    };
    CHECK(std::fabs(a.front()) < 0.01f);
    CHECK(seamIsClean(a));
    // Odd sample rates (44.1k) must be loop-clean too — the cycle counts are
    // quantized to the buffer, not to a nominal duration.
    CHECK(seamIsClean(sfx::horn(44100, 5)));
}

TEST_CASE(sfx_engine_is_loop_clean_and_seed_textured) {
    auto a = sfx::engine(48000, 3);
    auto b = sfx::engine(48000, 3);
    auto c = sfx::engine(48000, 4);
    CHECK(!a.empty());
    CHECK(a == b);                 // deterministic
    CHECK(a != c);                 // seeds vary the rasp texture
    double energy = 0;
    for (float f : a) {
        CHECK(std::fabs(f) <= 1.0f);
        energy += std::fabs(f);
    }
    CHECK(energy / a.size() > 0.1);   // a steady running engine, not a blip
    // LOOP-CLEAN, the horn's contract (a held voice loops this forever): every
    // partial is an integer number of cycles over the buffer, so the wrap seam
    // is no steeper than the steepest step inside it — at any sample rate.
    auto seamIsClean = [](const std::vector<float>& f) {
        float maxStep = 0.0f;
        for (size_t i = 1; i < f.size(); i++)
            maxStep = std::max(maxStep, std::fabs(f[i] - f[i - 1]));
        return std::fabs(f.front() - f.back()) <= maxStep * 1.5f + 1e-4f;
    };
    CHECK(seamIsClean(a));
    CHECK(seamIsClean(sfx::engine(44100, 3)));
}

TEST_CASE(sfx_engine_pulses_and_varies_instead_of_humming) {
    // Device: "it just sounds like a constant hum." A stack of steady sines
    // IS a hum; an engine is a train of combustion pulses, no two alike. Both
    // properties are measurable on the buffer.
    auto pcm = sfx::engine(48000, 1);
    const size_t firings = 42;              // kEngineRefHz * 0.75 s
    const size_t span = pcm.size() / firings;
    CHECK(span > 100);

    std::vector<double> perFiring(firings, 0.0);
    double headSum = 0, tailSum = 0;
    for (size_t f = 0; f < firings; f++) {
        double energy = 0;
        for (size_t i = 0; i < span; i++) {
            const double s = pcm[f * span + i];
            energy += s * s;
            if (i < span / 8) headSum += std::fabs(s);
            if (i >= span - span / 8) tailSum += std::fabs(s);
        }
        perFiring[f] = std::sqrt(energy / static_cast<double>(span));
    }

    // PULSE SHAPE: each firing is loud at its start and has blown down by its
    // end — the puff. A drone would have head ~= tail.
    CHECK(headSum > tailSum * 1.25);

    // CYCLE-TO-CYCLE VARIATION: no two firings are the same size, so the loop
    // never settles into an audible pattern.
    double lo = 1e9, hi = 0, mean = 0;
    for (double e : perFiring) {
        lo = std::min(lo, e);
        hi = std::max(hi, e);
        mean += e;
    }
    mean /= static_cast<double>(firings);
    CHECK(mean > 0.01);                    // audible
    CHECK((hi - lo) / mean > 0.25);        // genuinely uneven, not a metronome
}

TEST_CASE(engine_note_runs_continuously_and_revs_through_the_mixer) {
    // The END-TO-END claim the unit maths cannot make: a held, looping engine
    // voice actually SOUNDS — continuously, with no silent gap at the loop
    // seam — and pitching it up (revs) speeds the waveform up. Manual mode
    // pumps the real mixer, so this is the audible path, not a stand-in.
    AudioEngine audio;
    CHECK(audio.initialize(AudioBackendMode::Manual));
    std::vector<float> pcm = sfx::engine(audio.sampleRate(), 1);
    AudioClipHandle clip =
        audio.createClip(pcm.data(), pcm.size(), 1, audio.sampleRate());
    CHECK(clip.valid());

    AudioPlayParams params;
    params.loop = true;
    params.volume = 0.9f;
    params.pitch = 1.0f;
    AudioVoiceHandle voice = audio.play(clip, params);
    CHECK(voice.valid());

    // Pump ~3 clip lengths (well past several loop wraps) in blocks, and
    // require every block to carry sound: a seam that dropped out or a voice
    // that stopped at the end of the sample would show as a silent block.
    const uint64_t block = pcm.size() / 4;
    std::vector<float> out(block * 2);   // stereo interleaved
    auto blockEnergy = [&]() {
        std::fill(out.begin(), out.end(), 0.0f);
        audio.readFrames(out.data(), block);
        double e = 0;
        for (float f : out) e += std::fabs(f);
        return e / out.size();
    };
    auto zeroCrossings = [&]() {
        int crossings = 0;
        for (size_t i = 2; i < out.size(); i += 2)   // left channel only
            if ((out[i - 2] < 0.0f) != (out[i] < 0.0f)) crossings++;
        return crossings;
    };
    for (int i = 0; i < 12; i++) CHECK(blockEnergy() > 0.01);
    CHECK(audio.isPlaying(voice));

    // Revving: the same voice at a higher playback rate crosses zero more
    // often in the same window — the note went up.
    blockEnergy();
    const int idleCrossings = zeroCrossings();
    audio.setVoicePitch(voice, 2.2f);
    blockEnergy();
    const int revvedCrossings = zeroCrossings();
    CHECK(revvedCrossings > idleCrossings);

    audio.stop(voice);
    CHECK(!audio.isPlaying(voice));
    audio.shutdown();
}

TEST_CASE(audio_system_registered_procedural_clip_plays_by_name) {
    AudioEngine audio;
    audio.initialize(AudioBackendMode::Manual);
    AudioSystem system;

    auto pcm = sfx::impact(48000, 5);
    AudioClipHandle clip = audio.createClip(pcm.data(), pcm.size(), 1, 48000);
    system.registerClip("sfx/impact", clip);
    CHECK(system.cachedClipCount() == 1);

    PlaySound cue;
    cue.clip = "sfx/impact";      // resolved from the registry, not a file
    cue.spatial = true;
    cue.position = Vec3(1, 0, -2);
    cue.pitch = 1.2f;
    system.onPlaySound(cue, audio);
    CHECK(audio.activeVoiceCount() == 1);
    CHECK(pump(audio, 2048).total() > 0.001);
}
