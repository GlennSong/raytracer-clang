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

// ---- THE GROUND'S SOUNDS (#62/#63/#64/#65) ------------------------------------------------------
namespace {
// What an ear tells apart: how the energy splits across four bands (low body, low-mid, presence,
// air), and how long the sound lasts (time to 90% of its energy).
struct Voiceprint { double band[4]; double length; double centroidish; };
Voiceprint voiceprint(const std::vector<float>& f, double rate) {
    struct LP { double a, y = 0; LP(double hz, double r) : a(1 - std::exp(-6.283185307 * hz / r)) {} double operator()(double x) { y += a * (x - y); return y; } };
    LP l300(300, rate), l1500(1500, rate), l5000(5000, rate);
    double e[4] = {0, 0, 0, 0}, total = 0;
    std::vector<double> cum(f.size());
    for (std::size_t i = 0; i < f.size(); ++i) {
        const double x = f[i], a = l300(x), b = l1500(x), c = l5000(x);
        const double bands[4] = {a, b - a, c - b, x - c};
        for (int k = 0; k < 4; ++k) e[k] += bands[k] * bands[k];
        total += x * x;
        cum[i] = total;
    }
    Voiceprint v{};
    const double es = e[0] + e[1] + e[2] + e[3];
    for (int k = 0; k < 4; ++k) v.band[k] = es > 0 ? e[k] / es : 0;
    std::size_t n90 = 0;
    while (n90 < cum.size() && cum[n90] < 0.9 * total) ++n90;
    v.length = static_cast<double>(n90) / rate;
    v.centroidish = v.band[1] * 1 + v.band[2] * 2 + v.band[3] * 3;
    return v;
}
double distance(const Voiceprint& a, const Voiceprint& b) {
    double d = 0;
    for (int k = 0; k < 4; ++k) d += std::fabs(a.band[k] - b.band[k]);
    return d + std::fabs(a.length - b.length) / 0.1;   // 100 ms of length ~ a whole band's worth
}
void maybeDump(const std::string& name, const std::vector<float>& f, uint32_t rate) {
    const char* dir = std::getenv("RT_SFX_DUMP");   // listen to them: RT_SFX_DUMP=/some/dir run_tests
    if (!dir || !*dir) return;
    std::ofstream o(std::string(dir) + "/" + name + ".wav", std::ios::binary);
    auto u32 = [&](uint32_t v) { o.write(reinterpret_cast<const char*>(&v), 4); };
    auto u16 = [&](uint16_t v) { o.write(reinterpret_cast<const char*>(&v), 2); };
    const uint32_t bytes = static_cast<uint32_t>(f.size() * 2);
    o.write("RIFF", 4); u32(36 + bytes); o.write("WAVEfmt ", 8); u32(16); u16(1); u16(1); u32(rate); u32(rate * 2); u16(2); u16(16);
    o.write("data", 4); u32(bytes);
    for (float x : f) { const int16_t s = static_cast<int16_t>(std::lround(std::clamp(x, -1.0f, 1.0f) * 32767.0f)); o.write(reinterpret_cast<const char*>(&s), 2); }
}
}  // namespace

TEST_CASE(footsteps_sound_different_on_every_ground) {
    // The issue's gate: "walk grass -> asphalt -> sand -> snow: four distinct footstep sounds, no samples".
    const uint32_t rate = 48000;
    const sfx::Ground grounds[] = {sfx::Ground::Asphalt, sfx::Ground::Concrete, sfx::Ground::Grass, sfx::Ground::Dirt,
                                   sfx::Ground::Sand, sfx::Ground::Rock, sfx::Ground::Snow, sfx::Ground::Water};
    std::vector<Voiceprint> prints;
    for (sfx::Ground g : grounds) {
        const auto a = sfx::footstep(g, rate, 3), b = sfx::footstep(g, rate, 3), c = sfx::footstep(g, rate, 4);
        CHECK(!a.empty() && a == b && a != c);   // deterministic; seeds vary the step
        double energy = 0;
        for (float x : a) { CHECK(std::fabs(x) <= 1.0f); energy += std::fabs(x); }
        CHECK(energy / a.size() > 0.01);
        prints.push_back(voiceprint(a, rate));
        for (int s = 1; s <= 3; ++s) maybeDump(std::string("step_") + sfx::groundName(g) + "_" + std::to_string(s), sfx::footstep(g, rate, s), rate);
        maybeDump(std::string("jump_") + sfx::groundName(g), sfx::jumpPush(g, rate, 1), rate);
        maybeDump(std::string("land_hop_") + sfx::groundName(g), sfx::landing(g, 0.1, rate, 1), rate);
        maybeDump(std::string("land_drop_") + sfx::groundName(g), sfx::landing(g, 1.0, rate, 1), rate);
    }
    double closest = 1e9; int ci = -1, cj = -1;
    for (std::size_t i = 0; i < prints.size(); ++i)
        for (std::size_t j = i + 1; j < prints.size(); ++j) {
            const double d = distance(prints[i], prints[j]);
            if (d < closest) { closest = d; ci = static_cast<int>(i); cj = static_cast<int>(j); }
        }
    std::printf("    [steps] closest pair %s/%s at %.2f\n", sfx::groundName(grounds[ci]), sfx::groundName(grounds[cj]), closest);
    for (std::size_t i = 0; i < prints.size(); ++i)
        std::printf("      %-9s bands %.2f %.2f %.2f %.2f  length %.0f ms\n", sfx::groundName(grounds[i]), prints[i].band[0], prints[i].band[1],
                    prints[i].band[2], prints[i].band[3], prints[i].length * 1000);
    CHECK(closest > 0.15);   // no two grounds sound alike
}

TEST_CASE(a_pavement_step_is_a_low_thud) {
    // #98 round 2 ("still sounds like I'm walking on spindly legs"): measured against a CC0 recording of shoes on
    // concrete (Kenney Impact Sounds, footstep_concrete_000-004): centroid 160-360 Hz, energy 160-640 Hz, ~2% over
    // 1.25 kHz. Ours had its centroid at 2.7-4.8 kHz. Gate: sidewalk and asphalt steps centred under 600 Hz with
    // under 8% of the energy over 1.25 kHz.
    const uint32_t rate = 48000;
    for (sfx::Ground g : {sfx::Ground::Asphalt, sfx::Ground::Concrete}) {
        for (uint32_t seed : {1u, 2u, 3u, 4u}) {
            const auto f = sfx::footstep(g, rate, seed);
            // centroid by zero-crossing-free means: energy-weighted band split with one-pole low-passes
            double lo = 0, hi = 0, total = 0, yl = 0, yh = 0;
            const double al = 1 - std::exp(-6.283185307 * 600.0 / rate), ah = 1 - std::exp(-6.283185307 * 1250.0 / rate);
            for (float x : f) {
                yl += al * (x - yl); yh += ah * (x - yh);
                lo += yl * yl; hi += (x - yh) * (x - yh); total += double(x) * x;
            }
            std::printf("    [pavement] %-8s seed %u  under 600 Hz %.0f%%  over 1.25 kHz %.1f%%\n", sfx::groundName(g), seed,
                        100 * lo / total, 100 * hi / total);
            CHECK(lo / total > 0.6);
            CHECK(hi / total < 0.08);
        }
    }
}

TEST_CASE(a_drop_lands_heavier_and_lower_than_a_hop) {
    // #63's gate: "a 1 m hop and a 5 m drop sound different; landing on grass vs rock differs".
    const uint32_t rate = 48000;
    for (sfx::Ground g : {sfx::Ground::Grass, sfx::Ground::Rock, sfx::Ground::Asphalt}) {
        const Voiceprint hop = voiceprint(sfx::landing(g, 0.1, rate, 1), rate);
        const Voiceprint drop = voiceprint(sfx::landing(g, 1.0, rate, 1), rate);
        CHECK(drop.length > hop.length * 1.3);         // it rings on
        CHECK(drop.band[0] > hop.band[0]);             // more of it is the low body thud
    }
    CHECK(distance(voiceprint(sfx::landing(sfx::Ground::Grass, 0.6, rate, 1), rate),
                   voiceprint(sfx::landing(sfx::Ground::Rock, 0.6, rate, 1), rate)) > 0.15);
}

TEST_CASE(ambience_loops_are_seamless_and_audible) {
    const uint32_t rate = 48000;
    auto seamIsClean = [](const std::vector<float>& f) {
        float maxStep = 0.0f;
        for (size_t i = 1; i < f.size(); i++) maxStep = std::max(maxStep, std::fabs(f[i] - f[i - 1]));
        return std::fabs(f.front() - f.back()) <= maxStep * 1.5f + 1e-4f;
    };
    struct L { const char* name; std::vector<float> f; };
    for (const L& l : {L{"grass_rustle", sfx::grassRustle(rate, 1)}, L{"surf", sfx::surf(rate, 1)}, L{"river", sfx::river(rate, 1)}}) {
        CHECK(!l.f.empty());
        double energy = 0;
        for (float x : l.f) { CHECK(std::fabs(x) <= 1.0f); energy += std::fabs(x); }
        CHECK(energy / l.f.size() > 0.02);   // a bed, not near-silence
        CHECK(seamIsClean(l.f));
        maybeDump(std::string("loop_") + l.name, l.f, rate);
    }
    // surf and river are different beds: the surf breathes (two swells), the river does not
    // steadiness: the loud (90th percentile) 0.1 s window over the quiet (10th) one -- single extreme windows
    // are one bubble or one gap, not the character of the sound
    auto swing = [](const std::vector<float>& f) {
        const std::size_t win = 4800;
        std::vector<double> e;
        for (std::size_t i = 0; i + win <= f.size(); i += win) {
            double s2 = 0;
            for (std::size_t k = i; k < i + win; ++k) s2 += f[k] * f[k];
            e.push_back(s2);
        }
        std::sort(e.begin(), e.end());
        return e[e.size() * 9 / 10] / std::max(e[e.size() / 10], 1e-9);
    };
    const double surfSwing = swing(sfx::surf(rate, 1)), riverSwing = swing(sfx::river(rate, 1));
    std::printf("    [ambience] loudness swing (90th/10th percentile 0.1 s): surf %.1fx, river %.1fx\n", surfSwing, riverSwing);
    CHECK(surfSwing > 3.0 && surfSwing > 2.0 * riverSwing);
}

#include "../src/engine/audio/footsteps.h"

TEST_CASE(footsteps_keep_a_walking_cadence_and_land_by_the_fall) {
    const double dt = 1.0 / 60.0;
    auto count = [&](double speed, double seconds, bool crouch = false) {
        FootstepTracker t; int steps = 0;
        for (int i = 0; i < static_cast<int>(seconds / dt); ++i)
            if (t.update(true, speed, 0, dt, crouch).kind == FootstepEvent::Kind::Step) ++steps;
        return steps;
    };
    // a walk (1.4 m/s): ~1.9 steps a second; a run (6 m/s): ~3.2 (1.9 m strides -- 5.5 a second was
    // "sped up", #98); standing still: none
    const int walk = count(1.4, 10), run = count(6.0, 10), still = count(0.0, 10);
    std::printf("    [steps] 10 s: walk %d, run %d, still %d\n", walk, run, still);
    CHECK(walk >= 17 && walk <= 21);
    CHECK(run >= 29 && run <= 34);
    CHECK(still == 0);
    // the first step after starting off comes within half a stride, not a whole one
    { FootstepTracker t; int first = -1;
      for (int i = 0; i < 120 && first < 0; ++i) if (t.update(true, 1.4, 0, dt).kind == FootstepEvent::Kind::Step) first = i;
      CHECK(first > 0 && first * dt < 0.35); }

    // jump, hang, land: a push-off event, then a landing whose weight follows the fall
    auto fall = [&](double upSpeed, double airSeconds) {
        FootstepTracker t;
        for (int i = 0; i < 30; ++i) t.update(true, 0, 0, dt);
        FootstepEvent jump = t.update(false, 0, upSpeed, dt);
        double vy = upSpeed;
        FootstepEvent land;
        for (int i = 0; i < static_cast<int>(airSeconds / dt); ++i) { vy -= 9.81 * dt; t.update(false, 0, vy, dt); }
        land = t.update(true, 0, 0, dt);
        return std::make_pair(jump, land);
    };
    const auto hop = fall(4.4, 0.9);            // a jump on the flat: up and back down, ~4.4 m/s at touchdown
    const auto drop = fall(0.0, 1.1);           // stepping off a ~6 m wall: ~10.8 m/s
    CHECK(hop.first.kind == FootstepEvent::Kind::Jump);
    CHECK(drop.first.kind == FootstepEvent::Kind::None);   // walking off an edge is not a jump
    CHECK(hop.second.kind == FootstepEvent::Kind::Land && drop.second.kind == FootstepEvent::Kind::Land);
    std::printf("    [land] hop heavy %.2f vol %.2f, drop heavy %.2f vol %.2f\n", hop.second.heavy, hop.second.volume, drop.second.heavy, drop.second.volume);
    CHECK(hop.second.heavy < 0.35 && drop.second.heavy > 0.85);
    CHECK(drop.second.volume > hop.second.volume);
    // a kerb-sized bump in the ground (a tick or two of "air") is not a landing
    { FootstepTracker t; t.update(true, 1.4, 0, dt); t.update(false, 1.4, -0.5, dt);
      CHECK(t.update(true, 1.4, 0, dt).kind != FootstepEvent::Kind::Land); }
}

TEST_CASE(the_ground_under_the_foot_picks_the_sound) {
    using G = sfx::Ground;
    CHECK(groundForSurface(2, true, 1, 0, 0, 0, 0) == G::Asphalt);        // a road is a road over any cover
    CHECK(groundForSurface(3, true, 1, 0, 0, 0, 0) == G::Concrete);
    CHECK(groundForSurface(0, true, 1, 0, 0, 0, 0) == G::Concrete);       // an untagged floor
    CHECK(groundForSurface(1, true, 0.2, 0.1, 0.6, 0.1, 0) == G::Sand);   // terrain: the cover decides
    CHECK(groundForSurface(1, true, 0.6, 0.2, 0, 0.2, 0.55) == G::Snow);  // snow over grass is snow
    CHECK(groundForSurface(1, true, 0.1, 0.2, 0, 0.7, 0) == G::Rock);
    CHECK(groundForSurface(1, false, 0, 0, 0, 0, 0) == G::Grass);         // no cover map: grass
}

#include "../src/engine/components.h"

TEST_CASE(open_sea_keeps_the_ocean_and_drops_the_ponds) {
    // #65: waves on a beach, not on a pond the water plane fills in a basin. A 20x20 grid of 50 m cells
    // with a 3-cell-deep strip along the south edge (the ocean) and a 2x2 pond in the middle.
    std::vector<std::array<double, 4>> cells;
    for (int i = 0; i < 20; ++i)
        for (int j = 0; j < 3; ++j) cells.push_back({i * 50.0, j * 50.0, i * 50.0 + 50, j * 50.0 + 50});
    for (int i = 9; i < 11; ++i)
        for (int j = 12; j < 14; ++j) cells.push_back({i * 50.0, j * 50.0, i * 50.0 + 50, j * 50.0 + 50});
    cells.push_back({0, 950, 50, 1000});   // a corner cell on the far edge of the map: sea running off it
    const auto open = openSeaCells(cells, 0, 0, 1000, 1000);
    CHECK(open.size() == 61);   // the strip (60) and the corner, not the pond (4)
    // a map with only ponds has no sea, however the ponds sit relative to each other
    std::vector<std::array<double, 4>> ponds = {{300, 300, 350, 350}, {600, 700, 650, 750}};
    CHECK(openSeaCells(ponds, 0, 0, 1000, 1000).empty());
    Sea sea;
    for (const auto& b : open) sea.add(b[0], b[1], b[2], b[3]);
    CHECK(sea.contains(500, 60) && !sea.contains(500, 625) && !sea.contains(500, 400));
    // a big lake inland still has waves
    std::vector<std::array<double, 4>> lake;
    for (int i = 0; i < 20; ++i) lake.push_back({i * 50.0, 0, i * 50.0 + 50, 50});   // the "edge" row
    for (int i = 0; i < 16; ++i) for (int j = 0; j < 16; ++j) lake.push_back({2000 + i * 50.0, 2000 + j * 50.0, 2050 + i * 50.0, 2050 + j * 50.0});
    CHECK(openSeaCells(lake, 0, 0, 4000, 4000).size() == lake.size());   // 0.64 km^2 > the 0.5 km^2 bar
}

TEST_CASE(hard_ground_steps_are_a_heel_and_a_toe) {
    // #84: "they all sound like I'm walking on grass". Hard ground is two impacts, heel then toe ~0.09 s later,
    // with the energy in the hits; soft ground is one rolled contact carried by its texture.
    const uint32_t rate = 48000;
    auto analyse = [&](const std::vector<float>& f, std::vector<double>& onsets) {
        // 1 ms envelope; an onset is a rise past 30% of the peak after a >25 ms dip below half of it
        const std::size_t w = rate / 1000;
        std::vector<double> env(f.size() / w);
        for (std::size_t k = 0; k < env.size(); ++k) { double m = 0; for (std::size_t i = k * w; i < (k + 1) * w; ++i) m = std::max(m, double(std::fabs(f[i]))); env[k] = m; }
        const double peak = *std::max_element(env.begin(), env.end());
        bool armed = true; int quiet = 25;
        for (std::size_t k = 0; k < env.size(); ++k) {
            if (env[k] < 0.15 * peak) ++quiet; else if (env[k] < 0.5 * peak) {} else quiet = 0;
            if (quiet >= 25) armed = true;
            if (armed && env[k] > 0.3 * peak) { onsets.push_back(k / 1000.0); armed = false; quiet = 0; }
        }
        // share of the energy within 20 ms after an onset
        double inHits = 0, total = 0;
        for (std::size_t i = 0; i < f.size(); ++i) {
            const double t = double(i) / rate, e = double(f[i]) * f[i];
            total += e;
            for (double o : onsets) if (t >= o && t < o + 0.02) { inHits += e; break; }
        }
        return total > 0 ? inHits / total : 0.0;
    };
    for (sfx::Ground g : {sfx::Ground::Asphalt, sfx::Ground::Concrete, sfx::Ground::Rock, sfx::Ground::Wood}) {
        for (uint32_t seed : {1u, 2u, 3u}) {
            std::vector<double> on;
            const double hits = analyse(sfx::footstep(g, rate, seed), on);
            std::printf("    [hard] %-8s seed %u onsets %zu (gap %.0f ms), %.0f%% of the energy in the hits\n", sfx::groundName(g), seed, on.size(),
                                       on.size() > 1 ? (on[1] - on[0]) * 1000 : 0.0, hits * 100);
            CHECK(on.size() == 2);
            if (on.size() == 2) CHECK(on[1] - on[0] > 0.06 && on[1] - on[0] < 0.15);
            if (g != sfx::Ground::Wood) CHECK(hits > 0.6);   // wood's ring is the point of it
        }
    }
    for (sfx::Ground g : {sfx::Ground::Grass, sfx::Ground::Sand, sfx::Ground::Snow}) {
        std::vector<double> on;
        const double hits = analyse(sfx::footstep(g, rate, 1), on);
        std::printf("    [soft] %-8s onsets %zu, %.0f%% of the energy in the hits\n", sfx::groundName(g), on.size(), hits * 100);
        CHECK(on.size() == 1);
    }
}

#include "../src/engine/systems/underwater_system.h"
#include "../src/engine/world.h"

TEST_CASE(the_camera_knows_whose_water_it_is_under) {
    // #58: the sea's surface over the open sea, nothing over land or a pond the water plane doesn't call sea
    World world;
    Sea sea; sea.level = 0.5; sea.add(0, 0, 100, 100);
    world.add<Sea>(world.create(), std::move(sea));
    const auto in = UnderwaterSystem::surfaceAt(world, 50, 50);
    CHECK(in.kind == UnderwaterSystem::Water::Sea && std::fabs(in.level - 0.5) < 1e-9);
    CHECK(UnderwaterSystem::surfaceAt(world, 150, 50).kind == UnderwaterSystem::Water::None);
}

TEST_CASE(a_stream_and_a_broad_river_sound_different) {
    // #87, tuned by ear with Glenn: a river is a low, soft, steady noise ("maybe it should just be a low
    // static noise"), no hiss; the wide river the dark one ("the first one sounds like a wide river"), a
    // narrow one lighter and quieter ("the narrow rivers should be more calm").
    const uint32_t rate = 48000;
    const auto stream = sfx::river(rate, 9, 0.1), broad = sfx::river(rate, 11, 0.9);
    const Voiceprint vs = voiceprint(stream, rate), vb = voiceprint(broad, rate);
    std::printf("    [river] stream bands %.2f %.2f %.2f %.2f | broad %.2f %.2f %.2f %.2f\n", vs.band[0], vs.band[1], vs.band[2], vs.band[3],
                vb.band[0], vb.band[1], vb.band[2], vb.band[3]);
    // Glenn picked the dark one for the WIDE river; a narrow one is lighter: less rumble, a touch more air
    CHECK(vb.band[0] > 0.6);                             // wide: low, most of it under 300 Hz
    CHECK(vs.band[2] + vs.band[3] < 0.10 && vb.band[2] + vb.band[3] < 0.08);   // no hiss in either
    CHECK(vb.band[0] > vs.band[0] + 0.1);                // the wide river deeper
    CHECK(vs.band[1] > vb.band[1]);                      // the narrow one lighter
    auto seamIsClean = [](const std::vector<float>& f) {
        float maxStep = 0.0f;
        for (size_t i = 1; i < f.size(); i++) maxStep = std::max(maxStep, std::fabs(f[i] - f[i - 1]));
        return std::fabs(f.front() - f.back()) <= maxStep * 1.5f + 1e-4f;
    };
    CHECK(seamIsClean(stream) && seamIsClean(broad));
    maybeDump("loop_river_stream", stream, rate);
    maybeDump("loop_river_broad", broad, rate);
}

TEST_CASE(the_river_heard_from_under_the_water_is_muffled) {
    // Glenn: the muffled bubbling "works good for underwater ... above water it wouldn't sound so muffled"
    const uint32_t rate = 48000;
    const auto air = sfx::river(rate, 9, 0.5), under = sfx::underwaterRiver(rate, 13);
    const Voiceprint va = voiceprint(air, rate), vu = voiceprint(under, rate);
    std::printf("    [river] in the air bands %.2f %.2f %.2f %.2f | under water %.2f %.2f %.2f %.2f\n", va.band[0], va.band[1],
                va.band[2], va.band[3], vu.band[0], vu.band[1], vu.band[2], vu.band[3]);
    CHECK(va.band[1] + va.band[2] > vu.band[1] + vu.band[2]);   // the air carries the brighter surface sounds
    CHECK(vu.band[0] > va.band[0] && vu.band[2] + vu.band[3] < 0.02);   // under water: darker, nothing bright
    maybeDump("loop_river_air", air, rate);
    maybeDump("loop_river_underwater", under, rate);
}


TEST_CASE(splashes_and_a_lake_shore) {
    // #43: a stroke is a soft slap and a little spray; jumping in is a big one with a gulp of air under it.
    const uint32_t rate = 48000;
    const auto stroke = sfx::splash(0.12, rate, 1), plunge = sfx::splash(1.0, rate, 1);
    const Voiceprint vs = voiceprint(stroke, rate), vp = voiceprint(plunge, rate);
    std::printf("    [splash] stroke %.0f ms, plunge %.0f ms; plunge low band %.2f vs %.2f\n", vs.length * 1000, vp.length * 1000, vp.band[0], vs.band[0]);
    CHECK(vp.length > vs.length * 1.5);
    CHECK(vp.band[0] > vs.band[0]);
    for (float x : plunge) CHECK(std::fabs(x) <= 1.0f);
    maybeDump("splash_stroke", stroke, rate);
    maybeDump("splash_plunge", plunge, rate);
    // a lake's edge laps: seamless, gentle wavelets (calmer than the surf, not a flat bed)
    const auto lap = sfx::lap(rate, 1);
    auto seamIsClean = [](const std::vector<float>& f) {
        float maxStep = 0.0f;
        for (size_t i = 1; i < f.size(); i++) maxStep = std::max(maxStep, std::fabs(f[i] - f[i - 1]));
        return std::fabs(f.front() - f.back()) <= maxStep * 1.5f + 1e-4f;
    };
    CHECK(seamIsClean(lap));
    std::vector<double> e;
    for (std::size_t i = 0; i + 4800 <= lap.size(); i += 4800) { double s2 = 0; for (std::size_t k = i; k < i + 4800; ++k) s2 += lap[k] * lap[k]; e.push_back(s2); }
    std::sort(e.begin(), e.end());
    const double swing = e[e.size() * 9 / 10] / std::max(e[e.size() / 10], 1e-9);
    std::printf("    [lap] swing %.1fx\n", swing);
    CHECK(swing > 3.0);   // wavelets come and go
    maybeDump("loop_lake_lap", lap, rate);
}
