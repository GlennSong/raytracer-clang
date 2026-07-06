#ifndef RAYTRACER_ENGINE_AUDIO_ENGINE_H
#define RAYTRACER_ENGINE_AUDIO_ENGINE_H

#include "../../handle.h"
#include "../../rt_math.h"

#include <cstdint>
#include <memory>
#include <string>

namespace engine {

// Thin wrapper that seals miniaudio behind a miniaudio-free interface
// (ADR-0069), the way PhysicsWorld seals Jolt: no ma_* type appears in this
// header, so engine/game code and tests depend only on our own types. The
// AudioSystem (ECS integration) and tests drive sounds through this.
//
// Builds gated by RT_ENABLE_AUDIO (CMake option, ON): without it every method
// is an inert stub (initialize() returns false), so headless/offline builds
// and the Makefile test target need no miniaudio.

struct AudioClipTag {};
struct AudioVoiceTag {};
// A loaded/generated sound asset (decoded PCM, shared by voices).
using AudioClipHandle = Handle<AudioClipTag>;
// One playing instance of a clip. One-shot voices are reclaimed automatically
// on update() once they finish; a handle to a finished voice reports stale
// (isPlaying == false), never aliases a newer voice.
using AudioVoiceHandle = Handle<AudioVoiceTag>;

// Mix buses: per-bus volume over the master (menus want independent sliders).
enum class AudioBus { Sfx = 0, Music, Ambient };
constexpr int AUDIO_BUS_COUNT = 3;

enum class AudioBackendMode {
    Auto,      // real output device, falling back to the null device (silent
               // but time-advancing) where no device exists — CI stays green
    Null,      // null device explicitly (headless soak runs)
    Manual,    // no device; the caller pumps readFrames() — deterministic,
               // inspectable output for tests and offline renders
};

struct AudioPlayParams {
    float volume = 1.0f;
    float pitch = 1.0f;    // playback rate multiplier (> 0)
    float pan = 0.0f;      // -1 left .. +1 right; non-spatial voices only
    bool loop = false;
    AudioBus bus = AudioBus::Sfx;
};

// Fire-and-forget sound cue, published on the EventBus (ADR-0066): gameplay
// systems emit these without knowing an audio engine exists; AudioSystem
// subscribes and plays them. `clip` is a path, resolved through AudioSystem's
// clip cache.
struct PlaySound {
    std::string clip;
    Vec3 position;             // used when spatial
    bool spatial = false;
    float volume = 1.0f;
    Real range = 25.0;         // audible radius (world units) when spatial
    AudioBus bus = AudioBus::Sfx;
};

class AudioEngine {
public:
    AudioEngine();
    ~AudioEngine();

    AudioEngine(const AudioEngine&) = delete;
    AudioEngine& operator=(const AudioEngine&) = delete;

    bool initialize(AudioBackendMode mode = AudioBackendMode::Auto);
    void shutdown();
    bool ready() const;
    AudioBackendMode backendMode() const;

    // --- clips ---------------------------------------------------------------
    // Decode an audio file (wav/flac/mp3) fully into memory. Null handle on
    // failure.
    AudioClipHandle loadClip(const std::string& path);
    // Create a clip from raw PCM — the procgen path (synthesized effects) and
    // the test path. `frames` is interleaved float32, frameCount*channels
    // samples, copied in.
    AudioClipHandle createClip(const float* frames, uint64_t frameCount,
                               uint32_t channels, uint32_t sampleRate);
    // Stops any voices still playing the clip, then frees it.
    void destroyClip(AudioClipHandle clip);

    // --- voices --------------------------------------------------------------
    // 2D playback (UI, music): no spatialization, pan honored.
    AudioVoiceHandle play(AudioClipHandle clip,
                          const AudioPlayParams& params = {});
    // 3D playback at a world position: linear falloff to silence at `range`.
    AudioVoiceHandle playAt(AudioClipHandle clip, const Vec3& position,
                            Real range = 25.0,
                            const AudioPlayParams& params = {});
    void stop(AudioVoiceHandle voice);
    void stopAll();
    bool isPlaying(AudioVoiceHandle voice) const;
    void setVoicePosition(AudioVoiceHandle voice, const Vec3& position);
    void setVoiceVolume(AudioVoiceHandle voice, float volume);
    void setVoicePitch(AudioVoiceHandle voice, float pitch);

    // --- listener / mix ------------------------------------------------------
    void setListener(const Vec3& position, const Vec3& forward,
                     const Vec3& up = Vec3(0, 1, 0),
                     const Vec3& velocity = Vec3());
    void setMasterVolume(float volume);
    float masterVolume() const;
    void setBusVolume(AudioBus bus, float volume);
    float busVolume(AudioBus bus) const;

    // Reclaim finished one-shot voices. Called once per frame (AudioSystem).
    void update();

    // Manual mode only: mix the next `frameCount` stereo frames into `out`
    // (interleaved float32, frameCount * 2 samples). Returns frames written
    // (0 in other modes). This is what makes audio deterministic under test.
    uint64_t readFrames(float* out, uint64_t frameCount);
    uint32_t sampleRate() const;   // engine output rate (0 until initialized)

    std::size_t clipCount() const;
    std::size_t activeVoiceCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};


}  // namespace engine

#endif
