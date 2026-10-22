#ifndef RAYTRACER_ENGINE_AUDIO_ENGINE_H
#define RAYTRACER_ENGINE_AUDIO_ENGINE_H

#include "../../handle.h"
#include "../../rt_math.h"

#include <algorithm>
#include <cstdint>
#include <functional>
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
// `clip` is a file path OR a name registered via AudioSystem::registerClip
// (how procedural clips are addressed).
struct PlaySound {
    std::string clip;
    Vec3 position;             // used when spatial
    bool spatial = false;
    float volume = 1.0f;
    float pitch = 1.0f;        // playback-rate multiplier (variation)
    Real range = 25.0;         // audible radius (world units) when spatial
    AudioBus bus = AudioBus::Sfx;
};

// HOW LOUD A SPATIAL SOUND IS AT DISTANCE d (Glenn: "the train horn sounds the same inside a building and
// pretty much anywhere you are in the world"). It used to be a straight line from full at 1 m to silence at
// `range`, so a 600 m horn was still at half volume 300 m away. Now it is the free-field law: full out to the
// reference distance, then -6 dB per doubling (ref / d), faded to silence over the last third of `range` so
// a voice still ends at its range and can be culled there. `ref` <= 0 takes audioRefDistance(range): a
// source that carries far is a loud one, heard at full volume from further off.
inline Real audioRefDistance(Real range) { return std::max(Real(2), range * Real(0.05)); }
inline float audioDistanceGain(Real d, Real ref, Real range) {
    if (ref <= 0) ref = audioRefDistance(range);
    if (d >= range) return 0.0f;
    Real g = d <= ref ? Real(1) : ref / d;
    const Real fadeFrom = range * (Real(2) / Real(3));
    if (d > fadeFrom) {
        const Real t = (range - d) / (range - fadeFrom);   // 1 at the fade's start, 0 at range
        g *= t * t * (Real(3) - Real(2) * t);
    }
    return static_cast<float>(g);
}
// AIR ABSORPTION: air soaks up the high end with distance (a horn a few hundred metres off is a dull note,
// not a bright one). The cutoff of the low-pass a voice is heard through at distance d.
inline float audioAirCutoffHz(Real d) {
    return static_cast<float>(Real(20000) / (Real(1) + std::max(Real(0), d) / Real(120)));
}

// WHAT A SOUND CROSSES to reach the ear: a building's walls, a car's cabin. A sound from one enclosure heard
// in another passes the barriers of both (out of its own, into the listener's): the gains multiply, the
// lower cutoff wins. The open air is enclosure 0 with no barrier.
struct AudioBarrier {
    float gain = 1.0f;
    float cutoffHz = 20000.0f;
};
struct AudioEnclosure {
    uint64_t id = 0;   // 0 = the open air
    AudioBarrier barrier;
};
// Which enclosure a world point is in. Asked for the listener and for each spatial voice (again only when
// it has moved a metre), so it should be a cheap lookup (a building index, not a ray).
using AudioEnclosureQuery = std::function<AudioEnclosure(const Vec3&)>;

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
    // 3D playback at a world position: audioDistanceGain to silence at
    // `range`, air absorption, and the barriers between its enclosure and
    // the listener's.
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
    // Where the walls are (see AudioEnclosureQuery). Unset = everything is in the open air.
    void setEnclosureQuery(AudioEnclosureQuery query);
    // The listener rides in a CABIN (a car, a bus, a train car): the listener's enclosure is `id` wherever
    // the ear is (a chase camera hangs outside the car, but the player is in it), and any voice inside the
    // box -- the car's own engine -- is in it too. Renewed every frame by whoever knows the vehicle; it
    // lapses two update()s after the last call.
    void setListenerCabin(uint64_t id, const Vec3& centre, const Quat& orientation,
                          const Vec3& halfExtent, AudioBarrier barrier);
    // The listener's enclosure as of the last update() (0 = open air): for tests and the debug readout.
    uint64_t listenerEnclosure() const;
    // The cabin the listener rides in, while it lasts (the renderer's "your own glass" box: AudioSystem).
    bool listenerCabin(Vec3& centre, Quat& orientation, Vec3& halfExtent) const;
    void setMasterVolume(float volume);
    float masterVolume() const;
    void setBusVolume(AudioBus bus, float volume);
    float busVolume(AudioBus bus) const;

    // Reclaim finished one-shot voices, and re-hear every spatial voice
    // (distance, air, barriers -- eased over ~0.1 s so stepping through a
    // door doesn't click). Called once per frame (AudioSystem).
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
