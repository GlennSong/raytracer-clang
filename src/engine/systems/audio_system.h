#ifndef RAYTRACER_ENGINE_AUDIO_SYSTEM_H
#define RAYTRACER_ENGINE_AUDIO_SYSTEM_H

#include "../system.h"
#include "../audio/audio_engine.h"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine {

// Drives the AudioEngine from the ECS (ADR-0069): starts AudioSource voices
// (autoplay/trigger), feeds spatial voices their entity's Transform each
// frame, stops voices whose entity died, reclaims finished one-shots, and
// points the listener at the AudioListener entity (or the render camera when
// none exists). Also subscribes to PlaySound events (ADR-0066), so any system
// can fire a sound cue without touching audio directly.
//
// The System hooks are thin wrappers over the core methods, which take a
// World + AudioEngine directly so the integration is unit-testable without a
// FrameContext (the PhysicsSystem pattern).
class AudioSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    // --- headless-testable core -------------------------------------------
    // Start pending sources, sync spatial voice positions, stop orphans,
    // reclaim finished voices, clear finished handles on their sources.
    void step(World& world, AudioEngine& audio);
    // Listener from the AudioListener entity's Transform, else from `camera`.
    void syncListener(World& world, AudioEngine& audio,
                      const CameraState& camera);
    // The PlaySound handler (public so tests can invoke it directly).
    void onPlaySound(const PlaySound& event, AudioEngine& audio);

    std::size_t cachedClipCount() const { return clipCache.size(); }

private:
    AudioClipHandle resolveClip(AudioEngine& audio, const std::string& path);

    // Path -> decoded clip, shared by every source/event using the file.
    std::unordered_map<std::string, AudioClipHandle> clipCache;
    // Live voices by owning entity, so a destroyed entity's voice (e.g. a
    // looping ambience) is stopped instead of leaking.
    std::vector<std::pair<Entity, AudioVoiceHandle>> liveVoices;
    EventBus::SubscriptionId playSoundSub = EventBus::INVALID_SUBSCRIPTION;
};


}  // namespace engine

#endif
