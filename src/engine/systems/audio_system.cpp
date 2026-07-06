#include "audio_system.h"

#include "../components.h"

#include <algorithm>

namespace engine {

void AudioSystem::onStart(FrameContext& ctx) {
    if (!ctx.audio.ready()) return;
    AudioEngine* audio = &ctx.audio;
    playSoundSub = ctx.events.subscribe<PlaySound>(
        [this, audio](const PlaySound& event) { onPlaySound(event, *audio); });
}

void AudioSystem::update(FrameContext& ctx) {
    if (!ctx.audio.ready()) return;
    syncListener(ctx.world, ctx.audio, ctx.view.camera);
    step(ctx.world, ctx.audio);
}

void AudioSystem::onStop(FrameContext& ctx) {
    ctx.events.unsubscribe(playSoundSub);
    playSoundSub = EventBus::INVALID_SUBSCRIPTION;
    if (ctx.audio.ready()) ctx.audio.stopAll();
    liveVoices.clear();
    clipCache.clear();
}

void AudioSystem::syncListener(World& world, AudioEngine& audio,
                               const CameraState& camera) {
    bool haveListener = false;
    world.each<AudioListener, Transform>(
        [&](Entity, AudioListener&, Transform& t) {
            if (haveListener) return;   // at most one; first wins
            haveListener = true;
            audio.setListener(t.position, t.orientation.rotate(Vec3(0, 0, -1)));
        });
    if (!haveListener) {
        Vec3 forward = camera.target - camera.position;
        if (forward.lengthSquared() > 0)
            forward = normalize(forward);
        else
            forward = Vec3(0, 0, -1);
        audio.setListener(camera.position, forward, camera.up);
    }
}

void AudioSystem::step(World& world, AudioEngine& audio) {
    // Start pending sources and follow moving spatial ones.
    world.each<AudioSource, Transform>([&](Entity e, AudioSource& source,
                                           Transform& t) {
        bool wantsStart = source.trigger ||
                          (source.autoplay && !source.voice.valid());
        if (wantsStart) {
            source.trigger = false;
            if (source.voice.valid()) audio.stop(source.voice);
            if (!source.clipHandle.valid())
                source.clipHandle = resolveClip(audio, source.clip);
            if (source.clipHandle.valid()) {
                AudioPlayParams params;
                params.volume = source.volume;
                params.pitch = source.pitch;
                params.loop = source.loop;
                params.bus = source.bus;
                source.voice =
                    source.spatial
                        ? audio.playAt(source.clipHandle, t.position,
                                       source.range, params)
                        : audio.play(source.clipHandle, params);
                if (source.voice.valid())
                    liveVoices.emplace_back(e, source.voice);
            } else if (source.autoplay) {
                source.autoplay = false;   // missing clip: don't retry forever
            }
        }
        if (source.voice.valid()) {
            if (!audio.isPlaying(source.voice)) {
                source.voice = AudioVoiceHandle{};   // one-shot finished
            } else if (source.spatial) {
                audio.setVoicePosition(source.voice, t.position);
            }
        }
    });

    // Stop voices whose owning entity died (looping ambience would leak).
    liveVoices.erase(
        std::remove_if(liveVoices.begin(), liveVoices.end(),
                       [&](const std::pair<Entity, AudioVoiceHandle>& entry) {
                           if (!audio.isPlaying(entry.second)) return true;
                           if (!world.alive(entry.first)) {
                               audio.stop(entry.second);
                               return true;
                           }
                           return false;
                       }),
        liveVoices.end());

    audio.update();   // reclaim finished one-shot voices
}

void AudioSystem::onPlaySound(const PlaySound& event, AudioEngine& audio) {
    AudioClipHandle clip = resolveClip(audio, event.clip);
    if (!clip.valid()) return;
    AudioPlayParams params;
    params.volume = event.volume;
    params.bus = event.bus;
    if (event.spatial)
        audio.playAt(clip, event.position, event.range, params);
    else
        audio.play(clip, params);
}

AudioClipHandle AudioSystem::resolveClip(AudioEngine& audio,
                                         const std::string& path) {
    if (path.empty()) return {};
    auto it = clipCache.find(path);
    if (it != clipCache.end()) return it->second;
    AudioClipHandle clip = audio.loadClip(path);
    if (clip.valid()) clipCache.emplace(path, clip);
    return clip;
}

}  // namespace engine
