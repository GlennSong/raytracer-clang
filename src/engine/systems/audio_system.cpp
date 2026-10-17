#include "audio_system.h"

#include "../components.h"
#include "../procgen/city/building_records.h"

#include <algorithm>

namespace engine {

void AudioSystem::onStart(FrameContext& ctx) {
    if (!ctx.audio.ready()) return;
    AudioEngine* audio = &ctx.audio;
    playSoundSub = ctx.events.subscribe<PlaySound>(
        [this, audio](const PlaySound& event) { onPlaySound(event, *audio); });
}

void AudioSystem::update(FrameContext& ctx) {
    if (ctx.audio.ready()) {
        syncListener(ctx.world, ctx.audio, ctx.view.camera);
        syncEnclosures(ctx.world, ctx.audio);
        step(ctx.world, ctx.audio);
    }
    publishViewerBox(ctx);
}

// The renderer's half of the enclosures: the box the viewer is in -- the cabin they ride in, else the building
// they stand in (its plan's bounding box, a little proud of the walls so the glass lies inside it) -- so a
// window of their own reflects the room, not the sky (SceneLighting::viewerBox*).
void AudioSystem::publishViewerBox(FrameContext& ctx) {
    SceneLighting& L = ctx.view.lighting;
    L.viewerBoxValid = false;
    Vec3 c, half;
    Quat q;
    if (ctx.audio.ready() && ctx.audio.listenerCabin(c, q, half)) {
        L.viewerBoxValid = true;
        L.viewerBoxCentre = c;
        L.viewerBoxAxisU = q.rotate(Vec3(1, 0, 0));
        L.viewerBoxHalf = half + Vec3(0.15, 0.15, 0.15);
        return;
    }
    const Vec3 eye = ctx.view.camera.position;
    const CityBuildings* cb = nullptr;
    ctx.world.each<CityBuildings>([&](Entity, CityBuildings& b) { if (!cb) cb = &b; });
    const BuildingRecord* r = cb ? cb->recordAt(Vec2(eye.x, eye.z), eye.y) : nullptr;
    if (!r || r->plan.size() < 3) return;
    const OBB2 ob = orientedBoundingBox(r->plan);
    L.viewerBoxValid = true;
    L.viewerBoxCentre = Vec3(ob.center.x, r->baseY + r->height * 0.5, ob.center.y);
    L.viewerBoxAxisU = Vec3(ob.axis[0].x, 0, ob.axis[0].y);
    L.viewerBoxHalf = Vec3(ob.half[0] + 0.6, r->height * 0.5 + 0.6, ob.half[1] + 0.6);
}

void AudioSystem::onStop(FrameContext& ctx) {
    ctx.events.unsubscribe(playSoundSub);
    playSoundSub = EventBus::INVALID_SUBSCRIPTION;
    if (ctx.audio.ready()) {
        ctx.audio.stopAll();
        ctx.audio.setEnclosureQuery({});   // it holds the World
    }
    queryWorld_ = nullptr;
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

void AudioSystem::syncEnclosures(World& world, AudioEngine& audio) {
    if (queryWorld_ != &world) {
        queryWorld_ = &world;
        World* w = &world;
        audio.setEnclosureQuery([w](const Vec3& p) {
            const CityBuildings* cb = nullptr;
            w->each<CityBuildings>([&](Entity, CityBuildings& c) { if (!cb) cb = &c; });
            if (!cb) return AudioEnclosure{};
            const BuildingRecord* r = cb->recordAt(Vec2(p.x, p.z), p.y);
            if (!r) return AudioEnclosure{};
            return AudioEnclosure{
                buildingEnclosureId(static_cast<std::size_t>(r - cb->records.data())),
                kBuildingWalls};
        });
    }
    // In a car: its chassis box is the cabin.
    world.each<InVehicle>([&](Entity, InVehicle& in) {
        const Transform* t = world.get<Transform>(in.vehicle);
        const Vehicle* v = world.get<Vehicle>(in.vehicle);
        if (!t || !v) return;
        audio.setListenerCabin(cabinEnclosureId(in.vehicle.index), t->position, t->orientation,
                               v->config.chassisHalfExtent, kCabin);
    });
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

void AudioSystem::registerClip(const std::string& name, AudioClipHandle clip) {
    if (!name.empty() && clip.valid()) clipCache[name] = clip;
}

void AudioSystem::onPlaySound(const PlaySound& event, AudioEngine& audio) {
    AudioClipHandle clip = resolveClip(audio, event.clip);
    if (!clip.valid()) return;
    AudioPlayParams params;
    params.volume = event.volume;
    params.pitch = event.pitch;
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
