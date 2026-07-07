// Physics -> sound demo (ADR-0071), headless: the REAL chain, end to end.
// A Jolt-backed PhysicsSystem drops a row of spheres and boxes onto a floor;
// every NEW contact Jolt reports becomes a Collision on the EventBus; a
// subscriber (the same logic as the arena's ArenaSoundSystem) turns hits into
// spatial "sfx/impact" cues scaled by closing speed; AudioSystem plays them
// through the Manual-mode AudioEngine, and the mix is written to
// impact_demo.wav — listen to the physics. Procedural gunshots punctuate it.
//
// Build: the `impact_demo` CMake target (physics + audio, both default-ON).
#include "engine/audio/audio_engine.h"
#include "engine/audio/sfx.h"
#include "engine/components.h"
#include "engine/event_bus.h"
#include "engine/systems/audio_system.h"
#include "engine/systems/physics_system.h"
#include "engine/world.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <vector>

using namespace engine;

namespace {

constexpr uint32_t RATE = 48000;

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

Entity addFloor(World& world) {
    Entity e = world.create();
    Transform t;
    t.position = Vec3(0, -1, 0);   // top face at y = 0
    world.add<Transform>(e, t);
    Collider c;
    c.halfExtent = Vec3(50, 1, 50);
    world.add<Collider>(e, c);
    world.add<RigidBody>(e, RigidBody{BodyMotion::Static, INVALID_PHYSICS_BODY});
    return e;
}

// A bouncy dropper: spheres and boxes land, rebound, and settle — each new
// touch is a Collision, so one drop plays a little diminishing drum roll.
void addDropper(World& world, int index) {
    Entity e = world.create();
    Transform t;
    t.position = Vec3(-7.0 + index * 2.0, 3.0 + (index % 4) * 1.5,
                      -4.0 - (index % 3) * 1.5);
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});
    Collider c;
    if (index % 2 == 0) {
        c.shape = ColliderShape::Sphere;
        c.radius = 0.4;
    } else {
        c.shape = ColliderShape::Box;
        c.halfExtent = Vec3(0.35, 0.35, 0.35);
    }
    c.restitution = 0.55;
    world.add<Collider>(e, c);
    world.add<RigidBody>(e, RigidBody{BodyMotion::Dynamic, INVALID_PHYSICS_BODY});
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
    AudioSystem audioSystem;
    PhysicsSystem physics;
    physics.initialize();

    // The arena's procedural clips (same seeds as ArenaSoundSystem).
    auto shot = sfx::gunshot(RATE, 7);
    audioSystem.registerClip("sfx/shot",
                             audio.createClip(shot.data(), shot.size(), 1, RATE));
    auto knock = sfx::impact(RATE, 11);
    audioSystem.registerClip(
        "sfx/impact", audio.createClip(knock.data(), knock.size(), 1, RATE));

    // PlaySound -> voice (what AudioSystem::onStart wires in-game).
    events.subscribe<PlaySound>(
        [&](const PlaySound& cue) { audioSystem.onPlaySound(cue, audio); });

    // Collision -> impact cue: the ArenaSoundSystem reaction, verbatim.
    int impacts = 0;
    events.subscribe<Collision>([&](const Collision& hit) {
        if (hit.speed < 2.0) return;
        PlaySound cue;
        cue.clip = "sfx/impact";
        cue.spatial = true;
        cue.position = hit.position;
        cue.range = 45.0;
        double severity = std::min(hit.speed / 14.0, 1.0);
        cue.volume = static_cast<float>(0.25 + 0.75 * severity);
        cue.pitch = static_cast<float>(0.85 + 0.35 * severity);
        events.enqueue(cue);
        ++impacts;
    });

    // Ears where a player would stand, facing the drop zone.
    Entity listener = world.create();
    world.add<AudioListener>(listener, {});
    Transform ears;
    ears.position = Vec3(0, 1.7, 6);
    world.add<Transform>(listener, ears);

    addFloor(world);

    const double dt = 1.0 / 60.0;
    const int ticks = 12 * 60;
    const uint64_t framesPerTick = RATE / 60;
    std::vector<float> mix;
    mix.reserve(ticks * framesPerTick * 2);
    std::vector<float> block(framesPerTick * 2);
    CameraState unusedCamera;

    int spawned = 0, shots = 0;
    for (int tick = 0; tick < ticks; tick++) {
        double t = tick * dt;

        // A new body tumbles in every 1.1 s for the first 9 s.
        if (spawned < 8 && t >= 0.5 + spawned * 1.1) addDropper(world, spawned++);
        // The gun cracks off four flat shots between the landings.
        if (shots < 4 && t >= 2.0 + shots * 2.5) {
            PlaySound cue;
            cue.clip = "sfx/shot";
            cue.volume = 0.9f;
            events.enqueue(cue);
            ++shots;
        }

        physics.createBodies(world);
        physics.step(world, dt);
        physics.publishContacts(world, events);   // Jolt contacts -> Collision
        events.dispatchQueued();                  // cues land this frame

        audioSystem.syncListener(world, audio, unusedCamera);
        audioSystem.step(world, audio);

        uint64_t rendered = audio.readFrames(block.data(), framesPerTick);
        mix.insert(mix.end(), block.begin(), block.begin() + rendered * 2);
    }

    physics.shutdown();
    writeWav16("impact_demo.wav", mix, RATE, 2);
    std::printf("Wrote impact_demo.wav: %.1f s stereo — %d bodies dropped, "
                "%d impact cues, %d gunshots\n",
                mix.size() / 2.0 / RATE, spawned, impacts, shots);
    return 0;
}
