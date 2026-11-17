#include "city_traffic_audio.h"

#include "city_car.h"

#include <cstddef>

namespace citysim {

using engine::Vec2;
using engine::Vec3;

namespace {
// A household car's key in the voice pool: above every agent index (the old sim's cars' keys), and stable for the run
// (its number).
constexpr uint64_t kHouseholdCarKey = uint64_t(1) << 40;
}  // namespace

void CityTrafficAudioSystem::update(engine::FrameContext& ctx) {
    if (!ctx.audio.ready() || !city_.built()) return;

    const Vec3 ear = ctx.view.camera.position;

    std::vector<engine::EngineVoicePool::Engine> heard;
    // THE CITY'S CARS NEAR THE EAR (carsNear, ADR-0445): the old sim's driven cars (a far car has no body and no note;
    // a released one is the player's, voiced by VehicleSystem) and the households' cars on the road. A household car
    // the player holds has an engine::Vehicle and is VehicleSystem's: carsNear leaves it out. One car, one engine.
    // (a cheap reject on the plan's position before working out a pose)
    std::vector<CarSeen> cars;
    carsNear(city_.sim(), ctx.world, Vec2(ear.x, ear.z), kAudibleRange + 20.0, cars);
    for (const CarSeen& c : cars) {
        Vec3 pos;
        Vec2 heading;
        uint64_t key = 0;
        if (c.agent >= 0) {
            if (!city_.agentWorldPose(c.agent, pos, heading)) continue;
            key = static_cast<uint64_t>(c.agent);
        } else {
            if (!c.onRoad) continue;   // standing in its space: its engine is off
            pos = Vec3(c.pos.x, city_.surfaceY(-1, 0, c.pos), c.pos.y);
            key = kHouseholdCarKey | c.id;
        }
        const engine::Real d2 = (pos - ear).lengthSquared();
        if (d2 > kAudibleRange * kAudibleRange) continue;
        // (phased by its key, its own spread beside the physics cars')
        heard.push_back({key, pos, c.speed, 0, static_cast<engine::Real>(key % 23) * engine::Real(0.29), d2, true});
    }
    engines_.update(ctx.audio, heard, ctx.clock.simulatedTime(), ctx.frameDelta);
}

void CityTrafficAudioSystem::onStop(engine::FrameContext& ctx) {
    engines_.stopAll(ctx.audio);
}

}  // namespace citysim
