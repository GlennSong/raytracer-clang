#include "city_traffic_audio.h"

#include "city_car.h"

#include <cstddef>

namespace citysim {

using engine::Vec2;
using engine::Vec3;

namespace {
// A car entity's key in the voice pool: clear of the physics cars' keys, and stable for the run (its number).
constexpr uint64_t kHouseholdCarKey = uint64_t(1) << 40;
}  // namespace

void CityTrafficAudioSystem::update(engine::FrameContext& ctx) {
    if (!ctx.audio.ready() || !city_.built()) return;

    const Vec3 ear = ctx.view.camera.position;

    std::vector<engine::EngineVoicePool::Engine> heard;
    // THE CITY'S CARS NEAR THE EAR (carsNear, ADR-0445): the car entities on the road. One the player holds has an
    // engine::Vehicle and is VehicleSystem's: carsNear leaves it out. One car, one engine.
    std::vector<CarSeen> cars;
    carsNear(ctx.world, Vec2(ear.x, ear.z), kAudibleRange + 20.0, cars);
    for (const CarSeen& c : cars) {
        if (!c.onRoad) continue;   // standing in its space: its engine is off
        const Vec3 pos(c.pos.x, city_.surfaceY(c.pos), c.pos.y);
        const uint64_t key = kHouseholdCarKey | c.id;
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
