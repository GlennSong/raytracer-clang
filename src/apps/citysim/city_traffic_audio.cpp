#include "city_traffic_audio.h"

#include <cstddef>

namespace citysim {

using engine::Vec2;
using engine::Vec3;

void CityTrafficAudioSystem::update(engine::FrameContext& ctx) {
    if (!ctx.audio.ready() || !city_.built()) return;

    const std::vector<Agent>& agents = city_.sim().agents();
    const Vec3 ear = ctx.view.camera.position;

    std::vector<engine::EngineVoicePool::Engine> heard;
    // (the near list: a far car has no body and no note -- not all 100k agents a frame; and a cheap reject on the
    // plan's position before working out a pose)
    for (int ni : city_.sim().nearAgents()) {
        const std::size_t i = static_cast<std::size_t>(ni);
        const Agent& a = agents[i];
        if (a.mode != Agent::Mode::Driver) continue;
        if (a.far()) continue;        // no body, no note
        if (a.released) continue;     // the player's car — VehicleSystem's voice
        {
            const engine::Real px = a.pos.x - ear.x, pz = a.pos.y - ear.z, r = kAudibleRange + 20.0;
            if (px * px + pz * pz > r * r) continue;
        }
        Vec3 pos;
        Vec2 heading;
        if (!city_.agentWorldPose(static_cast<int>(i), pos, heading)) continue;
        const engine::Real d2 = (pos - ear).lengthSquared();
        if (d2 > kAudibleRange * kAudibleRange) continue;
        // (phased by agent id, its own spread beside the physics cars')
        heard.push_back({static_cast<uint64_t>(i), pos, a.speed, engine::EngineVoicePool::kEstimate,
                         static_cast<engine::Real>(i % 23) * engine::Real(0.29), d2});
    }
    engines_.update(ctx.audio, heard, ctx.clock.simulatedTime(), ctx.frameDelta);
}

void CityTrafficAudioSystem::onStop(engine::FrameContext& ctx) {
    engines_.stopAll(ctx.audio);
}

}  // namespace citysim
