#include "light_system.h"
#include "render_system.h"      // LampLightsRequest
#include "../components.h"
#include "../vehicle_lamps.h"   // duskRamp
#include "../lights/light_picker.h"

#include <cstdlib>

namespace engine {

void LightSystem::onStart(FrameContext& ctx) {
    onLampLights_ = ctx.events.listen<LampLightsRequest>([this](const LampLightsRequest& r) { lampLights_ = r.on; });
}

void LightSystem::render(FrameContext& ctx) {
    LightBoard& board = ctx.view.lights;
    board.points.clear();
    board.spots.clear();
    const LightBook& book = lightBook();
    static const int streetLamp = lightKindId("street_lamp");
    static const int stationLamp = lightKindId("station_lamp");
    static const int streetPool = book.pool("street");
    static const int beaconPool = book.pool("beacons");

    // the pools, as the level sets them
    std::vector<LightPool> pools = book.pools;
    float lampRange = 0.0f;
    ctx.world.each<CitySimConfig>([&](Entity, CitySimConfig& c) {
        if (streetPool >= 0) {
            pools[static_cast<std::size_t>(streetPool)].count = std::max(1, c.lampLightCount);
            pools[static_cast<std::size_t>(streetPool)].within = std::max(10.0f, c.lampLightRadius);
        }
        lampRange = std::max(2.0f, c.lampLightRange);
        if (beaconPool >= 0) {
            pools[static_cast<std::size_t>(beaconPool)].count = c.lightCount;
            pools[static_cast<std::size_t>(beaconPool)].within = c.lightRadius;
        }
    });
    // RT_LAMP_LIGHTS / RT_LAMP_RADIUS: the clustered lights' trial (ADR-0361), the street pool overridden
    static const int envLamps = [] { const char* e = std::getenv("RT_LAMP_LIGHTS"); return e ? std::atoi(e) : 0; }();
    static const float envRadius = [] { const char* e = std::getenv("RT_LAMP_RADIUS"); return e ? static_cast<float>(std::atof(e)) : 0.0f; }();
    if (streetPool >= 0) {
        LightPool& sp = pools[static_cast<std::size_t>(streetPool)];
        if (envLamps > 0) sp.count = envLamps;
        if (envRadius > 0) sp.within = envRadius;
        if (!lampLights_) sp.count = 0;
    }
    const float dusk = static_cast<float>(duskRamp(ctx.view.lighting.solarElevation));

    // the static emitters, in the street pool's reach (and only after dark: by day they light nothing)
    cands_.clear();
    const Vec3 eye = ctx.view.camera.position;
    if (dusk > 0.01f && streetPool >= 0) {
        const float within = pools[static_cast<std::size_t>(streetPool)].within;
        const Real reach2 = within > 0 ? Real(within) * within : 1e30;
        bool streetSeen = false;
        ctx.world.each<StreetFurniture>([&](Entity, StreetFurniture& f) {
            if (f.lampHeads.empty() || streetSeen) return;
            streetSeen = true;
            for (const Vec3& h : f.lampHeads)
                if ((h - eye).lengthSquared() < reach2) {
                    LightCandidate c;
                    c.kind = streetLamp;
                    c.position = h;
                    c.range = lampRange;
                    cands_.push_back(c);
                }
        });
        ctx.world.each<RailTracks>([&](Entity, RailTracks& rt) {
            for (const RailTrack& t : rt.tracks)
                for (const RailStop& st : t.stops)
                    for (const Vec3& h : st.lamps)
                        if ((h - eye).lengthSquared() < reach2) {
                            LightCandidate c;
                            c.kind = stationLamp;
                            c.position = h;
                            cands_.push_back(c);
                        }
        });
    }
    // the producers' channels, points first then spots (the order the forward renderer's 32 fill in)
    for (LightChannel ch : {LightChannel::Rooms, LightChannel::Cabs, LightChannel::Beacons, LightChannel::Vehicles, LightChannel::Tool}) {
        const std::vector<LightCandidate>& list = board.channel(ch);
        cands_.insert(cands_.end(), list.begin(), list.end());
    }
    pickLights(book, pools, cands_, eye, dusk, board.points, board.spots);
}

}  // namespace engine
