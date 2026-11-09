#include "light_system.h"
#include "render_system.h"      // LampLightsRequest
#include "../components.h"
#include "../vehicle_lamps.h"   // duskRamp
#include "../lights/light_picker.h"
#include "../sky/sky_atmosphere.h"   // kLuxPerUnit
#include "../../log.h"

#include <cmath>
#include <cstdio>

namespace engine {

namespace {
uint64_t cellKey(int64_t cx, int64_t cz) { return (static_cast<uint64_t>(cx) << 32) ^ static_cast<uint64_t>(static_cast<uint32_t>(cz)); }
int64_t cellOf(Real v) { return static_cast<int64_t>(std::floor(v / LightSystem::kCell)); }
}  // namespace

void LightSystem::onStart(FrameContext& ctx) {
    onLampLights_ = ctx.events.listen<LampLightsRequest>([this](const LampLightsRequest& r) { lampLights_ = r.on; });
}

// The street's and the stations' lamps into the grid, again whenever their count changes (a level, a streamed tile).
void LightSystem::indexStatics(FrameContext& ctx) {
    const StreetFurniture* sf = nullptr;
    ctx.world.each<StreetFurniture>([&](Entity, StreetFurniture& f) { if (!sf && !f.lampHeads.empty()) sf = &f; });
    std::size_t rail = 0;
    ctx.world.each<RailTracks>([&](Entity, RailTracks& rt) {
        for (const RailTrack& t : rt.tracks)
            for (const RailStop& st : t.stops) rail += st.lamps.size();
    });
    const std::size_t street = sf ? sf->lampHeads.size() : 0;
    if (street == streetSeen_ && rail == railSeen_) return;
    streetSeen_ = street;
    railSeen_ = rail;
    statics_.clear();
    cells_.clear();
    static const int streetLamp = lightKindId("street_lamp");
    static const int stationLamp = lightKindId("station_lamp");
    auto add = [&](int kind, const Vec3& at) {
        LightCandidate c;
        c.kind = kind;
        c.position = at;
        cells_[cellKey(cellOf(at.x), cellOf(at.z))].push_back(static_cast<uint32_t>(statics_.size()));
        statics_.push_back(c);
    };
    if (sf)
        for (const Vec3& h : sf->lampHeads) add(streetLamp, h);
    ctx.world.each<RailTracks>([&](Entity, RailTracks& rt) {
        for (const RailTrack& t : rt.tracks)
            for (const RailStop& st : t.stops)
                for (const Vec3& h : st.lamps) add(stationLamp, h);
    });
}

void LightSystem::render(FrameContext& ctx) {
    LightBoard& board = ctx.view.lights;
    board.points.clear();
    board.spots.clear();
    const LightBook& loaded = lightBook();
    // UNDER THE PHYSICAL SKY (ADR-0369) each kind lights at its real candela, in engine units
    // (made again when lights.lua is: a reload publishes a new book)
    if (physicalFrom_ != &loaded) {
        physicalBook_ = loaded;
        for (LightKind& k : physicalBook_.kinds)
            if (k.candela > 0.0f) k.intensity = static_cast<float>(k.candela / kLuxPerUnit);
        physicalFrom_ = &loaded;
    }
    const LightBook& book = ctx.view.lighting.physicalSky.enabled ? physicalBook_ : loaded;
    const int streetPool = book.pool("street");
    const int beaconPool = book.pool("beacons");
    const int streetLamp = book.kind("street_lamp");

    // THE BUDGETS, by what the renderer can light: the level's own lights first, the rest is ours
    const int capacity = ctx.renderer.lightCapacity();
    const bool many = capacity > 64;
    const int cap = std::max(0, capacity - static_cast<int>(ctx.view.lighting.pointLights.size() + ctx.view.lighting.spotLights.size()));
    std::vector<LightPool> pools = book.pools;
    for (LightPool& p : pools)
        if (many && p.hasMany) { p.count = p.manyCount; p.within = p.manyWithin; p.fade = p.manyFade; }
    float lampRange = 0.0f;
    ctx.world.each<CitySimConfig>([&](Entity, CitySimConfig& c) {
        lampRange = std::max(2.0f, c.lampLightRange);
        if (many) return;   // the level's counts are its forward budget
        if (streetPool >= 0) {
            pools[static_cast<std::size_t>(streetPool)].count = std::max(1, c.lampLightCount);
            pools[static_cast<std::size_t>(streetPool)].within = std::max(10.0f, c.lampLightRadius);
        }
        if (beaconPool >= 0) {
            pools[static_cast<std::size_t>(beaconPool)].count = c.lightCount;
            pools[static_cast<std::size_t>(beaconPool)].within = c.lightRadius;
        }
    });
    if (streetPool >= 0 && !lampLights_) pools[static_cast<std::size_t>(streetPool)].count = 0;
    const float dusk = static_cast<float>(duskRamp(ctx.view.lighting.solarElevation));

    // the rooms and the cabs first: the lights reach the renderer in this order, and the first 31 are the ones its
    // fixed loops still read -- grass and leaf transmission, the water's glint, and a probe texel or a ray's hit the
    // camera does not see (evaluateLighting's fallback); a point it sees, and the viewer's room, take their froxel's
    cands_.clear();
    for (LightChannel ch : {LightChannel::Rooms, LightChannel::Cabs}) {
        const std::vector<LightCandidate>& list = board.channel(ch);
        cands_.insert(cands_.end(), list.begin(), list.end());
    }
    // the static emitters in the street pool's reach (and only after dark: by day they light nothing)
    const Vec3 eye = ctx.view.camera.position;
    indexStatics(ctx);
    if (dusk > 0.01f && streetPool >= 0 && !statics_.empty()) {
        const float within = pools[static_cast<std::size_t>(streetPool)].within;
        const Real reach = within > 0 ? Real(within) : Real(4000);
        const Real reach2 = reach * reach;
        const int64_t x0 = cellOf(eye.x - reach), x1 = cellOf(eye.x + reach), z0 = cellOf(eye.z - reach), z1 = cellOf(eye.z + reach);
        for (int64_t cx = x0; cx <= x1; ++cx)
            for (int64_t cz = z0; cz <= z1; ++cz) {
                const std::unordered_map<uint64_t, std::vector<uint32_t>>::const_iterator it = cells_.find(cellKey(cx, cz));
                if (it == cells_.end()) continue;
                for (uint32_t i : it->second) {
                    const LightCandidate& s = statics_[i];
                    if ((s.position - eye).lengthSquared() >= reach2) continue;
                    cands_.push_back(s);
                    if (s.kind == streetLamp && lampRange > 0.0f) cands_.back().range = lampRange;
                }
            }
    }
    // the other channels
    for (LightChannel ch : {LightChannel::Beacons, LightChannel::Traffic, LightChannel::Vehicles, LightChannel::Tool}) {
        const std::vector<LightCandidate>& list = board.channel(ch);
        cands_.insert(cands_.end(), list.begin(), list.end());
    }
    pickLights(book, pools, cands_, eye, dusk, board.points, board.spots, cap);
    lastCands_ = cands_.size();
    lastPicked_ = board.points.size() + board.spots.size();
    if (++frames_ % 240 == 0 && dusk > 0.01f)
        LOG_INFO << "[lights] " << lastPicked_ << " lit of " << lastCands_ << " published (" << board.points.size()
                 << " points, " << board.spots.size() << " spots; renderer takes " << capacity << ")";
}

}  // namespace engine
