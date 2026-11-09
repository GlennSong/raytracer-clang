#include "probe_system.h"
#include "../components.h"
#include "../lights/light_kinds.h"   // lightBook().probes
#include "../procgen/height_pyramid.h"
#include "../procgen/noise.h"
#include "../procgen/terrain.h"
#include "../world.h"
#include "building_interior_system.h"   // storeyOf (the storey the interiors stream)
#include "../procgen/city/building_records.h"
#include "../../log.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>

namespace engine {

namespace {
bool probesOff() {
    const char* v = std::getenv("RT_PROBES");
    return v && std::strcmp(v, "0") == 0;
}
}  // namespace

// The sites along the level's road network (LevelRoadGraph: the streets and corridors welded at load, as drawn),
// again whenever it changes (a level, a re-plan).
void ProbeSystem::placeSites(FrameContext& ctx) {
    std::size_t roads = 0;
    ctx.world.each<LevelRoadGraph>([&](Entity, LevelRoadGraph& g) { roads += g.graph.edges.size() + 1; });
    if (roads == roadsSeen_) return;
    roadsSeen_ = roads;
    const auto t0 = std::chrono::steady_clock::now();
    const TerrainLodConfig* cfg = nullptr;
    ctx.world.each<TerrainLodConfig>([&](Entity, TerrainLodConfig& c) { if (!cfg) cfg = &c; });
    std::unique_ptr<Noise> noise = cfg && !cfg->baked ? std::make_unique<Noise>(cfg->seed) : nullptr;
    const ProbeGroundFn ground = [&](double x, double z) {
        if (!cfg) return 0.0;
        return cfg->baked ? cfg->baked->height(x, z) : terrainHeight(cfg->params, *noise, x, z);
    };
    sites_.clear();
    const ProbeRules& rules = lightBook().probes;
    ctx.world.each<LevelRoadGraph>([&](Entity, LevelRoadGraph& g) { placeProbeSites(g.graph, ground, rules, sites_); });
    streetCount_ = sites_.size();
    storeyKey_ = 0;
    storeyRecord_ = nullptr;
    std::fill(resident_.begin(), resident_.end(), -1);
    pending_.clear();
    lastEye_ = Vec3(1e30, 0, 0);
    LOG_INFO << "[probes] " << sites_.size() << " sites along the streets ("
             << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() << " ms)";
}

// THE STOREY THE CAMERA STANDS ON (ADR-0380; Glenn: "probes just need to exist for that floor when you're on it"): its
// sites after the street's, again whenever the building or the storey changes; the slots that held the last storey's
// give them up (the street's keep theirs). Nothing is kept once you leave: a floor is captured as you arrive.
void ProbeSystem::placeStorey(FrameContext& ctx) {
    const Vec3 eye = ctx.view.camera.position;
    const CityBuildings* cb = nullptr;
    ctx.world.each<CityBuildings>([&](Entity, CityBuildings& b) { if (!cb) cb = &b; });
    const BuildingRecord* r = cb ? cb->recordAt(Vec2(eye.x, eye.z), eye.y) : nullptr;
    uint64_t key = 0;
    double floorY = 0, ceilY = 0;
    if (r && r->plan.size() >= 3) {
        if (r != storeyRecord_) {
            storeyPlans_ = storeyPlans(r->plan, r->params);
            storeyRecord_ = r;
        }
        const int f = BuildingInteriorSystem::storeyOf(*r, eye.y);
        if (f >= 0 && f < static_cast<int>(storeyPlans_.size())) {
            const StoreyPlan& sp = storeyPlans_[static_cast<std::size_t>(f)];
            floorY = r->baseY + sp.y0;
            ceilY = floorY + sp.h;
            key = (static_cast<uint64_t>(reinterpret_cast<uintptr_t>(r)) << 8) ^ static_cast<uint64_t>(f + 1);
        }
    } else {
        storeyRecord_ = nullptr;
    }
    if (key == storeyKey_) return;
    storeyKey_ = key;
    sites_.resize(streetCount_);
    for (int& s : resident_)
        if (s >= static_cast<int>(streetCount_)) s = -1;   // (their queued faces are dropped as stale)
    if (key != 0) {
        const OBB2 ob = orientedBoundingBox(r->plan);
        const ProbeRules& rules = lightBook().probes;
        placeStoreyProbeSites(ob.center, ob.axis, ob.half, floorY, ceilY, rules.indoorSpacing, rules.indoorHeight, sites_);
    }
    lastEye_ = Vec3(1e30, 0, 0);   // (the slots are picked again this frame)
}

void ProbeSystem::render(FrameContext& ctx) {
    ProbeBoard& board = ctx.view.probes;
    board.captures.clear();
    static const bool off = probesOff();
    // (only the physical sky's branch of the scene pass reads a probe: without it, none is captured or relit)
    if (off || !ctx.renderer.supportsProbeCapture() || !ctx.view.lighting.physicalSky.enabled) return;
    placeSites(ctx);
    placeStorey(ctx);
    const Vec3 eye = ctx.view.camera.position;
    // who holds the slots: again once the camera has moved a few metres
    const double mx = eye.x - lastEye_.x, mz = eye.z - lastEye_.z;
    if (mx * mx + mz * mz > 16.0 || board.slots.size() != resident_.size()) {
        lastEye_ = eye;
        const std::vector<int> before = resident_;
        if (pickResidentProbes(sites_, eye, lightBook().probes.hysteresis, resident_) || board.slots.size() != resident_.size()) {
            board.slots.assign(resident_.size(), ReflectionProbe());
            // the new arrivals, nearest first, six faces each
            std::vector<std::pair<double, int>> joined;
            for (std::size_t i = 0; i < resident_.size(); ++i) {
                const int s = resident_[i];
                if (s < 0) {
                    // empty: a box inside out holds nothing
                    board.slots[i] = ReflectionProbe(Vec3(0, -1e6, 0), 0.0f, Vec3(1, 1, 1), Vec3(-1, -1, -1));
                    continue;
                }
                const ProbeSite& site = sites_[static_cast<std::size_t>(s)];
                board.slots[i] = ReflectionProbe(site.position, 0.0f, site.boxMin, site.boxMax);
                board.slots[i].indoor = site.indoor;
                if (s != before[i]) joined.emplace_back((site.position - eye).lengthSquared(), static_cast<int>(i));
            }
            std::sort(joined.begin(), joined.end());
            for (const std::pair<double, int>& j : joined)
                for (int f = 0; f < 6; ++f) pending_.push_back(Pending{j.second, resident_[static_cast<std::size_t>(j.second)], f});
            ++board.revision;
        }
    }
    // this frame's faces (a slot given away since it was queued is skipped)
    while (!pending_.empty() && static_cast<int>(board.captures.size()) < kFacesPerFrame) {
        const Pending p = pending_.front();
        pending_.pop_front();
        if (resident_[static_cast<std::size_t>(p.slot)] != p.site) continue;
        board.captures.push_back(ProbeBoard::Face{static_cast<uint32_t>(p.slot), p.face});
    }
}

}  // namespace engine
