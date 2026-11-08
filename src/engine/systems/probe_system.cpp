#include "probe_system.h"
#include "../components.h"
#include "../lights/light_kinds.h"   // lightBook().probes
#include "../procgen/height_pyramid.h"
#include "../procgen/noise.h"
#include "../procgen/terrain.h"
#include "../world.h"
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
    std::fill(resident_.begin(), resident_.end(), -1);
    pending_.clear();
    lastEye_ = Vec3(1e30, 0, 0);
    LOG_INFO << "[probes] " << sites_.size() << " sites along the streets ("
             << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() << " ms)";
}

void ProbeSystem::render(FrameContext& ctx) {
    ProbeBoard& board = ctx.view.probes;
    board.captures.clear();
    static const bool off = probesOff();
    if (off || !ctx.renderer.supportsProbeCapture()) return;
    placeSites(ctx);
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
