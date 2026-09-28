#include "nature_collider_system.h"

#include "physics_system.h"
#include "../components.h"
#include "../world.h"
#include "../../log.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace engine {

void NatureColliderSystem::fixedUpdate(FrameContext& ctx) {
    const NatureColliders* nc = nullptr;
    ctx.world.each<NatureColliders>([&](Entity, NatureColliders& c) { if (!nc) nc = &c; });
    if (!nc || nc->shapes.empty()) return;
    // the player, or the car it drives (as the terrain collider window does)
    Vec3 at, prev;
    bool found = false;
    ctx.world.each<Transform, ControlledBy>([&](Entity e, Transform& t, ControlledBy&) {
        if (found) return;
        Entity tracked = e;
        if (const InVehicle* iv = ctx.world.get<InVehicle>(e))
            if (iv->vehicle.valid() && ctx.world.alive(iv->vehicle) && ctx.world.has<Transform>(iv->vehicle)) tracked = iv->vehicle;
        const Transform* tt = tracked == e ? &t : ctx.world.get<Transform>(tracked);
        at = tt->position;
        prev = at;
        if (const PrevTransform* pt = ctx.world.get<PrevTransform>(tracked)) prev = pt->value.position;
        found = true;
    });
    if (!found) return;
    const Vec3 vel = (at - prev) * (1.0 / std::max(1e-6, ctx.clock.fixedStep()));
    const Vec3 ahead = at + vel * 1.5;
    constexpr double kReach = 60.0;
    const double C = NatureColliders::kCell;
    // the cells wanted: round the body and round where it will be
    std::unordered_map<int64_t, double> want;   // cell -> distance from the body
    for (const Vec3& c : {at, ahead}) {
        const int i0 = static_cast<int>(std::floor((c.x - kReach) / C)), i1 = static_cast<int>(std::floor((c.x + kReach) / C));
        const int j0 = static_cast<int>(std::floor((c.z - kReach) / C)), j1 = static_cast<int>(std::floor((c.z + kReach) / C));
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) {
                const int64_t k = NatureColliders::key(i, j);
                if (!nc->cells.count(k)) continue;
                const double cx = (i + 0.5) * C, cz = (j + 0.5) * C;
                const double d = std::hypot(cx - at.x, cz - at.z);
                auto it = want.find(k);
                if (it == want.end() || d < it->second) want[k] = d;
            }
    }
    PhysicsWorld& pw = physics_.physicsWorld();
    // drop the cells left behind
    for (auto it = live_.begin(); it != live_.end();) {
        if (want.count(it->first)) { ++it; continue; }
        for (PhysicsBodyId id : it->second) pw.removeBody(id);
        it = live_.erase(it);
    }
    // add the missing, nearest first, a budget a step (the cell under the body always)
    std::vector<std::pair<double, int64_t>> missing;
    for (const auto& [k, d] : want) if (!live_.count(k)) missing.push_back({d, k});
    std::sort(missing.begin(), missing.end());
    int budget = 600;
    for (const auto& [d, k] : missing) {
        const std::vector<int>& idx = nc->cells.at(k);
        if (budget <= 0 && d > C) break;
        std::vector<PhysicsBodyId>& ids = live_[k];
        for (int s : idx) {
            const NatureCollider& c = nc->shapes[static_cast<std::size_t>(s)];
            const PhysicsBodyId id = c.capsule
                ? pw.addCapsule(c.half.y, c.half.x, c.centre, c.orientation, BodyMotion::Static, 0.0, 0.8)
                : pw.addBox(c.half, c.centre, c.orientation, BodyMotion::Static, 0.0, 0.8);
            if (id != INVALID_PHYSICS_BODY) ids.push_back(id);
        }
        budget -= static_cast<int>(idx.size());
    }
}

void NatureColliderSystem::onStop(FrameContext&) {
    PhysicsWorld& pw = physics_.physicsWorld();
    for (auto& [k, ids] : live_)
        for (PhysicsBodyId id : ids) pw.removeBody(id);
    live_.clear();
}

}  // namespace engine
