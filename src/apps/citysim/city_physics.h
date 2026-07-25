#ifndef RAYTRACER_APPS_CITYSIM_CITY_PHYSICS_H
#define RAYTRACER_APPS_CITYSIM_CITY_PHYSICS_H

#include "../../engine/system.h"
#include "../../engine/systems/physics_system.h"   // PhysicsSystem, PhysicsBodyId
#include "city_render.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace citysim {

// Gives the kinematic AI cars a physics presence (ADR-0060). The CitySim decides
// each car's pose; this viewer-side system mirrors that pose into a KINEMATIC
// Jolt box per car (via PhysicsWorld::moveKinematic), so the player and the
// physics gun collide with cars and a moving car pushes what it touches — without
// the cars themselves being dynamically simulated (the sim still owns their
// motion). Lives in the viewer/editor build (it needs Jolt), not engine_core.
//
// It reads the car poses straight from CityRenderSystem's baked InstanceGroup, so
// the colliders track exactly what is drawn.
class CityPhysicsSystem : public engine::System {
public:
    CityPhysicsSystem(CityRenderSystem& city, engine::PhysicsSystem& physics)
        : city_(city), physics_(physics) {}

    void fixedUpdate(engine::FrameContext& ctx) override;
    void onStop(engine::FrameContext& ctx) override;

    // --- testable core (no FrameContext), the CityRenderSystem pattern ---
    // One physics-bridge step: possess/drive the physical tier, then track
    // every kinematic proxy. The headless soak drives this directly.
    void step(engine::World& world, engine::Real dt);
    // The live physical roster (agent ids + vehicle ids) for gates.
    struct Possessed {
        int agent = -1;
        engine::PhysicsWorld::VehicleId vid =
            engine::PhysicsWorld::INVALID_VEHICLE;
        engine::Real stuckTime = 0;   // body pinned while its ghost recedes
        engine::Real lastDist = 0;    // gap last tick, for closing-rate
    };
    const std::vector<Possessed>& possessed() const { return possessed_; }
    // Times the backstop snapped a lost body onto its ghost. The soak gate
    // asserts the CONTROLLER never needs it; in production it's the fuse.
    int snapCount() const { return snapCount_; }

private:
    void releaseBodies();
    // P4.3: one live kinematic proxy box per drawn instance, keyed by the
    // instance's STABLE identity — the baking agent's uid, or a negative bake
    // ordinal for build-time scenery cars. Replaces the old positional pool
    // that fully rebuilt whenever the drawn count changed (O(n) churn every
    // tier promotion/demotion).
    struct ProxyBody {
        engine::PhysicsBodyId id{};
        char parked = 0;       // last tick's possessed-parked state
        uint32_t stamp = 0;    // last sync pass that saw this instance
    };
    // Keep `proxies` matched to the instances across `groups` as an
    // incremental DIFF: add a body for a newly drawn instance (spawned at its
    // pose — never swept in), remove bodies whose instance vanished (tier
    // demotion, release), move the rest. `groupExtents` gives the collider
    // half-extent for each group (so a van's box is bigger than a sedan's).
    // `agentIds` (parallel per group): the baking agent id of each instance —
    // the key source, and instances whose agent is currently POSSESSED park
    // their proxy box far below instead of tracking (a kinematic box inside
    // the dynamic chassis would fight it — and it must park the SAME tick the
    // body spawns, which is why this reads the live possessed set, not a flag
    // captured at bake time).
    void syncKinematic(engine::World& world, const std::vector<engine::Entity>& groups,
                       const std::vector<engine::Vec3>& groupExtents,
                       std::unordered_map<long long, ProxyBody>& proxies,
                       engine::Real dt,
                       const std::vector<std::vector<int>>* agentIds = nullptr);
    // R5 (roads-v2.1): the PHYSICAL tier — the N moving drivers nearest the
    // player run on real Jolt wheeled vehicles, driven by the SAME plan the
    // kinematic brains produce (driveTowards chases the sim's ghost); their
    // render pose comes from the body (city_.setAgentPhysPose).
    void possessTier(engine::World& world, engine::Real dt);

    CityRenderSystem& city_;
    engine::PhysicsSystem& physics_;
    std::unordered_map<long long, ProxyBody> carProxies_;   // uid/scenery-keyed
    std::unordered_map<long long, ProxyBody> pedProxies_;
    uint32_t proxyStamp_ = 0;               // sync-pass counter (stale detection)
    std::vector<long long> staleScratch_;   // keys to drop, sorted (determinism)
    std::vector<int> nearScratch_;          // possessTier grid candidates (P4.1)
    std::vector<engine::PhysicsBodyId> poleBodies_;   // static, one per signal pole
    bool polesBuilt_ = false;
    std::vector<Possessed> possessed_;
    int snapCount_ = 0;
    int maxPhysical_ = 12;
    engine::Real possessRadius_ = 90.0;   // acquire inside; drop past 1.3x
};

}  // namespace citysim

#endif
