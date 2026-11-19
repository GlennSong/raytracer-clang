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
// A car entity's box is at where its body is this fixed step (city_car.h CityCarPose, ADR-0454), and a walker's where
// the walker stands this step (city_poses.h CityPoses::walkerBoxPoses), every step. (The old driving's cars, read from
// the drawing's bakes, and its physical tier -- Jolt cars chasing the old sim's ghosts -- went with it: ADR-0461.)
class CityPhysicsSystem : public engine::System {
public:
    CityPhysicsSystem(CityRenderSystem& city, engine::PhysicsSystem& physics)
        : city_(city), physics_(physics) {}

    void fixedUpdate(engine::FrameContext& ctx) override;
    void onStop(engine::FrameContext& ctx) override;

    // --- testable core (no FrameContext), the CityRenderSystem pattern ---
    // One physics-bridge step: track every kinematic proxy. The headless tests drive this directly.
    void step(engine::World& world, engine::Real dt);
    // The kinematic proxy boxes standing in for the city's cars — the
    // bodies the player actually bumps into. Exposed so the walk-through gate
    // (#26) can check every drawn car really has one where it is drawn. The
    // value type is private, so this hands out just the body ids.
    std::vector<engine::PhysicsBodyId> carProxyBodies() const {
        std::vector<engine::PhysicsBodyId> out;
        out.reserve(carProxies_.size());
        for (const auto& kv : carProxies_) out.push_back(kv.second.id);
        return out;
    }
    // The same boxes with their half extents (a van's is bigger than a sedan's), for overlap gates.
    // key: the uid of the agent at the car's wheel, or negative for a car with nobody at its wheel (-2 - its number).
    struct ProxyBox { engine::PhysicsBodyId id; engine::Vec3 he; long long key; };
    std::vector<ProxyBox> carProxyBoxes() const {
        std::vector<ProxyBox> out;
        out.reserve(carProxies_.size());
        for (const auto& kv : carProxies_) out.push_back({kv.second.id, kv.second.he, kv.first});
        return out;
    }
    // A HOUSEHOLD CAR THAT HAS JUST BECOME A JOLT CAR (the near tier, ADR-0449): its kinematic box goes NOW. Left to
    // the next sync it outlived the switch by one physics step, and the new chassis was made inside it: shoved,
    // damaged and off its lane (measured headless with the boxes on: 1040 chassis hits). False: it had no box.
    bool dropEntityCarBox(engine::World& world, engine::Entity car);
    // ...a household car's kinematic box (INVALID_PHYSICS_BODY: it has none): the cars the driving drives, which a near
    // car's look ahead passes over as it sees them already (ADR-0451)
    engine::PhysicsBodyId entityCarBox(engine::World& world, engine::Entity car) const;
    // Is this body one of the walkers' kinematic boxes (what a near car's hit was, ADR-0450)?
    bool isPedestrianBox(engine::PhysicsBodyId id) const {
        for (const std::pair<const long long, ProxyBody>& kv : pedProxies_)
            if (kv.second.id == id) return true;
        return false;
    }
private:
    void releaseBodies();
    bool entityCarKey(engine::World& world, engine::Entity car, long long& key) const;
    // P4.3: one live kinematic proxy box per body, keyed by its STABLE identity (a walker's uid, a car entity's
    // entityCarDrawKey turned into a key the same way), so one arriving or leaving adds or removes ONE box.
    struct ProxyBody {
        engine::PhysicsBodyId id{};
        engine::Vec3 he{};     // collider half extent (a van's box is bigger than a sedan's)
        uint32_t stamp = 0;    // last sync pass that saw this body
    };
    // Keep `proxies` matched to this step's bodies as an incremental DIFF: add a box for a new one (spawned at its
    // pose — never swept in), remove boxes whose body went, move the rest. `withCarEntities`: the car entities' boxes
    // from their CityCarPose, of the model each is drawn as (one of `carModels`), sized by `extents` (one per model);
    // `posed`: poses given for this step, by agent (the walkers'), sized by `extents.front()`.
    void syncKinematic(engine::World& world, std::size_t carModels, const std::vector<engine::Vec3>& extents,
                       std::unordered_map<long long, ProxyBody>& proxies, engine::Real dt, bool withCarEntities,
                       const std::vector<std::pair<int, engine::Mat4>>* posed = nullptr);
    std::vector<std::pair<int, engine::Mat4>> walkerPoses_;   // (scratch: CityPoses::walkerBoxPoses, each step)

    CityRenderSystem& city_;
    engine::PhysicsSystem& physics_;
    std::unordered_map<long long, ProxyBody> carProxies_;   // keyed as above
    std::unordered_map<long long, ProxyBody> pedProxies_;
    uint32_t proxyStamp_ = 0;               // sync-pass counter (stale detection)
    std::vector<long long> staleScratch_;   // keys to drop, sorted (determinism)
    std::vector<engine::PhysicsBodyId> poleBodies_;   // static, one per signal pole
    bool polesBuilt_ = false;
};

}  // namespace citysim

#endif
