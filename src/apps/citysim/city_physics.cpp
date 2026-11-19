#include "city_physics.h"

#include "../../engine/components.h"   // InstanceGroup
#include "../../engine/world.h"
#include "../../profile.h"

#include <algorithm>
#include <cmath>

namespace citysim {

using engine::Vec3;
using engine::Quat;
using engine::Mat4;
using engine::Real;
using engine::Entity;
using engine::World;
using engine::InstanceGroup;
using engine::PhysicsWorld;
using engine::PhysicsBodyId;
using engine::BodyMotion;

void CityPhysicsSystem::releaseBodies() {
    PhysicsWorld& pw = physics_.physicsWorld();
    for (const auto& kv : carProxies_) pw.removeBody(kv.second.id);
    for (const auto& kv : pedProxies_) pw.removeBody(kv.second.id);
    for (PhysicsBodyId id : poleBodies_) pw.removeBody(id);
    carProxies_.clear();
    pedProxies_.clear();
    poleBodies_.clear();
    polesBuilt_ = false;
}

// Keep `proxies` matched to every body this step as an incremental DIFF (P4.3):
// each keys its box by a stable identity (a walker's uid, a car entity's
// entityCarDrawKey), so a car or a walker arriving adds ONE body, one leaving
// removes ONE, and everything else just moves.
void CityPhysicsSystem::syncKinematic(World& world, std::size_t carModels, const std::vector<Vec3>& extents,
                                      std::unordered_map<long long, ProxyBody>& proxies, Real dt, bool withCarEntities,
                                      const std::vector<std::pair<int, Mat4>>* posed) {
    RT_PROFILE_ZONE_NAMED("syncKinematic");
    PhysicsWorld& pw = physics_.physicsWorld();
    const std::vector<Agent>& agents = city_.sim().agents();
    const uint32_t stamp = ++proxyStamp_;
    // Proxy identity. An agent keys on its uid; a car entity with nobody at its wheel on -2 - its number: stable
    // across frames.
    const auto keyOf = [&](int ai) {
        return (ai >= 0 && ai < static_cast<int>(agents.size()))
                   ? static_cast<long long>(agents[static_cast<std::size_t>(ai)].uid)
                   : static_cast<long long>(ai);
    };
    // ONE BOX TO ITS POSE `m` this step
    const auto place = [&](long long key, const Mat4& m, const Vec3& he) {
        const Vec3 p(m.m[0][3], m.m[1][3], m.m[2][3]);
        const Real yaw = std::atan2(m.m[0][2], m.m[2][2]);   // heading from local +Z basis
        const Quat rot = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
        std::unordered_map<long long, ProxyBody>::iterator it = proxies.find(key);
        if (it == proxies.end()) {
            // Newly here (first frame, or a walker promoted into the K bubble): SPAWN the box at its pose -- arriving
            // is a discontinuous move, never a sweep.
            ProxyBody nb;
            nb.he = he;
            nb.id = pw.addBox(he, p, rot, BodyMotion::Kinematic,
                              /*restitution*/ 0.0, /*friction*/ 0.6);
            nb.stamp = stamp;
            proxies.emplace(key, nb);
            return;
        }
        it->second.stamp = stamp;
        pw.moveKinematic(it->second.id, p, rot, dt);
    };
    // THE CAR ENTITIES (ADR-0454): each box at where its car's body is THIS step (city_car.h CityCarPose), set every
    // fixed step whether the drawing baked or not.
    if (withCarEntities)
        world.each<CityCarBody, CityCarLink, CityCarPose>([&](Entity, CityCarBody& body, CityCarLink& link, CityCarPose& pose) {
            const int v = city_.sim().poses().models.variantOf(body.slot, body.type);
            if (v < 0 || static_cast<std::size_t>(v) >= carModels || extents.empty()) return;
            place(keyOf(CityRenderSystem::entityCarDrawKey(body, link)), pose.body,
                  extents[std::min(static_cast<std::size_t>(v), extents.size() - 1)]);
        });
    // POSES GIVEN FOR THIS STEP (the walkers', ADR-0454)
    if (posed && !extents.empty())
        for (const std::pair<int, Mat4>& w : *posed) place(keyOf(w.first), w.second, extents.front());
    // Bodies gone this pass (a walker demoted, a car the player took): drop
    // them. Keys sorted so removal order reproduces run to run (ADR-0002).
    staleScratch_.clear();
    for (const auto& kv : proxies)
        if (kv.second.stamp != stamp) staleScratch_.push_back(kv.first);
    std::sort(staleScratch_.begin(), staleScratch_.end());
    for (long long k : staleScratch_) {
        auto it = proxies.find(k);
        pw.removeBody(it->second.id);
        proxies.erase(it);
    }
}

bool CityPhysicsSystem::entityCarKey(engine::World& world, Entity car, long long& key) const {
    const CityCarBody* body = world.get<CityCarBody>(car);
    const CityCarLink* link = world.get<CityCarLink>(car);
    if (!body || !link) return false;
    // the key syncKinematic gave its box: the driver's uid, or the car's own (negative) key with nobody at the wheel
    const int ai = CityRenderSystem::entityCarDrawKey(*body, *link);
    const std::vector<Agent>& agents = city_.sim().agents();
    key = ai >= 0 && ai < static_cast<int>(agents.size()) ? static_cast<long long>(agents[static_cast<std::size_t>(ai)].uid)
                                                          : static_cast<long long>(ai);
    return true;
}

engine::PhysicsBodyId CityPhysicsSystem::entityCarBox(engine::World& world, Entity car) const {
    long long key = 0;
    if (!entityCarKey(world, car, key)) return engine::INVALID_PHYSICS_BODY;
    const std::unordered_map<long long, ProxyBody>::const_iterator it = carProxies_.find(key);
    return it == carProxies_.end() ? engine::INVALID_PHYSICS_BODY : it->second.id;
}

bool CityPhysicsSystem::dropEntityCarBox(engine::World& world, Entity car) {
    long long key = 0;
    if (!entityCarKey(world, car, key)) return false;
    std::unordered_map<long long, ProxyBody>::iterator it = carProxies_.find(key);
    if (it == carProxies_.end()) return false;
    physics_.physicsWorld().removeBody(it->second.id);
    carProxies_.erase(it);
    return true;
}

void CityPhysicsSystem::fixedUpdate(engine::FrameContext& ctx) {
    if (!city_.built()) return;
    step(ctx.world, ctx.clock.fixedStep());
}

void CityPhysicsSystem::step(World& world, Real dt) {
    // Cars + pedestrians: kinematic bodies that track their poses this step so the
    // player and the physics gun collide with them.
    syncKinematic(world, city_.carGroups().size(), city_.sim().poses().halfExtents(city_.sim()), carProxies_, dt, true);
    // the walkers' boxes where the walkers are this step (the sim's walker pose, city_poses.h; not the drawing's bake,
    // which skips a catch-up frame's first steps: ADR-0454) -- where the city owns its walkers (else they are physics
    // characters of their own, CityWalkerSystem)
    { std::vector<Vec3> pedExtents{ city_.pedHalfExtent() };
      walkerPoses_.clear();
      if (!city_.pedsExternallyOwned())
          city_.sim().poses().walkerBoxPoses(city_.sim(), pedExtents.front().y * 2, walkerPoses_);
      syncKinematic(world, 0, pedExtents, pedProxies_, dt, false, &walkerPoses_); }

    // Signal poles are static: one thin tall box per pole, built once (they never
    // move). The pole foot is the instance translation; the mast rises +Y.
    if (!polesBuilt_) {
        InstanceGroup* posts = world.get<InstanceGroup>(city_.signalPostGroup());
        if (posts) {
            PhysicsWorld& pw = physics_.physicsWorld();
            const Real poleH = 5.6;                       // = street_kit SignalParams.poleHeight
            const Vec3 he(0.16, poleH * 0.5, 0.16);
            for (const Mat4& m : posts->transforms) {
                Vec3 foot(m.m[0][3], m.m[1][3], m.m[2][3]);
                Vec3 c = foot + Vec3(0, poleH * 0.5, 0);
                poleBodies_.push_back(pw.addBox(he, c, Quat(), BodyMotion::Static));
            }
            polesBuilt_ = true;
        }
    }
}

void CityPhysicsSystem::onStop(engine::FrameContext& ctx) {
    releaseBodies();
    (void)ctx;
}

}  // namespace citysim
