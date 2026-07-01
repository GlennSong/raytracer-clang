#include "city_physics.h"

#include "../../engine/components.h"   // InstanceGroup
#include "../../engine/world.h"

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
    for (PhysicsBodyId id : carBodies_) pw.removeBody(id);
    for (PhysicsBodyId id : pedBodies_) pw.removeBody(id);
    for (PhysicsBodyId id : poleBodies_) pw.removeBody(id);
    carBodies_.clear();
    pedBodies_.clear();
    poleBodies_.clear();
    polesBuilt_ = false;
}

// Keep `pool` matched to every transform across `groups`, driving each kinematic
// body to its instance's drawn pose. Group + agent order are stable, so pool
// index i tracks the same instance across frames.
void CityPhysicsSystem::syncKinematic(World& world, const std::vector<Entity>& groups,
                                      const Vec3& halfExtent,
                                      std::vector<PhysicsBodyId>& pool, Real dt) {
    std::vector<const Mat4*> poses;
    for (Entity e : groups) {
        InstanceGroup* g = world.get<InstanceGroup>(e);
        if (!g) continue;
        for (const Mat4& m : g->transforms) poses.push_back(&m);
    }
    PhysicsWorld& pw = physics_.physicsWorld();
    if (pool.size() != poses.size()) {
        for (PhysicsBodyId id : pool) pw.removeBody(id);
        pool.clear();
        pool.reserve(poses.size());
        for (const Mat4* m : poses) {
            Vec3 p(m->m[0][3], m->m[1][3], m->m[2][3]);
            pool.push_back(pw.addBox(halfExtent, p, Quat(), BodyMotion::Kinematic,
                                     /*restitution*/ 0.0, /*friction*/ 0.6));
        }
    }
    for (std::size_t i = 0; i < poses.size() && i < pool.size(); ++i) {
        const Mat4& m = *poses[i];
        Vec3 p(m.m[0][3], m.m[1][3], m.m[2][3]);
        Real yaw = std::atan2(m.m[0][2], m.m[2][2]);   // heading from local +Z basis
        pw.moveKinematic(pool[i], p, Quat::fromAxisAngle(Vec3(0, 1, 0), yaw), dt);
    }
}

void CityPhysicsSystem::fixedUpdate(engine::FrameContext& ctx) {
    if (!city_.built()) return;
    World& world = ctx.world;
    Real dt = ctx.clock.fixedStep();

    // Cars + pedestrians: kinematic bodies that track the drawn poses so the
    // player and the physics gun collide with them.
    syncKinematic(world, city_.carGroups(), city_.carHalfExtent(), carBodies_, dt);
    { std::vector<Entity> peds{ city_.pedGroup() };
      syncKinematic(world, peds, city_.pedHalfExtent(), pedBodies_, dt); }

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

void CityPhysicsSystem::onStop(engine::FrameContext&) { releaseBodies(); }

}  // namespace citysim
