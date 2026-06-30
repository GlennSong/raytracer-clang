#include "city_physics.h"

#include "../../engine/components.h"   // InstanceGroup
#include "../../engine/world.h"

#include <cmath>

namespace citysim {

using engine::Vec3;
using engine::Quat;
using engine::Mat4;
using engine::World;
using engine::InstanceGroup;
using engine::PhysicsWorld;
using engine::PhysicsBodyId;
using engine::BodyMotion;

void CityPhysicsSystem::releaseBodies() {
    PhysicsWorld& pw = physics_.physicsWorld();
    for (PhysicsBodyId id : bodies_) pw.removeBody(id);
    bodies_.clear();
}

void CityPhysicsSystem::fixedUpdate(engine::FrameContext& ctx) {
    if (!city_.built()) return;
    World& world = ctx.world;
    InstanceGroup* cars = world.get<InstanceGroup>(city_.carGroup());
    if (!cars) return;

    PhysicsWorld& pw = physics_.physicsWorld();
    const std::size_t n = cars->transforms.size();

    // Match the body pool to the car count (cars are a fixed set, so this runs
    // once; rebuild defensively if the count ever changes, e.g. a level reload).
    if (bodies_.size() != n) {
        releaseBodies();
        Vec3 he = city_.carHalfExtent();
        bodies_.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            const Mat4& m = cars->transforms[i];
            Vec3 p(m.m[0][3], m.m[1][3], m.m[2][3]);
            bodies_.push_back(pw.addBox(he, p, Quat(), BodyMotion::Kinematic,
                                        /*restitution*/ 0.0, /*friction*/ 0.6));
        }
    }

    // Drive each collider to its car's drawn pose. Yaw is read from the car
    // transform's local +Z basis column (the heading), so the box stays aligned to
    // travel regardless of the matrix's internal convention.
    Real dt = ctx.clock.fixedStep();
    for (std::size_t i = 0; i < n && i < bodies_.size(); ++i) {
        const Mat4& m = cars->transforms[i];
        Vec3 p(m.m[0][3], m.m[1][3], m.m[2][3]);
        Real yaw = std::atan2(m.m[0][2], m.m[2][2]);
        Quat rot = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
        pw.moveKinematic(bodies_[i], p, rot, dt);
    }
}

void CityPhysicsSystem::onStop(engine::FrameContext&) { releaseBodies(); }

}  // namespace citysim
