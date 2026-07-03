#ifndef RAYTRACER_APPS_CITYSIM_CITY_PHYSICS_H
#define RAYTRACER_APPS_CITYSIM_CITY_PHYSICS_H

#include "../../engine/system.h"
#include "../../engine/systems/physics_system.h"   // PhysicsSystem, PhysicsBodyId
#include "city_render.h"

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

private:
    void releaseBodies();
    // Keep a kinematic-box pool tracking the transforms of instance groups (cars or
    // pedestrians); rebuilds if the count changes. `groupExtents` gives the collider
    // half-extent for each group (so a van's box is bigger than a sedan's); it must
    // be the same length as `groups`.
    void syncKinematic(engine::World& world, const std::vector<engine::Entity>& groups,
                       const std::vector<engine::Vec3>& groupExtents,
                       std::vector<engine::PhysicsBodyId>& pool, engine::Real dt);

    CityRenderSystem& city_;
    engine::PhysicsSystem& physics_;
    std::vector<engine::PhysicsBodyId> carBodies_;    // kinematic, one per car
    std::vector<engine::PhysicsBodyId> pedBodies_;    // kinematic, one per pedestrian
    std::vector<engine::PhysicsBodyId> poleBodies_;   // static, one per signal pole
    bool polesBuilt_ = false;
};

}  // namespace citysim

#endif
