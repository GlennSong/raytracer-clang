// A stamped prefab's moving node carries its collision shape (#191, step 8): the crane's boom, swung by its cab,
// sweeps a box off a post; and a script moves a copy's nodes.
#include "test_framework.h"

#include "../src/engine/model_motion.h"
#include "../src/engine/prefab_stamp.h"
#include "../src/engine/scripting/prefab_list.h"
#include "../src/engine/systems/model_motion_system.h"
#include "../src/engine/systems/physics_system.h"
#include "../src/engine/components.h"
#include "../src/engine/world.h"

using namespace engine;

namespace {

Entity addBox(World& world, const Vec3& position, const Vec3& half, BodyMotion motion) {
    Entity e = world.create();
    Transform t;
    t.position = position;
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});
    Collider c;
    c.shape = ColliderShape::Box;
    c.halfExtent = half;
    c.friction = 0.8;
    world.add<Collider>(e, c);
    world.add<RigidBody>(e, RigidBody{motion, INVALID_PHYSICS_BODY});
    return e;
}

}  // namespace

TEST_CASE(a_cranes_boom_swung_by_its_cab_sweeps_a_box_off_a_post) {
    World world;
    ModelStore store;
    addBox(world, Vec3(0, -1, 0), Vec3(80, 1, 80), BodyMotion::Static);          // the floor
    Prefab crane;
    CHECK(findPrefab("crane", crane));
    const Entity e = stampPrefab(world, nullptr, crane, Transform(), std::string(), nullptr, &store);
    CHECK(e.valid());
    if (!e.valid()) return;
    // A post beside the crane, its top just under the boom's height (the boom's underside is 5.25 m up), 4 m from
    // the mast along x: where the boom's far half passes when the cab has turned a quarter. A box stands on it,
    // reaching up into the boom's path.
    addBox(world, Vec3(4, 2.5, 0), Vec3(0.3, 2.5, 0.3), BodyMotion::Static);
    const Entity box = addBox(world, Vec3(4, 5.3, 0), Vec3(0.25, 0.3, 0.25), BodyMotion::Dynamic);

    PhysicsSystem physics;
    physics.initialize();
    physics.createBodies(world);
    ModelMotionSystem motion(&physics);
    for (int i = 0; i < 120; i++) physics.step(world, 1.0 / 60.0);
    CHECK_APPROX(world.get<Transform>(box)->position.y, 5.3, 0.05);               // resting on the post
    CHECK_APPROX(world.get<Transform>(box)->position.x, 4.0, 0.05);

    // The boom starts along +z. Turn the cab a quarter (the rig turns it about +y: +z goes to +x), 30 degrees a second.
    const Real deg = 3.14159265358979 / 180.0;
    CHECK(setModelTarget(world, e, "cabin", 90 * deg, 30 * deg));
    int movedSteps = 0;
    for (int i = 0; i < 360; i++) {
        movedSteps += motion.step(world, 1.0 / 60.0);
        physics.step(world, 1.0 / 60.0);
    }
    CHECK(movedSteps >= 180 && movedSteps <= 181);
    // The boom's body went with the boom: it is where the pose says, along +x now.
    bool boomThere = false;
    for (Entity be : world.get<ModelInstance>(e)->bodies) {
        const RigidBody* rb = world.get<RigidBody>(be);
        if (rb->motion != BodyMotion::Kinematic) continue;
        CHECK(rb->bodyId != INVALID_PHYSICS_BODY);
        Vec3 want;
        Quat turn;
        CHECK(modelBodyPlace(world, be, want, turn));
        const Vec3 is = physics.physicsWorld().bodyPosition(rb->bodyId);
        CHECK((is - want).length() < 0.02);
        if ((want - Vec3(3.1, 5.4, 0)).length() < 0.02) boomThere = true;
    }
    CHECK(boomThere);
    // ...and it knocked the box off the post: the box is on the floor, pushed the way the boom was going (toward -z
    // is behind the sweep; the boom came from +z, so the box went to -z).
    const Vec3 fell = world.get<Transform>(box)->position;
    std::printf("    the box ended at %.2f %.2f %.2f\n", fell.x, fell.y, fell.z);
    CHECK(fell.y < 1.0);
    CHECK(fell.z < -0.2);
    physics.shutdown();
}
