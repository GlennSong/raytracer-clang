#include "test_framework.h"

#include "../src/engine/scripting/script_system.h"
#include "../src/engine/scripting/script_behaviour.h"
#include "../src/engine/world.h"
#include "../src/engine/components.h"
#include "../src/engine/input/input_map.h"
#include "../src/renderer/renderer.h"   // CameraState
#include "../src/renderer/event.h"
#include "../src/rt_math.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace engine;  // namespace migration (ADR-0015)

namespace {

// Read the actual shipped gun script, so this test exercises what runs in-game.
std::string gunSource() {
    std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/gun.lua");
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// A player entity: Transform + RigidBody + ControlledBy (how PlayerSystem finds
// it) + the gun ScriptBehaviour.
Entity makePlayer(World& w, const std::string& script) {
    Entity e = w.create();
    w.add<Transform>(e, Transform());
    RigidBody rb;
    w.add<RigidBody>(e, rb);
    w.add<ControlledBy>(e, ControlledBy{0});
    ScriptBehaviour sb;
    sb.source = script;
    w.add<ScriptBehaviour>(e, sb);
    return e;
}

// A camera looking down +Z from the origin, so aim forward = (0,0,1).
CameraState forwardCamera() {
    CameraState c;
    c.position = Vec3(0, 1, 0);
    c.target = Vec3(0, 1, 1);
    c.up = Vec3(0, 1, 0);
    return c;
}

// The first entity that has a Velocity but is not the player — i.e. a spawned
// bullet (the player has none).
Entity findBullet(World& w, Entity player) {
    Entity found;
    w.each<Velocity, RigidBody>([&](Entity e, Velocity&, RigidBody&) {
        if (e != player) found = e;
    });
    return found;
}

}  // namespace

TEST_CASE(gun_does_not_fire_without_input) {
    World world;
    Entity player = makePlayer(world, gunSource());
    InputMap input;
    input.bindButton("fire", MouseButton::Left);
    CameraState cam = forwardCamera();

    ScriptSystem sys;
    sys.setServices(&input, &cam, /*assets=*/nullptr);

    input.beginFrame();          // no fire event this frame
    sys.tick(world, 0.016);

    CHECK(!findBullet(world, player).valid());   // nothing spawned
}

TEST_CASE(gun_fires_a_physics_block_along_camera_aim) {
    World world;
    Entity player = makePlayer(world, gunSource());
    InputMap input;
    input.bindButton("fire", MouseButton::Left);
    CameraState cam = forwardCamera();   // forward = +Z

    ScriptSystem sys;
    sys.setServices(&input, &cam, nullptr);

    // Press fire this frame, then tick.
    input.beginFrame();
    Event click(EventType::MouseButtonPressed);
    click.button = MouseButton::Left;
    input.processEvent(click);
    CHECK(input.pressed("fire"));

    sys.tick(world, 0.016);

    Entity bullet = findBullet(world, player);
    if (!bullet.valid()) { CHECK(false); return; }

    // It's a dynamic box launched forward at the gun's 50 u/s.
    RigidBody* rb = world.get<RigidBody>(bullet);
    Velocity* vel = world.get<Velocity>(bullet);
    Collider* col = world.get<Collider>(bullet);
    if (!rb || !vel || !col) { CHECK(false); return; }
    CHECK(rb->motion == BodyMotion::Dynamic);
    CHECK(col->shape == ColliderShape::Box);
    CHECK_APPROX(vel->linear.z, 50.0, 1e-6);     // +Z aim * speed
    CHECK_APPROX(vel->linear.x, 0.0, 1e-6);
    CHECK_APPROX(vel->linear.y, 0.0, 1e-6);
    // Spawned in front of the eye (z ~ 0.5 ahead), not at the origin.
    Transform* t = world.get<Transform>(bullet);
    if (t) CHECK(t->position.z > 0.4);
}

TEST_CASE(gun_fires_once_per_press_not_while_held) {
    World world;
    Entity player = makePlayer(world, gunSource());
    InputMap input;
    input.bindButton("fire", MouseButton::Left);
    CameraState cam = forwardCamera();

    ScriptSystem sys;
    sys.setServices(&input, &cam, nullptr);

    auto countBullets = [&]() {
        int n = 0;
        world.each<Velocity, RigidBody>([&](Entity e, Velocity&, RigidBody&) {
            if (e != player) ++n;
        });
        return n;
    };

    // Frame 1: press -> one shot.
    input.beginFrame();
    Event click(EventType::MouseButtonPressed);
    click.button = MouseButton::Left;
    input.processEvent(click);
    sys.tick(world, 0.016);
    CHECK(countBullets() == 1);

    // Frame 2: button still held (no new press event) -> no new shot.
    input.beginFrame();
    sys.tick(world, 0.016);
    CHECK(countBullets() == 1);

    // Frame 3: release then press again -> a second shot.
    input.beginFrame();
    Event release(EventType::MouseButtonReleased);
    release.button = MouseButton::Left;
    input.processEvent(release);
    sys.tick(world, 0.016);
    input.beginFrame();
    input.processEvent(click);
    sys.tick(world, 0.016);
    CHECK(countBullets() == 2);
}
