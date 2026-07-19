// THE DRIVING LAB (roads-v2.1 R5, plan §3.2 gate): one physical car, one
// lane, one stop bar, one arc — the Jolt wheeled vehicle driven by the SAME
// plan the kinematic brains produce (a lane frame + a target speed), through
// the pure Stanley/pedal driver. Gates, in Glenn's terms: the car does not
// flip, stops at the line, and drives the arc.
#include "test_framework.h"

#include "../src/apps/citysim/driver_control.h"
#include "../src/engine/physics/physics_world.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace engine;

namespace {

PhysicsWorld::VehicleConfig sedan() {
    PhysicsWorld::VehicleConfig c;
    c.chassisHalfExtent = Vec3(0.92, 0.45, 2.15);
    c.mass = 1400.0;
    c.comOffsetY = -0.42;
    c.engineTorque = 900.0;   // brisk sedan: ~3.5 m/s^2 launch (envelope probe)
    auto wheel = [&](Real x, Real z, bool steered, bool driven) {
        PhysicsWorld::VehicleWheel w;
        w.position = Vec3(x, -0.35, z);
        w.radius = 0.34;
        w.width = 0.24;
        w.steered = steered;
        w.driven = driven;
        return w;
    };
    // ALL wheels driven: the launch is TRACTION-limited (measured — rear
    // drive capped every config at ~2.1 m/s^2 regardless of torque; AWD
    // doubled it). ~4 m/s^2 matches the brains' kCarAccel, which is the
    // tier-invariance contract.
    c.wheels = { wheel(-0.82, 1.45, true, true), wheel(0.82, 1.45, true, true),
                 wheel(-0.82, -1.42, false, true), wheel(0.82, -1.42, false, true) };
    return c;
}

// Forward direction in the plane from the chassis orientation (chassis
// forward is +z local).
Vec2 forward2(const Quat& q) {
    const Vec3 f = q.rotate(Vec3(0, 0, 1));
    const Real l = std::sqrt(f.x * f.x + f.z * f.z);
    return l > 1e-6 ? Vec2(f.x / l, f.z / l) : Vec2(0, 1);
}

Real upDot(const Quat& q) { return q.rotate(Vec3(0, 1, 0)).y; }

}  // namespace

// Powertrain envelope: full throttle from rest on the flat. Pins what the
// sedan can actually DO — the possession tier's catch-up authority budget
// (a ghost cruises at 8.5; the body must hold a real speed margin over it).
TEST_CASE(driving_lab_full_throttle_envelope) {
    PhysicsWorld world;
    world.initialize();
    world.addBox(Vec3(500, 1, 500), Vec3(0, -1, 0), Quat::identity(),
                 BodyMotion::Static);
    world.optimizeBroadPhase();
    PhysicsWorld::VehicleId car =
        world.addVehicle(sedan(), Vec3(0, 1.0, 0), Quat::identity());
    Real v3 = 0, v8 = 0;
    for (int i = 0; i < 60 * 8; ++i) {
        world.setVehicleInput(car, 1.0, 0.0, 0.0, 0.0);
        world.update(1.0 / 60.0);
        const Vec3 v = world.vehicleVelocity(car);
        const Real s = std::sqrt(v.x * v.x + v.z * v.z);
        if (i == 60 * 3 - 1) v3 = s;
        if (i == 60 * 8 - 1) v8 = s;
        if (i % 60 == 59)
            std::printf("    [envelope] t=%ds v=%.2f m/s\n", (i + 1) / 60, s);
    }
    CHECK(v3 > 10.0);   // the catch-up margin the tier plans with exists
    CHECK(v8 > 20.0);
}

TEST_CASE(driving_lab_straight_stop_at_the_bar) {
    PhysicsWorld world;
    world.initialize();
    world.addBox(Vec3(200, 1, 200), Vec3(0, -1, 0), Quat::identity(),
                 BodyMotion::Static);
    world.optimizeBroadPhase();
    PhysicsWorld::VehicleId car =
        world.addVehicle(sedan(), Vec3(0, 1.0, 0), Quat::identity());
    CHECK(car != PhysicsWorld::INVALID_VEHICLE);

    const Vec2 laneA(0, -10), laneB(0, 300);   // straight lane along +z
    const Real barZ = 90.0;
    Real minUp = 1.0, worstCross = 0.0;
    for (int i = 0; i < 60 * 30; ++i) {
        const Vec3 p = world.vehiclePosition(car);
        const Quat q = world.vehicleOrientation(car);
        const Vec3 v = world.vehicleVelocity(car);
        const Vec2 fwd = forward2(q);
        const Real speed = v.x * fwd.x + v.z * fwd.y;
        // The PLAN (what IDM+signals produce): cruise 9, ease to a stop at
        // the bar with comfortable decel.
        const Real dist = barZ - p.z;
        Real target = 9.0;
        if (dist < 45.0)
            target = std::min(target, dist > 1.0
                                          ? std::sqrt(std::max(0.0, 2.0 * 2.6 * (dist - 1.0)))
                                          : 0.0);
        const citysim::DriverInput in = citysim::driveTowards(
            Vec2(p.x, p.z), fwd, speed, target, laneA, laneB);
        world.setVehicleInput(car, in.throttle, in.steer, in.brake);
        world.update(1.0 / 60.0);
        minUp = std::min(minUp, upDot(q));
        if (p.z > 5.0) worstCross = std::max(worstCross, std::fabs(p.x));
    }
    const Vec3 pEnd = world.vehiclePosition(car);
    const Vec3 vEnd = world.vehicleVelocity(car);
    std::printf("[lab] straight: end z=%.1f x=%.2f speed=%.2f minUp=%.2f "
                "worstCross=%.2f\n",
                pEnd.z, pEnd.x, std::sqrt(vEnd.x * vEnd.x + vEnd.z * vEnd.z),
                minUp, worstCross);
    CHECK(minUp > 0.86);                          // never > ~30 deg of roll/pitch
    CHECK(std::fabs(pEnd.z - barZ) < 6.0);        // stopped AT the bar
    CHECK(std::sqrt(vEnd.x * vEnd.x + vEnd.z * vEnd.z) < 0.5);
    CHECK(worstCross < 0.8);                      // held the lane line
    world.shutdown();
}

TEST_CASE(driving_lab_arc_without_flipping) {
    PhysicsWorld world;
    world.initialize();
    world.addBox(Vec3(200, 1, 200), Vec3(0, -1, 0), Quat::identity(),
                 BodyMotion::Static);
    world.optimizeBroadPhase();
    PhysicsWorld::VehicleId car =
        world.addVehicle(sedan(), Vec3(0, 1.0, 0), Quat::identity());

    // A 90-degree junction arc, radius 16 (a real left turn through a pad):
    // straight in, arc, straight out.
    std::vector<Vec2> path;
    for (double z = 0; z < 30.0; z += 2.0) path.push_back(Vec2(0, z));
    const Vec2 c(-16.0, 30.0);
    for (double a = 0; a <= 1.5707963; a += 0.08)
        path.push_back(Vec2(c.x + 16.0 * std::cos(a), c.y + 16.0 * std::sin(a)));
    for (double x = -16.0; x > -60.0; x -= 2.0) path.push_back(Vec2(x, 46.0));

    Real minUp = 1.0, worstCross = 0.0;
    std::size_t nearest = 0;
    for (int i = 0; i < 60 * 40; ++i) {
        const Vec3 p = world.vehiclePosition(car);
        const Quat q = world.vehicleOrientation(car);
        const Vec3 v = world.vehicleVelocity(car);
        const Vec2 fwd = forward2(q);
        const Real speed = v.x * fwd.x + v.z * fwd.y;
        // Project onto the path (monotone nearest) and ride the local frame
        // ~4 m ahead — the pursuit the sim's lane frames provide.
        const Vec2 pos(p.x, p.z);
        while (nearest + 1 < path.size() &&
               (path[nearest + 1] - pos).lengthSquared() <
                   (path[nearest] - pos).lengthSquared())
            ++nearest;
        const std::size_t ahead = std::min(nearest + 2, path.size() - 1);
        if (ahead > nearest)
            worstCross = std::max(worstCross,
                                  (pos - path[nearest]).length() > 6.0
                                      ? (pos - path[nearest]).length()
                                      : worstCross);
        // Curvature-aware plan speed: slow for the arc (the lookahead rule).
        const Real target = nearest < 12 || nearest > path.size() - 18 ? 8.0 : 5.5;
        const citysim::DriverInput in = citysim::driveTowards(
            pos, fwd, speed, target, path[nearest], path[ahead]);
        world.setVehicleInput(car, in.throttle, in.steer, in.brake);
        world.update(1.0 / 60.0);
        minUp = std::min(minUp, upDot(q));
        if (nearest + 2 >= path.size()) break;
    }
    const Vec3 pEnd = world.vehiclePosition(car);
    std::printf("[lab] arc: end (%.1f, %.1f) nearest=%zu/%zu minUp=%.2f\n",
                pEnd.x, pEnd.z, nearest, path.size(), minUp);
    CHECK(minUp > 0.86);                          // the no-flip gate
    CHECK(nearest + 4 >= path.size());            // completed the turn
    CHECK(pEnd.x < -40.0);                        // really came out the far leg
    world.shutdown();
}

TEST_CASE(driving_lab_steer_sign_probe) {
    // Ground truth for the steering convention: constant +steer, gentle
    // throttle, from rest — which way does the nose go?
    PhysicsWorld world;
    world.initialize();
    world.addBox(Vec3(200, 1, 200), Vec3(0, -1, 0), Quat::identity(),
                 BodyMotion::Static);
    world.optimizeBroadPhase();
    PhysicsWorld::VehicleId car =
        world.addVehicle(sedan(), Vec3(0, 1.0, 0), Quat::identity());
    for (int i = 0; i < 60 * 4; ++i) {
        world.setVehicleInput(car, 0.4, 0.5, 0.0);
        world.update(1.0 / 60.0);
    }
    const Vec3 p = world.vehiclePosition(car);
    const Vec2 f = forward2(world.vehicleOrientation(car));
    std::printf("[lab] +steer probe: pos (%.1f, %.1f) fwd (%.2f, %.2f)\n",
                p.x, p.z, f.x, f.y);
    CHECK(p.z > 2.0);   // it drove somewhere
    world.shutdown();
}
