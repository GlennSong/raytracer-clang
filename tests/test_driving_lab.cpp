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

// The COMMANDEERED car's collision proportions (city_vehicles configFromBody +
// the fleet's wheel_layout): the chassis is the FULL body box, and the fleet
// parks each wheel's resting centre ONE RADIUS above that box's floor — so the
// collision floor rested exactly ON the road. Zero clearance is the kerb bug's
// mechanism: the bumper's collision corner reaches the 0.15 m kerb face ~0.9 m
// before the front wheels do, and the car stops dead. The lab's tuned `sedan()`
// above has an artificially high floor and climbs kerbs under any tester, which
// is exactly how the bug shipped unseen.
PhysicsWorld::VehicleConfig bodyBoxSedan() {
    PhysicsWorld::VehicleConfig c;
    c.chassisHalfExtent = Vec3(0.91, 0.725, 2.295);   // 1.82 x 1.45 x 4.59 m
    c.mass = 900.0 + 4.59 * 1.82 * 180.0;             // configFromBody's rule
    c.comOffsetY = -0.45;
    c.engineTorque = 650.0;
    c.brakeTorque = 1600.0;
    c.maxSteerDegrees = 32.0;
    // configFromBody's short-travel arcade suspension, attach lifted so the
    // wheel rests where the fleet drew it: one radius above the box floor.
    auto wheel = [&](Real x, Real z, bool steered) {
        PhysicsWorld::VehicleWheel w;
        const Real restCentreY = -0.725 + 0.31;
        w.position = Vec3(x, restCentreY + (0.15 - 0.075), z);
        w.suspensionMin = 0.05;
        w.suspensionMax = 0.15;
        w.radius = 0.31;          // 0.62 m wheel diameter
        w.width = 0.24;
        w.steered = steered;
        w.handBrake = !steered;
        return w;
    };
    const Real frontZ = 2.295 - 0.90, rearZ = -(2.295 - 1.00);
    c.wheels = { wheel(0.78, frontZ, true), wheel(-0.78, frontZ, true),
                 wheel(0.78, rearZ, false), wheel(-0.78, rearZ, false) };
    return c;
}

// One kerb world: road at y=0, a sidewalk slab whose kerb face is at z=20 with
// its top at y=0.15 (the city's road_lattice kerb height). Returns final z.
Real driveAtKerb(const PhysicsWorld::VehicleConfig& cfg, Real* yOnRoad,
                 Real* yOnSlab, Real* minUp) {
    PhysicsWorld world;
    world.initialize();
    world.addBox(Vec3(200, 1, 200), Vec3(0, -1, 0), Quat::identity(),
                 BodyMotion::Static);
    world.addBox(Vec3(50, 0.075, 50), Vec3(0, 0.075, 70), Quat::identity(),
                 BodyMotion::Static);
    world.optimizeBroadPhase();
    PhysicsWorld::VehicleId car =
        world.addVehicle(cfg, Vec3(0, 1.0, 0), Quat::identity());
    if (car == PhysicsWorld::INVALID_VEHICLE) return -1.0;
    for (int i = 0; i < 60 * 10; ++i) {
        world.setVehicleInput(car, 0.5, 0.0, 0.0, 0.0);   // city pace, not a ram
        world.update(1.0 / 60.0);
        const Vec3 p = world.vehiclePosition(car);
        if (p.z > 5.0 && p.z < 15.0 && yOnRoad) *yOnRoad = p.y;
        if (p.z > 40.0 && p.z < 100.0 && yOnSlab) *yOnSlab = p.y;
        if (p.z > 5.0 && minUp)
            *minUp = std::min(*minUp, upDot(world.vehicleOrientation(car)));
    }
    const Real endZ = world.vehiclePosition(car).z;
    world.shutdown();
    return endZ;
}

// KERB CLIMB (car-controls round): drive the body-box sedan at a sidewalk kerb
// and get ON it. Two fixes make this pass: the collision box's underside is
// raised by VehicleConfig::floorClearance (so the nose no longer rams the
// face), and the cast-cylinder wheel tester sees the step edge a ray down from
// the hub could not. Gate in Glenn's terms: the car mounts the kerb, keeps
// driving on the sidewalk, and doesn't flip doing it.
TEST_CASE(driving_lab_climbs_a_kerb) {
    Real yOnRoad = 0, yOnSlab = 0, minUp = 1.0;
    const Real endZ = driveAtKerb(bodyBoxSedan(), &yOnRoad, &yOnSlab, &minUp);
    std::printf("[lab] kerb: end z=%.1f road y=%.3f slab y=%.3f minUp=%.2f\n",
                endZ, yOnRoad, yOnSlab, minUp);
    CHECK(endZ > 30.0);                   // past the face and still driving
    CHECK(yOnSlab - yOnRoad > 0.10);      // rode a kerb height higher up there
    CHECK(minUp > 0.86);                  // mounted it, didn't launch or roll
}

// The CONTROL for the gate above: the same sedan with the floor clearance
// carved to zero (the pre-fix collision box) must STALL at the kerb — its nose
// hits the face before the wheels reach it. If this ever starts passing, the
// kerb test above has gone vacuous (it no longer exercises the mechanism), the
// way a high-floored lab config once did.
TEST_CASE(driving_lab_kerb_stops_the_uncarved_box) {
    PhysicsWorld::VehicleConfig cfg = bodyBoxSedan();
    cfg.floorClearance = 0.0;
    const Real endZ = driveAtKerb(cfg, nullptr, nullptr, nullptr);
    std::printf("[lab] kerb control (no clearance): end z=%.1f\n", endZ);
    CHECK(endZ > 0.0);       // the drive ran at all
    CHECK(endZ < 30.0);      // ...and the kerb face stopped it, as it once did
}

// ...and a WALL is still a wall: the kerb fix must not let the car crawl up
// anything with a face taller than its wheels. A 1 m parapet stops the sedan —
// the raised collision floor (~0.3 m) is still well under the wall top.
TEST_CASE(driving_lab_wall_still_stops) {
    PhysicsWorld world;
    world.initialize();
    world.addBox(Vec3(200, 1, 200), Vec3(0, -1, 0), Quat::identity(),
                 BodyMotion::Static);
    const Real wallZ = 30.0;   // near face of a 1 m tall, 2 m thick wall
    world.addBox(Vec3(50, 0.5, 1), Vec3(0, 0.5, wallZ + 1.0), Quat::identity(),
                 BodyMotion::Static);
    world.optimizeBroadPhase();
    PhysicsWorld::VehicleId car =
        world.addVehicle(bodyBoxSedan(), Vec3(0, 1.0, 0), Quat::identity());
    Real yOnRoad = 0;   // settled ride height, for a relative climb check
    for (int i = 0; i < 60 * 10; ++i) {
        world.setVehicleInput(car, 0.5, 0.0, 0.0, 0.0);
        world.update(1.0 / 60.0);
        const Vec3 p = world.vehiclePosition(car);
        if (p.z > 5.0 && p.z < 15.0) yOnRoad = p.y;
    }
    const Vec3 pEnd = world.vehiclePosition(car);
    std::printf("[lab] wall: end z=%.1f y=%.3f (road y=%.3f)\n", pEnd.z, pEnd.y,
                yOnRoad);
    CHECK(pEnd.z < wallZ);                // stopped at the wall, not on it
    CHECK(pEnd.y - yOnRoad < 0.3);        // and didn't ride up its face
    world.shutdown();
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

// RIDE HEIGHT. `VehicleWheel::position` is the suspension ATTACHMENT point, not
// the wheel's resting centre: Jolt hangs the wheel below it by however far the
// spring settles under load. Anything that publishes a DRAWN wheel centre as the
// attach point therefore parks the whole chassis that much too high — visibly
// jacked up, and top-heavy because the real CoG rises with it.
//
// This probe measures the settle distance for the shipped suspension defaults so
// the number is a fact rather than an estimate.
TEST_CASE(vehicle_suspension_settle_offset_is_measured) {
    PhysicsWorld world;
    world.initialize();
    world.addBox(Vec3(200, 1, 200), Vec3(0, -1, 0), Quat::identity(),
                 BodyMotion::Static);
    world.optimizeBroadPhase();

    // A car whose attach point IS the drawn resting centre: half-height 0.65,
    // wheel radius 0.31, so a wheel resting on the road has its centre at
    // -0.65 + 0.31 = -0.34 in chassis space.
    const Real halfH = 0.65, r = 0.31;
    const Real attachY = -halfH + r;
    PhysicsWorld::VehicleConfig c;
    c.chassisHalfExtent = Vec3(0.90, halfH, 2.10);
    c.mass = 1400.0;
    c.comOffsetY = -0.40;
    auto wheel = [&](Real x, Real z, bool front) {
        PhysicsWorld::VehicleWheel w;
        w.position = Vec3(x, attachY, z);
        w.radius = r;
        w.width = 0.22;
        w.steered = front;
        w.driven = true;
        w.handBrake = !front;
        return w;
    };
    c.wheels = { wheel(0.67, 1.28, true), wheel(-0.67, 1.28, true),
                 wheel(0.67, -1.28, false), wheel(-0.67, -1.28, false) };

    // Settle it with the STOCK travel first: the wheel comes to rest one
    // suspension-drop below its attachment point.
    PhysicsWorld::VehicleId car =
        world.addVehicle(c, Vec3(0, 2.0, 0), Quat::identity());
    CHECK(car != PhysicsWorld::INVALID_VEHICLE);
    for (int i = 0; i < 60 * 5; ++i) {
        world.setVehicleInput(car, 0.0, 0.0, 1.0, 1.0);   // brakes on, let it settle
        world.update(1.0 / 60.0);
    }
    const Real restY = world.vehiclePosition(car).y;
    const Real intendedY = r - attachY;   // wheel centre AT the attach point
    const Real settle = restY - intendedY;
    std::printf("    [ride height] stock travel: rests %.3f, drawn %.3f "
                "-> %.3f m too high\n", restY, intendedY, settle);
    // The stock 0.20/0.45 travel is what jacked commandeered cars into the air.
    CHECK(settle > 0.30);

    // THE RULE, and it is what makes the bug correctable: the resting drop is
    // clamp(max - kStaticDeflection, min, max), INDEPENDENT of mass — Jolt's
    // spring is frequency-based, so it stiffens with the load it carries. A
    // 1100 kg hatchback and a 2600 kg truck settle to the same ride height.
    const Real kStaticDeflection = 0.075;   // at the 1.5 Hz spring default
    for (double mn : {0.05, 0.10}) {
        for (double mx : {0.15, 0.25}) {
            for (double ms : {1100.0, 2600.0}) {
                PhysicsWorld w2;
                w2.initialize();
                w2.addBox(Vec3(200, 1, 200), Vec3(0, -1, 0), Quat::identity(),
                          BodyMotion::Static);
                w2.optimizeBroadPhase();
                PhysicsWorld::VehicleConfig c2 = c;
                c2.mass = ms;
                for (PhysicsWorld::VehicleWheel& ww : c2.wheels) {
                    ww.suspensionMin = mn;
                    ww.suspensionMax = mx;
                }
                PhysicsWorld::VehicleId id2 =
                    w2.addVehicle(c2, Vec3(0, 2.0, 0), Quat::identity());
                for (int i = 0; i < 60 * 5; ++i) {
                    w2.setVehicleInput(id2, 0.0, 0.0, 1.0, 1.0);
                    w2.update(1.0 / 60.0);
                }
                const Real got = w2.vehiclePosition(id2).y - intendedY;
                const Real want = std::min(std::max(mx - kStaticDeflection, mn), mx);
                CHECK(std::fabs(got - want) < 0.01);
            }
        }
    }

    // And the CORRECTION: lift the attachment point by that drop and the car
    // rests exactly where it is drawn. This is what citysim's configFromBody
    // does for a commandeered car.
    {
        const Real mn = 0.05, mx = 0.15, drop = mx - kStaticDeflection;
        PhysicsWorld w3;
        w3.initialize();
        w3.addBox(Vec3(200, 1, 200), Vec3(0, -1, 0), Quat::identity(),
                  BodyMotion::Static);
        w3.optimizeBroadPhase();
        PhysicsWorld::VehicleConfig c3 = c;
        for (PhysicsWorld::VehicleWheel& ww : c3.wheels) {
            ww.position = ww.position + Vec3(0, drop, 0);
            ww.suspensionMin = mn;
            ww.suspensionMax = mx;
        }
        PhysicsWorld::VehicleId id3 =
            w3.addVehicle(c3, Vec3(0, 2.0, 0), Quat::identity());
        for (int i = 0; i < 60 * 5; ++i) {
            w3.setVehicleInput(id3, 0.0, 0.0, 1.0, 1.0);
            w3.update(1.0 / 60.0);
        }
        const Real corrected = w3.vehiclePosition(id3).y;
        std::printf("    [ride height] corrected: rests %.3f, drawn %.3f\n",
                    corrected, intendedY);
        CHECK(std::fabs(corrected - intendedY) < 0.02);
    }
}
