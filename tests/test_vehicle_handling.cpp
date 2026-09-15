// VEHICLE HANDLING (Glenn, 2026-09-14: "the car fishtails and rolls easily").
// Three headless manoeuvres on the PLAYER'S sedan — the numbers the Lua spec
// (vehicles.lua from_class "sedan") hands PhysicsWorld::addVehicle, copied
// here because physics_tests has no Lua — measured, then gated:
//
//   step steer   at 90 km/h, keyboard full lock for 0.6 s then release: the
//                car must settle back to straight (no spin), body roll small.
//   tank slapper at 70 km/h, alternating full lock every 0.7 s: the sideslip
//                must stay bounded (no growing oscillation, no spin).
//   kerb trip    sliding sideways at 8 m/s into a 0.15 m kerb: stays upright.
#include "test_framework.h"

#include "../src/engine/physics/physics_world.h"
#include "../src/engine/vehicle_steering.h"
#include "../src/engine/procgen/city/polygon.h"   // Vec2
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace engine;

namespace {

constexpr Real kPi = 3.14159265358979323846;

// vehicles.lua, class sedan: 4.60 x 1.82 x 1.45 m, wheel 0.62 m, track 1.56,
// wheelbase 4.35 wd, overhangs 1.45 / 1.61 wd, mass = volume * 120,
// com = -(0.23 + 0.22 * 1.45/1.45), torque = mass * 0.46, coupe steer 32 deg,
// AWD with the rears on the handbrake; the host lifts each wheel by the
// spring's settle and uses the street travel.
struct Rig {
    Real min = PhysicsWorld::kStreetSuspensionMin, max = PhysicsWorld::kStreetSuspensionMax;
    Real freq = PhysicsWorld::kStreetSuspensionFrequency, damp = PhysicsWorld::kStreetSuspensionDamping;
    Real drop = PhysicsWorld::kStreetSuspensionRestDrop;
    Real cone = 65.0, arb = 0.0;
};

PhysicsWorld::VehicleConfig playerSedan(const Rig& rig = Rig{}) {
    PhysicsWorld::VehicleConfig c;
    c.maxPitchRollDegrees = rig.cone;
    c.antiRollStiffness = rig.arb;
    const Real L = 4.60, W = 1.82, H = 1.45, wd = 0.62;
    c.chassisHalfExtent = Vec3(W * 0.5, H * 0.5, L * 0.5);
    c.mass = std::floor(L * W * H * 120.0);
    c.comOffsetY = -0.45;
    c.engineTorque = std::floor(c.mass * 0.46);
    c.maxSteerDegrees = 32.0;
    c.brakeTorque = std::floor(c.mass * 1.15);
    c.handBrakeTorque = std::floor(c.mass * 2.9);
    const Real r = wd * 0.5, halfTrack = 0.78;
    const Real axleY = -H * 0.5 + r + rig.drop;
    const Real frontZ = L * 0.5 - 1.45 * wd, rearZ = -(L * 0.5 - 1.61 * wd);
    auto wheel = [&](Real x, Real z, bool steered, bool hand) {
        PhysicsWorld::VehicleWheel w;
        w.position = Vec3(x, axleY, z);
        w.radius = r;
        w.width = std::max(0.18, 1.56 * 0.13);
        w.suspensionMin = rig.min;
        w.suspensionMax = rig.max;
        w.suspensionFrequency = rig.freq;
        w.suspensionDamping = rig.damp;
        w.steered = steered;
        w.driven = true;
        w.handBrake = hand;
        return w;
    };
    c.wheels = { wheel(halfTrack, frontZ, true, false), wheel(-halfTrack, frontZ, true, false),
                 wheel(halfTrack, rearZ, false, true), wheel(-halfTrack, rearZ, false, true) };
    return c;
}

struct Probe {
    Real maxSlipDeg = 0;    // body sideslip: velocity vs forward, in the plane
    Real maxRollDeg = 0;    // lean of the right axis out of the plane
    Real minUp = 1.0;
    Real endSlipDeg = 0;
    Real endYawRate = 0;    // deg/s over the last 0.5 s
    Real endSpeed = 0;
};

Vec2 forward2(const Quat& q) {
    const Vec3 f = q.rotate(Vec3(0, 0, 1));
    const Real l = std::sqrt(f.x * f.x + f.z * f.z);
    return l > 1e-6 ? Vec2(f.x / l, f.z / l) : Vec2(0, 1);
}

Real headingDeg(const Quat& q) {
    const Vec2 f = forward2(q);
    return std::atan2(f.x, f.y) * 180.0 / kPi;
}

// One sample of the probe from the car's state.
void sample(Probe& p, PhysicsWorld& w, PhysicsWorld::VehicleId car) {
    const Quat q = w.vehicleOrientation(car);
    const Vec3 v = w.vehicleVelocity(car);
    const Vec2 f = forward2(q);
    const Real speed = std::sqrt(v.x * v.x + v.z * v.z);
    if (speed > 2.0) {
        const Real along = v.x * f.x + v.z * f.y;
        const Real across = v.x * f.y - v.z * f.x;
        const Real slip = std::fabs(std::atan2(across, along)) * 180.0 / kPi;
        p.maxSlipDeg = std::max(p.maxSlipDeg, slip);
        p.endSlipDeg = slip;
    }
    const Vec3 right = q.rotate(Vec3(1, 0, 0));
    const Real roll = std::fabs(std::asin(std::clamp(right.y, Real(-1), Real(1)))) * 180.0 / kPi;
    p.maxRollDeg = std::max(p.maxRollDeg, roll);
    p.minUp = std::min(p.minUp, q.rotate(Vec3(0, 1, 0)).y);
    p.endSpeed = speed;
}

PhysicsWorld::VehicleId launch(PhysicsWorld& world, const PhysicsWorld::VehicleConfig& cfg,
                               Real speed) {
    world.addBox(Vec3(3000, 1, 3000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static,
                 0.0, 0.85);   // the city's road friction (level_loader mesh colliders)
    world.optimizeBroadPhase();
    const PhysicsWorld::VehicleId car =
        world.addVehicle(cfg, Vec3(0, 0.9, 0), Quat::identity());
    for (int i = 0; i < 60; ++i) world.update(1.0 / 60.0);   // settle on the springs
    // Under its own power (a set velocity leaves the wheels, clutch and
    // gearbox behind — the engine then drags the driven wheels for seconds).
    for (int i = 0; i < 60 * 40; ++i) {
        const Real s = world.vehicleVelocity(car).length();
        if (s >= speed) break;
        world.setVehicleInput(car, 1.0, 0, 0);
        world.update(1.0 / 60.0);
    }
    return car;
}

// Drive `seconds` with steer from `steerAt(t)` (raw keyboard value), the
// throttle held at `throttle`; optionally shaped through SteerShaper.
Probe drive(PhysicsWorld& world, PhysicsWorld::VehicleId car, Real seconds, Real throttle,
            Real (*steerAt)(Real), bool shaped, const char* tag) {
    Probe p;
    SteerShaper shaper;
    Real headingPrev = headingDeg(world.vehicleOrientation(car));
    const int steps = static_cast<int>(seconds * 60.0);
    for (int i = 0; i < steps; ++i) {
        const Real t = i / 60.0;
        Real steer = steerAt(t);
        if (shaped) {
            const Vec3 v = world.vehicleVelocity(car);
            const Quat q = world.vehicleOrientation(car);
            const Vec2 f = forward2(q);
            const Real fwdSpeed = v.x * f.x + v.z * f.y;
            steer = shaper.step(steer, fwdSpeed, 1.0 / 60.0);
        }
        world.setVehicleInput(car, throttle, steer, 0);
        world.update(1.0 / 60.0);
        sample(p, world, car);
        if (i >= steps - 30) {
            const Real h = headingDeg(world.vehicleOrientation(car));
            Real d = h - headingPrev;
            while (d > 180) d -= 360;
            while (d < -180) d += 360;
            p.endYawRate = std::fabs(d) * 60.0;
        }
        headingPrev = headingDeg(world.vehicleOrientation(car));
    }
    std::printf("[handling] %-22s %s: maxSlip=%.1f deg endSlip=%.1f maxRoll=%.1f minUp=%.2f "
                "endYaw=%.0f deg/s endSpeed=%.1f\n",
                tag, shaped ? "shaped" : "raw   ", p.maxSlipDeg, p.endSlipDeg, p.maxRollDeg,
                p.minUp, p.endYawRate, p.endSpeed);
    return p;
}

Real stepSteer(Real t) { return t < 0.6 ? 1.0 : 0.0; }

void trace(PhysicsWorld& world, PhysicsWorld::VehicleId car, Real throttle, Real steerConst,
           Real seconds, const char* tag) {
    SteerShaper shaper;
    for (int i = 0; i < static_cast<int>(seconds * 60); ++i) {
        const Vec3 v = world.vehicleVelocity(car);
        const Quat q = world.vehicleOrientation(car);
        const Vec2 f = forward2(q);
        const Real fwdSpeed = v.x * f.x + v.z * f.y;
        const Real steer = shaper.step(i < 36 ? steerConst : 0.0, fwdSpeed, 1.0 / 60.0);
        world.setVehicleInput(car, throttle, steer, 0);
        world.update(1.0 / 60.0);
        if (i % 15 == 0) {
            Probe p;
            sample(p, world, car);
            std::printf("[trace %s] t=%.2f steer=%.3f speed=%.1f slip=%.1f heading=%.1f\n", tag,
                        i / 60.0, steer, p.endSpeed, p.endSlipDeg, headingDeg(world.vehicleOrientation(car)));
        }
    }
}
Real slalomSteer(Real t) {
    const int k = static_cast<int>(t / 0.7);
    return t < 5.6 ? (k % 2 == 0 ? 1.0 : -1.0) : 0.0;
}

}  // namespace

Real brakeSteer(Real t) { return t < 1.5 ? 1.0 : 0.0; }

TEST_CASE(handling_probe_sweep_prints) {
    // Diagnosis only: step steer at rising speeds, trail braking, the
    // handbrake, full throttle for 20 s (top speed), and a kerb at speed.
    for (Real speed : {25.0, 40.0, 55.0, 70.0}) {
        PhysicsWorld world;
        world.initialize();
        const PhysicsWorld::VehicleId car = launch(world, playerSedan(), speed);
        char tag[64];
        std::snprintf(tag, sizeof tag, "step %.0f m/s", speed);
        drive(world, car, 5.0, 0.6, stepSteer, false, tag);
        world.shutdown();
        PhysicsWorld w2;
        w2.initialize();
        const PhysicsWorld::VehicleId car2 = launch(w2, playerSedan(), speed);
        drive(w2, car2, 5.0, 0.6, stepSteer, true, tag);
        w2.shutdown();
    }
    {
        PhysicsWorld world;
        world.initialize();
        const PhysicsWorld::VehicleId car = launch(world, playerSedan(), 40.0);
        trace(world, car, 0.6, 1.0, 3.0, "40 throttle");
        world.shutdown();
        PhysicsWorld w2;
        w2.initialize();
        const PhysicsWorld::VehicleId car2 = launch(w2, playerSedan(), 40.0);
        trace(w2, car2, 0.0, 1.0, 3.0, "40 coast");
        w2.shutdown();
    }
    for (Real speed : {20.0, 35.0}) {
        PhysicsWorld world;
        world.initialize();
        const PhysicsWorld::VehicleId car = launch(world, playerSedan(), speed);
        Probe p;
        for (int i = 0; i < 60 * 5; ++i) {
            const Real t = i / 60.0;
            world.setVehicleInput(car, 0.0, brakeSteer(t), t < 1.5 ? 1.0 : 0.0);
            world.update(1.0 / 60.0);
            sample(p, world, car);
        }
        std::printf("[handling] trail brake %.0f m/s: maxSlip=%.1f endSlip=%.1f maxRoll=%.1f minUp=%.2f\n",
                    speed, p.maxSlipDeg, p.endSlipDeg, p.maxRollDeg, p.minUp);
        world.shutdown();
    }
    {
        PhysicsWorld world;
        world.initialize();
        const PhysicsWorld::VehicleId car = launch(world, playerSedan(), 20.0);
        Probe p;
        for (int i = 0; i < 60 * 5; ++i) {
            const Real t = i / 60.0;
            world.setVehicleInput(car, 0.0, brakeSteer(t), 0.0, t < 1.5 ? 1.0 : 0.0);
            world.update(1.0 / 60.0);
            sample(p, world, car);
        }
        std::printf("[handling] handbrake 20 m/s: maxSlip=%.1f endSlip=%.1f maxRoll=%.1f minUp=%.2f\n",
                    p.maxSlipDeg, p.endSlipDeg, p.maxRollDeg, p.minUp);
        world.shutdown();
    }
    {
        PhysicsWorld world;
        world.initialize();
        world.addBox(Vec3(3000, 1, 3000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
        world.optimizeBroadPhase();
        const PhysicsWorld::VehicleId car = world.addVehicle(playerSedan(), Vec3(0, 0.9, 0), Quat::identity());
        Real top = 0;
        for (int i = 0; i < 60 * 25; ++i) {
            world.setVehicleInput(car, 1.0, 0, 0);
            world.update(1.0 / 60.0);
            top = std::max(top, world.vehicleVelocity(car).length());
            if (i == 60 * 5 || i == 60 * 10)
                std::printf("[handling] full throttle %ds: %.1f m/s\n", i / 60, world.vehicleVelocity(car).length());
        }
        std::printf("[handling] top speed after 25 s: %.1f m/s (%.0f km/h)\n", top, top * 3.6);
        world.shutdown();
    }
    struct Case { const char* name; Rig rig; };
    const Rig old{0.05, 0.15, 1.5, 0.5, 0.075, 180.0, 0.0};
    Rig oldCone = old; oldCone.cone = 65.0;
    Rig oldArb = old; oldArb.arb = 6000.0;
    Rig street{}; street.cone = 180.0; street.arb = 0.0;
    Rig streetCone = street; streetCone.cone = 65.0;
    Rig all{};
    const Case cases[] = { {"old rig", old}, {"old+cone", oldCone}, {"old+arb", oldArb},
                           {"street", street}, {"street+cone", streetCone}, {"street+cone+arb", all} };
    for (const Case& cs : cases)
    for (Real speed : {10.0, 18.0, 28.0}) {
        // A 0.15 m kerb crossed at 25 degrees (one front wheel first).
        PhysicsWorld world;
        world.initialize();
        world.addBox(Vec3(3000, 1, 3000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
        const Quat kerbRot = Quat::fromAxisAngle(Vec3(0, 1, 0), 25.0 * kPi / 180.0);
        world.addBox(Vec3(80, 0.075, 40), kerbRot.rotate(Vec3(0, 0, 40)) + Vec3(0, 0.075, 60), kerbRot,
                     BodyMotion::Static, 0.0, 0.85);
        world.optimizeBroadPhase();
        const PhysicsWorld::VehicleId car = world.addVehicle(playerSedan(cs.rig), Vec3(0, 0.9, 0), Quat::identity());
        for (int i = 0; i < 60; ++i) world.update(1.0 / 60.0);
        world.setLinearVelocity(world.vehicleBody(car), Vec3(0, 0, speed));
        Probe p;
        Real maxY = -1e9, minY = 1e9;
        for (int i = 0; i < 60 * 6; ++i) {
            world.setVehicleInput(car, 0.3, 0, 0);
            world.update(1.0 / 60.0);
            sample(p, world, car);
            const Real y = world.vehiclePosition(car).y;
            maxY = std::max(maxY, y);
            minY = std::min(minY, y);
        }
        std::printf("[handling] kerb %-16s at %.0f m/s: maxSlip=%.1f maxRoll=%.1f minUp=%.2f hop=%.2f\n",
                    cs.name, speed, p.maxSlipDeg, p.maxRollDeg, p.minUp, maxY - minY);
        world.shutdown();
    }
}

// THE FISHTAIL: a dab of steering at 140 km/h and a lifted throttle. Without
// the yaw assist the sideslip grew from 3 to 22 degrees in two seconds with
// the wheel straight (the trace above, yaw_assist 0); with it the car
// straightens.
TEST_CASE(handling_lift_off_at_140kmh_does_not_spin) {
    for (int assist = 0; assist < 2; ++assist) {
        PhysicsWorld world;
        world.initialize();
        PhysicsWorld::VehicleConfig cfg = playerSedan();
        cfg.yawAssist = assist ? 3.0 : 0.0;
        const PhysicsWorld::VehicleId car = launch(world, cfg, 40.0);
        SteerShaper shaper;
        Probe p;
        const Real h0 = headingDeg(world.vehicleOrientation(car));
        for (int i = 0; i < 60 * 4; ++i) {
            const Vec3 v = world.vehicleVelocity(car);
            const Vec2 f = forward2(world.vehicleOrientation(car));
            const Real steer = shaper.step(i < 36 ? 1.0 : 0.0, v.x * f.x + v.z * f.y, 1.0 / 60.0);
            world.setVehicleInput(car, 0.0, steer, 0);
            world.update(1.0 / 60.0);
            sample(p, world, car);
        }
        Real turned = headingDeg(world.vehicleOrientation(car)) - h0;
        while (turned > 180) turned -= 360;
        while (turned < -180) turned += 360;
        std::printf("[handling] lift-off 140 km/h assist=%d: maxSlip=%.1f endSlip=%.1f turned=%.0f deg\n",
                    assist, p.maxSlipDeg, p.endSlipDeg, turned);
        if (assist) {
            CHECK(p.maxSlipDeg < 10.0);
            CHECK(p.endSlipDeg < 2.0);
            CHECK(std::fabs(turned) < 45.0);
        } else {
            CHECK(p.maxSlipDeg > 15.0);   // the bug, kept measurable
        }
        world.shutdown();
    }
}

// The assist damps only EXCESS yaw: a steady steered turn must turn as far
// with it as without.
TEST_CASE(handling_yaw_assist_lets_the_car_turn_as_steered) {
    Real turned[2] = {0, 0};
    for (int assist = 0; assist < 2; ++assist) {
        PhysicsWorld world;
        world.initialize();
        PhysicsWorld::VehicleConfig cfg = playerSedan();
        cfg.yawAssist = assist ? 3.0 : 0.0;
        const PhysicsWorld::VehicleId car = launch(world, cfg, 12.0);
        const Real h0 = headingDeg(world.vehicleOrientation(car));
        Real total = 0, prev = h0;
        for (int i = 0; i < 60 * 3; ++i) {
            world.setVehicleInput(car, 0.4, 0.5, 0);
            world.update(1.0 / 60.0);
            const Real h = headingDeg(world.vehicleOrientation(car));
            Real d = h - prev;
            while (d > 180) d -= 360;
            while (d < -180) d += 360;
            total += d;
            prev = h;
        }
        turned[assist] = total;
        std::printf("[handling] steady turn assist=%d: turned %.0f deg in 3 s\n", assist, total);
        world.shutdown();
    }
    CHECK(std::fabs(turned[1]) > 0.85 * std::fabs(turned[0]));
    CHECK(std::fabs(turned[1]) > 60.0);
}

// Diagnosis (Glenn's drive, 2026-09-14: "the turn radius is really bad"):
// the steady turn radius against speed, full lock held, raw + no assist
// (the car before tonight) beside shaped + assist (the car now).
TEST_CASE(handling_probe_turn_radius_prints) {
    for (Real speed : {5.0, 8.0, 12.0, 16.0, 22.0, 28.0}) {
        for (int mode = 0; mode < 2; ++mode) {
            PhysicsWorld world;
            world.initialize();
            PhysicsWorld::VehicleConfig cfg = playerSedan();
            cfg.yawAssist = mode ? 3.0 : 0.0;
            const PhysicsWorld::VehicleId car = launch(world, cfg, speed);
            SteerShaper shaper;
            Real prev = headingDeg(world.vehicleOrientation(car)), total = 0, vsum = 0;
            int n = 0;
            for (int i = 0; i < 60 * 4; ++i) {
                const Vec3 v = world.vehicleVelocity(car);
                const Vec2 f = forward2(world.vehicleOrientation(car));
                const Real fwdSpeed = v.x * f.x + v.z * f.y;
                const Real steer = mode ? shaper.step(1.0, fwdSpeed, 1.0 / 60.0) : 1.0;
                // Hold the speed: a little throttle when under, none when over.
                world.setVehicleInput(car, fwdSpeed < speed ? 0.5 : 0.0, steer, 0);
                world.update(1.0 / 60.0);
                const Real h = headingDeg(world.vehicleOrientation(car));
                Real d = h - prev;
                while (d > 180) d -= 360;
                while (d < -180) d += 360;
                prev = h;
                if (i >= 60) { total += d; vsum += fwdSpeed; ++n; }
            }
            const Real yawRate = std::fabs(total) / (n / 60.0) * kPi / 180.0;   // rad/s
            const Real vAvg = vsum / n;
            std::printf("[handling] radius %-14s at %4.1f m/s: v=%.1f yaw=%.2f rad/s radius=%.1f m\n",
                        mode ? "shaped+assist" : "raw", speed, vAvg, yawRate, yawRate > 1e-3 ? vAvg / yawRate : 0.0);
            world.shutdown();
        }
    }
}

TEST_CASE(handling_step_steer_at_90kmh_settles_straight) {
    for (int shaped = 0; shaped < 2; ++shaped) {
        PhysicsWorld world;
        world.initialize();
        const PhysicsWorld::VehicleId car = launch(world, playerSedan(), 25.0);
        const Probe p = drive(world, car, 5.0, 0.6, stepSteer, shaped == 1, "step steer 90 km/h");
        if (shaped) {
            CHECK(p.minUp > 0.95);
            CHECK(p.maxSlipDeg < 25.0);       // the rear never steps out into a spin
            CHECK(p.endSlipDeg < 3.0);        // and it comes back straight
            CHECK(p.endYawRate < 10.0);
            CHECK(p.maxRollDeg < 6.0);
        }
        world.shutdown();
    }
}

TEST_CASE(handling_tank_slapper_at_70kmh_stays_bounded) {
    for (int shaped = 0; shaped < 2; ++shaped) {
        PhysicsWorld world;
        world.initialize();
        const PhysicsWorld::VehicleId car = launch(world, playerSedan(), 19.5);
        const Probe p = drive(world, car, 8.0, 0.6, slalomSteer, shaped == 1, "tank slapper 70 km/h");
        if (shaped) {
            CHECK(p.minUp > 0.95);
            CHECK(p.maxSlipDeg < 30.0);
            CHECK(p.endSlipDeg < 4.0);
            CHECK(p.maxRollDeg < 6.0);
        }
        world.shutdown();
    }
}

TEST_CASE(handling_slanted_kerb_at_65kmh_stays_upright) {
    // The old street rig (0.05..0.15 at 1.5 Hz) rolled the car over here
    // (minUp -0.99, a 1.8 m hop); the softer rig crosses it with ~2 deg of
    // roll. The cone is the safety net for whatever the city still throws.
    PhysicsWorld world;
    world.initialize();
    world.addBox(Vec3(3000, 1, 3000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
    const Quat kerbRot = Quat::fromAxisAngle(Vec3(0, 1, 0), 25.0 * kPi / 180.0);
    world.addBox(Vec3(80, 0.075, 40), kerbRot.rotate(Vec3(0, 0, 40)) + Vec3(0, 0.075, 60), kerbRot,
                 BodyMotion::Static, 0.0, 0.85);
    world.optimizeBroadPhase();
    const PhysicsWorld::VehicleId car = world.addVehicle(playerSedan(), Vec3(0, 0.9, 0), Quat::identity());
    for (int i = 0; i < 60; ++i) world.update(1.0 / 60.0);
    world.setLinearVelocity(world.vehicleBody(car), Vec3(0, 0, 18.0));
    Probe p;
    Real maxY = -1e9, minY = 1e9;
    for (int i = 0; i < 60 * 6; ++i) {
        world.setVehicleInput(car, 0.3, 0, 0);
        world.update(1.0 / 60.0);
        sample(p, world, car);
        maxY = std::max(maxY, world.vehiclePosition(car).y);
        minY = std::min(minY, world.vehiclePosition(car).y);
    }
    std::printf("[handling] slanted kerb 65 km/h: maxRoll=%.1f minUp=%.2f hop=%.2f\n", p.maxRollDeg, p.minUp, maxY - minY);
    CHECK(p.minUp > 0.95);
    CHECK(p.maxRollDeg < 10.0);
    CHECK(maxY - minY < 0.45);
    world.shutdown();
}

TEST_CASE(handling_sideways_kerb_trip_stays_upright) {
    PhysicsWorld world;
    world.initialize();
    world.addBox(Vec3(3000, 1, 3000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static,
                 0.0, 0.85);
    // A kerb face across the slide, then the pavement behind it.
    world.addBox(Vec3(40, 0.075, 40), Vec3(46, 0.075, 0), Quat::identity(), BodyMotion::Static,
                 0.0, 0.85);
    world.optimizeBroadPhase();
    const PhysicsWorld::VehicleId car =
        world.addVehicle(playerSedan(), Vec3(0, 0.9, 0), Quat::identity());
    for (int i = 0; i < 60; ++i) world.update(1.0 / 60.0);
    world.setLinearVelocity(world.vehicleBody(car), Vec3(8.0, 0, 0));   // pure sideways slide
    Probe p;
    for (int i = 0; i < 60 * 4; ++i) {
        world.setVehicleInput(car, 0, 0, 0);
        world.update(1.0 / 60.0);
        sample(p, world, car);
    }
    const Vec3 pos = world.vehiclePosition(car);
    std::printf("[handling] kerb trip: x=%.1f maxRoll=%.1f minUp=%.2f\n", pos.x, p.maxRollDeg, p.minUp);
    CHECK(p.minUp > 0.85);
    world.shutdown();
}
