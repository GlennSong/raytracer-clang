// DRIVETRAINS (#41; Glenn: "offroad vehicles -- 2 wheel drive, 4 wheel drive, etc." and "the car once it hits
// a certain acceleration on the freeway begins to lag behind"). Headless, on the player sedan's numbers:
//   hill      a slippery slope: four-wheel drive climbs where one driven axle spins
//   switch    a part-time 4x4 stalled in 2WD climbs again when 4WD is engaged mid-hill
//   top speed quadratic aero drag gives a real top speed, and the quick gearbox no longer stalls a pull
#include "test_framework.h"

#include "../src/engine/physics/physics_world.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace engine;

namespace {

PhysicsWorld::VehicleConfig car(Real frontShare) {
    PhysicsWorld::VehicleConfig c;
    const Real L = 4.60, W = 1.82, H = 1.45, wd = 0.62;
    c.chassisHalfExtent = Vec3(W * 0.5, H * 0.5, L * 0.5);
    c.mass = std::floor(L * W * H * 120.0);
    c.comOffsetY = -0.45;
    c.engineTorque = std::floor(c.mass * 0.46);
    c.maxSteerDegrees = 32.0;
    c.brakeTorque = std::floor(c.mass * 1.15);
    c.handBrakeTorque = std::floor(c.mass * 2.9);
    c.frontDriveShare = frontShare;
    const Real r = wd * 0.5, halfTrack = 0.78;
    const Real axleY = -H * 0.5 + r + PhysicsWorld::kStreetSuspensionRestDrop;
    const Real frontZ = L * 0.5 - 1.45 * wd, rearZ = -(L * 0.5 - 1.61 * wd);
    auto wheel = [&](Real x, Real z, bool steered, bool hand) {
        PhysicsWorld::VehicleWheel w;
        w.position = Vec3(x, axleY, z);
        w.radius = r;
        w.width = 0.2;
        w.suspensionMin = PhysicsWorld::kStreetSuspensionMin;
        w.suspensionMax = PhysicsWorld::kStreetSuspensionMax;
        w.suspensionFrequency = PhysicsWorld::kStreetSuspensionFrequency;
        w.suspensionDamping = PhysicsWorld::kStreetSuspensionDamping;
        w.steered = steered;
        w.driven = true;          // a 4x4's wheels are all connected; the split decides who gets torque
        w.handBrake = hand;
        return w;
    };
    c.wheels = { wheel(halfTrack, frontZ, true, false), wheel(-halfTrack, frontZ, true, false),
                 wheel(halfTrack, rearZ, false, true), wheel(-halfTrack, rearZ, false, true) };
    return c;
}

// A long slippery slope, `deg` steep, friction `mu`, rising along +z; the car starts AT REST ON it (a run-up
// would test momentum, not traction), nose uphill, held on its brakes while it settles.
PhysicsWorld::VehicleId onHill(PhysicsWorld& w, const PhysicsWorld::VehicleConfig& cfg, Real deg, Real mu) {
    const Real a = deg * 3.14159265358979 / 180.0;
    const Quat tilt = Quat::fromAxisAngle(Vec3(1, 0, 0), -a);   // +z goes up
    w.addBox(Vec3(20, 1, 200), tilt.rotate(Vec3(0, -1, 0)), tilt, BodyMotion::Static, 0.0, mu);
    w.optimizeBroadPhase();
    const PhysicsWorld::VehicleId id = w.addVehicle(cfg, tilt.rotate(Vec3(0, 0.9, -150)), tilt);
    for (int i = 0; i < 90; ++i) { w.setVehicleInput(id, 0, 0, 1.0, 1.0); w.update(1.0 / 60.0); }
    return id;
}

// Metres of height gained from rest under full throttle over `seconds` (negative: it slid back).
Real climb(PhysicsWorld& w, PhysicsWorld::VehicleId id, Real seconds) {
    const Real y0 = w.vehiclePosition(id).y;
    for (int i = 0; i < static_cast<int>(seconds * 60); ++i) {
        w.setVehicleInput(id, 1.0, 0, 0);
        w.update(1.0 / 60.0);
    }
    return w.vehiclePosition(id).y - y0;
}

}  // namespace

TEST_CASE(drivetrain_four_wheel_drive_climbs_a_slippery_hill_that_two_wheel_drive_cannot) {
    Real height[3];
    const Real shares[3] = {0.0, 1.0, 0.5};   // RWD, FWD, 4WD
    for (int k = 0; k < 3; ++k) {
        PhysicsWorld w;
        w.initialize();
        const auto id = onHill(w, car(shares[k]), 18.0, 0.35);
        CHECK_APPROX(w.vehicleFrontDriveShare(id), shares[k], 1e-6);
        height[k] = climb(w, id, 10.0);
    }
    std::printf("    18 deg at mu 0.35, 10 s from rest: RWD %.2f m, FWD %.2f m, 4WD %.2f m\n", height[0], height[1], height[2]);
    CHECK(height[2] > 10.0);              // four driven wheels climb
    CHECK(height[0] < 3.0);               // one driven axle barely holds (rear: weight shifts onto it uphill)...
    CHECK(height[1] < 0.0);               // ...or slides back (front: weight shifts off it)
    CHECK(height[0] > height[1]);
}

TEST_CASE(drivetrain_engaging_four_wheel_drive_mid_hill_gets_a_stalled_truck_climbing) {
    PhysicsWorld w;
    w.initialize();
    const auto id = onHill(w, car(0.0), 18.0, 0.35);   // part-time 4x4 in 2WD (rear)
    const Real stuck = climb(w, id, 10.0);
    w.setVehicleFrontDriveShare(id, 0.5);              // 4WD
    CHECK_APPROX(w.vehicleFrontDriveShare(id), 0.5, 1e-6);
    const Real after = climb(w, id, 10.0);
    std::printf("    2WD stalled at %.2f m; 4WD engaged, reached %.2f m\n", stuck, after);
    CHECK(after > 10.0);
}

TEST_CASE(drivetrain_aero_gives_a_top_speed_and_quick_shifts_pull_through) {
    auto run = [](bool modern, Real& t50to110, Real& vTop, Real& gain) {
        PhysicsWorld w;
        w.initialize();
        w.addBox(Vec3(20000, 1, 20000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
        w.optimizeBroadPhase();
        PhysicsWorld::VehicleConfig cfg = car(0.5);
        if (modern) {
            cfg.dragArea = 2.0;   // Cd x A with rolling losses and the arcade engine's excess: ~205 km/h top
            cfg.shiftTime = 0.2;
            cfg.clutchReleaseTime = 0.15;
            cfg.shiftLatency = 0.25;
        }
        const auto id = w.addVehicle(cfg, Vec3(0, 0.9, 0), Quat::identity());
        for (int i = 0; i < 60; ++i) w.update(1.0 / 60.0);
        Real t = 0, t50 = -1, t110 = -1, v30 = 0, v40 = 0;
        for (int i = 0; i < 60 * 40; ++i) {
            w.setVehicleInput(id, 1.0, 0, 0);
            w.update(1.0 / 60.0);
            t += 1.0 / 60.0;
            const Real v = w.vehicleTelemetry(id).speed;
            if (t50 < 0 && v >= 50 / 3.6) t50 = t;
            if (t110 < 0 && v >= 110 / 3.6) t110 = t;
            if (i == 60 * 30) v30 = v;
            v40 = v;
        }
        t50to110 = (t50 >= 0 && t110 >= 0) ? t110 - t50 : 1e9;
        vTop = v40;
        gain = v40 - v30;
    };
    Real tOld, vOld, gOld, tNew, vNew, gNew;
    run(false, tOld, vOld, gOld);
    run(true, tNew, vNew, gNew);
    std::printf("    50->110 km/h: old %.2f s, new %.2f s; speed at 40 s: old %.1f, new %.1f m/s (last 10 s gained %.2f / %.2f)\n",
                tOld, tNew, vOld, vNew, gOld, gNew);
    CHECK(tNew < tOld);          // the quick gearbox pulls through the shifts
    CHECK(gNew < 0.03 * vNew);   // a real top speed: under 3% gained from 30 to 40 s (the asymptotic approach)
    CHECK(vNew * 3.6 < 230.0);   // ...and a believable one
}

TEST_CASE(drivetrain_probe_hill_sweep_prints) {
    for (const Real mu : {0.2, 0.35, 0.5}) {
        for (const Real deg : {12.0, 18.0, 24.0, 30.0}) {
            Real h[3];
            const Real shares[3] = {0.0, 1.0, 0.5};
            for (int k = 0; k < 3; ++k) {
                PhysicsWorld w;
                w.initialize();
                const auto id = onHill(w, car(shares[k]), deg, mu);
                h[k] = climb(w, id, 10.0);
            }
            std::printf("    mu %.2f %2.0f deg: RWD %6.2f  FWD %6.2f  4WD %6.2f m\n", mu, deg, h[0], h[1], h[2]);
        }
    }
    for (const Real cda : {1.2, 2.0, 3.0}) {
        PhysicsWorld w;
        w.initialize();
        w.addBox(Vec3(20000, 1, 20000), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
        w.optimizeBroadPhase();
        PhysicsWorld::VehicleConfig cfg = car(0.5);
        cfg.dragArea = cda;
        const auto id = w.addVehicle(cfg, Vec3(0, 0.9, 0), Quat::identity());
        Real v30 = 0, v = 0;
        for (int i = 0; i < 60 * 45; ++i) {
            w.setVehicleInput(id, 1.0, 0, 0);
            w.update(1.0 / 60.0);
            v = w.vehicleTelemetry(id).speed;
            if (i == 60 * 35) v30 = v;
        }
        std::printf("    CdA %.2f: %.1f km/h at 45 s (+%.2f m/s over the last 10)\n", cda, v * 3.6, v - v30);
    }
}

namespace {
// vehicles.lua vehicle.offroad, as its spec reads back (test_vehicle_body offroad_spec...): 2300 kg,
// 1265 Nm to 5200 rpm through low gears, grip 2.3, near-locking diffs, 0.48 m wheels on 0.40 m of 1.3 Hz
// travel resting 0.30 m below the attach points, CoG 0.62 m down.
PhysicsWorld::VehicleConfig offroad(bool fourWheel) {
    PhysicsWorld::VehicleConfig c;
    const Real W = 2.10, H = 2.20, L = 4.75, r = 0.48;
    c.chassisHalfExtent = Vec3(W * 0.5, H * 0.5, L * 0.5);
    c.mass = 2300;
    c.comOffsetY = -0.62;
    c.engineTorque = std::floor(2300 * 0.55);
    c.maxRPM = 5200;
    c.maxSteerDegrees = 32;
    c.brakeTorque = std::floor(2300 * 1.2);
    c.handBrakeTorque = std::floor(2300 * 2.9);
    c.lateralGrip = 2.3;
    c.frontDriveShare = fourWheel ? 0.5 : 0.0;
    c.axleLimitedSlip = 1.15;
    c.centerLimitedSlip = 1e30;   // open between the axles: a fixed 50/50 split
    c.dragArea = 2.8;
    c.shiftTime = 0.2;
    c.clutchReleaseTime = 0.15;
    c.shiftLatency = 0.25;
    c.gearRatios = {3.9, 2.4, 1.6, 1.15, 0.9};
    const Real axleY = -H * 0.5 + r + 0.30;
    auto wheel = [&](Real x, Real z, bool front) {
        PhysicsWorld::VehicleWheel w;
        w.position = Vec3(x, axleY, z);
        w.radius = r;
        w.width = 0.32;
        w.suspensionMin = 0.0;
        w.suspensionMax = 0.40;
        w.suspensionFrequency = 1.3;
        w.suspensionDamping = 0.5;
        w.steered = front;
        w.driven = true;
        w.handBrake = !front;
        return w;
    };
    c.wheels = { wheel(0.83, 1.60, true), wheel(-0.83, 1.60, true), wheel(0.83, -1.58, false), wheel(-0.83, -1.58, false) };
    return c;
}

// A square step `h` high across the car's path 3 m ahead; true once the whole car is up on it within 20 s.
bool clearsLedge(bool fourWheel, Real h) {
    PhysicsWorld w;
    w.initialize();
    w.addBox(Vec3(200, 1, 200), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
    w.addBox(Vec3(8, h, 20), Vec3(0, h * 0.5, 3.0 + 2.4 + 10.0), Quat::identity(), BodyMotion::Static, 0.0, 0.85);
    w.optimizeBroadPhase();
    const auto id = w.addVehicle(offroad(fourWheel), Vec3(0, 1.2, 0), Quat::identity());
    for (int i = 0; i < 90; ++i) w.update(1.0 / 60.0);
    for (int i = 0; i < 60 * 20; ++i) {
        w.setVehicleInput(id, 0.7, 0, 0);   // a crawl, not a charge
        w.update(1.0 / 60.0);
        const Vec3 p = w.vehiclePosition(id);
        if (p.z > 3.0 + 2.4 + 4.0 && p.y > h + 0.5) return true;   // the rear axle is past the lip, on top
    }
    return false;
}
}  // namespace

// Glenn: "could that offroader climb over rocks, steep hills?" -- the off-roader's own numbers, 2WD against
// 4WD, over the course's slopes and ledges (tools/offroad_course.py): a slope is CLIMBED when the truck
// gains its ramp's 6 m from rest within 12 s.
TEST_CASE(offroader_climbs_what_the_course_asks_in_four_wheel_drive) {
    struct Hill { Real deg, mu; };
    const Hill hills[] = { {15, 0.9}, {20, 0.9}, {25, 0.9}, {30, 0.9}, {35, 0.9},
                           {10, 0.35}, {15, 0.35}, {20, 0.35}, {25, 0.35}, {30, 0.35} };
    int climbed4 = 0, climbed2 = 0;
    bool grip30in4 = false, mud20in4 = false, mud20in2 = false;
    for (const Hill& hl : hills) {
        Real gained[2];
        for (int four = 0; four < 2; ++four) {
            PhysicsWorld w;
            w.initialize();
            const auto id = onHill(w, offroad(four != 0), hl.deg, hl.mu);
            gained[four] = climb(w, id, 12.0);
        }
        const bool c2 = gained[0] >= 6.0, c4 = gained[1] >= 6.0;
        climbed2 += c2;
        climbed4 += c4;
        if (hl.mu > 0.5 && hl.deg == 30) grip30in4 = c4;
        if (hl.mu < 0.5 && hl.deg == 20) { mud20in4 = c4; mud20in2 = c2; }
        std::printf("    %2.0f deg at mu %.2f: 2WD %+6.1f m %s   4WD %+6.1f m %s\n", hl.deg, hl.mu, gained[0],
                    c2 ? "CLIMBS" : "      ", gained[1], c4 ? "CLIMBS" : "");
    }
    std::printf("    ledges (crawl):");
    bool ledge40 = false;
    for (const Real h : {0.2, 0.3, 0.4, 0.5}) {
        const bool l2 = clearsLedge(false, h), l4 = clearsLedge(true, h);
        if (h == 0.4) ledge40 = l4;
        std::printf("  %.1f m: 2WD %s 4WD %s;", h, l2 ? "yes" : "no", l4 ? "yes" : "no");
    }
    std::printf("\n");
    CHECK(climbed4 > climbed2);   // 4WD climbs more of the course
    CHECK(grip30in4);             // a 30 degree dirt hill
    CHECK(mud20in4 && !mud20in2); // a 20 degree mud hill is a 4WD hill
    CHECK(ledge40);               // a 0.4 m ledge (over 80% of the wheel's radius)
}

TEST_CASE(offroader_probe_differential_sweep_prints) {
    const float big = 1e30f;
    struct Setup { const char* name; Real share, axle, center; };
    const Setup setups[] = { {"2WD open axle", 0.0, 1e30, 1e30}, {"2WD lsd 1.15", 0.0, 1.15, 1e30},
                             {"4WD c1.10 a1.15", 0.5, 1.15, 1.10}, {"4WD c1.4 a1.4", 0.5, 1.4, 1.4},
                             {"4WD c-open a1.15", 0.5, 1.15, big}, {"4WD c-open a-open", 0.5, 1e30, big} };
    for (const Real deg : {15.0, 25.0, 30.0}) {
        for (const Setup& s : setups) {
            PhysicsWorld w;
            w.initialize();
            PhysicsWorld::VehicleConfig c = offroad(s.share > 0.25);
            c.frontDriveShare = s.share;
            c.axleLimitedSlip = s.axle;
            c.centerLimitedSlip = s.center;
            const auto id = onHill(w, c, deg, 0.9);
            const Real g = climb(w, id, 12.0);
            std::printf("    %2.0f deg mu 0.9  %-20s %+7.1f m\n", deg, s.name, g);
        }
    }
}
