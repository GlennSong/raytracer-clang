// The player at the wheel (engine/player/driver_seat.h, ADR-0252): what a seat's jolt throws, how the body sways in
// the seat, and that the seated body's hands and feet land where the seat puts them.
#include "test_framework.h"

#include "../src/engine/player/driver_seat.h"

#include <cmath>
#include <cstdio>

using namespace engine;

namespace {

constexpr Real kDt = 1.0 / 60.0;

// a ride along +z: the most jolt it gives over the run
struct Run {
    RideJolt jolt;
    SeatSway sway;
    Vec3 last;
    bool have = false;
    Real most = 0;
    Real leanMost = 0;   // the furthest forward the shoulders went
    void step(const Vec3& v) {
        most = std::max(most, jolt.step(v, kDt));
        sway.step(have ? v - last : Vec3(0, 0, 0), kDt);
        last = v;
        have = true;
        leanMost = std::max(leanMost, sway.lean.z);
    }
};

// hold `v` a second, then stop in `steps` steps
Run stopFrom(Real speed, int steps) {
    Run r;
    for (int i = 0; i < 60; ++i) r.step(Vec3(0, 0, speed));
    for (int i = 1; i <= steps; ++i) r.step(Vec3(0, 0, speed * (1.0 - Real(i) / steps)));
    for (int i = 0; i < 30; ++i) r.step(Vec3(0, 0, 0));
    return r;
}

Real dist(const Vec3& a, const Vec3& b) { return (a - b).length(); }

}  // namespace

TEST_CASE(hard_braking_leans_you_forward_and_throws_nobody) {
    // 1 g from 72 km/h: two seconds of the hardest braking a road car does
    Run r;
    for (int i = 0; i < 60; ++i) r.step(Vec3(0, 0, 20));
    Real v = 20;
    while (v > 0) {
        v = std::max(Real(0), v - 9.8 * kDt);
        r.step(Vec3(0, 0, v));
    }
    std::printf("    braking: jolt %.2f m/s, lean %.3f m\n", r.most, r.leanMost);
    CHECK(r.most < ejectTolerance(true, true, false));   // not even a bike
    CHECK(r.leanMost > 0.04);                            // forward into the belt...
    CHECK(r.leanMost < SeatSway::kLeanMax * 0.6);        // ...not thrown to it
}

TEST_CASE(a_wall_at_fifty_throws_the_open_seats_and_keeps_the_closed_car) {
    const Run r = stopFrom(50 / 3.6, 2);
    std::printf("    50 km/h into a wall: jolt %.2f m/s, lean %.3f m\n", r.most, r.leanMost);
    CHECK(r.most > ejectTolerance(true, true, false));    // a bike
    CHECK(r.most > ejectTolerance(false, true, false));   // a convertible
    CHECK(!(r.most > ejectTolerance(false, false, false)));   // a closed car: never
    CHECK(!(r.most > ejectTolerance(false, false, true)));    // ...nor through the windscreen at 50
    CHECK(stopFrom(100 / 3.6, 2).most > ejectTolerance(false, false, true));   // ...but at 100, with it on
    // and the body in the seat goes to the belt
    CHECK(r.leanMost > SeatSway::kLeanMax * 0.95);
}

TEST_CASE(a_slow_bump_throws_nobody) {
    // 15 km/h into a kerb post: a bike's rider stays on
    CHECK(stopFrom(15 / 3.6, 2).most < ejectTolerance(true, true, false));
}

TEST_CASE(a_landing_is_not_a_crash) {
    // a 2 m drop (6.3 m/s down) taken by the springs over a few steps, rolling on at 40 km/h
    Run r;
    for (int i = 0; i < 30; ++i) r.step(Vec3(0, -6.3 * Real(i) / 30, 11));
    for (int i = 1; i <= 3; ++i) r.step(Vec3(0, -6.3 * (1 - Real(i) / 3), 11));
    for (int i = 0; i < 30; ++i) r.step(Vec3(0, 0, 11));
    std::printf("    2 m landing: jolt %.2f m/s\n", r.most);
    CHECK(r.most < ejectTolerance(true, true, false));
}

TEST_CASE(the_body_comes_back_upright_after_a_crash) {
    Run r = stopFrom(50 / 3.6, 2);
    for (int i = 0; i < 120; ++i) r.step(Vec3(0, 0, 0));
    std::printf("    two seconds on: lean %.4f m, nod %.4f m\n", r.sway.lean.length(), r.sway.nod.length());
    CHECK(r.sway.lean.length() < 0.01);
    CHECK(r.sway.nod.length() < 0.01);
}

TEST_CASE(struck_from_the_left_the_body_goes_left) {
    // standing at the lights, hit on the left (+x) side: the car goes right (-x) and the body stays where it was --
    // toward the side that was struck
    Run r;
    for (int i = 0; i < 30; ++i) r.step(Vec3(0, 0, 0));
    r.step(Vec3(-3, 0, 0));
    r.step(Vec3(-6, 0, 0));
    for (int i = 0; i < 4; ++i) r.step(Vec3(-6, 0, 0));
    CHECK(r.sway.lean.x > 0.1);
    CHECK(std::abs(r.sway.lean.z) < 0.02);
}

TEST_CASE(thrown_off_at_most_of_the_speed_it_had) {
    const Vec3 t = ejectThrow(Vec3(0, 0, 14), true);
    CHECK(std::abs(t.z - 14 * 0.8) < 1e-9);
    CHECK(t.y > 2.5);
    CHECK(ejectThrow(Vec3(0, 0, 10), false).z < t.z);   // going down is a slide off, not a launch
}

TEST_CASE(the_seated_body_holds_the_wheel_and_the_bars_and_reaches_the_pedals) {
    struct Seat {
        const char* name;
        SeatGeometry g;
    };
    SeatGeometry car;
    SeatGeometry moto;
    moto.grip = Vec3(0.30, 0.14, 0.67);
    moto.foot = Vec3(0.19, -0.44, 0.08);
    moto.lean = 28;
    moto.wheel = false;
    SeatGeometry bike = moto;
    bike.grip = Vec3(0.24, 0.08, 0.60);
    bike.foot = Vec3(0.10, -0.55, 0.04);
    bike.lean = 35;
    bike.pedals = true;
    const Seat seats[] = {{"car", car}, {"motorcycle", moto}, {"bicycle", bike}};
    for (const Seat& s : seats) {
        for (Real steer : {0.0, 0.6, -1.0}) {
            for (Real crank : {0.0, 0.3}) {
                const BodyPose p = seatedPose(s.g, SeatSway{}, steer, crank, 1.0);
                // the pelvis on the hip point
                const Vec3 pelvis = p.position + p.orientation.rotate(BlockBody::parts()[BlockBody::Torso].joint);
                CHECK(pelvis.length() < 1e-6);
                for (int side = 0; side < 2; ++side) {
                    const Real hand = dist(solveLimb(p, side).end, seatGrip(s.g, side, steer));
                    const Real foot = dist(solveLimb(p, 2 + side).end, seatFoot(s.g, side, crank));
                    if (hand > 0.02 || foot > 0.02)
                        std::printf("    %s steer %.1f crank %.1f side %d: hand off by %.3f (shoulder %.3f away), foot by %.3f\n",
                                    s.name, steer, crank, side, hand,
                                    dist(solveLimb(p, side).root, seatGrip(s.g, side, steer)), foot);
                    CHECK(hand < 0.02);
                    CHECK(foot < 0.02);
                }
            }
        }
    }
    // the wheel turns the hands round it; the cranks keep the feet half a turn apart
    CHECK(dist(seatGrip(car, 0, 0.5), seatGrip(car, 0, 0)) > 0.06);
    CHECK(std::abs(seatFoot(bike, 0, 0.1).y + seatFoot(bike, 1, 0.1).y - 2 * bike.foot.y) < 1e-9);
}

TEST_CASE(a_lurch_tips_the_torso_and_the_head_goes_further) {
    SeatSway sway;
    sway.lean = Vec3(0, 0, 0.2);
    sway.nod = Vec3(0, 0, 0.08);
    const SeatGeometry car;
    const BodyPose still = seatedPose(car, SeatSway{}, 0, 0, 1);
    const BodyPose thrown = seatedPose(car, sway, 0, 0, 1);
    const std::array<PartTransform, BlockBody::kParts> a = poseBlocks(still), b = poseBlocks(thrown);
    // the shoulders forward, the head forward of them (it nods on the neck)
    CHECK(b[BlockBody::Torso].position.z > a[BlockBody::Torso].position.z + 0.05);
    CHECK(b[BlockBody::Head].position.z - b[BlockBody::Torso].position.z >
          a[BlockBody::Head].position.z - a[BlockBody::Torso].position.z + 0.02);
    // and the hands still on the wheel
    CHECK(dist(solveLimb(thrown, 0).end, seatGrip(car, 0, 0)) < 0.02);
}
