// GETTING OUT OF THE WAY (ADR-0355): a car is its box, not its centre; a car coming at someone is a threat with an
// escape out of its path; people notice after their own reaction time, then sprint (or dive) clear -- a few freeze.

#include "test_framework.h"
#include "../src/engine/ai/danger_dodge.h"

using namespace engine;

namespace {
DangerCar carAt(Vec2 pos, Vec2 fwd, Real speed) {
    DangerCar c;
    c.pos = pos;
    c.fwd = fwd;
    c.vel = fwd * speed;
    return c;
}
}  // namespace

TEST_CASE(a_cars_nose_hits_before_its_centre_would) {
    const DangerCar c = carAt(Vec2(0, 0), Vec2(0, 1), 10);
    CHECK(insideFootprint(c, Vec2(0.2, 2.5), 0.35));    // at the bumper: 2.5 m from the centre
    CHECK(!insideFootprint(c, Vec2(1.6, 0), 0.35));     // beside it, clear
    CHECK(insideFootprint(c, Vec2(1.1, -1.0), 0.35));   // at its flank
}

TEST_CASE(a_car_coming_is_a_threat_with_a_way_out) {
    const DangerCar c = carAt(Vec2(0, 0), Vec2(0, 1), 12);   // 43 km/h up +Z
    const Threat ahead = assessThreat(c, Vec2(0.4, 12));      // in its lane, 1.2 s off
    CHECK(ahead.coming);
    CHECK(ahead.ttc > 0.6 && ahead.ttc < 1.0);
    CHECK(ahead.escape.x > 0.9);    // out of the path on the side they already stand (+X here)
    CHECK(!assessThreat(c, Vec2(4.0, 10)).coming);   // on the pavement beside its path
    CHECK(!assessThreat(c, Vec2(0, -5)).coming);     // behind it
    CHECK(!assessThreat(c, Vec2(0, 40)).coming);     // far ahead: not yet
    CHECK(!assessThreat(carAt(Vec2(0, 0), Vec2(0, 1), 2.0), Vec2(0, 4)).coming);   // creeping: no danger
    // reversing: the tail is the front
    DangerCar back = carAt(Vec2(0, 0), Vec2(0, 1), 0);
    back.vel = Vec2(0, -6);
    CHECK(assessThreat(back, Vec2(0, -6)).coming);
}

TEST_CASE(people_notice_then_get_clear_and_some_freeze) {
    const DangerCar c = carAt(Vec2(0, 0), Vec2(0, 1), 12);
    int froze = 0, cleared = 0;
    for (uint32_t who = 0; who < 70; ++who) {
        const DodgeTraits d = dodgeTraits(who);
        CHECK(d.reaction >= 0.25 && d.reaction <= 0.65);
        DodgeState s;
        Vec2 p(0.3, 16.0);   // in its lane, 1.1 s off
        DangerCar car = c;
        bool hit = false;
        for (int i = 0; i < 120 && !hit; ++i) {
            const Real dt = 1.0 / 60;
            const Threat t = assessThreat(car, p);
            Vec2 v;
            if (stepDodge(s, d, t, dt, v)) p = p + v * dt;
            car.pos = car.pos + car.vel * dt;
            hit = insideFootprint(car, p, 0.35);
        }
        froze += d.freezes;
        cleared += !hit;
        if (!d.freezes) CHECK(!hit);   // everyone who moves gets out of a 43 km/h car's way from 16 m
    }
    std::printf("    [dodge] of 70 people: %d froze, %d cleared\n", froze, cleared);
    CHECK(froze >= 4 && froze <= 18);   // about one in seven
}
