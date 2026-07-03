#include "test_framework.h"

#include "../src/engine/ai/lane_follow.h"
#include "../src/engine/ai/traffic_sense.h"

#include <cmath>
#include <vector>

using namespace engine;

// Real-pose traffic perception (ADR-0062): a physical car senses the other
// PHYSICAL road users ahead of it and caps its speed off them — plans alone
// can't keep two physics-driven cars apart. Unit rules first, then the whole
// stack closed-loop: two bicycle-model cars in convoy on one lane.

namespace {

VisionCone coneAt(Vec2 pos, Vec2 fwd, Real range = 22.0, Real halfAngle = 0.5) {
    VisionCone c;
    c.origin = pos;
    c.forward = fwd;
    c.range = range;
    c.halfAngleRad = halfAngle;
    return c;
}

SensedBody body(Vec2 pos, Vec2 heading, Real speed, Real halfLen = 2.1) {
    SensedBody b;
    b.pos = pos;
    b.heading = heading;
    b.speed = speed;
    b.halfLength = halfLen;
    return b;
}

// Same bicycle model as test_lane_follow.cpp: + steer = right, yaw rate
// proportional to speed.
struct BikeCar {
    Vec2 pos{0, 0};
    Vec2 fwd{0, 1};
    Real speed = 0;

    void step(const DriverInput& in, Real dt) {
        Real accel = in.throttle * 4.0 - in.brake * 7.0;
        speed = std::max(Real(0), std::min(Real(30), speed + accel * dt));
        Real wheel = in.steer * 0.55;
        Real yaw = speed / 2.8 * std::tan(wheel) * dt;
        Vec2 right(fwd.y, -fwd.x);
        Real ca = std::cos(yaw), sa = std::sin(yaw);
        fwd = Vec2(fwd.x * ca + right.x * sa, fwd.y * ca + right.y * sa);
        Real l = std::sqrt(fwd.x * fwd.x + fwd.y * fwd.y);
        fwd = Vec2(fwd.x / l, fwd.y / l);
        pos = pos + fwd * (speed * dt);
    }
};

}  // namespace

TEST_CASE(sense_finds_the_car_ahead_in_lane) {
    std::vector<SensedBody> bodies = { body(Vec2(0, 12), Vec2(0, 1), 5.0) };
    LeaderSense l = senseLeader(coneAt(Vec2(0, 0), Vec2(0, 1)), bodies);
    CHECK(l.found);
    CHECK(std::fabs(l.gap - 12.0) < 1e-9);
    CHECK(std::fabs(l.speed - 5.0) < 1e-9);
}

TEST_CASE(sense_ignores_moving_cross_traffic) {
    // A car crossing perpendicular 15 m ahead: braking for it is the junction
    // deadlock the planner already solved — not a leader.
    std::vector<SensedBody> bodies = { body(Vec2(0, 15), Vec2(1, 0), 6.0) };
    CHECK(!senseLeader(coneAt(Vec2(0, 0), Vec2(0, 1)), bodies).found);
}

TEST_CASE(sense_stops_for_a_stationary_body_regardless_of_facing) {
    // A stalled car sideways across my lane (or a standing pedestrian) IS an
    // obstacle, whatever direction it points.
    std::vector<SensedBody> bodies = { body(Vec2(0.5, 10), Vec2(1, 0), 0.0) };
    LeaderSense l = senseLeader(coneAt(Vec2(0, 0), Vec2(0, 1)), bodies);
    CHECK(l.found);
    CHECK(l.speed == 0.0);
}

TEST_CASE(sense_ignores_the_adjacent_lane_and_itself) {
    std::vector<SensedBody> bodies = {
        body(Vec2(0, 0), Vec2(0, 1), 4.0),      // me (skip index 0)
        body(Vec2(3.5, 12), Vec2(0, 1), 4.0),   // parallel car one lane over
    };
    CHECK(!senseLeader(coneAt(Vec2(0, 0), Vec2(0, 1)), bodies,
                       /*lateralMax=*/2.2, /*stationarySpeed=*/1.0, /*skip=*/0).found);
}

TEST_CASE(sense_picks_the_nearest_of_several) {
    std::vector<SensedBody> bodies = {
        body(Vec2(0, 18), Vec2(0, 1), 7.0),
        body(Vec2(0, 9), Vec2(0, 1), 3.0),
    };
    LeaderSense l = senseLeader(coneAt(Vec2(0, 0), Vec2(0, 1)), bodies);
    CHECK(l.found);
    CHECK(std::fabs(l.gap - 9.0) < 1e-9);
    CHECK(std::fabs(l.speed - 3.0) < 1e-9);
}

TEST_CASE(follow_speed_ramps_to_zero_at_the_bumper) {
    LeaderSense l;
    l.found = true;
    l.halfLength = 2.1;
    // Far: free flow. Mid: slowed. At the bumper gap: zero.
    l.gap = 40; l.speed = 0;
    CHECK(followSpeed(10.0, l, 2.1) == 10.0);
    l.gap = 11;   // minGap 5.0 + half the 12 m slow zone
    Real mid = followSpeed(10.0, l, 2.1);
    CHECK(mid > 1.0 && mid < 9.0);
    l.gap = 5.0;
    CHECK(followSpeed(10.0, l, 2.1) == 0.0);
}

TEST_CASE(follow_speed_matches_a_moving_leader) {
    // Close behind a leader doing 6: don't compress to a crawl — flow at 6.
    LeaderSense l;
    l.found = true;
    l.halfLength = 2.1;
    l.gap = 7.0;
    l.speed = 6.0;
    Real cap = followSpeed(10.0, l, 2.1);
    CHECK(cap >= 6.0);
    CHECK(cap < 10.0);
}

TEST_CASE(stuck_detector_fires_after_a_sustained_stall_only) {
    // dt = 0.25 is exactly representable, so the 4.0 s threshold lands on a tick.
    StuckDetector sd;
    // Wants 5 m/s, moving 0: fires once after stallSeconds, then re-arms.
    bool fired = false;
    for (int i = 0; i < 15; ++i) fired = fired || sd.update(5.0, 0.0, 0.25);
    CHECK(!fired);                                  // 3.75 s: not yet
    CHECK(sd.update(5.0, 0.0, 0.25));               // 4.0 s: recover
    CHECK(!sd.update(5.0, 0.0, 0.25));              // re-armed, counting again
    // Motion (or a brain asking for a stop) resets the count.
    for (int i = 0; i < 14; ++i) sd.update(5.0, 0.0, 0.25);
    sd.update(5.0, 2.0, 0.25);                      // rolled forward
    for (int i = 0; i < 15; ++i) CHECK(!sd.update(5.0, 0.0, 0.25));
    sd.update(0.0, 0.0, 1000.0);                    // parked on purpose: never fires
    CHECK(sd.timer == 0.0);
}

TEST_CASE(corridor_sense_bends_with_the_road) {
    // A quarter-circle right bend (radius 20). The straight cone's freeze mode:
    // a STOPPED car in the ONCOMING lane of the curve falls dead ahead of the
    // straight axis and reads as a blocker. The corridor follows the path, so it
    // must ignore that car — and still find one actually parked ON my lane.
    std::vector<Vec2> path;
    for (int k = 0; k <= 20; ++k) {
        Real a = 1.5707963267948966 * k / 20.0;
        path.push_back(Vec2(20 - 20 * std::cos(a), 20 * std::sin(a)));
    }
    LaneFollower lf;
    lf.setPath(path);
    lf.update(Vec2(0, 0));   // car at the start of the bend, heading ~+z

    // Oncoming-lane car (3.5 m LEFT of my lane, i.e. outside the corridor),
    // stopped mid-bend — geometrically almost dead ahead of my straight axis.
    Vec2 midOn = path[10];
    Vec2 tan = path[11] - path[9];
    Real tl = std::sqrt(tan.x * tan.x + tan.y * tan.y);
    tan = Vec2(tan.x / tl, tan.y / tl);
    Vec2 left(-tan.y, tan.x);
    std::vector<SensedBody> oncoming = { body(midOn + left * 3.5, Vec2(-tan.x, -tan.y), 0.0) };
    CHECK(!senseAlongPath(lf, 25.0, 1.6, oncoming).found);
    // The same stopped car placed ON my lane path IS the leader, at path distance.
    std::vector<SensedBody> inLane = { body(midOn, tan, 0.0) };
    LeaderSense l = senseAlongPath(lf, 25.0, 1.6, inLane);
    CHECK(l.found);
    CHECK(l.gap > 10.0 && l.gap < 20.0);   // along-path distance to mid-bend
}

TEST_CASE(corridor_sense_flags_cross_bodies_and_yielders) {
    // Straight path; a stopped car sideways across the lane 10 m ahead: found,
    // align ~ 0 (a cross body — the anti-gridlock valve may creep past it), and
    // a pedestrian in the same spot is found but flagged yield-always (never
    // creep through a person).
    std::vector<Vec2> path;
    for (Real z = 0; z <= 60; z += 3) path.push_back(Vec2(0, z));
    LaneFollower lf;
    lf.setPath(path);
    lf.update(Vec2(0, 0));

    std::vector<SensedBody> crossCar = { body(Vec2(0, 10), Vec2(1, 0), 0.0) };
    LeaderSense l = senseAlongPath(lf, 25.0, 1.6, crossCar);
    CHECK(l.found);
    CHECK(l.align < 0.5);
    CHECK(!l.yieldAlways);

    SensedBody ped = body(Vec2(0, 10), Vec2(1, 0), 0.0, 0.3);
    ped.yieldAlways = true;
    std::vector<SensedBody> walker = { ped };
    LeaderSense p = senseAlongPath(lf, 25.0, 1.6, walker);
    CHECK(p.found);
    CHECK(p.yieldAlways);
    // Moving cross traffic: far-field still passes unhindered (no deadlock)...
    std::vector<SensedBody> crossingFar = { body(Vec2(0, 10), Vec2(1, 0), 6.0) };
    CHECK(!senseAlongPath(lf, 25.0, 1.6, crossingFar).found);
    // ...but a crosser about to occupy my path IS braked for (no T-bone).
    std::vector<SensedBody> crossingNear = { body(Vec2(0, 6), Vec2(1, 0), 6.0) };
    CHECK(senseAlongPath(lf, 25.0, 1.6, crossingNear).found);
}

TEST_CASE(knockdown_timer_floors_then_recovers) {
    KnockdownTimer k;
    CHECK(!k.down());
    k.knock();
    CHECK(k.down());
    // Down for its window (dt 0.25 is exactly representable)...
    for (int i = 0; i < 9; ++i) CHECK(k.update(0.25));    // 2.25 s: still down
    CHECK(!k.update(0.25));                               // 2.5 s: back up
    CHECK(!k.down());
    // A second hit while recovering re-floors it from the top.
    k.knock();
    k.update(0.25);
    k.knock();
    CHECK(k.timer > 2.0);
}

TEST_CASE(convoy_follower_never_rear_ends_its_leader) {
    // The whole stack closed-loop: leader cruises the lane at 4; the follower is
    // commanded 9 from 25 m back. Real-pose sensing must brake it to convoy pace
    // — centre gap never below the bumper distance, yet it keeps up.
    std::vector<Vec2> path;
    for (Real z = 0; z <= 600; z += 5) path.push_back(Vec2(0, z));

    LaneFollower lfLead, lfFollow;
    lfLead.setPath(path);
    lfFollow.setPath(path);
    BikeCar lead, follow;
    lead.pos = Vec2(0, 25);
    follow.pos = Vec2(0, 0);

    const Real dt = 0.05, halfLen = 2.1, bumper = 0.8;
    Real minGap = 1e9;
    for (int i = 0; i < 2000; ++i) {                     // 100 s
        // Leader: plain pursuit at 4 m/s.
        lfLead.update(lead.pos);
        DriverCommand lc = pursuitCommand(lfLead, lead.pos, 4.0, pursuitLookahead(lead.speed));
        lead.step(computeDriverInput(DriverState{lead.fwd, lead.speed}, lc), dt);

        // Follower: pursuit at 9, capped by what it SEES ahead.
        lfFollow.update(follow.pos);
        std::vector<SensedBody> bodies = { body(lead.pos, lead.fwd, lead.speed, halfLen) };
        LeaderSense sensed = senseLeader(coneAt(follow.pos, follow.fwd), bodies);
        Real target = followSpeed(9.0, sensed, halfLen, bumper);
        DriverCommand fc = pursuitCommand(lfFollow, follow.pos, target,
                                          pursuitLookahead(follow.speed));
        follow.step(computeDriverInput(DriverState{follow.fwd, follow.speed}, fc), dt);

        Vec2 d = lead.pos - follow.pos;
        minGap = std::min(minGap, d.length());
    }
    CHECK(minGap > (halfLen + halfLen + bumper) * 0.9);   // bodies never touched
    Real finalGap = (lead.pos - follow.pos).length();
    CHECK(finalGap < 30.0);                               // ...but it kept up (a convoy)
    CHECK(follow.speed > 3.0);                            // flowing at ~leader pace
}
