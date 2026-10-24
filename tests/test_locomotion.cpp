// The player's movement past walking (engine/locomotion.h), against a real Jolt world: the jump gate, ledges,
// the pull-up, the climbing probes.
#include "test_framework.h"

#include "../src/engine/locomotion.h"
#include "../src/engine/physics/physics_world.h"

#include <cmath>
#include <cstdio>

using namespace engine;

namespace {

constexpr Real kDt = 1.0 / 60.0;
constexpr Real kHalf = 0.4, kRadius = 0.3, kLift = kHalf + kRadius;   // the level player's capsule

void floor0(PhysicsWorld& w) { w.addBox(Vec3(50, 1, 50), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static); }

// a block whose near face is at x = x0, top at y = h, `depth` deep along +x
void block(PhysicsWorld& w, Real x0, Real h, Real depth = 3.0, Real y0 = 0.0) {
    w.addBox(Vec3(depth / 2, (h - y0) / 2, 3), Vec3(x0 + depth / 2, (h + y0) / 2, 0), Quat::identity(),
             BodyMotion::Static);
}

CharacterId standAt(PhysicsWorld& w, Real x) {
    CharacterId c = w.addCharacter(kHalf, kRadius, Vec3(x, kLift + 0.05, 0));
    w.optimizeBroadPhase();
    for (int i = 0; i < 30; i++) w.moveCharacter(c, Vec3(), kDt);
    return c;
}

// a slab tilted `deg` from level about z, rising toward +x: its upper face looks up and to -x
void slope(PhysicsWorld& w, Real deg) {
    const Real a = deg * 3.14159265358979 / 180.0;
    w.addBox(Vec3(4, 0.5, 3), Vec3(0, 2, 0), Quat::fromAxisAngle(Vec3(0, 0, 1), a), BodyMotion::Static);
}

}  // namespace

TEST_CASE(the_physics_jump_is_refused_on_steep_ground) {
    // The bug as found (Glenn: "jump sometimes gets me stuck on geometry"): on ground steeper than the 50 degree
    // walkable limit -- a rock face, a scree slope -- the grounded-only physics jump refused every press.
    PhysicsWorld w;
    w.initialize();
    floor0(w);
    slope(w, 58);
    CharacterId c = w.addCharacter(kHalf, kRadius, Vec3(-1.2, 4.5, 0));
    w.optimizeBroadPhase();
    int touching = 0, refused = 0;
    for (int i = 0; i < 90; i++) {
        w.moveCharacter(c, Vec3(), kDt);
        if (w.characterGroundState(c) == GroundState::OnSteepGround) {
            ++touching;
            if (!w.jumpCharacter(c, 4.3)) ++refused;
        }
    }
    CHECK(touching > 3);
    CHECK((refused) == (touching));
    w.shutdown();
}

TEST_CASE(the_jump_gate_jumps_off_steep_ground_and_away_from_it) {
    PhysicsWorld w;
    w.initialize();
    floor0(w);
    slope(w, 58);
    CharacterId c = w.addCharacter(kHalf, kRadius, Vec3(-1.2, 4.5, 0));
    w.optimizeBroadPhase();
    MovementTuning t;
    JumpGate gate;
    bool launched = false;
    Real yAt = 0, peak = 0;
    Vec3 push;
    for (int i = 0; i < 90 && !launched; i++) {
        const GroundState g = w.characterGroundState(c);
        if (g == GroundState::OnSteepGround) {
            gate.wantJump(t);
            const JumpGate::Launch l = gate.take(g, w.characterGroundNormal(c), t, kDt);
            if (l.go) {
                launched = true;
                push = l.push;
                yAt = w.characterPosition(c).y;
                w.launchCharacter(c, t.jumpSpeed);
                for (int k = 0; k < 20; k++) {
                    w.moveCharacter(c, push, kDt);
                    peak = std::max(peak, w.characterPosition(c).y);
                }
                break;
            }
        }
        w.moveCharacter(c, Vec3(), kDt);
        gate.observe(w.characterGroundState(c), w.characterVelocity(c).y, kDt);
    }
    CHECK(launched);
    CHECK(peak > yAt + 0.4);   // a real leap...
    CHECK(push.x < -1.0);      // ...away from the slope (it rises toward +x, its face looks to -x)
    w.shutdown();
}

TEST_CASE(the_jump_gate_allows_a_jump_just_after_running_off_an_edge) {
    for (Real late : {0.05, 0.3}) {
        PhysicsWorld w;
        w.initialize();
        w.addBox(Vec3(2, 1, 3), Vec3(0, -1, 0), Quat::identity(), BodyMotion::Static);   // a floor ending at x = 2
        CharacterId c = standAt(w, 1.0);
        MovementTuning t;
        JumpGate gate;
        Real offAt = -1, clock = 0;
        bool went = false;
        for (int i = 0; i < 120 && !went; i++) {
            if (offAt >= 0 && clock - offAt >= late) {
                gate.wantJump(t);
                went = gate.take(w.characterGroundState(c), w.characterGroundNormal(c), t, kDt).go;
                break;
            }
            w.moveCharacter(c, Vec3(3, 0, 0), kDt);
            gate.observe(w.characterGroundState(c), w.characterVelocity(c).y, kDt);
            clock += kDt;
            if (offAt < 0 && w.characterGroundState(c) == GroundState::InAir) offAt = clock;
        }
        CHECK(offAt >= 0);
        CHECK((went) == (late < t.coyoteTime));   // a moment late still jumps; much later is a fall
        w.shutdown();
    }
}

TEST_CASE(the_jump_gate_keeps_a_press_made_just_before_landing) {
    PhysicsWorld w;
    w.initialize();
    floor0(w);
    CharacterId c = w.addCharacter(kHalf, kRadius, Vec3(0, 2.5, 0));
    w.optimizeBroadPhase();
    MovementTuning t;
    JumpGate gate;
    bool pressed = false, went = false;
    for (int i = 0; i < 120 && !went; i++) {
        const Real feet = w.characterPosition(c).y - kLift;
        if (!pressed && feet < 0.15 && w.characterGroundState(c) == GroundState::InAir) {
            gate.wantJump(t);   // pressed a hair early...
            pressed = true;
            CHECK(!gate.take(w.characterGroundState(c), w.characterGroundNormal(c), t, kDt).go);
        } else if (pressed) {
            went = gate.take(w.characterGroundState(c), w.characterGroundNormal(c), t, kDt).go;
        }
        w.moveCharacter(c, Vec3(), kDt);
        gate.observe(w.characterGroundState(c), w.characterVelocity(c).y, kDt);
    }
    CHECK(pressed);
    CHECK(went);   // ...and it jumps on touching down
    w.shutdown();
}

TEST_CASE(a_ledge_in_reach_is_found_and_the_pull_up_lands_on_top) {
    for (Real h : {0.8, 1.2, 1.9}) {
        PhysicsWorld w;
        w.initialize();
        floor0(w);
        block(w, 2.0, h);
        CharacterId c = standAt(w, 2.0 - kRadius - 0.1);
        MovementTuning t;
        Ledge l;
        const Vec3 at = w.characterPosition(c);
        CHECK(findLedge(w, c, at, kHalf, kRadius, Vec3(1, 0, 0), t.mantleMaxHeight, t, l));
        CHECK_APPROX(l.topY, h, 0.02);
        CHECK_APPROX(l.normal.x, -1.0, 0.01);
        CHECK(l.stand.x > 2.0 + kRadius);   // standing on the top, clear of the edge
        MantleMove m;
        m.start(at, l, t);
        while (!m.done()) {
            m.t += kDt;
            const Vec3 v = (m.at(m.t) - w.characterPosition(c)) * (1.0 / kDt);
            w.moveCharacterFree(c, v, kDt);
        }
        for (int i = 0; i < 30; i++) w.moveCharacter(c, Vec3(), kDt);
        const Vec3 p = w.characterPosition(c);
        CHECK_APPROX(p.y, h + kLift, 0.08);
        CHECK(w.characterGroundState(c) == GroundState::OnGround);
        CHECK(m.total() < 1.0);   // a pull-up, not a crawl
        w.shutdown();
    }
}

TEST_CASE(a_ledge_is_refused_when_out_of_reach_or_with_no_room_on_top) {
    MovementTuning t;
    auto tryLedge = [&](auto build) {
        PhysicsWorld w;
        w.initialize();
        floor0(w);
        build(w);
        CharacterId c = standAt(w, 2.0 - kRadius - 0.1);
        Ledge l;
        const bool found = findLedge(w, c, w.characterPosition(c), kHalf, kRadius, Vec3(1, 0, 0),
                                     t.mantleMaxHeight, t, l);
        w.shutdown();
        return found;
    };
    CHECK(!tryLedge([](PhysicsWorld& w) { block(w, 2.0, 3.5); }));    // a wall, not a ledge
    CHECK(!tryLedge([](PhysicsWorld& w) { block(w, 2.0, 0.3); }));    // a step: the controller walks it
    CHECK(!tryLedge([](PhysicsWorld& w) { block(w, 2.0, 1.2, 0.15); }));   // a fence: no top to stand on
    CHECK(!tryLedge([](PhysicsWorld& w) {                               // a top with a slab 0.9 m over it
        block(w, 2.0, 1.2);
        block(w, 1.0, 2.4, 6.0, 2.1);
    }));
    CHECK(!tryLedge([](PhysicsWorld&) {}));                             // nothing there
    // and facing away from the ledge finds nothing either
    PhysicsWorld w;
    w.initialize();
    floor0(w);
    block(w, 2.0, 1.2);
    CharacterId c = standAt(w, 2.0 - kRadius - 0.1);
    Ledge l;
    CHECK(!findLedge(w, c, w.characterPosition(c), kHalf, kRadius, Vec3(-1, 0, 0), t.mantleMaxHeight, t, l));
    w.shutdown();
}

TEST_CASE(a_wall_is_climbable_where_it_is_too_steep_to_walk) {
    MovementTuning t;
    for (Real deg : {40.0, 60.0, 90.0}) {
        PhysicsWorld w;
        w.initialize();
        floor0(w);
        slope(w, deg);
        w.optimizeBroadPhase();
        // a ray in from the open side (-x), at the slab's mid height
        const WallHit hit = probeWall(w, Vec3(-4, 2, 0), Vec3(1, 0, 0), 6, t);
        CHECK(hit.hit);
        CHECK((hit.climbable) == (deg > 51));
        Vec3 right, up;
        wallBasis(hit.normal, right, up);
        CHECK_APPROX(dot(right, hit.normal), 0.0, 1e-6);
        CHECK_APPROX(dot(up, hit.normal), 0.0, 1e-6);
        CHECK(up.y > 0);
        w.shutdown();
    }
}

TEST_CASE(a_swim_stroke_surges_and_averages_out) {
    for (Real surge : {0.0, 0.55}) {
        Real sum = 0, lo = 9, hi = 0;
        for (int i = 0; i < 1000; i++) {
            const Real s = strokeSpeedScale(i / 1000.0, surge);
            sum += s;
            lo = std::min(lo, s);
            hi = std::max(hi, s);
        }
        CHECK_APPROX(sum / 1000.0, 1.0, 1e-6);
        CHECK_APPROX(hi - lo, 2 * surge, 1e-3);
    }
}

TEST_CASE(a_plunge_goes_under_and_bobs_back_up) {
    // Glenn, 2026-10-24: "when I jump in I don't dip underwater and bob up ... There's no bouyancy". Off the 3 m
    // board (~7.7 m/s at the water) the body goes well under, comes back up past the float line once, and settles.
    MovementTuning t;
    const Real floatY = 0.0;
    Real y = 0.0, vy = -7.7, deepest = 0, overshoot = -1e9;
    bool cameUp = false;
    for (int i = 0; i < 6 * 60; i++) {
        vy = swimVerticalStep(vy, y, floatY, false, false, kDt, t);
        y += vy * kDt;
        deepest = std::min(deepest, y);
        if (vy > 0) cameUp = true;
        if (cameUp) overshoot = std::max(overshoot, y);
    }
    CHECK(deepest < -1.0);              // under, not stopped dead at the surface
    CHECK(deepest > -3.0);              // ...but not to the bottom of a pool
    CHECK(overshoot > floatY + 0.03);   // a bob up past where it floats
    CHECK(std::fabs(y - floatY) < 0.03);   // and at rest after a few seconds
    CHECK(std::fabs(vy) < 0.05);
    // a slow step in barely dips
    y = 0; vy = -1.0; deepest = 0;
    for (int i = 0; i < 120; i++) {
        vy = swimVerticalStep(vy, y, floatY, false, false, kDt, t);
        y += vy * kDt;
        deepest = std::min(deepest, y);
    }
    CHECK(deepest > -0.4);
}
