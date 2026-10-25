// The player's movement past walking (engine/locomotion.h), against a real Jolt world: the jump gate, ledges,
// the pull-up, the climbing probes.
#include "test_framework.h"

#include "../src/engine/block_body.h"
#include "../src/engine/locomotion.h"
#include "../src/engine/physics/physics_world.h"
#include "../src/engine/rope_line.h"

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

TEST_CASE(a_wingsuit_glides_pitched_down_and_stalls_pulled_up) {
    MovementTuning t;
    constexpr Real kDeg = 3.14159265358979 / 180.0;
    // pitched 20 degrees down from a standing start: it picks up speed and settles at a glide
    const Vec3 down20(0, -std::sin(20 * kDeg), -std::cos(20 * kDeg));
    Vec3 v(0, 0, 0), p(0, 1000, 0);
    for (int i = 0; i < 40 * 60; i++) {
        v = wingsuitStep(v, down20, kDt, t);
        p = p + v * kDt;
    }
    const Real speed = v.length();
    const Real ratio = std::hypot(v.x, v.z) / -v.y;
    CHECK(speed > 40 && speed < 56);   // ~48 m/s, about 170 km/h
    CHECK(ratio > 2.1 && ratio < 3.3); // a wingsuit's 2.5-3 : 1 (2.35 measured)
    // pulled up from that: it climbs for a while, then runs out of speed and drops
    const Vec3 up20(0, std::sin(20 * kDeg), -std::cos(20 * kDeg));
    const Real y0 = p.y, v0 = v.length();
    Real top = y0, slowest = 1e9;
    for (int i = 0; i < 20 * 60; i++) {
        v = wingsuitStep(v, up20, kDt, t);
        p = p + v * kDt;
        top = std::max(top, p.y);
        slowest = std::min(slowest, v.length());
    }
    CHECK(top > y0 + 5.0);                 // the swoop trades speed for height
    CHECK(slowest < v0 * 0.5);             // ...paid for in speed
    CHECK(p.y < top - 1.0);                // and then it mushes and sinks again
    // slow and pulled up, it can't hang there: the nose drops and it dives for speed
    Vec3 slow(0, 0, -12), q(0, 1000, 0);
    Real minY = 1e9;
    for (int i = 0; i < 6 * 60; i++) {
        slow = wingsuitStep(slow, up20, kDt, t);
        q = q + slow * kDt;
    }
    minY = q.y;
    CHECK(minY < 1000 - 15.0);   // (measured ~20 m in 6 s, where it used to hover)
    // the look's pitch is held to the suit's range
    const Vec3 straightDown = wingsuitFacing(Vec3(0.001, -1, 0), t);
    CHECK(straightDown.y > -std::sin(t.suitPitchDown * kDeg) - 1e-6);
}

TEST_CASE(a_parachute_settles_to_its_glide_and_answers_the_toggles) {
    MovementTuning t;
    auto settle = [&](Real inF) {
        Vec3 v(0, -25, 0), heading(0, 0, -1);
        for (int i = 0; i < 12 * 60; i++) v = parachuteStep(v, heading, Vec3(0, 0, -1), inF, 1.0, kDt, t);
        return v;
    };
    const Vec3 hands = settle(0), dive = settle(1), flare = settle(-1);
    CHECK_APPROX(-hands.y, t.chuteSink, 0.1);
    CHECK_APPROX(std::hypot(hands.x, hands.z), t.chuteSpeed, 0.1);
    CHECK(-dive.y > -hands.y + 1.0);
    CHECK(-flare.y < 2.5);
    // it turns toward the look at no more than its turn rate
    Vec3 v(0, -4.5, -8), heading(0, 0, -1);
    for (int i = 0; i < 30; i++) v = parachuteStep(v, heading, Vec3(1, 0, 0), 0, 1.0, kDt, t);
    const Real turned = std::atan2(heading.x, -heading.z) * 180.0 / 3.14159265358979;
    CHECK_APPROX(turned, t.chuteTurnRate * 0.5, 1.5);
    // an opening canopy is still mostly a fall
    Vec3 opening(0, -10, 0), h2(0, 0, -1);
    opening = parachuteStep(opening, h2, Vec3(0, 0, -1), 0, 0.05, kDt, t);
    CHECK(opening.y < -10.0);
}

TEST_CASE(a_rope_swings_like_a_pendulum_without_gaining_height) {
    // From level with the anchor, 10 m out: at the bottom the speed is sqrt(2 g L), and it never climbs above
    // where it started -- energy from nothing is what broke the wingsuit's first three models.
    const Vec3 anchor(0, 20, 0);
    const Real L = 10.0;
    Vec3 p(10, 20, 0), v(0, 0, 0);
    Real fastest = 0, highestAfter = -1e9;
    for (int i = 0; i < 6 * 60; i++) {
        v = ropeStep(p, v, anchor, L, Vec3(0, 0, 0), kDt);
        fastest = std::max(fastest, v.length());
        if (i > 60) highestAfter = std::max(highestAfter, p.y);
        CHECK((p - anchor).length() <= L + 1e-6);   // never past the rope
    }
    CHECK_APPROX(fastest, std::sqrt(2 * 9.81 * L), 0.6);
    CHECK(highestAfter <= 20.0 + 0.05);
    // slack rope: a body inside the length falls freely
    Vec3 q(0, 15, 0), w(0, 0, 0);
    w = ropeStep(q, w, anchor, L, Vec3(0, 0, 0), kDt);
    CHECK_APPROX(w.y, -9.81 * kDt, 1e-9);
}

TEST_CASE(a_rope_that_gives_catches_a_fall_with_a_bounce_and_no_energy_from_nothing) {
    // Glenn, 2026-10-24: "make the grapple a little more rope like". 5 m of slack, then the rope catches: it gives a
    // little, bounces the body back up, and settles where the weight stretches it -- never back up to the start.
    const Vec3 anchor(0, 20, 0);
    const Real L = 10.0, stretch = 0.02;
    Vec3 p(0, 15, 0), v(0, 0, 0);
    Real lowest = 1e9, highestAfter = -1e9;
    bool caught = false, bounced = false;
    for (int i = 0; i < 8 * 60; i++) {
        v = ropeStep(p, v, anchor, L, Vec3(0, 0, 0), kDt, stretch, 0.4);
        lowest = std::min(lowest, p.y);
        if (p.y < 10.0) caught = true;
        if (caught && v.y > 0.5) bounced = true;
        if (caught) highestAfter = std::max(highestAfter, p.y);
    }
    std::printf("    [rope give] lowest %.2f m below the anchor, settled at %.3f\n", 20 - lowest, 20 - p.y);
    CHECK(bounced);
    CHECK(20 - lowest > L * (1 + stretch));           // it gave more than the weight alone stretches it
    CHECK(20 - lowest <= L * (1 + 4 * stretch) + 1e-6);   // ...but never past its limit
    CHECK(highestAfter < 15.0);                        // the bounce never gets back to where it fell from
    CHECK_APPROX(20 - p.y, L * (1 + stretch), 0.03);   // hanging still: the weight's stretch
    CHECK(v.length() < 0.1);
    // a swing on it gains nothing either
    Vec3 q(10, 20, 0), w(0, 0, 0);
    Real top = -1e9;
    for (int i = 0; i < 6 * 60; i++) {
        w = ropeStep(q, w, anchor, L, Vec3(0, 0, 0), kDt, stretch, 0.4);
        if (i > 60) top = std::max(top, q.y);
    }
    CHECK(top <= 20.0 + 0.05);
}

TEST_CASE(the_drawn_rope_hangs_slack_runs_straight_taut_and_lies_over_a_ledge) {
    RopeLine r;
    const Vec3 hand(0, 0, 0), hook(10, 0, 0);
    // slack (12 m between ends 10 m apart): it sags in the middle
    r.reset(hand);
    for (int i = 0; i < 300; i++) r.step(hand, hook, false, 12.0, kDt, nullptr);
    Real sag = 0;
    for (const Vec3& q : r.points()) sag = std::min(sag, q.y);
    // taut (rope as long as the span): straight
    RopeLine t;
    t.reset(hand);
    for (int i = 0; i < 300; i++) t.step(hand, hook, false, 10.0, kDt, nullptr);
    Real bow = 0;
    for (const Vec3& q : t.points()) bow = std::max(bow, std::fabs(q.y));
    std::printf("    [rope line] slack sags %.2f m, taut bows %.3f m\n", -sag, bow);
    CHECK(sag < -1.5);
    CHECK(bow < 0.05);
    // the ends are where they were pinned
    CHECK((t.points().front() - hand).length() < 1e-9);
    CHECK((t.points().back() - hook).length() < 1e-9);
    // a slack rope from a hand on a roof (a building x < 2, y < 0) to a hook down past the edge: it lies on the
    // roof and over the edge, never through it
    const RopeLine::Sweep roof = [](const Vec3& a, const Vec3& b, Vec3& hit, Vec3& n) {
        if (a.y >= 0 && b.y < 0) {   // down through the roof
            const Vec3 at = a + (b - a) * (a.y / (a.y - b.y));
            if (at.x <= 2.0) { hit = at; n = Vec3(0, 1, 0); return true; }
        }
        if (a.x >= 2 && b.x < 2) {   // in through the wall
            const Vec3 at = a + (b - a) * ((a.x - 2) / (a.x - b.x));
            if (at.y <= 0) { hit = at; n = Vec3(1, 0, 0); return true; }
        }
        return false;
    };
    RopeLine e;
    e.reset(Vec3(0, 0.5, 0));
    for (int i = 0; i < 400; i++) e.step(Vec3(0, 0.5, 0), Vec3(4, -4, 0), false, 9.0, kDt, roof);
    int onRoof = 0, through = 0;
    for (const Vec3& q : e.points()) {
        if (q.x < 2.0 && q.y < -0.01) ++through;
        if (q.x > 0.2 && q.x < 2.0 && q.y < 0.1) ++onRoof;
    }
    std::printf("    [rope line] %d points lying on the roof, %d through it\n", onRoof, through);
    CHECK(onRoof >= 2);
    CHECK(through == 0);
    // let go: the free end comes back as the rope is reeled in
    RopeLine l;
    l.reset(hand);
    for (int i = 0; i < 60; i++) l.step(hand, hook, false, 10.0, kDt, nullptr);
    Real len = 10.0;
    for (int i = 0; i < 40; i++) {
        len = std::max(Real(0.2), len - 35.0 * kDt);
        l.step(hand, Vec3(), true, len, kDt, nullptr);
    }
    CHECK((l.points().back() - hand).length() < 0.5);
}

TEST_CASE(the_grapple_hook_snaps_to_a_beam_just_off_the_aim) {
    PhysicsWorld w;
    w.initialize();
    floor0(w);
    // a beam overhead, 15 m ahead, 10 m up
    w.addBox(Vec3(5, 0.3, 0.3), Vec3(0, 10, -15), Quat::identity(), BodyMotion::Static);
    w.optimizeBroadPhase();
    MovementTuning t;
    const Vec3 eye(0, 1.5, 0);
    const Vec3 atBeam = normalize(Vec3(0, 8.5, -15));
    const GrappleAnchor straight = findGrappleAnchor(w, eye, atBeam, t);
    CHECK(straight.ok);
    CHECK_APPROX(straight.point.y, 10, 0.4);
    // aimed 5 degrees high, over the beam, the ring still catches it
    const Real a = 5.0 * 3.14159265358979 / 180.0;
    const Vec3 high = normalize(atBeam * std::cos(a) + Vec3(0, 1, 0) * std::sin(a));
    const GrappleAnchor snapped = findGrappleAnchor(w, eye, high, t);
    CHECK(snapped.ok);
    CHECK_APPROX(snapped.point.y, 10, 0.4);
    // aimed at the open sky, nothing
    CHECK(!findGrappleAnchor(w, eye, normalize(Vec3(1, 1, 1)), t).ok);
    w.shutdown();
}

TEST_CASE(the_block_body_at_rest_is_the_walker_mesh) {
    // The ten blocks posed standing straight sit where buildStandingPerson welds its six (procgen/person.cpp): each
    // limb's two halves end to end make its one block.
    BodyPose pose;
    const std::array<PartTransform, BlockBody::kParts> b = poseBlocks(pose);
    CHECK_APPROX(b[BlockBody::Torso].position.y, 0.25, 1e-9);
    CHECK_APPROX(b[BlockBody::Head].position.y, 0.70, 1e-9);
    CHECK_APPROX((b[BlockBody::ArmL].position.y + b[BlockBody::ForearmL].position.y) * 0.5, 0.19, 1e-9);
    CHECK_APPROX(std::fabs(b[BlockBody::ArmL].position.x), 0.29, 1e-9);
    CHECK_APPROX(std::fabs(b[BlockBody::ForearmL].position.x), 0.29, 1e-9);
    CHECK_APPROX(b[BlockBody::ShinR].position.y - b[BlockBody::ShinR].size.y * 0.5, -0.9, 1e-9);   // the sole
    CHECK_APPROX(b[BlockBody::LegR].position.y + b[BlockBody::LegR].size.y * 0.5, -0.05, 1e-9);    // the hip
    for (int k = 0; k < BlockBody::kLimbs; ++k)   // each lower half starts where its upper one ends
        CHECK((endOf(BlockBody::upper(k), b[BlockBody::upper(k)], 1.0) -
               jointOf(BlockBody::lower(k), b[BlockBody::lower(k)], 1.0)).length() < 1e-9);
    // a limb reached at a point points at it from its joint
    pose.position = Vec3(3, 1, 2);
    pose.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), 0.7);
    const Vec3 target(3.5, 3, 2.4);
    pose.armR = reachToward(pose, BlockBody::ArmR, target);
    const std::array<PartTransform, BlockBody::kParts> c = poseBlocks(pose);
    const Vec3 shoulder = jointOf(BlockBody::ArmR, c[BlockBody::ArmR], 1.0);
    const Vec3 toCentre = normalize(c[BlockBody::ArmR].position - shoulder);
    CHECK_APPROX(dot(toCentre, normalize(target - shoulder)), 1.0, 1e-6);
}

TEST_CASE(a_limb_reaches_a_hold_by_bending_and_folds_the_right_way) {
    BodyPose pose;
    pose.position = Vec3(1, 2, -3);
    pose.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), -0.4);
    const Vec3 fwd = pose.orientation.rotate(Vec3(0, 0, 1)), up(0, 1, 0);
    // a hold in front of the right shoulder, closer than the straight arm: the hand lands on it, the elbow bends
    const LimbSolve straight = solveLimb(pose, 1);
    const Vec3 hold = straight.root + fwd * 0.3 + up * 0.25;
    reachTo(pose, BlockBody::ArmR, hold);
    CHECK(pose.elbowR > 0.5 && pose.elbowR < kElbowMax);
    const std::array<PartTransform, BlockBody::kParts> b = poseBlocks(pose);
    CHECK((endOf(BlockBody::ForearmR, b[BlockBody::ForearmR], 1.0) - hold).length() < 1e-6);
    CHECK((endOf(BlockBody::ArmR, b[BlockBody::ArmR], 1.0) - jointOf(BlockBody::ForearmR, b[BlockBody::ForearmR], 1.0))
              .length() < 1e-6);   // one limb: the forearm starts at the elbow
    const LimbSolve arm = solveLimb(pose, 1);
    CHECK(dot(arm.mid - (arm.root + arm.end) * 0.5, fwd) < 0);   // the point of the elbow is behind the line
    // out of reach: straight at it, short of it
    BodyPose far = pose;
    reachTo(far, BlockBody::ArmL, solveLimb(far, 0).root + fwd * 3.0);
    CHECK(far.elbowL < 1e-3);
    // a knee bent: its kneecap out in front of the hip-to-foot line, the foot drawn up toward the hip
    BodyPose sit;
    sit.kneeL = 1.4;
    const LimbSolve leg = solveLimb(sit, 2);
    CHECK(dot(leg.mid - (leg.root + leg.end) * 0.5, Vec3(0, 0, 1)) > 0.1);
    CHECK((leg.end - leg.root).length() < 0.85 * 0.8);
    // a thigh straight out in front (sat down) bends its knee up, not sideways
    sit.legL = Vec3(0, 0, 1);
    const LimbSolve out = solveLimb(sit, 2);
    CHECK(out.mid.y > out.root.y + 0.1);
    // the hinge axis turns the lower half further round: the fold grows
    const Real fold = std::acos(dot(out.upperDir, out.lowerDir));
    const Vec3 turned = Quat::fromAxisAngle(out.hingeAxis, 0.1).rotate(out.lowerDir);
    CHECK(std::acos(dot(out.upperDir, turned)) > fold);
}

TEST_CASE(a_skinned_limb_covers_its_blocks_and_keeps_its_thickness_round_the_bend) {
    for (int limb : {1, 2}) {   // an arm and a leg
        const BlockBody::Part up = BlockBody::upper(limb), lo = BlockBody::lower(limb);
        const Vec3 su = BlockBody::parts()[up].size;
        const Real a = su.y, b = BlockBody::parts()[lo].size.y;
        // straight: the two blocks' box exactly
        const RenderMesh s0 = skinnedLimb(limb, 0.0, Vec3(1, 1, 1));
        Vec3 lo0(1e9, 1e9, 1e9), hi0(-1e9, -1e9, -1e9);
        for (const Vertex& v : s0.vertices) {
            lo0 = Vec3(std::min(lo0.x, v.position.x), std::min(lo0.y, v.position.y), std::min(lo0.z, v.position.z));
            hi0 = Vec3(std::max(hi0.x, v.position.x), std::max(hi0.y, v.position.y), std::max(hi0.z, v.position.z));
        }
        CHECK_APPROX(hi0.y, 0.0, 1e-9);
        CHECK_APPROX(lo0.y, -(a + b), 1e-9);
        CHECK_APPROX(hi0.x - lo0.x, su.x, 1e-9);
        CHECK_APPROX(hi0.z - lo0.z, su.z, 1e-9);
        // bent 100 degrees: the far end lands where poseBlocks puts the hand or foot, and the outside of the bend keeps
        // its thickness -- the skin's furthest reach out along the bend's bisector is the half depth (blending the
        // corners would pull it in to half depth x cos(50 deg))
        const Real th = 100.0 * 3.14159265358979 / 180.0;
        const RenderMesh s1 = skinnedLimb(limb, th, Vec3(1, 1, 1));
        const Vec3 joint(0, -a, 0);
        const Vec3 upDir(0, 1, 0), downDir(0, -std::cos(th), -std::sin(th));   // to the root, to the end
        const Vec3 outward = normalize((upDir + downDir) * -1.0);
        Real reach = -1e9, endErr = 1e9;
        const Vec3 end = joint + downDir * b;
        for (const Vertex& v : s1.vertices) {
            reach = std::max(reach, dot(v.position - joint, outward));
            endErr = std::min(endErr, (v.position - end).length());
        }
        CHECK(reach > 0.97 * su.z * 0.5);
        CHECK(endErr < std::max(su.x, su.z) * 0.75);   // a cap corner by the end's centre
        // the same tube placed by a pose bent that far: the end at the pose's hand or foot
        BodyPose pose;
        pose.bend(limb) = th;
        const std::array<PartTransform, BlockBody::kParts> blocks = poseBlocks(pose);
        CHECK_APPROX(foldOf(blocks[up].orientation, blocks[lo].orientation), th, 1e-6);
        const Vec3 placedEnd = jointOf(up, blocks[up], 1.0) + blocks[up].orientation.rotate(end);
        CHECK((placedEnd - endOf(lo, blocks[lo], 1.0)).length() < 1e-6);
    }
}

TEST_CASE(a_ragdoll_falls_in_a_heap_and_holds_together) {
    PhysicsWorld w;
    w.initialize();
    floor0(w);
    w.optimizeBroadPhase();
    BodyPose pose;
    pose.position = Vec3(0, 2.0, 0);
    pose.scale = 0.78;   // the level player's 1.4 m capsule
    const std::vector<PhysicsWorld::RagdollPart> parts = blockRagdoll(pose);
    CHECK(parts.size() == BlockBody::kParts);
    // thrown forward at a run
    const PhysicsWorld::RagdollId r = w.addRagdoll(parts, Vec3(0, 0, 6));
    CHECK(r != PhysicsWorld::INVALID_RAGDOLL);
    for (int i = 0; i < 5 * 60; i++) w.update(kDt);
    Real top = -1e9;
    std::array<PartTransform, BlockBody::kParts> now;
    for (int i = 0; i < BlockBody::kParts; ++i) {
        CHECK(w.ragdollPart(r, i, now[i].position, now[i].orientation));
        top = std::max(top, now[i].position.y);
        CHECK(std::isfinite(now[i].position.x) && std::isfinite(now[i].position.y));
    }
    CHECK(top < 0.6);                      // down on the ground, not standing
    CHECK(now[BlockBody::Torso].position.z > 1.0);   // it went the way it was thrown
    CHECK(w.ragdollSpeed(r) < 0.3);        // and has come to rest
    // every joint still joined: the child's end meets the parent's socket (on the torso; a forearm or shin at the
    // end of its upper half)
    for (int i = 1; i < BlockBody::kParts; ++i) {
        const Vec3 childEnd = jointOf(static_cast<BlockBody::Part>(i), now[i], pose.scale);
        const BlockBody::PartSpec& spec = BlockBody::parts()[static_cast<std::size_t>(i)];
        const Vec3 socket =
            BlockBody::isLower(i)
                ? endOf(static_cast<BlockBody::Part>(spec.parent), now[static_cast<std::size_t>(spec.parent)], pose.scale)
                : now[BlockBody::Torso].position +
                      now[BlockBody::Torso].orientation.rotate((spec.joint - Vec3(0, 0.25, 0)) * pose.scale);
        CHECK((childEnd - socket).length() < 0.08);
    }
    // the knees and elbows folded only their own way, and no further than they go: the hinge's axis carried round
    // by the upper half (from where the ragdoll was made) still turns the lower half further round
    const std::array<PartTransform, BlockBody::kParts> made = poseBlocks(pose);
    for (int k = 0; k < BlockBody::kLimbs; ++k) {
        const PartTransform& u = now[BlockBody::upper(k)];
        const PartTransform& l = now[BlockBody::lower(k)];
        const Vec3 ud = u.orientation.rotate(Vec3(0, -1, 0)), ld = l.orientation.rotate(Vec3(0, -1, 0));
        const Real fold = std::acos(std::clamp(dot(ud, ld), Real(-1), Real(1)));
        CHECK(fold < (k >= 2 ? kKneeMax : kElbowMax) + 0.1);
        const Quat carried = u.orientation * made[BlockBody::upper(k)].orientation.conjugate();
        const Vec3 axis = carried.rotate(parts[BlockBody::lower(k)].hingeAxis);
        if (fold > 0.1) CHECK(dot(cross(ud, ld), axis) > 0);   // folded forward of straight, never back past it
    }
    w.removeRagdoll(r);
    CHECK(!w.ragdollPart(r, 0, now[0].position, now[0].orientation));
    w.shutdown();
}

TEST_CASE(a_board_kicks_up_to_speed_carves_rolls_downhill_and_stops) {
    MovementTuning t;
    const Vec3 flatGround(0, 1, 0);
    Vec3 h(0, 0, -1);
    Real kick = 0, v = 0;
    for (int i = 0; i < 10 * 60; i++) v = boardStep(v, h, kick, 1, 0, flatGround, t.boardPushMax, kDt, t);
    CHECK(v > t.boardPushMax - 1.0 && v <= t.boardPushMax);   // kicked up to cruising speed
    CHECK_APPROX(h.z, -1.0, 1e-9);                           // straight on, no input
    // carving right at speed: the heading swings clockwise from above (toward +x from -z)
    for (int i = 0; i < 30; i++) v = boardStep(v, h, kick, 0, 1, flatGround, t.boardPushMax, kDt, t);
    CHECK(h.x > 0.3);
    // a faster board turns wider: the turn rate is the grip over the speed
    Vec3 slowH(0, 0, -1), fastH(0, 0, -1);
    Real k1 = 1, k2 = 1;
    boardStep(2.0, slowH, k1, 0, 1, flatGround, 99, kDt, t);
    boardStep(8.0, fastH, k2, 0, 1, flatGround, 99, kDt, t);
    CHECK(slowH.x > fastH.x);
    // downhill it gathers speed without a kick; uphill it slows
    const Real a = 8 * 3.14159265358979 / 180;
    const Vec3 slopeDownNorth(0, std::cos(a), -std::sin(a));   // ground falling away to -z
    Vec3 hd(0, 0, -1);
    Real kd = 1, vd = 2;
    for (int i = 0; i < 120; i++) vd = boardStep(vd, hd, kd, 0, 0, slopeDownNorth, 99, kDt, t);
    CHECK(vd > 2.5);
    // and with nothing pushing, it rolls to a stop; braking stops it sooner
    Real coast = 5, brake = 5, kc = 1, kb = 1;
    Vec3 hc(0, 0, -1), hb(0, 0, -1);
    int coastT = 0, brakeT = 0;
    while (coast > 0 && coastT < 6000) { coast = boardStep(coast, hc, kc, 0, 0, flatGround, 99, kDt, t); ++coastT; }
    while (brake > 0 && brakeT < 6000) { brake = boardStep(brake, hb, kb, -1, 0, flatGround, 99, kDt, t); ++brakeT; }
    CHECK(brakeT < coastT / 4);
}

TEST_CASE(a_swing_pumped_in_time_goes_higher_and_dies_down_left_alone) {
    MovementTuning t;
    constexpr Real kDeg = 3.14159265358979 / 180.0;
    // pumped from nearly still, W held: in 10 s it swings high
    Real th = 0.05, om = 0, top = 0;
    for (int i = 0; i < 10 * 60; i++) {
        swingStep(th, om, 2.0, 1.0, kDt, t);
        top = std::max(top, std::fabs(th));
    }
    CHECK(top > 45 * kDeg);
    CHECK(top <= t.swingMaxDeg * kDeg + 1e-9);
    // left alone it dies down
    Real late = 0;
    for (int i = 0; i < 40 * 60; i++) {
        swingStep(th, om, 2.0, 0.0, kDt, t);
        if (i > 35 * 60) late = std::max(late, std::fabs(th));
    }
    CHECK(late < top * 0.3);
    // pumping against it (S held) brakes it sooner than coasting
    Real a1 = 0.6, w1 = 0, a2 = 0.6, w2 = 0;
    Real m1 = 0, m2 = 0;
    for (int i = 0; i < 6 * 60; i++) {
        swingStep(a1, w1, 2.0, 0.0, kDt, t);
        swingStep(a2, w2, 2.0, -1.0, kDt, t);
        if (i > 4 * 60) { m1 = std::max(m1, std::fabs(a1)); m2 = std::max(m2, std::fabs(a2)); }
    }
    CHECK(m2 < m1 * 0.5);
}
