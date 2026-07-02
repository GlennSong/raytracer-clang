#include "test_framework.h"

#include "../src/engine/ai/agent_memory.h"
#include "../src/engine/ai/commitment.h"

#include <cmath>
#include <vector>

using namespace engine;

// The cognition core (ADR-0062: sense -> remember -> predict -> decide -> act).
// These pin the three pure pieces the loop stands on: working MEMORY (tracks
// with velocity estimates, object permanence, decay), PREDICTION (closest
// approach / time-to-collision), and decision COMMITMENT (hysteresis: choices
// hold; near-ties never flap).

// --- remember ---------------------------------------------------------------

TEST_CASE(memory_estimates_a_tracked_bodys_velocity) {
    AgentMemory mem;
    // A body walking +x at 2 m/s, sighted every 0.35 s (think cadence).
    for (int i = 0; i <= 8; ++i)
        mem.observe(7, Vec2(2.0 * 0.35 * i, 5.0), 0.35 * i);
    const TrackedBody* t = mem.track(7);
    CHECK(t != nullptr);
    CHECK(std::fabs(t->vel.x - 2.0) < 0.05);   // converged on the true velocity
    CHECK(std::fabs(t->vel.y) < 0.05);
    CHECK(t->confidence == 1.0);
}

TEST_CASE(memory_extrapolates_unseen_bodies_then_forgets) {
    AgentMemory mem;
    mem.memorySeconds = 2.0;
    for (int i = 0; i <= 6; ++i)
        mem.observe(1, Vec2(3.0 * 0.25 * i, 0.0), 0.25 * i);   // 3 m/s +x
    Real lastSight = 0.25 * 6;

    // One second unseen: still remembered, position projected ahead (object
    // permanence — the agent expects it where it was heading).
    mem.update(lastSight + 1.0);
    const TrackedBody* t = mem.track(1);
    CHECK(t != nullptr);
    CHECK(t->confidence > 0.4 && t->confidence < 0.6);
    CHECK(std::fabs(t->pos.x - (3.0 * 0.25 * 6 + 3.0 * 1.0)) < 0.2);

    // Past the memory horizon: forgotten entirely.
    mem.update(lastSight + 2.1);
    CHECK(mem.track(1) == nullptr);
}

TEST_CASE(memory_resighting_reanchors_a_faded_track) {
    AgentMemory mem;
    mem.observe(4, Vec2(0, 0), 0.0);
    mem.observe(4, Vec2(1, 0), 0.5);
    mem.update(2.0);                       // faded, extrapolated
    mem.observe(4, Vec2(9, 9), 2.5);       // seen again somewhere else
    mem.update(2.5);
    const TrackedBody* t = mem.track(4);
    CHECK(t != nullptr);
    CHECK(t->confidence == 1.0);
    CHECK(std::fabs(t->pos.x - 9.0) < 1e-9);
}

TEST_CASE(memory_caps_tracks_by_evicting_the_weakest) {
    AgentMemory mem;
    mem.maxTracks = 4;
    // Four tracks; let #0 fade the most, then a fifth body appears.
    for (int id = 0; id < 4; ++id) mem.observe(id, Vec2(id, 0), 0.1 * id);
    mem.update(1.0);
    mem.observe(99, Vec2(50, 0), 1.0);
    CHECK(mem.tracks().size() == 4u);
    CHECK(mem.track(99) != nullptr);   // newcomer admitted
    CHECK(mem.track(0) == nullptr);    // stalest memory paid for it
}

// --- predict ----------------------------------------------------------------

TEST_CASE(closest_approach_and_ttc_for_head_on_bodies) {
    // Two bodies 20 m apart closing at a combined 10 m/s: nearest (touching) at
    // 2 s; with 1 m combined radius they collide at (20-1)/10 = 1.9 s.
    Approach a = closestApproach(Vec2(0, 0), Vec2(5, 0), Vec2(20, 0), Vec2(-5, 0));
    CHECK(std::fabs(a.time - 2.0) < 1e-9);
    CHECK(a.distance < 1e-9);
    Real ttc = timeToCollision(Vec2(0, 0), Vec2(5, 0), Vec2(20, 0), Vec2(-5, 0), 1.0);
    CHECK(std::fabs(ttc - 1.9) < 1e-9);
}

TEST_CASE(prediction_knows_a_miss_from_a_hit) {
    // Crossing paths that miss by 4 m: a real closest approach, but no collision.
    Approach a = closestApproach(Vec2(0, 0), Vec2(4, 0), Vec2(10, 4), Vec2(0, 0));
    CHECK(std::fabs(a.distance - 4.0) < 1e-9);
    CHECK(timeToCollision(Vec2(0, 0), Vec2(4, 0), Vec2(10, 4), Vec2(0, 0), 1.0) >
          1e9);   // never
    // Diverging bodies are closest NOW; overlap means TTC zero.
    Approach d = closestApproach(Vec2(0, 0), Vec2(-1, 0), Vec2(5, 0), Vec2(1, 0));
    CHECK(d.time == 0.0);
    CHECK(timeToCollision(Vec2(0, 0), Vec2(1, 0), Vec2(0.5, 0), Vec2(0, 0), 1.0) == 0.0);
}

// --- decide -----------------------------------------------------------------

TEST_CASE(commitment_suppresses_flip_flopping) {
    // Two options trading a hair's advantage every tick — the classic
    // oscillation. The committed choice must never flap.
    Commitment c;
    c.minHold = 1.0;
    c.switchMargin = 0.2;
    int first = c.choose({0.50, 0.49}, 0.0);
    CHECK(first == 0);
    int switches = 0, prev = first;
    for (int i = 1; i <= 100; ++i) {
        Real a = (i % 2) ? 0.49 : 0.51;    // lead alternates by 0.02 (< margin)
        Real b = (i % 2) ? 0.51 : 0.49;
        int now = c.choose({a, b}, 0.1 * i);
        if (now != prev) ++switches;
        prev = now;
    }
    CHECK(switches == 0);
}

TEST_CASE(commitment_switches_for_a_sustained_clear_winner) {
    Commitment c;
    c.minHold = 1.0;
    c.switchMargin = 0.2;
    CHECK(c.choose({1.0, 0.0}, 0.0) == 0);
    // A decisively better option appears, but the hold protects the incumbent...
    CHECK(c.choose({0.2, 1.0}, 0.5) == 0);
    // ...until the hold expires — then the agent visibly changes its mind, once.
    CHECK(c.choose({0.2, 1.0}, 1.1) == 1);
    CHECK(c.heldFor(1.5) > 0.3);
}

TEST_CASE(commitment_emergency_preempts_the_hold) {
    Commitment c;
    c.minHold = 5.0;
    CHECK(c.choose({1.0, 0.0}, 0.0) == 0);
    // An imminent-collision re-decide overrides the hold instantly.
    CHECK(c.choose({0.0, 1.0}, 0.2, /*emergency=*/true) == 1);
    // An emergency that CONFIRMS the current choice doesn't reset its clock.
    c.choose({0.0, 1.0}, 0.4, true);
    CHECK(c.heldFor(0.4) > 0.15);
}

// --- the 2.5D sensor ----------------------------------------------------------

TEST_CASE(sensor_volume_gates_on_height) {
    SensorVolume s;
    s.cone.origin = Vec2(0, 0);
    s.cone.forward = Vec2(0, 1);
    s.cone.range = 30.0;
    s.maxHeightDelta = 3.0;
    // Same plan position: seen on my level, NOT seen crossing the overpass.
    CHECK(sees(s, Vec2(0, 10), 0.0));
    CHECK(sees(s, Vec2(0, 10), 2.0));
    CHECK(!sees(s, Vec2(0, 10), 5.8));    // bridge deck above
    CHECK(!sees(s, Vec2(0, 10), -5.8));   // underpass below
}
