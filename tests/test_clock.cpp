#include "test_framework.h"

#include "../src/engine/clock.h"

using namespace engine;  // namespace migration (ADR-0015)

namespace {
constexpr double STEP = 1.0 / 60.0;
constexpr double EPS = 1e-9;
}

TEST_CASE(clock_accumulates_into_steps) {
    SimClock clock(STEP);

    // Less than one step: no step runs, time accumulates into alpha.
    CHECK(clock.advance(STEP * 0.5) == 0);
    CHECK_APPROX(clock.interpolationAlpha(), 0.5, 1e-6);

    // Crossing the threshold runs exactly one step and keeps the remainder.
    CHECK(clock.advance(STEP * 0.75) == 1);
    CHECK_APPROX(clock.interpolationAlpha(), 0.25, 1e-6);
}

TEST_CASE(clock_runs_multiple_steps) {
    SimClock clock(STEP);
    CHECK(clock.advance(STEP * 3) == 3);
    CHECK_APPROX(clock.simulatedTime(), STEP * 3, 1e-9);
}

TEST_CASE(clock_time_scale_speeds_up) {
    SimClock clock(STEP);
    clock.setTimeScale(2.0);
    // Real delta of one step at 2x advances two simulated steps.
    CHECK(clock.advance(STEP) == 2);
}

TEST_CASE(clock_pause_runs_no_steps) {
    SimClock clock(STEP);
    clock.setTimeScale(0.0);
    CHECK(clock.paused());
    CHECK(clock.advance(STEP * 10) == 0);
    CHECK_APPROX(clock.simulatedTime(), 0, EPS);
}

TEST_CASE(clock_negative_time_scale_clamped) {
    SimClock clock(STEP);
    clock.setTimeScale(-5.0);
    CHECK_APPROX(clock.timeScale(), 0, EPS);
    CHECK(clock.paused());
}

TEST_CASE(clock_spiral_of_death_guard) {
    SimClock clock(STEP);
    // A huge real delta is clamped (MAX_FRAME_SECONDS) and capped at
    // MAX_STEPS_PER_ADVANCE, so it never tries to run hundreds of steps.
    int steps = clock.advance(1000.0);
    CHECK(steps <= 8);
    CHECK(steps > 0);
}

TEST_CASE(clock_set_fixed_step_rejects_nonpositive) {
    SimClock clock(STEP);
    clock.setFixedStep(0.0);
    CHECK_APPROX(clock.fixedStep(), STEP, EPS);
    clock.setFixedStep(-1.0);
    CHECK_APPROX(clock.fixedStep(), STEP, EPS);
    clock.setFixedStep(1.0 / 30.0);
    CHECK_APPROX(clock.fixedStep(), 1.0 / 30.0, EPS);
}
