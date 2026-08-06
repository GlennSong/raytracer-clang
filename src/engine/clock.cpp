#include "clock.h"

#include <algorithm>

namespace engine {

SimClock::SimClock(double fixedStepSeconds)
    : fixedStepSeconds(fixedStepSeconds > 0.0 ? fixedStepSeconds : 1.0 / 60.0),
      scale(1.0), accumulator(0.0), alpha(0.0), simTime(0.0) {}

int SimClock::advance(double realDeltaSeconds) {
    if (realDeltaSeconds < 0.0) realDeltaSeconds = 0.0;
    if (realDeltaSeconds > MAX_FRAME_SECONDS) realDeltaSeconds = MAX_FRAME_SECONDS;

    if (paused()) {
        // Time stands still (the accumulator neither grows nor drains, so
        // the interpolation alpha freezes the frame where it is) — unless a
        // single step was requested, which advances exactly one fixed step
        // and renders it fully (alpha 1).
        if (stepRequested) {
            stepRequested = false;
            simTime += fixedStepSeconds;
            alpha = 1.0;
            return 1;
        }
        return 0;
    }
    stepRequested = false;   // running normally: nothing pending

    accumulator += realDeltaSeconds * scale;

    int steps = 0;
    while (accumulator >= fixedStepSeconds && steps < MAX_STEPS_PER_ADVANCE) {
        accumulator -= fixedStepSeconds;
        simTime += fixedStepSeconds;
        steps++;
    }

    // Hit the cap with work still pending: drop the backlog rather than carry
    // lag into the next frame forever.
    backlogDropped = false;
    if (steps == MAX_STEPS_PER_ADVANCE && accumulator > fixedStepSeconds) {
        accumulator = 0.0;
        backlogDropped = true;
        backlogDrops++;
    }

    alpha = accumulator / fixedStepSeconds;
    return steps;
}

void SimClock::setTimeScale(double newScale) {
    scale = std::max(0.0, newScale);
}

void SimClock::setFixedStep(double seconds) {
    if (seconds > 0.0) fixedStepSeconds = seconds;
}

}  // namespace engine

