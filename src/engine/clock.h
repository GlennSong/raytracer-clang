#ifndef RAYTRACER_ENGINE_CLOCK_H
#define RAYTRACER_ENGINE_CLOCK_H

// Decouples a fixed-timestep simulation from a variable frame rate.
//
// Each frame, feed the real elapsed seconds to advance(); it returns how many
// fixed steps the simulation should run now. Render between steps using
// interpolationAlpha() so motion stays smooth regardless of frame rate.
//
// timeScale changes how fast simulated time passes relative to real time
// without changing the step size, so accuracy is unaffected when you speed up,
// slow down, or pause. (Shrinking the step instead would change integration
// behaviour and cost more per simulated second — that knob is setFixedStep,
// for trading accuracy against performance, not for changing speed.)
class SimClock {
public:
    explicit SimClock(double fixedStepSeconds = 1.0 / 60.0);

    // Returns the number of fixed steps to run this frame.
    int advance(double realDeltaSeconds);

    double interpolationAlpha() const { return alpha; }
    double simulatedTime() const { return simTime; }

    void setTimeScale(double scale);
    double timeScale() const { return scale; }
    bool paused() const { return scale == 0.0; }

    void setFixedStep(double seconds);
    double fixedStep() const { return fixedStepSeconds; }

private:
    double fixedStepSeconds;
    double scale;
    double accumulator;
    double alpha;
    double simTime;

    // After a long stall (debugger pause, window drag) the accumulator must not
    // grow without bound, or the sim tries to "catch up" forever — the spiral
    // of death. Clamp the real delta and cap steps per frame.
    static constexpr double MAX_FRAME_SECONDS = 0.25;
    static constexpr int MAX_STEPS_PER_ADVANCE = 8;
};

#endif
