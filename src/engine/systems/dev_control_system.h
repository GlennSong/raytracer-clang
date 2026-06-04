#ifndef RAYTRACER_ENGINE_DEV_CONTROL_SYSTEM_H
#define RAYTRACER_ENGINE_DEV_CONTROL_SYSTEM_H

#include "../system.h"

// Built-in development controls: quit, pause, and simulation-speed adjustment.
// Translates discrete key events into time-scale changes on the clock and a
// quit request. Persists the chosen speed via Settings.
class DevControlSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void onEvent(const Event& event, FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    double simSpeed = 1.0;   // time scale applied when not paused
    bool paused = false;
};

#endif
