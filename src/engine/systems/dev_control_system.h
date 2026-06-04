#ifndef RAYTRACER_ENGINE_DEV_CONTROL_SYSTEM_H
#define RAYTRACER_ENGINE_DEV_CONTROL_SYSTEM_H

#include "../system.h"

// Built-in development controls: quit, pause, and simulation-speed adjustment.
// Bindings go through the named-action layer (see input_map.h), so keys are
// configurable via Settings rather than hardcoded. Persists the chosen speed
// via Settings.
class DevControlSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    double simSpeed = 1.0;   // time scale applied when not paused
    bool paused = false;
};

#endif
