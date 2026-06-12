#ifndef RAYTRACER_ENGINE_DEV_CONTROL_SYSTEM_H
#define RAYTRACER_ENGINE_DEV_CONTROL_SYSTEM_H

#include "../system.h"

namespace engine {

// Built-in development controls: quit, pause, and simulation-speed adjustment.
// Bindings go through the named-action layer (see input_map.h), so keys are
// configurable via Settings rather than hardcoded. Persists the chosen speed
// via Settings.
class DevControlSystem : public System {
public:
    // ownsQuit=false leaves the quit action's *handling* to the host state
    // (e.g. the arena's Esc returns to the editor instead of quitting); the
    // binding is still registered either way.
    explicit DevControlSystem(bool ownsQuit = true) : ownsQuit(ownsQuit) {}

    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    bool ownsQuit = true;
    double simSpeed = 1.0;   // time scale (pause is the clock's own switch)
};


}  // namespace engine

#endif
