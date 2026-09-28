#ifndef RAYTRACER_ENGINE_DEV_CONTROL_SYSTEM_H
#define RAYTRACER_ENGINE_DEV_CONTROL_SYSTEM_H

#include "../system.h"

namespace engine {

// Built-in development controls: quit, pause, and simulation-speed adjustment.
// Bindings go through the named-action layer (see input_map.h), so keys are
// configurable via Settings rather than hardcoded. The speed lives on the
// clock (never persisted); this system shows it on screen when it isn't 1x.
class DevControlSystem : public System {
public:
    // ownsQuit=false leaves the quit action's *handling* to the host state
    // (e.g. the arena's Esc returns to the editor instead of quitting); the
    // binding is still registered either way.
    explicit DevControlSystem(bool ownsQuit = true) : ownsQuit(ownsQuit) {}

    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;   // the sim-speed popup and badge
    void onStop(FrameContext& ctx) override;

private:
    bool ownsQuit = true;
    double shownSpeed = 1.0;    // the speed / pause last announced
    bool shownPaused = false;
    double wallClock = 0.0;     // real seconds (the popup times out in real time, whatever the speed)
    double popupUntil = -1.0;
};


}  // namespace engine

#endif
