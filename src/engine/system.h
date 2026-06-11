#ifndef RAYTRACER_ENGINE_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEM_H

#include "world.h"
#include "clock.h"
#include "input/input_map.h"
#include "input/player_input.h"
#include "../renderer/renderer.h"
#include "../renderer/window.h"
#include "../renderer/settings.h"
#include "../job_system.h"
#include <vector>

namespace engine {

// Shared "what to render this frame" resource. Systems that produce view data
// (camera, lighting, exposure) write it; the render system reads it. A minimal
// stand-in for the resource concept a full ECS will formalize later.
struct RenderView {
    CameraState camera;
    SceneLighting lighting;
    // The placed camera the view is rendered through, if any (invalid when the
    // editor controllers drive it). Lets the render system hide that entity's
    // gizmo — you don't draw the camera you are inside of.
    Entity activeCameraEntity;
};

// Services and per-frame data handed to every system hook. Field validity by
// phase: frameDelta in update(), interpolation in render(), clock.fixedStep()
// in fixedUpdate().
struct FrameContext {
    World& world;
    Renderer& renderer;
    RenderView& view;
    SimClock& clock;
    Settings& settings;
    JobSystem& jobs;           // shared thread pool (ADR-0014): physics today
    const InputState& input;   // polled continuous snapshot (mouse, raw keys)
    InputMap& actions;         // global/system actions (quit, pause): keyboard
    PlayerInputs& players;     // per-player gameplay input (see player_input.h)
    int framebufferWidth;
    int framebufferHeight;
    double frameDelta;
    double interpolation;
    bool& quit;
};

// A unit of engine behaviour, ticked by Application each frame. Override only
// the hooks you need; all default to no-ops. The interface is deliberately
// independent of the entity-storage model, so it survives the move to an ECS.
class System {
public:
    virtual ~System() = default;

    virtual void onStart(FrameContext&) {}
    virtual void onEvent(const Event&, FrameContext&) {}
    virtual void update(FrameContext&) {}
    virtual void fixedUpdate(FrameContext&) {}
    virtual void render(FrameContext&) {}
    virtual void onStop(FrameContext&) {}
};


}  // namespace engine

#endif
