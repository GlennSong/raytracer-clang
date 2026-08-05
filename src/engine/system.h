#ifndef RAYTRACER_ENGINE_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEM_H

#include "world.h"
#include "clock.h"
#include "audio/audio_engine.h"
#include "debug_draw.h"
#include "event_bus.h"
#include "input/input_map.h"
#include "input/player_input.h"
#include "xr/xr_state.h"
#include "../renderer/renderer.h"
#include "../renderer/window.h"
#include "../renderer/settings.h"
#include "../job_system.h"
#include <vector>
#include <memory>

namespace engine {

class AssetManager;

// End-of-frame request to swap the active app state (e.g. the editor's Play
// button, the game's Stop). Application pops the current state and pushes
// `next` after applyPending — states never touch the stack directly.
class AppState;
struct StateTransition {
    std::unique_ptr<AppState> next;

    // Special members live in application.cpp, where AppState is complete —
    // this header is included from places that never see app_state.h.
    StateTransition();
    ~StateTransition();
    void replaceWith(std::unique_ptr<AppState> state);
    bool pending() const { return next != nullptr; }
};

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
    // Explicit per-eye matrices when a headset drives the view, produced by
    // XrCameraSystem (which also rewrites `camera` to follow the head, so
    // culling/audio/Lua stay coherent). Inactive by default.
    XrRenderInfo xr;
};

// Services and per-frame data handed to every system hook. Field validity by
// phase: frameDelta in update(), interpolation in render(), clock.fixedStep()
// in fixedUpdate().
struct FrameContext {
    World& world;
    Renderer& renderer;
    AssetManager& assets;      // owns GPU mesh lifetime (ROADMAP 3.1)
    RenderView& view;
    SimClock& clock;
    Settings& settings;
    JobSystem& jobs;           // shared thread pool (ADR-0014): physics today
    EventBus& events;          // typed pub/sub (ADR-0066): publish now, or
                               // enqueue for the post-fixed-step dispatch
    DebugDraw& debug;          // immediate-mode debug lines (ADR-0067),
                               // drawn on top by DebugDrawSystem
    AudioEngine& audio;        // sealed miniaudio engine (ADR-0069); prefer
                               // PlaySound events over direct play() calls
    const InputState& input;   // polled continuous snapshot (mouse, raw keys)
    InputMap& actions;         // global/system actions (quit, pause): keyboard
    PlayerInputs& players;     // per-player gameplay input (see player_input.h)
    XrState& xr;               // headset state for this frame (engine/xr/);
                               // xr.active is false everywhere but headsets
    int framebufferWidth;
    int framebufferHeight;
    // Logical window size — mouse coordinates live in this space (it differs
    // from the framebuffer by the DPI scale on retina displays).
    int windowWidth;
    int windowHeight;
    double frameDelta;
    double interpolation;
    bool& quit;
    StateTransition& transition;   // end-of-frame state swap (editor <-> play)
    // The backtick debug overlay is up. Systems with always-registered debug
    // panels (Debug > Cameras) draw them only then, so plain play looks like
    // the shipped game.
    bool debugOverlayActive = false;
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
