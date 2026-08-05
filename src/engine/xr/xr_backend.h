// The seam a headset platform implements. One implementation exists today —
// CompositorXrBackend inside the Metal backend (visionOS CompositorServices);
// an OpenXR backend (Quest, Android XR) fills the same three methods:
//
//   beginFrame  ≈ xrWaitFrame + xrLocateViews   (predict pose for the frame)
//   pushInput   ≈ the runtime's event delivery  (any thread)
//   pollInput   ≈ xrSyncActions                 (engine thread, once a frame)
//
// The backend is owned by the RENDERER, not the window: on visionOS the
// tracking session and the compositor drawables are inseparable (every
// drawable needs a device anchor), and OpenXR sessions are graphics-bound
// too (xrCreateSession takes the graphics device).

#ifndef ENGINE_XR_BACKEND_H
#define ENGINE_XR_BACKEND_H

#include <vector>

#include "xr_state.h"

namespace engine {

struct XrInputEvent {
    enum class Kind { PinchBegan, PinchMoved, PinchEnded, PinchCancelled };
    Kind kind = Kind::PinchBegan;
    // Gaze/selection ray in ORIGIN space at the moment of the event.
    Vec3 rayOrigin;
    Vec3 rayDir;
};

class XrBackend {
public:
    virtual ~XrBackend() = default;

    // Tracking is live (session running, poses answerable).
    virtual bool active() const = 0;

    // Fill `out` with the predicted head + per-view poses for the frame about
    // to be simulated. Returns false (and clears out.active) when tracking is
    // not available; the engine then behaves exactly as without XR.
    virtual bool beginFrame(XrState& out) = 0;

    // Queue an input event. Safe from any thread (hosts deliver spatial
    // events on their UI thread).
    virtual void pushInput(const XrInputEvent& event) = 0;

    // Drain queued events, in arrival order. Engine thread, once per frame.
    virtual void pollInput(std::vector<XrInputEvent>& out) = 0;
};

}  // namespace engine

#endif  // ENGINE_XR_BACKEND_H
