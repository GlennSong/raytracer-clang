#ifndef RAYTRACER_ENGINE_CAMERA_CONTROLLER_H
#define RAYTRACER_ENGINE_CAMERA_CONTROLLER_H

#include "../../rt_math.h"
#include "../../renderer/renderer.h"

// Resolved camera intent for one frame, decoupled from how it was sourced
// (keyboard, mouse, gamepad). CameraSystem fills this from the action layer and
// pointer state; controllers consume it. Movement axes are normalized intent in
// [-1, 1] (the controller applies speed * dt); look/zoom are already-resolved
// per-frame deltas (mouse is inherently a delta, so stick look is folded in by
// the system, including its own dt). Keeping look as a delta lets one field
// carry both mouse and stick contributions.
struct CameraInput {
    Real moveForward = 0.0;
    Real moveRight = 0.0;
    Real moveUp = 0.0;
    Real lookYawDelta = 0.0;    // degrees this frame
    Real lookPitchDelta = 0.0;  // degrees this frame
    Real zoomDelta = 0.0;       // scroll units this frame
    bool boost = false;
};

// A camera behaviour that turns per-frame input into a CameraState. Orbit and
// fly are interchangeable implementations; CameraSystem holds a set and toggles
// the active one (ROADMAP 2.2). Pure and window-free, so it is unit-testable.
class CameraController {
public:
    virtual ~CameraController() = default;

    virtual void update(const CameraInput& input, Real dt) = 0;
    virtual CameraState cameraState(float aspect) const = 0;

    void setOrthographic(bool enabled) { orthoEnabled = enabled; }
    bool isOrthographic() const { return orthoEnabled; }

protected:
    bool orthoEnabled = false;
};

#endif
