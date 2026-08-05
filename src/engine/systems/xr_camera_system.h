#ifndef RAYTRACER_ENGINE_XR_CAMERA_SYSTEM_H
#define RAYTRACER_ENGINE_XR_CAMERA_SYSTEM_H

#include "../system.h"

namespace engine {

// Makes the shared RenderView camera FOLLOW THE HEAD when a headset is
// tracking (Phase B of the XR plan). Runs after every gameplay camera writer
// and before rendering:
//
//   1. Records the gameplay camera's position as the renderer's locomotion
//      base hint (teleports/vehicles move the world through it) — the hint
//      channel is separate precisely so the head-derived camera below never
//      feeds back into base following (which would double-count head motion).
//   2. Rewrites ctx.view.camera from the tracked head pose, with a widened
//      FOV that covers both eyes plus compositor reprojection — so frustum
//      culling, street-lamp selection, the audio listener, and Lua's camera
//      all follow where the user actually looks. (Before this, culling used
//      the spawn-facing gameplay camera: turn your head and the world behind
//      that frustum vanished.)
//
// The renderer still substitutes exact per-eye matrices at render time; this
// camera is the ENGINE-side truth, not the projection source.
class XrCameraSystem : public System {
public:
    void update(FrameContext& ctx) override;
};

}  // namespace engine

#endif
