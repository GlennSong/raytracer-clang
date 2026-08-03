// Engine-level XR state — the per-frame truth about the headset, in engine
// math types only. No Apple, simd, or OpenXR types may appear here: this is
// the surface every backend (visionOS CompositorServices today, OpenXR for
// Quest / Android XR later) fills, and every engine system reads.
//
// Spaces:
//   ORIGIN space  — the headset runtime's tracking origin (floor level, set
//                   where the user launched the app). Backends speak this.
//   WORLD space   — the game world. ORIGIN → WORLD is the locomotion base
//                   owned by XrCameraSystem (teleports move it); the renderer
//                   receives it as XrRenderInfo::worldFromOrigin and must
//                   never modify it (late-latch may refine ORIGIN-space eye
//                   poses only).
//
// Projections are reverse-Z, off-axis, GPU-ready as-is — pass them through,
// never recompose from FOV.

#ifndef ENGINE_XR_STATE_H
#define ENGINE_XR_STATE_H

#include <cstdint>

#include "../../rt_math.h"

namespace engine {

struct XrViewPose {
    Mat4 originFromEye;   // rigid: tracking origin -> this eye
    Mat4 projection;      // reverse-Z, off-axis
    int targetIndex = 0;  // color slice / texture index in the swapchain
    int width = 0;
    int height = 0;
};

struct XrState {
    bool active = false;    // backend present AND tracking live this frame
    uint64_t frameIndex = 0;  // must advance every frame; a stuck stage in the
                              // [xr] heartbeat shows up as a mismatch here

    Mat4 originFromHead;    // predicted head pose for this frame's update
    // World position of the tracking origin (the locomotion base). ORIGIN-
    // space rays become world rays by adding this (the base is translation-
    // only). Filled by the backend; XrCameraSystem will own it in Phase B.
    bool originBaseValid = false;
    Vec3 originBase;
    int viewCount = 0;      // 0 (inactive), 1 (simulator), 2 (device)
    XrViewPose views[2];

    // Spatial input, ORIGIN space. The latest gaze/selection ray plus pinch
    // edge state, derived by Application from the backend's input queue.
    bool gazeValid = false;
    Vec3 gazeOrigin;
    Vec3 gazeDir;
    bool pinchHeld = false;
    bool pinchBegan = false;   // edge: went down this frame
    bool pinchEnded = false;   // edge: RELEASED this frame (cancels excluded)
    double pinchHeldSeconds = 0.0;  // duration of the current/last hold —
                                    // lets gameplay separate a quick pinch
                                    // (teleport) from a long hold (menu)

    // WORLD-space mirror of the gaze ray, written by XrCameraSystem after the
    // locomotion base is composed. Gameplay (teleport targeting) reads these.
    bool gazeWorldValid = false;
    Vec3 gazeWorldOrigin;
    Vec3 gazeWorldDir;
};

}  // namespace engine

#endif  // ENGINE_XR_STATE_H
