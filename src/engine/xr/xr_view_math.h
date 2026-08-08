// Per-eye view math: the composition that turns a headset runtime's tracking
// poses into the world-space eye the renderer draws from, plus the locomotion
// base that carries the tracking origin through the world.
//
// This lives in engine math rather than inside the Metal backend for one
// reason — TESTABILITY. The backend is Objective-C++ behind TARGET_OS_VISION,
// so nothing in it can be exercised off a Mac, and stereo cannot be exercised
// ON one either: the visionOS simulator reports a SINGLE view
// (`cp_drawable_get_view_count` == 1), so the two-view path only ever executes
// on device. That leaves the eye math verified by looking at it — and a
// swapped or dropped eye offset is precisely the failure a person cannot see
// by looking, whether because the bug is subtle or because they do not fuse
// stereo at all. Everything here is therefore pure, host-agnostic, and pinned
// numerically by tests/test_xr_stereo.cpp.
//
// Conventions, shared with xr_state.h: Mat4 is row-major storage under the
// column-vector convention — the rotation basis lives in columns 0..2 and the
// translation in column 3; forward is -Z. Poses named `aFromB` map a point in
// B's space into A's space, so they compose left-to-right: originFromEye =
// originFromDevice * deviceFromEye.

#ifndef ENGINE_XR_VIEW_MATH_H
#define ENGINE_XR_VIEW_MATH_H

#include "../../rt_math.h"

namespace engine {

// Eye height above the floor assumed when seeding the locomotion base from a
// gameplay camera. In play mode the game camera IS the player's eye, so the
// floor it stands on is this far below it — seeding at the camera itself would
// bury a standing user's head at ceiling height.
constexpr Real XR_SEED_EYE_HEIGHT = 1.6;

// The translation column of a transform.
inline Vec3 xrTranslationOf(const Mat4& m) {
    return Vec3(m.m[0][3], m.m[1][3], m.m[2][3]);
}

// Real metres -> world units for a RIGID origin-space transform: scale the
// translation and leave the rotation alone.
//
// Scaling the translation of the composed origin->eye transform is what makes
// world scale feel like a change in the USER's size rather than the world's.
// Both distances grow together: how far the head travels from the tracking
// origin, and how far apart the two eyes sit. The second is the one that is
// easy to mistake for a bug — at scale 25 the eyes are 25x further apart than
// a real head's, which is exactly the hyper-stereo that makes a life-size
// world read as a tabletop model. A giant's eyes ARE far apart.
inline Mat4 xrScaleOriginTransform(Mat4 originTransform, Real worldScale) {
    originTransform.m[0][3] *= worldScale;
    originTransform.m[1][3] *= worldScale;
    originTransform.m[2][3] *= worldScale;
    return originTransform;
}

// The world-space pose of one eye: the locomotion base composed with the
// scaled origin->eye transform. `originFromDevice` is the head pose from the
// runtime's tracking anchor and `deviceFromEye` the runtime's head->eye offset
// for this view; both are in real metres, which is why the composition is
// scaled as ONE transform rather than scaling each factor separately.
inline Mat4 xrWorldFromEye(const Vec3& originBase, const Mat4& originFromDevice,
                           const Mat4& deviceFromEye, Real worldScale) {
    const Mat4 base = Mat4::translate(originBase.x, originBase.y, originBase.z);
    return base * xrScaleOriginTransform(originFromDevice * deviceFromEye,
                                         worldScale);
}

// Distance between two composed eye poses, in world units. At worldScale 1
// this is the user's interpupillary distance; it is the single number that
// says whether stereo separation survived composition, which makes it worth
// naming rather than open-coding at each call site.
inline Real xrEyeSeparation(const Mat4& worldFromEyeLeft,
                            const Mat4& worldFromEyeRight) {
    return (xrTranslationOf(worldFromEyeRight)
            - xrTranslationOf(worldFromEyeLeft)).length();
}

// The locomotion base: where the headset's tracking origin sits in the world.
//
// It is seeded once from the gameplay camera and thereafter FOLLOWS THAT
// CAMERA BY DELTAS, so gameplay motion (teleports, vehicles, elevators)
// carries the user along while their own head motion stays free.
//
// Be precise about what the delta form does and does not buy, because it is
// easy to over-read. With only this writer the deltas telescope —
// base_n = base_0 + (hint_n - hint_0) = hint_n - XR_SEED_EYE_HEIGHT — so it is
// ALGEBRAICALLY IDENTICAL to just tracking the latest hint. It is not, on its
// own, what prevents the head-motion feedback loop.
//
// What prevents that loop is upstream: the hint is the GAMEPLAY camera, and
// PlayerSystem/CameraSystem rewrite that camera unconditionally every frame
// before XrCameraSystem runs (see the ordering note in arena_state.cpp). If
// the hint ever went stale and came back head-derived instead, the base would
// walk by the head's offset every frame and the world would slide away as the
// user leaned — with either formulation. The delta form earns its keep only as
// the version that stays correct if the base ever gains a SECOND writer
// (recentering, a direct teleport-the-origin path); absolute following would
// silently overwrite such a writer on the next frame.
struct XrLocomotionBase {
    bool valid = false;
    Vec3 base;
    Vec3 hintPrev;

    // Advance the base for this frame's gameplay-camera position and return
    // it. The first call seeds; every later call applies the delta.
    Vec3 update(const Vec3& hint) {
        if (!valid) {
            base = Vec3(hint.x, hint.y - XR_SEED_EYE_HEIGHT, hint.z);
            hintPrev = hint;
            valid = true;
        } else {
            base += hint - hintPrev;
            hintPrev = hint;
        }
        return base;
    }

    void reset() { valid = false; }
};

}  // namespace engine

#endif  // ENGINE_XR_VIEW_MATH_H
