#include "xr_camera_system.h"

namespace engine {

void XrCameraSystem::update(FrameContext& ctx) {
    if (!ctx.xr.active || !ctx.xr.originBaseValid) return;

    // (1) The gameplay camera is the locomotion truth — hand its position to
    // the renderer's base-follow BEFORE the head overwrite below.
    ctx.renderer.setXrBaseHint(ctx.view.camera.position);

    // (2) Head-derived engine camera. Row-major Mat4, column-vector
    // convention: rotation basis lives in columns 0..2, translation in
    // column 3; forward is -Z.
    const Mat4& h = ctx.xr.originFromHead;
    const Vec3 headPos(h.m[0][3], h.m[1][3], h.m[2][3]);
    const Vec3 fwd(-h.m[0][2], -h.m[1][2], -h.m[2][2]);
    const Vec3 up(h.m[0][1], h.m[1][1], h.m[2][1]);

    CameraState& cam = ctx.view.camera;
    cam.position = ctx.xr.originBase + headPos;
    cam.target = cam.position + fwd;
    cam.up = up;
    cam.projection = CameraProjection::Perspective;
    // Wider than either eye's actual frustum: culling must never trim what
    // the second eye or the compositor's reprojection can still show.
    cam.fovDegrees = 100.0f;
    if (ctx.xr.viewCount > 0 && ctx.xr.views[0].height > 0) {
        cam.aspectRatio = static_cast<float>(ctx.xr.views[0].width)
                        / static_cast<float>(ctx.xr.views[0].height);
    }

    // (3) Hand skeletons, drawn as bones in the world: a line per joint to
    // its parent, tips capped with a small marker. The user's real hands,
    // rendered by the engine — left teal, right amber.
    for (int handIndex = 0; handIndex < 2; handIndex++) {
        const XrHand& hand = ctx.xr.hands[handIndex];
        if (!hand.tracked) continue;
        const Vec3 color = handIndex == 0 ? Vec3(0.3, 0.9, 0.9)
                                          : Vec3(1.0, 0.7, 0.25);
        Vec3 world[XR_HAND_JOINT_COUNT];
        for (int j = 0; j < XR_HAND_JOINT_COUNT; j++) {
            const Mat4& m = hand.joints[j].originFromJoint;
            world[j] = ctx.xr.originBase + Vec3(m.m[0][3], m.m[1][3], m.m[2][3]);
        }
        for (int j = 0; j < XR_HAND_JOINT_COUNT; j++) {
            if (!hand.joints[j].tracked) continue;
            const int parent = xrHandJointParent(j);
            if (parent >= 0 && hand.joints[parent].tracked)
                ctx.debug.line(world[parent], world[j], color);
        }
        // Fingertip caps make individual finger motion easy to read.
        static const int kTips[] = {XR_JOINT_THUMB_TIP, XR_JOINT_INDEX_TIP,
                                    XR_JOINT_MIDDLE_TIP, XR_JOINT_RING_TIP,
                                    XR_JOINT_LITTLE_TIP};
        for (int tip : kTips) {
            if (hand.joints[tip].tracked)
                ctx.debug.sphere(world[tip], 0.008, color, 0, 8);
        }
    }
}

}  // namespace engine
