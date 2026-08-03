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
}

}  // namespace engine
