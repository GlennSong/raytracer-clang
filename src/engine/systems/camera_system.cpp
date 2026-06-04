#include "camera_system.h"

void CameraSystem::onStart(FrameContext& ctx) {
    camera.target = Vec3(
        ctx.settings.getDouble("cameraTargetX", 0.0),
        ctx.settings.getDouble("cameraTargetY", 1.0),
        ctx.settings.getDouble("cameraTargetZ", 0.0));
    camera.distance = static_cast<float>(ctx.settings.getDouble("cameraDistance", 8.0));
    camera.yaw = static_cast<float>(ctx.settings.getDouble("cameraYaw", 0.0));
    camera.pitch = static_cast<float>(ctx.settings.getDouble("cameraPitch", 25.0));
    camera.orthographic = ctx.settings.getBool("cameraOrthographic", false);
}

void CameraSystem::onEvent(const Event& event, FrameContext&) {
    if (event.type == EventType::KeyPressed && !event.repeat &&
        event.key == KeyCode::P) {
        camera.orthographic = !camera.orthographic;
    }
}

void CameraSystem::update(FrameContext& ctx) {
    camera.update(ctx.input, ctx.frameDelta);
    float aspect = (ctx.framebufferHeight > 0)
        ? static_cast<float>(ctx.framebufferWidth) / ctx.framebufferHeight
        : 1.0f;
    ctx.view.camera = camera.getCameraState(aspect);
}

void CameraSystem::onStop(FrameContext& ctx) {
    ctx.settings.setDouble("cameraTargetX", camera.target.x);
    ctx.settings.setDouble("cameraTargetY", camera.target.y);
    ctx.settings.setDouble("cameraTargetZ", camera.target.z);
    ctx.settings.setDouble("cameraDistance", camera.distance);
    ctx.settings.setDouble("cameraYaw", camera.yaw);
    ctx.settings.setDouble("cameraPitch", camera.pitch);
    ctx.settings.setBool("cameraOrthographic", camera.orthographic);
}
