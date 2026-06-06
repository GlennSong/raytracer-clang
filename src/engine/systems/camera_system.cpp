#include "camera_system.h"

namespace engine {

void CameraSystem::registerBindings(InputMap& actions) const {
    // Movement: keyboard digital + gamepad sticks/triggers. Gamepad axis signs
    // account for GLFW's convention (stick up and look up are negative).
    actions.bindAxis("cam_forward", KeyCode::W, 1.0);
    actions.bindAxis("cam_forward", KeyCode::S, -1.0);
    actions.bindAxis("cam_forward", GamepadAxis::LeftY, -1.0);

    actions.bindAxis("cam_right", KeyCode::D, 1.0);
    actions.bindAxis("cam_right", KeyCode::A, -1.0);
    actions.bindAxis("cam_right", GamepadAxis::LeftX, 1.0);

    actions.bindAxis("cam_up", KeyCode::E, 1.0);
    actions.bindAxis("cam_up", KeyCode::Q, -1.0);
    actions.bindAxis("cam_up", GamepadAxis::RightTrigger, 1.0);
    actions.bindAxis("cam_up", GamepadAxis::LeftTrigger, -1.0);

    // Look: gamepad right stick (mouse look is folded in separately, from the
    // pointer delta, since it is unbounded rather than a normalized axis).
    actions.bindAxis("cam_look_x", GamepadAxis::RightX, 1.0);
    actions.bindAxis("cam_look_y", GamepadAxis::RightY, -1.0);

    actions.bindButton("cam_boost", KeyCode::LeftShift);
    actions.bindButton("cam_boost", GamepadButton::LeftBumper);
    actions.bindButton("cam_toggle", KeyCode::Tab);
    actions.bindButton("cam_toggle", GamepadButton::Back);
    actions.bindButton("cam_toggle_projection", KeyCode::P);
    actions.bindButton("cam_toggle_projection", GamepadButton::Start);
}

void CameraSystem::onStart(FrameContext& ctx) {
    registerBindings(ctx.actions);

    orbit.target = Vec3(
        ctx.settings.getDouble("orbitTargetX", 0.0),
        ctx.settings.getDouble("orbitTargetY", 1.0),
        ctx.settings.getDouble("orbitTargetZ", 0.0));
    orbit.distance = ctx.settings.getDouble("orbitDistance", 8.0);
    orbit.yaw = ctx.settings.getDouble("orbitYaw", 0.0);
    orbit.pitch = ctx.settings.getDouble("orbitPitch", 25.0);
    orbit.setOrthographic(ctx.settings.getBool("orbitOrtho", false));

    fly.eye = Vec3(
        ctx.settings.getDouble("flyEyeX", 0.0),
        ctx.settings.getDouble("flyEyeY", 1.5),
        ctx.settings.getDouble("flyEyeZ", 8.0));
    fly.yaw = ctx.settings.getDouble("flyYaw", 0.0);
    fly.pitch = ctx.settings.getDouble("flyPitch", 0.0);
    fly.setOrthographic(ctx.settings.getBool("flyOrtho", false));

    flyActive = ctx.settings.getString("cameraMode", "orbit") == "fly";
    active = flyActive ? static_cast<CameraController*>(&fly)
                       : static_cast<CameraController*>(&orbit);
}

CameraInput CameraSystem::gatherInput(FrameContext& ctx) const {
    CameraInput in;
    in.moveForward = ctx.actions.axis("cam_forward");
    in.moveRight = ctx.actions.axis("cam_right");
    in.moveUp = ctx.actions.axis("cam_up");
    in.boost = ctx.actions.held("cam_boost");

    // Mouse look is active while dragging: left button for orbit (rotate about
    // the subject), right button for fly (mouse-look). Stick look is always on.
    bool mouseLook = flyActive ? ctx.input.mouseRightDown : ctx.input.mouseLeftDown;
    Real mouseYaw = mouseLook ? -ctx.input.mouseDeltaX * mouseSensitivity : 0.0;
    Real mousePitch = mouseLook ? -ctx.input.mouseDeltaY * mouseSensitivity : 0.0;

    Real stick = stickLookSpeed * ctx.frameDelta;
    in.lookYawDelta = mouseYaw + ctx.actions.axis("cam_look_x") * stick;
    in.lookPitchDelta = mousePitch + ctx.actions.axis("cam_look_y") * stick;

    in.zoomDelta = ctx.input.scrollDelta;
    return in;
}

void CameraSystem::update(FrameContext& ctx) {
    if (ctx.actions.pressed("cam_toggle")) {
        flyActive = !flyActive;
        active = flyActive ? static_cast<CameraController*>(&fly)
                           : static_cast<CameraController*>(&orbit);
    }
    if (ctx.actions.pressed("cam_toggle_projection"))
        active->setOrthographic(!active->isOrthographic());

    active->update(gatherInput(ctx), ctx.frameDelta);

    float aspect = (ctx.framebufferHeight > 0)
        ? static_cast<float>(ctx.framebufferWidth) / ctx.framebufferHeight
        : 1.0f;
    ctx.view.camera = active->cameraState(aspect);
}

void CameraSystem::onStop(FrameContext& ctx) {
    ctx.settings.setDouble("orbitTargetX", orbit.target.x);
    ctx.settings.setDouble("orbitTargetY", orbit.target.y);
    ctx.settings.setDouble("orbitTargetZ", orbit.target.z);
    ctx.settings.setDouble("orbitDistance", orbit.distance);
    ctx.settings.setDouble("orbitYaw", orbit.yaw);
    ctx.settings.setDouble("orbitPitch", orbit.pitch);
    ctx.settings.setBool("orbitOrtho", orbit.isOrthographic());

    ctx.settings.setDouble("flyEyeX", fly.eye.x);
    ctx.settings.setDouble("flyEyeY", fly.eye.y);
    ctx.settings.setDouble("flyEyeZ", fly.eye.z);
    ctx.settings.setDouble("flyYaw", fly.yaw);
    ctx.settings.setDouble("flyPitch", fly.pitch);
    ctx.settings.setBool("flyOrtho", fly.isOrthographic());

    ctx.settings.setString("cameraMode", flyActive ? "fly" : "orbit");
}

}  // namespace engine

