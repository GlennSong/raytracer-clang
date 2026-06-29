#include "camera_system.h"

#include "../components.h"
#include "../mesh_builder.h"
#include "../camera_store.h"
#include "../../log.h"

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

    // Placed-camera viewports (docs/virtual-camera-plan.md, Phase 1).
    actions.bindButton("cam_place", KeyCode::C);
    actions.bindButton("cam_place", GamepadButton::Y);
    actions.bindButton("cam_cycle_next", KeyCode::V);
    actions.bindButton("cam_cycle_next", GamepadButton::DpadRight);
    actions.bindButton("cam_cycle_prev", KeyCode::B);
    actions.bindButton("cam_cycle_prev", GamepadButton::DpadLeft);
    actions.bindButton("cam_view_editor", KeyCode::X);
    actions.bindButton("cam_view_editor", KeyCode::Backspace);
    actions.bindButton("cam_view_editor", GamepadButton::DpadDown);

    // Detach the fly camera from whatever pins it (the first-person player in
    // game states) to roam freely — e.g. to place a camera up high.
    actions.bindButton("cam_detach", KeyCode::F);
    actions.bindButton("cam_detach", GamepadButton::RightThumb);
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

    // Editor-style states opt into buttonless mouse look (the F detach toggle
    // flips it at runtime either way).
    freeLook = ctx.settings.getBool("cameraFreeLook", false);
    detachEnabled = ctx.settings.getBool("cameraDetachEnabled", true);

    // Saved placed cameras (sidecar JSON next to the level; the game state
    // sets the path before systems start). Loaded entities need gizmos here —
    // the store is renderer-free.
    storePath = ctx.settings.getString("cameraStorePath", "");
    if (!storePath.empty()) {
        CameraStore::load(storePath, ctx.world);
        placedCount = static_cast<int>(collectCameras(ctx.world).size());
        ensureGizmos(ctx);
    }
}

CameraInput CameraSystem::gatherInput(FrameContext& ctx) const {
    CameraInput in;
    in.moveForward = ctx.actions.axis("cam_forward");
    in.moveRight = ctx.actions.axis("cam_right");
    in.moveUp = ctx.actions.axis("cam_up");
    in.boost = ctx.actions.held("cam_boost");

    // Fly looks freely when pinned first-person OR detached (freeLook) — no
    // button to hold — but never while the pointer is over a debug panel, so
    // dragging a slider doesn't spin the view.
    bool mouseLook = !ctx.input.uiWantsMouse &&
        (flyActive ? (fly.positionLocked || freeLook || ctx.input.mouseRightDown)
                   : ctx.input.mouseLeftDown);
    Real mouseYaw = mouseLook ? -ctx.input.mouseDeltaX * mouseSensitivity : 0.0;
    Real mousePitch = mouseLook ? -ctx.input.mouseDeltaY * mouseSensitivity : 0.0;

    Real stick = stickLookSpeed * ctx.frameDelta;
    in.lookYawDelta = mouseYaw + ctx.actions.axis("cam_look_x") * stick;
    in.lookPitchDelta = mousePitch + ctx.actions.axis("cam_look_y") * stick;

    in.zoomDelta = ctx.input.scrollDelta;
    return in;
}

void CameraSystem::update(FrameContext& ctx) {
    float aspect = (ctx.framebufferHeight > 0)
        ? static_cast<float>(ctx.framebufferWidth) / ctx.framebufferHeight
        : 1.0f;

    Entity viewBefore = activeCamera;

    // FPS walk vs. free fly (live toggle; default free). Web defaults to walk.
    fly.grounded = ctx.settings.getBool("cameraGrounded", false);

    // Detach/re-attach the fly camera (game states pin it to the player via
    // positionLocked; see PlayerSystem). Detaching switches to fly so the
    // freecam is immediately steerable.
    if (detachEnabled && ctx.actions.pressed("cam_detach")) {
        fly.positionLocked = !fly.positionLocked;
        freeLook = !fly.positionLocked;
        if (!fly.positionLocked) {
            flyActive = true;
            active = &fly;
            LOG_INFO << "Camera detached: free-fly (mouse looks, WASD/QE "
                        "moves, F re-attaches)";
        } else {
            LOG_INFO << "Camera attached";
        }
    }

    // A destroyed (or stripped) active camera falls back to the editor view —
    // the generation check makes stale handles detectable rather than silent.
    if (activeCamera.valid() &&
        (!ctx.world.alive(activeCamera) ||
         !ctx.world.has<SceneCamera>(activeCamera) ||
         !ctx.world.has<Transform>(activeCamera)))
        activeCamera = Entity{};

    // Placing only makes sense from the editor view, where you just framed the
    // shot; while looking through a placed camera the editor pose is hidden.
    if (ctx.actions.pressed("cam_place") && !activeCamera.valid())
        activeCamera = placeCamera(ctx, aspect);

    if (ctx.actions.pressed("cam_cycle_next"))
        activeCamera = cycleCamera(collectCameras(ctx.world), activeCamera, +1);
    if (ctx.actions.pressed("cam_cycle_prev"))
        activeCamera = cycleCamera(collectCameras(ctx.world), activeCamera, -1);
    if (ctx.actions.pressed("cam_view_editor"))
        activeCamera = Entity{};

    if (ctx.actions.pressed("cam_toggle")) {
        if (activeCamera.valid()) {
            activeCamera = Entity{};  // Tab from a placed camera returns home
        } else {
            flyActive = !flyActive;
            active = flyActive ? static_cast<CameraController*>(&fly)
                               : static_cast<CameraController*>(&orbit);
        }
    }
    if (ctx.actions.pressed("cam_toggle_projection"))
        active->setOrthographic(!active->isOrthographic());

    if (viewBefore != activeCamera) {
        if (activeCamera.valid())
            LOG_INFO << "Viewport: " << ctx.world.get<SceneCamera>(activeCamera)->name;
        else
            LOG_INFO << "Viewport: editor (" << (flyActive ? "fly" : "orbit") << ")";
    }

    if (activeCamera.valid()) {
        // Stationary by design: the controllers receive no input, the view is
        // rebuilt from the entity each frame so external Transform writes
        // (inspector, future pose sources) show up immediately.
        ctx.view.camera = sceneCameraState(*ctx.world.get<Transform>(activeCamera),
                                           *ctx.world.get<SceneCamera>(activeCamera),
                                           aspect);
    } else {
        active->update(gatherInput(ctx), ctx.frameDelta);
        ctx.view.camera = active->cameraState(aspect);
    }
    ctx.view.activeCameraEntity = activeCamera;
}

Entity CameraSystem::placeCameraAtView(FrameContext& ctx) {
    float aspect = (ctx.framebufferHeight > 0)
        ? static_cast<float>(ctx.framebufferWidth) / ctx.framebufferHeight
        : 1.0f;
    return placeCamera(ctx, aspect);
}

Entity CameraSystem::placeCamera(FrameContext& ctx, float aspect) {
    // Adopt the exact editor framing: position from the view, orientation from
    // its forward vector (roll-free, like the controllers themselves).
    CameraState view = active->cameraState(aspect);
    Transform transform;
    transform.position = view.position;
    transform.orientation = orientationFromForward(view.target - view.position);

    Entity entity = ctx.world.create();
    ctx.world.add<Transform>(entity, transform);
    ctx.world.add<PrevTransform>(entity, {transform});

    SceneCamera camera;
    camera.name = "Camera " + std::to_string(++placedCount);
    ctx.world.add<SceneCamera>(entity, camera);
    ensureGizmos(ctx);

    LOG_INFO << "Placed " << camera.name << " at (" << transform.position.x
             << ", " << transform.position.y << ", " << transform.position.z
             << ") — V/B cycle viewports, X returns to the editor";
    return entity;
}

// Camera-body gizmo: a box with a cone "lens" on the front face, so which way
// a placed camera points reads at a glance. MeshBuilder's cone points +Y;
// rotate it onto -Z (the camera's forward) and seat its base on the box front.
static RenderMesh cameraGizmoMesh() {
    RenderMesh body = MeshBuilder::box(Vec3(0.22, 0.28, 0.4));
    RenderMesh snout = MeshBuilder::cone(0.1f, 0.18f);

    const uint32_t base = static_cast<uint32_t>(body.vertices.size());
    for (Vertex v : snout.vertices) {
        // +Y -> -Z is (x, y, z) -> (x, z, -y); base lands at z = -0.2.
        v.position = Vec3(v.position.x, v.position.z, -v.position.y - 0.29);
        v.normal = Vec3(v.normal.x, v.normal.z, -v.normal.y);
        v.tangent = Vec3(v.tangent.x, v.tangent.z, -v.tangent.y);
        body.vertices.push_back(v);
    }
    for (uint32_t i : snout.indices) body.indices.push_back(base + i);
    return body;
}

void CameraSystem::ensureGizmos(FrameContext& ctx) {
    std::vector<Entity> missing;
    ctx.world.each<Transform, SceneCamera>(
        [&](Entity e, Transform&, SceneCamera& cam) {
            if (cam.showGizmo && !ctx.world.has<Renderable>(e))
                missing.push_back(e);
        });
    if (missing.empty()) return;

    if (!gizmoMesh.valid())
        gizmoMesh = ctx.renderer.uploadMesh(cameraGizmoMesh());
    if (!gizmoMesh.valid()) return;

    for (Entity e : missing) {
        Renderable gizmo;
        gizmo.mesh = gizmoMesh;
        gizmo.material.albedo = Vec3(0.35, 0.35, 0.4);
        gizmo.material.metallic = 0.7f;
        gizmo.material.roughness = 0.35f;
        ctx.world.add<Renderable>(e, gizmo);
    }
}

void CameraSystem::onStop(FrameContext& ctx) {
    if (!storePath.empty()) CameraStore::save(storePath, ctx.world);

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

