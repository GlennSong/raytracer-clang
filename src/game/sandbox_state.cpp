#include "sandbox_state.h"

#include "../engine/asset_manager.h"
#include "../engine/systems/camera_system.h"
#include "../engine/systems/hand_interaction_system.h"
#include "../engine/systems/xr_camera_system.h"
#include "../engine/systems/xr_surface_system.h"
#include "../engine/systems/render_system.h"
#include "../engine/systems/debug_draw_system.h"
#include "../engine/systems/audio_system.h"
#ifdef RT_ENABLE_PHYSICS
#include "../engine/systems/physics_system.h"
#endif
#include "../log.h"

using namespace engine;

SandboxState::SandboxState(Window& window, Renderer& renderer)
    : PlayingState(window), renderer_(renderer) {
    addSystem<CameraSystem>();
#ifdef RT_ENABLE_PHYSICS
    auto& physSys = addSystem<PhysicsSystem>();
    // Hands BEFORE surfaces: while the palette is up or a grab is active,
    // HandInteractionSystem consumes the system pinch so the surface
    // gaze-drop probe doesn't also fire on the same gesture.
    addSystem<HandInteractionSystem>(&physSys);
    // Surfaces WITH colliders (the point of the sandbox: things land on the
    // real furniture), drawn as stippled translucent planes (fillSurfaces
    // false) — the room stays visible through them. No gaze placement here:
    // the palette is the one spawn path, so a pinch only ever means grab or
    // palette pick (user-confirmed; the gaze drop was the accidental-pile
    // machine).
    addSystem<XrSurfaceSystem>(&physSys, /*fillSurfaces=*/false);
#else
    addSystem<XrSurfaceSystem>(nullptr, /*fillSurfaces=*/false);
#endif
    // No PlayerSystem: locomotion is the user's own legs. XrCameraSystem still
    // owns the locomotion base + head-following camera; with nothing moving
    // the base, the world stays put and the pinch gesture is free for
    // interaction.
    addSystem<XrCameraSystem>();
    addSystem<RenderSystem>();
    addSystem<DebugDrawSystem>();
    addSystem<AudioSystem>();
}

void SandboxState::onEnter(FrameContext& ctx) {
    ctx.world.destroyAll();
    ctx.assets.clear();

    // Life-size, always: sandbox objects are authored in real metres and the
    // room colliders assume it. The launcher also sets this, but the state is
    // the authority — a stale menu scale must not make the room knee-high.
    renderer_.xrWorldScale = 1.0f;

    // Near-noon sun (user's call): a mostly-vertical light we cannot measure
    // but everyone finds believable indoors, and — through the shadow
    // catchers on the real floor/table planes — the one that makes virtual
    // objects cast shadows onto the real world directly beneath them, the
    // strongest depth cue available without camera access.
    ctx.view.lighting = SceneLighting{};
    ctx.view.lighting.sun.direction = Vec3(0.15, 0.97, 0.2);
    ctx.view.lighting.sun.color = Vec3(1.0, 0.97, 0.92);
    ctx.view.lighting.sun.intensity = 1.6f;
    ctx.view.lighting.sun.castsShadow = true;
    ctx.view.lighting.ambientMultiplier = 0.55f;
    ctx.view.lighting.exposure = 1.0f;

    LOG_INFO("[xr] sandbox: scale=1 passthrough=%d",
             renderer_.xrPassthrough ? 1 : 0);

    PlayingState::onEnter(ctx);
}
