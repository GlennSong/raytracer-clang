#include "render_system.h"

#include <algorithm>

void RenderSystem::onStart(FrameContext& ctx) {
    exposure = static_cast<float>(ctx.settings.getDouble("exposure", 0.5));
    ctx.view.exposure = exposure;
}

void RenderSystem::update(FrameContext& ctx) {
    if (ctx.input.keyUp) exposure *= 1.0f + 2.0f * static_cast<float>(ctx.frameDelta);
    if (ctx.input.keyDown) exposure *= 1.0f - 2.0f * static_cast<float>(ctx.frameDelta);
    exposure = std::clamp(exposure, 0.05f, 20.0f);
    ctx.view.exposure = exposure;
}

void RenderSystem::render(FrameContext& ctx) {
    ctx.renderer.setCamera(ctx.view.camera);
    ctx.renderer.setLights(ctx.view.lights, ctx.view.exposure);

    for (const Entity& e : ctx.world.entities()) {
        Transform t = ctx.world.renderTransform(e, ctx.interpolation);
        ctx.renderer.drawMesh(e.mesh, t.matrix(), e.material);
    }
}

void RenderSystem::onStop(FrameContext& ctx) {
    ctx.settings.setDouble("exposure", exposure);
}
