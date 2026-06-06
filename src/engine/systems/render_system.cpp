#include "render_system.h"
#include "../components.h"

#include <algorithm>

namespace engine {

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

    Real alpha = ctx.interpolation;
    ctx.world.each<Transform, PrevTransform, Renderable>(
        [&](Entity, Transform& t, PrevTransform& prev, Renderable& r) {
            Mat4 model = lerp(prev.value, t, alpha).matrix();
            ctx.renderer.drawMesh(r.mesh, model, r.material);
        });
}

void RenderSystem::onStop(FrameContext& ctx) {
    ctx.settings.setDouble("exposure", exposure);
}

}  // namespace engine

