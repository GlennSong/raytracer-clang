#include "render_system.h"
#include "../components.h"

#include <algorithm>

namespace engine {

void RenderSystem::onStart(FrameContext& ctx) {
    exposure = static_cast<float>(ctx.settings.getDouble("exposure", 0.5));
    ctx.view.lighting.exposure = exposure;
}

void RenderSystem::update(FrameContext& ctx) {
    if (ctx.input.keyUp) exposure *= 1.0f + 2.0f * static_cast<float>(ctx.frameDelta);
    if (ctx.input.keyDown) exposure *= 1.0f - 2.0f * static_cast<float>(ctx.frameDelta);
    exposure = std::clamp(exposure, 0.05f, 20.0f);
    ctx.view.lighting.exposure = exposure;
}

void RenderSystem::render(FrameContext& ctx) {
    ctx.renderer.setCamera(ctx.view.camera);
    ctx.renderer.setLights(ctx.view.lighting);

    const auto& cam = ctx.view.camera;
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = (cam.projection == CameraProjection::Perspective)
        ? Mat4::perspective(degreesToRadians(cam.fovDegrees), cam.aspectRatio,
                            cam.nearPlane, cam.farPlane)
        : Mat4::orthographic(cam.orthoHeight, cam.aspectRatio,
                              cam.nearPlane, cam.farPlane);
    Frustum frustum = Frustum::fromViewProjection(proj * view);

    Real alpha = ctx.interpolation;
    ctx.world.each<Transform, PrevTransform, Renderable>(
        [&](Entity, Transform& t, PrevTransform& prev, Renderable& r) {
            Mat4 model = lerp(prev.value, t, alpha).matrix();
            BoundingSphere bounds = ctx.renderer.getMeshBounds(r.mesh);
            Vec3 worldCenter = model.transformPoint(bounds.center);
            Real maxScale = std::max({std::abs(t.scale.x),
                                      std::abs(t.scale.y),
                                      std::abs(t.scale.z)});
            if (!frustum.containsSphere(worldCenter, bounds.radius * maxScale))
                return;
            ctx.renderer.drawMesh(r.mesh, model, r.material);
        });
}

void RenderSystem::onStop(FrameContext& ctx) {
    ctx.settings.setDouble("exposure", exposure);
}

}  // namespace engine

