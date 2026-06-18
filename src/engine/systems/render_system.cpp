#include "render_system.h"
#include "../components.h"

#include <algorithm>
#include <vector>

namespace engine {

std::vector<Mat4> frustumCullInstances(const std::vector<Mat4>& transforms,
                                       const Frustum& frustum,
                                       const Vec3& meshCenter, Real meshRadius,
                                       const Vec3& cameraPos, Real maxDistance) {
    std::vector<Mat4> visible;
    visible.reserve(transforms.size());
    Real maxDistSq = maxDistance * maxDistance;
    for (const Mat4& m : transforms) {
        Vec3 center = m.transformPoint(meshCenter);
        if (maxDistance > 0 && (center - cameraPos).lengthSquared() > maxDistSq)
            continue;
        Vec3 cx(m.m[0][0], m.m[1][0], m.m[2][0]);
        Vec3 cy(m.m[0][1], m.m[1][1], m.m[2][1]);
        Vec3 cz(m.m[0][2], m.m[1][2], m.m[2][2]);
        Real maxScale = std::max({cx.length(), cy.length(), cz.length()});
        if (frustum.containsSphere(center, meshRadius * maxScale))
            visible.push_back(m);
    }
    return visible;
}

void RenderSystem::onStart(FrameContext& ctx) {
    // Adopt the level's exposure (set by the level loader from JSON) rather than
    // forcing our own value — other owners (level JSON, the ImGui slider) drive it.
    exposure = ctx.view.lighting.exposure;
}

void RenderSystem::update(FrameContext& ctx) {
    // Track the current exposure (the level JSON or the ImGui slider may have set
    // it) so the Up/Down keys ramp from there. Only write back when a key is
    // actually held, so we don't stomp the slider/JSON value every frame.
    exposure = ctx.view.lighting.exposure;
    if (ctx.input.keyUp || ctx.input.keyDown) {
        if (ctx.input.keyUp)   exposure *= 1.0f + 2.0f * static_cast<float>(ctx.frameDelta);
        if (ctx.input.keyDown) exposure *= 1.0f - 2.0f * static_cast<float>(ctx.frameDelta);
        exposure = std::clamp(exposure, 0.05f, 20.0f);
        ctx.view.lighting.exposure = exposure;
    }
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
        [&](Entity entity, Transform& t, PrevTransform& prev, Renderable& r) {
            if (entity == ctx.view.activeCameraEntity) return;
            // Interpolated local matrix, then composed through any parent
            // chain (editor only — PLAY flattens parenting at load, so there
            // parentId is 0 and this is just the interpolated local).
            Mat4 model = lerp(prev.value, t, alpha).matrix();
            SourceSpec* s = ctx.world.get<SourceSpec>(entity);
            if (s && s->parentId != 0) {
                Entity parent = findByDocumentId(ctx.world, s->parentId);
                if (parent.valid())
                    model = worldMatrix(ctx.world, parent) * model;
            }
            // Cull against the world-space AABB (tight for large/flat meshes like
            // terrain chunks, where a bounding sphere swallows the sky and culls
            // wrongly — ADR-0034 Phase 1).
            BoundingSphere bounds = ctx.renderer.getMeshBounds(r.mesh);
            Vec3 worldMin, worldMax;
            transformedAABB(model, bounds.boxMin, bounds.boxMax, worldMin, worldMax);
            if (!frustum.containsAABB(worldMin, worldMax))
                return;
            ctx.renderer.drawMesh(r.mesh, model, r.material);
        });

    // Instanced groups (static scatter — the forest). Cheap coarse reject on the
    // group's bounds, then PER-INSTANCE frustum cull so only the on-screen plants
    // draw (not the whole region) — then one instanced draw of the visible subset.
    ctx.world.each<InstanceGroup>(
        [&](Entity, InstanceGroup& g) {
            if (g.transforms.empty()) return;
            if (!frustum.containsSphere(g.boundsCenter, g.boundsRadius)) return;
            // Whole-group distance reject: skip if the nearest point of the group's
            // bounds is beyond the draw distance (cheap before the per-instance pass).
            if (g.drawDistance > 0 &&
                (g.boundsCenter - cam.position).length() - g.boundsRadius > g.drawDistance)
                return;
            BoundingSphere mb = ctx.renderer.getMeshBounds(g.mesh);
            std::vector<Mat4> visible =
                frustumCullInstances(g.transforms, frustum, mb.center, mb.radius,
                                     cam.position, g.drawDistance);
            if (!visible.empty())
                ctx.renderer.drawMeshInstanced(g.mesh, visible, g.material);
        });
}

void RenderSystem::onStop(FrameContext& /*ctx*/) {
    // Exposure is owned by the level (cascade: defaults -> level -> runtime), so it
    // is no longer persisted to settings.json — that key used to stomp the level on
    // load. The Up/Down keys still ramp the live value within a session.
}

}  // namespace engine

