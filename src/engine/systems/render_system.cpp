#include "render_system.h"
#include "../components.h"
#include "../../log.h"

#include <algorithm>
#include <vector>

namespace engine {

std::vector<Mat4> frustumCullInstances(const std::vector<Mat4>& transforms,
                                       const Frustum& frustum,
                                       const Vec3& meshCenter, Real meshRadius) {
    std::vector<Mat4> visible;
    visible.reserve(transforms.size());
    for (const Mat4& m : transforms) {
        Vec3 center = m.transformPoint(meshCenter);
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
    // TEMP diagnostic: log the actual near/far reaching the renderer whenever it
    // changes, to confirm the camera's view distance in the live viewport.
    static float dbgNear = -1, dbgFar = -1;
    if (cam.nearPlane != dbgNear || cam.farPlane != dbgFar) {
        dbgNear = cam.nearPlane;
        dbgFar = cam.farPlane;
        LOG_INFO << "[camera] near=" << cam.nearPlane << " far=" << cam.farPlane
                 << " fov=" << cam.fovDegrees << " pos=(" << cam.position.x << ","
                 << cam.position.y << "," << cam.position.z << ")";
    }
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
            BoundingSphere bounds = ctx.renderer.getMeshBounds(r.mesh);
            Vec3 worldCenter = model.transformPoint(bounds.center);
            Vec3 cx(model.m[0][0], model.m[1][0], model.m[2][0]);
            Vec3 cy(model.m[0][1], model.m[1][1], model.m[2][1]);
            Vec3 cz(model.m[0][2], model.m[1][2], model.m[2][2]);
            Real maxScale = std::max({cx.length(), cy.length(), cz.length()});
            // TEMP diagnostic: per-object frustum cull bypassed to test whether
            // it is wrongly rejecting the large terrain meshes. Restore after.
            (void)worldCenter; (void)bounds; (void)maxScale; (void)frustum;
            // if (!frustum.containsSphere(worldCenter, bounds.radius * maxScale))
            //     return;
            ctx.renderer.drawMesh(r.mesh, model, r.material);
        });

    // Instanced groups (static scatter — the forest). Cheap coarse reject on the
    // group's bounds, then PER-INSTANCE frustum cull so only the on-screen plants
    // draw (not the whole region) — then one instanced draw of the visible subset.
    ctx.world.each<InstanceGroup>(
        [&](Entity, InstanceGroup& g) {
            if (g.transforms.empty()) return;
            if (!frustum.containsSphere(g.boundsCenter, g.boundsRadius)) return;
            BoundingSphere mb = ctx.renderer.getMeshBounds(g.mesh);
            std::vector<Mat4> visible =
                frustumCullInstances(g.transforms, frustum, mb.center, mb.radius);
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

