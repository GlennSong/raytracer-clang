#include "debug_draw_system.h"

#include "../asset_manager.h"
#include "../debug_draw.h"

namespace engine {

void DebugDrawSystem::render(FrameContext& ctx) {
    MeshHandle& slot = retired[cursor];
    cursor = (cursor + 1) % 2;
    if (slot.valid()) {
        ctx.assets.releaseMesh(slot);
        slot = MeshHandle{};
    }

    if (ctx.debug.empty()) return;
    RenderMesh mesh = buildDebugLineMesh(ctx.debug.lines(),
                                         ctx.view.camera.position);
    if (mesh.vertices.empty()) return;

    MeshHandle handle = ctx.assets.acquireMesh(mesh);
    RenderMaterial material;
    material.albedo = Vec3(1, 1, 1);   // line color rides Vertex::color
    material.roughness = 1.0f;
    material.flags = RenderMaterial::FLAG_OVERLAY;
    ctx.renderer.drawMesh(handle, Mat4(), material);
    slot = handle;
}

void DebugDrawSystem::onStop(FrameContext& ctx) {
    for (MeshHandle& handle : retired) {
        if (handle.valid()) ctx.assets.releaseMesh(handle);
        handle = MeshHandle{};
    }
    cursor = 0;
}

}  // namespace engine
