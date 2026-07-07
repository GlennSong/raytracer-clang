#ifndef RAYTRACER_ENGINE_DEBUG_DRAW_SYSTEM_H
#define RAYTRACER_ENGINE_DEBUG_DRAW_SYSTEM_H

#include "../system.h"

namespace engine {

// Draws the frame's accumulated DebugDraw lines (ADR-0067): bakes them into
// one camera-facing ribbon mesh (buildDebugLineMesh — headless-tested) and
// submits it through the ordinary AssetManager/Renderer path with
// FLAG_OVERLAY, so debug lines draw on top of the scene on every backend
// without a dedicated line pipeline. Register it after RenderSystem.
class DebugDrawSystem : public System {
public:
    void render(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    // The mesh is rebuilt every frame; the previous frames' GPU meshes are
    // released two frames late so a double/triple-buffered backend never has
    // an in-flight command buffer referencing a freed mesh.
    MeshHandle retired[2];
    int cursor = 0;
};


}  // namespace engine

#endif
