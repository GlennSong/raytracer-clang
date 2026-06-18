#ifndef RAYTRACER_ENGINE_RENDER_SYSTEM_H
#define RAYTRACER_ENGINE_RENDER_SYSTEM_H

#include "../system.h"

namespace engine {

// Cull a group's instance transforms against the frustum (per instance), using
// the shared mesh's local bounds (center + radius). With maxDistance > 0, also drop
// instances farther than that from cameraPos (cheap LOD-0 distance cull). Returns
// the visible subset's world matrices. Free + pure so the draw path is unit-testable.
std::vector<Mat4> frustumCullInstances(const std::vector<Mat4>& transforms,
                                       const Frustum& frustum,
                                       const Vec3& meshCenter, Real meshRadius,
                                       const Vec3& cameraPos = Vec3(),
                                       Real maxDistance = 0);

// Draws the world each frame using the RenderView's camera and lights, with
// per-entity interpolation for smooth motion. Also ramps exposure from input.
// Application brackets render() with begin/endFrame, so this only emits draws.
class RenderSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    float exposure = 0.5f;
};


}  // namespace engine

#endif
