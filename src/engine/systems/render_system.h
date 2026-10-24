#ifndef RAYTRACER_ENGINE_RENDER_SYSTEM_H
#define RAYTRACER_ENGINE_RENDER_SYSTEM_H

#include "../system.h"
#include "draw_index.h"

#include <chrono>
#include <cstdint>
#include <unordered_map>
#include <vector>

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

// Same, writing into a caller-owned scratch buffer instead of allocating. The
// draw path uses this one; the returning overload above stays for the tests.
void frustumCullInstances(const std::vector<Mat4>& transforms,
                          const Frustum& frustum,
                          const Vec3& meshCenter, Real meshRadius,
                          const Vec3& cameraPos, Real maxDistance,
                          std::vector<Mat4>& out, Real minDistance = 0);

// The tier crossfade's dither (FLAG_LOD_BAND, ADR-0226): `m` drawn at `share` of its pixels, uniformly over a
// drawable `dist` metres away -- coming in, or going out on the complementary pixels.
void tierFadeBand(RenderMaterial& m, double dist, float share, bool coming);

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
    // Reused across groups and frames so per-instance culling stops heap-
    // allocating the full instance count every group every frame.
    std::vector<Mat4> instanceScratch_;
    // a moving group's instances between their last step and this one (InstanceGroup::ids), and the lookup by id when
    // the order changed between bakes
    std::vector<Mat4> lerpScratch_;
    std::unordered_map<uint64_t, std::size_t> prevById_;
    DrawIndex drawIndex_;   // static drawables by world cell (draw_index.h)
    // THE TIER CROSSFADE (ADR-0226): a lodCell drawable whose distance test flips while it is on screen fades over
    // kTierFade instead of popping -- a cell's detail out as its facades come in, a stand-in box out as a streamed
    // tier lands. Per entity index: the fade (0 hidden .. 1 drawn), the mesh it was for, the frame last seen.
    struct TierFade { float a = 0.0f; uint32_t mesh = ~0u; uint64_t seen = 0; };
    std::vector<TierFade> tierFade_;
    uint64_t frameNo_ = 0;
    std::chrono::steady_clock::time_point lastRender_;
};


}  // namespace engine

#endif
