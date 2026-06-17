#include "terrain_lod_system.h"
#include "../components.h"
#include "../asset_manager.h"
#include "../procgen/terrain_lod.h"
#include "../procgen/noise.h"

#include <cmath>
#include <string>

namespace engine {

namespace {
// Exact cache key for a node: level (4 bits) + integer grid coords (26 bits each,
// signed). Node corners are at integer multiples of the node size, so ix/iz round
// cleanly. The grid coord ranges (|coord| < 2^(numLods-1)) fit well inside 26 bits.
int64_t nodeKey(const LodNode& n) {
    int ix = static_cast<int>(std::lround(n.minX / n.size));
    int iz = static_cast<int>(std::lround(n.minZ / n.size));
    return (static_cast<int64_t>(n.level) << 52) |
           ((static_cast<int64_t>(ix) & 0x3FFFFFF) << 26) |
           (static_cast<int64_t>(iz) & 0x3FFFFFF);
}
}  // namespace

void TerrainLodSystem::render(FrameContext& ctx) {
    // One CDLOD terrain per level. Grab the first config (if any).
    const TerrainLodConfig* cfg = nullptr;
    ctx.world.each<TerrainLodConfig>(
        [&](Entity, TerrainLodConfig& c) { if (!cfg) cfg = &c; });
    if (!cfg) return;

    const CameraState& cam = ctx.view.camera;

    // Forward-Z frustum (the view volume is identical to the GPU's reverse-Z; CPU
    // culling stays forward-Z — ADR-0034/0035), matching RenderSystem.
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = (cam.projection == CameraProjection::Perspective)
        ? Mat4::perspective(degreesToRadians(cam.fovDegrees), cam.aspectRatio,
                            cam.nearPlane, cam.farPlane)
        : Mat4::orthographic(cam.orthoHeight, cam.aspectRatio,
                             cam.nearPlane, cam.farPlane);
    Frustum frustum = Frustum::fromViewProjection(proj * view);

    std::vector<float> ranges =
        lodRangesForWorld(cfg->worldHalf, cfg->numLods, cfg->rangeFactor);
    std::vector<LodNode> nodes = selectLodNodes(
        cfg->worldHalf, cfg->numLods, ranges,
        static_cast<float>(cam.position.x), static_cast<float>(cam.position.z));

    // World-consistent normal eps (the finest leaf step), the SAME for every node
    // so normals match across LOD boundaries — no shading seam (ADR-0036).
    float leafSize = (cfg->worldHalf * 2.0f) /
                     static_cast<float>(1 << std::max(0, cfg->numLods - 1));
    double normalEps = leafSize / static_cast<float>(std::max(2, cfg->gridRes));

    Noise noise(cfg->seed);
    for (const LodNode& node : nodes) {
        int64_t key = nodeKey(node);
        auto it = cache_.find(key);
        if (it == cache_.end()) {
            LodNodeMesh built =
                generateLodNodeMesh(cfg->params, noise, node, cfg->gridRes, normalEps);
            CachedNode cached;
            cached.mesh = ctx.assets.acquireMesh(built.mesh,
                                                 "cdlod_" + std::to_string(key));
            cached.boundsMin = built.boundsMin;
            cached.boundsMax = built.boundsMax;
            it = cache_.emplace(key, cached).first;
        }
        const CachedNode& cn = it->second;
        if (!frustum.containsAABB(cn.boundsMin, cn.boundsMax)) continue;

        MorphRange m = lodMorphRange(node.level, ranges);
        ctx.renderer.drawTerrain(cn.mesh, cfg->material, m.start, m.end);
    }
}

}  // namespace engine
