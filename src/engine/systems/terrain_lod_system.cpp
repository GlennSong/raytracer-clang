#include "terrain_lod_system.h"
#include "physics_system.h"
#include "../components.h"
#include "../asset_manager.h"
#include "../procgen/terrain_lod.h"
#include "../procgen/noise.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

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
    ++frame_;
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
        CachedNode& cn = it->second;
        cn.lastUsed = frame_;   // in the cut this frame (keep it cached)
        if (!frustum.containsAABB(cn.boundsMin, cn.boundsMax)) continue;

        MorphRange m = lodMorphRange(node.level, ranges);
        ctx.renderer.drawTerrain(cn.mesh, cfg->material, m.start, m.end);
    }

    // Evict node meshes that have been out of the cut for a while (the player moved
    // away). The cut itself is bounded, so this keeps the cache ~ a few recent cuts
    // instead of every node ever visited.
    for (auto it = cache_.begin(); it != cache_.end();) {
        if (frame_ - it->second.lastUsed > kEvictAfterFrames) {
            ctx.assets.releaseMesh(it->second.mesh);
            it = cache_.erase(it);
        } else {
            ++it;
        }
    }
}

// Static collider window: keep a triangle-mesh body under the leaf-level nodes
// around the player so they walk on the surface, freeing nodes that fall outside
// the window. Bodies are owned directly (addMesh/removeBody), not via ECS, so there
// is no entity/body churn. Runs only with physics (play mode).
void TerrainLodSystem::fixedUpdate(FrameContext& ctx) {
    if (!physics_) return;

    const TerrainLodConfig* cfg = nullptr;
    ctx.world.each<TerrainLodConfig>(
        [&](Entity, TerrainLodConfig& c) { if (!cfg) cfg = &c; });
    if (!cfg) return;

    // Centre the window on the player (the entity PlayerSystem drives).
    Vec3 player;
    bool found = false;
    ctx.world.each<Transform, ControlledBy>(
        [&](Entity, Transform& t, ControlledBy&) {
            if (!found) { player = t.position; found = true; }
        });
    if (!found) return;

    const int levels = std::max(1, cfg->numLods);
    const int leafCount = 1 << (levels - 1);                  // leaf cells per side
    const float worldHalf = cfg->worldHalf;
    const float leafSize = (worldHalf * 2.0f) / static_cast<float>(leafCount);
    const float radius =
        cfg->colliderRadius > 0.0f ? cfg->colliderRadius : leafSize * 1.5f;

    auto cellIndex = [&](float w) {
        return static_cast<int>(std::floor((w + worldHalf) / leafSize));
    };
    int ix0 = std::max(0, cellIndex(static_cast<float>(player.x) - radius));
    int ix1 = std::min(leafCount - 1, cellIndex(static_cast<float>(player.x) + radius));
    int iz0 = std::max(0, cellIndex(static_cast<float>(player.z) - radius));
    int iz1 = std::min(leafCount - 1, cellIndex(static_cast<float>(player.z) + radius));

    const double normalEps = leafSize / static_cast<float>(std::max(2, cfg->gridRes));
    Noise noise(cfg->seed);

    std::unordered_set<int64_t> desired;
    for (int iz = iz0; iz <= iz1; iz++) {
        for (int ix = ix0; ix <= ix1; ix++) {
            LodNode node{-worldHalf + ix * leafSize, -worldHalf + iz * leafSize,
                         leafSize, 0};
            int64_t key = nodeKey(node);
            desired.insert(key);
            if (colliders_.count(key)) continue;

            LodNodeMesh built = generateLodNodeMesh(cfg->params, noise, node,
                                                    cfg->gridRes, normalEps);
            std::vector<Vec3> verts;
            verts.reserve(built.mesh.vertices.size());
            for (const Vertex& v : built.mesh.vertices) verts.push_back(v.position);
            PhysicsBodyId id = physics_->physicsWorld().addMesh(
                verts, built.mesh.indices, Vec3(0, 0, 0), 0.8);
            if (id != INVALID_PHYSICS_BODY) colliders_[key] = id;
        }
    }
    for (auto it = colliders_.begin(); it != colliders_.end();) {
        if (desired.count(it->first)) { ++it; continue; }
        physics_->physicsWorld().removeBody(it->second);
        it = colliders_.erase(it);
    }
}

void TerrainLodSystem::onStop(FrameContext&) {
    if (!physics_) return;
    for (auto& kv : colliders_) physics_->physicsWorld().removeBody(kv.second);
    colliders_.clear();
}

}  // namespace engine
