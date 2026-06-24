#ifndef RAYTRACER_ENGINE_TERRAIN_LOD_SYSTEM_H
#define RAYTRACER_ENGINE_TERRAIN_LOD_SYSTEM_H

#include "../system.h"
#include "../../renderer/renderer.h"   // MeshHandle
#include "../physics/physics_world.h"  // PhysicsBodyId

#include <cstdint>
#include <unordered_map>

namespace engine {

class PhysicsSystem;

// Drives CDLOD heightfield terrain (ADR-0036). Each frame: read the level's
// TerrainLodConfig, select the quadtree node cut for the camera, lazily generate +
// upload each node's mesh (cached across frames), frustum-cull by the node AABB, and
// submit the visible nodes via Renderer::drawTerrain with their morph bands. The
// vertex morph (Metal terrain pipeline) hides the seams between adjacent LOD levels.
//
// When constructed with a PhysicsSystem (play, not edit), it also maintains a moving
// window of static triangle-mesh colliders for the leaf-level nodes near the player,
// so the player walks on the surface. The window follows the player; nodes that fall
// outside it are freed (addMesh / removeBody directly, so no ECS body churn).
//
// Node render meshes are cached and kept; distance/budget eviction is Phase 3.
class TerrainLodSystem : public System {
public:
    explicit TerrainLodSystem(PhysicsSystem* physics = nullptr) : physics_(physics) {}

    void fixedUpdate(FrameContext& ctx) override;   // collider window
    void render(FrameContext& ctx) override;          // selection + draw
    void onStop(FrameContext& ctx) override;          // free collider bodies

private:
    struct CachedNode {
        MeshHandle mesh;
        Vec3 boundsMin;
        Vec3 boundsMax;
        uint64_t lastUsed = 0;   // frame this node was last in the selected cut
    };
    std::unordered_map<int64_t, CachedNode> cache_;
    uint64_t frame_ = 0;
    // Release a node's GPU mesh after it has been out of the cut this many frames.
    // Generous enough that turning around / LOD-flip flicker doesn't thrash the
    // cache, small enough to bound memory as the player explores.
    static constexpr uint64_t kEvictAfterFrames = 180;

    PhysicsSystem* physics_ = nullptr;
    std::unordered_map<int64_t, PhysicsBodyId> colliders_;   // leaf key -> body
    // Last TerrainLodConfig::revision the cache / collider window were built at; a
    // mismatch (a re-conform) forces a rebuild from the new params.
    uint32_t cacheRevision_ = 0;
    uint32_t colliderRevision_ = 0;
};

}  // namespace engine

#endif
