#ifndef RAYTRACER_ENGINE_TERRAIN_LOD_SYSTEM_H
#define RAYTRACER_ENGINE_TERRAIN_LOD_SYSTEM_H

#include "../system.h"
#include "../../renderer/renderer.h"   // MeshHandle
#include "../physics/physics_world.h"  // PhysicsBodyId

#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>

namespace engine {

class PhysicsSystem;
class LodMeshStream;        // procgen/terrain_lod.h
struct TerrainGenInputs;    // per-revision recipe snapshot (terrain_lod_system.cpp)

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
// Node RENDER meshes stream in off-thread (P0.4): the LOD cut only refines into
// children whose meshes are already cached (selectLodNodesGated), missing nodes
// are built on the JobSystem and inserted at frame start — temporarily coarser
// terrain, never holes. The root and its children build synchronously so there
// is always ground. COLLIDER meshes stay synchronous (fixedUpdate): the player
// must never out-run their floor. RT_SYNC_CDLOD=1 restores the fully
// synchronous render path for bisection.
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

    // Async streaming state: the read-only recipe workers build from (snapshotted
    // per revision so jobs never chase the live component) and the in-flight /
    // completed bookkeeping. shared_ptr so a job still running when the system
    // dies keeps its inputs and queue alive.
    std::shared_ptr<const TerrainGenInputs> genInputs_;
    std::shared_ptr<LodMeshStream> stream_;
    static constexpr std::size_t kMaxInFlight = 16;

    PhysicsSystem* physics_ = nullptr;
    std::unordered_map<int64_t, PhysicsBodyId> colliders_;   // leaf key -> body
    // Last TerrainLodConfig::revision the cache / collider window were built at; a
    // mismatch (a re-conform) forces a rebuild from the new params.
    uint32_t cacheRevision_ = 0;
    uint32_t colliderRevision_ = 0;
};

}  // namespace engine

#endif
