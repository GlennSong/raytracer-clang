#ifndef RAYTRACER_ENGINE_TERRAIN_LOD_SYSTEM_H
#define RAYTRACER_ENGINE_TERRAIN_LOD_SYSTEM_H

#include "../system.h"
#include "../../renderer/renderer.h"   // MeshHandle

#include <cstdint>
#include <unordered_map>

namespace engine {

// Drives CDLOD heightfield terrain (ADR-0036). Each frame: read the level's
// TerrainLodConfig, select the quadtree node cut for the camera, lazily generate +
// upload each node's mesh (cached across frames), frustum-cull by the node AABB, and
// submit the visible nodes via Renderer::drawTerrain with their morph bands. The
// vertex morph (Metal terrain pipeline) hides the seams between adjacent LOD levels.
//
// Node meshes are cached and kept; distance/budget eviction is Phase 3 streaming.
class TerrainLodSystem : public System {
public:
    void render(FrameContext& ctx) override;

private:
    struct CachedNode {
        MeshHandle mesh;
        Vec3 boundsMin;
        Vec3 boundsMax;
    };
    std::unordered_map<int64_t, CachedNode> cache_;
};

}  // namespace engine

#endif
