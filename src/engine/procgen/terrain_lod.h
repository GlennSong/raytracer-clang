#ifndef RAYTRACER_ENGINE_TERRAIN_LOD_H
#define RAYTRACER_ENGINE_TERRAIN_LOD_H

#include "terrain.h"   // TerrainParams, Noise, RenderMesh, terrainHeight
#include <cstdint>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace engine {

// CDLOD geometric LOD for heightfield terrain (ADR-0036, open-world Phase 1c).
// The world is a square of half-extent `worldHalf` centred on the origin, cut into
// a restricted quadtree: the root covers the whole world at the coarsest level,
// leaves are level 0 (finest, smallest, nearest the camera). Distant nodes are
// larger and carry the same triangle budget, so the far field is cheap. Adjacent
// nodes differ by at most one LOD level; the seams between them are hidden by a
// per-vertex morph (CDLOD) instead of skirts.

// One selected node: a square world-XZ region rendered at a single LOD level.
// level 0 = finest; larger = coarser. size = worldHalf*2 / 2^(numLods-1-level).
struct LodNode {
    float minX = 0.0f, minZ = 0.0f;   // world-space min corner (XZ)
    float size = 0.0f;                // world units per side
    int   level = 0;                  // LOD level (0 = finest)
};

// CDLOD visibility ranges: ranges[L] is the max camera distance at which level L is
// used (a node subdivides when the camera is nearer than the finer level's range).
// Monotonically increasing; the default doubles each level: ranges[L] = leafRange·2^L.
std::vector<float> defaultLodRanges(int numLods, float leafRange);

// Build ranges sized to the world so the <=1-level-neighbour invariant holds. CDLOD
// stays restricted (adjacent nodes within one level) only when each level's range is
// at least ~2x its node size; since both node size and range double per level, one
// valid ratio fixes every level. leafNodeSize = worldHalf*2 / 2^(numLods-1);
// ranges[0] = leafNodeSize * max(2, rangeFactor), doubling thereafter. Prefer this
// over hand-picking `leafRange`.
std::vector<float> lodRangesForWorld(float worldHalf, int numLods,
                                     float rangeFactor = 2.5f);

// The morph band (camera distances) over which a level-L node's vertices morph
// toward the next-coarser level: morphK ramps 0 (full detail) -> 1 (collapsed to
// the coarse silhouette) across [start, end]. end == ranges[L]; start == ranges[L-1]
// (0 for level 0). Fed to the terrain vertex shader per node.
struct MorphRange {
    float start = 0.0f, end = 1.0f;
};
MorphRange lodMorphRange(int level, const std::vector<float>& ranges);

// Select the quadtree cut for a camera at world XZ (camX, camZ). Output tiles the
// world with no gaps/overlaps; adjacent nodes differ by <=1 level (guaranteed by the
// nearest-point box-distance criterion). `numLods` >= 1; with numLods==1 the whole
// world is a single level-0 node.
std::vector<LodNode> selectLodNodes(float worldHalf, int numLods,
                                    const std::vector<float>& ranges,
                                    float camX, float camZ);

// Readiness-gated selection for async mesh streaming: identical descent to
// selectLodNodes, except a node splits into its four children only when
// `ensure` reports ALL FOUR renderable — otherwise the (renderable) parent is
// emitted instead. `ensure(child)` returns true when that node's mesh is
// available now (the callee may build it on the spot — it does for the coarse
// levels); returning false is the callee's cue to start an async build. All
// four children are probed even after one misses, so every absent sibling gets
// requested in the same frame. Every emitted node had ensure() == true, so the
// cut is always fully renderable: streaming shows temporarily coarser terrain,
// never holes. If even the root isn't renderable the cut is empty.
std::vector<LodNode> selectLodNodesGated(
    float worldHalf, int numLods, const std::vector<float>& ranges,
    float camX, float camZ, const std::function<bool(const LodNode&)>& ensure);

// Distance in the XZ plane from a point to the nearest point of an axis-aligned
// box [minX, minX+size] x [minZ, minZ+size]; 0 if the point is inside. Exposed for
// testing the selection invariant.
float distanceToBoxXZ(float px, float pz, float minX, float minZ, float size);

// Build the render mesh for one CDLOD node at `gridRes` x `gridRes` cells (gridRes
// is forced even so the coarser grid aligns on even indices). Reuses the shared
// height field, so the surface matches `generateTerrainChunks`. The CDLOD morph
// target — the world position this vertex collapses to on the next-coarser grid
// (only the height differs on a regular grid) — is baked into `Vertex::tangent`;
// the terrain vertex shader lerps `position -> tangent` by the per-vertex morph
// factor. Returns the mesh plus the node's tight world AABB.
struct LodNodeMesh {
    RenderMesh mesh;
    Vec3 boundsMin, boundsMax;
};
// `normalEps` is the central-difference offset for the analytic normals. Pass the
// SAME value for every node in a world (the finest leaf step is a good choice) so a
// shared edge position gets an identical normal regardless of the two tiles' LOD —
// otherwise a per-tile eps gives different normals on each side and the LOD boundary
// shows a shading seam. 0 falls back to the node's own grid step (per-tile; seams).
LodNodeMesh generateLodNodeMesh(const TerrainParams& params, const Noise& noise,
                                const LodNode& node, int gridRes,
                                double normalEps = 0.0);

// Bookkeeping for streaming node meshes off the render thread (JobSystem,
// ADR-0014). The render thread `begin`s a cache key before enqueueing its build
// job (dedupes against jobs already in flight), workers `complete` finished
// builds into a mutex-guarded queue, and the render thread `drain`s them each
// frame. A result is tagged with the config revision it was built against;
// drain drops results whose revision is stale (a re-conform landed while the
// job ran), so no stale mesh can reach the cache. `invalidate` on a revision
// bump forgets the in-flight set so the same keys can be re-requested at the
// new revision — a still-flying old job then neither blocks the re-request nor
// corrupts its accounting (its entry only clears when revisions match). Only
// `complete` may be called off the render thread.
class LodMeshStream {
public:
    struct Result {
        int64_t key = 0;
        uint32_t revision = 0;
        LodNodeMesh built;
    };

    bool begin(int64_t key, uint32_t revision);   // false: already in flight
    void complete(Result r);                      // worker thread
    // Finished builds whose revision matches; stale results are discarded.
    std::vector<Result> drain(uint32_t currentRevision);
    void invalidate();
    std::size_t inFlightCount() const { return inFlight_.size(); }

private:
    std::unordered_map<int64_t, uint32_t> inFlight_;   // key -> revision at enqueue
    std::mutex doneMutex_;
    std::vector<Result> done_;
};

}  // namespace engine

#endif
