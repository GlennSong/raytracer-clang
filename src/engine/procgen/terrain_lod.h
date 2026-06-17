#ifndef RAYTRACER_ENGINE_TERRAIN_LOD_H
#define RAYTRACER_ENGINE_TERRAIN_LOD_H

#include "terrain.h"   // TerrainParams, Noise, RenderMesh, terrainHeight
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

}  // namespace engine

#endif
