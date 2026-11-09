#ifndef RAYTRACER_RENDERER_LIGHT_CLUSTERS_H
#define RAYTRACER_RENDERER_LIGHT_CLUSTERS_H

// CLUSTERED LIGHTS (renderer-realism plan phase 1; Glenn, 2026-11-07: "look into the deferred rendering so we could
// have many, many light sources"). The view frustum cut into froxels -- `tilesX` x `tilesY` screen tiles by `slices`
// depth slices, exponential in view depth -- and each light listed in every froxel its sphere of influence touches.
// A pixel then shades with only its froxel's lights, not all of them: a street of a thousand lamps costs a pixel the
// handful near it. The same lists serve a forward pass (each fragment finds its froxel) or a deferred lighting pass.
// Pure (CPU), so the tests can hold it; the renderer uploads the result.

#include "../rt_math.h"
#include <cstdint>
#include <vector>

namespace engine {

struct ClusterGrid {
    int tilesX = 16, tilesY = 9, slices = 24;
    float zNear = 0.5f, zFar = 2000.0f;   // view depth the slices span (m); nearer is slice 0, farther the last
    int cells() const { return tilesX * tilesY * slices; }
    // The slice a view depth falls in (exponential: each slice the same ratio deeper), clamped to the grid.
    int sliceOf(float depth) const;
    float sliceNear(int s) const;
};

struct ClusterLight {
    Vec3 position;     // world
    float range = 0;   // its influence ends here (the shader's window)
};

struct ClusterLists {
    std::vector<uint32_t> offsetCount;   // per cell: offset into `indices` (low 24 bits) and count (high 8 bits)
    std::vector<uint32_t> indices;       // light indices, cell by cell
    int maxInCell = 0;                   // the busiest cell's count (before the cap)
    int capped = 0;                      // cells that had more than the cap and kept the nearest `cap`
};

// Bin `lights` for a camera: `view` the world->view matrix (view looks down -Z), `tanHalfY` the tangent of half the
// vertical field of view, `aspect` width / height. At most `cap` lights per cell (the nearest the cell's middle stay).
void buildClusters(const ClusterGrid& grid, const Mat4& view, float tanHalfY, float aspect,
                   const std::vector<ClusterLight>& lights, ClusterLists& out, int cap = 64);

}  // namespace engine

#endif
