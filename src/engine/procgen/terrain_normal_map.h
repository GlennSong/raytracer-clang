#ifndef RAYTRACER_ENGINE_PROCGEN_TERRAIN_NORMAL_MAP_H
#define RAYTRACER_ENGINE_PROCGEN_TERRAIN_NORMAL_MAP_H

// THE TERRAIN NORMAL MAP (ADR-0205; Glenn, 2026-10-19: "increase the detail on the mountains so that they look more
// rocky"). A far CDLOD tile's vertices are 16-64 m apart, so whatever the ground does between them -- the erosion's
// gullies and spurs, every crag -- is gone from its shading however fine the baked ground is. This is the drawn
// ground's slope baked once per texel over the whole terrain, a world-aligned texture the lit pass reads per pixel
// beyond the distance where the mesh grows coarser than a texel: the far mountains shade with the detail the near
// ones have, at the cost of one texture read.
//
// Each texel holds the MEAN slope over a two-texel square round its centre (exactly: the square's edges' mean
// heights, read every half texel), so a texel shows the ground it covers instead of aliasing the bumps between
// samples -- down to half a texel (the island's 2.4 m texels sample every 1.2 m; its drawn ground has nothing finer
// than its 1 m cells); the GPU's mips average it further for the far distance. Stored as the normal's x and z (y up),
// 0..255 for -1..1.

#include "../../rt_math.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace engine {

struct TerrainNormalMap {
    int size = 0;                       // texels a side
    double originX = 0, originZ = 0;    // the world corner of texel (0, 0)
    double extent = 0;                  // metres a side
    std::vector<uint8_t> xz;            // size * size * 2, row-major (z rows, x columns): the normal's x, z

    double texel() const { return size > 0 ? extent / size : 0.0; }
    bool empty() const { return xz.empty(); }
    // the decoded normal of the texel under (x, z) (nearest; clamped to the map)
    Vec3 at(double x, double z) const;
};

// Bake over [originX, originX + extent)^2 at `size` texels a side from `height(x, z)` (thread-safe: it is called
// from every hardware thread).
TerrainNormalMap bakeTerrainNormalMap(const std::function<double(double, double)>& height, double originX,
                                      double originZ, double extent, int size);

// The disk cache (cache/terrain/<key>.nmap): false on a miss or a mismatched file.
bool writeTerrainNormalMap(const TerrainNormalMap& m, uint64_t key, const std::string& path);
bool readTerrainNormalMap(const std::string& path, uint64_t key, TerrainNormalMap& out);

}  // namespace engine

#endif
