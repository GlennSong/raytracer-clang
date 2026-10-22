#ifndef RAYTRACER_ENGINE_PROCGEN_TERRAIN_WORLD_MAP_H
#define RAYTRACER_ENGINE_PROCGEN_TERRAIN_WORLD_MAP_H

// TERRAIN WORLD MAPS: textures laid over the whole terrain, baked once from the ground and read per pixel by the lit
// pass where the CDLOD mesh is too coarse to carry what they hold. A far tile's vertices are 16-64 m apart, so what
// the ground does between them -- its slope, what covers it -- is lost in a per-vertex value however fine the ground
// itself is.
//
// THE NORMAL MAP (ADR-0205; Glenn, 2026-10-19: "increase the detail on the mountains so that they look more rocky"):
// the drawn ground's slope per texel, two channels (the normal's x and z, y up, 0..255 for -1..1). Each texel holds
// the MEAN slope over a two-texel square round its centre (exactly: the square's edges' mean heights, read every half
// texel), so a texel shows the ground it covers instead of aliasing the bumps between samples -- down to half a texel
// (the island's 2.4 m texels sample every 1.2 m; its drawn ground has nothing finer than its 1 m cells); the GPU's
// mips average it further for the far distance.
//
// THE COVER MAP (ADR-0208): what covers the ground per texel -- the ground-cover weights the vertices carry (grass,
// dirt, sand, snow; rock the remainder), four channels -- so a far mountain's rock, scree and meadow meet where they
// do rather than smeared across a 32 m triangle.

#include "../../rt_math.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace engine {

struct TerrainWorldMap {
    int size = 0;                       // texels a side
    double originX = 0, originZ = 0;    // the world corner of texel (0, 0)
    double extent = 0;                  // metres a side
    int channels = 2;
    std::vector<uint8_t> texels;        // size * size * channels, row-major (z rows, x columns)

    double texel() const { return size > 0 ? extent / size : 0.0; }
    bool empty() const { return texels.empty(); }
    // a two-channel normal map's decoded normal under (x, z) (nearest; clamped to the map)
    Vec3 normalAt(double x, double z) const;
    // channel `c` of the texel under (x, z), 0..1 (nearest; clamped)
    double at(double x, double z, int c) const;
};

// The normal map over [originX, originX + extent)^2 at `size` texels a side from `height(x, z)` (thread-safe: it is
// called from every hardware thread).
TerrainWorldMap bakeTerrainNormalMap(const std::function<double(double, double)>& height, double originX,
                                     double originZ, double extent, int size);
// Any map of `channels` bytes a texel: `texel(x, z, out)` at each texel's centre (thread-safe, as above).
TerrainWorldMap bakeTerrainWorldMap(int channels, const std::function<void(double, double, uint8_t*)>& texel,
                                    double originX, double originZ, double extent, int size);

// The disk cache (cache/terrain/<key>.wmap): false on a miss or a mismatched file.
bool writeTerrainWorldMap(const TerrainWorldMap& m, uint64_t key, const std::string& path);
bool readTerrainWorldMap(const std::string& path, uint64_t key, TerrainWorldMap& out);

}  // namespace engine

#endif
