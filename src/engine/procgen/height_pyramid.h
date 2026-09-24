#ifndef RAYTRACER_ENGINE_HEIGHT_PYRAMID_H
#define RAYTRACER_ENGINE_HEIGHT_PYRAMID_H

// A BAKED HEIGHT PYRAMID (ADR-0095, docs/world-streaming-plan.md §3).
//
// The final ground -- natural terrain, carved roads, block terraces, pads, every flatten --
// sampled once into a quadtree of square HEIGHT TILES instead of evaluated per vertex at
// run time. Level 0 is the finest; each level up doubles the cell. A tile is 128 x 128
// cells, 129 x 129 samples: its last row and column are its neighbour's first, so tiles of
// one level meet exactly. Every level samples the same height function at its own grid
// points, so a parent's samples ARE its children's even samples (a CDLOD morph target is
// always the true parent).
//
// SPARSE. A tile's four children are stored only where they would change the drawn surface
// by more than `tolerance`: its ERROR is the largest vertical difference between the tile,
// interpolated as it is drawn, and the samples of the level below. Plains and sea stop at
// coarse levels; streets, pad edges and ridges go deep -- disk grows with content, not area.
//
// Samples are uint16, quantized per tile (h = minH + q * step). Normals are not stored; the
// renderer derives them from neighbouring samples.
//
// The pyramid knows nothing about terrain recipes: it is built from a height function
// h(x, z, step), which is handed the level's cell so a caller can sample the way its mesher
// does (terrain_lod.h: lodVertexHeight dilates flatten footprints by the cell).

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine {
class JobSystem;
namespace pyramid {

constexpr int kTileCells = 128;
constexpr int kTileSamples = kTileCells + 1;

struct TileKey {
    int level = 0;
    int tx = 0, tz = 0;   // tile index at its level, from the pyramid's origin
    bool operator==(const TileKey& o) const { return level == o.level && tx == o.tx && tz == o.tz; }
};
struct TileKeyHash {
    std::size_t operator()(const TileKey& k) const {
        return (static_cast<std::size_t>(k.level) * 0x9E3779B97F4A7C15ull) ^
               (static_cast<std::size_t>(static_cast<uint32_t>(k.tx)) << 32) ^
               static_cast<std::size_t>(static_cast<uint32_t>(k.tz));
    }
};

// The square the pyramid covers: [originX, originX + extent] x [originZ, originZ + extent],
// one tile at the top level.
struct PyramidSpec {
    double originX = 0.0, originZ = 0.0;
    double cell0 = 1.0;        // level-0 cell (m)
    int levels = 1;            // top level = levels - 1
    double tolerance = 0.02;   // children are stored only where the parent errs more (m)

    double cell(int level) const { return cell0 * static_cast<double>(1ll << level); }
    double tileSize(int level) const { return cell(level) * kTileCells; }
    double extent() const { return tileSize(levels - 1); }
    // The smallest pyramid of `cell0` cells that covers the rectangle.
    static PyramidSpec covering(double minX, double minZ, double maxX, double maxZ,
                                double cell0, double tolerance);
};

struct HeightTile {
    TileKey key;
    float minH = 0.0f, maxH = 0.0f;
    float step = 0.0f;         // quantization step (m)
    float error = 0.0f;        // vs the level below, as drawn (0 at level 0)
    bool hasChildren = false;
    std::vector<uint16_t> q;   // kTileSamples^2, row-major (z rows, x columns)

    double at(int i, int j) const {
        return static_cast<double>(minH) + static_cast<double>(q[static_cast<std::size_t>(j) * kTileSamples + i]) * step;
    }
    // Height at local cell coordinates (0..kTileCells), interpolated across the triangle
    // split the mesher emits: (a,b,d) when u >= v, else (a,d,c) -- a the cell's min corner,
    // d the max (terrain_lod.cpp, lodSurfaceHeight).
    double sampleLocal(double gx, double gz) const;
};

struct Pyramid {
    PyramidSpec spec;
    std::unordered_map<TileKey, HeightTile, TileKeyHash> tiles;

    const HeightTile* find(const TileKey& k) const {
        auto it = tiles.find(k);
        return it == tiles.end() ? nullptr : &it->second;
    }
    // The finest stored tile covering (x, z), or null outside the pyramid.
    const HeightTile* finestAt(double x, double z) const;
    // THE DRAWN GROUND at (x, z): the finest stored tile, interpolated as drawn.
    double height(double x, double z) const;
    std::size_t sampleBytes() const { return tiles.size() * sizeof(uint16_t) * kTileSamples * kTileSamples; }
};

// h(x, z, step): the ground at (x, z) for a level whose cell is `step`. Must be safe to call
// from several threads at once when a JobSystem is passed.
using HeightFn = std::function<double(double x, double z, double step)>;

// Build top-down: the top tile, then each tile's four children, kept where the parent's
// drawn surface errs from them by more than the tolerance (they are then refined in turn).
Pyramid buildPyramid(const PyramidSpec& spec, const HeightFn& h, JobSystem* jobs = nullptr);

// ---- the bundle codec (ADR-0084: a section per tile) --------------------------------------
constexpr uint32_t kTileCodecMagic = 0x54485452;   // "RTHT"
constexpr uint32_t kTileCodecVersion = 1;          // raw uint16 samples; 2 will add compression
std::vector<uint8_t> encodeTile(const HeightTile& t);
bool decodeTile(const uint8_t* data, std::size_t size, HeightTile& out);
// "terrain/L<level>/<tx>_<tz>"
std::string tileSectionName(const TileKey& k, const std::string& prefix = "terrain");

}  // namespace pyramid
}  // namespace engine

#endif
