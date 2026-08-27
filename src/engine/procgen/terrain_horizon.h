#ifndef RAYTRACER_ENGINE_PROCGEN_TERRAIN_HORIZON_H
#define RAYTRACER_ENGINE_PROCGEN_TERRAIN_HORIZON_H

#include "terrain.h"

#include <cstdint>
#include <functional>
#include <vector>

namespace engine {

// TERRAIN HORIZON (device: "when the sun set behind the mountains ... I
// thought I might start seeing more darkness right away, or at least some
// shadow casting from the mountain"). Mountain shadows are kilometres long
// and the cascaded shadow maps reach 150 m — the terrain is not even a
// caster there. This is the classic answer at landscape scale: for every
// point of the terrain, the ELEVATION OF THE HORIZON in the sun's azimuth
// (how high the ridge stands between here and the sun). A fragment is in
// mountain shadow when the sun sits below that horizon; the camera's own
// texel says whether the disc is behind the ridge. One texture, one lookup.
//
// Two stages so the expensive part runs once: a height RASTER of the
// terrain (the analytic field sampled on a grid), then a horizon map for
// one azimuth — a march along that bearing per texel, cheap enough to
// redo whenever the sun has moved a couple of degrees.

struct HeightRaster {
    int    size = 0;             // texels per side
    double originX = 0, originZ = 0;   // world XZ of texel (0, 0)'s corner
    double extent = 0;           // world metres per side
    std::vector<float> height;   // size * size, row-major (z rows, x columns)

    double texel() const { return size > 0 ? extent / size : 0.0; }
    // Bilinear height at a world XZ (clamped to the raster).
    double at(double worldX, double worldZ) const;
    bool empty() const { return height.empty(); }
};

// Sample `heightAt(x, z)` over a square [originX, originX + extent) x
// [originZ, originZ + extent) at `size` texels per side.
HeightRaster rasterizeHeights(const std::function<double(double, double)>& heightAt,
                              double originX, double originZ, double extent, int size);

struct HorizonMap {
    int    size = 0;
    double originX = 0, originZ = 0, extent = 0;
    double azimuthDeg = 0;       // compass bearing the horizon was marched toward
    // sin(horizon elevation) per texel, encoded: 0 = kEncodeLo .. 255 =
    // kEncodeHi. A flat plain toward the sun decodes to ~0; a 30-deg ridge
    // to 0.5. The shader compares the sun's direction.y against it.
    std::vector<uint8_t> sinElevation;
    static constexpr double kEncodeLo = -0.10;
    static constexpr double kEncodeHi = 0.90;

    static uint8_t encode(double sinElev);
    static double decode(uint8_t v);
    // Decoded sin(horizon elevation) at a world XZ (nearest texel).
    double sinElevationAt(double worldX, double worldZ) const;
    bool empty() const { return sinElevation.empty(); }
};

// March every texel of `raster` toward `azimuthDeg` (0 north = -z, 90 east
// = +x) out to `maxDistance` metres, in steps of one raster texel, keeping
// the highest elevation angle of the terrain above the start point (the
// start is lifted by `eyeHeight` so a texel's own slope does not shadow
// it). `parallelRow` runs the per-row body across threads when given.
HorizonMap computeHorizonMap(const HeightRaster& raster, double azimuthDeg,
                             double maxDistance, double eyeHeight = 2.0,
                             const std::function<void(int, const std::function<void(int)>&)>*
                                 parallelRows = nullptr);

}  // namespace engine

#endif
