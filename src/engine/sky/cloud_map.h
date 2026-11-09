#ifndef RAYTRACER_ENGINE_SKY_CLOUD_MAP_H
#define RAYTRACER_ENGINE_SKY_CLOUD_MAP_H

// THE WEATHER MAP THAT PLACES THE CLOUDS (ADR-0386; Nubis, Schneider 2015). A square of the sky round the camera, in
// AIR coordinates (the world plus the clouds' drift: the map drifts with them, so the wind alone never rebakes it),
// where every cloud is a CELL: a soft, noise-warped footprint of one of the book's four types, its own size and
// height. Each type's cells are scattered over its own grid, how many a square holds and where by a hash of the square
// and the seed -- no tile anywhere, and a cell's fate never depends on the map's window, so two bakes agree where they
// meet. As many as the weather's coverage asks for (a Poisson field: overlapping, an overcast closes into a deck), of
// the types its range spreads over. RGBA8, row by row:
//   R  the footprint, 0..1 (the march's coverage: 0 clear air, 1 a cloud's heart)
//   G  the type there, its slot on the axis (0, 1/3, 2/3, 1)
//   B  the cell's own top, as a fraction of its type's height (small cumulus are shallow)
//   A  the CIRRUS: the high ice veils' cover there, streaked along cloudCirrusSweep(seed) (clouds.frag reads it at
//      the 9 km sheet)

#include "cloud_book.h"
#include <cstdint>
#include <vector>

namespace engine {

struct CloudMapInputs {
    float coverage = 0.25f;               // the share of the sky under cloud (the weather's)
    float typeLo = 0.5f, typeHi = 0.8f;   // the weather's range along the type axis
    float cirrus = 0.0f;                  // the cirrus's cover, 0..1
    uint32_t seed = 1;
    double centreX = 0.0, centreZ = 0.0;  // the camera, in the air (world + drift)
};

struct CloudMapImage {
    int size = 0;
    double originX = 0.0, originZ = 0.0;   // the map's corner, in the air
    float extent = 0.0f;                   // metres across
    std::vector<uint8_t> rgba;
    CloudMapInputs inputs;                 // what it was baked from
    float bakeMs = 0.0f;                   // how long the bake took (the system's log)
};

// The direction the cirrus streaks lie along, a unit (x, z), by the seed (the jet stream's, for now)
void cloudCirrusSweep(uint32_t seed, float& x, float& z);
// Each slot's share of the cells for a type range (sums to 1): the overlap of [lo, hi] with the slot's own third
std::array<float, 4> cloudTypeWeights(float typeLo, float typeHi);
// Bake the map, on the calling thread (a few hundred ms for 1024^2: CloudMapSystem runs it beside the frame)
void bakeCloudMap(const CloudBook& book, const CloudMapInputs& in, CloudMapImage& out);
// The share of the map's texels at or over `footprint` -- what the coverage came to (tests, the lab)
float cloudMapCover(const CloudMapImage& map, float footprint = 0.5f);

}  // namespace engine

#endif
