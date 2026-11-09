#ifndef RAYTRACER_HASH_H
#define RAYTRACER_HASH_H

// STATELESS HASHES (#184): a seed, a cell, a name in; well-mixed bits out -- for what must come out the same every run
// (a building's colour, a tree's lean, which furniture a room gets). The seeded STREAM is proc_rng.h's. Add a hash
// here rather than writing one into the file that needs it: the integer mixer below had been written twenty times.

#include <cstdint>

namespace engine {

// LOWBIAS32 (Chris Wellons' hash-prospector "lowbias32": xor-shift, multiply, twice): every input bit reaches every
// output bit. Bit-for-bit the mixer every copy used.
inline uint32_t lowbias32(uint32_t h) {
    h ^= h >> 16;
    h *= 0x7feb352du;
    h ^= h >> 15;
    h *= 0x846ca68bu;
    h ^= h >> 16;
    return h;
}

// TWO NUMBERS' bits (a seed and an index, a seed and a salt): a PCG-style multiply-add of each, then lowbias32.
inline uint32_t hashPair(uint32_t a, uint32_t b) { return lowbias32(a * 747796405u + b * 2891336453u + 0x9E3779B9u); }

// A GRID CELL's bits: its two indices and a seed multiplied apart (the golden ratio's and murmur3's odd constants),
// then lowbias32. (A one-index lattice -- rock beds -- passes seed 0.)
inline uint32_t hashCell(uint32_t a, uint32_t b, uint32_t seed) {
    return lowbias32(a * 0x9E3779B1u ^ b * 0x85EBCA77u ^ seed * 0xC2B2AE3Du);
}

// A hash's low 24 bits as a number in [0, 1) (never 1).
inline double unit24(uint32_t h) { return (h & 0xFFFFFFu) / 16777216.0; }

// FROZEN HASHES (2026-11-08, the value noise made one: procgen/noise.h). Each kept bit for bit only because what it
// lays is baked -- a tree's textures and gnarl, the lanes' recipe terrain, the procedural textures, the city plans, the
// street planting -- and a new hash would re-lay it without a tag bump. (The blend is frozen too where its rounding
// differed: procgen/noise.h FrozenTerrainRecipeBlend, FrozenTreeTextureBlend.) Retire each for hashCell / hashPair at its
// bake's next tag bump.
inline float frozenTreeTextureLattice(int x, int y, uint32_t seed) {
    uint32_t h = static_cast<uint32_t>(x) * 374761393u + static_cast<uint32_t>(y) * 668265263u + seed * 362437u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return (h ^ (h >> 16)) / 4294967296.0f;
}
inline double frozenTerrainRecipeLattice(int64_t ix, int64_t iy, uint32_t seed) {
    uint64_t h = static_cast<uint64_t>(ix) * 0x9E3779B97F4A7C15ull ^ static_cast<uint64_t>(iy) * 0xC2B2AE3D27D4EB4Full ^
                 (static_cast<uint64_t>(seed) + 0x165667B19E3779F9ull);
    h ^= h >> 31; h *= 0x7FB5D329728EA185ull; h ^= h >> 27; h *= 0x81DADEF4BC2DD44Dull; h ^= h >> 33;
    return static_cast<double>(h >> 11) / 9007199254740992.0;
}
// a tree's gnarl and bark, in 3D: a node value in [0, 1)
inline float frozenGnarlLattice(int x, int y, int z, uint32_t seed) {
    uint32_t h = static_cast<uint32_t>(x) * 374761393u ^ static_cast<uint32_t>(y) * 668265263u ^
                 static_cast<uint32_t>(z) * 2246822519u ^ seed * 3266489917u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return (h ^ (h >> 16)) / 4294967296.0f;
}
// the procedural textures' (texture_field.cpp: their lattices and cells)
inline uint32_t frozenTextureFieldHash(int x, int y, uint32_t seed) {
    uint32_t h = seed * 2654435761u + static_cast<uint32_t>(x) * 40503u + static_cast<uint32_t>(y) * 668265263u + 0x9e3779b9u;
    h ^= h >> 15; h *= 0x2c1b3c6du; h ^= h >> 12; h *= 0x297a2d39u; h ^= h >> 15;
    return h;
}
// the city planner's (city_plan.cpp: its noise and its rolls)
inline uint32_t frozenCityPlanHash(int x, int y, uint32_t s) {
    uint32_t h = s ^ (static_cast<uint32_t>(x) * 0x8da6b343u) ^ (static_cast<uint32_t>(y) * 0xd8163841u);
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    return h;
}
// the clouds' noise bake (renderer/cloud_noise.h: its Perlin lattice and Worley cells; cached as "pw-clouds-v2")
inline uint32_t frozenCloudNoiseHash(int x, int y, int z, uint32_t seed) {
    uint32_t h = seed;
    h ^= static_cast<uint32_t>(x) * 0x8da6b343u;
    h ^= static_cast<uint32_t>(y) * 0xd8163841u;
    h ^= static_cast<uint32_t>(z) * 0xcb1ab31fu;
    h ^= h >> 13;
    h *= 0x9e3779b1u;
    h ^= h >> 16;
    return h;
}
// the street planting's (level_loader.cpp): a node value in [0, 1)
inline double frozenPlantingLattice(long long x, long long z, uint32_t salt) {
    uint64_t k = static_cast<uint64_t>(x) * 0x9E3779B97F4A7C15ull ^ static_cast<uint64_t>(z) * 0xC2B2AE3D27D4EB4Full ^ salt;
    k ^= k >> 31; k *= 0xBF58476D1CE4E5B9ull; k ^= k >> 29;
    return (k & 0xFFFFFF) / static_cast<double>(0x1000000);
}

}  // namespace engine

#endif
