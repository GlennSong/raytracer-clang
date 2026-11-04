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

}  // namespace engine

#endif
