// Tileable Perlin-Worley cloud noise (CPU bake) — the 3D texture set the
// volumetric cloud shader samples (clouds.metal cl_density). The
// Schneider/Nubis-style pair, scoped v1:
//   BASE   128^3 RGBA8 — R = low-frequency Perlin fBm remapped by inverted-
//          Worley fBm (the classic "perlin-worley": PW = remap(perlin,
//          worley-1, 1, 0, 1) — billowy cauliflower forms instead of the old
//          value-noise potatoes); G/B/A = inverted-Worley fBm at 2x/4x/8x the
//          perlin base frequency (low-frequency shape modulation).
//   DETAIL 32^3 RGBA8 — R/G/B = inverted-Worley fBm at rising frequency
//          (edge erosion); A unused (Metal has no 3-channel 8-bit format).
// Every octave wraps its lattice/cell hash at that octave's period, so both
// textures tile seamlessly under repeat addressing. The bake is deterministic
// (fixed seed) and runs once: raw bytes are cached under cache/clouds/ keyed
// by a content hash including the "pw-clouds-v1" version tag — the same
// pattern as the erosion cache in src/engine/procgen/terrain_field.cpp.
// Header-only so metal_renderer.mm (compiled into both viewer and editor_app)
// picks it up without CMake changes.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <thread>
#include <vector>

namespace cloudnoise {

// Deterministic lattice-point hash. Coordinates are pre-wrapped to the octave
// period by the callers, which is what makes every octave tile.
inline uint32_t hash3(int x, int y, int z, uint32_t seed) {
    uint32_t h = seed;
    h ^= static_cast<uint32_t>(x) * 0x8da6b343u;
    h ^= static_cast<uint32_t>(y) * 0xd8163841u;
    h ^= static_cast<uint32_t>(z) * 0xcb1ab31fu;
    h ^= h >> 13;
    h *= 0x9e3779b1u;
    h ^= h >> 16;
    return h;
}

inline int wrapi(int v, int period) {
    int m = v % period;
    return m < 0 ? m + period : m;
}

inline float clamp01(float v) { return std::min(1.0f, std::max(0.0f, v)); }

inline float remap(float x, float a, float b, float c, float d) {
    return c + (x - a) / std::max(b - a, 1e-5f) * (d - c);
}

// --- Worley (cellular) --------------------------------------------------
// One feature point per cell; min distance over the 27 neighbours, cell
// indices wrapped to `period` for the hash so the field tiles. p is in cell
// space ([0, period)); the return is the distance in cell units.
inline float worleyDist(float px, float py, float pz, int period, uint32_t seed) {
    const int cx = static_cast<int>(std::floor(px));
    const int cy = static_cast<int>(std::floor(py));
    const int cz = static_cast<int>(std::floor(pz));
    float minD2 = 1e9f;
    for (int dz = -1; dz <= 1; ++dz)
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                const int nx = cx + dx, ny = cy + dy, nz = cz + dz;
                const uint32_t h = hash3(wrapi(nx, period), wrapi(ny, period),
                                         wrapi(nz, period), seed);
                const float fx = nx + (h & 1023u) * (1.0f / 1024.0f);
                const float fy = ny + ((h >> 10) & 1023u) * (1.0f / 1024.0f);
                const float fz = nz + ((h >> 20) & 1023u) * (1.0f / 1024.0f);
                const float ex = fx - px, ey = fy - py, ez = fz - pz;
                minD2 = std::min(minD2, ex * ex + ey * ey + ez * ez);
            }
    return std::sqrt(minD2);
}

// Inverted-Worley fBm: bright cell cores, 3 octaves at f/2f/4f with the
// canonical 0.625/0.25/0.125 weights. Octaves past `maxFreq` (texel-sized
// cells — pure per-texel jitter after filtering) are dropped and the weights
// renormalised. uvw in [0,1).
inline float worleyFbm(float u, float v, float w, int freq, int maxFreq, uint32_t seed) {
    static const float WEIGHTS[3] = {0.625f, 0.25f, 0.125f};
    float sum = 0.0f, norm = 0.0f;
    int f = freq;
    for (int oct = 0; oct < 3 && f <= maxFreq; ++oct, f *= 2) {
        const float d = worleyDist(u * f, v * f, w * f, f, seed + 0x17e5u * oct);
        sum += (1.0f - std::min(1.0f, d)) * WEIGHTS[oct];
        norm += WEIGHTS[oct];
    }
    return norm > 0.0f ? sum / norm : 0.0f;
}

// --- Perlin -------------------------------------------------------------
inline float pfade(float t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }

inline float pgrad(uint32_t h, float x, float y, float z) {
    switch (h & 15u) {                       // Perlin's 12 edge gradients (+4 repeats)
        case 0: return x + y;   case 1: return -x + y;
        case 2: return x - y;   case 3: return -x - y;
        case 4: return x + z;   case 5: return -x + z;
        case 6: return x - z;   case 7: return -x - z;
        case 8: return y + z;   case 9: return -y + z;
        case 10: return y - z;  case 11: return -y - z;
        case 12: return x + y;  case 13: return -x + y;
        case 14: return y - z;  default: return -y - z;
    }
}

// Classic improved Perlin, lattice hashed mod `period` so it tiles. p in cell
// space; returns roughly [-1, 1].
inline float perlin(float px, float py, float pz, int period, uint32_t seed) {
    const int X = static_cast<int>(std::floor(px));
    const int Y = static_cast<int>(std::floor(py));
    const int Z = static_cast<int>(std::floor(pz));
    const float x = px - X, y = py - Y, z = pz - Z;
    const float fu = pfade(x), fv = pfade(y), fw = pfade(z);
    float c[2][2][2];
    for (int k = 0; k < 2; ++k)
        for (int j = 0; j < 2; ++j)
            for (int i = 0; i < 2; ++i) {
                const uint32_t h = hash3(wrapi(X + i, period), wrapi(Y + j, period),
                                         wrapi(Z + k, period), seed);
                c[k][j][i] = pgrad(h, x - i, y - j, z - k);
            }
    const float x00 = c[0][0][0] + fu * (c[0][0][1] - c[0][0][0]);
    const float x10 = c[0][1][0] + fu * (c[0][1][1] - c[0][1][0]);
    const float x01 = c[1][0][0] + fu * (c[1][0][1] - c[1][0][0]);
    const float x11 = c[1][1][0] + fu * (c[1][1][1] - c[1][1][0]);
    const float y0 = x00 + fv * (x10 - x00);
    const float y1 = x01 + fv * (x11 - x01);
    return y0 + fw * (y1 - y0);
}

// Perlin fBm remapped to [0,1]. uvw in [0,1); octave frequency doubles and the
// lattice wrap follows it, so the sum still tiles.
inline float perlinFbm(float u, float v, float w, int freq, int octaves, uint32_t seed) {
    float sum = 0.0f, amp = 0.5f, norm = 0.0f;
    int f = freq;
    for (int oct = 0; oct < octaves; ++oct, f *= 2, amp *= 0.5f) {
        sum += perlin(u * f, v * f, w * f, f, seed + 0x9e37u * oct) * amp;
        norm += amp;
    }
    return clamp01(sum / norm * 0.5f + 0.5f);
}

// --- Bake ---------------------------------------------------------------
template <typename Fn>
inline void parallelSlices(int n, Fn&& fn) {
    unsigned tc = std::max(1u, std::thread::hardware_concurrency());
    tc = std::min(tc, static_cast<unsigned>(n));
    const int per = (n + static_cast<int>(tc) - 1) / static_cast<int>(tc);
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < tc; ++t) {
        const int z0 = static_cast<int>(t) * per;
        const int z1 = std::min(n, z0 + per);
        if (z0 >= z1) break;
        pool.emplace_back([&fn, z0, z1] { fn(z0, z1); });
    }
    for (auto& th : pool) th.join();
}

inline uint8_t toByte(float v) {
    return static_cast<uint8_t>(std::lround(clamp01(v) * 255.0f));
}

// BASE 128^3 RGBA8. Perlin base frequency 4 → with the shader's 0.25 uv scale
// on q = p * noiseScale, one perlin cell lands at ~1/noiseScale metres, the
// same feature scale the old value-noise fbm produced (levels keep their look
// scale-wise). Worley channels at 8/16/32 modulate it downward.
//
// Distribution note: the raw chain — PW = remap(perlin, worley-1, 1, 0, 1)
// baked, then the shader's canonical remap by the GBA fbm — compresses the
// histogram toward ~0.78 with almost no spread, and the coverage threshold
// then slices a near-uniform field: a washed-out full-sky veil instead of
// cumulus (observed on the first capture). So the bake normalises the
// SHADER-SIDE value: it computes b = remap(PW, fbm-1, 1, 0, 1) per voxel,
// percentile-stretches b to [0,1], and stores the exact pre-image of the
// stretched value under the shader's remap. The shader stays canonical; the
// coverage knob cuts a healthy billowy field again.
inline void bakeBase(int n, uint32_t seed, std::vector<uint8_t>& out) {
    const size_t voxels = static_cast<size_t>(n) * n * n;
    out.resize(voxels * 4);
    const int maxFreq = n / 2;
    std::vector<float> shaded(voxels);   // b: what the shader's remap yields
    std::vector<float> lowFbm(voxels);   // GBA fbm with the shader's weights
    parallelSlices(n, [&](int z0, int z1) {
        for (int z = z0; z < z1; ++z)
            for (int y = 0; y < n; ++y)
                for (int x = 0; x < n; ++x) {
                    const float u = (x + 0.5f) / n;
                    const float v = (y + 0.5f) / n;
                    const float w = (z + 0.5f) / n;
                    const float pf = perlinFbm(u, v, w, 4, 5, seed);
                    const float wl = worleyFbm(u, v, w, 4, maxFreq, seed ^ 0x1000193u);
                    const float pw = clamp01(remap(pf, wl - 1.0f, 1.0f, 0.0f, 1.0f));
                    const float g = worleyFbm(u, v, w, 8, maxFreq, seed ^ 0x22e2u);
                    const float b = worleyFbm(u, v, w, 16, maxFreq, seed ^ 0x33d3u);
                    const float a = worleyFbm(u, v, w, 32, maxFreq, seed ^ 0x44c4u);
                    const size_t i = (static_cast<size_t>(z) * n + y) * n + x;
                    const float fbm = g * 0.625f + b * 0.25f + a * 0.125f;
                    lowFbm[i] = fbm;
                    shaded[i] = clamp01(remap(pw, fbm - 1.0f, 1.0f, 0.0f, 1.0f));
                    out[i * 4 + 1] = toByte(g);
                    out[i * 4 + 2] = toByte(b);
                    out[i * 4 + 3] = toByte(a);
                }
    });
    // Percentile stretch of b (5th..95th → 0..1), then invert the shader's
    // remap so sampling R and applying remap(R, fbm-1, 1, 0, 1) lands on the
    // stretched value: R = (fbm-1) + bNorm * (2-fbm), clamped for storage.
    std::vector<uint32_t> hist(1024, 0);
    for (float b : shaded)
        hist[std::min<size_t>(1023, static_cast<size_t>(b * 1023.0f))]++;
    const auto percentile = [&](float p) {
        size_t target = static_cast<size_t>(p * voxels), acc = 0;
        for (size_t bin = 0; bin < hist.size(); ++bin) {
            acc += hist[bin];
            if (acc >= target) return bin / 1023.0f;
        }
        return 1.0f;
    };
    const float lo = percentile(0.05f);
    const float hi = std::max(percentile(0.95f), lo + 1e-3f);
    parallelSlices(n, [&](int z0, int z1) {
        const size_t i0 = static_cast<size_t>(z0) * n * n;
        const size_t i1 = static_cast<size_t>(z1) * n * n;
        for (size_t i = i0; i < i1; ++i) {
            const float bNorm = clamp01((shaded[i] - lo) / (hi - lo));
            out[i * 4 + 0] =
                toByte((lowFbm[i] - 1.0f) + bNorm * (2.0f - lowFbm[i]));
        }
    });
}

// DETAIL 32^3 RGBA8: three inverted-Worley fBm channels at rising frequency;
// the shader folds them 0.625/0.25/0.125 and erodes cloud edges with the sum.
inline void bakeDetail(int n, uint32_t seed, std::vector<uint8_t>& out) {
    out.resize(static_cast<size_t>(n) * n * n * 4);
    const int maxFreq = n / 2;
    parallelSlices(n, [&](int z0, int z1) {
        for (int z = z0; z < z1; ++z)
            for (int y = 0; y < n; ++y)
                for (int x = 0; x < n; ++x) {
                    const float u = (x + 0.5f) / n;
                    const float v = (y + 0.5f) / n;
                    const float w = (z + 0.5f) / n;
                    const size_t i =
                        ((static_cast<size_t>(z) * n + y) * n + x) * 4;
                    out[i + 0] = toByte(worleyFbm(u, v, w, 2, maxFreq, seed ^ 0x55b5u));
                    out[i + 1] = toByte(worleyFbm(u, v, w, 4, maxFreq, seed ^ 0x66a6u));
                    out[i + 2] = toByte(worleyFbm(u, v, w, 8, maxFreq, seed ^ 0x7797u));
                    out[i + 3] = 255;   // unused (no 3-channel 8-bit MTLPixelFormat)
                }
    });
}

struct NoiseSet {
    int baseSize = 128;
    int detailSize = 32;
    std::vector<uint8_t> base;     // baseSize^3 RGBA8
    std::vector<uint8_t> detail;   // detailSize^3 RGBA8
};

// Load the cached bake, or bake (a few hundred ms, threaded) and cache it.
// The key hashes a descriptor of everything that shapes the bytes — bumping
// the "pw-clouds-v1" tag (or any frequency) invalidates cleanly, and stale
// entries are simply never referenced again.
inline NoiseSet generateOrLoad() {
    NoiseSet s;
    const uint32_t seed = 0x510adb15u;   // fixed: the bake is deterministic
    char desc[160];
    std::snprintf(desc, sizeof desc, "pw-clouds-v2|%d|%d|%08x|p4o5|w4.8.16.32|d2.4.8",
                  s.baseSize, s.detailSize, seed);
    unsigned long long key = 1469598103934665603ull;   // FNV-1a
    for (const char* c = desc; *c; ++c) {
        key ^= static_cast<unsigned char>(*c);
        key *= 1099511628211ull;
    }
    char path[256];
    std::snprintf(path, sizeof path, "cache/clouds/%016llx.bin",
                  static_cast<unsigned long long>(key));
    const size_t baseBytes = static_cast<size_t>(s.baseSize) * s.baseSize * s.baseSize * 4;
    const size_t detailBytes =
        static_cast<size_t>(s.detailSize) * s.detailSize * s.detailSize * 4;
    if (std::FILE* fp = std::fopen(path, "rb")) {
        s.base.resize(baseBytes);
        s.detail.resize(detailBytes);
        const bool ok =
            std::fread(s.base.data(), 1, baseBytes, fp) == baseBytes &&
            std::fread(s.detail.data(), 1, detailBytes, fp) == detailBytes;
        std::fclose(fp);
        if (ok) return s;
    }
    bakeBase(s.baseSize, seed, s.base);
    bakeDetail(s.detailSize, seed ^ 0xd317a115u, s.detail);
    std::error_code ec;
    std::filesystem::create_directories("cache/clouds", ec);
    if (std::FILE* fp = std::fopen(path, "wb")) {
        std::fwrite(s.base.data(), 1, baseBytes, fp);
        std::fwrite(s.detail.data(), 1, detailBytes, fp);
        std::fclose(fp);
    }
    return s;
}

}  // namespace cloudnoise
