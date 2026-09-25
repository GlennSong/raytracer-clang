#include "ground_layers.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace engine {

namespace {

constexpr int kGeneratorVersion = 2;   // 2: warped, broken rock cracks

uint32_t hash3(int x, int y, uint32_t s) {
    uint32_t h = static_cast<uint32_t>(x) * 0x8DA6B343u ^ static_cast<uint32_t>(y) * 0xD8163841u ^ s * 0xCB1AB31Fu;
    h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12; h *= 0x297A2D39u; h ^= h >> 15;
    return h;
}
double lattice(int x, int y, int period, uint32_t s) {
    x = ((x % period) + period) % period;
    y = ((y % period) + period) % period;
    return (hash3(x, y, s) & 0xFFFFFF) / 16777215.0 * 2.0 - 1.0;
}
// Value noise on a lattice of `period` cells across the tile: exactly periodic in u, v in [0, 1).
double pnoise(double u, double v, int period, uint32_t s) {
    const double x = u * period, y = v * period;
    const int i = static_cast<int>(std::floor(x)), j = static_cast<int>(std::floor(y));
    const double fx = x - i, fy = y - j;
    const double sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy);
    const double a = lattice(i, j, period, s), b = lattice(i + 1, j, period, s);
    const double c = lattice(i, j + 1, period, s), d = lattice(i + 1, j + 1, period, s);
    return (a + (b - a) * sx) + ((c + (d - c) * sx) - (a + (b - a) * sx)) * sy;
}
double pfbm(double u, double v, int period, int octaves, uint32_t s) {
    double sum = 0, amp = 0.5, norm = 0;
    for (int o = 0; o < octaves; ++o) {
        sum += amp * pnoise(u, v, period << o, s + 101u * static_cast<uint32_t>(o));
        norm += amp; amp *= 0.5;
    }
    return sum / norm;   // ~-1..1
}
// Distance to the nearest of a jittered periodic point set (cells per side = period): 0 at a
// point, ~1 between. Pebbles, rock plates.
double pcell(double u, double v, int period, uint32_t s, double* edge = nullptr) {
    const double x = u * period, y = v * period;
    const int i = static_cast<int>(std::floor(x)), j = static_cast<int>(std::floor(y));
    double d1 = 9, d2 = 9;
    for (int dj = -1; dj <= 1; ++dj)
        for (int di = -1; di <= 1; ++di) {
            const int ci = i + di, cj = j + dj;
            const int wi = ((ci % period) + period) % period, wj = ((cj % period) + period) % period;
            const uint32_t h = hash3(wi, wj, s);
            const double px = ci + 0.15 + 0.7 * ((h & 0xFFFF) / 65535.0), py = cj + 0.15 + 0.7 * ((h >> 16) / 65535.0);
            const double d = std::sqrt((px - x) * (px - x) + (py - y) * (py - y));
            if (d < d1) { d2 = d1; d1 = d; } else if (d < d2) d2 = d;
        }
    if (edge) *edge = d2 - d1;   // small on the boundary between two cells (a crack)
    return d1;
}
double sm(double a, double b, double x) { const double t = std::clamp((x - a) / (b - a), 0.0, 1.0); return t * t * (3 - 2 * t); }
uint8_t enc(double linear) { return static_cast<uint8_t>(std::clamp(std::pow(std::max(linear, 0.0), 1.0 / 2.2), 0.0, 1.0) * 255.0 + 0.5); }

}  // namespace

TextureData groundLayerTexture(GroundLayer layer, const Vec3& base, int size, uint32_t seed) {
    TextureData t;
    t.width = t.height = size; t.channels = 4;
    t.pixels.resize(static_cast<std::size_t>(size) * size * 4);
    const uint32_t s = seed * 7919u + static_cast<uint32_t>(layer) * 104729u;
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x) {
            const double u = (x + 0.5) / size, v = (y + 0.5) / size;
            Vec3 c = base;
            double h = 0.5;
            switch (layer) {
                case GroundLayer::Grass: {
                    // soft painted blotches, a warmer/cooler drift, and short directional strokes
                    const double blot = pfbm(u, v, 3, 3, s);
                    const double warm = pfbm(u, v, 2, 2, s + 7);
                    const double stroke = pnoise(u * 1.0, v * 1.0, 24, s + 13) * 0.5 + pnoise(u, v, 48, s + 17) * 0.5;
                    const double l = 1.0 + 0.16 * blot + 0.07 * stroke;
                    c = Vec3(base.x * l * (1.0 + 0.18 * warm), base.y * l, base.z * l * (1.0 - 0.15 * warm));
                    h = 0.45 + 0.25 * blot + 0.2 * stroke;
                    break;
                }
                case GroundLayer::Dirt: {
                    const double blot = pfbm(u, v, 4, 3, s);
                    double edge = 0;
                    const double peb = pcell(u, v, 22, s + 3, &edge);
                    const double pebble = 1.0 - sm(0.18, 0.34, peb);          // a pebble's body
                    const double l = 1.0 + 0.14 * blot;
                    c = base * l;
                    c = c + (Vec3(0.20, 0.18, 0.15) - c) * (0.55 * pebble);   // pale stones in the earth
                    c = c * (1.0 - 0.25 * (1.0 - sm(0.0, 0.08, edge)) * (1.0 - pebble) * 0.4);
                    h = 0.3 + 0.2 * blot + 0.5 * pebble;
                    break;
                }
                case GroundLayer::Sand: {
                    const double warp = pfbm(u, v, 3, 2, s) * 0.08;
                    const double ripple = std::sin((v + warp) * 2.0 * 3.14159265 * 11.0);   // 11 ripples a tile: periodic
                    const double blot = pfbm(u, v, 4, 3, s + 5);
                    const double l = 1.0 + 0.05 * ripple + 0.08 * blot;
                    c = base * l;
                    h = 0.4 + 0.25 * ripple * 0.5 + 0.25 * blot;
                    break;
                }
                case GroundLayer::Rock: {
                    // a few big slabs whose cracks WANDER (the cell lookup is domain-warped by
                    // periodic noise) -- regular cells read as crazy paving, not stone
                    double edge = 0;
                    const double wu = u + 0.06 * pfbm(u, v, 3, 3, s + 29), wv = v + 0.06 * pfbm(u, v, 3, 3, s + 31);
                    pcell(std::fmod(wu + 1.0, 1.0), std::fmod(wv + 1.0, 1.0), 3, s + 11, &edge);
                    const double crack = (1.0 - sm(0.0, 0.035, edge)) * sm(-0.2, 0.3, pfbm(u, v, 4, 2, s + 37));   // broken, not continuous
                    const double grain = pfbm(u, v, 6, 4, s + 19);
                    const double strata = std::sin((v + 0.06 * pfbm(u, v, 2, 2, s + 23)) * 2.0 * 3.14159265 * 6.0);
                    const double tone = pfbm(u, v, 2, 3, s + 41);
                    const double l = (1.0 + 0.16 * grain + 0.05 * strata + 0.14 * tone) * (1.0 - 0.45 * crack);
                    c = base * l;
                    h = 0.75 + 0.2 * grain - 0.6 * crack;
                    break;
                }
            }
            uint8_t* px = &t.pixels[(static_cast<std::size_t>(y) * size + x) * 4];
            px[0] = enc(c.x); px[1] = enc(c.y); px[2] = enc(c.z);
            px[3] = static_cast<uint8_t>(std::clamp(h, 0.0, 1.0) * 255.0 + 0.5);
        }
    return t;
}

TextureData groundLayerTextureCached(GroundLayer layer, const Vec3& base, int size, uint32_t seed) {
    const char* nc = std::getenv("RT_NOCACHE");
    char key[256];
    std::snprintf(key, sizeof key, "cache/terrain_layers/v%d_L%d_%d_%u_%.5f_%.5f_%.5f.rgba", kGeneratorVersion,
                  static_cast<int>(layer), size, seed, static_cast<double>(base.x), static_cast<double>(base.y),
                  static_cast<double>(base.z));
    const std::size_t bytes = static_cast<std::size_t>(size) * size * 4;
    if (!(nc && nc[0] == '1')) {
        std::ifstream in(key, std::ios::binary);
        if (in) {
            TextureData t;
            t.width = t.height = size; t.channels = 4;
            t.pixels.resize(bytes);
            in.read(reinterpret_cast<char*>(t.pixels.data()), static_cast<std::streamsize>(bytes));
            if (static_cast<std::size_t>(in.gcount()) == bytes) return t;
        }
    }
    TextureData t = groundLayerTexture(layer, base, size, seed);
    std::error_code ec;
    std::filesystem::create_directories("cache/terrain_layers", ec);
    std::ofstream out(key, std::ios::binary);
    if (out) out.write(reinterpret_cast<const char*>(t.pixels.data()), static_cast<std::streamsize>(bytes));
    return t;
}

}  // namespace engine
