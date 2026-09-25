#include "texture_field.h"

#include "noise.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>

namespace engine {
namespace {

double clamp01(double x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

uint32_t hash2i(int x, int y, uint32_t seed) {
    uint32_t h = seed * 2654435761u + static_cast<uint32_t>(x) * 40503u +
                 static_cast<uint32_t>(y) * 668265263u + 0x9e3779b9u;
    h ^= h >> 15; h *= 0x2c1b3c6du; h ^= h >> 12; h *= 0x297a2d39u; h ^= h >> 15;
    return h;
}

// Wrapping value lattice: exactly periodic in `period` cells.
double tileLattice(int x, int y, int period, uint32_t seed) {
    x = ((x % period) + period) % period;
    y = ((y % period) + period) % period;
    return (hash2i(x, y, seed) & 0xFFFFFFu) / 16777215.0;
}
double tileValueNoise(double u, double v, int period, uint32_t seed) {
    const double x = u * period, y = v * period;
    const int i = static_cast<int>(std::floor(x)), j = static_cast<int>(std::floor(y));
    const double fx = x - i, fy = y - j;
    const double sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy);
    const double a = tileLattice(i, j, period, seed), b = tileLattice(i + 1, j, period, seed);
    const double c = tileLattice(i, j + 1, period, seed), d = tileLattice(i + 1, j + 1, period, seed);
    const double top = a + (b - a) * sx, bot = c + (d - c) * sx;
    return top + (bot - top) * sy;
}
// Worley over a wrapping jittered grid: nearest and second-nearest distances, and the nearest id.
struct CellHit { double d1, d2; uint32_t id; };
CellHit tileCells(double u, double v, int period, uint32_t seed) {
    const double x = u * period, y = v * period;
    const int i = static_cast<int>(std::floor(x)), j = static_cast<int>(std::floor(y));
    CellHit h{9.0, 9.0, 0};
    for (int dj = -1; dj <= 1; ++dj)
        for (int di = -1; di <= 1; ++di) {
            const int ci = i + di, cj = j + dj;
            const int wi = ((ci % period) + period) % period, wj = ((cj % period) + period) % period;
            const uint32_t r = hash2i(wi, wj, seed);
            const double px = ci + 0.1 + 0.8 * ((r & 0xFFFFu) / 65535.0), py = cj + 0.1 + 0.8 * ((r >> 16) / 65535.0);
            const double d = std::sqrt((px - x) * (px - x) + (py - y) * (py - y));
            if (d < h.d1) { h.d2 = h.d1; h.d1 = d; h.id = r; }
            else if (d < h.d2) h.d2 = d;
        }
    return h;
}
double wrap01(double t) { return t - std::floor(t); }

}  // namespace

Field2 fieldTileNoise(uint32_t seed, int period) {
    const int p = std::max(1, period);
    return [seed, p](double u, double v) { return tileValueNoise(wrap01(u), wrap01(v), p, seed); };
}
Field2 fieldTileFbm(uint32_t seed, int period, int octaves) {
    const int p = std::max(1, period), oc = std::max(1, octaves);
    return [seed, p, oc](double u, double v) {
        u = wrap01(u); v = wrap01(v);
        double sum = 0, amp = 0.5, norm = 0;
        for (int o = 0; o < oc; ++o) {
            sum += amp * tileValueNoise(u, v, p << o, seed + 101u * static_cast<uint32_t>(o));
            norm += amp; amp *= 0.5;
        }
        return sum / norm;
    };
}
Field2 fieldCells(uint32_t seed, int period) {
    const int p = std::max(1, period);
    return [seed, p](double u, double v) { return clamp01(tileCells(wrap01(u), wrap01(v), p, seed).d1); };
}
Field2 fieldCellEdges(uint32_t seed, int period) {
    const int p = std::max(1, period);
    return [seed, p](double u, double v) {
        const CellHit h = tileCells(wrap01(u), wrap01(v), p, seed);
        return clamp01(h.d2 - h.d1);
    };
}
Field2 fieldCellId(uint32_t seed, int period) {
    const int p = std::max(1, period);
    return [seed, p](double u, double v) {
        const uint32_t id = tileCells(wrap01(u), wrap01(v), p, seed).id;
        return ((id * 2654435761u) >> 8) / 16777215.0;
    };
}
Field2 fieldBands(double count) {
    return [count](double, double v) { return 0.5 + 0.5 * std::sin(v * count * 6.283185307179586); };
}
Field2 fieldSmoothstep(Field2 a, double lo, double hi) {
    return [a, lo, hi](double u, double v) {
        const double t = clamp01((a(u, v) - lo) / (hi - lo));
        return t * t * (3.0 - 2.0 * t);
    };
}
Field2 fieldInvert(Field2 a) { return [a](double u, double v) { return 1.0 - a(u, v); }; }
Field2 fieldMin(Field2 a, Field2 b) { return [a, b](double u, double v) { return std::min(a(u, v), b(u, v)); }; }
Field2 fieldMax(Field2 a, Field2 b) { return [a, b](double u, double v) { return std::max(a(u, v), b(u, v)); }; }
Field2 fieldMixBy(Field2 a, Field2 b, Field2 t) {
    return [a, b, t](double u, double v) {
        const double x = a(u, v);
        return x + (b(u, v) - x) * t(u, v);
    };
}
Field2 fieldPow(Field2 a, double e) {
    return [a, e](double u, double v) { return std::pow(std::max(a(u, v), 0.0), e); };
}
Field2 fieldWarp(Field2 a, Field2 du, Field2 dv, double amount) {
    return [a, du, dv, amount](double u, double v) {
        return a(wrap01(u + (du(u, v) - 0.5) * 2.0 * amount), wrap01(v + (dv(u, v) - 0.5) * 2.0 * amount));
    };
}

ColorField2 colorConstant(const Vec3& c) { return [c](double, double) { return c; }; }
ColorField2 colorMixBy(ColorField2 a, ColorField2 b, Field2 t) {
    return [a, b, t](double u, double v) {
        const Vec3 x = a(u, v);
        return x + (b(u, v) - x) * t(u, v);
    };
}
ColorField2 colorMul(ColorField2 c, Field2 k) { return [c, k](double u, double v) { return c(u, v) * k(u, v); }; }
ColorField2 colorTint(ColorField2 c, const Vec3& tint) {
    return [c, tint](double u, double v) {
        const Vec3 x = c(u, v);
        return Vec3(x.x * tint.x, x.y * tint.y, x.z * tint.z);
    };
}

TextureData bakeFieldRGBA(const ColorField2& color, const Field2& alpha, int size, bool gamma) {
    size = std::max(1, size);
    TextureData td;
    td.width = size; td.height = size; td.channels = 4;
    td.pixels.resize(static_cast<std::size_t>(size) * size * 4);
    auto enc = [gamma](double c) {
        c = clamp01(c);
        if (gamma) c = std::pow(c, 1.0 / 2.2);
        return static_cast<uint8_t>(c * 255.0 + 0.5);
    };
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x) {
            const double u = (x + 0.5) / size, v = (y + 0.5) / size;
            const Vec3 c = color(u, v);
            uint8_t* px = &td.pixels[(static_cast<std::size_t>(y) * size + x) * 4];
            px[0] = enc(c.x); px[1] = enc(c.y); px[2] = enc(c.z);
            px[3] = static_cast<uint8_t>(clamp01(alpha(u, v)) * 255.0 + 0.5);
        }
    return td;
}

TextureData bakeCached(const std::string& recipeKey, const std::function<TextureData()>& bake) {
    uint64_t h = 1469598103934665603ull;   // FNV-1a over the key
    for (unsigned char ch : recipeKey) { h ^= ch; h *= 1099511628211ull; }
    char path[96];
    std::snprintf(path, sizeof path, "cache/fields/%016llx.tex", static_cast<unsigned long long>(h));
    const char* nc = std::getenv("RT_NOCACHE");
    if (!(nc && nc[0] == '1')) {
        std::ifstream in(path, std::ios::binary);
        int32_t hdr[3] = {0, 0, 0};
        if (in && in.read(reinterpret_cast<char*>(hdr), sizeof hdr) && hdr[0] > 0 && hdr[1] > 0 && hdr[2] > 0 && hdr[2] <= 4) {
            TextureData td;
            td.width = hdr[0]; td.height = hdr[1]; td.channels = hdr[2];
            td.pixels.resize(static_cast<std::size_t>(hdr[0]) * hdr[1] * hdr[2]);
            if (in.read(reinterpret_cast<char*>(td.pixels.data()), static_cast<std::streamsize>(td.pixels.size()))) return td;
        }
    }
    TextureData td = bake();
    std::error_code ec;
    std::filesystem::create_directories("cache/fields", ec);
    std::ofstream out(path, std::ios::binary);
    if (out) {
        const int32_t hdr[3] = {td.width, td.height, td.channels};
        out.write(reinterpret_cast<const char*>(hdr), sizeof hdr);
        out.write(reinterpret_cast<const char*>(td.pixels.data()), static_cast<std::streamsize>(td.pixels.size()));
    }
    return td;
}

Field2 fieldConstant(double value) {
    return [value](double, double) { return value; };
}

Field2 fieldNoise(uint32_t seed, double scale) {
    auto n = std::make_shared<Noise>(seed);
    return [n, scale](double u, double v) {
        return clamp01(n->noise2(u * scale, v * scale) * 0.5 + 0.5);
    };
}

Field2 fieldFbm(uint32_t seed, double scale, int octaves) {
    auto n = std::make_shared<Noise>(seed);
    int oc = std::max(1, octaves);
    return [n, scale, oc](double u, double v) {
        return clamp01(n->fbm2(u * scale, v * scale, oc) * 0.5 + 0.5);
    };
}

Field2 fieldChecker(double cols, double rows) {
    return [cols, rows](double u, double v) {
        int cx = static_cast<int>(std::floor(u * cols));
        int cy = static_cast<int>(std::floor(v * rows));
        return ((cx + cy) & 1) ? 1.0 : 0.0;
    };
}

Field2 fieldBrick(double cols, double rows, double mortar, double variation,
                  uint32_t seed) {
    double half = mortar * 0.5;
    return [=](double u, double v) -> double {
        double rf = v * rows;
        int row = static_cast<int>(std::floor(rf));
        double offset = (row & 1) ? 0.5 : 0.0;       // running bond
        double uf = u * cols + offset;
        int col = static_cast<int>(std::floor(uf));
        double fu = uf - std::floor(uf);
        double fv = rf - row;
        double mu = std::min(fu, 1.0 - fu);          // distance to column edge
        double mv = std::min(fv, 1.0 - fv);          // distance to row edge
        if (mu < half || mv < half) return 0.0;      // mortar gap
        double t = (hash2i(col, row, seed) & 0xffffu) / 65535.0;
        return 1.0 - variation * t;                  // per-brick darkening
    };
}

Field2 fieldGradientY() {
    return [](double, double v) { return v; };
}

Field2 fieldAdd(Field2 a, Field2 b) {
    return [a, b](double u, double v) { return a(u, v) + b(u, v); };
}
Field2 fieldMul(Field2 a, Field2 b) {
    return [a, b](double u, double v) { return a(u, v) * b(u, v); };
}
Field2 fieldMix(Field2 a, Field2 b, double t) {
    return [a, b, t](double u, double v) {
        double x = a(u, v), y = b(u, v);
        return x + (y - x) * t;
    };
}
Field2 fieldScaleBias(Field2 a, double scale, double bias) {
    return [a, scale, bias](double u, double v) { return a(u, v) * scale + bias; };
}
Field2 fieldClamp(Field2 a, double lo, double hi) {
    return [a, lo, hi](double u, double v) {
        double x = a(u, v);
        return x < lo ? lo : (x > hi ? hi : x);
    };
}

TextureData bakeFieldGray(const Field2& f, int size) {
    size = std::max(1, size);
    TextureData td;
    td.width = size; td.height = size; td.channels = 3;
    td.pixels.resize(static_cast<std::size_t>(size) * size * 3);
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            double u = (x + 0.5) / size, v = (y + 0.5) / size;
            uint8_t g = static_cast<uint8_t>(clamp01(f(u, v)) * 255.0 + 0.5);
            std::size_t i = (static_cast<std::size_t>(y) * size + x) * 3;
            td.pixels[i] = td.pixels[i + 1] = td.pixels[i + 2] = g;
        }
    }
    return td;
}

TextureData bakeFieldColor(const Field2& mask, const Vec3& a, const Vec3& b,
                           int size) {
    size = std::max(1, size);
    TextureData td;
    td.width = size; td.height = size; td.channels = 3;
    td.pixels.resize(static_cast<std::size_t>(size) * size * 3);
    auto byte = [](double c) {
        return static_cast<uint8_t>((c < 0 ? 0 : (c > 1 ? 1 : c)) * 255.0 + 0.5);
    };
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            double u = (x + 0.5) / size, v = (y + 0.5) / size;
            double t = clamp01(mask(u, v));
            Vec3 c = a + (b - a) * t;
            std::size_t i = (static_cast<std::size_t>(y) * size + x) * 3;
            td.pixels[i] = byte(c.x);
            td.pixels[i + 1] = byte(c.y);
            td.pixels[i + 2] = byte(c.z);
        }
    }
    return td;
}

TextureData bakeFieldNormal(const Field2& height, double strength, int size) {
    size = std::max(1, size);
    TextureData td;
    td.width = size; td.height = size; td.channels = 3;
    td.pixels.resize(static_cast<std::size_t>(size) * size * 3);
    double d = 1.0 / size;
    auto byte = [](double c) {
        return static_cast<uint8_t>((c < 0 ? 0 : (c > 1 ? 1 : c)) * 255.0 + 0.5);
    };
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            double u = (x + 0.5) / size, v = (y + 0.5) / size;
            // Central differences on the (tiling) height field.
            double hl = height(u - d, v), hr = height(u + d, v);
            double hd = height(u, v - d), hu = height(u, v + d);
            Vec3 n(-(hr - hl) * strength, -(hu - hd) * strength, 1.0);
            n = normalize(n);
            std::size_t i = (static_cast<std::size_t>(y) * size + x) * 3;
            td.pixels[i] = byte(n.x * 0.5 + 0.5);
            td.pixels[i + 1] = byte(n.y * 0.5 + 0.5);
            td.pixels[i + 2] = byte(n.z * 0.5 + 0.5);
        }
    }
    return td;
}

}  // namespace engine
