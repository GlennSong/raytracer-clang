#include "texture_field.h"

#include "noise.h"
#include <algorithm>
#include <cmath>
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

}  // namespace

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

}  // namespace engine
