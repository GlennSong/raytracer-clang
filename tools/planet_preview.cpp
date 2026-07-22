// Headless "from space" preview: orthographic disc render of the procedural
// planets, so the generator can be eyeballed without a GPU. Not part of the build.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <tinygltf/stb_image_write.h>

#include "engine/procgen/planet.h"
#include "rt_math.h"

#include <cmath>
#include <vector>
#include <cstdint>
#include <cstdio>

using namespace engine;

static uint32_t hashU(uint32_t x){ x^=x>>16; x*=0x7feb352du; x^=x>>15; x*=0x846ca68bu; x^=x>>16; return x; }

// Simple starfield for the background pixels.
static Vec3 starField(int px, int py) {
    uint32_t h = hashU(px * 73856093u ^ py * 19349663u);
    double r = (h >> 8) * (1.0 / 16777216.0);
    if (r > 0.997) { double b = 0.5 + 0.5 * ((h & 0xff) / 255.0); return Vec3(b, b, b); }
    return Vec3(0.015, 0.02, 0.03);
}

// Orthographic disc of a rocky planet: relief-shaded (finite-difference normal from
// the displaced height field), sun-lit with a terminator, ocean where submerged.
static void renderRocky(const PlanetParams& p, uint32_t seed, const char* file, int W) {
    std::vector<uint8_t> img(static_cast<size_t>(W) * W * 3);
    Vec3 sun = normalize(Vec3(0.6, 0.35, 0.75));
    const double amp = p.reliefFraction;  // unit-radius preview
    auto disp = [&](Vec3 d) { d = normalize(d); return d * (1.0 + amp * planetHeight(p, seed, d)); };

    for (int py = 0; py < W; py++) {
        for (int px = 0; px < W; px++) {
            double x = ((px + 0.5) / W * 2 - 1) * 1.12;
            double y = -((py + 0.5) / W * 2 - 1) * 1.12;
            Vec3 col;
            if (x * x + y * y <= 1.0) {
                double z = std::sqrt(1.0 - x * x - y * y);
                Vec3 dir = normalize(Vec3(x, y, z));
                Vec3 t1 = cross(Vec3(0, 1, 0), dir);
                t1 = t1.lengthSquared() > 1e-8 ? normalize(t1) : Vec3(1, 0, 0);
                Vec3 t2 = cross(dir, t1);
                double e = 0.005;
                Vec3 P = disp(dir), Pu = disp(dir + t1 * e), Pv = disp(dir + t2 * e);
                Vec3 N = cross(Pv - P, Pu - P);
                if (N.lengthSquared() < 1e-12) N = dir;
                N = normalize(N);
                if (dot(N, dir) < 0) N = N * -1.0;

                double h = planetHeight(p, seed, dir);
                Vec3 albedo = planetSurfaceColor(p, seed, dir);
                if (p.hasOcean && h < p.seaLevel) { albedo = p.oceanColor; N = dir; }

                double diff = std::max(0.0, dot(N, sun));
                double lit = 0.05 + 0.95 * diff;
                col = albedo * lit;
                // faint blue atmosphere rim near the limb
                double rim = std::pow(1.0 - z, 3.0);
                col = col + Vec3(0.20, 0.32, 0.55) * (rim * std::max(0.0, dot(dir, sun)) * 0.9);
            } else {
                col = starField(px, py);
            }
            size_t idx = (static_cast<size_t>(py) * W + px) * 3;
            img[idx + 0] = (uint8_t)(std::min(1.0, col.x) * 255 + 0.5);
            img[idx + 1] = (uint8_t)(std::min(1.0, col.y) * 255 + 0.5);
            img[idx + 2] = (uint8_t)(std::min(1.0, col.z) * 255 + 0.5);
        }
    }
    stbi_write_png(file, W, W, 3, img.data(), W * 3);
    std::printf("wrote %s\n", file);
}

// Orthographic disc of a gas giant: sample the baked equirect albedo by direction,
// smooth sphere, lambert + limb darkening.
static void renderGas(const GasGiantParams& p, uint32_t seed, const char* file, int W) {
    TextureData tex = generateGasGiantTexture(p, seed);
    std::vector<uint8_t> img(static_cast<size_t>(W) * W * 3);
    Vec3 sun = normalize(Vec3(0.7, 0.3, 0.65));
    auto sample = [&](const Vec3& dir) {
        double u = 0.5 + std::atan2(dir.z, dir.x) / (2 * PI);
        double v = 0.5 - std::asin(std::max(-1.0, std::min(1.0, dir.y))) / PI;
        int tx = std::min(tex.width - 1, std::max(0, (int)(u * tex.width)));
        int ty = std::min(tex.height - 1, std::max(0, (int)(v * tex.height)));
        size_t idx = (static_cast<size_t>(ty) * tex.width + tx) * 3;
        return Vec3(tex.pixels[idx] / 255.0, tex.pixels[idx + 1] / 255.0, tex.pixels[idx + 2] / 255.0);
    };
    for (int py = 0; py < W; py++) {
        for (int px = 0; px < W; px++) {
            double x = ((px + 0.5) / W * 2 - 1) * 1.12;
            double y = -((py + 0.5) / W * 2 - 1) * 1.12;
            Vec3 col;
            if (x * x + y * y <= 1.0) {
                double z = std::sqrt(1.0 - x * x - y * y);
                Vec3 dir = normalize(Vec3(x, y, z));
                Vec3 albedo = sample(dir);
                double diff = std::max(0.0, dot(dir, sun));
                double limb = 0.4 + 0.6 * z;   // limb darkening
                col = albedo * (0.12 + 0.95 * diff) * limb;
            } else {
                col = starField(px, py);
            }
            size_t idx = (static_cast<size_t>(py) * W + px) * 3;
            img[idx + 0] = (uint8_t)(std::min(1.0, col.x) * 255 + 0.5);
            img[idx + 1] = (uint8_t)(std::min(1.0, col.y) * 255 + 0.5);
            img[idx + 2] = (uint8_t)(std::min(1.0, col.z) * 255 + 0.5);
        }
    }
    stbi_write_png(file, W, W, 3, img.data(), W * 3);
    std::printf("wrote %s\n", file);
}

int main() {
    const int W = 420;
    renderRocky(planetMars(), 12, "out_mars.png", W);
    renderRocky(planetMoon(), 7, "out_moon.png", W);
    renderRocky(planetEarthlike(), 3, "out_earth.png", W);
    renderGas(gasGiantJupiter(), 5, "out_jupiter.png", W);
    renderGas(gasGiantNeptune(), 2, "out_neptune.png", W);
    return 0;
}
