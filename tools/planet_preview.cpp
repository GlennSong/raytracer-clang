// Headless "from space" preview: orthographic disc render of the procedural
// planets, so the generator can be eyeballed without a GPU. Not part of the build.
//   make planet_preview && ./planet_preview   (writes out_*.png)
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <tinygltf/stb_image_write.h>

#include "engine/procgen/planet.h"
#include "engine/procgen/atmosphere.h"
#include "rt_math.h"

#include <cmath>
#include <vector>
#include <cstdint>
#include <cstdio>

using namespace engine;

static uint32_t hashU(uint32_t x){ x^=x>>16; x*=0x7feb352du; x^=x>>15; x*=0x846ca68bu; x^=x>>16; return x; }

static Vec3 starField(int px, int py) {
    uint32_t h = hashU(px * 73856093u ^ py * 19349663u);
    double r = (h >> 8) * (1.0 / 16777216.0);
    if (r > 0.9975) { double b = 0.5 + 0.5 * ((h & 0xff) / 255.0); return Vec3(b, b, b); }
    return Vec3(0.012, 0.015, 0.025);
}

static Vec3 tonemap(const Vec3& c, double exposure) {
    // Simple exponential exposure (keeps the bright sun-lit limb from clipping hard).
    return Vec3(1.0 - std::exp(-c.x * exposure), 1.0 - std::exp(-c.y * exposure),
                1.0 - std::exp(-c.z * exposure));
}

// Transmittance from a ground point toward the sun through the atmosphere; zero if
// the planet itself blocks the sun (ground in shadow).
static Vec3 sunTransmittance(const AtmosphereParams& atm, const Vec3& P, const Vec3& sun) {
    double g0, g1;
    if (raySphere(P, sun, atm.planetRadius, g0, g1) && g0 > 1e-4) return Vec3(0, 0, 0);
    double a0, a1;
    if (!raySphere(P, sun, atm.atmosphereRadius, a0, a1)) return Vec3(1, 1, 1);
    return transmittance(atm, P, P + sun * std::max(0.0, a1));
}

// Orthographic "from space" disc of a rocky planet. `atm` may be null (airless
// bodies like the Moon). Camera looks down -Z from outside the atmosphere.
static void renderRocky(const PlanetParams& p, uint32_t seed, const char* file, int W,
                        const AtmosphereParams* atm) {
    std::vector<uint8_t> img(static_cast<size_t>(W) * W * 3);
    Vec3 sun = normalize(Vec3(0.62, 0.32, 0.72));
    const double R = 1.0;                 // unit planet in the preview
    const double amp = p.reliefFraction;
    const double halfView = 1.32;         // leaves room around the disc for the halo
    const double camZ = 3.0;

    auto disp = [&](Vec3 d) { d = normalize(d); return d * (R + amp * planetHeight(p, seed, d)); };

    for (int py = 0; py < W; py++) {
        for (int px = 0; px < W; px++) {
            double X = ((px + 0.5) / W * 2 - 1) * halfView;
            double Y = -((py + 0.5) / W * 2 - 1) * halfView;
            Vec3 origin(X, Y, camZ);
            Vec3 dir(0, 0, -1);

            Vec3 color;
            double t0, t1;
            bool hit = raySphere(origin, dir, R, t0, t1) && t0 > 0.0;

            if (hit) {
                double tg = t0;
                Vec3 P = origin + dir * tg;
                Vec3 dirN = normalize(P);
                // Relief normal from the displaced height field (finite difference).
                Vec3 t1v = cross(Vec3(0, 1, 0), dirN);
                t1v = t1v.lengthSquared() > 1e-8 ? normalize(t1v) : Vec3(1, 0, 0);
                Vec3 t2v = cross(dirN, t1v);
                double e = 0.006;
                Vec3 Pc = disp(dirN), Pu = disp(dirN + t1v * e), Pv = disp(dirN + t2v * e);
                Vec3 N = cross(Pv - Pc, Pu - Pc);
                if (N.lengthSquared() < 1e-12) N = dirN;
                N = normalize(N);
                if (dot(N, dirN) < 0) N = N * -1.0;

                double h = planetHeight(p, seed, dirN);
                Vec3 albedo = planetSurfaceColor(p, seed, dirN);
                if (p.hasOcean && h < p.seaLevel) { albedo = p.oceanColor; N = dirN; }

                double ndl = std::max(0.0, dot(N, sun));
                Vec3 ground;
                if (atm) {
                    Vec3 st = sunTransmittance(*atm, P, sun);
                    ground = albedo * st * (ndl * 3.2) + albedo * 0.03;   // + faint ambient
                    ScatterResult s = atmosphereScatter(*atm, origin, dir, sun, tg);
                    color = ground * s.transmittance + s.inScatter;
                } else {
                    ground = albedo * (0.04 + 0.96 * ndl);
                    color = ground;
                }
            } else {
                Vec3 bg = starField(px, py);
                if (atm) {
                    ScatterResult s = atmosphereScatter(*atm, origin, dir, sun, 1e9);
                    color = bg * s.transmittance + s.inScatter;   // the limb halo
                } else {
                    color = bg;
                }
            }

            color = tonemap(color, 1.1);
            size_t idx = (static_cast<size_t>(py) * W + px) * 3;
            img[idx + 0] = (uint8_t)(std::min(1.0, std::max(0.0, color.x)) * 255 + 0.5);
            img[idx + 1] = (uint8_t)(std::min(1.0, std::max(0.0, color.y)) * 255 + 0.5);
            img[idx + 2] = (uint8_t)(std::min(1.0, std::max(0.0, color.z)) * 255 + 0.5);
        }
    }
    stbi_write_png(file, W, W, 3, img.data(), W * 3);
    std::printf("wrote %s\n", file);
}

// Gas giant disc: sample the baked equirect albedo by direction, smooth sphere,
// lambert + limb darkening.
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
                double limb = 0.4 + 0.6 * z;
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
    AtmosphereParams earthAtm = atmosphereEarth(1.0);
    AtmosphereParams marsAtm = atmosphereMars(1.0);
    renderRocky(planetMars(), 12, "out_mars.png", W, &marsAtm);
    renderRocky(planetMoon(), 7, "out_moon.png", W, nullptr);        // airless
    renderRocky(planetEarthlike(), 3, "out_earth.png", W, &earthAtm);
    renderGas(gasGiantJupiter(), 5, "out_jupiter.png", W);
    renderGas(gasGiantNeptune(), 2, "out_neptune.png", W);
    return 0;
}
