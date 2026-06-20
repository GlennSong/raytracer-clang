#include "surface_maps.h"

#include <algorithm>
#include <cmath>

namespace engine {

using Surface = RenderMaterial::Surface;

namespace {

double clamp01(double x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
double smooth(double e0, double e1, double x) {
    double t = clamp01((x - e0) / (e1 - e0));
    return t * t * (3 - 2 * t);
}
double frac(double x) { return x - std::floor(x); }
double mixd(double a, double b, double t) { return a + (b - a) * t; }
Vec3 mixv(const Vec3& a, const Vec3& b, double t) { return a + (b - a) * t; }

// Integer hash -> [0,1], seeded. Stable across platforms (no std::sin reliance).
double ihash(int x, int y, uint32_t seed) {
    uint32_t h = static_cast<uint32_t>(x) * 374761393u +
                 static_cast<uint32_t>(y) * 668265263u + seed * 2246822519u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= h >> 16;
    return (h & 0xFFFFFF) / static_cast<double>(0xFFFFFF);
}
// Value noise on a freq x freq lattice that WRAPS at the tile edge (seamless).
double wrapNoise(double u, double v, int freq, uint32_t seed) {
    double x = u * freq, y = v * freq;
    int xi = static_cast<int>(std::floor(x)), yi = static_cast<int>(std::floor(y));
    double xf = x - xi, yf = y - yi;
    auto H = [&](int a, int b) {
        a = ((a % freq) + freq) % freq;
        b = ((b % freq) + freq) % freq;
        return ihash(a, b, seed);
    };
    double ux = xf * xf * (3 - 2 * xf), uy = yf * yf * (3 - 2 * yf);
    return mixd(mixd(H(xi, yi), H(xi + 1, yi), ux),
                mixd(H(xi, yi + 1), H(xi + 1, yi + 1), ux), uy);
}
double wrapFbm(double u, double v, int baseFreq, int octaves, uint32_t seed) {
    double sum = 0, amp = 0.5;
    int f = baseFreq;
    for (int i = 0; i < octaves; ++i) {
        sum += amp * wrapNoise(u, v, f, seed + i * 17u);
        f *= 2; amp *= 0.5;
    }
    return sum;
}

struct SurfSample {
    double h = 0.5;      // height [0,1] (relief: high = proud, low = recessed)
    Vec3 albedo{0.6, 0.6, 0.6};
    double rough = 0.8;
    double metal = 0.0;
};

// A running-bond / grid mortar pattern shared by brick, pavement, tile, shingle.
// Returns the in-cell joint distance (0 = in the joint) plus the cell id, so the
// caller can colour the face and recess the joint.
struct CellHit { double joint; int cu; int cv; double fx; double fy; };
CellHit gridCell(double u, double v, int cellsU, int cellsV, bool runningBond,
                 double mortar) {
    double row = std::floor(v * cellsV);
    double off = (runningBond && (static_cast<int>(row) & 1)) ? 0.5 : 0.0;
    double uu = u * cellsU + off;
    double cu = std::floor(uu);
    double fx = uu - cu, fy = v * cellsV - row;
    double joint = std::min(std::min(fx, 1 - fx), std::min(fy, 1 - fy));
    CellHit c;
    c.joint = smooth(0.0, mortar, joint);   // 0 in joint .. 1 on the face
    c.cu = ((static_cast<int>(cu) % cellsU) + cellsU) % cellsU;
    c.cv = static_cast<int>(row);
    c.fx = fx; c.fy = fy;
    return c;
}

SurfSample evalBrick(double u, double v, uint32_t seed) {
    SurfSample s;
    CellHit c = gridCell(u, v, 4, 7, true, 0.10);
    double j = c.joint;
    double bump = wrapFbm(u, v, 16, 2, seed) - 0.5;
    double brickV = 0.85 + 0.15 * ihash(c.cu, c.cv, seed) + 0.08 * bump;
    if (ihash(c.cu * 3 + 1, c.cv * 5 + 2, seed) < 0.12) brickV *= 0.7;   // burnt
    s.h = mixd(0.25, brickV, j);                       // mortar recessed
    Vec3 brick = mixv(Vec3(0.50, 0.22, 0.16), Vec3(0.62, 0.40, 0.28),
                      ihash(c.cu, c.cv, seed));
    Vec3 mortar(0.74, 0.72, 0.67);
    s.albedo = mixv(mortar, brick * (0.85 + 0.25 * brickV), j);
    s.rough = mixd(0.95, 0.82, j);                     // mortar matte
    return s;
}
SurfSample evalConcrete(double u, double v, uint32_t seed) {
    SurfSample s;
    double mott = wrapFbm(u, v, 4, 4, seed);
    double fine = wrapNoise(u, v, 48, seed + 9u) - 0.5;
    // Control joints on a 2x2 grid.
    double gu = frac(u * 2) * 2 - 1, gv = frac(v * 2) * 2 - 1;
    double joint = std::min(1 - std::abs(gu), 1 - std::abs(gv));
    double jt = smooth(0.0, 0.04, joint);
    s.h = mixd(0.4, 0.5 + 0.4 * mott + 0.1 * fine, jt);
    double val = 0.86 + 0.16 * (mott - 0.5) + 0.05 * fine;
    s.albedo = Vec3(0.66, 0.66, 0.64) * val * mixd(0.8, 1.0, jt);
    s.rough = 0.9 - 0.08 * mott;
    return s;
}
SurfSample evalStucco(double u, double v, uint32_t seed) {
    SurfSample s;
    double coarse = wrapFbm(u, v, 6, 3, seed);
    double grain = wrapNoise(u, v, 64, seed + 3u);
    s.h = 0.5 + 0.3 * (grain - 0.5) + 0.15 * (coarse - 0.5);
    double val = 0.92 + 0.10 * (coarse - 0.5) + 0.08 * (grain - 0.5);
    s.albedo = Vec3(0.84, 0.79, 0.70) * val;
    s.rough = 0.88;
    return s;
}
SurfSample evalRoofTile(double u, double v, uint32_t seed) {
    SurfSample s;
    int tilesU = 6, rowsV = 5;
    double row = std::floor(v * rowsV);
    double off = (static_cast<int>(row) & 1) ? 0.5 : 0.0;
    double uu = u * tilesU + off;
    double cu = std::floor(uu), fx = uu - cu, fy = v * rowsV - row;
    double barrel = std::sin(3.14159265 * fx);          // round pan
    double lap = smooth(0.0, 0.18, fy);                 // shadow at head lap
    double valley = smooth(0.0, 0.06, std::min(fx, 1 - fx));
    s.h = (0.3 + 0.7 * barrel) * mixd(0.5, 1.0, lap) * mixd(0.4, 1.0, valley);
    double cv = ihash(static_cast<int>(cu), static_cast<int>(row), seed);
    s.albedo = mixv(Vec3(0.52, 0.24, 0.16), Vec3(0.70, 0.42, 0.26), cv) *
               mixd(0.7, 1.0, s.h);
    s.rough = 0.8;
    return s;
}
SurfSample evalShingle(double u, double v, uint32_t seed) {
    SurfSample s;
    CellHit c = gridCell(u, v, 5, 12, true, 0.06);
    double tabKey = smooth(0.0, 0.04, std::min(c.fx, 1 - c.fx));
    double drop = smooth(0.0, 0.12, c.fy);
    double grit = wrapNoise(u, v, 90, seed) - 0.5;
    s.h = mixd(0.45, 0.75, drop) * mixd(0.6, 1.0, tabKey) + 0.05 * grit;
    double cv = ihash(c.cu, c.cv, seed);
    s.albedo = Vec3(0.30, 0.30, 0.33) * (0.8 + 0.4 * cv + 0.15 * grit) *
               mixd(0.6, 1.0, drop);
    s.rough = 0.92;
    return s;
}
SurfSample evalCorrugated(double u, double v, uint32_t seed) {
    SurfSample s;
    double rib = std::cos(2 * 3.14159265 * u * 8.0);    // 8 ribs across the tile
    double rust = wrapFbm(u, v, 5, 4, seed);
    s.h = 0.5 + 0.5 * rib;
    double rmask = smooth(0.62, 0.85, rust);
    Vec3 steel = Vec3(0.55, 0.58, 0.62) * (0.72 + 0.28 * (0.5 + 0.5 * rib));
    s.albedo = mixv(steel, Vec3(0.40, 0.22, 0.12), rmask * 0.7);
    s.metal = mixd(0.85, 0.1, rmask);                   // rust isn't metallic
    s.rough = mixd(0.45, 0.85, rmask);
    return s;
}
SurfSample evalAsphalt(double u, double v, uint32_t seed) {
    SurfSample s;
    double spk = wrapNoise(u, v, 80, seed);
    double patch = wrapFbm(u, v, 4, 4, seed + 5u);
    s.h = 0.5 + 0.5 * (spk - 0.5);
    double val = 0.85 + 0.7 * (spk - 0.5) + 0.15 * (patch - 0.5);   // aggregate
    s.albedo = Vec3(0.18, 0.18, 0.19) * clamp01(val);
    s.rough = 0.96;
    return s;
}
SurfSample evalPavement(double u, double v, uint32_t seed) {
    SurfSample s;
    CellHit c = gridCell(u, v, 3, 3, false, 0.05);
    double spk = wrapNoise(u, v, 70, seed) - 0.5;
    double slabV = 0.9 + 0.12 * (ihash(c.cu, c.cv, seed) - 0.5) + 0.06 * spk;
    s.h = mixd(0.45, 0.85 + 0.1 * spk, c.joint);
    s.albedo = Vec3(0.60, 0.59, 0.57) * slabV * mixd(0.75, 1.0, c.joint);
    s.rough = 0.9;
    return s;
}
SurfSample evalCobble(double u, double v, uint32_t seed) {
    SurfSample s;
    const int cells = 5;
    double cu = u * cells, cv = v * cells;
    int iu = static_cast<int>(std::floor(cu)), iv = static_cast<int>(std::floor(cv));
    double best = 1e9, bh = 0; int bc = 0, br = 0;
    for (int dj = -1; dj <= 1; ++dj)
        for (int di = -1; di <= 1; ++di) {
            int ci = iu + di, cj = iv + dj;
            int wi = ((ci % cells) + cells) % cells, wj = ((cj % cells) + cells) % cells;
            double jx = ihash(wi, wj, seed), jy = ihash(wi + 31, wj + 17, seed);
            double px = ci + 0.5 + (jx - 0.5) * 0.6, py = cj + 0.5 + (jy - 0.5) * 0.6;
            double dx = cu - px, dy = cv - py, d = dx * dx + dy * dy;
            if (d < best) { best = d; bh = ihash(wi + 7, wj + 3, seed); bc = wi; br = wj; }
        }
    double dist = std::sqrt(best);
    double stone = smooth(0.55, 0.32, dist);            // 1 stone .. 0 gap
    double dome = smooth(0.5, 0.0, dist);               // rounded top
    s.h = mixd(0.25, 0.7 + 0.3 * dome, stone);
    Vec3 stoneCol = mixv(Vec3(0.42, 0.40, 0.38), Vec3(0.58, 0.55, 0.50), bh);
    s.albedo = mixv(Vec3(0.34, 0.32, 0.30), stoneCol, stone);
    s.rough = mixd(0.95, 0.7, stone);
    (void)bc; (void)br;
    return s;
}
SurfSample evalWood(double u, double v, uint32_t seed) {
    SurfSample s;
    int boards = 6;
    double row = std::floor(v * boards), fy = v * boards - row;
    double drop = smooth(0.0, 0.1, fy);
    double grain = wrapNoise(u * 0.5, row * 3.1 + v, 40, seed) +
                   0.3 * (wrapNoise(u, v, 120, seed + 2u) - 0.5);
    double bv = ihash(0, static_cast<int>(row), seed);
    s.h = mixd(0.45, 0.7, drop) + 0.12 * (grain - 0.5);
    s.albedo = mixv(Vec3(0.42, 0.29, 0.17), Vec3(0.56, 0.40, 0.24),
                    clamp01(0.4 + 0.5 * bv + 0.5 * (grain - 0.5))) *
               mixd(0.7, 1.0, drop);
    s.rough = 0.8;
    return s;
}

SurfSample evalSurface(Surface s, double u, double v, uint32_t seed) {
    switch (s) {
        case Surface::Brick:           return evalBrick(u, v, seed);
        case Surface::Concrete:        return evalConcrete(u, v, seed);
        case Surface::Stucco:          return evalStucco(u, v, seed);
        case Surface::RoofTile:        return evalRoofTile(u, v, seed);
        case Surface::RoofShingle:     return evalShingle(u, v, seed);
        case Surface::CorrugatedMetal: return evalCorrugated(u, v, seed);
        case Surface::Asphalt:         return evalAsphalt(u, v, seed);
        case Surface::Pavement:        return evalPavement(u, v, seed);
        case Surface::Cobblestone:     return evalCobble(u, v, seed);
        case Surface::WoodSiding:      return evalWood(u, v, seed);
        default:                       return SurfSample{};
    }
}

void alloc(TextureData& t, int size, int ch) {
    t.width = t.height = size; t.channels = ch;
    t.pixels.assign(static_cast<size_t>(size) * size * ch, 0);
}
uint8_t u8(double x) { return static_cast<uint8_t>(clamp01(x) * 255.0 + 0.5); }

}  // namespace

SurfaceMaps surfaceMaps(Surface surface, int size, uint32_t seed) {
    if (seed == 0) seed = 0x9e3779b9u;
    SurfaceMaps m;
    alloc(m.albedo, size, 3);
    alloc(m.normal, size, 3);
    alloc(m.mr, size, 3);
    alloc(m.ao, size, 1);
    alloc(m.height, size, 1);

    // First pass: sample the surface; cache height for the normal/AO pass.
    std::vector<double> hgt(static_cast<size_t>(size) * size);
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x) {
            double u = (x + 0.5) / size, v = (y + 0.5) / size;
            SurfSample s = evalSurface(surface, u, v, seed);
            size_t i = static_cast<size_t>(y) * size + x;
            hgt[i] = s.h;
            m.albedo.pixels[i * 3 + 0] = u8(s.albedo.x);
            m.albedo.pixels[i * 3 + 1] = u8(s.albedo.y);
            m.albedo.pixels[i * 3 + 2] = u8(s.albedo.z);
            m.mr.pixels[i * 3 + 0] = 255;            // R unused (keep white)
            m.mr.pixels[i * 3 + 1] = u8(s.rough);    // G = roughness
            m.mr.pixels[i * 3 + 2] = u8(s.metal);    // B = metallic
            m.height.pixels[i] = u8(s.h);
        }

    // Second pass: tangent-space normal from the height gradient (wrapped so it
    // stays seamless), and a cheap cavity AO from the local height deficit.
    auto H = [&](int x, int y) {
        x = ((x % size) + size) % size; y = ((y % size) + size) % size;
        return hgt[static_cast<size_t>(y) * size + x];
    };
    const double strength = size * 0.010;
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x) {
            size_t i = static_cast<size_t>(y) * size + x;
            double dx = (H(x + 1, y) - H(x - 1, y)) * strength;
            double dy = (H(x, y + 1) - H(x, y - 1)) * strength;
            Vec3 n = normalize(Vec3(-dx, -dy, 1.0));
            m.normal.pixels[i * 3 + 0] = u8(n.x * 0.5 + 0.5);
            m.normal.pixels[i * 3 + 1] = u8(n.y * 0.5 + 0.5);
            m.normal.pixels[i * 3 + 2] = u8(n.z * 0.5 + 0.5);
            // AO: how much this texel sits below a small neighbourhood average.
            double avg = 0;
            for (int dj = -2; dj <= 2; ++dj)
                for (int di = -2; di <= 2; ++di) avg += H(x + di, y + dj);
            avg /= 25.0;
            double ao = 1.0 - clamp01((avg - hgt[i]) * 1.6);
            m.ao.pixels[i] = u8(0.4 + 0.6 * ao);
        }
    return m;
}

double surfaceWorldTileSize(Surface surface) {
    switch (surface) {
        case Surface::Brick:           return 0.8;   // 4 bricks ~0.2 m, 7 courses
        case Surface::RoofShingle:     return 1.5;
        case Surface::RoofTile:        return 1.1;
        case Surface::Cobblestone:     return 0.9;
        case Surface::CorrugatedMetal: return 1.0;
        case Surface::WoodSiding:      return 1.1;
        case Surface::Pavement:        return 3.6;   // 3 slabs ~1.2 m
        case Surface::Concrete:        return 3.0;
        case Surface::Asphalt:         return 4.0;
        case Surface::Stucco:          return 2.5;
        default:                       return 2.0;
    }
}

}  // namespace engine
