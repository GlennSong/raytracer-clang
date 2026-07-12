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

// Brick tuning (device feedback: mortar amount + colour should be adjustable).
// kBrickMortar is the joint band as a fraction of the smaller cell dimension —
// bigger = more mortar showing. Courses per tile MUST stay EVEN: a running bond
// staggers odd rows, so an odd count seams two aligned rows at every vertical
// tile repeat (the "double row of bricks" device bug).
constexpr int    kBrickCoursesU = 4;      // bricks per tile row
constexpr int    kBrickCoursesV = 8;      // courses per tile (EVEN — see above)
constexpr double kBrickMortar   = 0.12;   // mortar band width (fraction of cell)
const     Vec3   kMortarColor{0.78, 0.75, 0.70};   // cream-grey lime mortar

SurfSample evalBrick(double u, double v, uint32_t seed) {
    SurfSample s;
    CellHit c = gridCell(u, v, kBrickCoursesU, kBrickCoursesV, true, kBrickMortar);
    double j = c.joint;
    double bump = wrapFbm(u, v, 16, 2, seed) - 0.5;
    double brickV = 0.85 + 0.15 * ihash(c.cu, c.cv, seed) + 0.08 * bump;
    if (ihash(c.cu * 3 + 1, c.cv * 5 + 2, seed) < 0.12) brickV *= 0.7;   // burnt
    s.h = mixd(0.25, brickV, j);                       // mortar recessed
    Vec3 brick = mixv(Vec3(0.50, 0.22, 0.16), Vec3(0.62, 0.40, 0.28),
                      ihash(c.cu, c.cv, seed));
    s.albedo = mixv(kMortarColor, brick * (0.85 + 0.25 * brickV), j);
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
    // Slabs at ~1.2 m (3 per 3.6 m tile). Two device notes drive this:
    // (1) "the pbr texture ... is too noisy" — the old high-freq speckle
    //     (wrapNoise at freq 70) is gone; only a gentle low-frequency mottle
    //     and a faint per-slab value shift remain, so the concrete reads calm.
    // (2) "the concrete divides ... aren't emphasized with a bump" — the joint
    //     is a crisp, DEEP height groove (0.12 vs a flat 0.9 face) so the baked
    //     normal map shows a real scored line, not a hairline colour change.
    CellHit c = gridCell(u, v, 3, 3, false, 0.06);
    double mott = wrapFbm(u, v, 5, 3, seed) - 0.5;          // slow mottle, not speckle
    double slabV = 0.95 + 0.045 * (ihash(c.cu, c.cv, seed) - 0.5) + 0.03 * mott;
    s.h = mixd(0.12, 0.9, c.joint);                         // deep scored groove
    s.albedo = Vec3(0.63, 0.62, 0.60) * slabV * mixd(0.70, 1.0, c.joint);
    s.rough = 0.92;
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

// --- Rooftop HVAC kit (VentGrille / UtilityPanel / FanTop) ---------------
// Distance to a vertical capsule (stadium) centred in a cell, for punched
// intake holes: negative inside.
double capsuleSdf(double x, double y, double halfLen, double radius) {
    const double cy = std::max(-halfLen, std::min(halfLen, y));
    return std::sqrt(x * x + (y - cy) * (y - cy)) - radius;
}

SurfSample evalVent(double u, double v, uint32_t seed) {
    // 6 x 3 capsule holes per tile, matte-dark in a metallic casing; the height
    // dips into each hole with a slightly PROUD rim so the baked normal gives
    // the punched-metal lip the device asked for.
    const double cols = 6, rows = 3;
    const double cu = u * cols, cv = v * rows;
    const double fx = (cu - std::floor(cu)) - 0.5;
    const double fy = (cv - std::floor(cv)) - 0.5;
    // capsule occupies the middle of the cell: vertical half-length + radius
    const double d = capsuleSdf(fx * 1.6, fy * 1.1, 0.16, 0.13);
    const double hole = clamp01(-d * 18.0);           // 1 deep inside the hole
    const double rim = clamp01(1.0 - std::fabs(d) * 22.0) * (d > 0 ? 1.0 : 0.0);
    const double brush =
        0.03 * ihash(static_cast<int>(u * 40.0), static_cast<int>(v * 3.0),
                     seed ^ 0x7a3f00d1u);
    SurfSample s;
    s.h = clamp01(0.62 - 0.5 * hole + 0.22 * rim);
    const double body = 0.52 + brush;
    s.albedo = Vec3(body, body, body + 0.01) * (1.0 - hole) +
               Vec3(0.030, 0.030, 0.032) * hole;
    s.rough = 0.34 * (1.0 - hole) + 0.92 * hole;      // matte hole, satin metal
    s.metal = 0.95 * (1.0 - hole);                    // hole reads as void
    return s;
}

SurfSample evalUtilityPanel(double u, double v, uint32_t seed) {
    // 2 x 2 riveted panels with recessed seams and one darker access hatch.
    const double pu = u * 2.0, pv = v * 2.0;
    const double fu = pu - std::floor(pu), fv = pv - std::floor(pv);
    const double seam = clamp01(1.0 - std::min(std::min(fu, 1.0 - fu),
                                               std::min(fv, 1.0 - fv)) * 26.0);
    // rivets: bumps near each panel corner
    const double rx = std::min(fu, 1.0 - fu) - 0.07;
    const double ry = std::min(fv, 1.0 - fv) - 0.07;
    const double rivet = clamp01(1.0 - std::sqrt(rx * rx + ry * ry) * 30.0);
    // hatch: a recessed darker rectangle in the lower-left panel of the tile
    const bool hatchPanel = pu < 1.0 && pv < 1.0;
    const double hatch = hatchPanel && fu > 0.25 && fu < 0.75 && fv > 0.2 &&
                                 fv < 0.7
                             ? 1.0
                             : 0.0;
    const double wear =
        0.05 * ihash(static_cast<int>(u * 9.0), static_cast<int>(v * 9.0),
                     seed ^ 0x00babe11u);
    SurfSample s;
    s.h = clamp01(0.6 - 0.3 * seam + 0.25 * rivet - 0.12 * hatch);
    const double tone = 0.46 + wear - 0.10 * hatch - 0.08 * seam;
    s.albedo = Vec3(tone, tone + 0.005, tone + 0.012);
    s.rough = clamp01(0.42 + 0.25 * hatch + 0.2 * seam);
    s.metal = 0.9;
    return s;
}

SurfSample evalFanTop(double u, double v, uint32_t seed) {
    // Radial blades under a hub, smooth ring outside the blade disc — the disc
    // mesh bakes centred UVs, and the cowl's sides sample the outer ring so
    // they stay smooth (device: "sides look smooth, top appears to have a
    // fan blade").
    (void)seed;
    const double x = u - 0.5, y = v - 0.5;
    const double r = std::sqrt(x * x + y * y) * 2.0;   // 0 centre, 1 at tile edge
    SurfSample s;
    if (r > 0.96) {                                   // casing ring / cowl side
        s.h = 0.62; s.albedo = Vec3(0.40, 0.41, 0.42);
        s.rough = 0.38; s.metal = 0.9;
        return s;
    }
    const double hub = clamp01(1.0 - (r - 0.16) * 14.0);
    const double nBlades = 5.0;
    double th = std::atan2(y, x) / (2.0 * 3.14159265358979323846);
    th -= std::floor(th);
    // blade sawtooth, twisted with radius; height ramps across each blade so
    // the normal map reads pitched blades
    double bladePhase = th * nBlades + r * 0.35;
    bladePhase -= std::floor(bladePhase);
    const double blade = clamp01(1.0 - std::fabs(bladePhase - 0.42) * 3.2);
    const double bladeMask = (r > 0.14 && r < 0.92) ? 1.0 : 0.0;
    s.h = clamp01(0.18 + 0.30 * blade * bladeMask + 0.45 * hub);
    const double tone = 0.16 + 0.10 * blade * bladeMask + 0.24 * hub;
    s.albedo = Vec3(tone, tone, tone + 0.008);
    s.rough = clamp01(0.55 - 0.15 * hub);
    s.metal = 0.75;
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
        case Surface::VentGrille:      return evalVent(u, v, seed);
        case Surface::UtilityPanel:    return evalUtilityPanel(u, v, seed);
        case Surface::FanTop:          return evalFanTop(u, v, seed);
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
    // Bump strength scales with resolution so the relief reads the same at any
    // size. Kept strong: facades should visibly catch light on the mortar/joint
    // grooves, not look like a flat decal.
    const double strength = size * 0.035;
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x) {
            size_t i = static_cast<size_t>(y) * size + x;
            double dx = (H(x + 1, y) - H(x - 1, y)) * strength;
            double dy = (H(x, y + 1) - H(x, y - 1)) * strength;
            Vec3 n = normalize(Vec3(-dx, -dy, 1.0));
            m.normal.pixels[i * 3 + 0] = u8(n.x * 0.5 + 0.5);
            m.normal.pixels[i * 3 + 1] = u8(n.y * 0.5 + 0.5);
            m.normal.pixels[i * 3 + 2] = u8(n.z * 0.5 + 0.5);
            // AO: how far this texel sits below a small neighbourhood average —
            // darkens the recessed joints/crevices.
            double avg = 0;
            for (int dj = -2; dj <= 2; ++dj)
                for (int di = -2; di <= 2; ++di) avg += H(x + di, y + dj);
            avg /= 25.0;
            double ao = 1.0 - clamp01((avg - hgt[i]) * 4.0);
            m.ao.pixels[i] = u8(0.2 + 0.8 * ao);
        }
    return m;
}

double surfaceWorldTileSize(Surface surface) {
    switch (surface) {
        case Surface::Brick:           return 0.8;   // 4 bricks ~0.2 m, 8 courses (even = seamless bond)
        case Surface::RoofShingle:     return 1.5;
        case Surface::RoofTile:        return 1.1;
        case Surface::Cobblestone:     return 0.9;
        case Surface::CorrugatedMetal: return 1.0;
        case Surface::WoodSiding:      return 1.1;
        case Surface::Pavement:        return 3.6;   // 3 slabs ~1.2 m
        case Surface::Concrete:        return 3.0;
        case Surface::Asphalt:         return 4.0;
        case Surface::Stucco:          return 2.5;
        case Surface::VentGrille:      return 0.9;   // 6 capsule holes across ~0.9 m
        case Surface::UtilityPanel:    return 1.6;   // 2 panels ~0.8 m each
        case Surface::FanTop:          return 1.2;   // one fan per tile (disc bakes own UVs)
        default:                       return 2.0;
    }
}

}  // namespace engine
