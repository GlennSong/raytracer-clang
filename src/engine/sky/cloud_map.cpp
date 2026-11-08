#include "cloud_map.h"

#include <algorithm>
#include <cmath>
#include <thread>

namespace engine {

namespace {

constexpr double kPi = 3.14159265358979323846;

uint32_t mix32(uint32_t x) {
    x ^= x >> 16; x *= 0x7feb352du;
    x ^= x >> 15; x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}
// a grid square's k-th roll in [0, 1)
double roll(int64_t i, int64_t j, uint32_t salt, uint32_t k) {
    const uint32_t h = mix32(static_cast<uint32_t>(i) * 0x9E3779B1u ^ mix32(static_cast<uint32_t>(j) * 0x85EBCA77u ^ salt) ^
                             (k + 1u) * 0xC2B2AE3Du);
    return (h >> 8) * (1.0 / 16777216.0);
}
// smooth value noise in [0, 1], one octave
double vnoise(double x, double z, uint32_t salt) {
    const double fx = std::floor(x), fz = std::floor(z);
    const int64_t ix = static_cast<int64_t>(fx), iz = static_cast<int64_t>(fz);
    double tx = x - fx, tz = z - fz;
    tx = tx * tx * (3.0 - 2.0 * tx);
    tz = tz * tz * (3.0 - 2.0 * tz);
    const double a = roll(ix, iz, salt, 0), b = roll(ix + 1, iz, salt, 0);
    const double c = roll(ix, iz + 1, salt, 0), d = roll(ix + 1, iz + 1, salt, 0);
    return (a + (b - a) * tx) + ((c + (d - c) * tx) - (a + (b - a) * tx)) * tz;
}
double smooth01(double e0, double e1, double x) {
    const double t = std::clamp((x - e0) / (e1 - e0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// One type's grid: its spacing, and how many cells a square holds on average for the coverage asked of it
struct Grid {
    double spacing = 1.0, perSquare = 0.0;
    const CloudTypeSpec* type = nullptr;
    uint32_t salt = 0;
    float slot = 0.0f;
};

// A footprint's area against its radius^2 x aspect x pi: the soft edge (full to a quarter of its radius, gone by its
// rim, warped) counts about this much of the ellipse as "under cloud" at the 0.5 footprint (measured: test_cloud_map)
constexpr double kAreaShare = 0.41;
// at most this many cells a grid square (an overcast's stratus overlap: several footprints' worth per square at 85 %)
constexpr int kMaxPerSquare = 16;

}  // namespace

void cloudCirrusSweep(uint32_t seed, float& x, float& z) {
    const double a = (mix32(seed ^ 0xC1A2u) >> 8) * (1.0 / 16777216.0) * kPi;
    x = static_cast<float>(std::cos(a));
    z = static_cast<float>(std::sin(a));
}

std::array<float, 4> cloudTypeWeights(float typeLo, float typeHi) {
    std::array<float, 4> w{};
    const float lo = std::clamp(std::min(typeLo, typeHi), 0.0f, 1.0f), hi = std::clamp(std::max(typeLo, typeHi), 0.0f, 1.0f);
    if (hi - lo < 1e-3f) {
        w[static_cast<std::size_t>(std::lround(lo * 3.0f))] = 1.0f;
        return w;
    }
    float sum = 0.0f;
    for (int k = 0; k < 4; ++k) {
        const float a = std::max(lo, k / 3.0f - 1.0f / 6.0f), b = std::min(hi, k / 3.0f + 1.0f / 6.0f);
        w[static_cast<std::size_t>(k)] = std::max(0.0f, b - a);
        sum += w[static_cast<std::size_t>(k)];
    }
    for (float& x : w) x /= sum;
    return w;
}

void bakeCloudMap(const CloudBook& book, const CloudMapInputs& in, CloudMapImage& out) {
    const int n = book.mapSize;
    const double texel = static_cast<double>(book.mapExtent) / n;
    out.size = n;
    out.extent = book.mapExtent;
    out.inputs = in;
    // the corner snapped to the texel grid, so a rebake lands its texels where the last one's were
    out.originX = std::floor((in.centreX - 0.5 * book.mapExtent) / texel) * texel;
    out.originZ = std::floor((in.centreZ - 0.5 * book.mapExtent) / texel) * texel;
    out.rgba.assign(static_cast<std::size_t>(n) * n * 4, 0);

    const float cover = std::clamp(in.coverage, 0.0f, 1.0f);
    // THE CIRRUS's veils: two octaves of value noise stretched 4:1 along the sweep, cut to its cover
    const float cirrus = std::clamp(in.cirrus, 0.0f, 1.0f);
    float sweepX = 1.0f, sweepZ = 0.0f;
    cloudCirrusSweep(in.seed, sweepX, sweepZ);
    const uint32_t cirrusSalt = mix32(in.seed ^ 0x0C1Bu);
    const std::array<float, 4> w = cloudTypeWeights(in.typeLo, in.typeHi);
    std::array<Grid, 4> grids{};
    // THE CELLS AS A POISSON FIELD: footprints dropped at random cover 1 - exp(-cells per m^2 x a footprint's area)
    // of the sky, so the coverage asked for is reached exactly by -ln(1 - coverage) footprints' worth per m^2, shared
    // by the types as the weather's range spreads them -- an overcast's stratus overlapping into a deck
    const double lambdaArea = -std::log(std::max(1e-4, 1.0 - static_cast<double>(cover)));
    for (int k = 0; k < 4; ++k) {
        Grid& g = grids[static_cast<std::size_t>(k)];
        const CloudTypeSpec& t = book.types[static_cast<std::size_t>(k)];
        g.type = &t;
        g.slot = k / 3.0f;
        g.salt = mix32(in.seed * 0x632BE5ABu + static_cast<uint32_t>(k) * 0x1B873593u);
        g.spacing = 2.6 * t.radiusMax * std::sqrt(static_cast<double>(t.aspect));
        const double a = t.radiusMin, b = t.radiusMax;
        const double meanArea = kPi * (a * a + a * b + b * b) / 3.0 * t.aspect * kAreaShare;
        g.perSquare = std::min(static_cast<double>(kMaxPerSquare),
                               lambdaArea * w[static_cast<std::size_t>(k)] * g.spacing * g.spacing / meanArea);
    }

    auto rows = [&](int j0, int j1) {
        for (int j = j0; j < j1; ++j) {
            const double z = out.originZ + (j + 0.5) * texel;
            for (int i = 0; i < n; ++i) {
                const double x = out.originX + (i + 0.5) * texel;
                double best = 0.0;
                float slot = 0.0f, height = 1.0f;
                for (const Grid& g : grids) {
                    if (g.perSquare <= 0.0) continue;
                    const int64_t gi = static_cast<int64_t>(std::floor(x / g.spacing));
                    const int64_t gj = static_cast<int64_t>(std::floor(z / g.spacing));
                    for (int64_t dj = -1; dj <= 1; ++dj)
                        for (int64_t di = -1; di <= 1; ++di) {
                            const int64_t ci = gi + di, cj = gj + dj;
                            // this square's cells: the whole part of the mean, and one more by the fraction's odds
                            const int whole = static_cast<int>(g.perSquare);
                            const int count = whole + (roll(ci, cj, g.salt, 0) < g.perSquare - whole ? 1 : 0);
                            for (int m = 0; m < count; ++m) {
                                const CloudTypeSpec& t = *g.type;
                                const uint32_t o = 8u * static_cast<uint32_t>(m);
                                const double rn = roll(ci, cj, g.salt, o + 1);
                                const double r = t.radiusMin + (t.radiusMax - t.radiusMin) * rn;
                                const double reach = r * t.aspect * 1.5;
                                const double cx = (ci + roll(ci, cj, g.salt, o + 2)) * g.spacing;
                                const double cz = (cj + roll(ci, cj, g.salt, o + 3)) * g.spacing;
                                const double dx = x - cx, dz = z - cz;
                                if (dx * dx + dz * dz > reach * reach) continue;
                                const double ang = roll(ci, cj, g.salt, o + 4) * kPi;
                                const double ca = std::cos(ang), sa = std::sin(ang);
                                const double u = (dx * ca + dz * sa) / (r * t.aspect), v = (-dx * sa + dz * ca) / r;
                                // the rim warped by two octaves of noise at the cell's own scale: a cloud, not a disc
                                const double warp = vnoise(x / (0.7 * r), z / (0.7 * r), g.salt ^ 0x51u) +
                                                    0.5 * vnoise(x / (0.3 * r), z / (0.3 * r), g.salt ^ 0xA7u) - 0.75;
                                const double d = std::sqrt(u * u + v * v) * (1.0 + 0.45 * warp);
                                const double f = 1.0 - smooth01(0.25, 1.0, d);   // (a long soft edge: the march narrows it as the cell rises into a dome)
                                if (f > best) {
                                    best = f;
                                    slot = g.slot;
                                    height = static_cast<float>(std::clamp(
                                        t.low + (1.0 - t.low) * rn + 0.15 * (roll(ci, cj, g.salt, o + 5) - 0.5), 0.15, 1.0));
                                }
                            }
                        }
                }
                double veil = 0.0;
                if (cirrus > 0.0f) {
                    const double a = x * sweepX + z * sweepZ, b = -x * sweepZ + z * sweepX;
                    const double nv = 0.65 * vnoise(a / 16000.0, b / 4000.0, cirrusSalt) +
                                      0.35 * vnoise(a / 6000.0, b / 1500.0, cirrusSalt ^ 0x77u);
                    veil = smooth01(1.0 - cirrus, 1.0 - cirrus + 0.35, nv);
                }
                uint8_t* p = &out.rgba[(static_cast<std::size_t>(j) * n + i) * 4];
                p[0] = static_cast<uint8_t>(std::lround(std::clamp(best, 0.0, 1.0) * 255.0));
                p[1] = static_cast<uint8_t>(std::lround(slot * 255.0f));
                p[2] = static_cast<uint8_t>(std::lround(height * 255.0f));
                p[3] = static_cast<uint8_t>(std::lround(veil * 255.0));
            }
        }
    };
    // (half the cores at most: it runs beside the frame, which must not starve)
    const int threads = std::clamp(static_cast<int>(std::thread::hardware_concurrency()) / 2, 1, 8);
    std::vector<std::thread> pool;
    const int per = (n + threads - 1) / threads;
    for (int t = 0; t < threads; ++t) {
        const int j0 = t * per, j1 = std::min(n, j0 + per);
        if (j0 < j1) pool.emplace_back(rows, j0, j1);
    }
    for (std::thread& t : pool) t.join();
}

float cloudMapCover(const CloudMapImage& map, float footprint) {
    if (map.size <= 0) return 0.0f;
    const uint8_t cut = static_cast<uint8_t>(std::lround(std::clamp(footprint, 0.0f, 1.0f) * 255.0f));
    std::size_t under = 0;
    for (std::size_t i = 0; i < map.rgba.size(); i += 4)
        if (map.rgba[i] >= cut) ++under;
    return static_cast<float>(static_cast<double>(under) / (static_cast<double>(map.size) * map.size));
}

}  // namespace engine
