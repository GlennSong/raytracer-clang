#include "cloud_map.h"

#include "../../hash.h"            // lowbias32, hashPair, hashCell, unit24
#include "../../rt_math.h"         // smoothstep
#include "../procgen/noise.h"      // Noise: the gradient noise, its curl
#include <algorithm>
#include <cmath>
#include <vector>

namespace engine {

namespace {


// a grid square's k-th roll in [0, 1) (hash.h)
double roll(int64_t i, int64_t j, uint32_t salt, uint32_t k) {
    return unit24(hashCell(static_cast<uint32_t>(i), static_cast<uint32_t>(j), hashPair(salt, k)));
}
// the gradient noise as a 0..1 field (procgen/noise.h)
double field01(const Noise& n, double x, double z) { return 0.5 + 0.5 * n.noise2(x, z); }

// One type's grid: its spacing, and how many cells a square holds on average for the coverage asked of it
struct Grid {
    double spacing = 1.0, perSquare = 0.0;
    // each square's mean cells over the map (and a square's margin round it), the air's field there: worked out once a
    // bake, not per texel
    int64_t i0 = 0, j0 = 0, ni = 0, nj = 0;
    std::vector<double> mean;
    const CloudTypeSpec* type = nullptr;
    uint32_t salt = 0;
    float slot = 0.0f;
};

// A footprint's area against its radius^2 x aspect x pi: the soft edge (full to a quarter of its radius, gone by its
// rim, warped) counts about this much of the ellipse as "under cloud" at the 0.5 footprint (measured: test_cloud_map)
constexpr double kAreaShare = 0.41;
// at most this many cells a grid square (an overcast's stratus overlap: several footprints' worth per square at 85 %)
constexpr int kMaxPerSquare = 16;

// A cell's radius from its roll: SKEWED to the small, as a real cloud field is -- many small cumulus, few big ones
// (the roll raised to `skew`; 1 even)
double cellRadius(const CloudTypeSpec& t, double rn, float skew) {
    return t.radiusMin + (t.radiusMax - t.radiusMin) * std::pow(rn, std::max(1.0, static_cast<double>(skew)));
}

}  // namespace

void cloudCirrusSweep(uint32_t seed, float& x, float& z) {
    const double a = unit24(hashPair(seed, 0xC1A2u)) * PI;
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
    const Noise cirrusNoise(hashPair(in.seed, 0x0C1Bu));
    // THE AIR's ORGANISATION: clouds bunch into fields and lines along the wind with clear air between, they are not
    // sprinkled evenly (the map from above was speckle: Glenn's "dense in the sky all the time"). A slow moisture
    // field, stretched along the sweep into streets, scales how many cells each square holds -- its mean about one,
    // so the cover asked for stays the sky's average. (The weather simulation, part C, will write this field.)
    // (a heavy sky is even -- an overcast's deck, a storm's mass: the bunching gives way as the cover passes 60 %)
    const double org = std::clamp(static_cast<double>(book.organisation), 0.0, 1.0) *
                       (1.0 - smoothstep(0.6, 0.9, static_cast<double>(cover)));
    const Noise airNoise(hashPair(in.seed, 0xA1F0u));
    // (the raw field: zero over wide clear stretches, rising into the fields; then scaled so its mean over the map
    // is one -- sampled, so the coverage stays the sky's average whatever its shape)
    // THE EDDIES (Glenn, 2026-11-08: "swirls and eddies in the wind that would guide the clouds"): the field read
    // through a swirling flow -- the curl of a slow noise (divergence-free: it turns and shears, never piles up), two
    // scales, so the fields and the cirrus wrap round eddies of ~10-30 km. (Part C's wind will be the real one.)
    const Noise eddyNoise(hashPair(in.seed, 0xEDD1u));
    auto swirl = [&](double& x, double& z) {
        const double scales[2] = {30000.0, 11000.0}, amps[2] = {4500.0, 1500.0};
        for (int o = 0; o < 2; ++o) {
            double cx = 0.0, cz = 0.0;
            eddyNoise.curl2(x / scales[o] + 17.3 * o, z / scales[o], cx, cz);   // (Noise::curl2: the shared curl)
            x += amps[o] * cx;
            z += amps[o] * cz;
        }
    };
    auto rawField = [&](double x, double z) {
        swirl(x, z);
        const double a = x * sweepX + z * sweepZ, b = -x * sweepZ + z * sweepX;
        const double nv = 0.6 * field01(airNoise, a / 24000.0, b / 8000.0) + 0.4 * field01(airNoise, a / 9000.0 + 31.7, b / 3500.0);
        return smoothstep(0.42, 0.72, nv);
    };
    // (sampled over a fixed 200 km round the world's origin, not the map's window: a cell's fate must not depend on
    // where the map is -- two bakes agree where they meet)
    std::vector<double> samples;
    samples.reserve(64 * 64);
    double fieldMean = 0.0;
    for (int s = 0; s < 64; ++s)
        for (int t = 0; t < 64; ++t) {
            samples.push_back(rawField((s + 0.5) / 64.0 * 200000.0 - 100000.0, (t + 0.5) / 64.0 * 200000.0 - 100000.0));
            fieldMean += samples.back();
        }
    fieldMean = std::max(1e-3, fieldMean / samples.size());
    for (double& f : samples) f = 1.0 - org + org * f / fieldMean;
    auto airField = [&](double x, double z) { return 1.0 - org + org * rawField(x, z) / fieldMean; };
    const std::array<float, 4> w = cloudTypeWeights(in.typeLo, in.typeHi);
    const Noise rimNoise(hashPair(in.seed, 0x51A7u));   // (the cells' rims; offset per type)
    std::array<Grid, 4> grids{};
    std::array<double, 4> meanAreaOf{};
    // THE CELLS AS A POISSON FIELD: footprints dropped at random cover 1 - exp(-cells per m^2 x a footprint's area)
    // of the sky, so the coverage asked for is reached exactly by -ln(1 - coverage) footprints' worth per m^2, shared
    // by the types as the weather's range spreads them -- an overcast's stratus overlapping into a deck
    // ...and where the field bunches them, the dense parts overlap and saturate: so the footprints' worth per m^2 is
    // solved over the field's samples, the mean of 1 - exp(-lambda x field) brought to the cover asked (bisection)
    double lambdaArea = -std::log(std::max(1e-4, 1.0 - static_cast<double>(cover)));
    if (org > 0.0 && cover > 0.0f) {
        double lo = 0.0, hi = 20.0;
        for (int it = 0; it < 40; ++it) {
            const double mid = 0.5 * (lo + hi);
            double got = 0.0;
            for (double f : samples) got += 1.0 - std::exp(-mid * f);
            (got / samples.size() < cover ? lo : hi) = mid;
        }
        lambdaArea = 0.5 * (lo + hi);
    }
    for (int k = 0; k < 4; ++k) {
        Grid& g = grids[static_cast<std::size_t>(k)];
        const CloudTypeSpec& t = book.types[static_cast<std::size_t>(k)];
        g.type = &t;
        g.slot = k / 3.0f;
        g.salt = hashPair(in.seed, 0x632BE5ABu + static_cast<uint32_t>(k));
        g.spacing = 2.6 * t.radiusMax * std::sqrt(static_cast<double>(t.aspect));
        // (the mean r^2 of the skewed draw, sampled)
        double meanR2 = 0.0;
        for (int s = 0; s < 256; ++s) {
            const double r = cellRadius(t, (s + 0.5) / 256.0, book.sizeSkew);
            meanR2 += r * r / 256.0;
        }
        const double meanArea = PI * meanR2 * t.aspect * kAreaShare;
        meanAreaOf[k] = meanArea;
        g.perSquare = std::min(static_cast<double>(kMaxPerSquare),
                               lambdaArea * w[static_cast<std::size_t>(k)] * g.spacing * g.spacing / meanArea);
        if (in.field) g.perSquare = 1.0;   // (each square its own, from the simulation)
        if (g.perSquare <= 0.0) continue;
        g.i0 = static_cast<int64_t>(std::floor(out.originX / g.spacing)) - 1;
        g.j0 = static_cast<int64_t>(std::floor(out.originZ / g.spacing)) - 1;
        g.ni = static_cast<int64_t>(std::ceil(book.mapExtent / g.spacing)) + 3;
        g.nj = g.ni;
        g.mean.resize(static_cast<std::size_t>(g.ni * g.nj));
        for (int64_t b = 0; b < g.nj; ++b)
            for (int64_t a = 0; a < g.ni; ++a) {
                const double ax = (g.i0 + a + 0.5) * g.spacing, az = (g.j0 + b + 0.5) * g.spacing;
                double m = 0.0;
                if (in.field) {
                    // the simulation's cloud here, as footprints' worth per m^2, shared by the types its convection leans
                    // to (this type's share of a narrow range round that point on the axis)
                    const double wx = ax - in.driftX, wz = az - in.driftZ;
                    const double c = std::min(0.97, static_cast<double>(in.field->coverAt(wx, wz)));
                    const double t = in.typeLo + (in.typeHi - in.typeLo) * in.field->convectionAt(wx, wz);
                    const std::array<float, 4> share = cloudTypeWeights(static_cast<float>(t - 0.08), static_cast<float>(t + 0.08));
                    m = -std::log(1.0 - c) * share[static_cast<std::size_t>(k)] * g.spacing * g.spacing / meanAreaOf[k];
                } else {
                    m = g.perSquare * airField(ax, az);
                }
                g.mean[static_cast<std::size_t>(b * g.ni + a)] = std::min(static_cast<double>(kMaxPerSquare), m);
            }
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
                            // this square's cells: the whole part of its mean (the air's field there), and one more by
                            // the fraction's odds
                            const double mean = g.mean[static_cast<std::size_t>((cj - g.j0) * g.ni + (ci - g.i0))];
                            const int whole = static_cast<int>(mean);
                            const int count = whole + (roll(ci, cj, g.salt, 0) < mean - whole ? 1 : 0);
                            for (int m = 0; m < count; ++m) {
                                const CloudTypeSpec& t = *g.type;
                                const uint32_t o = 8u * static_cast<uint32_t>(m);
                                const double rn = roll(ci, cj, g.salt, o + 1);
                                const double r = cellRadius(t, rn, book.sizeSkew);
                                const double reach = r * t.aspect * 1.5;
                                const double cx = (ci + roll(ci, cj, g.salt, o + 2)) * g.spacing;
                                const double cz = (cj + roll(ci, cj, g.salt, o + 3)) * g.spacing;
                                const double dx = x - cx, dz = z - cz;
                                if (dx * dx + dz * dz > reach * reach) continue;
                                const double ang = roll(ci, cj, g.salt, o + 4) * PI;
                                const double ca = std::cos(ang), sa = std::sin(ang);
                                const double u = (dx * ca + dz * sa) / (r * t.aspect), v = (-dx * sa + dz * ca) / r;
                                // the rim warped by two octaves of noise at the cell's own scale: a cloud, not a disc
                                const double warp = field01(rimNoise, x / (0.7 * r) + 11.0 * g.slot, z / (0.7 * r)) +
                                                    0.5 * field01(rimNoise, x / (0.3 * r) - 7.0 * g.slot, z / (0.3 * r) + 3.1) - 0.75;
                                const double d = std::sqrt(u * u + v * v) * (1.0 + 0.45 * warp);
                                const double f = 1.0 - smoothstep(0.25, 1.0, d);   // (a long soft edge: the march narrows it as the cell rises into a dome)
                                if (f > best) {
                                    best = f;
                                    slot = g.slot;
                                    height = static_cast<float>(std::clamp(
                                        t.low + (1.0 - t.low) * rn + 0.15 * (roll(ci, cj, g.salt, o + 5) - 0.5), 0.15, 1.0));
                                }
                            }
                        }
                }
                // THE SIMULATION's SOLID CLOUD: where it says the sky is closed, one continuous mass -- the structures
                // no scatter of cells makes (a merged afternoon mass over a mountain, a front's deck)
                if (in.field) {
                    const double wx = x - in.driftX, wz = z - in.driftZ;
                    const double mass = smoothstep(0.55, 0.92, static_cast<double>(in.field->coverAt(wx, wz)));
                    if (mass > best) {
                        best = mass;
                        const double t = in.typeLo + (in.typeHi - in.typeLo) * in.field->convectionAt(wx, wz);
                        slot = static_cast<float>(std::lround(std::clamp(t, 0.0, 1.0) * 3.0)) / 3.0f;
                        height = 1.0f;
                    }
                }
                double veil = 0.0;
                if (cirrus > 0.0f) {
                    double sx = x, sz = z;
                    swirl(sx, sz);   // (the high cloud wraps round the same eddies)
                    const double a = sx * sweepX + sz * sweepZ, b = -sx * sweepZ + sz * sweepX;
                    const double nv = 0.65 * field01(cirrusNoise, a / 16000.0, b / 4000.0) +
                                      0.35 * field01(cirrusNoise, a / 6000.0 + 53.0, b / 1500.0);
                    veil = smoothstep(1.0 - cirrus, 1.0 - cirrus + 0.35, nv);
                }
                uint8_t* p = &out.rgba[(static_cast<std::size_t>(j) * n + i) * 4];
                p[0] = static_cast<uint8_t>(std::lround(std::clamp(best, 0.0, 1.0) * 255.0));
                p[1] = static_cast<uint8_t>(std::lround(slot * 255.0f));
                p[2] = static_cast<uint8_t>(std::lround(height * 255.0f));
                p[3] = static_cast<uint8_t>(std::lround(veil * 255.0));
            }
        }
    };
    // (on the caller's thread, row by row: CloudMapSystem's bake runs on a worker of its own beside the frame, and
    // the shared pool -- parallelRows -- is the frame's, where a bake's tens of ms of rows would stall its jobs)
    rows(0, n);
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
