#include "terrain_weather.h"

#include "noise.h"
#include "../level_params.h"
#include "../../log.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <thread>

namespace engine {

namespace {
// bump when the pipeline's output changes for the same inputs: every weathered cache rebakes
constexpr const char* kWeatherCodeTag = "2026-09-26.7";   // .2: massifs, peakier uplift; .3: peaks; .4: plains keep the original; .5: crags; .6: pointed summits, sharper crags; .7: refine
// rows [0, n) in parallel (the refinement's per-cell passes over 16 M cells)
template <class F>
void parallelRows(int n, F&& f) {
    const int T = std::max(1u, std::min(32u, std::thread::hardware_concurrency()));
    std::vector<std::thread> th;
    for (int t = 0; t < T; ++t)
        th.emplace_back([&, t] { for (int z = t; z < n; z += T) f(z); });
    for (auto& x : th) x.join();
}

// UPSCALE, BLUR, DETAIL, DRAIN (ADR-0127; Glenn's pointer: Josh's "Better Mountain Generators That Aren't
// Perlin Noise or Erosion" -- DLA ridge trees refined by upscale + blur per level -- and IQ's derivative-damped
// fBm). The grown grid already is the ridge tree (the drainage network grown by stream power); each level
// doubles it, blurs away the interpolation creases (smooth at EVERY scale), adds detail at that level's
// wavelength damped where the ground is already steep (clean faces, busy crests and floors), then runs a few
// stream-power steps without uplift so the new detail drains -- gullies, not pits (the old breach + water pass
// filled every noise hollow and combed the rest along the grid).
Heightmap refineGrown(const Heightmap& grown, float base0, float sea, const WeatherParams& w, uint32_t seed, std::vector<float>* areaOut) {
    Heightmap g = grown;
    const Noise dn(seed * 2246822519u + 3u);
    auto blur121 = [](Heightmap& u) {   // one binomial [1 2 1] pass each way
        const int n = u.n;
        std::vector<float> t(u.h.size());
        parallelRows(n, [&](int z) {
            for (int x = 0; x < n; ++x) {
                const int a = std::max(0, x - 1), b = std::min(n - 1, x + 1);
                const std::size_t r = static_cast<std::size_t>(z) * n;
                t[r + x] = 0.25f * u.h[r + a] + 0.5f * u.h[r + x] + 0.25f * u.h[r + b];
            }
        });
        parallelRows(n, [&](int z) {
            const int a = std::max(0, z - 1), b = std::min(n - 1, z + 1);
            for (int x = 0; x < n; ++x)
                u.h[static_cast<std::size_t>(z) * n + x] = 0.25f * t[static_cast<std::size_t>(a) * n + x] + 0.5f * t[static_cast<std::size_t>(z) * n + x] + 0.25f * t[static_cast<std::size_t>(b) * n + x];
        });
    };
    // level 0 works on the grown grid as it is; each later level doubles it
    for (int level = 0;; ++level) {
        if (level > 0 && g.n - 1 >= w.res) break;
        Heightmap u;
        if (level == 0) u = g;
        else {
            const int n0 = g.n, n = 2 * (n0 - 1) + 1;
            u.n = n;
            u.worldSize = g.worldSize;
            u.h.resize(static_cast<std::size_t>(n) * n);
            // 1. upscale (bilinear: the even nodes are the old ones), then blur the creases out
            parallelRows(n, [&](int z) {
                const int z0 = z / 2, z1 = std::min(n0 - 1, (z + 1) / 2);
                for (int x = 0; x < n; ++x) {
                    const int x0 = x / 2, x1 = std::min(n0 - 1, (x + 1) / 2);
                    u.h[static_cast<std::size_t>(z) * n + x] = 0.25f * (g.get(x0, z0) + g.get(x1, z0) + g.get(x0, z1) + g.get(x1, z1));
                }
            });
            blur121(u);
        }
        const int n = u.n;
        // 2. detail at this level's wavelength, on land, by height above the plains, damped by slope
        const double cell = u.worldSize / (n - 1);
        const double lambda = w.refineWavelengthCells * cell;
        const double amp = w.refineDetailM * std::pow(cell / 10.0, w.refineRoughness);
        {
            const std::vector<float> src = u.h;
            parallelRows(n, [&](int z) {
                for (int x = 0; x < n; ++x) {
                    const std::size_t i = static_cast<std::size_t>(z) * n + x;
                    const float h = src[i];
                    if (h <= base0 + 0.5f) continue;
                    const int xa = std::max(0, x - 1), xb = std::min(n - 1, x + 1), za = std::max(0, z - 1), zb = std::min(n - 1, z + 1);
                    const double gx = (src[static_cast<std::size_t>(z) * n + xb] - src[static_cast<std::size_t>(z) * n + xa]) / ((xb - xa) * cell);
                    const double gz = (src[static_cast<std::size_t>(zb) * n + x] - src[static_cast<std::size_t>(za) * n + x]) / ((zb - za) * cell);
                    const double damp = 1.0 / (1.0 + w.refineSlopeDamp * (gx * gx + gz * gz));
                    const double up = std::clamp((h - base0 - w.plainHeightM) / (2.0 * w.reliefRampM), 0.0, 1.0);
                    const double m = 0.25 + 0.75 * up * up * (3.0 - 2.0 * up);   // hills get a quarter of it
                    const double wx = x * cell, wz = z * cell, f = 1.0 / lambda;
                    const double qx = wx * f + 0.6 * dn.noise2(wx * f * 0.5 + 7.1 + level, wz * f * 0.5), qz = wz * f + 0.6 * dn.noise2(wx * f * 0.5, wz * f * 0.5 - 3.9 - level);
                    // half ridged (crests), half plain gradient noise (knolls, hollows)
                    const double r = 1.0 - std::abs(dn.noise2(qx + 31.0 * level, qz));
                    const double d = 0.5 * (r * r - 0.45) + 0.5 * dn.noise2(qx * 1.7 - 11.0, qz * 1.7 + 5.0 * level);
                    u.h[i] = static_cast<float>(h + amp * m * damp * d);
                }
            });
        }
        // 3. drain it: a few stream-power steps, no uplift, no rescale -- the detail becomes gullies and spurs
        const bool last = n - 1 >= w.res;
        if (w.refineIncise > 0 && w.refineInciseIterations > 0) {
            StreamPowerParams sp;
            sp.iterations = w.refineInciseIterations;
            sp.dt = 1.0;
            sp.K = w.refineIncise;
            sp.uplift = 0.0;
            sp.diffusion = 0.0;
            sp.rescale = false;
            sp.seaLevel = sea;
            const std::vector<float> zero(u.h.size(), 0.0f);
            streamPowerErode(u, zero, sp, nullptr, last ? areaOut : nullptr);
        }
        // 4. and blur once more: the single-cell channels the drainage cuts run along the grid's 8 directions
        blur121(u);
        LOG_INFO << "[weather] refine level " << level << ": " << n << "^2 (" << cell << " m), detail " << amp << " m at " << lambda << " m";
        g = std::move(u);
    }
    return g;
}
}  // namespace

WeatherParams weatherFromJson(const nlohmann::json& w, const nlohmann::json& tj) {
    WeatherParams p;
    p.res = w.value("res", p.res);
    p.roughnessM = w.value("roughness", p.roughnessM);
    p.cragM = w.value("crags", p.cragM);
    p.shapeKeep = w.value("shapeKeep", p.shapeKeep);
    p.shapeScaleM = w.value("shapeScale", p.shapeScaleM);
    p.plainHeightM = w.value("plainHeight", p.plainHeightM);
    p.massifAmount = w.value("massifs", p.massifAmount);
    p.massifScaleM = w.value("massifScale", p.massifScaleM);
    if (w.contains("peaks")) {
        const auto& pk = w["peaks"];
        p.peakHeightM = pk.value("height", p.peakHeightM);
        p.peakSpacingM = pk.value("spacing", p.peakSpacingM);
        p.peakUplift = pk.value("uplift", p.peakUplift);
    }
    p.reliefRampM = w.value("reliefRamp", p.reliefRampM);
    if (w.contains("refine")) {
        const auto& r = w["refine"];
        p.refine = true;
        p.refineDetailM = r.value("detail", p.refineDetailM);
        p.refineRoughness = r.value("roughness", p.refineRoughness);
        p.refineWavelengthCells = r.value("wavelength", p.refineWavelengthCells);
        p.refineSlopeDamp = r.value("slopeDamp", p.refineSlopeDamp);
        p.refineIncise = r.value("incise", p.refineIncise);
        p.refineInciseIterations = r.value("iterations", p.refineInciseIterations);
    }
    const nlohmann::json g = w.value("grow", nlohmann::json::object());
    p.growRes = g.value("res", p.growRes);
    p.growIterations = g.value("iterations", p.growIterations);
    p.growBlurM = g.value("blur", p.growBlurM);
    p.grow.K = g.value("K", p.grow.K);
    p.grow.diffusion = g.value("diffusion", p.grow.diffusion);
    const nlohmann::json wa = w.value("water", nlohmann::json::object());
    ErosionParams& e = p.water;
    e.vulkan = true;
    e.droplets = 0;
    e.thermalIterations = wa.value("thermal", 16);
    e.talus = tj.value("erodeTalus", e.talus);
    e.seed = tj.value("seed", 0u) + 1234u;
    e.waterSteps = wa.value("steps", 20000);
    e.breachDepth = wa.value("breach", 25.0f);
    e.waterDeposit = wa.value("deposit", 0.15f);
    e.waterRockHardness = wa.value("rockHardness", e.waterRockHardness);
    e.waterCreep = wa.value("creep", e.waterCreep);
    e.waterRain = wa.value("rain", e.waterRain);
    return p;
}

Heightmap weatherTerrain(const TerrainParams& tpIn, uint32_t seed, const WeatherParams& w, Heightmap* grownOut) {
    TerrainParams tp = tpIn;
    const Noise noise(seed);
    const float sea = tp.seaLevel > -1e29 ? static_cast<float>(tp.seaLevel) : -1e30f;

    // 1. THE GROW (ADR-0125), on the coarse grid
    TerrainParams ctp = tp;
    ctp.resolution = w.growRes;
    Heightmap coarse = bakeHeightmap(ctp, noise);
    const Heightmap coarse0 = coarse;
    const int cn = coarse.n;
    const double cellM = tp.size / w.growRes;
    const int r = std::max(1, static_cast<int>(w.growBlurM / cellM));
    std::vector<float> u(coarse.h.begin(), coarse.h.end()), tmp(u.size());
    for (int pass = 0; pass < 3; ++pass)   // box blur x3: where the ranges are, none of the noise
        for (int axis = 0; axis < 2; ++axis) {
            for (int z = 0; z < cn; ++z)
                for (int x = 0; x < cn; ++x) {
                    double s = 0.0;
                    int c = 0;
                    for (int k = -r; k <= r; ++k) {
                        const int xx = axis == 0 ? std::clamp(x + k, 0, cn - 1) : x, zz = axis == 1 ? std::clamp(z + k, 0, cn - 1) : z;
                        s += u[static_cast<std::size_t>(zz) * cn + xx];
                        ++c;
                    }
                    tmp[static_cast<std::size_t>(z) * cn + x] = static_cast<float>(s / c);
                }
            u.swap(tmp);
        }
    const float base0 = sea > -1e29f ? sea : *std::min_element(u.begin(), u.end());
    float hiU = base0;
    for (float v : u) hiU = std::max(hiU, v);
    // PEAKS (Glenn: "the heights of those mountains look fairly regular"): the level's range is one broad
    // ridge of one height, and noise over it only ripples the wall. Real ranges stand up as separate summits
    // of unlike height with saddles between, so place them: one candidate a jittered cell of peakSpacingM,
    // kept where the range is high, each a cone of its own height and reach. The field lifts the uplift (the
    // rivers then radiate from the summits) and the kept envelope (the height the summits stand at).
    std::vector<float> peak(u.size(), 0.0f);
    if (w.peakHeightM > 0.0 && w.peakSpacingM > 0.0) {
        auto hash = [&](int a, int b2, int k) {
            uint32_t h = static_cast<uint32_t>(a) * 73856093u ^ static_cast<uint32_t>(b2) * 19349663u ^ static_cast<uint32_t>(k) * 83492791u ^ seed * 2654435761u;
            h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
            return (h & 0xffffffu) / 16777215.0;
        };
        const double sp = w.peakSpacingM, span = tp.size;
        const int cells = static_cast<int>(std::ceil(span / sp));
        for (int gz = 0; gz < cells; ++gz)
            for (int gx = 0; gx < cells; ++gx) {
                const double px = (gx + 0.15 + 0.7 * hash(gx, gz, 1)) * sp, pz = (gz + 0.15 + 0.7 * hash(gx, gz, 2)) * sp;
                const int cx = std::clamp(static_cast<int>(px / cellM), 0, cn - 1), cz = std::clamp(static_cast<int>(pz / cellM), 0, cn - 1);
                const float bAt = std::clamp((u[static_cast<std::size_t>(cz) * cn + cx] - base0) / std::max(1e-3f, hiU - base0), 0.0f, 1.0f);
                if (bAt < 0.25f) continue;   // summits stand on the range, not on the plains
                // heights skewed low: a few big summits over many lesser ones
                const double t = hash(gx, gz, 3);
                const double amp = (0.15 + 0.85 * t * t * t) * std::sqrt((bAt - 0.25) / 0.75);
                const double reach = sp * (0.45 + 0.4 * hash(gx, gz, 4));
                const int rc = static_cast<int>(reach / cellM) + 1;
                for (int z = std::max(0, cz - rc); z <= std::min(cn - 1, cz + rc); ++z)
                    for (int x = std::max(0, cx - rc); x <= std::min(cn - 1, cx + rc); ++x) {
                        const double d = std::hypot(x * cellM - px, z * cellM - pz) / reach;
                        if (d >= 1.0) continue;
                        const double f = amp * std::pow(1.0 - d, 1.6);   // a pointed cone (smooth shoulders read as domes)
                        float& pv = peak[static_cast<std::size_t>(z) * cn + x];
                        pv = std::max(pv, static_cast<float>(f));
                    }
            }
    }
    // the uplift over a base lift of all land, broken by two scales of noise; a rough, low start; soft and
    // hard rock in patches (a smooth symmetric uplift grew fishbone spurs; a smooth start drained straight)
    const Noise un(seed * 7919u + 17u);
    std::vector<float> upl(u.size()), erod(u.size());
    for (int z = 0; z < cn; ++z)
        for (int x = 0; x < cn; ++x) {
            const std::size_t i = static_cast<std::size_t>(z) * cn + x;
            const double wx = x * cellM, wz = z * cellM;
            const float b = std::clamp((u[i] - base0) / std::max(1e-3f, hiU - base0), 0.0f, 1.0f);
            const double big = un.fbm2(wx * 0.00035 + 3.1, wz * 0.00035 - 7.2, 3), mid = un.fbm2(wx * 0.0015 - 1.7, wz * 0.0015 + 4.4, 3);
            const bool isSea = coarse0.h[i] < base0 && sea > -1e29f;
            // peaks stand clear (b^1.6) and the mid scale varies harder: an even uplift grew summits all of a
            // height, a wall of a range (Glenn: "the heights of those mountains look fairly regular")
            upl[i] = isSea ? 0.0f : std::clamp(static_cast<float>((0.12 + 0.88 * std::pow(b, 1.6f)) * (1.0 + 0.45 * big + 0.35 * mid)), 0.0f, 1.8f) * (1.0f + static_cast<float>(w.peakUplift) * peak[i]);
            erod[i] = static_cast<float>(std::exp(0.6 * un.fbm2(wx * 0.0008 + 11.0, wz * 0.0008 - 2.0, 4)));
            coarse.h[i] = isSea ? coarse0.h[i] : static_cast<float>(base0 + 0.1 * (u[i] - base0) + 0.5 + 6.0 * (0.5 + 0.5 * un.fbm2(wx * 0.01, wz * 0.01, 4)));
        }
    StreamPowerParams sp = w.grow;
    sp.iterations = w.growIterations;
    sp.seaLevel = sea;
    {
        float lo = 1e30f, hi = -1e30f;
        for (float v : coarse0.h) if (v >= base0) { lo = std::min(lo, v); hi = std::max(hi, v); }
        sp.targetLo = lo;
        sp.targetHi = hi;
    }
    auto t0 = std::chrono::steady_clock::now();
    streamPowerErode(coarse, upl, sp, &erod);
    LOG_INFO << "[weather] grew " << cn << "^2 over " << sp.iterations << " iterations in "
             << std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() << " s";

    // KEEP THE SHAPE: the original's low frequencies, the grown terrain's high ones (land only)
    if (w.shapeKeep > 0.0) {
        auto blur = [&](const std::vector<float>& in, int rad) {
            std::vector<float> a(in), b2(in.size());
            for (int pass = 0; pass < 3; ++pass)
                for (int axis = 0; axis < 2; ++axis) {
                    for (int z = 0; z < cn; ++z) {
                        // running box sum along the row / column
                        double s = 0.0;
                        auto at = [&](int k) { const int kk = std::clamp(k, 0, cn - 1); return axis == 0 ? a[static_cast<std::size_t>(z) * cn + kk] : a[static_cast<std::size_t>(kk) * cn + z]; };
                        for (int k = -rad; k <= rad; ++k) s += at(k);
                        for (int x = 0; x < cn; ++x) {
                            const std::size_t o = axis == 0 ? static_cast<std::size_t>(z) * cn + x : static_cast<std::size_t>(x) * cn + z;
                            b2[o] = static_cast<float>(s / (2 * rad + 1));
                            s += at(x + rad + 1) - at(x - rad);
                        }
                    }
                    a.swap(b2);
                }
            return a;
        };
        const int rad = std::max(1, static_cast<int>(w.shapeScaleM / cellM / 2.0));
        const std::vector<float> lowO = blur(coarse0.h, rad), lowG = blur(coarse.h, rad);
        for (std::size_t i = 0; i < coarse.h.size(); ++i) {
            if (coarse0.h[i] < base0 && sea > -1e29f) continue;   // the sea as it was
            // the grown structure fades out on the original's LOW ground: coastal plains stay the flat
            // alluvium they were (and where the cities stand), the mountains take all of it
            const float up = std::clamp((lowO[i] - base0 - static_cast<float>(w.plainHeightM)) / static_cast<float>(w.reliefRampM), 0.0f, 1.0f);
            const float relief = up * up * (3.0f - 2.0f * up);
            // MASSIFS: the original's broad range is one even height along its length; a slow noise (~3 km)
            // lifts some stretches into high massifs and sinks others into saddles and passes
            const double wx = static_cast<double>(i % static_cast<std::size_t>(cn)) * cellM, wz = static_cast<double>(i / static_cast<std::size_t>(cn)) * cellM;
            const float massif = static_cast<float>(1.0 + w.massifAmount * un.fbm2(wx * (1.0 / w.massifScaleM) + 5.5, wz * (1.0 / w.massifScaleM) - 3.3, 3));
            // on the plains the ORIGINAL itself, not its low-pass: blurred over 1.5 km the shore sank under the
            // sea, the land-stays-land clamp pinned a flat strip at sea + 0.5 m, and the water combed it into
            // grid-straight channels (Glenn: "the beach ... has a ton of long stretch marks")
            const float form = lowO[i] + (coarse0.h[i] - lowO[i]) * (1.0f - relief);
            const float shaped = base0 + (form - base0) * (1.0f + (massif - 1.0f) * relief);
            // the summits stand on it, and the grown relief is fuller where they do (rugged high massifs,
            // softer saddles)
            const float lift = static_cast<float>(w.peakHeightM) * peak[i] * relief;
            const float kept = shaped + lift + (coarse.h[i] - lowG[i]) * relief * (0.75f + 0.6f * peak[i]);
            coarse.h[i] = static_cast<float>(coarse.h[i] + w.shapeKeep * (kept - coarse.h[i]));
            if (sea > -1e29f) coarse.h[i] = std::max(coarse.h[i], sea + 0.5f);   // land stays land
        }
    }

    // 2. THE FINE GRID: refined level by level (ADR-0127), or cubic from the grown one plus roughness
    Heightmap hm;
    if (w.refine) {
        hm = refineGrown(coarse, base0, sea, w, seed, w.drainageOut);
        if (w.roughnessM > 0.0) {
            const Noise rough(seed * 104729u + 5u);
            const double fc = tp.size / (hm.n - 1);
            parallelRows(hm.n, [&](int z) {
                for (int x = 0; x < hm.n; ++x) {
                    float& v = hm.h[static_cast<std::size_t>(z) * hm.n + x];
                    if (v > base0 + 0.5f) v += static_cast<float>(w.roughnessM * rough.fbm2(x * fc * 0.05 + 0.3, z * fc * 0.05 - 0.7, 3));
                }
            });
        }
    } else {
    hm.n = w.res + 1;
    hm.worldSize = static_cast<float>(tp.size);
    hm.h.resize(static_cast<std::size_t>(hm.n) * hm.n);
    auto cr = [](double p0, double p1, double p2, double p3, double t) {
        return p1 + 0.5 * t * (p2 - p0 + t * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 + t * (3.0 * (p1 - p2) + p3 - p0)));
    };
    const Noise rough(seed * 104729u + 5u);
    // CRAGS (Glenn: "the mountains seem to have no detail"): the grown grid is ~20 m, cubic between -- smooth
    // up close. Mountain ground takes cragM of warped ridged noise (420/155/57 m), by height and steepness,
    // before the water: the water then cuts gullies down it and fills the hollows, so it reads as rock
    std::vector<float> steep(coarse.h.size(), 0.0f);
    for (int z = 1; z < cn - 1; ++z)
        for (int x = 1; x < cn - 1; ++x) {
            const double gx = (coarse.get(x + 1, z) - coarse.get(x - 1, z)) / (2.0 * cellM), gz = (coarse.get(x, z + 1) - coarse.get(x, z - 1)) / (2.0 * cellM);
            const double sl = std::sqrt(gx * gx + gz * gz);
            steep[static_cast<std::size_t>(z) * cn + x] = static_cast<float>(std::clamp((sl - 0.15) / 0.45, 0.0, 1.0));
        }
    auto ridged = [&](double wx, double wz) {
        double sum = 0.0, amp = 1.0, f = 1.0 / 420.0, norm = 0.0;
        for (int o = 0; o < 3; ++o) {
            const double qx = wx * f + 0.8 * rough.fbm2(wx * f * 0.5 + 17.0, wz * f * 0.5, 2), qz = wz * f + 0.8 * rough.fbm2(wx * f * 0.5, wz * f * 0.5 - 9.0, 2);
            const double r = 1.0 - std::abs(rough.noise2(qx + o * 31.7, qz - o * 12.9));
            sum += amp * (r * r * r - 0.3);   // sharp crests, broad hollows
            norm += amp;
            amp *= 0.55;
            f *= 2.7;
        }
        return sum / norm;
    };
    const double fineCell = tp.size / w.res;
    for (int z = 0; z < hm.n; ++z)
        for (int x = 0; x < hm.n; ++x) {
            const double gx = static_cast<double>(x) * (cn - 1) / (hm.n - 1), gz = static_cast<double>(z) * (cn - 1) / (hm.n - 1);
            const int x1 = std::min(static_cast<int>(gx), cn - 2), z1 = std::min(static_cast<int>(gz), cn - 2);
            const double tx = gx - x1, tz = gz - z1;
            double rows[4];
            for (int j = 0; j < 4; ++j) {
                const int zz = std::clamp(z1 - 1 + j, 0, cn - 1);
                auto at = [&](int xx) { return static_cast<double>(coarse.get(std::clamp(xx, 0, cn - 1), zz)); };
                rows[j] = cr(at(x1 - 1), at(x1), at(x1 + 1), at(x1 + 2), tx);
            }
            double hv = cr(rows[0], rows[1], rows[2], rows[3], tz);
            if (hv > base0 + 0.5) hv += w.roughnessM * rough.fbm2(x * fineCell * 0.02 + 0.3, z * fineCell * 0.02 - 0.7, 4);
            if (w.cragM > 0.0) {
                const double hUp = std::clamp((hv - base0 - w.plainHeightM - w.reliefRampM) / (2.0 * w.reliefRampM), 0.0, 1.0);
                if (hUp > 0.0) {
                    const double st = (steep[static_cast<std::size_t>(z1) * cn + x1] * (1 - tx) + steep[static_cast<std::size_t>(z1) * cn + x1 + 1] * tx) * (1 - tz)
                                    + (steep[static_cast<std::size_t>(z1 + 1) * cn + x1] * (1 - tx) + steep[static_cast<std::size_t>(z1 + 1) * cn + x1 + 1] * tx) * tz;
                    const double m = hUp * hUp * (3.0 - 2.0 * hUp) * (0.35 + 0.65 * st);
                    hv += w.cragM * m * ridged(x * fineCell, z * fineCell);
                }
            }
            hm.set(x, z, static_cast<float>(hv));
        }
    }
    if (grownOut) *grownOut = hm;

    // 3. BREACH + WATER + THERMAL (ADR-0123/0124), on the GPU
    ErosionParams e = w.water;
    e.seaLevel = sea;
    if (e.waterSteps <= 0 && e.droplets <= 0 && e.thermalIterations <= 0 && e.breachDepth <= 0.0f) return hm;
    t0 = std::chrono::steady_clock::now();
    erode(hm, e);
    LOG_INFO << "[weather] water " << e.waterSteps << " steps on " << hm.n << "^2 (" << erosionBackendTag(&e) << ") in "
             << std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() << " s";
    return hm;
}

Heightmap weatheredTerrainCached(const nlohmann::json& tjIn) {
    nlohmann::json tj = tjIn;
    tj.erase("rivers");   // computed ON this ground
    std::uint64_t key = 1469598103934665603ULL;
    auto fold = [&](const std::string& s) { for (unsigned char c : s) { key ^= c; key *= 1099511628211ULL; } };
    fold(kWeatherCodeTag);
    fold(tj.dump());
    char path[256];
    std::snprintf(path, sizeof path, "cache/terrain/weather_%016llx.bin", static_cast<unsigned long long>(key));
    Heightmap hm;
    if (std::getenv("RT_NOCACHE") == nullptr)
        if (std::FILE* f = std::fopen(path, "rb")) {
            int n = 0;
            float size = 0.0f;
            bool ok = std::fread(&n, sizeof n, 1, f) == 1 && std::fread(&size, sizeof size, 1, f) == 1 && n > 2 && n < 70000;
            if (ok) {
                hm.n = n;
                hm.worldSize = size;
                hm.h.resize(static_cast<std::size_t>(n) * n);
                ok = std::fread(hm.h.data(), sizeof(float), hm.h.size(), f) == hm.h.size();
            }
            std::fclose(f);
            if (ok) { LOG_INFO << "[weather] cached " << path; return hm; }
        }
    const TerrainParams tp = readTerrainParams(tj);
    hm = weatherTerrain(tp, tj.value("seed", 0u), weatherFromJson(tj["weather"], tj));
    std::error_code ec;
    std::filesystem::create_directories("cache/terrain", ec);
    if (std::FILE* f = std::fopen(path, "wb")) {
        std::fwrite(&hm.n, sizeof hm.n, 1, f);
        std::fwrite(&hm.worldSize, sizeof hm.worldSize, 1, f);
        std::fwrite(hm.h.data(), sizeof(float), hm.h.size(), f);
        std::fclose(f);
        LOG_INFO << "[weather] baked and cached " << path;
    }
    return hm;
}

}  // namespace engine
