#include "test_framework.h"
#include "../src/engine/procgen/terrain_weather.h"

#include "../src/engine/procgen/erosion.h"
#include "../src/engine/procgen/erosion_gpu.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

using namespace engine;

namespace {

// Synthetic rolling hills with a regional slope: per-cell gradients up to
// ~2.4 height units exceed the default talus (1.2), so thermal has work to do.
Heightmap makeHills(int res) {
    Heightmap hm;
    hm.n = res + 1;
    hm.worldSize = 400.0f;
    hm.h.resize(static_cast<size_t>(hm.n) * hm.n);
    for (int z = 0; z < hm.n; z++)
        for (int x = 0; x < hm.n; x++) {
            float fx = x * 0.08f, fz = z * 0.08f;
            float h = 30.0f * std::sin(fx) * std::cos(fz) +
                      6.0f * std::sin(fx * 3.1f + 1.7f) *
                          std::sin(fz * 2.7f + 0.3f) +
                      0.05f * x;
            hm.set(x, z, h);
        }
    return hm;
}

struct Stats {
    double mean = 0, var = 0, mn = 1e30, mx = -1e30;
};
Stats stats(const Heightmap& hm) {
    Stats s;
    for (float v : hm.h) {
        s.mean += v;
        s.mn = std::min(s.mn, (double)v);
        s.mx = std::max(s.mx, (double)v);
    }
    s.mean /= hm.h.size();
    for (float v : hm.h) s.var += (v - s.mean) * (v - s.mean);
    s.var /= hm.h.size();
    return s;
}

double rmsDiff(const Heightmap& a, const Heightmap& b) {
    double sum = 0;
    for (size_t i = 0; i < a.h.size(); i++) {
        double d = (double)a.h[i] - (double)b.h[i];
        sum += d * d;
    }
    return std::sqrt(sum / a.h.size());
}

ErosionParams testParams() {
    ErosionParams p;
    p.vulkan = true;   // the GPU runs only when asked (ADR-0122)
    return p;          // thermal at the shipped defaults; no water
}

void runCpu(Heightmap& hm, const ErosionParams& p) {
    setenv("RT_CPU_EROSION", "1", 1);   // pin the reference sim
    erode(hm, p);
    unsetenv("RT_CPU_EROSION");
}

}  // namespace

TEST_CASE(erosion_gpu_unavailable_returns_false_and_env_pins_cpu) {
    // The env pin always reads as the CPU backend, GPU present or not.
    setenv("RT_CPU_EROSION", "1", 1);
    CHECK(std::strcmp(erosionBackendTag(), "cpu") == 0);
    unsetenv("RT_CPU_EROSION");

    if (!erodeGpuAvailable()) {
        // Graceful skip: no device (or no kernels) -- erodeGpu must decline so erode() runs the CPU, and
        // the cache tag must say so.
        Heightmap hm = makeHills(32);
        CHECK(!erodeGpu(hm, testParams()));
        const ErosionParams p = testParams();
        CHECK(std::strcmp(erosionBackendTag(&p), "cpu") == 0);
        std::printf("    [skip] no GPU erosion available\n");
        return;
    }
    const ErosionParams asks = testParams();
    CHECK(std::strcmp(erosionBackendTag(&asks), "vk-erosion-v1") == 0);
    ErosionParams notAsked = testParams();
    notAsked.vulkan = false;
    CHECK(std::strcmp(erosionBackendTag(&notAsked), "cpu") == 0);
}

TEST_CASE(erosion_gpu_two_runs_bit_identical) {
    if (!erodeGpuAvailable()) {
        std::printf("    [skip] no GPU erosion available\n");
        return;
    }
    // Run-to-run determinism is the load-bearing contract: fixed-point integer atomics make the result
    // independent of GPU thread scheduling -- two runs must match to the bit.
    const ErosionParams p = testParams();
    Heightmap a = makeHills(256), b = makeHills(256);
    CHECK(erodeGpu(a, p));
    CHECK(erodeGpu(b, p));
    CHECK(a.h.size() == b.h.size());
    CHECK(std::memcmp(a.h.data(), b.h.data(), a.h.size() * sizeof(float)) == 0);
}

TEST_CASE(erosion_gpu_thermal_matches_cpu) {
    if (!erodeGpuAvailable()) {
        std::printf("    [skip] no GPU erosion available\n");
        return;
    }
    // The same model on both: each pass finds every cell's lowest neighbour on the heights as they stood,
    // moves half the excess over the talus, then applies it all. They differ only by the GPU's 2^-16
    // fixed point, so the results agree far below the slumping itself.
    const ErosionParams p = testParams();
    const Heightmap input = makeHills(256);
    Heightmap gpu = input, cpu = input;
    CHECK(erodeGpu(gpu, p));
    runCpu(cpu, p);
    const Stats si = stats(input), sg = stats(gpu), sc = stats(cpu);
    CHECK(sg.var < si.var);   // slopes slumped
    CHECK(sc.var < si.var);
    const double rmsGC = rmsDiff(gpu, cpu), rmsCI = rmsDiff(cpu, input);
    std::printf("    gpu-vs-cpu rms %.6f, thermal carved rms %.4f\n", rmsGC, rmsCI);
    CHECK(rmsCI > 0.01);
    CHECK(rmsGC < 0.01 * rmsCI);
}

// THE WATER MODEL (ADR-0123, Mei et al. 2007 pipe model): on a V valley tilted down its length, rain runs
// off the slopes and collects along the floor; nothing blows up (every height finite, no spike above the
// valley's rim nor pit below its lowest point -- the relief clamp), and two runs are bit-identical.
TEST_CASE(erosion_gpu_water_collects_in_the_valley_and_stays_bounded) {
    if (!erodeGpuAvailable()) { std::printf("    [skip] no GPU erosion available\n"); return; }
#if !defined(RT_HAVE_VULKAN_EROSION)
    std::printf("    [skip] the water model is Vulkan only\n");
    return;
#else
    auto valley = [] {
        Heightmap hm;
        hm.n = 129;
        hm.worldSize = 512.0f;   // 4 m cells
        hm.h.resize(static_cast<std::size_t>(hm.n) * hm.n);
        for (int z = 0; z < hm.n; ++z)
            for (int x = 0; x < hm.n; ++x)
                hm.set(x, z, 0.35f * std::fabs(static_cast<float>(x - 64)) * 4.0f + 0.05f * static_cast<float>(hm.n - 1 - z) * 4.0f);
        return hm;
    };
    ErosionParams p;
    p.thermalIterations = 0;
    p.vulkan = true;
    p.waterSteps = 3000;
    Heightmap a = valley(), b = valley();
    const Heightmap before = valley();
    setenv("RT_EROSION_DUMP", "/tmp", 1);   // the maps, to read the water depth back
    CHECK(erodeGpu(a, p));
    unsetenv("RT_EROSION_DUMP");
    CHECK(erodeGpu(b, p));
    CHECK(a.h == b.h);   // deterministic
    float lo = 1e30f, hi = -1e30f, blo = 1e30f, bhi = -1e30f;
    for (std::size_t i = 0; i < a.h.size(); ++i) {
        CHECK(std::isfinite(a.h[i]));
        lo = std::min(lo, a.h[i]); hi = std::max(hi, a.h[i]);
        blo = std::min(blo, before.h[i]); bhi = std::max(bhi, before.h[i]);
    }
    CHECK(hi <= bhi + 1e-3f && lo >= blo - 1e-3f);   // no spike above the rim, no pit below the floor
    // the water stands on the floor, not the slopes
    std::FILE* f = std::fopen("/tmp/water.f32", "rb");
    CHECK(f != nullptr);
    if (!f) return;
    std::vector<float> w(a.h.size());
    CHECK(std::fread(w.data(), sizeof(float), w.size(), f) == w.size());
    std::fclose(f);
    // the floor is a BAND (|x - 64| <= 4): the flow may split round a bar on the V's one-cell bottom (the
    // central-difference slope there sees only the valley's fall -- ADR-0124), and the water still stands
    // at the bottom, not on the slopes
    double floor = 0.0, slope = 0.0;
    for (int z = 16; z < a.n - 16; ++z) {
        for (int x = 60; x <= 68; ++x) floor += w[static_cast<std::size_t>(z) * a.n + x] / 9.0;
        slope += w[static_cast<std::size_t>(z) * a.n + 32];
    }
    std::printf("    [water] mean depth at the bottom %.3f m, on the slope %.3f m; heights %.1f..%.1f (was %.1f..%.1f)\n",
                floor / (a.n - 32), slope / (a.n - 32), lo, hi, blo, bhi);
    CHECK(floor > 5.0 * slope);
#endif
}

#include "../src/engine/procgen/stream_power.h"

// MOUNTAINS FROM UPLIFT (ADR-0125): a disc of uplift on a flat, rough plain grows a drained relief -- every
// height finite, the land mapped onto the target range, and hardly a pit left (the flood routes every cell
// to the edge; the implicit sweep keeps each cell above its receiver).
TEST_CASE(stream_power_grows_a_drained_relief_from_uplift) {
    Heightmap hm;
    hm.n = 129;
    hm.worldSize = 5000.0f;
    hm.h.resize(static_cast<std::size_t>(hm.n) * hm.n);
    std::vector<float> up(hm.h.size());
    for (int z = 0; z < hm.n; ++z)
        for (int x = 0; x < hm.n; ++x) {
            const float dx = static_cast<float>(x - 64), dz = static_cast<float>(z - 64);
            const std::size_t i = static_cast<std::size_t>(z) * hm.n + x;
            up[i] = std::max(0.0f, 1.0f - std::sqrt(dx * dx + dz * dz) / 60.0f);
            hm.h[i] = 1.0f + 0.3f * static_cast<float>((x * 7919 + z * 104729) % 97) / 97.0f;   // rough start
        }
    StreamPowerParams p;
    p.iterations = 120;
    p.targetLo = 0.0; p.targetHi = 800.0;
    CHECK(streamPowerErode(hm, up, p) == 120);
    float lo = 1e30f, hi = -1e30f;
    int pits = 0, land = 0;
    for (int z = 1; z < hm.n - 1; ++z)
        for (int x = 1; x < hm.n - 1; ++x) {
            const float v = hm.get(x, z);
            CHECK(std::isfinite(v));
            lo = std::min(lo, v); hi = std::max(hi, v);
            float low = 1e30f;
            for (int k = -1; k <= 1; ++k) for (int j = -1; j <= 1; ++j) if (k || j) low = std::min(low, hm.get(x + j, z + k));
            ++land;
            if (low > v) ++pits;
        }
    std::printf("    [stream power] relief %.0f..%.0f m, %d pits of %d cells\n", lo, hi, pits, land);
    CHECK(hi > 700.0f && lo < 100.0f);
    CHECK(pits < land / 100);
}

// STOCHASTIC ROUTING (ADR-0206): on an even slope falling 15 deg off the grid's x axis, every cell's steepest neighbour
// is the same (+x), so steepest-descent flow lines all run along the grid, 15 deg off the true fall -- the parallel
// channels the island's 8192 ground cut. Drawn in proportion to slope, the lines wander but keep to the fall on
// average; and the same seed gives the same lines.
TEST_CASE(stochastic_routing_follows_the_true_fall_not_the_grid) {
    const double a = 15.0 * 3.14159265358979 / 180.0;
    auto run = [&](bool stochastic, std::vector<int32_t>& rec) {
        Heightmap hm;
        hm.n = 161;
        hm.worldSize = 1600.0f;
        hm.h.resize(static_cast<std::size_t>(hm.n) * hm.n);
        for (int z = 0; z < hm.n; ++z)
            for (int x = 0; x < hm.n; ++x)
                hm.h[static_cast<std::size_t>(z) * hm.n + x] = static_cast<float>(400.0 - 0.3 * 10.0 * (x * std::cos(a) + z * std::sin(a)));
        StreamPowerParams p;
        p.iterations = 1;
        p.K = 0.0;
        p.diffusion = 0.0;
        p.uplift = 0.0;
        p.rescale = false;
        p.stochastic = stochastic;
        p.seed = 7;
        streamPowerErode(hm, std::vector<float>(hm.h.size(), 0.0f), p, nullptr, nullptr, &rec);
        // forty flow lines from the up-slope side, 80 steps each: their mean heading
        double sx = 0, sz = 0;
        for (int k = 0; k < 40; ++k) {
            int c = (20 + 3 * k) * hm.n + 10;
            const int x0 = c % hm.n, z0 = c / hm.n;
            for (int s = 0; s < 80 && rec[static_cast<std::size_t>(c)] >= 0; ++s) c = rec[static_cast<std::size_t>(c)];
            sx += c % hm.n - x0;
            sz += c / hm.n - z0;
        }
        return std::atan2(sz, sx) * 180.0 / 3.14159265358979;
    };
    std::vector<int32_t> r0, r1, r2;
    const double steepest = run(false, r0), drawn = run(true, r1);
    run(true, r2);
    std::printf("    [stochastic routing] flow heading %.1f deg steepest, %.1f deg drawn (the fall: 15)\n", steepest, drawn);
    CHECK(std::fabs(steepest) < 1.0);
    CHECK(std::fabs(drawn - 15.0) < 4.0);
    CHECK(r1 == r2);
}

// ROCK LAYERS (ADR-0209): a relief grown with hard and soft beds stands steeper on the hard ones -- the erosion,
// slower there, leaves them as walls and ledges -- and the band tone spans dark to pale.
TEST_CASE(rock_layers_grow_steeper_ground_on_hard_beds) {
    RockLayersParams lp;
    lp.thicknessM = 60.0;
    lp.dipDeg = 0.0;
    lp.lateralScaleM = 1e6;   // (no patches along the outcrop: every hard bed hard everywhere)
    lp.hard = 0.5;
    lp.hardK = 0.05;
    lp.hardDiffusion = 0.1;
    const RockLayers layers(lp, 5);
    Heightmap hm;
    hm.n = 257;   // (19.5 m cells, as the island's grow: a bed is three of them)
    hm.worldSize = 5000.0f;
    hm.h.resize(static_cast<std::size_t>(hm.n) * hm.n);
    std::vector<float> up(hm.h.size());
    for (int z = 0; z < hm.n; ++z)
        for (int x = 0; x < hm.n; ++x) {
            const float dx = static_cast<float>(x - 128), dz = static_cast<float>(z - 128);
            const std::size_t i = static_cast<std::size_t>(z) * hm.n + x;
            up[i] = std::max(0.0f, 1.0f - std::sqrt(dx * dx + dz * dz) / 120.0f);
            hm.h[i] = 1.0f + 0.3f * static_cast<float>((x * 7919 + z * 104729) % 97) / 97.0f;
        }
    StreamPowerParams p;
    p.iterations = 160;
    p.rescale = false;
    p.layers = &layers;
    streamPowerErode(hm, up, p);
    const double cell = hm.worldSize / (hm.n - 1);
    double sHard = 0, sSoft = 0, tLo = 1, tHi = 0;
    int nHard = 0, nSoft = 0;
    for (int z = 2; z < hm.n - 2; ++z)
        for (int x = 2; x < hm.n - 2; ++x) {
            const double h = hm.get(x, z);
            if (up[static_cast<std::size_t>(z) * hm.n + x] < 0.3) continue;
            const double gx = (hm.get(x + 1, z) - hm.get(x - 1, z)) / (2 * cell), gz = (hm.get(x, z + 1) - hm.get(x, z - 1)) / (2 * cell);
            const double s = std::sqrt(gx * gx + gz * gz), k = layers.hardness(x * cell, z * cell, h);
            if (k > 0.5) { sHard += s; ++nHard; }
            if (k < 0.1) { sSoft += s; ++nSoft; }
            const double t = layers.tone(x * cell, z * cell, h);
            tLo = std::min(tLo, t); tHi = std::max(tHi, t);
        }
    float hlo = 1e30f, hhi = -1e30f;
    for (float v : hm.h) { hlo = std::min(hlo, v); hhi = std::max(hhi, v); }
    std::printf("    [rock layers] relief %.0f..%.0f m; mean slope %.3f on hard rock (%d cells), %.3f on soft (%d); tone %.2f..%.2f\n", hlo, hhi,
                sHard / std::max(1, nHard), nHard, sSoft / std::max(1, nSoft), nSoft, tLo, tHi);
    CHECK(nHard > 100 && nSoft > 100);
    CHECK(sHard / nHard > 1.2 * sSoft / nSoft);
    CHECK(tLo < 0.1 && tHi > 0.9);
}

// (review) The weathered ground's one content key: what only paints or stands on the ground leaves it alone;
// what shapes the ground changes it.
TEST_CASE(weathered_ground_key_folds_what_shapes_the_ground_and_nothing_else) {
    nlohmann::json tb = {{"seed", 8}, {"size", 2000.0}, {"resolution", 64}, {"seaLevel", 0.0},
                         {"weather", {{"res", 256}, {"grow", {{"res", 64}, {"iterations", 10}}}}}};
    const std::string k0 = engine::weatheredGroundKey(tb);
    nlohmann::json paint = tb;
    paint["groundCover"] = {{"rockSlopeDeg", 50}};
    paint["forest"] = {{"coverage", 0.3}};
    paint["trails"] = nlohmann::json::array({nlohmann::json::array({{0, 0}, {10, 10}})});
    CHECK(engine::weatheredGroundKey(paint) == k0);
    nlohmann::json shape = tb;
    shape["weather"]["roughness"] = 3.0;
    CHECK(engine::weatheredGroundKey(shape) != k0);
    nlohmann::json seed = tb;
    seed["seed"] = 9;
    CHECK(engine::weatheredGroundKey(seed) != k0);
}
