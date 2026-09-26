#include "test_framework.h"

#include "../src/engine/procgen/erosion.h"
#include "../src/engine/procgen/erosion_gpu.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>

using namespace engine;

namespace {

// erodeGpu compiles its kernels from shaders/metal/erosion.metal relative to
// CWD (the renderer's shader convention); ctest runs from the build dir, so
// hop to the source tree for the GPU cases and hop back after.
struct ScopedSourceDir {
    std::filesystem::path old;
    ScopedSourceDir() {
#ifdef RT_SOURCE_DIR
        std::error_code ec;
        old = std::filesystem::current_path(ec);
        std::filesystem::current_path(RT_SOURCE_DIR, ec);
#endif
    }
    ~ScopedSourceDir() {
#ifdef RT_SOURCE_DIR
        std::error_code ec;
        if (!old.empty()) std::filesystem::current_path(old, ec);
#endif
    }
};

// Synthetic rolling hills with a regional slope: per-cell gradients up to
// ~2.4 height units exceed the default talus (1.2) and give droplets real
// downhill runs, so both erosion phases have work to do.
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
    p.droplets = 50000;
    p.seed = 1234;
    return p;   // everything else: the shipped defaults
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

    ScopedSourceDir cd;
    if (!erodeGpuAvailable()) {
        // Graceful skip: no Metal device (or no kernel source) — erodeGpu
        // must decline so erode() runs the CPU sim, and the cache tag must
        // say so.
        Heightmap hm = makeHills(32);
        ErosionParams p = testParams();
        p.droplets = 100;
        CHECK(!erodeGpu(hm, p));
        CHECK(std::strcmp(erosionBackendTag(), "cpu") == 0);
        std::printf("    [skip] no Metal GPU erosion available\n");
        return;
    }
    ErosionParams asks = testParams();
    asks.vulkan = true;   // a Vulkan build runs on the GPU only when the terrain asks (ADR-0122)
    CHECK(std::strcmp(erosionBackendTag(&asks), "gpu-erosion-v1") == 0 ||   // Metal
          std::strcmp(erosionBackendTag(&asks), "vk-erosion-v1") == 0);
#if defined(RT_HAVE_VULKAN_EROSION)
    const ErosionParams notAsked = testParams();   // every level baked before ADR-0122 keeps the CPU sim
    CHECK(std::strcmp(erosionBackendTag(&notAsked), "cpu") == 0);
#endif
}

TEST_CASE(erosion_gpu_two_runs_bit_identical) {
    ScopedSourceDir cd;
    if (!erodeGpuAvailable()) {
        std::printf("    [skip] no Metal GPU erosion available\n");
        return;
    }
    // Run-to-run determinism is the load-bearing contract: fixed batches,
    // snapshot reads, and fixed-point integer atomics make the result
    // independent of GPU thread scheduling — two runs must match to the bit.
    ErosionParams p = testParams();
    Heightmap a = makeHills(256), b = makeHills(256);
    CHECK(erodeGpu(a, p));
    CHECK(erodeGpu(b, p));
    CHECK(a.h.size() == b.h.size());
    CHECK(std::memcmp(a.h.data(), b.h.data(),
                      a.h.size() * sizeof(float)) == 0);
}

TEST_CASE(erosion_gpu_statistically_matches_cpu) {
    ScopedSourceDir cd;
    if (!erodeGpuAvailable()) {
        std::printf("    [skip] no Metal GPU erosion available\n");
        return;
    }
    ErosionParams p = testParams();
    Heightmap input = makeHills(256);
    Heightmap gpu = input, cpu = input, cpuB = input;
    CHECK(erodeGpu(gpu, p));
    runCpu(cpu, p);
    // A second CPU run with a different seed is the yardstick for every
    // envelope below: the backends use different RNG streams by design
    // (mt19937 vs counter hash), so "statistical parity" means GPU-vs-CPU
    // differences of the same order as the CPU's own seed-to-seed noise.
    ErosionParams pB = p;
    pB.seed = 999;
    runCpu(cpuB, pB);

    Stats si = stats(input), sg = stats(gpu), sc = stats(cpu);
    // Both backends actually eroded: droplets shave peaks and fill hollows,
    // thermal slumps over-steep slopes — total height variance must drop.
    CHECK(sg.var < si.var);
    CHECK(sc.var < si.var);

    double rmsGC = rmsDiff(gpu, cpu);       // cross-backend disagreement
    double rmsCI = rmsDiff(cpu, input);     // how much the CPU sim carved
    double rmsGI = rmsDiff(gpu, input);     // how much the GPU sim carved
    double rmsCC = rmsDiff(cpu, cpuB);      // CPU seed-to-seed noise
    std::printf(
        "    gpu-vs-cpu rms %.4f (cpu seed-noise %.4f) | carve rms cpu %.4f "
        "gpu %.4f | mean %.4f/%.4f min %.3f/%.3f max %.3f/%.3f (gpu/cpu)\n",
        rmsGC, rmsCC, rmsCI, rmsGI, sg.mean, sc.mean, sg.mn, sc.mn, sg.mx,
        sc.mx);

    // Same amount of carving (measured 0.96x, seed-to-seed 0.99x)...
    CHECK(rmsGI > 0.7 * rmsCI);
    CHECK(rmsGI < 1.4 * rmsCI);
    // ...and pointwise disagreement on the order of seed noise (measured
    // 1.34x), with an absolute ceiling well under the ~8-unit carve signal
    // itself (8 percent of the ~81-unit height range) as a backstop against
    // a silently diverged model.
    CHECK(rmsGC < 2.5 * rmsCC);
    CHECK(rmsGC < 0.08 * (si.mx - si.mn));

    // Moments: mean within ~1 percent of the height range (the GPU loses
    // slightly more in-flight sediment when droplets die); max within the
    // seed-noise scale (the relief clamp additionally guarantees the GPU
    // never deposits above the local rim, so max cannot grow).
    CHECK(std::fabs(sg.mean - sc.mean) < 1.0);
    CHECK(std::fabs(sg.mx - sc.mx) < 2.0);
    // Min is an extreme-value statistic driven by the CPU's runaway gorge
    // incision (its min swings -71 vs -94 across seeds on this input). The
    // GPU's per-batch relief clamp bounds that singular deepening by design,
    // so the gate is one-sided: the GPU must never out-dig the CPU, and may
    // raise the global floor only by bounded pit-filling.
    CHECK(sg.mn >= sc.mn - 5.0);
    CHECK(sg.mn <= si.mn + 6.0);
}

TEST_CASE(erosion_gpu_speedup_bench) {
    // Opt-in (~seconds of CPU sim): RT_EROSION_BENCH=1 ./run_tests
    if (!std::getenv("RT_EROSION_BENCH")) return;
    ScopedSourceDir cd;
    if (!erodeGpuAvailable()) {
        std::printf("    [skip] no Metal GPU erosion available\n");
        return;
    }
    ErosionParams p;
    p.droplets = 500000;
    p.seed = 7;
    Heightmap gpu = makeHills(1024), cpu = makeHills(1024);
    auto t0 = std::chrono::steady_clock::now();
    CHECK(erodeGpu(gpu, p));
    auto t1 = std::chrono::steady_clock::now();
    runCpu(cpu, p);
    auto t2 = std::chrono::steady_clock::now();
    double gpuMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double cpuMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::printf("    res 1024, 500k droplets: gpu %.0f ms, cpu %.0f ms — %.1fx\n",
                gpuMs, cpuMs, cpuMs / gpuMs);
    CHECK(gpuMs < cpuMs);
}

// THE WATER MODEL (ADR-0123, Mei et al. 2007 pipe model): on a V valley tilted down its length, rain runs
// off the slopes and collects along the floor; nothing blows up (every height finite, no spike above the
// valley's rim nor pit below its lowest point -- the relief clamp), and two runs are bit-identical.
TEST_CASE(erosion_gpu_water_collects_in_the_valley_and_stays_bounded) {
    ScopedSourceDir cd;
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
    p.droplets = 0;
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
