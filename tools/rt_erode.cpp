// rt_erode -- bake a level's terrain through the erosion stages and write the grids, for looking at
// the weathering before it goes into a level (ADR-0123).
//
//   rt_erode <level.json> <out-dir> [--res N] [--droplets N] [--water-steps N] [--rain m/s]
//            [--capacity Kc] [--dissolve Ks] [--deposit Kd] [--evaporate Ke] [--thermal N] [--cpu]
//
// Writes <out>/before.f32 and <out>/after.f32 (float32, N+1 x N+1, row-major from (-size/2, -size/2))
// and, with water steps, water.f32 / wet.f32 / sed.f32 (the water model's depth, wetness and suspended
// sediment), plus <out>/meta.json. tools/erosion_preview.py turns them into pictures. Run from the repo
// root (the kernels load from the build's shader dir; the level's paths are repo-relative).
#include "engine/procgen/erosion.h"
#include "engine/procgen/stream_power.h"
#include "engine/procgen/noise.h"
#include "engine/procgen/terrain.h"
#include "engine/level_params.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

using json = nlohmann::json;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: rt_erode <level.json> <out-dir> [--res N] [--droplets N] [--water-steps N] [--rain m/s]\n"
                             "                [--capacity Kc] [--dissolve Ks] [--deposit Kd] [--evaporate Ke] [--thermal N] [--cpu]\n");
        return 2;
    }
    json level;
    { std::ifstream in(argv[1]); if (!in) { std::fprintf(stderr, "cannot read %s\n", argv[1]); return 1; } in >> level; }
    if (!level.contains("terrain")) { std::fprintf(stderr, "%s has no terrain block\n", argv[1]); return 1; }
    const std::string out = argv[2];
    std::filesystem::create_directories(out);

    json tj = level["terrain"];
    tj.erase("rivers");   // the rivers are computed on the eroded ground; here only the ground
    tj.erase("erode");
    engine::TerrainParams tp = engine::readTerrainParams(tj);
    engine::ErosionParams ep;
    ep.seed = tj.value("seed", 0u) + 1234u;
    ep.droplets = tj.value("erodeDroplets", ep.droplets);
    ep.erodeRadius = tj.value("erodeRadius", ep.erodeRadius);
    ep.thermalIterations = tj.value("erodeThermal", ep.thermalIterations);
    ep.talus = tj.value("erodeTalus", ep.talus);
    ep.vulkan = true;
    ep.seaLevel = tp.seaLevel > -1e29 ? static_cast<float>(tp.seaLevel) : -1e30f;
    int res = tj.value("erodeRes", 512);
    // a droplet's life in METRES, not cells: 32 cells is 640 m at the island's 20 m but 94 m at 3 m, where a
    // droplet dies before it finds a channel and only pits the ground (ADR-0124)
    double lifetimeM = 0.0;
    // COARSE TO FINE (ADR-0124): --coarse-res C runs the droplets + thermal at C (their constants are tuned
    // in grid units for ~10-20 m cells; at 3 m they only pit the ground), adds that erosion's change to the
    // fine grid (bilinear), then runs breaching + water at the fine resolution
    int coarseRes = 0;
    // MOUNTAINS FROM UPLIFT (ADR-0125): --stream-power N grows the coarse grid for N iterations under an
    // uplift map made from the level's own terrain, blurred over --sp-blur metres (where the ranges are,
    // none of the noise); the fine grid is its upsample
    int spIters = 0;
    double spBlurM = 150.0;
    engine::StreamPowerParams sp;
    for (int a = 3; a < argc; ++a) {
        const std::string k = argv[a];
        auto next = [&](double& v) { if (a + 1 < argc) v = std::atof(argv[++a]); };
        double v = 0.0;
        if (k == "--res") { next(v); res = static_cast<int>(v); }
        else if (k == "--droplets") { next(v); ep.droplets = static_cast<int>(v); }
        else if (k == "--water-steps") { next(v); ep.waterSteps = static_cast<int>(v); }
        else if (k == "--rain") { next(v); ep.waterRain = static_cast<float>(v); }
        else if (k == "--capacity") { next(v); ep.waterCapacity = static_cast<float>(v); }
        else if (k == "--dissolve") { next(v); ep.waterDissolve = static_cast<float>(v); }
        else if (k == "--deposit") { next(v); ep.waterDeposit = static_cast<float>(v); }
        else if (k == "--evaporate") { next(v); ep.waterEvaporate = static_cast<float>(v); }
        else if (k == "--thermal") { next(v); ep.thermalIterations = static_cast<int>(v); }
        else if (k == "--rock-hardness") { next(v); ep.waterRockHardness = static_cast<float>(v); }
        else if (k == "--deposit-rate") { next(v); ep.waterDeposit = static_cast<float>(v); }
        else if (k == "--lifetime-m") { next(v); lifetimeM = v; }
        else if (k == "--breach") { next(v); ep.breachDepth = static_cast<float>(v); }
        else if (k == "--coarse-res") { next(v); coarseRes = static_cast<int>(v); }
        else if (k == "--stream-power") { next(v); spIters = static_cast<int>(v); }
        else if (k == "--sp-k") { next(v); sp.K = v; }
        else if (k == "--sp-diffusion") { next(v); sp.diffusion = v; }
        else if (k == "--sp-blur") { next(v); spBlurM = v; }
        else if (k == "--cpu") ep.vulkan = false;
    }

    tp.resolution = res;
    if (lifetimeM > 0.0) ep.maxLifetime = std::max(8, static_cast<int>(lifetimeM / (tp.size / res)));
    const engine::Noise noise(tj.value("seed", 0u));
    auto t0 = std::chrono::steady_clock::now();
    engine::Heightmap hm = engine::bakeHeightmap(tp, noise);
    auto write = [&](const engine::Heightmap& h, const char* name) {
        std::ofstream(out + "/" + name, std::ios::binary).write(reinterpret_cast<const char*>(h.h.data()),
                                                                static_cast<std::streamsize>(h.h.size() * sizeof(float)));
    };
    write(hm, "before.f32");
    setenv("RT_EROSION_DUMP", out.c_str(), 1);
    auto t1 = std::chrono::steady_clock::now();
    if (coarseRes > 0 && coarseRes < res) {
        engine::TerrainParams ctp = tp;
        ctp.resolution = coarseRes;
        engine::Heightmap coarse = engine::bakeHeightmap(ctp, noise);
        const engine::Heightmap coarse0 = coarse;
        if (spIters > 0) {
            // the uplift: the terrain blurred (box, 3 passes) and normalised 0..1 over the land
            const int cn = coarse.n;
            const int r = std::max(1, static_cast<int>(spBlurM / (tp.size / coarseRes)));
            std::vector<float> u(coarse.h.begin(), coarse.h.end()), tmp(u.size());
            for (int pass = 0; pass < 3; ++pass)
                for (int axis = 0; axis < 2; ++axis) {
                    for (int z = 0; z < cn; ++z)
                        for (int x = 0; x < cn; ++x) {
                            double s = 0.0; int c = 0;
                            for (int k = -r; k <= r; ++k) {
                                const int xx = axis == 0 ? std::clamp(x + k, 0, cn - 1) : x, zz = axis == 1 ? std::clamp(z + k, 0, cn - 1) : z;
                                s += u[static_cast<std::size_t>(zz) * cn + xx]; ++c;
                            }
                            tmp[static_cast<std::size_t>(z) * cn + x] = static_cast<float>(s / c);
                        }
                    u.swap(tmp);
                }
            const float sea = ep.seaLevel > -1e29f ? ep.seaLevel : *std::min_element(u.begin(), u.end());
            float hiU = sea;
            for (float v2 : u) hiU = std::max(hiU, v2);
            // THE UPLIFT: where the level's ranges are, over a base that lifts all the land a little (the
            // lowlands get hills, not a flat), broken by two scales of noise -- a smooth, symmetric uplift grew
            // spurs as straight and parallel as a fishbone. The START: low, and ROUGH (a smooth start drains
            // straight downhill everywhere). ERODIBILITY: soft and hard rock in patches, so valleys differ.
            const engine::Noise un(tj.value("seed", 0u) * 7919u + 17u);
            const double cellM = tp.size / coarseRes;
            std::vector<float> upl(u.size()), erod(u.size());
            for (int z = 0; z < cn; ++z)
                for (int x = 0; x < cn; ++x) {
                    const std::size_t i = static_cast<std::size_t>(z) * cn + x;
                    const double wx = x * cellM, wz = z * cellM;
                    const float base = std::clamp((u[i] - sea) / std::max(1e-3f, hiU - sea), 0.0f, 1.0f);
                    const double big = un.fbm2(wx * 0.00035 + 3.1, wz * 0.00035 - 7.2, 3), mid = un.fbm2(wx * 0.0015 - 1.7, wz * 0.0015 + 4.4, 3);
                    upl[i] = coarse0.h[i] < sea ? 0.0f : std::clamp(static_cast<float>((0.12 + 0.88 * base) * (1.0 + 0.45 * big + 0.2 * mid)), 0.0f, 1.5f);
                    erod[i] = static_cast<float>(std::exp(0.6 * un.fbm2(wx * 0.0008 + 11.0, wz * 0.0008 - 2.0, 4)));
                    coarse.h[i] = coarse0.h[i] < sea ? coarse0.h[i]
                                                     : static_cast<float>(sea + 0.1 * (u[i] - sea) + 0.5 + 6.0 * (0.5 + 0.5 * un.fbm2(wx * 0.01, wz * 0.01, 4)));
                }
            sp.iterations = spIters;
            {   // the land's height range as the level drew it: the grown relief is mapped back onto it
                float lo = 1e30f, hi2 = -1e30f;
                for (float v2 : coarse0.h) if (v2 >= sea) { lo = std::min(lo, v2); hi2 = std::max(hi2, v2); }
                sp.targetLo = lo; sp.targetHi = hi2;
            }
            sp.seaLevel = ep.seaLevel;
            const auto s0 = std::chrono::steady_clock::now();
            engine::streamPowerErode(coarse, upl, sp, &erod);
            std::printf("stream power: %d iterations at %d^2 in %.1f s\n", spIters, cn,
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - s0).count());
            // the fine grid IS the grown one -- CUBIC (Catmull-Rom), not bilinear: bilinear left every slope a
            // perfect plane, and rain on a plane cut parallel one-cell rills along the grid -- plus a few metres of
            // multi-scale roughness on land so water gathers into gullies as it does on real ground
            auto cr = [](double p0, double p1, double p2, double p3, double t) {
                return p1 + 0.5 * t * (p2 - p0 + t * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 + t * (3.0 * (p1 - p2) + p3 - p0)));
            };
            const engine::Noise rough(tj.value("seed", 0u) * 104729u + 5u);
            const double fineCell = tp.size / res;
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
                    if (hv > sea + 0.5) hv += 2.5 * rough.fbm2(x * fineCell * 0.02 + 0.3, z * fineCell * 0.02 - 0.7, 4);
                    hm.set(x, z, static_cast<float>(hv));
                }
            write(hm, "grown.f32");
            ep.droplets = 0;
        } else {
        engine::ErosionParams cep = ep;
        cep.waterSteps = 0;
        cep.breachDepth = 0.0f;
        cep.maxLifetime = engine::ErosionParams{}.maxLifetime;
        engine::erode(coarse, cep);
        // the coarse erosion's change, bilinear onto the fine grid
        const int cn = coarse.n, fn = hm.n;
        for (int z = 0; z < fn; ++z)
            for (int x = 0; x < fn; ++x) {
                const double gx = static_cast<double>(x) * (cn - 1) / (fn - 1), gz = static_cast<double>(z) * (cn - 1) / (fn - 1);
                const int x0 = std::min(static_cast<int>(gx), cn - 2), z0 = std::min(static_cast<int>(gz), cn - 2);
                const double tx = gx - x0, tz = gz - z0;
                auto dAt = [&](int xx, int zz) { return static_cast<double>(coarse.get(xx, zz) - coarse0.get(xx, zz)); };
                const double d = (dAt(x0, z0) * (1 - tx) + dAt(x0 + 1, z0) * tx) * (1 - tz) + (dAt(x0, z0 + 1) * (1 - tx) + dAt(x0 + 1, z0 + 1) * tx) * tz;
                hm.set(x, z, hm.get(x, z) + static_cast<float>(d));
            }
        ep.droplets = 0;   // the fine pass is water only
        std::printf("coarse: droplets + thermal at %d^2 (%.1f m cells)\n", cn, tp.size / coarseRes);
        }
    }
    engine::erode(hm, ep);
    auto t2 = std::chrono::steady_clock::now();
    write(hm, "after.f32");
    const double bakeS = std::chrono::duration<double>(t1 - t0).count(), erodeS = std::chrono::duration<double>(t2 - t1).count();
    json meta = {{"n", hm.n}, {"size", hm.worldSize}, {"seaLevel", ep.seaLevel}, {"backend", engine::erosionBackendTag(&ep)},
                 {"droplets", ep.droplets}, {"waterSteps", ep.waterSteps}, {"bakeSeconds", bakeS}, {"erodeSeconds", erodeS}};
    std::ofstream(out + "/meta.json") << meta.dump(1) << "\n";
    std::printf("%s: %d^2 over %.0f m (%.2f m cells), %s, %d droplets, %d water steps: bake %.1f s, erode %.1f s\n",
                argv[1], hm.n, hm.worldSize, hm.worldSize / (hm.n - 1), engine::erosionBackendTag(&ep), ep.droplets, ep.waterSteps, bakeS, erodeS);
    return 0;
}
