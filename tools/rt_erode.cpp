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
#include "engine/procgen/terrain_weather.h"
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
    bool weathered = false;
    double shapeKeep = 1.0;
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
        else if (k == "--shape-keep") { next(v); shapeKeep = v; }
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
            // the engine's pipeline (ADR-0126): grow, cubic onto the fine grid with roughness, then water
            engine::WeatherParams wp;
            wp.res = res;
            wp.growRes = coarseRes;
            wp.growIterations = spIters;
            wp.growBlurM = spBlurM;
            wp.grow = sp;
            wp.water = ep;
            wp.water.droplets = 0;
            wp.shapeKeep = shapeKeep;
            engine::Heightmap grown;
            hm = engine::weatherTerrain(tp, tj.value("seed", 0u), wp, &grown);
            write(grown, "grown.f32");
            ep.waterSteps = 0;   // done
            ep.droplets = 0;
            ep.thermalIterations = 0;
            weathered = true;
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
    if (!weathered) engine::erode(hm, ep);
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
