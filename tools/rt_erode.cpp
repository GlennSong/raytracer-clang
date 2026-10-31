// rt_erode -- bake a level's weathered ground (its terrain "weather" block, ADR-0126) and write the grids,
// for looking at the weathering before it goes into a level (ADR-0123).
//
//   rt_erode <level.json> <out-dir> [--res N] [--grow-res N] [--grow-iterations N] [--sp-k K]
//            [--sp-diffusion D] [--sp-blur M] [--shape-keep F] [--water-steps N] [--rain m/s]
//            [--capacity Kc] [--dissolve Ks] [--deposit Kd] [--evaporate Ke] [--rock-hardness H]
//            [--breach M] [--thermal N] [--cpu]
//
// The flags override the level's block. Writes <out>/before.f32 (the drawn relief), grown.f32 (the ground
// before the water stage) and after.f32 (float32, N+1 x N+1, row-major from (-size/2, -size/2)) and, with
// water steps, water.f32 / wet.f32 / sed.f32 (the water model's depth, wetness and suspended sediment),
// plus <out>/meta.json. tools/erosion_preview.py turns them into pictures. Run from the repo root (the
// kernels load from the build's shader dir; the level's paths are repo-relative).
#include "engine/procgen/erosion.h"
#include "engine/procgen/terrain_weather.h"
#include "engine/procgen/noise.h"
#include "engine/procgen/terrain.h"
#include "engine/level_params.h"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

using json = nlohmann::json;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: rt_erode <level.json> <out-dir> [--res N] [--grow-res N] [--grow-iterations N] [--sp-k K]\n"
                             "                [--sp-diffusion D] [--sp-blur M] [--shape-keep F] [--water-steps N] [--rain m/s]\n"
                             "                [--capacity Kc] [--dissolve Ks] [--deposit Kd] [--evaporate Ke] [--rock-hardness H]\n"
                             "                [--breach M] [--thermal N] [--cpu]\n");
        return 2;
    }
    json level;
    { std::ifstream in(argv[1]); if (!in) { std::fprintf(stderr, "cannot read %s\n", argv[1]); return 1; } in >> level; }
    if (!level.contains("terrain")) { std::fprintf(stderr, "%s has no terrain block\n", argv[1]); return 1; }
    if (!level["terrain"].contains("weather")) {
        std::fprintf(stderr, "%s: the terrain has no \"weather\" block (the only erosion since ADR-0257)\n", argv[1]);
        return 1;
    }
    engine::propagateWaterSeaLevel(level);   // the sea the bake holds, as the level loader sets it
    const std::string out = argv[2];
    std::filesystem::create_directories(out);

    json tj = level["terrain"];
    tj.erase("rivers");   // the rivers are computed on the weathered ground; here only the ground
    tj.erase("erode");
    const engine::TerrainParams tp = engine::readTerrainParams(tj);
    engine::WeatherParams wp = engine::weatherFromJson(tj["weather"], tj);
    for (int a = 3; a < argc; ++a) {
        const std::string k = argv[a];
        auto next = [&]() { return a + 1 < argc ? std::atof(argv[++a]) : 0.0; };
        engine::ErosionParams& e = wp.water;
        if (k == "--res") wp.res = static_cast<int>(next());
        else if (k == "--grow-res") wp.growRes = static_cast<int>(next());
        else if (k == "--grow-iterations") wp.growIterations = static_cast<int>(next());
        else if (k == "--sp-k") wp.grow.K = next();
        else if (k == "--sp-diffusion") wp.grow.diffusion = next();
        else if (k == "--sp-blur") wp.growBlurM = next();
        else if (k == "--shape-keep") wp.shapeKeep = next();
        else if (k == "--water-steps") e.waterSteps = static_cast<int>(next());
        else if (k == "--rain") e.waterRain = static_cast<float>(next());
        else if (k == "--capacity") e.waterCapacity = static_cast<float>(next());
        else if (k == "--dissolve") e.waterDissolve = static_cast<float>(next());
        else if (k == "--deposit") e.waterDeposit = static_cast<float>(next());
        else if (k == "--evaporate") e.waterEvaporate = static_cast<float>(next());
        else if (k == "--rock-hardness") e.waterRockHardness = static_cast<float>(next());
        else if (k == "--breach") e.breachDepth = static_cast<float>(next());
        else if (k == "--thermal") e.thermalIterations = static_cast<int>(next());
        else if (k == "--cpu") e.vulkan = false;
        else { std::fprintf(stderr, "unknown option %s\n", k.c_str()); return 2; }
    }

    const uint32_t seed = tj.value("seed", 0u);
    auto t0 = std::chrono::steady_clock::now();
    engine::TerrainParams fine = tp;
    fine.resolution = wp.res;
    const engine::Heightmap before = engine::bakeHeightmap(fine, engine::Noise(seed));
    auto write = [&](const engine::Heightmap& h, const char* name) {
        std::ofstream(out + "/" + name, std::ios::binary).write(reinterpret_cast<const char*>(h.h.data()),
                                                                static_cast<std::streamsize>(h.h.size() * sizeof(float)));
    };
    write(before, "before.f32");
    setenv("RT_EROSION_DUMP", out.c_str(), 1);
    auto t1 = std::chrono::steady_clock::now();
    engine::Heightmap grown;
    const engine::Heightmap hm = engine::weatherTerrain(tp, seed, wp, &grown);
    auto t2 = std::chrono::steady_clock::now();
    write(grown, "grown.f32");
    write(hm, "after.f32");
    const double bakeS = std::chrono::duration<double>(t1 - t0).count(), erodeS = std::chrono::duration<double>(t2 - t1).count();
    const float sea = tp.seaLevel > -1e29 ? static_cast<float>(tp.seaLevel) : -1e30f;
    json meta = {{"n", hm.n}, {"size", hm.worldSize}, {"seaLevel", sea}, {"backend", engine::erosionBackendTag(&wp.water)},
                 {"waterSteps", wp.water.waterSteps}, {"bakeSeconds", bakeS}, {"erodeSeconds", erodeS}};
    std::ofstream(out + "/meta.json") << meta.dump(1) << "\n";
    std::printf("%s: %d^2 over %.0f m (%.2f m cells), grown at %d^2 over %d iterations, %s, %d water steps: bake %.1f s, weather %.1f s\n",
                argv[1], hm.n, hm.worldSize, hm.worldSize / (hm.n - 1), wp.growRes, wp.growIterations,
                engine::erosionBackendTag(&wp.water), wp.water.waterSteps, bakeS, erodeS);
    return 0;
}
