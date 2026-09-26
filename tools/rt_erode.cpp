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
#include "engine/procgen/noise.h"
#include "engine/procgen/terrain.h"
#include "engine/level_params.h"

#include <nlohmann/json.hpp>

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
        else if (k == "--cpu") ep.vulkan = false;
    }

    tp.resolution = res;
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
