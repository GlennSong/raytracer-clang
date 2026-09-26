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

namespace engine {

namespace {
// bump when the pipeline's output changes for the same inputs: every weathered cache rebakes
constexpr const char* kWeatherCodeTag = "2026-09-26.1";
}  // namespace

WeatherParams weatherFromJson(const nlohmann::json& w, const nlohmann::json& tj) {
    WeatherParams p;
    p.res = w.value("res", p.res);
    p.roughnessM = w.value("roughness", p.roughnessM);
    p.shapeKeep = w.value("shapeKeep", p.shapeKeep);
    p.shapeScaleM = w.value("shapeScale", p.shapeScaleM);
    p.plainHeightM = w.value("plainHeight", p.plainHeightM);
    p.reliefRampM = w.value("reliefRamp", p.reliefRampM);
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
            upl[i] = isSea ? 0.0f : std::clamp(static_cast<float>((0.12 + 0.88 * b) * (1.0 + 0.45 * big + 0.2 * mid)), 0.0f, 1.5f);
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
            const float kept = lowO[i] + (coarse.h[i] - lowG[i]) * (up * up * (3.0f - 2.0f * up));
            coarse.h[i] = static_cast<float>(coarse.h[i] + w.shapeKeep * (kept - coarse.h[i]));
            if (sea > -1e29f) coarse.h[i] = std::max(coarse.h[i], sea + 0.5f);   // land stays land
        }
    }

    // 2. THE FINE GRID: cubic from the grown one, plus metres of multi-scale roughness on land
    Heightmap hm;
    hm.n = w.res + 1;
    hm.worldSize = static_cast<float>(tp.size);
    hm.h.resize(static_cast<std::size_t>(hm.n) * hm.n);
    auto cr = [](double p0, double p1, double p2, double p3, double t) {
        return p1 + 0.5 * t * (p2 - p0 + t * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 + t * (3.0 * (p1 - p2) + p3 - p0)));
    };
    const Noise rough(seed * 104729u + 5u);
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
            hm.set(x, z, static_cast<float>(hv));
        }
    if (grownOut) *grownOut = hm;

    // 3. BREACH + WATER + THERMAL (ADR-0123/0124), on the GPU
    ErosionParams e = w.water;
    e.seaLevel = sea;
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
