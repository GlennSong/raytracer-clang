#include "terrain_maps.h"

#include "stream_power.h"
#include "terrain_weather.h"
#include "../../log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace engine {

namespace {
constexpr const char* kMapsCodeTag = "2026-09-26.1";

template <class F>
void rows(int n, F&& f) {
    const int T = std::max(1u, std::min(32u, std::thread::hardware_concurrency()));
    std::vector<std::thread> th;
    for (int t = 0; t < T; ++t) th.emplace_back([&, t] { for (int z = t; z < n; z += T) f(z); });
    for (auto& x : th) x.join();
}

// separable box blur of radius r (clamped edges), running sums
std::vector<float> boxBlur(const std::vector<float>& in, int n, int r) {
    std::vector<float> a(in), b(in.size());
    for (int axis = 0; axis < 2; ++axis) {
        rows(n, [&](int z) {
            auto at = [&](int k) { const int kk = std::clamp(k, 0, n - 1); return axis == 0 ? a[static_cast<std::size_t>(z) * n + kk] : a[static_cast<std::size_t>(kk) * n + z]; };
            double s = 0.0;
            for (int k = -r; k <= r; ++k) s += at(k);
            for (int x = 0; x < n; ++x) {
                const std::size_t o = axis == 0 ? static_cast<std::size_t>(z) * n + x : static_cast<std::size_t>(x) * n + z;
                b[o] = static_cast<float>(s / (2 * r + 1));
                s += at(x + r + 1) - at(x - r);
            }
        });
        a.swap(b);
    }
    return a;
}

double smooth(double a, double b, double x) {
    const double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
uint8_t u8(double v) { return static_cast<uint8_t>(std::lround(std::clamp(v, 0.0, 1.0) * 255.0)); }
}  // namespace

TerrainMapSample TerrainMaps::at(double worldX, double worldZ) const {
    TerrainMapSample s;
    if (n < 2) return s;
    const double gx = std::clamp((worldX / worldSize + 0.5) * (n - 1), 0.0, n - 1.0), gz = std::clamp((worldZ / worldSize + 0.5) * (n - 1), 0.0, n - 1.0);
    const int x0 = std::min(static_cast<int>(gx), n - 2), z0 = std::min(static_cast<int>(gz), n - 2);
    const double fx = gx - x0, fz = gz - z0;
    auto bl = [&](const std::vector<uint8_t>& m) {
        auto g = [&](int x, int z) { return m[static_cast<std::size_t>(z) * n + x] / 255.0; };
        return (g(x0, z0) * (1 - fx) + g(x0 + 1, z0) * fx) * (1 - fz) + (g(x0, z0 + 1) * (1 - fx) + g(x0 + 1, z0 + 1) * fx) * fz;
    };
    s.wet = bl(wet);
    s.scree = bl(scree);
    s.soil = bl(soil);
    s.convex = bl(convex) * 2.0 - 1.0;
    return s;
}

TerrainMaps computeTerrainMaps(const Heightmap& hmIn, double seaLevel) {
    const int n = hmIn.n;
    const double cell = hmIn.worldSize / (n - 1);
    const std::size_t N = static_cast<std::size_t>(n) * n;
    // drainage area: one routing pass of the stream-power solver (K = 0: the ground is not touched)
    std::vector<float> area;
    {
        Heightmap hm = hmIn;
        StreamPowerParams sp;
        sp.iterations = 1;
        sp.K = 0.0;
        sp.uplift = 0.0;
        sp.diffusion = 0.0;
        sp.rescale = false;
        sp.seaLevel = seaLevel;
        const std::vector<float> zero(N, 0.0f);
        streamPowerErode(hm, zero, sp, nullptr, &area);
    }
    std::vector<float> slope(N), cliff(N), wet(N), soil(N), convex(N);
    const std::vector<float> hs = boxBlur(hmIn.h, n, std::max(1, static_cast<int>(8.0 / cell)));   // ~15 m for curvature
    const double cliffTan = std::tan(40.0 * 3.14159265358979 / 180.0);
    rows(n, [&](int z) {
        for (int x = 0; x < n; ++x) {
            const std::size_t i = static_cast<std::size_t>(z) * n + x;
            const int xa = std::max(0, x - 1), xb = std::min(n - 1, x + 1), za = std::max(0, z - 1), zb = std::min(n - 1, z + 1);
            auto H = [&](int xx, int zz) { return static_cast<double>(hmIn.h[static_cast<std::size_t>(zz) * n + xx]); };
            const double gx = (H(xb, z) - H(xa, z)) / ((xb - xa) * cell), gz = (H(x, zb) - H(x, za)) / ((zb - za) * cell);
            const double s = std::sqrt(gx * gx + gz * gz);
            slope[i] = static_cast<float>(s);
            cliff[i] = s > cliffTan ? 1.0f : 0.0f;
            const int k = std::max(1, static_cast<int>(12.0 / cell));
            auto S = [&](int xx, int zz) { return static_cast<double>(hs[static_cast<std::size_t>(std::clamp(zz, 0, n - 1)) * n + std::clamp(xx, 0, n - 1)]); };
            const double lap = (S(x - k, z) + S(x + k, z) + S(x, z - k) + S(x, z + k) - 4.0 * S(x, z)) / (k * k * cell * cell);
            const double cv = std::clamp(-lap * 25.0, -1.0, 1.0);   // a ridge of ~50 m radius reads +0.5
            convex[i] = static_cast<float>(cv);
            const bool sea = hmIn.h[i] < seaLevel;
            wet[i] = sea ? 0.0f : static_cast<float>(smooth(3.3, 5.5, std::log10(std::max(1.0f, area[i]))));
            soil[i] = sea ? 1.0f : static_cast<float>(std::clamp((1.0 - smooth(0.45, 0.85, s)) * (0.65 - 0.6 * cv) + 0.3 * wet[i], 0.0, 1.0));
        }
    });
    // scree: moderate slopes with cliffs close by (the fall zone), thicker in the hollows below them
    const std::vector<float> cliffNear = boxBlur(cliff, n, std::max(1, static_cast<int>(35.0 / cell)));
    // half resolution out
    TerrainMaps m;
    m.n = (n - 1) / 2 + 1;
    m.worldSize = hmIn.worldSize;
    const std::size_t M = static_cast<std::size_t>(m.n) * m.n;
    m.wet.resize(M); m.scree.resize(M); m.soil.resize(M); m.convex.resize(M);
    rows(m.n, [&](int z) {
        for (int x = 0; x < m.n; ++x) {
            const std::size_t o = static_cast<std::size_t>(z) * m.n + x;
            const std::size_t i = static_cast<std::size_t>(std::min(n - 1, 2 * z)) * n + std::min(n - 1, 2 * x);
            const double s = slope[i];
            const double scree = smooth(0.08, 0.3, cliffNear[i]) * smooth(0.3, 0.45, s) * (1.0 - smooth(0.78, 0.9, s)) * (0.7 - 0.5 * convex[i]);
            m.wet[o] = u8(wet[i]);
            m.scree[o] = u8(scree);
            m.soil[o] = u8(soil[i] * (1.0 - 0.7 * scree));
            m.convex[o] = u8(0.5 + 0.5 * convex[i]);
        }
    });
    return m;
}

std::shared_ptr<const TerrainMaps> weatheredMapsFor(const nlohmann::json& tjIn) {
    if (!tjIn.contains("weather") || !tjIn["weather"].is_object()) return nullptr;
    nlohmann::json tj = tjIn;
    tj.erase("rivers");
    tj.erase("groundCover");   // the cover reads the maps; its palette does not change them
    tj.erase("forest");        // ...nor does the forest standing on them
    tj.erase("trails");        // ...nor the paths over them
    static std::mutex mu;
    static std::map<std::string, std::shared_ptr<const TerrainMaps>> memo;
    const std::string key = tj.dump();
    std::lock_guard<std::mutex> lock(mu);
    if (auto it = memo.find(key); it != memo.end()) return it->second;
    std::uint64_t h = 1469598103934665603ULL;
    auto fold = [&](const std::string& s) { for (unsigned char c : s) { h ^= c; h *= 1099511628211ULL; } };
    fold(kMapsCodeTag);
    fold(weatherCodeTag());
    fold(key);
    char path[256];
    std::snprintf(path, sizeof path, "cache/terrain/maps_%016llx.bin", static_cast<unsigned long long>(h));
    auto m = std::make_shared<TerrainMaps>();
    bool ok = false;
    if (std::getenv("RT_NOCACHE") == nullptr)
        if (std::FILE* f = std::fopen(path, "rb")) {
            ok = std::fread(&m->n, sizeof m->n, 1, f) == 1 && std::fread(&m->worldSize, sizeof m->worldSize, 1, f) == 1 && m->n > 2 && m->n < 70000;
            if (ok) {
                const std::size_t M = static_cast<std::size_t>(m->n) * m->n;
                for (auto* v : {&m->wet, &m->scree, &m->soil, &m->convex}) { v->resize(M); ok = ok && std::fread(v->data(), 1, M, f) == M; }
            }
            std::fclose(f);
        }
    if (!ok) {
        const Heightmap hm = weatheredTerrainCached(tjIn);
        *m = computeTerrainMaps(hm, tjIn.value("seaLevel", -1e30));
        std::error_code ec;
        std::filesystem::create_directories("cache/terrain", ec);
        if (std::FILE* f = std::fopen(path, "wb")) {
            std::fwrite(&m->n, sizeof m->n, 1, f);
            std::fwrite(&m->worldSize, sizeof m->worldSize, 1, f);
            for (auto* v : {&m->wet, &m->scree, &m->soil, &m->convex}) std::fwrite(v->data(), 1, v->size(), f);
            std::fclose(f);
        }
        LOG_INFO << "[maps] computed " << m->n << "^2 ground maps, cached " << path;
    }
    memo[key] = m;
    return m;
}

}  // namespace engine
