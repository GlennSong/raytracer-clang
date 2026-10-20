#include "terrain_world_map.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <thread>

namespace engine {

namespace {
uint8_t encode(double v) { return static_cast<uint8_t>(std::lround(std::clamp(127.5 + 127.5 * v, 0.0, 255.0))); }
double decode(uint8_t b) { return b / 127.5 - 1.0; }
constexpr char kMagic[8] = {'R', 'T', 'W', 'M', 'A', 'P', '0', '1'};

// rows [0, size) in bands, one a hardware thread
void parallelBands(int size, const std::function<void(int, int)>& band) {
    const int T = std::max(1u, std::min(32u, std::thread::hardware_concurrency()));
    const int per = (size + T - 1) / T;
    std::vector<std::thread> th;
    for (int t = 0; t < T; ++t) {
        const int j0 = t * per, j1 = std::min(size, j0 + per);
        if (j0 < j1) th.emplace_back([&band, j0, j1] { band(j0, j1); });
    }
    for (auto& x : th) x.join();
}

TerrainWorldMap emptyMap(int channels, double originX, double originZ, double extent, int size, uint8_t fill) {
    TerrainWorldMap m;
    m.size = std::max(1, size);
    m.originX = originX;
    m.originZ = originZ;
    m.extent = extent;
    m.channels = channels;
    m.texels.assign(static_cast<std::size_t>(m.size) * m.size * channels, fill);
    return m;
}
}  // namespace

double TerrainWorldMap::at(double x, double z, int c) const {
    if (empty() || c < 0 || c >= channels) return 0.0;
    const int i = std::clamp(static_cast<int>(std::floor((x - originX) / texel())), 0, size - 1);
    const int j = std::clamp(static_cast<int>(std::floor((z - originZ) / texel())), 0, size - 1);
    return texels[(static_cast<std::size_t>(j) * size + i) * channels + c] / 255.0;
}

Vec3 TerrainWorldMap::normalAt(double x, double z) const {
    if (empty() || channels < 2) return Vec3(0, 1, 0);
    const double nx = decode(static_cast<uint8_t>(std::lround(at(x, z, 0) * 255.0)));
    const double nz = decode(static_cast<uint8_t>(std::lround(at(x, z, 1) * 255.0)));
    return Vec3(nx, std::sqrt(std::max(0.0, 1.0 - nx * nx - nz * nz)), nz);
}

TerrainWorldMap bakeTerrainNormalMap(const std::function<double(double, double)>& height, double originX,
                                     double originZ, double extent, int size) {
    TerrainWorldMap m = emptyMap(2, originX, originZ, extent, size, 128);
    const double d = m.texel(), h = 0.5 * d;
    // The ground sampled every half texel, from two texels before the first centre: texel i's centre is half-step
    // column 2i + 2, its square's edges columns 2i and 2i + 4 (+-d), each edge's mean five samples (trapezoid). A
    // thread takes a band of rows and samples the half-step rows it needs.
    const int W = 2 * m.size + 3;
    auto xs = [&](int a) { return originX + 0.5 * d + (a - 2) * h; };
    auto zs = [&](int b) { return originZ + 0.5 * d + (b - 2) * h; };
    static constexpr double kW[5] = {0.125, 0.25, 0.25, 0.25, 0.125};
    parallelBands(m.size, [&](int j0, int j1) {
        const int b0 = 2 * j0, rows = 2 * (j1 - j0) + 3;   // half-step rows 2 j0 .. 2 (j1 - 1) + 4
        std::vector<double> H(static_cast<std::size_t>(rows) * W);
        for (int r = 0; r < rows; ++r)
            for (int a = 0; a < W; ++a) H[static_cast<std::size_t>(r) * W + a] = height(xs(a), zs(b0 + r));
        auto at = [&](int a, int b) { return H[static_cast<std::size_t>(b - b0) * W + a]; };
        for (int j = j0; j < j1; ++j)
            for (int i = 0; i < m.size; ++i) {
                const int a = 2 * i + 2, b = 2 * j + 2;
                double e = 0, w = 0, n = 0, s = 0;
                for (int k = 0; k < 5; ++k) {
                    e += kW[k] * at(a + 2, b - 2 + k);
                    w += kW[k] * at(a - 2, b - 2 + k);
                    n += kW[k] * at(a - 2 + k, b + 2);
                    s += kW[k] * at(a - 2 + k, b - 2);
                }
                const Vec3 nrm = normalize(Vec3(-(e - w) / (2.0 * d), 1.0, -(n - s) / (2.0 * d)));
                const std::size_t o = (static_cast<std::size_t>(j) * m.size + i) * 2;
                m.texels[o] = encode(nrm.x);
                m.texels[o + 1] = encode(nrm.z);
            }
    });
    return m;
}

TerrainWorldMap bakeTerrainWorldMap(int channels, const std::function<void(double, double, uint8_t*)>& texel,
                                    double originX, double originZ, double extent, int size) {
    TerrainWorldMap m = emptyMap(channels, originX, originZ, extent, size, 0);
    const double d = m.texel();
    parallelBands(m.size, [&](int j0, int j1) {
        for (int j = j0; j < j1; ++j)
            for (int i = 0; i < m.size; ++i)
                texel(originX + (i + 0.5) * d, originZ + (j + 0.5) * d,
                      &m.texels[(static_cast<std::size_t>(j) * m.size + i) * channels]);
    });
    return m;
}

bool writeTerrainWorldMap(const TerrainWorldMap& m, uint64_t key, const std::string& path) {
    const std::string tmp = path + ".tmp";
    std::FILE* f = std::fopen(tmp.c_str(), "wb");
    if (!f) return false;
    const int32_t head[2] = {m.size, m.channels};
    const double geo[3] = {m.originX, m.originZ, m.extent};
    bool ok = std::fwrite(kMagic, 1, 8, f) == 8 && std::fwrite(&key, sizeof key, 1, f) == 1 &&
              std::fwrite(head, sizeof head, 1, f) == 1 && std::fwrite(geo, sizeof geo, 1, f) == 1 &&
              std::fwrite(m.texels.data(), 1, m.texels.size(), f) == m.texels.size();
    ok = std::fclose(f) == 0 && ok;
    if (ok) ok = std::rename(tmp.c_str(), path.c_str()) == 0;
    if (!ok) std::remove(tmp.c_str());
    return ok;
}

bool readTerrainWorldMap(const std::string& path, uint64_t key, TerrainWorldMap& out) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    char magic[8];
    uint64_t k = 0;
    int32_t head[2] = {0, 0};
    double geo[3];
    bool ok = std::fread(magic, 1, 8, f) == 8 && std::equal(magic, magic + 8, kMagic) && std::fread(&k, sizeof k, 1, f) == 1 &&
              k == key && std::fread(head, sizeof head, 1, f) == 1 && head[0] > 0 && head[0] <= 32768 && head[1] >= 1 &&
              head[1] <= 4 && std::fread(geo, sizeof geo, 1, f) == 1;
    if (ok) {
        out.size = head[0];
        out.channels = head[1];
        out.originX = geo[0];
        out.originZ = geo[1];
        out.extent = geo[2];
        out.texels.resize(static_cast<std::size_t>(out.size) * out.size * out.channels);
        ok = std::fread(out.texels.data(), 1, out.texels.size(), f) == out.texels.size();
    }
    std::fclose(f);
    if (!ok) out = TerrainWorldMap{};
    return ok;
}

}  // namespace engine
