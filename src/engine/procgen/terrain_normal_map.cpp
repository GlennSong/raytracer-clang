#include "terrain_normal_map.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <thread>

namespace engine {

namespace {
uint8_t encode(double v) { return static_cast<uint8_t>(std::lround(std::clamp(127.5 + 127.5 * v, 0.0, 255.0))); }
double decode(uint8_t b) { return b / 127.5 - 1.0; }
constexpr char kMagic[8] = {'R', 'T', 'N', 'M', 'A', 'P', '0', '1'};
}  // namespace

Vec3 TerrainNormalMap::at(double x, double z) const {
    if (empty()) return Vec3(0, 1, 0);
    const int i = std::clamp(static_cast<int>(std::floor((x - originX) / texel())), 0, size - 1);
    const int j = std::clamp(static_cast<int>(std::floor((z - originZ) / texel())), 0, size - 1);
    const std::size_t o = (static_cast<std::size_t>(j) * size + i) * 2;
    const double nx = decode(xz[o]), nz = decode(xz[o + 1]);
    return Vec3(nx, std::sqrt(std::max(0.0, 1.0 - nx * nx - nz * nz)), nz);
}

TerrainNormalMap bakeTerrainNormalMap(const std::function<double(double, double)>& height, double originX,
                                      double originZ, double extent, int size) {
    TerrainNormalMap m;
    m.size = std::max(1, size);
    m.originX = originX;
    m.originZ = originZ;
    m.extent = extent;
    m.xz.assign(static_cast<std::size_t>(m.size) * m.size * 2, 128);
    const double d = m.texel(), h = 0.5 * d;
    // The ground sampled every half texel, from two texels before the first centre: texel i's centre is half-step
    // column 2i + 2, its square's edges columns 2i and 2i + 4 (+-d), each edge's mean five samples (trapezoid). A
    // thread takes a band of rows and samples the half-step rows it needs.
    const int W = 2 * m.size + 3;
    auto xs = [&](int a) { return originX + 0.5 * d + (a - 2) * h; };
    auto zs = [&](int b) { return originZ + 0.5 * d + (b - 2) * h; };
    static constexpr double kW[5] = {0.125, 0.25, 0.25, 0.25, 0.125};
    const int T = std::max(1u, std::min(32u, std::thread::hardware_concurrency()));
    const int band = (m.size + T - 1) / T;
    std::vector<std::thread> th;
    for (int t = 0; t < T; ++t)
        th.emplace_back([&, t] {
            const int j0 = t * band, j1 = std::min(m.size, j0 + band);
            if (j0 >= j1) return;
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
                    m.xz[o] = encode(nrm.x);
                    m.xz[o + 1] = encode(nrm.z);
                }
        });
    for (auto& x : th) x.join();
    return m;
}

bool writeTerrainNormalMap(const TerrainNormalMap& m, uint64_t key, const std::string& path) {
    const std::string tmp = path + ".tmp";
    std::FILE* f = std::fopen(tmp.c_str(), "wb");
    if (!f) return false;
    const int32_t size = m.size;
    const double geo[3] = {m.originX, m.originZ, m.extent};
    bool ok = std::fwrite(kMagic, 1, 8, f) == 8 && std::fwrite(&key, sizeof key, 1, f) == 1 &&
              std::fwrite(&size, sizeof size, 1, f) == 1 && std::fwrite(geo, sizeof geo, 1, f) == 1 &&
              std::fwrite(m.xz.data(), 1, m.xz.size(), f) == m.xz.size();
    ok = std::fclose(f) == 0 && ok;
    if (ok) ok = std::rename(tmp.c_str(), path.c_str()) == 0;
    if (!ok) std::remove(tmp.c_str());
    return ok;
}

bool readTerrainNormalMap(const std::string& path, uint64_t key, TerrainNormalMap& out) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    char magic[8];
    uint64_t k = 0;
    int32_t size = 0;
    double geo[3];
    bool ok = std::fread(magic, 1, 8, f) == 8 && std::equal(magic, magic + 8, kMagic) && std::fread(&k, sizeof k, 1, f) == 1 &&
              k == key && std::fread(&size, sizeof size, 1, f) == 1 && size > 0 && size <= 32768 &&
              std::fread(geo, sizeof geo, 1, f) == 1;
    if (ok) {
        out.size = size;
        out.originX = geo[0];
        out.originZ = geo[1];
        out.extent = geo[2];
        out.xz.resize(static_cast<std::size_t>(size) * size * 2);
        ok = std::fread(out.xz.data(), 1, out.xz.size(), f) == out.xz.size();
    }
    std::fclose(f);
    if (!ok) out = TerrainNormalMap{};
    return ok;
}

}  // namespace engine
