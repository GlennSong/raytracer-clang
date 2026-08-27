#include "terrain_horizon.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

double HeightRaster::at(double worldX, double worldZ) const {
    if (size <= 0 || height.empty()) return 0.0;
    const double t = texel();
    double fx = (worldX - originX) / t - 0.5, fz = (worldZ - originZ) / t - 0.5;
    fx = std::clamp(fx, 0.0, static_cast<double>(size - 1));
    fz = std::clamp(fz, 0.0, static_cast<double>(size - 1));
    const int x0 = static_cast<int>(fx), z0 = static_cast<int>(fz);
    const int x1 = std::min(x0 + 1, size - 1), z1 = std::min(z0 + 1, size - 1);
    const double ax = fx - x0, az = fz - z0;
    const double h00 = height[z0 * size + x0], h10 = height[z0 * size + x1];
    const double h01 = height[z1 * size + x0], h11 = height[z1 * size + x1];
    return (h00 * (1 - ax) + h10 * ax) * (1 - az) + (h01 * (1 - ax) + h11 * ax) * az;
}

HeightRaster rasterizeHeights(const std::function<double(double, double)>& heightAt,
                              double originX, double originZ, double extent, int size) {
    HeightRaster r;
    if (size < 2 || extent <= 0.0) return r;
    r.size = size;
    r.originX = originX;
    r.originZ = originZ;
    r.extent = extent;
    r.height.resize(static_cast<std::size_t>(size) * size);
    const double t = extent / size;
    for (int z = 0; z < size; ++z)
        for (int x = 0; x < size; ++x)
            r.height[static_cast<std::size_t>(z) * size + x] = static_cast<float>(
                heightAt(originX + (x + 0.5) * t, originZ + (z + 0.5) * t));
    return r;
}

uint8_t HorizonMap::encode(double sinElev) {
    const double u = (sinElev - kEncodeLo) / (kEncodeHi - kEncodeLo);
    return static_cast<uint8_t>(std::lround(std::clamp(u, 0.0, 1.0) * 255.0));
}

double HorizonMap::decode(uint8_t v) {
    return kEncodeLo + (kEncodeHi - kEncodeLo) * (v / 255.0);
}

double HorizonMap::sinElevationAt(double worldX, double worldZ) const {
    if (size <= 0 || sinElevation.empty()) return kEncodeLo;
    const double t = extent / size;
    int x = static_cast<int>(std::floor((worldX - originX) / t));
    int z = static_cast<int>(std::floor((worldZ - originZ) / t));
    x = std::clamp(x, 0, size - 1);
    z = std::clamp(z, 0, size - 1);
    return decode(sinElevation[static_cast<std::size_t>(z) * size + x]);
}

HorizonMap computeHorizonMap(const HeightRaster& raster, double azimuthDeg,
                             double maxDistance, double eyeHeight,
                             const std::function<void(int, const std::function<void(int)>&)>*
                                 parallelRows) {
    HorizonMap m;
    if (raster.empty()) return m;
    m.size = raster.size;
    m.originX = raster.originX;
    m.originZ = raster.originZ;
    m.extent = raster.extent;
    m.azimuthDeg = azimuthDeg;
    m.sinElevation.assign(static_cast<std::size_t>(m.size) * m.size, HorizonMap::encode(0.0));

    // Bearing: 0 north (-z), 90 east (+x).
    const double a = azimuthDeg * kPi / 180.0;
    const double dx = std::sin(a), dz = -std::cos(a);
    const double step = raster.texel();
    const int steps = std::max(1, static_cast<int>(maxDistance / step));
    const int size = m.size;

    auto row = [&](int z) {
        for (int x = 0; x < size; ++x) {
            const double sx = raster.originX + (x + 0.5) * step;
            const double sz = raster.originZ + (z + 0.5) * step;
            const double h0 = raster.height[static_cast<std::size_t>(z) * size + x] + eyeHeight;
            double best = 0.0;   // sin of the highest elevation seen (flat = 0)
            for (int i = 1; i <= steps; ++i) {
                const double d = i * step;
                const double px = sx + dx * d, pz = sz + dz * d;
                if (px < raster.originX || pz < raster.originZ ||
                    px >= raster.originX + raster.extent || pz >= raster.originZ + raster.extent)
                    break;   // off the raster: nothing known beyond
                const double dh = raster.at(px, pz) - h0;
                if (dh <= 0.0) continue;
                const double s = dh / std::sqrt(dh * dh + d * d);
                if (s > best) best = s;
            }
            m.sinElevation[static_cast<std::size_t>(z) * size + x] = HorizonMap::encode(best);
        }
    };
    if (parallelRows) (*parallelRows)(size, row);
    else for (int z = 0; z < size; ++z) row(z);
    return m;
}

}  // namespace engine
