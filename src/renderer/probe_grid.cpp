#include "probe_grid.h"

#include <algorithm>
#include <cmath>

namespace engine {

void buildProbeGrid(const std::vector<ReflectionProbe>& probes, ProbeGrid& out) {
    std::fill(out.cells.begin(), out.cells.end(), 0u);
    const std::size_t n = std::min<std::size_t>(probes.size(), 255);
    double lx = 1e30, lz = 1e30, hx = -1e30, hz = -1e30;
    for (std::size_t i = 0; i < n; ++i) {
        const ReflectionProbe& p = probes[i];
        if (p.boxMin.x > p.boxMax.x || p.boxMin.z > p.boxMax.z) continue;
        lx = std::min(lx, p.boxMin.x); lz = std::min(lz, p.boxMin.z);
        hx = std::max(hx, p.boxMax.x); hz = std::max(hz, p.boxMax.z);
    }
    if (lx > hx) return;
    const int k = ProbeGrid::kSize;
    out.cell = static_cast<float>(std::max(std::max(hx - lx, hz - lz) / k, 1.0));
    out.originX = static_cast<float>(lx);
    out.originZ = static_cast<float>(lz);
    for (int cz = 0; cz < k; ++cz)
        for (int cx = 0; cx < k; ++cx) {
            const double x0 = lx + cx * static_cast<double>(out.cell), z0 = lz + cz * static_cast<double>(out.cell);
            const double x1 = x0 + out.cell, z1 = z0 + out.cell;
            const double mx = 0.5 * (x0 + x1), mz = 0.5 * (z0 + z1);
            int best[2] = {-1, -1}, inBest = -1;
            double bestD[2] = {1e30, 1e30}, inD = 1e30;
            for (std::size_t i = 0; i < n; ++i) {
                const ReflectionProbe& p = probes[i];
                if (p.boxMin.x > x1 || p.boxMax.x < x0 || p.boxMin.z > z1 || p.boxMax.z < z0) continue;
                const double dx = p.position.x - mx, dz = p.position.z - mz, d = dx * dx + dz * dz;
                if (p.indoor) {
                    if (d < inD) { inD = d; inBest = static_cast<int>(i); }
                    continue;
                }
                if (d < bestD[0]) {
                    best[1] = best[0]; bestD[1] = bestD[0];
                    best[0] = static_cast<int>(i); bestD[0] = d;
                } else if (d < bestD[1]) {
                    best[1] = static_cast<int>(i); bestD[1] = d;
                }
            }
            out.cells[static_cast<std::size_t>(cz * k + cx)] =
                static_cast<uint32_t>(best[0] + 1) | (static_cast<uint32_t>(best[1] + 1) << 8) |
                (static_cast<uint32_t>(inBest + 1) << 16);
        }
}

}  // namespace engine
