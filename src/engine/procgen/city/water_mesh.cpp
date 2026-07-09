#include "water_mesh.h"

#include <cmath>

namespace engine {

RenderMesh buildWaterMesh(const HeightSampler& floor, const WaterMeshParams& p) {
    RenderMesh mesh;
    if (!floor || p.seaLevel <= -1e29 || p.cell <= 0.0) return mesh;

    auto depthAt = [&](double x, double z) { return p.seaLevel - floor(x, z); };
    // Distance to the nearest land, capped at foamBand — baked into UV.v so the
    // shader draws a foam band only at the true waterline (not on offshore shoals).
    auto shoreAt = [&](double x, double z) -> float {
        if (depthAt(x, z) <= 0.0) return 0.0f;
        for (double r = p.cell; r <= p.foamBand; r += p.cell * 0.75)
            for (int k = 0; k < 12; ++k) {
                double t = 6.28318530718 * k / 12.0;
                if (depthAt(x + std::cos(t) * r, z + std::sin(t) * r) <= 0.0)
                    return static_cast<float>(r);
            }
        return static_cast<float>(p.foamBand);
    };

    const int nx = std::max(1, static_cast<int>(std::ceil((p.hi.x - p.lo.x) / p.cell)));
    const int nz = std::max(1, static_cast<int>(std::ceil((p.hi.y - p.lo.y) / p.cell)));
    const Vec3 up(0, 1, 0);
    const Vec3 col(0.09, 0.22, 0.34);          // fallback tint; the shader grades by depth
    auto P = [&](double x, double z) { return Vec3(x, p.seaLevel, z); };

    for (int j = 0; j < nz; ++j)
        for (int i = 0; i < nx; ++i) {
            double x0 = p.lo.x + i * p.cell, z0 = p.lo.y + j * p.cell;
            double x1 = x0 + p.cell, z1 = z0 + p.cell;
            double d00 = depthAt(x0, z0), d10 = depthAt(x1, z0),
                   d01 = depthAt(x0, z1), d11 = depthAt(x1, z1);
            if (d00 <= 0 && d10 <= 0 && d01 <= 0 && d11 <= 0) continue;  // all land: skip
            float u00 = static_cast<float>(std::max(0.0, d00)), u10 = static_cast<float>(std::max(0.0, d10)),
                  u01 = static_cast<float>(std::max(0.0, d01)), u11 = static_cast<float>(std::max(0.0, d11));
            float s00 = shoreAt(x0, z0), s10 = shoreAt(x1, z0),
                  s01 = shoreAt(x0, z1), s11 = shoreAt(x1, z1);
            MeshBuilder::emitTriUV(mesh, P(x0, z0), P(x1, z0), P(x1, z1), up, col,
                                   u00, s00, u10, s10, u11, s11);
            MeshBuilder::emitTriUV(mesh, P(x0, z0), P(x1, z1), P(x0, z1), up, col,
                                   u00, s00, u11, s11, u01, s01);
        }
    return mesh;
}

}  // namespace engine
