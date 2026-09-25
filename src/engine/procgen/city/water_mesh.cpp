#include "water_mesh.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace engine {

namespace {
// A cell is sea when any of its corners is below sea level (on the extent's ground).
bool seaCell(const HeightSampler& ground, const WaterMeshParams& p, double x0, double z0) {
    const double x1 = x0 + p.cell, z1 = z0 + p.cell;
    return ground(x0, z0) < p.seaLevel || ground(x1, z0) < p.seaLevel || ground(x0, z1) < p.seaLevel ||
           ground(x1, z1) < p.seaLevel;
}
}  // namespace

std::vector<std::vector<Vec2>> waterMeshCells(const HeightSampler& floor, const WaterMeshParams& p) {
    std::vector<std::vector<Vec2>> out;
    if (!floor || p.seaLevel <= -1e29 || p.cell <= 0.0) return out;
    const HeightSampler& ground = p.extent ? p.extent : floor;
    const int nx = std::max(1, static_cast<int>(std::ceil((p.hi.x - p.lo.x) / p.cell)));
    const int nz = std::max(1, static_cast<int>(std::ceil((p.hi.y - p.lo.y) / p.cell)));
    for (int j = 0; j < nz; ++j)
        for (int i = 0; i < nx; ++i) {
            const double x0 = p.lo.x + i * p.cell, z0 = p.lo.y + j * p.cell;
            if (!seaCell(ground, p, x0, z0)) continue;
            out.push_back({Vec2(x0, z0), Vec2(x0 + p.cell, z0), Vec2(x0 + p.cell, z0 + p.cell), Vec2(x0, z0 + p.cell)});
        }
    return out;
}

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

    // An INDEXED grid (ADR-0096): a corner is shared by the cells that meet it -- its depth and
    // shore distance are the corner's own, so the shading is the same -- where each cell used to
    // carry six vertices of its own and run the shore search at every one of them.
    std::vector<int32_t> vid(static_cast<std::size_t>(nx + 1) * (nz + 1), -1);
    auto corner = [&](int i, int j) -> uint32_t {
        int32_t& id = vid[static_cast<std::size_t>(j) * (nx + 1) + i];
        if (id < 0) {
            const double x = p.lo.x + i * p.cell, z = p.lo.y + j * p.cell;
            Vertex v(P(x, z), up, Vec3(1, 0, 0), static_cast<float>(std::max(0.0, depthAt(x, z))), shoreAt(x, z));
            v.color = col;
            id = static_cast<int32_t>(mesh.vertices.size());
            mesh.vertices.push_back(v);
        }
        return static_cast<uint32_t>(id);
    };
    for (int j = 0; j < nz; ++j)
        for (int i = 0; i < nx; ++i) {
            const double x0 = p.lo.x + i * p.cell, z0 = p.lo.y + j * p.cell;
            if (!seaCell(p.extent ? p.extent : floor, p, x0, z0)) continue;  // all land: skip
            const uint32_t a00 = corner(i, j), a10 = corner(i + 1, j), a11 = corner(i + 1, j + 1), a01 = corner(i, j + 1);
            // wound to face up, as emitTriUV winds them
            mesh.indices.insert(mesh.indices.end(), {a00, a10, a11, a00, a11, a01});
        }
    return mesh;
}

}  // namespace engine
