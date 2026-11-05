#include "water_mesh.h"
#include "../../mesh_builder.h"
#include "../texture_field.h"   // chamferSweepGrid

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace engine {

namespace {
// A cell is sea when any of its corners is below sea level (on the extent's ground).
bool seaCell(const HeightField& ground, const WaterMeshParams& p, double x0, double z0) {
    const double x1 = x0 + p.cell, z1 = z0 + p.cell;
    return ground(x0, z0) < p.seaLevel || ground(x1, z0) < p.seaLevel || ground(x0, z1) < p.seaLevel ||
           ground(x1, z1) < p.seaLevel;
}
}  // namespace

std::vector<std::vector<Vec2>> waterMeshCells(const HeightField& floor, const WaterMeshParams& p) {
    std::vector<std::vector<Vec2>> out;
    if (!floor || p.seaLevel <= -1e29 || p.cell <= 0.0) return out;
    const HeightField& ground = p.extent ? p.extent : floor;
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

RenderMesh buildWaterMesh(const HeightField& floor, const WaterMeshParams& p) {
    RenderMesh mesh;
    if (!floor || p.seaLevel <= -1e29 || p.cell <= 0.0) return mesh;

    auto depthAt = [&](double x, double z) { return p.seaLevel - floor(x, z); };
    const int nx = std::max(1, static_cast<int>(std::ceil((p.hi.x - p.lo.x) / p.cell)));
    const int nz = std::max(1, static_cast<int>(std::ceil((p.hi.y - p.lo.y) / p.cell)));
    // THE SHORE DISTANCE, SIGNED (ADR-0330): metres from the WATERLINE at every corner, + offshore and - inland,
    // so the interpolated value crosses 0 where the interpolated depth does. Seeded on each edge that crosses the
    // waterline (the crossing's own distance from either end) and spread by the chamfer sweep, capped at the wider
    // of the foam and surf bands. (It used to be the first of 15 m rings to reach land from a corner, 0 at any land
    // corner: the foam's band ran out from a corner under the beach, and never showed.)
    const int n = std::max(nx, nz) + 1;
    const float cap = static_cast<float>(std::max(p.foamBand, p.surfBand));
    std::vector<float> depth(static_cast<std::size_t>(n) * n, -1.0f);
    for (int j = 0; j <= nz; ++j)
        for (int i = 0; i <= nx; ++i)
            depth[static_cast<std::size_t>(j) * n + i] = static_cast<float>(depthAt(p.lo.x + i * p.cell, p.lo.y + j * p.cell));
    const float far = 1e9f;
    std::vector<float> off(depth.size(), far), in(depth.size(), far);
    for (int j = 0; j <= nz; ++j)
        for (int i = 0; i <= nx; ++i) {
            const std::size_t a = static_cast<std::size_t>(j) * n + i;
            const int nb[4][2] = {{i + 1, j}, {i - 1, j}, {i, j + 1}, {i, j - 1}};
            for (const auto& b : nb) {
                if (b[0] < 0 || b[1] < 0 || b[0] > nx || b[1] > nz) continue;
                const float da = depth[a], db = depth[static_cast<std::size_t>(b[1]) * n + b[0]];
                if ((da > 0.0f) == (db > 0.0f)) continue;
                const float t = da / (da - db);   // the crossing, from this corner (0..1 of the edge)
                (da > 0.0f ? off[a] : in[a]) = std::min(da > 0.0f ? off[a] : in[a], t * static_cast<float>(p.cell));
            }
        }
    const float c = static_cast<float>(p.cell);
    chamferSweepGrid(off, n, c, c * 1.41421356f);
    chamferSweepGrid(in, n, c, c * 1.41421356f);
    // the chamfer's eight directions draw its contours as polygons -- the surf's crests zigzagged at the cell's
    // scale -- so the distance is blurred, all but within a cell or two of the waterline (which stays put)
    std::vector<float> sd(depth.size());
    for (std::size_t a = 0; a < sd.size(); ++a) sd[a] = depth[a] > 0.0f ? std::min(off[a], cap) : -std::min(in[a], cap);
    const std::vector<float> soft = boxBlurGrid(sd, n, 2, 2);
    auto shoreAt = [&](int i, int j) -> float {
        const std::size_t a = static_cast<std::size_t>(j) * n + i;
        const float k = std::clamp((std::fabs(sd[a]) - c) / c, 0.0f, 1.0f);
        return sd[a] + (soft[a] - sd[a]) * k;
    };

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
            // (the depth signed too: the shader clamps it, and it crosses 0 with the shore distance)
            Vertex v(P(x, z), up, Vec3(1, 0, 0), depth[static_cast<std::size_t>(j) * n + i], shoreAt(i, j));
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
            MeshBuilder::gridCell(mesh, a00, a10, a11, a01);
        }
    return mesh;
}

}  // namespace engine
