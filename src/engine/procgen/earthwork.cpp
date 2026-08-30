#include "earthwork.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace engine {

namespace {

// A displacement grid with its own origin (NOT origin-centred like the erosion
// Heightmap, whose sampleWorld assumes a square about zero). Bilinear inside,
// exactly 0 outside — the field never reaches past its own domain.
struct DisplacementGrid {
    double originX = 0, originZ = 0, cell = 1;
    int nx = 0, nz = 0;
    std::vector<float> d;
    double at(double x, double z) const {
        const double fx = (x - originX) / cell, fz = (z - originZ) / cell;
        if (fx < 0 || fz < 0 || fx > nx - 1 || fz > nz - 1) return 0.0;
        const int i = std::min(nx - 2, static_cast<int>(fx));
        const int j = std::min(nz - 2, static_cast<int>(fz));
        const double tx = fx - i, tz = fz - j;
        const std::size_t k = static_cast<std::size_t>(j) * nx + i;
        return d[k] * (1 - tx) * (1 - tz) + d[k + 1] * tx * (1 - tz) +
               d[k + nx] * (1 - tx) * tz + d[k + nx + 1] * tx * tz;
    }
};

struct Level {
    double cell = 1;
    int nx = 0, nz = 0;
    std::vector<double> y;        // D
    std::vector<unsigned char> fixed;
};

}  // namespace

std::shared_ptr<const std::function<double(double, double)>> buildEarthworkField(
    const std::vector<TerrainFlatten>& roadRegions,
    const std::function<double(double, double)>& natural,
    const EarthworkParams& p, double seaLevel, EarthworkStats* statsOut) {
    if (!p.enabled || !natural) return nullptr;
    // Domain: the road regions' union AABB, plus the margin the field needs to
    // decay in (3 * reach is where a screened field is under 5%).
    double minX = std::numeric_limits<double>::max(), minZ = minX;
    double maxX = std::numeric_limits<double>::lowest(), maxZ = maxX;
    int live = 0;
    for (const TerrainFlatten& r : roadRegions) {
        if (r.polygon.size() < 3 || r.priority != kRoadFlattenPriority) continue;
        ++live;
        minX = std::min(minX, r.minX); maxX = std::max(maxX, r.maxX);
        minZ = std::min(minZ, r.minZ); maxZ = std::max(maxZ, r.maxZ);
    }
    if (live == 0) return nullptr;
    const double margin = p.margin > 0 ? p.margin : 3.0 * p.reach;
    minX -= margin; minZ -= margin; maxX += margin; maxZ += margin;
    // Only the road regions take part (the block grades and pads come LATER in
    // the loader and must not be fitted — the field is what they are built on).
    std::vector<TerrainFlatten> roads;
    roads.reserve(live);
    for (const TerrainFlatten& r : roadRegions)
        if (r.polygon.size() >= 3 && r.priority == kRoadFlattenPriority) roads.push_back(r);
    const FlattenGrid index = buildFlattenGrid(roads);

    // Finest cell: the requested one, coarsened until the grid fits a sane
    // budget (a 2.8 km city at 4 m is ~0.5 M nodes; 16 km at 4 m would not be).
    double cell = std::max(0.5, p.cell);
    const double spanX = maxX - minX, spanZ = maxZ - minZ;
    while ((spanX / cell + 1) * (spanZ / cell + 1) > 4.0e6) cell *= 1.5;

    // The cascade: coarse to fine, each level 2x the next. The coarsest
    // levels seed the smooth long-range part cheaply; the finest resolves the
    // road edges. Fixed sweep counts per level keep it deterministic.
    const int levels = 5;
    const int sweeps[levels] = {400, 96, 64, 48, 32};   // coarsest .. finest
    Level prev;
    bool havePrev = false;
    double lastResidual = 0.0;
    for (int li = 0; li < levels; ++li) {
        Level L;
        L.cell = cell * std::pow(2.0, levels - 1 - li);
        L.nx = std::max(3, static_cast<int>(std::ceil(spanX / L.cell)) + 1);
        L.nz = std::max(3, static_cast<int>(std::ceil(spanZ / L.cell)) + 1);
        const std::size_t N = static_cast<std::size_t>(L.nx) * L.nz;
        L.y.assign(N, 0.0);
        L.fixed.assign(N, 0);
        // Dirichlet data at this level's nodes: road-covered nodes take the
        // carve plane minus natural (the SAME accumulator the terrain uses, so
        // the target is the carved plane — already deck-proud, junction discs
        // included, elevated decks excluded because their carve was skipped);
        // sea-floor nodes and the outer ring are pinned to zero.
        for (int j = 0; j < L.nz; ++j)
            for (int i = 0; i < L.nx; ++i) {
                const std::size_t k = static_cast<std::size_t>(j) * L.nx + i;
                const double x = minX + i * L.cell, z = minZ + j * L.cell;
                if (i == 0 || j == 0 || i == L.nx - 1 || j == L.nz - 1) { L.fixed[k] = 1; continue; }
                const double nat = natural(x, z);
                if (nat <= seaLevel) { L.fixed[k] = 1; continue; }
                // EXACT coverage, no dilation. Growing it by half a cell (so an
                // edge node is pinned rather than solved) was tried: it moved
                // the LOD0 poke count 134 -> 144 on metro_v2 for no measured
                // benefit beyond a unit-test tolerance, and the field is
                // continuous either way — a node just outside the footprint
                // lands within centimetres of the plane by the smoothness.
                if (flattenCovers(index, roads, x, z, 0.0)) {
                    L.y[k] = applyFlatten(index, roads, x, z, nat, 0.0) - nat;
                    L.fixed[k] = 1;
                }
            }
        // Start from the coarser solution (bilinear), where this node is free.
        if (havePrev) {
            DisplacementGrid pg;
            pg.originX = minX; pg.originZ = minZ; pg.cell = prev.cell;
            pg.nx = prev.nx; pg.nz = prev.nz;
            pg.d.assign(prev.y.begin(), prev.y.end());
            for (int j = 0; j < L.nz; ++j)
                for (int i = 0; i < L.nx; ++i) {
                    const std::size_t k = static_cast<std::size_t>(j) * L.nx + i;
                    if (!L.fixed[k]) L.y[k] = pg.at(minX + i * L.cell, minZ + j * L.cell);
                }
        }
        // Screened Poisson, Gauss-Seidel in fixed row-major order:
        //   D = alpha * (W + E + N + S) / (1 + 4 alpha),  alpha = (reach / h)^2
        const double alpha = (p.reach / L.cell) * (p.reach / L.cell);
        const double w = alpha / (1.0 + 4.0 * alpha);
        for (int s = 0; s < sweeps[li]; ++s) {
            double worst = 0.0;
            for (int j = 1; j + 1 < L.nz; ++j)
                for (int i = 1; i + 1 < L.nx; ++i) {
                    const std::size_t k = static_cast<std::size_t>(j) * L.nx + i;
                    if (L.fixed[k]) continue;
                    const double v = w * (L.y[k - 1] + L.y[k + 1] + L.y[k - L.nx] + L.y[k + L.nx]);
                    worst = std::max(worst, std::fabs(v - L.y[k]));
                    L.y[k] = v;
                }
            lastResidual = worst;
        }
        prev = std::move(L);
        havePrev = true;
    }

    auto grid = std::make_shared<DisplacementGrid>();
    grid->originX = minX; grid->originZ = minZ; grid->cell = prev.cell;
    grid->nx = prev.nx; grid->nz = prev.nz;
    grid->d.assign(prev.y.begin(), prev.y.end());
    if (statsOut) {
        EarthworkStats& st = *statsOut;
        st.cells = prev.nx * prev.nz;
        st.fixed = 0;
        for (unsigned char f : prev.fixed) st.fixed += f;
        st.maxAbsD = 0;
        for (int j = 0; j < prev.nz; ++j)
            for (int i = 0; i < prev.nx; ++i) {
                const double v = prev.y[static_cast<std::size_t>(j) * prev.nx + i];
                if (std::fabs(v) > st.maxAbsD) {
                    st.maxAbsD = std::fabs(v);
                    st.maxAbsDX = minX + i * prev.cell;
                    st.maxAbsDZ = minZ + j * prev.cell;
                }
            }
        st.residual = lastResidual;
        st.cell = prev.cell;
        st.minX = minX; st.minZ = minZ; st.maxX = maxX; st.maxZ = maxZ;
    }
    return std::make_shared<const std::function<double(double, double)>>(
        [grid](double x, double z) { return grid->at(x, z); });
}

}  // namespace engine
