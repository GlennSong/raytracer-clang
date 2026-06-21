#include "terrain_field.h"

#include "noise.h"
#include "erosion.h"
#include "terrain.h"          // TerrainFlatten, applyFlatten
#include "../mesh_builder.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <queue>

namespace engine {

std::vector<FlatSite> findFlatSites(const HeightField& h, const FlatSiteParams& p) {
    const double cell = std::max(1.0, p.cell);
    const int n = std::max(2, static_cast<int>(std::lround(2 * p.region / cell)));
    const double ox = p.center.x - p.region, oz = p.center.z - p.region;
    auto X = [&](int i) { return ox + (i + 0.5) * cell; };
    auto Z = [&](int j) { return oz + (j + 0.5) * cell; };
    auto idx = [&](int i, int j) { return j * n + i; };

    // 1. Sample the heightmap "bitmap" and mark each cell buildable: gentle slope
    //    (central-difference gradient) and below the mountain cutoff. The border
    //    ring is forced blocked so a region's disc can't run off the search area.
    std::vector<double> H(static_cast<std::size_t>(n) * n);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) H[idx(i, j)] = h(X(i), Z(j));
    std::vector<char> build(static_cast<std::size_t>(n) * n, 0);
    for (int j = 1; j < n - 1; ++j)
        for (int i = 1; i < n - 1; ++i) {
            double gx = (H[idx(i + 1, j)] - H[idx(i - 1, j)]) / (2 * cell);
            double gz = (H[idx(i, j + 1)] - H[idx(i, j - 1)]) / (2 * cell);
            double slope = std::sqrt(gx * gx + gz * gz);
            build[idx(i, j)] = (slope <= p.maxSlope && H[idx(i, j)] <= p.maxHeight) ? 1 : 0;
        }

    // 2. Distance transform: cells of each buildable cell to the nearest BLOCKED
    //    cell (multi-source BFS, in cell units). Its max over a region is the
    //    radius of the largest flat disc that fits — the inscribed circle.
    const int INF = n * n + 1;
    std::vector<int> dist(static_cast<std::size_t>(n) * n, INF);
    std::queue<int> q;
    for (int c = 0; c < n * n; ++c)
        if (!build[c]) { dist[c] = 0; q.push(c); }
    const int di[4] = {1, -1, 0, 0}, dj[4] = {0, 0, 1, -1};
    while (!q.empty()) {
        int c = q.front(); q.pop();
        int ci = c % n, cj = c / n;
        for (int k = 0; k < 4; ++k) {
            int ni = ci + di[k], nj = cj + dj[k];
            if (ni < 0 || nj < 0 || ni >= n || nj >= n) continue;
            int nc = idx(ni, nj);
            if (dist[nc] > dist[c] + 1) { dist[nc] = dist[c] + 1; q.push(nc); }
        }
    }

    // 3. Flood-fill connected buildable regions; each region's site is its cell of
    //    greatest distance-to-edge (disc centre), radius = that distance * cell.
    std::vector<char> seen(static_cast<std::size_t>(n) * n, 0);
    std::vector<FlatSite> sites;
    for (int start = 0; start < n * n; ++start) {
        if (seen[start] || !build[start]) continue;
        int best = start;
        std::queue<int> fq;
        fq.push(start); seen[start] = 1;
        while (!fq.empty()) {
            int c = fq.front(); fq.pop();
            if (dist[c] > dist[best]) best = c;
            int ci = c % n, cj = c / n;
            for (int k = 0; k < 4; ++k) {
                int ni = ci + di[k], nj = cj + dj[k];
                if (ni < 0 || nj < 0 || ni >= n || nj >= n) continue;
                int nc = idx(ni, nj);
                if (!seen[nc] && build[nc]) { seen[nc] = 1; fq.push(nc); }
            }
        }
        double radius = dist[best] * cell;
        if (radius >= p.minRadius)
            sites.push_back({X(best % n), Z(best / n), radius});
    }

    // 4. Largest disc first; drop sites huddled too close to a bigger one kept.
    std::sort(sites.begin(), sites.end(),
              [](const FlatSite& a, const FlatSite& b) { return a.radius > b.radius; });
    std::vector<FlatSite> kept;
    for (const FlatSite& s : sites) {
        bool tooClose = false;
        for (const FlatSite& k : kept) {
            double d = std::hypot(s.cx - k.cx, s.cz - k.cz);
            if (d < p.minSeparation) { tooClose = true; break; }
        }
        if (!tooClose) kept.push_back(s);
        if (static_cast<int>(kept.size()) >= std::max(1, p.count)) break;
    }
    return kept;
}

HeightField heightConstant(double h) {
    return [h](double, double) { return h; };
}

HeightField heightNoise(uint32_t seed, double freq, double amp) {
    auto n = std::make_shared<Noise>(seed);
    return [n, freq, amp](double x, double z) {
        return n->noise2(x * freq, z * freq) * amp;
    };
}

HeightField heightFbm(uint32_t seed, double freq, double amp, int octaves) {
    auto n = std::make_shared<Noise>(seed);
    int oc = std::max(1, octaves);
    return [n, freq, amp, oc](double x, double z) {
        return n->fbm2(x * freq, z * freq, oc) * amp;
    };
}

HeightField heightRidged(uint32_t seed, double freq, double amp, int octaves) {
    auto n = std::make_shared<Noise>(seed);
    int oc = std::max(1, octaves);
    return [n, freq, amp, oc](double x, double z) {
        double sum = 0, f = freq, a = amp;
        for (int i = 0; i < oc; ++i) {
            double v = 1.0 - std::fabs(n->noise2(x * f, z * f));
            sum += v * v * a;
            f *= 2.0; a *= 0.5;
        }
        return sum;
    };
}

HeightField heightWarp(HeightField base, HeightField by, double strength) {
    return [base, by, strength](double x, double z) {
        double ox = by(x, z) * strength;
        double oz = by(x + 5.2, z - 1.3) * strength;   // decorrelated offset
        return base(x + ox, z + oz);
    };
}

HeightField heightTerrace(HeightField base, double step) {
    double s = step > 1e-6 ? step : 1.0;
    return [base, s](double x, double z) {
        return std::round(base(x, z) / s) * s;
    };
}

HeightField heightAdd(HeightField a, HeightField b) {
    return [a, b](double x, double z) { return a(x, z) + b(x, z); };
}
HeightField heightMul(HeightField a, HeightField b) {
    return [a, b](double x, double z) { return a(x, z) * b(x, z); };
}
HeightField heightScale(HeightField a, double s) {
    return [a, s](double x, double z) { return a(x, z) * s; };
}
HeightField heightMax(HeightField a, HeightField b) {
    return [a, b](double x, double z) { return std::max(a(x, z), b(x, z)); };
}
HeightField heightMin(HeightField a, HeightField b) {
    return [a, b](double x, double z) { return std::min(a(x, z), b(x, z)); };
}
HeightField heightMix(HeightField a, HeightField b, double t) {
    return [a, b, t](double x, double z) {
        double u = a(x, z), v = b(x, z);
        return u + (v - u) * t;
    };
}
HeightField heightClamp(HeightField a, double lo, double hi) {
    return [a, lo, hi](double x, double z) {
        double v = a(x, z);
        return v < lo ? lo : (v > hi ? hi : v);
    };
}

HeightField erodeField(const HeightField& f, double worldSize, int resolution,
                       const ErosionParams& params) {
    int n = std::max(2, resolution) + 1;
    auto hm = std::make_shared<Heightmap>();
    hm->n = n;
    hm->worldSize = static_cast<float>(worldSize);
    hm->h.resize(static_cast<std::size_t>(n) * n);
    double half = worldSize * 0.5;
    double step = worldSize / (n - 1);
    for (int z = 0; z < n; ++z)
        for (int x = 0; x < n; ++x)            // same index convention as bakeHeightmap
            hm->set(x, z, static_cast<float>(f(-half + x * step, -half + z * step)));
    erode(*hm, params);
    return [hm](double x, double z) {
        return static_cast<double>(
            hm->sampleWorld(static_cast<float>(x), static_cast<float>(z)));
    };
}

HeightField conformField(HeightField base, std::vector<TerrainFlatten> regions) {
    // Share the regions into the closure (a closure is copied as the field is
    // passed around; the footprint list can be large for a whole road network).
    auto regs = std::make_shared<std::vector<TerrainFlatten>>(std::move(regions));
    HeightField b = std::move(base);
    return [b, regs](double x, double z) {
        return applyFlatten(*regs, x, z, b(x, z));
    };
}

RenderMesh bakeHeightMesh(const HeightField& h, double size, int resolution,
                          const Vec3& color) {
    RenderMesh mesh;
    int res = std::max(1, resolution);
    int n = res + 1;
    double half = size * 0.5;
    double cell = size / res;
    double e = cell;   // finite-difference step for normals

    mesh.vertices.reserve(static_cast<std::size_t>(n) * n);
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            double x = -half + i * cell;
            double z = -half + j * cell;
            double y = h(x, z);
            // Normal from the height gradient (central differences).
            double hl = h(x - e, z), hr = h(x + e, z);
            double hd = h(x, z - e), hu = h(x, z + e);
            Vec3 nrm = normalize(Vec3(hl - hr, 2.0 * e, hd - hu));
            Vec3 tan = normalize(Vec3(2.0 * e, hr - hl, 0.0));
            Vertex v(Vec3(x, y, z), nrm, tan, (x + half) / size, (z + half) / size);
            v.color = color;
            mesh.vertices.push_back(v);
        }
    }
    // Clockwise-front winding for an up-facing grid (the engine's convention —
    // see MeshBuilder::gridIndices). The previous hand-rolled order here was
    // inverted, so the terrain top was culled as a back face in the viewer.
    MeshBuilder::gridIndices(mesh, n, n);
    return mesh;
}

}  // namespace engine
