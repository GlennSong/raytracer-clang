#include "terrain_field.h"
#include "city/polyline.h"   // douglasPeucker, distToLine

#include "noise.h"
#include "terrain.h"          // TerrainFlatten, applyFlatten
#include "../mesh_builder.h"
#include "../../curve.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <utility>

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


std::vector<Vec3> smoothCentripetalCatmullRom(const std::vector<Vec3>& in, bool closed,
                                              double chordTol, double decimateTol) {
    // 1. Optionally decimate to control points first (in XZ): a grid path is a
    //    staircase, and a spline THROUGH every stair-step just wiggles down the
    //    staircase. Collapsing to the corners first is what lets the curve sweep.
    std::vector<Vec3> ctrl;
    if (decimateTol > 0 && in.size() > 2) {
        std::vector<char> keep(in.size(), 0);
        keep.front() = keep.back() = 1;
        douglasPeucker(0, in.size() - 1, decimateTol, [&](std::size_t i, std::size_t a, std::size_t b) {   // (polyline.h: in XZ)
            return distToLine(Vec2(in[i].x, in[i].z), Vec2(in[a].x, in[a].z), Vec2(in[b].x, in[b].z));
        }, keep);
        for (std::size_t i = 0; i < in.size(); ++i) if (keep[i]) ctrl.push_back(in[i]);
    } else {
        ctrl = in;
    }
    if (ctrl.size() < 3) return ctrl;                 // nothing to round

    // 2. Centripetal Catmull-Rom through the control points (the shared Spline, ADR-0031).
    const Spline<Vec3> curve = Spline<Vec3>::centripetal(ctrl, closed);

    // 3. Sample each span. Centripetal Catmull-Rom loads its curvature near the control points
    //    (the span ENDS), so a midpoint-flatness test misses the bend entirely -- instead step
    //    along each span by arc length so the corners get enough samples to read as a curve.
    //    `chordTol` sets the spacing (smaller = finer); the per-span count grows with the span's
    //    length.
    double step = std::max(chordTol > 0 ? chordTol : 0.5, 0.25);
    std::vector<Vec3> out;
    out.push_back(curve.knots.front().value);
    for (int i = 0; i < curve.segments(); ++i) {
        double chord = (curve.knots[i + 1].value - curve.knots[i].value).length();
        int n = std::max(2, static_cast<int>(std::ceil(chord / step)));
        for (int s = 1; s <= n; ++s) out.push_back(curve.eval(i + static_cast<double>(s) / n));
    }
    return out;
}

std::vector<Vec3> roundRoadCorners(const std::vector<Vec3>& in, double minRadius,
                                   double chordTol, double decimateTol) {
    constexpr double PI = 3.14159265358979323846;
    // Decimate to control points (the corners to round).
    std::vector<Vec3> C;
    if (decimateTol > 0 && in.size() > 2) {
        std::vector<char> keep(in.size(), 0);
        keep.front() = keep.back() = 1;
        douglasPeucker(0, in.size() - 1, decimateTol, [&](std::size_t i, std::size_t a, std::size_t b) {   // (polyline.h: in XZ)
            return distToLine(Vec2(in[i].x, in[i].z), Vec2(in[a].x, in[a].z), Vec2(in[b].x, in[b].z));
        }, keep);
        for (std::size_t i = 0; i < in.size(); ++i) if (keep[i]) C.push_back(in[i]);
    } else {
        C = in;
    }
    if (C.size() < 3 || minRadius <= 0) return C;

    const int m = static_cast<int>(C.size()) - 1;     // segments 0..m-1, corners 1..m-1
    std::vector<double> segLen(m);
    for (int i = 0; i < m; ++i)
        segLen[i] = std::hypot(C[i+1].x - C[i].x, C[i+1].z - C[i].z);
    double tol = chordTol > 0 ? chordTol : 0.5;

    // Replace each corner with a circular fillet tangent to both legs. Radius =
    // minRadius, but capped to half the shorter adjacent leg so adjacent fillets
    // never overlap; where that cap bites (legs too short for a 180 hairpin) the
    // corner stays tighter than minRadius — that's the "needs a junction" case.
    std::vector<Vec3> out;
    out.push_back(C[0]);
    for (int i = 1; i < m; ++i) {
        double ax = C[i-1].x - C[i].x, az = C[i-1].z - C[i].z;   // toward previous (XZ)
        double bx = C[i+1].x - C[i].x, bz = C[i+1].z - C[i].z;   // toward next
        double la = std::hypot(ax, az), lb = std::hypot(bx, bz);
        if (la < 1e-9 || lb < 1e-9) { out.push_back(C[i]); continue; }
        ax /= la; az /= la; bx /= lb; bz /= lb;
        double beta = std::acos(std::max(-1.0, std::min(1.0, ax*bx + az*bz)));   // interior angle
        if (beta > PI - 0.05) { out.push_back(C[i]); continue; }                 // ~straight
        double halfB = 0.5 * beta, tanHalf = std::tan(halfB);
        double Tdes = tanHalf > 1e-6 ? minRadius / tanHalf : 1e18;
        double T = std::min(Tdes, 0.5 * std::min(segLen[i-1], segLen[i]));       // fit the legs
        double R = T * tanHalf;                                                  // <= minRadius if capped
        // Too tight to round without the ribbon folding: leave the apex SHARP so the
        // mesh builds a turning pad (a switchback bulb) here instead of a thin arc.
        if (R < 0.85 * minRadius) { out.push_back(C[i]); continue; }

        double fA = (segLen[i-1] - T) / segLen[i-1];
        Vec3 A(C[i].x + ax*T, C[i-1].y + (C[i].y - C[i-1].y) * fA, C[i].z + az*T);
        double fB = T / segLen[i];
        Vec3 B(C[i].x + bx*T, C[i].y + (C[i+1].y - C[i].y) * fB, C[i].z + bz*T);

        double bisx = ax + bx, bisz = az + bz, lbis = std::hypot(bisx, bisz);
        if (lbis < 1e-9) { out.push_back(A); out.push_back(B); continue; }       // 180 fold
        bisx /= lbis; bisz /= lbis;
        double dC = R / std::sin(halfB);
        double cx = C[i].x + bisx*dC, cz = C[i].z + bisz*dC;                     // arc centre

        double angA = std::atan2(A.z - cz, A.x - cx), angB = std::atan2(B.z - cz, B.x - cx);
        double dAng = angB - angA;
        while (dAng >  PI) dAng -= 2*PI;
        while (dAng < -PI) dAng += 2*PI;                                         // minor arc
        double dThetaMax = R > tol ? 2.0 * std::acos(std::max(0.0, 1.0 - tol/R)) : std::fabs(dAng);
        int segs = std::max(1, static_cast<int>(std::ceil(std::fabs(dAng) / std::max(1e-6, dThetaMax))));
        out.push_back(A);
        for (int s = 1; s <= segs; ++s) {
            double f = static_cast<double>(s) / segs, ang = angA + dAng * f;
            out.push_back(Vec3(cx + std::cos(ang)*R, A.y + (B.y - A.y)*f, cz + std::sin(ang)*R));
        }
    }
    out.push_back(C[m]);
    return out;
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
