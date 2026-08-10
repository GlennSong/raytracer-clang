#include "terrain_field.h"
#include <cstdio>
#include <cstring>
#include <filesystem>

#include "noise.h"
#include "erosion.h"
#include "terrain.h"          // TerrainFlatten, applyFlatten
#include "../mesh_builder.h"
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

namespace {
// Perpendicular distance from point p to the segment a->b, in the XZ plane.
double perpDistXZ(const Vec3& p, const Vec3& a, const Vec3& b) {
    double ex = b.x - a.x, ez = b.z - a.z;
    double len = std::sqrt(ex * ex + ez * ez);
    if (len < 1e-9) return std::hypot(p.x - a.x, p.z - a.z);
    return std::fabs((p.x - a.x) * ez - (p.z - a.z) * ex) / len;
}
// Douglas-Peucker on an XZ polyline: keep the endpoints and any vertex that sits
// more than `tol` off the straight run, so collinear cells collapse but the
// switchback corners (which lie far off their chord) survive.
void douglasPeucker(const std::vector<Vec3>& pts, int lo, int hi, double tol,
                    std::vector<char>& keep) {
    if (hi <= lo + 1) return;
    double worst = -1; int split = -1;
    for (int i = lo + 1; i < hi; ++i) {
        double d = perpDistXZ(pts[i], pts[lo], pts[hi]);
        if (d > worst) { worst = d; split = i; }
    }
    if (worst > tol && split > 0) {
        keep[split] = 1;
        douglasPeucker(pts, lo, split, tol, keep);
        douglasPeucker(pts, split, hi, tol, keep);
    }
}

// Evaluate a non-uniform Catmull-Rom span on [t1,t2] (the piece through P1->P2),
// Barry-Goldman pyramid. Centripetal knots (alpha=0.5) avoid the cusps/overshoot
// uniform Catmull-Rom produces on the irregular point spacing a grid path gives.
Vec3 catmullRomAt(const Vec3& P0, const Vec3& P1, const Vec3& P2, const Vec3& P3,
                  double t0, double t1, double t2, double t3, double t) {
    auto mix = [&](const Vec3& a, const Vec3& b, double ta, double tb) {
        double w = (tb - ta) < 1e-12 ? 0.0 : (t - ta) / (tb - ta);
        return a * (1.0 - w) + b * w;
    };
    Vec3 A1 = mix(P0, P1, t0, t1), A2 = mix(P1, P2, t1, t2), A3 = mix(P2, P3, t2, t3);
    Vec3 B1 = mix(A1, A2, t0, t2), B2 = mix(A2, A3, t1, t3);
    return mix(B1, B2, t1, t2);
}

// Earthwork-aware routing (ADR-0047, after Galin et al. 2010). The road is no longer
// glued to the ground: the search runs over (x, z, road-elevation). The grade limit
// is a HARD constraint on the ROAD profile (always walkable, even across ground too
// steep to hug), while cut/fill |road - terrain| is a SOFT cost (weight p.cutFill).
// So the optimiser trades three things in one shortest path — horizontal length,
// turning, and earthwork — and p.cutFill slides the result from "hug & switchback"
// (dear earthwork) to "straight cut & fill" (cheap earthwork).
std::vector<Vec3> routeRoadEarthwork(const HeightField& h, const Vec3& from,
                                     const Vec3& to, const RouteParams& p) {
    const double straight = std::hypot(to.x - from.x, to.z - from.z);
    double cell = std::max(1.0, p.cell);
    double pad = p.pad > 0 ? p.pad : std::max(4 * cell, straight * 0.75);
    double minX = std::min(from.x, to.x) - pad, maxX = std::max(from.x, to.x) + pad;
    double minZ = std::min(from.z, to.z) - pad, maxZ = std::max(from.z, to.z) + pad;
    const long maxCells = 120000;
    long nx = 0, nz = 0;
    for (;;) {
        nx = static_cast<long>(std::ceil((maxX - minX) / cell)) + 1;
        nz = static_cast<long>(std::ceil((maxZ - minZ) / cell)) + 1;
        if (nx * nz <= maxCells || cell > straight) break;
        cell *= 1.5;
    }
    if (nx < 2 || nz < 2)
        return {Vec3(from.x, h(from.x, from.z), from.z), Vec3(to.x, h(to.x, to.z), to.z)};
    const int NX = static_cast<int>(nx), NZ = static_cast<int>(nz), NC = NX * NZ;
    auto X = [&](int i) { return minX + i * cell; };
    auto Z = [&](int j) { return minZ + j * cell; };
    auto cidx = [&](int i, int j) { return j * NX + i; };

    // Terrain height per cell, plus the elevation band the road profile lives in.
    std::vector<double> ter(static_cast<std::size_t>(NC));
    double tMin = 1e300, tMax = -1e300;
    for (int j = 0; j < NZ; ++j)
        for (int i = 0; i < NX; ++i) {
            double t = h(X(i), Z(j));
            ter[cidx(i, j)] = t; tMin = std::min(tMin, t); tMax = std::max(tMax, t);
        }
    double elevStep = p.elevStep > 0 ? p.elevStep : std::max(0.25, p.maxGrade * cell * 0.5);
    elevStep = std::min(elevStep, std::max(0.25, p.maxGrade * cell * 0.9));  // climbable per cell
    int nLev = static_cast<int>(std::ceil((tMax - tMin) / std::max(1e-6, elevStep))) + 1;
    while (static_cast<long>(NC) * nLev > 3000000 && nLev > 2) {
        elevStep *= 1.5; nLev = static_cast<int>(std::ceil((tMax - tMin) / elevStep)) + 1;
    }
    auto elevOf = [&](int L) { return tMin + L * elevStep; };
    auto levOf = [&](double y) {
        return std::max(0, std::min(nLev - 1, static_cast<int>(std::lround((y - tMin) / elevStep))));
    };
    auto sidx = [&](int c, int L) { return static_cast<long>(c) * nLev + L; };
    const long NS = static_cast<long>(NC) * nLev;

    auto snap = [&](const Vec3& v, int& i, int& j) {
        i = std::max(0, std::min(NX - 1, static_cast<int>(std::lround((v.x - minX) / cell))));
        j = std::max(0, std::min(NZ - 1, static_cast<int>(std::lround((v.z - minZ) / cell))));
    };
    int si, sj, gi, gj; snap(from, si, sj); snap(to, gi, gj);
    int sc = cidx(si, sj), gc = cidx(gi, gj);
    const long startS = sidx(sc, levOf(ter[sc]));       // endpoints tie into the ground
    const long goalS  = sidx(gc, levOf(ter[gc]));
    auto heur = [&](int c) { return std::hypot(X(c % NX) - X(gi), Z(c / NX) - Z(gj)); };

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> g(static_cast<std::size_t>(NS), INF);
    std::vector<long> came(static_cast<std::size_t>(NS), -1);
    std::vector<signed char> dirIn(static_cast<std::size_t>(NS), -1);
    using QN = std::pair<double, long>;
    std::priority_queue<QN, std::vector<QN>, std::greater<QN>> open;
    g[startS] = 0; open.push({heur(sc), startS});
    const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    bool found = false;
    while (!open.empty()) {
        QN top = open.top(); open.pop();
        long s = top.second; int c = static_cast<int>(s / nLev), L = static_cast<int>(s % nLev);
        if (top.first > g[s] + heur(c) + 1e-9) continue;          // stale
        if (s == goalS) { found = true; break; }
        int ci = c % NX, cj = c / NX; double y = elevOf(L);
        for (int k = 0; k < 8; ++k) {
            int ni = ci + di[k], nj = cj + dj[k];
            if (ni < 0 || nj < 0 || ni >= NX || nj >= NZ) continue;
            int nc = cidx(ni, nj);
            double horiz = (di[k] && dj[k]) ? cell * 1.41421356 : cell;
            double maxdy = p.maxGrade * horiz;
            int dLmax = static_cast<int>(std::floor(maxdy / elevStep + 1e-9));
            for (int dL = -dLmax; dL <= dLmax; ++dL) {           // road-profile grade is hard
                int nL = L + dL; if (nL < 0 || nL >= nLev) continue;
                double ny = elevOf(nL);
                double dev = std::fabs(ny - ter[nc]);            // cut/fill volume proxy
                double step = horiz + p.cutFill * dev * horiz + p.climbCost * std::max(0.0, ny - y);
                if (dirIn[s] >= 0 && dirIn[s] != k) step += p.turnPenalty;
                long ns = sidx(nc, nL);
                double ng = g[s] + step;
                if (ng < g[ns]) {
                    g[ns] = ng; came[ns] = s; dirIn[ns] = static_cast<signed char>(k);
                    open.push({ng + heur(nc), ns});
                }
            }
        }
    }
    if (!found)
        return {Vec3(from.x, h(from.x, from.z), from.z), Vec3(to.x, h(to.x, to.z), to.z)};

    std::vector<Vec3> raw;
    for (long s = goalS; s != -1; s = came[s]) {
        int c = static_cast<int>(s / nLev), L = static_cast<int>(s % nLev);
        raw.push_back(Vec3(X(c % NX), elevOf(L), Z(c / NX)));    // y = ROAD elevation
    }
    std::reverse(raw.begin(), raw.end());
    // Pin the exact endpoints in plan, but seat them at the road's own elevation at
    // the adjacent node (the start/goal levels were chosen at ground height, so this
    // meets the terrain) — a flat tie-in, never a steep jump from a non-grid endpoint.
    if (raw.size() >= 2) {
        raw.front() = Vec3(from.x, raw[1].y, from.z);
        raw.back()  = Vec3(to.x, raw[raw.size() - 2].y, to.z);
    }

    // Collapse only EXACTLY collinear runs (in 3D): unlike the XZ Douglas-Peucker the
    // hug path uses, this is lossless, so the road's grade-limited profile is never
    // distorted into an illegal slope by the simplification (a plan-only simplifier
    // would merge across a vertical bend and fake a steep segment).
    std::vector<Vec3> path;
    path.push_back(raw.front());
    for (std::size_t i = 1; i + 1 < raw.size(); ++i) {
        const Vec3& a = path.back(); const Vec3& b = raw[i]; const Vec3& c = raw[i + 1];
        double ax = b.x - a.x, ay = b.y - a.y, az = b.z - a.z;
        double bx = c.x - b.x, by = c.y - b.y, bz = c.z - b.z;
        double la = std::sqrt(ax*ax + ay*ay + az*az), lb = std::sqrt(bx*bx + by*by + bz*bz);
        if (la < 1e-9) continue;
        if (lb < 1e-9) { path.push_back(b); continue; }
        double cosang = (ax*bx + ay*by + az*bz) / (la * lb);
        if (cosang < 1 - 1e-6) path.push_back(b);               // a real corner: keep it
    }
    path.push_back(raw.back());
    return path;
}

}  // namespace

std::vector<Vec3> routeRoad(const HeightField& h, const Vec3& from, const Vec3& to,
                            const RouteParams& p) {
    if (p.cutFill >= 0.0) return routeRoadEarthwork(h, from, to, p);   // hug <-> cut dial
    const double straight = std::hypot(to.x - from.x, to.z - from.z);
    // Trivial: coincident endpoints, or endpoints already within one cell.
    if (straight <= std::max(1.0, p.cell)) {
        return {Vec3(from.x, h(from.x, from.z), from.z),
                Vec3(to.x, h(to.x, to.z), to.z)};
    }
    double cell = std::max(1.0, p.cell);
    // Search box: the endpoints' AABB, widened so the path has room to bend around
    // a peak or fan out a switchback. Then cap the grid so a long route can't blow
    // up the cell count — coarsen the cell instead.
    double pad = p.pad > 0 ? p.pad : std::max(4 * cell, straight * 0.75);
    double minX = std::min(from.x, to.x) - pad, maxX = std::max(from.x, to.x) + pad;
    double minZ = std::min(from.z, to.z) - pad, maxZ = std::max(from.z, to.z) + pad;
    const long maxCells = 600000;
    long nx = 0, nz = 0;
    for (;;) {
        nx = static_cast<long>(std::ceil((maxX - minX) / cell)) + 1;
        nz = static_cast<long>(std::ceil((maxZ - minZ) / cell)) + 1;
        if (nx * nz <= maxCells || cell > straight) break;
        cell *= 1.5;
    }
    if (nx < 2 || nz < 2) {
        return {Vec3(from.x, h(from.x, from.z), from.z),
                Vec3(to.x, h(to.x, to.z), to.z)};
    }
    const int N = static_cast<int>(nx * nz);
    auto idx = [&](int i, int j) { return j * static_cast<int>(nx) + i; };
    auto X = [&](int i) { return minX + i * cell; };
    auto Z = [&](int j) { return minZ + j * cell; };

    // Lazily cache the terrain height per grid point (A* only visits a fraction).
    std::vector<double> hc(static_cast<std::size_t>(N));
    std::vector<char> hv(static_cast<std::size_t>(N), 0);
    auto H = [&](int c, int i, int j) {
        if (!hv[c]) { hc[c] = h(X(i), Z(j)); hv[c] = 1; }
        return hc[c];
    };

    auto snap = [&](const Vec3& v, int& i, int& j) {
        i = std::min(static_cast<int>(nx) - 1,
                     std::max(0, static_cast<int>(std::lround((v.x - minX) / cell))));
        j = std::min(static_cast<int>(nz) - 1,
                     std::max(0, static_cast<int>(std::lround((v.z - minZ) / cell))));
    };
    int si, sj, gi, gj;
    snap(from, si, sj);
    snap(to, gi, gj);
    const int start = idx(si, sj), goal = idx(gi, gj);
    auto heur = [&](int i, int j) {
        return std::hypot(X(i) - X(gi), Z(j) - Z(gj));   // admissible (<= true cost)
    };

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> g(static_cast<std::size_t>(N), INF);
    std::vector<int> came(static_cast<std::size_t>(N), -1);
    std::vector<signed char> dirIn(static_cast<std::size_t>(N), -1);
    using QN = std::pair<double, int>;                   // (f-score, cell)
    std::priority_queue<QN, std::vector<QN>, std::greater<QN>> open;
    g[start] = 0.0;
    open.push({heur(si, sj), start});
    const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const double budget = straight * std::max(1.0, p.maxStretch);

    bool found = false;
    while (!open.empty()) {
        QN top = open.top(); open.pop();
        int c = top.second;
        if (top.first > g[c] + heur(c % static_cast<int>(nx), c / static_cast<int>(nx)) + 1e-9)
            continue;                                    // stale queue entry
        if (c == goal) { found = true; break; }
        int ci = c % static_cast<int>(nx), cj = c / static_cast<int>(nx);
        double hcur = H(c, ci, cj);
        for (int k = 0; k < 8; ++k) {
            int ni = ci + di[k], nj = cj + dj[k];
            if (ni < 0 || nj < 0 || ni >= nx || nj >= nz) continue;
            int nc = idx(ni, nj);
            double horiz = (di[k] && dj[k]) ? cell * 1.41421356 : cell;
            double dh = H(nc, ni, nj) - hcur;
            if (std::fabs(dh) > p.maxGrade * horiz) continue;   // too steep — forbidden
            double step = horiz + p.climbCost * std::max(0.0, dh);
            if (dirIn[c] >= 0 && dirIn[c] != k) step += p.turnPenalty;
            double ng = g[c] + step;
            if (ng > budget) continue;                   // detoured too far — prune
            if (ng < g[nc]) {
                g[nc] = ng;
                came[nc] = c;
                dirIn[nc] = static_cast<signed char>(k);
                open.push({ng + heur(ni, nj), nc});
            }
        }
    }
    if (!found) return {};                               // no grade-legal route in the box

    // Reconstruct the cell path (goal -> start), reverse, and pin the exact
    // endpoints so the road meets its junctions precisely.
    std::vector<Vec3> raw;
    for (int c = goal; c != -1; c = came[c]) {
        int ci = c % static_cast<int>(nx), cj = c / static_cast<int>(nx);
        raw.push_back(Vec3(X(ci), 0, Z(cj)));
    }
    std::reverse(raw.begin(), raw.end());
    raw.front() = Vec3(from.x, 0, from.z);
    raw.back() = Vec3(to.x, 0, to.z);

    // Collapse the staircase to its corners, then seat every waypoint on the ground.
    double tol = p.simplifyTol > 0 ? p.simplifyTol : cell * 0.4;
    std::vector<char> keep(raw.size(), 0);
    keep.front() = keep.back() = 1;
    douglasPeucker(raw, 0, static_cast<int>(raw.size()) - 1, tol, keep);
    std::vector<Vec3> path;
    for (std::size_t i = 0; i < raw.size(); ++i)
        if (keep[i]) path.push_back(Vec3(raw[i].x, h(raw[i].x, raw[i].z), raw[i].z));
    return path;
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
        douglasPeucker(in, 0, static_cast<int>(in.size()) - 1, decimateTol, keep);
        for (std::size_t i = 0; i < in.size(); ++i) if (keep[i]) ctrl.push_back(in[i]);
    } else {
        ctrl = in;
    }
    if (ctrl.size() < 3) return ctrl;                 // nothing to round

    // 2. Pad with phantom control points so the curve passes through the real ends
    //    (open) or wraps seamlessly (closed — a ring is just a closed path).
    std::vector<Vec3> P;
    if (closed) {
        P.push_back(ctrl.back());
        P.insert(P.end(), ctrl.begin(), ctrl.end());
        P.push_back(ctrl[0]);
        P.push_back(ctrl[1]);
    } else {
        P.push_back(ctrl[0] + (ctrl[0] - ctrl[1]));   // mirror -> tangent at the end
        P.insert(P.end(), ctrl.begin(), ctrl.end());
        P.push_back(ctrl.back() + (ctrl.back() - ctrl[ctrl.size() - 2]));
    }

    // 3. Centripetal knots (alpha = 0.5): t_{i+1} = t_i + |P_{i+1}-P_i|^0.5.
    std::vector<double> t(P.size(), 0.0);
    for (std::size_t i = 1; i < P.size(); ++i) {
        double dx = P[i].x - P[i-1].x, dy = P[i].y - P[i-1].y, dz = P[i].z - P[i-1].z;
        double d = std::sqrt(std::sqrt(dx*dx + dy*dy + dz*dz));   // ^0.5
        t[i] = t[i-1] + std::max(d, 1e-6);
    }

    // 4. Sample each span [P_i..P_{i+1}]. Centripetal Catmull-Rom loads its curvature
    //    near the control points (the span ENDS), so a midpoint-flatness test misses
    //    the bend entirely — instead step along each span by arc length so the corners
    //    get enough samples to read as a curve. `chordTol` sets the spacing (smaller =
    //    finer); the per-span count grows with the span's length.
    double step = std::max(chordTol > 0 ? chordTol : 0.5, 0.25);
    std::vector<Vec3> out;
    out.push_back(P[1]);
    std::size_t last = P.size() - 2;                  // last real span ends at P[last]
    for (std::size_t i = 1; i < last; ++i) {
        const Vec3 &P0 = P[i-1], &P1 = P[i], &P2 = P[i+1], &P3 = P[i+2];
        double chord = std::sqrt((P2.x-P1.x)*(P2.x-P1.x) + (P2.y-P1.y)*(P2.y-P1.y) +
                                 (P2.z-P1.z)*(P2.z-P1.z));
        int n = std::max(2, static_cast<int>(std::ceil(chord / step)));
        for (int s = 1; s <= n; ++s) {
            double tt = t[i] + (t[i+1] - t[i]) * (static_cast<double>(s) / n);
            out.push_back(catmullRomAt(P0, P1, P2, P3, t[i-1], t[i], t[i+1], t[i+2], tt));
        }
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
        douglasPeucker(in, 0, static_cast<int>(in.size()) - 1, decimateTol, keep);
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

HeightField erodeField(const HeightField& f, double worldSize, int resolution,
                       const ErosionParams& params) {
    int n = std::max(2, resolution) + 1;
    auto hm = std::make_shared<Heightmap>();
    hm->n = n;
    hm->worldSize = static_cast<float>(worldSize);
    hm->h.resize(static_cast<std::size_t>(n) * n);
    double half = worldSize * 0.5;
    double step = worldSize / (n - 1);

    // CONTENT-HASH CACHE (metropolis-scale-plan P4.1): the droplet sim is the
    // single biggest load cost and is DETERMINISTIC in (input field, size, res,
    // erosion params). The key hashes the INPUT FIELD BY SAMPLING it — any
    // terrain knob or seed change alters sampled heights, so invalidation is
    // automatic and no parameter enumeration can go stale. RT_NOCACHE=1 skips.
    const bool useCache = std::getenv("RT_NOCACHE") == nullptr;
    std::uint64_t key = 1469598103934665603ULL;   // FNV-1a
    auto fold = [&](double v) {
        std::uint64_t bits;
        std::memcpy(&bits, &v, sizeof bits);
        key ^= bits;
        key *= 1099511628211ULL;
    };
    if (useCache) {
        // MULTI-SCALE probe: a sparse pseudo-random pattern alone can miss a
        // localized feature entirely (a +/-1.4 km ridge network in an 8 km
        // domain slipped through all 64 points — the stale field then grew
        // ROADS against terrain that no longer existed). Uniform lattices at
        // full/half/quarter extent bound the largest invisible feature to
        // under a quarter of a fine cell.
        for (int i = 0; i < 64; ++i) {           // deterministic probe pattern
            const double px = -half + worldSize * ((i * 37) % 61) / 61.0;
            const double pz = -half + worldSize * ((i * 53) % 67) / 67.0;
            fold(f(px, pz));
        }
        for (int scale = 0; scale < 3; ++scale) {
            const double ext = worldSize / (1 << scale);   // full, half, quarter
            for (int j = 0; j < 16; ++j)
                for (int i = 0; i < 16; ++i) {
                    const double px = -ext * 0.5 + ext * (i + 0.5) / 16.0;
                    const double pz = -ext * 0.5 + ext * (j + 0.5) / 16.0;
                    fold(f(px, pz));
                }
        }
        fold(worldSize); fold(static_cast<double>(n));
        fold(static_cast<double>(params.droplets));
        fold(params.inertia); fold(params.sedimentCapacity);
        fold(params.depositSpeed); fold(params.erodeSpeed);
        fold(params.evaporation); fold(params.gravity); fold(params.minSlope);
        fold(static_cast<double>(params.erodeRadius));
        fold(static_cast<double>(params.thermalIterations));
        fold(params.talus); fold(params.thermalRate);
        // Backend tag (P0.5): the GPU and CPU sims are each deterministic but
        // not bit-identical to each other, so the key names which one will
        // run — CPU- and GPU-baked caches never cross-contaminate, and a
        // kernel revision (tag bump) invalidates cleanly.
        for (const char* t = erosionBackendTag(); *t; ++t) {
            key ^= static_cast<unsigned char>(*t);
            key *= 1099511628211ULL;
        }
    }
    char cachePath[256] = {0};
    if (useCache) {
        std::snprintf(cachePath, sizeof cachePath, "cache/terrain/%016llx.bin",
                      static_cast<unsigned long long>(key));
        if (std::FILE* fp = std::fopen(cachePath, "rb")) {
            int fn = 0;
            const bool ok =
                std::fread(&fn, sizeof fn, 1, fp) == 1 && fn == n &&
                std::fread(hm->h.data(), sizeof(float), hm->h.size(), fp) ==
                    hm->h.size();
            std::fclose(fp);
            if (ok)
                return [hm](double x, double z) {
                    return static_cast<double>(
                        hm->sampleWorld(static_cast<float>(x), static_cast<float>(z)));
                };
        }
    }

    for (int z = 0; z < n; ++z)
        for (int x = 0; x < n; ++x)            // same index convention as bakeHeightmap
            hm->set(x, z, static_cast<float>(f(-half + x * step, -half + z * step)));
    erode(*hm, params);
    if (useCache) {
        std::filesystem::create_directories("cache/terrain");
        if (std::FILE* fp = std::fopen(cachePath, "wb")) {
            std::fwrite(&n, sizeof n, 1, fp);
            std::fwrite(hm->h.data(), sizeof(float), hm->h.size(), fp);
            std::fclose(fp);
        }
    }
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
