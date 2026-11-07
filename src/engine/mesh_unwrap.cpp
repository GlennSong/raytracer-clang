#include "mesh_unwrap.h"

#include "mesh_builder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <queue>
#include <unordered_map>

namespace engine {

namespace {

struct P2 {
    double x = 0, y = 0;
};
P2 operator-(const P2& a, const P2& b) { return {a.x - b.x, a.y - b.y}; }
double cross2(const P2& a, const P2& b) { return a.x * b.y - a.y * b.x; }
double area2(const P2& a, const P2& b, const P2& c) { return cross2(b - a, c - a); }   // twice the signed area

bool tagged(const Vertex& v) { return v.u <= -999.0f; }

constexpr double kWeld = 1e-5;   // metres: positions this close are one point

// Every vertex's WELDED id: split vertices (a hard edge's two normals, a seam's two uvs) share their position's id.
std::vector<int> weldIds(const RenderMesh& m, int& count) {
    std::map<std::array<long long, 3>, int> ids;
    std::vector<int> out(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        const Vec3& p = m.vertices[i].position;
        const std::array<long long, 3> k{std::llround(p.x / kWeld), std::llround(p.y / kWeld), std::llround(p.z / kWeld)};
        const auto it = ids.find(k);
        if (it != ids.end()) out[i] = it->second;
        else { const int id = static_cast<int>(ids.size()); ids.emplace(k, id); out[i] = id; }
    }
    count = static_cast<int>(ids.size());
    return out;
}

struct Face {
    std::array<uint32_t, 3> v{};   // original vertex indices
    std::array<int, 3> w{};        // welded ids
    Vec3 normal;                   // unit (zero when degenerate)
    double area = 0;
    bool tag = false;
};

// An orthonormal frame round `n`: t, b with t x b = n
void frameOf(const Vec3& n, Vec3& t, Vec3& b) {
    const Vec3 a = std::fabs(n.x) < 0.6 ? Vec3(1, 0, 0) : (std::fabs(n.y) < 0.6 ? Vec3(0, 1, 0) : Vec3(0, 0, 1));
    t = normalize(cross(a, n));
    b = cross(n, t);
}

// THE FLATTENED CHART: a uv per welded id it uses, in metres
using ChartUV = std::unordered_map<int, P2>;

// Do the chart's triangles keep their winding (no fold), and does its outline cross itself nowhere? Both together:
// the flattening is one-to-one. (Planar projection within a cone keeps every triangle's winding; a chart can still
// overlap itself in the large -- a spiral -- and that crosses its outline.)
bool injective(const std::vector<Face>& faces, const std::vector<int>& chart, const ChartUV& uv) {
    for (int f : chart) {
        const Face& F = faces[static_cast<std::size_t>(f)];
        if (F.area <= 0) continue;
        if (area2(uv.at(F.w[0]), uv.at(F.w[1]), uv.at(F.w[2])) <= 0) return false;
    }
    // NO TWO TRIANGLES SHARE AREA: every pair whose boxes meet (bucketed on a grid) tested exactly -- an edge of one
    // properly crossing an edge of the other, or one's centroid strictly inside the other (containment, duplicates).
    // (An outline that never crosses itself is not enough: a small panel lying on a larger one it shares an edge with
    // overlaps it with the outline intact -- the cubicle's, the phone booth's.)
    struct T { P2 p[3]; int w[3]; double lo[2], hi[2]; };
    std::vector<T> tris;
    tris.reserve(chart.size());
    double lo[2] = {1e300, 1e300}, hi[2] = {-1e300, -1e300};
    for (int f : chart) {
        const Face& F = faces[static_cast<std::size_t>(f)];
        if (F.area <= 0) continue;
        T t;
        for (int k = 0; k < 3; ++k) { t.p[k] = uv.at(F.w[static_cast<std::size_t>(k)]); t.w[k] = F.w[static_cast<std::size_t>(k)]; }
        t.lo[0] = std::min({t.p[0].x, t.p[1].x, t.p[2].x}); t.hi[0] = std::max({t.p[0].x, t.p[1].x, t.p[2].x});
        t.lo[1] = std::min({t.p[0].y, t.p[1].y, t.p[2].y}); t.hi[1] = std::max({t.p[0].y, t.p[1].y, t.p[2].y});
        lo[0] = std::min(lo[0], t.lo[0]); lo[1] = std::min(lo[1], t.lo[1]);
        hi[0] = std::max(hi[0], t.hi[0]); hi[1] = std::max(hi[1], t.hi[1]);
        tris.push_back(t);
    }
    if (tris.size() < 2) return true;
    const int G = std::max(1, std::min(128, static_cast<int>(std::sqrt(static_cast<double>(tris.size())))));
    const double cw = std::max(1e-12, (hi[0] - lo[0]) / G), ch = std::max(1e-12, (hi[1] - lo[1]) / G);
    std::vector<std::vector<int>> cells(static_cast<std::size_t>(G * G));
    for (std::size_t i = 0; i < tris.size(); ++i) {
        const int x0 = std::clamp(static_cast<int>((tris[i].lo[0] - lo[0]) / cw), 0, G - 1);
        const int x1 = std::clamp(static_cast<int>((tris[i].hi[0] - lo[0]) / cw), 0, G - 1);
        const int y0 = std::clamp(static_cast<int>((tris[i].lo[1] - lo[1]) / ch), 0, G - 1);
        const int y1 = std::clamp(static_cast<int>((tris[i].hi[1] - lo[1]) / ch), 0, G - 1);
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) cells[static_cast<std::size_t>(y * G + x)].push_back(static_cast<int>(i));
    }
    // (the tolerance is a share of the chart's size: a shared edge's two triangles touch, they do not overlap)
    const double eps = 1e-9 * std::max(hi[0] - lo[0], hi[1] - lo[1]) * std::max(hi[0] - lo[0], hi[1] - lo[1]);
    auto properCross = [eps](const P2& a, const P2& b, const P2& c, const P2& d) {
        const double d1 = area2(a, b, c), d2 = area2(a, b, d), d3 = area2(c, d, a), d4 = area2(c, d, b);
        return ((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) && ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps));
    };
    auto strictlyIn = [eps](const P2& c, const T& t) {
        const double s = area2(t.p[0], t.p[1], t.p[2]) > 0 ? 1 : -1;
        return s * area2(t.p[0], t.p[1], c) > eps && s * area2(t.p[1], t.p[2], c) > eps && s * area2(t.p[2], t.p[0], c) > eps;
    };
    auto overlap = [&](const T& a, const T& b) {
        if (a.hi[0] <= b.lo[0] || b.hi[0] <= a.lo[0] || a.hi[1] <= b.lo[1] || b.hi[1] <= a.lo[1]) return false;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                const int a0 = a.w[i], a1 = a.w[(i + 1) % 3], b0 = b.w[j], b1 = b.w[(j + 1) % 3];
                if (a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1) continue;   // (edges meeting at a shared point)
                if (properCross(a.p[i], a.p[(i + 1) % 3], b.p[j], b.p[(j + 1) % 3])) return true;
            }
        const P2 ca{(a.p[0].x + a.p[1].x + a.p[2].x) / 3, (a.p[0].y + a.p[1].y + a.p[2].y) / 3};
        const P2 cb{(b.p[0].x + b.p[1].x + b.p[2].x) / 3, (b.p[0].y + b.p[1].y + b.p[2].y) / 3};
        return strictlyIn(ca, b) || strictlyIn(cb, a);
    };
    for (const std::vector<int>& cell : cells)
        for (std::size_t i = 0; i < cell.size(); ++i)
            for (std::size_t j = i + 1; j < cell.size(); ++j)
                if (overlap(tris[static_cast<std::size_t>(cell[i])], tris[static_cast<std::size_t>(cell[j])])) return false;
    return true;
}

// PLANAR: every point on the chart's plane (square to `n`)
ChartUV projectChart(const std::vector<Face>& faces, const std::vector<int>& chart, const RenderMesh& m,
                     const std::vector<int>& weldOfVertex, const Vec3& n) {
    Vec3 t, b;
    frameOf(n, t, b);
    ChartUV uv;
    for (int f : chart)
        for (int k = 0; k < 3; ++k) {
            const Face& F = faces[static_cast<std::size_t>(f)];
            const int w = F.w[static_cast<std::size_t>(k)];
            if (uv.count(w)) continue;
            const Vec3& p = m.vertices[F.v[static_cast<std::size_t>(k)]].position;
            uv[w] = {dot(p, t), dot(p, b)};
        }
    (void)weldOfVertex;
    return uv;
}

// LSCM (Lévy et al. 2002): the conformal energy over the chart's triangles, two points pinned where the projection put
// them, the rest by conjugate gradients on the normal equations from the projection as a start. False when it does
// not settle.
bool lscmChart(const std::vector<Face>& faces, const std::vector<int>& chart, const RenderMesh& m, ChartUV& uv) {
    std::unordered_map<int, int> var;   // welded id -> variable index
    std::vector<int> ids;
    for (int f : chart)
        for (int k = 0; k < 3; ++k) {
            const int w = faces[static_cast<std::size_t>(f)].w[static_cast<std::size_t>(k)];
            if (var.emplace(w, static_cast<int>(ids.size())).second) ids.push_back(w);
        }
    const int n = static_cast<int>(ids.size());
    if (n < 3) return false;
    // the pins: the two points furthest apart in the projection (along its longer axis)
    int pinA = 0, pinB = 0;
    {
        double lo = 1e300, hi = -1e300, loY = 1e300, hiY = -1e300;
        int a = 0, b = 0, ay = 0, by = 0;
        for (int i = 0; i < n; ++i) {
            const P2& p = uv[ids[static_cast<std::size_t>(i)]];
            if (p.x < lo) { lo = p.x; a = i; }
            if (p.x > hi) { hi = p.x; b = i; }
            if (p.y < loY) { loY = p.y; ay = i; }
            if (p.y > hiY) { hiY = p.y; by = i; }
        }
        if (hi - lo >= hiY - loY) { pinA = a; pinB = b; } else { pinA = ay; pinB = by; }
        if (pinA == pinB) return false;
    }
    // ROWS: two per triangle (the real and imaginary parts of sum_j W_j (u_j + i v_j) / sqrt(2 area)), six entries
    struct Entry { int col; double val; };
    std::vector<std::array<Entry, 6>> rows;
    rows.reserve(chart.size() * 2);
    for (int f : chart) {
        const Face& F = faces[static_cast<std::size_t>(f)];
        if (F.area <= 1e-14) continue;
        // the triangle in its own plane
        const Vec3 p0 = m.vertices[F.v[0]].position, p1 = m.vertices[F.v[1]].position, p2 = m.vertices[F.v[2]].position;
        const Vec3 ex = normalize(p1 - p0);
        const Vec3 ey = normalize(cross(F.normal, ex));
        const P2 q[3] = {{0, 0}, {dot(p1 - p0, ex), dot(p1 - p0, ey)}, {dot(p2 - p0, ex), dot(p2 - p0, ey)}};
        const double s = 1.0 / std::sqrt(2.0 * F.area);
        std::array<Entry, 6> re{}, im{};
        for (int j = 0; j < 3; ++j) {
            const P2& a = q[(j + 1) % 3];
            const P2& b = q[(j + 2) % 3];
            const double wr = (b.x - a.x) * s, wi = (b.y - a.y) * s;
            const int c = var[F.w[static_cast<std::size_t>(j)]];
            re[static_cast<std::size_t>(j * 2)] = {2 * c, wr};
            re[static_cast<std::size_t>(j * 2 + 1)] = {2 * c + 1, -wi};
            im[static_cast<std::size_t>(j * 2)] = {2 * c, wi};
            im[static_cast<std::size_t>(j * 2 + 1)] = {2 * c + 1, wr};
        }
        rows.push_back(re);
        rows.push_back(im);
    }
    if (rows.empty()) return false;
    const int N = 2 * n;
    std::vector<char> pinned(static_cast<std::size_t>(N), 0);
    for (int p : {pinA, pinB}) { pinned[static_cast<std::size_t>(2 * p)] = 1; pinned[static_cast<std::size_t>(2 * p + 1)] = 1; }
    std::vector<double> x(static_cast<std::size_t>(N));
    for (int i = 0; i < n; ++i) {
        const P2& p = uv[ids[static_cast<std::size_t>(i)]];
        x[static_cast<std::size_t>(2 * i)] = p.x;
        x[static_cast<std::size_t>(2 * i + 1)] = p.y;
    }
    // y = A^T A x over the free variables (the pinned are held: their entries fold into b)
    auto applyAtA = [&](const std::vector<double>& in, std::vector<double>& out, bool freeOnly) {
        std::fill(out.begin(), out.end(), 0.0);
        for (const auto& r : rows) {
            double dotv = 0;
            for (const Entry& e : r)
                if (!freeOnly || !pinned[static_cast<std::size_t>(e.col)]) dotv += e.val * in[static_cast<std::size_t>(e.col)];
            for (const Entry& e : r)
                if (!pinned[static_cast<std::size_t>(e.col)]) out[static_cast<std::size_t>(e.col)] += e.val * dotv;
        }
    };
    // b = -A^T (A x_pinned)
    std::vector<double> xp(static_cast<std::size_t>(N), 0.0), bvec(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) if (pinned[static_cast<std::size_t>(i)]) xp[static_cast<std::size_t>(i)] = x[static_cast<std::size_t>(i)];
    {
        std::fill(bvec.begin(), bvec.end(), 0.0);
        for (const auto& r : rows) {
            double dotv = 0;
            for (const Entry& e : r) dotv += e.val * xp[static_cast<std::size_t>(e.col)];
            for (const Entry& e : r)
                if (!pinned[static_cast<std::size_t>(e.col)]) bvec[static_cast<std::size_t>(e.col)] -= e.val * dotv;
        }
    }
    std::vector<double> xf(static_cast<std::size_t>(N), 0.0);
    for (int i = 0; i < N; ++i) if (!pinned[static_cast<std::size_t>(i)]) xf[static_cast<std::size_t>(i)] = x[static_cast<std::size_t>(i)];
    std::vector<double> r(static_cast<std::size_t>(N)), p(static_cast<std::size_t>(N)), Ap(static_cast<std::size_t>(N));
    applyAtA(xf, Ap, true);
    double rr = 0, bb = 0;
    for (int i = 0; i < N; ++i) {
        r[static_cast<std::size_t>(i)] = pinned[static_cast<std::size_t>(i)] ? 0.0 : bvec[static_cast<std::size_t>(i)] - Ap[static_cast<std::size_t>(i)];
        p[static_cast<std::size_t>(i)] = r[static_cast<std::size_t>(i)];
        rr += r[static_cast<std::size_t>(i)] * r[static_cast<std::size_t>(i)];
        bb += bvec[static_cast<std::size_t>(i)] * bvec[static_cast<std::size_t>(i)];
    }
    const double tol = 1e-20 + 1e-18 * bb;
    const int maxIt = std::max(200, 4 * N);
    for (int it = 0; it < maxIt && rr > tol; ++it) {
        applyAtA(p, Ap, true);
        double pAp = 0;
        for (int i = 0; i < N; ++i) pAp += p[static_cast<std::size_t>(i)] * Ap[static_cast<std::size_t>(i)];
        if (!(pAp > 0)) break;
        const double alpha = rr / pAp;
        double rr2 = 0;
        for (int i = 0; i < N; ++i) {
            xf[static_cast<std::size_t>(i)] += alpha * p[static_cast<std::size_t>(i)];
            r[static_cast<std::size_t>(i)] -= alpha * Ap[static_cast<std::size_t>(i)];
            rr2 += r[static_cast<std::size_t>(i)] * r[static_cast<std::size_t>(i)];
        }
        const double beta = rr2 / rr;
        rr = rr2;
        for (int i = 0; i < N; ++i) p[static_cast<std::size_t>(i)] = r[static_cast<std::size_t>(i)] + beta * p[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < N; ++i)
        if (!std::isfinite(xf[static_cast<std::size_t>(i)])) return false;
    for (int i = 0; i < n; ++i) {
        P2& q = uv[ids[static_cast<std::size_t>(i)]];
        if (!pinned[static_cast<std::size_t>(2 * i)]) { q.x = xf[static_cast<std::size_t>(2 * i)]; q.y = xf[static_cast<std::size_t>(2 * i + 1)]; }
    }
    return true;
}

// The chart's 3-D and uv areas
void chartAreas(const std::vector<Face>& faces, const std::vector<int>& chart, const ChartUV& uv, double& a3, double& a2) {
    a3 = a2 = 0;
    for (int f : chart) {
        const Face& F = faces[static_cast<std::size_t>(f)];
        a3 += F.area;
        a2 += 0.5 * std::fabs(area2(uv.at(F.w[0]), uv.at(F.w[1]), uv.at(F.w[2])));
    }
}

// The convex hull (Andrew's monotone chain), counter-clockwise
std::vector<P2> hull(std::vector<P2> pts) {
    std::sort(pts.begin(), pts.end(), [](const P2& a, const P2& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    if (pts.size() < 3) return pts;
    std::vector<P2> h(pts.size() * 2);
    std::size_t k = 0;
    for (const P2& p : pts) {
        while (k >= 2 && area2(h[k - 2], h[k - 1], p) <= 0) --k;
        h[k++] = p;
    }
    for (std::size_t i = pts.size() - 1, t = k + 1; i > 0; --i) {
        const P2& p = pts[i - 1];
        while (k >= t && area2(h[k - 2], h[k - 1], p) <= 0) --k;
        h[k++] = p;
    }
    h.resize(k - 1);
    return h;
}

struct Packed {
    std::vector<int> faces;
    ChartUV uv;        // metres, turned to its tightest box, its corner at 0
    double w = 0, h = 0;
    int px = 0, py = 0, tw = 0, th = 0;   // its place and size in texels, padding included
};

// SKYLINE bottom-left packing of `items` into a `side` x `side` square; false when they do not fit
bool skylinePack(std::vector<Packed*>& items, int side) {
    struct Seg { int x, y, w; };
    std::vector<Seg> sky{{0, 0, side}};
    for (Packed* it : items) {
        if (it->tw > side || it->th > side) return false;
        int bestY = INT32_MAX, bestX = INT32_MAX;
        std::size_t bestI = 0;
        for (std::size_t i = 0; i < sky.size(); ++i) {
            if (sky[i].x + it->tw > side) break;
            int y = 0, need = it->tw;
            for (std::size_t j = i; j < sky.size() && need > 0; ++j) {
                y = std::max(y, sky[j].y);
                need -= sky[j].w;
            }
            if (need > 0 || y + it->th > side) continue;
            if (y < bestY || (y == bestY && sky[i].x < bestX)) { bestY = y; bestX = sky[i].x; bestI = i; }
        }
        if (bestY == INT32_MAX) return false;
        it->px = bestX;
        it->py = bestY;
        // raise the skyline under it: the old segments clipped outside its span, its top as one segment, neighbours of
        // one height merged
        (void)bestI;
        const int t0 = bestX, t1 = bestX + it->tw;
        std::vector<Seg> next;
        for (const Seg& s : sky) {
            const int s0 = s.x, s1 = s.x + s.w;
            if (s1 <= t0 || s0 >= t1) { next.push_back(s); continue; }
            if (s0 < t0) next.push_back({s0, s.y, t0 - s0});
            if (s1 > t1) next.push_back({t1, s.y, s1 - t1});
        }
        next.push_back({t0, bestY + it->th, it->tw});
        std::sort(next.begin(), next.end(), [](const Seg& a, const Seg& b) { return a.x < b.x; });
        std::vector<Seg> merged;
        for (const Seg& s : next) {
            if (!merged.empty() && merged.back().y == s.y && merged.back().x + merged.back().w == s.x) merged.back().w += s.w;
            else merged.push_back(s);
        }
        sky.swap(merged);
    }
    return true;
}

}  // namespace

UnwrapReport unwrapMesh(RenderMesh& mesh, const UnwrapOptions& opts) {
    UnwrapReport rep;
    const std::size_t nf = mesh.indices.size() / 3;
    if (nf == 0) { rep.ok = true; return rep; }
    if (opts.faceGroups && opts.faceGroups->size() != nf) return rep;   // (one group a face, or none)
    int weldCount = 0;
    const std::vector<int> weld = weldIds(mesh, weldCount);
    std::vector<Face> faces(nf);
    for (std::size_t f = 0; f < nf; ++f) {
        Face& F = faces[f];
        for (int k = 0; k < 3; ++k) {
            F.v[static_cast<std::size_t>(k)] = mesh.indices[f * 3 + static_cast<std::size_t>(k)];
            F.w[static_cast<std::size_t>(k)] = weld[F.v[static_cast<std::size_t>(k)]];
        }
        const Vec3& a = mesh.vertices[F.v[0]].position;
        const Vec3& b = mesh.vertices[F.v[1]].position;
        const Vec3& c = mesh.vertices[F.v[2]].position;
        const Vec3 cr = cross(b - a, c - a);
        const double l = cr.length();
        F.area = 0.5 * l;
        F.normal = l > 1e-14 ? cr * (1.0 / l) : Vec3(0, 0, 0);
        F.tag = tagged(mesh.vertices[F.v[0]]) || tagged(mesh.vertices[F.v[1]]) || tagged(mesh.vertices[F.v[2]]);
        if (F.tag) ++rep.tagged;
        // a face through a welded point twice has no area to lay out
        if (F.w[0] == F.w[1] || F.w[1] == F.w[2] || F.w[0] == F.w[2]) { F.area = 0; F.normal = Vec3(0, 0, 0); }
    }
    // the faces round each welded edge
    std::map<std::pair<int, int>, std::vector<int>> edgeFaces;
    for (std::size_t f = 0; f < nf; ++f)
        for (int e = 0; e < 3; ++e) {
            const int a = faces[f].w[static_cast<std::size_t>(e)], b = faces[f].w[static_cast<std::size_t>((e + 1) % 3)];
            if (a != b) edgeFaces[{std::min(a, b), std::max(a, b)}].push_back(static_cast<int>(f));
        }
    auto neighbours = [&](int f, std::vector<int>& out) {
        out.clear();
        for (int e = 0; e < 3; ++e) {
            const int a = faces[static_cast<std::size_t>(f)].w[static_cast<std::size_t>(e)];
            const int b = faces[static_cast<std::size_t>(f)].w[static_cast<std::size_t>((e + 1) % 3)];
            if (a == b) continue;
            const auto it = edgeFaces.find({std::min(a, b), std::max(a, b)});
            if (it == edgeFaces.end()) continue;
            for (int g : it->second) if (g != f) out.push_back(g);
        }
    };

    // GROW CHARTS among `pool` (face indices) within `cosA` of each chart's seed; each chart's faces into `out`
    std::vector<int> owner(nf, -1);
    auto grow = [&](const std::vector<int>& pool, double cosA, std::vector<std::vector<int>>& out) {
        std::vector<int> order = pool;
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            return faces[static_cast<std::size_t>(a)].area > faces[static_cast<std::size_t>(b)].area;
        });
        std::unordered_map<int, char> inPool;
        for (int f : pool) inPool[f] = 1;
        std::unordered_map<int, char> taken;
        std::vector<int> nb;
        for (int seed : order) {
            if (taken.count(seed)) continue;
            const Vec3 axis = faces[static_cast<std::size_t>(seed)].normal;
            std::vector<int> chart{seed};
            taken[seed] = 1;
            std::queue<int> q;
            q.push(seed);
            while (!q.empty()) {
                const int f = q.front();
                q.pop();
                neighbours(f, nb);
                for (int g : nb) {
                    if (!inPool.count(g) || taken.count(g)) continue;
                    if (opts.faceGroups && (*opts.faceGroups)[static_cast<std::size_t>(g)] != (*opts.faceGroups)[static_cast<std::size_t>(seed)])
                        continue;
                    const Face& G = faces[static_cast<std::size_t>(g)];
                    const bool flat = G.area <= 0;   // no normal: it goes with whatever it touches
                    if (!flat && (axis.lengthSquared() == 0 || dot(G.normal, axis) < cosA)) continue;
                    taken[g] = 1;
                    chart.push_back(g);
                    q.push(g);
                }
            }
            out.push_back(std::move(chart));
        }
    };

    std::vector<int> all;
    for (std::size_t f = 0; f < nf; ++f) if (!faces[f].tag) all.push_back(static_cast<int>(f));
    std::vector<std::vector<int>> pending;
    grow(all, std::cos(opts.chartAngleDeg * 3.14159265358979 / 180.0), pending);

    // FLATTEN each chart; a chart that will not lay flat one-to-one is split (half the angle) and tried again
    std::vector<Packed> charts;
    std::vector<std::pair<std::vector<int>, double>> work;
    for (auto& c : pending) work.push_back({std::move(c), opts.chartAngleDeg});
    while (!work.empty()) {
        auto [chart, angle] = std::move(work.back());
        work.pop_back();
        // the chart's area-weighted normal (the seed's when it has none)
        Vec3 n(0, 0, 0);
        for (int f : chart) n = n + faces[static_cast<std::size_t>(f)].normal * faces[static_cast<std::size_t>(f)].area;
        if (n.lengthSquared() < 1e-24) n = Vec3(0, 1, 0);
        n = normalize(n);
        double worst = 1.0;
        for (int f : chart) if (faces[static_cast<std::size_t>(f)].area > 0) worst = std::min(worst, dot(faces[static_cast<std::size_t>(f)].normal, n));
        ChartUV uv = projectChart(faces, chart, mesh, weld, n);
        bool good = worst > 0.05 && injective(faces, chart, uv);
        if (good && worst < opts.lscmBelowCos && chart.size() >= 3) {
            ChartUV conformal = uv;
            if (lscmChart(faces, chart, mesh, conformal) && injective(faces, chart, conformal)) { uv.swap(conformal); ++rep.lscm; }
            else ++rep.planar;
        } else if (good) {
            ++rep.planar;
        }
        if (!good) {
            if (chart.size() == 1) {   // one face lays flat on its own plane, always
                const Face& F = faces[static_cast<std::size_t>(chart[0])];
                uv = projectChart(faces, chart, mesh, weld, F.area > 0 ? F.normal : Vec3(0, 1, 0));
                ++rep.planar;
            } else {
                ++rep.splits;
                std::vector<std::vector<int>> parts;
                const double a2 = std::max(2.0, angle * 0.5);
                grow(chart, std::cos(a2 * 3.14159265358979 / 180.0), parts);
                if (parts.size() == 1) {   // the angle cannot split it (a fold within the cone): one face at a time
                    for (int f : chart) work.push_back({{f}, a2});
                } else {
                    for (auto& p : parts) work.push_back({std::move(p), a2});
                }
                continue;
            }
        }
        // metres: the chart scaled so its uv area is its 3-D area (one density everywhere)
        double a3 = 0, auv = 0;
        chartAreas(faces, chart, uv, a3, auv);
        const double s = (auv > 1e-18 && a3 > 0) ? std::sqrt(a3 / auv) : 1.0;
        for (auto& [id, p] : uv) { p.x *= s; p.y *= s; }
        // its tightest box: the hull's edge directions tried, the least area kept
        std::vector<P2> pts;
        pts.reserve(uv.size());
        for (const auto& [id, p] : uv) pts.push_back(p);
        const std::vector<P2> h = hull(pts);
        double bestA = 1e300, bestC = 1, bestS = 0;
        for (std::size_t i = 0; i < std::max<std::size_t>(h.size(), 1); ++i) {
            double c = 1, sn = 0;
            if (h.size() >= 2) {
                const P2 e = h[(i + 1) % h.size()] - h[i];
                const double l = std::sqrt(e.x * e.x + e.y * e.y);
                if (l < 1e-15) continue;
                c = e.x / l;
                sn = e.y / l;
            }
            double lx = 1e300, hx = -1e300, ly = 1e300, hy = -1e300;
            for (const P2& p : h.empty() ? pts : h) {
                const double x = p.x * c + p.y * sn, y = -p.x * sn + p.y * c;
                lx = std::min(lx, x); hx = std::max(hx, x); ly = std::min(ly, y); hy = std::max(hy, y);
            }
            const double A = (hx - lx) * (hy - ly);
            if (A < bestA - 1e-15) { bestA = A; bestC = c; bestS = sn; }
        }
        Packed pk;
        double lx = 1e300, hx = -1e300, ly = 1e300, hy = -1e300;
        for (auto& [id, p] : uv) {
            const double x = p.x * bestC + p.y * bestS, y = -p.x * bestS + p.y * bestC;
            p = {x, y};
            lx = std::min(lx, x); hx = std::max(hx, x); ly = std::min(ly, y); hy = std::max(hy, y);
        }
        for (auto& [id, p] : uv) { p.x -= lx; p.y -= ly; }
        // the taller way up (packs better in rows)
        if (hx - lx > hy - ly) {
            for (auto& [id, p] : uv) p = {p.y, (hx - lx) - p.x};
            std::swap(lx, ly); std::swap(hx, hy);
        }
        pk.w = hx - lx;
        pk.h = hy - ly;
        pk.faces = std::move(chart);
        pk.uv = std::move(uv);
        charts.push_back(std::move(pk));
    }
    rep.charts = static_cast<int>(charts.size());

    // PACK: the density down until the charts fit the largest atlas
    double total = 0;
    for (const Packed& c : charts) total += (c.w + 1e-6) * (c.h + 1e-6);
    double tpm = opts.texelsPerMetre;
    const int pad = std::max(0, opts.padding);
    std::vector<Packed*> order;
    for (Packed& c : charts) order.push_back(&c);
    int side = 0;
    for (int attempt = 0; attempt < 64 && side == 0; ++attempt) {
        for (Packed& c : charts) {
            c.tw = static_cast<int>(std::ceil(c.w * tpm)) + 1 + 2 * pad;
            c.th = static_cast<int>(std::ceil(c.h * tpm)) + 1 + 2 * pad;
        }
        std::stable_sort(order.begin(), order.end(), [](const Packed* a, const Packed* b) {
            return a->th != b->th ? a->th > b->th : a->tw > b->tw;
        });
        double texArea = 0;
        int widest = 0;
        for (const Packed* c : order) { texArea += static_cast<double>(c->tw) * c->th; widest = std::max({widest, c->tw, c->th}); }
        int s = 1;
        while (s < widest || static_cast<double>(s) * s < texArea) s *= 2;
        for (; s <= opts.maxSize && side == 0; s *= 2)
            if (skylinePack(order, s)) side = s;
        if (side == 0) tpm *= 0.85;
    }
    if (side == 0) return rep;   // (cannot happen short of a degenerate density: rep.ok stays false)
    rep.size = side;
    rep.texelsPerMetre = tpm;

    // THE NEW VERTICES: each (chart, original vertex) its own, at its atlas place; tagged faces keep theirs
    std::vector<Vertex> verts;
    std::vector<uint32_t> idx(mesh.indices.size());
    std::map<std::pair<int, uint32_t>, uint32_t> made;
    auto vertexFor = [&](int chart, uint32_t v, const P2* at) {
        const auto key = std::make_pair(chart, v);
        const auto it = made.find(key);
        if (it != made.end()) return it->second;
        Vertex nv = mesh.vertices[v];
        if (at) { nv.u = static_cast<float>(at->x); nv.v = static_cast<float>(at->y); }
        const uint32_t id = static_cast<uint32_t>(verts.size());
        verts.push_back(nv);
        made.emplace(key, id);
        return id;
    };
    double covered = 0;
    for (std::size_t ci = 0; ci < charts.size(); ++ci) {
        const Packed& c = charts[ci];
        for (int f : c.faces) {
            const Face& F = faces[static_cast<std::size_t>(f)];
            P2 q[3];
            for (int k = 0; k < 3; ++k) {
                const P2& p = c.uv.at(F.w[static_cast<std::size_t>(k)]);
                q[k] = {(c.px + pad + 0.5 + p.x * tpm) / side, (c.py + pad + 0.5 + p.y * tpm) / side};
                idx[static_cast<std::size_t>(f) * 3 + static_cast<std::size_t>(k)] = vertexFor(static_cast<int>(ci), F.v[static_cast<std::size_t>(k)], &q[k]);
            }
            covered += 0.5 * std::fabs(area2(q[0], q[1], q[2]));
        }
    }
    for (std::size_t f = 0; f < nf; ++f)
        if (faces[f].tag)
            for (int k = 0; k < 3; ++k)
                idx[f * 3 + static_cast<std::size_t>(k)] = vertexFor(-1, faces[f].v[static_cast<std::size_t>(k)], nullptr);
    mesh.vertices.swap(verts);
    mesh.indices.swap(idx);
    // the tangents follow the atlas (a normal map baked on it is read in this frame; charts are turned, never mirrored)
    MeshBuilder::computeTangents(mesh);
    rep.fill = covered;
    rep.ok = true;
    return rep;
}

UvAudit auditUv(const RenderMesh& mesh, int res) {
    UvAudit a;
    const std::size_t nf = mesh.indices.size() / 3;
    std::vector<uint8_t> count(static_cast<std::size_t>(res) * static_cast<std::size_t>(res), 0);
    struct D { double ratio, area; };
    std::vector<D> dens;
    double sumUv = 0, sum3 = 0;
    std::vector<std::pair<double, double>> per;   // (uv area, 3-D area) per counted face
    for (std::size_t f = 0; f < nf; ++f) {
        const Vertex* v[3] = {&mesh.vertices[mesh.indices[f * 3]], &mesh.vertices[mesh.indices[f * 3 + 1]],
                              &mesh.vertices[mesh.indices[f * 3 + 2]]};
        if (tagged(*v[0]) || tagged(*v[1]) || tagged(*v[2])) continue;
        ++a.faces;
        const P2 q[3] = {{v[0]->u, v[0]->v}, {v[1]->u, v[1]->v}, {v[2]->u, v[2]->v}};
        bool out = false;
        for (const P2& p : q) if (p.x < -1e-6 || p.x > 1 + 1e-6 || p.y < -1e-6 || p.y > 1 + 1e-6) out = true;
        if (out) ++a.outside;
        const double auv = 0.5 * std::fabs(area2(q[0], q[1], q[2]));
        const double a3 = 0.5 * cross(v[1]->position - v[0]->position, v[2]->position - v[0]->position).length();
        per.push_back({auv, a3});
        sumUv += auv;
        sum3 += a3;
        // anisotropy (Sander et al. 2001): the uv-to-surface map's singular values
        const double A2 = area2(q[0], q[1], q[2]);
        if (std::fabs(A2) > 1e-18 && a3 > 1e-14) {
            const Vec3 &q1 = v[0]->position, &q2 = v[1]->position, &q3 = v[2]->position;
            const Vec3 Ss = (q1 * (q[1].y - q[2].y) + q2 * (q[2].y - q[0].y) + q3 * (q[0].y - q[1].y)) * (1.0 / A2);
            const Vec3 St = (q1 * (q[2].x - q[1].x) + q2 * (q[0].x - q[2].x) + q3 * (q[1].x - q[0].x)) * (1.0 / A2);
            const double aa = dot(Ss, Ss), bb = dot(Ss, St), cc = dot(St, St);
            const double root = std::sqrt((aa - cc) * (aa - cc) + 4 * bb * bb);
            const double big = std::sqrt(std::max(0.0, (aa + cc + root) * 0.5));
            const double small = std::sqrt(std::max(0.0, (aa + cc - root) * 0.5));
            if (small > 1e-12) a.worstStretch = std::max(a.worstStretch, big / small);
        }
        // coverage: cell centres strictly inside (a shared edge's cells go to neither: no false overlap)
        if (std::fabs(A2) < 1e-14 || out) continue;
        const double sgn = A2 > 0 ? 1 : -1;
        const double lx = std::min({q[0].x, q[1].x, q[2].x}), hx = std::max({q[0].x, q[1].x, q[2].x});
        const double ly = std::min({q[0].y, q[1].y, q[2].y}), hy = std::max({q[0].y, q[1].y, q[2].y});
        const int x0 = std::max(0, static_cast<int>(std::floor(lx * res))), x1 = std::min(res - 1, static_cast<int>(std::ceil(hx * res)));
        const int y0 = std::max(0, static_cast<int>(std::floor(ly * res))), y1 = std::min(res - 1, static_cast<int>(std::ceil(hy * res)));
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                const P2 c{(x + 0.5) / res, (y + 0.5) / res};
                if (sgn * area2(q[0], q[1], c) > 1e-12 && sgn * area2(q[1], q[2], c) > 1e-12 && sgn * area2(q[2], q[0], c) > 1e-12) {
                    uint8_t& k = count[static_cast<std::size_t>(y) * static_cast<std::size_t>(res) + static_cast<std::size_t>(x)];
                    if (k < 255) ++k;
                }
            }
    }
    long once = 0, twice = 0;
    for (uint8_t k : count) { if (k >= 1) ++once; if (k >= 2) ++twice; }
    a.overlapShare = once > 0 ? static_cast<double>(twice) / static_cast<double>(once) : 0.0;
    if (sum3 > 0 && sumUv > 0) {
        const double mean = std::sqrt(sumUv / sum3);
        for (const auto& [auv, a3] : per)
            if (a3 > 1e-14) dens.push_back({std::sqrt(auv / a3) / mean, a3});
        std::sort(dens.begin(), dens.end(), [](const D& x, const D& y) { return x.ratio < y.ratio; });
        double tot = 0;
        for (const D& d : dens) tot += d.area;
        double acc = 0;
        bool lo = false;
        for (const D& d : dens) {
            acc += d.area;
            if (!lo && acc >= 0.05 * tot) { a.densityLo = d.ratio; lo = true; }
            if (acc >= 0.95 * tot) { a.densityHi = d.ratio; break; }
        }
    }
    return a;
}

}  // namespace engine
