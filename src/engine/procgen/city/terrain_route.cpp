#include "terrain_route.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>

namespace engine {

namespace {
struct Move { int dx, dy; double len, ang; };

// every step (dx, dy) with |dx|, |dy| <= 3 and gcd 1: 32 directions, sorted by angle
std::vector<Move> movesUpTo3() {
    std::vector<Move> m;
    for (int dy = -3; dy <= 3; ++dy)
        for (int dx = -3; dx <= 3; ++dx) {
            if (dx == 0 && dy == 0) continue;
            if (std::gcd(std::abs(dx), std::abs(dy)) != 1) continue;
            m.push_back({dx, dy, std::hypot(dx, dy), std::atan2(dy, dx)});
        }
    std::sort(m.begin(), m.end(), [](const Move& a, const Move& b) { return a.ang < b.ang; });
    return m;
}

double pointSegDist(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const double L2 = dot(ab, ab);
    const double t = L2 > 1e-12 ? std::clamp(dot(p - a, ab) / L2, 0.0, 1.0) : 0.0;
    return (p - (a + ab * t)).length();
}

// Douglas-Peucker on plan position AND height together: a straight run keeps one segment, but a
// crest or a dip along it keeps its vertex (the builder needs the climb, not only the plan)
void simplify(const std::vector<Vec2>& in, const std::vector<double>& h, double tol, double htol,
              std::size_t a, std::size_t b, std::vector<char>& keep) {
    if (b <= a + 1) return;
    double worst = -1.0;
    std::size_t at = a;
    for (std::size_t k = a + 1; k < b; ++k) {
        const double d = pointSegDist(in[k], in[a], in[b]);
        const double s = std::clamp(dot(in[k] - in[a], in[b] - in[a]) / std::max(1e-9, dot(in[b] - in[a], in[b] - in[a])), 0.0, 1.0);
        const double dh = std::fabs(h[k] - (h[a] + (h[b] - h[a]) * s));
        const double e = std::max(d / tol, dh / htol);
        if (e > worst) { worst = e; at = k; }
    }
    if (worst > 1.0) {
        keep[at] = 1;
        simplify(in, h, tol, htol, a, at, keep);
        simplify(in, h, tol, htol, at, b, keep);
    }
}
}  // namespace

namespace {
// Where the route comes too close to ITSELF: two stretches more than `along` metres apart along it
// but nearer than `sep` in plan -- their pavements would overlap (or they cross outright). Sampled
// every few metres; one conflict point per offending stretch.
std::vector<Vec2> selfConflicts(const std::vector<Vec2>& pts, double sep, double along) {
    std::vector<Vec2> s;
    std::vector<double> st;
    double acc = 0.0;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const Vec2 d = pts[i + 1] - pts[i];
        const double L = d.length();
        const int n = std::max(1, static_cast<int>(std::ceil(L / 4.0)));
        for (int k = 0; k < n; ++k) { s.push_back(pts[i] + d * (static_cast<double>(k) / n)); st.push_back(acc + L * k / n); }
        acc += L;
    }
    s.push_back(pts.back()); st.push_back(acc);
    std::vector<Vec2> out;
    for (std::size_t i = 0; i < s.size(); ++i)
        for (std::size_t j = i + 1; j < s.size(); ++j) {
            if (st[j] - st[i] < along) continue;
            if ((s[i] - s[j]).length() >= sep) continue;
            const Vec2 m = (s[i] + s[j]) * 0.5;
            bool near = false;
            for (const Vec2& o : out) if ((o - m).length() < sep * 2.0) { near = true; break; }
            if (!near) out.push_back(m);
        }
    return out;
}
TerrainRoute routeOnce(const HeightField& height, const Vec2& from, const Vec2& to, const TerrainRouteParams& p);
}  // namespace

// The route may not cross or crowd ITSELF (a switchback leg within a road's width of another is a
// road on top of its own road): each conflict is blocked -- a disc about it -- and the search runs again.
TerrainRoute routeOnTerrain(const HeightField& height, const Vec2& from, const Vec2& to, const TerrainRouteParams& p) {
    TerrainRouteParams q = p;
    std::vector<Vec2> discs;
    TerrainRoute best;
    for (int round = 0; round < 6; ++round) {
        TerrainRoute r = routeOnce(height, from, to, q);
        if (r.points.empty()) return best.points.empty() ? r : best;   // no way with these discs: the last one stands
        const std::vector<Vec2> xs = selfConflicts(r.points, 20.0, 80.0);
        best = std::move(r);
        if (xs.empty()) return best;
        for (const Vec2& x : xs) discs.push_back(x);
        const auto outer = p.blocked;
        const double rad = std::max(14.0, 1.8 * p.cell);
        q.blocked = [outer, discs, rad, from, to](double x, double y) {
            if (outer && outer(x, y)) return true;
            for (const Vec2& d : discs) {
                const Vec2 v(x - d.x, y - d.y);
                if (v.length() < rad && (Vec2(x, y) - from).length() > rad && (Vec2(x, y) - to).length() > rad) return true;
            }
            return false;
        };
    }
    return best;
}

namespace {
TerrainRoute routeOnce(const HeightField& height, const Vec2& from, const Vec2& to, const TerrainRouteParams& p) {
    TerrainRoute out;
    const double c = p.cell;
    const double x0 = std::min(from.x, to.x) - p.margin, y0 = std::min(from.y, to.y) - p.margin;
    const double x1 = std::max(from.x, to.x) + p.margin, y1 = std::max(from.y, to.y) + p.margin;
    const int nx = static_cast<int>(std::ceil((x1 - x0) / c)) + 1, ny = static_cast<int>(std::ceil((y1 - y0) / c)) + 1;
    auto P = [&](int i, int j) { return Vec2(x0 + i * c, y0 + j * c); };
    // Heights on the HALF-cell lattice, sampled on demand and kept: every node is on it, and so is
    // every move's midpoint (a step of up to three cells, halved) -- the grade check's samples.
    const int hx = 2 * nx - 1, hy = 2 * ny - 1;
    std::vector<double> H(static_cast<std::size_t>(hx) * hy, std::numeric_limits<double>::quiet_NaN());
    auto hHalf = [&](int a, int b) {   // half-lattice indices
        double& v = H[static_cast<std::size_t>(b) * hx + a];
        if (std::isnan(v)) v = height(x0 + a * c * 0.5, y0 + b * c * 0.5);
        return v;
    };
    auto h = [&](int i, int j) { return hHalf(2 * i, 2 * j); };
    std::vector<char> blockedCell;
    if (p.blocked) {
        blockedCell.assign(static_cast<std::size_t>(nx) * ny, 0);
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) { const Vec2 q = P(i, j); blockedCell[static_cast<std::size_t>(j) * nx + i] = p.blocked(q.x, q.y) ? 1 : 0; }
    }
    const std::vector<Move> moves = movesUpTo3();
    const int nd = static_cast<int>(moves.size());
    const int si = std::clamp(static_cast<int>(std::lround((from.x - x0) / c)), 0, nx - 1);
    const int sj = std::clamp(static_cast<int>(std::lround((from.y - y0) / c)), 0, ny - 1);
    const int gi = std::clamp(static_cast<int>(std::lround((to.x - x0) / c)), 0, nx - 1);
    const int gj = std::clamp(static_cast<int>(std::lround((to.y - y0) / c)), 0, ny - 1);
    // states: (node, incoming move) + one start state per node (dir = nd: "no heading yet")
    const std::size_t nStates = static_cast<std::size_t>(nx) * ny * (nd + 1);
    std::vector<float> G(nStates, std::numeric_limits<float>::infinity());
    std::vector<int> came(nStates, -1);
    auto sid = [&](int i, int j, int d) { return (static_cast<std::size_t>(j) * nx + i) * (nd + 1) + d; };
    const Vec2 goalP = P(gi, gj);
    auto heur = [&](int i, int j) { return static_cast<float>((P(i, j) - goalP).length()); };
    struct Q { float f; std::size_t s; };
    struct Cmp { bool operator()(const Q& a, const Q& b) const { return a.f > b.f || (a.f == b.f && a.s > b.s); } };
    std::priority_queue<Q, std::vector<Q>, Cmp> open;
    const std::size_t start = sid(si, sj, nd);
    G[start] = 0.0f;
    open.push({heur(si, sj), start});
    const double maxTurn = p.maxTurnDeg * 3.14159265358979 / 180.0;
    std::size_t goalState = SIZE_MAX;
    while (!open.empty()) {
        const Q q = open.top();
        open.pop();
        const std::size_t s = q.s;
        const int d = static_cast<int>(s % (nd + 1));
        const std::size_t node = s / (nd + 1);
        const int i = static_cast<int>(node % nx), j = static_cast<int>(node / nx);
        if (q.f > G[s] + heur(i, j) + 1e-3f) continue;   // stale
        ++out.expanded;
        if (i == gi && j == gj) { goalState = s; break; }
        const double hij = h(i, j);
        for (int m = 0; m < nd; ++m) {
            const Move& mv = moves[static_cast<std::size_t>(m)];
            const int ni = i + mv.dx, nj = j + mv.dy;
            if (ni < 0 || nj < 0 || ni >= nx || nj >= ny) continue;
            if (!blockedCell.empty() && blockedCell[static_cast<std::size_t>(nj) * nx + ni]) continue;
            double turn = 0.0;
            if (d < nd) {
                turn = std::fabs(mv.ang - moves[static_cast<std::size_t>(d)].ang);
                if (turn > 3.14159265358979) turn = 2 * 3.14159265358979 - turn;
                if (turn > maxTurn) continue;
            }
            const double L = mv.len * c;
            // the grade over the move, checked at its middle too (a move must not hop a ridge)
            const double hm = hHalf(2 * i + mv.dx, 2 * j + mv.dy), hn = h(ni, nj);
            const double g = std::max({std::fabs(hn - hij) / L, std::fabs(hm - hij) / (0.5 * L), std::fabs(hn - hm) / (0.5 * L)});
            if (g > p.hardGrade) continue;
            const double over = std::max(0.0, g - p.maxGrade);
            double cost = L * (1.0 + p.flatWeight * g + p.gradeWeight * over * over) + p.turnWeight * turn * turn * c;
            if (d < nd && p.bendWeight > 0.0) {   // the change of SIGNED grade from the move before
                const Move& pm = moves[static_cast<std::size_t>(d)];
                const double gPrev = (hij - h(i - pm.dx, j - pm.dy)) / (pm.len * c);
                const double gHere = (hn - hij) / L;
                const double dg = gHere - gPrev;
                cost += p.bendWeight * dg * dg * L;
            }
            const std::size_t ns = sid(ni, nj, m);
            const float ng = G[s] + static_cast<float>(cost);
            if (ng < G[ns]) {
                G[ns] = ng;
                came[ns] = static_cast<int>(s);
                open.push({ng + heur(ni, nj), ns});
            }
        }
    }
    if (goalState == SIZE_MAX) return out;
    // walk back
    std::vector<Vec2> pts;
    std::vector<double> hs;
    for (std::size_t s = goalState;;) {
        const std::size_t node = s / (nd + 1);
        const int i = static_cast<int>(node % nx), j = static_cast<int>(node / nx);
        pts.push_back(P(i, j));
        hs.push_back(h(i, j));
        if (came[s] < 0) break;
        s = static_cast<std::size_t>(came[s]);
    }
    std::reverse(pts.begin(), pts.end());
    std::reverse(hs.begin(), hs.end());
    pts.front() = from;   // the exact endpoints (the grid snapped them)
    pts.back() = to;
    hs.front() = height(from.x, from.y);
    hs.back() = height(to.x, to.y);
    // simplify: 1.5 m off the line in plan, 0.6 m in height
    std::vector<char> keep(pts.size(), 0);
    keep.front() = keep.back() = 1;
    simplify(pts, hs, 1.5, 0.6, 0, pts.size() - 1, keep);
    for (std::size_t k = 0; k < pts.size(); ++k)
        if (keep[k]) {
            if (!out.points.empty()) {
                const double L = (pts[k] - out.points.back()).length();
                const double dh = hs[k] - height(out.points.back().x, out.points.back().y);
                out.length += L;
                if (dh > 0) out.climb += dh;
                if (L > 1e-6) out.worstGrade = std::max(out.worstGrade, std::fabs(dh) / L);
            }
            out.points.push_back(pts[k]);
        }
    return out;
}

}  // namespace

std::vector<Vec2> tightenRoute(const HeightField& height, const std::vector<Vec2>& pts, const TightenParams& p) {
    if (pts.size() < 3) return pts;
    auto drivable = [&](const Vec2& a, const Vec2& b) {
        const double L = (b - a).length();
        if (L > p.maxStraight) return false;
        const double ha = height(a.x, a.y), hb = height(b.x, b.y);
        if (L > 1e-6 && std::fabs(hb - ha) / L > p.maxGrade) return false;
        const int m = std::max(1, static_cast<int>(std::ceil(L / p.sample)));
        for (int k = 1; k < m; ++k) {
            const double t = static_cast<double>(k) / m;
            const Vec2 q = a + (b - a) * t;
            if (p.blocked && p.blocked(q.x, q.y)) return false;
            if (std::fabs(height(q.x, q.y) - (ha + (hb - ha) * t)) > p.maxCutFill) return false;
        }
        return true;
    };
    // along the road, measured, so the look-ahead is bounded by distance, not by vertex count
    std::vector<double> s(pts.size(), 0.0);
    for (std::size_t k = 1; k < pts.size(); ++k) s[k] = s[k - 1] + (pts[k] - pts[k - 1]).length();
    std::vector<Vec2> out{pts.front()};
    std::size_t i = 0;
    while (i + 1 < pts.size()) {
        std::size_t best = i + 1;
        // the farthest point it can reach straight -- up to three straights' worth along the road, so
        // a spur kilometres long is seen whole
        for (std::size_t j = i + 2; j < pts.size() && s[j] - s[i] <= 3.0 * p.maxStraight; ++j)
            if (drivable(pts[i], pts[j])) best = j;
        out.push_back(pts[best]);
        i = best;
    }
    return out;
}

std::vector<Vec2> roundRoute(const std::vector<Vec2>& pts, double radius, double step) {
    if (pts.size() < 3 || radius <= 0) return pts;
    std::vector<Vec2> r{pts.front()};
    for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
        const Vec2 a = pts[k], b = pts[k + 1];
        const int m = std::max(1, static_cast<int>(std::ceil((b - a).length() / step)));
        for (int j = 1; j <= m; ++j) r.push_back(a + (b - a) * (static_cast<double>(j) / m));
    }
    const int h = std::max(1, static_cast<int>(std::lround(radius / step)));
    const int n = static_cast<int>(r.size());
    // twice: a moving average's corner is a parabola; two make it a smooth arc
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<Vec2> o(r.size());
        for (int i = 0; i < n; ++i) {
            const int w = std::min({h, i, n - 1 - i});
            Vec2 sum(0, 0);
            for (int k = -w; k <= w; ++k) sum = sum + r[static_cast<std::size_t>(i + k)];
            o[static_cast<std::size_t>(i)] = sum * (1.0 / (2 * w + 1));
        }
        r.swap(o);
    }
    return r;
}

TerrainRoute measureRoute(const HeightField& height, const std::vector<Vec2>& pts, double window) {
    TerrainRoute out;
    out.points = pts;
    double prevH = pts.empty() ? 0.0 : height(pts.front().x, pts.front().y);
    std::vector<double> st{0.0}, hs{prevH};
    for (std::size_t k = 1; k < pts.size(); ++k) {
        const double L = (pts[k] - pts[k - 1]).length(), h = height(pts[k].x, pts[k].y);
        out.length += L;
        if (h > prevH) out.climb += h - prevH;
        prevH = h;
        st.push_back(out.length);
        hs.push_back(h);
    }
    for (std::size_t a = 0, b = 0; a < st.size(); ++a) {
        while (b < st.size() && st[b] - st[a] < window) ++b;
        if (b < st.size()) out.worstGrade = std::max(out.worstGrade, std::fabs(hs[b] - hs[a]) / (st[b] - st[a]));
    }
    return out;
}

}  // namespace engine
