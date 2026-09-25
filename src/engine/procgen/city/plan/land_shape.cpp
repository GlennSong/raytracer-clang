#include "land_shape.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>

namespace engine {
namespace plan {

namespace {

// the connected part of `m` containing (i0, j0) (4-neighbour)
std::vector<char> componentOf(const std::vector<char>& m, int n, int i0, int j0) {
    std::vector<char> out(m.size(), 0);
    if (i0 < 0 || j0 < 0 || i0 >= n || j0 >= n || !m[static_cast<std::size_t>(j0) * n + i0]) return out;
    std::vector<int> stack{j0 * n + i0};
    out[static_cast<std::size_t>(j0) * n + i0] = 1;
    while (!stack.empty()) {
        const int k = stack.back();
        stack.pop_back();
        const int i = k % n, j = k / n;
        const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (const auto& d : nb) {
            const int u = i + d[0], v = j + d[1];
            if (u < 0 || v < 0 || u >= n || v >= n) continue;
            const std::size_t q = static_cast<std::size_t>(v) * n + u;
            if (out[q] || !m[q]) continue;
            out[q] = 1;
            stack.push_back(v * n + u);
        }
    }
    return out;
}

// a separable box blur of a 0/1 mask, `r` cells, as a fraction
std::vector<float> boxBlur(const std::vector<float>& a, int n, int r) {
    std::vector<float> t(a.size()), o(a.size());
    for (int j = 0; j < n; ++j) {
        double s = 0;
        int c = 0;
        for (int i = -r; i < n + r; ++i) {
            if (i + r < n && i + r >= 0) { s += a[static_cast<std::size_t>(j) * n + i + r]; ++c; }
            if (i - r - 1 >= 0 && i - r - 1 < n) { s -= a[static_cast<std::size_t>(j) * n + i - r - 1]; --c; }
            if (i >= 0 && i < n) t[static_cast<std::size_t>(j) * n + i] = static_cast<float>(s / std::max(1, c));
        }
    }
    for (int i = 0; i < n; ++i) {
        double s = 0;
        int c = 0;
        for (int j = -r; j < n + r; ++j) {
            if (j + r < n && j + r >= 0) { s += t[static_cast<std::size_t>(j + r) * n + i]; ++c; }
            if (j - r - 1 >= 0 && j - r - 1 < n) { s -= t[static_cast<std::size_t>(j - r - 1) * n + i]; --c; }
            if (j >= 0 && j < n) o[static_cast<std::size_t>(j) * n + i] = static_cast<float>(s / std::max(1, c));
        }
    }
    return o;
}

std::vector<Vec2> resample(const std::vector<Vec2>& pts, bool closed, double step) {
    std::vector<Vec2> src = pts;
    if (closed && !src.empty()) src.push_back(src.front());
    std::vector<Vec2> out;
    if (src.size() < 2) return out;
    double carry = 0.0;
    out.push_back(src.front());
    for (std::size_t k = 0; k + 1 < src.size(); ++k) {
        const Vec2 a = src[k], b = src[k + 1];
        const double L = (b - a).length();
        double t = step - carry;
        while (t <= L) { out.push_back(a + (b - a) * (t / L)); t += step; }
        carry = L - (t - step);
    }
    if (closed) { if (out.size() > 1 && (out.back() - out.front()).length() < step * 0.5) out.pop_back(); }
    else if ((out.back() - src.back()).length() > 1e-6) out.push_back(src.back());
    return out;
}

}  // namespace

std::vector<Vec2> smoothPolyline(const std::vector<Vec2>& pts, bool closed, double window, double step) {
    const int h = static_cast<int>(std::lround(window / (2.0 * step)));
    const int n = static_cast<int>(pts.size());
    if (h < 1 || n < 3) return pts;
    std::vector<Vec2> out(pts.size());
    for (int i = 0; i < n; ++i) {
        const int r = closed ? h : std::min({h, i, n - 1 - i});   // an open line's ends stay put
        Vec2 sum(0, 0);
        for (int k = -r; k <= r; ++k) sum = sum + pts[static_cast<std::size_t>(((i + k) % n + n) % n)];
        out[static_cast<std::size_t>(i)] = sum * (1.0 / (2 * r + 1));
    }
    return out;
}

bool LandShape::isWater(const Vec2& p) const {
    const int i = static_cast<int>(std::lround((p.x - origin.x) / cell)), j = static_cast<int>(std::lround((p.y - origin.y) / cell));
    return i >= 0 && j >= 0 && i < n && j < n && !water.empty() && water[static_cast<std::size_t>(j) * n + i];
}

bool LandShape::inside(const Vec2& p) const {
    const int i = static_cast<int>(std::lround((p.x - origin.x) / cell)), j = static_cast<int>(std::lround((p.y - origin.y) / cell));
    return i >= 0 && j >= 0 && i < n && j < n && in[static_cast<std::size_t>(j) * n + i];
}

double LandShape::depthAt(const Vec2& p) const {
    if (n < 2) return 0.0;
    const double fx = std::clamp((p.x - origin.x) / cell, 0.0, n - 1.001), fy = std::clamp((p.y - origin.y) / cell, 0.0, n - 1.001);
    const int i = static_cast<int>(fx), j = static_cast<int>(fy);
    const double u = fx - i, v = fy - j;
    auto D = [&](int a, int b) { return static_cast<double>(depth[static_cast<std::size_t>(b) * n + a]); };
    return (D(i, j) * (1 - u) + D(i + 1, j) * u) * (1 - v) + (D(i, j + 1) * (1 - u) + D(i + 1, j + 1) * u) * v;
}

Vec2 LandShape::gradient(const Vec2& p) const {
    const double e = cell;
    const Vec2 g((depthAt(p + Vec2(e, 0)) - depthAt(p - Vec2(e, 0))) / (2 * e), (depthAt(p + Vec2(0, e)) - depthAt(p - Vec2(0, e))) / (2 * e));
    const double L = g.length();
    return L > 1e-6 ? g * (1.0 / L) : Vec2(0, 0);
}

double LandShape::depthHolding(double share) const {
    std::vector<float> d;
    for (std::size_t k = 0; k < in.size(); ++k) if (in[k]) d.push_back(depth[k]);
    if (d.empty()) return 0.0;
    const std::size_t at = std::min(d.size() - 1, static_cast<std::size_t>(std::clamp(share, 0.0, 1.0) * d.size()));
    std::nth_element(d.begin(), d.begin() + static_cast<std::ptrdiff_t>(at), d.end(), std::greater<float>());
    return d[at];
}

std::vector<std::vector<Vec2>> LandShape::contour(double level, double step, double minLength, double smooth) const {
    // MARCHING SQUARES over the depth grid. Each crossing is named by the grid edge it lies on
    // (cell index * 2 + 0 for the edge along x, + 1 along y), so the segments of neighbouring
    // squares share their ends exactly and chain into lines.
    auto val = [&](int i, int j) { return static_cast<double>(depth[static_cast<std::size_t>(j) * n + i]) - level; };
    auto key = [&](int i, int j, int axis) { return (static_cast<long long>(j) * n + i) * 2 + axis; };
    auto point = [&](int i, int j, int axis) {
        const double a = val(i, j), b = axis == 0 ? val(i + 1, j) : val(i, j + 1);
        const double t = std::fabs(a - b) > 1e-12 ? a / (a - b) : 0.5;
        return origin + Vec2((i + (axis == 0 ? t : 0.0)) * cell, (j + (axis == 1 ? t : 0.0)) * cell);
    };
    std::unordered_map<long long, std::vector<long long>> adj;
    std::unordered_map<long long, Vec2> at;
    auto link = [&](long long a, long long b) { adj[a].push_back(b); adj[b].push_back(a); };
    for (int j = 0; j + 1 < n; ++j)
        for (int i = 0; i + 1 < n; ++i) {
            const double v0 = val(i, j), v1 = val(i + 1, j), v2 = val(i + 1, j + 1), v3 = val(i, j + 1);
            const int c = (v0 > 0) | (v1 > 0) << 1 | (v2 > 0) << 2 | (v3 > 0) << 3;
            if (c == 0 || c == 15) continue;
            // the square's four edges: bottom (i,j,x), right (i+1,j,y), top (i,j+1,x), left (i,j,y)
            const long long E[4] = {key(i, j, 0), key(i + 1, j, 1), key(i, j + 1, 0), key(i, j, 1)};
            const Vec2 P[4] = {point(i, j, 0), point(i + 1, j, 1), point(i, j + 1, 0), point(i, j, 1)};
            bool cut[4] = {(v0 > 0) != (v1 > 0), (v1 > 0) != (v2 > 0), (v2 > 0) != (v3 > 0), (v3 > 0) != (v0 > 0)};
            std::vector<int> es;
            for (int k = 0; k < 4; ++k) if (cut[k]) { es.push_back(k); at[E[k]] = P[k]; }
            if (es.size() == 2) link(E[es[0]], E[es[1]]);
            else if (es.size() == 4) {   // a saddle: pair by the centre
                const bool centreIn = (v0 + v1 + v2 + v3) * 0.25 > 0;
                if ((c == 5) == centreIn) { link(E[0], E[1]); link(E[2], E[3]); }
                else { link(E[0], E[3]); link(E[1], E[2]); }
            }
        }
    std::vector<std::vector<Vec2>> out;
    std::unordered_map<long long, bool> used;
    auto walk = [&](long long start) {
        std::vector<Vec2> line{at[start]};
        used[start] = true;
        long long cur = start;
        bool closed = false;
        for (;;) {
            long long next = -1;
            for (long long nb : adj[cur]) if (!used[nb]) { next = nb; break; }
            if (next < 0) {
                for (long long nb : adj[cur]) if (nb == start && line.size() > 2) closed = true;
                break;
            }
            used[next] = true;
            line.push_back(at[next]);
            cur = next;
        }
        double L = 0;
        for (std::size_t k = 0; k + 1 < line.size(); ++k) L += (line[k + 1] - line[k]).length();
        if (L >= minLength) out.push_back(smoothPolyline(resample(line, closed, step), closed, smooth, step));
    };
    for (const auto& [k, nb] : adj) if (nb.size() == 1 && !used[k]) walk(k);   // open runs from an end
    for (const auto& [k, nb] : adj) if (!used[k]) walk(k);                       // then the loops
    return out;
}

LandShape growLandShape(const HeightField& ground, const std::function<bool(const Vec2&)>& buildable,
                        const Vec2& seed, double half, const LandShapeParams& p) {
    LandShape s;
    s.cell = p.cell;
    s.origin = seed - Vec2(half, half);
    s.n = static_cast<int>(std::ceil(2 * half / p.cell)) + 1;
    const int n = s.n;
    const std::size_t N = static_cast<std::size_t>(n) * n;
    std::vector<float> h(N);
    std::vector<char> ok(N);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const Vec2 q = s.origin + Vec2(i * p.cell, j * p.cell);
            h[static_cast<std::size_t>(j) * n + i] = static_cast<float>(ground(q.x, q.y));
            ok[static_cast<std::size_t>(j) * n + i] = buildable(q);
        }
    // the heart: the seed, or the nearest buildable cell to it
    int hi = n / 2, hj = n / 2;
    for (int r = 1; r < n / 2 && !ok[static_cast<std::size_t>(hj) * n + hi]; ++r)
        for (int dj = -r; dj <= r; ++dj)
            for (int di = -r; di <= r; ++di)
                if (!ok[static_cast<std::size_t>(hj) * n + hi] && ok[static_cast<std::size_t>(n / 2 + dj) * n + n / 2 + di]) { hi = n / 2 + di; hj = n / 2 + dj; }
    if (!ok[static_cast<std::size_t>(hj) * n + hi]) return s;
    const float hc = h[static_cast<std::size_t>(hj) * n + hi];
    // distance to the sea (chamfer), for the waterfront's pull
    std::vector<float> toSea(N, 1e9f);
    if (p.seaLevel > -1e8) {
        for (std::size_t k = 0; k < N; ++k) if (h[k] < p.seaLevel) toSea[k] = 0.0f;
        const float a = static_cast<float>(p.cell), b = static_cast<float>(p.cell * 1.41421356);
        for (int j = 1; j < n; ++j)
            for (int i = 1; i < n - 1; ++i) {
                float& v = toSea[static_cast<std::size_t>(j) * n + i];
                v = std::min({v, toSea[static_cast<std::size_t>(j) * n + i - 1] + a, toSea[static_cast<std::size_t>(j - 1) * n + i] + a,
                              toSea[static_cast<std::size_t>(j - 1) * n + i - 1] + b, toSea[static_cast<std::size_t>(j - 1) * n + i + 1] + b});
            }
        for (int j = n - 2; j >= 0; --j)
            for (int i = n - 2; i >= 1; --i) {
                float& v = toSea[static_cast<std::size_t>(j) * n + i];
                v = std::min({v, toSea[static_cast<std::size_t>(j) * n + i + 1] + a, toSea[static_cast<std::size_t>(j + 1) * n + i] + a,
                              toSea[static_cast<std::size_t>(j + 1) * n + i + 1] + b, toSea[static_cast<std::size_t>(j + 1) * n + i - 1] + b});
            }
    }
    // GROWTH: Dijkstra, cheapest ground first, to the target area
    std::vector<float> cost(N, 1e30f);
    std::vector<char> grown(N, 0);
    using Q = std::pair<float, int>;
    std::priority_queue<Q, std::vector<Q>, std::greater<Q>> pq;
    cost[static_cast<std::size_t>(hj) * n + hi] = 0;
    pq.push({0.0f, hj * n + hi});
    const std::size_t want = static_cast<std::size_t>(p.targetArea / (p.cell * p.cell));
    std::size_t count = 0;
    while (!pq.empty() && count < want) {
        const auto [c, k] = pq.top();
        pq.pop();
        if (grown[static_cast<std::size_t>(k)] || c > cost[static_cast<std::size_t>(k)]) continue;
        grown[static_cast<std::size_t>(k)] = 1;
        ++count;
        const int i = k % n, j = k / n;
        for (int dj = -1; dj <= 1; ++dj)
            for (int di = -1; di <= 1; ++di) {
                if (!di && !dj) continue;
                const int u = i + di, v = j + dj;
                if (u < 0 || v < 0 || u >= n || v >= n) continue;
                const std::size_t q = static_cast<std::size_t>(v) * n + u;
                if (!ok[q] || grown[q]) continue;
                const double len = p.cell * ((di && dj) ? 1.41421356 : 1.0);
                const double slope = std::fabs(h[q] - h[static_cast<std::size_t>(k)]) / len;
                const double pull = 1.0 - p.coastPull * std::exp(-toSea[q] / p.coastReach);
                const double step = len * pull * (1.0 + p.slopeWeight * slope + p.riseWeight * std::max(0.0f, h[q] - hc));
                if (c + step < cost[q]) { cost[q] = static_cast<float>(c + step); pq.push({cost[q], v * n + u}); }
            }
    }
    // SMOOTHED: a blur and a threshold (a close and an open at once), back on buildable ground,
    // the heart's piece of it, and small holes filled (a knoll or a pond inside the city, not a gap)
    std::vector<float> f(N);
    for (std::size_t k = 0; k < N; ++k) f[k] = grown[k];
    const int r = std::max(1, static_cast<int>(std::lround(p.smooth / p.cell)));
    f = boxBlur(boxBlur(f, n, r), n, r);
    std::vector<char> m(N);
    for (std::size_t k = 0; k < N; ++k) m[k] = f[k] > 0.5f && ok[k];
    m = componentOf(m, n, hi, hj);
    {
        std::vector<char> outside(N, 0), notIn(N);
        for (std::size_t k = 0; k < N; ++k) notIn[k] = !m[k];
        // what of the not-city reaches the grid's edge is outside; the rest are holes
        std::vector<int> stack;
        for (int i = 0; i < n; ++i)
            for (int e : {i, (n - 1) * n + i, i * n, i * n + n - 1})
                if (notIn[static_cast<std::size_t>(e)] && !outside[static_cast<std::size_t>(e)]) { outside[static_cast<std::size_t>(e)] = 1; stack.push_back(e); }
        while (!stack.empty()) {
            const int k = stack.back();
            stack.pop_back();
            const int i = k % n, j = k / n;
            const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
            for (const auto& d : nb) {
                const int u = i + d[0], v = j + d[1];
                if (u < 0 || v < 0 || u >= n || v >= n) continue;
                const std::size_t q = static_cast<std::size_t>(v) * n + u;
                if (outside[q] || !notIn[q]) continue;
                outside[q] = 1;
                stack.push_back(v * n + u);
            }
        }
        // a hole smaller than 4 ha is filled; a bigger one (a lake, a hill) stays out
        std::vector<char> seen(N, 0);
        for (std::size_t k0 = 0; k0 < N; ++k0) {
            if (!notIn[k0] || outside[k0] || seen[k0]) continue;
            std::vector<int> cells{static_cast<int>(k0)}, st{static_cast<int>(k0)};
            seen[k0] = 1;
            while (!st.empty()) {
                const int k = st.back();
                st.pop_back();
                const int i = k % n, j = k / n;
                const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
                for (const auto& d : nb) {
                    const int u = i + d[0], v = j + d[1];
                    if (u < 0 || v < 0 || u >= n || v >= n) continue;
                    const std::size_t q = static_cast<std::size_t>(v) * n + u;
                    if (seen[q] || !notIn[q] || outside[q]) continue;
                    seen[q] = 1;
                    st.push_back(v * n + u);
                    cells.push_back(v * n + u);
                }
            }
            if (cells.size() * p.cell * p.cell < 40000.0) for (int k : cells) m[static_cast<std::size_t>(k)] = 1;
        }
    }
    s.in = m;
    // DEPTH: a chamfer distance to the edge (3x3, 1 and sqrt 2 cells), smoothed so its contours are
    // streets, not staircases. Signed: outside, the distance out (the zero contour is the limits exactly,
    // and a street's overshoot past the edge still has a field to read).
    const float a = static_cast<float>(p.cell), b = static_cast<float>(p.cell * 1.41421356);
    auto chamfer = [&](std::vector<float>& v) {
        for (int j = 1; j < n - 1; ++j)
            for (int i = 1; i < n - 1; ++i) {
                float& x = v[static_cast<std::size_t>(j) * n + i];
                if (x == 0.0f) continue;
                x = std::min({x, v[static_cast<std::size_t>(j) * n + i - 1] + a, v[static_cast<std::size_t>(j - 1) * n + i] + a,
                              v[static_cast<std::size_t>(j - 1) * n + i - 1] + b, v[static_cast<std::size_t>(j - 1) * n + i + 1] + b});
            }
        for (int j = n - 2; j >= 1; --j)
            for (int i = n - 2; i >= 1; --i) {
                float& x = v[static_cast<std::size_t>(j) * n + i];
                if (x == 0.0f) continue;
                x = std::min({x, v[static_cast<std::size_t>(j) * n + i + 1] + a, v[static_cast<std::size_t>(j + 1) * n + i] + a,
                              v[static_cast<std::size_t>(j + 1) * n + i + 1] + b, v[static_cast<std::size_t>(j + 1) * n + i - 1] + b});
            }
    };
    auto signedDepth = [&](const std::vector<char>& mask) {
        std::vector<float> din(N), dout(N);
        for (std::size_t k = 0; k < N; ++k) {
            const int i = static_cast<int>(k % n), j = static_cast<int>(k / n);
            const bool border = i == 0 || j == 0 || i == n - 1 || j == n - 1;
            din[k] = mask[k] ? (border ? a : 1e9f) : 0.0f;
            dout[k] = mask[k] ? 0.0f : 1e9f;
        }
        chamfer(din);
        chamfer(dout);
        std::vector<float> d(N);
        for (std::size_t k = 0; k < N; ++k)
            d[k] = mask[k] ? din[k] - 0.5f * a : -std::min(dout[k], 1e5f) + 0.5f * a;   // the edge between the last cell in and the first out
        return boxBlur(boxBlur(d, n, 1), n, 1);
    };
    // the LIMITS from the footprint as it stands; the depth with its water taken out as edges too
    s.depth = signedDepth(m);
    s.limits = s.contour(0.0, 15.0, 100.0, 0.0);
    s.water.assign(N, 0);
    if (p.water) {
        std::vector<char> dry = m;
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                const std::size_t k = static_cast<std::size_t>(j) * n + i;
                if (m[k] && p.water(s.origin + Vec2(i * p.cell, j * p.cell))) { s.water[k] = 1; dry[k] = 0; }
            }
        s.depth = signedDepth(dry);
    }
    s.maxDepth = 0.0;
    for (std::size_t k = 0; k < N; ++k) {
        if (m[k]) s.area += p.cell * p.cell;
        if (s.depth[k] > s.maxDepth) { s.maxDepth = s.depth[k]; s.heart = s.origin + Vec2((k % n) * p.cell, (k / n) * p.cell); }
    }
    return s;
}

std::vector<Vec2> descendDepth(const LandShape& s, const Vec2& from, double stopDepth, double step) {
    std::vector<Vec2> out{from};
    Vec2 p = from;
    for (int k = 0; k < 2000; ++k) {
        if (s.depthAt(p) <= stopDepth) break;
        const Vec2 g = s.gradient(p);
        if (g.length() < 0.5) break;
        // half a step, then the gradient there: the midpoint rule keeps it on the line of steepest descent
        const Vec2 mid = p - g * (step * 0.5);
        const Vec2 g2 = s.gradient(mid);
        if (g2.length() < 0.5) break;
        p = p - g2 * step;
        out.push_back(p);
    }
    return out;
}

}  // namespace plan
}  // namespace engine
