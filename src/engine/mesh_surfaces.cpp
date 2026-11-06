#include "mesh_builder.h"

#include "../curve.h"   // Spline: the one curve kernel (pathSpline, pathBezier); rotationMinimizingFrames (sweep)

#include <algorithm>
#include <cmath>
#include <unordered_map>

// THE CURVE AND SURFACE WORDS (#191): profiles, paths, and the surfaces made from them. Every surface here is one
// GRID (rows along the path or the sections or round the axis, columns round the profile) built by `GridSurface`,
// which owns the rules the words share: a vertex is shared by the faces round it unless they meet at a CREASE
// (sharper than `creaseDeg`: the row or column is split, so a square tube keeps its edges); a POLE (a row or column
// whose points all coincide -- the tip of a dome, the axis of a revolve) takes the normal of every face round it;
// a closed direction wraps with a UV SEAM (u or v runs on to its full length at the copy of the first row/column);
// and every face is wound by triFacing to the outward HINT its word gives (so no index order is written by hand).

namespace engine {

namespace {

double signedArea2(const std::vector<Vec2>& p) {
    double a = 0;
    for (std::size_t i = 0, n = p.size(); i < n; ++i) {
        const Vec2& u = p[i];
        const Vec2& v = p[(i + 1) % n];
        a += u.x * v.y - v.x * u.y;
    }
    return a * 0.5;
}

MeshProfile ccw(MeshProfile p) {
    if (p.closed && p.pts.size() >= 3 && signedArea2(p.pts) < 0) std::reverse(p.pts.begin(), p.pts.end());
    return p;
}

// x^e keeping x's sign (the superellipse's and superquadric's power)
double spow(double x, double e) { return (x < 0 ? -1.0 : 1.0) * std::pow(std::fabs(x), e); }

// THE GRID: `rows` x `cols` points (row-major), each direction open or closed; `u` per column (0..1 round, the
// closing seam at 1), `v` per row (metres along); `hint(band, quad)` the outward direction of the face between rows
// band..band+1 and columns quad..quad+1.
struct GridSurface {
    int rows = 0, cols = 0;
    bool rowsClosed = false, colsClosed = false;
    std::vector<Vec3> p;
    std::vector<double> u, v;   // sizes: cols + colsClosed, rows + rowsClosed (the unrolled seam copy's value last)
    std::function<Vec3(int, int)> hint;
    double creaseDeg = 40.0;
    Vec3 color{1, 1, 1};

    const Vec3& at(int r, int c) const { return p[static_cast<std::size_t>((r % rows) * cols + (c % cols))]; }

    void emit(RenderMesh& m) const {
        if (rows < 2 || cols < 2) return;
        const int R = rows + (rowsClosed ? 1 : 0), C = cols + (colsClosed ? 1 : 0);   // unrolled: the seam copied
        const int nb = R - 1, nq = C - 1;
        // each face's outward unit normal (zero where it has no area)
        std::vector<Vec3> fn(static_cast<std::size_t>(nb * nq), Vec3(0, 0, 0));
        for (int b = 0; b < nb; ++b)
            for (int q = 0; q < nq; ++q) {
                const Vec3 &a = at(b, q), &bq = at(b, q + 1), &c = at(b + 1, q + 1), &d = at(b + 1, q);
                Vec3 n = cross(c - a, d - bq);   // the diagonals: a robust quad normal, either sign
                if (n.lengthSquared() < 1e-24) continue;
                n = normalize(n);
                if (dot(n, hint(b, q)) < 0) n = n * -1.0;
                fn[static_cast<std::size_t>(b * nq + q)] = n;
            }
        auto face = [&](int b, int q) -> Vec3 {
            if (rowsClosed) b = (b % nb + nb) % nb;
            if (colsClosed) q = (q % nq + nq) % nq;
            if (b < 0 || b >= nb || q < 0 || q >= nq) return Vec3(0, 0, 0);
            return fn[static_cast<std::size_t>(b * nq + q)];
        };
        // CREASES, by the mean angle between the faces either side of a row / column; POLES, where a row's or
        // column's points coincide
        const double cosCrease = std::cos(creaseDeg * DEG_TO_RAD);
        auto meanDot = [](const std::vector<std::pair<Vec3, Vec3>>& pairs) {
            double s = 0;
            int n = 0;
            for (const auto& [x, y] : pairs)
                if (x.lengthSquared() > 0.5 && y.lengthSquared() > 0.5) { s += dot(x, y); ++n; }
            return n ? s / n : 1.0;
        };
        std::vector<char> rowCrease(static_cast<std::size_t>(R), 0), colCrease(static_cast<std::size_t>(C), 0);
        for (int r = 0; r < R; ++r) {
            if (!rowsClosed && (r == 0 || r == R - 1)) continue;
            std::vector<std::pair<Vec3, Vec3>> pr;
            for (int q = 0; q < nq; ++q) pr.push_back({face(r - 1, q), face(r, q)});
            rowCrease[static_cast<std::size_t>(r)] = meanDot(pr) < cosCrease;
        }
        for (int c = 0; c < C; ++c) {
            if (!colsClosed && (c == 0 || c == C - 1)) continue;
            std::vector<std::pair<Vec3, Vec3>> pc;
            for (int b = 0; b < nb; ++b) pc.push_back({face(b, c - 1), face(b, c)});
            colCrease[static_cast<std::size_t>(c)] = meanDot(pc) < cosCrease;
        }
        if (rowsClosed) rowCrease[static_cast<std::size_t>(R - 1)] = rowCrease[0];
        if (colsClosed) colCrease[static_cast<std::size_t>(C - 1)] = colCrease[0];
        std::vector<char> poleRow(static_cast<std::size_t>(R), 1), poleCol(static_cast<std::size_t>(C), 1);
        for (int r = 0; r < R; ++r)
            for (int c = 1; c < cols && poleRow[static_cast<std::size_t>(r)]; ++c)
                if ((at(r, c) - at(r, 0)).lengthSquared() > 1e-18) poleRow[static_cast<std::size_t>(r)] = 0;
        for (int c = 0; c < C; ++c)
            for (int r = 1; r < rows && poleCol[static_cast<std::size_t>(c)]; ++r)
                if ((at(r, c) - at(0, c)).lengthSquared() > 1e-18) poleCol[static_cast<std::size_t>(c)] = 0;

        // a vertex per (row, which side of a row crease, column, which side of a column crease)
        std::unordered_map<uint64_t, uint32_t> made;
        auto vid = [&](int r, int rs, int c, int cs) -> uint32_t {
            if (!rowCrease[static_cast<std::size_t>(r)]) rs = 0;
            if (!colCrease[static_cast<std::size_t>(c)]) cs = 0;
            const uint64_t key = ((static_cast<uint64_t>(r) * 2 + rs) << 32) | (static_cast<uint64_t>(c) * 2 + cs);
            const auto it = made.find(key);
            if (it != made.end()) return it->second;
            Vec3 n(0, 0, 0);
            if (poleRow[static_cast<std::size_t>(r)]) {
                for (int q = 0; q < nq; ++q) n += face(r - 1, q) + face(r, q);
            } else if (poleCol[static_cast<std::size_t>(c)]) {
                for (int b = 0; b < nb; ++b) n += face(b, c - 1) + face(b, c);
            } else {
                const bool rc = rowCrease[static_cast<std::size_t>(r)], cc = colCrease[static_cast<std::size_t>(c)];
                for (int b : {r - 1, r}) {
                    if (rc && (b == r) != (rs == 1)) continue;   // a crease row: only its own side's band
                    for (int q : {c - 1, c}) {
                        if (cc && (q == c) != (cs == 1)) continue;
                        n += face(b, q);
                    }
                }
            }
            if (n.lengthSquared() < 1e-24) n = Vec3(0, 1, 0);
            const Vec3 along = at(r, c + 1) - at(r, c > 0 ? c - 1 : (colsClosed ? cols - 1 : 0));
            Vertex vx(at(r, c), normalize(n), along.lengthSquared() > 1e-24 ? normalize(along) : Vec3(1, 0, 0),
                      static_cast<float>(u[static_cast<std::size_t>(c)]), static_cast<float>(v[static_cast<std::size_t>(r)]));
            vx.color = color;
            const uint32_t idx = static_cast<uint32_t>(m.vertices.size());
            m.vertices.push_back(vx);
            made.emplace(key, idx);
            return idx;
        };
        for (int b = 0; b < nb; ++b)
            for (int q = 0; q < nq; ++q) {
                const Vec3 n = fn[static_cast<std::size_t>(b * nq + q)];
                if (n.lengthSquared() < 0.5) continue;
                const uint32_t a = vid(b, 1, q, 1), bq = vid(b, 1, q + 1, 0), c = vid(b + 1, 0, q + 1, 0), d = vid(b + 1, 0, q, 1);
                MeshBuilder::triFacing(m, a, bq, c, n);
                MeshBuilder::triFacing(m, a, c, d, n);
            }
    }
};

// u round a profile: each point's share of its length (closed: the seam copy at 1)
std::vector<double> profileU(const MeshProfile& p) {
    const std::size_t n = p.pts.size();
    std::vector<double> u(n + (p.closed ? 1 : 0), 0.0);
    for (std::size_t i = 1; i < u.size(); ++i) u[i] = u[i - 1] + (p.pts[i % n] - p.pts[i - 1]).length();
    const double total = u.back();
    if (total > 1e-12)
        for (double& x : u) x /= total;
    return u;
}

// the outward normal of profile edge k..k+1 (counter-clockwise ring: to its right)
Vec2 edgeNormal(const MeshProfile& p, std::size_t k) {
    const std::size_t n = p.pts.size();
    const Vec2 d = p.pts[(k + 1) % n] - p.pts[k];
    const double l = d.length();
    return l > 1e-12 ? Vec2(d.y / l, -d.x / l) : Vec2(0, 0);
}

Vec3 rotateAbout(const Vec3& v, const Vec3& axis, double a) {
    const double c = std::cos(a), s = std::sin(a);
    return v * c + cross(axis, v) * s + axis * (dot(axis, v) * (1 - c));
}

Vec3 anyPerpendicular(const Vec3& n) {
    const Vec3 ref = std::fabs(n.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    return normalize(cross(ref, n));
}

// the path's tangents (central differences; the ends one-sided unless it closes)
std::vector<Vec3> pathTangents(const MeshPath& path) {
    const std::size_t n = path.pts.size();
    std::vector<Vec3> t(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3 prev = i > 0 ? path.pts[i - 1] : (path.closed ? path.pts[n - 1] : path.pts[i]);
        const Vec3 next = i + 1 < n ? path.pts[i + 1] : (path.closed ? path.pts[0] : path.pts[i]);
        Vec3 d = next - prev;
        t[i] = d.lengthSquared() > 1e-24 ? normalize(d) : Vec3(0, 0, 1);
    }
    return t;
}

// THE FRAMES along a path: rotation-minimising (no spiral), the first one's up turned toward `up`; on a closed path
// the twist left over after going round is spread along it, so the ring meets itself.
std::vector<CurveFrame> sweepFrames(const MeshPath& path, const Vec3& up) {
    std::vector<Vec3> pts = path.pts, tan = pathTangents(path);
    if (path.closed) { pts.push_back(pts.front()); tan.push_back(tan.front()); }
    std::vector<CurveFrame> f = rotationMinimizingFrames(pts, tan);
    if (f.empty()) return f;
    // the first frame's up toward the hint (its projection square to the tangent)
    const Vec3 hp = up - f[0].tangent * dot(up, f[0].tangent);
    if (hp.lengthSquared() > 1e-12) {
        const Vec3 h = normalize(hp);
        const double phi = std::atan2(-dot(h, f[0].right), dot(h, f[0].up));
        for (CurveFrame& fr : f) {
            const Vec3 r = fr.right * std::cos(phi) + fr.up * std::sin(phi);
            const Vec3 u2 = fr.up * std::cos(phi) - fr.right * std::sin(phi);
            fr.right = r;
            fr.up = u2;
        }
    }
    if (path.closed) {
        const CurveFrame& a = f.front();
        const CurveFrame& z = f.back();
        const double resid = std::atan2(dot(cross(a.right, z.right), a.tangent), dot(a.right, z.right));
        double total = 0;
        std::vector<double> s(pts.size(), 0.0);
        for (std::size_t i = 1; i < pts.size(); ++i) s[i] = s[i - 1] + (pts[i] - pts[i - 1]).length();
        total = s.back();
        for (std::size_t i = 0; i < f.size(); ++i) {
            const double phi = total > 1e-12 ? -resid * s[i] / total : 0.0;
            const Vec3 r = rotateAbout(f[i].right, f[i].tangent, phi), u2 = rotateAbout(f[i].up, f[i].tangent, phi);
            f[i].right = r;
            f[i].up = u2;
        }
        f.pop_back();
    }
    return f;
}

// the arc-length position of every point (and the total)
std::vector<double> runningLength(const std::vector<Vec3>& pts, bool closed) {
    std::vector<double> s(pts.size() + (closed ? 1 : 0), 0.0);
    for (std::size_t i = 1; i < s.size(); ++i) s[i] = s[i - 1] + (pts[i % pts.size()] - pts[i - 1]).length();
    return s;
}

// points `count` evenly spaced along a densely sampled curve
std::vector<Vec3> evenlyAlong(const std::vector<Vec3>& dense, int count) {
    std::vector<Vec3> out;
    if (dense.size() < 2 || count < 2) return dense;
    const std::vector<double> s = runningLength(dense, false);
    const double total = s.back();
    std::size_t j = 0;
    for (int k = 0; k < count; ++k) {
        const double want = total * k / (count - 1);
        while (j + 2 < s.size() && s[j + 1] < want) ++j;
        const double span = s[j + 1] - s[j];
        const double t = span > 1e-12 ? std::clamp((want - s[j]) / span, 0.0, 1.0) : 0.0;
        out.push_back(dense[j] + (dense[j + 1] - dense[j]) * t);
    }
    return out;
}

}  // namespace

// --- PROFILES ------------------------------------------------------------------------------------------------------

MeshProfile MeshBuilder::profileRect(double w, double h) {
    return {{Vec2(-w * 0.5, -h * 0.5), Vec2(w * 0.5, -h * 0.5), Vec2(w * 0.5, h * 0.5), Vec2(-w * 0.5, h * 0.5)}, true};
}

MeshProfile MeshBuilder::profileRoundedRect(double w, double h, double r, int segs) {
    r = std::clamp(r, 0.0, 0.5 * std::min(w, h));
    return r > 1e-9 ? profileFillet(profileRect(w, h), r, segs) : profileRect(w, h);
}

MeshProfile MeshBuilder::profileCircle(double r, int sides) { return profileEllipse(r, r, sides); }

MeshProfile MeshBuilder::profileEllipse(double rx, double ry, int sides) {
    MeshProfile p;
    sides = std::max(3, sides);
    for (int k = 0; k < sides; ++k) {
        const double a = TAU * k / sides;
        p.pts.push_back(Vec2(rx * std::cos(a), ry * std::sin(a)));
    }
    return p;
}

MeshProfile MeshBuilder::profileSuperellipse(double rx, double ry, double n, int sides) {
    MeshProfile p;
    sides = std::max(4, sides);
    const double e = 2.0 / std::max(n, 0.05);
    for (int k = 0; k < sides; ++k) {
        const double a = TAU * k / sides;
        p.pts.push_back(Vec2(rx * spow(std::cos(a), e), ry * spow(std::sin(a), e)));
    }
    return p;
}

MeshProfile MeshBuilder::profilePolygon(int sides, double radius) { return profileEllipse(radius, radius, std::max(3, sides)); }

MeshProfile MeshBuilder::profileStar(int points, double outer, double inner) {
    MeshProfile p;
    points = std::max(2, points);
    for (int k = 0; k < 2 * points; ++k) {
        const double a = TAU * k / (2 * points), r = k % 2 ? inner : outer;
        p.pts.push_back(Vec2(r * std::cos(a), r * std::sin(a)));
    }
    return p;
}

MeshProfile MeshBuilder::profileFillet(const MeshProfile& in, double radius, int segs) {
    const MeshProfile p = ccw(in);
    const std::size_t n = p.pts.size();
    if (n < 3 || radius <= 0) return p;
    segs = std::max(1, segs);
    MeshProfile out;
    out.closed = p.closed;
    for (std::size_t i = 0; i < n; ++i) {
        const bool end = !p.closed && (i == 0 || i + 1 == n);
        const Vec2 P = p.pts[i];
        if (end) { out.pts.push_back(P); continue; }
        const Vec2 A = p.pts[(i + n - 1) % n], B = p.pts[(i + 1) % n];
        const double la = (A - P).length(), lb = (B - P).length();
        if (la < 1e-12 || lb < 1e-12) { out.pts.push_back(P); continue; }
        const Vec2 u1 = (A - P) * (1.0 / la), u2 = (B - P) * (1.0 / lb);
        const double half = 0.5 * std::acos(std::clamp(dot(u1, u2), -1.0, 1.0));   // half the corner's inside angle
        if (half > 1.5607) { out.pts.push_back(P); continue; }                    // straight through: no corner
        // (each edge is shared with the next corner's round: half of it each, at most)
        const double t = std::min(radius / std::tan(half), 0.5 * std::min(la, lb));
        const double r = t * std::tan(half);
        const Vec2 T1 = P + u1 * t, T2 = P + u2 * t;
        const Vec2 bis = normalize(u1 + u2);
        const Vec2 c = P + bis * (r / std::sin(half));
        double a1 = std::atan2(T1.y - c.y, T1.x - c.x), a2 = std::atan2(T2.y - c.y, T2.x - c.x);
        double da = a2 - a1;
        while (da > PI) da -= TAU;
        while (da < -PI) da += TAU;
        for (int k = 0; k <= segs; ++k) {
            const double a = a1 + da * k / segs;
            out.pts.push_back(Vec2(c.x + r * std::cos(a), c.y + r * std::sin(a)));
        }
    }
    return out;
}

MeshProfile MeshBuilder::profileOffset(const MeshProfile& in, double d) {
    const MeshProfile p = ccw(in);
    const std::size_t n = p.pts.size();
    MeshProfile out;
    out.closed = p.closed;
    if (n < 2) return out;
    for (std::size_t i = 0; i < n; ++i) {
        const bool first = !p.closed && i == 0, last = !p.closed && i + 1 == n;
        const Vec2 na = first ? edgeNormal(p, 0) : edgeNormal(p, (i + n - 1) % n);
        const Vec2 nb = last ? edgeNormal(p, n - 2) : edgeNormal(p, i);
        Vec2 m = na + nb;
        if (m.length() < 1e-9) m = na;
        m = normalize(m);
        const double c = std::max(dot(m, na), 0.25);   // (a mitre longer than 4 d is clipped)
        out.pts.push_back(p.pts[i] + m * (d / c));
    }
    if (p.closed && n >= 3 && signedArea2(out.pts) * signedArea2(p.pts) <= 0) return MeshProfile{{}, p.closed};   // collapsed
    return out;
}

MeshProfile MeshBuilder::profileResample(const MeshProfile& p, int count) {
    MeshProfile out;
    out.closed = p.closed;
    const std::size_t n = p.pts.size();
    if (n < 2 || count < 2) return p;
    std::vector<Vec3> dense;
    for (std::size_t i = 0; i < n + (p.closed ? 1 : 0); ++i) dense.push_back(Vec3(p.pts[i % n].x, p.pts[i % n].y, 0));
    std::vector<Vec3> even = evenlyAlong(dense, count + (p.closed ? 1 : 0));
    if (p.closed) even.pop_back();
    for (const Vec3& v : even) out.pts.push_back(Vec2(v.x, v.y));
    return out;
}

// --- PATHS ---------------------------------------------------------------------------------------------------------

MeshPath MeshBuilder::pathLine(const Vec3& a, const Vec3& b, int segs) {
    MeshPath p;
    segs = std::max(1, segs);
    for (int k = 0; k <= segs; ++k) p.pts.push_back(a + (b - a) * (static_cast<double>(k) / segs));
    return p;
}

MeshPath MeshBuilder::pathArc(const Vec3& centre, const Vec3& from, const Vec3& normal, double angle, int segs) {
    MeshPath p;
    segs = std::max(1, segs);
    const Vec3 n = normalize(normal);
    for (int k = 0; k <= segs; ++k) p.pts.push_back(centre + rotateAbout(from, n, angle * k / segs));
    return p;
}

MeshPath MeshBuilder::pathCircle(const Vec3& centre, double radius, const Vec3& normal, int segs) {
    const Vec3 n = normalize(normal);
    MeshPath p = pathArc(centre, anyPerpendicular(n) * radius, n, TAU, std::max(3, segs));
    p.pts.pop_back();   // (a ring: the last point is not the first again)
    p.closed = true;
    return p;
}

MeshPath MeshBuilder::pathHelix(const Vec3& centre, double radius, double pitch, double turns, int segsPerTurn) {
    MeshPath p;
    const int n = std::max(2, static_cast<int>(std::ceil(std::fabs(turns) * std::max(3, segsPerTurn)))) + 1;
    for (int k = 0; k < n; ++k) {
        const double t = static_cast<double>(k) / (n - 1), a = TAU * turns * t;
        p.pts.push_back(centre + Vec3(radius * std::cos(a), pitch * turns * t, radius * std::sin(a)));
    }
    return p;
}

MeshPath MeshBuilder::pathSpline(const std::vector<Vec3>& points, bool closed, int count) {
    MeshPath p;
    p.closed = closed && points.size() >= 3;
    if (points.size() < 2) { p.pts = points; return p; }
    const Spline<Vec3> s = Spline<Vec3>::centripetal(points, p.closed);
    count = std::max(2, count);
    for (double u : s.uniformArcLengthParams(count + (p.closed ? 1 : 0))) p.pts.push_back(s.eval(u));
    if (p.closed) p.pts.pop_back();
    return p;
}

MeshPath MeshBuilder::pathBezier(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, int count) {
    std::vector<Vec3> dense;
    for (int k = 0; k <= 256; ++k) dense.push_back(bezier(p0, p1, p2, p3, k / 256.0));
    MeshPath p;
    p.pts = evenlyAlong(dense, std::max(2, count));
    return p;
}

MeshPath MeshBuilder::pathResample(const MeshPath& in, double spacing) {
    if (in.pts.size() < 2 || spacing <= 0) return in;
    std::vector<Vec3> dense = in.pts;
    if (in.closed) dense.push_back(in.pts.front());
    const double total = runningLength(dense, false).back();
    const int count = std::max(2, static_cast<int>(std::lround(total / spacing)) + 1);
    MeshPath p;
    p.closed = in.closed;
    p.pts = evenlyAlong(dense, count);
    if (in.closed) p.pts.pop_back();
    return p;
}

double MeshBuilder::pathLength(const MeshPath& p) {
    return p.pts.size() < 2 ? 0.0 : runningLength(p.pts, p.closed).back();
}

// --- SURFACES ------------------------------------------------------------------------------------------------------

RenderMesh MeshBuilder::sweep(const MeshProfile& profileIn, const MeshPath& path, const SweepStyle& st) {
    RenderMesh m;
    const MeshProfile prof = ccw(profileIn);
    const int np = static_cast<int>(prof.pts.size()), nr = static_cast<int>(path.pts.size());
    if (np < 2 || nr < 2) return m;
    const std::vector<CurveFrame> fr = sweepFrames(path, st.up);
    const std::vector<double> s = runningLength(path.pts, path.closed);
    const double total = std::max(s.back(), 1e-12);
    // the profile placed at each path point, scaled and twisted by its share of the length
    GridSurface g;
    g.rows = nr;
    g.cols = np;
    g.rowsClosed = path.closed;
    g.colsClosed = prof.closed;
    g.creaseDeg = st.creaseDeg;
    g.color = st.color;
    g.u = profileU(prof);
    g.v = s;
    std::vector<double> sc(static_cast<std::size_t>(nr), 1.0), tw(static_cast<std::size_t>(nr), 0.0);
    for (int i = 0; i < nr; ++i) {
        const double t = s[static_cast<std::size_t>(i)] / total;
        if (st.scaleAt) sc[static_cast<std::size_t>(i)] = st.scaleAt(t);
        if (st.twistAt) tw[static_cast<std::size_t>(i)] = st.twistAt(t);
    }
    auto place = [&](int i, const Vec2& q) {
        const double c = std::cos(tw[static_cast<std::size_t>(i)]), sn = std::sin(tw[static_cast<std::size_t>(i)]);
        const double k = sc[static_cast<std::size_t>(i)];
        const Vec2 r(k * (q.x * c - q.y * sn), k * (q.x * sn + q.y * c));
        const CurveFrame& f = fr[static_cast<std::size_t>(i)];
        return path.pts[static_cast<std::size_t>(i)] + f.right * r.x + f.up * r.y;
    };
    for (int i = 0; i < nr; ++i)
        for (int j = 0; j < np; ++j) g.p.push_back(place(i, prof.pts[static_cast<std::size_t>(j)]));
    g.hint = [&](int b, int q) {
        const int i = b % nr;
        const Vec2 n2 = edgeNormal(prof, static_cast<std::size_t>(q % np));
        const double c = std::cos(tw[static_cast<std::size_t>(i)]), sn = std::sin(tw[static_cast<std::size_t>(i)]);
        const Vec2 r(n2.x * c - n2.y * sn, n2.x * sn + n2.y * c);
        const CurveFrame& f = fr[static_cast<std::size_t>(i)];
        return f.right * r.x + f.up * r.y;
    };
    g.emit(m);
    // the ends, a flat cap each: a closed profile on an open path
    if (st.caps && prof.closed && !path.closed && np >= 3) {
        for (int end : {0, nr - 1}) {
            std::vector<Vec3> ring;
            for (int j = 0; j < np; ++j) ring.push_back(place(end, prof.pts[static_cast<std::size_t>(j)]));
            const Vec3 out = fr[static_cast<std::size_t>(end)].tangent * (end == 0 ? -1.0 : 1.0);
            emitPolygon(m, ring, {}, out, st.color, 1.0);
        }
    }
    return m;
}

RenderMesh MeshBuilder::extrude(const MeshProfile& profileIn, double height, double bevel, int bevelSegs, const Vec3& color,
                                int heightSegs) {
    RenderMesh m;
    const MeshProfile prof = ccw(profileIn);
    const int np = static_cast<int>(prof.pts.size());
    if (np < 3 || !prof.closed || height <= 0) return m;
    // the rings bottom to top: (inset, height) -- a quarter round at each edge when bevelled
    bevel = std::clamp(bevel, 0.0, 0.5 * height);
    std::vector<std::pair<double, double>> rings;
    if (bevel > 1e-9 && bevelSegs > 0) {
        for (int k = 0; k <= bevelSegs; ++k) {
            const double a = (PI * 0.5) * k / bevelSegs;
            rings.push_back({bevel * (1 - std::sin(a)), bevel * (1 - std::cos(a))});
        }
        for (int k = 1; k < heightSegs; ++k) rings.push_back({0.0, bevel + (height - 2 * bevel) * k / heightSegs});
        for (int k = bevelSegs; k >= 0; --k) {
            const double a = (PI * 0.5) * k / bevelSegs;
            rings.push_back({bevel * (1 - std::sin(a)), height - bevel * (1 - std::cos(a))});
        }
    } else {
        for (int k = 0; k <= std::max(1, heightSegs); ++k) rings.push_back({0.0, height * k / std::max(1, heightSegs)});
    }
    std::vector<MeshProfile> at;
    for (const auto& [inset, y] : rings) {
        MeshProfile o = inset > 1e-12 ? profileOffset(prof, -inset) : prof;
        if (static_cast<int>(o.pts.size()) != np) return extrude(prof, height, 0.0, 0, color, heightSegs);   // (too thin to bevel)
        at.push_back(std::move(o));
    }
    GridSurface g;
    g.rows = static_cast<int>(rings.size());
    g.cols = np;
    g.colsClosed = true;
    g.color = color;
    g.u = profileU(prof);
    for (int i = 0; i < g.rows; ++i) {
        for (const Vec2& q : at[static_cast<std::size_t>(i)].pts) g.p.push_back(Vec3(q.x, rings[static_cast<std::size_t>(i)].second, q.y));
        g.v.push_back(rings[static_cast<std::size_t>(i)].second);
    }
    g.hint = [&](int b, int q) {
        const Vec2 n2 = edgeNormal(prof, static_cast<std::size_t>(q % np));
        const double mid = 0.5 * (rings[static_cast<std::size_t>(b)].second + rings[static_cast<std::size_t>(b + 1)].second);
        return Vec3(n2.x, mid < 0.5 * height ? -0.5 : 0.5, n2.y);
    };
    g.emit(m);
    for (int end : {0, g.rows - 1}) {
        std::vector<Vec3> ring;
        for (const Vec2& q : at[static_cast<std::size_t>(end)].pts) ring.push_back(Vec3(q.x, rings[static_cast<std::size_t>(end)].second, q.y));
        emitPolygon(m, ring, {}, Vec3(0, end == 0 ? -1.0 : 1.0, 0), color, 1.0);
    }
    return m;
}

RenderMesh MeshBuilder::revolve(const MeshProfile& profileIn, int segs, double from, double to, const Vec3& color,
                                double creaseDeg) {
    RenderMesh m;
    const MeshProfile prof = ccw(profileIn);
    const int np = static_cast<int>(prof.pts.size());
    if (np < 2) return m;
    segs = std::max(3, segs);
    const double span = std::clamp(to - from, -TAU, TAU);
    const bool full = std::fabs(std::fabs(span) - TAU) < 1e-9;
    GridSurface g;
    g.rows = full ? segs : segs + 1;
    g.rowsClosed = full;
    g.cols = np;
    g.colsClosed = prof.closed;
    g.creaseDeg = creaseDeg;
    g.color = color;
    g.u = profileU(prof);
    for (int i = 0; i < g.rows + (full ? 1 : 0); ++i) g.v.push_back(static_cast<double>(i) / segs);
    auto angle = [&](double i) { return from + span * i / segs; };
    for (int i = 0; i < g.rows; ++i) {
        const double a = angle(i), c = std::cos(a), sn = std::sin(a);
        for (const Vec2& q : prof.pts) g.p.push_back(Vec3(q.x * c, q.y, q.x * sn));
    }
    g.hint = [&](int b, int q) {
        const Vec2 n2 = edgeNormal(prof, static_cast<std::size_t>(q % np));
        const double a = angle(b + 0.5);
        return Vec3(n2.x * std::cos(a), n2.y, n2.x * std::sin(a));
    };
    g.emit(m);
    // a part turn of a closed profile: its two cut faces
    if (!full && prof.closed && np >= 3) {
        for (int end : {0, segs}) {
            const double a = angle(end), c = std::cos(a), sn = std::sin(a);
            std::vector<Vec3> ring;
            for (const Vec2& q : prof.pts) ring.push_back(Vec3(q.x * c, q.y, q.x * sn));
            const Vec3 along(-sn, 0, c);   // the way the turn runs at that end
            emitPolygon(m, ring, {}, along * ((end == 0) == (span > 0) ? -1.0 : 1.0), color, 1.0);
        }
    }
    return m;
}

RenderMesh MeshBuilder::loft(const std::vector<std::vector<Vec3>>& sections, bool closedRings, bool caps, const Vec3& color,
                             double creaseDeg) {
    RenderMesh m;
    const int nr = static_cast<int>(sections.size());
    if (nr < 2) return m;
    const int np = static_cast<int>(sections[0].size());
    if (np < 2) return m;
    for (const auto& s : sections)
        if (static_cast<int>(s.size()) != np) return m;   // (every section the same count)
    std::vector<Vec3> centre(static_cast<std::size_t>(nr), Vec3(0, 0, 0));
    for (int i = 0; i < nr; ++i) {
        for (const Vec3& q : sections[static_cast<std::size_t>(i)]) centre[static_cast<std::size_t>(i)] += q;
        centre[static_cast<std::size_t>(i)] = centre[static_cast<std::size_t>(i)] * (1.0 / np);
    }
    GridSurface g;
    g.rows = nr;
    g.cols = np;
    g.colsClosed = closedRings;
    g.creaseDeg = creaseDeg;
    g.color = color;
    for (const auto& s : sections)
        for (const Vec3& q : s) g.p.push_back(q);
    // u round the first section, v along the centres in metres
    {
        const std::vector<double> s0 = runningLength(sections[0], closedRings);
        for (double x : s0) g.u.push_back(s0.back() > 1e-12 ? x / s0.back() : 0.0);
        g.v = runningLength(centre, false);
    }
    g.hint = [&](int b, int q) {
        const Vec3 mid = (g.at(b, q) + g.at(b, q + 1) + g.at(b + 1, q) + g.at(b + 1, q + 1)) * 0.25;
        const Vec3 c = (centre[static_cast<std::size_t>(b)] + centre[static_cast<std::size_t>(b + 1)]) * 0.5;
        return mid - c;
    };
    g.emit(m);
    if (caps && closedRings && np >= 3) {
        emitPolygon(m, sections.front(), {}, centre[0] - centre[1], color, 1.0);
        emitPolygon(m, sections.back(), {}, centre[static_cast<std::size_t>(nr - 1)] - centre[static_cast<std::size_t>(nr - 2)],
                    color, 1.0);
    }
    return m;
}

RenderMesh MeshBuilder::superquadric(const Vec3& r, double e1, double e2, int stacks, int slices, const Vec3& color) {
    RenderMesh m;
    stacks = std::max(2, stacks);
    slices = std::max(3, slices);
    e1 = std::max(e1, 0.02);
    e2 = std::max(e2, 0.02);
    GridSurface g;
    g.rows = stacks + 1;
    g.cols = slices;
    g.colsClosed = true;
    g.color = color;
    g.creaseDeg = 89.0;   // (a near-box keeps soft edges: they are the shape)
    for (int i = 0; i <= stacks; ++i) {
        const double eta = -(PI * 0.5) + PI * i / stacks;
        g.v.push_back(static_cast<double>(i) / stacks);
        for (int j = 0; j < slices; ++j) {
            const double w = TAU * j / slices;
            const double ce = spow(std::cos(eta), e1);
            g.p.push_back(Vec3(r.x * ce * spow(std::cos(w), e2), r.y * spow(std::sin(eta), e1), r.z * ce * spow(std::sin(w), e2)));
        }
    }
    for (int j = 0; j <= slices; ++j) g.u.push_back(static_cast<double>(j) / slices);
    g.hint = [&](int b, int q) { return g.at(b, q) + g.at(b, q + 1) + g.at(b + 1, q) + g.at(b + 1, q + 1); };
    g.emit(m);
    return m;
}

// --- MODIFIERS ------------------------------------------------------------------------------------------------------

void MeshBuilder::warp(RenderMesh& mesh, const std::function<Vec3(const Vec3&)>& fn, double h) {
    long flipped = 0;
    for (Vertex& v : mesh.vertices) {
        const Vec3 p = v.position;
        // the Jacobian's columns: how the warp moves along x, y and z here
        const Vec3 a = (fn(p + Vec3(h, 0, 0)) - fn(p - Vec3(h, 0, 0))) * (0.5 / h);
        const Vec3 b = (fn(p + Vec3(0, h, 0)) - fn(p - Vec3(0, h, 0))) * (0.5 / h);
        const Vec3 c = (fn(p + Vec3(0, 0, h)) - fn(p - Vec3(0, 0, h))) * (0.5 / h);
        const double det = dot(a, cross(b, c));
        // a normal goes by the inverse transpose: the cofactors, signed by the determinant so it keeps facing out
        Vec3 n = cross(b, c) * v.normal.x + cross(c, a) * v.normal.y + cross(a, b) * v.normal.z;
        if (det < 0) { n = n * -1.0; ++flipped; }
        const Vec3 t = a * v.tangent.x + b * v.tangent.y + c * v.tangent.z;
        v.position = fn(p);
        if (n.lengthSquared() > 1e-24) v.normal = normalize(n);
        if (t.lengthSquared() > 1e-24) v.tangent = normalize(t);
    }
    // turned inside out (most of it): the winding turns with it, so the faces still face out
    if (2 * flipped > static_cast<long>(mesh.vertices.size()))
        for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) std::swap(mesh.indices[t + 1], mesh.indices[t + 2]);
}

void MeshBuilder::bend(RenderMesh& mesh, double angle, double length) {
    if (std::fabs(angle) < 1e-9 || length <= 0) return;
    const double R = length / angle;
    warp(mesh, [=](const Vec3& p) {
        if (p.y <= 0) return p;
        const double y = std::min(p.y, length), th = angle * y / length, over = p.y - y;
        const double ct = std::cos(th), st = std::sin(th);
        return Vec3(R - (R - p.x) * ct + over * st, (R - p.x) * st + over * ct, p.z);
    }, 1e-5 * std::max(1.0, length));
}

void MeshBuilder::twist(RenderMesh& mesh, double k) {
    if (std::fabs(k) < 1e-12) return;
    warp(mesh, [=](const Vec3& p) {
        const double a = k * p.y, c = std::cos(a), s = std::sin(a);
        return Vec3(p.x * c - p.z * s, p.y, p.x * s + p.z * c);
    });
}

void MeshBuilder::taper(RenderMesh& mesh, double topScale, double height) {
    if (height <= 0) return;
    warp(mesh, [=](const Vec3& p) {
        const double k = 1.0 + (topScale - 1.0) * std::clamp(p.y / height, 0.0, 1.0);
        return Vec3(p.x * k, p.y, p.z * k);
    });
}

RenderMesh MeshBuilder::mirrored(const RenderMesh& mesh, const Vec3& nIn, double d) {
    RenderMesh m = mesh;
    const Vec3 n = normalize(nIn);
    for (Vertex& v : m.vertices) {
        v.position = v.position - n * (2.0 * (dot(n, v.position) - d));
        v.normal = v.normal - n * (2.0 * dot(n, v.normal));
        v.tangent = v.tangent - n * (2.0 * dot(n, v.tangent));
    }
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) std::swap(m.indices[t + 1], m.indices[t + 2]);
    return m;
}

RenderMesh MeshBuilder::array(const RenderMesh& mesh, int count, const Mat4& step) {
    RenderMesh out;
    Mat4 xf = Mat4::identity();
    for (int k = 0; k < count; ++k) {
        appendTransformed(out, mesh, xf);
        xf = step * xf;
    }
    return out;
}

RenderMesh MeshBuilder::arrayAlong(const RenderMesh& mesh, const MeshPath& pathIn, int count, bool align, const Vec3& up) {
    RenderMesh out;
    if (pathIn.pts.size() < 2 || count < 1) return out;
    std::vector<Vec3> dense = pathIn.pts;
    if (pathIn.closed) dense.push_back(pathIn.pts.front());
    MeshPath at;
    at.closed = pathIn.closed;
    at.pts = count == 1 ? std::vector<Vec3>{dense.front()} : evenlyAlong(dense, count + (pathIn.closed ? 1 : 0));
    if (pathIn.closed && at.pts.size() > 1) at.pts.pop_back();
    const std::vector<CurveFrame> fr = at.pts.size() >= 2 ? sweepFrames(at, up) : std::vector<CurveFrame>{};
    for (std::size_t i = 0; i < at.pts.size(); ++i) {
        Mat4 xf = Mat4::translate(at.pts[i].x, at.pts[i].y, at.pts[i].z);
        if (align && i < fr.size()) {
            const CurveFrame& f = fr[i];
            xf.m[0][0] = f.right.x; xf.m[1][0] = f.right.y; xf.m[2][0] = f.right.z;
            xf.m[0][1] = f.up.x;    xf.m[1][1] = f.up.y;    xf.m[2][1] = f.up.z;
            xf.m[0][2] = f.tangent.x; xf.m[1][2] = f.tangent.y; xf.m[2][2] = f.tangent.z;
        }
        appendTransformed(out, mesh, xf);
    }
    return out;
}

}  // namespace engine
