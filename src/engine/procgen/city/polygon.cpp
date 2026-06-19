#include "polygon.h"

#include <algorithm>

namespace engine {

Real signedArea(const Poly2& poly) {
    const std::size_t n = poly.size();
    if (n < 3) return 0;
    Real sum = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[(i + 1) % n];
        sum += a.x * b.y - b.x * a.y;
    }
    return sum * 0.5;
}

void ensureCCW(Poly2& poly) {
    if (signedArea(poly) < 0) std::reverse(poly.begin(), poly.end());
}

Vec2 centroid(const Poly2& poly) {
    const std::size_t n = poly.size();
    if (n == 0) return {};
    Real a2 = signedArea(poly) * 2.0;     // 2*area
    if (std::abs(a2) < 1e-12) {           // degenerate: vertex mean
        Vec2 m;
        for (const Vec2& p : poly) m += p;
        return m / static_cast<Real>(n);
    }
    Vec2 c;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[(i + 1) % n];
        Real w = a.x * b.y - b.x * a.y;
        c += (a + b) * w;
    }
    return c / (3.0 * a2);
}

Real perimeter(const Poly2& poly) {
    const std::size_t n = poly.size();
    if (n < 2) return 0;
    Real sum = 0;
    for (std::size_t i = 0; i < n; ++i)
        sum += distance(poly[i], poly[(i + 1) % n]);
    return sum;
}

void bounds(const Poly2& poly, Vec2& lo, Vec2& hi) {
    if (poly.empty()) { lo = hi = {}; return; }
    lo = hi = poly[0];
    for (const Vec2& p : poly) {
        lo.x = std::min(lo.x, p.x); lo.y = std::min(lo.y, p.y);
        hi.x = std::max(hi.x, p.x); hi.y = std::max(hi.y, p.y);
    }
}

bool pointInPolygon(const Poly2& poly, const Vec2& p) {
    const std::size_t n = poly.size();
    if (n < 3) return false;
    bool inside = false;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[j];
        bool crosses = ((a.y > p.y) != (b.y > p.y));
        if (crosses) {
            Real xCross = (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x;
            if (p.x < xCross) inside = !inside;
        }
    }
    return inside;
}

Poly2 convexHull(const Poly2& points) {
    Poly2 pts = points;
    std::sort(pts.begin(), pts.end(), [](const Vec2& a, const Vec2& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    pts.erase(std::unique(pts.begin(), pts.end(), [](const Vec2& a, const Vec2& b) {
                  return a.x == b.x && a.y == b.y;
              }),
              pts.end());
    const std::size_t n = pts.size();
    if (n < 3) return pts;

    Poly2 hull(2 * n);
    std::size_t k = 0;
    // Lower hull.
    for (std::size_t i = 0; i < n; ++i) {
        while (k >= 2 && cross(hull[k - 1] - hull[k - 2], pts[i] - hull[k - 2]) <= 0) --k;
        hull[k++] = pts[i];
    }
    // Upper hull.
    const std::size_t lower = k + 1;
    for (std::size_t i = n - 1; i-- > 0;) {
        while (k >= lower && cross(hull[k - 1] - hull[k - 2], pts[i] - hull[k - 2]) <= 0) --k;
        hull[k++] = pts[i];
    }
    hull.resize(k - 1);
    return hull;   // CCW
}

OBB2 orientedBoundingBox(const Poly2& poly) {
    OBB2 obb;
    if (poly.size() < 2) {
        if (!poly.empty()) obb.center = poly[0];
        return obb;
    }
    Poly2 hull = convexHull(poly);
    if (hull.size() < 2) hull = poly;

    Real bestArea = -1;
    const std::size_t n = hull.size();
    for (std::size_t i = 0; i < n; ++i) {
        Vec2 edge = normalize(hull[(i + 1) % n] - hull[i]);
        if (edge.lengthSquared() < 1e-18) continue;
        Vec2 e0 = edge, e1 = perp(edge);
        Real minU = 1e30, maxU = -1e30, minV = 1e30, maxV = -1e30;
        for (const Vec2& p : hull) {
            Real u = dot(p, e0), v = dot(p, e1);
            minU = std::min(minU, u); maxU = std::max(maxU, u);
            minV = std::min(minV, v); maxV = std::max(maxV, v);
        }
        Real a = (maxU - minU) * (maxV - minV);
        if (bestArea < 0 || a < bestArea) {
            bestArea = a;
            Real cu = (minU + maxU) * 0.5, cv = (minV + maxV) * 0.5;
            obb.center = e0 * cu + e1 * cv;
            obb.axis[0] = e0; obb.axis[1] = e1;
            obb.half[0] = (maxU - minU) * 0.5;
            obb.half[1] = (maxV - minV) * 0.5;
        }
    }
    return obb;
}

Poly2 clipHalfPlane(const Poly2& poly, const Vec2& n, Real c) {
    // Keep the side where dot(n, p) <= c (inside).
    Poly2 out;
    const std::size_t count = poly.size();
    if (count == 0) return out;
    auto inside = [&](const Vec2& p) { return dot(n, p) <= c + 1e-12; };
    for (std::size_t i = 0; i < count; ++i) {
        const Vec2& cur = poly[i];
        const Vec2& prv = poly[(i + count - 1) % count];
        bool curIn = inside(cur), prvIn = inside(prv);
        if (curIn != prvIn) {
            Real dPrv = dot(n, prv) - c;
            Real dCur = dot(n, cur) - c;
            Real t = dPrv / (dPrv - dCur);
            out.push_back(lerp(prv, cur, t));
        }
        if (curIn) out.push_back(cur);
    }
    return out;
}

void splitByLine(const Poly2& poly, const Vec2& p, const Vec2& dir,
                 Poly2& left, Poly2& right) {
    Vec2 n = normalize(perp(dir));          // separating-plane normal
    Real c = dot(n, p);
    left = clipHalfPlane(poly, n, c);       // dot(n,·) <= c
    right = clipHalfPlane(poly, -n, -c);    // dot(n,·) >= c
}

Poly2 inset(const Poly2& poly, Real d) {
    const std::size_t n = poly.size();
    if (n < 3 || d <= 0) return poly;
    Poly2 p = poly;
    ensureCCW(p);

    // Each edge i: from p[i] to p[i+1]. For CCW winding the inward normal is the
    // right-hand perpendicular (rotate edge dir by -90°). Offset every edge line
    // inward by d, then intersect consecutive offset lines for the new vertices.
    struct Line { Vec2 pt, dir; };
    std::vector<Line> lines(n);
    for (std::size_t i = 0; i < n; ++i) {
        Vec2 a = p[i], b = p[(i + 1) % n];
        Vec2 dir = normalize(b - a);
        Vec2 inward = {-dir.y, dir.x};      // +90° (left) of edge dir = interior (CCW)
        lines[i] = {a + inward * d, dir};
    }

    Poly2 out(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Line& l0 = lines[(i + n - 1) % n];
        const Line& l1 = lines[i];
        Real denom = cross(l0.dir, l1.dir);
        if (std::abs(denom) < 1e-9) {       // parallel edges: fall back to point
            out[i] = l1.pt;
        } else {
            Real t = cross(l1.pt - l0.pt, l1.dir) / denom;
            out[i] = l0.pt + l0.dir * t;
        }
    }

    // Reject a degenerate inset: collapsed area, flipped winding, or any edge
    // whose direction reversed relative to the original (over-inset past the
    // medial axis — for a square the inner crossover stays CCW, so the area test
    // alone misses it). The caller drops such slivers.
    if (area(out) < 1e-6 || signedArea(out) <= 0) return {};
    for (std::size_t i = 0; i < n; ++i) {
        Vec2 oldDir = p[(i + 1) % n] - p[i];
        Vec2 newDir = out[(i + 1) % n] - out[i];
        if (dot(oldDir, newDir) < 0) return {};
    }
    return out;
}

}  // namespace engine
