#include "road_offset.h"

#include <cmath>

namespace engine {

std::vector<Vec2> offsetPolyline(const std::vector<Vec2>& cl, double d, double miterLimit) {
    const int n = static_cast<int>(cl.size());
    std::vector<Vec2> out;
    if (n < 2) return out;
    out.reserve(n);

    // Left normal of segment i (perp of its unit tangent); the +d offset direction.
    auto segNormal = [&](int i) -> Vec2 {
        Vec2 t = cl[i + 1] - cl[i];
        double L = t.length();
        return (L > 1e-12) ? perp(t / L) : Vec2(0, 0);
    };

    out.push_back(cl[0] + segNormal(0) * d);              // start cap: end-segment normal
    for (int i = 1; i + 1 < n; ++i) {
        Vec2 nPrev = segNormal(i - 1), nNext = segNormal(i);
        Vec2 bis = nPrev + nNext;
        double bl = bis.length();
        if (bl < 1e-9) {                                  // ~180 deg reversal: fall back to nPrev
            out.push_back(cl[i] + nPrev * d);
            continue;
        }
        bis = bis / bl;
        double cosHalf = dot(bis, nPrev);                 // = cos(half the turn) in (0,1]
        double miter = (std::fabs(cosHalf) > 1e-6) ? d / cosHalf : d;
        double cap = miterLimit * std::fabs(d);           // clamp the spike at a sharp corner
        if (std::fabs(miter) > cap) miter = (miter < 0 ? -cap : cap);
        out.push_back(cl[i] + bis * miter);
    }
    out.push_back(cl[n - 1] + segNormal(n - 2) * d);      // end cap
    return out;
}

Poly2 ribbonOutline(const std::vector<Vec2>& cl, double halfWidth, double miterLimit) {
    Poly2 poly;
    if (cl.size() < 2 || halfWidth <= 0.0) return poly;
    std::vector<Vec2> left = offsetPolyline(cl, +halfWidth, miterLimit);
    std::vector<Vec2> right = offsetPolyline(cl, -halfWidth, miterLimit);
    poly.reserve(left.size() + right.size());
    for (const Vec2& p : left) poly.push_back(p);                 // left rail, forward
    for (auto it = right.rbegin(); it != right.rend(); ++it) poly.push_back(*it);   // right rail, back
    if (signedArea(poly) < 0) ensureCCW(poly);                   // canonical CCW winding
    return poly;
}

}  // namespace engine
