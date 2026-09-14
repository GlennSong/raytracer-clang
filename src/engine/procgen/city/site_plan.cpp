#include "site_plan.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace engine {

LotSide lotSideOf(const Vec2& outwardNormal, const Vec2& frontage) {
    const Real d = dot(outwardNormal, frontage);
    if (d >= 0.7071) return LotSide::Front;
    if (d <= -0.7071) return LotSide::Rear;
    return LotSide::Side;
}

namespace {
Poly2 ccwCopy(const Poly2& lot) {
    Poly2 p = lot;
    ensureCCW(p);
    return p;
}
// Outward normal of the CCW polygon's edge i (a -> b): the interior lies to the
// left of the walk, so outward is the right-hand normal.
Vec2 outwardNormal(const Poly2& ccw, std::size_t i) {
    const Vec2 a = ccw[i], b = ccw[(i + 1) % ccw.size()];
    const Vec2 d = b - a;
    const Real len = d.length();
    if (len < 1e-9) return {0, 0};
    return {d.y / len, -d.x / len};
}
Real distToSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const Real len2 = ab.lengthSquared();
    Real t = len2 > 1e-12 ? dot(p - a, ab) / len2 : 0.0;
    t = std::max(Real(0), std::min(Real(1), t));
    return (p - (a + ab * t)).length();
}
}  // namespace

SiteFrame siteFrame(const Poly2& lotIn, const Vec2& frontageIn) {
    SiteFrame f;
    const Poly2 lot = ccwCopy(lotIn);
    if (lot.size() < 3) return f;
    Vec2 frontage = frontageIn;
    if (frontage.length() < 1e-9) frontage = {0, -1};
    frontage = normalize(frontage);
    // The frontage edge: longest of those facing the street, else best aligned.
    std::size_t best = lot.size();
    Real bestLen = -1, bestDot = -2;
    for (std::size_t i = 0; i < lot.size(); ++i) {
        const Vec2 n = outwardNormal(lot, i);
        const Real d = dot(n, frontage);
        const Real len = (lot[(i + 1) % lot.size()] - lot[i]).length();
        if (d >= 0.7) {
            if (bestDot < 0.7 || len > bestLen) { best = i; bestLen = len; bestDot = d; }
        } else if (bestDot < 0.7 && d > bestDot) {
            best = i; bestLen = len; bestDot = d;
        }
    }
    if (best == lot.size()) return f;
    const Vec2 a = lot[best], b = lot[(best + 1) % lot.size()];
    f.origin = a;
    f.u = normalize(b - a);
    f.v = perp(f.u);   // the left normal: inward for a CCW walk
    return f;
}

Poly2 largestAlignedRect(const Poly2& lotIn, const SiteFrame& frame, const Yards& yards, Real cell,
                         Real minSide) {
    Poly2 out;
    const Poly2 lot = ccwCopy(lotIn);
    if (lot.size() < 3 || cell <= 0) return out;
    // Per-edge yard by which side of the lot the edge is on.
    const Vec2 frontage = frame.v * -1.0;
    std::vector<Real> yard(lot.size());
    for (std::size_t i = 0; i < lot.size(); ++i) {
        switch (lotSideOf(outwardNormal(lot, i), frontage)) {
            case LotSide::Front: yard[i] = yards.front; break;
            case LotSide::Rear:  yard[i] = yards.rear;  break;
            case LotSide::Side:  yard[i] = yards.side;  break;
        }
    }
    auto usable = [&](const Vec2& w) {
        if (!pointInPolygon(lot, w)) return false;
        for (std::size_t i = 0; i < lot.size(); ++i) {
            if (yard[i] <= 0) continue;
            if (distToSegment(w, lot[i], lot[(i + 1) % lot.size()]) < yard[i]) return false;
        }
        return true;
    };
    // The raster over the lot's frame bounds.
    Real fx0 = std::numeric_limits<Real>::infinity(), fy0 = fx0, fx1 = -fx0, fy1 = -fx0;
    for (const Vec2& w : lot) {
        const Vec2 q = frame.toFrame(w);
        fx0 = std::min(fx0, q.x); fx1 = std::max(fx1, q.x);
        fy0 = std::min(fy0, q.y); fy1 = std::max(fy1, q.y);
    }
    const int nx = static_cast<int>(std::ceil((fx1 - fx0) / cell));
    const int ny = static_cast<int>(std::ceil((fy1 - fy0) / cell));
    if (nx < 1 || ny < 1 || static_cast<long long>(nx) * ny > 4000000LL) return out;
    std::vector<unsigned char> mask(static_cast<std::size_t>(nx) * ny, 0);
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i)
            mask[static_cast<std::size_t>(j) * nx + i] =
                usable(frame.toWorld({fx0 + (i + 0.5) * cell, fy0 + (j + 0.5) * cell})) ? 1 : 0;
    // Largest rectangle of 1s: per row, a histogram of consecutive usable
    // cells above, and the classic stack scan for the widest bar span.
    std::vector<int> height(nx, 0), stack;
    int bestArea = 0, bi0 = 0, bi1 = -1, bj0 = 0, bj1 = -1;
    const int minCells = std::max(1, static_cast<int>(std::ceil(minSide / cell)));
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i)
            height[i] = mask[static_cast<std::size_t>(j) * nx + i] ? height[i] + 1 : 0;
        stack.clear();
        for (int i = 0; i <= nx; ++i) {
            const int h = i < nx ? height[i] : 0;
            while (!stack.empty() && height[stack.back()] >= h) {
                const int top = stack.back();
                stack.pop_back();
                const int hh = height[top];
                const int left = stack.empty() ? 0 : stack.back() + 1;
                const int width = i - left;
                if (hh >= minCells && width >= minCells && hh * width > bestArea) {
                    bestArea = hh * width;
                    bi0 = left; bi1 = i - 1;
                    bj0 = j - hh + 1; bj1 = j;
                }
            }
            stack.push_back(i);
        }
    }
    if (bestArea <= 0) return out;
    // The rectangle spans the usable cell CENTRES (half a cell inside the
    // exact edge on every side).
    const Real x0 = fx0 + (bi0 + 0.5) * cell, x1 = fx0 + (bi1 + 0.5) * cell;
    const Real y0 = fy0 + (bj0 + 0.5) * cell, y1 = fy0 + (bj1 + 0.5) * cell;
    if (x1 - x0 < minSide - cell || y1 - y0 < minSide - cell) return out;
    out = {frame.toWorld({x0, y0}), frame.toWorld({x1, y0}), frame.toWorld({x1, y1}),
           frame.toWorld({x0, y1})};
    ensureCCW(out);
    return out;
}

const char* openKindName(OpenKind k) {
    switch (k) {
        case OpenKind::Forecourt: return "forecourt";
        case OpenKind::SideYard:  return "side_yard";
        case OpenKind::RearYard:  return "rear_yard";
        case OpenKind::Courtyard: return "courtyard";
        case OpenKind::Plaza:     return "plaza";
        case OpenKind::Lawn:      return "lawn";
        case OpenKind::Paving:    return "paving";
        case OpenKind::Parking:   return "parking";
    }
    return "open";
}

namespace {
// Cut `poly` by the line through frame point `p` with direction `dir`, keeping
// the piece whose centroid's frame coordinate along `axisIsV ? v : u` is on the
// `wantBelow` side of the cut value. The other piece goes to `rest`.
Poly2 keepSide(const Poly2& poly, const SiteFrame& f, const Vec2& pWorld, const Vec2& dirWorld,
               bool axisIsV, Real cut, bool wantBelow, Poly2& rest) {
    Poly2 a, b;
    splitByLine(poly, pWorld, dirWorld, a, b);
    auto coord = [&](const Poly2& q) {
        if (q.size() < 3) return std::numeric_limits<Real>::quiet_NaN();
        const Vec2 c = f.toFrame(centroid(q));
        return axisIsV ? c.y : c.x;
    };
    const Real ca = coord(a), cb = coord(b);
    const bool aBelow = !std::isnan(ca) && ca < cut;
    const bool bBelow = !std::isnan(cb) && cb < cut;
    if (wantBelow) {
        if (aBelow) { rest = b; return a; }
        if (bBelow) { rest = a; return b; }
    } else {
        if (!std::isnan(ca) && !aBelow) { rest = b; return a; }
        if (!std::isnan(cb) && !bBelow) { rest = a; return b; }
    }
    rest = poly;
    return {};
}
}  // namespace

std::vector<OpenSpace> openSpacePieces(const Poly2& lotIn, const Poly2& mass, const SiteFrame& f,
                                       Real minArea) {
    std::vector<OpenSpace> out;
    const Poly2 lot = ccwCopy(lotIn);
    if (lot.size() < 3 || mass.size() < 3) return out;
    Real x0 = std::numeric_limits<Real>::infinity(), y0 = x0, x1 = -x0, y1 = -x0;
    for (const Vec2& w : mass) {
        const Vec2 q = f.toFrame(w);
        x0 = std::min(x0, q.x); x1 = std::max(x1, q.x);
        y0 = std::min(y0, q.y); y1 = std::max(y1, q.y);
    }
    auto push = [&](Poly2 piece, OpenKind kind) {
        if (piece.size() < 3 || area(piece) < minArea) return;
        ensureCCW(piece);
        out.push_back({std::move(piece), kind});
    };
    Poly2 rest;
    push(keepSide(lot, f, f.toWorld({0, y0}), f.u, true, y0, true, rest), OpenKind::Forecourt);
    Poly2 rest2;
    push(keepSide(rest, f, f.toWorld({0, y1}), f.u, true, y1, false, rest2), OpenKind::RearYard);
    Poly2 rest3;
    push(keepSide(rest2, f, f.toWorld({x0, 0}), f.v, false, x0, true, rest3), OpenKind::SideYard);
    Poly2 rest4;
    push(keepSide(rest3, f, f.toWorld({x1, 0}), f.v, false, x1, false, rest4), OpenKind::SideYard);
    return out;
}

SitePlan planSite(const Poly2& lot, const Vec2& frontage, const Yards& yards, Real cell,
                  Real minSide) {
    SitePlan sp;
    sp.frame = siteFrame(lot, frontage);
    sp.buildable = largestAlignedRect(lot, sp.frame, yards, cell, minSide);
    if (!sp.buildable.empty()) sp.open = openSpacePieces(lot, sp.buildable, sp.frame);
    return sp;
}

}  // namespace engine
