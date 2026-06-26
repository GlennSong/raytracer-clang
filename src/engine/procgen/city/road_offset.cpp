#include "road_offset.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace engine {

namespace {

constexpr double kEps = 1e-7;

// Parameter t along p1->p2 where a point q (assumed near the line) projects.
double paramOnSeg(const Vec2& p1, const Vec2& p2, const Vec2& q) {
    Vec2 d = p2 - p1; double L2 = d.lengthSquared();
    return (L2 > 1e-18) ? dot(q - p1, d) / L2 : 0.0;
}

// Is q on segment p1p2 (within tol), strictly interior in the parameter sense?
bool pointOnSegInterior(const Vec2& p1, const Vec2& p2, const Vec2& q, double tol) {
    double t = paramOnSeg(p1, p2, q);
    if (t < kEps || t > 1.0 - kEps) return false;
    Vec2 proj = p1 + (p2 - p1) * t;
    return (q - proj).length() < tol;
}

bool pointInTri(const Vec2& p, const Vec2& a, const Vec2& b, const Vec2& c) {
    double d1 = cross(b - a, p - a), d2 = cross(c - b, p - b), d3 = cross(a - c, p - c);
    bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0), pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(neg && pos);
}

// Reflex vertex of a CCW polygon (interior angle > 180).
bool isReflexCCW(const Poly2& P, int i) {
    int n = static_cast<int>(P.size());
    Vec2 a = P[(i + n - 1) % n], b = P[i], c = P[(i + 1) % n];
    return cross(b - a, c - b) < 0;
}

// Bridge ONE hole into `outer` (outer CCW, hole CW): cut from a mutually-visible outer vertex
// to the hole's rightmost vertex and back (Eberly's algorithm).
Poly2 bridgeOneHole(const Poly2& outer, const Poly2& hole) {
    const int hn = static_cast<int>(hole.size());
    const int on = static_cast<int>(outer.size());
    if (hn < 3 || on < 3) return outer;

    int mi = 0;                                          // hole's rightmost vertex M
    for (int i = 1; i < hn; ++i) if (hole[i].x > hole[mi].x) mi = i;
    Vec2 M = hole[mi];

    int bestK = -1; double bestX = 1e30; Vec2 I(0, 0);   // nearest outer edge the +x ray hits
    for (int k = 0; k < on; ++k) {
        Vec2 a = outer[k], b = outer[(k + 1) % on];
        if ((a.y > M.y) == (b.y > M.y)) continue;        // edge must straddle the ray
        double t = (M.y - a.y) / (b.y - a.y);
        double x = a.x + t * (b.x - a.x);
        if (x >= M.x - 1e-9 && x < bestX) { bestX = x; bestK = k; I = Vec2(x, M.y); }
    }
    if (bestK < 0) return outer;                         // no edge to the right: give up on this hole

    int vi = (outer[bestK].x >= outer[(bestK + 1) % on].x) ? bestK : (bestK + 1) % on;
    Vec2 cand = outer[vi];
    double bestAng = 1e30;                               // refine: a reflex vertex may block the view
    for (int j = 0; j < on; ++j) {
        if (!isReflexCCW(outer, j) || outer[j].x <= M.x) continue;
        if (pointInTri(outer[j], M, I, cand)) {
            double ang = std::atan2(std::fabs(outer[j].y - M.y), outer[j].x - M.x);
            if (ang < bestAng) { bestAng = ang; vi = j; }
        }
    }

    Poly2 merged;
    merged.reserve(on + hn + 2);
    for (int k = 0; k <= vi; ++k) merged.push_back(outer[k]);
    for (int s = 0; s < hn; ++s) merged.push_back(hole[(mi + s) % hn]);   // the hole loop (CW)
    merged.push_back(hole[mi]);                          // back to M
    merged.push_back(outer[vi]);                         // ... and across the bridge
    for (int k = vi + 1; k < on; ++k) merged.push_back(outer[k]);
    return merged;
}

// Proper interior crossing of a->b and c->d; on a hit, `t` is along a->b (interior of both).
bool segCrossParam(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d, double& t) {
    Vec2 r = b - a, s = d - c;
    double denom = cross(r, s);
    if (std::fabs(denom) < 1e-12) return false;          // parallel/collinear
    Vec2 ac = c - a;
    t = cross(ac, s) / denom;
    double u = cross(ac, r) / denom;
    return t > kEps && t < 1.0 - kEps && u > kEps && u < 1.0 - kEps;
}

}  // namespace

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

void ringRibbon(const std::vector<Vec2>& cl, double halfWidth, Poly2& outer, Poly2& inner,
                double miterLimit) {
    outer.clear(); inner.clear();
    std::vector<Vec2> pts = cl;
    if (pts.size() >= 2 && (pts.front() - pts.back()).length() < 1e-6) pts.pop_back();
    const int n = static_cast<int>(pts.size());
    if (n < 3 || halfWidth <= 0.0) return;
    auto segN = [&](int i) {                              // left normal of segment i -> i+1 (wraps)
        Vec2 t = pts[(i + 1) % n] - pts[i]; double L = t.length();
        return (L > 1e-12) ? perp(t / L) : Vec2(0, 0);
    };
    Poly2 plus, minus;                                   // the +hw and -hw offset rings
    for (int i = 0; i < n; ++i) {
        Vec2 nPrev = segN((i + n - 1) % n), nNext = segN(i);
        Vec2 bis = nPrev + nNext; double bl = bis.length();
        Vec2 off = nPrev; double scale = 1.0;            // ~180° fold: fall back to prev normal
        if (bl >= 1e-9) {
            bis = bis / bl; double cosH = dot(bis, nPrev);
            scale = (std::fabs(cosH) > 1e-6) ? 1.0 / cosH : 1.0; off = bis;
        }
        if (scale > miterLimit) scale = miterLimit;
        plus.push_back(pts[i] + off * (halfWidth * scale));
        minus.push_back(pts[i] - off * (halfWidth * scale));
    }
    // The larger-area ring is the outer rail (force CCW); the smaller is the island (force CW hole).
    if (std::fabs(signedArea(plus)) >= std::fabs(signedArea(minus))) { outer = plus; inner = minus; }
    else { outer = minus; inner = plus; }
    if (signedArea(outer) < 0) std::reverse(outer.begin(), outer.end());
    if (signedArea(inner) > 0) std::reverse(inner.begin(), inner.end());
}

std::vector<Poly2> polygonUnion(const std::vector<Poly2>& polys) {
    struct Edge { Vec2 a, b; int poly; };
    std::vector<Edge> E;
    for (int p = 0; p < static_cast<int>(polys.size()); ++p) {
        const Poly2& P = polys[p];
        int n = static_cast<int>(P.size());
        if (n < 3) continue;
        for (int i = 0; i < n; ++i) E.push_back({P[i], P[(i + 1) % n], p});
    }
    const double tol = 1e-4;

    // Split every edge at its crossings (and T-touches) with other polygons' edges; keep the
    // sub-edges whose midpoint is NOT inside another polygon (those are on the union boundary).
    std::vector<std::pair<Vec2, Vec2>> kept;
    for (const Edge& e : E) {
        std::vector<double> ts = {0.0, 1.0};
        for (const Edge& o : E) {
            if (o.poly == e.poly) continue;
            double t;
            if (segCrossParam(e.a, e.b, o.a, o.b, t)) ts.push_back(t);
            if (pointOnSegInterior(e.a, e.b, o.a, tol)) ts.push_back(paramOnSeg(e.a, e.b, o.a));
            if (pointOnSegInterior(e.a, e.b, o.b, tol)) ts.push_back(paramOnSeg(e.a, e.b, o.b));
        }
        std::sort(ts.begin(), ts.end());
        for (int k = 0; k + 1 < static_cast<int>(ts.size()); ++k) {
            double t0 = ts[k], t1 = ts[k + 1];
            if (t1 - t0 < kEps) continue;
            Vec2 A = e.a + (e.b - e.a) * t0, B = e.a + (e.b - e.a) * t1;
            Vec2 mid = (A + B) * 0.5;
            bool interior = false;
            for (int p = 0; p < static_cast<int>(polys.size()); ++p) {
                if (p == e.poly) continue;
                if (pointInPolygon(polys[p], mid)) { interior = true; break; }
            }
            if (!interior) kept.push_back({A, B});
        }
    }

    // Chain the directed boundary sub-edges into loops (vertices keyed on a tol grid; at a
    // shared vertex pick the most-clockwise continuation to hug the boundary).
    auto key = [&](const Vec2& v) {
        return std::make_pair(static_cast<long long>(std::llround(v.x / tol)),
                              static_cast<long long>(std::llround(v.y / tol)));
    };
    std::map<std::pair<long long, long long>, std::vector<int>> byStart;
    for (int i = 0; i < static_cast<int>(kept.size()); ++i) byStart[key(kept[i].first)].push_back(i);

    std::vector<char> used(kept.size(), 0);
    std::vector<Poly2> loops;
    for (int s = 0; s < static_cast<int>(kept.size()); ++s) {
        if (used[s]) continue;
        Poly2 loop;
        int cur = s;
        while (cur != -1 && !used[cur]) {
            used[cur] = 1;
            loop.push_back(kept[cur].first);
            Vec2 endp = kept[cur].second;
            Vec2 din = normalize(kept[cur].second - kept[cur].first);
            int next = -1; double best = -1e9;
            auto it = byStart.find(key(endp));
            if (it != byStart.end()) {
                for (int cand : it->second) {
                    if (used[cand]) continue;
                    Vec2 dout = normalize(kept[cand].second - kept[cand].first);
                    double ang = std::atan2(cross(din, dout), dot(din, dout));  // signed turn
                    double score = -ang;                  // most clockwise = largest score
                    if (score > best) { best = score; next = cand; }
                }
            }
            if (next == s) break;                          // closed
            cur = next;
        }
        if (loop.size() >= 3) loops.push_back(loop);
    }
    return loops;
}

Poly2 bridgeHoles(const Poly2& outerIn, const std::vector<Poly2>& holesIn) {
    Poly2 outer = outerIn;
    if (signedArea(outer) < 0) std::reverse(outer.begin(), outer.end());     // outer CCW
    std::vector<Poly2> holes;
    holes.reserve(holesIn.size());
    for (Poly2 h : holesIn) {
        if (h.size() < 3) continue;
        if (signedArea(h) > 0) std::reverse(h.begin(), h.end());             // holes CW
        holes.push_back(std::move(h));
    }
    auto maxX = [](const Poly2& p) { double m = -1e30; for (const Vec2& v : p) m = std::max(m, v.x); return m; };
    std::sort(holes.begin(), holes.end(),                                    // rightmost hole first
              [&](const Poly2& a, const Poly2& b) { return maxX(a) > maxX(b); });
    Poly2 merged = outer;
    for (const Poly2& h : holes) merged = bridgeOneHole(merged, h);
    return merged;
}

}  // namespace engine
