#include "patch_fabric.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace engine {
namespace {

double polyArea(const Poly2& p) {
    double a = 0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec2& u = p[i];
        const Vec2& v = p[(i + 1) % p.size()];
        a += u.x * v.y - v.x * u.y;
    }
    return a * 0.5;
}

bool pointInPoly(const Poly2& poly, const Vec2& p) {
    bool in = false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[j];
        if ((a.y > p.y) != (b.y > p.y)) {
            double t = (p.y - a.y) / (b.y - a.y);
            if (p.x < a.x + (b.x - a.x) * t) in = !in;
        }
    }
    return in;
}

double distToBoundary(const Poly2& poly, const Vec2& p) {
    double best = 1e30;
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[(i + 1) % poly.size()];
        Vec2 ab = b - a;
        double len2 = ab.lengthSquared();
        double t = len2 > 1e-12 ? dot(p - a, ab) / len2 : 0.0;
        t = std::clamp(t, 0.0, 1.0);
        Vec2 q(a.x + ab.x * t, a.y + ab.y * t);
        best = std::min(best, (p - q).length());
    }
    return best;
}

// Densify the boundary to bounded segment length so offsets bend smoothly.
Poly2 resampleBoundary(const Poly2& in, double maxSeg) {
    Poly2 out;
    for (std::size_t i = 0; i < in.size(); ++i) {
        const Vec2& a = in[i];
        const Vec2& b = in[(i + 1) % in.size()];
        double len = (b - a).length();
        int n = std::max(1, static_cast<int>(std::ceil(len / maxSeg)));
        for (int k = 0; k < n; ++k)
            out.push_back(a + (b - a) * (static_cast<double>(k) / n));
    }
    return out;
}

// Outward normal convention: CCW polygon -> interior is left of travel, so
// the INWARD normal of edge dir (dx,dy) is (-dy,dx) flipped by orientation.
Vec2 inwardNormal(const Vec2& dir, bool ccw) {
    Vec2 n(-dir.y, dir.x);
    return ccw ? n : n * -1.0;
}

// ---- deterministic hashing (seeded, geometry-keyed, order-free) -----------

uint32_t mix32(uint32_t x) {
    x ^= x >> 16; x *= 0x7feb352dU;
    x ^= x >> 15; x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}
uint32_t hashCombine(uint32_t h, uint32_t v) {
    return mix32(h ^ (v + 0x9e3779b9u + (h << 6) + (h >> 2)));
}
uint32_t hashVec(uint32_t h, const Vec2& v) {
    h = hashCombine(h, static_cast<uint32_t>(
                           static_cast<int64_t>(std::llround(v.x * 4.0))));
    h = hashCombine(h, static_cast<uint32_t>(
                           static_cast<int64_t>(std::llround(v.y * 4.0))));
    return h;
}
double hash01(uint32_t h) { return (mix32(h) >> 8) * (1.0 / 16777216.0); }

// ---- SIDE CURVES: arc-length parameterized boundary chains ----------------

struct SideCurve {
    std::vector<Vec2> pts;      // corner -> corner, face order
    std::vector<double> cum;    // cumulative arc length
    double len = 0;

    void finish() {
        cum.resize(pts.size());
        cum[0] = 0;
        for (std::size_t i = 1; i < pts.size(); ++i)
            cum[i] = cum[i - 1] + (pts[i] - pts[i - 1]).length();
        len = cum.back();
    }
    Vec2 at(double t) const {
        if (pts.size() < 2) return pts.empty() ? Vec2(0, 0) : pts[0];
        double s = std::clamp(t, 0.0, 1.0) * len;
        std::size_t i = 1;
        while (i + 1 < pts.size() && cum[i] < s) ++i;
        double seg = cum[i] - cum[i - 1];
        double u = seg > 1e-12 ? (s - cum[i - 1]) / seg : 0.0;
        return pts[i - 1] + (pts[i] - pts[i - 1]) * u;
    }
};

// Split a side at arc parameter t into two sides (the split point is shared).
void splitSide(const SideCurve& s, double t, SideCurve& a, SideCurve& b) {
    Vec2 m = s.at(t);
    double target = std::clamp(t, 0.0, 1.0) * s.len;
    a.pts.clear();
    b.pts.clear();
    a.pts.push_back(s.pts.front());
    for (std::size_t i = 1; i < s.pts.size(); ++i) {
        if (s.cum[i] < target - 1e-9) {
            a.pts.push_back(s.pts[i]);
        }
    }
    a.pts.push_back(m);
    b.pts.push_back(m);
    for (std::size_t i = 1; i < s.pts.size(); ++i)
        if (s.cum[i] > target + 1e-9) b.pts.push_back(s.pts[i]);
    if ((b.pts.back() - s.pts.back()).length() > 1e-9)
        b.pts.push_back(s.pts.back());
    a.finish();
    b.finish();
}

// CANONICAL STATIONS (plan step 2.5, adapted): spacing = len/div with
// div = floor(len/blockLen) so every gap is >= blockLen by construction;
// phase + jitter hash off the side's GEOMETRY oriented from its
// lexicographically-smallest endpoint — the two patches sharing a chain
// mint identical station positions and planarize welds them into shared
// crossings.
struct Station {
    double t;    // parameter in FACE direction
    Vec2 pos;
};

std::vector<Station> stationsOn(const SideCurve& s,
                                const PatchFabricParams& p) {
    std::vector<Station> out;
    if (s.pts.size() < 2 || s.len <= 0) return out;
    const int div = static_cast<int>(std::floor(s.len / std::max(1.0, p.blockLen)));
    if (div < 2) return out;
    const Vec2 e0 = s.pts.front(), e1 = s.pts.back();
    const bool flip = (e1.x < e0.x) || (e1.x == e0.x && e1.y < e0.y);
    uint32_t h = p.seed;
    h = hashVec(h, flip ? e1 : e0);
    h = hashVec(h, flip ? e0 : e1);
    const double jit = std::clamp(p.jitter, 0.0, 0.25);
    const double clearT =
        (p.cornerClear > 0 ? p.cornerClear : p.blockLen * 0.45) / s.len;
    for (int k = 1; k < div; ++k) {
        double off = (hash01(hashCombine(h, static_cast<uint32_t>(k))) - 0.5) *
                     2.0 * jit;
        double tc = (static_cast<double>(k) + off) / div;   // canonical param
        double tf = flip ? 1.0 - tc : tc;                   // face param
        if (tf < clearT || tf > 1.0 - clearT) continue;
        out.push_back({tf, s.at(tf)});
    }
    std::sort(out.begin(), out.end(),
              [](const Station& a, const Station& b) { return a.t < b.t; });
    return out;
}

// ---- COONS PATCH (round 7): transfinite interpolation over four sides -----
// Frame: u runs along s0 (bottom), v from s0 toward s2 (top). s1 is the right
// boundary, s3 the left (both in face order, so top and left are mirrored).

struct CoonsQuad {
    const SideCurve* s[4];
    Vec2 c0, c1, c2, c3;   // corners: s0 start, s1 start, s2 start, s3 start

    Vec2 eval(double u, double v) const {
        Vec2 B = s[0]->at(u), T = s[2]->at(1.0 - u);
        Vec2 L = s[3]->at(1.0 - v), R = s[1]->at(v);
        Vec2 lin = B * (1.0 - v) + T * v + L * (1.0 - u) + R * u;
        Vec2 bil = c0 * ((1.0 - u) * (1.0 - v)) + c1 * (u * (1.0 - v)) +
                   c3 * ((1.0 - u) * v) + c2 * (u * v);
        return lin - bil;
    }
};

bool segIntersect(const Vec2& A, const Vec2& B, const Vec2& C, const Vec2& D,
                  double& t, double& u) {
    Vec2 r = B - A, sD = D - C;
    double den = r.x * sD.y - r.y * sD.x;
    if (std::fabs(den) < 1e-12) return false;
    Vec2 AC = C - A;
    t = (AC.x * sD.y - AC.y * sD.x) / den;
    u = (AC.x * r.y - AC.y * r.x) / den;
    return t >= -1e-9 && t <= 1.0 + 1e-9 && u >= -1e-6 && u <= 1.0 + 1e-6;
}

double polylineLen(const std::vector<Vec2>& pts) {
    double L = 0;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i)
        L += (pts[i + 1] - pts[i]).length();
    return L;
}

void emitPolyline(const std::vector<Vec2>& pts, const PatchFabricParams& p,
                  std::vector<FabricSegment>& out) {
    for (std::size_t i = 0; i + 1 < pts.size(); ++i)
        if ((pts[i + 1] - pts[i]).length() > 1e-6)
            out.push_back({pts[i], pts[i + 1], p.streetWidth, RoadClass::Local});
}

// The ROUND-9/10 quad fill: through chords per opposite-side pair (blended
// toward the Coons iso-curve by `conform`), round-11 T-terminated extras for
// mismatched counts. `clip` is the quad's own polygon (inside-validation).
void chordsQuad(const SideCurve* sides, const Poly2& clip,
                const PatchFabricParams& p, std::vector<FabricSegment>& out) {
    if (std::fabs(polyArea(clip)) < p.blockLen * p.blockDepth * 1.2) return;

    std::vector<Station> st[4];
    for (int i = 0; i < 4; ++i) st[i] = stationsOn(sides[i], p);

    CoonsQuad cq{{&sides[0], &sides[1], &sides[2], &sides[3]},
                 sides[0].pts.front(), sides[1].pts.front(),
                 sides[2].pts.front(), sides[3].pts.front()};

    const double conform = std::clamp(p.conform, 0.0, 1.0);

    // family 0 (U): between s0 (coord = t) and s2 (coord = 1 - t).
    // family 1 (V): between s3 (coord = 1 - t) and s1 (coord = t).
    auto famCoords = [&](int family, bool topEnd) {
        const std::vector<Station>& src =
            family == 0 ? (topEnd ? st[2] : st[0]) : (topEnd ? st[1] : st[3]);
        std::vector<double> c;
        c.reserve(src.size());
        for (const Station& x : src)
            c.push_back(family == 0 ? (topEnd ? 1.0 - x.t : x.t)
                                    : (topEnd ? x.t : 1.0 - x.t));
        std::sort(c.begin(), c.end());
        return c;
    };

    // A street from coord `a` on the family's near end to coord `b` on its
    // far end: straight chord blended toward the Coons iso-curve, endpoints
    // exact by construction.
    auto buildCurve = [&](int family, double a, double b, double blend) {
        Vec2 p0 = family == 0 ? cq.s[0]->at(a) : cq.s[3]->at(1.0 - a);
        Vec2 p1 = family == 0 ? cq.s[2]->at(1.0 - b) : cq.s[1]->at(b);
        double len = (p1 - p0).length();
        int m = std::clamp(static_cast<int>(std::ceil(len / 60.0)), 2, 10);
        std::vector<Vec2> pts;
        pts.reserve(m + 1);
        pts.push_back(p0);
        for (int k = 1; k < m; ++k) {
            double w = static_cast<double>(k) / m;
            double q = a + (b - a) * w;
            Vec2 chord = p0 + (p1 - p0) * w;
            Vec2 coons = family == 0 ? cq.eval(q, w) : cq.eval(w, q);
            pts.push_back(chord + (coons - chord) * blend);
        }
        pts.push_back(p1);
        return pts;
    };
    auto insideOk = [&](const std::vector<Vec2>& pts) {
        for (std::size_t i = 1; i + 1 < pts.size(); ++i)
            if (!pointInPoly(clip, pts[i])) return false;
        for (std::size_t i = 0; i + 1 < pts.size(); ++i)
            if (!pointInPoly(clip, (pts[i] + pts[i + 1]) * 0.5)) return false;
        return true;
    };

    std::vector<std::vector<Vec2>> through[2];
    std::vector<std::pair<double, bool>> extras[2];   // (coord, fromTopEnd)

    if (std::getenv("RT_FABRIC_DEBUG"))
        std::fprintf(stderr,
                     "  [chordsQuad] sides %.0f/%.0f/%.0f/%.0f st %zu/%zu/%zu/%zu\n",
                     sides[0].len, sides[1].len, sides[2].len, sides[3].len,
                     st[0].size(), st[1].size(), st[2].size(), st[3].size());

    for (int fam = 0; fam < 2; ++fam) {
        std::vector<double> bot = famCoords(fam, false);
        std::vector<double> top = famCoords(fam, true);
        const std::size_t nb = bot.size(), nt = top.size();
        if (nb == 0 && nt == 0) continue;
        if (nb == 0 || nt == 0) {
            const std::vector<double>& only = nb == 0 ? top : bot;
            for (double c : only) extras[fam].push_back({c, nb == 0});
            continue;
        }
        const bool topDense = nt > nb;
        const std::vector<double>& dense = topDense ? top : bot;
        const std::vector<double>& sparse = topDense ? bot : top;
        const std::size_t k = sparse.size(), nd = dense.size();
        const double ratio = static_cast<double>(nd) / static_cast<double>(k);
        // Index-spread pairing keeps both sequences monotonic, so sibling
        // chords never cross within a family.
        std::vector<char> used(nd, 0);
        for (std::size_t i = 0; i < k; ++i) {
            std::size_t j =
                k == 1 ? (nd - 1) / 2
                       : static_cast<std::size_t>(std::llround(
                             static_cast<double>(i) * (nd - 1) / (k - 1)));
            used[j] = 1;
            double a = topDense ? sparse[i] : dense[j];
            double b = topDense ? dense[j] : sparse[i];
            auto pts = buildCurve(fam, a, b, conform);
            if (!insideOk(pts)) {
                pts = buildCurve(fam, a, b, 0.0);   // fall back to pure chord
                if (!insideOk(pts)) continue;       // concave: skip the line
            }
            through[fam].push_back(std::move(pts));
        }
        // Side matching (round 11): within snapRatio the counts SNAP to equal
        // (unmatched dense stations dropped); above it the dense side's
        // extras T-terminate at the cross family below.
        if (ratio > p.snapRatio)
            for (std::size_t j = 0; j < nd; ++j)
                if (!used[j]) extras[fam].push_back({dense[j], topDense});
    }

    for (int fam = 0; fam < 2; ++fam)
        for (const auto& line : through[fam]) emitPolyline(line, p, out);

    // ROUND-11 T-TERMINATION: an extra runs inward from its own station and
    // stops at its first crossing with the cross family (near-perpendicular
    // incidence by construction — the cross family runs the other way). An
    // extra that meets nothing is DROPPED: no dead ends inside a closed face.
    for (int fam = 0; fam < 2; ++fam) {
        const auto& crossLines = through[1 - fam];
        if (crossLines.empty()) continue;
        for (const auto& ex : extras[fam]) {
            auto pts = buildCurve(fam, ex.first, ex.first, conform);
            if (ex.second) std::reverse(pts.begin(), pts.end());
            std::vector<Vec2> walk;
            walk.push_back(pts[0]);
            bool terminated = false;
            for (std::size_t i = 0; i + 1 < pts.size() && !terminated; ++i) {
                double bestT = 1e30;
                Vec2 X;
                for (const auto& cl : crossLines)
                    for (std::size_t j = 0; j + 1 < cl.size(); ++j) {
                        double t, u;
                        if (segIntersect(pts[i], pts[i + 1], cl[j], cl[j + 1],
                                         t, u) &&
                            t > 1e-6 && t < bestT) {
                            bestT = t;
                            X = pts[i] + (pts[i + 1] - pts[i]) * t;
                        }
                    }
                if (bestT < 1e29) {
                    walk.push_back(X);
                    terminated = true;
                } else {
                    walk.push_back(pts[i + 1]);
                }
            }
            if (!terminated) continue;
            if (polylineLen(walk) < std::max(60.0, p.blockLen * 0.3)) continue;
            bool ok = true;
            for (std::size_t i = 1; i < walk.size() && ok; ++i)
                if (!pointInPoly(clip, (walk[i - 1] + walk[i]) * 0.5)) ok = false;
            if (ok) emitPolyline(walk, p, out);
        }
    }
}

// The chord a bisecting line lays across a face (engine-faithful; the same
// construction district.cpp/metro.cpp use).
bool cutSpanLocal(const Poly2& poly, const Vec2& pt, const Vec2& dir, Vec2& a,
                  Vec2& b) {
    Vec2 n(-dir.y, dir.x);
    double c = n.x * pt.x + n.y * pt.y;
    double tMin = 1e30, tMax = -1e30;
    int hits = 0;
    const int m = static_cast<int>(poly.size());
    for (int i = 0; i < m; ++i) {
        Vec2 p0 = poly[i], p1 = poly[(i + 1) % m];
        double d0 = n.x * p0.x + n.y * p0.y - c;
        double d1 = n.x * p1.x + n.y * p1.y - c;
        if ((d0 <= 0 && d1 > 0) || (d0 > 0 && d1 <= 0)) {
            double u = d0 / (d0 - d1);
            Vec2 x = p0 + (p1 - p0) * u;
            double t = (x.x - pt.x) * dir.x + (x.y - pt.y) * dir.y;
            tMin = std::min(tMin, t);
            tMax = std::max(tMax, t);
            ++hits;
        }
    }
    if (hits < 2) return false;
    a = pt + dir * tMin;
    b = pt + dir * tMax;
    return true;
}

// Shared rings+ribs body; `maxRings` = 1 is the RING COURT (interior rows
// omitted — the middle stays one big block).
std::vector<FabricSegment> ringsRibs(const Poly2& patch,
                                     const PatchFabricParams& p, int maxRings) {
    std::vector<FabricSegment> out;
    if (patch.size() < 3) return out;
    const double area = std::fabs(polyArea(patch));
    if (area < p.blockLen * p.blockDepth * 1.2) return out;   // sub-block: nothing

    auto emitArc = [&](const std::vector<Vec2>& pts) {
        for (std::size_t i = 0; i + 1 < pts.size(); ++i)
            out.push_back({pts[i], pts[i + 1], p.streetWidth, RoadClass::Local});
    };

    // Rings at k*blockDepth from the ORIGINAL boundary (never ring-of-ring:
    // miter error would compound).
    std::vector<std::vector<std::vector<Vec2>>> rings;   // per depth: arcs
    for (int k = 1; k <= maxRings; ++k) {
        auto arcs = insetRingArcs(patch, k * p.blockDepth);
        if (arcs.empty()) break;
        for (const auto& a : arcs) emitArc(a);
        rings.push_back(std::move(arcs));
    }

    // Ribs: stations along the OUTER curve of each band, perpendicular inward
    // to the exact hit on the next ring. Station phase anchored per arc.
    auto stationRibs = [&](const std::vector<Vec2>& outer,
                           const std::vector<std::vector<Vec2>>& inner) {
        if (outer.size() < 2) return;
        double total = 0;
        for (std::size_t i = 0; i + 1 < outer.size(); ++i)
            total += (outer[i + 1] - outer[i]).length();
        int count = static_cast<int>(total / p.blockLen);
        if (count < 1) return;
        const double step = total / count;
        const double clear = p.cornerClear > 0 ? p.cornerClear : p.blockLen * 0.45;
        double want = step * 0.5, walked = 0;
        for (std::size_t i = 0; i + 1 < outer.size() && want < total; ++i) {
            Vec2 a = outer[i], b2 = outer[i + 1];
            double seg = (b2 - a).length();
            while (want <= walked + seg && want < total) {
                double t = (want - walked) / std::max(1e-9, seg);
                Vec2 pos = a + (b2 - a) * t;
                Vec2 dir = (b2 - a) * (1.0 / std::max(1e-9, seg));
                // ends of open arcs are seam cusps: keep clear of them
                double fromEnds = std::min(want, total - want);
                if (fromEnds < clear) { want += step; continue; }
                Vec2 nrm = inwardNormal(dir, polyArea(patch) > 0);
                // march inward to the nearest hit on any inner arc
                Vec2 hit;
                double bestT = 1e30;
                for (const auto& arc : inner)
                    for (std::size_t j = 0; j + 1 < arc.size(); ++j) {
                        // ray pos + s*nrm vs segment arc[j..j+1]
                        Vec2 q = arc[j], r = arc[j + 1] - arc[j];
                        double den = nrm.x * (-r.y) - nrm.y * (-r.x);
                        if (std::fabs(den) < 1e-9) continue;
                        double s = ((q.x - pos.x) * (-r.y) - (q.y - pos.y) * (-r.x)) / den;
                        double u = (nrm.x * (q.y - pos.y) - nrm.y * (q.x - pos.x)) / den;
                        if (s > 1.0 && s < bestT && u >= 0.0 && u <= 1.0) {
                            bestT = s;
                            hit = pos + nrm * s;
                        }
                    }
                if (bestT < p.blockDepth * 1.8) {
                    // incidence guard: skip glancing hits (angle > ~55 deg off
                    // perpendicular reads as a sliver seed)
                    out.push_back({pos, hit, p.streetWidth, RoadClass::Local});
                }
                want += step;
            }
            walked += seg;
        }
    };

    // band 0: boundary -> ring 1
    if (!rings.empty()) {
        Poly2 rb = resampleBoundary(patch, std::max(8.0, p.blockDepth * 0.25));
        std::vector<Vec2> outer(rb.begin(), rb.end());
        outer.push_back(rb.front());
        stationRibs(outer, rings[0]);
        // deeper bands: ring k -> ring k+1
        for (std::size_t k = 0; k + 1 < rings.size(); ++k)
            for (const auto& arc : rings[k]) stationRibs(arc, rings[k + 1]);
    }
    return out;
}

}  // namespace

std::vector<std::vector<Vec2>> insetRingArcs(const Poly2& patch, double depth) {
    std::vector<std::vector<Vec2>> arcs;
    if (patch.size() < 3 || depth <= 0) return arcs;
    const bool ccw = polyArea(patch) > 0;
    Poly2 b = resampleBoundary(patch, std::max(8.0, depth * 0.25));
    const std::size_t n = b.size();
    const double eps = depth * 0.02 + 0.25;

    // Naive per-vertex offset along averaged inward normals (miter by
    // normalizing the angle bisector, limited to 2x to stop spikes)...
    std::vector<Vec2> off(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2& p0 = b[(i + n - 1) % n];
        const Vec2& p1 = b[i];
        const Vec2& p2 = b[(i + 1) % n];
        Vec2 d0 = p1 - p0, d1 = p2 - p1;
        double l0 = d0.length(), l1 = d1.length();
        if (l0 > 1e-9) d0 = d0 * (1.0 / l0);
        if (l1 > 1e-9) d1 = d1 * (1.0 / l1);
        Vec2 n0 = inwardNormal(d0, ccw), n1 = inwardNormal(d1, ccw);
        Vec2 m = n0 + n1;
        double ml = m.length();
        if (ml < 1e-6) m = n1; else m = m * (1.0 / ml);
        double denom = std::max(0.5, dot(m, n1));   // miter limit 2x
        off[i] = p1 + m * (depth / denom);
    }
    // ...then the MEDIAL-VALIDITY PRUNE: a sample survives only if it is
    // inside the patch and no closer than depth-eps to ANY boundary stretch.
    // Everything on a self-intersection loop (lobe collapse, swallowtail at a
    // tight corner) fails the distance test — concavity handled without any
    // boolean geometry. Split survivors into maximal contiguous arcs.
    std::vector<char> ok(n, 0);
    for (std::size_t i = 0; i < n; ++i)
        ok[i] = pointInPoly(patch, off[i]) &&
                distToBoundary(patch, off[i]) >= depth - eps;

    std::size_t start = 0;
    while (start < n && ok[start]) ++start;
    if (start == n) {                      // fully valid: one closed ring
        std::vector<Vec2> ring(off.begin(), off.end());
        ring.push_back(off.front());       // close it
        arcs.push_back(std::move(ring));
        return arcs;
    }
    std::vector<Vec2> cur;
    for (std::size_t k = 1; k <= n; ++k) {
        std::size_t i = (start + k) % n;
        if (ok[i]) {
            cur.push_back(off[i]);
        } else if (!cur.empty()) {
            if (cur.size() >= 2) arcs.push_back(cur);
            cur.clear();
        }
    }
    if (cur.size() >= 2) arcs.push_back(std::move(cur));
    return arcs;
}

std::vector<FabricSegment> fabricRingsRibs(const Poly2& patch,
                                           const PatchFabricParams& p) {
    return ringsRibs(patch, p, 8);
}

std::vector<FabricSegment> fabricCourt(const Poly2& patch,
                                       const PatchFabricParams& p) {
    return ringsRibs(patch, p, 1);
}

std::vector<FabricSegment> fabricChords(const FabricPatch& patch,
                                        const PatchFabricParams& p) {
    std::vector<FabricSegment> out;
    const Poly2& b = patch.boundary;
    const int n = static_cast<int>(b.size());
    if (n < 3) return out;
    if (std::fabs(polyArea(b)) < p.blockLen * p.blockDepth * 1.2)
        return out;                                   // sub-block: nothing

    // Degenerate boundary (a filament walk visits a vertex twice — a slit
    // face): refuse rather than guess; the caller keeps its legacy fill.
    {
        std::vector<long long> keys;
        keys.reserve(b.size());
        for (const Vec2& v : b)
            keys.push_back((static_cast<long long>(std::llround(v.x * 4.0)) << 32) ^
                           (static_cast<long long>(std::llround(v.y * 4.0)) &
                            0xffffffffLL));
        std::sort(keys.begin(), keys.end());
        if (std::adjacent_find(keys.begin(), keys.end()) != keys.end())
            return out;
    }

    // Validated ascending corner list.
    std::vector<int> corners;
    for (int c : patch.corners)
        if (c >= 0 && c < n && (corners.empty() || c > corners.back()))
            corners.push_back(c);
    const int N = static_cast<int>(corners.size());
    if (N < 2) return out;

    // Sides: boundary chains between consecutive corners (wrapping).
    std::vector<SideCurve> sides(N);
    for (int i = 0; i < N; ++i) {
        int a = corners[i], c = corners[(i + 1) % N];
        SideCurve s;
        for (int k = a;; k = (k + 1) % n) {
            s.pts.push_back(b[k]);
            if (k == c) break;
        }
        if (s.pts.size() < 2) return out;
        s.finish();
        if (s.len < 1e-6) return out;
        sides[i] = std::move(s);
    }

    if (N == 4) {
        chordsQuad(sides.data(), b, p, out);
        return out;
    }

    // N != 4 (round 11): MIDPOINT DECOMPOSITION — side midpoints to the
    // centroid make N four-sided sub-patches (classic CAD/hex-mesh layout);
    // the seams are streets. A patch whose centroid falls outside (banana)
    // or whose seams would leave it emits nothing — never a dangle.
    Vec2 C = centroid(b);
    if (!pointInPoly(b, C)) return out;
    std::vector<SideCurve> half0(N), half1(N);
    std::vector<Vec2> mids(N);
    for (int i = 0; i < N; ++i) {
        splitSide(sides[i], 0.5, half0[i], half1[i]);
        mids[i] = half0[i].pts.back();
    }
    for (int i = 0; i < N; ++i)
        for (double w : {0.15, 0.35, 0.5, 0.65, 0.85})
            if (!pointInPoly(b, mids[i] + (C - mids[i]) * w)) return out;

    for (int i = 0; i < N; ++i)
        out.push_back({mids[i], C, p.streetWidth, RoadClass::Local});

    // Sub-patch around corner i: [m_{i-1} .. c_i .. m_i .. C], all quads.
    for (int i = 0; i < N; ++i) {
        int pi = (i + N - 1) % N;
        SideCurve q[4];
        q[0] = half1[pi];                       // m_{i-1} -> c_i
        q[1] = half0[i];                        // c_i -> m_i
        q[2].pts = {mids[i], C};                // seam in
        q[2].finish();
        q[3].pts = {C, mids[pi]};               // seam out
        q[3].finish();
        Poly2 clip;
        clip.insert(clip.end(), q[0].pts.begin(), q[0].pts.end());
        clip.insert(clip.end(), q[1].pts.begin() + 1, q[1].pts.end());
        clip.push_back(C);
        chordsQuad(q, clip, p, out);
    }
    return out;
}

std::vector<FabricSegment> fabricBisect(const Poly2& patch,
                                        const PatchFabricParams& p) {
    std::vector<FabricSegment> out;
    if (patch.size() < 3) return out;
    const double blockArea = p.blockLen * p.blockDepth;
    const double area0 = std::fabs(polyArea(patch));
    if (area0 < blockArea * 1.2) return out;   // sub-block: nothing

    // Patch-geometry seed: order-free, mixed with the global seed.
    uint32_t h0 = p.seed;
    h0 = hashVec(h0, centroid(patch));
    h0 = hashCombine(h0, static_cast<uint32_t>(std::llround(area0 * 0.01)));

    const double hard = std::max(1.0, p.hardCollapse);
    const double soft = std::max(hard, p.blockLen);
    std::vector<Vec2> junctions;   // every emitted cut endpoint

    struct Item {
        Poly2 poly;
        uint32_t seed;
        int depth;
    };
    std::vector<Item> stack;
    stack.push_back({patch, h0, 0});
    while (!stack.empty()) {
        Item it = std::move(stack.back());
        stack.pop_back();
        const Poly2& f = it.poly;
        if (f.size() < 3 || it.depth > 16) continue;
        if (std::fabs(polyArea(f)) < blockArea * 2.2) continue;   // a block now
        OBB2 o = orientedBoundingBox(f);
        int la = o.longAxis();
        if (o.half[la] * 2.0 < p.blockLen * 1.5) continue;

        // Near-midpoint (0.4..0.6) position on the long axis, cut direction
        // tilted toward a seeded diagonal (round 12: variety over correctness).
        double t = 0.4 + 0.2 * hash01(hashCombine(it.seed, 11u));
        Vec2 sp = o.center + o.axis[la] * (o.half[la] * (2.0 * t - 1.0));
        double ang = (hash01(hashCombine(it.seed, 23u)) - 0.5) * 2.0 * 0.32;
        Vec2 base = o.axis[1 - la];
        Vec2 dir(base.x * std::cos(ang) - base.y * std::sin(ang),
                 base.x * std::sin(ang) + base.y * std::cos(ang));
        Vec2 a, b;
        if (!cutSpanLocal(f, sp, dir, a, b)) continue;

        // ROUND-13 NODE HYGIENE at emission: fuse endpoints onto existing
        // junctions — ALWAYS under `hard`, with seeded per-pair probability
        // in the soft band. A mandatory fuse whose slid segment would leave
        // the patch drops the whole cut (never a sliver, never a <hard pair).
        Vec2 ends[2] = {a, b};
        bool dropCut = false;
        for (int e = 0; e < 2 && !dropCut; ++e) {
            int bj = -1;
            double bd = 1e30;
            for (std::size_t j = 0; j < junctions.size(); ++j) {
                double d = (junctions[j] - ends[e]).length();
                if (d < bd) { bd = d; bj = static_cast<int>(j); }
            }
            if (bj < 0 || bd < 1e-9) continue;
            bool mustFuse = bd < hard;
            bool wantFuse = false;
            if (!mustFuse && bd < soft) {
                uint32_t ph = p.seed;
                ph = hashVec(ph, ends[e]);
                ph = hashVec(ph, junctions[bj]);
                wantFuse = hash01(ph) < std::clamp(p.softCollapseP, 0.0, 1.0);
            }
            if (!mustFuse && !wantFuse) continue;
            Vec2 other = ends[1 - e];
            bool ok = true;
            for (int k = 1; k < 8 && ok; ++k)
                if (!pointInPoly(patch,
                                 other + (junctions[bj] - other) * (k / 8.0)))
                    ok = false;
            if (ok) ends[e] = junctions[bj];
            else if (mustFuse) dropCut = true;
        }
        if (dropCut) continue;
        Vec2 d2 = ends[1] - ends[0];
        double dl = d2.length();
        if (dl < p.blockLen * 0.6) continue;   // degenerate after a fuse
        out.push_back({ends[0], ends[1], p.streetWidth, RoadClass::Local});
        junctions.push_back(ends[0]);
        junctions.push_back(ends[1]);

        // Recurse both halves of the SNAPPED line (the emitted street and
        // the face split agree exactly, so children terminate on it).
        d2 = d2 * (1.0 / dl);
        Poly2 left, right;
        splitByLine(f, ends[0], d2, left, right);
        stack.push_back({std::move(right), hashCombine(it.seed, 41u), it.depth + 1});
        stack.push_back({std::move(left), hashCombine(it.seed, 43u), it.depth + 1});
    }
    return out;
}

}  // namespace engine
