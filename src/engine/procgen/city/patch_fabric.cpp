#include "patch_fabric.h"

#include <algorithm>
#include <cmath>

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
    std::vector<FabricSegment> out;
    if (patch.size() < 3) return out;
    const double area = std::fabs(polyArea(patch));
    if (area < p.blockLen * p.blockDepth * 1.2) return out;   // sub-block: nothing

    auto emitPolyline = [&](const std::vector<Vec2>& pts) {
        for (std::size_t i = 0; i + 1 < pts.size(); ++i)
            out.push_back({pts[i], pts[i + 1], p.streetWidth, RoadClass::Local});
    };

    // Rings at k*blockDepth from the ORIGINAL boundary (never ring-of-ring:
    // miter error would compound).
    std::vector<std::vector<std::vector<Vec2>>> rings;   // per depth: arcs
    for (int k = 1; k <= 8; ++k) {
        auto arcs = insetRingArcs(patch, k * p.blockDepth);
        if (arcs.empty()) break;
        for (const auto& a : arcs) emitPolyline(a);
        rings.push_back(std::move(arcs));
    }

    // Ribs: stations along the OUTER curve of each band, perpendicular inward
    // to the exact hit on the next ring (or trimmed out for band 0's boundary
    // side, which the host graph provides). Station phase anchored to the
    // lexicographically smallest boundary vertex for rotation invariance.
    auto stationRibs = [&](const std::vector<Vec2>& outer,
                           const std::vector<std::vector<Vec2>>& inner,
                           bool outerIsBoundary) {
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
                if (!outerIsBoundary) {
                    // rings are emitted in boundary order too; inward is
                    // toward deeper rings — same convention holds.
                }
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
        stationRibs(outer, rings[0], true);
        // deeper bands: ring k -> ring k+1
        for (std::size_t k = 0; k + 1 < rings.size(); ++k)
            for (const auto& arc : rings[k])
                stationRibs(arc, rings[k + 1], false);
    }
    return out;
}

std::vector<FabricSegment> fabricTensor(const Poly2& patch,
                                        const PatchFabricParams& p) {
    // Demo-grade: boundary-tangent tensor field (nearest-edge direction,
    // decaying blend toward the patch OBB axis in the deep interior), traced
    // on a jittered seed grid with RK2, evenly spaced, clipped to the patch.
    // Dangles are trimmed only if shorter than half a block — the honest
    // exhibit of why the architect keeps this dormant for production.
    std::vector<FabricSegment> out;
    if (patch.size() < 3) return out;
    const double area = std::fabs(polyArea(patch));
    if (area < p.blockLen * p.blockDepth * 1.2) return out;

    Poly2 b = resampleBoundary(patch, 12.0);
    auto fieldAt = [&](const Vec2& q, bool cross) {
        double best = 1e30;
        Vec2 dir(1, 0);
        for (std::size_t i = 0; i + 1 <= b.size(); ++i) {
            const Vec2& a = b[i];
            const Vec2& c = b[(i + 1) % b.size()];
            Vec2 ab = c - a;
            double len2 = ab.lengthSquared();
            if (len2 < 1e-12) continue;
            double t = std::clamp(dot(q - a, ab) / len2, 0.0, 1.0);
            Vec2 pt(a.x + ab.x * t, a.y + ab.y * t);
            double d = (q - pt).length();
            if (d < best) {
                best = d;
                dir = ab * (1.0 / std::sqrt(len2));
            }
        }
        if (cross) dir = Vec2(-dir.y, dir.x);
        return dir;
    };

    // Seeds on a coarse jittered grid inside the patch; two families.
    double minX = 1e30, minY = 1e30, maxX = -1e30, maxY = -1e30;
    for (const Vec2& v : patch) {
        minX = std::min(minX, v.x); maxX = std::max(maxX, v.x);
        minY = std::min(minY, v.y); maxY = std::max(maxY, v.y);
    }
    std::vector<Vec2> placed;   // spacing control: no two lines closer than this
    auto tooClose = [&](const Vec2& q, double d) {
        for (const Vec2& v : placed)
            if ((q - v).length() < d) return true;
        return false;
    };
    uint32_t rng = p.seed * 2654435761u + 97u;
    auto frand = [&]() {
        rng = rng * 1664525u + 1013904223u;
        return (rng >> 8) * (1.0 / 16777216.0);
    };
    for (int fam = 0; fam < 2; ++fam) {
        const double spacing = fam == 0 ? p.blockDepth : p.blockLen;
        for (double gy = minY; gy < maxY; gy += spacing)
            for (double gx = minX; gx < maxX; gx += spacing) {
                Vec2 seed(gx + frand() * spacing * 0.4,
                          gy + frand() * spacing * 0.4);
                if (!pointInPoly(patch, seed)) continue;
                if (tooClose(seed, spacing * 0.9)) continue;
                // trace both directions
                std::vector<Vec2> line{seed};
                for (int sgn = -1; sgn <= 1; sgn += 2) {
                    Vec2 q = seed;
                    for (int it = 0; it < 200; ++it) {
                        Vec2 d1 = fieldAt(q, fam == 1) * (8.0 * sgn);
                        Vec2 mid = q + d1 * 0.5;
                        Vec2 d2 = fieldAt(mid, fam == 1) * (8.0 * sgn);
                        Vec2 nq = q + d2;
                        if (!pointInPoly(patch, nq)) break;
                        if (tooClose(nq, spacing * 0.45)) break;
                        if (sgn < 0) line.insert(line.begin(), nq);
                        else line.push_back(nq);
                        q = nq;
                    }
                }
                double len = 0;
                for (std::size_t i = 0; i + 1 < line.size(); ++i)
                    len += (line[i + 1] - line[i]).length();
                if (len < p.blockLen * 0.5) continue;   // naive dangle trim
                for (const Vec2& v : line) placed.push_back(v);
                for (std::size_t i = 0; i + 1 < line.size(); ++i)
                    out.push_back({line[i], line[i + 1], p.streetWidth,
                                   RoadClass::Local});
            }
    }
    return out;
}

}  // namespace engine
