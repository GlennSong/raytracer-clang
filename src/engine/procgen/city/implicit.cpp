#include "implicit.h"
#include "section.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace engine {

Sdf sdfPlan(const Poly2& planIn) {
    Poly2 plan = planIn;
    if (plan.size() >= 3) ensureCCW(plan);
    return [plan](const Vec3& p) -> double {
        if (plan.size() < 3) return 1e9;
        const Vec2 q(p.x, p.z);
        double best = 1e300;
        for (std::size_t i = 0; i < plan.size(); ++i) {
            const Vec2 a = plan[i], b = plan[(i + 1) % plan.size()];
            const Vec2 ab = b - a;
            const double L2 = ab.lengthSquared();
            const double t = L2 > 1e-12 ? std::max(0.0, std::min(1.0, dot(q - a, ab) / L2)) : 0.0;
            best = std::min(best, (q - (a + ab * t)).lengthSquared());
        }
        const double d = std::sqrt(best);
        return pointInPolygon(plan, q) ? -d : d;
    };
}

Sdf sdfExtrude(const Poly2& plan, Real y0, Real y1) {
    const Sdf flat = sdfPlan(plan);
    return [flat, y0, y1](const Vec3& p) -> double {
        const double d = flat(p);
        const double dy = std::max(y0 - p.y, p.y - y1);   // the slab: negative between y0 and y1
        return std::max(d, dy);
    };
}

Sdf sdfWave(Sdf field, Real amplitude, Real wavelength, const Vec2& alongIn, Real riseLength) {
    const Vec2 along = normalize(alongIn);
    const Vec2 across(-along.y, along.x);
    const double kS = wavelength > 1e-6 ? 2.0 * 3.14159265358979323846 / wavelength : 0.0;
    const double kY = riseLength > 1e-6 ? 2.0 * 3.14159265358979323846 / riseLength : 0.0;
    return [field, amplitude, along, across, kS, kY](const Vec3& p) -> double {
        const double s = p.x * across.x + p.z * across.y;
        const double off = amplitude * std::sin(kS * s + kY * p.y);
        return field(Vec3(p.x - along.x * off, p.y, p.z - along.y * off));
    };
}

namespace {

Real dpDistance(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const Real L2 = ab.lengthSquared();
    const Real t = L2 > 1e-12 ? std::max(Real(0), std::min(Real(1), dot(p - a, ab) / L2)) : Real(0);
    return (p - (a + ab * t)).length();
}

void dpOpen(const Poly2& pts, std::size_t i0, std::size_t i1, Real tol, std::vector<bool>& keep) {
    if (i1 <= i0 + 1) return;
    Real worst = -1;
    std::size_t at = i0;
    for (std::size_t i = i0 + 1; i < i1; ++i) {
        const Real d = dpDistance(pts[i], pts[i0], pts[i1]);
        if (d > worst) { worst = d; at = i; }
    }
    if (worst <= tol) return;
    keep[at] = true;
    dpOpen(pts, i0, at, tol, keep);
    dpOpen(pts, at, i1, tol, keep);
}

}  // namespace

Poly2 simplifyLoop(const Poly2& loop, Real tol) {
    const std::size_t n = loop.size();
    if (n < 4 || tol <= 0) return loop;
    // split the loop at the point farthest from point 0, simplify the two open runs
    std::size_t far = 0;
    Real best = -1;
    for (std::size_t i = 1; i < n; ++i) {
        const Real d = (loop[i] - loop[0]).lengthSquared();
        if (d > best) { best = d; far = i; }
    }
    Poly2 pts = loop;
    pts.push_back(loop[0]);   // closed as an open run 0..n
    std::vector<bool> keep(n + 1, false);
    keep[0] = keep[far] = keep[n] = true;
    dpOpen(pts, 0, far, tol, keep);
    dpOpen(pts, far, n, tol, keep);
    Poly2 out;
    for (std::size_t i = 0; i < n; ++i)
        if (keep[i]) out.push_back(pts[i]);
    return out.size() >= 3 ? out : loop;
}

// Edges shorter than `minLen` (the chamfers marching squares cuts across a plan's corners) fold into the corner
// their neighbours meet at, when that corner is near.
static Poly2 mergeShortEdges(const Poly2& loop, Real minLen) {
    Poly2 out = loop;
    for (int pass = 0; pass < 4; ++pass) {
        const std::size_t n = out.size();
        if (n < 5) break;
        bool changed = false;
        for (std::size_t i = 0; i < out.size() && out.size() >= 5; ++i) {
            const std::size_t m = out.size();
            const Vec2 a = out[i], b = out[(i + 1) % m];
            if ((b - a).length() >= minLen) continue;
            const Vec2 pa = out[(i + m - 1) % m], nb = out[(i + 2) % m];
            const Vec2 d0 = a - pa, d1 = nb - b;
            const Real den = cross(d0, d1);
            if (std::fabs(den) < 1e-9) continue;
            const Real t = cross(a - pa, d1) / den;   // pa + d0 * t meets the line b..nb? (we want pa + d0 * t on line b + d1 * u)
            const Vec2 x = pa + d0 * (cross(b - pa, d1) / den);
            (void)t;
            if ((x - a).length() > minLen * 1.5 || (x - b).length() > minLen * 1.5) continue;
            out[i] = x;
            out.erase(out.begin() + static_cast<std::ptrdiff_t>((i + 1) % m));
            changed = true;
        }
        if (!changed) break;
    }
    return out;
}

Poly2 sliceSdf(const Sdf& field, Real y, const SdfBounds& bounds, Real cell, Real tol) {
    Poly2 none;
    if (cell <= 0.01) return none;
    const int nx = static_cast<int>(std::ceil((bounds.max.x - bounds.min.x) / cell)) + 1;
    const int nz = static_cast<int>(std::ceil((bounds.max.z - bounds.min.z) / cell)) + 1;
    if (nx < 2 || nz < 2 || nx * nz > 4000000) return none;
    std::vector<double> v(static_cast<std::size_t>(nx * nz));
    auto at = [&](int i, int j) -> double& { return v[static_cast<std::size_t>(j * nx + i)]; };
    // the grid off the field's own lines by an odd fraction of a cell, so a plan's edge never lies on a grid line
    // (crossings then sit on grid points and corners come out chamfered)
    auto P = [&](int i, int j) { return Vec2(bounds.min.x + (i + 0.37) * cell, bounds.min.z + (j + 0.37) * cell); };
    for (int j = 0; j < nz; ++j)
        for (int i = 0; i < nx; ++i) {
            const Vec2 q = P(i, j);
            at(i, j) = field(Vec3(q.x, y, q.y));
        }
    // crossings on grid edges: id = (j * nx + i) * 2 + (0 along x, 1 along z); the point interpolated
    auto edgeId = [&](int i, int j, int dir) { return static_cast<long long>(j * nx + i) * 2 + dir; };
    std::map<long long, Vec2> crossing;
    auto cross = [&](int i, int j, int dir) -> long long {
        const long long id = edgeId(i, j, dir);
        auto it = crossing.find(id);
        if (it == crossing.end()) {
            const double a = at(i, j), b = dir == 0 ? at(i + 1, j) : at(i, j + 1);
            const double t = std::fabs(b - a) > 1e-12 ? std::max(0.0, std::min(1.0, a / (a - b))) : 0.5;
            const Vec2 pa = P(i, j), pb = dir == 0 ? P(i + 1, j) : P(i, j + 1);
            crossing[id] = pa + (pb - pa) * t;
        }
        return id;
    };
    std::vector<std::pair<long long, long long>> segs;
    for (int j = 0; j + 1 < nz; ++j)
        for (int i = 0; i + 1 < nx; ++i) {
            const bool in00 = at(i, j) < 0, in10 = at(i + 1, j) < 0, in11 = at(i + 1, j + 1) < 0, in01 = at(i, j + 1) < 0;
            const int c = (in00 ? 1 : 0) | (in10 ? 2 : 0) | (in11 ? 4 : 0) | (in01 ? 8 : 0);
            if (c == 0 || c == 15) continue;
            // the four edges: 0 bottom (i,j)-(i+1,j), 1 right (i+1,j)-(i+1,j+1), 2 top (i,j+1)-(i+1,j+1), 3 left (i,j)-(i,j+1)
            auto e = [&](int k) {
                switch (k) {
                case 0: return cross(i, j, 0);
                case 1: return cross(i + 1, j, 1);
                case 2: return cross(i, j + 1, 0);
                default: return cross(i, j, 1);
                }
            };
            auto seg = [&](int ka, int kb) { segs.push_back({e(ka), e(kb)}); };
            switch (c) {
            case 1: case 14: seg(3, 0); break;
            case 2: case 13: seg(0, 1); break;
            case 3: case 12: seg(3, 1); break;
            case 4: case 11: seg(1, 2); break;
            case 6: case 9: seg(0, 2); break;
            case 7: case 8: seg(3, 2); break;
            case 5: case 10: {
                // the saddle: the centre's sign decides which pairs join
                const double centre = 0.25 * (at(i, j) + at(i + 1, j) + at(i + 1, j + 1) + at(i, j + 1));
                const bool centreIn = centre < 0;
                if ((c == 5) == centreIn) { seg(3, 2); seg(0, 1); }
                else { seg(3, 0); seg(1, 2); }
                break;
            }
            default: break;
            }
        }
    if (segs.empty()) return none;
    // link the segments into loops: every crossing is shared by two segments
    std::map<long long, std::vector<std::size_t>> atEdge;
    for (std::size_t k = 0; k < segs.size(); ++k) { atEdge[segs[k].first].push_back(k); atEdge[segs[k].second].push_back(k); }
    std::vector<bool> used(segs.size(), false);
    Poly2 bestLoop;
    Real bestArea = -1;
    for (std::size_t s0 = 0; s0 < segs.size(); ++s0) {
        if (used[s0]) continue;
        Poly2 loop;
        std::size_t k = s0;
        long long atId = segs[s0].first;
        for (int guard = 0; guard < static_cast<int>(segs.size()) + 2; ++guard) {
            used[k] = true;
            loop.push_back(crossing[atId]);
            const long long next = segs[k].first == atId ? segs[k].second : segs[k].first;
            const std::vector<std::size_t>& cand = atEdge[next];
            std::size_t nk = k;
            for (std::size_t c2 : cand) if (c2 != k && !used[c2]) { nk = c2; break; }
            atId = next;
            if (nk == k) break;   // the loop closed (back at s0's start) or an open run ended
            k = nk;
        }
        if (loop.size() < 3) continue;
        const Real a = std::fabs(signedArea(loop));
        if (a > bestArea) { bestArea = a; bestLoop = loop; }
    }
    if (bestLoop.size() < 3) return none;
    // crossings that coincide (the contour through a grid point) are one point
    Poly2 dedup;
    for (const Vec2& q : bestLoop)
        if (dedup.empty() || (q - dedup.back()).length() > cell * 0.3) dedup.push_back(q);
    while (dedup.size() > 3 && (dedup.front() - dedup.back()).length() <= cell * 0.3) dedup.pop_back();
    if (dedup.size() < 3) return none;
    ensureCCW(dedup);
    Poly2 out = mergeShortEdges(simplifyLoop(dedup, tol), cell * 1.6);
    ensureCCW(out);
    return out;
}

Slice ImplicitMass::slice(Real y) const {
    Slice s;
    if (base.size() < 3) return s;
    int k = 0;
    if (y >= groundH) k = 1 + static_cast<int>(std::floor((y - groundH) / std::max(Real(0.1), floorH) + 1e-6));
    k = std::max(0, std::min(floors, k));
    if (k <= baseStoreys || !field) {
        s.pieces.push_back(Section::fromLot(base));
        s.tier = 0;
        return s;
    }
    const Poly2 outline = sliceSdf(field, groundH + (k - 1) * floorH + 0.5 * floorH, box, cell, tol);
    s.pieces.push_back(Section::fromLot(outline.size() >= 3 ? outline : base));
    s.tier = 1;
    return s;
}

MassBounds ImplicitMass::bounds() const {
    MassBounds b;
    b.lo = Vec2(box.min.x, box.min.z);
    b.hi = Vec2(box.max.x, box.max.z);
    b.y0 = 0;
    b.y1 = height();
    return b;
}

RenderMesh ImplicitMass::shell(int lod, const Vec3& colour) const {
    if (lod >= 1 && field) {
        SdfBounds b = box;
        b.min.y = 0;
        b.max.y = height();
        return polygonizeSdf(field, b, 48);
    }
    RenderMesh m;
    std::vector<Poly2> outlines;
    std::vector<Real> heights;
    for (int k = 0; k <= floors; ++k) {
        outlines.push_back(slice(k == 0 ? -1.0 : groundH + (k - 1) * floorH).outline().tessellate());
        heights.push_back(k == 0 ? 0.0 : groundH + (k - 1) * floorH);
    }
    outlines.push_back(outlines.back());
    heights.push_back(height());
    loftSlices(m, outlines, heights, colour, true);
    return m;
}

ImplicitMass implicitOf(const Poly2& planIn, const BuildingParams& params) {
    ImplicitMass im;
    Poly2 plan = planIn;
    if (plan.size() < 3) return im;
    ensureCCW(plan);
    im.base = plan;
    im.baseStoreys = std::max(1, params.baseFloors);
    im.groundH = params.groundHeight;
    im.floorH = params.floorHeight;
    im.floors = params.floors;
    const Real amp = std::max(Real(0), params.wave);
    const Real len = std::max(Real(4), params.waveLen);
    Vec2 lo, hi;
    bounds(plan, lo, hi);
    im.box.min = Vec3(lo.x - amp - 1.0, 0, lo.y - amp - 1.0);
    im.box.max = Vec3(hi.x + amp + 1.0, im.height(), hi.y + amp + 1.0);
    const OBB2 ob = orientedBoundingBox(plan);
    // the plan, waved across its long axis by `wave`, the wave travelling up the building one cycle per `waveLen`
    Sdf f = sdfPlan(plan);
    if (amp > 0) f = sdfWave(f, amp, len, ob.axis[1 - ob.longAxis()], len);
    im.field = f;
    return im;
}

}  // namespace engine
