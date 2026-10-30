#include "road_mesh.h"
#include "../../../curve.h"
#include "road_rules.h"

#include "../../../log.h"          // LOG_WARN (grade-sep corkscrew guardrail)
#include "../../mesh_builder.h"
#include "road_offset.h"          // ribbonOutline, polygonUnion (the unified join engine)
#include "street_kit.h"           // roundPolygonCorners (the unified corner-fillet pass)
#include "triangulate.h"          // triangulateWithHoles (robust hole-aware ear clip)
#include "polygon.h"              // pointInPolygon, centroid
#include <algorithm>
#include <functional>
#include <map>
#include <array>
#include <climits>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace engine {

namespace {
// The tightest bend along a polyline, as a radius (circumradius of each consecutive
// triple; +inf for a straight run). Smaller = tighter.
double minCircumRadius(const std::vector<Vec2>& poly) {
    double worst = 1e30;
    for (std::size_t i = 1; i + 1 < poly.size(); ++i) {
        const Vec2 &a = poly[i - 1], &b = poly[i], &c = poly[i + 1];
        double ab = (b - a).length(), bc = (c - b).length(), ca = (a - c).length();
        double area2 = std::fabs(cross(b - a, c - a));        // 2 * triangle area
        if (area2 < 1e-9) continue;                           // collinear -> straight
        worst = std::min(worst, (ab * bc * ca) / (2.0 * area2));
    }
    return worst;
}
}  // namespace

std::vector<Vec2> fairHermite(const Vec2& p0, const Vec2& m0, const Vec2& p1, const Vec2& m1,
                              int segs, double minRadius) {
    segs = std::max(1, segs);
    // Blend the spline toward its straight chord by `alpha` (0 = the spline, 1 = the
    // chord). Blending toward a line scales the whole curve's amplitude — and therefore
    // every local radius — uniformly, endpoints included, so it can't trade apex
    // curvature for an endpoint hook the way shrinking the tangents alone would.
    auto sample = [&](double alpha) {
        std::vector<Vec2> poly;
        poly.reserve(segs + 1);
        for (int s = 0; s <= segs; ++s) {
            double t = static_cast<double>(s) / segs;
            Vec2 curve = hermite(p0, m0, p1, m1, t);
            Vec2 chord = p0 + (p1 - p0) * t;
            poly.push_back(curve * (1.0 - alpha) + chord * alpha);
        }
        return poly;
    };
    std::vector<Vec2> poly = sample(0.0);
    if (minRadius <= 0.0) return poly;
    // Flatten toward the chord until nothing bends tighter than minRadius. The chord has
    // infinite radius, so this always converges; an already-gentle curve keeps alpha 0.
    double alpha = 0.0;
    for (int iter = 0; iter < 12 && minCircumRadius(poly) < minRadius; ++iter) {
        alpha = std::min(1.0, alpha + 0.12);
        poly = sample(alpha);
        if (alpha >= 1.0) break;
    }
    return poly;
}

std::vector<std::array<int, 3>> triangulatePolygon(const std::vector<Vec2>& poly) {
    // the engine's one triangulator (triangulate.h); this was a second ear-clipper of its own
    return triangulateIndices(poly, {});
}





RenderMesh strokeRibbon(const std::vector<Vec2>& pts, const std::vector<double>& halfW,
                        double y, const Vec3& color, bool closed) {
    RenderMesh mesh;
    const int n = static_cast<int>(pts.size());
    if (n < 2 || halfW.empty()) return mesh;

    auto W   = [&](int i) { return halfW[std::min<int>(static_cast<int>(halfW.size()) - 1,
                                                       std::max(0, i))]; };
    auto P3  = [&](const Vec2& v) { return Vec3(v.x, y, v.y); };
    auto tri = [&](const Vec2& a, const Vec2& b, const Vec2& c) {
        MeshBuilder::emitTri(mesh, P3(a), P3(b), P3(c), Vec3(0, 1, 0), color);
    };

    // Each segment is a trapezoid (start half-width -> end half-width). On the inside
    // of a bend these overlap, but they're coplanar fills facing the same way — not a
    // fold — so the union is exactly the stroked region.
    const int segCount = closed ? n : n - 1;
    for (int i = 0; i < segCount; ++i) {
        Vec2 A = pts[i], B = pts[(i + 1) % n];
        Vec2 ab = B - A; double L = ab.length();
        if (L < 1e-9) continue;
        Vec2 nrm = perp(ab / L);
        double wa = W(i), wb = W((i + 1) % n);
        Vec2 AL = A + nrm*wa, AR = A - nrm*wa, BL = B + nrm*wb, BR = B - nrm*wb;
        tri(AL, AR, BR); tri(AL, BR, BL);
    }

    // Round join at each vertex: fan the OUTER wedge of the bend between the two
    // segments' edges (sweep == the signed turn angle, so a 180-degree vertex gets a
    // semicircular cap). The inner side's trapezoids overlap to meet. To avoid a
    // sliver where the two trapezoids leave a hairline gap (and the tiny inner cusp at
    // a convex vertex), the inner pair of corners is bridged with a single bevel
    // triangle — cheap, and harmless where it overlaps.
    const int vbeg = closed ? 0 : 1, vend = closed ? n : n - 1;
    for (int i = vbeg; i < vend; ++i) {
        Vec2 P = pts[i];
        Vec2 a = P - pts[(i - 1 + n) % n], b = pts[(i + 1) % n] - P;
        double la = a.length(), lb = b.length();
        if (la < 1e-9 || lb < 1e-9) continue;
        a = a / la; b = b / lb;
        double theta = std::atan2(cross(a, b), dot(a, b));   // signed turn (-pi..pi)
        if (std::fabs(theta) < 1e-4) continue;               // straight: segments meet flush
        double w = W(i);
        double outer = theta > 0 ? -1.0 : 1.0;               // outer side sign
        Vec2 c0dir = perp(a) * outer;                        // dir to the outer corner
        double a0 = std::atan2(c0dir.y, c0dir.x);
        int segs = std::max(1, static_cast<int>(std::ceil(std::fabs(theta) / 0.30)));
        Vec2 prev = P + c0dir * w;
        for (int s = 1; s <= segs; ++s) {
            double ang = a0 + theta * (static_cast<double>(s) / segs);
            tri(P, prev, P + Vec2(std::cos(ang), std::sin(ang)) * w);
            prev = P + Vec2(std::cos(ang), std::sin(ang)) * w;
        }
        // Inner bevel: bridge the two inner corners so no hairline gap/cusp remains.
        tri(P, P - perp(a) * outer * w, P - perp(b) * outer * w);
    }
    return mesh;
}


// ---- RoadDeckField ------------------------------------------------------------
void RoadDeckField::buildIndex() {
    cells_.clear();
    padCells_.clear();
    auto key = [](int cx, int cz) {
        // shifted as unsigned: a negative cell shifted signed is UB (UBSan)
        return static_cast<long long>((static_cast<unsigned long long>(static_cast<unsigned>(cx)) << 32) ^
                                      static_cast<unsigned>(cz));
    };
    for (int ti = 0; ti < static_cast<int>(pads.size()); ++ti) {
        const Tri& t = pads[ti];
        const double x0 = std::min({t.a.x, t.b.x, t.c.x}), x1 = std::max({t.a.x, t.b.x, t.c.x});
        const double z0 = std::min({t.a.z, t.b.z, t.c.z}), z1 = std::max({t.a.z, t.b.z, t.c.z});
        for (int cz = static_cast<int>(std::floor(z0 / cell_)); cz <= static_cast<int>(std::floor(z1 / cell_)); ++cz)
            for (int cx = static_cast<int>(std::floor(x0 / cell_)); cx <= static_cast<int>(std::floor(x1 / cell_)); ++cx)
                padCells_[key(cx, cz)].push_back(ti);
    }
    for (int si = 0; si < static_cast<int>(spines.size()); ++si) {
        const UnionSpine& sp = spines[si];
        if (sp.yAbs.size() != sp.points.size()) continue;   // no deck: skip
        for (int i = 0; i + 1 < static_cast<int>(sp.points.size()); ++i) {
            const Vec2& a = sp.points[i];
            const Vec2& b = sp.points[i + 1];
            const int x0 = static_cast<int>(std::floor(std::min(a.x, b.x) / cell_)) - 1;
            const int x1 = static_cast<int>(std::floor(std::max(a.x, b.x) / cell_)) + 1;
            const int z0 = static_cast<int>(std::floor(std::min(a.y, b.y) / cell_)) - 1;
            const int z1 = static_cast<int>(std::floor(std::max(a.y, b.y) / cell_)) + 1;
            for (int cz = z0; cz <= z1; ++cz)
                for (int cx = x0; cx <= x1; ++cx)
                    cells_[key(cx, cz)].push_back({si, i});
        }
    }
}

double RoadDeckField::depthInside(double x, double z, int* spineOut) const {
    if (spineOut) *spineOut = -1;
    const long long k = (static_cast<long long>(static_cast<int>(std::floor(x / cell_))) << 32) ^
                        (static_cast<long long>(static_cast<int>(std::floor(z / cell_))) & 0xffffffffLL);
    double depth = 0.0;
    if (auto pit = padCells_.find(k); pit != padCells_.end()) {
        for (int ti : pit->second) {
            const Tri& t = pads[ti];
            const double d1 = (x - t.b.x) * (t.a.z - t.b.z) - (t.a.x - t.b.x) * (z - t.b.z);
            const double d2 = (x - t.c.x) * (t.b.z - t.c.z) - (t.b.x - t.c.x) * (z - t.c.z);
            const double d3 = (x - t.a.x) * (t.c.z - t.a.z) - (t.c.x - t.a.x) * (z - t.a.z);
            const bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            const bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
            if (neg && pos) continue;
            auto edgeDist = [&](const Vec3& p, const Vec3& q) {
                const double ex = q.x - p.x, ez = q.z - p.z;
                const double l2 = ex * ex + ez * ez;
                double u = l2 < 1e-12 ? 0.0 : ((x - p.x) * ex + (z - p.z) * ez) / l2;
                u = u < 0 ? 0 : (u > 1 ? 1 : u);
                const double dx = p.x + ex * u - x, dz = p.z + ez * u - z;
                return std::sqrt(dx * dx + dz * dz);
            };
            depth = std::max(depth, std::min({edgeDist(t.a, t.b), edgeDist(t.b, t.c), edgeDist(t.c, t.a)}));
        }
    }
    auto it = cells_.find(k);
    if (it == cells_.end()) return depth;
    const Vec2 q(x, z);
    for (const Seg& sg : it->second) {
        const UnionSpine& sp = spines[sg.spine];
        if (sp.use != UnionSpine::Use::Road) continue;   // the asphalt: not a viaduct's track or a way in
        const Vec2& a = sp.points[sg.i];
        const Vec2& b = sp.points[sg.i + 1];
        const Vec2 ab = b - a;
        const double L2 = ab.lengthSquared();
        if (L2 < 1e-12) continue;
        const double t = dot(q - a, ab) / L2;
        if (t < 0.0 || t > 1.0) continue;   // past an end: a cap, not the carriageway
        const double d = (a + ab * t - q).length();
        const double hw = sp.hw.size() == sp.points.size()
                              ? sp.hw[sg.i] + (sp.hw[sg.i + 1] - sp.hw[sg.i]) * t
                              : sp.halfWidth;
        if (hw - d > depth) {
            depth = hw - d;
            if (spineOut) *spineOut = sg.spine;
        }
    }
    return depth;
}

bool RoadDeckField::coversAny(double x, double z, double margin) const {
    bool hit = false;
    surfacesAt(x, z, margin, [&](double, bool, int, double, double, RoadClass) { hit = true; }, true);
    return hit;
}

void RoadDeckField::surfacesAt(double x, double z, double margin,
                               const std::function<void(double y, bool pad, int layer, double d, double hw, RoadClass klass)>& fn,
                               bool anyUse) const {
    const long long k = (static_cast<long long>(static_cast<int>(std::floor(x / cell_))) << 32) ^
                        (static_cast<long long>(static_cast<int>(std::floor(z / cell_))) & 0xffffffffLL);
    if (auto pit = padCells_.find(k); pit != padCells_.end()) {
        for (int ti : pit->second) {
            const Tri& t = pads[ti];
            const double d1 = (x - t.b.x) * (t.a.z - t.b.z) - (t.a.x - t.b.x) * (z - t.b.z);
            const double d2 = (x - t.c.x) * (t.b.z - t.c.z) - (t.b.x - t.c.x) * (z - t.c.z);
            const double d3 = (x - t.a.x) * (t.c.z - t.a.z) - (t.c.x - t.a.x) * (z - t.a.z);
            const bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            const bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
            if (neg && pos) continue;
            const double den = (t.b.z - t.c.z) * (t.a.x - t.c.x) + (t.c.x - t.b.x) * (t.a.z - t.c.z);
            if (std::fabs(den) < 1e-12) continue;
            const double w0 = ((t.b.z - t.c.z) * (x - t.c.x) + (t.c.x - t.b.x) * (z - t.c.z)) / den;
            const double w1 = ((t.c.z - t.a.z) * (x - t.c.x) + (t.a.x - t.c.x) * (z - t.c.z)) / den;
            fn(w0 * t.a.y + w1 * t.b.y + (1.0 - w0 - w1) * t.c.y, true, 0, 0.0, 0.0, RoadClass::Local);
        }
    }
    auto it = cells_.find(k);
    if (it == cells_.end()) return;
    const Vec2 q(x, z);
    for (const Seg& sg : it->second) {
        const UnionSpine& sp = spines[sg.spine];
        if (sp.use != UnionSpine::Use::Road && !anyUse) continue;   // nothing on a street stands on a track or a way in
        const Vec2& a = sp.points[sg.i];
        const Vec2& b = sp.points[sg.i + 1];
        const Vec2 ab = b - a;
        const double L2 = ab.lengthSquared();
        double t = L2 < 1e-12 ? 0.0 : dot(q - a, ab) / L2;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        const double d = (a + ab * t - q).length();
        const double hw = sp.hw.size() == sp.points.size()
                              ? sp.hw[sg.i] + (sp.hw[sg.i + 1] - sp.hw[sg.i]) * t
                              : sp.halfWidth;
        if (d > hw + margin) continue;
        double lateral = 0.0;
        if (L2 > 1e-12) {
            const Vec2 fwd = ab * (1.0 / std::sqrt(L2));
            lateral = dot(q - a, Vec2(-fwd.y, fwd.x));
        }
        fn(sp.sectionY(sg.i, t, lateral), false, sp.layer, d, hw, sp.klass);
    }
}

double UnionSpine::sectionY(std::size_t i, double t, double lateral) const {
    const std::size_t j = std::min(i + 1, points.size() - 1);
    double y = yAbs[i] + (yAbs[j] - yAbs[i]) * t;
    // Superelevation: the lattice banks the deck by signedLateral *
    // crossSlope, lateral measured along `left` = fwd rotated +90 in XZ
    // (road_lattice.cpp, rings[i].left), + rising to the left.
    if (crossSlope.size() == points.size()) y += lateral * (crossSlope[i] + (crossSlope[j] - crossSlope[i]) * t);
    if (yLeft.size() == points.size() && yRight.size() == points.size()) {
        const double h = hw.size() == points.size() ? hw[i] + (hw[j] - hw[i]) * t : halfWidth;
        const std::vector<double>& edge = lateral >= 0.0 ? yLeft : yRight;
        const double ye = edge[i] + (edge[j] - edge[i]) * t;
        const double centre = yAbs[i] + (yAbs[j] - yAbs[i]) * t;
        y += (ye - centre) * (h > 1e-9 ? std::min(1.0, std::fabs(lateral) / h) : 0.0);
    }
    return y;
}

bool RoadDeckField::heightAt(double x, double z, double margin, double* outY) const {
    // Pads first: inside a pad the drawn triangle IS the surface (stacked pads: the top).
    // WHICH DECK, WHEN TWO ARE STACKED HERE. Everything that asks this question in 2-D is at
    // GRADE — a parked car, a painted bay, a crosswalk decal, a signal pole — while moving
    // traffic asks heightNear with its own level. So a lower LAYER wins outright over a nearer
    // one: the street under an overpass is the surface, not the deck eight metres above it.
    // Within a layer it is the nearest spine. Every lattice city is one layer, so this changes
    // nothing there; a lane-built city with a freeway over its streets was reading the freeway
    // (metro_lanes: cars ±1 m off, 161 samples past 25 cm).
    bool pad = false;
    double padY = -1e30;
    int bestLayer = INT_MAX;
    double bestD = 1e30, bestY = 0.0;
    surfacesAt(x, z, margin, [&](double y, bool isPad, int layer, double d, double, RoadClass) {
        if (isPad) { if (!pad || y > padY) padY = y; pad = true; return; }
        if (layer > bestLayer || (layer == bestLayer && d >= bestD)) return;
        bestLayer = layer; bestD = d; bestY = y;
    });
    if (pad) { *outY = padY; return true; }
    if (bestD > 1e29) return false;
    *outY = bestY;
    return true;
}

bool RoadDeckField::heightNear(double x, double z, double margin, double refY, double window, double* outY) const {
    return heightOn(x, z, margin, nullptr, refY, window, outY);
}

bool RoadDeckField::heightOn(double x, double z, double margin, const RoadClass* klass, double refY, double window,
                             double* outY) const {
    // THE ROAD, then its height. Of the spines within `window` of refY (a stacked level is never in it), one
    // of the caller's class when there is one -- the ramp, not the freeway it runs beside; the street, not
    // the overpass -- the one the point is MOST INSIDE (half-width - distance). Not the one nearest refY: a
    // lerp between nav ends can be further off than two roads are apart (#35: 1.46 m on island 8, and the
    // freeway won). A junction pad then answers if one lies within 1.5 m of that road (stacked pads, the
    // nearer); with no spine, a pad within the window.
    struct Pick { bool have = false; bool match = false; double depth = -1e30, y = 0; } sp;
    std::vector<double> pads;
    surfacesAt(x, z, margin, [&](double y, bool pad, int, double d, double hw, RoadClass k) {
        if (std::fabs(y - refY) > window) return;
        if (pad) { pads.push_back(y); return; }
        const bool m = klass && k == *klass;
        if (sp.have && (sp.match && !m)) return;
        if (sp.have && sp.match == m && hw - d <= sp.depth) return;
        sp.have = true; sp.match = m; sp.depth = hw - d; sp.y = y;
    });
    const double ref = sp.have ? sp.y : refY;
    const double padWin = sp.have ? 1.5 : window;
    double best = padWin;
    bool padHit = false;
    for (double y : pads)
        if (std::fabs(y - ref) <= best) { best = std::fabs(y - ref); *outY = y; padHit = true; }
    if (padHit) return true;
    if (sp.have) { *outY = sp.y; return true; }
    return false;
}

}  // namespace engine
