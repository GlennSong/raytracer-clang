#include "road_mesh.h"
#include "road_rules.h"

#include "../../../log.h"          // LOG_WARN (grade-sep corkscrew guardrail)
#include "../../mesh_builder.h"
#include "road_offset.h"          // ribbonOutline, polygonUnion (the unified join engine)
#include "street_kit.h"           // roundPolygonCorners (the unified corner-fillet pass)
#include "triangulate.h"          // triangulateWithHoles (robust hole-aware ear clip)
#include "polygon.h"              // pointInPolygon, centroid
#include <algorithm>
#include <map>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace engine {

std::vector<Vec2> curbReturnFillet(const Vec2& corner, const Vec2& dirA, const Vec2& dirB,
                                   double radius, double maxTangent, double maxStep) {
    if (radius <= 0.0 || maxTangent <= 1e-4) return {};
    Vec2 uA = normalize(dirA), uB = normalize(dirB);
    // Interior half-angle of the corner (between the two kerbs as they leave the corner).
    double c = std::max(-1.0, std::min(1.0, dot(uA, uB)));
    double phi = std::acos(c) * 0.5;
    // Near-straight (phi -> pi/2: barely a bend) or folded/parallel (phi -> 0): no real
    // arc — let the caller draw a straight chamfer.
    if (phi < 0.05 || phi > 1.52) return {};
    double tanPhi = std::tan(phi);
    double t = radius / tanPhi;            // tangent-point distance back from the corner
    if (t > maxTangent) {                  // shrink the radius so the fillet fits the trim
        t = maxTangent;
        radius = t * tanPhi;
    }
    Vec2 bis = normalize(uA + uB);         // bisector, pointing into the carriageway
    Vec2 center = corner + bis * (radius / std::sin(phi));
    Vec2 ta = corner + uA * t, tb = corner + uB * t;   // tangent points on each kerb
    double a0 = std::atan2(ta.y - center.y, ta.x - center.x);
    double a1 = std::atan2(tb.y - center.y, tb.x - center.x);
    double sweep = a1 - a0;
    while (sweep <= -3.14159265358979) sweep += 2 * 3.14159265358979;   // take the minor arc
    while (sweep >   3.14159265358979) sweep -= 2 * 3.14159265358979;
    int steps = std::max(1, static_cast<int>(std::ceil(std::fabs(sweep) / maxStep)));
    std::vector<Vec2> arc;
    arc.reserve(steps + 1);
    for (int s = 0; s <= steps; ++s) {
        double ang = a0 + sweep * (static_cast<double>(s) / steps);
        arc.push_back(center + Vec2(std::cos(ang), std::sin(ang)) * radius);
    }
    return arc;
}

namespace {
Vec2 hermitePoint(const Vec2& p0, const Vec2& m0, const Vec2& p1, const Vec2& m1, double t) {
    double t2 = t * t, t3 = t2 * t;
    double h00 = 2 * t3 - 3 * t2 + 1, h10 = t3 - 2 * t2 + t;
    double h01 = -2 * t3 + 3 * t2, h11 = t3 - t2;
    return p0 * h00 + m0 * h10 + p1 * h01 + m1 * h11;
}
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
            Vec2 curve = hermitePoint(p0, m0, p1, m1, t);
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

namespace {
bool pointInTriangle(const Vec2& p, const Vec2& a, const Vec2& b, const Vec2& c) {
    double d1 = cross(b - a, p - a), d2 = cross(c - b, p - b), d3 = cross(a - c, p - c);
    bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(neg && pos);                        // inside (or on edge) of CCW/CW triangle
}
}  // namespace

std::vector<double> roadProfile(const std::vector<double>& ground,
                                const std::vector<double>& s, double maxGrade) {
    int n = static_cast<int>(ground.size());
    std::vector<double> y = ground;
    if (n < 3 || maxGrade <= 0.0 || static_cast<int>(s.size()) != n) return y;
    // 1. Smooth out terrain bumps (two passes of a 3-point [1 2 1]/4 average; ends fixed).
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<double> sm = y;
        for (int i = 1; i + 1 < n; ++i) sm[i] = 0.25 * y[i - 1] + 0.5 * y[i] + 0.25 * y[i + 1];
        y = sm;
    }
    // 2. Slope-limit to <= maxGrade. A forward then backward clamp, iterated to a fixed
    //    point: each pass pulls any over-steep step back to the grade limit, and the two
    //    directions together settle into a profile within grade everywhere.
    for (int iter = 0; iter < 128; ++iter) {
        bool changed = false;
        for (int i = 1; i < n; ++i) {
            double lim = maxGrade * std::max(1e-6, s[i] - s[i - 1]);
            if (y[i] > y[i - 1] + lim) { y[i] = y[i - 1] + lim; changed = true; }
            else if (y[i] < y[i - 1] - lim) { y[i] = y[i - 1] - lim; changed = true; }
        }
        for (int i = n - 2; i >= 0; --i) {
            double lim = maxGrade * std::max(1e-6, s[i + 1] - s[i]);
            if (y[i] > y[i + 1] + lim) { y[i] = y[i + 1] + lim; changed = true; }
            else if (y[i] < y[i + 1] - lim) { y[i] = y[i + 1] - lim; changed = true; }
        }
        if (!changed) break;
    }
    return y;
}

std::vector<double> clearanceProfile(const std::vector<double>& s,
                                     const std::vector<double>& minHeight, double maxGrade) {
    int n = static_cast<int>(minHeight.size());
    std::vector<double> y = minHeight;
    if (n == 0 || maxGrade <= 0.0 || static_cast<int>(s.size()) != n) return y;
    // The minimal profile that dominates every constraint and obeys the slope limit is the
    // upper envelope of downward cones (slope maxGrade) cast from each required point.
    // Compute it in two linear sweeps: a forward pass propagates each height rightward,
    // decaying at the grade, and a backward pass leftward; the max of the two is the envelope.
    for (int i = 1; i < n; ++i)
        y[i] = std::max(y[i], y[i - 1] - maxGrade * std::max(1e-6, s[i] - s[i - 1]));
    for (int i = n - 2; i >= 0; --i)
        y[i] = std::max(y[i], y[i + 1] - maxGrade * std::max(1e-6, s[i + 1] - s[i]));
    return y;
}

std::vector<TerrainFlatten> roadConformRegions(const std::vector<Vec2>& centerline,
                                               const std::vector<double>& profileY,
                                               double halfWidth, double shoulder,
                                               double falloff) {
    std::vector<TerrainFlatten> regions;
    int n = static_cast<int>(centerline.size());
    if (n < 2 || static_cast<int>(profileY.size()) != n) return regions;
    double hw = halfWidth + shoulder;
    regions.reserve(n - 1);
    for (int i = 0; i + 1 < n; ++i) {
        Vec2 d = centerline[i + 1] - centerline[i];
        const double len = d.length();
        if (len < 1e-9) continue;
        d = d * (1.0 / len);
        // END CAPS: bare per-segment rectangles leave an uncovered WEDGE of
        // natural ground on the outside of every bend — the poke-report's
        // deck-edge tongues along curved roads (device: "still a lot of pokes").
        // Extending each rectangle by hw makes consecutive footprints overlap
        // across the bend; heights extrapolate along the segment's own grade,
        // and the overlap fold (lowest plane wins) keeps the seam consistent.
        const double cap = std::min(hw, len);
        const double grade = (profileY[i + 1] - profileY[i]) / len;
        const Vec2 ax = centerline[i] - d * cap, bx = centerline[i + 1] + d * cap;
        Vec3 a(ax.x, 0.0, ax.y);                                // Vec2.y is world Z
        Vec3 b(bx.x, 0.0, bx.y);
        regions.push_back(makeFlattenRamp(a, b, profileY[i] - grade * cap,
                                          profileY[i + 1] + grade * cap, hw, falloff));
    }
    return regions;
}

std::vector<std::array<int, 3>> triangulatePolygon(const std::vector<Vec2>& poly) {
    std::vector<std::array<int, 3>> tris;
    int n = static_cast<int>(poly.size());
    if (n < 3) return tris;
    // Drop consecutive duplicate vertices (incl. the wrap): coincident mouths at a sharp
    // junction give the ring degenerate zero-length edges that would stall ear finding.
    std::vector<int> v;
    for (int i = 0; i < n; ++i)
        if (v.empty() || (poly[i] - poly[v.back()]).length() > 1e-6) v.push_back(i);
    while (v.size() >= 2 && (poly[v.front()] - poly[v.back()]).length() < 1e-6) v.pop_back();
    if (v.size() < 3) return tris;
    double area2 = 0.0;                                  // signed area of the cleaned ring
    for (std::size_t i = 0; i < v.size(); ++i)
        area2 += cross(poly[v[i]], poly[v[(i + 1) % v.size()]]);
    if (area2 < 0.0) std::reverse(v.begin(), v.end());   // work CCW
    int guard = 3 * n;                           // bound the search so a bad ring can't spin
    while (static_cast<int>(v.size()) > 2 && guard-- > 0) {
        int m = static_cast<int>(v.size());
        bool clipped = false;
        for (int i = 0; i < m; ++i) {
            int ia = v[(i + m - 1) % m], ib = v[i], ic = v[(i + 1) % m];
            const Vec2 &a = poly[ia], &b = poly[ib], &c = poly[ic];
            if (cross(b - a, c - a) <= 1e-12) continue;   // reflex or collinear: not an ear
            bool ear = true;                              // no other vertex inside the ear
            for (int j = 0; j < m && ear; ++j) {
                int ij = v[j];
                if (ij == ia || ij == ib || ij == ic) continue;
                const Vec2& q = poly[ij];                 // a zero-width hole bridge revisits a
                if ((q - a).length() < 1e-6 ||            // corner under a different index; that
                    (q - b).length() < 1e-6 ||            // coincident vertex isn't really "inside"
                    (q - c).length() < 1e-6) continue;
                if (pointInTriangle(q, a, b, c)) ear = false;
            }
            if (!ear) continue;
            tris.push_back({ia, ib, ic});
            v.erase(v.begin() + i);
            clipped = true;
            break;
        }
        if (!clipped) {
            // No ear found (a near-degenerate ring from many bridged holes can stall the strict
            // test). Rather than break and leave the rest of the deck as a hole, force-clip the
            // sharpest convex corner to make progress — a tiny overlap beats a missing patch.
            int best = -1; double bestCross = 1e-9; int mm = static_cast<int>(v.size());
            for (int i = 0; i < mm; ++i) {
                const Vec2 &a = poly[v[(i + mm - 1) % mm]], &b = poly[v[i]], &c = poly[v[(i + 1) % mm]];
                double cr = cross(b - a, c - a);
                if (cr > bestCross) { bestCross = cr; best = i; }
            }
            if (best < 0) break;                 // truly degenerate: nothing convex left
            int i = best, mm2 = static_cast<int>(v.size());
            tris.push_back({v[(i + mm2 - 1) % mm2], v[i], v[(i + 1) % mm2]});
            v.erase(v.begin() + i);
        }
    }
    return tris;
}

namespace {
// Weld coincident vertices into one shared, indexed mesh: collapse vertices that agree in position
// (within posTol), normal, colour and UV into a single index, dropping triangles that go degenerate.
// The independently-emitted ribbons and junction pads meet at the same coordinates but as SEPARATE
// vertices (float drift -> hairline cracks / z-fight at the seam); welding fuses them into one
// continuous surface, and turns the non-indexed soup into a compact indexed mesh. Crease edges
// (different normals) and material seams (different colour) keep their own vertices, so nothing is
// rounded over or smeared. (road-network-v2-plan T3.1)
RenderMesh weldMesh(const RenderMesh& in, double posTol = 1e-3) {
    RenderMesh out;
    out.vertices.reserve(in.vertices.size());
    out.indices.reserve(in.indices.size());
    const double inv = 1.0 / posTol;
    auto cell = [inv](double x) { return static_cast<long long>(std::floor(x * inv)); };
    auto hash3 = [](long long x, long long y, long long z) {
        std::uint64_t h = 1469598103934665603ull;
        for (long long c : {x, y, z}) h = (h ^ static_cast<std::uint64_t>(c)) * 1099511628211ull;
        return h;
    };
    std::unordered_map<std::uint64_t, std::vector<uint32_t>> grid;   // spatial hash -> welded indices
    auto match = [posTol](const Vertex& a, const Vertex& b) {
        auto c = [posTol](double x, double y) { return std::fabs(x - y) <= posTol; };
        return c(a.position.x, b.position.x) && c(a.position.y, b.position.y) && c(a.position.z, b.position.z)
            && c(a.normal.x, b.normal.x) && c(a.normal.y, b.normal.y) && c(a.normal.z, b.normal.z)
            && c(a.color.x, b.color.x) && c(a.color.y, b.color.y) && c(a.color.z, b.color.z)
            && c(a.u, b.u) && c(a.v, b.v);
    };
    std::vector<uint32_t> remap(in.vertices.size());
    for (uint32_t i = 0; i < in.vertices.size(); ++i) {
        const Vertex& v = in.vertices[i];
        long long cx = cell(v.position.x), cy = cell(v.position.y), cz = cell(v.position.z);
        uint32_t found = UINT32_MAX;
        for (long long dx = -1; dx <= 1 && found == UINT32_MAX; ++dx)         // search the 27 neighbour
            for (long long dy = -1; dy <= 1 && found == UINT32_MAX; ++dy)     // cells so a vertex pair
                for (long long dz = -1; dz <= 1 && found == UINT32_MAX; ++dz) {  // straddling a cell edge
                    auto it = grid.find(hash3(cx + dx, cy + dy, cz + dz));   // still welds
                    if (it == grid.end()) continue;
                    for (uint32_t cand : it->second)
                        if (match(out.vertices[cand], v)) { found = cand; break; }
                }
        if (found == UINT32_MAX) {
            found = static_cast<uint32_t>(out.vertices.size());
            out.vertices.push_back(v);
            grid[hash3(cx, cy, cz)].push_back(found);
        }
        remap[i] = found;
    }
    for (std::size_t t = 0; t + 2 < in.indices.size(); t += 3) {
        uint32_t a = remap[in.indices[t]], b = remap[in.indices[t + 1]], c = remap[in.indices[t + 2]];
        if (a != b && b != c && a != c) {            // drop triangles welding collapsed to a sliver
            out.indices.push_back(a); out.indices.push_back(b); out.indices.push_back(c);
        }
    }
    return out;
}
}  // namespace




RenderMesh bridgePiers(const std::vector<Vec2>& center, const std::vector<double>& deckY,
                       const std::vector<int>& atSamples, double width, double depth,
                       double deckThk, const Vec3& color,
                       const std::function<double(double, double)>& ground) {
    RenderMesh mesh;
    const int n = static_cast<int>(center.size());
    if (n < 2 || static_cast<int>(deckY.size()) != n) return mesh;
    auto groundY = [&](const Vec2& v) { return ground ? ground(v.x, v.y) : 0.0; };
    for (int idx : atSamples) {
        if (idx < 0 || idx >= n) continue;
        Vec2 t = (idx + 1 < n) ? center[idx + 1] - center[idx] : center[idx] - center[idx - 1];
        double tl = t.length();
        if (tl < 1e-9) continue;
        t = t / tl;
        Vec2 nrm = perp(t), c = center[idx];
        double top = deckY[idx] - deckThk;        // rise to the deck underside
        double bot = groundY(c);
        if (top - bot < 0.5) continue;            // not elevated here: no pier
        double hw = width * 0.5, hd = depth * 0.5;
        auto P = [&](double along, double across, double yy) {
            Vec2 p = c + t * along + nrm * across;
            return Vec3(p.x, yy, p.y);
        };
        Vec3 b00 = P(-hd, -hw, bot), b10 = P(hd, -hw, bot), b11 = P(hd, hw, bot), b01 = P(-hd, hw, bot);
        Vec3 t00 = P(-hd, -hw, top), t10 = P(hd, -hw, top), t11 = P(hd, hw, top), t01 = P(-hd, hw, top);
        auto quad = [&](const Vec3& a, const Vec3& b, const Vec3& cc, const Vec3& d, const Vec3& nv) {
            MeshBuilder::emitTri(mesh, a, b, cc, nv, color);
            MeshBuilder::emitTri(mesh, a, cc, d, nv, color);
        };
        Vec3 nA(nrm.x, 0, nrm.y), nT(t.x, 0, t.y);
        quad(b01, t01, t11, b11, nA);             // +across faces
        quad(b00, b10, t10, t00, -nA);
        quad(b10, b11, t11, t10, nT);             // +along faces
        quad(b00, t00, t01, b01, -nT);
    }
    return mesh;
}




namespace {
// A thin painted stripe `off` to the side of the centerline, riding `deckY + lift`, optionally
// dashed by arc length. Used for edge lines, lane dividers, and the centreline.
void emitStripe(RenderMesh& mesh, const std::vector<Vec2>& cl, const std::vector<double>& deckY,
                double off, double w, double lift, const Vec3& col, double dashLen, double dashGap) {
    const int n = static_cast<int>(cl.size());
    double s = 0.0;
    for (int i = 0; i + 1 < n; ++i) {
        Vec2 ab = cl[i + 1] - cl[i]; double L = ab.length();
        if (L < 1e-9) continue;
        Vec2 dir = ab / L, nrm = perp(dir);
        if (dashLen > 0.0) {                              // skip the gap part of the dash cycle
            double phase = std::fmod(s, dashLen + dashGap);
            if (phase >= dashLen) { s += L; continue; }
        }
        Vec2 a = cl[i] + nrm * off, b = cl[i + 1] + nrm * off;
        double ya = deckY[i] + lift, yb = deckY[i + 1] + lift;
        Vec2 aw = nrm * (w * 0.5);
        Vec3 AL(a.x + aw.x, ya, a.y + aw.y), AR(a.x - aw.x, ya, a.y - aw.y);
        Vec3 BL(b.x + aw.x, yb, b.y + aw.y), BR(b.x - aw.x, yb, b.y - aw.y);
        MeshBuilder::emitTri(mesh, AL, AR, BR, Vec3(0, 1, 0), col);
        MeshBuilder::emitTri(mesh, AL, BR, BL, Vec3(0, 1, 0), col);
        s += L;
    }
}
}  // namespace


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

namespace {
// Squared distance from p to segment ab.
double segDist2(const Vec2& p, const Vec2& a, const Vec2& b) {
    Vec2 ab = b - a;
    double len2 = ab.lengthSquared();
    double t = len2 < 1e-12 ? 0.0 : std::max(0.0, std::min(1.0, dot(p - a, ab) / len2));
    Vec2 c = a + ab * t;
    return (p - c).lengthSquared();
}
}  // namespace


// Per-spine smoothed deck profiles with junction RECONCILIATION: each chain's
// profile is grade-limited independently (roadProfile), then chains sharing an
// endpoint AGREE on its height (the average of their independent ends, blended
// back linearly along each chain) — so decks meet flush at junctions on 3D
// terrain, and the terrain-conform pass can carve to exactly this surface
// (device: "the road is being buried by the terrain — it's not conforming").
std::vector<std::vector<double>> weldChainProfiles(
    const std::vector<UnionSpine>& spines,
    const std::function<double(double, double)>& heightAt, double topY,
    double maxGrade, double overlapReach) {
    std::vector<std::vector<double>> out(spines.size());
    std::vector<std::vector<double>> arcs(spines.size());
    for (std::size_t si = 0; si < spines.size(); ++si) {
        const UnionSpine& sp = spines[si];
        const int n = static_cast<int>(sp.points.size());
        if (n < 2) continue;
        std::vector<double> sArc(n, 0.0);
        for (int i = 1; i < n; ++i)
            sArc[i] = sArc[i - 1] + (sp.points[i] - sp.points[i - 1]).length();
        arcs[si] = sArc;
        // AUTHORED absolute heights (corridor deck / ramp): ride them as-is,
        // untouched by drape, junction-min, overlap-min, or the topY offset
        // below — they are already the real world Y. This is the 3-D path.
        if (!sp.yAbs.empty() && static_cast<int>(sp.yAbs.size()) == n) {
            out[si] = sp.yAbs;
            continue;
        }
        if (!heightAt) { out[si].assign(n, 0.0); continue; }   // += topY below
        std::vector<double> ground(n);
        for (int i = 0; i < n; ++i)
            ground[i] = heightAt(sp.points[i].x, sp.points[i].y);
        out[si] = roadProfile(ground, sArc, maxGrade);
    }
    if (heightAt) {
        // Junction endpoints: every incident chain takes the LOWEST arriving
        // deck — the SAME rule the terrain carve's overlapping-footprint fold
        // uses (lowest plane wins), so deck and carved ground agree at the
        // node. The old MEAN left decks up to half the arms' spread above the
        // min-carved ground (probe: 2-3 m proud walls on junction approaches,
        // all worst sites 7-28 m from a junction).
        auto key = [](const Vec2& v) {
            return std::make_pair(static_cast<long long>(std::llround(v.x * 8)),
                                  static_cast<long long>(std::llround(v.y * 8)));
        };
        std::map<std::pair<long long, long long>, double> nodes;
        for (std::size_t si = 0; si < spines.size(); ++si) {
            if (out[si].empty() || spines[si].closed) continue;
            if (!spines[si].yAbs.empty()) continue;   // authored deck: fixed
            const auto& pts = spines[si].points;
            if ((pts.front() - pts.back()).length() < 1e-6) continue;   // ring
            auto foldMin = [&](const Vec2& v, double h) {
                auto it = nodes.find(key(v));
                if (it == nodes.end()) nodes.emplace(key(v), h);
                else it->second = std::min(it->second, h);
            };
            foldMin(pts.front(), out[si].front());
            foldMin(pts.back(), out[si].back());
        }
        for (std::size_t si = 0; si < spines.size(); ++si) {
            if (out[si].empty() || spines[si].closed) continue;
            if (!spines[si].yAbs.empty()) continue;   // authored deck: fixed
            const auto& pts = spines[si].points;
            if ((pts.front() - pts.back()).length() < 1e-6) continue;
            const double dA = nodes.at(key(pts.front())) - out[si].front();
            const double dB = nodes.at(key(pts.back())) - out[si].back();
            const double L = std::max(1e-6, arcs[si].back());
            for (std::size_t i = 0; i < out[si].size(); ++i) {
                const double t = arcs[si][i] / L;
                out[si][i] += dA * (1.0 - t) + dB * t;   // linear blend to agree
            }
        }
    }
    if (heightAt && overlapReach > 0.0) {
        // MID-SPAN overlap reconciliation (P3.2): where corridors overlap, every
        // deck takes the LOWEST overlapping profile — the same rule the terrain
        // carve uses — so deck and carved ground cannot disagree there. Sampled
        // against the pre-adjustment snapshot so the pass is order-independent.
        const std::vector<std::vector<double>> snap = out;
        for (std::size_t si = 0; si < spines.size(); ++si) {
            if (out[si].size() < 2) continue;
            if (!spines[si].yAbs.empty()) continue;   // authored deck: not lowered
            const auto& pts = spines[si].points;
            for (std::size_t k = 0; k < out[si].size(); ++k) {
                const Vec2& q = pts[k];
                for (std::size_t sj = 0; sj < spines.size(); ++sj) {
                    if (snap[sj].size() < 2) continue;
                    const auto& pj = spines[sj].points;
                    const double reach = spines[sj].halfWidth + overlapReach;
                    // SELF-overlap counts too (an S-curve passing near its own
                    // earlier leg: the carve's lowest-plane rule doesn't care
                    // about chain identity, so neither can the deck) — but only
                    // legs FAR AWAY along the arc, or every sample would just
                    // min against its own neighbourhood.
                    const bool self = sj == si;
                    const double arcWin = 3.0 * (spines[si].halfWidth + reach);
                    for (std::size_t i = 0; i + 1 < pj.size(); ++i) {
                        if (self && std::fabs(arcs[sj][i] - arcs[si][k]) < arcWin) continue;
                        Vec2 ab = pj[i + 1] - pj[i];
                        double L2 = ab.lengthSquared();
                        double t = L2 < 1e-12 ? 0.0
                                              : std::max(0.0, std::min(1.0, dot(q - pj[i], ab) / L2));
                        if ((q - (pj[i] + ab * t)).length() > reach) continue;
                        out[si][k] = std::min(out[si][k],
                                              snap[sj][i] + (snap[sj][i + 1] - snap[sj][i]) * t);
                    }
                }
            }
            // Ease the approaches back into any dip at maxGrade (lower-only, so
            // the reconciled overlap height is never raised back up).
            const auto& sArc = arcs[si];
            for (std::size_t k = 1; k < out[si].size(); ++k)
                out[si][k] = std::min(out[si][k],
                                      out[si][k - 1] + maxGrade * (sArc[k] - sArc[k - 1]));
            for (std::size_t k = out[si].size() - 1; k-- > 0;)
                out[si][k] = std::min(out[si][k],
                                      out[si][k + 1] + maxGrade * (sArc[k + 1] - sArc[k]));
        }
    }
    for (std::size_t si = 0; si < out.size(); ++si) {
        if (!spines[si].yAbs.empty()) continue;   // authored deck: already absolute
        for (double& v : out[si]) v += topY;
    }
    return out;
}

// (weldSolid — the earcut polygon-union street mesher — DELETED, roads-v2
// S6: the swept lattice is the one road mesher. weldChainSpines /
// weldChainProfiles / clearanceProfile survive: the lattice builds on them.)






namespace {
// Drop `trim` of arc length from each end of a polyline (so a stripe stops short of a
// junction). Returns empty if the line is too short to survive.
std::vector<Vec2> trimEnds(const std::vector<Vec2>& pts, double trim) {
    int n = static_cast<int>(pts.size());
    if (n < 2) return {};
    std::vector<double> cum(n, 0);
    for (int i = 1; i < n; ++i) cum[i] = cum[i-1] + (pts[i] - pts[i-1]).length();
    double total = cum[n-1];
    if (total <= 2 * trim + 0.5) return {};
    double a = trim, b = total - trim;
    auto at = [&](double s) {
        int i = 1; while (i < n && cum[i] < s) ++i;
        double t = (s - cum[i-1]) / std::max(1e-9, cum[i] - cum[i-1]);
        return pts[i-1] + (pts[i] - pts[i-1]) * t;
    };
    std::vector<Vec2> out{ at(a) };
    for (int i = 1; i < n; ++i) if (cum[i] > a && cum[i] < b) out.push_back(pts[i]);
    out.push_back(at(b));
    return out;
}
}  // namespace


}  // namespace engine
