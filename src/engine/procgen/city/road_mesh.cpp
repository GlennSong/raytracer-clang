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

RenderMesh buildRoadMesh(const RoadGraph& g, const RoadMeshParams& p) {
    RenderMesh mesh;
    const int nNodes = static_cast<int>(g.nodes.size());
    const int nEdges = static_cast<int>(g.edges.size());

    auto height = [&](double x, double z) {
        return (p.heightAt ? p.heightAt(x, z) : 0.0) + p.lift;
    };
    auto ground = [&](double x, double z) {
        return p.heightAt ? p.heightAt(x, z) : 0.0;          // terrain, no lift
    };
    auto to3d = [&](const Vec2& v) { return Vec3(v.x, height(v.x, v.y), v.y); };

    // Road geometry lies flat-ish on the terrain, so every tri faces up; emitTri
    // winds each one clockwise-front (the engine convention) regardless of the
    // order the strip/junction code happens to pass its corners in.
    auto addTri = [&](const Vec3& a, const Vec3& b, const Vec3& c) {
        MeshBuilder::emitTri(mesh, a, b, c, Vec3(0, 1, 0), p.color);
    };

    // How many length-wise splits a span from s0 to s1 needs: ONE on flat ground,
    // more only where the terrain sags away from the straight chord by more than
    // conformTol (so polygons appear where the surface bends, not on flat runs). A
    // gentle conformStep cap keeps a very long flat quad from becoming unwieldy.
    auto stripSegs = [&](const Vec2& s0, const Vec2& s1) {
        double len = (s1 - s0).length();
        if (len < 1e-3) return 1;
        int cap = std::max(1, static_cast<int>(std::ceil(len / p.conformStep)));
        if (!p.heightAt) return cap;                       // flat world: just the cap
        double h0 = ground(s0.x, s0.y), h1 = ground(s1.x, s1.y), maxSag = 0.0;
        int probe = std::max(4, static_cast<int>(len / 2.0));
        for (int k = 1; k < probe; ++k) {
            double t = static_cast<double>(k) / probe;
            Vec2 m = lerp(s0, s1, t);
            maxSag = std::max(maxSag, std::fabs(ground(m.x, m.y) - (h0 + (h1 - h0) * t)));
        }
        int bySag = static_cast<int>(std::ceil(maxSag / std::max(1e-3, p.conformTol)));
        return std::min(std::max({1, cap, bySag}), static_cast<int>(len) + 1);
    };

    // A raised sidewalk skirt running along a curb line P0->P1, with `outN` the
    // unit direction away from the road. Per sub-segment it emits the curb lip
    // (vertical face toward the street), the slab top (raised curbHeight above the
    // road), and an outer face dropping back to the ground — so the kerb reads as
    // a real raised sidewalk, not a painted stripe. No-op when disabled.
    // A continuous sidewalk between an inner rail (the carriageway edge) and an outer
    // rail: curb lip toward the street, raised slab, outer face dropping to the
    // ground, split lengthwise only where the terrain bends. Used for both the chain
    // verges and the junction corners, so the kerb sweeps every corner as one rail
    // (no fanned slats) and there's no notch where pieces meet. No-op when disabled.
    auto sidewalkRail = [&](const std::vector<Vec2>& inner, const std::vector<Vec2>& outer) {
        if (p.sidewalkWidth <= 0.0) return;
        int n = static_cast<int>(inner.size());
        const Vec3 nUp(0, 1, 0);
        for (int i = 0; i < n - 1; ++i) {
            Vec2 toStreet = inner[i] - outer[i];                 // outer -> inner = toward road
            double tl = toStreet.length();
            Vec2 inDir = (tl > 1e-9) ? toStreet / tl : perp(inner[i + 1] - inner[i]);
            Vec3 nIn(inDir.x, 0, inDir.y), nOut(-inDir.x, 0, -inDir.y);
            int segs = stripSegs(inner[i], inner[i + 1]);
            for (int s = 1; s <= segs; ++s) {
                double u0 = static_cast<double>(s - 1) / segs, u1 = static_cast<double>(s) / segs;
                Vec2 a0 = lerp(inner[i], inner[i + 1], u0), a1 = lerp(inner[i], inner[i + 1], u1);
                Vec2 c0 = lerp(outer[i], outer[i + 1], u0), c1 = lerp(outer[i], outer[i + 1], u1);
                double r0 = height(a0.x, a0.y), r1 = height(a1.x, a1.y);
                double t0 = r0 + p.curbHeight, t1 = r1 + p.curbHeight;
                double g0 = std::min(ground(c0.x, c0.y), t0 - 0.01);
                double g1 = std::min(ground(c1.x, c1.y), t1 - 0.01);
                Vec3 A0(a0.x, r0, a0.y), A1(a1.x, r1, a1.y);
                Vec3 B0(a0.x, t0, a0.y), B1(a1.x, t1, a1.y);
                Vec3 C0(c0.x, t0, c0.y), C1(c1.x, t1, c1.y);
                Vec3 D0(c0.x, g0, c0.y), D1(c1.x, g1, c1.y);
                MeshBuilder::emitQuad(mesh, A0, A1, B1, B0, nIn, p.curbColor);
                MeshBuilder::emitQuad(mesh, B0, B1, C1, C0, nUp, p.sidewalkColor);
                MeshBuilder::emitQuad(mesh, C0, C1, D1, D0, nOut, p.curbColor);
            }
        }
    };

    // A flat strip between two cross-sections, draped on the terrain and split along
    // its length only where the ground bends (stripSegs) — flat ground stays one quad.
    // The carriageway ribbon: a0/b0 are the LEFT edge, a1/b1 the RIGHT, `sA`/`sB` the
    // arc-length at the two cross-sections. Each sub-quad bakes road-local UV so the
    // RoadMarkings surface can paint lane lines: u = lateral encoded as -1 (left) /
    // +1 (right) offset by +2 (so non-carriageway geometry, left at u=0, is excluded),
    // v = arc-length down the road (drives dashes). Plain asphalt where unused.
    auto addStrip = [&](const Vec2& a0, const Vec2& a1, const Vec2& b0,
                        const Vec2& b1, double sA = 0.0, double sB = 0.0) {
        int segs = stripSegs((a0 + a1) * 0.5, (b0 + b1) * 0.5);
        Vec2 prev0 = a0, prev1 = a1; double vPrev = sA;
        for (int s = 1; s <= segs; ++s) {
            double t = static_cast<double>(s) / segs;
            Vec2 c0 = lerp(a0, b0, t), c1 = lerp(a1, b1, t);
            double vCur = sA + (sB - sA) * t;
            MeshBuilder::emitQuadUV(mesh, to3d(prev0), to3d(prev1), to3d(c1), to3d(c0),
                                    Vec3(0, 1, 0), p.color,
                                    1.0f, static_cast<float>(vPrev),    // prev0 = left
                                    3.0f, static_cast<float>(vPrev),    // prev1 = right
                                    3.0f, static_cast<float>(vCur),     // c1    = right
                                    1.0f, static_cast<float>(vCur));    // c0    = left
            prev0 = c0; prev1 = c1; vPrev = vCur;
        }
    };

    // A zebra crosswalk centred at `center`, bars running along the road `d` and
    // repeated across a `roadW`-wide carriageway, draped + raised onto the asphalt.
    auto crosswalkBand = [&](const Vec2& center, const Vec2& d, double roadW) {
        Vec2 across = perp(d);
        double depth = p.crosswalkDepth, bar = p.crosswalkBar, gap = p.crosswalkGap;
        int n = std::max(1, static_cast<int>(roadW / (bar + gap)));
        auto P = [&](const Vec2& xz) {
            return Vec3(xz.x, height(xz.x, xz.y) + p.markLift, xz.y);
        };
        for (int i = 0; i < n; ++i) {
            double s = -roadW * 0.5 + bar * 0.5 + i * (bar + gap);
            if (std::fabs(s) > roadW * 0.5 - bar * 0.4) continue;
            Vec2 bc = center + across * s;
            Vec2 e0 = d * (depth * 0.5), e1 = across * (bar * 0.5);
            MeshBuilder::emitQuad(mesh, P(bc - e0 - e1), P(bc + e0 - e1),
                                  P(bc + e0 + e1), P(bc - e0 + e1),
                                  Vec3(0, 1, 0), p.crosswalkColor);
        }
    };


    // Incident edges per node.
    std::vector<std::vector<int>> inc(nNodes);
    for (int e = 0; e < nEdges; ++e) {
        inc[g.edges[e].a].push_back(e);
        inc[g.edges[e].b].push_back(e);
    }

    // Flag the sharp degree-2 bends (hairpins): a turn this tight can't be a simple
    // bend — the widened ribbon would fold — so it's built as a turning pad below.
    std::vector<char> hairpin(nNodes, 0);
    if (p.hairpinDeflection > 0.0) {
        for (int v = 0; v < nNodes; ++v) {
            if (static_cast<int>(inc[v].size()) != 2) continue;
            Vec2 V = g.nodes[v].pos;
            auto dir = [&](int e) {
                int o = (g.edges[e].a == v) ? g.edges[e].b : g.edges[e].a;
                return normalize(g.nodes[o].pos - V);
            };
            Vec2 d0 = dir(inc[v][0]), d1 = dir(inc[v][1]);
            double between = std::acos(std::max(-1.0, std::min(1.0, dot(d0, d1))));
            if (3.14159265358979 - between > p.hairpinDeflection) hairpin[v] = 1;   // deflection too sharp
        }
    }

    // trim[e][0] at endpoint a, trim[e][1] at endpoint b: how far the ribbon is
    // pulled back from each node so it stops at the junction edge.
    std::vector<std::array<double, 2>> trim(nEdges, {0.0, 0.0});
    auto edgeLen = [&](int e) {
        return (g.nodes[g.edges[e].b].pos - g.nodes[g.edges[e].a].pos).length();
    };

    // --- Junctions: compute trims + build the pad, per node (degree >= 3) ------
    struct Arm {
        int edge; double ang; Vec2 d; double w; double s;   // s = setback at this node
    };
    for (int v = 0; v < nNodes; ++v) {
        if (static_cast<int>(inc[v].size()) < 3) continue;
        Vec2 V = g.nodes[v].pos;

        std::vector<Arm> arms;
        for (int e : inc[v]) {
            int other = (g.edges[e].a == v) ? g.edges[e].b : g.edges[e].a;
            Vec2 d = normalize(g.nodes[other].pos - V);
            arms.push_back({e, std::atan2(d.y, d.x), d, g.edges[e].width * 0.5, p.minSetback});
        }
        std::sort(arms.begin(), arms.end(),
                  [](const Arm& a, const Arm& b) { return a.ang < b.ang; });

        // Each adjacent pair of arms shares a curb corner = where arm A's left
        // side-line meets arm B's right side-line. The setback along each arm is
        // pushed out to clear that corner.
        int m = static_cast<int>(arms.size());
        for (int k = 0; k < m; ++k) {
            Arm& A = arms[k];
            Arm& B = arms[(k + 1) % m];
            Vec2 O1 = V + perp(A.d) * A.w;     // A's left (CCW) edge origin
            Vec2 O2 = V - perp(B.d) * B.w;     // B's right (CW) edge origin
            Vec2 r = O2 - O1;
            double denom = cross(A.d, B.d);
            double sA, sB;
            if (std::fabs(denom) < 1e-6) {     // near-parallel arms
                sA = A.w + B.w; sB = A.w + B.w;
            } else {
                sA = cross(r, B.d) / denom;
                sB = -cross(r, A.d) / denom;
            }
            A.s = std::max(A.s, sA);
            B.s = std::max(B.s, sB);
        }

        // A many-armed hub pushes every arm out to the plaza radius, so the pad
        // fills a clean circular plaza rather than a cramped fan of corners.
        double floorS = (m >= p.plazaMinArms && p.plazaRadius > 0.0)
                            ? std::max(p.minSetback, p.plazaRadius)
                            : p.minSetback;

        // Clamp + record the trims, lay out the per-arm mouth corners (an extra
        // cornerRadius of setback makes room for the rounded kerb returns).
        std::vector<Vec2> mouthR(m), mouthL(m);
        for (int k = 0; k < m; ++k) {
            Arm& a = arms[k];
            double maxS = edgeLen(a.edge) * 0.45;
            a.s = std::min(std::max(a.s + p.cornerRadius, floorS), std::max(floorS, maxS));
            int end = (g.edges[a.edge].a == v) ? 0 : 1;
            trim[a.edge][end] = a.s;
            Vec2 base = V + a.d * a.s;
            mouthR[k] = base - perp(a.d) * a.w;
            mouthL[k] = base + perp(a.d) * a.w;
        }
        // Each corner runs from arm k's LEFT mouth to arm k+1's RIGHT mouth. With a
        // cornerRadius it is a circular arc centred where the two arms' kerb lines
        // cross (a real rounded kerb return); otherwise a straight chamfer. Near-
        // straight corners (a side road off a through street) stay straight.
        std::vector<std::vector<Vec2>> corner(m);
        for (int k = 0; k < m; ++k) {
            const Arm& A = arms[k]; const Arm& B = arms[(k + 1) % m];
            Vec2 Aleft = mouthL[k], Bright = mouthR[(k + 1) % m];
            std::vector<Vec2>& cc = corner[k];
            cc.push_back(Aleft);
            double denom = cross(A.d, B.d);
            if (p.cornerRadius > 0.0 && std::fabs(denom) > 1e-6) {
                // The corner is where arm A's left kerb line meets arm B's right kerb line.
                Vec2 pA = V + perp(A.d) * A.w, pB = V - perp(B.d) * B.w;
                Vec2 C = pA + A.d * (cross(pB - pA, B.d) / denom);   // kerb-line crossing
                // A true fixed-radius arc tangent to both kerbs, capped so the tangent
                // points stay between the corner and the mouths (no overrun of the trim).
                double tMax = std::min(dot(Aleft - C, -A.d), dot(Bright - C, -B.d));
                std::vector<Vec2> arc =
                    curbReturnFillet(C, -A.d, -B.d, p.cornerRadius, tMax);
                for (const Vec2& pt : arc) cc.push_back(pt);
            }
            cc.push_back(Bright);
        }
        // The pad's boundary ring: each arm's mouth, then its outgoing corner (mouth left
        // -> arc -> next mouth right). Triangulate the ring directly (ear clipping) rather
        // than fanning from V — a fan assumes the ring is star-convex from the node, which
        // T-junctions, mixed-width arms and notched rings break (the fan self-overlaps).
        std::vector<Vec2> ring;
        for (int k = 0; k < m; ++k) {
            ring.push_back(mouthR[k]);
            for (std::size_t i = 0; i + 1 < corner[k].size(); ++i) ring.push_back(corner[k][i]);
        }
        for (const std::array<int, 3>& t : triangulatePolygon(ring))
            addTri(to3d(ring[t[0]]), to3d(ring[t[1]]), to3d(ring[t[2]]));

        // Sidewalk wraps each corner as ONE rail — inner = the corner polyline, outer = each point
        // pushed out by sidewalkWidth. The push direction SWEEPS from arm A's outward normal at the
        // start mouth to arm B's at the end mouth, so the corner sidewalk meets each arm's sidewalk
        // exactly (both offset along the same normal there). A radial push from the node diverged from
        // the arm normals as the corner angle sharpened, tearing a gap open at acute junctions.
        for (int k = 0; k < m; ++k) {
            const Arm& A = arms[k]; const Arm& B = arms[(k + 1) % m];
            Vec2 nA = perp(A.d), nB = perp(B.d) * -1.0;   // arm A's left / arm B's right outward normal
            std::vector<Vec2>& cc = corner[k];
            std::vector<Vec2> out(cc.size());
            for (std::size_t i = 0; i < cc.size(); ++i) {
                double t = (cc.size() > 1) ? static_cast<double>(i) / (cc.size() - 1) : 0.0;
                Vec2 dir = nA * (1.0 - t) + nB * t;
                double dl = dir.length();
                out[i] = cc[i] + ((dl > 1e-6) ? dir / dl : nA) * p.sidewalkWidth;
            }
            sidewalkRail(cc, out);
        }

        // Crosswalk across each arm, just outside the pad mouth (where the ribbon
        // begins) — so a crossing lands exactly at the intersection. Skip arms with
        // no room for the band on their ribbon.
        if (p.crosswalks)
            for (const Arm& a : arms) {
                if (edgeLen(a.edge) - a.s < p.crosswalkDepth + 1.0) continue;
                if (a.w * 2.0 >= p.crosswalkMaxWidth) continue;   // freeway arm: no zebra
                Vec2 center = V + a.d * (a.s + p.crosswalkDepth * 0.5 + 0.4);
                crosswalkBand(center, a.d, a.w * 2.0);
            }
    }

    // --- Hairpins (sharp degree-2): a clean turning-head DISC. The junction pad
    // assumes arms spread around the node and fan-triangulates a ring; two
    // near-parallel hairpin arms make that fan wrap a ~340-degree triangle across the
    // back and fold over itself. A disc centred on the apex can't fold: pull both
    // legs back into it and fill it as a fan, so the tight inner curve is covered by
    // a turning head rather than a folded ribbon.
    for (int v = 0; v < nNodes; ++v) {
        if (!hairpin[v]) continue;
        Vec2 V = g.nodes[v].pos;
        double maxHalf = std::max(g.edges[inc[v][0]].width, g.edges[inc[v][1]].width) * 0.5;
        double tr = maxHalf;                                  // pull leg ends inside the disc
        for (int e : inc[v]) {
            int end = (g.edges[e].a == v) ? 0 : 1;
            trim[e][end] = std::max(trim[e][end], std::min(tr, edgeLen(e) * 0.45));
        }
        double R = maxHalf * 1.5 + p.sidewalkWidth;           // covers the trimmed leg corners
        Vec3 c = to3d(V);
        int segs = std::max(10, static_cast<int>(std::ceil(2.0 * 3.14159265 * R / 4.0)));
        Vec2 prev = V + Vec2(R, 0);
        for (int s = 1; s <= segs; ++s) {
            double ang = 2.0 * 3.14159265 * s / segs;
            Vec2 cur = V + Vec2(std::cos(ang) * R, std::sin(ang) * R);
            addTri(c, to3d(prev), to3d(cur));                 // CCW fan -> upward normal
            prev = cur;
        }
    }

    // --- Continuous chains -----------------------------------------------------
    // A curved street is a polyline of many short edges. Stroking each edge as its
    // own full-width ribbon makes consecutive straight pieces OVERLAP on the inside
    // of every bend and the per-edge lane lines double up at every joint (ADR-0048).
    // Instead, trace each run of degree-2 nodes between junctions into one CHAIN and
    // stroke it as a single ribbon with MITRED joins — so a sampled curve is one
    // smooth strip and the markings run unbroken down its length. Chains break at
    // junctions (degree != 2), hairpins (turned by a disc above) and dead ends.

    // Paint a thin draped stripe down a (continuous, already-offset) polyline — solid
    // or dashed with a phase that carries across the joints so the dashes are even.
    auto paintLine = [&](const std::vector<Vec2>& Q, const Vec3& col, bool dashed) {
        double hwm = p.markWidth * 0.5;
        auto P3 = [&](const Vec2& xz) { return Vec3(xz.x, height(xz.x, xz.y) + p.markLift, xz.y); };
        auto quad = [&](const Vec2& a, const Vec2& b, const Vec2& nrm) {
            MeshBuilder::emitQuad(mesh, P3(a - nrm), P3(b - nrm), P3(b + nrm), P3(a + nrm),
                                  Vec3(0, 1, 0), col);
        };
        double phase = 0.0, period = p.dashLength + p.dashGap;
        for (int i = 0; i + 1 < static_cast<int>(Q.size()); ++i) {
            Vec2 a = Q[i], b = Q[i + 1], dd = b - a;
            double L = dd.length();
            if (L < 1e-6) continue;
            Vec2 u = dd / L, nrm = perp(u) * hwm;
            if (!dashed) {
                int segs = stripSegs(a, b);
                for (int s = 0; s < segs; ++s)
                    quad(lerp(a, b, static_cast<double>(s) / segs),
                         lerp(a, b, static_cast<double>(s + 1) / segs), nrm);
            } else {
                double dist = 0.0;
                while (dist < L) {
                    double lp = std::fmod(phase + dist, period);
                    if (lp < p.dashLength) {
                        double run = std::min(p.dashLength - lp, L - dist);
                        quad(a + u * dist, a + u * (dist + run), nrm);
                        dist += run;
                    } else {
                        dist += period - lp;
                    }
                }
                phase += L;
            }
        }
    };

    std::vector<char> usedEdge(nEdges, 0);
    auto isBreak = [&](int v) { return static_cast<int>(inc[v].size()) != 2 || hairpin[v]; };
    auto traceChain = [&](int v, int e) {
        std::vector<int> nodes{ v }, edges;
        int cur = v, ce = e;
        for (;;) {
            usedEdge[ce] = 1;
            int nx = (g.edges[ce].a == cur) ? g.edges[ce].b : g.edges[ce].a;
            nodes.push_back(nx); edges.push_back(ce);
            if (isBreak(nx)) break;
            int ne = -1;
            for (int e2 : inc[nx]) if (e2 != ce && !usedEdge[e2]) { ne = e2; break; }
            if (ne < 0) break;
            cur = nx; ce = ne;
        }
        return std::make_pair(std::move(nodes), std::move(edges));
    };

    std::vector<std::pair<std::vector<int>, std::vector<int>>> chains;
    for (int v = 0; v < nNodes; ++v)
        if (isBreak(v))
            for (int e : inc[v]) if (!usedEdge[e]) chains.push_back(traceChain(v, e));
    for (int e = 0; e < nEdges; ++e)                     // pure degree-2 loops, no break node
        if (!usedEdge[e]) chains.push_back(traceChain(g.edges[e].a, e));

    const double MITER_LIMIT = 2.5;

    // Build one side's sidewalk rails (inner = carriageway edge, outer = inner + sw)
    // robustly for ANY curvature: a ROUND join on the convex side (so a sharp bend
    // sweeps an arc instead of spiking out a mitre) and an un-amplified mitre on the
    // concave side (so the inner kerb never folds through itself on a tight inner
    // bend). `side` = +1 left / -1 right. inner/outer come back the same length.
    const double PI = 3.14159265358979;
    auto buildSideRails = [&](const std::vector<Vec2>& P, const std::vector<double>& hwv,
                              double sw, double side,
                              std::vector<Vec2>& inner, std::vector<Vec2>& outer) {
        int n = static_cast<int>(P.size());
        inner.clear(); outer.clear();
        std::vector<Vec2> seg(std::max(0, n - 1)), nrm(std::max(0, n - 1));
        for (int i = 0; i + 1 < n; ++i) {
            Vec2 d = P[i + 1] - P[i]; double l = d.length();
            seg[i] = (l > 1e-9) ? d / l : Vec2(1, 0);
            nrm[i] = perp(seg[i]) * side;                // outward for this side
        }
        auto add = [&](const Vec2& base, const Vec2& u, double hw) {
            inner.push_back(base + u * hw); outer.push_back(base + u * (hw + sw));
        };
        for (int i = 0; i < n; ++i) {
            if (i == 0) { add(P[0], nrm[0], hwv[0]); continue; }
            if (i == n - 1) { add(P[n - 1], nrm[n - 2], hwv[n - 1]); continue; }
            Vec2 n0 = nrm[i - 1], n1 = nrm[i];
            double theta = std::atan2(cross(seg[i - 1], seg[i]), dot(seg[i - 1], seg[i]));
            if (side * theta < -1e-3) {                  // convex: round join
                double a0 = std::atan2(n0.y, n0.x), a1 = std::atan2(n1.y, n1.x);
                double sweep = a1 - a0;
                while (sweep <= -PI) sweep += 2 * PI;
                while (sweep >   PI) sweep -= 2 * PI;
                int steps = std::max(1, static_cast<int>(std::ceil(std::fabs(sweep) / 0.4)));
                for (int s = 0; s <= steps; ++s)
                    add(P[i], Vec2(std::cos(a0 + sweep * s / steps),
                                   std::sin(a0 + sweep * s / steps)), hwv[i]);
            } else {                                     // concave / straight: un-amplified
                Vec2 bis = n0 + n1; double bl = bis.length();
                add(P[i], (bl > 1e-6) ? bis / bl : n1, hwv[i]);
            }
        }
    };

    for (auto& ch : chains) {
        const std::vector<int>& cn = ch.first;
        const std::vector<int>& ce = ch.second;
        int np = static_cast<int>(cn.size());
        if (np < 2) continue;
        std::vector<Vec2> P(np);
        std::vector<double> hw(np);
        for (int i = 0; i < np; ++i) P[i] = g.nodes[cn[i]].pos;
        for (int i = 0; i < np; ++i) {
            double wa = g.edges[ce[std::max(0, i - 1)]].width;
            double wb = g.edges[ce[std::min(np - 2, i)]].width;
            hw[i] = 0.5 * std::max(wa, wb);
        }
        // Pull the ends back to the junction setbacks (so the chain meets the pad), walking the
        // polyline by ARC LENGTH. curved:true densifies each edge into many short samples, so a
        // setback usually spans SEVERAL of them; the old clamp to the first segment under-trimmed and
        // the ribbon then overran the junction pad — z-fight + a terrain gap at curved junctions
        // (road-network-v2-plan T3.2). Drop whole segments inside the setback, then land the new end
        // partway along the next; keep at least two nodes so the chain stays a strokeable strip.
        {
            int e0 = ce.front(); double t0 = trim[e0][(g.edges[e0].a == cn[0]) ? 0 : 1];
            int eL = ce.back();  double tL = trim[eL][(g.edges[eL].b == cn[np - 1]) ? 1 : 0];
            auto advance = [](std::vector<Vec2>& pts, std::vector<double>& w, double t, bool front) {
                if (t <= 0.0 || pts.size() < 2) return;
                if (!front) { std::reverse(pts.begin(), pts.end()); std::reverse(w.begin(), w.end()); }
                double acc = 0.0;
                while (pts.size() > 2) {
                    double seg = (pts[1] - pts[0]).length();
                    if (acc + seg >= t) break;
                    acc += seg; pts.erase(pts.begin()); w.erase(w.begin());
                }
                double seg = (pts[1] - pts[0]).length();
                double rem = std::min(std::max(0.0, t - acc), seg - 0.1);
                if (seg > 1e-9) pts[0] = pts[0] + (pts[1] - pts[0]) * (rem / seg);
                if (!front) { std::reverse(pts.begin(), pts.end()); std::reverse(w.begin(), w.end()); }
            };
            advance(P, hw, t0, true);
            advance(P, hw, tL, false);
            np = static_cast<int>(P.size());
        }
        // Per-vertex mitre direction m (unit, +left) and length factor f (>= 1).
        std::vector<Vec2> sn(np - 1);
        for (int i = 0; i < np - 1; ++i) {
            Vec2 dd = P[i + 1] - P[i]; double l = dd.length();
            sn[i] = perp((l > 1e-9) ? dd / l : Vec2(1, 0));
        }
        std::vector<Vec2> m(np); std::vector<double> f(np);
        m[0] = sn[0]; f[0] = 1.0; m[np - 1] = sn[np - 2]; f[np - 1] = 1.0;
        for (int i = 1; i < np - 1; ++i) {
            Vec2 bis = sn[i - 1] + sn[i]; double bl = bis.length();
            if (bl < 1e-6) { m[i] = sn[i]; f[i] = 1.0; }
            else {
                m[i] = bis / bl;
                double c = dot(m[i], sn[i]);
                f[i] = (c > 1e-3) ? std::min(1.0 / c, MITER_LIMIT) : MITER_LIMIT;
            }
        }
        // Carriageway edges use the plain mitre — an overlap on a tight inner bend is
        // a harmless coplanar fill — so the asphalt stays a continuous strip.
        std::vector<Vec2> L(np), R(np);
        for (int i = 0; i < np; ++i) {
            Vec2 off = m[i] * f[i];
            L[i] = P[i] + off * hw[i]; R[i] = P[i] - off * hw[i];
        }
        std::vector<double> slen(np, 0.0);          // arc-length along the centerline (for v)
        for (int i = 1; i < np; ++i) slen[i] = slen[i - 1] + (P[i] - P[i - 1]).length();
        for (int i = 0; i < np - 1; ++i)
            addStrip(L[i], R[i], L[i + 1], R[i + 1], slen[i], slen[i + 1]);
        // Sidewalks use the robust offset (round convex joins, clamped concave) so the
        // raised kerb never spikes or folds, even on curves tighter than the road.
        std::vector<Vec2> li, lo, ri, ro;
        buildSideRails(P, hw, p.sidewalkWidth, +1.0, li, lo);
        buildSideRails(P, hw, p.sidewalkWidth, -1.0, ri, ro);
        sidewalkRail(li, lo);
        sidewalkRail(ri, ro);

        if (p.laneMarkings && !p.shaderMarkings) {
            double hwm = hw[np / 2], inset = p.markWidth * 1.5;
            int perSide = std::max(1, static_cast<int>(std::lround(hwm / p.laneWidth)));
            double laneW = hwm / perSide;
            auto offsetPoly = [&](auto offFn) {
                std::vector<Vec2> q(np);
                for (int i = 0; i < np; ++i) q[i] = P[i] + m[i] * (f[i] * offFn(i));
                return q;
            };
            paintLine(offsetPoly([&](int) { return  p.markWidth; }), p.centerColor, false);
            paintLine(offsetPoly([&](int) { return -p.markWidth; }), p.centerColor, false);
            for (int k = 1; k < perSide; ++k) {
                paintLine(offsetPoly([&](int) { return  k * laneW; }), p.laneColor, true);
                paintLine(offsetPoly([&](int) { return -k * laneW; }), p.laneColor, true);
            }
            paintLine(offsetPoly([&](int i) { return  (hw[i] - inset); }), p.laneColor, false);
            paintLine(offsetPoly([&](int i) { return -(hw[i] - inset); }), p.laneColor, false);
        }
    }

    return weldMesh(mesh);    // fuse the coincident pad/ribbon/sidewalk seams; index the mesh (T3.1)
}

RenderMesh bridgeDeck(const std::vector<Vec2>& pts, const std::vector<double>& deckY,
                      double halfWidth, const Vec3& deckColor, double thickness,
                      const Vec3& sideColor) {
    return bridgeDeck(pts, deckY, std::vector<double>(pts.size(), halfWidth), deckColor,
                      thickness, sideColor);
}

RenderMesh bridgeDeck(const std::vector<Vec2>& pts, const std::vector<double>& deckY,
                      const std::vector<double>& halfW, const Vec3& deckColor, double thickness,
                      const Vec3& sideColor) {
    RenderMesh mesh;
    const int n = static_cast<int>(pts.size());
    if (n < 2 || static_cast<int>(deckY.size()) != n || halfW.empty()) return mesh;
    auto W = [&](int i) { return halfW[std::min<int>(static_cast<int>(halfW.size()) - 1,
                                                     std::max(0, i))]; };
    auto Y = [&](int i) { return deckY[i]; };
    auto top = [&](const Vec2& v, double y) { return Vec3(v.x, y, v.y); };
    for (int i = 0; i + 1 < n; ++i) {
        Vec2 A = pts[i], B = pts[i + 1];
        Vec2 ab = B - A; double L = ab.length();
        if (L < 1e-9) continue;
        Vec2 nrm = perp(ab / L);
        double wa = W(i), wb = W(i + 1);
        Vec2 AL = A + nrm * wa, AR = A - nrm * wa;
        Vec2 BL = B + nrm * wb, BR = B - nrm * wb;
        double ya = Y(i), yb = Y(i + 1);
        // Deck top (faces up).
        MeshBuilder::emitTri(mesh, top(AL, ya), top(AR, ya), top(BR, yb), Vec3(0, 1, 0), deckColor);
        MeshBuilder::emitTri(mesh, top(AL, ya), top(BR, yb), top(BL, yb), Vec3(0, 1, 0), deckColor);
        // Left fascia (slab edge, faces +nrm) and right fascia (faces -nrm).
        Vec3 nL(nrm.x, 0, nrm.y);
        MeshBuilder::emitTri(mesh, top(AL, ya), top(BL, yb), top(BL, yb - thickness), nL, sideColor);
        MeshBuilder::emitTri(mesh, top(AL, ya), top(BL, yb - thickness), top(AL, ya - thickness), nL, sideColor);
        MeshBuilder::emitTri(mesh, top(AR, ya), top(AR, ya - thickness), top(BR, yb - thickness), -nL, sideColor);
        MeshBuilder::emitTri(mesh, top(AR, ya), top(BR, yb - thickness), top(BR, yb), -nL, sideColor);
    }
    return mesh;
}

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

RenderMesh deckBarriers(const std::vector<Vec2>& pts, const std::vector<double>& deckY,
                        double halfWidth, double height, const Vec3& color) {
    return deckBarriers(pts, deckY, std::vector<double>(pts.size(), halfWidth), height, color);
}

RenderMesh deckBarriers(const std::vector<Vec2>& pts, const std::vector<double>& deckY,
                        const std::vector<double>& halfW, double height, const Vec3& color) {
    RenderMesh mesh;
    const int n = static_cast<int>(pts.size());
    if (n < 2 || static_cast<int>(deckY.size()) != n || halfW.empty()) return mesh;
    auto W = [&](int i) { return halfW[std::min<int>(static_cast<int>(halfW.size()) - 1,
                                                     std::max(0, i))]; };
    auto wall = [&](double side) {                       // side = +1 (left) / -1 (right)
        for (int i = 0; i + 1 < n; ++i) {
            Vec2 ab = pts[i + 1] - pts[i]; double L = ab.length();
            if (L < 1e-9) continue;
            Vec2 nrm = perp(ab / L) * side;
            Vec2 a = pts[i] + nrm * W(i), b = pts[i + 1] + nrm * W(i + 1);
            double ya = deckY[i], yb = deckY[i + 1];
            Vec3 nv(nrm.x, 0, nrm.y);                     // face inward (toward the road)
            Vec3 ba(a.x, ya, a.y), bb(b.x, yb, b.y);
            Vec3 ta(a.x, ya + height, a.y), tb(b.x, yb + height, b.y);
            MeshBuilder::emitTri(mesh, ba, bb, tb, nv, color);
            MeshBuilder::emitTri(mesh, ba, tb, ta, nv, color);
            MeshBuilder::emitTri(mesh, ba, tb, bb, -nv, color);   // back face too (double-sided)
            MeshBuilder::emitTri(mesh, ba, ta, tb, -nv, color);
        }
    };
    wall(+1.0); wall(-1.0);
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

RenderMesh deckMarkings(const std::vector<Vec2>& cl, const std::vector<double>& deckY,
                        double halfWidth, const DeckMarkParams& p) {
    RenderMesh mesh;
    const int n = static_cast<int>(cl.size());
    if (n < 2 || static_cast<int>(deckY.size()) != n) return mesh;
    int lanes = std::max(1, static_cast<int>(std::lround(2.0 * halfWidth / p.laneWidth)));
    double edge = halfWidth - 0.35;                       // edge lines just inside the verge
    emitStripe(mesh, cl, deckY,  edge, p.markWidth, p.lift, p.laneColor, 0, 0);   // solid edges
    emitStripe(mesh, cl, deckY, -edge, p.markWidth, p.lift, p.laneColor, 0, 0);
    if (p.center)                                         // solid centreline
        emitStripe(mesh, cl, deckY, 0.0, p.markWidth, p.lift, p.centerColor, 0, 0);
    for (int k = 1; k < lanes; ++k) {                     // dashed lane dividers
        double off = -halfWidth + p.laneWidth * k;
        if (std::fabs(off) < 0.25 && p.center) continue;  // don't overpaint the centreline
        emitStripe(mesh, cl, deckY, off, p.markWidth, p.lift, p.laneColor, p.dashLength, p.dashGap);
    }
    return mesh;
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

RenderMesh weldRibbons(const std::vector<UnionSpine>& spines, double y, const Vec3& color,
                       double cornerRadius) {
    std::vector<Poly2> ribbons;
    for (const UnionSpine& s : spines) {
        if (s.points.size() < 2) continue;
        Poly2 r = ribbonOutline(s.points, s.halfWidth);
        if (r.size() >= 3) ribbons.push_back(std::move(r));
    }
    // Split the welded loops into exterior surfaces (CCW) and holes (CW, e.g. block interiors),
    // round each loop's convex corners (the unified curb-return fillet), bridge each surface's
    // holes in, then triangulate — so enclosed blocks stay open with rounded inner curbs.
    std::vector<Poly2> outers, holes;
    for (Poly2& L : polygonUnion(ribbons)) {
        Poly2 R = (cornerRadius > 0.0) ? roundPolygonCorners(L, cornerRadius, 5) : L;
        (signedArea(R) > 0 ? outers : holes).push_back(std::move(R));
    }
    RenderMesh mesh;
    for (const Poly2& outer : outers) {
        std::vector<Poly2> mine;
        for (const Poly2& h : holes)
            if (h.size() >= 3 && pointInPolygon(outer, centroid(h))) mine.push_back(h);
        Poly2 merged = mine.empty() ? outer : bridgeHoles(outer, mine);
        for (const std::array<int, 3>& t : triangulatePolygon(merged))
            MeshBuilder::emitTri(mesh, Vec3(merged[t[0]].x, y, merged[t[0]].y),
                                 Vec3(merged[t[1]].x, y, merged[t[1]].y),
                                 Vec3(merged[t[2]].x, y, merged[t[2]].y), Vec3(0, 1, 0), color);
    }
    return mesh;
}

// Harden a union-boundary loop: snap vertices to a fine grid (collapses the sub-mm zigzag slivers a
// glancing ribbon-vs-ribbon crossing leaves behind), drop consecutive duplicates, and remove needle
// SPIKES — a vertex whose two edges nearly reverse, which is what the sidewalk offset later amplifies
// into a visible spire at a sharp junction. Iterated until stable.
static Poly2 cleanPolygon(const Poly2& in, double grid) {
    Poly2 s;
    for (const Vec2& p : in) {
        Vec2 q(std::round(p.x / grid) * grid, std::round(p.y / grid) * grid);
        if (s.empty() || (q - s.back()).length() > grid * 0.5) s.push_back(q);
    }
    if (s.size() >= 2 && (s.front() - s.back()).length() <= grid * 0.5) s.pop_back();
    bool changed = true;
    while (changed && static_cast<int>(s.size()) > 3) {
        changed = false;
        int n = static_cast<int>(s.size());
        for (int i = 0; i < n; ++i) {
            Vec2 e0 = s[i] - s[(i + n - 1) % n], e1 = s[(i + 1) % n] - s[i];
            double l0 = e0.length(), l1 = e1.length();
            if (l0 < 1e-9 || l1 < 1e-9 || dot(e0, e1) / (l0 * l1) < -0.985) {   // dup or ~reversal spike
                s.erase(s.begin() + i); changed = true; break;
            }
        }
    }
    return s;
}

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

RenderMesh weldSolid(const std::vector<UnionSpine>& spines, const WeldSolidParams& p) {
    // 1. Per-spine profile: cumulative arc length + a smoothed, grade-limited height. Terrain is
    //    sampled along each centerline and ironed by roadProfile (follows hills, not every bump);
    //    with no terrain the height is the flat topY. These also carry the road-local UV.
    struct Prof { std::vector<Vec2> cl; std::vector<double> s; std::vector<double> h; double hw; bool closed; RoadClass klass; double mouthFront; double mouthBack; bool authored; int group; };
    std::vector<Prof> profs;
    // Per-chain-END junction MOUTH depth: how far the junction pad reaches along
    // this arm. The shipping pad is a disc of radius = the WIDEST incident arm's
    // half-width (road_net.cpp:384), so the mouth is that radius, NOT this arm's
    // own hw. Crosswalks set back by this (below) so they land just OUTSIDE the
    // pad even when a crossing road is wider — fixing the jut where a narrow arm
    // met a wider one. Chains meet at a junction by sharing an exact endpoint
    // position (the graph node), so group endpoints by quantized position.
    std::vector<double> frontMouth(spines.size()), backMouth(spines.size());
    {
        for (std::size_t si = 0; si < spines.size(); ++si) {
            frontMouth[si] = spines[si].halfWidth;   // fallback = old behaviour
            backMouth[si]  = spines[si].halfWidth;
        }
        auto key = [](const Vec2& q) {
            return (static_cast<long long>(std::llround(q.x * 20.0)) << 21) ^
                   static_cast<long long>(std::llround(q.y * 20.0));   // 5 cm cells
        };
        struct EP { std::size_t si; int end; };
        std::unordered_map<long long, std::vector<EP>> cells;
        for (std::size_t si = 0; si < spines.size(); ++si) {
            const auto& pts = spines[si].points;
            if (pts.size() < 2 || spines[si].closed) continue;
            cells[key(pts.front())].push_back({si, 0});
            cells[key(pts.back())].push_back({si, 1});
        }
        for (const auto& kv : cells) {
            if (kv.second.size() < 2) continue;                 // not a junction
            double widest = 0.0;
            for (const EP& e : kv.second) widest = std::max(widest, spines[e.si].halfWidth);
            const double mouth = widest * 1.02;                 // matches the pad radius
            for (const EP& e : kv.second)
                (e.end == 0 ? frontMouth : backMouth)[e.si] = mouth;
        }
    }
    {
        // Junction-RECONCILED deck profiles (shared with the terrain-conform
        // pass, so the ground is carved to the same surface the deck rides).
        std::vector<std::vector<double>> hs =
            weldChainProfiles(spines, p.heightAt, p.topY, p.maxGrade,
                              p.sidewalkWidth + 4.0);
        for (std::size_t si = 0; si < spines.size(); ++si) {
            const UnionSpine& sp = spines[si];
            const int n = static_cast<int>(sp.points.size());
            if (n < 2) continue;
            std::vector<double> sArc(n, 0.0);
            for (int i = 1; i < n; ++i)
                sArc[i] = sArc[i - 1] + (sp.points[i] - sp.points[i - 1]).length();
            bool closed = sp.closed ||
                (n >= 4 && (sp.points.front() - sp.points.back()).length() < 1e-6);
            profs.push_back({sp.points, sArc, hs[si], sp.halfWidth, closed, sp.klass,
                             frontMouth[si], backMouth[si], !sp.yAbs.empty(), 0});
        }
    }
    // ---- Grade separation (P4): split authored decks + assign weld groups ----
    // A deck that flies over a live road must not fuse with it in the 2-D union.
    // SPLIT each authored spine at the clearance-above-ground crossing: the LOW
    // foot of a ramp (deck < clearance over the ground) is reclassified to ground
    // so it welds into the street grid at its junction (and piers avoid it), while
    // the HIGH span stays a viaduct on piers. The seam vertex at deck==clearance
    // is shared bit-identically so the driving surface stays continuous.
    {
        auto groundY = [&](const Vec2& q) { return p.heightAt ? p.heightAt(q.x, q.y) : p.topY; };
        const double kNoCross = 1e4;   // seam-end mouth sentinel: no crosswalk mid-ramp
        std::vector<Prof> split;
        split.reserve(profs.size());
        for (const Prof& pr : profs) {
            if (!pr.authored || pr.closed || pr.cl.size() < 2) { split.push_back(pr); continue; }
            const int n = static_cast<int>(pr.cl.size());
            std::vector<Vec2> ac; std::vector<double> ah, asx, ae;
            auto pushA = [&](const Vec2& c, double h, double s, double e) {
                ac.push_back(c); ah.push_back(h); asx.push_back(s); ae.push_back(e); };
            pushA(pr.cl[0], pr.h[0], pr.s[0], pr.h[0] - groundY(pr.cl[0]));
            for (int i = 0; i + 1 < n; ++i) {
                const double e0 = pr.h[i]   - groundY(pr.cl[i]);
                const double e1 = pr.h[i+1] - groundY(pr.cl[i+1]);
                if ((e0 < p.clearance) != (e1 < p.clearance)) {   // crosses the band
                    const double t = (p.clearance - e0) / (e1 - e0);
                    pushA(pr.cl[i] + (pr.cl[i+1] - pr.cl[i]) * t,
                          pr.h[i] + (pr.h[i+1] - pr.h[i]) * t,
                          pr.s[i] + (pr.s[i+1] - pr.s[i]) * t, p.clearance);
                }
                pushA(pr.cl[i+1], pr.h[i+1], pr.s[i+1], e1);
            }
            const int m = static_cast<int>(ac.size());
            auto segHigh = [&](int k) { return (ae[k] + ae[k+1]) * 0.5 >= p.clearance; };
            int s = 0;
            while (s < m - 1) {
                const bool hi = segHigh(s);
                int e = s;
                while (e < m - 1 && segHigh(e) == hi) ++e;        // segments [s..e-1]
                Prof q;
                q.cl.assign(ac.begin() + s, ac.begin() + e + 1);
                q.h.assign(ah.begin() + s, ah.begin() + e + 1);
                q.s.assign(asx.begin() + s, asx.begin() + e + 1);
                q.hw = pr.hw; q.closed = false; q.klass = pr.klass;
                q.authored = hi;                                 // LOW -> ground weld; HIGH -> viaduct
                q.mouthFront = (s == 0)     ? pr.mouthFront : kNoCross;
                q.mouthBack  = (e == m - 1) ? pr.mouthBack  : kNoCross;
                q.group = 0;
                split.push_back(std::move(q));
                s = e;
            }
        }
        profs.swap(split);
    }
    // Assign weld groups: ground (non-authored) = 0; authored spines cluster by
    // affinity (share an endpoint, or overlap within clearance) so a continuous
    // elevated road stays one deck while a road it flies over lands in a
    // different group and never fuses. A stacked overlap forced into one group
    // (corkscrew / stacked interchange) is DETECTED and warned, not silently
    // shipped as a merged surface.
    int nGroups = 1;
    {
        const int P = static_cast<int>(profs.size());
        std::vector<int> uf(P);
        for (int i = 0; i < P; ++i) uf[i] = i;
        auto find = [&](int x) { while (uf[x] != x) { uf[x] = uf[uf[x]]; x = uf[x]; } return x; };
        auto uni  = [&](int a, int b) { uf[find(a)] = find(b); };
        auto key  = [](const Vec2& q) {
            return std::make_pair(static_cast<long long>(std::llround(q.x * 8)),
                                  static_cast<long long>(std::llround(q.y * 8))); };
        auto nearestOn = [&](const Vec2& pt, const Prof& B, double& outH) {
            double best = 1e30; outH = 0.0;
            for (std::size_t k = 0; k + 1 < B.cl.size(); ++k) {
                Vec2 a = B.cl[k], ab = B.cl[k+1] - a; double L2 = ab.lengthSquared();
                double t = L2 < 1e-12 ? 0.0 : std::max(0.0, std::min(1.0, dot(pt - a, ab) / L2));
                double d = (pt - (a + ab * t)).length();
                if (d < best) { best = d; outH = B.h[k] + (B.h[k+1] - B.h[k]) * t; }
            }
            return best;
        };
        std::vector<int> auth;
        for (int i = 0; i < P; ++i) if (profs[i].authored) auth.push_back(i);
        int conflicts = 0;
        for (std::size_t ai = 0; ai < auth.size(); ++ai)
            for (std::size_t aj = ai + 1; aj < auth.size(); ++aj) {
                const Prof& A = profs[auth[ai]]; const Prof& B = profs[auth[aj]];
                bool affinity = key(A.cl.front()) == key(B.cl.front()) ||
                                key(A.cl.front()) == key(B.cl.back())  ||
                                key(A.cl.back())  == key(B.cl.front()) ||
                                key(A.cl.back())  == key(B.cl.back());
                bool conflict = false;
                for (std::size_t k = 0; k < A.cl.size(); ++k) {
                    double bh; double d = nearestOn(A.cl[k], B, bh);
                    if (d < A.hw + B.hw) {
                        if (std::fabs(A.h[k] - bh) < p.clearance) affinity = true;
                        else conflict = true;
                    }
                }
                if (affinity) uni(auth[ai], auth[aj]);
                if (affinity && conflict) ++conflicts;           // stacked yet forced together
            }
        std::map<int, int> rootGroup;
        int next = 1;
        for (int i = 0; i < P; ++i) {
            if (!profs[i].authored) { profs[i].group = 0; continue; }
            int r = find(i);
            auto it = rootGroup.find(r);
            if (it == rootGroup.end()) { rootGroup[r] = next; profs[i].group = next++; }
            else profs[i].group = it->second;
        }
        nGroups = next;
        if (conflicts > 0)
            LOG_WARN << "[weld3d] " << conflicts << " stacked overlap(s) forced into one weld "
                        "group (corkscrew/stacked interchange) — may render merged; needs sub-split";
    }
    // Sample anywhere from the NEAREST spine: surface height (smoothed profile), and road-local UV
    // — mu = 2 + lateral/halfWidth in [1,3] (centre 2, curbs 1 & 3; the RoadMarkings shader paints
    // from it), mv = arc-length along the road (for dashed dividers). Junctions agree where spines
    // meet. Off the carriageway (no spine) mu stays 0 so no paint lands there.
    // Grade-separation grouping: each Prof carries a `group`; the weld runs its
    // union/deck/wall/sidewalk/markings stages once PER group so two roads that
    // overlap in plan at different heights (a deck over a street) never merge.
    // `curGroup` scopes the nearest-spine sample to the group being welded (a low
    // street can't set the deck's height at a crossing); -1 = all groups (piers).
    int curGroup = -1;
    auto sample = [&](double x, double z, double& oh, double& omu, double& omv, int& oSpine) {
        oh = p.topY; omu = 0.0; omv = 0.0; oSpine = -1;
        Vec2 q(x, z); double bestD2 = 1e30;
        for (std::size_t pi = 0; pi < profs.size(); ++pi) {
            const Prof& pr = profs[pi];
            if (curGroup >= 0 && pr.group != curGroup) continue;   // group-local weld
            for (std::size_t i = 0; i + 1 < pr.cl.size(); ++i) {
                const Vec2& a = pr.cl[i]; Vec2 ab = pr.cl[i + 1] - a;
                double L2 = ab.lengthSquared();
                double t = L2 < 1e-12 ? 0.0 : std::max(0.0, std::min(1.0, dot(q - a, ab) / L2));
                Vec2 d = q - (a + ab * t); double d2 = d.lengthSquared();
                if (d2 < bestD2) {
                    bestD2 = d2;
                    oh = pr.h[i] + (pr.h[i + 1] - pr.h[i]) * t;
                    double sgn = cross(ab, d) >= 0 ? 1.0 : -1.0;        // side of the centerline
                    double latN = std::sqrt(d2) / std::max(1e-6, pr.hw);
                    omu = 2.0 + sgn * std::min(1.0, latN);
                    omv = pr.s[i] + (pr.s[i + 1] - pr.s[i]) * t;         // arc-length (height sampler only)
                    oSpine = static_cast<int>(pi);
                }
            }
        }
    };
    auto heightOf = [&](double x, double z) -> double {
        double h, mu, mv; int si; sample(x, z, h, mu, mv, si); return h;
    };
    // Is the nearest carriageway an AUTHORED elevated deck (yAbs)? Its underside
    // is a fixed slab (open air / piers beneath), NOT skirted to the ground —
    // the at-grade skirt would wall a viaduct off to the terrain far below.
    auto authoredAt = [&](double x, double z) -> bool {
        double h, mu, mv; int si; sample(x, z, h, mu, mv, si);
        return si >= 0 && si < static_cast<int>(profs.size()) && profs[si].authored;
    };

    // Route each junction pad to the weld group of its nearest arm endpoint, so a
    // pad welds into the surface it belongs to (a ground pad never fuses a deck).
    std::vector<int> padGroup(p.padCenters.size(), 0);
    for (std::size_t pc = 0; pc < p.padCenters.size(); ++pc) {
        double best = 1e30;
        for (const Prof& pr : profs) {
            if (pr.cl.empty()) continue;
            for (const Vec2& end : {pr.cl.front(), pr.cl.back()}) {
                double d = (p.padCenters[pc] - end).lengthSquared();
                if (d < best) { best = d; padGroup[pc] = pr.group; }
            }
        }
    }

    // Weld each grade group independently: two roads overlapping in plan at
    // different heights land in different groups, so their ribbons never union
    // into one surface (a deck flies over a street; no curtain wall). At
    // nGroups==1 (no elevated deck, or increment-1 no-op) this is the old single
    // union, byte for byte. sample/heightOf/authoredAt above are scoped by
    // curGroup so the deck's height wins over its own group and the street's over
    // its own. Piers (after the loop) keep a GLOBAL view of all roads.
    RenderMesh mesh;                                  // accumulates across groups
    for (int g = 0; g < nGroups; ++g) {
      curGroup = g;
    // 2. The welded outline (same join engine as weldRibbons), rounded at junctions. A CLOSED
    //    spine (a roundabout ring) becomes an annulus: its outer rail welds with the spokes, its
    //    inner rail is punched as a central island hole.
    std::vector<Poly2> ribbons, forcedHoles;
    for (const Prof& pr : profs) {
        if (pr.group != g || pr.cl.size() < 2) continue;
        if (pr.closed) {
            Poly2 outer, inner; ringRibbon(pr.cl, pr.hw, outer, inner);
            if (outer.size() >= 3) ribbons.push_back(std::move(outer));
            if (inner.size() >= 3) forcedHoles.push_back(std::move(inner));
        } else {
            Poly2 r = ribbonOutline(pr.cl, pr.hw);
            if (r.size() >= 3) ribbons.push_back(std::move(r));
        }
    }
    // Junction pads: a disc per junction node joins the union, so the welded
    // outline (walls + sidewalk) wraps the pad and no wedge of ground shows
    // between off-square arm caps (see WeldSolidParams::padCenters).
    for (std::size_t pi = 0; pi < p.padCenters.size() && pi < p.padRadii.size(); ++pi) {
        if (pi < padGroup.size() && padGroup[pi] != g) continue;
        const double r = p.padRadii[pi];
        if (r <= 0.0) continue;
        Poly2 disc;
        const int segs = 20;
        for (int i = 0; i < segs; ++i) {
            double a = 2.0 * 3.14159265358979323846 * i / segs;
            disc.push_back(p.padCenters[pi] + Vec2(std::cos(a), std::sin(a)) * r);
        }
        ribbons.push_back(std::move(disc));
    }
    std::vector<Poly2> outers, holes;
    for (Poly2& L : polygonUnion(ribbons)) {
        Poly2 C = cleanPolygon(L, 0.01);              // snap-round + de-spike the welded boundary (1 cm)
        if (C.size() < 3) continue;
        Poly2 R = (p.cornerRadius > 0.0) ? roundPolygonCorners(C, p.cornerRadius, 5) : C;
        (signedArea(R) > 0 ? outers : holes).push_back(std::move(R));
    }
    for (Poly2& h : forcedHoles) holes.push_back(std::move(h));   // roundabout islands

    auto P3 = [&](const Vec2& v, double dh) { return Vec3(v.x, heightOf(v.x, v.y) + dh, v.y); };
    // A vertical wall along a boundary loop: the outward normal is the RIGHT normal of each
    // directed edge (away from the solid for a CCW outer; into the shaft for a CW hole loop).
    auto wall = [&](const Poly2& loop) {
        const int m = static_cast<int>(loop.size());
        for (int i = 0; i < m; ++i) {
            const Vec2& a = loop[i]; const Vec2& b = loop[(i + 1) % m];
            Vec2 e = b - a; if (e.length() < 1e-9) continue;
            Vec2 rn = normalize(Vec2(e.y, -e.x));               // right normal in XZ
            Vec3 nrm(rn.x, 0, rn.y);
            // Skirt to the GROUND, not a fixed thickness: where the deck rides
            // high over a dip the fixed skirt ended mid-air and the terrain
            // showed underneath (device feedback). Drop each edge to the lower
            // of the sampled ground under its two ends, minus a margin, but
            // never less than the old thickness.
            double drop = p.thickness;
            // An AUTHORED elevated deck rides on piers with open air beneath, so
            // its edge is a fixed-thickness slab — never skirted down to the
            // terrain (which would curtain-wall a viaduct to the ground below).
            const bool onDeck = authoredAt(a.x, a.y) || authoredAt(b.x, b.y);
            if (p.heightAt && !onDeck) {
                const double gA = heightOf(a.x, a.y) - p.heightAt(a.x, a.y);
                const double gB = heightOf(b.x, b.y) - p.heightAt(b.x, b.y);
                drop = std::max(drop, std::max(gA, gB) + 0.5);
            }
            Vec3 tA = P3(a, 0), tB = P3(b, 0), bA = P3(a, -drop), bB = P3(b, -drop);
            MeshBuilder::emitTri(mesh, tA, tB, bB, nrm, p.sideColor);
            MeshBuilder::emitTri(mesh, tA, bB, bA, nrm, p.sideColor);
        }
    };

    // Walls from the welded outline: an outer skirt around each exterior loop, a shaft wall around
    // each hole (open block interiors, roundabout islands). The deck itself is filled by plain
    // per-spine strips below — robust for ANY number of holes (no hole-bridging + ear-clip to stall
    // or over-fill a block, which is what broke the welded-polygon triangulation on a grid).
    for (const Poly2& outer : outers) wall(outer);
    for (const Poly2& hole : holes) wall(hole);

    // Raised sidewalk band along each welded boundary loop: the carriageway edge wears a curb lip,
    // a concrete slab steps out by sidewalkWidth, and an outer face drops back to the road slab.
    // The band rides the loop's RIGHT normal (outward from the carriageway, same sense as wall())
    // mitered per vertex, so it wraps junction corners as one piece instead of crossing the road.
    if (p.sidewalkWidth > 0.0) {
        auto rnorm = [](const Vec2& e) {
            Vec2 n(e.y, -e.x); double l = n.length();
            return l < 1e-9 ? Vec2(0, 0) : n * (1.0 / l);
        };
        auto sidewalk = [&](const Poly2& loop) {
            const int m = static_cast<int>(loop.size());
            if (m < 3) return;
            // Per-vertex OUTER offset: one mitered point on gentle corners; on
            // sharp ones the clamped miter used to land SHORT of the true
            // intersection, folding adjacent band quads over each other
            // (device: "sidewalks don't miter together at the corners — they
            // overlap and z-fight"). Sharp corners now BEVEL instead: two
            // per-edge offset points and a corner fan, so the band turns every
            // junction corner as one continuous non-overlapping piece.
            struct Corner { Vec2 prevPt, nextPt; bool bevel; };
            std::vector<Corner> oc(m);
            for (int i = 0; i < m; ++i) {
                Vec2 e0 = loop[i] - loop[(i + m - 1) % m];
                Vec2 e1 = loop[(i + 1) % m] - loop[i];
                Vec2 n0 = rnorm(e0), n1 = rnorm(e1);
                Vec2 bis = n0 + n1; double bl = bis.length();
                Vec2 mm = bl < 1e-9 ? n1 : bis * (1.0 / bl);
                double cosH = dot(mm, n1);
                // The band rides the RIGHT of travel, so the turn direction
                // decides the corner's kind: a LEFT turn OPENS the band (fan /
                // bevel closes the gap), a RIGHT turn PINCHES it (the two band
                // edges meet at the offset lines' true intersection — a
                // junction crotch, a block frame's inner corner). Beveling or
                // clamping a pinch folds the segments over each other (device:
                // "parts of the sidewalk aren't being welded together").
                const bool opens = cross(e0, e1) > 0.0;
                if (!opens) {
                    Vec2 q = loop[i] + mm * (p.sidewalkWidth / std::max(0.2, cosH));
                    oc[i] = {q, q, false};
                } else if (cosH >= 0.5) {
                    Vec2 q = loop[i] + mm * (p.sidewalkWidth / cosH);
                    oc[i] = {q, q, false};
                } else {
                    oc[i] = {loop[i] + n0 * p.sidewalkWidth,
                             loop[i] + n1 * p.sidewalkWidth, true};
                }
            }
            const double ch = p.curbHeight;
            // Arc length along the loop, baked (negated) into the slab-top UV so
            // the shader scores joints PERPENDICULAR to the curb, following its
            // curvature (device: world-aligned plaza slabs looked wrong on a
            // sidewalk). Negative u keeps the band inside the shader's existing
            // "mu < 0.98 = sidewalk" gate; sArc[m] closes the wrap-around edge.
            std::vector<double> sArc(m + 1, 0.0);
            for (int i = 0; i < m; ++i)
                sArc[i + 1] = sArc[i] + (loop[(i + 1) % m] - loop[i]).length();
            for (int i = 0; i < m; ++i) {
                int j = (i + 1) % m;
                const Vec2& a = loop[i]; const Vec2& b = loop[j];
                if ((b - a).length() < 1e-9) continue;
                Vec2 ao = oc[i].nextPt, bo = oc[j].prevPt;
                Vec3 eo3(rnorm(b - a).x, 0, rnorm(b - a).y);  // outward (per edge)
                Vec3 aT = P3(a, ch), bT = P3(b, ch), aR = P3(a, 0), bR = P3(b, 0);
                Vec3 aoT = P3(ao, ch), boT = P3(bo, ch),
                     aoB = P3(ao, -p.thickness), boB = P3(bo, -p.thickness);
                MeshBuilder::emitQuad(mesh, aR, bR, bT, aT, eo3 * -1.0, p.curbColor);     // curb lip (road-facing)
                const float ua = static_cast<float>(-sArc[i]);
                const float ub = static_cast<float>(-sArc[i + 1]);
                MeshBuilder::emitTriUV(mesh, aT, bT, boT, Vec3(0, 1, 0), p.sidewalkColor,
                                       ua, 0.0f, ub, 0.0f, ub, 1.0f);      // slab top
                MeshBuilder::emitTriUV(mesh, aT, boT, aoT, Vec3(0, 1, 0), p.sidewalkColor,
                                       ua, 0.0f, ub, 1.0f, ua, 1.0f);
                MeshBuilder::emitQuad(mesh, aoT, boT, boB, aoB, eo3, p.curbColor);        // outer face
                // Bevel corner at j: the fan tri closing the band around the
                // corner, plus its outer face — no fold, no overlap.
                if (oc[j].bevel) {
                    Vec3 jT = P3(b, ch);
                    Vec3 p0T = P3(oc[j].prevPt, ch), p1T = P3(oc[j].nextPt, ch);
                    MeshBuilder::emitTriUV(mesh, jT, p0T, p1T, Vec3(0, 1, 0),
                                           p.sidewalkColor, ub, 0.0f, ub, 1.0f,
                                           ub, 1.0f);
                    Vec2 bev = oc[j].nextPt - oc[j].prevPt;
                    if (bev.length() > 1e-9) {
                        Vec2 bn = rnorm(bev);
                        Vec3 p0B = P3(oc[j].prevPt, -p.thickness),
                             p1B = P3(oc[j].nextPt, -p.thickness);
                        MeshBuilder::emitQuad(mesh, p0T, p1T, p1B, p0B,
                                              Vec3(bn.x, 0, bn.y), p.curbColor);
                    }
                }
            }
        };
        for (const Poly2& outer : outers) sidewalk(outer);
        for (const Poly2& hole : holes) sidewalk(hole);
    }

    // The road SURFACE as ONE welded sheet: triangulate the union interior (each
    // exterior loop minus the block/island holes) with a robust hole-aware ear
    // clip. This replaces the old overlapping per-spine strips, whose exclusive-
    // ownership skip deferred coverage to corridors that were never surfaced
    // (weld gaps between junctions) and whose sub-mm per-spine layer biases still
    // z-fought where sharp arms overlapped ("two ribbons z-fighting"). One sheet:
    // no gaps, no overlap, no z-fight. Every deck vertex takes its height from the
    // same heightOf() the walls/sidewalk use — the boundary is shared exactly, so
    // deck, skirt and curb weld watertight. Markings ride a separate overlay pass
    // below (they need per-road UV, which the flat sheet does not carry).
    for (const Poly2& outer : outers) {
        std::vector<Poly2> deckHoles;
        for (const Poly2& h : holes)
            if (h.size() >= 3 && pointInPolygon(outer, centroid(h)))
                deckHoles.push_back(h);
        for (const std::array<Vec2, 3>& t : triangulateWithHoles(outer, deckHoles)) {
            Vec3 a = P3(t[0], 0.0), b = P3(t[1], 0.0), c = P3(t[2], 0.0);
            MeshBuilder::emitTri(mesh, a, b, c, Vec3(0, 1, 0), p.topColor);        // deck up
            Vec3 aB = P3(t[0], -p.thickness), bB = P3(t[1], -p.thickness),
                 cB = P3(t[2], -p.thickness);
            MeshBuilder::emitTri(mesh, aB, bB, cB, Vec3(0, -1, 0), p.bottomColor);  // underside
        }
    }

    // Lane MARKINGS as a thin overlay just above the welded deck, laid per-spine so
    // each quad carries road-local UV (left rail u=1, right u=3, v = arc-length) for
    // crisp double-yellow, dashed dividers and edges. A quad straddling ANOTHER
    // road's corridor is a junction and stays unpainted.
    const double markLift = 0.03, stripStep = 3.0;
    auto distToSpine = [&](const Vec2& q, const Prof& pr) {
        double best = 1e30;
        for (std::size_t i = 0; i + 1 < pr.cl.size(); ++i) {
            const Vec2& a = pr.cl[i]; Vec2 ab = pr.cl[i + 1] - a; double L2 = ab.lengthSquared();
            double t = L2 < 1e-12 ? 0.0 : std::max(0.0, std::min(1.0, dot(q - a, ab) / L2));
            best = std::min(best, (q - (a + ab * t)).length());
        }
        return best;
    };
    for (std::size_t pi = 0; pi < profs.size(); ++pi) {
        const Prof& pr = profs[pi];
        if (pr.group != g) continue;                 // paint only this group's roads
        const double hw = pr.hw;
        const int n = static_cast<int>(pr.cl.size());
        if (n < 2) continue;
        // Per-vertex MITERED rail offset (left normal * miter length). Two strips meeting at a
        // vertex share this one offset, so their rail corners coincide and the deck stays gap-free
        // on the outside of curves. (A per-SEGMENT normal places each strip's corner on its own
        // tangent, leaving a comb of miter notches along every curve — the terrain showing through.)
        // Closed rings wrap at the seam; open ends square off with the lone adjacent normal.
        std::vector<Vec2> off(n);
        auto segDir = [&](int a, int b) {
            Vec2 d = pr.cl[b] - pr.cl[a]; double l = d.length();
            return l < 1e-9 ? Vec2(0, 0) : d * (1.0 / l);
        };
        for (int i = 0; i < n; ++i) {
            Vec2 dp = (i > 0) ? segDir(i - 1, i) : Vec2(0, 0);        // incoming tangent
            Vec2 dn = (i + 1 < n) ? segDir(i, i + 1) : Vec2(0, 0);    // outgoing tangent
            if (pr.closed) {                                          // ring: cl.back()==cl.front()
                if (i == 0) dp = segDir(n - 2, n - 1);
                if (i == n - 1) dn = segDir(0, 1);
            }
            bool hasP = dp.lengthSquared() > 1e-12, hasN = dn.lengthSquared() > 1e-12;
            Vec2 n0 = perp(dp), n1 = perp(dn);                       // left normals
            Vec2 bis = (hasP && hasN) ? (n0 + n1) : (hasN ? n1 : n0);
            double bl = bis.length();
            Vec2 mm = bl < 1e-9 ? (hasN ? n1 : n0) : bis * (1.0 / bl);
            double cosHalf = std::max(0.25, dot(mm, hasN ? n1 : n0)); // clamp miter <= 4*hw at sharp turns
            off[i] = mm * (hw / cosHalf);
        }
        for (int i = 0; i + 1 < n; ++i) {
            Vec2 A = pr.cl[i], B = pr.cl[i + 1]; Vec2 ab = B - A;
            double segLen = ab.length(); if (segLen < 1e-6) continue;
            int steps = std::max(1, static_cast<int>(std::ceil(segLen / stripStep)));
            for (int k = 0; k < steps; ++k) {
                double t0 = static_cast<double>(k) / steps, t1 = static_cast<double>(k + 1) / steps;
                Vec2 c0 = A + ab * t0, c1 = A + ab * t1;
                Vec2 o0 = off[i] + (off[i + 1] - off[i]) * t0;       // mitered rail, interpolated
                Vec2 o1 = off[i] + (off[i + 1] - off[i]) * t1;
                double s0 = pr.s[i] + (pr.s[i + 1] - pr.s[i]) * t0;
                double s1 = pr.s[i] + (pr.s[i + 1] - pr.s[i]) * t1;
                double h0 = pr.h[i] + (pr.h[i + 1] - pr.h[i]) * t0;
                double h1 = pr.h[i] + (pr.h[i + 1] - pr.h[i]) * t1;
                auto P = [&](const Vec2& c, const Vec2& o, double h, int side) {
                    return Vec3(c.x + side * o.x, h, c.y + side * o.y);
                };
                // Markings only — the welded deck above already surfaces the
                // asphalt. Paint the overlay unless this quad straddles another
                // road (a junction), which stays plain.
                Vec2 mid = (c0 + c1) * 0.5;
                bool junction = false;
                for (std::size_t pj = 0; pj < profs.size() && !junction; ++pj)
                    if (pj != pi && profs[pj].group == g &&
                        distToSpine(mid, profs[pj]) < profs[pj].hw + 0.5) junction = true;
                if (junction) continue;
                Vec3 mL0 = P(c0, o0, h0 + markLift, +1), mR0 = P(c0, o0, h0 + markLift, -1),
                     mL1 = P(c1, o1, h1 + markLift, +1), mR1 = P(c1, o1, h1 + markLift, -1);
                // The v coordinate (mv) carries CROSSWALK placement (ADR-0062):
                // metres PAST the junction mouth, so the RoadMarkings shader stripes
                // a set-back band on each approach (not in the intersection). = the
                // distance to the nearest chain end minus the road half-width (the
                // mouth). Off, or on a closed ring, a large sentinel = no band.
                double chainLen = pr.s.back();
                auto cwV = [&](double s) -> float {
                    if (!p.crosswalks || pr.closed) return 1e4f;
                    // Skip zebras by CLASS (freeways/ramps have no pedestrians),
                    // not by a width threshold — the old hw*2 >= 18 also wrongly
                    // suppressed crosswalks on a legitimately wide arterial, which
                    // the planned street-widening would trip.
                    if (pr.klass == RoadClass::Freeway || pr.klass == RoadClass::Ramp)
                        return 1e4f;
                    // Set back by the true junction MOUTH (widest-arm pad radius)
                    // at the NEARER end, not this arm's own hw — so the band lands
                    // just outside the pad and never juts into the intersection.
                    const double distF = s, distB = chainLen - s;
                    const double mouth = (distF < distB) ? pr.mouthFront : pr.mouthBack;
                    return static_cast<float>(std::min(distF, distB) - mouth);
                };
                float v0 = cwV(s0), v1 = cwV(s1);
                MeshBuilder::emitTriUV(mesh, mL0, mR0, mR1, Vec3(0, 1, 0), p.topColor,
                                       1.0f, v0, 3.0f, v0, 3.0f, v1);
                MeshBuilder::emitTriUV(mesh, mL0, mR1, mL1, Vec3(0, 1, 0), p.topColor,
                                       1.0f, v0, 3.0f, v1, 1.0f, v1);
                // Multilane roads (device: "do we still have multilane roads?"):
                // DASHED white dividers between same-direction lanes. One thin
                // strip per internal lane boundary (the centre boundary is the
                // double yellow), tagged u = 4 with v carrying RAW arc-length so
                // the shader phases 3 m dashes. Suppressed through the crosswalk
                // zone like the rest of the approach paint.
                // Lane count from the road's CLASS (the single lanesForClass
                // source), not a width guess with a hardcoded 3.5 — so the
                // dashed dividers match the design lane count and the nav lanes.
                const int lanes = lanesForClass(pr.klass);
                if (lanes >= 4) {
                    const double laneW = 2.0 * hw / lanes;
                    const bool nearCw = p.crosswalks && !pr.closed &&
                                        std::min(v0, v1) < 4.0f;
                    for (int lk = 1; lk < lanes && !nearCw; ++lk) {
                        if (lk * 2 == lanes) continue;          // centre: double yellow
                        const double lat = -hw + laneW * lk;    // boundary offset (m)
                        const double halfStripe = 0.14;
                        // A hair ABOVE the full-width marking quad (device:
                        // "markings z-fight"): the divider strips are separate
                        // geometry — coplanar with the paint quad they overlay —
                        // until the road gains a second UV channel and all the
                        // paint moves into the surface shader.
                        auto D = [&](const Vec2& c, const Vec2& o, double h, double off) {
                            const double t = off / hw;          // rail-relative offset
                            return Vec3(c.x + o.x * t, h + markLift + 0.012, c.y + o.y * t);
                        };
                        Vec3 sL0 = D(c0, o0, h0, lat - halfStripe),
                             sR0 = D(c0, o0, h0, lat + halfStripe),
                             sL1 = D(c1, o1, h1, lat - halfStripe),
                             sR1 = D(c1, o1, h1, lat + halfStripe);
                        MeshBuilder::emitTriUV(mesh, sL0, sR0, sR1, Vec3(0, 1, 0),
                                               p.topColor, 4.0f, (float)s0, 4.0f,
                                               (float)s0, 4.0f, (float)s1);
                        MeshBuilder::emitTriUV(mesh, sL0, sR1, sL1, Vec3(0, 1, 0),
                                               p.topColor, 4.0f, (float)s0, 4.0f,
                                               (float)s1, 4.0f, (float)s1);
                    }
                }
            }
        }
    }
    // (Junction pad interiors are part of the welded union boundary, so the
    // triangulated deck above already surfaces them as one sheet with the arms —
    // no separate pad fan is needed, and none is emitted to overlap it.)
    }   // end per-group weld
    curGroup = -1;                                    // piers see all groups

    // PIERS under authored elevated decks (welder-goes-3D): a viaduct rides on
    // columns, not a curtain wall (the slab underside is aloft; something must
    // hold it up). Drop a column every ~18 m along each authored deck, from the
    // deck underside to the ground — but NEVER onto a street passing beneath: a
    // candidate within (pierHalf + street halfWidth + shy) of any at-grade
    // carriageway is skipped, the obstruction-clearance rule the audit flagged
    // ("ramp piers drop a column on the road below with zero road-avoidance").
    {
        // Ground reference for the pier feet: the terrain if we have it, else a
        // flat plane at topY (an overpass on a box/flat level still gets piers).
        std::function<double(double, double)> pierGround = p.heightAt;
        if (!pierGround) { const double y0 = p.topY; pierGround = [y0](double, double) { return y0; }; }
        auto merge = [&](const RenderMesh& src) {
            uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
            mesh.vertices.insert(mesh.vertices.end(), src.vertices.begin(), src.vertices.end());
            for (uint32_t i : src.indices) mesh.indices.push_back(base + i);
        };
        const double pierSpacing = 18.0, pierW = 1.8, shy = 0.6;
        const Vec3 pierColor(0.34, 0.34, 0.36);
        // Nearest distance from q to a non-authored (street) carriageway edge; a
        // pier must clear it. Returns +inf if no street is near.
        auto streetGap = [&](const Vec2& q) {
            double best = 1e30;
            for (const Prof& pj : profs) {
                if (pj.authored) continue;                 // only avoid at-grade roads
                for (std::size_t i = 0; i + 1 < pj.cl.size(); ++i) {
                    const Vec2& a = pj.cl[i]; Vec2 ab = pj.cl[i + 1] - a;
                    double L2 = ab.lengthSquared();
                    double t = L2 < 1e-12 ? 0.0
                                          : std::max(0.0, std::min(1.0, dot(q - a, ab) / L2));
                    double d = (q - (a + ab * t)).length() - pj.hw;   // to carriageway edge
                    best = std::min(best, d);
                }
            }
            return best;
        };
        for (const Prof& pr : profs) {
            if (!pr.authored || pr.cl.size() < 2) continue;
            // Densify the centreline at pierSpacing; carry the deck Y alongside.
            std::vector<Vec2> dense; std::vector<double> denseY;
            double arc = 0.0, target = pierSpacing;        // first pier a span in
            for (std::size_t i = 0; i + 1 < pr.cl.size(); ++i) {
                Vec2 seg = pr.cl[i + 1] - pr.cl[i];
                double segLen = seg.length();
                while (segLen > 1e-9 && target <= arc + segLen) {
                    double t = (target - arc) / segLen;
                    dense.push_back(pr.cl[i] + seg * t);
                    denseY.push_back(pr.h[i] + (pr.h[i + 1] - pr.h[i]) * t);
                    target += pierSpacing;
                }
                arc += segLen;
            }
            if (dense.size() < 2) continue;
            std::vector<int> at;
            for (std::size_t k = 0; k < dense.size(); ++k)
                if (streetGap(dense[k]) > pierW * 0.5 + shy) at.push_back(static_cast<int>(k));
            if (at.empty()) continue;
            merge(bridgePiers(dense, denseY, at, pierW, pierW, p.thickness,
                              pierColor, pierGround));
        }
    }
    return mesh;
}

RenderMesh unionRibbons(const std::vector<UnionSpine>& spines, double cell,
                        double y, const Vec3& color) {
    RenderMesh mesh;
    if (spines.empty() || cell <= 0) return mesh;

    // Grid covering every spine, padded by the widest half-width so the round caps fit.
    double minX = 1e30, minZ = 1e30, maxX = -1e30, maxZ = -1e30, maxHW = 0;
    for (const UnionSpine& s : spines) {
        maxHW = std::max(maxHW, s.halfWidth);
        for (const Vec2& p : s.points) {
            minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
            minZ = std::min(minZ, p.y); maxZ = std::max(maxZ, p.y);
        }
    }
    if (minX > maxX) return mesh;
    double pad = maxHW + cell;
    minX -= pad; minZ -= pad; maxX += pad; maxZ += pad;
    const int nx = static_cast<int>(std::ceil((maxX - minX) / cell)) + 1;
    const int nz = static_cast<int>(std::ceil((maxZ - minZ) / cell)) + 1;
    if (nx < 2 || nz < 2) return mesh;
    auto X = [&](int i) { return minX + i * cell; };
    auto Z = [&](int j) { return minZ + j * cell; };
    auto idx = [&](int i, int j) { return j * nx + i; };

    // Signed distance to the union of stroked spines at each grid node:
    // min over spines of (distance-to-polyline - half-width). < 0 is inside.
    std::vector<double> sdf(static_cast<std::size_t>(nx) * nz);
    for (int j = 0; j < nz; ++j)
        for (int i = 0; i < nx; ++i) {
            Vec2 p(X(i), Z(j));
            double best = 1e30;
            for (const UnionSpine& s : spines) {
                int n = static_cast<int>(s.points.size());
                if (n < 2) continue;
                int segs = s.closed ? n : n - 1;
                double d2 = 1e30;
                for (int k = 0; k < segs; ++k)
                    d2 = std::min(d2, segDist2(p, s.points[k], s.points[(k + 1) % n]));
                best = std::min(best, std::sqrt(d2) - s.halfWidth);
            }
            sdf[idx(i, j)] = best;
        }

    // Marching-squares FILLED cells: per cell, walk its 4 edges keeping inside
    // corners and the interpolated zero-crossings, then fan the convex inside polygon.
    auto P3 = [&](const Vec2& v) { return Vec3(v.x, y, v.y); };
    auto emit = [&](const Vec2& a, const Vec2& b, const Vec2& c) {
        double area2 = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
        if (std::fabs(area2) < 1e-9) return;        // drop degenerate slivers
        MeshBuilder::emitTri(mesh, P3(a), P3(b), P3(c), Vec3(0, 1, 0), color);
    };
    for (int j = 0; j < nz - 1; ++j)
        for (int i = 0; i < nx - 1; ++i) {
            double s[4] = { sdf[idx(i, j)], sdf[idx(i+1, j)],
                            sdf[idx(i+1, j+1)], sdf[idx(i, j+1)] };
            Vec2 c[4] = { Vec2(X(i), Z(j)), Vec2(X(i+1), Z(j)),
                          Vec2(X(i+1), Z(j+1)), Vec2(X(i), Z(j+1)) };
            std::vector<Vec2> poly;
            for (int e = 0; e < 4; ++e) {
                int e2 = (e + 1) % 4;
                if (s[e] < 0) poly.push_back(c[e]);
                if ((s[e] < 0) != (s[e2] < 0)) {
                    double t = s[e] / (s[e] - s[e2]);           // zero crossing on the edge
                    poly.push_back(c[e] + (c[e2] - c[e]) * t);
                }
            }
            for (std::size_t k = 1; k + 1 < poly.size(); ++k)
                emit(poly[0], poly[k], poly[k + 1]);
        }
    return mesh;
}

RenderMesh unionRoadbed(const std::vector<UnionSpine>& spines, const RoadbedParams& p) {
    RenderMesh mesh;
    if (spines.empty() || p.cell <= 0) return mesh;
    const double cell = p.cell, sw = p.sidewalkWidth;

    double minX = 1e30, minZ = 1e30, maxX = -1e30, maxZ = -1e30, maxHW = 0;
    for (const UnionSpine& s : spines) {
        maxHW = std::max(maxHW, s.halfWidth);
        for (const Vec2& q : s.points) {
            minX = std::min(minX, q.x); maxX = std::max(maxX, q.x);
            minZ = std::min(minZ, q.y); maxZ = std::max(maxZ, q.y);
        }
    }
    if (minX > maxX) return mesh;
    double pad = maxHW + sw + cell;
    minX -= pad; minZ -= pad; maxX += pad; maxZ += pad;
    const int nx = static_cast<int>(std::ceil((maxX - minX) / cell)) + 1;
    const int nz = static_cast<int>(std::ceil((maxZ - minZ) / cell)) + 1;
    if (nx < 2 || nz < 2) return mesh;
    auto X = [&](int i) { return minX + i * cell; };
    auto Z = [&](int j) { return minZ + j * cell; };
    auto gi = [&](int i, int j) { return j * nx + i; };

    // SDF via the Jump Flood Algorithm (JFA): seed grid cells from the centrelines, then flood the
    // "nearest seed" outward in HALVING steps — N/2, N/4, ... 1 — so every cell learns its nearest
    // centreline in O(cells * logN) passes instead of the O(cells * segments) brute force. sdf =
    // distance to that nearest centreline point minus its half-width. (JFA also hands back the nearest
    // point itself, which is what a later pass uses to bake road-local UVs.)
    // A seed carries its centreline point + half-width AND arc-length + tangent, so the flooded result
    // gives every cell road-local UVs (U = lateral/hw, V = arc-length down the road) that the
    // RoadMarkings shader paints from — the "UV from the graph" fix: JFA's nearest point IS the UV anchor.
    struct Seed { float px, pz, hw, u, tx, tz; int has; };
    std::vector<Seed> seed(static_cast<std::size_t>(nx) * nz, Seed{0, 0, 0, 0, 0, 0, 0});
    auto plant = [&](int i, int j, const Seed& cand) {
        if (i < 0 || i >= nx || j < 0 || j >= nz) return;
        Seed& s = seed[gi(i, j)];
        float cx = static_cast<float>(X(i)), cz = static_cast<float>(Z(j));
        float dn = (cand.px - cx) * (cand.px - cx) + (cand.pz - cz) * (cand.pz - cz);
        if (!s.has || dn < (s.px - cx) * (s.px - cx) + (s.pz - cz) * (s.pz - cz)) s = cand;
    };
    for (const UnionSpine& s : spines) {
        int n = static_cast<int>(s.points.size()); if (n < 2) continue;
        int segs = s.closed ? n : n - 1;
        double arc = 0.0;
        for (int k = 0; k < segs; ++k) {
            Vec2 a = s.points[k], b = s.points[(k + 1) % n];
            Vec2 d = b - a; double len = d.length();
            Vec2 tan = len > 1e-9 ? d * (1.0 / len) : Vec2(1, 0);
            int steps = std::max(1, static_cast<int>(len / (cell * 0.35)));   // dense -> smoother field
            for (int t = 0; t <= steps; ++t) {
                double f = static_cast<double>(t) / steps;
                Vec2 q = a + d * f;
                plant(static_cast<int>(std::lround((q.x - minX) / cell)),
                      static_cast<int>(std::lround((q.y - minZ) / cell)),
                      Seed{static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(s.halfWidth),
                           static_cast<float>(arc + len * f), static_cast<float>(tan.x), static_cast<float>(tan.y), 1});
            }
            arc += len;
        }
    }
    std::vector<Seed> back(seed.size());
    // Only cells within (maxHW + sidewalk) of a road are ever used, so the flood only has to reach
    // that far — cap the JFA start step to the band width instead of the whole grid (far cells stay
    // unseeded = "outside", which the marcher skips). Fewer passes, same band result.
    int bandCells = static_cast<int>((maxHW + sw + cell) / cell) + 2;
    int startStep = 1; while (startStep < bandCells) startStep *= 2;
    startStep = std::min(startStep, 1 << static_cast<int>(std::floor(std::log2(std::max(2, std::max(nx, nz) - 1)))));
    for (int step = startStep; step >= 1; step /= 2) {
        back = seed;
        for (int j = 0; j < nz; ++j)
            for (int i = 0; i < nx; ++i) {
                float cx = static_cast<float>(X(i)), cz = static_cast<float>(Z(j));
                Seed best = seed[gi(i, j)];
                float bd = best.has ? (best.px - cx) * (best.px - cx) + (best.pz - cz) * (best.pz - cz) : 1e30f;
                for (int dj = -1; dj <= 1; ++dj)
                    for (int di = -1; di <= 1; ++di) {
                        if (!di && !dj) continue;
                        int ni = i + di * step, nj = j + dj * step;
                        if (ni < 0 || ni >= nx || nj < 0 || nj >= nz) continue;
                        const Seed& s = seed[gi(ni, nj)];
                        if (!s.has) continue;
                        float d = (s.px - cx) * (s.px - cx) + (s.pz - cz) * (s.pz - cz);
                        if (d < bd) { bd = d; best = s; }
                    }
                back[gi(i, j)] = best;
            }
        seed.swap(back);
    }
    // Rasterise the junction-interior suppress disks into a mask: markings are off inside them.
    std::vector<char> noMark(static_cast<std::size_t>(nx) * nz, 0);
    for (std::size_t d = 0; d < p.noPaintCenters.size(); ++d) {
        Vec2 c = p.noPaintCenters[d];
        double r = d < p.noPaintRadii.size() ? p.noPaintRadii[d] : 0.0;
        if (r <= 0) continue;
        int i0 = std::max(0, static_cast<int>((c.x - r - minX) / cell));
        int i1 = std::min(nx - 1, static_cast<int>((c.x + r - minX) / cell) + 1);
        int j0 = std::max(0, static_cast<int>((c.y - r - minZ) / cell));
        int j1 = std::min(nz - 1, static_cast<int>((c.y + r - minZ) / cell) + 1);
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) {
                double dx = X(i) - c.x, dz = Z(j) - c.y;
                if (dx * dx + dz * dz <= r * r) noMark[gi(i, j)] = 1;
            }
    }
    std::vector<double> sdf(static_cast<std::size_t>(nx) * nz);
    std::vector<double> gU(static_cast<std::size_t>(nx) * nz, 0.0);   // road-local UV per grid node
    std::vector<double> gV(static_cast<std::size_t>(nx) * nz, 0.0);
    for (int j = 0; j < nz; ++j)
        for (int i = 0; i < nx; ++i) {
            const Seed& s = seed[gi(i, j)];
            if (!s.has) { sdf[gi(i, j)] = 1e9; continue; }
            double ox = X(i) - s.px, oz = Z(j) - s.pz;               // node - nearest centreline point
            sdf[gi(i, j)] = std::sqrt(ox * ox + oz * oz) - s.hw;
            if (noMark[gi(i, j)]) continue;                          // junction interior: no markings (gU=0)
            double along   = ox * s.tx + oz * s.tz;                  // down the road
            double lateral = static_cast<double>(s.tx) * oz - static_cast<double>(s.tz) * ox;  // across (signed)
            gU[gi(i, j)] = 2.0 + lateral / std::max(0.5, static_cast<double>(s.hw));   // 1 left, 2 centre, 3 right
            gV[gi(i, j)] = static_cast<double>(s.u) + along;         // arc-length, for the dash pattern
        }

    // Light smoothing of the field near the contours tames the marching-squares wobble along the
    // medial axis where two roads meet (the acute-wedge "squiggle"); a straight edge is a linear field,
    // so the average leaves it put.
    for (int pass = 0; pass < 3; ++pass) {
        std::vector<double> sm = sdf;
        for (int j = 1; j < nz - 1; ++j)
            for (int i = 1; i < nx - 1; ++i) {
                double a = sdf[gi(i, j)];
                if (a > sw + cell) continue;                         // only near the contours we mesh
                sm[gi(i, j)] = (4.0 * a + sdf[gi(i-1,j)] + sdf[gi(i+1,j)] + sdf[gi(i,j-1)] + sdf[gi(i,j+1)]) / 8.0;
            }
        sdf.swap(sm);
    }

    auto drape = [&](const Vec2& v) { return (p.heightAt ? p.heightAt(v.x, v.y) : 0.0) + p.lift; };
    // A deterministic per-cell value jitter so the flat bands read as a textured surface.
    auto grainAt = [&](int i, int j) {
        uint32_t h = static_cast<uint32_t>(i * 73856093) ^ static_cast<uint32_t>(j * 19349663);
        h = (h ^ (h >> 13)) * 1274126177u;
        return (1.0 - p.grain) + p.grain * ((h >> 8) & 0xffff) / 65535.0 * 2.0;
    };
    struct SVtx { Vec2 p; double s; double u; double v; };
    auto clip = [](const std::vector<SVtx>& poly, double thr, bool below) {
        std::vector<SVtx> out; int n = static_cast<int>(poly.size());
        for (int i = 0; i < n; ++i) {
            const SVtx& A = poly[i]; const SVtx& B = poly[(i + 1) % n];
            bool ain = below ? (A.s < thr) : (A.s >= thr);
            bool bin = below ? (B.s < thr) : (B.s >= thr);
            if (ain) out.push_back(A);
            if (ain != bin) {
                double t = (thr - A.s) / (B.s - A.s);
                out.push_back({A.p + (B.p - A.p) * t, thr, A.u + (B.u - A.u) * t, A.v + (B.v - A.v) * t});
            }
        }
        return out;
    };

    for (int j = 0; j < nz - 1; ++j)
        for (int i = 0; i < nx - 1; ++i) {
            double s00 = sdf[gi(i,j)], s10 = sdf[gi(i+1,j)], s11 = sdf[gi(i+1,j+1)], s01 = sdf[gi(i,j+1)];
            if (s00 >= sw && s10 >= sw && s11 >= sw && s01 >= sw) continue;     // wholly outside
            SVtx corners[4] = {
                {Vec2(X(i),   Z(j)),   s00, gU[gi(i,j)],     gV[gi(i,j)]},
                {Vec2(X(i+1), Z(j)),   s10, gU[gi(i+1,j)],   gV[gi(i+1,j)]},
                {Vec2(X(i+1), Z(j+1)), s11, gU[gi(i+1,j+1)], gV[gi(i+1,j+1)]},
                {Vec2(X(i),   Z(j+1)), s01, gU[gi(i,j+1)],   gV[gi(i,j+1)]} };
            std::vector<SVtx> cellPoly(corners, corners + 4);
            double g = grainAt(i, j);

            auto fan = [&](const std::vector<SVtx>& poly, double extraH, Vec3 col, bool uv) {
                col = col * g;
                for (std::size_t k = 1; k + 1 < poly.size(); ++k) {
                    const SVtx& A = poly[0]; const SVtx& B = poly[k]; const SVtx& C = poly[k + 1];
                    double a2 = (B.p.x-A.p.x)*(C.p.y-A.p.y) - (C.p.x-A.p.x)*(B.p.y-A.p.y);
                    if (std::fabs(a2) < 1e-9) continue;
                    Vec3 va(A.p.x, drape(A.p) + extraH, A.p.y), vb(B.p.x, drape(B.p) + extraH, B.p.y),
                         vc(C.p.x, drape(C.p) + extraH, C.p.y);
                    if (uv)   // carriageway: bake road-local UV so the RoadMarkings shader paints lines
                        MeshBuilder::emitTriUV(mesh, va, vb, vc, Vec3(0, 1, 0), col,
                            static_cast<float>(A.u), static_cast<float>(A.v), static_cast<float>(B.u),
                            static_cast<float>(B.v), static_cast<float>(C.u), static_cast<float>(C.v));
                    else
                        MeshBuilder::emitTri(mesh, va, vb, vc, Vec3(0, 1, 0), col);
                }
            };
            fan(clip(cellPoly, 0.0, true), 0.0, p.roadColor, true);                              // carriageway (UV)
            fan(clip(clip(cellPoly, sw, true), 0.0, false), p.curbHeight, p.sidewalkColor, false);  // walk

            // Curb face: the vertical step up the sdf=0 contour, facing the street.
            std::vector<Vec2> zc;
            for (int e = 0; e < 4; ++e) {
                const SVtx& A = corners[e]; const SVtx& B = corners[(e + 1) % 4];
                if ((A.s < 0) != (B.s < 0)) {
                    double t = A.s / (A.s - B.s); zc.push_back(A.p + (B.p - A.p) * t);
                }
            }
            if (zc.size() == 2) {
                double gx = ((s10 + s11) - (s00 + s01)), gz = ((s01 + s11) - (s00 + s10));
                double gl = std::sqrt(gx*gx + gz*gz);
                Vec3 nrm = gl > 1e-9 ? Vec3(-gx/gl, 0, -gz/gl) : Vec3(0, 1, 0);   // toward the road
                Vec2 q0 = zc[0], q1 = zc[1];
                double y0 = drape(q0), y1 = drape(q1);
                Vec3 b0(q0.x, y0, q0.y), b1(q1.x, y1, q1.y);
                Vec3 t0(q0.x, y0 + p.curbHeight, q0.y), t1(q1.x, y1 + p.curbHeight, q1.y);
                Vec3 cc = p.curbColor * g;
                MeshBuilder::emitTri(mesh, b0, b1, t1, nrm, cc);
                MeshBuilder::emitTri(mesh, b0, t1, t0, nrm, cc);
            }
        }
    return mesh;
}

std::vector<UnionSpine> graphToSpines(const RoadGraph& graph) {
    std::vector<UnionSpine> spines;
    spines.reserve(graph.edges.size());
    for (const RoadEdge& e : graph.edges) {
        if (e.a == e.b) continue;
        UnionSpine s;
        s.halfWidth = e.width * 0.5;
        s.points = { graph.nodes[e.a].pos, graph.nodes[e.b].pos };
        spines.push_back(std::move(s));
    }
    return spines;
}

RenderMesh unionRoadbed(const RoadGraph& graph, const RoadbedParams& params) {
    return unionRoadbed(graphToSpines(graph), params);
}

std::vector<std::vector<Vec2>> traceChains(const RoadGraph& g) {
    const int n = static_cast<int>(g.nodes.size());
    std::vector<std::vector<int>> inc(n);
    for (int e = 0; e < static_cast<int>(g.edges.size()); ++e) {
        inc[g.edges[e].a].push_back(e); inc[g.edges[e].b].push_back(e);
    }
    auto deg = [&](int v) { return static_cast<int>(inc[v].size()); };
    auto other = [&](int e, int v) { return g.edges[e].a == v ? g.edges[e].b : g.edges[e].a; };
    std::vector<char> used(g.edges.size(), 0);
    std::vector<std::vector<Vec2>> chains;

    auto walk = [&](int v, int e) {
        std::vector<Vec2> chain{ g.nodes[v].pos };
        int cur = v, ce = e;
        for (;;) {
            used[ce] = 1; int nx = other(ce, cur); chain.push_back(g.nodes[nx].pos);
            if (deg(nx) != 2) break;                       // hit a junction / dead end
            int ne = -1; for (int e2 : inc[nx]) if (e2 != ce && !used[e2]) { ne = e2; break; }
            if (ne < 0) break;
            cur = nx; ce = ne;
        }
        if (chain.size() >= 2) chains.push_back(std::move(chain));
    };
    for (int v = 0; v < n; ++v)                            // chains anchored at junctions
        if (deg(v) != 2)
            for (int e : inc[v]) if (!used[e]) walk(v, e);
    for (int e = 0; e < static_cast<int>(g.edges.size()); ++e)   // leftover pure loops
        if (!used[e]) walk(g.edges[e].a, e);
    return chains;
}

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

RenderMesh laneMarkings(const RoadGraph& g, const LaneMarkParams& p) {
    RenderMesh mesh;
    double hw = std::max(0.02, p.markWidth * 0.5);
    auto append = [&](RenderMesh r) {
        for (Vertex& v : r.vertices)               // drape onto the road surface
            v.position.y = (p.heightAt ? p.heightAt(v.position.x, v.position.z) : 0.0) + p.lift;
        std::size_t base = mesh.vertices.size();
        for (const Vertex& v : r.vertices) mesh.vertices.push_back(v);
        for (unsigned idx : r.indices) mesh.indices.push_back(static_cast<unsigned>(base) + idx);
    };
    for (const std::vector<Vec2>& chain : traceChains(g)) {
        std::vector<Vec2> line = trimEnds(chain, p.trim);
        if (line.size() < 2) continue;
        if (p.dashLength <= 0.0) {                  // solid centerline
            append(strokeRibbon(line, { hw }, 0.0, p.color, false));
            continue;
        }
        // Dashed: walk the line, emitting a stroke for each dash-length on/off span.
        double phase = 0.0; bool on = true;
        std::vector<Vec2> dash{ line[0] };
        for (std::size_t i = 1; i < line.size(); ) {
            Vec2 a = dash.back(), b = line[i];
            double seg = (b - a).length();
            double want = (on ? p.dashLength : p.dashGap) - phase;
            if (seg <= want) { phase += seg; if (on) dash.push_back(b); ++i; }
            else {
                Vec2 cut = a + (b - a) * (want / std::max(1e-9, seg));
                if (on) { dash.push_back(cut); append(strokeRibbon(dash, { hw }, 0.0, p.color, false)); }
                dash = { cut }; on = !on; phase = 0.0;
            }
        }
        if (on && dash.size() >= 2) append(strokeRibbon(dash, { hw }, 0.0, p.color, false));
    }
    return mesh;
}

}  // namespace engine
