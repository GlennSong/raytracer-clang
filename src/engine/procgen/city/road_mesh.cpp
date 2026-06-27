#include "road_mesh.h"

#include "../../mesh_builder.h"
#include "road_offset.h"          // ribbonOutline, polygonUnion (the unified join engine)
#include "street_kit.h"           // roundPolygonCorners (the unified corner-fillet pass)
#include <algorithm>
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
        Vec3 a(centerline[i].x, 0.0, centerline[i].y);          // Vec2.y is world Z
        Vec3 b(centerline[i + 1].x, 0.0, centerline[i + 1].y);
        regions.push_back(makeFlattenRamp(a, b, profileY[i], profileY[i + 1], hw, falloff));
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

RenderMesh weldSolid(const std::vector<UnionSpine>& spines, const WeldSolidParams& p) {
    // 1. Per-spine profile: cumulative arc length + a smoothed, grade-limited height. Terrain is
    //    sampled along each centerline and ironed by roadProfile (follows hills, not every bump);
    //    with no terrain the height is the flat topY. These also carry the road-local UV.
    struct Prof { std::vector<Vec2> cl; std::vector<double> s; std::vector<double> h; double hw; bool closed; };
    std::vector<Prof> profs;
    for (const UnionSpine& sp : spines) {
        const int n = static_cast<int>(sp.points.size());
        if (n < 2) continue;
        std::vector<double> sArc(n, 0.0), h(n, p.topY);
        for (int i = 1; i < n; ++i) sArc[i] = sArc[i - 1] + (sp.points[i] - sp.points[i - 1]).length();
        if (p.heightAt) {
            std::vector<double> ground(n);
            for (int i = 0; i < n; ++i) ground[i] = p.heightAt(sp.points[i].x, sp.points[i].y);
            h = roadProfile(ground, sArc, p.maxGrade);
            for (double& v : h) v += p.topY;            // topY lifts the deck above the terrain
        }
        bool closed = sp.closed ||
            (n >= 4 && (sp.points.front() - sp.points.back()).length() < 1e-6);
        profs.push_back({sp.points, sArc, h, sp.halfWidth, closed});
    }
    // Sample anywhere from the NEAREST spine: surface height (smoothed profile), and road-local UV
    // — mu = 2 + lateral/halfWidth in [1,3] (centre 2, curbs 1 & 3; the RoadMarkings shader paints
    // from it), mv = arc-length along the road (for dashed dividers). Junctions agree where spines
    // meet. Off the carriageway (no spine) mu stays 0 so no paint lands there.
    auto sample = [&](double x, double z, double& oh, double& omu, double& omv, int& oSpine) {
        oh = p.topY; omu = 0.0; omv = 0.0; oSpine = -1;
        Vec2 q(x, z); double bestD2 = 1e30;
        for (std::size_t pi = 0; pi < profs.size(); ++pi) {
            const Prof& pr = profs[pi];
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
                    omv = pr.s[i] + (pr.s[i + 1] - pr.s[i]) * t;
                    oSpine = static_cast<int>(pi);
                }
            }
        }
    };
    auto heightOf = [&](double x, double z) -> double {
        double h, mu, mv; int si; sample(x, z, h, mu, mv, si); return h;
    };

    // 2. The welded outline (same join engine as weldRibbons), rounded at junctions. A CLOSED
    //    spine (a roundabout ring) becomes an annulus: its outer rail welds with the spokes, its
    //    inner rail is punched as a central island hole.
    std::vector<Poly2> ribbons, forcedHoles;
    for (const UnionSpine& s : spines) {
        if (s.points.size() < 2) continue;
        bool closed = s.closed ||
            (s.points.size() >= 4 && (s.points.front() - s.points.back()).length() < 1e-6);
        if (closed) {
            Poly2 outer, inner; ringRibbon(s.points, s.halfWidth, outer, inner);
            if (outer.size() >= 3) ribbons.push_back(std::move(outer));
            if (inner.size() >= 3) forcedHoles.push_back(std::move(inner));
        } else {
            Poly2 r = ribbonOutline(s.points, s.halfWidth);
            if (r.size() >= 3) ribbons.push_back(std::move(r));
        }
    }
    std::vector<Poly2> outers, holes;
    for (Poly2& L : polygonUnion(ribbons)) {
        Poly2 R = (p.cornerRadius > 0.0) ? roundPolygonCorners(L, p.cornerRadius, 5) : L;
        (signedArea(R) > 0 ? outers : holes).push_back(std::move(R));
    }
    for (Poly2& h : forcedHoles) holes.push_back(std::move(h));   // roundabout islands

    RenderMesh mesh;
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
            Vec3 tA = P3(a, 0), tB = P3(b, 0), bA = P3(a, -p.thickness), bB = P3(b, -p.thickness);
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
            std::vector<Vec2> off(m);                       // per-vertex outward mitered offset
            for (int i = 0; i < m; ++i) {
                Vec2 n0 = rnorm(loop[i] - loop[(i + m - 1) % m]);
                Vec2 n1 = rnorm(loop[(i + 1) % m] - loop[i]);
                Vec2 bis = n0 + n1; double bl = bis.length();
                Vec2 mm = bl < 1e-9 ? n1 : bis * (1.0 / bl);
                double cosH = std::max(0.25, dot(mm, n1));   // clamp miter <= 4*width at sharp corners
                off[i] = mm * (p.sidewalkWidth / cosH);
            }
            const double ch = p.curbHeight;
            for (int i = 0; i < m; ++i) {
                int j = (i + 1) % m;
                const Vec2& a = loop[i]; const Vec2& b = loop[j];
                if ((b - a).length() < 1e-9) continue;
                Vec2 ao = a + off[i], bo = b + off[j];
                Vec3 eo3(rnorm(b - a).x, 0, rnorm(b - a).y);  // outward (per edge)
                Vec3 aT = P3(a, ch), bT = P3(b, ch), aR = P3(a, 0), bR = P3(b, 0);
                Vec3 aoT = P3(ao, ch), boT = P3(bo, ch),
                     aoB = P3(ao, -p.thickness), boB = P3(bo, -p.thickness);
                MeshBuilder::emitQuad(mesh, aR, bR, bT, aT, eo3 * -1.0, p.curbColor);     // curb lip (road-facing)
                MeshBuilder::emitQuad(mesh, aT, bT, boT, aoT, Vec3(0, 1, 0), p.sidewalkColor); // slab top
                MeshBuilder::emitQuad(mesh, aoT, boT, boB, aoB, eo3, p.curbColor);        // outer face
            }
        };
        for (const Poly2& outer : outers) sidewalk(outer);
        for (const Poly2& hole : holes) sidewalk(hole);
    }

    // The road SURFACE as per-spine ribbon strips, resampled to short quads. Each quad lays a plain
    // deck (up) + underside (down); open blocks fall out for free (no strip lies over them) and
    // strips overlapping at a junction are coplanar same-colour asphalt, so they read as one
    // surface. Lane MARKINGS ride a third quad just above, with road-local UV (left rail u=1, right
    // u=3, v = arc-length) so the paint interpolates exactly — crisp double-yellow, dashed lane
    // dividers, edges. A quad over ANOTHER road's corridor is a junction: it stays plain (no paint).
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
                // Plain deck (up) and underside (down).
                Vec3 dL0 = P(c0, o0, h0, +1), dR0 = P(c0, o0, h0, -1),
                     dL1 = P(c1, o1, h1, +1), dR1 = P(c1, o1, h1, -1);
                MeshBuilder::emitTri(mesh, dL0, dR0, dR1, Vec3(0, 1, 0), p.topColor);
                MeshBuilder::emitTri(mesh, dL0, dR1, dL1, Vec3(0, 1, 0), p.topColor);
                Vec3 bL0 = P(c0, o0, h0 - p.thickness, +1), bR0 = P(c0, o0, h0 - p.thickness, -1),
                     bL1 = P(c1, o1, h1 - p.thickness, +1), bR1 = P(c1, o1, h1 - p.thickness, -1);
                MeshBuilder::emitTri(mesh, bL0, bR0, bR1, Vec3(0, -1, 0), p.bottomColor);
                MeshBuilder::emitTri(mesh, bL0, bR1, bL1, Vec3(0, -1, 0), p.bottomColor);
                // Markings on top, unless this quad straddles another road (a junction).
                Vec2 mid = (c0 + c1) * 0.5;
                bool junction = false;
                for (std::size_t pj = 0; pj < profs.size() && !junction; ++pj)
                    if (pj != pi && distToSpine(mid, profs[pj]) < profs[pj].hw + 0.5) junction = true;
                if (junction) continue;
                Vec3 mL0 = P(c0, o0, h0 + markLift, +1), mR0 = P(c0, o0, h0 + markLift, -1),
                     mL1 = P(c1, o1, h1 + markLift, +1), mR1 = P(c1, o1, h1 + markLift, -1);
                MeshBuilder::emitTriUV(mesh, mL0, mR0, mR1, Vec3(0, 1, 0), p.topColor,
                                       1.0f, (float)s0, 3.0f, (float)s0, 3.0f, (float)s1);
                MeshBuilder::emitTriUV(mesh, mL0, mR1, mL1, Vec3(0, 1, 0), p.topColor,
                                       1.0f, (float)s0, 3.0f, (float)s1, 1.0f, (float)s1);
            }
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

    std::vector<double> sdf(static_cast<std::size_t>(nx) * nz);
    for (int j = 0; j < nz; ++j)
        for (int i = 0; i < nx; ++i) {
            Vec2 pt(X(i), Z(j)); double best = 1e30;
            for (const UnionSpine& s : spines) {
                int n = static_cast<int>(s.points.size()); if (n < 2) continue;
                int segs = s.closed ? n : n - 1; double d2 = 1e30;
                for (int k = 0; k < segs; ++k)
                    d2 = std::min(d2, segDist2(pt, s.points[k], s.points[(k + 1) % n]));
                best = std::min(best, std::sqrt(d2) - s.halfWidth);
            }
            sdf[gi(i, j)] = best;
        }

    auto drape = [&](const Vec2& v) { return (p.heightAt ? p.heightAt(v.x, v.y) : 0.0) + p.lift; };
    // A deterministic per-cell value jitter so the flat bands read as a textured surface.
    auto grainAt = [&](int i, int j) {
        uint32_t h = static_cast<uint32_t>(i * 73856093) ^ static_cast<uint32_t>(j * 19349663);
        h = (h ^ (h >> 13)) * 1274126177u;
        return (1.0 - p.grain) + p.grain * ((h >> 8) & 0xffff) / 65535.0 * 2.0;
    };
    struct SVtx { Vec2 p; double s; };
    auto clip = [](const std::vector<SVtx>& poly, double thr, bool below) {
        std::vector<SVtx> out; int n = static_cast<int>(poly.size());
        for (int i = 0; i < n; ++i) {
            const SVtx& A = poly[i]; const SVtx& B = poly[(i + 1) % n];
            bool ain = below ? (A.s < thr) : (A.s >= thr);
            bool bin = below ? (B.s < thr) : (B.s >= thr);
            if (ain) out.push_back(A);
            if (ain != bin) {
                double t = (thr - A.s) / (B.s - A.s);
                out.push_back({A.p + (B.p - A.p) * t, thr});
            }
        }
        return out;
    };

    for (int j = 0; j < nz - 1; ++j)
        for (int i = 0; i < nx - 1; ++i) {
            double s00 = sdf[gi(i,j)], s10 = sdf[gi(i+1,j)], s11 = sdf[gi(i+1,j+1)], s01 = sdf[gi(i,j+1)];
            if (s00 >= sw && s10 >= sw && s11 >= sw && s01 >= sw) continue;     // wholly outside
            SVtx corners[4] = { {Vec2(X(i),   Z(j)),   s00}, {Vec2(X(i+1), Z(j)),   s10},
                                {Vec2(X(i+1), Z(j+1)), s11}, {Vec2(X(i),   Z(j+1)), s01} };
            std::vector<SVtx> cellPoly(corners, corners + 4);
            double g = grainAt(i, j);

            auto fan = [&](const std::vector<SVtx>& poly, double extraH, Vec3 col) {
                col = col * g;
                for (std::size_t k = 1; k + 1 < poly.size(); ++k) {
                    Vec2 a = poly[0].p, b = poly[k].p, c = poly[k + 1].p;
                    double a2 = (b.x-a.x)*(c.y-a.y) - (c.x-a.x)*(b.y-a.y);
                    if (std::fabs(a2) < 1e-9) continue;
                    MeshBuilder::emitTri(mesh,
                        Vec3(a.x, drape(a) + extraH, a.y), Vec3(b.x, drape(b) + extraH, b.y),
                        Vec3(c.x, drape(c) + extraH, c.y), Vec3(0, 1, 0), col);
                }
            };
            fan(clip(cellPoly, 0.0, true), 0.0, p.roadColor);                   // carriageway
            fan(clip(clip(cellPoly, sw, true), 0.0, false), p.curbHeight, p.sidewalkColor);  // walk

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
