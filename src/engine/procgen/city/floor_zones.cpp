#include "floor_zones.h"

#include "roads/lanes/geom2d.h"   // the Clipper booleans (every piece kept, holes too)
#include "space_graph.h"          // cellUseName: the tags' RoomKind names

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>

namespace engine {

namespace G = roads::lanes;

namespace {

constexpr Real kCorridorRing = 2.2;   // core_plan.cpp's kCorridor: the ring the core keeps clear
constexpr Real kMaxBulge = 1.0;       // a curve is cut into chords bulging no more than this
constexpr Real kSharpTurn = 0.6;      // radians: a facade turning more than this at a vertex is a corner
// an edge needs this much clear run onto circulation for a door: the doorway and the wall either side of it that
// emitRooms keeps (a wall shorter than kRoomDoorW + 0.6 gets no doorway at all)
constexpr Real kDoorRun = kRoomDoorW + 0.65;

uint32_t mix(uint32_t h) {
    h ^= h >> 16; h *= 0x7feb352du; h ^= h >> 15; h *= 0x846ca68bu; h ^= h >> 16;
    return h;
}
uint32_t hashAt(uint32_t seed, const Vec2& p) {
    return mix(seed ^ (static_cast<uint32_t>(std::lround(p.x * 10.0)) * 73856093u) ^
               (static_cast<uint32_t>(std::lround(p.y * 10.0)) * 19349663u));
}

Real ringAreaAbs(const Poly2& r) { return std::fabs(signedArea(r)); }

// The set's largest outer ring, CCW (a cut that leaves a sliver keeps the lot, not the sliver).
Poly2 largestOuter(const G::PolySet& s) {
    const G::Polygon2* best = nullptr;
    Real bestA = 0;
    for (const G::Polygon2& p : s) {
        const Real a = ringAreaAbs(p.outer);
        if (a > bestA) { bestA = a; best = &p; }
    }
    if (!best) return {};
    Poly2 r = best->outer;
    ensureCCW(r);
    return r;
}

// Everything on one side of the line through p along dir (the left side, or the right), as a big rectangle.
G::PolySet halfPlane(const Vec2& p, const Vec2& dir, bool left) {
    const Vec2 d = normalize(dir), n = left ? perp(d) : perp(d) * -1.0;
    constexpr Real R = 4000;
    Poly2 r = {p - d * R, p + d * R, p + d * R + n * R, p - d * R + n * R};
    ensureCCW(r);
    return G::fromRing(r);
}

Real distToSegment(const Vec2& q, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const Real t = std::clamp(dot(q - a, ab) / std::max(Real(1e-12), ab.lengthSquared()), Real(0), Real(1));
    return (q - (a + ab * t)).length();
}
Real distToRing(const Vec2& q, const Poly2& r) {
    Real best = 1e9;
    for (std::size_t i = 0; i < r.size(); ++i) best = std::min(best, distToSegment(q, r[i], r[(i + 1) % r.size()]));
    return best;
}

// THE LARGEST RECTANGLE SQUARE TO `u` INSIDE `poly` (the furniture's frame). Scanlines across the polygon at 0.1 m
// steps along perp(u) give, at each, the widest run inside it; every band of scanlines [t0, t1] keeps the overlap of
// its runs, and the band whose overlap times depth is largest wins (a trapezoid keeps its full depth, an L its bigger
// arm). CCW; frame[0] -> frame[1] the side lowest along perp(u). Empty when nothing a metre square fits.
Poly2 inscribedRect(const Poly2& poly, const Vec2& uIn) {
    const Vec2 u = normalize(uIn), v = perp(u);
    const std::size_t n = poly.size();
    if (n < 3) return {};
    Real tLo = 1e30, tHi = -1e30;
    for (const Vec2& p : poly) { tLo = std::min(tLo, dot(p, v)); tHi = std::max(tHi, dot(p, v)); }
    const Real step = 0.1;
    const int rows = static_cast<int>((tHi - tLo) / step);
    if (rows < 10) return {};
    std::vector<Real> L(static_cast<std::size_t>(rows) + 1), R(static_cast<std::size_t>(rows) + 1);
    std::vector<Real> xs;
    for (int r = 0; r <= rows; ++r) {
        const Real t = tLo + 0.02 + (tHi - tLo - 0.04) * r / rows;
        xs.clear();
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = poly[i], b = poly[(i + 1) % n];
            const Real ta = dot(a, v), tb = dot(b, v);
            if ((ta <= t) == (tb <= t)) continue;
            xs.push_back(dot(a, u) + (dot(b, u) - dot(a, u)) * (t - ta) / (tb - ta));
        }
        std::sort(xs.begin(), xs.end());
        Real bl = 0, br = -1;
        for (std::size_t k = 0; k + 1 < xs.size(); k += 2)
            if (xs[k + 1] - xs[k] > br - bl) { bl = xs[k]; br = xs[k + 1]; }
        L[static_cast<std::size_t>(r)] = bl;
        R[static_cast<std::size_t>(r)] = br;
    }
    Real bestA = 0, b0 = 0, b1 = 0, bt0 = 0, bt1 = 0;
    for (int r0 = 0; r0 <= rows; ++r0) {
        Real l = -1e30, rr = 1e30;
        for (int r1 = r0; r1 <= rows; ++r1) {
            l = std::max(l, L[static_cast<std::size_t>(r1)]);
            rr = std::min(rr, R[static_cast<std::size_t>(r1)]);
            if (rr - l < 1.0) break;
            const Real t0 = tLo + 0.02 + (tHi - tLo - 0.04) * r0 / rows, t1 = tLo + 0.02 + (tHi - tLo - 0.04) * r1 / rows;
            const Real a = (rr - l) * (t1 - t0);
            if (t1 - t0 >= 1.0 && a > bestA) { bestA = a; b0 = l; b1 = rr; bt0 = t0; bt1 = t1; }
        }
    }
    if (bestA <= 0) return {};
    b0 += 0.01; b1 -= 0.01;
    auto P = [&](Real s, Real t) { return u * s + v * t; };
    return {P(b0, bt0), P(b1, bt0), P(b1, bt1), P(b0, bt1)};
}

// The facade's bay lines along a chord of length W (room_plan.cpp's bayBoundaries: the 1.6 m curtain-wall bays, the
// masonry facade's bayWidth), so a partition meets a mullion or a pier.
std::vector<Real> bayLines(Real W, const BuildingParams& p) {
    const Real bw = p.curtainWall ? Real(1.6) : std::max(p.bayWidth, Real(0.5));
    const int bays = std::max(1, static_cast<int>(std::lround(W / bw)));
    std::vector<Real> xs;
    for (int b = 0; b <= bays; ++b) xs.push_back(W * b / bays);
    return xs;
}

// One chord of the facade a row of window lots fronts: a straight edge, or a stretch of a curve's chords.
struct Chord {
    Vec2 A, B, d, in;       // ends, direction, inward normal
    Real W = 0, bulge = 0;  // length; how far the facade stands out past it
    bool curved = false;
    enum End : uint8_t { Smooth, Convex, Reflex };
    End startK = Smooth, endK = Smooth;
    Vec2 c0, c1;            // the cut directions (inward) at its ends
    bool corner = false;    // a rounded corner's chord: its lots are corner lots
};

std::vector<Chord> facadeChords(const Poly2& plan) {
    const std::size_t n = plan.size();
    std::vector<Real> len(n), turn(n);
    std::vector<Vec2> dir(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 dv = plan[(i + 1) % n] - plan[i];
        len[i] = dv.length();
        dir[i] = dv * (1.0 / std::max(Real(1e-9), len[i]));
    }
    for (std::size_t i = 0; i < n; ++i) {   // turn[i]: at the vertex from edge i into edge i + 1
        const Vec2 a = dir[i], b = dir[(i + 1) % n];
        turn[i] = std::atan2(cross(a, b), dot(a, b));
    }
    // a run breaks at a corner, or where a curve's short chords meet a long straight wall
    std::vector<bool> brk(n, false);
    bool any = false;
    for (std::size_t i = 0; i < n; ++i) {
        const Real l0 = len[i], l1 = len[(i + 1) % n];
        const bool lenJump = std::max(l0, l1) > 3.0 && std::max(l0, l1) > 1.6 * std::min(l0, l1);
        brk[i] = std::fabs(turn[i]) > kSharpTurn || lenJump;
        any = any || brk[i];
    }
    // runs of edges between breaks (one closed run when nothing breaks: a drum)
    struct Run { std::vector<std::size_t> edges; Chord::End startK = Chord::Smooth, endK = Chord::Smooth; };
    std::vector<Run> runs;
    auto endKind = [&](std::size_t vertexTurn) {
        const Real t = turn[vertexTurn];
        if (std::fabs(t) <= kSharpTurn) return Chord::Smooth;
        return t > 0 ? Chord::Convex : Chord::Reflex;
    };
    if (!any) {
        Run r;
        for (std::size_t i = 0; i < n; ++i) r.edges.push_back(i);
        runs.push_back(r);
    } else {
        std::size_t s = 0;
        while (!brk[(s + n - 1) % n]) ++s;   // a run starts after a break
        Run cur;
        for (std::size_t k = 0; k < n; ++k) {
            const std::size_t e = (s + k) % n;
            if (cur.edges.empty()) cur.startK = endKind((e + n - 1) % n);
            cur.edges.push_back(e);
            if (brk[e]) { cur.endK = endKind(e); runs.push_back(cur); cur = Run{}; }
        }
        if (!cur.edges.empty()) runs.push_back(cur);
    }
    std::vector<Chord> out;
    for (const Run& r : runs) {
        // the run's chords: as many edges as keep the bulge under kMaxBulge and the sweep under 60 degrees
        Real sweep = 0;
        for (std::size_t k = 0; k + 1 < r.edges.size(); ++k) sweep += turn[r.edges[k]];
        const bool cornerArc = r.edges.size() > 1 && r.startK == Chord::Smooth && r.endK == Chord::Smooth && any &&
                               std::fabs(sweep) < 1.75;
        if (std::getenv("RT_ZONE_DEBUG"))
            std::fprintf(stderr, "[zones] run of %zu edges, ends %d/%d, sweep %.2f, corner arc %d\n", r.edges.size(),
                         static_cast<int>(r.startK), static_cast<int>(r.endK), sweep, cornerArc ? 1 : 0);
        std::size_t k = 0;
        const std::size_t first = out.size();
        while (k < r.edges.size()) {
            std::size_t m = 1;
            while (k + m < r.edges.size()) {
                const Vec2 A = plan[r.edges[k]], B = plan[(r.edges[k + m] + 1) % n];
                const Vec2 d = normalize(B - A), in = perp(d);
                Real bul = 0, sw = 0;
                for (std::size_t j = 1; j <= m; ++j) bul = std::max(bul, -dot(plan[r.edges[k + j]] - A, in));
                for (std::size_t j = 0; j < m; ++j) sw += std::fabs(turn[r.edges[k + j]]);
                if (bul > kMaxBulge || sw > 1.05) break;
                ++m;
            }
            Chord c;
            c.A = plan[r.edges[k]];
            c.B = plan[(r.edges[k + m - 1] + 1) % n];
            c.W = (c.B - c.A).length();
            c.d = (c.B - c.A) * (1.0 / std::max(Real(1e-9), c.W));
            c.in = perp(c.d);
            c.curved = m > 1;
            for (std::size_t j = 1; j < m; ++j) c.bulge = std::max(c.bulge, -dot(plan[r.edges[k + j]] - c.A, c.in));
            c.startK = k == 0 ? r.startK : Chord::Smooth;
            c.endK = k + m == r.edges.size() ? r.endK : Chord::Smooth;
            c.corner = cornerArc;
            if (c.W > 0.3) out.push_back(c);
            k += m;
        }
        (void)first;
    }
    // the cut at each shared vertex: a corner's is square to each chord; a smooth joint's the bisector of both
    // chords' normals, the same line for the chord either side so their lots meet without a gap
    const std::size_t m = out.size();
    for (std::size_t i = 0; i < m; ++i) {
        Chord& c = out[i];
        const Chord& prev = out[(i + m - 1) % m];
        const Chord& next = out[(i + 1) % m];
        c.c0 = c.startK == Chord::Smooth && m > 1 ? normalize(prev.in + c.in) : c.in;
        c.c1 = c.endK == Chord::Smooth && m > 1 ? normalize(c.in + next.in) : c.in;
    }
    return out;
}

// (folded through fromRing + unionSets: geom2d's unionRings result subtracted wrongly here -- F less the union of a
// floor's lots came out 106 m2 where subtracting the same lots one by one gave the true 37)
G::PolySet unionAll(const std::vector<Poly2>& rings) {
    G::PolySet u;
    for (const Poly2& r : rings) {
        if (r.size() < 3) continue;
        u = u.empty() ? G::fromRing(r) : G::unionSets(u, G::fromRing(r));
    }
    return u;
}

// The longest run of edge i (CCW poly) whose outside lies in `walk`: its length and the middle of it (a parameter
// along the edge).
Real accessRun(const Poly2& poly, std::size_t i, const G::PreparedSet& walk, Real& mid) {
    const Vec2 a = poly[i], b = poly[(i + 1) % poly.size()];
    const Vec2 ab = b - a;
    const Real L = ab.length();
    mid = 0.5;
    if (L < 0.3) return 0;
    const Vec2 d = ab * (1.0 / L), out(d.y, -d.x);
    const int samples = std::max(2, static_cast<int>(L / 0.25));
    Real best = 0, run0 = -1;
    for (int k = 0; k <= samples; ++k) {
        const Real t = (k + 0.5) / (samples + 1);
        const bool ok = walk.contains(a + ab * t + out * 0.3);
        if (ok && run0 < 0) run0 = t;
        if ((!ok || k == samples) && run0 >= 0) {
            const Real t1 = ok ? t : (k - 0.5) / (samples + 1);
            if ((t1 - run0) * L > best) { best = (t1 - run0) * L; mid = (run0 + t1) * 0.5; }
            run0 = -1;
        }
    }
    return best;
}

}  // namespace

// ------------------------------------------------------------------------------------------- the partition
FloorPartition partitionFloor(const Poly2& plateIn, const CorePlan& core, Real inset, const BuildingParams& params,
                              const FloorZoneParams& zp, const Poly2& keepOut) {
    FloorPartition fp;
    Poly2 plate;
    for (const Vec2& p : plateIn)
        if (plate.empty() || (p - plate.back()).length() > 1e-4) plate.push_back(p);
    if (plate.size() > 2 && (plate.front() - plate.back()).length() < 1e-4) plate.pop_back();
    if (plate.size() < 3) return fp;
    ensureCCW(plate);

    // the field: inside the walls, less the core with its corridor ring and the stair's well
    const G::PolySet IN = G::offsetSet(G::fromRing(plate), -inset);
    G::PolySet K;
    Poly2 coreRing;
    if (core.valid) {
        const Vec2 u = core.frame.u, v = core.frame.v;
        const Vec2 o = core.frame.origin;
        const Real c = kCorridorRing;
        coreRing = {o - u * c - v * c, o + u * (core.length + c) - v * c, o + u * (core.length + c) + v * (core.depth + c),
                    o - u * c + v * (core.depth + c)};
        ensureCCW(coreRing);
        K = G::fromRing(coreRing);
        fp.lobby = core.frame.toWorld({core.length * 0.5, -1.1});
    } else {
        fp.lobby = centroid(plate);
    }
    G::PolySet F = K.empty() ? IN : G::differenceSets(IN, K);
    if (keepOut.size() >= 3) F = G::differenceSets(F, G::offsetSet(G::fromRing(keepOut), 0.6));
    fp.fieldArea = G::setArea(F);
    if (fp.fieldArea < 4) return fp;
    // WINDOW LOTS: a row off each chord of the facade, the chord cut at the bays into lots about lotWidth wide; a
    // turning corner's square goes to the first row that reaches it (the next finds it claimed)
    // (read off the outline with its very short edges folded away: a stadium's ends are two quarter circles joined by
    // a few centimetres of straight, one half circle to the eye, not two rounded corners)
    Poly2 outline;
    for (const Vec2& p : plate)
        if (outline.empty() || (p - outline.back()).length() > 0.5) outline.push_back(p);
    while (outline.size() > 3 && (outline.front() - outline.back()).length() <= 0.5) outline.pop_back();
    const std::vector<Chord> chords = facadeChords(outline.size() >= 3 ? outline : plate);
    G::PolySet claimed;
    const Real dW = zp.windowDepth;
    for (std::size_t ci = 0; ci < chords.size(); ++ci) {
        const Chord& c = chords[ci];
        struct Span { Real s0, s1; bool corner; };
        std::vector<Span> spans;
        Real a = 0, b = c.W;
        // a chord owns the corner at its END; at its start, when the chord before took that corner, its lots begin an
        // aisle past it, so the aisle runs on to the glass there and the corner lot has a way in
        const Chord& prev = chords[(ci + chords.size() - 1) % chords.size()];
        const bool prevOwns = c.startK == Chord::Convex && prev.endK == Chord::Convex && prev.W > dW + 2.0 && chords.size() > 1;
        if (prevOwns) a = std::min(c.W, dW + zp.aisle);
        const bool cornerStart = !prevOwns && c.startK == Chord::Convex && c.W > dW + 2.0;
        const bool cornerEnd = c.endK == Chord::Convex && c.W - a - (cornerStart ? dW : 0) > dW + 2.0;
        if (cornerStart) { spans.push_back({0, dW, true}); a = dW; }
        Span tail{0, 0, false};
        if (cornerEnd) { tail = {c.W - dW, c.W, true}; b = c.W - dW; }
        if (b - a > 0.8) {
            const int k = c.curved && b - a < 1.6 * zp.lotWidth ? 1
                        : std::max(1, static_cast<int>(std::lround((b - a) / zp.lotWidth)));
            std::vector<Real> cuts;
            for (int j = 0; j <= k; ++j) cuts.push_back(a + (b - a) * j / k);
            if (!c.curved) {   // square to the bays: each inner cut on the nearest bay line
                const std::vector<Real> bays = bayLines(c.W, params);
                for (int j = 1; j < k; ++j) {
                    Real best = cuts[static_cast<std::size_t>(j)], bd = 1e9;
                    for (Real x : bays)
                        if (std::fabs(x - cuts[static_cast<std::size_t>(j)]) < bd) { bd = std::fabs(x - cuts[static_cast<std::size_t>(j)]); best = x; }
                    if (best > cuts[static_cast<std::size_t>(j - 1)] + 2.0 && best < b - 2.0) cuts[static_cast<std::size_t>(j)] = best;
                }
            }
            for (int j = 0; j < k; ++j) spans.push_back({cuts[static_cast<std::size_t>(j)], cuts[static_cast<std::size_t>(j + 1)], c.corner});
        }
        if (cornerEnd) spans.push_back(tail);
        for (const Span& sp : spans) {
            Vec2 k0 = sp.s0 <= 1e-6 ? c.c0 : c.in, k1 = sp.s1 >= c.W - 1e-6 ? c.c1 : c.in;
            if (sp.corner && sp.s0 <= 1e-6 && c.startK == Chord::Convex) k0 = c.in;
            if (sp.corner && sp.s1 >= c.W - 1e-6 && c.endK == Chord::Convex) k1 = c.in;
            auto cutPts = [&](Real s, const Vec2& k, Vec2& outer, Vec2& inner) {
                const Vec2 P = c.A + c.d * s;
                const Real cs = std::max(Real(0.35), dot(k, c.in));
                outer = P - k * ((c.bulge + 0.4) / cs);
                inner = P + k * (dW / cs);
            };
            Vec2 o0, i0, o1, i1;
            cutPts(sp.s0, k0, o0, i0);
            cutPts(sp.s1, k1, o1, i1);
            if (dot(i1 - i0, c.d) < 0.3) {   // the cuts cross short of the lot's depth (a tight curve): square ones
                cutPts(sp.s0, c.in, o0, i0);
                cutPts(sp.s1, c.in, o1, i1);
            }
            Poly2 strip = {o0, o1, i1, i0};
            ensureCCW(strip);
            G::PolySet lot = G::intersectSets(G::fromRing(strip), F);
            if (!claimed.empty()) lot = G::differenceSets(lot, claimed);
            Poly2 poly = largestOuter(lot);
            if (poly.size() < 3 || area(poly) < 2.5) continue;
            claimed = claimed.empty() ? G::fromRing(poly) : G::unionSets(claimed, G::fromRing(poly));
            FloorZone z;
            z.poly = poly;
            z.site = sp.corner ? ZoneSite::Corner : ZoneSite::Window;
            z.frontage = sp.s1 - sp.s0;
            z.curved = c.curved;
            z.group = static_cast<int>(ci);
            z.frame = inscribedRect(poly, c.d);
            fp.zones.push_back(z);
        }
    }

    // a SLIVER of a window lot (left where a curve's lots meet a straight wall's): joined to the neighbour it shares
    // most of a side with, else given back to the floor
    for (std::size_t z = 0; z < fp.zones.size();) {
        const FloorZone& Z = fp.zones[z];
        const bool sliver = area(Z.poly) < 6.0 || Z.frame.size() != 4 ||
                            std::min((Z.frame[1] - Z.frame[0]).length(), (Z.frame[3] - Z.frame[0]).length()) < 2.0;
        if (!sliver) { ++z; continue; }
        std::size_t best = fp.zones.size();
        Real bestGain = 0;
        for (std::size_t b = 0; b < fp.zones.size(); ++b) {
            if (b == z) continue;
            const G::PolySet un = G::unionSets(G::fromRing(Z.poly), G::fromRing(fp.zones[b].poly));
            if (un.size() != 1 || !un.front().holes.empty()) continue;
            // the shared side: what the union's outline loses against the two outlines; and the union still nearly a
            // rectangle (a sliver along a lot's side would make it an L with a doorway onto nothing)
            const Real gain = perimeter(Z.poly) + perimeter(fp.zones[b].poly) - perimeter(un.front().outer);
            const OBB2 ob = orientedBoundingBox(un.front().outer);
            if (ringAreaAbs(un.front().outer) < 0.85 * 4 * ob.half[0] * ob.half[1]) continue;
            if (gain > bestGain) { bestGain = gain; best = b; }
        }
        if (best < fp.zones.size() && bestGain > 1.0) {
            FloorZone& B = fp.zones[best];
            const Vec2 ax = B.frame.size() == 4 ? B.frame[1] - B.frame[0] : Vec2(1, 0);
            const G::PolySet un = G::unionSets(G::fromRing(Z.poly), G::fromRing(B.poly));
            B.poly = un.front().outer;
            ensureCCW(B.poly);
            B.frontage += Z.frontage;
            B.frame = inscribedRect(B.poly, ax);
        }
        fp.zones.erase(fp.zones.begin() + static_cast<std::ptrdiff_t>(z));
        z = 0;   // (indices moved: start over; there are few)
    }

    // THE COURT: what is left an aisle in from the window lots, less the corridor's long lanes run on to the aisle
    // (the streets), cut like a block's remainder
    G::PolySet I = claimed.empty() ? F : G::differenceSets(F, G::offsetSet(claimed, zp.aisle));
    if (core.valid && !I.empty()) {
        const Vec2 u = core.frame.u, v = core.frame.v, o = core.frame.origin;
        const Real c = kCorridorRing, s = std::min(zp.street, c);
        std::vector<Poly2> lanes;
        for (int side = 0; side < 2; ++side) {
            const Real v0 = side == 0 ? -c : core.depth + c - s, v1 = v0 + s;
            Poly2 lane = {o + u * -400.0 + v * v0, o + u * 400.0 + v * v0, o + u * 400.0 + v * v1, o + u * -400.0 + v * v1};
            ensureCCW(lane);
            lanes.push_back(lane);
        }
        I = G::differenceSets(I, unionAll(lanes));
    }
    // pieces with holes (a court round the core no street cut through) are halved across the hole first
    std::vector<G::Polygon2> pieces;
    for (const G::Polygon2& p : I) {
        if (p.holes.empty()) { pieces.push_back(p); continue; }
        const Poly2& h = p.holes.front();
        const Vec2 hc = centroid(h);
        const OBB2 ob = orientedBoundingBox(p.outer);
        const Vec2 cutDir = ob.axis[1 - ob.longAxis()];
        for (bool left : {true, false}) {
            const G::PolySet half = G::intersectSets(G::PolySet{p}, halfPlane(hc, cutDir, left));
            for (const G::Polygon2& q : half) pieces.push_back(q);
        }
    }
    // ...cut down like a block's remainder, every lot kept on circulation: a piece too deep for two rooms back to
    // back is split along its length with an aisle down the middle (a street through the block); a shallower one is
    // cut across, whichever way leaves both halves on circulation
    G::PolySet walkSoFar = G::differenceSets(F, claimed);
    {
        std::vector<Poly2> outs;
        for (const G::Polygon2& p : pieces) outs.push_back(p.outer);
        if (!outs.empty()) walkSoFar = G::differenceSets(walkSoFar, unionAll(outs));
        if (!K.empty()) walkSoFar = G::unionSets(walkSoFar, K);
    }
    auto onWalk = [&](const Poly2& o) {
        return G::setArea(G::intersectSets(G::offsetSet(G::fromRing(o), 0.3), walkSoFar)) > 0.3 * 1.6;
    };
    const Real deep = 2 * zp.windowDepth + 1.0;   // two rooms back to back
    int group = 0;
    std::function<void(const Poly2&, int, uint32_t)> bisect = [&](const Poly2& o, int depth, uint32_t h) {
        const Real A = area(o);
        if (A < 4.0) return;   // a scrap: floor to walk on
        const OBB2 ob = orientedBoundingBox(o);
        const int la = ob.longAxis();
        const Real longSide = 2 * ob.half[la], shortSide = 2 * ob.half[1 - la];
        auto leaf = [&]() {
            if (shortSide < zp.minSide * 0.75) return;   // too thin for a room: floor to walk on
            FloorZone z;
            z.poly = o;
            ensureCCW(z.poly);
            z.site = ZoneSite::Interior;
            z.group = group;
            z.frame = inscribedRect(z.poly, ob.axis[la]);
            fp.zones.push_back(z);
        };
        if (depth > 12) { leaf(); return; }
        auto halves = [&](const Vec2& at, const Vec2& dir, Real gap, std::vector<Poly2>& out) {
            out.clear();
            const Vec2 n = perp(normalize(dir));
            for (bool left : {true, false}) {
                const Vec2 side = at + n * ((left ? 0.5 : -0.5) * gap);
                for (const G::Polygon2& q : G::intersectSets(G::fromRing(o), halfPlane(side, dir, left))) {
                    Poly2 r = q.outer;
                    ensureCCW(r);
                    if (area(r) > 0.5) out.push_back(r);
                }
            }
        };
        std::vector<Poly2> hs;
        if (shortSide > deep && longSide > 2 * zp.minSide) {
            halves(ob.center, ob.axis[la], zp.aisle, hs);
            Poly2 strip;   // the new aisle joins the circulation
            const Vec2 u = ob.axis[la], n = perp(u);
            strip = {ob.center - u * 4000.0 - n * (zp.aisle * 0.5), ob.center + u * 4000.0 - n * (zp.aisle * 0.5),
                     ob.center + u * 4000.0 + n * (zp.aisle * 0.5), ob.center - u * 4000.0 + n * (zp.aisle * 0.5)};
            ensureCCW(strip);
            walkSoFar = G::unionSets(walkSoFar, G::intersectSets(G::fromRing(strip), G::fromRing(o)));
            for (std::size_t k = 0; k < hs.size(); ++k) bisect(hs[k], depth + 1, mix(h + 1u + static_cast<uint32_t>(k)));
            return;
        }
        if (A <= zp.interiorLot * 1.6 || longSide < 2 * zp.minSide) { leaf(); return; }
        const Real jitter = ((h % 1000u) / 1000.0 - 0.5) * 0.2 * longSide;
        for (int tryAxis = 0; tryAxis < 2; ++tryAxis) {
            const int ax = tryAxis == 0 ? la : 1 - la;   // cut across this axis
            if (2 * ob.half[ax] < 2 * zp.minSide) continue;
            halves(ob.center + ob.axis[ax] * (tryAxis == 0 ? jitter : 0.0), ob.axis[1 - ax], 0.0, hs);
            bool ok = hs.size() >= 2;
            for (const Poly2& q : hs) ok = ok && onWalk(q);
            if (!ok) continue;
            for (std::size_t k = 0; k < hs.size(); ++k) bisect(hs[k], depth + 1, mix(h + 1u + static_cast<uint32_t>(k)));
            return;
        }
        leaf();
    };
    for (const G::Polygon2& p : pieces) {
        Poly2 o = p.outer;
        ensureCCW(o);
        bisect(o, 0, mix(static_cast<uint32_t>(params.seed) * 2654435761u + static_cast<uint32_t>(group)));
        ++group;
    }

    // a lot whose frame is too narrow for a desk and its chair is no room: floor to walk on
    fp.zones.erase(std::remove_if(fp.zones.begin(), fp.zones.end(), [](const FloorZone& z) {
                       return z.frame.size() != 4 ||
                              std::min((z.frame[1] - z.frame[0]).length(), (z.frame[3] - z.frame[0]).length()) < 2.0;
                   }), fp.zones.end());

    // what no zone covers is the floor's circulation
    std::vector<Poly2> zr;
    for (const FloorZone& z : fp.zones) zr.push_back(z.poly);
    // (lot by lot: the field less the union of a floor's lots in one difference came out wrong -- 106 m2 where the
    // same lots taken off one at a time leave the true 37 -- on a twisted tower's storey; geom2d owes a look)
    G::PolySet C = F;
    for (const Poly2& r : zr) C = G::differenceSets(C, G::fromRing(r));
    G::PolySet walkC = K.empty() ? C : G::unionSets(C, K);
    for (const G::Polygon2& p : C) {
        std::vector<Poly2> piece = {p.outer};
        for (const G::Ring& h : p.holes) piece.push_back(h);
        fp.circulation.push_back(piece);
    }
    const G::PreparedSet walk(walkC);
    for (FloorZone& z : fp.zones) {
        z.area = area(z.poly);
        if (z.frame.size() == 4) z.depth = (z.frame[3] - z.frame[0]).length();
        const Vec2 cz = centroid(z.poly);
        z.toLobby = (cz - fp.lobby).length();
        z.toCore = coreRing.empty() ? 1e9 : distToRing(cz, coreRing);
        for (std::size_t i = 0; i < z.poly.size(); ++i) {
            Real mid;
            z.accessLen = std::max(z.accessLen, accessRun(z.poly, i, walk, mid));
        }
    }
    // a window lot too narrow on the corridor joins a window lot beside it -- the one it shares the most wall with of
    // those that leave it wider on the corridor -- until it is wide enough (the end of a stadium: three wedges a metre
    // wide at the back are one flat; a rounded corner's two halves, which meet the corridor only at its corner, join
    // the flat beside them)
    for (bool merged = zp.minAccess > 0 || zp.minLotArea > 0; merged;) {
        merged = false;
        for (std::size_t a = 0; a < fp.zones.size() && !merged; ++a) {
            const FloorZone& A = fp.zones[a];
            const bool small = A.area < zp.minLotArea;
            if (A.site == ZoneSite::Interior || (A.accessLen >= zp.minAccess && !small)) continue;
            struct Cand { std::size_t b; Real gain; Poly2 poly; };
            std::vector<Cand> cands;
            for (std::size_t b = 0; b < fp.zones.size(); ++b) {
                if (b == a || fp.zones[b].site == ZoneSite::Interior) continue;
                const G::PolySet un = G::unionSets(G::fromRing(A.poly), G::fromRing(fp.zones[b].poly));
                if (un.size() != 1 || !un.front().holes.empty()) continue;
                const Real gain = perimeter(A.poly) + perimeter(fp.zones[b].poly) - perimeter(un.front().outer);
                if (gain > 0.5) cands.push_back({b, gain, un.front().outer});
            }
            std::sort(cands.begin(), cands.end(), [](const Cand& x, const Cand& y) { return x.gain > y.gain; });
            for (Cand& cd : cands) {
                const FloorZone& B = fp.zones[cd.b];
                if (area(cd.poly) > 1.6 * zp.lotWidth * zp.windowDepth) continue;   // no flat the size of a floor
                ensureCCW(cd.poly);
                FloorZone z = A.area > B.area ? A : B;
                const Vec2 ax = z.frame.size() == 4 ? z.frame[1] - z.frame[0] : Vec2(1, 0);
                z.poly = cd.poly;
                z.area = area(cd.poly);
                z.frontage = A.frontage + B.frontage;
                z.curved = A.curved || B.curved;
                z.site = A.site == ZoneSite::Corner || B.site == ZoneSite::Corner ? ZoneSite::Corner : ZoneSite::Window;
                z.frame = inscribedRect(cd.poly, ax);
                z.depth = z.frame.size() == 4 ? (z.frame[3] - z.frame[0]).length() : 0;
                z.toLobby = (centroid(cd.poly) - fp.lobby).length();
                z.toCore = coreRing.empty() ? 1e9 : distToRing(centroid(cd.poly), coreRing);
                z.accessLen = 0;
                for (std::size_t i = 0; i < cd.poly.size(); ++i) {
                    Real mid;
                    z.accessLen = std::max(z.accessLen, accessRun(cd.poly, i, walk, mid));
                }
                if (!small && z.accessLen < A.accessLen + 0.5) continue;   // no wider on the corridor than it was
                fp.zones[cd.b] = z;
                fp.zones.erase(fp.zones.begin() + static_cast<std::ptrdiff_t>(a));
                merged = true;
                break;
            }
        }
    }
    return fp;
}

// ------------------------------------------------------------------------------------------- the program
void FloorProgramBook::set(FloorProgram p) {
    for (FloorProgram& q : programs_)
        if (q.name == p.name) { q = std::move(p); return; }
    programs_.push_back(std::move(p));
}
const FloorProgram* FloorProgramBook::find(const std::string& name) const {
    for (const FloorProgram& q : programs_)
        if (q.name == name) return &q;
    return nullptr;
}
FloorProgramBook& FloorProgramBook::global() {
    static FloorProgramBook book;
    return book;
}

bool roomKindFromName(const std::string& name, RoomKind& out) {
    for (int k = 0; k <= static_cast<int>(kLastRoomKind); ++k) {
        std::string n = cellUseName(static_cast<RoomKind>(k));
        std::replace(n.begin(), n.end(), ' ', '_');
        if (n == name) { out = static_cast<RoomKind>(k); return true; }
    }
    return false;
}

// What assets/scripts/floor_programs.lua ships with (keep the two alike; the Lua wins once loaded).
FloorProgram builtinOfficeProgram() {
    FloorProgram p;
    p.name = "office";
    const uint8_t W = zoneSiteBit(ZoneSite::Window), C = zoneSiteBit(ZoneSite::Corner), I = zoneSiteBit(ZoneSite::Interior);
    auto rule = [&](const char* tag, RoomKind kind, uint8_t sites) {
        ZoneRule r;
        r.tag = tag;
        r.kind = kind;
        r.sites = sites;
        p.rules.push_back(r);
        return &p.rules.back();
    };
    ZoneRule* r = rule("reception", RoomKind::Reception, I | W);
    r->count = 1; r->near = 1; r->lifts = true; r->minArea = 12; r->maxArea = 60;
    r = rule("corner_office", RoomKind::Office, C);
    r->share = 1.0; r->enclosed = true; r->minArea = 8;
    r = rule("pantry", RoomKind::Kitchenette, I);
    r->count = 1; r->near = 2; r->minArea = 10; r->maxArea = 50;
    r = rule("copy", RoomKind::CopyRoom, I);
    r->count = 1; r->near = 2; r->enclosed = true; r->walls = 0; r->minArea = 6; r->maxArea = 28; r->minDepth = 2.0;
    r->minFloor = 700;
    r = rule("meeting", RoomKind::Meeting, I | W);
    r->perM2 = 320; r->enclosed = true; r->walls = 1; r->minArea = 10; r->maxArea = 45; r->minDepth = 3.0;
    r = rule("office", RoomKind::Office, W);
    r->share = 0.0; r->enclosed = true; r->near = 3; r->minArea = 8; r->maxArea = 30; r->minDepth = 3.0;
    r = rule("water_cooler", RoomKind::WaterCooler, I | W);
    r->count = 1; r->minArea = 6; r->maxArea = 30; r->minDepth = 2.0; r->minFloor = 500;
    r = rule("phone_booths", RoomKind::PhoneBooths, I);
    r->perM2 = 900; r->minArea = 6; r->maxArea = 30; r->minDepth = 2.2; r->minFloor = 900;
    r = rule("lounge", RoomKind::Lounge, W | I);
    r->count = 1; r->minArea = 12; r->minFloor = 800;
    rule("open_plan", RoomKind::OpenPlan, W | C | I);
    rule("desk_row", RoomKind::DeskRow, W | C | I);
    ZoneStyle s;
    s.name = "corner_offices"; s.weight = 2; p.styles.push_back(s);
    s = ZoneStyle{}; s.name = "perimeter_offices"; s.weight = 1; s.sets = {{"office", "share", 0.5}}; p.styles.push_back(s);
    s = ZoneStyle{}; s.name = "open_floor"; s.weight = 1; s.sets = {{"meeting", "per_m2", 500}}; p.styles.push_back(s);
    return p;
}

FloorProgram builtinApartmentProgram() {
    FloorProgram p;
    p.name = "apartments";
    p.residential = true;
    p.zones.windowDepth = 12.0;
    p.zones.minAccess = 4.0;
    p.zones.minLotArea = 30.0;
    p.zones.lotWidth = 8.5;
    p.zones.interiorLot = 20.0;
    p.fill = "landing";
    p.shallow = "landing";
    p.shallowDepth = 0;
    const uint8_t W = zoneSiteBit(ZoneSite::Window), C = zoneSiteBit(ZoneSite::Corner), I = zoneSiteBit(ZoneSite::Interior);
    auto rule = [&](const char* tag, RoomKind kind, uint8_t sites) {
        ZoneRule r;
        r.tag = tag;
        r.kind = kind;
        r.sites = sites;
        r.walls = 0;
        p.rules.push_back(r);
        return &p.rules.back();
    };
    ZoneRule* r = rule("flat", RoomKind::Flat, W | C);
    r->share = 1.0; r->enclosed = true; r->minArea = 30; r->minDepth = 4.5;
    r = rule("storage", RoomKind::Closet, I);
    r->share = 1.0; r->enclosed = true; r->minArea = 4; r->maxArea = 40;
    rule("landing", RoomKind::Hall, W | C | I);
    return p;
}

const FloorProgram& floorProgram(const std::string& name) {
    if (const FloorProgram* p = FloorProgramBook::global().find(name)) return *p;
    static const FloorProgram office = builtinOfficeProgram(), apartments = builtinApartmentProgram();
    return name == "apartments" ? apartments : office;
}

namespace {
void setRuleField(ZoneRule& r, const std::string& f, Real v) {
    if (f == "count") r.count = static_cast<int>(v);
    else if (f == "per_m2") r.perM2 = v;
    else if (f == "share") r.share = v;
    else if (f == "min_area") r.minArea = v;
    else if (f == "max_area") r.maxArea = v;
    else if (f == "min_depth") r.minDepth = v;
    else if (f == "min_floor") r.minFloor = v;
    else if (f == "enclosed") r.enclosed = v != 0;
}
int ruleIndex(const std::vector<ZoneRule>& rules, const std::string& tag) {
    for (std::size_t i = 0; i < rules.size(); ++i)
        if (rules[i].tag == tag) return static_cast<int>(i);
    return -1;
}
}  // namespace

ZoneTags tagZones(const FloorPartition& fp, const FloorProgram& progIn, uint32_t seed, bool lifts) {
    ZoneTags out;
    std::vector<ZoneRule> rules = progIn.rules;
    // the building's habit
    if (!progIn.styles.empty()) {
        Real total = 0;
        for (const ZoneStyle& s : progIn.styles) total += std::max(Real(0), s.weight);
        Real pick = (mix(seed * 2246822519u + 77u) % 100000u) / 100000.0 * total;
        const ZoneStyle* st = &progIn.styles.back();
        for (const ZoneStyle& s : progIn.styles) {
            if (pick < s.weight) { st = &s; break; }
            pick -= s.weight;
        }
        out.style = st->name;
        for (const ZoneStyle::Set& set : st->sets) {
            const int ri = ruleIndex(rules, set.tag);
            if (ri >= 0) setRuleField(rules[static_cast<std::size_t>(ri)], set.field, set.value);
        }
    }
    const std::size_t nz = fp.zones.size();
    out.rule.assign(nz, -1);
    const int fill = ruleIndex(rules, progIn.fill), shallow = ruleIndex(rules, progIn.shallow);
    for (std::size_t ri = 0; ri < rules.size(); ++ri) {
        const ZoneRule& r = rules[ri];
        if (static_cast<int>(ri) == fill || static_cast<int>(ri) == shallow) continue;
        if (r.lifts && !lifts) continue;
        if (fp.fieldArea < r.minFloor) continue;
        std::vector<std::size_t> el;
        for (std::size_t z = 0; z < nz; ++z) {
            const FloorZone& Z = fp.zones[z];
            if (out.rule[z] >= 0 || !(r.sites & zoneSiteBit(Z.site))) continue;
            if (Z.area < r.minArea || Z.area > r.maxArea || Z.depth < r.minDepth || Z.frame.size() != 4) continue;
            if (r.enclosed && Z.accessLen < kDoorRun) continue;
            el.push_back(z);
        }
        int target = r.count > 0 ? r.count
                   : r.perM2 > 0 ? std::max(1, static_cast<int>(std::lround(fp.fieldArea / r.perM2)))
                   : r.share > 0 ? static_cast<int>(std::lround(r.share * static_cast<Real>(el.size()))) : 0;
        target = std::min(target, static_cast<int>(el.size()));
        if (target <= 0) continue;
        auto h = [&](std::size_t z) { return hashAt(seed ^ static_cast<uint32_t>(ri * 7919u), centroid(fp.zones[z].poly)); };
        if (r.near == 1 || r.near == 2 || r.near == 3) {
            std::sort(el.begin(), el.end(), [&](std::size_t a, std::size_t b) {
                const Real ka = r.near == 1 ? fp.zones[a].toLobby : r.near == 2 ? fp.zones[a].toCore : -fp.zones[a].toLobby;
                const Real kb = r.near == 1 ? fp.zones[b].toLobby : r.near == 2 ? fp.zones[b].toCore : -fp.zones[b].toLobby;
                if (std::fabs(ka - kb) > 1e-6) return ka < kb;
                return h(a) < h(b);
            });
            for (int k = 0; k < target; ++k) out.rule[el[static_cast<std::size_t>(k)]] = static_cast<int>(ri);
        } else {
            // spread out: the first by hash, then each the farthest from those picked
            std::vector<std::size_t> picked;
            std::sort(el.begin(), el.end(), [&](std::size_t a, std::size_t b) { return h(a) < h(b); });
            picked.push_back(el.front());
            while (static_cast<int>(picked.size()) < target) {
                std::size_t best = el.front();
                Real bd = -1;
                for (std::size_t z : el) {
                    if (std::find(picked.begin(), picked.end(), z) != picked.end()) continue;
                    Real d = 1e9;
                    for (std::size_t q : picked) d = std::min(d, (centroid(fp.zones[z].poly) - centroid(fp.zones[q].poly)).length());
                    if (d > bd) { bd = d; best = z; }
                }
                if (bd < 0) break;
                picked.push_back(best);
            }
            for (std::size_t z : picked) out.rule[z] = static_cast<int>(ri);
        }
    }
    for (std::size_t z = 0; z < nz; ++z)
        if (out.rule[z] < 0)
            out.rule[z] = shallow >= 0 && fp.zones[z].depth < progIn.shallowDepth ? shallow : fill;
    return out;
}

// ------------------------------------------------------------------------------------------- the floor
namespace {
// Add wall a->b, less whatever of it an existing wall already stands on (two enclosed rooms share a partition).
void addWall(std::vector<RoomWall>& walls, RoomWall w) {
    std::vector<std::pair<Real, Real>> keep = {{0, 1}};
    const Vec2 ab = w.b - w.a;
    const Real L = ab.length();
    if (L < 0.05) return;
    const Vec2 d = ab * (1.0 / L);
    for (const RoomWall& o : walls) {
        const Vec2 od = o.b - o.a;
        const Real oL = od.length();
        if (oL < 0.05 || std::fabs(cross(od * (1.0 / oL), d)) > 0.02) continue;
        if (std::fabs(cross(o.a - w.a, d)) > 0.04) continue;   // not on this line
        Real t0 = dot(o.a - w.a, d) / L, t1 = dot(o.b - w.a, d) / L;
        if (t0 > t1) std::swap(t0, t1);
        std::vector<std::pair<Real, Real>> next;
        for (const auto& k : keep) {
            if (t1 <= k.first || t0 >= k.second) { next.push_back(k); continue; }
            if (t0 > k.first) next.push_back({k.first, t0});
            if (t1 < k.second) next.push_back({t1, k.second});
        }
        keep.swap(next);
    }
    for (const auto& k : keep) {
        if ((k.second - k.first) * L < 0.1) continue;
        RoomWall piece = w;
        piece.a = w.a + ab * k.first;
        piece.b = w.a + ab * k.second;
        piece.doorAt = -1;
        if (w.doorAt >= 0 && w.doorAt >= k.first && w.doorAt <= k.second)
            piece.doorAt = (w.doorAt - k.first) / (k.second - k.first);
        walls.push_back(piece);
    }
}

// The pieces of wall w inside `poly` (its doorway kept where it falls in a piece long enough to carry it).
void clipWall(const RoomWall& w, const Poly2& poly, std::vector<RoomWall>& out) {
    const Vec2 ab = w.b - w.a;
    const Real L = ab.length();
    if (L < 0.1) return;
    std::vector<Real> ts = {0, 1};
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const Vec2 c = poly[i], d = poly[(i + 1) % poly.size()], cd = d - c;
        const Real den = cross(ab, cd);
        if (std::fabs(den) < 1e-12) continue;
        const Real t = cross(c - w.a, cd) / den, u = cross(c - w.a, ab) / den;
        if (t > 0 && t < 1 && u >= -1e-9 && u <= 1 + 1e-9) ts.push_back(t);
    }
    std::sort(ts.begin(), ts.end());
    for (std::size_t k = 0; k + 1 < ts.size(); ++k) {
        const Real t0 = ts[k], t1 = ts[k + 1];
        if ((t1 - t0) * L < 0.1 || !pointInPolygon(poly, w.a + ab * ((t0 + t1) * 0.5))) continue;
        RoomWall piece = w;
        piece.a = w.a + ab * t0;
        piece.b = w.a + ab * t1;
        piece.doorAt = -1;
        if (w.doorAt >= t0 && w.doorAt <= t1 && (t1 - t0) * L > kRoomDoorW + 0.6) piece.doorAt = (w.doorAt - t0) / (t1 - t0);
        out.push_back(piece);
    }
}

// ONE FLAT laid in `frame`, a rectangle of the lot square to its glass: the apartment layout (living room and bedrooms
// on the glass, the hall at the front door with the bath and kitchen beside it; room_plan.h layoutApartment).
bool layFlatIn(const FloorZone& Z, const Poly2& frame, const Vec2& door, std::vector<Room>& rooms, std::vector<RoomWall>& walls) {
    // laid in a rectangle of the lot square to its glass, so every room is a whole rectangle; then whatever of the
    // lot lies past it goes to the rooms beside it -- the end rooms stretched out to the lot's sides (a wedge's wide
    // glass), the glass rooms out to a curve's bulge, the back rooms on to the corridor -- and all of it cut to the lot
    const Vec2 O = frame[0], u = normalize(frame[1] - frame[0]), nIn = perp(u);
    const Real x0 = 0, x1 = (frame[1] - frame[0]).length(), v0 = 0, v1 = (frame[3] - frame[0]).length();
    if (x1 - x0 < 3.9 || v1 - v0 < 5.0) {
        if (std::getenv("RT_FLAT_DEBUG")) std::fprintf(stderr, "[flat] frame %.2f x %.2f\n", x1 - x0, v1 - v0);
        return false;
    }
    // the hall at the door; in a narrow flat, at the end of the back nearer it (the bath beside it, not squeezed out)
    Real xd = std::clamp(dot(door - O, u), x0 + 0.9, x1 - 0.9);
    if (x1 - x0 < 6.5) xd = xd < (x0 + x1) * 0.5 ? x0 + 0.8 : x1 - 0.8;
    RoomPlan lay;
    auto P = [&](Real x, Real v) { return O + u * x + nIn * v; };
    layoutApartment(lay, 0, P, x0, x1, v0, v1, v1 - v0, xd, false, false);
    bool bath = false, kitchen = false;
    for (const Room& r : lay.rooms) { bath = bath || r.kind == RoomKind::Bath; kitchen = kitchen || r.kind == RoomKind::Kitchen; }
    if (!bath) {
        if (std::getenv("RT_FLAT_DEBUG")) std::fprintf(stderr, "[flat] no bath in %.2f x %.2f\n", x1 - x0, v1 - v0);
        return false;
    }
    // too narrow for a kitchen of its own: a studio flat -- its hall and bath, and one room to live and sleep in with
    // the kitchenette along a wall (the `flat` furniture program)
    // ...and one with no bedroom sleeps in its living room: a studio too
    bool bed = false;
    for (const Room& r : lay.rooms) bed = bed || r.kind == RoomKind::Bed;
    if (!kitchen || !bed)
        for (Room& r : lay.rooms) if (r.kind == RoomKind::Living) r.kind = RoomKind::Flat;
    const Real far = 1000.0;
    // the back strip: rooms whose far side is the frame's back (they reach on to the corridor, not sideways); the
    // front rooms at either end take the lot beside the back strip as well (a wedge's wide end), cut round it
    const Real vb = [&] {
        Real m = v1;
        for (const Room& r : lay.rooms) {
            Real lo = 1e30, hi = -1e30;
            for (const Vec2& q : r.rect) { lo = std::min(lo, dot(q - O, nIn)); hi = std::max(hi, dot(q - O, nIn)); }
            if (std::fabs(hi - v1) < 1e-4 && lo > v0 + 1e-4) m = std::min(m, lo);
        }
        return m;
    }();
    auto isBack = [&](const Room& r) {
        Real lo = 1e30;
        for (const Vec2& q : r.rect) lo = std::min(lo, dot(q - O, nIn));
        return lo >= vb - 1e-4;
    };
    RoomKind backEnd[2] = {RoomKind::Hall, RoomKind::Hall}, frontEnd[2] = {RoomKind::Living, RoomKind::Living};
    for (const Room& r : lay.rooms) {
        Real lo = 1e30, hi = -1e30;
        for (const Vec2& q : r.rect) { lo = std::min(lo, dot(q - O, u)); hi = std::max(hi, dot(q - O, u)); }
        RoomKind* ends = isBack(r) ? backEnd : frontEnd;
        if (std::fabs(lo - x0) < 1e-4) ends[0] = r.kind;
        if (std::fabs(hi - x1) < 1e-4) ends[1] = r.kind;
    }
    G::PolySet backSet;
    std::vector<G::PolySet> sides(lay.rooms.size());   // a front end room's share of the lot beside the back strip
    for (std::size_t ri = 0; ri < lay.rooms.size(); ++ri) {
        Room& r = lay.rooms[ri];
        const bool back = isBack(r);
        for (Vec2& q : r.rect) {
            const Real x = dot(q - O, u), v = dot(q - O, nIn);
            Vec2 d(0, 0);
            if (!back && std::fabs(x - x0) < 1e-4) d = d - u * far;
            if (!back && std::fabs(x - x1) < 1e-4) d = d + u * far;
            if (std::fabs(v - v0) < 1e-4) d = d - nIn * far;
            if (!back) {
                if (std::fabs(x - x0) < 1e-4 && std::fabs(v - vb) < 1e-4) {
                    Poly2 sq = {P(x0 - far, vb), P(x0, vb), P(x0, v1), P(x0 - far, v1)};
                    ensureCCW(sq);
                    sides[ri] = G::fromRing(sq);
                }
                if (std::fabs(x - x1) < 1e-4 && std::fabs(v - vb) < 1e-4) {
                    Poly2 sq = {P(x1, vb), P(x1 + far, vb), P(x1 + far, v1), P(x1, v1)};
                    ensureCCW(sq);
                    sides[ri] = sides[ri].empty() ? G::fromRing(sq) : G::unionSets(sides[ri], G::fromRing(sq));
                }
            }
            q = q + d;
        }
        ensureCCW(r.rect);
        if (back) backSet = backSet.empty() ? G::fromRing(r.rect) : G::unionSets(backSet, G::fromRing(r.rect));
    }
    // THE FOYER: the lot past the frame's back, toward the corridor (a wedge's narrow end, beyond its largest
    // rectangle), is the hall's -- the way in from the front door; the private rooms beside it walled off from it
    {
        Poly2 beyond = {P(-far, v1), P(far, v1), P(far, v1 + far), P(-far, v1 + far)};
        ensureCCW(beyond);
        G::PolySet foyer = G::fromRing(beyond);
        // (a kitchen in the back strip, open to the hall, takes what lies behind it: a deeper kitchen, a smaller hall)
        for (Room& r : lay.rooms) {
            if (r.kind != RoomKind::Kitchen || !isBack(r)) continue;
            Real lo = 1e30, hi = -1e30;
            for (const Vec2& q : r.rect) { lo = std::min(lo, dot(q - O, u)); hi = std::max(hi, dot(q - O, u)); }
            Poly2 behind = {P(lo, v1), P(hi, v1), P(hi, v1 + far), P(lo, v1 + far)};
            ensureCCW(behind);
            const Poly2 whole = largestOuter(G::unionSets(G::fromRing(r.rect), G::fromRing(behind)));
            if (whole.size() >= 3) r.rect = whole;
            foyer = G::differenceSets(foyer, G::fromRing(behind));
        }
        for (Room& r : lay.rooms) {
            if (r.kind != RoomKind::Hall || !isBack(r)) continue;
            const Poly2 whole = largestOuter(G::unionSets(G::fromRing(r.rect), foyer));
            if (whole.size() >= 3) r.rect = whole;
            break;
        }
        for (const Room& r : lay.rooms) {
            if (!isBack(r) || (r.kind != RoomKind::Bath && r.kind != RoomKind::Closet)) continue;
            Real lo = 1e30, hi = -1e30;
            for (const Vec2& q : r.rect) { lo = std::min(lo, dot(q - O, u)); hi = std::max(hi, dot(q - O, u)); }
            RoomWall w;
            w.a = P(lo, v1);
            w.b = P(hi, v1);
            lay.walls.push_back(w);
        }
    }
    for (RoomWall& w : lay.walls) {
        const Vec2 door0 = w.a + (w.b - w.a) * std::max(Real(0), w.doorAt);
        const bool alongU = std::fabs(dot(w.b - w.a, nIn)) < 1e-4;
        for (Vec2* q : {&w.a, &w.b}) {
            const Real x = dot(*q - O, u), v = dot(*q - O, nIn);
            const bool divider = alongU && std::fabs(v - vb) < 1e-4;   // (stays the back strip's width)
            if (!divider && std::fabs(x - x0) < 1e-4) *q = *q - u * far;
            if (!divider && std::fabs(x - x1) < 1e-4) *q = *q + u * far;
            if (!alongU && std::fabs(v - v0) < 1e-4) *q = *q - nIn * far;
        }
        if (w.doorAt >= 0) w.doorAt = dot(door0 - w.a, normalize(w.b - w.a)) / std::max(Real(1e-6), (w.b - w.a).length());
    }
    // the back strip's ends, where the end rooms beside it are private (a bedroom in front, a bath or closet behind)
    for (int e = 0; e < 2; ++e) {
        const bool priv = frontEnd[e] == RoomKind::Bed || backEnd[e] == RoomKind::Bath || backEnd[e] == RoomKind::Closet;
        if (!priv) continue;
        RoomWall w;
        w.a = P(e == 0 ? x0 : x1, vb);
        w.b = P(e == 0 ? x0 : x1, v1);
        lay.walls.push_back(w);
    }
    // the front end rooms with the lot beside the back strip
    for (std::size_t ri = 0; ri < lay.rooms.size(); ++ri) {
        if (sides[ri].empty()) continue;
        Room& r = lay.rooms[ri];
        const G::PolySet un = G::unionSets(G::fromRing(r.rect), sides[ri]);
        const Poly2 whole = largestOuter(un);
        if (whole.size() >= 3) r.rect = whole;
    }
    std::vector<Room> rs;
    for (Room r : lay.rooms) {
        ensureCCW(r.rect);
        const Poly2 cut = largestOuter(G::intersectSets(G::fromRing(r.rect), G::fromRing(Z.poly)));
        if (cut.size() < 3 || area(cut) < 1.5) continue;
        r.rect = cut;
        ensureCCW(r.rect);
        r.frame = inscribedRect(r.rect, u);
        // a room cut too small to furnish (a bath a wedge squeezed to a metre): no flat, a studio
        const Real fw = r.frame.size() == 4 ? std::min((r.frame[1] - r.frame[0]).length(), (r.frame[3] - r.frame[0]).length()) : 0;
        const Real need = r.kind == RoomKind::Living ? 2.6 : r.kind == RoomKind::Bed ? 2.4 : r.kind == RoomKind::Hall ? 1.0 : 1.8;
        if (fw < need) {
            if (std::getenv("RT_FLAT_DEBUG")) std::fprintf(stderr, "[flat] a %s cut to %.2f (needs %.2f)\n", cellUseName(r.kind), fw, need);
            return false;
        }
        rs.push_back(r);
    }
    // a hall that is most of the flat is no layout (a frame far from the corridor leaves it all to the foyer)
    Real hallA = 0, allA = 0;
    for (const Room& r : rs) { allA += area(r.rect); if (r.kind == RoomKind::Hall) hallA += area(r.rect); }
    if (hallA > std::max(Real(8.0), 0.28 * allA)) {
        if (std::getenv("RT_FLAT_DEBUG")) std::fprintf(stderr, "[flat] hall %.1f of %.1f m2\n", hallA, allA);
        return false;
    }
    std::vector<RoomWall> ws;
    for (const RoomWall& w : lay.walls) clipWall(w, Z.poly, ws);
    // walk it from inside the front door
    RoomPlan check;
    check.rooms = rs;
    check.walls = ws;
    Vec2 in = Vec2(0, 0);
    {
        Real bd = 1e9;
        for (std::size_t i = 0; i < Z.poly.size(); ++i) {
            const Vec2 a = Z.poly[i], b = Z.poly[(i + 1) % Z.poly.size()];
            const Real d = distToSegment(door, a, b);
            if (d < bd && (b - a).length() > 1e-6) { bd = d; in = perp(normalize(b - a)); }
        }
    }
    if (rs.empty() || !floorIsWalkable(check, Z.poly, door + in * 0.6)) {
        if (std::getenv("RT_FLAT_DEBUG")) std::fprintf(stderr, "[flat] %zu rooms do not walk\n", rs.size());
        return false;
    }
    rooms.insert(rooms.end(), rs.begin(), rs.end());
    walls.insert(walls.end(), ws.begin(), ws.end());
    return true;
}

// A FLAT in its lot (a window or corner lot reaching the corridor), laid out inside -- a wedge on a drum, a trapezoid
// at a chamfer included. Tried in the lot's frame (its largest rectangle) and in the widest rectangles 6..13 m deep off
// its glass (a wedge's largest rectangle is narrow and deep; the flat wants the wide glass for its bedroom); the one
// that lives best kept. Its rooms and inner walls; false (nothing added) when none walks from the front door: the lot
// is a studio.
bool layFlat(const FloorZone& Z, const Vec2& door, std::vector<Room>& rooms, std::vector<RoomWall>& walls) {
    if (Z.frame.size() != 4) return false;
    std::vector<Poly2> frames = {Z.frame};
    const Vec2 O = Z.frame[0], u = normalize(Z.frame[1] - Z.frame[0]), nIn = perp(u);
    for (Real D : {6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0}) {
        Poly2 half = {O - u * 1000.0 - nIn * 1000.0, O + u * 1000.0 - nIn * 1000.0, O + u * 1000.0 + nIn * D, O - u * 1000.0 + nIn * D};
        ensureCCW(half);
        const Poly2 part = largestOuter(G::intersectSets(G::fromRing(Z.poly), G::fromRing(half)));
        if (part.size() < 3) continue;
        const Poly2 f = inscribedRect(part, u);
        if (f.size() == 4) frames.push_back(f);
    }
    // each laid and scored: the floor it gives to living (a bedroom, a kitchen of its own) over the hall it needs
    Real bestScore = -1e30;
    std::vector<Room> bestRooms;
    std::vector<RoomWall> bestWalls;
    for (const Poly2& f : frames) {
        std::vector<Room> rs;
        std::vector<RoomWall> ws;
        if (!layFlatIn(Z, f, door, rs, ws)) continue;
        Real score = 0;
        for (const Room& r : rs) {
            const Real a = area(r.rect);
            if (r.kind == RoomKind::Hall) score -= 0.5 * a;
            else if (r.kind == RoomKind::Bed) score += a + 25.0;
            else if (r.kind == RoomKind::Kitchen) score += a + 15.0;
            else if (r.kind == RoomKind::Living || r.kind == RoomKind::Flat) score += a;
        }
        if (score > bestScore) { bestScore = score; bestRooms = rs; bestWalls = ws; }
    }
    if (bestScore <= -1e29) return false;
    rooms.insert(rooms.end(), bestRooms.begin(), bestRooms.end());
    walls.insert(walls.end(), bestWalls.begin(), bestWalls.end());
    return true;
}
}  // namespace

RoomPlan zonedFloorPlan(const Poly2& plate, const BuildingParams& params, const CorePlan& core, Real inset,
                        int storey, bool lifts, const Vec2& entry, const Poly2& stairWell, const FloorProgram& prog) {
    RoomPlan rp;
    rp.topology = prog.residential ? PlateTopology::Apartments : PlateTopology::Ring;
    rp.office = !prog.residential;
    if (storey < 1) return rp;
    Poly2 plan = plate;
    ensureCCW(plan);
    FloorPartition fp = partitionFloor(plan, core, inset, params, prog.zones, core.valid ? Poly2{} : stairWell);
    if (fp.zones.empty()) return rp;
    const ZoneTags tags = tagZones(fp, prog, static_cast<uint32_t>(params.seed), lifts);
    std::vector<int> rule = tags.rule;

    // neighbouring OPEN lots of one tag along one chord (or out of one court piece) are one zone when together
    // they are still a rectangle: a run of open plan along the glass, not a desk pod per bay
    for (bool merged = true; merged;) {
        merged = false;
        for (std::size_t a = 0; a < fp.zones.size() && !merged; ++a) {
            const ZoneRule& ra = prog.rules[static_cast<std::size_t>(rule[a])];
            if (ra.enclosed) continue;
            for (std::size_t b = a + 1; b < fp.zones.size() && !merged; ++b) {
                if (rule[b] != rule[a] || fp.zones[b].group != fp.zones[a].group || fp.zones[b].site != fp.zones[a].site) continue;
                const G::PolySet u = G::unionSets(G::fromRing(fp.zones[a].poly), G::fromRing(fp.zones[b].poly));
                if (u.size() != 1 || !u.front().holes.empty()) continue;
                Poly2 up = u.front().outer;
                ensureCCW(up);
                const OBB2 ob = orientedBoundingBox(up);
                if (area(up) < 0.95 * 4 * ob.half[0] * ob.half[1]) continue;
                FloorZone z = fp.zones[a];
                z.poly = up;
                z.area = area(up);
                z.frontage += fp.zones[b].frontage;
                const Vec2 ax = fp.zones[a].frame.size() == 4 ? normalize(fp.zones[a].frame[1] - fp.zones[a].frame[0]) : ob.axis[ob.longAxis()];
                z.frame = inscribedRect(up, ax);
                z.accessLen = std::max(fp.zones[a].accessLen, fp.zones[b].accessLen);
                fp.zones[a] = z;
                fp.zones.erase(fp.zones.begin() + static_cast<std::ptrdiff_t>(b));
                rule.erase(rule.begin() + static_cast<std::ptrdiff_t>(b));
                merged = true;
            }
        }
    }

    const G::PolySet IN = G::offsetSet(G::fromRing(plan), -inset);
    const G::PreparedSet inside(IN);
    G::PolySet circSet;
    for (const std::vector<Poly2>& piece : fp.circulation) {
        G::Polygon2 p;
        p.outer = piece.front();
        p.holes.assign(piece.begin() + 1, piece.end());
        circSet.push_back(p);
    }
    if (core.valid) {
        const Vec2 u = core.frame.u, v = core.frame.v, o = core.frame.origin;
        const Real c = kCorridorRing;
        Poly2 ring = {o - u * c - v * c, o + u * (core.length + c) - v * c,
                      o + u * (core.length + c) + v * (core.depth + c), o - u * c + v * (core.depth + c)};
        ensureCCW(ring);
        circSet = G::unionSets(circSet, G::fromRing(ring));
    }
    const G::PreparedSet circ(circSet);
    const int fillRule = ruleIndex(prog.rules, prog.fill);
    const RoomKind fillKind = fillRule >= 0 ? prog.rules[static_cast<std::size_t>(fillRule)].kind : RoomKind::OpenPlan;

    // three tries: as tagged; the enclosed rooms whose only way in is through another room opened up; no walls (not
    // in homes: a floor of flats with no walls is no floor of flats, and the planner's older floors take it)
    bool walked = false;
    int flats = 0, studios = 0;
    for (int attempt = 0; attempt < (prog.residential ? 2 : 3); ++attempt) {
        flats = studios = 0;
        rp.rooms.clear();
        rp.walls.clear();
        std::vector<Poly2> open;
        for (std::size_t z = 0; z < fp.zones.size(); ++z)
            if (!prog.rules[static_cast<std::size_t>(rule[z])].enclosed || attempt == 2) open.push_back(fp.zones[z].poly);
        const G::PreparedSet walk(open.empty() ? circSet : G::unionSets(circSet, unionAll(open)));
        for (std::size_t z = 0; z < fp.zones.size(); ++z) {
            FloorZone& Z = fp.zones[z];
            const ZoneRule& R = prog.rules[static_cast<std::size_t>(rule[z])];
            Room rm;
            rm.kind = R.kind;
            rm.rect = Z.poly;
            rm.edge = static_cast<std::size_t>(std::max(0, Z.group));
            const bool enclosed = R.enclosed && attempt < 2;
            // the door: the longest clear run onto the corridors; failing that (first try only), onto an open zone
            // (a copy room beside the reception opens onto the corridor, not through the reception)
            std::size_t doorEdge = Z.poly.size();
            Real doorMid = 0.5, best = 0;
            for (int via = 0; enclosed && via < (attempt == 0 ? 2 : 1) && best < kDoorRun; ++via) {
                best = 0;
                doorEdge = Z.poly.size();
                for (std::size_t i = 0; i < Z.poly.size(); ++i) {
                    Real mid;
                    const Real run = accessRun(Z.poly, i, via == 0 ? circ : walk, mid);
                    // ...where the room is deep behind it (not a sliver's edge: the doorway must open INTO the room)
                    const Vec2 a = Z.poly[i], b = Z.poly[(i + 1) % Z.poly.size()];
                    const Vec2 d = normalize(b - a), out(d.y, -d.x);
                    if (!pointInPolygon(Z.poly, a + (b - a) * mid - out * 0.6)) continue;
                    if (run > best) { best = run; doorMid = mid; doorEdge = i; }
                }
            }
            // A HOME'S FLAT is laid out inside, its front door tried at the middle of its run onto the corridor and
            // at either end of it (a narrow back wants the hall at one end, the bath beside it); else it stays one
            // room, a studio
            std::vector<Room> flatRooms;
            std::vector<RoomWall> flatWalls;
            bool flatLaid = false;
            if (prog.residential && R.kind == RoomKind::Flat && enclosed && best >= kDoorRun && doorEdge < Z.poly.size()) {
                const Vec2 ea = Z.poly[doorEdge], eb = Z.poly[(doorEdge + 1) % Z.poly.size()];
                const Real EL = std::max(Real(1e-6), (eb - ea).length()), m = (kRoomDoorW * 0.5 + 0.35) / EL;
                const Real tLo = doorMid - best / (2 * EL) + m, tHi = doorMid + best / (2 * EL) - m;
                std::vector<Real> tries = {doorMid};
                if (tHi > tLo) { tries.push_back(tLo); tries.push_back(tHi); }
                for (Real t : tries) {
                    flatRooms.clear();
                    flatWalls.clear();
                    if (layFlat(Z, ea + (eb - ea) * t, flatRooms, flatWalls)) { doorMid = t; flatLaid = true; break; }
                }
            }
            // the frame: side 0 the glass for a window lot; else the side away from the door
            Poly2 fr = Z.frame;
            if (Z.site == ZoneSite::Interior && fr.size() == 4 && doorEdge < Z.poly.size()) {
                const Vec2 dp = Z.poly[doorEdge] + (Z.poly[(doorEdge + 1) % Z.poly.size()] - Z.poly[doorEdge]) * doorMid;
                std::size_t k = 0;
                Real bd = 1e9;
                for (std::size_t s = 0; s < 4; ++s) {   // the side nearest the door becomes side 2
                    const Real d = distToSegment(dp, fr[(s + 2) % 4], fr[(s + 3) % 4]);
                    if (d < bd) { bd = d; k = s; }
                }
                Poly2 r(4);
                for (std::size_t s = 0; s < 4; ++s) r[s] = fr[(s + k) % 4];
                fr = r;
            }
            rm.frame = fr;
            if (enclosed && std::getenv("RT_ZONE_DEBUG"))
                std::fprintf(stderr, "[zones] storey %d try %d %s at (%.2f %.2f): door edge %zu of %zu, run %.2f mid %.2f\n", storey, attempt,
                             cellUseName(R.kind), centroid(Z.poly).x, centroid(Z.poly).y, doorEdge, Z.poly.size(), best, doorMid);
            if (enclosed && best < kDoorRun) {   // no way in: it stays open floor
                rm.kind = fillKind;
                rp.rooms.push_back(rm);
                continue;
            }
            rp.rooms.push_back(rm);
            if (!enclosed) continue;
            const bool glass = R.walls == 1 || (R.walls == 2 && params.curtainWall);
            for (std::size_t i = 0; i < Z.poly.size(); ++i) {
                const Vec2 a = Z.poly[i], b = Z.poly[(i + 1) % Z.poly.size()];
                const Vec2 ab = b - a;
                const Real L = ab.length();
                if (L < 0.1) continue;
                const Vec2 d = ab * (1.0 / L), out(d.y, -d.x);
                if (!inside.contains((a + b) * 0.5 + out * 0.15)) continue;   // the outside wall: the facade's own
                RoomWall w;
                w.a = a;
                w.b = b;
                w.glass = glass;
                w.doorAt = i == doorEdge ? doorMid : -1;
                addWall(rp.walls, w);
            }
            // (the doorway's piece of wall must still be long enough to carry it, once shared stretches are taken off)
            bool doorCut = false;
            for (const RoomWall& w : rp.walls)
                if (w.doorAt >= 0 && (w.b - w.a).length() > kRoomDoorW + 0.6 &&
                    distToSegment(Z.poly[doorEdge] + (Z.poly[(doorEdge + 1) % Z.poly.size()] - Z.poly[doorEdge]) * doorMid, w.a, w.b) < 0.05)
                    doorCut = true;
            if (!doorCut && std::getenv("RT_ZONE_DEBUG"))
                std::fprintf(stderr, "[zones] storey %d: a %s's doorway was lost to a shared wall\n", storey, cellUseName(R.kind));
            if (flatLaid) {
                rp.rooms.pop_back();
                for (Room& r : flatRooms) { r.edge = rm.edge; rp.rooms.push_back(r); }
                for (const RoomWall& w : flatWalls) addWall(rp.walls, w);
                ++flats;
            } else if (prog.residential && R.kind == RoomKind::Flat) {
                ++studios;
            }
        }
        if (floorIsWalkable(rp, plan, entry, stairWell)) { walked = true; break; }
        if (std::getenv("RT_ZONE_DEBUG"))
            std::fprintf(stderr, "[zones] storey %d: try %d not walkable (%zu rooms, %zu walls)\n", storey, attempt,
                         rp.rooms.size(), rp.walls.size());
    }
    if (std::getenv("RT_ZONE_DEBUG")) {
        std::fprintf(stderr, "[zones] storey %d: core %d at", storey, core.valid ? 1 : 0);
        for (const Vec2& q : core.rect()) std::fprintf(stderr, " (%.2f %.2f)", q.x, q.y);
        std::fprintf(stderr, "; stair well");
        for (const Vec2& q : stairWell) std::fprintf(stderr, " (%.2f %.2f)", q.x, q.y);
        std::fprintf(stderr, "\n");
        std::fprintf(stderr, "[zones] storey %d: %zu zones, style %s, field %.0f m2:", storey, rp.rooms.size(),
                     tags.style.c_str(), fp.fieldArea);
        for (const Room& r : rp.rooms) std::fprintf(stderr, " %s", cellUseName(r.kind));
        std::fprintf(stderr, "\n");
        if (prog.residential) std::fprintf(stderr, "[zones] storey %d: %d flats, %d studios, walked %d\n", storey, flats, studios, walked ? 1 : 0);
    }
    // (a floor of homes with no flat on it -- every lot too small or shut off -- is left to the older floors too)
    if (prog.residential && (!walked || flats + studios == 0) && !std::getenv("RT_ZONE_KEEP_FAILED")) {
        RoomPlan none;
        none.topology = rp.topology;
        none.office = false;
        return none;
    }
    return rp;
}

}  // namespace engine
