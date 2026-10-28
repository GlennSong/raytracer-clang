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
    G::PolySet KW;   // what of K is floor to walk on: the corridor ring with its core, or in lobby mode the lobby alone
    Poly2 coreRing;
    if (core.valid) {
        const Vec2 u = core.frame.u, v = core.frame.v;
        const Vec2 o = core.frame.origin;
        const Real c = zp.lobbyOnly ? 0.0 : kCorridorRing;
        coreRing = {o - u * c - v * c, o + u * (core.length + c) - v * c, o + u * (core.length + c) + v * (core.depth + c),
                    o - u * c + v * (core.depth + c)};
        ensureCCW(coreRing);
        K = G::fromRing(coreRing);
        if (zp.lobbyOnly) {   // the lobby: in front of the lift doors (the core's v = 0 face) out to the glass
            Poly2 lobby = {o - v * 400.0, o + u * core.length - v * 400.0, o + u * core.length, o};
            ensureCCW(lobby);
            KW = G::intersectSets(G::fromRing(lobby), IN);
            K = G::unionSets(K, KW);
            for (const G::Polygon2& q : KW) fp.lobbyArea.push_back(q.outer);
            fp.lobbyMode = true;
        } else {
            KW = K;
        }
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
    if (core.valid && !zp.lobbyOnly && !I.empty()) {   // (lobby mode has no corridor to run on)
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
        if (!KW.empty()) walkSoFar = G::unionSets(walkSoFar, KW);
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
    if (zp.lobbyOnly && !KW.empty() && !C.empty()) {
        // THE WAY IN, AND ONLY IT: with no corridor round the core, what the lots leave can be a pocket cut off from
        // the lobby, or one reached only through a slit beside the core -- a door onto it opens onto nowhere. The
        // floor's circulation is what reaches the lobby at least a doorway wide (C opened by its half-width); a scrap
        // left by the rounding of its corners stays with it, and a pocket joins the lot it shares the most edge with.
        constexpr Real kHalfWay = 0.4;
        const G::PolySet opened = G::offsetSet(G::offsetSet(C, -kHalfWay), kHalfWay);
        const G::PolySet nearLobby = G::offsetSet(KW, 0.1);
        G::PolySet reach;
        for (const G::Polygon2& q : opened)
            if (G::setArea(G::intersectSets(G::PolySet{q}, nearLobby)) > 0.05) reach.push_back(q);
        const G::PolySet rest = reach.empty() ? C : G::differenceSets(C, reach);
        const G::PolySet nearReach = G::offsetSet(reach, 0.05);
        for (const G::Polygon2& q : rest) {
            const Real a = ringAreaAbs(q.outer);
            if (a < 1.0 && G::setArea(G::intersectSets(G::PolySet{q}, nearReach)) > 0.0) {
                reach = G::unionSets(reach, G::PolySet{q});   // a corner's scrap: still the way
                continue;
            }
            Poly2 pocket = q.outer;
            ensureCCW(pocket);
            std::size_t best = fp.zones.size();
            Real bestGain = 0.3;
            Poly2 bestPoly;
            for (std::size_t b = 0; b < fp.zones.size(); ++b) {
                const G::PolySet un = G::unionSets(G::fromRing(pocket), G::fromRing(fp.zones[b].poly));
                if (un.size() != 1 || !un.front().holes.empty()) continue;
                const Real gain = perimeter(pocket) + perimeter(fp.zones[b].poly) - perimeter(un.front().outer);
                if (gain > bestGain) { bestGain = gain; best = b; bestPoly = un.front().outer; }
            }
            if (best >= fp.zones.size()) continue;   // (shut in by the core and the facade alone: dead floor)
            FloorZone& B = fp.zones[best];
            const Vec2 ax = B.frame.size() == 4 ? B.frame[1] - B.frame[0] : Vec2(1, 0);
            ensureCCW(bestPoly);
            B.poly = bestPoly;
            B.frame = inscribedRect(B.poly, ax);
        }
        C = reach;
    }
    G::PolySet walkC = KW.empty() ? C : G::unionSets(C, KW);
    for (const G::Polygon2& p : C) {
        std::vector<Poly2> piece = {p.outer};
        for (const G::Ring& h : p.holes) piece.push_back(h);
        fp.circulation.push_back(piece);
    }
    G::PreparedSet walk(walkC);   // (a stub of corridor cut below joins it)
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
    // those that together are wide enough there (a rounded corner's two halves, which meet the corridor only at its
    // corner, join the flat beside them)
    // a window lot made of two (or of one and half of another): its facts again
    auto joined = [&](Poly2 poly, const FloorZone& A, const FloorZone& B) {
        ensureCCW(poly);
        FloorZone z = A.area > B.area ? A : B;
        const Vec2 ax = z.frame.size() == 4 ? z.frame[1] - z.frame[0] : Vec2(1, 0);
        z.poly = poly;
        z.area = area(poly);
        z.frontage = A.frontage + B.frontage;
        z.curved = A.curved || B.curved;
        z.site = A.site == ZoneSite::Corner || B.site == ZoneSite::Corner ? ZoneSite::Corner : ZoneSite::Window;
        z.frame = inscribedRect(poly, ax);
        z.depth = z.frame.size() == 4 ? (z.frame[3] - z.frame[0]).length() : 0;
        z.toLobby = (centroid(poly) - fp.lobby).length();
        z.toCore = coreRing.empty() ? 1e9 : distToRing(centroid(poly), coreRing);
        z.accessLen = 0;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            Real mid;
            z.accessLen = std::max(z.accessLen, accessRun(poly, i, walk, mid));
        }
        return z;
    };
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
                if (area(cd.poly) > 1.8 * zp.lotWidth * zp.windowDepth) continue;   // no flat the size of a floor
                const FloorZone z = joined(cd.poly, A, B);
                if (!small && z.accessLen < zp.minAccess) continue;   // still too narrow on the corridor

                fp.zones[cd.b] = z;
                fp.zones.erase(fp.zones.begin() + static_cast<std::ptrdiff_t>(a));
                merged = true;
                break;
            }
        }
    }
    // ...and a RUN of lots still too narrow there (a stadium's end: its wedges meet the corridor in a point) is taken
    // whole, halved down its middle -- the line from the core out through it -- and each half goes to the flat on its
    // side: the end's two flats share its arc and meet on the centreline
    if (zp.minAccess > 0) {
        const Vec2 hub = coreRing.empty() ? centroid(plate) : centroid(coreRing);
        std::vector<char> seen(fp.zones.size(), 0);
        auto narrow = [&](std::size_t k) { return fp.zones[k].site != ZoneSite::Interior && fp.zones[k].accessLen < zp.minAccess; };
        auto touches = [&](const Poly2& p0, const Poly2& p1) {
            const G::PolySet un = G::unionSets(G::fromRing(p0), G::fromRing(p1));
            return un.size() == 1 && un.front().holes.empty() && perimeter(p0) + perimeter(p1) - perimeter(un.front().outer) > 0.5;
        };
        std::vector<std::size_t> drop;
        for (std::size_t s0 = 0; s0 < fp.zones.size(); ++s0) {
            if (seen[s0] || !narrow(s0)) continue;
            std::vector<std::size_t> run = {s0};
            seen[s0] = 1;
            for (std::size_t q = 0; q < run.size(); ++q)
                for (std::size_t k = 0; k < fp.zones.size(); ++k)
                    if (!seen[k] && narrow(k) && touches(fp.zones[run[q]].poly, fp.zones[k].poly)) { seen[k] = 1; run.push_back(k); }
            std::vector<Poly2> parts;
            for (std::size_t k : run) parts.push_back(fp.zones[k].poly);
            const G::PolySet R = unionAll(parts);
            if (R.size() != 1 || !R.front().holes.empty()) continue;
            const Poly2 whole = R.front().outer;
            const Vec2 c = centroid(whole), out = c - hub;
            if (out.length() < 1e-3) continue;
            std::size_t to[2] = {fp.zones.size(), fp.zones.size()};
            Poly2 un[2];
            bool ok = true;
            for (int h = 0; h < 2 && ok; ++h) {
                const Poly2 half = largestOuter(G::intersectSets(G::fromRing(whole), halfPlane(c, out, h == 0)));
                if (half.size() < 3) { ok = false; break; }
                Real bestGain = 0.5;
                for (std::size_t b = 0; b < fp.zones.size(); ++b) {
                    if (narrow(b) || fp.zones[b].site == ZoneSite::Interior || (h == 1 && b == to[0])) continue;
                    if (std::find(drop.begin(), drop.end(), b) != drop.end()) continue;
                    const G::PolySet u2 = G::unionSets(G::fromRing(half), G::fromRing(fp.zones[b].poly));
                    if (u2.size() != 1 || !u2.front().holes.empty()) continue;
                    if (area(u2.front().outer) > 2.2 * zp.lotWidth * zp.windowDepth) continue;
                    const Real gain = perimeter(half) + perimeter(fp.zones[b].poly) - perimeter(u2.front().outer);
                    if (gain > bestGain) { bestGain = gain; to[h] = b; un[h] = u2.front().outer; }
                }
                ok = to[h] < fp.zones.size();
            }
            if (!ok) {
                // too much for the flats beside it: a STUB of corridor runs out along the middle from the core's side,
                // stopping a room short of the glass, and the run is two flats either side of it, their doors on it,
                // meeting on the centreline beyond its end
                // square to the building (the core's axes): a hallway runs straight, never off at a slant
                Vec2 d = normalize(out);
                if (core.valid) {
                    const Vec2 axes[4] = {core.frame.u, core.frame.u * -1.0, core.frame.v, core.frame.v * -1.0};
                    Vec2 bestAx = axes[0];
                    for (const Vec2& ax : axes) if (dot(ax, d) > dot(bestAx, d)) bestAx = ax;
                    d = normalize(bestAx);
                }
                Real tIn = 1e30, tOut = -1e30;
                for (const Vec2& q : whole) { tIn = std::min(tIn, dot(q - c, d)); tOut = std::max(tOut, dot(q - c, d)); }
                const Real t1 = tOut - 6.0, w = zp.aisle * 0.5;
                if (t1 - tIn < 3.0) continue;
                const Vec2 n = perp(d), a0 = c + d * (tIn - 1.0), a1 = c + d * t1;
                Poly2 stub = {a0 - n * w, a1 - n * w, a1 + n * w, a0 + n * w};
                ensureCCW(stub);
                const G::PolySet rest = G::differenceSets(G::fromRing(whole), G::fromRing(stub));
                const G::PreparedSet walk2(G::unionSets(walkC, G::fromRing(stub)));
                std::vector<FloorZone> halves;
                for (int h = 0; h < 2; ++h) {
                    const Poly2 hp = largestOuter(G::intersectSets(rest, halfPlane(c, d, h == 0)));
                    if (hp.size() < 3 || area(hp) < zp.minLotArea) break;
                    FloorZone z = fp.zones[s0];
                    z.poly = hp;
                    z.onStub = true;
                    ensureCCW(z.poly);
                    z.area = area(z.poly);
                    z.frontage = 0;
                    for (std::size_t k : run) z.frontage += fp.zones[k].frontage * 0.5;
                    const Vec2 ax = z.frame.size() == 4 ? z.frame[1] - z.frame[0] : n;
                    z.frame = inscribedRect(z.poly, ax);
                    z.depth = z.frame.size() == 4 ? (z.frame[3] - z.frame[0]).length() : 0;
                    z.toLobby = (centroid(z.poly) - fp.lobby).length();
                    z.toCore = coreRing.empty() ? 1e9 : distToRing(centroid(z.poly), coreRing);
                    z.accessLen = 0;
                    for (std::size_t i = 0; i < z.poly.size(); ++i) {
                        Real mid;
                        z.accessLen = std::max(z.accessLen, accessRun(z.poly, i, walk2, mid));
                    }
                    halves.push_back(z);
                }
                if (halves.size() != 2) continue;
                fp.circulation.push_back({stub});
                walkC = G::unionSets(walkC, G::fromRing(stub));
                walk = G::PreparedSet(walkC);
                for (FloorZone& z : halves) fp.zones.push_back(z);
                seen.resize(fp.zones.size(), 1);
                drop.insert(drop.end(), run.begin(), run.end());
                continue;
            }
            FloorZone half = fp.zones[s0];   // (a half's facts never win: the flat keeps its own frame's axis)
            half.area = 0;
            half.frontage = 0;
            for (std::size_t k : run) half.frontage += fp.zones[k].frontage * 0.5;
            fp.zones[to[0]] = joined(un[0], fp.zones[to[0]], half);
            fp.zones[to[1]] = joined(un[1], fp.zones[to[1]], half);
            drop.insert(drop.end(), run.begin(), run.end());
        }
        std::sort(drop.begin(), drop.end());
        for (auto it = drop.rbegin(); it != drop.rend(); ++it) fp.zones.erase(fp.zones.begin() + static_cast<std::ptrdiff_t>(*it));
    }
    // A CREVICE: floor no lot took that does not reach the corridor (a scrap between a tight corner's lots, cut off
    // by them) is walled in by flats and good to no one -- it joins the window lot it shares the most wall with
    if (zp.minAccess > 0 && !K.empty()) {
        // the corridor network: every piece of circulation, joined, and the parts of it that reach the core
        std::vector<Poly2> outs;
        for (const std::vector<Poly2>& piece : fp.circulation) outs.push_back(piece.front());
        G::PolySet net = G::offsetSet(G::unionSets(unionAll(outs), K), 0.05);
        G::PolySet reach;
        for (const G::Polygon2& q : net)
            if (G::setArea(G::intersectSets(G::PolySet{q}, K)) > 0.05) reach.push_back(q);
        for (std::size_t ci = fp.circulation.size(); ci-- > 0;) {
            const std::vector<Poly2>& piece = fp.circulation[ci];
            if (piece.size() != 1 || area(piece.front()) > 15.0) continue;
            Poly2 scrap = piece.front();
            ensureCCW(scrap);
            if (G::setArea(G::intersectSets(G::fromRing(scrap), reach)) > 0.05) continue;   // on the way in: corridor
            std::size_t best = fp.zones.size();
            Real bestGain = 0.3;
            Poly2 bestPoly;
            for (std::size_t b = 0; b < fp.zones.size(); ++b) {
                if (fp.zones[b].site == ZoneSite::Interior) continue;
                const G::PolySet un = G::unionSets(G::fromRing(scrap), G::fromRing(fp.zones[b].poly));
                if (un.size() != 1 || !un.front().holes.empty()) continue;
                const Real gain = perimeter(scrap) + perimeter(fp.zones[b].poly) - perimeter(un.front().outer);
                if (gain > bestGain) { bestGain = gain; best = b; bestPoly = un.front().outer; }
            }
            if (best >= fp.zones.size()) continue;
            FloorZone none = fp.zones[best];
            none.area = 0;
            none.frontage = 0;
            fp.zones[best] = joined(bestPoly, fp.zones[best], none);
            fp.circulation.erase(fp.circulation.begin() + static_cast<std::ptrdiff_t>(ci));
        }
    }
    // ...and what is still narrow pairs up with a narrow neighbour (a small drum: all its wedges meet the corridor at
    // the core's corners, none wide enough alone): two lots a flat, never more
    if (zp.minAccess > 0) {
        std::vector<char> paired(fp.zones.size(), 0), folded(fp.zones.size(), 0);
        for (std::size_t a = 0; a < fp.zones.size(); ++a) {
            const FloorZone& A = fp.zones[a];
            if (paired[a] || A.onStub || A.site == ZoneSite::Interior || A.accessLen >= zp.minAccess) continue;
            std::size_t best = fp.zones.size();
            Real bestScore = -1e30;
            Poly2 bestPoly;
            for (std::size_t b = 0; b < fp.zones.size(); ++b) {
                const FloorZone& B = fp.zones[b];
                if (b == a || paired[b] || B.onStub || B.site == ZoneSite::Interior || B.accessLen >= zp.minAccess) continue;
                const G::PolySet un = G::unionSets(G::fromRing(A.poly), G::fromRing(B.poly));
                if (un.size() != 1 || !un.front().holes.empty()) continue;
                if (area(un.front().outer) > 1.8 * zp.lotWidth * zp.windowDepth) continue;
                const Real gain = perimeter(A.poly) + perimeter(B.poly) - perimeter(un.front().outer);
                if (gain < 0.5) continue;
                const Real score = B.accessLen + 0.01 * gain;   // (the partner with the most way in)
                if (score > bestScore) { bestScore = score; best = b; bestPoly = un.front().outer; }
            }
            if (best >= fp.zones.size()) continue;
            fp.zones[a] = joined(bestPoly, A, fp.zones[best]);
            paired[a] = paired[best] = 1;
            folded[best] = 1;
        }
        for (std::size_t k = fp.zones.size(); k-- > 0;)
            if (folded[k]) fp.zones.erase(fp.zones.begin() + static_cast<std::ptrdiff_t>(k));
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

FloorProgram builtinDepartmentStoreProgram() {
    FloorProgram p;
    p.name = "department_store";
    p.zones.windowDepth = 6.0;
    p.zones.aisle = 2.4;
    p.zones.street = 2.4;
    p.zones.lotWidth = 7.2;
    p.zones.interiorLot = 36.0;
    p.fill = "sales";
    p.shallow = "sales";
    p.shallowDepth = 0;
    p.departments = {2, 2, 4, 3, 6};   // boutique, boutique, electronics, bookshop, bakery (the food hall)
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
    ZoneRule* r = rule("cash_desk", RoomKind::CashDesk, I | W);
    r->count = 1; r->near = 1; r->minArea = 8; r->maxArea = 60; r->minDepth = 2.6;
    r = rule("fitting_rooms", RoomKind::FittingRooms, I | W);
    r->count = 1; r->near = 2; r->enclosed = true; r->minArea = 8; r->maxArea = 50; r->minDepth = 2.4; r->departments = {2};
    r = rule("stockroom", RoomKind::Stockroom, I | W);
    r->count = 1; r->near = 2; r->enclosed = true; r->minArea = 8; r->maxArea = 50; r->minDepth = 2.0; r->minFloor = 300;
    rule("sales", RoomKind::SalesFloor, W | C | I);
    return p;
}

FloorProgram builtinDiningProgram() {
    FloorProgram p;
    p.name = "dining";
    p.zones.windowDepth = 7.0;
    p.zones.lotWidth = 9.0;
    p.zones.interiorLot = 30.0;
    p.zones.minAccess = 2.0;
    p.fill = "lounge";
    p.shallow = "lounge";
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
    ZoneRule* r = rule("restrooms", RoomKind::Bath, I | W);
    r->count = 1; r->near = 2; r->enclosed = true; r->minArea = 6; r->maxArea = 30; r->minDepth = 2.0; r->minFloor = 250;
    r = rule("restaurant", RoomKind::Shop, W | C);
    r->share = 0.5; r->enclosed = true; r->minArea = 25; r->minDepth = 4.5; r->trade = 13;
    r = rule("bar", RoomKind::Shop, W | C);
    r->share = 1.0; r->enclosed = true; r->minArea = 20; r->minDepth = 3.5; r->trade = 14;
    rule("lounge", RoomKind::Lounge, W | C | I);
    return p;
}

FloorProgram builtinHotelProgram() {
    FloorProgram p;
    p.name = "hotel";
    p.workplace = false;
    p.zones.windowDepth = 6.4;
    p.zones.lotWidth = 4.2;
    p.zones.minAccess = 1.6;
    p.zones.minLotArea = 14.0;
    p.fill = "hall";
    p.shallow = "hall";
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
    ZoneRule* r = rule("linen", RoomKind::LinenRoom, I | W);
    r->count = 1; r->near = 1; r->enclosed = true; r->minArea = 6; r->maxArea = 30; r->minDepth = 2.0;
    r = rule("suite", RoomKind::HotelSuite, C);
    r->share = 1.0; r->enclosed = true; r->minArea = 28; r->minDepth = 4.8; r->ensuite = true;
    r = rule("window_suite", RoomKind::HotelSuite, W);
    r->share = 0.0; r->enclosed = true; r->minArea = 26; r->minDepth = 4.8; r->ensuite = true;
    r = rule("room", RoomKind::HotelRoom, W | C);
    r->share = 1.0; r->enclosed = true; r->minArea = 14; r->minDepth = 4.6; r->ensuite = true;
    rule("hall", RoomKind::Hall, W | C | I);
    ZoneStyle s;
    s.name = "budget"; s.weight = 1;
    s.sets = {{"lots", "lot_width", 3.6}, {"lots", "window_depth", 5.6}, {"suite", "share", 0}};
    p.styles.push_back(s);
    s = ZoneStyle{}; s.name = "business"; s.weight = 2; p.styles.push_back(s);
    s = ZoneStyle{}; s.name = "luxury"; s.weight = 1;
    s.sets = {{"lots", "lot_width", 5.2}, {"lots", "window_depth", 7.2}, {"window_suite", "share", 0.25}};
    p.styles.push_back(s);
    return p;
}

int floorDepartment(const FloorProgram& prog, int storey) {
    if (prog.departments.empty()) return -1;
    const int n = static_cast<int>(prog.departments.size());
    return prog.departments[static_cast<std::size_t>(((std::max(1, storey) - 1) % n + n) % n)];
}

FloorProgram builtinResidentsProgram() {
    FloorProgram p;
    p.name = "residents";
    p.fill = "lounge";
    p.shallow = "lounge";
    p.shallowDepth = 0;
    ZoneRule r;
    r.tag = "lounge";
    r.kind = RoomKind::Lounge;
    r.sites = zoneSiteBit(ZoneSite::Window) | zoneSiteBit(ZoneSite::Corner) | zoneSiteBit(ZoneSite::Interior);
    p.rules.push_back(r);
    return p;
}

bool hasFloorProgram(const std::string& name) {
    return FloorProgramBook::global().find(name) != nullptr || name == "office" || name == "apartments" || name == "residents" ||
           name == "department_store" || name == "dining" || name == "hotel";
}

const FloorProgram& floorProgram(const std::string& name) {
    if (const FloorProgram* p = FloorProgramBook::global().find(name)) return *p;
    static const FloorProgram office = builtinOfficeProgram(), apartments = builtinApartmentProgram(),
                              residents = builtinResidentsProgram(), store = builtinDepartmentStoreProgram(),
                              dining = builtinDiningProgram(), hotel = builtinHotelProgram();
    return name == "apartments" ? apartments : name == "residents" ? residents : name == "department_store" ? store
         : name == "dining" ? dining : name == "hotel" ? hotel : office;
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

const ZoneStyle* floorStyle(const FloorProgram& prog, uint32_t seed) {
    if (prog.styles.empty()) return nullptr;
    Real total = 0;
    for (const ZoneStyle& s : prog.styles) total += std::max(Real(0), s.weight);
    Real pick = (mix(seed * 2246822519u + 77u) % 100000u) / 100000.0 * total;
    const ZoneStyle* st = &prog.styles.back();
    for (const ZoneStyle& s : prog.styles) {
        if (pick < s.weight) { st = &s; break; }
        pick -= s.weight;
    }
    return st;
}

ZoneTags tagZones(const FloorPartition& fp, const FloorProgram& progIn, uint32_t seed, bool lifts, int department) {
    ZoneTags out;
    std::vector<ZoneRule> rules = progIn.rules;
    // the building's habit
    if (const ZoneStyle* st = floorStyle(progIn, seed)) {
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
        if (!r.departments.empty() &&
            std::find(r.departments.begin(), r.departments.end(), static_cast<uint8_t>(std::max(0, department))) == r.departments.end())
            continue;
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
    // the back strip: the rooms whose far side is the frame's back
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
    // every room runs on to the lot's walls -- no crevice left between a room and a wall: the window rooms out to
    // the glass (a curve's bulge) and, at the ends, out to the lot's sides; the back strip's rooms (hall, bath,
    // kitchen, closet) back to the corridor wall and, at the ends, out to the sides as well, so a bath in a wedge's
    // corner takes the whole trapezoid
    for (Room& r : lay.rooms) {
        const bool back = isBack(r);
        for (Vec2& q : r.rect) {
            const Real x = dot(q - O, u), v = dot(q - O, nIn);
            Vec2 d(0, 0);
            if (std::fabs(x - x0) < 1e-4) d = d - u * far;
            if (std::fabs(x - x1) < 1e-4) d = d + u * far;
            if (!back && std::fabs(v - v0) < 1e-4) d = d - nIn * far;
            if (back && std::fabs(v - v1) < 1e-4) d = d + nIn * far;
            q = q + d;
        }
        ensureCCW(r.rect);
    }
    for (RoomWall& w : lay.walls) {
        const Vec2 door0 = w.a + (w.b - w.a) * std::max(Real(0), w.doorAt);
        const bool alongU = std::fabs(dot(w.b - w.a, nIn)) < 1e-4;
        for (Vec2* q : {&w.a, &w.b}) {
            const Real x = dot(*q - O, u), v = dot(*q - O, nIn);
            if (alongU && std::fabs(x - x0) < 1e-4) *q = *q - u * far;
            if (alongU && std::fabs(x - x1) < 1e-4) *q = *q + u * far;
            if (!alongU && std::fabs(v - v0) < 1e-4) *q = *q - nIn * far;
            if (!alongU && std::fabs(v - v1) < 1e-4) *q = *q + nIn * far;
        }
        if (w.doorAt >= 0) w.doorAt = dot(door0 - w.a, normalize(w.b - w.a)) / std::max(Real(1e-6), (w.b - w.a).length());
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
    // every bit of the flat is some room's: a scrap cut off a room by a slanted wall is a crevice no one can use
    {
        Real covered = 0;
        for (const Room& r : rs) covered += area(r.rect);
        if (covered < 0.97 * area(Z.poly)) {
            if (std::getenv("RT_FLAT_DEBUG")) std::fprintf(stderr, "[flat] rooms cover %.1f of %.1f m2\n", covered, area(Z.poly));
            return false;
        }
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
// A ROOM WITH ITS OWN BATHROOM (a hotel room, a suite): `room` (its zone's polygon and frame) split into the bedroom
// and a bathroom of up to 2.2 x 2.4 m in the corner of the door's wall away from the door, its own door onto the
// short passage from the room's. False -- the room stays whole -- where the frame is too small for a bath, a
// passage and a bed beyond.
bool layEnsuite(const Room& room, const Vec2& door, std::vector<Room>& rooms, std::vector<RoomWall>& walls) {
    if (room.frame.size() != 4) return false;
    // the frame turned so the side nearest the door is its side 2 (u along side 0, v in from it)
    std::size_t k = 0;
    Real bd0 = 1e9;
    for (std::size_t s = 0; s < 4; ++s) {
        const Real d = distToSegment(door, room.frame[(s + 2) % 4], room.frame[(s + 3) % 4]);
        if (d < bd0) { bd0 = d; k = s; }
    }
    if (bd0 > 0.8) return false;   // (the door is not on the frame's edge: a wedge's)
    Poly2 f(4);
    for (std::size_t s = 0; s < 4; ++s) f[s] = room.frame[(s + k) % 4];
    const Vec2 O = f[0], U = normalize(f[1] - f[0]), N = normalize(f[3] - f[0]);
    const Real W = (f[1] - f[0]).length(), D = (f[3] - f[0]).length();
    // the bath: up to 2.2 m along the door's wall beside the door, up to 2.4 m deep (1.5 x 1.7 at the least: a shower
    // room), leaving 3 m for the bed and its nightstands beyond; a room too shallow for that but wide has it across
    // its end instead, the full depth of the room
    const Real du = dot(door - O, U);
    const bool right = du < W * 0.5;   // the bath at the far end of the door's wall
    const Real beside = right ? W - (du + kRoomDoorW * 0.5 + 0.15) : du - kRoomDoorW * 0.5 - 0.15;
    const Real bw = std::min(Real(2.2), beside);
    Real bd = std::min(Real(2.4), D - 3.0);
    const bool across = bd < 1.7;   // (the bath across the room's end)
    if (across) bd = D;
    if (bw < 1.5 || bd < 1.7 || (across && (W - bw < 3.0 || D > 4.0))) return false;
    const Real u0 = right ? W - bw : 0, u1 = right ? W : bw, v0 = D - bd;
    auto P = [&](Real u, Real v) { return O + U * u + N * v; };
    Room bath;
    bath.kind = RoomKind::Bath;
    bath.edge = room.edge;
    bath.rect = {P(u0, v0), P(u1, v0), P(u1, D), P(u0, D)};
    ensureCCW(bath.rect);
    // its frame: side 0 the wall across from its door (the room's side wall), side 2 its door's
    const Real uIn = right ? u0 : u1, uOut = right ? u1 : u0;
    bath.frame = {P(uOut, D), P(uOut, v0), P(uIn, v0), P(uIn, D)};
    ensureCCW(bath.frame);
    Room bed = room;
    bed.rect = largestOuter(G::differenceSets(G::fromRing(room.rect), G::fromRing(bath.rect)));
    if (bed.rect.size() < 3) return false;
    bed.frame = across ? (right ? Poly2{P(0, 0), P(u0, 0), P(u0, D), P(0, D)} : Poly2{P(u1, 0), P(W, 0), P(W, D), P(u1, D)})
                       : Poly2{P(0, 0), P(W, 0), P(W, v0), P(0, v0)};
    ensureCCW(bed.frame);
    rooms = {bed, bath};
    walls.clear();
    if (!across) {   // the bath's back, into the room
        RoomWall back;
        back.a = P(u0, v0);
        back.b = P(u1, v0);
        walls.push_back(back);
    }
    RoomWall side;     // its side onto the passage (or the room), with its door
    side.a = P(uIn, v0);
    side.b = P(uIn, D);
    side.doorAt = across ? 0.75 : 0.5;   // (across the end: its door toward the room's)
    walls.push_back(side);
    return true;
}

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
            // (a bath, closet or hall much bigger than the rectangle in it is mostly a sliver along a slanted wall; a
            // living room or a bedroom wider at the glass is just a wedge's)
            const bool service = r.kind == RoomKind::Bath || r.kind == RoomKind::Closet || r.kind == RoomKind::Hall;
            if (service && r.frame.size() == 4) {
                const Real fa = (r.frame[1] - r.frame[0]).length() * (r.frame[3] - r.frame[0]).length();
                score -= 1.5 * std::max(Real(0), a - std::max(1.6 * fa, fa + 4.0));
            }
            // (a bath or a closet run on to a far wall has room to spare: a layout nearer the corridor is better)
            else if (r.kind == RoomKind::Bath) score -= 2.0 * std::max(Real(0), a - 8.0);
            else if (r.kind == RoomKind::Closet) score -= std::max(Real(0), a - 6.0);
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

static RoomPlan zonedFloorCut(const Poly2& plate, const BuildingParams& params, const CorePlan& core, Real inset,
                              int storey, bool lifts, const Vec2& entry, const Poly2& stairWell, const FloorProgram& prog) {
    RoomPlan rp;
    rp.topology = prog.residential ? PlateTopology::Apartments : PlateTopology::Ring;
    rp.office = !prog.residential && prog.workplace;
    if (storey < 1) return rp;
    Poly2 plan = plate;
    ensureCCW(plan);
    // the building's habit may cut the plate its own way (a budget hotel's narrow rooms)
    FloorZoneParams zp = prog.zones;
    if (const ZoneStyle* st = floorStyle(prog, static_cast<uint32_t>(params.seed)))
        for (const ZoneStyle::Set& set : st->sets) {
            if (set.tag != "lots") continue;
            if (set.field == "window_depth") zp.windowDepth = set.value;
            else if (set.field == "aisle") zp.aisle = set.value;
            else if (set.field == "street") zp.street = set.value;
            else if (set.field == "lot_width") zp.lotWidth = set.value;
            else if (set.field == "interior_lot") zp.interiorLot = set.value;
            else if (set.field == "min_side") zp.minSide = set.value;
            else if (set.field == "min_access") zp.minAccess = set.value;
            else if (set.field == "min_lot_area") zp.minLotArea = set.value;
        }
    FloorPartition fp = partitionFloor(plan, core, inset, params, zp, core.valid ? Poly2{} : stairWell);
    if (fp.zones.empty() && core.valid && !prog.residential) {   // a shallow floor: lobby mode (FloorZoneParams)
        FloorZoneParams lz = zp;
        lz.lobbyOnly = true;
        fp = partitionFloor(plan, core, inset, params, lz, Poly2{});
    }
    if (fp.zones.empty()) return rp;
    const int department = floorDepartment(prog, storey);
    const ZoneTags tags = tagZones(fp, prog, static_cast<uint32_t>(params.seed), lifts, department);
    std::vector<int> rule = tags.rule;
    // THE WAY IN is never shut in by walled rooms: a lot the floor's entry stands in or beside is the fill, open, and
    // so is the walled lot nearest it within the corridor round the hub -- a stair at the facade has no corridor out
    // to the aisles but through a lot (a restaurant floor's every window lot is walled; an office's are open floor)
    if (!prog.residential) {
        const int fillR = ruleIndex(prog.rules, prog.fill);
        std::size_t nearest = fp.zones.size();
        Real nearestD = kCorridorRing + 0.9;
        for (std::size_t z = 0; z < fp.zones.size() && fillR >= 0; ++z) {
            if (!prog.rules[static_cast<std::size_t>(rule[z])].enclosed) continue;
            const Poly2& P = fp.zones[z].poly;
            Real d = pointInPolygon(P, entry) ? 0 : 1e9;
            for (std::size_t i = 0; i < P.size(); ++i) d = std::min(d, distToSegment(entry, P[i], P[(i + 1) % P.size()]));
            if (d < 0.9) rule[z] = fillR;   // (in it, or a capsule beside its wall)
            else if (d < nearestD) { nearestD = d; nearest = z; }
        }
        if (nearest < fp.zones.size()) rule[nearest] = fillR;
    }
    // HOMES: the program's fill (the landing) is part of the corridor -- a landing lot with no edge on circulation (a
    // notched plate's arm, walled in by the facade and the flats beside it) is no corridor and no room; its floor is
    // left walled in rather than laid as a hall no door reaches
    if (prog.residential) {
        const int fill = ruleIndex(prog.rules, prog.fill);
        for (std::size_t z = fp.zones.size(); z-- > 0;)
            if (rule[z] == fill && fp.zones[z].accessLen < kDoorRun) {
                fp.zones.erase(fp.zones.begin() + static_cast<std::ptrdiff_t>(z));
                rule.erase(rule.begin() + static_cast<std::ptrdiff_t>(z));
            }
    }

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
    // the floor is walked inside the facade's inner face: the strip the outside wall stands on is no way round a room
    // walled across an arm of the plate (the space graph's rule; a fitting room cut a store's arm off, uses step 3).
    // (Not yet homes: some flats' bedrooms are reached only along that strip -- layFlat's doors, owed with item 35.)
    Poly2 walkPlan = plan;
    if (!prog.residential) {
        Real best = 0;
        for (const G::Polygon2& q : IN)
            if (ringAreaAbs(q.outer) > best) { best = ringAreaAbs(q.outer); walkPlan = q.outer; }
        ensureCCW(walkPlan);
    }
    G::PolySet circSet;
    for (const std::vector<Poly2>& piece : fp.circulation) {
        G::Polygon2 p;
        p.outer = piece.front();
        p.holes.assign(piece.begin() + 1, piece.end());
        circSet.push_back(p);
    }
    if (fp.lobbyMode) {   // no corridor round the core: the lobby is the way in, never the core's own walls
        for (const Poly2& l : fp.lobbyArea) circSet = G::unionSets(circSet, G::fromRing(l));
    } else if (core.valid) {
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

    // four tries: as tagged; the enclosed rooms whose only way in is through another room opened up; the rooms that
    // walk still could not reach opened up, the rest keeping their walls (a restaurant beside a stair at the facade);
    // no walls (not in homes, which stop after the second: a floor of flats with no walls is no floor of flats --
    // zonedFloorPlan lays it another way)
    // the walk goes round the core, never through it (its shafts are no floor; the stair is inside it): with a corridor
    // round the core that never mattered, but a lobby-mode floor's lots stand against it
    const Poly2 solid = core.valid ? core.rect() : stairWell;
    bool walked = false, pruned = false;
    int flats = 0, studios = 0;
    std::vector<std::size_t> roomZone;   // the zone each room was laid from
    std::vector<char> opened(fp.zones.size(), 0);   // the third try's: zones the first two tries' walk could not reach
    std::size_t missed = 0;
    for (int attempt = 0; attempt < (prog.residential ? 2 : 4); ++attempt) {
        flats = studios = 0;
        rp.rooms.clear();
        rp.walls.clear();
        roomZone.clear();
        std::vector<Poly2> open;
        for (std::size_t z = 0; z < fp.zones.size(); ++z)
            if (!prog.rules[static_cast<std::size_t>(rule[z])].enclosed || attempt == 3 || (attempt == 2 && opened[z]))
                open.push_back(fp.zones[z].poly);
        const G::PreparedSet walk(open.empty() ? circSet : G::unionSets(circSet, unionAll(open)));
        for (std::size_t z = 0; z < fp.zones.size(); ++z) {
            roomZone.resize(rp.rooms.size(), z ? z - 1 : 0);   // (the rooms the zone before laid)
            FloorZone& Z = fp.zones[z];
            const ZoneRule& R = prog.rules[static_cast<std::size_t>(rule[z])];
            Room rm;
            rm.kind = R.kind;
            if (R.trade >= 0) rm.style = static_cast<uint8_t>(R.trade);   // (a restaurant floor's: its own trade)
            else if (department >= 0) rm.style = static_cast<uint8_t>(department);   // (a shop floor: its department's)
            rm.rect = Z.poly;
            rm.edge = static_cast<std::size_t>(std::max(0, Z.group));
            const bool enclosed = R.enclosed && attempt < 3 && !(attempt == 2 && opened[z]);
            // the door: the longest clear run onto the corridors; failing that (first and third tries), onto an open zone
            // (a copy room beside the reception opens onto the corridor, not through the reception)
            std::size_t doorEdge = Z.poly.size();
            Real doorMid = 0.5, best = 0;
            for (int via = 0; enclosed && via < (attempt == 1 ? 1 : 2) && best < kDoorRun; ++via) {
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
            // A ROOM WITH ITS OWN BATHROOM (a hotel's): its door at one end of its run onto the corridor (the other end
            // of the run, then the middle, if that will not do), the bathroom carved beside it
            std::vector<Room> ensRooms;
            std::vector<RoomWall> ensWalls;
            bool ensLaid = false;
            if (R.ensuite && enclosed && best >= kDoorRun && doorEdge < Z.poly.size()) {
                const Vec2 ea = Z.poly[doorEdge], eb = Z.poly[(doorEdge + 1) % Z.poly.size()];
                const Real EL = std::max(Real(1e-6), (eb - ea).length()), m = (kRoomDoorW * 0.5 + 0.1) / EL;
                const Real tLo = doorMid - best / (2 * EL) + m, tHi = doorMid + best / (2 * EL) - m;
                std::vector<Real> tries;
                if (tHi > tLo) { tries.push_back(tLo); tries.push_back(tHi); }
                tries.push_back(doorMid);
                for (Real t : tries)
                    if (layEnsuite(rm, ea + (eb - ea) * t, ensRooms, ensWalls)) { doorMid = t; ensLaid = true; break; }
            }
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
            if (ensLaid) {
                rp.rooms.pop_back();
                for (const Room& r : ensRooms) rp.rooms.push_back(r);
                for (const RoomWall& w : ensWalls) addWall(rp.walls, w);
            }
            if (flatLaid) {
                rp.rooms.pop_back();
                for (Room& r : flatRooms) { r.edge = rm.edge; rp.rooms.push_back(r); }
                for (const RoomWall& w : flatWalls) addWall(rp.walls, w);
                ++flats;
            } else if (prog.residential && R.kind == RoomKind::Flat) {
                ++studios;
            }
        }
        roomZone.resize(rp.rooms.size(), fp.zones.empty() ? 0 : fp.zones.size() - 1);
        if (floorIsWalkable(rp, walkPlan, entry, solid)) { walked = true; break; }
        // HOMES: a landing or storage lot the flats round it have cut off (a notched plate's arm) is no room anyone
        // needs to reach -- drop it (its floor is left walled in) and lay the floor again, once. A flat cut off stays
        // a failure.
        if (prog.residential && !pruned) {
            std::vector<std::size_t> un;
            floorIsWalkable(rp, walkPlan, entry, solid, &un);
            std::vector<char> drop(fp.zones.size(), 0);
            bool onlyLeftovers = !un.empty();
            for (std::size_t k : un) {
                const std::size_t zi = k < roomZone.size() ? roomZone[k] : fp.zones.size();
                if (zi >= fp.zones.size() || prog.rules[static_cast<std::size_t>(rule[zi])].kind == RoomKind::Flat) {
                    onlyLeftovers = false;
                    break;
                }
                drop[zi] = 1;
            }
            if (onlyLeftovers) {
                for (std::size_t z = fp.zones.size(); z-- > 0;)
                    if (drop[z]) {
                        fp.zones.erase(fp.zones.begin() + static_cast<std::ptrdiff_t>(z));
                        rule.erase(rule.begin() + static_cast<std::ptrdiff_t>(z));
                    }
                pruned = true;
                attempt = -1;   // (the loop's ++ starts it over at the first try)
                continue;
            }
        }
        if (!prog.residential && attempt < 2) {   // (the rooms to open on the third try: the fewer the first two missed)
            std::vector<std::size_t> un;
            floorIsWalkable(rp, walkPlan, entry, solid, &un);
            if (attempt == 0 || un.size() < missed) {
                missed = un.size();
                std::fill(opened.begin(), opened.end(), 0);
                for (std::size_t k : un)
                    if (k < roomZone.size() && roomZone[k] < opened.size()) opened[roomZone[k]] = 1;
            }
        }
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
        std::fprintf(stderr, "; %zu walls, walked %d\n", rp.walls.size(), walked ? 1 : 0);
        if (prog.residential) std::fprintf(stderr, "[zones] storey %d: %d flats, %d studios, walked %d\n", storey, flats, studios, walked ? 1 : 0);
    }
    // (a floor of homes with no flat on it -- every lot too small or shut off -- is laid another way too)
    if (prog.residential && (!walked || flats + studios == 0) && !std::getenv("RT_ZONE_KEEP_FAILED")) {
        RoomPlan none;
        none.topology = rp.topology;
        none.office = false;
        return none;
    }
    return rp;
}

RoomPlan zonedFloorPlan(const Poly2& plate, const BuildingParams& params, const CorePlan& core, Real inset,
                        int storey, bool lifts, const Vec2& entry, const Poly2& stairWell, const FloorProgram& prog) {
    RoomPlan rp = zonedFloorCut(plate, params, core, inset, storey, lifts, entry, stairWell, prog);
    // A SHALLOW FLOOR OF HOMES (lobby mode, #117 item 35): too shallow for flats off a corridor round the core, the
    // lifts open onto a lobby and two flats wrap the rest of the core (room_plan.h wrapFlatsLayout). (An office floor
    // takes lobby mode inside the cut: its lots off the lobby are open zones, reached through each other.)
    if (prog.residential && rp.rooms.empty() && core.valid && storey >= 1) {
        RoomPlan wf = wrapFlatsLayout(plate, core, inset, storey);
        if (!wf.rooms.empty() && floorIsWalkable(wf, plate, entry, core.rect())) return wf;
        // ...and a floor too small even for that (a mid-rise's lift core leaving 2-3 m round it) is the RESIDENTS'
        // FLOOR: the `residents` program's lounge round the core, walked from the lifts -- never an empty floor
        RoomPlan lounge = zonedFloorCut(plate, params, core, inset, storey, lifts, entry, stairWell, floorProgram("residents"));
        if (!lounge.rooms.empty()) {
            lounge.topology = PlateTopology::Apartments;
            lounge.office = false;   // homes: no office lighting
            return lounge;
        }
    }
    return rp;
}

}  // namespace engine
