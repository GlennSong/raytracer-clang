#include "room_plan.h"
#include "floor_zones.h"
#include "building_uses.h"   // floorUseName
#include "../../mesh_builder.h"
#include <algorithm>
#include <functional>
#include <cmath>
#include <utility>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace engine {

namespace {

constexpr Real kCorridorRing = 2.2;   // core_plan.cpp's kCorridor: the ring the core keeps clear
constexpr Real kOfficeDepth = 4.5;
constexpr Real kFlatDepth = 5.5;
constexpr Real kOfficeWidth = 4.8;    // target room widths (bays are grouped up to these)
constexpr Real kFlatWidth = 5.6;
constexpr Real kMinDepth = 3.0;       // a band shallower than this is not worth a room
constexpr Real kDeskRowMin = 2.55;    // glass to core: the window gap, a desk and its chair, the walk (desk_row's grid)
constexpr Real kDeskRowMax = 6.0;

// The facade's bay boundaries along a face of width W — emitCurtainWallRect's
// 1.6 m bays and facadeLayout's `bayWidth` bays, reproduced so a partition
// lands on a mullion or a pier.
std::vector<Real> bayBoundaries(Real W, const BuildingParams& p) {
    const Real bw = p.curtainWall ? Real(1.6) : std::max(p.bayWidth, Real(0.5));
    const int bays = std::max(1, static_cast<int>(std::lround(W / bw)));
    std::vector<Real> xs;
    for (int b = 0; b <= bays; ++b) xs.push_back(W * b / bays);
    return xs;
}

// Distance from the line through a->b (outward normal n) to the nearest
// corner of the core's outline, measured inward; +inf without a core -- or
// when the core does not stand IN FRONT of the edge: on an L or cross plate
// (buildings M10) the core beside an arm's edge is no limit on that arm's
// rooms. The edge's span along `d` (length W) is widened by the corridor ring.
// How far a ray from `from` along `dir` runs inside `plan` before it leaves: the first crossing of any edge but
// `skipEdge` (the one the ray starts on).
Real depthInto(const Poly2& plan, const Vec2& from, const Vec2& dir, std::size_t skipEdge) {
    Real best = 1e9;
    const std::size_t n = plan.size();
    for (std::size_t e = 0; e < n; ++e) {
        if (e == skipEdge) continue;
        const Vec2 a = plan[e], b = plan[(e + 1) % n];
        const Vec2 ab = b - a;
        const Real den = dir.x * ab.y - dir.y * ab.x;
        if (std::fabs(den) < 1e-9) continue;   // parallel
        const Vec2 ao = a - from;
        const Real t = (ao.x * ab.y - ao.y * ab.x) / den;   // along the ray
        const Real s = (ao.x * dir.y - ao.y * dir.x) / den;   // along the edge
        if (t > 1e-6 && s >= -1e-6 && s <= 1.0 + 1e-6) best = std::min(best, t);
    }
    return best;
}

bool convexPlan(const Poly2& plan) {   // (CCW)
    const std::size_t n = plan.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = plan[i], b = plan[(i + 1) % n], c = plan[(i + 2) % n];
        const Vec2 u = b - a, v = c - b;
        if (u.x * v.y - u.y * v.x < -1e-6) return false;
    }
    return true;
}

Real depthToCore(const Vec2& a, const Vec2& d, Real W, const Vec2& nOut, const CorePlan& core) {
    if (!core.valid) return 1e9;
    Real t0 = 1e30, t1 = -1e30, best = 1e9;
    for (const Vec2& c : core.rect()) {
        t0 = std::min(t0, dot(c - a, d)); t1 = std::max(t1, dot(c - a, d));
        best = std::min(best, dot(a - c, nOut));
    }
    if (t1 < -kCorridorRing || t0 > W + kCorridorRing) return 1e9;
    return best;
}

}  // namespace

// THE RING: rooms along the outside walls, an open middle, the core kept
// clear. Offices behind a curtain wall, flats behind masonry.
static RoomPlan ringPlan(const Poly2& planIn, const BuildingParams& params, const CorePlan& core,
                         std::size_t blankEdge, Real inset, int storey) {
    RoomPlan rp;
    rp.topology = PlateTopology::Ring;
    rp.office = params.curtainWall || params.officeUse;   // offices (glass-fronted behind a curtain wall), else flats
    if (storey < 1 || planIn.size() < 3) return rp;
    Poly2 plan = planIn;
    ensureCCW(plan);
    const std::size_t n = plan.size();
    const Real depthWant = rp.office ? kOfficeDepth : kFlatDepth;
    const Real widthWant = rp.office ? kOfficeWidth : kFlatWidth;
    const bool concave = !convexPlan(plan);

    // Per edge: the frame and the band depth it can afford.
    struct Edge { Vec2 a, d, nOut; Real W = 0, depth = 0; bool rooms = false; Real cornerStart = 0, cornerEnd = 0; };
    std::vector<Edge> E(n);
    for (std::size_t e = 0; e < n; ++e) {
        Edge& ed = E[e];
        ed.a = plan[e];
        const Vec2 b = plan[(e + 1) % n];
        const Vec2 dv = b - ed.a;
        ed.W = dv.length();
        if (ed.W < 1e-6) continue;
        ed.d = dv * (1.0 / ed.W);
        ed.nOut = Vec2(ed.d.y, -ed.d.x);            // CCW: the interior is to the left
        if (e == blankEdge) continue;
        // The corridor ring: the band ends 2.2 m short of the core.
        const Real room = depthToCore(ed.a, ed.d, ed.W, ed.nOut, core) - kCorridorRing - inset;
        ed.depth = std::min(depthWant, room);
        // Two bands may NEVER meet. Cap the depth at half the plate's own
        // depth less a gap to walk through: a shallow plate used to grow the
        // opposite bands straight through each other (Glenn's houses,
        // 2026-09-15 — such a plate is now a whole floor, and this holds the
        // invariant for every plate in between).
        Real across = 0;
        for (const Vec2& w : plan) across = std::max(across, dot(w - ed.a, ed.nOut * -1.0));
        // ...measured INTO the plan where the plan is not convex: an L's arm is as deep as the arm, not as the
        // whole plate (the space graph's gate, issue #105, 2026-10-15: a ring office on a 5 m arm spanned it and
        // opened its door onto the exterior wall). A convex plate measures as before.
        if (concave) {
            Real into = 1e9;
            for (int si = 0; si < 5; ++si) {
                const Real x = inset + 0.5 + std::max(Real(0), ed.W - 2.0 * inset - 1.0) * (si / 4.0);
                if (x > ed.W) continue;
                into = std::min(into, depthInto(plan, ed.a + ed.d * x, ed.nOut * -1.0, e));
            }
            if (into < 1e8) across = std::min(across, into);
        }
        ed.depth = std::min(ed.depth, (across - 2.0 * inset - 1.4) * 0.5);
        ed.rooms = ed.depth >= kMinDepth && ed.W >= 2.0 * inset + 2.5;
    }

    // Rooms along an edge over the span [x0, x1] (along the edge, from a);
    // returns the first and last room widths (the corners' claims).
    auto lay = [&](std::size_t e, Real x0, Real x1, Real& firstW, Real& lastW) {
        Edge& ed = E[e];
        firstW = lastW = 0;
        if (x1 - x0 < 2.0) return;
        // Group the bay boundaries inside the span into rooms of ~widthWant.
        std::vector<Real> cuts;
        cuts.push_back(x0);
        Real last = x0;
        for (Real xb : bayBoundaries(ed.W, params)) {
            if (xb <= x0 + 1.0 || xb >= x1 - 1.0) continue;
            if (xb - last >= widthWant * 0.85) { cuts.push_back(xb); last = xb; }
        }
        if (x1 - last < widthWant * 0.45 && cuts.size() > 1) cuts.pop_back();   // a sliver joins its neighbour
        cuts.push_back(x1);
        const Real dIn = inset, dOut = inset + ed.depth;
        auto P = [&](Real x, Real depth) { return ed.a + ed.d * x - ed.nOut * depth; };
        for (std::size_t i = 0; i + 1 < cuts.size(); ++i) {
            const Real ra = cuts[i], rb = cuts[i + 1];
            Room r;
            r.edge = e;
            r.kind = rp.office ? RoomKind::Office : RoomKind::Flat;
            r.rect = {P(ra, dIn), P(rb, dIn), P(rb, dOut), P(ra, dOut)};
            rp.rooms.push_back(r);
            RoomWall front;
            front.a = P(ra, dOut);
            front.b = P(rb, dOut);
            front.doorAt = (rp.rooms.size() % 2) ? 0.35 : 0.65;
            front.glass = params.curtainWall;
            rp.walls.push_back(front);
            if (i + 2 < cuts.size()) {
                RoomWall side;
                side.a = P(rb, dIn);
                side.b = P(rb, dOut);
                side.accent = true;
                rp.walls.push_back(side);
            }
        }
        firstW = cuts[1] - cuts[0];
        lastW = cuts[cuts.size() - 1] - cuts[cuts.size() - 2];
    };

    // THE ARCS (the section's rounded corners, tessellated to chords; Glenn, 2026-10-15: "the corner office was not
    // the entire rounded section as a walled off space"): a run of two or more chords of one length turning one
    // way by one angle under 46 degrees, sweeping no more than a half circle (a drum's full ring stays as it was),
    // is ONE band -- its rooms follow the curve, the outside the chords, the inside the offset curve, the door on
    // the inner middle, the side walls radial. A plan without such a run lays exactly as before.
    struct Band { bool arc = false; bool wavy = false; std::size_t e = 0; std::size_t first = 0, count = 0; Real depth = 0; bool rooms = false; };
    std::vector<Band> bands;
    {
        std::vector<Real> len(n), turn(n);   // turn[i]: from edge i into edge i + 1
        for (std::size_t i = 0; i < n; ++i) {
            len[i] = E[i].W;
            const Vec2 d0 = E[i].d, d1 = E[(i + 1) % n].d;
            turn[i] = std::atan2(cross(d0, d1), dot(d0, d1));
        }
        auto chordLike = [&](std::size_t i, std::size_t j) {   // edge j continues edge i's arc
            const Real t0 = turn[i], t1 = turn[(j + n - 1) % n];
            return len[i] > 1e-6 && std::fabs(len[j] - len[i]) < 0.02 * len[i] && t0 > 0.0175 && t0 < 0.80 &&
                   std::fabs(t1 - t0) < 0.0175;
        };
        std::vector<int> group(n, -1);
        std::vector<Band> arcs;
        for (std::size_t i = 0; i < n; ++i) {
            if (group[i] >= 0) continue;
            // the run from i: edges i, i+1, ... while each continues the last
            std::size_t count = 1;
            Real sweep = 0;
            while (count < n) {
                const std::size_t j = (i + count) % n;
                if (group[j] >= 0 || !chordLike((i + count - 1) % n, j)) break;
                sweep += turn[(i + count - 1) % n];
                if (sweep > 3.1416 + 0.02) { sweep -= turn[(i + count - 1) % n]; break; }
                ++count;
            }
            if (count < 2) continue;
            // the run must not continue backwards from i either (i is its true start)
            if (chordLike((i + n - 1) % n, i) && group[(i + n - 1) % n] < 0 && sweep + turn[(i + n - 1) % n] <= 3.1416 + 0.02) continue;
            Band b;
            b.arc = true;
            b.first = i;
            b.count = count;
            for (std::size_t k = 0; k < count; ++k) group[(i + k) % n] = static_cast<int>(arcs.size());
            arcs.push_back(b);
        }
        // A WAVY RUN (an implicit form's skin, implicit.h): edges turning gently either way, under 35 degrees at
        // every vertex, are one curved band too -- its rooms follow the undulation with flat backs, a room's width
        // along the run, where the straight bands' corner claims (made for right angles) ran rooms into each other.
        auto gentle = [&](std::size_t i, std::size_t j) {   // edge j continues edge i's run
            return len[i] >= 1.0 && len[j] >= 1.0 && std::fabs(turn[(j + n - 1) % n]) < 0.61 && std::fabs(turn[(j + n - 1) % n]) > 0.0175;
        };
        for (std::size_t i = 0; i < n; ++i) {
            if (group[i] >= 0) continue;
            std::size_t count = 1;
            while (count < n) {
                const std::size_t j = (i + count) % n;
                if (group[j] >= 0 || !gentle((i + count - 1) % n, j)) break;
                ++count;
            }
            if (count < 3) continue;
            if (group[(i + n - 1) % n] < 0 && gentle((i + n - 1) % n, i)) continue;   // not its true start
            Band b;
            b.arc = true;
            b.wavy = true;
            b.first = i;
            b.count = count;
            for (std::size_t k = 0; k < count; ++k) group[(i + k) % n] = static_cast<int>(arcs.size());
            arcs.push_back(b);
        }
        std::sort(arcs.begin(), arcs.end(), [](const Band& a, const Band& b) { return a.first < b.first; });
        for (std::size_t i = 0; i < n; ++i) group[i] = -1;
        for (std::size_t a = 0; a < arcs.size(); ++a)
            for (std::size_t k = 0; k < arcs[a].count; ++k) group[(arcs[a].first + k) % n] = static_cast<int>(a);
        if (arcs.empty()) {
            for (std::size_t e = 0; e < n; ++e) { Band b; b.e = e; b.depth = E[e].depth; b.rooms = E[e].rooms; bands.push_back(b); }
        } else {
            // the sequence of bands round the plan, starting at the first arc so the arcs lay first (primary)
            const std::size_t s0 = arcs.front().first;
            for (std::size_t k = 0; k < n;) {
                const std::size_t e = (s0 + k) % n;
                if (group[e] >= 0) {
                    Band b = arcs[static_cast<std::size_t>(group[e])];
                    b.depth = 1e9;
                    for (std::size_t m = 0; m < b.count; ++m) {
                        const Edge& ed = E[(b.first + m) % n];
                        b.depth = std::min(b.depth, ed.depth);
                    }
                    b.rooms = b.depth >= kMinDepth;
                    bands.push_back(b);
                    k += b.count;
                } else {
                    Band b; b.e = e; b.depth = E[e].depth; b.rooms = E[e].rooms; bands.push_back(b); ++k;
                }
            }
        }
    }

    // A CURVED BAND's rooms: the corner office real towers draw. The glass follows the curve outside; the back
    // wall is a straight partition parallel to the arc's chord with the door in its middle; the side walls are
    // perpendicular to that chord. An arc sweeping more than a right angle (a stadium's end) is split into pieces of
    // a right angle at most, one such room each. The furniture's rectangle is the chord rectangle itself.
    // a plan with a wavy band: every curved band's pieces take radial sides, mitred with the edges beyond the band's
    // ends too, so pieces of neighbouring bands meet along one line (a sine's symmetric chords read as a short arc
    // beside the wave; square sides on it cut into the wave's first piece)
    const bool wavyPlan = std::any_of(bands.begin(), bands.end(), [](const Band& w) { return w.wavy; });
    auto layArc = [&](const Band& b) {
        const std::size_t m = b.count;
        const bool radial = b.wavy || wavyPlan;
        std::vector<Vec2> v(m + 1);
        for (std::size_t k = 0; k <= m; ++k) v[k] = plan[(b.first + k) % n];
        // the inward offset at every chord vertex: the bisector, mitred (the ends: the chord's own normal, or on a
        // wavy plan the mitre with the edge beyond)
        // ...where the edge beyond turns gently (the neighbouring curved band's chord); against a sharp straight
        // edge the mitre would throw the back corner across it, so the chord's own normal stays
        const Vec2 before = normalize(plan[b.first % n] - plan[(b.first + n - 1) % n]);
        const Vec2 after = normalize(plan[(b.first + m + 1) % n] - plan[(b.first + m) % n]);
        const Vec2 d0 = normalize(v[1] - v[0]), dm = normalize(v[m] - v[m - 1]);
        const bool mitreStart = radial && std::fabs(std::atan2(cross(before, d0), dot(before, d0))) < 0.61;
        const bool mitreEnd = radial && std::fabs(std::atan2(cross(dm, after), dot(dm, after))) < 0.61;
        std::vector<Vec2> in(m + 1);
        for (std::size_t k = 0; k <= m; ++k) {
            const Vec2 dPrev = k > 0 ? normalize(v[k] - v[k - 1]) : (mitreStart ? before : d0);
            const Vec2 dNext = k < m ? normalize(v[k + 1] - v[k]) : (mitreEnd ? after : dm);
            const Vec2 n0(-dPrev.y, dPrev.x), n1(-dNext.y, dNext.x);   // inward of a CCW chord: its left
            Vec2 bis = n0 + n1;
            const Real bl = bis.length();
            if (bl < 1e-6) { in[k] = n0; continue; }
            bis = bis * (1.0 / bl);
            in[k] = bis * (1.0 / std::max(Real(0.5), dot(bis, n0)));
        }
        // pieces of at most a right angle (an arc), or of a room's width along the run (a wavy band)
        std::vector<std::pair<std::size_t, std::size_t>> pieces;   // [k0, k1] vertex ranges
        {
            std::size_t k0 = 0;
            Real sweep = 0, run = 0;
            for (std::size_t k = 0; k + 1 < m; ++k) {
                const Vec2 d0 = normalize(v[k + 1] - v[k]), d1 = normalize(v[k + 2] - v[k + 1]);
                const Real t = std::atan2(cross(d0, d1), dot(d0, d1));
                run += (v[k + 1] - v[k]).length();
                // ...and an arc's piece no longer than two rooms along the glass (a drum's quarter was one office
                // with one desk in it -- Glenn, 2026-10-15); a rounded corner's quarter stays one corner office
                const bool full = b.wavy ? (std::fabs(sweep + t) > 1.5708 + 0.01 || run + (v[k + 2] - v[k + 1]).length() > widthWant * 1.4)
                                         : (sweep + t > 1.5708 + 0.01 || run + (v[k + 2] - v[k + 1]).length() > std::max(Real(9.6), widthWant * 2.0));
                if (full) { pieces.push_back({k0, k + 1}); k0 = k + 1; sweep = 0; run = 0; }
                sweep += t;
            }
            pieces.push_back({k0, m});
        }
        const Real dIn = inset;
        for (std::size_t pi = 0; pi < pieces.size(); ++pi) {
            const std::size_t k0 = pieces[pi].first, k1 = pieces[pi].second;
            const Vec2 A = v[k0] + in[k0] * dIn, B = v[k1] + in[k1] * dIn;
            const Real L = (B - A).length();
            if (L < 2.0) continue;
            const Vec2 c = normalize(B - A);
            const Vec2 nIn(-c.y, c.x);
            // The back wall: no deeper than the chord is wide, and measured from the PIECE'S chord -- which sits a
            // sagitta inside the glass -- it stops a corridor short of the core and short of the facing piece (a
            // lens tower's small storeys: a quarter-drum office whose back wall ran through the core, issue #105).
            const Real room = depthToCore(A, c, L, nIn * -1.0, core) - kCorridorRing;
            Real across = 0;
            for (const Vec2& w : plan) across = std::max(across, dot(w - A, nIn));
            const Real d = std::min({b.depth, L * 0.9, room, (across - 1.4) * 0.5});
            if (d < kMinDepth) continue;
            // the back corners: square to the chord (an arc's corner office), or along the mitred offsets at the
            // piece's ends (a wavy band: pieces turning both ways would overlap behind the glass otherwise)
            const Vec2 backA = radial ? v[k0] + in[k0] * (dIn + d) : A + nIn * d;
            const Vec2 backB = radial ? v[k1] + in[k1] * (dIn + d) : B + nIn * d;
            Room r;
            r.edge = (b.first + k0) % n;
            r.kind = rp.office ? RoomKind::Office : RoomKind::Flat;
            r.rect.push_back(A);
            for (std::size_t k = k0 + 1; k < k1; ++k) r.rect.push_back(v[k] + in[k] * dIn);
            r.rect.push_back(B);
            r.rect.push_back(backB);
            r.rect.push_back(backA);
            r.frame = {A, B, B + nIn * d, A + nIn * d};
            rp.rooms.push_back(r);
            RoomWall back;   // the straight partition, the door in its middle
            back.a = backB;
            back.b = backA;
            back.doorAt = (rp.rooms.size() % 2) ? 0.35 : 0.65;
            back.glass = params.curtainWall;
            rp.walls.push_back(back);
            RoomWall sideA;   // the side at the piece's start (the last piece's end too)
            sideA.a = A;
            sideA.b = backA;
            sideA.accent = true;
            rp.walls.push_back(sideA);
            if (pi + 1 == pieces.size()) {
                RoomWall sideB;
                sideB.a = B;
                sideB.b = backB;
                sideB.accent = true;
                rp.walls.push_back(sideB);
            }
        }
    };

    // The straight bands: as before, except beside an ARC, whose corner room's back corner reaches `depth` along
    // the straight edge from the tangent point -- the straight band starts past it, and walls its first room's
    // side there.
    const std::size_t nb = bands.size();
    std::vector<Real> cornerStart(nb, 0.0), cornerEnd(nb, 0.0);
    auto layStraight = [&](std::size_t bi, bool primary) {
        const Band& b = bands[bi];
        const std::size_t e = b.e;
        const Band& prevB = bands[(bi + nb - 1) % nb];
        const Band& nextB = bands[(bi + 1) % nb];
        const bool prevArc = prevB.arc && prevB.rooms, nextArc = nextB.arc && nextB.rooms;
        Real startAt = inset, endAt = E[e].W - inset;
        Real claimStart = inset, claimEnd = inset;   // how far the neighbour's corner room already covers this end
        if (prevArc) startAt = inset + prevB.depth;
        if (nextArc) endAt = E[e].W - inset - nextB.depth;
        if (!primary) {
            const Vec2 prevD = prevB.arc ? E[(prevB.first + prevB.count - 1) % n].d : E[prevB.e].d;
            const Vec2 nextD = nextB.arc ? E[nextB.first].d : E[nextB.e].d;
            const bool convexStart = !prevArc && cross(prevD, E[e].d) > 1e-6;
            const bool convexEnd = !nextArc && cross(E[e].d, nextD) > 1e-6;
            if (prevB.rooms && convexStart) { startAt = inset + prevB.depth; claimStart = inset + cornerEnd[(bi + nb - 1) % nb]; }
            if (nextB.rooms && convexEnd) { endAt = E[e].W - inset - nextB.depth; claimEnd = inset + cornerStart[(bi + 1) % nb]; }
        }
        auto P = [&](Real x, Real depth) { return E[e].a + E[e].d * x - E[e].nOut * depth; };
        // BESIDE A WAVY BAND (laid already) the claim is measured, not assumed: the band starts and ends where its
        // strip is clear of the wavy band's rooms and inside the undulating outline (a corner room of a straight
        // band ran past where the skin dipped, and under a wavy piece's oblique back).
        auto clearAt = [&](Real x) {
            const Vec2 pts[3] = {P(x, inset + 0.3), P(x, inset + E[e].depth * 0.5), P(x, inset + E[e].depth)};
            for (const Vec2& q : pts) {
                if (!pointInPolygon(plan, q)) return false;
                for (const Room& r : rp.rooms)
                    if (r.rect.size() >= 3 && pointInPolygon(r.rect, q)) return false;
            }
            return true;
        };
        // (an arc beside a wavy outline is measured too: a sine's symmetric chords read as a short arc)
        if (prevB.arc && (prevB.wavy || wavyPlan)) { Real x = startAt; while (x < endAt && !clearAt(x)) x += 0.25; startAt = x; claimStart = inset; }
        if (nextB.arc && (nextB.wavy || wavyPlan)) { Real x = endAt; while (x > startAt && !clearAt(x)) x -= 0.25; endAt = x; claimEnd = inset; }
        if (std::getenv("RT_RING_DEBUG"))
            std::fprintf(stderr, "[ring]   straight band %zu edge %zu: span %.1f..%.1f of %.1f (prev %s next %s, %zu rooms laid)\n", bi, e, startAt, endAt, E[e].W,
                         prevB.arc ? "curved" : "straight", nextB.arc ? "curved" : "straight", rp.rooms.size());
        Real fw, lw;
        lay(e, startAt, endAt, fw, lw);
        cornerStart[bi] = fw;
        cornerEnd[bi] = lw;
        E[e].cornerStart = fw;
        E[e].cornerEnd = lw;
        if (fw <= 0) return;
        auto endWall = [&](Real x, Real coveredTo) {
            const Real from = std::max(inset, coveredTo), to = inset + E[e].depth;
            if (to - from < 0.3) return;
            RoomWall w;
            w.a = P(x, from);
            w.b = P(x, to);
            rp.walls.push_back(w);
        };
        // a secondary edge's end rooms are always walled at their ends, as before; a primary edge's only beside an arc
        if (!primary || prevArc) endWall(startAt, claimStart);
        if (!primary || nextArc) endWall(endAt, claimEnd);
    };
    if (std::getenv("RT_RING_DEBUG")) {
        std::fprintf(stderr, "[ring] storey %d plan %zu edges, %zu bands:", storey, n, nb);
        for (std::size_t bi = 0; bi < nb; ++bi) {
            const Band& b = bands[bi];
            std::fprintf(stderr, " %zu:%s%s e%zu+%zu d%.1f%s", bi, b.arc ? "arc" : "str", b.wavy ? "(wavy)" : "", b.arc ? b.first : b.e, b.arc ? b.count : std::size_t(1), b.depth, b.rooms ? "" : " norooms");
        }
        std::fprintf(stderr, "\n");
    }
    // the curved bands first, wherever they sit (a straight band beside one measures its claim against their rooms),
    // then the straight ones, primary (even) before secondary (odd) as always
    for (std::size_t bi = 0; bi < nb; ++bi) {
        const Band& b = bands[bi];
        if (b.rooms && b.arc) layArc(b);
    }
    for (std::size_t bi = 0; bi < nb; bi += 2) {
        const Band& b = bands[bi];
        if (!b.rooms || b.arc) continue;
        layStraight(bi, true);
    }
    for (std::size_t bi = 1; bi < nb; bi += 2) {
        const Band& b = bands[bi];
        if (!b.rooms || b.arc) continue;
        layStraight(bi, false);
    }
    return rp;
}

// PRIVATE OFFICES ALONG THE GLASS (Glenn, 2026-10-13: "no office partitions in the office?"). A desk-row strip deep
// enough for an office AND the corridor ring behind it is a row of offices instead of open desks: each about two
// curtain bays wide (a partition on a mullion), fronted in glass with its door onto the corridor, drywall between.
// `T` is the strip's depth in from the wall's inner face, `P` maps (along the edge, in from the wall line) to the
// plan, [x0, x1] the span; a span end short of the exterior gets an end wall. False when too shallow: nothing added.
namespace {
constexpr Real kStripOfficeMin = 2.8;   // a desk, its chair, the door swing: about 9 m^2 at two bays wide
constexpr Real kStripOfficeMax = 4.0;
constexpr Real kStripOfficeW = 3.2;
}  // namespace
// The row itself, `od` deep from the wall's inner face (an office floor's band lays its perimeter offices with it too).
// (A hospital's wards and examination rooms are the same row: `kind`, about `wantW` wide.)
static void officeRowAt(RoomPlan& rp, const BuildingParams& params, std::size_t e, Real W,
                        const std::function<Vec2(Real, Real)>& P, Real x0, Real x1, Real inset, Real od,
                        RoomKind kind = RoomKind::Office, Real wantW = kStripOfficeW) {
    std::vector<Real> cuts{x0};
    Real last = x0;
    for (Real xb : bayBoundaries(W, params)) {
        if (xb <= x0 + 1.0 || xb >= x1 - 1.0) continue;
        if (xb - last >= wantW * 0.85) { cuts.push_back(xb); last = xb; }
    }
    if (x1 - last < wantW * 0.6 && cuts.size() > 1) cuts.pop_back();   // a sliver joins its neighbour
    cuts.push_back(x1);
    const Real vIn = inset, vF = inset + od;
    for (std::size_t i = 0; i + 1 < cuts.size(); ++i) {
        const Real a = cuts[i], b = cuts[i + 1];
        Room r;
        r.edge = e;
        r.kind = kind;
        r.rect = {P(a, vIn), P(b, vIn), P(b, vF), P(a, vF)};
        rp.rooms.push_back(r);
        RoomWall front;
        front.a = P(a, vF);
        front.b = P(b, vF);
        front.doorAt = (rp.rooms.size() % 2) ? 0.3 : 0.7;
        front.glass = params.curtainWall;
        rp.walls.push_back(front);
        if (i + 2 < cuts.size()) {
            RoomWall side;
            side.a = P(b, vIn);
            side.b = P(b, vF);
            side.accent = true;
            rp.walls.push_back(side);
        }
    }
    for (Real x : {x0, x1})
        if (x > inset + 0.05 && x < W - inset - 0.05) {
            RoomWall w;
            w.a = P(x, vIn);
            w.b = P(x, vF);
            rp.walls.push_back(w);
        }
}
static bool officeRow(RoomPlan& rp, const BuildingParams& params, std::size_t e, Real W,
                      const std::function<Vec2(Real, Real)>& P, Real x0, Real x1, Real inset, Real T) {
    const Real od = std::min(T - kCorridorRing, kStripOfficeMax);
    if (od < kStripOfficeMin || x1 - x0 < 2.4) return false;
    officeRowAt(rp, params, e, W, P, x0, x1, inset, od);
    return true;
}

// THE DESK ROW (Glenn, 2026-10-13: "some floors in skyscrapers are still empty"): a setback tier keeps the core it
// was sized for at the base, so the floor between the core and the glass narrows -- a 15.9 m art deco crown round a
// 10.5 x 7.2 m core leaves 2-4 m a side, too shallow for the ring's rooms (kMinDepth past a 2.2 m corridor) or the
// office floor's 4 m bands, and the floor grew empty. Such a floor is an open office: along each edge with the depth
// for it, a row of desks at the windows, its chairs facing out, the walk to the core behind them, low screens between
// the desks -- or, where the strip is deep enough for a room and the corridor, private offices (officeRow). The
// corners stay open (each strip stops short of its neighbours' -- enough at any corner of 90 degrees or more, so a
// chamfered plate is fine too). A
// residential tower's such floor is its residents' LOUNGE: sofas along the glass instead (`kind` Lounge).
static RoomPlan deskRowPlan(const Poly2& planIn, const BuildingParams& params, const CorePlan& core, Real inset,
                            int storey, RoomKind kind) {
    // the window gap, a desk and its chair (a sofa and its table), the walk behind: desk_row's and lounge's grids
    const Real kRowMin = kind == RoomKind::Lounge ? 2.85 : kDeskRowMin;
    constexpr Real kRowMax = kDeskRowMax;
    RoomPlan rp;
    rp.topology = PlateTopology::Ring;
    rp.office = kind == RoomKind::DeskRow;
    if (storey < 1 || !core.valid || planIn.size() < 4) return rp;
    Poly2 plan = planIn;
    ensureCCW(plan);
    const std::size_t n = plan.size();
    struct Strip { Vec2 a, d, nOut; Real W = 0, depth = 0; bool row = false; };
    std::vector<Strip> S(n);
    for (std::size_t e = 0; e < n; ++e) {
        Strip& s = S[e];
        s.a = plan[e];
        const Vec2 dv = plan[(e + 1) % n] - s.a;
        s.W = dv.length();
        if (s.W < 1e-6) continue;
        s.d = dv * (1.0 / s.W);
        s.nOut = Vec2(s.d.y, -s.d.x);
        s.depth = std::min(depthToCore(s.a, s.d, s.W, s.nOut, core) - inset, kRowMax);
        s.row = s.depth >= kRowMin;
    }
    for (std::size_t e = 0; e < n; ++e) {
        const Strip& s = S[e];
        if (!s.row) continue;
        const Strip& prev = S[(e + n - 1) % n];
        const Strip& next = S[(e + 1) % n];
        Real x0 = inset + (prev.row ? prev.depth : 0), x1 = s.W - inset - (next.row ? next.depth : 0);
        if (x1 - x0 < 2.0) continue;
        auto P = [&](Real x, Real in) { return s.a + s.d * x - s.nOut * in; };
        // ...and never past the storey's own skin (an implicit form's wavy outline, issue #105): the strip's corners
        // at both ends inside the outline, or the span steps in until they are
        auto insideAt = [&](Real x) {
            for (Real in : {inset + 0.3, inset + s.depth})
                if (!pointInPolygon(plan, P(x, in))) return false;
            return true;
        };
        auto trim = [&](Real& a, Real& b) {
            while (a < b && !insideAt(a)) a += 0.25;
            while (b > a && !insideAt(b)) b -= 0.25;
        };
        trim(x0, x1);
        if (x1 - x0 < 2.0) continue;
        if (kind == RoomKind::DeskRow) {
            // Offices where the strip is deep enough. An even strip's row runs on into a square corner (the odd
            // neighbour's strip already stops short of this one's whole depth), so the corner is an office too.
            const std::size_t pe = (e + n - 1) % n, ne = (e + 1) % n;
            const bool squareStart = e % 2 == 0 && pe % 2 == 1 && std::fabs(dot(prev.d, s.d)) < 0.1;
            const bool squareEnd = e % 2 == 0 && ne % 2 == 1 && std::fabs(dot(next.d, s.d)) < 0.1;
            Real r0 = squareStart ? inset : x0, r1 = squareEnd ? s.W - inset : x1;
            trim(r0, r1);
            if (officeRow(rp, params, e, s.W, P, r0, r1, inset, s.depth))
                continue;
        }
        Room r;
        r.edge = e;
        r.kind = kind;
        r.rect = {P(x0, inset), P(x1, inset), P(x1, inset + s.depth), P(x0, inset + s.depth)};
        rp.rooms.push_back(r);
    }
    return rp;
}

// ------------------------------------------------------------- the house
RoomPlan transferLoungePlan(const Poly2& planIn, const BuildingParams& params, Real inset, int storey,
                            const std::vector<Poly2>& keepOut, const std::vector<std::size_t>& blankEdges) {
    RoomPlan rp;
    rp.topology = PlateTopology::Ring;
    rp.office = false;
    if (storey < 1 || planIn.size() < 3) return rp;
    Poly2 plan = planIn;
    ensureCCW(plan);
    const std::size_t n = plan.size();
    const bool concave = !convexPlan(plan);
    const Real kRowMin = 2.85, kRowMax = kDeskRowMax;
    for (std::size_t e = 0; e < n; ++e) {
        if (std::find(blankEdges.begin(), blankEdges.end(), e) != blankEdges.end()) continue;
        const Vec2 a = plan[e];
        const Vec2 dv = plan[(e + 1) % n] - a;
        const Real W = dv.length();
        if (W < 2.0 * inset + 2.0) continue;
        const Vec2 d = dv * (1.0 / W);
        const Vec2 nOut(d.y, -d.x);
        // how deep the strip may go: half the floor across, less a walk in the middle
        Real across = 0;
        for (const Vec2& w : plan) across = std::max(across, dot(w - a, nOut * -1.0));
        if (concave) {
            Real into = 1e9;
            for (int si = 0; si < 5; ++si) {
                const Real x = inset + 0.5 + std::max(Real(0), W - 2.0 * inset - 1.0) * (si / 4.0);
                if (x > W) continue;
                into = std::min(into, depthInto(plan, a + d * x, nOut * -1.0, e));
            }
            if (into < 1e8) across = std::min(across, into);
        }
        const Real depth = std::min(kRowMax, (across - 2.0 * inset - 2.0) * 0.5);
        if (depth < kRowMin) continue;
        auto P = [&](Real x, Real in) { return a + d * x - nOut * in; };
        // the strip, cut where a well stands in it: the runs of the edge clear of every keep-out
        std::vector<std::pair<Real, Real>> busy;
        for (const Poly2& ko : keepOut) {
            if (ko.size() < 3) continue;
            Real lo = 1e9, hi = -1e9;
            bool near = false;
            for (const Vec2& q : ko) {
                const Real in = dot(a - q, nOut), x = dot(q - a, d);
                if (in > -0.5 && in < inset + depth + 0.5) near = true;
                lo = std::min(lo, x); hi = std::max(hi, x);
            }
            if (near && hi > 0 && lo < W) busy.push_back({lo - 0.6, hi + 0.6});
        }
        std::sort(busy.begin(), busy.end());
        Real x0 = inset;
        auto strip = [&](Real s0, Real s1) {
            if (s1 - s0 < 3.0) return;
            Room r;
            r.edge = e;
            r.kind = RoomKind::Lounge;
            r.rect = {P(s0, inset), P(s1, inset), P(s1, inset + depth), P(s0, inset + depth)};
            rp.rooms.push_back(r);
        };
        for (const auto& [b0, b1] : busy) { strip(x0, std::min(b0, W - inset)); x0 = std::max(x0, b1); }
        strip(x0, W - inset);
    }
    (void)params;
    return rp;
}

namespace {

constexpr Real kHallW = 1.4;       // clear width of the hall / landing
constexpr Real kMinRoomW = 2.4;    // a room narrower than this is not a room
constexpr Real kMinRoomD = 2.2;    // ...nor shallower
constexpr Real kStairSnug = 0.15;  // the side the stair hugs: just a margin
constexpr Real kStairPass = 1.10;  // the other side: a PASSAGE, wider than the player
constexpr Real kPlayerR = 0.30;    // the capsule's radius: what a doorway must pass

struct ProgramEntry { RoomKind kind; Real weight; Real minW; };

// What a floor of a house holds. The ground floor is where you live; the
// floors above are bedrooms off the landing. Entries come out biggest
// first, so a small plate drops the study before it drops the bathroom.
std::vector<ProgramEntry> houseProgram(int storey, Real area) {
    std::vector<ProgramEntry> prog;
    if (storey == 0) {
        prog.push_back({RoomKind::Living, 1.8, 3.0});
        prog.push_back({RoomKind::Kitchen, 1.3, 2.6});
        if (area > 68.0) prog.push_back({RoomKind::Bed, 1.1, 2.6});   // a study, or a ground bedroom
        prog.push_back({RoomKind::Bath, 0.7, 2.0});
    } else {
        const int beds = std::max(1, std::min(4, static_cast<int>(std::lround(area / 24.0))));
        for (int i = 0; i < beds; ++i) prog.push_back({RoomKind::Bed, 1.2, 2.6});
        prog.push_back({RoomKind::Bath, 0.7, 2.0});
    }
    return prog;
}

}  // namespace

// THE WHOLE FLOOR: one dwelling. A hall crosses the plate at the stair,
// rooms fill the bands either side of it, each with a door into the hall.
static RoomPlan housePlan(const Poly2& planIn, const BuildingParams& params, const Poly2& well,
                          std::size_t entranceEdge, Real inset, int storey) {
    RoomPlan rp;
    rp.topology = PlateTopology::WholeFloor;
    if (planIn.size() < 3) return rp;
    Poly2 plan = planIn;
    ensureCCW(plan);
    const OBB2 ob = orientedBoundingBox(plan);
    Vec2 axA = ob.axis[0], axB = ob.axis[1];
    Real LA = 2.0 * ob.half[0] - 2.0 * inset, LB = 2.0 * ob.half[1] - 2.0 * inset;
    if (LA < kMinRoomW || LB < kMinRoomD + kHallW) return rp;
    // The free rectangle's corner. Swapping axis AND length below leaves
    // this same point, so it is computed once.
    Vec2 origin = ob.center - axA * (LA * 0.5) - axB * (LB * 0.5);
    // CLIP IT TO THE PLAN. A bounding box fits a rectangle exactly and a
    // trapezoid not at all: on a wedge-shaped plate the box hangs outside
    // the building, and rooms laid in it walked straight through the facade
    // (Glenn, 2026-09-15: "the walls immediately jutted out of the building").
    for (int guard = 0; guard < 60; ++guard) {
        bool inside = true;
        for (Real ua : {0.0, 1.0})
            for (Real vb : {0.0, 1.0})
                if (!pointInPolygon(plan, origin + axA * (ua * LA) + axB * (vb * LB))) inside = false;
        if (inside) break;
        // Pull the LONG axis in first. A wedge overhangs sideways, and
        // shrinking both axes evenly throws away the depth the rooms need:
        // a trapezoid came back with no rooms at all (2026-09-15).
        const Vec2 c = origin + axA * (LA * 0.5) + axB * (LB * 0.5);
        if (LA > kMinRoomW + 1.0) LA -= 0.15;
        else LB -= 0.15;
        origin = c - axA * (LA * 0.5) - axB * (LB * 0.5);
        if (LA < kMinRoomW || LB < kMinRoomD + kHallW) return rp;
    }
    if (LA < kMinRoomW || LB < kMinRoomD + kHallW) return rp;

    // The stair's footprint in local (a, b).
    Real wA0 = 0, wA1 = 0, wB0 = 0, wB1 = 0;
    const bool haveWell = well.size() >= 3;
    if (haveWell) {
        wA0 = wB0 = 1e9;
        wA1 = wB1 = -1e9;
        for (const Vec2& w : well) {
            const Real a = dot(w - origin, axA), b = dot(w - origin, axB);
            wA0 = std::min(wA0, a); wA1 = std::max(wA1, a);
            wB0 = std::min(wB0, b); wB1 = std::max(wB1, b);
        }
        // The hall runs along the axis the stair is THIN across, so a stair
        // hugging a short wall cannot swallow the floor.
        if (wA1 - wA0 < wB1 - wB0) {
            std::swap(axA, axB);
            std::swap(LA, LB);
            std::swap(wA0, wB0);
            std::swap(wA1, wB1);
        }
    }
    if (LA < kMinRoomW || LB < kMinRoomD + kHallW) return rp;
    auto toWorld = [&](Real a, Real b) { return origin + axA * a + axB * b; };

    // The hall band, in b: over the stair when there is one, else central.
    // The stair needs a PASSAGE, not a margin. A 0.45 m strip either side is
    // narrower than the player is wide, so the hall was a dead end at the
    // well and half the floor was unreachable (Glenn, 2026-09-15: "hallways
    // with stair wells need to be wider"). Keep the hall tight on the side
    // the stair hugs and put the whole passage on the other.
    Real hall0, hall1;
    if (haveWell) {
        const bool passAbove = (LB - wB1) >= wB0;
        hall0 = std::max(Real(0), wB0 - (passAbove ? kStairSnug : kStairPass));
        hall1 = std::min(LB, wB1 + (passAbove ? kStairPass : kStairSnug));
    } else {
        hall0 = (LB - kHallW) * 0.5;
        hall1 = hall0 + kHallW;
    }
    if (hall1 - hall0 < kHallW) {
        const Real c = std::min(std::max((hall0 + hall1) * 0.5, kHallW * 0.5), LB - kHallW * 0.5);
        hall0 = c - kHallW * 0.5;
        hall1 = c + kHallW * 0.5;
    }
    const bool bandLo = hall0 >= kMinRoomD;
    const bool bandHi = LB - hall1 >= kMinRoomD;
    if (!bandLo && !bandHi) return rp;        // a plate this small stays one room

    // The front door, when it lands in a band rather than at the hall's end:
    // no partition may cross it.
    Real doorA = -1;
    if (entranceEdge < plan.size()) {
        const Vec2 m = (plan[entranceEdge] + plan[(entranceEdge + 1) % plan.size()]) * 0.5;
        const Real b = dot(m - origin, axB);
        if (b < hall0 - 0.05 || b > hall1 + 0.05) doorA = dot(m - origin, axA);
    }

    // Balance the program across the two bands by weight, respecting how
    // many minimum widths a band can actually hold.
    std::vector<ProgramEntry> lo, hi;
    Real wLo = 0, wHi = 0, mLo = 0, mHi = 0;
    for (const ProgramEntry& e : houseProgram(storey, LA * LB)) {
        const bool preferLo = bandLo && (!bandHi || wLo <= wHi);
        if (preferLo && mLo + e.minW <= LA) { lo.push_back(e); wLo += e.weight; mLo += e.minW; }
        else if (bandHi && mHi + e.minW <= LA) { hi.push_back(e); wHi += e.weight; mHi += e.minW; }
        else if (bandLo && mLo + e.minW <= LA) { lo.push_back(e); wLo += e.weight; mLo += e.minW; }
    }

    auto layBand = [&](const std::vector<ProgramEntry>& es, Real b0, Real b1, bool hallAbove) {
        if (es.empty() || b1 - b0 < kMinRoomD) return;
        Real total = 0;
        for (const ProgramEntry& e : es) total += e.weight;
        Real a = 0;
        for (std::size_t i = 0; i < es.size(); ++i) {
            Real a1 = LA;
            if (i + 1 < es.size()) {
                Real restMin = 0;
                for (std::size_t j = i + 1; j < es.size(); ++j) restMin += es[j].minW;
                a1 = a + std::max(es[i].minW, LA * (es[i].weight / total));
                a1 = std::min(a1, LA - restMin);
                if (doorA >= 0 && std::fabs(a1 - doorA) < 0.8) {   // keep the front door clear
                    const Real lower = doorA - 0.8;
                    a1 = lower >= a + es[i].minW ? lower : doorA + 0.8;
                    a1 = std::min(std::max(a1, a + es[i].minW), LA - restMin);
                }
                // SNAP TO A PIER. A partition meets the outside wall
                // somewhere; landing mid-window cuts the window in half
                // (Glenn, 2026-09-15: "some walls bisect a window"). The
                // facade's own bay boundaries are the piers between windows,
                // so snap to the nearest one within half a bay.
                {
                    const Vec2 hit = toWorld(a1, b0);
                    Real bestD = 1e9, bestShift = 0;
                    for (std::size_t e = 0; e < plan.size(); ++e) {
                        const Vec2 ea = plan[e], eb = plan[(e + 1) % plan.size()];
                        const Vec2 ev = eb - ea;
                        const Real eL = ev.length();
                        if (eL < 1e-6) continue;
                        const Vec2 ed = ev * (1.0 / eL);
                        if (std::fabs(dot(ed, axA)) < 0.9) continue;      // not parallel to the band
                        const Real s = dot(hit - ea, ed);
                        if (s < -0.5 || s > eL + 0.5) continue;
                        const Real perp = std::fabs(dot(hit - ea, Vec2(ed.y, -ed.x)));
                        if (perp > inset + 1.2) continue;                 // not this band's wall
                        for (Real xb : bayBoundaries(eL, params)) {
                            const Real shift = (xb - s) * (dot(ed, axA) > 0 ? 1.0 : -1.0);
                            if (std::fabs(shift) < std::fabs(bestD)) { bestD = shift; bestShift = shift; }
                        }
                    }
                    if (std::fabs(bestD) < std::max(params.bayWidth, Real(1.0)) * 0.5) {
                        const Real snapped = a1 + bestShift;
                        if (snapped >= a + es[i].minW && snapped <= LA - restMin &&
                            (doorA < 0 || std::fabs(snapped - doorA) >= 0.8))
                            a1 = snapped;
                    }
                }
            }
            if (a1 <= a + 0.05) continue;
            Room r;
            r.kind = es[i].kind;
            r.rect = {toWorld(a, b0), toWorld(a1, b0), toWorld(a1, b1), toWorld(a, b1)};
            ensureCCW(r.rect);
            rp.rooms.push_back(r);
            RoomWall front;                       // onto the hall, with its door
            front.a = toWorld(a, hallAbove ? b1 : b0);
            front.b = toWorld(a1, hallAbove ? b1 : b0);
            front.doorAt = (i % 2) ? 0.38 : 0.62;
            rp.walls.push_back(front);
            if (i + 1 < es.size()) {              // the partition to the next room
                RoomWall part;
                part.a = toWorld(a1, b0);
                part.b = toWorld(a1, b1);
                part.accent = true;                // ...and it wears the finish
                rp.walls.push_back(part);
            }
            a = a1;
        }
    };
    layBand(lo, 0, hall0, true);
    layBand(hi, hall1, LB, false);
    return rp;
}

// ------------------------------------------------------------- the apartments
// WHOLE-FLOOR APARTMENTS (Glenn, 2026-09-30: "you could pull the walls forward and make entire apartments out of
// the floor and then subdivide the apartments into living rooms, bedrooms, kitchens, bathrooms, closets ... then
// naturally there would be hallway where the stairwell or elevators would be"). A residential tower's plate:
//
//   +---------+-----------+---------+       a CORRIDOR rings the core; every band between it and the facade
//   |  bed |  living     | bed |   |       is cut into apartments, each with its door on the corridor. In one:
//   |------+--+=====+----+-----|   |       the window side holds the living room and the bedrooms (doors off
//   | bath |hall| kitchen | clo |   |       the living room); a back strip along the corridor holds the entry
//   +------+-=-+---------+-----+---+       hall, the bath (door off the hall), a closet, the kitchen (open to
//          corridor                         the living room). Party walls between apartments.
//
// Rectangular plates with a core only; the even edges' bands take the corners, an odd band starts past them
// (ringPlan's rule), and a band's corner apartment stretches until its front reaches the corridor.
namespace {
constexpr Real kCorridor = 2.2;   // core_plan.cpp's kCorridor: the ring kept clear round the core
}

// ONE APARTMENT (buildings B): the unit x0..x1 along a band, from the window wall (depth vIn) to its front on the
// corridor (vFront, the band's depth D), its front door at xd -- laid out into rooms and walls. P maps (along the
// band, depth in from the window wall) to the plan. Shared by the tower floor and the walk-up.
void layoutApartment(RoomPlan& rp, std::size_t e, const std::function<Vec2(Real, Real)>& P, Real x0, Real x1,
                     Real vIn, Real vFront, Real D, Real xd, bool partyWallRight, bool frontWall) {
    const Real kHall = 1.6, kBath = 2.3, kCloset = std::max(Real(1.4), kRoomDoorW + 0.7), kBedW = 3.4, kLivingMin = 4.2;
    const Real UW = x1 - x0;
        // The zones: the back strip along the corridor and the window side.
        const Real bz = std::clamp(D * 0.32, Real(2.3), Real(2.8));
        const Real vb = vFront - bz;
        // BACK STRIP: the hall at the door, the bath beside it on the roomier side, the closet on the other,
        // the kitchen in the largest remainder (else it joins the living room on the window side).
        const Real hx0 = std::clamp(xd - kHall * 0.5, x0, x1 - kHall), hx1 = hx0 + kHall;
        struct Seg { Real x0, x1; RoomKind kind; };
        std::vector<Seg> back;
        back.push_back({hx0, hx1, RoomKind::Hall});
        const Real leftW = hx0 - x0, rightW = x1 - hx1;
        const bool bathLeft = leftW >= rightW;
        Real l = hx0, r = hx1;   // the frontier on each side
        auto take = [&](bool left, Real w, RoomKind kind) {
            if (left) { if (l - x0 < w - 1e-6) return false; back.push_back({l - w, l, kind}); l -= w; }
            else { if (x1 - r < w - 1e-6) return false; back.push_back({r, r + w, kind}); r += w; }
            return true;
        };
        // The bath beside the hall on the roomier side; the KITCHEN beside the hall on the other (open to it,
        // so it is always reached), else it moves to the window side; the closet after whichever has room
        // (beside the hall it opens onto the hall, else onto the room in front of it).
        const bool bathOnLeft = take(bathLeft, kBath, RoomKind::Bath) ? bathLeft
                                : (take(!bathLeft, kBath, RoomKind::Bath), !bathLeft);
        bool kitchenBack = false;
        {
            const bool kLeft = !bathOnLeft;
            const Real room = kLeft ? l - x0 : x1 - r;
            if (room >= 2.4) {
                const Real kw = std::min(room, Real(3.4));
                take(kLeft, kw, RoomKind::Kitchen);
                kitchenBack = true;
            }
        }
        take(!bathOnLeft, kCloset, RoomKind::Closet) || take(bathOnLeft, kCloset, RoomKind::Closet);
        // A leftover sliver joins its neighbour: grow the room next to it.
        for (Seg& s : back) {
            if (l > x0 + 1e-6 && std::fabs(s.x0 - l) < 1e-6) s.x0 = x0;
            if (r < x1 - 1e-6 && std::fabs(s.x1 - r) < 1e-6) s.x1 = x1;
        }
        // WINDOW SIDE: bedrooms at the ends away from the hall, the living room between.
        int beds = UW >= 10.5 ? 2 : (UW >= 7.0 ? 1 : 0);
        // THE HALL FACES THE LIVING ROOM (the space graph's gate, issue #105, 2026-10-15: an end unit's hall opened onto
        // nothing but its bath -- with a bedroom at each end, one stood in front of the hall across the solid divider,
        // and the flat was a dead end). Where the hall sits within a bedroom's width of the unit's end, only the far
        // end takes a bedroom.
        const bool hallAtEnd = hx0 - x0 < kBedW - 1e-6 || x1 - hx1 < kBedW - 1e-6;
        if (beds == 2 && hallAtEnd) beds = 1;
        // A kitchen before a second bedroom: one that could not sit in the back strip needs the window side.
        const Real kitchenFront = kitchenBack ? 0.0 : 2.6;
        while (beds > 0 && UW - beds * kBedW - kitchenFront < kLivingMin) --beds;
        std::vector<Seg> front;
        Real f0 = x0, f1 = x1;
        const bool hallNearLeft = (xd - x0) < (x1 - xd);
        if (beds == 2) { front.push_back({x0, x0 + kBedW, RoomKind::Bed}); front.push_back({x1 - kBedW, x1, RoomKind::Bed}); f0 += kBedW; f1 -= kBedW; }
        else if (beds == 1) {
            if (hallNearLeft) { front.push_back({x1 - kBedW, x1, RoomKind::Bed}); f1 -= kBedW; }
            else { front.push_back({x0, x0 + kBedW, RoomKind::Bed}); f0 += kBedW; }
        }
        if (!kitchenBack && f1 - f0 >= kLivingMin + 2.6) {   // the kitchen takes the living room's far end
            if (hallNearLeft) { front.push_back({f1 - 2.6, f1, RoomKind::Kitchen}); f1 -= 2.6; }
            else { front.push_back({f0, f0 + 2.6, RoomKind::Kitchen}); f0 += 2.6; }
        }
        front.push_back({f0, f1, RoomKind::Living});
        // The rooms.
        for (const Seg& s : front) {
            Room rm; rm.edge = e; rm.kind = s.kind;
            rm.rect = {P(s.x0, vIn), P(s.x1, vIn), P(s.x1, vb), P(s.x0, vb)};
            rp.rooms.push_back(rm);
        }
        for (const Seg& s : back) {
            Room rm; rm.edge = e; rm.kind = s.kind;
            rm.rect = {P(s.x0, vb), P(s.x1, vb), P(s.x1, vFront), P(s.x0, vFront)};
            rp.rooms.push_back(rm);
        }
        // THE WALLS. The front, with the entry door.
        if (frontWall) {
            RoomWall w; w.a = P(x0, vFront); w.b = P(x1, vFront);
            w.doorAt = (xd - x0) / UW;
            rp.walls.push_back(w);
        }
        // The party wall at this unit's right end (the band's own ends are the exterior or the band's front).
        if (partyWallRight) {
            RoomWall w; w.a = P(x1, vIn); w.b = P(x1, vFront);
            rp.walls.push_back(w);
        }
        // Back-strip partitions: a door from the hall into the bath and the closet, solid between the rest; open
        // between the hall and the kitchen.
        std::sort(back.begin(), back.end(), [](const Seg& a2, const Seg& b2) { return a2.x0 < b2.x0; });
        for (std::size_t i = 0; i + 1 < back.size(); ++i) {
            const Seg& s = back[i];
            const Seg& t = back[i + 1];
            const bool hallSide = s.kind == RoomKind::Hall || t.kind == RoomKind::Hall;
            const RoomKind other = s.kind == RoomKind::Hall ? t.kind : s.kind;
            if (hallSide && other == RoomKind::Kitchen) continue;   // open
            RoomWall w; w.a = P(s.x1, vb); w.b = P(s.x1, vFront);
            if (hallSide && (other == RoomKind::Bath || other == RoomKind::Closet)) w.doorAt = 0.5;
            rp.walls.push_back(w);
        }
        // Window-side partitions: a bedroom's wall to the living room (or kitchen) carries its door near the
        // back; between two rooms that are not bedrooms the plan stays open.
        std::sort(front.begin(), front.end(), [](const Seg& a2, const Seg& b2) { return a2.x0 < b2.x0; });
        for (std::size_t i = 0; i + 1 < front.size(); ++i) {
            const Seg& s = front[i];
            const Seg& t = front[i + 1];
            const bool bedS = s.kind == RoomKind::Bed, bedT = t.kind == RoomKind::Bed;
            if (!bedS && !bedT) continue;
            RoomWall w; w.a = P(s.x1, vIn); w.b = P(s.x1, vb);
            if (bedS != bedT) w.doorAt = 0.78;
            rp.walls.push_back(w);
        }
        // The divider between the zones: solid wherever either side is private (a bedroom, the bath, the
        // closet), open between the hall, the kitchen and the living room.
        std::vector<Real> xs = {x0, x1};
        for (const Seg& s : front) { xs.push_back(s.x0); xs.push_back(s.x1); }
        for (const Seg& s : back) { xs.push_back(s.x0); xs.push_back(s.x1); }
        std::sort(xs.begin(), xs.end());
        auto kindAt = [](const std::vector<Seg>& v, Real x) {
            for (const Seg& s : v) if (x > s.x0 && x < s.x1) return s.kind;
            return RoomKind::Living;
        };
        Real runA = -1;
        auto flush = [&](Real upto) {
            if (runA >= 0 && upto - runA > 0.05) { RoomWall w; w.a = P(runA, vb); w.b = P(upto, vb); rp.walls.push_back(w); }
            runA = -1;
        };
        // A closet that is not beside the hall opens onto the room in front of it instead: a walk-in closet
        // off a bedroom, a coat closet off the living room.
        auto besideHall = [&](const Seg& c) {
            for (const Seg& t : back)
                if (t.kind == RoomKind::Hall && (std::fabs(t.x0 - c.x1) < 1e-6 || std::fabs(t.x1 - c.x0) < 1e-6)) return true;
            return false;
        };
        for (std::size_t i = 0; i + 1 < xs.size(); ++i) {
            if (xs[i + 1] - xs[i] < 1e-6) continue;
            const Real xm = (xs[i] + xs[i + 1]) * 0.5;
            const RoomKind fk = kindAt(front, xm), bk = kindAt(back, xm);
            const Seg* closet = nullptr;
            for (const Seg& c : back) if (c.kind == RoomKind::Closet && xm > c.x0 && xm < c.x1) closet = &c;
            if (closet && !besideHall(*closet)) {
                flush(xs[i]);
                RoomWall w; w.a = P(xs[i], vb); w.b = P(xs[i + 1], vb);
                if (xs[i + 1] - xs[i] > kRoomDoorW + 0.6) w.doorAt = 0.5;
                rp.walls.push_back(w);
                continue;
            }
            const bool solid = fk == RoomKind::Bed || bk == RoomKind::Bath || bk == RoomKind::Closet;
            if (solid) { if (runA < 0) runA = xs[i]; }
            else flush(xs[i]);
        }
        flush(x1);
}

// FLATS ROUND THE CORE (Glenn, 2026-10-13: "if you're showing me apartments then we need more walls for the separate
// apartments and then separate rooms"). A residential crown too slim for apartmentPlan's corridor and its 5 m bands --
// a 14 m pencil crown leaves 2-4 m between the glass and the smallest core there is -- used to be the residents'
// lounge. The way a slim tower does it: no corridor. The lifts and the stairs open onto a LOBBY (the strip in front of
// the core's doors), and two flats wrap the rest of the core, each through its own door off the lobby:
//
//     +-------+--------------------+-------+       each flat is a CHAIN of rooms down the strip it wraps: the
//     | bath  |  bed   ||   bed    | bath  |       column beside the core (corners included), then its half of
//     |-------+--------||----------+-------|       the strip behind -- hall, kitchen and living room open into
//     |  liv  |                    |  liv  |       each other, then a door to the bedroom, the bath en suite
//     |  ing  |       CORE         |  ing  |       at the end; partitions run ACROSS the strip, so a 2.5 m
//     |-------|                    |-------|       strip still makes a room a strip wide. Where the strip
//     |kitchen|                    |kitchen|       behind is too short for the bedroom and its bath, the
//     |hall  =|      L O B B Y     |=  hall|       bedroom ends the column and the strip behind is its dressing
//     +-------+--------------------+-------+       room and bath. Too short for a living room: a studio.
//
// Rectangular plates square to the core only; empty when it does not fit. The zoned floor's lobby-mode apartment
// layout (floor_zones.cpp zonedFloorPlan, #117 item 35): the floor too shallow for a corridor round the core.
RoomPlan wrapFlatsLayout(const Poly2& planIn, const CorePlan& core, Real inset, int storey) {
    constexpr Real kMinStrip = 2.4, kPassMin = 1.3, kHallLen = 2.0, kKitchenLen = 2.4, kLivingMin = 3.0, kBedLen = 3.0, kBedMax = 4.0,
                   kBathLen = 1.8, kClosetMin = 1.2, kStudioMin = 3.4;
    RoomPlan rp;
    rp.topology = PlateTopology::Apartments;
    rp.office = false;
    if (storey < 1 || !core.valid || planIn.size() != 4) return rp;
    Poly2 plan = planIn;
    ensureCCW(plan);
    struct Band { Vec2 a, d, nOut; Real W = 0, T = 0, t0 = 0, t1 = 0; };
    Band B[4];
    int front = -1, back = -1, side[2] = {-1, -1};   // side 0: the core's u = 0 end, side 1: its u = length end
    for (int e = 0; e < 4; ++e) {
        Band& b = B[e];
        b.a = plan[static_cast<std::size_t>(e)];
        const Vec2 dv = plan[static_cast<std::size_t>((e + 1) % 4)] - b.a;
        b.W = dv.length();
        if (b.W < 1e-6) return rp;
        b.d = dv * (1.0 / b.W);
        b.nOut = Vec2(b.d.y, -b.d.x);
        Real toCore = 1e9;
        b.t0 = 1e9; b.t1 = -1e9;
        for (const Vec2& c : core.rect()) {
            toCore = std::min(toCore, dot(b.a - c, b.nOut));
            const Real t = dot(c - b.a, b.d);
            b.t0 = std::min(b.t0, t); b.t1 = std::max(b.t1, t);
        }
        b.T = toCore - inset;
        if (dot(b.nOut, core.frame.v) < -0.99) front = e;
        else if (dot(b.nOut, core.frame.v) > 0.99) back = e;
        else if (dot(b.nOut, core.frame.u) < -0.99) side[0] = e;
        else if (dot(b.nOut, core.frame.u) > 0.99) side[1] = e;
        else return rp;   // not square to the core
    }
    if (front < 0 || back < 0 || side[0] < 0 || side[1] < 0) return rp;
    if (B[front].T < kRoomDoorW + 0.5) return rp;   // a lobby to stand in
    const bool armed = B[back].T >= kMinStrip;      // the strip behind the core is part of the flats

    struct Cell { Real s0, s1; RoomKind kind; bool arm; };
    std::vector<Cell> flats[2];
    for (int f = 0; f < 2; ++f) {
        const Band& col = B[side[f]];
        // A column too narrow for rooms is still a passage: the flat's own entrance hall, to its rooms behind.
        const bool passage = col.T < kMinStrip;
        if (passage && (col.T < kPassMin || !armed)) return RoomPlan{};
        // The column runs from the front corner to the back: s along it, x = xs + dir * s.
        const Vec2 frontMid = (B[front].a + B[front].a + B[front].d * B[front].W) * 0.5;
        const int dir = dot(frontMid - col.a, col.d) < col.W * 0.5 ? 1 : -1;
        const Real xs = dir > 0 ? inset : col.W - inset;
        const Real S1 = col.W - 2 * inset;
        const Real sc0 = (dir > 0 ? col.t0 : col.t1) - xs, sc1 = (dir > 0 ? col.t1 : col.t0) - xs;
        const Real c0 = std::fabs(sc0), c1 = std::fabs(sc1);   // the core's front and back faces along the column
        // Its half of the strip behind: from the column's inner face to the party wall at the middle.
        const Real S2 = armed ? (B[back].t1 - B[back].t0) * 0.5 : 0.0;
        // From the far end back: the bath, then the bedroom (in the strip behind if it fits beside the bath, else at
        // the column's end with a dressing room behind it), then the living room, the kitchen and the hall.
        std::vector<Cell>& cells = flats[f];
        std::vector<Cell> arm, colCells;
        if (passage) {
            // The hall up the column to the core's back face, a galley kitchen in the corner behind; the strip behind
            // holds the living room, the bedroom and the bath, or a studio and its bath.
            const bool galley = S1 - c1 >= 1.8;
            cells.push_back({0, galley ? c1 : S1, RoomKind::Hall, false});
            if (galley) cells.push_back({c1, S1, RoomKind::Kitchen, false});
            const Real kitIn = galley ? 0 : kKitchenLen;
            if (S2 - kitIn - kBathLen - kBedLen >= kLivingMin) {
                const Real bed = std::min(S2 - kitIn - kBathLen - kLivingMin, kBedMax);
                if (!galley) cells.push_back({0, kitIn, RoomKind::Kitchen, true});
                cells.push_back({kitIn, S2 - kBathLen - bed, RoomKind::Living, true});
                cells.push_back({S2 - kBathLen - bed, S2 - kBathLen, RoomKind::Bed, true});
            } else if (S2 - kitIn - kBathLen >= kStudioMin) {
                if (!galley) cells.push_back({0, kitIn, RoomKind::Kitchen, true});
                cells.push_back({kitIn, S2 - kBathLen, RoomKind::Flat, true});
            } else {
                return RoomPlan{};
            }
            cells.push_back({S2 - kBathLen, S2, RoomKind::Bath, true});
        }
        Real bedFrom = S1;   // where the bedroom starts in the column (s), or S1 when it is in the arm
        if (passage) {
        } else if (S2 >= kBathLen + kBedLen) {
            Real bed = std::min(S2 - kBathLen, kBedMax);
            if (S2 - kBathLen - bed < 2.0) bed = S2 - kBathLen;   // a sliver of living room joins the bedroom
            if (S2 - kBathLen - bed > 0.05) arm.push_back({0, S2 - kBathLen - bed, RoomKind::Living, true});
            arm.push_back({S2 - kBathLen - bed, S2 - kBathLen, RoomKind::Bed, true});
            arm.push_back({S2 - kBathLen, S2, RoomKind::Bath, true});
            bedFrom = S1;
        } else {
            if (S2 >= kBathLen + kClosetMin) {
                arm.push_back({0, S2 - kBathLen, RoomKind::Closet, true});
                arm.push_back({S2 - kBathLen, S2, RoomKind::Bath, true});
            } else if (S2 >= kBathLen) {
                arm.push_back({0, S2, RoomKind::Bath, true});
            }
            // the bedroom ends the column; with no room behind it, the bath does
            const Real bathIn = arm.empty() ? kBathLen : 0;
            bedFrom = std::min(S1 - bathIn - kBedLen, c1);   // never a cut beside the strip behind (rects only)
            if (bathIn > 0) colCells.push_back({S1 - bathIn, S1, RoomKind::Bath, false});
            colCells.insert(colCells.begin(), {bedFrom, S1 - bathIn, RoomKind::Bed, false});
        }
        // The hall at the door (inside the front corner), the kitchen, the living room to the bedroom.
        const Real hall = passage ? std::min(c1, S1) : std::min(kHallLen, c0);
        // a long column: the kitchen takes a dining table rather than the living room running on past 7 m
        const Real kit = hall + std::clamp(bedFrom - hall - 7.0, kKitchenLen, Real(4.0));
        std::vector<Cell> head;
        if (passage) {
        } else if (bedFrom - kit >= kLivingMin) {
            head = {{0, hall, RoomKind::Hall, false}, {hall, kit, RoomKind::Kitchen, false},
                    {kit, bedFrom, RoomKind::Living, false}};
        } else {
            // A STUDIO: the bed comes into the living room; the hall, the kitchen, the room, the bath at the end.
            const Real roomTo = arm.empty() ? S1 - kBathLen : S1;
            if (roomTo - kit < kStudioMin) return RoomPlan{};
            colCells.clear();
            arm.clear();
            if (S2 >= kBathLen) arm.push_back({0, S2, RoomKind::Bath, true});
            else colCells.push_back({S1 - kBathLen, S1, RoomKind::Bath, false});
            head = {{0, hall, RoomKind::Hall, false}, {hall, kit, RoomKind::Kitchen, false},
                    {kit, arm.empty() ? S1 - kBathLen : S1, RoomKind::Flat, false}};
        }
        if (!passage) {
            cells = head;
            cells.insert(cells.end(), colCells.begin(), colCells.end());
            cells.insert(cells.end(), arm.begin(), arm.end());
        }

        // Emit. The column's cells across its strip; the arm's across the strip behind, from this column's side.
        auto Pc = [&](Real s, Real v) { return col.a + col.d * (xs + dir * s) - col.nOut * v; };
        const Band& bk = B[back];
        const Vec2 colOnBack = col.a + col.d * (xs + dir * S1);   // the column's back corner, on the back band's line
        const int adir = dot(colOnBack - bk.a, bk.d) < bk.W * 0.5 ? 1 : -1;
        const Real ax = adir > 0 ? bk.t0 : bk.t1;
        auto Pa = [&](Real s, Real v) { return bk.a + bk.d * (ax + adir * s) - bk.nOut * v; };
        for (const Cell& c : cells) {
            Room r;
            r.edge = static_cast<std::size_t>(c.arm ? back : side[f]);
            r.kind = c.kind;
            // the window wall first (furniture's side 0), counter-clockwise like every other planner's rooms
            const auto& P = c.arm ? std::function<Vec2(Real, Real)>(Pa) : std::function<Vec2(Real, Real)>(Pc);
            const Real T = c.arm ? bk.T : col.T;
            const int sd = c.arm ? adir : dir;
            const Real lo = sd > 0 ? c.s0 : c.s1, hi = sd > 0 ? c.s1 : c.s0;
            r.rect = {P(lo, inset), P(hi, inset), P(hi, inset + T), P(lo, inset + T)};
            rp.rooms.push_back(r);
        }
        auto open = [](RoomKind a, RoomKind b) {
            auto pub = [](RoomKind k) { return k == RoomKind::Hall || k == RoomKind::Kitchen || k == RoomKind::Living; };
            return pub(a) && pub(b);
        };
        // The partitions between consecutive cells: across the strip within it; where the column meets the strip
        // behind, the column's inner face from the core's back face to the back wall.
        for (std::size_t i = 0; i + 1 < cells.size(); ++i) {
            const Cell& a = cells[i];
            const Cell& b = cells[i + 1];
            if (open(a.kind, b.kind)) continue;
            RoomWall w;
            if (a.arm == b.arm) {
                const auto& P = a.arm ? std::function<Vec2(Real, Real)>(Pa) : std::function<Vec2(Real, Real)>(Pc);
                const Real T = a.arm ? bk.T : col.T;
                w.a = P(a.s1, inset);
                w.b = P(a.s1, inset + T);
                w.doorAt = 0.7;   // near the core side: the glass side stays for the bed and the bath's fittings
            } else {
                w.a = Pc(c1, inset + col.T);
                w.b = Pc(S1, inset + col.T);
                w.doorAt = 0.5;
            }
            rp.walls.push_back(w);
        }
        // The front door: the column's inner face across the front corner, onto the lobby, in the hall.
        {
            RoomWall w;
            w.a = Pc(0, inset + col.T);
            w.b = Pc(c0, inset + col.T);
            w.doorAt = std::clamp(std::min(hall, c0) * 0.5 / c0, Real(0.0), Real(1.0));
            rp.walls.push_back(w);
        }
        // No strip behind: the column's inner face there closes it off (a riser space behind the core).
        if (!armed) {
            RoomWall w;
            w.a = Pc(c1, inset + col.T);
            w.b = Pc(S1, inset + col.T);
            rp.walls.push_back(w);
        }
    }
    // The party wall between the two flats, across the strip behind at its middle.
    if (armed) {
        const Band& bk = B[back];
        const Real xm = (bk.t0 + bk.t1) * 0.5;
        RoomWall w;
        w.a = bk.a + bk.d * xm - bk.nOut * inset;
        w.b = bk.a + bk.d * xm - bk.nOut * (inset + bk.T);
        rp.walls.push_back(w);
    }
    return rp;
}

// THE WALK-UP (Glenn, 2026-10-01: "I didn't see the apartments ... some of them look like dorms"): a residential
// building without a lift core. A corridor runs the long axis from the stair, apartments either side of it
// (double-loaded) -- or, on a plate too narrow for two, one row off a corridor along the back facade. The stair's
// hall is cut out of the band it stands in, facade to corridor, so the stair opens onto the corridor.
static RoomPlan walkupPlan(const Poly2& planIn, const BuildingParams& params, const Poly2& well, Real inset,
                           int storey, const Vec2& stairFoot) {
    RoomPlan rp;
    rp.topology = PlateTopology::Apartments;
    rp.office = false;
    if (storey < 1 || planIn.size() != 4 || well.size() < 3) return rp;
    const OBB2 ob = orientedBoundingBox(planIn);
    const int la = ob.longAxis();
    const Vec2 ua = ob.axis[la];
    const Vec2 va(ua.y, -ua.x);   // the outward normal of a band running along +ua (ringPlan's convention)
    const Real hl = ob.half[la], hw = ob.half[1 - la];
    const Vec2 o = ob.center;
    const Real kCorr = 1.6;
    const bool twoSides = 2 * hw >= 2 * (inset + 5.4) + kCorr;
    // The stair hall in OBB coordinates (s along, t across), widened for the landing.
    Real s0 = 1e9, s1 = -1e9, t0 = 1e9, t1 = -1e9;
    for (const Vec2& q : well) {
        const Vec2 d = q - o;
        s0 = std::min(s0, dot(d, ua)); s1 = std::max(s1, dot(d, ua));
        t0 = std::min(t0, dot(d, va)); t1 = std::max(t1, dot(d, va));
    }
    // THE WHOLE FLIGHT, not just the well (Glenn, 2026-10-03: "The stairwells are inaccessible the way they are"):
    // the well is the hole over the flight's UPPER part, so a hall cut round it alone stood the flight's foot --
    // and the floor you step onto it from -- behind the hall's closing wall. The hall reaches the foot and 1.4 m
    // beyond it.
    if (stairFoot.x < 1e29) {
        const Vec2 d = stairFoot - o;
        const Real fs = dot(d, ua), ft = dot(d, va);
        const Real wellMid = (s0 + s1) * 0.5;
        if (fs < wellMid) s0 = std::min(s0, fs - 1.4); else s1 = std::max(s1, fs + 1.4);
        t0 = std::min(t0, ft); t1 = std::max(t1, ft);
    }
    s0 -= 1.0; s1 += 1.0;
    // The bands: {P, depth from the window wall to the corridor front}. Band 0 faces +va and runs along +ua (its
    // x = s + hl); band 1 faces -va and runs along -ua (x = hl - s).
    struct Band { std::function<Vec2(Real, Real)> P; Real vFront; Real t0, t1; bool alongPlus; };
    std::vector<Band> bands;
    auto plusBand = [=](Real front, Real tLo) {
        return Band{[=](Real x, Real v) { return o - ua * hl + va * hw + ua * x - va * v; }, front, tLo, hw, true};
    };
    auto minusBand = [=](Real front, Real tHi) {
        return Band{[=](Real x, Real v) { return o + ua * hl - va * hw - ua * x + va * v; }, front, -hw, tHi, false};
    };
    if (twoSides) {
        bands.push_back(plusBand(hw - kCorr * 0.5, kCorr * 0.5));
        bands.push_back(minusBand(hw - kCorr * 0.5, -kCorr * 0.5));
    } else if ((t0 + t1) * 0.5 >= 0) {
        // One row: the apartments on the stair's side, the corridor along the facade OPPOSITE (the stair would
        // block a corridor beside it); the stair hall is cut through the row to reach it.
        bands.push_back(plusBand(2 * hw - inset - kCorr, -hw + inset + kCorr));
    } else {
        bands.push_back(minusBand(2 * hw - inset - kCorr, hw - inset - kCorr));
    }
    for (std::size_t bi = 0; bi < bands.size(); ++bi) {
        const Band& b = bands[bi];
        const Real vIn = inset, D = b.vFront - vIn;
        if (D < 5.0) continue;
        // The band's run along, less the stair hall where the stair stands in it. Band 0 runs along +ua (x = s +
        // hl), band 1 along -ua (x = hl - s).
        std::vector<std::pair<Real, Real>> runs = {{inset, 2 * hl - inset}};
        if (t1 > b.t0 && t0 < b.t1) {
            const Real a = b.alongPlus ? s0 + hl : hl - s1, c = b.alongPlus ? s1 + hl : hl - s0;
            runs = {{inset, a}, {c, 2 * hl - inset}};
        }
        for (const auto& [r0, r1] : runs) {
            if (r1 - r0 < 6.8) continue;
            int units = std::max(1, static_cast<int>((r1 - r0) / (7.0 + 2.0 * (((params.seed >> (bi * 3)) & 7u) / 7.0))));
            while (units > 1 && (r1 - r0) / units < 6.8) --units;
            for (int k = 0; k < units; ++k) {
                const Real x0 = r0 + (r1 - r0) * k / units, x1 = r0 + (r1 - r0) * (k + 1) / units;
                layoutApartment(rp, bi, b.P, x0, x1, vIn, b.vFront, D, (x0 + x1) * 0.5, k + 1 < units);
            }
            // The walls closing the run where it meets the stair hall (the plan's own ends are its facade).
            for (Real x : {r0, r1}) {
                if (x <= inset + 1e-6 || x >= 2 * hl - inset - 1e-6) continue;
                RoomWall w; w.a = b.P(x, vIn); w.b = b.P(x, b.vFront);
                rp.walls.push_back(w);
            }
        }
    }
    return rp;
}

// ------------------------------------------------------------- the university
// A CAMPUS BUILDING's floor (Glenn, 2026-10-03: "a university campus over blocks (library, classrooms, offices,
// dorms, quads, sports fields)"). A stair building like the walk-up: a 2.4 m corridor down the long axis from the
// stair, rooms either side of it, each with its door on the corridor -- but every room a single room of the
// building's use:
//   TEACHING HALL (campus 1)   the ground floor's larger side LECTURE HALLS (12-16 m), the rest CLASSROOMS (~8.5 m);
//                              upstairs classrooms with a LAB every third room; restrooms by the stair.
//   LIBRARY (2)                each run of the ground floor one READING room; the floors above, the STACKS.
//   RESIDENCE HALL (3)         two-bed DORM rooms (3.6 m), a shared bath every eighth, a lounge by the stair.
static RoomPlan campusPlan(const Poly2& planIn, const BuildingParams& params, const Poly2& well, Real inset,
                           int storey, const Vec2& stairFoot, std::size_t entranceEdge) {
    // the campus's halls, or a civic building's -- a fire station's upper floor is its crew's quarters, a residence hall's
    const int use = params.campus ? params.campus : params.civicRooms == 9 ? 3 : params.civicRooms;
    RoomPlan rp;
    rp.topology = PlateTopology::Apartments;
    rp.office = use != 3;
    if (planIn.size() < 4 || well.size() < 3) return rp;
    const OBB2 ob = orientedBoundingBox(planIn);
    const int la = ob.longAxis();
    const Vec2 ua = ob.axis[la];
    const Vec2 va(ua.y, -ua.x);
    Real hl = ob.half[la];
    const Real hw = ob.half[1 - la];
    Vec2 o = ob.center;
    // A TRIMMED HALL (its outer end cut to fit a rounded block corner -- 8 corners, not 4): the rooms are laid in
    // the box, so each end of it is drawn in until both its corners are inside the plan. (Such a hall fell through
    // to the apartment plan: beds and sofas in a teaching hall.)
    if (planIn.size() != 4) {
        Poly2 pl = planIn;
        ensureCCW(pl);
        Real e0 = -hl, e1 = hl;   // the box's ends along ua
        auto inside = [&](Real s) {
            for (Real t : {hw - 0.05, -hw + 0.05})
                if (!pointInPolygon(pl, o + ua * s + va * t)) return false;
            return true;
        };
        for (int k = 0; k < 80 && !inside(e0 + 0.05); ++k) e0 += 0.25;
        for (int k = 0; k < 80 && !inside(e1 - 0.05); ++k) e1 -= 0.25;
        if (e1 - e0 < 8.0) return rp;
        o = o + ua * ((e0 + e1) * 0.5);
        hl = (e1 - e0) * 0.5;
    }
    const Real kCorr = 2.4;
    const bool twoSides = 2 * hw >= 2 * (inset + 5.0) + kCorr;
    Real s0 = 1e9, s1 = -1e9, t0 = 1e9, t1 = -1e9;
    for (const Vec2& q : well) {
        const Vec2 d = q - o;
        s0 = std::min(s0, dot(d, ua)); s1 = std::max(s1, dot(d, ua));
        t0 = std::min(t0, dot(d, va)); t1 = std::max(t1, dot(d, va));
    }
    // THE WHOLE FLIGHT, not just the well (Glenn, 2026-10-03: "The stairwells are inaccessible the way they are"):
    // the well is the hole over the flight's UPPER part, so a hall cut round it alone stood the flight's foot --
    // and the floor you step onto it from -- behind the hall's closing wall. The hall reaches the foot and 1.4 m
    // beyond it.
    if (stairFoot.x < 1e29) {
        const Vec2 d = stairFoot - o;
        const Real fs = dot(d, ua), ft = dot(d, va);
        const Real wellMid = (s0 + s1) * 0.5;
        if (fs < wellMid) s0 = std::min(s0, fs - 1.4); else s1 = std::max(s1, fs + 1.4);
        t0 = std::min(t0, ft); t1 = std::max(t1, ft);
    }
    s0 -= 1.0; s1 += 1.0;
    struct Band { std::function<Vec2(Real, Real)> P; Real vFront; Real t0, t1; bool alongPlus; };
    std::vector<Band> bands;
    auto plusBand = [=](Real front, Real tLo) {
        return Band{[=](Real x, Real v) { return o - ua * hl + va * hw + ua * x - va * v; }, front, tLo, hw, true};
    };
    auto minusBand = [=](Real front, Real tHi) {
        return Band{[=](Real x, Real v) { return o + ua * hl - va * hw - ua * x + va * v; }, front, -hw, tHi, false};
    };
    if (twoSides) {
        bands.push_back(plusBand(hw - kCorr * 0.5, kCorr * 0.5));
        bands.push_back(minusBand(hw - kCorr * 0.5, -kCorr * 0.5));
    } else if ((t0 + t1) * 0.5 >= 0) {
        bands.push_back(plusBand(2 * hw - inset - kCorr, -hw + inset + kCorr));
    } else {
        bands.push_back(minusBand(2 * hw - inset - kCorr, hw - inset - kCorr));
    }
    uint32_t h = static_cast<uint32_t>(params.seed) * 2654435761u ^ static_cast<uint32_t>(storey * 131 + 7);
    auto rnd = [&]() { h ^= h << 13; h ^= h >> 17; h ^= h << 5; return (h & 0xffffu) / 65535.0; };
    int roomNo = 0;
    // THE LECTURE HALL: the ground floor's first run long enough for one (10.5 m: the stage, the aisle, a block of
    // rows), on whichever side of the corridor has it -- the stair's side is often cut too short. The city's halls
    // are single-loaded, their rooms ~7 m deep: deep enough for the stage and three tiers
    bool lectureDone = false;
    bool loungeDone = false;   // (a fire station's day room: the first run with the rooms for it)
    for (std::size_t bi = 0; bi < bands.size(); ++bi) {
        const Band& b = bands[bi];
        const Real vIn = inset, D = b.vFront - vIn;
        if (D < 4.5) continue;
        std::vector<std::pair<Real, Real>> runs = {{inset, 2 * hl - inset}};
        const bool stairHere = t1 > b.t0 && t0 < b.t1;
        if (stairHere) {
            const Real a = b.alongPlus ? s0 + hl : hl - s1, c = b.alongPlus ? s1 + hl : hl - s0;
            runs = {{inset, a}, {c, 2 * hl - inset}};
        }
        // A PLAN THAT IS NOT ITS BOX (a notch in a long side -- an entrance recess -- or a trimmed end): the run is
        // cut wherever the band's depth leaves the plan, as it is round the stair. (Laid in the box regardless, a
        // hall's lecture theatre and classrooms ran out through the notch's walls, half outside.)
        if (planIn.size() != 4) {
            Poly2 pl = planIn;
            ensureCCW(pl);
            auto colOk = [&](Real x) {
                for (Real v : {vIn + 0.05, (vIn + b.vFront) * 0.5, b.vFront - 0.05})
                    if (!pointInPolygon(pl, b.P(x, v))) return false;
                return true;
            };
            std::vector<std::pair<Real, Real>> cut;
            constexpr Real kStep = 0.25;
            for (const auto& r : runs) {
                Real x = r.first;
                while (x < r.second) {
                    while (x < r.second && !colOk(x)) x += kStep;
                    const Real start = x;
                    while (x < r.second && colOk(std::min(x + kStep, r.second))) x += kStep;
                    const Real end = std::min(x, r.second);
                    if (end - start >= 3.4) cut.push_back({start, end});
                    x += kStep;
                }
            }
            if (std::getenv("RT_CAMPUS_DEBUG")) {
                std::fprintf(stderr, "[campus cut] storey %d band %zu hl %.1f:", storey, bi, hl);
                for (const auto& r : runs) std::fprintf(stderr, " [%.1f %.1f]", r.first, r.second);
                std::fprintf(stderr, " ->");
                for (const auto& r : cut) std::fprintf(stderr, " [%.1f %.1f]", r.first, r.second);
                std::fprintf(stderr, "\n");
            }
            runs = cut;
        }
        // THE QUAD DOOR (campus halls have a back door onto the quad; campusQuadDoor places it, for the facade too):
        // a 4 m passage through this band from the corridor to it, cut like the stair's -- the door opens on the way
        // through, not into a classroom (or a bookcase). Usually it already opens into the stair hall: the stair hugs
        // the rear wall and the door stands just short of its foot.
        Vec2 mid, outward;
        if (storey == 0 && campusQuadDoor(planIn, params, entranceEdge, mid, outward)) {
            {
                const Real side = dot(mid - o, va);
                if (std::getenv("RT_CAMPUS_DEBUG"))
                    std::fprintf(stderr, "[quad door] campus %d band %zu alongPlus %d: rear side %+.1f -> %s\n", params.campus, bi,
                                 b.alongPlus ? 1 : 0, side, (side > 0) == b.alongPlus ? "this band: cut" : "the corridor's side");
                if ((side > 0) == b.alongPlus) {
                    const Real x = b.alongPlus ? dot(mid - o, ua) + hl : hl - dot(mid - o, ua);
                    std::vector<std::pair<Real, Real>> cut;
                    for (const auto& r : runs) {
                        if (x + 2.0 <= r.first || x - 2.0 >= r.second) { cut.push_back(r); continue; }
                        if (x - 2.0 - r.first >= 3.4) cut.push_back({r.first, x - 2.0});
                        if (r.second - (x + 2.0) >= 3.4) cut.push_back({x + 2.0, r.second});
                    }
                    runs = cut;
                }
            }
        }
        for (std::size_t ri = 0; ri < runs.size(); ++ri) {
            const Real r0 = runs[ri].first, r1 = runs[ri].second;
            if (r1 - r0 < 3.4) continue;
            // the rooms of this run: {width, kind}, laid from r0; the last takes the remainder
            struct Seg { Real w; RoomKind kind; };
            std::vector<Seg> want;
            // RESTROOMS beside the stair: a teaching hall's 3.6 m room carved off the run's end by the stair hall --
            // never a whole room turned into one (it took a 15 m lecture hall: a toilet and a vanity in a hall)
            const bool byStair = stairHere && runs.size() == 2;
            const bool restroom = byStair && ri == 0 && (use == 1 || use == 4 || use == 5 || use >= 10) && r1 - r0 >= 3.6 + 6.5;
            const Real len = r1 - r0 - (restroom ? 3.6 : 0.0);
            auto fill = [&](Real target, Real minW, RoomKind kind) {
                int n = std::max(1, static_cast<int>(len / target));
                while (n > 1 && len / n < minW) --n;
                for (int k = 0; k < n; ++k) want.push_back({len / n, kind});
            };
            switch (use) {
                case 4:   // a museum: galleries, ~10 m a room, every floor
                    fill(10.0, 6.0, RoomKind::Gallery);
                    break;
                case 10:  // a police station's ground floor: cells, an interview room every third
                    fill(3.0, 2.4, RoomKind::Cell);
                    for (std::size_t k = 0; k < want.size(); ++k)
                        if ((roomNo + static_cast<int>(k)) % 3 == 2) want[k].kind = RoomKind::Meeting;
                    break;
                case 11:  // a courthouse's: courtrooms
                    fill(12.0, 8.0, RoomKind::Courtroom);
                    break;
                case 12:  // a capitol's: the council chamber (its first run), committee rooms
                    if (roomNo == 0) want.push_back({len, RoomKind::Chamber});
                    else fill(6.0, 4.0, RoomKind::Meeting);
                    break;
                case 1:   // teaching hall
                case 5:   // a school: classrooms and labs, no lecture hall
                    if (use == 1 && storey == 0 && !lectureDone && D >= 6.5 && len >= 10.5) {   // 6.5 deep: the stage and three tiers
                        fill(13.0 + 3.0 * rnd(), 10.0, RoomKind::Lecture);
                        lectureDone = true;
                    } else {
                        fill(8.5, 6.5, RoomKind::Classroom);
                    }
                    if (storey > 0)
                        for (std::size_t k = 0; k < want.size(); ++k)
                            if ((roomNo + static_cast<int>(k)) % 3 == 2) want[k].kind = RoomKind::Lab;
                    break;
                case 2:   // library: one room a run
                    want.push_back({len, storey == 0 ? RoomKind::Reading : RoomKind::Stacks});
                    break;
                default:  // residence hall
                    fill(3.6, 3.3, RoomKind::Dorm);
                    for (std::size_t k = 0; k < want.size(); ++k)
                        if ((roomNo + static_cast<int>(k)) % 8 == 7) want[k].kind = RoomKind::Bath;
                    // the lounge: two rooms merged -- on a fire station's quarters (its first floor up), the crew's day
                    // room, their kitchen beside it
                    if (storey == (params.civicRooms == 9 ? 1 : 0) && want.size() >= 3 &&
                        (params.civicRooms == 9 ? !loungeDone : bi == 0 && ri == 0)) {
                        loungeDone = true;
                        want[1].w += want[0].w;
                        want[1].kind = RoomKind::Living;
                        want.erase(want.begin());
                        if (params.civicRooms == 9 && want.size() >= 2) want[1].kind = RoomKind::Kitchen;
                    }
                    break;
            }
            // THE LOBBY of a police station, a courthouse, a capitol: 7 m of the run the front door opens into, the
            // run's rooms either side of it (the rooms that fell across it give way)
            if (storey == 0 && use >= 10 && entranceEdge < planIn.size()) {
                const Vec2 door = (planIn[entranceEdge] + planIn[(entranceEdge + 1) % planIn.size()]) * 0.5;
                const Vec2 p0 = b.P(0, 0), ax = b.P(1, 0) - p0, av = b.P(0, 1) - p0;
                const Real xd = dot(door - p0, ax), vd = dot(door - p0, av);
                if (std::fabs(vd) < 1.0 && xd > r0 + 1.0 && xd < r0 + len - 1.0 && len >= 7.0 + 2.4) {
                    const Real la = std::clamp(xd - 3.5, r0, r0 + len - 7.0) - r0, lb = la + 7.0;   // run coordinates
                    std::vector<Seg> carved;
                    Real at = 0;
                    for (const Seg& sg : want) {
                        const Real s0 = at, s1 = at + sg.w;
                        at = s1;
                        if (s1 <= la + 1e-6) carved.push_back(sg);
                        else if (s0 >= lb - 1e-6) carved.push_back(sg);
                    }
                    // the room before the lobby runs to it; the lobby; the room after starts at it
                    std::vector<Seg> out;
                    Real end = 0;
                    std::size_t i = 0;
                    for (; i < carved.size() && end + carved[i].w <= la + 1e-6; ++i) { out.push_back(carved[i]); end += carved[i].w; }
                    // a stretch the lobby cut from a room keeps that room's kind (a courtroom either side of it),
                    // unless too short for one
                    auto kindFor = [&](Real w, RoomKind k) {
                        return (k == RoomKind::Courtroom || k == RoomKind::Chamber) && w < 8.0 ? RoomKind::Meeting : k;
                    };
                    if (la - end > 1e-6) {
                        if (!out.empty() && la - end < 2.4) out.back().w += la - end;
                        else out.push_back({la - end, kindFor(la - end, out.empty() ? want.front().kind : out.back().kind)});
                    }
                    out.push_back({7.0, RoomKind::Waiting});
                    for (; i < carved.size(); ++i) out.push_back(carved[i]);
                    Real sum = 0;
                    for (const Seg& sg : out) sum += sg.w;
                    if (sum < len - 1e-6 && out.back().kind != RoomKind::Waiting) out.back().w += len - sum;
                    else if (sum < len - 1e-6) out.push_back({len - sum, kindFor(len - sum, want.back().kind)});
                    want = out;
                }
            }
            if (restroom) want.push_back({3.6, RoomKind::Bath});
            if (std::getenv("RT_CAMPUS_DEBUG") && storey == 0)
                std::fprintf(stderr, "[campus plan] campus %d band %zu run %zu: D %.1f len %.1f stair %d -> %zu rooms, first kind %d\n",
                             use, bi, ri, D, len, stairHere ? 1 : 0, want.size(), want.empty() ? -1 : static_cast<int>(want[0].kind));
            roomNo += static_cast<int>(want.size());
            Real x = r0;
            for (std::size_t k = 0; k < want.size(); ++k) {
                const Real x0 = x, x1 = k + 1 == want.size() ? r1 : x + want[k].w;
                x = x1;
                Room rm;
                rm.edge = bi;
                rm.kind = want[k].kind;
                rm.rect = {b.P(x0, vIn), b.P(x1, vIn), b.P(x1, b.vFront), b.P(x0, b.vFront)};
                // a courtroom or a chamber runs ALONG its band: side 0 its public end (x0, by the door), side 2 the
                // bench's -- so its benches face down the room's length (the furniture's sides follow the rect)
                const bool along = want[k].kind == RoomKind::Courtroom || want[k].kind == RoomKind::Chamber;
                if (along) rm.rect = {b.P(x0, b.vFront), b.P(x0, vIn), b.P(x1, vIn), b.P(x1, b.vFront)};
                rp.rooms.push_back(rm);
                // the front, onto the corridor, with the room's door (a lecture hall's near its back corner)
                RoomWall fw;
                fw.a = b.P(x0, b.vFront);
                fw.b = b.P(x1, b.vFront);
                // a teaching room's door near the back corner (the middle of that wall is the front: the board)
                const bool teaching = want[k].kind == RoomKind::Lecture || want[k].kind == RoomKind::Classroom ||
                                      want[k].kind == RoomKind::Lab;
                fw.doorAt = teaching ? (b.alongPlus ? 0.88 : 0.12) : along ? 0.12 : 0.5;
                rp.walls.push_back(fw);
                // the partition to the next room
                if (k + 1 < want.size()) {
                    RoomWall pw;
                    pw.a = b.P(x1, vIn);
                    pw.b = b.P(x1, b.vFront);
                    rp.walls.push_back(pw);
                }
            }
            // the walls closing the run where it meets the stair hall (the plan's own ends are its facade)
            for (Real xe : {r0, r1}) {
                if (xe <= inset + 1e-6 || xe >= 2 * hl - inset - 1e-6) continue;
                RoomWall w; w.a = b.P(xe, vIn); w.b = b.P(xe, b.vFront);
                rp.walls.push_back(w);
            }
        }
    }
    // A POLICE STATION'S, A COURTHOUSE'S, A CAPITOL'S ground floor: the room the front door opens into is its LOBBY
    // (the front desk, waiting seats) -- never the chamber.
    bool hasLobby = false;
    for (const Room& r : rp.rooms) hasLobby = hasLobby || r.kind == RoomKind::Waiting;
    if (storey == 0 && use >= 10 && !hasLobby && entranceEdge < planIn.size()) {
        const Vec2 door = (planIn[entranceEdge] + planIn[(entranceEdge + 1) % planIn.size()]) * 0.5;
        Room* best = nullptr;
        Real bestD = 1e9;
        for (Room& r : rp.rooms) {
            if (r.kind == RoomKind::Chamber || r.kind == RoomKind::Bath) continue;
            const Real dd = (centroid(r.rect) - door).length();
            if (dd < bestD) { bestD = dd; best = &r; }
        }
        if (best) best->kind = RoomKind::Waiting;
    }
    return rp;
}

// ------------------------------------------------------------- the civic halls
// ONE-ROOM HALLS (Glenn, 2026-10-13: "I like your idea. Do that" -- the buildings standing empty, or fitted as a
// cafe-bar): the ground floor of a church is its NAVE, pews facing the altar at the far end from the door; a market
// hall's is its floor of stalls; a cinema's is its FOYER along the street (the ticket desk, the concession, seats)
// and two to four AUDITORIUMS behind it, each with its screen on the far wall and its rows raked up toward its door.
// Every room's side 0 is its street side (the entrance), so the far wall is side 2. Rectangular plans only.
static RoomPlan hallPlan(const Poly2& planIn, const BuildingParams& params, Real inset, std::size_t entranceEdge,
                         const Poly2& well, const Vec2& stairFoot = Vec2(1e30, 1e30)) {
    RoomPlan rp;
    rp.topology = PlateTopology::Ring;
    rp.office = true;   // (a restroom, were there one; lit as a public room)
    if (planIn.size() != 4 || entranceEdge >= planIn.size()) return rp;
    Poly2 plan = planIn;
    ensureCCW(plan);
    const Vec2 em = (planIn[entranceEdge] + planIn[(entranceEdge + 1) % 4]) * 0.5;
    std::size_t e = 0;
    Real best = 1e9;
    for (std::size_t i = 0; i < 4; ++i) {
        const Real dd = ((plan[i] + plan[(i + 1) % 4]) * 0.5 - em).length();
        if (dd < best) { best = dd; e = i; }
    }
    const Vec2 a = plan[e], dv = plan[(e + 1) % 4] - a;
    const Real W = dv.length();
    if (W < 6.0) return rp;
    const Vec2 d = dv * (1.0 / W), nOut(d.y, -d.x);
    Real Dp = 0;
    for (const Vec2& v : plan) Dp = std::max(Dp, dot(a - v, nOut));
    auto P = [&](Real x, Real v) { return a + d * x - nOut * v; };
    auto room = [&](Real xa, Real xb, Real va, Real vb, RoomKind kind) {
        Room r; r.edge = e; r.kind = kind;
        r.rect = {P(xa, va), P(xb, va), P(xb, vb), P(xa, vb)};
        rp.rooms.push_back(r);
    };
    const Real x0 = inset, x1 = W - inset, v0 = inset, v1 = Dp - inset;
    if (v1 - v0 < 5.0) return rp;
    const int use = params.civicRooms;
    // THE STAIR (a hall with a floor above): its well and its flight's foot are floor nothing may stand on
    auto keepStairClear = [&]() {
        if (well.size() < 3) return;
        Real s0 = 1e9, s1 = -1e9, t0 = 1e9, t1 = -1e9;
        auto take = [&](const Vec2& q) {
            const Real s = dot(q - a, d), t = dot(a - q, nOut);
            s0 = std::min(s0, s); s1 = std::max(s1, s); t0 = std::min(t0, t); t1 = std::max(t1, t);
        };
        for (const Vec2& q : well) take(q);
        if (stairFoot.x < 1e29) take(stairFoot);
        Room st; st.edge = e; st.kind = RoomKind::Apparatus;
        st.rect = {P(s0 - 0.8, t0 - 0.8), P(s1 + 0.8, t0 - 0.8), P(s1 + 0.8, t1 + 0.8), P(s0 - 0.8, t1 + 0.8)};
        rp.rooms.push_back(st);
    };
    if (use == 15 || use == 16) {
        // A FACTORY'S SHOP FLOOR, A WAREHOUSE'S RACKS: the whole floor, a loading strip kept clear inside each open bay
        // (a truck's length in: 5 m), the stair's flight clear
        room(x0, x1, v0, v1, use == 15 ? RoomKind::ShopFloor : RoomKind::Warehouse);
        Vec2 out(0, 0);
        for (const auto& [da, db] : vehicleBayDoors(planIn, params, out)) {
            const Real xa = dot(da - a, d), xb = dot(db - a, d);
            Room ld; ld.edge = e; ld.kind = RoomKind::Apparatus;
            ld.rect = {P(std::min(xa, xb) - 0.3, v0), P(std::max(xa, xb) + 0.3, v0), P(std::max(xa, xb) + 0.3, v0 + 5.0),
                       P(std::min(xa, xb) - 0.3, v0 + 5.0)};
            rp.rooms.push_back(ld);
        }
        keepStairClear();
        return rp;
    }
    if (use == 9) {
        // THE ENGINE BAY: the whole floor, and behind each open bay door the floor its engine stands on -- the city
        // sim parks one there (the fleet's fire engine, 9.2 m: its nose a metre in from the door), so nothing else
        // may; a bay too shallow for one keeps its floor
        room(x0, x1, v0, v1, RoomKind::EngineBay);
        Vec2 out(0, 0);
        constexpr Real kEngineL = 9.3, kEngineW = 2.6;
        for (const auto& [da, db] : vehicleBayDoors(planIn, params, out)) {
            const Vec2 mid = (da + db) * 0.5;
            Real deep = 0;
            for (const Vec2& q : planIn) deep = std::max(deep, dot(mid - q, out));
            if (deep < kEngineL + 1.2) continue;   // (a little short of the sim's own test: never a parked engine unreserved)
            const Real xm = dot(mid - a, d);
            Room ap; ap.edge = e; ap.kind = RoomKind::Apparatus;
            ap.rect = {P(xm - kEngineW * 0.5 - 0.3, v0), P(xm + kEngineW * 0.5 + 0.3, v0),
                       P(xm + kEngineW * 0.5 + 0.3, 1.0 + kEngineL + 0.4), P(xm - kEngineW * 0.5 - 0.3, 1.0 + kEngineL + 0.4)};
            rp.rooms.push_back(ap);
        }
        keepStairClear();
        return rp;
    }
    if (use != 8) {
        room(x0, x1, v0, v1, use == 7 ? RoomKind::Nave : RoomKind::Market);
        return rp;
    }
    // THE CINEMA: the foyer, then the auditoriums side by side (8 m wide or more, 6 m deep or more)
    const Real fd = std::clamp((v1 - v0) * 0.35, Real(4.5), Real(7.0)), vf = v0 + fd;
    const int n = std::clamp(static_cast<int>((x1 - x0) / 9.0), 1, 4);
    const bool screens = v1 - vf >= 6.0 && (x1 - x0) / n >= 8.0;
    // (a cinema with an upper floor: an auditorium its stair stands in is left out -- the stair's hall, open to the
    // foyer)
    Real wx0 = 1e9, wx1 = -1e9, wv1 = -1e9;
    for (const Vec2& q : well) {
        wx0 = std::min(wx0, dot(q - a, d)); wx1 = std::max(wx1, dot(q - a, d));
        wv1 = std::max(wv1, dot(a - q, nOut));
    }
    if (!screens) {
        room(x0, x1, v0, v1, RoomKind::Foyer);
        return rp;
    }
    room(x0, x1, v0, vf, RoomKind::Foyer);
    const Real aw = (x1 - x0) / n;
    for (int i = 0; i < n; ++i) {
        const Real xa = x0 + aw * i, xb = xa + aw;
        const bool stair = well.size() >= 3 && wv1 > vf && wx1 > xa - 1.0 && wx0 < xb + 1.0;
        if (!stair) {
            room(xa, xb, vf, v1, RoomKind::Auditorium);
            RoomWall front;   // onto the foyer, its door at the end away from its neighbour's
            front.a = P(xa, vf);
            front.b = P(xb, vf);
            front.doorAt = (i % 2 == 0) ? 0.12 : 0.88;
            rp.walls.push_back(front);
        }
        if (i + 1 < n) {
            RoomWall side;
            side.a = P(xb, vf);
            side.b = P(xb, v1);
            rp.walls.push_back(side);
        }
    }
    return rp;
}

// A CHAMFERED RECTANGLE (the tapered glass tower's upper plates: eight corners, each corner cut on the diagonal): the
// rectangle its four long walls stand on (CCW), and per edge of it how far the cut corners take off its start and its
// end. False for anything else.
static bool chamferedRect(const Poly2& planIn, Poly2& rect, Real cutS[4], Real cutE[4]) {
    if (planIn.size() <= 4 || planIn.size() > 8) return false;
    const OBB2 ob = orientedBoundingBox(planIn);
    const Vec2 u = ob.axis[0] * ob.half[0], v = ob.axis[1] * ob.half[1];
    rect = {ob.center - u - v, ob.center + u - v, ob.center + u + v, ob.center - u + v};
    ensureCCW(rect);
    std::vector<bool> on(planIn.size(), false);
    for (int e = 0; e < 4; ++e) {
        const Vec2 a = rect[static_cast<std::size_t>(e)], dv = rect[static_cast<std::size_t>((e + 1) % 4)] - a;
        const Real W = dv.length();
        const Vec2 d = dv * (1.0 / W), nOut(d.y, -d.x);
        Real t0 = 1e9, t1 = -1e9;
        for (std::size_t i = 0; i < planIn.size(); ++i)
            if (std::fabs(dot(planIn[i] - a, nOut)) < 0.05) {
                const Real t = dot(planIn[i] - a, d);
                t0 = std::min(t0, t); t1 = std::max(t1, t);
                on[i] = true;
            }
        if (t1 - t0 < W * 0.34) return false;   // a long wall, cut at most a third at each end
        cutS[e] = std::max(Real(0), t0);
        cutE[e] = std::max(Real(0), W - t1);
    }
    for (bool b : on) if (!b) return false;   // every corner of the plate on one of the four walls
    return true;
}
// THE LARGEST RECTANGLE IN A PLATE, square to its bounding box (Glenn, 2026-10-13, on a tapered tower's ground tier:
// "no meeting rooms, open workspaces, cubes, or even walls" -- a 98 x 64 m plate with a 39 x 5.8 m entrance notch was
// neither a rectangle nor a chamfered one, and fell to a ring of offices round an empty middle). Each side of the box
// steps in until the rectangle lies inside the plate; good enough when it keeps 85% of the plate. What it leaves (the
// notch's flanks) stays open floor.
static bool innerRect(const Poly2& planIn, Poly2& rect) {
    if (planIn.size() < 4) return false;
    const OBB2 ob = orientedBoundingBox(planIn);
    const Vec2 u = ob.axis[0], v = ob.axis[1];
    Real lo[2] = {-ob.half[0], -ob.half[1]}, hi[2] = {ob.half[0], ob.half[1]};
    Poly2 pl = planIn;
    ensureCCW(pl);
    auto P = [&](Real s, Real t) { return ob.center + u * s + v * t; };
    auto sideInside = [&](int axis, bool high) {
        const Real c = high ? hi[axis] : lo[axis];
        const Real a0 = axis == 0 ? lo[1] : lo[0], a1 = axis == 0 ? hi[1] : hi[0];
        for (int i = 0; i <= 40; ++i) {
            const Real w = a0 + (a1 - a0) * (0.002 + 0.996 * i / 40.0);
            const Vec2 q = axis == 0 ? P(c + (high ? -0.01 : 0.01), w) : P(w, c + (high ? -0.01 : 0.01));
            if (!pointInPolygon(pl, q)) return false;
        }
        return true;
    };
    for (int pass = 0; pass < 400; ++pass) {
        bool moved = false;
        for (int axis = 0; axis < 2; ++axis)
            for (bool high : {false, true})
                if (!sideInside(axis, high)) {
                    if (high) hi[axis] -= 0.25; else lo[axis] += 0.25;
                    moved = true;
                }
        if (!moved) break;
        if (hi[0] - lo[0] < 6.0 || hi[1] - lo[1] < 6.0) return false;
    }
    const Real kept = (hi[0] - lo[0]) * (hi[1] - lo[1]);
    if (kept < 0.85 * area(pl)) return false;
    rect = {P(lo[0], lo[1]), P(hi[0], lo[1]), P(hi[0], hi[1]), P(lo[0], hi[1])};
    ensureCCW(rect);
    return true;
}
// THE RECTANGLE A PLATE IS LAID IN: itself; a chamfered one's (its cut corners); else the largest inside it.
static bool officeRectFor(const Poly2& planIn, Poly2& plan, Real cutS[4], Real cutE[4], bool* inner = nullptr) {
    for (int i = 0; i < 4; ++i) cutS[i] = cutE[i] = 0;
    if (inner) *inner = false;
    if (planIn.size() == 4) { plan = planIn; return true; }
    if (chamferedRect(planIn, plan, cutS, cutE)) return true;
    for (int i = 0; i < 4; ++i) cutS[i] = cutE[i] = 0;
    if (inner) *inner = true;
    return innerRect(planIn, plan);
}
static bool officePlanFits(const Poly2& plan) {
    Poly2 r; Real a[4], b[4];
    return officeRectFor(plan, r, a, b);
}

// ------------------------------------------------------------- the hospital
// THE HOSPITAL (Glenn, 2026-10-13: "Do hospitals. I didn't even know we had them"): one a city (a landmark), four to
// six storeys round a core, and it was a ring of studio flats over an empty ground floor. Its floors now:
//
//   UPPER FLOORS, a ward    patients' rooms along the windows (one bed, or two in a room 4.4 m wide or more), each
//                           with its door on the ward corridor; behind the corridor, against the core's, the support
//                           strip: the NURSES' STATION facing the lifts (between a supply room and the utility room),
//                           opposite it the staff room, a restroom and a supply room, a restroom and stores elsewhere.
//   THE GROUND FLOOR        the entrance band is the LOBBY -- an information desk facing the doors, rows of waiting
//                           seats -- and the other bands EXAMINATION rooms along the windows off a corridor, a
//                           restroom and a supply room in their strip.
//
// The bands are the office floor's (even bands own the corners; a chamfered plate laid in its rectangle). The strip's
// ends stay open, so the ward corridor meets the core's corridor round both ends of every band.
static RoomPlan hospitalPlan(const Poly2& planIn, const BuildingParams& params, const CorePlan& core, Real inset,
                             int storey, std::size_t entranceEdge) {
    constexpr Real kWardD = 5.0, kWardW = 4.0, kExamD = 4.0, kExamW = 3.4, kWardCorr = 2.4, kStripMin = 3.0;
    RoomPlan rp;
    rp.topology = PlateTopology::Ring;
    rp.office = true;   // (its bath rooms are restrooms; it is lit as a workplace)
    Real cutS[4] = {0, 0, 0, 0}, cutE[4] = {0, 0, 0, 0};
    Poly2 plan = planIn;
    if (!core.valid) return rp;
    if (!officeRectFor(planIn, plan, cutS, cutE)) return rp;
    ensureCCW(plan);
    const std::size_t n = plan.size();
    const Poly2 coreR = core.rect();
    auto from = [&](std::size_t e) { return cutS[e] > 0 ? cutS[e] + inset * 1.5 : inset; };
    auto upto = [&](std::size_t e, Real W) { return cutE[e] > 0 ? W - cutE[e] - inset * 1.5 : W - inset; };
    struct Band { Vec2 a, d, nOut; Real W = 0, D = 0; bool rooms = false; Real s0 = 0, s1 = 0; };
    std::vector<Band> B(n);
    for (std::size_t e = 0; e < n; ++e) {
        Band& b = B[e];
        b.a = plan[e];
        const Vec2 dv = plan[(e + 1) % n] - b.a;
        b.W = dv.length();
        if (b.W < 1e-6) continue;
        b.d = dv * (1.0 / b.W);
        b.nOut = Vec2(b.d.y, -b.d.x);
        Real toCore = 1e9;
        for (const Vec2& c : coreR) toCore = std::min(toCore, dot(b.a - c, b.nOut));
        b.D = std::min(toCore - kCorridor - inset, Real(16.0));
        b.rooms = b.D >= (storey == 0 ? kExamD : kWardD) - 0.6;
    }
    for (std::size_t e = 0; e < n; ++e) {
        Band& b = B[e];
        if (!b.rooms) continue;
        if (e % 2 == 0) { b.s0 = from(e); b.s1 = upto(e, b.W); }
        else {
            const Band& p = B[(e + n - 1) % n];
            const Band& q = B[(e + 1) % n];
            b.s0 = std::max(inset + (p.rooms ? p.D : 0), from(e));
            b.s1 = std::min(b.W - inset - (q.rooms ? q.D : 0), upto(e, b.W));
        }
    }
    // The band the lifts face (the nurses' station's), the one opposite; the entrance's (the lobby, on the ground).
    std::size_t front = n, entrance = n;
    for (std::size_t e = 0; e < n; ++e)
        if (B[e].W > 1e-6 && dot(B[e].nOut, core.frame.v) < -0.9) front = e;
    if (entranceEdge < planIn.size()) {
        const Vec2 m = (planIn[entranceEdge] + planIn[(entranceEdge + 1) % planIn.size()]) * 0.5;
        Real best = 1e9;
        for (std::size_t e = 0; e < n; ++e) {
            if (B[e].W < 1e-6) continue;
            const Real off = std::fabs(dot(m - B[e].a, B[e].nOut));
            if (off < best) { best = off; entrance = e; }
        }
        if (best > 1.0) entrance = n;
    }
    const std::size_t opposite = ((front < n ? front : 0) + 2) % n;
    for (std::size_t e = 0; e < n; ++e) {
        const Band& b = B[e];
        if (!b.rooms || b.s1 - b.s0 < 6.0) continue;
        auto P = [&](Real x, Real v) { return b.a + b.d * x - b.nOut * v; };
        const Real vIn = inset, vFront = inset + b.D;
        auto room = [&](Real x0, Real x1, Real v0, Real v1, RoomKind kind) {
            Room rm; rm.edge = e; rm.kind = kind;
            rm.rect = {P(x0, v0), P(x1, v0), P(x1, v1), P(x0, v1)};
            rp.rooms.push_back(rm);
        };
        auto wall = [&](Real x0, Real v0, Real x1, Real v1, Real doorAt) {
            RoomWall w; w.a = P(x0, v0); w.b = P(x1, v1); w.doorAt = doorAt;
            rp.walls.push_back(w);
        };
        // THE LOBBY: the whole entrance band, open to the doors and the lifts.
        if (storey == 0 && e == entrance) {
            room(b.s0, b.s1, vIn, vFront, RoomKind::Waiting);
            continue;
        }
        // THE WINDOWS: wards upstairs, examination rooms on the ground, off the corridor behind them.
        const Real rd = std::min(storey == 0 ? kExamD : kWardD, b.D);
        const Real v0 = vIn + rd + kWardCorr;
        const bool strip = vFront - v0 >= kStripMin;
        Real c0 = 1e9, c1 = -1e9;   // the core's span along the band
        for (const Vec2& c : coreR) { const Real t = dot(c - b.a, b.d); c0 = std::min(c0, t); c1 = std::max(c1, t); }
        if (storey > 0 && e == front && !strip) {
            // NO STRIP for the station: it takes the ward row's middle instead, facing the lifts across the corridor
            // (its side 0 the corridor's, so the nurses sit with their backs to the window).
            const Real m = std::clamp((c0 + c1) * 0.5, b.s0 + 3.0 + kWardW, b.s1 - 3.0 - kWardW);
            if (m - 3.0 - b.s0 >= kWardW * 0.6) officeRowAt(rp, params, e, b.W, P, b.s0, m - 3.0, inset, rd, RoomKind::Ward, kWardW);
            if (b.s1 - m - 3.0 >= kWardW * 0.6) officeRowAt(rp, params, e, b.W, P, m + 3.0, b.s1, inset, rd, RoomKind::Ward, kWardW);
            Room st; st.edge = e; st.kind = RoomKind::NurseStation;
            st.rect = {P(m + 3.0, vIn + rd), P(m - 3.0, vIn + rd), P(m - 3.0, vIn), P(m + 3.0, vIn)};
            rp.rooms.push_back(st);
            continue;
        }
        officeRowAt(rp, params, e, b.W, P, b.s0, b.s1, inset, rd, storey == 0 ? RoomKind::Exam : RoomKind::Ward,
                    storey == 0 ? kExamW : kWardW);
        // THE SUPPORT STRIP behind the corridor, packed out from the band's middle (the nurses' station centred on
        // the core's doors), its ends left open.
        if (!strip) continue;
        struct Item { RoomKind kind; Real w; };
        std::vector<Item> items;
        if (storey == 0) items = {{RoomKind::Bath, 3.6}, {RoomKind::Closet, 3.0}};
        else if (e == front) items = {{RoomKind::Closet, 3.0}, {RoomKind::NurseStation, 6.0}, {RoomKind::Closet, 3.0}};
        else if (e == opposite) items = {{RoomKind::Kitchenette, 5.0}, {RoomKind::Bath, 3.6}, {RoomKind::Closet, 3.0}};
        else items = {{RoomKind::Bath, 3.6}, {RoomKind::Closet, 3.0}};
        const Real lo = b.s0 + 2.4, hi = b.s1 - 2.4;
        Real mid = (lo + hi) * 0.5;
        if (e == front && storey > 0) mid = (c0 + c1) * 0.5;
        auto total = [&]() { Real t = 0; for (const Item& it : items) t += it.w; return t; };
        while (!items.empty() && total() > hi - lo) {
            std::size_t drop = items.size() - 1;   // the last that is not the nurses' station
            while (drop > 0 && items[drop].kind == RoomKind::NurseStation) --drop;
            items.erase(items.begin() + static_cast<std::ptrdiff_t>(drop));
        }
        if (items.empty()) continue;
        Real x = std::clamp(mid - total() * 0.5, lo, hi - total());
        const Real x0run = x;
        for (const Item& it : items) {
            const Real x0 = x, x1 = x + it.w;
            room(x0, x1, v0, vFront, it.kind);
            // a station is a counter open to the corridor; every other room a door onto the ward corridor
            if (it.kind != RoomKind::NurseStation) wall(x0, v0, x1, v0, 0.5);
            x = x1;
        }
        // the run's partitions (between rooms, and at its ends) and its back against the core's corridor
        for (std::size_t i = 0; i <= items.size(); ++i) {
            Real xi = x0run;
            for (std::size_t j = 0; j < i; ++j) xi += items[j].w;
            wall(xi, v0, xi, vFront, -1);
        }
        wall(x0run, vFront, x, vFront, -1);
    }
    return rp;
}

PlateTopology plateTopologyFor(const Poly2& plan, const BuildingParams& params, const CorePlan& core) {
    // A RESIDENTIAL tower with a core and a rectangular plate is APARTMENTS, glass or masonry; offices keep the ring.
    if (core.valid && params.residential && plan.size() == 4) return PlateTopology::Apartments;
    if (core.valid || plan.size() < 3 || params.curtainWall) return PlateTopology::Ring;
    const OBB2 ob = orientedBoundingBox(plan);
    return 2.0 * std::min(ob.half[0], ob.half[1]) < kWholeFloorShortSide ? PlateTopology::WholeFloor
                                                                        : PlateTopology::Ring;
}

WallFinish interiorFinishFor(const BuildingParams& p) {
    WallFinish f;
    const bool roundHead = p.window.head == OpeningStyle::Head::Round;
    switch (p.wallPart) {
        case PartId::Concrete:          // BRUTALIST: the wood shuttering left in
            f.kind = WallFinishKind::Masonry;
            f.part = PartId::Concrete;
            f.base = {0.74, 0.73, 0.71};
            f.accent = {0.66, 0.65, 0.63};
            break;
        case PartId::Brick:             // LOFT: a painted brick feature wall.
            // NOT the brick surface: the procedural bakes are driven by world
            // position at facade scale, so a course reads a foot tall from
            // arm's length indoors (Glenn, 2026-09-15: "the brickwall sizing
            // is wrong... if we can't do brick then we shouldn't include
            // it"). Until there is an interior-scale bake, this is paint.
            f.kind = WallFinishKind::Accent;
            f.part = PartId::Interior;
            f.base = {0.88, 0.85, 0.81};
            f.accent = {0.72, 0.52, 0.46};
            break;
        case PartId::Stucco:
            f.kind = WallFinishKind::Wainscot;
            f.part = PartId::Wood;
            if (roundHead) {            // SPANISH: talavera tile under white plaster
                f.bandH = 0.95;
                f.base = {0.93, 0.90, 0.84};
                f.accent = {0.32, 0.50, 0.62};
            } else {                    // DECO: a tall lacquer dado, pale above
                f.bandH = 1.15;
                f.base = {0.90, 0.87, 0.81};
                f.accent = {0.22, 0.20, 0.23};
            }
            break;
        case PartId::Siding:            // CRAFTSMAN: painted timber panelling
            f.kind = WallFinishKind::Wainscot;
            f.part = PartId::Wood;
            f.bandH = 0.9;
            f.base = {0.87, 0.84, 0.74};
            f.accent = {0.82, 0.80, 0.74};
            break;
        case PartId::Metal:             // INDUSTRIAL: dark slats
            f.kind = WallFinishKind::Slats;
            f.part = PartId::Wood;
            f.base = {0.84, 0.84, 0.85};
            f.accent = {0.28, 0.24, 0.20};
            break;
        default:
            if (p.curtainWall) {        // MODERN: a walnut slat wall against white
                f.kind = WallFinishKind::Slats;
                f.part = PartId::Wood;
                f.base = {0.88, 0.87, 0.85};
                f.accent = {0.38, 0.25, 0.17};
            } else {                    // POSTMODERN: a colour block
                f.kind = WallFinishKind::Accent;
                f.part = PartId::Interior;
                f.base = {0.88, 0.86, 0.82};
                f.accent = {0.78, 0.52, 0.47};
            }
            break;
    }
    return f;
}

bool floorIsWalkable(const RoomPlan& rp, const Poly2& planIn, const Vec2& entry,
                     const Poly2& stairWell, std::vector<std::size_t>* unreached) {
    return floorIsWalkable(rp, planIn, entry, stairWell.size() >= 3 ? std::vector<Poly2>{stairWell} : std::vector<Poly2>{},
                           unreached);
}

bool floorIsWalkable(const RoomPlan& rp, const Poly2& planIn, const Vec2& entry,
                     const std::vector<Poly2>& solids, std::vector<std::size_t>* unreached,
                     const std::vector<Vec2>* reach) {
    if (rp.rooms.empty()) return true;
    Poly2 plan = planIn;
    ensureCCW(plan);
    Real minX = 1e9, minZ = 1e9, maxX = -1e9, maxZ = -1e9;
    for (const Vec2& v : plan) {
        minX = std::min(minX, v.x); maxX = std::max(maxX, v.x);
        minZ = std::min(minZ, v.y); maxZ = std::max(maxZ, v.y);
    }
    constexpr Real kCell = 0.15;
    const int nx = static_cast<int>((maxX - minX) / kCell) + 3;
    const int nz = static_cast<int>((maxZ - minZ) / kCell) + 3;
    if (nx < 3 || nz < 3 || nx * nz > 400000) return true;   // degenerate: don't judge it
    std::vector<uint8_t> blocked(static_cast<std::size_t>(nx * nz), 0);
    auto at = [&](int i, int j) { return static_cast<std::size_t>(j * nx + i); };
    auto centre = [&](int i, int j) { return Vec2(minX + (i - 1 + 0.5) * kCell, minZ + (j - 1 + 0.5) * kCell); };
    auto cellOf = [&](const Vec2& p) {
        return std::pair<int, int>(static_cast<int>((p.x - minX) / kCell) + 1,
                                   static_cast<int>((p.y - minZ) / kCell) + 1);
    };
    // Outside the building is not floor. NEITHER IS THE STAIRWELL: it is a
    // hole with a stair in it, and flooding across it made every floor look
    // reachable when the way round was actually blocked (Glenn, 2026-09-15).
    std::vector<Poly2> wells;
    for (const Poly2& s : solids)
        if (s.size() >= 3) { wells.push_back(s); ensureCCW(wells.back()); }
    for (int j = 0; j < nz; ++j)
        for (int i = 0; i < nx; ++i) {
            const Vec2 c = centre(i, j);
            if (!pointInPolygon(plan, c)) { blocked[at(i, j)] = 1; continue; }
            for (const Poly2& well : wells)
                if (pointInPolygon(well, c)) { blocked[at(i, j)] = 1; break; }
        }
    // Every wall, inflated by the capsule's radius, with its doorway left
    // open by the width a capsule actually needs.
    for (const RoomWall& w : rp.walls) {
        const Vec2 dv = w.b - w.a;
        const Real L = dv.length();
        if (L < 1e-6) continue;
        const Vec2 d = dv * (1.0 / L);
        const Vec2 nrm(d.y, -d.x);
        const Real reach = kRoomWallT * 0.5 + kPlayerR;
        Real d0 = -1, d1 = -1;
        if (w.doorAt >= 0 && L > kRoomDoorW + 0.6) {
            const Real c = std::min(std::max(w.doorAt * L, kRoomDoorW * 0.5 + 0.3), L - kRoomDoorW * 0.5 - 0.3);
            d0 = c - (kRoomDoorW * 0.5 - kPlayerR);
            d1 = c + (kRoomDoorW * 0.5 - kPlayerR);
        }
        // PERPENDICULAR ONLY. A square stamp reaches along the wall as far
        // as it reaches across it, so the stamps either side of a doorway
        // close the doorway — every floor then read as blocked and the
        // rebuild ladder flattened it (2026-09-15).
        for (Real t = 0; t <= L; t += kCell * 0.5) {
            if (d0 >= 0 && t > d0 && t < d1) continue;
            const Vec2 base = w.a + d * t;
            for (Real off = -reach; off <= reach; off += kCell * 0.5) {
                const auto [ci, cj] = cellOf(base + nrm * off);
                if (ci >= 0 && ci < nx && cj >= 0 && cj < nz) blocked[at(ci, cj)] = 1;
            }
        }
    }
    // Flood from the entry, then ask every room whether it was reached.
    std::vector<uint8_t> seen(blocked.size(), 0);
    std::vector<std::pair<int, int>> stack;
    auto push = [&](int i, int j) {
        if (i < 0 || i >= nx || j < 0 || j >= nz) return;
        if (blocked[at(i, j)] || seen[at(i, j)]) return;
        seen[at(i, j)] = 1;
        stack.push_back({i, j});
    };
    {   // the entry cell, or the nearest open cell to it
        const auto [ei, ej] = cellOf(entry);
        for (int r = 0; r <= 8 && stack.empty(); ++r)
            for (int dj = -r; dj <= r && stack.empty(); ++dj)
                for (int di = -r; di <= r && stack.empty(); ++di) push(ei + di, ej + dj);
    }
    if (stack.empty()) {
        if (unreached) for (std::size_t k = 0; k < rp.rooms.size(); ++k) unreached->push_back(k);
        return false;
    }
    while (!stack.empty()) {
        const auto [i, j] = stack.back();
        stack.pop_back();
        push(i + 1, j); push(i - 1, j); push(i, j + 1); push(i, j - 1);
    }
    bool all = true;
    for (std::size_t k = 0; k < rp.rooms.size(); ++k) {
        const Room& r = rp.rooms[k];
        const Vec2 c = centroid(r.rect);
        const auto [ci, cj] = cellOf(c);
        bool reached = false;
        for (int dj = -2; dj <= 2 && !reached; ++dj)
            for (int di = -2; di <= 2 && !reached; ++di) {
                const int i = ci + di, j = cj + dj;
                if (i >= 0 && i < nx && j >= 0 && j < nz && seen[at(i, j)]) reached = true;
            }
        if (!reached) {
            if (std::getenv("RT_APT_DEBUG"))
                std::fprintf(stderr, "[walk] unreached room kind %d at %.1f %.1f (entry %.1f %.1f)\n",
                             static_cast<int>(r.kind), c.x, c.y, entry.x, entry.y);
            if (!unreached) return false;
            unreached->push_back(k);
            all = false;
        }
    }
    if (reach)
        for (std::size_t k = 0; k < reach->size(); ++k) {
            const auto [ci, cj] = cellOf((*reach)[k]);
            bool reached = false;
            for (int dj = -4; dj <= 4 && !reached; ++dj)
                for (int di = -4; di <= 4 && !reached; ++di) {
                    const int i = ci + di, j = cj + dj;
                    if (i >= 0 && i < nx && j >= 0 && j < nz && seen[at(i, j)]) reached = true;
                }
            if (reached) continue;
            if (!unreached) return false;
            unreached->push_back(rp.rooms.size() + k);
            all = false;
        }
    return all;
}

RoomPlan roomPlan(const Poly2& planIn, const BuildingParams& params, const CorePlan& core,
                  std::size_t blankEdge, Real inset, int storey,
                  const Poly2& stairWell, std::size_t entranceEdge, const Vec2& stairFoot, const std::string& storeyUse) {
    const PlateTopology topo = plateTopologyFor(planIn, params, core);
    // WHAT THE FLOOR IS FOR (building_uses.lua): a use that names a floor program is laid as that program -- a new one
    // in floor_programs.lua needs no code here; else the planner's default for its kind of floor
    const std::string use = floorUseName(params.floorUse);
    const bool house = use == "house";
    auto programFor = [&](const char* fallback) -> const FloorProgram& {
        return floorProgram(hasFloorProgram(use) ? use : std::string(fallback));
    };
    // THE CORE, or in a building with only a stair, its well standing in for one (no lifts: no reception)
    auto hubFor = [&]() {
        CorePlan hub = core;
        if (!hub.valid && stairWell.size() >= 3) {
            const OBB2 ob = orientedBoundingBox(stairWell);
            hub.valid = true;
            hub.frame.u = ob.axis[0];
            hub.frame.v = ob.axis[1];
            hub.length = 2 * ob.half[0];
            hub.depth = 2 * ob.half[1];
            hub.frame.origin = ob.center - hub.frame.u * ob.half[0] - hub.frame.v * ob.half[1];
        }
        return hub;
    };
    // A TOWER'S STORE OR RESTAURANT FLOOR (storeyUseOf): the program the storey's use names -- `department_store`
    // (a department a floor, its tills by the lifts, fitting rooms and the stockroom against the core) or `dining`
    // (restaurants and bars along the glass off the corridor round the core). What it cannot lay is open floor,
    // never offices with their rooms renamed.
    if (!storeyUse.empty() && storey >= 1 && hasFloorProgram(storeyUse)) {
        const CorePlan hub = hubFor();
        if (hub.valid) {
            const Vec2 entry = core.valid ? core.frame.toWorld({core.length * 0.5, -1.1})
                             : stairFoot.x < 1e29 ? stairFoot : centroid(stairWell);
            RoomPlan zf = zonedFloorPlan(planIn, params, hub, inset, storey, core.valid, entry, stairWell,
                                         floorProgram(storeyUse));
            if (!zf.rooms.empty()) { zf.finish = interiorFinishFor(params); return zf; }
        }
        RoomPlan open;
        open.topology = topo;
        open.office = true;
        open.finish = interiorFinishFor(params);
        return open;
    }
    // A USE WITH A PROGRAM OF ITS OWN (building_uses.lua: a hotel's rooms): laid as it, round the core or the stair.
    // (Offices and flats have their own branches below: a block of flats' fallbacks, an office's curtain wall.)
    if (storey >= 1 && !house && use != "office" && use != "apartments" && hasFloorProgram(use)) {
        const CorePlan hub = hubFor();
        if (hub.valid) {
            const Vec2 entry = core.valid ? core.frame.toWorld({core.length * 0.5, -1.1})
                             : stairFoot.x < 1e29 ? stairFoot : centroid(stairWell);
            RoomPlan zf = zonedFloorPlan(planIn, params, hub, inset, storey, core.valid, entry, stairWell, floorProgram(use));
            if (!zf.rooms.empty()) { zf.finish = interiorFinishFor(params); return zf; }
        }
    }
    // AN OFFICE OR APARTMENT FLOOR IS THE ZONED FLOOR (floor_zones.h, #117 item 35): one planner for every plate, deep
    // or shallow, round a core or a stair; the planners below are the other building types' (a house, a walk-up, a
    // campus hall, a hospital, a hall, a hotel's ring). What the zoned floor cannot lay is open floor.
    if (params.residential && core.valid && storey >= 1 && !params.campus && params.civicRooms == 0) {
        // the `apartments` program: flats along the glass off the corridor round the core, or off its lobby on a
        // shallow plate, else the residents' floor
        const Vec2 entry = core.frame.toWorld({core.length * 0.5, -1.1});
        RoomPlan zf = zonedFloorPlan(planIn, params, core, inset, storey, true, entry, stairWell, programFor("apartments"));
        if (!zf.rooms.empty()) { zf.finish = interiorFinishFor(params); return zf; }
    }
    if (topo == PlateTopology::Apartments) {   // (the lobby storey, or a plate the zoned floor could not lay)
        RoomPlan open;
        open.topology = PlateTopology::Apartments;
        open.office = false;
        open.finish = interiorFinishFor(params);
        return open;
    }
    const OBB2 plateBox = orientedBoundingBox(planIn);
    const Real longSide = 2 * std::max(plateBox.half[0], plateBox.half[1]);
    const bool walkupShape = topo == PlateTopology::Ring || (topo == PlateTopology::WholeFloor && longSide >= 24.0);
    if (params.civicRooms == 6 && core.valid && core.blocks.size() <= 1 && officePlanFits(planIn)) {
        // THE HOSPITAL: wards upstairs, the lobby and examination rooms on the ground. Walked from the lifts (and on
        // the ground from the doors too); a floor it cannot lay falls through to the ring.
        RoomPlan hp = hospitalPlan(planIn, params, core, inset, storey, entranceEdge);
        hp.finish = interiorFinishFor(params);
        const Vec2 entry = core.frame.toWorld({core.length * 0.5, -1.1});
        if (!hp.rooms.empty() && floorIsWalkable(hp, planIn, entry)) return hp;
        if (std::getenv("RT_HOSPITAL_DEBUG"))
            std::fprintf(stderr, "[hospital] storey %d: %zu rooms -- not walkable, falls through\n", storey, hp.rooms.size());
    }
    const int civic = params.civicRooms;
    if ((civic == 7 || civic == 8 || civic == 9 || civic == 14 || civic == 15 || civic == 16) && storey == 0) {
        // A CHURCH, A CINEMA, A MARKET HALL, A FIRE STATION, A FACTORY, A WAREHOUSE: the ground floor is the hall
        // (hallPlan), walked from its front door.
        RoomPlan hp = hallPlan(planIn, params, inset, entranceEdge, stairWell, stairFoot);
        hp.finish = interiorFinishFor(params);
        Vec2 entry = centroid(planIn);
        if (entranceEdge < planIn.size()) {
            const Vec2 m = (planIn[entranceEdge] + planIn[(entranceEdge + 1) % planIn.size()]) * 0.5;
            entry = m + (centroid(planIn) - m) * (inset + 0.8) * (1.0 / std::max(Real(1e-6), (centroid(planIn) - m).length()));
        }
        RoomPlan walk = hp;   // (the floor kept clear -- a parked engine's, a loading strip, a stair's flight -- is no room to reach)
        walk.rooms.erase(std::remove_if(walk.rooms.begin(), walk.rooms.end(),
                                        [](const Room& r) { return r.kind == RoomKind::Apparatus; }), walk.rooms.end());
        if (!hp.rooms.empty() && floorIsWalkable(walk, planIn, entry, stairWell)) return hp;
    }
    const bool civicCorridor = (civic >= 1 && civic <= 5) || (civic >= 10 && civic <= 12 && storey == 0) ||
                               (civic == 9 && storey >= 1);   // (the fire crew's quarters: the residence hall's floor)
    if ((params.campus || civicCorridor) && !core.valid && planIn.size() >= 4 && stairWell.size() >= 3) {
        // A CAMPUS building: its rooms either side of a corridor from the stair, every floor (the ground floor too:
        // a lecture hall, the reading room, dorms), walked from the stair's foot.
        RoomPlan cp = campusPlan(planIn, params, stairWell, inset, storey, stairFoot, entranceEdge);
        cp.finish = interiorFinishFor(params);
        if (!cp.rooms.empty() && floorIsWalkable(cp, planIn, centroid(stairWell), stairWell)) return cp;
        if (std::getenv("RT_CAMPUS_DEBUG"))
            std::fprintf(stderr, "[campus] storey %d: %zu rooms, NOT walkable\n", storey, cp.rooms.size());
        // never an apartment floor in a university hall: open floor, unfurnished, rather than beds in a classroom
        RoomPlan open;
        open.finish = interiorFinishFor(params);
        return open;
    }
    if (walkupShape && params.residential && !core.valid && planIn.size() == 4 && stairWell.size() >= 3) {
        // A WALK-UP's floor: apartments off a corridor from the stair, walked from the stair's foot. A narrow plate
        // (one dwelling a floor) that is long enough for two takes a single-loaded corridor instead.
        RoomPlan wu = walkupPlan(planIn, params, stairWell, inset, storey, stairFoot);
        wu.finish = interiorFinishFor(params);
        if (!wu.rooms.empty() && floorIsWalkable(wu, planIn, centroid(stairWell), stairWell)) return wu;
    }
    // AN OFFICE FLOOR IS ZONED (floor_zones.h; Glenn, 2026-10-16: partition the floor the way a block is cut into
    // lots, tag the lots, furnish by tag): any plate, curved or not, round its core or its stair. RT_FLOOR_PLAN=bands
    // keeps the old rectangle-only office floor below, for A/B.
    if (!params.residential && storey >= 1 && (params.officeUse || (params.curtainWall && topo == PlateTopology::Ring))) {
        const CorePlan hub = hubFor();
        if (hub.valid) {
            const Vec2 entry = core.valid ? core.frame.toWorld({core.length * 0.5, -1.1})
                             : stairFoot.x < 1e29 ? stairFoot : centroid(stairWell);
            RoomPlan zf = zonedFloorPlan(planIn, params, hub, inset, storey, core.valid, entry, stairWell,
                                         programFor("office"));
            if (!zf.rooms.empty()) { zf.finish = interiorFinishFor(params); return zf; }
        }
    }
    // A BLOCK OF FLATS WITH ONLY A STAIR (not a walk-up's or a house's floor, which have their own planners): the zoned
    // apartment floor round the stair's well, as an office building's is (#117 item 35)
    if (params.residential && !house && !core.valid && topo != PlateTopology::WholeFloor && storey >= 1 &&
        stairWell.size() >= 3 && !params.campus && params.civicRooms == 0) {
        const CorePlan hub = hubFor();
        const Vec2 entry = stairFoot.x < 1e29 ? stairFoot : centroid(stairWell);
        RoomPlan zf = zonedFloorPlan(planIn, params, hub, inset, storey, false, entry, stairWell, programFor("apartments"));
        if (!zf.rooms.empty()) { zf.finish = interiorFinishFor(params); return zf; }
    }
    if (params.officeUse || (params.residential && !house && topo != PlateTopology::WholeFloor)) {
        // what the zoned floor could not lay in a workplace or a block of flats (a lobby storey, a plate a metre or
        // two round its core, a tier with no stair): open floor, never another planner's rooms
        RoomPlan open;
        open.topology = topo;
        open.office = params.officeUse;
        open.finish = interiorFinishFor(params);
        return open;
    }
    if (core.blocks.size() > 1 && storey >= 1) {
        // THE RING AND THE DESK ROW plan round one core: a floor whose stairs and lifts stand apart is zoned (the
        // office program), else open floor
        const Vec2 entry = core.frame.toWorld({core.length * 0.5, -1.1});
        RoomPlan zf = zonedFloorPlan(planIn, params, core, inset, storey, true, entry, stairWell, programFor("office"));
        zf.finish = interiorFinishFor(params);
        if (!zf.rooms.empty()) return zf;
        RoomPlan open;
        open.topology = topo;
        open.office = !params.residential;
        open.finish = zf.finish;
        return open;
    }
    RoomPlan rp = topo == PlateTopology::WholeFloor   // a house's floor, else the ring (a hotel's rooms, a civic floor)
                      ? housePlan(planIn, params, stairWell, entranceEdge, inset, storey)
                      : ringPlan(planIn, params, core, blankEdge, inset, storey);
    rp.finish = interiorFinishFor(params);
    if (params.civicRooms == 6)   // a hospital that fell through to the ring: its rooms are still wards
        for (Room& r : rp.rooms) if (r.kind == RoomKind::Flat) r.kind = RoomKind::Ward;
    if (topo != PlateTopology::WholeFloor && rp.rooms.empty()) {
        // too shallow for rooms round the core: a row of desks at the windows, or the residents' lounge
        RoomPlan dr = deskRowPlan(planIn, params, core, inset, storey,
                                  params.residential ? RoomKind::Lounge : RoomKind::DeskRow);
        if (!dr.rooms.empty()) { dr.finish = rp.finish; return dr; }
    }
    if (topo != PlateTopology::WholeFloor) return rp;   // a ring is open by construction

    // WALK IT before shipping it. A house is one dwelling, so a single bad
    // partition can wall the front door off from the stair; the ladder below
    // rebuilds the floor simpler until it passes rather than growing a
    // blocked one (Glenn, 2026-09-15: "I was immediately stopped by walls").
    Poly2 plan = planIn;
    ensureCCW(plan);
    Vec2 entry = centroid(plan);
    if (entranceEdge < plan.size()) {
        const Vec2 a = plan[entranceEdge], b = plan[(entranceEdge + 1) % plan.size()];
        const Vec2 dv = b - a;
        const Real L = dv.length();
        if (L > 1e-6) {
            const Vec2 d = dv * (1.0 / L);
            entry = (a + b) * 0.5 - Vec2(d.y, -d.x) * (inset + 0.6);   // CCW: inward is -nOut
        }
    } else if (stairWell.size() >= 3) {
        entry = centroid(stairWell);
    }
    if (floorIsWalkable(rp, plan, entry, stairWell)) return rp;
    // First rebuild: drop the partitions, keep the doors onto the hall.
    rp.walls.erase(std::remove_if(rp.walls.begin(), rp.walls.end(),
                                  [](const RoomWall& w) { return w.doorAt < 0; }),
                   rp.walls.end());
    if (floorIsWalkable(rp, plan, entry, stairWell)) return rp;
    // Second: an open floor beats a floor you cannot cross.
    rp.walls.clear();
    rp.rooms.clear();
    return rp;
}

namespace {

void quad(RenderMesh& m, RenderMesh* col, const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D,
          const Vec3& nrm, const Vec3& colr) {
    MeshBuilder::emitQuad(m, A, B, C, D, nrm, colr);
    if (col) MeshBuilder::emitQuad(*col, A, B, C, D, nrm, colr);
}

// A wall from a to b (world XZ) of thickness t, floor y0 to y0 + h: two
// skins, end caps, and when `doorAt` >= 0 a doorway with jambs and a head.
void wallRun(RenderMesh& m, RenderMesh* col, const Vec2& a, const Vec2& b, Real y0, Real h, Real t,
             Real doorAt, const Vec3& colr, const Vec3* colrB = nullptr) {
    const Vec2 dv = b - a;
    const Real L = dv.length();
    if (L < 0.05) return;
    const Vec2 d = dv * (1.0 / L);
    const Vec2 nn(d.y, -d.x);   // one side
    auto W = [&](Real x, Real side, Real y) { const Vec2 p = a + d * x + nn * side; return Vec3(p.x, y, p.y); };
    const Vec3 nA(nn.x, 0, nn.y), nB(-nn.x, 0, -nn.y), nD(d.x, 0, d.y), nDm(-d.x, 0, -d.y);
    const Real yt = y0 + h;
    Real d0 = -1, d1 = -1;
    if (doorAt >= 0 && L > kRoomDoorW + 0.6) {
        const Real c = std::min(std::max(doorAt * L, kRoomDoorW * 0.5 + 0.3), L - kRoomDoorW * 0.5 - 0.3);
        d0 = c - kRoomDoorW * 0.5;
        d1 = c + kRoomDoorW * 0.5;
    }
    auto skin = [&](Real side, const Vec3& nrm, const Vec3& paint) {
        auto piece = [&](Real x0, Real x1, Real yb, Real ytop) {
            if (x1 - x0 < 1e-4 || ytop - yb < 1e-4) return;
            quad(m, col, W(x0, side, yb), W(x1, side, yb), W(x1, side, ytop), W(x0, side, ytop), nrm, paint);
        };
        if (d0 < 0) { piece(0, L, y0, yt); return; }
        piece(0, d0, y0, yt);
        piece(d1, L, y0, yt);
        piece(d0, d1, y0 + kRoomDoorH, yt);
    };
    skin(t * 0.5, nA, colr);
    skin(-t * 0.5, nB, colrB ? *colrB : colr);   // side B in its own paint, when it has one
    // End caps.
    quad(m, col, W(0, -t * 0.5, y0), W(0, t * 0.5, y0), W(0, t * 0.5, yt), W(0, -t * 0.5, yt), nDm, colr);
    quad(m, col, W(L, t * 0.5, y0), W(L, -t * 0.5, y0), W(L, -t * 0.5, yt), W(L, t * 0.5, yt), nD, colr);
    if (d0 >= 0) {
        // The doorway's jambs and head, between the skins.
        const Real yh = y0 + kRoomDoorH;
        quad(m, col, W(d0, t * 0.5, y0), W(d0, -t * 0.5, y0), W(d0, -t * 0.5, yh), W(d0, t * 0.5, yh), nD, colr);
        quad(m, col, W(d1, -t * 0.5, y0), W(d1, t * 0.5, y0), W(d1, t * 0.5, yh), W(d1, -t * 0.5, yh), nDm, colr);
        quad(m, col, W(d0, -t * 0.5, yh), W(d1, -t * 0.5, yh), W(d1, t * 0.5, yh), W(d0, t * 0.5, yh),
             Vec3(0, -1, 0), colr);
    }
}

// A glass front: one clear pane on the centre line (both faces shade; the
// GlassClear material is two-sided), the doorway cut, a collider behind it.
void glassRun(RenderMesh& m, RenderMesh* col, const Vec2& a, const Vec2& b, Real y0, Real h, Real doorAt,
              RenderMesh* frost = nullptr) {
    const Vec2 dv = b - a;
    const Real L = dv.length();
    if (L < 0.05) return;
    const Vec2 d = dv * (1.0 / L);
    const Vec2 nn(d.y, -d.x);
    const Vec3 nrm(nn.x, 0, nn.y);
    auto W = [&](Real x, Real y) { const Vec2 p = a + d * x; return Vec3(p.x, y, p.y); };
    const Real yt = y0 + h;
    Real d0 = -1, d1 = -1;
    if (doorAt >= 0 && L > kRoomDoorW + 0.6) {
        const Real c = std::min(std::max(doorAt * L, kRoomDoorW * 0.5 + 0.3), L - kRoomDoorW * 0.5 - 0.3);
        d0 = c - kRoomDoorW * 0.5;
        d1 = c + kRoomDoorW * 0.5;
    }
    // TINTED (Glenn, 2026-10-02: "in the offices the glass should probably be reflective ... and maybe tint a bit
    // so that it stands out"): a cool green-blue over GlassClear's own colour, and a FROSTED STRIP at eye height on
    // both faces -- the manifestation band real office glass carries so nobody walks into it.
    const Vec3 white(0.70, 0.90, 0.92);
    auto piece = [&](Real x0, Real x1, Real yb, Real ytop) {
        if (x1 - x0 < 1e-4 || ytop - yb < 1e-4) return;
        quad(m, col, W(x0, yb), W(x1, yb), W(x1, ytop), W(x0, ytop), nrm, white);
        if (frost && ytop > y0 + 1.6 && yb < y0 + 1.4) {
            const Real s0 = y0 + 1.40, s1 = y0 + 1.52;
            const Vec3 off = nrm * 0.004, fc(0.93, 0.95, 0.96);
            quad(*frost, nullptr, W(x0, s0) + off, W(x1, s0) + off, W(x1, s1) + off, W(x0, s1) + off, nrm, fc);
            quad(*frost, nullptr, W(x1, s0) - off, W(x0, s0) - off, W(x0, s1) - off, W(x1, s1) - off, nrm * -1.0, fc);
        }
    };
    if (d0 < 0) { piece(0, L, y0, yt); return; }
    piece(0, d0, y0, yt);
    piece(d1, L, y0, yt);
    piece(d0, d1, y0 + kRoomDoorH, yt);
    // GLAZING, NOT A PLANE (Glenn, 2026-09-17: "the glass walls are hard to
    // see"). A flat quad has no edge to catch light. But a pane only READS as
    // thick where its edge is visible -- at a reveal -- and this renderer culls
    // no back faces, so a second face and caps along every run cost triangles
    // for nothing. A first cut did all four and put the streamed-window census
    // over its cap (12,900 of 12,000). So: the pane stays one face, and each
    // door jamb gets a 60 mm returned edge, which is the only place you see it.
    {
        const Real gt = human::GLASS_THICKNESS;
        auto jamb = [&](Real x, Real sgn) {
            auto J = [&](Real off, Real y) {
                const Vec2 p = a + d * x + nn * off;
                return Vec3(p.x, y, p.y);
            };
            const Vec3 e(d.x * sgn, 0, d.y * sgn);
            quad(m, nullptr, J(-gt * 0.5, y0), J(gt * 0.5, y0),
                 J(gt * 0.5, y0 + kRoomDoorH), J(-gt * 0.5, y0 + kRoomDoorH), e, white);
        };
        jamb(d0, 1.0);
        jamb(d1, -1.0);
    }
}

// SLATS on one face of a wall: vertical battens standing proud of it, the
// treatment behind every "wood slat wall". 45 mm wide, 25 mm proud, 90 mm
// pitch — the joinery sizes these are actually built at. No collider: a
// batten is 25 mm and the wall behind it already stops the player.
void slatFace(RenderMesh& m, const Vec2& a, const Vec2& b, Real y0, Real h, Real side,
              const Vec3& colr) {
    constexpr Real kW = 0.045, kPitch = 0.09, kD = 0.025;
    const Vec2 dv = b - a;
    const Real L = dv.length();
    if (L < kPitch) return;
    const Vec2 d = dv * (1.0 / L);
    const Vec2 nn(d.y, -d.x);
    const Real face = side >= 0 ? 1.0 : -1.0;
    const Vec3 nOut(nn.x * face, 0, nn.y * face);
    const Vec3 nA(d.x, 0, d.y), nB(-d.x, 0, -d.y);
    auto P = [&](Real x, Real off, Real y) {
        const Vec2 p = a + d * x + nn * (side + off * face);
        return Vec3(p.x, y, p.y);
    };
    const Real yt = y0 + h - 0.02;
    const int n = static_cast<int>((L - kW) / kPitch);
    for (int i = 0; i <= n; ++i) {
        const Real x0 = (L - (n * kPitch + kW)) * 0.5 + i * kPitch, x1 = x0 + kW;
        if (x0 < 0.01 || x1 > L - 0.01) continue;
        MeshBuilder::emitQuad(m, P(x0, kD, y0), P(x1, kD, y0), P(x1, kD, yt), P(x0, kD, yt), nOut, colr);
        MeshBuilder::emitQuad(m, P(x1, 0, y0), P(x1, kD, y0), P(x1, kD, yt), P(x1, 0, yt), nA, colr);
        MeshBuilder::emitQuad(m, P(x0, kD, y0), P(x0, 0, y0), P(x0, 0, yt), P(x0, kD, yt), nB, colr);
    }
}

// A DADO: a panelled band up to `bandH` with a rail on top, standing 20 mm
// proud. Spanish tile, deco lacquer and craftsman panelling are the same
// geometry in different colours.
void dadoFace(RenderMesh& m, const Vec2& a, const Vec2& b, Real y0, Real bandH, Real side,
              const Vec3& colr) {
    const Vec2 dv = b - a;
    const Real L = dv.length();
    if (L < 0.2) return;
    const Vec2 d = dv * (1.0 / L);
    const Vec2 nn(d.y, -d.x);
    const Real face = side >= 0 ? 1.0 : -1.0;
    const Vec3 nOut(nn.x * face, 0, nn.y * face);
    auto P = [&](Real x, Real off, Real y) {
        const Vec2 p = a + d * x + nn * (side + off * face);
        return Vec3(p.x, y, p.y);
    };
    const Real panelD = 0.02, railD = 0.035, railH = 0.06;
    const Real top = y0 + bandH;
    // The panel field, then the rail capping it (front + its underside).
    MeshBuilder::emitQuad(m, P(0, panelD, y0), P(L, panelD, y0), P(L, panelD, top - railH),
                          P(0, panelD, top - railH), nOut, colr);
    MeshBuilder::emitQuad(m, P(0, railD, top - railH), P(L, railD, top - railH), P(L, railD, top),
                          P(0, railD, top), nOut, colr * 0.94);
    MeshBuilder::emitQuad(m, P(0, railD, top), P(L, railD, top), P(L, 0, top), P(0, 0, top),
                          Vec3(0, 1, 0), colr * 0.9);
}

}  // namespace

void emitRooms(RoomMeshes& out, RenderMesh* colliderOut, const RoomPlan& rp, Real y0, Real h,
               const Vec3& paint) {
    const WallFinish& f = rp.finish;
    // A STRONG finish — timber battens, brick, board-formed concrete — is a
    // FEATURE wall: one per floor in a house, two on a ring floor. Putting
    // one on every partition is wrong twice over. A room with four brick
    // walls is a cellar, and the geometry is ruinous: battens on every ring
    // partition took the streamed five-storey window from 4 380 triangles to
    // 76 380 and 16 MB (2026-09-15, core_window_grow_cost_is_bounded). The
    // rest of the accent walls keep the field colour.
    const bool strong = f.kind == WallFinishKind::Slats || f.kind == WallFinishKind::Masonry;
    int strongLeft = rp.topology == PlateTopology::WholeFloor ? 1 : 2;
    for (const RoomWall& w : rp.walls) {
        if (w.glass) {
            glassRun(out.glass, colliderOut, w.a, w.b, y0, h, w.doorAt, &out.drywall);
            continue;
        }
        // The wall itself: the field colour, or the finish where the finish
        // IS the wall (a colour block, or masonry in its own part).
        bool acc = w.accent;
        if (acc && strong) {
            if (strongLeft > 0) --strongLeft;
            else acc = false;
        }
        RenderMesh* target = &out.drywall;
        Vec3 colr = paint;
        if (acc && f.kind == WallFinishKind::Accent) colr = f.accent;
        if (acc && f.kind == WallFinishKind::Masonry) {
            target = &out.accent;
            colr = f.accent;
        }
        if (w.faceA.set || w.faceB.set) {   // a shop's own walls: each face its room's paint and panelling
            const Vec3 ca = w.faceA.set ? w.faceA.paint : colr, cb = w.faceB.set ? w.faceB.paint : colr;
            wallRun(out.drywall, colliderOut, w.a, w.b, y0, h, kRoomWallT, w.doorAt, ca, &cb);
            const Real L = (w.b - w.a).length();
            Real d0 = -1, d1 = -1;   // (the doorway, as wallRun cuts it)
            if (w.doorAt >= 0 && L > kRoomDoorW + 0.6) {
                const Real c = std::min(std::max(w.doorAt * L, kRoomDoorW * 0.5 + 0.3), L - kRoomDoorW * 0.5 - 0.3);
                d0 = c - kRoomDoorW * 0.5;
                d1 = c + kRoomDoorW * 0.5;
            }
            const Vec2 d = (w.b - w.a) * (1.0 / std::max(L, Real(1e-6)));
            for (int k = 0; k < 2; ++k) {
                const RoomWall::Face& fc = k == 0 ? w.faceA : w.faceB;
                if (!fc.set || fc.dadoH <= 0) continue;
                const Real side = k == 0 ? kRoomWallT * 0.5 : -kRoomWallT * 0.5;
                if (d0 < 0) dadoFace(out.drywall, w.a, w.b, y0, fc.dadoH, side, fc.dado);
                else {
                    dadoFace(out.drywall, w.a, w.a + d * d0, y0, fc.dadoH, side, fc.dado);
                    dadoFace(out.drywall, w.a + d * d1, w.b, y0, fc.dadoH, side, fc.dado);
                }
            }
            continue;
        }
        wallRun(*target, colliderOut, w.a, w.b, y0, h, kRoomWallT, w.doorAt, colr);
        if (!acc) continue;
        // ...and the finish that stands proud of it, on both faces: a
        // partition is a wall in two rooms at once.
        for (Real side : {kRoomWallT * 0.5, -kRoomWallT * 0.5}) {
            if (f.kind == WallFinishKind::Slats)
                slatFace(out.accent, w.a, w.b, y0, h, side, f.accent);
            else if (f.kind == WallFinishKind::Wainscot)
                dadoFace(out.accent, w.a, w.b, y0, f.bandH, side, f.accent);
        }
    }
}

}  // namespace engine
