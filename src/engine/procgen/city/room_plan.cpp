#include "room_plan.h"
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
    rp.office = params.curtainWall;
    if (storey < 1 || planIn.size() < 3) return rp;
    Poly2 plan = planIn;
    ensureCCW(plan);
    const std::size_t n = plan.size();
    const Real depthWant = rp.office ? kOfficeDepth : kFlatDepth;
    const Real widthWant = rp.office ? kOfficeWidth : kFlatWidth;

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
            // The front, with its door a third of the way along (alternating).
            RoomWall front;
            front.a = P(ra, dOut);
            front.b = P(rb, dOut);
            front.doorAt = (rp.rooms.size() % 2) ? 0.35 : 0.65;
            front.glass = rp.office;
            rp.walls.push_back(front);
            // The partition to the next room. It faces INTO both rooms, so
            // it is the one that wears the finish.
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

    // Even edges first: they take the corners.
    for (std::size_t e = 0; e < n; e += 2) {
        if (!E[e].rooms) continue;
        Real fw, lw;
        lay(e, inset, E[e].W - inset, fw, lw);
        E[e].cornerStart = fw;
        E[e].cornerEnd = lw;
    }
    // Odd edges between them: the span starts past the previous edge's
    // corner room and stops before the next one's; an end partition closes
    // whatever of the band the corner room's own front does not.
    for (std::size_t e = 1; e < n; e += 2) {
        if (!E[e].rooms) continue;
        const Edge& prev = E[(e + n - 1) % n];
        const Edge& next = E[(e + 1) % n];
        // Only a CONVEX corner is shared: there the neighbour's band runs into this one's and takes the corner.
        // At an INSIDE corner (an L, T or cross plate -- a bundled tower once tubes drop out, buildings M10) the
        // two bands meet at the corner without crossing; trimming this one by the neighbour's depth left a dead
        // strip along the wall. Run to the corner instead and close the band with a full end partition.
        const bool convexStart = cross(prev.d, E[e].d) > 1e-6;
        const bool convexEnd = cross(E[e].d, next.d) > 1e-6;
        const Real startAt = prev.rooms && convexStart ? inset + prev.depth : inset;
        const Real endAt = E[e].W - (next.rooms && convexEnd ? inset + next.depth : inset);
        Real fw, lw;
        lay(e, startAt, endAt, fw, lw);
        if (fw <= 0) continue;
        auto P = [&](Real x, Real depth) { return E[e].a + E[e].d * x - E[e].nOut * depth; };
        auto endWall = [&](Real x, Real coveredTo) {
            // The neighbour's corner room covers depth [inset, inset + cornerW]
            // of this end; close the rest of the band's depth.
            const Real from = std::max(inset, coveredTo), to = inset + E[e].depth;
            if (to - from < 0.3) return;
            RoomWall w;
            w.a = P(x, from);
            w.b = P(x, to);
            rp.walls.push_back(w);
        };
        endWall(startAt, prev.rooms && convexStart ? inset + prev.cornerEnd : inset);
        endWall(endAt, next.rooms && convexEnd ? inset + next.cornerStart : inset);
    }
    return rp;
}

// ------------------------------------------------------------- the house
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
static void layoutApartment(RoomPlan& rp, std::size_t e, const std::function<Vec2(Real, Real)>& P, Real x0, Real x1,
                            Real vIn, Real vFront, Real D, Real xd, bool partyWallRight) {
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
        {
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

static RoomPlan apartmentPlan(const Poly2& planIn, const BuildingParams& params, const CorePlan& core, Real inset,
                              int storey) {
    RoomPlan rp;
    rp.topology = PlateTopology::Apartments;
    rp.office = false;
    if (storey < 1 || !core.valid || planIn.size() != 4) return rp;
    Poly2 plan = planIn;
    ensureCCW(plan);
    const std::size_t n = plan.size();
    const Poly2 coreR = core.rect();
    uint32_t h = static_cast<uint32_t>(params.seed) * 2654435761u ^ static_cast<uint32_t>(storey * 97);
    auto rnd = [&]() { h ^= h << 13; h ^= h >> 17; h ^= h << 5; return (h & 0xffffu) / 65535.0; };

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
        b.D = toCore - kCorridor - inset;
        b.rooms = b.D >= 5.0;
        b.D = std::min(b.D, Real(13.0));
    }
    for (std::size_t e = 0; e < n; ++e) {
        Band& b = B[e];
        if (!b.rooms) continue;
        if (e % 2 == 0) { b.s0 = inset; b.s1 = b.W - inset; }
        else {
            const Band& p = B[(e + n - 1) % n];
            const Band& q = B[(e + 1) % n];
            b.s0 = inset + (p.rooms ? p.D : 0);
            b.s1 = b.W - inset - (q.rooms ? q.D : 0);
        }
    }
    for (std::size_t e = 0; e < n; ++e) {
        const Band& b = B[e];
        if (!b.rooms || b.s1 - b.s0 < 6.8) continue;
        auto P = [&](Real x, Real v) { return b.a + b.d * x - b.nOut * v; };   // v: depth in from the wall line
        const Real vIn = inset, vFront = inset + b.D;
        // The corridor's reach along this band: the core's span widened by the corridor.
        Real c0 = 1e9, c1 = -1e9;
        for (const Vec2& c : coreR) { const Real t = dot(c - b.a, b.d); c0 = std::min(c0, t); c1 = std::max(c1, t); }
        c0 = std::max(b.s0, c0 - kCorridor);
        c1 = std::min(b.s1, c1 + kCorridor);
        if (c1 - c0 < 3.4) continue;
        // The units: ~7.5-9.5 m each, the first and last stretched to the band's ends but reaching 1.6 m into the
        // corridor's span, so every front door opens onto the corridor.
        const Real want = 7.5 + 2.0 * rnd();
        int units = std::max(1, static_cast<int>((b.s1 - b.s0) / want));
        std::vector<Real> cuts;
        for (; units >= 1; --units) {
            cuts.assign(1, b.s0);
            for (int k = 1; k < units; ++k) cuts.push_back(b.s0 + (b.s1 - b.s0) * k / units);
            cuts.push_back(b.s1);
            if (units > 1) {
                cuts[1] = std::max(cuts[1], c0 + 1.6);
                cuts[units - 1] = std::min(cuts[units - 1], c1 - 1.6);
            }
            bool ok = true;
            for (int k = 0; k < units; ++k)
                if (cuts[k + 1] - cuts[k] < 6.8) ok = false;   // a living room and a kitchen, at the least
            if (ok) break;
        }
        if (units < 1) continue;
        for (int k = 0; k < units; ++k) {
            const Real x0 = cuts[k], x1 = cuts[k + 1];
            // The front door, on the corridor.
            const Real dLo = std::max(x0, c0) + 0.9, dHi = std::min(x1, c1) - 0.9;
            const Real xd = dLo <= dHi ? std::clamp((x0 + x1) * 0.5, dLo, dHi) : (std::max(x0, c0) + std::min(x1, c1)) * 0.5;
            layoutApartment(rp, e, P, x0, x1, vIn, vFront, b.D, xd, k + 1 < units);
        }
    }
    return rp;
}

// THE WALK-UP (Glenn, 2026-10-01: "I didn't see the apartments ... some of them look like dorms"): a residential
// building without a lift core. A corridor runs the long axis from the stair, apartments either side of it
// (double-loaded) -- or, on a plate too narrow for two, one row off a corridor along the back facade. The stair's
// hall is cut out of the band it stands in, facade to corridor, so the stair opens onto the corridor.
static RoomPlan walkupPlan(const Poly2& planIn, const BuildingParams& params, const Poly2& well, Real inset,
                           int storey) {
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
                           int storey) {
    RoomPlan rp;
    rp.topology = PlateTopology::Apartments;
    rp.office = params.campus != 3;
    if (planIn.size() != 4 || well.size() < 3) return rp;
    const OBB2 ob = orientedBoundingBox(planIn);
    const int la = ob.longAxis();
    const Vec2 ua = ob.axis[la];
    const Vec2 va(ua.y, -ua.x);
    const Real hl = ob.half[la], hw = ob.half[1 - la];
    const Vec2 o = ob.center;
    const Real kCorr = 2.4;
    const bool twoSides = 2 * hw >= 2 * (inset + 5.0) + kCorr;
    Real s0 = 1e9, s1 = -1e9, t0 = 1e9, t1 = -1e9;
    for (const Vec2& q : well) {
        const Vec2 d = q - o;
        s0 = std::min(s0, dot(d, ua)); s1 = std::max(s1, dot(d, ua));
        t0 = std::min(t0, dot(d, va)); t1 = std::max(t1, dot(d, va));
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
        for (std::size_t ri = 0; ri < runs.size(); ++ri) {
            const Real r0 = runs[ri].first, r1 = runs[ri].second;
            if (r1 - r0 < 3.4) continue;
            // the rooms of this run: {width, kind}, laid from r0; the last takes the remainder
            struct Seg { Real w; RoomKind kind; };
            std::vector<Seg> want;
            const Real len = r1 - r0;
            auto fill = [&](Real target, Real minW, RoomKind kind) {
                int n = std::max(1, static_cast<int>(len / target));
                while (n > 1 && len / n < minW) --n;
                for (int k = 0; k < n; ++k) want.push_back({len / n, kind});
            };
            // the run beside the stair hall's corridor end: its first room a bath (restrooms / a shared bath)
            const bool byStair = stairHere && runs.size() == 2;
            switch (params.campus) {
                case 1:   // teaching hall
                    if (storey == 0 && bi == 0 && D >= 8.0) fill(13.0 + 3.0 * rnd(), 10.0, RoomKind::Lecture);
                    else fill(8.5, 6.5, RoomKind::Classroom);
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
                    if (storey == 0 && bi == 0 && ri == 0 && want.size() >= 3) {   // the lounge: two rooms merged
                        want[1].w += want[0].w;
                        want[1].kind = RoomKind::Living;
                        want.erase(want.begin());
                    }
                    break;
            }
            if (byStair && ri == 0 && params.campus != 2 && !want.empty() && want.back().w >= 3.0) want.back().kind = RoomKind::Bath;
            roomNo += static_cast<int>(want.size());
            Real x = r0;
            for (std::size_t k = 0; k < want.size(); ++k) {
                const Real x0 = x, x1 = k + 1 == want.size() ? r1 : x + want[k].w;
                x = x1;
                Room rm;
                rm.edge = bi;
                rm.kind = want[k].kind;
                rm.rect = {b.P(x0, vIn), b.P(x1, vIn), b.P(x1, b.vFront), b.P(x0, b.vFront)};
                rp.rooms.push_back(rm);
                // the front, onto the corridor, with the room's door (a lecture hall's near its back corner)
                RoomWall fw;
                fw.a = b.P(x0, b.vFront);
                fw.b = b.P(x1, b.vFront);
                // a teaching room's door near the back corner (the middle of that wall is the front: the board)
                const bool teaching = want[k].kind == RoomKind::Lecture || want[k].kind == RoomKind::Classroom ||
                                      want[k].kind == RoomKind::Lab;
                fw.doorAt = teaching ? (b.alongPlus ? 0.88 : 0.12) : 0.5;
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
    return rp;
}

// ------------------------------------------------------------- the office floor
// THE OFFICE FLOOR (Glenn, 2026-09-30: "the wider floors seem to have a lot of floorspace which in an office would
// be good for cubicles or open floorplans"). A glass tower's typical floor, rectangular with a core:
//
//   +-----+---------------------------+-----+    the corners: glass-fronted corner OFFICES;
//   | ofc |   open plan (desks by the  | ofc |    the rest of each band, windows to the corridor: OPEN PLAN,
//   +-----+   windows)  +------+-----+-+-----+    filled with desk clusters (furniture.cpp);
//   |     |   MEETING   |KITCH.|            |    on two bands, a strip against the corridor: a glass MEETING
//   ...         corridor round the core            room, and once a floor a KITCHENETTE (open, counters).
//
// No partitions but the corner offices' and the meeting rooms' glass, so the floor walks by construction.
static RoomPlan officePlan(const Poly2& planIn, const BuildingParams& params, const CorePlan& core, Real inset,
                           int storey) {
    RoomPlan rp;
    rp.topology = PlateTopology::Ring;
    rp.office = true;
    if (storey < 1 || !core.valid || planIn.size() != 4) return rp;
    Poly2 plan = planIn;
    ensureCCW(plan);
    const std::size_t n = plan.size();
    const Poly2 coreR = core.rect();
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
        b.rooms = b.D >= 4.0;
    }
    for (std::size_t e = 0; e < n; ++e) {
        Band& b = B[e];
        if (!b.rooms) continue;
        if (e % 2 == 0) { b.s0 = inset; b.s1 = b.W - inset; }
        else {
            const Band& p = B[(e + n - 1) % n];
            const Band& q = B[(e + 1) % n];
            b.s0 = inset + (p.rooms ? p.D : 0);
            b.s1 = b.W - inset - (q.rooms ? q.D : 0);
        }
    }
    // The longest band gets the kitchenette; it and the band opposite get a meeting room each.
    std::size_t longest = 0;
    for (std::size_t e = 0; e < n; ++e)
        if (B[e].rooms && B[e].s1 - B[e].s0 > B[longest].s1 - B[longest].s0) longest = e;
    const std::size_t opposite = (longest + 2) % n;
    const Real kCornerW = 5.0, kMeetW = 6.4, kMeetD = 4.0, kKitW = 5.0;
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
        auto glassWall = [&](const Vec2& a, const Vec2& c, Real doorAt) {
            RoomWall w; w.a = a; w.b = c; w.glass = true; w.doorAt = doorAt;
            rp.walls.push_back(w);
        };
        Real x0 = b.s0, x1 = b.s1;
        // CORNER OFFICES on the even bands, which own the corners: glass fronts, the door into the open plan.
        if (e % 2 == 0 && b.s1 - b.s0 > 2 * kCornerW + 8.0) {
            const Real cd = std::min(b.D - 1.6, Real(5.0));
            for (int side = 0; side < 2; ++side) {
                const Real a0 = side == 0 ? b.s0 : b.s1 - kCornerW, a1 = side == 0 ? b.s0 + kCornerW : b.s1;
                room(a0, a1, vIn, vIn + cd, RoomKind::Office);
                glassWall(P(a0, vIn + cd), P(a1, vIn + cd), side == 0 ? 0.75 : 0.25);
                const Real xs = side == 0 ? a1 : a0;
                glassWall(P(xs, vIn), P(xs, vIn + cd), -1);
            }
        }
        // THE CORE-SIDE STRIP: a meeting room (and the kitchenette) against the corridor, mid-band.
        const bool meet = (e == longest || e == opposite) && b.D >= kMeetD + 3.5 && b.s1 - b.s0 >= kMeetW + kKitW + 6.0;
        if (meet) {
            const Real mid = (x0 + x1) * 0.5;
            const Real m0 = e == longest ? mid - kMeetW : mid - kMeetW * 0.5, m1 = m0 + kMeetW;
            room(m0, m1, vFront - kMeetD, vFront, RoomKind::Meeting);
            glassWall(P(m0, vFront - kMeetD), P(m1, vFront - kMeetD), 0.5);    // the front onto the open plan
            glassWall(P(m0, vFront - kMeetD), P(m0, vFront), -1);
            glassWall(P(m1, vFront - kMeetD), P(m1, vFront), -1);
            glassWall(P(m0, vFront), P(m1, vFront), -1);                       // and onto the corridor
            if (e == longest) room(m1 + 0.4, m1 + 0.4 + kKitW, vFront - kMeetD, vFront, RoomKind::Kitchenette);
        }
        // THE OPEN PLAN: the band from the windows to the corridor, between the corner offices.
        room(x0, x1, vIn, vFront, RoomKind::OpenPlan);
    }
    (void)params;
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
                     const Poly2& stairWell) {
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
    Poly2 well = stairWell;
    if (well.size() >= 3) ensureCCW(well);
    for (int j = 0; j < nz; ++j)
        for (int i = 0; i < nx; ++i) {
            const Vec2 c = centre(i, j);
            if (!pointInPolygon(plan, c)) blocked[at(i, j)] = 1;
            else if (well.size() >= 3 && pointInPolygon(well, c)) blocked[at(i, j)] = 1;
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
    if (stack.empty()) return false;
    while (!stack.empty()) {
        const auto [i, j] = stack.back();
        stack.pop_back();
        push(i + 1, j); push(i - 1, j); push(i, j + 1); push(i, j - 1);
    }
    for (const Room& r : rp.rooms) {
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
            return false;
        }
    }
    return true;
}

RoomPlan roomPlan(const Poly2& planIn, const BuildingParams& params, const CorePlan& core,
                  std::size_t blankEdge, Real inset, int storey,
                  const Poly2& stairWell, std::size_t entranceEdge) {
    const PlateTopology topo = plateTopologyFor(planIn, params, core);
    if (topo == PlateTopology::Apartments) {
        // Walk it from the corridor in front of the core's doors; a floor that fails falls back to the ring.
        RoomPlan ap = apartmentPlan(planIn, params, core, inset, storey);
        ap.finish = interiorFinishFor(params);
        const Vec2 entry = core.frame.toWorld({core.length * 0.5, -1.1});
        const bool walk = !ap.rooms.empty() && floorIsWalkable(ap, planIn, entry);
        if (std::getenv("RT_APT_DEBUG"))
            std::fprintf(stderr, "[apts] storey %d: %zu rooms %zu walls, walkable %d\n", storey, ap.rooms.size(),
                         ap.walls.size(), walk ? 1 : 0);
        if (walk) return ap;
        RoomPlan rg = ringPlan(planIn, params, core, blankEdge, inset, storey);
        rg.finish = interiorFinishFor(params);
        return rg;
    }
    const OBB2 plateBox = orientedBoundingBox(planIn);
    const Real longSide = 2 * std::max(plateBox.half[0], plateBox.half[1]);
    const bool walkupShape = topo == PlateTopology::Ring || (topo == PlateTopology::WholeFloor && longSide >= 24.0);
    if (params.campus && !core.valid && planIn.size() == 4 && stairWell.size() >= 3) {
        // A CAMPUS building: its rooms either side of a corridor from the stair, every floor (the ground floor too:
        // a lecture hall, the reading room, dorms), walked from the stair's foot.
        RoomPlan cp = campusPlan(planIn, params, stairWell, inset, storey);
        cp.finish = interiorFinishFor(params);
        if (!cp.rooms.empty() && floorIsWalkable(cp, planIn, centroid(stairWell), stairWell)) return cp;
        if (std::getenv("RT_CAMPUS_DEBUG"))
            std::fprintf(stderr, "[campus] storey %d: %zu rooms, NOT walkable\n", storey, cp.rooms.size());
    }
    if (walkupShape && params.residential && !core.valid && planIn.size() == 4 && stairWell.size() >= 3) {
        // A WALK-UP's floor: apartments off a corridor from the stair, walked from the stair's foot. A narrow plate
        // (one dwelling a floor) that is long enough for two takes a single-loaded corridor instead.
        RoomPlan wu = walkupPlan(planIn, params, stairWell, inset, storey);
        wu.finish = interiorFinishFor(params);
        if (!wu.rooms.empty() && floorIsWalkable(wu, planIn, centroid(stairWell), stairWell)) return wu;
    }
    if (topo == PlateTopology::Ring && params.curtainWall && core.valid && planIn.size() == 4) {
        // An OFFICE floor: open plan with corner offices, meeting rooms and a kitchenette (buildings C).
        RoomPlan of = officePlan(planIn, params, core, inset, storey);
        if (!of.rooms.empty()) { of.finish = interiorFinishFor(params); return of; }
    }
    RoomPlan rp = topo == PlateTopology::WholeFloor
                      ? housePlan(planIn, params, stairWell, entranceEdge, inset, storey)
                      : ringPlan(planIn, params, core, blankEdge, inset, storey);
    rp.finish = interiorFinishFor(params);
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
             Real doorAt, const Vec3& colr) {
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
    auto skin = [&](Real side, const Vec3& nrm) {
        auto piece = [&](Real x0, Real x1, Real yb, Real ytop) {
            if (x1 - x0 < 1e-4 || ytop - yb < 1e-4) return;
            quad(m, col, W(x0, side, yb), W(x1, side, yb), W(x1, side, ytop), W(x0, side, ytop), nrm, colr);
        };
        if (d0 < 0) { piece(0, L, y0, yt); return; }
        piece(0, d0, y0, yt);
        piece(d1, L, y0, yt);
        piece(d0, d1, y0 + kRoomDoorH, yt);
    };
    skin(t * 0.5, nA);
    skin(-t * 0.5, nB);
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
