#include "room_plan.h"
#include "../../mesh_builder.h"
#include <algorithm>
#include <cmath>
#include <utility>
#include <cstdint>

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
// corner of the core's outline, measured inward; +inf without a core.
Real depthToCore(const Vec2& a, const Vec2& nOut, const CorePlan& core) {
    if (!core.valid) return 1e9;
    Real best = 1e9;
    for (const Vec2& c : core.rect()) best = std::min(best, dot(a - c, nOut));
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
        const Real room = depthToCore(ed.a, ed.nOut, core) - kCorridorRing - inset;
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
        const Real startAt = prev.rooms ? inset + prev.depth : inset;
        const Real endAt = E[e].W - (next.rooms ? inset + next.depth : inset);
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
        endWall(startAt, prev.rooms ? inset + prev.cornerEnd : inset);
        endWall(endAt, next.rooms ? inset + next.cornerStart : inset);
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

PlateTopology plateTopologyFor(const Poly2& plan, const BuildingParams& params, const CorePlan& core) {
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
        if (!reached) return false;
    }
    return true;
}

RoomPlan roomPlan(const Poly2& planIn, const BuildingParams& params, const CorePlan& core,
                  std::size_t blankEdge, Real inset, int storey,
                  const Poly2& stairWell, std::size_t entranceEdge) {
    const PlateTopology topo = plateTopologyFor(planIn, params, core);
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
void glassRun(RenderMesh& m, RenderMesh* col, const Vec2& a, const Vec2& b, Real y0, Real h, Real doorAt) {
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
    const Vec3 white(1, 1, 1);
    auto piece = [&](Real x0, Real x1, Real yb, Real ytop) {
        if (x1 - x0 < 1e-4 || ytop - yb < 1e-4) return;
        quad(m, col, W(x0, yb), W(x1, yb), W(x1, ytop), W(x0, ytop), nrm, white);
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
            glassRun(out.glass, colliderOut, w.a, w.b, y0, h, w.doorAt);
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
