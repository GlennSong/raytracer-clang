#include "space_graph.h"
#include "circulation.h"

#include "core_plan.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>

namespace engine {

Real FloorSurface::yAt(const Vec2& p) const {
    switch (kind) {
        case Kind::Level: return y0;
        case Kind::Sloped: return y0 + dot(p - origin, grade);
        case Kind::Stepped: {
            // one row per `run` along the climb: the tread's height is the row's top
            const Real g = grade.length();
            if (g < 1e-9 || run <= 0) return y0;
            const Real along = dot(p - origin, grade * (1.0 / g));
            return y0 + rise * std::floor(std::max(Real(0), along) / run);
        }
        case Kind::Helical: {
            const Real g = grade.length();
            if (g < 1e-9) return y0;
            const Real along = dot(p - origin, grade * (1.0 / g));
            return y0 + along * g;
        }
    }
    return y0;
}

const char* cellKindName(CellKind k) {
    switch (k) {
        case CellKind::Hall: return "hall";
        case CellKind::Room: return "room";
        case CellKind::Stair: return "stair";
        case CellKind::Lift: return "lift";
        case CellKind::Roof: return "roof";
        case CellKind::Pad: return "pad";
        case CellKind::Link: return "link";
    }
    return "?";
}

const char* portalKindName(Portal::Kind k) {
    switch (k) {
        case Portal::Kind::Door: return "door";
        case Portal::Kind::Passage: return "passage";
        case Portal::Kind::Void: return "void";
        case Portal::Kind::Stair: return "stair";
        case Portal::Kind::Escalator: return "escalator";
        case Portal::Kind::Lift: return "lift";
        case Portal::Kind::Ramp: return "ramp";
        case Portal::Kind::Vomitory: return "vomitory";
    }
    return "?";
}

const char* cellUseName(RoomKind k) {
    switch (k) {
        case RoomKind::Office: return "office";
        case RoomKind::Flat: return "flat";
        case RoomKind::Hall: return "hall";
        case RoomKind::Living: return "living";
        case RoomKind::Kitchen: return "kitchen";
        case RoomKind::Bath: return "bath";
        case RoomKind::Bed: return "bed";
        case RoomKind::Closet: return "closet";
        case RoomKind::OpenPlan: return "open plan";
        case RoomKind::Meeting: return "meeting";
        case RoomKind::Kitchenette: return "kitchenette";
        case RoomKind::Shop: return "shop";
        case RoomKind::Classroom: return "classroom";
        case RoomKind::Lecture: return "lecture";
        case RoomKind::Lab: return "lab";
        case RoomKind::Reading: return "reading";
        case RoomKind::Stacks: return "stacks";
        case RoomKind::Dorm: return "dorm";
        case RoomKind::DeskRow: return "desk row";
        case RoomKind::Lounge: return "lounge";
        case RoomKind::Reception: return "reception";
        case RoomKind::Gallery: return "gallery";
        case RoomKind::Ward: return "ward";
        case RoomKind::Exam: return "exam";
        case RoomKind::NurseStation: return "nurse station";
        case RoomKind::Waiting: return "waiting";
        case RoomKind::Nave: return "nave";
        case RoomKind::Foyer: return "foyer";
        case RoomKind::Auditorium: return "auditorium";
        case RoomKind::Market: return "market";
        case RoomKind::Cell: return "cell";
        case RoomKind::Courtroom: return "courtroom";
        case RoomKind::Chamber: return "chamber";
        case RoomKind::EngineBay: return "engine bay";
        case RoomKind::Apparatus: return "apparatus";
        case RoomKind::ShopFloor: return "shop floor";
        case RoomKind::Warehouse: return "warehouse";
        case RoomKind::HotelLobby: return "hotel lobby";
    }
    return "?";
}

bool passable(const Portal& p, const WalkerCaps& caps) {
    switch (p.kind) {
        case Portal::Kind::Door:
        case Portal::Kind::Passage:
        case Portal::Kind::Void:
        case Portal::Kind::Vomitory: return true;
        case Portal::Kind::Stair: return caps.stairs;
        case Portal::Kind::Escalator: return caps.escalator;
        case Portal::Kind::Lift: return caps.lift;
        case Portal::Kind::Ramp: return caps.ramp;
    }
    return false;
}

std::vector<bool> reachableFrom(const SpaceGraph& g, int from, const WalkerCaps& caps) {
    const int n = static_cast<int>(g.cells.size());
    std::vector<bool> seen(static_cast<std::size_t>(n), false);
    std::vector<std::vector<int>> adj(static_cast<std::size_t>(n));
    std::deque<int> q;
    for (const Portal& p : g.portals) {
        if (!passable(p, caps)) continue;
        if (p.a >= 0 && p.a < n && p.b >= 0 && p.b < n) {
            adj[static_cast<std::size_t>(p.a)].push_back(p.b);
            adj[static_cast<std::size_t>(p.b)].push_back(p.a);
        } else if (from < 0) {
            // a portal onto the street: the street is where we start
            const int c = p.a >= 0 && p.a < n ? p.a : (p.b >= 0 && p.b < n ? p.b : -1);
            if (c >= 0 && !seen[static_cast<std::size_t>(c)]) { seen[static_cast<std::size_t>(c)] = true; q.push_back(c); }
        }
    }
    if (from >= 0 && from < n && !seen[static_cast<std::size_t>(from)]) { seen[static_cast<std::size_t>(from)] = true; q.push_back(from); }
    while (!q.empty()) {
        const int c = q.front();
        q.pop_front();
        for (int d : adj[static_cast<std::size_t>(c)])
            if (!seen[static_cast<std::size_t>(d)]) { seen[static_cast<std::size_t>(d)] = true; q.push_back(d); }
    }
    return seen;
}

bool reachable(const SpaceGraph& g, int from, int to, const WalkerCaps& caps) {
    if (to < 0 || to >= static_cast<int>(g.cells.size())) return false;
    return reachableFrom(g, from, caps)[static_cast<std::size_t>(to)];
}

std::vector<int> unreachableCells(const SpaceGraph& g, const WalkerCaps& caps) {
    std::vector<int> out;
    if (!g.modelled) return out;
    const std::vector<bool> seen = reachableFrom(g, -1, caps);
    for (std::size_t i = 0; i < seen.size(); ++i) {
        if (seen[i]) continue;
        const Cell& c = g.cells[i];
        if (c.kind == CellKind::Hall && c.freeArea < kCoveredHall) continue;
        out.push_back(static_cast<int>(i));
    }
    return out;
}

std::vector<StoreyPlan> storeyView(const SpaceGraph& g, Real baseY) {
    std::vector<StoreyPlan> view(static_cast<std::size_t>(std::max(0, g.storeys)));
    for (const Cell& c : g.cells) {
        if (c.kind != CellKind::Hall || c.k0 < 0 || c.k0 >= g.storeys) continue;
        StoreyPlan& sp = view[static_cast<std::size_t>(c.k0)];
        sp.plan = c.plan;
        sp.y0 = c.floor.y0 - 0.05 - baseY;
        sp.h = c.height;
        sp.tier = c.tier;
    }
    return view;
}

namespace {

Real distToSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const Real l2 = ab.lengthSquared();
    const Real t = l2 < 1e-12 ? 0.0 : std::max(Real(0), std::min(Real(1), dot(p - a, ab) / l2));
    return (p - (a + ab * t)).length();
}

Real distToOutline(const Vec2& p, const Poly2& poly) {
    Real best = std::numeric_limits<Real>::max();
    for (std::size_t i = 0; i < poly.size(); ++i) best = std::min(best, distToSegment(p, poly[i], poly[(i + 1) % poly.size()]));
    return best;
}

}  // namespace

void steppedFloor(FloorSurface& f, Real storeyH, Real tread, const Vec2& up) {
    f.kind = FloorSurface::Kind::Stepped;
    const int risers = std::max(1, halfFlightRisers(storeyH));
    f.rise = storeyH * 0.5 / risers;
    f.run = tread;
    f.grade = up;
}

SpaceGraph spaceGraphOf(const Poly2& planIn, const BuildingParams& params, Real baseY, const std::vector<RoofItem>* roof) {
    SpaceGraph g;
    Poly2 plan = planIn;
    if (plan.size() < 3) { g.modelled = false; return g; }
    ensureCCW(plan);
    // A GARAGE is decks and ramps you drive up (garageColliderOf), not rooms: no graph of it yet.
    if (params.parkingDecks) { g.modelled = false; return g; }
    const std::size_t entranceEdge = entranceEdgeFor(plan, params);
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, params);
    const std::vector<StairRun> runs = stairRuns(plan, params, entranceEdge, storeys);
    CirculationPlan circ = circulationFor(plan, params, entranceEdge);
    const CorePlan core = asCorePlan(circ);
    const int nS = static_cast<int>(storeys.size());
    g.storeys = nS;
    if (nS == 0) { g.modelled = false; return g; }
    const Poly2 coreRect = core.valid ? core.rect() : Poly2{};

    auto add = [&](const Cell& c) { g.cells.push_back(c); return static_cast<int>(g.cells.size()) - 1; };
    std::vector<int> hallOf(static_cast<std::size_t>(nS), -1);
    std::vector<std::vector<int>> roomsOf(static_cast<std::size_t>(nS));
    // The cell a point on storey k is in: the smallest room holding it, else the hall (not the core), else none.
    auto cellAt = [&](int k, const Vec2& q) {
        int best = -1;
        Real bestA = std::numeric_limits<Real>::max();
        for (int r : roomsOf[static_cast<std::size_t>(k)]) {
            const Cell& c = g.cells[static_cast<std::size_t>(r)];
            if (!pointInPolygon(c.plan, q)) continue;
            const Real a = area(c.plan);
            if (a < bestA) { bestA = a; best = r; }
        }
        if (best >= 0) return best;
        if (core.valid && pointInPolygon(coreRect, q)) return -1;
        if (pointInPolygon(storeys[static_cast<std::size_t>(k)].plan, q)) return hallOf[static_cast<std::size_t>(k)];
        return -1;
    };
    auto doorPortal = [&](int a, int b, Portal::Kind kind, const Vec2& p0, const Vec2& p1, Real head, int storey) {
        Portal po;
        po.a = a;
        po.b = b;
        po.kind = kind;
        po.p0 = p0;
        po.p1 = p1;
        po.head = head;
        po.storey = storey;
        g.portals.push_back(po);
    };

    // STOREY BY STOREY: the hall, its rooms, their doors, the street doors.
    std::vector<StoreyRooms> rooms(static_cast<std::size_t>(nS));
    for (int k = 0; k < nS; ++k) {
        const StoreyPlan& sp = storeys[static_cast<std::size_t>(k)];
        StoreyRooms& sr = rooms[static_cast<std::size_t>(k)];
        sr = storeyRooms(plan, params, baseY, storeys, k, core, runs, entranceEdge);
        Cell hall;
        hall.kind = CellKind::Hall;
        hall.use = RoomKind::Hall;
        hall.plan = sp.plan;
        hall.floor.y0 = baseY + sp.y0 + 0.05;
        hall.floor.origin = centroid(sp.plan);
        hall.height = sp.h;
        hall.k0 = hall.k1 = k;
        hall.tier = sp.tier;
        if (core.valid) hall.holes.push_back(coreRect);
        for (const Poly2& h : sr.holes) hall.holes.push_back(h);
        {   // the floor inside the exterior walls, less the rooms and the core: where a person can stand
            const Poly2 inner = inset(sp.plan, interiorWallInset(params));
            hall.freeArea = area(inner.size() >= 3 ? inner : sp.plan);
            for (const Poly2& h : hall.holes) hall.freeArea -= area(h);
        }
        hallOf[static_cast<std::size_t>(k)] = add(hall);
        for (const Room& r : sr.rp.rooms) {
            if (r.rect.size() < 3 || r.kind == RoomKind::OpenPlan) continue;   // (a zone, not a room: it is the hall's)
            Cell c;
            c.kind = CellKind::Room;
            c.use = r.kind;
            c.style = r.style;
            c.plan = r.rect;
            c.floor = hall.floor;
            c.floor.origin = centroid(r.rect);
            c.height = sp.h;
            c.freeArea = area(r.rect);
            c.k0 = c.k1 = k;
            c.tier = sp.tier;
            roomsOf[static_cast<std::size_t>(k)].push_back(add(c));
            Cell& hc = g.cells[static_cast<std::size_t>(hallOf[static_cast<std::size_t>(k)])];
            hc.holes.push_back(r.rect);
            hc.freeArea -= area(r.rect);
        }
        // AN UNWALLED EDGE IS OPEN: a room edge the planner put no wall along (a lounge or a desk row open to the
        // corridor, a flat's living room off its hall, an office floor's open zones) is a passage to whatever stands
        // across it. The facade, the core's shafts and a mall's atria cover a room edge the way a wall does -- by the
        // stretch they run along it, so a desk row against the core's back and open past both its ends is open there.
        struct Cover { Vec2 a, b; Real tol; };
        std::vector<Cover> covers;
        for (const RoomWall& w : sr.rp.walls) covers.push_back({w.a, w.b, 0.25});
        for (std::size_t i = 0; i < sp.plan.size(); ++i) covers.push_back({sp.plan[i], sp.plan[(i + 1) % sp.plan.size()], 0.7});
        if (core.valid) for (std::size_t i = 0; i < coreRect.size(); ++i) covers.push_back({coreRect[i], coreRect[(i + 1) % coreRect.size()], 0.3});
        for (const Poly2& h : sr.holes) for (std::size_t i = 0; i < h.size(); ++i) covers.push_back({h[i], h[(i + 1) % h.size()], 0.3});
        for (int r : roomsOf[static_cast<std::size_t>(k)]) {
            const Poly2 rect = g.cells[static_cast<std::size_t>(r)].plan;   // (a copy: add() may move the cells)
            for (std::size_t e = 0; e < rect.size(); ++e) {
                const Vec2 A = rect[e], B = rect[(e + 1) % rect.size()];
                Vec2 d = B - A;
                const Real L = d.length();
                if (L < 0.8) continue;
                d = d * (1.0 / L);
                const Vec2 n(-d.y, d.x);
                std::vector<std::pair<Real, Real>> covered;
                for (const Cover& w : covers) {
                    if (std::fabs(dot(w.a - A, n)) > w.tol || std::fabs(dot(w.b - A, n)) > w.tol) continue;
                    const Real ta = dot(w.a - A, d), tb = dot(w.b - A, d);
                    const Real t0 = std::max(Real(0), std::min(ta, tb)), t1 = std::min(L, std::max(ta, tb));
                    if (t1 > t0) covered.push_back({t0, t1});
                }
                std::sort(covered.begin(), covered.end());
                Real at = 0;
                auto gap = [&](Real g0, Real g1) {
                    if (g1 - g0 < 0.8) return;
                    const Vec2 m = A + d * ((g0 + g1) * 0.5);
                    const int ca = cellAt(k, m + n * 0.45), cb = cellAt(k, m - n * 0.45);
                    const int other = ca == r ? cb : ca;
                    if (other < 0 || other == r) return;
                    for (const Portal& po : g.portals)   // (the room across found the same gap first)
                        if (po.storey == k && po.kind == Portal::Kind::Passage && ((po.a == r && po.b == other) || (po.a == other && po.b == r)) &&
                            ((po.p0 + po.p1) * 0.5 - m).length() < 0.5)
                            return;
                    doorPortal(r, other, Portal::Kind::Passage, A + d * g0, A + d * g1, sp.h, k);
                };
                for (const auto& [c0, c1] : covered) { gap(at, c0); at = std::max(at, c1); }
                gap(at, L);
            }
        }
        // The planner's doors: a doorway on a wall joins whatever stands either side of it.
        for (const RoomWall& w : sr.rp.walls) {
            if (w.doorAt < 0) continue;
            Vec2 d = w.b - w.a;
            const Real len = d.length();
            if (len < 1e-6) continue;
            d = d * (1.0 / len);
            const Vec2 n(-d.y, d.x);
            const Vec2 p = w.a + (w.b - w.a) * w.doorAt;
            const int ca = cellAt(k, p + n * 0.45), cb = cellAt(k, p - n * 0.45);
            if (ca < 0 || cb < 0 || ca == cb) continue;
            doorPortal(ca, cb, Portal::Kind::Door, p - d * (kRoomDoorW * 0.5), p + d * (kRoomDoorW * 0.5), kRoomDoorH, k);
        }
        // LINKS (shape_grammar.h linkDoors; cells.h, ADR-0179): a doorway through to the next cell above the ground,
        // and the bridge it opens onto -- a Link cell past the face (the bridge's footprint, or a stub of it when the
        // cells touch), a Passage from whatever stands at the door. The neighbour's graph has the same from its side.
        for (const LinkDoor& ld : linkDoors(sp.plan, params, k, 0)) {
            if (k < 1) continue;
            const Vec2 mid = (ld.a + ld.b) * 0.5;
            const int c = cellAt(k, mid - ld.n * 0.6);
            if (c < 0) continue;
            Cell lc;
            lc.kind = CellKind::Link;
            lc.plan = ld.bridge.size() >= 3 ? ld.bridge : Poly2{ld.a, ld.b, ld.b + ld.n * 1.5, ld.a + ld.n * 1.5};
            ensureCCW(lc.plan);
            lc.height = sp.h - 0.3;
            lc.freeArea = area(lc.plan);
            lc.k0 = lc.k1 = k;
            lc.tier = sp.tier;
            const int li = add(lc);
            doorPortal(c, li, Portal::Kind::Passage, ld.a, ld.b, std::min(sp.h - 0.3, human::DOOR_HEIGHT), k);
        }
        // WIDE OPENINGS in party walls (ADR-0180): the run is a passage into the next cell on every storey it spans --
        // a Link cell past the wall, as a link doorway's stub.
        for (std::size_t e = 0; e < sp.plan.size(); ++e) {
            Vec2 o0, o1;
            if (!partyOpenAt(sp.plan, params, e, k, o0, o1)) continue;
            const Vec2 d = normalize(o1 - o0);
            const Vec2 n(d.y, -d.x);
            const Vec2 mid = (o0 + o1) * 0.5;
            const int c = cellAt(k, mid - n * 0.6);
            if (c < 0) continue;
            Cell lc;
            lc.kind = CellKind::Link;
            lc.plan = {o0, o1, o1 + n * 1.5, o0 + n * 1.5};
            ensureCCW(lc.plan);
            lc.height = sp.h;
            lc.freeArea = area(lc.plan);
            lc.k0 = lc.k1 = k;
            lc.tier = sp.tier;
            const int li = add(lc);
            doorPortal(c, li, Portal::Kind::Passage, o0, o1, sp.h, k);
        }
        // The facade's doors onto the street (the entrance, a shop's own door, the back door).
        if (k == 0)
            for (const WallOpening& o : storeyOpenings(sp.plan, params, k, baseY + sp.y0, sp.h, entranceEdge)) {
                if (!o.door) continue;
                Vec2 d = o.b - o.a;
                const Real len = d.length();
                if (len < 1e-6) continue;
                d = d * (1.0 / len);
                const Vec2 n(-d.y, d.x);
                const Vec2 mid = (o.a + o.b) * 0.5;
                int c = cellAt(k, mid + n * 0.45);
                if (c < 0) c = cellAt(k, mid - n * 0.45);
                if (c < 0) continue;
                Portal po;
                po.a = c;
                po.b = -1;
                po.kind = Portal::Kind::Door;
                po.p0 = o.a;
                po.p1 = o.b;
                po.sill = o.sill;
                po.head = o.head;
                po.storey = 0;
                g.portals.push_back(po);
            }
        // An OPEN VEHICLE BAY (a fire station's, a depot's) is a passage onto the street.
        if (k == 0) {
            Vec2 outward;
            for (const auto& bay : vehicleBayDoors(sp.plan, params, outward)) {
                const Vec2 mid = (bay.first + bay.second) * 0.5;
                const int c = cellAt(k, mid - outward * 0.6);
                if (c < 0) continue;
                Portal po;
                po.a = c;
                po.b = -1;
                po.kind = Portal::Kind::Passage;
                po.p0 = bay.first;
                po.p1 = bay.second;
                po.head = sp.h;
                po.storey = 0;
                g.portals.push_back(po);
            }
        }
    }
    g.entrance = hallOf[0];

    // THE CORE: each stair and hoistway one shaft cell up the building, a door on every storey; the roof deck when
    // the stairs climb on into the bulkhead (coreReachesRoof).
    if (core.valid) {
        const bool roofExit = coreReachesRoof(params);
        const StoreyPlan& top = storeys.back();
        if (roofExit) {
            Cell r;
            r.kind = CellKind::Roof;
            r.plan = top.plan;
            r.holes.push_back(coreRect);
            r.floor.y0 = baseY + top.y0 + top.h + 0.05;
            r.floor.origin = centroid(top.plan);
            r.height = 0;
            r.freeArea = area(r.plan) - area(coreRect);
            if (roof)   // the tanks, the HVAC, the masts' bases, a lantern: not floor (a Screen is the edge, not a hole)
                for (const RoofItem& ri : *roof) {
                    if (ri.kind == RoofItem::Kind::Screen || ri.kind == RoofItem::Kind::Frame || ri.kind == RoofItem::Kind::Sloped ||
                        ri.kind == RoofItem::Kind::Ramp || ri.kind == RoofItem::Kind::Bridge || ri.kind == RoofItem::Kind::BridgeWall ||
                        ri.footprint.size() < 3)
                        continue;   // (a sloped glass top stands over the deck, not on it; a ramp is the way onto a pad)
                    r.holes.push_back(ri.footprint);
                    r.freeArea -= area(ri.footprint);
                }
            r.k0 = r.k1 = nS;
            r.tier = top.tier;
            g.roof = add(r);
            // THE HELIPAD: a floor of its own a metre over the deck, reached by its ramp -- a Ramp portal from the
            // roof cell at the ramp's top edge.
            if (roof) {
                std::vector<int> pads;
                for (const RoofItem& ri : *roof) {
                    if (ri.kind != RoofItem::Kind::Helipad || ri.footprint.size() < 3) continue;
                    Cell pc;
                    pc.kind = CellKind::Pad;
                    pc.plan = ri.footprint;
                    pc.floor.y0 = ri.y0 + ri.h;
                    pc.floor.origin = centroid(ri.footprint);
                    pc.height = 0;
                    pc.freeArea = area(ri.footprint);
                    pc.k0 = pc.k1 = nS;
                    pc.tier = top.tier;
                    pads.push_back(add(pc));
                }
                for (const RoofItem& ri : *roof) {
                    if (ri.kind != RoofItem::Kind::Ramp || ri.footprint.size() != 4) continue;
                    const Vec2 topMid = (ri.footprint[2] + ri.footprint[3]) * 0.5;
                    int best = -1;
                    Real bestD = 1e9;
                    for (int pi : pads) {
                        const Real d = distToOutline(topMid, g.cells[static_cast<std::size_t>(pi)].plan);
                        if (d < bestD) { bestD = d; best = pi; }
                    }
                    if (best < 0 || bestD > 1.0) continue;
                    doorPortal(g.roof, best, Portal::Kind::Ramp, ri.footprint[2], ri.footprint[3], ri.h, nS);
                }
            }
        }
        circ.addTo(g, storeys, baseY, hallOf, roofExit);
    } else if (nS > 1) {
        // THE STRAIGHT STAIR (ADR-0080), run by run (stairRuns): hall to hall up each run's well, the next run's
        // first flight leaving the storey the last arrived at.
        for (const StairRun& r : runs) {
            const InteriorLayout& il = r.il;
            const Vec2 perp(-il.stairDir.y, il.stairDir.x);
            const Vec2 wellC = il.well.size() >= 3 ? centroid(il.well) : il.stairFoot;
            for (int k = r.k0; k < r.k1 && k + 1 < nS; ++k) {
                int a = cellAt(k, il.stairFoot - il.stairDir * 0.3);
                if (a < 0) a = cellAt(k, wellC);
                int b = cellAt(k + 1, wellC);
                if (b < 0) b = hallOf[static_cast<std::size_t>(k + 1)];
                if (a < 0 || b < 0) continue;
                doorPortal(a, b, Portal::Kind::Stair, il.stairFoot - perp * (il.width * 0.5), il.stairFoot + perp * (il.width * 0.5),
                           storeys[static_cast<std::size_t>(k)].h, k);
            }
        }
    }

    // A MALL's escalators and glass lift (bigBox 6): the lower level's hall to the upper's.
    if (nS > 1) {
        const StoreyRooms& sr = rooms[0];
        for (const StoreyRooms::Escalator& e : sr.escalators) {
            const Vec2 perp(-e.dir.y, e.dir.x);
            int a = cellAt(0, e.foot - e.dir * 0.5);
            if (a < 0) a = hallOf[0];
            int b = cellAt(1, e.foot + e.dir * (sr.escalatorRun + 0.5));
            if (b < 0) b = hallOf[1];
            doorPortal(a, b, Portal::Kind::Escalator, e.foot - perp * 0.5, e.foot + perp * 0.5, storeys[0].h, 0);
        }
        if (sr.lift) {
            const Vec2 perp(-sr.liftDir.y, sr.liftDir.x);
            doorPortal(hallOf[0], hallOf[1], Portal::Kind::Lift, sr.liftAt - perp * 0.5, sr.liftAt + perp * 0.5, storeys[0].h, 0);
        }
    }
    return g;
}

// ---------------------------------------------------------------------------------------------------------------
// THE BLUEPRINT SHEET
namespace {

const char* useFill(const Cell& c) {
    if (c.kind == CellKind::Stair) return "#d9c48f";
    if (c.kind == CellKind::Lift) return "#9fb8d9";
    if (c.kind == CellKind::Roof) return "#e8e8e8";
    if (c.kind == CellKind::Pad) return "#c9d6e3";
    if (c.kind == CellKind::Link) return "#e3d6a0";
    if (c.kind == CellKind::Hall) return "#f7f4ec";
    switch (c.use) {
        case RoomKind::Shop: return "#f2b9a0";
        case RoomKind::Office: case RoomKind::Meeting: case RoomKind::OpenPlan: case RoomKind::DeskRow:
        case RoomKind::Reception: case RoomKind::Kitchenette: return "#cfe3f5";
        case RoomKind::Flat: case RoomKind::Living: case RoomKind::Bed: case RoomKind::Dorm: case RoomKind::Lounge: return "#d8ecc8";
        case RoomKind::Kitchen: case RoomKind::Bath: case RoomKind::Closet: return "#eadcf2";
        default: return "#e6dccb";
    }
}

std::string pathOf(const Poly2& p, Real ox, Real oz, Real s, Real tileY) {
    std::string d;
    char buf[64];
    for (std::size_t i = 0; i < p.size(); ++i) {
        std::snprintf(buf, sizeof buf, "%c%.1f %.1f ", i == 0 ? 'M' : 'L', (p[i].x - ox) * s, tileY - (p[i].y - oz) * s);
        d += buf;
    }
    d += "Z ";
    return d;
}

}  // namespace

std::string blueprintSvg(const SpaceGraph& g, const std::string& title, const std::vector<int>& unreachable) {
    // The sheet: one tile per storey (the roof last), five to a row, every tile the whole building's extent so the
    // storeys line up; the ground storey first (top left).
    Vec2 lo(1e30, 1e30), hi(-1e30, -1e30);
    for (const Cell& c : g.cells)
        for (const Vec2& q : c.plan) { lo.x = std::min(lo.x, q.x); lo.y = std::min(lo.y, q.y); hi.x = std::max(hi.x, q.x); hi.y = std::max(hi.y, q.y); }
    if (g.cells.empty()) { lo = Vec2(0, 0); hi = Vec2(1, 1); }
    const Real tile = 260, pad = 14;
    const Real ext = std::max(hi.x - lo.x, hi.y - lo.y) + 2.0;
    const Real s = (tile - 2 * pad) / ext;
    const int levels = g.storeys + (g.roof >= 0 ? 1 : 0);
    const int cols = 5, rows = (levels + cols - 1) / cols;
    const Real W = cols * tile, H = rows * (tile + 24) + 40;
    std::vector<bool> bad(g.cells.size(), false);
    for (int i : unreachable) if (i >= 0 && i < static_cast<int>(bad.size())) bad[static_cast<std::size_t>(i)] = true;
    std::string out;
    char buf[2048];
    std::snprintf(buf, sizeof buf,
                  "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%.0f\" height=\"%.0f\" viewBox=\"0 0 %.0f %.0f\" "
                  "font-family=\"monospace\" font-size=\"11\">\n<rect width=\"100%%\" height=\"100%%\" fill=\"white\"/>\n"
                  "<defs><pattern id=\"bad\" width=\"6\" height=\"6\" patternUnits=\"userSpaceOnUse\" patternTransform=\"rotate(45)\">"
                  "<rect width=\"6\" height=\"6\" fill=\"#ffd0d0\"/><line x1=\"0\" y1=\"0\" x2=\"0\" y2=\"6\" stroke=\"#c02020\" stroke-width=\"2\"/>"
                  "</pattern></defs>\n<text x=\"8\" y=\"18\" font-size=\"14\">%s -- %zu cells, %zu portals, %zu unreachable</text>\n",
                  W, H, W, H, title.c_str(), g.cells.size(), g.portals.size(), unreachable.size());
    out += buf;
    for (int level = 0; level < levels; ++level) {
        const int col = level % cols, row = level / cols;
        const Real tx = col * tile, ty = 40 + row * (tile + 24);
        const Real ox = lo.x - 1.0, oz = lo.y - 1.0, tileY = ty + tile - pad;
        auto toX = [&](Real x) { return tx + pad + (x - ox) * s; };
        auto toY = [&](Real z) { return tileY - (z - oz) * s; };
        std::snprintf(buf, sizeof buf, "<g><text x=\"%.0f\" y=\"%.0f\">%s</text>\n", tx + 6, ty + 12,
                      level == g.storeys ? "roof" : (std::string("storey ") + std::to_string(level)).c_str());
        out += buf;
        // the cells on this level: halls first (under), then rooms, then shafts
        for (int pass = 0; pass < 3; ++pass)
            for (std::size_t i = 0; i < g.cells.size(); ++i) {
                const Cell& c = g.cells[i];
                if (level < c.k0 || level > c.k1) continue;
                const int want = c.kind == CellKind::Hall || c.kind == CellKind::Roof ? 0 : c.kind == CellKind::Room ? 1 : 2;
                if (want != pass) continue;
                std::string d = pathOf(c.plan, ox, oz, s, tileY);
                if (pass == 0) for (const Poly2& h : c.holes) d += pathOf(h, ox, oz, s, tileY);
                // (a string, not a fixed buffer: a hall with thirty rooms' holes is a long path)
                std::snprintf(buf, sizeof buf, "<path transform=\"translate(%.1f,0)\" d=\"", tx + pad);
                out += buf;
                out += d;
                std::snprintf(buf, sizeof buf, "\" fill=\"%s\" fill-rule=\"evenodd\" stroke=\"#333\" stroke-width=\"%s\"/>\n",
                              bad[i] ? "url(#bad)" : useFill(c), pass == 0 ? "1.2" : "0.6");
                out += buf;
                if (pass == 1 || pass == 2) {
                    const Vec2 cc = centroid(c.plan);
                    std::snprintf(buf, sizeof buf, "<text x=\"%.1f\" y=\"%.1f\" font-size=\"7\" text-anchor=\"middle\" fill=\"#333\">%s</text>\n",
                                  toX(cc.x), toY(cc.y) + 2, c.kind == CellKind::Room ? cellUseName(c.use) : cellKindName(c.kind));
                    out += buf;
                }
            }
        // the portals crossed on this level
        for (const Portal& p : g.portals) {
            if (p.storey != level) continue;
            const char* col = p.b < 0 ? "#20a040" : p.kind == Portal::Kind::Door ? "#c02020"
                            : p.kind == Portal::Kind::Lift ? "#2050c0" : p.kind == Portal::Kind::Escalator ? "#a040c0" : "#b07000";
            std::snprintf(buf, sizeof buf, "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" stroke-width=\"2.5\"/>\n",
                          toX(p.p0.x), toY(p.p0.y), toX(p.p1.x), toY(p.p1.y), col);
            out += buf;
        }
        out += "</g>\n";
    }
    out += "</svg>\n";
    return out;
}

}  // namespace engine
