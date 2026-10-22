#include "circulation.h"
#include "core_plan.h"
#include "space_graph.h"
#include <algorithm>
#include <cmath>

namespace engine {

namespace {
Poly2 rectOf(const SiteFrame& f, Real u0, Real v0, Real u1, Real v1) {
    Poly2 r{f.toWorld({u0, v0}), f.toWorld({u1, v0}), f.toWorld({u1, v1}), f.toWorld({u0, v1})};
    ensureCCW(r);
    return r;
}
}  // namespace

Poly2 Shaft::rect() const { return rectOf(frame, 0, 0, w, d); }
Poly2 CirculationPlan::rect() const { return rectOf(frame, 0, 0, length, depth); }

bool CirculationPlan::serves(const Shaft& s, int k) const {
    if (s.doorW <= 0 || k < s.k0 || k > s.k1) return false;
    if (s.bank < 0 || zones.empty()) return true;
    // A HOISTWAY stops where its bank's zone says, and at the ground lobby (every bank starts there) and the sky
    // lobby it transfers at: an express hoistway passes the low zone's storeys with its doors blanked.
    if (k == 0 || k == skyLobby) return true;
    for (const Zone& z : zones)
        if (z.bank == s.bank && k >= z.k0 && k <= z.k1) return true;
    return false;
}

const Shaft* CirculationPlan::stairs(std::size_t i) const {
    std::size_t n = 0;
    for (const Shaft& s : shafts)
        if (s.kind == Shaft::Kind::Stair && n++ == i) return &s;
    return nullptr;
}

// THE DOORS take the engine's own clear opening (human::DOOR_HEIGHT), not a local literal: metro_v2_test's player
// capsule is halfHeight 0.8 + radius 0.3 = 2.2 m tall, so a 2.1 m core door had to be CROUCHED through while the
// building's own entrance (2.7 m) did not (Glenn, 2026-09-16: "the cabs are too short to enter... I have to duck
// and crawl"). Every lab level uses a 1.4 m capsule, which is why no test and no walk of mine ever hit it.
CirculationPlan corePreset(const Poly2& planIn, const std::vector<MassTier>& tiers, const BuildingParams& params,
                           std::size_t entranceEdge) {
    CirculationPlan cp;
    Poly2 base = planIn;
    if (base.size() < 3 || tiers.empty()) return cp;
    ensureCCW(base);
    const Poly2& top = tiers.back().plan;   // the smallest tier: every tier below contains it
    if (top.size() < 3) return cp;
    const OBB2 ob = orientedBoundingBox(top);
    const Vec2 centre = ob.center;
    // The lobby is toward the entrance edge: the door wall faces it.
    const std::size_t e = entranceEdge % base.size();
    Vec2 toDoor = (base[e] + base[(e + 1) % base.size()]) * 0.5 - centre;
    if (toDoor.length() < 1e-6) toDoor = Vec2(0, -1);
    toDoor = normalize(toDoor);
    const Real runMax = halfFlightRun(std::max(params.groundHeight, params.floorHeight));
    const Real depth = kStairLanding + runMax + kStairLanding;
    // The core plus its corridor ring must lie inside the base and every tier: sampled along the ring's EDGES (a
    // metre apart, corners included), not just its corners -- an L-shaped or notched plan passes the corner test
    // with a notch cutting straight through the bank.
    auto fitsAll = [&](const SiteFrame& f, Real L, Real D) {
        const Real m = kCoreCorridor;
        const Vec2 c[4] = {{-m, -m}, {L + m, -m}, {L + m, D + m}, {-m, D + m}};
        std::vector<Vec2> samples;
        for (int i = 0; i < 4; ++i) {
            const Vec2 a = c[i], b = c[(i + 1) % 4];
            const int n = std::max(1, static_cast<int>(std::ceil((b - a).length())));
            for (int j = 0; j < n; ++j) samples.push_back(f.toWorld(a + (b - a) * (static_cast<Real>(j) / n)));
        }
        for (const Vec2& q : samples)
            if (!pointInPolygon(base, q)) return false;
        for (const MassTier& t : tiers) {
            if (t.plan.size() < 3) return false;
            for (const Vec2& q : samples)
                if (!pointInPolygon(t.plan, q)) return false;
        }
        return true;
    };
    const int topStorey = std::max(0, params.floors);   // storey 0 is the ground, storeys 1..floors the floors (storeyPlans)
    for (int n = hoistwaysFor(params.floors); n >= 1; --n) {
        const Real L = kStairShaftW + kShaftWall + n * kHoistwayW + (n - 1) * kShaftWall + kShaftWall + kStairShaftW;
        for (int orient = 0; orient < 2; ++orient) {
            const Vec2 au = ob.axis[orient];
            const Vec2 av0 = ob.axis[1 - orient];
            const Vec2 av = dot(av0, toDoor) > 0 ? av0 * -1.0 : av0;   // v points AWAY from the door
            SiteFrame f;
            f.u = au;
            f.v = av;
            f.origin = centre - au * (L * 0.5) - av * (depth * 0.5);
            if (!fitsAll(f, L, depth)) continue;
            cp.valid = true;
            cp.frame = f;
            cp.length = L;
            cp.depth = depth;
            auto shaftAt = [&](Shaft::Kind kind, Real u0, Real v0, Real w, Real d) {
                Shaft s;
                s.kind = kind;
                s.frame.origin = f.toWorld({u0, v0});
                s.frame.u = f.u;
                s.frame.v = f.v;
                s.w = w;
                s.d = d;
                s.k0 = 0;
                s.k1 = topStorey;
                return s;
            };
            auto stairAt = [&](Real u0) {
                Shaft st = shaftAt(Shaft::Kind::Stair, u0, 0, kStairShaftW, depth);
                st.doorU = kStairShaftW * 0.5;
                st.doorW = 0.95;
                st.doorH = human::DOOR_HEIGHT;
                return st;
            };
            Bank bank;
            bank.lobbyEdge = e;
            cp.shafts.push_back(stairAt(0));
            Real u = kStairShaftW + kShaftWall;
            for (int i = 0; i < n; ++i) {
                Shaft h = shaftAt(Shaft::Kind::Hoistway, u, 0, kHoistwayW, kHoistwayD);
                h.doorU = kHoistwayW * 0.5;
                h.doorW = 1.1;
                h.doorH = human::DOOR_HEIGHT;
                h.bank = 0;
                bank.shafts.push_back(static_cast<int>(cp.shafts.size()));
                cp.shafts.push_back(h);
                u += kHoistwayW + kShaftWall;
            }
            cp.shafts.push_back(stairAt(u));
            const Real sv0 = kHoistwayD + kShaftWall;
            if (depth - sv0 > 1.0) {
                Shaft sv = shaftAt(Shaft::Kind::Service, kStairShaftW + kShaftWall, sv0, n * kHoistwayW + (n - 1) * kShaftWall, depth - sv0);
                sv.doorW = 0;
                cp.shafts.push_back(sv);
            }
            cp.banks.push_back(bank);
            cp.zones.push_back(Zone{0, 0, topStorey});
            return cp;
        }
    }
    return cp;
}

CirculationPlan circulationFor(const Poly2& plan, const BuildingParams& params, std::size_t entranceEdge) {
    if (!wantsCore(params) || plan.size() < 3) return {};
    return corePreset(plan, massStack(plan, params), params, entranceEdge);
}

CorePlan asCorePlan(const CirculationPlan& c) {
    CorePlan cp;
    if (!c.valid) return cp;
    cp.valid = true;
    cp.frame = c.frame;
    cp.length = c.length;
    cp.depth = c.depth;
    auto shaftOf = [](const Shaft& s) {
        CoreShaft cs;
        cs.frame = s.frame;
        cs.width = s.w;
        cs.depth = s.d;
        cs.doorX = s.doorU;
        cs.doorWidth = s.doorW;
        cs.doorHeight = s.doorH;
        return cs;
    };
    for (const Shaft& s : c.shafts) {
        switch (s.kind) {
        case Shaft::Kind::Stair: {
            CoreStair st;
            st.shaft = shaftOf(s);
            st.landing = kStairLanding;
            cp.stairs.push_back(st);
            break;
        }
        case Shaft::Kind::Hoistway:
            cp.hoistways.push_back(shaftOf(s));
            break;
        case Shaft::Kind::Service:
            cp.service = shaftOf(s);
            cp.hasService = true;
            break;
        default:
            break;
        }
    }
    return cp;
}

void CirculationPlan::addTo(SpaceGraph& g, const std::vector<StoreyPlan>& storeys, Real baseY, const std::vector<int>& hallOf,
                            bool roofExit) {
    if (!valid || storeys.empty()) return;
    const int nS = static_cast<int>(storeys.size());
    const StoreyPlan& top = storeys.back();
    const Real groundY = baseY + storeys.front().y0 + 0.05;
    auto add = [&](const Cell& c) { g.cells.push_back(c); return static_cast<int>(g.cells.size()) - 1; };
    auto portal = [&](int a, int b, Portal::Kind kind, const Vec2& p0, const Vec2& p1, Real head, int storey) {
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
    for (Shaft& s : shafts) {
        if (s.kind != Shaft::Kind::Stair && s.kind != Shaft::Kind::Hoistway) continue;
        const bool stair = s.kind == Shaft::Kind::Stair;
        const bool toRoof = stair && roofExit && g.roof >= 0;
        Cell c;
        c.kind = stair ? CellKind::Stair : CellKind::Lift;
        c.plan = s.rect();
        c.floor.y0 = groundY;
        if (stair) {
            c.floor.origin = s.frame.toWorld({s.w * 0.5, kStairLanding});
            steppedFloor(c.floor, storeys.front().h, kStairTread, s.frame.v);
        } else {
            c.floor.origin = s.frame.toWorld({s.w * 0.5, s.d * 0.5});
        }
        c.height = (baseY + top.y0 + top.h) - groundY + (toRoof ? kBulkheadH : 0.0);
        c.freeArea = area(c.plan);
        c.k0 = 0;
        c.k1 = toRoof ? nS : nS - 1;
        c.tier = 0;
        s.cell = add(c);
        const Vec2 p0 = s.frame.toWorld({s.doorU - s.doorW * 0.5, 0});
        const Vec2 p1 = s.frame.toWorld({s.doorU + s.doorW * 0.5, 0});
        for (int k = 0; k < nS; ++k) {
            if (!serves(s, k)) continue;
            portal(hallOf[static_cast<std::size_t>(k)], s.cell, stair ? Portal::Kind::Stair : Portal::Kind::Lift, p0, p1, s.doorH, k);
        }
        if (toRoof) portal(s.cell, g.roof, Portal::Kind::Stair, p0, p1, s.doorH, nS);
    }
}

}  // namespace engine
