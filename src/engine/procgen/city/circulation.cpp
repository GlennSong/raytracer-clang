#include "circulation.h"
#include "core_plan.h"
#include "space_graph.h"
#include <algorithm>
#include <cmath>

namespace engine {

namespace {
constexpr Real kLiftLobby = 3.2;   // between two banks facing each other: a cab's depth and a bit, door to door
Poly2 rectOf(const SiteFrame& f, Real u0, Real v0, Real u1, Real v1) {
    Poly2 r{f.toWorld({u0, v0}), f.toWorld({u1, v0}), f.toWorld({u1, v1}), f.toWorld({u0, v1})};
    ensureCCW(r);
    return r;
}
}  // namespace

Poly2 Shaft::rect() const { return rectOf(frame, 0, 0, w, d); }
Poly2 CirculationPlan::rect() const { return rectOf(frame, 0, 0, length, depth); }
Poly2 Block::rect() const { return rectOf(frame, 0, 0, length, depth); }

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
                           std::size_t entranceEdge, int lifts) {
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
    for (int n = lifts > 0 ? lifts : hoistwaysFor(params.floors); n >= std::max(1, lifts); --n) {
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
            cp.blocks.push_back(Block{f, L, depth});
            return cp;
        }
    }
    return cp;
}

// THE SHAFTS PLACED APART (step 3b): the lifts in one bank or two facing across a lift lobby, seated at the top
// tier's centre; the stairs beside them in their rows, or one at each end of the long axis, slid out toward the
// end walls as far as every tier allows. Every block fits inside every tier: a lift block with the corridor ring
// round it, a stairwell at an end with the ring at its door and a wall's margin elsewhere.
CirculationPlan apartPreset(const Poly2& planIn, const std::vector<MassTier>& tiers, const BuildingParams& params,
                            std::size_t entranceEdge, bool ends, bool facing, int lifts) {
    CirculationPlan none;
    Poly2 base = planIn;
    if (base.size() < 3 || tiers.empty()) return none;
    ensureCCW(base);
    const Poly2& top = tiers.back().plan;
    if (top.size() < 3) return none;
    const OBB2 ob = orientedBoundingBox(top);
    const Vec2 mid = ob.center;
    const std::size_t e = entranceEdge % base.size();
    Vec2 toDoor = (base[e] + base[(e + 1) % base.size()]) * 0.5 - mid;
    if (toDoor.length() < 1e-6) toDoor = Vec2(0, -1);
    toDoor = normalize(toDoor);
    const Real runMax = halfFlightRun(std::max(params.groundHeight, params.floorHeight));
    const Real stairD = kStairLanding + runMax + kStairLanding;
    const int topStorey = std::max(0, params.floors);
    const int li = ob.longAxis();
    const Vec2 al = ob.axis[li];
    // a rectangle of the frame grown by margins, inside the base and every tier (sampled along its edges)
    auto fits = [&](const SiteFrame& f, Real u0, Real v0, Real u1, Real v1) {
        const Vec2 c[4] = {{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}};
        for (int i = 0; i < 4; ++i) {
            const Vec2 a = c[i], b = c[(i + 1) % 4];
            const int n = std::max(1, static_cast<int>(std::ceil((b - a).length())));
            for (int j = 0; j < n; ++j) {
                const Vec2 q = f.toWorld(a + (b - a) * (static_cast<Real>(j) / n));
                if (!pointInPolygon(base, q)) return false;
                for (const MassTier& t : tiers)
                    if (t.plan.size() < 3 || !pointInPolygon(t.plan, q)) return false;
            }
        }
        return true;
    };
    auto stairShaft = [&](const SiteFrame& f, Real u0, int block) {
        Shaft st;
        st.kind = Shaft::Kind::Stair;
        st.frame.origin = f.toWorld({u0, 0});
        st.frame.u = f.u;
        st.frame.v = f.v;
        st.w = kStairShaftW;
        st.d = stairD;
        st.k0 = 0;
        st.k1 = topStorey;
        st.doorU = kStairShaftW * 0.5;
        st.doorW = 0.95;
        st.doorH = human::DOOR_HEIGHT;
        st.block = block;
        return st;
    };
    // A ROW: [stair] hoistways [stair] along u from the frame's origin, doors in the v = 0 wall, the service block
    // behind the hoistways where the row is deeper than they are. Its length when `cp` is null.
    auto row = [&](CirculationPlan* cp, const SiteFrame& f, int nHw, bool stairStart, bool stairEnd, Real D, int bank,
                   int block) {
        Real u = 0;
        if (stairStart) {
            if (cp) cp->shafts.push_back(stairShaft(f, u, block));
            u += kStairShaftW + kShaftWall;
        }
        const Real hw0 = u;
        for (int i = 0; i < nHw; ++i) {
            if (cp) {
                Shaft h;
                h.kind = Shaft::Kind::Hoistway;
                h.frame.origin = f.toWorld({u, 0});
                h.frame.u = f.u;
                h.frame.v = f.v;
                h.w = kHoistwayW;
                h.d = kHoistwayD;
                h.k0 = 0;
                h.k1 = topStorey;
                h.doorU = kHoistwayW * 0.5;
                h.doorW = 1.1;
                h.doorH = human::DOOR_HEIGHT;
                h.bank = bank;
                h.block = block;
                cp->banks[static_cast<std::size_t>(bank)].shafts.push_back(static_cast<int>(cp->shafts.size()));
                cp->shafts.push_back(h);
            }
            u += kHoistwayW + kShaftWall;
        }
        const Real hwLen = nHw * kHoistwayW + (nHw - 1) * kShaftWall;
        if (stairEnd) {
            if (cp) cp->shafts.push_back(stairShaft(f, u, block));
            u += kStairShaftW;
        } else {
            u -= kShaftWall;
        }
        const Real sv0 = kHoistwayD + kShaftWall;
        if (cp && D - sv0 > 1.0) {
            Shaft sv;
            sv.kind = Shaft::Kind::Service;
            sv.frame.origin = f.toWorld({hw0, sv0});
            sv.frame.u = f.u;
            sv.frame.v = f.v;
            sv.w = hwLen;
            sv.d = D - sv0;
            sv.k0 = 0;
            sv.k1 = topStorey;
            sv.block = block;
            cp->shafts.push_back(sv);
        }
        return u;
    };
    const Real m = kCoreCorridor;
    // the lifts at the top tier's centre, else slid along its long axis a metre at a time (a notch cut into the back
    // of a slab is where the middle would be)
    std::vector<Real> slides = {0.0};
    for (Real d = 1.0; d < ob.half[li]; d += 1.0) { slides.push_back(d); slides.push_back(-d); }
    for (int n = lifts > 0 ? lifts : hoistwaysFor(params.floors); n >= std::max(1, lifts); --n) {
        for (Real slide : slides)
        for (int orient : {li, 1 - li}) {
            CirculationPlan cp;
            const Vec2 centre = ob.center + al * slide;
            const Vec2 au = ob.axis[orient];
            const Vec2 av0 = ob.axis[1 - orient];
            const Real D = ends ? kHoistwayD : stairD;
            if (!facing || n < 2) {
                // ONE BANK, its doors toward the entrance (the classic core's rule)
                const Vec2 av = dot(av0, toDoor) > 0 ? av0 * -1.0 : av0;
                SiteFrame f;
                f.u = au;
                f.v = av;
                const Real L = row(nullptr, f, n, !ends, !ends, D, 0, 0);
                f.origin = centre - au * (L * 0.5) - av * (D * 0.5);
                if (!fits(f, -m, -m, L + m, D + m)) continue;
                cp.banks.push_back(Bank{{}, e});
                row(&cp, f, n, !ends, !ends, D, 0, 0);
                cp.blocks.push_back(Block{f, L, D});
                cp.zones.push_back(Zone{0, 0, topStorey});
            } else {
                // TWO BANKS FACING across the lift lobby, a stair at each row's far end (diagonally opposite) unless
                // the stairs stand at the ends
                const int nA = (n + 1) / 2, nB = n / 2;
                SiteFrame fa, fb;
                fa.u = au;
                fa.v = av0;
                fb.u = au * -1.0;
                fb.v = av0 * -1.0;
                const Real LA = row(nullptr, fa, nA, !ends, false, D, 0, 0);
                const Real LB = row(nullptr, fb, nB, !ends, false, D, 1, 1);
                const Real Lm = std::max(LA, LB);
                SiteFrame whole;   // the pair with its lobby, centred
                whole.u = au;
                whole.v = av0;
                whole.origin = centre - au * (Lm * 0.5) - av0 * (kLiftLobby * 0.5 + D);
                if (!fits(whole, -m, -m, Lm + m, kLiftLobby + 2 * D + m)) continue;
                fa.origin = centre + av0 * (kLiftLobby * 0.5) - au * (Lm * 0.5);
                fb.origin = centre - av0 * (kLiftLobby * 0.5) + au * (Lm * 0.5);
                cp.banks.push_back(Bank{{}, e});
                cp.banks.push_back(Bank{{}, e});
                row(&cp, fa, nA, !ends, false, D, 0, 0);
                cp.blocks.push_back(Block{fa, LA, D});
                row(&cp, fb, nB, !ends, false, D, 1, 1);
                cp.blocks.push_back(Block{fb, LB, D});
                cp.zones.push_back(Zone{0, 0, topStorey});
                cp.zones.push_back(Zone{1, 0, topStorey});
            }
            if (ends) {
                // ONE STAIRWELL AT EACH END of the long axis, its door toward the middle, as far out as fits
                const Real side = kShaftWall + interiorWallInset(params) + 0.2;
                bool both = true;
                for (Real sgn : {1.0, -1.0}) {
                    SiteFrame f;
                    f.v = al * sgn;   // away from the door: out toward the end wall
                    f.u = Vec2(-f.v.y, f.v.x);
                    Real reach = 0;   // the lifts' extent toward this end
                    for (const Block& b : cp.blocks)
                        for (const Vec2& q : b.rect()) reach = std::max(reach, dot(q - centre, f.v));
                    const Real tMin = reach + 2 * m;
                    Real best = -1;
                    Real tMax = tMin;   // the end wall, less the stair's depth
                    for (const Vec2& q : base) tMax = std::max(tMax, dot(q - centre, f.v) - stairD);
                    for (Real t = tMax; t >= tMin; t -= 0.5) {
                        f.origin = centre + f.v * t - f.u * (kStairShaftW * 0.5);
                        if (fits(f, -side, -m, kStairShaftW + side, stairD + side)) { best = t; break; }
                    }
                    if (best < 0) { both = false; break; }
                    f.origin = centre + f.v * best - f.u * (kStairShaftW * 0.5);
                    const int block = static_cast<int>(cp.blocks.size());
                    cp.shafts.push_back(stairShaft(f, 0, block));
                    cp.blocks.push_back(Block{f, kStairShaftW, stairD});
                }
                if (!both) continue;
            }
            cp.valid = true;
            cp.frame = cp.blocks.front().frame;
            cp.length = cp.blocks.front().length;
            cp.depth = cp.blocks.front().depth;
            return cp;
        }
    }
    return none;
}

uint8_t stairPlaceOf(const std::string& word) { return word == "core" ? 1 : word == "ends" ? 2 : 0; }
uint8_t liftBanksOf(const std::string& word) { return word == "bank" ? 1 : word == "facing" ? 2 : 0; }

std::vector<CirculationLayout> circulationLayouts(const BuildingParams& params, const std::vector<MassTier>& tiers) {
    const CirculationLayout all[4] = {{false, false}, {true, false}, {false, true}, {true, true}};
    const bool slab = !tiers.empty() && tiers.back().plan.size() >= 3 && [&] {
        const OBB2 ob = orientedBoundingBox(tiers.back().plan);
        const int li = ob.longAxis();
        // (flats either side of the corridor band round the lifts need the depth: a 15.6 m pencil tower's strips came
        // to 4.45 m against the apartments program's 4.5)
        return ob.half[li] >= kSlabAspect * ob.half[1 - li] && (!params.residential || 2 * ob.half[1 - li] >= kEndsFlatsDepth);
    }();
    const bool tall = hoistwaysFor(params.floors) >= 3;
    const bool facingFits = !params.residential || (!tiers.empty() && tiers.back().plan.size() >= 3 && [&] {
        const OBB2 ob = orientedBoundingBox(tiers.back().plan);
        return 2 * ob.half[1 - ob.longAxis()] >= kFacingFlatsDepth;
    }());
    // per-mille weights, classic / ends / facing / both
    int w[4] = {1000, 0, 0, 0};
    if (slab && tall) { w[0] = 350; w[1] = 300; w[2] = 150; w[3] = 200; }
    else if (slab) { w[0] = 450; w[1] = 550; }
    else if (tall) { w[0] = 650; w[2] = 350; }
    if (!facingFits) { w[0] += w[2]; w[1] += w[3]; w[2] = w[3] = 0; }   // (homes on a slim crown: one bank)
    std::vector<CirculationLayout> out;
    int sum = 0;
    for (int i = 0; i < 4; ++i) {
        const bool askedOk = (params.stairPlace == 0 || all[i].ends == (params.stairPlace == 2)) &&
                             (params.liftBanks == 0 || all[i].facing == (params.liftBanks == 2));
        if (!askedOk) { w[i] = 0; continue; }
        if (w[i] == 0 && params.stairPlace == 0 && params.liftBanks == 0) continue;
        if (w[i] == 0) w[i] = 1;   // asked for by name: its own layout, whatever the shape
        sum += w[i];
    }
    // the seed's draw first, then the rest in the fallback order (classic, ends, facing, both)
    if (sum > 0) {
        int r = static_cast<int>((((params.seed ^ 0x5bd1e995u) * 2654435761u) >> 12) % static_cast<uint32_t>(sum));
        for (int i = 0; i < 4; ++i) {
            if (w[i] == 0) continue;
            if (r < w[i]) { out.push_back(all[i]); break; }
            r -= w[i];
        }
    }
    for (int i = 0; i < 4; ++i)
        if (w[i] > 0 && (out.empty() || out.front().ends != all[i].ends || out.front().facing != all[i].facing))
            out.push_back(all[i]);
    // where none of those fits, the classic core: a building always tries for one
    if (w[0] == 0) out.push_back(all[0]);
    return out;
}

CirculationPlan circulationOf(const Poly2& plan, const std::vector<MassTier>& tiers, const BuildingParams& params,
                              std::size_t entranceEdge) {
    // every layout at the full count of lifts before any at one fewer: a tower that drew facing banks too wide for
    // its floor takes the classic core's three lifts, not facing banks of one
    const std::vector<CirculationLayout> layouts = circulationLayouts(params, tiers);
    for (int n = hoistwaysFor(params.floors); n >= 1; --n)
        for (const CirculationLayout& l : layouts) {
            // the classic core seated at the centre (byte-identical to the old layout), else slid clear of a notch
            CirculationPlan c = l.ends || l.facing ? apartPreset(plan, tiers, params, entranceEdge, l.ends, l.facing, n)
                                                   : corePreset(plan, tiers, params, entranceEdge, n);
            if (!c.valid && !l.ends && !l.facing) c = apartPreset(plan, tiers, params, entranceEdge, false, false, n);
            if (c.valid) return c;
        }
    return {};
}

CirculationPlan circulationFor(const Poly2& plan, const BuildingParams& params, std::size_t entranceEdge) {
    if (!wantsCore(params) || plan.size() < 3) return {};
    return circulationOf(plan, massStack(plan, params), params, entranceEdge);
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
        cs.block = s.block;
        return cs;
    };
    for (const Block& b : c.blocks) {
        CoreBlock cb;
        cb.frame = b.frame;
        cb.length = b.length;
        cb.depth = b.depth;
        cp.blocks.push_back(cb);
    }
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
            cp.services.push_back(shaftOf(s));
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
