#include "core_plan.h"
#include "../../mesh_builder.h"
#include <algorithm>
#include <cmath>

namespace engine {

namespace {
constexpr Real kWall = 0.15;       // shaft wall thickness
constexpr Real kHoistW = 2.4;      // hoistway along the bank
constexpr Real kHoistD = 2.6;      // hoistway into the core
constexpr Real kStairW = 2.6;      // two 1.2 m flights + the 0.2 m spine
constexpr Real kLanding = 1.5;
constexpr Real kCorridor = 2.2;    // ring the core needs inside every tier: 1.5 m clear past the inner wall skin
constexpr Real kDoorH = 2.1;
constexpr Real kSlab = 0.25;       // slab thickness (growInterior's)

Poly2 rectOf(const SiteFrame& f, Real u0, Real v0, Real u1, Real v1) {
    Poly2 r{f.toWorld({u0, v0}), f.toWorld({u1, v0}), f.toWorld({u1, v1}), f.toWorld({u0, v1})};
    ensureCCW(r);
    return r;
}
}  // namespace

Poly2 CoreShaft::rect() const { return rectOf(frame, 0, 0, width, depth); }
Poly2 CorePlan::rect() const { return rectOf(frame, 0, 0, length, depth); }

int hoistwaysFor(int floors) {
    if (floors <= 8) return 1;
    if (floors <= 20) return 2;
    if (floors <= 40) return 3;
    return 4;
}

int halfFlightRisers(Real storeyHeight) {
    return std::max(3, static_cast<int>(std::ceil((storeyHeight * 0.5) / 0.20 - 1e-9)));
}

Real halfFlightRun(Real storeyHeight, Real tread) { return halfFlightRisers(storeyHeight) * tread; }

bool wantsCore(const BuildingParams& params) {
    if (params.core == 1) return false;
    if (params.core == 2) return true;
    return params.floors >= 4;
}

CorePlan corePlan(const Poly2& planIn, const std::vector<MassTier>& tiers,
                  const BuildingParams& params, std::size_t entranceEdge) {
    CorePlan cp;
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
    const Real depth = kLanding + runMax + kLanding;
    // The core plus its corridor ring must lie inside the base and every
    // tier: sampled along the ring's EDGES (a metre apart, corners included),
    // not just its corners — an L-shaped or notched plan passes the corner
    // test with a notch cutting straight through the bank.
    auto fitsAll = [&](const SiteFrame& f, Real L, Real D) {
        const Real m = kCorridor;
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
    for (int n = hoistwaysFor(params.floors); n >= 1; --n) {
        const Real L = kStairW + kWall + n * kHoistW + (n - 1) * kWall + kWall + kStairW;
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
            auto shaftAt = [&](Real u0, Real v0, Real w, Real d) {
                CoreShaft s;
                s.frame.origin = f.toWorld({u0, v0});
                s.frame.u = f.u;
                s.frame.v = f.v;
                s.width = w;
                s.depth = d;
                return s;
            };
            auto stairAt = [&](Real u0) {
                CoreStair st;
                st.shaft = shaftAt(u0, 0, kStairW, depth);
                st.shaft.doorX = kStairW * 0.5;
                st.shaft.doorWidth = 0.95;
                st.shaft.doorHeight = kDoorH;
                st.landing = kLanding;
                return st;
            };
            cp.stairs.push_back(stairAt(0));
            Real u = kStairW + kWall;
            for (int i = 0; i < n; ++i) {
                CoreShaft h = shaftAt(u, 0, kHoistW, kHoistD);
                h.doorX = kHoistW * 0.5;
                h.doorWidth = 1.1;
                h.doorHeight = kDoorH;
                cp.hoistways.push_back(h);
                u += kHoistW + kWall;
            }
            cp.stairs.push_back(stairAt(u));
            const Real sv0 = kHoistD + kWall;
            if (depth - sv0 > 1.0) {
                cp.service = shaftAt(kStairW + kWall, sv0, n * kHoistW + (n - 1) * kWall, depth - sv0);
                cp.service.doorWidth = 0;
                cp.hasService = true;
            }
            return cp;
        }
    }
    return cp;
}

CorePlan coreFor(const Poly2& plan, const BuildingParams& params, std::size_t entranceEdge) {
    if (!wantsCore(params) || plan.size() < 3) return {};
    return corePlan(plan, massStack(plan, params), params, entranceEdge);
}

std::vector<Poly2> coreSlabHoles(const CorePlan& core) {
    std::vector<Poly2> holes;
    if (!core.valid) return holes;
    for (const CoreShaft& h : core.hoistways) holes.push_back(h.rect());
    for (const CoreStair& s : core.stairs) holes.push_back(s.shaft.rect());
    return holes;
}

namespace {
// A vertical quad between two XZ points with an explicit horizontal normal,
// into a mesh and (optionally) the collider.
void wallQuad(RenderMesh& m, RenderMesh* col, const Vec3& a, const Vec3& b, Real yb, Real yt,
              const Vec3& n, const Vec3& color) {
    if ((b - a).lengthSquared() < 1e-8 || yt - yb < 1e-4) return;
    const Vec3 A(a.x, yb, a.z), B(b.x, yb, b.z), C(b.x, yt, b.z), D(a.x, yt, a.z);
    MeshBuilder::emitQuad(m, A, B, C, D, n, color);
    if (col) MeshBuilder::emitQuad(*col, A, B, C, D, n, color);
}
// A horizontal quad (u0..u1 × v0..v1 in the shaft frame) at height y, facing up
// (top = true) or down.
void deckQuad(RenderMesh& m, RenderMesh* col, const CoreShaft& s, Real u0, Real v0, Real u1, Real v1,
              Real y, bool top, const Vec3& color) {
    const Vec3 A = s.at(u0, v0, y), B = s.at(u1, v0, y), C = s.at(u1, v1, y), D = s.at(u0, v1, y);
    const Vec3 n(0, top ? 1 : -1, 0);
    if (top) MeshBuilder::emitQuad(m, A, B, C, D, n, color);
    else MeshBuilder::emitQuad(m, A, D, C, B, n, color);
    if (col) MeshBuilder::emitQuad(*col, A, B, C, D, n, color);
}

// The four INNER skins of a shaft for one storey, on the shaft line facing
// in, the door cut into the v = 0 wall with its reveals out to the core's
// outer ring (coreRing). Colliders for every skin. The outer face of the
// core is the ring's, not the shafts': per-shaft outer skins left the
// wall-thick slot between two neighbours open — a dark slit beside every
// hoistway door (Glenn's walk, 2026-09-14: "gaps in the geometry
// surrounding the stairwell and elevators").
void shaftWalls(CoreMeshes& out, RenderMesh* col, const CorePlan& core, const CoreShaft& s, Real y0,
                Real h, const Vec3& paintOut, const Vec3& paintIn) {
    (void)core;
    // Edges in the shaft frame, CCW seen from above with v "up the page":
    // 0: v = 0 (u 0 -> W), 1: u = W (v 0 -> D), 2: v = D (u W -> 0), 3: u = 0 (v D -> 0).
    struct Edge { Vec2 a, b; Vec2 nOut; };   // frame coords; nOut in frame axes
    const Real W = s.width, D = s.depth;
    const Edge edges[4] = {{{0, 0}, {W, 0}, {0, -1}}, {{W, 0}, {W, D}, {1, 0}},
                           {{W, D}, {0, D}, {0, 1}}, {{0, D}, {0, 0}, {-1, 0}}};
    for (int ei = 0; ei < 4; ++ei) {
        const Edge& e = edges[ei];
        const Vec2 nW = s.frame.u * e.nOut.x + s.frame.v * e.nOut.y;   // world outward normal
        const Vec3 nIn3(-nW.x, 0, -nW.y);
        auto skinAt = [&](Real off, const Vec3& n, const Vec3& colr, Real ta, Real tb, Real yb, Real yt) {
            // The skin's segment from parameter ta to tb along the edge, `off` outward.
            const Vec2 pa = e.a + (e.b - e.a) * ta + e.nOut * off;
            const Vec2 pb = e.a + (e.b - e.a) * tb + e.nOut * off;
            const Vec2 wa = s.frame.toWorld(pa), wb = s.frame.toWorld(pb);
            wallQuad(out.drywall, col, Vec3(wa.x, 0, wa.y), Vec3(wb.x, 0, wb.y), yb, yt, n, colr);
        };
        const bool doorEdge = ei == 0 && s.doorWidth > 0;
        if (!doorEdge) {
            skinAt(0, nIn3, paintIn, 0, 1, y0, y0 + h);
            continue;
        }
        const Real x0 = std::max(Real(0), s.doorX - s.doorWidth * 0.5);
        const Real x1 = std::min(W, s.doorX + s.doorWidth * 0.5);
        const Real dh = std::min(s.doorHeight, h - 0.3);
        const Real len = (e.b - e.a).length();
        auto piece = [&](Real off, const Vec3& n, const Vec3& colr) {
            skinAt(off, n, colr, 0, x0 / len, y0, y0 + h);
            skinAt(off, n, colr, x1 / len, 1, y0, y0 + h);
            skinAt(off, n, colr, x0 / len, x1 / len, y0 + dh, y0 + h);   // the lintel band
        };
        piece(0, nIn3, paintIn);
        // Reveals: the two jambs and the lintel underside, between the skins.
        const Vec2 tangent = s.frame.u * (e.b - e.a).x / len + s.frame.v * (e.b - e.a).y / len;
        const Vec3 t3(tangent.x, 0, tangent.y);
        auto reveal = [&](Real x, const Vec3& n) {
            const Vec2 in = s.frame.toWorld(e.a + (e.b - e.a) * (x / len));
            const Vec2 ou = s.frame.toWorld(e.a + (e.b - e.a) * (x / len) + e.nOut * kWall);
            wallQuad(out.drywall, col, Vec3(ou.x, 0, ou.y), Vec3(in.x, 0, in.y), y0, y0 + dh, n, paintOut);
        };
        reveal(x0, t3);
        reveal(x1, t3 * -1.0);
        {
            const Vec2 a0 = s.frame.toWorld(e.a + (e.b - e.a) * (x0 / len));
            const Vec2 a1 = s.frame.toWorld(e.a + (e.b - e.a) * (x0 / len) + e.nOut * kWall);
            const Vec2 b1 = s.frame.toWorld(e.a + (e.b - e.a) * (x1 / len) + e.nOut * kWall);
            const Vec2 b0 = s.frame.toWorld(e.a + (e.b - e.a) * (x1 / len));
            const Real yl = y0 + dh;
            const Vec3 A(a0.x, yl, a0.y), B(a1.x, yl, a1.y), C(b1.x, yl, b1.y), Dd(b0.x, yl, b0.y);
            MeshBuilder::emitQuad(out.drywall, A, Dd, C, B, Vec3(0, -1, 0), paintOut);
            if (col) MeshBuilder::emitQuad(*col, A, Dd, C, B, Vec3(0, -1, 0), paintOut);
        }
    }
}

// One dog-leg storey: flight A up and away from the door on the left half,
// the half landing across the far end, flight B back to the next floor's
// landing on the right half, the spine wall between them, soffits under both.
void stairStorey(CoreMeshes& out, RenderMesh* col, const CoreStair& st, Real y0, Real h,
                 bool flightsUp, bool landing, const Vec3& floorCol, const Vec3& stairCol,
                 const Vec3& paint) {
    const CoreShaft& s = st.shaft;
    const Real fw = st.flightWidth, W = s.width, D = s.depth, Ld = st.landing;
    const Real yTop = y0 + 0.05;
    if (landing) {
        deckQuad(out.floor, col, s, 0, 0, W, Ld, yTop, true, floorCol);
        deckQuad(out.drywall, nullptr, s, 0, 0, W, Ld, yTop - kSlab, false, paint);
        // The landing's inner edge over flight A's half: the slab's cut face
        // (flight B's top riser and the spine wall's end cap already stand
        // on the rest of that line).
        const Vec3 a = s.at(0, Ld, yTop), b = s.at(fw, Ld, yTop);
        const Vec2 nv = s.frame.v;
        wallQuad(out.drywall, col, Vec3(a.x, 0, a.z), Vec3(b.x, 0, b.z), yTop - kSlab, yTop,
                 Vec3(nv.x, 0, nv.y), paint);
    }
    if (!flightsUp) {
        // The TOP storey: no flight leaves this landing, so the shaft beyond
        // it is the storey below's well — a guard wall across flight A's
        // half and the spine, floor to ceiling, keeps a walker on the landing
        // (flight B's half is the way down).
        const Vec3 a = s.at(0, Ld, 0), b = s.at(fw + st.spine, Ld, 0);
        const Vec2 nv = s.frame.v;
        wallQuad(out.drywall, col, a, b, y0, y0 + h, Vec3(nv.x, 0, nv.y), paint);
        wallQuad(out.drywall, col, a, b, y0, y0 + h, Vec3(-nv.x, 0, -nv.y), paint);
        return;
    }
    const int nR = halfFlightRisers(h);
    const Real riser = (h * 0.5) / nR;
    const Real run = nR * st.tread;
    if (Ld + run > D + 1e-6) return;   // cannot happen: the shaft is sized by the tallest storey
    const Real yMid = yTop + h * 0.5;
    // A flight: `uA..uB` across, climbing from frame v `vFoot` in direction
    // `dir` (+1 away from the door, -1 back), from height `yb`.
    auto flight = [&](Real uA, Real uB, Real vFoot, Real dir, Real yb) {
        const Vec2 nRise = s.frame.v * -dir;   // riser faces the climber
        for (int j = 0; j < nR; ++j) {
            const Real v0 = vFoot + dir * st.tread * j, v1 = vFoot + dir * st.tread * (j + 1);
            const Real yt = yb + riser * (j + 1);
            const Real lo = std::min(v0, v1), hi = std::max(v0, v1);
            deckQuad(out.stair, col, s, uA, lo, uB, hi, yt, true, stairCol);
            const Vec3 ra = s.at(uA, v0, 0), rb = s.at(uB, v0, 0);
            wallQuad(out.stair, col, ra, rb, yt - riser, yt, Vec3(nRise.x, 0, nRise.y), stairCol);
        }
        // Soffit: the sloped underside from the foot line at the deck to
        // under the last tread, facing down and back.
        const Real vHead = vFoot + dir * run;
        const Vec3 A = s.at(uA, vFoot, yb), B = s.at(uB, vFoot, yb);
        const Vec3 C = s.at(uB, vHead, yb + h * 0.5 - riser), Dd = s.at(uA, vHead, yb + h * 0.5 - riser);
        const Vec3 along = normalize(Dd - A);
        const Vec3 across = normalize(B - A);
        Vec3 n = cross(along, across);
        if (n.y > 0) n = n * -1.0;
        MeshBuilder::emitQuad(out.stair, A, Dd, C, B, n, stairCol);
        if (col) MeshBuilder::emitQuad(*col, A, Dd, C, B, n, stairCol);
    };
    flight(0, fw, Ld, 1.0, yTop);                       // A: away from the door
    // The half landing across the far end at mid height.
    deckQuad(out.floor, col, s, 0, Ld + run, W, D, yMid, true, floorCol);
    deckQuad(out.drywall, nullptr, s, 0, Ld + run, W, D, yMid - kSlab, false, paint);
    {
        const Vec3 a = s.at(0, Ld + run, yMid), b = s.at(W, Ld + run, yMid);
        const Vec2 nv = s.frame.v * -1.0;
        wallQuad(out.drywall, col, Vec3(a.x, 0, a.z), Vec3(b.x, 0, b.z), yMid - kSlab, yMid,
                 Vec3(nv.x, 0, nv.y), paint);
    }
    flight(fw + st.spine, W, Ld + run, -1.0, yMid);     // B: back toward the door
    // The spine wall between the flights, floor to ceiling, with end caps.
    // It starts a tread and a half up each flight, not at the landing line:
    // a walker hugging the spine while lining up for the first riser had its
    // step-up cast blocked by the cap and wedged there (the climb test only
    // passed by the luck of contact order until the lobby desk changed it).
    {
        const Real uS0 = fw, uS1 = fw + st.spine;
        const Real vS0 = Ld + st.tread * 1.5, vS1 = Ld + run - st.tread * 1.5;
        const Vec2 nu = s.frame.u;
        const Vec3 nL(-nu.x, 0, -nu.y), nRt(nu.x, 0, nu.y);
        const Vec3 a0 = s.at(uS0, vS0, 0), a1 = s.at(uS0, vS1, 0);
        const Vec3 b0 = s.at(uS1, vS0, 0), b1 = s.at(uS1, vS1, 0);
        wallQuad(out.drywall, col, a0, a1, y0, y0 + h, nL, paint);
        wallQuad(out.drywall, col, b0, b1, y0, y0 + h, nRt, paint);
        const Vec2 nv = s.frame.v;
        wallQuad(out.drywall, col, a0, b0, y0, y0 + h, Vec3(-nv.x, 0, -nv.y), paint);
        wallQuad(out.drywall, col, a1, b1, y0, y0 + h, Vec3(nv.x, 0, nv.y), paint);
    }
}
}  // namespace

// The core's OUTER face for one storey: one ring a wall thickness outside
// the core rectangle (the shafts fill it: stairs full depth, hoistways with
// the service block behind), facing out, the doors cut into its front wall.
// One skin for the whole core, so the slots between neighbouring shafts are
// sealed by construction.
void coreRing(CoreMeshes& out, RenderMesh* col, const CorePlan& core, Real y0, Real h, const Vec3& paint) {
    const Real w = kWall, L = core.length, D = core.depth;
    const SiteFrame& f = core.frame;
    auto W = [&](Real u, Real v) { const Vec2 p = f.toWorld({u, v}); return Vec3(p.x, 0, p.y); };
    const Vec3 nFront(-f.v.x, 0, -f.v.y), nBack(f.v.x, 0, f.v.y);
    const Vec3 nRight(f.u.x, 0, f.u.y), nLeft(-f.u.x, 0, -f.u.y);
    // The sides and the back: whole.
    wallQuad(out.drywall, col, W(L + w, -w), W(L + w, D + w), y0, y0 + h, nRight, paint);
    wallQuad(out.drywall, col, W(L + w, D + w), W(-w, D + w), y0, y0 + h, nBack, paint);
    wallQuad(out.drywall, col, W(-w, D + w), W(-w, -w), y0, y0 + h, nLeft, paint);
    // The front: full-height pieces between the doors, a lintel band over each.
    struct Hole { Real u0, u1, top; };
    std::vector<Hole> holes;
    auto door = [&](const CoreShaft& s) {
        if (s.doorWidth <= 0) return;
        const Real su = f.toFrame(s.frame.origin).x;
        holes.push_back({su + s.doorX - s.doorWidth * 0.5, su + s.doorX + s.doorWidth * 0.5,
                         std::min(s.doorHeight, h - 0.3)});
    };
    for (const CoreShaft& hw : core.hoistways) door(hw);
    for (const CoreStair& st : core.stairs) door(st.shaft);
    std::sort(holes.begin(), holes.end(), [](const Hole& a, const Hole& b) { return a.u0 < b.u0; });
    Real u = -w;
    for (const Hole& hole : holes) {
        if (hole.u0 > u + 1e-4) wallQuad(out.drywall, col, W(u, -w), W(hole.u0, -w), y0, y0 + h, nFront, paint);
        wallQuad(out.drywall, col, W(hole.u0, -w), W(hole.u1, -w), y0 + hole.top, y0 + h, nFront, paint);
        u = hole.u1;
    }
    if (L + w > u + 1e-4) wallQuad(out.drywall, col, W(u, -w), W(L + w, -w), y0, y0 + h, nFront, paint);
}

void emitCoreShaftWalls(CoreMeshes& out, RenderMesh* colliderOut, const CorePlan& core,
                        const StoreyPlan& sp, Real baseY, const BuildingParams& params) {
    if (!core.valid) return;
    const Real y0 = baseY + sp.y0, h = sp.h;
    const Vec3 paint = interiorPaintFor(params);
    const Vec3 shaftDark(0.34, 0.34, 0.36);
    coreRing(out, colliderOut, core, y0, h, paint);
    for (const CoreShaft& hw : core.hoistways) shaftWalls(out, colliderOut, core, hw, y0, h, paint, shaftDark);
    for (const CoreStair& st : core.stairs) shaftWalls(out, colliderOut, core, st.shaft, y0, h, paint, paint);
    if (core.hasService) shaftWalls(out, colliderOut, core, core.service, y0, h, paint, paint);
}

void emitCoreStorey(CoreMeshes& out, RenderMesh* colliderOut, const CorePlan& core,
                    const StoreyPlan& sp, Real baseY, const BuildingParams& params,
                    bool flightsUp, bool landing, bool walls) {
    if (!core.valid) return;
    const Real y0 = baseY + sp.y0, h = sp.h;
    const Vec3 paint = interiorPaintFor(params);
    const Real floorTone = 0.85 + 0.45 * (((params.seed >> 4) & 0xffu) / 255.0);
    const Vec3 floorCol = floorFinishFor(params).albedo * floorTone;
    const Vec3 stairCol = stairFinishFor(params).albedo * floorTone;
    // The walls: drawn here only when the exterior mesh does not already
    // carry them (a whole-building grow in a test); their colliders always.
    if (walls) {
        emitCoreShaftWalls(out, colliderOut, core, sp, baseY, params);
    } else {
        CoreMeshes scratch;
        emitCoreShaftWalls(scratch, colliderOut, core, sp, baseY, params);
    }
    for (const CoreShaft& hw : core.hoistways) {
        // The CALL BUTTON: a small dark plate beside the door at hand height,
        // proud of the outer skin, and a hall lantern plate above the door.
        auto plate = [&](Real u0, Real u1, Real yb, Real yt, const Vec3& colr) {
            const Vec2 nv = hw.frame.v * -1.0;   // out of the shaft, into the lobby
            const Real vOut = -kWall - 0.02, vIn = -kWall + 0.002;
            const Vec3 a = hw.at(u0, vOut, 0), b = hw.at(u1, vOut, 0);
            wallQuad(out.drywall, nullptr, a, b, yb, yt, Vec3(nv.x, 0, nv.y), colr);
            // Side and top faces so the plate has depth.
            const Vec2 nu = hw.frame.u;
            wallQuad(out.drywall, nullptr, hw.at(u0, vIn, 0), hw.at(u0, vOut, 0), yb, yt, Vec3(-nu.x, 0, -nu.y), colr);
            wallQuad(out.drywall, nullptr, hw.at(u1, vOut, 0), hw.at(u1, vIn, 0), yb, yt, Vec3(nu.x, 0, nu.y), colr);
            const Vec3 t0 = hw.at(u0, vIn, yt), t1 = hw.at(u1, vIn, yt), t2 = hw.at(u1, vOut, yt), t3 = hw.at(u0, vOut, yt);
            MeshBuilder::emitQuad(out.drywall, t0, t1, t2, t3, Vec3(0, 1, 0), colr);
        };
        const Real bx = hw.doorX + hw.doorWidth * 0.5 + 0.28;
        plate(bx, bx + 0.10, y0 + 1.05, y0 + 1.22, Vec3(0.28, 0.28, 0.30));               // the button plate
        plate(hw.doorX - 0.16, hw.doorX + 0.16, y0 + hw.doorHeight + 0.12, y0 + hw.doorHeight + 0.24,
              Vec3(0.30, 0.30, 0.32));                                                    // the hall lantern
    }
    for (const CoreStair& st : core.stairs)
        stairStorey(out, colliderOut, st, y0, h, flightsUp, landing, floorCol, stairCol, paint);
}

}  // namespace engine
