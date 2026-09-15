#include "room_plan.h"
#include "../../mesh_builder.h"
#include <algorithm>
#include <cmath>

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

RoomPlan roomPlan(const Poly2& planIn, const BuildingParams& params, const CorePlan& core,
                  std::size_t blankEdge, Real inset, int storey) {
    RoomPlan rp;
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
            r.rect = {P(ra, dIn), P(rb, dIn), P(rb, dOut), P(ra, dOut)};
            rp.rooms.push_back(r);
            // The front, with its door a third of the way along (alternating).
            RoomWall front;
            front.a = P(ra, dOut);
            front.b = P(rb, dOut);
            front.doorAt = (rp.rooms.size() % 2) ? 0.35 : 0.65;
            front.glass = rp.office;
            rp.walls.push_back(front);
            // The partition to the next room.
            if (i + 2 < cuts.size()) {
                RoomWall side;
                side.a = P(rb, dIn);
                side.b = P(rb, dOut);
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
}

}  // namespace

void emitRooms(RoomMeshes& out, RenderMesh* colliderOut, const RoomPlan& rp, Real y0, Real h,
               const Vec3& paint) {
    for (const RoomWall& w : rp.walls) {
        if (w.glass) glassRun(out.glass, colliderOut, w.a, w.b, y0, h, w.doorAt);
        else wallRun(out.drywall, colliderOut, w.a, w.b, y0, h, kRoomWallT, w.doorAt, paint);
    }
}

}  // namespace engine
