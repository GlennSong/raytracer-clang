#include "ladders.h"
#include "../../mesh_builder.h"
#include "room_plan.h"   // WallOpening
#include <algorithm>
#include <cmath>
#include <vector>

namespace engine {

bool roofLadderFor(const Poly2& planIn, const BuildingParams& params, Real baseY, Real roofY, LadderSpec& out) {
    if (planIn.size() < 3 || params.roofStyle != BuildingParams::RoofStyle::Flat) return false;
    if (params.floors < 1 || params.floors > 4 || params.curtainWall || params.bigBox || params.campus) return false;
    if (roofY - baseY < 3.0) return false;
    const uint32_t h = static_cast<uint32_t>(params.seed) * 2654435761u;
    if ((h >> 11) % 3u == 0) return false;   // two in three
    Poly2 plan = planIn;
    ensureCCW(plan);
    const std::size_t re = rearEdgeOf(plan, params);
    if (re >= plan.size()) return false;
    const Vec2 a = plan[re], b = plan[(re + 1) % plan.size()];
    const Real W = (b - a).length();
    if (W < 4.0) return false;
    const Vec2 d = (b - a) * (1.0 / W), nOut(d.y, -d.x);
    // CLEAR OF THE WINDOWS (Glenn's ask is a way up, not a ladder across the panes): the rear face's openings on every
    // storey, as the facade cuts them, and the ladder (its cage 0.95 m across) in the first stretch clear of them by
    // a hand's width, from the end away from the fire escape's
    std::vector<std::pair<Real, Real>> spans;
    {
        const std::vector<StoreyPlan> sps = storeyPlans(plan, params);
        const std::size_t ee = entranceEdgeFor(plan, params);
        for (std::size_t k = 0; k < sps.size(); ++k)
            for (const WallOpening& o : storeyOpenings(sps[k].plan, params, static_cast<int>(k), sps[k].y0, sps[k].h, ee)) {
                // this face's (its inner face lies the inset in from the edge line)
                if (std::fabs(dot(o.a - a, nOut) + std::max(params.wallThickness, Real(0.55))) > 0.4) continue;   // (interiorInset)
                const Real t0 = dot(o.a - a, d), t1 = dot(o.b - a, d);
                spans.push_back({std::min(t0, t1), std::max(t0, t1)});
            }
    }
    const Real half = 0.48 + 0.10;
    Real x = -1;
    for (int i = 0; i < 400 && x < 0; ++i) {
        const Real s = 0.8 + i * 0.1;
        if (s > W - 0.8) break;
        const Real cand = params.fireEscape ? W - s : s;
        bool clear = true;
        for (const auto& sp : spans) clear = clear && (cand + half <= sp.first || cand - half >= sp.second);
        if (clear) x = cand;
    }
    if (x < 0) return false;   // a rear face that is all windows: no ladder up it
    const Vec2 at = a + d * x + nOut * 0.20;   // the rails stand off the wall on their brackets
    out.foot = Vec3(at.x, baseY, at.y);
    out.out = nOut;
    out.width = 0.46;
    out.landY = roofY;
    out.topY = roofY + 1.1;   // over the parapet: a hand-hold to step off by
    const Vec2 ex = a + d * x - nOut * 1.0;
    out.exit = Vec3(ex.x, roofY, ex.y);
    out.cage = roofY - baseY > 4.5;
    return true;
}

namespace {
// A box from a to b (its long axis), `w` thick across, `t` thick up -- for rails, rungs and hoops.
void bar(RenderMesh& m, const Vec3& a, const Vec3& b, Real w, Real t, const Vec3& col) {
    const Vec3 ax = b - a;
    const Real L = ax.length();
    if (L < 1e-6) return;
    const Vec3 u = ax * (1.0 / L);
    Vec3 side = std::fabs(u.y) < 0.9 ? normalize(cross(u, Vec3(0, 1, 0))) : Vec3(1, 0, 0);
    const Vec3 up = normalize(cross(side, u));
    side = side * (w * 0.5);
    const Vec3 upv = up * (t * 0.5);
    const Vec3 c[8] = {a - side - upv, a + side - upv, a + side + upv, a - side + upv,
                       b - side - upv, b + side - upv, b + side + upv, b - side + upv};
    const int f[6][4] = {{0, 1, 2, 3}, {5, 4, 7, 6}, {4, 0, 3, 7}, {1, 5, 6, 2}, {3, 2, 6, 7}, {4, 5, 1, 0}};
    for (const auto& q : f) {
        const Vec3 n = normalize(cross(c[q[1]] - c[q[0]], c[q[2]] - c[q[0]]));
        MeshBuilder::emitQuad(m, c[q[0]], c[q[1]], c[q[2]], c[q[3]], n, col);
    }
}
}  // namespace

RenderMesh ladderMesh(const LadderSpec& l, const Vec3& col) {
    RenderMesh m;
    const Vec3 out(l.out.x, 0, l.out.y);
    const Vec3 along = normalize(cross(Vec3(0, 1, 0), out));   // across the ladder
    const Real hw = l.width * 0.5;
    const Vec3 base(l.foot.x, l.foot.y, l.foot.z);
    for (Real s : {-hw, hw}) {
        const Vec3 r0 = base + along * s, r1 = Vec3(r0.x, l.topY, r0.z);
        bar(m, r0, r1, 0.06, 0.06, col);                                           // the rail
        // over the parapet: the rail bent back onto the roof (a gooseneck), down to it
        const Vec3 back = r1 - out * 0.9;
        bar(m, r1, back, 0.06, 0.06, col);
        bar(m, back, Vec3(back.x, l.landY, back.z), 0.06, 0.06, col);
        for (Real y = l.foot.y + 1.5; y < l.landY - 0.3; y += 2.0)                // brackets to the wall
            bar(m, Vec3(r0.x, y, r0.z), Vec3(r0.x, y, r0.z) - out * 0.20, 0.05, 0.05, col);
    }
    for (Real y = l.foot.y + 0.3; y < l.landY + 0.05; y += 0.3)                     // the rungs
        bar(m, base + along * -hw + Vec3(0, y - l.foot.y, 0), base + along * hw + Vec3(0, y - l.foot.y, 0), 0.035, 0.035, col);
    if (l.cage) {
        // HOOPS from 2.4 m up to the parapet, every 0.9 m: two sides out from the rails and a back across them; three
        // straps run up the hoops
        const Real r = 0.70;
        for (Real y = l.foot.y + 2.4; y < l.topY; y += 0.9) {
            const Vec3 p0 = base + Vec3(0, y - l.foot.y, 0);
            const Vec3 aL = p0 + along * -(hw + 0.12), aR = p0 + along * (hw + 0.12);
            bar(m, aL, aL + out * r, 0.05, 0.02, col);
            bar(m, aR, aR + out * r, 0.05, 0.02, col);
            bar(m, aL + out * r, aR + out * r, 0.05, 0.02, col);
        }
        for (Real s : {-(hw + 0.12), Real(0), hw + 0.12}) {
            const Vec3 p = base + along * s + out * (s == 0 ? r : r * 0.6);
            bar(m, p + Vec3(0, 2.4, 0), Vec3(p.x, l.topY, p.z), 0.05, 0.02, col);
        }
    }
    return m;
}

}  // namespace engine
