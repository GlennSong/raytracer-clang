#include "corridor_mesh.h"
#include "../../../log.h"

#include "../../mesh_builder.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

// One cross-section rib: world positions across the deck at station s.
// Offsets are SIGNED (positive left of travel); y adds superelevation tilt.
struct Rib {
    Vec2 c;        // centreline (world XZ)
    Vec2 n;        // left normal
    Real z;        // deck elevation at the centreline
    Real slope;    // superelevation cross-slope (dy per metre of offset)
    Vec3 at(Real off) const {
        const Vec2 p = c + n * off;
        return Vec3(p.x, z + slope * off, p.y);
    }
};

// §11: the gore band adapts to the room available — a street target close
// to the corridor leaves the free clothoid enough length to make its turn
// (a fixed 90 m band starved it into a folded needle).
Real goreBandLen(const CorridorDef& c, const ExitDef& e, Real L) {
    const Real sg = std::max(Real(1), std::min(e.station, L - 1));
    const Real d = (e.target - c.horizontal.pos(sg)).length();
    return std::min(Real(90), std::max(Real(40), d * 0.35));
}

}  // namespace

CorridorAuthoring corridorAuthor(const CorridorDef& cIn,
                                 const std::function<Real(Real, Real)>& ground,
                                 Real step) {
    // A corridor's whole non-geometry contribution (one-mesher P8b). This was
    // extracted from the old sweep mesher, whose interleaved deck/ramp/pier/marking
    // emission is now gone — weldSolid draws all of it. The split was possible
    // because the gore band's centreline re-samples the alignment; it never read
    // the deck's rib array, despite a comment that claimed it did.
    CorridorAuthoring out;
    CorridorDef c = cIn;
    // Each exit grows its deceleration AUX LANE into the schedule — the deck
    // flares one lane on that side over the decel length, ending at the gore.
    // MUST run before any halfWidthAt: the flare feeds innerOff/freeHalf/corrGuard.
    for (const ExitDef& e : c.exits) {
        if (e.station < 0) continue;   // dropped at resolution
        const Real La = c.horizontal.length();
        if (e.onRamp)   // accel: full at the merge, closes GRADUALLY (90 m)
            c.lanes.aux.push_back({e.station,
                                   std::min(La - 30.0, e.station + e.decelLength),
                                   e.upStation, false, 90.0});
        else            // decel: opens over 40 m, full at the gore
            c.lanes.aux.push_back({std::max(Real(30), e.station - e.decelLength),
                                   e.station, e.upStation, true, 40.0});
    }
    const Real L = c.horizontal.length();
    if (L < step * 2) return out;
    auto gy = [&](const Vec2& p) { return ground ? ground(p.x, p.y) : Real(0); };

    const int n = std::max(2, static_cast<int>(std::ceil(L / step)));
    std::vector<Real> halfN(n + 1), halfP(n + 1);   // per-side deck reach (corrGuard)
    std::vector<bool> elevated(n + 1);              // tall enough to fly: no carve
    for (int i = 0; i <= n; ++i) {
        const Real s = L * i / n;
        const Vec2 pc = c.horizontal.pos(s);
        halfN[i] = c.halfWidthAt(s, -1);
        halfP[i] = c.halfWidthAt(s, 1);
        elevated[i] = c.vertical.elevation(s) - gy(pc) > 2.0;
    }

    // AT-GRADE flatten: windows of the deck footprint carve the terrain to
    // the deck plane. The gore BAND (§11) widens the footprint while it runs.
    auto bandExtra = [&](Real s2, int sideSign) {
        Real ex = 0;
        for (const ExitDef& e2 : c.exits) {
            if ((e2.upStation ? -1 : 1) != sideSign) continue;
            const Real dsf = e2.upStation ? 1.0 : -1.0;
            const Real sgx = std::max(Real(1), std::min(e2.station, L - 1.0));
            const Real Lg2 = goreBandLen(c, e2, L);
            const Real s0b = e2.onRamp ? sgx - dsf * Lg2 : sgx;
            const Real s1b = e2.onRamp ? sgx : sgx + dsf * Lg2;
            if (s2 >= std::min(s0b, s1b) && s2 <= std::max(s0b, s1b))
                ex = std::max(ex, Real(5.8 + 1.5));
        }
        return ex;
    };
    {
        const Real win = 4.0;
        Real s0 = 0;
        while (s0 < L) {
            const Real s1 = std::min(L, s0 + win);
            const Real sm = (s0 + s1) * 0.5;
            const int i0 = static_cast<int>(s0 / L * n);
            if (!elevated[std::min(i0, n)]) {
                const Vec2 p0 = c.horizontal.pos(s0), p1 = c.horizontal.pos(s1);
                const Vec2 n0 = c.horizontal.normal(s0), n1 = c.horizontal.normal(s1);
                const Real hp0 = c.halfWidthAt(s0, 1) + 2.0 + bandExtra(s0, 1);
                const Real hp1 = c.halfWidthAt(s1, 1) + 2.0 + bandExtra(s1, 1);
                const Real hn0 = c.halfWidthAt(s0, -1) + 2.0 + bandExtra(s0, -1);
                const Real hn1 = c.halfWidthAt(s1, -1) + 2.0 + bandExtra(s1, -1);
                std::vector<Vec3> poly{
                    Vec3(p0.x + n0.x * hp0, 0, p0.y + n0.y * hp0),
                    Vec3(p1.x + n1.x * hp1, 0, p1.y + n1.y * hp1),
                    Vec3(p1.x - n1.x * hn1, 0, p1.y - n1.y * hn1),
                    Vec3(p0.x - n0.x * hn0, 0, p0.y - n0.y * hn0)};
                out.flatten.push_back(
                    makeFlattenPad(std::move(poly), c.vertical.elevation(sm) - 0.18, 6.0));
            }
            s0 = s1;
        }
    }

    // Mainline samples for the never-under-the-deck check (§11), each with
    // its LOCAL side-aware deck reach. The old guard took the GLOBAL max
    // half-width — one exit's accel/decel flare anywhere on the corridor
    // inflated the guard everywhere, and a second ramp's legitimately
    // parallel descent then read as "crossing the mainline" and was DROPPED
    // (an exit + on-ramp pair could never coexist).
    struct CorrPt { Vec2 p; Real guardN, guardP; };
    std::vector<CorrPt> corrPts;
    for (Real s2 = 0; s2 <= L; s2 += 25.0) {
        const int i = std::min(n, static_cast<int>(s2 / L * n));
        corrPts.push_back({ c.horizontal.pos(s2),
                            halfN[i] + Real(1.5), halfP[i] + Real(1.5) });
    }
    for (const ExitDef& e : c.exits) {
        out.rampPaths.emplace_back();                    // parallel to exits
        std::vector<Vec3>& path = out.rampPaths.back().pts;
        if (e.station < 0) continue;   // dropped at resolution (no street)
        const Real sg = std::max(Real(1), std::min(e.station, L - 1.0));
        const int dirSign = e.upStation ? -1 : 1;        // offset side
        const Real ds = e.upStation ? 1.0 : -1.0;        // flow over stations
        // ---- GORE BAND: the ramp's first piece rides the deck edge. Its
        // centreline re-samples the alignment at the band's own stations.
        const Real Lg = goreBandLen(c, e, L);
        const Real rampW = 7.2;   // AASHTO 1-lane ramp: 12 ft lane + 4/8 ft shoulders
        const int nb = 30;
        const Real sBand0 = e.onRamp ? sg - ds * Lg : sg;   // flow-first end
        std::vector<Vec3> bandPath;
        for (int i = 0; i <= nb; ++i) {
            const Real t = static_cast<Real>(i) / nb;
            const Real si = std::min(L - 1.0, std::max(Real(1.0),
                                                        sBand0 + ds * t * Lg));
            const Real tw = e.onRamp
                ? rampW + (c.laneWidth - rampW) * t     // arrive -> accel lane
                : c.laneWidth + (rampW - c.laneWidth) * t;   // aux -> full ramp
            Rib rb;
            rb.c = c.horizontal.pos(si);
            rb.n = c.horizontal.normal(si);
            rb.z = c.vertical.elevation(si);
            rb.slope = c.superelevationAt(si);
            const Real innerOff = dirSign * c.halfWidthAt(si, dirSign);
            bandPath.push_back(rb.at(innerOff + dirSign * tw * 0.5));
        }
        // ---- FREE SECTION: starts EXACTLY where the band ends, clothoids to
        // the street. Never under the mainline: validated below.
        const Real sFree = e.onRamp ? sBand0 : sg + ds * Lg;
        const Real freeHalf = c.halfWidthAt(sFree, dirSign);
        const Vec2 C0 = c.horizontal.offset(
            sFree, dirSign * (freeHalf + rampW * 0.5));
        const Vec2 away = c.horizontal.tangent(sFree) * ds *
                          (e.onRamp ? -1.0 : 1.0);
        Alignment ra = Alignment::fromPolyline(
            {C0, C0 + away * 30.0, e.target}, e.rampRadius, e.rampSpiral, 2.0);
        bool folded = false;
        Real totalTurn = 0;
        if (!ra.empty()) {
            const Real rl2 = ra.length();
            Vec2 tPrev = ra.tangent(0);
            for (Real s2 = 6.0; s2 <= rl2; s2 += 6.0) {
                const Vec2 tc = ra.tangent(std::min(s2, rl2 - 0.5));
                if (dot(tPrev, tc) < 0.05) { folded = true; break; }
                totalTurn += std::acos(std::max(Real(-1), std::min(Real(1), dot(tPrev, tc))));
                tPrev = tc;
            }
            if (rl2 > 2.4 * (e.target - C0).length()) folded = true;
            // A ramp that curls more than ~100 deg total is a HOOK.
            if (totalTurn > 1.75) folded = true;
        }
        if (ra.empty() || ra.length() < 48.0 || folded) {
            out.rampPaths.back().pts.clear();
            continue;
        }
        const Real RL = std::max(Real(40), ra.length() - e.landingSetback);
        // Grade change happens NEAR THE DECK: drop/climb over the first ~45%.
        VerticalProfile rp;
        const Real zDeck = c.vertical.elevation(sFree) +
                           c.superelevationAt(sFree) * dirSign *
                               (freeHalf + rampW * 0.5);
        const Real sKnee = std::max(Real(80), RL * 0.45);
        if (sKnee < RL - 10.0)
            rp.pvis = {{0, zDeck, 0},
                       {sKnee, e.targetY, std::min(sKnee, Real(80))},
                       {RL, e.targetY, 0}};
        else
            rp.pvis = {{0, zDeck, 0}, {RL, e.targetY, std::min(RL * 0.5, Real(90))}};
        const Real rw = 3.6;                       // half-width (7.2 m ramp)
        const int rn = std::max(2, static_cast<int>(RL / step));
        std::vector<Rib> rr(rn + 1);
        std::vector<bool> rUp(rn + 1);
        for (int i = 0; i <= rn; ++i) {
            const Real s = RL * i / rn;
            rr[i].c = ra.pos(s);
            rr[i].n = ra.normal(s);
            rr[i].z = rp.elevation(s);
            rr[i].slope = 0;
            rUp[i] = rr[i].z - gy(rr[i].c) > 0.35;   // structure from 35 cm up
        }
        {   // §11 guard: the free run must stay OUT of the mainline footprint
            // — measured against the LOCAL reach on the ramp's own side.
            bool crosses = false;
            for (int i = 0; i <= rn && !crosses; ++i) {
                const Real rs2 = RL * i / rn;
                if (rs2 < 35.0) continue;        // the legit peel-off hugs the deck
                for (const CorrPt& q : corrPts) {
                    const Real g = dirSign < 0 ? q.guardN : q.guardP;
                    if ((q.p - rr[i].c).length() < g) { crosses = true; break; }
                }
            }
            if (crosses) {
                out.rampPaths.back().pts.clear();
                continue;
            }
        }
        std::vector<Vec3> freePath;
        for (int i = 0; i <= rn; ++i)
            freePath.push_back(Vec3(rr[i].c.x, rr[i].z, rr[i].c.y));
        if (e.onRamp) {   // flow order: street -> arrival -> merge
            path.assign(freePath.rbegin(), freePath.rend());
            path.insert(path.end(), bandPath.begin() + 1, bandPath.end());
            out.rampPaths.back().bandBack = static_cast<int>(bandPath.size()) - 1;
        } else {          // flow order: gore -> band end -> street
            path = bandPath;
            path.insert(path.end(), freePath.begin() + 1, freePath.end());
            out.rampPaths.back().bandFront = static_cast<int>(bandPath.size());
        }
        // LANDING conform: flatten only the last stretch meeting street grade.
        for (Real s0 = 0; s0 < RL; s0 += 10.0) {
            const Real s1 = std::min(RL, s0 + 12.0);   // 2 m overlap
            {   // only carve where the ramp is AT GRADE (structure spans fly)
                const int i2 = std::min(rn, static_cast<int>(s0 / RL * rn));
                if (rUp[i2]) continue;
            }
            const Vec2 p0 = ra.pos(s0), p1 = ra.pos(s1);
            const Vec2 n0 = ra.normal(s0), n1 = ra.normal(s1);
            const Real hw2 = rw + 2.4;
            std::vector<Vec3> poly{
                Vec3(p0.x + n0.x * hw2, 0, p0.y + n0.y * hw2),
                Vec3(p1.x + n1.x * hw2, 0, p1.y + n1.y * hw2),
                Vec3(p1.x - n1.x * hw2, 0, p1.y - n1.y * hw2),
                Vec3(p0.x - n0.x * hw2, 0, p0.y - n0.y * hw2)};
            out.flatten.push_back(
                makeFlattenPad(std::move(poly), rp.elevation((s0 + s1) * 0.5), 5.0));
        }
    }
    return out;
}

}  // namespace engine
