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

std::vector<UnionSpine> corridorDeckSpines(
    const CorridorDef& cIn, const std::function<Real(Real, Real)>& /*ground*/, Real step) {
    std::vector<UnionSpine> spines;
    CorridorDef c = cIn;
    // Synthesise the exits' AUX lanes into the schedule (the SAME rule
    // corridorAuthor uses) so `halfWidthAt` flares one lane at each gore.
    for (const ExitDef& e : c.exits) {
        if (e.station < 0) continue;
        const Real La = c.horizontal.length();
        if (e.onRamp)
            c.lanes.aux.push_back({e.station, std::min(La - 30.0, e.station + e.decelLength),
                                   e.upStation, false, 90.0});
        else
            c.lanes.aux.push_back({std::max(Real(30), e.station - e.decelLength),
                                   e.station, e.upStation, true, 40.0});
    }
    const Real L = c.horizontal.length();
    if (L < step * 2) return spines;
    const int n = std::max(2, static_cast<int>(std::ceil(L / step)));
    // ONE deck spine down the alignment centreline: per-point absolute Y from the
    // vertical profile (the same `vertical.elevation(s)` the corridor deck rides)
    // and PER-POINT symmetric half-width (P5 — the deck now flares at aux/gore
    // spans), class Freeway so the welder gives it barriers + freeway markings.
    UnionSpine deck;
    deck.klass = RoadClass::Freeway;
    deck.points.reserve(n + 1);
    deck.yAbs.reserve(n + 1);
    deck.hw.reserve(n + 1);
    deck.crossSlope.reserve(n + 1);
    Real maxHw = 0.0;
    for (int i = 0; i <= n; ++i) {
        const Real s = L * i / n;
        const Real hw = 0.5 * (c.halfWidthAt(s, -1) + c.halfWidthAt(s, 1));
        deck.points.push_back(c.horizontal.pos(s));
        deck.yAbs.push_back(c.vertical.elevation(s));
        deck.hw.push_back(hw);
        deck.crossSlope.push_back(c.superelevationAt(s));   // bank through the curves (P6)
        maxHw = std::max(maxHw, hw);
    }
    deck.halfWidth = maxHw;   // scalar representative (mouth / barriers / piers)
    spines.push_back(std::move(deck));
    return spines;
}

std::vector<UnionSpine> corridorRampSpines(
    const std::vector<RampPath>& rampPaths, Real halfWidth) {
    std::vector<UnionSpine> spines;
    for (const RampPath& rp : rampPaths) {
        if (rp.pts.size() < 2) continue;              // dropped ramp: no spine
        UnionSpine s;
        s.klass = RoadClass::Ramp;
        s.halfWidth = halfWidth;
        s.points.reserve(rp.pts.size());
        s.yAbs.reserve(rp.pts.size());
        for (const Vec3& p : rp.pts) {
            s.points.push_back(Vec2(p.x, p.z));
            s.yAbs.push_back(p.y);                     // absolute deck/street Y — grade-sep splits it
        }
        spines.push_back(std::move(s));
    }
    return spines;
}

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

    // Mainline samples for the never-under-the-deck check (§11).
    std::vector<Vec2> corrPts;
    for (Real s2 = 0; s2 <= L; s2 += 25.0) corrPts.push_back(c.horizontal.pos(s2));
    Real corrGuard = 0;
    for (int i = 0; i <= n; ++i)
        corrGuard = std::max(corrGuard, std::max(halfN[i], halfP[i]));
    corrGuard += 1.5;
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
        {   // §11 guard: the free run must stay OUT of the mainline footprint.
            bool crosses = false;
            for (int i = 0; i <= rn && !crosses; ++i) {
                const Real rs2 = RL * i / rn;
                if (rs2 < 35.0) continue;        // the legit peel-off hugs the deck
                for (const Vec2& q : corrPts)
                    if ((q - rr[i].c).length() < corrGuard) { crosses = true; break; }
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
        } else {          // flow order: gore -> band end -> street
            path = bandPath;
            path.insert(path.end(), freePath.begin() + 1, freePath.end());
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

RenderMesh corridorFurniture(const CorridorDef& c) {
    // SIGNAGE (device: "signage above the freeway ... rounded corner placards
    // (green)"): a gantry ~45 m before each exit gore — two posts, a beam across
    // the deck, a wide green board over the through lanes (blank for now) and a
    // smaller one over the exit lane wearing a white drop-arrow for directionality.
    // Fed by the CorridorDef alone so the ONE welder's deck can wear the same
    // signage; the deck's parapets/median/piers are already built by weldSolid.
    RenderMesh out;
    const Real L = c.horizontal.length();
    for (const ExitDef& e : c.exits) {
        if (e.station < 0 || e.onRamp) continue;   // exits with a street only (no on-ramps)
        const Real sg = std::max(Real(1), std::min(e.station, L - 1.0));
        const int dirSign = e.upStation ? -1 : 1;   // offset side
        const Real ds = e.upStation ? 1.0 : -1.0;   // flow over stations
        const Vec2 travel = c.horizontal.tangent(sg) * ds;
        const Real ss = std::max(Real(10), sg - 45.0);
        const Vec2 gc = c.horizontal.pos(ss);
        const Vec2 gn = c.horizontal.normal(ss);
        const Real gz2 = c.vertical.elevation(ss);
        const Real hL = c.halfWidthAt(ss, 1) + 0.6;
        const Real hR = c.halfWidthAt(ss, -1) + 0.6;
        // Derive the beam height from the REQUIRED headroom, not a guess: the
        // sign board (2.2 m tall) hangs 0.28 m below the beam, so the old
        // beamY = gz2 + 6.2 put the board bottom at gz2 + 3.72 m — below the
        // MUTCD overhead-sign minimum (17 ft / 5.18 m) and the 5 m this
        // engine already enforces for streets passing under a deck
        // (level_loader.cpp kMinUnderClear), so a vehicle hit it. Pin the
        // board bottom to 5.4 m (18 ft) of clearance over the deck.
        const Real kSignBottomClearance = 5.4;   // MUTCD 17 ft min + margin
        const Real kBoardHeight = 2.2, kBoardHang = 0.28;
        const Real beamY = gz2 + kSignBottomClearance + kBoardHang + kBoardHeight;
        const Vec3 kGrey(0.45, 0.46, 0.48);
        auto postAt = [&](Real off) {
            const Vec2 p2 = gc + gn * off;
            RenderMesh post = MeshBuilder::cylinder(0.24f, static_cast<float>(beamY - gz2 + 0.6), 8);
            for (Vertex& v : post.vertices) v.color = kGrey;
            MeshBuilder::transform(post, Mat4::translate(p2.x, gz2 + (beamY - gz2 + 0.6) * 0.5, p2.y));
            MeshBuilder::append(out, post);
        };
        postAt(hL);
        postAt(-hR);
        RenderMesh beam = MeshBuilder::box(Vec3(hL + hR, 0.75, 0.75));
        for (Vertex& v : beam.vertices) v.color = kGrey;
        const Real byaw = std::atan2(-gn.y, gn.x);
        MeshBuilder::transform(beam,
            Mat4::trs(Vec3(gc.x + gn.x * (hL - hR) * 0.5, beamY,
                           gc.y + gn.y * (hL - hR) * 0.5),
                      Quat::fromAxisAngle(Vec3(0, 1, 0), byaw), Vec3(1, 1, 1)));
        MeshBuilder::append(out, beam);
        // a rounded-corner board facing approaching traffic
        const Vec2 face2 = travel * -1.0;
        auto board = [&](Real off, Real w, Real h, bool arrow) {
            const Vec3 kGreen(0.03, 0.28, 0.12);
            const Vec2 bc2 = gc + gn * off;
            const Vec3 bc(bc2.x, beamY - 0.28 - h * 0.5, bc2.y);
            const Vec3 right(gn.x, 0, gn.y);
            const Vec3 up3(0, 1, 0);
            const Vec3 nrm(face2.x, 0, face2.y);
            // rounded rect ring (radius r at the corners)
            const Real r = 0.35;
            std::vector<Vec3> ring;
            auto corner = [&](Real cx, Real cy, Real a0) {
                for (int k = 0; k <= 4; ++k) {
                    const Real a = a0 + k * (3.14159265 * 0.5) / 4;
                    ring.push_back(bc + right * (cx + r * std::cos(a)) +
                                   up3 * (cy + r * std::sin(a)));
                }
            };
            corner(w * 0.5 - r, h * 0.5 - r, 0);
            corner(-(w * 0.5 - r), h * 0.5 - r, 3.14159265 * 0.5);
            corner(-(w * 0.5 - r), -(h * 0.5 - r), 3.14159265);
            corner(w * 0.5 - r, -(h * 0.5 - r), 3.14159265 * 1.5);
            RenderMesh bm;
            for (std::size_t k = 1; k + 1 < ring.size(); ++k)
                MeshBuilder::emitTri(bm, ring[0] + nrm * 0.06,
                                     ring[k] + nrm * 0.06,
                                     ring[k + 1] + nrm * 0.06, nrm, kGreen);
            for (std::size_t k = 1; k + 1 < ring.size(); ++k)
                MeshBuilder::emitTri(bm, ring[0], ring[k + 1], ring[k],
                                     nrm * -1.0, kGreen * 0.55);
            if (arrow) {   // white drop-arrow: "this lane leaves"
                MeshBuilder::emitTri(bm, bc + nrm * 0.09 + up3 * 0.55,
                                     bc + nrm * 0.09 + right * 0.4 - up3 * 0.25,
                                     bc + nrm * 0.09 - right * 0.4 - up3 * 0.25,
                                     nrm, Vec3(0.9, 0.9, 0.9));
            }
            // mounting struts tie the placard to the beam (device:
            // "the signs just float with the overhanging pipe")
            for (Real sx : {-w * 0.32, w * 0.32}) {
                RenderMesh strut = MeshBuilder::box(Vec3(0.12, 1.0, 0.12));
                for (Vertex& v : strut.vertices)
                    v.color = Vec3(0.45, 0.46, 0.48);
                const Vec3 sp = bc + right * sx + up3 * (h * 0.5 + 0.2);
                MeshBuilder::transform(strut,
                                       Mat4::translate(sp.x, sp.y, sp.z));
                MeshBuilder::append(out, strut);
            }
            MeshBuilder::append(out, bm);
        };
        const int dsn = dirSign;
        board(dsn * (c.medianWidth * 0.5 + c.shoulderIn +
                     c.lanes.throughLanes * 0.5 * c.laneWidth),
              7.5, 2.2, false);
        board(dsn * (c.halfWidthAt(ss, dsn) - c.shoulderOut -
                     c.laneWidth * 0.5),
              3.4, 2.2, true);
    }
    return out;
}


}  // namespace engine
