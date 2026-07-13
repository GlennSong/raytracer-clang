#include "corridor_mesh.h"
#include "../../../log.h"

#include "../../mesh_builder.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

const Vec3 kAsphalt(0.085, 0.085, 0.095);
const Vec3 kConcrete(0.58, 0.57, 0.55);
const Vec3 kWhite(0.85, 0.85, 0.85);
const Vec3 kYellow(0.75, 0.62, 0.12);

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

CorridorMeshOut buildCorridorMesh(
    const CorridorDef& cIn, const std::function<Real(Real, Real)>& ground,
    Real step, const RoadGraph* avoidRoads) {
    CorridorMeshOut out;
    CorridorDef c = cIn;
    // Each exit grows its deceleration AUX LANE into the schedule — the deck
    // flares one lane on that side over the decel length, ending at the gore.
    for (const ExitDef& e : c.exits) {
        if (e.station < 0) continue;   // dropped at resolution
        // spans CLAMP inside the corridor: a decel lane that opened at the
        // corridor's tip read as a peel-off to nowhere (device)
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
    std::vector<Rib> ribs(n + 1);
    std::vector<Real> halfN(n + 1), halfP(n + 1);   // per-side deck reach
    std::vector<bool> elevated(n + 1);   // tall enough for pier bents
    std::vector<bool> raised(n + 1);     // off the ground at all: structure box
    const Real deckDepth = 1.9;            // deep box girder (device: a real slab)
    for (int i = 0; i <= n; ++i) {
        const Real s = L * i / n;
        Rib& r = ribs[i];
        r.c = c.horizontal.pos(s);
        r.n = c.horizontal.normal(s);
        r.z = c.vertical.elevation(s);
        r.slope = c.superelevationAt(s);
        halfN[i] = c.halfWidthAt(s, -1);
        halfP[i] = c.halfWidthAt(s, 1);
        elevated[i] = r.z - gy(r.c) > 2.0;
        raised[i] = r.z - gy(r.c) > 0.9;
    }

    // A long quad-strip between two lateral offset FUNCTIONS, top face.
    auto strip = [&](RenderMesh& m, auto offA, auto offB, const Vec3& col,
                     Real lift) {
        for (int i = 0; i < n; ++i) {
            const Vec3 a0 = ribs[i].at(offA(i)) + Vec3(0, lift, 0);
            const Vec3 b0 = ribs[i].at(offB(i)) + Vec3(0, lift, 0);
            const Vec3 a1 = ribs[i + 1].at(offA(i + 1)) + Vec3(0, lift, 0);
            const Vec3 b1 = ribs[i + 1].at(offB(i + 1)) + Vec3(0, lift, 0);
            MeshBuilder::emitQuad(m, a0, b0, b1, a1, Vec3(0, 1, 0), col);
        }
    };

    // DECK: one band, full width (median included — the barrier sits on it).
    strip(out.deck, [&](int i) { return halfP[i]; },
          [&](int i) { return -halfN[i]; }, kAsphalt, 0.0);

    // LANE MARKINGS as geometry strips floated 15mm over the deck — honest
    // lane count now; the shader can take over later (task 13). Per
    // direction (right-hand traffic: up-station drives on NEGATIVE offsets,
    // down-station on positive):
    const Real lw = 0.12;                                   // line width
    auto solidLine = [&](auto offAt, const Vec3& col) {
        strip(out.markings, [&](int i) { return offAt(i) + lw * 0.5; },
              [&](int i) { return offAt(i) - lw * 0.5; }, col, 0.015);
    };
    for (int side = -1; side <= 1; side += 2) {
        // inner (median-side) yellow edge line
        auto medEdge = [&, side](int) {
            return side * (c.medianWidth * 0.5 + c.shoulderIn * 0.4);
        };
        solidLine(medEdge, kYellow);
        // outer white edge line: inside the outer shoulder
        auto outEdge = [&, side](int i) {
            return side * ((side < 0 ? halfN[i] : halfP[i]) - c.shoulderOut);
        };
        solidLine(outEdge, kWhite);
        // dashed separators between lanes (lane count can vary by station —
        // draw up to the max and let per-station lane count gate each dash)
        int maxLanes = c.lanes.throughLanes + 1;
        for (int l = 1; l < maxLanes; ++l) {
            auto sep = [&, side, l](int i) {
                return side * (c.medianWidth * 0.5 + c.shoulderIn +
                               l * c.laneWidth);
            };
            // gate: only draw where this separator is interior to the deck
            for (int i = 0; i < n; ++i) {
                const Real s = L * i / n;
                if (l >= c.lanes.lanesAt(s, side < 0)) continue;
                if (std::fmod(s, 12.0) > 3.0) continue;
                const Vec3 a0 = ribs[i].at(sep(i) + lw * 0.5) + Vec3(0, 0.015, 0);
                const Vec3 b0 = ribs[i].at(sep(i) - lw * 0.5) + Vec3(0, 0.015, 0);
                const Vec3 a1 = ribs[i + 1].at(sep(i + 1) + lw * 0.5) + Vec3(0, 0.015, 0);
                const Vec3 b1 = ribs[i + 1].at(sep(i + 1) - lw * 0.5) + Vec3(0, 0.015, 0);
                MeshBuilder::emitQuad(out.markings, a0, b0, b1, a1,
                                      Vec3(0, 1, 0), kWhite);
            }
        }
    }

    // MEDIAN BARRIER: a low continuous Jersey-profile box down the centre.
    {
        const Real bw = std::min(c.medianWidth * 0.5, Real(0.45));
        const Real bh = 0.85;
        auto face = [&](Real oA, Real oB, Real yA, Real yB) {
            for (int i = 0; i < n; ++i) {
                const Vec3 a0 = ribs[i].at(oA) + Vec3(0, yA, 0);
                const Vec3 b0 = ribs[i].at(oB) + Vec3(0, yB, 0);
                const Vec3 a1 = ribs[i + 1].at(oA) + Vec3(0, yA, 0);
                const Vec3 b1 = ribs[i + 1].at(oB) + Vec3(0, yB, 0);
                Vec3 nrm = normalize(cross(b0 - a0, a1 - a0));
                if (nrm.y < -0.9) nrm = nrm * -1;
                MeshBuilder::emitQuad(out.barrier, a0, b0, b1, a1, nrm, kConcrete);
            }
        };
        face(-bw, -bw, 0, bh);     // right wall (drawn as vertical band)
        face(bw, bw, bh, 0);       // left wall
        face(-bw, bw, bh, bh);     // cap
    }

    // EDGES to the world: at grade, skirts drop into the (flattened) ground;
    // GIRDER (device: "supported by a concrete thing under it ... those
    // slabs are connected to the pylons and braces and the freeway sits on
    // top"): the SUPERSTRUCTURE — a narrower, deeper concrete band riding
    // under the deck slab, carried by the bent caps.
    for (int i = 0; i < n; ++i) {
        if (!elevated[i] && !elevated[i + 1]) continue;
        const Real gf = 0.72, gd = 0.85;   // width factor / extra depth
        auto gq = [&](Real oA, Real oB, Real dA, Real dB, const Vec3& nn) {
            MeshBuilder::emitQuad(out.barrier,
                                  ribs[i].at(oA) + Vec3(0, dA, 0),
                                  ribs[i + 1].at(oA) + Vec3(0, dA, 0),
                                  ribs[i + 1].at(oB) + Vec3(0, dB, 0),
                                  ribs[i].at(oB) + Vec3(0, dB, 0), nn,
                                  kConcrete * 0.92);
        };
        const Real wL = halfP[i] * gf, wR = -halfN[i] * gf;
        const Real top = -deckDepth, bot = -deckDepth - gd;
        gq(wR, wL, bot, bot, Vec3(0, -1, 0));                        // soffit
        gq(wL, wL, top, bot, Vec3(ribs[i].n.x, 0, ribs[i].n.y));     // side
        gq(wR, wR, bot, top, Vec3(-ribs[i].n.x, 0, -ribs[i].n.y));   // side
    }

    // elevated, the deck becomes a box — fascia sides + underside — on piers.
    for (int side = -1; side <= 1; side += 2) {
        for (int i = 0; i < n; ++i) {
            const Vec3 e0 = ribs[i].at(side * (side < 0 ? halfN[i] : halfP[i]));
            const Vec3 e1 = ribs[i + 1].at(side * (side < 0 ? halfN[i + 1] : halfP[i + 1]));
            const bool up = raised[i] || raised[i + 1];
            const Real d0 = up ? deckDepth : e0.y - (gy(Vec2(e0.x, e0.z)) - 0.6);
            const Real d1 = up ? deckDepth : e1.y - (gy(Vec2(e1.x, e1.z)) - 0.6);
            const Vec3 f0 = e0 - Vec3(0, std::max(Real(0.3), d0), 0);
            const Vec3 f1 = e1 - Vec3(0, std::max(Real(0.3), d1), 0);
            Vec3 nrm(side * ribs[i].n.x, 0, side * ribs[i].n.y);
            MeshBuilder::emitQuad(out.deck, f0, f1, e1, e0, nrm, kAsphalt * 1.6);
        }
    }
    // Underside where elevated (seen from streets below).
    for (int i = 0; i < n; ++i) {
        if (!(raised[i] || raised[i + 1])) continue;
        const Vec3 a0 = ribs[i].at(halfP[i]) - Vec3(0, deckDepth, 0);
        const Vec3 b0 = ribs[i].at(-halfN[i]) - Vec3(0, deckDepth, 0);
        const Vec3 a1 = ribs[i + 1].at(halfP[i + 1]) - Vec3(0, deckDepth, 0);
        const Vec3 b1 = ribs[i + 1].at(-halfN[i + 1]) - Vec3(0, deckDepth, 0);
        MeshBuilder::emitQuad(out.deck, b0, a0, a1, b1, Vec3(0, -1, 0),
                              kConcrete * 0.8);
    }
    // EDGE RAILINGS (device): a low concrete parapet running BOTH outer
    // edges, continuous over structure and at-grade alike — EXCEPT across an
    // exit's departure span, where the deck edge is the ramp's pavement and
    // the wall would slice the gore (device: "intersects with the parapet
    // awkwardly"). The ramp's own parapet takes over past the nose.
    auto inExitOpening = [&](Real s, int side) {
        for (const ExitDef& e : c.exits) {
            const int ds = e.upStation ? -1 : 1;
            if (side != ds) continue;
            // open exactly across the GORE BAND (the ramp piece that rides
            // the deck edge, §11) — the flare approach keeps its wall
            const Real dsf = e.upStation ? 1.0 : -1.0;
            const Real sgx = std::max(Real(1),
                                      std::min(e.station, Real(1e9)));
            const Real Lg2 = goreBandLen(c, e, L);
            const Real s0b = e.onRamp ? sgx - dsf * Lg2 : sgx;
            const Real s1b = e.onRamp ? sgx : sgx + dsf * Lg2;
            if (s > std::min(s0b, s1b) - 4.0 && s < std::max(s0b, s1b) + 4.0)
                return true;
        }
        return false;
    };
    for (int side = -1; side <= 1; side += 2) {
        const Real ph = 0.85, pt = 0.28;   // parapet height / thickness
        for (int i = 0; i < n; ++i) {
            if (inExitOpening(L * i / n, side) ||
                inExitOpening(L * (i + 1) / n, side))
                continue;
            auto o0 = [&](int k) { return side * (side < 0 ? halfN[k] : halfP[k]); };
            auto o1 = [&](int k) { return side * ((side < 0 ? halfN[k] : halfP[k]) - pt); };
            const Vec3 a0 = ribs[i].at(o0(i)), a1 = ribs[i + 1].at(o0(i + 1));
            const Vec3 b0 = ribs[i].at(o1(i)), b1 = ribs[i + 1].at(o1(i + 1));
            const Vec3 up3(0, ph, 0);
            Vec3 outN(side * ribs[i].n.x, 0, side * ribs[i].n.y);
            MeshBuilder::emitQuad(out.barrier, a0, a1, a1 + up3, a0 + up3,
                                  outN, kConcrete);                  // outer face
            MeshBuilder::emitQuad(out.barrier, b1, b0, b0 + up3, b1 + up3,
                                  outN * -1.0, kConcrete);           // inner face
            MeshBuilder::emitQuad(out.barrier, a0 + up3, a1 + up3, b1 + up3,
                                  b0 + up3, Vec3(0, 1, 0), kConcrete);   // cap
        }
    }

    // PIER BENTS (device: "circular pillars and U braces"): every ~28 m of
    // elevated run, a real bent — two ROUND columns + a cap beam under the
    // deck. A bent whose columns would land in a street below SLIDES along
    // the corridor until clear (or the span just gets longer). Footprints
    // are exported so the city passes treat them as USED ground.
    {
        auto clearOfRoads = [&](const Vec2& p) {
            if (!avoidRoads) return true;
            for (const RoadEdge& e : avoidRoads->edges) {
                if (e.a < 0 || e.b < 0 ||
                    e.a >= static_cast<int>(avoidRoads->nodes.size()) ||
                    e.b >= static_cast<int>(avoidRoads->nodes.size())) continue;
                const Vec2& ra = avoidRoads->nodes[e.a].pos;
                const Vec2& rb = avoidRoads->nodes[e.b].pos;
                Vec2 ab = rb - ra;
                const Real len2 = ab.lengthSquared();
                Real t = len2 > 1e-12 ? dot(p - ra, ab) / len2 : 0.0;
                t = t < 0 ? 0 : (t > 1 ? 1 : t);
                const Vec2 q(ra.x + ab.x * t, ra.y + ab.y * t);
                if ((q - p).length() < e.width * 0.5 + 1.8) return false;
            }
            return true;
        };
        auto emitBent = [&](Real s) {
            const Vec2 cc = c.horizontal.pos(s);
            const Vec2 nn = c.horizontal.normal(s);
            const Real z = c.vertical.elevation(s);
            const Real gz = gy(cc);
            const Real capTop = z - deckDepth - 0.80;   // cap meets the girder soffit
            if (capTop - gz < 0.8) return;
            const Real colOff = c.halfWidthAt(s) * 0.52;   // column pair spread
            const Real colR = 1.35;   // girthy (device: "tiny and scrawny")
            for (int sd = -1; sd <= 1; sd += 2) {
                const Vec2 foot2 = cc + nn * (sd * colOff);
                const Real fgz = gy(foot2);
                const Real hgt = capTop - 0.6 - (fgz - 1.0);
                if (hgt < 0.6) continue;
                RenderMesh col = MeshBuilder::cylinder(
                    static_cast<float>(colR), static_cast<float>(hgt), 12);
                for (Vertex& v : col.vertices) v.color = kConcrete;
                MeshBuilder::transform(
                    col, Mat4::translate(foot2.x, fgz - 1.0 + hgt * 0.5, foot2.y));
                MeshBuilder::append(out.barrier, col);
                out.pierBases.push_back(foot2);
            }
            // The cap beam (the U-brace's crossbar) tying the pair under the
            // deck — a box rotated to lie across the carriageways.
            const Real capW = c.halfWidthAt(s) * 2.0 * 0.92;
            RenderMesh cap = MeshBuilder::box(Vec3(capW, 1.6, 2.6));
            for (Vertex& v : cap.vertices) v.color = kConcrete * 1.05;
            const Real yaw = std::atan2(-nn.y, nn.x);
            MeshBuilder::transform(
                cap, Mat4::trs(Vec3(cc.x, capTop - 0.8, cc.y),
                               Quat::fromAxisAngle(Vec3(0, 1, 0), yaw),
                               Vec3(1, 1, 1)));
            MeshBuilder::append(out.barrier, cap);
        };
        Real sincePier = 26.0;
        for (int i = 0; i <= n; ++i) {
            sincePier += i > 0 ? L / n : 0.0;
            if (!elevated[i] || sincePier < 26.0) continue;
            const Real s0 = L * i / n;
            // slide the bent so no column stands in a road below
            bool placed = false;
            for (Real shift : {0.0, 4.0, -4.0, 8.0, -8.0, 12.0, -12.0, 16.0, -16.0}) {
                const Real s = std::max(Real(0), std::min(L, s0 + shift));
                const Vec2 cc = c.horizontal.pos(s);
                const Vec2 nn = c.horizontal.normal(s);
                const Real colOff = c.halfWidthAt(s) * 0.52;
                if (clearOfRoads(cc + nn * colOff) &&
                    clearOfRoads(cc - nn * colOff)) {
                    emitBent(s);
                    placed = true;
                    break;
                }
            }
            sincePier = placed ? 0.0 : 23.0;   // blocked: retry within 3 m —
                                               // the span stretches, never
                                               // a column in the street
        }
    }

    // AT-GRADE flatten: windows of the deck footprint carve the terrain to
    // the deck plane (the proven pad machinery does the blending). The gore
    // BAND (§11) widens the footprint on its side while it runs.
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
        // 4 m windows: a 12 m flat step vs a graded deck left the terrain
        // poking through in tiles (device: the dark rectangles); short steps
        // + carving 18 cm under the deck keep the ground beneath everywhere.
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

    // EXIT RAMPS (P8.3, device: "the exit should only have the last lane
    // peel off and become a connecting road to a surface street"): the ramp
    // centreline starts in the MIDDLE of the aux lane at the gore, inherits
    // the mainline tangent, and clothoids down to the street target on its
    // own crest/sag profile. It conforms (flattens terrain) ONLY over its
    // landing run — the body answers to its alignment, not the ground.
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
        const Vec2 travel = c.horizontal.tangent(sg) * ds;
        // ---- GORE BAND (§11 lego parts): the ramp's first piece is emitted
        // from the MAINLINE'S OWN RIBS — its inner edge IS the deck edge,
        // vertex for vertex, so the ramp can never detach from or slide
        // under the deck. Width blends aux-lane -> full ramp along the run;
        // all divergence happens in the free section beyond the band.
        const Real Lg = goreBandLen(c, e, L);
        const Real rampW = 5.8;
        const int nb = 30;
        const Real sBand0 = e.onRamp ? sg - ds * Lg : sg;   // flow-first end
        std::vector<Vec3> bandIn(nb + 1), bandOut(nb + 1), bandPath;
        std::vector<bool> bandUp(nb + 1);
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
            bandIn[i] = rb.at(innerOff);
            bandOut[i] = rb.at(innerOff + dirSign * tw);
            bandUp[i] = rb.z - gy(rb.c) > 0.35;
            bandPath.push_back(rb.at(innerOff + dirSign * tw * 0.5));
        }
        for (int i = 0; i < nb; ++i) {
            MeshBuilder::emitQuad(out.deck, bandIn[i], bandOut[i],
                                  bandOut[i + 1], bandIn[i + 1],
                                  Vec3(0, 1, 0), kAsphalt);
            // outer white edge line
            const Vec3 lift(0, 0.02, 0);
            Vec3 d0 = bandOut[i] - bandIn[i], d1 = bandOut[i + 1] - bandIn[i + 1];
            const Real l0 = std::max(Real(0.4), Real(d0.length()));
            const Real l1 = std::max(Real(0.4), Real(d1.length()));
            auto edgeAt = [&](int k, Real back) {
                const Vec3& o = k ? bandOut[i + 1] : bandOut[i];
                const Vec3 dd = k ? d1 * (1.0 / l1) : d0 * (1.0 / l0);
                return o - dd * back + lift;
            };
            MeshBuilder::emitQuad(out.markings, edgeAt(0, 0.5), edgeAt(0, 0.28),
                                  edgeAt(1, 0.28), edgeAt(1, 0.5),
                                  Vec3(0, 1, 0), kWhite);
            // outer parapet + fascia (the deck-side needs nothing: it IS the deck)
            const bool up2 = bandUp[i] || bandUp[i + 1];
            Vec3 nrm(dirSign * (bandOut[i].x - bandIn[i].x),
                     0, dirSign * (bandOut[i].z - bandIn[i].z));
            nrm = nrm.lengthSquared() > 1e-12 ? normalize(nrm) : Vec3(1, 0, 0);
            const Real f0 = up2 ? deckDepth
                                : bandOut[i].y - (gy(Vec2(bandOut[i].x, bandOut[i].z)) - 0.6);
            const Real f1 = up2 ? deckDepth
                                : bandOut[i + 1].y - (gy(Vec2(bandOut[i + 1].x, bandOut[i + 1].z)) - 0.6);
            MeshBuilder::emitQuad(out.deck,
                                  bandOut[i] - Vec3(0, std::max(Real(0.3), f0), 0),
                                  bandOut[i + 1] - Vec3(0, std::max(Real(0.3), f1), 0),
                                  bandOut[i + 1], bandOut[i], nrm, kAsphalt * 1.6);
            const Vec3 up3(0, 0.85, 0);
            MeshBuilder::emitQuad(out.barrier, bandOut[i], bandOut[i + 1],
                                  bandOut[i + 1] + up3, bandOut[i] + up3,
                                  nrm, kConcrete);
            MeshBuilder::emitQuad(out.barrier, bandOut[i + 1] + up3 * 0,
                                  bandOut[i] + up3 * 0,
                                  bandOut[i] + up3, bandOut[i + 1] + up3,
                                  nrm * -1.0, kConcrete);
        }
        // ---- FREE SECTION: starts EXACTLY where the band ends (same pose),
        // clothoids to the street. Never under the mainline: validated below.
        const Real sFree = e.onRamp ? sBand0 : sg + ds * Lg;
        const Real freeHalf = c.halfWidthAt(sFree, dirSign);
        const Vec2 C0 = c.horizontal.offset(
            sFree, dirSign * (freeHalf + rampW * 0.5));
        const Vec2 away = c.horizontal.tangent(sFree) * ds *
                          (e.onRamp ? -1.0 : 1.0);
        Alignment ra = Alignment::fromPolyline(
            {C0, C0 + away * 30.0, e.target}, e.rampRadius, e.rampSpiral, 2.0);
        bool folded = false;
        if (!ra.empty()) {
            const Real rl2 = ra.length();
            Vec2 tPrev = ra.tangent(0);
            for (Real s2 = 6.0; s2 <= rl2; s2 += 6.0) {
                const Vec2 tc = ra.tangent(std::min(s2, rl2 - 0.5));
                if (dot(tPrev, tc) < 0.05) { folded = true; break; }
                tPrev = tc;
            }
            if (rl2 > 2.4 * (e.target - C0).length()) folded = true;
        }
        if (ra.empty() || ra.length() < 48.0 || folded) {
            LOG_WARN << "[corridor] ramp at s=" << e.station
                     << (e.onRamp ? " (on-ramp)" : " (exit)")
                     << " dropped: free run "
                     << (ra.empty() ? 0.0 : ra.length())
                     << (folded ? " m FOLDS — street target too close/sharp"
                                : " m is too short — move station or target");
            out.rampPaths.back().pts.clear();
            continue;
        }
        // Full alignment reaches the street node; the RIBBON stops at the
        // landing setback (§10.3) — the street mesher owns the junction
        // mouth, and the loader grafts a stub edge that carries the pavement
        // from the setback point to the node.
        const Real RL = std::max(Real(40), ra.length() - e.landingSetback);
        // Grade change happens NEAR THE DECK (device: "it takes way too long
        // for the exit to descend"): drop/climb over the first ~45% of the
        // ramp, then run at street grade to the junction.
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
        const Real rw = 2.9;                       // half-width: lane + shoulders
        const int rn = std::max(2, static_cast<int>(RL / step));
        std::vector<Rib> rr(rn + 1);
        std::vector<bool> rUp(rn + 1);
        for (int i = 0; i <= rn; ++i) {
            const Real s = RL * i / rn;
            rr[i].c = ra.pos(s);
            rr[i].n = ra.normal(s);
            rr[i].z = rp.elevation(s);
            rr[i].slope = 0;
            // structure (box + piers) from 35 cm up: the old 0.9 m cutoff
            // skirted mid-height runs down to ground as terrain-hugging
            // berms the flatten could not slope nicely (device: "give it
            // some pillars underneath" — done)
            rUp[i] = rr[i].z - gy(rr[i].c) > 0.35;
        }
        {   // §11 guard: the free run must stay OUT of the mainline footprint
            bool crosses = false;
            for (int i = 0; i <= rn && !crosses; ++i) {
                const Real rs2 = RL * i / rn;
                if (rs2 < 20.0) continue;
                for (const Vec2& q : corrPts)
                    if ((q - rr[i].c).length() < corrGuard) { crosses = true; break; }
            }
            if (crosses)
                LOG_WARN << "[corridor] ramp at s=" << e.station
                         << " runs through the mainline footprint — its street "
                            "target sits on the wrong side; pick another";
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
        // The street end FLARES into a landing apron (device: "the onramp
        // needs to merge with the roads below nicer") — the last 8 m widen
        // toward the junction so the pavement reads as a mouth, not a stub.
        // The MOUTH (device: "open the mouth of that road ribbon into a
        // multilane road"): the last 45 m widen smoothly into a two-lane
        // throat at the junction.
        auto rwAt = [&](int i) {
            const Real rs2 = RL * i / rn;
            const Real m = std::max(Real(0), (rs2 - (RL - 45.0)) / 45.0);
            return rw + (m * m * (3.0 - 2.0 * m)) * 3.2;
        };
        for (int i = 0; i < rn; ++i) {
            // deck
            MeshBuilder::emitQuad(out.deck, rr[i].at(rwAt(i)), rr[i].at(-rwAt(i)),
                                  rr[i + 1].at(-rwAt(i + 1)), rr[i + 1].at(rwAt(i + 1)),
                                  Vec3(0, 1, 0), kAsphalt);
            // edge lines (device: "no markings for the sides" — the old
            // 12 cm threads vanished at speed): wide painted edges, YELLOW
            // left of travel / WHITE right, US ramp convention. Ramp rib 0
            // sits at the deck for exits and at the street for on-ramps, so
            // "left of travel" flips with the kind.
            for (int sd = -1; sd <= 1; sd += 2) {
                if (RL * i / rn > RL - 9.0) break;   // apron: lines open out
                const Real o = sd * (rw - 0.40);
                const bool leftOfTravel = e.onRamp ? (sd == dirSign)
                                                   : (sd != dirSign);
                const Vec3 lc = leftOfTravel ? Vec3(0.82, 0.72, 0.22) : kWhite;
                MeshBuilder::emitQuad(out.markings,
                                      rr[i].at(o + 0.11) + Vec3(0, 0.025, 0),
                                      rr[i].at(o - 0.11) + Vec3(0, 0.025, 0),
                                      rr[i + 1].at(o - 0.11) + Vec3(0, 0.025, 0),
                                      rr[i + 1].at(o + 0.11) + Vec3(0, 0.025, 0),
                                      Vec3(0, 1, 0), lc);
            }
            // fascia / skirts + parapets (the deck-side wall starts past
            // the nose so it never bars the gore opening)
            const bool up = rUp[i] || rUp[i + 1];
            const Real rs = RL * i / rn;
            for (int sd = -1; sd <= 1; sd += 2) {
                const bool inner = sd == -dirSign;
                // no wall across the gore opening (deck end) NOR across the
                // junction mouth (street end) — but everywhere between, BOTH
                // edges carry rail (device: "no railings along the
                // entrance/exits to the onramp")
                (void)inner;
                const bool skipParapet = rs > RL - 14.0;
                const Vec3 e0 = rr[i].at(sd * rwAt(i)), e1 = rr[i + 1].at(sd * rwAt(i + 1));
                const Real d0 = up ? deckDepth
                                   : e0.y - (gy(Vec2(e0.x, e0.z)) - 0.6);
                const Real d1 = up ? deckDepth
                                   : e1.y - (gy(Vec2(e1.x, e1.z)) - 0.6);
                Vec3 nrm(sd * rr[i].n.x, 0, sd * rr[i].n.y);
                MeshBuilder::emitQuad(out.deck,
                                      e0 - Vec3(0, std::max(Real(0.3), d0), 0),
                                      e1 - Vec3(0, std::max(Real(0.3), d1), 0),
                                      e1, e0, nrm, kAsphalt * 1.6);
                // parapet
                if (skipParapet) continue;
                const Vec3 up3(0, 0.85, 0);
                const Vec3 b0 = rr[i].at(sd * (rw - 0.24));
                const Vec3 b1 = rr[i + 1].at(sd * (rw - 0.24));
                MeshBuilder::emitQuad(out.barrier, e0, e1, e1 + up3, e0 + up3,
                                      nrm, kConcrete);
                MeshBuilder::emitQuad(out.barrier, b1, b0, b0 + up3, b1 + up3,
                                      nrm * -1.0, kConcrete);
                MeshBuilder::emitQuad(out.barrier, e0 + up3, e1 + up3, b1 + up3,
                                      b0 + up3, Vec3(0, 1, 0), kConcrete);
            }
            if (up)   // underside
                MeshBuilder::emitQuad(out.deck,
                                      rr[i].at(-rw) - Vec3(0, deckDepth, 0),
                                      rr[i].at(rw) - Vec3(0, deckDepth, 0),
                                      rr[i + 1].at(rw) - Vec3(0, deckDepth, 0),
                                      rr[i + 1].at(-rw) - Vec3(0, deckDepth, 0),
                                      Vec3(0, -1, 0), kConcrete * 0.8);
        }
        // single round columns where the ramp rides high
        Real since = 20.0;
        for (int i = 0; i <= rn; ++i) {
            since += i > 0 ? RL / rn : 0.0;
            if (!rUp[i] || since < 24.0) continue;
            since = 0;
            const Vec2 f2 = rr[i].c;
            const Real fgz = gy(f2);
            const Real hgt = rr[i].z - deckDepth - (fgz - 1.0);
            if (hgt < 0.8) continue;
            RenderMesh col = MeshBuilder::cylinder(1.1f, static_cast<float>(hgt), 12);
            for (Vertex& v : col.vertices) v.color = kConcrete;
            MeshBuilder::transform(col,
                Mat4::translate(f2.x, fgz - 1.0 + hgt * 0.5, f2.y));
            MeshBuilder::append(out.barrier, col);
            out.pierBases.push_back(f2);
        }
        // LANDING conform: flatten only the last stretch where it meets the
        // street grade (device: "conform with the city roads it connects to
        // but we probably don't need it to conform beyond that").
        for (Real s0 = std::max(Real(0), RL - 70.0); s0 < RL; s0 += 12.0) {
            const Real s1 = std::min(RL, s0 + 12.0);
            const Vec2 p0 = ra.pos(s0), p1 = ra.pos(s1);
            const Vec2 n0 = ra.normal(s0), n1 = ra.normal(s1);
            const Real hw2 = rw + 1.6;
            std::vector<Vec3> poly{
                Vec3(p0.x + n0.x * hw2, 0, p0.y + n0.y * hw2),
                Vec3(p1.x + n1.x * hw2, 0, p1.y + n1.y * hw2),
                Vec3(p1.x - n1.x * hw2, 0, p1.y - n1.y * hw2),
                Vec3(p0.x - n0.x * hw2, 0, p0.y - n0.y * hw2)};
            out.flatten.push_back(
                makeFlattenPad(std::move(poly), rp.elevation((s0 + s1) * 0.5), 5.0));
        }
        // SIGNAGE (device: "signage above the freeway ... rounded corner
        // placards (green)"): a gantry ~45 m before the gore — two posts, a
        // beam across the deck, a wide green board over the through lanes
        // (blank for now) and a smaller one over the exit lane wearing a
        // white drop-arrow for directionality.
        if (!e.onRamp) {
            const Real ss = std::max(Real(10), sg - 45.0);
            const Vec2 gc = c.horizontal.pos(ss);
            const Vec2 gn = c.horizontal.normal(ss);
            const Real gz2 = c.vertical.elevation(ss);
            const Real hL = c.halfWidthAt(ss, 1) + 0.6;
            const Real hR = c.halfWidthAt(ss, -1) + 0.6;
            const Real beamY = gz2 + 6.2;
            const Vec3 kGrey(0.45, 0.46, 0.48);
            auto postAt = [&](Real off) {
                const Vec2 p2 = gc + gn * off;
                RenderMesh post = MeshBuilder::cylinder(0.24f, static_cast<float>(beamY - gz2 + 0.6), 8);
                for (Vertex& v : post.vertices) v.color = kGrey;
                MeshBuilder::transform(post, Mat4::translate(p2.x, gz2 + (beamY - gz2 + 0.6) * 0.5, p2.y));
                MeshBuilder::append(out.barrier, post);
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
            MeshBuilder::append(out.barrier, beam);
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
                    MeshBuilder::append(out.barrier, strut);
                }
                MeshBuilder::append(out.barrier, bm);
            };
            const int dsn = dirSign;
            board(dsn * (c.medianWidth * 0.5 + c.shoulderIn +
                         c.lanes.throughLanes * 0.5 * c.laneWidth),
                  7.5, 2.2, false);
            board(dsn * (c.halfWidthAt(ss, dsn) - c.shoulderOut -
                         c.laneWidth * 0.5),
                  3.4, 2.2, true);
        }

        // LANDING FAN: a flat asphalt disc under the junction mouth — the
        // ramp, the street deck, and the apron all overlap it, so the seam
        // between corridor pavement and street pavement never shows.
        {
            const Vec2 lc2 = ra.pos(RL);
            const Real ly = rp.elevation(RL) + 0.04;
            RenderMesh fan;
            const int segs = 14;
            for (int k = 0; k < segs; ++k) {
                const Real a0 = 6.283185307179586 * k / segs;
                const Real a1 = 6.283185307179586 * (k + 1) / segs;
                MeshBuilder::emitTri(
                    fan, Vec3(lc2.x, ly, lc2.y),
                    Vec3(lc2.x + std::cos(a1) * 7.0, ly, lc2.y + std::sin(a1) * 7.0),
                    Vec3(lc2.x + std::cos(a0) * 7.0, ly, lc2.y + std::sin(a0) * 7.0),
                    Vec3(0, 1, 0), kAsphalt);
            }
            MeshBuilder::append(out.deck, fan);
        }

        // RAMP PIERS: an elevated ramp is a structure too — one round column
        // every ~24 m of raised run, capped where it meets the ramp soffit.
        {
            Real since = 18.0;
            for (int i = 0; i <= rn; ++i) {
                since += i > 0 ? RL / rn : 0.0;
                if (!rUp[i] || since < 24.0) continue;
                since = 0;
                const Vec2 foot2 = rr[i].c;
                const Real fgz = gy(foot2);
                const Real topY = rr[i].z - deckDepth - 0.15;
                const Real hgt = topY - (fgz - 1.0);
                if (hgt < 1.2) continue;
                RenderMesh col = MeshBuilder::cylinder(0.8f,
                    static_cast<float>(hgt), 10);
                for (Vertex& v : col.vertices) v.color = kConcrete;
                MeshBuilder::transform(col,
                    Mat4::translate(foot2.x, fgz - 1.0 + hgt * 0.5, foot2.y));
                MeshBuilder::append(out.barrier, col);
                out.pierBases.push_back(foot2);
            }
        }


    }

    return out;
}

}  // namespace engine
