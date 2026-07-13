#include "corridor_mesh.h"

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

}  // namespace

CorridorMeshOut buildCorridorMesh(
    const CorridorDef& cIn, const std::function<Real(Real, Real)>& ground,
    Real step, const RoadGraph* avoidRoads) {
    CorridorMeshOut out;
    CorridorDef c = cIn;
    // Each exit grows its deceleration AUX LANE into the schedule — the deck
    // flares one lane on that side over the decel length, ending at the gore.
    for (const ExitDef& e : c.exits)
        c.lanes.aux.push_back({e.station - e.decelLength, e.station,
                               e.upStation});
    const Real L = c.horizontal.length();
    if (L < step * 2) return out;
    auto gy = [&](const Vec2& p) { return ground ? ground(p.x, p.y) : Real(0); };

    const int n = std::max(2, static_cast<int>(std::ceil(L / step)));
    std::vector<Rib> ribs(n + 1);
    std::vector<Real> halfN(n + 1), halfP(n + 1);   // per-side deck reach
    std::vector<bool> elevated(n + 1);   // tall enough for pier bents
    std::vector<bool> raised(n + 1);     // off the ground at all: structure box
    const Real deckDepth = 1.1;            // structure depth of the viaduct box
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
    // edges, continuous over structure and at-grade alike — nothing drives
    // off this deck.
    for (int side = -1; side <= 1; side += 2) {
        const Real ph = 0.85, pt = 0.28;   // parapet height / thickness
        for (int i = 0; i < n; ++i) {
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
            const Real capTop = z - deckDepth + 0.08;
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
    // the deck plane (the proven pad machinery does the blending).
    {
        const Real win = 12.0;
        Real s0 = 0;
        while (s0 < L) {
            const Real s1 = std::min(L, s0 + win);
            const Real sm = (s0 + s1) * 0.5;
            const int i0 = static_cast<int>(s0 / L * n);
            if (!elevated[std::min(i0, n)]) {
                const Vec2 p0 = c.horizontal.pos(s0), p1 = c.horizontal.pos(s1);
                const Vec2 n0 = c.horizontal.normal(s0), n1 = c.horizontal.normal(s1);
                const Real hp0 = c.halfWidthAt(s0, 1) + 2.0, hp1 = c.halfWidthAt(s1, 1) + 2.0;
                const Real hn0 = c.halfWidthAt(s0, -1) + 2.0, hn1 = c.halfWidthAt(s1, -1) + 2.0;
                std::vector<Vec3> poly{
                    Vec3(p0.x + n0.x * hp0, 0, p0.y + n0.y * hp0),
                    Vec3(p1.x + n1.x * hp1, 0, p1.y + n1.y * hp1),
                    Vec3(p1.x - n1.x * hn1, 0, p1.y - n1.y * hn1),
                    Vec3(p0.x - n0.x * hn0, 0, p0.y - n0.y * hn0)};
                out.flatten.push_back(
                    makeFlattenPad(std::move(poly), c.vertical.elevation(sm), 6.0));
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
    for (const ExitDef& e : c.exits) {
        const Real sg = std::max(Real(1), std::min(e.station, L - 1.0));
        const int dirSign = e.upStation ? -1 : 1;
        const Vec2 travel = c.horizontal.tangent(sg) * (e.upStation ? 1.0 : -1.0);
        const Real off0 =
            dirSign * (c.medianWidth * 0.5 + c.shoulderIn +
                       (c.lanes.lanesAt(sg - 1.0, e.upStation) - 0.5) * c.laneWidth);
        const Vec2 P0 = c.horizontal.offset(sg, off0);
        Alignment ra = Alignment::fromPolyline(
            {P0, P0 + travel * 45.0, e.target}, e.rampRadius, e.rampSpiral, 2.0);
        if (ra.empty() || ra.length() < 60.0) continue;
        const Real RL = ra.length();
        VerticalProfile rp;
        rp.pvis = {{0, c.vertical.elevation(sg), 0},
                   {RL, e.targetY, std::min(RL * 0.5, Real(90))}};
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
            rUp[i] = rr[i].z - gy(rr[i].c) > 0.9;
        }
        for (int i = 0; i < rn; ++i) {
            // deck
            MeshBuilder::emitQuad(out.deck, rr[i].at(rw), rr[i].at(-rw),
                                  rr[i + 1].at(-rw), rr[i + 1].at(rw),
                                  Vec3(0, 1, 0), kAsphalt);
            // edge lines
            for (int sd = -1; sd <= 1; sd += 2) {
                const Real o = sd * (rw - 0.35);
                MeshBuilder::emitQuad(out.markings,
                                      rr[i].at(o + 0.06) + Vec3(0, 0.015, 0),
                                      rr[i].at(o - 0.06) + Vec3(0, 0.015, 0),
                                      rr[i + 1].at(o - 0.06) + Vec3(0, 0.015, 0),
                                      rr[i + 1].at(o + 0.06) + Vec3(0, 0.015, 0),
                                      Vec3(0, 1, 0), kWhite);
            }
            // fascia / skirts + parapets
            const bool up = rUp[i] || rUp[i + 1];
            for (int sd = -1; sd <= 1; sd += 2) {
                const Vec3 e0 = rr[i].at(sd * rw), e1 = rr[i + 1].at(sd * rw);
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
        // GORE NOSE: the painted wedge between the mainline's outer edge and
        // the ramp's inner edge as they diverge past the gore.
        for (int k = 0; k < 5; ++k) {
            const Real ds0 = k * 3.0, ds1 = ds0 + 3.0;
            const Real sm0 = std::min(L, sg + (e.upStation ? ds0 : -ds0));
            const Real sm1 = std::min(L, sg + (e.upStation ? ds1 : -ds1));
            const Vec3 d0v = Vec3(0, 0.02, 0) +
                Vec3(c.horizontal.offset(sm0, dirSign * c.halfWidthAt(sm0, dirSign)).x,
                     c.vertical.elevation(sm0),
                     c.horizontal.offset(sm0, dirSign * c.halfWidthAt(sm0, dirSign)).y);
            const Vec3 d1v = Vec3(0, 0.02, 0) +
                Vec3(c.horizontal.offset(sm1, dirSign * c.halfWidthAt(sm1, dirSign)).x,
                     c.vertical.elevation(sm1),
                     c.horizontal.offset(sm1, dirSign * c.halfWidthAt(sm1, dirSign)).y);
            const Vec3 r0v = rr[std::min(rn, (int)(ds0 / RL * rn))].at(-dirSign * (rw - 0.3)) + Vec3(0, 0.02, 0);
            const Vec3 r1v = rr[std::min(rn, (int)(ds1 / RL * rn))].at(-dirSign * (rw - 0.3)) + Vec3(0, 0.02, 0);
            MeshBuilder::emitQuad(out.markings, d0v, r0v, r1v, d1v,
                                  Vec3(0, 1, 0), kWhite);
        }
    }

    return out;
}

}  // namespace engine
