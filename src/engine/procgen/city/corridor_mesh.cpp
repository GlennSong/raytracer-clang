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
    const CorridorDef& c, const std::function<Real(Real, Real)>& ground,
    Real step, const RoadGraph* avoidRoads) {
    CorridorMeshOut out;
    const Real L = c.horizontal.length();
    if (L < step * 2) return out;
    auto gy = [&](const Vec2& p) { return ground ? ground(p.x, p.y) : Real(0); };

    const int n = std::max(2, static_cast<int>(std::ceil(L / step)));
    std::vector<Rib> ribs(n + 1);
    std::vector<Real> half(n + 1);
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
        half[i] = c.halfWidthAt(s);
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
    strip(out.deck, [&](int i) { return half[i]; },
          [&](int i) { return -half[i]; }, kAsphalt, 0.0);

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
            return side * (half[i] - c.shoulderOut);
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
                if (l >= c.lanes.lanesAt(s)) continue;
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
            const Vec3 e0 = ribs[i].at(side * half[i]);
            const Vec3 e1 = ribs[i + 1].at(side * half[i + 1]);
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
        const Vec3 a0 = ribs[i].at(half[i]) - Vec3(0, deckDepth, 0);
        const Vec3 b0 = ribs[i].at(-half[i]) - Vec3(0, deckDepth, 0);
        const Vec3 a1 = ribs[i + 1].at(half[i + 1]) - Vec3(0, deckDepth, 0);
        const Vec3 b1 = ribs[i + 1].at(-half[i + 1]) - Vec3(0, deckDepth, 0);
        MeshBuilder::emitQuad(out.deck, b0, a0, a1, b1, Vec3(0, -1, 0),
                              kConcrete * 0.8);
    }
    // EDGE RAILINGS (device): a low concrete parapet running BOTH outer
    // edges, continuous over structure and at-grade alike — nothing drives
    // off this deck.
    for (int side = -1; side <= 1; side += 2) {
        const Real ph = 0.85, pt = 0.28;   // parapet height / thickness
        for (int i = 0; i < n; ++i) {
            auto o0 = [&](int k) { return side * half[k]; };
            auto o1 = [&](int k) { return side * (half[k] - pt); };
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
                const Real h0 = c.halfWidthAt(s0) + 2.0, h1 = c.halfWidthAt(s1) + 2.0;
                std::vector<Vec3> poly{
                    Vec3(p0.x + n0.x * h0, 0, p0.y + n0.y * h0),
                    Vec3(p1.x + n1.x * h1, 0, p1.y + n1.y * h1),
                    Vec3(p1.x - n1.x * h1, 0, p1.y - n1.y * h1),
                    Vec3(p0.x - n0.x * h0, 0, p0.y - n0.y * h0)};
                out.flatten.push_back(
                    makeFlattenPad(std::move(poly), c.vertical.elevation(sm), 6.0));
            }
            s0 = s1;
        }
    }
    return out;
}

}  // namespace engine
