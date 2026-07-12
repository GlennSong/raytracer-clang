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
    Real step) {
    CorridorMeshOut out;
    const Real L = c.horizontal.length();
    if (L < step * 2) return out;
    auto gy = [&](const Vec2& p) { return ground ? ground(p.x, p.y) : Real(0); };

    const int n = std::max(2, static_cast<int>(std::ceil(L / step)));
    std::vector<Rib> ribs(n + 1);
    std::vector<Real> half(n + 1);
    std::vector<bool> elevated(n + 1);
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
            const bool up = elevated[i] || elevated[i + 1];
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
        if (!(elevated[i] || elevated[i + 1])) continue;
        const Vec3 a0 = ribs[i].at(half[i]) - Vec3(0, deckDepth, 0);
        const Vec3 b0 = ribs[i].at(-half[i]) - Vec3(0, deckDepth, 0);
        const Vec3 a1 = ribs[i + 1].at(half[i + 1]) - Vec3(0, deckDepth, 0);
        const Vec3 b1 = ribs[i + 1].at(-half[i + 1]) - Vec3(0, deckDepth, 0);
        MeshBuilder::emitQuad(out.deck, b0, a0, a1, b1, Vec3(0, -1, 0),
                              kConcrete * 0.8);
    }
    // PIERS: a bent (column) every ~28 m of elevated run, at the median.
    {
        Real sincePier = 28.0;   // drop one near the start of the span
        for (int i = 0; i <= n; ++i) {
            const Real ds = i > 0 ? L / n : 0.0;
            sincePier += ds;
            if (!elevated[i] || sincePier < 28.0) continue;
            sincePier = 0;
            const Rib& r = ribs[i];
            const Real gz = gy(r.c);
            const Real top = r.z - deckDepth + 0.05;
            if (top - gz < 1.0) continue;
            // a rectangular column, wider across the road than along it
            const Vec2 across = r.n, along(r.n.y, -r.n.x);
            const Real hw2 = 1.1, hd2 = 0.7;
            const Vec3 base(r.c.x, gz - 1.0, r.c.y);
            Vec3 axA(across.x, 0, across.y), axB(along.x, 0, along.y);
            const Real h = top - (gz - 1.0);
            // 4 sides of the column
            auto post = [&](const Vec3& u, const Vec3& v, Real hu, Real hv) {
                const Vec3 c0 = base - u * hu - v * hv;
                const Vec3 c1 = base + u * hu - v * hv;
                const Vec3 c2 = base + u * hu + v * hv;
                const Vec3 c3 = base - u * hu + v * hv;
                const Vec3 up(0, h, 0);
                auto wall = [&](const Vec3& p0, const Vec3& p1, const Vec3& nn) {
                    MeshBuilder::emitQuad(out.barrier, p0, p1, p1 + up, p0 + up,
                                          nn, kConcrete);
                };
                wall(c0, c1, v * -1.0 * (1.0 / std::max(Real(1e-9), hv)));
                wall(c1, c2, u * (1.0 / std::max(Real(1e-9), hu)));
                wall(c2, c3, v * (1.0 / std::max(Real(1e-9), hv)));
                wall(c3, c0, u * -1.0 * (1.0 / std::max(Real(1e-9), hu)));
            };
            post(axA, axB, hw2, hd2);
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
