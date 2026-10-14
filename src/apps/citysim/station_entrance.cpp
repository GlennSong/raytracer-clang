#include "station_entrance.h"

#include <algorithm>
#include <cmath>

using engine::Mat4;
using engine::RenderMesh;
using engine::Vec3;
using engine::Vertex;

namespace citysim {

std::vector<StationEntrance> stationEntrances(const RailSim& sim, const std::function<double(double, double)>& ground) {
    std::vector<StationEntrance> out;
    for (const RailLine& L : sim.lines())
        for (const engine::RailStop& st : L.stops) {
            if (!st.hasAccess || st.name.empty()) continue;
            bool seen = false;
            for (const StationEntrance& e : out) seen = seen || e.name == st.name;
            if (seen) continue;
            const Vec3 mid = L.at((st.s0 + st.s1) * 0.5);
            Vec3 along(mid.x - st.access.x, 0, mid.z - st.access.y);
            const double len = along.length();
            if (len < 1.0) continue;
            StationEntrance e;
            e.name = st.name;
            e.cls = L.cls;
            e.along = along * (1.0 / len);
            // 2.5 m up the footway from where it leaves the street: clear of the kerb's lamps, signs and bus stop
            const double fx = st.access.x + e.along.x * 2.5, fz = st.access.y + e.along.z * 2.5;
            e.foot = Vec3(fx, ground(fx, fz), fz);
            out.push_back(e);
        }
    return out;
}

namespace {

const Vec3 kConcrete(0.78, 0.77, 0.74), kSteel(0.55, 0.57, 0.60), kMachine(0.18, 0.42, 0.72), kScreen(0.05, 0.10, 0.18),
    kWood(0.55, 0.38, 0.22), kGate(0.80, 0.81, 0.83), kGateTop(0.15, 0.35, 0.70);

// The pavilion in its own frame -- x across the footway, y up, z along it from the way in -- each box set on the ground
// under its middle; boxes go to `to` (and to the collider, unless `ghost`).
struct Builder {
    const StationEntrance& e;
    const std::function<double(double, double)>& ground;
    Vec3 xAx, zAx;
    EntranceMeshes& out;
    Vec3 world(double x, double y, double z) const { return e.foot + xAx * x + Vec3(0, y, 0) + zAx * z; }
    double groundUnder(double x, double z) const { const Vec3 w = world(x, 0, z); return ground(w.x, w.z) - e.foot.y; }
    void box(RenderMesh& to, double x0, double x1, double y0, double y1, double z0, double z1, const Vec3& col,
             bool solid = true, bool onGround = true) {
        const double g = onGround ? groundUnder((x0 + x1) * 0.5, (z0 + z1) * 0.5) : 0.0;
        const Vec3 c[8] = {world(x0, y0 + g, z0), world(x1, y0 + g, z0), world(x1, y1 + g, z0), world(x0, y1 + g, z0),
                           world(x0, y0 + g, z1), world(x1, y0 + g, z1), world(x1, y1 + g, z1), world(x0, y1 + g, z1)};
        const int f[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 4, 7, 3}, {1, 2, 6, 5}, {0, 1, 5, 4}, {3, 7, 6, 2}};
        const Vec3 n[6] = {zAx * -1.0, zAx, xAx * -1.0, xAx, Vec3(0, -1, 0), Vec3(0, 1, 0)};
        for (RenderMesh* m : {&to, solid ? &out.collider : nullptr}) {
            if (!m) continue;
            for (int k = 0; k < 6; ++k) {
                const uint32_t base = static_cast<uint32_t>(m->vertices.size());
                for (int j = 0; j < 4; ++j) {
                    Vertex v(c[f[k][j]], n[k]);
                    v.color = col;
                    m->vertices.push_back(v);
                }
                for (uint32_t i : {0u, 1u, 2u, 0u, 2u, 3u}) m->indices.push_back(base + i);
            }
        }
    }
};

}  // namespace

EntranceMeshes stationEntranceMeshes(const StationEntrance& e, const std::function<double(double, double)>& ground) {
    EntranceMeshes out;
    Builder b{e, ground, engine::cross(Vec3(0, 1, 0), e.along), e.along, out};   // (right-handed: x, y, z)
    // the roof sits 3.6 m over the highest ground under the columns
    double top = -1e9;
    const double cx[2] = {-3.9, 3.9}, cz[3] = {0.3, 3.6, 7.0};
    for (double x : cx) for (double z : cz) top = std::max(top, b.groundUnder(x, z));
    const double roofY = top + 3.6;
    const Vec3 band = e.cls == "light_rail" ? Vec3(0.05, 0.45, 0.55) : Vec3(0.10, 0.40, 0.25);   // the line's colour
    // COLUMNS, and the ROOF over them (out of reach: no collider) with its coloured fascia
    for (double x : cx)
        for (double z : cz) {
            const double g = b.groundUnder(x, z);
            b.box(out.body, x - 0.16, x + 0.16, g, roofY, z - 0.16, z + 0.16, kConcrete, true, false);
        }
    b.box(out.body, -4.4, 4.4, roofY, roofY + 0.32, -0.4, 7.6, kConcrete, false, false);
    b.box(out.body, -4.45, 4.45, roofY - 0.1, roofY + 0.45, -0.45, -0.38, band, false, false);   // the street face's fascia
    b.box(out.body, -4.45, 4.45, roofY - 0.1, roofY + 0.45, 7.58, 7.65, band, false, false);
    for (double s : {-1.0, 1.0}) b.box(out.body, s * 4.45 - 0.035, s * 4.45 + 0.035, roofY - 0.1, roofY + 0.45, -0.45, 7.65, band, false, false);
    // GLASS SIDE WALLS on a steel kerb, a rail along their top
    for (double s : {-1.0, 1.0}) {
        b.box(out.body, s * 3.95 - 0.06, s * 3.95 + 0.06, 0.0, 0.15, 0.5, 6.8, kSteel);
        b.box(out.glass, s * 3.95 - 0.02, s * 3.95 + 0.02, 0.15, 2.5, 0.5, 6.8, Vec3(0.75, 0.85, 0.90));
        b.box(out.body, s * 3.95 - 0.05, s * 3.95 + 0.05, 2.5, 2.58, 0.5, 6.8, kSteel, false);
    }
    // TICKET MACHINES down the left wall, their screens toward the way through
    for (double z : {0.9, 1.95, 3.0}) {
        b.box(out.body, -3.85, -3.25, 0.0, 1.75, z, z + 0.85, kMachine);
        b.box(out.body, -3.25, -3.23, 1.0, 1.45, z + 0.15, z + 0.70, kScreen, false);
        b.box(out.body, -3.25, -3.20, 0.85, 0.92, z + 0.30, z + 0.55, kSteel, false);   // the card slot
    }
    // the KIOSK and a BENCH down the right wall
    b.box(out.body, 2.7, 3.85, 0.0, 1.05, 0.8, 2.9, kWood);
    b.box(out.body, 2.65, 3.88, 1.05, 1.10, 0.75, 2.95, kSteel, false);
    for (int i = 0; i < 4; ++i)
        b.box(out.body, 3.25, 3.85, 1.15 + i * 0.30, 1.40 + i * 0.30, 0.9, 2.8, Vec3(0.70 - 0.1 * i, 0.30 + 0.12 * i, 0.20 + 0.15 * (i % 2)), false);
    b.box(out.body, 2.7, 3.85, 2.30, 2.55, 0.8, 2.9, Vec3(0.75, 0.12, 0.10), false);           // its sign
    b.box(out.body, 3.4, 3.85, 0.0, 0.46, 3.4, 5.2, kWood);                                     // the bench
    b.box(out.body, 3.75, 3.85, 0.46, 0.95, 3.4, 5.2, kWood, false);
    // THE GATE LINE across the way up: cabinets with a passage between each (1 m: a walker's width and more)
    for (double x : {-2.85, -1.55, -0.25, 1.05, 2.35}) {
        b.box(out.body, x - 0.13, x + 0.13, 0.0, 1.0, 4.4, 5.9, kGate);
        b.box(out.body, x - 0.13, x + 0.13, 1.0, 1.04, 4.4, 5.9, kGateTop, false);
        b.box(out.body, x - 0.135, x + 0.135, 0.55, 0.75, 4.35, 4.55, Vec3(0.2, 0.8, 0.3), false);   // its go light
    }
    // THE NAME on the roof, facing the street (station_boards.h's frame: +x along, +y up, +z toward the reader)
    const Vec3 zb = e.along * -1.0, xb = engine::cross(Vec3(0, 1, 0), zb);
    const Vec3 foot = b.world(0, roofY + 0.32, 0.4);
    Mat4& m = out.board;
    m.m[0][0] = xb.x; m.m[1][0] = xb.y; m.m[2][0] = xb.z;
    m.m[0][1] = 0;    m.m[1][1] = 1;    m.m[2][1] = 0;
    m.m[0][2] = zb.x; m.m[1][2] = zb.y; m.m[2][2] = zb.z;
    m.m[0][3] = foot.x - 0.0; m.m[1][3] = foot.y - 1.6; m.m[2][3] = foot.z;   // (its posts' first 1.6 m hidden in the roof)
    m.m[3][0] = m.m[3][1] = m.m[3][2] = 0; m.m[3][3] = 1;
    return out;
}

}  // namespace citysim
