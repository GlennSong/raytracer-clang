#include "train_mesh.h"

#include "../../mesh_builder.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

const Vec3 kSilver(0.80f, 0.81f, 0.83f), kUnder(0.16f, 0.16f, 0.17f), kBogie(0.12f, 0.12f, 0.12f), kWheel(0.30f, 0.29f, 0.28f),
           kFloor(0.30f, 0.33f, 0.38f), kCeiling(0.88f, 0.88f, 0.86f), kInnerWall(0.78f, 0.77f, 0.73f), kSeat(0.18f, 0.27f, 0.48f),
           kSeatFrame(0.55f, 0.55f, 0.56f), kRubber(0.07f, 0.07f, 0.07f), kPole(0.85f, 0.75f, 0.20f), kLamp(1.0f, 0.97f, 0.88f),
           kDarkGlass(0.05f, 0.06f, 0.08f), kGrille(0.10f, 0.10f, 0.11f), kHeadlight(1.0f, 1.0f, 0.95f);



// a wall piece in the plane x = const, facing `nx`
void wallX(RenderMesh& m, Real x, Real nx, Real z0, Real z1, Real y0, Real y1, const Vec3& col) {
    if (z1 - z0 < 1e-4 || y1 - y0 < 1e-4) return;
    MeshBuilder::emitQuad(m, {x, y0, z0}, {x, y0, z1}, {x, y1, z1}, {x, y1, z0}, {nx, 0, 0}, col);
}
// ...and in the plane z = const, facing `nz`
void wallZ(RenderMesh& m, Real z, Real nz, Real x0, Real x1, Real y0, Real y1, const Vec3& col) {
    if (x1 - x0 < 1e-4 || y1 - y0 < 1e-4) return;
    MeshBuilder::emitQuad(m, {x0, y0, z}, {x1, y0, z}, {x1, y1, z}, {x0, y1, z}, {0, 0, nz}, col);
}

void wheelset(RenderMesh& m, Real zc, Real gaugeHalf) {
    for (int side = -1; side <= 1; side += 2) {
        RenderMesh w = MeshBuilder::cylinder(0.46f, 0.14f, 18);
        MeshBuilder::transform(w, Mat4::translate(side * gaugeHalf, 0.46, zc) * Mat4::rotateZ(1.5707963267948966));
        for (Vertex& v : w.vertices) v.color = kWheel;
        MeshBuilder::append(m, w);
    }
    MeshBuilder::emitBox(m, {-gaugeHalf, 0.40, zc - 0.07}, {gaugeHalf, 0.52, zc + 0.07}, kBogie);   // the axle
}

// A BOGIE at body station zc: frame, springs, two wheelsets on the rails.
void bogie(RenderMesh& m, Real zc) {
    const Real gh = 0.7525;
    MeshBuilder::emitBox(m, {-1.05, 0.55, zc - 1.45}, {-0.85, 0.85, zc + 1.45}, kBogie);
    MeshBuilder::emitBox(m, {0.85, 0.55, zc - 1.45}, {1.05, 0.85, zc + 1.45}, kBogie);
    MeshBuilder::emitBox(m, {-0.85, 0.62, zc - 0.25}, {0.85, 0.82, zc + 0.25}, kBogie);   // bolster
    wheelset(m, zc - 1.25, gh);
    wheelset(m, zc + 1.25, gh);
}

// Colour of the outer skin at height y: silver, with the livery stripe under the windows.
struct Bands { Real s0, s1; Vec3 livery; };
void skinX(RenderMesh& m, Real x, Real nx, Real z0, Real z1, Real y0, Real y1, const Bands& b) {
    const Real cuts[4] = {y0, std::clamp(b.s0, y0, y1), std::clamp(b.s1, y0, y1), y1};
    for (int k = 0; k < 3; ++k) wallX(m, x, nx, z0, z1, cuts[k], cuts[k + 1], k == 1 ? b.livery : kSilver);
}

// the four reveal faces of an opening through a wall of thickness (xo..xi) at side s
void reveals(RenderMesh& m, Real xo, Real xi, Real z0, Real z1, Real y0, Real y1, bool sill, const Vec3& col) {
    const Real a = std::min(xo, xi), b = std::max(xo, xi);
    if (sill) MeshBuilder::emitQuad(m, {a, y0, z0}, {b, y0, z0}, {b, y0, z1}, {a, y0, z1}, {0, 1, 0}, col);
    MeshBuilder::emitQuad(m, {a, y1, z0}, {b, y1, z0}, {b, y1, z1}, {a, y1, z1}, {0, -1, 0}, col);
    MeshBuilder::emitQuad(m, {a, y0, z0}, {b, y0, z0}, {b, y1, z0}, {a, y1, z0}, {0, 0, 1}, col);
    MeshBuilder::emitQuad(m, {a, y0, z1}, {b, y0, z1}, {b, y1, z1}, {a, y1, z1}, {0, 0, -1}, col);
}

}  // namespace

TrainBodyMesh buildTrainCarriage(const TrainBodySpec& sp) {
    TrainBodyMesh out;
    RenderMesh& m = out.body;
    const bool longitudinal = sp.kind != "intercity";   // subway and light rail: benches along the walls
    const Real L = sp.length, hl = L / 2, hw = sp.width / 2, t = 0.08, F = sp.floor, R = sp.roof;
    const Real eave = R - 0.5, ceil = R - 0.75, sill = 0.95;
    const Real wb0 = F + 0.80, wb1 = F + 1.90;          // the window band
    const Bands bands{F + 0.52, F + 0.68, sp.livery};
    out.length = L; out.width = sp.width; out.floor = F; out.walkable = true;
    out.doorHeight = 2.0;
    out.doorHalf = longitudinal ? 0.70 : 0.60;
    out.aisleHalf = longitudinal ? 0.85 : 0.38;
    out.gangwayHalf = 0.55;
    if (longitudinal) { out.doorZ = {-L / 3, 0.0, L / 3}; }
    else out.doorZ = {-(hl - 2.0), hl - 2.0};
    const Real dh = out.doorHalf, gw = out.gangwayHalf;

    // under the floor: the frame and its equipment, the bogies
    // inside the skirt and the end walls, well under the floor: its faces coplanar with theirs (and its top 2 cm
    // under the floor's) z-fought from outside (Glenn, 2026-10-10)
    MeshBuilder::emitBox(m, {-hw + 0.06, sill, -hl + 0.06}, {hw - 0.06, F - 0.12, hl - 0.06}, kUnder);
    MeshBuilder::emitBox(m, {-hw + 0.25, 0.6, -hl * 0.55}, {hw - 0.25, sill, hl * 0.55}, kUnder);
    bogie(m, 0.32 * L);
    bogie(m, -0.32 * L);
    // the floor and the ceiling
    MeshBuilder::emitQuad(m, {-hw + t, F, -hl}, {hw - t, F, -hl}, {hw - t, F, hl}, {-hw + t, F, hl}, {0, 1, 0}, kFloor);
    MeshBuilder::emitQuad(m, {-hw + t, ceil, -hl + t}, {hw - t, ceil, -hl + t}, {hw - t, ceil, hl - t}, {-hw + t, ceil, hl - t}, {0, -1, 0}, kCeiling);
    for (Real z = -hl + 1.5; z < hl - 1.0; z += 3.0)   // light strips
        MeshBuilder::emitQuad(m, {-0.25, ceil - 0.01, z}, {0.25, ceil - 0.01, z}, {0.25, ceil - 0.01, z + 2.0}, {-0.25, ceil - 0.01, z + 2.0}, {0, -1, 0}, kLamp);
    // the roof: a flat top with chamfered eaves, and the upper wall between eave and ceiling inside
    MeshBuilder::emitQuad(m, {-hw + 0.45, R, -hl}, {hw - 0.45, R, -hl}, {hw - 0.45, R, hl}, {-hw + 0.45, R, hl}, {0, 1, 0}, kSilver);
    for (int s = -1; s <= 1; s += 2) {
        const Vec3 n = normalize(Vec3(s * 0.5, 0.45, 0));
        MeshBuilder::emitQuad(m, {s * hw, eave, -hl}, {s * (hw - 0.45), R, -hl}, {s * (hw - 0.45), R, hl}, {s * hw, eave, hl}, n, kSilver);
    }

    // THE SIDES: solid between openings, windows in runs between the doors, the doors open to the floor
    struct Span { Real z0, z1; int kind; };   // 0 solid, 1 window, 2 door
    std::vector<Span> spans;
    {
        std::vector<std::pair<Real, Real>> doors;
        for (double dz : out.doorZ) doors.push_back({static_cast<Real>(dz) - dh, static_cast<Real>(dz) + dh});
        Real z = -hl;
        auto windows = [&](Real a, Real b) {   // windows 1.5 m wide on 0.5 m piers, centred in [a, b]
            const Real span = b - a - 0.5;
            const int n = static_cast<int>(std::floor(span / 2.0));
            if (n <= 0) { spans.push_back({a, b, 0}); return; }
            const Real used = n * 1.5 + (n - 1) * 0.5, start = a + (b - a - used) / 2;
            Real zz = a;
            for (int k = 0; k < n; ++k) {
                const Real w0 = start + k * 2.0, w1 = w0 + 1.5;
                spans.push_back({zz, w0, 0});
                spans.push_back({w0, w1, 1});
                zz = w1;
            }
            spans.push_back({zz, b, 0});
        };
        for (const auto& [d0, d1] : doors) {
            windows(z, d0);
            spans.push_back({d0, d1, 2});
            z = d1;
        }
        windows(z, hl);
    }
    for (int s = -1; s <= 1; s += 2) {
        const Real xo = s * hw, xi = s * (hw - t);
        skinX(m, xo, s, -hl, hl, sill, F, bands);   // the skirt
        for (const Span& sp2 : spans) {
            if (sp2.kind == 0) {
                skinX(m, xo, s, sp2.z0, sp2.z1, F, eave, bands);
                wallX(m, xi, -s, sp2.z0, sp2.z1, F, ceil, kInnerWall);
            } else if (sp2.kind == 1) {
                skinX(m, xo, s, sp2.z0, sp2.z1, F, wb0, bands);
                skinX(m, xo, s, sp2.z0, sp2.z1, wb1, eave, bands);
                wallX(m, xi, -s, sp2.z0, sp2.z1, F, wb0, kInnerWall);
                wallX(m, xi, -s, sp2.z0, sp2.z1, wb1, ceil, kInnerWall);
                reveals(m, xo, xi, sp2.z0, sp2.z1, wb0, wb1, true, kInnerWall);
                for (int face = 0; face < 2; ++face) {   // the pane, seen from either side
                    const Real gx = s * (hw - 0.03) + (face ? -0.004 : 0.004) * s;
                    wallX(out.glass, gx, face ? -s : s, sp2.z0, sp2.z1, wb0, wb1, Vec3(0.55f, 0.62f, 0.66f));
                }
            } else {
                skinX(m, xo, s, sp2.z0, sp2.z1, F + out.doorHeight, eave, bands);
                wallX(m, xi, -s, sp2.z0, sp2.z1, F + out.doorHeight, ceil, kInnerWall);
                reveals(m, xo, xi, sp2.z0, sp2.z1, F, F + out.doorHeight, false, kInnerWall);
                MeshBuilder::emitBox(m, {std::min(xo, xi), F - 0.02, sp2.z0}, {std::max(xo, xi), F, sp2.z1}, kRubber);   // the threshold
            }
        }
        // the upper inner wall from ceiling to eave is hidden by the ceiling; the window band's reveals close it
    }

    // THE ENDS: a wall with the gangway through it, and half the diaphragm that bridges to the next car
    for (int e = -1; e <= 1; e += 2) {
        const Real zo = e * hl, zi = e * (hl - t);
        const Real gTop = F + 2.05;
        for (int face = 0; face < 2; ++face) {
            const Real z = face ? zi : zo, nz = face ? -e : e;
            const Vec3 col = face ? kInnerWall : kSilver;
            const Real top = face ? ceil : eave;
            wallZ(m, z, nz, -hw, -gw, sill, top, col);
            wallZ(m, z, nz, gw, hw, sill, top, col);
            wallZ(m, z, nz, -gw, gw, gTop, top, col);
            wallZ(m, z, nz, -gw, gw, sill, F, col);
            if (!face) {   // the end of the roof's chamfer, outside
                MeshBuilder::emitQuad(m, {-hw, eave, z}, {hw, eave, z}, {hw - 0.45, R, z}, {-hw + 0.45, R, z}, {0, 0, nz}, kSilver);
            }
        }
        // jambs of the gangway opening through the wall thickness
        const Real za = std::min(zo, zi), zb = std::max(zo, zi);
        MeshBuilder::emitQuad(m, {-gw, F, za}, {-gw, gTop, za}, {-gw, gTop, zb}, {-gw, F, zb}, {1, 0, 0}, kInnerWall);
        MeshBuilder::emitQuad(m, {gw, F, za}, {gw, gTop, za}, {gw, gTop, zb}, {gw, F, zb}, {-1, 0, 0}, kInnerWall);
        MeshBuilder::emitQuad(m, {-gw, gTop, za}, {gw, gTop, za}, {gw, gTop, zb}, {-gw, gTop, zb}, {0, -1, 0}, kInnerWall);
        // the diaphragm: a rubber tube from the end wall out to the coupling's middle, a floor plate in it
        const Real z1 = e * (hl + sp.gap / 2 + 0.02);
        const Real d0 = std::min(zo, z1), d1 = std::max(zo, z1), dw = gw + 0.12, dTop = gTop + 0.12;
        wallX(m, -dw, -1, d0, d1, F - 0.1, dTop, kRubber);
        wallX(m, dw, 1, d0, d1, F - 0.1, dTop, kRubber);
        wallX(m, -gw, 1, d0, d1, F, gTop, kRubber);
        wallX(m, gw, -1, d0, d1, F, gTop, kRubber);
        MeshBuilder::emitQuad(m, {-dw, dTop, d0}, {dw, dTop, d0}, {dw, dTop, d1}, {-dw, dTop, d1}, {0, 1, 0}, kRubber);
        MeshBuilder::emitQuad(m, {-gw, gTop, d0}, {gw, gTop, d0}, {gw, gTop, d1}, {-gw, gTop, d1}, {0, -1, 0}, kRubber);
        // the two halves overlap a little at the coupling: the front one 5 mm up, so they never share a plane
        const Real plate = F + (e > 0 ? 0.005f : 0.0f);
        MeshBuilder::emitQuad(m, {-gw, plate, d0}, {gw, plate, d0}, {gw, plate, d1}, {-gw, plate, d1}, {0, 1, 0}, kSeatFrame);
    }

    // SEATS
    const Real seatY = F + 0.45;
    auto seatAt = [&](Real x, Real z, int facing) {   // facing +1: looks toward +z
        MeshBuilder::emitBox(m, {x - 0.23, F + 0.38, z - 0.24}, {x + 0.23, seatY, z + 0.24}, kSeat);
        MeshBuilder::emitBox(m, {x - 0.23, F + 0.05, z - 0.04}, {x + 0.23, F + 0.38, z + 0.04}, kSeatFrame);
        const Real bz = z - facing * 0.26;
        MeshBuilder::emitBox(m, {x - 0.23, seatY, bz - 0.05}, {x + 0.23, seatY + 0.68, bz + 0.05}, kSeat);
        out.seats.push_back(Vec3(x, seatY, z));
        out.seatYaw.push_back(facing > 0 ? 0.0 : 3.14159265358979);
    };
    if (!longitudinal) {
        // transverse pairs either side of the aisle, in bays facing each other, between the vestibules
        const Real zA = -(hl - 2.0) + dh + 0.6, zB = (hl - 2.0) - dh - 0.6;
        int row = 0;
        for (Real z = zA + 0.35; z <= zB - 0.35; z += 0.95, ++row) {
            const int facing = (row % 2 == 0) ? +1 : -1;
            for (int s = -1; s <= 1; s += 2) {
                seatAt(s * (out.aisleHalf + 0.26), z, facing);
                seatAt(s * (out.aisleHalf + 0.76), z, facing);
            }
        }
        // luggage racks over the seats
        for (int s = -1; s <= 1; s += 2)
            MeshBuilder::emitBox(m, {s > 0 ? hw - t - 0.45 : -hw + t, wb1 + 0.25, zA}, {s > 0 ? hw - t : -hw + t + 0.45, wb1 + 0.29, zB}, kSeatFrame);
    } else {
        // benches along the walls between the doors, grab poles down the middle of each door's floor
        std::vector<Real> cuts{-hl + t};
        for (double dz : out.doorZ) { cuts.push_back(static_cast<Real>(dz) - dh - 0.1); cuts.push_back(static_cast<Real>(dz) + dh + 0.1); }
        cuts.push_back(hl - t);
        for (std::size_t k = 0; k + 1 < cuts.size(); k += 2) {
            const Real z0 = cuts[k] + 0.2, z1 = cuts[k + 1] - 0.2;
            if (z1 - z0 < 1.0) continue;
            for (int s = -1; s <= 1; s += 2) {
                const Real xin = s * (hw - t - 0.48), xw = s * (hw - t);
                MeshBuilder::emitBox(m, {std::min(xin, xw), F + 0.36, z0}, {std::max(xin, xw), seatY, z1}, kSeat);
                MeshBuilder::emitBox(m, {std::min(xin, xw), F + 0.05, z0 + 0.1}, {std::max(xin, xw), F + 0.36, z1 - 0.1}, kSeatFrame);
                MeshBuilder::emitBox(m, {s > 0 ? hw - t - 0.08 : -hw + t, seatY, z0}, {s > 0 ? hw - t : -hw + t + 0.08, wb0 + 0.1, z1}, kSeat);
                for (Real z = z0 + 0.3; z <= z1 - 0.3; z += 0.55) {
                    out.seats.push_back(Vec3(s * (hw - t - 0.28), seatY, z));
                    out.seatYaw.push_back(s > 0 ? -1.5707963267949 : 1.5707963267949);   // facing across the car
                }
            }
        }
        for (double dz : out.doorZ)
            for (int s = -1; s <= 1; s += 2) MeshBuilder::emitBox(m, {s * 0.5f - 0.025f, F, static_cast<Real>(dz) - 0.025f}, {s * 0.5f + 0.025f, ceil, static_cast<Real>(dz) + 0.025f}, kPole);
        for (int s = -1; s <= 1; s += 2) MeshBuilder::emitBox(m, {s * 0.6f - 0.02f, ceil - 0.25f, -hl + 1}, {s * 0.6f + 0.02f, ceil - 0.21f, hl - 1}, kPole);   // overhead rails
    }

    // THE DOOR LEAF: half an opening wide, a window in its upper half
    {
        RenderMesh& d = out.doorLeaf;
        const Real w = dh, h = out.doorHeight, th = 0.04;
        MeshBuilder::emitBox(d, {-th / 2, -h / 2, -w / 2}, {th / 2, h / 2, w / 2}, kSilver);
        for (int s = -1; s <= 1; s += 2)
            MeshBuilder::emitQuad(d, {s * (th / 2 + 0.003f), 0.05f, -w / 2 + 0.08f}, {s * (th / 2 + 0.003f), 0.05f, w / 2 - 0.08f},
                 {s * (th / 2 + 0.003f), h / 2 - 0.12f, w / 2 - 0.08f}, {s * (th / 2 + 0.003f), h / 2 - 0.12f, -w / 2 + 0.08f}, {static_cast<Real>(s), 0, 0}, kDarkGlass);
        wallX(d, th / 2 + 0.004f, 1, -w / 2, w / 2, -h / 2 + 0.55f, -h / 2 + 0.68f, sp.livery);
    }
    return out;
}

TrainBodyMesh buildTrainLocomotive(const TrainBodySpec& sp) {
    TrainBodyMesh out;
    RenderMesh& m = out.body;
    const Real L = sp.length, hl = L / 2, hw = sp.width / 2, sill = 0.95, R = sp.roof;
    out.length = L; out.width = sp.width; out.floor = sp.floor; out.walkable = false;
    // the side profile (z, y): flat roof, a raked windscreen, a short nose
    const Real zc = hl - 2.3, zw = hl - 0.75;   // the cab roof's front edge, the windscreen's foot
    const Real yw = 2.45, yn = 2.15;            // windscreen foot, nose top at the front face
    auto topAt = [&](Real z) {
        if (z <= zc) return R;
        if (z <= zw) return R + (yw - R) * (z - zc) / (zw - zc);
        return yw + (yn - yw) * (z - zw) / (hl - zw);
    };
    MeshBuilder::emitBox(m, {-hw, 0.6, -hl + 0.3}, {hw, sill, hl - 0.3}, kUnder);
    bogie(m, 0.30 * L);
    bogie(m, -0.30 * L);
    // the sides: grey below, silver above, a livery band sweeping up into the nose
    const Bands bands{1.45, 1.85, sp.livery};
    for (int s = -1; s <= 1; s += 2) {
        const Real x = s * hw;
        skinX(m, x, s, -hl, zc, sill, R, bands);
        // the cab side, under the raked roof line: strips 0.25 m long
        for (Real z = zc; z < hl - 1e-4; z += 0.25) {
            const Real z2 = std::min(hl, z + 0.25), t0 = topAt(z), t1 = topAt(z2);
            MeshBuilder::emitQuad(m, {x, sill, z}, {x, sill, z2}, {x, t1, z2}, {x, t0, z}, {static_cast<Real>(s), 0, 0}, sp.livery);
        }
        // grilles along the machine room, the cab's side window
        wallX(m, x + s * 0.005f, s, -hl + 2.0, zc - 2.5, 2.2, 3.4, kGrille);
        wallX(out.glass, x + s * 0.008f, s, zc + 0.15, zc + 1.0, 2.55, 3.35, Vec3(0.35f, 0.40f, 0.45f));
    }
    // the roof and its chamfers
    MeshBuilder::emitQuad(m, {-hw + 0.4, R + 0.12, -hl}, {hw - 0.4, R + 0.12, -hl}, {hw - 0.4, R + 0.12, zc}, {-hw + 0.4, R + 0.12, zc}, {0, 1, 0}, kSilver);
    for (int s = -1; s <= 1; s += 2)
        MeshBuilder::emitQuad(m, {s * hw, R, -hl}, {s * (hw - 0.4), R + 0.12, -hl}, {s * (hw - 0.4), R + 0.12, zc}, {s * hw, R, zc}, normalize(Vec3(s * 0.3, 1, 0)), kSilver);
    // the cab front: the raked windscreen (glass in a frame) and the nose
    for (Real z = zc; z < hl - 1e-4; z += 0.25) {
        const Real z2 = std::min(hl, z + 0.25), t0 = topAt(z), t1 = topAt(z2);
        const Vec3 n = normalize(Vec3(0, z2 - z, -(t1 - t0)));
        const bool screen = z >= zc - 1e-4 && z2 <= zw + 1e-4;
        MeshBuilder::emitQuad(m, {-hw, t0, z}, {hw, t0, z}, {hw, t1, z2}, {-hw, t1, z2}, n, screen ? kGrille : sp.livery);
        if (screen) MeshBuilder::emitQuad(out.glass, {-hw + 0.18f, t0 + 0.01f, z}, {hw - 0.18f, t0 + 0.01f, z}, {hw - 0.18f, t1 + 0.01f, z2}, {-hw + 0.18f, t1 + 0.01f, z2}, n, Vec3(0.30f, 0.36f, 0.42f));
    }
    // the roof's front edge, where the cab roof meets the windscreen
    MeshBuilder::emitQuad(m, {-hw + 0.4, R + 0.12, zc}, {hw - 0.4, R + 0.12, zc}, {hw, R, zc}, {-hw, R, zc}, {0, 0.3, 1}, kSilver);
    wallZ(m, hl, 1, -hw, hw, sill, yn, sp.livery);
    wallZ(m, hl + 0.005f, 1, -hw + 0.25, -hw + 0.65, 1.15, 1.35, kHeadlight);   // headlights
    wallZ(m, hl + 0.005f, 1, hw - 0.65, hw - 0.25, 1.15, 1.35, kHeadlight);
    MeshBuilder::emitBox(m, {-hw + 0.1, 0.55, hl - 0.1}, {hw - 0.1, sill + 0.1, hl + 0.25}, kGrille);   // the buffer beam
    // the back: a plain end, coupled to the first carriage
    wallZ(m, -hl, -1, -hw, hw, sill, R, kSilver);
    MeshBuilder::emitQuad(m, {-hw, R, -hl}, {hw, R, -hl}, {hw - 0.4, R + 0.12, -hl}, {-hw + 0.4, R + 0.12, -hl}, {0, 0, -1}, kSilver);
    // a pantograph on the roof, toward the back
    {
        const Real pz = -hl * 0.45, base = R + 0.12;
        MeshBuilder::emitBox(m, {-0.6, base, pz - 0.6}, {0.6, base + 0.18, pz + 0.6}, kGrille);
        MeshBuilder::emitBox(m, {-0.04, base + 0.18, pz - 0.5}, {0.04, base + 0.95, pz - 0.42}, kSeatFrame);
        MeshBuilder::emitBox(m, {-0.04, base + 0.9, pz - 0.5}, {0.04, base + 0.98, pz + 0.4}, kSeatFrame);
        MeshBuilder::emitBox(m, {-0.9, base + 0.98, pz + 0.35}, {0.9, base + 1.04, pz + 0.45}, kSeatFrame);
    }
    return out;
}

}  // namespace engine
