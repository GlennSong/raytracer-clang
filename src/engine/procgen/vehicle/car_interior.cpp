#include "car_interior.h"

#include "../city/shape_grammar.h"
#include "../../mesh_builder.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

// Local axis indices for splitScope, in this file's cabin frame.
constexpr int kRight = 0;
constexpr int kUp = 1;
constexpr int kAft = 2;      // NOTE: the cabin frame's third axis points REARWARD

// Rotate a scope's up/forward axes about its own right axis, and re-anchor at a
// given corner. This is the whole reason Scope carries an orthonormal frame
// instead of an AABB: a seatback is a box that LEANS, and without a rotatable
// frame the only expressible seat is an upright slab.
Scope rakeAbout(const Scope& s, Real degrees, const Vec3& anchor) {
    const Real a = degreesToRadians(degrees);
    const Real c = std::cos(a), sn = std::sin(a);
    Scope r = s;
    r.axis[kUp] = s.axis[kUp] * c + s.axis[kAft] * sn;
    r.axis[kAft] = s.axis[kAft] * c - s.axis[kUp] * sn;
    r.origin = anchor;
    return r;
}

void box(BuildingMesh& out, const Scope& s, const Vec3& color) {
    emitBox(out, s, PartId::Wall, color);
}

Real axisExtent(const Scope& s, int axis) {
    return axis == 0 ? s.size.x : (axis == 1 ? s.size.y : s.size.z);
}

// splitScope takes PROPORTIONS — it normalises by the sum of |sizes| and always
// fills the parent exactly (shape_grammar.cpp). Authoring a cabin in proportions
// is unusable, because every number the package gives us is a LENGTH: a 0.34 m
// dash and a 0.86 m couple distance are metres, not ratios. So convert here.
//
// Entries are metres; a NEGATIVE entry means "share whatever is left". Because
// the result is made to sum to the parent's extent, splitScope's normalisation
// becomes the identity and the metres survive. If the request overflows the
// parent, normalisation shrinks everything proportionally — which is the right
// failure: a cabin too short for two rows gets two squashed rows, not garbage.
std::vector<Scope> splitMetres(const Scope& s, int axis, std::vector<Real> parts) {
    const Real extent = axisExtent(s, axis);
    Real used = 0;
    int rests = 0;
    for (Real v : parts) {
        if (v < 0) ++rests;
        else used += v;
    }
    if (rests > 0) {
        const Real each = std::max(extent - used, Real(0)) / rests;
        for (Real& v : parts)
            if (v < 0) v = each;
    } else if (used > Real(1e-6) && std::fabs(used - extent) > Real(1e-6)) {
        // No remainder entry: scale to fill, so the caller's ratios are kept.
        for (Real& v : parts) v *= extent / used;
    }
    return splitScope(s, axis, parts);
}

// One seat: cushion, raked backrest, headrest. `s` is the seat's own volume,
// origin at its front-bottom-outboard corner with axis[kAft] running rearward.
void emitSeat(BuildingMesh& out, const Scope& s, const InteriorParams& p,
              Vec3* sgrpOut) {
    const Real cushionDepth = std::min(s.size.z * Real(0.55), Real(0.52));
    const std::vector<Scope> fb = splitMetres(s, kAft, {cushionDepth, -1});
    if (fb.empty()) return;

    // Cushion: the bottom slab of the front block, inset so it reads as a
    // cushion sitting in a frame rather than a full-width slab.
    const std::vector<Scope> cush = splitMetres(fb[0], kUp, {p.cushionHeight, -1});
    if (!cush.empty()) box(out, insetScope(cush[0], Real(0.02)), p.seatColor);

    if (fb.size() < 2) return;

    // Backrest: the rear block, raked back about the seat's right axis and
    // re-anchored on the cushion's rear top edge so it pivots there rather than
    // swinging through the cushion.
    Scope back = fb[1];
    back.size.z = std::min(back.size.z, Real(0.16));   // a backrest, not a block
    const Vec3 hinge = fb[0].corner(0, p.cushionHeight / std::max(fb[0].size.y, Real(1e-6)), 1);
    // Backrest plus headrest must fit UNDER the roof. The rear of a cabin sits
    // beneath the raked backlight, so a seat sized only against its own scope
    // pushed its headrest out through the rear glass.
    const Real headroom = std::max(s.size.y - p.cushionHeight, Real(0.1));
    Scope raked = rakeAbout(back, p.backAngle, hinge);
    raked.size.y = std::min(p.backHeight, headroom * Real(0.72));
    box(out, insetScope(raked, Real(0.015)), p.seatColor);

    // Headrest, on top of the raked frame — so it leans with the seat.
    Scope head = raked;
    head.origin = raked.corner(0, 1, 0);
    head.size.y = std::min(p.headrestHeight, headroom * Real(0.22));
    head.size.z = raked.size.z * Real(0.9);
    box(out, insetScope(head, Real(0.06)), p.seatColor);

    // SAE SgRP: the hip point, at the cushion's rear where the torso pivots.
    if (sgrpOut)
        *sgrpOut = fb[0].corner(0.5, p.cushionHeight / std::max(fb[0].size.y, Real(1e-6)), 0.85);
}

// The steering wheel: a lathed torus (the grammar's revolve op, the same call
// that makes columns and balusters), tipped to face the driver, plus spokes and
// a column. Low segment counts keep it in the faceted language of the body.
void emitSteeringWheel(RenderMesh& out, const Vec3& hub, const InteriorParams& p) {
    const Real R = p.steerDiameter * Real(0.5);
    const Real r = std::max(R * Real(0.11), Real(0.016));

    std::vector<Vec2> profile;
    const int kTube = 6;
    for (int i = 0; i <= kTube; ++i) {
        const Real th = Real(2) * PI * i / kTube;
        profile.push_back(Vec2(R + r * std::cos(th), r * std::sin(th)));
    }
    RenderMesh rim = latheMesh(Vec3(0, 0, 0), profile, 12, p.trimColor);

    // The lathe revolves about +Y, so the rim starts lying flat. Stand it up and
    // tip it back TOWARD THE SEAT.
    //
    // The sign matters and was wrong: +(90 - rake) leaves the rim's normal at
    // (0, sin rake, +cos rake), i.e. the top edge leaning toward the nose, away
    // from the driver. A real wheel faces up and BACK at the driver's chest, so
    // the normal wants -cos — hence rake - 90.
    Mat4 orient = Mat4::trs(hub,
                            Quat::fromAxisAngle(Vec3(1, 0, 0),
                                                degreesToRadians(p.columnRake - 90)),
                            Vec3(1, 1, 1));
    MeshBuilder::appendTransformed(out, rim, orient);

    // Spokes run from the HUB OUT TO THE RIM — one each, not full bars across.
    // The old version laid three bars of length 1.7R through the centre at 60
    // apart, which reads as a six-pointed asterisk, and tilted them by
    // +(90 - rake) — the opposite sign to the rim (rake - 90), so they leaned the
    // wrong way relative to the wheel they belonged to. A real wheel has a few
    // spokes reaching from the boss to the rim: here two at 3 and 9 o'clock and
    // one at 6 o'clock, the common three-spoke layout.
    //
    // The wheel's disc lies in its local XZ plane (the lathe revolves about Y),
    // so a spoke is a short bar along +X offset out to mid-radius, rolled about
    // the wheel axis (Y) to each clock position, then given the SAME tilt as the
    // rim so spokes and rim move together.
    const Quat tilt = Quat::fromAxisAngle(Vec3(1, 0, 0),
                                          degreesToRadians(p.columnRake - 90));
    RenderMesh spoke = MeshBuilder::box(Vec3(R * Real(0.92), Real(0.02), Real(0.03)));
    for (Vertex& v : spoke.vertices) {
        v.position.x += R * Real(0.5);   // inner end near the boss, outer at the rim
        v.color = p.trimColor;
    }
    for (const Real clock : {Real(0), Real(180), Real(270)}) {   // 3, 9, 6 o'clock
        const Quat q = tilt * Quat::fromAxisAngle(Vec3(0, 1, 0), degreesToRadians(clock));
        MeshBuilder::appendTransformed(out, spoke, Mat4::trs(hub, q, Vec3(1, 1, 1)));
    }

    // Centre boss: a short disc on the wheel axis, so the spokes meet something
    // instead of crossing in mid-air. cylinder() is built along +Y — the wheel's
    // own axis before the tilt — so it needs no extra rotation of its own.
    RenderMesh boss = MeshBuilder::cylinder(static_cast<float>(R * Real(0.22)),
                                            static_cast<float>(r * Real(1.4)), 12);
    for (Vertex& v : boss.vertices) v.color = p.trimColor;
    MeshBuilder::appendTransformed(out, boss, Mat4::trs(hub, tilt, Vec3(1, 1, 1)));
}

}  // namespace

RenderMesh buildBusInterior(const CabinSpace& cabin, const InteriorParams& p,
                            Vec3* driverSgrpOut, std::vector<Vec3>* seatsOut,
                            std::vector<Vec3>* doorsOut) {
    RenderMesh out;
    const Real len = cabin.frontZ - cabin.rearZ;
    const Real H = cabin.roofY - cabin.floorY;
    const Real hw = cabin.halfWidth;
    if (len <= Real(3.0) || H <= Real(1.2) || hw <= Real(0.6)) return out;

    // The saloon as a scope, third axis REARWARD like the car cabin, so every
    // placement below reads "x metres from the left wall, z metres back from
    // the windscreen".
    Scope cab;
    cab.origin = Vec3(-hw, cabin.floorY, cabin.frontZ);
    cab.axis[kRight] = Vec3(1, 0, 0);
    cab.axis[kUp] = Vec3(0, 1, 0);
    cab.axis[kAft] = Vec3(0, 0, -1);
    cab.size = Vec3(hw * 2, H, len);
    auto at = [&](Real x0, Real y0, Real z0, Real w, Real h, Real d) {
        Scope s = cab;
        s.origin = cab.origin + cab.axis[kRight] * x0 + cab.axis[kUp] * y0 +
                   cab.axis[kAft] * z0;
        s.size = Vec3(w, h, d);
        return s;
    };
    BuildingMesh bm;
    const Vec3 floorCol(0.30, 0.31, 0.33);
    const Vec3 poleCol(0.95, 0.74, 0.10);   // the stanchion yellow

    // Floor: a solid deck, so the saloon has one instead of the shell's pan.
    box(bm, at(0, 0, 0, hw * 2, Real(0.02), len), floorCol);

    // --- the driver's cab: dash, seat, partition ---------------------------
    const Real dashD = Real(0.50);
    const Real cabW = std::min(Real(0.95), hw * Real(0.85));
    box(bm, insetScope(at(0, 0, 0, cabW, Real(0.95), dashD), Real(0.02)), p.trimColor);
    // A console wing reaching toward the door: the ticket machine's shelf.
    box(bm, insetScope(at(cabW, 0, dashD * Real(0.2), Real(0.35), Real(1.05), Real(0.35)),
                       Real(0.02)), p.trimColor);
    Vec3 driverSgrp(0, 0, 0);
    const Real seatW = Real(0.52);
    const Scope driverSeat = at((cabW - seatW) * Real(0.5), 0, dashD + Real(0.35),
                                seatW, H, Real(0.70));
    emitSeat(bm, driverSeat, p, &driverSgrp);
    // The partition behind the driver: waist-high, glazed above in real life.
    const Real partZ = dashD + Real(0.35) + Real(0.72);
    box(bm, at(0, 0, partZ, cabW, Real(1.20), Real(0.04)), p.trimColor);

    // --- the saloon: pairs either side of a centre aisle -------------------
    const Real aisle = Real(0.62);
    const Real pairW = std::max(hw - aisle * Real(0.5), Real(0.5));
    const Real pitch = std::max(p.seatPitch, Real(0.62));
    const Real firstZ = partZ + Real(0.55);          // standing room by the door
    const Real benchD = Real(0.85);
    const Real lastZ = len - benchD - Real(0.10);
    // The middle door, kerb (right) side: no seats in front of it.
    const Real doorZ0 = len * Real(0.46), doorZ1 = len * Real(0.46) + Real(1.25);
    int row = 0;
    for (Real z = firstZ; z + pitch <= lastZ; z += pitch, ++row) {
        for (int side = 0; side < 2; ++side) {
            const bool kerbSide = side == 1;
            if (kerbSide && z + pitch > doorZ0 && z < doorZ1) continue;
            const Real x0 = kerbSide ? hw + aisle * Real(0.5) : Real(0);
            const std::vector<Scope> pair =
                splitMetres(at(x0, 0, z, pairW, H, pitch), kRight, {-1, -1});
            for (const Scope& seat : pair) {
                Vec3 hip;
                emitSeat(bm, insetScope(seat, Real(0.02)), p, &hip);
                if (seatsOut) seatsOut->push_back(hip);
            }
        }
        // Stanchions at the aisle edges every other row, floor to ceiling.
        if (row % 2 == 0)
            for (Real px : {hw - aisle * Real(0.5), hw + aisle * Real(0.5)})
                box(bm, at(px - Real(0.02), 0, z + Real(0.05), Real(0.04), H, Real(0.04)),
                    poleCol);
    }
    // The doors, as floor points just inside the kerb-side wall: the front
    // door beside the driver's partition, and the middle door's centre.
    if (doorsOut) {
        auto floorAt = [&](Real x0, Real z0) {
            return cab.origin + cab.axis[kRight] * x0 + cab.axis[kAft] * z0;
        };
        doorsOut->push_back(floorAt(hw * 2 - Real(0.45), dashD + Real(0.55)));
        doorsOut->push_back(floorAt(hw * 2 - Real(0.45), (doorZ0 + doorZ1) * Real(0.5)));
    }
    // Door poles, both doors.
    for (Real pz : {partZ + Real(0.25), doorZ0, doorZ1})
        box(bm, at(hw * 2 - Real(0.25), 0, pz, Real(0.04), H, Real(0.04)), poleCol);
    // The rear bench, full width.
    {
        const std::vector<Scope> bench =
            splitMetres(at(0, 0, len - benchD, hw * 2, H, benchD), kRight,
                        {-1, -1, -1, -1, -1});
        for (const Scope& seat : bench) {
            Vec3 hip;
            emitSeat(bm, insetScope(seat, Real(0.02)), p, &hip);
            if (seatsOut) seatsOut->push_back(hip);
        }
    }
    // Ceiling grab rails along both sides of the aisle.
    for (Real px : {hw - aisle * Real(0.5), hw + aisle * Real(0.5)})
        box(bm, at(px - Real(0.02), H - Real(0.16), partZ, Real(0.04), Real(0.04),
                   len - partZ - benchD),
            poleCol);

    out = bm.merged();

    // A bus wheel: bigger and much flatter than a car's.
    InteriorParams wp = p;
    wp.steerDiameter = std::max(p.steerDiameter, Real(0.50));
    wp.columnRake = std::max(p.columnRake, Real(55));
    const Vec3 hub(driverSgrp.x, driverSgrp.y + Real(0.33), driverSgrp.z + Real(0.46));
    emitSteeringWheel(out, hub, wp);

    if (driverSgrpOut) *driverSgrpOut = driverSgrp;
    return out;
}

RenderMesh buildCarInterior(const CabinSpace& cabin, const InteriorParams& p,
                            Vec3* sgrpOut, std::vector<Vec3>* seatsOut,
                            std::vector<Vec3>* doorsOut) {
    if (p.rows <= 0) return buildBusInterior(cabin, p, sgrpOut, seatsOut, doorsOut);
    RenderMesh out;
    const Real cabinLen = cabin.frontZ - cabin.rearZ;
    const Real cabinH = cabin.roofY - cabin.floorY;
    if (cabinLen <= Real(0.2) || cabinH <= Real(0.2) || cabin.halfWidth <= Real(0.1))
        return out;

    // The cabin cavity as a Scope. Its third axis points REARWARD, so splitting
    // along it reads front-to-back — dash, then front row, then the rest — the
    // order a person would describe the car in.
    Scope cab;
    cab.origin = Vec3(-cabin.halfWidth, cabin.floorY, cabin.frontZ);
    cab.axis[kRight] = Vec3(1, 0, 0);
    cab.axis[kUp] = Vec3(0, 1, 0);
    cab.axis[kAft] = Vec3(0, 0, -1);
    cab.size = Vec3(cabin.halfWidth * 2, cabinH, cabinLen);

    BuildingMesh bm;

    // dash | front row | remaining rows
    const Real dash = std::min(p.dashDepth, cabinLen * Real(0.30));
    const std::vector<Scope> bays = splitMetres(cab, kAft, {dash, p.seatPitch, -1});
    if (bays.empty()) return out;

    // --- dash bay ------------------------------------------------------------
    // knee bolster | fascia | cowl shelf (left empty — it is under the screen).
    const std::vector<Scope> dashStack =
        splitMetres(bays[0], kUp, {cabinH * Real(0.28), p.dashHeight, -1});
    if (dashStack.size() >= 2) {
        box(bm, insetScope(dashStack[0], Real(0.03)), p.trimColor);
        Scope fascia = insetScope(dashStack[1], Real(0.02));
        box(bm, fascia, p.trimColor);
        // Instrument binnacle: a raised block on the driver's half.
        const std::vector<Scope> half = splitScope(fascia, kRight, {1, 1});
        if (half.size() == 2) {
            Scope bin = half[p.driverOnLeft ? 0 : 1];
            bin.origin = bin.corner(0.18, 1, 0.1);
            bin.size = Vec3(bin.size.x * Real(0.5), Real(0.07), bin.size.z * Real(0.6));
            box(bm, bin, p.trimColor);
        }
    }

    // --- front row: seat | console | seat ------------------------------------
    Vec3 sgrp(0, cabin.floorY + Real(0.25), (cabin.frontZ + cabin.rearZ) * Real(0.5));
    if (bays.size() >= 2) {
        const Real sw = std::min(p.seatWidth, cabin.halfWidth * Real(0.85));
        const std::vector<Scope> row =
            splitMetres(bays[1], kRight, {sw, p.consoleWidth, sw});
        if (row.size() >= 3) {
            Vec3 driverSgrp, passSgrp;
            emitSeat(bm, row[0], p, p.driverOnLeft ? &driverSgrp : &passSgrp);
            emitSeat(bm, row[2], p, p.driverOnLeft ? &passSgrp : &driverSgrp);
            sgrp = driverSgrp;

            // Centre console with a shifter stub on it.
            const std::vector<Scope> con =
                splitMetres(row[1], kUp, {cabinH * Real(0.30), -1});
            if (!con.empty()) {
                box(bm, insetScope(con[0], Real(0.015)), p.trimColor);
                Scope shifter = con[0];
                shifter.origin = con[0].corner(0.5, 1, 0.25);
                shifter.size = Vec3(Real(0.05), Real(0.12), Real(0.05));
                box(bm, shifter, p.trimColor);
            }
        }
    }

    // --- rear rows -----------------------------------------------------------
    // `rows == 0` means "fill by repeat": one op turns the rest of the cabin
    // into a bus's worth of seating, which is the grammar earning its keep.
    if (bays.size() >= 3 && bays[2].size.z > Real(0.3)) {
        std::vector<Scope> rear;
        if (p.rows <= 0)
            rear = repeatScope(bays[2], kAft, p.seatPitch);
        else
            rear.push_back(bays[2]);
        for (const Scope& r : rear) {
            Scope bench = r;
            bench.size.z = std::min(bench.size.z, p.seatPitch);
            emitSeat(bm, bench, p, nullptr);
        }
    }

    out = bm.merged();

    // Steering wheel, hung ahead of and above the driver's hip point.
    const Real side = p.driverOnLeft ? Real(-1) : Real(1);
    const Vec3 hub(side * (cabin.halfWidth * Real(0.46)),
                   sgrp.y + Real(0.30),
                   sgrp.z + Real(0.44));
    emitSteeringWheel(out, hub, p);

    if (sgrpOut) *sgrpOut = sgrp;
    return out;
}

}  // namespace engine
