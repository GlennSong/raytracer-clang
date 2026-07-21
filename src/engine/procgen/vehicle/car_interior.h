#ifndef RAYTRACER_ENGINE_PROCGEN_VEHICLE_CAR_INTERIOR_H
#define RAYTRACER_ENGINE_PROCGEN_VEHICLE_CAR_INTERIOR_H

#include "../../../renderer/renderer.h"
#include "../../../rt_math.h"

namespace engine {

// The cabin interior, built with the BUILDING GRAMMAR's Scope ops.
//
// This is the half of the vehicle the shape grammar is genuinely right for. A
// Scope is an oriented box you split/inset/emit (shape_grammar.h): exactly the
// tool for "divide a cabin into rows, a row into seats, a seat into cushion and
// backrest". It is the wrong tool for the car's SHELL — no loft, no sweep — and
// box-splitting a body is precisely how the old cars came out as cybertrucks.
// So the sweep makes the skin, the grammar fills the cavity, and they meet at
// one seam: the CabinSpace below, which is the hollow the sweep closed.
//
// Scope carries an arbitrary orthonormal frame rather than an AABB, and that is
// what makes a RAKED seatback expressible: the backrest is the cushion's scope
// with its up/forward axes rotated about X and re-anchored.
//
// Dimensions come from SAE J1100/J826 (see vehicle_classes.lua), so a driver
// built from the standard's 50th-percentile manikin actually fits.

struct CabinSpace {
    Real frontZ = 0;      // firewall, world z (greater than rearZ; car faces +Z)
    Real rearZ = 0;       // rear bulkhead
    Real halfWidth = 0;   // inner half-width at the belt
    Real floorY = 0;      // cabin floor, world y
    Real roofY = 0;       // headliner, world y
};

struct InteriorParams {
    int rows = 2;               // seat rows; 0 = fill by repeat (a bus)
    Real seatPitch = 0.86;      // SAE L50 couple distance, or bus seat pitch
    Real seatWidth = 0.50;
    Real consoleWidth = 0.24;
    Real backAngle = 25.0;      // SAE A40, degrees from vertical
    Real steerDiameter = 0.37;  // SAE W9
    Real columnRake = 24.0;     // degrees from vertical
    // This IS SAE H30 — the hip point's height above the heel — because the
    // occupant's hip lands on the cushion top. Authoring it as a "cushion
    // thickness" of 0.14 silently halved the driver's seat height and dropped
    // the feet through the floor pan.
    Real cushionHeight = 0.28;
    Real backHeight = 0.58;
    Real headrestHeight = 0.20;
    Real dashDepth = 0.34;
    Real dashHeight = 0.24;
    bool driverOnLeft = true;
    Vec3 trimColor{0.17, 0.17, 0.19};
    Vec3 seatColor{0.23, 0.21, 0.20};
};

// Build the cabin's furniture. `sgrpOut`, when set, receives the driver's seating
// reference point (SAE SgRP) — the anchor a seated occupant is posed from and
// the position published as the "driver_seat" attach.
RenderMesh buildCarInterior(const CabinSpace& cabin, const InteriorParams& p,
                            Vec3* sgrpOut = nullptr);

}  // namespace engine

#endif
