#ifndef RAYTRACER_ENGINE_PROCGEN_VEHICLE_OCCUPANT_H
#define RAYTRACER_ENGINE_PROCGEN_VEHICLE_OCCUPANT_H

#include "../../../renderer/renderer.h"
#include "../../../rt_math.h"

namespace engine {

// A SEATED occupant for a vehicle.
//
// Lives in the engine rather than citysim because VehicleSystem — which drives
// the PLAYER's car — is the first consumer, and the engine cannot depend on an
// app. citysim's walking `buildPersonMesh` stays where it is; this is its seated
// sibling, and the two deliberately share nothing but a visual language.
//
// Posed from the SAE J826 H-point manikin, the same standard the car's interior
// is packaged around (see vehicle_classes.lua), so the driver actually fits the
// seat instead of being a capsule parked near it. The standing walker's leg is a
// single box pitched at the hip, which cannot sit; sitting needs a knee, so the
// leg here is a thigh and a shin with the shin pivoting about wherever the thigh
// ended.
//
// ORIGIN IS THE HIP (the SAE seating reference point), not the body centre —
// that is exactly what the car generator publishes as its "driver_seat" attach,
// so placing an occupant is "put the hip on the hip point".

struct OccupantParams {
    // SAE J826, 50th percentile. The numbers every real car interior is built
    // around, so a driver posed from them lands where a driver belongs.
    Real thigh = 0.4315;
    Real lowerLeg = 0.4175;
    Real backAngle = 25.0;      // SAE A40, degrees the torso reclines
    Real thighRise = 8.0;       // degrees the knee sits above the hip
    // SAE H30: hip (SgRP) height above the heel — a sedan's is ~0.28 m. The shin
    // angle is SOLVED from this rather than hanging straight down, because a
    // vertical shin drops its full 0.42 m and puts the feet through the floor.
    // In a real car the feet are well FORWARD of the knees, which is exactly
    // what makes the vertical drop shorter than the bone.
    Real hipToHeel = 0.28;
    // Hip to the top of the head, seated and upright. A 50th-percentile adult is
    // ~0.90 m, but a low sedan's cabin cannot hold that above a 0.28 m seat, so
    // the default is trimmed to fit and callers with taller cabins can raise it.
    Real sittingHeight = 0.78;
    Real armReach = 72.0;       // degrees the upper arm swings forward to the wheel
    Real hipHalfWidth = 0.115;

    Vec3 shirt{0.30, 0.36, 0.46};
    Vec3 trousers{0.18, 0.19, 0.22};
    Vec3 skin{0.80, 0.62, 0.50};
};

RenderMesh buildSeatedOccupant(const OccupantParams& p = {});

}  // namespace engine

#endif
