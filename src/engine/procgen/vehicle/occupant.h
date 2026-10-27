#ifndef RAYTRACER_ENGINE_PROCGEN_VEHICLE_OCCUPANT_H
#define RAYTRACER_ENGINE_PROCGEN_VEHICLE_OCCUPANT_H

#include "../../../renderer/renderer.h"
#include "../../../rt_math.h"

namespace engine {

// A SEATED person: a vehicle's driver or rider, a bench sitter, a picnic on the grass.
//
// The jointed body (block_body.h) in the seat (ADR-0254): posed by driver_seat.h's seatedPose, the player's own
// seat rule, from the SAE J826 H-point manikin's numbers -- the standard the car's interior is packaged around (see
// vehicle_classes.lua), so the driver fits the seat. They say how far the back reclines, where the heels land and
// where the hands reach; the body is scaled down to the cabin's sitting height when that is short.
//
// ORIGIN IS THE HIP (the SAE seating reference point), not the body centre --
// that is exactly what the car generator publishes as its "driver_seat" attach,
// so placing an occupant is "put the hip on the hip point".
struct OccupantParams {
    // SAE J826, 50th percentile. The numbers every real car interior is built
    // around, so a driver posed from them lands where a driver belongs. (The
    // bones are the body's own: its thigh and shin are J826's to a centimetre.)
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
    // the default is trimmed to fit (the body is scaled down to it) and callers
    // with taller cabins can raise it.
    Real sittingHeight = 0.78;
    Real armReach = 72.0;       // degrees the arm swings forward from hanging: to the wheel, or the lap

    Vec3 shirt{0.30, 0.36, 0.46};
    Vec3 trousers{0.18, 0.19, 0.22};
    Vec3 skin{0.80, 0.62, 0.50};
};

// On a bench, a chair or a bus seat: sitting up, the hands in the lap.
OccupantParams chairOccupant();
// A car's passenger: the sedan's seat and headroom (the defaults), the hands in the lap.
OccupantParams carPassenger();
struct BodyPose;
// The body's pose in the seat (the hip at the origin), and that pose as a mesh in the outfit's colours.
// `scale` is the person's own size (1: the 1.8 m body); a short cabin scales them down further.
BodyPose seatedOccupantPose(const OccupantParams& p = {}, Real scale = 1);
RenderMesh buildSeatedOccupant(const OccupantParams& p = {});

}  // namespace engine

#endif
