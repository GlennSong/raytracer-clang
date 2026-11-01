#ifndef RAYTRACER_ENGINE_PLAYER_DRIVER_SEAT_H
#define RAYTRACER_ENGINE_PLAYER_DRIVER_SEAT_H

// THE PLAYER IN THE DRIVER'S SEAT (ADR-0252): what a crash does to the one at the wheel, and how they sit.
//
// A crash is the seat's JOLT: how far its velocity changed over a moment (RideJolt), the impact rule's speed lost
// (ADR-0247) for a body strapped to a vehicle. An OPEN seat -- a bike's, a moped's, a convertible's -- throws its
// rider past what it can take (ejectTolerance), limp at the speed the vehicle had; a closed car keeps them in (but for
// the windscreen rule, on by default) and the body takes the hit in the seat instead: SeatSway, a spring that lurches
// the torso and snaps the head against what the vehicle did, and settles. The same spring leans you into the brakes.
//
// seatedPose is the body in the seat, in the vehicle's frame (the hip at the origin, +z ahead, +x its left): the
// hands on the wheel or the bars, the feet on the pedals or the pegs, the sway on top. Pure, for tests.

#include "../../rt_math.h"
#include "../block_body.h"
#include "seat_geometry.h"
#include "seat_motion.h"   // RideJolt, SeatSway (every rider's: the riding component carries them)

namespace engine {

// The jolt a seat takes before it throws its rider (m/s): a two-wheeler 7 (25 km/h gone in a moment, the bike rule
// of ADR-0235), a convertible 11 (belted in, but nothing over the head), a closed car never -- or 22 through the
// windscreen while that rule is on (vehicle.windscreen; on unless set false).
Real ejectTolerance(bool twoWheeler, bool openSeat, bool windscreen);
// How the rider leaves: on at most of the way the vehicle was going, up off the seat; a bike that went down (no hit)
// slides you off slower.
Vec3 ejectThrow(const Vec3& before, bool hit);


// The body in the seat (the vehicle's frame): the hip on the origin, sway on top. steer -1..1 turns the hands round
// the wheel (or the bars); crank is the pedals' turn (revolutions).
BodyPose seatedPose(const SeatGeometry& seat, const SeatSway& sway, Real steer, Real crank, Real scale);
// Where a hand (0 left, 1 right) and a foot are in the seat's frame, for tests and the pose.
Vec3 seatGrip(const SeatGeometry& seat, int side, Real steer);
Vec3 seatFoot(const SeatGeometry& seat, int side, Real crank);

}  // namespace engine

#endif
