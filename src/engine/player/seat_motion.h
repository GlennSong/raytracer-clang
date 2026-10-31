#ifndef RAYTRACER_ENGINE_PLAYER_SEAT_MOTION_H
#define RAYTRACER_ENGINE_PLAYER_SEAT_MOTION_H

// WHAT A SEAT DOES TO WHOEVER IS IN IT (ADR-0252; every rider's since item 26, #117): the jolt it takes and the sway
// it throws the body into. The riding component carries both (Passenger: engine/riding.h rideStep), so the driver, a
// passenger in a cab, on a bus or in a train are all thrown against what their carrier did; the driver's seat
// (driver_seat.h) reads the jolt for who is thrown off.

#include "../../rt_math.h"
#include <vector>

namespace engine {

// THE JOLT over the last kWindow s: the most the seat's velocity changed, the vertical at half (the springs take a
// landing; a 2 m drop on a bike is a ride, not a crash), and the velocity it had before.
struct RideJolt {
    static constexpr Real kWindow = 0.3;
    struct Sample {
        Real t;
        Vec3 v;
    };
    std::vector<Sample> samples;
    Real clock = 0;
    Real step(const Vec3& velocity, Real dt, Vec3* before = nullptr);
    void reset() { samples.clear(); clock = 0; }
};

// THE BODY IN THE SEAT, on a spring: `lean` the shoulders' and `nod` the head's offset (m, the vehicle's frame), each
// thrown against the vehicle's change of velocity and pulled back. The seat back and the belt stop the torso at
// kLeanMax.
struct SeatSway {
    static constexpr Real kLeanMax = 0.32, kNodMax = 0.11;
    Vec3 lean, leanRate, nod, nodRate;
    // dv: the vehicle's change of velocity over this step, in its own frame
    void step(const Vec3& dv, Real dt);
    void reset() { *this = SeatSway{}; }
};

}  // namespace engine

#endif
