#ifndef RAYTRACER_ENGINE_PLAYER_SEAT_GEOMETRY_H
#define RAYTRACER_ENGINE_PLAYER_SEAT_GEOMETRY_H

// (Its own header so the Vehicle component can carry it without the body: engine/player/driver_seat.h poses it.)

#include "../../rt_math.h"

namespace engine {

// WHERE A RIDER'S HANDS AND FEET GO, from the vehicle's recipe (vehicles.lua `seat`), about the hip point. The
// left hand's and foot's; the right ones mirror them across x.
struct SeatGeometry {
    Vec3 grip{0.17, 0.38, 0.30};   // a car: the wheel at quarter to three, in reach of a turn (~0.3 m on, ~0.38 up)
    Vec3 foot{0.11, -0.30, 0.70};  // ...the feet on the pedals and the rest
    Real lean = -12.0;             // degrees the torso leans forward (-ve: back, a car seat's recline)
    bool wheel = true;             // a steering wheel (the hands turn round it), not bars
    bool pedals = false;           // a bicycle's cranks: the feet go round (`foot` is the crank's centre then)
    Real crank = 0.17;             // ...this far out
};

}  // namespace engine

#endif
