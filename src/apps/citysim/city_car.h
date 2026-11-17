#ifndef RAYTRACER_APPS_CITYSIM_CITY_CAR_H
#define RAYTRACER_APPS_CITYSIM_CITY_CAR_H

// A HOUSEHOLD'S CAR IS ONE ENTITY (ADR-0444; docs/two-drivers-meeting-design.md sections 2 and 4, sitting 1a).
//
// A car of the agent brain's households (city_brain.h) is one entity in the engine's World (the sparse-set ECS,
// ADR-0006), made when the brain makes its cars. It is not a row of the city sim's vehicle list, and nothing else
// stands in for it: where it is, what it is, whose it is and whether it is on the road are its components, and every
// reader asks them.
//
//   engine::Transform   where it is: the middle of its body on the plan (x, z), the lane graph's height under it
//                       while it is on the road (y), and the way it points (a turn about the vertical; its local +Z
//                       is forward). PrevTransform is where it was when the sim's tick began (carTickBegins), so it
//                       is drawn between the two like any moving entity.
//   CityCarBody         what it is: the city's number for it, its fleet slot (the body it is drawn as), the body's
//                       kind, length, width and height, and the driving's vehicle class it is driven as.
//   CityCarLink         whose it is: its household, and the sim agent sitting at its wheel (-1: nobody).
//   CityCarOnLane       ON THE ROAD: present exactly while the new driving (driving::Driving) has the car, naming its
//                       car id there. The driving moves it then, and the hand-over (city_driving.h DrivingHost) copies
//                       the driving's pose into the Transform after each tick. A parked car has none: it keeps its
//                       pose and its kerb space (the brain's table of spaces) and nothing moves it.
//
// THE CAR'S NUMBER is the same in the brain (its CarRef), in the driving (its CarId) and in the drawing. Cars of the
// households are numbered on from the last row of the sim's vehicle list as it stands when the brain starts (rows are
// only added while the city is built, before that), so a household car's number is never a row's.
//
// A PERSON IN THE CAR is a sim agent on foot whose `inCar` names the car: they are not drawn on foot, and where they
// are is the car's pose (CityBrain::personPosition). Their own position stays where they got in.
//
// THE DRAWING (city_render.cpp) walks these entities and puts each into the city's instanced car groups, the same
// groups and the same pose functions as the sim's own cars: no draw call of its own.
//
// NOT HERE YET (sitting 1b): a physics body of the entity's own (today it has only the kinematic box city_physics.cpp
// gives every drawn car instance), and the player taking one with the key.

#include "../driving/traffic_state.h"   // driving::CarId
#include "../../engine/components.h"    // engine::Transform, engine::PrevTransform
#include "../../engine/world.h"
#include "../../rt_math.h"

#include <cstdint>

namespace citysim {

enum class VehicleType : uint8_t;   // (city_sim.h)
using engine::Real;
using engine::Vec2;

struct CityCarBody {
    uint32_t id = 0;                // the car's number: the brain's CarRef, the driving's CarId
    int slot = -1;                  // its fleet slot: the body it is drawn as (CitySim::fleetBody)
    VehicleType type{};             // the body's kind
    Real length = 0, width = 0, height = 0;
    uint32_t vehicleClass = 0;      // driven as this class of the driving's table
};

struct CityCarLink {
    int household = -1;
    int driver = -1;     // the sim agent sitting at its wheel (-1: nobody)
    bool away = false;   // the player has it as a physical car of their own: not drawn here (the brain's carLeftAt
                         // brings it back)
};

struct CityCarOnLane {
    driving::CarId car;
    Real roadYFrom = 0, roadY = 0;   // the lane graph's height under the body's middle: as the tick began, and now
    Real speed = 0;                  // the driving's, after the last tick
    bool braking = false;            // its brake lamps are lit (city_driving.h kBrakeLampDecel)
    bool driven = false;             // on a trip (a car left standing in a lane is not)
    bool fresh = true;               // just put on the road: its road height is not blended from an earlier one
};

// The car's place and heading on the plan, from its Transform.
Vec2 carPosition(const engine::Transform& t);
Vec2 carHeading(const engine::Transform& t);
// Sets the car's place on the plan, its height and the way it points (`heading`, need not be unit; a zero one keeps
// the way it pointed).
void setCarPose(engine::Transform& t, Vec2 pos, Vec2 heading, Real y);
// THE SIM'S TICK BEGINS: every car's PrevTransform is where it is now (what the drawing blends from).
void carTickBegins(engine::World& world);

}  // namespace citysim

#endif
