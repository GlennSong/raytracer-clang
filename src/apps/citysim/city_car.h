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
// THE CAR'S NUMBER is the same in the brain (its CarRef), in the driving (its CarId) and in the drawing. Car entities
// are numbered on from the last number in use (CitySim::nextCarNumber: past the sim's vehicle rows, which are only
// added while the city is built, and past every car entity), so a car's number is never a row's or another car's.
//
// A PERSON IN THE CAR is a sim agent on foot whose `inCar` names the car: they are not drawn on foot. The car is the
// authority on where they are: their own place and heading (Agent::pos, heading) follow the car's every tick, one way
// (CityBrain's tick, ADR-0445), so any reader of Agent::pos finds them where the car is.
//
// THE DRAWING (city_render.cpp) walks these entities and puts each into the city's instanced car groups, the same
// groups and the same pose functions as the sim's own cars: no draw call of its own. Those instances are what the
// kinematic box of every drawn car follows (city_physics.cpp).
//
// THE PLAYER'S CAR (ADR-0445): the player takes a household car with the key (CityVehicleSystem). The SAME entity gains
// an engine::Vehicle (VehicleSystem's: the Jolt car, its wheels, lamps and engine note) and loses its CityCarOnLane;
// from then VehicleSystem moves and draws it and nothing here does (carsNear, the bake, carTickBegins all pass over an
// entity with a Vehicle). Handing it back (removing the Vehicle) is not built (#293).
//
// A CAR THAT DRIVES ROUND FOR EVER (ADR-0453): the traffic lab's cars (citysim.cars on a level the new driving drives)
// are the same entities with nobody in them (CityCarLink: no household, no driver) and a CityCarRoams. Nobody in the
// city decides for them: the hand-over (DrivingHost) puts each down on the road and, every time it arrives, sends it
// on another trip, its picks hashed from the level's seed and the car's number, which is also the person number its
// trips are driven as. It has no kerb space, so while the driving does not have it (no room was found to put it down
// yet) it is nowhere: not drawn and not seen (carNowhere).
//
// EVERY SYSTEM THAT LOOKS FOR CARS asks carsNear: the old sim's driven cars and these, in one list.

#include "../driving/traffic_state.h"   // driving::CarId
#include "../../engine/components.h"    // engine::Transform, engine::PrevTransform
#include "../../engine/world.h"
#include "../../rt_math.h"

#include <cstdint>
#include <vector>

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
    bool away = false;   // the player has it as a car of their own (the brain's outsider; with the key, the entity
                         // also has an engine::Vehicle): not drawn here
};

struct CityCarOnLane {
    driving::CarId car;
    Real roadYFrom = 0, roadY = 0;   // the lane graph's height under the body's middle: as the tick began, and now
    Real speed = 0;                  // the driving's, after the last tick
    bool braking = false;            // its brake lamps are lit (city_driving.h kBrakeLampDecel)
    bool driven = false;             // on a trip (a car left standing in a lane is not)
    bool fresh = true;               // just put on the road: its road height is not blended from an earlier one
};

// DRIVES ROUND FOR EVER, sent on trip after trip by the hand-over (above). Only the hand-over's count of the places it
// has picked for the car (trip_places' draw counts), so each pick is a new one and every run makes the same.
struct CityCarRoams {
    uint32_t startDraws = 0;   // places picked to put the car down
    uint32_t tripDraws = 0;    // ...and destinations picked
};

// A car that drives round for ever which the driving does not have (no lane component, no physics car): nowhere.
bool carNowhere(engine::World& world, engine::Entity e);
// The car entity numbered `id` (CityCarBody::id), whoever's it is; none when there is no such car.
engine::Entity carEntityNumbered(engine::World& world, uint32_t id);

// The car's place and heading on the plan, from its Transform.
Vec2 carPosition(const engine::Transform& t);
Vec2 carHeading(const engine::Transform& t);
// Sets the car's place on the plan, its height and the way it points (`heading`, need not be unit; a zero one keeps
// the way it pointed).
void setCarPose(engine::Transform& t, Vec2 pos, Vec2 heading, Real y);
// THE SIM'S TICK BEGINS: every car's PrevTransform is where it is now (what the drawing blends from). A car the player
// holds (it has an engine::Vehicle) is VehicleSystem's and is passed over.
void carTickBegins(engine::World& world);

class CitySim;

// ONE CAR OF THE CITY AS A SYSTEM LOOKING FOR CARS SEES IT: where it is and which way it points, how fast it goes,
// whether it is on a trip, its body's length and width, and which kind of car it is: an old-sim car (`agent`: its
// driver, an agent in Mode::Driver; its pose is the agent's) or a household car (`entity`, its number `id`, and the
// agent at its wheel or -1). Its height is not here: each reader has its own ground question.
struct CarSeen {
    Vec2 pos, heading;
    Real speed = 0;
    bool moving = false;
    bool onRoad = true;   // a household car: the driving has it (CityCarOnLane); an old-sim car: always
    Real length = 0, width = 0;
    int agent = -1;
    engine::Entity entity;
    uint32_t id = 0;
    int driver = -1;
};
// THE CITY'S CARS WITHIN `radius` OF `p` (radius below zero: every one), appended to `out`, the old sim's first (in
// its near list's order: a far car has no body), then the households' (in the World's order):
//   - an old-sim car: an agent of the near list in Mode::Driver, not released to the player, not far;
//   - a car entity (a household's, or one that drives round for ever): an entity with CityCarBody, not away, not
//     nowhere (carNowhere) and with no engine::Vehicle (a car the player holds, or the near tier carries, is
//     VehicleSystem's, found as a Vehicle like any physical car: one car, one representation). Its speed and "on a
//     trip" are its CityCarOnLane's; a car off the road stands still.
// The caller applies its own further questions (a bus, a car on service, one the player rides) through `agent`.
void carsNear(const CitySim& sim, engine::World& world, Vec2 p, Real radius, std::vector<CarSeen>& out);

}  // namespace citysim

#endif
