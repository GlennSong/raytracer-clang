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
//   CityCarPose         WHERE ITS BODY IS THIS FIXED STEP (ADR-0454): its body on its wheels on the drawn deck under
//                       it (or, off the road, standing level on the ground), at the place between the tick's start and
//                       end this step stands for. Set once every fixed step after the sim's step (city_poses.h
//                       CityPoses::stepCarPoses), for every car entity that is not a Jolt car; the one pose the car's kinematic
//                       box (city_physics.cpp), the near tier's switch into physics (city_near.cpp toPhysics), the
//                       player's seats in it and the drawing all read.
//
// THE DRAWING (city_render.cpp) walks these entities and puts each into the city's instanced car groups, the same
// groups as the sim's own cars: no draw call of its own. It draws CityCarPose; the renderer blends one bake to the
// next for smooth frames, and that blend is drawing only.
//
//   CityCarKept         A HOUSEHOLD'S CAR ONLY: what the agent brain keeps about it (ADR-0455): where it stands as a
//                       place the brain plans with, which of the brain's people has it, and its slide between its kerb
//                       space and the lane. The brain keeps no record of its own beside it.
//
// THE PLAYER'S CAR (ADR-0445, ADR-0455): the player takes any car entity one way, CitySim::playerTakesCar (the key,
// CityVehicleSystem, and sitting in a car of the near tier both call it): the brain lets a household's car go, the
// hand-over one it sends, and CityCarLink::away says the player has it. With the key the SAME entity gains an
// engine::Vehicle (VehicleSystem's: the Jolt car, its wheels, lamps and engine note) and loses its CityCarOnLane; from
// then VehicleSystem moves and draws it and nothing here does (carsNear, the bake, carTickBegins all pass over an
// entity with a Vehicle). Left and stopped, a household's car loses its Vehicle (the near tier) and is given back one
// way, CitySim::playerLeftCar (#293).
//
// A CAR THAT DRIVES ROUND FOR EVER (ADR-0453): the traffic lab's cars (citysim.cars on a level the new driving drives)
// are the same entities with nobody in them (CityCarLink: no household, no driver) and a CityCarRoams. Nobody in the
// city decides for them: the hand-over (DrivingHost) puts each down on the road and, every time it arrives, sends it
// on another trip, its picks hashed from the level's seed and the car's number, which is also the person number its
// trips are driven as. It has no kerb space, so while the driving does not have it (no room was found to put it down
// yet) it is nowhere: not drawn and not seen (carNowhere).
//
// EVERY SYSTEM THAT LOOKS FOR CARS asks carsNear: the old sim's driven cars and these, in one list.

#include "brain/brain_record.h"         // brain::PlaceRef, brain::AgentId (CityCarKept)
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
    bool away = false;   // the player has it as a car of their own (taken with CitySim::playerTakesCar, whoever's it
                         // was; with the key, the entity also has an engine::Vehicle): not drawn here
};

// THE AGENT BRAIN'S KEEPING OF A HOUSEHOLD'S CAR (ADR-0455): on every household car, made with it by the brain
// (city_brain.cpp), and nowhere else. What only the brain knows about the car and every other component does not hold:
// where it stands as a place the brain plans with, which of the brain's people has it, and what it is doing between
// its kerb space and the lane. Its household, its driver and whether the player has it are CityCarLink's; whether the
// driving has it is CityCarOnLane's; its number is CityCarBody's. Its holder is IN it when the holder is a resident
// and that resident's sim agent is CityCarLink::driver; an outsider in it is CityCarLink::driver too.
enum class CityCarPhase : uint8_t {
    Standing,         // in a kerb space, or left standing in a lane
    WaitingForRoom,   // in its space, its driver aboard, waiting for the kerb lane to be clear
    EasingOut,        // placed in the kerb lane; drawn sliding out of the space
    OnRoad,           // the driving's
    EasingIn,         // out of the driving; drawn sliding into its space
};
struct CityCarKept {
    brain::PlaceRef standing;                 // where it stands, or last stood (not valid: an outsider has it)
    brain::AgentId heldBy = brain::kNoAgent;  // a resident, the brain's outsider (city_brain.cpp kOutsider), or nobody
    bool stranded = false;                    // left where no lane is beside it: nobody can drive it from there
    CityCarPhase phase = CityCarPhase::Standing;
    // the slide between its space and the lane (while EasingOut or EasingIn)
    Vec2 easeFromPos, easeFromHeading, easeToPos, easeToHeading;
    Real easeT = 0;
    // a trip to begin once it is in the lane: who drives it, from and to where
    bool tripWanted = false;
    brain::AgentId tripPerson = brain::kNoAgent;
    brain::PlaceRef tripFrom, tripTo;
};

struct CityCarOnLane {
    driving::CarId car;
    Real roadYFrom = 0, roadY = 0;   // the lane graph's height under the body's middle: as the tick began, and now
    Real speed = 0;                  // the driving's, after the last tick
    bool braking = false;            // its brake lamps are lit (city_driving.h kBrakeLampDecel)
    bool driven = false;             // on a trip (a car left standing in a lane is not)
    bool fresh = true;               // just put on the road: its road height is not blended from an earlier one
};

// WHERE THE BODY IS THIS FIXED STEP (above). Not the Transform: the Transform is the sim's record of the car (its place
// on the plan as the tick ends, PrevTransform as it began, the lane graph's height), which the brain, the hand-over and
// carsNear read; this is the body as it stands at this fixed step, derived from that record once, on the deck and
// tilted to its wheels.
struct CityCarPose {
    engine::Mat4 body;   // the body's frame: its middle on its wheels, local +Z forward
    Real road = 0;       // the height of the road it stands by (the lane graph's, blended to this step; 0 off the road)
};

// DRIVES ROUND FOR EVER, sent on trip after trip by the hand-over (above). Only the hand-over's count of the places it
// has picked for the car (trip_places' draw counts), so each pick is a new one and every run makes the same.
struct CityCarRoams {
    uint32_t startDraws = 0;   // places picked to put the car down
    uint32_t tripDraws = 0;    // ...and destinations picked
};

// A BUS ON ITS LINE (ADR-0483): one of the hand-over's own cars (it has a CityCarRoams too) whose trips are not picked
// at random: it is put down at a stop of its line (the road model's busStopLane) and drives to the next, stands there
// `dwell` seconds with its doors open, and drives on, round its line for ever. `line` indexes the road model's
// busLines; `stop` is the stop it is driving to (or standing at, while `standing`).
struct CityBusRuns {
    uint32_t line = 0;
    uint32_t stop = 0;
    bool standing = false;   // at `stop`, its doors open
    Real dwellLeft = 0;      // seconds left standing there
    uint64_t calls = 0;      // stops called at since it was put down
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
// whether it is on a trip, its body's length and width, and which car it is (`entity`, its number `id`, and the agent at
// its wheel or -1). Its height is not here: each reader has its own ground question.
struct CarSeen {
    Vec2 pos, heading;
    Real speed = 0;
    bool moving = false;
    bool onRoad = true;   // the driving has it (CityCarOnLane)
    Real length = 0, width = 0;
    engine::Entity entity;
    uint32_t id = 0;
    int driver = -1;
};
// THE CITY'S CARS WITHIN `radius` OF `p` (radius below zero: every one), appended to `out` in the World's order: every
// car entity (a household's, or one that drives round for ever) with CityCarBody, not away, not nowhere (carNowhere)
// and with no engine::Vehicle (a car the player holds, or the near tier carries, is VehicleSystem's, found as a Vehicle
// like any physical car: one car, one representation). Its speed and "on a trip" are its CityCarOnLane's; a car off the
// road stands still. (The old driving's cars were listed first until ADR-0461.)
void carsNear(engine::World& world, Vec2 p, Real radius, std::vector<CarSeen>& out);

}  // namespace citysim

#endif
