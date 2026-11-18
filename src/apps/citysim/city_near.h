#ifndef RAYTRACER_APPS_CITYSIM_CITY_NEAR_H
#define RAYTRACER_APPS_CITYSIM_CITY_NEAR_H

// THE NEAR TIER'S CAR (ADR-0447; docs/two-drivers-meeting-design.md sections 4 and 7): one car, two ways of carrying out
// what its driver wants.
//
// A car's driver decides the same way wherever the car is (the new driving, apps/driving): the line to follow, the
// speed and acceleration it wants, and where it means to be at rest (driving::CarIntent). A LANE CAR is moved by the
// driving itself along its lane. A PHYSICS CAR is a real Jolt car (an engine::Vehicle on the same entity), and this file
// turns the intent into its steering, throttle and brake every physics step:
//   - steering by pure pursuit over its way's centre line (engine::LaneFollower, pursuitCommand, computeDriverInput),
//     through the player's steering assist (ADR-0087);
//   - speed by the error between the speed wanted and the speed it has, plus a pedal predicted from what its engine and
//     brakes can do (PhysicsWorld::launchAccel, brakeDecel: its mass and engine, measured);
//   - a stop by a braking curve that ends at the place, corrected each step against the distance left.
// Physics is the truth for a physics car: before each tick of the driving its real pose is projected onto its way and
// written into the driving's record (Driving::carriedAt), so every other driver sees it where it is. It is never pulled
// toward where the driving thinks it should be. The controller's lag is counted in its own safety margins
// (Driving::carry's `lag`).
//
// SWITCHING, on one entity, both ways (section 4): into physics the Jolt car is made at the pose it is drawn at, standing
// on its springs (PhysicsWorld::restHeight), moving at its speed with its wheels rolling (Vehicle::startMoving); back to
// the lane only when it is near its lane's centre, nearly pointing along it, upright and not a wreck, put at the nearest
// point along the lane, keeping its speed (VehicleSystem takes the Jolt car away: VehicleRetire).

#include "../../engine/ai/lane_follow.h"
#include "../../engine/system.h"
#include "../driving/driving.h"
#include "city_car.h"

#include "../../engine/components.h"   // CitySimConfig::NearTier
#include "../../engine/event_bus.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace engine {
class PhysicsSystem;
class AssetManager;
class ScriptVM;
struct VehicleSpec;
}  // namespace engine

namespace citysim {

class CityRenderSystem;

// THE CONTROLLER'S LAG (s): how much later than a lane car a physics car answers what its driver wants -- the decision
// passed on at the next physics step, the pedals' and the tyres' response. Counted in its reaction (Driving::carry), so
// it keeps room to stop behind a car that brakes. ESTIMATE, bounded by the test bed's mixed queue.
constexpr engine::Real kNearLag = 0.3;
// The braking the curve to a stop asks for, as a share of what the car's brakes can do (brakeDecel): room to correct.
constexpr engine::Real kStopCurveShare = 0.5;
// Within this of where it means to be at rest, and this slow, a physics car holds the brake (m, m/s).
constexpr engine::Real kStopHoldWithin = 0.15;
constexpr engine::Real kStopHoldBelow = 0.3;
// The pursuit's lookahead: this far ahead at a standstill, and this many seconds of its speed more (lane_follow.h).
constexpr engine::Real kNearLookMin = 4.0;
constexpr engine::Real kNearLookGain = 0.5;
// THE HAND-BACK (design section 4): a physics car becomes a lane car again only with its front within this of its
// lane's centre, pointing within this angle of it, its up axis within this angle of the sky, and its engine not dead.
constexpr engine::Real kBackOffset = 0.3;          // m
constexpr engine::Real kBackAlignDegrees = 6.0;
constexpr engine::Real kBackUprightDegrees = 12.0;
// A car the player left is given back to its household once it is slower than this (m/s): not while it rolls (#293).
constexpr engine::Real kGiveBackBelow = 0.5;
// How many touches between two of the city's Jolt cars are kept, each (NearCounts::contactList).
constexpr std::size_t kContactsKept = 32;
// `near marks on`: how high the post over each near car stands (m).
constexpr engine::Real kNearPostHeight = 4.0;

// A CAR OF THE NEAR TIER: on an entity with a CityCarBody and an engine::Vehicle while physics carries it.
struct CityNearCar {
    driving::CarId car;
    engine::Real halfLength = 0;   // the driving class's: its front bumper is this far ahead of its middle
    engine::Real launch = 0;       // what its engine pulls away at (m/s2, PhysicsWorld::launchAccel)
    engine::Real brake = 0;        // what its brakes stop it at (m/s2, PhysicsWorld::brakeDecel)
    // the line it follows: its way's centre line, from the path before its front's
    engine::LaneFollower follower;
    std::vector<engine::PathId> line;
    // how far it has gone since the driving's last tick (m): what the intent's distance to rest has shrunk by
    engine::Real sinceTick = 0;
    uint64_t tickSeen = 0;
};

// WHAT THE CONTROLLER ASKS OF THE CAR this step: the lane frame and speed for computeDriverInput (with its pedal
// prediction), from the car's middle, heading and forward speed, and what its driver wants. Pure but for the follower.
engine::DriverCommand nearCommand(CityNearCar& n, const engine::LaneGraph& roads, const driving::Driving& d,
                                  engine::Vec2 middle, engine::Vec2 forward, engine::Real speed, engine::Real dt);

// THE DRIVABLE SPEC OF A FLEET SLOT: its recipe from vehicles.lua in its paint (the car the player takes is built the
// same way: city_vehicles.cpp). False, with the reason, when the slot has none or the recipe fails.
// `vehicles`: a VM with vehicles.lua loaded already (openVehiclesLibrary), for many slots at once; null: one is made.
bool slotVehicleSpec(const CityRenderSystem& city, int slot, engine::VehicleSpec& spec, std::string* recipe,
                     std::string* why, engine::ScriptVM* vehicles = nullptr);
std::unique_ptr<engine::ScriptVM> openVehiclesLibrary(std::string* why);

// THE TEST BED'S ROUTES (ADR-0447): through a junction with traffic lights, by its connectors, the tightest turn
// first: from `runUp` metres before the lane's end into the junction, round the turn, to `runOn` metres along the lane
// after it. Each lane long enough for that. Empty when the road has no lights running.
struct LabRoute {
    driving::PathPoint start, to;
    engine::PathId approach, turn, exit;   // the lane in, the connector round the junction, the lane out
    engine::Real turnRadius = 0;           // the connector's tightest bend (m)
};
std::vector<LabRoute> labRoutes(const driving::Driving& d, engine::Real runUp, engine::Real runOn);
// The test bed's numbers (tests/test_near_tier.cpp and `near route|queue`): metres before the junction a car starts
// (the traffic lab's longest lane into a light is 69.5 m), metres along the lane out it is sent to, and the distance
// between the fronts of a queue's two cars.
constexpr engine::Real kLabRunUp = 60.0;
constexpr engine::Real kLabRunOn = 40.0;
constexpr engine::Real kLabQueueBehind = 14.0;

// ONE TOUCH BETWEEN TWO CARS OF THE CITY THAT ARE BOTH JOLT CARS (ADR-0448): who, where, how fast they closed.
struct NearContact {
    uint32_t a = 0, b = 0;   // their car numbers
    engine::Vec3 at;
    engine::Real speed = 0;
};

struct NearCounts {
    uint32_t intoPhysics = 0, intoLane = 0;
    uint32_t refused = 0;
    std::map<std::string, uint32_t> refusedBy;   // by reason
    // THE TIER (ADR-0448)
    int nearNow = 0;               // physics cars of the near tier now
    int mostThisStep = 0;          // the most switches made in one fixed step
    int mostOverCap = 0;           // the most near cars over the cap (cars that could not go back)
    int overCapNow = 0;
    uint32_t taken = 0, givenBack = 0;   // household cars the player took, and gave back (#293)
    uint32_t contacts = 0;         // new touches between two of the city's Jolt cars
    std::vector<NearContact> contactList;   // the first few, each
    double tierMs = 0;             // the tier's choosing and switching, summed (wall clock)
    uint64_t tierSteps = 0;
    double measureMs = 0;          // measuring the fleet slots' Jolt cars at load
    int slotsMeasured = 0;
    uint64_t written = 0;      // poses written into the driving
    uint64_t offRoad = 0;      // ticks a physics car's front was on no centre line of its way
    double controlMs = 0;      // the controller and the writing, summed (wall clock)
    uint64_t controlSteps = 0;
};

class CityNearSystem : public engine::System {
public:
    CityNearSystem(CityRenderSystem& city, engine::PhysicsSystem& physics) : city_(city), physics_(physics) {}

    void onStart(engine::FrameContext& ctx) override;        // the hook into the driving's tick, the `near` verbs
    void fixedUpdate(engine::FrameContext& ctx) override;    // each physics car's pedals and wheel
    void onStop(engine::FrameContext& ctx) override;
    void render(engine::FrameContext& ctx) override;   // `near marks on`: a post over each car of the near tier
    void update(engine::FrameContext& ctx) override;   // `near key`: the key to get in or out, pressed

    // INTO PHYSICS: the car on `e` (a CityCarBody whose car the driving has on a trip, drawn as a lane car) becomes a Jolt
    // car where it is drawn, at its speed. VehicleSystem makes the Jolt car at the end of the next fixed step and physics
    // first moves it the step after, so it is put `step` seconds of its speed ahead: the drawn car does not stand still
    // for a step. False, with the reason, when it cannot.
    bool toPhysics(engine::World& world, engine::AssetManager& assets, engine::Entity e, engine::Real step,
                   std::string* why = nullptr);
    // BACK TO THE LANE: only near its lane's centre, nearly aligned, upright and not a wreck. False, with the reason.
    bool toLane(engine::World& world, engine::Entity e, std::string* why = nullptr);

    // A LAB CAR (the test bed and the `near` verbs): an entity drawn as a household's car of fleet slot `slot`, number
    // `id`, driven as vehicle class `vehicleClass`, put on the driving's road with its front at `front` and, when `to`
    // is a place, sent there (a person of number kLabPerson drives it). Invalid, with the reason, when refused.
    engine::Entity labCar(engine::World& world, uint32_t id, int slot, uint32_t vehicleClass,
                          const driving::PathPoint& front, const driving::PathPoint& to, std::string* why = nullptr);
    static constexpr driving::PersonId kLabPerson = 0xFFFF0000u;

    const NearCounts& counts() const { return counts_; }

    // THE TEST BED IN THE VIEWER (`near route|queue|clear`): the test's route with a physics car, or its mixed queue (a
    // physics car behind a lane car), the cars standing until the route's light turns red and then sent, as the test
    // does. False, with the reason, when it cannot be set up.
    bool stageLab(engine::World& world, bool queue, std::string* why);
    void clearLab(engine::World& world);

private:
    struct Staged {
        engine::Entity entity;
        driving::PathPoint to;
        bool physics = false;
    };
    std::vector<Staged> staged_;        // lab cars waiting for the light
    std::vector<engine::Entity> lab_;   // every lab car (cleared by `near clear`)
    LabRoute stagedRoute_;
    driving::Lamp lastLamp_ = driving::Lamp::Dark;
    engine::AssetManager* assets_ = nullptr;
    uint32_t nextLabId_ = 1000;
    bool marks_ = false;   // `near marks on|off`
    bool pressEnter_ = false;   // `near key`: VehicleSystem's own get-in/get-out press on the next frame
    void sendStaged(engine::World& world, engine::Real step);
    CityRenderSystem& city_;
    engine::PhysicsSystem& physics_;
    engine::World* world_ = nullptr;
    NearCounts counts_;
    // what each fleet slot's Jolt car is built from and measured to do (made once a slot: each a small world of its
    // own), measured for every slot the level's cars use as it loads (measureSlots), so no switch waits on it
    struct SlotCar {
        engine::Real restHeight = 0, launch = 0, brake = 0;
        std::shared_ptr<engine::VehicleSpec> spec;
        std::string why;   // why it cannot be built (no spec)
    };
    std::map<int, SlotCar> slotCars_;
    const SlotCar& slotCar(int slot);
    std::shared_ptr<engine::ScriptVM> vehiclesVm_;   // vehicles.lua, loaded once for every slot's spec
    bool measured_ = false;
    void measureSlots(engine::World& world);
    // THE TIER (ADR-0448): the level's rules, read once
    bool rulesRead_ = false;
    engine::CitySimConfig::NearTier rules_;
    std::vector<engine::Entity> taken_;   // household cars the player has (to give back when left)
    void tierStep(engine::FrameContext& ctx);
    void refuse(const std::string& reason);
    engine::EventBus::Subscription onCollision_;   // touches between two of the city's Jolt cars
    void writePoses(driving::Driving& d);   // the hook: where each physics car really is
};

}  // namespace citysim

#endif
