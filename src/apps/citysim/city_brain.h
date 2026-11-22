#ifndef RAYTRACER_APPS_CITYSIM_CITY_BRAIN_H
#define RAYTRACER_APPS_CITYSIM_CITY_BRAIN_H

// THE RESIDENTS WITH THE AGENT BRAIN (docs/agent-brain-design.md sections 7, 9 and 12; ADR-0441, ADR-0443): the city
// sim's side of the brain library (brain/). On a level that states MARKERS (named points beside streets, components.h
// CityMarker) and RESIDENTS (CityResident), each is driven by the brain's machine: they want to be at a marker, plan the trip with
// the trip planner (walk, car, skateboard, as travel.lua's rows and their traits allow), and carry it out with the
// sim's own walking and the new driving.
//
// Words: a PLACE is a handle the brain stores and hands back: here a marker, a kerb parking space of the road model,
// or a plan point where a car was left in a lane. A SKILL moves a body and reports by an event. The KERB LANE is the
// lane nearest the kerb; a car leaves its space into it and stops in it beside the space it is going to. A GUEST of
// the driving hand-over (city_driving.h) is a car the hand-over copies into its agent but does not send anywhere.
//
// WHAT IS HERE, each over something that already exists:
//   the world   the clock (the sim's seconds, in whole milliseconds); places (a marker is where the road model says
//               a person stands beside it: RoadModel::frontageOn or frontageAt, then sidewalkBeside); the kerb spaces
//               near a marker, nearest first on foot (kerbSpacesNear); THE ONE TABLE OF WHICH CAR IS IN WHICH SPACE,
//               who has the car, and which space is HELD for which driver (from the moment a plan chooses it until
//               they park there or drop the plan).
//   walking     the sim's own: the person is sent along the pavements (CitySim::sendAgentTo) and takes the last steps
//               off them straight (stepAgentToward, by the walk skill's stepsOffThePaths), at the way's speed and on
//               its board where the way is one (Agent::directedPace, directedRide). Reports `arrived`, or the walking
//               state's own failure event when there is no way on foot.
//   driving     the brain's DrivingSkill over driving::Driving, with the parts a real car needs round it: the person
//               gets in (Agent::inCar: they are in the car, not drawn on foot); when the kerb lane beside the space is
//               clear the car is placed in it and drawn easing out of the space, then the trip begins; when the driving
//               reports "parked" and the space is taken, the car leaves the driving and is drawn easing into the
//               space; the person gets out on the kerb side.
//   the cars    EACH HOUSEHOLD CAR IS ONE ENTITY of the engine's World (ADR-0444; city_car.h): its pose, its body,
//               its household and driver, and while the driving has it a CityCarOnLane. It is no row of the sim's
//               vehicles. The hand-over (city_driving.h) copies the driving's pose into it after each tick; the city
//               bake draws it in the instanced car groups.
//
//               Told to be somewhere else at the wheel, the trip is given a new end (Driving::changeDestination) or,
//               when the car has not left its space yet, sets off for the new end. A car that has waited for a clear
//               lane longer than its way's limit is given up (the brain's `blocked`), and stays in its space.
//
// THE RESIDENT IS THE BRAIN'S (ADR-0442): their sim agent is marked `brainOwned`, and the sim's day table never places
// them (CitySim::placeFromSchedule). Nothing here repairs where they are.
//
// SEVERAL RESIDENTS, HOUSEHOLDS AND CARS (ADR-0443; design sections 7 and 12). The level states a list of residents
// (each: a name, a home marker, traits, a day) and a list of households (each: its members and the cars they share).
// Nobody is assigned a car: a plan prices a car of the household only if it is free at that moment, from where it
// stands at that moment; whoever reaches it first takes it, and the others are told `vehicle_gone` at the kerb and
// plan again without it. A car stays where its last driver left it. Each person has a spot of their own at every
// marker (kStandApart along the sidewalk from the next person's), so two people at one marker do not stand in each
// other.
//
// SOMEBODY ELSE AND THE HOUSEHOLD'S CAR (ADR-0443). An OUTSIDER is anyone the brain does not own: the player, or a
// walker a director has taken over. An outsider gets into a household's car and out of it through this class (board,
// alight: what the possession channel calls); the player taking a car (CitySim::playerTakesCar, the one way: the key,
// which makes the same entity a physical Vehicle, ADR-0445, or a car of the near tier) is takenByPlayer, and giving it
// back (CitySim::playerLeftCar) is carLeftAt. While an outsider has a car it is not free and stands nowhere a resident
// could take it from. Its driver, if it had one, is stood on the kerb side and told `vehicle_gone`. Left again, it
// stands where it was left:
// in a kerb space when it is in one, otherwise at a point, put into the kerb lane beside that point so it can be
// driven away. A DIRECTOR HOLDING ONE OF THE BRAIN'S PEOPLE (the possession channel) is the brain's `possessed` state.
//
// INTERIM, said plainly: the car slides sideways between its space and the lane (real pulling out and parking come
// with lane changes); a car the player leaves at a point is drawn where it was left and driven from the kerb lane
// beside it; a household's car has no physics body of its own until the player takes it (only the kinematic box every
// drawn car instance gets, city_physics.cpp), and the near tier's while it is near; nothing here is saved.
//
// WHAT THE BRAIN KNOWS ABOUT A CAR IS ON THE CAR (ADR-0455): its CityCarKept component (city_car.h), beside its
// CityCarLink (household, driver, away). The brain holds only the cars' entities in the order they were made.
//
// Same result every run: nothing reads the wall clock, and the machine answers events in a fixed order.

#include "brain/brain.h"
#include "brain/brain_planner.h"   // PlanRoadCounts
#include "../../engine/world.h"   // engine::Entity: a household's car (ADR-0444)
#include "../../engine/procgen/city/polygon.h"
#include "../../engine/procgen/city/roads/road_model.h"
#include "../../rt_math.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace engine {
struct CityMarker;
struct CityResident;
struct CityHousehold;
}  // namespace engine

namespace citysim {

class CitySim;
using engine::Real;
using engine::Vec2;

// HOW THE GLUE BEHAVES (metres, seconds), named once.
// How long a car is drawn sliding between its space and the kerb lane.
constexpr Real kKerbEaseSeconds = 2.5;
// The kerb lane counts as clear for pulling out when no car's body is on its path from this far behind where the
// car's rear will be (a car arriving at a local street's 11 m/s and braking gently stops in about this), measured on
// back along every path leading into the lane where the lane itself begins nearer (brain_driving.h LaneRoom) ...
constexpr Real kPullOutRoomBehind = 30.0;
// ... to this far ahead of where its front will be.
constexpr Real kPullOutRoomAhead = 3.0;
// A car's front is put no nearer than this to the end of a lane path, so a car standing beside a space has no part in
// a junction (the hand-over's own margin for a place a trip may end, city_driving.h kPlaceMargin).
constexpr Real kKerbLaneEndMargin = 2.0;
// How far on foot from a marker a kerb space may be to be offered for it, and how many are looked at.
constexpr Real kSpaceReach = 400.0;
// (64: a crowd's homes share the spaces of their streets, and the ones a car cannot drive from are passed over, ADR-0446;
// it was 24)
constexpr std::size_t kSpacesLookedAt = 64;
// THE KERB SPACES OFFERED FOR A PLACE whose frontage is `f`: of the kSpacesLookedAt nearest it on foot within
// kSpaceReach (RoadModel::kerbSpacesNear), nearest first, those `keep` keeps (the caller's word that a car can stand
// there and be driven away). The one list both sides use: the brain offers a car its spaces from it (spacesFor; a
// household's car at the start takes the first free one), and the crowd maker holds a household's cars' spaces in it
// as it takes the home (city_render.cpp, makeCrowd's CrowdSpaces), so it never deals a household the brain must refuse.
std::vector<engine::KerbSpace> kerbSpacesOffered(const engine::RoadModel& model, const engine::RoadFrontage& f,
                                                 const std::function<bool(const engine::KerbSpace&)>& keep);
// Where a person stands to get into a car or after getting out: this far out from the car's side, on the kerb side.
constexpr Real kDoorStandOff = 0.6;
// A walk has arrived when the person is within this of where it ends.
constexpr Real kWalkArrivedWithin = 0.3;
// A MARKER IS DRAWN AS A RING AND A POST BESIDE IT. The ring, this wide, is round the spot where a person at the marker
// stands. The post stands kMarkerPostOffset from that spot, half-way between "away from the road" and "along the
// street": a debug line is drawn over everything, so a post in line with the camera and the person is drawn through
// them, and the two views the follow camera takes most (behind a person facing the road; behind a person walking along
// the sidewalk) both have a post on that diagonal well to one side.
constexpr Real kMarkerRingRadius = 0.7;
constexpr Real kMarkerPostOffset = 2.0;
// EACH PERSON'S OWN SPOT AT A MARKER (ADR-0443): this far along the sidewalk from the next person's (a little over a
// pair of shoulders), to one side and the other in turn: the first resident on the marker's own spot, the second
// kStandApart one way along the street, the third as far the other way, and so on.
constexpr Real kStandApart = 0.9;
// A CAR SOMEBODY ELSE LEFT IS "IN" A KERB SPACE when its middle is within this of where a car in that space stands
// (half a space's width: its body is then mostly inside the painted bay).
constexpr Real kLeftInSpaceWithin = 1.25;
// ...and is ON THE ROAD, to be driven away from the kerb lane beside it, when its middle is no farther than this
// beyond the kerb line (a wheel on the kerb still counts; a car in a field does not).
constexpr Real kLeftOnRoadWithin = 1.0;

// ONE MARKER, resolved against the road model.
struct BrainMarker {
    std::string name;
    std::string kind;                // engine::CityMarker::kind (empty: of no kind)
    engine::RoadFrontage frontage;   // its street, station and kerb
    Vec2 stand;                      // where a person at it stands: the middle of the sidewalk beside it
    Vec2 post;                       // where its post is drawn: kMarkerPostOffset from `stand`, beside the ring
    bool stated = false;             // by street, station and kerb (false: looked up from a plan point)
};

// WHAT HAS HAPPENED SO FAR, for a test and for `brain?`.
struct CityBrainCounts {
    uint32_t boarded = 0;          // times the person got into the car
    uint32_t pulledOut = 0;        // times the car was placed in the kerb lane from a space
    uint32_t waitedForRoom = 0;    // ticks a car waited in its space for the kerb lane to be clear
    uint32_t tripsBegun = 0;
    uint32_t parkedReports = 0;    // the driving's "parked" events for the resident's car
    uint32_t slidIn = 0;           // times the car left the driving for a space
    uint32_t alighted = 0;
    uint32_t walksBegun = 0, walksArrived = 0, walksBlocked = 0;
    uint32_t nextSpaces = 0;       // times a taken space was answered with another
    uint32_t redirected = 0;       // times a trip under way (or about to set off) was given a new end
    uint32_t otherPlaces = 0;      // times a person with no way to a place chose another of its kind (ADR-0446)
    // ADR-0443: somebody the brain does not own and a household's car
    uint32_t takenStanding = 0;    // times an outsider got into a household's car that stood empty
    uint32_t takenDriven = 0;      // times the player took one with its driver in it
    uint32_t leftAgain = 0;        // times an outsider left one somewhere
    uint32_t putOut = 0;           // drivers stood on the kerb side because their car was taken
    uint32_t held = 0, letGo = 0;  // times a director took one of the brain's people over, and let them go
};

// WHAT ITS TICKS COST on the wall clock (reporting only; nothing reads it back): how many, their sum and the worst.
struct CityBrainCost {
    uint64_t ticks = 0;
    double ms = 0, worstMs = 0;
};

// WHO HAS A HOUSEHOLD'S CAR, for a test and for the screen.
enum class CarHolder : uint8_t {
    Nobody,     // free: it stands where it was left
    Resident,   // one of the brain's people
    Outsider,   // the player, or a walker a director took over
};

// One leg of the plan, for drawing.
struct BrainPlanLeg {
    Vec2 from, to;
    engine::Vec3 colour;
    std::string way;
    bool underWay = false, done = false;
};

class CityBrain {
public:
    explicit CityBrain(CitySim& sim);
    ~CityBrain();
    CityBrain(const CityBrain&) = delete;
    CityBrain& operator=(const CityBrain&) = delete;

    // Reads travel.lua and brain.lua, resolves the markers, makes the residents and their households' cars, each car
    // an entity of `world` (city_car.h; the cars of a brain made before on that world are destroyed first). False,
    // with the reason in `report`: nothing is left behind. On success `report` says what was set up.
    // `drivableSpacesOnly` (ADR-0446; citysim.drivableSpacesOnly): offer only kerb spaces a car can drive away from
    // and back to (the kerb lane beside them in the largest part of the road its class drives).
    bool start(engine::World& world, const std::vector<engine::CityMarker>& markers,
               const std::vector<engine::CityResident>& residents, const std::vector<engine::CityHousehold>& households,
               const std::vector<int>& classOfSlot, std::string* report, bool drivableSpacesOnly = false);
    // One tick, after the driving's: reports in, skills stepped, the machine ticked, the day tables.
    void tick(Real dt);

    const std::vector<BrainMarker>& markers() const;
    int markerNamed(const std::string& name) const;   // -1: none

    // THE RESIDENTS, numbered in the order the level lists them (`who`; the first is 0, which is what every call
    // below means when it is given none).
    int residentCount() const;
    bool hasResident() const { return residentCount() > 0; }
    // A resident by name, or by number written as text ("2"). -1: none.
    int residentNamed(const std::string& nameOrNumber) const;
    std::string residentName(int who) const;
    std::string householdName(int who) const;
    // The members of `who`'s household, themselves included, in the order the level lists them.
    std::vector<int> householdOf(int who) const;
    int residentAgent(int who = 0) const;     // the sim's agent index (-1: none)
    int residentOfAgent(int agent) const;     // which resident a sim agent is (-1: none of them)
    // THE HOUSEHOLDS' CARS (ADR-0444): each is one entity (city_car.h), named here by its number (CityCarBody::id,
    // which is also its car id in the driving).
    // The number of the car `who` would plan with now: one of their household's (-1: it has none).
    int residentCar(int who = 0) const;
    // Every household car's number, in the order they were made.
    std::vector<int> cars() const;
    // The car's entity (not valid: no such car).
    engine::Entity carEntity(int car) const;
    // Where the car is and the way it points, on the plan. False: no such car, or the player has it as their own.
    bool carPose(int car, Vec2& pos, Vec2& heading) const;
    // Whose household a car is (the members), who has it now, and one line about it for the screen.
    std::vector<int> membersOfCar(int car) const;
    CarHolder carHolder(int car) const;
    std::string describeCar(int car) const;
    // Whether a resident sits in a household's car (Agent::inCar). Where they are is their agent's place either way:
    // in a car it follows the car's every tick (ADR-0445, #296).
    bool inCar(int who) const;

    // `brain? <who>`: the machine's line for a resident, with each place it names said in words.
    std::string describe(int who = 0) const;
    // A resident's plan as "walk>space s75 along 3, car>space s223 against 2, walk>work", and the name of the state
    // they are in.
    std::string planText(int who = 0) const;
    std::string stateName(int who = 0) const;
    // What they are said to be doing, on screen: the state's display name from the data (brain.lua `label`).
    std::string stateLabel(int who = 0) const;
    // The name of the marker they stand at (within kWalkArrivedWithin of their own spot at it); empty: none, or they
    // are in a car.
    std::string standingAt(int who = 0) const;
    // The marker they want to be at, as its place in markers() (-1: none).
    int wantedMarker(int who = 0) const;
    // Where `who` stands when they are at marker `marker`: their own spot (kStandApart).
    Vec2 standSpot(int who, int marker) const;
    // Why their last plan was chosen ("walk 9.3 min, car 7.2 min (...): car"), or why there was none ("no way there
    // (...)"). Empty: they have not planned yet.
    std::string planReason(int who = 0) const;
    // Every state a resident entered, as "indoors@0 walking@1500 ..." (milliseconds).
    std::string traceText(int who = 0) const;
    const std::vector<brain::Transition>& trace(int who = 0) const;
    const brain::Machine* machine() const;
    const CityBrainCounts& counts() const;
    // What the plans put on the roads: legs by car priced and chosen, and those on the freeway (brain_planner.h).
    brain::PlanRoadCounts planRoadCounts() const;
    const CityBrainCost& cost() const;
    brain::BrainTime now() const;
    std::vector<BrainPlanLeg> planLegs(int who = 0) const;

    // `goto <who> <marker>`: the resident wants to be there from now. It stops their day table (setDay starts it again).
    bool goTo(int who, const std::string& marker, std::string* why);
    bool goTo(const std::string& marker, std::string* why) { return goTo(0, marker, why); }
    // `brain trait <who> <name> on|off`: give or take a travel.lua trait (a board). It is used from their next plan,
    // and a person standing with no way there plans again at once.
    bool setTrait(int who, const std::string& name, bool on, std::string* why);
    bool setTrait(const std::string& name, bool on, std::string* why) { return setTrait(0, name, on, why); }
    // The day tables: everybody's, or one person's.
    void setDay(bool on);
    void setDay(int who, bool on);
    bool dayOn(int who = 0) const;

    // The kerb space the car `who` would plan with stands in (not valid: it is on the road, left at a point, or an
    // outsider has it), and its name.
    engine::KerbSpace carSpace(int who = 0) const;
    std::string carSpaceName(int who = 0) const;
    // The space held for a resident (not valid: none).
    engine::KerbSpace heldSpace(int who = 0) const;
    // ARE THE HOLDS SOUND (ADR-0443)? No two people hold one space; nobody holds a space another car stands in; a
    // person with no vehicle leg left in their plan holds none. False, with the fault in `why`.
    bool holdsSound(std::string* why) const;
    // TEST SUPPORT: another car is put in `space` (as far as the table of spaces knows; nothing is drawn).
    void occupyForTest(const engine::KerbSpace& space);

    // SOMEBODY THE BRAIN DOES NOT OWN AND A HOUSEHOLD'S CAR (ADR-0443, ADR-0444).
    // The household car nobody is in nearest `p`, within `radius` (-1: none): what a director means by "that one".
    int nearestFreeCar(Vec2 p, Real radius) const;
    // The sim agent `agent` (on foot; one of the brain's own people only while a director has them) gets into `car`,
    // which nobody is in: an outsider has it from now (the possession channel's `board`). False: they cannot.
    bool board(int agent, int car);
    // ...and gets out again, on the kerb side, leaving it where it stands, free (`alight`). False: they are in no
    // household's car an outsider has.
    bool alight(int agent);
    // THE PLAYER TAKES THE CAR, with whoever is at its wheel put out on the kerb side and told `vehicle_gone`: it is
    // the player's as a car of their own from now, not drawn here (CityCarLink::away). The brain's half of the one way a
    // car is taken, CitySim::playerTakesCar (the key, CityVehicleSystem::promote, ADR-0445; the near tier; ADR-0455),
    // called before the same entity gains its Vehicle. False: no such car, or an outsider has it already.
    bool takenByPlayer(int car);
    // The player's household car is handed back: the car stands at `pos`, pointing along `heading`, free. The brain's
    // half of CitySim::playerLeftCar (#293). False: `car` is no household car an outsider has, or it is still a
    // physical Vehicle (the near tier retires it first).
    bool carLeftAt(int car, Vec2 pos, Vec2 heading);
    // A director takes the sim agent `agent` over, or lets them go. False: the brain does not own that agent.
    bool directorHolds(int agent, bool on);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    friend struct Impl;
};

}  // namespace citysim

#endif
