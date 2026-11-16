#ifndef RAYTRACER_APPS_CITYSIM_CITY_BRAIN_H
#define RAYTRACER_APPS_CITYSIM_CITY_BRAIN_H

// THE RESIDENT WITH THE AGENT BRAIN (docs/agent-brain-design.md section 9; ADR-0441): the city sim's side of the brain
// library (brain/). On a level that states MARKERS (named points beside streets, components.h CityMarker) and one
// RESIDENT (CityResident), one person is driven by the brain's machine: they want to be at a marker, plan the trip with
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
//               off them straight (stepAgentToward, by the trip lab's stepsOffThePaths), at the way's speed and on
//               its board where the way is one (Agent::directedPace, directedRide). Reports `arrived`, or the walking
//               state's own failure event when there is no way on foot.
//   driving     the brain's DrivingSkill over driving::Driving, with the parts a real car needs round it: the person
//               gets in (boardVehicle) and is a guest of the hand-over; when the kerb lane beside the space is clear
//               the car is placed in it and drawn easing out of the space, then the trip begins; when the driving
//               reports "parked" and the space is taken, the car leaves the driving and is drawn easing into the
//               space; the person gets out on the kerb side (alightVehicle).
//
//               Told to be somewhere else at the wheel, the trip is given a new end (Driving::changeDestination) or,
//               when the car has not left its space yet, sets off for the new end. A car that has waited for a clear
//               lane longer than its way's limit is given up (the brain's `blocked`), and stays in its space.
//
// THE RESIDENT IS THE BRAIN'S (ADR-0442): their sim agent is marked `brainOwned`, and the sim's day table never places
// them (CitySim::placeFromSchedule). Nothing here repairs where they are.
//
// INTERIM, said plainly: the car slides sideways between its space and the lane (real pulling out and parking come
// with lane changes); one resident and one car; nothing here is saved.
//
// Same result every run: nothing reads the wall clock, and the machine answers events in a fixed order.

#include "brain/brain.h"
#include "../../engine/procgen/city/polygon.h"
#include "../../engine/procgen/city/roads/road_model.h"
#include "../../rt_math.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace engine {
struct CityMarker;
struct CityResident;
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
constexpr std::size_t kSpacesLookedAt = 24;
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

// ONE MARKER, resolved against the road model.
struct BrainMarker {
    std::string name;
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

    // Reads travel.lua and brain.lua, resolves the markers, makes the resident and their car. False, with the reason
    // in `report`: nothing is left behind. On success `report` says what was set up.
    bool start(const std::vector<engine::CityMarker>& markers, const engine::CityResident& resident,
               const std::vector<int>& classOfSlot, std::string* report);
    // One tick, after the driving's: reports in, skills stepped, the machine ticked, the day table.
    void tick(Real dt);

    const std::vector<BrainMarker>& markers() const;
    int markerNamed(const std::string& name) const;   // -1: none
    bool hasResident() const;
    int residentAgent() const;     // the sim's agent index (-1: none)
    int residentVehicle() const;   // the sim's vehicle index of their car (-1: none)

    // `brain?`: the machine's line for the resident, with each place it names said in words.
    std::string describe() const;
    // The resident's plan as "walk>space s75 along 3, car>space s223 against 2, walk>work", and the name of the state
    // they are in.
    std::string planText() const;
    std::string stateName() const;
    // What they are said to be doing, on screen: the state's display name from the data (brain.lua `label`).
    std::string stateLabel() const;
    // The name of the marker they stand at (within kWalkArrivedWithin of where a person at it stands); empty: none, or
    // they are in a car.
    std::string standingAt() const;
    // Every state the resident entered, as "indoors@0 walking@1500 ..." (milliseconds).
    std::string traceText() const;
    const std::vector<brain::Transition>& trace() const;
    const brain::Machine* machine() const;
    const CityBrainCounts& counts() const;
    brain::BrainTime now() const;
    std::vector<BrainPlanLeg> planLegs() const;

    // `goto <marker>`: the resident wants to be there from now. It stops the day table (setDay(true) starts it again).
    bool goTo(const std::string& marker, std::string* why);
    // `brain trait <name> on|off`: give or take a travel.lua trait (a board). It is used from their next plan.
    bool setTrait(const std::string& name, bool on, std::string* why);
    void setDay(bool on);
    bool dayOn() const;

    // The kerb space the resident's car stands in (not valid: it is on the road, or left in a lane), and its name.
    engine::KerbSpace carSpace() const;
    std::string carSpaceName() const;
    // The space held for the resident (not valid: none).
    engine::KerbSpace heldSpace() const;
    // TEST SUPPORT: another car is put in `space` (as far as the table of spaces knows; nothing is drawn).
    void occupyForTest(const engine::KerbSpace& space);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    friend struct Impl;
};

}  // namespace citysim

#endif
