#ifndef RAYTRACER_APPS_CITYSIM_CITY_DRIVING_H
#define RAYTRACER_APPS_CITYSIM_CITY_DRIVING_H

// THE CITY SIM'S SIDE OF THE NEW DRIVING MODEL (citysim.driving = "v2"; issues #255, #258, #235).
//
// On a v2 level the city sim hands its drivers' cars to the driving library (driving::Driving) and stops driving them
// itself. This module is the hand-over and what the city sim does for those cars from then on:
//   - it reads the level's rule set (loadDrivingRules) and gives it to the model; a rule set with problems is
//     refused, every problem named with its file and key path, and the old model keeps the cars;
//   - it puts each driver's car down on the lane graph, with room round it, and sends it on a trip; when the car
//     arrives it sends it on another, so the lab keeps moving (driving/trip_places.h). A car stuck at the end of a road
//     that just ends (the map's open edges) is put down again where it can drive from, and counted;
//   - after every tick it copies where each car is (driving::carPose: the middle of its body and the way it points)
//     into its driver's Agent and its SimVehicle, which is what the drawing, the walkers' car checks, the map and the
//     camera read; and it keeps the lane graph's height under the car, which the drawing uses to find the car's own
//     road among stacked decks;
//   - it answers `driving?`.
// The old model is kept away from these drivers by CitySim (drivenByV2): they are not in its active list, its spatial
// grid or its lane occupancy, so it neither drives nor steers nor checks them for crashes nor queues behind them.
//
// Same result every run: cars are walked in car order (a car's id is its vehicle index); every place comes from
// trip_places' hashes of the level's seed and the driver's id; the clock is read only to report what a tick cost.

#include "../driving/driving.h"
#include "../driving/measure.h"
#include "../driving/trip_places.h"
#include "../../rt_math.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace citysim {

struct Agent;
struct SimVehicle;
using engine::Real;

// HOW THE HAND-OVER PICKS PLACES (metres), named here because the rule set says how drivers drive, not where the lab
// puts its cars down.
// Room kept round a car put down when the level loads: every car is at rest then, so a little more than a standstill
// gap will do.
constexpr Real kStartRoom = 4.0;
// Room kept round a car put down again while traffic is moving: enough for a car at a freeway's speed behind it to
// stop braking hard (about 30 m/s at 7 m/s2 is 64 m).
constexpr Real kMovingRoom = 70.0;
// How far from either end of a lane a destination lies, so no trip ends inside a junction's mouth.
constexpr Real kPlaceMargin = 2.0;
// A part of the road counts for trips only with at least this many lanes: a car is never sent round a scrap.
constexpr std::size_t kSmallestTripPart = 8;
// Picks tried for one car before trying again on the next tick.
constexpr uint32_t kPlaceTries = 64;
// A car's brake lamps are lit while its acceleration is below minus this (m/s2): firmer than easing off the
// accelerator, and always when it is held at rest.
constexpr Real kBrakeLampDecel = 0.5;

// WHAT THE HAND-OVER HAS DONE SO FAR.
struct DrivingCounts {
    uint32_t drivers = 0;          // agents whose cars the new model drives
    uint32_t placed = 0;           // of those, cars on the road now
    uint64_t tripsBegun = 0;
    uint64_t arrivals = 0;         // trips that ended at their destination (Parked events)
    uint64_t replaced = 0;         // cars put down again from the end of a road that just ends
    uint64_t routesNotFound = 0;   // destinations refused for want of a route (each picked again)
    uint64_t noTripYet = 0;        // times a car found no destination within kPlaceTries picks (tried again next tick)
    uint64_t noRoomYet = 0;        // times a car found no room to be put down within kPlaceTries picks
};

class DrivingHost {
public:
    // Whether `begin` has run: the new model drives some agents.
    bool active() const { return active_; }

    // HAND OVER: the new model drives the cars of `who` (agent indices, each a driver with a vehicle), each as the
    // driving class of its vehicle's fleet slot (`classOfSlot`: per slot, an index into d.vehicleClasses(), or -1 for
    // the first class). Each car is put down and sent on a trip; picks are hashed from `seed`. Measurements are
    // attached to `d`. Returns one line saying what was done.
    std::string begin(driving::Driving& d, std::vector<Agent>& agents, std::vector<SimVehicle>& vehicles,
                      const std::vector<int>& who, const std::vector<int>& classOfSlot, uint32_t seed);

    // ONE TICK of the new model (timed, for the report), then the hand-over's own work: trips for cars that arrived,
    // cars put down again from a road's end or put down at last, and every car's place copied into its agent.
    void tick(driving::Driving& d, Real dt, std::vector<Agent>& agents, std::vector<SimVehicle>& vehicles);

    // Does the new model drive agent `agent`'s car?
    bool drives(int agent) const {
        return agent >= 0 && agent < static_cast<int>(seatOfAgent_.size()) && seatOfAgent_[static_cast<std::size_t>(agent)] >= 0;
    }
    // The lane graph's height under the middle of the agent's car at the start of the last tick and at its end (for
    // the drawing, which blends the two across the tick). False when the new model does not drive the agent's car.
    bool roadHeight(int agent, Real& from, Real& to) const;
    // Is the agent's car braking (its acceleration below -kBrakeLampDecel, or held at rest on its trip)?
    bool braking(const driving::Driving& d, int agent) const;

    const DrivingCounts& counts() const { return counts_; }
    const driving::Measurements& measurements() const { return *measurements_; }
    // `driving?`: one line -- cars placed, moving, standing, trips, arrivals, cars put down again, routes not found,
    // mean speed and what the tick cost.
    std::string describe(const driving::Driving& d) const;

    // What a tick cost on the wall clock (reporting only, sums over `ticks`): the model's own tick, the measurements
    // watching it, and the hand-over's work after it.
    struct Cost {
        uint64_t ticks = 0;
        double modelMs = 0, modelWorstMs = 0;
        double watchMs = 0;
        double hostMs = 0, hostWorstMs = 0;
    };
    const Cost& cost() const { return cost_; }

private:
    struct Seat {
        int agent = -1;
        driving::CarId car;
        driving::PersonId person = 0;
        uint32_t vehicleClass = 0;
        uint32_t startDraws = 0;   // places picked to put the car down (trip_places' draw counts)
        uint32_t tripDraws = 0;    // ... and destinations picked
        bool placed = false;
        Real roadYFrom = 0, roadY = 0;   // the lane graph's height under the body's middle: tick start, tick end
    };

    // The places trips can use for vehicle class `cls` (found the first time they are asked for).
    const driving::TripPlaces& placesFor(const driving::Driving& d, uint32_t cls);
    // Put the seat's car down (with `room`) and send it on a trip; false when no room was found this time.
    bool putDown(driving::Driving& d, Seat& s, Real room);
    // Send the seat's placed car on a trip; false when no destination was found this time.
    bool sendOn(driving::Driving& d, Seat& s);
    // Copy the seat's car's place into its agent and vehicle; `snap` for a car just put down (drawn there at once).
    void mirror(const driving::Driving& d, Seat& s, std::vector<Agent>& agents, std::vector<SimVehicle>& vehicles,
                bool snap);

    bool active_ = false;
    uint32_t seed_ = 0;
    std::vector<Seat> seats_;          // in car order
    std::vector<int> seatOfAgent_;     // per agent: its seat, or -1
    std::vector<int> seatOfCar_;       // per car id (vehicle index): its seat, or -1
    std::vector<driving::TripPlaces> placesOfClass_;
    std::vector<uint8_t> placesFound_;
    std::unique_ptr<driving::Measurements> measurements_ = std::make_unique<driving::Measurements>();
    DrivingCounts counts_;
    Cost cost_;
};

// READS THE RULE SET `file` (a path under assets/scripts, as a level names it) and hands it to `d` with the facts the
// model knows and the move step's needs, bound to `d`'s road. False, with every problem in `why` (each naming the file
// and the key path), when the file is missing, does not run, fails a check, or does not fit the road; `d` is then left
// without rules. Needs scripting; without it, false.
bool loadDrivingRules(const std::string& file, driving::Driving& d, std::string* why);

}  // namespace citysim

#endif
