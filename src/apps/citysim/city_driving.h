#ifndef RAYTRACER_APPS_CITYSIM_CITY_DRIVING_H
#define RAYTRACER_APPS_CITYSIM_CITY_DRIVING_H

// THE CITY SIM'S SIDE OF THE DRIVING (on a level that names citysim.drivingRules; issues #255, #258, #235).
//
// On a v2 level every car the new driving (driving::Driving) moves is a car entity (city_car.h; ADR-0444, ADR-0453):
// a household's car of the agent brain, a car that drives round for ever (the traffic lab's), or a test bed's car.
// This module is the hand-over: what the city does for those cars each tick of the driving.
//   - it reads the level's rule set (loadDrivingRules) and gives it to the model; a rule set with problems is
//     refused, every problem named with its file and key path, and the level then has no new driving;
//   - THE CARS THAT DRIVE ROUND FOR EVER (CityCarRoams) are its own to send: it puts each down on the lane graph,
//     with room round it, and sends it on a trip; when the car arrives it sends it on another, so the lab keeps moving
//     (driving/trip_places.h). A car stuck at the end of a road that just ends (the map's open edges) is put down
//     again where it can drive from, and counted. Nobody else's car is put down or sent anywhere here (the brain sends
//     its households' cars);
//   - after every tick it copies where each car entity the driving has is (driving::carPose: the middle of its body
//     and the way it points) into its Transform, with the lane graph's height under it (which the drawing uses to find
//     the car's own road among stacked decks), its speed and its brake lamps;
//   - it answers `driving?`.
//
// Same result every run: its cars are walked in the order of their numbers; every place comes from trip_places' hashes
// of the level's seed and the car's number; the clock is read only to report what a tick cost.

#include "../driving/driving.h"
#include "../driving/measure.h"
#include "../driving/trip_places.h"
#include "../../engine/world.h"
#include "../../rt_math.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace citysim {

struct CityCarRoams;
using engine::Real;

// HOW THE HAND-OVER PICKS PLACES (metres), named here because the rule set says how drivers drive, not where the lab
// puts its cars down.
// Room kept round a car put down when the level loads: every car is at rest then, so a little more than a standstill
// gap will do.
constexpr Real kStartRoom = 4.0;
// Room kept round a car put down again while traffic is moving: enough for a car at a freeway's speed behind it to
// stop braking hard (about 30 m/s at 7 m/s2 is 64 m).
constexpr Real kMovingRoom = 70.0;
// How far from either end of a lane a destination lies, so no trip ends inside a junction's mouth: at least this, and with
// room beyond (#264) at least the room the car's class needs (its body and a standstill gap), so a car stopping at its
// destination has no part of its body in a junction's box.
constexpr Real kPlaceMargin = 2.0;
// A part of the road counts for trips only with at least this many lanes: a car is never sent round a scrap.
constexpr std::size_t kSmallestTripPart = 8;
// Picks tried for one car before trying again on the next tick.
constexpr uint32_t kPlaceTries = 64;
// A car's brake lamps are lit while its acceleration is below minus this (m/s2): firmer than easing off the
// accelerator, and always when it is held at rest.
constexpr Real kBrakeLampDecel = 0.5;

// HOW THE BUSES RUN THEIR LINES (ADR-0483): assets/data/bus_service.json, read once (its comment says what each number
// is and where it came from). `usable` false: the file is missing or lacks a number, and no bus runs (the log says why).
struct BusService {
    bool usable = false;
    Real headwayMinutes = 0;
    int mostBusesPerLine = 0;
    Real dwellSeconds = 0;
};
const BusService& busService();

// WHAT THE HAND-OVER HAS DONE SO FAR.
struct DrivingCounts {
    uint32_t drivers = 0;          // cars that drive round for ever (CityCarRoams), the hand-over's own to send
    uint32_t placed = 0;           // of those, cars on the road now
    uint64_t tripsBegun = 0;
    uint64_t arrivals = 0;         // trips that ended at their destination (Parked events)
    uint64_t replaced = 0;         // cars put down again from the end of a road that just ends
    uint64_t routesNotFound = 0;   // destinations refused for want of a route (each picked again)
    uint64_t noTripYet = 0;        // times a car found no destination within kPlaceTries picks (tried again next tick)
    uint64_t noRoomYet = 0;        // times a car found no room to be put down within kPlaceTries picks
    uint32_t buses = 0;            // of the drivers, buses on their lines (CityBusRuns)
    uint64_t busCalls = 0;         // stops the buses have called at
};

class DrivingHost {
public:
    // Whether `begin` has run: the new driving runs here.
    bool active() const { return active_; }
    // BACK TO NOTHING HANDED OVER (issue #284): its cars that drive round for ever destroyed, no places, counts and
    // costs at zero, a fresh measurements table. CitySim::build calls it, as a rebuild makes a new city: cars kept from
    // the last build would be on the old one's driving. The host detaches the old measurements from its Driving first.
    void reset();

    // THE DRIVING BEGINS on the car entities of `world` (city_car.h), which it has from now on. Its own cars, every
    // entity with CityCarRoams (made by CitySim::startDrivingV2), are each put down and sent on a trip, in the order of
    // their numbers; picks are hashed from `seed`. Measurements are attached to `d`. Returns one line saying what was
    // done.
    std::string begin(driving::Driving& d, engine::World& world, uint32_t seed);

    // ONE TICK of the new model (timed, for the report), then the hand-over's own work: every car entity's
    // PrevTransform set to where it is as the tick begins (carTickBegins); after the model's tick, trips for its own
    // cars that arrived, its cars put down again from a road's end or put down at last, and every car entity the model
    // has on its road (with a CityCarOnLane) given the model's pose in its Transform (the middle of its body and the
    // way it points), the lane graph's height under it, its speed and its brake lamps.
    void tick(driving::Driving& d, Real dt);

    // THE EVENTS OF CARS THAT ARE NOT THE HOST'S OWN (the households' cars, a test bed's), since the last call, in the
    // order the model gave them. The host drains the model's events each tick; these are the ones that are not its own
    // to act on.
    std::vector<driving::Event> takeGuestEvents();
    // THE PLAYER TAKES ONE OF THE HOST'S OWN CARS (ADR-0453; the hand-over's half of CitySim::playerTakesCar, which is
    // the one way in, ADR-0455): its trip ended, it is off the model's road (a Jolt car of the near tier no longer
    // carried), and it no longer drives round for ever (no lane component, no CityCarRoams); CityCarLink::away: the
    // player's. False when `e` is not one of the host's own.
    bool takenByPlayer(driving::Driving& d, engine::Entity e);
    // Is the model's car `car` braking (its acceleration below -kBrakeLampDecel, or held at rest on its trip)?
    static bool braking(const driving::Driving& d, driving::CarId car);

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
    // The places trips can use for vehicle class `cls` (found the first time they are asked for).
    const driving::TripPlaces& placesFor(const driving::Driving& d, uint32_t cls);
    // THE HOST'S OWN CARS as they are now (every entity with CityCarRoams), by number, in the order of their numbers.
    using OwnCars = std::vector<std::pair<uint32_t, engine::Entity>>;
    OwnCars ownCars();
    // Put the car `e` down (with `room`), giving it its lane component and its pose; false when no room was found this
    // time.
    bool putDown(driving::Driving& d, engine::Entity e, Real room);
    // Send the car `e`, which the model has, on a trip; false when no destination was found this time.
    bool sendOn(driving::Driving& d, engine::Entity e);
    // Keeps `e` for takeGuestEvents when its car is not one of `own`; true when it did.
    bool keptForGuest(const driving::Event& e, const OwnCars& own);
    // The model's pose of every car entity on its road, copied in.
    void mirrorCarEntities(const driving::Driving& d);

    engine::World* carWorld_ = nullptr;        // where the car entities are (begin)
    std::vector<driving::Event> guestEvents_;  // the events of cars not the host's own, until taken
    bool active_ = false;
    uint32_t seed_ = 0;
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
