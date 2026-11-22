#ifndef RAYTRACER_APPS_CITYSIM_CITY_BUSES_H
#define RAYTRACER_APPS_CITYSIM_CITY_BUSES_H

// THE PEOPLE ON THE BUSES (ADR-0484): the one place riding happens, for everyone the sim walks -- the agent brain's
// residents (its ride skill hands them in) and the sim's own walkers (who take a bus when it beats the walk).
//   - THE TIMETABLE: for each of the road model's bus lines, each stop's waiting place (where it joins the walks,
//     RoadModel::busStopWalk) and the seconds from it to the next stop (the bus's route there at the free-flow speed,
//     times the service's measured traffic factor, and a stop's wait). What the trip planner prices a bus by.
//   - THE RIDERS: who waits at which stop for which line, and who is aboard which bus, getting off where. Each tick
//     every bus standing at a stop (city_car.h CityBusRuns) takes on whoever waits there for its line, while it has room
//     (the service's capacity), and sets down whoever is getting off there; whoever is aboard is carried where the bus
//     is, out of sight.
// Same result every run: riders are kept in the order they began, buses are walked in the order of their numbers.

#include "trip_plan.h"   // BusTimetable
#include "../../engine/world.h"

#include <cstdint>
#include <vector>

namespace driving {
class Driving;
}
namespace engine {
struct RoadModel;
}

namespace citysim {

struct Agent;

class BusRiders {
public:
    // THE TIMETABLE from the road model's lines, for the fleet's "bus" class (none: no timetable, nobody rides), and the
    // world whose buses it watches. `openWalks`: the walks a walker may use (RouteWalker::open).
    void build(const engine::RoadModel& model, const driving::Driving& d, const std::vector<uint8_t>& openWalks,
               engine::World& world);
    void reset();
    const BusTimetable& timetable() const { return timetable_; }
    bool any() const { return !timetable_.lines.empty(); }

    // `agent` waits at stop `board` of `line` for its bus, to get off at `alight`. False: no such stops.
    bool wait(int agent, uint32_t line, uint32_t board, uint32_t alight);
    // `agent` is aboard a bus of `line` already (raised from far part-way through a ride), getting off at `alight`.
    bool aboard(int agent, uint32_t line, uint32_t alight);
    // `agent` waits for no bus and rides none any more: put down where they are, seen again.
    void cancel(int agent, std::vector<Agent>& agents);
    // Waiting or aboard?
    bool riding(int agent) const;

    // WHAT A TICK DID: an agent got on (boarded) or off (not boarded), in the order it happened.
    struct Event {
        int agent = -1;
        bool boarded = false;
    };
    std::vector<Event> tick(std::vector<Agent>& agents);

    struct Counts {
        uint64_t waits = 0, boardings = 0, alightings = 0, leftFull = 0;   // leftFull: times a full bus left someone
    };
    const Counts& counts() const { return counts_; }

private:
    struct Rider {
        int agent = -1;
        uint32_t line = 0, board = 0, alight = 0;
        bool onBoard = false;
        engine::Entity bus;   // aboard: theirs (none: raised from far aboard "a bus of the line")
    };
    std::vector<Rider> riders_;
    BusTimetable timetable_;
    engine::World* world_ = nullptr;
    Counts counts_;
};

}  // namespace citysim

#endif
