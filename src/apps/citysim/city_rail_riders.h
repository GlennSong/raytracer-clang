#ifndef RAYTRACER_APPS_CITYSIM_CITY_RAIL_RIDERS_H
#define RAYTRACER_APPS_CITYSIM_CITY_RAIL_RIDERS_H

// PEOPLE TAKING THE TRAIN (Glenn, 2026-10-11, after the stations got their ways in: "people choosing the train" --
// what makes the stations look used). Rebuilt for the agent brain in the one-brain plan's step 4f: the brain plans a
// ride (walk to a station's way in, ride, walk on) from the transit timetable, and this carries the person between the
// two ways in, as BusRiders (city_buses.h) carries a bus rider between two stops.
//
// A ride: in along the station's footway (a timed absence: a rider on the stairs is not drawn), waiting on the
// platform (drawn, standing on it), aboard a train of the line (drawn on a seat by the train system), off when the
// train stands at their stop with its doors open, and out along the far station's footway to the street. Each getting
// on and getting off is reported, in order, as a RideEvent (city_buses.h); the brain posts them as `boarded` and
// `alighted`.
//
// RailSim (rail_sim.h) runs the trains and is only READ here.
//
// (A "street point" here is one of the sim's walk points, city_walk_points.h; a town is one part of the walks.)

#include "city_buses.h"   // RideEvent
#include "city_walk_points.h"
#include "rail_sim.h"
#include "trip_plan.h"    // TransitTimetable

#include <string>
#include <vector>

namespace citysim {

struct Agent;

struct RailStation {
    std::string name;
    engine::Vec2 access{0, 0};    // where its way in leaves the street
    int accessNode = -1;          // the street point there (the job search's door-to-door estimate)
    int network = -1;             // that point's part of the walks (its town)
    std::vector<int> stopOnLine;  // per RailSim line: the index of its stop here, -1 if the line does not stop
    engine::Real walkIn = 60;     // seconds from the street to the platform along the way in
};

struct TrainRide {
    enum class Phase : uint8_t { Entering, Waiting, Aboard, Leaving };
    int agent = -1;
    int line = -1, board = -1, alight = -1;   // the rail line and its stops (indices in RailLine::stops)
    int from = -1, to = -1;                   // the stations
    Phase phase = Phase::Entering;
    int train = -1;               // aboard: which train
    engine::Real timer = 0;       // Entering / Leaving: seconds left on the footway
    engine::Vec2 spot{0, 0};      // Waiting: where on the platform
    engine::Real top = 0;         // ...and its height
    int leftBy = -1;              // the last train that stood there full (counted once)
};

class TrainRiders {
public:
    // The stations of `rail`'s lines (by name), their ways in pinned to the street points `spots`: the nearest within
    // `reach` in a part of the walks of at least `minNetworkStreet` metres (or the largest part).
    void build(const RailSim* rail, const WalkPointMap& spots, engine::Real reach = 120.0,
               engine::Real minNetworkStreet = 3000.0);
    void clear();
    bool active() const { return rail_ != nullptr && stations_.size() >= 2; }
    const RailSim* rail() const { return rail_; }
    const std::vector<RailStation>& stations() const { return stations_; }

    // THE RAIL LINES AS THE PLANNER PRICES THEM, appended to `tt` (network "rail", `source` the rail line): each stop
    // waited for at its station's way in, the footway's seconds its access, the running time to the next stop at a
    // train's average pace plus the dwell, and the headway its trains keep (the line's lap over their number).
    void addTimetable(TransitTimetable& tt) const;

    // THE JOB SEARCH'S QUESTION: from street point `fromNode` (at `fromPos`) to `toNode`, by train door to door -- a
    // station in each end's part of the walks, the line between them that takes it quickest: the walks in a straight
    // line (a street's detour on them), the ways in and out, the mean wait and the ride. False: no line joins them.
    bool doorToDoor(int fromNode, engine::Vec2 fromPos, int toNode, engine::Vec2 toPos, engine::Real* seconds = nullptr) const;
    int networkOf(int node) const {
        return node >= 0 && node < static_cast<int>(comp_.size()) ? comp_[static_cast<std::size_t>(node)] : -1;
    }

    // A place on station `st`'s platform for line `line`, the `k`-th of a spread of spots along its middle, at the
    // platform's height; `face` the way a waiting rider looks (at the track).
    bool platformSpot(int st, int line, int k, engine::Vec2& pos, engine::Real& top, engine::Vec2& face) const;

    // `agent` goes in at stop `board` of rail line `line` for a train to stop `alight`. False: no such stops.
    bool wait(int agent, int line, int board, int alight);
    // `agent` is aboard a train of `line` already (raised from far part-way through a ride), getting off at `alight`:
    // on the line's train with the least running left to that stop.
    bool aboard(int agent, int line, int alight);
    // `agent` rides no train any more: put down on the street at the station they were in (or where they are), seen
    // again. No report follows.
    void cancel(int agent, std::vector<Agent>& agents);
    bool riding(int agent) const { return rideOf(agent) != nullptr; }
    const TrainRide* rideOf(int agent) const;
    std::vector<int> passengers() const;   // every rider, ascending
    std::size_t count() const { return rides_.size(); }

    // ONE TICK of `dt` seconds: the footways walked, riders put on the platform, onto trains standing at their stop
    // with the doors open (as many as the seats), off at theirs, out to the street. What it did, in order.
    std::vector<RideEvent> tick(engine::Real dt, std::vector<Agent>& agents);

    struct Counts {
        uint64_t waits = 0, boardings = 0, alightings = 0, leftFull = 0;   // leftFull: times a full train left someone
    };
    const Counts& counts() const { return counts_; }

private:
    int stationOf(int line, int stop) const;   // the station of a line's stop (-1: none)
    TrainRide* find(int agent);
    const RailSim* rail_ = nullptr;
    std::vector<int> comp_;   // the part of the walks per street point
    std::vector<RailStation> stations_;
    std::vector<TrainRide> rides_;   // ascending by agent
    Counts counts_;
};

}  // namespace citysim

#endif
