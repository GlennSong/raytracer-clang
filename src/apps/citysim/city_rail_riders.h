#ifndef RAYTRACER_APPS_CITYSIM_CITY_RAIL_RIDERS_H
#define RAYTRACER_APPS_CITYSIM_CITY_RAIL_RIDERS_H

// PEOPLE TAKING THE TRAIN (Glenn, 2026-10-11, after the stations got their ways in: "people choosing the train" --
// what makes the stations look used).
//
// The sim's view of the line: each station's way in pinned to a street node, which street network that node is in,
// and the stop each line makes there. A trip between two networks (two towns: no street joins them) goes by train
// when both have a station: to the station by the usual walk or bus, in along its footway, wait on the platform,
// ride, out along the far station's footway, and on by walk or bus. The legs on the street are the sim's own; this
// holds the part in between and who is in it.
//
// RailSim (rail_sim.h) runs the trains and is only READ here: a rider boards a train standing at their platform with
// its doors open, and steps off when the train stands at theirs. The footway itself is not in the nav graph, so the
// walk along it is a timed absence (a rider on the stairs is not drawn), the wait is a place on the platform.

#include "../../engine/ai/nav_graph.h"
#include "rail_sim.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace citysim {

struct RailStation {
    std::string name;
    engine::Vec2 access{0, 0};    // where its way in leaves the street
    int accessNode = -1;          // the street node there
    int network = -1;             // that node's street network
    std::vector<int> stopOnLine;  // per RailSim line: the index of its stop here, -1 if the line does not stop
    engine::Real walkIn = 60;     // seconds from the street to the platform along the way in
};

struct RailTrip {
    enum class Phase : uint8_t { ToStation, Entering, Waiting, Aboard, Leaving };
    int from = -1, to = -1;       // stations
    int line = -1, fromStop = -1, toStop = -1;
    Phase phase = Phase::ToStation;
    int train = -1;               // aboard: which train
    engine::Real timer = 0;       // Entering / Leaving: seconds left on the footway
    engine::Vec2 spot{0, 0};      // Waiting: where on the platform
    engine::Real top = 0;         // ...and its height
};

class RailRiders {
public:
    // The stations of `rail`'s lines (by name: each line stops at each), pinned to `nav`. Street networks are the
    // walkable links' components; a station's node is the nearest walkable street node to its way in (within
    // `reach`) in a network of at least `minNetworkStreet` metres -- the same rule the bus anchors use.
    void build(const RailSim* rail, const engine::NavGraph& nav, engine::Real reach = 120.0,
               engine::Real minNetworkStreet = 3000.0);
    void clear();
    bool active() const { return rail_ != nullptr && stations_.size() >= 2; }
    const RailSim* rail() const { return rail_; }

    int networkOf(int node) const {
        return node >= 0 && node < static_cast<int>(comp_.size()) ? comp_[static_cast<std::size_t>(node)] : -1;
    }
    const std::vector<RailStation>& stations() const { return stations_; }

    // A trip from street node `fromNode` to `toNode` by train: the pair of stations (one in each end's network) and the
    // line that make the quickest journey door to door. False when either network has no station or no line joins
    // them. `seconds`: the whole journey -- the walks at both ends, the ways in and out, the wait and the ride.
    bool plan(int fromNode, engine::Vec2 fromPos, int toNode, engine::Vec2 toPos, RailTrip& out,
              engine::Real* seconds = nullptr) const;

    // A place on station `st`'s platform for line `line`, the `k`-th of a spread of spots along its middle, at
    // the platform's height; `face` the way a waiting rider looks (at the track).
    bool platformSpot(int st, int line, int k, engine::Vec2& pos, engine::Real& top, engine::Vec2& face) const;

    // who is using it, keyed by agent index (sparse: a handful of thousands)
    RailTrip* tripOf(int passenger) {
        auto it = trips_.find(passenger);
        return it == trips_.end() ? nullptr : &it->second;
    }
    const RailTrip* tripOf(int passenger) const {
        auto it = trips_.find(passenger);
        return it == trips_.end() ? nullptr : &it->second;
    }
    void start(int passenger, const RailTrip& t) { trips_[passenger] = t; }
    void forget(int passenger) { trips_.erase(passenger); }
    // every rider, ascending by passenger (hash order must not reach the sim)
    std::vector<int> passengers() const;
    std::size_t count() const { return trips_.size(); }

    struct Stats {
        long planned = 0, entered = 0, boarded = 0, setDown = 0, gaveUp = 0;
    };
    Stats stats;

private:
    const RailSim* rail_ = nullptr;
    std::vector<int> comp_;   // street network per nav node
    std::vector<RailStation> stations_;
    std::unordered_map<int, RailTrip> trips_;
};

}  // namespace citysim

#endif
