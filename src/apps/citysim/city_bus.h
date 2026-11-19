#ifndef RAYTRACER_APPS_CITYSIM_CITY_BUS_H
#define RAYTRACER_APPS_CITYSIM_CITY_BUS_H

// BUS ROUTES, PLANNED (Glenn: "we should make buses and taxis and Lyfts for npcs to get around the city").
//
// What is left of the old driving's buses (ADR-0460): the ROUTE PLANNER alone. It lays a city's bus loops over its
// street map, and the island planner (tools/city_plan.cpp) draws them on the island's map as its local bus lines.
// The buses that drove these loops, their riders, their depots and the riders' trip planning went with the old
// driving; nothing in the city sim reads this any more. It plans over the street map (engine::NavGraph), which #306
// task 4 deletes: a bus of the new driving plans its loop on the road model's lanes, so this goes with the street
// map unless the island planner has moved to the road model first.
//
// ROUTES ARE DERIVED, NOT AUTHORED. The city is procedural, so a route written
// into a level file would not survive the next regeneration: the nodes it named
// would move or vanish. A level asks for N routes of M stops and the network is
// built from the road graph, which means it is correct by construction on any
// city, including one that has never existed before.
//
// STOPS ARE EXISTING NAV NODES. Buses then stop where roads already go, no stop
// geometry has to be placed or collided, and a route leg is an ordinary trip the
// existing router already knows how to plan.

#include "../../engine/ai/nav_graph.h"   // routes FOLLOW STREETS, so the graph
#include "../../engine/procgen/city/polygon.h"   // engine::Vec2, engine::Real

#include <cstddef>
#include <cstdint>
#include <vector>

namespace citysim {

struct BusStop {
    int node = -1;              // nav node this stop sits on
    engine::Vec2 pos{0, 0};
    // Which visit of the route's pathNodes this stop is (the loop may pass a node twice), set when the
    // route is built: pathNodes[pathIndex - 1] -> node is the street the bus ARRIVES by. -1: not on the route's path.
    int pathIndex = -1;
};

// A closed loop. The bus drives stops in order and wraps, for ever.
struct BusRoute {
    std::vector<BusStop> stops;
    // The route as DRIVEN: every nav node between one stop and the next, so the
    // line on a map follows the streets instead of cutting across blocks. A
    // route that looks like a straight chord between scattered points is not a
    // bus route (Glenn: "those do not look like bus routes. Buses tend to stop
    // at consecutive streets").
    std::vector<engine::Vec2> path;
    std::vector<int> pathNodes;   // the same, as nav node ids -- an OPEN loop (the last node is not the first again)
    // How far round the loop each stop is (metres from the first path node), and the loop's length.
    std::vector<engine::Real> stopArc;
    engine::Real loopLength = 0;
    // Stops shared with another route: where a rider could change buses.
    std::vector<int> hubStops;
    // THE YARD: the stop furthest from the city's middle (a peripheral depot).
    int depotNode = -1;
    engine::Vec2 depotPos{0, 0};
    // Which street network the loop serves (BusNetwork::networkOf); -1 for the
    // REGIONAL route, which joins the networks by freeway and stops only at
    // their interchange stops. Its pace is its own (m/s): mostly freeway.
    int network = -1;
    bool regional = false;
    engine::Real pace = 0;   // 0 = a city bus's (kBusPace)
    // BOTH WAYS (Glenn: "Buses should probably go in both directions"): every loop has a twin driving it the
    // other way round, its stops in reverse, its legs re-found by car (one-way streets differ). The twin's route
    // index; -1 for none.
    int twin = -1;
    bool valid() const { return stops.size() >= 2; }
};

class BusNetwork {
public:
    // Lay out `routeCount` LOOPS over the street graph.
    //
    // A real bus route is a corridor, not a tour of scattered points: it runs
    // ALONG streets, stopping every few hundred metres, and it meets other
    // routes at hubs so a rider can change. So the network is built as:
    //   1. HUBS -- well-connected junctions, spread across the city. Every
    //      route passes through several, and routes SHARE them.
    //   2. each route is a closed loop over its own ordered subset of hubs,
    //      with the legs between them PATHFOUND over the road graph, so the
    //      line follows real streets.
    //   3. stops are dropped along that path at a regular spacing, plus one at
    //      every hub, which is why consecutive stops share a street.
    //
    // Deterministic in (graph, counts, seed): hub choice is farthest-point with
    // ties on the lower node index, and the legs come from the deterministic
    // A* the agents themselves use (ADR-0002).
    // `anchors`: places a loop must stop at -- the island's stations, where each one's footway leaves the street
    // (Glenn, 2026-10-10: "each station a bus line"). Each becomes a corner of one loop in its street network.
    void build(const engine::NavGraph& nav, int routeCount, int stopsPerRoute,
               uint32_t seed, const std::vector<engine::Vec2>& anchors = {});
    // How many of the anchors a built route stops at (of how many there were).
    int anchorsServed() const { return anchorsServed_; }
    int anchorCount() const { return anchorCount_; }
    // Why each anchor is or isn't served: "" served, else a reason (no street near it, a network too small for buses,
    // not a stop on any route).
    const std::vector<const char*>& anchorMisses() const { return anchorMisses_; }

    // The street network a nav node belongs to (junctions joined by walkable
    // streets; the city is one, each town out along the freeway another), or
    // -1. Two nodes in different networks can be joined only by a vehicle.
    int networkOf(int node) const {
        return node >= 0 && node < static_cast<int>(comp_.size()) ? comp_[static_cast<std::size_t>(node)] : -1;
    }

    // The hubs the network was built around, in world XZ.
    const std::vector<engine::Vec2>& hubs() const { return hubs_; }

    int routeCount() const { return static_cast<int>(routes_.size()); }
    const BusRoute& route(int r) const { return routes_[static_cast<std::size_t>(r)]; }
    bool empty() const { return routes_.empty(); }
    void clear();

private:
    void buildRegional(const engine::NavGraph& nav);
    int anchorsServed_ = 0, anchorCount_ = 0;
    std::vector<const char*> anchorMisses_;
    BusRoute reverseOf(const BusRoute& f, const engine::NavGraph& nav) const;
    std::vector<int> comp_;   // street network per nav node
    std::vector<BusRoute> routes_;
    std::vector<engine::Vec2> hubs_;
};

}  // namespace citysim

#endif
