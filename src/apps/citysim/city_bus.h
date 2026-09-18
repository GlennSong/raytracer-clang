#ifndef RAYTRACER_APPS_CITYSIM_CITY_BUS_H
#define RAYTRACER_APPS_CITYSIM_CITY_BUS_H

// BUSES (Glenn: "we should make buses and taxis and Lyfts for npcs to get
// around the city. Ideally a subway or train at some point. We would need to
// plan that since we don't have rail lines").
//
// A bus differs from a taxi in exactly one way that matters: it does not go
// where you ask. It drives a FIXED LOOP for ever, and you ride it if its loop
// happens to pass near where you are going. So the taxi's Dispatch (one hail,
// one driver, a destination chosen by the passenger) is the wrong shape, and
// this carries its own waiting lists instead. RideBook still does the actual
// carrying — that part is the same for every vehicle in the city.
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

#include "../../engine/procgen/city/polygon.h"   // engine::Vec2, engine::Real

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace citysim {

struct BusStop {
    int node = -1;              // nav node this stop sits on
    engine::Vec2 pos{0, 0};
};

// A closed loop. The bus drives stops in order and wraps, for ever.
struct BusRoute {
    std::vector<BusStop> stops;
    bool valid() const { return stops.size() >= 2; }
};

// Where a rider is in the system: which route they are waiting for / riding,
// and the stop index they want to get off at.
struct BusTrip {
    int route = -1;
    int fromStop = -1;
    int toStop = -1;
    // A rider KEEPS their trip once aboard -- the bus needs toStop to know
    // where to set them down -- but must drop off the waiting list, or the
    // next bus would try to pick up someone already sitting on one.
    bool aboard = false;
    bool valid() const { return route >= 0 && fromStop >= 0 && toStop >= 0; }
};

class BusNetwork {
public:
    // Lay out `routeCount` loops of up to `stopsPerRoute` stops over the given
    // node positions. Deterministic in (nodes, counts, seed): stops are chosen
    // by farthest-point sampling so they SPREAD instead of clumping, and ties
    // break on the lower node index, so the same city always gets the same
    // network (ADR-0002).
    void build(const std::vector<engine::Vec2>& nodes, int routeCount,
               int stopsPerRoute, uint32_t seed);

    int routeCount() const { return static_cast<int>(routes_.size()); }
    const BusRoute& route(int r) const { return routes_[static_cast<std::size_t>(r)]; }
    bool empty() const { return routes_.empty(); }
    void clear();

    // The nearest stop on `r` to `p`, as an index into that route's stops, or
    // -1 when the route is empty.
    int nearestStop(int r, engine::Vec2 p) const;

    // The best route to get from `from` to `to`: the one whose nearest stop to
    // each is closest overall, scored on the walking a rider would still have to
    // do at both ends. Returns an invalid trip when no route helps — which is
    // the common case for a short hop, and the caller should simply walk.
    // `maxWalk` bounds how far a rider will walk to or from a stop.
    BusTrip planTrip(engine::Vec2 from, engine::Vec2 to, engine::Real maxWalk) const;

    // --- who is waiting, and who is aboard --------------------------------
    // These are keyed by AGENT INDEX and deliberately sparse: a city of
    // thousands has a handful of bus riders, and Agent is not the place for it.
    bool waitFor(int passenger, const BusTrip& trip);
    void stopWaiting(int passenger);
    const BusTrip* tripOf(int passenger) const;

    // Everyone waiting for `route` at stop index `stop`, ascending by passenger
    // so a traversal is reproducible.
    std::vector<int> waitingAt(int route, int stop) const;
    // Riders ABOARD `route` who get off at `stop`, ascending.
    std::vector<int> alightingAt(int route, int stop) const;
    void markAboard(int passenger);
    // Done with the system entirely (set down, or gave up waiting).
    void forget(int passenger) { stopWaiting(passenger); }
    std::size_t waitingCount() const { return waiting_.size(); }

private:
    std::vector<BusRoute> routes_;
    std::unordered_map<int, BusTrip> waiting_;
};

}  // namespace citysim

#endif
