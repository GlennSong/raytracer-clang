#ifndef RAYTRACER_APPS_DRIVING_TRIP_PLACES_H
#define RAYTRACER_APPS_DRIVING_TRIP_PLACES_H

// WHERE TRIPS CAN BEGIN AND END (issue #255): for a host that hands out trips without a reason to go anywhere in
// particular (the traffic lab's drivers, who drive one trip after another), the places on the road a car can be put
// down and sent to, and the two steps of doing it.
//
// A trip only makes sense between places a car can drive between both ways. A destination down a dead end is one a
// car can reach and never leave; a road that nothing leads into is one it can leave and never come back to. So trips
// begin and end on the LANES of one PART of the road: a set of paths that can all reach one another
// (lane_graph.h reachableParts), counting only the paths the vehicle class may use and only the moves its routes may
// make (RouteClass::laneChanges: without lane changes a car keeps to the lanes its lane leads into, and the road falls
// apart into more, smaller parts). Never a connector: a place inside a junction is no place to start or stop. Every
// path of a part reaches every other, so a route from any place in a part to any other always exists.
//
// A part counts only when it has at least `smallestPart` lanes, so a car is never sent round a scrap of road. Each of
// its lanes offers the stretch clear of both its ends by a margin, and a place is picked by a number in [0, 1) spread
// over the total of those stretches: a longer lane is picked more often, in proportion.
//
// Same result every run: lanes and parts are in id order, and every pick comes from a stateless hash (src/hash.h) of
// numbers the host gives: a seed, the person, which question, and how many times that person has asked it.
//
// Units: metres.

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/driving.h"
#include "apps/driving/route.h"
#include "apps/driving/traffic_state.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace driving {

using engine::Real;

// "In no part": a path whose part has too few lanes, or that the class may not use.
constexpr uint32_t kNoTripPart = 0xFFFFFFFFu;

// ONE PART OF THE ROAD trips can use: its lanes in id order, and where each lane's usable stretch starts in the
// running total of all of them.
struct TripPart {
    std::vector<engine::PathId> lanes;
    std::vector<Real> before;   // per lane: the usable metres of the lanes before it in this part
    Real total = 0;             // the usable metres of all of them
};

// EVERY PLACE TRIPS CAN USE, for one vehicle class on one road.
struct TripPlaces {
    Real margin = 0;                     // how far a place is kept from either end of its lane
    std::vector<uint32_t> partOfPath;    // per path: its index in `parts`, or kNoTripPart
    std::vector<TripPart> parts;         // in order of their lowest path id
    std::vector<Real> partBefore;        // per part: the usable metres of the parts before it
    Real total = 0;                      // the usable metres of every part
    std::size_t laneCount = 0;           // how many lanes the parts hold between them

    bool empty() const { return total <= 0; }
    // The part `path` lies in, or kNoTripPart.
    uint32_t partOf(engine::PathId path) const;
};

// FINDS THE PLACES for class `cls` on `roads`: the parts of the paths it may use, following only the moves its routes
// may make, that hold at least `smallestPart` lanes with a usable stretch; each lane's stretch is its length less
// `margin` at each end (a lane shorter than twice the margin offers none and is left out).
TripPlaces findTripPlaces(const engine::LaneGraph& roads, const RouteClass& cls, Real margin, std::size_t smallestPart);

// A PLACE picked by `unit` in [0, 1) (clamped): anywhere in any part, or (placeInPart) in part `part` only. A place
// with no length to pick from (an empty part, unknown part) has an invalid path.
PathPoint placeAnywhere(const TripPlaces& places, Real unit);
PathPoint placeInPart(const TripPlaces& places, uint32_t part, Real unit);

// WHICH QUESTION a pick answers, so the same person's picks for different questions are independent.
enum class PlaceQuestion : uint8_t {
    Start,         // where a car is put down
    Destination,   // where a trip goes
};
// A NUMBER IN [0, 1) for one pick: a stateless hash of the seed, the person, the question and the how-manyth time this
// person asks it (`draw`, counted by the host). The same four always give the same number.
Real placeDraw(uint32_t seed, PersonId person, PlaceQuestion question, uint32_t draw);

// ROOM FOR A CAR whose front would be at `front`: its whole body (`length` metres back from the front) lies on that
// one path, `gap` metres clear of the path's start, `gap` short of its end, and no part of any car listed in `lists`
// lies within `gap` of it on that path.
bool roomFor(const engine::LaneGraph& roads, const PathLists& lists, const PathPoint& front, Real length, Real gap);

// WHAT THE TWO STEPS BELOW DID, counted for the host's report.
struct TripPlacing {
    bool done = false;     // the car was put down, or the trip begun
    uint32_t tries = 0;    // how many places were picked (each one a draw)
    uint32_t noRoute = 0;  // destinations refused for want of a route (each also queued a NoRoute event)
};

// PUT A CAR DOWN somewhere in `places` with room round it (roomFor, keeping `gap`), as class `vehicleClass`: picks
// places one after another (draws `draw`, `draw` + 1, ... for the Start question; `draw` is moved past those used)
// until one has room and Driving::placeCar takes it, at most `maxTries` picks. The car must not already be placed.
TripPlacing putCarSomewhere(Driving& driving, const TripPlaces& places, CarId car, uint32_t vehicleClass, Real gap,
                            uint32_t seed, PersonId person, uint32_t& draw, uint32_t maxTries);

// SEND A PLACED CAR SOMEWHERE: a trip for `person` to a place picked in the part the car's front is on, picking again
// (Destination question, draws counted as above) while the trip is refused, at most `maxTries` picks. Each refusal for
// want of a route is counted (and Driving queued a NoRoute event for it). Not done when the car is on no part, is not
// placed, or is on a trip.
TripPlacing sendCarSomewhere(Driving& driving, const TripPlaces& places, CarId car, PersonId person, uint32_t seed,
                             uint32_t& draw, uint32_t maxTries);

}  // namespace driving

#endif
