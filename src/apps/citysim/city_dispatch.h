#ifndef RAYTRACER_APPS_CITYSIM_CITY_DISPATCH_H
#define RAYTRACER_APPS_CITYSIM_CITY_DISPATCH_H

// HAILING A RIDE (Glenn: "buses and taxis and Lyfts for npcs to get around the
// city", and later "that could make an interesting occupation to allow an npc
// or player to be a Lyft driver").
//
// RideBook (city_transit.h) knows who is ALREADY IN a vehicle. This knows who
// WANTS one and which driver is on the way. The two are deliberately separate:
// boarding is a fact, matching is a POLICY, and the policies differ — a taxi
// takes the nearest hail, a bus takes everyone at the next stop, a Lyft driver
// takes an offer they accepted, a player picks whoever they feel like. All of
// them end in the same RideBook::board.
//
// KNOWS NOTHING ABOUT THE CITY. `assign` takes the cost function from its
// caller rather than reaching for a nav graph, so this module has no dependency
// on CitySim, the graph, or the clock, and its tests need none of them either.
// That is also what lets a bus reuse it with a completely different cost.

#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

namespace citysim {

// One hail: who wants to go, from which nav node, to which.
struct Fare {
    int passenger = -1;
    int pickup = -1;
    int dropoff = -1;
    bool valid() const { return passenger >= 0 && pickup >= 0 && dropoff >= 0; }
};

class Dispatch {
public:
    // Queue a request. False when `passenger` is already waiting or already
    // assigned, or when the fare is malformed (a rider cannot hail twice).
    bool hail(int passenger, int pickup, int dropoff);
    // Give up waiting. Safe on someone who never hailed. A fare already picked
    // up by a driver is NOT cancellable here — that is an alight, not a cancel.
    void cancel(int passenger);

    // Match `driver` to the cheapest waiting hail, where `cost(pickup)` is
    // supplied by the caller (route length, straight-line distance, whatever
    // the caller can afford). A negative cost means unreachable and is skipped.
    // Returns the passenger taken, or -1 when nothing is waiting or reachable.
    //
    // TIES GO TO THE LOWER PASSENGER INDEX, and the scan runs in ascending
    // passenger order, so the same city in the same state always produces the
    // same assignment (ADR-0002 — determinism is compared bit-for-bit).
    int assign(int driver, const std::function<double(int pickup)>& cost);

    // The fare `driver` is currently serving, or nullptr when free.
    const Fare* fareOf(int driver) const;
    // The driver coming for `passenger`, or -1 when nobody is.
    int driverFor(int passenger) const;
    // Drop the assignment (the ride is over, or the driver gave up). The
    // passenger is NOT re-queued: a caller that wants that calls hail again.
    void complete(int driver);

    std::size_t waiting() const { return queue_.size(); }
    std::size_t assigned() const { return byDriver_.size(); }
    bool empty() const { return queue_.empty() && byDriver_.empty(); }
    void clear();

    // Everyone still waiting, ascending — for a deterministic traversal.
    std::vector<int> waitingPassengers() const;

private:
    std::vector<Fare> queue_;                  // kept sorted by passenger
    std::unordered_map<int, Fare> byDriver_;   // driver -> the fare it took
    std::unordered_map<int, int> driverOfPassenger_;
};

}  // namespace citysim

#endif
