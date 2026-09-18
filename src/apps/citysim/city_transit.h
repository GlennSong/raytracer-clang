#ifndef RAYTRACER_APPS_CITYSIM_CITY_TRANSIT_H
#define RAYTRACER_APPS_CITYSIM_CITY_TRANSIT_H

// CARRYING PEOPLE (Glenn, 2026-09-17: buses, taxis, Lyfts, and "a player could
// pick up npcs and deliver them elsewhere... or they could hitch a ride").
//
// All of those are ONE mechanism with different matching policies: a passenger
// is attached to whoever is driving, stops moving under its own power, and is
// detached at a destination. A taxi picks its passenger from a request queue; a
// bus picks everyone waiting at a stop; the player picks from an offer; a
// hitching player is the same thing with the roles swapped. So the primitive
// lives here, alone, and the policies are built on top of it.
//
// DELIBERATELY NOT IN THE AGENT STRUCT. The relation is sparse -- a handful of
// riders in a city of thousands -- and Agent is already 432 bytes that several
// whole-population passes walk every tick. The book owns the mapping; the sim
// asks it. That also keeps the dependency one-directional: this module knows
// nothing about CitySim.

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace citysim {

class RideBook {
public:
    // Board `passenger` into the vehicle driven by `driver` (both are agent
    // indices). False when either is already engaged, or when they are the
    // same agent -- a driver cannot be its own fare.
    bool board(int passenger, int driver);
    // Set down one passenger. Safe on an agent that is not riding.
    void alight(int passenger);
    // Set down everyone `driver` is carrying.
    void alightAll(int driver);
    // The agent driving `passenger`, or -1 when it is on foot.
    int driverOf(int passenger) const;
    // How many `driver` is carrying.
    int load(int driver) const;
    bool empty() const { return byPassenger_.empty(); }
    std::size_t rideCount() const { return byPassenger_.size(); }
    void clear();

    // Every (passenger, driver) pair, ascending by passenger, so any traversal
    // is deterministic -- the sim's results must not depend on hash order.
    std::vector<std::pair<int, int>> rides() const;

private:
    std::unordered_map<int, int> byPassenger_;
    std::unordered_map<int, int> loadByDriver_;
};

}  // namespace citysim

#endif
