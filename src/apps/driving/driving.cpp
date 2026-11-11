#include "apps/driving/driving.h"

#include "engine/bundle/bundle.h"   // Fnv: the repo's 64-bit hash folded field by field

#include <algorithm>
#include <tuple>

namespace driving {

bool eventBefore(const Event& a, const Event& b) {
    return std::make_tuple(a.car.index, static_cast<uint8_t>(a.kind), a.trip.index, a.trip.generation, a.tick, a.person) <
           std::make_tuple(b.car.index, static_cast<uint8_t>(b.kind), b.trip.index, b.trip.generation, b.tick, b.person);
}

Driving::Driving(const engine::LaneGraph* roads) : roads_(roads) {}

void Driving::tick(Real dt) {
    plan();
    settleClaims();
    move();
    ++tickCount_;
    clockSeconds_ += dt;
}

void Driving::plan() {}
void Driving::settleClaims() {}
void Driving::move() {}

bool Driving::onRoad(const PathPoint& p) const {
    if (!roads_ || !p.path.valid() || p.path.index >= roads_->paths.size()) return false;
    const engine::Path& path = roads_->paths[p.path.index];
    return p.along >= 0 && p.along <= path.length;
}

TripId Driving::beginTrip(CarId car, PersonId person, const PathPoint& destination) {
    if (!car.valid() || tripOfCar_.count(car) != 0 || !onRoad(destination)) return TripId{};
    Trip t;
    t.car = car;
    t.person = person;
    t.destination = destination;
    t.beganAtTick = tickCount_;
    const TripId id = trips_.insert(t);
    tripOfCar_[car] = id;
    return id;
}

const Trip* Driving::trip(TripId id) const { return trips_.get(id); }

bool Driving::changeDestination(TripId id, const PathPoint& destination) {
    Trip* t = trips_.get(id);
    if (!t || !onRoad(destination)) return false;
    t->destination = destination;
    return true;
}

bool Driving::placeHold(TripId id, HoldName name, const PathPoint& at) {
    Trip* t = trips_.get(id);
    if (!t || !onRoad(at)) return false;
    std::vector<Hold>::iterator it = std::lower_bound(t->holds.begin(), t->holds.end(), name,
                                                      [](const Hold& h, HoldName n) { return h.name < n; });
    if (it != t->holds.end() && it->name == name) {
        it->at = at;
    } else {
        Hold h;
        h.name = name;
        h.at = at;
        t->holds.insert(it, h);
    }
    return true;
}

bool Driving::releaseHold(TripId id, HoldName name) {
    Trip* t = trips_.get(id);
    if (!t) return false;
    std::vector<Hold>::iterator it = std::lower_bound(t->holds.begin(), t->holds.end(), name,
                                                      [](const Hold& h, HoldName n) { return h.name < n; });
    if (it == t->holds.end() || it->name != name) return false;
    t->holds.erase(it);
    return true;
}

bool Driving::endTrip(TripId id) {
    const Trip* t = trips_.get(id);
    if (!t) return false;
    Event e;
    e.kind = EventKind::EndedEarly;
    e.car = t->car;
    e.trip = id;
    e.person = t->person;
    e.tick = tickCount_;
    events_.push_back(e);
    tripOfCar_.erase(t->car);
    trips_.erase(id);
    return true;
}

std::vector<Event> Driving::drainEvents() {
    std::vector<Event> out;
    out.swap(events_);
    std::sort(out.begin(), out.end(), eventBefore);
    return out;
}

uint64_t Driving::stateHash() const {
    engine::bundle::Fnv h;
    h.pod(tickCount_);
    h.pod(clockSeconds_);
    // The road: which graph, by its size (the graph is fixed after load; hashing it whole every tick would cost more
    // than everything else here).
    const uint64_t pathCount = roads_ ? roads_->paths.size() : 0;
    h.pod(pathCount);
    // The trips, in car order.
    h.pod(static_cast<uint64_t>(tripOfCar_.size()));
    for (const std::pair<const CarId, TripId>& entry : tripOfCar_) {
        const Trip* t = trips_.get(entry.second);
        h.pod(entry.second.index);
        h.pod(entry.second.generation);
        h.pod(t->car.index);
        h.pod(t->person);
        h.pod(t->destination.path.index);
        h.pod(t->destination.along);
        h.pod(t->beganAtTick);
        h.pod(static_cast<uint64_t>(t->holds.size()));
        for (const Hold& hold : t->holds) {
            h.pod(hold.name);
            h.pod(hold.at.path.index);
            h.pod(hold.at.along);
        }
    }
    // The events not yet drained, in the order they will come out.
    std::vector<Event> queued = events_;
    std::sort(queued.begin(), queued.end(), eventBefore);
    h.pod(static_cast<uint64_t>(queued.size()));
    for (const Event& e : queued) {
        h.pod(static_cast<uint8_t>(e.kind));
        h.pod(e.car.index);
        h.pod(e.trip.index);
        h.pod(e.trip.generation);
        h.pod(e.person);
        h.pod(e.tick);
    }
    return h.h;
}

}  // namespace driving
