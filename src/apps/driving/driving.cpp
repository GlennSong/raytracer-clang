#include "apps/driving/driving.h"

#include "engine/bundle/bundle.h"   // Fnv: the repo's 64-bit hash folded field by field

#include <algorithm>
#include <string>
#include <tuple>

namespace driving {

bool eventBefore(const Event& a, const Event& b) {
    return std::make_tuple(a.car.index, static_cast<uint8_t>(a.kind), a.trip.index, a.trip.generation, a.tick, a.person) <
           std::make_tuple(b.car.index, static_cast<uint8_t>(b.kind), b.trip.index, b.trip.generation, b.tick, b.person);
}

Driving::Driving(const engine::LaneGraph* roads) : roads_(roads) {
    if (roads_) routeBounds_ = studyRoads(*roads_);
}

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

namespace {

// Says why, when the caller asked; always false (a refusal).
bool refuse(std::string* why, const std::string& reason) {
    if (why) *why = reason;
    return false;
}

std::string pathName(engine::PathId p) { return "path " + std::to_string(p.index); }
std::string carName(CarId c) { return "car " + std::to_string(c.index); }

// Does `from` lead straight on into `to` (is `to` among the paths leaving its end)?
bool leadsInto(const engine::LaneGraph& roads, engine::PathId from, engine::PathId to) {
    const engine::Range next = roads.paths[from.index].next;
    for (uint32_t i = next.first; i < next.end() && i < roads.nextPaths.size(); ++i)
        if (roads.nextPaths[i] == to) return true;
    return false;
}

}  // namespace

void Driving::setVehicleClasses(VehicleClasses classes) {
    vehicleClasses_ = std::move(classes);
    routeClasses_.clear();
    routeNotes_.clear();
    if (roads_) {
        for (const VehicleClass& c : vehicleClasses_.all())
            routeClasses_.push_back(bindRouteClass(*roads_, c, &routeNotes_));
        rebuildPathLists(*roads_, vehicleClasses_, cars_, pathLists_);
    }
}

Route Driving::routeFor(const Car& c, const PathPoint& to) const {
    PathPoint front;
    front.path = c.path;
    front.along = c.along;
    const RouteClass cls = c.vehicleClass < routeClasses_.size() ? routeClasses_[c.vehicleClass] : RouteClass();
    return findRoute(*roads_, routeBounds_, cls, front, to);
}

void Driving::queueEvent(EventKind kind, CarId car, TripId trip, PersonId person) {
    Event e;
    e.kind = kind;
    e.car = car;
    e.trip = trip;
    e.person = person;
    e.tick = tickCount_;
    events_.push_back(e);
}

Car* Driving::findPlaced(CarId id) {
    std::vector<Car>::iterator it =
        std::lower_bound(cars_.begin(), cars_.end(), id, [](const Car& c, CarId want) { return c.id < want; });
    return it != cars_.end() && it->id == id ? &*it : nullptr;
}

const Car* Driving::car(CarId id) const {
    std::vector<Car>::const_iterator it =
        std::lower_bound(cars_.begin(), cars_.end(), id, [](const Car& c, CarId want) { return c.id < want; });
    return it != cars_.end() && it->id == id ? &*it : nullptr;
}

bool Driving::placeCar(CarId id, uint32_t vehicleClass, const PathPoint& front, const std::vector<engine::PathId>& trail,
                       std::string* why) {
    if (!id.valid()) return refuse(why, "no car id given");
    if (car(id)) return refuse(why, carName(id) + " is already placed");
    if (vehicleClass >= vehicleClasses_.size())
        return refuse(why, carName(id) + ": no vehicle class " + std::to_string(vehicleClass) + " (there are " +
                               std::to_string(vehicleClasses_.size()) + ")");
    if (!roads_ || !front.path.valid() || front.path.index >= roads_->paths.size())
        return refuse(why, carName(id) + ": there is no " + pathName(front.path) + " on the road");
    if (!onRoad(front))
        return refuse(why, carName(id) + ": " + std::to_string(front.along) + " m is off " + pathName(front.path) +
                               ", which is " + std::to_string(roads_->paths[front.path.index].length) + " m long");
    if (trail.size() > kMaxTrail)
        return refuse(why, carName(id) + ": a trail of " + std::to_string(trail.size()) + " paths is more than the " +
                               std::to_string(kMaxTrail) + " a car may lie on behind its front");
    engine::PathId ahead = front.path;
    for (const engine::PathId p : trail) {
        if (!p.valid() || p.index >= roads_->paths.size())
            return refuse(why, carName(id) + ": its trail names " + pathName(p) + ", which is not on the road");
        if (!leadsInto(*roads_, p, ahead))
            return refuse(why, carName(id) + ": its trail's " + pathName(p) + " does not lead into " + pathName(ahead));
        ahead = p;
    }

    Car c;
    c.id = id;
    c.vehicleClass = vehicleClass;
    c.path = front.path;
    c.along = front.along;
    c.trailCount = static_cast<uint8_t>(trail.size());
    for (std::size_t i = 0; i < trail.size(); ++i) c.trail[i] = trail[i];
    const Real length = vehicleClasses_.at(vehicleClass).length;
    const Body body = layBody(*roads_, c, length);
    if (body.unplaced > 0) {
        if (trail.size() == kMaxTrail)
            return refuse(why, carName(id) + ": its body (" + std::to_string(length) + " m) reaches back over more than " +
                                   std::to_string(kMaxTrail) + " paths behind its front, the most a car may lie on");
        return refuse(why, carName(id) + ": its body (" + std::to_string(length) + " m) does not fit on " +
                               pathName(front.path) + " and its trail: " + std::to_string(body.unplaced) +
                               " m has nowhere to lie");
    }
    // Keep only the trail paths the body reaches.
    c.trailCount = static_cast<uint8_t>(body.count - 1);
    for (std::size_t i = c.trailCount; i < kMaxTrail; ++i) c.trail[i] = engine::PathId();

    std::vector<Car>::iterator at =
        std::lower_bound(cars_.begin(), cars_.end(), id, [](const Car& x, CarId want) { return x.id < want; });
    cars_.insert(at, c);
    rebuildPathLists(*roads_, vehicleClasses_, cars_, pathLists_);
    return true;
}

bool Driving::removeCar(CarId id, std::string* why) {
    if (!car(id)) return refuse(why, carName(id) + " is not placed");
    if (tripOfCar_.count(id) != 0) return refuse(why, carName(id) + " is on a trip: end the trip first");
    std::vector<Car>::iterator at =
        std::lower_bound(cars_.begin(), cars_.end(), id, [](const Car& x, CarId want) { return x.id < want; });
    cars_.erase(at);
    if (roads_) rebuildPathLists(*roads_, vehicleClasses_, cars_, pathLists_);
    return true;
}

TripId Driving::beginTrip(CarId car, PersonId person, const PathPoint& destination) {
    Car* placed = findPlaced(car);
    if (!placed || tripOfCar_.count(car) != 0 || !onRoad(destination)) return TripId{};
    Route route = routeFor(*placed, destination);
    if (!route.found) {
        queueEvent(EventKind::NoRoute, car, TripId{}, person);
        return TripId{};
    }
    placed->driven = true;
    Trip t;
    t.car = car;
    t.person = person;
    t.destination = destination;
    t.beganAtTick = tickCount_;
    t.route = std::move(route);
    const TripId id = trips_.insert(t);
    tripOfCar_[car] = id;
    return id;
}

const Trip* Driving::trip(TripId id) const { return trips_.get(id); }

const Route* Driving::route(TripId id) const {
    const Trip* t = trips_.get(id);
    return t ? &t->route : nullptr;
}

bool Driving::changeDestination(TripId id, const PathPoint& destination) {
    Trip* t = trips_.get(id);
    if (!t || !onRoad(destination)) return false;
    const Car* c = car(t->car);
    if (!c) return false;
    Route route = routeFor(*c, destination);
    if (!route.found) {
        queueEvent(EventKind::NoRoute, t->car, id, t->person);
        return false;
    }
    t->destination = destination;
    t->route = std::move(route);
    return true;
}

bool Driving::reroute(TripId id) {
    Trip* t = trips_.get(id);
    if (!t) return false;
    const Car* c = car(t->car);
    if (!c) return false;
    Route route = routeFor(*c, t->destination);
    if (!route.found) {
        queueEvent(EventKind::NoRoute, t->car, id, t->person);
        return false;
    }
    t->route = std::move(route);
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
    if (Car* c = findPlaced(t->car)) c->driven = false;
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
    // The cars, in car-id order (cars_ is kept in it).
    h.pod(static_cast<uint64_t>(cars_.size()));
    for (const Car& c : cars_) {
        h.pod(c.id.index);
        h.pod(c.vehicleClass);
        h.pod(c.path.index);
        h.pod(c.besidePath.index);
        h.pod(c.along);
        h.pod(c.offset);
        h.pod(c.speed);
        h.pod(c.acceleration);
        h.pod(static_cast<uint8_t>(c.driven));
        h.pod(c.trailCount);
        for (std::size_t i = 0; i < c.trailCount; ++i) h.pod(c.trail[i].index);
    }
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
        h.pod(routeHash(t->route));
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
