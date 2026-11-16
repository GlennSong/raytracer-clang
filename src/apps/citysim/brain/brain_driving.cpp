#include "brain_driving.h"

#include <algorithm>

namespace citysim {
namespace brain {

namespace {
// The driving's kinds of event, in its own order (driving::EventKind), as brain.lua names them.
const char* const kDrivingEventNames[driving::kEventKindCount] = {
    "parked", "no_route", "cannot_park", "missed_turn", "crashed", "ended_early",
};
constexpr uint32_t kIndexMask = (1u << RoadPlaces::kIndexBits) - 1;
}  // namespace

RoadPlaces::RoadPlaces(const engine::RoadModel& model, uint32_t tag) : model_(model), tag_(tag & ~kIndexMask) {}

bool RoadPlaces::owns(PlaceRef place) const {
    return place.valid() && (place.id & ~kIndexMask) == tag_ && (place.id & kIndexMask) < spaces_.size();
}

PlaceRef RoadPlaces::placeOf(const engine::KerbSpace& space) {
    PlaceRef out;
    if (!space.valid()) return out;
    for (std::size_t i = 0; i < spaces_.size(); ++i) {
        const engine::KerbSpace& s = spaces_[i];
        if (s.street == space.street && s.dir == space.dir && s.index == space.index) {
            out.id = tag_ | static_cast<uint32_t>(i);
            return out;
        }
    }
    if (spaces_.size() >= kIndexMask) return out;
    spaces_.push_back(space);
    out.id = tag_ | static_cast<uint32_t>(spaces_.size() - 1);
    return out;
}

bool RoadPlaces::spaceOf(PlaceRef place, engine::KerbSpace& out) const {
    if (!owns(place)) return false;
    out = spaces_[place.id & kIndexMask];
    return true;
}

bool RoadPlaces::positionOf(PlaceRef place, Vec2& out) const {
    engine::KerbSpace space;
    engine::KerbSpacePose pose;
    if (!spaceOf(place, space) || !model_.kerbSpacePose(space, pose)) return false;
    out = pose.at;
    return true;
}

bool RoadPlaces::kerbLane(PlaceRef place, driving::PathPoint& out) const {
    engine::KerbSpace space;
    if (!spaceOf(place, space)) return false;
    const engine::KerbLanePlace lane = model_.kerbLaneBeside(space);
    if (!lane.valid()) return false;
    out.path = lane.path;
    out.along = lane.along;
    return true;
}

std::vector<PlaceRef> RoadPlaces::spacesNear(Vec2 door, Real reach, std::size_t most) {
    std::vector<PlaceRef> out;
    const engine::RoadFrontage frontage = model_.frontageAt(door);
    if (!frontage.street.valid()) return out;
    for (const engine::KerbSpaceNear& found : model_.kerbSpacesNear(frontage, reach, most)) {
        const PlaceRef place = placeOf(found.space);
        if (place.valid()) out.push_back(place);
    }
    return out;
}

DrivingSkill::DrivingSkill(driving::Driving& driving, const KerbLanes& lanes, uint32_t vehicleClass)
    : driving_(driving), lanes_(lanes), vehicleClass_(vehicleClass) {
    for (EventId& e : events_) e = kNoId;
}

bool DrivingSkill::bindEvents(const BrainTable& table, std::string* err) {
    for (uint8_t k = 0; k < driving::kEventKindCount; ++k) {
        const int e = table.event(kDrivingEventNames[k]);
        if (e < 0) {
            if (err) *err = table.file + ": events has no `" + kDrivingEventNames[k] + "`, which the driving reports";
            return false;
        }
        events_[k] = static_cast<EventId>(e);
    }
    return true;
}

bool DrivingSkill::pullOut(CarRef car, PlaceRef space) {
    const driving::CarId id(car.id);
    if (driving_.car(id)) return true;   // already on the road (left in the lane by an earlier trip)
    driving::PathPoint at;
    return lanes_.kerbLane(space, at) && driving_.placeCar(id, vehicleClass_, at);
}

bool DrivingSkill::beginTrip(CarRef car, AgentId person, PlaceRef fromSpace, PlaceRef toSpace, Real) {
    const driving::CarId id(car.id);
    // (part-way along is not built: a car not yet on the road starts from the space it was planned from)
    if (!driving_.car(id) && !pullOut(car, fromSpace)) return false;
    driving::PathPoint to;
    if (!lanes_.kerbLane(toSpace, to)) return false;
    return driving_.beginTrip(id, static_cast<driving::PersonId>(person), to).valid();
}

void DrivingSkill::endTrip(CarRef car) {
    const driving::CarId id(car.id);
    const driving::TripId trip = driving_.tripOf(id);
    if (trip.valid() && driving_.endTrip(trip)) endedHere_.push_back(id);
}

void DrivingSkill::park(CarRef car, PlaceRef) { driving_.removeCar(driving::CarId(car.id)); }

void DrivingSkill::report(Machine& machine) {
    for (const driving::Event& e : driving_.drainEvents()) {
        const uint8_t kind = static_cast<uint8_t>(e.kind);
        if (kind >= driving::kEventKindCount || events_[kind] == kNoId) continue;
        if (e.kind == driving::EventKind::EndedEarly) {
            const std::vector<driving::CarId>::iterator mine = std::find(endedHere_.begin(), endedHere_.end(), e.car);
            if (mine != endedHere_.end()) {
                endedHere_.erase(mine);
                continue;
            }
        }
        machine.post(static_cast<AgentId>(e.person), events_[kind], e.car.index);
    }
}

}  // namespace brain
}  // namespace citysim
