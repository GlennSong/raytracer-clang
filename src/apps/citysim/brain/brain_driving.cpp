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

bool DrivingSkill::changeDestination(CarRef car, PlaceRef toSpace) {
    const driving::CarId id(car.id);
    const driving::TripId trip = driving_.tripOf(id);
    const engine::LaneGraph* roads = driving_.roads();
    driving::PathPoint to;
    if (!trip.valid() || !driving_.car(id) || !roads || !lanes_.kerbLane(toSpace, to)) return false;
    // (a place off the road is refused here, so that the one way the driving can still refuse is "no route", which it
    // reports itself)
    if (to.path.index >= roads->paths.size() || to.along < 0 || to.along > roads->paths[to.path.index].length) return false;
    if (driving_.changeDestination(trip, to)) return true;
    refusedHere_.push_back(id);
    return false;
}

void DrivingSkill::report(Machine& machine) { report(machine, driving_.drainEvents()); }

void DrivingSkill::report(Machine& machine, const std::vector<driving::Event>& events) {
    for (const driving::Event& e : events) {
        const uint8_t kind = static_cast<uint8_t>(e.kind);
        if (kind >= driving::kEventKindCount || events_[kind] == kNoId) continue;
        if (e.kind == driving::EventKind::EndedEarly) {
            const std::vector<driving::CarId>::iterator mine = std::find(endedHere_.begin(), endedHere_.end(), e.car);
            if (mine != endedHere_.end()) {
                endedHere_.erase(mine);
                continue;
            }
        }
        if (e.kind == driving::EventKind::NoRoute) {
            // a new destination the driving refused: the brain was told by changeDestination's answer
            const std::vector<driving::CarId>::iterator mine = std::find(refusedHere_.begin(), refusedHere_.end(), e.car);
            if (mine != refusedHere_.end()) {
                refusedHere_.erase(mine);
                continue;
            }
        }
        machine.post(static_cast<AgentId>(e.person), events_[kind], e.car.index);
    }
}

// ---- room to put a car into a lane -------------------------------------------------------------------------------------

namespace {
// The most stretches of path one question looks back over. Each uses up some of the distance, so only paths of no
// length could reach it; reaching it answers "not clear" (the car waits) rather than looking for ever.
constexpr int kMostStretchesBack = 64;
}  // namespace

LaneRoom::LaneRoom(const driving::Driving& driving) : driving_(driving) {
    const engine::LaneGraph* roads = driving_.roads();
    const std::size_t n = roads ? roads->paths.size() : 0;
    firstInto_.assign(n + 1, 0);
    if (!roads) return;
    // every "path, then the path after it" of the road, in path order
    std::vector<std::pair<uint32_t, uint32_t>> leads;
    for (uint32_t p = 0; p < n; ++p) {
        const engine::Path& path = roads->paths[p];
        for (uint32_t k = 0; k < path.next.count; ++k) {
            const engine::PathId next = roads->nextPaths[path.next.first + k];
            if (next.index < n) leads.push_back({p, next.index});
        }
    }
    for (const std::pair<uint32_t, uint32_t>& l : leads) ++firstInto_[l.second + 1];
    for (std::size_t i = 0; i < n; ++i) firstInto_[i + 1] += firstInto_[i];
    into_.resize(firstInto_[n]);
    std::vector<uint32_t> filled(n, 0);
    for (const std::pair<uint32_t, uint32_t>& l : leads) into_[firstInto_[l.second] + filled[l.second]++] = engine::PathId(l.first);
}

std::vector<engine::PathId> LaneRoom::pathsInto(engine::PathId path) const {
    std::vector<engine::PathId> out;
    if (path.index + 1 >= firstInto_.size()) return out;
    for (uint32_t k = firstInto_[path.index]; k < firstInto_[path.index + 1]; ++k) out.push_back(into_[k]);
    return out;
}

bool LaneRoom::clear(const driving::PathPoint& front, Real length, Real behind, Real ahead) const {
    const engine::LaneGraph* roads = driving_.roads();
    if (!roads || front.path.index + 1 >= firstInto_.size()) return false;
    const driving::PathLists& lists = driving_.pathLists();
    const Real pathLength = roads->paths[front.path.index].length;
    // on the lane itself: from as far back as it goes to `ahead` past the front
    const Real from = front.along - length - behind;
    if (!driving::carsOnStretch(lists, front.path, std::max(Real(0), from), std::min(pathLength, front.along + ahead)).empty())
        return false;
    // what is left of `behind` before the lane's start, back along every path that leads in
    struct Back {
        engine::PathId path;   // a path leading in
        Real left = 0;         // metres still to look back over, from its end
    };
    std::vector<Back> open;
    const auto openInto = [&](engine::PathId path, Real left) {
        if (!engine::strictlyPast(left, 0, engine::kSamePoint)) return;
        for (uint32_t k = firstInto_[path.index]; k < firstInto_[path.index + 1]; ++k) open.push_back(Back{into_[k], left});
    };
    openInto(front.path, -from);
    for (int looked = 0; !open.empty(); ++looked) {
        if (looked >= kMostStretchesBack) return false;
        const Back b = open.back();
        open.pop_back();
        const Real len = roads->paths[b.path.index].length;
        if (!driving::carsOnStretch(lists, b.path, std::max(Real(0), len - b.left), len).empty()) return false;
        openInto(b.path, b.left - len);
    }
    return true;
}

}  // namespace brain
}  // namespace citysim
