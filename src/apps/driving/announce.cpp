#include "apps/driving/announce.h"

#include "apps/driving/notice.h"    // kStandingBelow
#include "engine/bundle/bundle.h"   // Fnv: the repo's 64-bit hash folded field by field

#include <algorithm>
#include <cmath>
#include <tuple>

namespace driving {

namespace {

constexpr Real kTwo = 2;   // the 2 in v^2 = u^2 + 2 a s, and in "average speed is half of v" when braking to rest

// The order within one crossing point.
bool boardOrder(const Announcement& a, const Announcement& b) {
    return std::make_tuple(a.arriveIn, a.who, a.onPath.index) < std::make_tuple(b.arriveIn, b.who, b.onPath.index);
}

bool knownPath(const engine::LaneGraph& roads, engine::PathId p) { return p.valid() && p.index < roads.paths.size(); }

}  // namespace

// ---- The board -----------------------------------------------------------------------------------------------------

void AnnouncementBoard::begin(uint64_t tick, std::size_t crossingCount) {
    entries_.clear();
    tick_ = tick;
    crossingCount_ = crossingCount;
    firstOf_.assign(crossingCount + 1, 0);
}

void AnnouncementBoard::seal() {
    // Count each crossing point's, turn the counts into where each begins, then file them there.
    firstOf_.assign(crossingCount_ + 1, 0);
    std::size_t kept = 0;
    for (const Announcement& a : entries_)
        if (a.crossing.valid() && a.crossing.index < crossingCount_) {
            ++firstOf_[a.crossing.index + 1];
            ++kept;
        }
    for (std::size_t c = 0; c < crossingCount_; ++c) firstOf_[c + 1] += firstOf_[c];
    filed_.resize(kept);
    for (const Announcement& a : entries_) {
        if (!a.crossing.valid() || a.crossing.index >= crossingCount_) continue;
        // firstOf_[c] is used as the next free place for crossing c while filing, then set back below.
        filed_[firstOf_[a.crossing.index]++] = a;
    }
    // Each crossing point's run is now [firstOf_[c - 1], firstOf_[c]): shift the index back one, and order each run.
    for (std::size_t c = crossingCount_; c > 0; --c) firstOf_[c] = firstOf_[c - 1];
    firstOf_[0] = 0;
    for (std::size_t c = 0; c < crossingCount_; ++c)
        if (firstOf_[c + 1] - firstOf_[c] > 1)
            std::sort(filed_.begin() + firstOf_[c], filed_.begin() + firstOf_[c + 1], boardOrder);
    entries_.swap(filed_);
    most_ = std::max(most_, entries_.size());
}

AnnouncementRange AnnouncementBoard::onCrossings(uint32_t first, uint32_t end) const {
    AnnouncementRange r;
    r.first = r.last = entries_.data();
    if (first >= end || first >= crossingCount_ || firstOf_.size() != crossingCount_ + 1) return r;
    const std::size_t hi = std::min<std::size_t>(end, crossingCount_);
    r.first = entries_.data() + firstOf_[first];
    r.last = entries_.data() + firstOf_[hi];
    return r;
}

AnnouncementRange AnnouncementBoard::on(engine::CrossingId c) const {
    if (!c.valid()) return AnnouncementRange();
    return onCrossings(c.index, c.index + 1);
}

const Announcement* AnnouncementBoard::find(engine::CrossingId c, uint64_t who, engine::PathId onPath) const {
    for (const Announcement& a : on(c))
        if (a.who == who && a.onPath == onPath) return &a;
    return nullptr;
}

uint64_t boardHash(const AnnouncementBoard& board) {
    engine::bundle::Fnv h;
    h.pod(board.tick());
    h.pod(static_cast<uint64_t>(board.size()));
    for (const Announcement& a : board.all()) {
        h.pod(a.crossing.index);
        h.pod(a.onPath.index);
        h.pod(a.arriveIn);
        h.pod(a.clearIn);
        h.pod(a.speedThere);
        h.pod(static_cast<uint8_t>(a.stopsFirst));
        h.pod(static_cast<uint8_t>(a.notGoing));
        h.pod(static_cast<uint8_t>(a.going));
        h.pod(static_cast<uint8_t>(a.arrivalFixed));
        h.pod(a.arrivalTick);
        h.pod(a.arrivalAt);
        h.pod(static_cast<uint8_t>(a.committed));
        h.pod(static_cast<uint8_t>(a.waitsInside));
        h.pod(a.committedTick);
        h.pod(a.waited);
        h.pod(a.waitingFor);
        h.pod(static_cast<uint8_t>(a.kind));
        h.pod(a.who);
    }
    return h.h;
}

// ---- The shared estimate -------------------------------------------------------------------------------------------

Real secondsToCover(Real distance, const Mover& m) {
    if (!(distance > 0)) return 0;
    const Real v = std::max(Real(0), m.speed);
    const Real through = std::max(kSlowestThrough, m.speedThrough);
    if (v < through && m.speedUp > 0) {
        // Speeding up toward the speed through, then holding it.
        const Real speedingUp = (through * through - v * v) / (kTwo * m.speedUp);
        if (distance <= speedingUp) return (std::sqrt(v * v + kTwo * m.speedUp * distance) - v) / m.speedUp;
        return (through - v) / m.speedUp + (distance - speedingUp) / through;
    }
    if (v > through && m.brake > 0) {
        // Slowing toward the speed through, then holding it.
        const Real slowing = (v * v - through * through) / (kTwo * m.brake);
        if (distance <= slowing)
            return (v - std::sqrt(std::max(Real(0), v * v - kTwo * m.brake * distance))) / m.brake;
        return (v - through) / m.brake + (distance - slowing) / through;
    }
    return distance / std::max(v, through);
}

Real secondsToStopAt(Real distance, const Mover& m) {
    if (!(distance > 0)) return 0;
    const Real v = std::max(Real(0), m.speed);
    if (v < kSlowestThrough) return secondsToCover(distance, m);   // creeping up to the line from at or near rest
    if (!(m.brake > 0)) return distance / v;
    const Real braking = v * v / (kTwo * m.brake);
    if (distance <= braking) return kTwo * distance / v;          // braking harder than comfortable, evenly to rest
    return (distance - braking) / v + v / m.brake;
}

Real secondsToCoverStoppingFirst(Real distance, Real toLine, const Mover& m) {
    if (!(distance > toLine)) return secondsToCover(distance, m);
    Mover fromRest = m;
    fromRest.speed = 0;
    const Real line = std::max(Real(0), toLine);
    return secondsToStopAt(line, m) + secondsToCover(distance - line, fromRest);
}

Real speedThrough(const engine::LaneGraph& roads, const RoadRules& roadRules, engine::PathId connector, Real topSpeed,
                  Real comfortableSideAccel) {
    Real v = topSpeed > 0 ? topSpeed : kNever;
    if (knownPath(roads, connector)) {
        const Real limit = roadRules.baseLimit(roads, connector);
        if (limit > 0) v = std::min(v, limit);
        // its tightest stretch, as corner_speed holds a car through a connector (#281)
        const Real radius = roads.tightestBend(connector).radius;
        if (std::isfinite(radius) && radius > 0 && comfortableSideAccel > 0)
            v = std::min(v, std::sqrt(comfortableSideAccel * radius));
    }
    if (!std::isfinite(v)) v = kSlowestThrough;
    return std::max(v, kSlowestThrough);
}

// ---- The line -------------------------------------------------------------------------------------------------------

bool isGoing(const JunctionPlan& plan, double odometer, Real speed, Real brake, double heldStopAt, Real within) {
    const double toLine = plan.startOdometer - odometer;
    if (toLine <= 0) return true;
    if (plan.decision == GiveWayDecision::GiveWay || heldStopAt <= plan.startOdometer) return false;
    // Looking a decision ahead only for a driver that has decided to go: one that has not decided yet must still judge.
    const Real ahead = plan.decision == GiveWayDecision::Go ? std::max(Real(0), within) : Real(0);
    return brake > 0 && speed * speed / (kTwo * brake) + speed * ahead > toLine;
}

double heldStopOf(const Car& car) { return car.held.hasStop ? car.held.stopAt : static_cast<double>(kNever); }

void noteAtTheLine(JunctionPlan& plan, const Car& car, Real brake, Real atLineWithin, uint64_t tick, double now) {
    const double odometer = car.odometer;
    const Real speed = car.speed;
    const double toLine = plan.startOdometer - odometer;
    const bool atRest = speed < kStandingBelow;
    if (!plan.cameToRest && atRest && toLine <= atLineWithin && toLine >= -atLineWithin) {
        plan.cameToRest = true;
        plan.restAt = now;
    }
    if (!plan.arrivalFixed && (plan.cameToRest || isGoing(plan, odometer, speed, brake, heldStopOf(car)))) {
        plan.arrivalFixed = true;
        plan.arrivalTick = tick;
        plan.arrivalAt = now;
    }
    if (!plan.committed && odometer > plan.startOdometer + plan.lastWaitingPoint()) {
        plan.committed = true;
        plan.committedTick = tick;
    }
}

void fixArrival(JunctionPlan& plan, Real lineIn, uint64_t tick, double now, Real step) {
    if (plan.arrivalFixed || !std::isfinite(lineIn)) return;
    const Real ahead = std::max(Real(0), lineIn);
    plan.arrivalFixed = true;
    plan.arrivalAt = now + ahead;
    plan.arrivalTick = tick + (step > 0 ? static_cast<uint64_t>(std::ceil(ahead / step)) : uint64_t(0));
}

// ---- Announcing ----------------------------------------------------------------------------------------------------

void announcePlan(const engine::LaneGraph& roads, const JunctionPlan& plan, const PlanAnnouncer& car,
                  AnnouncementBoard& out) {
    if (!knownPath(roads, plan.connector)) return;
    const engine::Path& path = roads.paths[plan.connector.index];
    const bool going = isGoing(plan, car.odometer, car.move.speed, car.move.brake, car.heldStop, car.reaction);
    const uint64_t waitingFor = plan.decision == GiveWayDecision::GiveWay && plan.judged.cause == GiveWayCause::Gap &&
                                        plan.judged.other.valid()
                                    ? plan.judged.other.index
                                    : Announcement::kNoOne;
    const Real toLine = static_cast<Real>(plan.startOdometer - car.odometer);
    const auto when = [&](Real distance) {
        return car.stopsFirst ? secondsToCoverStoppingFirst(distance, toLine, car.move)
                              : secondsToCover(distance, car.move);
    };
    for (uint32_t k = path.crossings.first; k < path.crossings.end() && k < roads.pathCrossings.size(); ++k) {
        const engine::CrossingId cid = roads.pathCrossings[k];
        const engine::CrossingPoint& rec = roads.crossings[cid.index];
        if (rec.otherSide != engine::OtherSide::Path) continue;
        const bool iAmA = rec.a == plan.connector;
        const Real from = iAmA ? rec.aFrom : rec.otherFrom;
        const Real to = iAmA ? rec.aTo : rec.otherTo;
        // Where the front is when it reaches the stretch, and when the tail is past its end.
        const double startAt = plan.startOdometer + from;
        const double clearAt = plan.startOdometer + to + car.length;
        const Real toStart = static_cast<Real>(startAt - car.odometer);
        const Real toClear = static_cast<Real>(clearAt - car.odometer);
        if (!(toClear > 0)) continue;   // its tail is past it already

        Announcement a;
        a.crossing = cid;
        a.onPath = plan.connector;
        a.arriveIn = toStart > 0 ? when(toStart) + car.shift : Real(0);
        a.clearIn = when(toClear) + car.shift;
        a.speedThere = car.move.speedThrough;
        a.stopsFirst = car.stopsFirst;
        // Waiting inside the junction to give way (task D4): not going through the crossing points at or beyond where it
        // waits (its stop is a stretch's start or a waiting point, which is one), though it is past its line.
        const bool waitsShort =
            plan.decision == GiveWayDecision::GiveWay && plan.judged.stopAlong > 0 && from >= plan.judged.stopAlong;
        a.going = going && !waitsShort;
        a.waitsInside = waitsShort;
        // Not going: stopping short of the stretch, or (while it can still stop at the line) for another reason.
        a.notGoing = (car.notGoing && !going) || car.heldStopAt < startAt || car.frontLimit < startAt;
        // A front that cannot get far enough for the tail to clear the stretch: never clear of it.
        if (!a.notGoing && car.frontLimit < clearAt) a.clearIn = kNever;
        a.arrivalFixed = plan.arrivalFixed;
        a.arrivalTick = plan.arrivalTick;
        a.arrivalAt = plan.arrivalAt;
        a.committed = plan.committed;
        a.committedTick = plan.committedTick;
        a.waited = car.waited;
        a.waitingFor = waitingFor;
        a.who = car.who.index;
        out.add(a);
    }
}

void announceLane(const engine::LaneGraph& roads, engine::PathId lane, Real laneStart, CarId who, Real length,
                  const Mover& move, Real lookAheadTime, AnnouncementBoard& out) {
    if (!knownPath(roads, lane)) return;
    const engine::Path& path = roads.paths[lane.index];
    for (uint32_t k = path.crossings.first; k < path.crossings.end() && k < roads.pathCrossings.size(); ++k) {
        const engine::CrossingId cid = roads.pathCrossings[k];
        const engine::CrossingPoint& rec = roads.crossings[cid.index];
        if (rec.otherSide != engine::OtherSide::Path) continue;
        const bool iAmA = rec.a == lane;
        const Real toStart = laneStart + (iAmA ? rec.aFrom : rec.otherFrom);
        const Real toClear = laneStart + (iAmA ? rec.aTo : rec.otherTo) + length;
        if (!(toClear > 0)) continue;
        Announcement a;
        a.crossing = cid;
        a.onPath = lane;
        a.arriveIn = toStart > 0 ? secondsToCover(toStart, move) : Real(0);
        if (a.arriveIn > lookAheadTime) continue;
        a.clearIn = secondsToCover(toClear, move);
        a.speedThere = move.speedThrough;
        // A lane has no line to wait at: the car on it is going, and "arrived" before anyone.
        a.going = true;
        a.arrivalFixed = true;
        a.who = who.index;
        out.add(a);
    }
}

}  // namespace driving
