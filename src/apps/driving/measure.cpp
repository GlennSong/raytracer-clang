#include "apps/driving/measure.h"

#include "apps/driving/act.h"   // lookAheadDistance: how far a driver looks, for reading its own limit as it does
#include "apps/driving/route.h"   // bindRouteClass: a class's vehicle tags, for the records that concern it

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>

namespace driving {

namespace {

constexpr double kNoValue = std::numeric_limits<double>::quiet_NaN();
constexpr double kNotYet = -1;   // a time not seen yet (the clock never runs backwards from zero)
// Two clock readings this close are the same moment: what adding a tick sixty times can drift by, and far less.
constexpr double kClockSlackSeconds = 1e-6;
constexpr double kMillisecondsPerSecond = 1000.0;
constexpr double kCarsPerCostUnit = 1000.0;   // the cost row is per 1,000 driven cars

NamedNumber named(const char* key, double value, const char* label, const char* unit) {
    NamedNumber n;
    n.key = key;
    n.value = value;
    n.label = label;
    n.unit = unit;
    return n;
}

Offender offender(CarId car, uint64_t tick, double seconds, std::vector<NamedNumber> numbers) {
    Offender o;
    o.car = car;
    o.tick = tick;
    o.seconds = seconds;
    o.numbers = std::move(numbers);
    return o;
}

std::string fmt(const char* format, double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, format, v);
    return buf;
}
std::string count(uint64_t n) { return std::to_string(n); }
// A number for the table, or "none" when there is none.
std::string orNone(const char* format, double v) { return std::isfinite(v) ? fmt(format, v) : std::string("none"); }

// The words and file keys for what held a car back (traffic_state.h's Limit, in its order).
struct LimitWords {
    Limit limit;
    const char* key;
    const char* words;
};
constexpr LimitWords kLimitWords[kLimitCount] = {
    {Limit::Nothing, "nothing_s", "held by nothing"},
    {Limit::CarAhead, "behind_a_car_s", "behind a car"},
    {Limit::Stop, "at_a_stop_s", "at a place to stop"},
    {Limit::TopSpeed, "top_speed_s", "held by a top speed"},
    {Limit::WayEnd, "way_end_s", "at the end of the way known"},
};

std::string ruleName(const std::vector<std::string>& names, RuleIndex rule) {
    if (rule < names.size() && !names[rule].empty()) return names[rule];
    return "rule " + std::to_string(rule);
}

bool knownPath(const engine::LaneGraph* roads, engine::PathId p) {
    return roads && p.valid() && p.index < roads->paths.size();
}

}  // namespace

// ---- Offenders and statuses ---------------------------------------------------------------------------------------

std::size_t Offenders::offer(const Offender& o) {
    ++total;
    if (kept.size() >= kOffendersKept) return kNotKept;
    kept.push_back(o);
    return kept.size() - 1;
}

const char* rowStatusKey(RowStatus s) {
    switch (s) {
        case RowStatus::Pass: return "pass";
        case RowStatus::Fail: return "fail";
        case RowStatus::NoTarget: return "no_target";
        case RowStatus::NoData: return "no_data";
        case RowStatus::NotMeasured: return "not_measured";
    }
    return "no_target";
}

const char* rowStatusWords(RowStatus s) {
    switch (s) {
        case RowStatus::Pass: return "pass";
        case RowStatus::Fail: return "FAIL";
        case RowStatus::NoTarget: return "no target yet";
        case RowStatus::NoData: return "nothing to see";
        case RowStatus::NotMeasured: return "not measured yet";
    }
    return "no target yet";
}

// ---- The pure parts ------------------------------------------------------------------------------------------------

bool isJump(double moved, double speedBefore, double speedAfter, double dt, double* excess) {
    const double allowed = std::max(std::fabs(speedBefore), std::fabs(speedAfter)) * dt + kJumpToleranceMetres;
    if (excess) *excess = moved - allowed;
    return moved > allowed;
}

bool turnsTooTight(double angle, double moved, double tightestRadius) {
    if (!(moved > kMovedNoiseMetres) || !(angle > kAngleNoiseRadians) || !(tightestRadius > 0)) return false;
    return angle / moved > (1.0 / tightestRadius) * (1.0 + kTurnToleranceShare);
}

bool pivotsAtRest(double angle, double moved) { return !(moved > kMovedNoiseMetres) && angle > kAngleNoiseRadians; }

double planAngle(double ax, double az, double bx, double bz) {
    const double crossed = ax * bz - az * bx;
    const double dotted = ax * bx + az * bz;
    return std::fabs(std::atan2(crossed, dotted));
}

double sideAcceleration(double speed, double radius) {
    if (!std::isfinite(radius) || !(radius > 0)) return 0;
    return speed * speed / radius;
}

std::vector<OverlapPair> overlapsOn(const PathList& list) {
    std::vector<OverlapPair> out;
    for (std::size_t i = 0; i < list.size(); ++i) {
        const PathEntry& a = list[i];
        if (a.why == OnPath::Beside) continue;
        for (std::size_t j = i + 1; j < list.size(); ++j) {
            const PathEntry& b = list[j];
            if (b.why == OnPath::Beside || b.car == a.car) continue;
            const double overlap = std::min<double>(a.front, b.front) - std::max<double>(a.rear, b.rear);
            if (!(overlap > kOverlapToleranceMetres)) continue;
            OverlapPair p;
            p.a = a.car < b.car ? a.car : b.car;
            p.b = a.car < b.car ? b.car : a.car;
            p.overlap = overlap;
            out.push_back(p);
        }
    }
    return out;
}

bool crossesLine(const PathPoint& before, const PathPoint& now, double moved, const PathPoint& line,
                 double* beforeLine) {
    double reach = 0;
    if (before.path == line.path && before.along < line.along) {
        if (now.path == line.path && now.along < line.along) return false;
        reach = line.along - before.along;   // still on the line's path and past it, or moved on beyond its end
    } else if (before.path != line.path && now.path == line.path && now.along >= line.along) {
        reach = moved - (now.along - line.along);
    } else {
        return false;
    }
    if (beforeLine) *beforeLine = std::max(0.0, std::min(reach, moved));
    return true;
}

std::vector<QueueBurst> findQueueBursts(const std::vector<LineCrossing>& crossings) {
    std::vector<QueueBurst> out;
    bool anyBefore = false;
    double lastCrossing = 0;
    bool inBurst = false;
    QueueBurst burst;
    double lastCounted = 0;
    const LineCrossing* ahead = nullptr;   // the queue's last car counted
    for (const LineCrossing& x : crossings) {
        if (inBurst) {
            const bool stays = x.seconds - lastCounted < kQueueQuietSeconds && x.restLeftAt >= 0 &&
                               x.restCameAt >= 0 && x.restCameAt <= ahead->restLeftAt &&
                               x.restLeftAt >= ahead->restLeftAt && x.restDistance >= ahead->restDistance &&
                               x.restDistance - ahead->restDistance <= kQueueSpacingMetres;
            if (stays) {
                burst.gaps.push_back(x.seconds - lastCounted);
                burst.straight.push_back(x.straight && ahead->straight ? 1 : 0);
                lastCounted = x.seconds;
                lastCrossing = x.seconds;
                ahead = &x;
                continue;
            }
            out.push_back(burst);
            inBurst = false;
        }
        const bool quiet = !anyBefore || x.seconds - lastCrossing >= kQueueQuietSeconds;
        const bool fromRestNearby = x.restLeftAt >= 0 && x.restLeftAt <= x.seconds &&
                                    x.restDistance <= kQueueStartNearMetres;
        if (quiet && fromRestNearby) {
            burst = QueueBurst();
            burst.firstCar = x.car;
            burst.tick = x.tick;
            burst.seconds = x.seconds;
            burst.firstCarDelay = x.seconds - x.restLeftAt;
            lastCounted = x.seconds;
            ahead = &x;
            inBurst = true;
        }
        anyBefore = true;
        lastCrossing = x.seconds;
    }
    if (inBurst) out.push_back(burst);
    return out;
}

QueueFigures queueFigures(const std::vector<QueueBurst>& bursts) {
    QueueFigures f;
    double gapSum = 0, allSum = 0, delaySum = 0;
    // The gap before car n is gaps[n - 2] (cars counted from 1).
    const std::size_t firstGap = kSaturationFromCar >= 2 ? kSaturationFromCar - 2 : 0;
    for (const QueueBurst& b : bursts) {
        for (std::size_t i = firstGap; i < b.gaps.size(); ++i) {
            allSum += b.gaps[i];
            ++f.gapsAll;
            if (i < b.straight.size() && !b.straight[i]) continue;
            gapSum += b.gaps[i];
            ++f.gaps;
        }
        delaySum += b.firstCarDelay;
        ++f.bursts;
    }
    f.secondsPerCar = f.gaps > 0 ? gapSum / static_cast<double>(f.gaps) : kNoValue;
    f.secondsPerCarAll = f.gapsAll > 0 ? allSum / static_cast<double>(f.gapsAll) : kNoValue;
    f.firstCarDelay = f.bursts > 0 ? delaySum / static_cast<double>(f.bursts) : kNoValue;
    return f;
}

// ---- The rows ------------------------------------------------------------------------------------------------------

void NoReasonRow::note(CarId car, uint64_t tick, double seconds, double dt, bool noReason) {
    if (!noReason) {
        open_.erase(car);
        return;
    }
    std::map<CarId, Open>::iterator it = open_.find(car);
    if (it == open_.end()) {
        Open o;
        o.start = seconds - dt;
        o.offender = offenders.offer(offender(car, tick, seconds, {named("spell_s", dt, "standing for", "s")}));
        it = open_.emplace(car, o).first;
        ++spells;
    }
    carSeconds += dt;
    const double spell = seconds - it->second.start;
    longestSpell = std::max(longestSpell, spell);
    if (it->second.offender != Offenders::kNotKept) offenders.kept[it->second.offender].numbers[0].value = spell;
}

void NoReasonRow::forget(CarId car) { open_.erase(car); }

void LoopRow::note(const std::vector<WaitLoop>& loops, uint64_t tick, double seconds) {
    std::map<std::vector<uint32_t>, Open> next;
    for (const WaitLoop& loop : loops) {
        std::vector<uint32_t> key;
        for (const CarId c : loop.cars) key.push_back(c.index);
        std::sort(key.begin(), key.end());
        PerClass& per = loop.kind == LoopClass::Deadlock ? deadlock : gridlock;
        const std::map<std::vector<uint32_t>, Open>::const_iterator it = open_.find(key);
        Open o;
        if (it != open_.end() && it->second.kind == loop.kind) {
            o = it->second;
        } else {
            o.kind = loop.kind;
            o.first = seconds;
            const CarId lowest = key.empty() ? CarId() : CarId(key.front());
            o.offender = per.offenders.offer(offender(lowest, tick, seconds,
                                                      {named("cars", static_cast<double>(key.size()), "cars in it", ""),
                                                       named("lasted_s", 0, "lasted", "s")}));
            ++per.loops;
            per.mostCars = std::max<uint64_t>(per.mostCars, key.size());
        }
        const double lasted = seconds - o.first;
        per.longest = std::max(per.longest, lasted);
        if (o.offender != Offenders::kNotKept) per.offenders.kept[o.offender].numbers[1].value = lasted;
        next[key] = o;
    }
    open_.swap(next);
}

void JumpRow::note(CarId car, uint64_t tick, double seconds, double moved, double speedBefore, double speedAfter,
                   double dt) {
    double excess = 0;
    if (!isJump(moved, speedBefore, speedAfter, dt, &excess)) return;
    ++ticks;
    worstExcess = std::max(worstExcess, excess);
    offenders.offer(offender(car, tick, seconds,
                             {named("moved_m", moved, "moved", "m"), named("speed_before", speedBefore, "speed before", "m/s"),
                              named("speed_after", speedAfter, "speed after", "m/s")}));
}

void TurnRow::note(CarId car, uint64_t tick, double seconds, double angle, double moved, double tightestRadius) {
    if (moved > kMovedNoiseMetres && angle > kAngleNoiseRadians && tightestRadius > 0)
        worstRatio = std::max(worstRatio, angle / moved * tightestRadius);
    if (turnsTooTight(angle, moved, tightestRadius)) {
        ++tooTight;
        offenders.offer(offender(car, tick, seconds,
                                 {named("turn_per_m", angle / moved, "turning", "rad/m"),
                                  named("allowed_per_m", 1.0 / tightestRadius, "its tightest", "rad/m"),
                                  named("moved_m", moved, "over", "m")}));
    } else if (pivotsAtRest(angle, moved)) {
        ++pivots;
        offenders.offer(offender(car, tick, seconds,
                                 {named("angle_rad", angle, "turned at rest by", "rad"), named("moved_m", moved, "moved", "m")}));
    }
}

void JunctionStopRow::note(const Binding& binding, double dt) {
    carSeconds += dt;
    byLimit[static_cast<uint8_t>(binding.by)] += dt;
    if (binding.by == Limit::Stop) byStopRule[binding.rule] += dt;
}

// ---- At rest in a junction box (issue #264) ----------------------------------------------------------------------------

const char* boxReasonKey(BoxReason r) {
    switch (r) {
    case BoxReason::GivingWay: return "giving_way";
    case BoxReason::BehindGivingWay: return "behind_giving_way";
    case BoxReason::BehindMovingInBox: return "behind_moving_in_box";
    case BoxReason::BehindOther: return "behind_other";
    case BoxReason::WaitingForRoom: return "waiting_for_room";
    case BoxReason::StopByRule: return "stop_by_rule";
    case BoxReason::TopSpeed: return "top_speed";
    case BoxReason::WayEnd: return "way_end";
    case BoxReason::Nothing: return "nothing";
    }
    return "?";
}

const char* boxReasonWords(BoxReason r) {
    switch (r) {
    case BoxReason::GivingWay: return "giving way (a recorded give-way, or waiting inside for a far turn)";
    case BoxReason::BehindGivingWay: return "behind a car giving way in a box";
    case BoxReason::BehindMovingInBox: return "behind a car moving off inside a box";
    case BoxReason::BehindOther: return "behind a car held by something else (a queue reaching into the box)";
    case BoxReason::WaitingForRoom: return "waiting for room beyond";
    case BoxReason::StopByRule: return "stopped by another rule";
    case BoxReason::TopSpeed: return "held by a top speed";
    case BoxReason::WayEnd: return "at the end of its way";
    case BoxReason::Nothing: return "held by nothing";
    }
    return "?";
}

bool boxReasonAllowed(BoxReason r) {
    return r == BoxReason::GivingWay || r == BoxReason::BehindGivingWay || r == BoxReason::BehindMovingInBox;
}

bool bodyInBox(const engine::LaneGraph& roads, const VehicleClasses& classes, const Car& car, Real atLineWithin) {
    if (car.vehicleClass >= classes.size() || !knownPath(&roads, car.path)) return false;
    const Body body = layBody(roads, car, classes.at(car.vehicleClass).length);
    for (std::size_t k = 0; k < body.count; ++k) {
        const BodyPiece& piece = body.pieces[k];
        if (!knownPath(&roads, piece.path)) continue;
        const engine::Path& path = roads.paths[piece.path.index];
        if (path.kind == engine::PathKind::Connector) {
            if (piece.front - std::max(Real(0), piece.rear) > atLineWithin) return true;
            continue;
        }
        for (uint32_t c = path.crossings.first; c < path.crossings.end() && c < roads.pathCrossings.size(); ++c) {
            const engine::CrossingPoint& x = roads.crossings[roads.pathCrossings[c].index];
            if (x.otherSide == engine::OtherSide::Crosswalk && x.a == piece.path && piece.front > x.aFrom &&
                piece.rear < x.aTo)
                return true;
        }
    }
    return false;
}

BoxReason boxReason(const std::vector<Car>& cars, const PathLists& lists, const Car& car,
                    bool (*inBox)(const Car& car, const void* context), const void* context) {
    switch (car.binding.by) {
    case Limit::Nothing: return BoxReason::Nothing;
    case Limit::TopSpeed: return BoxReason::TopSpeed;
    case Limit::WayEnd: return BoxReason::WayEnd;
    case Limit::Stop:
        if (givesWay(car.binding.stopFor)) return BoxReason::GivingWay;
        if (car.binding.stopFor == StopFor::NoRoom) return BoxReason::WaitingForRoom;
        return BoxReason::StopByRule;
    case Limit::CarAhead: break;
    }
    // Held by the car ahead, but itself holding a stop to give way further on (a far turn waiting inside behind another):
    // its own decision is a recorded give-way.
    if (car.held.hasStop && givesWay(car.held.stopFor)) return BoxReason::GivingWay;
    // Behind a car: follow the queue to its head (each waiting, held by the car ahead), no further than the cars there are.
    const Car* head = &car;
    for (std::size_t steps = 0; steps <= cars.size(); ++steps) {
        if (head->binding.by != Limit::CarAhead) break;
        const Car* ahead = findCar(cars, lists, head->binding.car);
        if (!ahead || ahead == &car) {
            head = nullptr;   // a ring, or a car gone: nothing in a box gives way at its head
            break;
        }
        if (!isWaiting(*ahead)) {
            // The queue's head is moving off. Inside a box itself, the queue inside is starting off (a far turn that
            // waited inside going in its gap, the one behind not yet moving): allowed. Beyond the box, it is a queue that
            // reached back into it.
            return inBox(*ahead, context) ? BoxReason::BehindMovingInBox : BoxReason::BehindOther;
        }
        head = ahead;
    }
    if (head && head->binding.by == Limit::Stop && givesWay(head->binding.stopFor) && inBox(*head, context))
        return BoxReason::BehindGivingWay;
    return BoxReason::BehindOther;
}

void BoxStopRow::note(CarId car, uint64_t tick, double seconds, double dt, BoxReason why, const Binding& binding) {
    carSeconds += dt;
    byReason[static_cast<uint8_t>(why)] += dt;
    if (why == BoxReason::StopByRule) byStopRule[binding.rule] += dt;
    if (boxReasonAllowed(why)) {
        bad_.erase(car.index);
        return;
    }
    notAllowed += dt;
    if (bad_.insert(car.index).second)   // a new spell
        offenders.offer(offender(car, tick, seconds,
                                 {named("reason", static_cast<double>(static_cast<uint8_t>(why)), boxReasonKey(why), ""),
                                  named("binding_car", binding.car.valid() ? static_cast<double>(binding.car.index) : -1,
                                        "held by car", "")}));
}

void BoxStopRow::notInBox(CarId car) { bad_.erase(car.index); }

void OverlapRow::note(const std::vector<std::pair<OverlapPair, engine::PathId>>& found, uint64_t tick, double seconds) {
    std::set<std::pair<uint32_t, uint32_t>> now;
    for (const std::pair<OverlapPair, engine::PathId>& f : found) {
        const std::pair<uint32_t, uint32_t> key(f.first.a.index, f.first.b.index);
        if (!now.insert(key).second) continue;   // the same pair on another path this tick: one pair-tick
        ++pairTicks;
        worstOverlap = std::max(worstOverlap, f.first.overlap);
        pairs.insert(key);
        if (lastTick_.count(key) == 0)   // a new spell of this pair overlapping
            offenders.offer(offender(f.first.a, tick, seconds,
                                     {named("other_car", f.first.b.index, "with car", ""),
                                      named("overlap_m", f.first.overlap, "by", "m"),
                                      named("path", f.second.index, "on path", "")}));
    }
    lastTick_.swap(now);
}

// ---- Body against body -----------------------------------------------------------------------------------------------

PlanBody planBody(CarId car, const CarPose& pose, double length, double width) {
    // The lane graph's one body rule (engine::bodyOnPlan), so what is measured here is what the road builder swept.
    const engine::PlanRectangle r =
        engine::bodyOnPlan(engine::Vec2(pose.front.x, pose.front.z), engine::Vec2(pose.rear.x, pose.rear.z),
                           engine::Vec2(pose.travel.x, pose.travel.z), length, width);
    PlanBody b;
    b.car = car;
    b.x = r.middle.x;
    b.z = r.middle.y;
    b.height = (pose.front.y + pose.rear.y) * 0.5;
    b.ux = r.forward.x;
    b.uz = r.forward.y;
    b.halfLength = r.halfLength;
    b.halfWidth = r.halfWidth;
    return b;
}

namespace {

// How far a body reaches along a unit direction (dx, dz) from its middle (half its extent projected on the direction).
double reachAlong(const PlanBody& b, double dx, double dz) {
    return b.halfLength * std::fabs(b.ux * dx + b.uz * dz) + b.halfWidth * std::fabs(-b.uz * dx + b.ux * dz);
}

}  // namespace

double bodyOverlapDepth(const PlanBody& a, const PlanBody& b) {
    // The four axes of two rectangles: each one's length and width directions. The depth on an axis is how far the
    // two projections overlap; the shallowest is the push that parts them, and any axis with none parts them already.
    const double axes[4][2] = {{a.ux, a.uz}, {-a.uz, a.ux}, {b.ux, b.uz}, {-b.uz, b.ux}};
    const double cx = b.x - a.x, cz = b.z - a.z;
    double depth = std::numeric_limits<double>::infinity();
    for (const double* axis : axes) {
        const double apart = std::fabs(cx * axis[0] + cz * axis[1]);
        const double d = reachAlong(a, axis[0], axis[1]) + reachAlong(b, axis[0], axis[1]) - apart;
        if (!(d > 0)) return 0;
        depth = std::min(depth, d);
    }
    return depth;
}

namespace {

struct TouchKindWords {
    TouchKind kind;
    const char* key;
    const char* words;
};
constexpr TouchKindWords kTouchKindWords[kTouchKindCount] = {
    {TouchKind::OnePath, "one_path", "on one path (following, or after a join, failed)"},
    {TouchKind::Split, "split", "a split (two paths leaving one lane)"},
    {TouchKind::InJunction, "in_junction", "a cross or join inside one junction (the conflict resolver's to order)"},
    {TouchKind::LaneAndConnector, "lane_and_connector", "a lane against a connector (filed under no junction)"},
    {TouchKind::TwoJunctions, "two_junctions", "connectors of two junctions (filed under no junction)"},
    {TouchKind::NoRecordClose, "no_record_paths_close",
     "no crossing point, the paths' centre lines within the two half widths"},
    {TouchKind::NoRecordSwing, "no_record_body_swing",
     "no crossing point, centre lines apart: a body reaching outside its path's band"},
};

// The stretches of path a car's body covers (its class's length), front path first.
Body bodyOf(const engine::LaneGraph& roads, const VehicleClasses& classes, const Car& c) {
    const Real length = c.vehicleClass < classes.size() ? classes.at(c.vehicleClass).length : Real(0);
    return layBody(roads, c, length);
}

// Does a body piece on `path` overlap [from, to] of it?
bool coversStretch(const Body& body, engine::PathId path, Real from, Real to) {
    for (std::size_t k = 0; k < body.count; ++k)
        if (body.pieces[k].path == path && body.pieces[k].front >= from && body.pieces[k].rear <= to) return true;
    return false;
}

// The step the band gap samples the centre lines at (metres): the crossing points' own step (lane_graph_crossings.h).
constexpr double kBandStep = 0.25;

}  // namespace

const char* touchKindKey(TouchKind k) { return kTouchKindWords[static_cast<uint8_t>(k)].key; }
const char* touchKindWords(TouchKind k) { return kTouchKindWords[static_cast<uint8_t>(k)].words; }

TouchKind sortTouch(const engine::LaneGraph& roads, const VehicleClasses& classes, const PathLists& lists, const Car& a,
                    const Car& b, uint32_t* crossing, uint32_t* junction, double* bandGap) {
    if (crossing) *crossing = engine::kNoIndex;
    if (junction) *junction = engine::kNoIndex;
    const Body ba = bodyOf(roads, classes, a), bb = bodyOf(roads, classes, b);
    // The band gap: the closest the two bodies' stretches of centre line come, less both half widths.
    {
        double closest = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < ba.count; ++i)
            for (double s = ba.pieces[i].rear; s <= ba.pieces[i].front + kBandStep * 0.5; s += kBandStep) {
                const engine::Vec2 p = roads.pointAt(ba.pieces[i].path, std::min(s, ba.pieces[i].front));
                for (std::size_t j = 0; j < bb.count; ++j)
                    for (double t = bb.pieces[j].rear; t <= bb.pieces[j].front + kBandStep * 0.5; t += kBandStep) {
                        const engine::Vec2 q = roads.pointAt(bb.pieces[j].path, std::min(t, bb.pieces[j].front));
                        closest = std::min(closest, (p - q).length());
                    }
            }
        const double half = (a.vehicleClass < classes.size() ? classes.at(a.vehicleClass).width * 0.5 : 0) +
                            (b.vehicleClass < classes.size() ? classes.at(b.vehicleClass).width * 0.5 : 0);
        if (bandGap) *bandGap = closest - half;
    }
    // Listed together on one path.
    for (std::size_t i = 0; i < ba.count; ++i)
        for (const PathEntry& e : lists.on(ba.pieces[i].path))
            if (e.car == b.id) return TouchKind::OnePath;
    // A crossing point between the paths their bodies are on, over the stretches the bodies cover: the first kind in the
    // enum's order wins.
    TouchKind best = TouchKind::NoRecordSwing;
    bool found = false;
    // Only records that concern both cars (ADR-0412: one made for a longer body is not theirs).
    const engine::TagMask tagsA = a.vehicleClass < classes.size() ? bindRouteClass(roads, classes.at(a.vehicleClass)).tags
                                                                  : engine::kEveryVehicle;
    const engine::TagMask tagsB = b.vehicleClass < classes.size() ? bindRouteClass(roads, classes.at(b.vehicleClass)).tags
                                                                  : engine::kEveryVehicle;
    for (std::size_t i = 0; i < ba.count; ++i) {
        const engine::PathId pa = ba.pieces[i].path;
        const engine::Path& path = roads.paths[pa.index];
        for (uint32_t k = path.crossings.first; k < path.crossings.end(); ++k) {
            const engine::CrossingId cid = roads.pathCrossings[k];
            const engine::CrossingPoint& rec = roads.crossings[cid.index];
            if (rec.otherSide != engine::OtherSide::Path) continue;
            const bool aIsA = rec.a == pa;
            const engine::PathId other = aIsA ? rec.otherPath() : rec.a;
            if (!rec.concerns(pa, tagsA) || !rec.concerns(other, tagsB)) continue;
            const Real myFrom = aIsA ? rec.aFrom : rec.otherFrom, myTo = aIsA ? rec.aTo : rec.otherTo;
            const Real theirFrom = aIsA ? rec.otherFrom : rec.aFrom, theirTo = aIsA ? rec.otherTo : rec.aTo;
            if (!coversStretch(ba, pa, myFrom, myTo) || !coversStretch(bb, other, theirFrom, theirTo)) continue;
            TouchKind kind = TouchKind::InJunction;
            if (rec.meeting == engine::Meeting::Split) kind = TouchKind::Split;
            else if (!rec.junction.valid())
                kind = path.kind != roads.paths[other.index].kind ? TouchKind::LaneAndConnector : TouchKind::TwoJunctions;
            if (!found || static_cast<uint8_t>(kind) < static_cast<uint8_t>(best)) {
                best = kind;
                found = true;
                if (crossing) *crossing = cid.index;
                if (junction) *junction = rec.junction.index;
            }
        }
    }
    if (found) return best;
    return bandGap && *bandGap < 0 ? TouchKind::NoRecordClose : TouchKind::NoRecordSwing;
}

void TouchRow::note(const std::vector<TouchSpell>& now, uint64_t tick, double seconds) {
    // Spells that did not go on this tick have ended: keep them whole (the first kTouchSpellsKept).
    std::map<std::pair<uint32_t, uint32_t>, TouchSpell>::iterator it = open_.begin();
    std::size_t k = 0;
    while (it != open_.end()) {
        while (k < now.size() && std::make_pair(now[k].a.car.index, now[k].b.car.index) < it->first) ++k;
        const bool goesOn = k < now.size() && std::make_pair(now[k].a.car.index, now[k].b.car.index) == it->first;
        if (goesOn) {
            ++it;
            continue;
        }
        if (kept.size() < kTouchSpellsKept) kept.push_back(it->second);
        it = open_.erase(it);
    }
    for (const TouchSpell& t : now) {
        const std::pair<uint32_t, uint32_t> key(t.a.car.index, t.b.car.index);
        const uint8_t kind = static_cast<uint8_t>(t.kind);
        ++pairTicks;
        worstDepth = std::max(worstDepth, t.worstDepth);
        std::map<std::pair<uint32_t, uint32_t>, TouchSpell>::iterator o = open_.find(key);
        if (o != open_.end()) {
            ++pairTicksByKind[static_cast<uint8_t>(o->second.kind)];
            worstByKind[static_cast<uint8_t>(o->second.kind)] =
                std::max(worstByKind[static_cast<uint8_t>(o->second.kind)], t.worstDepth);
            o->second.lastTick = tick;
            o->second.lastSeconds = seconds;
            o->second.worstDepth = std::max(o->second.worstDepth, t.worstDepth);
            continue;
        }
        // A new spell, sorted by how it began.
        TouchSpell s = t;
        s.firstTick = s.lastTick = tick;
        s.firstSeconds = s.lastSeconds = seconds;
        open_.emplace(key, s);
        ++spells;
        ++spellsByKind[kind];
        ++pairTicksByKind[kind];
        worstByKind[kind] = std::max(worstByKind[kind], t.worstDepth);
        spellsOneNotDriven += !t.a.driven || !t.b.driven;
        spellsOneAtRest += t.a.speed < kStandingBelow || t.b.speed < kStandingBelow;
        spellsDrawnBigger += t.a.drawnBigger || t.b.drawnBigger;
        offenders.offer(offender(t.a.car, tick, seconds,
                                 {named("other_car", t.b.car.index, "with car", ""),
                                  named("depth_m", t.worstDepth, "by", "m"),
                                  named("kind", kind, "kind", ""),
                                  named("path", t.a.path.index, "on path", ""),
                                  named("other_path", t.b.path.index, "against path", "")}));
    }
}

std::vector<TouchSpell> TouchRow::open() const {
    std::vector<TouchSpell> out;
    for (const std::pair<const std::pair<uint32_t, uint32_t>, TouchSpell>& o : open_) out.push_back(o.second);
    return out;
}

void OverLimitRow::note(CarId car, uint64_t tick, double seconds, double dt, double speed, double myLimit,
                        engine::PathId path) {
    checkedCarSeconds += dt;
    const double excess = speed - myLimit;
    if (!(excess > kOverLimitToleranceMps)) {
        over_.erase(car);
        return;
    }
    carSeconds += dt;
    worstExcess = std::max(worstExcess, excess);
    std::map<CarId, Spell>::iterator it = over_.find(car);
    if (it == over_.end()) {
        Spell s;
        s.offender = offenders.offer(offender(car, tick, seconds,
                                              {named("speed", speed, "at", "m/s"), named("my_limit", myLimit, "its own limit", "m/s"),
                                               named("path", path.index, "on path", "")}));
        it = over_.emplace(car, s).first;
    }
    // The kept offender shows the worst moment of its spell.
    if (it->second.offender != Offenders::kNotKept && excess > it->second.worst) {
        Offender& o = offenders.kept[it->second.offender];
        o.tick = tick;
        o.seconds = seconds;
        o.numbers[0].value = speed;
        o.numbers[1].value = myLimit;
        o.numbers[2].value = path.index;
    }
    it->second.worst = std::max(it->second.worst, excess);
}

void OverLimitRow::notChecked(CarId car) { over_.erase(car); }

void BendRow::note(CarId car, uint64_t tick, double seconds, double dt, double speed, double radius, double comfortable) {
    checkedCarSeconds += dt;
    const double side = sideAcceleration(speed, radius);
    worstSideAccel = std::max(worstSideAccel, side);   // the worst seen, over the target or not (#281)
    if (!(side > comfortable + kSideAccelToleranceMps2)) {
        over_.erase(car);
        return;
    }
    carSeconds += dt;
    std::map<CarId, Spell>::iterator it = over_.find(car);
    if (it == over_.end()) {
        Spell s;
        s.offender = offenders.offer(offender(car, tick, seconds,
                                              {named("speed", speed, "at", "m/s"), named("radius_m", radius, "round a radius of", "m"),
                                               named("side_accel", side, "sideways", "m/s2")}));
        it = over_.emplace(car, s).first;
    }
    if (it->second.offender != Offenders::kNotKept && side > it->second.worst) {
        Offender& o = offenders.kept[it->second.offender];
        o.tick = tick;
        o.seconds = seconds;
        o.numbers[0].value = speed;
        o.numbers[1].value = radius;
        o.numbers[2].value = side;
    }
    it->second.worst = std::max(it->second.worst, side);
}

void BendRow::notChecked(CarId car) { over_.erase(car); }

void CostRow::note(double milliseconds, std::size_t drivenCars) {
    if (drivenCars == 0) return;
    const double perThousand = milliseconds * kCarsPerCostUnit / static_cast<double>(drivenCars);
    ++ticksTimed;
    sumPerThousand += perThousand;
    worstPerThousand = std::max(worstPerThousand, perThousand);
}

// ---- The table -----------------------------------------------------------------------------------------------------

namespace {

ReportRow makeRow(const char* key, const char* title, const char* target) {
    ReportRow r;
    r.key = key;
    r.title = title;
    r.target = target;
    return r;
}

RowStatus zeroTarget(bool broken) { return broken ? RowStatus::Fail : RowStatus::Pass; }

ReportRow notMeasuredRow(const char* key, const char* title, const char* target, const char* needs) {
    ReportRow r = makeRow(key, title, target);
    r.status = RowStatus::NotMeasured;
    r.summary = std::string("not measured yet (") + needs + ")";
    return r;
}

ReportRow loopRow(const char* key, const char* title, const char* target, const LoopRow::PerClass& per, bool hasTarget) {
    ReportRow r = makeRow(key, title, target);
    r.status = hasTarget ? zeroTarget(per.loops > 0) : RowStatus::NoTarget;
    r.values = {named("loops", static_cast<double>(per.loops), "loops", ""),
                named("longest_s", per.longest, "the longest lasted", "s"),
                named("most_cars", static_cast<double>(per.mostCars), "the most cars in one", "")};
    r.summary = count(per.loops) + " loops seen; the longest lasted " + fmt("%.1f", per.longest) +
                " s; the most cars in one " + count(per.mostCars);
    r.offenders = per.offenders;
    return r;
}

}  // namespace

MeasurementReport buildReport(const MeasureRows& m, const std::vector<std::string>& ruleNames) {
    MeasurementReport rep;
    rep.ticks = m.ticks;
    rep.seconds = m.seconds;
    rep.drivenCars = m.drivenCars;

    {
        ReportRow r = makeRow("standing_no_reason", "Standing still with no reason (nothing named, or a reason that no "
                                                    "longer applies)",
                              "none: every driven car at rest is waiting for something it can name, that still holds");
        r.status = zeroTarget(m.noReason.spells > 0);
        r.values = {named("car_seconds", m.noReason.carSeconds, "", "car-seconds"),
                    named("longest_spell_s", m.noReason.longestSpell, "the longest spell", "s"),
                    named("spells", static_cast<double>(m.noReason.spells), "spells", "")};
        r.summary = fmt("%.1f", m.noReason.carSeconds) + " car-seconds in " + count(m.noReason.spells) +
                    " spells; the longest " + fmt("%.1f", m.noReason.longestSpell) + " s";
        r.offenders = m.noReason.offenders;
        rep.rows.push_back(r);
    }
    {
        ReportRow r = makeRow("endless_waits",
                              "Waiting with no way it can end (refused room beyond for a reason that cannot change)",
                              "none: the router avoids every such way");
        r.status = zeroTarget(m.endless.spells > 0);
        r.values = {named("car_seconds", m.endless.carSeconds, "", "car-seconds"),
                    named("longest_spell_s", m.endless.longestSpell, "the longest spell", "s"),
                    named("spells", static_cast<double>(m.endless.spells), "spells", "")};
        r.summary = fmt("%.1f", m.endless.carSeconds) + " car-seconds in " + count(m.endless.spells) +
                    " spells; the longest " + fmt("%.1f", m.endless.longestSpell) + " s";
        r.offenders = m.endless.offenders;
        rep.rows.push_back(r);
    }
    {
        ReportRow r = makeRow("no_progress",
                              "No progress (a car on a trip advancing less than progress_least over progress_window, "
                              "standing or creeping), with no reason that applies",
                              "none, but for cars held by a reason that still applies");
        const ProgressRow& p = m.progress;
        r.status = !m.progressMeasured ? RowStatus::NotMeasured : zeroTarget(p.unexcused.spells > 0);
        r.values = {named("car_seconds", p.carSeconds, "", "car-seconds without progress"),
                    named("unexcused_s", p.unexcused.carSeconds, "with no reason that applies", "car-seconds"),
                    named("longest_spell_s", p.unexcused.longestSpell, "the longest such spell", "s"),
                    named("spells", static_cast<double>(p.unexcused.spells), "spells", "")};
        r.summary = !m.progressMeasured
                        ? std::string("not measured (the rule set has no progress_window and progress_least)")
                        : fmt("%.1f", p.carSeconds) + " car-seconds without progress, " +
                              fmt("%.1f", p.unexcused.carSeconds) + " of them with no reason that applies, in " +
                              count(p.unexcused.spells) + " spells; the longest " + fmt("%.1f", p.unexcused.longestSpell) +
                              " s";
        r.offenders = p.unexcused.offenders;
        rep.rows.push_back(r);
    }
    rep.rows.push_back(loopRow("deadlock_loops",
                               "Deadlock loops (cars waiting on each other in a ring, a right of way among them)",
                               "none", m.loops.deadlock, true));
    rep.rows.push_back(loopRow("gridlock_loops",
                               "Gridlock loops (cars waiting on each other in a ring, every one for space: room beyond, "
                               "the car ahead, or a body on a crossing point)",
                               "none set yet: a road-design or demand problem, not a bug", m.loops.gridlock, false));
    {
        ReportRow r = makeRow("jumps", "Jumps (a front moving further in a tick than its speed could carry it)", "none");
        r.status = zeroTarget(m.jumps.ticks > 0);
        r.values = {named("ticks", static_cast<double>(m.jumps.ticks), "ticks", ""),
                    named("worst_excess_m", m.jumps.worstExcess, "the worst by", "m")};
        r.summary = count(m.jumps.ticks) + " car-ticks; the worst " + fmt("%.3f", m.jumps.worstExcess) +
                    " m further than its speed allows";
        r.offenders = m.jumps.offenders;
        rep.rows.push_back(r);
    }
    {
        ReportRow r = makeRow("tight_turns", "Turning tighter than the car can, or turning at rest", "none");
        r.status = zeroTarget(m.turns.tooTight + m.turns.pivots > 0);
        r.values = {named("too_tight", static_cast<double>(m.turns.tooTight), "too tight", ""),
                    named("pivots", static_cast<double>(m.turns.pivots), "at rest", ""),
                    named("worst_ratio", m.turns.worstRatio, "the tightest", "of its turning circle")};
        r.summary = count(m.turns.tooTight) + " car-ticks turning too tight, " + count(m.turns.pivots) +
                    " turning at rest; the tightest turn " + fmt("%.2f", m.turns.worstRatio) +
                    " times what its turning circle allows";
        r.offenders = m.turns.offenders;
        rep.rows.push_back(r);
    }
    {
        ReportRow r = makeRow("junction_stops", "At rest inside a junction (on a connector), by what held the car",
                              "none set yet: allowed when giving way by a rule, and recorded as such");
        r.status = RowStatus::NoTarget;
        r.values.push_back(named("car_seconds", m.junctionStops.carSeconds, "", "car-seconds"));
        std::string parts;
        for (const LimitWords& w : kLimitWords) {
            const double s = m.junctionStops.byLimit[static_cast<uint8_t>(w.limit)];
            r.values.push_back(named(w.key, s, w.words, "s"));
            if (s > 0) parts += std::string(parts.empty() ? "" : ", ") + fmt("%.1f", s) + " " + w.words;
        }
        for (const std::pair<const RuleIndex, double>& s : m.junctionStops.byStopRule) {
            const std::string name = ruleName(ruleNames, s.first);
            NamedNumber n = named("", s.second, "", "s");
            n.key = "stop_rule:" + name + "_s";
            n.label = "stopped by rule \"" + name + "\"";
            r.values.push_back(n);
            parts += ", " + fmt("%.1f", s.second) + " by rule \"" + name + "\"";
        }
        r.summary = fmt("%.1f", m.junctionStops.carSeconds) + " car-seconds" + (parts.empty() ? "" : ": " + parts);
        rep.rows.push_back(r);
    }
    {
        // Issue #264 (the D5 brief's "done when"): nobody at rest in a box except by a recorded give-way.
        ReportRow r = makeRow("box_stops",
                              "At rest with any part of the body in a junction box (or on a crosswalk), by why",
                              "only giving way (a recorded give-way, waiting inside for a far turn, or behind a car "
                              "giving way in the box or moving off inside it)");
        const BoxStopRow& b = m.boxStops;
        r.status = zeroTarget(b.notAllowed > 0);
        r.values.push_back(named("car_seconds", b.carSeconds, "", "car-seconds"));
        r.values.push_back(named("not_allowed_s", b.notAllowed, "not allowed", "s"));
        std::string parts;
        for (uint8_t k = 0; k < kBoxReasonCount; ++k) {
            const BoxReason why = static_cast<BoxReason>(k);
            NamedNumber n = named("", b.byReason[k], boxReasonWords(why), "s");
            n.key = std::string(boxReasonKey(why)) + "_s";
            r.values.push_back(n);
            if (b.byReason[k] > 0 && why != BoxReason::StopByRule)
                parts += std::string(parts.empty() ? "" : ", ") + fmt("%.1f", b.byReason[k]) + " " + boxReasonWords(why);
        }
        for (const std::pair<const RuleIndex, double>& s : b.byStopRule) {
            const std::string name = ruleName(ruleNames, s.first);
            NamedNumber n = named("", s.second, "", "s");
            n.key = "stop_rule:" + name + "_s";
            n.label = "stopped by rule \"" + name + "\"";
            r.values.push_back(n);
            parts += std::string(parts.empty() ? "" : ", ") + fmt("%.1f", s.second) + " by rule \"" + name + "\"";
        }
        r.summary = fmt("%.1f", b.carSeconds) + " car-seconds, " + fmt("%.1f", b.notAllowed) + " not allowed" +
                    (parts.empty() ? "" : ": " + parts);
        r.offenders = b.offenders;
        rep.rows.push_back(r);
    }
    {
        ReportRow r = makeRow("overlaps", "Bodies overlapping on one path (cars listed beside a lane are not counted)",
                              "none");
        r.status = zeroTarget(m.overlaps.pairTicks > 0);
        r.values = {named("pair_ticks", static_cast<double>(m.overlaps.pairTicks), "pair-ticks", ""),
                    named("pairs", static_cast<double>(m.overlaps.pairs.size()), "pairs", ""),
                    named("worst_overlap_m", m.overlaps.worstOverlap, "the worst by", "m")};
        r.summary = count(m.overlaps.pairTicks) + " pair-ticks, " + count(m.overlaps.pairs.size()) +
                    " pairs of cars; the worst overlap " + fmt("%.3f", m.overlaps.worstOverlap) + " m";
        r.offenders = m.overlaps.offenders;
        rep.rows.push_back(r);
    }
    {
        const TouchRow& t = m.touches;
        ReportRow r = makeRow("bodies_touching",
                              "Bodies touching on the plan, any two cars, as drawn (every pair, every tick, paths not "
                              "consulted), sorted by how their paths relate",
                              "none");
        r.status = zeroTarget(t.spells > 0);
        r.values = {named("pair_ticks", static_cast<double>(t.pairTicks), "pair-ticks", ""),
                    named("spells", static_cast<double>(t.spells), "spells", ""),
                    named("worst_depth_m", t.worstDepth, "the deepest", "m")};
        std::string parts;
        for (uint8_t k = 0; k < kTouchKindCount; ++k) {
            r.values.push_back(named((std::string(touchKindKey(static_cast<TouchKind>(k))) + "_spells").c_str(),
                                     static_cast<double>(t.spellsByKind[k]), "", "spells"));
            if (t.spellsByKind[k] > 0)
                parts += std::string(parts.empty() ? "" : "; ") + count(t.spellsByKind[k]) + " " +
                         touchKindWords(static_cast<TouchKind>(k));
        }
        r.summary = count(t.spells) + " spells over " + count(t.pairTicks) + " pair-ticks; the deepest " +
                    fmt("%.3f", t.worstDepth) + " m" + (parts.empty() ? "" : ": " + parts);
        r.offenders = t.offenders;
        rep.rows.push_back(r);
    }
    {
        ReportRow r = makeRow("over_own_limit", "Faster than the driver's own limit (the road's limit times its habit)",
                              "none (beyond the tolerance)");
        r.status = m.overOwnLimit.checkedCarSeconds > 0 ? zeroTarget(m.overOwnLimit.carSeconds > 0) : RowStatus::NoData;
        r.values = {named("car_seconds", m.overOwnLimit.carSeconds, "", "car-seconds"),
                    named("checked_car_seconds", m.overOwnLimit.checkedCarSeconds, "of", "car-seconds checked"),
                    named("worst_excess_mps", m.overOwnLimit.worstExcess, "the worst by", "m/s")};
        r.summary = fmt("%.1f", m.overOwnLimit.carSeconds) + " car-seconds over, of " +
                    fmt("%.1f", m.overOwnLimit.checkedCarSeconds) + " checked; the worst " +
                    fmt("%.2f", m.overOwnLimit.worstExcess) + " m/s over";
        r.offenders = m.overOwnLimit.offenders;
        rep.rows.push_back(r);
    }
    {
        ReportRow r = makeRow("bend_speed", "Round bends faster than is comfortable (speed squared over the radius)",
                              "no more than the rule set's comfortable_side_accel (beyond the tolerance)");
        r.status = m.bends.checkedCarSeconds > 0 ? zeroTarget(m.bends.carSeconds > 0) : RowStatus::NoData;
        r.values = {named("car_seconds", m.bends.carSeconds, "", "car-seconds"),
                    named("checked_car_seconds", m.bends.checkedCarSeconds, "of", "car-seconds checked"),
                    named("worst_side_accel", m.bends.worstSideAccel, "the worst", "m/s2")};
        r.summary = fmt("%.1f", m.bends.carSeconds) + " car-seconds over, of " +
                    fmt("%.1f", m.bends.checkedCarSeconds) + " on bends; the most sideways " +
                    fmt("%.2f", m.bends.worstSideAccel) + " m/s2";
        r.offenders = m.bends.offenders;
        rep.rows.push_back(r);
    }
    {
        const QueueFigures f = queueFigures(m.queues);
        ReportRow h = makeRow("queue_headway",
                              "How a queue empties: seconds per car from the fourth car on, both cars going straight on",
                              "1.7 to 2.1 s (the plan: about 1.9 s)");
        h.status = f.gaps == 0 ? RowStatus::NoData
                               : (f.secondsPerCar >= kHeadwayLowest && f.secondsPerCar <= kHeadwayHighest
                                      ? RowStatus::Pass
                                      : RowStatus::Fail);
        h.values = {named("seconds_per_car", f.secondsPerCar, "", "s per car"),
                    named("gaps", static_cast<double>(f.gaps), "gaps", ""),
                    named("bursts", static_cast<double>(f.bursts), "queues", ""),
                    named("seconds_per_car_every_turn", f.secondsPerCarAll, "every turn", "s per car"),
                    named("gaps_every_turn", static_cast<double>(f.gapsAll), "gaps", "")};
        const std::string everyTurn = f.gapsAll == 0 ? std::string()
                                                     : "; turns included " + fmt("%.2f", f.secondsPerCarAll) +
                                                           " s per car over " + count(f.gapsAll) + " gaps (no target)";
        h.summary = (f.gaps == 0 ? "no queue of " + count(kSaturationFromCar) + " or more going straight on seen at a "
                                                                                  "counting line"
                                 : fmt("%.2f", f.secondsPerCar) + " s per car, over " + count(f.gaps) + " gaps in " +
                                       count(f.bursts) + " queues") +
                    everyTurn;
        // (Issue #284: no target. The plan's "about 2 s" is traffic engineers' start-up lost time, counted from the
        // GREEN; this row counts from the first car beginning to move, so a car at rest at its line reads a fraction
        // of a second whatever the drivers do. Fed by the lab's lit lines it read 0.19 s. The row keeps the figure;
        // the target waits for drivers with a start-up reaction and a row counted from the green.)
        ReportRow s = makeRow("queue_start_up",
                              "How a queue empties: the first car, from beginning to move to crossing the line",
                              "none set yet: the plan's 1.5 to 2.5 s (about 2 s) is counted from the green, not from "
                              "moving off; owed with a start-up reaction");
        s.status = f.bursts == 0 ? RowStatus::NoData : RowStatus::NoTarget;
        s.values = {named("first_car_delay_s", f.firstCarDelay, "", "s"),
                    named("bursts", static_cast<double>(f.bursts), "queues", "")};
        s.summary = f.bursts == 0 ? std::string("no queue seen at a counting line")
                                  : fmt("%.2f", f.firstCarDelay) + " s, averaged over " + count(f.bursts) + " queues";
        for (std::size_t i = 0; i < m.queues.size(); ++i) {
            const QueueBurst& b = m.queues[i];
            const double line = i < m.queueLine.size() ? m.queueLine[i] : 0;
            const QueueFigures one = queueFigures({b});
            h.offenders.offer(offender(b.firstCar, b.tick, b.seconds,
                                       {named("cars", static_cast<double>(b.gaps.size() + 1), "cars", ""),
                                        named("seconds_per_car", one.secondsPerCar, "", "s per car"),
                                        named("line", line, "at line", "")}));
            s.offenders.offer(offender(b.firstCar, b.tick, b.seconds,
                                       {named("first_car_delay_s", b.firstCarDelay, "first car", "s"),
                                        named("line", line, "at line", "")}));
        }
        rep.rows.push_back(h);
        rep.rows.push_back(s);
    }
    {
        ReportRow r = makeRow("tick_cost", "What a tick costs: wall-clock milliseconds per 1,000 driven cars",
                              "none set yet");
        r.status = RowStatus::NoTarget;
        const double average =
            m.cost.ticksTimed > 0 ? m.cost.sumPerThousand / static_cast<double>(m.cost.ticksTimed) : kNoValue;
        const double worst = m.cost.ticksTimed > 0 ? m.cost.worstPerThousand : kNoValue;
        r.values = {named("ms_per_1000_average", average, "average", "ms"),
                    named("ms_per_1000_worst", worst, "worst", "ms"),
                    named("ticks_timed", static_cast<double>(m.cost.ticksTimed), "ticks timed", "")};
        r.summary = m.cost.ticksTimed == 0 ? std::string("no tick timed (the host ticks through tickTimed to time them)")
                                           : fmt("%.3f", average) + " ms on average, " + fmt("%.3f", worst) +
                                                 " ms at worst, over " + count(m.cost.ticksTimed) + " ticks timed";
        rep.rows.push_back(r);
    }
    {
        ReportRow r = makeRow("caps_hit", "Fixed caps that cut something short (counted, never silent)", "none set yet");
        r.status = RowStatus::NoTarget;
        r.values = {named("tails_off_trail", static_cast<double>(m.caps.tailsOffTrail), "tails off a trail", ""),
                    named("look_aheads_cut", static_cast<double>(m.caps.lookAheadsCut), "look-aheads cut", "")};
        r.summary = count(m.caps.tailsOffTrail) + " car-ticks with a body longer than its trail, " +
                    count(m.caps.lookAheadsCut) + " look-aheads cut short";
        rep.rows.push_back(r);
    }
    rep.rows.push_back(notMeasuredRow("lane_change_indicators", "Lane changes: indicator on before the move",
                                      "always", "needs lane changes"));
    rep.rows.push_back(
        notMeasuredRow("lane_change_durations", "Lane changes: how long each takes", "3 to 4 s", "needs lane changes"));
    rep.rows.push_back(notMeasuredRow("lane_change_gaps", "Lane changes: gaps accepted",
                                      "within the rule set's", "needs lane changes"));
    rep.rows.push_back(notMeasuredRow("crashes_with_personalities",
                                      "Crashes with personalities on, each with the rules and lapses behind it",
                                      "each one reported", "needs personalities and lapses"));
    return rep;
}

bool MeasurementReport::passed() const {
    for (const ReportRow& r : rows)
        if (r.status == RowStatus::Fail) return false;
    return true;
}

const ReportRow* MeasurementReport::row(const std::string& key) const {
    for (const ReportRow& r : rows)
        if (r.key == key) return &r;
    return nullptr;
}

namespace {

constexpr int kStatusColumn = 16;   // the widest status words, "not measured yet"

std::string offenderLine(const Offender& o) {
    std::string line = "car " + std::to_string(o.car.index) + " at tick " + std::to_string(o.tick) + " (" +
                       fmt("%.2f", o.seconds) + " s):";
    bool first = true;
    for (const NamedNumber& n : o.numbers) {
        line += first ? " " : ", ";
        first = false;
        if (!n.label.empty()) line += n.label + " ";
        line += orNone("%.3f", n.value);
        if (!n.unit.empty()) line += " " + n.unit;
    }
    return line;
}

}  // namespace

std::string MeasurementReport::describe() const {
    std::string out = "Driving measurements over " + std::to_string(ticks) + " ticks (" + fmt("%.1f", seconds) +
                      " s simulated), " + std::to_string(drivenCars) + " cars driven at the last tick: " +
                      (passed() ? "passed" : "FAILED") + "\n";
    for (const ReportRow& r : rows) {
        char status[32];
        std::snprintf(status, sizeof status, "%-*s", kStatusColumn, rowStatusWords(r.status));
        out += std::string("  ") + status + "  " + r.title + ": " + r.summary + ". Target: " + r.target + ".\n";
        if (r.status != RowStatus::Fail) continue;
        for (const Offender& o : r.offenders.kept) out += "      " + offenderLine(o) + "\n";
        if (r.offenders.total > r.offenders.kept.size())
            out += "      ... and " + std::to_string(r.offenders.total - r.offenders.kept.size()) + " more\n";
    }
    return out;
}

namespace {

nlohmann::json number(double v) {
    if (!std::isfinite(v)) return nullptr;
    return v;
}

}  // namespace

bool MeasurementReport::writeJson(const std::string& path, std::string* why) const {
    nlohmann::json root;
    root["ticks"] = ticks;
    root["seconds"] = number(seconds);
    root["driven_cars"] = drivenCars;
    root["passed"] = passed();
    nlohmann::json rowsJson = nlohmann::json::object();
    for (const ReportRow& r : rows) {
        nlohmann::json j;
        j["title"] = r.title;
        j["status"] = rowStatusKey(r.status);
        j["target"] = r.target;
        nlohmann::json values = nlohmann::json::object();
        for (const NamedNumber& n : r.values) values[n.key] = number(n.value);
        j["values"] = values;
        j["offenders_total"] = r.offenders.total;
        nlohmann::json offenders = nlohmann::json::array();
        for (const Offender& o : r.offenders.kept) {
            nlohmann::json oj;
            oj["car"] = o.car.index;
            oj["tick"] = o.tick;
            oj["seconds"] = number(o.seconds);
            nlohmann::json numbers = nlohmann::json::object();
            for (const NamedNumber& n : o.numbers) numbers[n.key] = number(n.value);
            oj["numbers"] = numbers;
            offenders.push_back(oj);
        }
        j["offenders"] = offenders;
        rowsJson[r.key] = j;
    }
    root["rows"] = rowsJson;
    std::ofstream file(path);
    if (!file) {
        if (why) *why = "cannot open " + path + " to write";
        return false;
    }
    constexpr int kIndent = 2;
    file << root.dump(kIndent) << "\n";
    if (!file) {
        if (why) *why = "could not write all of " + path;
        return false;
    }
    return true;
}

// ---- The watcher ---------------------------------------------------------------------------------------------------

std::size_t Measurements::addCountingLine(const PathPoint& at) {
    lines_.push_back(at);
    crossings_.emplace_back();
    linesOnPath_[at.path.index].push_back(lines_.size() - 1);
    return lines_.size() - 1;
}

bool registerMeasureNeeds(FactRegistry& registry, std::vector<std::string>* problems) {
    ReaderSpec reader;
    reader.name = kMeasureReaderName;
    reader.meaning = "finds cars on a trip that make no progress for a long while, standing or creeping";
    for (const std::pair<const char*, UnitFamily>& n :
         {std::make_pair(kNumberProgressWindow, UnitFamily::Time), std::make_pair(kNumberProgressLeast, UnitFamily::Length)}) {
        FactNeed need;
        need.name = n.first;
        need.family = n.second;
        reader.numbers.push_back(need);
    }
    std::string why;
    if (!registry.addReader(reader, &why)) {
        if (problems) problems->push_back(why);
        return false;
    }
    return true;
}

Measurements::Seen Measurements::seeCar(const Driving& d, const Car& c) const {
    Seen s;
    s.car = c.id;
    s.trip = d.tripOf(c.id);
    s.driven = c.driven;
    s.odometer = c.odometer;
    s.front.path = c.path;
    s.front.along = c.along;
    s.speed = c.speed;
    s.atRest = c.speed < kStandingBelow;
    s.restCameAt = s.atRest ? d.clockSeconds() : kNotYet;
    if (knownPath(d.roads(), c.path)) {
        const CarPose pose = carPose(*d.roads(), d.vehicleClasses(), d.ruleSet(), c);
        s.frontAt = pose.front;
        const double planLength = std::sqrt(pose.body.x * pose.body.x + pose.body.z * pose.body.z);
        if (planLength > 0) {
            s.bodyX = pose.body.x / planLength;
            s.bodyZ = pose.body.z / planLength;
        }
    }
    return s;
}

void Measurements::begin(const Driving& d) {
    driving_ = &d;
    rows_ = MeasureRows();
    for (std::vector<LineCrossing>& c : crossings_) c.clear();
    seen_.clear();
    for (const Car& c : d.cars()) seen_.push_back(seeCar(d, c));
    lastClock_ = d.clockSeconds();
    sampledLoops_ = false;
    nextLoopSample_ = 0;
    ruleNames_ = d.rules().labels.rules;
    wantLimit_.clear();
    const FactSlot slot = d.facts().find(kFactMyLimit);
    if (slot != kNoFact) wantLimit_.push_back(slot);
    // The no-progress row: its numbers, and a ring per car sized once (one look a second over the window).
    const Number* window = d.hasRules() ? d.ruleSet().findNumber(kNumberProgressWindow) : nullptr;
    const Number* least = d.hasRules() ? d.ruleSet().findNumber(kNumberProgressLeast) : nullptr;
    rows_.progressMeasured = window && least && window->value >= kLoopSampleSeconds;
    progressWindow_ = rows_.progressMeasured ? window->value : 0;
    progressLeast_ = rows_.progressMeasured ? least->value : 0;
    progressSlots_ = rows_.progressMeasured ? static_cast<std::size_t>(progressWindow_ / kLoopSampleSeconds + 0.5) + 1 : 0;
    progress_.clear();
    lastProgressSample_ = d.clockSeconds();
}

void Measurements::lookForProgress(const Driving& d, double now, double sampleDt) {
    if (!rows_.progressMeasured) return;
    for (const Car& c : d.cars()) {
        if (c.id.index >= progress_.size()) progress_.resize(c.id.index + 1);
        Progress& p = progress_[c.id.index];
        const TripId trip = d.tripOf(c.id);
        if (!c.driven || !trip.valid() || trip != p.trip) {   // a new trip (or none): its window starts again
            p.filled = p.next = 0;
            p.trip = trip;
            rows_.progress.unexcused.forget(c.id);
            if (!c.driven || !trip.valid()) continue;
        }
        if (p.odometer.size() != progressSlots_) p.odometer.assign(progressSlots_, 0);
        p.odometer[p.next] = c.odometer;
        p.next = (p.next + 1) % progressSlots_;
        if (p.filled < progressSlots_) ++p.filled;
        // The window is full: the oldest look is the one about to be overwritten. Stuck: it advanced less than the least
        // over the window, and at its speed now would not advance that much over another (a car moving off after a long
        // wait is making progress from that moment).
        bool stuck = false;
        if (p.filled == progressSlots_)
            stuck = c.odometer - p.odometer[p.next] < progressLeast_ && c.speed * progressWindow_ < progressLeast_ &&
                    !(c.acceleration > 0);
        if (stuck) rows_.progress.carSeconds += sampleDt;
        rows_.progress.unexcused.note(c.id, d.tickCount(), now, sampleDt, stuck && !d.waitReasonHolds(c));
    }
}

void Measurements::lookAtCar(const Driving& d, const Car& c, const Seen* before, double now, double dt, Seen& out) {
    out = seeCar(d, c);
    const uint64_t tick = d.tickCount();
    // The same car on the same drive as last tick (not taken off the road and put back by the host between ticks): a
    // car that was driven then, and is now on the same trip, or has just ended it where it stood.
    const bool sameDrive = before && before->driven &&
                           (out.trip.valid() ? out.trip == before->trip : c.odometer >= before->odometer);
    out.restLeftAt = before && sameDrive ? before->restLeftAt : kNotYet;
    out.restOdometer = before && sameDrive ? before->restOdometer : 0;
    out.restCameAt = before && sameDrive ? before->restCameAt : kNotYet;
    if (before && !before->driven && !c.driven) {
        // Parked both times: it keeps what it was.
        out.restLeftAt = before->restLeftAt;
        out.restOdometer = before->restOdometer;
        out.restCameAt = before->restCameAt;
    }
    if (before && (sameDrive || !before->driven) && before->atRest && !out.atRest) {
        // It began to move during this tick.
        out.restLeftAt = lastClock_;
        out.restOdometer = before->odometer;
    }
    // It came to rest during this tick (or is first seen at rest on this drive).
    if (out.atRest && (!before || !sameDrive || !before->atRest)) out.restCameAt = now;

    if (sameDrive) {
        const double moved = c.odometer - before->odometer;
        const double dx = out.frontAt.x - before->frontAt.x, dz = out.frontAt.z - before->frontAt.z;
        const double chord = std::sqrt(dx * dx + dz * dz);
        rows_.jumps.note(c.id, tick, now, std::max(chord, moved), before->speed, c.speed, dt);
        const double tightest =
            c.vehicleClass < d.vehicleClasses().size() ? d.vehicleClasses().at(c.vehicleClass).tightestTurnRadius : 0;
        rows_.turns.note(c.id, tick, now, planAngle(before->bodyX, before->bodyZ, out.bodyX, out.bodyZ), moved,
                         tightest);
        if (moved > kMovedNoiseMetres && !linesOnPath_.empty()) {
            // Only the lines on the path it was on and the path it is on now (a line on a path crossed whole in one tick
            // is not seen: crossesLine).
            for (int side = 0; side < 2; ++side) {
                const engine::PathId p = side == 0 ? before->front.path : out.front.path;
                if (side == 1 && p == before->front.path) break;
                const std::map<uint32_t, std::vector<std::size_t>>::const_iterator on = linesOnPath_.find(p.index);
                if (on == linesOnPath_.end()) continue;
                for (const std::size_t i : on->second) {
                    double reach = 0;
                    if (!crossesLine(before->front, out.front, moved, lines_[i], &reach)) continue;
                    LineCrossing x;
                    x.car = c.id;
                    x.tick = tick;
                    x.seconds = lastClock_ + dt * (reach / moved);
                    x.restLeftAt = out.restLeftAt;
                    x.restDistance = before->odometer + reach - out.restOdometer;
                    x.restCameAt = out.restCameAt;
                    const engine::LaneGraph* g = d.roads();
                    x.straight = !knownPath(g, out.front.path) ||
                                 g->paths[out.front.path.index].kind != engine::PathKind::Connector ||
                                 g->paths[out.front.path.index].turn == engine::Turn::Straight;
                    crossings_[i].push_back(x);
                }
            }
        }
    }

    // Waiting with nothing named, or naming a reason that no longer applies (issue #284).
    rows_.noReason.note(c.id, tick, now, dt,
                        isWaiting(c) && (waitReasonOf(c).sort == ReasonSort::None || !d.waitReasonHolds(c)));
    // Waiting for room it can never be given (issue #284).
    rows_.endless.note(c.id, tick, now, dt, d.waitCanNeverEnd(c));

    const engine::LaneGraph* roads = d.roads();
    if (!c.driven || !knownPath(roads, c.path) || c.vehicleClass >= d.vehicleClasses().size()) {
        rows_.overOwnLimit.notChecked(c.id);
        rows_.bends.notChecked(c.id);
        return;
    }
    const engine::Path& path = roads->paths[c.path.index];
    if (path.kind == engine::PathKind::Connector && c.speed < kStandingBelow) rows_.junctionStops.note(c.binding, dt);

    // The driver's own limit, read through the registered fact, as the driver reads it.
    const Trip* t = d.trip(out.trip);
    bool checked = false;
    if (t && d.hasRules() && !wantLimit_.empty()) {
        const VehicleClass& cls = d.vehicleClasses().at(c.vehicleClass);
        const DriverView view{c,           cls,           t->person,
                              t->way,      *roads,        d.cars(),
                              d.pathLists(), d.ruleSet(), d.roadRules(),
                              lookAheadDistance(c.speed, cls.brake), &t->destination, &t->holds};
        noticeFacts(view, d.facts(), wantLimit_, noticed_, scratch_);
        const NoticedFact& limit = noticed_[wantLimit_[0]];
        if (limit.present) {
            rows_.overOwnLimit.note(c.id, tick, now, dt, c.speed, limit.number, c.path);
            checked = true;
        }
    }
    if (!checked) rows_.overOwnLimit.notChecked(c.id);

    // The bend where the car's front is, from its path's bend profile (#281): the radius a driver feels there, as the
    // fact corner_speed reads it, not the tightest anywhere on the path.
    const Number* comfortable = d.hasRules() ? d.ruleSet().findNumber(kNumberComfortableSideAccel) : nullptr;
    const Real radius = roads->radiusAt(c.path, c.along);
    if (comfortable && std::isfinite(radius) && radius > 0)
        rows_.bends.note(c.id, tick, now, dt, c.speed, radius, comfortable->value);
    else
        rows_.bends.notChecked(c.id);
}

void Measurements::lookForOverlaps(const Driving& d, double now) {
    overlapsFound_.clear();
    const engine::LaneGraph* roads = d.roads();
    if (!roads) return;
    for (uint32_t p = 0; p < roads->paths.size(); ++p) {
        const PathList list = d.pathLists().on(engine::PathId(p));
        if (list.size() < 2) continue;
        for (const OverlapPair& pair : overlapsOn(list)) overlapsFound_.emplace_back(pair, engine::PathId(p));
    }
    rows_.overlaps.note(overlapsFound_, d.tickCount(), now);
}

void Measurements::setDrawnSize(CarId car, double length, double width) {
    if (!car.valid()) return;
    if (car.index >= drawnSize_.size()) drawnSize_.resize(car.index + 1, {0.0, 0.0});
    drawnSize_[car.index] = {length, width};
}

void Measurements::lookForTouches(const Driving& d, double now) {
    touchesFound_.clear();
    const engine::LaneGraph* roads = d.roads();
    if (!roads) return;
    const VehicleClasses& classes = d.vehicleClasses();
    // Every car on the road, driven or not, as the host draws it.
    bodies_.clear();
    bodyRecord_.clear();
    for (std::size_t r = 0; r < d.cars().size(); ++r) {
        const Car& c = d.cars()[r];
        if (!knownPath(roads, c.path) || c.vehicleClass >= classes.size()) continue;
        const VehicleClass& cls = classes.at(c.vehicleClass);
        double length = cls.length, width = cls.width;
        if (c.id.index < drawnSize_.size() && drawnSize_[c.id.index].first > 0) {
            length = drawnSize_[c.id.index].first;
            width = drawnSize_[c.id.index].second;
        }
        bodies_.push_back(planBody(c.id, carPose(*roads, classes, d.ruleSet(), c), length, width));
        bodyRecord_.push_back(r);
    }
    // THE BROAD PHASE: bodies in order of the lowest x their box on the plan reaches (then car id); sweeping along x,
    // each is tested only against those whose boxes it reaches into.
    sweep_.resize(bodies_.size());
    for (std::size_t i = 0; i < sweep_.size(); ++i) sweep_[i] = i;
    const auto lowX = [&](std::size_t i) { return bodies_[i].x - reachAlong(bodies_[i], 1, 0); };
    std::sort(sweep_.begin(), sweep_.end(), [&](std::size_t p, std::size_t q) {
        const double lp = lowX(p), lq = lowX(q);
        return lp != lq ? lp < lq : bodies_[p].car < bodies_[q].car;
    });
    for (std::size_t s = 0; s < sweep_.size(); ++s) {
        const PlanBody& a = bodies_[sweep_[s]];
        const double highX = a.x + reachAlong(a, 1, 0);
        const double reachZ = reachAlong(a, 0, 1);
        for (std::size_t t = s + 1; t < sweep_.size() && lowX(sweep_[t]) <= highX; ++t) {
            const PlanBody& b = bodies_[sweep_[t]];
            if (std::fabs(b.z - a.z) > reachZ + reachAlong(b, 0, 1)) continue;
            if (std::fabs(b.height - a.height) > kTouchSameLevelMetres) continue;
            const double depth = bodyOverlapDepth(a, b);
            if (!(depth > kTouchToleranceMetres)) continue;
            const bool aFirst = a.car < b.car;
            const Car& ca = d.cars()[bodyRecord_[aFirst ? sweep_[s] : sweep_[t]]];
            const Car& cb = d.cars()[bodyRecord_[aFirst ? sweep_[t] : sweep_[s]]];
            const PlanBody& pa = aFirst ? a : b;
            const PlanBody& pb = aFirst ? b : a;
            TouchSpell spell;
            const auto side = [&](const Car& c, const PlanBody& p) {
                TouchSide o;
                o.car = c.id;
                o.path = c.path;
                o.along = c.along;
                o.speed = c.speed;
                o.driven = c.driven;
                o.binding = c.binding;
                const VehicleClass& cls = classes.at(c.vehicleClass);
                o.drawnBigger = p.halfLength * 2 > cls.length + kTouchToleranceMetres ||
                                p.halfWidth * 2 > cls.width + kTouchToleranceMetres;
                return o;
            };
            spell.a = side(ca, pa);
            spell.b = side(cb, pb);
            spell.kind = sortTouch(*roads, classes, d.pathLists(), ca, cb, &spell.crossing, &spell.junction,
                                   &spell.bandGap);
            spell.x = (pa.x + pb.x) * 0.5;
            spell.z = (pa.z + pb.z) * 0.5;
            spell.worstDepth = depth;
            touchesFound_.push_back(spell);
        }
    }
    std::sort(touchesFound_.begin(), touchesFound_.end(), [](const TouchSpell& p, const TouchSpell& q) {
        return p.a.car != q.a.car ? p.a.car < q.a.car : p.b.car < q.b.car;
    });
    rows_.touches.note(touchesFound_, d.tickCount(), now);
}

namespace {

// What bodyInBox needs, passed through boxReason to look at the car at the head of a queue.
struct BoxLook {
    const engine::LaneGraph* roads = nullptr;
    const VehicleClasses* classes = nullptr;
    Real atLineWithin = 0;
};
bool lookInBox(const Car& car, const void* context) {
    const BoxLook* look = static_cast<const BoxLook*>(context);
    return bodyInBox(*look->roads, *look->classes, car, look->atLineWithin);
}

}  // namespace

void Measurements::lookForBoxStops(const Driving& d, double now, double dt) {
    if (!d.roads()) return;
    BoxLook look;
    look.roads = d.roads();
    look.classes = &d.vehicleClasses();
    look.atLineWithin = d.junctionStep() ? d.junctionNumbers().atLineWithin : Real(0);
    for (const Car& c : d.cars()) {
        if (!isWaiting(c) || !lookInBox(c, &look)) {
            rows_.boxStops.notInBox(c.id);
            continue;
        }
        rows_.boxStops.note(c.id, d.tickCount(), now, dt, boxReason(d.cars(), d.pathLists(), c, lookInBox, &look),
                            c.binding);
    }
}

void Measurements::observe(const Driving& d) {
    if (driving_ != &d) {
        begin(d);
        return;
    }
    const double now = d.clockSeconds();
    const double dt = now - lastClock_;
    if (ruleNames_.size() != d.rules().labels.rules.size()) ruleNames_ = d.rules().labels.rules;
    if (wantLimit_.empty()) {
        const FactSlot slot = d.facts().find(kFactMyLimit);
        if (slot != kNoFact) wantLimit_.push_back(slot);
    }

    // Every car, beside what was seen of it last tick (both in car-id order).
    seenNext_.clear();
    std::size_t k = 0;
    uint32_t driven = 0;
    const auto forget = [&](CarId car) {
        rows_.noReason.forget(car);
        rows_.endless.forget(car);
        rows_.progress.unexcused.forget(car);
    };
    for (const Car& c : d.cars()) {
        while (k < seen_.size() && seen_[k].car < c.id) forget(seen_[k++].car);
        const Seen* before = k < seen_.size() && seen_[k].car == c.id ? &seen_[k] : nullptr;
        Seen out;
        lookAtCar(d, c, before, now, dt, out);
        seenNext_.push_back(out);
        if (c.driven) ++driven;
    }
    while (k < seen_.size()) forget(seen_[k++].car);
    seen_.swap(seenNext_);

    lookForOverlaps(d, now);
    lookForTouches(d, now);
    lookForBoxStops(d, now, dt);

    // The loops and the no-progress row, once a second of simulated time.
    if (!sampledLoops_ || now >= nextLoopSample_ - kClockSlackSeconds) {
        rows_.loops.note(d.waitReport().loops, d.tickCount(), now);
        lookForProgress(d, now, now - lastProgressSample_);
        lastProgressSample_ = now;
        sampledLoops_ = true;
        nextLoopSample_ = now + kLoopSampleSeconds;
    }

    rows_.caps = d.capCounts();
    ++rows_.ticks;
    rows_.seconds += dt;
    rows_.drivenCars = driven;
    lastClock_ = now;
}

MeasurementReport Measurements::report() const {
    MeasureRows rows = rows_;
    rows.queues.clear();
    rows.queueLine.clear();
    for (std::size_t i = 0; i < crossings_.size(); ++i) {
        for (const QueueBurst& b : findQueueBursts(crossings_[i])) {
            rows.queues.push_back(b);
            rows.queueLine.push_back(static_cast<uint32_t>(i));
        }
    }
    return buildReport(rows, ruleNames_);
}

double tickTimed(Driving& d, Real dt) {
    Measurements* m = d.measurements();
    std::size_t driven = 0;
    for (const Car& c : d.cars())
        if (c.driven) ++driven;
    d.attachMeasurements(nullptr);
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    d.tick(dt);
    const std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    d.attachMeasurements(m);
    const double ms = std::chrono::duration<double>(end - start).count() * kMillisecondsPerSecond;
    if (!m) return ms;
    m->observe(d);
    m->noteTickCost(ms, driven);
    return ms;
}

}  // namespace driving
