#include "apps/driving/measure.h"

#include "apps/driving/act.h"   // lookAheadDistance: how far a driver looks, for reading its own limit as it does

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
    double firstRestLeft = 0;
    double lastCounted = 0;
    for (const LineCrossing& x : crossings) {
        if (inBurst) {
            const bool stays = x.seconds - lastCounted < kQueueQuietSeconds && x.restLeftAt >= 0 &&
                               x.restLeftAt >= firstRestLeft;
            if (stays) {
                burst.gaps.push_back(x.seconds - lastCounted);
                lastCounted = x.seconds;
                lastCrossing = x.seconds;
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
            firstRestLeft = x.restLeftAt;
            lastCounted = x.seconds;
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
    double gapSum = 0, delaySum = 0;
    // The gap before car n is gaps[n - 2] (cars counted from 1).
    const std::size_t firstGap = kSaturationFromCar >= 2 ? kSaturationFromCar - 2 : 0;
    for (const QueueBurst& b : bursts) {
        for (std::size_t i = firstGap; i < b.gaps.size(); ++i) {
            gapSum += b.gaps[i];
            ++f.gaps;
        }
        delaySum += b.firstCarDelay;
        ++f.bursts;
    }
    f.secondsPerCar = f.gaps > 0 ? gapSum / static_cast<double>(f.gaps) : kNoValue;
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
    if (!(side > comfortable + kSideAccelToleranceMps2)) {
        over_.erase(car);
        return;
    }
    carSeconds += dt;
    worstSideAccel = std::max(worstSideAccel, side);
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
        ReportRow r = makeRow("standing_no_reason", "Standing still with no reason",
                              "none: every driven car at rest is waiting for something it can name");
        r.status = zeroTarget(m.noReason.spells > 0);
        r.values = {named("car_seconds", m.noReason.carSeconds, "", "car-seconds"),
                    named("longest_spell_s", m.noReason.longestSpell, "the longest spell", "s"),
                    named("spells", static_cast<double>(m.noReason.spells), "spells", "")};
        r.summary = fmt("%.1f", m.noReason.carSeconds) + " car-seconds in " + count(m.noReason.spells) +
                    " spells; the longest " + fmt("%.1f", m.noReason.longestSpell) + " s";
        r.offenders = m.noReason.offenders;
        rep.rows.push_back(r);
    }
    rep.rows.push_back(loopRow("deadlock_loops",
                               "Deadlock loops (cars waiting on each other in a ring, a right of way among them)",
                               "none", m.loops.deadlock, true));
    rep.rows.push_back(loopRow("gridlock_loops",
                               "Gridlock loops (cars waiting on each other in a ring, every one for room)",
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
        ReportRow h = makeRow("queue_headway", "How a queue empties: seconds per car from the fourth car on",
                              "1.7 to 2.1 s (the plan: about 1.9 s)");
        h.status = f.gaps == 0 ? RowStatus::NoData
                               : (f.secondsPerCar >= kHeadwayLowest && f.secondsPerCar <= kHeadwayHighest
                                      ? RowStatus::Pass
                                      : RowStatus::Fail);
        h.values = {named("seconds_per_car", f.secondsPerCar, "", "s per car"),
                    named("gaps", static_cast<double>(f.gaps), "gaps", ""),
                    named("bursts", static_cast<double>(f.bursts), "queues", "")};
        h.summary = f.gaps == 0 ? "no queue of " + count(kSaturationFromCar) + " or more seen at a counting line"
                                : fmt("%.2f", f.secondsPerCar) + " s per car, over " + count(f.gaps) + " gaps in " +
                                      count(f.bursts) + " queues";
        ReportRow s = makeRow("queue_start_up",
                              "How a queue empties: the first car, from beginning to move to crossing the line",
                              "1.5 to 2.5 s (the plan: about 2 s)");
        s.status = f.bursts == 0 ? RowStatus::NoData
                                 : (f.firstCarDelay >= kStartUpLowest && f.firstCarDelay <= kStartUpHighest
                                        ? RowStatus::Pass
                                        : RowStatus::Fail);
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
    return lines_.size() - 1;
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
    if (before && !before->driven && !c.driven) {
        // Parked both times: it keeps what it was.
        out.restLeftAt = before->restLeftAt;
        out.restOdometer = before->restOdometer;
    }
    if (before && (sameDrive || !before->driven) && before->atRest && !out.atRest) {
        // It began to move during this tick.
        out.restLeftAt = lastClock_;
        out.restOdometer = before->odometer;
    }

    if (sameDrive) {
        const double moved = c.odometer - before->odometer;
        const double dx = out.frontAt.x - before->frontAt.x, dz = out.frontAt.z - before->frontAt.z;
        const double chord = std::sqrt(dx * dx + dz * dz);
        rows_.jumps.note(c.id, tick, now, std::max(chord, moved), before->speed, c.speed, dt);
        const double tightest =
            c.vehicleClass < d.vehicleClasses().size() ? d.vehicleClasses().at(c.vehicleClass).tightestTurnRadius : 0;
        rows_.turns.note(c.id, tick, now, planAngle(before->bodyX, before->bodyZ, out.bodyX, out.bodyZ), moved,
                         tightest);
        if (moved > kMovedNoiseMetres) {
            for (std::size_t i = 0; i < lines_.size(); ++i) {
                double reach = 0;
                if (!crossesLine(before->front, out.front, moved, lines_[i], &reach)) continue;
                LineCrossing x;
                x.car = c.id;
                x.tick = tick;
                x.seconds = lastClock_ + dt * (reach / moved);
                x.restLeftAt = out.restLeftAt;
                x.restDistance = before->odometer + reach - out.restOdometer;
                crossings_[i].push_back(x);
            }
        }
    }

    rows_.noReason.note(c.id, tick, now, dt, isWaiting(c) && waitReasonOf(c).sort == ReasonSort::None);

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

    const Number* comfortable = d.hasRules() ? d.ruleSet().findNumber(kNumberComfortableSideAccel) : nullptr;
    if (comfortable && std::isfinite(path.tightestRadius) && path.tightestRadius > 0)
        rows_.bends.note(c.id, tick, now, dt, c.speed, path.tightestRadius, comfortable->value);
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
    for (const Car& c : d.cars()) {
        while (k < seen_.size() && seen_[k].car < c.id) rows_.noReason.forget(seen_[k++].car);
        const Seen* before = k < seen_.size() && seen_[k].car == c.id ? &seen_[k] : nullptr;
        Seen out;
        lookAtCar(d, c, before, now, dt, out);
        seenNext_.push_back(out);
        if (c.driven) ++driven;
    }
    while (k < seen_.size()) rows_.noReason.forget(seen_[k++].car);
    seen_.swap(seenNext_);

    lookForOverlaps(d, now);

    // The loops, once a second of simulated time.
    if (!sampledLoops_ || now >= nextLoopSample_ - kClockSlackSeconds) {
        rows_.loops.note(d.waitReport().loops, d.tickCount(), now);
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
