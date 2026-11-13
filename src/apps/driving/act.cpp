#include "apps/driving/act.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace driving {

namespace {

constexpr Real kTwo = 2;   // the 2 in v^2 = u^2 + 2 a s, and in 2*sqrt(a*b)

FactNeed need(const char* name, UnitFamily family) {
    FactNeed n;
    n.name = name;
    n.family = family;
    return n;
}

}  // namespace

Real stoppingDistance(Real speed, Real brake, Real reaction) {
    const Real v = std::max(Real(0), speed);
    return v * std::max(Real(0), reaction) + (brake > 0 ? v * v / (kTwo * brake) : Real(0));
}

bool canStopWithin(Real distance, Real speed, Real brake, Real reaction) {
    return stoppingDistance(speed, brake, reaction) <= distance;
}

bool registerMoveNeeds(FactRegistry& registry, std::string* why) {
    ReaderSpec spec;
    spec.name = kMoveReaderName;
    spec.meaning = "works out each driven car's speed from its held answers and the car ahead, every tick";
    spec.numbers.push_back(need(kNumberStandstillGap, UnitFamily::Length));
    spec.numbers.push_back(need(kNumberAnticipation, UnitFamily::Time));
    spec.settings.push_back(need(kSettingGapKept, UnitFamily::Time));
    return registry.addReader(spec, why);
}

Real lookAheadDistance(Real speed, Real comfortableBrake) {
    return std::max(kLookAheadFloor, stoppingDistance(speed, comfortableBrake) + kLookAheadMargin);
}

Real reactionTime(Real decideInterval, Real step) { return std::max(decideInterval, step); }

Real freeRoadAcceleration(const Follower& f, Real speed, Real topSpeed) {
    if (!(topSpeed > 0)) return speed > 0 ? -f.brake : Real(0);
    // Above the top speed: brake comfortably down to it. The formula below slows a car that is only a little over
    // far more gently than that, and the top speeds for a bend or a lower limit ahead (corner_speed, slower_ahead) are
    // "the speed I may do NOW if I brake comfortably from here", so a gentler brake arrives too fast (#257: bends
    // taken at four times the comfortable sideways acceleration).
    if (speed > topSpeed) return -f.brake;
    return f.speedUp * (1 - std::pow(speed / topSpeed, kFreeRoadExponent));
}

Real gapWanted(const Follower& f, Real speed, Real speedAhead) {
    const Real both = f.speedUp * f.brake;
    const Real closing = both > 0 ? speed * (speed - speedAhead) / (kTwo * std::sqrt(both)) : Real(0);
    return f.standstillGap + std::max(Real(0), speed * f.gapKept + closing);
}

Real easingAcceleration(const Follower& f, Real speed, Real topSpeed, Real gap, Real speedAhead) {
    if (!(gap > 0)) return -f.hardestBrake;
    const Real ratio = gapWanted(f, speed, speedAhead) / std::max(gap, kGapFloor);
    return freeRoadAcceleration(f, speed, topSpeed) - f.speedUp * ratio * ratio;
}

Real safeSpeed(Real hardestBrake, Real reaction, Real gap, Real speedAhead, Real aheadHardestBrake) {
    if (!(hardestBrake > 0)) return 0;
    const Real room = gap + stoppingDistance(speedAhead, aheadHardestBrake);
    const Real bt = hardestBrake * reaction;
    const Real underRoot = bt * bt + kTwo * hardestBrake * room;
    if (!(underRoot > 0)) return 0;
    return std::max(Real(0), -bt + std::sqrt(underRoot));
}

StepOutcome stepSpeed(const Follower& f, Real speed, Real topSpeed, RuleIndex topSpeedRule, const Obstacle* obstacles,
                      std::size_t obstacleCount, Real step, Real reaction) {
    StepOutcome out;

    // 1. Easing. Each term is how much it takes off the comfortable speeding-up; the largest decides the acceleration
    // and says what held the car back.
    const Real free = freeRoadAcceleration(f, speed, topSpeed);
    Real mostTaken = f.speedUp - free;   // the top speed's own pull
    if (mostTaken > 0) {
        out.binding.by = Limit::TopSpeed;
        out.binding.rule = topSpeedRule;
    } else {
        mostTaken = 0;
    }
    for (std::size_t i = 0; i < obstacleCount; ++i) {
        const Obstacle& o = obstacles[i];
        if (o.hardOnly) continue;
        // A place to stop by (and a true end of the way) is a stopped car whose rear stands one standstill gap past
        // it. A car ahead pulling away: the gap as it will be `anticipation` seconds on (easing only; the hard limit
        // below reads the gap as it is).
        const Real gap = o.kind == Limit::CarAhead
                             ? o.distance + f.anticipation * std::max(Real(0), o.speed - speed)
                             : o.distance + f.standstillGap;
        const Real speedAhead = o.kind == Limit::CarAhead ? o.speed : Real(0);
        const Real taken = f.speedUp - easingAcceleration(f, speed, topSpeed, gap, speedAhead);
        if (taken > mostTaken) {
            mostTaken = taken;
            out.binding.by = o.kind;
            out.binding.car = o.car;
            out.binding.rule = o.rule;
            out.binding.stopFor = o.kind == Limit::Stop ? o.stopFor : StopFor::Place;
        }
    }
    const Real accel = std::max(f.speedUp - mostTaken, -f.brake);

    // 2. At rest: move off only on a clear go.
    Real next = (speed <= 0 && accel <= kMoveOffAbove) ? Real(0) : speed + accel * step;
    // A car braking down to its top speed stops braking AT it, not a fraction of a tick's braking below it (it would
    // then speed up again, and what limits it would flip every tick).
    if (out.binding.by == Limit::TopSpeed && speed > topSpeed && next < topSpeed) next = topSpeed;

    // 3. The hard limit against each thing ahead.
    for (std::size_t i = 0; i < obstacleCount; ++i) {
        const Obstacle& o = obstacles[i];
        Real limit = 0;
        if (o.kind == Limit::CarAhead)
            limit = safeSpeed(f.hardestBrake, reaction, o.distance - f.standstillGap, o.speed, o.hardestBrake);
        else
            limit = safeSpeed(f.hardestBrake, reaction, o.distance, 0, 0);
        if (limit < next) {
            next = limit;
            out.binding.by = o.kind;
            out.binding.car = o.kind == Limit::CarAhead || o.kind == Limit::Stop ? o.car : CarId();
            out.binding.rule = o.kind == Limit::Stop ? o.rule : kNoRule;
            out.binding.stopFor = o.kind == Limit::Stop ? o.stopFor : StopFor::Place;
        }
    }
    // Comfortable braking may be exceeded only as far as the hardest braking.
    next = std::max({next, speed - f.hardestBrake * step, Real(0)});
    if (next < kComesToRestBelow && next < speed) next = 0;

    out.speed = next;
    out.acceleration = step > 0 ? (next - speed) / step : Real(0);
    return out;
}

}  // namespace driving
