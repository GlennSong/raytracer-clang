// The measurements table (#257), the parts that need no road: each row's arithmetic fed plain numbers, so a breach the
// model itself can no longer produce can be staged by hand:
//   1. a jump, and a move its speed explains;
//   2. a turn tighter than the turning circle, one inside it, and a turn at rest;
//   3. two bodies overlapping on one path (and a car beside the lane that is not counted);
//   4. a speed over the driver's own limit, and one inside the tolerance;
//   5. a bend taken too fast;
//   6. a car standing with no reason for 25 seconds;
//   7. a deadlock loop lasting 10 seconds, and a gridlock loop (counted, no target), a changed set of cars being a new
//      loop;
//   8. whether a front crossed a counting line;
//   9. the queue figures from hand-written crossing times: a clean queue, a queue with a straggler, two queues at one
//      line;
//  10. describe() for one passing and one failing row, and writeJson read back key by key.
// The live cases (a real tick on test roads) are in test_driving_tick.cpp beside the tick's own.

#include "test_framework.h"

#include "apps/driving/measure.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

using namespace driving;

namespace {

constexpr double kEps = 1e-9;
constexpr double kTick = 1.0 / 60;

// The row of that key in a report built from `rows`; fails the test when it is missing.
ReportRow rowOf(const MeasureRows& rows, const char* key) {
    const MeasurementReport rep = buildReport(rows, {"speed limit", "held"});
    const ReportRow* r = rep.row(key);
    CHECK(r != nullptr);
    return r ? *r : ReportRow();
}

double numberOf(const Offender& o, const char* key) {
    for (const NamedNumber& n : o.numbers)
        if (n.key == key) return n.value;
    return std::nan("");
}

PathEntry entry(uint32_t car, OnPath why, double front, double rear) {
    PathEntry e;
    e.car = CarId(car);
    e.why = why;
    e.front = front;
    e.rear = rear;
    return e;
}

WaitLoop loopOf(const std::vector<uint32_t>& cars, LoopClass kind) {
    WaitLoop l;
    for (const uint32_t c : cars) {
        l.cars.push_back(CarId(c));
        l.arrows.push_back(kind == LoopClass::Deadlock ? ArrowKind::GivingWay : ArrowKind::Behind);
    }
    l.kind = kind;
    return l;
}

LineCrossing crossing(uint32_t car, double seconds, double restLeftAt, double restDistance) {
    LineCrossing x;
    x.car = CarId(car);
    x.tick = static_cast<uint64_t>(std::lround(seconds * 60));
    x.seconds = seconds;
    x.restLeftAt = restLeftAt;
    x.restDistance = restDistance;
    return x;
}

}  // namespace

TEST_CASE(measure_1_a_jump_is_counted_and_named) {
    // At 12 m/s a tick of 1/60 s carries a front 0.2 m: moving that far is no jump.
    CHECK(!isJump(0.2, 12, 12, kTick));
    CHECK(!isJump(0.2 + kJumpToleranceMetres * 0.5, 12, 12, kTick));
    // Car 3 at tick 12: 5 m in one tick at 1 m/s.
    MeasureRows rows;
    rows.jumps.note(CarId(3), 12, 0.2, 0.2, 12, 12, kTick);
    rows.jumps.note(CarId(3), 12, 0.2, 5.0, 1, 1, kTick);
    const ReportRow r = rowOf(rows, "jumps");
    CHECK(rows.jumps.ticks == 1u);
    CHECK(r.status == RowStatus::Fail);
    CHECK(r.offenders.total == 1u && r.offenders.kept.size() == 1u);
    CHECK(r.offenders.kept[0].car == CarId(3) && r.offenders.kept[0].tick == 12u);
    CHECK_APPROX(numberOf(r.offenders.kept[0], "moved_m"), 5.0, kEps);
    CHECK_APPROX(rows.jumps.worstExcess, 5.0 - (kTick + kJumpToleranceMetres), kEps);
}

TEST_CASE(measure_2_turns_too_tight_and_at_rest) {
    constexpr double kRadius = 4.7;   // the test car's tightest circle
    // Inside the circle: 0.2 rad over a metre (allowed 1 / 4.7 = 0.213 per metre).
    CHECK(!turnsTooTight(0.2, 1.0, kRadius));
    // Within the tolerance: 0.23 per metre.
    CHECK(!turnsTooTight(0.23, 1.0, kRadius));
    // Too tight: 0.4 rad over a metre.
    CHECK(turnsTooTight(0.4, 1.0, kRadius));
    // At rest: any turn beyond rounding is a pivot.
    CHECK(pivotsAtRest(0.1, 0));
    CHECK(!pivotsAtRest(kAngleNoiseRadians / 2, 0));
    CHECK(!pivotsAtRest(0.1, 0.5));
    // The angle between two directions on the plan.
    CHECK_APPROX(planAngle(1, 0, 0, 1), std::acos(-1.0) / 2, kEps);
    CHECK_APPROX(planAngle(1, 0, 0, -1), std::acos(-1.0) / 2, kEps);
    CHECK_APPROX(planAngle(1, 0, 1, 0), 0, kEps);

    MeasureRows rows;
    rows.turns.note(CarId(5), 100, 1.0, 0.2, 1.0, kRadius);   // fine
    rows.turns.note(CarId(5), 101, 1.1, 0.4, 1.0, kRadius);   // too tight
    rows.turns.note(CarId(8), 200, 2.0, 0.1, 0.0, kRadius);   // turned at rest
    const ReportRow r = rowOf(rows, "tight_turns");
    CHECK(rows.turns.tooTight == 1u && rows.turns.pivots == 1u);
    CHECK(r.status == RowStatus::Fail);
    CHECK(r.offenders.kept.size() == 2u);
    CHECK(r.offenders.kept[0].car == CarId(5) && r.offenders.kept[0].tick == 101u);
    CHECK_APPROX(numberOf(r.offenders.kept[0], "turn_per_m"), 0.4, kEps);
    CHECK_APPROX(numberOf(r.offenders.kept[0], "allowed_per_m"), 1 / kRadius, kEps);
    CHECK(r.offenders.kept[1].car == CarId(8) && r.offenders.kept[1].tick == 200u);
    CHECK_APPROX(numberOf(r.offenders.kept[1], "angle_rad"), 0.1, kEps);
    CHECK_APPROX(rows.turns.worstRatio, 0.4 * kRadius, kEps);
}

TEST_CASE(measure_3_overlapping_bodies) {
    // Car 1 from 5.5 to 10 m, car 2 from 7.5 to 12 m: 2.5 m of overlap. Car 9 is beside the lane (part-way through a
    // lane change) level with both: not counted. Car 4 is clear of everyone, 2 m behind car 1.
    const std::vector<PathEntry> entries = {entry(4, OnPath::Front, 3.5, -1), entry(1, OnPath::Front, 10, 5.5),
                                            entry(9, OnPath::Beside, 11, 6.5), entry(2, OnPath::Front, 12, 7.5)};
    PathList list;
    list.first = entries.data();
    list.count = entries.size();
    const std::vector<OverlapPair> found = overlapsOn(list);
    CHECK(found.size() == 1u);
    CHECK(!found.empty() && found[0].a == CarId(1) && found[0].b == CarId(2));
    CHECK(!found.empty() && std::fabs(found[0].overlap - 2.5) < kEps);

    // Three ticks overlapping: three pair-ticks, one pair, one offender (one spell).
    MeasureRows rows;
    std::vector<std::pair<OverlapPair, engine::PathId>> tick;
    for (const OverlapPair& p : found) tick.emplace_back(p, engine::PathId(17));
    for (uint64_t t = 40; t < 43; ++t) rows.overlaps.note(tick, t, static_cast<double>(t) * kTick);
    const ReportRow r = rowOf(rows, "overlaps");
    CHECK(rows.overlaps.pairTicks == 3u && rows.overlaps.pairs.size() == 1u);
    CHECK(r.status == RowStatus::Fail);
    CHECK(r.offenders.total == 1u);
    CHECK(r.offenders.kept[0].car == CarId(1) && r.offenders.kept[0].tick == 40u);
    CHECK_APPROX(numberOf(r.offenders.kept[0], "other_car"), 2, kEps);
    CHECK_APPROX(numberOf(r.offenders.kept[0], "path"), 17, kEps);
}

// BODY AGAINST BODY (issue #283): two drawn bodies, as rectangles on the plan; their depth; and the row's spells.
TEST_CASE(measure_3b_bodies_touching_on_the_plan) {
    const auto body = [](uint32_t car, double x, double z, double ux, double uz, double length, double width) {
        CarPose pose;
        pose.front = engine::Vec3(x + ux * length * 0.5, 0, z + uz * length * 0.5);
        pose.rear = engine::Vec3(x - ux * length * 0.5, 0, z - uz * length * 0.5);
        pose.body = engine::Vec3(ux, 0, uz);
        pose.travel = pose.body;
        return planBody(CarId(car), pose, length, width);
    };
    // Side by side, 1.5 m apart, 1.8 m wide: 0.3 m deep. 2 m apart: clear.
    const PlanBody a = body(1, 0, 0, 1, 0, 4.5, 1.8);
    CHECK_APPROX(bodyOverlapDepth(a, body(2, 0, 1.5, 1, 0, 4.5, 1.8)), 0.3, kEps);
    CHECK(bodyOverlapDepth(a, body(2, 0, 2.0, 1, 0, 4.5, 1.8)) == 0);
    // Nose to tail, 0.5 m into it: 0.5 m deep; across it at right angles, its nose 0.4 m into my side: 0.4 m.
    CHECK_APPROX(bodyOverlapDepth(a, body(3, 4.0, 0, 1, 0, 4.5, 1.8)), 0.5, kEps);
    CHECK_APPROX(bodyOverlapDepth(a, body(4, 0, 0.9 + 2.25 - 0.4, 0, -1, 4.5, 1.8)), 0.4, kEps);
    // A body turned 45 degrees whose side passes just clear of my corner: boxes square to the plan would overlap, the
    // rectangles do not.
    const double r = std::sqrt(0.5);
    CHECK(bodyOverlapDepth(a, body(5, 3.85, 2.5, r, r, 4.5, 1.8)) == 0);

    // The row: a pair touching for three ticks, gone, then back: two spells, four pair-ticks, by its kind.
    TouchSpell t;
    t.a.car = CarId(1);
    t.b.car = CarId(2);
    t.kind = TouchKind::Split;
    t.worstDepth = 0.3;
    TouchRow row;
    for (uint64_t k = 10; k < 13; ++k) row.note({t}, k, static_cast<double>(k) * kTick);
    row.note({}, 13, 13 * kTick);
    t.worstDepth = 0.5;
    row.note({t}, 14, 14 * kTick);
    CHECK(row.spells == 2u && row.pairTicks == 4u);
    CHECK(row.spellsByKind[static_cast<uint8_t>(TouchKind::Split)] == 2u);
    CHECK_APPROX(row.worstDepth, 0.5, kEps);
    CHECK(row.kept.size() == 1u && row.kept[0].firstTick == 10u && row.kept[0].lastTick == 12u);
    CHECK(row.open().size() == 1u && row.open()[0].firstTick == 14u);
    MeasureRows rows;
    rows.touches = row;
    const ReportRow rep = rowOf(rows, "bodies_touching");
    CHECK(rep.status == RowStatus::Fail);
    CHECK(rep.offenders.total == 2u);
    CHECK(!rep.offenders.kept.empty() && std::fabs(numberOf(rep.offenders.kept[0], "other_car") - 2) < kEps);
    CHECK(rowOf(MeasureRows(), "bodies_touching").status == RowStatus::Pass);
}

TEST_CASE(measure_4_speed_over_the_drivers_own_limit) {
    MeasureRows rows;
    // Car 6 at 13.8 m/s under a limit of 13.4: within the tolerance. Then a second at 15 m/s.
    for (int k = 0; k < 60; ++k)
        rows.overOwnLimit.note(CarId(6), 300 + k, (300 + k) * kTick, kTick, 13.8, 13.4, engine::PathId(4));
    CHECK(rows.overOwnLimit.carSeconds == 0);
    CHECK(rowOf(rows, "over_own_limit").status == RowStatus::Pass);
    for (int k = 0; k < 60; ++k)
        rows.overOwnLimit.note(CarId(6), 360 + k, (360 + k) * kTick, kTick, k == 30 ? 16.0 : 15.0, 13.4,
                               engine::PathId(4));
    const ReportRow r = rowOf(rows, "over_own_limit");
    CHECK_APPROX(rows.overOwnLimit.carSeconds, 1.0, 1e-9);
    CHECK_APPROX(rows.overOwnLimit.checkedCarSeconds, 2.0, 1e-9);
    CHECK_APPROX(rows.overOwnLimit.worstExcess, 16.0 - 13.4, 1e-9);
    CHECK(r.status == RowStatus::Fail);
    // One spell, one offender, showing its worst moment.
    CHECK(r.offenders.total == 1u);
    CHECK(r.offenders.kept[0].car == CarId(6) && r.offenders.kept[0].tick == 390u);
    CHECK_APPROX(numberOf(r.offenders.kept[0], "speed"), 16.0, kEps);
    CHECK_APPROX(numberOf(r.offenders.kept[0], "my_limit"), 13.4, kEps);
}

TEST_CASE(measure_5_a_bend_taken_too_fast) {
    constexpr double kComfortable = 2.0;   // m/s2
    constexpr double kRadius = 7.75;       // the test grid's near turn
    // sqrt(2 * 7.75) = 3.94 m/s is comfortable; 6 m/s is 4.65 m/s2 sideways.
    CHECK_APPROX(sideAcceleration(6, kRadius), 36 / kRadius, kEps);
    CHECK(sideAcceleration(6, std::numeric_limits<double>::infinity()) == 0);
    MeasureRows rows;
    rows.bends.note(CarId(2), 500, 8.0, kTick, 3.9, kRadius, kComfortable);
    rows.bends.note(CarId(2), 501, 8.0 + kTick, kTick, 6.0, kRadius, kComfortable);
    const ReportRow r = rowOf(rows, "bend_speed");
    CHECK(r.status == RowStatus::Fail);
    CHECK(r.offenders.total == 1u && r.offenders.kept[0].car == CarId(2) && r.offenders.kept[0].tick == 501u);
    CHECK_APPROX(numberOf(r.offenders.kept[0], "side_accel"), 36 / kRadius, kEps);
    CHECK_APPROX(rows.bends.carSeconds, kTick, kEps);
    // A row that passes still says its worst figure (#281).
    MeasureRows calm;
    calm.bends.note(CarId(2), 500, 8.0, kTick, 3.9, kRadius, kComfortable);
    CHECK(rowOf(calm, "bend_speed").status == RowStatus::Pass);
    CHECK_APPROX(calm.bends.worstSideAccel, 3.9 * 3.9 / kRadius, kEps);
}

TEST_CASE(measure_6_standing_with_no_reason_for_25_seconds) {
    MeasureRows rows;
    // Car 11 waits for no reason from tick 600 (10 s) for 25 s, then something holds it again.
    constexpr int kStart = 600, kTicks = 25 * 60;
    for (int k = 1; k <= kStart; ++k) rows.noReason.note(CarId(11), k, k * kTick, kTick, false);
    for (int k = kStart + 1; k <= kStart + kTicks; ++k) rows.noReason.note(CarId(11), k, k * kTick, kTick, true);
    rows.noReason.note(CarId(11), kStart + kTicks + 1, (kStart + kTicks + 1) * kTick, kTick, false);
    const ReportRow r = rowOf(rows, "standing_no_reason");
    CHECK(rows.noReason.spells == 1u);
    CHECK_APPROX(rows.noReason.carSeconds, 25.0, 1e-6);
    CHECK_APPROX(rows.noReason.longestSpell, 25.0, 1e-6);
    CHECK(r.status == RowStatus::Fail);
    CHECK(r.offenders.total == 1u && r.offenders.kept[0].car == CarId(11));
    CHECK(r.offenders.kept[0].tick == static_cast<uint64_t>(kStart + 1));
    CHECK_APPROX(numberOf(r.offenders.kept[0], "spell_s"), 25.0, 1e-6);
}

TEST_CASE(measure_7_deadlock_and_gridlock_loops) {
    MeasureRows rows;
    // A deadlock of cars 9, 3 and 7 (any order: the same set), looked at once a second from 5 s to 15 s.
    for (int s = 5; s <= 15; ++s) {
        const std::vector<uint32_t> cars = s % 2 ? std::vector<uint32_t>{3, 7, 9} : std::vector<uint32_t>{7, 9, 3};
        rows.loops.note({loopOf(cars, LoopClass::Deadlock)}, static_cast<uint64_t>(s) * 60, s);
    }
    rows.loops.note({}, 16 * 60, 16);
    // A gridlock of four, for three looks; then a fifth car joins it: a new loop.
    for (int s = 20; s <= 22; ++s) rows.loops.note({loopOf({20, 21, 22, 23}, LoopClass::Gridlock)}, s * 60, s);
    rows.loops.note({loopOf({20, 21, 22, 23, 24}, LoopClass::Gridlock)}, 23 * 60, 23);

    CHECK(rows.loops.deadlock.loops == 1u);
    CHECK_APPROX(rows.loops.deadlock.longest, 10.0, kEps);
    CHECK(rows.loops.deadlock.mostCars == 3u);
    const ReportRow dead = rowOf(rows, "deadlock_loops");
    CHECK(dead.status == RowStatus::Fail);
    CHECK(dead.offenders.total == 1u && dead.offenders.kept[0].car == CarId(3) && dead.offenders.kept[0].tick == 300u);
    CHECK_APPROX(numberOf(dead.offenders.kept[0], "lasted_s"), 10.0, kEps);
    CHECK_APPROX(numberOf(dead.offenders.kept[0], "cars"), 3, kEps);

    CHECK(rows.loops.gridlock.loops == 2u);
    CHECK_APPROX(rows.loops.gridlock.longest, 2.0, kEps);
    CHECK(rows.loops.gridlock.mostCars == 5u);
    const ReportRow grid = rowOf(rows, "gridlock_loops");
    CHECK(grid.status == RowStatus::NoTarget);
    CHECK(grid.offenders.total == 2u && grid.offenders.kept[0].car == CarId(20) &&
          grid.offenders.kept[0].tick == 1200u);

    // A gridlock alone fails nothing.
    MeasureRows onlyGrid;
    onlyGrid.loops.note({loopOf({1, 2}, LoopClass::Gridlock)}, 60, 1);
    CHECK(buildReport(onlyGrid, {}).passed());
}

TEST_CASE(measure_8_crossing_a_counting_line) {
    const engine::PathId lane(4), next(5), before(3);
    PathPoint line;
    line.path = lane;
    line.along = 20;
    auto point = [](engine::PathId p, double along) {
        PathPoint x;
        x.path = p;
        x.along = along;
        return x;
    };
    double reach = 0;
    // On the line's path, from short of it to past it.
    CHECK(crossesLine(point(lane, 19.9), point(lane, 20.1), 0.2, line, &reach));
    CHECK_APPROX(reach, 0.1, kEps);
    // Short of it, and still short; already past it.
    CHECK(!crossesLine(point(lane, 19.0), point(lane, 19.5), 0.5, line, &reach));
    CHECK(!crossesLine(point(lane, 20.0), point(lane, 20.5), 0.5, line, &reach));
    // Off the end of its path onto the next.
    CHECK(crossesLine(point(lane, 19.9), point(next, 0.3), 0.4, line, &reach));
    CHECK_APPROX(reach, 0.1, kEps);
    // From the path before, past a line at a path's start.
    PathPoint atStart = line;
    atStart.along = 0;
    CHECK(crossesLine(point(before, 99.8), point(lane, 0.1), 0.3, atStart, &reach));
    CHECK_APPROX(reach, 0.2, kEps);
}

// THE QUEUE FIGURES, worked by hand.
//
// A clean queue: car 1 stood 1 m short of the line, began to move at 10.0 s and crossed at 12.0 s (a first-car delay of
// 2.0 s). Cars 2 to 6, each moving off a little after the one ahead, crossed at 14.5, 16.6, 18.5, 20.4 and 22.3 s:
// gaps of 2.5, 2.1, 1.9, 1.9, 1.9 s. From the fourth car on (the gap before car 4, 5 and 6) that is 1.9 s per car.
//
// A straggler: car 7 crossed at 24.0 s, 1.7 s after car 6, but it had been moving since the start (it began to move at
// 0 s, before car 1 did): it was never in the queue, so it ends it uncounted. The figures are the same.
//
// Two queues at one line: after the first, car 10 drove over at 40.0 s without having stopped (not a queue), then the
// line was quiet until car 11, at rest 0.5 m short, moved off at 58.4 s and crossed at 60.0 s (a delay of 1.6 s); cars
// 12 to 15 crossed at 62.4, 64.4, 66.2, 68.0 s: gaps of 2.4, 2.0, 1.8, 1.8. From the fourth car on: 1.8 and 1.8.
// Together: (1.9 * 3 + 1.8 * 2) / 5 = 1.86 s per car over 5 gaps; first-car delay (2.0 + 1.6) / 2 = 1.8 s.
TEST_CASE(measure_9_queue_figures_by_hand) {
    std::vector<LineCrossing> clean = {crossing(1, 12.0, 10.0, 1.0), crossing(2, 14.5, 10.8, 7.5),
                                       crossing(3, 16.6, 11.6, 14.0), crossing(4, 18.5, 12.4, 20.5),
                                       crossing(5, 20.4, 13.2, 27.0), crossing(6, 22.3, 14.0, 33.5)};
    std::vector<QueueBurst> bursts = findQueueBursts(clean);
    CHECK(bursts.size() == 1u);
    CHECK(bursts[0].firstCar == CarId(1) && bursts[0].gaps.size() == 5u);
    QueueFigures f = queueFigures(bursts);
    CHECK_APPROX(f.secondsPerCar, 1.9, 1e-9);
    CHECK(f.gaps == 3u && f.bursts == 1u);
    CHECK_APPROX(f.firstCarDelay, 2.0, 1e-9);

    std::vector<LineCrossing> straggler = clean;
    straggler.push_back(crossing(7, 24.0, 0.0, 300.0));
    bursts = findQueueBursts(straggler);
    CHECK(bursts.size() == 1u && bursts[0].gaps.size() == 5u);
    f = queueFigures(bursts);
    CHECK_APPROX(f.secondsPerCar, 1.9, 1e-9);
    CHECK(f.gaps == 3u);

    std::vector<LineCrossing> two = straggler;
    two.push_back(crossing(10, 40.0, 0.0, 500.0));
    for (const LineCrossing& x : {crossing(11, 60.0, 58.4, 0.5), crossing(12, 62.4, 59.0, 7.0),
                                  crossing(13, 64.4, 59.6, 13.5), crossing(14, 66.2, 60.2, 20.0),
                                  crossing(15, 68.0, 60.8, 26.5)})
        two.push_back(x);
    bursts = findQueueBursts(two);
    CHECK(bursts.size() == 2u);
    CHECK(bursts.size() == 2u && bursts[1].firstCar == CarId(11) && bursts[1].gaps.size() == 4u);
    f = queueFigures(bursts);
    CHECK_APPROX(f.secondsPerCar, 1.86, 1e-9);
    CHECK(f.gaps == 5u && f.bursts == 2u);
    CHECK_APPROX(f.firstCarDelay, 1.8, 1e-9);

    // In the table: 1.86 s is inside the band, 1.8 s too.
    MeasureRows rows;
    rows.queues = bursts;
    rows.queueLine = {0, 0};
    CHECK(rowOf(rows, "queue_headway").status == RowStatus::Pass);
    CHECK(rowOf(rows, "queue_start_up").status == RowStatus::Pass);
    // No queue at all: nothing to see, which fails nothing.
    CHECK(rowOf(MeasureRows(), "queue_headway").status == RowStatus::NoData);
    CHECK(!std::isfinite(queueFigures({}).secondsPerCar));
}

TEST_CASE(measure_10_describe_and_json) {
    MeasureRows rows;
    rows.ticks = 120;
    rows.seconds = 2.0;
    rows.drivenCars = 4;
    rows.jumps.note(CarId(3), 12, 1.2, 5.0, 1, 1, 0.1);   // allowed 0.1 + 0.01 m: 4.89 m too far
    const MeasurementReport rep = buildReport(rows, {});
    CHECK(!rep.passed());
    const std::string text = rep.describe();
    std::printf("%s", text.c_str());
    // The passing row, and the failing row with its offender under it.
    const std::string passing =
        "  pass              Standing still with no reason: 0.0 car-seconds in 0 spells; the longest 0.0 s. Target: "
        "none: every driven car at rest is waiting for something it can name.\n";
    const std::string failing =
        "  FAIL              Jumps (a front moving further in a tick than its speed could carry it): 1 car-ticks; the "
        "worst 4.890 m further than its speed allows. Target: none.\n"
        "      car 3 at tick 12 (1.20 s): moved 5.000 m, speed before 1.000 m/s, speed after 1.000 m/s\n";
    CHECK(text.find(passing) != std::string::npos);
    CHECK(text.find(failing) != std::string::npos);
    CHECK(text.find("Driving measurements over 120 ticks (2.0 s simulated), 4 cars driven at the last tick: FAILED\n") ==
          0u);
    CHECK(text.find("not measured yet  Lane changes: indicator on before the move: not measured yet (needs lane "
                    "changes).") != std::string::npos);

    const std::filesystem::path file = std::filesystem::temp_directory_path() / "driving_measure_test.json";
    std::string why;
    CHECK(rep.writeJson(file.string(), &why));
    std::ifstream in(file);
    const nlohmann::json j = nlohmann::json::parse(in, nullptr, false);
    CHECK(!j.is_discarded());
    CHECK(j["ticks"] == 120 && j["passed"] == false && j["driven_cars"] == 4);
    const nlohmann::json& jumps = j["rows"]["jumps"];
    CHECK(jumps["status"] == "fail");
    CHECK(jumps["values"]["ticks"] == 1.0);
    CHECK(std::fabs(jumps["values"]["worst_excess_m"].get<double>() - 4.89) < 1e-9);
    CHECK(jumps["offenders_total"] == 1);
    CHECK(jumps["offenders"][0]["car"] == 3 && jumps["offenders"][0]["tick"] == 12);
    CHECK(jumps["offenders"][0]["numbers"]["moved_m"] == 5.0);
    const nlohmann::json& standing = j["rows"]["standing_no_reason"];
    CHECK(standing["status"] == "pass");
    CHECK(standing["values"]["car_seconds"] == 0.0 && standing["values"]["spells"] == 0.0);
    CHECK(standing["offenders"].is_array() && standing["offenders"].empty());
    const nlohmann::json& headway = j["rows"]["queue_headway"];
    CHECK(headway["status"] == "no_data");
    CHECK(headway["values"]["seconds_per_car"].is_null());
    CHECK(headway["values"]["gaps"] == 0.0);
    CHECK(j["rows"]["lane_change_durations"]["status"] == "not_measured");
    std::filesystem::remove(file);
}
