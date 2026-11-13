#ifndef RAYTRACER_APPS_DRIVING_MEASURE_H
#define RAYTRACER_APPS_DRIVING_MEASURE_H

// THE MEASUREMENTS TABLE (driving-model-plan.md "Measurements (the lab is the judge)"; driving-model-design.md 13;
// issue #257): while a simulation runs, this module counts what the plan says the model is judged by -- cars standing
// still for no reason, loops of waiting cars, jumps, turns tighter than a car can make, stops inside junctions, bodies
// overlapping, speeding, bends taken too fast, how a queue empties, what a tick costs -- and reports it as a table a
// person can read (describe) and a file a script can read (writeJson).
//
// IT ONLY WATCHES. A host (or a test) attaches a Measurements to a Driving (Driving::attachMeasurements); after each
// tick it is shown the world read-only and updates its counts. It keeps a little memory of its own per car (where each
// car was last tick). Nothing it holds is read by the tick, and the driving state hash is the same at every tick with
// it attached or not.
//
// EVERY ROW IS A COUNT OR A SPREAD, WITH THE CARS THAT BROKE IT: for each row that can be broken, the first
// kOffendersKept offenders are kept, each with its car, the tick and clock time, and the two or three numbers that
// show the breach, so a failed row points at a car and a moment.
//
// THE ARITHMETIC IS PURE: each row's test (is this a jump, is this turn too tight, the queue figures from crossing
// times, the loop bookkeeping from a run of wait reports) is a function or a small class over plain numbers, so a test
// can stage a breach the model itself can no longer produce. Measurements only reads the world and feeds them.
//
// Same result every run: cars are walked in car-id order, paths in path order, and every map here is ordered. The one
// exception is the wall-clock cost row, which the caller times (tickTimed) and which is in no hash or comparison.
//
// THE FILE (writeJson). Key names are a promise to the scripts that read it; change them only with the scripts:
//   {
//     "ticks": <ticks watched>, "seconds": <simulated seconds watched>, "driven_cars": <driven at the last tick>,
//     "passed": <true when no row failed>,
//     "rows": {
//       "<row key>": {
//         "title": <the row in words>, "status": "pass" | "fail" | "no_target" | "no_data" | "not_measured",
//         "target": <the target in words>,
//         "values": { "<value key>": <number, or null when there is nothing to give> , ... },
//         "offenders_total": <how many breaches in all>,
//         "offenders": [ { "car": <id>, "tick": <tick>, "seconds": <clock>, "numbers": { "<key>": <number> } } ]
//       }, ...
//     }
//   }
// The rows and their value keys:
//   standing_no_reason      car_seconds, longest_spell_s, spells               (offender numbers: spell_s)
//   endless_waits           car_seconds, longest_spell_s, spells               (offender numbers: spell_s)
//   no_progress             car_seconds, unexcused_s, longest_spell_s, spells  (offender numbers: spell_s)
//   deadlock_loops         loops, longest_s, most_cars                        (offender numbers: cars, lasted_s)
//   gridlock_loops          loops, longest_s, most_cars                        (offender numbers: cars, lasted_s)
//   jumps                   ticks, worst_excess_m                  (offender numbers: moved_m, speed_before, speed_after)
//   tight_turns             too_tight, pivots, worst_ratio         (offender numbers: turn_per_m, allowed_per_m, moved_m;
//                                                                    a turn at rest: angle_rad, moved_m)
//   junction_stops          car_seconds, behind_a_car_s, at_a_stop_s, top_speed_s, way_end_s, nothing_s, and one
//                           stop_rule:<rule name>_s per rule that stopped a car on a connector (no offenders)
//   box_stops               car_seconds, not_allowed_s, and <reason>_s for each BoxReason key (giving_way_s,
//                           behind_giving_way_s, behind_moving_in_box_s, behind_other_s, waiting_for_room_s,
//                           stop_by_rule_s, top_speed_s,
//                           way_end_s, nothing_s), and one stop_rule:<rule name>_s per rule  (offender numbers: reason,
//                           binding_car)
//   overlaps                pair_ticks, pairs, worst_overlap_m               (offender numbers: other_car, overlap_m, path)
//   bodies_touching         pair_ticks, spells, worst_depth_m, and <kind>_spells for each TouchKind key (one_path,
//                           split, in_junction, lane_and_connector, two_junctions, no_record_paths_close,
//                           no_record_body_swing)   (offender numbers: other_car, depth_m, kind, path, other_path)
//   over_own_limit          car_seconds, checked_car_seconds, worst_excess_mps  (offender numbers: speed, my_limit, path)
//   bend_speed              car_seconds, checked_car_seconds, worst_side_accel  (offender numbers: speed, radius_m,
//                           side_accel)
//   queue_headway           seconds_per_car, gaps, bursts             (offender numbers: cars, seconds_per_car, line)
//   queue_start_up          first_car_delay_s, bursts                 (offender numbers: first_car_delay_s, line)
//   tick_cost               ms_per_1000_average, ms_per_1000_worst, ticks_timed  (no offenders)
//   caps_hit                tails_off_trail, look_aheads_cut                     (no offenders)
//   lane_change_indicators, lane_change_durations, lane_change_gaps, crashes_with_personalities   (not measured yet;
//                           no values)
// Units are in the key where a key has one (_s seconds, _m metres, _mps m/s); speed is m/s, side_accel m/s2,
// turn_per_m and allowed_per_m radians per metre, worst_ratio a plain ratio.
//
// Units: metres, seconds.

#include "apps/driving/driving.h"
#include "apps/driving/notice.h"
#include "apps/driving/traffic_state.h"
#include "apps/driving/wait_graph.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace driving {

// ---- The tolerances and targets, in one place ----------------------------------------------------------------------

// How many offenders each row keeps (the first ones; the total is counted beyond).
constexpr std::size_t kOffendersKept = 16;

// JUMPS: a front may move by its speed times the tick (the larger of its speeds before and after, since the move
// step moves by the new speed), plus this. A centimetre: far above the rounding in reading two points off the curve,
// far below any jump a person would see.
constexpr double kJumpToleranceMetres = 0.01;

// TURNING: the body's direction is read as the chord from its rear to its front. Through a bend that chord is a
// little shorter than the body, so it turns a few percent faster than the curve does (4% for an ordinary car on its
// own tightest circle). A turn counts as too tight beyond the allowed rate times (1 + this).
constexpr double kTurnToleranceShare = 0.10;
// A change of direction smaller than this (radians) is rounding, not a turn: a millionth of a radian.
constexpr double kAngleNoiseRadians = 1e-6;
// A front that moved less than this in a tick (metres) has not moved: a turn then is a turn at rest (a pivot).
constexpr double kMovedNoiseMetres = 1e-9;

// OVERLAPS: two bodies on one path whose stretches overlap by more than this (metres). A micrometre: only rounding.
constexpr double kOverlapToleranceMetres = 1e-6;

// SPEED OVER THE DRIVER'S OWN LIMIT, by more than this (m/s). Drivers decide ten times a second, so a car passing into
// a lower limit can still be shedding the last fraction of a m/s; half a m/s (about 1 mph) is under what a
// speedometer shows.
constexpr double kOverLimitToleranceMps = 0.5;

// BENDS: the sideways acceleration (speed squared over the bend's radius) may pass the rule set's
// comfortable_side_accel by this (m/s2) before it counts: enough for the tenth of a second between decisions, small
// against the couple of m/s2 a comfortable bend allows.
constexpr double kSideAccelToleranceMps2 = 0.25;

// LOOPS: the wait report is taken once per this many simulated seconds (design 9.6: "the lab builds the wait-for
// graph each second"), not every tick.
constexpr double kLoopSampleSeconds = 1.0;

// QUEUES at a counting line.
// A line that has seen no crossing for this long is quiet: the next car to cross from rest nearby starts a queue. Ten
// seconds is longer than any gap inside a queue emptying (about 2 s) and shorter than a red light.
constexpr double kQueueQuietSeconds = 10.0;
// "Nearby": the first car of a queue was at rest no further back from the line than this (metres): two car lengths,
// so a first car that stopped a little short of the line still counts.
constexpr double kQueueStartNearMetres = 10.0;
// The steady rate is read from the gap before this car of the queue on (the first few cars are still getting going;
// traffic engineers read it from the fourth or fifth).
constexpr std::size_t kSaturationFromCar = 4;
// THE PLAN'S BANDS. One car every ~1.9 seconds once the queue is moving (FHWA's saturation flow, about 1,900 cars an
// hour per lane): 1.7 to 2.1 s, a tenth either side. (The first car loses about 2 seconds getting going, counted from
// the green: the start-up row counts from moving off, so it holds no band until it is counted from the green; #284.)
constexpr double kHeadwayLowest = 1.7;
constexpr double kHeadwayHighest = 2.1;

// ---- What the report holds -----------------------------------------------------------------------------------------

// ONE NUMBER, with its file key and the words describe() prints before and after it ("moved 5.000 m").
struct NamedNumber {
    std::string key;
    double value = 0;
    std::string label;
    std::string unit;
};

// ONE CAR THAT BROKE A ROW, and when.
struct Offender {
    CarId car;
    uint64_t tick = 0;
    double seconds = 0;
    std::vector<NamedNumber> numbers;
};

// THE FIRST FEW OFFENDERS, and how many in all.
struct Offenders {
    std::vector<Offender> kept;
    uint64_t total = 0;
    // Counts it, and keeps it while fewer than kOffendersKept are kept. The index it was kept at, or kNotKept.
    static constexpr std::size_t kNotKept = static_cast<std::size_t>(-1);
    std::size_t offer(const Offender& o);
};

enum class RowStatus : uint8_t {
    Pass,          // it has a target and met it
    Fail,          // it has a target and missed it
    NoTarget,      // measured; the plan gives no target yet
    NoData,        // it has a target, but this run gave nothing to measure (no queue at any line)
    NotMeasured,   // the model cannot produce it yet (lane changes, personalities)
};
constexpr uint8_t kRowStatusCount = 5;
const char* rowStatusKey(RowStatus s);     // the file's word: "pass", "fail", "no_target", "no_data", "not_measured"
const char* rowStatusWords(RowStatus s);   // the table's words: "pass", "FAIL", "no target yet", ...

// ONE ROW OF THE TABLE.
struct ReportRow {
    std::string key;                  // the file's key (the list at the top of this file)
    std::string title;                // what it measures, in words
    RowStatus status = RowStatus::NoTarget;
    std::string target;               // the target in words
    std::string summary;              // what was measured, in words
    std::vector<NamedNumber> values;  // the same as numbers (a value with no data is not finite: null in the file)
    Offenders offenders;
};

// THE WHOLE TABLE, rows in a fixed order.
struct MeasurementReport {
    uint64_t ticks = 0;          // ticks watched
    double seconds = 0;          // simulated seconds watched
    uint32_t drivenCars = 0;     // driven cars at the last tick watched
    std::vector<ReportRow> rows;

    bool passed() const;                          // no row failed
    const ReportRow* row(const std::string& key) const;
    // Plain text, one row a line, the offenders listed under a failing row.
    std::string describe() const;
    // The same as JSON (the keys at the top of this file). False, with the reason in `why`, when it cannot be written.
    bool writeJson(const std::string& path, std::string* why = nullptr) const;
};

// ---- The pure parts: each row's arithmetic over plain numbers ------------------------------------------------------

// Did a front that moved `moved` metres in a tick of `dt` seconds, at `speedBefore` then `speedAfter`, jump? How far
// past what its speed could carry it is `excess` (when given).
bool isJump(double moved, double speedBefore, double speedAfter, double dt, double* excess = nullptr);
// The turn of a body over a tick: its direction changed by `angle` radians while its front moved `moved` metres.
// Too tight: it moved, and turned faster per metre than one over `tightestRadius` allows (with the tolerance).
bool turnsTooTight(double angle, double moved, double tightestRadius);
// A pivot: its direction changed while its front did not move.
bool pivotsAtRest(double angle, double moved);
// The angle between two directions on the plan, (x, z) each (radians, 0 to pi).
double planAngle(double ax, double az, double bx, double bz);
// Sideways acceleration round a bend of `radius` at `speed` (a bend with no finite positive radius: 0).
double sideAcceleration(double speed, double radius);

// THE OVERLAPPING PAIRS on one path's list (entries in list order). A pair is two different cars whose stretches
// overlap by more than kOverlapToleranceMetres, NEITHER of them listed Beside (a car part-way through a lane change is
// level with the lane's stretch but not in it). Each pair once, lower car first, in the order found.
struct OverlapPair {
    CarId a, b;
    double overlap = 0;   // metres
};
std::vector<OverlapPair> overlapsOn(const PathList& list);

// WHETHER A FRONT CROSSED A COUNTING LINE this tick: it was on `before` and is now on `now`, having moved `moved`
// metres (a front only moves forward along its way). True when it was short of the line on the line's path and is now
// at or past it (or has moved on to a later path), or came onto the line's path and is now at or past it. `beforeLine`
// gets how far it moved to reach the line. (A line on a path the front crossed whole within one tick is not seen; no
// path is that short at a sane tick.)
bool crossesLine(const PathPoint& before, const PathPoint& now, double moved, const PathPoint& line,
                 double* beforeLine);

// ONE CAR'S FRONT CROSSING A COUNTING LINE.
struct LineCrossing {
    CarId car;
    uint64_t tick = 0;
    double seconds = 0;          // when it crossed (between the ticks, by distance)
    // The last time it began to move from rest (a negative number: never seen at rest), and how far it then was from
    // the line (metres).
    double restLeftAt = -1;
    double restDistance = 0;
};

// ONE QUEUE EMPTYING over a line: after kQueueQuietSeconds with no crossing, a car crossed that began to move from rest
// no more than kQueueStartNearMetres short of the line (the first car). The cars after it are in the queue while each
// crosses less than kQueueQuietSeconds after the one before AND began to move from rest no earlier than the first car
// did (it was standing in the queue when the first car moved off). The first crossing that fails ends the queue and is
// not counted: a straggler that never stopped is not part of it.
struct QueueBurst {
    CarId firstCar;
    uint64_t tick = 0;              // when the first car crossed
    double seconds = 0;
    double firstCarDelay = 0;       // from the first car beginning to move to its front crossing the line
    std::vector<double> gaps;       // gaps[i]: from car i + 1's crossing to car i + 2's (cars counted from 1)
};
// The queues in one line's crossings (in the order they crossed).
std::vector<QueueBurst> findQueueBursts(const std::vector<LineCrossing>& crossings);

// THE QUEUE FIGURES over any number of queues: the average gap from car kSaturationFromCar on (the gap before it, the
// gap before the next, ...), and the average first-car delay. Not finite when there is nothing to average.
struct QueueFigures {
    double secondsPerCar = 0;       // traffic engineers' "saturation headway"
    std::size_t gaps = 0;           // how many gaps went into it
    double firstCarDelay = 0;       // ... their "start-up lost time", as the plan words it
    std::size_t bursts = 0;
};
QueueFigures queueFigures(const std::vector<QueueBurst>& bursts);

// ---- The rows that keep counts, each fed plain numbers -------------------------------------------------------------

// STANDING STILL FOR NO REASON: per car, spells of waiting with nothing named as the reason.
struct NoReasonRow {
    double carSeconds = 0;
    double longestSpell = 0;
    uint64_t spells = 0;
    Offenders offenders;            // one per spell (spell_s grows while it lasts)
    // `noReason`: the car is waiting with no reason now. `dt`: the seconds since the last look.
    void note(CarId car, uint64_t tick, double seconds, double dt, bool noReason);
    void forget(CarId car);         // the car has left the road

private:
    struct Open {
        double start = 0;
        std::size_t offender = Offenders::kNotKept;
    };
    std::map<CarId, Open> open_;
};

// LOOPS OF WAITING CARS, from the wait report taken once a second. A loop is the same loop while its set of cars is
// unchanged from one look to the next.
struct LoopRow {
    struct PerClass {
        uint64_t loops = 0;          // distinct loops seen
        double longest = 0;          // the longest any lasted (from its first look to its last), seconds
        uint64_t mostCars = 0;
        Offenders offenders;         // one per loop: its lowest car, when first seen; cars, lasted_s
    };
    PerClass gridlock, deadlock;
    void note(const std::vector<WaitLoop>& loops, uint64_t tick, double seconds);

private:
    struct Open {
        LoopClass kind = LoopClass::Gridlock;
        double first = 0;
        std::size_t offender = Offenders::kNotKept;
    };
    std::map<std::vector<uint32_t>, Open> open_;   // by the loop's car ids, sorted
};

// JUMPS.
struct JumpRow {
    uint64_t ticks = 0;
    double worstExcess = 0;
    Offenders offenders;
    void note(CarId car, uint64_t tick, double seconds, double moved, double speedBefore, double speedAfter, double dt);
};

// TURNING TIGHTER THAN THE CAR CAN, and turning at rest.
struct TurnRow {
    uint64_t tooTight = 0;
    uint64_t pivots = 0;
    double worstRatio = 0;           // the fastest turn seen over what the car's turning circle allows
    Offenders offenders;
    void note(CarId car, uint64_t tick, double seconds, double angle, double moved, double tightestRadius);
};

// STOPPED ON A CONNECTOR (in a junction), by what held the car: car-seconds per kind of limit, and per rule for stops.
struct JunctionStopRow {
    double carSeconds = 0;
    double byLimit[kLimitCount] = {0, 0, 0, 0, 0};
    std::map<RuleIndex, double> byStopRule;
    void note(const Binding& binding, double dt);
};

// AT REST IN A JUNCTION BOX (issue #264, the D5 brief's "done when"): a car waiting (wait_graph.h isWaiting) with any
// part of its body in a box -- on a connector by more than at_line_within, or across a crosswalk -- by why. Read from
// what held it, following a car held behind another to the head of its queue:
enum class BoxReason : uint8_t {
    GivingWay,         // a stop that gives way to a car (StopFor::GivingWay): a recorded give-way, inside the junction or
                       // waiting inside for a far turn. Allowed.
    BehindGivingWay,   // behind a car (and so on) whose head is itself in a box giving way. Allowed: the give-way holds it.
    BehindMovingInBox, // behind a car (and so on) moving off inside a box: a queue inside, starting off. Allowed.
    BehindOther,       // behind a car whose head is held by anything else, or moving off beyond the box: a queue
                       // reaching back into the box
    WaitingForRoom,    // a stop for room beyond (StopFor::NoRoom) with its body already in a box
    StopByRule,        // a stop by any other rule (a red, a yellow, a full stop, its destination, ...): by rule below
    TopSpeed,          // held by a top speed
    WayEnd,            // at the end of its way
    Nothing,           // held by nothing at all
};
constexpr uint8_t kBoxReasonCount = 9;
const char* boxReasonKey(BoxReason r);     // "giving_way", "behind_giving_way", ...
const char* boxReasonWords(BoxReason r);   // in words
bool boxReasonAllowed(BoxReason r);        // GivingWay, BehindGivingWay and BehindMovingInBox
// IS ANY PART OF THE CAR'S BODY IN A JUNCTION BOX: over a connector by more than `atLineWithin`, or over a crosswalk.
bool bodyInBox(const engine::LaneGraph& roads, const VehicleClasses& classes, const Car& car, Real atLineWithin);
// WHY A WAITING CAR IN A BOX IS THERE (see BoxReason). `inBox` says, for a car record, whether its body is in a box.
BoxReason boxReason(const std::vector<Car>& cars, const PathLists& lists, const Car& car,
                    bool (*inBox)(const Car& car, const void* context), const void* context);

struct BoxStopRow {
    double carSeconds = 0;
    double byReason[kBoxReasonCount] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::map<RuleIndex, double> byStopRule;   // StopByRule: car-seconds by the rule that stopped it
    double notAllowed = 0;                    // car-seconds for a reason that is not allowed
    Offenders offenders;                      // one per spell of a car at rest in a box for a reason not allowed
    void note(CarId car, uint64_t tick, double seconds, double dt, BoxReason why, const Binding& binding);
    void notInBox(CarId car);

private:
    std::set<uint32_t> bad_;   // cars at rest in a box for a reason not allowed, last tick
};

// OVERLAPPING BODIES.
struct OverlapRow {
    uint64_t pairTicks = 0;
    double worstOverlap = 0;
    std::set<std::pair<uint32_t, uint32_t>> pairs;   // distinct pairs, lower car first
    Offenders offenders;             // one per spell of a pair overlapping (a new spell: not overlapping last tick)
    // All the pairs overlapping on this tick, with the path each was found on, in path order; a pair overlapping on
    // two paths at once is one pair-tick.
    void note(const std::vector<std::pair<OverlapPair, engine::PathId>>& found, uint64_t tick, double seconds);

private:
    std::set<std::pair<uint32_t, uint32_t>> lastTick_;
};

// ---- Body against body (issue #283) --------------------------------------------------------------------------------
//
// The overlaps row above looks along each path's list, so it cannot see two cars on DIFFERENT paths whose bodies touch.
// This one knows nothing about paths: every tick, every pair of car bodies as the host draws them -- a rectangle on the
// plan, its middle halfway between the bumpers carPose gives, pointing from the rear bumper to the front, as long and
// wide as the drawn vehicle -- is tested for overlap. Only afterwards is each overlap SORTED by how the two cars' paths
// relate, so the count says where to look.

// A CAR'S BODY ON THE PLAN, as the host draws it.
struct PlanBody {
    CarId car;
    double x = 0, z = 0;            // its middle
    double ux = 1, uz = 0;          // unit, from its rear to its front
    double halfLength = 0, halfWidth = 0;
    double height = 0;              // the road's height under its middle
};
// The drawn body of a car at `pose`, `length` long and `width` wide (the host's drawing centres the vehicle between the
// bumpers and points it from the rear bumper to the front: citysim DrivingHost::mirror). The lane graph's own body rule
// (engine::bodyOnPlan), the one the road builder sweeps along each path for its crossing points (ADR-0412).
PlanBody planBody(CarId car, const CarPose& pose, double length, double width);
// HOW DEEP TWO BODIES OVERLAP on the plan: the shortest push that would part them (metres; separating axes over the
// two rectangles), or zero when they do not overlap.
double bodyOverlapDepth(const PlanBody& a, const PlanBody& b);
// Two bodies whose roads differ in height by more than this are on different decks (a street under a bridge), not
// touching: the road builder's same-level figure (Rules::sameLevelDz), which the crossing points use too.
constexpr double kTouchSameLevelMetres = 1.0;
// Overlaps shallower than this (metres) are rounding, not touching: a millimetre.
constexpr double kTouchToleranceMetres = 1e-3;

// HOW THE PATHS OF TWO TOUCHING CARS RELATE, the first that fits, in this order.
enum class TouchKind : uint8_t {
    OnePath,            // both bodies are listed on one path: following (or after a join) failed
    Split,              // their paths leave one lane: a split crossing point between them
    InJunction,         // a cross or join crossing point filed under one junction: the conflict resolver's to order
    LaneAndConnector,   // a crossing point filed under no junction, between a lane and a connector
    TwoJunctions,       // a crossing point filed under no junction, between connectors of two junctions
    NoRecordClose,      // no crossing point, yet the two bodies' stretches of path centre line come within the two
                        // half widths: the paths themselves are too close (two lanes side by side, or a pair the
                        // crossing stage does not pair)
    NoRecordSwing,      // no crossing point, and the centre lines stay apart: a body reaches outside its path's band
                        // (a long body through a bend lies on the chord, its corners swing out)
};
constexpr uint8_t kTouchKindCount = 7;
const char* touchKindKey(TouchKind k);     // "one_path", "split", ...
const char* touchKindWords(TouchKind k);   // in words

// ONE CAR IN A TOUCH, as it was when the touch began.
struct TouchSide {
    CarId car;
    engine::PathId path;            // its front's path
    Real along = 0;
    Real speed = 0;
    bool driven = false;
    Binding binding;
    bool drawnBigger = false;       // the host draws it larger than its vehicle class (the model's body)
};

// ONE SPELL OF TWO BODIES TOUCHING: from the first tick they overlap to the last.
struct TouchSpell {
    TouchSide a, b;                 // a the lower car id
    TouchKind kind = TouchKind::NoRecordSwing;
    uint32_t crossing = engine::kNoIndex;   // the crossing point that sorted it (a record's kinds), else none
    uint32_t junction = engine::kNoIndex;   // ... and the junction it is filed under
    double x = 0, z = 0;            // where, when it began: between the two middles
    double bandGap = 0;             // the closest the two stretches of centre line came, less both half widths (m;
                                    // below zero: the bands overlap)
    uint64_t firstTick = 0, lastTick = 0;
    double firstSeconds = 0, lastSeconds = 0;
    double worstDepth = 0;
    double seconds() const { return lastSeconds - firstSeconds; }
};

// SORTS a touch between cars `a` and `b` (a the lower id): the kind, and the crossing point and junction that decided
// it (when one did), and the band gap.
TouchKind sortTouch(const engine::LaneGraph& roads, const VehicleClasses& classes, const PathLists& lists, const Car& a,
                    const Car& b, uint32_t* crossing, uint32_t* junction, double* bandGap);

// HOW MANY SPELLS the row keeps whole (every one is counted beyond).
constexpr std::size_t kTouchSpellsKept = 2048;

struct TouchRow {
    uint64_t pairTicks = 0;
    uint64_t spells = 0;
    double worstDepth = 0;
    uint64_t spellsByKind[kTouchKindCount] = {};
    uint64_t pairTicksByKind[kTouchKindCount] = {};
    double worstByKind[kTouchKindCount] = {};
    uint64_t spellsOneNotDriven = 0;   // spells where one car was not driven (left at its destination)
    uint64_t spellsOneAtRest = 0;      // ... where one car stood when it began
    uint64_t spellsDrawnBigger = 0;    // ... where one car is drawn larger than its class
    std::vector<TouchSpell> kept;      // the spells that have ended, the first kTouchSpellsKept
    Offenders offenders;               // one per spell (when it began)
    // This tick's touches (each as a spell of one tick: its depth now in worstDepth), lower car first, in pair order.
    void note(const std::vector<TouchSpell>& now, uint64_t tick, double seconds);
    // The spells still going (for a report at the end of a run), in pair order.
    std::vector<TouchSpell> open() const;

private:
    std::map<std::pair<uint32_t, uint32_t>, TouchSpell> open_;
};

// FASTER THAN THE DRIVER'S OWN LIMIT (my_limit) by more than kOverLimitToleranceMps. One offender per spell over,
// showing the worst moment of it.
struct OverLimitRow {
    double carSeconds = 0;
    double checkedCarSeconds = 0;    // car-seconds that had a limit to check against
    double worstExcess = 0;
    Offenders offenders;
    void note(CarId car, uint64_t tick, double seconds, double dt, double speed, double myLimit, engine::PathId path);
    void notChecked(CarId car);      // nothing to check this tick: any spell over ends

private:
    struct Spell {
        double worst = 0;
        std::size_t offender = Offenders::kNotKept;
    };
    std::map<CarId, Spell> over_;
};

// ROUND A BEND TOO FAST: speed squared over the bend's radius where the car's front is (its path's bend profile, issue
// #281; Measurements reads it with LaneGraph::radiusAt) above `comfortable` by more than
// kSideAccelToleranceMps2. One offender per spell, showing the worst moment of it.
struct BendRow {
    double carSeconds = 0;
    double checkedCarSeconds = 0;    // car-seconds on a path with a bend
    double worstSideAccel = 0;
    Offenders offenders;
    void note(CarId car, uint64_t tick, double seconds, double dt, double speed, double radius, double comfortable);
    void notChecked(CarId car);

private:
    struct Spell {
        double worst = 0;
        std::size_t offender = Offenders::kNotKept;
    };
    std::map<CarId, Spell> over_;
};

// WHAT A TICK COSTS, as the caller timed it (tickTimed).
struct CostRow {
    uint64_t ticksTimed = 0;
    double sumPerThousand = 0;
    double worstPerThousand = 0;
    void note(double milliseconds, std::size_t drivenCars);
};

// WAITING WITH NO WAY IT CAN END (issue #284): a car refused room beyond a junction for a reason that cannot change
// (Driving::waitCanNeverEnd), in spells per car, kept as the standing-with-no-reason row keeps its spells. Target: none.
// The router avoids every such way (room.h turnsClosedForRoom, chainsClosedForRoom), so a spell here points at a car
// routed or put down past them.
using EndlessWaitRow = NoReasonRow;

// NO PROGRESS (issue #284): a car on a trip whose odometer advanced less than the rule set's progress_least metres over
// its last progress_window seconds -- a car creeping a few centimetres every few seconds is never "standing", so the
// rows above cannot see it. Looked at once a second. EXCUSED while what holds it names a reason that still applies
// (Driving::waitReasonHolds: a car ahead on its way, a stop it still holds, a car it gives way to ...); the rest are
// offenders, in spells per car. Target: none unexcused.
struct ProgressRow {
    double carSeconds = 0;        // car-seconds without progress over the window, excused or not
    NoReasonRow unexcused;        // ... the spells with no reason that applies
};

// What the no-progress row reads from the rule set (Measurements reads them when the rule set has them; a host that
// wants the row declares them with registerMeasureNeeds, so a rule set without them is refused, named).
constexpr const char* kNumberProgressWindow = "progress_window";   // s
constexpr const char* kNumberProgressLeast = "progress_least";     // m
constexpr const char* kMeasureReaderName = "the measurements table (the no-progress row)";
bool registerMeasureNeeds(FactRegistry& registry, std::vector<std::string>* problems = nullptr);

// ALL THE COUNTS, which buildReport turns into the table.
struct MeasureRows {
    NoReasonRow noReason;
    EndlessWaitRow endless;
    ProgressRow progress;
    bool progressMeasured = false;    // the rule set has the no-progress row's numbers
    LoopRow loops;
    JumpRow jumps;
    TurnRow turns;
    JunctionStopRow junctionStops;
    BoxStopRow boxStops;
    OverlapRow overlaps;
    TouchRow touches;
    OverLimitRow overOwnLimit;
    BendRow bends;
    std::vector<QueueBurst> queues;   // every queue at every line, in line order then time
    std::vector<uint32_t> queueLine;  // ... and the line each came from
    CostRow cost;
    CapCounts caps;
    uint64_t ticks = 0;
    double seconds = 0;
    uint32_t drivenCars = 0;
};

// THE TABLE from the counts. `ruleNames` names rules by index (the rule set's labels); a rule with no name is
// "rule <n>".
MeasurementReport buildReport(const MeasureRows& rows, const std::vector<std::string>& ruleNames);

// ---- The watcher ---------------------------------------------------------------------------------------------------

class Measurements {
public:
    // A COUNTING LINE: a place on the road where the times successive fronts cross are kept, for the queue figures.
    // Its index.
    std::size_t addCountingLine(const PathPoint& at);
    const std::vector<PathPoint>& countingLines() const { return lines_; }

    // Called by Driving::attachMeasurements: remembers where every car is now, so the first tick has something to
    // compare with. Again for another Driving starts over.
    void begin(const Driving& d);
    bool begun() const { return driving_ != nullptr; }
    // Called by Driving after each tick (or by tickTimed): updates every count. Reads the world only.
    void observe(const Driving& d);
    // What a tick cost on the wall clock, as the caller timed it, and how many cars were driven in it.
    void noteTickCost(double milliseconds, std::size_t drivenCars) { rows_.cost.note(milliseconds, drivenCars); }
    // THE SIZE THE HOST DRAWS car `car` at (metres), for the body-against-body row; a car not given one is drawn at its
    // vehicle class's size.
    void setDrawnSize(CarId car, double length, double width);

    const MeasureRows& rows() const { return rows_; }
    // The table so far.
    MeasurementReport report() const;

private:
    // What this module remembers of a car from the last look.
    struct Seen {
        CarId car;
        TripId trip;
        bool driven = false;
        double odometer = 0;
        PathPoint front;
        engine::Vec3 frontAt;
        double bodyX = 0, bodyZ = 0;  // the body's direction on the plan
        double speed = 0;
        bool atRest = true;
        double restLeftAt = -1;       // when it last began to move from rest (negative: never seen)
        double restOdometer = 0;      // its odometer then
    };

    void lookAtCar(const Driving& d, const Car& c, const Seen* before, double now, double dt, Seen& out);
    Seen seeCar(const Driving& d, const Car& c) const;
    void lookForOverlaps(const Driving& d, double now);
    void lookForTouches(const Driving& d, double now);
    void lookForBoxStops(const Driving& d, double now, double dt);
    // The no-progress row, once a second: each driven car's odometer then, kept for progress_window seconds.
    void lookForProgress(const Driving& d, double now, double sampleDt);

    const Driving* driving_ = nullptr;
    std::vector<PathPoint> lines_;
    std::vector<std::vector<LineCrossing>> crossings_;   // per line, in the order they crossed
    std::map<uint32_t, std::vector<std::size_t>> linesOnPath_;   // the counting lines on each path, by path id
    // THE NO-PROGRESS ROW's memory: per car id, its odometer at each once-a-second look of the window (a ring, sized once
    // when the window is known), how many are filled, the next slot, and the trip it was on.
    struct Progress {
        std::vector<double> odometer;
        std::size_t filled = 0, next = 0;
        TripId trip;
    };
    std::vector<Progress> progress_;
    double progressWindow_ = 0, progressLeast_ = 0;
    std::size_t progressSlots_ = 0;
    double lastProgressSample_ = 0;
    std::vector<Seen> seen_;                             // by car id
    std::vector<Seen> seenNext_;
    double lastClock_ = 0;
    bool sampledLoops_ = false;
    double nextLoopSample_ = 0;
    std::vector<std::string> ruleNames_;
    MeasureRows rows_;
    // Working memory for reading a driver's own limit through the registered fact.
    std::vector<NoticedFact> noticed_;
    NoticeScratch scratch_;
    std::vector<FactSlot> wantLimit_;
    std::vector<std::pair<OverlapPair, engine::PathId>> overlapsFound_;
    // Body against body: the drawn sizes (by car id; zero length: the class's), and working memory kept between ticks.
    std::vector<std::pair<double, double>> drawnSize_;
    std::vector<PlanBody> bodies_;
    std::vector<std::size_t> bodyRecord_;   // per body: its record in d.cars()
    std::vector<std::size_t> sweep_;        // bodies in order of their lowest x
    std::vector<TouchSpell> touchesFound_;
};

// ONE TICK, TIMED: ticks `d` and hands what it cost on the wall clock to the measurements attached to it (none: only
// ticks). The tick is timed with the measurements set aside, and they then look at it, so the cost is the tick's own
// and the module never reads the clock while it watches. Returns what the tick cost, in milliseconds (for a host's own
// report; nothing that drives may read it).
double tickTimed(Driving& d, Real dt);

}  // namespace driving

#endif
