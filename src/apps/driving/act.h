#ifndef RAYTRACER_APPS_DRIVING_ACT_H
#define RAYTRACER_APPS_DRIVING_ACT_H

// HOW A CAR SPEEDS UP, FOLLOWS AND STOPS (driving-model-design.md 8; issue #253): the arithmetic of the move step, as
// pure functions of numbers, so each can be tested with no road and no world. driving.cpp gathers the numbers (the
// car ahead, the held answers, the way's end) and calls these.
//
// Two published formulas, used together:
//
//   EASING toward a speed and off whatever is ahead (the Intelligent Driver Model, Treiber, Hennecke & Helbing 2000).
//   Speed up toward the top speed, less as you near it; ease off as the gap to the thing ahead falls below the gap
//   you like, which is a standstill gap, plus so many seconds of travel, plus more when you are closing in fast:
//       accel      = a * [ 1 - (v / v_top)^4 - (gap_wanted / gap)^2 ]
//       gap_wanted = s0 + v*T + v*(v - v_ahead) / (2*sqrt(a*b))
//   a: comfortable speeding-up, b: comfortable braking (both the vehicle class's), T: the seconds of gap this driver
//   keeps (the personal setting gap_kept), s0: the standstill gap (the rule-set number standstill_gap).
//
//   A HARD LIMIT: never faster than the speed from which you could still stop behind the thing ahead if it braked as
//   hard as it can, given your reaction time tau (the Krauss safe speed, Krauss 1998):
//       v_safe = -b_max*tau + sqrt( (b_max*tau)^2 + 2*b_max*(gap + v_ahead^2 / (2*b_ahead)) )
//   b_max: my hardest braking; b_ahead: the hardest braking of the thing ahead. (Krauss writes it for two cars that
//   brake alike, where v_ahead^2 / (2*b_ahead) * 2*b_max is just v_ahead^2; keeping the two apart stays safe behind a
//   car that can stop shorter than I can.)
//
// THINGS AHEAD (obstacles) come in three kinds, each fed to the same two formulas:
//   - the CAR AHEAD: the gap is from my front to its rear. The hard limit keeps the standstill gap clear: it reads the
//     gap less s0.
//   - a PLACE TO STOP BY (design 8: "a stopped car at that place"): a stopped car whose rear stands one standstill
//     gap PAST the place, so that easing brings my front to rest at the place itself; the hard limit reads the
//     distance to the place, so my front never passes it.
//   - the END OF THE WAY (the end of the last path of the way ahead the driver holds): a wall my front never passes.
//     Where the way truly ends (its last path leads nowhere on the route) it is a place to come to rest at, like a
//     place to stop by, so a car whose rules forgot the lane's end still eases to a halt there and stays. Where the
//     way ends only because the driver looked no further, it is the hard limit only (`hardOnly`): slowing for it
//     would be wrong, and the look-ahead keeps it out of reach.
//
// Units: metres, seconds, m/s, m/s2.

#include "apps/driving/notice.h"          // FactRegistry, kStandingBelow
#include "apps/driving/rule_set.h"
#include "apps/driving/traffic_state.h"   // CarId, Limit, Binding
#include "rt_math.h"

#include <cstddef>

namespace driving {

using engine::Real;

// ---- What the move step reads from the rule set ---------------------------------------------------------------------

// The personal setting: how many seconds of gap this driver keeps behind the car ahead (T), a time.
constexpr const char* kSettingGapKept = "gap_kept";
// The rule-set number: the gap kept to the car ahead when both stand still (s0), a length.
constexpr const char* kNumberStandstillGap = "standstill_gap";
// The name the move step reads the rule set under, in the problems buildRoadRules reports.
constexpr const char* kMoveReaderName = "the move step (speeding up, following and stopping)";

// Declares the two above in `registry` (a ReaderSpec), so buildRoadRules reports a rule set that lacks either. False
// (with the reason) when it was declared already.
bool registerMoveNeeds(FactRegistry& registry, std::string* why = nullptr);

// ---- The named constants of the formulas -----------------------------------------------------------------------------

// The power on (v / v_top) in the easing formula: how sharply a car stops speeding up as it nears its top speed. The
// Intelligent Driver Model's own value.
constexpr Real kFreeRoadExponent = 4;
// A CAR AT REST moves off only when its acceleration is above this (m/s2). Without it a car stopped a hair beyond the
// standstill gap would creep forward a little on every tick.
constexpr Real kMoveOffAbove = 0.05;
// The smallest gap the easing formula divides by (m). A gap at or below zero (a car placed too close) is not divided
// by at all: it means "brake as hard as allowed".
constexpr Real kGapFloor = 0.1;
// A car that is slowing down comes to rest once it is slower than this (m/s): the crawl `standing` reads as stopped.
// Easing alone only ever approaches zero, so without this a stopped car would keep inching forward.
constexpr Real kComesToRestBelow = kStandingBelow;

// HOW FAR A DRIVER LOOKS AHEAD (m): far enough to stop from its speed braking comfortably, plus a margin, and never
// less than a floor. The margin and the floor move to the rule set's `detail` table in a later task.
constexpr Real kLookAheadMargin = 30;
constexpr Real kLookAheadFloor = 50;
Real lookAheadDistance(Real speed, Real comfortableBrake);

// THE REACTION TIME in the hard limit: the driver's decide interval, or the step if that is longer (s).
Real reactionTime(Real decideInterval, Real step);

// ---- The two formulas -------------------------------------------------------------------------------------------------

// WHAT THE FORMULAS NEED TO KNOW about a driver and its car.
struct Follower {
    Real speedUp = 0;        // a: comfortable speeding-up, m/s2 (the class's)
    Real brake = 0;          // b: comfortable braking, m/s2 (the class's)
    Real hardestBrake = 0;   // b_max: hardest braking, m/s2 (the class's)
    Real gapKept = 0;        // T: seconds of gap this driver keeps (gap_kept)
    Real standstillGap = 0;  // s0: metres between the two when both stand (standstill_gap)
};

// EASING ON AN EMPTY ROAD: a * (1 - (v / v_top)^4). A top speed of zero or below (a car held still) is never divided
// by: a moving car brakes comfortably, a car at rest stays at rest (zero).
Real freeRoadAcceleration(const Follower& f, Real speed, Real topSpeed);
// The gap this driver would like to the thing ahead at these speeds: s0 + v*T + v*(v - v_ahead) / (2*sqrt(a*b)), and
// never below s0 (the closing part is below zero when the thing ahead pulls away; it may cancel the seconds of gap but
// never eat into the standstill gap, as in Treiber's own later writing of the model).
Real gapWanted(const Follower& f, Real speed, Real speedAhead);
// EASING BEHIND SOMETHING `gap` metres ahead going at `speedAhead`: the whole formula. A gap at or below zero gives
// minus the hardest braking; a gap above zero but below kGapFloor is read as kGapFloor.
Real easingAcceleration(const Follower& f, Real speed, Real topSpeed, Real gap, Real speedAhead);
// THE HARD LIMIT: the fastest I may go now and still stop behind something `gap` metres ahead going at `speedAhead`
// that can brake at `aheadHardestBrake` (zero or below: it can stop dead), with reaction time `reaction`. Never below
// zero; zero when even standing still now would not be enough.
Real safeSpeed(Real hardestBrake, Real reaction, Real gap, Real speedAhead, Real aheadHardestBrake);

// ---- One car's step ----------------------------------------------------------------------------------------------------

// ONE THING AHEAD of a driver this tick.
struct Obstacle {
    Limit kind = Limit::CarAhead;   // CarAhead, Stop or WayEnd
    Real distance = 0;              // my front to: its rear (CarAhead), the place (Stop), the end (WayEnd)
    Real speed = 0;                 // CarAhead: its speed; otherwise 0
    Real hardestBrake = 0;          // CarAhead: its hardest braking; otherwise unused (the place cannot move)
    CarId car;                      // CarAhead: which car
    RuleIndex rule = kNoRule;       // Stop: which rule gave it
    bool hardOnly = false;          // WayEnd: only where the driver stopped looking (the hard limit alone applies)
};
// The most things one driver weighs at once: the car ahead, the place to stop by, the end of the way.
constexpr std::size_t kMaxObstacles = 3;

// WHAT ONE STEP COMES TO.
struct StepOutcome {
    Real speed = 0;          // after the step, m/s; never below zero
    Real acceleration = 0;   // (speed after - speed before) / step
    Binding binding;         // what held the car back
};

// ONE CAR'S NEW SPEED over a step of `step` seconds, from its speed now, the top speed it holds (and the rule that gave
// it; kNoRule for the class's own), and what is ahead:
//   1. Easing: the lowest acceleration over the empty road and each car or place ahead, never braking harder than the
//      comfortable braking. What held it back is whichever term took the most off: the top speed, or one thing ahead;
//      Nothing when no term took anything off.
//   2. A car at rest stays at rest unless that acceleration is above kMoveOffAbove.
//   3. new speed = speed + acceleration * step, capped by the hard limit against each thing ahead (the lowest; when it
//      caps, that thing held the car back), never below what the hardest braking reaches in one step, never below
//      zero; and a car slowing below kComesToRestBelow comes to rest.
// The caller moves the car by the NEW speed times the step.
StepOutcome stepSpeed(const Follower& f, Real speed, Real topSpeed, RuleIndex topSpeedRule, const Obstacle* obstacles,
                      std::size_t obstacleCount, Real step, Real reaction);

}  // namespace driving

#endif
