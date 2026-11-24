#ifndef RAYTRACER_APPS_DRIVING_LANE_CHANGE_H
#define RAYTRACER_APPS_DRIVING_LANE_CHANGE_H

// LANE CHANGES (two-drivers-meeting-design.md section 6, sitting 5; driving-model-design.md section 10; #229, #285).
// A driver whose route needs another lane of the road it is on wants to move over; it looks at the lane beside for a
// gap; given one, it moves across, listed in both lanes while it does (traffic_state.h Car::besidePath), easing sideways
// over `lane_change_seconds`. Nothing central hands out gaps: each driver judges the one it sees.
//   - WANTING TO: `wrong_lane`, from the route (route.h lanesToCross: the nearest lane with fewer changes still needed).
//   - A GAP: `gap_beside`, MOBIL's safety criterion (Kesting, Treiber & Helbing 2007, eq. 2 and table 1): after the
//     change, the car that would be behind me in that lane need brake no harder than `lane_change_safe_brake` to keep
//     behind me, by the following model itself (act.h easingAcceleration, the IDM); and so need I, behind the car
//     that would be ahead of me there.
//   - NOT GETTING ACROSS: past `last_chance` (route.h lastChance) the driver does not stop in a through lane to wait:
//     it carries on in its lane and is routed again from there (the missed turn), as section 10 says.
// What a driver does with the facts is the rule set's ("change lanes for my route"); the move is the tick's.
// PULLING OUT OF A KERB SPACE AND INTO ONE (design principle 5: pulling out follows a real path too) is the same move
// across, between the space's offset beside the kerb lane and the lane's line (LaneMove::kerb, LaneMove::to): out when
// a car placed in its space sets off (Driving::placeCarInSpace), in at the end of a trip to a space (Trip::pullInOffset).

#include "apps/driving/decide.h"
#include "apps/driving/notice.h"
#include "apps/driving/route.h"
#include "apps/driving/things.h"          // nearestOn
#include "apps/driving/traffic_state.h"
#include "engine/procgen/city/roads/lane_graph.h"
#include "rt_math.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace driving {

// ---- The facts, the answer, the numbers --------------------------------------------------------------------------------
constexpr const char* kFactWrongLane = "wrong_lane";     // yes/no: my route needs another lane of this road, beside mine
constexpr const char* kFactLastChance = "last_chance";   // m: how far I may still stay in my lane (absent: no need to leave)
constexpr const char* kFactGapBeside = "gap_beside";     // yes/no: a safe gap in the lane I need, beside me now
// yes/no, and m: no way on from my lane, so I wait for a gap -- stopped where a move across still fits
constexpr const char* kFactMustWaitToChange = "must_wait_to_change";
constexpr const char* kFactChangeWaitLine = "change_wait_line";
// yes/no: a car in the lane beside, its front ahead of mine, is indicating into my lane (letting in, sitting 5)
constexpr const char* kFactCarWantingIn = "car_wanting_in";
constexpr const char* kIntentionLetIn = "let_in";
constexpr const char* kWordLetIn = "yes";
// yes/no: my route needs the lane beside and a car there closes the gap I would take (dropping back, sitting 5)
constexpr const char* kFactCarBlockingGap = "car_blocking_my_gap";
constexpr const char* kIntentionGap = "gap";
constexpr const char* kWordDropBack = "drop_back";
constexpr const char* kNumberLaneChangeBrake = "lane_change_safe_brake";   // m/s2: MOBIL's b_safe
constexpr const char* kNumberLaneChangeSeconds = "lane_change_seconds";    // s: how long the move across takes
constexpr const char* kIntentionLane = "lane";
constexpr const char* kWordChange = "change";
constexpr const char* kLaneChangeReaderName = "lane change step";
// THE MOVE ACROSS: an S-curve (smoothstep) of the sideways offset over the distance the car covers, so it eases out and
// in and never slides sideways at rest (a car that stops part-way stays part-way). Its length: the distance covered in
// lane_change_seconds at the speed it starts at, and never shorter than its turning circle allows -- the S-curve's
// sharpest bend is 6 * across / length^2, so a length of sqrt(6 * across * radius) is the tightest it can take (with a
// margin of kMoveMargin).
struct LaneMove {
    Real from = 0;                // its offset from the new lane's line when it began (+ toward the kerb)
    // ...and the offset it ends at: 0 (the lane's line) for a lane change and for pulling out of a kerb space; the
    // space's offset for pulling into one (sitting 5: parking on the same drivable curve).
    Real to = 0;
    bool kerb = false;            // pulling out of or into a kerb space beside its lane, not a lane change
    double startOdometer = 0;     // where it began, by its odometer
    Real length = 0;              // the distance it takes (m)
    // Its front is on the new line: the move is over for the driving (no longer listed in both lanes). It is kept only
    // while its REAR still follows the same curve, a body length behind -- drawn, never decided by (Car::offsetRearLag).
    bool frontDone = false;
    // The lane it moves out of: until its rear is across, a car behind in that lane still follows it (a wait for it holds).
    engine::PathId leftLane;
    // A MOVE BEGUN WHILE THE LAST ONE'S REAR WAS STILL CROSSING (one lane change straight after another): that move's
    // curve, which the rear goes on following, added to this one's (drawn only). carryLength 0: none.
    Real carryFrom = 0;
    double carryStart = 0;
    Real carryLength = 0;
};
constexpr Real kMoveMargin = 1.25;
// How far back a driver looks for the car that would be behind it in the lane beside, along the paths leading into that
// lane where it begins close behind (m): [~] room for a car at 50 km/h to stop from, braking at lane_change_safe_brake.
constexpr Real kLookBehind = 80.0;
// The least speed a car begins a move across at (m/s): it cannot step sideways from rest, and a car only just setting
// off waits until it is rolling. [~] a judgement: walking pace.
constexpr Real kLaneChangeLeastSpeed = 1.0;
// How far apart in direction (the sine of the angle) two lanes may run where a car moves from one to the other: about a
// third of a degree, so the switch puts it beside where it was to within a couple of centimetres on a 3.5 m lane.
constexpr Real kLanesSideBySide = 0.006;
// The widest move across a route plans for (m): a lane's width, as the road builder lays the widest (traffic lanes of
// 3.65 m, ramps a little wider) [~]; a route plans no move in a lane too short for one this wide.
constexpr Real kWidestLaneAcross = 4.0;
// Below this sideways offset (m) a car is on its lane's line: its speed is read along the lane itself.
constexpr Real kOffsetOnLine = 0.01;
// A move across is over once its front is within this (m) of its length: a move laid to end where its lane ends is
// driven up to a stop there, and a stop is reached only to within rounding -- left a hair short, the car stood listed
// in both lanes for good, never looking for its next move (the 20-minute lab).
constexpr Real kMoveDoneWithin = 0.01;
// Dropping back for a gap: while the car closing it is level with me or behind, I keep this much (m/s) below its speed,
// so it draws ahead and the gap behind it comes level with me [~].
constexpr Real kDropBackBelow = 1.0;
// Waiting to merge: a car held to give way this near (m) to where its lane joins another shows it wants in [~].
constexpr Real kMergeWaitWithin = 5.0;
inline Real laneMoveShortest(Real across, Real tightestRadius) {
    return kMoveMargin * std::sqrt(6 * std::fabs(across) * std::max(tightestRadius, Real(1)));
}
// ...and done before the lane it moves into ends (`room` metres on), so it never carries a move into a junction: shorter
// to fit, never below the shortest its turning circle allows (no room for that: it does not begin, `ok` false).
inline Real laneMoveLength(Real across, Real speed, Real seconds, Real tightestRadius, Real room, bool* ok = nullptr) {
    const Real shortest = laneMoveShortest(across, tightestRadius);
    if (ok) *ok = room >= shortest;
    return std::max(shortest, std::min(speed * seconds, room));
}
// The offset `travelled` metres into a move (`to` once it is done).
inline Real laneMoveOffset(const LaneMove& m, Real travelled) {
    const Real u = m.length > 0 ? std::clamp(travelled / m.length, Real(0), Real(1)) : Real(1);
    return m.to + (m.from - m.to) * (1 - u * u * (3 - 2 * u));
}
// Where the rear is in a move (its offset from the new lane's line), `body` metres behind a front `travelled` metres in:
// this move's curve a body length back, plus what is left of a move it was begun on top of.
inline Real laneMoveRearOffset(const LaneMove& m, Real travelled, Real body) {
    Real rear = laneMoveOffset(m, travelled - body);
    if (m.carryLength > 0) {
        LaneMove carried;
        carried.from = m.carryFrom;
        carried.length = m.carryLength;
        rear += laneMoveOffset(carried, static_cast<Real>(m.startOdometer - m.carryStart) + travelled - body);
    }
    return rear;
}

// THE LANE BESIDE ME AND THE GAP IN IT, as one driver sees it now.
struct LaneGap {
    engine::PathId target;     // the lane I would move into (none: no lane beside on that side I may cross into)
    Real targetAlong = 0;      // where my front would be along it (the nearest point of its line to my front)
    Real across = 0;           // how far its centre line is from where I am, + toward the kerb (m)
    Real newOffset = 0;        // where I am from its centre line, as an offset in it (+ toward the kerb): -across
    bool safe = false;         // MOBIL's safety criterion holds both ways (and nobody is beside me there)
    CarId ahead, behind;       // the cars that would be ahead of and behind me in it (not valid: none within the lane)
    Real gapAhead = 0, gapBehind = 0;
    // NO GAP: the car that closes it -- level with me there, or too near ahead, or too near behind (blockerBehind) --
    // for a driver that comes level with a gap instead of driving on to its end (design section 6).
    CarId blocker;
    bool blockerBehind = false;
};

// WHAT A DRIVER KNOWS ABOUT CHANGING LANES this decision, for its facts.
struct LaneChangeView {
    LanesToCross need;         // how many lanes over the lane my route wants is, and which way
    LastChance last;           // how far I may still stay in this one
    LaneGap gap;               // the lane beside, toward the one I want, and its gap
    // Where my move across must begin at the latest, from my front (m): last.metres less the shortest move it can make.
    Real changeBy = 0;
    // NO WAY ON FROM MY LANE (routed again past my last chance, and nothing led on from it): I wait for a gap, stopped
    // no further on than changeBy (never below 0), and move across from there, from rest if I must.
    bool mustWait = false;
    // LETTING IN (design section 6: the other driver's own choice): the car indicating into my lane whose front is ahead
    // of mine, nearest first -- or the one I already let in, until it is across (zipper: one at a time) -- and the gap
    // from my front to where its rear would be in my lane. Not valid: none.
    CarId wantingIn;
    Real wantingInGap = 0;
};

// THE LANE BESIDE `me` TOWARD `towardKerb` AND ITS GAP: where I would be in it, who would be ahead and behind, and
// whether a change now is safe. `followerBrake(car, gap, speedAhead)` answers how hard (m/s2, below zero braking) that car
// would accelerate following something `gap` metres ahead going at `speedAhead`, by its own driver's following (the
// IDM with its class and personal gap); the same for me with `myBrake`. Cars listed Beside count as in the lane.
// `kerbSign`: kerbwardPerpSign(drivesOn), as the pose reads an offset. `before[p]`: the paths leading into path p (the
// car that would be behind me may still be on one of them: up to kLookBehind metres back).
template <typename FollowFn>
LaneGap findLaneGap(const engine::LaneGraph& roads, const std::vector<Car>& cars, const PathLists& lists, const Car& me,
                    Real myLength, bool towardKerb, Real safeBrake, Real kerbSign, const std::vector<std::vector<engine::PathId>>& before,
                    FollowFn&& follow);

// The facts, the answer and the reader that switches the step on (as registerThingFacts / registerThingAnswers).
bool registerLaneChangeFacts(FactRegistry& registry, std::vector<std::string>* problems);
bool registerLaneChangeAnswers(AnswerVocabulary& vocabulary, std::string* why);
bool hasLaneChangeStep(const FactRegistry& registry);

// ---- findLaneGap -------------------------------------------------------------------------------------------------------
template <typename FollowFn>
LaneGap findLaneGap(const engine::LaneGraph& roads, const std::vector<Car>& cars, const PathLists& lists, const Car& me,
                    Real myLength, bool towardKerb, Real safeBrake, Real kerbSign, const std::vector<std::vector<engine::PathId>>& before,
                    FollowFn&& follow) {
    LaneGap g;
    if (!me.path.valid() || me.path.index >= roads.paths.size()) return g;
    g.target = laneBeside(roads, me.path, towardKerb);
    if (!g.target.valid() || g.target.index >= roads.paths.size()) return g;
    // WHERE MY FRONT IS, IN THE LANE BESIDE'S TERMS, by the same reading of a lane that draws me (traffic_state.h
    // carPose: the point along it, moved sideways square to its direction there): the nearest point of its line, then
    // moved along until my front lies square to it, so moving in leaves me exactly where I am
    const Real theirs = roads.paths[g.target.index].length;
    const engine::Vec2 myFront = roads.pointAt(me.path, me.along) +
                                 engine::perp(roads.directionAt(me.path, me.along)) * (kerbSign * me.offset);
    g.targetAlong = std::clamp(nearestOn(roads, g.target, myFront).along, Real(0), theirs);
    for (int k = 0; k < 3; ++k) {
        const engine::Vec2 there = roads.pointAt(g.target, g.targetAlong), dir = roads.directionAt(g.target, g.targetAlong);
        const engine::Vec2 d = myFront - there;
        g.targetAlong = std::clamp(g.targetAlong + d.x * dir.x + d.y * dir.y, Real(0), theirs);
    }
    {
        const engine::Vec2 there = roads.pointAt(g.target, g.targetAlong);
        const engine::Vec2 side = engine::perp(roads.directionAt(g.target, g.targetAlong));
        g.newOffset = ((myFront - there).x * side.x + (myFront - there).y * side.y) * kerbSign;
    }
    g.across = me.offset - g.newOffset;
    if (g.targetAlong < myLength) return g;   // (my body must lie in that lane whole: none of it on the paths before it)
    // (the two lanes must run side by side here: where they splay apart, moving over would turn me on the spot)
    {
        const Real step = std::min(Real(1), roads.paths[me.path.index].length - me.along);
        const Real stepThere = std::min(Real(1), roads.paths[g.target.index].length - g.targetAlong);
        if (!(step > 0) || !(stepThere > 0)) return g;
        engine::Vec2 a = roads.pointAt(me.path, me.along + step) - roads.pointAt(me.path, me.along);
        engine::Vec2 b = roads.pointAt(g.target, g.targetAlong + stepThere) - roads.pointAt(g.target, g.targetAlong);
        a = a * (Real(1) / a.length());
        b = b * (Real(1) / b.length());
        if (std::fabs(a.x * b.y - a.y * b.x) > kLanesSideBySide) return g;
    }
    // who would be ahead of me and behind me there; anyone level with me means no gap
    const Real myRear = g.targetAlong - myLength;
    Real speedAhead = 0;
    for (const PathEntry& e : lists.on(g.target)) {
        if (e.car == me.id) return g;   // (already listed there: changing)
        if (e.rear >= g.targetAlong) {
            if (!g.ahead.valid() || e.rear - g.targetAlong < g.gapAhead) {
                g.ahead = e.car;
                g.gapAhead = e.rear - g.targetAlong;
            }
        } else if (e.front <= myRear) {
            if (!g.behind.valid() || myRear - e.front < g.gapBehind) {
                g.behind = e.car;
                g.gapBehind = myRear - e.front;
            }
        } else {
            g.blocker = e.car;   // beside me
            return g;
        }
    }
    // (nobody behind me in the lane itself: the car that would be is on a path leading into it -- a connector, the lane
    // before -- looked for back along them as far as kLookBehind)
    if (!g.behind.valid() && myRear < kLookBehind) {
        struct Back { engine::PathId path; Real behind; };   // a path, and how far its end lies behind my rear
        std::vector<Back> todo;
        if (g.target.index < before.size())
            for (const engine::PathId p : before[g.target.index]) todo.push_back({p, myRear});
        for (std::size_t k = 0; k < todo.size() && k < 16; ++k) {
            const Back b = todo[k];
            if (!b.path.valid() || b.path.index >= roads.paths.size() || b.behind > kLookBehind) continue;
            const Real len = roads.paths[b.path.index].length;
            for (const PathEntry& e : lists.on(b.path)) {
                if (e.car == me.id) continue;
                const Real gap = b.behind + (len - e.front);
                if (!g.behind.valid() || gap < g.gapBehind) {
                    g.behind = e.car;
                    g.gapBehind = gap;
                }
            }
            if (!g.behind.valid() && b.path.index < before.size())
                for (const engine::PathId p : before[b.path.index]) todo.push_back({p, b.behind + len});
        }
    }
    const auto carOf = [&](CarId id) -> const Car* {
        if (!id.valid() || id.index >= lists.recordOfCar.size()) return nullptr;
        const uint32_t r = lists.recordOfCar[id.index];
        return r < cars.size() ? &cars[r] : nullptr;
    };
    if (const Car* a = carOf(g.ahead)) speedAhead = a->speed;
    // MOBIL's safety criterion, eq. 2: the new follower's acceleration behind me is no harsher than -b_safe; and mine
    // behind the new leader (the following model's own answer, not a gap table)
    if (g.ahead.valid() && follow(me, g.gapAhead, speedAhead) < -safeBrake) {
        g.blocker = g.ahead;
        return g;
    }
    if (const Car* b = carOf(g.behind))
        if (follow(*b, g.gapBehind, me.speed) < -safeBrake) {
            g.blocker = g.behind;
            g.blockerBehind = true;
            return g;
        }
    g.safe = true;
    return g;
}

}  // namespace driving

#endif
