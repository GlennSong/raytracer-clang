#ifndef RAYTRACER_APPS_DRIVING_THINGS_H
#define RAYTRACER_APPS_DRIVING_THINGS_H

// WHAT IS IN MY WAY (two-drivers-meeting-design.md sections 5 and 6; ADR-0451). One question every driver asks at each
// decision: what is ahead of me, along the way I mean to go, whatever it is, and how is it moving? The WORLD answers it;
// the driver never asks "is it a person".
//
// Words: a THING is anything that stands or moves on the road that is not a car the driving drives (those it sees by
// itself: traffic_state.h carAhead). A person crossing, the player on foot, a crate, a car nobody here drives, a wreck.
// What a driver knows of the thing ahead of it is a THING AHEAD: how far along its way, how fast it moves along it, how
// far to the side of its line it is and how wide.
//
// WHO ANSWERS:
//   - a MIDDLE car (a lane car): the driving, from the things the host says stand on the roads this tick (RoadThing, one
//     list the world writes to: Driving::setThingsOnRoad). Each is looked for along the car's own way ahead
//     (findThingAhead): a thing whose middle is closer to the car's line than the two half widths and a clearance is in
//     its way.
//   - a NEAR car (a physics car the host carries): the host, from physics: a box the car's width swept along the line it
//     means to drive (Driving::seeAhead, before the tick). Physics also says whether there is room to go round it.
//
// WHAT A DRIVER DOES WITH IT is the rule set's (rules/lab.lua): the facts below, the action `horn` and the intentions
// `pass` (swerve) and `way` (another), registered here (registerThingAnswers). The move step acts on them
// (driving.cpp). Nothing here names a kind of thing.

#include "apps/driving/notice.h"
#include "apps/driving/decide.h"
#include "engine/procgen/city/roads/lane_graph.h"
#include "rt_math.h"

#include <cstdint>
#include <string>
#include <vector>

namespace driving {

using engine::Real;

// ONE THING STANDING OR MOVING ON THE ROAD, as the host writes it each tick: its middle on the plan, how far it reaches
// across (a person's shoulders, half a crate) and how it moves. `id`: the host's number for it (the same thing keeps its
// number from tick to tick, so a driver can tell it has stood still).
struct RoadThing {
    uint32_t id = 0;
    engine::Vec2 at;
    Real halfAcross = 0;
    engine::Vec2 velocity;
};

// A thing slower than this (m/s) stands still: what a driver watching it judges (thing_stood counts from then).
constexpr Real kThingStillBelow = 0.2;
// How fast a lane car eases to the side of its line going round something (m/s): about a lane's width in three
// seconds, an unhurried move across. [~] a judgement, as the design's "move across over a few seconds" (section 6).
constexpr Real kSwerveSideSpeed = 1.0;

// WHAT ONE DRIVER SEES IN ITS WAY: the nearest thing along its way ahead, within its look-ahead.
struct ThingAhead {
    bool found = false;
    uint32_t id = 0;
    Real distance = 0;     // my front bumper to the near side of it, along my way (m; below zero: already against it)
    Real speed = 0;        // how fast it moves along my way (m/s; below zero: toward me)
    Real moves = 0;        // how fast it moves at all (m/s): a person crossing moves, though not along my way
    Real side = 0;         // its middle's distance from my line (m; + toward the kerb side)
    Real halfAcross = 0;   // how far it reaches either side of its middle (m)
    engine::PathId path;   // the path it stands on (another way goes round that)
    // ROOM TO GO ROUND IT: worked out with the thing (a lane car: by the lane's width; a near car: by physics), and the
    // sideways offset from my line that clears it (+ toward the kerb side).
    bool roomToPass = false;
    Real passOffset = 0;
};

// THE NEAREST POINT OF A PATH'S CENTRE LINE to `p`: how far along it, how far `p` is from it, and how far to its kerb
// side (+, the driver's left: the world drives on the left) or offside (-). `away` < 0: the path has no line.
struct OnLine {
    Real along = 0, away = 0, side = 0;
};
OnLine nearestOn(const engine::LaneGraph& roads, engine::PathId path, const engine::Vec2& p);

// THE THING AHEAD OF A LANE CAR, from the things on the road: along my path from `along` and then `pathsAfter`, no
// further than `reach` metres from my front, the nearest whose middle lies within `myHalfWidth` + its half across +
// `clearance` of my line (moved `myOffset` to the side). `stillSpeed`: below this a thing stands. Room to pass: within
// my lane (the path's width), a sideways offset that leaves `clearance` either side of it and of the lane's edges.
// A thing NOT YET in my way is in it all the same when it is COMING INTO IT: within kComingWiden more to the side, moving
// toward my line, and reaching my side of it before I reach it (at `mySpeed`), or within kComingMargin after.
constexpr Real kComingWiden = 5.0;      // m: what a person walking covers in the few seconds a car takes to stop
constexpr Real kComingMargin = 1.0;     // s
constexpr Real kComingSlowest = 0.5;    // m/s (a car standing reaches nothing)
ThingAhead findThingAhead(const engine::LaneGraph& roads, engine::PathId path, Real along,
                          const std::vector<engine::PathId>& pathsAfter, Real reach, Real myHalfWidth, Real myOffset,
                          Real clearance, const std::vector<RoadThing>& things, Real mySpeed = 0);

// ---- The facts -------------------------------------------------------------------------------------------------------

constexpr const char* kFactThingAhead = "thing_ahead";     // yes/no: something in my way within my look-ahead
constexpr const char* kFactThingGap = "thing_gap";         // m: my front to it (about its path)
constexpr const char* kFactThingStop = "thing_stop";       // place: where I stop short of it (thing_stand_off before it;
                                                           // at the line of a junction beyond which it leaves me no room)
constexpr const char* kFactThingSpeed = "thing_speed";     // m/s: how fast it moves along my way
constexpr const char* kFactThingStood = "thing_stood";     // s: how long it has stood still in my way (0 moving)
constexpr const char* kFactRoomToPass = "room_to_pass";    // yes/no: there is room to go round it
constexpr const char* kFactWaited = "waited_for_it";       // s: how long I have stood behind it (0 moving)
// The rule-set numbers the facts read.
constexpr const char* kNumberThingStandOff = "thing_stand_off";   // m: how far short of a thing a driver stops
constexpr const char* kNumberPassClearance = "pass_clearance";    // m: the gap left beside a thing when going round it
// The answers (registerThingAnswers): the action and the intention kinds, with their words.
constexpr const char* kActionHorn = "horn";
constexpr const char* kIntentionPass = "pass";
constexpr const char* kWordSwerve = "swerve";
constexpr const char* kIntentionWay = "way";
constexpr const char* kWordAnother = "another";

// What the driving adds to a driver's view for these facts (DriverView::thing): the thing ahead, and how long it has
// stood in my way.
struct ThingView {
    ThingAhead thing;
    Real stood = 0;
    Real waited = 0;
    Real standOff = 0;
    // The nearest junction line ahead and the end of that junction's box beyond it (m from my front; < 0: none): a thing
    // just beyond a junction that leaves no room for me past the box is stopped for at the line.
    Real lineAt = -1, boxEndsAt = -1;
    Real myLength = 0;
};

// Adds the facts above (and the reader of the numbers the driving reads for them). False with the reasons when one is
// refused (only when a name was registered before).
bool registerThingFacts(FactRegistry& registry, std::vector<std::string>* problems = nullptr);
// Adds the action `horn`, the intention `pass` (swerve) and the intention `way` (another).
bool registerThingAnswers(AnswerVocabulary& vocabulary, std::string* why = nullptr);
// Does the registry hold them (the driving looks for things only then)?
bool hasThingStep(const FactRegistry& registry);

}  // namespace driving

#endif
