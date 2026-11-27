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
#include "engine/procgen/city/roads/road_model.h"   // the crosswalks (ADR-0452)
#include "rt_math.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace driving {

using engine::Real;

// ONE THING STANDING OR MOVING ON THE ROAD, as the host writes it each tick: its middle on the plan, how far it reaches
// across (a person's shoulders, half a crate) and how it moves. `id`: the host's number for it (the same thing keeps its
// number from tick to tick, so a driver can tell it has stood still).
// A car nobody here drives, standing on a lane, is a thing too (the driving adds it: Driving::plan), numbered with this bit
// set over its car id so it never shares a host thing's number.
constexpr uint32_t kCarThingBit = 0x80000000u;
struct RoadThing {
    uint32_t id = 0;
    engine::Vec2 at;
    Real halfAcross = 0;
    engine::Vec2 velocity;
    // WHERE IT STANDS AGAINST A CROSSWALK (ADR-0452), as the world sees it: the road model's crosswalk it is on, or
    // stands at the kerb end of plainly about to cross (kNoIndex: neither), and which. A person waiting at a kerb for a
    // walk signal to change is not about to cross: the world says nothing of them.
    uint32_t crosswalk = engine::kNoIndex;
    bool onCrosswalk = false;
};

// ---- PEOPLE ON CROSSWALKS (ADR-0452) ----------------------------------------------------------------------------------
// Drivers give way to a person on a crosswalk their way crosses, and to one at its kerb plainly about to cross where no
// lights run it (the UK's zebra rule; Japan's Road Traffic Act article 38). The crosswalks and the paths over them are
// the road model's (road_model.h ModelCrosswalk, RoadModel::crossersOf); the people, the world's (RoadThing).
// THE CROSSWALK AHEAD WITH SOMEONE ON IT OR ABOUT TO STEP ON IT: the nearest along my way whose stretch on my path I
// have not yet reached, within `reach`. `stopAt`: my front to where that stretch begins (m).
struct CrosswalkAhead {
    bool found = false;
    uint32_t crosswalk = engine::kNoIndex;   // the road model's
    Real stopAt = 0;
    engine::PathId path;                     // the path the stretch is on
    bool someoneOn = false;                  // someone is on it (else: waiting at its kerb)
};
// WHO IS ON OR AT EACH CROSSWALK this tick, from the things on the road: by the road model's crosswalk id.
struct CrosswalkPeople {
    std::vector<uint8_t> on, waiting;
    void fill(const std::vector<RoadThing>& things, std::size_t crosswalks);
};
// Along my path from `along` and then `pathsAfter`, no further than `reach`: the nearest crossing point of a crosswalk
// (the lane level's records, each naming the model's crosswalk) whose stretch on my path begins ahead of my front, with
// someone on that crosswalk, or someone waiting at its kerb where its control is not lights and I can still stop before
// it braking at `comfortDecel` from `mySpeed`.
CrosswalkAhead findCrosswalkAhead(const engine::RoadModel& model, engine::PathId path, Real along,
                                  const std::vector<engine::PathId>& pathsAfter, Real reach, Real mySpeed, Real comfortDecel,
                                  const CrosswalkPeople& people);

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
// THE THINGS BY WHERE THEY ARE (ADR-0534): each thing's index in its list, by the square cell its middle lies in, made
// once a tick from the list the drivers look along. `near` names, ascending, the things in the cells a circle's box
// touches: every thing within the circle, and some beyond it. False (and `out` left as it is): the box covers so many
// cells that looking at every thing is cheaper. With it, each lane path's box (made once for the road): a thing farther
// from a path's box than it could be from the path and still matter is not measured against the path.
class ThingGrid {
public:
    static constexpr Real kCell = 32.0;   // m
    void build(const std::vector<RoadThing>& things, const engine::LaneGraph& roads);
    bool near(engine::Vec2 centre, Real radius, std::vector<uint32_t>& out) const;
    Real widest() const { return widest_; }   // the most any thing reaches either side of its middle (m)
    // Is `p` farther than `reach` from every point of path `path`'s centre line (measured to the line's box)? False when
    // it may be within reach, or the path has no box.
    bool beyond(engine::PathId path, engine::Vec2 p, Real reach) const;

private:
    const engine::LaneGraph* roads_ = nullptr;
    std::size_t pathCount_ = 0;
    struct PathBox {
        engine::Vec2 lo = engine::Vec2(Real(1e30), Real(1e30)), hi = engine::Vec2(Real(-1e30), Real(-1e30));
    };
    std::vector<PathBox> pathBoxes_;
    static uint64_t key(int64_t x, int64_t z) { return (static_cast<uint64_t>(x) << 32) ^ static_cast<uint64_t>(z & 0xffffffff); }
    std::unordered_map<uint64_t, std::vector<uint32_t>> cells_;
    std::size_t count_ = 0;
    Real widest_ = 0;
};

constexpr Real kComingWiden = 5.0;      // m: what a person walking covers in the few seconds a car takes to stop
constexpr Real kComingMargin = 1.0;     // s
constexpr Real kComingSlowest = 0.5;    // m/s (a car standing reaches nothing)
ThingAhead findThingAhead(const engine::LaneGraph& roads, engine::PathId path, Real along,
                          const std::vector<engine::PathId>& pathsAfter, Real reach, Real myHalfWidth, Real myOffset,
                          Real clearance, const std::vector<RoadThing>& things, Real mySpeed = 0,
                          const ThingGrid* grid = nullptr);
// (`grid`: the things by where they are, made from `things`; each path is looked along for the things near it only, in
// the list's order, so the same thing is found.)

// ---- The facts -------------------------------------------------------------------------------------------------------

constexpr const char* kFactThingAhead = "thing_ahead";     // yes/no: something in my way within my look-ahead
constexpr const char* kFactThingGap = "thing_gap";         // m: my front to it (about its path)
constexpr const char* kFactThingStop = "thing_stop";       // place: where I stop short of it (thing_stand_off before it;
                                                           // at the line of a junction beyond which it leaves me no room)
constexpr const char* kFactThingSpeed = "thing_speed";     // m/s: how fast it moves along my way
constexpr const char* kFactThingStood = "thing_stood";     // s: how long it has stood still in my way (0 moving)
constexpr const char* kFactRoomToPass = "room_to_pass";    // yes/no: there is room to go round it
constexpr const char* kFactWaited = "waited_for_it";       // s: how long I have stood behind it (0 moving)
constexpr const char* kFactCrosswalkPerson = "person_at_crosswalk";   // yes/no: someone on, or about to step on, a
                                                                      // crosswalk ahead of me (CrosswalkAhead)
constexpr const char* kFactCrosswalkStop = "crosswalk_stop";          // place: crosswalk_stand_off before it
constexpr const char* kNumberCrosswalkStandOff = "crosswalk_stand_off";   // m: how far short of a crosswalk a driver stops
constexpr const char* kNumberCrosswalkDecel = "crosswalk_deceleration";   // m/s2: the braking a driver stops at for a
                                                                          // person waiting at a kerb (no more)
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
    // A CROSSWALK AHEAD with someone on it or about to step on it (ADR-0452), and how far short of it I stop.
    CrosswalkAhead crosswalk;
    Real crosswalkStandOff = 0;
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
