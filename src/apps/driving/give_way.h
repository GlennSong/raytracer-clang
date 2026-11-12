#ifndef RAYTRACER_APPS_DRIVING_GIVE_WAY_H
#define RAYTRACER_APPS_DRIVING_GIVE_WAY_H

// GIVING WAY AT A JUNCTION, AND INSIDE IT (driving-model-design.md 9.3, 9.4; issues #262 and #263,
// docs/driving-briefs/D3-give-way.md and D4-inside-the-junction.md).
//
// At each decision a driver JUDGES the junctions it has planned for, nearest first, from the board of announcements
// (announce.h). For each crossing point along its connector, and each car announced there that goes BEFORE it -- which
// the one conflict resolver says (conflict.h), never this file --:
//   - it may go AHEAD of that car if its own tail will be clear of the stretch before that car's front arrives, by the
//     gap it needs; or BEHIND it if it arrives after that car's tail is clear, by the gap it needs. Where the resolver
//     says the other goes first because its body is already on the crossing point or because it committed first
//     (conflict.h followOnly), only BEHIND, by the same gaps: that car's crossing point is "a car to follow until its tail
//     is clear";
//   - otherwise it must GIVE WAY. Where it stops: at the junction's line (the connector's start) while it can still stop
//     there braking comfortably; once it cannot (it is past the line, committed or not, or too fast to stop at it), INSIDE
//     the junction, short of the crossing point: at the last of its waiting points (junction_plan.h) before it, or else
//     with its front at the crossing point's stretch. A car whose front is already on that stretch cannot keep out of
//     it, and goes on. The judgement records to whom, at which crossing point, why that car goes first, the gaps, and
//     where it stops.
// Room beyond the junction is task D5.
//
// WHO GOES FIRST is the conflict resolver's (conflict.h): one record per pair of cars, settled once a tick, so the one
// that goes first never looks at the one going second (if both looked, both would wait) and no two cars wait for each
// other in a ring. Committed cars judge too: a car inside the junction still keeps out of the way of a body on a
// crossing point and of a car that committed before it.
// A car announced as NOT GOING (stopping short of the crossing for another reason) is passed over.
//
// NO DITHERING: two gaps from the rule set, a comfortable one and a smaller hard one. A driver decides to go by the
// comfortable one; once it has decided, it keeps going unless the hard one is broken; and once it could no longer stop
// before the line at comfortable braking, it is going (announce.h isGoing): it no longer stops at the line, and stops
// inside only for a car the resolver puts before it whose gap the hard one breaks.
// WHEN A DRIVER DECIDES TO GO (not who goes first: these say when a clear road counts as a decision), in one place,
// judgeGiveWay:
//   - within its decision horizon (its comfortable stopping distance, one decision's travel and at_line_within):
//     deciding further out, in a queue say, is deciding on guesses that change before it gets there;
//   - not NOT GOING there (held, blocked by a stopped car beyond the line): such a driver still judges, and stops if it
//     must, but a clear road is no decision to go: it may be let go this very decision, and must not then cross without
//     having judged;
//   - not still OWING A FULL STOP at the line (below): it decides once it has come to rest there (ADR-0403).
// A driver that is going has passed all three.
//
// PATIENCE: the comfortable gap shrinks the longer the driver has stood at the line, at the rule set's gap_shrink seconds
// of gap per second waited divided by the person's own `patience`, down to the rule set's gap_floor (never below the hard
// gap). It is not a timer that overrides a rule: the floor is still a safe gap.
//
// A FULL STOP FIRST (an all-way stop, a stop sign) is a rule in the rule set, not here: it stops at `next_line` until
// `stopped_at_line` ("I came to rest at this line", JunctionPlan::cameToRest) is yes. Such a stop is the junction's own
// (StopFor::FullStop): the car still announces that it will cross, after stopping. Who then goes is by who came to rest
// at the line first (its arrival is fixed then: the resolver's arrived_first).
//
// SEVERAL JUNCTIONS IN VIEW: the nearest first; the first where it must give way is where it stops. If that is not the
// nearest and its body would not fit between the two junctions (the next line nearer the end of the connector before it
// than its length and a standstill gap), it stops at the line before instead (task D5 completes this with room claims).
//
// THE FACTS a rule set reads (registerJunctionFacts): must_give_way, give_way_line (the place to stop by, which waits for
// the car given way to: the junction's line, or inside the junction short of the crossing point), give_way_gap and
// give_way_gap_needed (seconds), committed and stopped_at_line (yes/no), and for full stops next_line, everyone_stops and
// sign_rank. The judgement is worked out once per decision (judgeGiveWay) and the facts read it.
//
// TRAFFIC LIGHTS (issue #261; signal_programme.h is the clock). The table is read at the phase the junction is in now. A
// driver that can still stop before a RED decides nothing there (the light holds it; the red starts its approach afresh:
// announce.h noteAtTheLine). The facts light, light_line and can_stop_for_light say what its turn's lamp shows at the
// lights ahead, where it stops for them and whether it can (the dilemma at a yellow); the rule set's rules act on them.
// WAITING INSIDE: on its own green, a far turn that must give way at a crossing point at or beyond its place to wait
// inside (JunctionPlan::waitInsideAt) pulls forward to it instead of stopping at the line. A car from the same lane that
// would leave by another turn (a split) waits behind a car standing inside on the split's stretch (cause BehindInMyLane:
// its tail may be off the lane, where following no longer sees it), and is not this junction's traffic while it does.
//
// Same result every run: the board is sorted, the resolver settles each conflict once a tick from it, and crossing points
// are read in their order along the connector.
//
// Units: metres, seconds.

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/announce.h"
#include "apps/driving/conflict.h"
#include "apps/driving/junction_plan.h"
#include "apps/driving/notice.h"
#include "apps/driving/right_of_way.h"
#include "apps/driving/rule_set.h"
#include "apps/driving/signal_programme.h"   // the lights' clock: the lamp a driver sees, and the yellow's numbers
#include "apps/driving/traffic_state.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace driving {

using engine::Real;

// ---- What the junction step reads from the rule set ----------------------------------------------------------------

constexpr const char* kNumberJunctionLookAhead = "junction_look_ahead";   // s: who announces
constexpr const char* kNumberGapComfortable = "gap_comfortable";          // s: the gap a driver decides to go by
constexpr const char* kNumberGapHard = "gap_hard";                        // s: the gap a driver who has decided keeps
constexpr const char* kNumberGapFloor = "gap_floor";                      // s: patience shrinks the comfortable gap to this
constexpr const char* kNumberGapShrink = "gap_shrink";                    // ratio: seconds of gap lost per second waited
constexpr const char* kNumberAtLineWithin = "at_line_within";             // m: "at the line"
constexpr const char* kSettingPatience = "patience";                      // ratio: a person's patience (1: usual)
// The name the junction step reads the rule set under, in the problems buildRoadRules reports.
constexpr const char* kJunctionReaderName = "the junction step (announcing, and judging gaps to give way)";

// The facts.
constexpr const char* kFactMustGiveWay = "must_give_way";             // yes/no: I must give way at a junction ahead
constexpr const char* kFactGiveWayLine = "give_way_line";             // place: where to stop to give way (the line,
                                                                      // or inside, short of the crossing point)
constexpr const char* kFactGiveWayGap = "give_way_gap";               // s: the gap I have where I give way
constexpr const char* kFactGiveWayGapNeeded = "give_way_gap_needed";  // s: the gap I need there
constexpr const char* kFactCommitted = "committed";                   // yes/no: past my last waiting point
constexpr const char* kFactStoppedAtLine = "stopped_at_line";         // yes/no: I came to rest at the next line
// The line of the nearest junction ahead, as a place to come to a FULL STOP at before going on (StopFor::FullStop):
// a rule set says { when = { all = { "everyone_stops", { "not", "stopped_at_line" } } }, give = { stop_at = "next_line" } }
// for an all-way stop, and { all = { { "sign_rank", "is", "stop" }, { "not", "stopped_at_line" } } } for a stop sign.
constexpr const char* kFactNextLine = "next_line";                    // place
constexpr const char* kFactEveryoneStops = "everyone_stops";          // yes/no: that junction's control says so
constexpr const char* kFactSignRank = "sign_rank";                    // word: the rank of the sign on my approach
// TRAFFIC LIGHTS (issue #261): among the junctions ahead whose lights run a programme (signal_programme.h) and whose line
// my front has not passed by more than at_line_within, the nearest whose lamp for my turn is not green -- a green near
// junction must not hide a red just beyond it -- or else the nearest. Absent where there is none (no lights, or dark).
// What my turn's lamp shows there: "green", "yellow", "red", "red_may_turn" (signal_programme.h lampWords).
constexpr const char* kFactLight = "light";                           // word
// Where I stop for it: its line (even where my body would not fit between the junction before and it: see give_way.cpp).
// A rule set says { when = { "light", "is", "red" }, give = { stop_at = "light_line" } }.
constexpr const char* kFactLightLine = "light_line";                  // place
// THE DILEMMA: can I stop at light_line, holding what I decide now for my decide interval and then braking at the rule
// set's signal_deceleration (the deceleration the yellow is timed for)? A driver who cannot is, by the yellow's own
// formula, able to reach the line before the red. { all = { { "light", "is", "yellow" }, "can_stop_for_light" } }.
constexpr const char* kFactCanStopForLight = "can_stop_for_light";    // yes/no

// THE NUMBERS, read once when the rules are handed over.
struct JunctionNumbers {
    Real lookAheadTime = 0;
    Real gapComfortable = 0;
    Real gapHard = 0;
    Real gapFloor = 0;
    Real gapShrink = 0;
    Real atLineWithin = 0;
    Real comfortableSideAccel = 0;   // comfortable_side_accel: the speed a car holds through a turn
    Real standstillGap = 0;          // standstill_gap: room between two junctions; a stopped car beyond the line
    PersonalSetting patience;
    uint32_t patienceHash = 0;
    PersonalSetting gapKept;         // gap_kept: the following interval of a queue
    uint32_t gapKeptHash = 0;
};

// Adds the junction facts above, and the junction step as a reader of the numbers and settings it needs, so
// buildRoadRules reports a rule set that lacks one. A host that wants giving way registers these (after
// registerBuiltInFacts, and after reading the rule set: `ranks` are its sign ranks, the words of `sign_rank`); a host
// that does not gets no junction step at all. False (with the reasons) when one was registered already.
bool registerJunctionFacts(FactRegistry& registry, const std::vector<std::string>& ranks,
                           std::vector<std::string>* problems = nullptr);
// Whether `registry` has the junction step (registerJunctionFacts ran on it).
bool hasJunctionStep(const FactRegistry& registry);
// Reads the numbers from `rules`. False (the reason in `why`) when one is missing.
bool readJunctionNumbers(const RuleSet& rules, JunctionNumbers& out, std::string* why = nullptr);

// ---- The judgement --------------------------------------------------------------------------------------------------

// A junction's control: its row of the rule set's `control` table, or null (no junction, or no row).
const ControlKind* junctionControl(const RightOfWay& row, const RuleSet& rules, engine::JunctionId junction);

// IS THE STOP A CAR HOLDS THE JUNCTION'S OWN: to give way where the plan's judgement put it (the line, or inside the
// junction short of a crossing point), or a full stop first at the plan's line (each within at_line_within). Then the
// car still announces that it will cross, after; any other stop at or short of the line (a hold, its destination, giving
// way at an earlier junction) means it is not going there.
bool stopIsJunctionsOwn(const Car& car, const JunctionPlan& plan, Real atLineWithin);
// MUST THE CAR COME TO REST at the plan's line first: it holds a full stop there and has not yet come to rest.
bool stopsFirstAt(const Car& car, const JunctionPlan& plan, Real atLineWithin);

// THE COMFORTABLE GAP after waiting `waited` seconds at the line, for a person of patience `patience`: shrunk from
// gap_comfortable at gap_shrink / patience seconds per second waited, never below the larger of gap_floor and gap_hard.
Real comfortableGap(const JunctionNumbers& n, Real waited, Real patience);

// WHAT ONE DRIVER JUDGES FROM.
struct GiveWayInputs {
    const engine::LaneGraph& roads;
    const RightOfWay& row;
    const RuleSet& rules;
    const RoadRules& roadRules;   // what each regulation kind on the road means (a stop sign's stop line)
    const AnnouncementBoard& board;
    const ConflictResolver& resolver;   // who goes first, settled from this tick's board (conflict.h)
    const JunctionNumbers& numbers;
    const Car& me;
    Real length = 0;          // my body, metres
    Real brake = 0;           // my comfortable braking, m/s2
    Real reaction = 0;        // seconds between my decisions
    PersonId person = 0;
    double now = 0;           // the clock, seconds
    const JunctionPlans& plans;
    // Per plan: the time the board has me reaching its line, seconds (kNever: too far to announce it yet).
    std::array<Real, kMaxJunctionPlans> lineIn{};
    // The light clock's numbers when the lights run a programme (signal_programme.h); null when they are dark.
    const LightTiming* lights = nullptr;
    Real hardestBrake = 0;    // my hardest braking, m/s2 (a driver already stopping for a light keeps stopping while it can)
};

// WHAT ONE DRIVER JUDGED.
struct GiveWayVerdict {
    bool judged = false;             // the junction step ran for this driver
    bool giveWay = false;            // it must give way somewhere ahead
    uint8_t stopPlan = 0;            // ... at this plan's junction (its line, or inside: why.stopAlong)
    Real toLine = 0;                 // ... stopping this many metres from its front
    GiveWayJudgement why;            // ... and why (to whom, the gaps)
    // Per plan (in the plans' order): what was decided this time (Undecided: not judged, kept as it was), and why.
    std::array<GiveWayDecision, kMaxJunctionPlans> decision{};
    std::array<GiveWayJudgement, kMaxJunctionPlans> judgement{};
    bool committed = false;          // past the last waiting point of its nearest junction
    // THE NEAREST JUNCTION WHOSE LINE IS STILL AHEAD: how far its line is, whether the car came to rest there, whether
    // its control makes everyone stop first, and the rank of the sign on the car's approach (RuleSet::ranks; an approach
    // with no ranked sign shares the first, as right_of_way.h reads it).
    bool hasNextLine = false;
    Real toNextLine = 0;
    bool stoppedAtLine = false;
    bool everyoneStops = false;
    int signRank = 0;
    // THE NEAREST LIGHTS AHEAD (issue #261; the light facts above): what my turn's lamp shows, how far ahead I stop for
    // it, and whether I can stop there (the dilemma).
    bool hasLight = false;
    Lamp lamp = Lamp::Dark;
    Real toLightLine = 0;
    bool canStopForLight = false;
};

// JUDGES every planned junction, nearest first (see the top of this file).
void judgeGiveWay(const GiveWayInputs& in, GiveWayVerdict& out);

}  // namespace driving

#endif
