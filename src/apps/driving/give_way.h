#ifndef RAYTRACER_APPS_DRIVING_GIVE_WAY_H
#define RAYTRACER_APPS_DRIVING_GIVE_WAY_H

// GIVING WAY AT A JUNCTION (driving-model-design.md 9.3; issue #262, docs/driving-briefs/D3-give-way.md).
//
// At each decision a driver JUDGES the junctions it has planned for, nearest first, from the board of announcements
// (announce.h) and the right-of-way table (right_of_way.h). For each crossing point along its connector, and each car
// announced there that it must give way to:
//   - it may go AHEAD of that car if its own tail will be clear of the stretch before that car's front arrives, by the
//     gap it needs; or BEHIND it if it arrives after that car's tail is clear, by the gap it needs;
//   - otherwise it must GIVE WAY: it stops at the junction's line (the connector's start). The judgement records to
//     whom, at which crossing point, the gap it needed and the gap it had.
// Waiting inside the junction is task D4; room beyond is task D5.
//
// WHO GIVES WAY TO WHOM at a meeting comes from the table. The one with right of way never looks at the one giving way
// (if both looked, both would wait). A meeting the table leaves to be ASKED WHILE DRIVING is settled in this order:
//   1. a car that is GOING goes before one that is not: it cannot stop at its line (past it, or too fast to stop there
//      braking comfortably, while not giving way), or, having decided to go, will not be able to by the time it next
//      decides. It cannot give way, and it judged its gap against the other before it got there (announce.h isGoing;
//      the decision interval's look ahead is what stops a driver deciding to go a moment before such a car becomes one
//      it must give way to);
//   2. whoever REACHES ITS LINE FIRST, by the tick of its ARRIVAL (JunctionPlan::arrivalTick). The arrival is fixed
//      once: when the driver first judges the junction, as the tick its own estimate then has it at the line; for a
//      full stop, the tick it comes to rest there; for a car that never judged, the tick it could no longer stop. A car
//      whose arrival is fixed goes before one whose is not (that one is further off, and has not judged yet);
//   3. neither fixed yet: the table's fall-back (kerb side, movement, lower path id);
//   4. both arriving in the same tick, and the fall-backs among the cars arriving at this junction in that tick run in
//      a RING (four cars reaching a four-way together, each with another on its kerb side): within the ring, the lower
//      car id goes first. That is the tie-break that cannot tie (design 9.3), and it is the same for every car in the
//      ring, so exactly one goes.
// Why fixed, and not read off the line as each car gets there (the brief's words): read off the line, a car that
// stopped at its line to give way to me would, by having got there first, take the right of way from me as I arrived,
// and I would stop for it: the order flipped under drivers that had already decided. Fixed when first judged, the order
// of any two cars never changes, so it has no loop, and nobody is made to change their mind by it.
// A car announced as NOT GOING (stopping short of the crossing for another reason) is passed over.
//
// NO DITHERING: two gaps from the rule set, a comfortable one and a smaller hard one. A driver decides to go by the
// comfortable one; once it has decided, it keeps going unless the hard one is broken; and once it could no longer stop
// before the line at comfortable braking, it is going whatever happens (announce.h isGoing). A driver DECIDES only once
// it is within its decision horizon (its comfortable stopping distance, one decision's travel and at_line_within):
// deciding further out, in a queue say, is deciding on guesses that change before it gets there. A driver that is NOT
// GOING there (held, blocked by a stopped car beyond the line) still judges, and stops if it must, but a clear road is no
// decision to go: it may be let go this very decision, and must not then cross without having judged.
//
// PATIENCE: the comfortable gap shrinks the longer the driver has stood at the line, at the rule set's gap_shrink seconds
// of gap per second waited divided by the person's own `patience`, down to the rule set's gap_floor (never below the hard
// gap). It is not a timer that overrides a rule: the floor is still a safe gap.
//
// A FULL STOP FIRST (an all-way stop, a stop sign) is a rule in the rule set, not here: it stops at `next_line` until
// `stopped_at_line` ("I came to rest at this line", JunctionPlan::cameToRest) is yes. Such a stop is the junction's own
// (StopFor::FullStop): the car still announces that it will cross, after stopping. Who then goes is by who came to rest
// at the line first, as above.
//
// SEVERAL JUNCTIONS IN VIEW: the nearest first; the first where it must give way is where it stops. If that is not the
// nearest and its body would not fit between the two junctions (the next line nearer the end of the connector before it
// than its length and a standstill gap), it stops at the line before instead (task D5 completes this with room claims).
//
// THE FACTS a rule set reads (registerJunctionFacts): must_give_way, give_way_line (a place to stop by, which waits for
// the car given way to), give_way_gap and give_way_gap_needed (seconds), committed and stopped_at_line (yes/no), and for
// full stops next_line, everyone_stops and sign_rank. The judgement is worked out once per decision (judgeGiveWay) and
// the facts read it.
//
// Same result every run: the board is sorted, crossing points are read in their order along the connector, and the
// ring is found over a few cars by their ids.
//
// Units: metres, seconds.

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/announce.h"
#include "apps/driving/junction_plan.h"
#include "apps/driving/notice.h"
#include "apps/driving/right_of_way.h"
#include "apps/driving/rule_set.h"
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
constexpr const char* kFactGiveWayLine = "give_way_line";             // place: the line to stop at to give way
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

// The phase of a junction's lights a driver judges by. The lights' clock is task D2: until it runs, every junction shows
// its first phase (0), and this is the one place that says so.
uint32_t currentPhase(const RightOfWay& row, engine::JunctionId junction);
// A junction's control: its row of the rule set's `control` table, or null (no junction, or no row).
const ControlKind* junctionControl(const RightOfWay& row, const RuleSet& rules, engine::JunctionId junction);

// IS THE STOP A CAR HOLDS THE JUNCTION'S OWN: at the plan's line (within at_line_within), to give way there or for a
// full stop first. Then the car still announces that it will cross, after; any other stop at or short of the line (a
// hold, its destination, giving way at an earlier junction) means it is not going there.
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
    const AnnouncementBoard& board;
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
};

// WHAT ONE DRIVER JUDGED.
struct GiveWayVerdict {
    bool judged = false;             // the junction step ran for this driver
    bool giveWay = false;            // it must give way somewhere ahead: stop at a line
    uint8_t stopPlan = 0;            // ... at the line of this plan
    Real toLine = 0;                 // ... this many metres from its front
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
    uint32_t tieGroupsCut = 0;       // tie groups bigger than kMaxTieGroup met (counted: CapCounts::tieGroupsCut)
};

// The most cars a ring of fall-backs is looked for among (cars arriving at one junction's lines in the same tick). More
// is counted (CapCounts::tieGroupsCut); those past the cap go by the plain fall-back.
constexpr std::size_t kMaxTieGroup = 8;

// JUDGES every planned junction, nearest first (see the top of this file).
void judgeGiveWay(const GiveWayInputs& in, GiveWayVerdict& out);

// WHETHER I GO FIRST at a meeting the table leaves to be asked while driving: `mine` and `theirs` are our announcements
// on it, `answer` the table's (its fall-back), `iAmA` whether I am its side A. The ring (rule 4 above) is looked for
// on the board among the cars arriving at `junction`'s lines in the same tick. Exposed for the tests.
bool goFirstWhenAsked(const engine::LaneGraph& roads, const RightOfWay& row, const AnnouncementBoard& board,
                      engine::JunctionId junction, const MeetingAnswer& answer, bool iAmA, const Announcement& mine,
                      const Announcement& theirs, uint32_t* tieGroupsCut = nullptr);

}  // namespace driving

#endif
