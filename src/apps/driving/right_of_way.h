#ifndef RAYTRACER_APPS_DRIVING_RIGHT_OF_WAY_H
#define RAYTRACER_APPS_DRIVING_RIGHT_OF_WAY_H

// WHO GIVES WAY TO WHOM, WORKED OUT AT LOAD (driving-model-design.md 6.1 b and 9.1; docs/driving-briefs/D1; issue #260).
//
// When the rules are handed to the model, every junction's crossing points (the lane graph's CrossingPoint records,
// ADR-0400), its control (a name in the graph, bound to a row of the rule set's `control` table) and the rule set's
// `movement_rank` are combined into one table: for every meeting of two turns, who gives way. Drivers look the answer
// up; they never work it out (task D3 reads it; nothing reads it yet, so building it changes nothing a car does).
//
// HOW A PAIR IS SETTLED. The control's `settle` list is asked in written order; the first question that tells the two turns
// apart decides (rule_set.h SettleQuestion):
//   - light (lights only, per phase): a green goes before a red; a turn the control allows on red gives way to every
//     other turn, green or red; two greens, or two reds, go on to the next question;
//   - sign_rank: the sign each turn's approach carries at its end (a regulation on the arriving lane whose device has a
//     rank); the rank listed first in RuleSet::ranks goes first, and an approach with no ranked sign shares it;
//   - movement: by RuleSet::movementRank;
//   - kerb_side: the one whose approach is ON THE OTHER'S KERB SIDE goes first. "On my kerb side" is the first other
//     approach found turning from my direction of travel toward the kerb, within the half turn on that side (at a
//     five-way, only the nearest one is). Two approaches facing each other are on neither's kerb side;
//   - arrived_first: answered while driving. The answer is "ask while driving", and the rest of the list is still
//     asked, to find the answer to fall back on if asking ties.
// If the list runs out with nothing decided, the LOWER PATH ID GIVES WAY, and the table notes the pair.
//
// KINDS OF MEETING. A cross or a join is settled as above. A split (two turns from one lane) is settled by nobody: the
// car behind follows the car ahead. A meeting filed under no junction (two junctions' paths, or a connector and a lane,
// crowded together; ADR-0400) has no control to ask: it is "ask while driving", falling back on the lower path id, and
// noted. A crosswalk's or a track's crossing point gets no answer here: people and trains announce (design 4).
//
// LIGHTS. A control whose settle list asks `light` has a signal programme: PHASES, each a set of turns that are green
// together. Two turns may be green in one phase only if they don't meet, or come from approaches facing each other and
// one of them is a movement the control lets go on a plain green giving way (`gives_way_on_green`, the far turn). The
// grouping is the old signal controller's (colouring what conflicts, ADR-0060, ADR-0109), applied first to whole
// approaches (so a plain four-way gets its classic two phases), then to single turns: any turn that meets nothing
// already green in a phase is added to it. A movement in the control's `arrow` list that would have to give way in its
// phase gets an ARROW PHASE of its own just before it, green for those turns only. A turn the control allows on red
// (`turn_on_red`) is "red, may turn" in every phase where it is not green. Phases are in order; their timing and the clock
// that walks them are signal_programme.h's (task D2, #261), which fills the timing fields below.
//
// WHERE A CAR CAN WAIT on a connector (for the junction plan, junction_plan.h): the stretches of its crossing points
// (crosses, joins, crosswalks; not splits, where the car behind follows) are merged along it; the GAPS between them are
// the places a body can stand inside the junction without being in anyone's stretch, if it is short enough to fit.
//
// Same result every run: built in path id order (junctions, their connectors and crossing points in id order), and
// nothing here walks an unordered container.
//
// Units: metres.

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/road_rules.h"
#include "apps/driving/rule_set.h"

#include <cstdint>
#include <string>
#include <vector>

namespace driving {

using engine::Real;

// WHO GIVES WAY at one meeting: the record's side A (CrossingPoint::a), its other side, neither (a split: the car
// behind follows), or asked while driving (then MeetingAnswer::fallback says who if asking ties).
enum class GivesWay : uint8_t { A, Other, Neither, Ask };
constexpr uint8_t kGivesWayCount = 4;

// WHAT DECIDED IT: one of the settle questions (the same numbers as SettleQuestion), the fall-back on path ids, the
// shape of a split, or a meeting filed under no junction.
enum class SettledBy : uint8_t {
    Light = static_cast<uint8_t>(SettleQuestion::Light),
    SignRank = static_cast<uint8_t>(SettleQuestion::SignRank),
    Movement = static_cast<uint8_t>(SettleQuestion::Movement),
    KerbSide = static_cast<uint8_t>(SettleQuestion::KerbSide),
    ArrivedFirst = static_cast<uint8_t>(SettleQuestion::ArrivedFirst),
    LowerPathId,       // nothing told them apart: the lower path id gives way
    Split,             // two turns from one lane: nobody gives way
    AcrossJunctions,   // filed under no junction: ask while driving
};
constexpr uint8_t kSettledByCount = 8;

// THE ANSWER FOR ONE MEETING AT ONE PHASE.
struct MeetingAnswer {
    GivesWay who = GivesWay::Ask;
    SettledBy by = SettledBy::LowerPathId;
    // When `who` is Ask: who gives way if asking ties (A or Other), and what decided that.
    GivesWay fallback = GivesWay::A;
    SettledBy fallbackBy = SettledBy::LowerPathId;
};
bool operator==(const MeetingAnswer& a, const MeetingAnswer& b);
inline bool operator!=(const MeetingAnswer& a, const MeetingAnswer& b) { return !(a == b); }

// WHAT A TURN'S LIGHT SHOWS in one phase.
enum class Light : uint8_t {
    Green,
    Red,
    RedMayTurn,   // red, but the control allows this turn on red, giving way to everything
};
constexpr uint8_t kLightCount = 3;

// "No such thing" in the table's indexes.
constexpr uint32_t kNoControl = 0xFFFFFFFFu;
constexpr uint32_t kNoApproach = 0xFFFFFFFFu;

// TWO APPROACHES FACE EACH OTHER when their directions of travel point more nearly opposite than this (the dot of the
// two unit directions is below it): within about 37 degrees of head on, the figure the old signal controller uses for
// opposing approaches (ADR-0109).
constexpr Real kFacingDot = engine::kOncomingDot;
// ... and run alongside each other when it is above minus this (above 0.8): neither is on the other's kerb side.
constexpr Real kAlongsideDot = -kFacingDot;

// ONE APPROACH TO A JUNCTION: the arriving lanes of one section (a lane in no section is an approach of its own), the
// way its traffic points as it reaches the junction, and the sign it carries.
struct Approach {
    engine::SectionId section;      // not valid for a lane in no section
    engine::PathId firstLane;       // the first of its lanes that a connector leaves (lowest id)
    engine::Vec2 heading;           // unit: the direction of travel at the end of that lane
    int signRank = 0;               // its sign's place in RuleSet::ranks; 0 (shares the first) when it has none
};

// ONE PHASE of a junction's lights: the light each of its connectors shows, in the junction's connector order.
struct Phase {
    bool arrow = false;             // an arrow phase (only `arrow` movements green)
    engine::Range lights;           // into RightOfWay::lights
    // ITS TIMING (signal_programme.h, task D2): seconds of green, then of yellow and of all-red for the movements losing
    // their green as the next phase begins (both 0 when none does). All 0 while no programme runs.
    Real green = 0, yellow = 0, allRed = 0;
    Real length() const { return green + yellow + allRed; }
};

// ONE JUNCTION'S PART OF THE TABLE.
struct JunctionRules {
    uint32_t control = kNoControl;  // its control, an index into RuleSet::controls (kNoControl: the rule set has none)
    engine::Range approaches;       // into RightOfWay::approaches, in order of their first connector
    engine::Range phases;           // into RightOfWay::phases; none when the control has no lights
    // ITS PROGRAMME (signal_programme.h): one pass through every phase, seconds (0: no programme runs: no lights, or the
    // lights are dark), and how far its cycle is shifted, seconds (the green wave).
    Real cycle = 0;
    Real offset = 0;
    uint32_t phaseCount() const { return phases.count > 0 ? phases.count : 1; }   // a junction with no lights has one
    bool running() const { return cycle > 0 && phases.count > 0; }
};

// ONE CONNECTOR'S PART: its approach, its movement, and where a body can stand inside the junction.
struct ConnectorRules {
    uint32_t approach = kNoApproach;   // an index into RightOfWay::approaches
    uint32_t indexInJunction = 0;      // its place in its junction's connectors
    Movement movement = Movement::Straight;   // the connector's own stated turn (B2's), never worked out again
    engine::Range gaps;                // into RightOfWay::gaps: the clear stretches between its crossing stretches
    Real firstStretchAt = 0;           // where its first crossing stretch begins (its length when it has none)
    // WHERE A FAR TURN MAY WAIT INSIDE THE JUNCTION on its green, giving way (the road builder's Path::waitInsideAt, kept
    // only at lights running a programme and only where the check at load passes: signal_programme.h). kNoWaitInside:
    // nowhere.
    Real waitInsideAt = engine::kNoWaitInside;
};

// A STRETCH OF A CONNECTOR clear of every crossing stretch, between two of them: a body standing wholly inside it is
// in nobody's way.
struct ClearGap {
    Real from = 0, to = 0;   // metres along the connector
};

// HOW BUILDING THE TABLE WENT, counted.
struct RightOfWayCounts {
    uint32_t junctions = 0;
    uint32_t junctionsWithLights = 0;
    uint32_t phases = 0;               // over the junctions with lights
    uint32_t meetings = 0;             // path-to-path crossing records answered
    uint32_t settledAtLoad = 0;        // ... where one gives way, at every phase
    uint32_t askWhileDriving = 0;      // ... asked while driving at some phase
    uint32_t splits = 0;
    uint32_t acrossJunctions = 0;      // filed under no junction
    uint32_t byLowerPathId = 0;        // ... decided (or their fall-back decided) by the lower path id, at some phase
    uint32_t junctionsWithoutControl = 0;   // the rule set has no row for their control
    uint32_t junctionsDark = 0;             // lights with no programme running, treated as their `when_dark` row
    // THE PROGRAMMES (signal_programme.h timeSignalProgrammes).
    uint32_t junctionsTimed = 0;            // lights running a programme
    Real shortestCycle = 0, longestCycle = 0;
    uint32_t waitInsidePoints = 0;          // far turns that may wait inside on their green
    uint32_t waitInsideDropped = 0;         // ... places the builder marked that the check at load refused
};

// THE WHOLE TABLE.
struct RightOfWay {
    std::vector<JunctionRules> junctions;     // by junction id
    std::vector<ConnectorRules> paths;        // by path id (lanes have the defaults)
    std::vector<Approach> approaches;
    std::vector<Phase> phases;
    std::vector<Light> lights;
    std::vector<ClearGap> gaps;
    // Per crossing record (by CrossingId): its answers in `answers`, one per phase of its junction (one when filed under
    // no junction); none (count 0) for a crosswalk's or a track's.
    std::vector<engine::Range> answersOf;
    std::vector<MeetingAnswer> answers;
    RightOfWayCounts counts;
    // What the loader noted, one plain line each, in id order: junctions whose control the rule set lacks, and pairs
    // left to the lower path id.
    std::vector<std::string> notes;

    // The answer for crossing record `crossing` at phase `phase` of its junction (0 for a junction with no lights); null
    // for an unknown record, one with no answer, or a phase out of range.
    const MeetingAnswer* answer(engine::CrossingId crossing, uint32_t phase = 0) const;
    // The light connector `connector` shows in phase `phase` of its junction: Green when its junction has no lights.
    Light light(const engine::LaneGraph& roads, engine::PathId connector, uint32_t phase) const;
};

// BUILDS THE TABLE for every junction of `roads`. `roadRules` is buildRoadRules' binding (it says which control row each
// of the graph's control names is, and what each regulation kind means).
RightOfWay buildRightOfWay(const engine::LaneGraph& roads, const RuleSet& rules, const RoadRules& roadRules);

// (The phase a junction's table is read at, a pure function of the clock, is signal_programme.h's currentPhase.)

// FROM ONE PATH'S SIDE: at crossing record `crossing`, does a car on `path` give way? GivesWay::A/Other read from its
// side: `mine` is Ask, Neither, or whether I give way (true) or the other does (false). The other path is put in
// `otherPath` (not valid for a crosswalk's or a track's).
enum class MyTurn : uint8_t { GiveWay, GoFirst, Neither, Ask };
constexpr uint8_t kMyTurnCount = 4;
MyTurn myTurn(const engine::LaneGraph& roads, const MeetingAnswer& a, engine::CrossingId crossing, engine::PathId path,
              engine::PathId* otherPath = nullptr);

// WHETHER APPROACH `a` IS ON APPROACH `b`'S KERB SIDE among the approaches `all` (unit headings of travel) of one
// junction, for traffic keeping to `drivesOn`'s side: `a` is the first approach found turning from b's direction of
// travel toward its kerb, within the half turn on that side; approaches facing `b` or running alongside it are passed
// over. Exposed for the five-way test.
bool onKerbSideOf(const std::vector<engine::Vec2>& all, std::size_t a, std::size_t b, DrivesOn drivesOn);

// ---- Words -------------------------------------------------------------------------------------------------------------

// A movement as a driver says it: "straight on", "the near turn", "the far turn", "the U-turn".
const char* movementPhrase(Movement m);
// Where an approach comes from, as a compass point on the plan ("the north", "the south-east"): north is -z, as the
// world's compass reads it (watch_system.h).
std::string fromWords(const engine::Vec2& heading);
// What decided an answer, in words ("by the lights", "by movement", ...).
const char* settledByWords(SettledBy by);
// A path as the table sees it: "the far turn from the north (connector 345)", or "lane 12".
std::string pathTurnWords(const engine::LaneGraph& roads, const RightOfWay& row, engine::PathId p);

// ONE JUNCTION'S TABLE AS TEXT: its control and phases, then every meeting of two turns in id order, in words ("the far
// turn from the north (connector 345) gives way to straight on from the south (connector 350), by movement"). For the
// driver card and for checking by hand. Same table, same text.
std::string describeJunctionRules(const engine::LaneGraph& roads, const RuleSet& rules, const RightOfWay& row,
                                  engine::JunctionId junction);
// The counts and the notes, as text.
std::string describeRightOfWay(const RightOfWay& row);

}  // namespace driving

#endif
