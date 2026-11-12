#ifndef RAYTRACER_APPS_DRIVING_SIGNAL_PROGRAMME_H
#define RAYTRACER_APPS_DRIVING_SIGNAL_PROGRAMME_H

// THE TRAFFIC LIGHTS' CLOCK (driving-model-design.md 9.1; issue #261, task D2; ADR-0406).
//
// Words: a MOVEMENT is one turn through a junction (a connector); a PHASE is a set of movements whose lights are green
// together (right_of_way.h groups them, at load, so that nothing green together has to cross unless the rule set lets one
// of them give way: a far turn on a plain green); a SIGNAL PROGRAMME is a junction's phases in order, each with how long
// it is GREEN, then YELLOW (the warning that red is coming) for the movements losing their green, then ALL-RED (every
// movement losing its green shows red while the last cars to go clear the junction, before the next phase's green);
// the CYCLE is one pass through every phase; the OFFSET shifts a junction's cycle so neighbouring lights do not all
// change on the same tick.
//
// WHAT IT DOES. At load, every junction whose control has lights gets its timing, worked out from the lane graph and the
// rule set's numbers (no number lives here):
//   - YELLOW, from the formula traffic engineers publish (the ITE kinematic change interval): long enough for a driver
//     who sees the yellow just too late to stop comfortably to reach the line before red:
//         yellow = reaction time + approach speed / (2 x deceleration + 2 x gravity x grade)
//     with the approach speed the arriving lane's limit and the grade its climb at the line (uphill shortens it), the
//     longest over the movements losing their green, held between the rule set's shortest and longest yellow;
//   - ALL-RED, from the same source's clearance formula: (the distance to clear + a design vehicle's length) / approach
//     speed, where the distance to clear is how far along the connector its last crossing stretch ends (the "far side of
//     the farthest conflicting lane"; joins and splits are left out, as past them a car only follows another); the
//     longest over the movements losing their green, held between the rule set's shortest and longest. (A car still
//     inside when the next green begins is safe whatever the all-red: on a red, a car past its line is committed, and the
//     next green's cars wait for it: junction_plan.h);
//   - GREEN: the rule set's target cycle, less the yellows and all-reds, shared equally between the phases (the lab has
//     no traffic counts to share it by), held between the rule set's shortest and longest green;
//   - OFFSET: a GREEN WAVE: the phase serving the approach that runs most nearly along the wave's line begins as much
//     later as a car at the wave's speed takes to drive to the junction from the plan's origin along the wave's
//     direction (both rule-set numbers), so lights along that direction turn green one after another for traffic going
//     that way -- and two lights close together serve the same road at nearly the same time, whatever order their
//     phases happen to be numbered in (a red just beyond a green with no room between would otherwise hold a car at the
//     green for good: the lab's junctions 110 and 11, 26 m apart).
//   A movement green in two phases running one after the other stays green through the change between them (no yellow).
//
// STATE IS A PURE FUNCTION OF THE CLOCK: which phase a junction is in, and what each movement's lamp shows, are worked
// out from the sim time and the programme alone (signalAt, lampAt, currentPhase). Nothing is stored per junction while
// cars drive, so the answer is the same every run, for every reader (drivers, the resolver, lamps, tools), in any order,
// and a junction nobody is near costs nothing.
//
// WAITING INSIDE FOR A FAR TURN (docs/driving-briefs/D4). The road builder marks where a far turn may wait inside its
// junction (Path::waitInsideAt). At load each such point is checked against the give-way table: in every phase where the
// turn is green, every crossing point its body would have reached by then (its stretch begins before the point) must be
// one where it goes first. A point that fails is dropped (counted, with a note): a body waiting there would stand in the
// way of something that may move with it. Points are kept only at junctions whose lights run a programme.
//
// THE LIGHTS RUN only when the host registered the light clock (registerLightClock, which declares the numbers it reads,
// so buildRoadRules names any the rule set lacks). Without it a control with lights is treated as the row its `when_dark`
// names (a dark signal is an all-way stop), or, naming none, keeps its lights fixed at their first phase.
//
// Units: metres, seconds.

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/notice.h"
#include "apps/driving/right_of_way.h"
#include "apps/driving/road_rules.h"
#include "apps/driving/rule_set.h"

#include <cstdint>
#include <string>
#include <vector>

namespace driving {

using engine::Real;

// ---- What the light clock reads from the rule set -------------------------------------------------------------------

constexpr const char* kNumberSignalReaction = "signal_reaction_time";        // s: the yellow formula's reaction time
constexpr const char* kNumberSignalDeceleration = "signal_deceleration";     // m/s2: ... its deceleration; the dilemma
constexpr const char* kNumberGravity = "gravity";                             // m/s2: ... its grade term
constexpr const char* kNumberYellowShortest = "yellow_shortest";             // s
constexpr const char* kNumberYellowLongest = "yellow_longest";               // s
constexpr const char* kNumberAllRedVehicleLength = "all_red_vehicle_length"; // m: the clearance formula's vehicle
constexpr const char* kNumberAllRedShortest = "all_red_shortest";            // s
constexpr const char* kNumberAllRedLongest = "all_red_longest";              // s
constexpr const char* kNumberGreenShortest = "green_shortest";               // s
constexpr const char* kNumberGreenLongest = "green_longest";                 // s
constexpr const char* kNumberSignalCycle = "signal_cycle";                   // s: the cycle the greens share out
constexpr const char* kNumberWaveSpeed = "signal_wave_speed";                // m/s: the green wave's speed
constexpr const char* kNumberWaveEast = "signal_wave_east";                  // ratio: its direction, east part
constexpr const char* kNumberWaveNorth = "signal_wave_north";                // ratio: ... north part
// The name the light clock reads the rule set under, in the problems buildRoadRules reports.
constexpr const char* kLightClockReaderName = "the light clock (traffic lights' programmes and the yellow)";

// THE NUMBERS, read once when the rules are handed over.
struct LightTiming {
    Real reaction = 0;
    Real deceleration = 0;
    Real gravity = 0;
    Real yellowShortest = 0, yellowLongest = 0;
    Real vehicleLength = 0;
    Real allRedShortest = 0, allRedLongest = 0;
    Real greenShortest = 0, greenLongest = 0;
    Real cycle = 0;
    Real waveSpeed = 0, waveEast = 0, waveNorth = 0;
};

// Adds the light clock as a reader of the numbers it needs. A host that wants lights to run registers it (after reading
// the rule set); one that does not gets dark lights (when_dark). False (with the reason) when it was registered already.
bool registerLightClock(FactRegistry& registry, std::vector<std::string>* problems = nullptr);
// Whether `registry` has the light clock.
bool hasLightClock(const FactRegistry& registry);
// Reads the numbers. False (the reason in `why`) when one is missing.
bool readLightTiming(const RuleSet& rules, LightTiming& out, std::string* why = nullptr);

// ---- The formulas ---------------------------------------------------------------------------------------------------

// THE YELLOW for traffic arriving at `approachSpeed` (m/s) on a grade `grade` (metres up per metre along; uphill above
// zero): reaction + speed / (2 x deceleration + 2 x gravity x grade), held between the shortest and longest yellow. A
// grade so steep downhill that the bottom is not above zero takes the longest.
Real yellowSeconds(const LightTiming& t, Real approachSpeed, Real grade);
// THE ALL-RED for a movement that is clear of everyone `clearDistance` metres past its line, at `speed` m/s:
// (clearDistance + the design vehicle's length) / speed, held between the shortest and longest all-red.
Real allRedSeconds(const LightTiming& t, Real clearDistance, Real speed);

// ---- At load --------------------------------------------------------------------------------------------------------

// TIMES EVERY PROGRAMME of `row` (its phases, built by buildRightOfWay) and checks every far turn's place to wait inside
// (see the top of this file). `roadRules` gives the limits. Fills each lit junction's cycle and offset and each phase's
// green, yellow and all-red; each connector's waitInsideAt that passes; and the counts and notes.
void timeSignalProgrammes(const engine::LaneGraph& roads, const RoadRules& roadRules, const LightTiming& timing,
                          RightOfWay& row);

// ---- At any moment: pure functions of the clock --------------------------------------------------------------------

// WHERE A JUNCTION'S PROGRAMME IS.
enum class Stage : uint8_t { Green, Yellow, AllRed };
struct SignalNow {
    bool running = false;   // the junction has a timed programme; otherwise nothing below means anything
    uint32_t phase = 0;     // the phase whose green, yellow or all-red it is in
    Stage stage = Stage::Green;
    Real into = 0;          // seconds since this stage began
    Real remaining = 0;     // seconds until it ends
};
SignalNow signalAt(const RightOfWay& row, engine::JunctionId junction, double now);

// THE PHASE THE GIVE-WAY TABLE IS READ AT (right_of_way.h MeetingAnswer per phase): the one whose green, yellow or
// all-red the junction is in. A junction with lights that run no programme shows its first phase (0); a junction without
// lights has one phase, 0.
uint32_t currentPhase(const RightOfWay& row, engine::JunctionId junction, double now);

// WHAT ONE MOVEMENT'S LAMP SHOWS.
enum class Lamp : uint8_t {
    Dark,         // no lights here, or none running a programme
    Green,
    Yellow,
    Red,
    RedMayTurn,   // red, but the control allows this turn on red, giving way to everything
};
constexpr uint8_t kLampCount = 5;
// The lamp of `connector` at `now`: in its phase's green, the phase's light; in the yellow and all-red after it, a
// movement losing its green shows yellow, then red (or red, may turn, where the next phase allows it); one green in the
// next phase too stays green; one not green stays as it was.
Lamp lampAt(const engine::LaneGraph& roads, const RightOfWay& row, engine::PathId connector, double now);
// The words a rule set's `light` fact speaks, in Lamp order from Green ("green", "yellow", "red", "red_may_turn"); and a
// lamp's word ("dark" for Dark).
std::vector<std::string> lampWords();
const char* lampWord(Lamp l);
// The word index of a lamp in lampWords(), or kNoWord for Dark.
uint32_t lampWordIndex(Lamp l);

// THE SAFETY CHECK ON THE PROGRAMMES: at every lit junction, in every stage of every phase (its green, its yellow, its
// all-red), any two movements whose lamps both let traffic in (green or yellow) and whose paths cross or join must be
// allowed to by the rule set: from approaches facing each other, one of them a movement the control lets go on a plain
// green giving way (`gives_way_on_green`). Returns each breach in plain words; empty when sound.
std::vector<std::string> checkSignalProgrammes(const engine::LaneGraph& roads, const RuleSet& rules, const RightOfWay& row);

// ONE JUNCTION'S PROGRAMME AS TEXT: each phase's green, yellow and all-red, the cycle and the offset.
std::string describeProgramme(const engine::LaneGraph& roads, const RightOfWay& row, engine::JunctionId junction);

}  // namespace driving

#endif
