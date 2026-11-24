#ifndef RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_TABLE_H
#define RAYTRACER_APPS_CITYSIM_BRAIN_BRAIN_TABLE_H

// THE BRAIN'S STATES AND TRANSITIONS, AS DATA (docs/agent-brain-design.md section 5; ADR-0440). The list of states, the
// events each accepts and what each event leads to are rows of assets/scripts/brain.lua, read once at load
// (scripting/brain_lua.h) into a BrainSpec and compiled here into flat tables the machine indexes by number. The
// machine (brain.h) names no state: what a state DOES is a small table of functions (StateFns) registered under the
// state's NAME, and compiling refuses a state in the data with no functions and functions with no state in the data.
//
// WHAT AN EVENT CAN LEAD TO (the right-hand side of a row's `on`, and an event's `otherwise`): the name of a state, or
// one of these verbs, which are the machine's own mechanisms:
//   next_leg        this leg is done: begin the plan's next leg, or, after the last, arrive
//   replan          drop the plan and plan again from where the person stands
//   replan_without  the same, first ruling out the way of the leg that failed (for as long as this intent lasts)
//   replan_afresh   the same, first allowing again every way that was ruled out: what a person with no way there does
//                   when they try again after a while, or when something they have changed (ADR-0443)
//   resume          leave a state that holds and carry on with what was interrupted, from where it had got to
//   redirect        stay in this state and in the vehicle of the leg under way, and go somewhere else in it: the plan is
//                   made again as "in this vehicle from here to where it will be left, then on to the place wanted",
//                   and the state's skill is told the leg now ends there (ADR-0442). Only a row of a state whose
//                   behaviour can be sent elsewhere (StateFns::retarget) may say it. When no such plan can be made,
//                   nothing changes, and the new intent is taken up when the old trip ends
//   ignore          nothing happens
//   refuse          nothing happens, and the machine counts it as an event that should not have come

#include "brain_record.h"
#include "brain_skills.h"

#include <string>
#include <utility>
#include <vector>

namespace citysim {

struct TravelCatalog;

namespace brain {

constexpr int kMaxStates = 32;
constexpr int kMaxEvents = 32;
constexpr int kMaxIntents = 16;
constexpr int kMaxWays = 16;      // one bit each of AgentBrain::ruledOut
constexpr int kMaxRaises = 4;     // events one state's functions may raise themselves

// ---- THE DATA AS READ (names, before anything is checked) ----------------------------------------------------------------
struct StateSpec {
    std::string name;
    std::string label;            // what a person in it is said to be doing, on screen ("driving"); empty: its name
    std::string group;           // for a reader: travelling, transit, at_place, resting, body, held
    std::string skill = "none";   // the skill it asks: one of BehaviourSet::skills, or "none"
    // HOW IT ENDS. "never": only an event ends it. "skill": the skill's report ends it for a near person, and the
    // clock for a far one (who has no skill running). "clock": the clock ends it for everyone.
    std::string ends = "never";
    std::string clock;            // the event the clock raises when the state's planned time is up
    // WHERE THE PERSON IS while in it, for placing them and for planning again: "place" (where they last arrived or
    // set out from), "leg_start", "along_leg", "leg_end".
    std::string where = "place";
    bool holds = false;           // entering it interrupts what the person was doing, to be carried on or dropped later
    bool seen = true;             // false: the person is out of sight while in it (inside a building), not drawn
    // HOW LONG IT LASTS, in seconds, for a state whose time is its own and not a leg's (ADR-0443: how long a person
    // with no way there waits before trying again). 0: not stated. Only for a state the clock ends whose behaviour
    // has no `lasts` function of its own.
    double seconds = 0;
    std::vector<std::pair<std::string, std::string>> on;   // event -> a state or a verb
};
struct EventSpec {
    std::string name;
    std::string otherwise = "refuse";   // what it does in a state whose row does not list it
};
struct IntentSpec {
    std::string name;
    std::string arrive;           // the state a person is in once the intent's place is reached
};
struct BrainSpec {
    std::string file = "brain.lua";   // named in every message
    std::string start;            // the state a new person is in
    std::string noPlan;           // the state of a person who wants to be somewhere and has no way there
    std::string intentEvent;      // the event a new intent arrives as
    // The event a person is sent when something they plan with has changed (ADR-0443: their household's car became
    // free or was moved, they were given a board). Empty: none is sent.
    std::string changedEvent;
    std::vector<StateSpec> states;
    std::vector<EventSpec> events;
    std::vector<IntentSpec> intents;
};

// ---- WHAT A STATE DOES ---------------------------------------------------------------------------------------------------
class Machine;
struct BrainTable;

// What a state's function is handed: the one person it is called for, the time, and the world.
struct StepContext {
    Machine* machine = nullptr;
    const BrainTable* table = nullptr;
    World* world = nullptr;
    WalkSkill* walk = nullptr;
    DriveSkill* drive = nullptr;
    RideSkill* ride = nullptr;
    AgentId agent = kNoAgent;
    BrainTime now = 0;
    bool skillsRun = false;       // the person is near, so skills move them (false: far, the clock alone)
};

// Why a state is being left.
enum class ExitWhy : uint8_t {
    Done,        // it finished: the next state follows on from it
    Abandoned,   // the plan it was part of is dropped (planning again)
    Held,        // interrupted by a state that holds (possessed, knocked down)
    Lowered,     // the person is no longer near: its skill stops, the state itself carries on by the clock
};

struct StateFns {
    // The state begins. Returns an event to raise at once (kNoId: none): how a state says "this cannot be done"
    // (the car is gone, there is no route).
    EventId (*enter)(StepContext&, AgentBrain&) = nullptr;
    // The state is taken up part-way: a far person raised to near, or a held one carrying on. `fractionDone` is how
    // much of the state's planned time is already behind them. The same return as enter.
    EventId (*enterMidLeg)(StepContext&, AgentBrain&, Real fractionDone) = nullptr;
    void (*exit)(StepContext&, AgentBrain&, ExitWhy) = nullptr;
    // Once a tick for a near person.
    void (*step)(StepContext&, AgentBrain&) = nullptr;
    // An event has come for the person in this state, before the table's answer is carried out.
    void (*onEvent)(StepContext&, AgentBrain&, const BrainEvent&) = nullptr;
    // THE LEG UNDER WAY NOW ENDS SOMEWHERE ELSE (the verb `redirect`): the plan has been made again, and its first leg
    // is this one with a new end. The state tells its skill. The same return as enter.
    EventId (*retarget)(StepContext&, AgentBrain&) = nullptr;
    // How long the state lasts as planned, in milliseconds (kNever: not known). The machine sets the state's due time
    // from it and works out fractions with it.
    BrainTime (*lasts)(const StepContext&, const AgentBrain&) = nullptr;
    // False: the state is in the data and not built yet. Nobody can enter it; trying says so.
    bool built = true;
};

struct Behaviour {
    std::string state;
    StateFns fns;
    std::vector<std::string> raises;   // the events its functions return, by name, in the order they index them
};
struct BehaviourSet {
    std::vector<Behaviour> rows;
    std::vector<std::string> skills;   // the skills C++ has an interface for, by the name the data uses
    void add(const std::string& state, const StateFns& fns, const std::vector<std::string>& raises = {});
    void addNotBuilt(const std::string& state);
    void addSkill(const std::string& name);
};

// ---- THE COMPILED TABLE ------------------------------------------------------------------------------------------------------
enum class Verb : uint8_t { Refuse, Ignore, Go, NextLeg, Replan, ReplanWithout, Resume, Redirect, ReplanAfresh };
struct Answer {
    Verb verb = Verb::Refuse;
    StateId state = kNoId;        // Go: where
};
enum class Ends : uint8_t { Never, Skill, Clock };
enum class Where : uint8_t { Place, LegStart, AlongLeg, LegEnd };

struct StateRow {
    std::string name, group, skill;
    std::string label;            // the data's display name (never empty: the name where the data gives none)
    Ends ends = Ends::Never;
    EventId clock = kNoId;
    Where where = Where::Place;
    bool holds = false;
    bool seen = true;
    BrainTime fixedMillis = 0;    // the data's `seconds` (0: none): how long the state lasts when its behaviour does not say
    StateFns fns;
    EventId raises[kMaxRaises] = {kNoId, kNoId, kNoId, kNoId};
};
struct IntentRow {
    std::string name;
    StateId arrive = kNoId;
};
// A way of travelling as the brain needs it: bound from travel.lua by bindWays.
struct WayRow {
    std::string name;
    StateId begins = kNoId;       // the state a leg of it begins in
    BrainTime startMillis = 0;    // the way's fixed time at the start of a leg (pulling out)
    BrainTime endMillis = 0;      // ...and at its end (parking)
    // The longest a leg of it waits to set off (travel.lua `set_off_wait_minutes`: a car for a clear lane beside its
    // space) before it is given up as blocked. 0: no limit.
    BrainTime setOffWaitMillis = 0;
};

struct BrainTable {
    std::string file;
    std::vector<StateRow> states;
    std::vector<std::string> events;
    std::vector<IntentRow> intents;
    std::vector<WayRow> ways;
    Answer on[kMaxStates][kMaxEvents];
    StateId start = kNoId, noPlan = kNoId;
    EventId intentEvent = kNoId;
    EventId changedEvent = kNoId;   // kNoId: the data names none

    int state(const std::string& name) const;     // -1: none
    int event(const std::string& name) const;
    int intent(const std::string& name) const;
};

// The data checked and compiled against the registered behaviours. False, with `err` naming the file and the key at
// fault, on: a state with no behaviour; a behaviour with no state; a skill C++ has no interface for; an `on` key that
// is not an event; an answer that is neither a state nor a verb; `resume` in a state that does not hold; `redirect`
// in a state whose behaviour cannot be sent elsewhere, or as an event's `otherwise`; an event a
// behaviour raises that the data lacks; a missing start, no_plan, intent_event or intent; two rows of one name; more
// rows than the fixed tables hold; `seconds` on a state the clock does not end or whose behaviour says how long it
// lasts itself; a changed_event that is not an event.
bool compileBrainTable(const BrainSpec& spec, const BehaviourSet& behaviours, BrainTable& out, std::string* err);

// The ways of travelling bound to the states their legs begin in (travel.lua `begins_in`). False, with `err` naming
// travel.lua and the row, when a row names no state of brain.lua, or there are more than kMaxWays rows.
bool bindWays(const TravelCatalog& travel, BrainTable& table, std::string* err);

}  // namespace brain
}  // namespace citysim

#endif
