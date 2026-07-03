#ifndef RAYTRACER_APPS_CITYSIM_CITY_GOALS_H
#define RAYTRACER_APPS_CITYSIM_CITY_GOALS_H

#include "../../engine/state_machine.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace citysim {

// The per-agent GOAL layer as DATA (ADR-0064). What an agent does with its day
// — AtHome -> commute -> AtWork -> return, wander's perpetual trips, a future
// shopper's errand loop or an animal's graze/flee — is a TABLE over the generic
// engine::StateMachine: named states that each bind one small C++ ACTION (with
// params), and transitions on the fixed events CitySim emits. The table is
// plain data, buildable from C++ literals or (in scripting builds) from an
// `agents.lua` asset at LOAD time; every per-tick transition then executes in
// C++ — no Lua in the tick, and the Makefile test build stays Lua-free.

// The label an agent's day reads as from outside — what tests and the debug
// widgets consume (the historical Agent::Activity). Each goal STATE carries
// the value it shows, so the legacy field stays in sync with the table.
enum class Activity : uint8_t { AtHome, Commuting, AtWork, Returning };

// The C++ action vocabulary a goal state wires together. Deliberately tiny:
// behaviours come from how the TABLE composes these, not from new actions.
enum class GoalAction : uint8_t {
    Rest,   // stay put (park on the verge / stand at the kerb) until an event
    GoTo,   // travel to `target`; while at rest, the departure is retried each
            // tick (launch-clearance gated for drivers) until the trip launches
};

// Where a GoTo state travels: the agent's work node, its home node, or a
// random routable node drawn from the sim rng (wander's goal pick).
enum class GoalTarget : uint8_t { None, Work, Home, Random };

// The events CitySim EMITS at fixed points — the full trigger vocabulary a
// table may transition on. Emission lives in C++ (clock windows, arrival,
// route failure), so tables stay declarative.
enum class GoalEvent : uint8_t {
    DepartWork,   // resting, and the clock sits inside [departWork, departHome)
    DepartHome,   // resting, and the clock sits outside that window
    Arrived,      // a trip just completed (fires at the arrival tick)
    NoRoute,      // a departure/chain found no route to its goal
    Idle,         // resting, every tick — the hook for perpetual behaviours
    DwellDone,    // resting, and `dwellHours` of clock time passed in this state
    Count
};
const char* goalEventName(GoalEvent e);
// Name -> event for table readers (the Lua side); `ok` reports an unknown name.
GoalEvent goalEventFromName(const std::string& name, bool* ok = nullptr);

// One goal state: an action id plus its params. `activity` is the legacy label
// mirrored into Agent::activity whenever the agent occupies (Rest) or launches
// (GoTo) this state; `dwellHours` (Rest only) emits DwellDone after that much
// in-world clock in the state — "go to a random node and dwell 2 h" is a
// GoTo(Random) state plus a Rest state with dwellHours = 2.
struct GoalState {
    std::string name;
    GoalAction action = GoalAction::Rest;
    GoalTarget target = GoalTarget::None;
    Activity activity = Activity::AtHome;
    double dwellHours = 0;   // 0 = rest forever (only events can end it)
};

// The table itself: states + transitions over an engine::StateMachine, shared
// by every agent of an archetype — each agent holds only a current-state int
// (Agent::goal) and the machine answers transitions statelessly via peek().
class GoalTable {
public:
    GoalTable() { events_.fill(-1); }

    // Adding an existing name overwrites its params and returns its id.
    int addState(const std::string& name, GoalAction action,
                 GoalTarget target = GoalTarget::None,
                 Activity activity = Activity::AtHome, double dwellHours = 0);
    // False (and no row) when either state name is unknown.
    bool addTransition(const std::string& from, GoalEvent event,
                       const std::string& to);
    bool setEntry(const std::string& name);   // false when unknown

    int entry() const { return entry_; }
    int stateCount() const { return static_cast<int>(states_.size()); }
    const GoalState& state(int id) const { return states_[id]; }
    int findState(const std::string& name) const { return machine_.findState(name); }
    // The state `event` moves `state` to (first matching row wins); -1 = none.
    int onEvent(int state, GoalEvent event) const;

    // Canonical text dump — entry, states, transitions in insertion order.
    // Two tables that describe identically behave identically; the scripting
    // test compares the Lua-built tables against the C++ defaults with this.
    std::string describe() const;

private:
    struct Row {
        int from;
        GoalEvent event;
        int to;
    };
    engine::StateMachine machine_;
    std::vector<GoalState> states_;
    std::vector<Row> rows_;   // mirror of machine_'s rows, for describe()
    std::array<int, static_cast<std::size_t>(GoalEvent::Count)> events_;
    int entry_ = 0;
};

// The built-in defaults — bit-exact mirrors of the pre-table control flow.
// The daily schedule: AtHome -> CommuteToWork -> AtWork -> ReturnHome -> ...
GoalTable defaultScheduleGoals();
// Wander (the agent lab): one perpetual GoTo(Random) loop. A driver CHAINS
// straight through the arrival node (Arrived self-loop onto a GoTo state); a
// walker rests one tick at the kerb first (Arrived -> RoamRest -Idle-> Roam).
GoalTable wanderGoals(bool driver);

}  // namespace citysim

#endif
