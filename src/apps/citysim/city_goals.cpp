#include "city_goals.h"

#include <cstdio>

namespace citysim {

namespace {
const char* kEventNames[] = {"departWork", "departHome", "arrived",
                             "noRoute",    "idle",       "dwellDone"};
static_assert(sizeof(kEventNames) / sizeof(kEventNames[0]) ==
                  static_cast<std::size_t>(GoalEvent::Count),
              "goal event names out of sync with GoalEvent");

const char* actionName(GoalAction a) {
    return a == GoalAction::GoTo ? "goto" : "rest";
}

const char* targetName(GoalTarget t) {
    switch (t) {
        case GoalTarget::Work: return "work";
        case GoalTarget::Home: return "home";
        case GoalTarget::Random: return "random";
        default: return "none";
    }
}

const char* activityName(Activity a) {
    switch (a) {
        case Activity::Commuting: return "Commuting";
        case Activity::AtWork: return "AtWork";
        case Activity::Returning: return "Returning";
        default: return "AtHome";
    }
}
}  // namespace

const char* goalEventName(GoalEvent e) {
    return kEventNames[static_cast<std::size_t>(e)];
}

GoalEvent goalEventFromName(const std::string& name, bool* ok) {
    for (std::size_t i = 0; i < static_cast<std::size_t>(GoalEvent::Count); ++i) {
        if (name == kEventNames[i]) {
            if (ok) *ok = true;
            return static_cast<GoalEvent>(i);
        }
    }
    if (ok) *ok = false;
    return GoalEvent::Count;
}

int GoalTable::addState(const std::string& name, GoalAction action,
                        GoalTarget target, Activity activity, double dwellHours) {
    int id = machine_.addState(name);
    if (id >= static_cast<int>(states_.size())) states_.resize(id + 1);
    states_[id] = {name, action, target, activity, dwellHours};
    return id;
}

bool GoalTable::addTransition(const std::string& from, GoalEvent event,
                              const std::string& to) {
    int f = machine_.findState(from);
    int t = machine_.findState(to);
    if (f < 0 || t < 0 || event >= GoalEvent::Count) return false;
    int& ev = events_[static_cast<std::size_t>(event)];
    if (ev < 0) ev = machine_.addEvent(goalEventName(event));
    machine_.addTransition(f, ev, t);
    rows_.push_back({f, event, t});
    return true;
}

bool GoalTable::setEntry(const std::string& name) {
    int id = machine_.findState(name);
    if (id < 0) return false;
    entry_ = id;
    return true;
}

int GoalTable::onEvent(int state, GoalEvent event) const {
    if (state < 0 || state >= stateCount() || event >= GoalEvent::Count) return -1;
    int ev = events_[static_cast<std::size_t>(event)];
    if (ev < 0) return -1;   // no row ever used this event
    return machine_.peek(state, ev);
}

std::string GoalTable::describe() const {
    std::string out;
    if (entry_ >= 0 && entry_ < stateCount())
        out += "entry " + states_[entry_].name + "\n";
    for (const GoalState& s : states_) {
        out += "state " + s.name + " " + actionName(s.action);
        if (s.action == GoalAction::GoTo)
            out += std::string(" ") + targetName(s.target);
        out += std::string(" activity=") + activityName(s.activity);
        if (s.dwellHours > 0) {
            char buf[32];
            std::snprintf(buf, sizeof(buf), " dwell=%g", s.dwellHours);
            out += buf;
        }
        out += "\n";
    }
    for (const Row& r : rows_)
        out += states_[r.from].name + " " + goalEventName(r.event) + " -> " +
               states_[r.to].name + "\n";
    return out;
}

GoalTable defaultScheduleGoals() {
    GoalTable t;
    t.addState("AtHome", GoalAction::Rest, GoalTarget::None, Activity::AtHome);
    t.addState("CommuteToWork", GoalAction::GoTo, GoalTarget::Work,
               Activity::Commuting);
    t.addState("AtWork", GoalAction::Rest, GoalTarget::None, Activity::AtWork);
    t.addState("ReturnHome", GoalAction::GoTo, GoalTarget::Home,
               Activity::Returning);
    t.addTransition("AtHome", GoalEvent::DepartWork, "CommuteToWork");
    t.addTransition("CommuteToWork", GoalEvent::Arrived, "AtWork");
    t.addTransition("CommuteToWork", GoalEvent::NoRoute, "AtHome");
    t.addTransition("AtWork", GoalEvent::DepartHome, "ReturnHome");
    t.addTransition("ReturnHome", GoalEvent::Arrived, "AtHome");
    t.addTransition("ReturnHome", GoalEvent::NoRoute, "AtWork");
    t.setEntry("AtHome");
    return t;
}

GoalTable wanderGoals(bool driver) {
    GoalTable t;
    t.addState("Roam", GoalAction::GoTo, GoalTarget::Random, Activity::Commuting);
    if (driver) {
        // Arrival CHAINS straight into the next trip (a GoTo destination on
        // Arrived keeps the car rolling through the node — see arriveOrChain).
        t.addTransition("Roam", GoalEvent::Arrived, "Roam");
    } else {
        // A walker turns around AT the kerb: rest for one tick (the historical
        // AtWork label), then Idle relaunches the loop next tick.
        t.addState("RoamRest", GoalAction::Rest, GoalTarget::None,
                   Activity::AtWork);
        t.addTransition("Roam", GoalEvent::Arrived, "RoamRest");
        t.addTransition("RoamRest", GoalEvent::Idle, "Roam");
    }
    t.setEntry("Roam");
    return t;
}

}  // namespace citysim
