#include "city_goals.h"

#include <cstdio>

namespace citysim {

namespace {
const char* kEventNames[] = {"departWork", "departHome", "arrived",
                             "noRoute",    "idle",       "dwellDone",
                             "gotFare",    "serviceEnd", "serviceStart",
                             "evening",    "eveningOver"};
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
        case GoalTarget::Shop: return "shop";
        case GoalTarget::Fare: return "fare";
        case GoalTarget::Drop: return "drop";
        case GoalTarget::Stop: return "stop";
        case GoalTarget::Depot: return "depot";
        case GoalTarget::Lunch: return "lunch";
        case GoalTarget::Outing: return "outing";
        case GoalTarget::Campus: return "campus";
        case GoalTarget::Activity: return "activity";
        default: return "none";
    }
}

const char* activityName(Activity a) {
    switch (a) {
        case Activity::Commuting: return "Commuting";
        case Activity::AtWork: return "AtWork";
        case Activity::Returning: return "Returning";
        case Activity::Shopping: return "Shopping";
        case Activity::Outing: return "Outing";
        case Activity::Lunch: return "Lunch";
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

bool GoalTable::setMenu(const std::string& name, const std::string& menu) {
    const int id = machine_.findState(name);
    if (id < 0 || id >= static_cast<int>(states_.size())) return false;
    states_[static_cast<std::size_t>(id)].menu = menu;
    return true;
}

bool GoalTable::setActivity(const std::string& name, uint32_t spotKinds, uint32_t spotTags) {
    const int id = machine_.findState(name);
    if (id < 0 || id >= static_cast<int>(states_.size())) return false;
    states_[static_cast<std::size_t>(id)].spotKinds = spotKinds;
    states_[static_cast<std::size_t>(id)].spotTags = spotTags;
    return true;
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
        if (s.target == GoalTarget::Activity) {
            char buf[48];
            std::snprintf(buf, sizeof(buf), " spots=%x/%x", s.spotKinds, s.spotTags);
            out += buf;
            if (!s.menu.empty()) out += " menu=" + s.menu;
        }
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

GoalTable busGoals() {
    GoalTable t;
    // ONE state. A bus has no decisions to make: it drives to the next stop,
    // for ever. Arriving CHAINS into the next leg (arriveOrChain), and the
    // stop index advances there too, so the self-loop is the whole timetable.
    t.addState("Drive", GoalAction::GoTo, GoalTarget::Stop, Activity::Commuting);
    t.addTransition("Drive", GoalEvent::Arrived, "Drive");
    // A leg it cannot route is skipped, not fatal: the stop index has already
    // moved on, so the bus simply tries the one after it.
    t.addTransition("Drive", GoalEvent::NoRoute, "Drive");

    // OUT OF SERVICE. At the end of the service day the bus stops taking
    // passengers and drives to the yard, then sits there until service resumes.
    // Deadheading is a REAL state, not an absence of one: a bus crossing town
    // empty with its doors shut is doing its job, and a viewer can tell it from
    // one that is simply stuck.
    t.addState("ToDepot", GoalAction::GoTo, GoalTarget::Depot, Activity::Returning);
    t.addState("OffDuty", GoalAction::Rest, GoalTarget::None, Activity::AtHome);
    t.addTransition("Drive", GoalEvent::ServiceEnd, "ToDepot");
    t.addTransition("ToDepot", GoalEvent::Arrived, "OffDuty");
    // A yard it cannot reach must not strand the bus mid-road for the night.
    t.addTransition("ToDepot", GoalEvent::NoRoute, "OffDuty");
    t.addTransition("OffDuty", GoalEvent::ServiceStart, "Drive");
    // ...and one still on its way in when the morning service starts turns back to its route
    t.addTransition("ToDepot", GoalEvent::ServiceStart, "Drive");
    t.setEntry("Drive");
    return t;
}

GoalTable taxiGoals() {
    GoalTable t;
    // Cruise: a wandering car, exactly like wanderGoals' Roam -- Arrived chains
    // straight into the next leg so the cab keeps rolling through junctions.
    t.addState("Cruise", GoalAction::GoTo, GoalTarget::Random, Activity::Commuting);
    t.addState("ToPickup", GoalAction::GoTo, GoalTarget::Fare, Activity::Commuting);
    t.addState("ToDrop", GoalAction::GoTo, GoalTarget::Drop, Activity::Commuting);
    t.addTransition("Cruise", GoalEvent::Arrived, "Cruise");
    // The dispatch matched us: break off the cruise and go and collect.
    t.addTransition("Cruise", GoalEvent::GotFare, "ToPickup");
    // Arriving AT the pickup boards the passenger (CitySim::arriveOrChain does
    // the boarding, then the table moves the cab on to the destination).
    t.addTransition("ToPickup", GoalEvent::Arrived, "ToDrop");
    // Arriving at the destination sets them down; back to cruising for another.
    t.addTransition("ToDrop", GoalEvent::Arrived, "Cruise");
    // A fare we cannot route to is not a fare: drop it and keep cruising,
    // rather than a cab frozen forever on an unreachable pickup.
    t.addTransition("ToPickup", GoalEvent::NoRoute, "Cruise");
    t.addTransition("ToDrop", GoalEvent::NoRoute, "Cruise");
    t.addTransition("Cruise", GoalEvent::NoRoute, "Cruise");
    t.setEntry("Cruise");
    return t;
}

}  // namespace citysim
