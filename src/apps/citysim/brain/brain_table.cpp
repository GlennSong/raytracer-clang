#include "brain_table.h"

#include "../trip_plan.h"

#include <cmath>

namespace citysim {
namespace brain {

namespace {

struct VerbName {
    const char* name;
    Verb verb;
};
// The verbs the data may write (a state may not be named like one).
const VerbName kVerbs[] = {
    {"refuse", Verb::Refuse},         {"ignore", Verb::Ignore},
    {"next_leg", Verb::NextLeg},      {"replan", Verb::Replan},
    {"replan_without", Verb::ReplanWithout}, {"resume", Verb::Resume},
    {"redirect", Verb::Redirect},     {"replan_afresh", Verb::ReplanAfresh},
};

bool verbOf(const std::string& word, Verb& out) {
    for (const VerbName& v : kVerbs)
        if (word == v.name) { out = v.verb; return true; }
    return false;
}

std::string verbList() {
    std::string s;
    for (const VerbName& v : kVerbs) s += (s.empty() ? "" : ", ") + std::string(v.name);
    return s;
}

template <typename Row>
int indexByName(const std::vector<Row>& rows, const std::string& name) {
    for (std::size_t i = 0; i < rows.size(); ++i)
        if (rows[i].name == name) return static_cast<int>(i);
    return -1;
}

BrainTime millisOf(Real minutes) { return static_cast<BrainTime>(std::lround(minutes * 60.0 * kMillisPerSecond)); }

}  // namespace

void BehaviourSet::add(const std::string& state, const StateFns& fns, const std::vector<std::string>& raises) {
    Behaviour b;
    b.state = state;
    b.fns = fns;
    b.raises = raises;
    rows.push_back(b);
}

void BehaviourSet::addNotBuilt(const std::string& state) {
    StateFns fns;
    fns.built = false;
    add(state, fns);
}

void BehaviourSet::addSkill(const std::string& name) { skills.push_back(name); }

int BrainTable::state(const std::string& name) const { return indexByName(states, name); }
int BrainTable::intent(const std::string& name) const { return indexByName(intents, name); }
int BrainTable::event(const std::string& name) const {
    for (std::size_t i = 0; i < events.size(); ++i)
        if (events[i] == name) return static_cast<int>(i);
    return -1;
}

bool compileBrainTable(const BrainSpec& spec, const BehaviourSet& behaviours, BrainTable& out, std::string* err) {
    auto refuse = [&](const std::string& key, const std::string& why) {
        if (err) *err = spec.file + ": " + key + " " + why;
        return false;
    };
    BrainTable t;
    t.file = spec.file;
    if (spec.states.size() > static_cast<std::size_t>(kMaxStates))
        return refuse("states", "lists " + std::to_string(spec.states.size()) + " rows; the table holds " +
                                    std::to_string(kMaxStates));
    if (spec.events.size() > static_cast<std::size_t>(kMaxEvents))
        return refuse("events", "lists " + std::to_string(spec.events.size()) + " rows; the table holds " +
                                    std::to_string(kMaxEvents));
    if (spec.intents.size() > static_cast<std::size_t>(kMaxIntents))
        return refuse("intents", "lists " + std::to_string(spec.intents.size()) + " rows; the table holds " +
                                     std::to_string(kMaxIntents));

    // THE EVENTS, by name
    for (const EventSpec& e : spec.events) {
        if (e.name.empty()) return refuse("events", "has a row with no name");
        if (t.event(e.name) >= 0) return refuse("events." + e.name, "is listed twice");
        t.events.push_back(e.name);
    }
    // THE STATES, each with the functions registered under its name
    for (const StateSpec& s : spec.states) {
        if (s.name.empty()) return refuse("states", "has a row with no name");
        const std::string key = "states." + s.name;
        Verb verb;
        if (verbOf(s.name, verb)) return refuse(key, "is named like a verb (" + verbList() + ")");
        if (t.state(s.name) >= 0) return refuse(key, "is listed twice");
        StateRow row;
        row.name = s.name;
        row.label = s.label.empty() ? s.name : s.label;
        row.group = s.group;
        row.skill = s.skill;
        row.holds = s.holds;
        row.seen = s.seen;
        const Behaviour* found = nullptr;
        for (const Behaviour& b : behaviours.rows)
            if (b.state == s.name) { found = &b; break; }
        if (!found)
            return refuse(key, "has no behaviour registered in C++ (register its functions, or register it as not built "
                               "yet)");
        row.fns = found->fns;
        if (found->raises.size() > static_cast<std::size_t>(kMaxRaises))
            return refuse(key, "raises " + std::to_string(found->raises.size()) + " events itself; a state holds " +
                                   std::to_string(kMaxRaises));
        for (std::size_t r = 0; r < found->raises.size(); ++r) {
            const int e = t.event(found->raises[r]);
            if (e < 0)
                return refuse("events", "has no `" + found->raises[r] + "`, which the behaviour of `" + s.name +
                                            "` raises");
            row.raises[r] = static_cast<EventId>(e);
        }
        bool skillKnown = s.skill == "none";
        for (const std::string& k : behaviours.skills) skillKnown = skillKnown || k == s.skill;
        if (!skillKnown) return refuse(key + ".skill", "names `" + s.skill + "`, which is not a skill C++ has");
        if (s.ends == "never") row.ends = Ends::Never;
        else if (s.ends == "skill") row.ends = Ends::Skill;
        else if (s.ends == "clock") row.ends = Ends::Clock;
        else return refuse(key + ".ends", "is `" + s.ends + "`; it must be never, skill or clock");
        if (row.ends == Ends::Never) {
            if (!s.clock.empty()) return refuse(key + ".clock", "is set, but the state never ends by the clock (ends = never)");
        } else {
            const int e = t.event(s.clock);
            if (e < 0) return refuse(key + ".clock", "names `" + s.clock + "`, which is not an event");
            row.clock = static_cast<EventId>(e);
        }
        if (s.where == "place") row.where = Where::Place;
        else if (s.where == "leg_start") row.where = Where::LegStart;
        else if (s.where == "along_leg") row.where = Where::AlongLeg;
        else if (s.where == "leg_end") row.where = Where::LegEnd;
        else return refuse(key + ".where", "is `" + s.where + "`; it must be place, leg_start, along_leg or leg_end");
        if (s.seconds < 0 || !std::isfinite(s.seconds)) return refuse(key + ".seconds", "must be 0 or more");
        if (s.seconds > 0) {
            if (row.ends != Ends::Clock)
                return refuse(key + ".seconds", "is set, but the clock does not end the state (ends = clock)");
            if (row.fns.lasts)
                return refuse(key + ".seconds", "is set, but the behaviour of `" + s.name + "` says how long it lasts itself");
            row.fixedMillis = static_cast<BrainTime>(std::lround(s.seconds * kMillisPerSecond));
        }
        t.states.push_back(row);
    }
    // ...and no functions without a state
    for (const Behaviour& b : behaviours.rows)
        if (t.state(b.state) < 0)
            return refuse("states", "has no row `" + b.state + "`, which C++ registered a behaviour for");

    // What a word on the right of a row means: a verb, or a state.
    auto answerOf = [&](const std::string& word, Answer& a) {
        if (verbOf(word, a.verb)) return true;
        const int s = t.state(word);
        if (s < 0) return false;
        a.verb = Verb::Go;
        a.state = static_cast<StateId>(s);
        return true;
    };
    const std::string neither = ", which is neither a state nor a verb (" + verbList() + ")";
    // EVERY STATE'S ANSWER TO EVERY EVENT: the row's own, or else the event's `otherwise`. A state that holds answers
    // only what its row lists, so nothing moves a held person on by default.
    std::vector<Answer> otherwise(spec.events.size());
    for (std::size_t e = 0; e < spec.events.size(); ++e) {
        const EventSpec& ev = spec.events[e];
        if (!answerOf(ev.otherwise, otherwise[e]))
            return refuse("events." + ev.name + ".otherwise", "names `" + ev.otherwise + "`" + neither);
        if (otherwise[e].verb == Verb::Resume)
            return refuse("events." + ev.name + ".otherwise", "is resume, which only a row of a state that holds may say");
        if (otherwise[e].verb == Verb::Redirect)
            return refuse("events." + ev.name + ".otherwise",
                          "is redirect, which only a row of a state that can be sent elsewhere may say");
    }
    for (std::size_t s = 0; s < spec.states.size(); ++s) {
        const StateSpec& st = spec.states[s];
        for (std::size_t e = 0; e < spec.events.size(); ++e) {
            Answer a = otherwise[e];
            if (st.holds) a = Answer{Verb::Ignore, kNoId};
            t.on[s][e] = a;
        }
        for (const std::pair<std::string, std::string>& row : st.on) {
            const std::string key = "states." + st.name + ".on." + row.first;
            const int e = t.event(row.first);
            if (e < 0) return refuse(key, "is not an event");
            Answer a;
            if (!answerOf(row.second, a)) return refuse(key, "names `" + row.second + "`" + neither);
            if (a.verb == Verb::Resume && !st.holds)
                return refuse(key, "is resume, but `" + st.name + "` does not hold (holds = true)");
            if (a.verb == Verb::Redirect && !t.states[s].fns.retarget)
                return refuse(key, "is redirect, but the behaviour of `" + st.name + "` cannot be sent elsewhere (it has no "
                                   "retarget function)");
            t.on[s][static_cast<std::size_t>(e)] = a;
        }
    }
    // THE NAMED ROLES
    auto stateNamed = [&](const std::string& key, const std::string& name, StateId& into) {
        const int s = t.state(name);
        if (s < 0) return refuse(key, name.empty() ? "is missing" : "names `" + name + "`, which is not a state");
        into = static_cast<StateId>(s);
        return true;
    };
    if (!stateNamed("start", spec.start, t.start)) return false;
    if (!stateNamed("no_plan", spec.noPlan, t.noPlan)) return false;
    const int intentEvent = t.event(spec.intentEvent);
    if (intentEvent < 0)
        return refuse("intent_event", spec.intentEvent.empty() ? "is missing"
                                                              : "names `" + spec.intentEvent + "`, which is not an event");
    t.intentEvent = static_cast<EventId>(intentEvent);
    if (!spec.changedEvent.empty()) {
        const int changed = t.event(spec.changedEvent);
        if (changed < 0) return refuse("changed_event", "names `" + spec.changedEvent + "`, which is not an event");
        t.changedEvent = static_cast<EventId>(changed);
    }
    if (spec.intents.empty()) return refuse("intents", "is empty");
    for (const IntentSpec& i : spec.intents) {
        if (i.name.empty()) return refuse("intents", "has a row with no name");
        if (t.intent(i.name) >= 0) return refuse("intents." + i.name, "is listed twice");
        IntentRow row;
        row.name = i.name;
        if (!stateNamed("intents." + i.name + ".arrive", i.arrive, row.arrive)) return false;
        t.intents.push_back(row);
    }
    out = t;
    return true;
}

bool bindWays(const TravelCatalog& travel, BrainTable& table, std::string* err) {
    auto refuse = [&](const std::string& why) {
        if (err) *err = "travel.lua: " + why;
        return false;
    };
    if (travel.modes.size() > static_cast<std::size_t>(kMaxWays))
        return refuse("modes lists " + std::to_string(travel.modes.size()) + " rows; a person's record holds " +
                      std::to_string(kMaxWays));
    std::vector<WayRow> ways;
    for (const TravelMode& m : travel.modes) {
        WayRow w;
        w.name = m.name;
        const int s = table.state(m.beginsIn);
        if (s < 0)
            return refuse("modes `" + m.name + "`: begins_in " +
                          (m.beginsIn.empty() ? std::string("is missing")
                                              : "names `" + m.beginsIn + "`, which is not a state of " + table.file));
        w.begins = static_cast<StateId>(s);
        w.startMillis = millisOf(m.startMinutes);
        w.endMillis = millisOf(m.endMinutes);
        w.setOffWaitMillis = millisOf(m.setOffWaitMinutes);
        ways.push_back(w);
    }
    table.ways = ways;
    return true;
}

}  // namespace brain
}  // namespace citysim
