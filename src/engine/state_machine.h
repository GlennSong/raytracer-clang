#ifndef RAYTRACER_ENGINE_STATE_MACHINE_H
#define RAYTRACER_ENGINE_STATE_MACHINE_H

#include <functional>
#include <string>
#include <vector>

namespace engine {

// A generic, DATA-DRIVEN finite state machine — one mechanism for AI goals,
// animation states, and UI transitions, so each domain stops baking its own
// switch statements into control flow. The machine is a TABLE: named states,
// named events, and (from, event) -> to transitions, buildable at runtime from
// any source — C++ literals, level JSON, or Lua — which is what lets a
// pedestrian's day or an animal's graze/startle loop become data instead of
// code. Guards and enter/exit hooks are optional std::functions; a table with
// no hooks is pure data and trivially serializable.
//
// Deliberately NOT here: timers, utility scores, hysteresis. Cadence lives
// with the caller (the sim's think clock), and utility choice with hysteresis
// is Commitment (ADR-0063) — compose them, don't merge them.
//
// Deterministic: transitions fire in the order they were added; the first
// whose guard passes wins. No hidden clocks, no randomness.
class StateMachine {
public:
    static constexpr int kAny = -1;   // wildcard `from`: fires from every state
                                      // (a knockdown interrupts anything)

    // --- building the table -------------------------------------------------
    // Adding an existing name returns its id (idempotent), so tables can be
    // assembled from several sources without coordination.
    int addState(const std::string& name) { return internName(states_, name); }
    int addEvent(const std::string& name) { return internName(events_, name); }

    struct Hooks {
        std::function<bool()> guard;    // may this transition fire? (empty = yes)
        std::function<void()> action;   // runs after exit(from), before enter(to)
    };
    void addTransition(int from, int event, int to, Hooks hooks = {}) {
        transitions_.push_back({from, event, to, std::move(hooks)});
    }

    // Optional per-state enter/exit hooks (animation starts, sound cues).
    void onEnter(int state, std::function<void()> fn) { hook(enter_, state, std::move(fn)); }
    void onExit(int state, std::function<void()> fn) { hook(exit_, state, std::move(fn)); }

    // --- running -------------------------------------------------------------
    // Set the initial state (fires its enter hook). Also the reset.
    void start(int state) {
        current_ = state;
        if (state >= 0 && state < static_cast<int>(enter_.size()) && enter_[state])
            enter_[state]();
    }

    // Deliver an event. The first matching transition (in insertion order, exact
    // `from` and kAny both match) whose guard passes fires: exit(from), action,
    // enter(to). Returns true if a transition fired. A transition to the SAME
    // state re-fires its hooks (an explicit self-loop restarts an animation);
    // an event with no matching row is simply ignored.
    bool handle(int event) {
        for (const Row& r : transitions_) {
            if (r.event != event) continue;
            if (r.from != kAny && r.from != current_) continue;
            if (r.hooks.guard && !r.hooks.guard()) continue;
            if (current_ >= 0 && current_ < static_cast<int>(exit_.size()) && exit_[current_])
                exit_[current_]();
            if (r.hooks.action) r.hooks.action();
            current_ = r.to;
            if (r.to >= 0 && r.to < static_cast<int>(enter_.size()) && enter_[r.to])
                enter_[r.to]();
            return true;
        }
        return false;
    }

    int current() const { return current_; }
    const std::string& stateName(int id) const { return states_[id]; }
    int stateCount() const { return static_cast<int>(states_.size()); }
    int eventCount() const { return static_cast<int>(events_.size()); }
    // Lookup by name (tooling / Lua): -1 when absent.
    int findState(const std::string& name) const { return find(states_, name); }
    int findEvent(const std::string& name) const { return find(events_, name); }

private:
    struct Row {
        int from, event, to;
        Hooks hooks;
    };

    static int internName(std::vector<std::string>& v, const std::string& name) {
        for (int i = 0; i < static_cast<int>(v.size()); ++i)
            if (v[i] == name) return i;
        v.push_back(name);
        return static_cast<int>(v.size()) - 1;
    }
    static int find(const std::vector<std::string>& v, const std::string& name) {
        for (int i = 0; i < static_cast<int>(v.size()); ++i)
            if (v[i] == name) return i;
        return -1;
    }
    void hook(std::vector<std::function<void()>>& v, int state, std::function<void()> fn) {
        if (state < 0) return;
        if (state >= static_cast<int>(v.size())) v.resize(state + 1);
        v[state] = std::move(fn);
    }

    std::vector<std::string> states_;
    std::vector<std::string> events_;
    std::vector<Row> transitions_;
    std::vector<std::function<void()>> enter_, exit_;
    int current_ = -1;
};

}  // namespace engine

#endif
