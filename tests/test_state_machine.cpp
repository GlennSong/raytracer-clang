#include "test_framework.h"

#include "../src/engine/state_machine.h"

#include <string>
#include <vector>

using namespace engine;

// The generic data-driven FSM: one mechanism for AI goals, animation states,
// and UI transitions. These pin the contract the callers will lean on: tables
// are data (buildable from any source), events with no row are ignored,
// wildcard transitions interrupt from anywhere, guards veto, hook order is
// exit -> action -> enter, and everything is deterministic (insertion order).

TEST_CASE(state_machine_walks_a_pedestrian_day) {
    // The citysim goal loop as a TABLE — the shape the Lua layer will feed in.
    StateMachine m;
    int home = m.addState("AtHome");
    int commuting = m.addState("Commuting");
    int work = m.addState("AtWork");
    int leave = m.addEvent("depart");
    int arrive = m.addEvent("arrive");
    m.addTransition(home, leave, commuting);
    m.addTransition(commuting, arrive, work);
    m.addTransition(work, leave, commuting);
    m.addTransition(commuting, arrive, home);   // dead row: first match wins

    m.start(home);
    CHECK(m.current() == home);
    CHECK(m.handle(leave));
    CHECK(m.current() == commuting);
    CHECK(m.handle(arrive));
    CHECK(m.current() == work);           // the first matching row won
    CHECK(!m.handle(arrive));             // no row from AtWork on `arrive`: ignored
    CHECK(m.current() == work);
    CHECK(m.stateName(m.current()) == "AtWork");
}

TEST_CASE(state_machine_names_are_idempotent_and_findable) {
    StateMachine m;
    int a = m.addState("Grazing");
    CHECK(m.addState("Grazing") == a);    // re-adding returns the same id
    CHECK(m.findState("Grazing") == a);
    CHECK(m.findState("Fleeing") == -1);
    int e = m.addEvent("startled");
    CHECK(m.addEvent("startled") == e);
    CHECK(m.findEvent("startled") == e);
    CHECK(m.stateCount() == 1);
    CHECK(m.eventCount() == 1);
}

TEST_CASE(state_machine_wildcard_interrupts_from_any_state) {
    // A knockdown must preempt whatever the agent was doing — the kAny row.
    StateMachine m;
    int walk = m.addState("Walking");
    int shop = m.addState("Shopping");
    int down = m.addState("KnockedDown");
    int hit = m.addEvent("hit");
    m.addTransition(StateMachine::kAny, hit, down);

    m.start(walk);
    CHECK(m.handle(hit));
    CHECK(m.current() == down);
    m.start(shop);
    CHECK(m.handle(hit));
    CHECK(m.current() == down);
}

TEST_CASE(state_machine_guards_veto_and_order_decides) {
    // Two rows for the same (from, event): the first whose guard passes fires.
    StateMachine m;
    int idle = m.addState("Idle");
    int fight = m.addState("Fight");
    int flee = m.addState("Flee");
    int threat = m.addEvent("threat");
    bool brave = false;
    m.addTransition(idle, threat, fight, {[&] { return brave; }, {}});
    m.addTransition(idle, threat, flee);   // unguarded fallback

    m.start(idle);
    CHECK(m.handle(threat));
    CHECK(m.current() == flee);            // guard vetoed the fight row
    m.start(idle);
    brave = true;
    CHECK(m.handle(threat));
    CHECK(m.current() == fight);           // guard passed: first row wins
}

TEST_CASE(state_machine_peek_is_stateless) {
    // One shared table, many agents: peek answers "where would this event take
    // state X" without running hooks or moving current() — each agent keeps its
    // own current-state int (the citysim goal layer).
    StateMachine m;
    int home = m.addState("AtHome");
    int commuting = m.addState("Commuting");
    int leave = m.addEvent("depart");
    bool open = false;
    int fired = 0;
    m.addTransition(home, leave, commuting,
                    {[&] { return open; }, [&] { ++fired; }});

    m.start(home);
    CHECK(m.peek(home, leave) == -1);          // guard vetoes
    open = true;
    CHECK(m.peek(home, leave) == commuting);   // row found...
    CHECK(m.current() == home);                // ...but nothing moved
    CHECK(fired == 0);                         // ...and no action ran
    CHECK(m.peek(commuting, leave) == -1);     // no row from here
}

TEST_CASE(state_machine_hook_order_is_exit_action_enter) {
    StateMachine m;
    int a = m.addState("A");
    int b = m.addState("B");
    int go = m.addEvent("go");
    std::vector<std::string> log;
    m.onEnter(a, [&] { log.push_back("enter A"); });
    m.onExit(a, [&] { log.push_back("exit A"); });
    m.onEnter(b, [&] { log.push_back("enter B"); });
    m.addTransition(a, go, b, {{}, [&] { log.push_back("action"); }});

    m.start(a);
    CHECK(m.handle(go));
    CHECK(log.size() == 4u);
    CHECK(log[0] == "enter A");
    CHECK(log[1] == "exit A");
    CHECK(log[2] == "action");
    CHECK(log[3] == "enter B");

    // An explicit self-loop re-fires the hooks (restart an animation).
    int again = m.addEvent("again");
    m.addTransition(b, again, b);
    log.clear();
    CHECK(m.handle(again));
    CHECK(log.size() == 1u);   // B has no exit hook; enter B re-fired
    CHECK(log[0] == "enter B");
}
