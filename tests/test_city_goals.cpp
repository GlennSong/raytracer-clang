#include "test_framework.h"

#include "city_test_util.h"
#include "../src/apps/citysim/city_goals.h"
#include "../src/apps/citysim/city_sim.h"

#include <vector>

using namespace engine;
using namespace citysim;

// The goal layer as DATA (ADR-0064): an agent's day is a GoalTable — states
// binding the small C++ action vocabulary (Rest, GoTo work/home/random) with
// transitions on the events CitySim emits (clock windows, arrival, no-route,
// idle, dwell). These pin the table contract, that the built-in default IS
// today's behaviour (installing it explicitly changes nothing, bit for bit),
// and that a new behaviour — "go somewhere random and dwell two hours" — is
// pure data, no C++ changes.

TEST_CASE(goal_default_table_mirrors_the_legacy_schedule) {
    GoalTable t = defaultScheduleGoals();
    int home = t.findState("AtHome");
    int commute = t.findState("CommuteToWork");
    int work = t.findState("AtWork");
    int ret = t.findState("ReturnHome");
    CHECK(t.stateCount() == 4);
    CHECK(t.entry() == home);
    CHECK(t.state(home).action == GoalAction::Rest);
    CHECK(t.state(commute).action == GoalAction::GoTo);
    CHECK(t.state(commute).target == GoalTarget::Work);
    CHECK(t.state(ret).target == GoalTarget::Home);
    // The day's loop, plus the no-route fallbacks to the origin's rest state.
    CHECK(t.onEvent(home, GoalEvent::DepartWork) == commute);
    CHECK(t.onEvent(home, GoalEvent::DepartHome) == -1);   // already home
    CHECK(t.onEvent(commute, GoalEvent::Arrived) == work);
    CHECK(t.onEvent(commute, GoalEvent::NoRoute) == home);
    CHECK(t.onEvent(work, GoalEvent::DepartHome) == ret);
    CHECK(t.onEvent(ret, GoalEvent::Arrived) == home);
    CHECK(t.onEvent(ret, GoalEvent::NoRoute) == work);
    // Labels the tests/renderers read (Agent::activity) come from the states.
    CHECK(t.state(home).activity == Activity::AtHome);
    CHECK(t.state(commute).activity == Activity::Commuting);
    CHECK(t.state(work).activity == Activity::AtWork);
    CHECK(t.state(ret).activity == Activity::Returning);
}

TEST_CASE(goal_wander_tables_chain_drivers_and_rest_walkers) {
    GoalTable d = wanderGoals(true);
    int roam = d.findState("Roam");
    CHECK(d.entry() == roam);
    CHECK(d.state(roam).target == GoalTarget::Random);
    // A driver chains: arrival lands in another GoTo state (the self-loop).
    CHECK(d.onEvent(roam, GoalEvent::Arrived) == roam);

    GoalTable p = wanderGoals(false);
    int proam = p.findState("Roam");
    int rest = p.findState("RoamRest");
    CHECK(rest >= 0);
    // A walker rests one tick at the kerb, then Idle relaunches the loop.
    CHECK(p.onEvent(proam, GoalEvent::Arrived) == rest);
    CHECK(p.state(rest).action == GoalAction::Rest);
    CHECK(p.onEvent(rest, GoalEvent::Idle) == proam);
}

TEST_CASE(goal_event_names_round_trip) {
    for (int i = 0; i < static_cast<int>(GoalEvent::Count); ++i) {
        GoalEvent e = static_cast<GoalEvent>(i);
        bool ok = false;
        CHECK(goalEventFromName(goalEventName(e), &ok) == e);
        CHECK(ok);
    }
    bool ok = true;
    goalEventFromName("noSuchEvent", &ok);
    CHECK(!ok);
}

TEST_CASE(installing_the_default_tables_changes_nothing) {
    // The seam's parity pin: a sim running the built-in tables and one handed
    // the same tables through setGoalTables must be BIT-IDENTICAL — same rng
    // draws, same tick every transition fires, same poses and labels.
    NavGraph nav = citytest::cross4(50.0);
    CitySim a, b;
    a.build(nav, 16, 8, 17);
    b.build(nav, 16, 8, 17);
    b.setGoalTables(defaultScheduleGoals(), defaultScheduleGoals());
    bool same = true;
    for (int i = 0; i < 6000; ++i) {
        a.step(0.1, 0.5);
        b.step(0.1, 0.5);
    }
    for (std::size_t k = 0; k < a.agents().size(); ++k) {
        if (a.agents()[k].pos.x != b.agents()[k].pos.x ||
            a.agents()[k].pos.y != b.agents()[k].pos.y ||
            a.agents()[k].activity != b.agents()[k].activity ||
            a.agents()[k].trips != b.agents()[k].trips)
            same = false;
    }
    CHECK(same);
    CHECK(a.faults() == b.faults());
}

TEST_CASE(custom_goal_table_runs_errands_with_a_two_hour_dwell) {
    // "Go to a random node and dwell 2 h" — the behaviour the vocabulary must
    // express as pure data: a GoTo(Random) state chained to a Rest state whose
    // dwell clock emits DwellDone after 2 in-world hours.
    GoalTable errands;
    errands.addState("Errand", GoalAction::GoTo, GoalTarget::Random,
                     Activity::Commuting);
    errands.addState("Browse", GoalAction::Rest, GoalTarget::None,
                     Activity::AtWork, /*dwellHours=*/2.0);
    CHECK(errands.addTransition("Errand", GoalEvent::Arrived, "Browse"));
    CHECK(errands.addTransition("Browse", GoalEvent::DwellDone, "Errand"));
    CHECK(errands.setEntry("Errand"));
    CHECK(!errands.addTransition("Errand", GoalEvent::Arrived, "NoSuchState"));

    NavGraph nav = citytest::cross4(40.0);
    CitySim sim;
    sim.build(nav, 1, 1, 9);
    sim.setGoalTables(errands, errands);

    // Track each agent's rest bouts: every completed stay must last ~2 h of
    // clock (>= the dwell, < it plus a generous launch-wait allowance), and
    // trips must keep chaining through the day.
    const Real hoursPerSecond = 0.5, dt = 0.1;
    std::vector<bool> wasResting(sim.agents().size(), false);
    std::vector<Real> restStart(sim.agents().size(), 0.0);
    std::vector<int> restBouts(sim.agents().size(), 0);
    Real hours = 0;
    bool dwellOk = true;
    for (int i = 0; i < 40000; ++i) {
        sim.step(dt, hoursPerSecond);
        hours += dt * hoursPerSecond;   // unwrapped clock (timeOfDay wraps at 24)
        const auto& ag = sim.agents();
        for (std::size_t k = 0; k < ag.size(); ++k) {
            bool resting = ag[k].activity == Activity::AtWork;   // Browse's label
            if (resting && !wasResting[k]) restStart[k] = hours;
            if (!resting && wasResting[k]) {
                Real stay = hours - restStart[k];
                // ~2 h against the dwell clock (a tick of float slack under,
                // a generous launch-wait allowance over).
                if (stay < 2.0 - 0.06 || stay > 3.0) dwellOk = false;
                ++restBouts[k];
            }
            wasResting[k] = resting;
        }
    }
    CHECK(dwellOk);
    for (std::size_t k = 0; k < sim.agents().size(); ++k) {
        CHECK(restBouts[k] >= 2);              // the loop really cycles
        CHECK(sim.agents()[k].trips >= 3);     // ...trip after trip
    }
}
