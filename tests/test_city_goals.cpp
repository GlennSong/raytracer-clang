#include "test_framework.h"

#include "city_test_util.h"
#include "../src/apps/citysim/city_goals.h"
#include "../src/apps/citysim/city_sim.h"

#include <cstdio>
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

// The day is home -> work -> ERRAND -> home (Glenn, 2026-09-17: "go to work
// / work till 5pm / go shopping / go home"). It no longer mirrors the
// pre-table control flow, which had no errand leg.
TEST_CASE(goal_default_table_is_the_daily_round) {
    GoalTable t = defaultScheduleGoals();
    int home = t.findState("AtHome");
    int commute = t.findState("CommuteToWork");
    int work = t.findState("AtWork");
    int ret = t.findState("ReturnHome");
    CHECK(t.stateCount() == 10);
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
    const int shopGo = t.findState("GoShopping");
    const int shopAt = t.findState("AtShop");
    CHECK(t.onEvent(work, GoalEvent::DepartHome) == shopGo);
    CHECK(t.onEvent(shopGo, GoalEvent::Arrived) == shopAt);
    CHECK(t.onEvent(shopGo, GoalEvent::NoRoute) == ret);
    CHECK(t.onEvent(shopAt, GoalEvent::DwellDone) == ret);
    CHECK(t.onEvent(ret, GoalEvent::Arrived) == home);
    // Lunch OUT, about four hours into the shift; no lunch (brought one, car
    // commuter, nothing open) carries straight on into the afternoon.
    const int lunchGo = t.findState("GoLunch");
    const int lunchAt = t.findState("AtLunch");
    const int back = t.findState("BackToWork");
    const int pm = t.findState("AtWorkPM");
    CHECK(t.state(work).dwellHours > 3.0);
    CHECK(t.onEvent(work, GoalEvent::DwellDone) == lunchGo);
    CHECK(t.state(lunchGo).target == GoalTarget::Lunch);
    CHECK(t.onEvent(lunchGo, GoalEvent::Arrived) == lunchAt);
    CHECK(t.onEvent(lunchGo, GoalEvent::NoRoute) == pm);
    CHECK(t.onEvent(lunchAt, GoalEvent::DwellDone) == back);
    CHECK(t.onEvent(back, GoalEvent::Arrived) == pm);
    CHECK(t.onEvent(pm, GoalEvent::DepartHome) == shopGo);
    CHECK(t.onEvent(ret, GoalEvent::NoRoute) == pm);
    // Labels the tests/renderers read (Agent::activity) come from the states.
    CHECK(t.state(home).activity == Activity::AtHome);
    CHECK(t.state(commute).activity == Activity::Commuting);
    CHECK(t.state(work).activity == Activity::AtWork);
    CHECK(t.state(ret).activity == Activity::Returning);
}

TEST_CASE(goal_stroller_table_is_an_outing) {
    // A day OUT: from home, stop after stop until the window closes.
    GoalTable t = strollerGoals();
    const int home = t.findState("AtHome");
    const int go = t.findState("Outing");
    const int pause = t.findState("OutAndAbout");
    const int ret = t.findState("ReturnHome");
    CHECK(t.entry() == home);
    CHECK(t.state(go).target == GoalTarget::Outing);
    CHECK(t.onEvent(home, GoalEvent::DepartWork) == go);
    CHECK(t.onEvent(go, GoalEvent::Arrived) == pause);
    CHECK(t.onEvent(pause, GoalEvent::DwellDone) == go);       // the next stop
    CHECK(t.onEvent(pause, GoalEvent::DepartHome) == ret);     // the window closed
    CHECK(t.onEvent(ret, GoalEvent::Arrived) == home);
    CHECK(t.state(pause).activity == Activity::Outing);
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

// PEOPLE SIT ON THE BENCHES (the furniture library, M5): an outing may be a sit -- a free seat reserved, the walk
// off the path to it, a sit of a few minutes facing the way the seat faces, the walk back -- and a seat is only
// ever one person's.
TEST_CASE(outings_sometimes_sit_on_a_bench_and_get_up_again) {
    GoalTable out;
    out.addState("Out", GoalAction::GoTo, GoalTarget::Outing, Activity::Outing);
    out.addState("Pause", GoalAction::Rest, GoalTarget::None, Activity::Outing, 0.05);
    CHECK(out.addTransition("Out", GoalEvent::Arrived, "Pause"));
    CHECK(out.addTransition("Out", GoalEvent::NoRoute, "Pause"));
    CHECK(out.addTransition("Pause", GoalEvent::DwellDone, "Out"));
    CHECK(out.setEntry("Pause"));
    NavGraph nav = citytest::cityNav(600.0, 100.0, 4);
    CitySim sim;
    sim.build(nav, 0, 12, 21);
    sim.setGoalTables(out, out);
    std::vector<CitySim::SeatSpot> seats;
    for (int i = 0; i < nav.nodeCount(); i += 3) {
        CitySim::SeatSpot s;
        s.pos = nav.nodes[static_cast<std::size_t>(i)] + Vec2(7.0, 4.0);
        s.face = Vec2(0, 1);
        s.hip = 0.47;
        seats.push_back(s);
    }
    sim.setSeats(seats);
    CHECK(!sim.seats().empty());
    int sat = 0, satDone = 0, shared = 0, off = 0;
    std::vector<uint8_t> was(sim.agents().size(), 0);
    for (int i = 0; i < 60000; ++i) {
        sim.step(0.1, 0.25);
        const auto& ag = sim.agents();
        std::vector<int> owner(sim.seats().size(), -1);
        for (std::size_t k = 0; k < ag.size(); ++k) {
            const CitySim::SeatSpot* st = sim.seatedOn(static_cast<int>(k));
            if (st) {
                if ((ag[k].pos - st->pos).length() > 1e-6) ++off;
                const std::size_t si = static_cast<std::size_t>(st - sim.seats().data());
                if (owner[si] >= 0) ++shared;
                owner[si] = static_cast<int>(k);
            }
            const uint8_t now = ag[k].seatPhase;
            if (now == 2 && was[k] != 2) ++sat;
            if (now == 0 && was[k] == 3) ++satDone;
            was[k] = now;
        }
    }
    std::printf("    [seats] %zu seats; %d sits begun, %d finished; shared %d, off-seat %d\n", sim.seats().size(), sat, satDone,
                shared, off);
    CHECK(sat >= 5);
    CHECK(satDone >= 3);
    CHECK(shared == 0);
    CHECK(off == 0);
    for (const CitySim::SeatSpot& s : sim.seats()) {   // every seat held is held by someone sitting there or on the way
        if (s.occupant < 0) continue;
        CHECK(sim.agents()[static_cast<std::size_t>(s.occupant)].tripSeat == static_cast<int>(&s - sim.seats().data()));
    }
}

// CAMPUS LIFE (campus milestone 4): a city with a university houses students in its residence hall -- as many as
// it has beds -- and runs their day on the student table: to class in the teaching hall, a break (the library, a
// bench on the quad, lunch), back to class, and home to the hall in the evening.
TEST_CASE(students_live_in_the_hall_and_spend_the_day_on_campus) {
    NavGraph nav = citytest::cityNav(800.0, 80.0, 5);
    CitySim sim;
    sim.build(nav, 20, 400, 21);
    PlaceMap places;
    for (int i = 0; i < 10; ++i) places.add(PlaceType::Home, Vec2(-350 + i * 70.0, -330), nav);
    for (int i = 0; i < 4; ++i) places.add(PlaceType::Office, Vec2(-300 + i * 160.0, 330), nav, 9, 17);
    places.add(PlaceType::Cafe, Vec2(120, -40), nav, 7, 19);
    const PlaceId hall = places.add(PlaceType::Home, Vec2(-120, 60), nav, 0, 24, 60);
    const PlaceId teach = places.add(PlaceType::Civic, Vec2(60, 120), nav);
    const PlaceId lib = places.add(PlaceType::Civic, Vec2(-40, 200), nav);
    const PlaceId quad = places.add(PlaceType::Park, Vec2(0, 130), nav);
    places.setCampus(hall, 3); places.setCampus(teach, 1); places.setCampus(lib, 2); places.setCampus(quad, 4);
    std::vector<CitySim::SeatSpot> seats;
    for (int i = 0; i < 8; ++i) {   // benches round the quad
        CitySim::SeatSpot s;
        s.pos = places[quad].site + Vec2(-21.0 + i * 6.0, (i % 2) ? 12.0 : -12.0);
        s.face = Vec2(0, (i % 2) ? -1.0 : 1.0);
        s.hip = 0.47;
        seats.push_back(s);
    }
    sim.setSeats(seats);
    sim.assignPlaces(places, nav);
    sim.seedFromSchedule(6.0);

    CHECK(sim.studentCount() == 60);   // the hall's beds (fewer than a quarter of the 400 walkers)
    const int hallNode = nav.nearestNode(places[hall].entrance);
    int homeless = 0, othersInHall = 0;
    for (const Agent& a : sim.agents()) {
        if (a.role == Agent::Role::Student) homeless += a.home != hallNode || a.homePlace != hall;
        else othersInHall += a.homePlace == hall;
    }
    CHECK(homeless == 0);
    CHECK(othersInHall == 0);   // the hall is the students' alone

    int inClass = 0, onBench = 0, inLibrary = 0, backHome = 0, peakOut = 0;
    std::vector<uint8_t> sawClass(sim.agents().size(), 0), sawBreak(sim.agents().size(), 0);
    const int teachNode = nav.nearestNode(places[teach].entrance), libNode = nav.nearestNode(places[lib].entrance);
    for (int i = 0; i < 30000 && sim.timeOfDay() < 22.5; ++i) {
        sim.step(0.5, 0.002);
        const auto& ag = sim.agents();
        int out = 0;
        for (std::size_t k = 0; k < ag.size(); ++k) {
            const Agent& a = ag[k];
            if (a.role != Agent::Role::Student) continue;
            if (a.indoors && a.restNode == teachNode && a.activity == Activity::AtWork) sawClass[k] = 1;
            if (sim.seatedOn(static_cast<int>(k))) { sawBreak[k] |= 1; }
            if (a.indoors && a.restNode == libNode) sawBreak[k] |= 2;
            out += a.moving ? 1 : 0;
        }
        peakOut = std::max(peakOut, out);
    }
    for (std::size_t k = 0; k < sim.agents().size(); ++k) {
        const Agent& a = sim.agents()[k];
        if (a.role != Agent::Role::Student) continue;
        inClass += sawClass[k];
        onBench += (sawBreak[k] & 1) ? 1 : 0;
        inLibrary += (sawBreak[k] & 2) ? 1 : 0;
        backHome += a.indoors && a.restNode == hallNode;
    }
    std::printf("    [students] %d students: %d went to class, %d sat on the quad, %d studied in the library, %d home by "
                "%.1f h; at most %d walking at once\n",
                sim.studentCount(), inClass, onBench, inLibrary, backHome, sim.timeOfDay(), peakOut);
    CHECK(inClass >= 50);
    CHECK(onBench >= 5);
    CHECK(inLibrary >= 10);
    CHECK(backHome >= 50);
}

// THE DORM BLOCK (campus gap pass): with two residence halls the students fill both, in proportion to their beds, and
// their courses are spread over every teaching hall.
TEST_CASE(students_fill_every_hall_and_take_classes_in_every_teaching_hall) {
    NavGraph nav = citytest::cityNav(800.0, 80.0, 5);
    CitySim sim;
    sim.build(nav, 20, 400, 21);
    PlaceMap places;
    for (int i = 0; i < 10; ++i) places.add(PlaceType::Home, Vec2(-350 + i * 70.0, -330), nav);
    places.add(PlaceType::Office, Vec2(0, 330), nav, 9, 17);
    const PlaceId h1 = places.add(PlaceType::Home, Vec2(-120, 60), nav, 0, 24, 30);
    const PlaceId h2 = places.add(PlaceType::Home, Vec2(200, -150), nav, 0, 24, 20);
    const PlaceId t1 = places.add(PlaceType::Civic, Vec2(60, 120), nav);
    const PlaceId t2 = places.add(PlaceType::Civic, Vec2(-60, 200), nav);
    places.setCampus(h1, 3); places.setCampus(h2, 3); places.setCampus(t1, 1); places.setCampus(t2, 1);
    sim.assignPlaces(places, nav);
    int in1 = 0, in2 = 0, at1 = 0, at2 = 0;
    for (const Agent& a : sim.agents()) {
        if (a.role != Agent::Role::Student) continue;
        in1 += a.homePlace == h1; in2 += a.homePlace == h2;
        at1 += a.workPlace == t1; at2 += a.workPlace == t2;
    }
    std::printf("    [halls] %d students: %d + %d in the halls, %d + %d in the teaching halls\n", sim.studentCount(), in1, in2,
                at1, at2);
    CHECK(sim.studentCount() == 50);
    CHECK(in1 + in2 == 50);
    CHECK(std::abs(in1 - 30) <= 1 && std::abs(in2 - 20) <= 1);
    CHECK(at1 >= 15 && at2 >= 15);
}
