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

// THE WALKS IN THE WALKING NETWORK (Glenn: students should cross the quad; and every park's paths): footpaths join the
// nav graph as walker-only links. A walker's route takes the diagonal across a block when it is shorter; a car's never
// does; nearestNode never answers with a walk node; a walk's end joins the street at its pavement, not its middle; a
// tee joins a walk to another's middle; a walk marked off-street (a quad door's) stays off the streets.
#include "../src/engine/ai/pathfind.h"
TEST_CASE(footpaths_are_walked_across_a_block_and_never_driven) {
    NavGraph nav = citytest::cityNav(600.0, 100.0, 4);
    const int c00 = nav.nearestNode(Vec2(0, 0)), c11 = nav.nearestNode(Vec2(100, 100));
    CHECK(c00 >= 0 && c11 >= 0 && c00 != c11);
    const Vec2 p00 = nav.nodes[static_cast<std::size_t>(c00)], p11 = nav.nodes[static_cast<std::size_t>(c11)];
    const Route before = findRoute(nav, c00, c11, true);
    Real walkBefore = 0;
    for (int li : before.links) walkBefore += nav.links[static_cast<std::size_t>(li)].length;
    const int streetLinks = nav.linkCount(), streetNodes = nav.nodeCount();
    const Vec2 d = normalize(p11 - p00);
    std::vector<std::array<Real, 5>> segs = {
        {p00.x + d.x * 8, p00.y + d.y * 8, p11.x - d.x * 8, p11.y - d.y * 8, 2.0},   // the diagonal, its ends short of the corners
        // a spur off its middle, toward a door (its far end must not be joined to a street)
        {(p00.x + p11.x) * 0.5 + 0.6, (p00.y + p11.y) * 0.5 - 0.6, (p00.x + p11.x) * 0.5 + 20.0, (p00.y + p11.y) * 0.5 - 20.0, -2.0}};
    const NavGraph::FootpathReport rep = nav.appendFootpaths(segs);
    std::printf("    [footpaths] %d nodes, %d links, %d tees, %d street joins; street walk %.0f m\n", rep.nodes, rep.links, rep.tees,
                rep.streetJoins, walkBefore);
    CHECK(rep.tees == 1);
    CHECK(rep.streetJoins == 2);   // the diagonal's two ends; not the spur's
    // a walker cuts across; the walk is shorter than the streets round the block
    const Route after = findRoute(nav, c00, c11, true);
    Real walkAfter = 0;
    bool usedPath = false;
    for (int li : after.links) {
        walkAfter += nav.links[static_cast<std::size_t>(li)].length;
        usedPath = usedPath || nav.links[static_cast<std::size_t>(li)].footpath;
    }
    CHECK(usedPath);
    CHECK(walkAfter < walkBefore * 0.85);
    // a car never takes a walk
    const Route car = findRoute(nav, c00, c11, false);
    CHECK(car.valid());
    for (int li : car.links) CHECK(!nav.links[static_cast<std::size_t>(li)].footpath);
    // and cannot get to a walk node at all
    CHECK(!findRoute(nav, c00, streetNodes, false).valid());
    // nearestNode keeps to the streets, even standing on the walk
    CHECK(nav.nearestNode((p00 + p11) * 0.5) < streetNodes);
    // the connector from the street starts on its pavement (a carriageway's half width + 1 m out), never in its middle
    for (int li = streetLinks; li < nav.linkCount(); ++li) {
        const NavLink& L = nav.links[static_cast<std::size_t>(li)];
        CHECK(L.footpath && L.walkable && (L.access & road_access::kFootpath));
        if (L.from < streetNodes) {
            const Real off = (L.footA - nav.nodes[static_cast<std::size_t>(L.from)]).length();
            CHECK(off > 1.0);
            CHECK((nav.sidewalkPoint(li, 0.0) - L.footA).length() < 1e-9);
        }
    }
}

// AN ACTIVITY GOAL TAKES ONLY WHAT IT ASKS FOR (behaviour plan, step 2): a goal asking for a SIT on the CAMPUS uses the
// campus's benches and never the town's, never a spot of another kind, and never one already held.
TEST_CASE(an_activity_goal_takes_only_spots_of_its_kind_and_tags) {
    GoalTable out;
    out.addState("Find", GoalAction::GoTo, GoalTarget::Activity, Activity::Outing);
    CHECK(out.setActivity("Find", spotKindBit(SpotKind::Sit), spot_tag::kCampus));
    out.addState("Pause", GoalAction::Rest, GoalTarget::None, Activity::Outing, 0.05);
    CHECK(out.addTransition("Find", GoalEvent::Arrived, "Pause"));
    CHECK(out.addTransition("Find", GoalEvent::NoRoute, "Pause"));
    CHECK(out.addTransition("Pause", GoalEvent::DwellDone, "Find"));
    CHECK(out.setEntry("Pause"));
    NavGraph nav = citytest::cityNav(600.0, 100.0, 4);
    CitySim sim;
    sim.build(nav, 0, 40, 21);
    sim.setGoalTables(out, out);
    PlaceMap places;
    for (int i = 0; i < 6; ++i) places.add(PlaceType::Home, Vec2(-250 + i * 100.0, -250), nav);
    const PlaceId quad = places.add(PlaceType::Park, Vec2(0, 0), nav);
    places.setCampus(quad, 4);
    std::vector<CitySim::ActivitySpot> spots;
    auto spot = [&](Vec2 p, SpotKind k) { CitySim::ActivitySpot s; s.pos = p; s.face = Vec2(0, 1); s.kind = k; spots.push_back(s); };
    for (int i = 0; i < 6; ++i) spot(Vec2(-15.0 + i * 6.0, 8.0), SpotKind::Sit);          // the quad's benches
    for (int i = 0; i < 3; ++i) spot(Vec2(-12.0 + i * 8.0, -8.0), SpotKind::Watch);       // ...and somewhere to watch
    for (int i = 0; i < 12; ++i) spot(Vec2(-250.0 + i * 40.0, 210.0), SpotKind::Sit);     // the town's benches, far off
    sim.setSpots(spots);
    sim.assignPlaces(places, nav);
    int campusSits = 0, wrong = 0, shared = 0;
    for (int i = 0; i < 40000; ++i) {
        sim.step(0.1, 0.25);
        std::vector<int> owner(sim.spots().size(), -1);
        for (std::size_t k = 0; k < sim.agents().size(); ++k) {
            const CitySim::ActivitySpot* st = sim.seatedOn(static_cast<int>(k));
            if (!st) continue;
            const std::size_t si = static_cast<std::size_t>(st - sim.spots().data());
            if (owner[si] >= 0) ++shared;
            owner[si] = static_cast<int>(k);
            // the city's day-off strollers run their own outing table (any bench); judge only this table's agents
            if (sim.agents()[k].role == Agent::Role::Stroller) continue;
            if (st->kind != SpotKind::Sit || !(st->tags & spot_tag::kCampus)) ++wrong;
            else ++campusSits;
        }
    }
    std::printf("    [activity] %d campus sit-samples, %d on the wrong spot, %d shared\n", campusSits, wrong, shared);
    CHECK(campusSits > 100);
    CHECK(wrong == 0);
    CHECK(shared == 0);
}

// JOGGERS (behaviour plan, step 3): an activity goal for a JOG spot walks onto the track, runs its loop at a jogger's
// pace -- on the loop's line, laps and all -- and walks off when the run is done; a runner is never drawn seated, and
// no start spot is ever two runners'.
TEST_CASE(joggers_run_laps_of_the_track_and_come_off_it) {
    GoalTable out;
    out.addState("Run", GoalAction::GoTo, GoalTarget::Activity, Activity::Outing);
    CHECK(out.setActivity("Run", spotKindBit(SpotKind::Jog), 0));
    out.addState("Rest", GoalAction::Rest, GoalTarget::None, Activity::Outing, 0.2);
    CHECK(out.addTransition("Run", GoalEvent::Arrived, "Rest"));
    CHECK(out.addTransition("Run", GoalEvent::NoRoute, "Rest"));
    CHECK(out.addTransition("Rest", GoalEvent::DwellDone, "Run"));
    CHECK(out.setEntry("Rest"));
    NavGraph nav = citytest::cityNav(600.0, 100.0, 4);
    CitySim sim;
    sim.build(nav, 0, 30, 21);
    sim.setGoalTables(out, out);
    // a track: a 40 x 30 m oval between the streets
    std::vector<Vec2> loop;
    for (int i = 0; i < 48; ++i) {
        const Real a = 6.2831853 * i / 48;
        loop.push_back(Vec2(50.0 + 20.0 * std::cos(a), 50.0 + 15.0 * std::sin(a)));
    }
    sim.setLoops({loop});
    std::vector<CitySim::ActivitySpot> spots;
    for (int k = 0; k < 8; ++k) {
        CitySim::ActivitySpot s;
        s.kind = SpotKind::Jog; s.loop = 0; s.loopS = sim.loopLength(0) * k / 8.0; s.hip = 0;
        Vec2 tg;
        s.pos = sim.loopPoint(0, s.loopS, &tg);
        s.face = tg;
        spots.push_back(s);
    }
    sim.setSpots(spots);
    std::printf("    [jog] %zu of 8 spots reachable\n", sim.spots().size());
    CHECK(sim.spots().size() == 8);
    int runSamples = 0, offLoop = 0, seatedRunners = 0, shared = 0, finished = 0;
    Real ran = 0, ranTime = 0;
    std::vector<Vec2> last(sim.agents().size());
    std::vector<uint8_t> was(sim.agents().size(), 0);
    const Real dt = 0.1;
    for (int i = 0; i < 30000; ++i) {
        sim.step(dt, 0.002);   // a run is ~0.45 h: ~13 min of this clock, laps and laps
        std::vector<int> owner(8, -1);
        for (std::size_t k = 0; k < sim.agents().size(); ++k) {
            const Agent& a = sim.agents()[k];
            const CitySim::ActivitySpot* sp = sim.usingSpot(static_cast<int>(k));
            if (a.seatPhase == 0 && was[k] == 3) ++finished;
            if (sp && sp->kind == SpotKind::Jog) {
                ++runSamples;
                const std::size_t si = static_cast<std::size_t>(sp - sim.spots().data());
                if (owner[si] >= 0) ++shared;
                owner[si] = static_cast<int>(k);
                if (sim.seatedOn(static_cast<int>(k))) ++seatedRunners;
                // on the loop's line
                Real best = 1e9;
                for (std::size_t j = 0; j < loop.size(); ++j) {
                    const Vec2 p = loop[j], q = loop[(j + 1) % loop.size()], d = q - p;
                    const Real t = std::max(Real(0), std::min(Real(1), dot(a.pos - p, d) / d.lengthSquared()));
                    best = std::min(best, (p + d * t - a.pos).length());
                }
                if (best > 0.05) ++offLoop;
                if (was[k] == 2) { ran += (a.pos - last[k]).length(); ranTime += dt; }
            }
            last[k] = a.pos;
            was[k] = a.seatPhase;
        }
    }
    const Real pace = ranTime > 0 ? ran / ranTime : 0;
    std::printf("    [jog] %d run-samples, %.0f m run (%.2f m/s), %d runs finished; off the loop %d, seated %d, shared %d\n",
                runSamples, ran, pace, finished, offLoop, seatedRunners, shared);
    CHECK(runSamples > 1000);
    CHECK(ran > sim.loopLength(0) * 3);        // laps, not a lap
    CHECK(pace > 2.4 && pace < 3.0);          // a jog (chords of the loop read a touch short of 2.8 m/s)
    CHECK(finished >= 3);
    CHECK(offLoop == 0);
    CHECK(seatedRunners == 0);
    CHECK(shared == 0);
}

// A CROWD ON A WALK (the island's stuck students: 32 of them held at the foot of a walk, speed 0): walkers whose
// commute runs over a footpath -- many leaving the same door at once -- get to work.
TEST_CASE(a_crowd_walks_a_footpath_to_work) {
    NavGraph nav = citytest::cityNav(600.0, 100.0, 4);
    const int c00 = nav.nearestNode(Vec2(0, 0)), c11 = nav.nearestNode(Vec2(100, 100));
    const Vec2 p00 = nav.nodes[static_cast<std::size_t>(c00)], p11 = nav.nodes[static_cast<std::size_t>(c11)];
    const Vec2 d = normalize(p11 - p00);
    nav.appendFootpaths({{p00.x + d.x * 8, p00.y + d.y * 8, p11.x - d.x * 8, p11.y - d.y * 8, 2.0}});
    CitySim sim;
    sim.build(nav, 0, 60, 21);
    PlaceMap places;
    const PlaceId home = places.add(PlaceType::Home, p00 + Vec2(-6, -6), nav);
    const PlaceId job = places.add(PlaceType::Office, p11 + Vec2(6, 6), nav, 8, 17);
    (void)home; (void)job;
    sim.assignPlaces(places, nav);
    sim.seedFromSchedule(7.9);
    int onWalk = 0, arrived = 0, maxStuck = 0;
    std::vector<int> stuck(sim.agents().size(), 0);
    std::vector<Real> lastD(sim.agents().size(), -1);
    for (int i = 0; i < 6000; ++i) {
        sim.step(0.1, 0.0005);
        for (std::size_t k = 0; k < sim.agents().size(); ++k) {
            const Agent& a = sim.agents()[k];
            if (!a.moving || a.leg >= static_cast<int>(a.route.links.size())) continue;
            const int li = a.route.links[static_cast<std::size_t>(a.leg)];
            if (!nav.links[static_cast<std::size_t>(li)].footpath) { stuck[k] = 0; continue; }
            ++onWalk;
            if (std::fabs(a.distOnLeg - lastD[k]) < 1e-6) ++stuck[k]; else stuck[k] = 0;
            lastD[k] = a.distOnLeg;
            maxStuck = std::max(maxStuck, stuck[k]);
        }
    }
    for (const Agent& a : sim.agents()) arrived += !a.moving && a.restNode == nav.nearestNode(places[job].entrance);
    std::printf("    [crowd] %d walker-steps on the walk, %d at work, longest stall %.1f s\n", onWalk, arrived, maxStuck * 0.1);
    CHECK(onWalk > 100);
    CHECK(maxStuck < 100);   // nobody held on a walk for 10 s
}
