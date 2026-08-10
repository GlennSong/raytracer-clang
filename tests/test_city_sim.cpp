#include "test_framework.h"

#include "city_test_util.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/apps/citysim/places.h"

#include <algorithm>
#include <cstdio>

using namespace engine;
using namespace citysim;

namespace {
// This file's grid town (city_test_util.h cityNav, historical dimensions/seed).
NavGraph cityNav() { return citytest::cityNav(200, 50, 7); }
}  // namespace

TEST_CASE(city_build_possession) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 20, 10, 1234);
    CHECK(sim.agents().size() == 30u);
    CHECK(sim.vehicles().size() == 20u);   // one car per driver, none for pedestrians

    int drivers = 0, peds = 0;
    for (const Agent& a : sim.agents()) {
        if (a.mode == Agent::Mode::Driver) {
            ++drivers;
            CHECK(a.vehicle >= 0);                       // a driver possesses a car
        } else {
            ++peds;
            CHECK(a.vehicle == -1);                      // a pedestrian has none
        }
    }
    CHECK(drivers == 20);
    CHECK(peds == 10);

    // Possession is two-way: each car's driver points back at an agent that owns it.
    for (std::size_t v = 0; v < sim.vehicles().size(); ++v) {
        int d = sim.vehicles()[v].driver;
        CHECK(d >= 0);
        CHECK(sim.agents()[d].vehicle == static_cast<int>(v));
    }
}

TEST_CASE(city_possessed_car_tracks_its_driver) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 25, 0, 99);
    bool anyMoved = false;
    for (int i = 0; i < 4000; ++i) sim.step(0.1, 0.3);
    for (const Agent& a : sim.agents()) {
        if (a.mode != Agent::Mode::Driver || a.vehicle < 0) continue;
        const SimVehicle& v = sim.vehicles()[a.vehicle];
        // The car mirrors the agent that drives it.
        CHECK_APPROX(v.pos.x, a.pos.x, 1e-9);
        CHECK_APPROX(v.pos.y, a.pos.y, 1e-9);
        if ((a.pos - nav.nodes[a.home]).length() > 5.0) anyMoved = true;
    }
    CHECK(anyMoved);   // at least one driver actually drove its car somewhere
}

TEST_CASE(city_player_agent_is_not_auto_driven) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 10, 0, 7);
    sim.setPlayerControlled(0, true);
    Vec2 start = sim.agents()[0].pos;
    bool othersMoved = false;
    for (int i = 0; i < 4000; ++i) sim.step(0.1, 0.3);
    // The player agent stays put (its brain is the host, not the sim).
    CHECK_APPROX(sim.agents()[0].pos.x, start.x, 1e-9);
    CHECK_APPROX(sim.agents()[0].pos.y, start.y, 1e-9);
    for (std::size_t i = 1; i < sim.agents().size(); ++i)
        if ((sim.agents()[i].pos - start).length() > 5.0) othersMoved = true;
    CHECK(othersMoved);   // the AI crowd still drives
}

TEST_CASE(city_drivers_commute) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 40, 0, 55);
    bool atWork = false;
    for (int i = 0; i < 6000 && !atWork; ++i) {
        sim.step(0.1, 0.3);
        for (const Agent& a : sim.agents())
            if (a.activity == Agent::Activity::AtWork) atWork = true;
    }
    CHECK(atWork);
}

TEST_CASE(city_is_deterministic) {
    NavGraph nav = cityNav();
    CitySim a, b;
    a.build(nav, 20, 20, 4242);
    b.build(nav, 20, 20, 4242);
    for (int i = 0; i < 2500; ++i) { a.step(0.1, 0.3); b.step(0.1, 0.3); }
    bool same = a.agents().size() == b.agents().size();
    for (std::size_t i = 0; i < a.agents().size() && same; ++i)
        if (a.agents()[i].pos.x != b.agents()[i].pos.x ||
            a.agents()[i].pos.y != b.agents()[i].pos.y)
            same = false;
    CHECK(same);
}

// SHIFT DIVERSITY (city_sim.cpp build): agents draw one of five departure
// archetypes from their own brain bits — early / standard / swing / night /
// part-time — so the city has hot and cold hours instead of one synchronised
// surge.
//
// This test exists because the feature silently did not work. The ladder read
// `a.brain` twenty-one lines BEFORE `a.brain` was seeded, so `shift` was 0 for
// every agent in the city, every agent took the first branch, and every agent
// departed inside the same 90-minute window. With the build clock starting at
// 8.5 — inside that window — the entire fleet left home on the first think and
// the opening minutes of play were one city-wide jam.
TEST_CASE(shift_archetypes_spread_the_departure_hours) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 1200, 800, 11);

    // Bucket by the archetype each departWork belongs to.
    int early = 0, standard = 0, swing = 0, night = 0, part = 0, other = 0;
    int commuters = 0;
    for (const Agent& a : sim.agents()) {
        if (a.home == a.work) continue;      // stranded: never departs, no window
        ++commuters;
        const Real d = a.departWork;
        if (d >= 5.0 && d < 6.5)        ++early;
        else if (d >= 7.0 && d < 9.0)   ++standard;
        else if (d >= 11.5 && d < 14.5) ++swing;
        else if (d >= 21.5 && d < 23.0) ++night;
        else if (d >= 9.5 && d < 11.5)  ++part;
        else                            ++other;
    }
    CHECK(commuters > 500);
    std::printf("    [shifts] early %d standard %d swing %d night %d part %d "
                "other %d (of %d)\n",
                early, standard, swing, night, part, other, commuters);

    // Every archetype must be represented — the bug left four of five empty.
    CHECK(early > 0);
    CHECK(standard > 0);
    CHECK(swing > 0);
    CHECK(night > 0);
    CHECK(part > 0);
    CHECK(other == 0);           // every agent lands in a known archetype

    // ...and no single hour may own the city. The bug put 100% in one bucket;
    // the design shares are 10/55/15/9/11, so the largest is ~55%.
    const int largest = std::max({early, standard, swing, night, part});
    CHECK(largest < commuters * 7 / 10);
}

// The midnight wrap. A night-shift agent leaves at ~21:30 and comes home at
// ~05:30, so its "at work" window runs THROUGH midnight. The naive
// `clock >= departWork && clock < departHome` test makes that window empty and
// the agent never goes to work at all — see inWindow() in city_sim.cpp.
TEST_CASE(night_shift_agents_have_a_wrapping_window) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 1200, 800, 11);

    int wrapping = 0;
    for (const Agent& a : sim.agents())
        if (a.home != a.work && a.departWork > a.departHome) ++wrapping;
    std::printf("    [shifts] %d agents work across midnight\n", wrapping);
    CHECK(wrapping > 0);   // the night archetype exists at all
}

// IS THERE ENOUGH DAY TO LIVE IN? Agents move at true metres per second while
// the clock runs at hoursPerSecond, so a journey costs
// (distance / speed) * hoursPerSecond * 3600 times more of the in-world day
// than it would in reality. At the historical default of 0.05 that is a 180x
// compression: on an 8 km city a 3 km commute eats 12.5 in-world hours — the
// agent arrives at work as its shift ends and can never get home, and a 50 m
// walk from a parking bay to a door costs nearly two hours.
//
// The failure is silent (the city just looks permanently stuck in rush hour),
// so this pins the relationship between the world's SIZE and its CLOCK.
TEST_CASE(a_commute_fits_inside_the_working_day) {
    NavGraph nav = citytest::cityNav(1200, 90, 5);   // a town ~1.2 km across
    CitySim sim;
    sim.build(nav, 300, 100, 13);

    // Places make assignPlaces (which measures the commute) meaningful.
    PlaceMap places;
    for (int i = 0; i < 12; ++i)
        places.add(PlaceType::Home, Vec2(-500 + i * 60.0, -400), nav);
    for (int i = 0; i < 6; ++i)
        places.add(PlaceType::Office, Vec2(-300 + i * 90.0, 400), nav, 9, 17);
    sim.assignPlaces(places, nav);

    const Real secs = sim.commuteSecondsMedian();
    CHECK(secs > 0);   // the measurement is actually wired up

    // At the historical 0.05 h/s this town's commute is already absurd; at a
    // scale-appropriate clock it is not. Both directions are asserted so the
    // test states the RELATIONSHIP rather than a magic constant.
    const Real fastHours = secs * 0.05;
    const Real saneHours = secs * 0.004;
    std::printf("    [clock] median commute %.0f s = %.2f h at 0.05, %.2f h at 0.004\n",
                secs, fastHours, saneHours);

    // A round trip plus a shift has to fit in a day: one leg must be a small
    // part of the ~8 h between departWork and departHome.
    CHECK(saneHours < 2.0);
    CHECK(fastHours > saneHours);   // faster clock = more of the day burned
}

// SLEEPING AGENTS. An NPC waiting out its day at home or at work has nothing to
// decide until its own schedule says otherwise, so it names the moment it next
// needs thinking about and drops out of the active list until then. It used to
// be asked the same question sixty times a second, and — since the sim resolves
// its active set once and every pass iterates it — that put a resting agent
// through a dozen sweeps per step to do nothing.
//
// Its CAR stays where it is, drawn and collidable. Cars are state; agents are
// the thing that can be recomputed.
TEST_CASE(resting_agents_sleep_until_their_next_activity) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 200, 100, 5);

    int peakSleeping = 0;
    long sleepingTicks = 0;
    for (int i = 0; i < 4000; ++i) {
        sim.step(0.1, 0.05);
        peakSleeping = std::max(peakSleeping, sim.sleepingAgents());
        sleepingTicks += sim.sleepingAgents();
    }
    const int population = static_cast<int>(sim.agents().size());
    std::printf("    [sleep] peak %d of %d agents asleep (mean %.0f)\n",
                peakSleeping, population, double(sleepingTicks) / 4000.0);
    // A real fraction of the city is idle at any moment — that is the whole
    // point. (Before the schedule was fixed, every agent commuted in the same
    // 90 minutes and almost nobody was ever resting at the same time.)
    CHECK(peakSleeping > population / 10);
    CHECK(peakSleeping <= population);
}

// A slept day must be the SAME day. The wake time is derived from the schedule,
// so an agent that sleeps through a stretch and wakes on its window must end up
// where one that was polled the whole time does.
TEST_CASE(sleeping_does_not_change_the_day) {
    NavGraph nav = cityNav();
    CitySim a;
    a.build(nav, 120, 60, 31);

    // Run a full in-world day and count what everyone is doing at the end.
    int atWork = 0, atHome = 0, commuting = 0;
    for (int i = 0; i < 6000; ++i) a.step(0.1, 0.05);
    for (const Agent& ag : a.agents()) {
        if (ag.activity == Agent::Activity::AtWork) ++atWork;
        else if (ag.activity == Agent::Activity::AtHome) ++atHome;
        else ++commuting;
    }
    std::printf("    [sleep] after a day: %d at work, %d at home, %d travelling\n",
                atWork, atHome, commuting);
    // The city is LIVED IN: agents reached work and came home, rather than
    // being stuck asleep past their own departure windows.
    CHECK(atWork + atHome > 0);
    CHECK(commuting < static_cast<int>(a.agents().size()));
    // Determinism survives the wake path.
    CitySim b;
    b.build(nav, 120, 60, 31);
    for (int i = 0; i < 6000; ++i) b.step(0.1, 0.05);
    bool same = true;
    for (std::size_t i = 0; i < a.agents().size() && same; ++i)
        if (a.agents()[i].pos.x != b.agents()[i].pos.x ||
            a.agents()[i].pos.y != b.agents()[i].pos.y ||
            a.agents()[i].activity != b.agents()[i].activity)
            same = false;
    CHECK(same);
}

// THE SNAPSHOT MUST AGREE WITH THE SIMULATION. scheduleSnapshot answers "where
// does this agent's day put it at hour T" without simulating anything, which is
// what lets a city be seeded at its start hour instead of stepped up to it. If
// it disagrees with what stepping actually produces, a seeded city is a
// different city.
//
// Asserted as a DISTRIBUTION rather than pose-for-pose: the snapshot places an
// agent from the population's median travel time, not its own route, so
// individuals legitimately differ while the shape of the day must not.
TEST_CASE(schedule_snapshot_agrees_with_a_stepped_day) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 400, 0, 17);

    // Step to a definite hour, then ask the snapshot about that same hour.
    const Real rate = 0.05;
    while (sim.timeOfDay() < 14.0) sim.step(0.1, rate);
    const Real clock = sim.timeOfDay();

    int simAtWork = 0, simAtHome = 0, simMoving = 0;
    int snapAtWork = 0, snapAtHome = 0, snapMoving = 0;
    for (const Agent& a : sim.agents()) {
        if (a.home == a.work) continue;   // never departs; not a schedule case
        if (a.moving) ++simMoving;
        else if (a.activity == Agent::Activity::AtWork) ++simAtWork;
        else ++simAtHome;

        const CitySim::Snapshot s = sim.scheduleSnapshot(a, clock);
        switch (s.where) {
            case CitySim::Snapshot::Where::AtWork: ++snapAtWork; break;
            case CitySim::Snapshot::Where::AtHome: ++snapAtHome; break;
            default: ++snapMoving; break;
        }
    }
    std::printf("    [snapshot] at %.1fh  sim: %d work / %d home / %d moving | "
                "snapshot: %d / %d / %d\n",
                clock, simAtWork, simAtHome, simMoving,
                snapAtWork, snapAtHome, snapMoving);
    // The same half of the day dominates in both. At 14:00 the standard and
    // early shifts are at work, so "at work" must lead in both accounts.
    CHECK(snapAtWork > 0);
    CHECK(simAtWork > 0);
    CHECK((snapAtWork > snapAtHome) == (simAtWork > simAtHome));
}

// It is a PURE function of the hour: same agent, same clock, same answer, and
// no dependence on what the sim happens to be doing.
TEST_CASE(schedule_snapshot_is_pure_and_covers_the_whole_day) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 200, 0, 23);
    const Agent& a = sim.agents()[0];

    for (Real h = 0; h < 24.0; h += 0.5) {
        const CitySim::Snapshot s1 = sim.scheduleSnapshot(a, h);
        const CitySim::Snapshot s2 = sim.scheduleSnapshot(a, h);
        CHECK(s1.where == s2.where);                  // pure
        CHECK(s1.elapsedSeconds == s2.elapsedSeconds);
    }
    // Over a full day a commuter must be found at BOTH ends — a schedule that
    // only ever reports one is the "never goes home" class of bug.
    int atWork = 0, atHome = 0;
    for (const Agent& ag : sim.agents()) {
        if (ag.home == ag.work) continue;
        bool sawWork = false, sawHome = false;
        for (Real h = 0; h < 24.0; h += 0.25) {
            const auto w = sim.scheduleSnapshot(ag, h).where;
            if (w == CitySim::Snapshot::Where::AtWork) sawWork = true;
            if (w == CitySim::Snapshot::Where::AtHome) sawHome = true;
        }
        if (sawWork) ++atWork;
        if (sawHome) ++atHome;
    }
    std::printf("    [snapshot] over a day: %d reach work, %d reach home\n",
                atWork, atHome);
    CHECK(atWork > 0);
    CHECK(atWork == atHome);   // everyone who works also gets home
}

// SEEDING REPLACES STEPPING. The host used to run 400 full-fidelity steps just
// to get the city moving before the level appeared — thousands of agents, 40
// sim-seconds, and it could only ever reach "a little after the build hour".
// seedFromSchedule places everyone directly from their own day, so any start
// hour costs the same.
//
// Asserted as a DISTRIBUTION, not pose-for-pose: seeding uses the population's
// median travel time rather than each agent's own route, so individuals differ
// while the shape of the day must not.
TEST_CASE(seeding_produces_a_lived_in_city_at_any_hour) {
    NavGraph nav = cityNav();

    auto census = [&](Real hour, int& atWork, int& atHome, int& moving) {
        CitySim sim;
        sim.build(nav, 300, 100, 29);
        sim.seedFromSchedule(hour);
        atWork = atHome = moving = 0;
        for (const Agent& a : sim.agents()) {
            if (a.home == a.work) continue;
            if (a.moving) ++moving;
            else if (a.activity == Agent::Activity::AtWork) ++atWork;
            else ++atHome;
        }
        CHECK_APPROX(sim.timeOfDay(), hour, 1e-9);   // the clock really moved
    };

    int dayWork = 0, dayHome = 0, dayMoving = 0;
    int nightWork = 0, nightHome = 0, nightMoving = 0;
    census(13.0, dayWork, dayHome, dayMoving);
    census(3.0, nightWork, nightHome, nightMoving);
    std::printf("    [seed] 13:00 -> %d work / %d home / %d moving\n",
                dayWork, dayHome, dayMoving);
    std::printf("    [seed] 03:00 -> %d work / %d home / %d moving\n",
                nightWork, nightHome, nightMoving);

    // Midday: most of the city is at work. Small hours: most is at home. If
    // these came out the same, the schedule would not be driving the seeding.
    CHECK(dayWork > dayHome);
    CHECK(nightHome > nightWork);
    // Both hours place SOMEBODY, and nobody is left in limbo.
    CHECK(dayWork + dayHome + dayMoving > 200);
    CHECK(nightWork + nightHome + nightMoving > 200);
}

// Seeding is deterministic and repeatable — it is the same code path a woken
// dormant agent will use, so an unstable seed would be an unstable world.
TEST_CASE(seeding_is_deterministic) {
    NavGraph nav = cityNav();
    CitySim a, b;
    a.build(nav, 200, 80, 77);
    b.build(nav, 200, 80, 77);
    a.seedFromSchedule(9.25);
    b.seedFromSchedule(9.25);
    bool same = true;
    for (std::size_t i = 0; i < a.agents().size() && same; ++i)
        if (a.agents()[i].pos.x != b.agents()[i].pos.x ||
            a.agents()[i].pos.y != b.agents()[i].pos.y ||
            a.agents()[i].activity != b.agents()[i].activity ||
            a.agents()[i].moving != b.agents()[i].moving)
            same = false;
    CHECK(same);

    // ...and the city keeps running normally from a seeded start.
    for (int i = 0; i < 600; ++i) a.step(0.1, 0.05);
    bool anyMoved = false;
    for (const Agent& ag : a.agents())
        if (ag.moving) anyMoved = true;
    CHECK(anyMoved);
}

// SEEDING MUST USE THE LEVEL'S OWN CLOCK RATE.
//
// scheduleSnapshot turns "a commute takes N seconds" into "a commute eats H
// hours of the day" by multiplying by hoursPerSecond. Seeding runs before any
// step(), so the sim has to be TOLD the rate — and when it was not, it used the
// 0.05 header default against a level authored at 0.004. That made the travel
// window 12.5x too wide: every agent whose last schedule boundary was under
// ~8 in-world hours back seeded as "still travelling", which is roughly three
// quarters of the population launched onto full-length routes the moment the
// level loaded. It showed up as a city permanently at 60% in transit.
TEST_CASE(seeding_respects_the_levels_clock_rate) {
    NavGraph nav = cityNav();

    auto travellingAt = [&](Real rate, Real hour) {
        CitySim sim;
        sim.build(nav, 600, 400, 19);
        sim.setHoursPerSecond(rate);
        sim.seedFromSchedule(hour);
        int moving = 0;
        for (const Agent& a : sim.agents())
            if (a.moving) ++moving;
        return moving;
    };

    const int population = 1000;
    const int slow = travellingAt(0.004, 10.5);   // a big world's clock
    const int fast = travellingAt(0.05, 10.5);    // the old default
    std::printf("    [seed] travelling at 0.004: %d/%d | at 0.05: %d/%d\n",
                slow, population, fast, population);

    // At a scale-appropriate clock only agents within one commute of a
    // boundary are under way — a small minority, not the city.
    CHECK(slow < population / 4);
    // ...and the rate genuinely matters, so this cannot pass by accident.
    CHECK(fast > slow);
}
