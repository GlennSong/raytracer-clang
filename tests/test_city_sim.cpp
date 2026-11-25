#include "test_framework.h"

#include "model_scene.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/apps/citysim/places.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace engine;
using namespace citysim;

namespace {
// This file's grid town (city_test_util.h gridCity, its historical size and seed) as a road file, built as a level's roads
// are and walked on the street map made from its road model (model_scene.h, #306 piece 6 W2).
const citytest::ModelScene& town() { return citytest::gridScene(200, 50, 7); }
}  // namespace

// THE DRIVING (#234): a sim that holds the driving library ticks it once per sim tick, and its people do exactly what
// they do in a sim without it.
TEST_CASE(city_sim_driving_ticks_alongside_the_people) {
    const citytest::ModelScene& scene = town();
    CitySim v1, v2;
    scene.build(v1, 35, 99);
    scene.build(v2, 35, 99);
    v2.useDrivingV2(true);
    CHECK(v1.drivingV2() == nullptr);
    CHECK(v2.drivingV2() != nullptr);
    const int steps = 2000;
    const Real dt = 0.1;
    for (int i = 0; i < steps; ++i) {
        v1.step(dt, 0.3);
        v2.step(dt, 0.3);
    }
    const driving::Driving* d = v2.drivingV2();
    CHECK(d != nullptr);
    if (d) {
        CHECK(d->tickCount() == static_cast<uint64_t>(steps));   // the default rate ticks every step
        CHECK_APPROX(d->clockSeconds(), steps * dt, 1e-6);
        CHECK(d->tripCount() == 0u);                            // an empty world: no cars handed over yet
    }
    int differ = 0;
    CHECK(v1.agents().size() == v2.agents().size());
    for (std::size_t i = 0; i < v1.agents().size() && i < v2.agents().size(); ++i) {
        const Agent& a = v1.agents()[i];
        const Agent& b = v2.agents()[i];
        if (a.pos.x != b.pos.x || a.pos.y != b.pos.y || a.speed != b.speed || a.state != b.state) ++differ;
    }
    CHECK(differ == 0);
}

TEST_CASE(city_player_agent_is_not_auto_driven) {
    const citytest::ModelScene& scene = town();
    CitySim sim;
    scene.build(sim,10, 7);
    sim.setPlayerControlled(0, true);
    Vec2 start = sim.agents()[0].pos;
    std::vector<Vec2> starts;
    for (const Agent& ag : sim.agents()) starts.push_back(ag.pos);
    bool othersMoved = false;
    for (int i = 0; i < 4000; ++i) {
        citytest::keepEveryoneWalking(sim, i / 600);   // (no day walks them: ADR-0499; the player is skipped)
        sim.step(0.1, 0.3);
    }
    // The player agent stays put (its brain is the host, not the sim).
    CHECK_APPROX(sim.agents()[0].pos.x, start.x, 1e-9);
    CHECK_APPROX(sim.agents()[0].pos.y, start.y, 1e-9);
    for (std::size_t i = 1; i < sim.agents().size(); ++i)
        if ((sim.agents()[i].pos - starts[i]).length() > 5.0) othersMoved = true;
    CHECK(othersMoved);   // the AI crowd still moves
}

TEST_CASE(city_is_deterministic) {
    const citytest::ModelScene& scene = town();
    CitySim a, b;
    scene.build(a,40, 4242);
    scene.build(b,40, 4242);
    for (int i = 0; i < 2500; ++i) {
        citytest::keepEveryoneWalking(a, i / 600);   // (no day walks them: ADR-0499)
        citytest::keepEveryoneWalking(b, i / 600);
        a.step(0.1, 0.3);
        b.step(0.1, 0.3);
    }
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
    const citytest::ModelScene& scene = town();
    CitySim sim;
    scene.build(sim,2000, 11);

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
    const citytest::ModelScene& scene = town();
    CitySim sim;
    scene.build(sim,2000, 11);

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
// so this pins the relationship between the world's SIZE and its CLOCK. (On foot since ADR-0462: the old driving's
// commuters drove a town 1.2 km across; a walker walks one 600 m across, its homes and offices at half the spacing.
// The 1.2 km walk measured 656 s, 2.6 h at 0.004.)
TEST_CASE(a_commute_fits_inside_the_working_day) {
    const citytest::ModelScene& scene = citytest::gridScene(600, 90, 5);   // a town ~600 m across
    const engine::RoadModel* nav = scene.model.get();
    CitySim sim;
    scene.build(sim, 400, 13);

    // Places make assignPlaces (which measures the commute) meaningful.
    PlaceMap places;
    for (int i = 0; i < 12; ++i)
        places.add(PlaceType::Home, Vec2(-250 + i * 30.0, -200), nav);
    for (int i = 0; i < 6; ++i)
        places.add(PlaceType::Office, Vec2(-150 + i * 45.0, 200), nav, 9, 17);
    sim.assignPlaces(places);

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

// A slept day must be the SAME day. The wake time is derived from the schedule,
// so an agent that sleeps through a stretch and wakes on its window must end up
// where one that was polled the whole time does.
TEST_CASE(sleeping_does_not_change_the_day) {
    const citytest::ModelScene& scene = town();
    CitySim a;
    scene.build(a,180, 31);

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
    scene.build(b,180, 31);
    for (int i = 0; i < 6000; ++i) b.step(0.1, 0.05);
    bool same = true;
    for (std::size_t i = 0; i < a.agents().size() && same; ++i)
        if (a.agents()[i].pos.x != b.agents()[i].pos.x ||
            a.agents()[i].pos.y != b.agents()[i].pos.y ||
            a.agents()[i].activity != b.agents()[i].activity)
            same = false;
    CHECK(same);
}

// THE POPULATION CACHE (Glenn: "could we assign the job and home for each agent offline and save/load that
// information?"). A population READ from the cache is the population DECIDED: every agent's assignment and
// the commute statistics match field for field, and the two cities then run identically. A changed input
// (one shop's hours) misses the cache; RT_POPULATION_VERIFY decides again and finds no difference.
#include <filesystem>
TEST_CASE(population_read_from_the_cache_is_the_population_decided) {
    const std::string dir = (std::filesystem::temp_directory_path() / "rt_population_cache_test").string();
    std::filesystem::remove_all(dir);
    auto town = [&](CitySim& sim, const engine::RoadModel*& nav, PlaceMap& places, Real shopClose) {
        const citytest::ModelScene& scene = citytest::gridScene(1200, 90, 5);   // (model_scene.h, W2)
        nav = scene.model.get();
        scene.build(sim, 500, 13);
        for (int i = 0; i < 14; ++i) places.add(PlaceType::Home, Vec2(-500 + i * 60.0, -400), nav);
        for (int i = 0; i < 6; ++i) places.add(PlaceType::Office, Vec2(-300 + i * 90.0, 400), nav, 9, 17);
        for (int i = 0; i < 4; ++i) places.add(PlaceType::Shop, Vec2(-400 + i * 200.0, 0), nav, 8, shopClose);
        places.add(PlaceType::Park, Vec2(300, -100), nav);
        places.add(PlaceType::Cafe, Vec2(-100, 150), nav, 7, 19);
    };
    const engine::RoadModel *navA = nullptr, *navB = nullptr, *navC = nullptr, *navD = nullptr;
    PlaceMap pA, pB, pC, pD;
    CitySim A, B, C, D;
    town(A, navA, pA, 20);
    A.assignPlaces(pA);                                           // no cache: the reference
    town(B, navB, pB, 20); B.setPopulationCacheDir(dir); B.assignPlaces(pB);   // decides, saves
    town(C, navC, pC, 20); C.setPopulationCacheDir(dir); C.assignPlaces(pC);   // reads
    CHECK(B.populationCache().used && !B.populationCache().hit && B.populationCache().saved);
    CHECK(C.populationCache().hit);
    CHECK(C.populationCache().key == B.populationCache().key);

    int differ = 0;
    for (std::size_t i = 0; i < A.agents().size(); ++i) {
        const Agent& a = A.agents()[i]; const Agent& c = C.agents()[i];
        const bool same = a.home == c.home && a.work == c.work && a.shop == c.shop && a.restNode == c.restNode &&
                          a.homePlace == c.homePlace && a.workPlace == c.workPlace && a.shopPlace == c.shopPlace &&
                          a.homeDoor.x == c.homeDoor.x && a.homeDoor.y == c.homeDoor.y && a.workDoor.x == c.workDoor.x &&
                          a.workDoor.y == c.workDoor.y && a.shopDoor.x == c.shopDoor.x && a.shopDoor.y == c.shopDoor.y &&
                          a.pos.x == c.pos.x && a.pos.y == c.pos.y && a.heading.x == c.heading.x && a.heading.y == c.heading.y &&
                          a.departHome == c.departHome && a.departWork == c.departWork && a.commuteSeconds == c.commuteSeconds &&
                          a.role == c.role && a.indoors == c.indoors;
        differ += !same;
    }
    std::printf("    %zu agents, %d differ; median commute %.1f/%.1f s\n", A.agents().size(), differ,
                A.commuteSecondsMedian(), C.commuteSecondsMedian());
    CHECK(differ == 0);
    CHECK(A.commuteSecondsMedian() == C.commuteSecondsMedian());
    CHECK(A.commuteSecondsMedian() == C.commuteSecondsMedian());
    int roles[16] = {};   // by role id: the first three, then students and the staff roles
    for (const Agent& a : A.agents()) roles[std::min(15, static_cast<int>(a.role))]++;
    std::printf("    roles: %d commuters, %d shopkeepers, %d strollers\n", roles[0], roles[1], roles[2]);
    CHECK((roles[0] > 0) + (roles[1] > 0) + (roles[2] > 0) >= 2);   // more than one branch of the role choice ran

    // and they RUN the same
    for (int s = 0; s < 60 * 30; ++s) { A.step(1.0 / 60.0, 0.004); C.step(1.0 / 60.0, 0.004); }
    int apart = 0;
    for (std::size_t i = 0; i < A.agents().size(); ++i)
        if (A.agents()[i].pos.x != C.agents()[i].pos.x || A.agents()[i].pos.y != C.agents()[i].pos.y) ++apart;
    std::printf("    after 30 s: %d agents in different places\n", apart);
    CHECK(apart == 0);

    // a changed input misses
    town(D, navD, pD, 21); D.setPopulationCacheDir(dir); D.assignPlaces(pD);
    CHECK(!D.populationCache().hit && D.populationCache().key != C.populationCache().key);

    // VERIFY: decide again over a hit, compare
    setenv("RT_POPULATION_VERIFY", "1", 1);
    const engine::RoadModel* navE = nullptr; PlaceMap pE; CitySim E;
    town(E, navE, pE, 20); E.setPopulationCacheDir(dir); E.assignPlaces(pE);
    unsetenv("RT_POPULATION_VERIFY");
    CHECK(E.populationCache().hit && E.populationCache().mismatches == 0);
    std::filesystem::remove_all(dir);
}
