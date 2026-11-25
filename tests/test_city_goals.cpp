#include "test_framework.h"

#include "model_scene.h"   // the towns: road files built as a level's roads are (#306 piece 6 W2)
#include "../src/apps/citysim/city_goals.h"
#include "../src/apps/citysim/city_sim.h"

#include <cstdio>
#include <map>
#include <vector>

using namespace engine;
using namespace citysim;

// A place given a kind by name, from the built-in place kinds (places.lua's twin): its tags come with it.
static void kindOf(PlaceMap& places, PlaceId id, const char* name) {
    const PlaceKindCatalog c = defaultPlaceKindCatalog();
    const int k = c.find(name);
    places.setKind(id, k, k >= 0 ? c.kinds[static_cast<std::size_t>(k)].tags : 0u);
}

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
    GoalTable t = builtinDayTable("worker");
    int home = t.findState("AtHome");
    int commute = t.findState("ToWork");
    int work = t.findState("AtWork");
    int ret = t.findState("ReturnHome");
    CHECK(t.stateCount() == 14);   // the day's ten, and a night out's four (addNightOut)
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
    const int lunchGo = t.findState("GoBreak");
    const int lunchAt = t.findState("OnBreak");
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
    GoalTable t = builtinDayTable("at_home");
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

// THE DORM BLOCK (campus gap pass): with two residence halls the students fill both, in proportion to their beds, and
// their courses are spread over every teaching hall.
TEST_CASE(students_fill_every_hall_and_take_classes_in_every_teaching_hall) {
    const citytest::ModelScene& scene = citytest::gridScene(800.0, 80.0, 5);
    const engine::RoadModel* nav = scene.model.get();
    CitySim sim;
    scene.build(sim,420, 21);
    PlaceMap places;
    for (int i = 0; i < 10; ++i) places.add(PlaceType::Home, Vec2(-350 + i * 70.0, -330), nav);
    places.add(PlaceType::Office, Vec2(0, 330), nav, 9, 17);
    const PlaceId h1 = places.add(PlaceType::Home, Vec2(-120, 60), nav, 0, 24, 30);
    const PlaceId h2 = places.add(PlaceType::Home, Vec2(200, -150), nav, 0, 24, 20);
    const PlaceId t1 = places.add(PlaceType::Civic, Vec2(60, 120), nav);
    const PlaceId t2 = places.add(PlaceType::Civic, Vec2(-60, 200), nav);
    kindOf(places, h1, "residence"); kindOf(places, h2, "residence"); kindOf(places, t1, "teaching"); kindOf(places, t2, "teaching");
    sim.assignPlaces(places);
    int in1 = 0, in2 = 0, at1 = 0, at2 = 0;
    for (const Agent& a : sim.agents()) {
        if (!sim.isStudent(a)) continue;
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

// HOUSEHOLDS IN THE DWELLINGS (households.h; Glenn, 2026-11-01: "a family would be expressed by like a unit"; "Do a
// cap for now"). Every seed: one household to a dwelling at most; every member of one lives at its home; a child (or
// any walk-only member) is a walker and holds no job; everyone has a home. With more people than the housing holds,
// the rest are counted unhoused (and take a home by area, as before); with housing to spare, nobody is.
TEST_CASE(households_fill_the_dwellings_one_to_each) {
    for (int perBlock : {6, 40})
        for (uint32_t seed = 1; seed <= 4; ++seed) {
            const citytest::ModelScene& scene = citytest::gridScene(800.0, 80.0, seed);
        const engine::RoadModel* nav = scene.model.get();
            CitySim sim;
            scene.build(sim,480, seed);
            PlaceMap places;
            for (int i = 0; i < 24; ++i) {
                const PlaceId h = places.add(PlaceType::Home, Vec2(-350 + (i % 12) * 60.0, i < 12 ? -330 : 330), nav);
                std::vector<int> beds;
                if (i % 3 == 0) beds = {i % 2 ? 3 : 5};   // a house
                else for (int d = 0; d < perBlock; ++d) beds.push_back(d % 4);   // flats, studio to 3-bed
                places.setDwellings(h, beds);
            }
            places.add(PlaceType::Office, Vec2(0, 0), nav, 9, 17);
            places.add(PlaceType::Park, Vec2(150, 150), nav);
            places.add(PlaceType::Cafe, Vec2(-150, 150), nav, 7, 19);
            sim.assignPlaces(places);
            const HouseholdPlan& plan = sim.households();
            std::vector<int> taken(places.dwellings().size(), 0);
            for (const Household& h : plan.households)
                if (h.dwelling != Household::kNoDwelling) ++taken[h.dwelling];
            int twice = 0;
            for (int t : taken) twice += t > 1 ? 1 : 0;
            int housed = 0, wrongHome = 0, kidsWorking = 0, homeless = 0, kids = 0;
            for (int i = 0; i < static_cast<int>(sim.agents().size()); ++i) {
                const Agent& a = sim.agents()[static_cast<std::size_t>(i)];
                homeless += a.homePlace == kNoPlace ? 1 : 0;
                if (a.household < 0) continue;
                ++housed;
                const Household& h = plan.households[static_cast<std::size_t>(a.household)];
                wrongHome += a.homePlace != h.place ? 1 : 0;
                const LifeStage& st = sim.people().stages[static_cast<std::size_t>(a.stage)];
                if (st.walks && h.type != Household::kHall) {
                    ++kids;
                    kidsWorking += a.workPlace != kNoPlace ? 1 : 0;
                }
            }
            const int residents = static_cast<int>(sim.agents().size());
            std::printf("    [households] %d a block, seed %u: %s; unhoused %d\n", perBlock, seed, sim.populationLine().c_str(),
                        sim.unhousedCount());
            CHECK(twice == 0);
            CHECK(housed == static_cast<int>(plan.people.size()));
            CHECK(sim.unhousedCount() == residents - housed);
            CHECK(wrongHome == 0);
            CHECK(kids > 0 && kidsWorking == 0);
            CHECK(homeless == 0);
            CHECK(plan.capacity >= housed);
            if (perBlock == 6) CHECK(sim.unhousedCount() > 0 && plan.dwellingsFilled >= plan.dwellings * 9 / 10);
            else CHECK(sim.unhousedCount() == 0);
        }
    // the same city houses the same people the same way
    auto who = [](uint32_t seed) {
        const citytest::ModelScene& scene = citytest::gridScene(800.0, 80.0, seed);
        const engine::RoadModel* nav = scene.model.get();
        CitySim sim;
        scene.build(sim,260, seed);
        PlaceMap places;
        for (int i = 0; i < 12; ++i) places.setDwellings(places.add(PlaceType::Home, Vec2(-350 + i * 60.0, -330), nav), {0, 1, 2, 3, 2, 1});
        places.add(PlaceType::Office, Vec2(0, 0), nav, 9, 17);
        sim.assignPlaces(places);
        std::vector<int> out;
        for (const Agent& a : sim.agents()) { out.push_back(a.household); out.push_back(a.age); out.push_back(a.stage); }
        return out;
    };
    CHECK(who(7) == who(7));
    CHECK(who(7) != who(8));
}

// THE HOUSEHOLDS' TIES (people.lua ties; Glenn's plan step 4): on every seed, every household's members are as old as
// its ties need (a parent 18 years older than each child, a grandparent 36), and the sim ties its housemates -- each
// pair a Household bond, a parent's child seeing them as its parent -- and nobody outside the dwelling.
TEST_CASE(housemates_are_tied_as_their_household_says) {
    const PeopleCatalog cat = defaultPeopleCatalog();
    const citytest::ModelScene& scene = citytest::gridScene(800.0, 80.0, 3);
    const engine::RoadModel* nav = scene.model.get();
    PlaceMap places;
    std::vector<int> beds;
    for (int d = 0; d < 12; ++d) beds.push_back(d % 6);
    for (int i = 0; i < 200; ++i) places.setDwellings(places.add(PlaceType::Home, Vec2(-350 + (i % 20) * 35.0, -330 + (i / 20) * 60.0), nav), beds);
    for (uint32_t seed = 1; seed <= 6; ++seed) {
        const HouseholdPlan plan = planHouseholds(cat, places, 0, 6000, 3000, seed);
        int checked = 0, wrong = 0;
        for (const Household& hh : plan.households) {
            if (hh.type >= cat.households.size()) continue;
            for (const HouseholdTie& ht : cat.households[hh.type].ties) {
                const int older = cat.ties[static_cast<std::size_t>(ht.tie)].older;
                if (older <= 0) continue;
                for (uint32_t i = hh.first; i < hh.first + hh.count; ++i)
                    for (uint32_t j = hh.first; j < hh.first + hh.count; ++j)
                        if (plan.people[i].group == ht.from && plan.people[j].group == ht.to) {
                            ++checked;
                            wrong += plan.people[i].age < plan.people[j].age + older ? 1 : 0;
                        }
            }
        }
        std::printf("    [ties] seed %u: %zu households, %d aged pairs, %d too close in age\n", seed, plan.households.size(), checked, wrong);
        CHECK(checked > 0);
        CHECK(wrong == 0);
    }
    // the sim: every Household bond is between agents of one household, and a parent / child pair reads both ways
    CitySim sim;
    scene.build(sim,300, 5);
    places.add(PlaceType::Office, Vec2(0, 0), nav, 9, 17);
    sim.assignPlaces(places);
    const RelationshipTable& rel = sim.relationships();
    std::map<AgentId, int> hhOf;
    for (const Agent& a : sim.agents()) hhOf[a.uid] = a.household;
    int bonds = 0, across = 0, parents = 0, unanswered = 0;
    for (const Agent& a : sim.agents())
        for (const auto& [other, bond] : rel.relationsOf(a.uid)) {
            if (bond != Relationship::Household) continue;
            ++bonds;
            across += hhOf[other] != a.household ? 1 : 0;
            const int t = rel.tieOf(a.uid, other), back = rel.tieOf(other, a.uid);
            if (t == cat.tie("parent")) ++parents;
            unanswered += (t < 0 || back != cat.ties[static_cast<std::size_t>(t)].other) ? 1 : 0;
        }
    std::printf("    [ties] the sim: %d household bonds (%d a parent's), %d across dwellings, %d unanswered\n", bonds, parents, across, unanswered);
    CHECK(bonds > 0);
    CHECK(parents > 0);
    CHECK(across == 0);
    CHECK(unanswered == 0);
}

// HOUSEHOLDS AT ISLAND SCALE (Glenn, 2026-11-01: "How well does this scale to 300k people?"): the plan is sequential
// -- one sort of the dwellings, one pass -- so 170,000 dwellings and 300,000 people must plan in well under a second.
#include <chrono>
TEST_CASE(households_plan_three_hundred_thousand_people_quickly) {
    const citytest::ModelScene& scene = citytest::gridScene(800.0, 80.0, 3);
    const engine::RoadModel* nav = scene.model.get();
    PlaceMap places;
    std::vector<int> beds;
    for (int d = 0; d < 170; ++d) beds.push_back(d % 4);
    for (int i = 0; i < 1000; ++i) places.setDwellings(places.add(PlaceType::Home, Vec2(-350 + (i % 40) * 18.0, -330 + (i / 40) * 25.0), nav), beds);
    const auto t0 = std::chrono::steady_clock::now();
    const HouseholdPlan plan = planHouseholds(defaultPeopleCatalog(), places, 0, 300000, 240000, 9);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("    [households at scale] %zu dwellings, %zu households, %zu people in %.1f ms\n", places.dwellings().size(),
                plan.households.size(), plan.people.size(), ms);
    CHECK(plan.people.size() == 300000);
    CHECK(ms < 2000.0);
}

// (DELETED in #306 piece 6 W2: `footpaths_are_walked_across_a_block_and_never_driven`, which joined a diagonal to the
// hand-made street map with its own footpath joining and routed it with the street map's router. It tested the street
// map's own footpath links, which piece 10 deletes. What it showed is shown on the one road model: a walkway is walked
// and never driven, joined where its ends are stated (road_model_43, road_model_45, lanes_tests), and a crowd walks one
// to work in the sim (below).)
#include "../src/engine/procgen/city/roads/walkway.h"

// A KICKABOUT (activities framework, step B/C: the first GROUP activity, a session): players gather on the pitch, the
// game starts once six are there, each side keeps to its half and they swap ends at half time, the game ends and
// everyone walks off; with too few players it is given up after its gathering time, and nobody is left on the pitch.
namespace {
struct KickRun { int games = 0, started = 0, swapped = 0, givenUp = 0, outOfHalf = 0, offPitch = 0, maxOn = 0; };
}  // namespace



// EVERY ACTIVITY KEEPS THE RULES (activities framework: the generic invariant test). A town with a site of every kind
// runs a crowd on a menu offering EVERY activity in assets/scripts/activities.lua at once, all day. Whatever is added
// to the catalog is held to the same rules without a test of its own:
//   - a held spot is held by someone on their way to it or using it (no leaked reservation);
//   - nobody uses a spot much past its activity's longest stay (nobody stuck);
//   - a session's players are its members, and a dead session has none;
//   - and every activity the town can offer is actually taken up.
#include "../src/apps/citysim/scripting/activities_lua.h"
#include "../src/engine/scripting/script_vm.h"
#include <fstream>
#include <sstream>

// THE LAWN GROUPS (settled formations): a chat is a ring standing round its middle, facing in, that grows as people
// join it; a picnic's ring sits on the grass; sunbathers lie side by side; a game of catch is two, about ten metres
// apart, facing. Several groups share a lawn, never on top of each other.
namespace {
struct LawnRun {
    int sessions = 0, maxMembers = 0, maxConcurrent = 0;
    int settledSamples = 0, notFacingIn = 0, tooClose = 0, sitGround = 0, lying = 0, standing = 0;
    int groupsTooClose = 0;
    double pairLo = 1e9, pairHi = 0;
};
}  // namespace


// THE BUSY STREETS: homes and jobs are drawn by floor area -- a block of flats houses a block of flats' people, a
// tower holds a tower's staff -- not one share per building.
TEST_CASE(people_live_and_work_where_the_floor_space_is) {
    const citytest::ModelScene& scene = citytest::gridScene(800.0, 80.0, 5);
    const engine::RoadModel* nav = scene.model.get();
    CitySim sim;
    scene.build(sim,800, 31);
    PlaceMap places;
    std::vector<PlaceId> houses;
    for (int i = 0; i < 10; ++i) {
        const PlaceId h = places.add(PlaceType::Home, Vec2(-350 + i * 70.0, -330), nav);
        places.setFloorArea(h, 180);   // a two-storey house
        houses.push_back(h);
    }
    const PlaceId flats = places.add(PlaceType::Home, Vec2(-200, -170), nav);
    places.setFloorArea(flats, 18000);   // twelve storeys of 1500 m2
    std::vector<PlaceId> shops;
    for (int i = 0; i < 6; ++i) {
        const PlaceId s = places.add(PlaceType::Shop, Vec2(-300 + i * 120.0, 330), nav, 8, 22);
        places.setFloorArea(s, 120);
        shops.push_back(s);
    }
    const PlaceId tower = places.add(PlaceType::Office, Vec2(150, 170), nav);
    places.setFloorArea(tower, 30000);
    sim.assignPlaces(places);
    int inFlats = 0, inHouses = 0, atTower = 0, atShops = 0;
    for (const Agent& a : sim.agents()) {
        if (a.homePlace == flats) ++inFlats;
        for (PlaceId h : houses) inHouses += a.homePlace == h;
        if (a.workPlace == tower) ++atTower;
        for (PlaceId s : shops) atShops += a.workPlace == s;
    }
    std::printf("    [floors] flats %d residents, 10 houses %d; tower %d staff, 6 shops %d\n", inFlats, inHouses, atTower, atShops);
    CHECK(inFlats > 5 * inHouses);   // 18000 m2 against 1800: ten to one
    CHECK(inHouses > 0);             // ...and the houses are still lived in
    CHECK(atTower > 2 * atShops);    // the walkers' band and the drivers' draws both lean to the tower
}
