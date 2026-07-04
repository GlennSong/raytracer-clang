#include "test_framework.h"

#include "../src/apps/citysim/relationships.h"
#include "../src/apps/citysim/places.h"
#include "../src/apps/citysim/city_sim.h"
#include "city_test_util.h"

#include <set>

using namespace engine;
using namespace citysim;

// Living City, ADR-0066 Phase 3: agents live in the city — a home, a job, and a
// surface-level social graph. The relationship table is pure; place assignment
// runs through a real CitySim so the home/job wiring + relationship seeding are
// pinned headless (the visible commute is viewer-gated).

// ----- relationship table ----------------------------------------------------

TEST_CASE(relationship_table_is_symmetric) {
    RelationshipTable t;
    t.set(1, 2, Relationship::Coworker);
    CHECK(t.get(1, 2) == Relationship::Coworker);
    CHECK(t.get(2, 1) == Relationship::Coworker);   // order-independent
    CHECK(t.get(1, 3) == Relationship::None);        // strangers
    CHECK(t.countFor(1) == 1);
    CHECK(t.countFor(2) == 1);
    CHECK(t.countFor(3) == 0);
}

TEST_CASE(relationship_table_ignores_self_and_noagent) {
    RelationshipTable t;
    t.set(5, 5, Relationship::Neighbor);       // self
    t.set(5, kNoAgent, Relationship::Neighbor); // gone
    t.set(5, 6, Relationship::None);            // no bond
    CHECK(t.pairCount() == 0);
    CHECK(t.countFor(5) == 0);
}

TEST_CASE(relationship_stronger_tag_wins_no_downgrade) {
    RelationshipTable t;
    t.set(1, 2, Relationship::Acquaintance);
    t.set(1, 2, Relationship::Coworker);       // upgrade
    CHECK(t.get(1, 2) == Relationship::Coworker);
    t.set(1, 2, Relationship::Neighbor);       // weaker → ignored
    CHECK(t.get(1, 2) == Relationship::Coworker);
    CHECK(t.countFor(1) == 1);                 // still one bond, not duplicated
    CHECK(t.relationsOf(1).size() == 1u);
    CHECK(t.relationsOf(1)[0].second == Relationship::Coworker);
}

// ----- place assignment through the sim --------------------------------------

namespace {
// A PlaceMap with `homes` homes and one shared office, snapped onto a 4-way grid.
PlaceMap townPlaces(const NavGraph& nav, int homes) {
    PlaceMap p;
    for (int i = 0; i < homes; ++i)
        p.add(PlaceType::Home, Vec2(-30 + i * 8.0, 20), nav);
    p.add(PlaceType::Office, Vec2(30, -20), nav, 9, 17);
    return p;
}
}  // namespace

TEST_CASE(assign_places_gives_every_agent_a_home) {
    NavGraph nav = citytest::cross4(45);
    PlaceMap places = townPlaces(nav, 3);
    CitySim sim;
    sim.build(nav, /*drivers*/ 2, /*peds*/ 3, /*seed*/ 4);
    sim.assignPlaces(places, nav);

    for (const Agent& a : sim.agents()) {
        CHECK(a.homePlace != kNoPlace);
        CHECK(a.homePlace < static_cast<PlaceId>(places.size()));
        CHECK(places[a.homePlace].type == PlaceType::Home);
    }
}

TEST_CASE(assign_places_seeds_coworkers_and_is_deterministic) {
    NavGraph nav = citytest::cross4(45);
    PlaceMap places = townPlaces(nav, 2);   // one shared office → coworkers likely
    CitySim a, b;
    a.build(nav, 2, 3, 7);
    a.assignPlaces(places, nav);
    b.build(nav, 2, 3, 7);
    b.assignPlaces(places, nav);

    // Anyone sharing the single office is a coworker; and two runs of the same
    // seed produce the same assignment + social graph (ADR-0002).
    const auto& ag = a.agents();
    int coworkerPairs = 0;
    for (std::size_t i = 0; i < ag.size(); ++i) {
        CHECK(ag[i].uid == b.agents()[i].uid);
        CHECK(ag[i].homePlace == b.agents()[i].homePlace);
        CHECK(ag[i].workPlace == b.agents()[i].workPlace);
        for (std::size_t j = i + 1; j < ag.size(); ++j)
            if (ag[i].workPlace != kNoPlace && ag[i].workPlace == ag[j].workPlace) {
                CHECK(a.relationships().get(ag[i].uid, ag[j].uid) ==
                      Relationship::Coworker);
                ++coworkerPairs;
            }
    }
    CHECK(coworkerPairs > 0);   // the shared office made at least one coworker bond
}

TEST_CASE(assign_places_no_homes_is_a_noop) {
    NavGraph nav = citytest::cross4(45);
    PlaceMap empty;                       // no places at all
    CitySim sim;
    sim.build(nav, 1, 1, 3);
    sim.assignPlaces(empty, nav);         // must not crash or assign
    for (const Agent& a : sim.agents()) CHECK(a.homePlace == kNoPlace);
    CHECK(sim.relationships().pairCount() == 0);
}
