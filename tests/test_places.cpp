#include "test_framework.h"

#include "../src/apps/citysim/agent_id.h"
#include "../src/apps/citysim/places.h"
#include "../src/apps/citysim/city_sim.h"
#include "city_test_util.h"

#include <set>

using namespace engine;
using namespace citysim;

// Living City, ADR-0066 Phase 1: the PLACES layer + agent IDENTITY. Both are
// pure data — no ECS, no rendering, no Lua — so every case here is headless.
// Places turn buildings into routable, type-indexed destinations; AgentIds give
// every agent a stable UID the later relationship / memory / job layers key on.

// ----- place type tags -------------------------------------------------------

TEST_CASE(place_type_name_roundtrip) {
    for (int t = 0; t < static_cast<int>(PlaceType::Count); ++t) {
        const auto pt = static_cast<PlaceType>(t);
        PlaceType parsed;
        CHECK(parsePlaceType(placeTypeName(pt), parsed));
        CHECK(parsed == pt);   // name → parse → same type
    }
    PlaceType junk;
    CHECK(!parsePlaceType("warehouse", junk));   // unknown tag rejected
    CHECK(!parsePlaceType("", junk));
}

// ----- entrance snapping -----------------------------------------------------

TEST_CASE(place_entrance_lands_on_sidewalk) {
    // A real grid town: every door must snap onto a walkable edge, off the road.
    NavGraph nav = citytest::cityNav(200, 90, 7);
    CHECK(nav.linkCount() > 0);

    PlaceMap places;
    // Scatter a few buildings at arbitrary sites across the town.
    const Vec2 sites[] = {{20, 15}, {-40, 60}, {75, -30}, {5, -80}};
    for (const Vec2& s : sites) places.add(PlaceType::Home, s, nav);

    CHECK(places.size() == 4);
    for (const Place& p : places.places()) {
        CHECK(p.entranceLink >= 0);
        CHECK(p.entranceLink < nav.linkCount());
        CHECK(p.entranceT >= 0.0 && p.entranceT <= 1.0);
        // The door sits on the verge OUTSIDE the carriageway: its distance from
        // the road centreline is at least the road's half-width.
        const NavLink& link = nav.links[p.entranceLink];
        const Vec2 centre = nav.pointOnLink(p.entranceLink, p.entranceT);
        const Real offset = (p.entrance - centre).length();
        CHECK(offset >= link.width * 0.5 - 1e-6);
        // And it is exactly the sidewalk point for its snapped (link, t).
        const Vec2 walk = nav.sidewalkPoint(p.entranceLink, p.entranceT);
        CHECK_APPROX(p.entrance.x, walk.x, 1e-9);
        CHECK_APPROX(p.entrance.y, walk.y, 1e-9);
    }
}

TEST_CASE(place_entrance_empty_graph_falls_back) {
    NavGraph empty;
    PlaceMap places;
    PlaceId id = places.add(PlaceType::Shop, Vec2(3, 4), empty);
    const Place& p = places[id];
    CHECK(p.entranceLink == -1);      // nothing to snap to
    CHECK_APPROX(p.entrance.x, 3.0, 1e-9);   // entrance = site, untouched
    CHECK_APPROX(p.entrance.y, 4.0, 1e-9);
}

// ----- indexing + nearest ----------------------------------------------------

TEST_CASE(placemap_indexes_by_type) {
    NavGraph nav = citytest::straightRoad(120, 8);
    PlaceMap places;
    places.add(PlaceType::Home,  {10, 5}, nav);
    places.add(PlaceType::Shop,  {30, 5}, nav);
    places.add(PlaceType::Home,  {50, 5}, nav);
    places.add(PlaceType::Office,{70, 5}, nav);

    CHECK(places.countOfType(PlaceType::Home) == 2);
    CHECK(places.countOfType(PlaceType::Shop) == 1);
    CHECK(places.countOfType(PlaceType::Office) == 1);
    CHECK(places.countOfType(PlaceType::Park) == 0);
    // Ids come back in insertion order.
    CHECK(places.ofType(PlaceType::Home)[0] == 0u);
    CHECK(places.ofType(PlaceType::Home)[1] == 2u);
}

TEST_CASE(placemap_nearest_of_type) {
    NavGraph nav = citytest::straightRoad(200, 8);
    PlaceMap places;
    PlaceId a = places.add(PlaceType::Shop, {20, 6}, nav);   // near x=20
    PlaceId b = places.add(PlaceType::Shop, {150, 6}, nav);  // near x=150
    places.add(PlaceType::Home, {80, 6}, nav);               // wrong type, ignored

    CHECK(places.nearest(PlaceType::Shop, {25, 6}) == a);
    CHECK(places.nearest(PlaceType::Shop, {140, 6}) == b);
    // No park anywhere → kNoPlace.
    CHECK(places.nearest(PlaceType::Park, {0, 0}) == kNoPlace);
}

// ----- open hours ------------------------------------------------------------

TEST_CASE(place_open_hours) {
    Place day;                       // 9..17 shop
    day.openHour = 9; day.closeHour = 17;
    CHECK(!day.openAt(8.0));
    CHECK(day.openAt(9.0));
    CHECK(day.openAt(12.0));
    CHECK(!day.openAt(17.0));         // closed AT close
    CHECK(!day.openAt(20.0));

    Place always;                    // default 0..24 window
    CHECK(always.openAt(0.0));
    CHECK(always.openAt(23.9));

    Place night;                     // 20..4 — wraps midnight
    night.openHour = 20; night.closeHour = 4;
    CHECK(night.openAt(22.0));
    CHECK(night.openAt(2.0));
    CHECK(!night.openAt(12.0));
}

// ----- agent identity --------------------------------------------------------

TEST_CASE(agent_id_allocator_is_monotonic) {
    AgentIdAllocator ids;
    CHECK(ids.issued() == 0u);
    CHECK(ids.next() == 0u);
    CHECK(ids.next() == 1u);
    CHECK(ids.next() == 2u);
    CHECK(ids.issued() == 3u);
    ids.reset();                     // rebuild → back to 0, reproducible
    CHECK(ids.next() == 0u);
}

TEST_CASE(agent_uids_stable_and_unique) {
    NavGraph nav = citytest::cross4(40);
    CitySim sim;
    sim.build(nav, /*drivers*/ 3, /*peds*/ 4, /*seed*/ 99);

    // Every agent has a distinct, dense UID in [0, count).
    std::set<AgentId> seen;
    for (const Agent& a : sim.agents()) {
        CHECK(a.uid != kNoAgent);
        CHECK(a.uid < static_cast<AgentId>(sim.agents().size()));
        CHECK(seen.insert(a.uid).second);   // no duplicates
    }
    CHECK(seen.size() == sim.agents().size());

    // Determinism (ADR-0002): the same seed reproduces the same UID sequence.
    CitySim sim2;
    sim2.build(nav, 3, 4, 99);
    for (std::size_t i = 0; i < sim.agents().size(); ++i)
        CHECK(sim.agents()[i].uid == sim2.agents()[i].uid);
}


TEST_CASE(place_entrance_hint_wins_over_the_site_snap) {
    // ADR-0080: a building knows its real door; the sidewalk snap must run
    // from the door, not the footprint centroid (which can be around a
    // corner from it).
    NavGraph nav = citytest::straightRoad(120, 8);
    PlaceMap places;
    const Vec2 site(60, 20);
    const Vec2 door(20, 12);
    PlaceId a = places.add(PlaceType::Shop, site, nav);
    PlaceId b = places.add(PlaceType::Shop, site, nav, 0, 24, 0, {}, &door);
    const Real dxa = places[a].entrance.x - site.x;   // legacy: near the site
    const Real dxb = places[b].entrance.x - door.x;   // hinted: near the door
    CHECK(dxa < 6.0);
    CHECK(dxa > -6.0);
    CHECK(dxb < 6.0);
    CHECK(dxb > -6.0);
    // And the two entrances genuinely differ -- the hint changed the snap.
    const Real sep = places[b].entrance.x - places[a].entrance.x;
    CHECK((sep < -20.0) || (sep > 20.0));
}
