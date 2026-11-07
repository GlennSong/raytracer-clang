// CAR DAMAGE (ADR-0353): the zone a touch lands on, what it costs, whether it sparks, and the dent it leaves.

#include "test_framework.h"
#include "../src/engine/vehicle_damage.h"
#include "../src/engine/mesh_builder.h"

using namespace engine;

TEST_CASE(a_touch_lands_on_the_zone_it_hits) {
    const Vec3 h(0.9, 0.7, 2.3);   // a sedan's half-extents
    CHECK(damageZone(Vec3(0.1, 0.0, 2.3), h) == DamageZone::Front);
    CHECK(damageZone(Vec3(-0.2, 0.1, -2.25), h) == DamageZone::Rear);
    CHECK(damageZone(Vec3(0.9, 0.0, 0.3), h) == DamageZone::Left);     // +X: the driver's left
    CHECK(damageZone(Vec3(-0.9, 0.0, -0.5), h) == DamageZone::Right);
    CHECK(damageZone(Vec3(0.0, 0.7, 0.0), h) == DamageZone::Roof);
    CHECK(damageZone(Vec3(0.1, -0.7, 0.2), h) == DamageZone::Under);
    CHECK(damageZone(Vec3(0.85, 0.0, 2.2), h) == DamageZone::Front);   // a corner knock is a nose knock
}

TEST_CASE(a_knock_costs_by_its_speed_and_the_car_by_its_mass) {
    const DamageRule r;
    CHECK(touchHarm(r, false, 2.0, 0, 1.0 / 60, 1500) == 0);           // a nudge leaves no mark
    const Real mild = touchHarm(r, false, 8.0, 0, 1.0 / 60, 1500);
    const Real hard = touchHarm(r, false, 25.0, 0, 1.0 / 60, 1500);
    CHECK(mild > 0 && mild < 0.2);
    CHECK(hard > 0.7 && hard <= 1.0);                                    // ~90 km/h into a wall spends a zone
    CHECK(touchHarm(r, false, 25.0, 0, 1.0 / 60, 12000) < hard * 0.2);   // a bus shrugs it off
    // a scrape: a few metres along a wall at speed costs a little, not a zone
    Real scraped = 0;
    for (int i = 0; i < 60; ++i) scraped += touchHarm(r, true, 0.3, 10.0, 1.0 / 60, 1500);
    CHECK(scraped > 0.01 && scraped < 0.1);
}

TEST_CASE(hard_knocks_and_fast_scrapes_spark) {
    const DamageRule r;
    CHECK(touchSparks(r, false, 2.0, 0) == 0);
    CHECK(touchSparks(r, false, 12.0, 0) > 10);
    CHECK(touchSparks(r, true, 0.2, 1.0) == 0);    // leaning on a wall
    CHECK(touchSparks(r, true, 0.2, 8.0) > 0);     // grinding along it
}

TEST_CASE(a_dent_pushes_the_skin_in_round_the_point_and_nowhere_else) {
    RenderMesh m;
    // a flat 2 x 2 m panel facing +X at x = 1, 21 x 21 points
    const int n = 21;
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            Vertex v;
            v.position = Vec3(1.0, -1.0 + 2.0 * j / (n - 1), -1.0 + 2.0 * i / (n - 1));
            v.normal = Vec3(1, 0, 0);
            m.vertices.push_back(v);
        }
    for (int j = 0; j + 1 < n; ++j)
        for (int i = 0; i + 1 < n; ++i) {
            const uint32_t a = static_cast<uint32_t>(j * n + i), b = a + 1, c = a + static_cast<uint32_t>(n), d = c + 1;
            m.indices.insert(m.indices.end(), {a, c, b, b, c, d});
        }
    const RenderMesh before = m;
    CHECK(dentMesh(m, Vec3(1.0, 0, 0), Vec3(-1, 0, 0), 0.5, 0.1));
    Real deepest = 0;
    bool farMoved = false;
    for (std::size_t k = 0; k < m.vertices.size(); ++k) {
        const Real moved = before.vertices[k].position.x - m.vertices[k].position.x;
        deepest = std::max(deepest, moved);
        if ((before.vertices[k].position - Vec3(1, 0, 0)).length() >= 0.5 && std::fabs(moved) > 1e-9) farMoved = true;
        CHECK(moved >= -1e-9);   // only ever in
    }
    CHECK(std::fabs(deepest - 0.1) < 1e-6);   // the full depth at the point
    CHECK(!farMoved);                          // nothing past the radius
    CHECK(!dentMesh(m, Vec3(5, 5, 5), Vec3(-1, 0, 0), 0.5, 0.1));   // a miss moves nothing
}
