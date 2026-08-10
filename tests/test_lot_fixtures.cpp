#include "test_framework.h"

#include "../src/engine/procgen/city/lot_fixtures.h"
#include <cmath>

using namespace engine;

namespace {
bool near(Real a, Real b, Real tol) { return std::fabs(a - b) <= tol; }

Shape2 streetLot(Real w, Real d) {
    Shape2 lot = rectShape(0, 0, w, d);
    lot.outer.edges[0].tag = EdgeTag::Street;
    lot.outer.edges[1].tag = EdgeTag::Side;
    lot.outer.edges[2].tag = EdgeTag::Rear;
    lot.outer.edges[3].tag = EdgeTag::Side;
    return lot;
}

LotProgram villaProgram() {
    for (const LotProgram& p : residentialPrograms().programs)
        if (p.name == "villa") return p;
    return LotProgram{};
}

int countKind(const LotFixtures& f, InteractKind k) {
    int n = 0;
    for (const InteractableSpec& s : f.interactables) if (s.kind == k) ++n;
    return n;
}
}  // namespace

// --- instancing (P6) -------------------------------------------------------

TEST_CASE(fixtures_props_batch_by_kind_without_losing_any) {
    std::vector<PropInstance> props;
    for (int i = 0; i < 12; ++i) {
        PropInstance p;
        p.kind = (i % 3 == 0) ? PropKind::Tree
                              : (i % 3 == 1 ? PropKind::Shrub : PropKind::Bench);
        p.at = Vec2(i, i * 2);
        p.yaw = i * 0.1;
        props.push_back(p);
    }
    std::vector<InstanceBatch> b = batchProps(props);
    CHECK(b.size() == 3);
    std::size_t total = 0;
    for (const InstanceBatch& x : b) {
        CHECK(!x.instances.empty());
        total += x.instances.size();
    }
    CHECK(total == props.size());          // nothing dropped in the regrouping
}

TEST_CASE(fixtures_empty_props_batch_to_nothing) {
    CHECK(batchProps({}).empty());
}

// --- light budget (P8) -----------------------------------------------------

TEST_CASE(light_budget_ranks_by_intensity_over_distance) {
    // A bright distant lamp can outrank a dim near one — the whole reason the
    // score is falloff-weighted rather than nearest-N.
    LightSpec near1;
    near1.at = Vec3(1, 0, 0);
    near1.intensity = 0.05;
    LightSpec far1;
    far1.at = Vec3(6, 0, 0);
    far1.intensity = 8.0;
    const Vec3 eye(0, 0, 0);
    CHECK(LightBudget::score(far1, eye) > LightBudget::score(near1, eye));
    // ... but at equal intensity, nearer wins.
    far1.intensity = 0.05;
    CHECK(LightBudget::score(near1, eye) > LightBudget::score(far1, eye));
}

TEST_CASE(light_budget_never_exceeds_its_slots) {
    std::vector<LightSpec> lights;
    for (int i = 0; i < 500; ++i) {
        LightSpec l;
        l.at = Vec3(i * 0.7, 3, (i % 17) * 1.1);
        l.intensity = 0.2 + (i % 5) * 0.3;
        lights.push_back(l);
    }
    LightBudget b;
    std::vector<int> sel = b.select(lights, Vec3(0, 2, 0));
    CHECK(static_cast<int>(sel.size()) == b.slots);
    // No index appears twice.
    std::vector<char> seen(lights.size(), 0);
    for (int i : sel) {
        CHECK(i >= 0 && i < static_cast<int>(lights.size()));
        CHECK(!seen[i]);
        seen[i] = 1;
    }
}

// Emissive fixtures cost nothing and must NEVER take a slot — that is tier 1 of
// the lighting ladder doing its job, and it is what lets a city have thousands
// of lit windows behind a 32-light wall.
TEST_CASE(light_budget_ignores_emissive_only_fixtures) {
    std::vector<LightSpec> lights;
    for (int i = 0; i < 100; ++i) {
        LightSpec l;
        l.at = Vec3(0.1 * i, 1, 0);
        l.intensity = 9.0;                 // very bright, very near
        l.emissiveOnly = true;
        lights.push_back(l);
    }
    LightSpec real;
    real.at = Vec3(80, 1, 0);              // far away and dim by comparison
    real.intensity = 0.1;
    lights.push_back(real);

    LightBudget b;
    std::vector<int> sel = b.select(lights, Vec3(0, 1, 0));
    CHECK(sel.size() == 1);
    CHECK(sel[0] == static_cast<int>(lights.size()) - 1);
}

// The starvation the plan warns about: a dense lot's own fixtures must not win
// every slot near the camera and leave the STREET dark.
TEST_CASE(light_budget_reserves_slots_for_street_lamps) {
    std::vector<LightSpec> lights;
    for (int i = 0; i < 200; ++i) {        // a swarm of near, bright lot lights
        LightSpec l;
        l.kind = FixtureKind::PorchLight;
        l.at = Vec3(0.05 * i, 2, 0.05 * i);
        l.intensity = 5.0;
        lights.push_back(l);
    }
    for (int i = 0; i < 14; ++i) {         // street lamps, further off
        LightSpec l;
        l.kind = FixtureKind::StreetLamp;
        l.at = Vec3(40 + i * 12, 6, 0);
        l.intensity = 2.0;
        lights.push_back(l);
    }
    LightBudget b;
    std::vector<int> sel = b.select(lights, Vec3(0, 2, 0));
    int street = 0;
    for (int i : sel) if (lights[i].kind == FixtureKind::StreetLamp) ++street;
    CHECK(street > 0);
    CHECK(street >= static_cast<int>(b.slots * b.reserveForStreet) - 1);
    CHECK(static_cast<int>(sel.size()) == b.slots);
}

TEST_CASE(light_budget_selection_is_deterministic) {
    std::vector<LightSpec> lights;
    for (int i = 0; i < 90; ++i) {
        LightSpec l;
        l.at = Vec3(i % 9, 2, i / 9);
        l.intensity = 1.0;                 // deliberately all equal: ties must
        lights.push_back(l);               // resolve stably, not arbitrarily
    }
    LightBudget b;
    CHECK(b.select(lights, Vec3(4, 2, 4)) == b.select(lights, Vec3(4, 2, 4)));
}

// --- interactivity (P7) ----------------------------------------------------

TEST_CASE(fixtures_every_gate_becomes_an_interactable_with_a_sensor) {
    Shape2 lot = streetLot(28, 36);
    LotTags tags = measureLot(lot, {}, StreetClass::Street, true, 0.3);
    LotProgram villa = villaProgram();
    SitePlan site = layoutSite(lot, tags, villa, 12);
    SiteFurnishing furn = furnishSite(site, villa, 12);
    CHECK(!site.gates.empty());

    PaletteLibrary lib = stockPalettes();
    RecipeBook book = stockRecipes();
    BuiltBuilding b = buildFromRecipe(*book.byName("villa"), site.buildEnvelope,
                                      tags, lib, 4, 12);
    LotFixtures f = fixturesFor(site, furn, villa, b, 0.0, 12);

    CHECK(countKind(f, InteractKind::Gate) == static_cast<int>(site.gates.size()));
    // Each gate carries a sensor pointing back at it.
    int gateSensors = 0;
    for (const SensorSpec& s : f.sensors) {
        CHECK(s.interactableIndex >= 0);
        CHECK(s.interactableIndex < static_cast<int>(f.interactables.size()));
        if (f.interactables[s.interactableIndex].kind == InteractKind::Gate)
            ++gateSensors;
    }
    CHECK(gateSensors == static_cast<int>(site.gates.size()));
}

// A gate that pivots about its middle is a turnstile. The hinge must sit at an
// END of the leaf.
TEST_CASE(fixtures_a_gate_hinges_on_its_post_not_its_centre) {
    Shape2 lot = streetLot(28, 36);
    LotTags tags = measureLot(lot, {}, StreetClass::Street, true, 0.3);
    LotProgram villa = villaProgram();
    SitePlan site = layoutSite(lot, tags, villa, 5);
    SiteFurnishing furn = furnishSite(site, villa, 5);
    PaletteLibrary lib = stockPalettes();
    RecipeBook book = stockRecipes();
    BuiltBuilding b = buildFromRecipe(*book.byName("villa"), site.buildEnvelope,
                                      tags, lib, 4, 5);
    LotFixtures f = fixturesFor(site, furn, villa, b, 0.0, 5);

    for (std::size_t i = 0; i < f.interactables.size(); ++i) {
        const InteractableSpec& s = f.interactables[i];
        if (s.kind != InteractKind::Gate) continue;
        const Vec2 centre(s.at.x, s.at.z);
        const Vec2 post(s.hinge.anchor.x, s.hinge.anchor.z);
        // The post is half a leaf-width from the centre, not on top of it.
        CHECK(near((post - centre).length(), s.width * 0.5, 1e-6));
        CHECK(near(s.hinge.axis.y, 1.0, 1e-9));      // gates swing about vertical
        CHECK(s.hinge.maxAngle > 0.5);
        CHECK(s.tierBCapable);                        // upgradeable to a real hinge
    }
}

TEST_CASE(fixtures_entrance_becomes_a_door_and_a_porch_light) {
    Shape2 lot = streetLot(26, 32);
    LotTags tags = measureLot(lot, {}, StreetClass::Street, true, 0.3);
    LotProgram villa = villaProgram();
    SitePlan site = layoutSite(lot, tags, villa, 6);
    SiteFurnishing furn = furnishSite(site, villa, 6);
    PaletteLibrary lib = stockPalettes();
    RecipeBook book = stockRecipes();
    BuiltBuilding b = buildFromRecipe(*book.byName("villa"), site.buildEnvelope,
                                      tags, lib, 4, 6);
    LotFixtures f = fixturesFor(site, furn, villa, b, 0.0, 6);

    CHECK(site.hasEntrance);
    CHECK(countKind(f, InteractKind::Door) == 1);
    int porch = 0;
    for (const LightSpec& l : f.lights)
        if (l.kind == FixtureKind::PorchLight) ++porch;
    CHECK(porch == 1);
}

// A shopfront's door is powered; a garden gate swings free. The distinction is
// data, from the program, not a branch in the gate code.
TEST_CASE(fixtures_a_shop_door_is_motorised_and_a_gate_is_not) {
    Shape2 lot = streetLot(26, 24);
    LotTags tags = measureLot(lot, {}, StreetClass::Arterial, true, 0.5);
    LotProgram shop;
    for (const LotProgram& p : commercialPrograms().programs)
        if (p.name == "shopfront") shop = p;
    SitePlan site = layoutSite(lot, tags, shop, 8);
    SiteFurnishing furn = furnishSite(site, shop, 8);
    PaletteLibrary lib = stockPalettes();
    RecipeBook book = stockRecipes();
    BuiltBuilding b = buildFromRecipe(*book.byName("shopfront"), site.buildEnvelope,
                                      tags, lib, 4, 8);
    LotFixtures f = fixturesFor(site, furn, shop, b, 0.0, 8);

    for (const InteractableSpec& s : f.interactables)
        if (s.kind == InteractKind::Door) CHECK(s.hinge.motor);
    // A Direct-frontage program has no fence, so no gates at all.
    CHECK(countKind(f, InteractKind::Gate) == 0);
}

TEST_CASE(fixtures_benches_become_seats) {
    SitePlan site;
    site.lot = streetLot(20, 20);
    SiteFurnishing furn;
    PropInstance bench;
    bench.kind = PropKind::Bench;
    bench.at = Vec2(5, 5);
    bench.yaw = 0.7;
    furn.props.push_back(bench);
    PropInstance shrub;
    shrub.kind = PropKind::Shrub;
    shrub.at = Vec2(8, 8);
    furn.props.push_back(shrub);

    BuiltBuilding b;
    LotFixtures f = fixturesFor(site, furn, villaProgram(), b, 0.0, 3);
    CHECK(f.seats.size() == 1);
    CHECK(near(f.seats[0].yaw, 0.7, 1e-9));
    CHECK(f.seats[0].propIndex == 0);
    CHECK(countKind(f, InteractKind::Seat) == 1);
}

TEST_CASE(fixtures_path_and_window_lights_are_emissive_only) {
    Shape2 lot = streetLot(28, 36);
    LotTags tags = measureLot(lot, {}, StreetClass::Street, true, 0.3);
    LotProgram villa = villaProgram();
    SitePlan site = layoutSite(lot, tags, villa, 15);
    SiteFurnishing furn = furnishSite(site, villa, 15);
    PaletteLibrary lib = stockPalettes();
    RecipeBook book = stockRecipes();
    BuiltBuilding b = buildFromRecipe(*book.byName("villa"), site.buildEnvelope,
                                      tags, lib, 4, 15);
    LotFixtures f = fixturesFor(site, furn, villa, b, 0.0, 15);

    for (const LightSpec& l : f.lights) {
        if (l.kind == FixtureKind::PathLight || l.kind == FixtureKind::WindowGlow ||
            l.kind == FixtureKind::SignFace)
            CHECK(l.emissiveOnly);
        if (l.kind == FixtureKind::PorchLight || l.kind == FixtureKind::GateLamp)
            CHECK(!l.emissiveOnly);
    }
    // A lot spends at most a couple of real light slots, however much it glows.
    int real = 0;
    for (const LightSpec& l : f.lights) if (!l.emissiveOnly) ++real;
    CHECK(real <= 4);
}

TEST_CASE(fixtures_are_deterministic) {
    Shape2 lot = streetLot(30, 34);
    LotTags tags = measureLot(lot, {}, StreetClass::Street, true, 0.3);
    LotProgram villa = villaProgram();
    SitePlan site = layoutSite(lot, tags, villa, 21);
    SiteFurnishing furn = furnishSite(site, villa, 21);
    PaletteLibrary lib = stockPalettes();
    RecipeBook book = stockRecipes();
    BuiltBuilding b = buildFromRecipe(*book.byName("villa"), site.buildEnvelope,
                                      tags, lib, 4, 21);
    LotFixtures a = fixturesFor(site, furn, villa, b, 0.0, 21);
    LotFixtures c = fixturesFor(site, furn, villa, b, 0.0, 21);
    CHECK(a.interactables.size() == c.interactables.size());
    CHECK(a.lights.size() == c.lights.size());
    CHECK(a.sensors.size() == c.sensors.size());
    CHECK(a.batches.size() == c.batches.size());
}
