#include "test_framework.h"

#include "../src/engine/procgen/city/facade_plan.h"
#include <cmath>

using namespace engine;

namespace {
bool near(Real a, Real b, Real tol) { return std::fabs(a - b) <= tol; }

int countKind(const std::vector<ElementPlacement>& v, ElementKind k) {
    int n = 0;
    for (const ElementPlacement& p : v) if (p.kind == k) ++n;
    return n;
}
}  // namespace

// --- bay grid --------------------------------------------------------------

TEST_CASE(bay_grid_closes_on_the_wall_exactly) {
    // Whole bays, so the rhythm never leaves a ragged remainder at one end.
    BayGrid g = bayGridFor(17.0, 3.4);
    CHECK(g.bays == 5);
    CHECK(near(g.bayWidth * g.bays, 17.0, 1e-9));

    // A wall that does not divide evenly still closes: the ACTUAL spacing is
    // reported, not the requested one.
    BayGrid h = bayGridFor(15.0, 4.0);
    CHECK(h.bays == 4);
    CHECK(near(h.bayWidth, 3.75, 1e-9));
    CHECK(near(h.bayWidth * h.bays, 15.0, 1e-9));
}

TEST_CASE(bay_grid_never_reports_zero_bays) {
    // A short wall must still have one bay — zero would make it invisible to
    // every selector, so it would silently lose all its elements.
    BayGrid g = bayGridFor(1.4, 3.4);
    CHECK(g.bays == 1);
    CHECK(near(g.bayWidth, 1.4, 1e-9));
    BayGrid z = bayGridFor(0.0, 3.4);
    CHECK(z.bays >= 1);
}

TEST_CASE(bay_grid_spans_tile_without_gaps_or_overlap) {
    BayGrid g = bayGridFor(24.0, 3.0);
    CHECK(g.bays == 8);
    Real prev = 0;
    for (int i = 0; i < g.bays; ++i) {
        Real t0, t1;
        g.bayspan(i, t0, t1);
        CHECK(near(t0, prev, 1e-12));
        CHECK(t1 > t0);
        CHECK(g.bayAt((t0 + t1) * 0.5) == i);
        prev = t1;
    }
    CHECK(near(prev, 1.0, 1e-12));
}

// --- fenestration ----------------------------------------------------------

// The payoff of tagging edges once at plan time: a party wall cannot sprout
// windows into the neighbour's building, however the geometry is oriented.
TEST_CASE(facade_party_walls_are_blank_by_construction) {
    BayGrid g = bayGridFor(20.0, 3.4);
    FenestrationStyle st;
    CHECK(roleForTag(EdgeTag::Party, 0, false) == WallRole::Blank);
    CHECK(fenestrate(g, WallRole::Blank, st, 0, false).empty());
    // ... even when it is asked to carry the entrance.
    CHECK(fenestrate(g, WallRole::Blank, st, 0, true).empty());
}

TEST_CASE(facade_roles_differ_by_tag_and_floor) {
    CHECK(roleForTag(EdgeTag::Rear, 0, false) == WallRole::Sparse);
    // Retail only on the ground floor of a street wall.
    CHECK(roleForTag(EdgeTag::Street, 0, true) == WallRole::Storefront);
    CHECK(roleForTag(EdgeTag::Street, 1, true) == WallRole::Regular);
    CHECK(roleForTag(EdgeTag::Street, 0, false) == WallRole::Regular);

    BayGrid g = bayGridFor(20.0, 3.4);
    FenestrationStyle st;
    const std::size_t regular = fenestrate(g, WallRole::Regular, st, 1, false).size();
    const std::size_t sparse = fenestrate(g, WallRole::Sparse, st, 1, false).size();
    CHECK(sparse < regular);          // a service wall is not a show facade
    CHECK(sparse > 0);
}

// A door must not be trapped inside one glazing strategy: every wall carrying
// the entrance gets one, whatever its style.
TEST_CASE(facade_the_entrance_wall_always_gets_a_door) {
    BayGrid g = bayGridFor(18.0, 3.4);
    FenestrationStyle st;
    for (WallRole role : {WallRole::Regular, WallRole::Sparse,
                          WallRole::Storefront}) {
        std::vector<Opening> op = fenestrate(g, role, st, 0, true);
        int doors = 0;
        for (const Opening& o : op) if (o.kind == OpeningKind::Door) ++doors;
        CHECK(doors == 1);
    }
}

TEST_CASE(facade_openings_stay_inside_their_bays_and_never_overlap) {
    BayGrid g = bayGridFor(26.0, 3.2);
    FenestrationStyle st;
    for (WallRole role : {WallRole::Regular, WallRole::Storefront,
                          WallRole::Curtain}) {
        std::vector<Opening> op = fenestrate(g, role, st, 0, true);
        CHECK(!op.empty());
        Real lastEnd = -1;
        for (const Opening& o : op) {
            CHECK(o.t0 >= -1e-9 && o.t1 <= 1 + 1e-9);
            CHECK(o.t1 > o.t0);
            CHECK(o.head > o.sill);
            CHECK(o.t0 >= lastEnd - 1e-9);      // ordered and disjoint
            lastEnd = o.t1;
            if (o.bay >= 0) {
                Real b0, b1;
                g.bayspan(o.bay, b0, b1);
                CHECK(o.t0 >= b0 - 1e-9 && o.t1 <= b1 + 1e-9);
            }
        }
    }
}

// --- element registry ------------------------------------------------------

TEST_CASE(element_selectors_compose_over_tag_floor_and_bay) {
    Shape2 plan = rectShape(0, 0, 24, 16);
    plan.outer.edges[0].tag = EdgeTag::Street;
    plan.outer.edges[2].tag = EdgeTag::Rear;
    MassStack stack = simpleStack(plan, 6, 3.2);
    BuildingSurfaces surf = surfacesFor(stack, 3.0);
    CHECK(surf.levels.size() == 6);
    CHECK(surf.grids.size() == 6);
    CHECK(surf.grids[0].size() == 4);

    // "Balconies on the street wall, floors 1 and up, every bay."
    ElementRegistry reg;
    ElementSpec balcony;
    balcony.kind = ElementKind::Balcony;
    balcony.on.anyTag = false;
    balcony.on.tag = EdgeTag::Street;
    balcony.on.floorMin = 1;
    balcony.depth = 0.9;
    reg.add(balcony);

    std::vector<ElementPlacement> placed = resolveElements(reg, surf, 42);
    CHECK(!placed.empty());
    for (const ElementPlacement& p : placed) {
        CHECK(p.kind == ElementKind::Balcony);
        CHECK(p.tag == EdgeTag::Street);
        CHECK(p.level >= 1);                    // never on the ground floor
    }
    // One per bay on the 24 m wall (8 bays at 3 m), on 5 of the 6 floors.
    CHECK(placed.size() == 8 * 5);
}

TEST_CASE(element_centre_bay_selector_finds_the_doorway_bay) {
    Shape2 plan = rectShape(0, 0, 21, 14);
    plan.outer.edges[0].tag = EdgeTag::Street;
    MassStack stack = simpleStack(plan, 1, 3.4);
    BuildingSurfaces surf = surfacesFor(stack, 3.0);

    ElementRegistry reg;
    ElementSpec portico;
    portico.kind = ElementKind::Portico;
    portico.on.anyTag = false;
    portico.on.tag = EdgeTag::Street;
    portico.on.bays = ElementSelector::Bays::Centre;
    reg.add(portico);

    std::vector<ElementPlacement> placed = resolveElements(reg, surf, 7);
    CHECK(placed.size() == 1);
    CHECK(placed[0].bay == surf.grids[0][0].centreBay());
}

// Pilasters must land BETWEEN bays, never through a window — the thing a
// privately computed bay count made impossible.
TEST_CASE(element_pilasters_land_on_bay_boundaries) {
    Shape2 plan = rectShape(0, 0, 18, 12);
    plan.outer.edges[0].tag = EdgeTag::Street;
    MassStack stack = simpleStack(plan, 1, 3.4);
    BuildingSurfaces surf = surfacesFor(stack, 3.0);
    const BayGrid& g = surf.grids[0][0];
    CHECK(g.bays == 6);

    ElementRegistry reg;
    ElementSpec pil;
    pil.kind = ElementKind::Pilaster;
    pil.on.anyTag = false;
    pil.on.tag = EdgeTag::Street;
    pil.on.bays = ElementSelector::Bays::Boundaries;
    reg.add(pil);
    std::vector<ElementPlacement> placed = resolveElements(reg, surf, 3);
    CHECK(placed.size() == 7);                  // 6 bays -> 7 boundaries

    // Every pilaster sits exactly on a bay edge, and the window in that bay
    // does not straddle it.
    FenestrationStyle st;
    std::vector<Opening> op = fenestrate(g, WallRole::Regular, st, 1, false);
    for (const ElementPlacement& p : placed) {
        CHECK(near(p.t0, p.t1, 1e-12));
        for (const Opening& o : op)
            CHECK(p.t0 <= o.t0 + 1e-9 || p.t0 >= o.t1 - 1e-9);
    }
}

TEST_CASE(element_top_tier_selector_finds_the_crown) {
    Shape2 podium = rectShape(0, 0, 40, 30);
    Shape2 tower = rectShape(8, 6, 32, 24);
    MassStack stack = podiumTower(podium, tower, 3, 12, 3.4);
    BuildingSurfaces surf = surfacesFor(stack, 4.0);
    CHECK(surf.topTier == 1);

    ElementRegistry reg;
    ElementSpec cornice;
    cornice.kind = ElementKind::Cornice;
    cornice.on.tier = ElementSelector::kTopTier;
    cornice.on.bays = ElementSelector::Bays::None;
    reg.add(cornice);
    std::vector<ElementPlacement> placed = resolveElements(reg, surf, 5);
    CHECK(!placed.empty());
    for (const ElementPlacement& p : placed) {
        CHECK(p.tier == 1);
        CHECK(p.bay == -1);                     // a whole-wall element
    }
}

// The registry's real job: adding an element must be appending a row, not
// adding a field and a branch. Eight kinds, one registry, no code change.
TEST_CASE(element_registry_carries_a_whole_dressing_as_data) {
    Shape2 plan = rectShape(0, 0, 24, 16);
    plan.outer.edges[0].tag = EdgeTag::Street;
    plan.outer.edges[2].tag = EdgeTag::Rear;
    MassStack stack = simpleStack(plan, 8, 3.2);
    BuildingSurfaces surf = surfacesFor(stack, 3.0);

    ElementRegistry reg;
    ElementSpec e;
    e.kind = ElementKind::BaseCourse;
    e.on.floorMax = 0;
    e.on.bays = ElementSelector::Bays::None;
    reg.add(e);

    e = ElementSpec{};
    e.kind = ElementKind::Balcony;
    e.on.anyTag = false;
    e.on.tag = EdgeTag::Street;
    e.on.floorMin = 2;
    e.on.bays = ElementSelector::Bays::Alternate;
    reg.add(e);

    e = ElementSpec{};
    e.kind = ElementKind::Cornice;
    e.on.tier = ElementSelector::kTopTier;
    e.on.bays = ElementSelector::Bays::None;
    reg.add(e);

    std::vector<ElementPlacement> placed = resolveElements(reg, surf, 11);
    CHECK(countKind(placed, ElementKind::BaseCourse) == 4);   // 4 walls, floor 0
    CHECK(countKind(placed, ElementKind::Balcony) > 0);
    CHECK(countKind(placed, ElementKind::Cornice) > 0);
    // Balconies only on alternate bays of the street wall, floors 2+.
    for (const ElementPlacement& p : placed)
        if (p.kind == ElementKind::Balcony) {
            CHECK(p.bay % 2 == 0);
            CHECK(p.level >= 2);
            CHECK(p.tag == EdgeTag::Street);
        }
}

// Weights are resolved against a stable per-surface hash, NOT a running dice
// roll — so adding an element to a registry must not move where the others
// landed. Without this, editing a recipe reshuffles the whole building and an
// author can never tune one thing at a time.
TEST_CASE(element_weights_are_stable_when_the_registry_grows) {
    Shape2 plan = rectShape(0, 0, 30, 20);
    plan.outer.edges[0].tag = EdgeTag::Street;
    MassStack stack = simpleStack(plan, 6, 3.2);
    BuildingSurfaces surf = surfacesFor(stack, 3.0);

    ElementSpec occasional;
    occasional.kind = ElementKind::Planter;
    occasional.on.anyTag = false;
    occasional.on.tag = EdgeTag::Street;
    occasional.weight = 0.4;                    // fires on some bays only

    ElementRegistry one;
    one.add(occasional);
    std::vector<ElementPlacement> before = resolveElements(one, surf, 99);
    CHECK(!before.empty());

    ElementRegistry two;
    two.add(occasional);
    ElementSpec extra;
    extra.kind = ElementKind::Sign;
    extra.on.bays = ElementSelector::Bays::Centre;
    two.add(extra);
    std::vector<ElementPlacement> after = resolveElements(two, surf, 99);

    std::vector<ElementPlacement> planters;
    for (const ElementPlacement& p : after)
        if (p.kind == ElementKind::Planter) planters.push_back(p);
    CHECK(planters.size() == before.size());
    for (std::size_t i = 0; i < planters.size(); ++i) {
        CHECK(planters[i].level == before[i].level);
        CHECK(planters[i].edge == before[i].edge);
        CHECK(planters[i].bay == before[i].bay);
    }
}

TEST_CASE(element_resolution_is_deterministic) {
    Shape2 plan = planTemplate(PlanTemplate::L, 30, 22, 4);
    MassStack stack = simpleStack(plan, 5, 3.2);
    BuildingSurfaces surf = surfacesFor(stack, 3.0);
    ElementRegistry reg;
    ElementSpec e;
    e.kind = ElementKind::BayWindow;
    e.weight = 0.5;
    reg.add(e);
    std::vector<ElementPlacement> a = resolveElements(reg, surf, 2024);
    std::vector<ElementPlacement> b = resolveElements(reg, surf, 2024);
    CHECK(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CHECK(a[i].level == b[i].level && a[i].edge == b[i].edge);
        CHECK(a[i].bay == b[i].bay);
    }
}

// --- sided parts (§17.7) ---------------------------------------------------

TEST_CASE(sided_parts_extend_without_renumbering_the_exterior) {
    // Recording which side a surface faces must not change what any existing
    // PartId ordinal means — otherwise every material array shifts under the
    // renderer and interiors become a rewrite rather than a binding.
    const int partCount = 20;
    SidedPart ext{7, Side::Exterior};
    SidedPart inn{7, Side::Interior};
    CHECK(ext.ordinal(partCount) == 7);
    CHECK(inn.ordinal(partCount) == 27);
    for (std::uint8_t p = 0; p < partCount; ++p)
        CHECK((SidedPart{p, Side::Exterior}.ordinal(partCount)) == p);
}
