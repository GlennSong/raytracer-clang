#include "test_framework.h"

#include "../src/engine/procgen/city/building_recipe.h"
#include "../src/engine/procgen/city/site_plan.h"
#include <cmath>
#include <set>
#include <string>

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
}  // namespace

// --- palettes --------------------------------------------------------------

TEST_CASE(palette_sets_are_coherent_bundles) {
    PaletteLibrary lib = stockPalettes();
    CHECK(lib.sets.size() >= 15);
    CHECK(lib.character.size() == lib.sets.size());
    CHECK(lib.era.size() == lib.sets.size());
    for (const MaterialSet& m : lib.sets) {
        CHECK(!m.name.empty());
        CHECK(m.minStoreys >= 1);
        CHECK(m.maxStoreys >= m.minStoreys);
        // The window belongs to the palette, not to a separate roll.
        CHECK(m.window.preferredBay > 1.0);
        CHECK(m.window.head > m.window.sill);
    }
}

// ADR-0040: cladding follows structure follows height. A timber cottage palette
// must never land on a forty-storey tower.
TEST_CASE(palette_height_band_is_respected) {
    PaletteLibrary lib = stockPalettes();
    for (int i : lib.candidates(Character::Suburban, 2)) {
        CHECK(lib.sets[i].minStoreys <= 2 && lib.sets[i].maxStoreys >= 2);
    }
    for (int i : lib.candidates(Character::Downtown, 40)) {
        CHECK(lib.sets[i].maxStoreys >= 40);
        CHECK(lib.sets[i].style != FacadeStyle::Wood);
    }
    // A tall building always finds SOMETHING to wear, even if its character
    // has nothing that tall.
    CHECK(!lib.candidates(Character::Suburban, 40).empty());
}

// A street must read as ONE PLACE with variation on it. A per-building draw
// from the whole library is the "same-y but random" failure from both
// directions at once.
TEST_CASE(palette_a_street_shares_a_family) {
    PaletteLibrary lib = stockPalettes();
    const std::uint32_t district = 4242;
    std::set<int> chosen;
    int matchesFamily = 0;
    const int n = 40;
    for (int b = 0; b < n; ++b) {
        const int s = lib.pick(Character::Terraced, Era::Victorian, 3, district,
                               b * 7919u + 1);
        CHECK(s >= 0);
        chosen.insert(s);
    }
    const int family =
        lib.districtFamily(Character::Terraced, Era::Victorian, 3, district);
    CHECK(family >= 0);
    for (int b = 0; b < n; ++b)
        if (lib.pick(Character::Terraced, Era::Victorian, 3, district,
                     b * 7919u + 1) == family)
            ++matchesFamily;
    // Most of the street wears the district's family; a minority does not.
    CHECK(matchesFamily > n / 2);
    CHECK(matchesFamily < n);
    // Different districts must reach different families — otherwise the whole
    // city wears one palette, which is what a raw-seeded RNG actually did here.
    bool differs = false;
    for (std::uint32_t d = 1; d < 40; ++d)
        if (lib.districtFamily(Character::Terraced, Era::Victorian, 3, d) != family)
            differs = true;
    CHECK(differs);
}

TEST_CASE(palette_perturbation_varies_colour_but_never_parts) {
    PaletteLibrary lib = stockPalettes();
    const MaterialSet& base = lib.sets[3];
    MaterialSet a = perturb(base, 11), b = perturb(base, 12);
    // Parts are what make the bundle coherent — they must never move.
    CHECK(a.wall == base.wall && a.roof == base.roof && a.trim == base.trim);
    CHECK(b.wall == base.wall);
    CHECK(a.style == base.style);
    // Colours shift, but stay in the family.
    CHECK((a.wallColor - b.wallColor).length() > 1e-6);
    CHECK((a.wallColor - base.wallColor).length() < 0.2);
}

// One family per mass, with a legitimate SECOND treatment at the base: the
// glass tower on a stone podium.
TEST_CASE(palette_a_podium_may_reclad_once) {
    PaletteLibrary lib = stockPalettes();
    BuildingMaterials m = assignMaterials(lib, Character::Downtown, Era::Modern,
                                          {3, 20}, 7, 9);
    CHECK(m.perTier.size() == 2);
    // The base is masonry-ish; the shaft is whatever downtown chose.
    CHECK(m.perTier[0].style != FacadeStyle::GlassCurtain ||
          m.perTier[0].name == m.perTier[1].name);
    CHECK(m.forTier(5).name == m.perTier[1].name);   // clamps past the end
}

TEST_CASE(palette_part_mapping_honours_sides) {
    PaletteLibrary lib = stockPalettes();
    BuildingMaterials m = assignMaterials(lib, Character::Terraced, Era::Victorian,
                                          {4}, 3, 5);
    SidedPart ext = m.partFor(0, PartId::Wall, Side::Exterior);
    SidedPart inn = m.partFor(0, PartId::Wall, Side::Interior);
    CHECK(ext.part == inn.part);                    // the same material
    CHECK(ext.side != inn.side);                    // on different sides
    CHECK(ext.ordinal(static_cast<int>(PartId::Count)) !=
          inn.ordinal(static_cast<int>(PartId::Count)));
}

// --- the recipe book -------------------------------------------------------

TEST_CASE(recipes_are_data_rows_with_the_five_slots_filled) {
    RecipeBook book = stockRecipes();
    CHECK(book.recipes.size() >= 30);
    std::set<std::string> names;
    for (const LotRecipe& r : book.recipes) {
        CHECK(!r.name.empty());
        CHECK(!r.placeType.empty());
        CHECK(names.insert(r.name).second);         // no duplicate names
        CHECK(r.minStoreys >= 1);
        CHECK(r.maxStoreys >= r.minStoreys);
        CHECK(!r.bands.empty());
        CHECK(r.weight > 0);
        for (const RecipeBand& b : r.bands) {
            CHECK(b.height > 1.5);
            CHECK(b.maxCount >= b.minCount);
        }
    }
}

// The plan's audit: five recipes whose NAME promises a complex emitted one
// prism. They now declare several masses.
TEST_CASE(recipes_that_promise_a_complex_declare_several_masses) {
    RecipeBook book = stockRecipes();
    for (const char* n : {"office_park", "strip_mall", "church", "market_hall",
                          "hospital"}) {
        const LotRecipe* r = book.byName(n);
        CHECK(r != nullptr);
        CHECK(r->mass == MassKind::MultiMass);
        CHECK(r->maxMasses >= 2);
    }
}

// Six recipes were overloading one floor-height number. They now declare bands.
TEST_CASE(recipes_that_need_per_band_heights_declare_them) {
    RecipeBook book = stockRecipes();
    for (const char* n : {"hotel", "factory", "warehouse", "capitol", "library",
                          "museum"}) {
        const LotRecipe* r = book.byName(n);
        CHECK(r != nullptr);
        CHECK(r->bands.size() >= 2);
        // The bands genuinely differ — otherwise declaring them changes nothing.
        bool differs = false;
        for (std::size_t i = 1; i < r->bands.size(); ++i)
            if (std::fabs(r->bands[i].height - r->bands[0].height) > 0.3)
                differs = true;
        CHECK(differs);
    }
}

// A recipe must never ask how big its site is: the program reads the lot and
// picks a recipe that fits. The type simply has no lot in it.
TEST_CASE(recipes_never_see_the_lot) {
    RecipeBook book = stockRecipes();
    LotProgram villa;
    for (const LotProgram& p : residentialPrograms().programs)
        if (p.name == "villa") villa = p;
    // forProgram is answered from the PROGRAM and the height budget alone.
    std::vector<int> a = book.forProgram(villa, 6);
    std::vector<int> b = book.forProgram(villa, 6);
    CHECK(a == b);
    CHECK(!a.empty());
    // A tighter height budget can only ever shrink the candidate list.
    std::vector<int> tight = book.forProgram(villa, 2);
    CHECK(tight.size() <= a.size());
}

TEST_CASE(recipes_pick_deterministically_and_within_the_height_budget) {
    RecipeBook book = stockRecipes();
    LotProgram office;
    for (const LotProgram& p : commercialPrograms().programs)
        if (p.name == "office_block") office = p;
    for (std::uint32_t s = 1; s <= 25; ++s) {
        const int a = book.pick(office, 10, s);
        const int b = book.pick(office, 10, s);
        CHECK(a == b);
        if (a >= 0) CHECK(book.recipes[a].minStoreys <= 10);
    }
}

// --- assembly: the proof the layers compose --------------------------------

TEST_CASE(built_building_composes_every_layer) {
    PaletteLibrary lib = stockPalettes();
    RecipeBook book = stockRecipes();
    Shape2 lot = streetLot(30, 40);
    LotTags tags = measureLot(lot, {}, StreetClass::Street, true, 0.4);
    ProgramSet set = residentialPrograms();
    const int pi = set.pick(tags, 3);
    CHECK(pi >= 0);
    SitePlan site = layoutSite(lot, tags, set.programs[pi], 3);
    CHECK(site.buildEnvelope.outer.size() >= 3);

    const int ri = book.pick(set.programs[pi],
                             std::max(1, static_cast<int>(tags.maxStoreys)), 3);
    CHECK(ri >= 0);
    BuiltBuilding b = buildFromRecipe(book.recipes[ri], site.buildEnvelope, tags,
                                      lib, 99, 3);
    CHECK(b.storeys >= 1);
    CHECK(!b.stack.tiers.empty());
    CHECK(!b.surfaces.levels.empty());
    CHECK(!b.materials.perTier.empty());
    // Levels stack without gaps.
    for (std::size_t i = 1; i < b.surfaces.levels.size(); ++i)
        CHECK(near(b.surfaces.levels[i].y0, b.surfaces.levels[i - 1].y1, 1e-9));
}

// Every recipe must build a valid building at a range of envelope sizes — the
// port is only real if all 30+ rows actually produce geometry.
TEST_CASE(built_every_recipe_produces_a_valid_building) {
    PaletteLibrary lib = stockPalettes();
    RecipeBook book = stockRecipes();
    PlanQuality quality;
    for (const LotRecipe& r : book.recipes) {
        for (Real w : {Real(22), Real(48), Real(90)}) {
            Shape2 env = rectShape(0, 0, w, w * 0.72);
            for (Edge2& e : env.outer.edges) e.tag = EdgeTag::Side;
            env.outer.edges[0].tag = EdgeTag::Street;
            LotTags tags;
            tags.area = area(env);
            inscribedRect(env, tags.inscribedW, tags.inscribedD);
            tags.maxStoreys = maxStoreysFor(std::min(tags.inscribedW, tags.inscribedD),
                                            tags.inscribedW * tags.inscribedD);
            BuiltBuilding b = buildFromRecipe(r, env, tags, lib, 5, 17);
            CHECK(b.storeys >= 1);
            CHECK(!b.stack.tiers.empty());
            CHECK(!b.surfaces.levels.empty());
            for (const Level& lv : b.surfaces.levels) {
                CHECK(!lv.plans.empty());
                CHECK(lv.y1 > lv.y0);
                for (const Shape2& p : lv.plans) CHECK(area(p) > 0.5);
            }
            // The stack's height is the sum of its levels, with no gaps.
            for (std::size_t i = 1; i < b.surfaces.levels.size(); ++i)
                CHECK(near(b.surfaces.levels[i].y0, b.surfaces.levels[i - 1].y1, 1e-6));
        }
    }
}

// §17.2: one height model above the tables. A big recipe on a small plate must
// be capped by the PLATE, not by its own copy-pasted constant.
TEST_CASE(built_height_is_capped_by_the_plate) {
    PaletteLibrary lib = stockPalettes();
    RecipeBook book = stockRecipes();
    const LotRecipe* tower = book.byName("glass_tower");
    CHECK(tower != nullptr);
    CHECK(tower->minStoreys >= 20);

    // A generous plate carries a tall tower.
    Shape2 big = rectShape(0, 0, 46, 44);
    LotTags bigTags;
    inscribedRect(big, bigTags.inscribedW, bigTags.inscribedD);
    bigTags.maxStoreys = maxStoreysFor(std::min(bigTags.inscribedW, bigTags.inscribedD),
                                       bigTags.inscribedW * bigTags.inscribedD);
    BuiltBuilding tall = buildFromRecipe(*tower, big, bigTags, lib, 1, 2);

    // A small plate cannot, whatever the recipe wants.
    Shape2 small = rectShape(0, 0, 14, 13);
    LotTags smallTags;
    inscribedRect(small, smallTags.inscribedW, smallTags.inscribedD);
    smallTags.maxStoreys = maxStoreysFor(
        std::min(smallTags.inscribedW, smallTags.inscribedD),
        smallTags.inscribedW * smallTags.inscribedD);
    BuiltBuilding squat = buildFromRecipe(*tower, small, smallTags, lib, 1, 2);

    CHECK(squat.storeys < tall.storeys);
    CHECK(squat.storeys <= static_cast<int>(smallTags.maxStoreys) + 1);
}

TEST_CASE(built_multimass_recipes_really_emit_several_masses) {
    PaletteLibrary lib = stockPalettes();
    RecipeBook book = stockRecipes();
    const LotRecipe* park = book.byName("office_park");
    CHECK(park != nullptr);
    Shape2 env = rectShape(0, 0, 120, 70);
    for (Edge2& e : env.outer.edges) e.tag = EdgeTag::Side;
    env.outer.edges[0].tag = EdgeTag::Street;
    LotTags tags;
    inscribedRect(env, tags.inscribedW, tags.inscribedD);
    tags.maxStoreys = maxStoreysFor(std::min(tags.inscribedW, tags.inscribedD),
                                    tags.inscribedW * tags.inscribedD);
    bool sawSeveral = false;
    for (std::uint32_t s = 1; s <= 8; ++s) {
        BuiltBuilding b = buildFromRecipe(*park, env, tags, lib, 3, s);
        if (!b.surfaces.levels.empty() && b.surfaces.levels[0].plans.size() > 1)
            sawSeveral = true;
    }
    CHECK(sawSeveral);
}

TEST_CASE(built_buildings_are_deterministic) {
    PaletteLibrary lib = stockPalettes();
    RecipeBook book = stockRecipes();
    Shape2 env = rectShape(0, 0, 34, 26);
    env.outer.edges[0].tag = EdgeTag::Street;
    LotTags tags;
    inscribedRect(env, tags.inscribedW, tags.inscribedD);
    tags.maxStoreys = 20;
    for (const LotRecipe& r : book.recipes) {
        BuiltBuilding a = buildFromRecipe(r, env, tags, lib, 8, 44);
        BuiltBuilding b = buildFromRecipe(r, env, tags, lib, 8, 44);
        CHECK(a.storeys == b.storeys);
        CHECK(a.surfaces.levels.size() == b.surfaces.levels.size());
        CHECK(a.placements.size() == b.placements.size());
        CHECK(a.materials.perTier.size() == b.materials.perTier.size());
    }
}

// The registry really drives the dressing: a recipe with balconies places them,
// one without does not — with no code difference between the two.
TEST_CASE(built_element_registry_drives_the_dressing) {
    PaletteLibrary lib = stockPalettes();
    RecipeBook book = stockRecipes();
    Shape2 env = rectShape(0, 0, 30, 22);
    env.outer.edges[0].tag = EdgeTag::Street;
    LotTags tags;
    inscribedRect(env, tags.inscribedW, tags.inscribedD);
    tags.maxStoreys = 30;

    const LotRecipe* walkup = book.byName("walkup");
    const LotRecipe* shed = book.byName("warehouse");
    CHECK(walkup && shed);
    BuiltBuilding a = buildFromRecipe(*walkup, env, tags, lib, 2, 6);
    BuiltBuilding b = buildFromRecipe(*shed, env, tags, lib, 2, 6);
    auto has = [](const BuiltBuilding& x, ElementKind k) {
        for (const ElementPlacement& p : x.placements) if (p.kind == k) return true;
        return false;
    };
    CHECK(has(a, ElementKind::Balcony));
    CHECK(!has(b, ElementKind::Balcony));
    CHECK(has(a, ElementKind::Opening) && has(b, ElementKind::Opening));
}

// The first drawn sheet handed a residential walkup an industrial WORKS shed,
// because "a recipe that fits" meant only "a recipe of roughly the right
// height". A program names the buildings that belong on it.
TEST_CASE(recipes_match_the_program_not_just_its_height) {
    RecipeBook book = stockRecipes();
    auto namesFor = [&](const LotProgram& p, int storeys) {
        std::set<std::string> out;
        for (int i : book.forProgram(p, storeys)) out.insert(book.recipes[i].name);
        return out;
    };
    for (const LotProgram& p : residentialPrograms().programs) {
        CHECK(!p.recipes.empty());
        std::set<std::string> got = namesFor(p, 12);
        CHECK(!got.empty());
        // Nothing industrial ever lands on a home.
        CHECK(got.count("works") == 0);
        CHECK(got.count("factory") == 0);
        CHECK(got.count("warehouse") == 0);
        CHECK(got.count("cathedral") == 0);
        // ... and every name the program asked for is a real recipe.
        for (const std::string& want : p.recipes)
            CHECK(book.byName(want) != nullptr);
    }
    for (const LotProgram& p : downtownPrograms().programs)
        for (const std::string& want : p.recipes)
            CHECK(book.byName(want) != nullptr);
    for (const LotProgram& p : rimPrograms().programs)
        for (const std::string& want : p.recipes)
            CHECK(book.byName(want) != nullptr);
    for (const LotProgram& p : commercialPrograms().programs)
        for (const std::string& want : p.recipes)
            CHECK(book.byName(want) != nullptr);

    // A cathedral program builds a cathedral or a church, never a shopfront.
    for (const LotProgram& p : downtownPrograms().programs)
        if (p.name == "cathedral") {
            std::set<std::string> got = namesFor(p, 6);
            CHECK(got.count("shopfront") == 0);
            CHECK(got.count("cathedral") + got.count("church") > 0);
        }
}

// A program's declared height range binds as hard as the plate does. A walkup
// that says 3-6 storeys must not come out 14 storeys tall because the recipe it
// named happens to reach that high — a brief that can be ignored is not a brief.
TEST_CASE(recipes_respect_the_programs_height_range) {
    RecipeBook book = stockRecipes();
    LotProgram walkup;
    for (const LotProgram& p : residentialPrograms().programs)
        if (p.name == "walkup") walkup = p;
    CHECK(walkup.maxStoreys <= 6);
    for (int i : book.forProgram(walkup, 60))
        CHECK(book.recipes[i].minStoreys <= walkup.maxStoreys);

    // And the built height honours the budget it is handed.
    PaletteLibrary lib = stockPalettes();
    Shape2 env = rectShape(0, 0, 30, 24);
    env.outer.edges[0].tag = EdgeTag::Street;
    LotTags tags;
    inscribedRect(env, tags.inscribedW, tags.inscribedD);
    tags.maxStoreys = walkup.maxStoreys;
    for (int i : book.forProgram(walkup, walkup.maxStoreys)) {
        BuiltBuilding b = buildFromRecipe(book.recipes[i], env, tags, lib, 2, 31);
        CHECK(b.storeys <= walkup.maxStoreys);
    }
}
