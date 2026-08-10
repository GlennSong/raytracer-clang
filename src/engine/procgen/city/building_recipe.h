#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_BUILDING_RECIPE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_BUILDING_RECIPE_H

#include "facade_plan.h"
#include "lot_program.h"
#include "mass_stack.h"
#include "material_set.h"
#include "plan_grammar.h"
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

// The recipe library as DATA (docs/lot-system-plan.md §12 "the recipe port").
//
// architect.cpp carries 55 recipe FUNCTIONS, and the plan's audit found they
// all have the same five-slot shape: a floor range, a ground-floor mode, a
// cladding bundle, a handful of element flags, a place type. That uniformity is
// the evidence they are data typed as code. Ported here they become rows, and
// the audit's conclusion holds: 46 port directly, 5 needed several masses per
// tier, 6 needed per-band floor heights, 8 collapsed into others with different
// numbers, and 2 were never buildings at all (pocket_park and plaza are lot
// programs, and live in lot_program.h).
//
// TWO THINGS THE PORT DELIBERATELY FIXES:
//
//   `coreness` was copy-pasted across six tower recipes with a different
//   constant each. That is a HEIGHT MODEL and it belongs above the tables — it
//   now lives in maxStoreysFor() (§17.2) and the architect consults it once.
//
//   RecipeCtx let a recipe read shortSide/area/roomy, so a recipe knew about
//   parcels. That INVERTS: the program reads the lot and picks a recipe that
//   fits, so a recipe never asks how big its site is. A recipe here has no
//   access to the lot at all — only to the envelope it was handed.

// How a recipe's mass is put together. Each maps onto a MassStack shape from
// §5, so this is a configuration choice rather than a code path.
enum class MassKind : std::uint8_t {
    Simple,        // one plan, one tier
    PodiumTower,   // a broad base with a smaller shaft
    WeddingCake,   // stepped setbacks
    Gherkin,       // one plan, a swell profile
    Pyramid,       // one plan, a taper
    Arch,          // two legs merging into one crown
    MultiMass,     // several masses on one lot: office park, church, hospital
};

const char* massKindName(MassKind m);

// A run of floors at one height. SIX of the existing recipes fight for this by
// overloading floorHeight/groundHeight — a hotel's lobby is not its bedroom
// floor, and a factory's shed is not its office block.
struct RecipeBand {
    Real height = 3.2;
    int minCount = 1, maxCount = 1;
};

enum class RoofKind : std::uint8_t {
    Flat, Parapet, Gable, Hip, Mansard, Sawtooth, Dome, Steeple, Spire,
};

struct BuildingRecipe {
    std::string name;
    std::string placeType;

    // Plan: a weighted choice of templates, so one recipe is not one shape.
    std::vector<PlanTemplate> plans;

    MassKind mass = MassKind::Simple;
    std::vector<RecipeBand> bands;      // in order, ground first
    int minStoreys = 1, maxStoreys = 3;

    // How many separate masses, for MultiMass. The five recipes whose NAME
    // promises a complex (office_park, strip_mall, church, market_hall,
    // hospital) but which emit one prism today.
    int minMasses = 1, maxMasses = 1;

    RoofKind roof = RoofKind::Flat;
    Real roofPitchMin = 0.35, roofPitchMax = 0.55;

    Character character = Character::Suburban;
    Era era = Era::PostWar;

    bool groundRetail = false;
    // Cantilever budget (§17.4). Ordinary buildings keep containment; a recipe
    // opts into an overhang deliberately.
    SupportRule support;

    // The dressing, as data (§6.2). Adding a feature is appending a row.
    ElementRegistry elements;

    Real weight = 1.0;
};

// Every ported recipe.
struct RecipeBook {
    std::vector<BuildingRecipe> recipes;

    const BuildingRecipe* byName(const std::string& name) const;
    // Recipes credible for a program on a lot this tall. The PROGRAM does the
    // asking; the recipe never inspects the lot.
    std::vector<int> forProgram(const LotProgram& program, int storeysAvailable) const;
    int pick(const LotProgram& program, int storeysAvailable,
             std::uint32_t seed) const;
};

RecipeBook stockRecipes();

// --- assembly --------------------------------------------------------------

// How the plan actually turned out. A recipe naming an L and getting a plain
// rectangle used to be SILENT, so a whole city could come out square with
// nothing in the output saying so — the author's only signal was looking at it.
// Recording the outcome is what makes that measurable.
enum class PlanFit : std::uint8_t {
    Template,    // the template survived intact
    Clipped,     // it survived, but the envelope trimmed it
    Envelope,    // it was discarded and the plan is the bare envelope
};

const char* planFitName(PlanFit f);

// Everything a built lot produces. This is the type that proves the layers
// compose: L0 shapes, L1 plan/stack/facade, L3 site, and the palette.
struct BuiltBuilding {
    MassStack stack;
    BuildingSurfaces surfaces;
    std::vector<ElementPlacement> placements;
    BuildingMaterials materials;
    std::string recipeName;
    int storeys = 0;
    PlanTemplate plan = PlanTemplate::Bar;   // what the recipe asked for
    PlanFit planFit = PlanFit::Template;     // what it got
};

// Build a recipe inside a site's building envelope. `tags` supplies only the
// height cap and the coreness — never the lot's dimensions to the recipe.
// `tags.maxStoreys` is the WHOLE height budget: a caller that also has a
// program should pass min(plate cap, program.maxStoreys), because the
// program's declared range binds as much as the plate does.
// Deterministic for `seed`.
BuiltBuilding buildFromRecipe(const BuildingRecipe& recipe,
                              const Shape2& envelope, const LotTags& tags,
                              const PaletteLibrary& palettes,
                              std::uint32_t districtSeed, std::uint32_t seed);

}  // namespace engine

#endif
