#ifndef RAYTRACER_ENGINE_PROCGEN_TREE_RECIPE_H
#define RAYTRACER_ENGINE_PROCGEN_TREE_RECIPE_H

// TREE RECIPES (Glenn, 2026-10-17: "Could lua be used to define recipes and then the C++ could ingest those recipes
// and build out those trees?"). A species is DATA -- assets/scripts/tree_recipes.lua, read by the scripting layer
// (scripting/tree_recipes_lua.h) -- and C++ grows it: the grammar expanded (ParametricLSystem), the turtle run,
// the pipe model, the gnarl and the tube skin (tree.h skinTree). No Lua runs per branch.
//
// Variety lives in the recipe's RANGES: every number may be a {lo, hi} the seed draws within, so one recipe is a
// species and its seeds the trees of it ("same type of tree and different trees"). Two kinds of number:
//   params  TreeParams fields by name (droop, lobes, leafSize, ...: tree.h)
//   vars    the recipe's own names, written $name in its rules and axiom (an angle, a falloff) -- substituted per
//           seed before the grammar is compiled, so a rule reads "&($pitch)A(l*$side)"
// A recipe with no rules grows from the built-in grammar (growTree) and its params alone.

#include "tree.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace engine {

// A grammar rule and a drawn number: the tree's and its leaf card's.
struct RecipeRule {
    std::string predecessor;     // "A(l)" or guarded "A(l):l>0.5"
    std::string successor;
    double weight = 1.0;
};
struct RecipeRange {
    std::string name;
    double lo = 0, hi = 0;       // lo == hi: a fixed number
};

// THE LEAF CARD (leaf_card.h): a 2D twig grammar and the leaf shape it draws at every L.
struct LeafCardSpec {
    bool set = false;                     // the recipe has a card (else the generic leaf cluster)
    std::string axiom = "S(5)";
    int iterations = 6;
    std::vector<RecipeRule> rules;
    std::vector<RecipeRange> vars;        // drawn per card variant
    std::string shape = "oval";           // oval | lobed | serrate | needle | leaflet | palmate
    double leafLength = 0.30, leafWidth = 0.14;   // a leaf at L(1), in the grammar's units
    int lobes = 3;                        // lobed: lobes down each side; palmate: lobes in all (a maple's 5 or 7)
    int teeth = 9;                        // serrate: teeth down each side
    double stemWidth = 0.012;             // the twig's stroke
    double cup = 0.5;                     // how far a leaf's normal tips across it (0: flat)
    double shadeJitter = 0.18;            // leaf-to-leaf brightness variation
    double tipBright = 0.0;               // the new growth: leaves lighter toward the card's far end (0 none)
    int variants = 2;                     // cards a side of the atlas
    // COLOUR in the card (Glenn, 2026-10-17: "more color"): the atlas RGB is each part's brightness times its tint,
    // and the tree's vertex colour multiplies that. White (the default) leaves the vertex colour the whole colour;
    // a card with blossoms on it tints them apart from its leaves and its twig.
    Vec3 leafTint{1, 1, 1}, stemTint{1, 1, 1};
    // THE FLOWER the card grammar draws at every W(scale), facing along the turtle:
    //   blossom  `petals` round petals, each notched at its tip by `notch` (sakura, apple)
    //   daisy    `petals` narrow rays round a disc (euryops, cosmos, marguerite)
    //   floret   one small two-lipped flower: stacked up a stem they make a spike (salvia, lavender)
    // `eye` is the centre's share of the radius, coloured eyeTint (a sakura's pink eye, a daisy's disc).
    struct Flower {
        bool set = false;
        std::string shape = "blossom";
        double size = 0.08;               // across, at W(1), in the grammar's units
        int petals = 5;
        double notch = 0.2, eye = 0.25;
        Vec3 tint{1, 1, 1}, eyeTint{1, 1, 1};
    } flower;
};

// THE BRANCH CARDS (branch_cards.h; Glenn, 2026-10-18: "When we get to small enough branches that we instead just use
// some kind of card to stand in for those branches and clusters of leaves"): the recipe's `branch_cards` block. Its
// trees model the wood down to sprigs of `sprig` metres (TreeParams::sprigSize), built by need (woodEdge, ringAngle),
// and wear a card for every sprig, baked from `pictures` of its own sprigs grown whole.
struct BranchCardSpec {
    bool set = false;
    double sprig = 0.8;              // a sprig's reach (m): smaller is modelled wood, card-sized and under a card
    double woodEdge = 0.05;          // the wood's sides about this far apart (m)
    double ringAngle = 12.0;         // ...and a ring every this many degrees of bend
    int pictures = 8;                // sprigs baked
    int views = 3;                   // ...each seen from this many sides round its axis: a card each (the atlas's cells)
    int cell = 256;                  // a cell's side (px)
    double fold = 25.0;              // each half of a card lifts this many degrees off flat about its spine
    double bend = 0.55;              // its normals bent this far toward out of the crown (the crown shades as a volume)
    // TWIG SEGMENTS (a species with near leaves): segments along the fine branching, not sprig pictures
    bool twigs = true;
    int variants = 2;                // of each segment (leafy, leafy Y, bare, bare Y)
    double fillRadius = 0.025;       // leafy segments sprout along the modelled limbs thinner than this (m)...
    double fillSpacing = 0.35;       // ...about this far apart (0: none)
    int nearLeaves = 6000;           // the hero's leaves one by one (branch_cards.h wearNearLeaves)...
    int nearTwigTris = 9000;         // ...and its fine twigs' bark at most (triangles)
    int heroWoodTris = 8000;         // the hero's trunk and limbs at most (tree_lods.h heroBarkBudget)
};

// THE NEEDLES of a conifer (branch_cards.h bakeNeedleCards / wearNeedles; Glenn, 2026-10-18, with a photo of a fir, a
// pine and a spruce sprig: "the spruce leaves look pretty bad"): its needle shoots drawn from needle geometry and
// planted along its fine branching.
//   brush  a spruce: needles all round the twig, swept forward, a fishbone of side shoots in the branch's plane
//   flat   a fir: needles in two rows to the sides, flat sprays
//   tuft   a pine: long needles in bundles crowded toward the twig ends, the inner twig bare
struct NeedleSpec {
    bool set = false;
    std::string kind = "brush";
    double length = 0.02;            // a needle (m)
    double width = 0.0012;           // ...across (m)
    double density = 3.0;            // needles a centimetre of shoot
    double forward = 50.0;           // a needle's angle off the shoot, toward its tip (deg)
    double shootLength = 0.16;       // a side shoot (m), shortening toward the twig's end
    int sideShoots = 4;              // side shoots each twig puts out (each side, in its plane)
    double sideAngle = 50.0;         // ...off it (deg)
    double reach = 1.0;              // the share of a twig, from its end, that carries needles (a pine's: its last 0.4)
    double hang = 0.0;               // the share of side shoots that HANG from the branch (a spruce's curtains of
                                     //   pendant shoots, filling the gap to the whorl below)
    double limbRadius = 0.0;         // the modelled limbs this thin carry needles and side shoots too (a spruce's whole
                                     //   branches are clothed; a pine's bare inside: 0)
    double frondSpacing = 0.0;       // FRONDS (Glenn, 2026-10-18, of the spruce and pine: "they need more volume in the
                                     //   middle because you can see through them"): along the modelled branches (not
                                     //   the trunk, not the finest), a frond each side every this many metres --
                                     //   a big card of a whole needled sub-branch, some hanging (0: none)
    double frondLength = 0.55;       // ...each (m)
    Vec3 color{0.07, 0.17, 0.07}, colorHi{0.10, 0.22, 0.09};
    Vec3 tipColor{0.20, 0.36, 0.16};  // this year's growth at the shoot ends
    Vec3 twigColor{0.45, 0.30, 0.18};
};

// AN AGE of a species (Glenn, 2026-10-18: "They all look like young trees ... Where's the 300 year old hero tree that
// could sit on a hill ... we need height and oldness in our variety"): the recipe grown older -- its own axiom (a
// longer first stretch: the grammar's length thresholds then give more orders of branching, a wider, taller crown),
// iterations, params and vars laid OVER the recipe's (a name the age gives wins; the rest are the recipe's), and the
// branch cards' budgets for a tree that size.
struct TreeAge {
    std::string axiom;               // empty: the recipe's
    std::string axiomPrefix;         // ...or this put before whatever axiom the tree has (a form's: a stand's clear bole,
                                     //   "F(3)", under every age's trunk -- an open-grown tree's draw untouched)
    int iterations = -1;             // < 0: the recipe's
    std::vector<RecipeRange> params, vars;
    double sprig = -1, nearLeaves = -1, nearTwigTris = -1;   // < 0: the recipe's branch_cards
    // ...and numbers SCALED rather than set (`scale = { params = {...}, vars = {...} }`, each a factor): a FORM's way
    // (below), so one form fits every age -- a stand oak's crown 0.6 of whatever crown its age grows
    std::vector<RecipeRange> paramsScale, varsScale;
};

struct TreeRecipe {
    std::string name;
    bool shrub = false;              // kind = "shrub" (or "blades"): a shrub or a perennial (knee to head height), not a tree
    std::string axiom = "A(1.6)";   // module literals once its $vars are substituted
    int iterations = 5;
    using Rule = RecipeRule;
    std::vector<Rule> rules;         // empty: the built-in grammar (growTree) from the params
    using Range = RecipeRange;
    std::vector<Range> params;       // TreeParams fields
    std::vector<Range> vars;         // $names in the rules and the axiom
    std::string bark = "oak";        // the bark maps' style (tree.h barkStyleFromName)
    Vec3 barkColor{0.33, 0.24, 0.16}, barkColorHi{0.33, 0.24, 0.16};   // drawn between
    Vec3 leafColor{0.19, 0.44, 0.13}, leafColorHi{0.19, 0.44, 0.13};
    // LEAF PALETTES: a tree's cards take their colour from a palette, blended across the crown in patches of
    // paletteScale metres (one limb red, the next orange: an autumn maple). Several palettes: each seed picks one.
    // Empty: leafColor..leafColorHi, one colour a tree.
    std::vector<std::vector<Vec3>> leafPalettes;
    LeafCardSpec card;
    // NEAR FOLIAGE (foliage_kit.h): the strand -- one shoot drawn flat, its own card grammar -- and the spray the
    // shoots are built into. A tree grown with foliage = 1 wears these instead of the card.
    LeafCardSpec strand;
    SpraySpec spray;
    LeafKitSpec leafKit;             // foliage = 2: every leaf a blade, blossoms cups (wearing the card's leaf and flower)
    BladeSpec blades;                // kind = "blades": no wood, a clump of strap leaves (flax, rushes, grass tufts)
    FrondSpec fronds;                // a palm's crown (params.foliage = 4)
    BranchCardSpec branchCards;      // branch_cards = {...}: the far side of the near kit (branch_cards.h)
    NeedleSpec needles;              // needles = {...}: a conifer's foliage at every level (branch_cards.h)
    std::map<std::string, TreeAge> ages;   // ages = { young = {...}, old = {...}, ... } (agedRecipe)
    // FORMS (Glenn, 2026-10-18: forests on the island; formedRecipe): how the tree grows WHERE it grows, on top of its
    // age -- `stand`, among other trees: its lower limbs shaded off (pruneBelow, a clear bole), its crown narrow and
    // high (scaled) -- as against the open-grown tree a recipe is
    std::map<std::string, TreeAge> forms;
};

class TreeRecipeBook {
public:
    void set(TreeRecipe r);
    const TreeRecipe* find(const std::string& name) const;
    std::size_t size() const { return recipes_.size(); }
    std::vector<std::string> names() const;
private:
    std::vector<TreeRecipe> recipes_;
};

// Set a TreeParams field by its name (the recipe's and a level's tree block's spelling); false for an unknown name.
bool setTreeParam(TreeParams& p, const std::string& name, double value);
// Every name setTreeParam knows (the reader rejects others: a typo would otherwise silently grow the default).
const std::vector<std::string>& treeParamNames();

// The tree a recipe's ranges give for `seed`: its TreeParams and its vars' values.
TreeParams drawTreeParams(const TreeRecipe& r, uint32_t seed, std::map<std::string, double>* vars = nullptr);

// `text` with every $name replaced by its value in `vars`; `missing` gets a name with no value.
std::string substituteVars(const std::string& text, const std::map<std::string, double>& vars, std::string* missing = nullptr);

// The recipe at `age` (TreeRecipe::ages): its axiom, iterations, params and vars laid over, its branch cards' budgets.
// An age the recipe does not name (or "") is the recipe as it is.
TreeRecipe agedRecipe(const TreeRecipe& r, const std::string& age);
// ...and in `form` (TreeRecipe::forms: "stand"), laid over the same way after the age (its scales scale the aged
// numbers). A form the recipe does not name (or "") is the recipe as it is.
TreeRecipe formedRecipe(const TreeRecipe& r, const std::string& form);

// Grow one tree of the recipe. `overrides` (a level's per-tree knobs: the lab's gnarl row) win over the draw.
TreeMesh growRecipeTree(const TreeRecipe& r, uint32_t seed, const std::map<std::string, double>& overrides = {});

}  // namespace engine

#endif
