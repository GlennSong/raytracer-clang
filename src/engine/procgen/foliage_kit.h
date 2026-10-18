#ifndef RAYTRACER_ENGINE_PROCGEN_FOLIAGE_KIT_H
#define RAYTRACER_ENGINE_PROCGEN_FOLIAGE_KIT_H

// THE FOLIAGE KIT -- near foliage as small geometry, not crossed cards (Glenn, 2026-10-17: "the spruce leaves are
// clustered but they're flat ... have a 2d texture image of a strand of spruce blades and then put them around a
// cylinder to create one of the leaves and then cluster those using geometry ... Make a variety of those so we can
// instance different ones and create a tree that way"; the crossed cards "create weird slits and gaps ... could be
// okay mid-tier lods but not close up").
//
//   STRAND  a texture: one shoot drawn flat -- its twig down the middle, needles both sides, a bud at the tip -- by the
//           recipe's `strand` card grammar (leaf_card.h), an atlas of a few of them
//   SHOOT   geometry: `fins` planes through the shoot's own axis, each wearing a strand -- fletching, not a tube: a
//           shoot reads full from the side and end-on alike -- bent down a little along its length
//   SPRAY   geometry: a main shoot with side shoots fanned off it alternately, mostly flat, shortening toward the
//           tip, a few lifted out of the fan for depth; `variants` of them per species, built once
//   TREE    every leafy node of the skeleton takes a spray variant, laid along its twig with the fan level, turned
//           and scaled a little (TreeParams::foliage = 1)
//
// A spray is built in its own frame -- +Y along the main shoot, +X across the fan, +Z the fan's up -- at unit length.

#include "../../renderer/renderer.h"   // RenderMesh
#include "../../rt_math.h"             // Vec3

#include <cstdint>
#include <vector>

namespace engine {

struct SpraySpec {
    bool set = false;
    int sideShoots = 7;          // side shoots off the main one, alternating sides
    double sideAngle = 50.0;     // their angle off the main shoot (deg)
    double sideLength = 0.55;    // the first (longest) side shoot, a fraction of the main shoot
    double sideFalloff = 0.88;   // each next side shoot this x the last
    double lift = 14.0;          // side shoots tilted out of the fan by up to this (deg): depth
    int fins = 3;                // strand planes through each shoot's axis
    double aspect = 0.40;        // a strand's width over its length (the atlas tile's aspect)
    double droop = 0.25;         // the shoots bend down this much (rad) over their length
    int variants = 6;            // sprays built per species
    int perNode = 1;             // sprays on each leafy node of the tree
    int budget = 2500;           // sprays a tree at most (thinned evenly over the crown, as cards are)
    Vec3 color{1, 1, 1}, colorHi{1, 1, 1};   // a spray's vertex colour, drawn between (the strand carries the hues)
};

// The species' spray variants, each in the spray frame at unit length; the strand atlas is `tiles` x `tiles`.
std::vector<RenderMesh> buildSprays(const SpraySpec& spec, int tiles, uint32_t seed);

// LEAVES AND BLOSSOMS (TreeParams::foliage = 2; Glenn: the leaves "look bad and also really big especially for
// sakura ... Have you updated to leaves to what we discussed?"): broadleaf near foliage as EVERY LEAF ITS OWN small
// blade -- a few quads folded along the midrib and curled down at the tip, wearing one leaf of the species'
// ELEMENT ATLAS (leaf_card.h foliageElementAtlas: square tiles, a whole leaf or a whole flower each) -- set along
// each leafy twig in its arrangement (alternate, or opposite pairs), hanging off it and turned to the light; and
// BLOSSOMS as small CUPS (a shallow cone wearing a flower) in clusters on short stalks along the twigs.
struct LeafKitSpec {
    bool set = false;
    double size = 0.10;          // a blade's length (m), stalk to tip
    int shoots = 3;              // LEAFY SHOOTS each leafy twig puts out (the fine twigs the skeleton does not grow)
    double shootLength = 0.25;   // ...their length (m), drawn 0.7 .. 1.3 x
    double shootAngle = 45.0;    // ...off the twig (deg)
    double lift = 0.3;           // ...lifted toward the light
    double sag = 0.08;           // ...and sagging under their leaves, per third of their length
    int perTwig = 8;             // leaves along each shoot
    bool pairs = false;          // opposite pairs, each pair turned 90 deg from the last (maple); else alternate
    double angle = 50.0;         // a leaf's stalk off the twig (deg)
    double fold = 20.0;          // each half of the blade raised about the midrib (deg)
    double curl = 0.35;          // the blade's tip bent down (rad)
    double droop = 0.35;         // the blade hangs: 0 on along its stalk, 1 straight down
    double face = 0.6;           // the blade turned to face the sky: 0 as it hangs, 1 flat to the sky
    int budget = 30000;          // leaves on a tree at most (thinned evenly over the crown)
    double blossomSize = 0.0;    // a blossom across (m); 0: no blossoms
    int perCluster = 4;          // blossoms in a cluster (an umbel), on stalks spread round it
    int clusters = 3;            // clusters along each shoot
    double cup = 0.35;           // the cup's depth, a fraction of its radius
    int blossomBudget = 20000;
};

// One LEAF: its stalk's end at `base`, the blade along `dir`, its face toward `normal`, `len` long; `tile` of the
// element atlas (`tiles` a side). One winding (the leaf material is two-sided).
void addKitLeaf(RenderMesh& m, const Vec3& base, const Vec3& dir, const Vec3& normal, double len, double fold,
                double curl, int tiles, int tile, const Vec3& color);
// One BLOSSOM: a cup `size` across round `centre`, opening toward `facing`, turned `roll` about it.
void addKitBlossom(RenderMesh& m, const Vec3& centre, const Vec3& facing, double size, double cup, double roll,
                   int tiles, int tile, const Vec3& color);

// BLADES (Glenn's street-planting photos: New Zealand flax, rushes, grass tufts; and a palm's leaflets): a STRAP
// LEAF as geometry -- a strip up its own curve, keeled along the midrib (`fold`), arching over toward the tip
// (`arch`), twisting as it goes, full width to `taper` of its length then narrowing to a point -- wearing the blade
// texture (material_recipes.h bladeTextures: u across, v along). Its edges may take their own colour (a flax's
// bronze-red margins); the midrib takes the blade's.
struct BladeShape {
    double length = 1.0, width = 0.06;
    double arch = 0.6;           // the tip bent down by this (rad), the bend growing along the blade
    double fold = 25.0;          // each half raised about the midrib (deg)
    double twist = 0.3;          // turned about its own midrib by this (rad) by the tip
    double taper = 0.7;          // full width to here (a fraction of the length), then narrowing to the point
    int segments = 6;
};
void addBlade(RenderMesh& m, const Vec3& base, const Vec3& dir, const Vec3& across, const BladeShape& shape,
              const Vec3& color, const Vec3& edgeColor);

// A BLADE PLANT (a recipe of kind = "blades"): blades from a crown at the ground -- a radial clump, or (fan -> 1)
// flattened into a fan as a flax's are; the outer blades leaning out further than the inner, longest in the middle.
struct BladeSpec {
    bool set = false;
    int count = 40;
    double length = 1.2, lengthJitter = 0.35;   // a blade's length (m), drawn +- lengthJitter of it
    double width = 0.06;
    double rise = 10.0, spread = 45.0;          // the inner blades this far off upright (deg), the outer this more
    double fan = 0.0;                           // 0 a round clump, 1 a flat fan
    double arch = 0.6, fold = 25.0, twist = 0.3, taper = 0.7;
    double crown = 0.08;                        // the crown's radius (m): where the blades rise from
    Vec3 color{0.2, 0.3, 0.1}, colorHi{0.2, 0.3, 0.1};   // a blade's colour, drawn between
    Vec3 edgeColor{-1, -1, -1};                 // its margins' colour (negative: the blade's own)
};
// The plant: its blades into `leaves`, its crown (a low dark mound the blades rise from) into `crown`.
void buildBladePlant(const BladeSpec& spec, uint32_t seed, RenderMesh& leaves, RenderMesh& crown);

// A PALM'S CROWN (TreeParams::foliage = 4; Glenn: the palms "don't quite look like palm leaves or trunks"): fronds
// radiating from the top of the trunk, the youngest upright, the oldest hanging, each one either
//   PINNATE (a coconut, a date): an arching rachis -- a thin tube -- with leaflet blades down both sides, angled
//            forward and hanging from it, longest mid-frond
//   FAN      (a Washingtonia, a Chinese fan palm): a long petiole ending in a fan of blades radiating from its tip,
//            their tips hanging
// and, below, a SKIRT of dead fronds hanging down the trunk (fan palms keep theirs).
struct FrondSpec {
    bool set = false;
    bool fan = false;
    int count = 22;
    double length = 3.6, lengthJitter = 0.15;   // a frond's rachis (pinnate) or petiole + fan (fan), m
    double arch = 0.9;                          // the rachis bends down by this (rad) over its length
    double rachisRadius = 0.025;
    int leaflets = 40;                          // per side (pinnate) / blades in the fan (fan)
    double leafletLength = 0.75, leafletWidth = 0.04;
    double leafletAngle = 55.0;                 // a leaflet's angle forward off the rachis (deg)
    double leafletDroop = 0.6;                  // ...and its hang (rad)
    double fanSpread = 200.0;                   // a fan's opening (deg)
    double petiole = 0.55;                      // a fan frond: the petiole's share of its length
    double pitchYoung = 25.0, pitchOld = 105.0; // off upright (deg): the newest frond, the oldest
    int skirt = 0;                              // dead fronds hanging down the trunk
    Vec3 color{0.18, 0.32, 0.09}, colorHi{0.18, 0.32, 0.09};
    Vec3 rachisColor{0.45, 0.40, 0.22};
    Vec3 skirtColor{0.33, 0.25, 0.16};
};
// The crown at `top` (the trunk heading `up` there): blades into `leaves`, rachises and petioles into `wood`.
void addFrondCrown(const FrondSpec& spec, const Vec3& top, const Vec3& up, float trunkRadius, uint32_t seed,
                   RenderMesh& leaves, RenderMesh& wood);

}  // namespace engine

#endif
