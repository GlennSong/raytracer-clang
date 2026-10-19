#ifndef RAYTRACER_ENGINE_TREE_H
#define RAYTRACER_ENGINE_TREE_H

#include "../../renderer/renderer.h"   // RenderMesh, Vertex
#include "../../rt_math.h"             // Vec3
#include "lsystem.h"                   // ModuleString (for skinTree)
#include "foliage_kit.h"               // SpraySpec
#include <cstdint>
#include <vector>

namespace engine {

// A parametric tree generator (ROADMAP Tier 4 Phase B; the realism plan in
// docs/lsystem-botany-plan.md). It grows a branch skeleton with a parametric
// L-system (lsystem.h), assigns branch radii bottom-up by the pipe model
// (da Vinci / Murray's law), then skins each branch as a generalized cylinder
// — swept tapered rings with bark UVs + tangents — instead of an SDF blob, so
// thin twigs survive and bark textures apply. Leaves are alpha-cut cards, not
// spheres. The bark mesh doubles as the static collision surface.
struct TreeParams {
    // --- L-system shape ---
    int   iterations    = 5;        // branch orders (recursion depth)
    float trunkLength   = 1.6f;     // length of the first internode
    float lengthFalloff = 0.82f;    // SIDE-branch internode = parent * this (lower
                                    //   = sides shrink fast and ramify densely)
    float leaderFalloff = 0.9f;     // central-leader internode = parent * this (high
                                    //   = a tall trunk that keeps its height)
    float branchAngle   = 36.0f;    // pitch of a branch away from its parent (deg)
    float angleJitter   = 14.0f;    // +/- random variation applied to every turn (deg)
    int   branchesPerNode = 2;      // children spawned at each node (2 = forking)
    float phyllotaxis   = 137.5f;   // roll advance between successive children (deg)
    // Terminal ramification: once an internode shrinks below
    // `terminalFraction * trunkLength`, a node bursts into a denser spray of
    // `terminalForks` short twigs (guarded production), so branch ends fork
    // repeatedly and carry many leaf clusters — the real source of crown density.
    // 0 forks disables it. Needs a falloff < ~0.85 so internodes actually reach
    // the threshold within `iterations`.
    float terminalFraction = 0.34f;
    int   terminalForks    = 3;

    // --- Organic shaping (centerline bend; thin branches bend most) ---
    float droop         = 0.22f;    // gravity sag per segment, accumulates down a limb
    float wander        = 0.06f;    // lateral random meander (kills the stiffness)

    // --- Gnarl (Glenn, 2026-10-17: "trees aren't just tubes, there's all kinds of craggly movement in a tree
    //     branch/trunk"). The centreline's wander and the cross-section's shape; 0 = the plain swept tube. ---
    float light         = 0.0f;     // phototropism: the thin ends turn back up toward the sky (the S of a sagging limb)
    float crookChance   = 0.0f;     // per node, the chance a limb kinks (a lost leader, a lateral taking over)
    float crookAngle    = 25.0f;    // ...by this much (deg)
    float meander       = 0.0f;     // smooth multi-octave meander, as a fraction of the internode length
    float lean          = 0.0f;     // the whole tree leans this far (deg) off vertical, toward a seeded azimuth
    float flare         = 0.0f;     // the trunk's radius swells toward the ground by 1 + flare (the root flare)
    int   lobes         = 0;        // flutes / buttress ridges round the trunk
    float lobeDepth     = 0.0f;     // ...their depth as a fraction of the radius at the foot
    float lobeHeight    = 2.0f;     // ...fading to nothing over this height (m)
    float nodeSwell     = 0.0f;     // each node bulges by this fraction of the radius (and pinches between)
    float barkNoise     = 0.0f;     // irregular cross-section (knotty, uneven), a fraction of the radius
    float twist         = 0.0f;     // grain twist: the lobes and bumps spiral by this many degrees a metre
    float collar        = 0.0f;     // THE WELD: a side branch starts on its parent's surface and flares into it by
                                    //   this (a fraction of its radius) -- no tube buried in the fork, no crease;
                                    //   0 = the old overlapping tubes
    float forkRatio     = 0.7f;     // ...and where a limb splits into near-equal halves (a side child at least this
                                    //   share of the continuing one's radius) neither carries on through the fork:
                                    //   the limb ends in a short capped stub and BOTH halves weld into it (a crotch)
    float forkMinRadius = 0.06f;    // ...on limbs at least this thick (m): a twig's fork is not worth the rings
    // THE JOINTS (ADR-0181; Glenn: "I can see the seams between the branches ... it's obviously not one continuous
    // structure"): 1 = at every fork of wood at least jointMinRadius thick, each limb meeting there is cut back and the
    // end rings are joined by ONE surface (their hull) -- the trunk and limbs one mesh. 0 = the welds above.
    int   joints        = 0;
    float jointMinRadius = 0.03f;
    // THE FAR WOOD (a forest's light level of detail): limbs thinner than this at their start are not skinned -- their
    // leaves still hang where they grew, and past ~60 m the cards hide where the twigs were. 0 = all of it.
    float woodMinRadius = 0.0f;
    // THE COARSE LEAVES (a distant level of the leaf kit; Glenn, 2026-10-17, of a forest's card trees: "you can tell
    // they're cards"): leaves leafLod times as long, 1/leafLod of the shoots carrying 1/leafLod as many each -- the
    // crown's leaf area kept on a leafLod^2 share of the leaves, each still turned its own way (no sheet to see edge
    // on). Past 1.5 the shoots' own twigs go too. 1 = the leaves as authored.
    float leafLod = 1.0f;
    // THE BRANCH CARDS (Glenn, 2026-10-18: "When we get to small enough branches ... use some kind of card to stand in
    // for those branches and clusters of leaves"; the wood held to ~8000 triangles a hero): a side shoot whose whole
    // spread -- every node of it from where it leaves its limb -- is within sprigSize metres is a SPRIG. Its wood and
    // leaves are not built; it is recorded (TreeMesh::sprigs) for a card baked from sprigs like it (branch_cards.h).
    // sprigDonor = 1 builds the sprigs after all, each into its own meshes (the pictures the cards are baked from).
    // 0 = everything modelled.
    float sprigSize = 0.0f;
    int   sprigDonor = 0;
    // THE WOOD'S RESOLUTION by need, not one number for every limb: round a limb, sides about woodEdge metres
    // apart (at least 4, at most ringSegments; 0 = ringSegments everywhere); along it, a ring where it has turned by
    // ringAngle degrees or gone ringSpacing of its radii since the last (0 = three rings to every internode)
    float woodEdge = 0.0f;
    float ringAngle = 0.0f;
    float ringSpacing = 4.0f;
    // THE TRUNK ZONE (Glenn, 2026-10-18: "a full on interconnected trunk structure"; ADR-0183): the wood at least
    // trunkZone metres in radius that is joined to the foot through wood as thick -- the trunk, its first forks, its
    // buttress roots -- is ONE implicit surface: each limb a tapered capsule, smooth-unioned with a fillet a share of
    // its radius, the trunk's flare and lobes and the bark's knots displacing it; sampled every trunkZoneCell metres
    // (sdf.h polygonizeSdfSparse), simplified to trunkZoneTris (MeshBuilder::simplify) and settled back onto the
    // field. Thinner limbs are tubes welded into it as before. 0 = off (every limb a tube).
    float trunkZone = 0.0f;
    // AGE'S MARKS (Glenn, 2026-10-18: "oldness in our variety"): this share of the side limbs between 1.5 cm and two
    // fifths of the trunk's girth are DEAD -- snapped off one or two internodes out (a broken stub), no foliage, the
    // bark greyed. 0 = none.
    float deadChance = 0.0f;
    // THE CROWN GROWN TO ITS SHAPE (Glenn, 2026-10-18, of the ancient oak: "skip the tangle of branches ... I would
    // expect more plumage and foliage"; space colonization, Runions, Lane & Prusinkiewicz 2007): colonize = 1 takes the
    // grammar's tree as the SCAFFOLD -- its trunk and few big limbs -- and grows the rest toward colonizePoints points
    // scattered through the crown's envelope (a dome round the scaffold, crownWiden wider and crownHeighten taller),
    // most of them in its outer shell (colonizeShell: the share of the radius they keep to). Each step every node with
    // points nearer it than any other node grows colonizeStep toward their mean; a point a node reaches is spent. The
    // branches so compete for room -- they spread, they do not cross -- and end in the shell, the inside left open to
    // the big limbs, as an old tree's is.
    int   colonize = 0;
    int   colonizePoints = 4000;
    float colonizeStep = 0.3f;
    float colonizeShell = 0.35f;
    float crownWiden = 1.15f;
    float crownHeighten = 1.08f;
    // THE LOBES (colonize = 2; Glenn, 2026-10-18: "how do we develop the lobes ... What's the best way to harness both
    // of these algorithms"): the L-system grows the SCAFFOLD -- the species' trunk and big limbs, its character -- and the
    // crown (authored: crownRadius / crownBottom / crownTop) is laid with LOBES over its surface, each lobeSize of the
    // crown's radius, its underside flatter; the lobes RELAXED apart where they overlap more than lobeOverlap. Each lobe
    // is grown from the scaffold's nearest limb (its ANCHOR: the limb reaches out to its clump), colonized only by that
    // limb's branches, its points (colonizePoints shared by volume) in its outer shell: clumps of foliage on a few
    // big limbs, the gaps of sky between them -- an old tree's billowing crown.
    float lobeSize = 0.35f;
    // THE CROWN'S SHAPE, authored (colonize = 2): a dome crownRadius metres round and from crownBottom to crownTop metres
    // up (0: from the scaffold's own reach -- which a coarse scaffold shrinks). The lobes are laid over its surface,
    // each grown from the scaffold's nearest limb.
    float crownRadius = 0.0f;
    float crownBottom = 0.0f;
    float crownTop = 0.0f;
    float lobeOverlap = 0.35f;
    float trunkZoneCell = 0.04f;
    int   trunkZoneTris = 3500;

    // --- Roots (buttress roots flaring from the base, splayed along the ground) ---
    int   rootCount     = 5;        // number of surface roots (0 = none)
    float rootSpread    = 5.0f;     // root length = base radius * this

    // --- Radii (pipe model) ---
    float tipRadius     = 0.018f;   // radius of a terminal twig
    float pipeExponent  = 2.3f;     // n in r_parent^n = sum(r_child^n); 2..3
    float radiusScale   = 1.0f;     // multiply every radius (overall thickness)

    // --- Mesh resolution ---
    int   ringSegments  = 7;        // cross-section vertices per branch ring
    float barkVScale    = 1.0f;     // bark UV tiling along branch length

    // --- Leaves ---
    bool  leaves        = true;
    float leafSize      = 0.16f;    // leaf card width
    int   leavesPerTip  = 4;
    float leafThickness = 0.045f;   // also leaf branches at/under this radius (not
                                    //   just twig tips), to fill out the ends; 0 = tips only
    float leafClump     = 0.9f;     // how far back along the twig's last internode its cards spread (a fraction of
                                    //   it; 0 = all at the node). Every card's foot is ON the twig's bark.
    float leafPitch     = 40.0f;    // a card's tilt out from its twig (deg): 0 along it, 90 straight out
    int   cardTiles     = 1;        // the leaf texture is an atlas of cardTiles x cardTiles cards (leaf_card.h): each
                                    //   card picks one
    bool  tipCards      = false;    // ...the atlas's first row at the tips, the other rows along the stems (a salvia's
                                    //   flower spikes ending its stems, leaves below)
    // CLIPPED (a hedge, a topiary): growth outside the box -- |x| <= clipX, 0 <= y <= clipY, |z| <= clipZ about the
    // foot -- is pruned back to the box, so the cut twigs' leaves crowd its faces. 0 = unclipped.
    float clipX = 0.0f, clipY = 0.0f, clipZ = 0.0f;
    int   maxLeafCards  = 6000;     // hard budget: stop emitting cluster cards past
                                    //   this, so one tree can't blow up the triangle count
    // THE FOLIAGE: 0 crossed cards (the mid distance), 1 SPRAYS (foliage_kit.h: shoots of strands, near trees) --
    // leavesPerTip sprays a leafy node, maxLeafCards the sprays' budget, leafSize a spray's length
    int   foliage       = 0;        // ... 2 LEAVES (foliage_kit.h LeafKitSpec: each leaf its own blade, blossoms cups)
    SpraySpec spray;
    LeafKitSpec leafKit;
    FrondSpec fronds;               // foliage = 4: a palm's crown

    // --- Bark variety (Glenn, 2026-10-17: "the birch texture is good, but looks to repetitive") ---
    float barkMottle    = 0.0f;     // soft darker / greyer patches over the bark, about a metre across (0 none)
    float barkFootDark  = 0.0f;     // the foot of the trunk darkened toward barkFootColor (a birch's black base)
    float barkFootHeight = 1.0f;    // ...fading out over this height (m)
    Vec3  barkFootColor = Vec3(0.045, 0.042, 0.04);   // (near black: a birch's base under full sun)

    // --- Colors (baked into vertex color; multiply the material albedo) ---
    Vec3  barkColor     = Vec3(0.33, 0.24, 0.16);
    Vec3  leafColor     = Vec3(0.19, 0.44, 0.13);
    std::vector<Vec3> leafPalette;    // non-empty: each card's colour blends along this palette by a noise over the
                                      //   crown (patches paletteScale metres across) instead of leafColor
    float paletteScale  = 2.5f;
};

// A grown tree: bark and leaves as separate meshes (different materials — bark
// is opaque, leaves are alpha-cut foliage), plus a collision triangle soup
// (branches only — you bounce off wood, not leaves). Feed the collision arrays
// straight to a MeshCollider / PhysicsWorld::addMesh.
struct TreeMesh {
    RenderMesh branches;                  // generalized-cylinder bark (UVs + tangents)
    RenderMesh leaves;                    // textured leaf cards
    // THE SKELETON it was skinned on: every internode, parent end to child end, with its radii (the validation that
    // every leaf and every piece of bark hangs on the tree; later the LODs)
    struct Limb { Vec3 a, b; float ra = 0, rb = 0; bool dead = false; };   // (dead: a dead limb's, TreeParams::deadChance)
    std::vector<Limb> limbs;
    int joints = 0, jointsPlain = 0;      // forks joined as one surface (ADR-0181), and those closed by a plain hull
    std::vector<uint32_t> jointRingStarts;   // ...the first vertex of every limb end ring a joint joined
    std::size_t jointIndexStart = 0, jointIndexEnd = 0;   // ...and the joints' own triangles in branches.indices
    std::size_t zoneIndexStart = 0, zoneIndexEnd = 0;     // the trunk zone's triangles in branches.indices (TreeParams::trunkZone)
    int zoneLimbs = 0, zoneRawTris = 0;                   // ...the limbs it joined, and its triangles before simplifying
    std::size_t zoneVertexStart = 0, zoneVertexEnd = 0;   // ...and its vertices in branches.vertices
    // THE BARK BLENDS (the trunk zone's: where two limbs' bark mappings meet): branches.tangentIsData, each vertex's
    // tangent slot a second UV and its weight (a tube's own UV, weight 0) -- drawn with RenderMaterial::FLAG_UV_BLEND
    bool barkBlend = false;
    // THE SPRIGS (TreeParams::sprigSize): each one's frame -- from `base` (where it leaves its limb) out along `axis`,
    // spread across `side`, `normal` = axis x side (turned skyward) -- and its reach in that frame: its nodes out to
    // `length` along the axis, `halfWidth` either side, `depth` either face. A donor tree (sprigDonor) builds each
    // into its own `bark` and `leaves`, in the tree's space.
    struct Sprig {
        Vec3 base, axis, side, normal;
        float length = 0, halfWidth = 0, depth = 0, radius = 0;
        int nodes = 0;
        RenderMesh bark, leaves;
    };
    std::vector<Sprig> sprigs;
    struct Lobe { Vec3 centre; float radius = 0; };   // (colonize = 2: the crown's lobes, TreeParams::lobeSize)
    std::vector<Lobe> lobes;
    // THE FINE BRANCHING the sprigs stand for (Glenn, 2026-10-18, of baked mini-tree cards: "it needs to be a branch
    // without leaves and with leaves or maybe a Y branch ... smaller segments planted along"): every internode inside a
    // sprig, parent end to node, its radius and whether it carries leaves (a tip or thin enough: leafThickness) --
    // branch_cards.h lays a twig segment card along each
    struct Twig { Vec3 a, b; float r = 0; bool leafy = false; int forks = 0; };
    std::vector<Twig> twigs;
    int foliage = 0;                      // what `leaves` is: 0 crossed cards (both windings), 1 sprays, 2 leaves, 3
                                          //   blades or 4 fronds (one winding: draw it two-sided) -- TreeParams::foliage
    std::vector<Vec3>     collisionVertices;
    std::vector<uint32_t> collisionIndices;
};

// Grow a tree. `seed` selects the stochastic variant (angle jitter + leaf
// scatter); the same params + seed always produce the same tree (ADR-0002).
TreeMesh growTree(const TreeParams& params, uint32_t seed = 0);

// Skin a pre-expanded module string into a tree (turtle-interpret -> skeleton ->
// pipe-model radii -> droop/wander -> curved generalized-cylinder sweep + leaf
// cards). The grammar-agnostic half of growTree: a Lua-authored parametric
// L-system feeds its expanded modules straight in (ADR-0030). The grammar
// fields of `params` (iterations/trunkLength/...) are unused here; the shaping
// fields (radii, ringSegments, droop, wander, angleJitter, leaves, colors) apply.
TreeMesh skinTree(const ModuleString& modules, const TreeParams& params,
                  uint32_t seed = 0);

// CPU pixel data for a procedural texture — uploaded by the renderer-aware
// caller (level loader), so procgen stays GPU-agnostic (ADR-0021). Row-major,
// `channels` bytes per texel.
struct TextureData {
    int width = 0;
    int height = 0;
    int channels = 0;
    std::vector<uint8_t> pixels;
};

// A seamless grayscale bark texture (RGB, vertical fibrous streaks) meant to
// *modulate* the bark vertex color — values around 0.55..1.0. Tiles vertically.
TextureData barkTexture(int size = 256, uint32_t seed = 0);

// Per-species bark relief: a grayscale albedo value pattern (modulates the bark
// vertex color) plus a tangent-space normal map for bump/relief (the renderer
// samples normalMap). Oak = deep vertical furrows, birch = pale with horizontal
// lenticels, pine = scale plates. Seamless (wraps).
enum class BarkStyle { Oak, Birch, Pine };
struct BarkMaps {
    TextureData albedo;
    TextureData normal;
};
BarkMaps barkMaps(BarkStyle style, int size = 256, uint32_t seed = 0);
BarkStyle barkStyleFromName(const std::string& name);   // "bark_birch" -> Birch

// A leaf card texture (RGBA): white RGB so the leaf vertex color tints it, with
// a pointed-oval alpha silhouette for alpha-cut foliage.
TextureData leafTexture(int size = 128);

}  // namespace engine

#endif
