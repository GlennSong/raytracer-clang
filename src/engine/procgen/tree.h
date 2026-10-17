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
    int   foliage       = 0;
    SpraySpec spray;

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
    struct Limb { Vec3 a, b; float ra = 0, rb = 0; };
    std::vector<Limb> limbs;
    int foliage = 0;                      // what `leaves` is: 0 crossed cards (both windings), 1 sprays (one winding:
                                          //   draw it two-sided) -- TreeParams::foliage
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
