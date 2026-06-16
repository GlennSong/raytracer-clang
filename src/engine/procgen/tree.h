#ifndef RAYTRACER_ENGINE_TREE_H
#define RAYTRACER_ENGINE_TREE_H

#include "../../renderer/renderer.h"   // RenderMesh, Vertex
#include "../../rt_math.h"             // Vec3
#include "lsystem.h"                   // ModuleString (for skinTree)
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
    float lengthFalloff = 0.82f;    // child internode = parent * this
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
    float leafClump     = 0.9f;     // jitter cards into the gaps around a node, in
                                    //   leafSize units (canopy-volume fill); 0 = on-node
    int   maxLeafCards  = 6000;     // hard budget: stop emitting cluster cards past
                                    //   this, so one tree can't blow up the triangle count

    // --- Colors (baked into vertex color; multiply the material albedo) ---
    Vec3  barkColor     = Vec3(0.33, 0.24, 0.16);
    Vec3  leafColor     = Vec3(0.19, 0.44, 0.13);
};

// A grown tree: bark and leaves as separate meshes (different materials — bark
// is opaque, leaves are alpha-cut foliage), plus a collision triangle soup
// (branches only — you bounce off wood, not leaves). Feed the collision arrays
// straight to a MeshCollider / PhysicsWorld::addMesh.
struct TreeMesh {
    RenderMesh branches;                  // generalized-cylinder bark (UVs + tangents)
    RenderMesh leaves;                    // textured leaf cards
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
