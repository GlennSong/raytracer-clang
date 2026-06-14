#include "test_framework.h"

#include "../src/engine/procgen/tree.h"
#include <algorithm>

using namespace engine;

TEST_CASE(tree_grows_branch_and_leaf_geometry) {
    TreeParams p;
    p.iterations = 4;
    TreeMesh t = growTree(p, 1);
    CHECK(t.branches.vertices.size() > 0);
    CHECK(t.branches.indices.size() % 3 == 0);
    CHECK(t.leaves.vertices.size() > 0);     // leaves on by default
}

TEST_CASE(tree_branch_indices_are_in_range) {
    TreeParams p;
    p.iterations = 4;
    TreeMesh t = growTree(p, 3);
    uint32_t maxIdx = 0;
    for (uint32_t i : t.branches.indices) maxIdx = std::max(maxIdx, i);
    CHECK(maxIdx < t.branches.vertices.size());
}

TEST_CASE(tree_collision_mirrors_branch_geometry) {
    TreeParams p;
    p.iterations = 4;
    TreeMesh t = growTree(p, 5);
    // Collision is the bark surface: one position per bark vertex, same indices.
    CHECK(t.collisionVertices.size() == t.branches.vertices.size());
    CHECK(t.collisionIndices.size() == t.branches.indices.size());
    CHECK(t.collisionIndices.size() >= 3);
}

TEST_CASE(tree_has_bark_uvs_and_tapers) {
    TreeParams p;
    p.iterations = 5;
    TreeMesh t = growTree(p, 2);
    // Bark v-coordinate climbs the trunk, so some vertices have v > 0.
    bool anyV = false;
    for (const Vertex& v : t.branches.vertices) if (v.v > 0.01f) anyV = true;
    CHECK(anyV);
    // Pipe model: the thickest ring (near the root) is far wider than a twig.
    // Radius shows up as distance from the branch centerline isn't directly
    // available, so check the trunk base spans more than the tip radius: the
    // widest vertex offset in X/Z near the base exceeds tipRadius.
    float maxR = 0.0f;
    for (const Vertex& v : t.branches.vertices) {
        float r = std::sqrt(v.position.x * v.position.x + v.position.z * v.position.z);
        // near the base (low Y) the trunk is centered on the axis
        if (v.position.y < 0.3f) maxR = std::max(maxR, r);
    }
    CHECK(maxR > p.tipRadius);                // trunk is thicker than a twig
}

TEST_CASE(tree_is_deterministic_per_seed) {
    TreeParams p;
    p.iterations = 4;
    TreeMesh a = growTree(p, 7);
    TreeMesh b = growTree(p, 7);
    TreeMesh c = growTree(p, 8);
    CHECK(a.branches.vertices.size() == b.branches.vertices.size());
    bool identical = a.branches.vertices.size() == b.branches.vertices.size();
    for (size_t i = 0; identical && i < a.branches.vertices.size(); i++)
        if ((a.branches.vertices[i].position - b.branches.vertices[i].position)
                .lengthSquared() > 1e-18) identical = false;
    CHECK(identical);                        // same seed -> identical tree
    // A different seed jitters the angles, so the geometry differs.
    bool differs = a.branches.vertices.size() != c.branches.vertices.size();
    for (size_t i = 0; !differs && i < a.branches.vertices.size(); i++)
        if ((a.branches.vertices[i].position - c.branches.vertices[i].position)
                .lengthSquared() > 1e-9) differs = true;
    CHECK(differs);
}

TEST_CASE(tree_leaves_can_be_disabled) {
    TreeParams p;
    p.iterations = 4;
    p.leaves = false;
    TreeMesh t = growTree(p, 1);
    CHECK(t.leaves.vertices.empty());
    CHECK(t.branches.vertices.size() > 0);
}

TEST_CASE(bark_texture_is_rgb_and_modulating) {
    TextureData t = barkTexture(32, 5);
    CHECK(t.width == 32);
    CHECK(t.channels == 3);
    CHECK(t.pixels.size() == 32u * 32u * 3u);
    // Values stay bright enough to modulate (>= ~0.4) and vary across the image.
    uint8_t lo = 255, hi = 0;
    for (uint8_t px : t.pixels) { lo = std::min(lo, px); hi = std::max(hi, px); }
    CHECK(lo >= 100);            // 0.4 * 255 floor
    CHECK(hi > lo);              // there is actual variation
    CHECK(barkTexture(32, 5).pixels == t.pixels);   // deterministic per seed
}

TEST_CASE(leaf_texture_has_alpha_cutout) {
    TextureData t = leafTexture(64);
    CHECK(t.channels == 4);
    CHECK(t.pixels.size() == 64u * 64u * 4u);
    // The silhouette has both opaque interior and transparent corners.
    bool opaque = false, clear = false;
    for (size_t i = 3; i < t.pixels.size(); i += 4) {
        if (t.pixels[i] > 200) opaque = true;
        if (t.pixels[i] < 50) clear = true;
    }
    CHECK(opaque);
    CHECK(clear);
}
