// Tree recipes (procgen/tree_recipe.h): species as Lua data, grown by C++ -- the tree lab's trees.

#include "test_framework.h"
#include "../src/engine/procgen/tree_recipe.h"
#include "../src/engine/scripting/tree_recipes_lua.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include <string>

using namespace engine;

namespace {

TreeRecipeBook shipped() {
    std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/tree_recipes.lua");
    std::stringstream ss;
    ss << in.rdbuf();
    TreeRecipeBook book;
    std::string err;
    const bool ok = loadTreeRecipesSource(ss.str(), book, &err);
    if (!ok) std::printf("    [recipes] %s\n", err.c_str());
    CHECK(ok);
    return book;
}

Real topOf(const TreeMesh& t) {
    Real y = 0;
    for (const Vertex& v : t.branches.vertices) y = std::max(y, v.position.y);
    return y;
}

// How far from round the trunk is at height y: max / min distance of its bark vertices from their centre, in a thin
// band (above the roots' reach, below the crown)
Real trunkOutOfRound(const TreeMesh& t, Real y) {
    std::vector<Vec3> ring;   // (a ring repeats its first vertex to close its texture seam: once each)
    for (const Vertex& v : t.branches.vertices)
        if (std::fabs(v.position.y - y) < 0.06 &&
            std::none_of(ring.begin(), ring.end(), [&](const Vec3& q) { return (q - v.position).length() < 1e-5; }))
            ring.push_back(v.position);
    if (ring.size() < 6) return 0;
    // the trunk's vertices are the ones nearest the trunk's axis at this height (the band also cuts limbs higher up)
    Vec3 c(0, 0, 0);
    for (const Vec3& p : ring) c = c + p;
    c = c * (1.0 / ring.size());
    std::sort(ring.begin(), ring.end(), [&](const Vec3& a, const Vec3& b) { return (a - c).length() < (b - c).length(); });
    ring.resize(std::min<std::size_t>(ring.size(), 24));
    c = Vec3(0, 0, 0);
    for (const Vec3& p : ring) c = c + p;
    c = c * (1.0 / ring.size());
    Real lo = 1e9, hi = 0;
    for (const Vec3& p : ring) {
        const Real d = std::hypot(p.x - c.x, p.z - c.z);
        lo = std::min(lo, d);
        hi = std::max(hi, d);
    }
    return lo > 1e-6 ? hi / lo : 0;
}

}  // namespace

// THE SHIPPED RECIPES (assets/scripts/tree_recipes.lua) load and every species grows a whole tree -- bark, leaves,
// a collider, nothing non-finite -- and the same seed grows the same tree twice.
TEST_CASE(shipped_tree_recipes_load_and_grow) {
    const TreeRecipeBook book = shipped();
    CHECK(book.size() >= 3);
    for (const std::string& name : book.names()) {
        const TreeRecipe* r = book.find(name);
        const TreeMesh t = growRecipeTree(*r, 1);
        bool finite = true;
        for (const Vertex& v : t.branches.vertices)
            finite = finite && std::isfinite(v.position.x) && std::isfinite(v.position.y) && std::isfinite(v.position.z) &&
                     std::isfinite(v.normal.x) && std::fabs(v.normal.length() - 1.0) < 1e-3;
        std::printf("    [recipes] %-8s seed 1: %.1f m tall, %zu bark tris, %zu leaf tris\n", name.c_str(), topOf(t),
                    t.branches.indices.size() / 3, t.leaves.indices.size() / 3);
        CHECK(finite);
        CHECK(t.branches.indices.size() > 300);
        CHECK(!t.leaves.vertices.empty());
        CHECK(!t.collisionIndices.empty());
        CHECK(topOf(t) > 4.0);
        const TreeMesh again = growRecipeTree(*r, 1);
        CHECK(again.branches.vertices.size() == t.branches.vertices.size());
        CHECK(again.branches.vertices.empty() || (again.branches.vertices.back().position - t.branches.vertices.back().position).length() < 1e-9);
    }
}

// VARIETY (Glenn: "a variety of trees (same type of tree and different trees)"): a recipe's seeds are different trees
// of it -- six oaks are six heights, six branchings -- and the species differ from each other.
TEST_CASE(a_recipes_seeds_are_different_trees_of_it) {
    const TreeRecipeBook book = shipped();
    for (const std::string& name : book.names()) {
        std::set<long> heights, sizes;
        for (uint32_t seed = 1; seed <= 6; ++seed) {
            const TreeMesh t = growRecipeTree(*book.find(name), seed);
            heights.insert(std::lround(topOf(t) * 20.0));   // to 5 cm
            sizes.insert(static_cast<long>(t.branches.vertices.size()));
        }
        std::printf("    [variety] %-8s 6 seeds: %zu heights, %zu branchings\n", name.c_str(), heights.size(), sizes.size());
        CHECK(heights.size() >= 4);
        CHECK(sizes.size() >= 4);
    }
}

// THE GNARL (Glenn: "trees aren't just tubes"): an oak's foot is fluted -- out of round by its lobes and knots --
// where the same oak with the gnarl pinned off is a round tube.
TEST_CASE(a_gnarled_trunk_is_not_a_tube) {
    const TreeRecipeBook book = shipped();
    const TreeRecipe* oak = book.find("oak");
    CHECK(oak != nullptr);
    if (!oak) return;
    const std::map<std::string, double> plain = {{"lobes", 0}, {"lobeDepth", 0}, {"barkNoise", 0}, {"nodeSwell", 0},
                                                 {"flare", 0}, {"crookChance", 0}, {"meander", 0}, {"lean", 0}};
    const TreeMesh g = growRecipeTree(*oak, 2), p = growRecipeTree(*oak, 2, plain);
    if (std::getenv("RT_GNARL_PROBE"))
        for (Real y : {0.6, 0.8, 1.0, 1.4, 1.8, 2.4, 3.0}) {
            std::size_t n = 0;
            for (const Vertex& v : p.branches.vertices) n += std::fabs(v.position.y - y) < 0.06;
            std::printf("    [gnarl] y %.1f: plain %.3f (%zu verts) gnarled %.3f\n", y, trunkOutOfRound(p, y), n, trunkOutOfRound(g, y));
        }
    // (at the first height from 0.8 m up that cuts a ring of the trunk in both)
    Real og = 0, op = 0;
    for (Real y = 0.8; y < 2.5 && (og == 0 || op == 0); y += 0.05) { og = trunkOutOfRound(g, y); op = trunkOutOfRound(p, y); }
    std::printf("    [gnarl] oak trunk: out of round %.3f gnarled, %.3f plain\n", og, op);
    CHECK(op > 0 && op < 1.10);   // (a 10-sided ring: its flats are cos 18 deg of its corners)
    CHECK(og > 1.12);
}

// THE READER rejects what would silently grow a default tree: an unknown knob, a $name with no var, half a rule.
TEST_CASE(tree_recipe_reader_rejects_typos) {
    TreeRecipeBook b;
    std::string err;
    CHECK(!loadTreeRecipesSource("tree_recipes = { x = { params = { drop = 0.2 } } }", b, &err));
    CHECK(err.find("drop") != std::string::npos);
    CHECK(!loadTreeRecipesSource("tree_recipes = { x = { rules = { {'A(l)', 'F(l)&($pitch)A(l)'} } } }", b, &err));
    CHECK(err.find("pitch") != std::string::npos);
    CHECK(!loadTreeRecipesSource("tree_recipes = { x = { rules = { {'A(l)'} } } }", b, &err));
    TreeRecipeBook ok;
    CHECK(loadTreeRecipesSource("tree_recipes = { x = { vars = { pitch = {30, 40} }, rules = { {'A(l)', 'F(l)&($pitch)A(l)'} },"
                                " params = { droop = {0.1, 0.2}, lobes = 5 } } }", ok, &err));
    CHECK(ok.find("x") != nullptr);
    TreeParams tp;
    CHECK(setTreeParam(tp, "lobes", 6) && tp.lobes == 6);
    CHECK(!setTreeParam(tp, "lobez", 6));
}
