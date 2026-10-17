// Tree recipes (procgen/tree_recipe.h): species as Lua data, grown by C++ -- the tree lab's trees.

#include "test_framework.h"
#include "../src/engine/procgen/tree_recipe.h"
#include "../src/engine/scripting/tree_recipes_lua.h"
#include "../src/engine/procgen/leaf_card.h"
#include "../src/image.h"
#include "../src/engine/procgen/material_recipes.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <map>
#include <tuple>
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
        if (r->shrub) CHECK(topOf(t) > 0.3 && topOf(t) < 3.0);   // knee to head height
        else CHECK(topOf(t) > 4.0);
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
        const TreeRecipe* r = book.find(name);
        const bool clipped = r->shrub && std::any_of(r->params.begin(), r->params.end(), [](const TreeRecipe::Range& g) {
            return g.name == "clipY" && g.hi > 0; });
        CHECK(clipped || heights.size() >= (r->shrub ? 3u : 4u));   // (a clipped hedge is its box's height)
        CHECK(sizes.size() >= (r->shrub ? 3u : 4u));
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

// THE WELD (Glenn: "the tree trunks and branches were not seamless"): with a collar, every side branch starts on its
// parent's surface -- the rings it had buried inside the parent are gone (fewer bark vertices than the same tree's
// overlapping tubes), and its normals stay whole across the joint.
TEST_CASE(welded_branches_start_on_their_parent_not_inside_it) {
    const TreeRecipeBook book = shipped();
    const TreeRecipe* oak = book.find("oak");
    CHECK(oak != nullptr);
    if (!oak) return;
    const TreeMesh tube = growRecipeTree(*oak, 3, {{"collar", 0}}), weld = growRecipeTree(*oak, 3, {{"collar", 0.6}});
    bool whole = true;
    for (const Vertex& v : weld.branches.vertices)
        whole = whole && std::isfinite(v.normal.x) && std::fabs(v.normal.length() - 1.0) < 1e-3 && std::isfinite(v.position.y);
    std::printf("    [weld] oak seed 3: %zu bark vertices as overlapping tubes, %zu welded\n", tube.branches.vertices.size(),
                weld.branches.vertices.size());
    CHECK(whole);
    // (the crotches' stubs add rings where a thick limb splits; the buried rings dropped take some back -- a weld costs
    // at most a third more bark)
    CHECK(weld.branches.vertices.size() < tube.branches.vertices.size() * 4 / 3);
    CHECK(weld.branches.indices.size() % 3 == 0);
}

// THE LEAF CARDS (Glenn: "when I look at the leaf cards I can see where it's cut off ... Can we generate those cards
// using the l-system to draw those leaves?"): every species with a card draws its own -- lobed oak leaves, toothed
// birch leaves, spruce needles -- and NOTHING touches a card's edge (the twig fitted inside a margin), every tile of
// the atlas holds leaves, the normals are unit. RT_CARD_DUMP=<dir> writes each atlas over a grey for a look.
TEST_CASE(leaf_cards_are_drawn_whole_inside_the_card) {
    const TreeRecipeBook book = shipped();
    int cards = 0;
    for (const std::string& name : book.names()) {
        const TreeRecipe* r = book.find(name);
        if (!r->card.set) continue;
        ++cards;
        const LeafCardAtlas at = leafCardAtlas(r->card, 11u);
        const int n = at.tiles, W = at.albedo.width, H = at.albedo.height, tw = W / n, th = H / n;
        int edge = 0, emptyTiles = 0;
        double coverage = 0;
        for (int t = 0; t < n * n; ++t) {
            long covered = 0;
            for (int y = 0; y < th; ++y)
                for (int x = 0; x < tw; ++x) {
                    const uint8_t a = at.albedo.pixels[(static_cast<std::size_t>((t / n) * th + y) * W + (t % n) * tw + x) * 4 + 3];
                    if (a == 0) continue;
                    ++covered;
                    if (x < 2 || y < 2 || x >= tw - 2 || y >= th - 2) ++edge;
                }
            if (covered == 0) ++emptyTiles;
            coverage += covered / double(tw * th);
        }
        std::printf("    [cards] %-8s %s: %dx%d atlas of %d, %.0f%% covered, %d texels on a tile's edge\n", name.c_str(),
                    r->card.shape.c_str(), W, H, n * n, 100.0 * coverage / (n * n), edge);
        CHECK(emptyTiles == 0);
        CHECK(edge == 0);
        CHECK(coverage / (n * n) > 0.04);
        if (const char* dir = std::getenv("RT_CARD_DUMP")) {
            Image img(W, H);
            for (int y = 0; y < H; ++y)
                for (int x = 0; x < W; ++x) {
                    const uint8_t* p = &at.albedo.pixels[(static_cast<std::size_t>(y) * W + x) * 4];
                    // (as the tree wears it: the atlas colour times the tree's vertex colour)
                    const Vec3 vc = r->leafPalettes.empty() ? r->leafColor : r->leafPalettes[0][1 % r->leafPalettes[0].size()];
                    const double a = p[3] / 255.0;
                    const Vec3 rgb(p[0] / 255.0, p[1] / 255.0, p[2] / 255.0);
                    const Vec3 leaf(rgb.x * vc.x, rgb.y * vc.y, rgb.z * vc.z), bg(0.55, 0.55, 0.58);
                    img.setPixel(x, H - 1 - y, bg * (1.0 - a) + leaf * a);
                }
            img.writePng(std::string(dir) + "/card_" + name + ".png");
        }
    }
    CHECK(cards >= 3);
    // the STRANDS (near foliage, foliage_kit.h): whole inside their narrow tiles too
    for (const std::string& name : book.names()) {
        const TreeRecipe* r = book.find(name);
        if (!r->strand.set) continue;
        const int tw = std::max(32, static_cast<int>(std::lround(436.0 * r->spray.aspect)));
        const LeafCardAtlas at = leafCardAtlas(r->strand, 11u, tw, 436);
        const int W = at.albedo.width, H = at.albedo.height;
        int edge = 0, solid = 0;
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x) {
                const uint8_t a = at.albedo.pixels[(static_cast<std::size_t>(y) * W + x) * 4 + 3];
                solid += a > 128;
                const bool border = x % tw == 0 || x % tw == tw - 1 || y % 436 == 0 || y % 436 == 435;
                edge += border && a > 128;
            }
        std::printf("    [strand] %-8s %dx%d, %d solid texels, %d on a tile edge\n", name.c_str(), W, H, solid, edge);
        CHECK(solid > W * H / 20);
        CHECK(edge == 0);
        if (const char* dir = std::getenv("RT_CARD_DUMP")) {
            Image img(W, H);
            for (int y = 0; y < H; ++y)
                for (int x = 0; x < W; ++x) {
                    const uint8_t* p = &at.albedo.pixels[(static_cast<std::size_t>(y) * W + x) * 4];
                    const double a = p[3] / 255.0;
                    img.setPixel(x, H - 1 - y, Vec3(0.55, 0.55, 0.58) * (1.0 - a) + Vec3(p[0], p[1], p[2]) * (a / 255.0));
                }
            img.writePng(std::string(dir) + "/strand_" + name + ".png");
        }
    }
}

// EVERY PART HANGS ON THE TREE (Glenn: "some of the spruce leaves seem to be floating and not attached to anything. We
// should validate that all of the geometry makes sense for a tree"): every limb of the skeleton starts where another
// ends (or at the foot); every leaf card's foot is on a twig's bark; every bark vertex lies on a limb -- within its
// radius swollen by the flare, the lobes, the collar and a crotch's stub, never metres out in the air.
TEST_CASE(every_part_of_a_tree_hangs_on_it) {
    const TreeRecipeBook book = shipped();
    auto segDist = [](const Vec3& p, const TreeMesh::Limb& l, float& radAt) {
        const Vec3 ab = l.b - l.a;
        const Real L2 = std::max(Real(1e-12), dot(ab, ab));
        const Real t = std::clamp(dot(p - l.a, ab) / L2, Real(0), Real(1));
        radAt = l.ra + (l.rb - l.ra) * static_cast<float>(t);
        return (p - (l.a + ab * t)).length();
    };
    for (const std::string& name : book.names())
        for (uint32_t seed : {1u, 4u}) {
            const TreeMesh t = growRecipeTree(*book.find(name), seed);
            CHECK(!t.limbs.empty());
            // connected: every limb's start is the foot or another limb's end
            int loose = 0;
            for (const TreeMesh::Limb& l : t.limbs) {
                bool joined = (l.a - t.limbs.front().a).length() < 1e-6;
                for (const TreeMesh::Limb& m : t.limbs) joined = joined || (m.b - l.a).length() < 1e-6;
                loose += !joined;
            }
            // the cards: each quad's foot (its first two corners' midpoint) on a twig
            int floating = 0;
            Real worstCard = 0;
            for (std::size_t q = 0; q + 3 < t.leaves.vertices.size(); q += 4) {
                const Vec3 foot = (t.leaves.vertices[q].position + t.leaves.vertices[q + 1].position) * 0.5;
                Real best = 1e9;
                for (const TreeMesh::Limb& l : t.limbs) {
                    float r = 0;
                    const Real d = segDist(foot, l, r) - r;
                    best = std::min(best, d);
                }
                worstCard = std::max(worstCard, best);
                floating += best > 0.03;
            }
            // the bark: every 7th vertex near a limb
            int adrift = 0;
            Real worstBark = 0;
            for (std::size_t v = 0; v < t.branches.vertices.size(); v += 7) {
                const Vec3 p = t.branches.vertices[v].position;
                Real best = 1e9;
                for (const TreeMesh::Limb& l : t.limbs) {
                    float r = 0;
                    const Real d = segDist(p, l, r);
                    best = std::min(best, d - (4.6 * r + 0.08));
                }
                worstBark = std::max(worstBark, best);
                adrift += best > 0;
            }
            if (seed == 1)
                std::printf("    [attached] %-8s %zu limbs (%d loose), %zu cards (%d off a twig, worst %.3f m), bark %d adrift\n",
                            name.c_str(), t.limbs.size(), loose, t.leaves.vertices.size() / 8, floating, worstCard, adrift);
            CHECK(loose == 0);
            CHECK(floating == 0);
            CHECK(adrift == 0);
        }
}

// THE BARKS (material_recipes.h): every kind bakes, TILES (its left column meets its right, its top its bottom, as
// closely as two neighbouring columns inside it do), and each reads differently -- an oak's furrows rough, a beech's
// skin smooth.
// RT_BARK_DUMP=<dir> writes each tile, 2 x 2, tinted a bark brown.
TEST_CASE(barks_by_kind_tile_and_differ) {
    const char* names[] = {"furrowed", "plated", "smooth", "papery", "ringed"};
    std::vector<double> means;
    for (const char* n : names) {
        BarkKind k;
        CHECK(barkKindFromName(n, k));
        const BarkTextures t = barkTextures(k, 1u, 256);
        const int W = t.albedo.width, H = t.albedo.height, C = t.albedo.channels;
        CHECK(W == 256 && H == 256 && !t.normal.pixels.empty());
        auto px = [&](int x, int y) { return static_cast<double>(t.albedo.pixels[(static_cast<std::size_t>(y) * W + x) * C]); };
        // the seam against the step between two neighbouring columns (rows) inside, the same way
        double seamU = 0, seamV = 0, innerU = 0, innerV = 0, mean = 0, sq = 0;
        for (int i = 0; i < H; ++i) { seamU += std::fabs(px(0, i) - px(W - 1, i)); innerU += std::fabs(px(W / 2, i) - px(W / 2 - 1, i)); }
        for (int i = 0; i < W; ++i) { seamV += std::fabs(px(i, 0) - px(i, H - 1)); innerV += std::fabs(px(i, H / 2) - px(i, H / 2 - 1)); }
        for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) { mean += px(x, y); sq += px(x, y) * px(x, y); }
        mean /= W * H;
        const double sd = std::sqrt(std::max(0.0, sq / (W * H) - mean * mean)) / 255.0;
        means.push_back(sd);
        std::printf("    [bark] %-8s mean %.2f contrast %.3f, seam %.1f / %.1f, inner step %.1f / %.1f\n", n, mean / 255.0, sd,
                    seamU / H, seamV / W, innerU / H, innerV / W);
        CHECK(seamU < 2.5 * innerU + H * 2.0);
        CHECK(seamV < 2.5 * innerV + W * 2.0);
        if (const char* dir = std::getenv("RT_BARK_DUMP")) {
            Image img(W * 2, H * 2);
            for (int y = 0; y < H * 2; ++y)
                for (int x = 0; x < W * 2; ++x)
                    img.setPixel(x, y, Vec3(0.42, 0.33, 0.25) * (px(x % W, y % H) / 255.0) * 1.6);
            img.writePng(std::string(dir) + "/bark_" + n + ".png");
        }
    }
    CHECK(means[0] > 2.0 * means[2]);   // an oak's furrows far rougher than a beech's skin (the colour is the vertex's)
}

// COLOUR (Glenn, 2026-10-17: "more color"): a sakura's card carries its blossoms' pink apart from the twig, and a
// maple's palette varies its cards across the crown -- several colours on one tree, not one.
TEST_CASE(cards_carry_colour_and_palettes_vary_the_crown) {
    const TreeRecipeBook book = shipped();
    const TreeRecipe* sakura = book.find("sakura");
    const TreeRecipe* maple = book.find("maple");
    CHECK(sakura && maple);
    if (!sakura || !maple) return;
    CHECK(sakura->card.flower.set);
    const LeafCardAtlas at = leafCardAtlas(sakura->card, 3u);
    int pink = 0, solid = 0;
    for (std::size_t o = 0; o < at.albedo.pixels.size(); o += 4) {
        const uint8_t* p = &at.albedo.pixels[o];
        if (p[3] < 128) continue;
        ++solid;
        if (p[0] > p[1] + 40 && p[2] > p[1]) ++pink;   // red over green, blue over green: pink
    }
    std::printf("    [colour] sakura card: %d of %d texels pink\n", pink, solid);
    CHECK(solid > 0 && pink > solid / 3);
    // a maple's leaf vertex colours: more than one hue across the crown
    const TreeMesh m = growRecipeTree(*maple, 2u);
    double lo = 1e9, hi = -1e9;
    for (const Vertex& v : m.leaves.vertices) {
        const double ratio = v.color.x / std::max(1e-6, v.color.x + v.color.y);   // red's share
        lo = std::min(lo, ratio);
        hi = std::max(hi, ratio);
    }
    std::printf("    [colour] maple red share across the crown %.2f .. %.2f\n", lo, hi);
    CHECK(hi - lo > 0.08);
}

// NEAR FOLIAGE (foliage_kit.h; Glenn: "a 2d texture image of a strand of spruce blades ... put them around a cylinder
// ... cluster those using geometry"): a spruce grown with foliage = 1 wears sprays -- shoots of strand fins -- within
// their budget, every spray on the tree (no piece further from a limb than a spray is long), and the same seed grows
// the same sprays.
TEST_CASE(spruce_sprays_are_built_from_strands_and_hang_on_the_tree) {
    const TreeRecipeBook book = shipped();
    const TreeRecipe* r = book.find("spruce");
    CHECK(r && r->strand.set && r->spray.set);
    if (!r) return;
    const TreeMesh t = growRecipeTree(*r, 3u, {{"foliage", 1.0}, {"leafSize", 0.7}});
    const TreeMesh cards = growRecipeTree(*r, 3u);
    CHECK(t.foliage == 1 && cards.foliage == 0);
    const std::vector<RenderMesh> one = buildSprays(r->spray, 2, 7u);
    CHECK(static_cast<int>(one.size()) == r->spray.variants);
    const std::size_t sprayTris = one.empty() ? 0 : one[0].indices.size() / 3;
    std::printf("    [sprays] spruce: %zu tris a spray, %zu leaf tris (cards: %zu), %zu bark tris\n", sprayTris,
                t.leaves.indices.size() / 3, cards.leaves.indices.size() / 3, t.branches.indices.size() / 3);
    CHECK(t.leaves.indices.size() / 3 <= static_cast<std::size_t>(r->spray.budget) * (sprayTris + 40));
    int off = 0;
    for (std::size_t v = 0; v < t.leaves.vertices.size(); v += 53) {
        const Vec3 p = t.leaves.vertices[v].position;
        Real best = 1e9;
        for (const TreeMesh::Limb& l : t.limbs) {
            const Vec3 ab = l.b - l.a;
            const Real s = std::clamp(dot(p - l.a, ab) / std::max(Real(1e-12), dot(ab, ab)), Real(0), Real(1));
            best = std::min(best, (p - (l.a + ab * s)).length());
        }
        off += best > 0.7 * 1.15 * 1.2;
    }
    CHECK(off == 0);
    const TreeMesh again = growRecipeTree(*r, 3u, {{"foliage", 1.0}, {"leafSize", 0.7}});
    CHECK(again.leaves.vertices.size() == t.leaves.vertices.size());
}

// THE JOINTS (ADR-0181; Glenn: "I can see the seams between the branches and looking at them in wireframe it's
// obviously not one continuous structure"): grown with joints = 1, a tree's forks of wood are joined as one surface --
// nearly every fork built whole (not the plain-hull fallback), and the trunk and its limbs one connected mesh (the
// largest piece of the bark, vertices joined by index and at the u seam by position, carries most of the bark;
// what is left are the twigs welded on as before).
TEST_CASE(limbs_meet_in_one_surface_at_their_joints) {
    const TreeRecipeBook book = shipped();
    for (const char* name : {"oak", "birch", "sakura", "maple", "pine", "spruce"}) {
        const TreeRecipe* r = book.find(name);
        CHECK(r != nullptr);
        if (!r) continue;
        const TreeMesh t = growRecipeTree(*r, 2u, {{"joints", 1.0}});
        const RenderMesh& m = t.branches;
        // union-find over vertices: by triangle, and by position (the u seam's duplicates)
        std::vector<uint32_t> up(m.vertices.size());
        for (uint32_t i = 0; i < up.size(); ++i) up[i] = i;
        std::function<uint32_t(uint32_t)> find = [&](uint32_t x) { return up[x] == x ? x : up[x] = find(up[x]); };
        auto join = [&](uint32_t a, uint32_t b) { up[find(a)] = find(b); };
        for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) { join(m.indices[k], m.indices[k + 1]); join(m.indices[k], m.indices[k + 2]); }
        std::map<std::tuple<long, long, long>, uint32_t> at;
        for (uint32_t i = 0; i < m.vertices.size(); ++i) {
            const Vec3& p = m.vertices[i].position;
            const auto key = std::make_tuple(std::lround(p.x * 1e5), std::lround(p.y * 1e5), std::lround(p.z * 1e5));
            auto it = at.find(key);
            if (it == at.end()) at[key] = i; else join(i, it->second);
        }
        // the wood the joints joined: every end ring in one piece
        std::map<uint32_t, std::size_t> ringPieces;
        for (uint32_t r : t.jointRingStarts) ++ringPieces[find(r)];
        std::size_t inBiggest = 0;
        for (const auto& [root, n] : ringPieces) inBiggest = std::max(inBiggest, n);
        if (std::getenv("RT_JOINT_DEBUG")) {
            uint32_t bigRoot = 0;
            for (const auto& [root, n] : ringPieces) if (n == inBiggest) bigRoot = root;
            int shown = 0;
            for (uint32_t r : t.jointRingStarts)
                if (find(r) != bigRoot && shown++ < 6) {
                    const Vec3 p = m.vertices[r].position;
                    std::printf("    [apart] ring at (%.2f %.2f %.2f), piece of %zu rings\n", p.x, p.y, p.z, ringPieces[find(r)]);
                }
        }
        std::map<uint32_t, std::size_t> tris;
        for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) ++tris[find(m.indices[k])];
        std::size_t biggest = 0;
        for (const auto& [root, n] : tris) biggest = std::max(biggest, n);
        const double share = m.indices.empty() ? 0.0 : static_cast<double>(biggest) / (m.indices.size() / 3);
        // SLIVERS: the joints' triangles with a least angle under 8 deg
        std::size_t slivers = 0, jointTris = 0;
        for (std::size_t k = t.jointIndexStart; k + 2 < t.jointIndexEnd; k += 3) {
            const Vec3 &A = m.vertices[m.indices[k]].position, &B = m.vertices[m.indices[k + 1]].position, &C = m.vertices[m.indices[k + 2]].position;
            auto ang = [](const Vec3& p, const Vec3& q, const Vec3& r) {
                const Vec3 u = q - p, w = r - p;
                return std::acos(std::clamp(dot(u, w) / std::max(Real(1e-18), u.length() * w.length()), Real(-1), Real(1)));
            };
            const Real least = std::min({ang(A, B, C), ang(B, C, A), ang(C, A, B)});
            slivers += least < 8.0 * 3.14159265 / 180.0;
            ++jointTris;
        }
        std::printf("    [joints] %-7s %d joints whole, %d plain; %zu of %zu end rings in one piece; %zu pieces, largest %.0f%% of the bark; %zu joint tris, %.1f%% slivers\n",
                    name, t.joints, t.jointsPlain, inBiggest, t.jointRingStarts.size(), tris.size(), share * 100.0, jointTris,
                    jointTris ? 100.0 * slivers / jointTris : 0.0);
        CHECK(slivers * 10 <= jointTris);
        CHECK(inBiggest * 10 >= t.jointRingStarts.size() * 9);
        CHECK(t.joints >= 3);
        CHECK(t.jointsPlain * 2 <= t.joints + 1);   // (a plain joint is shaped the same; its rings' caps just cut)
        // RT_TREE_WIRE=<dir>: the bark round the lowest forks, shaded and wired, the joints' triangles tinted
        if (const char* dir = std::getenv("RT_TREE_WIRE")) {
            Vec3 lo(1e9, 1e9, 1e9);
            for (std::size_t k = t.jointIndexStart; k < t.jointIndexEnd; ++k) {
                const Vec3& p = m.vertices[m.indices[k]].position;
                if (p.y < lo.y) lo = p;
            }
            const double half = std::string(name) == "spruce" ? 0.8 : 1.4;
            const Vec3 ctr(lo.x, lo.y + half * 0.7, lo.z);
            const int W = 900, H = 900;
            const double yaw = 0.5, tilt = -0.25;
            auto view = [&](const Vec3& p) {   // turned about y, tipped about x; x right, y up, z depth
                const Vec3 q = p - ctr;
                const double x1 = q.x * std::cos(yaw) - q.z * std::sin(yaw), z1 = q.x * std::sin(yaw) + q.z * std::cos(yaw);
                const double y2 = q.y * std::cos(tilt) - z1 * std::sin(tilt), z2 = q.y * std::sin(tilt) + z1 * std::cos(tilt);
                return Vec3((x1 / half * 0.5 + 0.5) * W, (0.5 - y2 / half * 0.5) * H, z2);
            };
            std::vector<double> zb(static_cast<std::size_t>(W) * H, 1e9);
            Image img(W, H);
            for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) img.setPixel(x, y, Vec3(0.85, 0.87, 0.9));
            const Vec3 light = normalize(Vec3(-0.4, 0.7, -0.6));
            auto drawTris = [&](bool wire) {
                for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
                    const Vec3 a = view(m.vertices[m.indices[k]].position), b = view(m.vertices[m.indices[k + 1]].position),
                               c = view(m.vertices[m.indices[k + 2]].position);
                    const bool joint = k >= t.jointIndexStart && k < t.jointIndexEnd;
                    if (!wire) {
                        const Vec3 wn = normalize(cross(m.vertices[m.indices[k + 1]].position - m.vertices[m.indices[k]].position,
                                                        m.vertices[m.indices[k + 2]].position - m.vertices[m.indices[k]].position));
                        const double sh = 0.35 + 0.65 * std::max(0.0, std::fabs(dot(wn, light)));
                        bool sliver = false;
                        if (joint) {
                            const Vec3 &A = m.vertices[m.indices[k]].position, &B = m.vertices[m.indices[k + 1]].position, &Cc = m.vertices[m.indices[k + 2]].position;
                            auto ang = [](const Vec3& p, const Vec3& q, const Vec3& r) {
                                const Vec3 u = q - p, w = r - p;
                                return std::acos(std::clamp(dot(u, w) / std::max(Real(1e-18), u.length() * w.length()), Real(-1), Real(1)));
                            };
                            sliver = std::min({ang(A, B, Cc), ang(B, Cc, A), ang(Cc, A, B)}) < 8.0 * 3.14159265 / 180.0;
                        }
                        const Vec3 col = (sliver ? Vec3(0.9, 0.1, 0.1) : joint ? Vec3(0.85, 0.55, 0.30) : Vec3(0.55, 0.45, 0.38)) * sh;
                        const int x0 = std::max(0, static_cast<int>(std::min({a.x, b.x, c.x}))), x1 = std::min(W - 1, static_cast<int>(std::max({a.x, b.x, c.x})) + 1);
                        const int y0 = std::max(0, static_cast<int>(std::min({a.y, b.y, c.y}))), y1 = std::min(H - 1, static_cast<int>(std::max({a.y, b.y, c.y})) + 1);
                        const double area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
                        if (std::fabs(area) < 1e-9) continue;
                        for (int y = y0; y <= y1; ++y)
                            for (int x = x0; x <= x1; ++x) {
                                const double px = x + 0.5, py = y + 0.5;
                                const double w0 = ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) / area;
                                const double w1 = ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) / area;
                                const double w2 = 1.0 - w0 - w1;
                                if (w0 < 0 || w1 < 0 || w2 < 0) continue;
                                const double z = w0 * a.z + w1 * b.z + w2 * c.z;
                                const std::size_t o = static_cast<std::size_t>(y) * W + x;
                                if (z < zb[o]) { zb[o] = z; img.setPixel(x, y, col); }
                            }
                    } else {
                        const Vec3 pts[3] = {a, b, c};
                        for (int e = 0; e < 3; ++e) {
                            const Vec3 p = pts[e], q = pts[(e + 1) % 3];
                            const int n = static_cast<int>(std::max(std::fabs(q.x - p.x), std::fabs(q.y - p.y))) + 1;
                            for (int i = 0; i <= n; ++i) {
                                const double u = static_cast<double>(i) / n;
                                const int x = static_cast<int>(p.x + (q.x - p.x) * u), y = static_cast<int>(p.y + (q.y - p.y) * u);
                                if (x < 0 || y < 0 || x >= W || y >= H) continue;
                                const double z = p.z + (q.z - p.z) * u;
                                if (z <= zb[static_cast<std::size_t>(y) * W + x] + 0.01)
                                    img.setPixel(x, y, joint ? Vec3(0.45, 0.15, 0.02) : Vec3(0.12, 0.1, 0.1));
                            }
                        }
                    }
                }
            };
            drawTris(false);
            drawTris(true);
            img.writePng(std::string(dir) + "/wire_" + name + ".png");
        }
    }
}

// NEAR LEAVES (foliage_kit.h LeafKitSpec; Glenn: the card leaves "look bad and also really big especially for sakura"):
// grown with foliage = 2 a broadleaf wears real-sized leaves one by one on leafy shoots, within its budget, and a
// sakura its blossoms as small cups -- every piece near the tree (within a shoot's reach of a limb).
TEST_CASE(near_leaves_are_one_by_one_on_shoots) {
    const TreeRecipeBook book = shipped();
    for (const char* name : {"oak", "birch", "maple", "sakura"}) {
        const TreeRecipe* r = book.find(name);
        CHECK(r && r->leafKit.set);
        if (!r) continue;
        const TreeMesh t = growRecipeTree(*r, 2u, {{"foliage", 2.0}});
        CHECK(t.foliage == 2);
        const LeafKitSpec& lk = r->leafKit;
        // a leaf is 9 vertices, a blossom 1 + 7
        const std::size_t verts = t.leaves.vertices.size();
        std::printf("    [leaves] %-7s %zu leaf tris, %zu bark tris\n", name, t.leaves.indices.size() / 3, t.branches.indices.size() / 3);
        CHECK(verts > 1000);
        CHECK(verts <= static_cast<std::size_t>(lk.budget) * 9 + static_cast<std::size_t>(lk.blossomBudget) * 8 + 9 * 64);
        int off = 0;
        const Real reach = lk.shootLength * 1.3 + lk.size * 1.5 + lk.blossomSize * 2.0 + 0.1;
        for (std::size_t v = 0; v < verts; v += 97) {
            const Vec3 p = t.leaves.vertices[v].position;
            Real best = 1e9;
            for (const TreeMesh::Limb& l : t.limbs) {
                const Vec3 ab = l.b - l.a;
                const Real s = std::clamp(dot(p - l.a, ab) / std::max(Real(1e-12), dot(ab, ab)), Real(0), Real(1));
                best = std::min(best, (p - (l.a + ab * s)).length());
            }
            off += best > reach;
        }
        CHECK(off == 0);
    }
}
