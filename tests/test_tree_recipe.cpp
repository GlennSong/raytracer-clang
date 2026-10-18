// Tree recipes (procgen/tree_recipe.h): species as Lua data, grown by C++ -- the tree lab's trees.

#include "test_framework.h"
#include "../src/engine/procgen/tree_recipe.h"
#include "../src/engine/procgen/tree_lods.h"
#include "../src/engine/procgen/branch_cards.h"
#include "../src/engine/scripting/tree_recipes_lua.h"
#include "../src/engine/procgen/leaf_card.h"
#include "../src/image.h"
#include "../src/engine/procgen/material_recipes.h"

#include <algorithm>
#include <chrono>
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


// The bark round the lowest of the triangles [hiStart, hiEnd) (index positions), shaded and wired, those tinted and their
// slivers red, `half` metres either side of the middle: a PNG at `path` (RT_TREE_WIRE in the tests)
void writeWire(const TreeMesh& t, std::size_t hiStart, std::size_t hiEnd, const std::string& path, double half,
               double yaw = 0.5, double tilt = -0.25) {
    const RenderMesh& m = t.branches;
    Vec3 lo(1e9, 1e9, 1e9);
    for (std::size_t k = hiStart; k < hiEnd; ++k) {
        const Vec3& p = m.vertices[m.indices[k]].position;
        if (p.y < lo.y) lo = p;
    }
    if (hiStart >= hiEnd) lo = Vec3(0, 0, 0);   // (nothing to tint: about the foot)
    Vec3 ctr(lo.x, lo.y + half * 0.7, lo.z);
    if (const char* at = std::getenv("RT_WIRE_AT")) {   // "x y z half yaw tilt": look there instead
        double x, y, z, h, yw, tl;
        if (std::sscanf(at, "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &h, &yw, &tl) == 6) { ctr = Vec3(x, y, z); half = h; yaw = yw; tilt = tl; }
    }
    const int W = 900, H = 900;
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
            const bool joint = k >= hiStart && k < hiEnd;
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
    img.writePng(path);

}

// RT_TIDY_SET="sprigSize=0.8,woodEdge=0.05": knobs to grow the measured trees with (exploring; empty in the suite)
std::map<std::string, double> knobsFromEnv() {
    std::map<std::string, double> set;
    if (const char* env = std::getenv("RT_TIDY_SET")) {
        std::stringstream ss(env);
        std::string kv;
        while (std::getline(ss, kv, ','))
            if (const auto eq = kv.find('='); eq != std::string::npos) set[kv.substr(0, eq)] = std::stod(kv.substr(eq + 1));
    }
    return set;
}

// THE SKELETON'S MESS (Glenn, 2026-10-18: "some trees look like branching messes"), from the limbs a tree was skinned
// on: CROWDED forks (four or more limbs of wood meeting at one point -- the joint planner's arms: the limb in and
// every limb out at least 0.4 jointMinRadius and 0.3 the fork's radius), forks TOO CLOSE (two such forks joined by an
// internode shorter than their radii together, so their joints overlap), and CROSSINGS (two limbs that share no end
// running through each other).
struct SkeletonMess { int crowded = 0, close = 0, crossings = 0, thickCrossings = 0, forks = 0, stubby = 0; };
SkeletonMess skeletonMess(const TreeMesh& t, Real jointMinRadius, FILE* dump = nullptr) {
    SkeletonMess m;
    if (dump)
        for (const TreeMesh::Limb& l : t.limbs)
            std::fprintf(dump, "L %f %f %f %f %f %f %f %f\n", l.a.x, l.a.y, l.a.z, l.b.x, l.b.y, l.b.z, l.ra, l.rb);
    auto key = [](const Vec3& p) { return std::make_tuple(std::lround(p.x * 1e4), std::lround(p.y * 1e4), std::lround(p.z * 1e4)); };
    std::map<std::tuple<long, long, long>, std::vector<std::size_t>> outOf;
    for (std::size_t i = 0; i < t.limbs.size(); ++i) outOf[key(t.limbs[i].a)].push_back(i);
    auto thickOuts = [&](const Vec3& at, Real r) {
        int n = 0;
        auto it = outOf.find(key(at));
        if (it != outOf.end())
            for (std::size_t i : it->second) n += t.limbs[i].rb >= 0.4 * jointMinRadius && t.limbs[i].rb >= 0.3 * r;
        return n;
    };
    for (const TreeMesh::Limb& l : t.limbs) {
        m.stubby += (l.b - l.a).length() < l.ra + l.rb;   // (an internode shorter than it is thick)
        if (l.rb < jointMinRadius) continue;
        const int outs = thickOuts(l.b, l.rb);
        if (outs < 2) continue;
        ++m.forks;
        if (outs + 1 >= 4) ++m.crowded;
        if (l.ra >= jointMinRadius && thickOuts(l.a, l.ra) >= 2 && (l.b - l.a).length() < l.ra + l.rb) ++m.close;
    }
    auto segDist = [](const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2) {
        const Vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
        const Real a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
        Real s = 0, u = 0;
        const Real c = dot(d1, r), b = dot(d1, d2), den = a * e - b * b;
        if (a < 1e-12 || e < 1e-12) return (p1 - p2).length();
        s = den > 1e-12 ? std::clamp((b * f - c * e) / den, Real(0), Real(1)) : 0;
        u = (b * s + f) / e;
        if (u < 0) { u = 0; s = std::clamp(-c / a, Real(0), Real(1)); }
        else if (u > 1) { u = 1; s = std::clamp((b - c) / a, Real(0), Real(1)); }
        return ((p1 + d1 * s) - (p2 + d2 * u)).length();
    };
    for (std::size_t i = 0; i < t.limbs.size(); ++i)
        for (std::size_t j = i + 1; j < t.limbs.size(); ++j) {
            const TreeMesh::Limb &A = t.limbs[i], &B = t.limbs[j];
            if (key(A.a) == key(B.a) || key(A.a) == key(B.b) || key(A.b) == key(B.a) || key(A.b) == key(B.b)) continue;
            const Real reach = 0.5 * (A.ra + A.rb) + 0.5 * (B.ra + B.rb);
            // (cheap reject on the boxes)
            const Real lo = std::max({std::min(A.a.y, A.b.y), std::min(B.a.y, B.b.y)}), hi = std::min({std::max(A.a.y, A.b.y), std::max(B.a.y, B.b.y)});
            if (lo - hi > reach) continue;
            if (segDist(A.a, A.b, B.a, B.b) < 0.8 * reach) {
                ++m.crossings;
                if (dump) std::fprintf(dump, "X %zu %zu\n", i, j);
                m.thickCrossings += std::min(A.ra, B.ra) >= jointMinRadius;   // (wood the joints are for)
            }
        }
    return m;
}

}  // namespace

// THE SHIPPED RECIPES (assets/scripts/tree_recipes.lua) load and every species grows a whole tree -- bark, leaves,
// a collider, nothing non-finite -- and the same seed grows the same tree twice.
TEST_CASE(shipped_tree_recipes_load_and_grow) {
    const TreeRecipeBook book = shipped();
    CHECK(book.size() >= 3);
    for (const std::string& name : book.names()) {
        const TreeRecipe* r = book.find(name);
        if (r->blades.set) continue;   // (blade plants: their own test)
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
        if (book.find(name)->blades.set) continue;
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
        CHECK(r->fronds.set || sizes.size() >= (r->shrub ? 3u : 4u));   // (a palm: one stem, its seeds its heights)
    }
}

// THE GNARL (Glenn: "trees aren't just tubes"): an oak's foot is fluted -- out of round by its lobes and knots --
// where the same oak with the gnarl pinned off is a round tube.
TEST_CASE(a_gnarled_trunk_is_not_a_tube) {
    const TreeRecipeBook book = shipped();
    const TreeRecipe* oak = book.find("oak");
    CHECK(oak != nullptr);
    if (!oak) return;
    // (the tube's own cross-section: the trunk zone off -- its surface is not cut in rings)
    const std::map<std::string, double> plain = {{"lobes", 0}, {"lobeDepth", 0}, {"barkNoise", 0}, {"nodeSwell", 0},
                                                 {"flare", 0}, {"crookChance", 0}, {"meander", 0}, {"lean", 0},
                                                 {"trunkZone", 0}};
    const TreeMesh g = growRecipeTree(*oak, 2, {{"trunkZone", 0}}), p = growRecipeTree(*oak, 2, plain);
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
            if (book.find(name)->blades.set || book.find(name)->fronds.set) continue;   // (palms: their own test)
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
    // (RT_JOINT_TREES="multistem:2,oak:5": other trees to look at, by name and seed)
    std::vector<std::pair<std::string, uint32_t>> trees;
    for (const char* name : {"oak", "birch", "sakura", "maple", "pine", "spruce"}) trees.push_back({name, 2u});
    if (const char* env = std::getenv("RT_JOINT_TREES")) {
        trees.clear();
        std::stringstream ss(env);
        std::string item;
        while (std::getline(ss, item, ',')) {
            const auto colon = item.find(':');
            trees.push_back({item.substr(0, colon), colon == std::string::npos ? 2u : static_cast<uint32_t>(std::stoul(item.substr(colon + 1)))});
        }
    }
    for (const auto& [nameStr, seed] : trees) {
        const char* name = nameStr.c_str();
        const TreeRecipe* r = book.find(name);
        CHECK(r != nullptr);
        if (!r) continue;
        std::map<std::string, double> set = knobsFromEnv();
        set["joints"] = 1.0;
        if (!set.count("trunkZone")) set["trunkZone"] = 0.0;   // (the hull joins: a trunk zone stands in for them)
        const TreeMesh t = growRecipeTree(*r, seed, set);
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
        if (const char* dir = std::getenv("RT_TREE_WIRE"))
            writeWire(t, t.jointIndexStart, t.jointIndexEnd, std::string(dir) + "/wire_" + name + ".png",
                      std::string(name) == "spruce" ? 0.8 : 1.4);
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

// BARK VARIETY (Glenn: "the birch texture is good, but looks to repetitive"): a birch's foot is dark, rising in
// tongues, and its bark carries patches -- the bark's vertex colours near the ground far darker than up the trunk,
// and not one colour up it.
TEST_CASE(a_birch_has_a_dark_foot_and_patchy_bark) {
    const TreeRecipeBook book = shipped();
    const TreeRecipe* r = book.find("birch");
    CHECK(r != nullptr);
    if (!r) return;
    for (const auto& ov : {std::map<std::string, double>{}, std::map<std::string, double>{{"foliage", 2.0}}}) {
    const TreeMesh t = growRecipeTree(*r, 1u, ov);
    double footSum = 0, upSum = 0, upMin = 9, upMax = 0;
    int footN = 0, upN = 0;
    for (const Vertex& v : t.branches.vertices) {
        const double lum = (v.color.x + v.color.y + v.color.z) / 3.0;
        if (v.position.y > 0.1 && v.position.y < 0.4 && std::hypot(v.position.x, v.position.z) < 0.25) { footSum += lum; ++footN; }
        if (v.position.y > 2.5 && v.position.y < 5.0 && std::hypot(v.position.x, v.position.z) < 0.6) {
            upSum += lum; ++upN; upMin = std::min(upMin, lum); upMax = std::max(upMax, lum);
        }
    }
    if (std::getenv("RT_BARK_BANDS")) {
        for (double y0 : {0.0, 0.4, 0.8, 1.2, 1.6, 2.0, 2.5, 3.0}) {
            double sum = 0; int cnt = 0;
            for (const Vertex& v : t.branches.vertices)
                if (v.position.y >= y0 && v.position.y < y0 + 0.4 && std::hypot(v.position.x, v.position.z) < 0.3) {
                    sum += (v.color.x + v.color.y + v.color.z) / 3.0; ++cnt;
                }
            std::printf("    [birch] y %.1f: %.2f (%d)\n", y0, cnt ? sum / cnt : -1.0, cnt);
        }
    }
    const double foot = footN ? footSum / footN : 0, up = upN ? upSum / upN : 0;
    std::printf("    [birch] foot %.2f (%d verts), trunk %.2f (%.2f .. %.2f, %d verts)\n", foot, footN, up, upMin, upMax, upN);
    CHECK(footN > 0 && upN > 0);
    CHECK(foot < 0.6 * up);
    CHECK(upMax - upMin > 0.08);
    }
}

// BLADE PLANTS (Glenn's photos: flax, rushes, fescue): a clump of strap leaves from a crown -- the blades within
// their length of the crown, pointed (narrow at the tip), the flax a FAN (its blades' bearings gathered to two
// sides), the fescue a low tuft.
TEST_CASE(blade_plants_are_clumps_of_strap_leaves) {
    const TreeRecipeBook book = shipped();
    for (const char* name : {"flax", "rush", "fescue"}) {
        const TreeRecipe* r = book.find(name);
        CHECK(r && r->blades.set);
        if (!r) continue;
        const TreeMesh t = growRecipeTree(*r, 2u);
        CHECK(t.foliage == 3);
        CHECK(t.leaves.vertices.size() == static_cast<std::size_t>(r->blades.count) * 3u * 7u);   // 7 stations, 3 across
        Real top = 0, reach = 0;
        for (const Vertex& v : t.leaves.vertices) {
            top = std::max(top, v.position.y);
            reach = std::max(reach, std::hypot(v.position.x, v.position.z));
        }
        std::printf("    [blades] %-7s %zu tris, %.2f m tall, %.2f m reach\n", name, t.leaves.indices.size() / 3, top, reach);
        CHECK(top > 0.15 && top < r->blades.length * 1.4);
        CHECK(reach < r->blades.length * 1.4 + r->blades.crown);
        if (std::string(name) == "fescue") CHECK(top < 0.5);
    }
}

// PALMS (Glenn: "the palm leaves ... don't quite look like palm leaves or trunks"): a ringed stem with a CROWN of fronds
// at its top -- every blade of the crown above half the trunk (old fronds and the skirt hang below the crown, but
// not to the ground), the crown wider than tall, the youngest fronds rising above the top, the oldest hanging below it.
TEST_CASE(a_palm_wears_its_crown_at_the_top) {
    const TreeRecipeBook book = shipped();
    for (const char* name : {"coconut_palm", "fan_palm"}) {
        const TreeRecipe* r = book.find(name);
        CHECK(r && r->fronds.set);
        if (!r) continue;
        const TreeMesh t = growRecipeTree(*r, 2u);
        CHECK(t.foliage == 4);
        Real trunkTop = 0;
        for (const TreeMesh::Limb& l : t.limbs) trunkTop = std::max(trunkTop, l.b.y);
        Real lo = 1e9, hi = -1e9, reach = 0;
        for (const Vertex& v : t.leaves.vertices) {
            lo = std::min(lo, v.position.y);
            hi = std::max(hi, v.position.y);
            reach = std::max(reach, std::hypot(v.position.x - t.limbs.back().b.x, v.position.z - t.limbs.back().b.z));
        }
        std::printf("    [palm] %-12s trunk %.1f m, crown %.1f .. %.1f m, reach %.1f m, %zu leaf tris\n", name, trunkTop, lo, hi,
                    reach, t.leaves.indices.size() / 3);
        CHECK(lo > trunkTop * 0.45);   // (old coconut fronds hang metres down; a skirt to under half way)
        CHECK(hi > trunkTop + 0.5);
        CHECK(reach > 1.2 * (hi - trunkTop));
    }
}

// A RECIPE TREE'S LEVELS (tree_lods.h; Glenn, 2026-10-17, of a forest of card trees: "you can tell they're cards"):
// hero, mid and light each lighter than the last, with the crown's leaf area about kept. A species without branch
// cards wears the same leaves coarser at the mid and light -- leaves, not crossed cards; one with them
// (branch_cards.h) wears its branch cards at every level, its wood coarser each time.
TEST_CASE(a_recipe_trees_levels_get_lighter_and_keep_their_leaves) {
    const TreeRecipeBook book = shipped();
    RecipeLodBands bands;
    bands.mid0 = 58; bands.mid1 = 70;
    auto area = [](const RenderMesh& m) {
        double a = 0;
        for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3)
            a += 0.5 * cross(m.vertices[m.indices[i + 1]].position - m.vertices[m.indices[i]].position,
                             m.vertices[m.indices[i + 2]].position - m.vertices[m.indices[i]].position).length();
        return a;
    };
    for (const char* name : {"oak", "birch", "maple", "sakura", "river_birch", "magnolia"}) {
        const TreeRecipe* r = book.find(name);
        CHECK(r != nullptr);
        if (!r) continue;
        const std::vector<RecipeTreeLevel> L = growRecipeLevels(*r, 3, {}, bands);
        CHECK(L.size() == 3);
        if (L.size() != 3) continue;
        std::size_t tris[3];
        for (int i = 0; i < 3; ++i) tris[i] = (L[i].tree.branches.indices.size() + L[i].tree.leaves.indices.size()) / 3;
        const double a0 = area(L[0].tree.leaves), a2 = area(L[2].tree.leaves);
        std::printf("    [levels] %-11s hero %zu, mid %zu, light %zu tris (leaf area %.0f / %.0f / %.0f m2)\n", name, tris[0],
                    tris[1], tris[2], a0, area(L[1].tree.leaves), a2);
        if (r->branchCards.set) {
            CHECK(tris[1] < tris[0]);
            // (a species with near leaves: the hero's one by one, twig segment cards past it)
            CHECK(L[0].tree.foliage == (r->leafKit.set && r->branchCards.twigs ? 2 : 5) && L[1].tree.foliage == 5 && L[2].tree.foliage == 5);
        } else {
            CHECK(tris[1] < tris[0] / 2);
            CHECK(L[1].tree.foliage == 2 && L[2].tree.foliage == 2);   // leaves, not cards
        }
        CHECK(tris[2] < tris[1]);
        CHECK(a2 > 0.4 * a0 && a2 < 2.5 * a0);
        CHECK(L[0].out1 > L[0].out0 && L[1].in0 == L[0].out0 && L[1].out1 == 70.0f && L[2].in1 == 70.0f);
    }
}

// THE SKELETON IS TIDY (Glenn, 2026-10-18: "some trees look like branching messes"): see skeletonMess.
TEST_CASE(a_trees_skeleton_is_tidy) {
    const TreeRecipeBook book = shipped();
    for (const char* name : {"oak", "birch", "sakura", "maple", "pine", "spruce", "river_birch", "magnolia"}) {
        const TreeRecipe* r = book.find(name);
        CHECK(r != nullptr);
        if (!r) continue;
        const std::map<std::string, double> set = knobsFromEnv();
        TreeParams tp = drawTreeParams(*r, 2u);
        for (const auto& [k, v] : set) setTreeParam(tp, k, v);
        const TreeMesh t = growRecipeTree(*r, 2u, set);
        FILE* dump = nullptr;
        if (const char* dir = std::getenv("RT_TIDY_DUMP")) dump = std::fopen((std::string(dir) + "/" + name + ".txt").c_str(), "w");
        const SkeletonMess m = skeletonMess(t, tp.jointMinRadius, dump);
        if (dump) std::fclose(dump);
        std::printf("    [tidy] %-11s %4d forks: %3d crowded, %3d too close; %4d crossings (%3d of wood); %zu limbs (%d stubby); bark %zu tris (joints %zu), %zu sprigs, leaves %zu\n",
                    name, m.forks, m.crowded, m.close, m.crossings, m.thickCrossings, t.limbs.size(), m.stubby, t.branches.indices.size() / 3,
                    (t.jointIndexEnd - t.jointIndexStart) / 3, t.sprigs.size(), t.leaves.indices.size() / 3);
    }
}

// THE BRANCH CARDS (branch_cards.h; Glenn, 2026-10-18: "for a hero tree ... the upper budget to be about 8000 triangles
// and no more"): a recipe with branch_cards grows its hero with the wood to sprigs, built by need, inside 8000
// triangles, and every sprig wearing a card of the atlas baked from its own sprigs -- each picture drawn (covering a
// fair share of its cell), each card hanging where its sprig grew (its base on a limb of the tree). The mid's wood is
// lighter than the hero's, and wears the same atlas.
TEST_CASE(a_card_trees_wood_is_held_to_budget_and_wears_its_sprigs) {
    const TreeRecipeBook book = shipped();
    int trees = 0;
    for (const std::string& name : book.names()) {
        const TreeRecipe* r = book.find(name);
        if (!r || !r->branchCards.set) continue;
        ++trees;
        BranchCardSet cache;
        const std::vector<RecipeTreeLevel> levels = growRecipeLevels(*r, 2u, {}, RecipeLodBands{}, true, &cache);
        CHECK(levels.size() == 2);
        if (levels.size() < 2) continue;
        const TreeMesh& hero = levels[0].tree;
        const std::size_t bark = hero.branches.indices.size() / 3, cards = hero.leaves.indices.size() / 3;
        {
            std::size_t leafy = 0;
            double len = 0;
            for (const TreeMesh::Twig& t : hero.twigs) { leafy += t.leafy; len += (t.b - t.a).length(); }
            std::printf("    [twigs] %-14s %zu twigs (%zu leafy), %.0f m\n", name.c_str(), hero.twigs.size(), leafy, len);
        }
        std::printf("    [cards] %-14s hero bark %5zu tris (joints %4zu), %3zu sprigs -> %4zu card tris; mid bark %5zu; %zu pictures\n",
                    name.c_str(), bark, (hero.jointIndexEnd - hero.jointIndexStart) / 3, hero.sprigs.size(), cards,
                    levels[1].tree.branches.indices.size() / 3, cache.pictures.size());
        // (a near hero: the trunk and limbs in 8000, its fine twigs' bark besides -- branch_cards.h wearNearLeaves)
        const bool near = r->leafKit.set && r->branchCards.twigs;
        CHECK(bark <= 8000 + (near ? static_cast<std::size_t>(r->branchCards.nearTwigTris) : 0));
        if (const char* dir = std::getenv("RT_CARD_ATLAS")) {   // the baked pictures, alpha over grey
            const TextureData& a = cache.atlas.albedo;
            Image img(a.width, a.height);
            for (int y = 0; y < a.height; ++y)
                for (int x = 0; x < a.width; ++x) {
                    const uint8_t* px = &a.pixels[(static_cast<std::size_t>(y) * a.width + x) * 4];
                    const double al = px[3] / 255.0;
                    img.setPixel(x, y, Vec3(px[0] / 255.0, px[1] / 255.0, px[2] / 255.0) * al + Vec3(0.6, 0.62, 0.66) * (1.0 - al));
                }
            img.writePng(std::string(dir) + "/atlas_" + name + ".png");
            // ...and LIT by its normal map from the upper left (what the shader makes of it)
            const TextureData& nm = cache.atlas.normal;
            const Vec3 L = normalize(Vec3(-0.5, 0.6, 0.65));
            Image lit(a.width, a.height);
            for (int y = 0; y < a.height; ++y)
                for (int x = 0; x < a.width; ++x) {
                    const uint8_t* px = &a.pixels[(static_cast<std::size_t>(y) * a.width + x) * 4];
                    const uint8_t* nx = &nm.pixels[(static_cast<std::size_t>(y) * nm.width + x) * 4];
                    const Vec3 n = normalize(Vec3(nx[0] / 127.5 - 1.0, nx[1] / 127.5 - 1.0, nx[2] / 127.5 - 1.0));
                    const double al = px[3] / 255.0, sh = 0.3 + 0.9 * std::max(0.0, static_cast<double>(dot(n, L)));
                    lit.setPixel(x, y, Vec3(px[0] / 255.0, px[1] / 255.0, px[2] / 255.0) * (sh * al) + Vec3(0.6, 0.62, 0.66) * (1.0 - al));
                }
            lit.writePng(std::string(dir) + "/lit_" + name + ".png");
        }
        if (const char* dir = std::getenv("RT_TREE_WIRE"))   // (the hero as the lab and the forest draw it)
            writeWire(hero, hero.zoneIndexEnd > hero.zoneIndexStart ? hero.zoneIndexStart : 0,
                      hero.zoneIndexEnd > hero.zoneIndexStart ? hero.zoneIndexEnd : hero.branches.indices.size(),
                      std::string(dir) + "/hero_" + name + ".png", 2.2);
        CHECK(!hero.sprigs.empty());
        const bool twigCards = near;
        const TreeMesh& carded = near ? levels[1].tree : hero;   // (a near hero's cards are its mid's)
        CHECK(carded.foliage == 5);
        if (near) {   // the hero: its leaves one by one, within their budget
            CHECK(hero.foliage == 2);
            CHECK(cards >= 4 * 8 * static_cast<std::size_t>(r->branchCards.nearLeaves) / 10);
            CHECK(cards <= 8 * static_cast<std::size_t>(r->branchCards.nearLeaves) + 30000);
        }
        if (twigCards) {   // (a twig segment along every fine internode -- two planes each -- and the fill besides)
            CHECK(!carded.twigs.empty());
            CHECK(carded.leaves.indices.size() / 3 >= carded.twigs.size() * 4);
            for (const BranchCardSet::Picture& pic : cache.pictures) CHECK(pic.kind >= 0 && pic.kind < 4);
        } else {
            CHECK(cards == hero.sprigs.size() * 4 * static_cast<std::size_t>(r->branchCards.views));
        }
        CHECK(levels[1].tree.branches.indices.size() < hero.branches.indices.size());
        CHECK(levels[1].atlas.albedo.width == cache.atlas.albedo.width);
        // every picture drawn
        for (const BranchCardSet::Picture& pic : cache.pictures)
          for (const BranchCardSet::View& p : pic.views) {
            const TextureData& a = cache.atlas.albedo;
            const int x0 = static_cast<int>(p.u0 * a.width), x1 = static_cast<int>(p.u1 * a.width);
            const int y0 = static_cast<int>(p.v0 * a.height), y1 = static_cast<int>(p.v1 * a.height);
            std::size_t on = 0, all = 0;
            for (int y = y0; y < y1; ++y)
                for (int x = x0; x < x1; ++x, ++all) on += a.pixels[(static_cast<std::size_t>(y) * a.width + x) * 4 + 3] > 127;
            // (a bare twig is a thin line in its cell: a fortieth is drawn)
            CHECK(on * (pic.kind >= 2 ? 60 : 20) >= all);
            if (on * (pic.kind >= 2 ? 60 : 20) < all) std::printf("    [cards] %s: a picture %.1f%% drawn\n", name.c_str(), 100.0 * on / all);
          }
        // every card hangs on the tree: its base (the first vertex: x0 at the sprig's base) beside a limb -- or, twig
        // segments, every card's corners within a segment's reach of the tree's limbs
        int off = 0;
        if (twigCards)
            for (std::size_t v = 0; v < carded.leaves.vertices.size(); v += 4) {
                const Vec3 p = carded.leaves.vertices[v].position;
                Real best = 1e9;
                for (const TreeMesh::Limb& l : carded.limbs) {
                    const Vec3 ab = l.b - l.a;
                    const Real t = std::clamp(dot(p - l.a, ab) / std::max(Real(1e-12), dot(ab, ab)), Real(0), Real(1));
                    best = std::min(best, (p - (l.a + ab * t)).length());
                }
                off += best > 2.0 * cache.twigLength + 0.1;
            }
        for (std::size_t c = 0; c < (twigCards ? 0 : hero.sprigs.size()); ++c) {
            const Vec3 b = hero.sprigs[c].base;
            Real best = 1e9;
            for (const TreeMesh::Limb& l : hero.limbs) {
                const Vec3 ab = l.b - l.a;
                const Real s = std::clamp(dot(b - l.a, ab) / std::max(Real(1e-12), dot(ab, ab)), Real(0), Real(1));
                best = std::min(best, (b - (l.a + ab * s)).length());
            }
            off += best > 0.05;
            const Vec3 v = hero.leaves.vertices[c * 6 * static_cast<std::size_t>(r->branchCards.views)].position;   // (its first corner)
            off += (v - b).length() > 2.0 * (hero.sprigs[c].length + hero.sprigs[c].halfWidth);
        }
        CHECK(off == 0);
    }
    CHECK(trees >= 6);
}

// THE TRUNK ZONE (tree.h TreeParams::trunkZone, ADR-0183; Glenn, 2026-10-18: "a full on interconnected trunk
// structure"): the trunk, its thick limbs and its roots are one implicit surface -- CLOSED (every edge, by position,
// on at least two of its triangles: no hole; a few edges in a thousand on four, where two limbs touch inside a cell and
// the mesh is split there by index), FACING OUT (each triangle wound to the side its normals point), EVEN (no sliver:
// Glenn, "those triangles look really stretched"), near its triangle budget, and the whole wood with it under the
// hero's 8000. RT_TREE_WIRE renders its foot; RT_JOINT_TREES picks the trees (name:seed).
TEST_CASE(a_trees_trunk_zone_is_one_closed_surface) {
    const TreeRecipeBook book = shipped();
    std::vector<std::pair<std::string, uint32_t>> trees{{"oak", 2u}, {"oak", 3u}, {"oak", 5u}};
    if (const char* env = std::getenv("RT_JOINT_TREES")) {
        trees.clear();
        std::stringstream ss(env);
        std::string item;
        while (std::getline(ss, item, ',')) {
            const auto colon = item.find(':');
            trees.push_back({item.substr(0, colon), colon == std::string::npos ? 2u : static_cast<uint32_t>(std::stoul(item.substr(colon + 1)))});
        }
    }
    for (const auto& [name, seed] : trees) {
        const TreeRecipe* r = book.find(name);
        CHECK(r != nullptr);
        if (!r) continue;
        std::map<std::string, double> set{{"foliage", 2.0}, {"sprigSize", 0.8}, {"woodEdge", 0.06}, {"ringAngle", 12.0},
                                          {"trunkZone", 0.10}, {"trunkZoneTris", 2800.0}};
        for (const auto& [k, v] : knobsFromEnv()) set[k] = v;
        const auto t0 = std::chrono::steady_clock::now();
        const TreeMesh t = growRecipeTree(*r, seed, set);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        const RenderMesh& m = t.branches;
        const std::size_t zoneTris = (t.zoneIndexEnd - t.zoneIndexStart) / 3;
        // closed: edges by position
        auto key = [](const Vec3& p) { return std::make_tuple(std::lround(p.x * 1e5), std::lround(p.y * 1e5), std::lround(p.z * 1e5)); };
        std::map<std::pair<std::tuple<long, long, long>, std::tuple<long, long, long>>, int> edges;
        int outward = 0;
        for (std::size_t k = t.zoneIndexStart; k + 2 < t.zoneIndexEnd; k += 3) {
            const Vertex* v[3] = {&m.vertices[m.indices[k]], &m.vertices[m.indices[k + 1]], &m.vertices[m.indices[k + 2]]};
            for (int e = 0; e < 3; ++e) {
                auto a = key(v[e]->position), b = key(v[(e + 1) % 3]->position);
                if (b < a) std::swap(a, b);
                ++edges[{a, b}];
            }
            // (the engine's front is clockwise from the normal's side: cross(c - a, b - a) points out)
            const Vec3 fn = cross(v[2]->position - v[0]->position, v[1]->position - v[0]->position);
            outward += dot(fn, v[0]->normal + v[1]->normal + v[2]->normal) > 0.0;
        }
        std::size_t slivers = 0;
        for (std::size_t k = t.zoneIndexStart; k + 2 < t.zoneIndexEnd; k += 3) {
            const Vec3 &A = m.vertices[m.indices[k]].position, &B = m.vertices[m.indices[k + 1]].position, &C = m.vertices[m.indices[k + 2]].position;
            auto ang = [](const Vec3& p, const Vec3& q, const Vec3& r) {
                const Vec3 u = q - p, w = r - p;
                return std::acos(std::clamp(dot(u, w) / std::max(Real(1e-18), u.length() * w.length()), Real(-1), Real(1)));
            };
            slivers += std::min({ang(A, B, C), ang(B, C, A), ang(C, A, B)}) < 8.0 * 3.14159265 / 180.0;
        }
        int open = 0, over = 0;
        for (const auto& [e, n] : edges) {
            open += n == 1; over += n > 2;
            if (n != 2 && open + over <= 40)
                std::printf("    [zone] bad edge (%d faces) at (%.3f %.3f %.3f)\n", n, std::get<0>(e.first) * 1e-5, std::get<1>(e.first) * 1e-5, std::get<2>(e.first) * 1e-5);
        }
        std::printf("    [zone] %-6s seed %u: %d limbs, %d raw -> %zu tris (%zu verts); %d open edges, %d shared by 3+, %.1f%% outward, %.1f%% slivers; "
                    "bark %zu tris in all; %.0f ms\n",
                    name.c_str(), seed, t.zoneLimbs, t.zoneRawTris, zoneTris, m.vertices.size(), open, over,
                    zoneTris ? 100.0 * outward / zoneTris : 0.0, zoneTris ? 100.0 * slivers / zoneTris : 0.0, m.indices.size() / 3, ms);
        CHECK(t.zoneLimbs >= 3);
        CHECK(zoneTris > 0 && zoneTris <= 2800 + 2800 / 10);
        CHECK(open == 0 && over * 500 <= static_cast<int>(edges.size()));   // (touching sheets: a few in a thousand at most)
        CHECK(slivers * 100 <= zoneTris);
        CHECK(outward * 100 >= static_cast<int>(zoneTris) * 99);
        CHECK(m.indices.size() / 3 <= 8000);
        if (const char* dir = std::getenv("RT_TREE_WIRE")) {
            writeWire(t, t.zoneIndexStart, t.zoneIndexEnd, std::string(dir) + "/zone_" + name + "_" + std::to_string(seed) + ".png", 2.2);
            writeWire(t, t.zoneIndexStart, t.zoneIndexEnd, std::string(dir) + "/zone_" + name + "_" + std::to_string(seed) + "_far.png", 4.5, 1.6, -0.1);
        }
    }
}
