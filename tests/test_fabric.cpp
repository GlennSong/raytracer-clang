// P7 patch-conforming fabric (8km-city plan): the constructive proofs. The
// three nasty fixtures for fabricChords (banana, acute-corner quad,
// sub-block), fabricBisect's round-13 node hygiene, and the piedmont-recipe
// integration — fabric present, no dead ends minted, and every gate
// test_metro_sites holds for the legacy fill still holds with fabric on.
#include "test_framework.h"

#include "../src/engine/procgen/city/metro.h"
#include "../src/engine/procgen/city/patch_fabric.h"
#include "../src/engine/procgen/city/road_constraints.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <queue>
#include <vector>

using namespace engine;

namespace {

constexpr double kPi = 3.14159265358979323846;

double segDist(const Vec2& a, const Vec2& b, const Vec2& p) {
    Vec2 ab = b - a;
    double len2 = ab.lengthSquared();
    double t = len2 > 1e-12 ? dot(p - a, ab) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    return (p - (a + ab * t)).length();
}

double boundaryDist(const Poly2& poly, const Vec2& p) {
    double best = 1e30;
    for (std::size_t i = 0; i < poly.size(); ++i)
        best = std::min(
            best, segDist(poly[i], poly[(i + 1) % poly.size()], p));
    return best;
}

// Weld boundary + fabric into one planar graph — the same treatment the
// metro's planarize gives the fill (boundary chains T-split at stations).
RoadGraph fabricGraph(const Poly2& boundary,
                      const std::vector<FabricSegment>& segs) {
    RoadGraph g;
    int first = -1, prev = -1;
    for (const Vec2& v : boundary) {
        int id = g.addNode(v, 0.25);
        if (prev >= 0 && id != prev) g.addEdge(prev, id, 17.0, RoadClass::Arterial);
        if (first < 0) first = id;
        prev = id;
    }
    if (first >= 0 && prev >= 0 && prev != first)
        g.addEdge(prev, first, 17.0, RoadClass::Arterial);
    for (const FabricSegment& s : segs) {
        int a = g.addNode(s.a, 0.25), b = g.addNode(s.b, 0.25);
        if (a != b) g.addEdge(a, b, s.width, s.klass);
    }
    return planarize(g, 0.5);
}

// Dead ends strictly INSIDE the patch (off the boundary) are the fabric
// defect every round outlawed: a chord/cut endpoint must land on the
// boundary or a crossing.
int degree1Interior(const RoadGraph& g, const Poly2& boundary) {
    std::vector<int> deg(g.nodes.size(), 0);
    for (const RoadEdge& e : g.edges) {
        if (e.a == e.b) continue;
        ++deg[e.a];
        ++deg[e.b];
    }
    int bad = 0;
    for (std::size_t i = 0; i < g.nodes.size(); ++i)
        if (deg[i] == 1 && boundaryDist(boundary, g.nodes[i].pos) > 1.5) ++bad;
    return bad;
}

bool sameSegments(const std::vector<FabricSegment>& a,
                  const std::vector<FabricSegment>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if ((a[i].a - b[i].a).length() > 1e-12 ||
            (a[i].b - b[i].b).length() > 1e-12)
            return false;
    return true;
}

PatchFabricParams cityParams() {
    PatchFabricParams p;
    p.blockLen = 150.0;
    p.blockDepth = 150.0;
    p.streetWidth = 11.0;
    p.seed = 7;
    p.conform = 0.15;
    p.jitter = 0.12;
    return p;
}

// Crescent between two sharp tips: width `bulge` at the middle, zero at the
// ends, bent along a radius-400 arc through `sweepDeg`.
FabricPatch banana(double bulge, double sweepDeg) {
    FabricPatch patch;
    const int steps = 24;
    const double sweep = sweepDeg * kPi / 180.0;
    auto rim = [&](double t, double sign) {
        double ang = -sweep * 0.5 + sweep * t;
        double r = 400.0 + sign * bulge * std::sin(kPi * t);
        return Vec2(std::cos(ang) * r, std::sin(ang) * r);
    };
    for (int i = 0; i <= steps; ++i)
        patch.boundary.push_back(rim(i / static_cast<double>(steps), 1.0));
    for (int i = steps - 1; i >= 1; --i)
        patch.boundary.push_back(rim(i / static_cast<double>(steps), -1.0));
    patch.corners = {0, steps};   // the two tips
    return patch;
}

}  // namespace

TEST_CASE(fabric_banana_decomposed_or_empty_never_dangling) {
    PatchFabricParams p = cityParams();
    // Thin, strongly bent: the centroid falls in the hole -> refuse, emit
    // NOTHING (never guess on a patch the construction cannot prove).
    {
        FabricPatch thin = banana(45.0, 160.0);
        std::vector<FabricSegment> segs = fabricChords(thin, p);
        CHECK(segs.empty());
    }
    // Fat crescent: the 2-corner patch midpoint-decomposes into quads; every
    // endpoint lands on the boundary or a crossing — zero interior dangles.
    {
        FabricPatch fat = banana(220.0, 110.0);
        std::vector<FabricSegment> segs = fabricChords(fat, p);
        std::printf("        [banana] fat crescent -> %zu segments\n",
                    segs.size());
        if (!segs.empty()) {
            RoadGraph g = fabricGraph(fat.boundary, segs);
            CHECK(degree1Interior(g, fat.boundary) == 0);
        }
        // determinism
        CHECK(sameSegments(segs, fabricChords(fat, p)));
    }
}

TEST_CASE(fabric_acute_quad_t_terminations_only_at_crossings) {
    // Skewed trapezoid, mismatched side stationing beyond the 1.3x snap:
    // the dense side's extras must T-TERMINATE on the cross family — never
    // stop mid-block (no degree-1 interior nodes), never run past.
    PatchFabricParams p = cityParams();
    FabricPatch patch;
    patch.boundary = {{0, 0}, {900, 0}, {1450, 600}, {50, 450}};
    patch.corners = {0, 1, 2, 3};
    std::vector<FabricSegment> segs = fabricChords(patch, p);
    CHECK(!segs.empty());
    RoadGraph g = fabricGraph(patch.boundary, segs);
    CHECK(degree1Interior(g, patch.boundary) == 0);
    // T's exist: interior degree-3 nodes are the round-11 terminations.
    std::vector<int> deg(g.nodes.size(), 0);
    for (const RoadEdge& e : g.edges) {
        ++deg[e.a];
        ++deg[e.b];
    }
    int interiorT = 0, interiorNodes = 0;
    for (std::size_t i = 0; i < g.nodes.size(); ++i) {
        if (boundaryDist(patch.boundary, g.nodes[i].pos) <= 1.5) continue;
        ++interiorNodes;
        if (deg[i] >= 3) ++interiorT;
    }
    std::printf("        [acute quad] %zu segs, %d interior nodes, %d crossings\n",
                segs.size(), interiorNodes, interiorT);
    CHECK(interiorT > 0);
    CHECK(sameSegments(segs, fabricChords(patch, p)));
}

TEST_CASE(fabric_sub_block_patch_emits_nothing) {
    PatchFabricParams p = cityParams();
    FabricPatch patch;
    patch.boundary = {{0, 0}, {200, 0}, {200, 120}, {0, 120}};
    patch.corners = {0, 1, 2, 3};
    CHECK(fabricChords(patch, p).empty());
    CHECK(fabricBisect(patch.boundary, p).empty());
    CHECK(fabricCourt(patch.boundary, p).empty());
}

TEST_CASE(fabric_bisect_node_hygiene_and_determinism) {
    PatchFabricParams p = cityParams();
    // Big irregular hexagonal patch — several recursion levels deep.
    Poly2 patch = {{0, 0},    {1250, -60}, {1600, 500},
                   {1300, 1050}, {350, 1150}, {-150, 520}};
    std::vector<FabricSegment> segs = fabricBisect(patch, p);
    CHECK(!segs.empty());
    // ROUND-13 HARD FLOOR: after collapse, no two distinct junctions
    // (cut endpoints) sit closer than hardCollapse.
    std::vector<Vec2> ends;
    for (const FabricSegment& s : segs) {
        ends.push_back(s.a);
        ends.push_back(s.b);
    }
    double worst = 1e30;
    for (std::size_t i = 0; i < ends.size(); ++i)
        for (std::size_t j = i + 1; j < ends.size(); ++j) {
            double d = (ends[i] - ends[j]).length();
            if (d > 1e-9) worst = std::min(worst, d);
        }
    std::printf("        [bisect] %zu cuts, closest junction pair %.1f m\n",
                segs.size(), worst);
    CHECK(worst >= p.hardCollapse);
    // No dead ends: every cut endpoint lies on the boundary or an earlier cut.
    RoadGraph g = fabricGraph(patch, segs);
    CHECK(degree1Interior(g, patch) == 0);
    // Determinism: two runs bit-identical.
    CHECK(sameSegments(segs, fabricBisect(patch, p)));
}

// ---- piedmont-recipe integration -----------------------------------------
// The multi-site metro with "fabric": "mix" must keep EVERY gate the legacy
// fill holds (test_metro_sites) while actually subdividing city faces.

namespace {

// dense=false: the piedmont_roads sandbox recipe (P2.5 arterial-scale
// growth — the recipe every test_metro_sites gate is proven on, and the one
// the capture battery renders). dense=true: the FULL-CITY recipe
// (assets/levels/piedmont.json, the level that ships buildings) — denser
// growth, many more enclosed faces for the fabric to subdivide.
MetroParams piedmontFabricParams(bool dense) {
    MetroParams p;
    p.seed = 7;
    p.freeways = true;
    p.backboneClass = RoadClass::Arterial;
    p.freewayWidth = 17;
    p.arteryWidth = 17;
    p.collectorWidth = 13;
    p.streetWidth = 11;
    p.blockSize = 220;
    p.minBlockEdge = 150;
    if (dense) {
        p.segLength = 50;
        p.influence = 560;
        p.killRadius = 110;
        p.mergeRadius = 80;
        p.corridorSpacing = 120;
        p.ambientPer500 = 14;
        p.loopMin = 200;
        p.loopMax = 650;
    } else {
        p.segLength = 120;
        p.influence = 800;
        p.killRadius = 300;
        p.mergeRadius = 200;
        p.corridorSpacing = 240;
        p.ambientPer500 = 5;
        p.loopMin = 550;
        p.loopMax = 1300;
    }
    p.interchangeSpacing = 600;

    p.fabric = "mix";
    p.fabricConform = 0.15;
    p.fabricJitter = 0.12;
    p.fabricSoftCollapse = 0.8;

    MetroSite city;
    city.center = {900, 900};
    city.radius = 1400;
    city.hotspots = 9;
    city.blockSize = 220;
    MetroSite east;
    east.center = {3050, 400};
    east.radius = 420;
    east.hotspots = 3;
    east.blockSize = 180;
    east.density = 0.55;
    east.kindBias = 3;
    MetroSite south;
    south.center = {300, 3050};
    south.radius = 380;
    south.hotspots = 3;
    south.blockSize = 180;
    south.density = 0.5;
    south.kindBias = 2;
    p.sites = {city, east, south};
    return p;
}

// Mirrors applyGenerateRecipe's big-block tail exactly (test_metro_sites).
RoadGraph runRecipeTail(RoadGraph g) {
    RoadRules rules;
    rules.autoRoundabout = false;
    g = capDegree(planarize(applyConstraints(g, rules), 1.0), rules);
    for (int round = 0; round < 2; ++round) {
        g = dropParallelEdges(g);
        g = dissolveAcuteArms(g, 0.85, 3);
        g = consolidateJunctionSpans(g, 150.0, rules.maxDegree);
        g = capDegree(planarize(g, 1.0), rules);
        g = mergeShortEdges(g, 30.0, rules.maxDegree);
        g = relaxSharpBends(g, 0.5, 64);
    }
    return g;
}

int reachableFrom0(const RoadGraph& g) {
    if (g.nodes.empty()) return 0;
    std::vector<std::vector<int>> adj(g.nodes.size());
    for (const RoadEdge& e : g.edges) {
        adj[e.a].push_back(e.b);
        adj[e.b].push_back(e.a);
    }
    std::vector<char> seen(g.nodes.size(), 0);
    std::queue<int> q;
    q.push(0);
    seen[0] = 1;
    int reach = 1;
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        for (int v : adj[u])
            if (!seen[v]) {
                seen[v] = 1;
                ++reach;
                q.push(v);
            }
    }
    return reach;
}

std::vector<double> junctionSpans(const RoadGraph& g) {
    std::vector<int> deg(g.nodes.size(), 0);
    for (const RoadEdge& e : g.edges) {
        ++deg[e.a];
        ++deg[e.b];
    }
    std::vector<std::vector<std::pair<int, int>>> nbr(g.nodes.size());
    for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
        nbr[g.edges[ei].a].push_back({g.edges[ei].b, ei});
        nbr[g.edges[ei].b].push_back({g.edges[ei].a, ei});
    }
    std::vector<char> walked(g.edges.size(), 0);
    std::vector<double> spans;
    for (std::size_t n = 0; n < g.nodes.size(); ++n) {
        if (deg[n] == 2) continue;
        for (auto [next, ei0] : nbr[n]) {
            if (walked[ei0]) continue;
            walked[ei0] = 1;
            double len = (g.nodes[n].pos - g.nodes[next].pos).length();
            int prev = static_cast<int>(n), cur = next;
            while (deg[cur] == 2) {
                auto [a0, ea] = nbr[cur][0];
                auto [a1, eb] = nbr[cur][1];
                int nn = (a0 == prev) ? a1 : a0;
                int ne = (a0 == prev) ? eb : ea;
                if (walked[ne]) break;
                walked[ne] = 1;
                len += (g.nodes[cur].pos - g.nodes[nn].pos).length();
                prev = cur;
                cur = nn;
            }
            spans.push_back(len);
        }
    }
    return spans;
}

int degree1Count(const RoadGraph& g) {
    std::vector<int> deg(g.nodes.size(), 0);
    for (const RoadEdge& e : g.edges) {
        if (e.a == e.b) continue;
        ++deg[e.a];
        ++deg[e.b];
    }
    int n = 0;
    for (int d : deg)
        if (d == 1) ++n;
    return n;
}

double worstFold(const RoadGraph& g, const char* tag) {
    std::vector<int> deg(g.nodes.size(), 0);
    std::vector<std::array<int, 2>> nbr(g.nodes.size(), {-1, -1});
    for (const RoadEdge& e : g.edges) {
        if (e.a == e.b) continue;
        if (deg[e.a] < 2) nbr[e.a][deg[e.a]] = e.b;
        if (deg[e.b] < 2) nbr[e.b][deg[e.b]] = e.a;
        ++deg[e.a];
        ++deg[e.b];
    }
    double worst = 0;
    Vec2 at{0, 0};
    for (std::size_t v = 0; v < g.nodes.size(); ++v) {
        if (deg[v] != 2) continue;
        Vec2 d0 = g.nodes[v].pos - g.nodes[nbr[v][0]].pos;
        Vec2 d1 = g.nodes[nbr[v][1]].pos - g.nodes[v].pos;
        double l0 = d0.length(), l1 = d1.length();
        if (l0 < 1e-6 || l1 < 1e-6) continue;
        double c = std::clamp(dot(d0, d1) / (l0 * l1), -1.0, 1.0);
        double a = std::acos(c);
        if (a > worst) { worst = a; at = g.nodes[v].pos; }
    }
    std::printf("        [%s foldback] worst %.1f deg at (%.0f,%.0f)\n", tag,
                worst * 180.0 / kPi, at.x, at.y);
    return worst;
}

int under120Spans(const RoadGraph& g) {
    int n = 0;
    for (double s : junctionSpans(g))
        if (s < 120.0) ++n;
    return n;
}

// Degree-2 nodes bent past the 35-degree foldback bar — the stage-1 count.
int foldCount(const RoadGraph& g) {
    std::vector<int> deg(g.nodes.size(), 0);
    std::vector<std::array<int, 2>> nbr(g.nodes.size(), {-1, -1});
    for (const RoadEdge& e : g.edges) {
        if (e.a == e.b) continue;
        if (deg[e.a] < 2) nbr[e.a][deg[e.a]] = e.b;
        if (deg[e.b] < 2) nbr[e.b][deg[e.b]] = e.a;
        ++deg[e.a];
        ++deg[e.b];
    }
    int n = 0;
    for (std::size_t v = 0; v < g.nodes.size(); ++v) {
        if (deg[v] != 2) continue;
        Vec2 d0 = g.nodes[v].pos - g.nodes[nbr[v][0]].pos;
        Vec2 d1 = g.nodes[nbr[v][1]].pos - g.nodes[v].pos;
        double l0 = d0.length(), l1 = d1.length();
        if (l0 < 1e-6 || l1 < 1e-6) continue;
        double c = std::clamp(dot(d0, d1) / (l0 * l1), -1.0, 1.0);
        if (std::acos(c) > 35.0 * kPi / 180.0) ++n;
    }
    return n;
}

}  // namespace

TEST_CASE(piedmont_roads_fabric_keeps_the_gated_recipe_clean) {
    // The SANDBOX recipe (piedmont_roads.json — the P2.5 arterial-scale
    // growth every test_metro_sites gate is proven on, and the level the
    // capture battery shows Glenn). With fabric on, every one of those
    // gates must hold ABSOLUTELY: the fabric survives the recipe tail.
    RoadGraph fab = runRecipeTail(buildMetro(piedmontFabricParams(false)));
    MetroParams lp = piedmontFabricParams(false);
    lp.fabric.clear();
    RoadGraph legacy = runRecipeTail(buildMetro(lp));

    std::vector<Poly2> blocks = extractBlocks(fab, 200.0);
    std::vector<Poly2> legacyBlocks = extractBlocks(legacy, 200.0);
    std::printf("        [roads fabric] %zu nodes %zu edges %zu blocks (legacy %zu)\n",
                fab.nodes.size(), fab.edges.size(), blocks.size(),
                legacyBlocks.size());
    // The metro_sites regression floor holds for the fabric build too. (The
    // raw count may sit BELOW the legacy fill's: mix keeps court/bisect
    // centers whole where gridFill would cut them — fewer, bigger blocks is
    // the intended big-block look, not lost enclosure.)
    CHECK(blocks.size() >= 18u);

    CHECK(reachableFrom0(fab) == static_cast<int>(fab.nodes.size()));

    std::vector<double> spans = junctionSpans(fab);
    int under120 = under120Spans(fab);
    std::printf("        [roads fabric span audit] %zu spans, %d under 120\n",
                spans.size(), under120);
    // Span-count floor tracks the fabric fill (fewer streets than gridFill
    // means fewer T's — 54 measured); the 120 m floor stays ABSOLUTE.
    CHECK(spans.size() > 45u);
    CHECK(under120 == 0);

    CHECK(worstFold(fab, "roads fabric") <= 35.0 * kPi / 180.0);

    // NO DEAD ENDS MINTED: every chord/cut endpoint lands on a boundary or
    // crossing, so fabric adds zero degree-1 nodes over the legacy fill
    // (the skeleton's own rim stubs are identical in both builds).
    int fabDead = degree1Count(fab), legacyDead = degree1Count(legacy);
    std::printf("        [roads dead-end audit] fabric %d, legacy %d\n",
                fabDead, legacyDead);
    CHECK(fabDead <= legacyDead);
}

TEST_CASE(piedmont_city_fabric_subdivides_the_dense_recipe) {
    // The FULL-CITY recipe (piedmont.json): enough enclosed faces that the
    // fabric's presence is measurable — block count clears 60. The dense
    // growth recipe carries two PRE-EXISTING town-site defects (2 spans in
    // 100..120 m and a ~179 deg hairpin near the east town at (3000,430) —
    // present with fabric OFF, outside the fabric's city-site domain, and
    // never gated: test_metro_sites only proves the sandbox recipe), so
    // those two gates are comparative: the fabric may not ADD offenders.
    RoadGraph fab = runRecipeTail(buildMetro(piedmontFabricParams(true)));
    MetroParams lp = piedmontFabricParams(true);
    lp.fabric.clear();
    RoadGraph legacy = runRecipeTail(buildMetro(lp));

    std::vector<Poly2> blocks = extractBlocks(fab, 200.0);
    std::vector<Poly2> legacyBlocks = extractBlocks(legacy, 200.0);
    std::printf("        [city fabric] %zu nodes %zu edges %zu blocks (legacy %zu)\n",
                fab.nodes.size(), fab.edges.size(), blocks.size(),
                legacyBlocks.size());
    CHECK(blocks.size() > 60u);

    CHECK(reachableFrom0(fab) == static_cast<int>(fab.nodes.size()));

    int under120 = under120Spans(fab), legacyUnder120 = under120Spans(legacy);
    std::printf("        [city span audit] %d under 120 (legacy %d)\n",
                under120, legacyUnder120);
    CHECK(under120 <= legacyUnder120);

    worstFold(fab, "city fabric");
    worstFold(legacy, "city legacy");
    // The dense recipe's lone hairpin lives in the east town either way;
    // the fabric may not add a single fold past the 35-degree bar.
    int fabFolds = foldCount(fab), legFolds = foldCount(legacy);
    std::printf("        [city fold count] fabric %d, legacy %d over 35 deg\n",
                fabFolds, legFolds);
    CHECK(fabFolds <= legFolds);

    int fabDead = degree1Count(fab), legacyDead = degree1Count(legacy);
    std::printf("        [city dead-end audit] fabric %d, legacy %d\n",
                fabDead, legacyDead);
    CHECK(fabDead <= legacyDead);
}

TEST_CASE(piedmont_fabric_mix_is_deterministic) {
    RoadGraph a = runRecipeTail(buildMetro(piedmontFabricParams(true)));
    RoadGraph b = runRecipeTail(buildMetro(piedmontFabricParams(true)));
    CHECK(a.nodes.size() == b.nodes.size());
    CHECK(a.edges.size() == b.edges.size());
    bool same = a.nodes.size() == b.nodes.size();
    for (std::size_t i = 0; same && i < a.nodes.size(); ++i)
        same = (a.nodes[i].pos - b.nodes[i].pos).length() < 1e-12;
    CHECK(same);
}
