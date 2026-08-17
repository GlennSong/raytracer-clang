// THE ORDER GATE for the road-graph unification (docs/road-graph-unification-plan.md).
//
// The refactor moves RoadNet's eight parallel arrays into std::vector<RoadEdge>,
// and the one way that can go quietly wrong is REORDERING. Generators seed their
// rng off node and edge indices, and every downstream pass — lot rng streams, the
// architect's per-lot seed, landmark scoring — rides on that ordering. Shuffle it
// and nothing fails: the city just silently becomes a different city, and you find
// out when a screenshot you took last week no longer reproduces.
//
// So this pins the generators' output ORDER before any call site moves, on both
// paths that ship: the district recipe and the multi-site metro. It hashes
// positions and endpoints in sequence, so a permutation changes the hash even when
// the SET of roads is identical.
//
// If this fails after a refactor step, the geometry did not break — the ORDER did.
#include "test_framework.h"

#include "../src/engine/procgen/city/district.h"
#include "../src/engine/procgen/city/metro.h"
#include "../src/engine/procgen/city/road_network.h"

#include <cstdint>
#include <cstdio>

using namespace engine;

namespace {

// Order-sensitive FNV-1a over the graph's sequence. Positions are quantised to
// 1e-3 m so the hash survives harmless float drift but not a reshuffle.
uint64_t hashGraphOrder(const RoadGraph& g) {
    uint64_t h = 1469598103934665603ull;
    auto mix = [&h](uint64_t v) {
        h ^= v;
        h *= 1099511628211ull;
    };
    auto mixCoord = [&mix](double v) {
        mix(static_cast<uint64_t>(static_cast<int64_t>(std::llround(v * 1000.0))));
    };
    mix(g.nodes.size());
    for (const RoadNode& n : g.nodes) {
        mixCoord(n.pos.x);
        mixCoord(n.pos.y);
    }
    mix(g.edges.size());
    for (const RoadEdge& e : g.edges) {
        mix(static_cast<uint64_t>(e.a));
        mix(static_cast<uint64_t>(e.b));
        mixCoord(e.width);
        mix(static_cast<uint64_t>(e.klass));
    }
    return h;
}

DistrictParams livingCityDistrict() {
    // living_city.json's recipe, so the gate tracks a level that actually ships.
    DistrictParams p;
    p.radius = 170;
    p.arterials = 3;
    p.blockSizeMax = 82;
    p.blockSizeMin = 82 * 0.55;
    p.irregular = 0.22;
    p.jitter = 0.16;
    p.arteryWidth = 13;
    p.streetWidth = 7;
    p.seed = 5;
    return p;
}

}  // namespace

TEST_CASE(district_graph_order_is_stable) {
    const RoadGraph a = buildDistrict(livingCityDistrict()).graph;
    const RoadGraph b = buildDistrict(livingCityDistrict()).graph;
    CHECK(!a.nodes.empty());
    CHECK(!a.edges.empty());
    // Same seed, same run: identical sequence, not merely an identical set.
    CHECK(hashGraphOrder(a) == hashGraphOrder(b));
    std::printf("        [order] district %zu nodes %zu edges hash %llu\n",
                a.nodes.size(), a.edges.size(),
                static_cast<unsigned long long>(hashGraphOrder(a)));
}

TEST_CASE(metro_graph_order_is_stable) {
    MetroParams p;
    p.seed = 7;
    p.freeways = true;
    MetroSite city;
    city.center = {900, 900};
    city.radius = 1400;
    city.hotspots = 9;
    city.blockSize = 220;
    city.density = 1.0;
    p.sites = {city};

    const RoadGraph a = buildMetro(p);
    const RoadGraph b = buildMetro(p);
    CHECK(!a.nodes.empty());
    CHECK(!a.edges.empty());
    CHECK(hashGraphOrder(a) == hashGraphOrder(b));
    std::printf("        [order] metro %zu nodes %zu edges hash %llu\n",
                a.nodes.size(), a.edges.size(),
                static_cast<unsigned long long>(hashGraphOrder(a)));
}

// A permutation of the SAME roads must be caught. Without this, the gate above
// could be passing for the trivial reason that the hash ignores order.
TEST_CASE(graph_order_hash_detects_a_permutation) {
    RoadGraph g = buildDistrict(livingCityDistrict()).graph;
    CHECK(g.edges.size() > 4u);
    RoadGraph swapped = g;
    std::swap(swapped.edges[0], swapped.edges[1]);
    CHECK(hashGraphOrder(g) != hashGraphOrder(swapped));
}
