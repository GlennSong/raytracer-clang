// Multi-site metro (8km-city plan P2): the city + its satellite towns grow in
// ONE graph — the backbone MST spans every site, so town connectivity is
// structural. These gates encode the piedmont recipe's promises: one connected
// component, big blocks (junction spans, not raw edges — an arterial is a
// chain of curve segments), towns populated and reachable, determinism.
#include "test_framework.h"

#include "../src/engine/procgen/city/metro.h"
#include "../src/engine/procgen/city/road_constraints.h"

#include <algorithm>
#include <queue>
#include <vector>

using namespace engine;

namespace {

MetroParams piedmontParams() {
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
    p.segLength = 50;
    p.influence = 560;
    p.killRadius = 110;
    p.mergeRadius = 80;
    p.corridorSpacing = 120;
    p.ambientPer500 = 14;
    p.loopMin = 200;
    p.loopMax = 650;
    p.interchangeSpacing = 600;

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
    east.kindBias = 3;   // oldtown
    MetroSite south;
    south.center = {300, 3050};
    south.radius = 380;
    south.hotspots = 3;
    south.blockSize = 180;
    south.density = 0.5;
    south.kindBias = 2;   // residential
    p.sites = {city, east, south};
    return p;
}

// The recipe pipeline tail applyGenerateRecipe runs for a big-block metro,
// minus terrain/curviness (covered elsewhere).
RoadGraph buildPiedmontGraph(std::vector<CityHub>* hubs = nullptr) {
    MetroParams p = piedmontParams();
    p.outHubs = hubs;
    RoadGraph g = buildMetro(p);
    RoadRules rules;
    rules.autoRoundabout = false;
    g = capDegree(planarize(applyConstraints(g, rules), 1.0), rules);
    g = mergeShortEdges(g, 30.0, rules.maxDegree);
    g = consolidateJunctionSpans(g, 150.0, rules.maxDegree);
    g = capDegree(planarize(g, 1.0), rules);
    g = mergeShortEdges(g, 30.0, rules.maxDegree);
    g = consolidateJunctionSpans(g, 150.0, rules.maxDegree);
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

// Junction-to-junction span lengths: walk chains through degree-2 curve nodes.
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

int nodesInSite(const RoadGraph& g, Vec2 c, double r) {
    int n = 0;
    for (const RoadNode& node : g.nodes) {
        Vec2 d = node.pos - c;
        if (std::fabs(d.x) <= r && std::fabs(d.y) <= r) ++n;
    }
    return n;
}

}  // namespace

TEST_CASE(multi_site_metro_is_one_connected_network) {
    RoadGraph g = buildPiedmontGraph();
    CHECK(g.nodes.size() > 200);
    CHECK(reachableFrom0(g) == static_cast<int>(g.nodes.size()));
}

TEST_CASE(multi_site_metro_hubs_carry_site_and_bias) {
    std::vector<CityHub> hubs;
    buildPiedmontGraph(&hubs);
    CHECK(hubs.size() == 15u);   // 9 + 3 + 3
    // City core is financial, town centrals take the bias.
    CHECK(hubs[0].site == 0);
    CHECK(hubs[0].kind == 0);
    CHECK(hubs[9].site == 1);
    CHECK(hubs[9].kind == 3);    // oldtown
    CHECK(hubs[12].site == 2);
    CHECK(hubs[12].kind == 2);   // residential
    // Towns never grow financial or industrial quarters.
    for (std::size_t i = 9; i < hubs.size(); ++i) {
        CHECK(hubs[i].kind != 0);
        CHECK(hubs[i].kind != 4);
    }
}

TEST_CASE(multi_site_metro_towns_are_populated_and_reachable) {
    RoadGraph g = buildPiedmontGraph();
    // Connectivity is total (case above), so presence == reachability.
    CHECK(nodesInSite(g, {3050, 400}, 420) >= 10);
    CHECK(nodesInSite(g, {300, 3050}, 380) >= 10);
    // The countryside between sites carries only the connector capsules, not
    // sprawl: nodes outside every site square + capsule margin are rare.
    int stray = 0;
    for (const RoadNode& n : g.nodes) {
        auto inBox = [&](Vec2 c, double r) {
            Vec2 d = n.pos - c;
            return std::fabs(d.x) <= r && std::fabs(d.y) <= r;
        };
        if (!inBox({900, 900}, 1400) && !inBox({3050, 400}, 420) &&
            !inBox({300, 3050}, 380))
            ++stray;
    }
    // Connector chains between sites are legitimate; a bounded share.
    CHECK(stray < static_cast<int>(g.nodes.size()) / 4);
}

TEST_CASE(multi_site_metro_enforces_junction_spacing) {
    RoadGraph g = buildPiedmontGraph();
    std::vector<double> spans = junctionSpans(g);
    CHECK(spans.size() > 80u);
    int under120 = 0;
    for (double s : spans)
        if (s < 120.0) ++under120;
    CHECK(under120 == 0);
}

TEST_CASE(multi_site_metro_grows_big_blocks) {
    RoadGraph g = buildPiedmontGraph();
    std::vector<Poly2> blocks = extractBlocks(g, 200.0);
    CHECK(blocks.size() >= 60u);
    std::vector<double> areas;
    for (const Poly2& b : blocks) {
        double A = 0;
        for (std::size_t i = 0; i < b.size(); ++i) {
            const Vec2& u = b[i];
            const Vec2& v = b[(i + 1) % b.size()];
            A += u.x * v.y - v.x * u.y;
        }
        areas.push_back(std::fabs(A) * 0.5);
    }
    std::sort(areas.begin(), areas.end());
    CHECK(areas[areas.size() / 2] >= 25000.0);   // median >= 2.5 ha
}

TEST_CASE(multi_site_metro_is_deterministic) {
    RoadGraph a = buildPiedmontGraph();
    RoadGraph b = buildPiedmontGraph();
    CHECK(a.nodes.size() == b.nodes.size());
    CHECK(a.edges.size() == b.edges.size());
    bool same = a.nodes.size() == b.nodes.size();
    for (std::size_t i = 0; same && i < a.nodes.size(); ++i)
        same = (a.nodes[i].pos - b.nodes[i].pos).length() < 1e-12;
    CHECK(same);
}

TEST_CASE(single_site_legacy_params_still_work) {
    // Empty sites[] must reproduce the legacy single-site behavior shape:
    // hubs from the flat params, graph centered on p.center.
    MetroParams p;
    p.seed = 5;
    p.center = {0, 0};
    p.radius = 700;
    p.hotspots = 6;
    p.blockSize = 70;
    std::vector<CityHub> hubs;
    p.outHubs = &hubs;
    RoadGraph g = buildMetro(p);
    CHECK(hubs.size() == 6u);
    for (const CityHub& h : hubs) CHECK(h.site == 0);
    CHECK(g.nodes.size() > 100);
    for (const RoadNode& n : g.nodes) {
        CHECK(std::fabs(n.pos.x) <= 700.0 + 1e-6);
        CHECK(std::fabs(n.pos.y) <= 700.0 + 1e-6);
    }
}
