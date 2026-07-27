// P8-C footprint-first arterial skeleton gates: the constructed rim + spine
// + bisection network must deliver what space colonization couldn't —
// arterial spans with real breadth (median >= 600 m, none under 400 inside
// the city polygon), no foldbacks, no interior dead ends, a capped center
// junction, town connectivity, derived hubs on the metro kind contract, and
// bit-identical determinism. Numbers are printed so re-tuning is measured.
#include "test_framework.h"

#include "../src/engine/procgen/city/metro.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <queue>
#include <vector>

using namespace engine;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Two-site footprint fixture on open ground (flat sampler -> footprints are
// full discs): city r1600 + one town r420 to the east.
MetroParams fixtureParams() {
    MetroParams p;
    p.seed = 5;
    p.skeleton = "footprint";
    p.districtLen = 1400.0;
    p.gateSpacing = 1100.0;
    p.arteryWidth = 17.0;
    p.streetWidth = 10.0;
    p.arterialsOnly = true;   // skeleton under test, not the fill
    p.minBlockEdge = 150.0;
    MetroSite city;
    city.center = Vec2(0, 0);
    city.radius = 1600.0;
    city.hotspots = 7;
    p.sites.push_back(city);
    MetroSite town;
    town.center = Vec2(3400, 300);
    town.radius = 420.0;
    town.hotspots = 3;
    town.kindBias = 3;   // oldtown
    p.sites.push_back(town);
    return p;
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

std::vector<double> junctionSpans(const RoadGraph& g,
                                  std::vector<Vec2>* midsOut = nullptr) {
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
    if (midsOut) midsOut->clear();
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
            if (midsOut)
                midsOut->push_back((g.nodes[n].pos + g.nodes[cur].pos) * 0.5);
        }
    }
    return spans;
}

// Worst adjacent-segment deflection along degree-2 chains (the foldback
// audit — same walk as test_metro_sites).
double worstChainDeflection(const RoadGraph& g, Vec2* whereOut = nullptr) {
    std::vector<int> deg(g.nodes.size(), 0);
    std::vector<std::vector<int>> adj(g.nodes.size());
    for (const RoadEdge& e : g.edges) {
        ++deg[e.a];
        ++deg[e.b];
        adj[e.a].push_back(e.b);
        adj[e.b].push_back(e.a);
    }
    double worst = 0;
    for (std::size_t n = 0; n < g.nodes.size(); ++n) {
        if (deg[n] != 2) continue;
        const Vec2 a = g.nodes[adj[n][0]].pos, m = g.nodes[n].pos,
                   b = g.nodes[adj[n][1]].pos;
        const Vec2 d0 = normalize(m - a), d1 = normalize(b - m);
        const double c = std::clamp(dot(d0, d1), -1.0, 1.0);
        const double ang = std::acos(c) * 180.0 / kPi;
        if (ang > worst) {
            worst = ang;
            if (whereOut) *whereOut = m;
        }
    }
    return worst;
}

}  // namespace

TEST_CASE(footprint_skeleton_is_one_connected_network) {
    MetroParams p = fixtureParams();
    RoadGraph g = buildMetro(p, nullptr);
    CHECK(static_cast<int>(g.nodes.size()) > 150);
    CHECK(reachableFrom0(g) == static_cast<int>(g.nodes.size()));
    // Town has road presence (rim of its own footprint at least).
    int townNodes = 0;
    for (const RoadNode& n : g.nodes)
        if (std::fabs(n.pos.x - 3400.0) < 500.0 &&
            std::fabs(n.pos.y - 300.0) < 500.0)
            ++townNodes;
    CHECK(townNodes > 10);
}

TEST_CASE(footprint_skeleton_arterial_spans_have_breadth) {
    MetroParams p = fixtureParams();
    std::vector<CityHub> hubs;
    p.outHubs = &hubs;
    std::vector<Footprint> fps;
    p.outFootprints = &fps;
    RoadGraph g = buildMetro(p, nullptr);
    CHECK(!fps.empty());
    const Poly2& cityPoly = fps[0].polygon;
    std::vector<Vec2> mids;
    std::vector<double> spans = junctionSpans(g, &mids);
    std::vector<double> inCity;
    for (std::size_t i = 0; i < spans.size(); ++i)
        if (pointInPolygon(cityPoly, mids[i])) inCity.push_back(spans[i]);
    CHECK(inCity.size() >= 8);
    std::sort(inCity.begin(), inCity.end());
    const double median = inCity.empty() ? 0 : inCity[inCity.size() / 2];
    int under400 = 0;
    for (double s : inCity)
        if (s < 400.0) ++under400;
    std::printf(
        "        [skeleton span audit] %zu city spans, median %.0f m, "
        "min %.0f m, %d under 400\n",
        inCity.size(), median, inCity.empty() ? 0.0 : inCity.front(),
        under400);
    CHECK(median >= 600.0);
    CHECK(under400 == 0);
}

TEST_CASE(footprint_skeleton_has_no_foldbacks_or_dead_ends) {
    MetroParams p = fixtureParams();
    std::vector<Footprint> fps;
    p.outFootprints = &fps;
    RoadGraph g = buildMetro(p, nullptr);
    Vec2 where;
    const double worst = worstChainDeflection(g, &where);
    std::printf(
        "        [skeleton foldback] worst chain deflection %.1f deg at "
        "(%.0f, %.0f)\n",
        worst, where.x, where.y);
    {   // offender autopsy: the node's neighbors and their distances
        for (std::size_t n = 0; n < g.nodes.size(); ++n) {
            if ((g.nodes[n].pos - where).length() > 0.5) continue;
            for (const RoadEdge& e : g.edges) {
                if (e.a != static_cast<int>(n) && e.b != static_cast<int>(n))
                    continue;
                const Vec2 o =
                    g.nodes[e.a == static_cast<int>(n) ? e.b : e.a].pos;
                std::printf("            nbr (%.0f, %.0f) d=%.1f w=%.0f\n",
                            o.x, o.y, (o - where).length(), e.width);
            }
        }
    }
    CHECK(worst <= 35.0);

    // No degree-1 node strictly INSIDE a footprint (rim/connector ends sit on
    // or outside the boundary; a cut may never dangle mid-cell).
    std::vector<int> deg(g.nodes.size(), 0);
    for (const RoadEdge& e : g.edges) {
        ++deg[e.a];
        ++deg[e.b];
    }
    auto boundaryDist = [](const Poly2& poly, const Vec2& p) {
        double best = 1e30;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const Vec2 a = poly[i], b = poly[(i + 1) % poly.size()];
            const Vec2 ab = b - a;
            const double len2 = ab.lengthSquared();
            const double t =
                len2 > 1e-12
                    ? std::clamp(dot(p - a, ab) / len2, 0.0, 1.0)
                    : 0.0;
            best = std::min(best, (p - (a + ab * t)).length());
        }
        return best;
    };
    int interiorDead = 0;
    for (std::size_t n = 0; n < g.nodes.size(); ++n) {
        if (deg[n] != 1) continue;
        for (const Footprint& f : fps)
            if (pointInPolygon(f.polygon, g.nodes[n].pos) &&
                boundaryDist(f.polygon, g.nodes[n].pos) > 40.0)
                ++interiorDead;
    }
    CHECK(interiorDead == 0);
}

TEST_CASE(footprint_skeleton_center_junction_is_capped) {
    MetroParams p = fixtureParams();
    std::vector<Footprint> fps;
    p.outFootprints = &fps;
    RoadGraph g = buildMetro(p, nullptr);
    CHECK(!fps.empty());
    std::vector<int> deg(g.nodes.size(), 0);
    for (const RoadEdge& e : g.edges) {
        ++deg[e.a];
        ++deg[e.b];
    }
    // The node nearest the city center must not be a star of every road.
    int best = -1;
    double bd = 1e30;
    for (std::size_t n = 0; n < g.nodes.size(); ++n) {
        const double d = (g.nodes[n].pos - fps[0].center).length();
        if (d < bd) {
            bd = d;
            best = static_cast<int>(n);
        }
    }
    CHECK(best >= 0 && deg[best] <= 4);
}

TEST_CASE(footprint_skeleton_derives_hub_contract) {
    MetroParams p = fixtureParams();
    std::vector<CityHub> hubs;
    p.outHubs = &hubs;
    RoadGraph g = buildMetro(p, nullptr);
    (void)g;
    CHECK(hubs.size() >= 4);
    int citySeen = 0, townSeen = 0;
    bool cityFinancial = false;
    for (const CityHub& h : hubs) {
        if (h.site == 0) {
            ++citySeen;
            if (h.kind == 0) cityFinancial = true;
        } else {
            ++townSeen;
            // A town never grows a financial or industrial quarter.
            CHECK(h.kind != 0 && h.kind != 4);
        }
    }
    std::printf("        [skeleton hubs] %d city + %d town\n", citySeen,
                townSeen);
    CHECK(cityFinancial);
    CHECK(citySeen >= 2);
    CHECK(townSeen >= 1);
}

TEST_CASE(footprint_skeleton_svg_debug_dump) {
    // Visual debugging artifact (not a gate): /tmp/skel_debug.svg
    MetroParams p = fixtureParams();
    std::vector<Footprint> fps;
    p.outFootprints = &fps;
    RoadGraph g = buildMetro(p, nullptr);
    FILE* f = std::fopen("/tmp/skel_debug.svg", "w");
    if (!f) return;
    std::fprintf(f,
                 "<svg viewBox=\"-2000 -2200 6200 4600\" "
                 "xmlns=\"http://www.w3.org/2000/svg\">\n"
                 "<rect x=\"-2000\" y=\"-2200\" width=\"6200\" height=\"4600\" "
                 "fill=\"#fafaf8\"/>\n");
    for (const Footprint& fp : fps) {
        std::fprintf(f, "<polygon points=\"");
        for (const Vec2& v : fp.polygon)
            std::fprintf(f, "%.0f,%.0f ", v.x, v.y);
        std::fprintf(f, "\" fill=\"#9fc19a22\" stroke=\"#c8b48f\" "
                        "stroke-width=\"8\"/>\n");
        for (const FootprintGate& gate : fp.gates)
            std::fprintf(f,
                         "<circle cx=\"%.0f\" cy=\"%.0f\" r=\"28\" "
                         "fill=\"%s\"/>\n",
                         gate.pos.x, gate.pos.y,
                         gate.toSite >= 0 ? "#8f5a2c" : "#c8b48f");
    }
    std::vector<int> deg(g.nodes.size(), 0);
    for (const RoadEdge& e : g.edges) {
        ++deg[e.a];
        ++deg[e.b];
    }
    for (const RoadEdge& e : g.edges)
        std::fprintf(f,
                     "<line x1=\"%.0f\" y1=\"%.0f\" x2=\"%.0f\" y2=\"%.0f\" "
                     "stroke=\"#3d4654\" stroke-width=\"14\"/>\n",
                     g.nodes[e.a].pos.x, g.nodes[e.a].pos.y,
                     g.nodes[e.b].pos.x, g.nodes[e.b].pos.y);
    for (std::size_t n = 0; n < g.nodes.size(); ++n)
        if (deg[n] >= 3)
            std::fprintf(f,
                         "<circle cx=\"%.0f\" cy=\"%.0f\" r=\"16\" "
                         "fill=\"#b3452f\"/>\n",
                         g.nodes[n].pos.x, g.nodes[n].pos.y);
    std::fprintf(f, "</svg>\n");
    std::fclose(f);
}

TEST_CASE(footprint_skeleton_is_deterministic) {
    MetroParams p = fixtureParams();
    RoadGraph a = buildMetro(p, nullptr);
    RoadGraph b = buildMetro(p, nullptr);
    CHECK(a.nodes.size() == b.nodes.size());
    CHECK(a.edges.size() == b.edges.size());
    bool same = a.nodes.size() == b.nodes.size();
    for (std::size_t i = 0; same && i < a.nodes.size(); ++i)
        same = std::fabs(a.nodes[i].pos.x - b.nodes[i].pos.x) < 1e-12 &&
               std::fabs(a.nodes[i].pos.y - b.nodes[i].pos.y) < 1e-12;
    CHECK(same);
}
