// road_map_svg — export a level's road network as a layered diagnostic SVG.
//
// Runs the REAL engine pipeline (applyGenerateRecipe from the level's own
// "generate" block, then growLotBuildingsOnNets), so the map is ground truth:
// what you see here is what the engine builds. No SVG library — SVG is XML;
// std::ofstream is the whole dependency.
//
// Layers (toggle by deleting a <g> in any editor / dev tools):
//   districts  — hub-tinted discs + district names at the growth hubs
//   blocks     — city-block interiors (outline)
//   lots       — parcels, filled by TYPE (home/shop/office/civic/park/green)
//   roads      — every chain at true scaled width; color by class
//   nodes      — junction vertices: arterial-tier vs fabric-tier
//   hotspots   — the colonization hubs the city grew from
//   names      — generated names for the longest arterial chains
//
// Build (standalone, like the other diagnostics — links the engine lib):
//   clang++ -std=c++17 -O2 -Isrc -Ithird_party \
//     tools/diagnostics/road_map_svg.cpp -Lbuild -lengine_core \
//     -framework Metal -framework Foundation -o /tmp/road_map_svg
// Run:
//   /tmp/road_map_svg assets/levels/piedmont_roads.json /tmp/piedmont_map.svg

#include "engine/procgen/city/road_net.h"
#include "engine/procgen/city/city_lots.h"
#include "engine/procgen/city/road_network.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace engine;
using nlohmann::json;

namespace {

const char* kDistrictNames[] = {"Financial", "Commercial", "Residential",
                                "Old Town", "Industrial"};
const char* kDistrictTint[] = {"#8fa8c8", "#c8b48f", "#9fc19a", "#c89f9a",
                               "#b0a6c4"};
// Seeded name tables for the longest arterial chains.
const char* kNameFirst[] = {"Meridian", "Cascade", "Juniper", "Granite",
                            "Alder",    "Summit",  "Laurel",  "Foothill",
                            "Prospect", "Sierra",  "Cedar",   "Vista"};

struct Chain {
    std::vector<Vec2> pts;
    double width = 0;
    RoadClass klass = RoadClass::Local;
    double len = 0;
};

// Walk junction-to-junction chains of the spline-sampled graph.
std::vector<Chain> chainsOf(const RoadGraph& g) {
    std::vector<int> deg(g.nodes.size(), 0);
    for (const RoadEdge& e : g.edges) { ++deg[e.a]; ++deg[e.b]; }
    std::vector<std::vector<std::pair<int, int>>> nbr(g.nodes.size());
    for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
        nbr[g.edges[ei].a].push_back({g.edges[ei].b, ei});
        nbr[g.edges[ei].b].push_back({g.edges[ei].a, ei});
    }
    std::vector<char> walked(g.edges.size(), 0);
    std::vector<Chain> out;
    for (std::size_t n = 0; n < g.nodes.size(); ++n) {
        if (deg[n] == 2) continue;
        for (auto [next, ei0] : nbr[n]) {
            if (walked[ei0]) continue;
            Chain c;
            c.width = g.edges[ei0].width;
            c.klass = g.edges[ei0].klass;
            c.pts.push_back(g.nodes[n].pos);
            walked[ei0] = 1;
            int prev = static_cast<int>(n), cur = next;
            c.pts.push_back(g.nodes[cur].pos);
            while (deg[cur] == 2) {
                auto [a0, ea] = nbr[cur][0];
                auto [a1, eb] = nbr[cur][1];
                int nn = (a0 == prev) ? a1 : a0;
                int ne = (a0 == prev) ? eb : ea;
                if (walked[ne]) break;
                walked[ne] = 1;
                prev = cur;
                cur = nn;
                c.pts.push_back(g.nodes[cur].pos);
            }
            for (std::size_t i = 0; i + 1 < c.pts.size(); ++i)
                c.len += (c.pts[i + 1] - c.pts[i]).length();
            out.push_back(std::move(c));
        }
    }
    return out;
}

std::string pathOf(const std::vector<Vec2>& pts) {
    std::ostringstream s;
    s.setf(std::ios::fixed);
    s.precision(1);
    for (std::size_t i = 0; i < pts.size(); ++i)
        s << (i == 0 ? "M" : " L") << pts[i].x << ' ' << pts[i].y;
    return s.str();
}

std::string polyOf(const Poly2& p) {
    std::ostringstream s;
    s.setf(std::ios::fixed);
    s.precision(1);
    for (std::size_t i = 0; i < p.size(); ++i)
        s << (i ? " " : "") << p[i].x << ',' << p[i].y;
    return s.str();
}

const char* lotFill(const std::string& t) {
    if (t == "home") return "#b7d4a8";
    if (t == "shop") return "#e3c08a";
    if (t == "office") return "#9db8dc";
    if (t == "civic") return "#c5a8dc";
    if (t == "park" || t == "green") return "#8fc47f";
    return "#cfcabe";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: road_map_svg <level.json> <out.svg>\n");
        return 1;
    }
    std::ifstream in(argv[1]);
    if (!in) { std::fprintf(stderr, "cannot open %s\n", argv[1]); return 1; }
    json level = json::parse(in, nullptr, false);
    if (level.is_discarded()) { std::fprintf(stderr, "bad json\n"); return 1; }

    // The road entity's generate block -> the REAL recipe pipeline.
    json gen;
    double roadWidth = 8.0, sidewalk = 3.5;
    for (const auto& ent : level.value("entities", json::array())) {
        if (ent.value("shape", "") != "road") continue;
        const json& road = ent["road"];
        gen = road.value("generate", json());
        roadWidth = road.value("width", roadWidth);
        sidewalk = road.value("sidewalk", sidewalk);
        break;
    }
    if (gen.is_null()) { std::fprintf(stderr, "no generated road entity\n"); return 1; }

    RoadNet net;
    net.width = roadWidth;
    net.autoRoundabout = false;
    applyGenerateRecipe(net, gen);
    std::printf("[map] net: %zu nodes, %zu edges, %zu hubs\n", net.nodes.size(),
                net.edges.size(), net.cityHubs.size());

    // Spline-sampled graph: curves rendered as the engine drives them.
    RoadGraph g = navRoadGraph(net);
    std::vector<Chain> chains = chainsOf(g);

    // Spawn hint: the junction nearest the primary hub (player + parked car
    // authoring after a regen — the downtown crossing moves with the recipe).
    if (!net.cityHubs.empty()) {
        std::vector<int> deg(g.nodes.size(), 0);
        for (const RoadEdge& e : g.edges) {
            ++deg[e.a];
            ++deg[e.b];
        }
        int best = -1;
        double bd = 1e30;
        for (std::size_t n = 0; n < g.nodes.size(); ++n) {
            if (deg[n] < 3) continue;
            const double d =
                (g.nodes[n].pos - net.cityHubs[0].pos).length();
            if (d < bd) {
                bd = d;
                best = static_cast<int>(n);
            }
        }
        if (best >= 0)
            std::printf("[map] spawn hint: downtown junction (%.0f, %.0f), "
                        "hub0 (%.0f, %.0f)\n",
                        g.nodes[best].pos.x, g.nodes[best].pos.y,
                        net.cityHubs[0].pos.x, net.cityHubs[0].pos.y);
    }

    // Blocks + lots via the real grower (flat ground, level parcel defaults).
    LotParams lp;
    lp.seed = gen.value("seed", 7u);
    const json& cs = level.value("citysim", json::object());
    lp.plinth = cs.value("plinth", 0.15);
    NetLotResult lots = growLotBuildingsOnNets({net}, lp, EdgeBlockParams{}, 2.0);
    std::printf("[map] blocks %zu, lot outlines %zu, buildings %zu\n",
                lots.plan.blocks.size(), lots.plan.lots.size(), lots.lots.size());

    // Bounds.
    Vec2 lo(1e30, 1e30), hi(-1e30, -1e30);
    for (const RoadNode& n : g.nodes) {
        lo.x = std::min(lo.x, n.pos.x); lo.y = std::min(lo.y, n.pos.y);
        hi.x = std::max(hi.x, n.pos.x); hi.y = std::max(hi.y, n.pos.y);
    }
    const double pad = 120;
    std::ostringstream svg;
    svg.setf(std::ios::fixed);
    svg.precision(1);
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\""
        << lo.x - pad << ' ' << lo.y - pad << ' ' << (hi.x - lo.x) + 2 * pad
        << ' ' << (hi.y - lo.y) + 2 * pad << "\" style=\"background:#f6f5f0\">\n";

    // districts — tinted discs + names at hubs.
    svg << "<g id=\"districts\" opacity=\"0.35\">\n";
    for (const CityHub& h : net.cityHubs) {
        int k = std::clamp(h.kind, 0, 4);
        double r = h.site == 0 ? 340.0 : 260.0;
        svg << "<circle cx=\"" << h.pos.x << "\" cy=\"" << h.pos.y << "\" r=\""
            << r << "\" fill=\"" << kDistrictTint[k] << "\"/>\n";
    }
    svg << "</g>\n<g id=\"district-names\" font-family=\"sans-serif\""
           " font-size=\"64\" fill=\"#5b616b\" text-anchor=\"middle\">\n";
    for (const CityHub& h : net.cityHubs) {
        int k = std::clamp(h.kind, 0, 4);
        svg << "<text x=\"" << h.pos.x << "\" y=\"" << h.pos.y - 20 << "\">"
            << kDistrictNames[k] << "</text>\n";
    }
    svg << "</g>\n";

    // blocks.
    svg << "<g id=\"blocks\" fill=\"none\" stroke=\"#b9b29e\" stroke-width=\"3\">\n";
    for (const Poly2& b : lots.plan.blocks)
        svg << "<polygon points=\"" << polyOf(b) << "\"/>\n";
    svg << "</g>\n";

    // lots — filled by building type where a building grew; bare parcels faint.
    svg << "<g id=\"lots\" stroke=\"#8e887a\" stroke-width=\"1\" opacity=\"0.85\">\n";
    for (const Poly2& l : lots.plan.lots)
        svg << "<polygon points=\"" << polyOf(l)
            << "\" fill=\"#e8e4d8\" opacity=\"0.5\"/>\n";
    for (const LotBuilding& b : lots.lots) {
        // Oriented footprint rectangle.
        Vec2 ax(std::cos(b.yaw), std::sin(b.yaw));
        Vec2 ay(-ax.y, ax.x);
        Vec2 hw = ax * (b.width * 0.5), hd = ay * (b.depth * 0.5);
        Poly2 r{b.site - hw - hd, b.site + hw - hd, b.site + hw + hd,
                b.site - hw + hd};
        svg << "<polygon points=\"" << polyOf(r) << "\" fill=\""
            << lotFill(b.type) << "\"/>\n";
    }
    svg << "</g>\n";

    // roads — true widths, class colors; fabric (Local) lighter.
    svg << "<g id=\"roads\" fill=\"none\" stroke-linecap=\"round\""
           " stroke-linejoin=\"round\">\n";
    for (const Chain& c : chains) {
        const char* col = c.klass == RoadClass::Arterial ? "#3d4654"
                        : c.klass == RoadClass::Collector ? "#5a6472"
                        : "#8b93a1";
        svg << "<path d=\"" << pathOf(c.pts) << "\" stroke=\"" << col
            << "\" stroke-width=\"" << c.width << "\"/>\n";
    }
    svg << "</g>\n";

    // nodes — arterial-tier junctions vs fabric junctions.
    std::vector<int> deg(g.nodes.size(), 0);
    std::vector<int> minK(g.nodes.size(), 99);
    for (const RoadEdge& e : g.edges) {
        ++deg[e.a]; ++deg[e.b];
        minK[e.a] = std::min(minK[e.a], static_cast<int>(e.klass));
        minK[e.b] = std::min(minK[e.b], static_cast<int>(e.klass));
    }
    svg << "<g id=\"nodes\">\n";
    for (std::size_t i = 0; i < g.nodes.size(); ++i) {
        if (deg[i] < 3) continue;
        const bool arterial = minK[i] <= static_cast<int>(RoadClass::Collector);
        svg << "<circle cx=\"" << g.nodes[i].pos.x << "\" cy=\""
            << g.nodes[i].pos.y << "\" r=\"" << (arterial ? 9 : 5)
            << "\" fill=\"" << (arterial ? "#b3452f" : "#2f6fb3") << "\"/>\n";
    }
    svg << "</g>\n";

    // hotspots.
    svg << "<g id=\"hotspots\">\n";
    for (const CityHub& h : net.cityHubs)
        svg << "<circle cx=\"" << h.pos.x << "\" cy=\"" << h.pos.y
            << "\" r=\"18\" fill=\"none\" stroke=\"#333a45\""
               " stroke-width=\"5\"/>\n";
    svg << "</g>\n";

    // names — the longest arterial chains, seeded from the table. The widest
    // two chains are Boulevards, arterials Avenues, collectors Streets.
    std::vector<const Chain*> arts;
    for (const Chain& c : chains)
        if (c.klass != RoadClass::Local && c.len > 400) arts.push_back(&c);
    std::sort(arts.begin(), arts.end(), [](const Chain* a, const Chain* b) {
        return a->len > b->len;
    });
    svg << "<g id=\"names\" font-family=\"sans-serif\" font-size=\"40\""
           " fill=\"#3d4654\" text-anchor=\"middle\">\n";
    for (std::size_t i = 0; i < arts.size() && i < 12; ++i) {
        const Chain& c = *arts[i];
        const Vec2 mid = c.pts[c.pts.size() / 2];
        const char* suffix = i < 2 ? "Blvd"
                            : c.klass == RoadClass::Arterial ? "Ave" : "St";
        svg << "<text x=\"" << mid.x << "\" y=\"" << mid.y - 14 << "\">"
            << kNameFirst[i % 12] << ' ' << suffix << "</text>\n";
    }
    svg << "</g>\n</svg>\n";

    std::ofstream out(argv[2]);
    out << svg.str();
    std::printf("[map] wrote %s (%zu chains, %zu blocks, %zu buildings)\n",
                argv[2], chains.size(), lots.plan.blocks.size(),
                lots.lots.size());
    return 0;
}
