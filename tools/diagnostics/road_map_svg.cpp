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
//   traffic    — WHERE THE CARS START: one dot per driver at its spawn pose,
//                plus a coarse density grid showing where they pile up. Runs
//                the real CitySim::build over the real nav graph, so this is
//                the actual fleet the level boots with, not an estimate.
//   parking    — curbside bays (the parked scenery cars)
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
#include "engine/ai/nav_graph.h"
#include "apps/citysim/city_sim.h"

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
        std::fprintf(stderr,
                     "usage: road_map_svg <level.json> <out.svg> [--traffic]\n"
                     "  --traffic  drop the lot/district layers and draw a clean\n"
                     "             TRAFFIC map: recessive roads under a car-density\n"
                     "             choropleth + one dot per driver.\n");
        return 1;
    }
    // The traffic view answers "where do the cars start?", so the city itself
    // becomes context: magnitude over space needs a quiet base or the ramp has
    // nothing to read against.
    const bool trafficOnly = argc > 3 && std::string(argv[3]) == "--traffic";
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

    const json& cs = level.value("citysim", json::object());

    // TRAFFIC. The real nav graph and the real sim, built exactly as the level
    // boots it — same seed, same counts — then read at t=0. `cars` may be
    // negative (density-derived at load); the sim clamps, so pass it through and
    // report whatever it actually created.
    NavGraph nav = buildNavGraph(g);
    citysim::CitySim sim;
    const int carCount = cs.value("cars", 0);
    const int pedCount = cs.value("pedestrians", 0);
    const uint32_t simSeed = cs.value("seed", 7u);
    std::vector<Vec2> carStarts;
    std::vector<Vec2> bays;
    if (carCount > 0 || pedCount > 0) {
        sim.build(nav, std::max(0, carCount), std::max(0, pedCount), simSeed);
        for (const citysim::Agent& a : sim.agents())
            if (a.mode == citysim::Agent::Mode::Driver && a.vehicle >= 0)
                carStarts.push_back(a.pos);
        for (const citysim::CitySim::ParkingBay& b : sim.parkingBays())
            bays.push_back(b.pos);
        std::printf("[map] traffic: %zu drivers at spawn, %zu parking bays "
                    "(nav %d links)\n",
                    carStarts.size(), bays.size(), nav.linkCount());
    }

    // Blocks + lots via the real grower (flat ground, level parcel defaults).
    LotParams lp;
    lp.seed = gen.value("seed", 7u);
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
    // The traffic legend sits below the map, so that view needs extra room at
    // the bottom or the ramp and its caption fall outside the viewBox.
    const double padBottom = pad + 260;
    std::ostringstream svg;
    svg.setf(std::ios::fixed);
    svg.precision(1);
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\""
        << lo.x - pad << ' ' << lo.y - pad << ' ' << (hi.x - lo.x) + 2 * pad
        << ' ' << (hi.y - lo.y) + pad + padBottom
        << "\" style=\"background:#f6f5f0\">\n";

    // districts — tinted discs + names at hubs.
    if (!trafficOnly) {
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

    }

    // blocks.
    if (!trafficOnly) {
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

    }

    // roads — true widths, class colors; fabric (Local) lighter.
    svg << "<g id=\"roads\" fill=\"none\" stroke-linecap=\"round\""
           " stroke-linejoin=\"round\">\n";
    for (const Chain& c : chains) {
        const char* col = trafficOnly
                              ? (c.klass == RoadClass::Local ? "#e2e0d8" : "#cbc8bd")
                          : c.klass == RoadClass::Arterial ? "#3d4654"
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

    // TRAFFIC DENSITY — cars per 150 m cell (about a downtown block, and the
    // scale the render/sim bubble cares about). Magnitude over space, so it is a
    // SEQUENTIAL encoding: one hue, light to dark, binned into named steps with
    // a legend. Not a rainbow, and not red — red reads as "alarm" on a map whose
    // job is "here is where the fleet is".
    if (!carStarts.empty()) {
        const double cell = 150.0;
        const int gw = std::max(1, int((hi.x - lo.x) / cell) + 1);
        const int gh = std::max(1, int((hi.y - lo.y) / cell) + 1);
        std::vector<int> bins(std::size_t(gw) * gh, 0);
        for (const Vec2& p : carStarts) {
            const int cx = std::clamp(int((p.x - lo.x) / cell), 0, gw - 1);
            const int cy = std::clamp(int((p.y - lo.y) / cell), 0, gh - 1);
            ++bins[std::size_t(cy) * gw + cx];
        }
        int peak = 0, occupied = 0;
        for (int b : bins) { peak = std::max(peak, b); if (b) ++occupied; }

        // Thresholds are cars per cell.
        // Blue steps 250..650, evenly spaced. Five, not six: the validator
        // (dataviz skill) fails a six-step cut of this ramp on adjacent
        // lightness — two of the bins were not tellable apart.
        const int kEdges[] = {1, 3, 7, 13, 20};
        const char* kRamp[] = {"#86b6ef", "#5598e7", "#2a78d6", "#1c5cab",
                               "#104281"};
        const int kSteps = 5;
        auto stepOf = [&](int n) {
            int s2 = 0;
            for (int i = 0; i < kSteps; ++i) if (n >= kEdges[i]) s2 = i;
            return s2;
        };

        svg << "<g id=\"traffic-density\">\n";
        for (int cy = 0; cy < gh; ++cy)
            for (int cx = 0; cx < gw; ++cx) {
                const int n = bins[std::size_t(cy) * gw + cx];
                if (n == 0) continue;
                svg << "<rect x=\"" << lo.x + cx * cell << "\" y=\""
                    << lo.y + cy * cell << "\" width=\"" << cell << "\" height=\""
                    << cell << "\" fill=\"" << kRamp[stepOf(n)]
                    << "\" opacity=\"" << (trafficOnly ? 0.92 : 0.45) << "\"/>\n";
            }
        svg << "</g>\n";

        // Legend — a sequential ramp is unreadable without one.
        const double lx = lo.x, ly = hi.y + 40, sw = 150, sh = 46;
        svg << "<g id=\"traffic-legend\" font-family=\"sans-serif\">\n";
        svg << "<text x=\"" << lx << "\" y=\"" << ly - 14
            << "\" font-size=\"38\" fill=\"#0b0b0b\">Cars at spawn per "
            << int(cell) << " m cell</text>\n";
        for (int i = 0; i < kSteps; ++i) {
            svg << "<rect x=\"" << lx + i * sw << "\" y=\"" << ly
                << "\" width=\"" << sw << "\" height=\"" << sh << "\" fill=\""
                << kRamp[i] << "\"/>\n";
            svg << "<text x=\"" << lx + i * sw + 6 << "\" y=\"" << ly + sh + 34
                << "\" font-size=\"32\" fill=\"#52514e\">" << kEdges[i]
                << (i + 1 < kSteps ? "" : "+") << "</text>\n";
        }
        svg << "<text x=\"" << lx << "\" y=\"" << ly + sh + 82
            << "\" font-size=\"32\" fill=\"#52514e\">" << carStarts.size()
            << " drivers - peak " << peak << " in one cell - " << occupied
            << " cells occupied</text>\n";
        svg << "</g>\n";
        std::printf("[map] density: peak %d cars in one %.0f m cell, %d cells occupied\n",
                    peak, cell, occupied);
    }

    // parking — the curbside bays the parked scenery cars occupy.
    if (!bays.empty() && !trafficOnly) {
        svg << "<g id=\"parking\" fill=\"#5b6b8a\" opacity=\"0.55\">\n";
        for (const Vec2& b : bays)
            svg << "<circle cx=\"" << b.x << "\" cy=\"" << b.y
                << "\" r=\"2.4\"/>\n";
        svg << "</g>\n";
    }

    // traffic — one dot per driver at its spawn pose, over the ramp.
    if (!carStarts.empty()) {
        svg << "<g id=\"traffic\" fill=\"#0b0b0b\" opacity=\""
            << (trafficOnly ? 0.85 : 1.0) << "\">\n";
        for (const Vec2& p : carStarts)
            svg << "<circle cx=\"" << p.x << "\" cy=\"" << p.y
                << "\" r=\"" << (trafficOnly ? 4.5 : 3.6) << "\"/>\n";
        svg << "</g>\n";
    }

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
    if (trafficOnly) arts.clear();
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
