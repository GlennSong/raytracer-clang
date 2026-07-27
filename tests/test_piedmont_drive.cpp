// The 8km-city plan's acceptance road: "drivable from the city to the towns."
// Routes the multi-site piedmont graph with the SAME nav machinery the citysim
// drives (buildNavGraph + A*), then interrogates the actual swept road mesh
// under those routes with the drive probe — the gate that catches a network
// whose graph connects but whose geometry doesn't carry a car.
#include "test_framework.h"

#include "../src/engine/ai/nav_graph.h"
#include "../src/engine/ai/pathfind.h"
#include "../src/engine/procgen/city/metro.h"
#include "../src/engine/procgen/city/road_constraints.h"
#include "../src/engine/procgen/city/road_net.h"
#include "drive_probe.h"

#include <cstdio>
#include <cmath>
#include <vector>

using namespace engine;

namespace {

// Mirrors tests/test_metro_sites.cpp's piedmont recipe (kept in sync by hand;
// the metro-sites file owns the growth-quality gates, this one owns drivability).
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
    p.segLength = 120;
    p.influence = 800;
    p.killRadius = 300;
    p.mergeRadius = 200;
    p.corridorSpacing = 240;
    p.ambientPer500 = 5;
    p.loopMin = 550;
    p.loopMax = 1300;
    p.interchangeSpacing = 600;
    // P8: the shipping recipe is footprint-first (growth dials above stay as
    // the legacy fallback mirror; footprint mode ignores them).
    p.skeleton = "footprint";
    p.districtLen = 1200;
    p.gateSpacing = 1100;

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

RoadGraph buildPiedmontGraph() {
    MetroParams p = piedmontParams();
    RoadGraph g = buildMetro(p);
    RoadRules rules;
    rules.autoRoundabout = false;
        auto spanFloor = [](const Vec2& q) -> Real {
            const bool inCore =
                std::fabs(q.x - 900.0) <= 1400.0 && std::fabs(q.y - 900.0) <= 1400.0;
            return inCore ? 104.5 : 150.0;
        };
    g = capDegree(planarize(applyConstraints(g, rules), 1.0), rules);
    for (int round = 0; round < 2; ++round) {
        g = dropParallelEdges(g);
        g = dissolveAcuteArms(g, 0.85, 3);
        g = consolidateJunctionSpans(g, spanFloor, rules.maxDegree);
        g = capDegree(planarize(g, 1.0), rules);
        g = mergeShortEdges(g, 30.0, rules.maxDegree);
        g = relaxSharpBends(g, 0.5, 64);
    }
    return g;
}

RoadNet netFromGraph(const RoadGraph& g) {
    RoadNet net;
    net.width = 11.0;
    net.autoRoundabout = false;
    net.nodes.reserve(g.nodes.size());
    for (const RoadNode& n : g.nodes) net.nodes.push_back(n.pos);
    net.edges.reserve(g.edges.size());
    for (const RoadEdge& e : g.edges) {
        net.edges.push_back({e.a, e.b});
        net.edgeWidths.push_back(e.width);
        net.edgeLayers.push_back(e.layer);
        net.edgeClasses.push_back(e.klass);
        net.edgeSpecs.push_back(-1);
        net.edgeBaked.push_back(0);
    }
    return net;
}

}  // namespace

TEST_CASE(city_to_towns_are_drivable_on_the_real_mesh) {
    RoadGraph g = buildPiedmontGraph();
    RoadNet net = netFromGraph(g);
    const RenderMesh mesh = buildRoadNetMesh(net);
    CHECK(mesh.vertices.size() > 10000u);

    // Nav from the SPLINE-SAMPLED derived graph — the same one the loader
    // hands the citysim (navRoadGraph). Nav links interpolate straight lines;
    // on the raw graph a 120m chord across a curved chain leaves the swept
    // ribbon entirely (measured: 25 "holes" that were really corner-cutting).
    NavGraph nav = buildNavGraph(navRoadGraph(net));

    const Vec2 city(900, 900);
    const Vec2 towns[2] = {Vec2(3050, 400), Vec2(300, 3050)};
    const char* names[2] = {"east", "south"};

    for (int t = 0; t < 2; ++t) {
        Route route = findRouteBetween(nav, city, towns[t]);
        CHECK(!route.links.empty());
        if (route.links.empty()) continue;

        // Centreline polyline sampled ~6m — the ghost guide a sim car chases.
        std::vector<Vec3> path;
        for (int link : route.links) {
            const NavLink& l = nav.links[link];
            const int steps = std::max(1, static_cast<int>(l.length / 6.0));
            for (int s = (path.empty() ? 0 : 1); s <= steps; ++s) {
                Vec2 q = nav.pointOnLink(link, static_cast<Real>(s) / steps);
                path.push_back(Vec3(q.x, 0.0, q.y));
            }
        }
        CHECK(path.size() > 100u);   // a real inter-site trip, not a hop

        driveprobe::Report rep;
        driveprobe::drivePath(mesh, path, rep);
        std::printf("        [drive audit] city->%s: %.1f km, holes %d, steps %d, grades %d, blocked %d\n",
                    names[t], route.length(nav) / 1000.0, rep.holes, rep.steps,
                    rep.grades, rep.blocked);
        int shown = 0;
        for (const auto& d : rep.defects) {
            if (d.kind != "hole" || shown >= 3) continue;
            std::printf("            hole at (%.0f, %.0f)\n", d.where.x, d.where.z);
            ++shown;
        }
        // Localize the south-exit defect: what are the first links made of?
        for (int li = 0; li < 3 && li < static_cast<int>(route.links.size()); ++li) {
            const NavLink& l = nav.links[route.links[li]];
            Vec2 a = nav.pointOnLink(route.links[li], 0.0);
            Vec2 b = nav.pointOnLink(route.links[li], 1.0);
            std::printf("            link[%d] (%.0f,%.0f)->(%.0f,%.0f) len %.0f "
                        "klass %d lanes %d width %.1f layer %d walk %d\n",
                        li, a.x, a.y, b.x, b.y, l.length,
                        static_cast<int>(l.klass), l.lanes, l.width, l.layer,
                        l.walkable ? 1 : 0);
        }
        // The bar: continuous surface (no holes), nothing standing across
        // the carriageway (no blocked). The hub-exit wedge holes are FIXED —
        // dissolveAcuteArms deletes the un-meshable sliver twin (road
        // provenance: two width-17 arterials 26 deg apart at the city hub).
        CHECK(rep.holes == 0);
        CHECK(rep.blocked == 0);
        CHECK(rep.steps <= 2);
        CHECK(rep.grades <= 2);
    }
}
