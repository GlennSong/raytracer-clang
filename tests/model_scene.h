#ifndef RAYTRACER_TESTS_MODEL_SCENE_H
#define RAYTRACER_TESTS_MODEL_SCENE_H

// THE CITY SIM'S TEST SCENES, MADE AS A LEVEL MAKES ITS ROADS (#306 task 4, piece 6, W2; ADR-0480). A scene is a small
// road file, stated in memory: each road a polyline with its class; its layout, controls and lane rules filled in as the
// one-off tools would write them (lane_level_scene.h withLaneLevelFacts). It is built by the one road builder and the road
// model is assembled from it, lane level and walks included (assembleRoadModel), exactly as a level's roads are; the
// street map the sim still walks on until W3 is made from that model by the loader's own function (appendStreetMap,
// ADR-0425), never by hand. The sim is handed the model too (CitySim::setRoads), as the city host does on every lanes
// level.
//
// A scene is built once per test run and shared (the build takes a moment): `sceneNamed` keeps each by its name.
//
// The shapes are the ones the sim tests used as hand-made street maps (city_test_util.h): a cross, a T, a straight road,
// the jittered grid town. They are stated here as road files, so the street map's nodes, links and widths are whatever
// the model makes of them: a test finds a place by where it is (nearestNode, nearestLink), never by a node's number.

#include "test_framework.h"   // CHECK: every scene asked for is checked to have built cleanly (sceneNamed)
#include "city_test_util.h"
#include "lane_level_scene.h"

#include <cstdio>

#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/ai/nav_graph.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/roads/lanes/lanes.h"
#include "../src/engine/procgen/city/roads/lanes/lane_graph_build.h"     // LaneGraphOptions, LaneGraphBuild
#include "../src/engine/procgen/city/roads/lanes/lane_graph_connect.h"
#include "../src/engine/procgen/city/roads/road_model.h"
#include "../src/engine/procgen/city/roads/street_map.h"

#include <nlohmann/json.hpp>

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace citytest {

struct ModelScene {
    std::shared_ptr<engine::RoadModel> model;         // the built model: its streets, its lane level, its walks. Shared by
                                                      // every test on the scene: read it; to change it, copy it
    engine::NavGraph nav;                             // the street map made from it, as a level's loader makes it
    std::string problems;                             // what building it found (the lane level's error, its own check's
                                                      // faults, the street map's errors); empty: none

    // A sim on this scene: built over its street map with `walkers` people from `seed`, and handed the model.
    void build(citysim::CitySim& sim, int walkers, uint32_t seed) const {
        sim.build(nav, walkers, seed);
        sim.setRoads(model, "test scene");
    }
    // The middle of the sidewalk beside `p`, on p's side of the road the model says p fronts (RoadModel::frontageAt,
    // sidewalkBeside): where a test puts a place or sends a walker. `p` itself when the model finds none.
    engine::Vec2 sidewalkBeside(engine::Vec2 p) const {
        engine::Vec2 out = p;
        const engine::RoadFrontage f = model->frontageAt(p);
        if (f.valid()) model->sidewalkBeside(f, out);
        return out;
    }
};

// The road class word a road file states for a street-map class.
inline const char* roadClassWord(engine::RoadClass c) {
    switch (c) {
        case engine::RoadClass::Arterial: return "arterial";
        case engine::RoadClass::Collector: return "collector";
        default: return "local";
    }
}

// A ROAD FILE FROM A HAND-DRAWN ROAD GRAPH: each edge a straight road of its class, from node a to node b. Nothing else is
// stated: the layout (which ends meet at a junction, which carry on), the controls and the lane rules are filled in as the
// tools would (withLaneLevelFacts).
inline nlohmann::json roadFileOf(const engine::RoadGraph& g, const std::string& name) {
    nlohmann::json f;
    f["name"] = name;
    f["edges"] = nlohmann::json::array();
    for (std::size_t e = 0; e < g.edges.size(); ++e) {
        const engine::RoadEdge& E = g.edges[e];
        const engine::Vec2 a = g.nodes[static_cast<std::size_t>(E.a)].pos, b = g.nodes[static_cast<std::size_t>(E.b)].pos;
        f["edges"].push_back({{"id", "r" + std::to_string(e)},
                              {"class", roadClassWord(E.klass)},
                              {"path", {{"type", "polyline"}, {"points", {{a.x, a.y}, {b.x, b.y}}}}}});
    }
    return lane_level_test::withLaneLevelFacts(f);
}

// The vehicle the scenes' turns are checked for: one ordinary car (the lane-level tests' own).
inline engine::roads::lanes::LaneGraphOptions sceneVehicles() {
    engine::roads::lanes::LaneGraphOptions o;
    engine::roads::lanes::VehicleSize car;
    car.tag = "car";
    car.turningRadius = 5.0;
    car.width = 1.9;
    car.length = 4.5;
    o.vehicles = {car};
    return o;
}

// BUILDS a road file into a scene: the road build, the model with its lane level, the street map from the model.
inline ModelScene modelScene(const nlohmann::json& roadFile) {
    using namespace engine::roads::lanes;
    ModelScene s;
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(roadFile));
    LaneGraphBuild report;
    std::shared_ptr<engine::RoadModel> m = std::make_shared<engine::RoadModel>(engine::assembleRoadModel(*r, sceneVehicles(), &report));
    s.problems = report.error;
    for (const std::string& p : engine::validate(m->laneLevel)) s.problems += (s.problems.empty() ? "" : "; ") + p;   // its own check
    engine::StreetMapReport rep;
    if (!engine::appendStreetMap(s.nav, *m, 0, nullptr, &rep)) {
        for (const std::string& e : rep.errors) s.problems += (s.problems.empty() ? "" : "; ") + e;
    }
    s.model = std::move(m);
    return s;
}

// ONE SCENE PER NAME, built on first asking and kept for the run. EVERY asking checks it built cleanly (a scene the road
// build, the lane level's own check or the street map found faults in would make the test on it meaningless): a failed
// check in the asking test, with the scene's name and what was found.
template <typename Make>
const ModelScene& sceneNamed(const std::string& name, Make make) {
    static std::map<std::string, ModelScene> scenes;
    std::map<std::string, ModelScene>::iterator it = scenes.find(name);
    if (it == scenes.end()) it = scenes.emplace(name, make()).first;
    const ModelScene& s = it->second;
    if (!s.problems.empty()) std::printf("    scene %s: %s\n", name.c_str(), s.problems.c_str());
    CHECK(s.problems.empty() && s.model && s.nav.linkCount() > 0 && !s.model->laneLevel.walks.empty());
    return s;
}

// A test's own drawing, as a road file (each edge one straight road of its class), kept under its name.
inline const ModelScene& graphScene(const std::string& name, const engine::RoadGraph& g) {
    return sceneNamed("graph_" + name, [&g, name]() { return modelScene(roadFileOf(g, name)); });
}

// THE SHAPES (city_test_util.h's, as road files).
// A 4-way cross: four `arm`-metre local roads meeting at (0, 0).
inline const ModelScene& crossScene(engine::Real arm) {
    return sceneNamed("cross_" + std::to_string(arm), [arm]() {
        engine::RoadGraph g;
        g.nodes = {{engine::Vec2(0, 0)}, {engine::Vec2(0, arm)}, {engine::Vec2(0, -arm)}, {engine::Vec2(arm, 0)}, {engine::Vec2(-arm, 0)}};
        for (int k = 1; k <= 4; ++k) g.edges.push_back(engine::RoadEdge{0, k, 8, engine::RoadClass::Local, 0});
        return modelScene(roadFileOf(g, "cross"));
    });
}
// A T: three `arm`-metre local roads meeting at (0, 0), north, east and west.
inline const ModelScene& teeScene(engine::Real arm) {
    return sceneNamed("tee_" + std::to_string(arm), [arm]() {
        engine::RoadGraph g;
        g.nodes = {{engine::Vec2(0, 0)}, {engine::Vec2(0, arm)}, {engine::Vec2(arm, 0)}, {engine::Vec2(-arm, 0)}};
        for (int k = 1; k <= 3; ++k) g.edges.push_back(engine::RoadEdge{0, k, 8, engine::RoadClass::Local, 0});
        return modelScene(roadFileOf(g, "tee"));
    });
}
// AN OPEN ROAD: a `len`-metre local road east from (0, 0), ending at a T with a 60 m cross road at its east end (the
// hand-made map was one road with no junction at all; a road file with no junction gets no lane level, so the T is the
// least that gives it one: the rest of the road is open, with no junction along it).
inline const ModelScene& straightScene(engine::Real len) {
    return sceneNamed("straight_" + std::to_string(len), [len]() {
        engine::RoadGraph g;
        g.nodes = {{engine::Vec2(0, 0)}, {engine::Vec2(len, 0)}, {engine::Vec2(len, -30)}, {engine::Vec2(len, 30)}};
        g.edges = {engine::RoadEdge{0, 1, 8, engine::RoadClass::Local, 0}, engine::RoadEdge{1, 2, 8, engine::RoadClass::Local, 0},
                   engine::RoadEdge{1, 3, 8, engine::RoadClass::Local, 0}};
        return modelScene(roadFileOf(g, "straight"));
    });
}
// The jittered grid town (city_test_util.h gridCity, no dropout), as a road file: every grid segment one road.
inline const ModelScene& gridScene(engine::Real extent, engine::Real cellSize, uint32_t seed) {
    return sceneNamed("grid_" + std::to_string(extent) + "_" + std::to_string(cellSize) + "_" + std::to_string(seed),
                      [extent, cellSize, seed]() {
                          GridCityParams p;
                          p.extent = extent;
                          p.cellSize = cellSize;
                          p.seed = seed;
                          return modelScene(roadFileOf(gridCity(p), "grid"));
                      });
}

}  // namespace citytest

#endif
