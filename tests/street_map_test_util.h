#ifndef RAYTRACER_TESTS_STREET_MAP_TEST_UTIL_H
#define RAYTRACER_TESTS_STREET_MAP_TEST_UTIL_H

// A TEST WORLD'S ROAD MODEL, MADE AS A LEVEL'S IS (#306 piece 6 W2). A level publishes its road model
// (engine::LevelRoadModel::cities, ADR-0435), and the city host reads only that. A test world drawn as RoadEntity
// components is turned into a road file, built by the one road builder into a road model with its lane level
// (model_scene.h), and published the same way: its first city the built model. (It was a hand-made street map,
// buildNavGraph over the drawn graph, with no road model behind it; the street map itself went with #306.)

#include "model_scene.h"

#include "../src/engine/components.h"
#include "../src/engine/procgen/city/roads/road_entity.h"   // RoadEntity: the drawing
#include "../src/engine/world.h"

#include <memory>
#include <utility>

namespace testmap {

// (Re)publish the world's road model and street map from every RoadEntity in it, merged in the order they were added
// into one road file (one city). The scene is built once a run for each drawing (model_scene.h sceneNamed, by the road
// file's text) and checked to have built cleanly.
inline void publishStreetMap(engine::World& world) {
    engine::RoadGraph combined;
    world.each<engine::RoadEntity>([&](engine::Entity, engine::RoadEntity& net) {
        const engine::RoadGraph& g = net.graph;   // the drawing itself: each edge one road (not the sampled street-map graph)
        const int base = static_cast<int>(combined.nodes.size());
        for (const engine::RoadNode& n : g.nodes) combined.nodes.push_back(n);
        for (engine::RoadEdge e : g.edges) {
            e.a += base;
            e.b += base;
            combined.edges.push_back(e);
        }
    });
    engine::LevelRoadModel* roads = nullptr;
    world.each<engine::LevelRoadModel>([&](engine::Entity, engine::LevelRoadModel& m) { if (!roads) roads = &m; });
    if (!roads) roads = &world.add<engine::LevelRoadModel>(world.create(), engine::LevelRoadModel{});
    roads->cities.clear();
    if (combined.edges.empty()) {
        roads->laneLevelReport = "no roads";
        return;
    }
    const nlohmann::json file = citytest::roadFileOf(combined, "test_world");
    const citytest::ModelScene& s = citytest::sceneNamed("world_" + file.dump(), [&file]() { return citytest::modelScene(file); });
    roads->cities = {s.model};
    roads->laneLevelReport = "test world: " + std::to_string(s.model->laneLevel.walks.size()) + " walks";
}

// A road entity added to a test world, and the world's road model and street map published again with it.
inline void addRoads(engine::World& world, engine::RoadEntity net) {
    world.add<engine::RoadEntity>(world.create(), std::move(net));
    publishStreetMap(world);
}

}  // namespace testmap

#endif
