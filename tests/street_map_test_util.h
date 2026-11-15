#ifndef RAYTRACER_TESTS_STREET_MAP_TEST_UTIL_H
#define RAYTRACER_TESTS_STREET_MAP_TEST_UTIL_H

// A TEST WORLD'S STREET MAP. A level's street map is built from its road model when it loads (engine::LevelRoadModel::streetMap,
// ADR-0425), and the city sim reads only that. A test world made of bare RoadEntity components has no road model, so
// this builds its map from those entities' hand-made graphs the way any hand-made graph becomes a NavGraph (navRoadGraph,
// buildNavGraph: what the city sim did privately for such worlds before) and publishes it as the world's map.

#include "../src/engine/ai/nav_graph.h"
#include "../src/engine/components.h"
#include "../src/engine/procgen/city/roads/road_entity.h"   // navRoadGraph
#include "../src/engine/world.h"

#include <memory>
#include <utility>

namespace testmap {

// (Re)publish the world's street map from every RoadEntity in it, merged in the order they were added. `ground`: the
// ground the graphs' grades are read against (none: flat).
inline void publishStreetMap(engine::World& world, const engine::RoadGroundFn& ground = nullptr) {
    engine::RoadGraph combined;
    world.each<engine::RoadEntity>([&](engine::Entity, engine::RoadEntity& net) {
        const engine::RoadGraph g = engine::navRoadGraph(net, ground);
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
    roads->streetMap = combined.edges.empty() ? std::make_shared<engine::NavGraph>() : std::make_shared<engine::NavGraph>(engine::buildNavGraph(combined));
}

// A road entity added to a test world, and the world's street map published again with it.
inline void addRoads(engine::World& world, engine::RoadEntity net, const engine::RoadGroundFn& ground = nullptr) {
    world.add<engine::RoadEntity>(world.create(), std::move(net));
    publishStreetMap(world, ground);
}

}  // namespace testmap

#endif
