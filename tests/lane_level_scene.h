#ifndef RAYTRACER_TESTS_LANE_LEVEL_SCENE_H
#define RAYTRACER_TESTS_LANE_LEVEL_SCENE_H

// A HAND-MADE LAB SCENE WITH THE FACTS THE LANE LEVEL READS (ADR-0426), stated in memory as the one-off tools would write
// them into a road file: its layout (`lanes_tool layout --write`: junctions and streets as today's inference finds them),
// every junction's control (here `unmarked`, the rule sets' word for no control, where the scene states none) and its
// lane rules (`lanes_tool layout --lane-rules 20 --write`). The lab scenes under assets/lanelab that predate the layout
// are read so, never changing the files; a scene whose file states its junctions already is read as it is -- its arms'
// ring order and "arms_in_ring" mark included (ADR-0427: never put in order here).

#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/lanes/road_layout.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <memory>
#include <string>

namespace lane_level_test {

inline nlohmann::json withLaneLevelFacts(nlohmann::json g) {
    using namespace engine::roads::lanes;
    if (!g.contains("streets")) {
        g.erase("junctions");
        writeLayout(g, deriveLayout(RoadLabGraph::fromJson(g), LayoutSource::Inferred));
    }
    for (nlohmann::json& j : g["junctions"])
        if (!j.contains("control")) j["control"] = "unmarked";
    if (!g.contains("lane_rules")) g["lane_rules"] = {{"solid_before_stop_line", 20.0}};
    return g;
}

// A scene file under the source tree, read and given the facts, then built.
inline std::unique_ptr<engine::roads::lanes::Result> buildWithLaneLevelFacts(const std::string& path) {
    using namespace engine::roads::lanes;
    std::ifstream f(path);
    nlohmann::json g;
    f >> g;
    return build(RoadLabGraph::fromJson(withLaneLevelFacts(g), path.substr(0, path.find_last_of('/'))));
}

}  // namespace lane_level_test

#endif
