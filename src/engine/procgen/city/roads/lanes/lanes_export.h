#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANELAB_EXPORT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANELAB_EXPORT_H

// lanelab outputs for looking at: a binary glTF of the material meshes (Blender, the viewer's
// importer), a plan SVG in the road_map_svg spirit, a JSON of stats + invariants, and the lane graph's plan.

#include "engine/procgen/city/roads/lanes/deck_mesh.h"
#include "engine/procgen/city/roads/lane_graph.h"
#include <string>

namespace engine {
namespace roads::lanes {

bool writeGlb(const std::vector<NamedMesh>& meshes, const std::string& path, std::string* error = nullptr);
bool writePlanSvg(const Result& r, const std::string& path);
bool writeStatsJson(const Result& r, const std::string& path);
// THE LANE GRAPH AS A PLAN PICTURE: every lane path's centre line with an arrowhead at its end and its id, coloured by
// section; a tick across the road where each section starts; junction outlines; the ends of arriving lanes (red
// circles) and the starts of leaving lanes (green squares). 1 unit = 1 m, drawn as the plan SVG is (plan y up).
bool writeLaneGraphSvg(const LaneGraph& g, const std::string& path);

}  // namespace roads::lanes
}  // namespace engine

#endif
