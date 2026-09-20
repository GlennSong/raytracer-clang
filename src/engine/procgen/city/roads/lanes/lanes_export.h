#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANELAB_EXPORT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANELAB_EXPORT_H

// lanelab outputs for looking at: a binary glTF of the material meshes (Blender, the viewer's
// importer), a plan SVG in the road_map_svg spirit, and a JSON of stats + invariants.

#include "engine/procgen/city/roads/lanes/deck_mesh.h"
#include <string>

namespace engine {
namespace roads::lanes {

bool writeGlb(const std::vector<NamedMesh>& meshes, const std::string& path, std::string* error = nullptr);
bool writePlanSvg(const Result& r, const std::string& path);
bool writeStatsJson(const Result& r, const std::string& path);

}  // namespace roads::lanes
}  // namespace engine

#endif
