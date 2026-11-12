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
// What the lane graph's plan marks beyond the graph itself: the junctions refused, and where each refused turn is; and,
// optionally, the one window of the plan to draw (a junction close up) instead of all of it.
struct LaneGraphSvgMarks {
    std::vector<uint32_t> refusedJunctions;
    std::vector<Vec2> refusedTurns;
    bool cropped = false;
    Vec2 cropLo, cropHi;   // the window, when cropped
};
// THE LANE GRAPH AS A PLAN PICTURE: every lane path's centre line with an arrowhead at its end and its id, coloured by
// section; a tick across the road where each section starts; junction outlines; the ends of arriving lanes (red
// circles) and the starts of leaving lanes (green squares); the connectors, thinner, coloured by their turn (a key in
// the corner); refused junctions in red and refused turns as red crosses; each crossing point's stretch drawn wide and
// pale over each of its paths, with a mark at the middle of the stretch on its first path, coloured by its kind (cross,
// join, split; a circle when filed under a junction, a square when it reaches across junctions or is out on a road);
// each junction's control (a coloured disc above its id, the control's name beneath), and a triangle at the end of each
// arriving lane that faces a regulation (a stop sign), coloured by its kind, with both in the key (issue #240).
// 1 unit = 1 m, drawn as the plan SVG is (plan y up). A comment at the top gives the plan point at the picture's origin,
// for cropping.
bool writeLaneGraphSvg(const LaneGraph& g, const std::string& path, const LaneGraphSvgMarks& marks = LaneGraphSvgMarks());

}  // namespace roads::lanes
}  // namespace engine

#endif
