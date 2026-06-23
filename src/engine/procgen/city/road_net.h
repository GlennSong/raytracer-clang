#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_NET_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_NET_H

#include "road_mesh.h"          // RoadMeshParams, buildRoadMesh, RenderMesh
#include <nlohmann/json.hpp>
#include <array>
#include <functional>
#include <vector>

namespace engine {

// An editor-authored road network (ADR-0049): a small graph of control nodes
// joined by edges, plus the look (width, sidewalk, markings, ...). The editable
// counterpart to the procedural city.road_mesh — promoted to a first-class entity
// so the inspector can WIDEN it and the viewport can DRAG its nodes, regenerating
// the carriageway live through buildRoadMesh (a fast pure function, no SDF grid).
// `heightAt` drapes it on the level terrain; it is set on load, not serialized.
struct RoadNet {
    std::vector<Vec2> nodes;
    std::vector<std::array<int, 2>> edges;     // node-index pairs (0-based)
    // Spline control (ADR-0049): when `curved`, each edge is sampled as a Hermite
    // cubic through its endpoints' tangents instead of a straight chord, so the
    // road follows a smooth curve the editor can shape by dragging tangent handles.
    // `tangents` is one through-direction per node (parallel to `nodes`); a zero (or
    // missing) tangent is auto — Catmull-Rom on a through-road, straight into a
    // junction/dead-end. Editing a tangent overrides the auto for that knot.
    std::vector<Vec2> tangents;
    bool   curved = false;
    double width = 10.0;                        // carriageway width (m) — the widen control
    double sidewalk = 2.5;                      // raised sidewalk width per verge (m)
    double curb = 0.16;                         // curb height (m)
    double cornerRadius = 3.0;                  // rounded kerb-return radius (m)
    double lift = 0.3;                          // raise above the ground (m)
    bool   markings = true;
    bool   crosswalks = true;
    Vec3   color{0.09, 0.09, 0.10};
    std::function<double(double, double)> heightAt;   // terrain drape (flat if unset)
};

// Build the road surface for `net` (its graph fed to buildRoadMesh with the look).
RenderMesh buildRoadNetMesh(const RoadNet& net);

// --- editor edit ops (each leaves the net ready for buildRoadNetMesh) ----------
// Set the carriageway width (the inspector "Width" control — "widen a road").
void roadNetSetWidth(RoadNet& net, double width);
// Move control node `i` to `pos` (the viewport node drag). False if out of range.
bool roadNetMoveNode(RoadNet& net, int i, const Vec2& pos);
// Set node `i`'s tangent (the viewport tangent-handle drag). A zero tangent reverts
// the knot to auto. Enables `curved` so the spline shows. False if out of range.
bool roadNetSetTangent(RoadNet& net, int i, const Vec2& tangent);
// Node `i`'s effective tangent (the stored override, or the auto Catmull-Rom/chord
// the curve actually uses) — what the viewport seeds the tangent handle from.
Vec2 roadNetTangentAt(const RoadNet& net, int i);

// --- level I/O: the `road` block of a shape:"road" entity ----------------------
RoadNet roadNetFromJson(const nlohmann::json& j);
nlohmann::json roadNetToJson(const RoadNet& net);

}  // namespace engine

#endif
