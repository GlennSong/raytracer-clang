#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_MESH_H

#include "road_network.h"
#include "../../../renderer/renderer.h"   // RenderMesh
#include "../../../rt_math.h"             // Vec3
#include <functional>

namespace engine {

// Turn a road graph into a connected road *surface* (ADR-0044). The naive
// approach — one full-width ribbon per edge, run to the node centre — overlaps
// badly where roads meet (worst at a radial hub). This builds it properly:
//   * at each junction (degree >= 3) the incident roads are trimmed back to their
//     curb corners — the point where neighbouring roads' edges cross — and the
//     gap is filled with one triangulated junction pad, so roads meet *at* the
//     intersection instead of through it;
//   * the trimmed ribbons span the ground between junctions.
// Every patch of asphalt belongs to exactly one element, so nothing overlaps.
// Draped on `heightAt` (world XZ -> height; null = flat). Vertex-coloured.
struct RoadMeshParams {
    double lift = 0.25;                 // raise above the terrain to avoid z-fight
    double minSetback = 1.0;            // floor on the junction trim distance (m)
    Vec3   color{0.08, 0.08, 0.09};     // asphalt
    // Sidewalks: a raised skirt along the carriageway edges (and around the
    // junction corners) — a curb lip facing the street, a concrete slab top, and
    // an outer face dropping back to the ground. 0 width = no sidewalks.
    double sidewalkWidth = 0.0;         // slab width beyond the carriageway (m)
    double curbHeight = 0.15;           // how far the lip stands above the road (m)
    Vec3   sidewalkColor{0.62, 0.62, 0.60};   // concrete slab
    Vec3   curbColor{0.48, 0.48, 0.47};       // curb faces
    // Lane markings: thin raised stripes painted on the carriageway — solid edge
    // lines, a double-yellow centreline between opposing directions, and dashed
    // white lane dividers. The lane count is derived from the road width
    // (width / laneWidth), so wider arterials read as multi-lane for free.
    bool   laneMarkings = false;
    double laneWidth = 3.5;             // nominal lane width -> lane count (m)
    double markWidth = 0.16;            // painted stripe width (m)
    double markLift = 0.02;             // raise stripes above the asphalt (m)
    double dashLength = 3.0, dashGap = 3.0;   // lane-divider dash pattern (m)
    Vec3   laneColor{0.85, 0.85, 0.82};       // white lines
    Vec3   centerColor{0.80, 0.70, 0.12};     // yellow centreline
    std::function<double(double, double)> heightAt;
};

RenderMesh buildRoadMesh(const RoadGraph& graph, const RoadMeshParams& params);

}  // namespace engine

#endif
