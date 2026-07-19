#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_STRUCTURE_SET_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_STRUCTURE_SET_H

#include "../../mesh_builder.h"       // RenderMesh, Vec3
#include <array>
#include <vector>

namespace engine {

// A grade-break structure: the explicit 3-D geometry the 2.5-D height field can't
// represent (ADR-0075). Where a road's cut/fill batter is too steep to daylight
// into natural ground within its reach, the terrain leaves a residual STEP; a
// retaining wall (cut side) or fill wall (downhill side) caps that step. Walls are
// their own meshes + collider footprints — NOT baked into the heightfield — which
// is exactly the multi-valued case (a vertical face) the single-valued ground
// oracle defers to StructureSet. Bridges + tunnels join this set later.
struct WallSegment {
    Vec3   topA, topB;             // top edge along the shoulder (at the batter-end height)
    double dropA = 0, dropB = 0;   // height down to natural ground at each end (> 0)
    bool   retaining = true;       // true = cut wall (holds the hill), false = fill wall
    // Outward unit (XZ, away from the road). When set on a retaining wall,
    // the bake adds a BACKFILL BENCH: a horizontal cap from the top edge
    // back over the conform's feather to where it meets natural ground —
    // without it the feathered dip behind the wall reads as an open trench
    // between wall and hill (hillcity smoke).
    Vec3   out{0, 0, 0};
    double bench = 0;              // cap depth (m); 0 = no cap
};

struct StructureSet {
    std::vector<WallSegment> walls;
    RenderMesh mesh;               // wall faces (front + back + cap), one material
    std::vector<std::array<Vec3, 2>> colliderEdges;  // top edges for a thin-box collider bake
    bool empty() const { return walls.empty(); }
};

// Parameters shared with the road conform, so a wall stands exactly where the
// terrain batter clamps (never double-counting the step it already cut).
struct StructureParams {
    double reach      = 12.0;   // batter daylight reach (must match roadNetConformRegions)
    double cutBatter  = 1.0;    // uphill cut slope rise:run (match the conform)
    double fillBatter = 0.6;    // downhill fill embankment rise:run (match the conform)
    double shoulder   = 1.5;    // extra past the sidewalk, as in roadNetConformRegions
    double minWall    = 0.6;    // ignore steps shorter than this (the batter handles them)
    Vec3   color{0.62, 0.60, 0.57};   // concrete grey
};

// Tessellate a set of wall segments into a two-sided vertical mesh (front + back +
// top cap), flat-shaded in `color`. Pure geometry — the wall-placement logic lives
// with the conform (road_net.cpp buildRoadWalls) so the two stay in lockstep.
RenderMesh bakeWallMesh(const std::vector<WallSegment>& walls, const Vec3& color);

}  // namespace engine

#endif
