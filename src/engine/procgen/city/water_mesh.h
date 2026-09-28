#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_WATER_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_WATER_MESH_H

#include "polygon.h"          // Vec2
#include "buildability.h"     // HeightSampler
#include "../../mesh_builder.h"   // RenderMesh, MeshBuilder, Vec3

namespace engine {

// A flat water surface at `seaLevel` over the region [lo,hi], emitted only where
// the terrain floor sits below the sea (land cells are skipped; the terrain mesh
// occludes the plane at the shoreline). Each vertex bakes two values into its UV
// for the Water surface shader — with NO dependence on a depth buffer:
//   u = water DEPTH   (seaLevel - floorHeight), the depth-colour gradient
//   v = SHORE distance (metres to the nearest land, capped at foamBand), the foam
// The mesh is drawn with a Surface::Water material (low roughness + <1 opacity),
// so waves + foam animate on windTime and it reflects via SSR / fresnel.
struct WaterMeshParams {
    double seaLevel = 0.0;
    Vec2   lo{-500, -500};
    Vec2   hi{ 500,  500};
    double cell     = 10.0;   // water grid step (m); shore detail comes from the shader
    double foamBand = 60.0;   // shore-distance cap baked into UV.v (m)
    // Which cells are sea, when not `floor`: the NATURAL ground, so a river channel cut below
    // sea level near its mouth stays the river's (hydrology waterMesh), not a staircase of sea.
    HeightSampler extent;
};

RenderMesh buildWaterMesh(const HeightSampler& floor, const WaterMeshParams& p);

// The sea cells buildWaterMesh emits, as squares: what other water surfaces clip against so
// the two meet edge to edge and never overlap.
std::vector<std::vector<Vec2>> waterMeshCells(const HeightSampler& floor, const WaterMeshParams& p);

}  // namespace engine

#endif
