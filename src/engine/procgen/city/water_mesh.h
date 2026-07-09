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
};

RenderMesh buildWaterMesh(const HeightSampler& floor, const WaterMeshParams& p);

}  // namespace engine

#endif
