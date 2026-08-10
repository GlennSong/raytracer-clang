#ifndef RAYTRACER_ENGINE_ROAD_CHUNKS_H
#define RAYTRACER_ENGINE_ROAD_CHUNKS_H

#include "world.h"

namespace engine {

class AssetManager;
struct RoadNet;
struct RenderMesh;

// One spatial cell of rendered road per chunk entity. Matches the citysim
// building bake's renderCell default so the two cull at the same granularity.
constexpr double kRoadRenderCell = 250.0;

// Build (or REBUILD) the rendered carriageway for a road entity as per-cell
// chunk companion entities, so the frustum cull works street by street instead
// of treating the whole network as one always-visible draw (Piedmont measured
// 587 K triangles in a single road mesh — docs/city-render-perf.md R1).
//
// This is the ONE owner of the RoadNet -> rendered-mesh step. The loader, the
// editor's node-drag/property regenerates, and the City Planner's [Bake] all
// call it, so the material setup (lane-marking surface, vertex-colour albedo)
// and the mesh lifetime exist in exactly one place. Meshes go through the
// AssetManager (acquire/release), which also retires the documented
// uploadMesh-per-regenerate leak.
//
// The previous chunk set (RenderChunks on `road`) is destroyed and its meshes
// released before the new one spawns. Chunks carry PickTarget{road} so
// clicking a street in the editor still selects the editable road entity.
// `prebuilt` lets a caller that already meshed the net (the loader builds it
// once for the collider too) skip a second multi-second mesh build; when
// null, the helper meshes the net itself. Returns the number of chunks.
int rebuildRoadRenderChunks(World& world, AssetManager& assets, Entity road,
                            const RoadNet& net, double cell = kRoadRenderCell,
                            const RenderMesh* prebuilt = nullptr);

}  // namespace engine

#endif
