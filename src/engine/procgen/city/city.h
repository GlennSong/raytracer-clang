#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_CITY_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_CITY_H

#include "polygon.h"
#include "road_network.h"
#include "shape_grammar.h"
#include "../../../renderer/renderer.h"
#include <cstdint>
#include <functional>
#include <vector>

namespace engine {

// The city region recipe (ADR-0038 §1, §7 Phase 3): assemble the whole pipeline
// — roads → planarize → block faces → parcels → per-lot buildings — into a
// renderable model, with district-driven zoning (downtown towers → midtown →
// residential, plus parks). Deterministic for the seed (ADR-0002). Headless: the
// generation is pure data; only the final render needs a backend.

// Zoned areas of the city (the "define areas: residential, commercial, high-rise,
// industrial" axis). Laid out by radial falloff from the centre plus an industrial
// wedge; each maps to a building-archetype distribution (archetypeFor in city.cpp).
// Coastal is reserved for when water lands (it would key off a shoreline field).
enum class District : uint8_t {
    HighRise,      // downtown core: glass/metal towers with setbacks
    Commercial,    // office mid-rises, retail ground floors
    Residential,   // brick/stucco walk-ups and rowhouses
    Industrial,    // low, wide metal warehouses
    Park,          // green space + trees
    Count
};

struct CityParams {
    Vec2 center{0, 0};
    Real extent = 400;        // half-size of the city square (m)
    Real cellSize = 95;       // target block spacing (m)
    Real baseY = 0;           // ground elevation (later: sample terrain)
    Real roadJitter = 0.16;
    Real sidewalk = 3.0;      // extra inset beyond the road half-width
    Real downtownRadius = 130;// district falloff from the centre
    Real midtownRadius = 300;
    Real parkFraction = 0.10; // fraction of blocks left as parks
    Real buildChance = 0.9;   // per-lot occupancy (some lots become plazas)
    uint32_t seed = 1;

    // The City Arena (ADR-0027/0038 §6): drape the city on terrain. When set,
    // building foundations sit on the ground (at the min terrain height under
    // their footprint, so they never float on a slope) and roads follow it; when
    // null the city is flat at baseY and gets its own ground plane. Sampler is
    // world-XZ -> height. Kept out of the deterministic seed path (pure function).
    std::function<Real(const Vec2&)> groundAt;

    bool scatterTrees = true;     // street + park trees on terrain/ground
    Real streetTreeSpacing = 26;  // metres between street trees along a road
};

inline Real cityGroundAt(const CityParams& cp, const Vec2& p) {
    return cp.groundAt ? cp.groundAt(p) : cp.baseY;
}

// One placed building: its world meshes are already baked into the model parts,
// but we keep the footprint + summary so consumers can collide / inspect / LOD.
struct CityBuilding {
    Vec2 site;                // footprint centroid (world XZ)
    Real baseY = 0;           // foundation elevation (tracks terrain)
    Real height = 0;
    District district = District::Residential;
    // An oriented bounding box for a static collider (walk-into-it physics): the
    // viewer spawns one Box collider per building from these (ADR-0038 walkable).
    Vec3 boxCenter;           // world centre of the box
    Vec3 boxHalf;             // half-extents in the building's local frame
    Real yaw = 0;             // rotation about +Y (radians)
};

struct CityModel {
    // Geometry merged by material class across every building (one mesh per
    // non-empty PartId), so the whole city is a handful of draws. Each part keeps
    // its own materialIndex (== the PartId it was emitted under); map it to a
    // RenderMaterial with materialFor(static_cast<PartId>(part.materialIndex)).
    std::vector<RenderMesh> parts;
    RenderMesh roads;         // road + sidewalk surface (draped on terrain if set)
    RenderMesh ground;        // ground plane under the city (empty when on terrain)
    RenderMesh props;         // street + park trees (vertex-coloured, on the ground)
    int treeCount = 0;

    // HLOD proxy (ADR-0034 §5 / ADR-0038 §6): every building's coarse mass box
    // merged into one mesh — the cheap distant-city draw. Orders of magnitude
    // fewer triangles than the detailed parts; swap to it past a distance, or use
    // it as the source for an impostor-card bake (the bake itself needs a GPU).
    RenderMesh hlodProxy;

    std::vector<CityBuilding> buildings;
    std::vector<Poly2> blocks;     // extracted block footprints (debug / collision)
    int blockCount = 0;
    int lotCount = 0;

    RenderMesh mergedBuildings() const;   // all building parts in one mesh
};

CityModel generateCity(const CityParams& params);

// District at a world-XZ point, by radial falloff from the centre + a seeded park
// scatter. Exposed for tests and for terrain-material/biome coupling.
District districtAt(const CityParams& params, const Vec2& p, int blockIndex);

}  // namespace engine

#endif
