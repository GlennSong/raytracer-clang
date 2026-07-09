#ifndef RAYTRACER_ENGINE_TERRAIN_H
#define RAYTRACER_ENGINE_TERRAIN_H

#include "../../renderer/renderer.h"   // RenderMesh
#include "noise.h"
#include <memory>
#include <vector>

namespace engine {

// One ridge axis segment of a (possibly branching) mountain range: a ground-plane
// segment a->b with per-endpoint crest height.
struct RidgeSegment {
    Vec3  a, b;          // ground positions (x, 0, z)
    float ha = 0.0f, hb = 0.0f;
};

// A footprint where the terrain is graded flat — cut where the natural ground is
// too high, filled where it's too low — to a target surface, so a road or city
// block sits on level earth instead of the raw noise (ADR-0038). The target is a
// plane y = c + dx*x + dz*z: a constant pad for a block (dx=dz=0), or a ramp for
// a road segment that climbs along its length. Inside `polygon` the height is
// forced to the plane; within `falloff` metres outside, it eases back to the
// natural terrain (smoothstep) so the cut blends in rather than leaving a cliff.
struct TerrainFlatten {
    std::vector<Vec3> polygon;   // world footprint; uses .x and .z (.y ignored)
    double c = 0.0, dx = 0.0, dz = 0.0;
    double falloff = 6.0;
    // Tight XZ AABB, filled by the makers; queries quick-reject against it
    // (expanded by falloff) before the per-edge polygon test.
    double minX = 0, minZ = 0, maxX = 0, maxZ = 0;
    // How the graded surface returns to natural ground OUTSIDE the footprint
    // (ADR-0075 Phase 1). Smoothstep: ease plane -> natural over `falloff` (the
    // original feather). DaylightBatter: leave the deck edge on a real earthwork
    // slope — a CUT batter climbing the uphill side, a FILL embankment descending
    // the downhill side — until it DAYLIGHTS where it meets natural ground. Where
    // it can't daylight within `falloff` (the reach), the residual step is capped
    // by a retaining wall (emitted as a StructureSet, not baked into the terrain).
    enum class Falloff { Smoothstep, DaylightBatter };
    Falloff falloffMode = Falloff::Smoothstep;
    double cutBatter = 1.0;    // uphill cut slope, rise:run (1.0 = 45°, steeper rock cut)
    double fillBatter = 0.6;   // downhill fill embankment, rise:run (gentler than cut)
    double planeY(double x, double z) const { return c + dx * x + dz * z; }
};

// A constant-height pad over a polygon (a city block apron sits flat at targetY).
TerrainFlatten makeFlattenPad(std::vector<Vec3> polygon, double targetY,
                              double falloff = 6.0);
// A ramped corridor for a road segment a->b carrying heights yA..yB across a
// rectangle of the given half-width (so the terrain follows the road's grade).
TerrainFlatten makeFlattenRamp(const Vec3& a, const Vec3& b, double yA, double yB,
                               double halfWidth, double falloff = 6.0);

// Blend a set of flatten footprints over a natural height at one point: the
// strongest (closest) footprint wins, levelling to its plane inside and easing
// back to `base` across each footprint's falloff. The pointwise core both the
// noise terrain and the composable HeightField (terrain.conform) cut/fill with.
double applyFlatten(const std::vector<TerrainFlatten>& regions, double x,
                    double z, double base);
// Dilated variant: every footprint is grown by `dilate` metres for this query.
// Coarse samplers (a distant CDLOD node) pass ~half their cell size so a grid
// triangle can never straddle a narrow corridor and lift natural ground over it.
double applyFlatten(const std::vector<TerrainFlatten>& regions, double x,
                    double z, double base, double dilate);

// Spatial index over flatten footprints (ADR-0075 Phase 0): a uniform grid,
// sized to the union of the footprints (i.e. the city, not the whole world),
// binning each region by its falloff-expanded AABB. A height query then tests
// only the footprints near it instead of the whole list — applyFlatten is
// otherwise O(regions) per sample, and it runs per CDLOD vertex under
// terrainHeight, so it dominates terrain build at city scale. Pure accelerator:
// the indexed applyFlatten returns bit-for-bit the same height as the linear
// scan (parity-tested). Build once when the footprint set changes.
struct FlattenGrid {
    double originX = 0, originZ = 0;      // min corner of the union AABB
    double cell = 0;                       // grid pitch (m)
    int    nx = 0, nz = 0;                 // grid dimensions
    std::vector<std::vector<int>> bins;    // nx*nz cells, each a list of region indices
    bool empty() const { return bins.empty(); }
};
FlattenGrid buildFlattenGrid(const std::vector<TerrainFlatten>& regions);
// Indexed applyFlatten: identical result to the linear overloads, but visits
// only the regions binned near (x,z). `grid` must have been built from `regions`.
double applyFlatten(const FlattenGrid& grid, const std::vector<TerrainFlatten>& regions,
                    double x, double z, double base, double dilate);

// Heightfield terrain (ROADMAP 4 Phase B.2) — the first generator combining the
// noise field (3.7) and the mesh builder (3.3). Deterministic for a given Noise,
// so the same recipe rebuilds the same terrain (ADR-0021).
struct TerrainParams {
    float size = 100.0f;       // world units across, centered on the origin (XZ)
    int   resolution = 64;     // grid cells per side; vertices = (resolution+1)^2
    float heightScale = 10.0f; // peak height (noise is ~[-1,1] * this)
    double noiseScale = 0.02;  // frequency: multiplies world coords before noise
    int   octaves = 5;
    double warp = 0.0;         // domain-warp amount (0 = plain FBM)
    // Long-range relief: a ridged-MULTIFRACTAL layer (varied, sharp, irregular
    // peaks — not uniform bumps), domain-warped, added on top of the hill octaves.
    // 0 = off.
    float  mountainHeight = 0.0f;
    double mountainScale = 0.004;
    // Regional mask: where mountains rise vs grassland. A large-scale field gates
    // the mountain layer (smoothstep maskLo..maskHi over it), so a range sits in
    // part of the map and plains/foothills elsewhere. 0 scale = mountains
    // everywhere (no regions).
    double mountainMaskScale = 0.0;
    float  mountainMaskLo = -0.12f;
    float  mountainMaskHi = 0.30f;
    // A mountain RANGE driven by a spine curve (range axis), instead of (or on
    // top of) the regional mask: uplift falls off with distance from the spine
    // (range -> foothills -> plains) and varies along it (tall massifs, low
    // passes). `rangeSpine` is a polyline of (x,_,z) points (sampled from the
    // authored control curve by the loader); empty = no range.
    std::vector<Vec3> rangeSpine;
    float rangeWidth = 220.0f;     // cross-axis falloff distance (to plains)
    float rangeHeight = 0.0f;      // peak uplift on the spine (0 = off)
    float rangeVariation = 0.55f;  // 0..1 along-spine height variation

    // A BRANCHING range: a ridge network (compiled from an L-system skeleton by
    // the loader). Each segment is a ridge axis with per-endpoint height (tall
    // main divide -> lower spurs by branch depth). Empty = no branching range.
    std::vector<RidgeSegment> rangeRidges;

    // Cut/fill footprints (ADR-0038): where a city road or block sits, the raw
    // noise is graded flat to a target surface so the ground doesn't poke through
    // the carriageway and the road sits on level earth. Applied last in
    // terrainHeight, so the mesh, the collider, the CDLOD field and every
    // placement query all see the same levelled ground. Empty = pristine terrain.
    std::vector<TerrainFlatten> flatten;
    // Optional spatial index over `flatten` (ADR-0075 Phase 0). Null => the
    // linear scan (unchanged behaviour). Shared so copying the recipe into
    // closures/entities stays cheap; rebuild via buildFlattenGrid after the
    // footprint set is assembled. terrainHeight uses it when present.
    std::shared_ptr<const FlattenGrid> flattenIndex;
};

// Refresh params.flattenIndex from params.flatten (ADR-0075 Phase 0). The index
// is a derived cache keyed by position in `flatten`, so it MUST be rebuilt in
// lockstep whenever the cut/fill set is assigned or mutated — a stale index would
// sample the wrong regions. Cheap; clears the index when `flatten` is empty. Call
// after finishing a flatten assembly; copies that never touch `flatten` keep the
// (shared) index safely.
void rebuildFlattenIndex(TerrainParams& params);

// Build a branching ridge network from a planar L-system skeleton (consumer #2
// of the shared Skeleton): a main divide throws off spur ridges, sub-spurs, etc.
// Height falls by branch depth (depthFalloff). Lays the turtle's growth plane
// (x, y) onto the ground (x, z). Feeds TerrainParams::rangeRidges.
std::vector<RidgeSegment> buildRangeRidges(float length, float branchAngle,
                                           float falloff, float leaderFalloff,
                                           int iterations, float height,
                                           float depthFalloff, float angleJitter,
                                           uint32_t seed);

// Sample the terrain height at a world (x, z). The single source of truth for
// the surface — the mesh is built from it, and scatter/placement queries it
// directly (height, and slope via finite differences) without needing the mesh.
double terrainHeight(const TerrainParams& params, const Noise& noise,
                     double worldX, double worldZ);
// Coarse-sampler variant: `flattenDilate` grows the cut/fill footprints for
// this query (see applyFlatten's dilated overload). 0 = exact footprints.
double terrainHeight(const TerrainParams& params, const Noise& noise,
                     double worldX, double worldZ, double flattenDilate);

// Sample a smooth spine polyline from authored control points (Catmull-Rom) for
// TerrainParams::rangeSpine. Points are (x, 0, z).
std::vector<Vec3> sampleRangeSpine(const std::vector<Vec3>& controls,
                                   int samples = 80);

// Height/slope-based ground color (ROADMAP 4 Phase D): grass on low flats, rock
// on steep or high ground, plus a noise term to break up the bands. `normalUp`
// is the surface normal's y (1 flat, 0 vertical); `noiseValue` ~[-1,1] varies
// it. Pure and testable; baked into the terrain's per-vertex colors.
Vec3 terrainColor(double height, double normalUp, double noiseValue);

// Build the terrain mesh: a grid in the XZ plane with y = terrainHeight, smooth
// normals, planar UVs spanning [0,1], and per-vertex height/slope coloration
// (terrainColor) baked in. Centered on the origin.
RenderMesh generateTerrain(const TerrainParams& params, const Noise& noise);

// Build one square annular ring of terrain from inner to outer half-extent at
// `cells` resolution (coarse), with a hole for the inner (higher-detail) tile.
// Used to extend terrain to the horizon as concentric coarsening LOD rings —
// cheap distant mountains/hills. Same height field as generateTerrain.
RenderMesh generateTerrainRing(const TerrainParams& params, const Noise& noise,
                               float innerHalf, float outerHalf, int cells);

// Concentric LOD rings around the central tile: `levels` rings, each doubling
// the extent (so triangle count per ring stays ~constant while coverage grows
// geometrically). Ring 0 starts at the central tile's edge (size/2).
std::vector<RenderMesh> generateTerrainLOD(const TerrainParams& params,
                                           const Noise& noise, int levels,
                                           int cells = 40);

// One generated terrain chunk: a world-space mesh plus its tight world AABB, grid
// coordinate, and whether it should carry a collider. Position is baked into the
// vertices (identity transform), matching generateTerrain.
struct TerrainChunk {
    RenderMesh mesh;
    Vec3 boundsMin;        // world-space tight AABB (spans the chunk's height range)
    Vec3 boundsMax;
    int  cx = 0, cz = 0;   // chunk grid coordinate (0,0 = min corner)
    bool collider = false; // near chunks: the player walks on them
};

// Tile a bounded square world into a grid of terrain chunks (ADR-0034 Phase 1),
// replacing the single origin-centred tile + concentric rings. The world is
// `chunksPerSide` x `chunksPerSide` chunks of `chunkSize` world units each,
// centred on the origin. Each chunk is meshed independently at `resolution` cells
// per side from the shared height field, so it has its own tight AABB for frustum
// culling. Normals are taken analytically from the height field (finite
// differences), so they are continuous across chunk borders and seams don't show
// — no T-junction cracks (matching grids share exact edge vertices) and no skirts
// needed. A chunk is flagged for a collider when its centre is within
// `colliderRadius` world units of the origin.
std::vector<TerrainChunk> generateTerrainChunks(const TerrainParams& params,
                                                const Noise& noise,
                                                int chunksPerSide, float chunkSize,
                                                int resolution, float colliderRadius);

// Analytic surface normal of the height field at world (x, z), from central
// finite differences at offset `eps`. Used for seamless per-chunk normals; also
// handy for placement queries.
Vec3 terrainNormal(const TerrainParams& params, const Noise& noise,
                   double worldX, double worldZ, double eps);

}  // namespace engine

#endif
