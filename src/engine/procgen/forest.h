#ifndef RAYTRACER_ENGINE_PROCGEN_FOREST_H
#define RAYTRACER_ENGINE_PROCGEN_FOREST_H

// FORESTS (ADR-0129, Glenn: "Forests with many trees as well"). A level-scale forest is hundreds of
// thousands of trees, so it is not a scatter of instanced models; it is three things:
//   1. WHERE: a density field from the ground itself -- soil (terrain_maps.h), slope, the cover's rock,
//      sand and snow, a wandering TREELINE, and stands and clearings (a slow noise) -- sampled on a
//      jittered grid at the stand's spacing: a full stand where the density is 1, thinning at its edges.
//   2. WHAT: the species by altitude, each preferring its own band, with a patchy noise per species so
//      a hillside holds stands of one kind rather than a salad; stunted near the treeline.
//   3. HOW FAR: real_tree.h models near (instanced, per cell), IMPOSTOR CARDS far -- two crossed side
//      cards and a top card per tree, merged per cell into one mesh on one atlas -- crossfaded per
//      pixel over a distance band (RenderMaterial::FLAG_LOD_BAND).
// The city keeps it out through the caller's `exclude` (roads, pads, graded lots).

#include "../../renderer/renderer.h"   // RenderMesh
#include "../../rt_math.h"
#include "real_tree.h"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace engine {

class GroundCover;
struct TerrainMaps;

struct ForestSpecies {
    RealSpecies species = RealSpecies::Spruce;
    int variants = 4;
    double altLo = 0.0, altHi = 1e9;   // metres above the sea it prefers (soft edges)
    double weight = 1.0;
    double wet = 0.0;                  // affinity for wet ground (the maps' drainage): 1 = willow, alder
    bool understory = false;           // a bush under and around the canopy, in its own pass
};

// ROCKS (ADR-0131, Glenn: "scattered rocks"): layers placed by the same fast grid, each by what the
// ground's maps say is there -- boulder fields on scree, outcrops on bare convex rock, erratics in the
// open, mossy stones under the canopy, stones on the beach.
struct RockLayer {
    std::string family = "boulder", stone = "granite";   // the rock library's names (stylized_rock.h)
    double size = 2.0;
    int variants = 3;
    int ground = 0;              // 0 scree, 1 outcrop, 2 field, 3 forest, 4 beach, 5 shore (rivers, lakes)
    double spacing = 12.0;       // its grid
    double density = 1.0;        // the share of grid points kept where the ground is right
    double moss = -1.0;          // -1: the stone's default
    double drawM = 600.0;
};

struct ForestParams {
    std::vector<ForestSpecies> species;
    double spacing = 7.5;          // metres between trees in a full stand
    double coverage = 0.55;        // share of the eligible land that is forest
    double standScaleM = 650.0;    // stands and clearings
    double treelineM = 620.0;      // above the sea (wanders +-90 m)
    double shoreClearM = 4.0;      // no trees within this height of the sea
    double maxSlopeDeg = 40.0;
    double cellM = 256.0;          // cull cells (near groups, far merged meshes)
    double nearM = 150.0;          // full models to here, crossfading over the last nearFadeM
    double nearFadeM = 40.0;
    double farM = 7000.0;          // impostors to here
    double understorySpacing = 5.0;  // the understory pass's grid
    double understoryDensity = 0.35; // its share where the canopy is, doubled along the edges
    uint32_t seed = 1;
    std::vector<RockLayer> rocks;
};

ForestParams forestFromJson(const nlohmann::json& j);

struct ForestTree {
    Vec3 pos;
    float yaw = 0.0f, scale = 1.0f;
    uint16_t variant = 0;          // index into the global variant list (species-major)
};

// The canopy's cover at a point, 0..1, WITHOUT asking the ground cover (the cover asks this, to lay
// leaf and needle litter under the trees): the density's stands, soil, shore, treeline and slope.
double forestCanopy(const ForestParams& p, double seaLevel, const TerrainMaps* maps, double x, double z, double y,
                    double slopeDeg);

// Place the forest over the square [-half, half]^2. `ground` is the drawn ground (thread-safe),
// `exclude` true where no tree may stand (roads, pads, water). Variant `v` of species `s` is global
// index s * variantsPerSpecies + v (variantsPerSpecies = the max over species).
std::vector<ForestTree> placeForest(const ForestParams& p, double half, double seaLevel, const GroundCover* cover,
                                    const TerrainMaps* maps, const std::function<double(double, double)>& ground,
                                    const std::function<bool(double, double)>& exclude, int variantsPerSpecies);

struct PlacedRock {
    Vec3 pos;
    float yaw = 0.0f, scale = 1.0f, tiltDir = 0.0f, tilt = 0.0f;
    uint16_t layer = 0, variant = 0;
};
std::vector<PlacedRock> placeRocks(const ForestParams& p, double half, double seaLevel, const GroundCover* cover,
                                   const TerrainMaps* maps, const std::function<double(double, double)>& ground,
                                   const std::function<bool(double, double)>& exclude,
                                   const std::function<double(double, double, double)>& shore = {},
                                   const std::function<bool(double, double)>& excludeShore = {});   // the shore layer's own

// SEATING A ROCK on the ground (#54): the ground's normal across the footprint (radius `foot`), and the height
// y0 of the rock's base plane at its centre -- tilted with it, nowhere above the ground over the footprint
// (centre + 8 points) and `bed` below it where it rides highest. The mesh's bottom goes on that plane.
struct Seat { Vec3 normal{0, 1, 0}; double baseY = 0.0; };
Seat seatOnGround(const std::function<double(double, double)>& ground, double x, double z, double foot, double bed);

// One tree variant's impostor slot in the atlas, and the tree's measures (unit scale).
struct ImpostorSlot {
    double u0 = 0, v0 = 0, u1 = 1, v1 = 1;     // side picture
    double tu0 = 0, tv0 = 0, tu1 = 1, tv1 = 1; // top picture
    double halfW = 1.0, height = 1.0, crownBase = 0.0;
};

// Append `t`'s impostor (two crossed side cards + a top card) to `mesh`, world space. `colour` is the
// vertex colour (1 / the atlas's colour scale).
void appendImpostor(RenderMesh& mesh, const ForestTree& t, const ImpostorSlot& slot, double colour);

}  // namespace engine

#endif
