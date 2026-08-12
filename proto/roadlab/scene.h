#ifndef ROADLAB_SCENE_H
#define ROADLAB_SCENE_H

// A scene is a network plus everything derived from it. The derived half —
// paint, props, meshes, sim tables — is thrown away and rebuilt by
// finalizeScene(); only the network is authored. Roads can come from JSON, from
// a named demo, or from the generator, and all three land in exactly the same
// place, which is the point: procgen is not a separate pipeline, it is another
// front end onto the same builders.

#include "builders.h"
#include "props.h"
#include "raster.h"
#include "structure.h"
#include "surface.h"
#include "tessellate.h"
#include <string>
#include <vector>

namespace roadlab {

struct Scene {
    std::string name;
    Network net;
    PaintSet paint;
    std::vector<Prop> props;
    TerrainParams terrain;
    ShadeParams shade;
    PropRules propRules;
    TessParams tess;
    Mesh mesh;              // roads + junctions + structures + terrain + props
    std::vector<std::string> lint;
    bool built = false;
};

// Resolve junctions, build the lane graph, run the design lint, generate paint
// and props, then tessellate everything.
void finalizeScene(Scene& scene, bool withTerrain = true, bool withProps = true);

// Re-tessellate only the agent geometry on top of a finished scene.
void appendAgents(Scene& scene, const class Simulation& sim, Mesh& out);

// --- authoring ------------------------------------------------------------

bool loadSceneJson(const std::string& path, Scene& out, std::string& error);
bool saveSceneJson(const Scene& scene, const std::string& path, std::string& error);

std::vector<std::string> demoNames();
bool buildDemo(const std::string& name, Scene& out);

// --- generation -----------------------------------------------------------

struct CityParams {
    uint32_t seed = 7;
    int blocksX = 4;
    int blocksZ = 4;
    double blockSize = 150.0;
    double jitter = 18.0;
    double arterialEvery = 3;      // every Nth grid line is an arterial
    bool freeway = true;
    bool roundabout = true;
    bool overpass = true;
    bool parking = true;
    bool signals = true;
};

// A Piedmont-style layout: a grid of streets with arterials through it, a
// roundabout where one node would have been, and a freeway corridor with a
// grade separation and a pair of ramps. Deterministic in `seed`.
void generateCity(Scene& out, const CityParams& params);

// A whole metropolitan layout, rather than a grid with a freeway beside it.
//
// The difference from CityParams is structural, not cosmetic. A bypass has to
// WRAP the street network, which means the ring is the thing that gets planned
// first and the street network has to fit inside it; interchanges are then
// placed on the ring and each one pulls a radial arterial inward until it meets
// the grid. Blocks vary in size because the cell spacing is irregular and
// because whole runs of interior streets are removed to leave superblocks —
// a campus, a park, a rail yard.
struct MetroParams {
    uint32_t seed = 11;

    // Street lattice. Cells are irregular, so blocks come out different sizes
    // before anything else happens to them.
    int cols = 7;
    int rows = 6;
    double minCell = 95.0;
    double maxCell = 205.0;

    // Curvature. `warp` bends the whole lattice with smooth noise so streets
    // curve COHERENTLY — a warped grid still reads as a street plan, where
    // per-node jitter just reads as noise. `curveChance` then bows individual
    // edges on top, and the rest stay dead straight for contrast.
    double warp = 0.55;          // multiples of a cell at the largest scale
    double curveChance = 0.40;
    double curveBow = 0.10;      // bow as a fraction of the edge's length

    // Superblocks: rectangles of the lattice whose interior streets are removed.
    int superblocks = 5;

    // The ring, and how many interchanges it carries. Each interchange gets an
    // off-ramp, an on-ramp, and a radial arterial running inward to the grid.
    bool bypass = true;
    // Beyond the street network's CORNERS. A radial aimed at a corner has only
    // (standoff - ramp reach) of room to exist in, so this is what decides
    // whether the diagonal interchanges can be built at all.
    double bypassStandoff = 300.0;
    int interchanges = 5;
    std::string bypassPreset = "freeway3";

    bool roundabout = true;
    bool parking = true;
    bool signals = true;
};

void generateMetro(Scene& out, const MetroParams& params);

}  // namespace roadlab

#endif
