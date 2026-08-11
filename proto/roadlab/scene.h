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

}  // namespace roadlab

#endif
