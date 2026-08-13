#ifndef ENGINE_PROCGEN_ROADLAB_BRIDGE_H
#define ENGINE_PROCGEN_ROADLAB_BRIDGE_H

// The one place src/ knows proto/roadlab exists.
//
// roadlab is a clean-room prototype and its README promises isolation in both
// directions. Loading it into the editor necessarily breaks that once — so it
// breaks it HERE, in a single translation unit, rather than by scattering
// roadlab types through level_scene.cpp. Nothing else in src/ includes a
// roadlab header, and this file's interface is engine types only, so deleting
// the prototype means deleting two files and one `shape` branch.
//
// What it does is regenerate a road network at load and hand back RenderMeshes,
// the same contract `shape:"city"` already has (level_scene.cpp: generate the
// model, pull its terrain footprints out, bake the parts). Nothing is
// serialised: the generator runs in 5 ms and the format that would carry its
// output between two halves of one program would be pure overhead.

#include "renderer/renderer.h"

#include <string>
#include <vector>

namespace roadlab_bridge {

// What a `shape:"roadlab"` entity asks for. Mirrors the CLI, because the CLI is
// how the prototype is already driven and a second vocabulary would drift.
struct Recipe {
    std::string generator = "showcase";   // a demo name, or "city" / "metro"
    unsigned seed = 7;
    double terrainAmp = -1;               // <0 keeps the generator's own choice
    double terrainWave = -1;
    bool terrain = true;
    bool props = true;
};

// The road network as engine meshes, split the way the baker wants them: road
// surface and junction pads take the procedural asphalt surface, everything else
// (terrain, structures, props) carries vertex colour.
//
// `u` and `v` carry the road-local (t, s) IN METRES — not the engine's
// half-width-normalised convention. That is the whole point of the prototype and
// the coordinate the ported marking shader reads; a first bake that only wants
// asphalt can ignore them.
struct Baked {
    engine::RenderMesh roadSurface;
    engine::RenderMesh junctionPads;
    engine::RenderMesh flat;
    int roads = 0, junctions = 0;
    double buildMs = 0;
};

// Regenerate and convert. Returns false with `error` set if the generator name
// is not one roadlab knows.
bool build(const Recipe& recipe, Baked& out, std::string& error);

}  // namespace roadlab_bridge

#endif
