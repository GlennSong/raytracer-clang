#ifndef RAYTRACER_ENGINE_GLB_EXPORT_H
#define RAYTRACER_ENGINE_GLB_EXPORT_H

// Binary glTF export of render meshes — the inspection format (Blender: File → Import → glTF 2.0), not a
// load format (ADR-0084). One glTF mesh + node + flat PBR material per entry; POSITION, NORMAL, TANGENT
// (w = 1), TEXCOORD_0, and COLOR_0 when the vertex colour varies across the mesh; u32 indices. Extras
// JSON rides on each node and on the scene, so a bundle's non-mesh data (road graph, blocks, manifest) is
// visible to anything that reads glTF, while Blender simply ignores it.

#include "renderer/renderer.h"
#include "rt_math.h"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace engine {

struct TextImage;

struct GlbEntry {
    std::string name;            // node, mesh and material name
    const RenderMesh* mesh = nullptr;
    Vec3 color{0.7, 0.7, 0.7};   // material base colour
    float roughness = 0.9f;
    nlohmann::json extras;       // per-node extras (object or null)
    // A picture on the surface (TEXCOORD_0), embedded as PNG once however many entries share it: a sign face's
    // atlas page. `alphaCut`: transparent texels are not drawn (glTF alphaMode MASK at 0.5), as the engine's
    // FLAG_ALPHA_TEST does.
    const TextImage* texture = nullptr;
    bool alphaCut = false;
    float metallic = 0.0f;
    nlohmann::json materialExtras;   // per-material extras (object or null): data a renderer may use later
};

bool writeGlb(const std::vector<GlbEntry>& entries, const std::string& path, const nlohmann::json& sceneExtras, std::string* error);

// --- A TREE OF NODES AS A glTF FILE (#191: a model written out) --------------------------------------------------------
// `writeGlb` above writes a flat list for looking at. This writes what a model is: nodes under nodes, each with its
// own position, turn and scale, several meshes on one node (one glTF primitive per material), nodes with no mesh
// (empties), and one mesh used by many nodes. It writes TRUE glTF, so another tool reads it right and the engine's
// importer reads it back as it was: triangles counter-clockwise (the engine's are clockwise; the importer turns them
// back) and every vertex's colour (COLOR_0), which is where a model keeps its hue.
struct GlbTreeMaterial {
    std::string name;
    Vec3 color{1, 1, 1};             // base colour (a model's is white: the hue is on the vertices)
    float metallic = 0.0f, roughness = 0.9f, opacity = 1.0f;
    Vec3 emission{0, 0, 0};
    bool twoSided = false;
    // maps, embedded as PNG (empty: none). RGB or RGBA bytes, row-major.
    int albedoW = 0, albedoH = 0, albedoChannels = 0;
    const std::vector<uint8_t>* albedo = nullptr;
    int normalW = 0, normalH = 0, normalChannels = 0;
    const std::vector<uint8_t>* normal = nullptr;
    nlohmann::json extras;           // object or null
};
struct GlbTreePrimitive {
    const RenderMesh* mesh = nullptr;
    int material = -1;               // an index into the materials given; -1: none
};
struct GlbTreeNode {
    std::string name;
    int parent = -1;                 // an index into the nodes given, lower than this node's own; -1: a top object
    Vec3 translation{0, 0, 0};
    Quat rotation;
    Real scale = 1;
    std::vector<GlbTreePrimitive> primitives;   // this node's mesh; empty with meshOf < 0: an empty
    int meshOf = -1;                 // draw the same mesh as this earlier node (stored once), instead of `primitives`
    nlohmann::json extras;           // object or null
};
bool writeGlbTree(const std::vector<GlbTreeNode>& nodes, const std::vector<GlbTreeMaterial>& materials, const std::string& path,
                  const nlohmann::json& sceneExtras, std::string* error);

}  // namespace engine

#endif
