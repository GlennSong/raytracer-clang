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

}  // namespace engine

#endif
