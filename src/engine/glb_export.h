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

struct GlbEntry {
    std::string name;            // node, mesh and material name
    const RenderMesh* mesh = nullptr;
    Vec3 color{0.7, 0.7, 0.7};   // material base colour
    float roughness = 0.9f;
    nlohmann::json extras;       // per-node extras (object or null)
};

bool writeGlb(const std::vector<GlbEntry>& entries, const std::string& path, const nlohmann::json& sceneExtras, std::string* error);

}  // namespace engine

#endif
