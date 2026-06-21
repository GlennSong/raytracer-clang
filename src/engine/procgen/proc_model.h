#ifndef RAYTRACER_ENGINE_PROCGEN_PROC_MODEL_H
#define RAYTRACER_ENGINE_PROCGEN_PROC_MODEL_H

#include "../../renderer/renderer.h"   // RenderMesh
#include "../../rt_math.h"             // Mat4
#include "tree.h"                      // TextureData
#include <vector>

namespace engine {

// A baked material for a procedural part (ADR-0043): a bundle of map sets +
// scalar PBR params, applied with a world-planar tiling frame at `tile` world
// units per repeat (no authored UVs needed). Empty maps mean "use the scalar /
// vertex colour". The same bundle binds into the path tracer's Material and the
// viewer's RenderMaterial — the renderer is the only divergence.
struct ProcMaterial {
    bool textured = false;
    TextureData albedo;        // RGB albedo map (empty = vertex colour only)
    TextureData normal;        // tangent-space normal map (empty = none)
    float metallic = 0.0f;
    float roughness = 0.85f;
    double tile = 1.0;         // world units per texture repeat
};

// A model part: geometry + its material (default = untextured, vertex-coloured).
struct ProcPart {
    RenderMesh mesh;
    ProcMaterial material;
};

// A composable procedural model (ADR-0042): the value a recipe returns at any
// granularity — a single part, a collection of parts, or a whole scene. It is
// the script-side sibling of CityModel: opaque vertex-coloured part meshes plus
// instance groups (a prototype placed by many transforms, ADR-0041). Models nest
// via merge, so a block model embeds building + prop models and a city model
// embeds block models — "individual parts or collected together, or anything in
// between." (Colliders + attach points are a planned addition.)
struct ProcInstanceGroup {
    RenderMesh proto;
    std::vector<Mat4> transforms;
    float metallic = 0.0f;
    float roughness = 0.85f;
    bool  alphaFoliage = false;
};

struct ProcModel {
    std::vector<ProcPart> parts;
    std::vector<ProcInstanceGroup> instances;

    // Fold another model into this one (composition).
    void merge(const ProcModel& o) {
        parts.insert(parts.end(), o.parts.begin(), o.parts.end());
        instances.insert(instances.end(), o.instances.begin(), o.instances.end());
    }

    int instanceCount() const {
        int n = 0;
        for (const ProcInstanceGroup& g : instances)
            n += static_cast<int>(g.transforms.size());
        return n;
    }
};

}  // namespace engine

#endif
