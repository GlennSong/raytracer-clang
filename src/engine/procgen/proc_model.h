#ifndef RAYTRACER_ENGINE_PROCGEN_PROC_MODEL_H
#define RAYTRACER_ENGINE_PROCGEN_PROC_MODEL_H

#include "../../renderer/renderer.h"   // RenderMesh
#include "../../rt_math.h"             // Mat4
#include <vector>

namespace engine {

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
    std::vector<RenderMesh> parts;
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
