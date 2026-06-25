#ifndef RAYTRACER_ENGINE_PROCGEN_PROC_MODEL_H
#define RAYTRACER_ENGINE_PROCGEN_PROC_MODEL_H

#include "../../renderer/renderer.h"   // RenderMesh
#include "../../rt_math.h"             // Mat4
#include "terrain.h"                   // TerrainFlatten
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
    // Analytic surface library id (renderer.h Surface; 0 = none). Evaluated at shade
    // time from the mesh's own UVs — e.g. RoadMarkings paints lane lines from the
    // welded carriageway's road-local u=lateral / v=arc-length, no stripe geometry.
    int surface = 0;
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
// A physics collider for a procedural model (ADR-0042). Procgen scenery is
// static world geometry, so collision follows the *actual* generated mesh:
// `Mesh` is the real triangle surface (terrain, roads, building shells) baked
// into one static Jolt mesh body — accurate, the standard for level geometry.
// The primitive kinds are for shapes that genuinely ARE a primitive (a round
// tower is exactly a capsule), not as a loose bounding approximation. The
// viewer turns each of these into the same Collider/MeshCollider + RigidBody
// the hand-authored loaders build; the offline path tracer (no physics) ignores
// them. Mesh triangles are world-space; primitives carry their own placement.
struct ProcCollider {
    enum class Kind { Mesh, Box, Sphere, Capsule };
    Kind kind = Kind::Mesh;
    RenderMesh mesh;                 // Kind::Mesh — exact static triangles
    Vec3 center{0, 0, 0};            // primitive placement (world)
    Real yaw = 0;                    // primitive Y rotation (radians)
    Vec3 halfExtent{0.5, 0.5, 0.5};  // Box
    Real radius = 0.5;               // Sphere, Capsule
    Real halfHeight = 0.5;           // Capsule (half the cylinder segment)
    Real friction = 0.6;
    bool dynamic = false;            // static world geometry by default
};

// How an instance group collides: each placement stamps a collider. `None`
// leaves the props non-solid; `Mesh` bakes the proto's triangles at every
// transform into the shared static mesh body; the primitive kinds place one
// Collider per instance (a lamp post is exactly a thin capsule).
enum class InstanceCollision { None, Mesh, Box, Sphere, Capsule };

struct ProcInstanceGroup {
    RenderMesh proto;
    std::vector<Mat4> transforms;
    float metallic = 0.0f;
    float roughness = 0.85f;
    bool  alphaFoliage = false;
    // Optional per-instance collision (ADR-0042). `collisionProto` overrides the
    // render proto for Mesh collision (a cheaper-but-faithful collision surface);
    // empty means collide with the render proto itself.
    InstanceCollision collision = InstanceCollision::None;
    RenderMesh collisionProto;
    Real colliderRadius = 0.5, colliderHalfHeight = 0.5;
    Vec3 colliderHalfExtent{0.5, 0.5, 0.5};
    Real colliderFriction = 0.6;
};

struct ProcModel {
    std::vector<ProcPart> parts;
    std::vector<ProcInstanceGroup> instances;
    std::vector<ProcCollider> colliders;
    // Cut/fill footprints (ADR-0044) the recipe wants graded into the *level*
    // terrain — the script-side sibling of CityModel::flatten. A recipe that
    // drapes on the engine terrain (terrain.conform with the injected `ground`)
    // hands these back via m:conform(regions); the loader folds them into the
    // CDLOD TerrainParams before the terrain mesh/collider is built, so the
    // ground meets the roads. Empty for a recipe that owns its own ground.
    std::vector<TerrainFlatten> flatten;

    // Fold another model into this one (composition).
    void merge(const ProcModel& o) {
        parts.insert(parts.end(), o.parts.begin(), o.parts.end());
        instances.insert(instances.end(), o.instances.begin(), o.instances.end());
        colliders.insert(colliders.end(), o.colliders.begin(), o.colliders.end());
        flatten.insert(flatten.end(), o.flatten.begin(), o.flatten.end());
    }

    int instanceCount() const {
        int n = 0;
        for (const ProcInstanceGroup& g : instances)
            n += static_cast<int>(g.transforms.size());
        return n;
    }

    int colliderCount() const { return static_cast<int>(colliders.size()); }
};

}  // namespace engine

#endif
