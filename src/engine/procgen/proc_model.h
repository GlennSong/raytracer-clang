#ifndef RAYTRACER_ENGINE_PROCGEN_PROC_MODEL_H
#define RAYTRACER_ENGINE_PROCGEN_PROC_MODEL_H

#include "../../renderer/renderer.h"   // RenderMesh
#include "../../rt_math.h"             // Mat4
#include "terrain.h"                   // TerrainFlatten
#include "tree.h"                      // TextureData
#include <string>
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
    // Transparency, emission and two-sidedness were MISSING from this struct,
    // so `material.new{ opacity = ..., emission = ... }` parsed fine in Lua and
    // was then silently dropped on the way to the renderer. Car glass authored
    // at 0.32 rendered fully opaque, which is why a cabin with seats and a
    // driver in it looked like a solid panel. Anything a recipe can set must
    // exist here or it does not survive the trip.
    float opacity = 1.0f;
    Vec3 emission{0, 0, 0};
    bool twoSided = false;     // -> RenderMaterial::FLAG_TWO_SIDED
};

// Where a thing sits and how it is turned, relative to whatever holds it: a position and a rotation, no scale. A
// node's frame is relative to its parent node; the frame `ProcModel::nodeFrame` returns is relative to the model.
struct ProcFrame {
    Vec3 position{0, 0, 0};
    Quat rotation;

    // A point given in this frame, in the space the frame sits in.
    Vec3 point(const Vec3& local) const { return position + rotation.rotate(local); }
    // `inner` is a frame given in this one; the result is `inner` in the space this frame sits in.
    ProcFrame operator*(const ProcFrame& inner) const { return ProcFrame{point(inner.position), rotation * inner.rotation}; }
    Mat4 matrix() const { return Mat4::trs(position, rotation, Vec3(1, 1, 1)); }
};

// One named piece of a model (#191, model step 1): a car's door is a node under the car, the handle a node under the
// door. A node holds no geometry itself; a part says which node it hangs on, and is drawn at that node's frame.
struct ProcNode {
    std::string name;      // unique in the model
    int parent = -1;       // an index into ProcModel::nodes, always lower than this node's own; -1 for the root only
    ProcFrame frame;       // relative to the parent
};

// A model part: geometry + its material (default = untextured, vertex-coloured). The mesh is in the space of the
// node it hangs on; node 0 is the root, which is the model's own space, so a part that names no node is where its
// vertices say it is.
struct ProcPart {
    RenderMesh mesh;
    ProcMaterial material;
    int node = 0;
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
    // The model's pieces as a tree. There is always a root, node 0, whose frame is the model's own space and never
    // changes; a model that adds no node is the flat model every recipe built before nodes existed. Instance groups,
    // colliders and flatten footprints are in the root's space.
    std::vector<ProcNode> nodes = {ProcNode{"root", -1, ProcFrame{}}};
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

    // The node with this name, or -1.
    int findNode(const std::string& name) const {
        for (std::size_t i = 0; i < nodes.size(); ++i)
            if (nodes[i].name == name) return static_cast<int>(i);
        return -1;
    }

    // Add a node under `parent`. Returns its index, or -1 (and adds nothing) when the name is empty or taken, or the
    // parent is not a node of this model.
    int addNode(const std::string& name, int parent, const ProcFrame& frame) {
        if (name.empty() || findNode(name) >= 0) return -1;
        if (parent < 0 || parent >= static_cast<int>(nodes.size())) return -1;
        nodes.push_back(ProcNode{name, parent, frame});
        return static_cast<int>(nodes.size()) - 1;
    }

    // A node's frame relative to the model: its own frame under its parent's, up to the root.
    ProcFrame nodeFrame(int node) const {
        ProcFrame f = nodes[static_cast<std::size_t>(node)].frame;
        for (int p = nodes[static_cast<std::size_t>(node)].parent; p >= 0; p = nodes[static_cast<std::size_t>(p)].parent)
            f = nodes[static_cast<std::size_t>(p)].frame * f;
        return f;
    }

    // Fold another model into this one (composition). The other model's root becomes this model's root, so its flat
    // parts land where they did; its other nodes come across with their parts, keeping their tree. Returns false and
    // changes nothing when the two models both have a node of one name; `clash`, when given, is that name.
    bool merge(const ProcModel& o, std::string* clash = nullptr) {
        for (std::size_t i = 1; i < o.nodes.size(); ++i)
            if (findNode(o.nodes[i].name) >= 0) {
                if (clash != nullptr) *clash = o.nodes[i].name;
                return false;
            }
        // The other model's node i is this model's node `base + i`, except its root, which is this root.
        const int base = static_cast<int>(nodes.size()) - 1;
        for (std::size_t i = 1; i < o.nodes.size(); ++i) {
            ProcNode n = o.nodes[i];
            if (n.parent > 0) n.parent += base;
            nodes.push_back(std::move(n));
        }
        const std::size_t firstPart = parts.size();
        parts.insert(parts.end(), o.parts.begin(), o.parts.end());
        for (std::size_t i = firstPart; i < parts.size(); ++i)
            if (parts[i].node > 0) parts[i].node += base;
        instances.insert(instances.end(), o.instances.begin(), o.instances.end());
        colliders.insert(colliders.end(), o.colliders.begin(), o.colliders.end());
        flatten.insert(flatten.end(), o.flatten.begin(), o.flatten.end());
        return true;
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
