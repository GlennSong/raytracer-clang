#ifndef RAYTRACER_ENGINE_PROCGEN_PROC_MODEL_H
#define RAYTRACER_ENGINE_PROCGEN_PROC_MODEL_H

#include "../../renderer/renderer.h"   // RenderMesh
#include "../../rt_math.h"             // Mat4
#include "terrain.h"                   // TerrainFlatten
#include "tree.h"                      // TextureData
#include <memory>
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
    // The built-in leaf card (tree.h leafTexture), alpha-cut: what `alpha_foliage` on placed copies asks for.
    bool alphaFoliage = false;
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

// A named point on a node where something can hook on later (#191, model step 2): a seat, a lamp, a hook, an exhaust.
// The same idea as an empty in Blender. It does nothing by itself: a sound or a light or a script names it and asks
// the model where it is. Its +z is the way it faces.
struct ProcMarker {
    std::string name;             // unique in the model, across every node
    std::string role = "point";   // what kind of thing hooks on: "seat", "lamp", "hook"...; "point" when none is said
    ProcFrame frame;              // relative to the node it is on
    Real size = 0;                // metres; 0 when the recipe gave none
};

// One named piece of a model (#191, model step 1): a car's door is a node under the car, the handle a node under the
// door. A node holds no geometry itself; a part says which node it hangs on, and is drawn at that node's frame.
struct ProcNode {
    std::string name;      // unique in the model
    int parent = -1;       // an index into ProcModel::nodes, always lower than this node's own; -1 for the root only
    ProcFrame frame;       // relative to the parent
    std::vector<ProcMarker> markers;
};

// One material of a model, in the model's table (#191, model step 2). A part says which one it is made of, so forty
// parts in one paint hold one material, and "paint" can be found by its name. `name` is empty for a material a
// recipe used without naming it. The material itself is shared and never changed once made.
struct ProcModelMaterial {
    std::string name;
    std::shared_ptr<const ProcMaterial> material;
};

// A model part: geometry + which of the model's materials it is made of (0 = the plain one: untextured,
// vertex-coloured). The mesh is in the space of the node it hangs on; node 0 is the root, which is the model's own
// space, so a part that names no node is where its vertices say it is.
struct ProcPart {
    RenderMesh mesh;
    int material = 0;      // an index into ProcModel::materials
    int node = 0;          // an index into ProcModel::nodes
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

// How a used piece collides: each copy stamps a collider. `None` leaves the copies non-solid; `Mesh` bakes the
// piece's triangles at every copy into the shared static mesh body; the primitive kinds place one Collider per copy
// (a lamp post is exactly a thin capsule).
enum class InstanceCollision { None, Mesh, Box, Sphere, Capsule };

struct ProcModel;

// Where one copy of a used piece goes: a position, a rotation and one scale for all three axes.
struct ProcPlacement {
    Vec3 position{0, 0, 0};
    Quat rotation;
    Real scale = 1;

    Mat4 matrix() const { return Mat4::trs(position, rotation, Vec3(scale, scale, scale)); }
};

// A USE (#191, model step 3): another model placed inside this one, once or many times, and stored once. A wheel
// used four times, a bolt forty, a lamp at every verge. The piece is a whole model, so it brings its own parts,
// materials, nodes, markers and its own uses; the copies are drawn as instances of the piece's meshes. This is what
// the instance group (ADR-0041/0042) grew into: that was a bare mesh with a roughness, placed in the root's space.
struct ProcUse {
    std::shared_ptr<const ProcModel> piece;
    int node = 0;                             // the node whose space the placements are in
    std::vector<ProcPlacement> placements;    // one per copy
    // Optional collision per copy (ADR-0042). `collisionProto` overrides the piece's own triangles for Mesh
    // collision (a cheaper-but-faithful collision surface); empty means collide with the piece itself.
    InstanceCollision collision = InstanceCollision::None;
    RenderMesh collisionProto;
    Real colliderRadius = 0.5, colliderHalfHeight = 0.5;
    Vec3 colliderHalfExtent{0.5, 0.5, 0.5};
    Real colliderFriction = 0.6;
};

struct ProcModel {
    // The model's pieces as a tree. There is always a root, node 0, whose frame is the model's own space and never
    // changes; a model that adds no node is the flat model every recipe built before nodes existed. Colliders and
    // flatten footprints are in the root's space.
    std::vector<ProcNode> nodes = {ProcNode{"root", -1, ProcFrame{}, {}}};
    // The model's materials. There is always entry 0, the plain material (untextured, vertex-coloured, no name),
    // which a part is made of unless it says otherwise.
    std::vector<ProcModelMaterial> materials = {ProcModelMaterial{"", std::make_shared<const ProcMaterial>()}};
    std::vector<ProcPart> parts;
    std::vector<ProcUse> uses;                // other models placed in this one (see ProcUse)
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
        nodes.push_back(ProcNode{name, parent, frame, {}});
        return static_cast<int>(nodes.size()) - 1;
    }

    // A node's frame relative to the model: its own frame under its parent's, up to the root.
    ProcFrame nodeFrame(int node) const {
        ProcFrame f = nodes[static_cast<std::size_t>(node)].frame;
        for (int p = nodes[static_cast<std::size_t>(node)].parent; p >= 0; p = nodes[static_cast<std::size_t>(p)].parent)
            f = nodes[static_cast<std::size_t>(p)].frame * f;
        return f;
    }

    // --- materials -------------------------------------------------------------------------------------------
    // The material with this name, or -1. (The plain material and unnamed ones have no name to find.)
    int findMaterial(const std::string& name) const {
        if (name.empty()) return -1;
        for (std::size_t i = 0; i < materials.size(); ++i)
            if (materials[i].name == name) return static_cast<int>(i);
        return -1;
    }

    // Put a material in the model's table and return its index. The same material given twice is one entry. With a
    // name: the entry is found by that name from then on; returns -1 (and adds nothing) when the name already
    // belongs to a different material. Without a name: an entry holding this material, new if there is none.
    int addMaterial(const std::string& name, const std::shared_ptr<const ProcMaterial>& material) {
        if (material == nullptr) return -1;
        const int named = findMaterial(name);
        if (named >= 0) return materials[static_cast<std::size_t>(named)].material == material ? named : -1;
        for (std::size_t i = 1; i < materials.size(); ++i) {
            if (materials[i].material != material) continue;
            if (name.empty()) return static_cast<int>(i);
            if (materials[i].name.empty()) { materials[i].name = name; return static_cast<int>(i); }   // now it has a name
        }
        materials.push_back(ProcModelMaterial{name, material});
        return static_cast<int>(materials.size()) - 1;
    }

    // What a part is made of.
    const ProcMaterial& materialOf(const ProcPart& part) const { return *materials[static_cast<std::size_t>(part.material)].material; }

    // --- markers ---------------------------------------------------------------------------------------------
    // The marker with this name, wherever in the tree it is, or null.
    const ProcMarker* findMarker(const std::string& name, int* node = nullptr) const {
        for (std::size_t n = 0; n < nodes.size(); ++n)
            for (const ProcMarker& mk : nodes[n].markers)
                if (mk.name == name) {
                    if (node != nullptr) *node = static_cast<int>(n);
                    return &mk;
                }
        return nullptr;
    }

    // Put a marker on a node. Returns false (and adds nothing) when the name is empty or taken, or the node is not
    // one of this model's.
    bool addMarker(int node, const ProcMarker& marker) {
        if (marker.name.empty() || findMarker(marker.name) != nullptr) return false;
        if (node < 0 || node >= static_cast<int>(nodes.size())) return false;
        nodes[static_cast<std::size_t>(node)].markers.push_back(marker);
        return true;
    }

    // Where a marker is in the model: its frame under its node's. A marker inside a used piece is reached through
    // the node that uses it, "wheel_fl/valve": the piece is stored once and its names are its own, so the node's
    // name says which copy (the first, when a node places many). A copy's scale moves the marker with it. False
    // when the model has no such marker.
    bool markerFrame(const std::string& path, ProcFrame& out) const;

    // --- uses ------------------------------------------------------------------------------------------------
    // Does this model hold `other` as a piece, at any depth (or is it `other`)? A model may not use a piece that
    // reaches back to it: there would be no end to drawing it.
    bool reaches(const ProcModel* other) const {
        if (this == other) return true;
        for (const ProcUse& u : uses)
            if (u.piece != nullptr && u.piece->reaches(other)) return true;
        return false;
    }

    // A piece that is one mesh in one material: what a use of a bare mesh is.
    static std::shared_ptr<ProcModel> ofMesh(RenderMesh mesh, const std::shared_ptr<const ProcMaterial>& material = nullptr) {
        std::shared_ptr<ProcModel> piece = std::make_shared<ProcModel>();
        ProcPart part;
        part.mesh = std::move(mesh);
        if (material != nullptr) part.material = piece->addMaterial("", material);
        piece->parts.push_back(std::move(part));
        return piece;
    }

    int markerCount() const {
        int n = 0;
        for (const ProcNode& node : nodes) n += static_cast<int>(node.markers.size());
        return n;
    }

    // Fold another model into this one (composition). The other model's root becomes this model's root, so its flat
    // parts land where they did; its other nodes come across with their parts and markers, keeping their tree, and
    // its materials join this model's table (one entry where both hold the same material). Returns false and changes
    // nothing when the two models both have a node or a marker of one name, or give one name to two different
    // materials, or when `o` holds this model as a piece; `clash`, when given, says which ("node 'door'").
    bool merge(const ProcModel& o, std::string* clash = nullptr) {
        auto refuse = [&](const char* kind, const std::string& name) {
            if (clash != nullptr) *clash = std::string(kind) + " '" + name + "'";
            return false;
        };
        if (o.reaches(this)) return refuse("model", "itself");
        for (std::size_t i = 1; i < o.nodes.size(); ++i)
            if (findNode(o.nodes[i].name) >= 0) return refuse("node", o.nodes[i].name);
        for (const ProcNode& n : o.nodes)
            for (const ProcMarker& mk : n.markers)
                if (findMarker(mk.name) != nullptr) return refuse("marker", mk.name);
        for (const ProcModelMaterial& mm : o.materials) {
            const int mine = findMaterial(mm.name);
            if (mine >= 0 && materials[static_cast<std::size_t>(mine)].material != mm.material) return refuse("material", mm.name);
        }
        // The other model's node i is this model's node `base + i`, except its root, which is this root.
        const int base = static_cast<int>(nodes.size()) - 1;
        nodes[0].markers.insert(nodes[0].markers.end(), o.nodes[0].markers.begin(), o.nodes[0].markers.end());
        for (std::size_t i = 1; i < o.nodes.size(); ++i) {
            ProcNode n = o.nodes[i];
            if (n.parent > 0) n.parent += base;
            nodes.push_back(std::move(n));
        }
        // The other model's material i is this model's `remap[i]`; its plain material is this plain material.
        std::vector<int> remap(o.materials.size(), 0);
        for (std::size_t i = 1; i < o.materials.size(); ++i) remap[i] = addMaterial(o.materials[i].name, o.materials[i].material);
        const std::size_t firstPart = parts.size();
        parts.insert(parts.end(), o.parts.begin(), o.parts.end());
        for (std::size_t i = firstPart; i < parts.size(); ++i) {
            if (parts[i].node > 0) parts[i].node += base;
            parts[i].material = remap[static_cast<std::size_t>(parts[i].material)];
        }
        const std::size_t firstUse = uses.size();
        uses.insert(uses.end(), o.uses.begin(), o.uses.end());
        for (std::size_t i = firstUse; i < uses.size(); ++i)
            if (uses[i].node > 0) uses[i].node += base;
        colliders.insert(colliders.end(), o.colliders.begin(), o.colliders.end());
        flatten.insert(flatten.end(), o.flatten.begin(), o.flatten.end());
        return true;
    }

    // How many copies of used pieces the model draws, at every depth (four wheels of five bolts each are twenty-four).
    int instanceCount() const;

    int colliderCount() const { return static_cast<int>(colliders.size()); }
};

// A use as it stands in the model that was asked: every copy's transform in that model's space, through the node it
// is on and, for a use inside a piece, through every copy of that piece.
struct ProcPlacedUse {
    const ProcUse* use = nullptr;
    std::vector<Mat4> transforms;
};

// Every use of a model at every depth, placed. THE one walk of the uses: the offline bake, the level loader,
// flatten and the model sheet all read this, so they cannot disagree about where a copy is. A use on the root of the
// model asked has exactly its placements' matrices.
std::vector<ProcPlacedUse> placedUses(const ProcModel& m);

// One instanced draw of a model: one part of a used piece, and where every copy of it stands in the model. A piece
// used from four nodes is ONE of these with four transforms (per part of the piece), which is the point of a use.
struct ProcDrawnPiece {
    const ProcModel* piece = nullptr;
    const ProcPart* part = nullptr;
    std::vector<Mat4> transforms;
};

// Every instanced draw of a model, over `placedUses`: what the offline bake, the level loader, flatten and the model
// sheet draw. In the order pieces are first met; parts with no triangles are left out.
std::vector<ProcDrawnPiece> drawnPieces(const ProcModel& m);

// One mesh of everything the model draws, in the model's space: its parts at their nodes and every copy of every
// used piece. Bridges a model to single-mesh consumers.
RenderMesh flattenModel(const ProcModel& m);

// THE MIRRORED TWIN of a model, left to right (across the plane x = 0 through its origin): a right door from the
// left door, authored once. Every mesh is mirrored and wound to face out again (MeshBuilder::mirrored), and every
// node, marker and copy is put where the mirror puts it, so the twin is an ordinary model that any renderer draws
// as it draws the first. (Mirroring by a copy's matrix would not do: the renderers cull by a fixed winding, and a
// mirroring matrix turns every face inside out.) A piece the model uses is mirrored once however often it is used.
// Names are kept.
//   - A frame keeps its up (y) and its facing (z), mirrored; its own x swaps sides, as it must for the frame to stay
//     a rotation. So a marker that faced out of the left door faces out of the right one.
//   - Left to right only. For another plane, hang the twin on a turned node.
// Returns null, with `why` when given, for a model that grades terrain: that has no mirror here.
std::shared_ptr<ProcModel> mirroredModel(const ProcModel& m, std::string* why = nullptr);

}  // namespace engine

#endif
