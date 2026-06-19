#ifndef RAYTRACER_ENGINE_COMPONENTS_H
#define RAYTRACER_ENGINE_COMPONENTS_H

#include "../rt_math.h"
#include "../renderer/renderer.h"
#include "physics/physics_world.h"
#include "procgen/terrain.h"
#include "world.h"
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

// Position / orientation / scale. Compose into a model matrix with matrix().
struct Transform {
    Vec3 position;
    Quat orientation;
    Vec3 scale;

    Transform() : scale(1, 1, 1) {}

    Mat4 matrix() const;
};

// Interpolated for smooth rendering between fixed steps: position/scale linearly,
// orientation via slerp (no Euler wobble — see ADR-0006).
Transform lerp(const Transform& a, const Transform& b, Real t);

// Decompose a model matrix (assumed M = T*R*S) back into a Transform: the
// translation is the last column, scale the rotation columns' lengths, the
// orientation the normalized rotation part. Used to flatten parented
// transforms and to read manipulated gizmo matrices back.
Transform transformFromMatrix(const Mat4& m);

// Previous step's transform, kept so rendering can interpolate to the current
// step. Present on every renderable; only moving entities have it updated.
struct PrevTransform {
    Transform value;
};

// Linear + angular velocity. An entity's presence in this pool means it is
// simulated (integrated each fixed step).
struct Velocity {
    Vec3 linear;
    Vec3 angular;
};

// What to draw for an entity.
struct Renderable {
    MeshHandle mesh;   // null until assigned an uploaded mesh (ADR-0007)
    RenderMaterial material;
};

// Many instances of one mesh, drawn as a batch (ROADMAP Phase B instancing).
// Replaces "thousands of Renderable entities" for static scatter (the forest):
// one InstanceGroup carries the shared mesh/material and a baked world matrix per
// instance, so RenderSystem iterates a handful of groups instead of every plant,
// culls by the group's bounds, and issues one drawMeshInstanced. The instances
// are static (no per-instance Transform/interpolation). Per-instance culling /
// chunking for huge worlds is the Tier 5 follow-up.
struct InstanceGroup {
    MeshHandle mesh;
    RenderMaterial material;
    std::vector<Mat4> transforms;   // world matrices, one per instance
    Vec3 boundsCenter;              // world-space group bounds (coarse cull)
    Real boundsRadius = 0;
    // Per-instance draw distance (0 = unlimited): instances farther than this from
    // the camera are not drawn. Distant L-system trees/rocks dominate the triangle
    // budget, so a finite radius is the cheapest large fps win in a big world.
    Real drawDistance = 0;
};

// CDLOD heightfield terrain (ADR-0036, open-world Phase 1c). One per level: when a
// terrain block opts in via "cdlod", the loader stamps this instead of static chunk
// entities, and TerrainLodSystem reads it each frame to select a quadtree of nodes
// by camera distance, cache/upload their meshes, frustum-cull, and draw them with
// vertex morphing. Carries the same height field (TerrainParams + seed) as the
// static terrain so the surface is identical.
struct TerrainLodConfig {
    TerrainParams params;          // shared height field
    uint32_t seed = 0;
    float worldHalf = 1024.0f;     // half-extent of the square world (XZ, origin-centred)
    int   numLods = 6;             // quadtree depth (level 0 = finest)
    int   gridRes = 32;            // grid cells per node (forced even)
    float rangeFactor = 2.5f;      // LOD range = leafNodeSize * this (>= 2)
    RenderMaterial material;
    // Static collider window: leaf-level nodes within this distance of the player
    // get a triangle-mesh collider so the player walks on the surface. 0 = derive
    // (~1.5 leaf nodes). The window follows the player; far colliders are freed.
    float colliderRadius = 0.0f;
};

// Associates an entity with a local player slot (ADR-0010). This is the only
// bridge the engine provides between players and entities: the game tags
// whatever entity it wants and reads the player's input via that index. The
// engine does not define any other notion of "player".
struct ControlledBy {
    int playerIndex = 0;
};

// Authoring provenance for level-document entities (docs/edit-mode-plan.md).
// The level loader and the editor's Add menu fill it; LevelWriter serializes
// entities carrying it back to the level JSON. Runtime-spawned entities
// (bullets, gizmos) lack it and are never saved — by construction.
struct SourceSpec {
    // Stable document id: survives save/load so parenting (and future
    // cross-references) can name an entity independently of its runtime
    // Handle, which is reminted every load. 0 = unassigned (the loader and
    // editor fill these in). `parentId` is another entity's id, 0 = root.
    uint32_t id = 0;
    uint32_t parentId = 0;
    std::string name;            // optional display name ("name" in JSON)
    std::string shape = "box";   // MeshBuilder shape; empty when meshFile is set
    Vec3 size{1, 1, 1};
    std::string meshFile;        // glTF path, level-relative ("mesh" in JSON)
    // A procedural recipe entity (e.g. shape == "tree"): the raw JSON of the
    // generator's parameter block (the "tree" object), kept verbatim so the
    // entity round-trips through save/load. Empty for primitives and meshes.
    // Without this the generated entity carries no document state and the
    // LevelWriter drops it on save (silent data loss).
    std::string recipe;
    bool hasPhysics = false;
    std::string motion = "static";
    Real friction = 0.5;
    Real restitution = 0.0;
    bool lockRotation = false;

    // A group/null object: no mesh of its own, just a named transform other
    // entities parent under. Shape is empty AND no glTF mesh is set.
    bool isGroup() const { return shape.empty() && meshFile.empty(); }
};

// Where the player starts (editor-app plan): in the editor the spawn is a
// real, pickable, gizmo-movable entity; LevelWriter syncs its Transform back
// into the level's "player" block, which the game loader consumes unchanged.
struct PlayerSpawn {};

enum class ColliderShape { Box, Sphere, Capsule };
struct Collider {
    ColliderShape shape = ColliderShape::Box;
    Vec3 halfExtent{0.5, 0.5, 0.5};   // Box
    Real radius = 0.5;                // Sphere, Capsule
    Real halfHeight = 0.5;            // Capsule (half of the cylinder segment)
    Real restitution = 0.0;
    Real friction = 0.2;
};

// Marks an entity as simulated by the PhysicsSystem (ADR-0012). The body is
// created from the entity's Transform + Collider; bodyId is filled in then.
// PhysicsSystem owns the Transform of these entities — MotionSystem yields to
// it — so an entity should not carry both a RigidBody and a script-driven
// Velocity for the same motion.
struct RigidBody {
    BodyMotion motion = BodyMotion::Dynamic;
    PhysicsBodyId bodyId = INVALID_PHYSICS_BODY;
    bool lockRotation = false;
    // Continuous collision (linear sweep) — for fast movers like the player, so a
    // long fall doesn't tunnel through thin terrain colliders. Capsule bodies only.
    bool continuousCollision = false;
};

// A kinematic capsule character controller (ADR-0012). Unlike a dynamic
// RigidBody capsule, it collide-and-slides and steps up ledges up to stepHeight
// (curbs, sidewalks, stairs, low cubes) instead of being stopped by them. The
// PhysicsSystem creates the controller from this + the entity's Transform;
// PlayerSystem drives it. radius/halfHeight describe the same capsule a Capsule
// Collider would, and `position` (the Transform) is the capsule centre.
struct CharacterController {
    Real radius = 0.3;
    Real halfHeight = 0.4;
    Real stepHeight = 0.4;   // tallest ledge it can walk up in one step
    CharacterId characterId = INVALID_CHARACTER;
};

// A static triangle-mesh collider (terrain, and later any baked static geometry)
// the PhysicsSystem turns into one static Jolt mesh body. Separate from Collider
// because it owns geometry (CPU triangles in world space), not just dimensions.
// bodyId is filled when the body is created, so it is made exactly once.
struct MeshCollider {
    std::vector<Vec3> vertices;
    std::vector<uint32_t> indices;
    Real friction = 0.6;
    PhysicsBodyId bodyId = INVALID_PHYSICS_BODY;
};

// --- Document hierarchy (stable ids + parenting) --------------------------
// Document entities (those carrying a SourceSpec) reference one another by
// stable id, not by runtime Handle. These helpers index and compose that
// graph; the editor owns it, and PLAY flattens it into world transforms at
// load (so the runtime never walks a hierarchy — see level_loader).

// The largest SourceSpec.id in the world (0 if none): nextDocumentId is +1.
uint32_t maxDocumentId(World& world);
uint32_t nextDocumentId(World& world);

// Give every SourceSpec with id == 0 a fresh unique id (e.g. a hand-authored
// level with no ids yet). Stable across a single load: assigned in iteration
// order from maxDocumentId+1.
void assignMissingDocumentIds(World& world);

// The document entity with this id, or an invalid Entity. id 0 never matches.
Entity findByDocumentId(World& world, uint32_t id);

// Local-to-world matrix for `e`, composing its local Transform with its
// ancestors' (SourceSpec.parentId chain). Unparented entities return their
// own Transform::matrix(). Cycles and missing parents terminate the walk.
Mat4 worldMatrix(World& world, Entity e);

// True if making `child` a child of `parent` would form a cycle (parent is
// child itself, or a descendant of child). Reparenting guards with this.
bool wouldCreateCycle(World& world, Entity child, Entity parent);


}  // namespace engine

#endif
