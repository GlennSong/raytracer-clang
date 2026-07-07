#ifndef RAYTRACER_ENGINE_COMPONENTS_H
#define RAYTRACER_ENGINE_COMPONENTS_H

#include "../rt_math.h"
#include "../renderer/renderer.h"
#include "ai/driver_agent.h"   // DriverCommand / DriverTuning (AgentDriver)
#include "physics/physics_world.h"
#include "procgen/city/polygon.h"   // Poly2 (city-plan debug outlines, ADR-0066)
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
    // Road conforming (ADR-0044): the non-road cut/fill set (city/script grading) the
    // level loaded with. The editor's "Conform terrain to roads" action rebuilds
    // `params.flatten` = baseFlatten + fresh road footprints, so re-conforming never
    // double-applies or strands a moved road's old grading.
    std::vector<TerrainFlatten> baseFlatten;
    // Bumped whenever `params` changes at runtime (a re-conform). TerrainLodSystem
    // watches it and rebuilds its tile cache + collider window when it changes.
    uint32_t revision = 0;
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
    // Reference into the level's named "materials" table (ADR-0039). When set,
    // the entity shares that material asset; LevelWriter emits "material": <name>
    // and the table, instead of an inline material block. Empty = inline material.
    std::string materialName;
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
    // Continuous collision (linear sweep) — for fast movers (the player on a long
    // fall, a bullet), so they don't tunnel through thin colliders. Capsule and
    // box bodies.
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

// A physics-driven car (ADR-0059). VehicleSystem creates the Jolt vehicle from
// `config` + the entity's Transform, drives it from the seated driver's input,
// and writes the chassis Transform back each fixed step. The entity also carries
// a Renderable (body mesh) + Transform/PrevTransform so RenderSystem draws it;
// `wheelEntities` are optional child Renderables whose transforms VehicleSystem
// refreshes from the wheels. UNVERIFIED submodule-gated path (needs Jolt).
struct Vehicle {
    PhysicsWorld::VehicleConfig config;
    PhysicsWorld::VehicleId vehicleId = PhysicsWorld::INVALID_VEHICLE;
    Entity driver;                 // invalid = unoccupied; set on enter, cleared on exit
    // Live driver input, written by VehicleSystem each step (for inspection/debug).
    Real throttle = 0, steer = 0, brake = 0, handBrake = 0;
    std::vector<Entity> wheelEntities;   // rendered wheels (optional)
    // Head/taillight glow toggle (ADR-0059) and the lens entities VehicleSystem
    // positions + lights each frame; `driverModel` is a capsule shown in the seat
    // while occupied.
    bool lightsOn = false;
    std::vector<Entity> headlights;      // front lenses (warm)
    std::vector<Entity> taillights;      // rear lenses (red)
    Entity driverModel;                  // driver capsule (stowed when unoccupied)
};

// Marks a player entity currently seated in a vehicle (ADR-0059). PlayerSystem
// suppresses on-foot character movement while this is present; the enter/exit
// logic in VehicleSystem adds it on entry and removes it on exit. `vehicle` is
// the car entity being driven.
struct InVehicle {
    Entity vehicle;
};

// Marks a Vehicle as driven by an AI brain rather than the player (ADR-0062): the
// SAME physics Vehicle, but its {throttle, steer, brake} come from
// computeDriverInput(command) instead of host input, so an NPC car and the
// player's car share one physics path. A brain (e.g. the CitySim bridge) writes
// `command` each step; VehicleSystem consumes it. `agentId` links back to that
// brain (e.g. a CitySim agent index) so the player can EJECT the agent on entry
// (remove this component) and the brain can free the agent. When the player takes
// the wheel this component is removed; on exit it may be restored.
struct AgentDriver {
    DriverCommand command;      // heading + speed the brain currently wants
    DriverTuning tuning;        // controller gains
    int agentId = -1;           // brain handle (CitySim agent index); -1 = none
};

// Level-authored city-simulation settings (ADR-0063): a top-level "citysim"
// block in the level JSON becomes one entity carrying this, and the city render
// bridge reads it at build. Lets a level choose its own population — the agent
// lab runs ONE driver and ONE walker with the debug HUD on, while grown.json
// keeps the default bustle. Defaults mirror CityRenderParams so an absent field
// changes nothing.
// A level-authored place (Living City, ADR-0066): a labelled destination in the
// citysim world. Its TYPE is a citysim PlaceType TAG kept as a string here — the
// citysim bridge parses it (parsePlaceType) — so engine core stays free of
// citysim types. `x`/`z` are the site (world XZ); the bridge snaps the entrance
// onto the nearest sidewalk. Hours are in-world 0..24 (default = always open).
// The city PLAN as data (Living City, ADR-0066): the block interiors the lot
// pass subdivided and every lot it produced. One entity carries this when a
// citysim level grew lot buildings; the citysim render bridge draws the
// polygons as ground-outline debug layers ("show me the blocks and the lots").
// Plain data — absent on levels that grew nothing.
struct CityPlanDebug {
    std::vector<Poly2> blocks;   // buildable block interiors (inset from the roads)
    std::vector<Poly2> lots;     // every parcelled lot inside them
    // Physics-collider outlines (device: "maybe we need a physics hull
    // visualizer"): every building's plan-prism collider footprint with its
    // world base/top, so the debug layer can draw the exact volumes Jolt sees.
    struct Prism {
        Poly2 plan;      // world-XZ footprint (the collider's side walls)
        Real y0 = 0, y1 = 0;   // world base/top of the extrusion
    };
    std::vector<Prism> prisms;
};

struct AuthoredPlace {
    std::string type;      // "home" | "shop" | "office" | "park" | "civic"
    float x = 0, z = 0;    // building site (world XZ)
    std::string name;      // optional label (may be empty)
    float openHour = 0, closeHour = 24;
    // Optional building (Living City Phase 4): when width/depth/height are all > 0
    // the loader spawns a static box STRUCTURE of this footprint at the site, so
    // the place IS a building you can walk up to (its door snaps to the sidewalk).
    // All-zero = a bare marker (e.g. a park). `buildingColor` tints the structure.
    float buildingW = 0, buildingH = 0, buildingD = 0;   // full extents (m)
    Vec3 buildingColor{0.72f, 0.70f, 0.64f};
};

struct CitySimConfig {
    int cars = 40;
    int pedestrians = 40;
    uint32_t seed = 1;
    float hoursPerSecond = 0.05f;        // sim-clock hours per real second
    float perceptionReliability = 0.97f; // <1 -> agents occasionally err
    bool debugWidgets = false;           // start with the agent-state HUD on
    bool wander = false;                 // agents take perpetual random trips
                                         // (no schedule) — the lab car keeps lapping
    // Scripted goal tables (ADR-0064): the loaded TEXT of the level's
    // `"agents"` script (an agents.lua-style file resolved by level_loader).
    // The citysim render bridge runs it at build (scripting builds only) and
    // installs its archetype tables over the built-ins. "" = built-ins.
    std::string agentScript;
    // Data-driven fleet bodies (ADR-0065): the loaded TEXT of the level's
    // `"vehicles"` script (a vehicles.lua-style file resolved by level_loader).
    // The citysim render bridge runs it at build (scripting builds only) and
    // builds each instanced fleet mesh from its `vehicle.fleet` recipes. Any
    // failure falls back to the C++ fleetCarMesh; "" = built-in fleet meshes.
    std::string vehicleScript;
    // Level-authored places (ADR-0066): labelled destinations (home/shop/office/
    // park/civic) the citysim bridge turns into a routable PlaceMap at build.
    // Empty for levels that don't author any (the generator emits them later).
    std::vector<AuthoredPlace> places;
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
