#ifndef RAYTRACER_ENGINE_COMPONENTS_H
#define RAYTRACER_ENGINE_COMPONENTS_H

#include "../rt_math.h"
#include "../renderer/renderer.h"
#include "ai/driver_agent.h"   // DriverCommand / DriverTuning (AgentDriver)
#include "audio/audio_engine.h"
#include "physics/physics_world.h"
#include "procgen/city/polygon.h"   // Poly2 (city-plan debug outlines, ADR-0066)
#include "procgen/city/road_network.h"   // RoadGraph (ExtraNavGraph, plan §8 P8.4)
#include "procgen/city/road_mesh.h"      // RoadDeckField (RoadDeck)
#include "procgen/terrain.h"
#include "world.h"
#include <cstdint>
#include <string>
#include <memory>
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

// Scene-wide physics gravity (a singleton the level loader adds from a top-level
// "gravity" field). PhysicsSystem applies it to the Jolt world each step, so a
// space level can set [0,0,0] and the player floats instead of falling. Absent =
// the engine default (Earth). Vec3 is axis·(m/s²).
struct SceneGravity {
    Vec3 value{0.0, -9.81, 0.0};
};

// Opt-in flag (a singleton the level loader adds from a top-level "planetLab":true)
// that turns on the PlanetLabSystem's floating editor window for THAT scene only —
// so the panel doesn't appear over every level. Absent = no lab panel.
struct ScenePlanetLab {};

// Debug render layers (device: "layers for roads, buildings, simulation ... so
// we can turn them on or off"). A bit set on Renderable::renderLayer /
// InstanceGroup::renderLayer; RenderSystem skips the draw when the same bit is
// set in Renderer::hiddenLayers. 0 = untagged (terrain, sky, props) — always
// drawn, so hiding roads/buildings/sim reveals the ground underneath.
enum RenderLayer : uint32_t {
    LayerRoads     = 1u << 0,
    LayerBuildings = 1u << 1,
    LayerSim       = 1u << 2,
    // Trees and scattered greenery. Untagged until now, which put them in the
    // layer-0 residual along with terrain and ground paint — and that residual
    // turned out to be ~3.9 M of piedmont's 8.2 M triangles, larger than the
    // building share, with no way to find out what it was.
    LayerFoliage   = 1u << 3,
};

// WHAT a drawable is, so that DrawPolicy can decide HOW FAR it draws.
//
// Draw distance used to be a bare number set at each creation site, defaulting
// to 0 = "draw to the far plane". That default made forgetting silent, and of
// every drawable in the engine exactly two remembered — vegetation and street
// lamps. Cars, pedestrians, signal lenses and posts, car lamps, road markings
// and a 7964-instance parking-bay paint group all drew to the horizon, and the
// bay group additionally carried a city-wide bounding sphere so even the coarse
// group reject could never fire.
//
// So the class is the thing a site states, and the distance is derived. A site
// that says nothing lands in Unset, which the loader COUNTS AND REPORTS — the
// point of this enum is that the next forgotten site is loud instead of free.
// Unlimited is still available, but it must now be asked for by name.
enum class DrawClass : uint8_t {
    Unset = 0,     // nobody said — reported at load, treated as Unlimited
    Terrain,       // the ground itself; CDLOD does its own selection
    Structure,     // buildings and large static world geometry
    Scenery,       // trees, rocks, props — the bulk of the triangles
    Furniture,     // lamps, signals, signs, benches
    GroundPaint,   // road markings, bay paint — flat, short range, no shadow
    SimBody,       // cars and pedestrians; follows the sim's tier radii
    Effect,        // glows, signal lenses, lamp halos
    Unlimited,     // explicit opt-out: the only way to say "draw forever"
    Count
};

// The single owner of draw-distance policy: class -> metres (0 = unlimited).
// One per level (a singleton component the loader stamps), so distances scale
// with the world — 250 m of scenery is right for an 8 km city and absurd for a
// 100 m arena. Absent entirely = every class unlimited, i.e. exactly the old
// behaviour, which is what keeps levels that have not opted in unchanged.
// (A per-class shadow opt-out belongs here too — flat ground paint is invisible
// in a shadow map and is currently cast into all three cascades — but there is
// no per-drawable caster flag to hang it on yet. That arrives with the shadow
// pass's own culling; the field is deliberately absent until something reads it.)
struct DrawPolicy {
    Real distance[static_cast<int>(DrawClass::Count)] = {};

    Real distanceFor(DrawClass c) const {
        return distance[static_cast<int>(c)];
    }
};

// What to draw for an entity.
struct Renderable {
    MeshHandle mesh;   // null until assigned an uploaded mesh (ADR-0007)
    RenderMaterial material;
    uint32_t renderLayer = 0;   // debug layer bits (0 = always visible)
    // Distance policy (HLOD, metropolis-scale-plan P1.2). drawDistance > 0:
    // cull when the camera is farther than this from the mesh bounds — the
    // full-detail city chunks. minDistance > 0: cull when NEARER than this —
    // the chunk's mass-box HLOD proxy, which takes over exactly where the
    // detail chunk fades. 0 = defer to DrawPolicy via drawClass below.
    Real drawDistance = 0;
    Real minDistance = 0;
    // Content class. An explicit drawDistance above still wins (the HLOD chunks
    // compute theirs); this is what everything else resolves through.
    DrawClass drawClass = DrawClass::Unset;
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
    // Per-instance draw distance (0 = defer to DrawPolicy via drawClass below):
    // instances farther than this from the camera are not drawn. Distant
    // L-system trees/rocks dominate the triangle budget, so a finite radius is
    // the cheapest large fps win in a big world.
    Real drawDistance = 0;
    uint32_t renderLayer = 0;   // debug layer bits (0 = always visible)
    DrawClass drawClass = DrawClass::Unset;   // see DrawClass above
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

// The ears of the scene (ADR-0069): AudioSystem drives the AudioEngine
// listener from this entity's Transform (position + forward). At most one per
// world; with none, the listener follows the render camera, so audio "just
// works" before any entity opts in.
struct AudioListener {};

// A sound attached to an entity (ADR-0069). AudioSystem loads `clip` through
// its path cache, starts it (immediately when `autoplay`, or when `trigger` is
// set), and — for spatial sources — feeds the entity's Transform position to
// the voice every frame. One-shot voices clear `voice` when they finish;
// destroying the entity stops its voice.
struct AudioSource {
    std::string clip;          // audio file path (wav/flac/mp3)
    float volume = 1.0f;
    float pitch = 1.0f;
    Real range = 25.0;         // audible radius (world units, spatial only)
    bool loop = false;
    bool spatial = true;
    bool autoplay = true;      // start as soon as the system sees the source
    bool trigger = false;      // set true to (re)start manually; system clears
    AudioBus bus = AudioBus::Sfx;

    AudioClipHandle clipHandle;   // filled by AudioSystem (cache)
    AudioVoiceHandle voice;       // live voice while playing
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
    bool lightsOn = false;               // manual switch: forces headlights ON
    std::vector<Entity> headlights;      // front lenses (warm)
    std::vector<Entity> taillights;      // rear lenses (red)
    Entity driverModel;                  // driver capsule (stowed when unoccupied)

    // Lamp state, so the player's car behaves like the city's traffic rather than
    // having only a manual headlight toggle: headlights come on at dusk, brake
    // lights on deceleration, indicators while turning. Decided by
    // engine::vehicleLampState (vehicle_lamps.h) — the same predicate citysim
    // uses — from these per-step inputs.
    Real speed = 0;                      // chassis speed, m/s
    Real prevSpeed = 0;                  // last step's, so a hard decel reads as braking
    int turnSignal = 0;                  // -1 left, +1 right, 0 none
    Real signalHold = 0;                 // seconds an indicator stays on after the
                                         // steering straightens, so it doesn't flicker
    // Lamp lenses placed from the body recipe's markers. Empty falls back to the
    // hardcoded chassis corners, so a car with no marker data still lights.
    struct Lamp {
        Entity entity;
        Vec3 local{0, 0, 0};
        bool front = true;               // headlight end vs tail end
        bool left = false;               // which indicator side it belongs to
    };
    std::vector<Lamp> lamps;

    // Hip point for the seated occupant, from the body recipe. Without it the
    // driver falls back to a guess off the chassis half-extents.
    Vec3 driverSeat{0, 0, 0};
    bool hasDriverSeat = false;

    // Extra body meshes rigidly fixed to the chassis, each its own Renderable so
    // it can carry its own material — the reason `mesh.car` returns separate
    // parts. Transparent glass and the matte interior live here; they cannot
    // share the body mesh because a Renderable holds ONE material and glass needs
    // opacity < 1. Posed at chassis-local origin (0,0,0) every step, exactly like
    // the lamp lenses.
    std::vector<Entity> bodyParts;
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

// Night-gated emission (WS3, "street lights, building window lights"): a
// DayNightSystem pass scales this entity's Renderable (or InstanceGroup)
// material emission from black at noon to `fullEmission` at night, on the
// SAME dusk ramp the headlights and street-lamp point lights use — every
// light in the city agrees on when evening starts. Runtime-only (the loader
// tags lamp glow shells and lit-window chunks; never saved).
struct NightGlow {
    Vec3 fullEmission{1.0, 0.85, 0.55};
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
        std::string district;  // the block's district ("financial", ...) — the city map's tint
        std::string type;      // "home" | "shop" | "office" | "civic" | ...
    };
    std::vector<Prism> prisms;
};

// The road mesher's curb/sidewalk BAND outlines for one road entity — its own
// CurbBandAudit, kept so the city map (RT_CITY_SVG, `citymap`) draws the
// sidewalks that were BUILT: closed loops around every asphalt union, the band
// riding each loop's right normal outward by sidewalkWidth; mouthGaps where
// the band is suppressed. A few thousand points per city.
struct RoadBandDebug {
    std::vector<Poly2> loops;
    std::vector<std::pair<Vec2, Vec2>> mouthGaps;
    Real sidewalkWidth = 3.5;
};

// THE CITY MAP's data (procgen/city/city_svg.h), assembled by the loader after
// the streets, lots, furniture and scatter exist, so the map can be written
// at any time from the running viewer (`citymap <path> [layers]`).
struct CityMapData;
struct SidewalkCrossing;
struct CityMap {
    std::shared_ptr<CityMapData> data;
    // The sidewalk-on-asphalt census (city_svg.h), computed at load so the
    // Teleport panel can list the places with Go buttons.
    std::shared_ptr<const std::vector<SidewalkCrossing>> conflicts;
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

// Level-authored day/night cycle policy (device: "the scene loads bright but
// as soon as the level starts everything gets dark" — DayNightSystem was
// overwriting the level's sun + ambient every frame with its own mid-morning
// default, the same silent-override class as the bloom-settings stomp). A
// level that authors a static sun sets enabled=false; one that wants the
// cycle seeds its start time/speed. Absent block = cycle runs as before.
struct DayNightConfig {
    bool enabled = true;
    float timeOfDay = -1.0f;   // [0,1) start; <0 = keep the cycle's default
    // Loop length in REAL MINUTES (the authored knob since the 30-minute
    // day); <0 = keep the default. `speed` (days per second) is the legacy
    // spelling, honoured only when dayMinutes is absent.
    float dayMinutes = -1.0f;
    float speed = -1.0f;
    float latitude = -1000.0f; // degrees north; < -90 = keep the default
    int dayOfYear = -1;        // 1..365 (season); < 1 = keep the default
    float newMoonDay = -1.0f;  // day of year of a new moon (the month); < 0 = default
    float lightPollution = -1.0f;   // downtown sky glow 0..1; < 0 = default (0.7)
    float pollutionFalloff = -1.0f; // metres past the city edge to a dark sky; < 0 = 1500
};

struct CitySimConfig {
    // Population: explicit counts, or -1 = BY DENSITY (roads-v2.1 4c) — the
    // citysim bridge computes counts from the built nav graph's lane-km /
    // sidewalk-km, so a metropolis is busy and a hamlet is quiet from the
    // same recipe. Flat counts remain as overrides.
    int cars = 40;
    int pedestrians = 40;
    float carsPerLaneKm = 10.0f;   // density mode: ambient cars per lane-km
    float pedsPerKm = 6.0f;        // density mode: walkers per sidewalk-km
    uint32_t seed = 1;
    float hoursPerSecond = 0.05f;        // sim-clock hours per real second
    // The in-world hour the level OPENS at. Agents are placed straight from
    // their schedules at this hour, so any start costs the same — a level can
    // open at 03:00 as cheaply as at midday.
    float startHour = 10.5f;
    float perceptionReliability = 0.97f; // <1 -> agents occasionally err
    // Draw radius for PARKED scenery cars (m). City-wide instance groups can't
    // be partially frustum-culled, so this is what keeps thousands of parked
    // bodies out of the colour and shadow passes. 0 = draw them all.
    float sceneryRadius = 450.0f;
    // How many pedestrians may hold a real Jolt capsule at once. The rest are
    // drawn and animated from the sim ghost. Bounds an O(bodies^2) separation
    // pass that was otherwise a function of how crowded downtown happened to be.
    int maxWalkerBodies = 200;
    // Sim tick rate for LOCAL agents (Hz). 0 = every fixed step (historical).
    // 30 halves the traffic sim's cost; poses extrapolate between ticks.
    float localHz = 0.0f;
    bool adaptiveRate = true;   // dip further while the clock is behind
    bool debugWidgets = false;           // start with the agent-state HUD on
    bool tieredAgents = false;           // P4: opt into V/K traffic tiering
    // P5: opt into the DORMANT tier below V — far agents stop being simulated
    // entirely and are rebuilt from their schedule when the player returns.
    // Requires tieredAgents (dormancy is a demotion from V).
    bool dormantAgents = false;
    bool showPlan = false;               // boot with block/lot outlines on
                                         // (plan-only demarcation levels)
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
    // "" = this level draws no cars (vehicles are optional content).
    std::string vehicleScript;
    // Level-authored places (ADR-0066): labelled destinations (home/shop/office/
    // park/civic) the citysim bridge turns into a routable PlaceMap at build.
    // Empty for levels that don't author any (the generator emits them later).
    std::vector<AuthoredPlace> places;
};

// Build-time street furniture (device: "place the stop lights when we build
// the city ... the simulation should use it but it shouldn't be responsible
// for where they are"). The loader plans and SPAWNS every signal pole and
// street lamp from the roads' deterministic nav graph; this component tells
// the sim where they stand. The sim animates lenses / reacts to phases only.
// THE level's road network (§10): streets and corridor chains welded into
// ONE derived graph at load. Every consumer — the citysim nav build, street
// furniture, editor queries — reads this; nothing merges graphs privately.
// (Supersedes the P8.4 ExtraNavGraph bolt-on, which only the sim could see.)
struct LevelRoadGraph {
    RoadGraph graph;
};

// The DECK a road entity's mesh actually rode (RoadDeckField), stored by the
// loader beside the Renderable. Everything placed ON a road — sim cars, parked
// cars, bay paint, signal lenses and poles — samples this, not the terrain
// the mesher carved 0.22 m under it.
struct RoadDeck {
    RoadDeckField field;
};

struct StreetFurniture {
    struct Signal {
        Vec3 base;      // pole foot (on the road deck)
        Vec3 face;      // head faces this way (toward its approaching traffic)
        int link = -1;  // NavGraph link the signal governs
    };
    std::vector<Signal> signalPoles;   // ("signals" is a Qt macro — editor includes this)
    std::vector<Vec3> lampHeads;  // street-lamp bulb positions (night lights)
    int navLinkCount = 0;         // consumer sanity check: same nav build?
    Entity postGroup{};           // the loader's signal-post InstanceGroup
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
