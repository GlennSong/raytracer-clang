#ifndef RAYTRACER_ENGINE_COMPONENTS_H
#define RAYTRACER_ENGINE_COMPONENTS_H

#include "../rt_math.h"
#include "../renderer/renderer.h"
#include "physics/physics_world.h"
#include <string>

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
    std::string shape = "box";   // MeshBuilder shape; empty when meshFile is set
    Vec3 size{1, 1, 1};
    std::string meshFile;        // glTF path, level-relative ("mesh" in JSON)
    bool hasPhysics = false;
    std::string motion = "static";
    Real friction = 0.5;
    Real restitution = 0.0;
    bool lockRotation = false;
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
};


}  // namespace engine

#endif
