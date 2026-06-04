#ifndef RAYTRACER_ENGINE_COMPONENTS_H
#define RAYTRACER_ENGINE_COMPONENTS_H

#include "../math.h"
#include "../renderer/renderer.h"

// Position / orientation / scale. Compose into a model matrix with matrix().
struct Transform {
    Vec3 position;
    Vec3 rotation;   // euler radians, applied X then Y then Z
    Vec3 scale;

    Transform() : scale(1, 1, 1) {}

    Mat4 matrix() const;
};

// Component-wise interpolation. Euler rotation is lerped, which is fine for the
// small per-step deltas the fixed timestep produces; swap to quaternions if
// larger angular steps ever wobble.
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
    MeshHandle mesh = 0;
    RenderMaterial material;
};

// Associates an entity with a local player slot (ADR-0010). This is the only
// bridge the engine provides between players and entities: the game tags
// whatever entity it wants and reads the player's input via that index. The
// engine does not define any other notion of "player".
struct ControlledBy {
    int playerIndex = 0;
};

#endif
