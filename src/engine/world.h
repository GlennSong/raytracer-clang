#ifndef RAYTRACER_ENGINE_WORLD_H
#define RAYTRACER_ENGINE_WORLD_H

#include "../math.h"
#include "../renderer/renderer.h"
#include <vector>

// Position / orientation / scale stored as components so the simulation can
// integrate them directly and rendering can interpolate between two states.
// Compose into a model matrix with matrix().
struct Transform {
    Vec3 position;
    Vec3 rotation;   // euler radians, applied X then Y then Z
    Vec3 scale;

    Transform() : scale(1, 1, 1) {}

    Mat4 matrix() const;
};

// Component-wise interpolation. Euler rotation is lerped, which is fine for the
// small per-step deltas the fixed timestep produces; swap to quaternions here
// if larger angular steps ever introduce visible wobble.
Transform lerp(const Transform& a, const Transform& b, double t);

// A simulated object: a renderable mesh plus the state the engine integrates
// each fixed step. previousTransform holds the prior step's state so rendering
// can interpolate to the current step for smooth motion at any frame rate.
struct Entity {
    MeshHandle mesh;
    RenderMaterial material;
    Transform transform;
    Transform previousTransform;
    Vec3 velocity;          // world units per second
    Vec3 angularVelocity;   // euler radians per second
    bool simulated;

    Entity()
        : mesh(0), velocity(0, 0, 0), angularVelocity(0, 0, 0),
          simulated(true) {}
};

class World {
public:
    // Returned reference is valid until the next add().
    Entity& add(const Entity& entity);

    std::vector<Entity>& entities() { return entityList; }
    const std::vector<Entity>& entities() const { return entityList; }

    // Advance every simulated entity by one fixed step. Call once per fixed
    // tick. This is the seam future collision / physics systems hook into.
    void step(double dt);

    // Transform to render this frame, interpolated between the entity's
    // previous and current step by alpha in [0, 1].
    Transform renderTransform(const Entity& entity, double alpha) const;

private:
    std::vector<Entity> entityList;
};

#endif
