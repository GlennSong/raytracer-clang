#ifndef RAYTRACER_ENGINE_MODEL_MOTION_H
#define RAYTRACER_ENGINE_MODEL_MOTION_H

// MOTION ON A MODEL INSTANCE (#191, step 8). The model says a crane's boom may tilt 0 to 75 degrees
// (ProcMotion). A ModelInstance (components.h) holds THIS crane's boom at 40. These are the words that change it:
//
//   setModelTarget(world, crane, "boom", radians, speed)   head for a value, at a speed; the limits stop it
//   advanceModelMotion(world, crane, dt)                    one step toward every target; the pieces follow
//   modelMarkerWorld(world, crane, "hook", ...)             where a marker is now, in the world
//
// Values are the engine's: radians for a turn, metres for a slide (degrees are Lua's, converted at the binding).
// A node inside a used piece is named through the node that uses it ("door_fl/window"), as in a ProcPose.
//
// The collision bodies follow through ModelMotionSystem (systems/model_motion_system.h), which needs the physics;
// everything here does not, so a tool or a test without physics can still pose and read an instance.

#include "components.h"
#include "world.h"

#include <string>

namespace engine {

// Head for `value` on a moving node, `speed` a second (<= 0: be there at once). The value is stopped at the node's
// limits. False with `why` when the entity is not a model instance or the model has no such moving node.
bool setModelTarget(World& world, Entity instance, const std::string& node, Real value, Real speed = 0, std::string* why = nullptr);

// A moving node's value now (0 for one never moved, or no such node).
Real modelValue(World& world, Entity instance, const std::string& node);

// One step of `dt` seconds: every node with a target moves toward it. Returns true when anything moved; the
// instance's pieces have then been put where the new pose says (poseModelPieces).
bool advanceModelMotion(World& world, Entity instance, Real dt);

// Put every piece of an instance where its entity's place and its pose say (and its distance point). What the
// stamp calls; call it after moving the entity or changing its pose by hand.
void poseModelPieces(World& world, Entity instance);

// Where one of an instance's collision bodies (an entity with a ModelShape) belongs now: its shape's place on its
// node, in the pose, in the world. False when the entity is not such a body.
bool modelBodyPlace(World& world, Entity body, Vec3& position, Quat& rotation);

// Where a node, or a marker, of an instance is now in the world. False when there is none of that name.
bool modelNodeWorld(World& world, Entity instance, const std::string& node, Vec3& position, Quat& rotation);
bool modelMarkerWorld(World& world, Entity instance, const std::string& marker, Vec3& position, Quat& rotation);

}  // namespace engine

#endif
