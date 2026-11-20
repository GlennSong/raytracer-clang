#ifndef RAYTRACER_ENGINE_PREFAB_STAMP_H
#define RAYTRACER_ENGINE_PREFAB_STAMP_H

// STAMPING A PREFAB (#191, step 7): one entity of a prefab (procgen/prefab.h) at a place in the world.
//
// The entity gets a ModelInstance (components.h): the model, from the model store, shared with every other copy,
// and this copy's own pose. Around it the stamp makes:
//
//  - the PIECES that draw it, one entity per part of the model and per part of each copy of a used piece
//    (ProcModelDraw). Each is drawn in its band of distances, and every one measures its distance to the model's
//    own point, so the whole model changes level of detail at once.
//  - the BODIES it collides with, from the model's collision shapes (model_spawn.h), when the prefab is solid.
//
// The pieces and bodies are the instance's: `unstampPrefab` takes them with it.
//
// A copy's moving nodes are moved through engine/model_motion.h (step 8): the pieces follow at once, the bodies of
// shapes on moving nodes (kinematic) through ModelMotionSystem.
//
// Not here yet (docs/model-record-design.md): sounds, lights and offers at markers (step 9), many copies drawn as a
// group (step 10).

#include "components.h"
#include "procgen/model_store.h"
#include "procgen/prefab.h"
#include "world.h"

#include <functional>
#include <string>

namespace engine {

class ModelDresser;

// One entity of `prefab` at `at` (its position and orientation; a scale is not carried). `dresser` gives the pieces
// their meshes and materials; null makes the pieces without anything to draw (a headless test, a server). Returns
// an invalid entity with `error` when the prefab's model cannot be had. `store`, `models`: for a test with its own.
Entity stampPrefab(World& world, ModelDresser* dresser, const Prefab& prefab, const Transform& at, const std::string& levelDir = std::string(),
                   std::string* error = nullptr, ModelStore* store = nullptr, const std::string& models = "models.lua");

// The same, the prefab found by its name in prefabs.lua.
Entity stampPrefab(World& world, ModelDresser* dresser, const std::string& name, const Transform& at, const std::string& levelDir = std::string(),
                   std::string* error = nullptr);

// Destroy an instance with its pieces and bodies. An entity's physics body does not go with the entity (its owner
// takes it out, as everywhere in the engine): `removeBody` is called with each body the physics made for this
// instance, for the caller to hand to its PhysicsWorld. (Not a PhysicsWorld here: tools without physics stamp too.)
void unstampPrefab(World& world, Entity instance, const std::function<void(PhysicsBodyId)>& removeBody = {});

}  // namespace engine

#endif
