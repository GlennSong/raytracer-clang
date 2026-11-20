#ifndef RAYTRACER_ENGINE_PROCGEN_PREFAB_H
#define RAYTRACER_ENGINE_PROCGEN_PREFAB_H

// A PREFAB (#191, step 7): what a thing in the world is made of, as data. It names the model it shows (a name in
// assets/scripts/models.lua) and says what else a copy of it carries. `assets/scripts/prefabs.lua` is the list:
//
//     return {
//       crane = { model = "crane", physics = { body = "static", friction = 0.7 }, draw = { class = "structure" } },
//     }
//
// STAMPING a prefab (engine/prefab_stamp.h) makes one entity of it at a place: a model instance, the entities that
// draw it (the whole model changing level of detail at once) and its collision bodies, read from the model's shapes.
// A new kind of thing is a new line in the list. No C++ changes.
//
// What a line may say today: `model`, `physics`, `draw`. Sounds, lights and offers at markers, a script, and bodies
// that move are the later steps of docs/model-record-design.md; a line that names one is refused, not ignored.

#include "../../rt_math.h"

#include <string>

namespace engine {

struct Prefab {
    std::string name;
    std::string model;                 // a name in models.lua
    // physics = { body = "static" }: the model's collision shapes become bodies that do not move. No `physics`: a
    // copy is not solid.
    enum class Body { None, Static };
    Body body = Body::None;
    Real friction = -1;                // physics.friction: for every shape; < 0: each shape's own
    // draw = { class = "structure", distance = 400 }: how far it is drawn. `drawClass` is a level's draw class by name
    // (engine/components.h DrawClass: structure, scenery, furniture, ...), whose distance the level sets; `distance`
    // is one in metres that wins over it. Neither: furniture.
    std::string drawClass = "furniture";
    Real drawDistance = 0;
};

}  // namespace engine

#endif
