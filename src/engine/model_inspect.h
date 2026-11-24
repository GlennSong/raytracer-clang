#ifndef RAYTRACER_ENGINE_MODEL_INSPECT_H
#define RAYTRACER_ENGINE_MODEL_INSPECT_H

// A STAMPED MODEL, DESCRIBED (#191): the model instances in a world and, for one of them, its tree as data: every
// node under its parent with where it was placed and how it moves, and what hangs on each (parts, used pieces,
// markers, collision shapes), each with the recipe line that made it. ONE walk, read by everything that shows a
// model's make-up: the editor's model studio draws its tree from it, and the control channel's `model?` hands it
// to a tool (an assistant looking at the same model as the person).

#include "components.h"
#include "world.h"

#include <string>
#include <vector>

namespace engine {

// Every model instance in the world, in the order the world holds them.
std::vector<Entity> modelInstances(World& world);

// One line per instance: `<n> <prefab> at x y z, <pieces> pieces, <bodies> bodies; ...` ("none" when there are none).
std::string modelsListing(World& world);

// The tree of one instance as JSON: an array of rows, a node first and then what hangs on it, then its child nodes.
//   { "depth": 0, "kind": "node" | "part" | "use" | "marker" | "shape", "name": ..., "detail": words,
//     "line": recipe line (0: not known), "piece": for a part, which of the instance's drawn pieces it is (-1: none),
//     and for a node that moves: "joint": its name in a pose, "turn": true (degrees) | false (metres),
//     "lo", "hi" (equal: no limits), "value": where it is now }
// "[]" when the entity is not a model instance.
std::string modelTreeJson(World& world, Entity instance);

}  // namespace engine

#endif
