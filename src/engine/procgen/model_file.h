#ifndef RAYTRACER_ENGINE_PROCGEN_MODEL_FILE_H
#define RAYTRACER_ENGINE_PROCGEN_MODEL_FILE_H

// A MODEL AS A glTF FILE, AND BACK (#191, model step 6).
//
// A model made outside the engine (in Blender) says what its pieces are by their NAMES; AGENTS.md "Model file names"
// is the convention. A model made by a recipe is written out by the same convention, so it opens in Blender as the
// tree it is and reads back as the model it was.
//
//   an object                          a NODE of that name; the file's parenting is the model's tree
//     ...its mesh                      the node's PARTS, one glTF primitive per material
//     ...custom property `motion`      how the node may move: { kind = "turn" | "slide", axis, min, max }
//   an object named `root`, alone      the model's root (written always; a file without one has its top objects
//                                      hang on a root of the model's own)
//   an object named `<node>_lod<N>`    the parts of level of detail N of that node (the node's own mesh is level 0)
//   an object with no mesh, no child   a MARKER of that name (custom properties `role`, `size`); `node = true`
//                                      makes it a node all the same
//   an object named `col_<kind>_<n>`   a COLLISION SHAPE named n, not drawn: kind box / sphere / capsule fit to the
//                                      object's mesh, hull round its points, mesh its triangles
//   an object with property `use`      one COPY of a used piece: every object with the same `use` number is a copy
//                                      of the piece the first one holds, placed where the object is
//   the root's property `levels`       the distance each level of detail is drawn from
//   a material's properties            `tile`, `surface`, `alpha_foliage`, and `emission` when it is brighter than a
//                                      glTF material may say (1)
//
// Positions are written as single-precision floats (glTF's), so a model read back agrees with the one written to
// about a micron, not bit for bit.

#include "proc_model.h"

#include <memory>
#include <string>
#include <vector>

namespace engine {

// Write `m` as a binary glTF file. False with `error` for what a file cannot say yet: a model that grades terrain,
// or a use with a collision mesh of its own.
bool writeModelGlb(const ProcModel& m, const std::string& path, std::string* error = nullptr);

// Read a model from one glTF file. Null with `error` when the file cannot be read or says two nodes of one name.
// `notes`, when given, takes what was put right while reading and is worth a recipe writer's eye (an object scaled
// in the file, a level with no distance).
std::shared_ptr<ProcModel> readModelGlb(const std::string& path, std::string* error = nullptr, std::vector<std::string>* notes = nullptr);

// Read a model whose levels of detail are separate files, the full one first: the first file gives the nodes, the
// markers and the collision shapes; each later file's meshes are matched to those nodes by name and become that
// level's parts. A mesh on a node the first file lacks hangs on the root, and is noted. `distances`, when it has one
// entry per level (the first 0), says where each level starts; without it the files' own `levels` are used.
std::shared_ptr<ProcModel> loadModelFiles(const std::vector<std::string>& paths, const std::vector<Real>& distances, std::string* error = nullptr,
                                          std::vector<std::string>* notes = nullptr);

}  // namespace engine

#endif
