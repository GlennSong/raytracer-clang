#ifndef RAYTRACER_ENGINE_ANIM_SKIN_IMPORT_H
#define RAYTRACER_ENGINE_ANIM_SKIN_IMPORT_H

#include "skeleton.h"
#include "skinned_mesh.h"

#include <string>
#include <vector>

namespace engine {
namespace anim {

// A rigged character imported from glTF (character-posing-plan P2, ADR-0073):
// the file's skin becomes an anim::Skeleton (joints reordered parent-before-
// child, the accessor's inverse binds adopted verbatim) and every skinned
// primitive becomes a SkinnedMesh with influences remapped to skeleton
// indices. Blender / Mixamo-converted rigs flow through here; the same
// Skeleton the mannequin fills, filled from a file instead.
struct SkinnedModel {
    Skeleton skeleton;
    std::vector<SkinnedMesh> meshes;

    bool valid() const { return skeleton.jointCount() > 0 && !meshes.empty(); }
};

// Load the FIRST skin of a .gltf/.glb (multi-skin files: one character per
// file is the supported shape for now). Returns an invalid model on any
// failure (missing file, no skin, no skinned primitives), with the reason
// logged. P2 scope, documented in ADR-0073: node transforms above the
// skeleton's root joint and on the mesh node are ignored (exporters bake
// these for character rigs); materials contribute baseColorFactor only.
SkinnedModel loadSkinnedModel(const std::string& path);


}  // namespace anim
}  // namespace engine

#endif
