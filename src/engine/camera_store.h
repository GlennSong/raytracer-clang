#ifndef RAYTRACER_ENGINE_CAMERA_STORE_H
#define RAYTRACER_ENGINE_CAMERA_STORE_H

#include "camera/scene_camera.h"
#include <string>

namespace engine {

// Persists placed SceneCamera entities to a JSON sidecar next to the level
// (e.g. arena.json -> arena.json.cameras.json), so interactively placed
// cameras survive a restart (docs/virtual-camera-plan.md, Phase 2). The
// orientation round-trips as a forward vector — placed cameras are roll-free
// by construction, so a quaternion would persist nothing extra.
struct CameraStore {
    // Appends one entity per stored camera (Transform + PrevTransform +
    // SceneCamera; gizmos are the caller's concern — they need a renderer).
    // Returns false if the file is missing or unreadable (not an error: the
    // level simply has no saved cameras yet).
    static bool load(const std::string& path, World& world);

    // Writes all cameras in the world. Returns false if the file can't be
    // written. An empty world writes an empty list (deleting the last camera
    // and quitting must not resurrect it from a stale file).
    static bool save(const std::string& path, World& world);
};

}  // namespace engine

#endif
