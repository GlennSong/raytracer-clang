#ifndef RAYTRACER_LEVEL_SCENE_H
#define RAYTRACER_LEVEL_SCENE_H

#include "scene.h"
#include "lens_params.h"
#include <string>

namespace engine {

// Imports a level JSON (the viewer's format, assets/levels/*.json) into the
// offline path tracer's Scene so placed cameras can be rendered offline —
// the "virtual filming" output (docs/virtual-camera-plan.md). Shapes are
// tessellated via MeshBuilder and baked into world-space triangles (spheres
// without rotation/scale stay analytic); PBR materials map onto the tracer's
// diffuse/metal/emissive set. glTF models are skipped with a warning. Levels
// rarely carry emissive geometry, so the scene's EnvironmentLight is enabled:
// sky tint from the level's environment, plus the level's sun (or a default
// noon sun when none is specified).
struct LevelScene {
    static bool load(const std::string& levelPath, Scene& scene);
};

// A camera read from the level's sidecar (CameraStore format,
// <level>.cameras.json) for offline rendering. Pose is position + forward —
// placed cameras are roll-free.
struct SidecarCamera {
    std::string name;
    Vec3 position;
    Vec3 forward;
    LensParams lens;
};

// Empty name selects the first camera. Returns false when the sidecar or the
// named camera is missing.
bool loadSidecarCamera(const std::string& levelPath, const std::string& name,
                       SidecarCamera& out);

}  // namespace engine

#endif
