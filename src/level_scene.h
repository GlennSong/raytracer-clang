#ifndef RAYTRACER_LEVEL_SCENE_H
#define RAYTRACER_LEVEL_SCENE_H

#include "scene.h"
#include "lens_params.h"
#include "engine/procgen/proc_model.h"
#include <string>

namespace engine {

// Imports a level JSON (the viewer's format, assets/levels/*.json) into the
// offline path tracer's Scene so placed cameras can be rendered offline —
// the "virtual filming" output (docs/virtual-camera-plan.md). Shapes are
// tessellated via MeshBuilder and baked into world-space triangles (spheres
// without rotation/scale stay analytic). Materials keep their full PBR
// parameters (albedo/metallic/roughness + checkerboard flag) and are shaded
// with the viewer's GGX model; the level's sun/point/spot lights become
// explicitly sampled SceneLights, paralleling the realtime light pass. glTF
// models are skipped with a warning.
//
// If outHdrPath is given and the level binds an HDR environment, the resolved
// path is returned so the caller can decode it (stb lives in the importer TU,
// which tests don't link) and install it as the scene's environment map.
struct LevelScene {
    static bool load(const std::string& levelPath, Scene& scene,
                     std::string* outHdrPath = nullptr);
};

// Bake a composable procedural model (ADR-0042) into the path-tracer Scene: each
// part becomes world-space triangles, each instance group a shared BLAS proto
// placed by the TLAS (ADR-0041), with the alpha-cut leaf material for foliage.
// Renderer-agnostic — the same ProcModel a Lua recipe returns feeds this offline
// path and the viewer's InstanceGroup spawn. `offset` places the whole model in
// world space (an entity's `position`), matching the realtime loader so offline
// and viewer agree — e.g. a planet sat in front of the camera rather than at the
// origin.
void bakeProcModel(const ProcModel& model, Scene& scene, const Vec3& offset = Vec3());

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
