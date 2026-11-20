#ifndef RAYTRACER_ENGINE_MODEL_SPAWN_H
#define RAYTRACER_ENGINE_MODEL_SPAWN_H

// A MODEL INTO THE WORLD (#191, step 7): the two things every way of putting a ProcModel in a World does the same,
// kept once. A level's recipe entity (level_loader.cpp, shape:"script") and a stamped prefab (prefab_stamp.h) both
// call these, so a model's materials look the same and its collision shapes stand the same whichever way it came.
//
//  - ModelDresser: what a mesh is made of, for the renderer (a ProcMaterial's numbers and maps on a RenderMaterial,
//    and the mesh's handle).
//  - spawnModelColliders: the model's collision shapes, and its used pieces' per copy, as physics entities.
//
// The surface texture helpers the loader's other entity kinds share with the dresser are here too.

#include "components.h"
#include "procgen/proc_model.h"
#include "world.h"
#include "../renderer/renderer.h"

#include <array>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine {

class AssetManager;

// Rewrite a mesh's UVs (and tangent) to a world-planar tiling frame, so a baked
// tiling texture set repeats at human scale over world-space geometry: v = height
// up a wall, u = horizontal run; horizontal faces lay it in XZ. The tangent is
// set along U so a tangent-space normal map resolves correctly. Matches the
// offline tracer's surfFrame (ADR-0039 Phase B).
void applyWorldPlanarUVs(RenderMesh& mesh, double scale);

// One bake+upload per surface, shared across a level load (every brick entity
// binds the same uploaded set). The cache keys on the surface id.
using SurfaceTexCache = std::unordered_map<int, std::array<TextureHandle, 4>>;
std::array<TextureHandle, 4> bakeSurfaceTextures(Renderer& renderer, RenderMaterial::Surface surf, SurfaceTexCache& cache);
// Bind a baked set onto a material. The surface flag is kept as provenance (so
// it round-trips through save/load); the shader skips the analytic applySurface
// when an albedo map is present, so the textures drive the look, not the flag.
void bindSurfaceMaps(RenderMaterial& mat, const std::array<TextureHandle, 4>& h);
// The surfaces that exist ONLY as baked maps on the mesh's own UVs (the furniture finishes, buildings M4b): the
// shader has no analytic version, so a script part that names one must be bound its baked set.
bool surfaceIsBakedOnMeshUVs(RenderMaterial::Surface s);

// What a mesh is made of, for the renderer: the material's numbers and maps on `out`, and the mesh's handle (a
// textured mesh gets its tiling laid on first). A model's parts and every used piece's parts go through one of
// these, so a part looks the same placed once or many times. A material's maps go to the GPU once per dresser,
// however many meshes are made of it: keep one dresser for everything loaded together.
class ModelDresser {
public:
    ModelDresser(Renderer& renderer, AssetManager& assets) : renderer_(renderer), assets_(assets) {}
    // `meshKey` names the mesh to the asset manager: two asks with one key share one uploaded mesh.
    MeshHandle dress(const ProcMaterial& made, const RenderMesh& mesh, const std::string& meshKey, RenderMaterial& out);
    AssetManager& assets() { return assets_; }

private:
    struct MaterialMaps { TextureHandle albedo{}; TextureHandle normal{}; TextureHandle leaf{}; };
    Renderer& renderer_;
    AssetManager& assets_;
    SurfaceTexCache surfaceTex_;   // the baked sets these parts bind (one bake per surface)
    std::map<const ProcMaterial*, MaterialMaps> uploaded_;
};

// The model's collision shapes as physics entities (ADR-0042): static world geometry, so collision follows the
// generated shapes. Triangle meshes and hulls are merged by friction into one static mesh body each (a city is
// hundreds of solids, and a few bodies keep the physics' broad phase cheap); a box, sphere or capsule is a body of
// its own. Used pieces that collide stamp theirs at every copy. `at`: where the model stands (null: at the origin,
// unturned, as a recipe entity's does). `frictionAll` >= 0 gives every shape that friction in place of its own.
// Returns the entities made.
// `movingAreKinematic`: a box, sphere or capsule on a node that can move (ProcModel::nodeMoves) is a KINEMATIC body,
// one its owner moves, where the others are static. A mesh or hull on such a node stays a static mesh where it is at
// rest, with a warning: the physics has no moving mesh or hull shape yet. `colliderOf`, when given, is filled beside
// the returned entities: for each, the index of the model's shape it stands for, or -1 (a merged mesh body, or a
// used piece's own collider).
std::vector<Entity> spawnModelColliders(World& world, const ProcModel& model, const ProcFrame* at = nullptr, Real frictionAll = -1,
                                        bool movingAreKinematic = false, std::vector<int>* colliderOf = nullptr);

}  // namespace engine

#endif
