#ifndef RAYTRACER_ENGINE_SCRIPTING_PROCGEN_MESH_H
#define RAYTRACER_ENGINE_SCRIPTING_PROCGEN_MESH_H

#include <memory>

struct lua_State;

namespace engine {

struct RenderMesh;

// Read a procgen Mesh userdatum (the value pushed by mesh.* / polygonize / etc.)
// at stack index `idx`. Returns nullptr if the value isn't a Mesh. Shared across
// the scripting module so the gameplay surface (spawn.model) can accept a mesh a
// gameplay script generated with the procgen builders — both open in one VM.
std::shared_ptr<RenderMesh> luaToMesh(lua_State* L, int idx);

}  // namespace engine

#endif
