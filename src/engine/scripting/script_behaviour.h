#ifndef RAYTRACER_ENGINE_SCRIPTING_SCRIPT_BEHAVIOUR_H
#define RAYTRACER_ENGINE_SCRIPTING_SCRIPT_BEHAVIOUR_H

#include <string>

namespace engine {

// The MonoBehaviour analog (ADR-0024): a script attached to an entity. The
// `source` is a Lua chunk that returns a behaviour table, e.g.
//
//   local M = {}
//   function M:start(e)        self.t = 0 end
//   function M:update(e, dt)   self.t = self.t + dt
//                              entity.set_position(e, {0, self.t, 0}) end
//   return M
//
// The returned table IS the per-entity instance (its fields are the behaviour's
// state, like MonoBehaviour members) — the engine's System lifecycle already has
// the hooks (onStart/update/...), so a Lua behaviour just mirrors them. Lives in
// the scripting module (not components.h) so the core ECS stays scripting-
// agnostic and the whole layer gates on RT_ENABLE_SCRIPTING.
struct ScriptBehaviour {
    std::string source;

    // Runtime state, filled by ScriptSystem (kept here so it travels with the
    // component, but they are plain ints — no Lua type leaks into this header):
    // a Lua registry ref to the loaded instance table, and whether start() ran.
    // -1 means "not yet loaded" (a valid luaL_ref is always >= 0).
    int instanceRef = -1;
    bool started = false;
    bool failed = false;   // a load/start error was reported; don't retry forever
};

}  // namespace engine

#endif
