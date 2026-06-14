#ifndef RAYTRACER_ENGINE_SCRIPTING_GAMEPLAY_BINDINGS_H
#define RAYTRACER_ENGINE_SCRIPTING_GAMEPLAY_BINDINGS_H

namespace engine {

class ScriptVM;
class World;

// The gameplay (effectful) binding surface (ADR-0024): the API a ScriptBehaviour
// calls. Unlike the procgen surface (pure, sandboxed), this reads and MUTATES
// the World — it is the gameplay layer the ADR-0023 split named. Registered into
// a ScriptVM owned by the ScriptSystem.
//
//   log(msg)
//   entity.alive(e)                 -> bool
//   entity.get_position(e)          -> {x, y, z}
//   entity.set_position(e, {x,y,z})
//   entity.translate(e, {dx,dy,dz})
//   entity.set_yaw(e, radians)      -- orientation about +Y
//
// `e` is the opaque entity id start()/update() receive (a packed Handle).
void openGameplayLibrary(ScriptVM& vm);

// Bind the World that the gameplay functions act on, for the duration of a tick.
// ScriptSystem sets it before driving behaviours and clears it (null) after, so
// a stale World can never be dereferenced between ticks.
void setActiveWorld(ScriptVM& vm, World* world);

}  // namespace engine

#endif
