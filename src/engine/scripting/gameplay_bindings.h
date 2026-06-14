#ifndef RAYTRACER_ENGINE_SCRIPTING_GAMEPLAY_BINDINGS_H
#define RAYTRACER_ENGINE_SCRIPTING_GAMEPLAY_BINDINGS_H

#include "../../rt_math.h"
#include <vector>

namespace engine {

class ScriptVM;
class World;
class InputMap;
struct CameraState;

// A deferred spawn request (ADR-0024). Scripts can't create entities during
// World::each (the no-structural-mutation contract, ADR-0006), so spawn.block
// records this and ScriptSystem applies it after iteration — a tiny command
// buffer. Describes a dynamic physics cube: the Lua port of ShootingSystem.
struct SpawnCommand {
    Vec3 position;
    Vec3 velocity;
    double size = 0.5;
    double restitution = 0.7;
    double friction = 0.3;
    Vec3 color{1, 1, 1};
    Vec3 emission{0, 0, 0};
};

// Per-tick services the gameplay surface reads/writes. Bound for the duration of
// a ScriptSystem tick and cleared after, so nothing stale is reachable between
// ticks. `input`/`camera`/`spawns` may be null (then those bindings error if a
// script calls them) — e.g. a headless test can wire only what it exercises.
struct GameplayContext {
    World* world = nullptr;
    InputMap* input = nullptr;
    const CameraState* camera = nullptr;
    std::vector<SpawnCommand>* spawns = nullptr;
};

// The gameplay (effectful) binding surface (ADR-0024): the API a ScriptBehaviour
// calls. Unlike the procgen sandbox, it reads/mutates the running game. Surface:
//   log(msg)
//   entity.alive(e) / get_position(e) / set_position(e,v) / translate(e,v)
//   entity.set_yaw(e, radians)
//   input.pressed(name) / held(name) / axis(name)        -- bound actions
//   camera.eye() / forward() / right()                   -- the active view
//   spawn.block{position=, velocity=, size=, restitution=, friction=,
//               color=, emission=}                       -- a dynamic physics cube
// Vectors are 3-element Lua arrays; `e` is the packed-Handle id start/update get.
void openGameplayLibrary(ScriptVM& vm);

// Bind the per-tick context the gameplay functions act on. ScriptSystem sets it
// before driving behaviours and clears it (null) after.
void setGameplayContext(ScriptVM& vm, GameplayContext* ctx);

}  // namespace engine

#endif
