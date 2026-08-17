#ifndef RAYTRACER_ENGINE_SCRIPTING_GAMEPLAY_BINDINGS_H
#define RAYTRACER_ENGINE_SCRIPTING_GAMEPLAY_BINDINGS_H

#include "../../rt_math.h"
#include <memory>
#include <string>
#include <vector>

namespace engine {

class ScriptVM;
class World;
class InputMap;
class EventBus;
struct CameraState;
struct RenderMesh;

// A deferred spawn request (ADR-0024). Scripts can't create entities during
// World::each (the no-structural-mutation contract, ADR-0006), so spawn.* records
// this and ScriptSystem applies it after iteration — a tiny command buffer.
//   Block — a dynamic physics cube (the Lua port of ShootingSystem's bullet):
//           Transform/Collider/RigidBody/Velocity.
//   Model — a procgen-generated mesh entity (e.g. the gun viewmodel): a
//           Renderable, plus an optional ScriptBehaviour so it can drive itself
//           (the gun model follows the camera via its own follow script).
enum class SpawnKind { Block, Model };

struct SpawnCommand {
    SpawnKind kind = SpawnKind::Block;
    Vec3 position;
    Vec3 velocity;
    double size = 0.5;
    double restitution = 0.7;
    double friction = 0.3;
    Vec3 color{1, 1, 1};
    Vec3 emission{0, 0, 0};

    // Model only:
    std::shared_ptr<RenderMesh> mesh;   // the generated mesh to render
    double metallic = 0.0;
    double roughness = 0.5;
    std::string script;                 // optional behaviour attached to it
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
    // Sound cues (ADR-0069/0071): sound.play enqueues a PlaySound on the bus,
    // delivered at the frame's queued dispatch — deferred, like spawns.
    EventBus* events = nullptr;
    // camera.first_person(): is the view the player's own eyes? False in the
    // on-foot shoulder rig and in a vehicle's chase camera — the gun script
    // stows its viewmodel and holds fire on that. Defaults true so headless
    // ticks (tests) behave as before.
    bool firstPersonView = true;
};

// The gameplay (effectful) binding surface (ADR-0024): the API a ScriptBehaviour
// calls. Unlike the procgen sandbox, it reads/mutates the running game. Surface:
//   log(msg)
//   entity.alive(e) / get_position(e) / set_position(e,v) / translate(e,v)
//   entity.set_yaw(e, radians) / look_along(e, fwd[, up]) / snap_prev(e)
//   input.pressed(name) / held(name) / axis(name)        -- bound actions
//   camera.eye() / forward() / right()                   -- the active view
//   camera.first_person()                                -- eyes, not a rig?
//   spawn.block{position=, velocity=, size=, restitution=, friction=,
//               color=, emission=}                       -- a dynamic physics cube
//   spawn.model{mesh=, position=, color=, metallic=, roughness=, script=}
//                                                        -- a procgen mesh entity
//   sound.play{clip=, volume=, pitch=, position=, range=, music=}
//     -- fire a sound cue (PlaySound on the EventBus). `clip` is a file path
//        or a name registered via AudioSystem::registerClip (procedural PCM).
//        A `position` makes it spatial; without one it plays flat (UI/gun).
// `mesh` is a value built with the procgen builders (also open in this VM, so a
// gameplay script can generate geometry). Vectors are 3-element Lua arrays; `e`
// is the packed-Handle id start/update get.
void openGameplayLibrary(ScriptVM& vm);

// Bind the per-tick context the gameplay functions act on. ScriptSystem sets it
// before driving behaviours and clears it (null) after.
void setGameplayContext(ScriptVM& vm, GameplayContext* ctx);

}  // namespace engine

#endif
