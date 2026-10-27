#ifndef RAYTRACER_ENGINE_SCRIPTING_GAMEPLAY_BINDINGS_H
#define RAYTRACER_ENGINE_SCRIPTING_GAMEPLAY_BINDINGS_H

#include "../../rt_math.h"
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace engine {

class ScriptVM;
class World;
class InputMap;
class EventBus;
class ControlCommands;
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
    // Block only: where the shot LEAVES from (the eye). With a physics world
    // bound, a block whose spawn point lies behind a wall from here is
    // swallowed — the muzzle sits half a metre in front of the eye, which at
    // a window is already outside (Glenn's walk, 2026-09-14: "I can shoot
    // the gun through the window").
    bool hasOrigin = false;
    Vec3 origin;

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
    // camera.hands_free(): are the player's hands free to hold something? False while they grip a ledge, a wall
    // or a ladder, or swim (engine/player_pose.h) -- the gun stows and holds fire then. Defaults true.
    bool handsFree = true;
    // The control channel's verbs (ADR-0253). control.run(line) runs any verb on `commands`;
    // control.command{...} hands its `run` function (a registry ref) to `addCommand`, which registers it owned by
    // the calling script and answers "" or why it was refused. Null / empty = control.* errors.
    ControlCommands* commands = nullptr;
    std::function<std::string(const std::string& name, const std::string& usage, const std::string& summary,
                              int fnRef)> addCommand;
    // Named events (ADR-0253): events.on hands its function (a registry ref) to `subscribe`, owned by the calling
    // script; it answers the subscription id, or 0 with `error` set. events.off(id) goes to `unsubscribe`.
    // events.publish needs only `events`. Empty = events.on/off error.
    std::function<uint64_t(const std::string& name, int fnRef, std::string* error)> subscribe;
    std::function<bool(uint64_t id)> unsubscribe;
};

// The gameplay (effectful) binding surface (ADR-0024): the API a ScriptBehaviour
// calls. Unlike the procgen sandbox, it reads/mutates the running game. Surface:
//   log(msg)
//   entity.alive(e) / get_position(e) / set_position(e,v) / translate(e,v)
//   entity.set_yaw(e, radians) / look_along(e, fwd[, up]) / snap_prev(e)
//   input.pressed(name) / held(name) / axis(name)        -- bound actions
//   camera.eye() / forward() / right()                   -- the active view
//   camera.first_person()                                -- eyes, not a rig?
//   camera.hands_free()                                  -- not climbing, pulling up, swimming?
//   spawn.block{position=, velocity=, size=, restitution=, friction=,
//               color=, emission=}                       -- a dynamic physics cube
//   spawn.model{mesh=, position=, color=, metallic=, roughness=, script=}
//                                                        -- a procgen mesh entity
//   sound.play{clip=, volume=, pitch=, position=, range=, music=}
//     -- fire a sound cue (PlaySound on the EventBus). `clip` is a file path
//        or a name registered via AudioSystem::registerClip (procedural PCM).
//        A `position` makes it spatial; without one it plays flat (UI/gun).
//   control.command{name=, usage=, summary=, run=function(args) ... end}
//     -- add a verb to the control channel (socket, MCP, `commands?`), owned by
//        this script: it goes when the script's entity does. `run` gets the
//        words after the verb and returns the reply ("ok ..."/"err ..."; any
//        other text is sent as "ok <text>", nil as "ok"). True, or nil + why.
//   control.run(line) -- run any verb now, as the socket would; its reply
//   events.publish(name, fields) -- a named event (events? lists them), its
//        fields a table; missing ones keep their defaults. Delivered at the
//        frame's queued dispatch, like sound.play. True, or nil + why.
//   events.on(name, function(fields) ... end) -- every such event, as a
//        table, until events.off(id) or the script's entity goes; the id
// `mesh` is a value built with the procgen builders (also open in this VM, so a
// gameplay script can generate geometry). Vectors are 3-element Lua arrays; `e`
// is the packed-Handle id start/update get.
void openGameplayLibrary(ScriptVM& vm);

// Bind the per-tick context the gameplay functions act on. ScriptSystem sets it
// before driving behaviours and clears it (null) after.
void setGameplayContext(ScriptVM& vm, GameplayContext* ctx);
// The context bound now (null between ticks), so a call that runs Lua from outside a tick can tell whether it is
// nested inside one.
GameplayContext* gameplayContext(ScriptVM& vm);

}  // namespace engine

#endif
