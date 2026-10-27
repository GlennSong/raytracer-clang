#ifndef RAYTRACER_ENGINE_SCRIPTING_SCRIPT_SYSTEM_H
#define RAYTRACER_ENGINE_SCRIPTING_SCRIPT_SYSTEM_H

#include "../system.h"
#include "script_vm.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

struct lua_State;

namespace engine {

class World;
class InputMap;
class AssetManager;
class ControlCommands;
struct CameraState;
struct ControlCommand;
struct GameplayContext;
struct SpawnCommand;

// Drives MonoBehaviour-style scripts (ADR-0024): each frame it finds entities
// with a ScriptBehaviour, lazily loads each one's chunk into a per-entity
// instance table, calls start() once, then update(e, dt) every frame — the same
// lifecycle the engine's C++ Systems have, exposed to Lua. Owns one gameplay
// ScriptVM (effectful bindings; not the procgen sandbox).
//
// Gameplay scripting is intentionally NOT in the deterministic sandbox: scripts
// read/mutate the World. It runs in the variable-rate update() (frame logic),
// not fixedUpdate() — physics stays the deterministic fixed-step authority
// (ADR-0002/0012). Entity spawns requested from scripts are deferred and applied
// after iteration (the World::each no-structural-mutation contract, ADR-0006).
class ScriptSystem : public System {
public:
    ScriptSystem();
    ~ScriptSystem() override;
    ScriptSystem(const ScriptSystem&) = delete;
    ScriptSystem& operator=(const ScriptSystem&) = delete;

    void update(FrameContext& ctx) override;

    // Services the gameplay surface reads each tick. update() fills these from
    // FrameContext; tests set only what they exercise. `assets` may be null —
    // then spawned entities get physics components but no Renderable (headless).
    // `events` may be null — then sound.play errors if a script calls it.
    void setServices(InputMap* input, const CameraState* camera,
                     AssetManager* assets, EventBus* events = nullptr);
    // The muzzle check for spawn.block's `origin`: `blocked(from, to)` says
    // whether a wall stands between the eye and the spawn point — a shot
    // born behind a pane is swallowed. The host binds the physics world's
    // ray cast (the engine core links no physics); unset = no check.
    using BlockedFn = std::function<bool(const Vec3& from, const Vec3& to)>;
    void setMuzzleCheck(BlockedFn blocked) { blocked_ = std::move(blocked); }

    // View mode for camera.first_person(): true when the view is the player's
    // own eyes (on foot, not the V shoulder rig, not a vehicle chase camera).
    // update() derives it each frame from Settings + InVehicle; tests and
    // headless ticks keep the default (true) unless they set it.
    void setFirstPersonView(bool firstPerson) { firstPersonView_ = firstPerson; }
    void setHandsFree(bool free) { handsFree_ = free; }
    // The control channel's table (ADR-0253): scripts add verbs to it with control.command and run any with
    // control.run. update() takes it from FrameContext; null = control.* errors. A script's verbs (and its
    // events.on subscriptions) go when its entity (or its ScriptBehaviour) does, and all of them when this
    // system does, so the table and the bus must outlive it.
    void setCommands(ControlCommands* commands) { commands_ = commands; }

    // Headless-testable core: run start/update for every ScriptBehaviour in
    // `world`, advancing by `dt`, then apply any deferred spawns. update()
    // forwards to this (like PhysicsSystem).
    void tick(World& world, double dt);

private:
    // A verb a script added: its owner, and the registry ref to its `run` function.
    struct ScriptVerb {
        Entity owner;
        std::string name;
        int fnRef;
    };
    // An events.on subscription: its owner, its bus id, and the registry ref to its function.
    struct ScriptSubscription {
        Entity owner;
        uint64_t id;
        int fnRef;
    };
    GameplayContext context(World* world, std::vector<SpawnCommand>* spawns);
    std::string addVerb(Entity owner, const std::string& name, const std::string& usage,
                        const std::string& summary, int fnRef);
    uint64_t addSubscription(Entity owner, const std::string& name, int fnRef, std::string* error);
    bool removeSubscription(uint64_t id);
    // Call a script function from outside its hooks (a verb from the socket, an event from the bus). `push`
    // pushes its one argument; `result` gets what it returned as text (nil = ""). False with `error` set on a
    // Lua error.
    bool callScript(int fnRef, const std::function<void(lua_State*)>& push, std::string* result, std::string* error);
    std::string runVerb(int fnRef, const ControlCommand& cmd);
    void dropScript(Entity owner);
    void applySpawns(World& world, const std::vector<SpawnCommand>& spawns);

    ScriptVM vm_;
    ControlCommands* commands_ = nullptr;
    World* world_ = nullptr;          // the last tick's world: a script verb the socket runs acts on it
    Entity running_;                  // whose hook is running, so control.command knows its owner
    std::vector<ScriptVerb> verbs_;
    std::vector<ScriptSubscription> subscriptions_;
    BlockedFn blocked_;
    InputMap* input_ = nullptr;
    const CameraState* camera_ = nullptr;
    AssetManager* assets_ = nullptr;
    EventBus* events_ = nullptr;
    bool firstPersonView_ = true;
    bool handsFree_ = true;
};

}  // namespace engine

#endif
