#include "script_system.h"
#include "../player_pose.h"

#include "lua_state.h"
#include "lua_helpers.h"     // packEntity (scripts see self.entity as this)
#include "gameplay_bindings.h"
#include "lua_json.h"
#include "procgen_bindings.h"
#include "script_behaviour.h"
#include "../world.h"
#include "../components.h"
#include "../asset_manager.h"
#include "../control/control_commands.h"
#include "../../log.h"
#include "../../profile.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <vector>

namespace engine {
namespace {

// Load a behaviour's chunk, which must return a table (the per-entity instance);
// store it in the registry and record the ref. Returns false (logging) on error.
bool loadInstance(lua_State* L, ScriptBehaviour& sb) {
    if (luaL_loadstring(L, sb.source.c_str()) != LUA_OK ||
        lua_pcall(L, 0, 1, 0) != LUA_OK) {
        LOG_ERROR << "[script] behaviour load error: " << lua_tostring(L, -1);
        lua_pop(L, 1);
        return false;
    }
    if (!lua_istable(L, -1)) {
        LOG_ERROR << "[script] behaviour must `return` a table of hooks";
        lua_pop(L, 1);
        return false;
    }
    sb.instanceRef = luaL_ref(L, LUA_REGISTRYINDEX);   // pops the table
    return true;
}

// Call instance:hook(e[, dt]) if it exists. The instance table must be at the
// top of the stack on entry, and is left there on return.
void callHook(lua_State* L, const char* hook, Entity e, double dt, bool hasDt) {
    lua_getfield(L, -1, hook);                 // [inst, fn?]
    if (!lua_isfunction(L, -1)) {
        lua_pop(L, 1);                         // [inst]
        return;
    }
    lua_pushvalue(L, -2);                       // self = inst -> [inst, fn, inst]
    lua_pushinteger(L, packEntity(e));         // [inst, fn, inst, e]
    int nargs = 2;
    if (hasDt) {
        lua_pushnumber(L, dt);                  // [inst, fn, inst, e, dt]
        nargs = 3;
    }
    if (lua_pcall(L, nargs, 0, 0) != LUA_OK) {  // [inst] or [inst, errmsg]
        LOG_ERROR << "[script] " << hook << "() error: " << lua_tostring(L, -1);
        lua_pop(L, 1);                          // [inst]
    }
}

// The owner name a script's verbs are listed under: "script:<entity index>.<generation>".
std::string ownerName(Entity e) {
    return "script:" + std::to_string(e.index) + "." + std::to_string(e.generation);
}

}  // namespace

ScriptSystem::~ScriptSystem() {
    // The VM (and every function ref) dies with this system; the verbs and subscriptions pointing at them go first.
    if (commands_ != nullptr)
        for (const ScriptVerb& v : verbs_) commands_->remove(v.name);
    if (events_ != nullptr)
        for (const ScriptSubscription& sub : subscriptions_) events_->unsubscribe(sub.id);
}

ScriptSystem::ScriptSystem() {
    // Gameplay scripts get both surfaces in one VM: the effectful gameplay API
    // and the (pure) procgen builders, so a behaviour can generate geometry
    // (e.g. the gun viewmodel) and hand it to spawn.model.
    openGameplayLibrary(vm_);
    openProcgenLibrary(vm_);
}

GameplayContext ScriptSystem::context(World* world, std::vector<SpawnCommand>* spawns) {
    GameplayContext g;
    g.world = world;
    g.input = input_;
    g.camera = camera_;
    g.spawns = spawns;
    g.events = events_;
    g.firstPersonView = firstPersonView_;
    g.handsFree = handsFree_;
    g.commands = commands_;
    if (commands_ != nullptr)
        g.addCommand = [this](const std::string& name, const std::string& usage, const std::string& summary,
                              int fnRef) { return addVerb(running_, name, usage, summary, fnRef); };
    if (events_ != nullptr) {
        g.subscribe = [this](const std::string& name, int fnRef, std::string* error) {
            return addSubscription(running_, name, fnRef, error);
        };
        g.unsubscribe = [this](uint64_t id) { return removeSubscription(id); };
    }
    return g;
}

uint64_t ScriptSystem::addSubscription(Entity owner, const std::string& name, int fnRef, std::string* error) {
    lua_State* L = luaState(vm_);
    EventBus::SubscriptionId id = EventBus::INVALID_SUBSCRIPTION;
    if (!owner.valid()) {
        *error = "events.on is called from a script's start() or update()";
    } else {
        id = events_->subscribeNamed(name, [this, fnRef, name](const std::string& fields) {
            std::string error;
            if (!callScript(fnRef, [&](lua_State* L) { pushJson(L, nlohmann::json::parse(fields)); }, nullptr, &error))
                LOG_ERROR << "[script] events.on(" << name << ") error: " << error;
        });
        if (id == EventBus::INVALID_SUBSCRIPTION) *error = "unknown event: " + name + " (try events?)";
    }
    if (id == EventBus::INVALID_SUBSCRIPTION) {
        luaL_unref(L, LUA_REGISTRYINDEX, fnRef);
        return 0;
    }
    subscriptions_.push_back({owner, id, fnRef});
    return id;
}

bool ScriptSystem::removeSubscription(uint64_t id) {
    for (std::size_t i = 0; i < subscriptions_.size(); ++i) {
        if (subscriptions_[i].id != id) continue;
        events_->unsubscribe(id);
        luaL_unref(luaState(vm_), LUA_REGISTRYINDEX, subscriptions_[i].fnRef);
        subscriptions_.erase(subscriptions_.begin() + static_cast<std::ptrdiff_t>(i));
        return true;
    }
    return false;
}

std::string ScriptSystem::addVerb(Entity owner, const std::string& name, const std::string& usage,
                                  const std::string& summary, int fnRef) {
    lua_State* L = luaState(vm_);
    if (!owner.valid()) {
        luaL_unref(L, LUA_REGISTRYINDEX, fnRef);
        return "control.command is called from a script's start() or update()";
    }
    const bool added = commands_->add(name, usage, summary,
        [this, fnRef](const ControlCommand& cmd) { return runVerb(fnRef, cmd); }, ownerName(owner));
    if (!added) {
        luaL_unref(L, LUA_REGISTRYINDEX, fnRef);
        return "'" + name + "' is taken (or not one word)";
    }
    std::string stored = name;
    std::transform(stored.begin(), stored.end(), stored.begin(), [](unsigned char c) { return std::tolower(c); });
    verbs_.push_back({owner, stored, fnRef});
    return "";
}

// A script function called from outside its hooks: a verb from the socket (between ticks) or from control.run
// inside one, an event from the bus. Between ticks it gets a context of its own on the last tick's world, and its
// spawns are applied when it returns; inside a tick it shares the tick's, so its spawns wait for the tick's
// iteration to end like any other.
bool ScriptSystem::callScript(int fnRef, const std::function<void(lua_State*)>& push, std::string* result,
                              std::string* error) {
    lua_State* L = luaState(vm_);
    GameplayContext* outer = gameplayContext(vm_);
    std::vector<SpawnCommand> spawns;
    GameplayContext own = context(world_, &spawns);
    if (outer == nullptr) setGameplayContext(vm_, &own);

    lua_rawgeti(L, LUA_REGISTRYINDEX, fnRef);
    push(L);
    const bool ok = lua_pcall(L, 1, 1, 0) == LUA_OK;
    if (!ok) {
        *error = lua_tostring(L, -1) ? lua_tostring(L, -1) : "error";
    } else if (result != nullptr) {
        const char* text = lua_isnil(L, -1) ? "" : lua_tostring(L, -1);
        *result = text ? text : "";
    }
    lua_pop(L, 1);

    if (outer == nullptr) {
        setGameplayContext(vm_, nullptr);
        if (world_ != nullptr) applySpawns(*world_, spawns);
    }
    return ok;
}

std::string ScriptSystem::runVerb(int fnRef, const ControlCommand& cmd) {
    std::string reply, error;
    const bool ok = callScript(fnRef, [&](lua_State* L) {
        lua_createtable(L, static_cast<int>(cmd.args.size()), 0);
        for (std::size_t i = 0; i < cmd.args.size(); ++i) {
            lua_pushstring(L, cmd.args[i].c_str());
            lua_seti(L, -2, static_cast<lua_Integer>(i + 1));
        }
    }, &reply, &error);
    if (!ok) return "err " + cmd.name + ": " + error;
    if (reply.empty()) return "ok";
    if (reply.rfind("ok", 0) != 0 && reply.rfind("err", 0) != 0) reply = "ok " + reply;
    return reply;
}

void ScriptSystem::dropScript(Entity owner) {
    lua_State* L = luaState(vm_);
    if (commands_ != nullptr) commands_->removeOwnedBy(ownerName(owner));
    for (const ScriptVerb& v : verbs_)
        if (v.owner == owner) luaL_unref(L, LUA_REGISTRYINDEX, v.fnRef);
    verbs_.erase(std::remove_if(verbs_.begin(), verbs_.end(), [&](const ScriptVerb& v) { return v.owner == owner; }),
                 verbs_.end());
    for (const ScriptSubscription& sub : subscriptions_) {
        if (sub.owner != owner) continue;
        events_->unsubscribe(sub.id);
        luaL_unref(L, LUA_REGISTRYINDEX, sub.fnRef);
    }
    subscriptions_.erase(std::remove_if(subscriptions_.begin(), subscriptions_.end(),
                                        [&](const ScriptSubscription& sub) { return sub.owner == owner; }),
                         subscriptions_.end());
}

void ScriptSystem::setServices(InputMap* input, const CameraState* camera,
                               AssetManager* assets, EventBus* events) {
    input_ = input;
    camera_ = camera;
    assets_ = assets;
    events_ = events;
}

void ScriptSystem::update(FrameContext& ctx) {
    setServices(&ctx.actions, &ctx.view.camera, &ctx.assets, &ctx.events);
    // First person = the player's own eyes: not the on-foot shoulder rig
    // (PlayerSystem's V toggle, published via Settings) and not seated in a
    // vehicle (the chase camera). The gun script gates its viewmodel and fire
    // on this — a first-person viewmodel makes no sense floating beside a
    // third-person camera.
    bool inVehicle = false, handsFree = true;
    ctx.world.each<ControlledBy>([&](Entity e, ControlledBy&) {
        if (ctx.world.has<InVehicle>(e)) inVehicle = true;
        // hands on a ledge, a wall, a ladder, or swimming: no gun in them (engine/player_pose.h)
        if (const PlayerPose* p = ctx.world.get<PlayerPose>(e))
            handsFree = p->handsFree >= 2;   // the body's mode says (not on the toggles of a chute, nor in a suit)
    });
    setHandsFree(handsFree);
    setFirstPersonView(!ctx.settings.getBool("playerThirdPerson", false) &&
                       !inVehicle);
    if (ctx.commands != nullptr && commands_ == nullptr) setCommands(ctx.commands);
    tick(ctx.world, ctx.frameDelta);
}

void ScriptSystem::tick(World& world, double dt) {
    RT_PROFILE_ZONE_NAMED("scriptTick");
    lua_State* L = luaState(vm_);

    world_ = &world;
    // A script whose entity died (or lost its behaviour) takes its verbs and subscriptions with it.
    std::vector<Entity> gone;
    auto lost = [&](Entity e) {
        if ((!world.alive(e) || !world.has<ScriptBehaviour>(e)) && std::find(gone.begin(), gone.end(), e) == gone.end())
            gone.push_back(e);
    };
    for (const ScriptVerb& v : verbs_) lost(v.owner);
    for (const ScriptSubscription& sub : subscriptions_) lost(sub.owner);
    for (Entity e : gone) dropScript(e);

    std::vector<SpawnCommand> spawns;
    GameplayContext gctx = context(&world, &spawns);
    setGameplayContext(vm_, &gctx);

    // Mutating ScriptBehaviour's own fields (refs/flags) is allowed inside each;
    // entity creation is deferred to after iteration, so the no-structural-
    // mutation contract (ADR-0006) holds.
    world.each<ScriptBehaviour>([&](Entity e, ScriptBehaviour& sb) {
        if (sb.failed) return;
        if (sb.instanceRef < 0 && !loadInstance(L, sb)) {
            sb.failed = true;
            return;
        }

        lua_rawgeti(L, LUA_REGISTRYINDEX, sb.instanceRef);   // [inst]
        running_ = e;
        if (!sb.started) {
            callHook(L, "start", e, dt, /*hasDt=*/false);
            sb.started = true;
        }
        callHook(L, "update", e, dt, /*hasDt=*/true);
        running_ = Entity{};
        lua_pop(L, 1);                                        // pop inst
    });

    setGameplayContext(vm_, nullptr);
    applySpawns(world, spawns);
}

void ScriptSystem::applySpawns(World& world, const std::vector<SpawnCommand>& spawns) {

    // Apply deferred spawns now that iteration is done (the no-structural-
    // mutation contract held during each). Renderables need GPU upload, so they
    // are added only when an AssetManager is present (headless ticks skip them).
    for (const SpawnCommand& c : spawns) {
        // The muzzle behind a wall (a window pane, a door) from the eye: the
        // shot is swallowed rather than born on the far side.
        if (c.kind == SpawnKind::Block && c.hasOrigin && blocked_ && blocked_(c.origin, c.position))
            continue;
        Entity e = world.create();
        Transform t;
        t.position = c.position;
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});

        if (c.kind == SpawnKind::Block) {
            // A dynamic cube the PhysicsSystem turns into a body and launches at
            // `velocity` next fixed step — the Lua port of ShootingSystem's bullet.
            Collider col;
            col.shape = ColliderShape::Box;
            col.halfExtent = Vec3(c.size * 0.5, c.size * 0.5, c.size * 0.5);
            col.restitution = c.restitution;
            col.friction = c.friction;
            world.add<Collider>(e, col);

            RigidBody rb;
            rb.motion = BodyMotion::Dynamic;
            world.add<RigidBody>(e, rb);

            Velocity vel;
            vel.linear = c.velocity;
            world.add<Velocity>(e, vel);

            if (assets_ != nullptr) {
                Renderable r;
                r.mesh = assets_->acquirePrimitive(
                    "box", Vec3(c.size, c.size, c.size));
                r.material.albedo = c.color;
                r.material.emission = c.emission;
                r.material.roughness = 0.4f;
                world.add<Renderable>(e, r);
            }
        } else {  // SpawnKind::Model — a procgen mesh entity (e.g. the gun)
            if (assets_ != nullptr && c.mesh) {
                Renderable r;
                r.mesh = assets_->acquireMesh(*c.mesh);
                r.material.albedo = c.color;
                r.material.emission = c.emission;
                r.material.metallic = static_cast<float>(c.metallic);
                r.material.roughness = static_cast<float>(c.roughness);
                world.add<Renderable>(e, r);
            }
            // An optional behaviour drives it (the gun model follows the camera).
            if (!c.script.empty()) {
                ScriptBehaviour sb;
                sb.source = c.script;
                world.add<ScriptBehaviour>(e, sb);
            }
        }
    }
}

}  // namespace engine
