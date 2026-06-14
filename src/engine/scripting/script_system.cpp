#include "script_system.h"

#include "lua_state.h"
#include "gameplay_bindings.h"
#include "script_behaviour.h"
#include "../world.h"
#include "../components.h"
#include "../asset_manager.h"
#include "../../log.h"

#include <cstdint>
#include <vector>

namespace engine {
namespace {

// Pack an Entity to the integer scripts see (must match gameplay_bindings'
// toEntity: generation<<32 | index).
lua_Integer packEntity(Entity e) {
    return static_cast<lua_Integer>(
        (static_cast<uint64_t>(e.generation) << 32) | static_cast<uint64_t>(e.index));
}

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

}  // namespace

ScriptSystem::ScriptSystem() {
    openGameplayLibrary(vm_);
}

void ScriptSystem::setServices(InputMap* input, const CameraState* camera,
                               AssetManager* assets) {
    input_ = input;
    camera_ = camera;
    assets_ = assets;
}

void ScriptSystem::update(FrameContext& ctx) {
    setServices(&ctx.actions, &ctx.view.camera, &ctx.assets);
    tick(ctx.world, ctx.frameDelta);
}

void ScriptSystem::tick(World& world, double dt) {
    lua_State* L = luaState(vm_);

    std::vector<SpawnCommand> spawns;
    GameplayContext gctx;
    gctx.world = &world;
    gctx.input = input_;
    gctx.camera = camera_;
    gctx.spawns = &spawns;
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
        if (!sb.started) {
            callHook(L, "start", e, dt, /*hasDt=*/false);
            sb.started = true;
        }
        callHook(L, "update", e, dt, /*hasDt=*/true);
        lua_pop(L, 1);                                        // pop inst
    });

    setGameplayContext(vm_, nullptr);

    // Apply deferred spawns now that iteration is done. Each is a dynamic cube
    // (Transform/Collider/RigidBody/Velocity) the PhysicsSystem turns into a body
    // and launches with `velocity` next fixed step — the Lua port of the C++
    // ShootingSystem's bullet. The Renderable needs GPU upload, so it's added
    // only when an AssetManager is present (headless ticks skip it).
    for (const SpawnCommand& c : spawns) {
        Entity e = world.create();
        Transform t;
        t.position = c.position;
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});

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
    }
}

}  // namespace engine
