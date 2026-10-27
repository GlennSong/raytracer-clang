#include "test_framework.h"

#include "../src/engine/scripting/script_system.h"
#include "../src/engine/scripting/script_behaviour.h"
#include "../src/engine/control/control_commands.h"
#include "../src/engine/event_types.h"
#include "../src/engine/audio/audio_engine.h"   // PlaySound
#include "../src/engine/world.h"
#include "../src/engine/components.h"
#include "../src/rt_math.h"

#include <cmath>

using namespace engine;  // namespace migration (ADR-0015)

namespace {
Entity makeScripted(World& w, const char* src, Vec3 pos = Vec3(9, 9, 9)) {
    Entity e = w.create();
    Transform t;
    t.position = pos;
    w.add<Transform>(e, t);
    ScriptBehaviour sb;
    sb.source = src;
    w.add<ScriptBehaviour>(e, sb);
    return e;
}
}  // namespace

TEST_CASE(script_behaviour_start_runs_once_update_every_tick) {
    // start() zeroes the position once; update() adds dt to y each tick. After 5
    // ticks of 0.1, y == 0.5 ONLY if start ran once (if it re-ran each tick it
    // would reset to 0 and y would stay 0.1) — the MonoBehaviour lifecycle.
    World world;
    const char* src = R"LUA(
        local M = {}
        function M:start(e)      entity.set_position(e, {0, 0, 0}) end
        function M:update(e, dt) entity.translate(e, {0, dt, 0}) end
        return M
    )LUA";
    Entity e = makeScripted(world, src);
    ScriptSystem sys;
    for (int i = 0; i < 5; ++i) sys.tick(world, 0.1);

    Transform* t = world.get<Transform>(e);
    if (!t) { CHECK(false); return; }
    CHECK_APPROX(t->position.y, 0.5, 1e-9);
    CHECK_APPROX(t->position.x, 0.0, 1e-9);
}

TEST_CASE(script_behaviour_state_is_per_entity) {
    // Two entities, same source. Each update bumps self.n and writes x = n. With
    // independent per-entity instance tables both reach x == 3 after 3 ticks; a
    // shared table would interleave to 5 and 6. So x == 3 proves isolation.
    World world;
    const char* src = R"LUA(
        local M = {}
        function M:start(e)      self.n = 0 end
        function M:update(e, dt) self.n = self.n + 1
                                 entity.set_position(e, {self.n, 0, 0}) end
        return M
    )LUA";
    Entity a = makeScripted(world, src);
    Entity b = makeScripted(world, src);
    ScriptSystem sys;
    for (int i = 0; i < 3; ++i) sys.tick(world, 0.016);

    Transform* ta = world.get<Transform>(a);
    Transform* tb = world.get<Transform>(b);
    if (!ta || !tb) { CHECK(false); return; }
    CHECK_APPROX(ta->position.x, 3.0, 1e-9);
    CHECK_APPROX(tb->position.x, 3.0, 1e-9);
}

TEST_CASE(script_behaviour_can_rotate_entity) {
    World world;
    const char* src = R"LUA(
        local M = {}
        function M:start(e)      self.a = 0 end
        function M:update(e, dt) self.a = self.a + dt; entity.set_yaw(e, self.a) end
        return M
    )LUA";
    Entity e = makeScripted(world, src);
    ScriptSystem sys;
    sys.tick(world, 1.5707963);   // ~pi/2 about +Y

    Transform* t = world.get<Transform>(e);
    if (!t) { CHECK(false); return; }
    CHECK(std::fabs(t->orientation.y) > 0.6);   // sin(pi/4) ~ 0.707; was identity
}

TEST_CASE(script_behaviour_alive_binding) {
    World world;
    const char* src = R"LUA(
        local M = {}
        function M:update(e, dt)
            if entity.alive(e) then entity.set_position(e, {1, 2, 3}) end
        end
        return M
    )LUA";
    Entity e = makeScripted(world, src);
    ScriptSystem sys;
    sys.tick(world, 0.016);

    Transform* t = world.get<Transform>(e);
    if (!t) { CHECK(false); return; }
    CHECK_APPROX(t->position.x, 1.0, 1e-9);
    CHECK_APPROX(t->position.z, 3.0, 1e-9);
}

TEST_CASE(script_behaviour_error_is_isolated) {
    // A behaviour whose chunk returns no table fails to load; the system marks it
    // and moves on. A sibling behaviour must still run (one bad script can't take
    // down the others). NOTE: this case intentionally logs a script error.
    World world;
    Entity bad = makeScripted(world, "x = 1");   // returns nothing -> not a table
    const char* good = R"LUA(
        local M = {}
        function M:update(e, dt) entity.set_position(e, {7, 7, 7}) end
        return M
    )LUA";
    Entity ok = makeScripted(world, good);

    ScriptSystem sys;
    sys.tick(world, 0.016);   // must not crash
    sys.tick(world, 0.016);

    Transform* tok = world.get<Transform>(ok);
    Transform* tbad = world.get<Transform>(bad);
    if (!tok || !tbad) { CHECK(false); return; }
    CHECK_APPROX(tok->position.x, 7.0, 1e-9);    // good sibling ran
    CHECK_APPROX(tbad->position.x, 9.0, 1e-9);   // bad one untouched (initial)
}

// --- control.command / control.run (ADR-0253) ---

TEST_CASE(script_verb_is_added_runs_and_goes_with_its_entity) {
    // A script adds a verb in start(); the socket's path (commands.run) reaches its function with the words, and
    // its reply comes back as written ("ok" added when the script leaves it off). When the entity dies, the verb
    // goes at the next tick.
    World world;
    ControlCommands table;
    Entity e = makeScripted(world, R"LUA(
        local M = {}
        function M:start(e)
            assert(control.command{name = "Greet", usage = "greet [who]", summary = "Say hello.",
                                   run = function(args) return "hello " .. (args[1] or "you") end})
            local ok, why = control.command{name = "greet", run = function() end}
            self.refused = (ok == nil) and why ~= nil
        end
        return M
    )LUA");
    ScriptSystem sys;
    sys.setCommands(&table);
    sys.tick(world, 0.1);

    CHECK(table.has("greet"));
    CHECK(table.refused() == 1);   // the second add of the same name
    CHECK(table.run(parseControlCommand("greet Glenn")) == std::string("ok hello Glenn"));
    CHECK(table.run(parseControlCommand("greet")) == std::string("ok hello you"));
    const std::vector<ControlCommandInfo> list = table.list();
    CHECK(list.size() == static_cast<std::size_t>(1));
    if (!list.empty()) {
        CHECK(list[0].usage == std::string("greet [who]"));
        CHECK(list[0].owner.rfind("script:", 0) == 0);
    }

    world.destroy(e);
    sys.tick(world, 0.1);
    CHECK(!table.has("greet"));
}

TEST_CASE(script_verb_errors_and_spawns_from_the_socket) {
    // Run between ticks (as the socket does): a Lua error comes back as "err <verb>: ...", and a spawn the verb
    // asks for is applied when it returns, on the last tick's world.
    World world;
    ControlCommands table;
    makeScripted(world, R"LUA(
        local M = {}
        function M:start(e)
            control.command{name = "boom", run = function() error("no") end}
            control.command{name = "drop", run = function() spawn.block{position = {0, 5, 0}, velocity = {0, 0, 0}} end}
        end
        return M
    )LUA");
    ScriptSystem sys;
    sys.setCommands(&table);
    sys.tick(world, 0.1);

    const std::string boom = table.run(parseControlCommand("boom"));
    CHECK(boom.rfind("err boom:", 0) == 0);
    int bodies = 0;
    world.each<RigidBody>([&](Entity, RigidBody&) { ++bodies; });
    CHECK(bodies == 0);
    CHECK(table.run(parseControlCommand("drop")) == std::string("ok"));
    world.each<RigidBody>([&](Entity, RigidBody&) { ++bodies; });
    CHECK(bodies == 1);
}

TEST_CASE(script_runs_any_verb_and_its_verbs_go_with_the_system) {
    // control.run reaches an engine verb from inside a tick; destroying the ScriptSystem removes its verbs and
    // leaves the others.
    World world;
    ControlCommands table;
    table.add("echo", "echo <word>", "Say it back.", [](const ControlCommand& c) {
        return "ok " + (c.args.empty() ? std::string() : c.args[0]);
    });
    Entity e = makeScripted(world, R"LUA(
        local M = {}
        function M:start(e)
            control.command{name = "mine", run = function() return "yes" end}
            local r = control.run("echo hi")
            entity.set_position(e, {r == "ok hi" and 1 or 0, 0, 0})
        end
        return M
    )LUA");
    {
        ScriptSystem sys;
        sys.setCommands(&table);
        sys.tick(world, 0.1);
        CHECK(table.has("mine"));
    }
    const Transform* t = world.get<Transform>(e);
    CHECK(t != nullptr && t->position.x == 1.0);
    CHECK(!table.has("mine"));
    CHECK(table.has("echo"));
}

// --- events.publish / events.on (ADR-0253) ---

TEST_CASE(script_events_reach_cpp_and_back) {
    // A script publishes a named event (C++ hears it, typed, at the queued dispatch) and subscribes to one (C++
    // publishes it typed; the script gets the fields as a table). Its subscription goes with its entity.
    World world;
    EventBus bus;
    describeEngineEvents(bus);
    std::string heard;
    bus.subscribe<PlaySound>([&](const PlaySound& s) { heard = s.clip; });
    Entity e = makeScripted(world, R"LUA(
        local M = {}
        function M:start(e)
            assert(events.publish("PlaySound", {clip = "bell", volume = 0.5}))
            local ok, why = events.publish("Nope", {})
            assert(ok == nil and why:find("unknown event"))
            self.id = assert(events.on("PlaySound", function(f)
                entity.set_position(e, {f.volume, f.position[2], #f.clip})
            end))
        end
        return M
    )LUA");
    ScriptSystem sys;
    sys.setServices(nullptr, nullptr, nullptr, &bus);
    sys.tick(world, 0.1);
    CHECK(heard.empty());   // queued, like sound.play
    bus.dispatchQueued();   // the script's own bell reaches both
    CHECK(heard == "bell");

    PlaySound cue;
    cue.clip = "door";
    cue.volume = 0.25f;
    cue.position = Vec3(0, 7, 0);
    bus.publish(cue);
    const Transform* t = world.get<Transform>(e);
    CHECK(t != nullptr && std::fabs(t->position.x - 0.25) < 1e-6 && t->position.y == 7 && t->position.z == 4);
    CHECK(bus.subscriberCount<PlaySound>() == 2);

    world.destroy(e);
    sys.tick(world, 0.1);
    CHECK(bus.subscriberCount<PlaySound>() == 1);
}

TEST_CASE(script_events_survive_deep_tables) {
    // A table nested past what the converter follows (or the stack grows) becomes null there, not a crash.
    World world;
    EventBus bus;
    describeEngineEvents(bus);
    int heard = 0;
    bus.subscribe<PlaySound>([&](const PlaySound&) { ++heard; });
    makeScripted(world, R"LUA(
        local M = {}
        function M:start(e)
            local t = {}
            local deep = t
            for i = 1, 200 do deep.x = {}; deep = deep.x end
            assert(events.publish("PlaySound", {clip = "deep", extra = t}))
        end
        return M
    )LUA");
    ScriptSystem sys;
    sys.setServices(nullptr, nullptr, nullptr, &bus);
    sys.tick(world, 0.1);
    bus.dispatchQueued();
    CHECK(heard == 1);
}
