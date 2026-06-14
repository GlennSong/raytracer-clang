#include "test_framework.h"

#include "../src/engine/scripting/script_system.h"
#include "../src/engine/scripting/script_behaviour.h"
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
