#include "brain_lua.h"

// The table reader the driving library's Lua readers share (faults collected as "file: key.path what was wrong", keys
// checked against the ones a row may have): used here rather than written again.
#include "../../../driving/scripting/lua_table_reader.h"

#include <algorithm>

namespace citysim {
namespace brain {

namespace {

// How many faults one reading lists before it says there are more.
constexpr std::size_t kMaxFaults = 20;

class BrainReader : public driving::LuaTableReader {
public:
    BrainReader(lua_State* state, const std::string& file) : driving::LuaTableReader(state, file, kMaxFaults) {}

    void read(BrainSpec& spec) {
        const int top = lua_gettop(L);
        lua_getglobal(L, "brain");
        const int bt = lua_gettop(L);
        if (!lua_istable(L, bt)) {
            fail("brain", "is " + describe(bt) + ", expected the table of the brain's states");
            lua_settop(L, top);
            return;
        }
        checkKeys(bt, "brain", {"start", "no_plan", "intent_event", "intents", "events", "states"});
        spec.start = text(bt, "brain", "start", "");
        spec.noPlan = text(bt, "brain", "no_plan", "");
        spec.intentEvent = text(bt, "brain", "intent_event", "");
        eachRow(bt, "intents", [&](int t, const std::string& at) {
            checkKeys(t, at, {"name", "arrive"});
            IntentSpec row;
            row.name = text(t, at, "name", "");
            row.arrive = text(t, at, "arrive", "");
            spec.intents.push_back(row);
        });
        eachRow(bt, "events", [&](int t, const std::string& at) {
            checkKeys(t, at, {"name", "otherwise"});
            EventSpec row;
            row.name = text(t, at, "name", "");
            row.otherwise = text(t, at, "otherwise", row.otherwise);
            spec.events.push_back(row);
        });
        eachRow(bt, "states", [&](int t, const std::string& at) {
            checkKeys(t, at, {"name", "group", "skill", "ends", "clock", "where", "holds", "on"});
            StateSpec row;
            row.name = text(t, at, "name", "");
            const std::string key = row.name.empty() ? at : "states." + row.name;
            row.group = text(t, key, "group", "");
            row.skill = text(t, key, "skill", row.skill);
            row.ends = text(t, key, "ends", row.ends);
            row.clock = text(t, key, "clock", "");
            row.where = text(t, key, "where", row.where);
            const int holds = pushField(t, "holds");
            if (lua_isboolean(L, holds)) row.holds = lua_toboolean(L, holds) != 0;
            else if (!lua_isnil(L, holds)) fail(key + ".holds", "is " + describe(holds) + ", expected true or false");
            lua_pop(L, 1);
            const int on = pushField(t, "on");
            if (lua_istable(L, on)) {
                // (the keys come back sorted, so the same file always reads the same: Lua walks a table in no
                // promised order)
                std::vector<std::string> events, odd;
                keysOf(on, events, odd);
                for (const std::string& o : odd) fail(key + ".on", "has the key " + o + ", which is not an event's name");
                for (const std::string& event : events) {
                    const int value = pushField(on, event);
                    std::string leadsTo;
                    if (stringAt(value, leadsTo)) row.on.emplace_back(event, leadsTo);
                    else fail(key + ".on." + event, "is " + describe(value) + ", expected the name of a state or a verb");
                    lua_pop(L, 1);
                }
            } else if (!lua_isnil(L, on)) {
                fail(key + ".on", "is " + describe(on) + ", expected a table of event = what it leads to");
            }
            lua_pop(L, 1);
            spec.states.push_back(row);
        });
        lua_settop(L, top);
    }

private:
    // A text field; `fallback` when the key is absent.
    std::string text(int t, const std::string& at, const std::string& key, const std::string& fallback) {
        const int v = pushField(t, key);
        std::string out = fallback;
        if (!lua_isnil(L, v) && !stringAt(v, out)) fail(at + "." + key, "is " + describe(v) + ", expected a name");
        lua_pop(L, 1);
        return out;
    }
    // Each row of the list `key` of the table at `t`, handed to `row` with its absolute index and its name in messages.
    template <typename Fn>
    void eachRow(int t, const std::string& key, Fn row) {
        const int list = pushField(t, key);
        if (!lua_istable(L, list)) {
            fail(key, "is " + describe(list) + ", expected a list of rows");
            lua_pop(L, 1);
            return;
        }
        const lua_Integer n = static_cast<lua_Integer>(lua_rawlen(L, list));
        for (lua_Integer i = 1; i <= n; ++i) {
            lua_rawgeti(L, list, i);
            const int item = lua_gettop(L);
            const std::string at = key + "[" + std::to_string(i) + "]";
            if (lua_istable(L, item)) row(item, at);
            else fail(at, "is " + describe(item) + ", expected a table");
            lua_pop(L, 1);
        }
        lua_pop(L, 1);
    }
};

}  // namespace

bool loadBrainSpec(engine::ScriptVM& vm, BrainSpec& out, std::string* err) {
    BrainSpec spec;
    BrainReader reader(engine::luaState(vm), spec.file);
    reader.read(spec);
    if (!reader.clean()) {
        if (err) *err = reader.report();
        return false;
    }
    out = spec;
    return true;
}

}  // namespace brain
}  // namespace citysim
