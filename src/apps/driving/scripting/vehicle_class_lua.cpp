#include "apps/driving/scripting/vehicle_class_lua.h"

#include "engine/scripting/lua_state.h"

#include <algorithm>
#include <sstream>
#include <utility>
#include <vector>

namespace driving {

namespace {

// The block's words, each once.
constexpr const char* kKeySpeedUp = "speed_up";
constexpr const char* kKeyBrake = "brake";
constexpr const char* kKeyHardestBrake = "hardest_brake";
constexpr const char* kKeyTopSpeed = "top_speed";
constexpr const char* kKeyTags = "tags";
constexpr const char* kKeyTrailer = "trailer";
constexpr const char* kKeyTrailerLength = "length";
constexpr const char* kKeyTrailerHitch = "hitch_ahead";

std::string joined(const std::vector<std::string>& words) {
    std::string out;
    for (std::size_t i = 0; i < words.size(); ++i) out += (i ? ", " : "") + words[i];
    return out;
}

class BlockReader {
public:
    BlockReader(lua_State* state, const std::string& where) : L(state), where_(where) {}

    std::vector<std::string> errors;

    void fail(const std::string& key, const std::string& what) { errors.push_back(where_ + "." + key + " " + what); }

    // Every key of the table at `t` that is not in `allowed` is a fault (reported in name order).
    void checkKeys(int t, const std::string& path, const std::vector<std::string>& allowed) {
        std::vector<std::string> names;
        const int top = lua_gettop(L);
        lua_pushnil(L);
        while (lua_next(L, t) != 0) {
            lua_pop(L, 1);
            if (lua_type(L, -1) == LUA_TSTRING) names.push_back(lua_tostring(L, -1));
            else names.push_back(std::string("(a ") + lua_typename(L, lua_type(L, -1)) + " key)");
        }
        lua_settop(L, top);
        std::sort(names.begin(), names.end());
        for (const std::string& n : names)
            if (std::find(allowed.begin(), allowed.end(), n) == allowed.end())
                errors.push_back(where_ + path + " has the key \"" + n + "\", which is not one of (" + joined(allowed) +
                                 ")");
    }

    // A number field of the table at `t`; `positive` asks for one above zero, otherwise zero or more.
    bool number(int t, const std::string& key, bool positive, Real& out) {
        lua_getfield(L, t, key.c_str());
        const bool isNumber = lua_type(L, -1) == LUA_TNUMBER;
        const Real v = isNumber ? static_cast<Real>(lua_tonumber(L, -1)) : 0;
        lua_pop(L, 1);
        if (!isNumber) {
            fail(key, "is missing or not a number");
            return false;
        }
        if (positive ? !(v > 0) : !(v >= 0)) {
            std::ostringstream s;
            s << v;
            fail(key, "is " + s.str() + ", expected a number " + (positive ? "above zero" : "of zero or more"));
            return false;
        }
        out = v;
        return true;
    }

    // The list of tag names at `t`.`tags`: sorted, repeats dropped.
    void tags(int t, std::vector<std::string>& out) {
        lua_getfield(L, t, kKeyTags);
        const int list = lua_gettop(L);
        if (!lua_istable(L, list)) {
            fail(kKeyTags, "is missing or not a list of tag names");
            lua_pop(L, 1);
            return;
        }
        const lua_Integer n = static_cast<lua_Integer>(luaL_len(L, list));
        if (n == 0) fail(kKeyTags, "is empty: a vehicle needs at least one tag for the lanes to match");
        std::vector<std::string> names;
        for (lua_Integer i = 1; i <= n; ++i) {
            lua_rawgeti(L, list, i);
            if (lua_type(L, -1) == LUA_TSTRING && lua_rawlen(L, -1) > 0) names.push_back(lua_tostring(L, -1));
            else fail(std::string(kKeyTags) + "[" + std::to_string(i) + "]", "is not a tag name (a non-empty string)");
            lua_pop(L, 1);
        }
        lua_pop(L, 1);
        std::sort(names.begin(), names.end());
        names.erase(std::unique(names.begin(), names.end()), names.end());
        out = names;
    }

private:
    lua_State* L;
    std::string where_;
};

}  // namespace

bool readVehicleClassBlock(lua_State* L, int t, const std::string& where, VehicleClass& out, std::string* err) {
    const int base = lua_gettop(L);
    t = lua_absindex(L, t);
    BlockReader r(L, where);
    if (!lua_istable(L, t)) {
        if (err) *err = where + " is missing or not a table";
        return false;
    }
    VehicleClass c = out;
    r.checkKeys(t, "", {kKeyBrake, kKeyHardestBrake, kKeySpeedUp, kKeyTags, kKeyTopSpeed, kKeyTrailer});
    r.number(t, kKeySpeedUp, true, c.speedUp);
    r.number(t, kKeyBrake, true, c.brake);
    r.number(t, kKeyHardestBrake, true, c.hardestBrake);
    r.number(t, kKeyTopSpeed, true, c.topSpeed);
    r.tags(t, c.tags);
    lua_getfield(L, t, kKeyTrailer);
    const int trailer = lua_gettop(L);
    if (lua_istable(L, trailer)) {
        r.checkKeys(trailer, std::string(".") + kKeyTrailer, {kKeyTrailerHitch, kKeyTrailerLength});
        r.number(trailer, kKeyTrailerLength, true, c.trailerLength);
        // (the hitch may sit behind the back axle: any number)
        lua_getfield(L, trailer, kKeyTrailerHitch);
        if (lua_type(L, -1) == LUA_TNUMBER) c.hitchAheadOfBackAxle = static_cast<Real>(lua_tonumber(L, -1));
        else r.fail(std::string(kKeyTrailer) + "." + kKeyTrailerHitch, "is missing or not a number");
    } else if (!lua_isnil(L, trailer)) {
        r.fail(kKeyTrailer, "is not a table");
    }
    lua_settop(L, base);
    if (!r.errors.empty()) {
        if (err) {
            std::string text;
            for (std::size_t i = 0; i < r.errors.size(); ++i) text += (i ? "\n" : "") + r.errors[i];
            *err = text;
        }
        return false;
    }
    out = std::move(c);
    return true;
}

}  // namespace driving
