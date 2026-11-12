#include "apps/driving/scripting/lua_table_reader.h"

#include <algorithm>
#include <sstream>

namespace driving {

namespace {

// Longest value kept when one string is shown back in an error (a long string is cut).
constexpr std::size_t kShownValueLimit = 60;

}  // namespace

LuaTableReader::LuaTableReader(lua_State* state, const std::string& file, std::size_t maxErrors)
    : L(state), fileName(file), maxErrors_(maxErrors) {}

std::string LuaTableReader::report() const {
    std::string text;
    for (std::size_t i = 0; i < errors_.size(); ++i) {
        if (i) text += "\n";
        text += errors_[i];
    }
    if (overflowed_) text += "\n" + fileName + ": more faults found than are listed here";
    return text;
}

void LuaTableReader::fail(const std::string& path, const std::string& what) {
    if (errors_.size() >= maxErrors_) {
        overflowed_ = true;
        return;
    }
    errors_.push_back(fileName + ": " + path + " " + what);
}

std::string LuaTableReader::numberText(double v) {
    std::ostringstream s;
    s << v;
    return s.str();
}

std::string LuaTableReader::describe(int idx) {
    const int type = lua_type(L, idx);
    if (type == LUA_TNIL || type == LUA_TNONE) return "missing";
    if (type == LUA_TSTRING) {
        std::string s = lua_tostring(L, idx);
        if (s.size() > kShownValueLimit) s = s.substr(0, kShownValueLimit) + "...";
        return "\"" + s + "\"";
    }
    if (type == LUA_TNUMBER) return numberText(lua_tonumber(L, idx));
    if (type == LUA_TBOOLEAN) return lua_toboolean(L, idx) ? "true" : "false";
    if (type == LUA_TTABLE) return "a table";
    return std::string("a ") + lua_typename(L, type);
}

std::string LuaTableReader::joined(const std::vector<std::string>& words) {
    std::string out;
    for (std::size_t i = 0; i < words.size(); ++i) {
        if (i) out += ", ";
        out += words[i];
    }
    return out;
}

bool LuaTableReader::stringAt(int idx, std::string& out) {
    if (lua_type(L, idx) != LUA_TSTRING) return false;
    out = lua_tostring(L, idx);
    return true;
}

int LuaTableReader::pushField(int t, const std::string& key) {
    lua_getfield(L, t, key.c_str());
    return lua_gettop(L);
}

void LuaTableReader::keysOf(int t, std::vector<std::string>& names, std::vector<std::string>& odd) {
    const int top = lua_gettop(L);
    t = lua_absindex(L, t);
    lua_pushnil(L);
    while (lua_next(L, t) != 0) {
        lua_pop(L, 1);   // the value; the key stays for the next step
        if (lua_type(L, -1) == LUA_TSTRING) {
            names.push_back(lua_tostring(L, -1));
        } else if (lua_type(L, -1) == LUA_TNUMBER) {
            lua_pushvalue(L, -1);   // a copy: converting the key itself would confuse lua_next
            odd.push_back(lua_tostring(L, -1));
            lua_pop(L, 1);
        } else {
            odd.push_back(std::string("a ") + lua_typename(L, lua_type(L, -1)));
        }
    }
    lua_settop(L, top);
    std::sort(names.begin(), names.end());
    std::sort(odd.begin(), odd.end());
}

std::vector<std::string> LuaTableReader::checkKeys(int t, const std::string& path,
                                                   const std::vector<std::string>& allowed,
                                                   const std::vector<std::string>& allowedOdd) {
    std::vector<std::string> names;
    std::vector<std::string> odd;
    keysOf(t, names, odd);
    for (const std::string& n : names)
        if (std::find(allowed.begin(), allowed.end(), n) == allowed.end())
            fail(path, "has the key \"" + n + "\", which is not one of (" + joined(allowed) + ")");
    for (const std::string& o : odd)
        if (std::find(allowedOdd.begin(), allowedOdd.end(), o) == allowedOdd.end())
            fail(path, "has the key " + o + ", which is not a name; expected only names (" + joined(allowed) + ")");
    return names;
}

bool LuaTableReader::stringList(int idx, const std::string& path, bool singleOk, std::vector<std::string>& out) {
    const int top = lua_gettop(L);
    std::string one;
    if (singleOk && stringAt(idx, one)) {
        out.assign(1, one);
        return true;
    }
    if (!lua_istable(L, idx)) {
        fail(path, "is " + describe(idx) + ", expected a list of strings" + (singleOk ? " or one string" : ""));
        return false;
    }
    bool ok = true;
    std::vector<std::string> names;
    std::vector<std::string> odd;
    keysOf(idx, names, odd);
    if (!names.empty()) {
        fail(path, "has the key \"" + names[0] + "\", expected only a list of strings");
        ok = false;
    }
    const lua_Integer n = static_cast<lua_Integer>(lua_rawlen(L, idx));
    std::vector<std::string> items;
    for (lua_Integer i = 1; i <= n; ++i) {
        lua_rawgeti(L, idx, i);
        std::string s;
        if (stringAt(-1, s)) {
            items.push_back(s);
        } else {
            fail(path + "[" + std::to_string(i) + "]", "is " + describe(-1) + ", expected a string");
            ok = false;
        }
        lua_settop(L, top);
    }
    if (ok) out = items;
    return ok;
}

}  // namespace driving
