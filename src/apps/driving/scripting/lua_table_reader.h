#ifndef RAYTRACER_APPS_DRIVING_SCRIPTING_LUA_TABLE_READER_H
#define RAYTRACER_APPS_DRIVING_SCRIPTING_LUA_TABLE_READER_H

// WHAT EVERY RULE-SET READER SHARES (internal to scripting/): collecting faults as "file: key.path what was wrong",
// showing a Lua value back in a message, and reading names, strings and lists out of tables without leaving anything
// on the Lua stack. The rule-set reader (rule_set_lua) and the rules reader (rules_lua) both build on it.
//
// Every helper restores the stack to what it found, except pushField, which pushes one value and says where it is
// (the caller restores).

#include "engine/scripting/lua_state.h"

#include <cstddef>
#include <string>
#include <vector>

namespace driving {

class LuaTableReader {
public:
    // `file` names the file in messages; at most `maxErrors` faults are kept (one more line says there were more).
    LuaTableReader(lua_State* state, const std::string& file, std::size_t maxErrors);

    // All faults, one per line.
    std::string report() const;
    bool clean() const { return errors_.empty(); }

protected:
    lua_State* L;
    std::string fileName;

    // Records "file: path what".
    void fail(const std::string& path, const std::string& what);

    // A number as messages show it.
    static std::string numberText(double v);
    // The value at `idx` as it would be shown in a message ("missing", "\"text\"", 3, "a table", ...).
    std::string describe(int idx);
    // Words joined with ", ".
    static std::string joined(const std::vector<std::string>& words);
    // The string at `idx` (a real string, not a number made into one).
    bool stringAt(int idx, std::string& out);
    // Pushes field `key` of the table at `t` and returns its absolute index (the caller restores the stack).
    int pushField(int t, const std::string& key);
    // The names (string keys) of the table at `t`, sorted; keys that are not names go to `odd` as text, sorted.
    void keysOf(int t, std::vector<std::string>& names, std::vector<std::string>& odd);
    // Reports every key of the table at `t` that is not in `allowed`, and every key that is not a name (except the
    // ones in `allowedOdd`). Returns the sorted names.
    std::vector<std::string> checkKeys(int t, const std::string& path, const std::vector<std::string>& allowed,
                                       const std::vector<std::string>& allowedOdd = std::vector<std::string>());
    // A list of strings at `idx`, in written order. A single string is a list of one when `singleOk`.
    bool stringList(int idx, const std::string& path, bool singleOk, std::vector<std::string>& out);

private:
    std::vector<std::string> errors_;
    std::size_t maxErrors_;
    bool overflowed_ = false;
};

}  // namespace driving

#endif
