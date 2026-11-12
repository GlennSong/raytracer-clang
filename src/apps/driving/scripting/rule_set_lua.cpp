#include "apps/driving/scripting/rule_set_lua.h"

#include "apps/driving/scripting/lua_table_reader.h"
#include "engine/scripting/lua_state.h"
#include "engine/scripting/script_vm.h"

#include <algorithm>
#include <cmath>

namespace driving {

namespace {

// The global the rule-set file defines, and the root of every key path in a message.
constexpr const char* kRoot = "rule_set";

// The words of the file's own language that this reader checks for. (drives_on's two words are the one place left and
// right are named: the fact is what is being recorded.)
constexpr const char* kKeyName = "name";
constexpr const char* kKeyDrivesOn = "drives_on";
constexpr const char* kKeyRanks = "ranks";
constexpr const char* kKeyDevices = "devices";
constexpr const char* kKeyNumbers = "numbers";
constexpr const char* kKeyPersonality = "personality";
constexpr const char* kKeyMeans = "means";
constexpr const char* kKeySign = "sign";
constexpr const char* kKeyPaint = "paint";
constexpr const char* kKeyUnit = "unit";
constexpr const char* kKeySource = "source";
constexpr const char* kKeyUsual = "usual";
constexpr const char* kKeyLow = "low";
constexpr const char* kKeyHigh = "high";
constexpr const char* kEffectStopLine = "stop_line";
constexpr const char* kEffectRank = "rank";
constexpr const char* kEffectTopSpeed = "top_speed";
constexpr const char* kEffectClosedTo = "closed_to";
constexpr const char* kEffectOpenTo = "open_to";
constexpr const char* kEffectForbids = "forbids";
constexpr const char* kWordDrivesLeft = "left";
constexpr const char* kWordDrivesRight = "right";

// A number's value is the first entry of its table.
constexpr lua_Integer kNumberValueSlot = 1;

// Reads one rule set. Every problem found is collected (up to the limit) and reading carries on, so one load reports
// every fault.
class Reader : public LuaTableReader {
public:
    Reader(lua_State* state, const std::string& file) : LuaTableReader(state, file, kMaxRuleSetErrors) {}

    // Reads everything into `out` (a scratch rule set). Returns true when no fault was found.
    bool read(RuleSet& out) {
        const int base = lua_gettop(L);
        lua_getglobal(L, kRoot);
        if (!lua_istable(L, -1)) {
            fail(kRoot, "is " + describe(-1) + ", expected a table: the rule-set file must define a global `rule_set`");
            lua_settop(L, base);
            return false;
        }
        const int root = lua_gettop(L);
        // Unknown keys at this level are not errors: the later readers' parts (control, rules, modes, signals,
        // markings, lane_kinds, announcers, detail) live here too.
        readName(root, out);
        readDrivesOn(root, out);
        const bool ranksOk = readRanks(root, out);
        readDevices(root, out, ranksOk);
        readNumbers(root, out);
        readPersonality(root, out);
        lua_settop(L, base);
        return clean();
    }

private:
    void readName(int root, RuleSet& out) {
        const int top = lua_gettop(L);
        const int v = pushField(root, kKeyName);
        const std::string path = std::string(kRoot) + "." + kKeyName;
        std::string s;
        if (!stringAt(v, s) || s.empty()) fail(path, "is " + describe(v) + ", expected a non-empty string");
        else out.name = s;
        lua_settop(L, top);
    }

    void readDrivesOn(int root, RuleSet& out) {
        const int top = lua_gettop(L);
        const int v = pushField(root, kKeyDrivesOn);
        const std::string path = std::string(kRoot) + "." + kKeyDrivesOn;
        std::string s;
        if (stringAt(v, s) && s == kWordDrivesLeft) out.drivesOn = DrivesOn::Left;
        else if (s == kWordDrivesRight && lua_type(L, v) == LUA_TSTRING) out.drivesOn = DrivesOn::Right;
        else fail(path, "is " + describe(v) + ", which is not one of (" + kWordDrivesLeft + ", " + kWordDrivesRight + ")");
        lua_settop(L, top);
    }

    bool readRanks(int root, RuleSet& out) {
        const int top = lua_gettop(L);
        const int v = pushField(root, kKeyRanks);
        const std::string path = std::string(kRoot) + "." + kKeyRanks;
        std::vector<std::string> list;
        bool ok = stringList(v, path, false, list);
        if (ok) {
            for (std::size_t i = 0; i < list.size(); ++i) {
                if (std::find(list.begin(), list.begin() + static_cast<std::ptrdiff_t>(i), list[i]) != list.begin() + static_cast<std::ptrdiff_t>(i)) {
                    fail(path, "lists \"" + list[i] + "\" twice, expected each rank once");
                    ok = false;
                }
            }
        }
        if (ok) out.ranks = list;
        lua_settop(L, top);
        return ok;
    }

    // Who a closed_to / open_to names: the word "all", or a list of vehicle tag names.
    void readScope(int idx, const std::string& path, VehicleScope& scope) {
        std::string s;
        if (stringAt(idx, s)) {
            if (s == kWordAll) {
                scope.kind = VehicleScope::Kind::All;
                scope.tags.clear();
            } else {
                fail(path, "is \"" + s + "\", expected a list of vehicle tag names or the word \"" + kWordAll + "\"");
            }
            return;
        }
        std::vector<std::string> tags;
        if (!lua_istable(L, idx)) {
            fail(path, "is " + describe(idx) + ", expected a list of vehicle tag names or the word \"" + kWordAll + "\"");
            return;
        }
        if (stringList(idx, path, false, tags)) {
            scope.kind = VehicleScope::Kind::Listed;
            scope.tags = tags;
        }
    }

    void readMeans(int means, const std::string& path, const RuleSet& rules, bool ranksOk, Device& d) {
        const int top = lua_gettop(L);
        const std::vector<std::string> allowed = {kEffectStopLine, kEffectRank,     kEffectTopSpeed,
                                                  kEffectClosedTo, kEffectOpenTo,   kEffectForbids};
        const std::vector<std::string> given = checkKeys(means, path, allowed);
        for (const std::string& key : given) {
            if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) continue;
            const std::string keyPath = path + "." + key;
            const int v = pushField(means, key);
            if (key == kEffectStopLine) {
                if (lua_type(L, v) == LUA_TBOOLEAN) d.stopLine = lua_toboolean(L, v) != 0;
                else fail(keyPath, "is " + describe(v) + ", expected true or false");
            } else if (key == kEffectRank) {
                std::string s;
                if (!stringAt(v, s)) {
                    fail(keyPath, "is " + describe(v) + ", expected one of ranks (" + joined(rules.ranks) + ")");
                } else if (ranksOk) {
                    const int index = rules.rankIndex(s);
                    if (index == kNoRank) fail(keyPath, "is \"" + s + "\", which is not one of ranks (" + joined(rules.ranks) + ")");
                    else d.rank = index;
                }
            } else if (key == kEffectTopSpeed) {
                std::string s;
                if (stringAt(v, s) && s == kWordValue) d.topSpeedIsValue = true;
                else fail(keyPath, "is " + describe(v) + ", expected the word \"" + kWordValue + "\" (the regulation's own number is the speed)");
            } else if (key == kEffectClosedTo) {
                readScope(v, keyPath, d.closedTo);
            } else if (key == kEffectOpenTo) {
                readScope(v, keyPath, d.openTo);
            } else if (key == kEffectForbids) {
                std::vector<std::string> names;
                if (stringList(v, keyPath, true, names)) d.forbids = names;
            }
            lua_settop(L, top);
        }
    }

    void readDevices(int root, RuleSet& out, bool ranksOk) {
        const int top = lua_gettop(L);
        const int list = pushField(root, kKeyDevices);
        const std::string path = std::string(kRoot) + "." + kKeyDevices;
        if (!lua_istable(L, list)) {
            fail(path, "is " + describe(list) + ", expected a table of devices");
            lua_settop(L, top);
            return;
        }
        std::vector<std::string> names;
        std::vector<std::string> odd;
        keysOf(list, names, odd);
        for (const std::string& o : odd) fail(path, "has the key " + o + ", which is not a name; expected device names");
        const std::vector<std::string> allowed = {kKeyMeans, kKeySign, kKeyPaint};
        std::vector<Device> devices;
        for (const std::string& name : names) {
            const int mark = lua_gettop(L);
            const std::string devicePath = path + "." + name;
            const int dt = pushField(list, name);
            if (!lua_istable(L, dt)) {
                fail(devicePath, "is " + describe(dt) + ", expected a table with (" + joined(allowed) + ")");
                lua_settop(L, mark);
                continue;
            }
            Device d;
            d.name = name;
            const std::vector<std::string> given = checkKeys(dt, devicePath, allowed);
            const int means = pushField(dt, kKeyMeans);
            if (lua_istable(L, means)) readMeans(means, devicePath + "." + kKeyMeans, out, ranksOk, d);
            else fail(devicePath + "." + kKeyMeans, "is " + describe(means) + ", expected a table of effects");
            lua_settop(L, mark + 1);
            const int sign = pushField(dt, kKeySign);
            if (!lua_isnil(L, sign)) {
                std::string s;
                if (stringAt(sign, s)) d.sign = s;
                else fail(devicePath + "." + kKeySign, "is " + describe(sign) + ", expected a string");
            }
            lua_settop(L, mark + 1);
            const int paint = pushField(dt, kKeyPaint);
            if (!lua_isnil(L, paint)) {
                std::vector<std::string> marks;
                if (stringList(paint, devicePath + "." + kKeyPaint, false, marks)) d.paint = marks;
            }
            devices.push_back(d);
            lua_settop(L, mark);
        }
        out.devices = devices;
        lua_settop(L, top);
    }

    // The unit word at `idx` and its factor. Reports and returns false when it is missing or unknown.
    bool readUnit(int table, const std::string& path, std::string& unit, double& factor) {
        const int top = lua_gettop(L);
        const int v = pushField(table, kKeyUnit);
        const std::string unitPath = path + "." + kKeyUnit;
        bool ok = false;
        std::string s;
        if (!stringAt(v, s)) {
            fail(unitPath, "is " + describe(v) + ", expected a unit, one of (" + unitWords() + ")");
        } else if (!unitFactor(s, factor)) {
            fail(unitPath, "is \"" + s + "\", which is not one of the units (" + unitWords() + ")");
        } else {
            unit = s;
            ok = true;
        }
        lua_settop(L, top);
        return ok;
    }

    // A number in a table at `table` under `key`.
    bool readNumberField(int table, const std::string& path, const char* key, double& value) {
        const int top = lua_gettop(L);
        const int v = pushField(table, key);
        bool ok = false;
        if (lua_type(L, v) == LUA_TNUMBER && std::isfinite(lua_tonumber(L, v))) {
            value = lua_tonumber(L, v);
            ok = true;
        } else {
            fail(path + "." + key, "is " + describe(v) + ", expected a number");
        }
        lua_settop(L, top);
        return ok;
    }

    void readNumbers(int root, RuleSet& out) {
        const int top = lua_gettop(L);
        const int list = pushField(root, kKeyNumbers);
        const std::string path = std::string(kRoot) + "." + kKeyNumbers;
        if (!lua_istable(L, list)) {
            fail(path, "is " + describe(list) + ", expected a table of numbers");
            lua_settop(L, top);
            return;
        }
        std::vector<std::string> names;
        std::vector<std::string> odd;
        keysOf(list, names, odd);
        for (const std::string& o : odd) fail(path, "has the key " + o + ", which is not a name; expected number names");
        const std::vector<std::string> allowed = {kKeyUnit, kKeySource};
        const std::vector<std::string> allowedOdd = {std::to_string(kNumberValueSlot)};
        std::vector<Number> numbers;
        for (const std::string& name : names) {
            const int mark = lua_gettop(L);
            const std::string numberPath = path + "." + name;
            const int nt = pushField(list, name);
            if (!lua_istable(L, nt)) {
                fail(numberPath, "is " + describe(nt) + ", expected { value, unit = \"...\", source = \"...\" }");
                lua_settop(L, mark);
                continue;
            }
            checkKeys(nt, numberPath, allowed, allowedOdd);
            Number n;
            n.name = name;
            double factor = 0;
            bool ok = readUnit(nt, numberPath, n.unit, factor);
            lua_rawgeti(L, nt, kNumberValueSlot);
            const int vslot = lua_gettop(L);
            if (lua_type(L, vslot) == LUA_TNUMBER && std::isfinite(lua_tonumber(L, vslot))) {
                n.writtenValue = lua_tonumber(L, vslot);
            } else {
                fail(numberPath + "[" + std::to_string(kNumberValueSlot) + "]",
                     "is " + describe(vslot) + ", expected the number itself as the first entry");
                ok = false;
            }
            lua_settop(L, mark + 1);
            const int src = pushField(nt, kKeySource);
            if (!lua_isnil(L, src)) {
                std::string s;
                if (stringAt(src, s)) {
                    n.source = s;
                    n.hasSource = true;
                } else {
                    fail(numberPath + "." + kKeySource, "is " + describe(src) + ", expected a string");
                }
            }
            if (ok) {
                n.value = n.writtenValue * factor;
                numbers.push_back(n);
            }
            lua_settop(L, mark);
        }
        out.numbers = numbers;
        lua_settop(L, top);
    }

    void readPersonality(int root, RuleSet& out) {
        const int top = lua_gettop(L);
        const int list = pushField(root, kKeyPersonality);
        const std::string path = std::string(kRoot) + "." + kKeyPersonality;
        if (!lua_istable(L, list)) {
            fail(path, "is " + describe(list) + ", expected a table of driver settings");
            lua_settop(L, top);
            return;
        }
        std::vector<std::string> names;
        std::vector<std::string> odd;
        keysOf(list, names, odd);
        for (const std::string& o : odd) fail(path, "has the key " + o + ", which is not a name; expected setting names");
        const std::vector<std::string> allowed = {kKeyUsual, kKeyLow, kKeyHigh, kKeyUnit};
        std::vector<PersonalSetting> settings;
        for (const std::string& name : names) {
            const int mark = lua_gettop(L);
            const std::string settingPath = path + "." + name;
            const int st = pushField(list, name);
            if (!lua_istable(L, st)) {
                fail(settingPath, "is " + describe(st) + ", expected { usual, low, high, unit }");
                lua_settop(L, mark);
                continue;
            }
            checkKeys(st, settingPath, allowed);
            PersonalSetting p;
            p.name = name;
            double factor = 0;
            bool ok = readUnit(st, settingPath, p.unit, factor);
            ok = readNumberField(st, settingPath, kKeyUsual, p.writtenUsual) && ok;
            ok = readNumberField(st, settingPath, kKeyLow, p.writtenLow) && ok;
            ok = readNumberField(st, settingPath, kKeyHigh, p.writtenHigh) && ok;
            if (ok && !(p.writtenLow <= p.writtenUsual && p.writtenUsual <= p.writtenHigh)) {
                fail(settingPath, "has low " + numberText(p.writtenLow) + ", usual " + numberText(p.writtenUsual) +
                                      ", high " + numberText(p.writtenHigh) + "; expected low <= usual <= high");
                ok = false;
            }
            if (ok) {
                p.usual = p.writtenUsual * factor;
                p.low = p.writtenLow * factor;
                p.high = p.writtenHigh * factor;
                settings.push_back(p);
            }
            lua_settop(L, mark);
        }
        out.personality = settings;
        lua_settop(L, top);
    }
};

}  // namespace

bool loadRuleSet(engine::ScriptVM& vm, const std::string& fileName, RuleSet& out, std::string* err) {
    lua_State* L = engine::luaState(vm);
    Reader reader(L, fileName);
    RuleSet scratch;
    if (!reader.read(scratch)) {
        if (err) *err = reader.report();
        return false;
    }
    out = scratch;
    return true;
}

}  // namespace driving
