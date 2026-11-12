#include "apps/driving/scripting/rule_set_lua.h"

#include "apps/driving/scripting/lua_table_reader.h"
#include "engine/procgen/city/roads/lanes/control_plan.h"   // ControlPolicy: the placement policy's plain form
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
// Right of way (issue #260).
constexpr const char* kKeyControl = "control";
constexpr const char* kKeyMovementRank = "movement_rank";
constexpr const char* kKeySettle = "settle";
constexpr const char* kKeyEveryoneStops = "everyone_stops";
constexpr const char* kKeyGivesWayOnGreen = "gives_way_on_green";
constexpr const char* kKeyArrow = "arrow";
constexpr const char* kKeyTurnOnRed = "turn_on_red";
constexpr const char* kKeyMovement = "movement";
constexpr const char* kKeyAfter = "after";
constexpr const char* kKeyWhenDark = "when_dark";
constexpr const char* kKeyBetweenJunctions = "between_junctions";

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
        readNumbers(root, kKeyNumbers, true, out.numbers);
        readNumbers(root, kKeyDefaultLimits, false, out.defaultLimits);
        checkDefaultLimitsAreSpeeds(out.defaultLimits);
        readPersonality(root, out);
        readMovementRank(root, out);
        readControls(root, out);
        readBetweenJunctions(root, out);
        lua_settop(L, base);
        return clean();
    }

protected:
    // A list of movement words at `idx` into a mask, each named once. False (reported) on any fault.
    bool movementList(int idx, const std::string& path, std::vector<Movement>& out) {
        std::vector<std::string> words;
        if (!stringList(idx, path, true, words)) return false;
        bool ok = true;
        out.clear();
        for (const std::string& w : words) {
            Movement m = Movement::Straight;
            if (!movementOf(w, m)) {
                fail(path, "names \"" + w + "\", which is not one of the movements (" + movementWords() + ")");
                ok = false;
            } else if (std::find(out.begin(), out.end(), m) != out.end()) {
                fail(path, "lists \"" + w + "\" twice, expected each movement once");
                ok = false;
            } else {
                out.push_back(m);
            }
        }
        return ok;
    }

    static MovementMask maskOf(const std::vector<Movement>& list) {
        MovementMask mask = 0;
        for (const Movement m : list) mask = static_cast<MovementMask>(mask | (1u << static_cast<uint8_t>(m)));
        return mask;
    }

    // movement_rank: optional; the movements in the order they go first.
    void readMovementRank(int root, RuleSet& out) {
        const int top = lua_gettop(L);
        const int v = pushField(root, kKeyMovementRank);
        if (!lua_isnil(L, v)) {
            std::vector<Movement> list;
            if (movementList(v, std::string(kRoot) + "." + kKeyMovementRank, list)) out.movementRank = list;
        }
        lua_settop(L, top);
    }

    // A settle list at `idx`: one or more questions, each once. False (reported) on any fault.
    bool settleList(int idx, const std::string& settlePath, std::vector<SettleQuestion>& out) {
        std::vector<std::string> words;
        bool ok = true;
        if (!stringList(idx, settlePath, false, words)) {
            ok = false;
        } else if (words.empty()) {
            fail(settlePath, "is empty, expected at least one of (" + settleQuestionWords() + ")");
            ok = false;
        }
        for (const std::string& w : words) {
            SettleQuestion q = SettleQuestion::Light;
            if (!settleQuestionOf(w, q)) {
                fail(settlePath, "names \"" + w + "\", which is not one of the questions (" + settleQuestionWords() + ")");
                ok = false;
            } else if (std::find(out.begin(), out.end(), q) != out.end()) {
                fail(settlePath, "asks \"" + w + "\" twice, expected each question once");
                ok = false;
            } else {
                out.push_back(q);
            }
        }
        return ok;
    }

    // between_junctions: optional (issue #284); how a meeting filed under no junction is settled, { settle = { ... } }:
    // only the questions that mean something there (no lights, and no approaches of one junction to compare).
    void readBetweenJunctions(int root, RuleSet& out) {
        const int top = lua_gettop(L);
        const int v = pushField(root, kKeyBetweenJunctions);
        const std::string path = std::string(kRoot) + "." + kKeyBetweenJunctions;
        if (lua_isnil(L, v)) {
            lua_settop(L, top);
            return;
        }
        if (!lua_istable(L, v)) {
            fail(path, "is " + describe(v) + ", expected { " + kKeySettle + " = { ... } }");
            lua_settop(L, top);
            return;
        }
        checkKeys(v, path, {kKeySettle});
        std::vector<SettleQuestion> settle;
        bool ok = settleList(pushField(v, kKeySettle), path + "." + kKeySettle, settle);
        for (const SettleQuestion q : settle)
            if (q == SettleQuestion::Light || q == SettleQuestion::KerbSide) {
                fail(path + "." + kKeySettle, "asks \"" + std::string(settleQuestionWord(q)) +
                                                  "\", which means nothing between junctions (expected " +
                                                  settleQuestionWord(SettleQuestion::SignRank) + ", " +
                                                  settleQuestionWord(SettleQuestion::Movement) + " or " +
                                                  settleQuestionWord(SettleQuestion::ArrivedFirst) + ")");
                ok = false;
            }
        if (ok) {
            out.hasBetweenJunctions = true;
            out.betweenJunctions = settle;
        }
        lua_settop(L, top);
    }

    // control: optional; each kind of junction control.
    void readControls(int root, RuleSet& out) {
        const int top = lua_gettop(L);
        const int list = pushField(root, kKeyControl);
        const std::string path = std::string(kRoot) + "." + kKeyControl;
        if (lua_isnil(L, list)) {
            lua_settop(L, top);
            return;
        }
        if (!lua_istable(L, list)) {
            fail(path, "is " + describe(list) + ", expected a table of junction controls");
            lua_settop(L, top);
            return;
        }
        std::vector<std::string> names;
        std::vector<std::string> odd;
        keysOf(list, names, odd);
        for (const std::string& o : odd) fail(path, "has the key " + o + ", which is not a name; expected control names");
        const std::vector<std::string> allowed = {kKeySettle, kKeyEveryoneStops, kKeyGivesWayOnGreen, kKeyArrow,
                                                  kKeyTurnOnRed, kKeyWhenDark};
        std::vector<ControlKind> controls;
        for (const std::string& name : names) {
            const int mark = lua_gettop(L);
            const std::string controlPath = path + "." + name;
            const int ct = pushField(list, name);
            if (!lua_istable(L, ct)) {
                fail(controlPath, "is " + describe(ct) + ", expected a table with (" + joined(allowed) + ")");
                lua_settop(L, mark);
                continue;
            }
            checkKeys(ct, controlPath, allowed);
            ControlKind k;
            k.name = name;
            bool ok = true;
            // settle: required, a list of questions, each once.
            {
                const int v = pushField(ct, kKeySettle);
                if (!settleList(v, controlPath + "." + kKeySettle, k.settle)) ok = false;
                lua_settop(L, mark + 1);
            }
            {
                const int v = pushField(ct, kKeyEveryoneStops);
                if (lua_type(L, v) == LUA_TBOOLEAN) k.everyoneStops = lua_toboolean(L, v) != 0;
                else if (!lua_isnil(L, v)) {
                    fail(controlPath + "." + kKeyEveryoneStops, "is " + describe(v) + ", expected true or false");
                    ok = false;
                }
                lua_settop(L, mark + 1);
            }
            for (const char* key : {kKeyGivesWayOnGreen, kKeyArrow}) {
                const int v = pushField(ct, key);
                if (!lua_isnil(L, v)) {
                    std::vector<Movement> list;
                    if (!movementList(v, controlPath + "." + key, list)) ok = false;
                    const MovementMask mask = maskOf(list);
                    if (std::string(key) == kKeyArrow) k.arrow = mask;
                    else k.givesWayOnGreen = mask;
                }
                lua_settop(L, mark + 1);
            }
            {
                const int v = pushField(ct, kKeyTurnOnRed);
                const std::string redPath = controlPath + "." + kKeyTurnOnRed;
                if (lua_istable(L, v)) {
                    checkKeys(v, redPath, {kKeyMovement, kKeyAfter});
                    const int m = pushField(v, kKeyMovement);
                    std::string s;
                    if (stringAt(m, s) && movementOf(s, k.turnOnRedMovement)) {
                        k.turnOnRed = true;
                    } else {
                        fail(redPath + "." + kKeyMovement,
                             "is " + describe(m) + ", expected one of the movements (" + movementWords() + ")");
                        ok = false;
                    }
                    lua_settop(L, mark + 2);
                    const int a = pushField(v, kKeyAfter);
                    if (!lua_isnil(L, a)) {
                        if (stringAt(a, s) && s == kWordFullStop) {
                            k.turnOnRedAfterFullStop = true;
                        } else {
                            fail(redPath + "." + kKeyAfter,
                                 "is " + describe(a) + ", expected \"" + std::string(kWordFullStop) + "\"");
                            ok = false;
                        }
                    }
                } else if (!lua_isnil(L, v)) {
                    fail(redPath, "is " + describe(v) + ", expected { movement = ..., after = \"" +
                                      std::string(kWordFullStop) + "\" }");
                    ok = false;
                }
                lua_settop(L, mark + 1);
            }
            {
                const int v = pushField(ct, kKeyWhenDark);
                if (!lua_isnil(L, v) && !stringAt(v, k.whenDark)) {
                    fail(controlPath + "." + kKeyWhenDark, "is " + describe(v) + ", expected the name of another control");
                    ok = false;
                }
                lua_settop(L, mark + 1);
            }
            // Lights' tables mean nothing to a control with no lights: a slip of the pen, said so.
            if (!k.hasLights() && (k.givesWayOnGreen != 0 || k.arrow != 0 || k.turnOnRed || !k.whenDark.empty())) {
                fail(controlPath, "gives " + std::string(kKeyGivesWayOnGreen) + ", " + kKeyArrow + ", " + kKeyTurnOnRed +
                                      " or " + kKeyWhenDark + ", which only a control whose settle list asks \"" +
                                      settleQuestionWord(SettleQuestion::Light) + "\" can use");
                ok = false;
            }
            if (ok) controls.push_back(k);
            lua_settop(L, mark);
        }
        // A light treated as another control while dark names a control there is, one with no lights of its own.
        for (const ControlKind& k : controls) {
            if (k.whenDark.empty()) continue;
            const std::vector<ControlKind>::const_iterator it =
                std::find_if(controls.begin(), controls.end(), [&](const ControlKind& c) { return c.name == k.whenDark; });
            if (it == controls.end())
                fail(path + "." + k.name + "." + kKeyWhenDark, "names \"" + k.whenDark + "\", which is not a control here");
            else if (it->hasLights())
                fail(path + "." + k.name + "." + kKeyWhenDark, "names \"" + k.whenDark +
                                                                   "\", which has lights of its own; expected a control without");
        }
        out.controls = controls;
        lua_settop(L, top);
    }

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

    // Where a value came from: the optional string under `source` in the table at `table`.
    void readSource(int table, const std::string& path, std::string& source, bool& hasSource) {
        const int top = lua_gettop(L);
        const int src = pushField(table, kKeySource);
        if (!lua_isnil(L, src)) {
            std::string s;
            if (stringAt(src, s)) {
                source = s;
                hasSource = true;
            } else {
                fail(path + "." + kKeySource, "is " + describe(src) + ", expected a string");
            }
        }
        lua_settop(L, top);
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

    // A table of numbers under `key` (numbers, default_limits), each { value, unit = "...", source = "..." }, into
    // `out`. An absent table is a fault when `required`, and an empty list otherwise. `within` is the key path of the
    // table at `root` (rule_set, or rule_set.placement).
    void readNumbers(int root, const char* key, bool required, std::vector<Number>& out,
                     const std::string& within = kRoot) {
        const int top = lua_gettop(L);
        const int list = pushField(root, key);
        const std::string path = within + "." + key;
        if (!required && lua_isnil(L, list)) {
            lua_settop(L, top);
            return;
        }
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
            readSource(nt, numberPath, n.source, n.hasSource);
            if (ok) {
                n.value = n.writtenValue * factor;
                numbers.push_back(n);
            }
            lua_settop(L, mark);
        }
        out = numbers;
        lua_settop(L, top);
    }

    // Every default limit is a speed: a limit in seconds or metres is a slip of the pen.
    void checkDefaultLimitsAreSpeeds(const std::vector<Number>& limits,
                                     const std::string& path = std::string(kRoot) + "." + kKeyDefaultLimits) {
        for (const Number& n : limits) {
            UnitFamily family = UnitFamily::Ratio;
            if (unitFamily(n.unit, family) && family != UnitFamily::Speed)
                fail(path + "." + n.name + "." + kKeyUnit,
                     "is \"" + n.unit + "\", which is " + unitFamilyWords(family) + "; a limit is a speed");
        }
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
        const std::vector<std::string> allowed = {kKeyUsual, kKeyLow, kKeyHigh, kKeyUnit, kKeySource};
        std::vector<PersonalSetting> settings;
        for (const std::string& name : names) {
            const int mark = lua_gettop(L);
            const std::string settingPath = path + "." + name;
            const int st = pushField(list, name);
            if (!lua_istable(L, st)) {
                fail(settingPath, "is " + describe(st) + ", expected { usual, low, high, unit, source }");
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
            readSource(st, settingPath, p.source, p.hasSource);
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

// ---- The placement policy (issue #240) -------------------------------------------------------------------------------

constexpr const char* kKeyPlacement = "placement";
constexpr const char* kKeyClassOrder = "class_order";
constexpr const char* kKeySpeedLimits = "speed_limits";
constexpr const char* kKeyJunctions = "junctions";
constexpr const char* kKeyWhen = "when";
constexpr const char* kKeyApart = "apart";
constexpr const char* kKeySigns = "signs";
constexpr const char* kKeyMin = "min";
constexpr const char* kKeyMax = "max";
constexpr const char* kFactArms = "arms";
constexpr const char* kFactApproaches = "approaches";
constexpr const char* kFactMajor = "major";
constexpr const char* kFactMinor = "minor";
constexpr const char* kFactHasMinor = "has_minor";
constexpr const char* kFactRampEnd = "ramp_end";
constexpr const char* kFactMajorLanes = "major_lanes";
constexpr const char* kFactMinorLanes = "minor_lanes";
constexpr const char* kFactMajorLimit = "major_limit";
constexpr const char* kFactMinorLimit = "minor_limit";
constexpr const char* kSignsAll = "all";

// Reads rule_set.placement, checked against the rule set the same file holds.
class PlacementReader : public Reader {
public:
    PlacementReader(lua_State* state, const std::string& file) : Reader(state, file) {}

    bool readPlacement(const RuleSet& rules, engine::roads::lanes::ControlPolicy& out) {
        const int base = lua_gettop(L);
        lua_getglobal(L, kRoot);
        const std::string path = std::string(kRoot) + "." + kKeyPlacement;
        if (!lua_istable(L, -1)) {
            fail(kRoot, "is " + describe(-1) + ", expected a table: the rule-set file must define a global `rule_set`");
            lua_settop(L, base);
            return false;
        }
        const int p = pushField(lua_gettop(L), kKeyPlacement);
        if (!lua_istable(L, p)) {
            fail(path, "is " + describe(p) + ", expected the placement policy { " + kKeyClassOrder + ", " + kKeySpeedLimits +
                           ", " + kKeyJunctions + " }");
            lua_settop(L, base);
            return false;
        }
        checkKeys(p, path, {kKeyClassOrder, kKeySpeedLimits, kKeyJunctions});
        out.name = rules.name;
        {
            const int v = pushField(p, kKeyClassOrder);
            const std::string at = path + "." + kKeyClassOrder;
            if (stringList(v, at, false, out.classOrder)) {
                std::vector<std::string> sorted = out.classOrder;
                std::sort(sorted.begin(), sorted.end());
                if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end())
                    fail(at, "names a class twice, expected each road class once");
                if (out.classOrder.empty()) fail(at, "is empty, expected the road classes, the most important first");
            }
            lua_settop(L, p);
        }
        {
            std::vector<Number> limits;
            readNumbers(p, kKeySpeedLimits, false, limits, path);
            checkDefaultLimitsAreSpeeds(limits, path + "." + kKeySpeedLimits);
            out.speedLimits.clear();
            for (const Number& n : limits) out.speedLimits.emplace_back(n.name, n.value);
        }
        readRows(p, path + "." + kKeyJunctions, rules, out.rows);
        lua_settop(L, base);
        return clean();
    }

private:
    // { min = n, max = n } of plain counts at `idx`.
    void countRange(int idx, const std::string& path, engine::roads::lanes::FactRange& out) {
        const int top = lua_gettop(L);
        if (!lua_istable(L, idx)) {
            fail(path, "is " + describe(idx) + ", expected { " + kKeyMin + " = n, " + kKeyMax + " = n }");
            return;
        }
        checkKeys(idx, path, {kKeyMin, kKeyMax});
        out.set = true;
        for (const char* key : {kKeyMin, kKeyMax}) {
            const int v = pushField(idx, key);
            if (lua_type(L, v) == LUA_TNUMBER && std::isfinite(lua_tonumber(L, v)))
                (std::string(key) == kKeyMin ? out.lo : out.hi) = lua_tonumber(L, v);
            else if (!lua_isnil(L, v))
                fail(path + "." + key, "is " + describe(v) + ", expected a count");
            lua_settop(L, top);
        }
    }

    // { value, unit = "...", source = "..." } at `idx`, of the family `family`, into `value` (converted).
    bool unitNumber(int idx, const std::string& path, UnitFamily family, double& value) {
        const int top = lua_gettop(L);
        if (!lua_istable(L, idx)) {
            fail(path, "is " + describe(idx) + ", expected { value, unit = \"...\", source = \"...\" } (" +
                           unitFamilyWords(family) + ")");
            return false;
        }
        checkKeys(idx, path, {kKeyUnit, kKeySource}, {std::to_string(kNumberValueSlot)});
        std::string unit;
        double factor = 0;
        bool ok = readUnit(idx, path, unit, factor);
        UnitFamily got = UnitFamily::Ratio;
        if (ok && unitFamily(unit, got) && got != family) {
            fail(path + "." + kKeyUnit, "is \"" + unit + "\", which is " + unitFamilyWords(got) + "; expected " +
                                            unitFamilyWords(family));
            ok = false;
        }
        lua_rawgeti(L, idx, kNumberValueSlot);
        if (lua_type(L, -1) == LUA_TNUMBER && std::isfinite(lua_tonumber(L, -1))) {
            if (ok) value = lua_tonumber(L, -1) * factor;
        } else {
            fail(path + "[" + std::to_string(kNumberValueSlot) + "]", "is " + describe(-1) + ", expected the number itself");
            ok = false;
        }
        lua_settop(L, top);
        return ok;
    }

    // { min = { value, unit = ... }, max = { ... } } of speeds at `idx`.
    void speedRange(int idx, const std::string& path, engine::roads::lanes::FactRange& out) {
        const int top = lua_gettop(L);
        if (!lua_istable(L, idx)) {
            fail(path, "is " + describe(idx) + ", expected { " + kKeyMin + " = { value, unit = ... }, " + kKeyMax + " = ... }");
            return;
        }
        checkKeys(idx, path, {kKeyMin, kKeyMax});
        out.set = true;
        for (const char* key : {kKeyMin, kKeyMax}) {
            const int v = pushField(idx, key);
            if (!lua_isnil(L, v)) unitNumber(v, path + "." + key, UnitFamily::Speed, std::string(key) == kKeyMin ? out.lo : out.hi);
            lua_settop(L, top);
        }
    }

    void yesNo(int idx, const std::string& path, engine::roads::lanes::YesNo& out) {
        if (lua_type(L, idx) == LUA_TBOOLEAN)
            out = lua_toboolean(L, idx) ? engine::roads::lanes::YesNo::Yes : engine::roads::lanes::YesNo::No;
        else
            fail(path, "is " + describe(idx) + ", expected true or false");
    }

    void readRows(int placement, const std::string& path, const RuleSet& rules, std::vector<engine::roads::lanes::PolicyRow>& out) {
        const int top = lua_gettop(L);
        const int list = pushField(placement, kKeyJunctions);
        if (!lua_istable(L, list)) {
            fail(path, "is " + describe(list) + ", expected a list of rows { name, when, control, signs }");
            lua_settop(L, top);
            return;
        }
        std::vector<std::string> names, odd;
        keysOf(list, names, odd);
        if (!names.empty()) fail(path, "has the key \"" + names[0] + "\", expected a list of rows");
        const lua_Integer n = static_cast<lua_Integer>(lua_rawlen(L, list));
        if (n == 0) fail(path, "is empty, expected at least one row");
        std::vector<engine::roads::lanes::PolicyRow> rows;
        bool lastHoldsForAll = false;
        for (lua_Integer i = 1; i <= n; ++i) {
            const std::string rowPath = path + "[" + std::to_string(i) + "]";
            lua_rawgeti(L, list, i);
            const int rt = lua_gettop(L);
            if (!lua_istable(L, rt)) {
                fail(rowPath, "is " + describe(rt) + ", expected a row { name, when, control, signs }");
                lua_settop(L, list);
                continue;
            }
            checkKeys(rt, rowPath, {kKeyName, kKeyWhen, kKeyApart, kKeyControl, kKeySigns});
            engine::roads::lanes::PolicyRow row;
            {
                const int v = pushField(rt, kKeyName);
                if (!stringAt(v, row.name) || row.name.empty()) fail(rowPath + "." + kKeyName, "is " + describe(v) + ", expected what the row is for, in words");
                lua_settop(L, rt);
            }
            {
                const int v = pushField(rt, kKeyControl);
                if (!stringAt(v, row.control)) fail(rowPath + "." + kKeyControl, "is " + describe(v) + ", expected a control's name");
                else if (!rules.findControl(row.control))
                    fail(rowPath + "." + kKeyControl, "names \"" + row.control + "\", which " + kRoot + "." + kKeyControl + " has no row for");
                lua_settop(L, rt);
            }
            bool asks = false;
            {
                const int w = pushField(rt, kKeyWhen);
                const std::string whenPath = rowPath + "." + kKeyWhen;
                if (lua_istable(L, w)) {
                    const std::vector<std::string> given =
                        checkKeys(w, whenPath, {kFactArms, kFactApproaches, kFactMajor, kFactMinor, kFactHasMinor, kFactRampEnd,
                                                kFactMajorLanes, kFactMinorLanes, kFactMajorLimit, kFactMinorLimit});
                    asks = !given.empty();
                    auto field = [&](const char* key) { return pushField(w, key); };
                    int v = field(kFactArms);
                    if (!lua_isnil(L, v)) countRange(v, whenPath + "." + kFactArms, row.arms);
                    lua_settop(L, w);
                    v = field(kFactApproaches);
                    if (!lua_isnil(L, v)) countRange(v, whenPath + "." + kFactApproaches, row.approaches);
                    lua_settop(L, w);
                    v = field(kFactMajor);
                    if (!lua_isnil(L, v)) stringList(v, whenPath + "." + kFactMajor, true, row.major);
                    lua_settop(L, w);
                    v = field(kFactMinor);
                    if (!lua_isnil(L, v)) stringList(v, whenPath + "." + kFactMinor, true, row.minor);
                    lua_settop(L, w);
                    v = field(kFactHasMinor);
                    if (!lua_isnil(L, v)) yesNo(v, whenPath + "." + kFactHasMinor, row.hasMinor);
                    lua_settop(L, w);
                    v = field(kFactRampEnd);
                    if (!lua_isnil(L, v)) yesNo(v, whenPath + "." + kFactRampEnd, row.rampEnd);
                    lua_settop(L, w);
                    v = field(kFactMajorLanes);
                    if (!lua_isnil(L, v)) countRange(v, whenPath + "." + kFactMajorLanes, row.majorLanes);
                    lua_settop(L, w);
                    v = field(kFactMinorLanes);
                    if (!lua_isnil(L, v)) countRange(v, whenPath + "." + kFactMinorLanes, row.minorLanes);
                    lua_settop(L, w);
                    v = field(kFactMajorLimit);
                    if (!lua_isnil(L, v)) speedRange(v, whenPath + "." + kFactMajorLimit, row.majorLimit);
                    lua_settop(L, w);
                    v = field(kFactMinorLimit);
                    if (!lua_isnil(L, v)) speedRange(v, whenPath + "." + kFactMinorLimit, row.minorLimit);
                    lua_settop(L, w);
                } else if (!lua_isnil(L, w)) {
                    fail(whenPath, "is " + describe(w) + ", expected a table of conditions");
                }
                lua_settop(L, rt);
            }
            {
                const int v = pushField(rt, kKeyApart);
                if (!lua_isnil(L, v)) {
                    if (unitNumber(v, rowPath + "." + kKeyApart, UnitFamily::Length, row.apart)) asks = true;
                }
                lua_settop(L, rt);
            }
            {
                const int s = pushField(rt, kKeySigns);
                const std::string signsPath = rowPath + "." + kKeySigns;
                if (lua_istable(L, s)) {
                    checkKeys(s, signsPath, {kFactMajor, kFactMinor, kSignsAll});
                    for (const char* key : {kSignsAll, kFactMajor, kFactMinor}) {
                        const int v = pushField(s, key);
                        std::vector<std::string> kinds;
                        if (!lua_isnil(L, v) && stringList(v, signsPath + "." + key, true, kinds)) {
                            for (const std::string& k : kinds)
                                if (!rules.findDevice(k))
                                    fail(signsPath + "." + key, "names \"" + k + "\", which " + kRoot + "." + kKeyDevices +
                                                                     " has no device for");
                            const std::string which = key;
                            if (which != kFactMinor) row.majorSigns.insert(row.majorSigns.end(), kinds.begin(), kinds.end());
                            if (which != kFactMajor) row.minorSigns.insert(row.minorSigns.end(), kinds.begin(), kinds.end());
                        }
                        lua_settop(L, s);
                    }
                } else if (!lua_isnil(L, s)) {
                    fail(signsPath, "is " + describe(s) + ", expected { " + kFactMajor + " = ..., " + kFactMinor + " = ..., " +
                                        kSignsAll + " = ... }");
                }
                lua_settop(L, rt);
            }
            lastHoldsForAll = !asks;
            rows.push_back(row);
            lua_settop(L, list);
        }
        if (n > 0 && !lastHoldsForAll)
            fail(path + "[" + std::to_string(n) + "]", "asks something (" + std::string(kKeyWhen) + " or " + kKeyApart +
                                                           "); the last row must hold for every junction, so none is left "
                                                           "without a control");
        out = rows;
        lua_settop(L, top);
    }
};

}  // namespace

bool loadPlacementPolicy(engine::ScriptVM& vm, const std::string& fileName, const RuleSet& rules,
                         engine::roads::lanes::ControlPolicy& out, std::string* err) {
    lua_State* L = engine::luaState(vm);
    PlacementReader reader(L, fileName);
    engine::roads::lanes::ControlPolicy scratch;
    if (!reader.readPlacement(rules, scratch)) {
        if (err) *err = reader.report();
        return false;
    }
    out = scratch;
    return true;
}

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
