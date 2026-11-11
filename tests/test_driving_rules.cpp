// The driving library's rule-set reader (#250): the example rule set loads field by field, units are converted to
// metres and seconds, every kind of fault is refused with a message naming the file and the key path, and one person's
// own value of a driver setting is stable, in range, bunched around the usual value, and independent between settings.
// Rule sets here are small Lua strings; no real rule set is read.

#include "test_framework.h"

#include "apps/driving/rule_set.h"

#ifdef RT_ENABLE_SCRIPTING

#include "apps/driving/scripting/rule_set_lua.h"
#include "engine/scripting/lua_state.h"
#include "engine/scripting/script_vm.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

using namespace driving;

namespace {

constexpr const char* kFile = "us_left.lua";

const char* kExample = R"LUA(
rule_set = {
  name = "us_left",
  drives_on = "left",
  ranks = { "priority", "give_way", "stop" },
  devices = {
    stop        = { means = { stop_line = true, rank = "stop" },   sign = "R1-1", paint = { "stop_bar", "STOP" } },
    give_way    = { means = { rank = "give_way" },                 sign = "R1-2", paint = { "give_way_teeth" } },
    speed_limit = { means = { top_speed = "value" },               sign = "R2-1" },
    no_entry    = { means = { closed_to = "all" },                 sign = "R5-1" },
    bus_lane    = { means = { open_to = { "bus", "taxi" } },       paint = { "BUS ONLY" } },
    no_turn_on_red = { means = { forbids = "turn_on_red" },        sign = "R10-11a" },
  },
  numbers = {
    far_turn_gap      = { 4.1, unit = "s",    source = "HCM 2010 exhibit 19-10" },
    default_limit     = { 25,  unit = "mph",  source = "UVC 11-801" },
    comfortable_brake = { 10,  unit = "ft/s2", source = "ITE" },
    patience_floor    = { 0.6, unit = "ratio" },
  },
  personality = {
    gap_kept     = { usual = 1.5, low = 0.8, high = 2.5, unit = "s" },
    speed_habit  = { usual = 1.0, low = 0.9, high = 1.15, unit = "ratio" },
  },
}
)LUA";

// A whole rule set around the parts a test wants to vary.
std::string withParts(const std::string& driveson, const std::string& devices, const std::string& numbers,
                      const std::string& personality) {
    return "rule_set = { name = 'x', drives_on = " + driveson + ", ranks = { 'priority', 'give_way', 'stop' }, " +
           "devices = { " + devices + " }, numbers = { " + numbers + " }, personality = { " + personality + " } }";
}

struct Loaded {
    bool ok = false;
    std::string err;
    int stackBefore = 0;
    int stackAfter = 0;
    RuleSet rules;
};

Loaded load(const std::string& lua) {
    engine::ScriptVM vm;
    std::string luaErr;
    CHECK(vm.doString(lua, &luaErr));
    Loaded r;
    lua_State* L = engine::luaState(vm);
    r.stackBefore = lua_gettop(L);
    r.ok = loadRuleSet(vm, kFile, r.rules, &r.err);
    r.stackAfter = lua_gettop(L);
    return r;
}

bool has(const std::string& text, const std::string& piece) { return text.find(piece) != std::string::npos; }

// A refused file: no success, the message names the file and the key path, and `out` was not touched.
void expectRefused(const std::string& lua, const std::string& path, const std::string& piece) {
    engine::ScriptVM vm;
    CHECK(vm.doString(lua));
    RuleSet out;
    out.name = "untouched";
    std::string err;
    lua_State* L = engine::luaState(vm);
    const int before = lua_gettop(L);
    CHECK(!loadRuleSet(vm, kFile, out, &err));
    CHECK(lua_gettop(L) == before);
    CHECK(out.name == "untouched");
    CHECK(out.devices.empty());
    if (!has(err, std::string(kFile) + ": " + path) || !has(err, piece)) {
        std::printf("    wanted '%s' and '%s' in: %s\n", path.c_str(), piece.c_str(), err.c_str());
        CHECK(false);
    }
}

}  // namespace

TEST_CASE(rules_example_loads_every_field) {
    const Loaded a = load(kExample);
    CHECK(a.ok);
    CHECK(a.err.empty());
    const RuleSet& r = a.rules;
    CHECK(r.name == "us_left");
    CHECK(r.drivesOn == DrivesOn::Left);
    CHECK(r.ranks.size() == 3u);
    if (r.ranks.size() == 3u) CHECK(r.ranks[0] == "priority" && r.ranks[1] == "give_way" && r.ranks[2] == "stop");
    CHECK(r.rankIndex("stop") == 2);
    CHECK(r.rankIndex("nonsense") == kNoRank);

    // Devices come out sorted by name.
    CHECK(r.devices.size() == 6u);
    const char* sorted[] = {"bus_lane", "give_way", "no_entry", "no_turn_on_red", "speed_limit", "stop"};
    for (std::size_t i = 0; i < r.devices.size() && i < 6u; ++i) CHECK(r.devices[i].name == sorted[i]);

    const Device* stop = r.findDevice("stop");
    CHECK(stop != nullptr);
    if (stop) {
        CHECK(stop->stopLine);
        CHECK(stop->rank == 2);
        CHECK(stop->sign == "R1-1");
        CHECK(stop->paint.size() == 2u && stop->paint[0] == "stop_bar" && stop->paint[1] == "STOP");
        CHECK(!stop->topSpeedIsValue);
        CHECK(stop->closedTo.kind == VehicleScope::Kind::Unsaid);
    }
    const Device* giveWay = r.findDevice("give_way");
    CHECK(giveWay && giveWay->rank == 1 && !giveWay->stopLine);
    const Device* limit = r.findDevice("speed_limit");
    CHECK(limit && limit->topSpeedIsValue && limit->rank == kNoRank && limit->paint.empty());
    const Device* noEntry = r.findDevice("no_entry");
    CHECK(noEntry && noEntry->closedTo.kind == VehicleScope::Kind::All && noEntry->openTo.kind == VehicleScope::Kind::Unsaid);
    const Device* bus = r.findDevice("bus_lane");
    CHECK(bus != nullptr);
    if (bus) {
        CHECK(bus->openTo.kind == VehicleScope::Kind::Listed);
        CHECK(bus->openTo.tags.size() == 2u && bus->openTo.tags[0] == "bus" && bus->openTo.tags[1] == "taxi");
        CHECK(bus->sign.empty());
    }
    const Device* red = r.findDevice("no_turn_on_red");
    CHECK(red && red->forbids.size() == 1u && red->forbids[0] == "turn_on_red");
    CHECK(r.findDevice("nope") == nullptr);

    CHECK(r.numbers.size() == 4u);
    const Number* gap = r.findNumber("far_turn_gap");
    CHECK(gap != nullptr);
    if (gap) {
        CHECK_APPROX(gap->value, 4.1, 1e-9);
        CHECK(gap->unit == "s" && gap->hasSource && gap->source == "HCM 2010 exhibit 19-10");
    }
    const Number* limitNumber = r.findNumber("default_limit");
    CHECK(limitNumber != nullptr);
    if (limitNumber) {
        CHECK_APPROX(limitNumber->value, 11.176, 1e-6);
        CHECK_APPROX(limitNumber->writtenValue, 25.0, 1e-12);
        CHECK(limitNumber->unit == "mph");
    }
    const Number* brake = r.findNumber("comfortable_brake");
    CHECK(brake != nullptr);
    if (brake) {
        CHECK_APPROX(brake->value, 3.048, 1e-6);
        CHECK_APPROX(brake->writtenValue, 10.0, 1e-12);
        CHECK(brake->unit == "ft/s2");
    }
    const Number* floorRatio = r.findNumber("patience_floor");
    CHECK(floorRatio && !floorRatio->hasSource && floorRatio->source.empty());
    CHECK(r.unsourcedNumberCount() == 1u);
    CHECK(r.findNumber("nope") == nullptr);

    CHECK(r.personality.size() == 2u);
    const PersonalSetting* gapKept = r.findPersonal("gap_kept");
    CHECK(gapKept != nullptr);
    if (gapKept) {
        CHECK_APPROX(gapKept->usual, 1.5, 1e-12);
        CHECK_APPROX(gapKept->low, 0.8, 1e-12);
        CHECK_APPROX(gapKept->high, 2.5, 1e-12);
        CHECK(gapKept->unit == "s");
    }
    CHECK(r.findPersonal("speed_habit") != nullptr);
    CHECK(r.findPersonal("nope") == nullptr);

    // The same file loads to the same lists, in the same order, every time; the stack is back where it was.
    const Loaded b = load(kExample);
    CHECK(b.ok);
    CHECK(b.rules.devices.size() == r.devices.size());
    for (std::size_t i = 0; i < r.devices.size() && i < b.rules.devices.size(); ++i)
        CHECK(r.devices[i].name == b.rules.devices[i].name);
    for (std::size_t i = 0; i < r.numbers.size() && i < b.rules.numbers.size(); ++i)
        CHECK(r.numbers[i].name == b.rules.numbers[i].name && r.numbers[i].value == b.rules.numbers[i].value);
    CHECK(a.stackBefore == a.stackAfter);
}

TEST_CASE(rules_units_convert_to_metres_and_seconds) {
    CHECK(unitWords() == "s, m, ft, mph, km/h, m/s, m/s2, ft/s2, ratio");
    double f = 0;
    CHECK(unitFactor("km/h", f));
    CHECK_APPROX(f * 36.0, 10.0, 1e-9);
    CHECK(unitFactor("ft", f));
    CHECK_APPROX(f, 0.3048, 1e-12);
    CHECK(!unitFactor("furlong", f));
}

TEST_CASE(rules_refused_with_file_and_key_path) {
    const std::string okDevice = "stop = { means = { rank = 'stop' } }";
    const std::string okNumber = "a = { 1, unit = 's' }";
    const std::string okSetting = "g = { usual = 1, low = 0, high = 2, unit = 's' }";

    expectRefused(withParts("'left'", okDevice, "a = { 1, unit = 'parsecs' }", okSetting), "rule_set.numbers.a.unit",
                  "parsecs");
    expectRefused(withParts("'left'", okDevice, "a = { 4.1 }", okSetting), "rule_set.numbers.a.unit", "missing");
    expectRefused(withParts("'left'", "stop = { means = { rank = 'halt' } }", okNumber, okSetting),
                  "rule_set.devices.stop.means.rank", "is \"halt\", which is not one of ranks (priority, give_way, stop)");
    expectRefused(withParts("'left'", "stop = { means = { colour = 'red' } }", okNumber, okSetting),
                  "rule_set.devices.stop.means", "colour");
    expectRefused(withParts("'left'", "b = { means = { open_to = { 3, 4 } } }", okNumber, okSetting),
                  "rule_set.devices.b.means.open_to[1]", "expected a string");
    expectRefused(withParts("'left'", "b = { means = { open_to = 'some' } }", okNumber, okSetting),
                  "rule_set.devices.b.means.open_to", "\"all\"");
    expectRefused(withParts("'left'", "b = { means = { open_to = 5 } }", okNumber, okSetting),
                  "rule_set.devices.b.means.open_to", "expected a list");
    expectRefused(withParts("'left'", okDevice, okNumber, "g = { usual = 1, low = 3, high = 4, unit = 's' }"),
                  "rule_set.personality.g", "low <= usual <= high");
    expectRefused(withParts("'middle'", okDevice, okNumber, okSetting), "rule_set.drives_on", "\"middle\"");
    expectRefused(withParts("'left'", "stop = { sing = 'R1-1', means = {} }", okNumber, okSetting),
                  "rule_set.devices.stop", "\"sing\"");
    expectRefused(withParts("'left'", okDevice, "a = { 1, unit = 's', sauce = 'x' }", okSetting),
                  "rule_set.numbers.a", "\"sauce\"");
    expectRefused("x = 1", "rule_set", "global `rule_set`");
    expectRefused("rule_set = 5", "rule_set", "expected a table");
}

TEST_CASE(rules_three_faults_are_all_reported) {
    const std::string lua =
        withParts("'middle'", "stop = { means = { rank = 'halt' } }", "a = { 1, unit = 'parsecs' }", "");
    engine::ScriptVM vm;
    CHECK(vm.doString(lua));
    RuleSet out;
    std::string err;
    CHECK(!loadRuleSet(vm, kFile, out, &err));
    CHECK(has(err, "rule_set.drives_on"));
    CHECK(has(err, "rule_set.devices.stop.means.rank"));
    CHECK(has(err, "rule_set.numbers.a.unit"));
    std::size_t lines = 1;
    for (std::size_t i = 0; i < err.size(); ++i)
        if (err[i] == '\n') ++lines;
    CHECK(lines == 3u);

    // A file with more faults than the limit lists the limit's worth and says there were more.
    std::string many;
    for (std::size_t i = 0; i < kMaxRuleSetErrors + 5; ++i) many += "n" + std::to_string(i) + " = { 1 }, ";
    engine::ScriptVM vm2;
    CHECK(vm2.doString(withParts("'left'", "", many, "")));
    std::string err2;
    CHECK(!loadRuleSet(vm2, kFile, out, &err2));
    std::size_t lines2 = 1;
    for (std::size_t i = 0; i < err2.size(); ++i)
        if (err2[i] == '\n') ++lines2;
    CHECK(lines2 == kMaxRuleSetErrors + 1);
    CHECK(has(err2, "more faults"));
}

TEST_CASE(rules_stack_unchanged_and_syntax_error_keeps_lua_line) {
    const Loaded good = load(kExample);
    CHECK(good.stackBefore == good.stackAfter);
    const Loaded bad = load(withParts("'middle'", "", "a = { 1 }", ""));
    CHECK(!bad.ok);
    CHECK(bad.stackBefore == bad.stackAfter);

    engine::ScriptVM vm;
    std::string luaErr;
    CHECK(!vm.doString("rule_set = {\n  name = \n}\nrule_set = = 3", &luaErr));
    CHECK(has(luaErr, ":3:"));   // Lua's own message keeps its line
}

TEST_CASE(rules_personal_value_is_stable_in_range_and_bunched) {
    PersonalSetting gap;
    gap.name = "gap_kept";
    gap.low = 0.8;
    gap.usual = 1.5;
    gap.high = 2.5;
    PersonalSetting habit;
    habit.name = "speed_habit";
    habit.low = 0.9;
    habit.usual = 1.0;
    habit.high = 1.15;

    constexpr uint64_t kPeople = 10000;
    constexpr uint64_t kBigIdBit = 1ull << 40;   // an id past 32 bits still counts
    CHECK(personalValue(gap, 42) == personalValue(gap, 42));
    CHECK(personalValue(gap, kBigIdBit + 42) == personalValue(gap, kBigIdBit + 42));
    CHECK(personalValue(gap, 42) != personalValue(gap, kBigIdBit + 42));

    std::vector<double> a;
    std::vector<double> b;
    bool allInRange = true;
    double sumA = 0;
    for (uint64_t id = 0; id < kPeople; ++id) {
        const double va = personalValue(gap, id);
        const double vb = personalValue(habit, id);
        if (va < gap.low || va > gap.high || vb < habit.low || vb > habit.high) allInRange = false;
        a.push_back(va);
        b.push_back(vb);
        sumA += va;
    }
    CHECK(allInRange);
    const double expectedMean = (gap.low + gap.usual + gap.high) / 3.0;
    CHECK(std::fabs(sumA / static_cast<double>(kPeople) - expectedMean) < 0.02 * expectedMean);

    double meanA = 0, meanB = 0;
    for (uint64_t i = 0; i < kPeople; ++i) {
        meanA += a[i];
        meanB += b[i];
    }
    meanA /= static_cast<double>(kPeople);
    meanB /= static_cast<double>(kPeople);
    double cov = 0, varA = 0, varB = 0;
    for (uint64_t i = 0; i < kPeople; ++i) {
        cov += (a[i] - meanA) * (b[i] - meanB);
        varA += (a[i] - meanA) * (a[i] - meanA);
        varB += (b[i] - meanB) * (b[i] - meanB);
    }
    CHECK(std::fabs(cov / std::sqrt(varA * varB)) < 0.05);

    // No range: everyone has the one value.
    PersonalSetting flat;
    flat.name = "flat";
    flat.low = flat.usual = flat.high = 3.0;
    CHECK(personalValue(flat, 7) == 3.0);
}

#else   // RT_ENABLE_SCRIPTING

TEST_CASE(rules_personal_value_needs_no_lua) {
    driving::PersonalSetting s;
    s.name = "x";
    s.low = 0;
    s.usual = 1;
    s.high = 2;
    const double v = driving::personalValue(s, 5);
    CHECK(v >= 0 && v <= 2);
}

#endif
