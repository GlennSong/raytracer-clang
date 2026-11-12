#include "apps/driving/scripting/rules_lua.h"

#include "apps/driving/scripting/lua_table_reader.h"
#include "engine/scripting/lua_state.h"
#include "engine/scripting/script_vm.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace driving {

namespace {

// The global the rule-set file defines, and the root of every key path in a message.
constexpr const char* kRoot = "rule_set";

// The words of the rule language this reader checks for.
constexpr const char* kKeyRules = "rules";
constexpr const char* kKeyName = "name";
constexpr const char* kKeyWhen = "when";
constexpr const char* kKeyUnless = "unless";
constexpr const char* kKeyGive = "give";
constexpr const char* kKeyAll = "all";
constexpr const char* kKeyAny = "any";
constexpr const char* kWordNot = "not";
constexpr const char* kKeyNumbers = "numbers";
constexpr const char* kKeyPersonality = "personality";

// The shapes of a list condition: { "not", condition } and { fact, op, value }, and where each part sits.
constexpr lua_Integer kNotLength = 2;
constexpr lua_Integer kComparisonLength = 3;
constexpr lua_Integer kFirst = 1;
constexpr lua_Integer kSecond = 2;
constexpr lua_Integer kThird = 3;

const char* kConditionForms =
    "a fact's name, { \"not\", condition }, { all = { ... } }, { any = { ... } } or { fact, op, value }";

struct OpWord {
    const char* word;
    CompareOp op;
};
const OpWord kOps[] = {
    {"<", CompareOp::Less},     {"<=", CompareOp::LessOrEqual}, {">", CompareOp::Greater},
    {">=", CompareOp::GreaterOrEqual}, {"==", CompareOp::Equal}, {"~=", CompareOp::NotEqual},
    {"is", CompareOp::Is},      {"is_not", CompareOp::IsNot},
};

std::string opWords() {
    std::string out;
    for (const OpWord& o : kOps) out += std::string(out.empty() ? "" : ", ") + o.word;
    return out;
}

bool comparesNumbers(CompareOp op) { return op != CompareOp::Is && op != CompareOp::IsNot; }

const char* factKindWords(FactKind kind) {
    switch (kind) {
    case FactKind::Number: return "a number fact";
    case FactKind::YesNo: return "a yes/no fact";
    case FactKind::Word: return "a word fact";
    case FactKind::Place: return "a place fact";
    }
    return "a fact";
}

std::string indexed(const std::string& path, lua_Integer i) { return path + "[" + std::to_string(i) + "]"; }

// What a unit family is, with the unit it is stored in: "a length (m)".
std::string unitText(UnitFamily unit) {
    return std::string(unitFamilyWords(unit)) + " (" + unitFamilyStoredUnit(unit) + ")";
}

// Reads `rule_set.rules` and compiles it.
class RulesReader : public LuaTableReader {
public:
    RulesReader(lua_State* state, const std::string& file, const FactRegistry& facts, const RuleSet& ruleSet,
                const AnswerVocabulary& vocabulary)
        : LuaTableReader(state, file, kMaxRulesErrors), facts_(facts), ruleSet_(ruleSet), vocab_(vocabulary) {}

    bool read(CompiledRules& out) {
        const int base = lua_gettop(L);
        lua_getglobal(L, kRoot);
        if (!lua_istable(L, -1)) {
            fail(kRoot, "is " + describe(-1) + ", expected a table: the rule-set file must define a global `rule_set`");
            lua_settop(L, base);
            return false;
        }
        const int root = lua_gettop(L);
        checkNameClashes();
        built_.program.intentionKindCount = static_cast<uint32_t>(vocab_.intentions().size());
        built_.program.actionCount = static_cast<uint32_t>(vocab_.actions().size());

        const std::string path = std::string(kRoot) + "." + kKeyRules;
        const int list = pushField(root, kKeyRules);
        if (!lua_istable(L, list)) {
            fail(path, "is " + describe(list) + ", expected a list of rules");
        } else {
            const lua_Integer n = static_cast<lua_Integer>(lua_rawlen(L, list));
            checkListKeys(list, path, n, "a list of rules");
            for (lua_Integer i = 1; i <= n; ++i) {
                const int mark = lua_gettop(L);
                lua_rawgeti(L, list, i);
                readRule(lua_gettop(L), indexed(path, i));
                lua_settop(L, mark);
            }
        }
        lua_settop(L, base);
        if (!clean()) return false;

        for (const NamedRead& r : built_.program.reads)
            if (r.value.source == ValueSource::Fact) built_.usedFacts.push_back(r.value.index);
        std::sort(built_.usedFacts.begin(), built_.usedFacts.end());
        built_.usedFacts.erase(std::unique(built_.usedFacts.begin(), built_.usedFacts.end()), built_.usedFacts.end());
        out = std::move(built_);
        return true;
    }

private:
    const FactRegistry& facts_;
    const RuleSet& ruleSet_;
    const AnswerVocabulary& vocab_;
    CompiledRules built_;
    std::vector<std::pair<std::string, std::string>> seenRules_;   // each rule's name and the path it was written at

    // While compiling one rule: where its reads start, and how many yes/no values its steps hold at once.
    std::size_t readStart_ = 0;
    std::size_t stackNow_ = 0;
    std::size_t stackPeak_ = 0;

    // ---- Names ----------------------------------------------------------------------------------------------------

    // A fact, a rule-set number and a personal setting may not share a name.
    void checkNameClashes() {
        const std::string why = "; a fact, a rule-set number and a personal setting may not share a name";
        for (const FactSpec& f : facts_.all()) {
            if (ruleSet_.findNumber(f.name))
                fail(std::string(kRoot) + "." + kKeyNumbers + "." + f.name,
                     "has the same name as the fact \"" + f.name + "\"" + why);
            if (ruleSet_.findPersonal(f.name))
                fail(std::string(kRoot) + "." + kKeyPersonality + "." + f.name,
                     "has the same name as the fact \"" + f.name + "\"" + why);
        }
        for (const Number& n : ruleSet_.numbers)
            if (ruleSet_.findPersonal(n.name))
                fail(std::string(kRoot) + "." + kKeyPersonality + "." + n.name,
                     std::string("has the same name as the number ") + kRoot + "." + kKeyNumbers + "." + n.name + why);
    }

    // What a name stands for, as a read: a fact, a rule-set number (its value now) or a personal setting. False when
    // it is none of them.
    bool lookUp(const std::string& name, NamedRead& read) {
        const FactSlot slot = facts_.find(name);
        if (slot != kNoFact) {
            const FactSpec& f = facts_.at(slot);
            read.value.source = ValueSource::Fact;
            read.value.index = slot;
            read.factKind = f.kind;
            read.unit = f.kind == FactKind::Place ? UnitFamily::Length : f.unit;
            return true;
        }
        const Number* n = ruleSet_.findNumber(name);
        if (n) {
            read.value.source = ValueSource::Written;
            read.value.index = static_cast<uint32_t>(n - ruleSet_.numbers.data());
            read.value.value = static_cast<Real>(n->value);
            if (!unitFamily(n->unit, read.unit)) read.unit = UnitFamily::Ratio;
            return true;
        }
        const PersonalSetting* p = ruleSet_.findPersonal(name);
        if (p) {
            read.value.source = ValueSource::Setting;
            read.value.index = static_cast<uint32_t>(p - ruleSet_.personality.data());
            read.value.nameHash = settingNameHash(name);
            if (!unitFamily(p->unit, read.unit)) read.unit = UnitFamily::Ratio;
            return true;
        }
        return false;
    }

    // "the fact \"gap_ahead\"", "the number \"horn_gap\"", "the setting \"speed_habit\"".
    static std::string thing(const NamedRead& read, const std::string& name) {
        switch (read.value.source) {
        case ValueSource::Fact: return "the " + std::string(factKindWords(read.factKind)).substr(2) + " \"" + name + "\"";
        case ValueSource::Written: return "the rule-set number \"" + name + "\"";
        case ValueSource::Setting: return "the personal setting \"" + name + "\"";
        case ValueSource::Word: break;
        }
        return "\"" + name + "\"";
    }

    // Keeps a read for the rule being compiled (each once), so its value can be shown with the answer.
    void addRead(const NamedRead& read, const std::string& name) {
        std::vector<NamedRead>& reads = built_.program.reads;
        for (std::size_t i = readStart_; i < reads.size(); ++i)
            if (reads[i].value.source == read.value.source && reads[i].value.index == read.value.index) return;
        reads.push_back(read);
        built_.labels.reads.push_back(name);
        std::vector<std::string> words;
        if (read.value.source == ValueSource::Fact && read.factKind == FactKind::Word)
            words = facts_.at(read.value.index).words;
        built_.labels.readWords.push_back(words);
        // A rule-set number as the file wrote it, when that is not how it is stored ("45 mph").
        std::string written;
        if (read.value.source == ValueSource::Written && read.value.index < ruleSet_.numbers.size()) {
            const Number& n = ruleSet_.numbers[read.value.index];
            if (n.unit != unitFamilyStoredUnit(read.unit) || n.writtenValue != n.value)
                written = numberText(n.writtenValue) + " " + n.unit;
        }
        built_.labels.readWritten.push_back(written);
    }
    void addFactRead(FactSlot slot) {
        NamedRead read;
        lookUp(facts_.at(slot).name, read);
        addRead(read, facts_.at(slot).name);
    }

    // A number at `idx`: written out (when `writtenOk`), or the name of a number fact, rule-set number or setting
    // measuring `want`. `wantWhy` ends the message when the units disagree.
    bool numberValue(int idx, const std::string& path, UnitFamily want, const std::string& wantWhy, bool writtenOk,
                     ValueRef& out) {
        if (writtenOk && lua_type(L, idx) == LUA_TNUMBER && std::isfinite(lua_tonumber(L, idx))) {
            out = ValueRef();
            out.source = ValueSource::Written;
            out.value = static_cast<Real>(lua_tonumber(L, idx));
            return true;
        }
        const std::string expected = std::string(writtenOk ? "a number, or " : "") +
                                     "the name of a number fact, a rule-set number or a personal setting";
        std::string name;
        if (!stringAt(idx, name)) {
            fail(path, "is " + describe(idx) + ", expected " + expected);
            return false;
        }
        NamedRead read;
        if (!lookUp(name, read)) {
            fail(path, "is \"" + name + "\", which is not a fact, a rule-set number or a personal setting");
            return false;
        }
        if (read.value.source == ValueSource::Fact && read.factKind != FactKind::Number &&
            read.factKind != FactKind::Place) {
            fail(path, "is " + thing(read, name) + ", expected " + expected);
            return false;
        }
        if (read.unit != want) {
            fail(path, "is " + thing(read, name) + ", which is " + unitText(read.unit) + ", but " + wantWhy);
            return false;
        }
        addRead(read, name);
        out = read.value;
        return true;
    }

    // ---- Steps ------------------------------------------------------------------------------------------------------

    void emit(const Step& s) {
        built_.program.steps.push_back(s);
        if (s.op == StepOp::FactHolds || s.op == StepOp::Compare) {
            ++stackNow_;
            stackPeak_ = std::max(stackPeak_, stackNow_);
        } else if ((s.op == StepOp::And || s.op == StepOp::Or) && stackNow_ > 0) {
            --stackNow_;
        }
    }
    void emitOp(StepOp op) {
        Step s;
        s.op = op;
        emit(s);
    }

    // A list's keys must be 1..n and nothing else.
    void checkListKeys(int t, const std::string& path, lua_Integer n, const std::string& expected) {
        std::vector<std::string> names;
        std::vector<std::string> odd;
        keysOf(t, names, odd);
        for (const std::string& k : names) fail(path, "has the key \"" + k + "\", expected only " + expected);
        std::vector<std::string> allowed;
        for (lua_Integer i = 1; i <= n; ++i) allowed.push_back(std::to_string(i));
        for (const std::string& o : odd)
            if (std::find(allowed.begin(), allowed.end(), o) == allowed.end())
                fail(path, "has the key " + o + ", expected only " + expected);
    }

    // ---- Conditions -------------------------------------------------------------------------------------------------

    bool condition(int idx, const std::string& path, std::size_t depth) {
        if (depth > kMaxConditionNesting) {
            fail(path, "is nested deeper than " + std::to_string(kMaxConditionNesting) +
                           " levels (each not, all and any is a level); split the rule, or make a fact");
            return false;
        }
        const int top = lua_gettop(L);
        idx = lua_absindex(L, idx);
        bool ok = false;
        std::string name;
        if (stringAt(idx, name)) ok = factCondition(name, path);
        else if (lua_istable(L, idx)) ok = tableCondition(idx, path, depth);
        else fail(path, "is " + describe(idx) + ", expected " + kConditionForms);
        lua_settop(L, top);
        return ok;
    }

    // A fact's name on its own.
    bool factCondition(const std::string& name, const std::string& path) {
        const FactSlot slot = facts_.find(name);
        if (slot == kNoFact) {
            NamedRead read;
            if (lookUp(name, read))
                fail(path, "is " + thing(read, name) + ", expected a fact: a number only counts in a comparison");
            else fail(path, "is \"" + name + "\", which is not a fact");
            return false;
        }
        Step s;
        s.op = StepOp::FactHolds;
        s.fact = slot;
        emit(s);
        addFactRead(slot);
        return true;
    }

    bool tableCondition(int t, const std::string& path, std::size_t depth) {
        std::vector<std::string> names;
        std::vector<std::string> odd;
        keysOf(t, names, odd);
        if (!names.empty()) {
            if (names.size() == 1 && odd.empty() && (names[0] == kKeyAll || names[0] == kKeyAny))
                return group(t, names[0], path, depth);
            fail(path, "has the key \"" + names[0] + "\", expected " + kConditionForms);
            return false;
        }
        const lua_Integer n = static_cast<lua_Integer>(lua_rawlen(L, t));
        checkListKeys(t, path, n, kConditionForms);
        const int top = lua_gettop(L);
        if (n == kNotLength) {
            lua_rawgeti(L, t, kFirst);
            std::string word;
            const bool isNot = stringAt(-1, word) && word == kWordNot;
            if (!isNot) fail(indexed(path, kFirst), "is " + describe(-1) + ", expected \"not\" in { \"not\", condition }");
            lua_settop(L, top);
            if (!isNot) return false;
            lua_rawgeti(L, t, kSecond);
            const bool ok = condition(-1, indexed(path, kSecond), depth + 1);
            lua_settop(L, top);
            emitOp(StepOp::Not);
            return ok;
        }
        if (n == kComparisonLength) return comparison(t, path);
        fail(path, "is a list of " + std::to_string(n) + ", expected " + kConditionForms);
        return false;
    }

    bool group(int t, const std::string& key, const std::string& path, std::size_t depth) {
        const int top = lua_gettop(L);
        const std::string groupPath = path + "." + key;
        const int list = pushField(t, key);
        if (!lua_istable(L, list)) {
            fail(groupPath, "is " + describe(list) + ", expected a list of conditions");
            lua_settop(L, top);
            return false;
        }
        const lua_Integer n = static_cast<lua_Integer>(lua_rawlen(L, list));
        checkListKeys(list, groupPath, n, "a list of conditions");
        if (n == 0) {
            fail(groupPath, "is empty, expected at least one condition");
            lua_settop(L, top);
            return false;
        }
        bool ok = true;
        for (lua_Integer i = 1; i <= n; ++i) {
            lua_rawgeti(L, list, i);
            ok = condition(-1, indexed(groupPath, i), depth + 1) && ok;
            lua_settop(L, list);
            if (i > 1) emitOp(key == kKeyAll ? StepOp::And : StepOp::Or);
        }
        lua_settop(L, top);
        return ok;
    }

    bool comparison(int t, const std::string& path) {
        const int top = lua_gettop(L);
        // The fact on the left.
        lua_rawgeti(L, t, kFirst);
        std::string left;
        FactSlot slot = kNoFact;
        if (!stringAt(-1, left)) {
            fail(indexed(path, kFirst), "is " + describe(-1) + ", expected the name of a fact");
        } else {
            slot = facts_.find(left);
            NamedRead other;
            if (slot == kNoFact && lookUp(left, other))
                fail(indexed(path, kFirst), "is " + thing(other, left) + ", expected a fact on the left of a comparison");
            else if (slot == kNoFact) fail(indexed(path, kFirst), "is \"" + left + "\", which is not a fact");
        }
        lua_settop(L, top);
        // The comparison.
        lua_rawgeti(L, t, kSecond);
        std::string opWord;
        bool haveOp = false;
        CompareOp op = CompareOp::Less;
        if (stringAt(-1, opWord)) {
            for (const OpWord& o : kOps) {
                if (opWord == o.word) {
                    op = o.op;
                    haveOp = true;
                }
            }
        }
        if (!haveOp) fail(indexed(path, kSecond), "is " + describe(-1) + ", expected one of (" + opWords() + ")");
        lua_settop(L, top);
        if (slot == kNoFact || !haveOp) return false;

        const FactSpec& fact = facts_.at(slot);
        Step s;
        s.op = StepOp::Compare;
        s.compare = op;
        s.fact = slot;
        if (comparesNumbers(op)) {
            if (fact.kind != FactKind::Number && fact.kind != FactKind::Place) {
                fail(indexed(path, kSecond),
                     "is \"" + opWord + "\", which compares numbers, but \"" + left + "\" is " + factKindWords(fact.kind) +
                         (fact.kind == FactKind::YesNo ? "; a yes/no fact is a condition on its own"
                                                       : "; a word fact takes is or is_not"));
                return false;
            }
            const UnitFamily unit = fact.kind == FactKind::Place ? UnitFamily::Length : fact.unit;
            lua_rawgeti(L, t, kThird);
            // A number written out has no unit, so it is only allowed against a plain ratio.
            if (unit != UnitFamily::Ratio && lua_type(L, -1) == LUA_TNUMBER) {
                fail(path, "compares \"" + left + "\", which is " + unitText(unit) +
                               ", with the written-out number " + describe(-1) +
                               "; name a rule-set number with a unit instead (numbers = { some_name = { 45, unit = "
                               "\"mph\" } })");
                lua_settop(L, top);
                return false;
            }
            addFactRead(slot);
            const bool ok = numberValue(lua_gettop(L), indexed(path, kThird), unit,
                                        "\"" + left + "\" is " + unitText(unit) +
                                            ": both sides of a comparison must measure the same thing",
                                        true, s.right);
            lua_settop(L, top);
            if (!ok) return false;
        } else {
            if (fact.kind != FactKind::Word) {
                fail(indexed(path, kSecond), "is \"" + opWord + "\", which compares words, but \"" + left + "\" is " +
                                                 factKindWords(fact.kind) +
                                                 (fact.kind == FactKind::YesNo
                                                      ? "; a yes/no fact is a condition on its own"
                                                      : "; numbers take <, <=, >, >=, == or ~="));
                return false;
            }
            lua_rawgeti(L, t, kThird);
            std::string word;
            WordIndex index = kNoWord;
            if (stringAt(-1, word))
                for (std::size_t w = 0; w < fact.words.size(); ++w)
                    if (fact.words[w] == word) index = static_cast<WordIndex>(w);
            if (index == kNoWord)
                fail(indexed(path, kThird), "is " + describe(-1) + ", which is not one of " + left + "'s words (" +
                                                joined(fact.words) + ")");
            lua_settop(L, top);
            if (index == kNoWord) return false;
            addFactRead(slot);
            s.right.source = ValueSource::Word;
            s.right.index = index;
        }
        emit(s);
        return true;
    }

    // ---- Answers ----------------------------------------------------------------------------------------------------

    bool give(int g, const std::string& path, CompiledRule& rule) {
        if (!lua_istable(L, g)) {
            fail(path, "is " + describe(g) + ", expected a table of answers");
            return false;
        }
        std::vector<std::string> keys;
        std::vector<std::string> odd;
        keysOf(g, keys, odd);
        bool ok = true;
        for (const std::string& o : odd) {
            fail(path, "has the key " + o + ", which is not a name; expected answers by name");
            ok = false;
        }
        if (keys.empty()) {
            fail(path, "is empty, expected at least one answer");
            return false;
        }
        bool hasUrgency = false;
        for (const std::string& key : keys) {
            const int top = lua_gettop(L);
            const std::string keyPath = path + "." + key;
            const int v = pushField(g, key);
            if (key == kGiveStopAt) {
                ok = stopAt(v, keyPath, rule) && ok;
            } else if (key == kGiveTopSpeed) {
                ok = numberValue(v, keyPath, UnitFamily::Speed, "top_speed must be " + unitText(UnitFamily::Speed),
                                 false, rule.topSpeed) && ok;
                rule.givesTopSpeed = true;
            } else if (key == kGiveAction) {
                ok = actions(v, keyPath) && ok;
            } else if (key == kGiveUrgency) {
                hasUrgency = true;
                ok = numberValue(v, keyPath, UnitFamily::Ratio, "urgency must be " + unitText(UnitFamily::Ratio), true,
                                 rule.urgency) && ok;
            } else {
                ok = intention(v, key, path, keyPath) && ok;
            }
            lua_settop(L, top);
        }
        rule.intentionCount = static_cast<uint32_t>(built_.program.intentions.size()) - rule.firstIntention;
        rule.actionCount = static_cast<uint32_t>(built_.program.actions.size()) - rule.firstAction;
        if (hasUrgency && rule.intentionCount == 0) {
            fail(path + "." + kGiveUrgency, "is given, but the rule gives no intention for it to rank");
            ok = false;
        }
        return ok;
    }

    bool stopAt(int v, const std::string& path, CompiledRule& rule) {
        std::string name;
        if (!stringAt(v, name)) {
            fail(path, "is " + describe(v) + ", expected the name of a place fact");
            return false;
        }
        const FactSlot slot = facts_.find(name);
        if (slot == kNoFact || facts_.at(slot).kind != FactKind::Place) {
            NamedRead read;
            fail(path, "is " + (lookUp(name, read) ? thing(read, name) : "\"" + name + "\"") +
                           ", which is not a place fact; stop_at takes a place to stop by");
            return false;
        }
        rule.givesStop = true;
        rule.stopAt = slot;
        addFactRead(slot);
        return true;
    }

    bool actions(int v, const std::string& path) {
        std::vector<std::string> names;
        if (!stringList(v, path, true, names)) return false;
        const bool single = lua_type(L, v) == LUA_TSTRING;
        std::vector<std::string> known;
        for (const ActionSpec& a : vocab_.actions()) known.push_back(a.name);
        bool ok = true;
        const std::size_t first = built_.program.actions.size();
        for (std::size_t i = 0; i < names.size(); ++i) {
            const ActionId id = vocab_.findAction(names[i]);
            if (id == kNoAction) {
                fail(single ? path : indexed(path, static_cast<lua_Integer>(i + 1)),
                     "is \"" + names[i] + "\", which is not an action (" + joined(known) + ")");
                ok = false;
                continue;
            }
            std::vector<ActionId>& list = built_.program.actions;
            if (std::find(list.begin() + static_cast<std::ptrdiff_t>(first), list.end(), id) == list.end())
                list.push_back(id);
        }
        return ok;
    }

    bool intention(int v, const std::string& key, const std::string& givePath, const std::string& keyPath) {
        const IntentionKindId kind = vocab_.findIntention(key);
        if (kind == kNoIntentionKind) {
            std::vector<std::string> kinds;
            for (const IntentionKindSpec& k : vocab_.intentions()) kinds.push_back(k.name);
            fail(givePath, "has the key \"" + key + "\", which is not " + kGiveStopAt + ", " + kGiveTopSpeed + ", " +
                               kGiveAction + ", " + kGiveUrgency + " or an intention kind (" + joined(kinds) + ")");
            return false;
        }
        const IntentionKindSpec& spec = vocab_.intentions()[kind];
        std::string word;
        const WordIndex index = stringAt(v, word) ? vocab_.findWord(kind, word) : kNoWord;
        if (index == kNoWord) {
            fail(keyPath, "is " + describe(v) + ", which is not one of " + key + "'s words (" + joined(spec.words) + ")");
            return false;
        }
        IntentionGiven g;
        g.kind = kind;
        g.word = index;
        built_.program.intentions.push_back(g);
        return true;
    }

    // ---- One rule ---------------------------------------------------------------------------------------------------

    void readRule(int t, const std::string& path) {
        if (!lua_istable(L, t)) {
            fail(path, "is " + describe(t) + ", expected a rule { name = \"...\", when = ..., give = { ... } }");
            return;
        }
        const int top = lua_gettop(L);
        // The name, which the driver card shows and every later message carries.
        std::string name;
        const int nameAt = pushField(t, kKeyName);
        const bool named = stringAt(nameAt, name) && !name.empty();
        const std::string nameShown = describe(nameAt);
        lua_settop(L, top);
        std::string rulePath = path;
        bool ok = named;
        if (!named) {
            fail(path, "has no name (name is " + nameShown + "), expected name = \"...\": the driver card shows it");
        } else {
            rulePath = path + " (\"" + name + "\")";
            for (const std::pair<std::string, std::string>& seen : seenRules_) {
                if (seen.first != name) continue;
                fail(rulePath, "has the same name as " + seen.second + "; each rule needs its own name");
                ok = false;
                break;
            }
            seenRules_.push_back(std::make_pair(name, path));
        }
        checkKeys(t, rulePath, {kKeyName, kKeyWhen, kKeyUnless, kKeyGive});

        CompiledRule rule;
        rule.firstStep = static_cast<uint32_t>(built_.program.steps.size());
        rule.firstRead = static_cast<uint32_t>(built_.program.reads.size());
        rule.firstIntention = static_cast<uint32_t>(built_.program.intentions.size());
        rule.firstAction = static_cast<uint32_t>(built_.program.actions.size());
        readStart_ = built_.program.reads.size();
        stackNow_ = 0;
        stackPeak_ = 0;

        const int when = pushField(t, kKeyWhen);
        const bool hasWhen = !lua_isnil(L, when);
        if (hasWhen) ok = condition(when, rulePath + "." + kKeyWhen, 1) && ok;
        lua_settop(L, top);
        const int unless = pushField(t, kKeyUnless);
        if (!lua_isnil(L, unless)) {
            ok = condition(unless, rulePath + "." + kKeyUnless, 1) && ok;
            emitOp(StepOp::Not);
            if (hasWhen) emitOp(StepOp::And);
        }
        lua_settop(L, top);
        const int g = pushField(t, kKeyGive);
        if (lua_isnil(L, g)) {
            fail(rulePath, "has no give, expected give = { ... } with at least one answer");
            ok = false;
        } else {
            ok = give(g, rulePath + "." + kKeyGive, rule) && ok;
        }
        lua_settop(L, top);
        if (stackPeak_ > kMaxConditionStack) {
            fail(rulePath, "has a condition too deep to decide; split the rule, or make a fact");
            ok = false;
        }
        if (!ok) return;   // the load fails; nothing of this rule is used
        rule.stepCount = static_cast<uint32_t>(built_.program.steps.size()) - rule.firstStep;
        rule.readCount = static_cast<uint32_t>(built_.program.reads.size()) - rule.firstRead;
        built_.program.rules.push_back(rule);
        built_.labels.rules.push_back(name);
    }
};

}  // namespace

bool loadRules(engine::ScriptVM& vm, const std::string& fileName, const FactRegistry& facts, const RuleSet& ruleSet,
               const AnswerVocabulary& vocabulary, CompiledRules& out, std::string* err) {
    lua_State* L = engine::luaState(vm);
    RulesReader reader(L, fileName, facts, ruleSet, vocabulary);
    CompiledRules scratch;
    if (!reader.read(scratch)) {
        if (err) *err = reader.report();
        return false;
    }
    out = std::move(scratch);
    return true;
}

}  // namespace driving
