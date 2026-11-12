// The driving library's rules and answers (#252): rules written in a rule set's Lua file are read, checked and
// compiled; decide() runs them over noticed facts and combines their answers (the nearest stop, the lowest top speed,
// the most urgent intention per kind, every action once); explain() writes each answer's reason as one line; coverage
// counts which rules held and which won. The facts here are filled in by hand, so no road or car is needed. The
// intention kinds and actions are registered by the tests: the library has none of its own.

#include "test_framework.h"

#include "apps/driving/decide.h"

#ifdef RT_ENABLE_SCRIPTING

#include "apps/driving/scripting/rule_set_lua.h"
#include "apps/driving/scripting/rules_lua.h"
#include "engine/scripting/lua_state.h"
#include "engine/scripting/script_vm.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

using namespace driving;

namespace {

constexpr const char* kFile = "us_left.lua";

// The rules of the hand-out's example.
const char* kExampleRules = R"LUA(
  { name = "speed limit",   give = { top_speed = "my_limit" } },
  { name = "bend ahead",    give = { top_speed = "corner_speed" } },
  { name = "lower limit ahead", give = { top_speed = "slower_ahead" } },
  { name = "lane ends",     when = "lane_ends_ahead", give = { stop_at = "lane_end" } },
  { name = "tailgated horn", when = { all = { "car_ahead", { "gap_ahead", "<", "horn_gap" }, { "not", "standing" } } },
    unless = { "closing_speed", "<", "no_speed" },
    give = { action = "horn" } },
  { name = "keep kerb side", when = { any = { { "my_speed", ">", "fast_road_speed" }, "lane_ends_ahead" } },
    give = { move = "toward_kerb", urgency = 0.3 } },
)LUA";

// A whole rule set around `rules` (and any extra numbers). Numbers with a unit are named: a rule may write a number
// out only against a plain ratio.
std::string ruleSetWith(const std::string& rules, const std::string& extraNumbers = "") {
    return std::string("rule_set = { name = 'test', drives_on = 'left', ranks = { 'stop' }, devices = {}, ") +
           "numbers = { horn_gap = { 5, unit = 'm', source = 'test' }, " +
           "far_turn_gap = { 4.1, unit = 's', source = 'test' }, " +
           "fast_road_speed = { 45, unit = 'mph', source = 'test' }, no_speed = { 0, unit = 'm/s', source = 'test' }, " +
           "impossible_speed = { 1000, unit = 'm/s', source = 'test' }, " +
           "calm = { 0.4, unit = 'ratio', source = 'test' }, " + extraNumbers + " }, " +
           "personality = { speed_habit = { usual = 1.0, low = 0.9, high = 1.15, unit = 'ratio' }, " +
           "keen_speed = { usual = 15, low = 10, high = 20, unit = 'm/s' } }, " +
           "rules = { " + rules + " } }";
}

// A fact function for test facts: the tests fill facts by hand, so it is never called.
void noticeNothing(const DriverView&, NoticeScratch&, NoticedFact&) {}

constexpr const char* kLight = "light";          // a word fact: red, yellow, green
constexpr const char* kStopLine = "stop_line";   // a place fact
constexpr const char* kPatience = "patience";    // a number fact, a ratio
constexpr const char* kWaited = "waited";        // a number fact, a time
const std::vector<std::string> kLightWords = {"red", "yellow", "green"};

struct Setup {
    FactRegistry facts;
    AnswerVocabulary vocab;
    RuleSet ruleSet;
    CompiledRules rules;
    bool ok = false;
    std::string err;
    int stackBefore = 0;
    int stackAfter = 0;

    explicit Setup(const std::string& lua) {
        CHECK(registerBuiltInFacts(facts));
        FactSpec light;
        light.name = kLight;
        light.kind = FactKind::Word;
        light.words = kLightWords;
        light.notice = noticeNothing;
        CHECK(facts.add(light) != kNoFact);
        FactSpec line;
        line.name = kStopLine;
        line.kind = FactKind::Place;
        line.unit = UnitFamily::Length;
        line.notice = noticeNothing;
        CHECK(facts.add(line) != kNoFact);
        FactSpec patience;   // a number fact measuring a plain ratio
        patience.name = kPatience;
        patience.unit = UnitFamily::Ratio;
        patience.notice = noticeNothing;
        CHECK(facts.add(patience) != kNoFact);
        FactSpec waited;     // a number fact measuring a time
        waited.name = kWaited;
        waited.unit = UnitFamily::Time;
        waited.notice = noticeNothing;
        CHECK(facts.add(waited) != kNoFact);
        CHECK(vocab.addIntention("move", {"toward_kerb", "toward_offside"}, "change lanes") != kNoIntentionKind);
        CHECK(vocab.addIntention("indicate", {"kerb_side", "offside"}, "show the indicator") != kNoIntentionKind);
        CHECK(vocab.addAction("horn", "sound the horn") != kNoAction);
        CHECK(vocab.addAction("flash", "flash the headlights") != kNoAction);

        engine::ScriptVM vm;
        std::string luaErr;
        CHECK(vm.doString(lua, &luaErr));
        if (!luaErr.empty()) std::printf("    lua: %s\n", luaErr.c_str());
        std::string setErr;
        CHECK(loadRuleSet(vm, kFile, ruleSet, &setErr));
        if (!setErr.empty()) std::printf("    rule set: %s\n", setErr.c_str());
        lua_State* L = engine::luaState(vm);
        stackBefore = lua_gettop(L);
        rules.labels.rules.push_back("untouched");
        ok = loadRules(vm, kFile, facts, ruleSet, vocab, rules, &err);
        stackAfter = lua_gettop(L);
    }

    // One absent fact per slot, each with its kind.
    std::vector<NoticedFact> blank() const {
        std::vector<NoticedFact> out(facts.size());
        for (std::size_t i = 0; i < facts.size(); ++i) out[i].kind = facts.at(static_cast<FactSlot>(i)).kind;
        return out;
    }
    FactSlot slot(const char* name) const { return facts.find(name); }
    RuleIndex rule(const char* name) const { return rules.findRule(name); }
};

void setNumber(const Setup& s, std::vector<NoticedFact>& f, const char* name, Real v) {
    NoticedFact& x = f[s.slot(name)];
    x.present = true;
    x.number = v;
}
void setYes(const Setup& s, std::vector<NoticedFact>& f, const char* name, bool yes) {
    NoticedFact& x = f[s.slot(name)];
    x.present = true;
    x.yes = yes;
}
void setWord(const Setup& s, std::vector<NoticedFact>& f, const char* name, WordIndex w) {
    NoticedFact& x = f[s.slot(name)];
    x.present = true;
    x.word = w;
}

// Which rules held in one decision, through a fresh coverage count.
std::vector<bool> heldRules(const Setup& s, const std::vector<NoticedFact>& f, PersonId person = 1) {
    RuleCoverage c;
    Decision d;
    decide(s.rules, f, s.ruleSet, person, d, &c);
    std::vector<bool> out;
    for (uint64_t h : c.held) out.push_back(h > 0);
    return out;
}

bool has(const std::string& text, const std::string& piece) { return text.find(piece) != std::string::npos; }

// A refused file: the message names the file and the key path and says `piece`; `out` and the stack are untouched.
void expectRefused(const std::string& rules, const std::string& path, const std::string& piece,
                   const std::string& extraNumbers = "") {
    const Setup s(ruleSetWith(rules, extraNumbers));
    CHECK(!s.ok);
    CHECK(s.stackAfter == s.stackBefore);
    CHECK(s.rules.labels.rules.size() == 1u && s.rules.labels.rules[0] == "untouched");
    if (!has(s.err, std::string(kFile) + ": " + path) || !has(s.err, piece)) {
        std::printf("    wanted '%s' and '%s' in: %s\n", path.c_str(), piece.c_str(), s.err.c_str());
        CHECK(false);
    }
}

}  // namespace

// ---- 1. the example loads -----------------------------------------------------------------------------------------

TEST_CASE(decide_example_loads_and_lists_used_facts) {
    const Setup s(ruleSetWith(kExampleRules));
    CHECK(s.ok);
    if (!s.ok) std::printf("    %s\n", s.err.c_str());
    CHECK(s.stackAfter == s.stackBefore);
    CHECK(s.rules.size() == 6u);
    const char* names[] = {"speed limit", "bend ahead", "lower limit ahead", "lane ends", "tailgated horn",
                           "keep kerb side"};
    for (std::size_t i = 0; i < 6u && i < s.rules.size(); ++i) CHECK(s.rules.labels.rules[i] == names[i]);

    std::vector<FactSlot> expected;
    const char* used[] = {kFactMyLimit,  kFactCornerSpeed, kFactSlowerAhead,  kFactLaneEndsAhead, kFactLaneEnd,
                          kFactCarAhead, kFactGapAhead,    kFactStanding,     kFactClosingSpeed,  kFactMySpeed};
    for (const char* u : used) expected.push_back(s.slot(u));
    std::sort(expected.begin(), expected.end());
    CHECK(s.rules.usedFacts == expected);
    // Facts nobody reads are not wanted.
    CHECK(std::find(s.rules.usedFacts.begin(), s.rules.usedFacts.end(), s.slot(kFactRoadLimit)) ==
          s.rules.usedFacts.end());
}

// ---- 2. condition forms ----------------------------------------------------------------------------------------------

TEST_CASE(decide_condition_forms) {
    const Setup s(ruleSetWith(R"LUA(
      { name = "close",        when = { "gap_ahead", "<", "horn_gap" },            give = { action = "horn" } },
      { name = "not close",    when = { "not", { "gap_ahead", "<", "horn_gap" } }, give = { action = "horn" } },
      { name = "unless",       when = "car_ahead", unless = { "closing_speed", "<", "no_speed" },
                               give = { action = "horn" } },
      { name = "nested",       when = { all = { "car_ahead",
                                                { any = { { "my_speed", ">", "fast_road_speed" }, "lane_ends_ahead" } } } },
                               give = { action = "horn" } },
      { name = "patient",      when = { "patience", ">=", 0.5 },                  give = { action = "horn" } },
      { name = "keen",         when = { "my_speed", ">", "keen_speed" },          give = { action = "horn" } },
      { name = "slower ahead", when = { "speed_ahead", "<", "my_speed" },         give = { action = "horn" } },
      { name = "red",          when = { "light", "is", "red" },                   give = { action = "horn" } },
      { name = "not red",      when = { "light", "is_not", "red" },               give = { action = "horn" } },
      { name = "always",       give = { action = "flash" } },
      { name = "only unless",  unless = "standing",                               give = { action = "flash" } },
    )LUA"));
    CHECK(s.ok);
    if (!s.ok) std::printf("    %s\n", s.err.c_str());
    if (!s.ok) return;
    const RuleIndex close = s.rule("close"), notClose = s.rule("not close"), unless = s.rule("unless"),
                    nested = s.rule("nested"), keen = s.rule("keen"), slower = s.rule("slower ahead"),
                    red = s.rule("red"), notRed = s.rule("not red"), always = s.rule("always"),
                    onlyUnless = s.rule("only unless");

    // Nothing noticed: a comparison with an absent fact is false, and "not" of it is true; a rule with no `when`
    // holds; `unless` over an absent fact holds.
    std::vector<NoticedFact> f = s.blank();
    std::vector<bool> h = heldRules(s, f);
    CHECK(!h[close] && h[notClose] && !h[unless] && !h[nested] && !h[keen] && !h[slower] && !h[red] && !h[notRed]);
    CHECK(h[always] && h[onlyUnless]);

    // A name on the right as a rule-set number (horn_gap = 5 m), and `unless`.
    setYes(s, f, kFactCarAhead, true);
    setNumber(s, f, kFactGapAhead, 3);
    setNumber(s, f, kFactClosingSpeed, 1);
    h = heldRules(s, f);
    CHECK(h[close] && !h[notClose] && h[unless]);
    setNumber(s, f, kFactClosingSpeed, -1);
    CHECK(!heldRules(s, f)[unless]);
    setNumber(s, f, kFactGapAhead, 5);
    CHECK(!heldRules(s, f)[close]);

    // Nested all / any.
    setNumber(s, f, kFactMySpeed, 10);
    setYes(s, f, kFactLaneEndsAhead, false);
    CHECK(!heldRules(s, f)[nested]);
    setYes(s, f, kFactLaneEndsAhead, true);
    CHECK(heldRules(s, f)[nested]);
    setYes(s, f, kFactLaneEndsAhead, false);
    setNumber(s, f, kFactMySpeed, 25);
    CHECK(heldRules(s, f)[nested]);
    // ... and the named number is compared in m/s: 45 mph is 20.1168 m/s.
    setNumber(s, f, kFactMySpeed, 20.0);
    CHECK(!heldRules(s, f)[nested]);
    setNumber(s, f, kFactMySpeed, 20.2);
    CHECK(heldRules(s, f)[nested]);
    setNumber(s, f, kFactMySpeed, 25);

    // A number written out against a ratio fact.
    const RuleIndex patient = s.rule("patient");
    CHECK(!heldRules(s, f)[patient]);
    setNumber(s, f, kPatience, 0.6);
    CHECK(heldRules(s, f)[patient]);
    setNumber(s, f, kPatience, 0.4);
    CHECK(!heldRules(s, f)[patient]);
    setYes(s, f, kFactCarAhead, false);
    CHECK(!heldRules(s, f)[nested]);

    // Another fact on the right.
    setNumber(s, f, kFactSpeedAhead, 20);
    CHECK(heldRules(s, f)[slower]);
    setNumber(s, f, kFactSpeedAhead, 30);
    CHECK(!heldRules(s, f)[slower]);

    // A personal setting on the right: two people at the edge get different outcomes.
    const PersonalSetting* keenSetting = s.ruleSet.findPersonal("keen_speed");
    CHECK(keenSetting != nullptr);
    PersonId low = 0, high = 0;
    for (PersonId p = 1; p < 100 && keenSetting; ++p) {
        const double v = personalValue(*keenSetting, p);
        if (low == 0 || v < personalValue(*keenSetting, low)) low = p;
        if (high == 0 || v > personalValue(*keenSetting, high)) high = p;
    }
    const double lowValue = personalValue(*keenSetting, low), highValue = personalValue(*keenSetting, high);
    CHECK(highValue - lowValue > 1.0);
    setNumber(s, f, kFactMySpeed, static_cast<Real>((lowValue + highValue) / 2));
    CHECK(heldRules(s, f, low)[keen]);
    CHECK(!heldRules(s, f, high)[keen]);

    // `is` and `is_not` on a word fact.
    setWord(s, f, kLight, 0);   // red
    h = heldRules(s, f);
    CHECK(h[red] && !h[notRed]);
    setWord(s, f, kLight, 2);   // green
    h = heldRules(s, f);
    CHECK(!h[red] && h[notRed]);

    // `unless` alone.
    setYes(s, f, kFactStanding, true);
    CHECK(!heldRules(s, f)[onlyUnless]);
}

// ---- 3 and 4. combining --------------------------------------------------------------------------------------------

TEST_CASE(decide_combining_answers) {
    const Setup s(ruleSetWith(R"LUA(
      { name = "lane ends",   when = "lane_ends_ahead", give = { stop_at = "lane_end" } },
      { name = "stop line",   give = { stop_at = "stop_line" } },
      { name = "speed limit", give = { top_speed = "my_limit" } },
      { name = "bend ahead",  give = { top_speed = "corner_speed" } },
      { name = "lower limit", give = { top_speed = "slower_ahead" } },
      { name = "drift kerb",  give = { move = "toward_kerb", urgency = 0.3 } },
      { name = "overtake",    give = { move = "toward_offside", urgency = 0.5, indicate = "offside" } },
      { name = "late kerb",   give = { move = "toward_kerb", urgency = 0.5 } },
      { name = "calm signal", give = { indicate = "kerb_side", urgency = "calm" } },
      { name = "horn one",    give = { action = "horn" } },
      { name = "horn two",    give = { action = { "flash", "horn" } } },
    )LUA"));
    CHECK(s.ok);
    if (!s.ok) std::printf("    %s\n", s.err.c_str());
    if (!s.ok) return;

    std::vector<NoticedFact> f = s.blank();
    setYes(s, f, kFactLaneEndsAhead, true);
    setNumber(s, f, kFactLaneEnd, 38.2);
    setNumber(s, f, kStopLine, 20);
    setNumber(s, f, kFactMyLimit, 13);
    setNumber(s, f, kFactCornerSpeed, 9);
    setNumber(s, f, kFactSlowerAhead, 11);
    Decision d;
    RuleCoverage c;
    decide(s.rules, f, s.ruleSet, 1, d, &c);

    // Two stops: the nearest wins; then the other way round.
    CHECK(d.stop.given && d.stop.distance == 20 && d.stop.why.rule == s.rule("stop line"));
    // Three top speeds: the lowest wins.
    CHECK(d.topSpeed.given && d.topSpeed.speed == 9 && d.topSpeed.why.rule == s.rule("bend ahead"));
    // One kind of intention: the most urgent wins, and a tie stays with the rule written first.
    const IntentionKindId move = s.vocab.findIntention("move"), indicate = s.vocab.findIntention("indicate");
    CHECK(d.intentions.size() == 2u);
    CHECK(d.intentions[move].given && d.intentions[move].why.rule == s.rule("overtake"));
    CHECK(d.intentions[move].word == s.vocab.findWord(move, "toward_offside") && d.intentions[move].urgency == 0.5);
    // Another kind, kept separately (and its urgency read from a rule-set number).
    CHECK(d.intentions[indicate].given && d.intentions[indicate].why.rule == s.rule("overtake"));
    // The same action from two rules happens once, from the first; the other action from the second.
    CHECK(d.actions.size() == 2u);
    if (d.actions.size() == 2u) {
        CHECK(d.actions[0].action == s.vocab.findAction("horn") && d.actions[0].why.rule == s.rule("horn one"));
        CHECK(d.actions[1].action == s.vocab.findAction("flash") && d.actions[1].why.rule == s.rule("horn two"));
    }

    setNumber(s, f, kFactLaneEnd, 12);
    setNumber(s, f, kFactCornerSpeed, 14);
    decide(s.rules, f, s.ruleSet, 1, d, &c);
    CHECK(d.stop.distance == 12 && d.stop.why.rule == s.rule("lane ends"));
    CHECK(d.topSpeed.speed == 11 && d.topSpeed.why.rule == s.rule("lower limit"));

    // An absent stop_at or top_speed fact gives nothing and does not count as winning.
    std::vector<NoticedFact> g = s.blank();
    setNumber(s, g, kFactMyLimit, 13);
    RuleCoverage only;
    decide(s.rules, g, s.ruleSet, 1, d, &only);
    CHECK(!d.stop.given);
    CHECK(d.topSpeed.given && d.topSpeed.speed == 13 && d.topSpeed.why.rule == s.rule("speed limit"));
    CHECK(only.held[s.rule("stop line")] == 1u && only.won[s.rule("stop line")] == 0u);
    CHECK(only.held[s.rule("bend ahead")] == 1u && only.won[s.rule("bend ahead")] == 0u);
    CHECK(only.won[s.rule("speed limit")] == 1u);
    // Nothing given at all.
    decide(s.rules, s.blank(), s.ruleSet, 1, d);
    CHECK(!d.stop.given && !d.topSpeed.given);
}

// ---- 5. explain -------------------------------------------------------------------------------------------------------

TEST_CASE(decide_explain_lines) {
    const Setup s(ruleSetWith(kExampleRules));
    CHECK(s.ok);
    if (!s.ok) return;
    std::vector<NoticedFact> f = s.blank();
    setYes(s, f, kFactLaneEndsAhead, true);
    setNumber(s, f, kFactLaneEnd, 38.2);
    setNumber(s, f, kFactMyLimit, 13.4);
    setNumber(s, f, kFactCornerSpeed, 12.5);
    setYes(s, f, kFactCarAhead, true);
    setNumber(s, f, kFactGapAhead, 3);
    setYes(s, f, kFactStanding, false);
    setNumber(s, f, kFactClosingSpeed, 2);
    setNumber(s, f, kFactMySpeed, 22);
    Decision d;
    decide(s.rules, f, s.ruleSet, 1, d);
    const std::string stop = explain(s.rules, d, d.stop.why);
    const std::string top = explain(s.rules, d, d.topSpeed.why);
    const std::string move = explain(s.rules, d, d.intentions[s.vocab.findIntention("move")].why);
    CHECK(d.actions.size() == 1u);
    const std::string horn = d.actions.empty() ? std::string() : explain(s.rules, d, d.actions[0].why);
    std::printf("    %s\n    %s\n    %s\n    %s\n", stop.c_str(), top.c_str(), move.c_str(), horn.c_str());
    CHECK(stop == "lane ends: lane_ends_ahead is yes; lane_end is 38.2 m");
    CHECK(top == "bend ahead: corner_speed is 12.5 m/s");
    // A named number shows what was written when that is not how it is stored.
    CHECK(move == "keep kerb side: my_speed is 22.0 m/s; fast_road_speed is 20.1 m/s (45 mph); lane_ends_ahead is yes");
    CHECK(horn == "tailgated horn: car_ahead is yes; gap_ahead is 3.0 m; horn_gap is 5.0 m; standing is no; "
                  "closing_speed is 2.0 m/s; no_speed is 0.0 m/s");
}

// ---- 6. coverage --------------------------------------------------------------------------------------------------

TEST_CASE(decide_coverage_never_held_and_never_won) {
    const Setup s(ruleSetWith(std::string(kExampleRules) + R"LUA(
      { name = "never",      when = { "my_speed", ">", "impossible_speed" }, give = { action = "horn" } },
      { name = "high limit", give = { top_speed = "road_limit" } },
      { name = "second horn", when = "car_ahead", give = { action = "horn" } },
    )LUA"));
    CHECK(s.ok);
    if (!s.ok) std::printf("    %s\n", s.err.c_str());
    if (!s.ok) return;
    constexpr int kDecisions = 300;
    RuleCoverage c;
    Decision d;
    for (int i = 0; i < kDecisions; ++i) {
        std::vector<NoticedFact> f = s.blank();
        const Real limit = 13 + static_cast<Real>(i % 5);
        setNumber(s, f, kFactRoadLimit, limit + 5);   // always above the others: never the lowest
        setNumber(s, f, kFactMyLimit, limit);
        if (i % 3 == 0) setNumber(s, f, kFactCornerSpeed, limit - 1 - static_cast<Real>(i % 4));
        if (i % 7 == 0) setNumber(s, f, kFactSlowerAhead, limit - 2);
        setYes(s, f, kFactLaneEndsAhead, i % 11 == 0);
        if (i % 11 == 0) setNumber(s, f, kFactLaneEnd, 30 + static_cast<Real>(i % 13));
        setYes(s, f, kFactCarAhead, i % 2 == 0);
        if (i % 2 == 0) {
            setNumber(s, f, kFactGapAhead, static_cast<Real>(2 + i % 9));
            setNumber(s, f, kFactClosingSpeed, static_cast<Real>(i % 5) - 2);
        }
        setNumber(s, f, kFactMySpeed, static_cast<Real>(i % 30));
        setYes(s, f, kFactStanding, i % 30 == 0);
        decide(s.rules, f, s.ruleSet, static_cast<PersonId>(i), d, &c);
    }
    CHECK(c.decisions == static_cast<uint64_t>(kDecisions));
    std::vector<RuleIndex> neverHeld, heldNeverWon;
    coverageGaps(c, s.rules.size(), neverHeld, heldNeverWon);
    // "second horn" holds whenever there is a car ahead, but "tailgated horn" (written earlier) gets the horn first
    // whenever both give it, and it gives nothing else.
    CHECK(neverHeld == std::vector<RuleIndex>{s.rule("never")});
    CHECK(c.won[s.rule("tailgated horn")] > 0u);
    std::vector<RuleIndex> expected = {s.rule("high limit")};
    // Whether "second horn" ever won depends on cars ahead where "tailgated horn" did not hold: with a gap of 2..10 m
    // against horn_gap 5 m, some do, so it won sometimes; it is not in the list.
    CHECK(c.won[s.rule("second horn")] > 0u);
    CHECK(heldNeverWon == expected);
    for (RuleIndex r : heldNeverWon) std::printf("    held, never won: %s\n", s.rules.labels.rules[r].c_str());
    for (RuleIndex r : neverHeld) std::printf("    never held: %s\n", s.rules.labels.rules[r].c_str());
}

// ---- 7. refusals --------------------------------------------------------------------------------------------------

TEST_CASE(decide_reader_refuses_every_fault) {
    const std::string horn = "give = { action = 'horn' }";
    const std::string r1 = "rule_set.rules[1] (\"x\")";
    expectRefused("{ " + horn + " }", "rule_set.rules[1] has no name", "expected name");
    expectRefused("{ name = 'x' }", r1 + " has no give", "at least one answer");
    expectRefused("{ name = 'x', " + horn + " }, { name = 'x', " + horn + " }", "rule_set.rules[2] (\"x\")",
                  "same name as rule_set.rules[1]");
    expectRefused("{ name = 'x', when = 'nope', " + horn + " }", r1 + ".when", "\"nope\", which is not a fact");
    expectRefused("{ name = 'x', when = { 'gap_ahead', '<', 'nope' }, " + horn + " }", r1 + ".when[3]",
                  "not a fact, a rule-set number or a personal setting");
    // A number written out has no unit: refused against anything but a ratio fact (a speed, a length, a time).
    const std::string nameOne = "; name a rule-set number with a unit instead (numbers = { some_name = { 45, unit = "
                                "\"mph\" } })";
    expectRefused("{ name = 'x', when = { any = { { 'my_speed', '>', 20 }, 'car_ahead' } }, " + horn + " }",
                  r1 + ".when.any[1]",
                  "compares \"my_speed\", which is a speed (m/s), with the written-out number 20" + nameOne);
    expectRefused("{ name = 'x', when = { 'gap_ahead', '<', 3 }, " + horn + " }", r1 + ".when",
                  "compares \"gap_ahead\", which is a length (m), with the written-out number 3" + nameOne);
    expectRefused("{ name = 'x', when = { 'waited', '>=', 9 }, " + horn + " }", r1 + ".when",
                  "compares \"waited\", which is a time (s), with the written-out number 9" + nameOne);
    {
        const Setup s(ruleSetWith("{ name = 'x', when = { 'my_speed', '>', 20 }, " + horn + " }"));
        std::printf("%s\n", s.err.c_str());
    }
    expectRefused("{ name = 'x', give = { wiggle = 'x' } }", r1 + ".give", "\"wiggle\", which is not stop_at");
    expectRefused("{ name = 'x', give = { move = 'sideways' } }", r1 + ".give.move", "not one of move's words");
    expectRefused("{ name = 'x', give = { action = 'siren' } }", r1 + ".give.action", "not an action (horn, flash)");
    expectRefused("{ name = 'x', when = { 'light', 'is', 'blue' }, " + horn + " }", r1 + ".when[3]",
                  "not one of light's words (red, yellow, green)");
    expectRefused("{ name = 'x', when = { 'car_ahead', '<', 3 }, " + horn + " }", r1 + ".when[2]",
                  "\"car_ahead\" is a yes/no fact");
    expectRefused("{ name = 'x', when = { 'light', '>', 3 }, " + horn + " }", r1 + ".when[2]",
                  "\"light\" is a word fact");
    expectRefused("{ name = 'x', when = { 'gap_ahead', 'is', 'red' }, " + horn + " }", r1 + ".when[2]",
                  "compares words, but \"gap_ahead\" is a number fact");
    expectRefused("{ name = 'x', when = { 'gap_ahead', '<', 'far_turn_gap' }, " + horn + " }", r1 + ".when[3]",
                  "which is a time (s), but \"gap_ahead\" is a length (m)");
    expectRefused("{ name = 'x', give = { stop_at = 'gap_ahead' } }", r1 + ".give.stop_at", "not a place fact");
    expectRefused("{ name = 'x', give = { top_speed = 'horn_gap' } }", r1 + ".give.top_speed",
                  "top_speed must be a speed");
    expectRefused("{ name = 'x', wen = 'car_ahead', " + horn + " }", r1 + " has the key \"wen\"", "not one of");
    expectRefused("{ name = 'x', when = { all = {} }, " + horn + " }", r1 + ".when.all", "is empty");
    expectRefused("{ name = 'x', when = { any = {} }, " + horn + " }", r1 + ".when.any", "is empty");
    expectRefused("{ name = 'x', when = { all = { 'car_ahead', { 'car_ahead', '<', 1 } } }, " + horn + " }",
                  r1 + ".when.all[2][2]", "yes/no fact");
    expectRefused("{ name = 'x', give = { urgency = 0.5, action = 'horn' } }", r1 + ".give.urgency",
                  "no intention");
    expectRefused("{ name = 'x', when = 'horn_gap', " + horn + " }", r1 + ".when", "expected a fact");
    // Nested one level deeper than allowed.
    std::string deep = "'car_ahead'";
    for (std::size_t i = 0; i < kMaxConditionNesting; ++i) deep = "{ 'not', " + deep + " }";
    expectRefused("{ name = 'x', when = " + deep + ", " + horn + " }", r1 + ".when", "nested deeper than 8 levels");
    // ... and exactly at the limit is fine.
    std::string atLimit = "'car_ahead'";
    for (std::size_t i = 0; i + 1 < kMaxConditionNesting; ++i) atLimit = "{ 'not', " + atLimit + " }";
    CHECK(Setup(ruleSetWith("{ name = 'x', when = " + atLimit + ", " + horn + " }")).ok);
    // A fact and a rule-set number with one name.
    expectRefused("", "rule_set.numbers.gap_ahead", "has the same name as the fact \"gap_ahead\"",
                  "gap_ahead = { 3, unit = 'm', source = 'test' },");
    // The rules list itself.
    expectRefused("speed = 1", "rule_set.rules has the key \"speed\"", "expected only a list of rules");
    {
        const Setup s("rule_set = { name = 't', drives_on = 'left', ranks = {}, devices = {}, numbers = {}, "
                      "personality = {} }");
        CHECK(!s.ok && has(s.err, std::string(kFile) + ": rule_set.rules is missing, expected a list of rules"));
    }

    // Four faults in one file: all four reported, one per line.
    const Setup four(ruleSetWith("{ name = 'a', when = 'nope', " + horn + " }, "
                                 "{ name = 'b', give = { move = 'sideways' } }, "
                                 "{ name = 'c', when = { 'gap_ahead', '<', 'far_turn_gap' }, " + horn + " }, "
                                 "{ name = 'd', when = { 'car_ahead', '<', 3 }, " + horn + " }"));
    CHECK(!four.ok);
    std::size_t lines = four.err.empty() ? 0 : 1;
    for (char ch : four.err) lines += ch == '\n' ? 1 : 0;
    CHECK(lines == 4u);
    CHECK(has(four.err, "rule_set.rules[1] (\"a\").when") && has(four.err, "rule_set.rules[2] (\"b\").give.move") &&
          has(four.err, "rule_set.rules[3] (\"c\").when[3]") && has(four.err, "rule_set.rules[4] (\"d\").when[2]"));
    std::printf("%s\n", four.err.c_str());
}

// ---- 8. timing --------------------------------------------------------------------------------------------------------

TEST_CASE(decide_timing_example_rules) {
    const Setup s(ruleSetWith(kExampleRules));
    CHECK(s.ok);
    if (!s.ok) return;
    std::vector<NoticedFact> f = s.blank();
    setYes(s, f, kFactLaneEndsAhead, true);
    setNumber(s, f, kFactLaneEnd, 38.2);
    setNumber(s, f, kFactMyLimit, 13.4);
    setNumber(s, f, kFactCornerSpeed, 12.5);
    setNumber(s, f, kFactSlowerAhead, 11);
    setYes(s, f, kFactCarAhead, true);
    setNumber(s, f, kFactGapAhead, 3);
    setYes(s, f, kFactStanding, false);
    setNumber(s, f, kFactClosingSpeed, 2);
    constexpr int kRuns = 100000;
    Decision d;
    RuleCoverage c;
    double checksum = 0;
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    for (int i = 0; i < kRuns; ++i) {
        f[s.slot(kFactMySpeed)].present = true;
        f[s.slot(kFactMySpeed)].number = static_cast<Real>(i % 40);
        decide(s.rules, f, s.ruleSet, static_cast<PersonId>(i), d, &c);
        checksum += d.topSpeed.speed + (d.intentions[0].given ? 1 : 0);
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    std::printf("    100000 decisions of the example rules: %.1f ms (%.0f ns each; checksum %.0f)\n", ms,
                ms * 1.0e6 / kRuns, checksum);
    CHECK(c.decisions == static_cast<uint64_t>(kRuns));
}

#endif   // RT_ENABLE_SCRIPTING

// The vocabulary needs no Lua.
TEST_CASE(decide_vocabulary_refusals) {
    driving::AnswerVocabulary v;
    std::string why;
    CHECK(v.addIntention("move", {"toward_kerb", "toward_offside"}, "change lanes", &why) == 0u);
    CHECK(v.addIntention("move", {"a"}, "again", &why) == driving::kNoIntentionKind);
    CHECK(v.addIntention("urgency", {"a"}, "a give key", &why) == driving::kNoIntentionKind);
    CHECK(v.addIntention("wave", {}, "no words", &why) == driving::kNoIntentionKind);
    CHECK(v.addIntention("wave", {"a", "a"}, "twice", &why) == driving::kNoIntentionKind);
    CHECK(v.addAction("horn", "sound the horn", &why) == 0u);
    CHECK(v.addAction("horn", "again", &why) == driving::kNoAction);
    CHECK(v.findWord(0, "toward_offside") == 1u);
    CHECK(v.findWord(0, "sideways") == driving::kNoWord);
}
