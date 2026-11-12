#ifndef RAYTRACER_APPS_DRIVING_DECIDE_H
#define RAYTRACER_APPS_DRIVING_DECIDE_H

// WHAT A DRIVER DOES WITH WHAT IT NOTICES (driving-model-design.md 4 "Answers", 6.1 e, 6.2, 6.4, 13; issue #252).
//
// A rule set's `rules` turn noticed facts into ANSWERS. Each rule has a condition over facts ("when", and "unless",
// which means "and not") and gives one or more answers. There are exactly four kinds of answer, and that list is
// closed:
//   - a place to stop by (the nearest wins),
//   - a top speed (the lowest wins),
//   - an INTENTION: one word of a kind, with an urgency (per kind, the most urgent wins; a tie goes to the rule written
//     first),
//   - an ACTION: something that just happens (all happen, each once).
// Because answers combine by this fixed arithmetic, rules never need to know about each other and there are no rule
// priorities.
//
// The intention kinds, their words and the action names are a vocabulary REGISTERED in C++ (AnswerVocabulary), like
// facts, because later code acts on them. The library registers none of its own.
//
// The reader (scripting/rules_lua.h) checks the rules and COMPILES each condition into a short flat list of steps over
// fact slots, with rule-set numbers already turned into their values. Deciding then runs those steps: it reads only
// numbers and slots, never a name, a map or Lua (every type deciding reads is trivially copyable, which the .cpp checks
// at compile time, so none of them can hold a string). The names live apart, in RuleLabels, and only explain() and the
// coverage helpers read them.
//
// A DECISION keeps the winning answer of each kind and, for each, which rule gave it and the values of the facts and
// named numbers that rule read, so explain() can write "lane ends: lane_ends_ahead is yes; lane_end is 38.2 m" with
// no code per rule. Which of the stop and the top speed actually limits the car is decided later; both are kept.
//
// Same result every run: rules are kept in written order, nothing walks an unordered container.
//
// Units: metres, seconds, m/s. A rule may write a number out only where it is a plain ratio (an urgency, a comparison
// with a ratio fact); anything with a unit is named, as a rule-set number that carries its unit.

#include "apps/driving/notice.h"
#include "apps/driving/rule_set.h"
#include "rt_math.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace driving {

using engine::Real;

// ---- The answer vocabulary ------------------------------------------------------------------------------------------

// The `give` keys the rule language itself knows; every other key of a `give` must be a registered intention kind.
constexpr const char* kGiveStopAt = "stop_at";
constexpr const char* kGiveTopSpeed = "top_speed";
constexpr const char* kGiveAction = "action";
constexpr const char* kGiveUrgency = "urgency";

// An intention kind's place in registration order, and an action's.
using IntentionKindId = uint32_t;
using ActionId = uint32_t;
constexpr IntentionKindId kNoIntentionKind = 0xFFFFFFFFu;
constexpr ActionId kNoAction = 0xFFFFFFFFu;

// ONE KIND OF INTENTION: its name (the `give` key), the words it may take, and what it means.
struct IntentionKindSpec {
    std::string name;
    std::vector<std::string> words;
    std::string meaning;
};

// ONE ACTION: something that just happens when a rule gives it.
struct ActionSpec {
    std::string name;
    std::string meaning;
};

// EVERY INTENTION KIND AND ACTION rules may give, in registration order. An object the host owns: no global table.
class AnswerVocabulary {
public:
    // Adds an intention kind and gives its id. Refused (kNoIntentionKind, reason in `why`) for an empty name, a name
    // already registered, one of the rule language's own give keys, no words, or an empty or repeated word.
    IntentionKindId addIntention(const std::string& kind, const std::vector<std::string>& words,
                                 const std::string& meaning, std::string* why = nullptr);
    // Adds an action and gives its id. Refused (kNoAction, reason in `why`) for an empty name or one already registered.
    ActionId addAction(const std::string& name, const std::string& meaning, std::string* why = nullptr);

    IntentionKindId findIntention(const std::string& kind) const;
    ActionId findAction(const std::string& name) const;
    // Which of the kind's words, or kNoWord.
    WordIndex findWord(IntentionKindId kind, const std::string& word) const;
    const std::vector<IntentionKindSpec>& intentions() const { return intentions_; }
    const std::vector<ActionSpec>& actions() const { return actions_; }

private:
    std::vector<IntentionKindSpec> intentions_;
    std::vector<ActionSpec> actions_;
};

// ---- Compiled rules --------------------------------------------------------------------------------------------------

// A rule's place in written order.
using RuleIndex = uint32_t;
constexpr RuleIndex kNoRule = 0xFFFFFFFFu;

// How deeply conditions may nest: a condition at the top is level 1; each "not", "all" and "any" adds one.
constexpr std::size_t kMaxConditionNesting = 8;
// How many yes/no values a condition may hold at once while deciding (the steps keep at most one pending value per
// level, plus "unless"; the reader checks every program against this).
constexpr std::size_t kMaxConditionStack = kMaxConditionNesting + 2;

// The comparisons a condition may make.
enum class CompareOp : uint8_t { Less, LessOrEqual, Greater, GreaterOrEqual, Equal, NotEqual, Is, IsNot };

// WHERE A VALUE COMES FROM while deciding.
enum class ValueSource : uint8_t {
    Written,   // a number fixed at load: written out in the rule, or a rule-set number already turned into its value
    Fact,      // a fact's slot
    Setting,   // a personal setting (its place in RuleSet::personality): the driver's own value, through personalValue
    Word,      // one word of a word fact (`index` is the word), for is / is_not
};
struct ValueRef {
    ValueSource source = ValueSource::Written;
    uint32_t index = 0;       // the fact slot, the setting's place, or the word
    Real value = 0;           // a Written number
    uint32_t nameHash = 0;    // a Setting's settingNameHash, worked out at load
};

// ONE STEP of a condition's program. The steps run in order over a small stack of yes/no values: FactHolds and
// Compare push one, Not flips the top, And and Or join the top two into one. An empty program always holds.
enum class StepOp : uint8_t {
    FactHolds,   // push: the fact is present and, for a yes/no fact, yes
    Compare,     // push: fact `fact` compared by `compare` with `right`; false when either side is absent
    Not,
    And,
    Or,
};
struct Step {
    StepOp op = StepOp::FactHolds;
    CompareOp compare = CompareOp::Less;
    FactSlot fact = kNoFact;
    ValueRef right;
};

// A FACT OR NAMED VALUE a rule reads, kept so its value can be shown with the answer.
struct NamedRead {
    ValueRef value;                         // Fact, Setting, or Written (a rule-set number)
    FactKind factKind = FactKind::Number;   // a Fact's kind
    UnitFamily unit = UnitFamily::Ratio;    // what a number measures
};

// One intention a rule gives: which kind, which word.
struct IntentionGiven {
    IntentionKindId kind = kNoIntentionKind;
    WordIndex word = kNoWord;
};

// ONE RULE AS COMPILED: a stretch of steps, its answers, and a stretch of reads (all indices into RuleProgram).
struct CompiledRule {
    uint32_t firstStep = 0;
    uint32_t stepCount = 0;
    bool givesStop = false;
    FactSlot stopAt = kNoFact;                // a place fact
    bool givesTopSpeed = false;
    ValueRef topSpeed;                        // a speed: Fact, Written or Setting
    uint32_t firstIntention = 0;
    uint32_t intentionCount = 0;
    ValueRef urgency;                         // a ratio; Written 0 when the rule gives none
    uint32_t firstAction = 0;
    uint32_t actionCount = 0;
    uint32_t firstRead = 0;
    uint32_t readCount = 0;
};

// WHAT DECIDING READS: numbers and slots only.
struct RuleProgram {
    std::vector<CompiledRule> rules;          // written order
    std::vector<Step> steps;
    std::vector<IntentionGiven> intentions;
    std::vector<ActionId> actions;
    std::vector<NamedRead> reads;
    uint32_t intentionKindCount = 0;          // the vocabulary's, when the rules were read
    uint32_t actionCount = 0;
};

// THE NAMES, for explain() and coverage only; never read while deciding.
struct RuleLabels {
    std::vector<std::string> rules;                   // per rule
    std::vector<std::string> reads;                   // per RuleProgram::reads entry
    std::vector<std::vector<std::string>> readWords;  // per read: a word fact's words (empty otherwise)
    // Per read: a rule-set number as the file wrote it ("45 mph") when that differs from its stored value and unit,
    // else empty.
    std::vector<std::string> readWritten;
};

struct CompiledRules {
    RuleProgram program;
    RuleLabels labels;
    // Every fact slot the rules read, sorted, each once: the `wanted` list noticeFacts takes.
    std::vector<FactSlot> usedFacts;

    std::size_t size() const { return program.rules.size(); }
    // The rule of that name, or kNoRule.
    RuleIndex findRule(const std::string& name) const;
};

// ---- The decision ----------------------------------------------------------------------------------------------------

// A value a winning rule read, as it was when deciding.
struct ReadValue {
    bool present = false;
    Real number = 0;           // a number, place, rule-set number or setting
    bool yes = false;
    WordIndex word = kNoWord;
};

// WHY AN ANSWER: which rule gave it, and the values (in Decision::values) of what that rule read, in the order of its
// reads.
struct Reason {
    RuleIndex rule = kNoRule;
    uint32_t firstValue = 0;
    uint32_t valueCount = 0;
};

struct StopAnswer {
    bool given = false;
    Real distance = 0;   // metres ahead of my front bumper
    Reason why;
};
struct TopSpeedAnswer {
    bool given = false;
    Real speed = 0;      // m/s
    Reason why;
};
struct IntentionAnswer {
    bool given = false;
    WordIndex word = kNoWord;
    Real urgency = 0;
    Reason why;
};
struct ActionAnswer {
    ActionId action = kNoAction;
    Reason why;          // the first rule (in written order) that gave it
};

struct Decision {
    StopAnswer stop;
    TopSpeedAnswer topSpeed;
    std::vector<IntentionAnswer> intentions;   // one per intention kind, in registration order
    std::vector<ActionAnswer> actions;         // each once, in the order first given
    std::vector<ReadValue> values;             // the reasons' values
    std::vector<uint8_t> actionDone;           // working memory: per action, given already this decision
};

// HOW OFTEN EACH RULE HELD AND WON, over many decisions. A rule WON a decision when it gave at least one winning answer
// (an action counts for the first rule that gave it).
struct RuleCoverage {
    std::vector<uint64_t> held;
    std::vector<uint64_t> won;
    uint64_t decisions = 0;
    std::vector<uint64_t> lastWon;   // working memory: the decision a rule last won (so it is counted once)
};

// DECIDES for one driver: runs every rule over the noticed facts (one per registry slot, as noticeFacts fills them)
// and combines the answers. `ruleSet` and `person` are read only for personal settings. Pure: the same inputs give the
// same decision. `coverage`, when given, is added to.
void decide(const CompiledRules& rules, const std::vector<NoticedFact>& facts, const RuleSet& ruleSet, PersonId person,
            Decision& out, RuleCoverage* coverage = nullptr);

// One plain line for an answer: "<rule>: <read> is <value>; ...", e.g. "lane ends: lane_ends_ahead is yes; lane_end
// is 38.2 m". A rule that read nothing: "<rule>: always".
std::string explain(const CompiledRules& rules, const Decision& decision, const Reason& why);

// The rules that never held, and those that held but never won, each in written order.
void coverageGaps(const RuleCoverage& coverage, std::size_t ruleCount, std::vector<RuleIndex>& neverHeld,
                  std::vector<RuleIndex>& heldNeverWon);

}  // namespace driving

#endif
