#include "apps/driving/decide.h"

#include <iomanip>
#include <sstream>
#include <type_traits>

namespace driving {

// Deciding reads only these types (and NoticedFact, and a PersonalSetting's numbers). Trivially copyable types cannot
// hold a std::string, a container or anything that allocates, so deciding touches no strings by construction.
static_assert(std::is_trivially_copyable<ValueRef>::value, "deciding must not read strings");
static_assert(std::is_trivially_copyable<Step>::value, "deciding must not read strings");
static_assert(std::is_trivially_copyable<NamedRead>::value, "deciding must not read strings");
static_assert(std::is_trivially_copyable<IntentionGiven>::value, "deciding must not read strings");
static_assert(std::is_trivially_copyable<CompiledRule>::value, "deciding must not read strings");
static_assert(std::is_trivially_copyable<NoticedFact>::value, "deciding must not read strings");
static_assert(std::is_trivially_copyable<ReadValue>::value, "a decision must not hold strings");
static_assert(std::is_trivially_copyable<StopAnswer>::value, "a decision must not hold strings");
static_assert(std::is_trivially_copyable<TopSpeedAnswer>::value, "a decision must not hold strings");
static_assert(std::is_trivially_copyable<IntentionAnswer>::value, "a decision must not hold strings");
static_assert(std::is_trivially_copyable<ActionAnswer>::value, "a decision must not hold strings");

namespace {

// How many decimals explain() shows: measured values to a tenth, ratios to a hundredth.
constexpr int kShownDecimals = 1;
constexpr int kShownRatioDecimals = 2;

const char* kWordYes = "yes";
const char* kWordNo = "no";
const char* kWordAbsent = "absent";
const char* kWordAlways = "always";

// ---- Deciding: numbers and slots only ------------------------------------------------------------------------------

const NoticedFact* factAt(const std::vector<NoticedFact>& facts, FactSlot slot) {
    return slot < facts.size() ? &facts[slot] : nullptr;
}

// The number a value names, or false when it is absent (an absent fact, a setting the rule set does not have).
bool numberOf(const ValueRef& v, const std::vector<NoticedFact>& facts, const RuleSet& ruleSet, PersonId person,
              Real& out) {
    switch (v.source) {
    case ValueSource::Written:
        out = v.value;
        return true;
    case ValueSource::Fact: {
        const NoticedFact* f = factAt(facts, v.index);
        if (!f || !f->present) return false;
        out = f->number;
        return true;
    }
    case ValueSource::Setting:
        if (v.index >= ruleSet.personality.size()) return false;
        out = static_cast<Real>(personalValue(ruleSet.personality[v.index], v.nameHash, person));
        return true;
    case ValueSource::Word:
        return false;
    }
    return false;
}

bool factHolds(const NoticedFact* f) { return f && f->present && (f->kind != FactKind::YesNo || f->yes); }

bool compare(const Step& s, const std::vector<NoticedFact>& facts, const RuleSet& ruleSet, PersonId person) {
    const NoticedFact* left = factAt(facts, s.fact);
    if (!left || !left->present) return false;
    if (s.compare == CompareOp::Is) return left->word == s.right.index;
    if (s.compare == CompareOp::IsNot) return left->word != s.right.index;
    Real right = 0;
    if (!numberOf(s.right, facts, ruleSet, person, right)) return false;
    const Real l = left->number;
    switch (s.compare) {
    case CompareOp::Less: return l < right;
    case CompareOp::LessOrEqual: return l <= right;
    case CompareOp::Greater: return l > right;
    case CompareOp::GreaterOrEqual: return l >= right;
    case CompareOp::Equal: return l == right;
    case CompareOp::NotEqual: return l != right;
    case CompareOp::Is:
    case CompareOp::IsNot: break;
    }
    return false;
}

// Runs one rule's steps. A program that would overrun the stack (the reader never makes one) does not hold.
bool holds(const RuleProgram& p, const CompiledRule& rule, const std::vector<NoticedFact>& facts,
           const RuleSet& ruleSet, PersonId person) {
    if (rule.stepCount == 0) return true;
    bool stack[kMaxConditionStack];
    std::size_t top = 0;
    const uint32_t end = rule.firstStep + rule.stepCount;
    for (uint32_t i = rule.firstStep; i < end && i < p.steps.size(); ++i) {
        const Step& s = p.steps[i];
        switch (s.op) {
        case StepOp::FactHolds:
        case StepOp::Compare:
            if (top >= kMaxConditionStack) return false;
            stack[top++] = s.op == StepOp::FactHolds ? factHolds(factAt(facts, s.fact))
                                                     : compare(s, facts, ruleSet, person);
            break;
        case StepOp::Not:
            if (top < 1) return false;
            stack[top - 1] = !stack[top - 1];
            break;
        case StepOp::And:
        case StepOp::Or:
            if (top < 2) return false;
            --top;
            stack[top - 1] = s.op == StepOp::And ? (stack[top - 1] && stack[top]) : (stack[top - 1] || stack[top]);
            break;
        }
    }
    return top == 1 && stack[0];
}

// Fills a winner's reason: the values of everything its rule read, as they are now.
void attachReason(const RuleProgram& p, const std::vector<NoticedFact>& facts, const RuleSet& ruleSet,
                  PersonId person, Decision& out, Reason& why) {
    if (why.rule >= p.rules.size()) return;
    const CompiledRule& rule = p.rules[why.rule];
    why.firstValue = static_cast<uint32_t>(out.values.size());
    why.valueCount = rule.readCount;
    for (uint32_t i = rule.firstRead; i < rule.firstRead + rule.readCount && i < p.reads.size(); ++i) {
        const NamedRead& r = p.reads[i];
        ReadValue v;
        if (r.value.source == ValueSource::Fact) {
            const NoticedFact* f = factAt(facts, r.value.index);
            if (f && f->present) {
                v.present = true;
                v.number = f->number;
                v.yes = f->yes;
                v.word = f->word;
            }
        } else {
            v.present = numberOf(r.value, facts, ruleSet, person, v.number);
        }
        out.values.push_back(v);
    }
}

void markWon(RuleCoverage* coverage, RuleIndex rule) {
    if (!coverage || rule >= coverage->won.size()) return;
    if (coverage->lastWon[rule] == coverage->decisions) return;
    coverage->lastWon[rule] = coverage->decisions;
    ++coverage->won[rule];
}

// ---- Explaining: names allowed ---------------------------------------------------------------------------------

std::string numberText(Real v, UnitFamily unit) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(unit == UnitFamily::Ratio ? kShownRatioDecimals : kShownDecimals) << v;
    if (unit != UnitFamily::Ratio) s << " " << unitFamilyStoredUnit(unit);
    return s.str();
}

std::string valueText(const NamedRead& r, const std::vector<std::string>& words, const ReadValue& v) {
    if (!v.present) return kWordAbsent;
    if (r.value.source == ValueSource::Fact) {
        if (r.factKind == FactKind::YesNo) return v.yes ? kWordYes : kWordNo;
        if (r.factKind == FactKind::Word) return v.word < words.size() ? words[v.word] : kWordAbsent;
    }
    return numberText(v.number, r.unit);
}

}  // namespace

// ---- The vocabulary -----------------------------------------------------------------------------------------------

IntentionKindId AnswerVocabulary::addIntention(const std::string& kind, const std::vector<std::string>& words,
                                               const std::string& meaning, std::string* why) {
    std::string problem;
    if (kind.empty()) problem = "an intention kind needs a name";
    else if (findIntention(kind) != kNoIntentionKind) problem = "intention kind \"" + kind + "\" is already registered";
    else if (kind == kGiveStopAt || kind == kGiveTopSpeed || kind == kGiveAction || kind == kGiveUrgency)
        problem = "\"" + kind + "\" is one of the rule language's own give keys, so it cannot be an intention kind";
    else if (words.empty()) problem = "intention kind \"" + kind + "\" needs at least one word";
    for (std::size_t i = 0; problem.empty() && i < words.size(); ++i) {
        if (words[i].empty()) problem = "intention kind \"" + kind + "\" has an empty word";
        for (std::size_t j = 0; problem.empty() && j < i; ++j)
            if (words[j] == words[i]) problem = "intention kind \"" + kind + "\" lists \"" + words[i] + "\" twice";
    }
    if (!problem.empty()) {
        if (why) *why = problem;
        return kNoIntentionKind;
    }
    IntentionKindSpec spec;
    spec.name = kind;
    spec.words = words;
    spec.meaning = meaning;
    intentions_.push_back(spec);
    return static_cast<IntentionKindId>(intentions_.size() - 1);
}

ActionId AnswerVocabulary::addAction(const std::string& name, const std::string& meaning, std::string* why) {
    if (name.empty() || findAction(name) != kNoAction) {
        if (why) *why = name.empty() ? "an action needs a name" : "action \"" + name + "\" is already registered";
        return kNoAction;
    }
    ActionSpec spec;
    spec.name = name;
    spec.meaning = meaning;
    actions_.push_back(spec);
    return static_cast<ActionId>(actions_.size() - 1);
}

IntentionKindId AnswerVocabulary::findIntention(const std::string& kind) const {
    for (std::size_t i = 0; i < intentions_.size(); ++i)
        if (intentions_[i].name == kind) return static_cast<IntentionKindId>(i);
    return kNoIntentionKind;
}

ActionId AnswerVocabulary::findAction(const std::string& name) const {
    for (std::size_t i = 0; i < actions_.size(); ++i)
        if (actions_[i].name == name) return static_cast<ActionId>(i);
    return kNoAction;
}

WordIndex AnswerVocabulary::findWord(IntentionKindId kind, const std::string& word) const {
    if (kind >= intentions_.size()) return kNoWord;
    const std::vector<std::string>& words = intentions_[kind].words;
    for (std::size_t i = 0; i < words.size(); ++i)
        if (words[i] == word) return static_cast<WordIndex>(i);
    return kNoWord;
}

RuleIndex CompiledRules::findRule(const std::string& name) const {
    for (std::size_t i = 0; i < labels.rules.size(); ++i)
        if (labels.rules[i] == name) return static_cast<RuleIndex>(i);
    return kNoRule;
}

// ---- Deciding -----------------------------------------------------------------------------------------------------

void decide(const CompiledRules& rules, const std::vector<NoticedFact>& facts, const RuleSet& ruleSet, PersonId person,
            Decision& out, RuleCoverage* coverage) {
    // Only the program from here on: the labels (the names) are not reachable.
    const RuleProgram& p = rules.program;
    out.stop = StopAnswer();
    out.topSpeed = TopSpeedAnswer();
    out.intentions.assign(p.intentionKindCount, IntentionAnswer());
    out.actions.clear();
    out.values.clear();
    out.actionDone.assign(p.actionCount, 0);
    if (coverage) {
        if (coverage->held.size() != p.rules.size()) {
            coverage->held.assign(p.rules.size(), 0);
            coverage->won.assign(p.rules.size(), 0);
            coverage->lastWon.assign(p.rules.size(), 0);
            coverage->decisions = 0;
        }
        ++coverage->decisions;   // starts at 1, so lastWon's 0 means "never"
    }

    for (std::size_t r = 0; r < p.rules.size(); ++r) {
        const CompiledRule& rule = p.rules[r];
        if (!holds(p, rule, facts, ruleSet, person)) continue;
        if (coverage) ++coverage->held[r];
        const RuleIndex index = static_cast<RuleIndex>(r);

        if (rule.givesStop) {
            const NoticedFact* f = factAt(facts, rule.stopAt);
            if (f && f->present && (!out.stop.given || f->number < out.stop.distance)) {
                out.stop.given = true;
                out.stop.distance = f->number;
                out.stop.why.rule = index;
            }
        }
        if (rule.givesTopSpeed) {
            Real speed = 0;
            if (numberOf(rule.topSpeed, facts, ruleSet, person, speed) &&
                (!out.topSpeed.given || speed < out.topSpeed.speed)) {
                out.topSpeed.given = true;
                out.topSpeed.speed = speed;
                out.topSpeed.why.rule = index;
            }
        }
        Real urgency = 0;
        if (rule.intentionCount > 0 && numberOf(rule.urgency, facts, ruleSet, person, urgency)) {
            for (uint32_t i = rule.firstIntention; i < rule.firstIntention + rule.intentionCount; ++i) {
                const IntentionGiven& g = p.intentions[i];
                if (g.kind >= out.intentions.size()) continue;
                IntentionAnswer& a = out.intentions[g.kind];
                if (a.given && !(urgency > a.urgency)) continue;   // equal urgency: the earlier rule keeps it
                a.given = true;
                a.word = g.word;
                a.urgency = urgency;
                a.why.rule = index;
            }
        }
        for (uint32_t i = rule.firstAction; i < rule.firstAction + rule.actionCount; ++i) {
            const ActionId action = p.actions[i];
            if (action >= out.actionDone.size() || out.actionDone[action]) continue;
            out.actionDone[action] = 1;
            ActionAnswer a;
            a.action = action;
            a.why.rule = index;
            out.actions.push_back(a);
        }
    }

    // The winners' reasons, and who won.
    if (out.stop.given) {
        attachReason(p, facts, ruleSet, person, out, out.stop.why);
        markWon(coverage, out.stop.why.rule);
    }
    if (out.topSpeed.given) {
        attachReason(p, facts, ruleSet, person, out, out.topSpeed.why);
        markWon(coverage, out.topSpeed.why.rule);
    }
    for (IntentionAnswer& a : out.intentions) {
        if (!a.given) continue;
        attachReason(p, facts, ruleSet, person, out, a.why);
        markWon(coverage, a.why.rule);
    }
    for (ActionAnswer& a : out.actions) {
        attachReason(p, facts, ruleSet, person, out, a.why);
        markWon(coverage, a.why.rule);
    }
}

// ---- Explaining and coverage --------------------------------------------------------------------------------------

std::string explain(const CompiledRules& rules, const Decision& decision, const Reason& why) {
    if (why.rule >= rules.program.rules.size() || why.rule >= rules.labels.rules.size()) return std::string();
    const CompiledRule& rule = rules.program.rules[why.rule];
    std::string line = rules.labels.rules[why.rule] + ":";
    if (why.valueCount == 0) return line + " " + kWordAlways;
    for (uint32_t i = 0; i < why.valueCount && i < rule.readCount; ++i) {
        const uint32_t read = rule.firstRead + i;
        const uint32_t value = why.firstValue + i;
        if (read >= rules.program.reads.size() || value >= decision.values.size()) break;
        line += (i ? "; " : " ") + rules.labels.reads[read] + " is " +
                valueText(rules.program.reads[read], rules.labels.readWords[read], decision.values[value]);
        if (read < rules.labels.readWritten.size() && !rules.labels.readWritten[read].empty())
            line += " (" + rules.labels.readWritten[read] + ")";
    }
    return line;
}

void coverageGaps(const RuleCoverage& coverage, std::size_t ruleCount, std::vector<RuleIndex>& neverHeld,
                  std::vector<RuleIndex>& heldNeverWon) {
    neverHeld.clear();
    heldNeverWon.clear();
    for (std::size_t r = 0; r < ruleCount; ++r) {
        const uint64_t held = r < coverage.held.size() ? coverage.held[r] : 0;
        const uint64_t won = r < coverage.won.size() ? coverage.won[r] : 0;
        if (held == 0) neverHeld.push_back(static_cast<RuleIndex>(r));
        else if (won == 0) heldNeverWon.push_back(static_cast<RuleIndex>(r));
    }
}

}  // namespace driving
