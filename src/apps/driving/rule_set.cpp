#include "apps/driving/rule_set.h"

#include "hash.h"

#include <algorithm>
#include <cmath>

namespace driving {

namespace {

// THE ONE UNIT TABLE: each word a rule set may write, the number that turns it into metres / seconds / m/s / m/s2 /
// a plain ratio, and what it measures. Adding a unit is adding a row here.
struct UnitRow {
    const char* word;
    double factor;
    UnitFamily family;
};
constexpr double kMetresPerFoot = 0.3048;
constexpr double kMetresPerSecondPerMph = 0.44704;
constexpr double kSecondsPerHour = 3600.0;
constexpr double kMetresPerKilometre = 1000.0;
constexpr double kIdentity = 1.0;
const UnitRow kUnits[] = {
    {"s", kIdentity, UnitFamily::Time},
    {"m", kIdentity, UnitFamily::Length},
    {"ft", kMetresPerFoot, UnitFamily::Length},
    {"mph", kMetresPerSecondPerMph, UnitFamily::Speed},
    {"km/h", kMetresPerKilometre / kSecondsPerHour, UnitFamily::Speed},
    {"m/s", kIdentity, UnitFamily::Speed},
    {"m/s2", kIdentity, UnitFamily::Acceleration},
    {"ft/s2", kMetresPerFoot, UnitFamily::Acceleration},
    {"ratio", kIdentity, UnitFamily::Ratio},
};

// Each family in words, and the unit its numbers are stored in, in UnitFamily order.
struct FamilyRow {
    const char* words;
    const char* storedUnit;
};
const FamilyRow kFamilies[kUnitFamilyCount] = {
    {"a length", "m"},
    {"a time", "s"},
    {"a speed", "m/s"},
    {"an acceleration", "m/s2"},
    {"a ratio", "ratio"},
};

// A person's 64-bit id is folded into the 32-bit hash as its two halves.
constexpr uint32_t kHalfWordBits = 32;

// A name folded into one number by hashPair, character by character (the hash module has no string hash).
uint32_t nameHash(const std::string& s) {
    uint32_t h = 0;
    for (std::size_t i = 0; i < s.size(); ++i) h = engine::hashPair(h, static_cast<unsigned char>(s[i]));
    return h;
}

// Lookup in a vector sorted by `name`.
template <typename T>
const T* findByName(const std::vector<T>& sorted, const std::string& name) {
    const typename std::vector<T>::const_iterator it =
        std::lower_bound(sorted.begin(), sorted.end(), name,
                         [](const T& item, const std::string& n) { return item.name < n; });
    if (it == sorted.end() || it->name != name) return nullptr;
    return &*it;
}

}  // namespace

bool unitFactor(const std::string& word, double& factor) {
    for (const UnitRow& row : kUnits) {
        if (word == row.word) {
            factor = row.factor;
            return true;
        }
    }
    return false;
}

bool unitFamily(const std::string& word, UnitFamily& family) {
    for (const UnitRow& row : kUnits) {
        if (word == row.word) {
            family = row.family;
            return true;
        }
    }
    return false;
}

const char* unitFamilyWords(UnitFamily family) { return kFamilies[static_cast<uint8_t>(family)].words; }
const char* unitFamilyStoredUnit(UnitFamily family) { return kFamilies[static_cast<uint8_t>(family)].storedUnit; }

std::string unitWords() {
    std::string out;
    for (const UnitRow& row : kUnits) {
        if (!out.empty()) out += ", ";
        out += row.word;
    }
    return out;
}

// ---- Right of way's words ---------------------------------------------------------------------------------------------

namespace {

// THE QUESTIONS' WORDS, in SettleQuestion order.
const char* const kSettleWords[kSettleQuestionCount] = {"light", "sign_rank", "movement", "kerb_side", "arrived_first"};
// THE MOVEMENTS' WORDS, in Movement order.
const char* const kMovementWords[kMovementCount] = {"straight", "near_turn", "far_turn", "u_turn"};

template <std::size_t N>
std::string wordList(const char* const (&words)[N]) {
    std::string out;
    for (const char* w : words) out += (out.empty() ? "" : ", ") + std::string(w);
    return out;
}

}  // namespace

const char* settleQuestionWord(SettleQuestion q) {
    const uint8_t i = static_cast<uint8_t>(q);
    return i < kSettleQuestionCount ? kSettleWords[i] : "";
}

bool settleQuestionOf(const std::string& word, SettleQuestion& out) {
    for (uint8_t i = 0; i < kSettleQuestionCount; ++i)
        if (word == kSettleWords[i]) {
            out = static_cast<SettleQuestion>(i);
            return true;
        }
    return false;
}

std::string settleQuestionWords() { return wordList(kSettleWords); }

const char* movementWord(Movement m) {
    const uint8_t i = static_cast<uint8_t>(m);
    return i < kMovementCount ? kMovementWords[i] : "";
}

bool movementOf(const std::string& word, Movement& out) {
    for (uint8_t i = 0; i < kMovementCount; ++i)
        if (word == kMovementWords[i]) {
            out = static_cast<Movement>(i);
            return true;
        }
    return false;
}

std::string movementWords() { return wordList(kMovementWords); }

bool ControlKind::hasLights() const {
    return std::find(settle.begin(), settle.end(), SettleQuestion::Light) != settle.end();
}

const ControlKind* RuleSet::findControl(const std::string& controlName) const { return findByName(controls, controlName); }

uint32_t RuleSet::movementRankOf(Movement m) const {
    for (std::size_t i = 0; i < movementRank.size(); ++i)
        if (movementRank[i] == m) return static_cast<uint32_t>(i);
    return static_cast<uint32_t>(movementRank.size());
}

const Device* RuleSet::findDevice(const std::string& deviceName) const { return findByName(devices, deviceName); }
const Number* RuleSet::findNumber(const std::string& numberName) const { return findByName(numbers, numberName); }
const PersonalSetting* RuleSet::findPersonal(const std::string& settingName) const {
    return findByName(personality, settingName);
}
const Number* RuleSet::findDefaultLimit(const std::string& roadClass) const { return findByName(defaultLimits, roadClass); }

int RuleSet::rankIndex(const std::string& rankName) const {
    for (std::size_t i = 0; i < ranks.size(); ++i)
        if (ranks[i] == rankName) return static_cast<int>(i);
    return kNoRank;
}

std::size_t RuleSet::unsourcedNumberCount() const {
    std::size_t count = 0;
    for (const std::vector<Number>* table : {&numbers, &defaultLimits})
        for (const Number& n : *table)
            if (!n.hasSource) ++count;
    return count;
}

double personalValue(const PersonalSetting& setting, PersonId personId) {
    return personalValue(setting, nameHash(setting.name), personId);
}

uint32_t settingNameHash(const std::string& settingName) { return nameHash(settingName); }

double personalValue(const PersonalSetting& setting, uint32_t settingHash, PersonId personId) {
    const double span = setting.high - setting.low;
    if (!(span > 0)) return setting.low;   // no range: everyone has the one value
    const uint32_t personBits = engine::hashPair(static_cast<uint32_t>(personId),
                                                 static_cast<uint32_t>(personId >> kHalfWordBits));
    const double u = engine::unit24(engine::hashPair(personBits, settingHash));
    // The triangular spread, turned from a uniform number by its inverse: the peak sits at `usual`, which a share
    // `peak` of the people are below.
    const double peak = (setting.usual - setting.low) / span;
    double v;
    if (u < peak) v = setting.low + std::sqrt(u * span * (setting.usual - setting.low));
    else v = setting.high - std::sqrt((1.0 - u) * span * (setting.high - setting.usual));
    return std::min(setting.high, std::max(setting.low, v));
}

}  // namespace driving
