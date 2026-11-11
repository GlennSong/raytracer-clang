#include "apps/driving/rule_set.h"

#include "hash.h"

#include <algorithm>
#include <cmath>

namespace driving {

namespace {

// THE ONE UNIT TABLE: each word a rule set may write, and the number that turns it into metres / seconds / m/s / m/s2 /
// a plain ratio. Adding a unit is adding a row here.
struct UnitRow {
    const char* word;
    double factor;
};
constexpr double kMetresPerFoot = 0.3048;
constexpr double kMetresPerSecondPerMph = 0.44704;
constexpr double kSecondsPerHour = 3600.0;
constexpr double kMetresPerKilometre = 1000.0;
constexpr double kIdentity = 1.0;
const UnitRow kUnits[] = {
    {"s", kIdentity},
    {"m", kIdentity},
    {"ft", kMetresPerFoot},
    {"mph", kMetresPerSecondPerMph},
    {"km/h", kMetresPerKilometre / kSecondsPerHour},
    {"m/s", kIdentity},
    {"m/s2", kIdentity},
    {"ft/s2", kMetresPerFoot},
    {"ratio", kIdentity},
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

std::string unitWords() {
    std::string out;
    for (const UnitRow& row : kUnits) {
        if (!out.empty()) out += ", ";
        out += row.word;
    }
    return out;
}

const Device* RuleSet::findDevice(const std::string& deviceName) const { return findByName(devices, deviceName); }
const Number* RuleSet::findNumber(const std::string& numberName) const { return findByName(numbers, numberName); }
const PersonalSetting* RuleSet::findPersonal(const std::string& settingName) const {
    return findByName(personality, settingName);
}

int RuleSet::rankIndex(const std::string& rankName) const {
    for (std::size_t i = 0; i < ranks.size(); ++i)
        if (ranks[i] == rankName) return static_cast<int>(i);
    return kNoRank;
}

std::size_t RuleSet::unsourcedNumberCount() const {
    std::size_t count = 0;
    for (const Number& n : numbers)
        if (!n.hasSource) ++count;
    return count;
}

double personalValue(const PersonalSetting& setting, uint64_t personId) {
    const double span = setting.high - setting.low;
    if (!(span > 0)) return setting.low;   // no range: everyone has the one value
    const uint32_t personBits = engine::hashPair(static_cast<uint32_t>(personId),
                                                 static_cast<uint32_t>(personId >> kHalfWordBits));
    const double u = engine::unit24(engine::hashPair(personBits, nameHash(setting.name)));
    // The triangular spread, turned from a uniform number by its inverse: the peak sits at `usual`, which a share
    // `peak` of the people are below.
    const double peak = (setting.usual - setting.low) / span;
    double v;
    if (u < peak) v = setting.low + std::sqrt(u * span * (setting.usual - setting.low));
    else v = setting.high - std::sqrt((1.0 - u) * span * (setting.high - setting.usual));
    return std::min(setting.high, std::max(setting.low, v));
}

}  // namespace driving
