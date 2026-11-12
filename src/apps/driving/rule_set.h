#ifndef RAYTRACER_APPS_DRIVING_RULE_SET_H
#define RAYTRACER_APPS_DRIVING_RULE_SET_H

// THE RULES OF THE ROAD, AS PLAIN DATA (driving-model-design.md 6.1 a, c, d; issue #250). A country's rules live in
// one Lua file (a "rule set"); the reader in scripting/rule_set_lua.h reads it once when a level loads and fills these
// structs. Nothing here touches Lua, and no Lua runs while cars drive.
//
// This file holds four parts of a rule set:
//   - which side of the road traffic keeps, and the sign ranks in order (which sign goes first);
//   - the DEVICES: things that can stand on or be painted on the road, and what each one means;
//   - the NUMBERS: every tunable number, stored in metres / seconds / metres per second (squared) / plain ratios,
//     with the unit and the source it was written with;
//   - the PERSONALITY settings: each driver setting's usual value and the range people fall in.
//
// What a device can mean is a small fixed vocabulary of EFFECTS (stop line, rank, top speed, closed to, open to,
// forbids). That vocabulary is the only thing C++ knows: the device names and the vehicle tag names are data.
//
// Every list kept by name is stored in the alphabetical order of the names (Lua table order is not stable), so the
// result is the same every run. The lookups find a name by binary search over that order and say "not found" with a
// null pointer or -1.
//
// Units: metres, seconds.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace driving {

// WHICH SIDE OF THE ROAD traffic keeps. The one place the two words are allowed: it is the fact being recorded.
enum class DrivesOn { Left, Right };

// Words of the rule-set language that the reader checks for. Kept here so C++ has each in one place.
constexpr const char* kWordAll = "all";       // closed_to / open_to: every kind of vehicle
constexpr const char* kWordValue = "value";   // top_speed: the regulation's own number is the speed

// "No rank": a device that does not say which sign rank it carries.
constexpr int kNoRank = -1;

// WHO A DEVICE SHUTS OUT OR LETS IN: nothing said, everyone, or the listed vehicle tags (names from the data).
struct VehicleScope {
    enum class Kind { Unsaid, All, Listed };
    Kind kind = Kind::Unsaid;
    std::vector<std::string> tags;   // when Listed: the tag names, as written
};

// WHAT ONE THING ON THE ROAD MEANS and how it looks. Only the effects the file gave are set.
struct Device {
    std::string name;
    bool stopLine = false;           // a stop line goes with it
    int rank = kNoRank;              // index into RuleSet::ranks, or kNoRank
    bool topSpeedIsValue = false;    // the regulation's own number is the top speed
    VehicleScope closedTo;
    VehicleScope openTo;
    std::vector<std::string> forbids;   // names of things forbidden, as written
    std::string sign;                // the sign's code, as written ("" when none)
    std::vector<std::string> paint;  // the paint's marks, as written
};

// ONE TUNABLE NUMBER. `value` is always in metres, seconds, m/s, m/s2 or a plain ratio; the original value and unit
// are kept for the driver card to show ("25 mph").
struct Number {
    std::string name;
    double value = 0;                // converted
    double writtenValue = 0;         // as written
    std::string unit;                // as written
    std::string source;              // where it came from ("" when the file gave none)
    bool hasSource = false;
};

// ONE DRIVER SETTING: its usual value and the range people fall in. Values are converted like Numbers.
struct PersonalSetting {
    std::string name;
    double usual = 0;
    double low = 0;
    double high = 0;
    double writtenUsual = 0;
    double writtenLow = 0;
    double writtenHigh = 0;
    std::string unit;                // as written
};

struct RuleSet {
    std::string name;
    DrivesOn drivesOn = DrivesOn::Right;
    std::vector<std::string> ranks;          // the one that goes first listed first (written order)
    std::vector<Device> devices;             // sorted by name
    std::vector<Number> numbers;             // sorted by name
    std::vector<PersonalSetting> personality;   // sorted by name

    const Device* findDevice(const std::string& deviceName) const;
    const Number* findNumber(const std::string& numberName) const;
    const PersonalSetting* findPersonal(const std::string& settingName) const;
    // Position of a rank name in `ranks` (0 goes first), or kNoRank.
    int rankIndex(const std::string& rankName) const;
    // How many numbers came without a source. A later check fails a real rule set when this is not zero.
    std::size_t unsourcedNumberCount() const;
};

// UNITS the rule-set language knows, in ONE table (rule_set.cpp). Says whether `word` is one, and what to multiply a
// number written in it by to get metres / seconds / m/s / m/s2 / a plain ratio.
bool unitFactor(const std::string& word, double& factor);
// The unit words, comma separated in table order, for error messages.
std::string unitWords();

// WHAT A UNIT MEASURES, so a check can tell a speed from a time: every unit word belongs to one of these, in the same
// table as its factor.
enum class UnitFamily : uint8_t { Length, Time, Speed, Acceleration, Ratio };
constexpr uint8_t kUnitFamilyCount = 5;
// Says whether `word` is a unit, and which family it is in.
bool unitFamily(const std::string& word, UnitFamily& family);
// The family in words for messages, with its article ("a speed", "an acceleration"), and the unit it is stored in.
const char* unitFamilyWords(UnitFamily family);
const char* unitFamilyStoredUnit(UnitFamily family);

// WHO IS AT THE WHEEL: the host's stable number for a person (the city sim's AgentId). The library never reads meaning
// into it; it hands it back on events and works out the person's own driving habits from a hash of it.
using PersonId = uint64_t;

// ONE PERSON'S OWN VALUE of a setting: the same person always gets the same value, it lies in [low, high], and values
// bunch around `usual` (a triangular spread whose peak is at `usual`). Mixes the person's id with a hash of the
// setting's name, so two settings are independent for one person. Stateless, so it costs no memory.
double personalValue(const PersonalSetting& setting, PersonId personId);
// The same, with the hash of the setting's name worked out beforehand (settingNameHash), so code that runs every tick
// never reads the name.
uint32_t settingNameHash(const std::string& settingName);
double personalValue(const PersonalSetting& setting, uint32_t nameHash, PersonId personId);

}  // namespace driving

#endif
