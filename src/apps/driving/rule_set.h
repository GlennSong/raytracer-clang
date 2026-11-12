#ifndef RAYTRACER_APPS_DRIVING_RULE_SET_H
#define RAYTRACER_APPS_DRIVING_RULE_SET_H

// THE RULES OF THE ROAD, AS PLAIN DATA (driving-model-design.md 6.1 a, c, d; issue #250). A country's rules live in
// one Lua file (a "rule set"); the reader in scripting/rule_set_lua.h reads it once when a level loads and fills these
// structs. Nothing here touches Lua, and no Lua runs while cars drive.
//
// This file holds these parts of a rule set:
//   - which side of the road traffic keeps, and the sign ranks in order (which sign goes first);
//   - RIGHT OF WAY: each kind of junction control and the questions that settle who gives way there, and the order the
//     kinds of turn go in (right_of_way.h turns them into a table per junction at load);
//   - the DEVICES: things that can stand on or be painted on the road, and what each one means;
//   - the NUMBERS: every tunable number, stored in metres / seconds / metres per second (squared) / plain ratios,
//     with the unit and the source it was written with;
//   - the DEFAULT LIMITS: the speed limit on a road with none posted, by the road's class, written like the numbers
//     (an optional table: a rule set for roads whose every path has a posted limit needs none);
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
// The table of limits on roads with none posted, by road class (RuleSet::defaultLimits).
constexpr const char* kKeyDefaultLimits = "default_limits";

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
    std::string source;              // where the values came from ("" when the file gave none)
    bool hasSource = false;
};

// ---- Right of way (design 6.1 b; issue #260) --------------------------------------------------------------------------
//
// WHO GIVES WAY AT A JUNCTION is settled by asking a short list of QUESTIONS in order (a control's `settle` list); the
// first that tells two turns apart decides. The questions are a small fixed vocabulary, because code answers each:
enum class SettleQuestion : uint8_t {
    Light,          // at load, per signal phase: a green goes before a red (a turn allowed on red gives way to all)
    SignRank,       // at load: the rank of the sign each approach carries (RuleSet::ranks; no sign shares the first)
    Movement,       // at load: the kind of turn, by RuleSet::movementRank (earlier goes first)
    KerbSide,       // at load: the one arriving from the other's kerb side goes first
    ArrivedFirst,   // while driving: whoever reached their line first
};
constexpr uint8_t kSettleQuestionCount = 5;
// The question's word in a rule-set file ("light", "sign_rank", ...); empty for an unknown one.
const char* settleQuestionWord(SettleQuestion q);
// The question a word names; false for a word that is none.
bool settleQuestionOf(const std::string& word, SettleQuestion& out);
// The words, comma separated, for messages.
std::string settleQuestionWords();

// THE KINDS OF TURN a rule set can name, as words: straight on, the near turn (stays by the kerb), the far turn
// (crosses the oncoming traffic), the U-turn. The lane graph's connectors carry the same four (engine::Turn).
enum class Movement : uint8_t { Straight, NearTurn, FarTurn, UTurn };
constexpr uint8_t kMovementCount = 4;
const char* movementWord(Movement m);   // "straight", "near_turn", "far_turn", "u_turn"
bool movementOf(const std::string& word, Movement& out);
std::string movementWords();
// A set of movements: bit (1 << movement).
using MovementMask = uint8_t;
inline bool hasMovement(MovementMask mask, Movement m) { return (mask & (1u << static_cast<uint8_t>(m))) != 0; }

// The one thing a turn on red may have to do first, as the rule-set file writes it: come to a full stop.
constexpr const char* kWordFullStop = "full_stop";

// ONE KIND OF JUNCTION CONTROL (a row of the rule set's `control` table): the questions that settle two turns, and, for
// a control with lights (one whose settle list asks `light`), how its turns are grouped into phases.
struct ControlKind {
    std::string name;
    std::vector<SettleQuestion> settle;   // in written order
    bool everyoneStops = false;           // every driver stops at the line first (D3 acts on it; the table does not)
    // LIGHTS. Turns that may go on a plain green and give way to what comes from the approach facing theirs (a far turn
    // on a green gives way to oncoming traffic): they may share a phase with what they meet from that approach.
    MovementMask givesWayOnGreen = 0;
    // Turns given their own green arrow: a phase of their own, before the phase their approach shares with the one facing
    // it, in which they go and what they meet waits.
    MovementMask arrow = 0;
    // A turn that may be made on a red, giving way to everything (turn_on_red = { movement = ..., after = ... }).
    bool turnOnRed = false;
    Movement turnOnRedMovement = Movement::NearTurn;
    bool turnOnRedAfterFullStop = false;  // ... only after coming to a full stop (the rule set's own rules act on it)
    // WHILE NO LIGHT RUNS A PROGRAMME (road_rules.h RoadRules::lightsRun: a host that registered no light clock,
    // signal_programme.h), a junction with this control is treated as the control named here (`when_dark`; a dark or
    // flashing-red signal is an all-way stop). Empty: it keeps its own row, its lights fixed at their first phase.
    std::string whenDark;

    bool hasLights() const;               // its settle list asks `light`
};

struct RuleSet {
    std::string name;
    DrivesOn drivesOn = DrivesOn::Right;
    std::vector<std::string> ranks;          // the one that goes first listed first (written order)
    std::vector<Device> devices;             // sorted by name
    std::vector<Number> numbers;             // sorted by name
    std::vector<PersonalSetting> personality;   // sorted by name
    std::vector<Number> defaultLimits;          // by road class name, sorted; each a speed
    // RIGHT OF WAY (an optional table: a rule set for roads with no junctions needs none). The kinds of junction control,
    // sorted by name, and the movements in the order they go first (written order; a movement not listed goes after
    // every listed one).
    std::vector<ControlKind> controls;
    std::vector<Movement> movementRank;
    // BETWEEN JUNCTIONS (optional; issue #284): how a meeting filed under no junction is settled -- two junctions' paths,
    // or a lane and a connector, crowded together (ADR-0400) -- as a settle list asked of each path's own approach and
    // movement (a lane: no sign, straight on). Only sign_rank, movement and arrived_first mean anything there. Absent
    // (hasBetweenJunctions false): such a meeting is asked while driving, falling back on the lower path id.
    bool hasBetweenJunctions = false;
    std::vector<SettleQuestion> betweenJunctions;

    const Device* findDevice(const std::string& deviceName) const;
    const ControlKind* findControl(const std::string& controlName) const;
    // Some control has lights (a host then registers the light clock, signal_programme.h, for them to run).
    bool anyLights() const;
    // A movement's place in movementRank (0 goes first); movementRank's size for one not listed.
    uint32_t movementRankOf(Movement m) const;
    const Number* findNumber(const std::string& numberName) const;
    const PersonalSetting* findPersonal(const std::string& settingName) const;
    // The default limit for road class `roadClass`, or null.
    const Number* findDefaultLimit(const std::string& roadClass) const;
    // Position of a rank name in `ranks` (0 goes first), or kNoRank.
    int rankIndex(const std::string& rankName) const;
    // How many numbers (default limits among them) came without a source. A later check fails a real rule set when
    // this is not zero.
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

// WHICH RULE: a rule's place in the rule set's `rules`, in written order (decide.h compiles them). Kept here so a car's
// record (traffic_state.h) can say which rule gave the answer that held it back.
using RuleIndex = uint32_t;
constexpr RuleIndex kNoRule = 0xFFFFFFFFu;

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
