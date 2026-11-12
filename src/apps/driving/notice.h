#ifndef RAYTRACER_APPS_DRIVING_NOTICE_H
#define RAYTRACER_APPS_DRIVING_NOTICE_H

// WHAT A DRIVER NOTICES (driving-model-design.md 4 "Facts" and 6.2; issue #251).
//
// When a driver decides, it first notices things: named values called FACTS ("the car ahead is 12 m away", "this lane
// ends in 40 m"). Rules written as data refer to facts by name; the C++ that works one out is one small function
// registered under that name in a FactRegistry. A fact's SLOT is its place in registration order.
//
// noticeFacts works out, for one driver, only the facts asked for. A fact function is handed the driver's view (all
// read-only: it cannot change a car) and a scratch pad for work several facts share (the car ahead is looked up once
// and read by four facts). The scratch is cleared between drivers.
//
// Every fact says what it was about (a car, a path, so many seconds or metres), so a driver card can show "car 14,
// 12.0 m ahead" with no extra code, and every fact can be ABSENT ("no car ahead") rather than a made-up number.
//
// Same result every run: nothing here walks an unordered container, and the facts read the per-path lists, which are
// sorted whole, so the order cars are stored in changes nothing.
//
// Units: metres, seconds, m/s, m/s2.

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/rule_set.h"
#include "apps/driving/traffic_state.h"
#include "apps/driving/vehicle_class.h"
#include "rt_math.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace driving {

using engine::Real;

struct RoadRules;      // road_rules.h: the graph's regulation kinds bound to the rule set's devices
struct RightOfWay;     // right_of_way.h: who gives way to whom at every junction
struct JunctionPlans;  // junction_plan.h: one driver's plans for the junctions ahead
class AnnouncementBoard;   // announce.h: who will be on which crossing point, and when
struct GiveWayVerdict;     // give_way.h: what one driver judged about giving way at the junctions ahead

// WHAT KIND OF VALUE a fact holds.
enum class FactKind : uint8_t {
    Number,   // a number, in metres, seconds, m/s, m/s2 or a ratio
    YesNo,    // yes or no
    Word,     // one word from the fact's own short list
    Place,    // a distance ahead along my own way, in metres from my front bumper (where to stop, where a lane ends)
};
constexpr uint8_t kFactKindCount = 4;

// Which word of a Word fact's list, or none.
using WordIndex = uint32_t;
constexpr WordIndex kNoWord = 0xFFFFFFFFu;

// ONE THING A DRIVER HAS NOTICED: its value and what it was about. When `present` is false the fact is absent
// (nothing to report, or not asked for) and no other field means anything.
struct NoticedFact {
    FactKind kind = FactKind::Number;
    bool present = false;
    Real number = 0;           // a Number's value, or a Place's distance ahead in metres
    bool yes = false;          // a YesNo's value
    WordIndex word = kNoWord;  // a Word's value
    // What it was about, whichever apply.
    CarId aboutCar;            // not valid: about no car
    engine::PathId aboutPath;  // not valid: about no path
    bool hasSeconds = false;
    Real seconds = 0;
    bool hasMetres = false;
    Real metres = 0;
};
bool operator==(const NoticedFact& a, const NoticedFact& b);   // every field that means something, exactly
inline bool operator!=(const NoticedFact& a, const NoticedFact& b) { return !(a == b); }

// THE PATHS AHEAD OF A DRIVER, in order: the first is the path its front bumper is on, each next one leads on from the
// one before. The caller supplies it (routes are a later task). When it is empty or does not start on the driver's own
// path, the facts read it as the driver's own path alone.
struct WayAhead {
    std::vector<engine::PathId> paths;
};
// For tests and for a car with no route: from `path`, follow `next` while a path has exactly one, until the paths
// reach `lookAhead` metres past `along` (or kMaxFollowedPaths paths are taken, which only a loop of empty paths reaches).
constexpr std::size_t kMaxFollowedPaths = 256;
WayAhead followOnlyWay(const engine::LaneGraph& roads, engine::PathId path, Real along, Real lookAhead);

// WHAT ONE DRIVER CAN SEE THIS TICK, all read-only.
struct DriverView {
    const Car& me;                       // my own car's record
    const VehicleClass& myClass;         // ... and its class
    PersonId person;                     // who is driving (for personal settings, through personalValue)
    const WayAhead& way;                 // the paths ahead of me, mine first
    const engine::LaneGraph& roads;
    const std::vector<Car>& cars;        // every car's record
    const PathLists& lists;              // ... and each path's cars in order (built from `cars`)
    const RuleSet& rules;
    const RoadRules& roadRules;          // the graph's regulation kinds, bound to the rule set's devices
    Real lookAhead;                      // how far ahead I look, in metres
    // Where my trip goes and the holds the host has placed on it; null when I am on no trip (then the facts about
    // them are absent).
    const PathPoint* destination = nullptr;
    const std::vector<Hold>* holds = nullptr;
    // My plans for the junctions on my way ahead, brought up to date before I notice anything, and the table of who gives
    // way to whom (issue #260; the junction facts of task D3 read them). Null when there are none (no rules, no trip).
    const JunctionPlans* junctions = nullptr;
    const RightOfWay* rightOfWay = nullptr;
    // This tick's board of announcements, and what I judged from it about giving way at the junctions ahead (issue #262:
    // the junction facts of give_way.h read the judgement). Null when the junction step is off (no rules, no trip, or a
    // host that did not register it).
    const AnnouncementBoard* board = nullptr;
    const GiveWayVerdict* giveWay = nullptr;
};

// WORK SEVERAL FACTS SHARE for one driver, filled on first use and cleared between drivers. It holds only things
// worked out from the view, never a car.
struct NoticeScratch {
    bool haveCarAhead = false;
    Neighbour carAhead;                          // the car ahead, once looked up
    bool haveRoadLimit = false;
    Real roadLimit = 0;                          // the limit where my front is, once worked out
    std::vector<engine::PathId> pathsAfterMine;  // the way ahead without my own path (kept to reuse its memory)
    std::vector<uint8_t> worked;                 // per slot: worked out already for this driver
    std::size_t carAheadLookups = 0;             // how many times carAhead was called for this driver (tests count)

    void clearForDriver();
};

// A FACT FUNCTION: looks at the view (and the shared scratch) and fills `out`, which arrives absent with its kind set.
// It sets `present` when it has something to report.
using NoticeFn = void (*)(const DriverView& view, NoticeScratch& scratch, NoticedFact& out);

// A NUMBER OR PERSONAL SETTING a fact needs from the rule set, by name, and what kind of unit it must have.
struct FactNeed {
    std::string name;
    UnitFamily family = UnitFamily::Ratio;
};

// ONE FACT AS REGISTERED.
struct FactSpec {
    std::string name;                  // snake_case, unique in the registry
    FactKind kind = FactKind::Number;
    NoticeFn notice = nullptr;
    std::string meaning;               // one line, plain language, for the driver card and the docs
    // What a Number or Place fact measures, so a rule comparing it with a number of another kind (a distance with a
    // time) is refused at load. A Place is always a length. Means nothing for YesNo and Word facts.
    UnitFamily unit = UnitFamily::Ratio;
    std::vector<std::string> words;    // a Word fact's list (empty otherwise)
    std::vector<FactNeed> numbers;     // rule-set numbers it reads
    std::vector<FactNeed> settings;    // personal settings it reads
    // A Place fact a rule stops by: what that stop waits for (traffic_state.h). GivingWay: the car the fact is about
    // (NoticedFact::aboutCar) is the one given way to, and the wait-for graph draws a right-of-way arrow to it.
    StopFor stopFor = StopFor::Place;
};

// SOMETHING OTHER THAN A FACT THAT READS THE RULE SET, declaring what it reads the way a fact does, so the same check
// (buildRoadRules) reports a rule set that lacks one, naming who needs it. The move step is one (act.h,
// registerMoveNeeds): it reads the gap a driver keeps and the standstill gap on every tick.
struct ReaderSpec {
    std::string name;                  // unique among the readers; named in the problems reported
    std::string meaning;               // one line, plain language
    std::vector<FactNeed> numbers;     // rule-set numbers it reads
    std::vector<FactNeed> settings;    // personal settings it reads
};

// A fact's place in registration order.
using FactSlot = uint32_t;
constexpr FactSlot kNoFact = 0xFFFFFFFFu;

// EVERY FACT A DRIVER CAN NOTICE, in registration order, and every other reader of the rule set. An object the host
// owns: no global table.
class FactRegistry {
public:
    // Adds a fact at the end and gives its slot. Refused (kNoFact, with the reason in `why` when given) for an empty
    // name, a name already registered, no function, or a Place fact whose unit is not a length.
    FactSlot add(FactSpec spec, std::string* why = nullptr);
    // The slot of the fact of that name, or kNoFact.
    FactSlot find(const std::string& name) const;
    std::size_t size() const { return facts_.size(); }
    const FactSpec& at(FactSlot slot) const { return facts_[slot]; }
    const std::vector<FactSpec>& all() const { return facts_; }

    // Adds a reader at the end. Refused (false, the reason in `why`) for an empty name or one already a reader's.
    bool addReader(ReaderSpec spec, std::string* why = nullptr);
    // The reader of that name, or null.
    const ReaderSpec* findReader(const std::string& name) const;
    const std::vector<ReaderSpec>& readers() const { return readers_; }

private:
    std::vector<FactSpec> facts_;
    std::vector<ReaderSpec> readers_;
};

// WORKS OUT THE FACTS ASKED FOR, for one driver. `wanted` lists slots (a rule set asks only for those it uses; repeats
// and unknown slots are passed over). `out` gets one NoticedFact per registry slot, each with its kind; those not asked
// for stay absent. A fact not asked for is never worked out. The scratch is cleared first; the version without
// one uses its own.
void noticeFacts(const DriverView& view, const FactRegistry& registry, const std::vector<FactSlot>& wanted,
                 std::vector<NoticedFact>& out, NoticeScratch& scratch);
void noticeFacts(const DriverView& view, const FactRegistry& registry, const std::vector<FactSlot>& wanted,
                 std::vector<NoticedFact>& out);

// ---- The built-in facts --------------------------------------------------------------------------------------------

// The names of the built-in facts, each next to what it needs.
constexpr const char* kFactCarAhead = "car_ahead";            // yes/no: a car ahead within my look-ahead
constexpr const char* kFactGapAhead = "gap_ahead";            // m: my front to its rear (about that car)
constexpr const char* kFactSpeedAhead = "speed_ahead";        // m/s: its speed (about that car)
constexpr const char* kFactClosingSpeed = "closing_speed";    // m/s: my speed minus its speed (about that car)
constexpr const char* kFactRoadLimit = "road_limit";          // m/s: the limit in force where my front is
constexpr const char* kFactMyLimit = "my_limit";              // m/s: road_limit times my speed habit
constexpr const char* kFactSlowerAhead = "slower_ahead";      // m/s: the speed I may do now for a lower limit ahead
constexpr const char* kFactCornerSpeed = "corner_speed";      // m/s: the speed I may do now for a bend ahead
constexpr const char* kFactLaneEndsAhead = "lane_ends_ahead"; // yes/no: my way runs out within my look-ahead
constexpr const char* kFactLaneEnd = "lane_end";              // place: where it runs out
constexpr const char* kFactMySpeed = "my_speed";              // m/s
constexpr const char* kFactStanding = "standing";             // yes/no: my speed is below kStandingBelow
// Where my trip ends, metres ahead along my way, when it is within my look-ahead (about its path). A rule set says
// { name = "arrive", give = { stop_at = "destination" } }: the library never stops at a destination by itself.
constexpr const char* kFactDestination = "destination";       // place
// The nearest hold the host has placed on my trip that is still ahead, within my look-ahead (about its path). A rule
// set says { name = "held", give = { stop_at = "hold_ahead" } }.
constexpr const char* kFactHoldAhead = "hold_ahead";          // place
// I am standing (as `standing`) with my destination no further ahead than kArrivedWithin. The library ends a driven
// car's trip when this is yes at a decision (a Parked event), so it is always worked out, whatever the rules use.
constexpr const char* kFactArrived = "arrived";               // yes/no

// The personal setting my_limit and slower_ahead read: how far above or below the limit this person drives (a ratio).
constexpr const char* kSettingSpeedHabit = "speed_habit";
// The rule-set number corner_speed reads: the sideways acceleration a driver takes a bend at comfortably (m/s2).
constexpr const char* kNumberComfortableSideAccel = "comfortable_side_accel";
// `standing` is yes below this speed (m/s): slower than a walking crawl, the car is as good as stopped.
constexpr Real kStandingBelow = 0.1;
// `arrived` is yes when the destination is no further than this from my front (m): a car that stopped for it is
// there. (A destination this little BEHIND my front counts too, so rounding never strands a car that stopped on it.)
constexpr Real kArrivedWithin = 0.5;

// Adds the built-in facts above, in the order listed. False (with the reasons, one line each) when one is refused,
// which only happens when one of their names was registered before.
bool registerBuiltInFacts(FactRegistry& registry, std::vector<std::string>* problems = nullptr);

}  // namespace driving

#endif
