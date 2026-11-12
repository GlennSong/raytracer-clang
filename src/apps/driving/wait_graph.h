#ifndef RAYTRACER_APPS_DRIVING_WAIT_GRAPH_H
#define RAYTRACER_APPS_DRIVING_WAIT_GRAPH_H

// WHO IS WAITING FOR WHOM (driving-model-design.md 9.6 and 13; issue #256): the wait-for graph, as pure functions over
// a list of waiting cars, each with the one thing it waits for.
//
// A WAITING car is a driven car at rest: slower than the `standing` fact's threshold (kStandingBelow) and not speeding
// up (a car moving off from rest is still slower than that for a tick or two, and is not waiting for anything). A car
// whose driver has not yet made its first decision on this trip is not counted: it decides on its next tick.
//
// Each waiting car has exactly one REASON, read from what held it back on its last tick (Car::binding):
//   - an ARROW to another car, of some KIND. Each kind is either a matter of SPACE (the car is in my way; there is no
//     room beyond for me) or of RIGHT OF WAY (I am giving way to it). The tick gives "behind" (the car ahead holds me back)
//     and "giving way to" (a stop at a junction's line that names the car given way to: Binding::stopFor, issue #262);
//     and "waiting for room behind the tail of" (a stop at a junction's line for room beyond it that names the car whose
//     tail bounds the room: StopFor::NoRoom, issue #264). Adding a kind is one enum value and one row of kArrowKinds.
//   - a LEAF: something that is not another car (a place to stop by given by a rule, a top speed, the end of its way).
//   - NONE: held back by nothing at all. That is the worst bug there is (a car standing still for no reason), so those
//     cars are listed on their own.
//
// A LOOP is a ring of arrows: each car waits on the next, back to the first. A loop whose arrows are all about space is
// GRIDLOCK: every rule obeyed, the road simply full (a road-design or demand problem). A loop with any right-of-way
// arrow is DEADLOCK: a bug.
//
// Each waiting car has at most one arrow, so the graph is a set of chains, some ending in loops: finding the loops and
// measuring the chains is following each chain once, in time proportional to the number of cars (plus finding a car
// by its id in the sorted list), with no recursion.
//
// Same result every run: the list is sorted by car id before anything is read from it; loops come out in order of
// their lowest car, each starting from it; every list in the report is in car-id order.
//
// Nothing here changes what a car does. Text is made only by describeWaiting, when someone asks.

#include "apps/driving/rule_set.h"        // RuleIndex
#include "apps/driving/traffic_state.h"   // CarId, Car

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace driving {

// WHAT AN ARROW IS ABOUT.
enum class WaitSort : uint8_t {
    Space,        // the other car is in my way, or there is no room past it: no rule is broken by waiting
    RightOfWay,   // I am giving way to the other car: a loop of these is a deadlock, a bug
};

// THE KINDS OF ARROW from one waiting car to another.
enum class ArrowKind : uint8_t {
    Behind,         // the car ahead is what limits me
    GivingWay,      // I am giving way to that car: stopped at a junction's line for it (#262)
    NoRoomBeyond,   // there is no room past the junction: I wait for the tail of that car (#264)
};
constexpr uint8_t kArrowKindCount = 3;

// One row per arrow kind, in enum order: the words for it ("car 3 <words> car 9") and what it is about.
struct ArrowKindInfo {
    ArrowKind kind;
    const char* words;
    WaitSort sort;
};
constexpr ArrowKindInfo kArrowKinds[kArrowKindCount] = {
    {ArrowKind::Behind, "behind", WaitSort::Space},
    {ArrowKind::GivingWay, "giving way to", WaitSort::RightOfWay},
    {ArrowKind::NoRoomBeyond, "waiting for room behind the tail of", WaitSort::Space},
};
const ArrowKindInfo& arrowKindInfo(ArrowKind kind);

// THE KINDS OF LEAF: a waiting car held by something that is not another car.
enum class LeafKind : uint8_t {
    Stop,       // a place to stop by, given by a rule (a hold, the destination, a lane's end)
    TopSpeed,   // a top speed (of zero, for a car at rest), given by a rule or the vehicle class
    WayEnd,     // the end of the paths it knows its way along
};
constexpr uint8_t kLeafKindCount = 3;

enum class ReasonSort : uint8_t { None, Arrow, Leaf };

// WHAT ONE WAITING CAR WAITS FOR.
struct WaitReason {
    ReasonSort sort = ReasonSort::None;
    ArrowKind arrow = ArrowKind::Behind;   // Arrow: what kind
    CarId toCar;                           // Arrow: which car
    LeafKind leaf = LeafKind::Stop;        // Leaf: what kind
    RuleIndex rule = kNoRule;              // Leaf Stop or TopSpeed: which rule (kNoRule: the vehicle class's own)
};

struct WaitingCar {
    CarId car;
    WaitReason why;
};

// Is this car waiting (see the top of this file)?
bool isWaiting(const Car& car);
// Its reason, read from what held it back.
WaitReason waitReasonOf(const Car& car);
// THE SNAPSHOT: every waiting car among `cars`, with its reason, in the order given (Driving keeps cars in id order).
std::vector<WaitingCar> waitingCars(const std::vector<Car>& cars);

enum class LoopClass : uint8_t {
    Gridlock,   // every arrow is about space: the road is full, no rule is broken
    Deadlock,   // some arrow is about right of way: a bug
};
// A loop's class from the kinds of its arrows.
LoopClass classifyLoop(const std::vector<ArrowKind>& arrows);

// ONE LOOP: cars[i] waits on cars[i + 1] (the last on the first) by arrows[i]. cars[0] is its lowest car id.
struct WaitLoop {
    std::vector<CarId> cars;
    std::vector<ArrowKind> arrows;
    LoopClass kind = LoopClass::Gridlock;
};

// Every loop among the waiting cars, each once, in order of its lowest car. A car listed twice is read once (its first
// listing); an arrow to a car not in the list (moving, parked, or unknown) ends a chain.
std::vector<WaitLoop> findLoops(const std::vector<WaitingCar>& waiting);

// WHAT A CHAIN OF WAITING ENDS AT: its HEAD car's reason (the waiting car nearest the front).
enum class HeadKind : uint8_t {
    Leaf,            // a leaf: the head waits for a stop, a top speed or the end of its way
    CarNotWaiting,   // the head's arrow points to a car that is not waiting (it is moving, or parked)
    Loop,            // the chain runs into a loop (the head is the last car before it)
    NoReason,        // the head is waiting for no reason (the bug)
};

// ONE KIND OF QUEUE HEAD, and how many cars wait behind heads of that kind. Leaf heads are told apart by leaf kind and
// rule; CarNotWaiting heads by their arrow's kind.
struct QueueHeads {
    HeadKind head = HeadKind::Leaf;
    LeafKind leaf = LeafKind::Stop;
    RuleIndex rule = kNoRule;
    ArrowKind arrow = ArrowKind::Behind;
    uint32_t queues = 0;    // how many chains end at a head of this kind
    uint32_t cars = 0;      // how many waiting cars in all are in those chains (the heads included)
    uint32_t deepest = 0;   // the longest of those chains, in cars
};

// THE WHOLE PICTURE.
struct WaitReport {
    uint32_t driven = 0;         // driven cars looked at (filled by Driving; 0 for a hand-built list)
    uint32_t waiting = 0;        // waiting cars (each listed car once)
    uint32_t onCars = 0;         // ... of them waiting on another car (an arrow)
    uint32_t atLeaves = 0;       // ... waiting on something that is not a car
    std::vector<CarId> noReason; // ... waiting for no reason at all: ALWAYS EMPTY, or there is a bug
    std::vector<WaitLoop> loops;
    uint32_t gridlocks = 0;
    uint32_t deadlocks = 0;
    // THE LONGEST CHAIN of waiting cars not in a loop, counted in cars from its back to its head (a car alone waiting
    // at a stop is a chain of 1); its back car and its head car (on a tie, the lowest back car id).
    uint32_t longestChain = 0;
    CarId longestBack;
    CarId longestHead;
    // Per kind of head, in the order of HeadKind, then leaf kind, then rule, then arrow kind.
    std::vector<QueueHeads> heads;
};

// BUILDS THE REPORT from a list of waiting cars (any order; it is sorted by car id first). `driven` is left 0.
WaitReport analyseWaiting(std::vector<WaitingCar> waiting);

// THE REPORT IN PLAIN TEXT, one fact a line. `ruleNames`, when given, names rules by their index (the rule set's labels);
// otherwise a rule is "rule <n>".
std::string describeWaiting(const WaitReport& report, const std::vector<std::string>* ruleNames = nullptr);

}  // namespace driving

#endif
