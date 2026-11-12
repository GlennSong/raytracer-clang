// Seeing why (#256), the parts that need no road: the wait-for graph on hand-built lists of waiting cars (any arrow
// kinds, so the give-way and no-room kinds later tasks bring can be tested now), and the trace's ring on its own.
//   1. a chain of five behind a hold: no loop, a chain five deep, the hold's rule counted as its head;
//   2. a loop of four "behind" arrows: one loop, gridlock;
//   3. a loop of three with one "giving way" arrow: one loop, deadlock;
//   4. two separate loops and a chain feeding into one: two loops, each once, each from its lowest car; the chain is
//      no loop;
//   5. a waiting car with no reason is on the no-reason list;
//   6. 100,000 cars in one chain, and in one loop: quick (the time is printed);
//   7. the ring keeps the last eight entries, oldest first.
// The live cases (a real tick on test roads) are in test_driving_tick.cpp beside the tick's own.

#include "test_framework.h"

#include "apps/driving/trace.h"
#include "apps/driving/wait_graph.h"

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

using namespace driving;

namespace {

constexpr RuleIndex kHoldRule = 4;

WaitingCar arrow(uint32_t car, uint32_t to, ArrowKind kind = ArrowKind::Behind) {
    WaitingCar w;
    w.car = CarId(car);
    w.why.sort = ReasonSort::Arrow;
    w.why.arrow = kind;
    w.why.toCar = CarId(to);
    return w;
}

WaitingCar leaf(uint32_t car, LeafKind kind, RuleIndex rule = kNoRule) {
    WaitingCar w;
    w.car = CarId(car);
    w.why.sort = ReasonSort::Leaf;
    w.why.leaf = kind;
    w.why.rule = rule;
    return w;
}

WaitingCar noReason(uint32_t car) {
    WaitingCar w;
    w.car = CarId(car);
    return w;
}

std::vector<uint32_t> ids(const std::vector<CarId>& cars) {
    std::vector<uint32_t> out;
    for (const CarId c : cars) out.push_back(c.index);
    return out;
}

}  // namespace

TEST_CASE(wait_graph_chain_of_five_behind_a_hold) {
    // Car 10 is held at a stop by the hold rule; 11 is behind 10, 12 behind 11, and so on. Listed out of order.
    const std::vector<WaitingCar> waiting = {arrow(13, 12), arrow(11, 10), leaf(10, LeafKind::Stop, kHoldRule),
                                             arrow(14, 13), arrow(12, 11)};
    const WaitReport r = analyseWaiting(waiting);
    CHECK(r.waiting == 5u && r.onCars == 4u && r.atLeaves == 1u);
    CHECK(r.noReason.empty());
    CHECK(r.loops.empty() && findLoops(waiting).empty());
    CHECK(r.longestChain == 5u && r.longestBack == CarId(14) && r.longestHead == CarId(10));
    CHECK(r.heads.size() == 1u);
    CHECK(!r.heads.empty() && r.heads[0].head == HeadKind::Leaf && r.heads[0].leaf == LeafKind::Stop &&
          r.heads[0].rule == kHoldRule && r.heads[0].queues == 1u && r.heads[0].cars == 5u &&
          r.heads[0].deepest == 5u);
    const std::vector<std::string> names = {"speed limit", "bend ahead", "lane ends", "arrive", "held"};
    const std::string text = describeWaiting(r, &names);
    std::printf("%s", text.c_str());
    CHECK(text.find("a stop by rule \"held\": 1 queue, 5 cars in all, the longest 5") != std::string::npos);
    CHECK(text.find("no loops") != std::string::npos);
}

TEST_CASE(wait_graph_loop_of_four_behind_is_gridlock) {
    const std::vector<WaitingCar> waiting = {arrow(7, 3), arrow(3, 5), arrow(5, 9), arrow(9, 7)};
    const WaitReport r = analyseWaiting(waiting);
    CHECK(r.loops.size() == 1u);
    CHECK(r.gridlocks == 1u && r.deadlocks == 0u);
    if (!r.loops.empty()) {
        CHECK(r.loops[0].kind == LoopClass::Gridlock);
        CHECK((ids(r.loops[0].cars) == std::vector<uint32_t>{3, 5, 9, 7}));
        CHECK(r.loops[0].arrows.size() == 4u);
    }
    CHECK(r.longestChain == 0u && r.heads.empty() && r.noReason.empty());
    std::printf("%s", describeWaiting(r).c_str());
}

TEST_CASE(wait_graph_loop_with_a_give_way_arrow_is_deadlock) {
    const std::vector<WaitingCar> waiting = {arrow(1, 2), arrow(2, 3, ArrowKind::GivingWay), arrow(3, 1)};
    const WaitReport r = analyseWaiting(waiting);
    CHECK(r.loops.size() == 1u);
    CHECK(r.gridlocks == 0u && r.deadlocks == 1u);
    if (!r.loops.empty()) {
        CHECK(r.loops[0].kind == LoopClass::Deadlock);
        CHECK((ids(r.loops[0].cars) == std::vector<uint32_t>{1, 2, 3}));
        CHECK(r.loops[0].arrows[1] == ArrowKind::GivingWay);
    }
    // The classifier alone: space kinds only is gridlock; any right-of-way kind is deadlock.
    CHECK(classifyLoop({ArrowKind::Behind, ArrowKind::NoRoomBeyond}) == LoopClass::Gridlock);
    CHECK(classifyLoop({ArrowKind::NoRoomBeyond, ArrowKind::GivingWay}) == LoopClass::Deadlock);
    // Every arrow kind has a row in the table, in enum order.
    for (uint8_t k = 0; k < kArrowKindCount; ++k) CHECK(static_cast<uint8_t>(kArrowKinds[k].kind) == k);
    std::printf("%s", describeWaiting(r).c_str());
}

TEST_CASE(wait_graph_two_loops_and_a_feeding_chain) {
    // Loop A: 20 -> 21 -> 22 -> 20. Loop B: 8 -> 30 -> 8 (no room). A chain 40 -> 41 -> 21 feeds loop A. And a car
    // behind a car that is not waiting (99 is moving).
    const std::vector<WaitingCar> waiting = {arrow(41, 21),
                                             arrow(22, 20),
                                             arrow(30, 8, ArrowKind::NoRoomBeyond),
                                             arrow(40, 41),
                                             arrow(21, 22),
                                             arrow(8, 30),
                                             arrow(20, 21),
                                             arrow(50, 99)};
    const WaitReport r = analyseWaiting(waiting);
    CHECK(r.loops.size() == 2u);
    if (r.loops.size() == 2u) {
        CHECK((ids(r.loops[0].cars) == std::vector<uint32_t>{8, 30}));
        CHECK((ids(r.loops[1].cars) == std::vector<uint32_t>{20, 21, 22}));
        CHECK(r.loops[0].kind == LoopClass::Gridlock && r.loops[1].kind == LoopClass::Gridlock);
    }
    CHECK(findLoops(waiting).size() == 2u);
    // The feeding chain: two cars deep, its head (41) runs into a loop.
    CHECK(r.longestChain == 2u && r.longestBack == CarId(40) && r.longestHead == CarId(41));
    uint32_t intoLoops = 0, behindMoving = 0;
    for (const QueueHeads& h : r.heads) {
        if (h.head == HeadKind::Loop) intoLoops += h.cars;
        if (h.head == HeadKind::CarNotWaiting) behindMoving += h.cars;
    }
    CHECK(intoLoops == 2u && behindMoving == 1u);
    std::printf("%s", describeWaiting(r).c_str());
}

TEST_CASE(wait_graph_no_reason_is_listed) {
    const std::vector<WaitingCar> waiting = {leaf(2, LeafKind::WayEnd), noReason(6), arrow(7, 6), noReason(3)};
    const WaitReport r = analyseWaiting(waiting);
    CHECK((ids(r.noReason) == std::vector<uint32_t>{3, 6}));
    CHECK(r.loops.empty());
    bool headedByNoReason = false;
    for (const QueueHeads& h : r.heads)
        if (h.head == HeadKind::NoReason && h.queues == 2u && h.cars == 3u) headedByNoReason = true;
    CHECK(headedByNoReason);
    const std::string text = describeWaiting(r);
    std::printf("%s", text.c_str());
    CHECK(text.find("WAITING WITH NO REASON (a bug): car 3, car 6") != std::string::npos);
}

TEST_CASE(wait_graph_a_hundred_thousand_in_one_chain_is_quick) {
    constexpr uint32_t kCars = 100000;
    // Car k waits behind car k + 1 (each higher id is further ahead); the last is held at a stop. Listed back to front.
    std::vector<WaitingCar> chain;
    for (uint32_t k = kCars; k-- > 0;)
        chain.push_back(k + 1 == kCars ? leaf(k, LeafKind::Stop, kHoldRule) : arrow(k, k + 1));
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    const WaitReport r = analyseWaiting(chain);
    const double chainMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    CHECK(r.loops.empty() && r.longestChain == kCars && r.longestBack == CarId(0) && r.longestHead == CarId(kCars - 1));
    // And the same cars in one loop.
    chain.front() = arrow(kCars - 1, 0);
    start = std::chrono::steady_clock::now();
    const WaitReport loop = analyseWaiting(chain);
    const double loopMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    CHECK(loop.loops.size() == 1u && !loop.loops.empty() && loop.loops[0].cars.size() == kCars &&
          loop.loops[0].cars[0] == CarId(0));
    constexpr double kQuickMs = 500;
    CHECK(chainMs < kQuickMs && loopMs < kQuickMs);
    std::printf("    100,000 cars: one chain analysed in %.1f ms, one loop in %.1f ms\n", chainMs, loopMs);
}

TEST_CASE(trace_ring_keeps_the_last_eight_oldest_first) {
    CarTrace t;
    CHECK(t.size() == 0u && t.latest() == nullptr);
    constexpr uint32_t kWritten = 12;
    for (uint32_t k = 0; k < kWritten; ++k) {
        TraceEntry e;
        e.tick = k;
        e.binding.by = Limit::CarAhead;
        e.binding.car = CarId(k);
        t.add(e);
    }
    CHECK(t.size() == kTraceLength && t.written == kWritten);
    for (std::size_t i = 0; i < t.size(); ++i) CHECK(t.oldestFirst(i).tick == kWritten - kTraceLength + i);
    CHECK(t.latest() != nullptr && t.latest()->tick == kWritten - 1);
    std::printf("    a driven car's trace: %zu bytes (%zu entries of %zu bytes, and the two held reasons)\n",
                sizeof(CarTrace), kTraceLength, sizeof(TraceEntry));
}
