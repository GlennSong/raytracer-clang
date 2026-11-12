#include "apps/driving/wait_graph.h"

#include "apps/driving/notice.h"   // kStandingBelow

#include <algorithm>
#include <map>
#include <tuple>

namespace driving {

namespace {

// "No car here": an index into the sorted list that points nowhere.
constexpr uint32_t kNoIndex = 0xFFFFFFFFu;

std::string carWords(CarId c) { return "car " + std::to_string(c.index); }

std::string ruleWords(RuleIndex rule, const std::vector<std::string>* names) {
    if (names && rule < names->size()) return "\"" + (*names)[rule] + "\"";
    return "rule " + std::to_string(rule);
}

std::string plural(uint32_t n, const char* one, const char* many) {
    return std::to_string(n) + " " + (n == 1 ? one : many);
}

// What a head of this kind is, in words.
std::string headWords(const QueueHeads& h, const std::vector<std::string>* names) {
    switch (h.head) {
    case HeadKind::Leaf:
        switch (h.leaf) {
        case LeafKind::Stop: return "a stop by rule " + ruleWords(h.rule, names);
        case LeafKind::TopSpeed:
            return h.rule == kNoRule ? std::string("its vehicle class's top speed")
                                     : "a top speed by rule " + ruleWords(h.rule, names);
        case LeafKind::WayEnd: return "the end of its way";
        }
        break;
    case HeadKind::CarNotWaiting:
        return std::string(arrowKindInfo(h.arrow).words) + " a car that is not waiting (moving, or parked)";
    case HeadKind::Loop: return "a loop";
    case HeadKind::NoReason: return "a car waiting for no reason";
    }
    return "?";
}

}  // namespace

const ArrowKindInfo& arrowKindInfo(ArrowKind kind) {
    const std::size_t i = static_cast<std::size_t>(kind);
    return kArrowKinds[i < kArrowKindCount ? i : 0];
}

bool isWaiting(const Car& car) {
    return car.driven && car.held.decided && car.speed < kStandingBelow && !(car.acceleration > 0);
}

WaitReason waitReasonOf(const Car& car) {
    WaitReason r;
    switch (car.binding.by) {
    case Limit::Nothing: r.sort = ReasonSort::None; break;
    case Limit::CarAhead:
        r.sort = ReasonSort::Arrow;
        r.arrow = ArrowKind::Behind;
        r.toCar = car.binding.car;
        break;
    case Limit::Stop:
        // Giving way to a car: an arrow to it. A full stop first (no car named) or any other place: a leaf.
        if (car.binding.stopFor == StopFor::GivingWay && car.binding.car.valid()) {
            r.sort = ReasonSort::Arrow;
            r.arrow = ArrowKind::GivingWay;
            r.toCar = car.binding.car;
            r.rule = car.binding.rule;
            break;
        }
        // Waiting for a body already on the crossing point (issue #284): an arrow about space, as behind a car.
        if (car.binding.stopFor == StopFor::ForABody && car.binding.car.valid()) {
            r.sort = ReasonSort::Arrow;
            r.arrow = ArrowKind::ForABody;
            r.toCar = car.binding.car;
            r.rule = car.binding.rule;
            break;
        }
        // Waiting at the line for room beyond, behind a car's tail (issue #264): an arrow about space. With no car named
        // (the lights or a stop at the next junction will not let it through) it is a leaf.
        if (car.binding.stopFor == StopFor::NoRoom && car.binding.car.valid()) {
            r.sort = ReasonSort::Arrow;
            r.arrow = ArrowKind::NoRoomBeyond;
            r.toCar = car.binding.car;
            r.rule = car.binding.rule;
            break;
        }
        r.sort = ReasonSort::Leaf;
        r.leaf = LeafKind::Stop;
        r.rule = car.binding.rule;
        break;
    case Limit::TopSpeed:
        r.sort = ReasonSort::Leaf;
        r.leaf = LeafKind::TopSpeed;
        r.rule = car.binding.rule;
        break;
    case Limit::WayEnd:
        r.sort = ReasonSort::Leaf;
        r.leaf = LeafKind::WayEnd;
        break;
    }
    return r;
}

std::vector<WaitingCar> waitingCars(const std::vector<Car>& cars) {
    std::vector<WaitingCar> out;
    for (const Car& c : cars) {
        if (!isWaiting(c)) continue;
        WaitingCar w;
        w.car = c.id;
        w.why = waitReasonOf(c);
        out.push_back(w);
    }
    return out;
}

LoopClass classifyLoop(const std::vector<ArrowKind>& arrows) {
    for (const ArrowKind a : arrows)
        if (arrowKindInfo(a).sort == WaitSort::RightOfWay) return LoopClass::Deadlock;
    return LoopClass::Gridlock;
}

namespace {

// The waiting cars sorted by id, each once, and each one's arrow as an index into that list.
struct Graph {
    std::vector<WaitingCar> cars;
    std::vector<uint32_t> next;   // the car it waits on, when that car is waiting too; kNoIndex otherwise
};

Graph buildGraph(std::vector<WaitingCar> waiting) {
    Graph g;
    std::stable_sort(waiting.begin(), waiting.end(),
                     [](const WaitingCar& a, const WaitingCar& b) { return a.car < b.car; });
    // A car listed twice keeps its first listing.
    for (const WaitingCar& w : waiting)
        if (g.cars.empty() || g.cars.back().car != w.car) g.cars.push_back(w);
    g.next.assign(g.cars.size(), kNoIndex);
    for (std::size_t i = 0; i < g.cars.size(); ++i) {
        const WaitReason& why = g.cars[i].why;
        if (why.sort != ReasonSort::Arrow) continue;
        const std::vector<WaitingCar>::const_iterator it =
            std::lower_bound(g.cars.begin(), g.cars.end(), why.toCar,
                             [](const WaitingCar& w, CarId want) { return w.car < want; });
        if (it != g.cars.end() && it->car == why.toCar) g.next[i] = static_cast<uint32_t>(it - g.cars.begin());
    }
    return g;
}

// Every loop of the graph, each once, from its lowest car, in order of it; `inLoop` marks their cars.
std::vector<WaitLoop> loopsOf(const Graph& g, std::vector<uint8_t>& inLoop) {
    std::vector<WaitLoop> loops;
    const std::size_t n = g.cars.size();
    inLoop.assign(n, 0);
    // Which walk first reached each car (the walk's starting index + 1); 0 for none yet.
    std::vector<uint32_t> walk(n, 0);
    for (std::size_t s = 0; s < n; ++s) {
        if (walk[s] != 0) continue;
        const uint32_t mark = static_cast<uint32_t>(s) + 1;
        uint32_t j = static_cast<uint32_t>(s);
        while (j != kNoIndex && walk[j] == 0) {
            walk[j] = mark;
            j = g.next[j];
        }
        // Back on a car of this same walk: the walk closed a loop through it.
        if (j == kNoIndex || walk[j] != mark) continue;
        WaitLoop loop;
        uint32_t lowest = j;
        uint32_t k = j;
        do {
            lowest = std::min(lowest, k);   // the list is in car-id order, so the lowest index is the lowest car
            k = g.next[k];
        } while (k != j);
        k = lowest;
        do {
            inLoop[k] = 1;
            loop.cars.push_back(g.cars[k].car);
            loop.arrows.push_back(g.cars[k].why.arrow);
            k = g.next[k];
        } while (k != lowest);
        loop.kind = classifyLoop(loop.arrows);
        loops.push_back(std::move(loop));
    }
    std::sort(loops.begin(), loops.end(), [](const WaitLoop& a, const WaitLoop& b) { return a.cars[0] < b.cars[0]; });
    return loops;
}

}  // namespace

std::vector<WaitLoop> findLoops(const std::vector<WaitingCar>& waiting) {
    const Graph g = buildGraph(waiting);
    std::vector<uint8_t> inLoop;
    return loopsOf(g, inLoop);
}

WaitReport analyseWaiting(std::vector<WaitingCar> waiting) {
    WaitReport r;
    const Graph g = buildGraph(std::move(waiting));
    const std::size_t n = g.cars.size();
    r.waiting = static_cast<uint32_t>(n);
    for (const WaitingCar& w : g.cars) {
        if (w.why.sort == ReasonSort::Arrow) ++r.onCars;
        else if (w.why.sort == ReasonSort::Leaf) ++r.atLeaves;
        else r.noReason.push_back(w.car);
    }

    std::vector<uint8_t> inLoop;
    r.loops = loopsOf(g, inLoop);
    for (const WaitLoop& l : r.loops) {
        if (l.kind == LoopClass::Gridlock) ++r.gridlocks;
        else ++r.deadlocks;
    }

    // THE CHAINS: each car not in a loop gets its depth (cars from it to its head, itself included) and its head.
    // Each car is pushed once and popped once.
    std::vector<uint32_t> depth(n, 0);
    std::vector<uint32_t> headOf(n, kNoIndex);
    std::vector<uint32_t> stack;
    for (std::size_t i = 0; i < n; ++i) {
        if (inLoop[i] || depth[i] != 0) continue;
        stack.clear();
        uint32_t j = static_cast<uint32_t>(i);
        while (j != kNoIndex && !inLoop[j] && depth[j] == 0) {
            stack.push_back(j);
            j = g.next[j];
        }
        uint32_t base = 0;
        uint32_t head = stack.back();   // ends nowhere, or runs into a loop: the last car walked is the head
        if (j != kNoIndex && !inLoop[j]) {
            base = depth[j];            // runs into a chain already measured
            head = headOf[j];
        }
        while (!stack.empty()) {
            const uint32_t k = stack.back();
            stack.pop_back();
            depth[k] = ++base;
            headOf[k] = head;
        }
    }

    // THE HEADS, tallied by kind.
    using HeadKey = std::tuple<uint8_t, uint8_t, RuleIndex, uint8_t>;
    std::map<HeadKey, QueueHeads> tally;
    for (std::size_t i = 0; i < n; ++i) {
        if (inLoop[i]) continue;
        const uint32_t h = headOf[i];
        const WaitReason& why = g.cars[h].why;
        QueueHeads key;
        if (why.sort == ReasonSort::None) {
            key.head = HeadKind::NoReason;
        } else if (why.sort == ReasonSort::Leaf) {
            key.head = HeadKind::Leaf;
            key.leaf = why.leaf;
            key.rule = why.leaf == LeafKind::WayEnd ? kNoRule : why.rule;
        } else if (g.next[h] != kNoIndex && inLoop[g.next[h]]) {
            key.head = HeadKind::Loop;
        } else {
            key.head = HeadKind::CarNotWaiting;
            key.arrow = why.arrow;
        }
        const HeadKey k(static_cast<uint8_t>(key.head), static_cast<uint8_t>(key.leaf), key.rule,
                        static_cast<uint8_t>(key.arrow));
        std::map<HeadKey, QueueHeads>::iterator it = tally.find(k);
        if (it == tally.end()) it = tally.emplace(k, key).first;
        QueueHeads& t = it->second;
        ++t.cars;
        if (h == i) ++t.queues;
        t.deepest = std::max(t.deepest, depth[i]);
        if (depth[i] > r.longestChain) {
            r.longestChain = depth[i];
            r.longestBack = g.cars[i].car;
            r.longestHead = g.cars[h].car;
        }
    }
    for (const std::pair<const HeadKey, QueueHeads>& t : tally) r.heads.push_back(t.second);
    return r;
}

std::string describeWaiting(const WaitReport& r, const std::vector<std::string>* ruleNames) {
    std::string out;
    if (r.waiting == 0) return "nobody waiting (" + plural(r.driven, "driven car", "driven cars") + ")\n";
    out += plural(r.waiting, "car", "cars") + " waiting";
    if (r.driven > 0) out += " of " + plural(r.driven, "driven car", "driven cars");
    out += ": " + std::to_string(r.onCars) + " on another car, " + std::to_string(r.atLeaves) +
           " on something else, " + std::to_string(r.noReason.size()) + " with no reason\n";
    if (!r.noReason.empty()) {
        out += "WAITING WITH NO REASON (a bug):";
        for (std::size_t i = 0; i < r.noReason.size(); ++i) out += (i ? ", " : " ") + carWords(r.noReason[i]);
        out += "\n";
    }
    if (r.loops.empty()) {
        out += "no loops\n";
    } else {
        out += plural(static_cast<uint32_t>(r.loops.size()), "loop", "loops") + ": " + std::to_string(r.gridlocks) +
               " gridlock, " + std::to_string(r.deadlocks) + " deadlock\n";
        for (std::size_t l = 0; l < r.loops.size(); ++l) {
            const WaitLoop& loop = r.loops[l];
            out += "loop " + std::to_string(l + 1) + ", " +
                   (loop.kind == LoopClass::Gridlock
                        ? "GRIDLOCK (every arrow is about room: the road is full, no rule is broken)"
                        : "DEADLOCK (an arrow is about right of way: a bug)") +
                   ", " + plural(static_cast<uint32_t>(loop.cars.size()), "car", "cars") + ":";
            for (std::size_t i = 0; i < loop.cars.size(); ++i)
                out += (i ? ", " : " ") + carWords(loop.cars[i]) + " " + arrowKindInfo(loop.arrows[i]).words + " " +
                       carWords(loop.cars[(i + 1) % loop.cars.size()]);
            out += "\n";
        }
    }
    if (r.longestChain > 0)
        out += "longest queue: " + plural(r.longestChain, "car", "cars") + ", " + carWords(r.longestBack) +
               " at the back, " + carWords(r.longestHead) + " at the head\n";
    if (!r.heads.empty()) {
        out += "queue heads:\n";
        for (const QueueHeads& h : r.heads)
            out += "  " + headWords(h, ruleNames) + ": " + plural(h.queues, "queue", "queues") + ", " +
                   plural(h.cars, "car", "cars") + " in all, the longest " + std::to_string(h.deepest) + "\n";
    }
    return out;
}

}  // namespace driving
