#include "apps/driving/conflict.h"

#include "apps/driving/announce.h"
#include "apps/driving/signal_programme.h"   // currentPhase: the lights' phase, a pure function of the clock
#include "engine/bundle/bundle.h"   // Fnv: the repo's 64-bit hash folded field by field

#include <algorithm>
#include <tuple>

namespace driving {

namespace {

// One car's side on one crossing point, from its announcement there.
Side sideOf(const Announcement& a) {
    Side s;
    s.who = a.who;
    // The board holds an announcement only while the tail is not yet past the stretch: arriving now means on it.
    s.onIt = !(a.arriveIn > 0);
    s.committed = a.committed;
    s.committedTick = a.committedTick;
    s.going = a.going;
    s.arrivalFixed = a.arrivalFixed;
    s.arrivalTick = a.arrivalTick;
    return s;
}

// Folds `b` into `a` (the same car, another crossing point): any one on it, committed or going makes it so; the
// earliest commit and arrival stand.
void gather(Side& a, const Side& b) {
    a.onIt = a.onIt || b.onIt;
    if (b.committed && (!a.committed || b.committedTick < a.committedTick)) a.committedTick = b.committedTick;
    a.committed = a.committed || b.committed;
    a.going = a.going || b.going;
    if (b.arrivalFixed && (!a.arrivalFixed || b.arrivalTick < a.arrivalTick)) a.arrivalTick = b.arrivalTick;
    a.arrivalFixed = a.arrivalFixed || b.arrivalFixed;
}

Answer firstIf(bool first) { return first ? Answer::First : Answer::Second; }

}  // namespace

const char* questionName(Question q) {
    switch (q) {
    case Question::None: return "none";
    case Question::OnIt: return "on_it";
    case Question::CommittedFirst: return "committed_first";
    case Question::Table: return "table";
    case Question::Going: return "going";
    case Question::ArrivedFirst: return "arrived_first";
    case Question::Fallback: return "fallback";
    case Question::LowerId: return "lower_id";
    case Question::Ring: return "ring";
    }
    return "?";
}

const char* questionWords(Question q) {
    switch (q) {
    case Question::None: return "no conflict";
    case Question::OnIt: return "its body is already on the crossing point";
    case Question::CommittedFirst: return "it committed first";
    case Question::Table: return "it has right of way by the give-way table";
    case Question::Going: return "it can no longer stop at its line";
    case Question::ArrivedFirst: return "it reached its line first";
    case Question::Fallback: return "the table's fall-back puts it first";
    case Question::LowerId: return "nothing else told us apart: the lower car id";
    case Question::Ring: return "we stood in a ring of right of way, broken in a fixed order";
    }
    return "?";
}

bool followOnly(Question q) { return q == Question::OnIt || q == Question::CommittedFirst; }

bool operator==(const Resolution& a, const Resolution& b) { return a.firstGoes == b.firstGoes && a.by == b.by; }

// ---- The questions ----------------------------------------------------------------------------------------------------

Answer askOnIt(const Conflict& c) {
    if (c.first.onIt == c.second.onIt) return Answer::NoAnswer;
    return firstIf(c.first.onIt);
}

Answer askCommittedFirst(const Conflict& c) {
    if (c.first.committed != c.second.committed) return firstIf(c.first.committed);
    if (!c.first.committed || c.first.committedTick == c.second.committedTick) return Answer::NoAnswer;
    return firstIf(c.first.committedTick < c.second.committedTick);
}

Answer askTable(const Conflict& c) {
    switch (c.table) {
    case TableSays::FirstGivesWay: return Answer::Second;
    case TableSays::SecondGivesWay: return Answer::First;
    case TableSays::Nothing: break;
    }
    return Answer::NoAnswer;
}

Answer askGoing(const Conflict& c) {
    if (c.first.going == c.second.going) return Answer::NoAnswer;
    return firstIf(c.first.going);
}

Answer askArrivedFirst(const Conflict& c) {
    if (c.first.arrivalFixed != c.second.arrivalFixed) return firstIf(c.first.arrivalFixed);
    if (!c.first.arrivalFixed || c.first.arrivalTick == c.second.arrivalTick) return Answer::NoAnswer;
    return firstIf(c.first.arrivalTick < c.second.arrivalTick);
}

Answer askFallback(const Conflict& c) {
    switch (c.fallback) {
    case TableSays::FirstGivesWay: return Answer::Second;
    case TableSays::SecondGivesWay: return Answer::First;
    case TableSays::Nothing: break;
    }
    return Answer::NoAnswer;
}

Answer askLowerId(const Conflict& c) {
    if (c.first.who == c.second.who) return Answer::NoAnswer;
    return firstIf(c.first.who < c.second.who);
}

const std::array<QuestionSpec, kAskedQuestions> kQuestions = {{
    {Question::OnIt, askOnIt},
    {Question::CommittedFirst, askCommittedFirst},
    {Question::Table, askTable},
    {Question::Going, askGoing},
    {Question::ArrivedFirst, askArrivedFirst},
    {Question::Fallback, askFallback},
    {Question::LowerId, askLowerId},
}};

Resolution resolve(const Conflict& c) {
    for (const QuestionSpec& q : kQuestions) {
        const Answer a = q.ask(c);
        if (a == Answer::NoAnswer) continue;
        Resolution r;
        r.firstGoes = a == Answer::First;
        r.by = q.question;
        return r;
    }
    Resolution r;   // one car on both sides: not a conflict, but an answer all the same
    r.by = Question::LowerId;
    return r;
}

bool ringOrder(const Side& a, const Side& b) {
    // Smaller goes first: committed (0) before not (1), earlier commit first; going before not; a fixed arrival before
    // none, earlier first; then the lower id.
    const uint64_t never = ~uint64_t(0);
    return std::make_tuple(a.committed ? 0 : 1, a.committed ? a.committedTick : never, a.going ? 0 : 1,
                           a.arrivalFixed ? a.arrivalTick : never, a.who) <
           std::make_tuple(b.committed ? 0 : 1, b.committed ? b.committedTick : never, b.going ? 0 : 1,
                           b.arrivalFixed ? b.arrivalTick : never, b.who);
}

void addCounts(ConflictCounts& a, const ConflictCounts& b) {
    a.conflicts += b.conflicts;
    for (std::size_t q = 0; q < kQuestionCount; ++q) a.byQuestion[q] += b.byQuestion[q];
    a.rings += b.rings;
    a.ringThroughABody += b.ringThroughABody;
    a.physicalRings += b.physicalRings;
    a.queuesCut += b.queuesCut;
}

// ---- The resolver -----------------------------------------------------------------------------------------------------

uint32_t ConflictResolver::groupOf(const engine::LaneGraph& roads, engine::CrossingId crossing) {
    if (!crossing.valid() || crossing.index >= roads.crossings.size()) return static_cast<uint32_t>(roads.junctions.size());
    const engine::JunctionId j = roads.crossings[crossing.index].junction;
    return j.valid() && j.index < roads.junctions.size() ? j.index : static_cast<uint32_t>(roads.junctions.size());
}

void ConflictResolver::study(const engine::LaneGraph& roads, const RightOfWay& row, const AnnouncementBoard& board,
                             const std::vector<std::pair<uint64_t, uint64_t>>& behind, double now) {
    met_.clear();
    settled_.clear();
    counts_ = ConflictCounts();
    noJunction_ = static_cast<uint32_t>(roads.junctions.size());

    // 1. Every pair of cars met on one crossing point, from different paths, where somebody must give way.
    for (uint32_t c = 0; c < roads.crossings.size(); ++c) {
        const engine::CrossingId cid(c);
        const AnnouncementRange on = board.on(cid);
        if (on.size() < 2) continue;
        const engine::CrossingPoint& rec = roads.crossings[c];
        if (rec.otherSide != engine::OtherSide::Path) continue;
        const MeetingAnswer* answer = row.answer(cid, currentPhase(row, rec.junction, now));
        if (!answer || answer->who == GivesWay::Neither) continue;   // a split: the car behind follows
        const uint32_t group = groupOf(roads, cid);
        for (const Announcement* x = on.begin(); x != on.end(); ++x)
            for (const Announcement* y = x + 1; y != on.end(); ++y) {
                if (x->onPath == y->onPath || x->who == y->who) continue;
                const Announcement& lo = x->who < y->who ? *x : *y;
                const Announcement& hi = x->who < y->who ? *y : *x;
                Met m;
                m.group = group;
                m.crossing = c;
                m.lo = sideOf(lo);
                m.hi = sideOf(hi);
                switch (myTurn(roads, *answer, cid, lo.onPath)) {
                case MyTurn::GiveWay: m.settled = TableSays::FirstGivesWay; break;
                case MyTurn::GoFirst: m.settled = TableSays::SecondGivesWay; break;
                case MyTurn::Ask:
                    m.asked = true;
                    m.fallback = (answer->fallback == GivesWay::A) == (rec.a == lo.onPath) ? TableSays::FirstGivesWay
                                                                                          : TableSays::SecondGivesWay;
                    break;
                case MyTurn::Neither: break;
                }
                met_.push_back(m);
            }
    }
    std::sort(met_.begin(), met_.end(), [](const Met& a, const Met& b) {
        return std::make_tuple(a.group, a.lo.who, a.hi.who, a.crossing) <
               std::make_tuple(b.group, b.lo.who, b.hi.who, b.crossing);
    });

    // 2. One conflict per pair of cars per group, gathered over every crossing point they share there, and settled.
    for (std::size_t i = 0; i < met_.size();) {
        std::size_t e = i + 1;
        while (e < met_.size() && met_[e].group == met_[i].group && met_[e].lo.who == met_[i].lo.who &&
               met_[e].hi.who == met_[i].hi.who)
            ++e;
        SettledConflict s;
        s.group = met_[i].group;
        s.conflict.first = met_[i].lo;
        s.conflict.second = met_[i].hi;
        bool firstGives = false, secondGives = false;
        for (std::size_t k = i; k < e; ++k) {
            const Met& m = met_[k];
            if (k > i) {
                gather(s.conflict.first, m.lo);
                gather(s.conflict.second, m.hi);
            }
            firstGives = firstGives || m.settled == TableSays::FirstGivesWay;
            secondGives = secondGives || m.settled == TableSays::SecondGivesWay;
            // The fall-back of the lowest crossing point asked while driving (they are in crossing order).
            if (m.asked && s.conflict.fallback == TableSays::Nothing) s.conflict.fallback = m.fallback;
        }
        if (firstGives != secondGives) s.conflict.table = firstGives ? TableSays::FirstGivesWay : TableSays::SecondGivesWay;
        s.resolution = resolve(s.conflict);
        settled_.push_back(s);
        i = e;
    }

    // 3. A body lies where it lies: a car on a crossing point it shares with the other in one group is on it for their
    // conflicts in every group (two junctions whose paths overlap), so the two are never told opposite things by bodies.
    byPair_.clear();
    for (uint32_t k = 0; k < settled_.size(); ++k) byPair_.push_back(k);
    std::sort(byPair_.begin(), byPair_.end(), [this](uint32_t x, uint32_t y) {
        const SettledConflict& a = settled_[x];
        const SettledConflict& b = settled_[y];
        return std::make_tuple(a.conflict.first.who, a.conflict.second.who, a.group) <
               std::make_tuple(b.conflict.first.who, b.conflict.second.who, b.group);
    });
    for (std::size_t i = 0; i < byPair_.size();) {
        std::size_t e = i + 1;
        const SettledConflict& head = settled_[byPair_[i]];
        while (e < byPair_.size() && settled_[byPair_[e]].conflict.first.who == head.conflict.first.who &&
               settled_[byPair_[e]].conflict.second.who == head.conflict.second.who)
            ++e;
        if (e - i > 1) {
            bool firstOn = false, secondOn = false;
            for (std::size_t k = i; k < e; ++k) {
                firstOn = firstOn || settled_[byPair_[k]].conflict.first.onIt;
                secondOn = secondOn || settled_[byPair_[k]].conflict.second.onIt;
            }
            for (std::size_t k = i; k < e; ++k) {
                SettledConflict& s = settled_[byPair_[k]];
                if (s.conflict.first.onIt == firstOn && s.conflict.second.onIt == secondOn) continue;
                s.conflict.first.onIt = firstOn;
                s.conflict.second.onIt = secondOn;
                s.resolution = resolve(s.conflict);
            }
        }
        i = e;
    }

    // 4. No rings: every arrow checked for loops, with the arrows nobody can turn round, and any loop's conflicts answered
    // by one order.
    breakRings(behind);

    counts_.conflicts = settled_.size();
    for (const SettledConflict& s : settled_) ++counts_.byQuestion[static_cast<std::size_t>(s.resolution.by)];
}

uint32_t ConflictResolver::nodeOf(uint64_t who) const {
    return static_cast<uint32_t>(std::lower_bound(nodes_.begin(), nodes_.end(), who) - nodes_.begin());
}

void ConflictResolver::breakRings(const std::vector<std::pair<uint64_t, uint64_t>>& behind) {
    // Each car behind another: (the car behind, the car ahead of it), sorted to be looked up. A car may be behind one car
    // on the path its front is on and another on a path its tail is still on.
    aheadOf_.clear();
    for (const std::pair<uint64_t, uint64_t>& b : behind)
        if (b.first != b.second) aheadOf_.push_back({b.second, b.first});
    std::sort(aheadOf_.begin(), aheadOf_.end());
    aheadOf_.erase(std::unique(aheadOf_.begin(), aheadOf_.end()), aheadOf_.end());
    // Every group together: a car can be held at one junction's line by its conflict at the next (when its body would
    // not fit between the two, give_way.h), so a loop can run through two junctions.
    breakRingsIn(0, settled_.size());
}

void ConflictResolver::breakRingsIn(std::size_t first, std::size_t end) {
    // The group's cars, sorted, each with its side gathered over all its conflicts here (for the order).
    nodes_.clear();
    for (std::size_t k = first; k < end; ++k) {
        nodes_.push_back(settled_[k].conflict.first.who);
        nodes_.push_back(settled_[k].conflict.second.who);
    }
    std::sort(nodes_.begin(), nodes_.end());
    nodes_.erase(std::unique(nodes_.begin(), nodes_.end()), nodes_.end());
    const uint32_t n = static_cast<uint32_t>(nodes_.size());
    if (n < 2) return;
    nodeSides_.assign(n, Side());
    placed_.assign(n, 0);   // here: "its side has been set from a conflict"
    // The arrows: from the one that goes second to the one it goes after.
    arrows_.clear();
    for (std::size_t k = first; k < end; ++k) {
        const SettledConflict& s = settled_[k];
        const uint32_t a = nodeOf(s.conflict.first.who), b = nodeOf(s.conflict.second.who);
        for (const std::pair<uint32_t, const Side*>& node :
             {std::make_pair(a, &s.conflict.first), std::make_pair(b, &s.conflict.second)}) {
            if (!placed_[node.first]) nodeSides_[node.first] = *node.second;
            else gather(nodeSides_[node.first], *node.second);
            placed_[node.first] = 1;
        }
        Arrow arrow;
        arrow.from = s.resolution.firstGoes ? b : a;
        arrow.to = s.resolution.firstGoes ? a : b;
        arrow.conflict = static_cast<uint32_t>(k);
        arrow.fixed = s.resolution.by == Question::OnIt;
        arrows_.push_back(arrow);
    }
    // ... and from a car in a queue to the nearest cars in a conflict ahead of it in that queue (the cars between, if any,
    // are in none). A queue is looked along no further than kMaxQueueLooked cars from each: one cut short is counted.
    for (uint32_t v = 0; v < n; ++v) {
        queueWalk_.clear();
        queueWalk_.push_back(nodes_[v]);
        for (std::size_t look = 0; look < queueWalk_.size(); ++look) {
            if (look == kMaxQueueLooked) {
                ++counts_.queuesCut;
                break;
            }
            const std::vector<std::pair<uint64_t, uint64_t>>::const_iterator from =
                std::lower_bound(aheadOf_.begin(), aheadOf_.end(), std::make_pair(queueWalk_[look], uint64_t(0)));
            for (std::vector<std::pair<uint64_t, uint64_t>>::const_iterator it = from;
                 it != aheadOf_.end() && it->first == queueWalk_[look]; ++it) {
                const uint64_t ahead = it->second;
                const uint32_t w = nodeOf(ahead);
                if (w < n && nodes_[w] == ahead) {
                    if (w != v) {
                        Arrow arrow;
                        arrow.from = v;
                        arrow.to = w;
                        arrow.conflict = kFixedArrow;
                        arrow.fixed = true;
                        arrows_.push_back(arrow);
                    }
                } else if (std::find(queueWalk_.begin(), queueWalk_.end(), ahead) == queueWalk_.end()) {
                    queueWalk_.push_back(ahead);
                }
            }
        }
    }
    std::sort(arrows_.begin(), arrows_.end(), [](const Arrow& x, const Arrow& y) {
        return std::make_tuple(x.from, x.to, x.conflict) < std::make_tuple(y.from, y.to, y.conflict);
    });
    firstArrow_.assign(n + 1, 0);
    for (const Arrow& arrow : arrows_) ++firstArrow_[arrow.from + 1];
    for (uint32_t i = 0; i < n; ++i) firstArrow_[i + 1] += firstArrow_[i];

    strongParts();
    for (uint32_t p = 0; p < partSize_.size(); ++p)
        if (partSize_[p] >= 2) orderPart(p);
}

void ConflictResolver::strongParts() {
    // Tarjan's method, walked without recursion: each car's visit number and the lowest visit number it reaches; a car
    // whose two are equal heads a strongly connected part, made of the cars above it on the stack.
    const uint32_t n = static_cast<uint32_t>(nodes_.size());
    constexpr uint32_t kUnvisited = 0xFFFFFFFFu;
    visitOrder_.assign(n, kUnvisited);
    lowLink_.assign(n, 0);
    part_.assign(n, 0);
    onStack_.assign(n, 0);
    partSize_.clear();
    stack_.clear();
    uint32_t visits = 0;
    for (uint32_t root = 0; root < n; ++root) {
        if (visitOrder_[root] != kUnvisited) continue;
        walk_.clear();
        walkArrow_.clear();
        const auto enter = [&](uint32_t v) {
            visitOrder_[v] = lowLink_[v] = visits++;
            stack_.push_back(v);
            onStack_[v] = 1;
            walk_.push_back(v);
            walkArrow_.push_back(firstArrow_[v]);
        };
        enter(root);
        while (!walk_.empty()) {
            const uint32_t v = walk_.back();
            if (walkArrow_.back() < firstArrow_[v + 1]) {
                const uint32_t w = arrows_[walkArrow_.back()++].to;
                if (visitOrder_[w] == kUnvisited) enter(w);
                else if (onStack_[w]) lowLink_[v] = std::min(lowLink_[v], visitOrder_[w]);
                continue;
            }
            if (lowLink_[v] == visitOrder_[v]) {
                const uint32_t p = static_cast<uint32_t>(partSize_.size());
                partSize_.push_back(0);
                uint32_t w = kUnvisited;
                do {
                    w = stack_.back();
                    stack_.pop_back();
                    onStack_[w] = 0;
                    part_[w] = p;
                    ++partSize_[p];
                } while (w != v);
            }
            walk_.pop_back();
            walkArrow_.pop_back();
            if (!walk_.empty()) lowLink_[walk_.back()] = std::min(lowLink_[walk_.back()], lowLink_[v]);
        }
    }
}

void ConflictResolver::orderPart(uint32_t part) {
    ++counts_.rings;
    const uint32_t n = static_cast<uint32_t>(nodes_.size());
    partNodes_.clear();
    for (uint32_t v = 0; v < n; ++v)
        if (part_[v] == part) partNodes_.push_back(v);
    // How many fixed arrows each car of the part has to cars of the part not yet placed (it comes after all of them), and
    // how many of those are a body's (on_it).
    waitingOn_.assign(n, 0);
    waitingOnBody_.assign(n, 0);
    rank_.assign(n, 0);
    placed_.assign(n, 0);
    for (const uint32_t v : partNodes_)
        for (uint32_t a = firstArrow_[v]; a < firstArrow_[v + 1]; ++a)
            if (arrows_[a].fixed && part_[arrows_[a].to] == part) {
                ++waitingOn_[v];
                if (arrows_[a].conflict != kFixedArrow) ++waitingOnBody_[v];
            }
    bool brokeFixed = false;
    for (uint32_t pos = 0; pos < partNodes_.size(); ++pos) {
        // Next: of the cars free to come next (nothing fixed still ahead of them), the first by ringOrder. When none is
        // free (a loop of fixed arrows), the first of those waiting on no body (a queue arrow gives way first: it is no
        // answer, so putting a car before the car ahead of it in its queue changes nothing it does); then any.
        uint32_t best = n;
        for (int pass = 0; pass < 3 && best == n; ++pass)
            for (const uint32_t v : partNodes_)
                if (!placed_[v] && (pass == 2 || (pass == 1 ? waitingOnBody_[v] == 0 : waitingOn_[v] == 0)) &&
                    (best == n || ringOrder(nodeSides_[v], nodeSides_[best])))
                    best = v;
        if (waitingOn_[best] != 0) brokeFixed = true;
        placed_[best] = 1;
        rank_[best] = pos;
        // The cars with a fixed arrow to it wait on one fewer.
        for (const uint32_t v : partNodes_)
            for (uint32_t a = firstArrow_[v]; a < firstArrow_[v + 1]; ++a)
                if (arrows_[a].fixed && arrows_[a].to == best && waitingOn_[v] > 0) {
                    --waitingOn_[v];
                    if (arrows_[a].conflict != kFixedArrow && waitingOnBody_[v] > 0) --waitingOnBody_[v];
                }
    }
    if (brokeFixed) ++counts_.physicalRings;
    // Every conflict inside the part answered by the order (its question kept where the order agrees with it).
    for (const uint32_t v : partNodes_)
        for (uint32_t a = firstArrow_[v]; a < firstArrow_[v + 1]; ++a) {
            const Arrow& arrow = arrows_[a];
            if (arrow.conflict == kFixedArrow || part_[arrow.to] != part) continue;
            SettledConflict& s = settled_[arrow.conflict];
            const bool firstGoes = rank_[nodeOf(s.conflict.first.who)] < rank_[nodeOf(s.conflict.second.who)];
            if (firstGoes == s.resolution.firstGoes) continue;
            if (s.resolution.by == Question::OnIt) ++counts_.ringThroughABody;
            s.beforeRing = s.resolution;
            s.resolution.firstGoes = firstGoes;
            s.resolution.by = Question::Ring;
        }
}

const SettledConflict* ConflictResolver::find(uint32_t group, uint64_t a, uint64_t b) const {
    const uint64_t lo = std::min(a, b), hi = std::max(a, b);
    const std::vector<SettledConflict>::const_iterator it =
        std::lower_bound(settled_.begin(), settled_.end(), std::make_tuple(group, lo, hi),
                         [](const SettledConflict& s, const std::tuple<uint32_t, uint64_t, uint64_t>& key) {
                             return std::make_tuple(s.group, s.conflict.first.who, s.conflict.second.who) < key;
                         });
    if (it == settled_.end() || it->group != group || it->conflict.first.who != lo || it->conflict.second.who != hi)
        return nullptr;
    return &*it;
}

bool ConflictResolver::goesFirst(const engine::LaneGraph& roads, engine::CrossingId crossing, uint64_t me,
                                 uint64_t them, Question* by) const {
    const SettledConflict* s = find(groupOf(roads, crossing), me, them);
    if (by) *by = s ? s->resolution.by : Question::None;
    if (!s) return true;
    return (s->conflict.first.who == me) == s->resolution.firstGoes;
}

uint64_t ConflictResolver::hash() const {
    engine::bundle::Fnv h;
    h.pod(static_cast<uint64_t>(settled_.size()));
    for (const SettledConflict& s : settled_) {
        h.pod(s.group);
        h.pod(s.conflict.first.who);
        h.pod(s.conflict.second.who);
        h.pod(static_cast<uint8_t>(s.resolution.firstGoes));
        h.pod(static_cast<uint8_t>(s.resolution.by));
    }
    return h.h;
}

}  // namespace driving
