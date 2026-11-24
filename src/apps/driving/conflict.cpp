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
    s.cannotStop = a.cannotStop || s.onIt;
    s.committed = a.committed;
    s.committedTick = a.committedTick;
    s.going = a.going;
    s.arrivalFixed = a.arrivalFixed;
    s.arrivalTick = a.arrivalTick;
    return s;
}

// Folds `b` into `a` (the same car, another crossing point): any one on it, unable to stop short of it, committed or
// going makes it so; the earliest commit and arrival stand.
void gather(Side& a, const Side& b) {
    a.onIt = a.onIt || b.onIt;
    a.cannotStop = a.cannotStop || b.cannotStop;
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
    case Question::CannotStop: return "cannot_stop";
    case Question::CommittedFirst: return "committed_first";
    case Question::Table: return "table";
    case Question::Going: return "going";
    case Question::ArrivedFirst: return "arrived_first";
    case Question::Fallback: return "fallback";
    case Question::Hesitation: return "hesitation";
    }
    return "?";
}

const char* questionWords(Question q) {
    switch (q) {
    case Question::None: return "no conflict";
    case Question::OnIt: return "its body is already on the crossing point";
    case Question::CannotStop: return "it cannot stop short of the crossing point";
    case Question::CommittedFirst: return "it committed first";
    case Question::Table: return "it has right of way by the give-way table";
    case Question::Going: return "it can no longer stop at its line";
    case Question::ArrivedFirst: return "it reached its line first";
    case Question::Fallback: return "the table's fall-back puts it first";
    case Question::Hesitation: return "nothing else told us apart: one of us hesitated";
    }
    return "?";
}

bool followOnly(Question q) {
    return q == Question::OnIt || q == Question::CannotStop || q == Question::CommittedFirst;
}

bool fixedAnswer(Question q) { return q == Question::OnIt || q == Question::CannotStop; }

bool operator==(const Resolution& a, const Resolution& b) { return a.firstGoes == b.firstGoes && a.by == b.by; }

// ---- The questions ----------------------------------------------------------------------------------------------------

Answer askOnIt(const Conflict& c) {
    if (c.first.onIt == c.second.onIt) return Answer::NoAnswer;
    return firstIf(c.first.onIt);
}

Answer askCannotStop(const Conflict& c) {
    if (c.first.cannotStop == c.second.cannotStop) return Answer::NoAnswer;
    return firstIf(c.first.cannotStop);
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

// THE TIE: neither has right of way and neither came first. People hesitate, and the bolder goes: each driver's own
// seeded boldness (a hash of the car and when it reached its line -- not tossed afresh each tick, which would have them
// swap places), compared. One number per driver, so the answers between any number of cars hold no loop: four cars at
// an all-way stop in the same tick go one by one, where the kerb-side fall-back alone runs in a ring (each has another
// on its kerb side). No central order is needed: each driver reads the other's boldness off how it moves. Not the lower
// id: that put the same car first in every tie it met.
uint64_t boldness(const Side& s) {
    engine::bundle::Fnv h;
    h.pod(s.who);
    h.pod(s.arrivalFixed ? s.arrivalTick : uint64_t(0));
    return h.h;
}
Answer askHesitation(const Conflict& c) {
    if (c.first.who == c.second.who) return Answer::NoAnswer;
    const uint64_t a = boldness(c.first), b = boldness(c.second);
    return firstIf(a != b ? a > b : c.first.who < c.second.who);
}
// ...asked BEFORE the fall-back when both already stand at their lines, having reached them in the same tick: the
// fall-back is the law for cars still driving up, and among cars at their lines it can run in a ring.
Answer askTieAtLines(const Conflict& c) {
    if (!c.first.arrivalFixed || !c.second.arrivalFixed) return Answer::NoAnswer;
    return askHesitation(c);
}

const std::array<QuestionSpec, kAskedQuestions> kQuestions = {{
    {Question::OnIt, askOnIt},
    {Question::CannotStop, askCannotStop},
    {Question::CommittedFirst, askCommittedFirst},
    {Question::Table, askTable},
    {Question::Going, askGoing},
    {Question::ArrivedFirst, askArrivedFirst},
    {Question::Hesitation, askTieAtLines},
    {Question::Fallback, askFallback},
    {Question::Hesitation, askHesitation},
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
    r.by = Question::Hesitation;
    return r;
}


void addCounts(ConflictCounts& a, const ConflictCounts& b) {
    a.conflicts += b.conflicts;
    for (std::size_t q = 0; q < kQuestionCount; ++q) a.byQuestion[q] += b.byQuestion[q];
    a.secondCannotStop += b.secondCannotStop;
}

// ---- The resolver -----------------------------------------------------------------------------------------------------

uint32_t ConflictResolver::groupOf(const engine::LaneGraph& roads, engine::CrossingId crossing) {
    if (!crossing.valid() || crossing.index >= roads.crossings.size()) return static_cast<uint32_t>(roads.junctions.size());
    const engine::JunctionId j = roads.crossings[crossing.index].junction;
    return j.valid() && j.index < roads.junctions.size() ? j.index : static_cast<uint32_t>(roads.junctions.size());
}

void ConflictResolver::study(const engine::LaneGraph& roads, const RightOfWay& row, const AnnouncementBoard& board,
                             double now) {
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
    // conflicts in every group (two junctions whose paths overlap), so the two are never told opposite things by bodies;
    // and the same for a car that cannot stop short of one.
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
            bool firstOn = false, secondOn = false, firstCannot = false, secondCannot = false;
            for (std::size_t k = i; k < e; ++k) {
                const Conflict& c = settled_[byPair_[k]].conflict;
                firstOn = firstOn || c.first.onIt;
                secondOn = secondOn || c.second.onIt;
                firstCannot = firstCannot || c.first.cannotStop;
                secondCannot = secondCannot || c.second.cannotStop;
            }
            for (std::size_t k = i; k < e; ++k) {
                SettledConflict& s = settled_[byPair_[k]];
                if (s.conflict.first.onIt == firstOn && s.conflict.second.onIt == secondOn &&
                    s.conflict.first.cannotStop == firstCannot && s.conflict.second.cannotStop == secondCannot)
                    continue;
                s.conflict.first.onIt = firstOn;
                s.conflict.second.onIt = secondOn;
                s.conflict.first.cannotStop = firstCannot;
                s.conflict.second.cannotStop = secondCannot;
                s.resolution = resolve(s.conflict);
            }
        }
        i = e;
    }

    // (No ring breaking, no order imposed on a group of cars: each pair is answered by the questions both drivers ask of
    // what they see, sitting 6. A ring of waits that follows is counted by the wait-for graph, a measurement.)

    counts_.conflicts = settled_.size();
    for (const SettledConflict& s : settled_) {
        ++counts_.byQuestion[static_cast<std::size_t>(s.resolution.by)];
        // The car told to go second, not yet on the stretch, cannot keep short of it either: it cannot comply. Counted, so
        // it is never silent. (Two bodies both on a stretch already, as at a join where one follows the other, are not
        // counted: there is nothing left to comply with.)
        const Side& second = s.resolution.firstGoes ? s.conflict.second : s.conflict.first;
        if (second.cannotStop && !second.onIt) ++counts_.secondCannotStop;
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
