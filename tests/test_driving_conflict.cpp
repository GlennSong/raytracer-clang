// WHO GOES FIRST WHERE TWO CARS' PATHS MEET (issue #263, task D4; sitting 6: junctions by judgement; conflict.h): the
// questions a driver asks about another. Checked over many random boards on a hand-made junction whose every pair of
// paths crosses, with a give-way table drawn at random:
//
//   1. the questions are asked in the written order: each one decides over every one after it;
//   2. CONSISTENT: whoever asks, at the same instant, if A goes before B then B goes after A; and resolving the two sides
//      the other way round gives the mirror answer;
//   3. ALWAYS AN ANSWER: every pair of cars announced on a shared crossing point has one, decided by a named question
//      (a tie by the seeded hesitation, the same for both);
//   4. DETERMINISTIC: the same announcements written in any order, studied by a fresh or a reused resolver, give the
//      same conflicts and answers;
//   6. A CAR THAT CANNOT STOP short of a crossing point goes first, whatever the table says (#284; conflict_6).
// Nothing breaks rings any more (sitting 6): a ring of waits the answers leave is counted by the wait-for graph.

#include "test_framework.h"

#include "apps/driving/announce.h"
#include "apps/driving/conflict.h"
#include "apps/driving/right_of_way.h"
#include "apps/driving/wait_graph.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <map>
#include <set>
#include <utility>
#include <vector>

using namespace driving;

namespace {

// A small stateless stream of numbers, so every run draws the same boards.
struct Draw {
    uint64_t state;
    explicit Draw(uint64_t seed) : state(seed * 0x9E3779B97F4A7C15ull + 1) {}
    uint32_t next() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return static_cast<uint32_t>(state >> 11);
    }
    uint32_t below(uint32_t n) { return n ? next() % n : 0; }
    bool coin() { return (next() & 1u) != 0; }
};

// A HAND-MADE JUNCTION: `paths` connectors, every pair crossing once, filed under junction 0; and `unfiled` more pairs
// (path i with path i + 1) filed under no junction. Only what the resolver reads is filled in.
struct Ground {
    engine::LaneGraph roads;
    RightOfWay row;
};

Ground makeGround(uint32_t paths, Draw& draw, bool unfiled) {
    Ground g;
    g.roads.paths.resize(paths);
    g.roads.junctions.resize(1);
    for (uint32_t a = 0; a < paths; ++a)
        for (uint32_t b = a + 1; b < paths; ++b) {
            engine::CrossingPoint c;
            c.a = engine::PathId(a);
            c.otherSide = engine::OtherSide::Path;
            c.meeting = engine::Meeting::Cross;
            c.other = b;
            c.junction = engine::JunctionId(0);
            g.roads.crossings.push_back(c);
        }
    // Paths 0 and 1 meet twice (two paths crossing twice make two records): the table may answer the two differently.
    if (paths >= 2) {
        engine::CrossingPoint c;
        c.a = engine::PathId(0);
        c.otherSide = engine::OtherSide::Path;
        c.other = 1;
        c.junction = engine::JunctionId(0);
        g.roads.crossings.push_back(c);
    }
    if (unfiled)
        for (uint32_t a = 0; a + 1 < paths; ++a) {
            engine::CrossingPoint c;
            c.a = engine::PathId(a);
            c.otherSide = engine::OtherSide::Path;
            c.other = a + 1;
            g.roads.crossings.push_back(c);   // no junction
        }
    for (uint32_t c = 0; c < g.roads.crossings.size(); ++c) {
        MeetingAnswer m;
        switch (draw.below(3)) {
        case 0: m.who = GivesWay::A; break;
        case 1: m.who = GivesWay::Other; break;
        default:
            m.who = GivesWay::Ask;
            m.fallback = draw.coin() ? GivesWay::A : GivesWay::Other;
            break;
        }
        g.row.answersOf.push_back(engine::Range{c, 1});
        g.row.answers.push_back(m);
    }
    return g;
}

// A random car's announcement on crossing `c` from path `p`: its flags drawn from a few values, so ties are common.
Announcement drawAnnouncement(Draw& draw, uint32_t c, uint32_t p, uint64_t who, bool steady, const Announcement& base) {
    Announcement a = steady ? base : Announcement();
    a.crossing = engine::CrossingId(c);
    a.onPath = engine::PathId(p);
    a.who = who;
    if (!steady) {
        a.committed = draw.below(3) == 0;
        a.committedTick = draw.below(4);
        a.going = draw.coin();
        a.arrivalFixed = draw.below(3) != 0;
        a.arrivalTick = draw.below(4);
        a.cannotStop = draw.below(5) == 0;
        a.notGoing = draw.below(5) == 0;
    }
    a.arriveIn = draw.below(4) == 0 ? Real(0) : Real(1 + draw.below(5));
    a.clearIn = a.arriveIn + 2;
    return a;
}

// ONE RANDOM BOARD: on each path a few cars, each announcing on every crossing point along its path (most with one set
// of flags for all, as a real car's plan gives; some with flags drawn per crossing point).
std::vector<Announcement> drawBoard(const Ground& g, Draw& draw, uint32_t carsPerPath) {
    std::vector<Announcement> out;
    uint64_t who = 0;
    for (uint32_t p = 0; p < g.roads.paths.size(); ++p)
        for (uint32_t k = 0, n = 1 + draw.below(carsPerPath); k < n; ++k, ++who) {
            const bool steady = draw.below(4) != 0;
            const Announcement base = drawAnnouncement(draw, 0, p, who, false, Announcement());
            for (uint32_t c = 0; c < g.roads.crossings.size(); ++c) {
                const engine::CrossingPoint& rec = g.roads.crossings[c];
                if (rec.a.index != p && rec.other != p) continue;
                out.push_back(drawAnnouncement(draw, c, p, who, steady, base));
            }
        }
    return out;
}

void sealInto(const Ground& g, const std::vector<Announcement>& list, AnnouncementBoard& board) {
    board.begin(7, g.roads.crossings.size());
    for (const Announcement& a : list) board.add(a);
    board.seal();
}


Conflict mirror(const Conflict& c) {
    const auto flip = [](TableSays t) {
        return t == TableSays::FirstGivesWay ? TableSays::SecondGivesWay
               : t == TableSays::SecondGivesWay ? TableSays::FirstGivesWay
                                                : TableSays::Nothing;
    };
    Conflict m;
    m.first = c.second;
    m.second = c.first;
    m.table = flip(c.table);
    m.fallback = flip(c.fallback);
    return m;
}

}  // namespace

// ---- 1. The order of the questions ------------------------------------------------------------------------------------

TEST_CASE(conflict_1_the_questions_are_asked_in_the_written_order) {
    // A conflict where every question would put the SECOND car first, then each question in turn made to put the first
    // car first: that question decides, and none after it can overrule it.
    Conflict base;
    base.first.who = 9;
    base.second.who = 3;   // the lower id goes first: the second
    base.second.onIt = true;
    base.second.cannotStop = true;
    base.second.committed = true;
    base.second.committedTick = 1;
    base.first.committed = true;
    base.first.committedTick = 2;
    base.table = TableSays::FirstGivesWay;
    base.second.going = true;
    base.second.arrivalFixed = base.first.arrivalFixed = true;
    base.second.arrivalTick = 1;
    base.first.arrivalTick = 2;
    base.fallback = TableSays::FirstGivesWay;
    Resolution r = resolve(base);
    CHECK(!r.firstGoes && r.by == Question::OnIt);
    CHECK(kQuestions.size() == kAskedQuestions);
    // Undo the questions one at a time, from the top: each time the next question down decides.
    const Question order[] = {Question::OnIt,  Question::CannotStop,   Question::CommittedFirst, Question::Table,
                              Question::Going, Question::ArrivedFirst, Question::Hesitation, Question::Fallback, Question::Hesitation};
    for (std::size_t q = 0; q < kAskedQuestions; ++q) CHECK(kQuestions[q].question == order[q]);
    Conflict c = base;
    c.second.onIt = false;
    CHECK(resolve(c).by == Question::CannotStop && !resolve(c).firstGoes);
    c.second.cannotStop = false;
    CHECK(resolve(c).by == Question::CommittedFirst && !resolve(c).firstGoes);
    c.second.committed = c.first.committed = false;
    CHECK(resolve(c).by == Question::Table && !resolve(c).firstGoes);
    c.table = TableSays::Nothing;
    CHECK(resolve(c).by == Question::Going && !resolve(c).firstGoes);
    c.second.going = false;
    CHECK(resolve(c).by == Question::ArrivedFirst && !resolve(c).firstGoes);
    c.first.arrivalTick = c.second.arrivalTick;
    // both at their lines since the same tick: the bolder goes, before the fall-back (which can run in a ring there)
    CHECK(resolve(c).by == Question::Hesitation && resolve(mirror(c)).firstGoes != resolve(c).firstGoes);
    c.first.arrivalFixed = c.second.arrivalFixed = false;   // still driving up: the fall-back
    CHECK(resolve(c).by == Question::Fallback && !resolve(c).firstGoes);
    c.fallback = TableSays::Nothing;
    CHECK(resolve(c).by == Question::Hesitation && resolve(mirror(c)).by == Question::Hesitation &&
          resolve(mirror(c)).firstGoes != resolve(c).firstGoes);   // (the same hesitation, whichever side asks)
    // And each one decides over everything after it: the first car on it beats all the rest pointing the other way.
    c = base;
    c.second.onIt = false;
    c.first.onIt = true;
    CHECK(resolve(c).firstGoes && resolve(c).by == Question::OnIt);
    c = base;
    c.second.onIt = false;
    c.first.cannotStop = true;   // both cannot stop: no answer there; the next question decides
    CHECK(!resolve(c).firstGoes && resolve(c).by == Question::CommittedFirst);
    c.second.cannotStop = false;   // only the first cannot stop: it goes first, whatever comes after
    CHECK(resolve(c).firstGoes && resolve(c).by == Question::CannotStop);
    c = base;
    c.second.onIt = c.second.cannotStop = false;
    c.first.committedTick = 0;   // committed before the second
    CHECK(resolve(c).firstGoes && resolve(c).by == Question::CommittedFirst);
    // Only the three that say "a car to follow" forbid going ahead; only a body or a car that cannot stop is an answer
    // nobody may turn round.
    CHECK(followOnly(Question::OnIt) && followOnly(Question::CannotStop) && followOnly(Question::CommittedFirst));
    CHECK(!followOnly(Question::Table) && !followOnly(Question::ArrivedFirst) && !followOnly(Question::Hesitation));
    CHECK(fixedAnswer(Question::OnIt) && fixedAnswer(Question::CannotStop) && !fixedAnswer(Question::CommittedFirst) &&
          !fixedAnswer(Question::Hesitation));
}

// ---- 2 and 3. Consistent, and always an answer --------------------------------------------------------------------------

TEST_CASE(conflict_2_consistent_and_always_an_answer) {
    uint64_t pairs = 0, byQuestion[kQuestionCount] = {};
    for (uint64_t seed = 1; seed <= 400; ++seed) {
        Draw draw(seed);
        const Ground g = makeGround(4 + draw.below(4), draw, seed % 2 == 0);
        const std::vector<Announcement> list = drawBoard(g, draw, 3);
        AnnouncementBoard board;
        sealInto(g, list, board);
        ConflictResolver r;
        r.study(g.roads, g.row, board);
        // Every pair of cars announced on one crossing point from different paths has exactly one answer there, and
        // asking from either side agrees.
        for (uint32_t c = 0; c < g.roads.crossings.size(); ++c) {
            const AnnouncementRange on = board.on(engine::CrossingId(c));
            for (const Announcement& x : on)
                for (const Announcement& y : on) {
                    if (x.who == y.who || x.onPath == y.onPath) continue;
                    Question byX = Question::None, byY = Question::None;
                    const bool xFirst = r.goesFirst(g.roads, engine::CrossingId(c), x.who, y.who, &byX);
                    const bool yFirst = r.goesFirst(g.roads, engine::CrossingId(c), y.who, x.who, &byY);
                    CHECK(xFirst != yFirst);
                    CHECK(byX == byY);
                    CHECK(byX != Question::None);
                    const SettledConflict* s = r.find(ConflictResolver::groupOf(g.roads, engine::CrossingId(c)), x.who, y.who);
                    CHECK(s != nullptr);
                }
        }
        // Resolving each conflict with its sides the other way round gives the mirror answer by the same question.
        for (const SettledConflict& s : r.conflicts()) {
            ++pairs;
            ++byQuestion[static_cast<std::size_t>(s.resolution.by)];
            const Resolution there = resolve(s.conflict);
            const Resolution back = resolve(mirror(s.conflict));
            CHECK(there.by != Question::None && back.by == there.by && back.firstGoes != there.firstGoes);
        }
    }
    std::printf("    %llu conflicts settled; by question:", static_cast<unsigned long long>(pairs));
    for (uint8_t q = 0; q < kQuestionCount; ++q)
        std::printf(" %s %llu", questionName(static_cast<Question>(q)), static_cast<unsigned long long>(byQuestion[q]));
    std::printf("\n");
    // The draw reaches every question but the last. (The hesitation decides only when the table's answers at two shared
    // crossing points disagree and nothing else differs: conflict_1 stages it.)
    for (uint8_t q = static_cast<uint8_t>(Question::OnIt); q < kQuestionCount; ++q)
        if (static_cast<Question>(q) != Question::Hesitation) CHECK(byQuestion[q] > 0);
}

// ---- 4a. A car that cannot stop goes first (issue #284 item A) ---------------------------------------------------------

TEST_CASE(conflict_6_a_car_that_cannot_stop_short_goes_first_whatever_the_table_says) {
    // Two paths through one junction crossing once; by the table path 1 gives way to path 0. Car 41 on path 1 (a bus at
    // speed) has announced that it CANNOT keep its front short of the crossing point even braking as hard as it can; car 40
    // on path 0 can. Before #284 the table put the bus second, and the judgement assumed it could stop (it "never looked"
    // at whether it could): it drove on, a reversal the resolver never saw. Now cannot_stop, asked right after on_it, puts
    // the bus first, and the car that can stop waits; nothing is counted as unable to comply. Two cars that both cannot
    // stop go on to the next question (and are counted).
    Ground g;
    g.roads.paths.resize(2);
    g.roads.junctions.resize(1);
    engine::CrossingPoint c;
    c.a = engine::PathId(0);
    c.other = 1;
    c.junction = engine::JunctionId(0);
    g.roads.crossings.push_back(c);
    MeetingAnswer m;
    m.who = GivesWay::Other;   // path 1 gives way
    g.row.answersOf.push_back(engine::Range{0, 1});
    g.row.answers.push_back(m);
    const auto board = [&](bool carCannotStop) {
        std::vector<Announcement> list;
        Announcement car;
        car.crossing = engine::CrossingId(0);
        car.onPath = engine::PathId(0);
        car.who = 40;
        car.arriveIn = 1.2;
        car.clearIn = 2.5;
        car.cannotStop = carCannotStop;
        list.push_back(car);
        Announcement bus = car;
        bus.onPath = engine::PathId(1);
        bus.who = 41;
        bus.arriveIn = 1.0;
        bus.clearIn = 2.4;
        bus.cannotStop = true;
        list.push_back(bus);
        return list;
    };
    AnnouncementBoard b;
    sealInto(g, board(false), b);
    ConflictResolver r;
    r.study(g.roads, g.row, b);
    Question by = Question::None;
    CHECK(!r.goesFirst(g.roads, engine::CrossingId(0), 40, 41, &by));
    CHECK(by == Question::CannotStop);
    CHECK(r.counts().secondCannotStop == 0u);
    CHECK(followOnly(by));   // the car may only come behind the bus, never dash ahead of it
    // Both cannot stop: no answer there; the table decides, and the car told second is counted (it cannot comply).
    sealInto(g, board(true), b);
    r.study(g.roads, g.row, b);
    CHECK(r.goesFirst(g.roads, engine::CrossingId(0), 40, 41, &by) && by == Question::Table);
    CHECK(r.counts().secondCannotStop == 1u);
}

// ---- 5. Deterministic ----------------------------------------------------------------------------------------------------

TEST_CASE(conflict_4_deterministic) {
    for (uint64_t seed = 1; seed <= 100; ++seed) {
        Draw draw(seed * 13);
        const Ground g = makeGround(6, draw, true);
        std::vector<Announcement> list = drawBoard(g, draw, 3);
        AnnouncementBoard board;
        sealInto(g, list, board);
        ConflictResolver fresh;
        fresh.study(g.roads, g.row, board);
        // The same announcements written backwards and shuffled, into a resolver that has studied other boards.
        std::reverse(list.begin(), list.end());
        for (std::size_t i = list.size(); i > 1; --i) std::swap(list[i - 1], list[draw.below(static_cast<uint32_t>(i))]);
        AnnouncementBoard other;
        sealInto(g, list, other);
        static ConflictResolver reused;
        reused.study(g.roads, g.row, other);
        CHECK(fresh.hash() == reused.hash());
        CHECK(fresh.conflicts().size() == reused.conflicts().size());
        // Studying the same board again changes nothing.
        reused.study(g.roads, g.row, other);
        CHECK(fresh.hash() == reused.hash());
    }
}
