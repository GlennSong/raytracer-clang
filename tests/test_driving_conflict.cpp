// THE ONE CONFLICT RESOLVER (issue #263, task D4; src/apps/driving/conflict.h): who goes first where two cars' paths
// meet. Its four guarantees, each checked over many random boards on a hand-made junction whose every pair of paths
// crosses, with a give-way table drawn at random (settled answers either way, and "asked while driving" with fall-backs
// that run in rings, as kerb side does at a four-way):
//
//   1. the questions are asked in the written order: each one decides over every one after it;
//   2. CONSISTENT: whoever asks, at the same instant, if A goes before B then B goes after A; and resolving the two sides
//      the other way round gives the mirror answer;
//   3. ALWAYS AN ANSWER: every pair of cars announced on a shared crossing point has one, decided by a named question;
//   4. NO RINGS: in each group (a junction, or "no junction") the arrows "goes after" have no loop, and a ring the
//      questions made is broken in the fixed order (the lower id among equals);
//   5. DETERMINISTIC: the same announcements written in any order, studied by a fresh or a reused resolver, give the
//      same conflicts and answers.

#include "test_framework.h"

#include "apps/driving/announce.h"
#include "apps/driving/conflict.h"
#include "apps/driving/right_of_way.h"

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

// Every group together, for hasLoop.
constexpr uint32_t kAllGroups = 0xFFFFFFFFu;

// Does the "goes after" relation among the conflicts of `group` have a loop? (A depth-first walk with colours.)
bool hasLoop(const std::vector<SettledConflict>& all, uint32_t group) {
    std::map<uint64_t, std::vector<uint64_t>> after;   // car -> the cars it goes after
    for (const SettledConflict& s : all) {
        if (group != kAllGroups && s.group != group) continue;
        const uint64_t first = s.resolution.firstGoes ? s.conflict.first.who : s.conflict.second.who;
        const uint64_t second = s.resolution.firstGoes ? s.conflict.second.who : s.conflict.first.who;
        after[second].push_back(first);
        after[first];
    }
    std::map<uint64_t, int> colour;   // 0 unseen, 1 on the walk, 2 done
    bool loop = false;
    std::vector<std::pair<uint64_t, std::size_t>> stack;
    for (const std::pair<const uint64_t, std::vector<uint64_t>>& start : after) {
        if (colour[start.first] != 0) continue;
        stack.push_back({start.first, 0});
        colour[start.first] = 1;
        while (!stack.empty() && !loop) {
            std::pair<uint64_t, std::size_t>& top = stack.back();
            const std::vector<uint64_t>& next = after[top.first];
            if (top.second == next.size()) {
                colour[top.first] = 2;
                stack.pop_back();
                continue;
            }
            const uint64_t n = next[top.second++];
            if (colour[n] == 1) loop = true;
            else if (colour[n] == 0) {
                colour[n] = 1;
                stack.push_back({n, 0});
            }
        }
        if (loop) break;
    }
    return loop;
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
    const Question order[] = {Question::OnIt,  Question::CommittedFirst, Question::Table,  Question::Going,
                              Question::ArrivedFirst, Question::Fallback, Question::LowerId};
    for (std::size_t q = 0; q < kAskedQuestions; ++q) CHECK(kQuestions[q].question == order[q]);
    Conflict c = base;
    c.second.onIt = false;
    CHECK(resolve(c).by == Question::CommittedFirst && !resolve(c).firstGoes);
    c.second.committed = c.first.committed = false;
    CHECK(resolve(c).by == Question::Table && !resolve(c).firstGoes);
    c.table = TableSays::Nothing;
    CHECK(resolve(c).by == Question::Going && !resolve(c).firstGoes);
    c.second.going = false;
    CHECK(resolve(c).by == Question::ArrivedFirst && !resolve(c).firstGoes);
    c.first.arrivalTick = c.second.arrivalTick;
    CHECK(resolve(c).by == Question::Fallback && !resolve(c).firstGoes);
    c.fallback = TableSays::Nothing;
    CHECK(resolve(c).by == Question::LowerId && !resolve(c).firstGoes);
    // And each one decides over everything after it: the first car on it beats all the rest pointing the other way.
    c = base;
    c.second.onIt = false;
    c.first.onIt = true;
    CHECK(resolve(c).firstGoes && resolve(c).by == Question::OnIt);
    c = base;
    c.second.onIt = false;
    c.first.committedTick = 0;   // committed before the second
    CHECK(resolve(c).firstGoes && resolve(c).by == Question::CommittedFirst);
    // Only the two that say "a car to follow" forbid going ahead.
    CHECK(followOnly(Question::OnIt) && followOnly(Question::CommittedFirst));
    CHECK(!followOnly(Question::Table) && !followOnly(Question::ArrivedFirst) && !followOnly(Question::Ring));
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
    // The draw reaches every question but the last, and rings. (The lower id decides only when the table's answers at
    // two shared crossing points disagree and nothing else differs: conflict_1 stages it.)
    for (uint8_t q = static_cast<uint8_t>(Question::OnIt); q < kQuestionCount; ++q)
        if (static_cast<Question>(q) != Question::LowerId) CHECK(byQuestion[q] > 0);
}

// ---- 4. No rings ---------------------------------------------------------------------------------------------------------

TEST_CASE(conflict_3_no_rings) {
    // The order rings are broken by is strict and total over cars with different ids (one of any two goes first, and
    // never both), and transitive.
    {
        Draw draw(99);
        std::vector<Side> sides;
        for (uint64_t who = 0; who < 40; ++who) {
            Side s;
            s.who = who;
            s.committed = draw.coin();
            s.committedTick = draw.below(3);
            s.going = draw.coin();
            s.arrivalFixed = draw.coin();
            s.arrivalTick = draw.below(3);
            sides.push_back(s);
        }
        for (const Side& a : sides)
            for (const Side& b : sides) {
                if (a.who == b.who) {
                    CHECK(!ringOrder(a, b));
                    continue;
                }
                CHECK(ringOrder(a, b) != ringOrder(b, a));
                for (const Side& c : sides)
                    if (ringOrder(a, b) && ringOrder(b, c)) CHECK(ringOrder(a, c));
            }
    }
    uint64_t groupsWithRings = 0;
    for (uint64_t seed = 1; seed <= 400; ++seed) {
        Draw draw(seed * 7);
        const Ground g = makeGround(5 + draw.below(4), draw, seed % 3 == 0);
        AnnouncementBoard board;
        sealInto(g, drawBoard(g, draw, 4), board);
        ConflictResolver r;
        r.study(g.roads, g.row, board);
        groupsWithRings += r.counts().rings;
        for (uint32_t group = 0; group <= g.roads.junctions.size(); ++group) CHECK(!hasLoop(r.conflicts(), group));
        CHECK(!hasLoop(r.conflicts(), kAllGroups));   // nor through two groups
        // A body is overruled only inside a loop made of bodies alone. (Random boards draw such impossible ones: three
        // cars each with its body on the next one's crossing point. The queue case below is the real one.)
        if (r.counts().physicalRings == 0) CHECK(r.counts().ringThroughABody == 0u);
        // Every conflict a ring overruled says what the questions had answered.
        for (const SettledConflict& s : r.conflicts())
            if (s.resolution.by == Question::Ring) CHECK(s.beforeRing.by != Question::None && s.beforeRing.by != Question::Ring);
    }
    std::printf("    %llu rings the questions made were broken; none is left\n",
                static_cast<unsigned long long>(groupsWithRings));
    CHECK(groupsWithRings > 0);   // the draw does make rings, so the breaking was tested

    // THE CLASSIC RING: four cars reaching a four-way in one tick, each with another on its kerb side (fall-backs in a
    // circle). The lower id goes first among them, as D3 did; the rest follow in the total order.
    Ground g;
    g.roads.paths.resize(4);
    g.roads.junctions.resize(1);
    for (uint32_t a = 0; a < 4; ++a) {
        engine::CrossingPoint c;
        c.a = engine::PathId(a);
        c.other = (a + 1) % 4;
        c.junction = engine::JunctionId(0);
        g.roads.crossings.push_back(c);
        MeetingAnswer m;
        m.who = GivesWay::Ask;
        m.fallback = GivesWay::A;   // path a gives way to path a + 1: round and round
        g.row.answersOf.push_back(engine::Range{a, 1});
        g.row.answers.push_back(m);
    }
    std::vector<Announcement> list;
    for (uint32_t c = 0; c < 4; ++c)
        for (const uint32_t p : {g.roads.crossings[c].a.index, g.roads.crossings[c].other}) {
            Announcement a;
            a.crossing = engine::CrossingId(c);
            a.onPath = engine::PathId(p);
            a.who = 10 + p;   // car 10 + p on path p
            a.arriveIn = 3;
            a.clearIn = 5;
            a.arrivalFixed = true;
            a.arrivalTick = 100;
            list.push_back(a);
        }
    AnnouncementBoard board;
    sealInto(g, list, board);
    ConflictResolver r;
    r.study(g.roads, g.row, board);
    CHECK(r.conflicts().size() == 4u);
    CHECK(r.counts().rings == 1u);
    CHECK(!hasLoop(r.conflicts(), 0));
    uint32_t overruled = 0;
    for (const SettledConflict& s : r.conflicts()) {
        CHECK(s.resolution.firstGoes);   // the lower id of each pair: car 10, then 11, 12, 13
        if (s.resolution.by == Question::Ring) {
            ++overruled;
            CHECK(s.beforeRing.by == Question::Fallback);
        } else {
            CHECK(s.resolution.by == Question::Fallback);   // car 13 gave way to car 10 already
        }
    }
    CHECK(overruled == 3u);

    // A RING THROUGH A QUEUE: car 20 waits for car 21's body on their crossing point (on_it), car 21 is held behind
    // car 22 in its lane, and car 22 would go after car 20 by the table. The ring cannot be broken through the body or
    // the queue: car 22 goes before car 20, and car 20 still waits for car 21.
    Ground q;
    q.roads.paths.resize(3);
    q.roads.junctions.resize(1);
    for (const std::pair<uint32_t, uint32_t>& pq : {std::make_pair(0u, 1u), std::make_pair(0u, 2u)}) {
        engine::CrossingPoint c;
        c.a = engine::PathId(pq.first);
        c.other = pq.second;
        c.junction = engine::JunctionId(0);
        q.roads.crossings.push_back(c);
        MeetingAnswer m;
        m.who = GivesWay::Other;   // path 0 has the right of way by the table at both
        q.row.answersOf.push_back(engine::Range{static_cast<uint32_t>(q.row.answers.size()), 1});
        q.row.answers.push_back(m);
    }
    std::vector<Announcement> ql;
    const auto put = [&](uint32_t c, uint32_t p, uint64_t who, Real arriveIn) {
        Announcement a;
        a.crossing = engine::CrossingId(c);
        a.onPath = engine::PathId(p);
        a.who = who;
        a.arriveIn = arriveIn;
        a.clearIn = arriveIn + 3;
        ql.push_back(a);
    };
    put(0, 0, 20, 2);   // car 20 on path 0, short of both crossing points
    put(1, 0, 20, 4);
    put(0, 1, 21, 0);   // car 21's body on crossing point 0
    put(1, 2, 22, 3);   // car 22 coming to crossing point 1
    sealInto(q, ql, board);
    const std::vector<std::pair<uint64_t, uint64_t>> held = {{22, 21}};   // car 21 is held behind car 22
    r.study(q.roads, q.row, board, held);
    const SettledConflict* body = r.find(0, 20, 21);
    const SettledConflict* table = r.find(0, 20, 22);
    CHECK(body && table);
    if (body && table) {
        CHECK(!body->resolution.firstGoes && body->resolution.by == Question::OnIt);   // 20 still waits for 21's body
        CHECK(!table->resolution.firstGoes && table->resolution.by == Question::Ring);   // 22 now goes before 20
        CHECK(table->beforeRing.by == Question::Table && table->beforeRing.firstGoes);
    }
    CHECK(r.counts().rings == 1u && r.counts().ringThroughABody == 0u && r.counts().physicalRings == 0u);
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
