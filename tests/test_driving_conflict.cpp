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
//      questions made is broken in the fixed order (the lower id among equals), with the queue arrows a host passes in;
//      the one exception since #284: a loop of bodies (and cars that cannot stop) alone, which is left standing, never
//      broken by telling a car to drive through another (conflict_5), and an answer by on_it or cannot_stop is never
//      overruled;
//   6. A CAR THAT CANNOT STOP short of a crossing point goes first, whatever the table says (#284; conflict_6);
//   5. DETERMINISTIC: the same announcements written in any order, studied by a fresh or a reused resolver, give the
//      same conflicts and answers.

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

// THE QUEUE ARROWS of a drawn board, as a host passes them: on each path, its cars one behind another in the order they
// arrive (their earliest arrival, then the lower id): (the car ahead, the car behind it).
std::vector<std::pair<uint64_t, uint64_t>> queuesOf(const std::vector<Announcement>& list) {
    std::map<uint32_t, std::map<uint64_t, Real>> earliest;   // path -> car -> its earliest arrival there
    for (const Announcement& a : list) {
        std::map<uint64_t, Real>& cars = earliest[a.onPath.index];
        const std::map<uint64_t, Real>::iterator it = cars.find(a.who);
        if (it == cars.end()) cars[a.who] = a.arriveIn;
        else it->second = std::min(it->second, a.arriveIn);
    }
    std::vector<std::pair<uint64_t, uint64_t>> out;
    for (const std::pair<const uint32_t, std::map<uint64_t, Real>>& path : earliest) {
        std::vector<std::pair<Real, uint64_t>> order;
        for (const std::pair<const uint64_t, Real>& car : path.second) order.push_back({car.second, car.first});
        std::sort(order.begin(), order.end());
        for (std::size_t k = 0; k + 1 < order.size(); ++k) out.push_back({order[k].second, order[k + 1].second});
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

// THE NO-RINGS GUARANTEE as it stands since #284: an answer by on_it or cannot_stop is never overruled; so a loop of
// those alone (bodies locked round, which random boards do draw: three cars each with its body on the next one's
// crossing point) is left standing and counted (bodyRings), and then the loop is one of those answers alone; otherwise no
// group, nor all of them together, holds a loop.
std::vector<SettledConflict> fixedOnly(const std::vector<SettledConflict>& all) {
    std::vector<SettledConflict> out;
    for (const SettledConflict& s : all)
        if (fixedAnswer(s.resolution.by)) out.push_back(s);
    return out;
}
uint64_t gBoardsWithBodyRings = 0;
void checkNoRingsButBodies(const ConflictResolver& r, uint32_t groups = 0) {
    for (const SettledConflict& s : r.conflicts())
        if (s.resolution.by == Question::Ring) CHECK(!fixedAnswer(s.beforeRing.by));
    if (r.counts().bodyRings > 0) {
        ++gBoardsWithBodyRings;
        CHECK(hasLoop(fixedOnly(r.conflicts()), kAllGroups));
        return;
    }
    for (uint32_t group = 0; group <= groups; ++group) CHECK(!hasLoop(r.conflicts(), group));
    CHECK(!hasLoop(r.conflicts(), kAllGroups));   // nor through two groups
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
                              Question::Going, Question::ArrivedFirst, Question::Fallback,       Question::LowerId};
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
    CHECK(!followOnly(Question::Table) && !followOnly(Question::ArrivedFirst) && !followOnly(Question::Ring));
    CHECK(fixedAnswer(Question::OnIt) && fixedAnswer(Question::CannotStop) && !fixedAnswer(Question::CommittedFirst) &&
          !fixedAnswer(Question::Ring));
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
    uint64_t groupsWithRings = 0, queueRings = 0;
    gBoardsWithBodyRings = 0;
    for (uint64_t seed = 1; seed <= 400; ++seed) {
        Draw draw(seed * 7);
        const Ground g = makeGround(5 + draw.below(4), draw, seed % 3 == 0);
        AnnouncementBoard board;
        const std::vector<Announcement> list = drawBoard(g, draw, 4);
        sealInto(g, list, board);
        ConflictResolver r;
        // With the queue arrows a host passes (issue #284: the fuzz passed none before): the cars on each path held one
        // behind another, in the order they arrive.
        r.study(g.roads, g.row, board, queuesOf(list));
        groupsWithRings += r.counts().rings;
        queueRings += r.counts().physicalRings;
        checkNoRingsButBodies(r, static_cast<uint32_t>(g.roads.junctions.size()));
        // Every conflict a ring overruled says what the questions had answered.
        for (const SettledConflict& s : r.conflicts())
            if (s.resolution.by == Question::Ring) CHECK(s.beforeRing.by != Question::None && s.beforeRing.by != Question::Ring);
    }
    std::printf("    %llu rings the questions made were broken (%llu through a queue); none is left but %llu boards' rings of "
                "bodies alone, left standing\n",
                static_cast<unsigned long long>(groupsWithRings), static_cast<unsigned long long>(queueRings),
                static_cast<unsigned long long>(gBoardsWithBodyRings));
    CHECK(groupsWithRings > 0);   // the draw does make rings, so the breaking was tested
    CHECK(queueRings > 0);        // ... and rings through a queue, untied at the queue arrow

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
    CHECK(r.counts().rings == 1u && r.counts().bodyRings == 0u && r.counts().physicalRings == 0u);
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
    CHECK(r.order(41, 40, &by) == Answer::First);
    CHECK(by == Question::CannotStop);
    CHECK(r.counts().secondCannotStop == 0u);
    CHECK(followOnly(by));   // the car may only come behind the bus, never dash ahead of it
    // Both cannot stop: no answer there; the table decides, and the car told second is counted (it cannot comply).
    sealInto(g, board(true), b);
    r.study(g.roads, g.row, b);
    CHECK(r.order(41, 40, &by) == Answer::Second && by == Question::Table);
    CHECK(r.counts().secondCannotStop == 1u);
}

// ---- 4b. A ring of bodies alone is left standing (issue #284 item C) --------------------------------------------------

TEST_CASE(conflict_5_a_ring_of_bodies_is_left_standing_never_answered_by_an_order_to_go) {
    // Three paths through one junction, each pair crossing once: car 30 on path 0 has its body on the crossing point it
    // shares with path 1, car 31 on path 1 on the one it shares with path 2, car 32 on path 2 on the one it shares with path
    // 0. Each waits for the next one's body (on_it): a ring no answer can undo. Before #284 the ring was "broken" by placing
    // one car anyway and telling it to go through a body (counted as ringThroughABody). Now every on_it answer stands, the
    // ring is counted (bodyRings), and each car is still told to wait for the body it waits for.
    Ground g;
    g.roads.paths.resize(3);
    g.roads.junctions.resize(1);
    for (uint32_t a = 0; a < 3; ++a) {
        engine::CrossingPoint c;
        c.a = engine::PathId(a);
        c.other = (a + 1) % 3;
        c.junction = engine::JunctionId(0);
        g.roads.crossings.push_back(c);
        MeetingAnswer m;
        m.who = GivesWay::A;   // by the table path a gives way: the bodies say otherwise
        g.row.answersOf.push_back(engine::Range{a, 1});
        g.row.answers.push_back(m);
    }
    // Crossing point c is shared by paths c and c + 1; the car on path c + 1 has its body there.
    std::vector<Announcement> list;
    for (uint32_t c = 0; c < 3; ++c) {
        const uint32_t onIt = (c + 1) % 3, coming = c;
        Announcement body;
        body.crossing = engine::CrossingId(c);
        body.onPath = engine::PathId(onIt);
        body.who = 30 + onIt;
        body.arriveIn = 0;
        body.clearIn = kNever;   // it stands there
        list.push_back(body);
        Announcement wait;
        wait.crossing = engine::CrossingId(c);
        wait.onPath = engine::PathId(coming);
        wait.who = 30 + coming;
        wait.arriveIn = 1;
        wait.clearIn = 3;
        list.push_back(wait);
    }
    AnnouncementBoard board;
    sealInto(g, list, board);
    ConflictResolver r;
    r.study(g.roads, g.row, board);
    CHECK(r.conflicts().size() == 3u);
    CHECK(r.counts().bodyRings == 1u);
    for (const SettledConflict& s : r.conflicts()) {
        CHECK(s.resolution.by == Question::OnIt);   // never Ring: nobody is told to drive through a body
        CHECK(s.beforeRing.by == Question::None);
    }
    // Each car waits for the next one's body: 30 for 31, 31 for 32, 32 for 30.
    for (uint64_t car = 30; car < 33; ++car) {
        const uint64_t body = 30 + (car - 30 + 1) % 3;
        CHECK(r.order(car, body) == Answer::Second);
    }
    // THE WAIT-FOR GRAPH shows it as gridlock (a matter of space), not a deadlock: three cars at rest, each giving way to
    // the body of the next (StopFor::ForABody, as the judgement's stop is when the resolver answered by on_it).
    std::vector<WaitingCar> waiting;
    for (uint32_t car = 30; car < 33; ++car) {
        Car c;
        c.id = CarId(car);
        c.driven = true;
        c.held.decided = true;
        c.binding.by = Limit::Stop;
        c.binding.stopFor = StopFor::ForABody;
        c.binding.car = CarId(30 + (car - 30 + 1) % 3);
        waiting.push_back({c.id, waitReasonOf(c)});
    }
    const WaitReport report = analyseWaiting(waiting);
    CHECK(report.loops.size() == 1u && report.gridlocks == 1u && report.deadlocks == 0u);
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
