#ifndef RAYTRACER_APPS_DRIVING_CONFLICT_H
#define RAYTRACER_APPS_DRIVING_CONFLICT_H

// WHO GOES FIRST WHERE TWO CARS' PATHS MEET: THE ONE CONFLICT RESOLVER (driving-model-design.md 9.3-9.6; issue #263,
// docs/driving-briefs/D4-inside-the-junction.md; ADR-0405).
//
// Words: a CROSSING POINT is a place where two paths' cars could touch (ADR-0400); an ANNOUNCEMENT is a car saying when
// it will be on one (announce.h); a CONFLICT is two cars that have both announced on crossing points their two paths
// share; a car is COMMITTED once its front is past its last waiting point (junction_plan.h), and GOING once it can no
// longer stop at its line braking comfortably; a car's ARRIVAL is the tick it reached (or will reach) its line, fixed
// once (give_way.h). A GROUP is where a conflict is settled: one junction (the crossing points filed under it), or "no
// junction" (crossing points filed under none: two junctions' connectors, or a connector and a lane).
//
// WHAT IT DOES. Once a tick, after the board of announcements is sealed, every pair of cars announced on the same
// crossing point (from different paths, at a meeting where somebody must give way) becomes one CONFLICT per group,
// gathered over every crossing point the two share there. For each, this asks a fixed list of named QUESTIONS, in
// order, until one tells the two apart; that one says who goes first. Each question is a small pure function of what
// the two cars announced and what the give-way table says (`ask...` below):
//
//   1. on_it          -- whose body is already on a crossing point the two share (its front is past the stretch's
//                        start and its tail not past its end): that one goes first. A body on a stretch blocks it for
//                        everyone, whoever has right of way (the brief's rule 1). Both on one: no answer.
//   2. committed_first -- who committed first: a committed car before one that is not; two committed cars by the tick
//                        each committed. "First in goes first" (the brief's rule 2; design 9.4).
//   3. table          -- the give-way table's settled answer (right_of_way.h: the lights, the signs, the movement, the
//                        lower path id, as the junction's control lists them in the rule set). "Asked while driving",
//                        or answers that disagree across the crossing points the two share: no answer.
//   4. going          -- who can no longer stop at its line: it goes before one that can (D3).
//   5. arrived_first  -- who reached its line first, by its fixed arrival tick; a car whose arrival is fixed before one
//                        whose is not (D3).
//   6. fallback       -- the table's fall-back for a meeting asked while driving (kerb side, movement, lower path id,
//                        as the control lists them).
//   7. lower_id       -- the lower car id goes first. It cannot tie, so every conflict gets an answer.
//
// Why this order. Questions 1 and 2 are the brief's rules 1 and 2, asked before anything else because once a car is
// inside it cannot give way at the line any more: it can only be waited for. Questions 3 to 6 are D3's giving way for
// cars not yet committed, in D3's order (the table's settled answers first; then, where the table says "ask while
// driving", going, arrival, the fall-back). Question 7 is the tie-break that cannot tie.
//
// WHAT THE ONE WHO GOES SECOND DOES is the judgement's (give_way.h), not this file's: it keeps out of the other's way by
// the gaps the rule set gives. Questions 1 and 2 also say it may only come BEHIND the other (followOnly): an earlier
// committed car's crossing point, or one a body is on, is "a car to follow until its tail is clear" -- never one to
// dash ahead of.
//
// POLICY AND MECHANISM. Who yields to whom at a meeting (the questions a control asks, the ranks, which turns give way
// on a green), every gap and every number live in the rule set (assets/scripts/rules/*.lua), read into the give-way
// table at load. This file is the mechanism: it holds no number and names no kind of control, sign or vehicle.
//
// THE FOUR GUARANTEES (tests/test_driving_conflict.cpp checks each):
//   - CONSISTENT: a conflict is ONE record per pair of cars per group, worked out once a tick from the sealed board, so
//     if A goes before B then B goes after A, at the same instant, whoever asks (and a body on one of their shared
//     crossing points counts in every group the two share, so bodies never say opposite things);
//   - ALWAYS AN ANSWER: the last question cannot tie;
//   - NO RINGS: the "goes after" arrows of every conflict, in every group together, are checked for loops (a car
//     reaching itself by following them), with the arrows nobody can turn round: a car held BEHIND the car ahead of it in
//     its lane (the host passes these in; a queue is looked along to the next car in a conflict), and a conflict settled
//     by on_it (a body is where it is: a body on a crossing point the two share in one group counts in each of their
//     groups). All groups together, because a car can be held at one junction's line by its conflict at the next (its
//     body would not fit between the two: give_way.h), and two junctions' paths can overlap. Every loop lies inside
//     one strongly connected part of the arrows (Tarjan's method: a part where each car can reach each other). Inside
//     such a part the cars are put in one TOTAL ORDER: an order that keeps every fixed arrow (each car after the ones
//     it is held behind or must wait for the body of), choosing among the cars free to come next by ringOrder below
//     (committed first, by when; then going; then by arrival; then the lower id); and each conflict inside the part is
//     answered by it. Inside the part every arrow now follows one order, which has no loop; arrows between parts were
//     already loop free, so the whole has none. Most rings are the table's own (an all-way stop where each of four cars
//     has another on its kerb side, reaching it in one tick): broken by the lower id, as D3 did. A loop of fixed arrows
//     alone is counted (physicalRings): the order then first sets aside a queue arrow (putting a car before the one ahead
//     of it in its lane answers nothing it does), and only if bodies alone are locked round (ringThroughABody, never seen
//     in the lab) a body's; the wait-for graph shows any such loop.
//   - ONE MORE GUARD, in the judgement (give_way.h): nobody plans to go BEHIND a car that waits inside the junction for
//     somebody else. Committing behind it would put it after you, and a stream of such cars would hold it in the box.
//   - DETERMINISTIC: built from the board (itself sorted), walked in crossing-id order, sorted by (group, car, car,
//     crossing): no pointer, no unordered container, no clock.
//
// Cost: a flat list of the pairs met on each crossing point, sorted once a tick, merged into conflicts; one walk over
// the arrows (Tarjan's, linear in cars and arrows); an ordering only inside the rare parts holding a loop. All memory is
// kept from tick to tick: the steady state allocates nothing.
//
// Units: none (ticks and ids).

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/right_of_way.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace driving {

class AnnouncementBoard;

// THE QUESTIONS, in the order they are asked (None: not resolved; Ring: answered by the total order that breaks a ring).
enum class Question : uint8_t {
    None,
    OnIt,
    CommittedFirst,
    Table,
    Going,
    ArrivedFirst,
    Fallback,
    LowerId,
    Ring,
};
constexpr uint8_t kQuestionCount = 9;
// "on_it", "committed_first", ... (the names above).
const char* questionName(Question q);
// What it means when it decided, from the side of the one going second: "its body is already on the crossing point",
// "it committed first", ...
const char* questionWords(Question q);
// The one going second may only come behind the other, never go ahead of it.
bool followOnly(Question q);

// ONE CAR'S SIDE OF A CONFLICT, as the board shows it over the crossing points the two share in one group.
struct Side {
    uint64_t who = 0;                // the car (Announcement::who)
    bool onIt = false;               // its body is on one of the shared crossing points' stretches
    bool committed = false;          // past its last waiting point
    uint64_t committedTick = 0;      // ... since this tick (the earliest, over the shared crossing points)
    bool going = false;              // it can no longer stop at its line
    bool arrivalFixed = false;
    uint64_t arrivalTick = 0;        // its fixed arrival at its line (the earliest)
};

// WHAT THE GIVE-WAY TABLE SAYS between the two (right_of_way.h), read from the first side's path.
enum class TableSays : uint8_t { Nothing, FirstGivesWay, SecondGivesWay };

// ONE CONFLICT: two cars, the table's settled answer over their shared crossing points, and its fall-back where it is
// asked while driving.
struct Conflict {
    Side first, second;
    TableSays table = TableSays::Nothing;      // all shared crossing points the table settles agree (none: Nothing)
    TableSays fallback = TableSays::Nothing;   // asked while driving: who gives way if asking ties (the lowest crossing)
};

// WHO GOES FIRST, by one question.
enum class Answer : uint8_t { NoAnswer, First, Second };

struct Resolution {
    bool firstGoes = true;
    Question by = Question::None;
};
bool operator==(const Resolution& a, const Resolution& b);

// THE QUESTIONS, each a pure function (see the top of this file).
Answer askOnIt(const Conflict& c);
Answer askCommittedFirst(const Conflict& c);
Answer askTable(const Conflict& c);
Answer askGoing(const Conflict& c);
Answer askArrivedFirst(const Conflict& c);
Answer askFallback(const Conflict& c);
Answer askLowerId(const Conflict& c);

// THE LIST, in order: every question the resolver asks, by name and function.
struct QuestionSpec {
    Question question;
    Answer (*ask)(const Conflict&);
};
constexpr std::size_t kAskedQuestions = 7;
extern const std::array<QuestionSpec, kAskedQuestions> kQuestions;

// ASKS THE QUESTIONS IN ORDER until one answers. Always answers (two sides with one id: the first goes, by LowerId).
Resolution resolve(const Conflict& c);
// THE TOTAL ORDER a ring is broken by: does `a` go before `b`? Committed before not, earlier commit first; then going
// before not; then a fixed arrival before none, earlier first; then the lower id. Strict and total over distinct ids.
bool ringOrder(const Side& a, const Side& b);

// ONE CONFLICT AS SETTLED THIS TICK.
struct SettledConflict {
    uint32_t group = 0;              // a junction's index, or the graph's junction count for "no junction"
    Conflict conflict;               // first.who < second.who
    Resolution resolution;
    Resolution beforeRing;           // what the questions said, when a ring overruled it (by == None otherwise)
};

// WHAT ONE TICK'S STUDY FOUND.
struct ConflictCounts {
    uint64_t conflicts = 0;
    std::array<uint64_t, kQuestionCount> byQuestion{};   // conflicts answered by each question (Ring included)
    uint64_t rings = 0;                                  // strongly connected parts holding a loop, answered by the order
    uint64_t ringThroughABody = 0;                       // conflicts settled by on_it that a ring overruled (a loop of
                                                         // fixed arrows: should stay 0)
    uint64_t physicalRings = 0;                          // loops of fixed arrows alone (bodies and queues)
    uint64_t queuesCut = 0;                              // queues not looked along past kMaxQueueLooked cars
};
// Adds `b` to `a`.
void addCounts(ConflictCounts& a, const ConflictCounts& b);

// HOW MANY CARS of one queue the ring check looks along, from a car in a conflict to the next cars ahead of it that are
// in one too. A loop of waiting through a longer queue is not plausible (the lab's longest queue is five); one cut short
// is counted (ConflictCounts::queuesCut). A ring road full of cars each following the next would otherwise be walked for
// ever.
constexpr std::size_t kMaxQueueLooked = 16;

// THE RESOLVER: settles every conflict on a sealed board, once a tick; then answers "who goes first" by lookup.
class ConflictResolver {
public:
    // Settles every conflict on `board` (sealed for this tick), each junction's table read at its current light phase
    // (right_of_way.h currentPhase).
    // `behind` lists the arrows nobody can turn round from outside the board: (the car ahead, the car behind it on one
    // path: the next in that path's list), as the cars stood at the start of the tick, in any order.
    void study(const engine::LaneGraph& roads, const RightOfWay& row, const AnnouncementBoard& board,
               const std::vector<std::pair<uint64_t, uint64_t>>& behind = {});

    // The group a crossing point is settled in: its junction, or "no junction".
    static uint32_t groupOf(const engine::LaneGraph& roads, engine::CrossingId crossing);
    // THE CONFLICT between cars `a` and `b` in `group`, or null when they have none there this tick.
    const SettledConflict* find(uint32_t group, uint64_t a, uint64_t b) const;
    // DOES CAR `me` GO BEFORE CAR `them` where they meet on crossing point `crossing`? True when they have no conflict
    // there (nothing to give way to). `by` says which question decided (None for no conflict).
    bool goesFirst(const engine::LaneGraph& roads, engine::CrossingId crossing, uint64_t me, uint64_t them,
                   Question* by = nullptr) const;

    // Every conflict settled this tick, in (group, first, second) order.
    const std::vector<SettledConflict>& conflicts() const { return settled_; }
    // What this tick's study found.
    const ConflictCounts& counts() const { return counts_; }
    // One number over this tick's conflicts and answers, in order (the sameness tests compare it).
    uint64_t hash() const;

private:
    // One pair of cars met on one crossing point.
    struct Met {
        uint32_t group = 0;
        uint32_t crossing = 0;
        Side lo, hi;                       // lo.who < hi.who
        TableSays settled = TableSays::Nothing;   // the table settles this crossing point (from lo's side)
        TableSays fallback = TableSays::Nothing;  // asked while driving here: its fall-back (from lo's side)
        bool asked = false;
    };
    // One arrow: car `from` (an index into nodes_) goes after car `to`; `conflict` is the settled conflict it came from
    // (kFixedArrow for a car held behind another); `fixed`: nobody can turn it round.
    struct Arrow {
        uint32_t from = 0, to = 0;
        uint32_t conflict = 0;
        bool fixed = false;
    };
    static constexpr uint32_t kFixedArrow = 0xFFFFFFFFu;
    void breakRings(const std::vector<std::pair<uint64_t, uint64_t>>& behind);
    void breakRingsIn(std::size_t first, std::size_t end);
    uint32_t nodeOf(uint64_t who) const;
    void strongParts();
    void orderPart(uint32_t part);

    std::vector<Met> met_;
    std::vector<SettledConflict> settled_;
    std::vector<uint32_t> byPair_;   // settled_ by pair of cars, then group (working memory)
    // While breaking rings (working memory, kept from tick to tick): the cars (sorted), each one's side for the order,
    // the arrows (by `from`, with where each car's begin), and the strongly connected parts (Tarjan's walk).
    std::vector<uint64_t> nodes_;
    std::vector<Side> nodeSides_;
    std::vector<Arrow> arrows_;
    std::vector<uint32_t> firstArrow_;
    std::vector<uint32_t> part_, partSize_, lowLink_, visitOrder_, stack_, walk_, walkArrow_;
    std::vector<uint8_t> onStack_;
    std::vector<uint32_t> partNodes_, waitingOn_, waitingOnBody_, rank_;
    std::vector<uint8_t> placed_;
    std::vector<std::pair<uint64_t, uint64_t>> aheadOf_;   // (a car behind another, the car ahead), sorted
    std::vector<uint64_t> queueWalk_;                      // the cars looked along one queue
    ConflictCounts counts_;
    uint32_t noJunction_ = 0;
};

}  // namespace driving

#endif
