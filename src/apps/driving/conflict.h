#ifndef RAYTRACER_APPS_DRIVING_CONFLICT_H
#define RAYTRACER_APPS_DRIVING_CONFLICT_H

// WHO GOES FIRST WHERE TWO CARS' PATHS MEET: THE QUESTIONS A DRIVER ASKS ABOUT ANOTHER (driving-model-design.md 9.3-9.6;
// two-drivers-meeting-design.md sections 6 and 10, sitting 6: junctions by judgement; ADR-0405).
//
// Words: a CROSSING POINT is a place where two paths' cars could touch (ADR-0400); an ANNOUNCEMENT is a car's estimate of
// when it will be on one (announce.h); a CONFLICT is two cars that have both announced on crossing points their two paths
// share; a car is COMMITTED once its front is past its last waiting point (junction_plan.h), and GOING once it can no
// longer stop at its line braking comfortably; a car's ARRIVAL is the tick it reached (or will reach) its line, fixed
// once (give_way.h). A GROUP is a junction (the crossing points filed under it), or "no junction".
//
// WHAT IT DOES. Each pair of cars met on crossing points of one group is asked a fixed list of QUESTIONS, in order, until
// one tells them apart. Each is a small pure function of what the two announced and what the give-way table (the law as
// drivers know it, right_of_way.h) says, so the two drivers, each asking of what it sees, reach the same answer:
//
//   1. on_it           -- whose body is already on a crossing point the two share: that one goes first.
//   2. cannot_stop     -- who cannot keep short of one even braking as hard as it can: it will be on it, so it goes.
//   3. committed_first -- who committed first ("first in goes first").
//   4. table           -- the give-way table's settled answer (lights, signs, movement; between junctions the rule
//                         set's between_junctions). "Asked while driving", or answers that disagree: no answer.
//   5. going           -- who can no longer stop at its line goes before one that can.
//   6. arrived_first   -- who reached its line first.
//   7. hesitation      -- both already at their lines, reached in the same tick: the bolder goes (each driver's own
//                         seeded boldness, compared: no loop among any number of cars).
//   8. fallback        -- the table's fall-back for a meeting asked while driving (kerb side, movement).
//   9. hesitation      -- still a tie: the bolder goes.
//
// NOTHING IS ARBITRATED. There is no order imposed on a group of cars and no ring breaking (both removed in sitting 6):
// a ring of waits that the answers leave is a stand-off, counted by the wait-for graph (wait_graph.h, a measurement), and
// fixed in the drivers' habits or the roads, never by an arbiter. The answers are worked out once a tick and looked up,
// which is the same as each driver working out its own pairs.
//
// NOT HERE, ON PURPOSE: a car announced NOT GOING (stopping short of the crossing for another reason) is passed over by
// the judgement of a car that would give way to it (give_way.h): nobody is coming. As a question here it made the two
// swap places every decision (issue #284).
//
// WHAT THE ONE WHO GOES SECOND DOES is the judgement's (give_way.h): it keeps out of the other's way by the rule set's
// gaps. Questions 1, 2 and 3 say it may only come BEHIND the other (followOnly).
//
// POLICY AND MECHANISM: who yields to whom, every gap and every number live in the rule set (assets/scripts/rules/*.lua),
// read into the give-way table at load. This file holds no rule-set number and names no kind of control or vehicle.
//
// Same result every run: built from the sorted board, walked in crossing-id order, sorted by (group, car, car).
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

// THE QUESTIONS, in the order they are asked (None: not resolved).
enum class Question : uint8_t {
    None,
    OnIt,
    CannotStop,
    CommittedFirst,
    Table,
    Going,
    ArrivedFirst,
    Fallback,
    Hesitation,
};
constexpr uint8_t kQuestionCount = 9;
// "on_it", "cannot_stop", "committed_first", ... (the names above).
const char* questionName(Question q);
// What it means when it decided, from the side of the one going second: "its body is already on the crossing point",
// "it committed first", ...
const char* questionWords(Question q);
// The one going second may only come behind the other, never go ahead of it (on_it, cannot_stop, committed_first).
bool followOnly(Question q);
// An answer nobody can turn round, since it says where a car physically is or will be (on_it, cannot_stop).
bool fixedAnswer(Question q);

// ONE CAR'S SIDE OF A CONFLICT, as the board shows it over the crossing points the two share in one group.
struct Side {
    uint64_t who = 0;                // the car (Announcement::who)
    bool onIt = false;               // its body is on one of the shared crossing points' stretches
    bool cannotStop = false;         // it cannot keep short of one of them braking as hard as it can (or is on it)
    bool committed = false;         // it can no longer wait short of its next crossing stretch
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
Answer askCannotStop(const Conflict& c);
Answer askCommittedFirst(const Conflict& c);
Answer askTable(const Conflict& c);
Answer askGoing(const Conflict& c);
Answer askArrivedFirst(const Conflict& c);
Answer askFallback(const Conflict& c);
Answer askHesitation(const Conflict& c);
Answer askTieAtLines(const Conflict& c);

// THE LIST, in order: every question the resolver asks, by name and function.
struct QuestionSpec {
    Question question;
    Answer (*ask)(const Conflict&);
};
constexpr std::size_t kAskedQuestions = 9;
extern const std::array<QuestionSpec, kAskedQuestions> kQuestions;

// ASKS THE QUESTIONS IN ORDER until one answers. Always answers (two sides with one id: the first goes, by LowerId).
Resolution resolve(const Conflict& c);

// ONE CONFLICT AS SETTLED THIS TICK.
struct SettledConflict {
    uint32_t group = 0;              // a junction's index, or the graph's junction count for "no junction"
    Conflict conflict;               // first.who < second.who
    Resolution resolution;
};

// WHAT ONE TICK'S STUDY FOUND.
struct ConflictCounts {
    uint64_t conflicts = 0;
    std::array<uint64_t, kQuestionCount> byQuestion{};   // conflicts answered by each question
    uint64_t secondCannotStop = 0;                       // conflicts whose car going second cannot keep short of the
                                                         // crossing either (both cannot stop, or it cannot and the
                                                         // other's body is there): it cannot comply. Should stay 0
};
// Adds `b` to `a`.
void addCounts(ConflictCounts& a, const ConflictCounts& b);


// THE RESOLVER: settles every conflict on a sealed board, once a tick; then answers "who goes first" by lookup.
class ConflictResolver {
public:
    // Settles every conflict on `board` (sealed for this tick), each junction's table read at the phase its lights are in
    // at clock `now` (signal_programme.h currentPhase: a pure function of the clock).
    void study(const engine::LaneGraph& roads, const RightOfWay& row, const AnnouncementBoard& board, double now = 0);

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

    std::vector<Met> met_;
    std::vector<SettledConflict> settled_;
    std::vector<uint32_t> byPair_;   // settled_ by pair of cars, then group (order() reads it after the study)
    ConflictCounts counts_;
    uint32_t noJunction_ = 0;
};

}  // namespace driving

#endif
