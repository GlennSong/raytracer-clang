#ifndef RAYTRACER_APPS_DRIVING_ANNOUNCE_H
#define RAYTRACER_APPS_DRIVING_ANNOUNCE_H

// WHO WILL BE WHERE, AND WHEN: THE BOARD OF ANNOUNCEMENTS (driving-model-design.md 4 "Announcers", 9.2; issue #262,
// docs/driving-briefs/D3-give-way.md).
//
// At the start of every tick, every driven car with a junction within its look-ahead TIME (the rule set's
// `junction_look_ahead`, about 8 seconds: a far-off slow car announces nothing, so it holds nobody) writes, for each
// crossing point along its connector through that junction, an ANNOUNCEMENT: when it expects its front to reach its
// stretch there, when its tail will be past the stretch's end (vehicle length matters here), when it arrives at the line
// (fixed once, for who goes first: give_way.h), whether it is going (it can no longer stop before the line), whether it
// CANNOT STOP short of the crossing at its hardest braking (issue #284), and whether it is NOT GOING (it is stopping
// before the crossing for a reason other than this junction's traffic: a hold, its destination, a stop at an earlier
// junction, a stopped car beyond the line). A car on a LANE that crosses a junction's connector (ADR-0400's carry-overs)
// announces too, with the same flags read off the crossing's stretch (a lane has no line: it never arrives at one and
// never commits; who goes first there is the rule set's `between_junctions` and the resolver's, issue #284).
//
// HONEST CLEAR TIMES (issue #284). A car that will come to rest before its tail is past a stretch -- it holds a stop
// there for any reason (giving way inside, a red at the next close junction, a hold, no room beyond), or a stopped car
// beyond its line limits how far its front can get -- is NEVER clear of it (kNever): nobody plans to go behind it. Only
// a full stop at its own line, which the estimate already counts, does not make it so. What it would clear in if it went
// on now is announced apart (clearIfGoing): the judgement reads it for its own car, deciding whether to go.
//
// THE ESTIMATE of "when" is one shared function, from the car's own state alone (secondsToCover and its kin below):
// not stopping first, the time to cover the distance speeding up or slowing toward the speed it will hold through the
// turn; stopping first, the time to come to rest at the line plus the time from rest to the point. Estimates are guesses
// (the other car may brake); the hard margin of give_way.h is what keeps a wrong guess safe, so nothing here tries to be
// exact. One thing is not the car's own: a car queued behind another for the same line announces NO EARLIER than that
// car plus a following interval (Driving works the queues nearest the line first), so a waiting driver never sees a queue
// of five arriving at once and then a false gap.
//
// THE BOARD is rewritten whole every tick: nothing is ever withdrawn, so a car that changes its mind or goes away simply
// is not on it next tick. It is one flat array, sorted by crossing point, then arrival, then who, then path before
// anyone reads it, so it never depends on the order cars wrote to it; a crossing point's announcements are found by
// binary search. Its memory is kept from tick to tick: the steady state allocates nothing.
//
// Anything else that crosses a crossing point (people on a crosswalk, a train, the player) announces on the same board
// in the same form (design 4); the rule set's `announcers` table and the host's announcers are task E5. Cars are the only
// kind yet.
//
// Units: metres, seconds, m/s, m/s2.

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/junction_plan.h"
#include "apps/driving/road_rules.h"
#include "apps/driving/traffic_state.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace driving {

using engine::Real;

// "Never": a time that will not come (a car that will stop on a stretch is never clear of it).
constexpr Real kNever = std::numeric_limits<Real>::infinity();
// THE SLOWEST SPEED AN ESTIMATE ASSUMES through a junction (m/s): a car that holds a top speed of zero now (held at
// rest) is still assumed to cross at a crawl once it goes, a little faster than walking pace, so no estimate divides by
// zero or puts a car's crossing off for ever.
constexpr Real kSlowestThrough = 2.0;

// WHAT KIND OF THING ANNOUNCED (design 4's AnnouncerKindId). Cars only, until the people and trains of task E5.
enum class AnnouncerKind : uint8_t { Car };

// ONE ANNOUNCEMENT: "I will be on this crossing point from then to then".
struct Announcement {
    engine::CrossingId crossing;   // which crossing point (road data: ADR-0400)
    engine::PathId onPath;         // which of the crossing's two paths the announcer is on
    Real arriveIn = 0;             // seconds from now until its front reaches its stretch (0: on it or past its start)
    Real clearIn = 0;              // ... until its tail is past the stretch's end (kNever: it will come to rest before
                                   // then: HONEST CLEAR TIMES above)
    Real clearIfGoing = 0;         // ... were it to go on now, whatever stop it holds (its own judgement reads this)
    bool notGoing = false;         // it is stopping before this crossing for a reason other than this junction's traffic
                                   // (never while its body is on the stretch, or it cannot stop short of it)
    bool going = false;            // it will not stop before its line: past it, or too fast to stop there comfortably
                                   // while not giving way; not when it waits inside short of this crossing. (A car on a
                                   // lane: it cannot stop comfortably short of the stretch.)
    bool cannotStop = false;       // it cannot keep its front short of the stretch even braking as hard as it can (after
                                   // one decide interval, unless it already holds a stop there): it will be on it
    bool arrivalFixed = false;     // its arrival at its line is fixed (JunctionPlan::arrivalFixed)
    uint64_t arrivalTick = 0;      // ... at this tick ("equal to within a tick" compares these)
    bool committed = false;        // it can no longer wait short of its next crossing stretch (JunctionPlan::commitAt;
                                   // design 9.4; task D4 orders by it)
    uint64_t committedTick = 0;
    AnnouncerKind kind = AnnouncerKind::Car;
    uint64_t who = 0;              // an opaque id (a car's CarId index): for the driver card and for tie-breaks
};

// THE ANNOUNCEMENTS ON ONE CROSSING POINT (or on a run of them), a view into the board, in board order.
struct AnnouncementRange {
    const Announcement* first = nullptr;
    const Announcement* last = nullptr;
    const Announcement* begin() const { return first; }
    const Announcement* end() const { return last; }
    bool empty() const { return first == last; }
    std::size_t size() const { return static_cast<std::size_t>(last - first); }
};

// THE BOARD, rewritten every tick: an INDEXED FLAT ARRAY. Announcements are filed by crossing point (a counting sort over
// the graph's crossing ids, which are dense), each crossing point's few in order of arrival, then who, then path; an
// index of where each crossing point's announcements begin makes finding them one lookup.
class AnnouncementBoard {
public:
    // Wipes it for tick `tick`, on a graph of `crossingCount` crossing points, keeping its memory.
    void begin(uint64_t tick, std::size_t crossingCount);
    void add(const Announcement& a) { entries_.push_back(a); }
    // Files them (crossing, arrival, who, path): after this it may be read. An announcement on a crossing id the graph
    // does not have is dropped.
    void seal();

    // The announcements on crossing point `c`, in board order (empty for none).
    AnnouncementRange on(engine::CrossingId c) const;
    // Those on every crossing point with an id from `first` up to (not including) `end`: a junction's own crossing
    // points have ids next to each other (Junction::crossings).
    AnnouncementRange onCrossings(uint32_t first, uint32_t end) const;
    // The announcement by `who` on `c` from path `onPath`, or null.
    const Announcement* find(engine::CrossingId c, uint64_t who, engine::PathId onPath) const;

    uint64_t tick() const { return tick_; }
    std::size_t size() const { return entries_.size(); }
    const std::vector<Announcement>& all() const { return entries_; }
    // The most announcements one tick has held (the size its memory has grown to).
    std::size_t most() const { return most_; }

private:
    std::vector<Announcement> entries_;
    std::vector<Announcement> filed_;       // working memory for filing
    std::vector<uint32_t> firstOf_;         // per crossing id, and one more: where its announcements begin
    std::size_t crossingCount_ = 0;
    uint64_t tick_ = 0;
    std::size_t most_ = 0;
};
// One number over a board's announcements, in board order (tests compare boards built in different orders).
uint64_t boardHash(const AnnouncementBoard& board);

// ---- The shared estimate -------------------------------------------------------------------------------------------

// HOW A CAR MOVES, as its estimate reads it.
struct Mover {
    Real speed = 0;          // now, m/s
    Real speedThrough = 0;   // the speed it will hold through the junction (kSlowestThrough at least)
    Real speedUp = 0;        // comfortable speeding-up, m/s2
    Real brake = 0;          // comfortable braking, m/s2
};

// SECONDS TO COVER `distance` metres, speeding up (or slowing) toward the speed through and holding it once there. Zero
// for a distance of zero or less.
Real secondsToCover(Real distance, const Mover& m);
// SECONDS TO COME TO REST `distance` metres ahead: holding the speed until braking comfortably brings it to rest there
// (harder, at an even rate, when it is already too close; creeping up from at or near rest as secondsToCover).
Real secondsToStopAt(Real distance, const Mover& m);
// SECONDS TO COVER `distance` metres when the car must first come to rest at a line `toLine` metres ahead: the time to
// stop there, then the time from rest. A point short of the line is reached as by secondsToCover.
Real secondsToCoverStoppingFirst(Real distance, Real toLine, const Mover& m);

// THE SPEED A CAR WILL HOLD THROUGH A CONNECTOR: the lowest of its class's top speed, the connector's limit (its posted
// one, or its road class's default: RoadRules::baseLimit) and the speed its tightest bend (LaneGraph::tightestBend, as
// corner_speed holds a car through a connector) allows at `comfortableSideAccel` (the rule set's comfortable_side_accel);
// never below kSlowestThrough.
Real speedThrough(const engine::LaneGraph& roads, const RoadRules& roadRules, engine::PathId connector, Real topSpeed,
                  Real comfortableSideAccel);

// ---- What a car notes about the line, and is it going? --------------------------------------------------------------

// IS THE CAR GOING THROUGH this plan's junction without stopping at the line: its front is past the line, or it has not
// decided to give way, holds no stop at or before the line (`heldStopAt`, an odometer reading; kNever for none), and is
// too fast to stop before the line braking comfortably ("once it could no longer stop before the line at comfortable
// braking, it is going").
// With `within` seconds (the driver's decide interval), going is judged that much ahead for a driver that has decided to
// go: it will be going by the time it next decides (how other drivers see it on the board). A driver that has not
// decided yet is not going until it truly cannot stop: it must judge first.
bool isGoing(const JunctionPlan& plan, double odometer, Real speed, Real brake, double heldStopAt, Real within = 0);
// The odometer reading of the stop a car holds, or kNever.
double heldStopOf(const Car& car);
// CAN THE CAR STILL KEEP ITS FRONT SHORT OF a point `distance` metres ahead, braking as hard as it can (`hardestBrake`)?
// It reacts first for `reaction` seconds (its decide interval: what it is told now it acts on at its next decision)
// unless it already holds a stop at or short of that point, which the move step obeys every tick (`holdsStopShort`). A
// point at or behind its front (distance 0 or less) it cannot keep short of. The test behind Announcement::cannotStop and
// committing (issue #284), over act.h canStopWithin.
bool canKeepShort(Real distance, Real speed, Real hardestBrake, Real reaction, bool holdsStopShort);

// NOTES WHAT THE CAR'S OWN STATE SAYS ABOUT THE LINE of `plan`, at tick `tick` and clock `now`: it came to rest at the
// line (within the rule set's at_line_within), it is committed, and -- when its arrival is not yet fixed -- it arrives
// now, because it came to rest at the line or can no longer stop before it. Each, once noted, stays for the plan's life.
// COMMITTED: it cannot keep its front short of the plan's commitAt (its first crossing stretch beyond its last waiting
// point) braking as hard as it can (`hardestBrake`, after `reaction` seconds: canKeepShort), or its front is on it; or,
// when `redForMe` (its turn's light shows red: issue #261), past the line by more than at_line_within: inside on a red,
// it may not wait for anyone of the next green (it entered lawfully; the next green waits for it). CAME TO REST is
// not noted while `heldByLight` (its light shows red, not "red, may turn", or yellow): standing at a red is not waiting
// for this junction's traffic, so its patience, and its arrival for who goes first, start when its light turns green.
// (A car held at its line for room beyond keeps its arrival: started afresh at each grant, it was put after every car
// that came meanwhile, gave the room back to give way to them, and lost it again: issue #264.)
void noteAtTheLine(JunctionPlan& plan, const Car& car, Real brake, Real hardestBrake, Real reaction, Real atLineWithin,
                   uint64_t tick, double now, bool redForMe = false, bool heldByLight = false);
// FIXES THE CAR'S ARRIVAL at `plan`'s line, when not yet fixed: `lineIn` seconds from now (tick `tick`, clock `now`,
// ticks of `step` seconds), its own estimate when its driver first judges the junction.
void fixArrival(JunctionPlan& plan, Real lineIn, uint64_t tick, double now, Real step);

// ---- Announcing ----------------------------------------------------------------------------------------------------

// ONE CAR ANNOUNCING THE CROSSING POINTS OF ONE PLAN'S CONNECTOR: what it knows of itself (Driving gathers it).
struct PlanAnnouncer {
    CarId who;
    double odometer = 0;
    Real length = 0;               // its body, metres
    Mover move;                    // speedThrough worked out for this connector
    Real hardestBrake = 0;         // m/s2: whether it can still keep short of a crossing (cannotStop)
    double heldStop = kNever;      // the odometer reading of the stop it holds, whatever it is for (isGoing; HONEST
                                   // CLEAR TIMES: a stop before a stretch's end, but for the full stop counted by
                                   // stopsFirst, means it is never clear of it)
    Real reaction = 0;             // seconds between its decisions: it is going if it will be by its next one
    Real shift = 0;                // seconds added to every time: how much later than its own estimate the queue makes it
    bool stopsFirst = false;       // it must come to rest at the line first (it holds a full stop there: StopFor)
    bool notGoing = false;         // the whole junction: queued behind a car that is not going, or giving way elsewhere
    double heldStopAt = kNever;    // the odometer reading it holds a stop at for a reason other than this junction's
                                   // traffic (kNever: none): it is not going for any crossing beyond it
    double frontLimit = kNever;    // the odometer reading its front cannot pass (a stopped car beyond the line, less the
                                   // standstill gap; kNever: none): it is not going for a crossing beyond, and is never
                                   // clear of one it would stop on
    bool neverClear = false;       // queued for this line behind a car that holds a stop at or short of it (not a full
                                   // stop first): it comes to rest behind that car, short of every stretch's end
};

// ANNOUNCES every crossing point along `plan`'s connector whose stretch the car's tail is not yet past (crossings with
// another path only: people and trains announce for themselves). The plan's remembered state gives arrival, going and
// committed.
void announcePlan(const engine::LaneGraph& roads, const JunctionPlan& plan, const PlanAnnouncer& car,
                  AnnouncementBoard& out);

// ONE CAR ANNOUNCING THE CROSSING POINTS OF ONE LANE (connector-and-lane meetings, filed under no junction).
struct LaneAnnouncer {
    CarId who;
    double odometer = 0;
    Real length = 0;               // its body, metres
    Mover move;                    // speedThrough: the top speed it holds
    Real hardestBrake = 0;         // m/s2
    Real reaction = 0;             // seconds between its decisions
    Real lookAheadTime = 0;        // the rule set's junction_look_ahead: crossing points further off are not announced
    Real atLineWithin = 0;         // the rule set's at_line_within: its own stop to give way at a stretch is "at" it
    double heldStop = kNever;      // the odometer reading of the stop it holds, whatever it is for
    StopFor heldStopFor = StopFor::Place;   // ... and what that stop is for
};

// ANNOUNCES the crossing points along a LANE the car's body is on or its way runs along: `laneStart` is metres from the
// car's front to the lane's start (below zero when it is behind the front). Only those it will reach within the
// look-ahead time. A lane has no line: the car never arrives at one and never commits; it is GOING when it cannot stop
// short of the stretch braking comfortably, CANNOT STOP as a plan's car, NOT GOING when it holds a stop short of the
// stretch that is not its own giving way there (give_way.h: a lane car keeps out of the way of a car the resolver puts
// first by stopping with its front at the stretch), and never clear of a stretch it holds a stop before the end of.
void announceLane(const engine::LaneGraph& roads, engine::PathId lane, Real laneStart, const LaneAnnouncer& car,
                  AnnouncementBoard& out);

}  // namespace driving

#endif
