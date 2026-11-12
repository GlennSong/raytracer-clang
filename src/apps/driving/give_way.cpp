#include "apps/driving/give_way.h"

#include "apps/driving/act.h"   // kSettingGapKept, kNumberStandstillGap; canStopWithin, stoppingDistance
#include "apps/driving/road_rules.h"   // RoadRules::device: what a regulation on the road means

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace driving {

namespace {

bool knownPath(const engine::LaneGraph& roads, engine::PathId p) { return p.valid() && p.index < roads.paths.size(); }

FactNeed need(const char* name, UnitFamily family) {
    FactNeed n;
    n.name = name;
    n.family = family;
    return n;
}

// ---- Judging one junction's meetings ----------------------------------------------------------------------------

// Did I announce, on any crossing point along plan `p`'s connector, that I am not going?
bool announcedNotGoing(const GiveWayInputs& in, const JunctionPlan& p) {
    const engine::Path& conn = in.roads.paths[p.connector.index];
    for (uint32_t k = conn.crossings.first; k < conn.crossings.end() && k < in.roads.pathCrossings.size(); ++k) {
        const Announcement* mine = in.board.find(in.roads.pathCrossings[k], in.me.id.index, p.connector);
        if (mine && mine->notGoing) return true;
    }
    return false;
}

// THE GAP I HAVE to car `e` (announced on crossing point `cid`, on `otherPath`), which the resolver puts before me by
// question `by`, from my own announcement there `mine`: AHEAD of it, my tail clear before its front arrives, reading my
// clear time were I to go now (I may hold a stop I am deciding whether to drop); BEHIND it, my front arriving after its
// tail is clear, reading its HONEST clear time (announce.h: never, while it will come to rest before its tail is clear, so
// nobody goes behind a car waiting inside the junction or at its line). A body on the crossing point, a car that cannot
// stop short of it, or a car committed before me, I may only follow (conflict.h followOnly). The larger of the two.
Real gapTo(const Announcement& mine, const Announcement& e, Question by) {
    const bool follow = followOnly(by);
    const Real ahead = follow ? -kNever : e.arriveIn - mine.clearIfGoing;
    const Real behind = mine.arriveIn - e.clearIn;
    return std::max(ahead, behind);
}

// Notes car `e` in `j` when it is the tightest gap so far (or one too small), and says whether the gap is too small.
bool judgeOne(const Announcement& e, engine::CrossingId cid, engine::PathId otherPath, Question by, Real have, Real need,
              bool& tightest, GiveWayJudgement& j) {
    if (have < need || !tightest || have < j.have) {
        j.cause = GiveWayCause::Gap;
        j.other = CarId(static_cast<uint32_t>(e.who));
        j.crossing = cid;
        j.otherPath = otherPath;
        j.need = need;
        j.have = have;
        j.by = by;
        tightest = true;
    }
    return have < need;
}

// Fills `j` and says whether I must give way at plan `p`'s junction: for each crossing point along its connector, each car
// announced there that the resolver puts before me. For Go, `j` keeps the tightest gap I accepted. `hard`: my hard gap.
bool judgeMeetings(const GiveWayInputs& in, const JunctionPlan& p, Real patience, Real hard, GiveWayJudgement& j) {
    const engine::Path& conn = in.roads.paths[p.connector.index];
    const Real waited = p.cameToRest ? static_cast<Real>(std::max(0.0, in.now - p.restAt)) : Real(0);
    // Having decided to go, a driver keeps going unless the hard gap is broken; until then it needs the comfortable one.
    const Real needed = p.decision == GiveWayDecision::Go ? hard : comfortableGap(in.numbers, waited, patience, hard);
    bool tightest = false;
    for (uint32_t k = conn.crossings.first; k < conn.crossings.end() && k < in.roads.pathCrossings.size(); ++k) {
        const engine::CrossingId cid = in.roads.pathCrossings[k];
        const engine::CrossingPoint& rec = in.roads.crossings[cid.index];
        if (rec.otherSide != engine::OtherSide::Path) continue;   // people and trains: task E5
        const MeetingAnswer* answer = in.row.answer(cid, currentPhase(in.row, rec.junction, in.now));
        if (!answer) continue;
        engine::PathId otherPath;
        // A SPLIT (two turns leaving my lane): nobody gives way; the car behind follows the car ahead, along either path
        // while its body is on the split's stretch (traffic_state.h carAhead, since #283).
        if (myTurn(in.roads, *answer, cid, p.connector, &otherPath) == MyTurn::Neither) continue;
        const Announcement* mine = in.board.find(cid, in.me.id.index, p.connector);
        if (!mine) continue;   // my tail is past it, or I have not announced it
        for (const Announcement& e : in.board.on(cid)) {
            // A car NOT GOING (stopping short of this crossing point for something else) is passed over: nobody is coming.
            // Not an order between the two (conflict.h says why it is kept out of the resolver); never a body on the
            // stretch, nor a car that cannot stop short of it (announce.h).
            if (e.onPath != otherPath || e.who == in.me.id.index || e.notGoing) continue;
            // Who goes first is the resolver's; the one that goes first never looks at the other.
            Question by = Question::None;
            if (in.resolver.goesFirst(in.roads, cid, in.me.id.index, e.who, &by)) continue;
            if (judgeOne(e, cid, otherPath, by, gapTo(*mine, e, by), needed, tightest, j)) return true;
        }
    }
    return false;
}

// A LANE'S CROSSING POINTS AHEAD (issue #284): where a lane on my way meets a connector (ADR-0400's carry-overs, filed
// under no junction), I keep out of the way of a car the resolver puts before me there, as a car on a connector does. A
// lane has no line to wait at, so I am going already: the gap I need is the hard one, and if it is broken I stop with my
// front at the crossing point's stretch. Once stopped there (I hold that stop), I go on again by the comfortable gap, as
// a driver at a line decides to go: else I would stop and start with every decision. A stretch my front is on already I
// cannot keep out of (the resolver then has the other wait for my body: on_it). Says whether I must stop, the nearest
// such place (metres from my front) and why.
bool judgeLanes(const GiveWayInputs& in, Real hard, Real patience, GiveWayJudgement& j, Real& toStop) {
    if (!in.way) return false;
    const std::vector<engine::PathId>& way = in.way->paths;
    const bool wayIsMine = !way.empty() && way[0] == in.me.path;
    const std::size_t count = wayIsMine ? way.size() : 1;
    Real start = -in.me.along;   // where the k-th path of the way begins, metres ahead of my front
    for (std::size_t k = 0; k < count; ++k) {
        const engine::PathId lane = wayIsMine ? way[k] : in.me.path;
        if (!knownPath(in.roads, lane)) break;
        const engine::Path& path = in.roads.paths[lane.index];
        if (path.kind == engine::PathKind::Lane) {
            for (uint32_t x = path.crossings.first; x < path.crossings.end() && x < in.roads.pathCrossings.size(); ++x) {
                const engine::CrossingId cid = in.roads.pathCrossings[x];
                const engine::CrossingPoint& rec = in.roads.crossings[cid.index];
                if (rec.otherSide != engine::OtherSide::Path) continue;
                const MeetingAnswer* answer = in.row.answer(cid, currentPhase(in.row, rec.junction, in.now));
                engine::PathId otherPath;
                if (!answer || myTurn(in.roads, *answer, cid, lane, &otherPath) == MyTurn::Neither) continue;
                const Real toStart = start + (rec.a == lane ? rec.aFrom : rec.otherFrom);
                if (!(toStart > 0)) continue;   // my front is on it: I go first there (on_it)
                const Announcement* mine = in.board.find(cid, in.me.id.index, lane);
                if (!mine) continue;   // beyond my look-ahead: not announced, not judged yet
                const bool stoppedHere = in.me.held.hasStop && givesWay(in.me.held.stopFor) &&
                                         std::fabs(in.me.held.stopAt - (in.me.odometer + toStart)) <=
                                             in.numbers.atLineWithin;
                const Real need = stoppedHere ? comfortableGap(in.numbers, 0, patience, hard) : hard;
                bool tightest = false;
                for (const Announcement& e : in.board.on(cid)) {
                    if (e.onPath != otherPath || e.who == in.me.id.index || e.notGoing) continue;
                    Question by = Question::None;
                    if (in.resolver.goesFirst(in.roads, cid, in.me.id.index, e.who, &by)) continue;
                    if (judgeOne(e, cid, otherPath, by, gapTo(*mine, e, by), need, tightest, j)) {
                        toStop = toStart;
                        return true;
                    }
                }
            }
        }
        start += path.length;
    }
    return false;
}

// WHERE A CAR THAT CAN NO LONGER STOP AT ITS LINE WAITS INSIDE the junction of plan `p` to give way at crossing point
// `crossing`: metres along the connector. The last of its waiting points beyond its front and short of that crossing
// point's stretch (its body then stands in nobody's stretch), or else its front at the stretch itself. Below or at
// `frontAlong` (the front is already on the stretch): it cannot keep out of it.
Real insideStop(const GiveWayInputs& in, const JunctionPlan& p, engine::CrossingId crossing, Real frontAlong) {
    const engine::CrossingPoint& rec = in.roads.crossings[crossing.index];
    const Real stretch = rec.a == p.connector ? rec.aFrom : rec.otherFrom;
    Real at = stretch;
    for (std::size_t w = 1; w < p.waitingCount && w < kMaxWaitingPoints; ++w)
        if (p.waitingAt[w] > frontAlong && p.waitingAt[w] <= stretch) at = p.waitingAt[w];
    return at;
}

// ---- The facts --------------------------------------------------------------------------------------------------

void setYes(NoticedFact& out, bool yes) {
    out.present = true;
    out.yes = yes;
}

const GiveWayVerdict* verdictOf(const DriverView& v) {
    return v.giveWay && v.giveWay->judged ? v.giveWay : nullptr;
}

void about(NoticedFact& out, const GiveWayVerdict& g) {
    if (g.why.other.valid()) out.aboutCar = g.why.other;
    if (g.why.cause == GiveWayCause::Gap) {
        out.hasSeconds = true;
        out.seconds = g.why.have;
    }
    out.hasMetres = true;
    out.metres = g.toLine;
}

void noticeMustGiveWay(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const GiveWayVerdict* g = verdictOf(v);
    if (!g) return;
    setYes(out, g->giveWay);
    if (g->giveWay) about(out, *g);
}
void noticeGiveWayLine(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const GiveWayVerdict* g = verdictOf(v);
    if (!g || !g->giveWay) return;
    out.present = true;
    out.number = g->toLine;
    about(out, *g);
    // Waiting for a body already on the crossing point: a matter of space, as behind a car (wait_graph.h), not of right of
    // way. A ring of such waits is gridlock, never an order to go (issue #284).
    out.forABody = g->why.cause == GiveWayCause::Gap && g->why.by == Question::OnIt;
}
void noticeGiveWayGap(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const GiveWayVerdict* g = verdictOf(v);
    if (!g || !g->giveWay || g->why.cause != GiveWayCause::Gap) return;
    out.present = true;
    out.number = g->why.have;
    about(out, *g);
}
void noticeGiveWayGapNeeded(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const GiveWayVerdict* g = verdictOf(v);
    if (!g || !g->giveWay || g->why.cause != GiveWayCause::Gap) return;
    out.present = true;
    out.number = g->why.need;
    about(out, *g);
}
void noticeCommitted(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const GiveWayVerdict* g = verdictOf(v);
    if (g) setYes(out, g->committed);
}
void noticeStoppedAtLine(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const GiveWayVerdict* g = verdictOf(v);
    if (g && g->hasNextLine) setYes(out, g->stoppedAtLine);
}
void noticeNextLine(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const GiveWayVerdict* g = verdictOf(v);
    if (!g || !g->hasNextLine) return;
    out.present = true;
    out.number = g->toNextLine;
    out.hasMetres = true;
    out.metres = g->toNextLine;
}
void noticeEveryoneStops(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const GiveWayVerdict* g = verdictOf(v);
    if (g && g->hasNextLine) setYes(out, g->everyoneStops);
}
void noticeSignRank(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const GiveWayVerdict* g = verdictOf(v);
    if (!g || !g->hasNextLine || g->signRank < 0 || static_cast<std::size_t>(g->signRank) >= v.rules.ranks.size()) return;
    out.present = true;
    out.word = static_cast<WordIndex>(g->signRank);
}
void noticeLight(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const GiveWayVerdict* g = verdictOf(v);
    if (!g || !g->hasLight || lampWordIndex(g->lamp) == kNoWord) return;
    out.present = true;
    out.word = lampWordIndex(g->lamp);
    out.hasMetres = true;
    out.metres = g->toLightLine;
}
void noticeLightLine(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const GiveWayVerdict* g = verdictOf(v);
    if (!g || !g->hasLight) return;
    out.present = true;
    out.number = g->toLightLine;
    out.hasMetres = true;
    out.metres = g->toLightLine;
}
void noticeCanStopForLight(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    const GiveWayVerdict* g = verdictOf(v);
    if (!g || !g->hasLight) return;
    setYes(out, g->canStopForLight);
    out.hasMetres = true;
    out.metres = g->toLightLine;
}

FactSpec spec(const char* name, FactKind kind, UnitFamily unit, NoticeFn fn, const char* meaning) {
    FactSpec s;
    s.name = name;
    s.kind = kind;
    s.unit = unit;
    s.notice = fn;
    s.meaning = meaning;
    return s;
}

}  // namespace

bool stopIsJunctionsOwn(const Car& c, const JunctionPlan& p, Real atLineWithin) {
    if (!c.held.hasStop) return false;
    if (c.held.stopFor == StopFor::FullStop) return std::fabs(c.held.stopAt - p.startOdometer) <= atLineWithin;
    // Giving way where the judgement put it: the line, or inside the junction (stopAlong).
    return givesWay(c.held.stopFor) && p.decision == GiveWayDecision::GiveWay &&
           p.judged.cause == GiveWayCause::Gap &&
           std::fabs(c.held.stopAt - (p.startOdometer + p.judged.stopAlong)) <= atLineWithin;
}

bool stopsFirstAt(const Car& c, const JunctionPlan& p, Real atLineWithin) {
    return c.held.hasStop && c.held.stopFor == StopFor::FullStop && !p.cameToRest &&
           std::fabs(c.held.stopAt - p.startOdometer) <= atLineWithin;
}

const ControlKind* junctionControl(const RightOfWay& row, const RuleSet& rules, engine::JunctionId junction) {
    if (!junction.valid() || junction.index >= row.junctions.size()) return nullptr;
    const uint32_t c = row.junctions[junction.index].control;
    return c < rules.controls.size() ? &rules.controls[c] : nullptr;
}

bool stopLineAtEnd(const engine::LaneGraph& roads, const RuleSet& rules, const RoadRules& roadRules, engine::PathId lane,
                   Real atLineWithin) {
    if (!knownPath(roads, lane)) return false;
    const engine::Path& l = roads.paths[lane.index];
    for (uint32_t r = l.regulations.first; r < l.regulations.end() && r < roads.regulations.size(); ++r) {
        const Device* d = roadRules.device(rules, roads.regulations[r].kind);
        if (d && d->stopLine && roads.regulations[r].from >= l.length - atLineWithin) return true;
    }
    return false;
}

bool stopsEveryoneFirst(const engine::LaneGraph& roads, const RightOfWay& row, const RuleSet& rules,
                        const RoadRules& roadRules, engine::PathId connector, Real atLineWithin) {
    if (!knownPath(roads, connector)) return false;
    const ControlKind* control = junctionControl(row, rules, roads.paths[connector.index].junction);
    return (control && control->everyoneStops) ||
           stopLineAtEnd(roads, rules, roadRules, roads.paths[connector.index].from, atLineWithin);
}

Real hardGap(const JunctionNumbers& n, Real speed, Real hardestBrake, Real reaction) {
    // The seconds of travel at my speed now that my stopping distance takes; a car at rest needs only its reaction.
    const Real v = std::max(Real(0), speed);
    const Real stopping = v > 0 ? stoppingDistance(v, hardestBrake, reaction) / v : std::max(Real(0), reaction);
    return std::max(n.gapHard, stopping);
}

Real comfortableGap(const JunctionNumbers& n, Real waited, Real patience, Real hard) {
    const Real floor = std::max(n.gapFloor, std::max(n.gapHard, hard));
    const Real shrink = patience > 0 ? n.gapShrink / patience : Real(0);
    return std::max(floor, n.gapComfortable - std::max(Real(0), waited) * shrink);
}

void judgeGiveWay(const GiveWayInputs& in, GiveWayVerdict& out) {
    out = GiveWayVerdict();
    out.judged = true;
    const JunctionPlans& plans = in.plans;
    const Car& me = in.me;
    const Real brake = in.brake;
    const double heldStop = heldStopOf(me);
    if (plans.count > 0) out.committed = plans.plans[0].committed;
    // The nearest junction whose line is still ahead: its line, whether everyone stops there, the sign on my approach.
    for (std::size_t i = 0; i < plans.count; ++i) {
        const JunctionPlan& p = plans.plans[i];
        if (!(p.startOdometer > me.odometer) || !knownPath(in.roads, p.connector)) continue;
        out.hasNextLine = true;
        out.toNextLine = static_cast<Real>(p.startOdometer - me.odometer);
        out.stoppedAtLine = p.cameToRest;
        const ControlKind* control = junctionControl(in.row, in.rules, p.junction);
        out.everyoneStops = control && control->everyoneStops;
        const uint32_t approach = p.connector.index < in.row.paths.size() ? in.row.paths[p.connector.index].approach
                                                                          : kNoApproach;
        out.signRank = approach < in.row.approaches.size() ? in.row.approaches[approach].signRank : 0;
        break;
    }
    // The lights ahead whose line my front has not passed (by more than at_line_within): the nearest that is not green for
    // me (a green at a near junction must not hide a red just beyond it), else the nearest; what my turn shows there,
    // where I stop for it, and whether I can (issue #261).
    std::size_t lit = kMaxJunctionPlans;
    Lamp litLamp = Lamp::Dark;
    for (std::size_t i = 0; in.lights && i < plans.count && i < kMaxJunctionPlans; ++i) {
        const JunctionPlan& p = plans.plans[i];
        if (!(p.startOdometer > me.odometer - in.numbers.atLineWithin) || !knownPath(in.roads, p.connector)) continue;
        const Lamp lamp = lampAt(in.roads, in.row, p.connector, in.now);
        if (lamp == Lamp::Dark) continue;
        // Committed through to these lights from the junction before (room.h commitThrough: no place to wait between
        // the two), a yellow here is one it cannot stop for without its tail in the box behind: it goes on, as the
        // dilemma lets a car that cannot stop. A red still stops it.
        if (p.committed && lamp == Lamp::Yellow) continue;
        if (lit == kMaxJunctionPlans || litLamp == Lamp::Green) {
            lit = i;
            litLamp = lamp;
        }
        if (lamp != Lamp::Green) break;
    }
    if (lit < kMaxJunctionPlans) {
        // Where to stop: this light's own line. A light at the next junction whose line is nearer the junction before than
        // my body will need is not this fact's: room beyond (room.h, issue #264) grants entry to the junction before only
        // when that light will be green as I get there, so I never stop at its line with my tail in the box behind.
        const double line = plans.plans[lit].startOdometer;
        out.hasLight = true;
        out.lamp = litLamp;
        out.toLightLine = static_cast<Real>(line - me.odometer);
        // THE DILEMMA. Holding what I decide now for one decide interval, then braking at the deceleration the yellow is
        // timed for. Once I hold a stop at that line I keep it while my hardest braking still can make it: a driver who
        // has begun to stop does not change its mind and run on as it nears the line (a class that brakes more gently
        // than the yellow assumes would otherwise find itself "unable to stop" a car's length short).
        const Real reach = out.toLightLine + in.numbers.atLineWithin;
        // (Whatever rule's stop at that line won: giving way there as well stops it at the same place.)
        const bool holding = me.held.hasStop && std::fabs(me.held.stopAt - line) <= in.numbers.atLineWithin;
        out.canStopForLight = canStopWithin(reach, me.speed, in.lights->deceleration, in.reaction) ||
                              (holding && canStopWithin(reach, me.speed, in.hardestBrake));
    }
    const Real patience = static_cast<Real>(personalValue(in.numbers.patience, in.numbers.patienceHash, in.person));
    // MY HARD GAP (issue #284): never below my own stopping time, so a gap I keep going on is one I can still stop short in
    // when it closes.
    const Real hard = hardGap(in.numbers, me.speed, in.hardestBrake, in.reaction);

    // THE LANES AHEAD (issue #284): a crossing point on a lane where a car the resolver puts first leaves me too small a
    // gap. Nothing beyond such a stop is decided this time: I stop there first.
    GiveWayJudgement laneWhy;
    Real laneToStop = kNever;
    const bool laneStop = judgeLanes(in, hard, patience, laneWhy, laneToStop);

    for (std::size_t i = 0; i < plans.count && i < kMaxJunctionPlans; ++i) {
        const JunctionPlan& p = plans.plans[i];
        if (!knownPath(in.roads, p.connector)) continue;
        // Too far to have announced it: nothing here or beyond is judged yet.
        if (!(in.lineIn[i] <= in.numbers.lookAheadTime)) break;
        // Beyond a stop on a lane I must make first: not judged yet.
        if (laneStop && p.startOdometer - me.odometer > static_cast<double>(laneToStop)) break;
        // Past the line, or no longer able to stop before it braking comfortably: GOING. It no longer stops at the line,
        // and has passed every gate below; but it still keeps out of the way of a car the resolver puts before it (a body
        // on a crossing point, a car committed before it), waiting inside the junction (task D4).
        const bool going = isGoing(p, me.odometer, me.speed, brake, heldStop);
        const double toLine = p.startOdometer - me.odometer;
        // At a RED (not "red, may turn") a driver that can still stop decides nothing there or beyond: the light holds it,
        // and what it decides is for its green (issue #261; noteAtTheLine starts the approach afresh).
        if (!going && in.lights && lampAt(in.roads, in.row, p.connector, in.now) == Lamp::Red) break;
        bool notGoing = false, owesStop = false;
        if (!going) {
            // THE GATES: when a clear road counts as a decision to go (the top of give_way.h).
            // A driver that has not yet decided decides only once it nears the point where it could no longer stop at
            // the line: its comfortable stopping distance, one decision's travel, and the line's own reach. Deciding
            // further out (in a queue, say) would be deciding on guesses that change before it gets there.
            if (p.decision == GiveWayDecision::Undecided &&
                canStopWithin(static_cast<Real>(toLine) - in.numbers.atLineWithin, me.speed, brake, in.reaction))
                break;
            // Not going there, as it announced (held, a stopped car beyond its line it cannot pass, a queue that is not
            // going): others do not wait for it. It still judges, and stops at the line if it must give way (it may be
            // let go this very decision), but a clear road is no decision to go: it has not decided anything there yet.
            // One exception: a car whose only stop at this line is its own wait for room beyond (room.h) judges and
            // decides as ever, since deciding to go is when it asks for room again (it announced "not going" so that cross
            // traffic would use the time).
            // (Refused room here at its last ask: whichever rule's stop at the line won, a wait for room or a give-way.)
            const bool waitingForRoomHere =
                p.room.state == RoomState::Refused ||
                (me.held.hasStop && me.held.stopFor == StopFor::NoRoom &&
                 std::fabs(me.held.stopAt - p.startOdometer) <= in.numbers.atLineWithin);
            notGoing = !waitingForRoomHere && announcedNotGoing(in, p);
            // Still owing a full stop at this line (it holds one there, the junction's control makes everyone stop, or the
            // lane it arrives by carries a device with a stop line at its end) and not yet at rest there: held there as
            // surely, so a clear road is no decision to go either. It decides when it has stopped; deciding before (issue
            // #240: the lab's all-way stops and stop signs) turned every driver that met a car arriving first into "go",
            // then "give way", then "go".
            owesStop = stopsFirstAt(me, p, in.numbers.atLineWithin) ||
                       (!p.cameToRest && stopsEveryoneFirst(in.roads, in.row, in.rules, in.roadRules, p.connector,
                                                            in.numbers.atLineWithin));
        }

        GiveWayJudgement j;
        bool give = judgeMeetings(in, p, patience, hard, j);
        if (give && going) {
            // Inside the junction, short of the crossing point. A front already on its stretch cannot keep out of it. The
            // resolver puts such a car second only when the other's body is on a crossing point they share as well (a car
            // whose front is on a stretch is on it, and cannot stop short of it, which questions 1 and 2 put first): both
            // bodies are there already, and it goes on, there being nowhere short to stop. (A car not yet on the stretch
            // that cannot keep short of it and is put second is counted: ConflictCounts::secondCannotStop.)
            const Real frontAlong = static_cast<Real>(me.odometer - p.startOdometer);
            j.stopAlong = insideStop(in, p, j.crossing, frontAlong);
            if (!(j.stopAlong > frontAlong)) {
                give = false;
                j.stopAlong = 0;
            }
        }
        if (give && !going && !notGoing && !owesStop && p.waitInsideAt > 0 && j.crossing.valid()) {
            // WAITING INSIDE FOR A FAR TURN (issue #261): on its own green, a car that must give way at a crossing point at
            // or beyond its place to wait inside pulls forward to that place instead of stopping at the line. Every
            // crossing point before it was judged clear above (they come first along the connector); the check at load made
            // sure its body there is in the way of nothing green with it. Never on a yellow or a red, nor on an arrow
            // (nothing it gives way to moves then).
            const engine::CrossingPoint& rec = in.roads.crossings[j.crossing.index];
            const Real stretch = rec.a == p.connector ? rec.aFrom : rec.otherFrom;
            if (stretch >= p.waitInsideAt && lampAt(in.roads, in.row, p.connector, in.now) == Lamp::Green)
                j.stopAlong = p.waitInsideAt;
        }
        if (!give) {
            out.clear[i] = !owesStop;
            if (notGoing || owesStop) break;
            out.decision[i] = GiveWayDecision::Go;
            out.judgement[i] = j;
            continue;
        }
        out.decision[i] = GiveWayDecision::GiveWay;
        out.judgement[i] = j;

        // Where to stop: inside (going, or waiting inside on a green), this line, or the line before when my body would
        // not fit between the two junctions.
        std::size_t at = i;
        if (!going && !(j.stopAlong > 0) && i > 0) {
            const JunctionPlan& before = plans.plans[i - 1];
            if (knownPath(in.roads, before.connector) && !isGoing(before, me.odometer, me.speed, brake, heldStop)) {
                const double beforeEnds = before.startOdometer + in.roads.paths[before.connector.index].length;
                if (p.startOdometer - beforeEnds < in.length + in.numbers.standstillGap) {
                    at = i - 1;
                    out.decision[at] = GiveWayDecision::GiveWay;
                    out.judgement[at] = j;
                    out.judgement[at].cause = GiveWayCause::NextJunction;
                }
            }
        }
        out.giveWay = true;
        out.stopPlan = static_cast<uint8_t>(at);
        out.toLine = static_cast<Real>(plans.plans[at].startOdometer + out.judgement[at].stopAlong - me.odometer);
        out.why = out.judgement[at];
        break;
    }
    // A stop on a lane nearer than any junction's: that is where I stop (no plan holds it).
    if (laneStop && (!out.giveWay || laneToStop < out.toLine)) {
        out.giveWay = true;
        out.onALane = true;
        out.stopPlan = static_cast<uint8_t>(kMaxJunctionPlans);
        out.toLine = laneToStop;
        out.why = laneWhy;
    }
}

// ---- Registering -----------------------------------------------------------------------------------------------------

bool registerJunctionFacts(FactRegistry& registry, const std::vector<std::string>& ranks,
                           std::vector<std::string>* problems) {
    std::vector<FactSpec> specs;
    specs.push_back(spec(kFactMustGiveWay, FactKind::YesNo, UnitFamily::Ratio, noticeMustGiveWay,
                         "I must give way at a junction ahead (about the car I give way to, the gap I have, and how far "
                         "my front is from where I stop)"));
    FactSpec line = spec(kFactGiveWayLine, FactKind::Place, UnitFamily::Length, noticeGiveWayLine,
                         "where I stop to give way, metres ahead: the junction's line, or once I can no longer stop "
                         "there, inside the junction short of the crossing point (about the car I give way to)");
    line.stopFor = StopFor::GivingWay;
    specs.push_back(line);
    specs.push_back(spec(kFactGiveWayGap, FactKind::Number, UnitFamily::Time, noticeGiveWayGap,
                         "the gap I have at the crossing point where I give way, seconds"));
    specs.push_back(spec(kFactGiveWayGapNeeded, FactKind::Number, UnitFamily::Time, noticeGiveWayGapNeeded,
                         "the gap I need there, seconds (shrunk by how long I have waited)"));
    specs.push_back(spec(kFactCommitted, FactKind::YesNo, UnitFamily::Ratio, noticeCommitted,
                         "my front is past the last place I could wait at the nearest junction"));
    specs.push_back(spec(kFactStoppedAtLine, FactKind::YesNo, UnitFamily::Ratio, noticeStoppedAtLine,
                         "I came to rest at the line of the next junction"));
    FactSpec next = spec(kFactNextLine, FactKind::Place, UnitFamily::Length, noticeNextLine,
                         "the line of the next junction, as a place to come to a full stop at before going on");
    next.stopFor = StopFor::FullStop;
    specs.push_back(next);
    specs.push_back(spec(kFactEveryoneStops, FactKind::YesNo, UnitFamily::Ratio, noticeEveryoneStops,
                         "the next junction's control makes everyone come to rest at the line first"));
    FactSpec rank = spec(kFactSignRank, FactKind::Word, UnitFamily::Ratio, noticeSignRank,
                         "the rank of the sign on my approach to the next junction (no sign: the first rank)");
    rank.words = ranks;
    specs.push_back(rank);
    // The lights (issue #261): absent wherever no light runs a programme.
    FactSpec light = spec(kFactLight, FactKind::Word, UnitFamily::Ratio, noticeLight,
                          "what my turn's light shows at the nearest lights ahead whose line I have not passed (about how "
                          "far its line is)");
    light.words = lampWords();
    specs.push_back(light);
    specs.push_back(spec(kFactLightLine, FactKind::Place, UnitFamily::Length, noticeLightLine,
                         "where I stop for those lights: their line"));
    specs.push_back(spec(kFactCanStopForLight, FactKind::YesNo, UnitFamily::Ratio, noticeCanStopForLight,
                         "I can stop there, holding what I decide now until I next decide and then braking at the "
                         "deceleration the yellow is timed for (no: I can reach the line before the red)"));
    bool ok = true;
    for (FactSpec& s : specs) {
        std::string why;
        if (registry.add(std::move(s), &why) == kNoFact) {
            ok = false;
            if (problems) problems->push_back(why);
        }
    }
    ReaderSpec reader;
    reader.name = kJunctionReaderName;
    reader.meaning = "announces each driven car at the junctions ahead, every tick, and judges the gaps it gives way by";
    reader.numbers.push_back(need(kNumberJunctionLookAhead, UnitFamily::Time));
    reader.numbers.push_back(need(kNumberGapComfortable, UnitFamily::Time));
    reader.numbers.push_back(need(kNumberGapHard, UnitFamily::Time));
    reader.numbers.push_back(need(kNumberGapFloor, UnitFamily::Time));
    reader.numbers.push_back(need(kNumberGapShrink, UnitFamily::Ratio));
    reader.numbers.push_back(need(kNumberAtLineWithin, UnitFamily::Length));
    reader.numbers.push_back(need(kNumberComfortableSideAccel, UnitFamily::Acceleration));
    reader.numbers.push_back(need(kNumberStandstillGap, UnitFamily::Length));
    reader.settings.push_back(need(kSettingPatience, UnitFamily::Ratio));
    reader.settings.push_back(need(kSettingGapKept, UnitFamily::Time));
    std::string why;
    if (!registry.addReader(reader, &why)) {
        ok = false;
        if (problems) problems->push_back(why);
    }
    return ok;
}

bool hasJunctionStep(const FactRegistry& registry) { return registry.findReader(kJunctionReaderName) != nullptr; }

bool readJunctionNumbers(const RuleSet& rules, JunctionNumbers& out, std::string* why) {
    const std::pair<const char*, Real*> numbers[] = {
        {kNumberJunctionLookAhead, &out.lookAheadTime}, {kNumberGapComfortable, &out.gapComfortable},
        {kNumberGapHard, &out.gapHard},                 {kNumberGapFloor, &out.gapFloor},
        {kNumberGapShrink, &out.gapShrink},             {kNumberAtLineWithin, &out.atLineWithin},
        {kNumberComfortableSideAccel, &out.comfortableSideAccel}, {kNumberStandstillGap, &out.standstillGap},
    };
    for (const std::pair<const char*, Real*>& n : numbers) {
        const Number* found = rules.findNumber(n.first);
        if (!found) {
            if (why) *why = "rule set \"" + rules.name + "\" lacks the number \"" + n.first + "\", which " + kJunctionReaderName + " reads";
            return false;
        }
        *n.second = static_cast<Real>(found->value);
    }
    const PersonalSetting* patience = rules.findPersonal(kSettingPatience);
    const PersonalSetting* gapKept = rules.findPersonal(kSettingGapKept);
    if (!patience || !gapKept) {
        if (why)
            *why = "rule set \"" + rules.name + "\" lacks the setting \"" + kSettingPatience + "\" or \"" + kSettingGapKept +
                   "\", which " + kJunctionReaderName + " reads";
        return false;
    }
    out.patience = *patience;
    out.patienceHash = settingNameHash(kSettingPatience);
    out.gapKept = *gapKept;
    out.gapKeptHash = settingNameHash(kSettingGapKept);
    return true;
}

}  // namespace driving
