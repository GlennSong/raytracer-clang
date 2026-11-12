#include "apps/driving/give_way.h"

#include "apps/driving/act.h"   // kSettingGapKept, kNumberStandstillGap
#include "apps/driving/road_rules.h"   // RoadRules::device: what a regulation on the road means

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace driving {

namespace {

constexpr Real kTwo = 2;   // the 2 in v^2 = 2 a s

bool knownPath(const engine::LaneGraph& roads, engine::PathId p) { return p.valid() && p.index < roads.paths.size(); }

FactNeed need(const char* name, UnitFamily family) {
    FactNeed n;
    n.name = name;
    n.family = family;
    return n;
}

// ---- Judging one junction's meetings ----------------------------------------------------------------------------

// Does the lane I arrive at plan `p`'s junction by carry, at its end, a device with a stop line (a stop sign)?
bool stopLineAtEnd(const GiveWayInputs& in, const JunctionPlan& p) {
    const engine::PathId lane = in.roads.paths[p.connector.index].from;
    if (!knownPath(in.roads, lane)) return false;
    const engine::Path& l = in.roads.paths[lane.index];
    for (uint32_t r = l.regulations.first; r < l.regulations.end() && r < in.roads.regulations.size(); ++r) {
        const Device* d = in.roadRules.device(in.rules, in.roads.regulations[r].kind);
        if (d && d->stopLine && in.roads.regulations[r].from >= l.length - in.numbers.atLineWithin) return true;
    }
    return false;
}

// Did I announce, on any crossing point along plan `p`'s connector, that I am not going?
bool announcedNotGoing(const GiveWayInputs& in, const JunctionPlan& p) {
    const engine::Path& conn = in.roads.paths[p.connector.index];
    for (uint32_t k = conn.crossings.first; k < conn.crossings.end() && k < in.roads.pathCrossings.size(); ++k) {
        const Announcement* mine = in.board.find(in.roads.pathCrossings[k], in.me.id.index, p.connector);
        if (mine && mine->notGoing) return true;
    }
    return false;
}

// Fills `j` and says whether I must give way at plan `p`'s junction: for each crossing point along its connector, each car
// announced there that the resolver puts before me. For Go, `j` keeps the tightest gap I accepted.
bool judgeMeetings(const GiveWayInputs& in, const JunctionPlan& p, Real patience, GiveWayJudgement& j) {
    const engine::Path& conn = in.roads.paths[p.connector.index];
    const Real waited = p.cameToRest ? static_cast<Real>(std::max(0.0, in.now - p.restAt)) : Real(0);
    // Having decided to go, a driver keeps going unless the hard gap is broken; until then it needs the comfortable one.
    const Real needed = p.decision == GiveWayDecision::Go ? in.numbers.gapHard : comfortableGap(in.numbers, waited, patience);
    bool tightest = false;
    for (uint32_t k = conn.crossings.first; k < conn.crossings.end() && k < in.roads.pathCrossings.size(); ++k) {
        const engine::CrossingId cid = in.roads.pathCrossings[k];
        const engine::CrossingPoint& rec = in.roads.crossings[cid.index];
        if (rec.otherSide != engine::OtherSide::Path) continue;   // people and trains: task E5
        const MeetingAnswer* answer = in.row.answer(cid, currentPhase(in.row, rec.junction));
        if (!answer) continue;
        engine::PathId otherPath;
        if (myTurn(in.roads, *answer, cid, p.connector, &otherPath) == MyTurn::Neither) continue;   // a split: follow
        const Announcement* mine = in.board.find(cid, in.me.id.index, p.connector);
        if (!mine) continue;   // my tail is past it, or I have not announced it
        for (const Announcement& e : in.board.on(cid)) {
            if (e.onPath != otherPath || e.who == in.me.id.index || e.notGoing) continue;
            // Who goes first is the resolver's; the one that goes first never looks at the other.
            Question by = Question::None;
            if (in.resolver.goesFirst(in.roads, cid, in.me.id.index, e.who, &by)) continue;
            // Ahead of it: my tail clear before its front arrives. Behind it: my front arriving after its tail is clear --
            // never behind a car WAITING INSIDE the junction for somebody else (task D4): it may not go when its estimate
            // says, and once I had committed "behind" it, it would have to wait for me, inside the box, for as long as
            // cars kept doing so. A body on the crossing point, or a car committed before me, I may only follow.
            const bool follow = followOnly(by);
            const Real need = needed;
            const Real ahead = follow ? -kNever : e.arriveIn - mine->clearIn;
            const Real behind = follow || !e.waitsInside ? mine->arriveIn - e.clearIn : -kNever;
            const Real have = std::max(ahead, behind);
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
            if (have < need) return true;
        }
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
    return c.held.stopFor == StopFor::GivingWay && p.decision == GiveWayDecision::GiveWay &&
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

Real comfortableGap(const JunctionNumbers& n, Real waited, Real patience) {
    const Real floor = std::max(n.gapFloor, n.gapHard);
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
    const Real patience = static_cast<Real>(personalValue(in.numbers.patience, in.numbers.patienceHash, in.person));

    for (std::size_t i = 0; i < plans.count && i < kMaxJunctionPlans; ++i) {
        const JunctionPlan& p = plans.plans[i];
        if (!knownPath(in.roads, p.connector)) continue;
        // Too far to have announced it: nothing here or beyond is judged yet.
        if (!(in.lineIn[i] <= in.numbers.lookAheadTime)) break;
        // Past the line, or no longer able to stop before it braking comfortably: GOING. It no longer stops at the line,
        // and has passed every gate below; but it still keeps out of the way of a car the resolver puts before it (a body
        // on a crossing point, a car committed before it), waiting inside the junction (task D4).
        const bool going = isGoing(p, me.odometer, me.speed, brake, heldStop);
        const double toLine = p.startOdometer - me.odometer;
        bool notGoing = false, owesStop = false;
        if (!going) {
            // THE GATES: when a clear road counts as a decision to go (the top of give_way.h).
            // A driver that has not yet decided decides only once it nears the point where it could no longer stop at
            // the line: its comfortable stopping distance, one decision's travel, and the line's own reach. Deciding
            // further out (in a queue, say) would be deciding on guesses that change before it gets there.
            const Real decideWithin = (brake > 0 ? me.speed * me.speed / (kTwo * brake) : Real(0)) +
                                      me.speed * in.reaction + in.numbers.atLineWithin;
            if (p.decision == GiveWayDecision::Undecided && toLine > decideWithin) break;
            // Not going there, as it announced (held, a stopped car beyond its line it cannot pass, a queue that is not
            // going): others do not wait for it. It still judges, and stops at the line if it must give way (it may be
            // let go this very decision), but a clear road is no decision to go: it has not decided anything there yet.
            notGoing = announcedNotGoing(in, p);
            // Still owing a full stop at this line (it holds one there, the junction's control makes everyone stop, or the
            // lane it arrives by carries a device with a stop line at its end) and not yet at rest there: held there as
            // surely, so a clear road is no decision to go either. It decides when it has stopped; deciding before (issue
            // #240: the lab's all-way stops and stop signs) turned every driver that met a car arriving first into "go",
            // then "give way", then "go".
            const ControlKind* control = junctionControl(in.row, in.rules, p.junction);
            owesStop = stopsFirstAt(me, p, in.numbers.atLineWithin) ||
                       (!p.cameToRest && ((control && control->everyoneStops) || stopLineAtEnd(in, p)));
        }

        GiveWayJudgement j;
        bool give = judgeMeetings(in, p, patience, j);
        if (give && going) {
            // Inside the junction, short of the crossing point. A front already on its stretch cannot keep out of it:
            // it goes on, and the resolver has the other wait for its body (on_it) from the next tick.
            const Real frontAlong = static_cast<Real>(me.odometer - p.startOdometer);
            j.stopAlong = insideStop(in, p, j.crossing, frontAlong);
            if (!(j.stopAlong > frontAlong)) {
                give = false;
                j.stopAlong = 0;
            }
        }
        if (!give) {
            if (notGoing || owesStop) break;
            out.decision[i] = GiveWayDecision::Go;
            out.judgement[i] = j;
            continue;
        }
        out.decision[i] = GiveWayDecision::GiveWay;
        out.judgement[i] = j;

        // Where to stop: inside (going), this line, or the line before when my body would not fit between the two
        // junctions.
        std::size_t at = i;
        if (!going && i > 0) {
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
