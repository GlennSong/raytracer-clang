#include "apps/driving/give_way.h"

#include "apps/driving/act.h"   // kSettingGapKept, kNumberStandstillGap

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

// ---- The ring of fall-backs (rule 4) --------------------------------------------------------------------------------

// At a meeting of paths `mine` and `other` filed under `junction`, both cars arriving at their lines in the same tick:
// do I give way? By the table where it settles the pair; by its fall-back where it is asked while driving.
bool givesWayAtATie(const engine::LaneGraph& roads, const RightOfWay& row, engine::PathId mine, engine::PathId other,
                    engine::JunctionId junction) {
    if (!knownPath(roads, mine)) return false;
    const engine::Path& p = roads.paths[mine.index];
    for (uint32_t k = p.crossings.first; k < p.crossings.end() && k < roads.pathCrossings.size(); ++k) {
        const engine::CrossingId cid = roads.pathCrossings[k];
        const engine::CrossingPoint& rec = roads.crossings[cid.index];
        if (rec.otherSide != engine::OtherSide::Path || rec.junction != junction) continue;
        const bool iAmA = rec.a == mine;
        if ((iAmA ? rec.otherPath() : rec.a) != other) continue;
        const MeetingAnswer* answer = row.answer(cid, currentPhase(row, junction));
        if (!answer) continue;
        switch (myTurn(roads, *answer, cid, mine)) {
        case MyTurn::GiveWay: return true;
        case MyTurn::Ask:
            if ((answer->fallback == GivesWay::A) == iAmA) return true;
            break;
        case MyTurn::GoFirst:
        case MyTurn::Neither: break;
        }
    }
    return false;
}

// Do `me` and `them` stand in one ring of "gives way to" among the cars arriving at `junction`'s lines in tick `tick`?
bool inRing(const engine::LaneGraph& roads, const RightOfWay& row, const AnnouncementBoard& board,
            engine::JunctionId junction, uint64_t tick, const Announcement& me, const Announcement& them,
            uint32_t* tieGroupsCut) {
    if (!junction.valid() || junction.index >= roads.junctions.size()) return false;
    const engine::Junction& jn = roads.junctions[junction.index];
    std::array<uint64_t, kMaxTieGroup> who{};
    std::array<engine::PathId, kMaxTieGroup> path{};
    std::size_t n = 0;
    bool cut = false;
    for (const Announcement& a : board.onCrossings(jn.crossings.first, jn.crossings.end())) {
        if (!a.arrivalFixed || a.going || a.arrivalTick != tick) continue;
        bool seen = false;
        for (std::size_t m = 0; m < n && !seen; ++m) seen = who[m] == a.who && path[m] == a.onPath;
        if (seen) continue;
        if (n == kMaxTieGroup) {
            cut = true;
            continue;
        }
        who[n] = a.who;
        path[n] = a.onPath;
        ++n;
    }
    if (cut && tieGroupsCut) ++*tieGroupsCut;
    std::size_t i = n, j = n;
    for (std::size_t m = 0; m < n; ++m) {
        if (who[m] == me.who && path[m] == me.onPath) i = m;
        if (who[m] == them.who && path[m] == them.onPath) j = m;
    }
    if (i == n || j == n) return false;
    // Who gives way to whom among them, then everyone reachable along those arrows (Floyd-Warshall on a few cars).
    bool reach[kMaxTieGroup][kMaxTieGroup] = {};
    for (std::size_t a = 0; a < n; ++a)
        for (std::size_t b = 0; b < n; ++b)
            reach[a][b] = a != b && givesWayAtATie(roads, row, path[a], path[b], junction);
    for (std::size_t k = 0; k < n; ++k)
        for (std::size_t a = 0; a < n; ++a)
            for (std::size_t b = 0; b < n; ++b) reach[a][b] = reach[a][b] || (reach[a][k] && reach[k][b]);
    return reach[i][j] && reach[j][i];
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

// Fills `j` and says whether I must give way at plan `p`'s junction: for each crossing point along its connector, each car
// announced there that I give way to. For Go, `j` keeps the tightest gap I accepted.
bool judgeMeetings(const GiveWayInputs& in, const JunctionPlan& p, Real patience, GiveWayJudgement& j,
                   uint32_t& tieGroupsCut) {
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
        const MyTurn turn = myTurn(in.roads, *answer, cid, p.connector, &otherPath);
        if (turn == MyTurn::Neither || turn == MyTurn::GoFirst) continue;   // the one with right of way never looks
        const Announcement* mine = in.board.find(cid, in.me.id.index, p.connector);
        if (!mine) continue;   // my tail is past it, or I have not announced it
        const bool iAmA = rec.a == p.connector;
        for (const Announcement& e : in.board.on(cid)) {
            if (e.onPath != otherPath || e.who == in.me.id.index || e.notGoing) continue;
            if (turn == MyTurn::Ask &&
                goFirstWhenAsked(in.roads, in.row, in.board, rec.junction, *answer, iAmA, *mine, e, &tieGroupsCut))
                continue;
            // Ahead of it: my tail clear before its front arrives. Behind it: my front arriving after its tail is clear.
            const Real ahead = e.arriveIn - mine->clearIn;
            const Real behind = mine->arriveIn - e.clearIn;
            const Real have = std::max(ahead, behind);
            if (have < needed || !tightest || have < j.have) {
                j.cause = GiveWayCause::Gap;
                j.other = CarId(static_cast<uint32_t>(e.who));
                j.crossing = cid;
                j.otherPath = otherPath;
                j.need = needed;
                j.have = have;
                tightest = true;
            }
            if (have < needed) return true;
        }
    }
    return false;
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

uint32_t currentPhase(const RightOfWay&, engine::JunctionId) { return 0; }

bool stopIsJunctionsOwn(const Car& c, const JunctionPlan& p, Real atLineWithin) {
    if (!c.held.hasStop || std::fabs(c.held.stopAt - p.startOdometer) > atLineWithin) return false;
    if (c.held.stopFor == StopFor::FullStop) return true;
    return c.held.stopFor == StopFor::GivingWay && p.decision == GiveWayDecision::GiveWay &&
           p.judged.cause == GiveWayCause::Gap;
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

bool goFirstWhenAsked(const engine::LaneGraph& roads, const RightOfWay& row, const AnnouncementBoard& board,
                      engine::JunctionId junction, const MeetingAnswer& answer, bool iAmA, const Announcement& mine,
                      const Announcement& theirs, uint32_t* tieGroupsCut) {
    // 1. A car that is going (it cannot stop at its line, or will not be able to by the time it next decides) before one
    // that is not: it cannot give way, and it judged its gap against the other before it got there.
    if (mine.going != theirs.going) return mine.going;
    // 2. Whoever reaches the line first, by the tick of its fixed arrival; one whose arrival is fixed before one whose
    // is not (it is nearer its line, and has judged).
    if (mine.arrivalFixed && theirs.arrivalFixed) {
        if (mine.arrivalTick != theirs.arrivalTick) return mine.arrivalTick < theirs.arrivalTick;
        // 4. The same tick, and the fall-backs run in a ring: the lower car id first.
        if (inRing(roads, row, board, junction, mine.arrivalTick, mine, theirs, tieGroupsCut)) return mine.who < theirs.who;
    } else if (mine.arrivalFixed != theirs.arrivalFixed) {
        return mine.arrivalFixed;
    }
    // 3. The table's fall-back.
    const bool aGivesWay = answer.fallback == GivesWay::A;
    return iAmA ? !aGivesWay : aGivesWay;
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
        // Past the line, or no longer able to stop before it: it goes (task D4 orders what is inside the junction).
        if (isGoing(p, me.odometer, me.speed, brake, heldStop)) continue;
        // Too far to have announced it: nothing here or beyond is judged yet.
        if (!(in.lineIn[i] <= in.numbers.lookAheadTime)) break;
        // A driver that has not yet decided decides only once it nears the point where it could no longer stop at the
        // line: its comfortable stopping distance, one decision's travel, and the line's own reach. Deciding further out
        // (in a queue, say) would be deciding on guesses that change before it gets there.
        const double toLine = p.startOdometer - me.odometer;
        const Real decideWithin = (brake > 0 ? me.speed * me.speed / (kTwo * brake) : Real(0)) +
                                  me.speed * in.reaction + in.numbers.atLineWithin;
        if (p.decision == GiveWayDecision::Undecided && toLine > decideWithin) break;
        // Not going there, as it announced (held, a stopped car beyond its line it cannot pass, a queue that is not
        // going): others do not wait for it. It still judges, and stops at the line if it must give way (it may be let
        // go this very decision), but a clear road is no decision to go: it has not decided anything there yet.
        const bool notGoing = announcedNotGoing(in, p);

        GiveWayJudgement j;
        const bool give = judgeMeetings(in, p, patience, j, out.tieGroupsCut);
        if (!give) {
            if (notGoing) break;
            out.decision[i] = GiveWayDecision::Go;
            out.judgement[i] = j;
            continue;
        }
        out.decision[i] = GiveWayDecision::GiveWay;
        out.judgement[i] = j;

        // Where to stop: this line, or the one before when my body would not fit between the two junctions.
        std::size_t at = i;
        if (i > 0) {
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
        out.toLine = static_cast<Real>(plans.plans[at].startOdometer - me.odometer);
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
                         "my front is from the line)"));
    FactSpec line = spec(kFactGiveWayLine, FactKind::Place, UnitFamily::Length, noticeGiveWayLine,
                         "the junction's line where I stop to give way, metres ahead (about the car I give way to)");
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
