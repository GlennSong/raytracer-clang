#include "apps/driving/room.h"

#include "apps/driving/act.h"                // kNumberStandstillGap
#include "apps/driving/give_way.h"           // GiveWayVerdict; stopsEveryoneFirst; kNumberAtLineWithin
#include "apps/driving/signal_programme.h"   // lampAt: the lights, a pure function of the clock

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <tuple>
#include <utility>

namespace driving {

namespace {

constexpr Real kTwo = 2;   // the 2 in v^2 = 2 a s
// HOW OFTEN A LIGHT IS LOOKED AT while checking that it stays green for a car passing through (seconds). A light's
// shortest stage (an all-red, at least the rule set's all_red_shortest) is far longer.
constexpr Real kLampLookEvery = 0.25;
// HOW MANY CYCLES the load-time report looks over for two lights' greens to line up (of the longer of the two cycles).
constexpr Real kReportCycles = 4;
constexpr Real kFar = std::numeric_limits<Real>::infinity();

bool knownPath(const engine::LaneGraph& roads, engine::PathId p) { return p.valid() && p.index < roads.paths.size(); }

FactNeed need(const char* name, UnitFamily family) {
    FactNeed n;
    n.name = name;
    n.family = family;
    return n;
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

void noticeNoRoomBeyond(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.room) return;
    out.present = true;
    out.yes = v.room->waits;
    if (!v.room->waits) return;
    if (v.room->by.valid()) out.aboutCar = v.room->by;
    out.hasMetres = true;
    out.metres = v.room->toLine;
}

void noticeRoomLine(const DriverView& v, NoticeScratch&, NoticedFact& out) {
    if (!v.room || !v.room->waits) return;
    out.present = true;
    out.number = v.room->toLine;
    if (v.room->by.valid()) out.aboutCar = v.room->by;
    out.hasMetres = true;
    out.metres = v.room->toLine;
}

// WILL `connector`'s LIGHT LET A CAR IN at any moment from `from` for `window` seconds: green, or yellow (a car committed
// through to it from the junction before goes on at a yellow, which it could only stop for with its tail in the box
// behind: commitThrough), never red?
bool letsInThrough(const engine::LaneGraph& roads, const RightOfWay& row, engine::PathId connector, double from,
                   Real window) {
    for (Real t = 0;; t += kLampLookEvery) {
        const Lamp lamp = lampAt(roads, row, connector, from + static_cast<double>(std::min(t, window)));
        if (lamp != Lamp::Green && lamp != Lamp::Yellow) return false;
        if (!(t < window)) return true;
    }
}

// THE ONE LANE THE ROAD RUNS ON INTO after `from`, where it does not branch: a connector's lane, or a lane's single next
// lane. None where it branches or meets a junction's connectors.
engine::PathId onlyNext(const engine::LaneGraph& roads, engine::PathId from) {
    if (!knownPath(roads, from)) return engine::PathId();
    const engine::Path& fp = roads.paths[from.index];
    if (fp.next.count != 1 || fp.next.first >= roads.nextPaths.size()) return engine::PathId();
    const engine::PathId nx = roads.nextPaths[fp.next.first];
    return knownPath(roads, nx) && roads.paths[nx.index].kind == engine::PathKind::Lane ? nx : engine::PathId();
}

bool lightsRunAt(const RightOfWay& row, engine::JunctionId j) {
    return j.valid() && j.index < row.junctions.size() && row.junctions[j.index].running();
}

std::string one(double v) {
    char b[48];
    std::snprintf(b, sizeof b, "%.1f", v);
    return b;
}

}  // namespace

// ---- The rule set ---------------------------------------------------------------------------------------------------

bool registerRoomFacts(FactRegistry& registry, std::vector<std::string>* problems) {
    std::vector<FactSpec> specs;
    specs.push_back(spec(kFactNoRoomBeyond, FactKind::YesNo, UnitFamily::Ratio, noticeNoRoomBeyond,
                         "I was refused room beyond a junction ahead whose line I have not passed (about the car whose "
                         "tail bounds the room, and how far my front is from that line)"));
    FactSpec line = spec(kFactRoomLine, FactKind::Place, UnitFamily::Length, noticeRoomLine,
                         "where I wait for room beyond: that junction's line, metres ahead (about the car whose tail "
                         "bounds the room)");
    line.stopFor = StopFor::NoRoom;
    specs.push_back(line);
    bool ok = true;
    for (FactSpec& s : specs) {
        std::string why;
        if (registry.add(std::move(s), &why) == kNoFact) {
            ok = false;
            if (problems) problems->push_back(why);
        }
    }
    ReaderSpec reader;
    reader.name = kRoomReaderName;
    reader.meaning = "grants room beyond a junction, one car at a time, before a car may enter it";
    reader.numbers.push_back(need(kNumberStandstillGap, UnitFamily::Length));
    reader.numbers.push_back(need(kNumberAtLineWithin, UnitFamily::Length));
    reader.numbers.push_back(need(kNumberRoomThroughJunctions, UnitFamily::Ratio));
    reader.numbers.push_back(need(kNumberRoomClearOfCrosswalk, UnitFamily::Length));
    reader.numbers.push_back(need(kNumberRoomToWaitInside, UnitFamily::Ratio));
    reader.numbers.push_back(need(kNumberRoomArrivalSlack, UnitFamily::Time));
    reader.numbers.push_back(need(kNumberRoomCyclesThrough, UnitFamily::Ratio));
    std::string why;
    if (!registry.addReader(reader, &why)) {
        ok = false;
        if (problems) problems->push_back(why);
    }
    return ok;
}

bool hasRoomStep(const FactRegistry& registry) { return registry.findReader(kRoomReaderName) != nullptr; }

bool readRoomNumbers(const RuleSet& rules, RoomNumbers& out, std::string* why) {
    constexpr std::size_t kCount = 7;
    const char* names[kCount] = {kNumberStandstillGap,        kNumberAtLineWithin,      kNumberRoomThroughJunctions,
                                 kNumberRoomClearOfCrosswalk, kNumberRoomToWaitInside, kNumberRoomArrivalSlack,
                                 kNumberRoomCyclesThrough};
    double values[kCount] = {};
    for (std::size_t i = 0; i < kCount; ++i) {
        const Number* n = rules.findNumber(names[i]);
        if (!n) {
            if (why) *why = "rule set \"" + rules.name + "\" lacks the number \"" + names[i] + "\", which " + kRoomReaderName + " reads";
            return false;
        }
        values[i] = n->value;
    }
    out.standstillGap = static_cast<Real>(values[0]);
    out.atLineWithin = static_cast<Real>(values[1]);
    const double through = std::floor(values[2] + 0.5);
    out.throughJunctions = static_cast<uint32_t>(std::max(1.0, std::min(static_cast<double>(kMaxRoomJunctions), through)));
    out.clearOfCrosswalk = static_cast<Real>(values[3]);
    out.toWaitInside = values[4] != 0;
    out.arrivalSlack = static_cast<Real>(std::max(0.0, values[5]));
    out.cyclesThrough = static_cast<Real>(std::max(0.0, values[6]));
    return true;
}

Real bodyLength(const VehicleClass& cls) { return cls.length + (cls.hasTrailer() ? cls.trailerLength : Real(0)); }

Real roomNeeded(const VehicleClass& cls, Real standstillGap) { return bodyLength(cls) + standstillGap; }

// ---- The road ---------------------------------------------------------------------------------------------------------

std::pair<const engine::PathId*, const engine::PathId*> RoomRoads::connectorsInto(engine::PathId lane) const {
    if (!lane.valid() || lane.index + 1 >= firstInto.size()) return {nullptr, nullptr};
    const engine::PathId* base = into.data();
    return {base + firstInto[lane.index], base + firstInto[lane.index + 1]};
}

RoomRoads studyRoomRoads(const engine::LaneGraph& roads, Real clearOfCrosswalk) {
    RoomRoads out;
    const std::size_t n = roads.paths.size();
    out.firstInto.assign(n + 1, 0);
    out.roomFrom.assign(n, 0);
    for (uint32_t p = 0; p < n; ++p) {
        const engine::Path& path = roads.paths[p];
        if (path.kind != engine::PathKind::Connector) continue;
        for (uint32_t k = path.next.first; k < path.next.end() && k < roads.nextPaths.size(); ++k)
            if (knownPath(roads, roads.nextPaths[k])) ++out.firstInto[roads.nextPaths[k].index + 1];
    }
    for (std::size_t i = 0; i < n; ++i) out.firstInto[i + 1] += out.firstInto[i];
    out.into.assign(out.firstInto[n], engine::PathId());
    std::vector<uint32_t> fill(out.firstInto.begin(), out.firstInto.end() - 1);
    for (uint32_t p = 0; p < n; ++p) {   // in id order, so each lane's list is too
        const engine::Path& path = roads.paths[p];
        if (path.kind != engine::PathKind::Connector) continue;
        for (uint32_t k = path.next.first; k < path.next.end() && k < roads.nextPaths.size(); ++k)
            if (knownPath(roads, roads.nextPaths[k])) out.into[fill[roads.nextPaths[k].index]++] = engine::PathId(p);
    }
    // A crosswalk across the start of a lane leaving a junction is inside the box: room begins past it.
    for (const engine::CrossingPoint& x : roads.crossings) {
        if (x.otherSide != engine::OtherSide::Crosswalk || !x.junction.valid() || !knownPath(roads, x.a)) continue;
        const engine::Path& lane = roads.paths[x.a.index];
        if (lane.kind != engine::PathKind::Lane || out.firstInto[x.a.index] == out.firstInto[x.a.index + 1]) continue;
        if (!(x.aFrom < lane.length * Real(0.5))) continue;   // the crosswalk at its far end belongs to the next junction
        out.roomFrom[x.a.index] = std::max(out.roomFrom[x.a.index], std::min(lane.length, x.aTo + clearOfCrosswalk));
    }
    return out;
}

namespace {

// THE STRETCH A LANE BEGINS: it and the lanes it runs straight on into, to the next junction's line or the road's end.
struct StretchShape {
    Real length = 0;
    engine::PathId last;        // its last lane
    bool atJunction = false;    // it ends at a junction's line (its last lane leads into connectors)
};
StretchShape stretchFrom(const engine::LaneGraph& roads, engine::PathId lane) {
    StretchShape s;
    engine::PathId at = lane;
    for (std::size_t guard = 0; guard < kMaxFollowedPaths && knownPath(roads, at); ++guard) {
        s.length += roads.paths[at.index].length;
        s.last = at;
        const engine::PathId nx = onlyNext(roads, at);
        if (!nx.valid()) break;
        at = nx;
    }
    if (knownPath(roads, s.last)) {
        const engine::Path& p = roads.paths[s.last.index];
        s.atJunction = p.next.count > 0 && knownPath(roads, roads.nextPaths[p.next.first]) &&
                       roads.paths[roads.nextPaths[p.next.first].index].kind == engine::PathKind::Connector;
    }
    return s;
}

// HOW LONG A CAR OF CLASS `cls` CAN WAIT at `c1`'s line, at rest, for the two lights to let it through `c2` (the turn
// after a stretch of `length` it cannot stop in): the longest time between moments when `c1` is green and `c2` lets it in
// from its arrival for room_arrival_slack, over a few of the longer cycle (taken as repeating), seconds. Zero where
// either junction's lights do not run; kFar where the two never line up.
Real longestWaitInTurn(const engine::LaneGraph& roads, const RightOfWay& row, const RoadRules& roadRules,
                       engine::PathId c1, engine::PathId c2, Real length, const VehicleClass& cls,
                       const RoomNumbers& numbers, bool lightsRun, Real comfortableSideAccel) {
    const engine::JunctionId from = roads.paths[c1.index].junction, to = roads.paths[c2.index].junction;
    if (!lightsRun || !lightsRunAt(row, from) || !lightsRunAt(row, to)) return 0;
    const Real horizon = kReportCycles * std::max(row.junctions[from.index].cycle, row.junctions[to.index].cycle);
    Mover m;
    m.speedUp = cls.speedUp;
    m.brake = cls.brake;
    m.speedThrough = speedThrough(roads, roadRules, c1, cls.topSpeed, comfortableSideAccel);
    const Real eta = secondsToCover(roads.paths[c1.index].length + length, m);
    Real first = -1, last = -1, longest = 0;
    for (Real t = 0; t < horizon; t += kLampLookEvery) {
        if (lampAt(roads, row, c1, t) != Lamp::Green || !letsInThrough(roads, row, c2, t + eta, numbers.arrivalSlack))
            continue;
        if (last >= 0) longest = std::max(longest, t - last);
        if (first < 0) first = t;
        last = t;
    }
    if (first < 0) return kFar;
    return std::max(longest, horizon - last + first);   // and round the end, the lights repeating
}

// ... as a share of the longer of the two junctions' cycles (how many cycles a car may wait there).
Real longestWaitInCycles(const engine::LaneGraph& roads, const RightOfWay& row, const RoadRules& roadRules,
                         engine::PathId c1, engine::PathId c2, Real length, const VehicleClass& cls,
                         const RoomNumbers& numbers, bool lightsRun, Real comfortableSideAccel) {
    const Real wait =
        longestWaitInTurn(roads, row, roadRules, c1, c2, length, cls, numbers, lightsRun, comfortableSideAccel);
    if (!(wait > 0) || !std::isfinite(wait)) return wait;
    const Real cycle = std::max(row.junctions[roads.paths[c1.index].junction.index].cycle,
                                row.junctions[roads.paths[c2.index].junction.index].cycle);
    return cycle > 0 ? wait / cycle : kFar;
}

}  // namespace

std::vector<engine::PathId> turnsClosedForRoom(const engine::LaneGraph& roads, const RightOfWay& row,
                                               const RuleSet& rules, const RoadRules& roadRules, const RoomRoads& layout,
                                               const RoomNumbers& numbers, bool lightsRun, Real comfortableSideAccel,
                                               const VehicleClass& cls) {
    std::vector<engine::PathId> out;
    const Real need = roomNeeded(cls, numbers.standstillGap);
    for (uint32_t l = 0; l < roads.paths.size(); ++l) {
        const engine::PathId lane(l);
        const std::pair<const engine::PathId*, const engine::PathId*> in = layout.connectorsInto(lane);
        if (in.first == in.second || roads.paths[l].kind != engine::PathKind::Lane) continue;
        const StretchShape s = stretchFrom(roads, lane);
        if (s.length - layout.roomFrom[l] >= need) continue;
        const engine::Path& last = roads.paths[s.last.index];
        for (const engine::PathId* c1 = in.first; c1 != in.second; ++c1) {
            bool someWay = false;
            for (uint32_t k = last.next.first; s.atJunction && k < last.next.end() && !someWay; ++k) {
                const engine::PathId c2 = roads.nextPaths[k];
                someWay = !stopsEveryoneFirst(roads, row, rules, roadRules, c2, numbers.atLineWithin) &&
                          longestWaitInCycles(roads, row, roadRules, *c1, c2, s.length, cls, numbers, lightsRun,
                                              comfortableSideAccel) <= numbers.cyclesThrough;
            }
            if (!someWay) out.push_back(*c1);
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

RoomRoadReport reportRoomRoads(const engine::LaneGraph& roads, const RightOfWay& row, const RuleSet& rules,
                               const RoadRules& roadRules, const VehicleClasses& classes, const RoomRoads& layout,
                               const RoomNumbers& numbers, bool lightsRun, Real comfortableSideAccel) {
    RoomRoadReport out;
    if (classes.empty()) return out;
    for (uint32_t l = 0; l < roads.paths.size(); ++l) {
        const engine::PathId lane(l);
        const std::pair<const engine::PathId*, const engine::PathId*> in = layout.connectorsInto(lane);
        if (in.first == in.second || roads.paths[l].kind != engine::PathKind::Lane) continue;
        const StretchShape shape = stretchFrom(roads, lane);
        const Real length = shape.length;
        const engine::PathId at = shape.last;
        const Real room = length - layout.roomFrom[l];
        const VehicleClass* shortest = nullptr;   // the shortest class that does not fit
        std::string cannot;
        for (const VehicleClass& c : classes.all())
            if (roomNeeded(c, numbers.standstillGap) > room) {
                cannot += (cannot.empty() ? "" : ", ") + c.name;
                if (!shortest || c.length < shortest->length) shortest = &c;
            }
        if (!shortest) continue;
        ++out.shortStretches;
        const engine::Path& last = roads.paths[at.index];
        const bool atJunction = shape.atJunction;
        const engine::JunctionId from = roads.paths[in.first->index].junction;
        const engine::JunctionId to = atJunction ? roads.paths[roads.nextPaths[last.next.first].index].junction
                                                 : engine::JunctionId();
        std::string line = "lane " + std::to_string(l) + ", junction " + std::to_string(from.index) + " to " +
                           (to.valid() ? "junction " + std::to_string(to.index) : std::string("the road's end")) + ": " +
                           one(room) + " m of room; cannot stop in it: " + cannot;
        if (!atJunction) {
            line += " (THE ROAD ENDS: they can never use it)";
            ++out.blockedByAStop;
        } else if (stopsEveryoneFirst(roads, row, rules, roadRules, roads.nextPaths[last.next.first],
                                      numbers.atLineWithin)) {
            line += " (everyone stops first at junction " + std::to_string(to.index) + ": they can never use it)";
            ++out.blockedByAStop;
        }
        // Two junctions with lights: each pair of turns through the two whose lights let the shortest class that does not
        // fit through, arriving from rest at the first line, less often than once in room_cycles_through of the longer
        // cycle (sampled over a few of it): closed to it (turnsClosedForRoom).
        if (atJunction && lightsRun && lightsRunAt(row, from) && lightsRunAt(row, to)) {
            for (const engine::PathId* c1 = in.first; c1 != in.second; ++c1)
                for (uint32_t k = last.next.first; k < last.next.end(); ++k) {
                    const engine::PathId c2 = roads.nextPaths[k];
                    if (longestWaitInCycles(roads, row, roadRules, *c1, c2, length, *shortest, numbers, lightsRun,
                                            comfortableSideAccel) <= numbers.cyclesThrough)
                        continue;
                    const Real wait = longestWaitInTurn(roads, row, roadRules, *c1, c2, length, *shortest, numbers,
                                                        lightsRun, comfortableSideAccel);
                    ++out.turnPairsTooSeldom;
                    line += "; connector " + std::to_string(c1->index) + " then " + std::to_string(c2.index) + ": " +
                            (std::isfinite(wait) ? "in turn for it only once in " + one(wait) + " s"
                                                 : std::string("never in turn for it")) +
                            " (it would wait at junction " + std::to_string(from.index) + "'s line)";
                }
        }
        out.lines.push_back(line);
    }
    return out;
}

// ---- Asking -----------------------------------------------------------------------------------------------------------

std::size_t roomToAsk(const JunctionPlans& plans, const Car& me, const GiveWayVerdict& v, const RoomNumbers& n,
                      bool* toWaitInside) {
    if (!v.judged) return kMaxJunctionPlans;
    for (std::size_t i = 0; i < plans.count && i < kMaxJunctionPlans; ++i) {
        const JunctionPlan& p = plans.plans[i];
        if (!p.connector.valid()) continue;
        // Granted here already, or past the line (by more than at_line_within: inside, it can only clear): the next
        // junction may be asked. (A car that rolled a few centimetres over the line is "committed" to the resolver, but
        // can still stop and wait for room.)
        const double toLine = p.startOdometer - me.odometer;
        if (p.room.state == RoomState::Granted || toLine < -n.atLineWithin) continue;
        // Going there: it decided to, or -- at rest at the line, judged with nothing to give way to -- it is about to move
        // off without that counting as a decision (released by a red, say: give_way.h's gate), and must ask first.
        const bool atRestAtLine = me.speed < kStandingBelow && toLine <= n.atLineWithin;
        const bool go = v.decision[i] == GiveWayDecision::Go || (v.clear[i] && atRestAtLine);
        const bool waitInside = n.toWaitInside && v.giveWay && v.stopPlan == i &&
                                v.decision[i] == GiveWayDecision::GiveWay && v.judgement[i].stopAlong > 0;
        if (toWaitInside) *toWaitInside = waitInside && !go;
        // The nearest junction it holds no room at: it asks there or nowhere.
        return go || waitInside ? i : kMaxJunctionPlans;
    }
    return kMaxJunctionPlans;
}

void commitThrough(JunctionPlans& plans) {
    for (std::size_t k = 1; k < plans.count && k < kMaxJunctionPlans; ++k) {
        JunctionPlan& p = plans.plans[k];
        const JunctionPlan& before = plans.plans[k - 1];
        if (p.committed || !before.committed || p.room.state != RoomState::Granted || !p.room.chained) continue;
        p.committed = true;
        p.committedTick = before.committedTick;
    }
}

Real roomReach(const engine::LaneGraph& roads, const RoomRoads& layout, const JunctionPlans& plans, const Car& me,
               const WayAhead& way, Real need, const RoomNumbers& n) {
    // The junction a claim would be made at: the nearest whose line is ahead (or passed by no more than at_line_within),
    // with no room held.
    const JunctionPlan* p = nullptr;
    for (std::size_t i = 0; i < plans.count && i < kMaxJunctionPlans && !p; ++i) {
        const JunctionPlan& q = plans.plans[i];
        if (!knownPath(roads, q.connector) || q.room.state == RoomState::Granted ||
            q.startOdometer - me.odometer < -n.atLineWithin)
            continue;
        p = &q;
    }
    if (!p) return 0;
    // Where the way has it: the index of the connector, then of each path after it.
    std::size_t i = 0;
    while (i < way.paths.size() && way.paths[i] != p->connector) ++i;
    if (i == way.paths.size()) return 0;   // not on the way at all: nothing to measure along it
    double reach = p->startOdometer - me.odometer + roads.paths[p->connector.index].length;
    ++i;
    engine::PathId came = p->connector;
    for (uint32_t junctions = 1; junctions < n.throughJunctions && junctions < kMaxRoomJunctions; ++junctions) {
        // The stretch after `came`: along the way while it has lanes, then on along the road where it does not branch.
        const engine::PathId first = i < way.paths.size() ? way.paths[i] : onlyNext(roads, came);
        if (!knownPath(roads, first)) return 0;
        Real length = 0;
        engine::PathId last = first;
        for (engine::PathId at = first; at.valid();) {
            length += roads.paths[at.index].length;
            last = at;
            if (i < way.paths.size()) ++i;
            if (i < way.paths.size())
                at = roads.paths[way.paths[i].index].kind == engine::PathKind::Lane ? way.paths[i] : engine::PathId();
            else
                at = onlyNext(roads, last);
        }
        const Real from = first.index < layout.roomFrom.size() ? layout.roomFrom[first.index] : Real(0);
        if (length - from >= need) return 0;   // it can stop there: the road itself says how far that room goes
        reach += length;
        // Too short: the claim passes through the next junction, whose connector only the way can name.
        if (i >= way.paths.size()) {
            const engine::Path& lp = roads.paths[last.index];
            if (lp.next.count == 0) return 0;            // the road ends: no way could reach further
            return static_cast<Real>(reach) + n.atLineWithin;   // into the next connector, so it is planned for
        }
        const engine::PathId next = way.paths[i];
        reach += roads.paths[next.index].length;
        came = next;
        ++i;
    }
    return 0;
}

void askForRoom(JunctionPlans& plans, std::size_t plan, bool toWaitInside, uint64_t tick) {
    if (plan >= plans.count || plan >= kMaxJunctionPlans) return;
    RoomClaim& r = plans.plans[plan].room;
    if (r.state == RoomState::Granted) return;
    r.state = RoomState::Asked;
    r.toWaitInside = toWaitInside;
    if (!r.asked) {
        r.asked = true;
        r.askedTick = tick;
    }
}

RoomStop roomStopFor(const JunctionPlans& plans, const Car& me, const RoomNumbers& n) {
    RoomStop out;
    for (std::size_t i = 0; i < plans.count && i < kMaxJunctionPlans; ++i) {
        const JunctionPlan& p = plans.plans[i];
        if (p.room.state != RoomState::Refused) continue;
        // Its front past the line by no more than at_line_within (it rolled off the line as the refusal came): it stops
        // where it is.
        const double toLine = p.startOdometer - me.odometer;
        if (toLine < -n.atLineWithin) continue;
        out.waits = true;
        out.toLine = static_cast<Real>(std::max(0.0, toLine));
        out.by = p.room.by;
        return out;
    }
    return out;
}

// ---- Granting ---------------------------------------------------------------------------------------------------------

void RoomLedger::keepOrLetGo(const RoomWorld& world, RoomSeat& seat) {
    const Car& c = *seat.car;
    JunctionPlans& plans = *seat.plans;
    const Real atLine = world.numbers.atLineWithin;
    for (std::size_t k = 0; k < plans.count && k < kMaxJunctionPlans; ++k) {
        JunctionPlan& p = plans.plans[k];
        RoomClaim& r = p.room;
        if (r.state != RoomState::Granted) continue;
        if (c.odometer >= r.laneAt) {   // its front is on the lane: it is counted where it is from now on
            r.state = RoomState::None;
            r.chained = false;
            ++counts_.usedOnLane;
            continue;
        }
        if (r.chained) {
            // Part of the claim at the junction before: let go with it, when that was given back before the car entered.
            const JunctionPlan* before = k > 0 ? &plans.plans[k - 1] : nullptr;
            if (before && before->room.state != RoomState::Granted && c.odometer < before->startOdometer + atLine) {
                r.state = RoomState::None;
                r.chained = false;
                ++counts_.givenBack;
            }
            continue;
        }
        // Given back: short of the line, its driver now means to stop there or before for something else (it decided to
        // give way at the line, a red stops it, a hold): it asks again when it means to go.
        const bool shortOfLine = c.odometer < p.startOdometer + atLine;
        const bool stopsShort = c.held.hasStop && c.held.stopFor != StopFor::NoRoom &&
                                c.held.stopAt <= p.startOdometer + atLine;
        if (shortOfLine && stopsShort) {
            r.state = RoomState::None;
            ++counts_.givenBack;
        }
    }
}

void RoomLedger::addGrant(const RoomGrant& g) {
    const std::vector<RoomGrant>::iterator at =
        std::lower_bound(grants_.begin(), grants_.end(), g, [](const RoomGrant& a, const RoomGrant& b) {
            return std::make_tuple(a.lane.index, a.car) < std::make_tuple(b.lane.index, b.car);
        });
    grants_.insert(at, g);
}

void RoomLedger::orderSameTick(const RoomWorld& world, std::size_t first, std::size_t end) {
    // A topological order of the resolver's answers (Kahn's method), the lower car id first among those free to come
    // next. Its answers hold no ring, so every car is placed; were one found, the lower id would go first.
    const std::size_t n = end - first;
    ordered_.clear();
    placed_.assign(n, 0);
    for (std::size_t pos = 0; pos < n; ++pos) {
        std::size_t pick = n, lowest = n;
        for (std::size_t i = 0; i < n && pick == n; ++i) {
            if (placed_[i]) continue;
            if (lowest == n) lowest = i;
            bool free = true;
            for (std::size_t j = 0; j < n && free; ++j)
                if (j != i && !placed_[j] &&
                    world.resolver.order(claims_[first + j].car, claims_[first + i].car) == Answer::First)
                    free = false;
            if (free) pick = i;
        }
        if (pick == n) pick = lowest;
        if (pick != lowest) ++counts_.orderedByResolver;
        placed_[pick] = 1;
        ordered_.push_back(claims_[first + pick]);
    }
    std::copy(ordered_.begin(), ordered_.end(), claims_.begin() + static_cast<std::ptrdiff_t>(first));
}

CarId RoomLedger::aheadWithoutRoom(const RoomWorld& world, const std::vector<RoomSeat>& seats, const RoomSeat& seat,
                                   const JunctionPlan& plan) {
    const Car& c = *seat.car;
    const double toLine = plan.startOdometer - c.odometer;
    if (!(toLine > world.numbers.atLineWithin)) return CarId();   // at the line: nobody is between it and the line
    pathsAfter_.clear();
    const std::vector<engine::PathId>& way = seat.way->paths;
    if (!way.empty() && way[0] == c.path)
        for (std::size_t k = 1; k < way.size() && way[k] != plan.connector; ++k) pathsAfter_.push_back(way[k]);
    const Neighbour n = carAhead(world.roads, world.cars, world.lists, c.id, pathsAfter_, static_cast<Real>(toLine));
    if (!n.found()) return CarId();
    const Car* a = findCar(world.cars, world.lists, n.car);
    if (!a || a->vehicleClass >= world.classes.size()) return CarId();
    // Its front past the line: it is entering, not queued short of it.
    if (n.gap + bodyLength(world.classes.at(a->vehicleClass)) > static_cast<Real>(toLine) + world.numbers.atLineWithin)
        return CarId();
    const std::vector<RoomSeat>::const_iterator s =
        std::lower_bound(seats.begin(), seats.end(), a->id, [](const RoomSeat& x, CarId want) { return x.car->id < want; });
    if (s == seats.end() || s->car->id != a->id) return n.car;   // nobody drives it: it holds this car anyway
    for (std::size_t k = 0; k < s->plans->count && k < kMaxJunctionPlans; ++k)
        if (s->plans->plans[k].junction == plan.junction && s->plans->plans[k].room.state == RoomState::Granted)
            return CarId();
    return n.car;
}

RoomAnswer RoomLedger::measure(const RoomWorld& world, const RoomSeat& seat, std::size_t k,
                               const std::vector<RoomGrant>& grants) {
    RoomAnswer ans;
    const engine::LaneGraph& roads = world.roads;
    const RoomNumbers& n = world.numbers;
    const Car& c = *seat.car;
    const VehicleClass& cls = *seat.cls;
    const JunctionPlans& plans = *seat.plans;
    ans.need = roomNeeded(cls, n.standstillGap);
    ans.refusal = RoomRefusal::NotPlanned;
    if (k >= plans.count || !knownPath(roads, plans.plans[k].connector)) return ans;
    const JunctionPlan& p = plans.plans[k];
    ans.at = p.junction;
    const std::vector<engine::PathId>& way = seat.way->paths;
    std::size_t i = 0;
    while (i < way.size() && way[i] != p.connector) ++i;
    if (i == way.size()) return ans;
    ++i;
    // The odometer reading where the car's front will reach the first lane beyond; the way is measured from there.
    const double laneStart = p.startOdometer + roads.paths[p.connector.index].length;

    // 1. THE WAY: stretch after stretch, through each junction its body would not fit before, up to the rule set's limit.
    // Where the driver's way stops short (a car at rest looks no further than the junctions it planned), the road is
    // followed on where it does not branch: a connector leads into one lane, and a lane runs on into the next while it
    // has one. Which connector it takes at the next junction only its way can say.
    segs_.clear();
    stretches_.clear();
    Real pos = 0;
    uint8_t junctions = 1;
    ans.plans[0] = static_cast<uint8_t>(k);
    engine::PathId came = p.connector;   // the connector the stretch begins after
    for (;;) {
        const engine::PathId first = i < way.size() ? way[i] : onlyNext(roads, came);
        if (!knownPath(roads, first) || roads.paths[first.index].kind != engine::PathKind::Lane) return ans;   // NotPlanned
        Stretch s;
        s.start = pos;
        s.firstLane = first;
        s.roomFrom = first.index < world.layout.roomFrom.size() ? world.layout.roomFrom[first.index] : Real(0);
        engine::PathId last = first;
        for (engine::PathId at = first; at.valid();) {
            Seg g;
            g.path = at;
            g.from = pos;
            segs_.push_back(g);
            pos += roads.paths[at.index].length;
            last = at;
            if (i < way.size()) ++i;
            if (i < way.size()) {
                at = roads.paths[way[i].index].kind == engine::PathKind::Lane ? way[i] : engine::PathId();
            } else {
                at = segs_.size() < kMaxFollowedPaths ? onlyNext(roads, last) : engine::PathId();
            }
        }
        if (i < way.size()) s.intoNext = way[i];
        s.end = pos;
        stretches_.push_back(s);
        ans.lanes[junctions - 1] = s.firstLane;
        ans.laneAt[junctions - 1] = laneStart + s.start;
        if (s.end - s.start - s.roomFrom >= ans.need) break;   // it can come to rest here
        // Too short to stop in: it must pass through the next junction too, without stopping.
        const engine::Path& lp = roads.paths[last.index];
        const bool roadEnds = lp.next.count == 0;
        if (junctions >= n.throughJunctions || junctions >= kMaxRoomJunctions || roadEnds) {
            ans.refusal = RoomRefusal::TooShort;
            ans.at = s.intoNext.valid() ? roads.paths[s.intoNext.index].junction : p.junction;
            return ans;
        }
        if (!s.intoNext.valid()) return ans;   // NotPlanned: its way does not say where it goes after
        const engine::PathId next = s.intoNext;
        const engine::JunctionId jn = roads.paths[next.index].junction;
        ans.at = jn;
        std::size_t q = kMaxJunctionPlans;
        for (std::size_t m = ans.plans[junctions - 1] + 1u; m < plans.count && m < kMaxJunctionPlans; ++m)
            if (plans.plans[m].connector == next) {
                q = m;
                break;
            }
        if (q == kMaxJunctionPlans) return ans;   // NotPlanned
        if (stopsEveryoneFirst(roads, world.row, world.rules, world.roadRules, next, n.atLineWithin)) {
            ans.refusal = RoomRefusal::MustStopAhead;
            return ans;
        }
        if (world.lightsRun && lightsRunAt(world.row, jn)) {
            Mover m;
            m.speed = c.speed;
            m.speedUp = cls.speedUp;
            m.brake = cls.brake;
            m.speedThrough =
                speedThrough(roads, world.roadRules, p.connector, cls.topSpeed, world.comfortableSideAccel);
            const Real toThatLine = static_cast<Real>(laneStart - c.odometer) + s.end;
            const Real eta = secondsToCover(std::max(Real(0), toThatLine), m);
            if (!letsInThrough(roads, world.row, next, world.now + static_cast<double>(eta), n.arrivalSlack)) {
                ans.refusal = RoomRefusal::LightsAhead;
                return ans;
            }
        }
        Seg g;
        g.path = next;
        g.from = pos;
        segs_.push_back(g);
        pos += roads.paths[next.index].length;
        ++i;
        came = next;
        ans.plans[junctions] = static_cast<uint8_t>(q);
        ++junctions;
    }
    ans.junctions = junctions;
    ans.at = engine::JunctionId();

    // 2. THE CARS ON THE WAY, each once: its rear-most point on it, and its front if that is on it.
    onWay_.clear();
    for (const Seg& g : segs_)
        for (const PathEntry& e : world.lists.on(g.path)) {
            if (e.why == OnPath::Beside || e.car == c.id) continue;
            OnWay o;
            o.car = e.car.index;
            o.rear = g.from + e.rear;
            o.front = g.from + e.front;
            o.frontOn = e.why == OnPath::Front;
            onWay_.push_back(o);
        }
    std::sort(onWay_.begin(), onWay_.end(), [](const OnWay& a, const OnWay& b) {
        return std::make_tuple(a.car, a.rear) < std::make_tuple(b.car, b.rear);
    });
    std::size_t kept = 0;
    for (std::size_t a = 0; a < onWay_.size(); ++a) {
        if (kept > 0 && onWay_[kept - 1].car == onWay_[a].car) {
            OnWay& o = onWay_[kept - 1];
            o.rear = std::min(o.rear, onWay_[a].rear);
            if (onWay_[a].frontOn) {
                o.front = onWay_[a].front;
                o.frontOn = true;
            }
            continue;
        }
        onWay_[kept++] = onWay_[a];
    }
    onWay_.resize(kept);

    // 3. IN TRANSIT to each stretch's first lane: on a connector into it (not the car's own way), or granted room on it
    // with its front not there yet; and FIRST IN THE QUEUE for it: refused room on it before this car first asked, and
    // waiting at its line for it (through a red too); each once, and none already on the way.
    const uint64_t myAsk = plans.plans[k].room.asked ? plans.plans[k].room.askedTick : world.tick;
    transit_.clear();
    for (std::size_t s = 0; s < stretches_.size(); ++s) {
        const engine::PathId lane = stretches_[s].firstLane;
        const engine::PathId mine = s > 0 ? stretches_[s - 1].intoNext : engine::PathId();
        const std::pair<const engine::PathId*, const engine::PathId*> in = world.layout.connectorsInto(lane);
        for (const engine::PathId* conn = in.first; conn != in.second; ++conn) {
            if (*conn == mine) continue;
            for (const PathEntry& e : world.lists.on(*conn))
                if (e.why == OnPath::Front && e.car != c.id) transit_.push_back(e.car.index);
        }
        RoomGrant key;
        key.lane = lane;
        key.car = 0;
        for (std::vector<RoomGrant>::const_iterator g =
                 std::lower_bound(grants.begin(), grants.end(), key,
                                  [](const RoomGrant& a, const RoomGrant& b) { return a.lane.index < b.lane.index; });
             g != grants.end() && g->lane == lane; ++g)
            if (g->car != c.id.index && g->odometer < g->laneAt) transit_.push_back(g->car);
        Waiting first;
        first.lane = lane;
        for (std::vector<Waiting>::const_iterator w =
                 std::lower_bound(waiting_.begin(), waiting_.end(), first,
                                  [](const Waiting& a, const Waiting& b) { return a.lane.index < b.lane.index; });
             w != waiting_.end() && w->lane == lane; ++w)
            if (w->car != c.id.index && std::make_pair(w->askedTick, w->car) < std::make_pair(myAsk, c.id.index))
                transit_.push_back(w->car);
    }
    std::sort(transit_.begin(), transit_.end());
    transit_.erase(std::unique(transit_.begin(), transit_.end()), transit_.end());
    Real inTransit = 0;
    CarId transitCar;
    for (const uint32_t t : transit_) {
        const std::vector<OnWay>::const_iterator o =
            std::lower_bound(onWay_.begin(), onWay_.end(), t, [](const OnWay& x, uint32_t want) { return x.car < want; });
        if (o != onWay_.end() && o->car == t) continue;   // on the way: counted where it is
        const Car* rec = findCar(world.cars, world.lists, CarId(t));
        if (!rec || rec->vehicleClass >= world.classes.size()) continue;
        inTransit += roomNeeded(world.classes.at(rec->vehicleClass), n.standstillGap);
        if (!transitCar.valid()) transitCar = CarId(t);
    }

    // 4. WHERE THE BACK OF THE QUEUE WOULD COME TO REST: from the front one back, each car braking comfortably from its
    // speed now, held behind the one ahead by a gap and never past the end of the last stretch (or a hold of the
    // asker's own on it). A car whose front is beyond the way (leaving it) rests where its own braking takes it.
    std::sort(onWay_.begin(), onWay_.end(), [](const OnWay& a, const OnWay& b) {
        const Real fa = a.frontOn ? a.front : kFar, fb = b.frontOn ? b.front : kFar;
        if (fa != fb) return fa > fb;
        if (a.rear != b.rear) return a.rear > b.rear;
        return a.car < b.car;
    });
    const Stretch& last = stretches_.back();
    Real limit = last.end;
    if (seat.holds)
        for (const Hold& h : *seat.holds)
            for (const Seg& g : segs_) {
                if (g.path != h.at.path || g.from < last.start) continue;
                const Real at = g.from + h.at.along;
                if (at > last.start + last.roomFrom && at < limit) limit = at;
            }
    CarId tail;
    for (const OnWay& o : onWay_) {
        const Car* rec = findCar(world.cars, world.lists, CarId(o.car));
        if (!rec || rec->vehicleClass >= world.classes.size()) continue;
        const VehicleClass& oc = world.classes.at(rec->vehicleClass);
        const Real speed = std::max(Real(0), rec->speed);
        const Real braking = oc.brake > 0 ? speed * speed / (kTwo * oc.brake) : Real(0);
        const Real restRear = o.frontOn ? std::min(o.front + braking, limit) - bodyLength(oc) : o.rear + braking;
        if (restRear - n.standstillGap < limit) {
            limit = restRear - n.standstillGap;
            tail = CarId(o.car);
        }
    }
    const Real beyondBox = limit - (last.start + last.roomFrom);
    ans.room = beyondBox - inTransit;
    ans.granted = ans.room >= ans.need;
    ans.refusal = ans.granted ? RoomRefusal::None : RoomRefusal::NoRoom;
    if (!ans.granted) ans.by = beyondBox < ans.need || !transitCar.valid() ? tail : transitCar;
    return ans;
}

void RoomLedger::settle(const RoomWorld& world, std::vector<RoomSeat>& seats) {
    // 1. Grants held: kept, or let go.
    for (RoomSeat& s : seats) keepOrLetGo(world, s);
    grants_.clear();
    for (const RoomSeat& s : seats)
        for (std::size_t k = 0; k < s.plans->count && k < kMaxJunctionPlans; ++k) {
            const RoomClaim& r = s.plans->plans[k].room;
            if (r.state != RoomState::Granted) continue;
            RoomGrant g;
            g.lane = r.lane;
            g.car = s.car->id.index;
            g.laneAt = r.laneAt;
            g.odometer = s.car->odometer;
            g.need = r.need;
            grants_.push_back(g);
        }
    std::sort(grants_.begin(), grants_.end(), [](const RoomGrant& a, const RoomGrant& b) {
        return std::make_tuple(a.lane.index, a.car) < std::make_tuple(b.lane.index, b.car);
    });
    // ... and the cars first in the queue for room: refused it for want of it at their last ask, still at their line.
    waiting_.clear();
    for (const RoomSeat& s : seats)
        for (std::size_t k = 0; k < s.plans->count && k < kMaxJunctionPlans; ++k) {
            const JunctionPlan& p = s.plans->plans[k];
            if (p.room.state != RoomState::Refused || p.room.refusal != RoomRefusal::NoRoom || !p.room.lane.valid() ||
                p.startOdometer - s.car->odometer < -world.numbers.atLineWithin)
                continue;
            Waiting w;
            w.lane = p.room.lane;
            w.car = s.car->id.index;
            w.askedTick = p.room.askedTick;
            w.need = p.room.need;
            waiting_.push_back(w);
        }
    const auto waitingOrder = [](const Waiting& a, const Waiting& b) {
        return std::make_tuple(a.lane.index, a.askedTick, a.car) < std::make_tuple(b.lane.index, b.askedTick, b.car);
    };
    std::sort(waiting_.begin(), waiting_.end(), waitingOrder);

    // 2. This tick's asks, in order: who first asked earlier; then the resolver's order; then the lower id.
    claims_.clear();
    for (std::size_t s = 0; s < seats.size(); ++s)
        for (std::size_t k = 0; k < seats[s].plans->count && k < kMaxJunctionPlans; ++k) {
            const RoomClaim& r = seats[s].plans->plans[k].room;
            if (r.state != RoomState::Asked) continue;
            Claim c;
            c.seat = static_cast<uint32_t>(s);
            c.plan = static_cast<uint8_t>(k);
            c.askedTick = r.askedTick;
            c.car = seats[s].car->id.index;
            claims_.push_back(c);
        }
    std::sort(claims_.begin(), claims_.end(), [](const Claim& a, const Claim& b) {
        return std::make_tuple(a.askedTick, a.car, a.plan) < std::make_tuple(b.askedTick, b.car, b.plan);
    });
    for (std::size_t i = 0; i < claims_.size();) {
        std::size_t e = i + 1;
        while (e < claims_.size() && claims_[e].askedTick == claims_[i].askedTick) ++e;
        if (e - i > 1) orderSameTick(world, i, e);
        i = e;
    }

    // 3. Each ask granted or refused, every grant counted against the room before the next is looked at.
    for (const Claim& cl : claims_) {
        RoomSeat& seat = seats[cl.seat];
        JunctionPlans& plans = *seat.plans;
        JunctionPlan& p = plans.plans[cl.plan];
        RoomClaim& r = p.room;
        if (r.state != RoomState::Asked) continue;
        ++counts_.asked;
        r.need = roomNeeded(*seat.cls, world.numbers.standstillGap);
        RoomAnswer a;
        const CarId ahead = aheadWithoutRoom(world, seats, seat, p);
        if (ahead.valid()) {
            a.refusal = RoomRefusal::CarAheadFirst;
            a.by = ahead;
            a.need = r.need;
        } else {
            a = measure(world, seat, cl.plan, grants_);
            if (a.granted && r.toWaitInside && a.junctions > 1) {
                a.granted = false;
                a.refusal = RoomRefusal::NoStopToWait;
                a.at = engine::JunctionId();
            }
        }
        r.had = a.room;
        if (!a.granted) {
            r.state = RoomState::Refused;
            r.refusal = a.refusal;
            r.by = a.by;
            r.at = a.at;
            r.through = a.junctions;
            r.lane = a.junctions > 0 ? a.lanes[0] : engine::PathId();
            ++counts_.refused[static_cast<std::size_t>(a.refusal)];
            // Refused for want of room: from now on it is in the queue for it, ahead of every car that asked later.
            if (a.refusal == RoomRefusal::NoRoom && r.lane.valid()) {
                Waiting w;
                w.lane = r.lane;
                w.car = seat.car->id.index;
                w.askedTick = r.askedTick;
                w.need = r.need;
                waiting_.insert(std::lower_bound(waiting_.begin(), waiting_.end(), w, waitingOrder), w);
            }
            if ((a.refusal == RoomRefusal::TooShort || a.refusal == RoomRefusal::MustStopAhead) && a.at.valid()) {
                const std::pair<uint32_t, uint32_t> place(a.at.index, a.lanes[0].index);
                const std::vector<std::pair<uint32_t, uint32_t>>::iterator at =
                    std::lower_bound(counts_.roadDataPlaces.begin(), counts_.roadDataPlaces.end(), place);
                if (at == counts_.roadDataPlaces.end() || *at != place) counts_.roadDataPlaces.insert(at, place);
            }
            continue;
        }
        ++counts_.granted;
        if (a.junctions > 1) ++counts_.grantedThrough;
        if (r.toWaitInside) ++counts_.grantedToWaitInside;
        for (uint8_t j = 0; j < a.junctions; ++j) {
            RoomClaim& g = plans.plans[a.plans[j]].room;
            g.state = RoomState::Granted;
            g.chained = j > 0;
            g.through = j == 0 ? a.junctions : uint8_t(0);
            g.lane = a.lanes[j];
            g.laneAt = a.laneAt[j];
            g.need = r.need;
            g.had = a.room;
            g.refusal = RoomRefusal::None;
            g.by = CarId();
            g.at = engine::JunctionId();
            RoomGrant grant;
            grant.lane = a.lanes[j];
            grant.car = seat.car->id.index;
            grant.laneAt = a.laneAt[j];
            grant.odometer = seat.car->odometer;
            grant.need = r.need;
            addGrant(grant);
        }
    }
    counts_.mostHeld = std::max<uint32_t>(counts_.mostHeld, static_cast<uint32_t>(grants_.size()));
}

}  // namespace driving
