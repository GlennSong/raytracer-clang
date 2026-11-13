#include "apps/driving/room.h"

#include "apps/driving/act.h"                // kNumberStandstillGap
#include "apps/driving/give_way.h"           // GiveWayVerdict; stopsEveryoneFirst; kNumberAtLineWithin
#include "apps/driving/signal_programme.h"   // lampAt: the lights, a pure function of the clock

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <tuple>
#include <utility>

namespace driving {

namespace {

// HOW OFTEN A LIGHT IS LOOKED AT while checking that it stays green for a car passing through (seconds). A light's
// shortest stage (an all-red, at least the rule set's all_red_shortest) is far longer.
constexpr Real kLampLookEvery = 0.25;
// HOW MANY CYCLES the load-time report looks over for two lights' greens to line up (of the longer of the two cycles).
constexpr Real kReportCycles = 4;
// ... and how long, when the chain's lights run cycles of different lengths, which drift against each other (seconds):
// an hour, long enough for the lab's to come round (40 s against 42.9 s: once in ten minutes).
constexpr Real kMixedCyclesLook = 3600;
// Two cycles this close (seconds) are one: the same programme length, lining up the same way every cycle.
constexpr Real kSameCycleWithin = 0.01;
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

// DOES CAR `me` SEE ITS TURN `next` CLEAR as it passes through it without stopping (issue #284)? A claim through close
// junctions holds only if the car waits nowhere on the way: committed through (commitThrough), it would go before every
// car at the next junction that has not committed (the resolver's committed_first), so taking nobody's turn there, and
// it must not be made to wait inside a junction it passes through, or it reaches the next one later than its claim was
// timed for (the soak: a car held inside junction 58 for a far turn that committed before it, then at junction 73's red
// with its tail in 58's box). So at each crossing point along `next`, every car announced on the other path that the
// resolver puts before the car now (by the table at the phase that junction is in, by its arrival, ...), or that will
// reach the crossing point before it does (it may commit first), must leave it the gap, one way or the other, by the
// car's own announcements there (as if it went on now: clearIfGoing; it may hold a stop at the first line). A car not
// going is passed over, as the judgement does. A crossing point the car has not announced cannot be seen: not clear. A
// car not yet announced is further off than the look-ahead time; arriving after the car, committed through, it gives way
// to it, as to any committed car. With no board (no junction step), nothing is judged. A crossing point that does not
// concern the car (made for a longer body than its own, ADR-0412) is not its to see.
bool seenClearThrough(const RoomWorld& world, const Car& car, engine::PathId next) {
    if (!world.board || !knownPath(world.roads, next)) return true;
    const CarId me = car.id;
    const engine::TagMask myTags = world.routeClasses && car.vehicleClass < world.routeClasses->size()
                                       ? (*world.routeClasses)[car.vehicleClass].tags
                                       : engine::kEveryVehicle;
    const engine::Path& p = world.roads.paths[next.index];
    for (uint32_t k = p.crossings.first; k < p.crossings.end() && k < world.roads.pathCrossings.size(); ++k) {
        const engine::CrossingId cid = world.roads.pathCrossings[k];
        const engine::CrossingPoint& rec = world.roads.crossings[cid.index];
        if (rec.otherSide != engine::OtherSide::Path || !rec.concerns(next, myTags)) continue;
        const MeetingAnswer* answer = world.row.answer(cid, currentPhase(world.row, rec.junction, world.now));
        engine::PathId other;
        if (!answer || myTurn(world.roads, *answer, cid, next, &other) == MyTurn::Neither) continue;
        const Announcement* mine = world.board->find(cid, me.index, next);
        for (const Announcement& e : world.board->on(cid)) {
            if (e.onPath != other || e.who == me.index || e.notGoing) continue;
            Question by = Question::None;
            const bool meFirst = world.resolver.goesFirst(world.roads, cid, me.index, e.who, &by);
            if (meFirst && !(mine && e.arriveIn < mine->arriveIn)) continue;
            if (!mine) return false;
            const bool ahead = !followOnly(by) && mine->clearIfGoing + world.gapNeeded <= e.arriveIn;
            const bool behind = e.clearIn + world.gapNeeded <= mine->arriveIn;
            if (!ahead && !behind) return false;
        }
    }
    return true;
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
    reader.numbers.push_back(need(kNumberRoomAnotherWay, UnitFamily::Time));
    std::string why;
    if (!registry.addReader(reader, &why)) {
        ok = false;
        if (problems) problems->push_back(why);
    }
    return ok;
}

bool hasRoomStep(const FactRegistry& registry) { return registry.findReader(kRoomReaderName) != nullptr; }

bool readRoomNumbers(const RuleSet& rules, RoomNumbers& out, std::string* why) {
    constexpr std::size_t kCount = 8;
    const char* names[kCount] = {kNumberStandstillGap,        kNumberAtLineWithin,      kNumberRoomThroughJunctions,
                                 kNumberRoomClearOfCrosswalk, kNumberRoomToWaitInside, kNumberRoomArrivalSlack,
                                 kNumberRoomCyclesThrough,    kNumberRoomAnotherWay};
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
    out.anotherWayAfter = static_cast<Real>(std::max(0.0, values[7]));
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

// ONE WAY THROUGH JUNCTIONS TOO CLOSE TO STOP BETWEEN, as a claim for room passes through them (RoomLedger::measure):
// the turns taken one after another without stopping, the first at the line the car waits at, and how far beyond that
// line each turn begins (metres; the first's is 0).
struct TurnChain {
    std::array<engine::PathId, kMaxRoomJunctions> turns{};
    std::array<Real, kMaxRoomJunctions> lineAt{};
    std::size_t count = 0;
};

// HOW LONG A CAR OF CLASS `cls` CAN WAIT at the chain's first line, at rest, for the lights to let it through every turn
// of the chain: the longest time between moments when the first turn is green and each later turn with lights lets it
// in from its arrival for room_arrival_slack, over a few of the longest cycle (taken as repeating), seconds. Zero where
// the first junction's lights do not run or no later one's do; kFar where they never line up. `cycle` gets the longest
// cycle of the chain's lights.
Real longestWaitThrough(const engine::LaneGraph& roads, const RightOfWay& row, const RoadRules& roadRules,
                        const TurnChain& chain, const VehicleClass& cls, const RoomNumbers& numbers, bool lightsRun,
                        Real comfortableSideAccel, Real* cycle = nullptr) {
    if (cycle) *cycle = 0;
    if (!lightsRun || chain.count < 2) return 0;
    const engine::PathId c1 = chain.turns[0];
    if (!lightsRunAt(row, roads.paths[c1.index].junction)) return 0;
    Real longestCycle = row.junctions[roads.paths[c1.index].junction.index].cycle;
    bool laterLights = false;
    for (std::size_t j = 1; j < chain.count; ++j) {
        const engine::JunctionId at = roads.paths[chain.turns[j].index].junction;
        if (!lightsRunAt(row, at)) continue;
        laterLights = true;
        longestCycle = std::max(longestCycle, row.junctions[at.index].cycle);
    }
    if (!laterLights) return 0;
    if (cycle) *cycle = longestCycle;
    // HOW LONG TO LOOK: lights of one cycle repeat together every cycle, so a few of it show every way they line up.
    // Lights of different cycles drift against each other (two of 40 s and 42.9 s by 2.9 s a cycle, round in about ten
    // minutes), and how they line up at the start says nothing of later: a car can then wait many cycles where they do
    // not (the soak: 137 s at junction 88 for 87, a chain the first few cycles showed lining up). Those are looked at for
    // kMixedCyclesLook (issue #284).
    bool oneCycle = true;
    for (std::size_t j = 1; j < chain.count; ++j) {
        const engine::JunctionId at = roads.paths[chain.turns[j].index].junction;
        if (lightsRunAt(row, at) &&
            std::fabs(row.junctions[at.index].cycle - row.junctions[roads.paths[c1.index].junction.index].cycle) >
                kSameCycleWithin)
            oneCycle = false;
    }
    const Real horizon =
        oneCycle ? kReportCycles * longestCycle : std::max(kReportCycles * longestCycle, kMixedCyclesLook);
    Mover m;
    m.speedUp = cls.speedUp;
    m.brake = cls.brake;
    m.speedThrough = speedThrough(roads, roadRules, c1, cls.topSpeed, comfortableSideAccel);
    std::array<Real, kMaxRoomJunctions> eta{};
    for (std::size_t j = 1; j < chain.count; ++j) eta[j] = secondsToCover(chain.lineAt[j], m);
    // A MOMENT TO GO FROM REST AT THE FIRST LINE (issue #284, the soak's 301 s at junction 90): its turn green, and still
    // green when the car's front is at_line_within past the line -- short of that a yellow stops it again (the dilemma:
    // it can still stop at the line), so a green's last second is no moment to go from rest -- and every later light
    // letting it in as it gets there. And the moments that work must last a look's step at least: the car asks at
    // moments of its own, a decide interval apart, and one instant that works between two of them is one it misses.
    const Real pastLine = secondsToCover(numbers.atLineWithin, m);
    const auto goesThrough = [&](Real t) {
        if (lampAt(roads, row, c1, t) != Lamp::Green || lampAt(roads, row, c1, t + pastLine) != Lamp::Green) return false;
        for (std::size_t j = 1; j < chain.count; ++j)
            if (lightsRunAt(row, roads.paths[chain.turns[j].index].junction) &&
                !letsInThrough(roads, row, chain.turns[j], t + eta[j], numbers.arrivalSlack))
                return false;
        return true;
    };
    Real first = -1, last = -1, longest = 0;
    bool now = goesThrough(0);
    for (Real t = 0; t < horizon; t += kLampLookEvery) {
        const bool soon = goesThrough(t + kLampLookEvery);
        const bool both = now && soon;
        now = soon;
        if (!both) continue;
        if (last >= 0) longest = std::max(longest, t - last);
        if (first < 0) first = t;
        last = t;
    }
    if (first < 0) return kFar;
    return std::max(longest, horizon - last + first);   // and round the end, the lights repeating
}

// ... as a share of the longest cycle of the chain's lights (how many cycles a car may wait there).
Real longestWaitInCycles(const engine::LaneGraph& roads, const RightOfWay& row, const RoadRules& roadRules,
                         const TurnChain& chain, const VehicleClass& cls, const RoomNumbers& numbers, bool lightsRun,
                         Real comfortableSideAccel) {
    Real cycle = 0;
    const Real wait =
        longestWaitThrough(roads, row, roadRules, chain, cls, numbers, lightsRun, comfortableSideAccel, &cycle);
    if (!(wait > 0) || !std::isfinite(wait)) return wait;
    return cycle > 0 ? wait / cycle : kFar;
}

// THE WAYS A CAR OF CLASS `cls` WAITING AT `c1`'s LINE CAN BE GRANTED ROOM THROUGH, `c1` leading into a stretch too short
// for it (issue #265): every chain of turns its claim would pass through, as RoomLedger::measure goes on (each next
// junction's turns while the stretch before it is too short), up to room_through_junctions, ending in a stretch it can
// stop in. Each chain is handed to `visit` with how it ends: THROUGH (it lets the car through often enough: once in
// room_cycles_through of its longest cycle, arriving from rest), TOO SELDOM (it ends in such a stretch, but its lights line
// up less often than that), or NEVER (no stretch to stop in within the limit, everyone stops first at a junction on it,
// or its lights never line up). `visit` returns false to stop looking. Same chains, same order, every run (the graph's
// order). Before #265 only the next junction's turns were looked at, so a car whose claim had to pass through two close
// junctions to lights that never let it in as it got there was sent that way and waited for ever.
enum class ChainEnds : uint8_t { Through, TooSeldom, Never };
template <typename Visit>
void waysThrough(const engine::LaneGraph& roads, const RightOfWay& row, const RuleSet& rules, const RoadRules& roadRules,
                 const RoomRoads& layout, const RoomNumbers& numbers, bool lightsRun, Real comfortableSideAccel,
                 const VehicleClass& cls, engine::PathId c1, Visit visit) {
    const Real need = roomNeeded(cls, numbers.standstillGap);
    const std::size_t most = std::min<std::size_t>(numbers.throughJunctions, kMaxRoomJunctions);
    TurnChain chain;
    chain.turns[0] = c1;
    chain.count = 1;
    // Walks on from the chain's last turn; false once `visit` asks to stop.
    const std::function<bool()> walk = [&]() -> bool {
        const engine::PathId lastTurn = chain.turns[chain.count - 1];
        const engine::PathId lane = onlyNext(roads, lastTurn);
        if (!knownPath(roads, lane)) return visit(chain, ChainEnds::Never);
        const StretchShape s = stretchFrom(roads, lane);
        const Real room = s.length - (lane.index < layout.roomFrom.size() ? layout.roomFrom[lane.index] : Real(0));
        if (room >= need) {
            if (chain.count < 2) return visit(chain, ChainEnds::Through);
            const Real cycles =
                longestWaitInCycles(roads, row, roadRules, chain, cls, numbers, lightsRun, comfortableSideAccel);
            return visit(chain, cycles <= numbers.cyclesThrough ? ChainEnds::Through
                                : std::isfinite(cycles)         ? ChainEnds::TooSeldom
                                                                : ChainEnds::Never);
        }
        if (!s.atJunction || chain.count >= most) return visit(chain, ChainEnds::Never);
        const engine::Path& last = roads.paths[s.last.index];
        const Real nextLine = chain.lineAt[chain.count - 1] + roads.paths[lastTurn.index].length + s.length;
        for (uint32_t k = last.next.first; k < last.next.end(); ++k) {
            const engine::PathId next = roads.nextPaths[k];
            chain.turns[chain.count] = next;
            chain.lineAt[chain.count] = nextLine;
            ++chain.count;
            if (stopsEveryoneFirst(roads, row, rules, roadRules, next, numbers.atLineWithin)) {
                // (the chain handed over ends with the turn where everyone stops first: issue #284 closes that chain)
                const bool goOn = visit(chain, ChainEnds::Never);
                --chain.count;
                if (!goOn) return false;
                continue;
            }
            const bool goOn = walk();
            --chain.count;
            if (!goOn) return false;
        }
        return true;
    };
    walk();
}

// The chain in words: "connector a then b then c".
std::string chainWords(const TurnChain& chain) {
    std::string out = "connector " + std::to_string(chain.turns[0].index);
    for (std::size_t j = 1; j < chain.count; ++j) out += " then " + std::to_string(chain.turns[j].index);
    return out;
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
        // Closed when no way on lets the class through often enough. (The router closes turns one at a time, so where
        // one way on is fine and another never is -- the soak's 1,129 s at junction 106: on through 87, straight on at
        // 88 never in turn, the near turn at 88 fine -- the turn stays open and a car routed the bad way still waits
        // for ever. Closing the turn whenever any way on never works was tried: 183 turns closed instead of 150, and
        // the traffic it pushed elsewhere stood up to 145 s for room in five minutes. Owed: routes that avoid a bad
        // continuation, or a driver refused for good taking another way (design 9.6 point 2).)
        for (const engine::PathId* c1 = in.first; c1 != in.second; ++c1) {
            bool someWay = false;
            waysThrough(roads, row, rules, roadRules, layout, numbers, lightsRun, comfortableSideAccel, cls, *c1,
                        [&](const TurnChain&, ChainEnds ends) {
                            someWay = ends == ChainEnds::Through;
                            return !someWay;
                        });
            if (!someWay) out.push_back(*c1);
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

ClosedChains chainsClosedForRoom(const engine::LaneGraph& roads, const RightOfWay& row, const RuleSet& rules,
                                 const RoadRules& roadRules, const RoomRoads& layout, const RoomNumbers& numbers,
                                 bool lightsRun, Real comfortableSideAccel, const VehicleClass& cls,
                                 const std::vector<engine::PathId>& closedTurns) {
    std::vector<std::vector<engine::PathId>> chains;
    std::vector<uint8_t> never;
    const Real need = roomNeeded(cls, numbers.standstillGap);
    for (uint32_t l = 0; l < roads.paths.size(); ++l) {
        const engine::PathId lane(l);
        const std::pair<const engine::PathId*, const engine::PathId*> in = layout.connectorsInto(lane);
        if (in.first == in.second || roads.paths[l].kind != engine::PathKind::Lane) continue;
        const StretchShape s = stretchFrom(roads, lane);
        if (s.length - layout.roomFrom[l] >= need) continue;
        for (const engine::PathId* c1 = in.first; c1 != in.second; ++c1) {
            if (std::binary_search(closedTurns.begin(), closedTurns.end(), *c1)) continue;   // closed whole already
            waysThrough(roads, row, rules, roadRules, layout, numbers, lightsRun, comfortableSideAccel, cls, *c1,
                        [&](const TurnChain& chain, ChainEnds ends) {
                            if (ends == ChainEnds::Through || chain.count < 2) return true;
                            chains.push_back(std::vector<engine::PathId>(chain.turns.begin(),
                                                                         chain.turns.begin() + chain.count));
                            never.push_back(ends == ChainEnds::Never ? 1 : 0);
                            return true;
                        });
        }
    }
    return buildClosedChains(std::move(chains), never);
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
        // Through two junctions, or more where the stretch after the next is too short too (the chains the claim passes
        // through, issue #265).
        if (atJunction && lightsRun && lightsRunAt(row, from)) {
            for (const engine::PathId* c1 = in.first; c1 != in.second; ++c1)
                waysThrough(roads, row, rules, roadRules, layout, numbers, lightsRun, comfortableSideAccel, *shortest, *c1,
                            [&](const TurnChain& chain, ChainEnds ends) {
                                if (ends == ChainEnds::Through || chain.count < 2) return true;
                                const Real wait = longestWaitThrough(roads, row, roadRules, chain, *shortest, numbers,
                                                                     lightsRun, comfortableSideAccel);
                                // (lights that line up often enough, or none: it was refused for another reason)
                                if (!(wait > 0) ||
                                    longestWaitInCycles(roads, row, roadRules, chain, *shortest, numbers, lightsRun,
                                                        comfortableSideAccel) <= numbers.cyclesThrough)
                                    return true;
                                ++out.turnPairsTooSeldom;
                                if (chain.count > 2) ++out.turnChainsTooSeldom;
                                line += "; " + chainWords(chain) + ": " +
                                        (std::isfinite(wait) ? "in turn for it only once in " + one(wait) + " s"
                                                             : std::string("never in turn for it")) +
                                        " (it would wait at junction " + std::to_string(from.index) + "'s line)";
                                return true;
                            });
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
        // A CLAIM THROUGH CLOSE JUNCTIONS, checked again from where the car is now, while it is short of the line and can
        // still stop at it (its hardest braking, after one decide interval) (issue #284). No longer holding, it is given
        // back as refused: the car stops at the line (roomStopFor) and asks again when it next means to go.
        const double toLine = p.startOdometer - c.odometer;
        RoomRefusal why = RoomRefusal::None;
        engine::JunctionId where;
        if (r.through > 1 && toLine > 0 &&
            canStopWithin(static_cast<Real>(toLine), c.speed, seat.cls->hardestBrake, world.reaction) &&
            !chainHolds(world, seat, k, why, where)) {
            r.state = RoomState::Refused;
            r.refusal = why;
            r.at = where;
            r.by = CarId();
            ++counts_.givenBack;
            ++counts_.chainsGivenBack;
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

Real RoomLedger::arrivalThrough(const RoomWorld& world, const RoomSeat& seat, engine::PathId connector, Real distance) {
    const Car& c = *seat.car;
    const VehicleClass& cls = *seat.cls;
    const Real d = std::max(Real(0), distance);
    Mover m;
    m.speed = c.speed;
    m.speedUp = cls.speedUp;
    m.brake = cls.brake;
    m.speedThrough = speedThrough(world.roads, world.roadRules, connector, cls.topSpeed, world.comfortableSideAccel);
    Real eta = secondsToCover(d, m);
    // The car ahead on the way: I get there no sooner than it has covered the gap between us and the distance, and a
    // standstill gap more at my speed through.
    pathsAfter_.clear();
    const std::vector<engine::PathId>& way = seat.way->paths;
    if (!way.empty() && way[0] == c.path)
        for (std::size_t k = 1; k < way.size(); ++k) pathsAfter_.push_back(way[k]);
    const Neighbour n = carAhead(world.roads, world.cars, world.lists, c.id, pathsAfter_, d);
    if (!n.found()) return eta;
    const Car* a = findCar(world.cars, world.lists, n.car);
    if (!a || a->vehicleClass >= world.classes.size()) return eta;
    const VehicleClass& ac = world.classes.at(a->vehicleClass);
    Mover am;
    am.speed = a->speed;
    am.speedUp = ac.speedUp;
    am.brake = ac.brake;
    am.speedThrough = speedThrough(world.roads, world.roadRules, connector, ac.topSpeed, world.comfortableSideAccel);
    const Real behindIt = secondsToCover(std::max(Real(0), d - n.gap), am) +
                          world.numbers.standstillGap / std::max(m.speedThrough, kSlowestThrough);
    return std::max(eta, behindIt);
}

bool RoomLedger::chainHolds(const RoomWorld& world, const RoomSeat& seat, std::size_t k, RoomRefusal& why,
                            engine::JunctionId& at) {
    const JunctionPlans& plans = *seat.plans;
    const Car& c = *seat.car;
    if (!seenClearThrough(world, c, plans.plans[k].connector)) {
        why = RoomRefusal::GivesWayAhead;
        at = plans.plans[k].junction;
        return false;
    }
    for (std::size_t j = k + 1; j < plans.count && j < kMaxJunctionPlans; ++j) {
        const JunctionPlan& q = plans.plans[j];
        if (q.room.state != RoomState::Granted || !q.room.chained) break;
        const engine::JunctionId jn = q.junction;
        if (world.lightsRun && lightsRunAt(world.row, jn)) {
            const Real eta =
                arrivalThrough(world, seat, plans.plans[k].connector, static_cast<Real>(q.startOdometer - c.odometer));
            if (!letsInThrough(world.roads, world.row, q.connector, world.now + static_cast<double>(eta),
                               world.numbers.arrivalSlack)) {
                why = RoomRefusal::LightsAhead;
                at = jn;
                return false;
            }
        }
        if (!seenClearThrough(world, c, q.connector)) {
            why = RoomRefusal::GivesWayAhead;
            at = jn;
            return false;
        }
    }
    return true;
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
            const Real toThatLine = static_cast<Real>(laneStart - c.odometer) + s.end;
            const Real eta = arrivalThrough(world, seat, p.connector, toThatLine);
            if (!letsInThrough(roads, world.row, next, world.now + static_cast<double>(eta), n.arrivalSlack)) {
                ans.refusal = RoomRefusal::LightsAhead;
                return ans;
            }
        }
        // ... and its turn there seen clear as it gets there: it could only wait for a car it gives way to with its tail in
        // the box behind (issue #284).
        if (!seenClearThrough(world, c, next)) {
            ans.refusal = RoomRefusal::GivesWayAhead;
            return ans;
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
    // Through junctions too close to stop between, it must not be made to wait inside the first either (issue #284).
    if (junctions > 1 && !seenClearThrough(world, c, p.connector)) {
        ans.refusal = RoomRefusal::GivesWayAhead;
        ans.at = p.junction;
        return ans;
    }
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
    // waiting at its line for it (through a red too); each once, and none already on the way. A car refused room on it
    // and waiting at its line with its front a few centimetres onto its connector (rolled over the line as the refusal
    // came) is first in the queue by when it asked, not in transit: counted as in transit, two cars refused for one lane
    // at two lines each counted the other ahead of it and neither was granted (a two-car ring the five-minute lab showed
    // at junction 62 once drivers moved off sooner, issue #284).
    const uint64_t myAsk = plans.plans[k].room.asked ? plans.plans[k].room.askedTick : world.tick;
    transit_.clear();
    for (std::size_t s = 0; s < stretches_.size(); ++s) {
        const engine::PathId lane = stretches_[s].firstLane;
        const engine::PathId mine = s > 0 ? stretches_[s - 1].intoNext : engine::PathId();
        Waiting first;
        first.lane = lane;
        const std::vector<Waiting>::const_iterator waitingFrom =
            std::lower_bound(waiting_.begin(), waiting_.end(), first,
                             [](const Waiting& a, const Waiting& b) { return a.lane.index < b.lane.index; });
        const auto waitsForIt = [&](CarId car) {
            for (std::vector<Waiting>::const_iterator w = waitingFrom; w != waiting_.end() && w->lane == lane; ++w)
                if (w->car == car.index) return true;
            return false;
        };
        const std::pair<const engine::PathId*, const engine::PathId*> in = world.layout.connectorsInto(lane);
        for (const engine::PathId* conn = in.first; conn != in.second; ++conn) {
            if (*conn == mine) continue;
            for (const PathEntry& e : world.lists.on(*conn))
                if (e.why == OnPath::Front && e.car != c.id && !waitsForIt(e.car)) transit_.push_back(e.car.index);
        }
        RoomGrant key;
        key.lane = lane;
        key.car = 0;
        for (std::vector<RoomGrant>::const_iterator g =
                 std::lower_bound(grants.begin(), grants.end(), key,
                                  [](const RoomGrant& a, const RoomGrant& b) { return a.lane.index < b.lane.index; });
             g != grants.end() && g->lane == lane; ++g)
            if (g->car != c.id.index && g->odometer < g->laneAt) transit_.push_back(g->car);
        for (std::vector<Waiting>::const_iterator w = waitingFrom; w != waiting_.end() && w->lane == lane; ++w)
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
        const Real braking = stoppingDistance(speed, oc.brake);
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
            // A CLAIM THROUGH CLOSE JUNCTIONS waits, as at its own line, for a car ahead of it on its way that will reach
            // a junction it passes through first and holds no grant there (issue #284: the soak's first gridlock. A pickup
            // granted through junctions 14 and 11 behind a moped -- short enough to stop between them -- took the room
            // beyond 11 the moped then needed; the moped waited at 11 for room behind the pickup's tail, the pickup behind
            // the moped, for good.)
            for (uint8_t j = 1; a.granted && j < a.junctions && j < kMaxRoomJunctions; ++j) {
                if (a.plans[j] >= plans.count) break;
                const CarId first = aheadWithoutRoom(world, seats, seat, plans.plans[a.plans[j]]);
                if (!first.valid()) continue;
                a.granted = false;
                a.refusal = RoomRefusal::CarAheadFirst;
                a.by = first;
                a.at = plans.plans[a.plans[j]].junction;
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
