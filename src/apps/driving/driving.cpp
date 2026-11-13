#include "apps/driving/driving.h"

#include "apps/driving/act.h"
#include "apps/driving/measure.h"
#include "engine/bundle/bundle.h"   // Fnv: the repo's 64-bit hash folded field by field

#include <algorithm>
#include <array>
#include <string>
#include <tuple>

namespace driving {

bool eventBefore(const Event& a, const Event& b) {
    return std::make_tuple(a.car.index, static_cast<uint8_t>(a.kind), a.trip.index, a.trip.generation, a.tick, a.person) <
           std::make_tuple(b.car.index, static_cast<uint8_t>(b.kind), b.trip.index, b.trip.generation, b.tick, b.person);
}

Driving::Driving(const engine::LaneGraph* roads) : roads_(roads) {
    if (roads_) routeBounds_ = studyRoads(*roads_);
}

void Driving::tick(Real dt) {
    if (hasRules_ && roads_ && dt > 0) {
        step_ = dt;
        plan();
        settleClaims();
        move(dt);
    }
    ++tickCount_;
    clockSeconds_ += dt;
    if (measurements_) measurements_->observe(*this);   // it only watches
}

void Driving::attachMeasurements(Measurements* m) {
    measurements_ = m;
    if (m && !m->begun()) m->begin(*this);
}

// SETTLE CLAIMS (design 7.1 step 2): the room beyond each junction asked for in this tick's plan step, granted one car at
// a time in one fixed order (room.h). Every driven car is a seat, in car-id order: its grants are kept or let go, and its
// position counts against the room others ask for.
void Driving::settleClaims() {
    if (!roomOn_ || !roads_) return;
    findTripsOfRecords();
    roomSeats_.clear();
    for (std::size_t i = 0; i < cars_.size(); ++i) {
        const Car& c = cars_[i];
        Trip* t = trips_.get(tripOfRecord_[i]);
        if (!c.driven || !t || c.vehicleClass >= vehicleClasses_.size()) continue;
        RoomSeat s;
        s.car = &c;
        s.plans = &t->junctions;
        s.way = &t->way;
        s.holds = &t->holds;
        s.cls = &vehicleClasses_.at(c.vehicleClass);
        roomSeats_.push_back(s);
    }
    RoomWorld world{*roads_,  rightOfWay_,    ruleSet_, roadRules_,  vehicleClasses_, cars_,
                    pathLists_, roomRoads_, roomNumbers_, resolver_, lightsRun_,     junctionNumbers_.comfortableSideAccel,
                    tickCount_, clockSeconds_};
    // A claim through close junctions sees whether its onward turns are clear by this tick's board (issue #284).
    world.board = &board_;
    world.gapNeeded = junctionNumbers_.gapComfortable;
    world.reaction = static_cast<Real>(std::max<uint32_t>(1, settings_.decideEveryTicks)) * step_;
    world.routeClasses = &routeClasses_;
    room_.settle(world, roomSeats_);
}

void Driving::closeTurnsForRoom() {
    if (!roads_ || !hasRules_) return;
    for (std::size_t i = 0; i < routeClasses_.size() && i < vehicleClasses_.size(); ++i) {
        routeClasses_[i].closed =
            roomOn_ ? turnsClosedForRoom(*roads_, rightOfWay_, ruleSet_, roadRules_, roomRoads_, roomNumbers_, lightsRun_,
                                         junctionNumbers_.comfortableSideAccel, vehicleClasses_.at(i))
                    : std::vector<engine::PathId>();
        // ... and the ways on from an open turn that never (or too seldom) let it through (issue #284).
        routeClasses_[i].closedChains =
            roomOn_ ? chainsClosedForRoom(*roads_, rightOfWay_, ruleSet_, roadRules_, roomRoads_, roomNumbers_, lightsRun_,
                                          junctionNumbers_.comfortableSideAccel, vehicleClasses_.at(i),
                                          routeClasses_[i].closed)
                    : ClosedChains();
    }
}

RouteClass Driving::routeClass(uint32_t vehicleClass) const {
    RouteClass cls = vehicleClass < routeClasses_.size() ? routeClasses_[vehicleClass] : RouteClass();
    cls.laneChanges = settings_.laneChanges;
    return cls;
}

RoomRoadReport Driving::roomRoadReport() const {
    if (!roomOn_ || !roads_) return RoomRoadReport();
    return reportRoomRoads(*roads_, rightOfWay_, ruleSet_, roadRules_, vehicleClasses_, roomRoads_, roomNumbers_,
                           lightsRun_, junctionNumbers_.comfortableSideAccel);
}

bool Driving::onRoad(const PathPoint& p) const {
    if (!roads_ || !p.path.valid() || p.path.index >= roads_->paths.size()) return false;
    const engine::Path& path = roads_->paths[p.path.index];
    return p.along >= 0 && p.along <= path.length;
}

namespace {

// Says why, when the caller asked; always false (a refusal).
bool refuse(std::string* why, const std::string& reason) {
    if (why) *why = reason;
    return false;
}

std::string pathName(engine::PathId p) { return "path " + std::to_string(p.index); }
std::string carName(CarId c) { return "car " + std::to_string(c.index); }

// Does `from` lead straight on into `to` (is `to` among the paths leaving its end)?
bool leadsInto(const engine::LaneGraph& roads, engine::PathId from, engine::PathId to) {
    const engine::Range next = roads.paths[from.index].next;
    for (uint32_t i = next.first; i < next.end() && i < roads.nextPaths.size(); ++i)
        if (roads.nextPaths[i] == to) return true;
    return false;
}

}  // namespace

// ---- The rules --------------------------------------------------------------------------------------------------------

bool Driving::useRules(RuleSet ruleSet, FactRegistry facts, AnswerVocabulary vocabulary, CompiledRules rules,
                       RoadRules roadRules, std::string* why) {
    const FactSlot arrived = facts.find(kFactArrived);
    if (arrived == kNoFact)
        return refuse(why, std::string("the fact registry has no \"") + kFactArrived +
                               "\" fact (registerBuiltInFacts adds it): the library ends a trip by it");
    const PersonalSetting* gapKept = ruleSet.findPersonal(kSettingGapKept);
    const Number* standstill = ruleSet.findNumber(kNumberStandstillGap);
    const Number* anticipation = ruleSet.findNumber(kNumberAnticipation);
    if (!gapKept || !standstill || !anticipation)
        return refuse(why, "rule set \"" + ruleSet.name + "\" lacks the setting \"" + kSettingGapKept +
                               "\" or the number \"" + kNumberStandstillGap + "\" or \"" + kNumberAnticipation +
                               "\", which " + kMoveReaderName +
                               " reads (buildRoadRules reports it, given a registry with registerMoveNeeds)");
    // The junction step, when the registry has it: its numbers must all be there (buildRoadRules names any missing).
    const bool junctions = hasJunctionStep(facts);
    JunctionNumbers junctionNumbers;
    if (junctions && !readJunctionNumbers(ruleSet, junctionNumbers, why)) return false;
    // The lights' clock, when the host registered it (buildRoadRules then bound the lights as running, not dark).
    LightTiming lightTiming;
    if (roadRules.lightsRun && !readLightTiming(ruleSet, lightTiming, why)) return false;
    // Room beyond (#264), when the registry has it: it needs the junction step (its asks come from the judgement).
    const bool room = hasRoomStep(facts);
    if (room && !junctions)
        return refuse(why, std::string(kRoomReaderName) + " needs the junction step (registerJunctionFacts) too");
    RoomNumbers roomNumbers;
    if (room && !readRoomNumbers(ruleSet, roomNumbers, why)) return false;
    roomOn_ = room;
    roomNumbers_ = roomNumbers;
    lightsRun_ = roadRules.lightsRun;
    lightTiming_ = lightTiming;
    junctionsOn_ = junctions;
    junctionNumbers_ = junctionNumbers;
    gapKept_ = *gapKept;
    gapKeptHash_ = settingNameHash(kSettingGapKept);
    standstillGap_ = static_cast<Real>(standstill->value);
    anticipation_ = static_cast<Real>(std::max(0.0, anticipation->value));
    ruleSet_ = std::move(ruleSet);
    facts_ = std::move(facts);
    vocabulary_ = std::move(vocabulary);
    rules_ = std::move(rules);
    roadRules_ = std::move(roadRules);
    arrivedSlot_ = arrived;
    wanted_ = rules_.usedFacts;
    wanted_.push_back(arrived);
    std::sort(wanted_.begin(), wanted_.end());
    wanted_.erase(std::unique(wanted_.begin(), wanted_.end()), wanted_.end());
    hasRules_ = true;
    // Who gives way to whom, from the road and these rules; every driver plans its junctions afresh against it.
    if (roads_) {
        rightOfWay_ = buildRightOfWay(*roads_, ruleSet_, roadRules_);
        // ... and every junction's traffic lights timed, with the far turns' places to wait inside checked (#261).
        if (lightsRun_) timeSignalProgrammes(*roads_, roadRules_, lightTiming_, rightOfWay_);
        // ... and checked: no stage lets in together two movements whose paths meet unless the rule set allows it. A
        // breach is reported (signalProblems, and in the table's notes), not refused: the host decides (issue #284).
        signalProblems_ = lightsRun_ ? checkSignalProgrammes(*roads_, ruleSet_, rightOfWay_) : std::vector<std::string>();
        // (First among the notes, so a host printing only the first few still shows them.)
        for (std::size_t i = 0; i < signalProblems_.size(); ++i)
            rightOfWay_.notes.insert(rightOfWay_.notes.begin() + static_cast<std::ptrdiff_t>(i),
                                     "unsafe programme: " + signalProblems_[i]);
        // ... and the road as room beyond reads it: what leads into each lane, and where its room begins (#264).
        roomRoads_ = roomOn_ ? studyRoomRoads(*roads_, roomNumbers_.clearOfCrosswalk) : RoomRoads();
        closeTurnsForRoom();
    }
    for (std::pair<const CarId, TripId>& entry : tripOfCar_)
        if (Trip* t = trips_.get(entry.second)) t->junctions = JunctionPlans();
    // Answers held under other rules name other rules: every driver decides afresh on the next tick.
    for (Car& c : cars_) c.held.decided = false;
    return true;
}

void Driving::notePlanUpdate(const PlanUpdate& u, CarId car, Trip& t, const JunctionPlans& plans) {
    planCounts_.made += u.made;
    for (std::size_t i = 0; i < u.noteCount; ++i) {
        switch (u.notes[i].change) {
        case PlanChange::Made: break;
        case PlanChange::Cleared: ++planCounts_.cleared; break;
        case PlanChange::WayChanged: ++planCounts_.wayChanged; break;
        case PlanChange::RouteChanged: ++planCounts_.routeChanged; break;
        }
        if (settings_.trace) t.trace.addPlanNote(u.notes[i]);
    }
    if (u.plansCut) {
        ++capCounts_.junctionPlansCut;
        capCounts_.lastJunctionPlanCut = car;
    }
    if (u.waitingPointsCut) {
        ++capCounts_.waitingPointsCut;
        capCounts_.lastWaitingPointCut = car;
    }
    planCounts_.mostAtOnce = std::max<uint32_t>(planCounts_.mostAtOnce, plans.count);
    for (std::size_t i = 0; i < plans.count; ++i)
        if (plans.plans[i].crossingCount > planCounts_.mostCrossings) {
            planCounts_.mostCrossings = plans.plans[i].crossingCount;
            planCounts_.mostCrossingsConnector = plans.plans[i].connector;
        }
}

void Driving::applyVerdict(const GiveWayVerdict& v, const std::array<Real, kMaxJunctionPlans>& lineIn, const Car& car,
                           Trip& t) {
    if (!v.judged) return;
    for (std::size_t k = 0; k < t.junctions.count && k < kMaxJunctionPlans; ++k) {
        const GiveWayDecision d = v.decision[k];
        if (d == GiveWayDecision::Undecided) continue;   // not judged this time: kept as it was
        JunctionPlan& plan = t.junctions.plans[k];
        // Now stopping at or short of the line for something that is no junction's (a hold, its destination, a lane's
        // end): it is not going there, so nothing it judged there or beyond is a decision yet.
        if (car.held.hasStop && car.held.stopFor == StopFor::Place &&
            car.held.stopAt <= plan.startOdometer + junctionNumbers_.atLineWithin)
            break;
        // Judged for the first time: its arrival at the line is fixed now, by its own estimate -- unless it comes to a
        // full stop there first, when it arrives as it comes to rest (noteAtTheLine): it holds a full stop there, or the
        // junction's control makes everyone stop (its stop to give way may be the one held, at the same place).
        const ControlKind* control = junctionControl(rightOfWay_, ruleSet_, plan.junction);
        const bool stopsFirst = stopsFirstAt(car, plan, junctionNumbers_.atLineWithin) ||
                                (control && control->everyoneStops && !plan.cameToRest);
        if (!stopsFirst) fixArrival(plan, lineIn[k], tickCount_, clockSeconds_, step_);
        const GiveWayJudgement& j = v.judgement[k];
        const bool changed = plan.decision != d;
        const bool toAnother = d == GiveWayDecision::GiveWay && !changed && plan.judged.cause != j.cause;
        const bool toAnotherCar = d == GiveWayDecision::GiveWay && !changed && plan.judged.other != j.other;
        // A wait INSIDE the junction (task D4): stopping inside for a car that goes first, or going on again from such a
        // stop. It is not a change of mind about the line, and it takes no gap at the line: counted on its own.
        const bool inside = (d == GiveWayDecision::GiveWay && j.stopAlong > 0) ||
                            (plan.decision == GiveWayDecision::GiveWay && plan.judged.stopAlong > 0);
        if (changed) {
            if (d == GiveWayDecision::Go) {
                ++planCounts_.decidedGo;
                if (plan.decision == GiveWayDecision::GiveWay && j.cause == GiveWayCause::Gap && !inside) {
                    // A gap taken after giving way.
                    const bool first = planCounts_.wentAfterGivingWay == 0;
                    ++planCounts_.wentAfterGivingWay;
                    planCounts_.smallestGapTaken = first ? j.have : std::min(planCounts_.smallestGapTaken, j.have);
                    planCounts_.largestGapTaken = first ? j.have : std::max(planCounts_.largestGapTaken, j.have);
                    planCounts_.smallestGapNeeded = first ? j.need : std::min(planCounts_.smallestGapNeeded, j.need);
                }
            } else {
                ++planCounts_.decidedGiveWay;
            }
            // Held at the line for room beyond (#264): it cannot go whatever it judges, and announces that it is not going,
            // so its judgement turning with the cross traffic is no change of mind anyone acts on: counted on its own.
            const bool heldForRoom = plan.room.state == RoomState::Refused;
            if (inside) {
                if (d == GiveWayDecision::GiveWay) ++planCounts_.waitedInside;
            } else if (heldForRoom) {
                ++planCounts_.switchedWhileHeldForRoom;
            } else if (plan.decision != GiveWayDecision::Undecided && ++plan.mindChanges == 2) {
                ++planCounts_.changedMindTwice;
            }
        }
        plan.decision = d;
        plan.judged = j;
        if (settings_.trace && (changed || toAnother || toAnotherCar)) {
            GiveWayNote n;
            n.tick = tickCount_;
            n.seconds = clockSeconds_;
            n.junction = plan.junction;
            n.connector = plan.connector;
            n.decision = d;
            n.judged = j;
            t.trace.addGiveWayNote(n);
        }
    }
}

// ---- The tick ---------------------------------------------------------------------------------------------------------

void Driving::findTripsOfRecords() {
    tripOfRecord_.assign(cars_.size(), TripId{});
    std::map<CarId, TripId>::const_iterator it = tripOfCar_.begin();
    for (std::size_t i = 0; i < cars_.size(); ++i) {
        while (it != tripOfCar_.end() && it->first < cars_[i].id) ++it;
        if (it != tripOfCar_.end() && it->first == cars_[i].id) tripOfRecord_[i] = it->second;
    }
}

void Driving::leaveParked(Car& c) {
    c.driven = false;
    c.speed = 0;
    c.acceleration = 0;
    c.held = HeldAnswers();
    c.binding = Binding();
}

// THE START OF THE PLAN STEP (issue #262): every driven car notes what its own state says about the lines ahead, then
// announces itself on the board for the junctions within its look-ahead time. Each car writes only its own plans and
// its own announcements, from the cars as they stood at the start of the tick; the one thing taken from another car --
// a queue's following interval, and a stopped car beyond the line -- is worked nearest the line first, so the car ahead
// is always settled before the car behind it. The board is then sorted, so the order cars wrote to it changes nothing.
void Driving::announce() {
    board_.begin(tickCount_, roads_->crossings.size());
    const JunctionNumbers& n = junctionNumbers_;
    lines_.assign(cars_.size(), LineWork());
    lineTimes_.resize(cars_.size());
    lineOrder_.clear();

    // Seconds between a driver's decisions: what it is told now, it acts on at its next one.
    const Real reaction = static_cast<Real>(std::max<uint32_t>(1, settings_.decideEveryTicks)) * step_;
    // 1. Each car's own notes, and its nearest line.
    for (std::size_t i = 0; i < cars_.size(); ++i) {
        LineTimes& times = lineTimes_[i];
        for (std::pair<engine::PathId, Real>& lt : times) lt = {engine::PathId(), kNever};
        const Car& c = cars_[i];
        Trip* t = trips_.get(tripOfRecord_[i]);
        if (!c.driven || !t || !c.held.decided || c.vehicleClass >= vehicleClasses_.size()) continue;
        const VehicleClass& cls = vehicleClasses_.at(c.vehicleClass);
        JunctionPlans& plans = t->junctions;
        for (std::size_t k = 0; k < plans.count; ++k) {
            // What its turn's light shows now (a pure function of the clock; Dark where no light runs a programme): inside
            // on a red it is committed; standing at a red or a yellow it has not yet come to rest for the traffic (#261).
            const Lamp lamp = lightsRun_ ? lampAt(*roads_, rightOfWay_, plans.plans[k].connector, clockSeconds_) : Lamp::Dark;
            const bool red = lamp == Lamp::Red || lamp == Lamp::RedMayTurn;
            const bool held = lamp == Lamp::Red || lamp == Lamp::Yellow;
            noteAtTheLine(plans.plans[k], c, cls.brake, cls.hardestBrake, reaction, n.atLineWithin, tickCount_,
                          clockSeconds_, red, held);
        }
        // Granted room through junctions too close to stop between (#264): committed at the first, at all of them.
        if (roomOn_) commitThrough(plans);
        for (std::size_t k = 0; k < plans.count; ++k) {
            const JunctionPlan& p = plans.plans[k];
            if (!(p.startOdometer > c.odometer) || !p.connector.valid()) continue;
            LineWork& w = lines_[i];
            w.has = true;
            w.plan = static_cast<uint8_t>(k);
            w.toLine = p.startOdometer - c.odometer;
            w.fromLane = roads_->paths[p.connector.index].from;
            w.through = speedThrough(*roads_, roadRules_, p.connector, cls.topSpeed, n.comfortableSideAccel);
            Mover m;
            m.speed = c.speed;
            m.speedThrough = w.through;
            m.speedUp = cls.speedUp;
            m.brake = cls.brake;
            w.ownIn = stopsFirstAt(c, p, n.atLineWithin) ? secondsToStopAt(static_cast<Real>(w.toLine), m)
                                                         : secondsToCover(static_cast<Real>(w.toLine), m);
            w.in = w.ownIn;
            // Stopping short of the line for something that is not this junction's traffic.
            // ... or refused room beyond it at its last ask: it may not enter, whichever rule's stop at the line won (a
            // give-way there as well): read off the claim, so being put second for it does not make it look going again
            // (issue #284).
            w.notGoing = (c.held.hasStop && !stopIsJunctionsOwn(c, p, n.atLineWithin) &&
                          c.held.stopAt <= p.startOdometer + n.atLineWithin) ||
                         p.room.state == RoomState::Refused;
            // Stopping at or short of the line for anything but the full stop the estimate counts: the car behind it
            // comes to rest behind it, and so is never clear of anything beyond the line either (HONEST CLEAR TIMES).
            w.stopsShort = c.held.hasStop && c.held.stopAt <= p.startOdometer + n.atLineWithin &&
                           !stopsFirstAt(c, p, n.atLineWithin);
            // A car too far off to announce even by its own estimate needs no queue worked out (a queue only delays).
            if (w.ownIn <= n.lookAheadTime) lineOrder_.push_back(static_cast<uint32_t>(i));
            break;
        }
    }

    // 2. The queues, nearest the line first: no earlier than the car ahead for the same line plus a following interval;
    // not going behind a car that is not going; and never past a car standing beyond the line.
    std::sort(lineOrder_.begin(), lineOrder_.end(), [this](uint32_t a, uint32_t b) {
        return lines_[a].toLine != lines_[b].toLine ? lines_[a].toLine < lines_[b].toLine : cars_[a].id < cars_[b].id;
    });
    for (const uint32_t i : lineOrder_) {
        LineWork& w = lines_[i];
        const Car& c = cars_[i];
        const Trip* t = trips_.get(tripOfRecord_[i]);
        const std::vector<engine::PathId>& way = t->way.paths;
        pathsAfter_.clear();
        Real toEnd = roads_->paths[c.path.index].length - c.along;
        if (t->wayAt < way.size() && way[t->wayAt] == c.path)
            for (std::size_t k = t->wayAt + 1; k < way.size(); ++k) {
                pathsAfter_.push_back(way[k]);
                toEnd += roads_->paths[way[k].index].length;
            }
        // Look as far as the end of the last junction planned.
        const JunctionPlan& last = t->junctions.plans[t->junctions.count - 1];
        const Real reach = static_cast<Real>(last.startOdometer - c.odometer) + roads_->paths[last.connector.index].length;
        const Neighbour ahead = carAhead(*roads_, cars_, pathLists_, c.id, pathsAfter_, std::min(toEnd, reach));
        if (!ahead.found()) continue;
        const Car* a = findCar(cars_, pathLists_, ahead.car);
        if (!a) continue;
        const LineWork& wa = lines_[static_cast<std::size_t>(a - cars_.data())];
        const Real aheadLength =
            a->vehicleClass < vehicleClasses_.size() ? vehicleClasses_.at(a->vehicleClass).length : Real(0);
        const Real interval = static_cast<Real>(personalValue(n.gapKept, n.gapKeptHash, t->person)) +
                              aheadLength / std::max(w.through, kSlowestThrough);
        if (wa.has && wa.fromLane == w.fromLane && ahead.gap < w.toLine) {
            // Queued for the same line.
            w.in = std::max(w.in, wa.in + interval);
            w.notGoing = w.notGoing || wa.notGoing;
            w.neverClear = w.neverClear || wa.neverClear || wa.stopsShort;
        } else if (ahead.gap >= w.toLine) {
            if (a->speed < kStandingBelow) {
                // Standing beyond my line: my front gets no further than a standstill gap behind it.
                w.frontLimit = c.odometer + ahead.gap - n.standstillGap;
            } else {
                // Moving off beyond my line: it crossed the line this long ago, and I follow it by the interval, so a
                // queue's estimate does not jump earlier the moment the car in front crosses.
                const Real past = static_cast<Real>(ahead.gap - w.toLine) + aheadLength;
                w.in = std::max(w.in, interval - past / std::max(a->speed, kSlowestThrough));
            }
        }
    }

    // 3. Every car's announcements, in car order.
    for (std::size_t i = 0; i < cars_.size(); ++i) {
        const Car& c = cars_[i];
        const Trip* t = trips_.get(tripOfRecord_[i]);
        if (!c.driven || !t || !c.held.decided || c.vehicleClass >= vehicleClasses_.size()) continue;
        const VehicleClass& cls = vehicleClasses_.at(c.vehicleClass);
        const LineWork& w = lines_[i];
        Mover base;
        base.speed = c.speed;
        base.speedUp = cls.speedUp;
        base.brake = cls.brake;
        const JunctionPlans& plans = t->junctions;
        for (std::size_t k = 0; k < plans.count; ++k) {
            const JunctionPlan& p = plans.plans[k];
            if (!p.connector.valid() || p.connector.index >= roads_->paths.size()) continue;
            PlanAnnouncer a;
            a.who = c.id;
            a.odometer = c.odometer;
            a.length = cls.length;
            a.tags = tagsOf(c.vehicleClass);
            a.move = base;
            a.move.speedThrough = speedThrough(*roads_, roadRules_, p.connector, cls.topSpeed, n.comfortableSideAccel);
            a.hardestBrake = cls.hardestBrake;
            a.heldStop = heldStopOf(c);
            a.reaction = reaction;
            a.stopsFirst = stopsFirstAt(c, p, n.atLineWithin);
            const bool queued = w.has && k >= w.plan;
            a.shift = queued ? w.in - w.ownIn : Real(0);
            a.notGoing = queued && w.notGoing;
            a.neverClear = queued && w.neverClear;
            a.heldStopAt = c.held.hasStop && !stopIsJunctionsOwn(c, p, n.atLineWithin) ? c.held.stopAt
                                                                                        : static_cast<double>(kNever);
            a.frontLimit = w.frontLimit;
            // When it reaches the line; a junction beyond the look-ahead time is not announced, nor any after it.
            const Real toLine = static_cast<Real>(p.startOdometer - c.odometer);
            const Real lineIn =
                toLine <= 0 ? Real(0)
                            : (a.stopsFirst ? secondsToStopAt(toLine, a.move) : secondsToCover(toLine, a.move)) + a.shift;
            if (lineIn > n.lookAheadTime) break;
            lineTimes_[i][k] = {p.connector, lineIn};
            announcePlan(*roads_, p, a, board_);
        }
        // The lanes its body is on and its way runs along, where a lane meets a junction's connector (issue #284: read off
        // the crossing point's stretch, as a plan's car reads its line).
        LaneAnnouncer lane;
        lane.who = c.id;
        lane.odometer = c.odometer;
        lane.length = cls.length;
        lane.tags = tagsOf(c.vehicleClass);
        lane.move = base;
        lane.move.speedThrough = c.held.topSpeed > 0 ? c.held.topSpeed : cls.topSpeed;
        lane.hardestBrake = cls.hardestBrake;
        lane.reaction = reaction;
        lane.lookAheadTime = n.lookAheadTime;
        lane.atLineWithin = n.atLineWithin;
        lane.heldStop = heldStopOf(c);
        lane.heldStopFor = c.held.stopFor;
        Real start = -c.along;
        for (std::size_t k = 0; k < c.trailCount && k < kMaxTrail; ++k) {
            start -= roads_->paths[c.trail[k].index].length;
            if (roads_->paths[c.trail[k].index].kind == engine::PathKind::Lane)
                announceLane(*roads_, c.trail[k], start, lane, board_);
        }
        start = -c.along;
        if (roads_->paths[c.path.index].kind == engine::PathKind::Lane) announceLane(*roads_, c.path, start, lane, board_);
        start += roads_->paths[c.path.index].length;
        const std::vector<engine::PathId>& way = t->way.paths;
        if (t->wayAt < way.size() && way[t->wayAt] == c.path)
            for (std::size_t k = t->wayAt + 1; k < way.size(); ++k) {
                if (secondsToCover(start, lane.move) > n.lookAheadTime) break;
                if (roads_->paths[way[k].index].kind == engine::PathKind::Lane)
                    announceLane(*roads_, way[k], start, lane, board_);
                start += roads_->paths[way[k].index].length;
            }
    }
    board_.seal();
    // Who goes first wherever two announced cars meet: settled once, from the sealed board, for every driver (#263). A
    // car behind another on one path (the next in its list: a front, or a tail still on it) can never go before it: an
    // arrow no answer can turn round, so a ring through it is broken at an answer, never through the queue. Read from
    // where the cars are, not from what binds them, so the arrow does not come and go as the car ahead moves off.
    behind_.clear();
    for (uint32_t p = 0; p < roads_->paths.size(); ++p) {
        const PathList list = pathLists_.on(engine::PathId(p));
        for (std::size_t k = 0; k + 1 < list.size(); ++k)
            if (list[k].car != list[k + 1].car && list[k].why != OnPath::Beside && list[k + 1].why != OnPath::Beside)
                behind_.push_back({list[k + 1].car.index, list[k].car.index});
    }
    resolver_.study(*roads_, rightOfWay_, board_, behind_, clockSeconds_);
    addCounts(conflictTotals_, resolver_.counts());
}

// PLAN: each driven car whose turn it is (or that has not decided yet on this trip) notices its facts along its way
// ahead and decides by the rules. Every decision reads the cars as they stood at the start of the tick (and this tick's
// board); the answers are applied only once all are made.
void Driving::plan() {
    findTripsOfRecords();
    if (junctionsOn_) announce();
    const uint32_t every = std::max<uint32_t>(1, settings_.decideEveryTicks);
    planned_.clear();
    for (std::size_t i = 0; i < cars_.size(); ++i) {
        const Car& c = cars_[i];
        const Trip* t = trips_.get(tripOfRecord_[i]);
        if (!c.driven || !t || c.vehicleClass >= vehicleClasses_.size()) continue;
        if (c.held.decided && tickCount_ % every != c.id.index % every) continue;
        const VehicleClass& cls = vehicleClasses_.at(c.vehicleClass);

        Planned p;
        p.record = i;
        p.trip = tripOfRecord_[i];
        PathPoint front;
        front.path = c.path;
        front.along = c.along;
        p.legReached = t->legReached;
        const RoutePlace place = placeOnRoute(t->route, *roads_, front, t->legReached);
        if (place.part == RoutePart::Leg || place.part == RoutePart::Turn) p.legReached = place.index;
        // How far it looks: far enough to stop, and (with the junction step) as far as it travels in its junction
        // look-ahead time, so a junction is planned for, and announced, that long before the car gets there.
        Real lookAhead = lookAheadDistance(c.speed, cls.brake);
        if (junctionsOn_) {
            lookAhead = std::max(lookAhead, c.speed * junctionNumbers_.lookAheadTime);
            // ... and never short of the junctions already planned for: the look-ahead shrinks as a car slows, and a plan
            // dropped and made again would forget what the driver decided there.
            for (std::size_t k = 0; k < t->junctions.count; ++k) {
                const JunctionPlan& jp = t->junctions.plans[k];
                if (!jp.connector.valid() || jp.connector.index >= roads_->paths.size()) continue;
                lookAhead = std::max(lookAhead, static_cast<Real>(jp.startOdometer - c.odometer) +
                                                    roads_->paths[jp.connector.index].length);
            }
        }
        // The way ahead, and the junctions on it: plans kept, dropped and made before the driver notices anything. With
        // room beyond (#264), as far again as a claim at the nearest junction needs: through each junction its body
        // would not fit before (room.h roomReach; at most room_through_junctions more looks).
        for (uint32_t look = 0;; ++look) {
            p.way = wayAhead(t->route, *roads_, c.path, c.along, lookAhead, p.legReached);
            p.junctions = t->junctions;
            p.planUpdate = PlanUpdate();
            updateJunctionPlans(*roads_, rightOfWay_, c, cls.length, p.way, tickCount_, clockSeconds_, p.junctions,
                                p.planUpdate, tagsOf(c.vehicleClass));
            if (!roomOn_ || look >= roomNumbers_.throughJunctions) break;
            const Real further = roomReach(*roads_, roomRoads_, p.junctions, c, p.way,
                                           roomNeeded(cls, roomNumbers_.standstillGap), roomNumbers_);
            if (!(further > lookAhead)) break;
            lookAhead = further;
        }
        Real reach = -c.along;
        for (const engine::PathId path : p.way.paths) reach += roads_->paths[path.index].length;
        p.wayEnds = reach < lookAhead && p.way.paths.size() < kMaxFollowedPaths;
        if (reach < lookAhead && p.way.paths.size() >= kMaxFollowedPaths) {
            // Cut short by the cap on paths followed, not by how far the driver looks: counted, never silent.
            ++capCounts_.lookAheadsCut;
            capCounts_.lastLookAheadCut = c.id;
        }
        // ... and whether it must give way at them, judged from this tick's board.
        if (junctionsOn_) {
            GiveWayInputs in{*roads_,   rightOfWay_, ruleSet_,   roadRules_,
                             board_,    resolver_,   junctionNumbers_, c,
                             cls.length, cls.brake,  static_cast<Real>(every) * step_, t->person,
                             clockSeconds_, p.junctions, {}, lightsRun_ ? &lightTiming_ : nullptr, cls.hardestBrake};
            in.way = &p.way;   // the lanes on it judge their crossing points too (issue #284)
            for (std::size_t k = 0; k < kMaxJunctionPlans; ++k) {
                in.lineIn[k] = kNever;
                if (k >= p.junctions.count) continue;
                for (const std::pair<engine::PathId, Real>& lt : lineTimes_[i])
                    if (lt.first == p.junctions.plans[k].connector) in.lineIn[k] = lt.second;
            }
            judgeGiveWay(in, p.verdict);
            p.lineIn = in.lineIn;
            // Room beyond (#264): deciding to go at a junction it holds no room at is asking for it; refused at its last
            // ask, it waits at that line (its facts read where).
            if (roomOn_) {
                p.roomAsk = roomToAsk(p.junctions, c, p.verdict, roomNumbers_, &p.roomAskInside);
                p.roomStop = roomStopFor(p.junctions, c, roomNumbers_);
            }
        }

        DriverView view{c,          cls,       t->person,  p.way,         *roads_,     cars_,     pathLists_,
                        ruleSet_,   roadRules_, lookAhead, &t->destination, &t->holds, &p.junctions, &rightOfWay_};
        if (junctionsOn_) {
            view.board = &board_;
            view.giveWay = &p.verdict;
        }
        if (roomOn_) view.room = &p.roomStop;
        // What it decides now it holds until its next turn to decide (#281: a bend's speed allows for that).
        view.decideInterval = static_cast<Real>(every) * step_;
        noticeFacts(view, facts_, wanted_, noticed_, noticeScratch_);
        const NoticedFact& arrived = noticed_[arrivedSlot_];
        if (arrived.present && arrived.yes) {
            p.arrived = true;
            planned_.push_back(std::move(p));
            continue;
        }
        decide(rules_, noticed_, ruleSet_, t->person, decision_);
        p.held.decided = true;
        if (decision_.stop.given) {
            p.held.hasStop = true;
            p.held.stopAt = c.odometer + decision_.stop.distance;
            p.held.stopRule = decision_.stop.why.rule;
            // What the stop waits for: the place fact that gave it says (a give-way line waits for the car given way to).
            const RuleIndex r = decision_.stop.why.rule;
            const FactSlot slot = r < rules_.program.rules.size() ? rules_.program.rules[r].stopAt : kNoFact;
            if (slot < facts_.size()) {
                p.held.stopFor = facts_.at(slot).stopFor;
                // Giving way to a body already on the crossing point: a matter of space (issue #284; wait_graph.h).
                if (p.held.stopFor == StopFor::GivingWay && noticed_[slot].forABody) p.held.stopFor = StopFor::ForABody;
                if (givesWay(p.held.stopFor) || p.held.stopFor == StopFor::NoRoom)
                    p.held.stopCar = noticed_[slot].aboutCar;
            }
        }
        // The top speed to aim for: the rules' when it is below what the car can do, else the class's own.
        p.held.topSpeed = cls.topSpeed;
        p.held.topSpeedRule = kNoRule;
        if (decision_.topSpeed.given && decision_.topSpeed.speed < cls.topSpeed) {
            p.held.topSpeed = std::max(Real(0), decision_.topSpeed.speed);
            p.held.topSpeedRule = decision_.topSpeed.why.rule;
        }
        if (settings_.trace) {
            // The reasons behind what it will hold, copied out before the next driver's decision reuses decision_.
            if (p.held.hasStop) p.stopWhy = keepReason(decision_, decision_.stop.why);
            if (p.held.topSpeedRule != kNoRule) p.topSpeedWhy = keepReason(decision_, decision_.topSpeed.why);
        }
        planned_.push_back(std::move(p));
    }

    for (Planned& p : planned_) {
        Car& c = cars_[p.record];
        Trip* t = trips_.get(p.trip);
        if (p.arrived) {
            // Undriven from here on; its speed (a crawl at most) is set to rest by the move step, which writes the
            // copy the other cars read next tick.
            queueEvent(EventKind::Parked, c.id, p.trip, t->person);
            c.driven = false;
            c.held = HeldAnswers();
            c.binding = Binding();
            tripOfCar_.erase(c.id);
            trips_.erase(p.trip);
            continue;
        }
        // A stop on a lane to give way (issue #284), counted as it begins: the stop the car now holds, not before.
        if (p.verdict.onALane && p.held.hasStop && givesWay(p.held.stopFor) &&
            !(c.held.hasStop && givesWay(c.held.stopFor)))
            ++planCounts_.gaveWayOnALane;
        c.held = p.held;
        t->junctions = p.junctions;
        notePlanUpdate(p.planUpdate, c.id, *t, t->junctions);
        applyVerdict(p.verdict, p.lineIn, c, *t);
        if (roomOn_) askForRoom(t->junctions, p.roomAsk, p.roomAskInside, tickCount_);
        t->way = std::move(p.way);
        t->wayAt = 0;
        t->wayEnds = p.wayEnds;
        t->legReached = p.legReached;
        if (settings_.trace) {
            t->trace.heldStop = p.stopWhy;
            t->trace.heldTopSpeed = p.topSpeedWhy;
        }
        // Refused room for long at the junction it waits at: it asks for another way (issue #284). Visited in car order,
        // after its own decision is applied; a route found changes only this car's trip.
        if (roomOn_) askAnotherWay(c, *t);
    }
}

// MOVE: every driven car's new speed from its held answers and a fresh look at the car ahead (act.h), then along its
// held way. Reads cars_ (the state at the start of the step), writes carsNext_; the two then swap.
void Driving::move(Real dt) {
    findTripsOfRecords();
    carsNext_ = cars_;
    const uint32_t every = std::max<uint32_t>(1, settings_.decideEveryTicks);
    const Real reaction = reactionTime(static_cast<Real>(every) * dt, dt);

    for (std::size_t i = 0; i < cars_.size(); ++i) {
        const Car& c = cars_[i];
        Trip* t = trips_.get(tripOfRecord_[i]);
        if (!c.driven) {
            // Nobody drives it: it stands (a car that has just arrived comes to rest from its last crawl).
            if (c.speed != 0) leaveParked(carsNext_[i]);
            continue;
        }
        if (!t || !c.held.decided || c.vehicleClass >= vehicleClasses_.size()) continue;
        const VehicleClass& cls = vehicleClasses_.at(c.vehicleClass);
        const std::vector<engine::PathId>& way = t->way.paths;
        const bool onWay = t->wayAt < way.size() && way[t->wayAt] == c.path;

        // The way from the car's front on, and how far it is to its end.
        pathsAfter_.clear();
        Real toEnd = roads_->paths[c.path.index].length - c.along;
        if (onWay) {
            for (std::size_t k = t->wayAt + 1; k < way.size(); ++k) {
                pathsAfter_.push_back(way[k]);
                toEnd += roads_->paths[way[k].index].length;
            }
        }

        // What is ahead: the car ahead (looked at afresh), the place to stop by it holds, the end of its way.
        Obstacle ahead[kMaxObstacles];
        std::size_t count = 0;
        const Neighbour n = carAhead(*roads_, cars_, pathLists_, c.id, pathsAfter_, toEnd);
        if (n.found()) {
            Obstacle& o = ahead[count++];
            o.kind = Limit::CarAhead;
            o.distance = n.gap;
            o.speed = n.speed;
            o.car = n.car;
            const Car* other = findCar(cars_, pathLists_, n.car);
            o.hardestBrake = other && other->vehicleClass < vehicleClasses_.size()
                                 ? vehicleClasses_.at(other->vehicleClass).hardestBrake
                                 : Real(0);
        }
        if (c.held.hasStop) {
            Obstacle& o = ahead[count++];
            o.kind = Limit::Stop;
            o.distance = static_cast<Real>(c.held.stopAt - c.odometer);
            o.rule = c.held.stopRule;
            o.car = c.held.stopCar;
            o.stopFor = c.held.stopFor;
        }
        {
            Obstacle& o = ahead[count++];
            o.kind = Limit::WayEnd;
            o.distance = toEnd;
            o.hardOnly = !(onWay && t->wayEnds);
        }

        Follower f;
        f.speedUp = cls.speedUp;
        f.brake = cls.brake;
        f.hardestBrake = cls.hardestBrake;
        f.gapKept = static_cast<Real>(personalValue(gapKept_, gapKeptHash_, t->person));
        f.standstillGap = standstillGap_;
        f.anticipation = anticipation_;
        const StepOutcome step = stepSpeed(f, c.speed, c.held.topSpeed, c.held.topSpeedRule, ahead, count, dt, reaction);

        Car& out = carsNext_[i];
        out.speed = step.speed;
        out.acceleration = step.acceleration;
        out.binding = step.binding;

        // Along the way: carry what remains past the end of a path onto the next, as many times as it takes. The end
        // of the way is a wall; the hard limit keeps the car short of it, so stopping there is only ever rounding (or
        // a stop the hardest braking could not make).
        Real along = c.along + step.speed * dt;
        Real moved = step.speed * dt;
        engine::PathId path = c.path;
        std::size_t at = t->wayAt;
        std::array<engine::PathId, kMaxTrail> crossed;   // the paths passed this step, latest first (the last few)
        std::size_t crossedCount = 0;
        while (along > roads_->paths[path.index].length) {
            const Real length = roads_->paths[path.index].length;
            if (!onWay || at + 1 >= way.size()) {
                moved -= along - length;
                along = length;
                break;
            }
            along -= length;
            // Keep the latest kMaxTrail paths crossed: shift the older ones back.
            for (std::size_t k = std::min(crossedCount, kMaxTrail - 1); k > 0; --k) crossed[k] = crossed[k - 1];
            crossed[0] = path;
            crossedCount = std::min(crossedCount + 1, kMaxTrail);
            ++at;
            path = way[at];
        }
        out.path = path;
        out.along = along;
        out.odometer = c.odometer + moved;
        if (onWay) t->wayAt = static_cast<uint32_t>(at);

        // The trail: the paths just passed, latest first, then the old trail; only those the body still reaches.
        std::array<engine::PathId, kMaxTrail> trail;
        std::size_t trailCount = 0;
        for (std::size_t k = 0; k < crossedCount && trailCount < kMaxTrail; ++k) trail[trailCount++] = crossed[k];
        for (std::size_t k = 0; k < c.trailCount && trailCount < kMaxTrail; ++k) trail[trailCount++] = c.trail[k];
        out.trail = trail;
        out.trailCount = static_cast<uint8_t>(trailCount);
        const Body body = layBody(*roads_, out, cls.length);
        out.trailCount = static_cast<uint8_t>(body.count > 0 ? body.count - 1 : 0);
        if (body.unplaced > 0) {
            // The body lies on more paths than the trail holds. The lists then extend its oldest piece back by the rest
            // (rebuildPathLists), so followers stay clear of all of it; counted, never silent.
            ++capCounts_.tailsOffTrail;
            capCounts_.lastTailOffTrail = c.id;
        }
        for (std::size_t k = out.trailCount; k < kMaxTrail; ++k) out.trail[k] = engine::PathId();

        // THE TRACE: a new entry only when what limits the car changed (or its trip's first). Only written, never read
        // by the tick.
        if (settings_.trace) {
            const TraceEntry* last = t->trace.latest();
            if (!last || last->binding != out.binding) {
                TraceEntry e;
                e.tick = tickCount_ + 1;
                e.seconds = clockSeconds_ + dt;
                e.binding = out.binding;
                e.speed = out.speed;
                e.path = out.path;
                e.along = out.along;
                if (out.binding.by == Limit::Stop && out.binding.rule == t->trace.heldStop.rule)
                    e.why = t->trace.heldStop;
                else if (out.binding.by == Limit::TopSpeed && out.binding.rule != kNoRule &&
                         out.binding.rule == t->trace.heldTopSpeed.rule)
                    e.why = t->trace.heldTopSpeed;
                else
                    e.why.rule = out.binding.rule;
                t->trace.add(e);
            }
        }
    }

    cars_.swap(carsNext_);
    rebuildPathLists(*roads_, vehicleClasses_, cars_, pathLists_);
}

void Driving::setVehicleClasses(VehicleClasses classes) {
    vehicleClasses_ = std::move(classes);
    routeClasses_.clear();
    routeNotes_.clear();
    if (roads_) {
        for (const VehicleClass& c : vehicleClasses_.all())
            routeClasses_.push_back(bindRouteClass(*roads_, c, &routeNotes_));
        rebuildPathLists(*roads_, vehicleClasses_, cars_, pathLists_);
        closeTurnsForRoom();
    }
}

Route Driving::routeFor(const Car& c, const PathPoint& to) const {
    PathPoint front;
    front.path = c.path;
    front.along = c.along;
    return findRoute(*roads_, routeBounds_, routeClass(c.vehicleClass), front, to);
}

void Driving::queueEvent(EventKind kind, CarId car, TripId trip, PersonId person) {
    Event e;
    e.kind = kind;
    e.car = car;
    e.trip = trip;
    e.person = person;
    e.tick = tickCount_;
    events_.push_back(e);
}

Car* Driving::findPlaced(CarId id) {
    std::vector<Car>::iterator it =
        std::lower_bound(cars_.begin(), cars_.end(), id, [](const Car& c, CarId want) { return c.id < want; });
    return it != cars_.end() && it->id == id ? &*it : nullptr;
}

const Car* Driving::car(CarId id) const {
    std::vector<Car>::const_iterator it =
        std::lower_bound(cars_.begin(), cars_.end(), id, [](const Car& c, CarId want) { return c.id < want; });
    return it != cars_.end() && it->id == id ? &*it : nullptr;
}

bool Driving::placeCar(CarId id, uint32_t vehicleClass, const PathPoint& front, const std::vector<engine::PathId>& trail,
                       std::string* why) {
    if (!id.valid()) return refuse(why, "no car id given");
    if (car(id)) return refuse(why, carName(id) + " is already placed");
    if (vehicleClass >= vehicleClasses_.size())
        return refuse(why, carName(id) + ": no vehicle class " + std::to_string(vehicleClass) + " (there are " +
                               std::to_string(vehicleClasses_.size()) + ")");
    if (!roads_ || !front.path.valid() || front.path.index >= roads_->paths.size())
        return refuse(why, carName(id) + ": there is no " + pathName(front.path) + " on the road");
    if (!onRoad(front))
        return refuse(why, carName(id) + ": " + std::to_string(front.along) + " m is off " + pathName(front.path) +
                               ", which is " + std::to_string(roads_->paths[front.path.index].length) + " m long");
    if (trail.size() > kMaxTrail)
        return refuse(why, carName(id) + ": a trail of " + std::to_string(trail.size()) + " paths is more than the " +
                               std::to_string(kMaxTrail) + " a car may lie on behind its front");
    engine::PathId ahead = front.path;
    for (const engine::PathId p : trail) {
        if (!p.valid() || p.index >= roads_->paths.size())
            return refuse(why, carName(id) + ": its trail names " + pathName(p) + ", which is not on the road");
        if (!leadsInto(*roads_, p, ahead))
            return refuse(why, carName(id) + ": its trail's " + pathName(p) + " does not lead into " + pathName(ahead));
        ahead = p;
    }

    Car c;
    c.id = id;
    c.vehicleClass = vehicleClass;
    c.path = front.path;
    c.along = front.along;
    c.trailCount = static_cast<uint8_t>(trail.size());
    for (std::size_t i = 0; i < trail.size(); ++i) c.trail[i] = trail[i];
    const Real length = vehicleClasses_.at(vehicleClass).length;
    const Body body = layBody(*roads_, c, length);
    if (body.unplaced > 0) {
        if (trail.size() == kMaxTrail)
            return refuse(why, carName(id) + ": its body (" + std::to_string(length) + " m) reaches back over more than " +
                                   std::to_string(kMaxTrail) + " paths behind its front, the most a car may lie on");
        return refuse(why, carName(id) + ": its body (" + std::to_string(length) + " m) does not fit on " +
                               pathName(front.path) + " and its trail: " + std::to_string(body.unplaced) +
                               " m has nowhere to lie");
    }
    // Keep only the trail paths the body reaches.
    c.trailCount = static_cast<uint8_t>(body.count - 1);
    for (std::size_t i = c.trailCount; i < kMaxTrail; ++i) c.trail[i] = engine::PathId();

    std::vector<Car>::iterator at =
        std::lower_bound(cars_.begin(), cars_.end(), id, [](const Car& x, CarId want) { return x.id < want; });
    cars_.insert(at, c);
    rebuildPathLists(*roads_, vehicleClasses_, cars_, pathLists_);
    return true;
}

bool Driving::removeCar(CarId id, std::string* why) {
    if (!car(id)) return refuse(why, carName(id) + " is not placed");
    if (tripOfCar_.count(id) != 0) return refuse(why, carName(id) + " is on a trip: end the trip first");
    std::vector<Car>::iterator at =
        std::lower_bound(cars_.begin(), cars_.end(), id, [](const Car& x, CarId want) { return x.id < want; });
    cars_.erase(at);
    if (roads_) rebuildPathLists(*roads_, vehicleClasses_, cars_, pathLists_);
    return true;
}

TripId Driving::beginTrip(CarId car, PersonId person, const PathPoint& destination) {
    Car* placed = findPlaced(car);
    if (!placed || tripOfCar_.count(car) != 0 || !onRoad(destination)) return TripId{};
    Route route = routeFor(*placed, destination);
    if (!route.found) {
        queueEvent(EventKind::NoRoute, car, TripId{}, person);
        return TripId{};
    }
    placed->driven = true;
    placed->held = HeldAnswers();   // nothing decided yet: it decides on its next tick
    placed->binding = Binding();
    Trip t;
    t.car = car;
    t.person = person;
    t.destination = destination;
    t.beganAtTick = tickCount_;
    t.route = std::move(route);
    const TripId id = trips_.insert(t);
    tripOfCar_[car] = id;
    return id;
}

const Trip* Driving::trip(TripId id) const { return trips_.get(id); }

const Route* Driving::route(TripId id) const {
    const Trip* t = trips_.get(id);
    return t ? &t->route : nullptr;
}

bool Driving::changeDestination(TripId id, const PathPoint& destination) {
    Trip* t = trips_.get(id);
    if (!t || !onRoad(destination)) return false;
    const Car* c = car(t->car);
    if (!c) return false;
    Route route = routeFor(*c, destination);
    if (!route.found) {
        queueEvent(EventKind::NoRoute, t->car, id, t->person);
        return false;
    }
    t->destination = destination;
    t->route = std::move(route);
    t->legReached = 0;
    findPlaced(t->car)->held.decided = false;   // decides again on its next tick, on the new route
    dropPlansForNewRoute(*t);
    return true;
}

void Driving::dropPlansForNewRoute(Trip& t) {
    PlanUpdate u;
    dropJunctionPlans(tickCount_, clockSeconds_, PlanChange::RouteChanged, t.junctions, u);
    notePlanUpdate(u, t.car, t, t.junctions);
}

void Driving::askAnotherWay(Car& c, Trip& t) {
    if (!(roomNumbers_.anotherWayAfter > 0) || !(step_ > 0)) return;
    const double after = static_cast<double>(roomNumbers_.anotherWayAfter);
    if (t.askedAnotherWayTick > 0 && static_cast<double>(tickCount_ - t.askedAnotherWayTick) * step_ < after) return;
    // The nearest junction whose line its front has not passed: refused there, having asked that long?
    const JunctionPlan* refused = nullptr;
    for (std::size_t k = 0; k < t.junctions.count && k < kMaxJunctionPlans; ++k) {
        const JunctionPlan& p = t.junctions.plans[k];
        if (!p.connector.valid() || p.startOdometer - c.odometer < -roomNumbers_.atLineWithin) continue;
        if (p.room.state == RoomState::Refused && p.room.asked &&
            static_cast<double>(tickCount_ - p.room.askedTick) * step_ >= after)
            refused = &p;
        break;
    }
    if (!refused) return;
    t.askedAnotherWayTick = tickCount_;
    ++planCounts_.askedAnotherWay;
    // A way from where it is that does not take that turn (closed for this one search only).
    RouteClass cls = routeClass(c.vehicleClass);
    const std::vector<engine::PathId>::iterator at = std::lower_bound(cls.closed.begin(), cls.closed.end(), refused->connector);
    if (at == cls.closed.end() || *at != refused->connector) cls.closed.insert(at, refused->connector);
    PathPoint front;
    front.path = c.path;
    front.along = c.along;
    Route route = findRoute(*roads_, routeBounds_, cls, front, t.destination);
    if (!route.found) return;   // no other way: it keeps its own and waits on (the measurements report a wait that cannot end)
    // ... that it can take now: the first turn of the new way, where lights run there, shows green (a driver gives up its
    // turn for one it sees it may take, not for another red; otherwise it waits on and asks again later).
    const engine::PathId firstTurn = !route.turns.empty() ? route.turns[0].connector : route.endConnector;
    if (lightsRun_ && firstTurn.valid() && lampAt(*roads_, rightOfWay_, firstTurn, clockSeconds_) != Lamp::Green &&
        lampAt(*roads_, rightOfWay_, firstTurn, clockSeconds_) != Lamp::Dark)
        return;
    route.vehicle = routeClass(c.vehicleClass);   // later questions of the route see the class as it is
    ++planCounts_.tookAnotherWay;
    t.route = std::move(route);
    t.legReached = 0;
    c.held.decided = false;   // decides again on its next tick, on the new route
    dropPlansForNewRoute(t);
}

bool Driving::reroute(TripId id) {
    Trip* t = trips_.get(id);
    if (!t) return false;
    const Car* c = car(t->car);
    if (!c) return false;
    Route route = routeFor(*c, t->destination);
    if (!route.found) {
        queueEvent(EventKind::NoRoute, t->car, id, t->person);
        return false;
    }
    t->route = std::move(route);
    t->legReached = 0;
    findPlaced(t->car)->held.decided = false;   // decides again on its next tick, on the new route
    dropPlansForNewRoute(*t);
    return true;
}

bool Driving::placeHold(TripId id, HoldName name, const PathPoint& at) {
    Trip* t = trips_.get(id);
    if (!t || !onRoad(at)) return false;
    std::vector<Hold>::iterator it = std::lower_bound(t->holds.begin(), t->holds.end(), name,
                                                      [](const Hold& h, HoldName n) { return h.name < n; });
    if (it != t->holds.end() && it->name == name) {
        it->at = at;
    } else {
        Hold h;
        h.name = name;
        h.at = at;
        t->holds.insert(it, h);
    }
    return true;
}

bool Driving::releaseHold(TripId id, HoldName name) {
    Trip* t = trips_.get(id);
    if (!t) return false;
    std::vector<Hold>::iterator it = std::lower_bound(t->holds.begin(), t->holds.end(), name,
                                                      [](const Hold& h, HoldName n) { return h.name < n; });
    if (it == t->holds.end() || it->name != name) return false;
    t->holds.erase(it);
    return true;
}

bool Driving::endTrip(TripId id) {
    const Trip* t = trips_.get(id);
    if (!t) return false;
    Event e;
    e.kind = EventKind::EndedEarly;
    e.car = t->car;
    e.trip = id;
    e.person = t->person;
    e.tick = tickCount_;
    events_.push_back(e);
    // The host has taken the car: it stands where it is, undriven (an undriven car never moves, so it must not keep a
    // speed the cars behind would read).
    if (Car* c = findPlaced(t->car)) leaveParked(*c);
    tripOfCar_.erase(t->car);
    trips_.erase(id);
    return true;
}

bool Driving::atRoadEnd(CarId id) const {
    const Car* c = car(id);
    if (!c || !c->driven || !roads_ || c->speed >= kStandingBelow) return false;
    const engine::Path& p = roads_->paths[c->path.index];
    return p.next.count == 0 && p.length - c->along <= kArrivedWithin;
}

std::vector<Event> Driving::drainEvents() {
    std::vector<Event> out;
    out.swap(events_);
    std::sort(out.begin(), out.end(), eventBefore);
    return out;
}

uint64_t Driving::stateHash() const {
    engine::bundle::Fnv h;
    h.pod(tickCount_);
    h.pod(clockSeconds_);
    // The road: which graph, by its size (the graph is fixed after load; hashing it whole every tick would cost more
    // than everything else here).
    const uint64_t pathCount = roads_ ? roads_->paths.size() : 0;
    h.pod(pathCount);
    // The cars, in car-id order (cars_ is kept in it).
    h.pod(static_cast<uint64_t>(cars_.size()));
    for (const Car& c : cars_) {
        h.pod(c.id.index);
        h.pod(c.vehicleClass);
        h.pod(c.path.index);
        h.pod(c.besidePath.index);
        h.pod(c.along);
        h.pod(c.offset);
        h.pod(c.speed);
        h.pod(c.acceleration);
        h.pod(static_cast<uint8_t>(c.driven));
        h.pod(c.trailCount);
        for (std::size_t i = 0; i < c.trailCount; ++i) h.pod(c.trail[i].index);
        h.pod(c.odometer);
        h.pod(static_cast<uint8_t>(c.held.decided));
        h.pod(static_cast<uint8_t>(c.held.hasStop));
        h.pod(c.held.stopAt);
        h.pod(c.held.stopRule);
        h.pod(static_cast<uint8_t>(c.held.stopFor));
        h.pod(c.held.stopCar.index);
        h.pod(c.held.topSpeed);
        h.pod(c.held.topSpeedRule);
        h.pod(static_cast<uint8_t>(c.binding.by));
        h.pod(c.binding.car.index);
        h.pod(c.binding.rule);
        h.pod(static_cast<uint8_t>(c.binding.stopFor));
    }
    h.pod(settings_.decideEveryTicks);
    h.pod(static_cast<uint8_t>(settings_.laneChanges));
    h.pod(capCounts_.tailsOffTrail);
    h.pod(capCounts_.lastTailOffTrail.index);
    h.pod(capCounts_.lookAheadsCut);
    h.pod(capCounts_.lastLookAheadCut.index);
    h.pod(static_cast<uint8_t>(junctionsOn_));
    h.pod(static_cast<uint8_t>(roomOn_));
    // The trips, in car order.
    h.pod(static_cast<uint64_t>(tripOfCar_.size()));
    for (const std::pair<const CarId, TripId>& entry : tripOfCar_) {
        const Trip* t = trips_.get(entry.second);
        h.pod(entry.second.index);
        h.pod(entry.second.generation);
        h.pod(t->car.index);
        h.pod(t->person);
        h.pod(t->destination.path.index);
        h.pod(t->destination.along);
        h.pod(t->beganAtTick);
        h.pod(static_cast<uint64_t>(t->holds.size()));
        for (const Hold& hold : t->holds) {
            h.pod(hold.name);
            h.pod(hold.at.path.index);
            h.pod(hold.at.along);
        }
        h.pod(routeHash(t->route));
        h.pod(t->legReached);
        h.pod(t->askedAnotherWayTick);
        h.pod(t->wayAt);
        h.pod(static_cast<uint8_t>(t->wayEnds));
        h.pod(static_cast<uint64_t>(t->way.paths.size()));
        for (const engine::PathId p : t->way.paths) h.pod(p.index);
        // The junction plans: with the junction step on, what a car does reads them (what it remembers of the line, and
        // what it decided there).
        if (junctionsOn_) h.pod(junctionPlansHash(t->junctions));
    }
    // The events not yet drained, in the order they will come out.
    std::vector<Event> queued = events_;
    std::sort(queued.begin(), queued.end(), eventBefore);
    h.pod(static_cast<uint64_t>(queued.size()));
    for (const Event& e : queued) {
        h.pod(static_cast<uint8_t>(e.kind));
        h.pod(e.car.index);
        h.pod(e.trip.index);
        h.pod(e.trip.generation);
        h.pod(e.person);
        h.pod(e.tick);
    }
    return h.h;
}

// ---- Seeing why --------------------------------------------------------------------------------------------------------

TripId Driving::tripOf(CarId car) const {
    const std::map<CarId, TripId>::const_iterator it = tripOfCar_.find(car);
    return it == tripOfCar_.end() ? TripId{} : it->second;
}

const CarTrace* Driving::trace(CarId car) const {
    const Trip* t = trips_.get(tripOf(car));
    return t ? &t->trace : nullptr;
}

uint64_t Driving::traceHash() const {
    engine::bundle::Fnv h;
    h.pod(static_cast<uint64_t>(tripOfCar_.size()));
    for (const std::pair<const CarId, TripId>& entry : tripOfCar_) {
        const Trip* t = trips_.get(entry.second);
        h.pod(entry.first.index);
        h.pod(t ? driving::traceHash(t->trace) : uint64_t(0));
    }
    return h.h;
}

WaitReport Driving::waitReport() const {
    std::vector<WaitingCar> waiting = waitingCars(cars_);
    // A reason named that no longer applies is no reason (issue #284): such a car counts as waiting for none.
    for (WaitingCar& w : waiting)
        if (w.why.sort != ReasonSort::None) {
            const Car* c = car(w.car);
            if (c && !waitReasonHolds(*c)) w.why = WaitReason();
        }
    WaitReport r = analyseWaiting(std::move(waiting));
    for (const Car& c : cars_)
        if (c.driven) ++r.driven;
    return r;
}

bool Driving::waitCanNeverEnd(const Car& c, std::string* why) const {
    if (!roads_ || !roomOn_ || !isWaiting(c) || c.binding.by != Limit::Stop || c.binding.stopFor != StopFor::NoRoom)
        return false;
    const std::map<CarId, TripId>::const_iterator tripIt = tripOfCar_.find(c.id);
    const Trip* t = tripIt != tripOfCar_.end() ? trips_.get(tripIt->second) : nullptr;
    if (!t || c.vehicleClass >= routeClasses_.size()) return false;
    const RouteClass& cls = routeClasses_[c.vehicleClass];
    for (std::size_t k = 0; k < t->junctions.count && k < kMaxJunctionPlans; ++k) {
        const JunctionPlan& p = t->junctions.plans[k];
        if (!p.connector.valid() || p.startOdometer - c.odometer < -roomNumbers_.atLineWithin) continue;
        if (p.room.state != RoomState::Refused) return false;
        // (Words only when asked: this runs for every refused car every tick the measurements watch.)
        const auto say = [&](const std::string& what) {
            if (why)
                *why = "at junction " + std::to_string(p.junction.index) + " by connector " +
                       std::to_string(p.connector.index) + ": " + what;
            return true;
        };
        if (p.room.refusal == RoomRefusal::TooShort || p.room.refusal == RoomRefusal::MustStopAhead)
            return say(roomRefusalWords(p.room.refusal));
        if (std::binary_search(cls.closed.begin(), cls.closed.end(), p.connector))
            return say("a turn closed to its class for want of room");
        if (p.room.refusal != RoomRefusal::LightsAhead) return false;
        std::array<engine::PathId, kMaxJunctionPlans> chain{};
        std::size_t n = 0;
        for (std::size_t j = k; j < t->junctions.count && j < kMaxJunctionPlans && n < std::max<std::size_t>(p.room.through, 2);
             ++j)
            chain[n++] = t->junctions.plans[j].connector;
        // The claim's chain, or any longer chain through the plans the car has (the claim may have stopped short).
        for (std::size_t len = 2; len <= n; ++len) {
            bool never = false;
            if (cls.closedChains.closedChain(chain.data(), len, &never) && never)
                return say("the lights on the way on through the next junctions never let it through");
        }
        return false;
    }
    return false;
}

bool Driving::waitReasonHolds(const Car& c, std::string* why) const {
    const auto no = [&](const std::string& reason) {
        if (why) *why = reason;
        return false;
    };
    if (!roads_) return no("no road");
    const Binding& b = c.binding;
    if (b.by == Limit::Nothing) return no("held by nothing");
    const std::map<CarId, TripId>::const_iterator tripIt = tripOfCar_.find(c.id);
    const Trip* t = tripIt != tripOfCar_.end() ? trips_.get(tripIt->second) : nullptr;
    if (!t) return no("on no trip");
    const auto knownPath = [](const engine::LaneGraph& g, engine::PathId p) { return p.valid() && p.index < g.paths.size(); };
    // The paths it knows its way along, from where its front is: the named car must be on one of them (or on a path
    // leaving the same lane as one of them: the car ahead at a split, traffic_state.h carAhead).
    const auto onMyWay = [&](const Car& other) {
        const auto touches = [&](engine::PathId p) {
            for (std::size_t k = t->wayAt; k < t->way.paths.size(); ++k) {
                const engine::PathId w = t->way.paths[k];
                if (w == p) return true;
                if (knownPath(*roads_, w) && knownPath(*roads_, p) &&
                    roads_->paths[w.index].kind == engine::PathKind::Connector &&
                    roads_->paths[p.index].kind == engine::PathKind::Connector &&
                    roads_->paths[w.index].from == roads_->paths[p.index].from)
                    return true;
            }
            return p == c.path;
        };
        if (touches(other.path)) return true;
        for (std::size_t k = 0; k < other.trailCount && k < kMaxTrail; ++k)
            if (touches(other.trail[k])) return true;
        return false;
    };
    // The paths whose crossing points it may give way at: its plans' connectors and the paths of its way (#284: lanes too).
    const auto announcedOnMine = [&](CarId other) {
        const auto onPath = [&](engine::PathId p) {
            if (!knownPath(*roads_, p)) return false;
            const engine::Range xs = roads_->paths[p.index].crossings;
            for (uint32_t x = xs.first; x < xs.end() && x < roads_->pathCrossings.size(); ++x)
                for (const Announcement& a : board_.on(roads_->pathCrossings[x]))
                    if (a.who == other.index) return true;
            return false;
        };
        for (std::size_t k = 0; k < t->junctions.count && k < kMaxJunctionPlans; ++k)
            if (onPath(t->junctions.plans[k].connector)) return true;
        for (std::size_t k = t->wayAt; k < t->way.paths.size(); ++k)
            if (onPath(t->way.paths[k])) return true;
        return false;
    };
    switch (b.by) {
    case Limit::CarAhead: {
        // (A car named that is no longer on the road was taken off by the host after this tick's move, which named it:
        // the driver sees that at its next look. The tick itself never names a car that is not placed.)
        const Car* other = car(b.car);
        if (other && other->id == c.id) return no("behind itself");
        if (other && !onMyWay(*other))
            return no("behind car " + std::to_string(b.car.index) + ", which is not on its way ahead");
        return true;
    }
    case Limit::Stop: {
        if (!c.held.hasStop || c.held.stopRule != b.rule) return no("a stop it no longer holds");
        if (c.held.stopAt < c.odometer - static_cast<double>(junctionNumbers_.atLineWithin))
            return no("a place to stop that is behind it");
        if (b.car.valid()) {
            const Car* other = car(b.car);
            if (other && other->id == c.id) return no("waiting for itself");   // (gone from the road: as above)
            // (Read against this tick's board only on a tick the driver decided: between decisions it holds what it
            // decided, by the board then, for up to one decide interval.)
            const uint32_t every = std::max<uint32_t>(1, settings_.decideEveryTicks);
            const bool decidedNow = tickCount_ > 0 && (tickCount_ - 1) % every == c.id.index % every;
            if (givesWay(b.stopFor) && junctionsOn_ && decidedNow && !announcedOnMine(b.car))
                return no("giving way to car " + std::to_string(b.car.index) +
                          ", which is announced on none of the crossing points ahead of it");
            // (Waiting for room behind a car's tail: the car is on the road, above. Which car bounds the room is settled
            // again after each decision -- the same tick's settle step may refuse it for another reason -- and the
            // driver acts on that at its next decision, so the claim is not read against it.)
        }
        // A PLAIN STOP AT A LINE WHERE THE LIGHTS RUN (a red, a yellow) whose lamp has shown green for longer than a
        // decision: the driver has decided since with the light green, so whatever the stop is for, it is not that lamp.
        if (b.stopFor == StopFor::Place && lightsRun_) {
            const double interval = static_cast<double>(std::max<uint32_t>(1, settings_.decideEveryTicks) + 1) * step_;
            for (std::size_t k = 0; k < t->junctions.count && k < kMaxJunctionPlans; ++k) {
                const JunctionPlan& p = t->junctions.plans[k];
                if (std::fabs(p.startOdometer - c.held.stopAt) > static_cast<double>(junctionNumbers_.atLineWithin))
                    continue;
                if (lampAt(*roads_, rightOfWay_, p.connector, clockSeconds_) == Lamp::Green &&
                    lampAt(*roads_, rightOfWay_, p.connector, clockSeconds_ - interval) == Lamp::Green)
                    return no("a stop at junction " + std::to_string(p.junction.index) +
                              "'s line, whose lamp has shown green since before its last decision");
            }
        }
        return true;
    }
    case Limit::WayEnd:
        if (!t->wayEnds) return no("the end of a way that does not end");
        return true;
    case Limit::TopSpeed:
    case Limit::Nothing:
        break;
    }
    return b.by == Limit::TopSpeed;
}

std::string Driving::describeWaiting(const WaitReport& report) const {
    return driving::describeWaiting(report, hasRules_ ? &rules_.labels.rules : nullptr);
}

// ---- Junctions ---------------------------------------------------------------------------------------------------------

std::string Driving::describeJunction(engine::JunctionId junction) const {
    if (!roads_ || !hasRules_) return "no junction table: the model has no road or no rules yet\n";
    return describeJunctionRules(*roads_, ruleSet_, rightOfWay_, junction);
}

const JunctionPlans* Driving::junctionPlans(CarId car) const {
    const Trip* t = trips_.get(tripOf(car));
    return t ? &t->junctions : nullptr;
}

uint64_t Driving::planHash() const {
    engine::bundle::Fnv h;
    h.pod(static_cast<uint64_t>(tripOfCar_.size()));
    for (const std::pair<const CarId, TripId>& entry : tripOfCar_) {
        const Trip* t = trips_.get(entry.second);
        h.pod(entry.first.index);
        h.pod(t ? junctionPlansHash(t->junctions) : uint64_t(0));
    }
    h.pod(planCounts_.made);
    h.pod(planCounts_.cleared);
    h.pod(planCounts_.wayChanged);
    h.pod(planCounts_.routeChanged);
    h.pod(planCounts_.mostCrossings);
    h.pod(planCounts_.mostCrossingsConnector.index);
    h.pod(planCounts_.mostAtOnce);
    h.pod(planCounts_.askedAnotherWay);
    h.pod(planCounts_.tookAnotherWay);
    h.pod(capCounts_.junctionPlansCut);
    h.pod(capCounts_.lastJunctionPlanCut.index);
    h.pod(capCounts_.waitingPointsCut);
    h.pod(capCounts_.lastWaitingPointCut.index);
    return h.h;
}

}  // namespace driving
