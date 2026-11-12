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

void Driving::settleClaims() {}

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
    if (!gapKept || !standstill)
        return refuse(why, "rule set \"" + ruleSet.name + "\" lacks the setting \"" + kSettingGapKept +
                               "\" or the number \"" + kNumberStandstillGap + "\", which " + kMoveReaderName +
                               " reads (buildRoadRules reports it, given a registry with registerMoveNeeds)");
    // The junction step, when the registry has it: its numbers must all be there (buildRoadRules names any missing).
    const bool junctions = hasJunctionStep(facts);
    JunctionNumbers junctionNumbers;
    if (junctions && !readJunctionNumbers(ruleSet, junctionNumbers, why)) return false;
    junctionsOn_ = junctions;
    junctionNumbers_ = junctionNumbers;
    gapKept_ = *gapKept;
    gapKeptHash_ = settingNameHash(kSettingGapKept);
    standstillGap_ = static_cast<Real>(standstill->value);
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
    if (roads_) rightOfWay_ = buildRightOfWay(*roads_, ruleSet_, roadRules_);
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
    if (v.tieGroupsCut > 0) {
        capCounts_.tieGroupsCut += v.tieGroupsCut;
        capCounts_.lastTieGroupCut = car.id;
    }
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
        if (changed) {
            if (d == GiveWayDecision::Go) {
                ++planCounts_.decidedGo;
                if (plan.decision == GiveWayDecision::GiveWay && j.cause == GiveWayCause::Gap) {
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
            if (plan.decision != GiveWayDecision::Undecided && ++plan.mindChanges == 2) ++planCounts_.changedMindTwice;
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

    // 1. Each car's own notes, and its nearest line.
    for (std::size_t i = 0; i < cars_.size(); ++i) {
        LineTimes& times = lineTimes_[i];
        for (std::pair<engine::PathId, Real>& lt : times) lt = {engine::PathId(), kNever};
        const Car& c = cars_[i];
        Trip* t = trips_.get(tripOfRecord_[i]);
        if (!c.driven || !t || !c.held.decided || c.vehicleClass >= vehicleClasses_.size()) continue;
        const VehicleClass& cls = vehicleClasses_.at(c.vehicleClass);
        JunctionPlans& plans = t->junctions;
        for (std::size_t k = 0; k < plans.count; ++k)
            noteAtTheLine(plans.plans[k], c, cls.brake, n.atLineWithin, tickCount_, clockSeconds_);
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
            w.notGoing = c.held.hasStop && !stopIsJunctionsOwn(c, p, n.atLineWithin) &&
                         c.held.stopAt <= p.startOdometer + n.atLineWithin;
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
            a.move = base;
            a.move.speedThrough = speedThrough(*roads_, roadRules_, p.connector, cls.topSpeed, n.comfortableSideAccel);
            a.heldStop = heldStopOf(c);
            a.reaction = static_cast<Real>(std::max<uint32_t>(1, settings_.decideEveryTicks)) * step_;
            a.stopsFirst = stopsFirstAt(c, p, n.atLineWithin);
            const bool queued = w.has && k >= w.plan;
            a.shift = queued ? w.in - w.ownIn : Real(0);
            a.notGoing = queued && w.notGoing;
            a.heldStopAt = c.held.hasStop && !stopIsJunctionsOwn(c, p, n.atLineWithin) ? c.held.stopAt
                                                                                        : static_cast<double>(kNever);
            a.frontLimit = w.frontLimit;
            a.waited = p.cameToRest ? static_cast<Real>(std::max(0.0, clockSeconds_ - p.restAt)) : Real(0);
            // When it reaches the line; a junction beyond the look-ahead time is not announced, nor any after it.
            const Real toLine = static_cast<Real>(p.startOdometer - c.odometer);
            const Real lineIn =
                toLine <= 0 ? Real(0)
                            : (a.stopsFirst ? secondsToStopAt(toLine, a.move) : secondsToCover(toLine, a.move)) + a.shift;
            if (lineIn > n.lookAheadTime) break;
            lineTimes_[i][k] = {p.connector, lineIn};
            announcePlan(*roads_, p, a, board_);
        }
        // The lanes its body is on and its way runs along, where a lane meets a junction's connector: as going.
        Mover lane = base;
        lane.speedThrough = c.held.topSpeed > 0 ? c.held.topSpeed : cls.topSpeed;
        Real start = -c.along;
        for (std::size_t k = 0; k < c.trailCount && k < kMaxTrail; ++k) {
            start -= roads_->paths[c.trail[k].index].length;
            if (roads_->paths[c.trail[k].index].kind == engine::PathKind::Lane)
                announceLane(*roads_, c.trail[k], start, c.id, cls.length, lane, n.lookAheadTime, board_);
        }
        start = -c.along;
        if (roads_->paths[c.path.index].kind == engine::PathKind::Lane)
            announceLane(*roads_, c.path, start, c.id, cls.length, lane, n.lookAheadTime, board_);
        start += roads_->paths[c.path.index].length;
        const std::vector<engine::PathId>& way = t->way.paths;
        if (t->wayAt < way.size() && way[t->wayAt] == c.path)
            for (std::size_t k = t->wayAt + 1; k < way.size(); ++k) {
                if (secondsToCover(start, lane) > n.lookAheadTime) break;
                if (roads_->paths[way[k].index].kind == engine::PathKind::Lane)
                    announceLane(*roads_, way[k], start, c.id, cls.length, lane, n.lookAheadTime, board_);
                start += roads_->paths[way[k].index].length;
            }
    }
    board_.seal();
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
        p.way = wayAhead(t->route, *roads_, c.path, c.along, lookAhead, p.legReached);
        Real reach = -c.along;
        for (const engine::PathId path : p.way.paths) reach += roads_->paths[path.index].length;
        p.wayEnds = reach < lookAhead && p.way.paths.size() < kMaxFollowedPaths;
        if (reach < lookAhead && p.way.paths.size() >= kMaxFollowedPaths) {
            // Cut short by the cap on paths followed, not by how far the driver looks: counted, never silent.
            ++capCounts_.lookAheadsCut;
            capCounts_.lastLookAheadCut = c.id;
        }
        // The junctions on the way ahead: plans kept, dropped and made before the driver notices anything.
        p.junctions = t->junctions;
        updateJunctionPlans(*roads_, rightOfWay_, c, cls.length, p.way, tickCount_, clockSeconds_, p.junctions,
                            p.planUpdate);
        // ... and whether it must give way at them, judged from this tick's board.
        if (junctionsOn_) {
            GiveWayInputs in{*roads_, rightOfWay_, ruleSet_, board_, junctionNumbers_, c, cls.length, cls.brake,
                             static_cast<Real>(every) * step_, t->person, clockSeconds_, p.junctions, {}};
            for (std::size_t k = 0; k < kMaxJunctionPlans; ++k) {
                in.lineIn[k] = kNever;
                if (k >= p.junctions.count) continue;
                for (const std::pair<engine::PathId, Real>& lt : lineTimes_[i])
                    if (lt.first == p.junctions.plans[k].connector) in.lineIn[k] = lt.second;
            }
            judgeGiveWay(in, p.verdict);
            p.lineIn = in.lineIn;
        }

        DriverView view{c,          cls,       t->person,  p.way,         *roads_,     cars_,     pathLists_,
                        ruleSet_,   roadRules_, lookAhead, &t->destination, &t->holds, &p.junctions, &rightOfWay_};
        if (junctionsOn_) {
            view.board = &board_;
            view.giveWay = &p.verdict;
        }
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
                if (p.held.stopFor == StopFor::GivingWay) p.held.stopCar = noticed_[slot].aboutCar;
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
        c.held = p.held;
        t->junctions = p.junctions;
        notePlanUpdate(p.planUpdate, c.id, *t, t->junctions);
        applyVerdict(p.verdict, p.lineIn, c, *t);
        t->way = std::move(p.way);
        t->wayAt = 0;
        t->wayEnds = p.wayEnds;
        t->legReached = p.legReached;
        if (settings_.trace) {
            t->trace.heldStop = p.stopWhy;
            t->trace.heldTopSpeed = p.topSpeedWhy;
        }
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
    }
}

Route Driving::routeFor(const Car& c, const PathPoint& to) const {
    PathPoint front;
    front.path = c.path;
    front.along = c.along;
    RouteClass cls = c.vehicleClass < routeClasses_.size() ? routeClasses_[c.vehicleClass] : RouteClass();
    cls.laneChanges = settings_.laneChanges;
    return findRoute(*roads_, routeBounds_, cls, front, to);
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
    h.pod(capCounts_.tieGroupsCut);
    h.pod(capCounts_.lastTieGroupCut.index);
    h.pod(static_cast<uint8_t>(junctionsOn_));
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
    WaitReport r = analyseWaiting(waitingCars(cars_));
    for (const Car& c : cars_)
        if (c.driven) ++r.driven;
    return r;
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
    h.pod(capCounts_.junctionPlansCut);
    h.pod(capCounts_.lastJunctionPlanCut.index);
    h.pod(capCounts_.waitingPointsCut);
    h.pod(capCounts_.lastWaitingPointCut.index);
    return h.h;
}

}  // namespace driving
