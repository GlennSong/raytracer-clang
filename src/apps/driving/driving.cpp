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

Driving::Driving(const engine::RoadModel* model) : model_(model), roads_(model ? &model->laneLevel : nullptr) {
    if (model_) routeBounds_ = studyRoads(*model_);
}

void Driving::tick(Real dt) {
    if (listsStale_ && roads_) rebuildPathLists(*roads_, vehicleClasses_, cars_, pathLists_);   // (carriedAt wrote)
    listsStale_ = false;
    horns_.clear();
    if (hasRules_ && roads_ && dt > 0) {
        step_ = dt;
        plan();
        if (listsStale_ && roads_) rebuildPathLists(*roads_, vehicleClasses_, cars_, pathLists_);   // (a lane change began)
        listsStale_ = false;
        settleClaims();
        move(dt);
        if (crashesOn_) crash();
    }
    ++tickCount_;
    clockSeconds_ += dt;
    if (measurements_) measurements_->observe(*this);   // it only watches
}

// ---- CONTACT AND CRASHES (sitting 7) -----------------------------------------------------------------------------------

void Driving::crash() {
    // 1. Wrecks whose time is up are cleared: off the road, and the host told.
    for (std::map<CarId, Wreck>::iterator w = wrecks_.begin(); w != wrecks_.end();) {
        if (tickCount_ < w->second.clearAtTick) {
            ++w;
            continue;
        }
        const CarId id = w->first;
        const PersonId person = w->second.person;
        w = wrecks_.erase(w);
        if (car(id) && tripOfCar_.count(id) == 0) removeCar(id);
        queueEvent(EventKind::WreckCleared, id, TripId{}, person);
        ++crashCounts_.cleared;
    }
    // 2. Every body against every other (contact.h), at the class's size.
    crashBodies_.clear();
    crashBodyRecord_.clear();
    for (std::size_t r = 0; r < cars_.size(); ++r) {
        const Car& c = cars_[r];
        if (!c.path.valid() || c.path.index >= roads_->paths.size() || c.vehicleClass >= vehicleClasses_.size()) continue;
        const VehicleClass& cls = vehicleClasses_.at(c.vehicleClass);
        crashBodies_.push_back(planBody(c.id, carPose(*roads_, vehicleClasses_, ruleSet_, c), cls.length, cls.width));
        crashBodyRecord_.push_back(r);
    }
    findContacts(crashBodies_, crashSweep_, crashContacts_);
    // 3. A new contact is counted and rolled for: a crash leaves its driven cars as wrecks.
    std::set<std::pair<uint32_t, uint32_t>> now;
    std::vector<CarId> crashed;
    for (const BodyContact& k : crashContacts_) {
        const Car& a = cars_[crashBodyRecord_[k.a]];
        const Car& b = cars_[crashBodyRecord_[k.b]];
        const std::pair<uint32_t, uint32_t> key(a.id.index, b.id.index);
        now.insert(key);
        if (touching_.count(key) != 0) continue;   // (the same contact, going on)
        if (!a.driven && !b.driven) continue;      // (two cars nobody drives: placed so by the host)
        ++crashCounts_.contacts;
        engine::bundle::Fnv h;
        h.pod(key.first);
        h.pod(key.second);
        h.pod(tickCount_);
        const double roll = static_cast<double>(h.h >> 11) * (1.0 / 9007199254740992.0);   // [0, 1)
        if (!(roll < static_cast<double>(crashChance_))) continue;
        ++crashCounts_.crashes;
        if (a.driven) crashed.push_back(a.id);
        if (b.driven) crashed.push_back(b.id);
    }
    touching_.swap(now);
    for (const CarId id : crashed)
        if (Car* c = findPlaced(id); c && c->driven) wreck(*c);
}

void Driving::wreck(Car& c) {
    const std::map<CarId, TripId>::iterator ti = tripOfCar_.find(c.id);
    PersonId person = 0;
    if (ti != tripOfCar_.end()) {
        if (const Trip* t = trips_.get(ti->second)) person = t->person;
        queueEvent(EventKind::Crashed, c.id, ti->second, person);
        trips_.erase(ti->second);
        tripOfCar_.erase(ti);
    }
    leaveParked(c);   // (where it stands, at rest: a crash stops it)
    laneMoves_.erase(c.id);
    indicating_.erase(c.id);
    lettingIn_.erase(c.id);
    droppingBack_.erase(c.id);
    Wreck w;
    w.clearAtTick = tickCount_ + (step_ > 0 ? static_cast<uint64_t>(std::ceil(wreckSeconds_ / step_)) : 0);
    w.person = person;
    wrecks_[c.id] = w;
    ++crashCounts_.wrecks;
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
    world.reaction = settings_.decideIntervalAt(step_);
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
    // (with the lane change step: no move across planned in a lane too short to make one in, a lane's width over)
    if (changingLanes() && vehicleClass < vehicleClasses_.size())
        cls.laneChangeLeast = vehicleClasses_.at(vehicleClass).length +
                              laneMoveShortest(kWidestLaneAcross, vehicleClasses_.at(vehicleClass).tightestTurnRadius);
    if (hasRules_) cls.limits = &roadRules_.limitOfPath;   // timed as its cars drive: the rules' limit on each path
    return cls;
}

RouteClass Driving::routeClass(uint32_t vehicleClass, bool exactLane) const {
    RouteClass cls = routeClass(vehicleClass);
    cls.anyLaneAtEnd = !exactLane;
    if (cls.anyLaneAtEnd && roads_) {
        if (classParts_.size() <= vehicleClass) return cls;   // (no class set: any lane of the stretch)
        ClassParts& parts = classParts_[vehicleClass];
        if (!parts.made || parts.laneChanges != cls.laneChanges || parts.laneChangeLeast != cls.laneChangeLeast) {
            engine::ReachRules rules;
            rules.laneChanges = cls.laneChanges;
            rules.laneChangeLeast = cls.laneChangeLeast;
            rules.open.assign(roads_->paths.size(), 0);
            for (std::size_t p = 0; p < roads_->paths.size(); ++p)
                rules.open[p] = mayUse(*roads_, cls, engine::PathId(static_cast<uint32_t>(p))) ? 1 : 0;
            parts.partOf = engine::reachableParts(*roads_, rules).partOf;
            parts.made = true;
            parts.laneChanges = cls.laneChanges;
            parts.laneChangeLeast = cls.laneChangeLeast;
        }
        cls.partOf = &parts.partOf;
    }
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
    // What is in a driver's way (ADR-0451), when the registry has it: its numbers, and the answers it acts on.
    thingsOn_ = hasThingStep(facts);
    if (thingsOn_) {
        const Number* standOff = ruleSet.findNumber(kNumberThingStandOff);
        const Number* clearance = ruleSet.findNumber(kNumberPassClearance);
        if (!standOff || !clearance)
            return refuse(why, "rule set \"" + ruleSet.name + "\" lacks the number \"" + kNumberThingStandOff + "\" or \"" +
                                   kNumberPassClearance + "\", which the thing step reads");
        thingStandOff_ = static_cast<Real>(standOff->value);
        passClearance_ = static_cast<Real>(clearance->value);
        // (people on crosswalks, ADR-0452: the facts' own numbers, which buildRoadRules has checked are there)
        const Number* cwStandOff = ruleSet.findNumber(kNumberCrosswalkStandOff);
        const Number* cwDecel = ruleSet.findNumber(kNumberCrosswalkDecel);
        crosswalkStandOff_ = cwStandOff ? static_cast<Real>(cwStandOff->value) : Real(0);
        crosswalkDecel_ = cwDecel ? static_cast<Real>(cwDecel->value) : Real(0);
        hornAction_ = vocabulary.findAction(kActionHorn);
        passKind_ = vocabulary.findIntention(kIntentionPass);
        wayKind_ = vocabulary.findIntention(kIntentionWay);
    }
    // CONTACT AND CRASHES (sitting 7): on where the rule set gives the chance of a contact being a crash and how long a
    // wreck stands.
    {
        const Number* chance = ruleSet.findNumber(kNumberCrashChance);
        const Number* seconds = ruleSet.findNumber(kNumberWreckSeconds);
        crashesOn_ = chance && seconds && chance->value > 0 && seconds->value > 0;
        crashChance_ = chance ? static_cast<Real>(chance->value) : Real(0);
        wreckSeconds_ = seconds ? static_cast<Real>(seconds->value) : Real(0);
    }
    // LANE CHANGES (lane_change.h; sitting 5), when the registry has the step: its numbers and its answer. It moves cars
    // across only where their routes plan changes (DrivingSettings::laneChanges).
    laneChangesOn_ = hasLaneChangeStep(facts);
    if (laneChangesOn_) {
        const Number* brake = ruleSet.findNumber(kNumberLaneChangeBrake);
        const Number* seconds = ruleSet.findNumber(kNumberLaneChangeSeconds);
        if (!brake || !seconds || !(seconds->value > 0))
            return refuse(why, "rule set \"" + ruleSet.name + "\" lacks the number \"" + kNumberLaneChangeBrake + "\" or \"" +
                                   kNumberLaneChangeSeconds + "\" (above zero), which the lane change step reads");
        laneChangeBrake_ = static_cast<Real>(brake->value);
        // (the paths leading into each path, for the look back along them)
        pathsBefore_.assign(roads_ ? roads_->paths.size() : 0, {});
        if (roads_)
            for (uint32_t k = 0; k < roads_->paths.size(); ++k) {
                const engine::Range nx = roads_->paths[k].next;
                for (uint32_t n = nx.first; n < nx.end() && n < roads_->nextPaths.size(); ++n) {
                    const engine::PathId q = roads_->nextPaths[n];
                    if (q.valid() && q.index < pathsBefore_.size()) pathsBefore_[q.index].push_back(engine::PathId(k));
                }
            }
        laneChangeSeconds_ = static_cast<Real>(seconds->value);
        laneKind_ = vocabulary.findIntention(kIntentionLane);
        letInKind_ = vocabulary.findIntention(kIntentionLetIn);
        gapKind_ = vocabulary.findIntention(kIntentionGap);
    }
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
        rightOfWay_ = buildRightOfWay(*model_, ruleSet_, roadRules_);
        // ... and every junction's traffic lights timed, with the far turns' places to wait inside checked (#261).
        if (lightsRun_) timeSignalProgrammes(*model_, roadRules_, lightTiming_, rightOfWay_);
        // ... and checked: no stage lets in together two movements whose paths meet unless the rule set allows it. A
        // breach is reported (signalProblems, and in the table's notes), not refused: the host decides (issue #284).
        signalProblems_ = lightsRun_ ? checkSignalProgrammes(*model_, ruleSet_, rightOfWay_) : std::vector<std::string>();
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
    const Real reaction = settings_.decideIntervalAt(step_);
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
            // ... and queued behind a car standing at the line giving way there: I am not coming until it moves, which
            // nobody can time (sitting 6: a driver sees the oncoming queue stand behind a car that waits for it; without
            // the resolver's ring order, two far turns waiting inside, each for the other's oncoming queue, stood for good)
            const bool aheadWaitsAtLine = a->speed < kStandingBelow && a->held.hasStop && givesWay(a->held.stopFor) &&
                                          wa.toLine <= n.atLineWithin;
            w.notGoing = w.notGoing || wa.notGoing || aheadWaitsAtLine;
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
    // Who goes first wherever two announced cars meet: the questions both drivers ask (conflict.h), worked out once from
    // the sealed board. A car behind another on one path (the next in its list: a front, or a tail still on it) can never
    // go before it, and the car ahead never gives way to it (GiveWayInputs::behind). Read from where the cars are, not
    // from what binds them, so it does not come and go as the car ahead moves off.
    behind_.clear();
    for (uint32_t p = 0; p < roads_->paths.size(); ++p) {
        const PathList list = pathLists_.on(engine::PathId(p));
        for (std::size_t k = 0; k + 1 < list.size(); ++k)
            if (list[k].car != list[k + 1].car && list[k].why != OnPath::Beside && list[k + 1].why != OnPath::Beside)
                behind_.push_back({list[k + 1].car.index, list[k].car.index});
    }
    // ... and a car following another along a split's stretch (two paths leaving one lane: the car behind follows the car
    // ahead, whichever path each takes, traffic_state.h carAhead) can never go before it either. Without this arrow a car
    // ahead on one path could be told to give way, further on, to the car queued behind it on the other: brain_crowd's
    // car 102 waited inside junction 7 for car 136, which stood behind it, from minute 11 to the end.
    {
        std::vector<std::pair<uint64_t, uint64_t>> onOnePath = behind_;
        std::sort(onOnePath.begin(), onOnePath.end());
        const std::vector<engine::PathId> noPathsAhead;
        for (const Car& c : cars_) {
            if (!c.driven || !c.path.valid() || c.path.index >= roads_->paths.size()) continue;
            const Neighbour n = carAhead(*roads_, cars_, pathLists_, c.id, noPathsAhead, kNever);
            if (!n.found()) continue;
            const std::pair<uint64_t, uint64_t> arrow{n.car.index, c.id.index};
            if (!std::binary_search(onOnePath.begin(), onOnePath.end(), arrow)) behind_.push_back(arrow);
        }
    }
    std::sort(behind_.begin(), behind_.end());
    behind_.erase(std::unique(behind_.begin(), behind_.end()), behind_.end());
    resolver_.study(*roads_, rightOfWay_, board_, clockSeconds_);
    addCounts(conflictTotals_, resolver_.counts());
}

// PLAN: each driven car whose turn it is (or that has not decided yet on this trip) notices its facts along its way
// ahead and decides by the rules. Every decision reads the cars as they stood at the start of the tick (and this tick's
// board); the answers are applied only once all are made.
void Driving::plan() {
    findTripsOfRecords();
    if (junctionsOn_) announce();
    const uint32_t every = settings_.decideEveryTicks(step_);   // the decide interval in ticks of this tick's length
    planned_.clear();
    if (thingsOn_) crosswalkPeople_.fill(things_, model_ ? model_->crosswalks.size() : 0);   // (ADR-0452)
    // WHAT DRIVERS SEE IN THEIR WAY (things.h): the host's things, and every car nobody here drives standing on a lane --
    // a wreck, a car left in the road -- as things.h always named them (sitting 7). A driver follows one to a stop as a
    // car ahead, and as a thing honks, goes round it where there is room, or after waiting finds another way past (a car
    // left parked just past a junction stood the queue behind it for good: #326). Numbered apart from the host's things.
    seenThings_.clear();
    if (thingsOn_) {
        seenThings_ = things_;
        for (const Car& c : cars_) {
            if (c.driven || carried_.count(c.id) || !c.path.valid() || c.path.index >= roads_->paths.size() ||
                c.vehicleClass >= vehicleClasses_.size() || roads_->paths[c.path.index].kind != engine::PathKind::Lane)
                continue;
            const CarPose pose = carPose(*roads_, vehicleClasses_, ruleSet_, c);
            RoadThing th;
            th.id = kCarThingBit | c.id.index;
            th.at = engine::Vec2((pose.front.x + pose.rear.x) * Real(0.5), (pose.front.z + pose.rear.z) * Real(0.5));
            th.halfAcross = vehicleClasses_.at(c.vehicleClass).width * Real(0.5);
            seenThings_.push_back(th);
        }
    }
    for (std::size_t i = 0; i < cars_.size(); ++i) {
        const Car& c = cars_[i];
        const Trip* t = trips_.get(tripOfRecord_[i]);
        if (!c.driven || !t || c.vehicleClass >= vehicleClasses_.size()) continue;
        if (c.held.decided && tickCount_ % every != c.id.index % every &&
            !std::binary_search(decideNext_.begin(), decideNext_.end(), c.id))
            continue;
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
                // ...and, with the thing step, a car's length and a stand-off past the box: whether something stands
                // there, leaving it no room, is seen at the line (things.h thing_stop)
                if (thingsOn_ && k == 0)
                    lookAhead = std::max(lookAhead, static_cast<Real>(jp.startOdometer - c.odometer) +
                                                        roads_->paths[jp.connector.index].length + cls.length +
                                                        thingStandOff_ + passClearance_);
            }
        }
        // The way ahead, and the junctions on it: plans kept, dropped and made before the driver notices anything. With
        // room beyond (#264), as far again as a claim at the nearest junction needs: through each junction its body
        // would not fit before (room.h roomReach; at most room_through_junctions more looks).
        for (uint32_t look = 0;; ++look) {
            p.way = wayAhead(t->route, *roads_, c.path, c.along, lookAhead, p.legReached);
            p.junctions = t->junctions;
            p.planUpdate = PlanUpdate();
            updateJunctionPlans(*model_, rightOfWay_, c, cls.length, p.way, tickCount_, clockSeconds_, p.junctions,
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
        // (with lane changes, a way that stops short only because the route wants the lane beside -- its last lane leads
        // on, just not where this trip goes, and is not the destination's -- is not a place to come to rest: the driver
        // moves over, or past its last chance is routed again from its lane, and never stands in a through lane waiting)
        if (p.wayEnds && changingLanes() && !p.way.paths.empty()) {
            const engine::PathId last = p.way.paths.back();
            if (!sameSection(last, t->destination.path) && roads_->paths[last.index].next.count > 0) p.wayEnds = false;
        }
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
            in.behind = &behind_;
            if (changingLanes()) in.lettingIn = &lettingIn_;   // (a driver holding back to let me in is not coming)
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
        view.destinationExact = t->exactLane;
        if (junctionsOn_) {
            view.board = &board_;
            view.giveWay = &p.verdict;
        }
        if (roomOn_) view.room = &p.roomStop;
        // WHAT IS IN ITS WAY (ADR-0451): physics' answer for a carried car, the things on the road for a lane car
        ThingView things;
        if (thingsOn_) {
            if (carried_.count(c.id)) {
                const std::map<CarId, ThingAhead>::const_iterator s = seen_.find(c.id);
                if (s != seen_.end()) things.thing = s->second;
            } else {
                const std::vector<engine::PathId> after(p.way.paths.size() > 1 ? p.way.paths.begin() + 1 : p.way.paths.end(),
                                                        p.way.paths.end());
                things.thing = findThingAhead(*roads_, c.path, c.along, after, lookAhead, cls.width * Real(0.5), c.offset,
                                              passClearance_, seenThings_, c.speed);
            }
            ThingWatch& w = watch_[c.id];
            if (!things.thing.found || things.thing.id != w.id) {
                w.id = things.thing.found ? things.thing.id : 0;
                w.still = false;
                w.waiting = false;
                w.honked = false;
                w.askedAnotherWayTick = 0;
            }
            if (things.thing.found) {
                ++thingCounts_.seen;
                const bool inJunction = (things.thing.path.valid() && things.thing.path.index < roads_->paths.size() &&
                                         roads_->paths[things.thing.path.index].kind == engine::PathKind::Connector) ||
                                        roads_->paths[c.path.index].kind == engine::PathKind::Connector;
                if (inJunction && c.speed < kStandingBelow) ++thingCounts_.atJunctions;
                if (things.thing.moves < kThingStillBelow) {
                    if (!w.still) w.stillSince = tickCount_;
                    w.still = true;
                    things.stood = static_cast<Real>(static_cast<double>(tickCount_ - w.stillSince) * step_);
                } else {
                    w.still = false;
                }
                if (c.speed < kStandingBelow) {
                    if (!w.waiting) w.waitingSince = tickCount_;
                    w.waiting = true;
                    things.waited = static_cast<Real>(static_cast<double>(tickCount_ - w.waitingSince) * step_);
                } else {
                    w.waiting = false;
                }
            } else {
                w.waiting = false;
            }
            w.last = things.thing;
            // PEOPLE ON CROSSWALKS (ADR-0452): the nearest crosswalk ahead my way crosses with someone on it, or about to
            // step on it
            {
                const std::vector<engine::PathId> after(p.way.paths.size() > 1 ? p.way.paths.begin() + 1 : p.way.paths.end(),
                                                        p.way.paths.end());
                things.crosswalk = findCrosswalkAhead(*model_, c.path, c.along, after, lookAhead, c.speed, crosswalkDecel_,
                                                      crosswalkPeople_);
                things.crosswalkStandOff = crosswalkStandOff_;
                uint32_t& was = crosswalkOf_[c.id];
                if (things.crosswalk.found && things.crosswalk.crosswalk != was) {
                    const engine::ModelCrosswalk* cw = model_->crosswalk(things.crosswalk.crosswalk);
                    if (cw) ++thingCounts_.crosswalkGiveWays[static_cast<std::size_t>(cw->control)];
                }
                was = things.crosswalk.found ? things.crosswalk.crosswalk : engine::kNoIndex;
                // (standing for someone who is only waiting at the kerb: both hesitate, until the person sees it stand)
                if (things.crosswalk.found && !things.crosswalk.someoneOn && c.speed < kStandingBelow)
                    ++thingCounts_.crosswalkStandOffs;
            }
            things.standOff = thingStandOff_;
            things.myLength = cls.length;
            for (std::size_t k = 0; k < p.junctions.count; ++k) {
                const JunctionPlan& jp = p.junctions.plans[k];
                if (!jp.connector.valid() || jp.connector.index >= roads_->paths.size()) continue;
                const Real toLine = static_cast<Real>(jp.startOdometer - c.odometer);
                if (toLine < 0) continue;
                things.lineAt = toLine;
                things.boxEndsAt = toLine + roads_->paths[jp.connector.index].length;
                break;
            }
            p.thing = things.thing;
            view.things = &things;
        }
        // ITS LANE (lane_change.h; sitting 5): whether its route needs another lane of this road, how long it may stay in
        // its own, and the gap in the lane beside it. Not while it is already moving across, nor for a car physics carries.
        LaneChangeView lane;
        // (in the destination's own stretch it does not move over for it: level with it in any lane of that stretch is
        // there, as the arrive fact reads a destination -- a short stretch has no room for a move anyway)
        const bool lanesLooked = changingLanes() && !carried_.count(c.id);
        // (...unless it must end in the destination's own lane: then it moves over in that stretch too)
        if (lanesLooked && !c.besidePath.valid() && (t->exactLane || !sameSection(c.path, t->destination.path))) {
            lane.need = lanesToCross(t->route, c.path, p.legReached);
            lane.last = lastChance(t->route, c.path, c.along, p.legReached);
            if (lane.need.onRoute && lane.need.lanes > 0) {
                ++laneChangeCounts_.wanted;
                // (the move needs room: where it must begin at the latest; a lane with no way on is waited in, there)
                // (room for every move still needed: the one after this one begins where this one ends)
                lane.changeBy = lane.last.metres - static_cast<Real>(std::max<uint32_t>(lane.need.inLeg, 1)) *
                                                       laneMoveShortest(roads_->paths[c.path.index].width, cls.tightestTurnRadius);
                // (while it still needs the change, on whichever piece of lane it has rolled onto: tied to the lane it
                // missed on, it was let go a metre later, on the next piece of the same road, and drove to its end)
                lane.mustWait = t->waitToChange;
                lane.gap = findLaneGap(*roads_, cars_, pathLists_, c, cls.length, lane.need.toward == Across::TowardKerb,
                                       laneChangeBrake_, kerbwardPerpSign(ruleSet_.drivesOn), pathsBefore_,
                                       [&](const Car& who, Real gap, Real speedAhead) {
                                           return followAcceleration(who, gap, speedAhead);
                                       });
                if (lane.gap.safe && landsInAMeeting(c, lane.gap, cls.length)) lane.gap.safe = false;
            }
        }
        if (lanesLooked) {
            // ITS INDICATOR: on while it wants the lane beside with no safe gap, and must have it -- it waits for one, or
            // it has a last chance -- so a driver there may let it in.
            if (lane.need.onRoute && lane.need.lanes > 0 && lane.gap.target.valid() && !lane.gap.safe &&
                (lane.mustWait || lane.last.needed))
                indicating_[c.id] = Indicating{lane.gap.target, lane.gap.targetAlong - c.along,
                                               lane.need.toward == Across::TowardKerb, false};
            else if (const Indicating merge = mergeWait(c); merge.into.valid())
                indicating_[c.id] = merge;
            else
                indicating_.erase(c.id);
            // A CAR WANTING INTO MINE, ahead of my front: the one I let in already while it still indicates (zipper),
            // else the nearest within the distance I would leave it (its body and my following gap at my speed)
            const Real reach = cls.length + std::max(c.speed, Real(5)) * static_cast<Real>(personalValue(gapKept_, gapKeptHash_, t->person)) * 2;
            Real front = 0, rear = 0;
            const std::map<CarId, CarId>::const_iterator had = lettingIn_.find(c.id);
            // (only a car whose rear would be ahead of my front: one already level with me I cannot drop behind -- held
            // to it, I stood on the join's stretch while it gave way to me, both for good: #323)
            if (had != lettingIn_.end() && indicatedInto(had->second, c.path, &front, &rear) && rear > c.along) {
                lane.wantingIn = had->second;
                lane.wantingInGap = rear - c.along;
            } else {
                for (const std::pair<const CarId, Indicating>& ind : indicating_) {
                    if (ind.first == c.id || ind.second.into != c.path) continue;
                    if (!indicatedInto(ind.first, c.path, &front, &rear) || rear <= c.along || rear - c.along > reach) continue;
                    if (!lane.wantingIn.valid() || rear - c.along < lane.wantingInGap) {
                        lane.wantingIn = ind.first;
                        lane.wantingInGap = rear - c.along;
                    }
                }
            }
            p.laneSeen = true;
            view.laneChange = &lane;
        }
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
        if (p.laneSeen) {
            p.lane = lane;
            if (laneKind_ < decision_.intentions.size()) p.changeLane = decision_.intentions[laneKind_].given;
            if (letInKind_ < decision_.intentions.size()) p.letIn = decision_.intentions[letInKind_].given;
            if (p.letIn && lane.wantingIn.valid()) {
                CarId& letting = lettingIn_[c.id];
                if (letting != lane.wantingIn) ++laneChangeCounts_.letIn;
                letting = lane.wantingIn;
            } else
                lettingIn_.erase(c.id);
            if (gapKind_ < decision_.intentions.size()) p.dropBack = decision_.intentions[gapKind_].given;
            if (p.dropBack && lane.gap.blocker.valid() && lane.gap.target.valid()) {
                const std::map<CarId, DropBack>::const_iterator was = droppingBack_.find(c.id);
                if (was == droppingBack_.end() || was->second.behind != lane.gap.blocker) ++laneChangeCounts_.droppedBack;
                droppingBack_[c.id] = DropBack{lane.gap.blocker, lane.gap.target, lane.gap.targetAlong - c.along};
            }
            else
                droppingBack_.erase(c.id);
        }
        if (thingsOn_ && p.thing.found) {
            for (const ActionAnswer& a : decision_.actions) p.horn = p.horn || (a.action != kNoAction && a.action == hornAction_);
            if (passKind_ < decision_.intentions.size()) p.pass = decision_.intentions[passKind_].given;
            if (wayKind_ < decision_.intentions.size()) p.anotherWay = decision_.intentions[wayKind_].given;
        }
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

    decideNext_.clear();
    for (Planned& p : planned_) {
        Car& c = cars_[p.record];
        Trip* t = trips_.get(p.trip);
        if (p.arrived) {
            // Undriven from here on; its speed (a crawl at most) is set to rest by the move step, which writes the
            // copy the other cars read next tick.
            queueEvent(EventKind::Parked, c.id, p.trip, t->person);
            if (t->pullInOffset != 0) {
                // In its kerb space: the curve is driven to within the arrival distance of its end, which is flat
                // there, so what is left of it is a centimetre or so; it stands on the space's line from now.
                const std::map<CarId, LaneMove>::iterator m = laneMoves_.find(c.id);
                if (t->pullInBegun && !t->pullInMissed) {
                    if (m != laneMoves_.end() && m->second.kerb) {   // (gone already: driven to its very end)
                        c.offset = m->second.to;
                        c.offsetRearLag = 0;
                        laneMoves_.erase(m);
                    }
                    inSpace_.insert(c.id);
                    ++kerbMoveCounts_.pulledIn;
                } else {
                    ++kerbMoveCounts_.stoppedInLane;
                }
            }
            ++tripTimes_.arrived;
            tripTimes_.seconds += static_cast<double>(tickCount_ - t->beganAtTick) * step_;
            tripTimes_.freeFlowSeconds += t->plannedSeconds;
            tripTimes_.metres += t->plannedMetres;
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
        // ITS LANE (sitting 5): over into the lane beside, or, past its last chance, routed again from the one it is in
        if (p.laneSeen) changeLanes(c, *t, p);
        // Refused room for long at the junction it waits at: it asks for another way (issue #284). Visited in car order,
        // after its own decision is applied; a route found changes only this car's trip.
        if (roomOn_) askAnotherWay(c, *t);
        // WHAT IT DECIDED ABOUT THE THING IN ITS WAY (ADR-0451): a horn, once for each thing; going round it; another way.
        if (thingsOn_ && p.thing.found) {
            ThingWatch& w = watch_[c.id];
            if (p.horn && !w.honked) {
                w.honked = true;
                horns_.push_back(c.id);
                ++thingCounts_.horns;
            }
            if (p.pass && p.thing.roomToPass && !w.passing) {
                w.passing = true;
                w.passOffset = p.thing.passOffset;
                w.passEnd = c.odometer + std::max(Real(0), p.thing.distance) + 2 * p.thing.halfAcross +
                            vehicleClasses_.at(c.vehicleClass).length + passClearance_;
                ++thingCounts_.swerves;
            }
            if (p.anotherWay && w.askedAnotherWayTick == 0) {
                w.askedAnotherWayTick = tickCount_;
                askAnotherWayRound(c, *t, p.thing);
            }
        }
    }
}

// MOVE: every driven car's new speed from its held answers and a fresh look at the car ahead (act.h), then along its
// held way. Reads cars_ (the state at the start of the step), writes carsNext_; the two then swap.
void Driving::move(Real dt) {
    findTripsOfRecords();
    carsNext_ = cars_;
    const uint32_t every = settings_.decideEveryTicks(dt);
    const Real reaction = reactionTime(static_cast<Real>(every) * dt, dt);
    intents_.assign(cars_.size(), CarIntent());
    intentSet_.assign(cars_.size(), 0);

    for (std::size_t i = 0; i < cars_.size(); ++i) {
        const Car& c = cars_[i];
        Trip* t = trips_.get(tripOfRecord_[i]);
        // (a car physics carries: its controller's lag is counted in its reaction)
        const std::map<CarId, Carried>::const_iterator carriedIt = carried_.empty() ? carried_.end() : carried_.find(c.id);
        const bool isCarried = carriedIt != carried_.end();
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
        Neighbour n = carAhead(*roads_, cars_, pathLists_, c.id, pathsAfter_, toEnd);
        // LETTING IN (sitting 5): the car I let in is kept behind as if already ahead in my lane, until it is across
        if (const std::map<CarId, CarId>::iterator li = lettingIn_.find(c.id); li != lettingIn_.end()) {
            Real front = 0, rear = 0;
            if (!indicatedInto(li->second, c.path, &front, &rear) || front <= c.along) {
                lettingIn_.erase(li);
            } else if (!n.found() || rear - c.along < n.gap) {
                n.car = li->second;
                n.gap = rear - c.along;
                const Car* other = car(li->second);
                n.speed = other ? other->speed : Real(0);
            }
        }
        // DROPPING BACK (sitting 5): behind the car in the lane beside that closes my gap, where its rear is in my lane;
        // while it is level with me or behind, no faster than it less kDropBackBelow, so that it passes
        Real dropBackTop = kNever;
        if (const std::map<CarId, DropBack>::iterator db = droppingBack_.find(c.id); db != droppingBack_.end()) {
            const Car* other = car(db->second.behind);
            if (!other || other->vehicleClass >= vehicleClasses_.size() ||
                (other->path != db->second.into && other->besidePath != db->second.into)) {
                droppingBack_.erase(db);
            } else {
                const Real theirAlong = other->path == db->second.into ? other->along : other->along + other->besideShift;
                const Real rearHere = theirAlong - vehicleClasses_.at(other->vehicleClass).length - db->second.shift;
                if (rearHere > c.along) {
                    if (!n.found() || rearHere - c.along < n.gap) {
                        n.car = other->id;
                        n.gap = rearHere - c.along;
                        n.speed = other->speed;
                    }
                } else if (other->speed > kDropBackBelow) {
                    dropBackTop = other->speed - kDropBackBelow;
                }
            }
        }
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
        // MOVING ACROSS (sitting 5): part of it is still in the lane it leaves, so it keeps behind the car ahead there too
        if (c.besidePath.valid()) {
            Neighbour old;
            const Real there = c.along + c.besideShift;   // (my front, along the lane I leave)
            for (const PathEntry& e : pathLists_.on(c.besidePath)) {
                if (e.car == c.id || e.rear < there) continue;
                if (!old.found() || e.rear - there < old.gap) {
                    old.car = e.car;
                    old.gap = e.rear - there;
                }
            }
            if (old.found()) {
                const Car* other = findCar(cars_, pathLists_, old.car);
                Obstacle& o = ahead[count++];
                o.kind = Limit::CarAhead;
                o.distance = old.gap;
                o.speed = other ? other->speed : Real(0);
                o.car = old.car;
                o.hardestBrake = other && other->vehicleClass < vehicleClasses_.size()
                                     ? vehicleClasses_.at(other->vehicleClass).hardestBrake
                                     : Real(0);
            }
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
        const StepOutcome step = stepSpeed(f, c.speed, std::min(c.held.topSpeed, dropBackTop), c.held.topSpeedRule, ahead, count, dt,
                                           reaction + (isCarried ? carriedIt->second.lag : Real(0)));

        // WHAT IT WANTS (design section 7): the step's speed and acceleration, and where it means to be at rest
        {
            CarIntent& want = intents_[i];
            want.speed = step.speed;
            want.acceleration = step.acceleration;
            want.binding = step.binding;
            for (std::size_t k = 0; k < count; ++k) {
                const Obstacle& o = ahead[k];
                Real rest = -1;
                if (o.kind == Limit::Stop || (o.kind == Limit::WayEnd && !o.hardOnly)) rest = std::max(Real(0), o.distance);
                else if (o.kind == Limit::CarAhead && o.speed < kStandingBelow)
                    rest = std::max(Real(0), o.distance - standstillGap_);
                if (rest < 0) continue;
                if (!want.hasRest || rest < want.restIn) want.restIn = rest;
                want.hasRest = true;
            }
            intentSet_[i] = 1;
        }

        Car& out = carsNext_[i];
        out.acceleration = step.acceleration;
        out.binding = step.binding;
        // GOING ROUND SOMETHING (ADR-0451): back on its line once past it; a lane car eases sideways to the offset its
        // driver means to drive at (a carried car's host steers there, passOffset)
        // MOVING ACROSS INTO THE LANE BESIDE (sitting 5): the same easing onto the new lane's line, at the pace that
        // takes lane_change_seconds; listed in both lanes until it is there
        const std::map<CarId, LaneMove>::const_iterator moving = laneMoves_.find(c.id);
        const bool crossing = moving != laneMoves_.end() && !moving->second.frontDone;
        if (thingsOn_ && !crossing) {
            Real target = 0, pace = kSwerveSideSpeed;
            if (thingsOn_) {
                const std::map<CarId, ThingWatch>::iterator wi = watch_.find(c.id);
                if (wi != watch_.end() && wi->second.passing) {
                    if (c.odometer >= wi->second.passEnd) wi->second.passing = false;
                    else target = wi->second.passOffset;
                }
            }
            if (!isCarried && out.offset != target) {
                const Real most = pace * dt;
                out.offset = std::fabs(target - out.offset) <= most ? target : out.offset + (target > out.offset ? most : -most);
            }
        }
        if (isCarried) {
            // physics moves it: the record keeps where the host last wrote it is (carriedAt), and its speed
            out.speed = c.speed;
            continue;
        }
        out.speed = step.speed;

        // Along the way: carry what remains past the end of a path onto the next, as many times as it takes. The end
        // of the way is a wall; the hard limit keeps the car short of it, so stopping there is only ever rounding (or
        // a stop the hardest braking could not make).
        // (its speed is along the line it drives: offset from its lane's line on a bend, that line is longer or shorter
        // than the lane by the bend -- a move across, a swerve -- so its front goes along the lane by that much less or more)
        Real perLaneMetre = 1;
        if ((changingLanes() || moving != laneMoves_.end()) && std::fabs(c.offset) > kOffsetOnLine) {
            const Real len = roads_->paths[c.path.index].length;
            const Real a0 = std::clamp(c.along, Real(0), len), a1 = std::min(len, a0 + 1), back = a1 > a0 ? a0 : std::max(Real(0), a0 - 1);
            const Real k = kerbwardPerpSign(ruleSet_.drivesOn) * c.offset;
            const Real from = back < a0 ? back : a0, to = back < a0 ? a0 : a1;
            const engine::Vec2 p0 = roads_->pointAt(c.path, from) + engine::perp(roads_->directionAt(c.path, from)) * k;
            const engine::Vec2 p1 = roads_->pointAt(c.path, to) + engine::perp(roads_->directionAt(c.path, to)) * k;
            const Real lane = to - from, line = (p1 - p0).length();
            if (lane > 0 && line > 0) perLaneMetre = line / lane;
        }
        Real along = c.along + step.speed * dt / perLaneMetre;
        Real moved = step.speed * dt / perLaneMetre;
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
        if (path != c.path) {   // (the old lane ended where its own did)
            out.besidePath = engine::PathId();
            out.besideShift = 0;
        }
        // MOVING ACROSS INTO THE LANE BESIDE (sitting 5): the S-curve by the distance covered; listed in both lanes
        // until it is on the new one's line
        out.offsetRearLag = 0;
        if (const std::map<CarId, LaneMove>::iterator m = laneMoves_.find(c.id); m != laneMoves_.end()) {
            const Real travelled = static_cast<Real>(out.odometer - m->second.startOdometer);
            if (!m->second.frontDone) {
                out.offset = laneMoveOffset(m->second, travelled);
                if (travelled >= m->second.length - kMoveDoneWithin) {
                    out.offset = m->second.to;
                    out.besidePath = engine::PathId();
                    out.besideShift = 0;
                    m->second.frontDone = true;
                    if (!m->second.kerb) ++laneChangeCounts_.finished;
                    else if (m->second.to == 0) ++kerbMoveCounts_.pulledOut;
                }
            }
            // (the rear on the same curve a body length behind: the car points along its path across, as a car steered
            // over does, rather than sliding square; drawn only)
            const Real body = vehicleClasses_.at(c.vehicleClass).length;
            out.offsetRearLag = laneMoveRearOffset(m->second, travelled, body) - out.offset;
            if (m->second.frontDone && travelled - body >= m->second.length) {
                out.offsetRearLag = 0;
                laneMoves_.erase(m);
            }
        }
        // PULLING INTO A KERB SPACE AT THE END (sitting 5): once the destination is as near as the move across and a body
        // length after it, the car moves across on the same curve, so it comes to rest at its destination standing
        // straight in the space. Begun too near (the trip began there, or a move out of a space was still under way): it
        // ends in the lane.
        if (t->pullInOffset != 0 && !t->pullInBegun && onWay) {
            Real toDestination = -1, before = -out.along;
            for (std::size_t k = at; k < way.size(); ++k) {
                if (way[k] == t->destination.path) {
                    toDestination = before + t->destination.along;
                    break;
                }
                before += roads_->paths[way[k].index].length;
            }
            const Real shortest = laneMoveShortest(t->pullInOffset - out.offset, cls.tightestTurnRadius);
            const std::map<CarId, LaneMove>::iterator last = laneMoves_.find(c.id);
            const bool busy = last != laneMoves_.end() && !last->second.frontDone;
            if (toDestination >= 0 && toDestination <= std::max(shortest, out.speed * laneChangeSeconds_) + cls.length) {
                const Real length = toDestination - cls.length;
                if (length >= shortest / kMoveMargin && !busy) {
                    LaneMove move;
                    move.from = out.offset;
                    move.to = t->pullInOffset;
                    move.kerb = true;
                    move.startOdometer = out.odometer;
                    move.length = length;
                    if (last != laneMoves_.end()) {   // (the rear still finishing the move out of a space)
                        move.carryFrom = last->second.from - last->second.to;
                        move.carryStart = last->second.startOdometer;
                        move.carryLength = last->second.length;
                    }
                    laneMoves_[c.id] = move;
                    t->pullInBegun = true;
                } else if (length < shortest / kMoveMargin) {
                    t->pullInBegun = true;
                    t->pullInMissed = true;
                }
            }
        }
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
    classParts_.assign(vehicleClasses_.size(), ClassParts());   // (sized once: routes keep a pointer to their class's)
    if (roads_) {
        for (const VehicleClass& c : vehicleClasses_.all())
            routeClasses_.push_back(bindRouteClass(*roads_, c, &routeNotes_));
        rebuildPathLists(*roads_, vehicleClasses_, cars_, pathLists_);
        closeTurnsForRoom();
    }
}

Route Driving::routeFor(const Car& c, const PathPoint& to, bool exactLane) const {
    return routeFromWhereItIs(c, routeClass(c.vehicleClass, exactLane), to);
}

// A ROUTE FROM WHERE A PLACED CAR'S FRONT IS: as `cls` finds it, unless its first move across does not fit in what is
// left of the lane it stands on (a car at rest at a junction's line, its new way beginning in the lane beside: it could
// never move over, and stood at the end of its way for good -- the near tier's road block, lane changes on). Then a way
// keeping its lane to the end of this stretch, when there is one; otherwise the first, as it was.
Route Driving::routeFromWhereItIs(const Car& c, RouteClass cls, const PathPoint& to) const {
    if (!model_) return Route();
    PathPoint front;
    front.path = c.path;
    front.along = c.along;
    Route route = findRoute(*model_, routeBounds_, cls, front, to);
    if (!route.found || !changingLanes() || cls.keepLaneAtStart || c.path.index >= roads_->paths.size()) return route;
    const LanesToCross need = lanesToCross(route, c.path, 0);
    const engine::Path& lane = roads_->paths[c.path.index];
    const Real shortest = laneMoveShortest(lane.width, vehicleClasses_.at(c.vehicleClass).tightestTurnRadius);
    if (need.inLeg == 0 || lane.length - c.along >= static_cast<Real>(need.inLeg) * shortest) return route;
    cls.keepLaneAtStart = true;
    Route inLane = findRoute(*model_, routeBounds_, cls, front, to);
    return inLane.found ? inLane : route;
}

Route Driving::routeBetween(uint32_t vehicleClass, const PathPoint& from, const PathPoint& to) const {
    if (!model_) return Route();
    return findRoute(*model_, routeBounds_, routeClass(vehicleClass), from, to);
}

std::vector<std::string> Driving::busLineProblems(uint32_t vehicleClass) const {
    std::vector<std::string> out;
    if (!model_) return out;
    for (const engine::ModelBusLine& line : model_->busLines) {
        std::vector<PathPoint> at(line.stops.size());
        bool placed = true;
        for (std::size_t i = 0; i < line.stops.size(); ++i) {
            const engine::ModelBusStop& stop = line.stops[i];
            const engine::KerbLanePlace k = model_->busStopLane(stop);
            if (!k.valid()) {
                out.push_back("bus line " + line.id + ", stop " + stop.id + ": no kerb lane beside it for its bus to pull into");
                placed = false;
                continue;
            }
            at[i].path = k.path;
            at[i].along = k.along;
            if (!model_->busStopWalk(stop).valid())
                out.push_back("bus line " + line.id + ", stop " + stop.id + ": no walk beside it for its people to wait on");
        }
        if (!placed) continue;
        // the legs: each stop to the next, and the last round to the first
        std::vector<std::pair<std::size_t, std::size_t>> legs;
        for (std::size_t i = 0; i < line.stops.size(); ++i) legs.push_back({i, (i + 1) % line.stops.size()});
        for (const std::pair<std::size_t, std::size_t>& leg : legs) {
            const Route r = routeBetween(vehicleClass, at[leg.first], at[leg.second]);
            if (!r.found)
                out.push_back("bus line " + line.id + ": no route for its buses from stop " + line.stops[leg.first].id + " to stop " +
                              line.stops[leg.second].id + " (" + r.whyWords + ")");
        }
    }
    return out;
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

// ---- A car carried out by physics (design section 7; ADR-0447) --------------------------------------------------------

namespace {
// How many paths past the one its front is on a carried car's front is looked for on (one tick never takes it further).
constexpr std::size_t kCarriedLookOn = 3;

// (nearestOn, the nearest point of a path's centre line: things.h)
}  // namespace

const ThingAhead* Driving::thingAhead(CarId car) const {
    const std::map<CarId, ThingWatch>::const_iterator w = watch_.find(car);
    return w != watch_.end() ? &w->second.last : nullptr;
}

Real Driving::passOffset(CarId car) const {
    const std::map<CarId, ThingWatch>::const_iterator w = watch_.find(car);
    return w != watch_.end() && w->second.passing ? w->second.passOffset : Real(0);
}

// ANOTHER WAY ROUND A THING THAT STANDS (ADR-0451): a way from where the car is that does not use the path the thing
// stands on (closed for this one search). None (the thing stands on the car's own path, or nothing else leads there):
// it keeps its way and waits.
// Whether two paths are lanes of one section (a path is in its own).
bool Driving::sameSection(engine::PathId a, engine::PathId b) const {
    if (a == b) return true;
    if (!roads_ || !a.valid() || !b.valid() || a.index >= roads_->paths.size() || b.index >= roads_->paths.size()) return false;
    const engine::SectionId sa = roads_->paths[a.index].section, sb = roads_->paths[b.index].section;
    return sa.valid() && sa == sb;
}

// HOW HARD A CAR WOULD ACCELERATE following something `gap` metres ahead going at `speedAhead`: its own driver's
// following (act.h easingAcceleration, the IDM), with its class and its driver's personal gap. A car nobody drives is
// read with the usual gap. (MOBIL's safety criterion asks it of the car that would be behind a lane change.)
Real Driving::followAcceleration(const Car& who, Real gap, Real speedAhead) const {
    if (who.vehicleClass >= vehicleClasses_.size()) return 0;
    const VehicleClass& cls = vehicleClasses_.at(who.vehicleClass);
    PersonId person = 0;
    if (const std::map<CarId, TripId>::const_iterator ti = tripOfCar_.find(who.id); ti != tripOfCar_.end())
        if (const Trip* t = trips_.get(ti->second)) person = t->person;
    Follower f;
    f.speedUp = cls.speedUp;
    f.brake = cls.brake;
    f.hardestBrake = cls.hardestBrake;
    f.gapKept = static_cast<Real>(personalValue(gapKept_, gapKeptHash_, person));
    f.standstillGap = standstillGap_;
    f.anticipation = anticipation_;
    const Real top = who.driven && who.held.decided ? who.held.topSpeed : cls.topSpeed;
    return easingAcceleration(f, who.speed, std::max(top, who.speed), gap, speedAhead);
}

// A CAR'S LANE, after its decision (sitting 5). Its rules said "change" and the gap beside is safe: it moves into the
// lane beside where it is -- its front at the same share of that lane's length -- still on its old line (an offset
// from the new lane's), listed in both lanes, and eases across in the move step. No gap before its last chance, and
// none to wait for without standing in a through lane: it carries on in its own lane, routed again from there.
// WHERE ANOTHER PATH MEETS THE LANE I WOULD MOVE INTO (a ramp's lane joining it, a lane crossing it): I do not move
// across into that stretch while a car on the other path is announced to be on it before I would be clear of it -- it
// may no longer be able to stop for me (UK Highway Code rule 133: do not change lane where that would make another driver
// change course or speed). Read off this tick's board, by the comfortable gap a driver decides to go into a meeting by.
// (The 20-minute lab: a car moved across into a lane 30 m from where a ramp joined it, 0.4 s ahead of the ramp's car,
// which could no longer stop; they overlapped by 1.8 m.)
// WAITING TO MERGE (sitting 5, the zipper at a merge, #323): a car held to give way within kMergeWaitWithin of where its
// lane joins another out on the road (a ramp's lane into its host's) shows the drivers there that it wants in, as a car
// wanting the lane beside does: its front projected onto their lane along the join's matched stretch. A driver there
// may let it in (the rule "let in"): it keeps behind that projected rear, and the waiting car, seeing it hold back for
// it, does not give way to it (GiveWayInputs::lettingIn). Without it a queue crawling along the host lane never left a
// gap and the ramp's car waited for good (the 20-minute lab: 836 s).
Driving::Indicating Driving::mergeWait(const Car& c) const {
    Indicating none;
    if (!c.held.hasStop || !givesWay(c.held.stopFor) || !c.path.valid() || c.path.index >= roads_->paths.size()) return none;
    if (static_cast<Real>(c.held.stopAt - c.odometer) > kMergeWaitWithin) return none;
    const engine::Path& p = roads_->paths[c.path.index];
    for (uint32_t x = p.crossings.first; x < p.crossings.end() && x < roads_->pathCrossings.size(); ++x) {
        const engine::CrossingPoint& rec = roads_->crossings[roads_->pathCrossings[x].index];
        if (rec.otherSide != engine::OtherSide::Path || rec.meeting != engine::Meeting::Join || rec.junction.valid()) continue;
        const bool mineIsA = rec.a == c.path;
        const Real myFrom = mineIsA ? rec.aFrom : rec.otherFrom, myTo = mineIsA ? rec.aTo : rec.otherTo;
        const Real theirFrom = mineIsA ? rec.otherFrom : rec.aFrom, theirTo = mineIsA ? rec.otherTo : rec.aTo;
        if (c.along > myTo || myFrom - c.along > kMergeWaitWithin) continue;   // (past it, or not near it)
        const Real scale = myTo > myFrom ? (theirTo - theirFrom) / (myTo - myFrom) : Real(1);
        Indicating in;
        in.into = mineIsA ? rec.otherPath() : rec.a;
        in.shift = theirFrom + (c.along - myFrom) * scale - c.along;
        in.merging = true;
        return in;
    }
    return none;
}

int Driving::signalSide(CarId car) const {
    bool towardKerb = false;
    if (const std::map<CarId, LaneMove>::const_iterator m = laneMoves_.find(car);
        m != laneMoves_.end() && !m->second.frontDone && m->second.from != m->second.to)
        towardKerb = m->second.to > m->second.from;   // (offsets from the new lane's line, + toward the kerb)
    else if (const std::map<CarId, Indicating>::const_iterator i = indicating_.find(car);
             i != indicating_.end() && !i->second.merging)
        towardKerb = i->second.towardKerb;
    else
        return 0;
    // (the kerb is the driver's left where traffic drives on the left)
    return towardKerb == (ruleSet_.drivesOn == DrivesOn::Left) ? -1 : 1;
}

bool Driving::indicatedInto(CarId who, engine::PathId into, Real* front, Real* rear) const {
    const std::map<CarId, Indicating>::const_iterator ind = indicating_.find(who);
    if (ind == indicating_.end() || ind->second.into != into) return false;
    const Car* other = car(who);
    if (!other || other->vehicleClass >= vehicleClasses_.size()) return false;
    *front = other->along + ind->second.shift;
    *rear = *front - vehicleClasses_.at(other->vehicleClass).length;
    return true;
}

bool Driving::landsInAMeeting(const Car& c, const LaneGap& gap, Real length) const {
    if (!gap.target.valid() || gap.target.index >= roads_->paths.size()) return false;
    const engine::Path& lane = roads_->paths[gap.target.index];
    const Real speed = std::max(c.speed, Real(1));
    const Real rear = gap.targetAlong - length;
    const Real gapNeeded = junctionNumbers_.gapComfortable;
    for (uint32_t x = lane.crossings.first; x < lane.crossings.end(); ++x) {
        const engine::CrossingId id = roads_->pathCrossings[x];
        const engine::CrossingPoint& cp = roads_->crossings[id.index];
        if (cp.otherSide != engine::OtherSide::Path) continue;
        const bool mineIsA = cp.a == gap.target;
        const Real from = mineIsA ? cp.aFrom : cp.otherFrom, to = mineIsA ? cp.aTo : cp.otherTo;
        if (to < rear) continue;   // behind me there
        const Real arrive = std::max(Real(0), from - gap.targetAlong) / speed;
        if (arrive > junctionNumbers_.lookAheadTime) continue;
        const Real clear = (to - rear) / speed;
        const engine::PathId other = mineIsA ? cp.otherPath() : cp.a;
        for (const Announcement& a : board_.on(id)) {
            if (a.who == c.id.index || a.onPath != other) continue;
            if (a.arriveIn < clear + gapNeeded && a.clearIn + gapNeeded > arrive) return true;
        }
    }
    return false;
}

void Driving::changeLanes(Car& c, Trip& t, const Planned& p) {
    const LaneChangeView& l = p.lane;
    if (!(l.need.onRoute && l.need.lanes > 0)) return;
    // (not while its front is still coming out of a kerb space, or going into one)
    if (const std::map<CarId, LaneMove>::const_iterator k = laneMoves_.find(c.id);
        k != laneMoves_.end() && k->second.kerb && !k->second.frontDone)
        return;
    LaneGap gap = l.gap;
    if (p.changeLane && gap.safe && listsStale_) {
        // (a car before it in this tick's order has already moved over: it looks again, and backs off if that took
        // its gap -- each driver sees the other start to move, nothing hands gaps out)
        rebuildPathLists(*roads_, vehicleClasses_, cars_, pathLists_);
        listsStale_ = false;
        gap = findLaneGap(*roads_, cars_, pathLists_, c, vehicleClasses_.at(c.vehicleClass).length,
                          l.need.toward == Across::TowardKerb, laneChangeBrake_, kerbwardPerpSign(ruleSet_.drivesOn),
                          pathsBefore_, [&](const Car& who, Real g, Real speedAhead) { return followAcceleration(who, g, speedAhead); });
        if (!gap.safe) ++laneChangeCounts_.backedOff;
    }
    // (moving: a car at rest cannot step sideways, and the move across is laid by the distance it covers)
    // (done in time to begin the moves still needed after it: a move laid over all the room left stranded a car two
    // lanes from its turn at the end of the lane between, the second move no longer fitting)
    bool fits = false;
    const Real radius = vehicleClasses_.at(c.vehicleClass).tightestTurnRadius;
    Real room = gap.target.valid() ? roads_->paths[gap.target.index].length - gap.targetAlong : Real(0);
    // (not for a driver already waiting for its gap: it has no better place to be, takes the move that fits, and meets
    // the next one from the lane it moves into -- held to room for both, a car stopped half-way along a short lane two
    // changes from its turn could never move at all: the 20-minute lab's longest queues)
    if (gap.target.valid() && l.last.needed && l.need.inLeg > 1 && !t.waitToChange)
        room = std::min(room, l.last.metres - static_cast<Real>(l.need.inLeg - 1) *
                                                  laneMoveShortest(roads_->paths[gap.target.index].width, radius));
    const Real moveLength =
        gap.target.valid() ? laneMoveLength(gap.across, c.speed, laneChangeSeconds_, radius, room, &fits) : Real(0);
    // (a driver waiting for its gap with no other way on moves across from rest: the move is laid by distance)
    if (p.changeLane && gap.safe && gap.target.valid() && fits && (c.speed >= kLaneChangeLeastSpeed || t.waitToChange)) {
        Real looked = -c.along;   // (how far its way ahead reached: the new one reaches as far)
        for (const engine::PathId path : t.way.paths) looked += roads_->paths[path.index].length;
        t.waitToChange = false;
        indicating_.erase(c.id);
        droppingBack_.erase(c.id);
        c.besidePath = c.path;
        c.besideShift = c.along - gap.targetAlong;
        c.path = gap.target;
        c.along = gap.targetAlong;
        c.offset = gap.newOffset;
        c.trailCount = 0;   // (its body lies in the new lane whole: findLaneGap asked)
        for (engine::PathId& old : c.trail) old = engine::PathId();
        LaneMove move;
        move.from = c.offset;
        move.startOdometer = c.odometer;
        move.length = moveLength;
        move.leftLane = c.besidePath;
        // (begun while the last move's rear is still crossing: the rear finishes that curve as it begins this one)
        if (const std::map<CarId, LaneMove>::iterator last = laneMoves_.find(c.id); last != laneMoves_.end()) {
            move.carryFrom = last->second.from;
            move.carryStart = last->second.startOdometer;
            move.carryLength = last->second.length;
        }
        laneMoves_[c.id] = move;
        // (what it holds was decided in the old lane: it decides afresh, from the new one, on the next tick)
        decideNext_.insert(std::lower_bound(decideNext_.begin(), decideNext_.end(), c.id), c.id);
        t.way = wayAhead(t.route, *roads_, c.path, c.along, looked, t.legReached);
        t.wayAt = 0;
        Real reach = -c.along;
        for (const engine::PathId path : t.way.paths) reach += roads_->paths[path.index].length;
        t.wayEnds = reach < looked && t.way.paths.size() < kMaxFollowedPaths;
        if (t.wayEnds && !t.way.paths.empty()) {   // (as the plan step reads it: only a lane that ends, or the destination's)
            const engine::PathId last = t.way.paths.back();
            if (!sameSection(last, t.destination.path) && roads_->paths[last.index].next.count > 0) t.wayEnds = false;
        }
        listsStale_ = true;
        ++laneChangeCounts_.begun;
        return;
    }
    const VehicleClass& cls = vehicleClasses_.at(c.vehicleClass);
    // (asked for a way on as it nears where a move across must begin at the latest, while one still fits)
    if (!l.last.needed || l.changeBy > stoppingDistance(c.speed, cls.brake) + kLastChanceMargin) return;
    if (t.missedOn == c.path) return;   // (asked once from this lane already)
    t.missedOn = c.path;
    // THE MISSED TURN: routed again from where it is, keeping to its lane to the end of this stretch, and free to
    // change lanes again past the junction. (Not by closing the stretch's other lanes to the whole route: a way round
    // the block often comes back along this same road, in the lane it missed.)
    ++laneChangeCounts_.missed;
    RouteClass inLane = routeClass(c.vehicleClass, t.exactLane);
    inLane.keepLaneAtStart = true;
    PathPoint front;
    front.path = c.path;
    front.along = c.along;
    Route route = findRoute(*model_, routeBounds_, inLane, front, t.destination);
    // (none keeping its lane -- its destination just behind it in the lane beside, overshot, say: any way on, round the
    // block; the router plans no change from where it stands without room for one)
    // -- taken only if it keeps its lane from here, as the first did (the change it missed is no way on)
    if (!route.found) {
        Route round = findRoute(*model_, routeBounds_, routeClass(c.vehicleClass, t.exactLane), front, t.destination);
        if (round.found && lanesToCross(round, c.path, 0).lanes == 0) route = std::move(round);
    }
    if (!route.found) {
        // (no way on from its lane to where it goes: it waits in it for a gap, stopped where a move across still fits --
        // the one place a driver stops in a through lane to change lanes. The host is NOT told "no route": the trip goes
        // on, and a resident's brain told so planned without the car and got out, leaving it standing in the lane for good
        // -- river_town's car 365, 1.5 m past junction 11, and the queue behind it: #326)
        ++laneChangeCounts_.missedNoRoute;
        t.waitToChange = true;
        return;
    }
    route.vehicle = routeClass(c.vehicleClass, t.exactLane);
    t.route = std::move(route);
    t.legReached = 0;
    c.held.decided = false;
    dropPlansForNewRoute(t);
}

void Driving::askAnotherWayRound(Car& c, Trip& t, const ThingAhead& thing) {
    ++thingCounts_.anotherWaysWanted;
    if (!thing.path.valid() || thing.path == c.path || thing.path.index >= roads_->paths.size()) return;
    // (a route is closed by its turns: the connector the thing stands on, or every connector into the lane it stands on)
    RouteClass cls = routeClass(c.vehicleClass, t.exactLane);
    const auto close = [&](engine::PathId p) {
        const std::vector<engine::PathId>::iterator at = std::lower_bound(cls.closed.begin(), cls.closed.end(), p);
        if (at == cls.closed.end() || *at != p) cls.closed.insert(at, p);
    };
    if (roads_->paths[thing.path.index].kind == engine::PathKind::Connector) {
        close(thing.path);
    } else {
        close(thing.path);   // (and the lane itself: with lane changes a route could move across into it beyond the turns)
        for (uint32_t k = 0; k < roads_->paths.size(); ++k) {
            const engine::Path& P = roads_->paths[k];
            if (P.kind != engine::PathKind::Connector) continue;
            for (uint32_t n = P.next.first; n < P.next.end() && n < roads_->nextPaths.size(); ++n)
                if (roads_->nextPaths[n] == thing.path) close(engine::PathId(k));
        }
    }
    Route route = routeFromWhereItIs(c, cls, t.destination);
    if (!route.found) return;
    route.vehicle = routeClass(c.vehicleClass, t.exactLane);
    ++thingCounts_.anotherWays;
    t.route = std::move(route);
    t.legReached = 0;
    c.held.decided = false;
    dropPlansForNewRoute(t);
}

namespace {
}  // namespace

bool Driving::carry(CarId id, bool carriedNow, Real lag) {
    Car* c = findPlaced(id);
    if (!c) return false;
    if (carriedNow) {
        carried_[id].lag = std::max(Real(0), lag);
        // (physics has it now: a move across ends where it is; the host steers it on its own line)
        if (laneMoves_.erase(id) > 0) {
            c->besidePath = engine::PathId();
            c->besideShift = 0;
            listsStale_ = true;
        }
    } else if (carried_.erase(id) > 0) {
        seen_.erase(id);
        // back on its lane: at the nearest point along it, on its centre line, at the speed last written
        c->offset = 0;
        c->offsetRearLag = 0;
        listsStale_ = true;
    }
    return true;
}

bool Driving::carriedAt(CarId id, const engine::Vec2& front, Real speed) {
    if (!roads_ || !carried(id)) return false;
    Car* c = findPlaced(id);
    const std::map<CarId, TripId>::const_iterator ti = tripOfCar_.find(id);
    Trip* t = ti != tripOfCar_.end() ? trips_.get(ti->second) : nullptr;
    if (!c || !t || c->vehicleClass >= vehicleClasses_.size()) return false;
    const std::vector<engine::PathId>& way = t->way.paths;
    const bool onWay = t->wayAt < way.size() && way[t->wayAt] == c->path;

    // the nearest centre line: its own path, then the paths after it on its way
    std::vector<engine::PathId> look{c->path};
    if (onWay)
        for (std::size_t k = t->wayAt + 1; k < way.size() && k <= t->wayAt + kCarriedLookOn; ++k) look.push_back(way[k]);
    std::size_t best = look.size();
    OnLine at;
    for (std::size_t k = 0; k < look.size(); ++k) {
        const OnLine o = nearestOn(*roads_, look[k], front);
        // (a tie goes to the later path: the car has moved onto it)
        if (o.away < 0 || (best < look.size() && o.away > at.away)) continue;
        best = k;
        at = o;
    }
    if (best == look.size() || at.away > kCarriedReach) return false;

    // how far its front moved along the way, and the paths it passed
    Real moved = at.along - c->along;
    std::array<engine::PathId, kMaxTrail> crossed;
    std::size_t crossedCount = 0;
    for (std::size_t k = 0; k < best; ++k) {
        moved += roads_->paths[look[k].index].length;
        for (std::size_t m = std::min(crossedCount, kMaxTrail - 1); m > 0; --m) crossed[m] = crossed[m - 1];
        crossed[0] = look[k];
        crossedCount = std::min(crossedCount + 1, kMaxTrail);
    }
    c->path = look[best];
    c->along = at.along;
    c->offset = at.side;
    c->speed = std::max(Real(0), speed);
    c->odometer += moved;
    if (onWay) t->wayAt = static_cast<uint32_t>(t->wayAt + best);
    // the trail: the paths just passed, latest first, then the old trail; only those the body still reaches
    std::array<engine::PathId, kMaxTrail> trail;
    std::size_t trailCount = 0;
    for (std::size_t k = 0; k < crossedCount && trailCount < kMaxTrail; ++k) trail[trailCount++] = crossed[k];
    for (std::size_t k = 0; k < c->trailCount && trailCount < kMaxTrail; ++k) trail[trailCount++] = c->trail[k];
    c->trail = trail;
    c->trailCount = static_cast<uint8_t>(trailCount);
    const Body body = layBody(*roads_, *c, vehicleClasses_.at(c->vehicleClass).length);
    c->trailCount = static_cast<uint8_t>(body.count > 0 ? body.count - 1 : 0);
    for (std::size_t k = c->trailCount; k < kMaxTrail; ++k) c->trail[k] = engine::PathId();
    listsStale_ = true;
    return true;
}

const CarIntent* Driving::intent(CarId id) const {
    const Car* c = car(id);
    if (!c || intents_.size() != cars_.size()) return nullptr;
    const std::size_t i = static_cast<std::size_t>(c - cars_.data());
    return intentSet_[i] ? &intents_[i] : nullptr;
}

std::vector<engine::PathId> Driving::wayFromFront(CarId id) const {
    std::vector<engine::PathId> out;
    const std::map<CarId, TripId>::const_iterator ti = tripOfCar_.find(id);
    const Trip* t = ti != tripOfCar_.end() ? trips_.get(ti->second) : nullptr;
    if (!t) return out;
    for (std::size_t k = t->wayAt; k < t->way.paths.size(); ++k) out.push_back(t->way.paths[k]);
    return out;
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
    intents_.clear();   // (by record: the records moved)
    intentSet_.clear();
    rebuildPathLists(*roads_, vehicleClasses_, cars_, pathLists_);
    return true;
}

bool Driving::placeCarInSpace(CarId id, uint32_t vehicleClass, const PathPoint& front, Real offset, std::string* why) {
    if (!placeCar(id, vehicleClass, front, {}, why)) return false;
    Car* c = findPlaced(id);
    c->offset = offset;
    layPullOut(*c);
    return true;
}

bool Driving::atTheKerb(CarId id, Real within) const {
    const Car* c = car(id);
    if (!c) return false;
    if (const std::map<CarId, LaneMove>::const_iterator m = laneMoves_.find(id); m != laneMoves_.end() && m->second.kerb)
        return true;
    if (inSpace_.count(id) != 0) return true;
    const std::map<CarId, TripId>::const_iterator ti = tripOfCar_.find(id);
    const Trip* t = ti == tripOfCar_.end() ? nullptr : trips_.get(ti->second);
    if (!t || t->pullInOffset == 0 || t->pullInMissed) return false;
    // (how far on its way its destination is; not on the way yet: farther than any reach worth asking about)
    Real before = -c->along;
    for (std::size_t k = t->wayAt; k < t->way.paths.size(); ++k) {
        if (t->way.paths[k] == t->destination.path) return before + t->destination.along <= within;
        before += roads_->paths[t->way.paths[k].index].length;
    }
    return false;
}

Real Driving::offsetBeside(const PathPoint& at, engine::Vec2 point) const {
    if (!roads_ || !onRoad(at)) return 0;
    const engine::Vec2 side = engine::perp(roads_->directionAt(at.path, at.along));
    return engine::dot(point - roads_->pointAt(at.path, at.along), side) * kerbwardPerpSign(ruleSet_.drivesOn);
}

// THE MOVE OUT OF A KERB SPACE onto the lane's line, for a car standing beside its lane (placed in a space, or one whose
// last trip ended in one): the lane change's curve at the tightest its turning circle allows, driven as it moves off.
void Driving::layPullOut(const Car& c) {
    if (std::fabs(c.offset) <= kOffsetOnLine || laneMoves_.count(c.id) != 0) return;
    LaneMove move;
    move.from = c.offset;
    move.to = 0;
    move.kerb = true;
    move.startOdometer = c.odometer;
    move.length = laneMoveShortest(c.offset, vehicleClasses_.at(c.vehicleClass).tightestTurnRadius);
    laneMoves_[c.id] = move;
}

bool Driving::removeCar(CarId id, std::string* why) {
    if (!car(id)) return refuse(why, carName(id) + " is not placed");
    if (tripOfCar_.count(id) != 0) return refuse(why, carName(id) + " is on a trip: end the trip first");
    std::vector<Car>::iterator at =
        std::lower_bound(cars_.begin(), cars_.end(), id, [](const Car& x, CarId want) { return x.id < want; });
    cars_.erase(at);
    carried_.erase(id);
    laneMoves_.erase(id);
    inSpace_.erase(id);
    // Everything else the driving keeps per car goes with it: a later car may be given the same id (the alpha review)
    watch_.erase(id);
    seen_.erase(id);
    crosswalkOf_.erase(id);
    wrecks_.erase(id);
    for (std::set<std::pair<uint32_t, uint32_t>>::iterator t = touching_.begin(); t != touching_.end();)
        t = t->first == id.index || t->second == id.index ? touching_.erase(t) : std::next(t);
    indicating_.erase(id);
    lettingIn_.erase(id);
    droppingBack_.erase(id);
    for (std::map<CarId, DropBack>::iterator db = droppingBack_.begin(); db != droppingBack_.end();)
        db = db->second.behind == id ? droppingBack_.erase(db) : std::next(db);
    for (std::map<CarId, CarId>::iterator li = lettingIn_.begin(); li != lettingIn_.end();)
        li = li->second == id ? lettingIn_.erase(li) : std::next(li);
    intents_.clear();   // (by record: the records moved)
    intentSet_.clear();
    if (roads_) rebuildPathLists(*roads_, vehicleClasses_, cars_, pathLists_);
    return true;
}

TripId Driving::beginTrip(CarId car, PersonId person, const PathPoint& destination, Real pullInOffset, bool exactLane) {
    Car* placed = findPlaced(car);
    if (!placed || tripOfCar_.count(car) != 0 || !onRoad(destination)) return TripId{};
    exactLane = exactLane || pullInOffset != 0;
    Route route = routeFor(*placed, destination, exactLane);
    if (!route.found) {
        queueEvent(EventKind::NoRoute, car, TripId{}, person);
        return TripId{};
    }
    placed->driven = true;
    placed->held = HeldAnswers();   // nothing decided yet: it decides on its next tick
    if (inSpace_.erase(car) != 0) layPullOut(*placed);   // (its last trip ended in a kerb space: it drives out first)
    placed->binding = Binding();
    Trip t;
    t.car = car;
    t.person = person;
    t.destination = destination;
    t.pullInOffset = pullInOffset;
    t.exactLane = exactLane;
    t.beganAtTick = tickCount_;
    t.plannedMetres = route.metres;
    t.plannedSeconds = route.seconds;
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

bool Driving::changeDestination(TripId id, const PathPoint& destination, Real pullInOffset, bool exactLane) {
    Trip* t = trips_.get(id);
    if (!t || !onRoad(destination)) return false;
    const Car* c = car(t->car);
    if (!c) return false;
    exactLane = exactLane || pullInOffset != 0;
    Route route = routeFor(*c, destination, exactLane);
    if (!route.found) {
        queueEvent(EventKind::NoRoute, t->car, id, t->person);
        return false;
    }
    t->destination = destination;
    t->pullInOffset = pullInOffset;
    t->exactLane = exactLane;
    t->pullInBegun = false;
    t->pullInMissed = false;
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
    RouteClass cls = routeClass(c.vehicleClass, t.exactLane);
    const std::vector<engine::PathId>::iterator at = std::lower_bound(cls.closed.begin(), cls.closed.end(), refused->connector);
    if (at == cls.closed.end() || *at != refused->connector) cls.closed.insert(at, refused->connector);
    Route route = routeFromWhereItIs(c, cls, t.destination);
    if (!route.found) return;   // no other way: it keeps its own and waits on (the measurements report a wait that cannot end)
    // ... that it can take now: the first turn of the new way, where lights run there, shows green (a driver gives up its
    // turn for one it sees it may take, not for another red; otherwise it waits on and asks again later).
    const engine::PathId firstTurn = !route.turns.empty() ? route.turns[0].connector : route.endConnector;
    if (lightsRun_ && firstTurn.valid() && lampAt(*roads_, rightOfWay_, firstTurn, clockSeconds_) != Lamp::Green &&
        lampAt(*roads_, rightOfWay_, firstTurn, clockSeconds_) != Lamp::Dark)
        return;
    route.vehicle = routeClass(c.vehicleClass, t.exactLane);   // later questions of the route see the class as it is
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
    Route route = routeFor(*c, t->destination, t->exactLane);
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
        if (changingLanes()) h.pod(c.besideShift);
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
    // (how often drivers decide, as the tick runs it: the decide interval in ticks of the last tick's length -- the
    // number the hash has always folded here, so a run at one tick length hashes as it did when ticks were stated)
    h.pod(settings_.decideEveryTicks(step_));
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
        // A missed turn (sitting 5): only with the lane change step, so a run without it hashes as it did.
        if (changingLanes()) h.pod(t->missedOn.index);
        if (changingLanes()) h.pod(static_cast<uint8_t>(t->waitToChange));
        if (t->exactLane) h.pod(static_cast<uint8_t>(1));   // (a trip that must end in its own lane)
        if (t->pullInOffset != 0) {   // (a trip ending in a kerb space; one ending in the lane hashes as before)
            h.pod(t->pullInOffset);
            h.pod(static_cast<uint8_t>(t->pullInBegun));
            h.pod(static_cast<uint8_t>(t->pullInMissed));
        }
    }
    // The cars moving across into the lane beside, and their move (sitting 5); and who decides again next tick.
    if (changingLanes())
        for (const CarId id : decideNext_) h.pod(id.index);
    for (const std::pair<const CarId, LaneMove>& r : laneMoves_) {
        if (r.second.frontDone) continue;   // (a rear still easing over is drawn, not decided by)
        if (!changingLanes() && !r.second.kerb) continue;
        h.pod(r.first.index);
        h.pod(r.second.from);
        h.pod(r.second.startOdometer);
        h.pod(r.second.length);
        if (r.second.kerb) h.pod(r.second.to);   // (pulling out of or into a kerb space: a run with none hashes as before)
    }
    for (const CarId id : inSpace_) h.pod(id.index);   // (standing in a kerb space)
    for (const std::pair<const CarId, Wreck>& w : wrecks_) {   // (wrecks, sitting 7: a run with none hashes as before)
        h.pod(w.first.index);
        h.pod(w.second.clearAtTick);
    }
    for (const std::pair<uint32_t, uint32_t>& k : touching_) {
        h.pod(k.first);
        h.pod(k.second);
    }
    if (changingLanes()) {   // (letting in, sitting 5)
        for (const std::pair<const CarId, Indicating>& r : indicating_) {
            h.pod(r.first.index);
            h.pod(r.second.into.index);
            h.pod(r.second.shift);
        }
        for (const std::pair<const CarId, CarId>& r : lettingIn_) {
            h.pod(r.first.index);
            h.pod(r.second.index);
        }
        for (const std::pair<const CarId, DropBack>& r : droppingBack_) {
            h.pod(r.first.index);
            h.pod(r.second.behind.index);
            h.pod(r.second.into.index);
            h.pod(r.second.shift);
        }
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
    // (a split's or a road merge's two paths: carAhead follows a car on the other one over the stretch, followedAcross)
    const auto splitApart = [&](engine::PathId a, engine::PathId b) {
        if (!knownPath(*roads_, a) || !knownPath(*roads_, b)) return false;
        const engine::Range xs = roads_->paths[a.index].crossings;
        for (uint32_t x = xs.first; x < xs.end() && x < roads_->pathCrossings.size(); ++x) {
            const engine::CrossingPoint& rec = roads_->crossings[roads_->pathCrossings[x].index];
            if (!followedAcross(rec)) continue;
            if ((rec.a == a && rec.otherPath() == b) || (rec.a == b && rec.otherPath() == a)) return true;
        }
        return false;
    };
    const auto onMyWay = [&](const Car& other) {
        const auto touches = [&](engine::PathId p) {
            if (splitApart(c.path, p)) return true;
            for (std::size_t k = t->wayAt; k < t->way.paths.size(); ++k) {
                const engine::PathId w = t->way.paths[k];
                if (w == p || splitApart(w, p)) return true;
                if (knownPath(*roads_, w) && knownPath(*roads_, p) &&
                    roads_->paths[w.index].kind == engine::PathKind::Connector &&
                    roads_->paths[p.index].kind == engine::PathKind::Connector &&
                    roads_->paths[w.index].from == roads_->paths[p.index].from)
                    return true;
            }
            return p == c.path || (c.besidePath.valid() && p == c.besidePath);   // (moving across: the lane I leave too)
        };
        if (touches(other.path)) return true;
        if (other.besidePath.valid() && touches(other.besidePath)) return true;   // (moving across, still in my lane)
        // (its front across, its rear still leaving my lane)
        if (const std::map<CarId, LaneMove>::const_iterator m = laneMoves_.find(other.id);
            m != laneMoves_.end() && m->second.leftLane.valid() && touches(m->second.leftLane))
            return true;
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
        // (the car I let in, indicating into my lane ahead of me: I keep behind where it would be)
        const std::map<CarId, CarId>::const_iterator li = lettingIn_.find(c.id);
        const std::map<CarId, DropBack>::const_iterator db = droppingBack_.find(c.id);
        const bool lettingItIn = (li != lettingIn_.end() && li->second == b.car) ||
                                 (db != droppingBack_.end() && db->second.behind == b.car);   // (or dropping back behind it)
        if (other && !lettingItIn && !onMyWay(*other))
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
            const uint32_t every = settings_.decideEveryTicks(step_);
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
            const double interval = static_cast<double>(settings_.decideEveryTicks(step_) + 1) * step_;
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
        // (or where its route needs another lane of this road: the way stops with the lane it must leave -- a move
        // across ended at the lane's end, its next one still to come)
        if (!t->wayEnds && !(changingLanes() && lanesToCross(t->route, c.path, t->legReached).lanes > 0))
            return no("the end of a way that does not end");
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
    return describeJunctionRules(*model_, ruleSet_, rightOfWay_, junction);
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
