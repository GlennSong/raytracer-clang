#include "city_driving.h"

#include "city_sim.h"

#include "../driving/act.h"            // registerMoveNeeds
#include "../driving/give_way.h"       // registerJunctionFacts
#include "../driving/room.h"           // registerRoomFacts
#include "../driving/road_rules.h"
#include "../../engine/script_assets.h"   // loadScriptCode, resolveScriptPath
#ifdef RT_ENABLE_SCRIPTING
#include "../driving/scripting/rule_set_lua.h"
#include "../driving/scripting/rules_lua.h"
#include "../../engine/scripting/script_vm.h"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace citysim {

namespace {

constexpr double kMillisecondsPerSecond = 1000.0;

double msSince(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() * kMillisecondsPerSecond;
}

}  // namespace

// ---- The rules -------------------------------------------------------------------------------------------------------

bool loadDrivingRules(const std::string& file, driving::Driving& d, std::string* why) {
#ifdef RT_ENABLE_SCRIPTING
    const auto refuse = [&](const std::string& reason) {
        if (why) *why = reason;
        return false;
    };
    if (file.empty()) return refuse("the level names no rule set (citysim.drivingRules)");
    if (!d.roads()) return refuse("the new model has no road to bind the rule set " + file + " to");
    const std::string code = engine::loadScriptCode(file, "");
    if (code.empty()) return refuse(file + ": not found under assets/scripts (or it is empty)");
    driving::FactRegistry facts;
    std::vector<std::string> problems;
    std::string reason;
    if (!driving::registerBuiltInFacts(facts, &problems) || !driving::registerMoveNeeds(facts, &reason)) {
        problems.push_back(reason);
        std::string all;
        for (const std::string& p : problems) all += (all.empty() ? "" : "; ") + p;
        return refuse("the model's own facts would not register: " + all);
    }
    engine::ScriptVM vm;
    std::string err;
    if (!vm.doString(code, &err)) return refuse(file + ": " + err);
    driving::RuleSet ruleSet;
    driving::AnswerVocabulary vocabulary;
    driving::CompiledRules rules;
    if (!driving::loadRuleSet(vm, file, ruleSet, &err)) return refuse(err);
    // The junction step: announcing, and giving way (#262); its sign_rank fact speaks the rule set's ranks.
    if (!driving::registerJunctionFacts(facts, ruleSet.ranks, &problems)) {
        std::string all;
        for (const std::string& p : problems) all += (all.empty() ? "" : "; ") + p;
        return refuse("the junction step's facts would not register: " + all);
    }
    // Room beyond a junction (#264): asked for and granted one car at a time; nobody enters a box it cannot leave.
    if (!driving::registerRoomFacts(facts, &problems)) {
        std::string all;
        for (const std::string& p : problems) all += (all.empty() ? "" : "; ") + p;
        return refuse("the room step's facts would not register: " + all);
    }
    // The lights' clock (#261): a rule set with lights has them run a programme (without it they would be dark).
    if (ruleSet.anyLights() && !driving::registerLightClock(facts, &problems)) {
        std::string all;
        for (const std::string& p : problems) all += (all.empty() ? "" : "; ") + p;
        return refuse("the light clock would not register: " + all);
    }
    if (!driving::loadRules(vm, file, facts, ruleSet, vocabulary, rules, &err)) return refuse(err);
    driving::RoadRules roadRules;
    if (!driving::buildRoadRules(*d.roads(), ruleSet, facts, roadRules, &problems)) {
        std::string all;
        for (const std::string& p : problems) all += (all.empty() ? "" : "; ") + p;
        return refuse(file + ": does not fit the road: " + all);
    }
    if (!d.useRules(std::move(ruleSet), std::move(facts), std::move(vocabulary), std::move(rules), std::move(roadRules),
                    &err))
        return refuse(file + ": " + err);
    return true;
#else
    (void)file;
    (void)d;
    if (why) *why = "this build has no scripting, so no rule set can be read";
    return false;
#endif
}

// ---- The hand-over ---------------------------------------------------------------------------------------------------

const driving::TripPlaces& DrivingHost::placesFor(const driving::Driving& d, uint32_t cls) {
    if (cls >= placesOfClass_.size()) {
        placesOfClass_.resize(cls + 1);
        placesFound_.resize(cls + 1, 0);
    }
    if (!placesFound_[cls]) {
        // As the model routes the class: turns it could never be granted room through closed (#264).
        const driving::RouteClass rc = d.routeClass(cls);
        // With room beyond (#264) a destination is at least the room this class needs from either end of its lane, so
        // a car stopping there has its whole body clear of the junction box behind it (and of the next one's line).
        const Real margin = d.roomStep() ? std::max(kPlaceMargin, driving::roomNeeded(d.vehicleClasses().at(cls),
                                                                                       d.roomNumbers().standstillGap))
                                         : kPlaceMargin;
        placesOfClass_[cls] = driving::findTripPlaces(*d.roads(), rc, margin, kSmallestTripPart);
        placesFound_[cls] = 1;
    }
    return placesOfClass_[cls];
}

bool DrivingHost::putDown(driving::Driving& d, Seat& s, Real room) {
    const driving::TripPlacing p = driving::putCarSomewhere(d, placesFor(d, s.vehicleClass), s.car, s.vehicleClass, room,
                                                            seed_, s.person, s.startDraws, kPlaceTries);
    if (!p.done) {
        ++counts_.noRoomYet;
        return false;
    }
    s.placed = true;
    ++counts_.placed;
    return true;
}

bool DrivingHost::sendOn(driving::Driving& d, Seat& s) {
    const driving::TripPlacing p =
        driving::sendCarSomewhere(d, placesFor(d, s.vehicleClass), s.car, s.person, seed_, s.tripDraws, kPlaceTries);
    counts_.routesNotFound += p.noRoute;
    if (!p.done) {
        ++counts_.noTripYet;
        return false;
    }
    ++counts_.tripsBegun;
    return true;
}

void DrivingHost::mirror(const driving::Driving& d, Seat& s, std::vector<Agent>& agents,
                         std::vector<SimVehicle>& vehicles, bool snap) {
    const driving::Car* c = d.car(s.car);
    if (!c) return;
    Agent& a = agents[static_cast<std::size_t>(s.agent)];
    const driving::CarPose pose = driving::carPose(*d.roads(), d.vehicleClasses(), d.ruleSet(), *c);
    // The middle of the body, between its bumpers, and the way it points (its rear to its front, on the plan).
    const engine::Vec3 mid = (pose.front + pose.rear) * 0.5;
    engine::Vec2 heading(pose.body.x, pose.body.z);
    const Real len = heading.length();
    heading = len > 1e-9 ? heading * (1.0 / len) : engine::Vec2(pose.travel.x, pose.travel.z);
    a.pos = engine::Vec2(mid.x, mid.z);
    a.heading = heading;
    a.speed = c->speed;
    a.moving = c->driven;
    a.state = c->speed < driving::kStandingBelow ? Agent::State::Waiting
              : c->binding.by == driving::Limit::CarAhead ? Agent::State::Following
                                                          : Agent::State::Cruising;
    s.roadYFrom = snap ? mid.y : s.roadY;
    s.roadY = mid.y;
    if (snap) {
        a.tickFromPos = a.pos;
        a.tickFromHeading = a.heading;
    }
    if (a.vehicle >= 0 && a.vehicle < static_cast<int>(vehicles.size())) {
        vehicles[static_cast<std::size_t>(a.vehicle)].pos = a.pos;
        vehicles[static_cast<std::size_t>(a.vehicle)].heading = a.heading;
    }
}

std::string DrivingHost::begin(driving::Driving& d, std::vector<Agent>& agents, std::vector<SimVehicle>& vehicles,
                               const std::vector<int>& who, const std::vector<int>& classOfSlot, uint32_t seed) {
    if (!d.roads() || !d.hasRules() || d.vehicleClasses().empty()) return "the new model has no road, rules or vehicles";
    seed_ = seed;
    seatOfAgent_.assign(agents.size(), -1);
    seatOfCar_.assign(vehicles.size(), -1);
    seats_.clear();
    for (const int i : who) {
        if (i < 0 || i >= static_cast<int>(agents.size())) continue;
        const Agent& a = agents[static_cast<std::size_t>(i)];
        if (a.vehicle < 0 || a.vehicle >= static_cast<int>(vehicles.size())) continue;
        if (seatOfCar_[static_cast<std::size_t>(a.vehicle)] >= 0) continue;   // (one driver a car)
        Seat s;
        s.agent = i;
        s.car = driving::CarId(static_cast<uint32_t>(a.vehicle));
        s.person = a.uid;
        const int slot = vehicles[static_cast<std::size_t>(a.vehicle)].slot;
        const int cls = slot >= 0 && slot < static_cast<int>(classOfSlot.size()) ? classOfSlot[static_cast<std::size_t>(slot)] : -1;
        s.vehicleClass = cls >= 0 && cls < static_cast<int>(d.vehicleClasses().size()) ? static_cast<uint32_t>(cls) : 0u;
        seatOfCar_[static_cast<std::size_t>(a.vehicle)] = 0;   // (taken; the real seat number is set below)
        seats_.push_back(s);
    }
    std::sort(seats_.begin(), seats_.end(), [](const Seat& x, const Seat& y) { return x.car < y.car; });
    for (std::size_t k = 0; k < seats_.size(); ++k) {
        seatOfAgent_[static_cast<std::size_t>(seats_[k].agent)] = static_cast<int>(k);
        seatOfCar_[seats_[k].car.index] = static_cast<int>(k);
    }
    counts_ = DrivingCounts();
    counts_.drivers = static_cast<uint32_t>(seats_.size());
    d.attachMeasurements(measurements_.get());
    for (Seat& s : seats_) {
        if (!putDown(d, s, kStartRoom)) continue;
        sendOn(d, s);
        mirror(d, s, agents, vehicles, true);
    }
    active_ = true;
    std::size_t lanes = 0, parts = 0;
    for (std::size_t c = 0; c < placesOfClass_.size(); ++c)
        if (placesFound_[c]) {
            lanes = std::max(lanes, placesOfClass_[c].laneCount);
            parts = std::max(parts, placesOfClass_[c].parts.size());
        }
    char line[320];
    std::snprintf(line, sizeof line,
                  "%u drivers handed to the new model: %u cars put down, %llu sent on trips (places: up to %zu parts "
                  "of the road, %zu lanes, for one class)",
                  counts_.drivers, counts_.placed, static_cast<unsigned long long>(counts_.tripsBegun), parts, lanes);
    return line;
}

void DrivingHost::tick(driving::Driving& d, Real dt, std::vector<Agent>& agents, std::vector<SimVehicle>& vehicles) {
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    const double modelMs = driving::tickTimed(d, dt);
    const double watchMs = std::max(0.0, msSince(t0) - modelMs);
    const std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Trips that ended: the car stands at its destination, and is sent on again.
    std::vector<driving::CarId> arrived;
    for (const driving::Event& e : d.drainEvents())
        if (e.kind == driving::EventKind::Parked) {
            ++counts_.arrivals;
            arrived.push_back(e.car);
        }
    for (const driving::CarId car : arrived) {
        const int k = car.index < seatOfCar_.size() ? seatOfCar_[car.index] : -1;
        if (k >= 0) sendOn(d, seats_[static_cast<std::size_t>(k)]);
    }
    for (Seat& s : seats_) {
        bool snap = false;
        if (!s.placed) {
            // Not put down yet (no room was found): try again, now with moving traffic about.
            if (!putDown(d, s, kMovingRoom)) continue;
            sendOn(d, s);
            snap = true;
        } else if (d.atRoadEnd(s.car)) {
            // At the end of a road that just ends: nothing will ever move it on, so it is taken off and put down
            // where it can drive from.
            d.endTrip(d.tripOf(s.car));
            d.removeCar(s.car);
            s.placed = false;
            --counts_.placed;
            ++counts_.replaced;
            if (!putDown(d, s, kMovingRoom)) continue;
            sendOn(d, s);
            snap = true;
        } else if (!d.tripOf(s.car).valid()) {
            // Arrived earlier but no destination was found then: try again.
            sendOn(d, s);
        }
        mirror(d, s, agents, vehicles, snap);
    }
    d.drainEvents();   // (the EndedEarly and NoRoute events of the work above: counted already)

    const double hostMs = msSince(t1);
    ++cost_.ticks;
    cost_.modelMs += modelMs;
    cost_.modelWorstMs = std::max(cost_.modelWorstMs, modelMs);
    cost_.watchMs += watchMs;
    cost_.hostMs += hostMs;
    cost_.hostWorstMs = std::max(cost_.hostWorstMs, hostMs);
}

bool DrivingHost::roadHeight(int agent, Real& from, Real& to) const {
    if (!drives(agent)) return false;
    const Seat& s = seats_[static_cast<std::size_t>(seatOfAgent_[static_cast<std::size_t>(agent)])];
    if (!s.placed) return false;
    from = s.roadYFrom;
    to = s.roadY;
    return true;
}

bool DrivingHost::braking(const driving::Driving& d, int agent) const {
    if (!drives(agent)) return false;
    const driving::Car* c = d.car(seats_[static_cast<std::size_t>(seatOfAgent_[static_cast<std::size_t>(agent)])].car);
    if (!c || !c->driven) return false;
    return c->acceleration < -kBrakeLampDecel || c->speed < driving::kStandingBelow;
}

std::string DrivingHost::describe(const driving::Driving& d) const {
    if (!active_) return "the new model drives no cars here";
    uint32_t moving = 0, standing = 0;
    double speedSum = 0;
    for (const driving::Car& c : d.cars()) {
        if (!c.driven) continue;
        speedSum += c.speed;
        if (c.speed < driving::kStandingBelow) ++standing;
        else ++moving;
    }
    const uint32_t driven = moving + standing;
    const double perTick = cost_.ticks > 0 ? 1.0 / static_cast<double>(cost_.ticks) : 0.0;
    char b[640];
    std::snprintf(b, sizeof b,
                  "%u drivers, %u cars placed: %u moving, %u standing (%u placed with no trip); trips begun %llu, "
                  "arrived %llu; put down again from a road's end %llu; routes not found %llu; no destination found "
                  "yet %llu, no room yet %llu; mean speed %.2f m/s over %.1f s; tick cost %.3f ms a tick (worst %.3f), "
                  "measuring it %.3f ms, the hand-over's work %.3f ms (worst %.3f), over %llu ticks",
                  counts_.drivers, counts_.placed, moving, standing, counts_.placed - std::min(counts_.placed, driven),
                  static_cast<unsigned long long>(counts_.tripsBegun), static_cast<unsigned long long>(counts_.arrivals),
                  static_cast<unsigned long long>(counts_.replaced), static_cast<unsigned long long>(counts_.routesNotFound),
                  static_cast<unsigned long long>(counts_.noTripYet), static_cast<unsigned long long>(counts_.noRoomYet),
                  driven > 0 ? speedSum / driven : 0.0, d.clockSeconds(), cost_.modelMs * perTick, cost_.modelWorstMs,
                  cost_.watchMs * perTick, cost_.hostMs * perTick, cost_.hostWorstMs,
                  static_cast<unsigned long long>(cost_.ticks));
    return b;
}

}  // namespace citysim
