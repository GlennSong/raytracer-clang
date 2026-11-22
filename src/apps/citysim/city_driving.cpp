#include "city_driving.h"

#include "city_car.h"

#include "../driving/act.h"            // registerMoveNeeds
#include "../driving/give_way.h"       // registerJunctionFacts
#include "../driving/room.h"           // registerRoomFacts
#include "../driving/road_rules.h"
#include "../../engine/asset_root.h"   // assetPath: the bus service file
#include "../../log.h"
#include <fstream>
#include <nlohmann/json.hpp>
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
    // (The measurements' no-progress row reads two numbers too, issue #284: declared, so a rule set without them is named.)
    if (!driving::registerBuiltInFacts(facts, &problems) || !driving::registerMoveNeeds(facts, &reason) ||
        !driving::registerMeasureNeeds(facts, &problems)) {
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
    // What is in a driver's way, whatever it is (ADR-0451): its facts, and the horn, going round and another way.
    if (!driving::registerThingFacts(facts, &problems) || !driving::registerThingAnswers(vocabulary, &err)) {
        std::string all = err;
        for (const std::string& p : problems) all += (all.empty() ? "" : "; ") + p;
        return refuse("the thing step would not register: " + all);
    }
    if (!driving::loadRules(vm, file, facts, ruleSet, vocabulary, rules, &err)) return refuse(err);
    driving::RoadRules roadRules;
    if (!driving::buildRoadRules(*d.model(), ruleSet, facts, roadRules, &problems)) {
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

// ---- The buses' service (ADR-0483) -------------------------------------------------------------------------------------

const BusService& busService() {
    static const BusService service = [] {
        BusService b;
        const std::string path = engine::assetPath("assets/data/bus_service.json");
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            LOG_WARN << "[buses] " << path << " is missing: no bus runs";
            return b;
        }
        const nlohmann::json doc = nlohmann::json::parse(in, nullptr, false);
        if (doc.is_discarded() || !doc.contains("headway_minutes") || !doc.contains("most_buses_per_line") || !doc.contains("dwell_seconds") ||
            !doc.contains("traffic_factor") || !doc.contains("capacity")) {
            LOG_WARN << "[buses] " << path << ": not JSON, or a number is missing (headway_minutes, most_buses_per_line, dwell_seconds, "
                                              "traffic_factor, capacity): no bus runs";
            return b;
        }
        b.headwayMinutes = doc["headway_minutes"].get<Real>();
        b.mostBusesPerLine = doc["most_buses_per_line"].get<int>();
        b.dwellSeconds = doc["dwell_seconds"].get<Real>();
        b.trafficFactor = doc["traffic_factor"].get<Real>();
        b.capacity = doc["capacity"].get<int>();
        b.usable = b.headwayMinutes > 0 && b.mostBusesPerLine > 0 && b.dwellSeconds >= 0 && b.trafficFactor >= 1 && b.capacity > 0;
        if (!b.usable) LOG_WARN << "[buses] " << path << ": a number is out of range: no bus runs";
        return b;
    }();
    return service;
}

// Where a bus stands at stop `stop` of line `line`: the kerb lane beside it (RoadModel::busStopLane). False: no such stop,
// or no kerb lane there.
static bool busStopPoint(const driving::Driving& d, uint32_t line, uint32_t stop, driving::PathPoint& out) {
    const engine::RoadModel* m = d.model();
    if (!m || line >= m->busLines.size() || stop >= m->busLines[line].stops.size()) return false;
    const engine::KerbLanePlace k = m->busStopLane(m->busLines[line].stops[stop]);
    if (!k.valid()) return false;
    out.path = k.path;
    out.along = k.along;
    return true;
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

// ---- The host's own cars: those that drive round for ever (CityCarRoams; ADR-0453) -----------------------------------

DrivingHost::OwnCars DrivingHost::ownCars() {
    OwnCars out;
    if (!carWorld_) return out;
    carWorld_->each<CityCarRoams, CityCarBody>(
        [&](engine::Entity e, CityCarRoams&, CityCarBody& body) { out.push_back({body.id, e}); });
    std::sort(out.begin(), out.end(), [](const std::pair<uint32_t, engine::Entity>& a,
                                         const std::pair<uint32_t, engine::Entity>& b) { return a.first < b.first; });
    return out;
}

// WHERE THE MODEL HAS THE CAR, copied into its entity: the middle of the body, the way it points (its rear to its
// front, on the plan), the lane graph's height under it (blended across the tick by the drawing: `lane.fresh` for a
// car just put down, drawn there at once), its speed and its brake lamps.
static void copyPose(const driving::Driving& d, const driving::Car& c, CityCarOnLane& lane, engine::Transform& t) {
    const driving::CarPose pose = driving::carPose(*d.roads(), d.vehicleClasses(), d.ruleSet(), c);
    const engine::Vec3 mid = (pose.front + pose.rear) * 0.5;
    const engine::Vec2 body(pose.body.x, pose.body.z);
    setCarPose(t, engine::Vec2(mid.x, mid.z),
               body.length() > engine::kSamePoint ? body : engine::Vec2(pose.travel.x, pose.travel.z), mid.y);
    lane.roadYFrom = lane.fresh ? mid.y : lane.roadY;
    lane.roadY = mid.y;
    lane.speed = c.speed;
    lane.driven = c.driven;
    lane.braking = DrivingHost::braking(d, lane.car);
    lane.fresh = false;
}

bool DrivingHost::putDown(driving::Driving& d, engine::Entity e, Real room) {
    const CityCarBody& body = *carWorld_->get<CityCarBody>(e);
    CityCarRoams& roams = *carWorld_->get<CityCarRoams>(e);
    const driving::CarId car(body.id);
    if (CityBusRuns* bus = carWorld_->get<CityBusRuns>(e)) {
        // A BUS is put down at the stop it is to start from, when there is room there (a standstill gap: it pulls in at
        // its own stop, so the room asked of a car dropped anywhere into moving traffic is not asked of it); then it
        // drives to the next
        (void)room;
        driving::PathPoint at;
        const Real length = d.vehicleClasses().at(body.vehicleClass).length;
        if (!busStopPoint(d, bus->line, bus->stop, at) || !driving::roomFor(*d.roads(), d.pathLists(), at, length, kStartRoom) ||
            !d.placeCar(car, body.vehicleClass, at)) {
            ++counts_.noRoomYet;
            return false;
        }
        bus->stop = (bus->stop + 1) % static_cast<uint32_t>(d.model()->busLines[bus->line].stops.size());
        bus->standing = false;
    } else {
        const driving::TripPlacing p = driving::putCarSomewhere(d, placesFor(d, body.vehicleClass), car, body.vehicleClass,
                                                                room, seed_, driving::PersonId(body.id), roams.startDraws,
                                                                kPlaceTries);
        if (!p.done) {
            ++counts_.noRoomYet;
            return false;
        }
    }
    ++counts_.placed;
    CityCarOnLane lane;
    lane.car = car;
    engine::Transform& t = *carWorld_->get<engine::Transform>(e);
    copyPose(d, *d.car(car), carWorld_->add<CityCarOnLane>(e, lane), t);
    // (drawn where it was put down at once: it does not slide there from where it was)
    if (engine::PrevTransform* prev = carWorld_->get<engine::PrevTransform>(e)) prev->value = t;
    return true;
}

bool DrivingHost::takenByPlayer(driving::Driving& d, engine::Entity e) {
    if (!carWorld_ || !carWorld_->has<CityCarRoams>(e)) return false;
    const driving::CarId car(carWorld_->get<CityCarBody>(e)->id);
    if (d.car(car)) {
        const driving::TripId trip = d.tripOf(car);
        if (trip.valid()) d.endTrip(trip);
        if (d.carried(car)) d.carry(car, false);
        d.removeCar(car);
        --counts_.placed;
    }
    carWorld_->remove<CityCarOnLane>(e);
    carWorld_->remove<CityCarRoams>(e);
    if (CityCarLink* link = carWorld_->get<CityCarLink>(e)) link->away = true;   // the player's from now (ADR-0455)
    --counts_.drivers;
    return true;
}

bool DrivingHost::sendOn(driving::Driving& d, engine::Entity e) {
    const CityCarBody& body = *carWorld_->get<CityCarBody>(e);
    CityCarRoams& roams = *carWorld_->get<CityCarRoams>(e);
    if (const CityBusRuns* bus = carWorld_->get<CityBusRuns>(e)) {
        // A BUS drives to the stop it is bound for
        driving::PathPoint to;
        const driving::TripId trip = busStopPoint(d, bus->line, bus->stop, to)
                                         ? d.beginTrip(driving::CarId(body.id), driving::PersonId(body.id), to)
                                         : driving::TripId();
        if (!trip.valid()) {
            ++counts_.routesNotFound;
            return false;
        }
        CityBusRuns& runs = *carWorld_->get<CityBusRuns>(e);
        runs.driving = 0;
        const driving::Route* r = d.route(trip);
        runs.freeFlow = r ? r->seconds : 0;
        ++counts_.tripsBegun;
        if (CityCarOnLane* lane = carWorld_->get<CityCarOnLane>(e)) lane->driven = true;
        return true;
    }
    const driving::TripPlacing p = driving::sendCarSomewhere(d, placesFor(d, body.vehicleClass), driving::CarId(body.id),
                                                             driving::PersonId(body.id), seed_, roams.tripDraws,
                                                             kPlaceTries);
    counts_.routesNotFound += p.noRoute;
    if (!p.done) {
        ++counts_.noTripYet;
        return false;
    }
    ++counts_.tripsBegun;
    if (CityCarOnLane* lane = carWorld_->get<CityCarOnLane>(e)) lane->driven = true;
    return true;
}

void DrivingHost::reset() {
    // (its own cars were made for the last build's city: they go with it)
    if (carWorld_) {
        std::vector<engine::Entity> mine;
        for (const std::pair<uint32_t, engine::Entity>& c : ownCars()) mine.push_back(c.second);
        for (const engine::Entity e : mine) carWorld_->destroy(e);
    }
    active_ = false;
    seed_ = 0;
    carWorld_ = nullptr;
    guestEvents_.clear();
    placesOfClass_.clear();
    placesFound_.clear();
    measurements_ = std::make_unique<driving::Measurements>();
    counts_ = DrivingCounts();
    cost_ = Cost();
}

// ---- Every car entity the model has (ADR-0444, ADR-0453) ---------------------------------------------------------------

void DrivingHost::mirrorCarEntities(const driving::Driving& d) {
    carWorld_->each<CityCarOnLane, engine::Transform>([&](engine::Entity, CityCarOnLane& lane, engine::Transform& t) {
        if (const driving::Car* c = d.car(lane.car)) copyPose(d, *c, lane, t);
    });
}

std::vector<driving::Event> DrivingHost::takeGuestEvents() {
    std::vector<driving::Event> out;
    out.swap(guestEvents_);
    return out;
}

bool DrivingHost::keptForGuest(const driving::Event& e, const OwnCars& own) {
    const OwnCars::const_iterator it =
        std::lower_bound(own.begin(), own.end(), e.car.index,
                         [](const std::pair<uint32_t, engine::Entity>& c, uint32_t id) { return c.first < id; });
    if (it != own.end() && it->first == e.car.index) return false;
    guestEvents_.push_back(e);
    return true;
}

std::string DrivingHost::begin(driving::Driving& d, engine::World& world, uint32_t seed) {
    if (!d.roads() || !d.hasRules() || d.vehicleClasses().empty()) return "the new model has no road, rules or vehicles";
    seed_ = seed;
    carWorld_ = &world;
    const OwnCars own = ownCars();
    counts_ = DrivingCounts();
    counts_.drivers = static_cast<uint32_t>(own.size());
    for (const std::pair<uint32_t, engine::Entity>& c : own) counts_.buses += world.has<CityBusRuns>(c.second) ? 1 : 0;
    // THE COUNTING LINES (issue #284): the line of every junction whose lights run a programme -- the end of each lane
    // that leads into its connectors -- so the measurements' queue rows see queues empty over them (before, the host
    // registered none and those rows had no data). Only lit lines: the rows' targets are a queue leaving on a green
    // (saturation flow), not cars taking turns at a stop sign. In path order, before the measurements begin.
    if (measurements_->countingLines().empty()) {
        const engine::LaneGraph& roads = *d.roads();
        for (uint32_t p = 0; p < roads.paths.size(); ++p) {
            const engine::Path& lane = roads.paths[p];
            if (lane.kind != engine::PathKind::Lane || lane.next.count == 0) continue;
            const engine::PathId first = roads.nextPaths[lane.next.first];
            if (!first.valid() || first.index >= roads.paths.size() ||
                roads.paths[first.index].kind != engine::PathKind::Connector)
                continue;
            const engine::JunctionId j = roads.paths[first.index].junction;
            if (!j.valid() || j.index >= d.rightOfWay().junctions.size() || !d.rightOfWay().junctions[j.index].running())
                continue;
            driving::PathPoint line;
            line.path = engine::PathId(p);
            line.along = lane.length;
            measurements_->addCountingLine(line);
        }
    }
    d.attachMeasurements(measurements_.get());
    // The bodies-touching row measures each car at the size it is DRAWN (its fleet slot's body), which may differ from
    // its driving class's (the class takes the first slot of its name).
    for (const std::pair<uint32_t, engine::Entity>& c : own) {
        const CityCarBody& body = *world.get<CityCarBody>(c.second);
        measurements_->setDrawnSize(driving::CarId(body.id), body.length, body.width);
    }
    for (const std::pair<uint32_t, engine::Entity>& c : own)
        if (putDown(d, c.second, kStartRoom)) sendOn(d, c.second);
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

void DrivingHost::tick(driving::Driving& d, Real dt) {
    // (where each car entity is as the tick begins, which the drawing blends from)
    if (carWorld_) carTickBegins(*carWorld_);
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    const double modelMs = driving::tickTimed(d, dt);
    const double watchMs = std::max(0.0, msSince(t0) - modelMs);
    const std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    const OwnCars own = ownCars();
    const auto entityOf = [&](driving::CarId car) {
        const OwnCars::const_iterator it =
            std::lower_bound(own.begin(), own.end(), car.index,
                             [](const std::pair<uint32_t, engine::Entity>& c, uint32_t id) { return c.first < id; });
        return it != own.end() && it->first == car.index ? it->second : engine::Entity{};
    };

    // Trips that ended: the car stands at its destination, and is sent on again.
    std::vector<driving::CarId> arrived;
    for (const driving::Event& e : d.drainEvents()) {
        if (keptForGuest(e, own)) continue;   // (a household's car: its own driver's to act on)
        if (e.kind == driving::EventKind::Parked) {
            ++counts_.arrivals;
            arrived.push_back(e.car);
        }
    }
    for (const driving::CarId car : arrived) {
        const engine::Entity e = entityOf(car);
        if (CityBusRuns* bus = carWorld_->get<CityBusRuns>(e)) {
            // A BUS AT ITS STOP stands there with its doors open, then drives on to the next
            bus->standing = true;
            bus->dwellLeft = busService().dwellSeconds;
            ++bus->calls;
            ++counts_.busCalls;
            if (bus->freeFlow > 0) {
                counts_.busDriven += bus->driving;
                counts_.busFreeFlow += bus->freeFlow;
            }
            continue;
        }
        sendOn(d, e);
    }
    for (const std::pair<uint32_t, engine::Entity>& c : own) {
        const driving::CarId car(c.first);
        const engine::Entity e = c.second;
        if (CityBusRuns* bus = carWorld_->get<CityBusRuns>(e); bus && !bus->standing) bus->driving += dt;
        if (CityBusRuns* bus = carWorld_->get<CityBusRuns>(e); bus && bus->standing && d.car(car)) {
            bus->dwellLeft -= dt;
            if (bus->dwellLeft > 0) continue;
            bus->standing = false;
            bus->stop = (bus->stop + 1) % static_cast<uint32_t>(d.model()->busLines[bus->line].stops.size());
            sendOn(d, e);
            continue;
        }
        if (!d.car(car)) {
            // Not put down yet (no room was found): try again, now with moving traffic about.
            if (putDown(d, e, kMovingRoom)) sendOn(d, e);
        } else if (!d.carried(car) && d.atRoadEnd(car)) {
            // At the end of a road that just ends: nothing will ever move it on, so it is taken off and put down
            // where it can drive from. (A car physics carries is the near tier's until it is a lane car again.)
            d.endTrip(d.tripOf(car));
            d.removeCar(car);
            carWorld_->remove<CityCarOnLane>(e);
            --counts_.placed;
            ++counts_.replaced;
            if (putDown(d, e, kMovingRoom)) sendOn(d, e);
        } else if (!d.tripOf(car).valid()) {
            // Arrived earlier but no destination was found then: try again.
            sendOn(d, e);
        }
    }
    // Every car entity on the road: where it is, copied in.
    if (carWorld_) mirrorCarEntities(d);
    // (the EndedEarly and NoRoute events of the work above: counted already; a household car's are kept for its driver)
    for (const driving::Event& e : d.drainEvents()) keptForGuest(e, own);

    const double hostMs = msSince(t1);
    ++cost_.ticks;
    cost_.modelMs += modelMs;
    cost_.modelWorstMs = std::max(cost_.modelWorstMs, modelMs);
    cost_.watchMs += watchMs;
    cost_.hostMs += hostMs;
    cost_.hostWorstMs = std::max(cost_.hostWorstMs, hostMs);
}

bool DrivingHost::braking(const driving::Driving& d, driving::CarId car) {
    const driving::Car* c = d.car(car);
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
