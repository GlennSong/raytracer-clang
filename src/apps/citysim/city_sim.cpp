#include "city_sim.h"
#include "city_car.h"
#include "../../log.h"     // the new driving's cars are entities (startDrivingV2)
#include "../../hash.h"   // lowbias32

#include "../../profile.h"
#include "../../engine/clock.h"   // kDefaultFixedStep
#include "../../engine/bundle/bundle.h"   // the population cache key
#include "../../engine/bundle/cache_use.h"
#include "../../engine/procgen/city/roads/lanes/walk_lanes.h"   // kWalkFromKerb: a corner walk's line from its kerb
#include "../../job_system.h"   // assignPlaces decides the population in parallel

#include <algorithm>
#include <set>
#include <atomic>
#include <iterator>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <cmath>
#include <cstdio>    // RT_CRASH_DEBUG contact telemetry
#include <cstdlib>
#include <limits>
#include <unordered_map>

namespace citysim {

using engine::Vec2;

namespace {
// (kWalkSpeed, a walker's speed, is in city_sim.h: the agent brain's walk skill sets a pace against it, ADR-0441)
// "Wakes on nothing": a rest with no dwell and no commute. Large enough to mean
// never in any real session, finite so the arithmetic stays ordinary.
constexpr Real kNeverWakes = CitySim::kNoRestEvent;
constexpr Real kPedAccel = 1.0;
constexpr Real kNoLeaderMinGap = 5.0;   // a walker's follow gap with nobody ahead (computeGaps; unread then)
constexpr Real kPedMinGap = 0.8, kPedSlowZone = 2.5;

// The fleet of body slots (ADR-0061 Phase 4). A sedan is the player's body
// exactly (4.2 long) and its follow gap works out to the historical 5.0 m; larger
// bodies keep proportionally more room. Lengths are all >= the sedan, so adding
// them never lets cars pack tighter than before. Order is mirrored by the render
// fleet (city_render.cpp kCarVariants), slot for slot.
const VehicleBody kFleet[] = {
    {4.2, 1.80, 1.30, VehicleType::Sedan},
    {4.2, 1.80, 1.30, VehicleType::Sedan},
    {4.2, 1.80, 1.30, VehicleType::Sedan},
    {4.2, 1.82, 1.45, VehicleType::Hatchback},
    {4.2, 1.82, 1.45, VehicleType::Hatchback},
    {4.2, 1.82, 1.45, VehicleType::Hatchback},
    {4.6, 1.95, 1.70, VehicleType::SUV},
    {4.6, 1.95, 1.70, VehicleType::SUV},
    {4.6, 1.95, 1.70, VehicleType::SUV},
    {5.2, 1.95, 1.60, VehicleType::Pickup},
    {5.4, 2.00, 2.10, VehicleType::Van},
    {6.6, 2.40, 2.80, VehicleType::BoxTruck},
};
constexpr int kFleetSize = static_cast<int>(sizeof(kFleet) / sizeof(kFleet[0]));
constexpr Real kPedDecel = 3.0;
constexpr Real kPedVisionRange = 4.5;      // how far ahead a walker perceives (m)
constexpr Real kPedVisionHalfAngle = 1.2;  // ~69 deg to each side (wide peripheral)
constexpr Real kPedMaxLateral = 1.6;       // furthest a walker leans off its path (m)
// (...toward the road, only as far as the walk's own kerb allows: Agent::kerbRoom, #314. kPedRoadsideRoom, a 0.7 m
// limit for every pavement from an assumed 1 m verge, went with it.)
constexpr Real kPedLateralRate = 1.6;      // how fast that lean changes (m/s) — smooth, not a pop
constexpr Real kPedBodyMin = 0.5;          // hard floor: bodies never closer than this
constexpr Real kPoleClearance = 0.7;       // a walker keeps its centre this far from a pole
constexpr Real kPlayerClearance = 1.1;     // ...and this far from the PLAYER (a wide berth,
                                           // so a near miss is a step-around, not a brush)
constexpr Real kPedSideBySide = 0.4;   // walkers this close along the way walk abreast: neither leads
// The cognition loop (ADR-0063): agents act on MEMORY, not on the momentary
// snapshot. A track must hold at least this much confidence to be acted on —
// with the default 4 s memory horizon that means a body is reacted to for ~3 s
// after it was last actually seen (extrapolated along where it was heading).
constexpr Real kMemoryActConfidence = 0.25;
constexpr Real kPedAnticipation = 0.4;  // walkers dodge where a neighbour WILL be (s ahead)

// Three-tier traffic (P4). The far (V) tier traverses its real route at the
// link class speed shaped by FIXED average junction costs — it runs no sensing
// and reads no live signal state, so a junction is a modelled dwell, not a
// queue. kAgentGridCell sizes the P4.1 spatial grid; kGridPad widens its
// bounds past the graph for parked verges, doorsteps and corner blends.
constexpr Real kVJunctionDelay = 1.5;   // avg dwell per junction crossed (no old signal clock: ADR-0456)
constexpr Real kAgentGridCell = 64.0;
constexpr Real kGridPad = 250.0;

// Speed a follower may travel given the centre-to-centre gap to its leader.
Real followCap(Real freeSpeed, Real gap, Real minGap, Real slowZone) {
    if (gap <= minGap) return 0.0;
    if (gap >= slowZone) return freeSpeed;
    return freeSpeed * (gap - minGap) / (slowZone - minGap);
}

// Is in-world hour `clock` inside the window [from, to)? A window whose `from`
// is LATER than its `to` wraps midnight — a night shift leaving at 21:30 and
// coming home at 05:30 is at work from 21:30 THROUGH to 05:30, not never.
//
// The naive `clock >= from && clock < to` silently makes every wrapping window
// empty, so a night-shift agent would never depart for work at all. Mirrors
// Place::openAt (places.cpp:30-35), which already got this right.
bool inWindow(Real clock, Real from, Real to) {
    if (from == to) return true;                  // degenerate → always inside
    if (from < to) return clock >= from && clock < to;
    return clock >= from || clock < to;           // wraps midnight
}

// Personality traits read from FIXED bits of an agent's `brain` (ADR-0062): no
// rng draw, so seeded build streams are unchanged. Each helper owns its bits.

// Personal pace in [0.85, 1.15] of the nominal speed — owns brain bits 9..23.
Real paceFactor(uint32_t brain) {
    return 0.85 + 0.30 * (((brain >> 9) & 0x7FFF) / Real(0x7FFF));
}

// Think-clock stagger as a 0..1 fraction of the think period — owns bits 17..24.
Real thinkStagger(uint32_t brain) {
    return ((brain >> 17) & 0xFF) / Real(0xFF);
}

// Park/idle setback slot (0..7) spacing same-node agents along the verge — owns
// bits 3..5.
Real parkSetbackSlot(uint32_t brain) {
    return static_cast<Real>((brain >> 3) & 7);
}
}  // namespace

// (ADR-0060; moved here from traffic_rules.h with ADR-0463: a walker eases to a stop at a crosswalk's kerb end with it)
double approachStop(double dist, double decel, double maxSpeed) {
    if (dist <= 0.0) return 0.0;
    double v = std::sqrt(2.0 * decel * dist);
    return std::min(v, maxSpeed);
}

int vehicleFleetSize() { return kFleetSize; }

const VehicleBody& vehicleFleetBody(int slot) {
    int s = ((slot % kFleetSize) + kFleetSize) % kFleetSize;   // wrap, handle negatives
    return kFleet[s];
}

driving::VehicleClasses builtInDrivingClasses() {
    driving::VehicleClasses out;
    for (int s = 0; s < kFleetSize; ++s) {
        const VehicleBody& b = kFleet[s];
        out.add(driving::deriveVehicleClass("fleet_slot_" + std::to_string(s), b.length, b.width,
                                            b.length * driving::kUnknownWheelbaseShareOfLength,
                                            driving::kUnknownMaxSteerDegrees));
    }
    return out;
}

void CitySim::setRoads(std::shared_ptr<const engine::RoadModel> roads, std::string report) {
    roads_ = std::move(roads);
    roadsReport_ = std::move(report);
    // THE WALKER on these walks (W3): its open walks and waits; and, for each walk beside a road -- a street's sidewalk
    // or footway stretch, or a corner walk round a junction's kerb (by the model's ids: Walk::street, Walk::modelWalk) --
    // which side of it the road is and how far its line is from that kerb (a stretch's line is the sidewalk's middle,
    // half its width; a corner's is kWalkFromKerb in, walk_lanes.h)
    walker_ = engine::roads::RouteWalker{};
    walkParts_ = engine::ReachableParts{};
    walkRoadSide_.clear();
    walkKerbOff_.clear();
    if (roads_ && roads_->built) {
        const engine::LaneGraph& G = roads_->laneLevel;
        walker_ = engine::roads::bindWalker(*roads_, "walker", kWalkSpeed);
        walkParts_ = engine::reachableWalks(G, walker_.open);
        walkRoadSide_.assign(G.walks.size(), 0);
        walkKerbOff_.assign(G.walks.size(), 0.0f);
        for (uint32_t w = 0; w < G.walks.size(); ++w) {
            const engine::Walk& k = G.walks[w];
            const bool stretch = k.street != engine::kNoIndex && k.street < roads_->streets.size();
            const bool corner = !stretch && k.modelWalk != engine::kNoIndex;
            if ((!stretch && !corner) || k.length <= 0) continue;
            const Real mid = k.length * 0.5;
            const Vec2 p = G.pointAt(engine::WalkId(w), mid);
            const Vec2 d = G.pointAt(engine::WalkId(w), std::min(k.length, mid + 0.5)) - G.pointAt(engine::WalkId(w), std::max(Real(0), mid - 0.5));
            const engine::RoadFrontage f = stretch ? roads_->frontageOn(engine::ModelStreetId(k.street), (k.stationFrom + k.stationTo) * 0.5, 1)
                                                   : roads_->frontageAt(p);
            if (!f.valid()) continue;
            const Vec2 toRoad = f.on - p;
            const Real c = d.x * toRoad.y - d.y * toRoad.x;   // > 0: the road on its left, walked forward
            walkRoadSide_[w] = c > 0 ? int8_t(-1) : int8_t(1);
            walkKerbOff_[w] = static_cast<float>(stretch ? k.width * 0.5 : engine::roads::lanes::kWalkFromKerb);
        }
    }
    if (!drivingV2_) return;
    driving::VehicleClasses classes = drivingV2_->vehicleClasses();
    drivingV2_.emplace(roads_.get());
    drivingV2_->setVehicleClasses(std::move(classes));
}

// ---- THE WALK ROUTE (#306 piece 6, W3): where a walker on its route is, and how a route begins --------------------

bool CitySim::walkJoin(Vec2 p, engine::WalkPoint& out, bool* cutOff) const {
    if (!roads_ || !roads_->built || walker_.open.empty()) return false;
    const engine::WalkJoin j = roads_->walkJoinNear(p, walker_.open, &walkParts_);   // the one fallback, by position
    if (!j.valid()) return false;
    out = j.at;
    if (cutOff) *cutOff = j.cutOff;
    return true;
}

engine::roads::WalkRoute CitySim::walkRouteBetween(Vec2 from, Vec2 to, const engine::StatedFrontage* fromAt,
                                                  const engine::StatedFrontage* toAt) const {
    engine::roads::WalkRoute r;
    if (!roads_ || !roads_->built || walker_.open.empty()) return r;
    // each end: what its place states (a door's frontage: the one answer, walkJoinOn), else its point (walkJoinNear)
    const auto join = [&](Vec2 p, const engine::StatedFrontage* stated, engine::WalkPoint& out) {
        if (stated && (stated->onStreet() || stated->onWalkway())) {
            const engine::WalkJoin j = roads_->walkJoinOn(*stated, walker_.open);
            if (j.valid()) {
                out = j.at;
                return true;
            }
        }
        return walkJoin(p, out);
    };
    engine::WalkPoint a, b;
    if (!join(from, fromAt, a) || !join(to, toAt, b)) return r;
    return engine::roads::findWalkRoute(*roads_, walker_, a, b);
}

const engine::StatedFrontage* CitySim::placeFrontageAt(const Agent& a, int node) const {
    PlaceId p = kNoPlace;
    if (node == a.home) p = a.homePlace;
    else if (node == a.work) p = a.workPlace;
    else if (node == a.shop) p = a.shopPlace;
    if (p == kNoPlace || p >= placeFrontage_.size()) return nullptr;
    const engine::StatedFrontage& f = placeFrontage_[p];
    return f.onStreet() || f.onWalkway() ? &f : nullptr;
}

Real CitySim::legLength(const Agent& a, int leg) const {
    if (!roads_ || leg < 0 || leg >= static_cast<int>(a.route.steps.size())) return 0;
    const engine::roads::WalkStep& s = a.route.steps[static_cast<std::size_t>(leg)];
    Real L = std::fabs(s.to - s.from);
    if (leg + 1 < static_cast<int>(a.route.steps.size())) {
        const engine::roads::WalkStep& n = a.route.steps[static_cast<std::size_t>(leg + 1)];
        const engine::LaneGraph& G = roads_->laneLevel;
        L += (G.pointAt(n.walk, n.from) - G.pointAt(s.walk, s.to)).length();
    }
    return L;
}

Real CitySim::routeLeft(const Agent& a) const {
    if (!onRoute(a)) return 0;
    Real left = -a.distOnLeg;
    for (int k = a.leg; k < static_cast<int>(a.route.steps.size()); ++k) left += legLength(a, k);
    return std::max(Real(0), left);
}

Vec2 CitySim::routePoint(const Agent& a, int leg, Real d, Vec2* dir) const {
    const engine::LaneGraph& G = roads_->laneLevel;
    const engine::roads::WalkStep& s = a.route.steps[static_cast<std::size_t>(leg)];
    const Real along = std::fabs(s.to - s.from);
    const Real sgn = s.forward ? 1.0 : -1.0;
    if (d <= along || leg + 1 >= static_cast<int>(a.route.steps.size())) {
        const Real at = s.from + sgn * std::clamp(d, Real(0), along);
        const Vec2 p = G.pointAt(s.walk, at);
        if (dir) {
            const Real w = G.walks[s.walk.index].length;
            const Vec2 q0 = G.pointAt(s.walk, std::clamp(at - 0.75, Real(0), w)), q1 = G.pointAt(s.walk, std::clamp(at + 0.75, Real(0), w));
            const Vec2 t = (q1 - q0) * sgn;
            const Real l = t.length();
            *dir = l > 1e-6 ? t * (1.0 / l) : (dir->length() > 0.5 ? *dir : Vec2(1, 0));
        }
        return p;
    }
    // across the gap to the next step's start
    const engine::roads::WalkStep& n = a.route.steps[static_cast<std::size_t>(leg + 1)];
    const Vec2 e = G.pointAt(s.walk, s.to), b = G.pointAt(n.walk, n.from);
    const Real gap = (b - e).length();
    const Real u = gap > 1e-6 ? std::clamp((d - along) / gap, Real(0), Real(1)) : 1.0;
    if (dir && gap > 1e-6) *dir = (b - e) * (1.0 / gap);
    return e + (b - e) * u;
}

void CitySim::beginRoute(Agent& a, engine::roads::WalkRoute route) {
    a.route = std::move(route);
    a.leg = 0;
    a.distOnLeg = 0;
    a.speed = 0;
    a.crosswalkAt = -1;
    a.onCrosswalk = false;
    a.aboutToCross = false;
    a.crossWaited = 0;
    ++a.trips;   // a new route
    a.moving = true;
    a.indoors = false;   // out of the door
    a.restDwell = 0;
    refreshPose(a);
    a.tickFromPos = a.pos;          // a placement, not a motion (see tickFromPos)
    a.tickFromHeading = a.heading;
}

bool CitySim::sendAgentTo(int agentIndex, Vec2 dest) {
    if (agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size()) || !roads_) return false;
    Agent& a = agents_[static_cast<std::size_t>(agentIndex)];
    engine::WalkPoint from, to;
    if (onRoute(a)) {
        // UNDER WAY: from where it stands on its route (on a step's walk; in a gap, the next step's start), so the
        // errand changes the plan and never the pose (a drawn walker's ghost is leashed to its body, ADR-0062)
        const engine::roads::WalkStep& s = a.route.steps[static_cast<std::size_t>(a.leg)];
        const Real along = std::fabs(s.to - s.from);
        if (a.distOnLeg <= along || a.leg + 1 >= static_cast<int>(a.route.steps.size())) {
            from.walk = s.walk;
            from.along = s.from + (s.forward ? 1.0 : -1.0) * std::min(a.distOnLeg, along);
        } else {
            const engine::roads::WalkStep& n = a.route.steps[static_cast<std::size_t>(a.leg + 1)];
            from.walk = n.walk;
            from.along = n.from;
        }
    }
    // RT_WALK_REFUSED: each errand refused, and why (where it is from or to joins no walk; the router's reason)
    static const bool sayWhy = std::getenv("RT_WALK_REFUSED") != nullptr;
    const auto refuse = [&](const std::string& why) {
        if (sayWhy)
            std::fprintf(stderr, "[walk refused] agent %d from %.1f %.1f to %.1f %.1f: %s\n", agentIndex, a.pos.x, a.pos.y, dest.x,
                         dest.y, why.c_str());
        return false;
    };
    if (!onRoute(a) && !walkJoin(a.pos, from)) return refuse("where it stands joins no walk");
    if (!walkJoin(dest, to)) return refuse("where it is going joins no walk");
    engine::roads::WalkRoute r = engine::roads::findWalkRoute(*roads_, walker_, from, to);
    // NOTHING has been changed yet, so a refusal leaves the agent its day.
    if (!r.found || r.steps.empty()) {
        const auto streetOf = [&](engine::WalkId w) {
            const engine::Walk& k = roads_->laneLevel.walks[w.index];
            return k.street < roads_->streets.size() ? roads_->streets[k.street].id : roads_->walkKind(w);
        };
        return refuse("no route: " + r.whyWords + " (walk " + std::to_string(from.walk.index) + " on " + streetOf(from.walk) +
                      " to walk " + std::to_string(to.walk.index) + " on " + streetOf(to.walk) + ")");
    }
    a.tripGoal = walkPoints_.nearestNode(dest);
    const bool underWay = a.moving;
    const Vec2 was = a.pos;
    beginRoute(a, std::move(r));
    if (underWay) a.tickFromPos = was;   // (a re-plan, not a placement: drawn on from where it was)
    // A DIRECTOR'S ERRAND OVERRIDES THE DAY: the reactive state and the day's label move with it (left Resting or AtHome,
    // the goal layer parked it again on its own schedule).
    a.state = Agent::State::Walking;
    a.activity = Activity::Outing;
    return true;
}

bool CitySim::glideAlong(Agent& a, Real metres, Real stopShort) const {
    const int n = static_cast<int>(a.route.steps.size());
    if (!a.route.found || a.leg >= n) return true;
    if (stopShort >= 0) metres = std::min(metres, std::max(Real(0), routeLeft(a) - stopShort));
    while (metres > 0 && a.leg < n) {
        const Real room = legLength(a, a.leg) - a.distOnLeg;
        if (metres < room || (stopShort >= 0 && a.leg + 1 >= n)) {   // (held short: never past the last step's end)
            a.distOnLeg = std::min(a.distOnLeg + metres, legLength(a, a.leg));
            metres = 0;
            break;
        }
        metres -= room;
        if (a.leg + 1 >= n) { a.distOnLeg = legLength(a, a.leg); ++a.leg; break; }
        ++a.leg;
        a.distOnLeg = 0;
    }
    return a.leg >= n;
}

uint32_t CitySim::nextCarNumber(engine::World& world) const {
    uint32_t next = 0;
    world.each<CityCarBody>([&](engine::Entity, CityCarBody& b) { next = std::max(next, b.id + 1); });
    return next;
}

bool CitySim::startDrivingV2(engine::World& world, const std::string& rulesFile, int cars,
                             const std::vector<int>& classOfSlot, std::string* report) {
    std::string why;
    if (!drivingV2_ || !drivingV2_->roads()) why = "the new model has no road (see `lanegraph?`)";
    else if (drivingHost_.active()) why = "the new model already runs on this level";
    else if (!loadDrivingRules(rulesFile, *drivingV2_, &why)) why = "rule set refused: " + why;
    if (!why.empty()) {
        if (report) *report = why;
        return false;
    }
    // LANE CHANGES (sitting 5): routes plan them where the rule set has the lane change step to carry them out. On by
    // default since letting in, dropping back, the zipper at merges and parking on the same curve (ADR-0489 had them
    // opt-in); RT_LANE_CHANGES=0 turns them off: every route then keeps to lanes the car's own lane leads into.
    driving::DrivingSettings settings = drivingV2_->settings();
    const char* laneChanges = std::getenv("RT_LANE_CHANGES");
    settings.laneChanges = drivingV2_->laneChangeStep() && !(laneChanges && std::string(laneChanges) == "0");
    drivingV2_->setSettings(settings);

    // THE CARS THAT DRIVE ROUND FOR EVER (city_car.h CityCarRoams): each one entity, with nobody in it; where it is
    // comes from the driving once the hand-over has put it down.
    const uint32_t classes = static_cast<uint32_t>(drivingV2_->vehicleClasses().size());
    for (int k = 0; k < cars; ++k) {
        CityCarBody body;
        body.id = nextCarNumber(world);
        body.slot = ambientSlotFor(static_cast<int>(body.id));
        const VehicleBody& fleet = fleetBody(body.slot);
        body.type = fleet.type;
        body.length = fleet.length;
        body.width = fleet.width;
        body.height = fleet.height;
        const int cls = body.slot >= 0 && body.slot < static_cast<int>(classOfSlot.size())
                            ? classOfSlot[static_cast<std::size_t>(body.slot)]
                            : -1;
        body.vehicleClass = cls >= 0 && static_cast<uint32_t>(cls) < classes ? static_cast<uint32_t>(cls) : 0u;
        const engine::Entity e = world.create();
        world.add<engine::Transform>(e);
        world.add<engine::PrevTransform>(e);
        world.add<CityCarBody>(e, body);
        world.add<CityCarLink>(e, CityCarLink{});
        world.add<CityCarRoams>(e);
    }
    // THE BUSES ON THE ROAD FILE'S LINES (ADR-0483): each line as many buses as its round trip (the bus's own routes
    // from stop to stop, at the free-flow speed times the measured traffic factor, and every stop's wait) needs for the headway the service file asks
    // (city_driving.h busService), at most its cap; spread evenly round the line, each starting at its own stop. Drawn as
    // the fleet's bus (the slot whose class is "bus"). The hand-over runs them (DrivingHost).
    const engine::RoadModel* model = drivingV2_->model();
    const int busClass = drivingV2_->vehicleClasses().indexOf("bus");
    int busSlot = -1;
    for (std::size_t s = 0; s < classOfSlot.size() && busSlot < 0; ++s)
        if (classOfSlot[s] == busClass) busSlot = static_cast<int>(s);
    const BusService& service = busService();
    uint32_t buses = 0;
    if (model && !model->busLines.empty() && busClass >= 0 && busSlot >= 0 && service.usable) {
        for (uint32_t li = 0; li < model->busLines.size(); ++li) {
            const engine::ModelBusLine& line = model->busLines[li];
            const std::size_t n = line.stops.size();
            Real loop = 0;
            bool drivable = true;
            for (std::size_t k = 0; k < n && drivable; ++k) {
                const engine::KerbLanePlace a = model->busStopLane(line.stops[k]), b = model->busStopLane(line.stops[(k + 1) % n]);
                driving::PathPoint pa, pb;
                pa.path = a.path;
                pa.along = a.along;
                pb.path = b.path;
                pb.along = b.along;
                const engine::roads::Route r = a.valid() && b.valid() ? drivingV2_->routeBetween(static_cast<uint32_t>(busClass), pa, pb)
                                                                      : engine::roads::Route();
                drivable = r.found;
                loop += r.seconds * service.trafficFactor + service.dwellSeconds;
            }
            if (!drivable) continue;   // (said at load: Driving::busLineProblems)
            const int count = std::clamp(static_cast<int>(std::ceil(loop / (service.headwayMinutes * 60.0))), 1, service.mostBusesPerLine);
            for (int k = 0; k < count; ++k) {
                CityCarBody body;
                body.id = nextCarNumber(world);
                body.slot = busSlot;
                const VehicleBody& fleet = fleetBody(busSlot);
                body.type = fleet.type;
                body.length = fleet.length;
                body.width = fleet.width;
                body.height = fleet.height;
                body.vehicleClass = static_cast<uint32_t>(busClass);
                const engine::Entity e = world.create();
                world.add<engine::Transform>(e);
                world.add<engine::PrevTransform>(e);
                world.add<CityCarBody>(e, body);
                world.add<CityCarLink>(e, CityCarLink{});
                world.add<CityCarRoams>(e);
                CityBusRuns runs;
                runs.line = li;
                runs.stop = static_cast<uint32_t>((static_cast<std::size_t>(k) * n) / static_cast<std::size_t>(count));
                world.add<CityBusRuns>(e, runs);
                ++buses;
            }
        }
    } else if (model && !model->busLines.empty()) {
        LOG_WARN << "[buses] " << model->busLines.size() << " lines stated and no bus runs: "
                 << (busClass < 0 ? "the fleet has no \"bus\" class" : busSlot < 0 ? "no fleet slot is a bus" : "the service file is not usable");
    }
    if (buses) LOG_INFO << "[buses] " << buses << " buses on " << model->busLines.size() << " lines";
    // THE PEOPLE ON THEM (ADR-0484): the timetable they are priced by, and who rides
    buses_.reset();
    brainRideEvents_.clear();
    if (buses && model) buses_.build(*model, *drivingV2_, walker_.open, world);
    rebuildTransit();
    const std::string line = drivingHost_.begin(*drivingV2_, world, seed_);
    if (report) *report = line;
    return true;
}

Real CitySim::pairMinGap(int, int) const { return kPedMinGap; }

engine::roads::WalkRoute CitySim::pointRoute(int from, int to) const {
    engine::roads::WalkRoute r;
    if (!roads_ || !roads_->built || walker_.open.empty() || from < 0 || to < 0 || from >= walkPoints_.nodeCount() ||
        to >= walkPoints_.nodeCount())
        return r;
    return engine::roads::findWalkRoute(*roads_, walker_, walkPoints_.at[static_cast<std::size_t>(from)],
                                        walkPoints_.at[static_cast<std::size_t>(to)]);
}

// WALKERS IN FILES ACROSS THE PAVEMENT (Glenn: "sitting on top of one another like it's some spawn point"). Every
// walker kept the one line 1 m in from the kerb and followed whoever was ahead on it, so a tower door's worth of
// strollers (homes by floor area: hundreds to a door) went down a 5 m pavement in single file, 0.4 m apart, crawling at
// a third of a walking pace -- and stood in a heap at the door. Up to three files, 0.9 m apart, as wide as the
// pavement allows (junctionPad_, the street look's), each its own following chain. RT_ONE_FILE: as before.
int CitySim::walkerFile(const Agent& a) const {
    static const bool oneFile = std::getenv("RT_ONE_FILE") != nullptr;
    if (oneFile) return 0;
    const int files = std::clamp(static_cast<int>(std::floor((junctionPad_ - 1.7) / 0.9)) + 1, 1, 3);
    return files <= 1 ? 0 : static_cast<int>((a.brain >> 20) % static_cast<uint32_t>(files));
}

Real CitySim::walkerVerge(const Agent& a) const { return 1.0 + 0.9 * static_cast<Real>(walkerFile(a)); }

std::vector<Vec2> CitySim::lanePath(int agentIndex, Real step) const {
    std::vector<Vec2> out;
    if (agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size())) return out;
    Agent c = agents_[static_cast<std::size_t>(agentIndex)];
    if (!c.route.found || c.route.steps.empty() || !roads_) return out;
    c.leg = 0;
    c.distOnLeg = 0;
    c.moving = true;
    step = std::max(step, Real(0.5));
    for (int guard = 0; guard < 100000; ++guard) {
        refreshPose(c);
        if (out.empty() || (c.pos - out.back()).length() > 1e-6) out.push_back(c.pos);
        if (glideAlong(c, step, 0.0) || routeLeft(c) <= 1e-6) {
            refreshPose(c);
            if ((c.pos - out.back()).length() > 1e-6) out.push_back(c.pos);
            break;
        }
    }
    return out;
}

void CitySim::walkPoses(int agentIndex, Real step, const std::function<void(const Agent&)>& visit) const {
    if (agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size())) return;
    Agent c = agents_[static_cast<std::size_t>(agentIndex)];
    if (!onRoute(c) || !roads_) return;
    c.speed = 0;
    step = std::max(step, Real(0.25));
    for (int guard = 0; guard < 100000; ++guard) {
        refreshPose(c);
        c.tickFromPos = c.pos;      // a pose, not a motion: drawn where it is
        visit(c);
        if (routeLeft(c) <= 1e-6) return;
        glideAlong(c, step, 0.0);   // (held at the end: the last pose is where it stops)
    }
}

uint32_t CitySim::rnd() { return rng_.bits(); }
Real CitySim::rndUnit() { return rng_.next(); }

// A draw from THIS AGENT'S OWN stream. Trip decisions (which lane to take, where
// to wander to) used to come off the shared `rng_`, which made every agent's
// choices depend on how many decisions every OTHER agent had made first. That is
// fine while the population is stepped in one fixed order, and fatal the moment
// anything advances an agent out of turn — which is exactly what waking a
// dormant agent from its schedule does. Per-agent streams make a decision depend
// only on how many decisions that agent has made.
//
// Seeded from `brain` by hash at build (never a draw), so the build stream — and
// every seeded scenario — is untouched.
uint32_t CitySim::tripRnd(Agent& a) { return a.tripRng.bits(); }

void CitySim::build(int pedCount, uint32_t seed) {
    // THE WALK POINTS (city_walk_points.h): along the walks of every part a walker can get round that is long enough;
    // the places' doors are added by assignPlaces
    walkPoints_ = WalkPointMap{};
    if (roads_ && roads_->built) walkPoints_.build(*roads_, walker_.open, walkParts_);
    // THE NEW DRIVING MODEL STARTS OVER TOO (issue #284): the agents and vehicles it was handed are about to go. Its host
    // forgets them (seats kept would name new agents as the new model's and keep them from the old one), and the model
    // is made afresh on the same road with the same vehicle classes: no cars, no trips, no rules until startDrivingV2.
    if (drivingV2_) {
        drivingV2_->attachMeasurements(nullptr);
        driving::VehicleClasses classes = drivingV2_->vehicleClasses();
        const engine::RoadModel* roads = drivingV2_->model();
        drivingV2_.emplace(roads);
        drivingV2_->setVehicleClasses(std::move(classes));
    }
    drivingHost_.reset();
    buses_.reset();
    brainRideEvents_.clear();
    rebuildTransit();
    brain_.reset();   // the resident with the agent brain names the old agents too (startBrain makes it again)
    agents_.clear();
    sensed_.clear();
    externalObstacles_.clear();   // a REBUILD must not keep the previous level's
    staticObstacles_.clear();     // player/pole positions as phantom obstacles
    obstacleCells_.clear();
    obstacleMaxClearance_ = 0;
    clockHours_ = 8.5;   // start mid morning-rush so agents commute right away
    simSeconds_ = 0;
    rng_ = engine::ProcRng(seed);
    seed_ = seed;
    ids_.reset();   // fresh scene → UIDs restart at 0 (reproducible from seed)
    // (the brain's attention centre is re-fed by the host each step)
    haveTierCenter_ = false;
    grid_.configure(engine::Vec2(0, 0), engine::Vec2(1, 1), kAgentGridCell, 0);
    rebuildRoleTables();

    const int n = walkPoints_.streetNodeCount();   // spawns and random homes/works: the street points
    if (n == 0) return;

    agents_.reserve(pedCount);
    for (int i = 0; i < pedCount; ++i) {
        Agent a;
        a.uid = ids_.next();   // stable identity for relationships / memory / jobs
        a.home = static_cast<int>(rnd() % n);
        a.work = static_cast<int>(rnd() % n);
        // Pick a work node the home can actually REACH. An unroutable pair used to
        // teleport the agent to its goal on departure (it "disappeared" and
        // reappeared); instead insist on a routable pair, and if none turns up
        // just keep the agent home (work == home -> it never commutes).
        {
            // BOTH directions must route (the graph is directed — one-way ramps):
            // a valid home->work with no work->home used to retry a failing A*
            // every single tick once the agent wanted to come home.
            // (on foot, there and back: the same part of the walks)
            auto commutable = [&](int h, int w) { return w != h && walkPoints_.samePart(h, w); };
            bool ok = commutable(a.home, a.work);
            for (int tries = 0; tries < 8 && !ok && n > 1; ++tries) {
                a.work = static_cast<int>(rnd() % n);
                ok = commutable(a.home, a.work);
            }
            if (!ok) a.work = a.home;   // stranded: stay put, never depart
        }
        // SHIFT DIVERSITY jitter. The two draws stay HERE, at this exact point
        // in the rng stream — the shift ladder that consumes them lives below
        // the brain seed (see there for why), and moving these would shift every
        // later draw and re-pin every seeded scenario in the suite.
        const Real j0 = rndUnit(), j1 = rndUnit();       // per-agent jitter
        a.activity = Agent::Activity::AtHome;
        a.goal = tableFor(a).entry();   // the day starts at the table's entry
        a.brain = rnd() | 1u;            // the agent's fixed identity bits
        // its trip stream seeded from the brain, not drawn, so the build sequence is the same with or without it
        a.tripRng = engine::ProcRng(a.brain, 0x68E31DA4B4F2A8C1ull);
        // SHIFT DIVERSITY. A single 7.5-9.0 departure window put the whole
        // population on the road at once: one hot rush, a dead city either
        // side of it, and a sim-cost spike where nearly every agent is moving
        // in the same minutes. Real cities run shifts, so agents draw one from
        // their own brain bits (deterministic, no rng draw) — the city now has
        // hot and cold hours per district instead of one synchronised surge.
        //
        // THIS BLOCK MUST FOLLOW THE BRAIN SEED. It used to sit above it and so
        // read the default `brain = 1`, making `shift` 0 for EVERY agent: the
        // whole city took the first branch, the five archetypes below never
        // existed, and — because the build clock starts at 8.5, inside every
        // agent's [5.0, 14.0) window — the entire fleet departed on the first
        // think and spent the opening minutes of play as one city-wide jam.
        {
            const uint32_t shift = (a.brain >> 12) & 0xFF;   // 0..255
            if (shift < 26) {            // ~10% early shift (trades, transit)
                a.departWork = 5.0 + j0 * 1.5;
                a.departHome = 14.0 + j1 * 1.5;
            } else if (shift < 166) {    // ~55% standard day, widened
                a.departWork = 7.0 + j0 * 2.0;
                a.departHome = 16.0 + j1 * 2.5;
            } else if (shift < 204) {    // ~15% swing / late start
                a.departWork = 11.5 + j0 * 3.0;
                a.departHome = 20.0 + j1 * 3.0;
            } else if (shift < 227) {    // ~9% night shift (wraps midnight)
                a.departWork = 21.5 + j0 * 1.5;
                a.departHome = 5.5 + j1 * 1.5;
            } else {                     // ~11% part-time / midday
                a.departWork = 9.5 + j0 * 2.0;
                a.departHome = 14.5 + j1 * 2.0;
            }
        }
        // Personality from the brain's own bits (NOT an rnd() draw, so the build
        // stream — and every seeded test scenario — is unchanged).
        a.speedFactor = paceFactor(a.brain);
        // Stagger the think clocks so the crowd doesn't re-decide in lockstep.
        a.thinkTimer = thinkPeriod_ * thinkStagger(a.brain);
        a.restNode = a.home;
        a.pos = idlePose(a.home, a.brain);
        // Idle bodies face along their walk from the start.
        a.heading = walkPoints_.along[static_cast<std::size_t>(a.home)];
        agents_.push_back(a);
    }

    // P4.1: size the spatial grid to the spots (padded for verges, doorsteps
    // and corner blends) and hash the initial cohort.
    {
        Vec2 lo = walkPoints_.nodes[0], hi = walkPoints_.nodes[0];
        for (const Vec2& p : walkPoints_.nodes) {
            lo.x = std::min(lo.x, p.x);
            lo.y = std::min(lo.y, p.y);
            hi.x = std::max(hi.x, p.x);
            hi.y = std::max(hi.y, p.y);
        }
        const Vec2 pad(kGridPad, kGridPad);
        grid_.configure(lo - pad, hi + pad, kAgentGridCell,
                        static_cast<int>(agents_.size()));
        pedGrid_.configure(lo - pad, hi + pad, kAgentGridCell, static_cast<int>(agents_.size()));
        pedGridIds_.clear();
        rehashAll_ = true;
        for (std::size_t i = 0; i < agents_.size(); ++i)
            grid_.place(static_cast<int>(i), agents_[i].pos);
    }
    // Measure the commute here as well as after assignPlaces: a level with no
    // authored places never calls that, and scheduleSnapshot (and the host's
    // clock-scale warning) both need a travel time to be meaningful.
    measureCommute();
    if (!rail_.lines().empty()) trainRiders_.build(&rail_, walkPoints_); else trainRiders_.clear();   // the stations' ways in
    rebuildTransit();
}


namespace {
// ---- the population cache (CitySim::setPopulationCacheDir) --------------------------------------------
constexpr uint32_t kPopulationFormat = 14;   // (14: homes, jobs and rests are spots on the road model's walks, not street-map nodes, #306; 13: no drivers -- the record keeps no mode or archetype, ADR-0463; 12: no buses, no towns -- the bus network's street networks -- and no bus commuters, ADR-0460; 11: a walker's job is in reach on foot, by train or by bus, its commute timed so; 10: roles from life stages and jobs, lodgers; 9: households in the dwellings, life stages decide who works; 8: place kinds from places.lua, not campus codes; 7: homes and jobs drawn by floor area; 6: garage stalls are bays) bump when assignPlaces' rules or this record change
struct Fnv : engine::bundle::Fnv {
    void real(Real v) { const double d = static_cast<double>(v); pod(d); }
    void vec(const engine::Vec2& v) { real(v.x); real(v.y); }
};
// One agent's assignment, as stored.
struct PopRecord {
    int32_t home, work, shop, restNode;
    uint32_t homePlace, workPlace, shopPlace;
    double homeDoor[2], workDoor[2], shopDoor[2], pos[2];
    double heading[2], departHome, departWork, commuteSeconds;
    uint8_t role, indoors;
};
struct PopTail {
    double medianAll, medianWalk;
};
}  // namespace

void CitySim::assignPlaces(const PlaceMap& places) {
    relationships_.clear();
    for (Agent& a : agents_) { a.homePlace = kNoPlace; a.workPlace = kNoPlace; a.household = -1; a.age = 0; a.stage = -1; }
    households_ = HouseholdPlan{};
    unhoused_ = 0;
    // WHERE EACH PLACE'S DOOR FRONTS THE ROAD (ADR-0474, carried onto the place by ADR-0480): a trip to or from an
    // agent's home, work or shop starts and ends there (startTrip)
    placeFrontage_.clear();
    for (const Place& p : places.places()) placeFrontage_.push_back(p.frontage);
    if (places.empty() || walkPoints_.empty()) return;

    // A hall with BEDS (the university's residence hall) is the STUDENTS' home (below), not one more house in the pool.
    const uint32_t kBeds = placeKinds_.tag("beds");
    std::vector<PlaceId> homes;
    for (PlaceId id : places.ofType(PlaceType::Home))
        if (!(places[id].tags & kBeds)) homes.push_back(id);
    if (homes.empty()) homes = places.ofType(PlaceType::Home);
    const std::vector<PlaceId>& parks = places.ofType(PlaceType::Park);
    // A "job" is a workplace place: shop / office / civic (a park is not a job).
    std::vector<PlaceId> jobs;
    for (PlaceType t : {PlaceType::Shop, PlaceType::Office, PlaceType::Civic,
                        PlaceType::Cafe, PlaceType::Restaurant, PlaceType::Supermarket, PlaceType::Bar, PlaceType::Club})
        for (PlaceId id : places.ofType(t)) jobs.push_back(id);
    venues_.clear();
    if (homes.empty()) return;   // nowhere to live → leave the built schedule alone

    // THE HOUSEHOLDS (households.h; Glenn, 2026-11-01: "a family would be expressed by like a unit"). The dwellings
    // and the halls' beds are filled with as many people as there are residents -- the buses' and cabs' drivers aside
    // -- and each member is matched to an agent: a child or a student to a walker, the adults to the rest, the
    // drivers spread evenly over them. Sequential and cheap; decided on a cache hit too (the cache holds what follows).
    std::vector<std::vector<std::pair<int, uint8_t>>> housemates;   // per household: (agent, its member group)
    {
        std::vector<int> drivers, walkers;
        for (int i = 0; i < static_cast<int>(agents_.size()); ++i) walkers.push_back(i);   // (nobody drives: ADR-0463)
        households_ = planHouseholds(people_, places, kBeds, static_cast<int>(drivers.size() + walkers.size()),
                                     static_cast<int>(walkers.size()), seed_);
        const int adults = static_cast<int>(households_.people.size()) - households_.walkersOnly;
        const int adultDrivers = std::min(static_cast<int>(drivers.size()), adults);
        std::size_t nextD = 0, nextW = 0;
        int adult = 0;
        housemates.assign(households_.households.size(), {});
        for (std::size_t h = 0; h < households_.households.size(); ++h) {
            const Household& hh = households_.households[h];
            for (uint32_t m = hh.first; m < hh.first + hh.count; ++m) {
                const Resident& r = households_.people[m];
                bool drive = false;
                if (!r.walks) {   // this adult's turn to drive: the even spread of adultDrivers over the adults
                    drive = static_cast<long long>(adult + 1) * adultDrivers / adults > static_cast<long long>(adult) * adultDrivers / adults;
                    ++adult;
                }
                int ai = -1;
                if (drive && nextD < drivers.size()) ai = drivers[nextD++];
                else if (nextW < walkers.size()) ai = walkers[nextW++];
                else if (!r.walks && nextD < drivers.size()) ai = drivers[nextD++];
                if (ai < 0) continue;   // (the plan housed no more people than there are: never)
                Agent& a = agents_[static_cast<std::size_t>(ai)];
                a.household = static_cast<int32_t>(h);
                a.age = r.age;
                a.stage = static_cast<int8_t>(r.stage);
                housemates[h].push_back({ai, r.group});
            }
        }
        unhoused_ = static_cast<int>(drivers.size() + walkers.size() - nextD - nextW);
        // LODGERS: anyone past the housing (a level's own count) is someone on their own, of people.lua's lodger bands
        for (const std::vector<int>* pool : {&drivers, &walkers})
            for (std::size_t k = pool == &drivers ? nextD : nextW; k < pool->size(); ++k) {
                Agent& a = agents_[static_cast<std::size_t>((*pool)[k])];
                const Resident r = drawLodger(people_, a.brain * 0x2545F491u);
                a.age = r.age;
                a.stage = static_cast<int8_t>(r.stage);
            }
    }
    // EACH STAGE'S ROLE (people.lua `role`, a roles.lua name); a stage with a job takes the job's
    std::vector<int> stageRole(people_.stages.size(), catchAllRole_);
    for (std::size_t st = 0; st < people_.stages.size(); ++st) {
        const LifeStage& ls = people_.stages[st];
        if (ls.job) continue;
        const int r = roles_.find(ls.role);
        if (r >= 0) stageRole[st] = r;
        else std::fprintf(stderr, "[citysim] WARN: stage `%s` holds role `%s`, which roles.lua does not have (they work)\n",
                          ls.name.c_str(), ls.role.c_str());
    }

    // THE SPOT OF EACH PLACE: its door joined to the walks -- where it states its door fronts (RoadModel::walkJoinOn),
    // else by its entrance's position (walkJoinNear) -- given a spot of its own, each made once (in place order, so the
    // same places give the same spots). -1: its door joins no walk.
    walkPoints_.dropPlacePoints();
    std::vector<int> placeNode(places.places().size(), -2);
    auto nodeOf = [&](PlaceId id) {
        int& n = placeNode[static_cast<std::size_t>(id)];
        if (n != -2) return n;
        n = -1;
        if (!roads_ || !roads_->built) return n;
        const Place& pl = places[id];
        engine::WalkJoin j;
        if (pl.frontage.onStreet() || pl.frontage.onWalkway()) j = roads_->walkJoinOn(pl.frontage, walker_.open, &walkParts_);
        if (!j.valid()) j = roads_->walkJoinNear(pl.entrance, walker_.open, &walkParts_);
        if (j.valid()) n = walkPoints_.add(*roads_, j.at, static_cast<int>(j.part));
        else n = walkPoints_.nearestNode(pl.entrance);   // (a door that joins no walk: the spot nearest it)
        return n;
    };
    for (PlaceId id = 0; id < static_cast<PlaceId>(places.places().size()); ++id) nodeOf(id);
    placePoint_ = placeNode;   // (assignStudents, and the trips: each place's spot)
    if (!rail_.lines().empty()) trainRiders_.build(&rail_, walkPoints_);   // (the places' spots now among the street points)
    rebuildTransit();
    // THERE AND BACK: two walk points are commutable exactly when they sit in the same part of the walks. (The
    // old sim's cars went, ADR-0461: a commute is walked, or ridden by train between towns.)
    auto sameComp = [&](int u, int v) { return walkPoints_.samePart(u, v); };
    // ...and ON FOOT (the people census, 2026-11-02: walkers given a job in another town -- joined to home by road but
    // by no footway -- never had a commute measured, and could only have got there by luck): a walker's job is in its
    // home's part of the walks, or in one a train line joins it to -- in the train riders' own towns (TrainRiders::
    // networkOf), so what passes here is what TrainRiders::doorToDoor will price; each pair of stations' towns asked once.
    std::set<std::pair<int, int>> railJoined;
    if (trainRiders_.active()) {
        const std::vector<RailStation>& st = trainRiders_.stations();
        for (const RailStation& s1 : st)
            for (const RailStation& s2 : st) {
                const int n1 = s1.accessNode, n2 = s2.accessNode;
                if (n1 < 0 || n2 < 0 || n1 == n2) continue;
                const int c1 = trainRiders_.networkOf(n1), c2 = trainRiders_.networkOf(n2);
                if (c1 < 0 || c2 < 0 || c1 == c2 || railJoined.count({c1, c2})) continue;
                if (trainRiders_.doorToDoor(n1, walkPoints_.nodes[static_cast<std::size_t>(n1)], n2, walkPoints_.nodes[static_cast<std::size_t>(n2)]))
                    railJoined.insert({c1, c2});
            }
    }
    auto walkable = [&](int h, int w) {
        if (h < 0 || w < 0 || h == w) return false;
        if (walkPoints_.samePart(h, w)) return true;
        const int rh = trainRiders_.networkOf(h), rw = trainRiders_.networkOf(w);
        return rh >= 0 && rw >= 0 && rh != rw && railJoined.count({rh, rw}) > 0;
    };
    // THE JOB SEARCH's test: there and back by car for a driver, on foot (or by train) for a walker
    auto reachable = [&](const Agent&, int h, int w) { return walkable(h, w); };

    auto doorOf = [&](PlaceId id) { return placeDoor(places[id]); };
    // Everywhere people go OUT to: every place but a home. What outings and
    // lunch breaks choose from at trip time (pickOuting / pickLunch).
    for (const Place& p : places.places()) {
        if (p.type == PlaceType::Home) continue;
        Venue v;
        v.type = p.type;
        v.node = nodeOf(p.id);
        v.door = doorOf(p.id);
        v.openHour = p.openHour;
        v.closeHour = p.closeHour;
        v.kind = p.kind;
        v.tags = p.tags;
        v.place = p.id;
        if (v.node >= 0) venues_.push_back(v);
    }

    // THE VENUES BY CELL (a day-off walker's anchor was the nearest venue by a scan of every venue: 80k walkers x
    // ~15k venues on a 309k island). Rings outward from the home's cell until no nearer ring can hold a closer one;
    // ties go to the lower venue index, exactly as the scan's strict < did.
    constexpr Real kVenueCell = 250.0;
    std::unordered_map<int64_t, std::vector<int>> venueCells;
    int64_t vx0 = INT64_MAX, vx1 = INT64_MIN, vz0 = INT64_MAX, vz1 = INT64_MIN;
    for (int vi = 0; vi < static_cast<int>(venues_.size()); ++vi) {
        const Vec2 q = walkPoints_.nodes[static_cast<std::size_t>(venues_[static_cast<std::size_t>(vi)].node)];
        const int64_t cx = static_cast<int64_t>(std::floor(q.x / kVenueCell)), cz = static_cast<int64_t>(std::floor(q.y / kVenueCell));
        vx0 = std::min(vx0, cx); vx1 = std::max(vx1, cx); vz0 = std::min(vz0, cz); vz1 = std::max(vz1, cz);
        venueCells[(cx << 32) ^ (cz & 0xffffffff)].push_back(vi);
    }
    auto nearestVenueNode = [&](int hn) {
        const Vec2 p = walkPoints_.nodes[static_cast<std::size_t>(hn)];
        const int64_t cx = static_cast<int64_t>(std::floor(p.x / kVenueCell)), cz = static_cast<int64_t>(std::floor(p.y / kVenueCell));
        const int64_t maxRing = std::max({cx - vx0, vx1 - cx, cz - vz0, vz1 - cz, int64_t(0)});
        Real best = 1e30;
        int bestIdx = -1;
        auto visit = [&](int64_t x, int64_t z) {
            auto it = venueCells.find((x << 32) ^ (z & 0xffffffff));
            if (it == venueCells.end()) return;
            for (int vi : it->second) {
                const Venue& v = venues_[static_cast<std::size_t>(vi)];
                if (v.node == hn) continue;
                const Vec2 d = walkPoints_.nodes[static_cast<std::size_t>(v.node)] - p;
                const Real d2 = d.x * d.x + d.y * d.y;
                if (d2 < best || (d2 == best && vi < bestIdx)) { best = d2; bestIdx = vi; }
            }
        };
        for (int64_t r = 0; r <= maxRing; ++r) {
            for (int64_t x = cx - r; x <= cx + r; ++x) {
                visit(x, cz - r);
                if (r > 0) visit(x, cz + r);
            }
            for (int64_t z = cz - r + 1; z <= cz + r - 1; ++z) {
                visit(cx - r, z);
                visit(cx + r, z);
            }
            // every cell of ring r+1 is at least r cells' width from p
            if (bestIdx >= 0 && best <= (r * kVenueCell) * (r * kVenueCell)) break;
        }
        return bestIdx >= 0 ? venues_[static_cast<std::size_t>(bestIdx)].node : -1;
    };

    // THE JOBS BY CELL (pushing past 100k: a walker's job search measured every job on the island against every
    // walker -- 250,000 x ~15,000 -- 165 s of a 309k load). Square cells; a search reads only the cells it needs.
    constexpr Real kJobCell = 250.0;
    std::unordered_map<int64_t, std::vector<PlaceId>> jobCells;
    auto jobCellKey = [](int64_t cx, int64_t cz) { return (cx << 32) ^ (cz & 0xffffffff); };
    Vec2 jobLo(1e30, 1e30), jobHi(-1e30, -1e30);
    // every job within the square of half-side r round p (a superset of the circle)
    auto jobsNear = [&](Vec2 p, Real r, std::vector<std::pair<Real, PlaceId>>& out) {
        out.clear();
        const int64_t x0 = static_cast<int64_t>(std::floor((p.x - r) / kJobCell)), x1 = static_cast<int64_t>(std::floor((p.x + r) / kJobCell));
        const int64_t z0 = static_cast<int64_t>(std::floor((p.y - r) / kJobCell)), z1 = static_cast<int64_t>(std::floor((p.y + r) / kJobCell));
        for (int64_t cx = x0; cx <= x1; ++cx)
            for (int64_t cz = z0; cz <= z1; ++cz) {
                auto it = jobCells.find(jobCellKey(cx, cz));
                if (it == jobCells.end()) continue;
                for (PlaceId cand : it->second) {
                    const Vec2 d = places[cand].site - p;
                    out.push_back({d.x * d.x + d.y * d.y, cand});
                }
            }
    };

    // THE POPULATION CACHE: the key is everything the decisions below read.
    popCache_ = PopulationCacheReport{};
    std::string popPath;
    std::vector<PopRecord> cached;
    PopTail cachedTail{};
    const bool verify = std::getenv("RT_POPULATION_VERIFY") != nullptr;
    if (!populationCacheDir_.empty()) {
        Fnv k;
        k.pod(kPopulationFormat);
        k.str(placeKinds_.describe());   // the kinds' tags and beds decide who lives where
        k.str(people_.describe());       // ...and the households who
        k.pod(seed_);
        k.pod(static_cast<uint32_t>(places.dwellings().size()));
        for (const PlaceMap::Dwelling& d : places.dwellings()) { k.pod(d.place); k.pod(d.beds); }
        k.pod(static_cast<int32_t>(walkPoints_.nodeCount()));
        for (std::size_t s = 0; s < walkPoints_.nodes.size(); ++s) {
            k.vec(walkPoints_.nodes[s]);
            k.pod(walkPoints_.at[s].walk.index);
            k.real(walkPoints_.at[s].along);
            k.pod(static_cast<int32_t>(walkPoints_.part[s]));
        }
        for (const Place& p : places.places()) {
            k.pod(p.id); k.pod(p.type); k.vec(p.site); k.vec(p.entrance);
            k.real(p.openHour); k.real(p.closeHour); k.pod(p.capacity); k.pod(p.authoredHours); k.str(placeKinds_.nameOf(p.kind)); k.real(p.floorArea);
        }
        for (std::size_t i = 0; i < agents_.size(); ++i) {
            const Agent& a = agents_[i];
            k.pod(a.uid); k.pod(a.brain); k.real(a.speedFactor);
            k.real(a.departHome); k.real(a.departWork);
        }
        { const std::string rd = roles_.describe(); k.bytes(rd.data(), rd.size()); }   // the roles hire the staff
        popCache_.used = true;
        popCache_.key = k.h;
        char name[40];
        std::snprintf(name, sizeof name, "%016llx.pop", static_cast<unsigned long long>(k.h));
        popPath = populationCacheDir_ + "/" + name;
        std::ifstream in(popPath, std::ios::binary);
        char magic[6] = {0};
        uint32_t fmt = 0, count = 0;
        uint64_t key = 0;
        if (in && in.read(magic, 6) && std::string(magic, 6) == "RTPOP1" && in.read(reinterpret_cast<char*>(&fmt), 4) &&
            fmt == kPopulationFormat && in.read(reinterpret_cast<char*>(&key), 8) && key == k.h &&
            in.read(reinterpret_cast<char*>(&count), 4) && count == agents_.size()) {
            cached.resize(count);
            if (in.read(reinterpret_cast<char*>(cached.data()), static_cast<std::streamsize>(count * sizeof(PopRecord))) &&
                in.read(reinterpret_cast<char*>(&cachedTail), sizeof cachedTail)) {
                popCache_.hit = true;
                engine::bundle::noteCacheUse(popPath, "population");
            }
            else
                cached.clear();
        }
    }
    auto restore = [&](Agent& a, const PopRecord& r) {
        a.home = r.home; a.work = r.work; a.shop = r.shop; a.restNode = r.restNode;
        a.homePlace = r.homePlace; a.workPlace = r.workPlace; a.shopPlace = r.shopPlace;
        a.homeDoor = Vec2(r.homeDoor[0], r.homeDoor[1]); a.workDoor = Vec2(r.workDoor[0], r.workDoor[1]);
        a.shopDoor = Vec2(r.shopDoor[0], r.shopDoor[1]); a.pos = Vec2(r.pos[0], r.pos[1]);
        a.heading = Vec2(r.heading[0], r.heading[1]); a.departHome = r.departHome; a.departWork = r.departWork;
        a.commuteSeconds = r.commuteSeconds;
        a.role = static_cast<RoleId>(r.role); a.indoors = r.indoors != 0;
    };
    auto record = [](const Agent& a) {
        PopRecord r;
        std::memset(&r, 0, sizeof r);   // padding too: VERIFY compares records byte for byte
        r.home = a.home; r.work = a.work; r.shop = a.shop; r.restNode = a.restNode;
        r.homePlace = a.homePlace; r.workPlace = a.workPlace; r.shopPlace = a.shopPlace;
        r.homeDoor[0] = a.homeDoor.x; r.homeDoor[1] = a.homeDoor.y; r.workDoor[0] = a.workDoor.x; r.workDoor[1] = a.workDoor.y;
        r.shopDoor[0] = a.shopDoor.x; r.shopDoor[1] = a.shopDoor.y; r.pos[0] = a.pos.x; r.pos[1] = a.pos.y;
        r.heading[0] = a.heading.x; r.heading[1] = a.heading.y; r.departHome = a.departHome; r.departWork = a.departWork; r.commuteSeconds = a.commuteSeconds;
        r.role = static_cast<uint8_t>(a.role); r.indoors = a.indoors ? 1 : 0;
        return r;
    };
    if (popCache_.hit && !verify) {
        for (std::size_t i = 0; i < agents_.size(); ++i) restore(agents_[i], cached[i]);
        commuteSecondsMedian_ = cachedTail.medianAll;
        commuteSecondsWalk_ = cachedTail.medianWalk;
    } else {
    // THREADED (Glenn: "We should have them do that logic on another thread maybe?"). Each agent's decisions read
    // only shared, settled data -- places, the graph, the job and venue indexes, the bus network -- and write only
    // that agent, so they run across the cores and come out identical (RT_POPULATION_VERIFY checks it). What was
    // lazy and shared is settled first: every place's node, the job cells.
    engine::JobSystem& decide = engine::sharedJobs();   // the process's pool (ADR-0288)
    decide.parallelFor(0, places.places().size(), [&](std::size_t pi) { nodeOf(static_cast<PlaceId>(pi)); }, 64);
    for (PlaceId cand : jobs) {
        const Vec2 q = places[cand].site;
        jobCells[jobCellKey(static_cast<int64_t>(std::floor(q.x / kJobCell)), static_cast<int64_t>(std::floor(q.y / kJobCell)))].push_back(cand);
        jobLo = Vec2(std::min(jobLo.x, q.x), std::min(jobLo.y, q.y));
        jobHi = Vec2(std::max(jobHi.x, q.x), std::max(jobHi.y, q.y));
    }
    // PEOPLE WHERE THE FLOORS ARE (the busy streets): a home or a job is drawn in proportion to its floor area, not one
    // per building -- a 30-storey block of flats housed as many as a bungalow, so downtown, its towers full of offices
    // and flats, held 711 people within 300 m of the core and every street looked like a suburb's. A place with no
    // area (an authored marker) weighs as a house. Cumulative weights per pool; a draw is one binary search.
    auto weightOf = [&](PlaceId id) {
        const Real m2 = places[id].floorArea;
        return m2 > 0 ? std::clamp(static_cast<double>(m2), 40.0, 60000.0) : 120.0;
    };
    std::unordered_map<const std::vector<PlaceId>*, std::vector<double>> poolCum;
    auto addPool = [&](const std::vector<PlaceId>& v) {
        std::vector<double>& c = poolCum[&v];
        c.resize(v.size());
        double t = 0;
        for (std::size_t j = 0; j < v.size(); ++j) c[j] = (t += weightOf(v[j]));
    };
    addPool(homes);
    addPool(jobs);
    auto drawFrom = [&](const std::vector<PlaceId>& v, uint32_t h) {
        const std::vector<double>& c = poolCum.at(&v);
        const double u = (static_cast<double>(h) + 0.5) / 4294967296.0 * c.back();
        const std::size_t j = static_cast<std::size_t>(std::upper_bound(c.begin(), c.end(), u) - c.begin());
        return v[std::min(j, v.size() - 1)];
    };
    decide.parallelFor(0, agents_.size(), [&](std::size_t ai) {
        Agent& a = agents_[ai];
        thread_local std::vector<std::pair<Real, PlaceId>> jobDist;   // the on-foot job search's scratch
        // Home: deterministic pick from the agent's own brain bits (no rng
        // draw). MIXED first: brains are forced odd (rnd()|1), so a raw
        // modulo could only ever reach odd home indices when homes.size()
        // is even — half the housing stock sat empty.
        const std::uint32_t hmix = engine::lowbias32(a.brain);
        // A HOUSEHOLD'S HOME is its dwelling's (or its hall's); someone the housing could not hold takes one by area.
        PlaceId hp = a.household >= 0 ? households_.households[static_cast<std::size_t>(a.household)].place : drawFrom(homes, hmix);
        int hn = nodeOf(hp);
        a.homePlace = hp;
        a.home = hn;
        a.homeDoor = doorOf(hp);
        a.restNode = hn;
        // Walkers start the day at their door (indoors); cars stay parked on
        // the verge (idlePose) — a car "at home" is a car parked outside it.
        a.pos = a.homeDoor;
        a.indoors = true;
        if (hn >= 0) a.heading = walkPoints_.along[static_cast<std::size_t>(hn)];

        // THE ROLE (Phase 4; #172) decides how the day runs. WHO WORKS is their life stage's (people.lua): a child,
        // someone at home, the retired hold their stage's role and spend the day out -- a park, a cafe, the shops; a
        // working resident's role is the job they find (roles.lua `job`: the shop they keep, else the catch-all).
        // A bus's or a cab's driver has no stage, and works.
        a.work = a.home;
        const LifeStage* stage = a.stage >= 0 ? &people_.stages[static_cast<std::size_t>(a.stage)] : nullptr;
        const bool works = !stage || stage->job;
        a.role = static_cast<RoleId>(works ? catchAllRole_ : stageRole[static_cast<std::size_t>(a.stage)]);
        const bool stroller = !parks.empty() && !works;
        auto unit = [](uint32_t bits) { return (bits & 0xFF) / 255.0; };
        // A WALKER WITH THE DAY OFF (Glenn, 2026-09-19: "we should have non
        // workers and pedestrians who are out for a stroll or going to public
        // spaces"): a walker with no job spends part of the day out -- park,
        // cafe, shops, a walk round the block (the outing table,
        // strollerGoals). Their outings are spread over the day (early,
        // mid-morning, afternoon, evening) so the street is never just the
        // commute. One who finds no venue goes to a park instead.
        const bool dayOff = !venues_.empty() && !works;
        int anchor = -1;
        if (dayOff) {
            // The window mechanics need somewhere that is not home: the
            // nearest routable venue. Outings themselves choose afresh.
            anchor = nearestVenueNode(hn);
            if (anchor >= 0 && !reachable(a, hn, anchor)) anchor = -1;
        }
        if (anchor >= 0) {
            a.work = anchor;
            a.workPlace = kNoPlace;
            a.workDoor = walkPoints_.nodes[static_cast<std::size_t>(anchor)];
            const Real u0 = unit(a.brain >> 4), u1 = unit(a.brain >> 12);
            switch ((a.brain >> 28) & 3u) {
                case 0:  a.departWork = 6.5 + 1.5 * u0;  a.departHome = a.departWork + 2.0 + 2.0 * u1; break;
                case 1:  a.departWork = 9.0 + 2.0 * u0;  a.departHome = a.departWork + 3.0 + 3.0 * u1; break;
                case 2:  a.departWork = 12.5 + 2.0 * u0; a.departHome = a.departWork + 3.0 + 2.0 * u1; break;
                default: a.departWork = 17.5 + 1.5 * u0; a.departHome = a.departWork + 2.0 + 2.0 * u1; break;
            }
        } else if (stroller) {
            PlaceId pk = parks[(a.brain >> 4) % parks.size()];
            if (reachable(a, hn, nodeOf(pk))) {
                a.work = nodeOf(pk);              // a daytime destination, not a job
                a.workPlace = kNoPlace;
                a.workDoor = doorOf(pk);          // rests at the park, not the kerb
                a.departWork = 9.5 + 2.0 * unit(a.brain);         // late-morning out
                a.departHome = 15.5 + 1.5 * unit(a.brain >> 8);   // home mid-afternoon
            }
        } else if (!jobs.empty() && works) {   // (a child, say, with no outing to go to stays home)
            // Prefer the brain-picked workplace; if it isn't routable from
            // home, scan for any that is — STARTING FROM A PER-AGENT OFFSET.
            // Scanning from index 0 funneled every fallback agent onto the
            // FIRST routable workplace: one office, one street, the whole
            // fleet parked in a line (Glenn's "cars all on one street").
            // WORKPLACE CHOICE IS GRAVITY-BIASED, not uniform. Offices, shops
            // and civic buildings concentrate downtown by district, so drawing
            // uniformly over them sent most of the city to the same few blocks
            // — a permanent downtown crush, and every agent inside the
            // player's bubble at once when he stands there. Real commuting
            // follows distance decay: sample a handful of candidates from the
            // agent's own bits and take the NEAREST routable one. Because the
            // candidates are random, long cross-town commutes still happen —
            // they just stop being the default.
            const Vec2 homePos = places[hp].site;
            PlaceId pick = kNoPlace;
            // SHORTER COMMUTES (Glenn, 2026-09-16: "the agents should have
            // shorter commutes") -- and it is NOT the same problem for both
            // modes. A walker covers ground ~5x slower, so the same home/work
            // pair costs it ~5x the day. Measured on metro_v2 after the first
            // pass: drivers 0.59 in-world hours each way, walkers 3.86. On foot: a job in a band of distance, below.
            // (The drivers' search, the nearest routable of sampled jobs, went with the old driving: ADR-0463.)
            {
                // A WALK WORTH TAKING (Glenn, 2026-09-17: "I haven't seen
                // anybody in the suburbs"). Taking the NEAREST job put walkers
                // ~70 m from home, so they reached work almost at once and sat
                // indoors: measured, moving fell 847 -> 377 at 17:00 and
                // 332 -> 76 at noon. What fills a street is not headcount but
                // the SHARE OF THE DAY spent walking -- at a 5% outdoor share
                // you need ~70k walkers for the street life 12k gives at 30%.
                // So aim for a real commute: the nearest routable job at least
                // kWalkJobFloor out, and only if none of those route does the
                // nearest of all win.
                constexpr Real kWalkJobFloor = 300.0;   // metres, straight line
                constexpr Real kWalkJobCeil  = 900.0;   // "a local place to work"
                // (from the job cells: every job within the band's reach -- the same candidates the old scan of all
                // jobs ranked, in the same (distance, id) order; widened below only if the fallback needs farther)
                jobsNear(homePos, kWalkJobCeil, jobDist);
                const auto byDist = [](const std::pair<Real, PlaceId>& x,
                                       const std::pair<Real, PlaceId>& y) {
                    if (x.first != y.first) return x.first < y.first;
                    return x.second < y.second;   // deterministic ties
                };
                // [begin, split) are far enough to be a commute; [split, end) are not.
                // BAND, not a floor. The floor alone stopped walkers working
                // 70 m away and emptying the streets; a ceiling stops them
                // crossing town. Rank: inside the band first, then beyond it,
                // then nearer than it -- nearest first within each group.
                const auto split = std::stable_partition(
                    jobDist.begin(), jobDist.end(),
                    [&](const std::pair<Real, PlaceId>& e) {
                        return e.first >= kWalkJobFloor * kWalkJobFloor &&
                               e.first <= kWalkJobCeil * kWalkJobCeil;
                    });
                const std::size_t far =
                    static_cast<std::size_t>(split - jobDist.begin());
                // ...by floor area among those in walking range (the office tower down the road, more often than
                // the corner shop): eight draws, in order of the band (sorted, so the draws are deterministic)
                std::sort(jobDist.begin(), split, byDist);
                thread_local std::vector<double> bandCum;
                bandCum.resize(far);
                double bandW = 0;
                for (std::size_t c = 0; c < far; ++c) bandCum[c] = (bandW += weightOf(jobDist[c].second));
                for (int c = 0; c < 8 && far > 0 && pick == kNoPlace; ++c) {
                    uint32_t wh = a.brain * 0x27d4eb2fU + static_cast<uint32_t>(c) * 0x9E3779B9u;
                    wh ^= wh >> 15; wh *= 0x2c1b3c6dU; wh ^= wh >> 12;
                    const double u = (static_cast<double>(wh) + 0.5) / 4294967296.0 * bandW;
                    const std::size_t j = std::min(far - 1, static_cast<std::size_t>(std::upper_bound(bandCum.begin(), bandCum.end(), u) - bandCum.begin()));
                    if (reachable(a, hn, nodeOf(jobDist[j].second))) pick = jobDist[j].second;
                }
                if (pick == kNoPlace) {   // nothing beyond the floor routes
                    // the nearest 8 OUTSIDE the band, from every job: widen the reach until 8 of them lie inside it
                    // (or it covers every job), so the ranking matches a scan of all of them
                    std::vector<std::pair<Real, PlaceId>> outside;
                    for (Real reach = kWalkJobCeil * 2;; reach *= 2) {
                        jobsNear(homePos, reach, outside);
                        outside.erase(std::remove_if(outside.begin(), outside.end(),
                                                     [&](const std::pair<Real, PlaceId>& e) {
                                                         return e.first >= kWalkJobFloor * kWalkJobFloor &&
                                                                e.first <= kWalkJobCeil * kWalkJobCeil;
                                                     }),
                                      outside.end());
                        std::size_t within = 0;
                        for (const auto& e : outside) within += e.first <= reach * reach ? 1 : 0;
                        const bool coversAll = homePos.x - reach <= jobLo.x && homePos.x + reach >= jobHi.x &&
                                               homePos.y - reach <= jobLo.y && homePos.y + reach >= jobHi.y;
                        if (within >= 8 || coversAll) break;
                    }
                    const std::size_t takeNear = std::min<std::size_t>(8, outside.size());
                    std::partial_sort(outside.begin(), outside.begin() + takeNear, outside.end(), byDist);
                    for (std::size_t c = 0; c < takeNear && pick == kNoPlace; ++c)
                        if (reachable(a, hn, nodeOf(outside[c].second)))
                            pick = outside[c].second;
                }
            }
            if (pick == kNoPlace) {   // none of the samples routed: widen out
                const std::size_t start =
                    (a.brain * 2654435761u) % jobs.size();
                for (std::size_t k = 0; k < jobs.size(); ++k) {
                    PlaceId cand = jobs[(start + k) % jobs.size()];
                    if (reachable(a, hn, nodeOf(cand))) { pick = cand; break; }
                }
            }
            if (pick != kNoPlace) {
                a.workPlace = pick;
                a.work = nodeOf(pick);
                a.workDoor = doorOf(pick);
                {   // An errand stop: the nearest shop to HOME that is routable
                    // from work, so the last leg home is short. Deterministic:
                    // nearest first, ties by place id.
                    // A store or a supermarket: the errand on the way home.
                    std::vector<PlaceId> shops = places.ofType(PlaceType::Shop);
                    for (PlaceId sp : places.ofType(PlaceType::Supermarket)) shops.push_back(sp);
                    Real bestD2 = 1e30;
                    for (PlaceId sp : shops) {
                        const Vec2 d = places[sp].site - places[hp].site;
                        const Real d2 = d.x * d.x + d.y * d.y;
                        if (d2 >= bestD2) continue;
                        const int sn = nodeOf(sp);
                        if (sn == a.work || sn == hn) continue;
                        // the shop is walked to from work and home: in their part of the walks
                        if (!sameComp(a.work, sn) || !sameComp(sn, hn)) continue;
                        bestD2 = d2; a.shop = sn; a.shopPlace = sp; a.shopDoor = doorOf(sp);
                    }
                }
                {   // What this commute COSTS, in seconds (see Agent::commuteSeconds).
                    // (on foot: the one router's walk, point to point)
                    const engine::roads::WalkRoute rt = sameComp(hn, a.work) ? pointRoute(hn, a.work) : engine::roads::WalkRoute{};
                    Real secs = rt.found ? rt.metres / std::max(Real(0.1), kWalkSpeed * a.speedFactor) : 0;
                    if (!rt.found && trainRiders_.active()) {   // another town's job: by train, door to door
                        Real railSecs = 0;
                        if (trainRiders_.doorToDoor(hn, walkPoints_.nodes[static_cast<std::size_t>(hn)], a.work,
                                                    walkPoints_.nodes[static_cast<std::size_t>(a.work)], &railSecs))
                            secs = railSecs;
                    }
                    a.commuteSeconds = secs;
                }
                // THE JOB'S ROLE (roles.lua `job`): the first role that takes this kind of place -- the shopkeeper
                // a shop whose hours the level wrote, else the catch-all worker. ONLY authored hours count: a place
                // minted from a LotBuilding carries the wide-open default (0..24, components.h:373), and running
                // that through the formula below yields departWork 0.0 / departHome 24.0 -- a window that is open
                // every hour of the day. Such an agent departs for work once and is never again outside its work
                // window, so it never goes home and its car never comes back. On piedmont, where every place is
                // lot-derived, that was every shop worker in the city. Without authored hours a worker keeps the
                // shift it drew at build.
                const Place& wp = places[pick];
                const Real open = wp.openHour, close = wp.closeHour;
                const Real dw = open > 0.5 ? open - 0.5 : 0.0;
                const Real dh = close < 23.5 ? close + 0.5 : 24.0;
                const bool authored = (open > 0.0 || close < 24.0) && wp.authoredHours && dw != dh;
                const int jr = roles_.jobRoleFor(wp.kind >= 0 ? placeKinds_.nameOf(wp.kind) : placeTypeName(wp.type), authored);
                if (jr >= 0) {
                    a.role = static_cast<RoleId>(jr);
                    if (roles_.roles[static_cast<std::size_t>(jr)].job.authoredHours) {   // open up before, close up after
                        a.departWork = dw;
                        a.departHome = dh;
                    }
                }
            }
        }
    });

    // STUDENTS are decided with everyone else, so a cache hit restores them too.
    assignStudents(places);

    // THE STAFF (Glenn: "you'd need teachers for the students, managers for shops, waiters for restaurants etc. a
    // unified way to define these is better than a lot of one offs"). Every role with a StaffSpec hires the same way:
    // each place of its kinds pulls the commuters living nearest it -- a commute they can make (by car:
    // the car graph joins them; on foot: the same walkable streets, 1.5 km at most) -- one hire per place per round,
    // so every place has its first before any has its second. At most half the commuters become staff: the
    // population stays what it was, its jobs change. Their hours are the role's (fixed) or the place's.
    {
        constexpr Real kCell = 250.0, kReach = 2500.0, kWalkReach = 1500.0;
        auto cellKey = [](int64_t cx, int64_t cz) { return (cx << 32) ^ (cz & 0xffffffff); };
        std::unordered_map<int64_t, std::vector<int>> homeCells;
        int commuters = 0;
        for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
            const Agent& a = agents_[static_cast<std::size_t>(i)];
            // the catch-all job's holders (a child, the retired, a shopkeeper are not hired away)
            if (a.role != catchAllRole_ || a.homePlace == kNoPlace || a.home < 0) continue;
            if (a.stage >= 0 && !people_.stages[static_cast<std::size_t>(a.stage)].job) continue;
            homeCells[cellKey(static_cast<int64_t>(std::floor(a.homeDoor.x / kCell)),
                              static_cast<int64_t>(std::floor(a.homeDoor.y / kCell)))].push_back(i);
            ++commuters;
        }
        auto placeKind = [&](const Place& p) -> std::string { return p.kind >= 0 ? placeKinds_.nameOf(p.kind) : placeTypeName(p.type); };
        struct Job { int role; PlaceId place; int node; int want; int got; };
        std::vector<Job> openings;
        for (int r = 0; r < static_cast<int>(roles_.roles.size()); ++r) {
            const StaffSpec& sp = roles_.roles[static_cast<std::size_t>(r)].staff;
            if (sp.places.empty() || sp.perPlace <= 0) continue;
            for (const Place& p : places.places()) {
                if (std::find(sp.places.begin(), sp.places.end(), placeKind(p)) == sp.places.end()) continue;
                const int node = nodeOf(p.id);
                if (node >= 0) openings.push_back({r, p.id, node, sp.perPlace, 0});
            }
        }
        std::vector<char> hired(agents_.size(), 0);
        // the nearest commuter not yet hired who can get to `j` every day; -1 none within reach
        auto nearestFor = [&](const Job& j) {
            const Vec2 at = places[j.place].site;
            const int64_t cx = static_cast<int64_t>(std::floor(at.x / kCell)), cz = static_cast<int64_t>(std::floor(at.y / kCell));
            const int64_t rings = static_cast<int64_t>(kReach / kCell) + 1;
            Real best = kReach * kReach;
            int pick = -1;
            for (int64_t r = 0; r <= rings; ++r) {
                for (int64_t x = cx - r; x <= cx + r; ++x)
                    for (int64_t z = cz - r; z <= cz + r; ++z) {
                        if (std::max(std::llabs(x - cx), std::llabs(z - cz)) != r) continue;   // this ring's cells only
                        auto it = homeCells.find(cellKey(x, z));
                        if (it == homeCells.end()) continue;
                        for (int i : it->second) {
                            if (hired[static_cast<std::size_t>(i)]) continue;
                            const Agent& a = agents_[static_cast<std::size_t>(i)];
                            const Real d2 = (a.homeDoor - at).lengthSquared();
                            if (d2 > best || (d2 == best && i > pick)) continue;
                            if (d2 > kWalkReach * kWalkReach || a.home == j.node || !walkable(a.home, j.node)) continue;
                            best = d2;
                            pick = i;
                        }
                    }
                if (pick >= 0 && best <= (r * kCell) * (r * kCell)) break;   // nothing in a farther ring is nearer
            }
            return pick;
        };
        auto wrap24 = [](Real h) { h = std::fmod(h, Real(24)); return h < 0 ? h + 24 : h; };
        int made = 0;
        const int quota = commuters / 2;
        for (bool any = true; any && made < quota;) {
            any = false;
            for (Job& j : openings) {
                if (j.got >= j.want || made >= quota) continue;
                const int i = nearestFor(j);
                if (i < 0) { j.want = j.got; continue; }   // nobody near enough: it hires no more
                any = true;
                ++j.got;
                ++made;
                hired[static_cast<std::size_t>(i)] = 1;
                Agent& a = agents_[static_cast<std::size_t>(i)];
                const ResidentRole& role = roles_.roles[static_cast<std::size_t>(j.role)];
                const StaffSpec& sp = role.staff;
                const Place& p = places[j.place];
                a.role = static_cast<RoleId>(j.role);
                a.workPlace = j.place;
                a.work = j.node;
                a.workDoor = doorOf(j.place);
                const Real u0 = static_cast<Real>((a.brain >> 3) & 0xFF) / 255.0, u1 = static_cast<Real>((a.brain >> 11) & 0xFF) / 255.0;
                if (sp.placeHours) {
                    Real open = p.openHour, close = p.closeHour;
                    if (open == close || (open <= 0 && close >= 24)) { open = 9; close = 18; }   // open all day: a day shift
                    const Real len = std::fmod(close - open + 24.0, 24.0);
                    Real start = open - 0.5, end = close + 0.25;
                    if (len > sp.splitOver) {   // two shifts: early, or late (its own bit)
                        const Real mid = open + len * 0.5;
                        if ((a.brain >> 19) & 1u) start = mid - 0.5; else end = mid + 0.25;
                    }
                    a.departWork = wrap24(start + 0.25 * u0);
                    a.departHome = wrap24(end);
                } else {
                    a.departWork = sp.startLo + (sp.startHi - sp.startLo) * u0;
                    a.departHome = sp.finishLo + (sp.finishHi - sp.finishLo) * u1;
                }
                const engine::roads::WalkRoute rt = pointRoute(a.home, a.work);
                a.commuteSeconds = rt.found ? rt.metres / std::max(Real(0.1), kWalkSpeed * a.speedFactor) : 0;
            }
        }
    }

    measureCommute();
    PopTail tail;
    std::memset(&tail, 0, sizeof tail);
    tail.medianAll = commuteSecondsMedian_;
    tail.medianWalk = commuteSecondsWalk_;
    if (popCache_.hit && verify) {   // decided again: does the cache agree, field for field?
        int bad = 0;
        for (std::size_t i = 0; i < agents_.size(); ++i) {
            const PopRecord now = record(agents_[i]);
            if (std::memcmp(&cached[i], &now, sizeof(PopRecord)) != 0) ++bad;
        }
        if (std::memcmp(&cachedTail, &tail, sizeof tail) != 0) ++bad;
        popCache_.mismatches = bad;
        std::fprintf(stderr, "[citysim] population cache VERIFY: %d of %zu records differ\n", bad, agents_.size());
    } else if (popCache_.used) {
        std::error_code ec;
        std::filesystem::create_directories(populationCacheDir_, ec);
        const std::string tmp = popPath + ".tmp";
        std::ofstream out(tmp, std::ios::binary);
        const uint32_t fmt = kPopulationFormat, count = static_cast<uint32_t>(agents_.size());
        out.write("RTPOP1", 6);
        out.write(reinterpret_cast<const char*>(&fmt), 4);
        out.write(reinterpret_cast<const char*>(&popCache_.key), 8);
        out.write(reinterpret_cast<const char*>(&count), 4);
        for (const Agent& a : agents_) { const PopRecord r = record(a); out.write(reinterpret_cast<const char*>(&r), sizeof r); }
        out.write(reinterpret_cast<const char*>(&tail), sizeof tail);
        out.close();
        if (out) { std::filesystem::rename(tmp, popPath, ec); popCache_.saved = !ec; }
        if (popCache_.saved) engine::bundle::noteCacheUse(popPath, "population");
    }
    }   // decided (not read from the cache)
    students_ = 0;
    for (const Agent& a : agents_) students_ += isStudent(a) ? 1 : 0;
    tagSpots();   // the venues are known now: which spots are on the campus

    // THE HOUSEHOLDS' TIES (people.lua): housemates are partners, parent and child, siblings, roommates... as their
    // household type ties its member groups -- the strongest bond, so the work and building ties below never cover it
    for (std::size_t h = 0; h < housemates.size(); ++h) {
        const Household& hh = households_.households[h];
        if (hh.type >= people_.households.size()) continue;   // (a hall bed: on their own)
        for (const HouseholdTie& ht : people_.households[hh.type].ties) {
            const int back = people_.ties[static_cast<std::size_t>(ht.tie)].other;
            for (const auto& [i, gi] : housemates[h])
                for (const auto& [j, gj] : housemates[h])
                    if (i != j && gi == ht.from && gj == ht.to && (ht.from != ht.to || i < j))
                        relationships_.setTie(agents_[static_cast<std::size_t>(i)].uid, agents_[static_cast<std::size_t>(j)].uid,
                                              ht.tie, back);
        }
    }

    // Seed the surface-level social graph: agents sharing a workplace are
    // coworkers; those sharing a home building (another dwelling) are neighbors.
    //
    // Only agents who share a PLACE can be related, so bucket by place and pair
    // within a bucket instead of testing every pair in the city against every
    // other. The old double loop was ~11.5 M pair tests on piedmont — paid on
    // every single level load — to discover a comparatively tiny number of
    // bonds. Emission order is preserved exactly: for each agent in ascending
    // index, its partners in ascending index, with the same work-beats-home
    // precedence, so the resulting table is identical.
    {
        std::unordered_map<PlaceId, std::vector<int>> byWork, byHome;
        for (std::size_t i = 0; i < agents_.size(); ++i) {
            const Agent& A = agents_[i];
            if (A.workPlace != kNoPlace)
                byWork[A.workPlace].push_back(static_cast<int>(i));
            if (A.homePlace != kNoPlace)
                byHome[A.homePlace].push_back(static_cast<int>(i));
        }
        std::vector<int> partners;
        for (std::size_t i = 0; i < agents_.size(); ++i) {
            const Agent& A = agents_[i];
            partners.clear();
            // Buckets were filled in ascending index, so the tail past `i` is
            // already ascending; merging two such tails and sorting reproduces
            // the inner loop's visit order over exactly the pairs that matter.
            auto tailOf = [&](std::unordered_map<PlaceId, std::vector<int>>& m,
                              PlaceId key) {
                if (key == kNoPlace) return;
                auto it = m.find(key);
                if (it == m.end()) return;
                for (int j : it->second)
                    if (j > static_cast<int>(i)) partners.push_back(j);
            };
            tailOf(byWork, A.workPlace);
            tailOf(byHome, A.homePlace);
            std::sort(partners.begin(), partners.end());
            partners.erase(std::unique(partners.begin(), partners.end()),
                           partners.end());
            for (int j : partners) {
                const Agent& B = agents_[j];
                if (A.workPlace != kNoPlace && A.workPlace == B.workPlace)
                    relationships_.set(A.uid, B.uid, Relationship::Coworker);
                else if (A.homePlace != kNoPlace && A.homePlace == B.homePlace)
                    relationships_.set(A.uid, B.uid, Relationship::Neighbor);
            }
        }
    }
    assignSideJobs();     // the students' evening jobs (before the staffing: their shifts staff the place)
    indexNightVenues();   // where the sleepers go out to (eveningVenue)
    indexVenueStaffing();   // ...and only to places somebody is working at
}

// MEDIAN COMMUTE, in sim-seconds. Sampled (every 64th commuter) because each
// probe is a fresh uncached A*.
//
// Two things read it. The host multiplies it by hoursPerSecond to find what a
// commute COSTS in in-world hours — the number that decides whether an agent's
// day is livable at all, since a clock running 180x real time turns a 3 km
// drive into 12.5 in-world hours. And scheduleSnapshot uses it to decide how
// long after a departure boundary an agent is still on the road.
//
// Measured at build (where home/work are first assigned) and again after
// assignPlaces moves work to a real building, so it is never stale.
void CitySim::measureCommute() {
    // (On foot: at kWalkSpeed a kilometre-scale home/work pair is an hours-long journey in world time.)
    std::vector<Real> walkSamples;
    for (std::size_t i = 0; i < agents_.size(); i += 16) {
        const Agent& a = agents_[i];
        if (a.home == a.work) continue;
        const engine::roads::WalkRoute r = pointRoute(a.home, a.work);
        if (!r.found) continue;
        walkSamples.push_back(r.metres / std::max(Real(0.1), kWalkSpeed * a.speedFactor));
    }
    auto median = [](std::vector<Real>& v) -> Real {
        if (v.empty()) return 0;
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };
    commuteSecondsWalk_ = median(walkSamples);
    if (commuteSecondsWalk_ > 0) commuteSecondsMedian_ = commuteSecondsWalk_;   // (the figure scheduleSnapshot uses)
}

void CitySim::staffOnShift(std::vector<uint16_t>& out, int placeCount, std::vector<uint16_t>* hired,
                           const std::vector<uint8_t>* only, PlaceId watch, std::vector<int>* watchers) const {
    out.assign(static_cast<std::size_t>(std::max(0, placeCount)), 0);
    if (hired) hired->assign(out.size(), 0);
    if (watchers) watchers->clear();
    for (const Agent& a : agents_) {
        if (a.workPlace == kNoPlace || a.workPlace >= static_cast<PlaceId>(std::max(0, placeCount))) continue;
        if (watchers && a.workPlace == watch) watchers->push_back(static_cast<int>(&a - agents_.data()));
        if (only && (a.workPlace >= static_cast<PlaceId>(only->size()) || !(*only)[a.workPlace])) continue;
        if (hired && (*hired)[a.workPlace] < 0xFFFF) ++(*hired)[a.workPlace];
        if (atWorkNow(a)) {
            uint16_t& n = out[static_cast<std::size_t>(a.workPlace)];
            if (n < 0xFFFF) ++n;
        }
    }
}

void CitySim::indexVenueStaffing() {
    staffingRate_ = hoursPerSecond_;
    if (venues_.empty()) return;
    auto gated = [](PlaceType t) {
        return t == PlaceType::Shop || t == PlaceType::Cafe || t == PlaceType::Restaurant || t == PlaceType::Bar ||
               t == PlaceType::Club || t == PlaceType::Supermarket;
    };
    PlaceId maxPlace = 0;
    for (const Venue& v : venues_) if (v.place != kNoPlace) maxPlace = std::max(maxPlace, v.place);
    std::vector<uint32_t> mask(static_cast<std::size_t>(maxPlace) + 1, 0u);
    std::vector<uint8_t> hired(mask.size(), 0), isGated(mask.size(), 0);
    for (Venue& v : venues_) {
        v.staffed = 0xFFFFFFu;   // ungated while the staff's own schedules are read (an evening's venue is chosen by openAt)
        if (v.place != kNoPlace && gated(v.type)) isGated[v.place] = 1;
    }
    for (const Agent& a : agents_) {
        // an evening job's shift staffs its place for those hours (on a shift night; the mask is any night's)
        if (a.sideVenue >= 0) {
            const PlaceId sp = venues_[static_cast<std::size_t>(a.sideVenue)].place;
            if (sp != kNoPlace && sp <= maxPlace && isGated[sp]) {
                hired[sp] = 1;
                for (int h = 0; h < 24; ++h)
                    if (inWindow(h + 0.5, a.sideStart, a.sideEnd)) mask[sp] |= 1u << h;
            }
        }
        if (a.workPlace == kNoPlace || a.workPlace > maxPlace || !isGated[a.workPlace]) continue;
        hired[a.workPlace] = 1;
        for (int h = 0; h < 24; ++h)   // at work at h:30: from the hour it arrives to the hour it leaves (its shift)
            if (inWindow(h + 0.5, a.departWork, a.departHome)) mask[a.workPlace] |= 1u << h;
    }
    bool anyHired = false;
    for (uint8_t x : hired) anyHired = anyHired || x;
    for (Venue& v : venues_) {
        v.staffed = 0xFFFFFFu;
        // a place nobody was hired at keeps its posted hours (a lab town, a population capped below its jobs): only
        // a place WITH staff can be found with none of them on shift
        // ...and so does one whose staff the schedule never has at work (a commute longer than the shift, on the game
        // clock): shut for good would be the sim's arithmetic, not a shop
        if (!anyHired || !gated(v.type) || v.place == kNoPlace || !hired[v.place] || !mask[v.place]) continue;
        v.staffed = mask[v.place];
    }
}

CitySim::VenueStaffing CitySim::venueStaffing(Real clock) const {
    VenueStaffing s;
    const int hr = static_cast<int>(std::floor(std::fmod(clock + 24.0, 24.0))) % 24;
    for (const Venue& v : venues_) {
        if (v.staffed == 0xFFFFFFu) continue;
        ++s.gated;
        if (v.staffed == 0) ++s.neverStaffed;
        if (v.staffed >> hr & 1u) ++s.staffedNow;
    }
    return s;
}

void CitySim::indexNightVenues() {
    nightVenues_.clear();
    for (int vi = 0; vi < static_cast<int>(venues_.size()); ++vi) {
        const Venue& v = venues_[static_cast<std::size_t>(vi)];
        if (v.type != PlaceType::Restaurant && v.type != PlaceType::Bar && v.type != PlaceType::Club) continue;
        const int64_t cx = static_cast<int64_t>(std::floor(v.door.x / 250.0)), cz = static_cast<int64_t>(std::floor(v.door.y / 250.0));
        nightVenues_[(cx << 32) ^ (cz & 0xffffffff)].push_back(vi);
    }
}

int CitySim::eveningVenue(const Agent& a, Real eveningStart) const {
    Real ss = 0, se = 0;
    if (sideShiftTonight(a, ss, se)) return a.sideVenue;   // a shift night: at the evening job
    if (nightVenues_.empty() || walkPoints_.empty() || a.home < 0) return -1;
    constexpr Real kReach = 1200.0;
    const Vec2 p = a.homeDoor;
    const Real at = std::fmod(eveningStart + 0.3, Real(24));
    std::vector<std::pair<Real, int>> near;
    const int64_t cx = static_cast<int64_t>(std::floor(p.x / 250.0)), cz = static_cast<int64_t>(std::floor(p.y / 250.0));
    for (int64_t x = cx - 5; x <= cx + 5; ++x)
        for (int64_t z = cz - 5; z <= cz + 5; ++z) {
            auto it = nightVenues_.find((x << 32) ^ (z & 0xffffffff));
            if (it == nightVenues_.end()) continue;
            for (int vi : it->second) {
                const Venue& v = venues_[static_cast<std::size_t>(vi)];
                const Real d2 = (v.door - p).lengthSquared();
                if (d2 > kReach * kReach || v.node == a.home || !v.openAt(at)) continue;
                near.push_back({d2, vi});
            }
        }
    if (near.empty()) return -1;
    const std::size_t k = std::min<std::size_t>(6, near.size());
    std::partial_sort(near.begin(), near.begin() + static_cast<long>(k), near.end());
    const long night = static_cast<long>(std::floor((clockTotalHours_ - 12.0) / 24.0));
    uint32_t h = a.brain * 0x2545F491u ^ static_cast<uint32_t>(night) * 0x9E3779B9u;
    h ^= h >> 15; h *= 0x846ca68bU; h ^= h >> 16;
    return near[h % k].second;
}

Ride CitySim::rideOf(int i) const {
    if (i < 0 || i >= static_cast<int>(agents_.size())) return Ride::None;
    const Agent& a = agents_[static_cast<std::size_t>(i)];
    // a director's pace (Agent::directedPace, ADR-0441): what it rides on is the director's too
    if (a.directedPace > 0) return a.speed > 0 ? a.directedRide : Ride::None;
    if (!a.moving || a.tripActivity < 0 ||
        a.tripActivity >= static_cast<int>(catalog_.defs.size()))
        return Ride::None;
    return catalog_.defs[static_cast<std::size_t>(a.tripActivity)].ride;
}

Real CitySim::ridePace(const Agent& a) const {
    if (a.directedPace > 0) return a.directedPace;   // (ADR-0441)
    if (a.tripActivity < 0 || a.tripActivity >= static_cast<int>(catalog_.defs.size()))
        return 1.0;
    const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(a.tripActivity)];
    if (d.ride == Ride::None) return 1.0;
    return d.paceLo + (d.paceHi - d.paceLo) * static_cast<Real>((a.brain >> 11) & 0xFF) / 255.0;   // its own bits
}

bool CitySim::sideShiftTonight(const Agent& a, Real& start, Real& end) const {
    if (a.sideVenue < 0 || a.role >= roles_.roles.size()) return false;
    const long night = static_cast<long>(std::floor((clockTotalHours_ - 12.0) / 24.0));   // noon to noon
    uint32_t h = a.brain * 0x5BD1E995u ^ (static_cast<uint32_t>(night) * 0x27D4EB2Du);
    h ^= h >> 15; h *= 0x846ca68bU; h ^= h >> 16;
    if (static_cast<Real>(h & 0x3FF) >= roles_.roles[a.role].side.nights * 1024.0) return false;
    start = a.sideStart;
    end = a.sideEnd;
    return true;
}

void CitySim::assignSideJobs() {
    sideJobs_ = 0;
    for (Agent& a : agents_) a.sideVenue = -1;
    if (venues_.empty()) return;
    for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
        Agent& a = agents_[static_cast<std::size_t>(i)];
        if (a.role >= roles_.roles.size() || a.home < 0) continue;
        const SideJobSpec& sj = roles_.roles[a.role].side;
        if (sj.share <= 0) continue;
        uint32_t h = a.brain * 0x2C1B3C6Du ^ 0x51DEu;
        h ^= h >> 15; h *= 0x7feb352dU; h ^= h >> 16;
        if (static_cast<Real>(h & 0x3FF) >= sj.share * 1024.0) continue;
        const Real u1 = ((h >> 10) & 0xFF) / Real(255), u2 = ((h >> 18) & 0xFF) / Real(255);
        const Real start = sj.startLo + (sj.startHi - sj.startLo) * u1, end = sj.finishLo + (sj.finishHi - sj.finishLo) * u2;
        // one of the four nearest of its kinds, open at the shift's start and an hour before it ends (posted hours)
        std::vector<std::pair<Real, int>> near;
        for (int v = 0; v < static_cast<int>(venues_.size()); ++v) {
            const Venue& V = venues_[static_cast<std::size_t>(v)];
            const std::string vk = venueKind(V);
            if (std::find(sj.places.begin(), sj.places.end(), vk) == sj.places.end()) continue;
            const Real d2 = (V.door - a.homeDoor).lengthSquared();
            if (d2 > sj.reach * sj.reach || !V.openAt(start + 0.1) || !V.openAt(std::fmod(end - 1.0 + 24.0, 24.0))) continue;
            near.push_back({d2, v});
        }
        if (near.empty()) continue;
        const std::size_t k = std::min<std::size_t>(4, near.size());
        std::partial_sort(near.begin(), near.begin() + static_cast<long>(k), near.end());
        a.sideVenue = near[(h >> 26) % k].second;
        a.sideStart = static_cast<float>(start);
        a.sideEnd = static_cast<float>(end);
        ++sideJobs_;
    }
    if (sideJobs_ > 0) std::fprintf(stderr, "[citysim] evening jobs: %d\n", sideJobs_);
}

bool CitySim::eveningPlan(const Agent& a, Real& start, Real& end) const {
    if (sideShiftTonight(a, start, end)) return true;   // a shift night: the evening is the shift (roles.lua side_job)
    const int r = static_cast<int>(a.role);
    const Real share = r >= 0 && r < static_cast<int>(roles_.roles.size()) ? roles_.roles[static_cast<std::size_t>(r)].nightOutShare : 0;
    if (share <= 0) return false;
    const long night = static_cast<long>(std::floor((clockTotalHours_ - 12.0) / 24.0));   // noon to noon
    const uint32_t h = engine::lowbias32(a.brain ^ (static_cast<uint32_t>(night) * 0x9E3779B9u));
    if (static_cast<Real>(h & 0x3FF) >= share * 1024.0) return false;
    const Real u1 = ((h >> 10) & 0xFF) / Real(255), u2 = ((h >> 18) & 0xFF) / Real(255);
    start = 18.5 + 2.5 * u1;
    end = std::fmod(start + 2.0 + 3.5 * u2, Real(24));
    return true;
}

void CitySim::setStaticObstacles(std::vector<Vec2> poles) {
    staticObstacles_.clear();
    obstacleCells_.clear();
    obstacleMaxClearance_ = 0;
    addStaticObstacles(poles, kPoleClearance);
}

void CitySim::addStaticObstacles(const std::vector<Vec2>& at, Real clearance) {
    for (const Vec2& p : at) {
        const int k = static_cast<int>(staticObstacles_.size());
        staticObstacles_.push_back({p, clearance});
        const int64_t cx = static_cast<int64_t>(std::floor(p.x / 8.0)), cz = static_cast<int64_t>(std::floor(p.y / 8.0));
        obstacleCells_[(cx << 32) ^ (cz & 0xffffffff)].push_back(k);
    }
    obstacleMaxClearance_ = std::max(obstacleMaxClearance_, clearance);
}


Vec2 CitySim::pushPoseClearOfLanes(Vec2 p, Real margin) const {
    // the road the point fronts (RoadModel::frontageAt): when it stands within its carriageway (+ `margin`), out across
    // to just past that, on the point's own side
    if (!roads_ || !roads_->built) return p;
    const engine::RoadFrontage f = roads_->frontageAt(p);
    if (!f.valid()) return p;
    const Real need = f.section->carriagewayHalfWidth() + margin;
    const Real d = dot(p - f.on, f.towards);
    if (d >= need) return p;
    return p + f.towards * (need - d + 0.15);
}

Vec2 CitySim::idlePose(int node, uint32_t brain) const {
    if (walkPoints_.empty() || node < 0 || node >= walkPoints_.nodeCount()) return Vec2();
    // Per-agent setback ALONG the walk: several agents sharing a spot must not stack in one place. (A spot is on the
    // walk, off the carriageway already.)
    const Real back = 2.0 + parkSetbackSlot(brain) * 0.6;
    return walkPoints_.nodes[static_cast<std::size_t>(node)] + walkPoints_.along[static_cast<std::size_t>(node)] * back;
}

// --- the GOAL layer (ADR-0064) ----------------------------------------------
// An agent's day is a GoalTable (city_goals.h): states that each bind one small
// C++ action, transitions on the events emitted below. The built-in tables
// mirror the historical schedule/wander control flow bit-exactly — same rng
// draws in the same order, same tick the transitions fire; a scripting build
// may replace them at load (setGoalTables), and every per-tick transition still
// executes right here, in C++.

// WHERE A GoTo STATE GOES WHEN IT IS PICKED, not fixed (an outing, lunch, a campus break, an activity, the errand menu):
// chosen once per trip and kept in outingTo (a trip resumed after a bus leg keeps it), the pick's venue, seat or
// session held as the pickers hold them. -1: nothing to pick for this state (its target is fixed, or none was found).
// The one chooser: the sim's own trips (startGoalTrip) and the brain's day (CityBrain, ADR-0491).
int CitySim::choosePickedNode(Agent& a, int origin, const GoalState& s) {
    const bool errandMenu = s.target == GoalTarget::Shop && catalog_.menu("errand");
    if (!(s.target == GoalTarget::Outing || s.target == GoalTarget::Lunch || s.target == GoalTarget::Campus ||
          s.target == GoalTarget::Activity || errandMenu)) {
        a.tripVenue = -1;
        a.outingTo = -1;
        return -1;
    }
    if (a.outingTo >= 0) return a.outingTo;
    const int chosen = s.target == GoalTarget::Outing ? pickOuting(a, origin)
                       : errandMenu ? pickFromMenu(a, origin, *catalog_.menu("errand"))
                       : s.target == GoalTarget::Lunch ? pickLunch(a, origin)
                       : s.target == GoalTarget::Campus ? pickCampusBreak(a, origin)
                       : !s.menu.empty() && catalog_.menu(s.menu) ? pickFromMenu(a, origin, *catalog_.menu(s.menu))
                                                                  : pickActivity(a, origin, s);
    a.outingTo = chosen;
    return chosen;
}

// step()'s pass 1 for one agent: run its current goal state. A GoTo state whose
// trip isn't running retries the departure each tick; a Rest state emits this
// tick's events in a fixed order — DwellDone, the clock-window depart, Idle —
// and the first row that fires (or blocks on launch clearance) wins.
// A RESTING AGENT'S EVENTS THIS TICK, in the order they are tried (the first with a row in its table wins). The one
// reading of the day's clock: the sim's own day (goalThink) and the brain's (CityBrain, ADR-0491).
//
// The commute clock: inside the [departWork, departHome) window the day says "be at work", outside it "be at home".
// Only meaningful for agents with a real commute -- a stranded pair (work == home, no route at build) never departs.
// Checked BEFORE the dwell: a state that has both (an outing's pause, the morning at work before lunch) ends the day
// when the window closes instead of starting another stop -- else a stroller whose pauses kept finishing never went
// home. THE NIGHT OUT: also before the dwell, so a bar-hop ends when the evening does. Then the stop's own length
// when it set one (a coffee, a browse), else the state's; then Idle.
bool CitySim::inWorkWindow(const Agent& a) const { return inWindow(clockHours_, departWorkHour(a), a.departHome); }

int CitySim::restEvents(const Agent& a, const GoalState& s, GoalEvent (&out)[kRestEvents]) const {
    int n = 0;
    if (a.home != a.work)
        out[n++] = inWorkWindow(a) ? GoalEvent::DepartWork : GoalEvent::DepartHome;
    Real eveStart = 0, eveEnd = 0;
    const bool outTonight = eveningPlan(a, eveStart, eveEnd);
    out[n++] = outTonight && inWindow(clockHours_, eveStart, eveEnd) ? GoalEvent::Evening : GoalEvent::EveningOver;
    const Real dwell = a.restDwell > 0 ? a.restDwell : s.dwellHours;
    if (dwell > 0 && a.goalHours >= dwell) out[n++] = GoalEvent::DwellDone;
    out[n++] = GoalEvent::Idle;
    return n;
}

// HOW LONG UNTIL A RESTING AGENT HAS SOMETHING TO DECIDE (hours; kNeverWakes: never, unless the world tells it): the
// end of its stop's dwell, the next flip of its work window, tonight's start or end if it goes out, and noon (the next
// night's plan). The one reckoning of when a rest ends: the sim's own sleepers (goalThink) and the brain's (ADR-0496).
// (The work window opens at departWorkHour -- the arrival less the trip -- and closes at departHome; before ADR-0496 a
// sleeper at home woke at departWork, the arrival itself, and so left late by its whole trip.)
Real CitySim::hoursUntilRestEvent(const Agent& a, const GoalState& s) const {
    const Real dwell = a.restDwell > 0 ? a.restDwell : s.dwellHours;
    Real hoursUntil = kNeverWakes;
    if (dwell > 0) hoursUntil = std::max(Real(0), dwell - a.goalHours);
    auto ahead = [&](Real h) { Real d = std::fmod(h - clockHours_, Real(24)); if (d < 0) d += 24.0; return d; };
    if (a.home != a.work) {
        // the commute fires when the window predicate FLIPS: the next boundary is whichever end of it is ahead
        const Real opens = departWorkHour(a);
        hoursUntil = std::min(hoursUntil, ahead(inWindow(clockHours_, opens, a.departHome) ? a.departHome : opens));
    }
    Real eveStart = 0, eveEnd = 0;
    if (eveningPlan(a, eveStart, eveEnd)) hoursUntil = std::min(hoursUntil, ahead(inWindow(clockHours_, eveStart, eveEnd) ? eveEnd : eveStart));
    const Real noon = ahead(12.0);
    if (noon > 1e-6) hoursUntil = std::min(hoursUntil, noon);
    return hoursUntil;
}

void CitySim::rebuildRoleTables() {
    roleTables_.clear();
    for (const ResidentRole& r : roles_.roles) roleTables_.push_back(buildDayTable(r.day));
    catchAllRole_ = std::max(0, roles_.catchAllJobRole());
}

void CitySim::setRoleCatalog(RoleCatalog c) {
    roles_ = std::move(c);
    rebuildRoleTables();
}

bool CitySim::setRoleTable(const std::string& role, GoalTable t) {
    const int id = roles_.find(role);
    if (id < 0 || id >= static_cast<int>(roleTables_.size())) return false;
    roleTables_[static_cast<std::size_t>(id)] = std::move(t);
    return true;
}

// TRAINS (city_rail_riders.h) ------------------------------------------------------------------------------------
void CitySim::setRail(const std::vector<engine::RailTrack>& tracks) {
    trainRiders_.clear();
    rail_ = RailSim();
    if (!tracks.empty()) rail_.buildService(tracks);
    if (!rail_.lines().empty() && !walkPoints_.empty()) trainRiders_.build(&rail_, walkPoints_);
    rebuildTransit();
}

void CitySim::rebuildTransit() {
    transit_ = buses_.timetable();     // (the bus lines first: a line's index is its bus line's)
    trainRiders_.addTimetable(transit_);
}

bool CitySim::pedVisible(int i) const {
    if (i < 0 || i >= static_cast<int>(agents_.size())) return false;
    const Agent& a = agents_[static_cast<std::size_t>(i)];
    if (a.inCar >= 0) return false;   // drawn in a household's car, not on the pavement (ADR-0444)
    return a.moving || !a.indoors;
}

// A place to STAND near `want` that nobody else is standing in: `want` itself,
// else steps along the sidewalk (`along`) either way, then a second row a step
// back from the kerb. People waiting at a stop or pausing on a walk otherwise
// all end on the one sidewalk point their street leads to (measured on metro:
// 427 overlapping pairs among ~190 standing figures near the player).
engine::Vec2 CitySim::freeStandingSpot(const Agent& a, engine::Vec2 want,
                                       engine::Vec2 along) const {
    const int self = indexOf(a);
    const Real al = along.length();
    const Vec2 u = al > 1e-6 ? along * (1.0 / al) : Vec2(1, 0);
    const Vec2 side(u.y, -u.x);   // rightOf(u): the walker's own left, away from the kerb (walkers keep left too)
    std::vector<int> near;
    // the nearest body to p, squared (capped at the 1.5 m query)
    auto nearest2 = [&](Vec2 p) {
        grid_.query(p, 1.5, near);
        Real best = 1.5 * 1.5;
        for (int bi : near) {
            if (bi == self) continue;
            const Agent& b = agents_[static_cast<std::size_t>(bi)];
            if (!b.moving && b.indoors) continue;   // inside: not on the pavement
            best = std::min(best, (b.pos - p).lengthSquared());
        }
        return best;
    };
    constexpr Real kStep = 0.8;
    // A CROWD (a busy stop) can fill every spot tried; the fallback is then the roomiest of them, not `want`
    // itself -- which stood the newcomer inside whoever already stood there (two STILL people are never pushed
    // apart: macOS CI's one overlapping pair in metro_pedestrians_walk_and_keep_apart). Three rows now.
    Vec2 roomiest = want;
    Real roomiest2 = -1;
    for (int row = 0; row < 3; ++row) {
        const Vec2 base = want + side * (row * kStep);
        for (int k = 0; k < 9; ++k) {
            const Real off = kStep * static_cast<Real>((k + 1) / 2) * ((k & 1) ? 1.0 : -1.0);
            const Vec2 p = base + u * off;
            const Real d2 = nearest2(p);
            if (d2 >= 0.75 * 0.75) return p;
            if (d2 > roomiest2) { roomiest2 = d2; roomiest = p; }
        }
    }
    return roomiest;
}

// Somewhere NEAR to go next on an outing: an open park, cafe, store,
// supermarket, restaurant or civic building within a short walk -- or just a
// walk round the block (a random street corner 150-450 m off). Weighted so a
// park or a coffee is likelier than the town hall; never straight back to
// the stop just left. Sets a.tripVenue (-1 for the plain walk).
int CitySim::pickOuting(Agent& a, int origin) {
    // the role's own day out (roles.lua `outings`: a child's kid_outing), else the outing menu
    const Menu* m = a.role < roles_.roles.size() ? catalog_.menu(roles_.roles[a.role].day.outingMenu) : nullptr;
    if (!m) m = catalog_.menu("outing");
    return m ? pickFromMenu(a, origin, *m) : -1;
}

// ---- THE CATALOG'S CHOOSER (activities framework, step A) --------------------------------------------------------

std::string CitySim::venueKind(const Venue& v) const {
    return v.kind >= 0 ? placeKinds_.nameOf(v.kind) : placeTypeName(v.type);
}

static const char* spotSiteKind(SpotKind k) {
    switch (k) {
        case SpotKind::Sit: return "seat";
        case SpotKind::Lie: return "bed";
        case SpotKind::Stand: return "stand";
        case SpotKind::Jog: return "loop";
        case SpotKind::Play: return "pitch";
        case SpotKind::Watch: return "watch";
        default: return "?";
    }
}

std::string CitySim::siteKindOf(const Agent& a) const {
    if (a.tripSeat >= 0 && a.tripSeat < static_cast<int>(seats_.size())) return spotSiteKind(seats_[static_cast<std::size_t>(a.tripSeat)].kind);
    if (a.tripVenue >= 0 && a.tripVenue < static_cast<int>(venues_.size())) return venueKind(venues_[static_cast<std::size_t>(a.tripVenue)]);
    if (a.tripActivity >= 0 && a.tripActivity < static_cast<int>(catalog_.defs.size())) {
        const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(a.tripActivity)];
        if (!d.sites.empty()) return d.sites.front();
    }
    return "";
}

// One activity's site for this agent from `origin`: a spot (reserved), a place, or a street corner. countOnly: how
// many candidate sites there are (a per-site menu weight), nothing chosen.
int CitySim::tryActivity(Agent& a, int origin, int di, int prevVenue, bool countOnly, int* count) {
    if (count) *count = 0;
    if (di < 0 || di >= static_cast<int>(catalog_.defs.size()) || walkPoints_.empty()) return -1;
    const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(di)];
    const Real h = clockHours_;
    if (!hourIn(h, d.hourLo, d.hourHi)) return -1;
    if (d.bringOwnEighths > 0 && static_cast<int>((a.brain >> 13) & 7u) < d.bringOwnEighths) return -1;
    if (d.inShift && !inWindow(clockHours_, departWorkHour(a), a.departHome)) return -1;
    const int fromNode = d.fromHome && a.home >= 0 && a.home < walkPoints_.nodeCount() ? a.home : origin;
    const Vec2 here = walkPoints_.nodes[static_cast<std::size_t>(fromNode)];
    // WATCHING: only while that activity is running within reach
    if (!d.during.empty()) {
        const int dd = catalog_.find(d.during);
        bool on = false;
        for (const Session& se : sessions_)
            if (se.def == dd && se.state == Session::State::Running &&
                (areas_[static_cast<std::size_t>(se.area)].center - here).lengthSquared() <= d.distHi * d.distHi)
                on = true;
        if (!on) return -1;
    }
    // AREAS (a group activity: a session on a pitch, a lawn)
    for (const std::string& k : d.sites)
        if (isAreaSiteKind(k)) {
            if (countOnly) {
                for (const ActivityArea& ar : areas_)
                    if (ar.kind == k && (ar.tags & d.tags) == d.tags &&
                        (ar.center - here).lengthSquared() <= d.distHi * d.distHi) { if (count) *count += 1; }
                return -1;
            }
            const int ai = joinSession(a, di, here, d.distLo, d.distHi, d.nearest);
            if (ai < 0) return -1;
            a.tripActivity = di;
            return areas_[static_cast<std::size_t>(ai)].node;
        }
    // SPOTS
    uint32_t kinds = 0;
    bool venuesToo = false, street = false;
    for (const std::string& k : d.sites) {
        bool found = false;
        for (int sk = 0; sk < static_cast<int>(SpotKind::Count); ++sk)
            if (k == spotSiteKind(static_cast<SpotKind>(sk))) { kinds |= spotKindBit(static_cast<SpotKind>(sk)); found = true; }
        if (k == "street") { street = true; found = true; }
        if (!found) venuesToo = true;
    }
    if (kinds) {
        ActivityQuery q;
        q.kinds = kinds; q.tags = d.tags; q.minDist = d.distLo; q.maxDist = d.distHi;
        q.nearest = d.nearest > 0 ? d.nearest : 1 << 30;
        if (countOnly) {
            int n = 0;
            for (const ActivitySpot& sp : seats_)
                if (sp.occupant < 0 && (q.kinds & spotKindBit(sp.kind)) && (sp.tags & q.tags) == q.tags) {
                    const Real d2 = (sp.pos - here).lengthSquared();
                    if (d2 >= q.minDist * q.minDist && d2 <= q.maxDist * q.maxDist) ++n;
                }
            if (count) *count += n;
        } else {
            const int sp = pickSpot(a, here, q);
            if (sp >= 0) {
                a.tripSeat = sp;
                a.tripActivity = di;
                return seats_[static_cast<std::size_t>(sp)].node;
            }
        }
    }
    // PLACES
    if (venuesToo) {
        std::vector<std::pair<Real, int>> cand;
        // `side_job`: the agent's own evening job, on a shift night only (roles.lua side_job), open or not -- they open it
        Real ss = 0, se = 0;
        bool sideSite = false;
        for (const std::string& k : d.sites) sideSite = sideSite || k == "side_job";
        const int sideTonight = sideSite && sideShiftTonight(a, ss, se) && inWindow(h, ss - 1.0, se) ? a.sideVenue : -1;
        for (int i = 0; i < static_cast<int>(venues_.size()); ++i) {
            const Venue& v = venues_[static_cast<std::size_t>(i)];
            if (i == prevVenue || v.node == origin || (i != sideTonight && !v.openAt(h))) continue;
            const std::string vk = venueKind(v);
            bool want = i == sideTonight;
            for (const std::string& k : d.sites) want = want || k == vk;
            if (!want) continue;
            const Real d2 = (walkPoints_.nodes[static_cast<std::size_t>(v.node)] - here).lengthSquared();
            if (d2 < d.distLo * d.distLo || d2 > d.distHi * d.distHi) continue;
            cand.push_back({d2, i});
        }
        if (countOnly) { if (count) *count += static_cast<int>(cand.size()); }
        else if (!cand.empty()) {
            int pick;
            if (d.nearest > 0) {
                const std::size_t k = std::min<std::size_t>(cand.size(), static_cast<std::size_t>(d.nearest));
                std::partial_sort(cand.begin(), cand.begin() + static_cast<std::ptrdiff_t>(k), cand.end());
                pick = cand[tripRnd(a) % k].second;
            } else {
                pick = cand[rnd() % static_cast<uint32_t>(cand.size())].second;
            }
            a.tripVenue = pick;
            a.tripActivity = di;
            return venues_[static_cast<std::size_t>(pick)].node;
        }
    }
    // A STREET CORNER: somewhere in the distance band a walker leaves by (a walk round the block)
    if (street) {
        if (countOnly) { if (count) *count += 1; return -1; }
        const int n = walkPoints_.streetNodeCount();
        for (int tries = 0; tries < 48; ++tries) {
            const int cand = static_cast<int>(rnd() % static_cast<uint32_t>(n));
            const Real d2 = (walkPoints_.nodes[static_cast<std::size_t>(cand)] - here).lengthSquared();
            if (d2 < d.distLo * d.distLo || d2 > d.distHi * d.distHi) continue;
            a.tripActivity = di;   // (every spot is on a walk)
            return cand;
        }
    }
    return -1;
}

int CitySim::pickFromMenu(Agent& a, int origin, const Menu& m) {
    const int prev = a.tripVenue;
    a.tripVenue = -1;
    a.tripActivity = -1;
    releaseSeat(a);
    if (walkPoints_.empty() || origin < 0 || origin >= walkPoints_.nodeCount()) return -1;
    const MenuBand* band = nullptr;
    for (const MenuBand& b : m.bands)
        if (hourIn(clockHours_, b.hourLo, b.hourHi)) { band = &b; break; }
    if (!band) return -1;
    // FIRST: tried in order, each with its chance
    for (const MenuEntry& e : band->first) {
        if (static_cast<Real>(rnd() % 1000u) >= e.chance * 1000.0) continue;
        const int node = tryActivity(a, origin, catalog_.find(e.activity), prev, false, nullptr);
        if (node >= 0) return node;
    }
    // THEN a weighted pick: a per-site activity weighs as much again for every site it could go to
    std::vector<std::pair<Real, int>> cum;   // cumulative weight -> entry
    Real total = 0;
    for (std::size_t k = 0; k < band->pick.size(); ++k) {
        const MenuEntry& e = band->pick[k];
        const int di = catalog_.find(e.activity);
        if (di < 0) continue;
        int n = 0;
        tryActivity(a, origin, di, prev, true, &n);
        if (n <= 0) continue;
        const Real w = e.weight * (catalog_.defs[static_cast<std::size_t>(di)].perSite ? n : 1);
        if (w <= 0) continue;
        total += w;
        cum.push_back({total, static_cast<int>(k)});
    }
    if (cum.empty()) return -1;
    const Real roll = static_cast<Real>(rnd() % 100000u) / 100000.0 * total;
    std::size_t at = 0;
    while (at + 1 < cum.size() && roll > cum[at].first) ++at;
    // the chosen one, then (if its site has gone) the others in order
    for (std::size_t t = 0; t < cum.size(); ++t) {
        const MenuEntry& e = band->pick[static_cast<std::size_t>(cum[(at + t) % cum.size()].second)];
        const int node = tryActivity(a, origin, catalog_.find(e.activity), prev, false, nullptr);
        if (node >= 0) return node;
    }
    return -1;
}

// A STUDENT'S BREAK between classes (campus milestone 4): a session in the
// library, a sit on a bench on the quad or a bleacher by the pitch, a walk
// across the quad -- and at midday, lunch out like anyone. Sets tripVenue or
// tripSeat; -1 = no stop (the table's NoRoute row: straight back to class).
int CitySim::pickCampusBreak(Agent& a, int origin) {
    const Menu* m = catalog_.menu("student_break");
    return m ? pickFromMenu(a, origin, *m) : -1;
}

int CitySim::pickCampusSeat(Agent& a, Vec2 here) {
    // a free seat on the campus (within 120 m of the quad or the field: tagSpots), one of the nearest six to here
    ActivityQuery q;
    q.kinds = spotKindBit(SpotKind::Sit);
    q.tags = spot_tag::kCampus;
    q.maxDist = 600.0; q.nearest = 6;
    return pickSpot(a, here, q);
}

void CitySim::assignStudents(const PlaceMap& places) {
    const auto spotOf = [&](PlaceId id) {
        return id < placePoint_.size() ? placePoint_[static_cast<std::size_t>(id)] : walkPoints_.nearestNode(places[id].entrance);
    };
    // Every hall of each kind: the quad block's residence hall and the dorm block's, both teaching halls.
    std::vector<PlaceId> halls, teach;
    PlaceId lib = kNoPlace;
    const uint32_t kBeds = placeKinds_.tag("beds"), kClasses = placeKinds_.tag("classes"), kLibrary = placeKinds_.tag("library");
    for (const Place& p : places.places()) {
        if (p.tags & kBeds) halls.push_back(p.id);
        if (p.tags & kClasses) teach.push_back(p.id);
        if ((p.tags & kLibrary) && lib == kNoPlace) lib = p.id;
    }
    if (halls.empty() || teach.empty()) return;
    auto doorOf = [&](PlaceId id) { return placeDoor(places[id]); };
    // the halls a student can live in: routable to every teaching hall and back on foot
    struct Hall { PlaceId id; int node; };
    std::vector<Hall> live;
    std::vector<std::pair<PlaceId, int>> classes;
    for (PlaceId t : teach) {
        const int tn = spotOf(t);
        if (tn >= 0) classes.push_back({t, tn});
    }
    if (classes.empty()) return;
    for (PlaceId h : halls) {
        const int hn = spotOf(h);
        if (hn < 0) continue;
        bool ok = true;
        for (const auto& c : classes) ok = ok && c.second != hn && walkPoints_.samePart(hn, c.second);   // (walked there and back)
        if (ok) live.push_back({h, hn});
    }
    if (live.empty()) return;
    const int ln = lib != kNoPlace ? spotOf(lib) : -1;

    const int hallRole = people_.hallStage >= 0 ? roles_.find(people_.stages[static_cast<std::size_t>(people_.hallStage)].role) : -1;
    const int studentRole = hallRole >= 0 ? hallRole : catchAllRole_;
    // WHO: the households' students (households.h -- the halls' beds are filled first, with walkers, never more than
    // the people.lua share of them), each in the hall the plan gave them, when it is one a student can live in.
    for (Agent& a : agents_) {
        if (a.household < 0) continue;
        const Household& hh = households_.households[static_cast<std::size_t>(a.household)];
        if (hh.type != Household::kHall) continue;
        const Hall* hall = nullptr;
        for (const Hall& h : live)
            if (h.id == hh.place) hall = &h;
        if (!hall) continue;
        const Hall& H = *hall;
        a.role = static_cast<RoleId>(studentRole);
        a.homePlace = H.id; a.home = H.node; a.homeDoor = doorOf(H.id);
        a.restNode = H.node; a.pos = a.homeDoor; a.indoors = true;
        a.heading = walkPoints_.along[static_cast<std::size_t>(H.node)];
        const auto& cl = classes[(a.brain >> 9) % classes.size()];   // which hall their course is taught in
        a.workPlace = cl.first; a.work = cl.second; a.workDoor = doorOf(cl.first);
        a.shopPlace = lib; a.shop = ln; a.shopDoor = lib != kNoPlace ? doorOf(lib) : a.workDoor;
        // The first class at 8:30-10, the last out at 16:00-18:30.
        const Real u0 = static_cast<Real>((a.brain >> 5) & 0xFF) / 255.0, u1 = static_cast<Real>((a.brain >> 17) & 0xFF) / 255.0;
        a.departWork = 8.5 + 1.5 * u0;
        a.departHome = 16.0 + 2.5 * u1;
        const engine::roads::WalkRoute there = pointRoute(H.node, cl.second);
        a.commuteSeconds = (there.found ? there.metres : 0) / std::max(Real(0.1), kWalkSpeed * a.speedFactor);
    }
}

// Lunch OUT: a walker at work, in its shift, goes to one of the few nearest
// open cafes or restaurants. About three in eight brought lunch and stay in;
// a car commuter stays in too (its car is parked at work -- a lunch trip
// would drive it round the block). -1 = no trip (the table's NoRoute row).
int CitySim::pickLunch(Agent& a, int origin) {
    const Menu* m = catalog_.menu("lunch");
    return m ? pickFromMenu(a, origin, *m) : -1;
}

// WHERE A WALKER ON ITS ROUTE STANDS (W3): on its step's walk at the metres walked (LaneGraph::pointAt), at the walk's
// own height (heightAt), facing the way it walks; on a street's walk, in its own file across the walk -- its distance in
// from the kerb as before, walkerVerge, measured from the walk's middle (the sidewalk's), eased in over the first and last
// metres of the step so the file is taken up and given back without a jump; between two steps, across the short gap
// from one walk's end to the next walk's start. (The old step's links, its corner curves and its stop lines are gone.)
void CitySim::refreshPose(Agent& a) const {
    if (!onRoute(a) || !roads_) return;
    constexpr Real kFileEase = 4.0;   // m: over this much of a step's start and end a walker moves into its file (a
                                      // metre across at most: a quarter of a metre sideways for each metre on)
    const engine::LaneGraph& G = roads_->laneLevel;
    const engine::roads::WalkStep& s = a.route.steps[static_cast<std::size_t>(a.leg)];
    const Real along = std::fabs(s.to - s.from);
    const Real d = std::clamp(a.distOnLeg, Real(0), legLength(a, a.leg));
    Vec2 dir = a.heading.length() > 0.5 ? a.heading : Vec2(1, 0);
    Vec2 p = routePoint(a, a.leg, d, &dir);
    int8_t side = 0;
    Real height = 0, kerbRoom = 0;
    if (d <= along) {
        const Real at = s.from + (s.forward ? 1.0 : -1.0) * d;
        height = G.heightAt(s.walk, at);
        const int8_t ws = s.walk.index < walkRoadSide_.size() ? walkRoadSide_[s.walk.index] : int8_t(0);
        side = static_cast<int8_t>(s.forward ? ws : -ws);
        if (side != 0) {
            // its line's distance from its kerb, and on a street's stretch the walker's own file across it (a corner walk
            // is walked on its line)
            const Real half = walkKerbOff_[s.walk.index];
            const Real room = std::max(Real(0), half - kWalkerHalfAcross);
            const bool stretch = G.walks[s.walk.index].street != engine::kNoIndex;
            const Real file = stretch ? std::clamp(walkerVerge(a) - half, -room, room) : Real(0);   // + away from the road
            const Real ease = std::clamp(std::min(d, along - d) / kFileEase, Real(0), Real(1));
            const Vec2 right(dir.y, -dir.x);
            const Vec2 away = side > 0 ? right * -1.0 : right;
            p = p + away * (file * ease);
            // ...and how far it may lean toward the road: to the walk's kerb edge, less half a body
            kerbRoom = std::max(Real(0), half + file * ease - kWalkerHalfAcross);
        }
    } else if (a.leg + 1 < static_cast<int>(a.route.steps.size())) {
        const engine::roads::WalkStep& n = a.route.steps[static_cast<std::size_t>(a.leg + 1)];
        const Real gap = std::max(Real(1e-6), legLength(a, a.leg) - along);
        const Real u = std::clamp((d - along) / gap, Real(0), Real(1));
        height = G.heightAt(s.walk, s.to) * (1.0 - u) + G.heightAt(n.walk, n.from) * u;
    }
    a.pos = p;
    a.heading = dir;
    a.roadSide = side;
    a.kerbRoom = static_cast<float>(kerbRoom);
    // the height: the walk's own (absolute: the drawing stands the body on the surface nearest it, a bridge's sidewalk
    // or the ground: city_poses deckSurfaceNear); above-the-ground heights the walks do not state
    a.deckY = static_cast<float>(height);
    a.elevation = 0;
    a.grade = 0;
}

// THE NEXT CROSSING ON THE ROUTE (W3): read off the route (WalkRoute::crossings, the steps it crosses a crosswalk on),
// the first not yet crossed within kCrossingLookAhead route metres. Before it: how far to where the walker waits, a
// step back from the crossing step's start (the kerb end it steps off from); on it: toHold negative.
CitySim::CrossingAhead CitySim::nextCrossing(const Agent& a) const {
    CrossingAhead out;
    if (!onRoute(a) || !roads_) return out;
    for (const engine::roads::WalkCrossing& c : a.route.crossings) {
        const int step = static_cast<int>(c.step);
        if (step < a.leg) continue;
        const engine::roads::WalkStep& s = a.route.steps[static_cast<std::size_t>(step)];
        const Real across = std::fabs(s.to - s.from);
        Real toStart = -a.distOnLeg;
        for (int k = a.leg; k < step; ++k) toStart += legLength(a, k);
        if (toStart > kCrossingLookAhead) break;
        if (step == a.leg && a.distOnLeg >= across) continue;   // across it, on the gap after it
        out.found = true;
        out.crosswalk = c.crosswalk;
        out.fromEnd = c.fromEnd;
        out.toHold = step == a.leg ? -(a.distOnLeg + 1e-3) : std::max(Real(0), toStart - kCrossingKerbStep);
        out.toFar = toStart + across;
        return out;
    }
    return out;
}

// advance()'s crossing rule (ADR-0452, from the route since W3): before the kerb end it steps off from, a person goes
// on WALK where lights run it (a rare jaywalker on any phase, given a jaywalker's gap), elsewhere when the gap they
// accept is there (where the level does not drive there are no cars, and the gap always is); else they wait a step
// back from that kerb end. On the crossing they carry on. Pure query: no rng draws, no agent mutation.
CitySim::CrossingGate CitySim::crossingGate(const Agent& a, Real target) const {
    CrossingGate gate;
    gate.cap = target;
    gate.crossing = nextCrossing(a);
    if (!gate.crossing.found || gate.crossing.toHold < 0) return gate;
    const engine::ModelCrosswalk* c = roads_->crosswalk(gate.crossing.crosswalk);
    const driving::WalkSignal ws = drivingV2_ && drivingV2_->lightsRun()
                                       ? driving::walkAtCrosswalk(drivingV2_->rightOfWay(), gate.crossing.crosswalk,
                                                                  drivingV2_->clockSeconds())
                                       : driving::WalkSignal::None;
    bool go = false;
    if (c && c->control == engine::CrosswalkControl::Lights && ws != driving::WalkSignal::None) {
        gate.kind = CrossingKind::Lights;
        go = ws == driving::WalkSignal::Walk;
        if (!go && jaywalks(a) && gapToCross(a, gate.crossing.crosswalk, a.crossWaited, true)) {
            go = true;
            gate.kind = CrossingKind::AgainstTheLights;
        }
    } else {
        gate.kind = !c ? CrossingKind::Unmarked
                    : c->control == engine::CrosswalkControl::Signs ? CrossingKind::Signs
                    : c->marked ? CrossingKind::Marked : CrossingKind::Unmarked;
        go = gapToCross(a, gate.crossing.crosswalk, a.crossWaited, false);
    }
    if (!go) {
        gate.walkHold = true;
        gate.cap = std::min(target, approachStop(gate.crossing.toHold, kPedDecel, target));
    }
    return gate;
}

void CitySim::advance(Agent& a, Real dt, Real gap, Real minGap) {
    if (!a.moving) return;
    if (!onRoute(a)) {   // (no steps left: arrived)
        arrive(a);
        return;
    }
    // Nominal pace scaled by personality (ADR-0062): each walker holds its OWN pace, so the crowd doesn't move in
    // lockstep. The crossing's hold is a shared rule and stays unscaled.
    Real target = kWalkSpeed * ridePace(a) * a.speedFactor;   // (a walker on a board rides at its pace: activities.lua `ride`)
    const Real accel = kPedAccel;
    CrossingGate gate = crossingGate(a, target);
    target = gate.cap;
    // WHERE THIS WALKER STANDS AGAINST A CROSSWALK (ADR-0452), for the drivers (thingsForDriving) and the counts
    {
        const CrossingAhead& x = gate.crossing;
        const bool atKerb = x.found && x.toHold >= 0 && x.toHold < kCrossingAtKerb && gate.walkHold;
        const bool on = x.found && x.toHold < 0 && x.toFar > 0;
        if (on && !(a.onCrosswalk && a.crosswalkAt == static_cast<int>(x.crosswalk)))
            ++walkStats_.crossings[std::min<std::size_t>(a.crossingKind, kCrossingKindCount - 1)];
        if (x.found && x.toHold >= 0) a.crossingKind = static_cast<uint8_t>(gate.kind);
        a.crosswalkAt = atKerb || on ? static_cast<int>(x.crosswalk) : -1;
        a.crossFromEnd = static_cast<int8_t>(x.fromEnd);
        a.onCrosswalk = on;
        a.aboutToCross = atKerb && gate.kind != CrossingKind::Lights;
        if (atKerb) {
            a.crossWaited += dt;
            ++walkStats_.waitingTicks;
        } else {
            a.crossWaited = 0;
        }
    }
    // Following: a walker eases in behind whoever is ahead on its file (computeGaps; the fixed footprint gap).
    {
        Real slowZone = kPedSlowZone;
        target = followCap(target, gap, minGap, slowZone);
        a.speed = std::min(target, a.speed + accel * dt);
        if (a.speed < 0.3 && gap < 1e8) holdWhy(a, 4);
    }

    // Hard hold for a walker held at a crosswalk's kerb end (ADR-0452's walkHold): the smooth cap above slows it but
    // never to exactly zero, so without this the leftover motion would carry it THROUGH the kerb end.
    Real motion = a.speed * dt;
    if (gate.walkHold) {
        const Real room = std::max(Real(0), gate.crossing.toHold);
        if (motion >= room) { motion = room; a.speed = 0; holdWhy(a, 2); }   // held at the kerb end
        if (room < kCrossingAtKerb) a.state = Agent::State::Waiting;        // a pedestrian held at the kerb is Waiting
    }
    // ON ALONG THE ROUTE: its steps and the gaps between them (glideAlong), to its end, where it arrives
    if (glideAlong(a, motion)) {
        const int n = static_cast<int>(a.route.steps.size());   // at its route's end first: arriving is where it walked to
        a.leg = n - 1;
        a.distOnLeg = legLength(a, a.leg);
        refreshPose(a);
        a.leg = n;
        arrive(a);
    } else {
        refreshPose(a);
    }
}

// The arrival branch of advance(), table-driven (ADR-0064): the Arrived event
// asks the agent's GoalTable what comes next. A DRIVER whose next state is
// another trip (a GoTo state — wander's Roam self-loop) CHAINS straight into
// it; otherwise the agent rests — a driver parks on the verge, a walker rests
// at its sidewalk end — wearing the new state's activity label.
// `vArrive` is the speed still carried at the arrival tick — a chain keeps it.
// ARRIVED: the walk's end, where the stepper last placed it. Whoever sent it there -- the brain, or a director --
// says what happens next; the sim only stops it (ADR-0499: no day of its own chains the next trip).
void CitySim::arrive(Agent& a) {
    a.moving = false;
    a.speed = 0;
    a.restNode = a.tripGoal >= 0 ? a.tripGoal : walkPoints_.nearestNode(a.pos);
    a.elevation = 0;
    a.state = Agent::State::Resting;
}


// A trip decision's draw between lo and hi (hours), from the agent's own stream.
Real CitySim::jitterHours(Agent& a, Real lo, Real hi) {
    return lo + (hi - lo) * static_cast<Real>(tripRnd(a) % 1000u) / 999.0;
}

// HOW LONG A VISIT LASTS (the activity catalog): the activity's own minutes, else what its kind of site keeps a visitor.
Real CitySim::visitHours(Agent& a) {
    double lo = 0, hi = 0;
    if (a.tripActivity >= 0 && a.tripActivity < static_cast<int>(catalog_.defs.size())) {
        const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(a.tripActivity)];
        lo = d.minutesLo; hi = d.minutesHi;
    }
    if (hi <= 0) siteKindMinutes(siteKindOf(a), lo, hi);
    return jitterHours(a, lo / 60.0, hi / 60.0);
}

void CitySim::computeGaps() {
    const Real INF = std::numeric_limits<Real>::infinity();
    // (read only for K agents: sized once, reset where they are -- not 2.4 MB of writes a tick at 100k)
    if (gaps_.size() != agents_.size()) {
        gaps_.assign(agents_.size(), INF);
        minGaps_.assign(agents_.size(), kNoLeaderMinGap);
        leaderSpeeds_.assign(agents_.size(), 0.0);
    } else {
        for (int k : active_) {
            gaps_[static_cast<std::size_t>(k)] = INF;
            minGaps_[static_cast<std::size_t>(k)] = kNoLeaderMinGap;   // overwritten where a leader exists
            leaderSpeeds_[static_cast<std::size_t>(k)] = 0.0;     // valid only where gaps_ < INF
        }
    }
    // FOLLOWING ON A WALK (W3): walkers on one walk, the same way, in the same file follow one another, by how far
    // along the walk the way they walk each is (a step's gap past its walk's end counts on from that end).
    if (!roads_) return;
    const engine::LaneGraph& G = roads_->laneLevel;
    auto keyOf = [this](const Agent& a, int leg) {
        const engine::roads::WalkStep& s = a.route.steps[static_cast<std::size_t>(leg)];
        return static_cast<long long>(s.walk.index) * 8 + (s.forward ? 4 : 0) + walkerFile(a);   // each file its own
    };
    auto wayAlong = [&](const engine::roads::WalkStep& s, Real metresOnStep) {   // metres along the walk, its way
        const Real W = G.walks[s.walk.index].length;
        const Real entry = s.forward ? s.from : W - s.from;
        return entry + metresOnStep;
    };
    std::unordered_map<long long, std::vector<std::pair<Real, int>>> lanes;
    for (int i : active_) {
        const Agent& a = agents_[static_cast<std::size_t>(i)];
        if (!onRoute(a)) continue;
        lanes[keyOf(a, a.leg)].push_back({wayAlong(a.route.steps[static_cast<std::size_t>(a.leg)], a.distOnLeg), i});
    }
    // (walk, way, file) -> the walker nearest its start, so a follower coming onto that walk picks it up as its leader
    std::unordered_map<long long, std::pair<Real, int>> minEntry;
    for (auto& kv : lanes) {
        std::vector<std::pair<Real, int>>& v = kv.second;
        std::sort(v.begin(), v.end(), [](const std::pair<Real, int>& a,
                                          const std::pair<Real, int>& b) {
            return a.first != b.first ? a.first < b.first : a.second < b.second;
        });
        minEntry[kv.first] = { v.front().first, v.front().second };
        for (std::size_t k = 0; k + 1 < v.size(); ++k) {
            // WALKERS go side by side, not in file: one LEVEL with me (within a body's depth along the way) is not in
            // front of me -- the lean takes us past each other. Counted as a leader, a crowd that set off from one
            // door together (a class change) stood at the same distance, each held behind the next, for ever.
            std::size_t j = k + 1;
            while (j < v.size() && v[j].first - v[k].first < kPedSideBySide) ++j;
            if (j >= v.size()) continue;
            const Real g = std::max(Real(0), v[j].first - v[k].first);
            if (g >= gaps_[v[k].second]) continue;
            gaps_[v[k].second] = g;
            minGaps_[v[k].second] = pairMinGap(v[k].second, v[j].second);
            leaderSpeeds_[v[k].second] = agents_[v[j].second].speed;
        }
    }
    // FOLLOWING ACROSS A JOIN: the front walker on a walk (no leader ahead on its own) keeps its gap to whoever is
    // ahead on the next steps of its route, the same way and file, within kFollowSight route metres. Never a walker
    // on a crossing: it walks on off the carriageway onto the pavement, beside whoever stands there, and is not held in
    // the road behind a queue on the far side. (Held there, river_town's crowd locked at one corner: a queue for the
    // next crosswalk filled the short corner walk, the people crossing to it stood on the first crosswalk behind it,
    // and the cars that gave way to them stood for good, among them the cars the queue was waiting for a gap in.)
    constexpr Real kFollowSight = 20.0;
    for (int i : active_) {
        const Agent& a = agents_[static_cast<std::size_t>(i)];
        if (gaps_[static_cast<std::size_t>(i)] != INF) continue;   // has a leader on its own walk already
        if (!onRoute(a)) continue;
        if (G.walks[a.route.steps[static_cast<std::size_t>(a.leg)].walk.index].modelCrosswalk != engine::kNoIndex) continue;
        const int legN = static_cast<int>(a.route.steps.size());
        Real ahead = legLength(a, a.leg) - a.distOnLeg;  // to the next step's start
        for (int step = 1; a.leg + step < legN && ahead < kFollowSight; ++step) {
            const int next = a.leg + step;
            const engine::roads::WalkStep& s = a.route.steps[static_cast<std::size_t>(next)];
            auto it = minEntry.find(keyOf(a, next));
            const Real entry = wayAlong(s, 0);
            if (it != minEntry.end() && it->second.first >= entry - kPedSideBySide) {
                gaps_[i] = std::max(Real(0), ahead + it->second.first - entry);
                minGaps_[i] = pairMinGap(i, it->second.second);
                leaderSpeeds_[i] = agents_[it->second.second].speed;
                break;
            }
            ahead += legLength(a, next);       // nobody on that step ahead of us; look one further
        }
    }
}


void CitySim::step(Real dt, Real hoursPerSecond) {
    // Cadence gate. Traffic is not physics: agents follow lanes, so advancing
    // by a bigger dt costs precision, not correctness. Bank the time and tick
    // when the period is due; everything downstream (including the far tier's
    // own schedule) then measures in SIM SECONDS rather than in calls.
    // THE TRAINS run every step (the train system draws them between steps), whatever the people's cadence
    if (!rail_.trains().empty()) rail_.step(dt);
    tickAccum_ += dt;
    sinceTick_ += dt;
    if (tickPeriodSeconds_ > 0.0 && tickAccum_ + 1e-9 < tickPeriodSeconds_) return;
    const Real simDt = tickAccum_;
    tickAccum_ = 0.0;
    sinceTick_ = 0.0;
    // The interpolation origin for this tick: where everyone was before it.
    for (int ti : street_) {   // (the street's people as of the last tick: nobody else moves but by being placed)
        Agent& a = agents_[static_cast<std::size_t>(ti)];
        a.tickFromPos = a.pos;
        a.tickFromHeading = a.heading;
    }
    stepTick(simDt, hoursPerSecond);
    // THE NEW DRIVING MODEL (#234), after the old one: it drives the cars handed to it (and its hand-over copies where
    // each is into its agent); with none handed over it ticks an empty world.
    if (drivingV2_) {
        if (beforeDriving_) beforeDriving_(*drivingV2_);   // (where the cars physics carries really are: ADR-0447)
        if (drivingV2_->thingStep()) {   // (what stands on the roads: ADR-0451)
            thingsForDriving(thingsScratch_);
            drivingV2_->setThingsOnRoad(thingsScratch_);
        }
        if (drivingHost_.active()) drivingHost_.tick(*drivingV2_, simDt);
        else drivingV2_->tick(simDt);
    }
    // THE PEOPLE ON THE BUSES (ADR-0484), after the buses moved: on and off at the stops the buses stand at, reported to
    // the brain
    if (buses_.any())
        for (const RideEvent& e : buses_.tick(agents_)) brainRideEvents_.push_back(e);
    // ...and ON THE TRAINS (step 4f): in at the stations, on and off at the platforms, out to the street
    if (trainRiders_.count() > 0)
        for (const RideEvent& e : trainRiders_.tick(simDt, agents_)) brainRideEvents_.push_back(e);
    // THE RESIDENT WITH THE AGENT BRAIN (ADR-0441), after the driving: its reports are answered and its skills stepped.
    if (brain_) tickBrain(simDt);
    // RT_TRACE_AGENT=<id>: that agent's plan, every tick (with the speed watchdog: what a flickering one is doing)
    static const int traceId = std::getenv("RT_TRACE_AGENT") ? std::atoi(std::getenv("RT_TRACE_AGENT")) : -1;
    if (traceId >= 0 && traceId < static_cast<int>(agents_.size())) {
        const Agent& t = agents_[static_cast<std::size_t>(traceId)];
        std::fprintf(stderr, "[trace %d] dt %.3f from %.2f %.2f -> %.2f %.2f (%.2f m) heading %.2f %.2f speed %.2f lean %.2f -> %.2f leg %d dist %.2f state %d\n",
                     traceId, simDt, t.tickFromPos.x, t.tickFromPos.y, t.pos.x, t.pos.y, (t.pos - t.tickFromPos).length(),
                     t.heading.x, t.heading.y, t.speed, t.lateralOffset, t.leanTarget, t.leg, t.distOnLeg, static_cast<int>(t.state));
    }
}

// ---- SEATS (the furniture library, M5) ------------------------------------------------------------------------
void CitySim::setSpots(std::vector<ActivitySpot> seats) {
    seats_.clear();
    if (walkPoints_.empty()) return;
    // the nearest node a walker can leave by, within 60 m: where a sit starts and ends
    const int n = walkPoints_.nodeCount();
    for (SeatSpot& s : seats) {
        // a seat is reached from the path within 60 m; a place on a track from as far as 150 (a field is wide, and a
        // runner crosses the grass to it)
        const Real reach = s.kind == SpotKind::Jog ? 150.0 : 60.0;
        Real best = reach * reach;
        s.node = -1;
        for (int i = 0; i < n; ++i) {
            const Vec2 d = walkPoints_.nodes[static_cast<std::size_t>(i)] - s.pos;
            const Real d2 = d.x * d.x + d.y * d.y;
            if (d2 >= best) continue;
            best = d2;
            s.node = i;
        }
        s.occupant = -1;
        if (s.node >= 0) seats_.push_back(s);
    }
    tagSpots();
}

void CitySim::tagSpots() {
    // IN A PARK: near a park's site (its door is its middle when it has no gate)
    auto nearPark = [&](Vec2 p, Real reach) {
        for (const Venue& v : venues_)
            if (v.type == PlaceType::Park && (v.door - p).lengthSquared() < reach * reach) return true;
        return false;
    };
    // the campus's lawns and its fields (places.lua tags "grounds", "sports")
    const uint32_t kGrounds = placeKinds_.tag("grounds"), kSports = placeKinds_.tag("sports");
    for (ActivityArea& ar : areas_) {
        ar.tags &= ~(spot_tag::kCampus | spot_tag::kSports | spot_tag::kPark);
        if (nearPark(ar.center, 60.0)) ar.tags |= spot_tag::kPark;
        for (const Venue& v : venues_) {
            if (!(v.tags & kGrounds)) continue;
            if ((v.door - ar.center).lengthSquared() >= 200.0 * 200.0) continue;
            ar.tags |= spot_tag::kCampus;
            if (v.tags & kSports) ar.tags |= spot_tag::kSports;
        }
    }
    for (ActivitySpot& s : seats_) {
        s.tags &= ~(spot_tag::kCampus | spot_tag::kSports | spot_tag::kPark);
        if (s.kind == SpotKind::Sit && nearPark(s.pos, 60.0)) s.tags |= spot_tag::kPark;
        for (const Venue& v : venues_) {
            if (!(v.tags & kGrounds)) continue;
            // (a track's far side is a field's width from its door)
            const Real reach = s.kind == SpotKind::Jog ? 200.0 : 120.0;
            if ((v.door - s.pos).lengthSquared() >= reach * reach) continue;
            s.tags |= spot_tag::kCampus;
            if (v.tags & kSports) s.tags |= spot_tag::kSports;
        }
    }
}

int CitySim::pickSpot(Agent& a, Vec2 here, const ActivityQuery& q) {
    std::vector<std::pair<Real, int>> c;
    for (int i = 0; i < static_cast<int>(seats_.size()); ++i) {
        const ActivitySpot& s = seats_[static_cast<std::size_t>(i)];
        if (s.occupant >= 0 || !(q.kinds & spotKindBit(s.kind)) || (s.tags & q.tags) != q.tags) continue;
        const Real d2 = (s.pos - here).lengthSquared();
        if (d2 < q.minDist * q.minDist || d2 > q.maxDist * q.maxDist) continue;
        c.push_back({d2, i});
    }
    if (c.empty()) return -1;
    const std::size_t k = std::min<std::size_t>(static_cast<std::size_t>(std::max(1, q.nearest)), c.size());
    std::partial_sort(c.begin(), c.begin() + static_cast<std::ptrdiff_t>(k), c.end());
    const int pick = c[rnd() % static_cast<uint32_t>(k)].second;
    seats_[static_cast<std::size_t>(pick)].occupant = indexOf(a);
    return pick;
}

int CitySim::pickActivity(Agent& a, int origin, const GoalState& st) {
    a.tripVenue = -1;
    releaseSeat(a);
    if (walkPoints_.empty() || origin < 0 || origin >= walkPoints_.nodeCount()) return -1;
    ActivityQuery q;
    q.kinds = st.spotKinds ? st.spotKinds : spotKindBit(SpotKind::Sit);
    q.tags = st.spotTags;
    const int s = pickSpot(a, walkPoints_.nodes[static_cast<std::size_t>(origin)], q);
    if (s < 0) return -1;
    a.tripSeat = s;
    return seats_[static_cast<std::size_t>(s)].node;
}

const CitySim::ActivitySpot* CitySim::usingSpot(int i) const {
    if (i < 0 || i >= static_cast<int>(agents_.size())) return nullptr;
    const Agent& a = agents_[static_cast<std::size_t>(i)];
    if (a.seatPhase != 2 || a.tripSeat < 0 || a.tripSeat >= static_cast<int>(seats_.size())) return nullptr;
    return &seats_[static_cast<std::size_t>(a.tripSeat)];
}

CitySim::Census CitySim::census(engine::World* world) const {
    Census c;
    if (world)
        world->each<CityCarRoams, CityCarBody, CityCarLink>([&](engine::Entity, CityCarRoams&, CityCarBody& body,
                                                                CityCarLink& link) {
            if (link.driver >= 0) return;
            ++c.emptyCars;
            const driving::Car* car = drivingV2_ ? drivingV2_->car(driving::CarId(body.id)) : nullptr;
            c.emptyCarsOnRoad += car ? 1 : 0;
            c.emptyCarsDriven += car && car->driven ? 1 : 0;
        });
    for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
        const Agent& a = agents_[static_cast<std::size_t>(i)];
        ++c.agents;
        const int r = std::min(Census::kRoles - 1, static_cast<int>(a.role));
        ++c.byRole[r];
        if (!a.moving && a.activity == Activity::AtWork) ++c.atWorkByRole[r];
        if (a.activity == Activity::Outing) ++c.outing;   // out: a day's stop, a night's (inside or not)
        if (trainRiders_.riding(i)) { ++c.waiting; ++c.outsideByRole[r]; continue; }
        ++c.walkers;
        const bool out = a.moving || !a.indoors;
        if (out) { ++c.walkersOutside; ++c.outsideByRole[r]; }
        if (a.moving) ++c.walkersMoving;
    }
    return c;
}

// ---- CROSSWALKS (ADR-0452) ----------------------------------------------------------------------------------------------
namespace {
// A car braking harder than this (m/s2) is slowing; one slower than this (m/s) stands: either means go to a person at
// the kerb (people.lua crossing: "a car slowing or standing means go").
constexpr Real kCarSlowing = 0.5;
constexpr Real kCarStanding = 0.5;
// The salts of the two crossing traits (each a hash of who the person is, as every trait).
constexpr uint32_t kStartUpSalt = 0x43524F53u, kJaywalkSalt = 0x4A415957u;
}  // namespace

const char* CitySim::crossingKindWord(CrossingKind k) {
    switch (k) {
    case CrossingKind::Lights: return "on WALK";
    case CrossingKind::Signs: return "at a stop or give-way sign";
    case CrossingKind::Marked: return "at a marked crossing with no lights";
    case CrossingKind::Unmarked: return "at an unmarked crossing";
    case CrossingKind::AgainstTheLights: return "against the lights";
    }
    return "?";
}

bool CitySim::gapToCross(const Agent& a, uint32_t crosswalk, Real waited, bool bigger, std::string* why) const {
    const engine::RoadModel* m = roads_.get();
    const engine::ModelCrosswalk* c = m ? m->crosswalk(crosswalk) : nullptr;
    if (!c || !drivingV2_ || !drivingV2_->roads()) return true;
    const CrossingTuning& t = people_.crossing;
    const Real personal = static_cast<Real>(t.startUpLo + (t.startUpHi - t.startUpLo) * engine::unit24(engine::hashPair(a.uid, kStartUpSalt)));
    const Real margin = std::max(static_cast<Real>(t.bolderFloor), personal - static_cast<Real>(t.bolderPerSecond) * waited);
    Real need = c->length / static_cast<Real>(t.walkSpeed) + margin;
    if (bigger) need *= static_cast<Real>(t.jaywalkGap);
    const engine::LaneGraph& roads = *drivingV2_->roads();
    const driving::PathLists& lists = drivingV2_->pathLists();
    // the cars that would cross it (on a path the model says crosses it, or on the lane leading into one), as a person
    // sees them: how far off, how fast, whether braking
    const auto threat = [&](const driving::PathEntry& e, Real distance) {
        const driving::Car* car = drivingV2_->car(e.car);
        if (!car || car->speed < kCarStanding || car->acceleration < -kCarSlowing) return false;   // standing or slowing: go
        const bool held = distance / car->speed < need;
        if (held && why)
            *why = "car " + std::to_string(e.car.index) + " coming at " + std::to_string(car->speed) + " m/s, " +
                   std::to_string(distance) + " m off (gap needed " + std::to_string(need) + " s)";
        return held;
    };
    for (const engine::CrosswalkCrosser& x : m->crossersOf(crosswalk)) {
        for (const driving::PathEntry& e : lists.on(x.path)) {
            // a car MOVING over the crosswalk: never step into it. A car standing on it is walked round, as people do
            // (it waits for something else -- the car ahead, a gap in the junction -- and nobody waits for it to go:
            // waiting for it while it waited behind a driver waiting for them stood 12 people and two cars 20 minutes
            // on brain_crowd's j81). A car standing short of it has stopped for them.
            const driving::Car* on = drivingV2_->car(e.car);
            const bool standing = on && on->speed < kCarStanding;
            if (e.rear <= x.to && e.front >= x.from && !standing) {
                if (why) *why = "car " + std::to_string(e.car.index) + " on the crosswalk, path " + std::to_string(x.path.index);
                return false;
            }
            if (e.why == driving::OnPath::Front && e.front < x.from && threat(e, x.from - e.front)) return false;
        }
        const engine::PathId lane = x.path.index < roads.paths.size() ? roads.paths[x.path.index].from : engine::PathId();
        if (!lane.valid() || lane.index >= roads.paths.size()) continue;
        for (const driving::PathEntry& e : lists.on(lane))
            if (e.why == driving::OnPath::Front && threat(e, roads.paths[lane.index].length - e.front + x.from)) return false;
    }
    return true;
}

bool CitySim::jaywalks(const Agent& a) const {
    // a seeded share of people, by who they are (a hash of their id, as every personal trait)
    return engine::unit24(engine::hashPair(a.uid, kJaywalkSalt)) < people_.crossing.jaywalkShare;
}

int CitySim::walkersWaitingAtKerbs() const {
    int n = 0;
    for (const Agent& a : agents_) n += a.moving && a.crosswalkAt >= 0 && !a.onCrosswalk ? 1 : 0;
    return n;
}

int CitySim::walkersOnCrosswalks() const {
    int n = 0;
    for (const Agent& a : agents_) n += a.moving && a.crosswalkAt >= 0 && a.onCrosswalk ? 1 : 0;
    return n;
}

void CitySim::thingsForDriving(std::vector<driving::RoadThing>& out) const {
    out.clear();
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        const Agent& a = agents_[i];
        if (a.inCar >= 0) continue;
        if (!a.moving && a.indoors) continue;
        driving::RoadThing t;
        t.id = a.uid;
        t.at = a.pos;
        t.halfAcross = kWalkerHalfAcross;
        t.velocity = a.moving ? a.heading * a.speed : Vec2();
        // (where they stand against a crosswalk, ADR-0452: on it, or at its kerb plainly about to cross)
        if (a.moving && a.crosswalkAt >= 0 && (a.onCrosswalk || a.aboutToCross)) {
            t.crosswalk = static_cast<uint32_t>(a.crosswalkAt);
            t.onCrosswalk = a.onCrosswalk;
        }
        out.push_back(t);
    }
    out.insert(out.end(), externalThings_.begin(), externalThings_.end());
}

std::string CitySim::describeAgent(int i) const {
    if (i < 0 || i >= static_cast<int>(agents_.size())) return "no such agent";
    const Agent& a = agents_[static_cast<std::size_t>(i)];
    static const char* kStates[] = {"resting", "walking", "avoiding", "waiting", "cruising", "following", "yielding", "turning"};
    const int st = static_cast<int>(a.state);
    auto actName = [&](int d) { return d >= 0 && d < static_cast<int>(catalog_.defs.size()) ? catalog_.defs[static_cast<std::size_t>(d)].name.c_str() : "-"; };
    int walk = -1;
    std::string walkKind;   // what the walk under them is (the model's word: sidewalk, corner, crossing, ...)
    if (onRoute(a) && roads_) {
        walk = static_cast<int>(a.route.steps[static_cast<std::size_t>(a.leg)].walk.index);
        walkKind = " (" + roads_->walkKind(engine::WalkId(static_cast<uint32_t>(walk))) + ")";
    }
    const char* sess = "-";
    if (a.session >= 0 && a.session < static_cast<int>(sessions_.size())) {
        static const char* kSes[] = {"gathering", "running", "ending", "dead"};
        sess = kSes[static_cast<int>(sessions_[static_cast<std::size_t>(a.session)].state)];
    }
    const char* stageName = a.stage >= 0 && a.stage < static_cast<int>(people_.stages.size())
                                ? people_.stages[static_cast<std::size_t>(a.stage)].name.c_str() : "-";
    char b[800];
    std::snprintf(b, sizeof b,
                  "agent %d %s age %d %s household %d height %.2f ride %s pos %.1f %.1f speed %.2f heading %.2f %.2f moving %d state %s indoors %d visible %d | trip venue %d "
                  "seat %d phase %d activity %s (at %s) session %d %s role %d | route leg %d/%zu walk %d%s dist %.1f | rest %.3f h "
                  "goal %.3f h | tethered %d player %d | lean %.2f -> %.2f",
                  i, "walker", a.age, stageName, a.household,
                  static_cast<double>(bodyScale(i)), rideName(rideOf(i)), a.pos.x, a.pos.y,
                  a.speed, a.heading.x, a.heading.y, a.moving ? 1 : 0, st >= 0 && st < 8 ? kStates[st] : "?", a.indoors ? 1 : 0,
                  pedVisible(i) ? 1 : 0, a.tripVenue, a.tripSeat, a.seatPhase, actName(a.tripActivity), actName(a.atActivity),
                  a.session, sess, a.sessionRole, a.leg, a.route.steps.size(), walk, walkKind.c_str(), a.distOnLeg,
                  a.restDwell, a.goalHours, a.tethered ? 1 : 0, a.playerControlled ? 1 : 0, a.lateralOffset, a.leanTarget);
    return b;
}

bool CitySim::restPose(int i, RestPose& out) const {
    if (const SeatSpot* st = seatedOn(i)) {
        out.kind = st->kind == SpotKind::Lie ? RestPose::Kind::Lie : RestPose::Kind::Seat;
        out.raised = st->kind == SpotKind::Lie;
        out.pos = st->pos;
        out.face = st->face;
        out.hip = st->hip;
        return true;
    }
    if (i < 0 || i >= static_cast<int>(agents_.size())) return false;
    const Agent& a = agents_[static_cast<std::size_t>(i)];
    if (a.seatPhase != 2 || a.speed > 0 || a.session < 0 || a.session >= static_cast<int>(sessions_.size())) return false;
    const Session& se = sessions_[static_cast<std::size_t>(a.session)];
    if (se.formation == Formation::Roam || se.def < 0) return false;
    const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(se.def)];
    if (a.sessionRole < 0 || a.sessionRole >= static_cast<int>(d.roles.size())) return false;
    const Pose pose = d.roles[static_cast<std::size_t>(a.sessionRole)].pose;
    if (pose == Pose::Stand) return false;
    out.kind = pose == Pose::Lie ? RestPose::Kind::Lie : RestPose::Kind::SitGround;
    out.pos = a.pos;
    out.face = a.heading;
    out.hip = 0;
    return true;
}

const CitySim::SeatSpot* CitySim::seatedOn(int i) const {
    const ActivitySpot* s = usingSpot(i);
    return s && (s->kind == SpotKind::Sit || s->kind == SpotKind::Lie) ? s : nullptr;
}

void CitySim::setLoops(std::vector<std::vector<Vec2>> loops) {
    loops_.clear();
    for (std::vector<Vec2>& pts : loops) {
        if (pts.size() < 3) continue;
        Loop L;
        L.pts = std::move(pts);
        L.cum.push_back(0);
        for (std::size_t i = 0; i < L.pts.size(); ++i) {
            L.length += (L.pts[(i + 1) % L.pts.size()] - L.pts[i]).length();
            L.cum.push_back(L.length);
        }
        loops_.push_back(std::move(L));
    }
}

void CitySim::setAreas(std::vector<ActivityArea> areas) {
    areas_.clear();
    sessions_.clear();
    if (walkPoints_.empty()) return;
    const int n = walkPoints_.streetNodeCount();
    for (ActivityArea& ar : areas) {
        Real best = 150.0 * 150.0;   // a field is wide: its players cross the grass to it
        ar.node = -1;
        for (int i = 0; i < n; ++i) {
            const Real d2 = (walkPoints_.nodes[static_cast<std::size_t>(i)] - ar.center).lengthSquared();
            if (d2 >= best) continue;
            best = d2;
            ar.node = i;
        }
        if (ar.node >= 0) areas_.push_back(ar);
    }
    tagSpots();
}

int CitySim::Session::roleCount(int role) const {
    int n = 0;
    for (int r : roles) n += r == role ? 1 : 0;
    return n;
}

Vec2 CitySim::zonePoint(const Session& s, int role, uint32_t bits) const {
    const ActivityArea& ar = areas_[static_cast<std::size_t>(s.area)];
    const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(s.def)];
    int zone = role >= 0 && role < static_cast<int>(d.roles.size()) ? d.roles[static_cast<std::size_t>(role)].zone : -1;
    if (zone >= 0 && s.swapped) zone = 1 - zone;
    const Real inL = std::max(Real(0.5), ar.halfL - 2.0), inW = std::max(Real(0.5), ar.halfW - 1.5);
    const Real u0 = zone == 1 ? 1.0 : -inL, u1 = zone == 0 ? -1.0 : inL;
    const Real fu = static_cast<Real>(bits & 0xFFFF) / 65535.0, fv = static_cast<Real>((bits >> 16) & 0xFFFF) / 65535.0;
    const Vec2 side(ar.axis.y, -ar.axis.x);
    return ar.center + ar.axis * (u0 + (u1 - u0) * fu) + side * (-inW + 2 * inW * fv);
}

// Join a session of activity `di` on the nearest suitable area: one gathering or running with a free role, else a
// fresh one on an area with nothing on it. Returns the area, -1 for none.
int CitySim::joinSession(Agent& a, int di, Vec2 here, Real distLo, Real distHi, int nearest) {
    const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(di)];
    std::vector<std::pair<Real, int>> cand;
    for (int i = 0; i < static_cast<int>(areas_.size()); ++i) {
        const ActivityArea& ar = areas_[static_cast<std::size_t>(i)];
        bool kind = false;
        for (const std::string& k : d.sites) kind = kind || k == ar.kind;
        if (!kind || (ar.tags & d.tags) != d.tags) continue;
        const Real d2 = (ar.center - here).lengthSquared();
        if (d2 < distLo * distLo || d2 > distHi * distHi) continue;
        cand.push_back({d2, i});
    }
    std::sort(cand.begin(), cand.end());
    if (nearest > 0 && static_cast<int>(cand.size()) > nearest) cand.resize(static_cast<std::size_t>(nearest));
    auto freeRole = [&](const Session& se) {
        int best = -1;
        Real bestFill = 1e9;
        for (int r = 0; r < static_cast<int>(d.roles.size()); ++r) {
            const int n = se.roleCount(r), cap = d.roles[static_cast<std::size_t>(r)].n;
            if (n >= cap) continue;
            const Real fill = static_cast<Real>(n) / std::max(1, cap);
            if (fill < bestFill) { bestFill = fill; best = r; }
        }
        return best;
    };
    // A SETTLED GROUP (a ring, a pair, sunbathers) shares its lawn: a chat anyone may join anywhere in reach first,
    // then a fresh one wherever a lawn has room for it
    if (d.formation != Formation::Roam) return joinSettled(a, di, cand, freeRole);
    for (const auto& c : cand) {
        // a session already here, of this activity, with room
        int si = -1, role = -1;
        bool busy = false;
        for (int k = 0; k < static_cast<int>(sessions_.size()); ++k) {
            Session& se = sessions_[static_cast<std::size_t>(k)];
            if (se.area != c.second || se.state == Session::State::Dead) continue;
            busy = true;
            if (se.def != di || (se.state != Session::State::Gathering && se.state != Session::State::Running)) continue;
            role = freeRole(se);
            if (role >= 0) { si = k; break; }
        }
        if (si < 0 && !busy) {   // nothing on this area: start one
            Session se;
            se.def = di;
            se.area = c.second;
            se.gatherUntil = clockTotalHours_ + d.gatherMinutes / 60.0;
            se.center = areas_[static_cast<std::size_t>(c.second)].center;
            se.axis = areas_[static_cast<std::size_t>(c.second)].axis;
            for (int k = 0; k <= static_cast<int>(sessions_.size()); ++k)
                if (k == static_cast<int>(sessions_.size()) || sessions_[static_cast<std::size_t>(k)].state == Session::State::Dead) {
                    if (k == static_cast<int>(sessions_.size())) sessions_.push_back(se); else sessions_[static_cast<std::size_t>(k)] = se;
                    si = k;
                    break;
                }
            role = freeRole(sessions_[static_cast<std::size_t>(si)]);
        }
        if (si < 0 || role < 0) continue;
        Session& se = sessions_[static_cast<std::size_t>(si)];
        se.members.push_back(indexOf(a));
        se.roles.push_back(role);
        // a group still forming waits a while longer for each new player (they are walking over)
        if (se.state == Session::State::Gathering)
            se.gatherUntil = std::max(se.gatherUntil, clockTotalHours_ + d.gatherMinutes / 60.0 * 0.5);
        a.session = si;
        a.sessionRole = role;
        return c.second;
    }
    return -1;
}

// a small integer hash (lowbias32): places and beats that must be the same whoever asks

// Is `p` at least `r` from every walkway's line (a walk's half width counted in)? The road model's walkways (ADR-0475),
// as the lane level laid them: each walkway walk's line, a segment every couple of metres.
bool CitySim::clearOfWalks(Vec2 p, Real r) const {
    if (!roads_ || !roads_->built) return true;
    const engine::LaneGraph& g = roads_->laneLevel;
    const Real reach2 = (r + 1.0) * (r + 1.0);
    for (uint32_t w = 0; w < g.walks.size(); ++w) {
        const engine::Walk& k = g.walks[w];
        if (k.modelWalkway == engine::kNoIndex || !(k.length > 0)) continue;
        const int steps = std::max(1, static_cast<int>(std::ceil(k.length / 2.0)));
        Vec2 a = g.pointAt(engine::WalkId(w), 0);
        for (int i = 1; i <= steps; ++i) {
            const Vec2 b = g.pointAt(engine::WalkId(w), k.length * static_cast<Real>(i) / static_cast<Real>(steps));
            const Vec2 ab = b - a;
            const Real l2 = ab.lengthSquared();
            const Real t = l2 > 1e-9 ? std::clamp(dot(p - a, ab) / l2, Real(0), Real(1)) : 0;
            if ((a + ab * t - p).lengthSquared() < reach2) return false;
            a = b;
        }
    }
    return true;
}

int CitySim::joinSettled(Agent& a, int di, const std::vector<std::pair<Real, int>>& cand,
                         const std::function<int(const Session&)>& freeRole) {
    const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(di)];
    auto enlist = [&](int si, int role) {
        Session& se = sessions_[static_cast<std::size_t>(si)];
        se.members.push_back(indexOf(a));
        se.roles.push_back(role);
        if (se.state == Session::State::Gathering)
            se.gatherUntil = std::max(se.gatherUntil, clockTotalHours_ + d.gatherMinutes / 60.0 * 0.5);
        a.session = si;
        a.sessionRole = role;
        return se.area;
    };
    // join one going (gathering or under way) with a place free, on the nearest area that has one
    for (const auto& c : cand)
        for (int k = 0; k < static_cast<int>(sessions_.size()); ++k) {
            const Session& se = sessions_[static_cast<std::size_t>(k)];
            if (se.area != c.second || se.def != di) continue;
            if (se.state != Session::State::Gathering && se.state != Session::State::Running) continue;
            const int role = freeRole(se);
            if (role >= 0) return enlist(k, role);
        }
    // start one: a spot on the area clear of the groups already on it (and of a game: a lawn with a kickabout on it is
    // the kickabout's), the farthest of a few tries from them
    const Real need = std::max(Real(2.0), static_cast<Real>(d.radius)) + 3.0;
    for (const auto& c : cand) {
        const ActivityArea& ar = areas_[static_cast<std::size_t>(c.second)];
        bool game = false;
        std::vector<Vec2> taken;
        for (const Session& se : sessions_) {
            if (se.area != c.second || se.state == Session::State::Dead) continue;
            if (se.formation == Formation::Roam) game = true;
            taken.push_back(se.center);
        }
        if (game) continue;
        const Vec2 side(ar.axis.y, -ar.axis.x);
        // a pair needs its length along the area, a ring its width
        const Real reachL = static_cast<Real>(d.formation == Formation::Pair ? d.radius * 0.5 + 1.0 : 2.0);
        // ...and a spread its row across it (sunbathers side by side, the most its role takes)
        Real reachW = 2.0;
        if (d.formation == Formation::Spread)
            for (const RoleDef& r : d.roles) reachW = std::max(reachW, static_cast<Real>((r.n - 1) * d.radius * 0.5 + 0.6));
        const Real inL = ar.halfL - reachL, inW = ar.halfW - reachW;
        if (inL <= 0 || inW <= 0) continue;
        Vec2 best;
        Real bestD = -1;
        uint32_t h = engine::lowbias32(static_cast<uint32_t>(indexOf(a)) * 2654435761u ^ static_cast<uint32_t>(clockTotalHours_ * 3600.0));
        for (int t = 0; t < 16; ++t) {
            h = engine::lowbias32(h + 0x9E37u);
            const Real fu = static_cast<Real>(h & 0xFFFF) / 65535.0 * 2 - 1, fv = static_cast<Real>(h >> 16) / 65535.0 * 2 - 1;
            const Vec2 p = ar.center + ar.axis * (inL * fu) + side * (inW * fv);
            // not on a walk (a lawn's box takes in the walks across it): a ring's edge, or each end of a pair
            if (d.formation == Formation::Pair) {
                if (!clearOfWalks(p - ar.axis * (d.radius * 0.5), 1.5) || !clearOfWalks(p + ar.axis * (d.radius * 0.5), 1.5)) continue;
            } else if (!clearOfWalks(p, std::max(Real(1.0), static_cast<Real>(d.radius)) + 1.2)) continue;
            Real dmin = 1e9;
            for (const Vec2& q : taken) dmin = std::min(dmin, (q - p).length());
            if (dmin > bestD) { bestD = dmin; best = p; }
        }
        if (bestD < 2 * need) continue;   // this lawn is full
        Session se;
        se.def = di;
        se.area = c.second;
        se.formation = d.formation;
        se.center = best;
        se.axis = ar.axis;
        se.gatherUntil = clockTotalHours_ + d.gatherMinutes / 60.0;
        int si = -1;
        for (int k = 0; k <= static_cast<int>(sessions_.size()); ++k)
            if (k == static_cast<int>(sessions_.size()) || sessions_[static_cast<std::size_t>(k)].state == Session::State::Dead) {
                if (k == static_cast<int>(sessions_.size())) sessions_.push_back(se); else sessions_[static_cast<std::size_t>(k)] = se;
                si = k;
                break;
            }
        const int role = freeRole(sessions_[static_cast<std::size_t>(si)]);
        if (role < 0) { sessions_[static_cast<std::size_t>(si)].state = Session::State::Dead; continue; }
        return enlist(si, role);
    }
    return -1;
}

// A SETTLED GROUP'S PLACES: a ring round its middle facing in (a body's shoulders apart, never tighter than the
// definition's radius); a pair `radius` apart along the session's axis, facing, each stepping a little to and fro
// every few seconds (a catch); sunbathers side by side, feet the same way.
Vec2 CitySim::memberPlace(const Session& se, int agentIndex, Vec2* face) const {
    const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(se.def)];
    int k = 0;
    const int n = std::max(1, static_cast<int>(se.members.size()));
    for (int i = 0; i < static_cast<int>(se.members.size()); ++i)
        if (se.members[static_cast<std::size_t>(i)] == agentIndex) k = i;
    const Vec2 side(se.axis.y, -se.axis.x);
    Vec2 p = se.center, f = se.axis;
    switch (se.formation) {
        case Formation::Circle: {
            const Real r = std::max(static_cast<Real>(d.radius), Real(0.75) * n / Real(6.2831853));
            const Real ang = 6.2831853 * k / n + static_cast<Real>(se.def) * 0.7;
            const Vec2 out(std::cos(ang), std::sin(ang));
            p = se.center + out * (n == 1 ? 0.0 : r);
            f = n == 1 ? se.axis : out * -1.0;
            break;
        }
        case Formation::Pair: {
            const Real s = k == 0 ? -1.0 : 1.0;
            // a step this way or that every four seconds or so, as the throws come
            const uint32_t beat = static_cast<uint32_t>(clockTotalHours_ * 3600.0 / 4.0);
            const uint32_t h = engine::lowbias32(static_cast<uint32_t>(agentIndex) * 2246822519u + beat);
            const Real lat = (static_cast<Real>(h & 0xFF) / 255.0 * 2 - 1) * 1.2;
            const Real lon = (static_cast<Real>((h >> 8) & 0xFF) / 255.0 * 2 - 1) * 0.8;
            p = se.center + se.axis * (s * (d.radius * 0.5 + lon)) + side * lat;
            f = se.axis * -s;
            break;
        }
        case Formation::Spread: {
            const Real gap = d.radius > 0 ? d.radius : 1.1;
            p = se.center + side * ((k - (n - 1) * 0.5) * gap);
            f = se.axis;
            break;
        }
        case Formation::Roam: break;
    }
    if (face) *face = f;
    return p;
}

void CitySim::leaveSession(Agent& a) {
    if (a.session < 0 || a.session >= static_cast<int>(sessions_.size())) { a.session = -1; return; }
    Session& se = sessions_[static_cast<std::size_t>(a.session)];
    const int me = indexOf(a);
    for (std::size_t k = 0; k < se.members.size(); ++k)
        if (se.members[k] == me) {
            se.members.erase(se.members.begin() + static_cast<std::ptrdiff_t>(k));
            se.roles.erase(se.roles.begin() + static_cast<std::ptrdiff_t>(k));
            break;
        }
    if (se.members.empty()) se.state = Session::State::Dead;
    a.session = -1;
    a.sessionRole = -1;
}

void CitySim::stepSessions() {
    for (Session& se : sessions_) {
        if (se.state == Session::State::Dead) continue;
        const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(se.def)];
        int present = 0;
        for (int m : se.members) present += agents_[static_cast<std::size_t>(m)].seatPhase == 2 ? 1 : 0;
        const int need = d.minPlayers > 0 ? d.minPlayers : 1;
        if (se.state == Session::State::Gathering) {
            if (present >= need) {
                se.state = Session::State::Running;
                se.startedAt = clockTotalHours_;
                double lo = d.minutesLo, hi = d.minutesHi;
                if (hi <= 0) siteKindMinutes(d.sites.empty() ? "pitch" : d.sites.front(), lo, hi);
                const Real f = static_cast<Real>(rnd() % 1000u) / 999.0;
                se.endAt = clockTotalHours_ + (lo + (hi - lo) * f) / 60.0;
            } else if (clockTotalHours_ >= se.gatherUntil) {
                se.state = Session::State::Ending;   // never filled: give it up
            }
        } else if (se.state == Session::State::Running) {
            if (d.swapAt > 0 && !se.swapped && clockTotalHours_ >= se.startedAt + (se.endAt - se.startedAt) * d.swapAt) {
                se.swapped = true;
                for (int m : se.members) {   // to the other end
                    Agent& p = agents_[static_cast<std::size_t>(m)];
                    p.roamTarget = zonePoint(se, p.sessionRole, tripRnd(p));
                }
            }
            if (clockTotalHours_ >= se.endAt) se.state = Session::State::Ending;
        }
        if (se.state == Session::State::Ending) {
            std::vector<int> onTheWay;   // still walking to it: let go (they arrive to nothing, and stand about)
            for (int m : se.members) {
                Agent& p = agents_[static_cast<std::size_t>(m)];
                if (p.seatPhase == 1 || p.seatPhase == 2) p.seatPhase = 3;
                else if (p.seatPhase == 0) onTheWay.push_back(m);
            }
            for (int m : onTheWay) leaveSession(agents_[static_cast<std::size_t>(m)]);
        }
    }
}

// A TRIP'S SPOT BEGINS (ADR-0512): the brain's person has arrived where their pick held a seat, a track or a place in a
// group (tripSeat, session; CitySim::choosePickedNode). Near: they walk onto it (stepSpots). Far: they are put on it at
// once, nothing drawn moving. A group's player stays until the group ends (the session says, stepSessions). False: the
// trip held none (or lost it), and nothing changes.
bool CitySim::beginSpotUse(Agent& a, bool far) {
    if (a.session >= 0 && a.session < static_cast<int>(sessions_.size())) {
        const Session& se = sessions_[static_cast<std::size_t>(a.session)];
        a.restDwell = 1e9;
        a.seatBack = a.pos;
        a.roamTarget = se.formation != Formation::Roam ? memberPlace(se, indexOf(a), nullptr) : zonePoint(se, a.sessionRole, tripRnd(a));
        a.seatPhase = 1;
        if (far) {
            a.pos = a.roamTarget;
            a.speed = 0;
            a.seatPhase = 2;
            grid_.place(indexOf(a), a.pos);
        }
        return true;
    }
    if (a.tripSeat >= 0 && a.tripSeat < static_cast<int>(seats_.size()) &&
        seats_[static_cast<std::size_t>(a.tripSeat)].occupant == indexOf(a)) {
        a.seatBack = a.pos;
        a.seatPhase = 1;
        if (far) {
            const SeatSpot& sp = seats_[static_cast<std::size_t>(a.tripSeat)];
            a.pos = sp.pos;
            a.heading = sp.face;
            a.loopS = sp.loopS;
            a.speed = 0;
            a.seatPhase = 2;
            grid_.place(indexOf(a), a.pos);
        }
        return true;
    }
    if (a.tripSeat >= 0) releaseSeat(a);
    return false;
}

// THE BODIES ON A SPOT (ADR-0512; the old sim's stepSeats, back as a body step): whoever the brain has put on a seat, a
// track or a group's area walks onto it, sits or runs there, holds its place in the group, and walks back to where it
// left the path when the group ends. The brain says when they go and how long they stay; this only moves them.
void CitySim::stepSpots(Real dt) {
    constexpr Real kWalk = 1.2;   // an amble across the grass
    constexpr Real kJog = 2.8;    // a steady run
    stepSessions();
    for (int si : street_) {
        Agent& a = agents_[static_cast<std::size_t>(si)];
        if (a.seatPhase == 0) continue;
        // A GROUP'S PLAYER (a session): onto the area to its first point; there, standing by while the group gathers,
        // or running to point after point of its zone; and, the session over, back to where it left the path
        if (a.session >= 0 && a.session < static_cast<int>(sessions_.size())) {
            const Session& se = sessions_[static_cast<std::size_t>(a.session)];
            const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(se.def)];
            const RoleDef* role = a.sessionRole >= 0 && a.sessionRole < static_cast<int>(d.roles.size())
                                      ? &d.roles[static_cast<std::size_t>(a.sessionRole)] : nullptr;
            Vec2 target = a.seatPhase == 3 ? a.seatBack : a.roamTarget;
            Real speed = kWalk;
            if (a.seatPhase == 2 && se.formation != Formation::Roam) {
                // A SETTLED GROUP: to its place (it moves as the ring grows, or with the catch), then held there,
                // facing its way -- gathering or under way alike
                Vec2 face;
                const Vec2 place = memberPlace(se, indexOf(a), &face);
                const Vec2 dd = place - a.pos;
                const Real L = dd.length();
                const Real pace = se.formation == Formation::Pair ? 1.6 : kWalk;
                if (L <= pace * dt || L < 0.02) {
                    a.pos = place;
                    a.heading = face;
                    a.speed = 0;
                } else {
                    a.pos = a.pos + dd * (std::min(L, pace * dt) / L);
                    a.heading = dd * (1.0 / L);
                    a.speed = pace;
                }
                grid_.place(indexOf(a), a.pos);
                continue;
            }
            if (a.seatPhase == 2) {
                if (se.state != Session::State::Running) {   // gathering: stand by, facing the middle
                    a.speed = 0;
                    const Vec2 to = areas_[static_cast<std::size_t>(se.area)].center - a.pos;
                    if (to.lengthSquared() > 1e-6) a.heading = normalize(to);
                    continue;
                }
                const Real f = static_cast<Real>((a.brain >> 7) & 0xFF) / 255.0;
                speed = role ? role->speedLo + (role->speedHi - role->speedLo) * f : 2.0;
            }
            const Vec2 dd = target - a.pos;
            const Real L = dd.length(), step = speed * dt;
            if (L <= step) {
                a.pos = target;
                if (a.seatPhase == 1) { a.seatPhase = 2; a.goalHours = 0; a.speed = 0; }
                else if (a.seatPhase == 2) a.roamTarget = zonePoint(se, a.sessionRole, tripRnd(a));
                else {   // off the area: out of the session, and the rest that held it ends now
                    leaveSession(a);
                    a.seatPhase = 0;
                    a.restDwell = 1e-3;
                    a.speed = 0;
                }
            } else {
                a.pos = a.pos + dd * (step / L);
                a.heading = dd * (1.0 / L);
                a.speed = speed;
            }
            grid_.place(indexOf(a), a.pos);
            continue;
        }
        if (a.tripSeat < 0 || a.tripSeat >= static_cast<int>(seats_.size())) { a.seatPhase = 0; continue; }
        const SeatSpot& s = seats_[static_cast<std::size_t>(a.tripSeat)];
        // RUNNING THE LOOP (a Jog spot): round it at a jogger's pace until the run's time is up (goalThink)
        if (a.seatPhase == 2) {
            if (s.kind != SpotKind::Jog || s.loop < 0) continue;
            a.loopS += kJog * dt;
            Vec2 tg(1, 0);
            a.pos = loopPoint(s.loop, a.loopS, &tg);
            a.heading = tg;
            a.speed = kJog;
            grid_.place(indexOf(a), a.pos);
            continue;
        }
        // to the seat, stopping in front of it (a jogger: onto the track at its place); back to where it left the path
        const Vec2 target = a.seatPhase == 1 ? (s.kind == SpotKind::Jog ? s.pos : s.pos + s.face * 0.35) : a.seatBack;
        const Vec2 d = target - a.pos;
        const Real L = d.length(), step = kWalk * dt;
        if (L <= step) {
            a.pos = target;
            a.speed = 0;
            if (a.seatPhase == 1) {
                a.seatPhase = 2;
                a.pos = s.pos;
                a.heading = s.face;
                a.loopS = s.loopS;
                a.goalHours = 0;   // the sit's own clock starts now, not on the walk over
            } else {
                releaseSeat(a);
            }
        } else {
            a.pos = a.pos + d * (step / L);
            a.heading = d * (1.0 / L);
            a.speed = kWalk;
        }
        grid_.place(indexOf(a), a.pos);
    }
}

Real CitySim::loopLength(int loop) const {
    return loop >= 0 && loop < static_cast<int>(loops_.size()) ? loops_[static_cast<std::size_t>(loop)].length : 0;
}

Vec2 CitySim::loopPoint(int loop, Real s, Vec2* tangent) const {
    if (loop < 0 || loop >= static_cast<int>(loops_.size())) return Vec2(0, 0);
    const Loop& L = loops_[static_cast<std::size_t>(loop)];
    if (L.length <= 0) return L.pts.front();
    s = std::fmod(s, L.length);
    if (s < 0) s += L.length;
    const std::size_t i = static_cast<std::size_t>(std::upper_bound(L.cum.begin(), L.cum.end(), s) - L.cum.begin()) - 1;
    const std::size_t k = std::min(i, L.pts.size() - 1);
    const Vec2 a = L.pts[k], b = L.pts[(k + 1) % L.pts.size()];
    const Real seg = L.cum[k + 1] - L.cum[k];
    const Real t = seg > 1e-9 ? (s - L.cum[k]) / seg : 0;
    if (tangent) *tangent = seg > 1e-9 ? (b - a) * (1.0 / seg) : Vec2(1, 0);
    return a + (b - a) * t;
}

void CitySim::releaseSeat(Agent& a) {
    if (a.session >= 0) leaveSession(a);
    if (a.tripSeat >= 0 && a.tripSeat < static_cast<int>(seats_.size()) &&
        seats_[static_cast<std::size_t>(a.tripSeat)].occupant == indexOf(a))
        seats_[static_cast<std::size_t>(a.tripSeat)].occupant = -1;
    a.tripSeat = -1;
    a.seatPhase = 0;
}

int CitySim::pickSeat(Agent& a, Vec2 here) {
    // a free seat 30-500 m off, one of the nearest four (so neighbours do not all take the same bench)
    ActivityQuery q;
    q.kinds = spotKindBit(SpotKind::Sit);
    q.minDist = 30.0; q.maxDist = 500.0; q.nearest = 4;
    return pickSpot(a, here, q);
}

void CitySim::stepTick(Real dt, Real hoursPerSecond) {
    RT_PROFILE_ZONE_NAMED("CitySim step");
    if (walkPoints_.empty() || agents_.empty()) return;
    // Print-only phase timing (CitySim::PhaseTimes): whole-population passes
    // vs per-active-agent passes. Stage 1 of the 100k plan needs a baseline.
    const auto phaseNow = std::chrono::steady_clock::now;
    auto phaseT0 = phaseNow();
    const auto stepBegin = phaseT0;
    const auto phaseMark = [&](double& into) {
        const auto n2 = phaseNow();
        into += std::chrono::duration<double, std::micro>(n2 - phaseT0).count();
        phaseT0 = n2;
    };

    // WHO IS OUT ON THE STREET (ADR-0499): walking, or standing outside -- the only people the passes below are about.
    // Everyone else is indoors, or carried (in a car, on a bus), and stands where the brain put them. (These were the
    // K tier's lists: every agent the brain owns was pinned there, and the island's 115,000 were walked through every
    // tick for nothing.)
    street_.clear();
    active_.clear();
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        const Agent& a = agents_[i];
        if (!a.moving && !pedVisible(static_cast<int>(i))) continue;
        street_.push_back(static_cast<int>(i));
        if (a.moving) active_.push_back(static_cast<int>(i));
    }
    stepSpots(dt);   // (the people on a seat, a track or a group's area: ADR-0512)
    if (rehashAll_) {
        for (std::size_t i = 0; i < agents_.size(); ++i) grid_.place(static_cast<int>(i), agents_[i].pos);
        rehashAll_ = false;
    } else {
        for (int i : street_) grid_.place(i, agents_[static_cast<std::size_t>(i)].pos);
    }
    phaseMark(phase_.rehash);
    hoursPerSecond_ = hoursPerSecond;
    // the staffed hours ride on the clock rate (a commute is so many seconds, so much of the day): a new rate, new hours
    if (hoursPerSecond > 0 && staffingRate_ > 0 && std::fabs(hoursPerSecond - staffingRate_) > staffingRate_ * 1e-3) indexVenueStaffing();
    phaseMark(phase_.activeList);

    clockHours_ += dt * hoursPerSecond;
    clockHours_ = std::fmod(clockHours_, 24.0);
    if (clockHours_ < 0) clockHours_ += 24.0;
    clockTotalHours_ += dt * hoursPerSecond;
    simSeconds_ += dt;   // memory's time base: sightings age and fade against this

    // Snapshot the bodies agents may SENSE this step — pedestrians and the
    // player — so perception sees a consistent world. Each carries a STABLE id
    // (agent index; -(1+k) for the k-th host-injected point) so an observer's
    // memory can track it across steps, and its elevation so the 2.5D sensor can
    // ignore bodies a bridge deck away. AI cars are deliberately excluded (see
    // advance()): lanes + car-following + signals govern car-vs-car, and braking
    // for cross/oncoming cars in the cone deadlocked traffic.
    sensed_.clear();
    // agent -> its ghost (grid lookups, which find any tier: every entry not set this tick must read -1, so the
    // ones set LAST tick are cleared rather than the whole array)
    if (sensedIndex_.size() != agents_.size()) sensedIndex_.assign(agents_.size(), -1);
    else for (int k : sensedSet_) sensedIndex_[static_cast<std::size_t>(k)] = -1;
    sensedSet_.clear();
    for (int ai : active_) {
        const Agent& a = agents_[ai];
        const std::size_t i = static_cast<std::size_t>(ai);
        // A RESTING pedestrian is INSIDE its place (home/shop/office) — not a
        // body on the street. Sensing it anyway had cars braking forever for a
        // person who is semantically indoors (a rest pose near the kerb sat in
        // the sensed corridor and froze traffic for good — device jam).
        if (!a.moving && !a.playerControlled)
            continue;
        {
            // A tethered walker has a physical BODY (the tether anchor the
            // bridge feeds every step) that may trail its ghost by metres —
            // others must avoid where the person IS, not where the plan is.
            Vec2 sp = a.tethered ? a.tetherAnchor : a.pos;
            sensedIndex_[i] = static_cast<int>(sensed_.size());
            sensedSet_.push_back(static_cast<int>(i));
            sensed_.push_back({sp, a.elevation, static_cast<int>(i)});
        }
    }
    // The live player (host-injected) is a body too — cars brake for it.
    for (std::size_t k = 0; k < externalObstacles_.size(); ++k)
        sensed_.push_back({externalObstacles_[k], 0.0, -(1 + static_cast<int>(k))});

    // Pass 2: leading gaps. Pass 3: advance. `advanced` records who really ran
    // advance() (and so had refreshPose re-anchor its base position) this tick —
    // the reactive lean below is only valid on a re-anchored pose.
    // (kept between ticks; the agents set last tick cleared -- read for any agent the grid finds)
    if (advancedFlags_.size() != agents_.size()) advancedFlags_.assign(agents_.size(), 0);
    else for (int k : advancedSet_) advancedFlags_[static_cast<std::size_t>(k)] = 0;
    advancedSet_.clear();
    std::vector<uint8_t>& advanced = advancedFlags_;
    phaseMark(phase_.goals);   // clock + signals + sensed build
    computeGaps();
    phaseMark(phase_.gaps);
    int tetherHeldThisTick = 0;   // stranded by the leash, city-wide
    for (int ai : active_) {
        Agent& a = agents_[ai];
        const std::size_t i = static_cast<std::size_t>(ai);
        if (a.playerControlled || a.released) continue;
        // Tethered ghost too far from its physical body: WAIT for it. The plan can
        // never outrun the physics (ADR-0062).
        if (a.moving && a.tethered) {
            Real dx = a.pos.x - a.tetherAnchor.x, dy = a.pos.y - a.tetherAnchor.y;
            if (std::sqrt(dx * dx + dy * dy) > a.tetherLead) {
                a.speed = 0;
                a.state = Agent::State::Waiting;
                ++tetherHeldThisTick;
                continue;
            }
        }
        if (a.moving) { advance(a, dt, gaps_[i], minGaps_[i]); advanced[i] = 1; advancedSet_.push_back(static_cast<int>(i)); }
    }
    // Keep the flags: "did this agent get stepped" is the question a director
    // asks when its agent stands still (ADR-0091).
    advancedLast_ = advanced;
    tetherHeld_ = tetherHeldThisTick;

    phaseMark(phase_.advMove);

    // Reactive pedestrian behaviour (ADR-0061): each walker acts on what it SEES
    // in its vision cone. It steers to one side to go AROUND the neighbours (and
    // the player) it sees ahead — and never reacts to what's behind it. A hard
    // body-overlap floor below is the physical backstop so two people can't occupy
    // the same spot. Deterministic (index order); resets to the sidewalk each
    // step, so the sidestep is a transient lean while someone is in view.
    phaseMark(phase_.advPairs);
    crowdBase_.resize(agents_.size());
    if (crowdHas_.size() != agents_.size()) crowdHas_.assign(agents_.size(), 0);
    else for (int k : street_) crowdHas_[static_cast<std::size_t>(k)] = 0;   // (set only for walkers out on the street)
    for (int ki : street_) {
        Agent& a = agents_[static_cast<std::size_t>(ki)];
        if (!a.moving) { a.state = Agent::State::Resting; a.lateralOffset = 0; a.crowdOffset = Vec2(0, 0); }
    }
    for (int ki : active_) {
        const std::size_t i = static_cast<std::size_t>(ki);
        Agent& a = agents_[i];
        if (!a.moving) continue;
        // Only agents advance() re-anchored this tick may lean: the lean is
        // POS += offset on top of a fresh refreshPose. A tether-held (or host-
        // controlled) ghost skipped advance(), so leaning it again would stack
        // offset on offset — a pinned walker's ghost slid sideways without
        // bound, dragging its body after it through the tether feedback.
        if (!advanced[i]) continue;
        // `travel` is the walker's path direction this step (set by steer()); it
        // leans to `right` of that to go around what it SEES ahead.
        Vec2 travel = a.heading;
        Vec2 rightv(travel.y, -travel.x);

        // THINK on the slow clock, ACT every tick (ADR-0062). The reactive scan
        // (who's ahead, which side do I pass) runs only when this agent's think
        // timer expires — staggered per agent — and its answer is COMMITTED to
        // `leanTarget` + the FSM state until the next think. Re-deciding every
        // tick made walkers flip-flop ("wigging out"); a held decision reads as
        // intent, and the integration below still moves smoothly every tick.
        a.thinkTimer -= dt;
        if (a.thinkTimer <= 0) {
            a.thinkTimer += thinkPeriod_;
            engine::VisionCone cone;
            cone.origin = a.pos; cone.forward = travel;
            cone.range = kPedVisionRange; cone.halfAngleRad = kPedVisionHalfAngle;
            // SENSE -> REMEMBER (ADR-0063): sight the walking neighbours (and the
            // player) into memory at think cadence — successive sightings a think
            // apart are what give each track its velocity estimate.
            engine::SensorVolume sensor;
            sensor.cone = cone;
            // Agent-backed ghosts from the grid (P4.1), ascending, then the
            // host-injected externals — the order the full sensed_ scan used.
            // Slack past the cone range covers a tethered ghost at its anchor.
            grid_.query(a.pos, cone.range + 12.0, queryScratch_);
            for (int gi : queryScratch_) {
                if (gi == static_cast<int>(i)) continue;   // not myself
                if (sensedIndex_[gi] < 0) continue;        // no sensable body
                // Skip player-agent ghosts: the host already injects the live
                // player as an external point — seeing both would double it.
                if (agents_[gi].playerControlled) continue;
                const SensedGhost& g = sensed_[sensedIndex_[gi]];
                if (engine::sees(sensor, g.pos, g.elevation - a.elevation))
                    a.memory.observe(g.id, g.pos, simSeconds_);
            }
            for (const SensedGhost& g : sensed_) {
                if (g.id >= 0) continue;   // agent ghosts handled above
                if (engine::sees(sensor, g.pos, 0.0))
                    a.memory.observe(g.id, g.pos, simSeconds_);
            }
            a.memory.update(simSeconds_);
            // PREDICT -> DECIDE: lean away from where each remembered body will
            // BE shortly (its track velocity run a beat ahead), not where the
            // snapshot last had it — so two approaching walkers part early and
            // smoothly. The answer is COMMITTED (leanTarget) until the next
            // think. Poles are static and eternal: no memory needed.
            Real bias = 0; bool saw = false;
            auto consider = [&](const Vec2& p) {
                if (!engine::sees(cone, p)) return;
                Real fd = engine::forwardDistance(cone, p);
                if (fd <= 0.1 || fd >= kPedVisionRange) return;
                saw = true;
                Real side = (a.pos.x - p.x) * rightv.x + (a.pos.y - p.y) * rightv.y;
                Real w = (kPedVisionRange - fd) / kPedVisionRange;   // nearer -> stronger
                bias += (side >= 0 ? 1.0 : -1.0) * w;                // pass on the side I'm on
            };
            for (const engine::TrackedBody& t : a.memory.tracks()) {
                if (t.confidence < kMemoryActConfidence) continue;
                consider(t.pos + t.vel * kPedAnticipation);
            }
            forObstaclesNear(a.pos, kPedVisionRange + 1.0, [&](const StaticObstacle& o) { consider(o.p); });   // poles, furniture
            a.leanTarget = std::max(Real(-1), std::min(Real(1), bias)) * kPedMaxLateral;
            // Held at a signal (advance() parked us at the kerb, speed 0,
            // Waiting): keep the honest red ring rather than repainting Walking.
            if (!(a.speed == 0 && a.state == Agent::State::Waiting))
                a.state = saw ? Agent::State::Avoiding : Agent::State::Walking;
        }

        // ACT: move the lean toward the COMMITTED target at a bounded RATE
        // (continuous, not a pop): grows while the decision says "step aside",
        // decays back to the path once a think says clear. This is what lets you
        // herd a walker like a boid.
        Real prev = a.lateralOffset;
        Real maxDelta = kPedLateralRate * dt;
        // TOWARD THE ROAD (its side of the walk: Agent::roadSide, + its right) only as far as its walk's kerb allows
        // (Agent::kerbRoom, #314) -- a full 1.6 m lean put a walker stepping round someone 0.6 m into the carriageway
        // (walkersInRoad: "avoiding, in the street it walks along"); toward the buildings, the whole lean -- and where the
        // road side has not the room for the step, it goes round on the buildings' side instead
        const Real towardRoad = a.leanTarget * static_cast<Real>(a.roadSide);
        const Real leanGoal = onStreet(a) && towardRoad > a.kerbRoom
                                  ? -static_cast<Real>(a.roadSide) * std::min(std::fabs(a.leanTarget), kPedMaxLateral)
                                  : a.leanTarget;
        Real delta = std::max(-maxDelta, std::min(maxDelta, leanGoal - prev));
        a.lateralOffset = prev + delta;

        a.pos.x += rightv.x * a.lateralOffset;
        a.pos.y += rightv.y * a.lateralOffset;
        // ...and where the crowd had it last tick (kept below, after the overlap floor)
        crowdBase_[i] = a.pos;
        crowdHas_[i] = 1;
        a.pos = a.pos + a.crowdOffset;
        // Turn to face where it's actually moving (forward walk + the sideways
        // drift), so the body visibly rotates as it steers away. Uses the ped's
        // OWN speed (personality-scaled), not the nominal walk speed.
        Real walk = a.speed > 0.1 ? a.speed : kWalkSpeed * a.speedFactor;
        Vec2 vel(travel.x * walk + rightv.x * (delta / dt),
                 travel.y * walk + rightv.y * (delta / dt));
        Real vl = std::sqrt(vel.x * vel.x + vel.y * vel.y);
        if (vl > 1e-6) a.heading = Vec2(vel.x / vl, vel.y / vl);
    }
    // Hard body-overlap floor: several symmetric relaxation passes so two people
    // (whether or not they saw each other) never interpenetrate.
    phaseMark(phase_.advPop);
    // WHO EACH WALKER CAN TOUCH, gathered ONCE a tick (below: why the radius carries six passes of drift) instead of
    // queried six times. Nothing that picks a neighbour changes between passes -- mode, tier, moving, stepped, seen
    // -- and the grid itself is not re-placed during them, so the passes see the same pairs in the same order. The
    // query was the step: a tower's door holds its hundreds of residents INDOORS in the grid, every walker near it
    // waded through them six times a tick (72 ms of a 112 ms step downtown at 07:48, Glenn: "that's killing the
    // frame rate"); now the indoor ones are dropped once, when the list is made. RT_SOLVER_REQUERY: as before.
    static const bool requery = std::getenv("RT_SOLVER_REQUERY") != nullptr;
    solverFrom_.clear();
    solverNbr_.clear();
    // ...from the walkers OUT ON THE STREET only (the K tier's, moving or standing in view): the indoor residents at a
    // door were most of what every gather waded through (10 ms of a 21 ms step). (Where they stand now: the query
    // radius carries 4 m of slack, a tick of walking is 2 cm.)
    for (int id : pedGridIds_) pedGrid_.remove(id);
    pedGridIds_.clear();
    if (!requery)
        for (int ki : street_) {
            const Agent& b = agents_[static_cast<std::size_t>(ki)];
            pedGrid_.place(ki, b.pos);
            pedGridIds_.push_back(ki);
        }
    for (int ai : active_) {
        const std::size_t i = static_cast<std::size_t>(ai);
        solverFrom_.push_back(static_cast<int>(solverNbr_.size()));
        if (requery) continue;
        const Agent& a = agents_[i];
        if (!a.moving || !advanced[i]) continue;
        pedGrid_.query(a.pos, kPedBodyMin + 4.0, pairScratch_);
        for (int gj : pairScratch_) {
            const std::size_t j = static_cast<std::size_t>(gj);
            if (j == i) continue;
            const Agent& b = agents_[j];
            const bool still = (!b.moving || !advanced[j]) && pedVisible(gj);
            if (!still && (!b.moving || j <= i)) continue;
            solverNbr_.push_back(still ? -1 - gj : gj);   // (still: its own ground, encoded negative)
        }
    }
    solverFrom_.push_back(static_cast<int>(solverNbr_.size()));
    // THE RELAXATION, a pass at a time: run here, and once more after the pushes off poles, the player and cars below
    // (a walker squeezed out of a car's footprint was set down on the walker beside it, up to 1.5 m in a tick, and
    // nothing parted them: metro_pedestrians_walk_and_keep_apart's overlaps)
    auto relaxWalkers = [&](int passes) {
    for (int iter = 0; iter < passes; ++iter)
        for (std::size_t k = 0; k < active_.size(); ++k) {
            const int ai = active_[k];
            Agent& a = agents_[ai];
            const std::size_t i = static_cast<std::size_t>(ai);
            if (!a.moving)
                continue;
            // a walker HELD by its leash this tick (not stepped) stands its ground like anyone standing: nothing
            // re-anchors it, so a shove would stay -- and the walkers round it now keep their spacing, so they press
            if (!advanced[i]) continue;
            // THE BIG ONE. This was `for (j = i + 1; j < agents_.size(); ++j)`
            // -- every agent in the city, per active walker, SIX TIMES a tick,
            // with a sqrt on most of them. Measured: 163,259 us of a 178,607 us
            // step at 50k, i.e. 91% of the whole simulation, and it scales as
            // n^2: 2,588 us at 10k, 163,259 at 50k -- 5x the agents, 63x the
            // cost. It ignores tiers entirely, which is why it got worse with
            // every agent added and why fixing the car pair scan barely moved.
            //
            // grid_ returns a superset sorted ascending and callers apply their
            // own predicates, so skipping j <= i keeps the same pairs in the
            // same order. RADIUS: this loop MOVES agents, so grid positions go
            // stale as it relaxes. Pushes are always apart, but an agent shoved
            // clear of one neighbour can drift toward another, so the radius
            // carries the body minimum plus room for six passes of drift. Pairs
            // beyond it could not have touched anyway -- the push only fires
            // below kPedBodyMin.
            auto touch = [&](int gj, bool still) {
                Agent& b = agents_[static_cast<std::size_t>(gj)];
                Real dx = a.pos.x - b.pos.x, dy = a.pos.y - b.pos.y;
                Real d = std::sqrt(dx * dx + dy * dy);
                if (d > 1e-4 && d < kPedBodyMin) {
                    if (still) {
                        const Real push = kPedBodyMin - d;
                        a.pos.x += dx / d * push; a.pos.y += dy / d * push;
                    } else {
                        Real push = (kPedBodyMin - d) * 0.5;
                        a.pos.x += dx / d * push; a.pos.y += dy / d * push;
                        b.pos.x -= dx / d * push; b.pos.y -= dy / d * push;
                    }
                } else if (d <= 1e-4) {
                    if (still) {
                        a.pos.x += kPedBodyMin;
                    } else {
                        a.pos.x += kPedBodyMin * 0.5;
                        b.pos.x -= kPedBodyMin * 0.5;
                    }
                }
            };
            if (!requery) {
                for (int n = solverFrom_[k]; n < solverFrom_[k + 1]; ++n) {
                    const int e = solverNbr_[static_cast<std::size_t>(n)];
                    touch(e < 0 ? -1 - e : e, e < 0);
                }
                continue;
            }
            grid_.query(a.pos, kPedBodyMin + 4.0, pairScratch_);
            for (int gj : pairScratch_) {
                const std::size_t j = static_cast<std::size_t>(gj);
                if (j == i) continue;
                Agent& b = agents_[j];
                    // A person STANDING or SITTING out on the street holds their ground too: the walker steps round
                // them. (Only walker pairs were separated, so a walker could brush through someone standing at
                // a stop or sitting on a bench -- the one overlapping pair macOS CI caught.) Walker pairs are
                // visited once (j > i); a still body never runs this loop, so it is visited from every walker.
                const bool still = (!b.moving || !advanced[j]) && pedVisible(gj);
                if (!still && (!b.moving || j <= i)) continue;
                touch(gj, still);
            }
        }
    };
    relaxWalkers(6);

    // KEEP THE CROWD'S SPACING: what the floor (and the offset carried in) moved each walker off its path this tick is
    // its crowd offset next tick -- at most 1.5 m, shrinking 0.4 m/s once nobody presses, so it drifts back to its
    // path as the crowd thins rather than hopping there.
    for (int ai : active_) {
        const std::size_t i = static_cast<std::size_t>(ai);
        if (i >= crowdHas_.size() || !crowdHas_[i]) continue;
        Agent& a = agents_[i];
        // not at a kerb (waiting to cross: the crowd's push must never carry it into the road), and sideways no
        // further than the lean's own limit, lean and push together
        // (WAITING at a kerb: the crowd's push must not hold it toward the road, so the offset goes -- but DRAINS, at
        // 3 m/s (half a second for the most it can be), rather than vanishing in one tick: a hop of up to 1.5 m the
        // speed watchdog caught at every crossing)
        if (a.state == Agent::State::Waiting) {
            const Real L0 = a.crowdOffset.length(), drain = 3.0 * dt;
            a.crowdOffset = L0 > drain ? a.crowdOffset * ((L0 - drain) / L0) : Vec2(0, 0);
            continue;
        }
        Vec2 off = a.pos - crowdBase_[i];
        const Vec2 rightv(a.heading.y, -a.heading.x);
        const Real side = dot(off, rightv);
        Real room = std::max(Real(0), kPedMaxLateral - std::fabs(a.lateralOffset));
        if (side * static_cast<Real>(a.roadSide) > 0 && onStreet(a))   // toward the road: what its kerb leaves after the lean
            room = std::max(Real(0), static_cast<Real>(a.kerbRoom) - std::max(Real(0), a.lateralOffset * static_cast<Real>(a.roadSide)));
        if (std::fabs(side) > room) off = off - rightv * (side - std::copysign(room, side));
        Real L = off.length();
        if (L > 1.5) { off = off * (1.5 / L); L = 1.5; }
        const Real shrink = 0.4 * dt;
        a.crowdOffset = L > shrink ? off * ((L - shrink) / L) : Vec2(0, 0);
    }

    // Hard radial push-out from static obstacles (signal poles) and the PLAYER: a
    // walker never ends up standing inside a pole — or brushing through the
    // player (a wider berth, so a near miss reads as a step-around). The cone
    // bias above makes it lean away in advance; this is the physical backstop.
    phaseMark(phase_.advSolver);
    auto clearOfPolesAndPlayer = [&](Agent& a) {
        auto pushOut = [&](const Vec2& o, Real clearance) {
            Real dx = a.pos.x - o.x, dy = a.pos.y - o.y;
            Real d = std::sqrt(dx * dx + dy * dy);
            if (d > 1e-4 && d < clearance) {
                Real push = clearance - d;
                a.pos.x += dx / d * push;
                a.pos.y += dy / d * push;
            } else if (d <= 1e-4) {
                a.pos.x += clearance;        // dead-centre: pick a direction
            }
        };
        forObstaclesNear(a.pos, obstacleMaxClearance_ + 0.5, [&](const StaticObstacle& o) { pushOut(o.p, o.clearance); });
        for (const Vec2& o : externalObstacles_) pushOut(o, kPlayerClearance);
    };
    for (int ki : active_) clearOfPolesAndPlayer(agents_[static_cast<std::size_t>(ki)]);
    // ...a pass at a time with the poles' and the player's berth, which hold last (a hard rule:
    // walkers_keep_a_wide_berth_around_the_player); the two settle together
    for (int pass = 0; pass < 4; ++pass) {
        relaxWalkers(1);
        for (int ki : active_) clearOfPolesAndPlayer(agents_[static_cast<std::size_t>(ki)]);
    }


    phaseMark(phase_.advance);
    phase_.total += std::chrono::duration<double, std::micro>(
                        std::chrono::steady_clock::now() - stepBegin).count();
    ++phase_.steps;
}

// --- three-tier traffic (P4.2) ----------------------------------------------

}  // namespace citysim
