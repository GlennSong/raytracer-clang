#include "city_brain.h"

#include "city_sim.h"
#include "city_trip.h"   // stepsOffThePaths: a walker's last steps off the pavements
#include "brain/brain_driving.h"
#include "brain/brain_planner.h"
#include "brain/brain_states.h"
#include "../../engine/components.h"   // CityMarker, CityResident
#include "../../engine/script_assets.h"
#include "../../log.h"
#ifdef RT_ENABLE_SCRIPTING
#include "../../engine/scripting/script_vm.h"
#include "brain/scripting/brain_lua.h"
#include "scripting/travel_lua.h"
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>

namespace citysim {

using brain::AgentId;
using brain::BrainTime;
using brain::CarRef;
using brain::PlaceRef;
using engine::Vec3;

namespace {

// HOW A PLACE'S HANDLE SAYS WHAT IT IS: the top byte. A marker is its place in the level's list; a kerb space is a
// handle of brain::RoadPlaces; a point is where a car was left in a lane.
constexpr uint32_t kKindMask = 0xFF000000u;
constexpr uint32_t kMarkerTag = 0x00000000u;
constexpr uint32_t kSpaceTag = 0x01000000u;
constexpr uint32_t kPointTag = 0x02000000u;
// The car a test puts in a space (occupyForTest): no vehicle of the sim.
constexpr uint32_t kTestCar = brain::kMaxHandle;
constexpr Real kMillis = 1000.0;
constexpr Real kSecondsPerMinute = 60.0;

std::string fmt(const char* f, double a) {
    char b[64];
    std::snprintf(b, sizeof(b), f, a);
    return b;
}

Vec2 unit(Vec2 v, Vec2 otherwise) {
    const Real len = v.length();
    return len > engine::kSamePoint ? v * (1.0 / len) : otherwise;
}

}  // namespace

// ---- Everything the resident needs, in one place ---------------------------------------------------------------------

struct CityBrain::Impl {
    explicit Impl(CitySim& s) : sim(s) {}

    CitySim& sim;
    const engine::RoadModel* model = nullptr;
    driving::Driving* d = nullptr;
    TravelCatalog travel;
    brain::BehaviourSet behaviours;
    brain::BrainTable table;
    std::vector<BrainMarker> markers;
    std::unique_ptr<brain::RoadPlaces> spaces;
    std::vector<Vec2> points;   // where a car was left in a lane, by the low bits of a point place

    // THE CAR. Its CarRef and its id in the driving are both its vehicle index in the sim (the hand-over's rule).
    enum class CarPhase : uint8_t {
        Standing,         // in a kerb space, or left standing in a lane
        WaitingForRoom,   // in its space, its driver aboard, waiting for the kerb lane to be clear
        EasingOut,        // placed in the kerb lane; drawn sliding out of the space
        OnRoad,           // the driving's
        EasingIn,         // out of the driving; drawn sliding into its space
    };
    struct CarRec {
        int vehicle = -1;
        uint32_t vehicleClass = 0;
        PlaceRef standing;                    // where it stands, or last stood
        AgentId heldBy = brain::kNoAgent;
        bool aboard = false;                  // its holder is in it
        bool onRoad = false;                  // it is placed in the driving
        CarPhase phase = CarPhase::Standing;
        Vec2 easeFromPos, easeFromHeading, easeToPos, easeToHeading;
        Real easeT = 0;
        bool tripWanted = false;              // a trip to begin once it is in the lane
        AgentId tripPerson = brain::kNoAgent;
        PlaceRef tripFrom, tripTo;
    };
    std::vector<CarRec> cars;
    std::map<uint32_t, uint32_t> occupant;   // kerb space (place id) -> the car in it
    std::map<AgentId, uint32_t> held;        // person -> the kerb space (place id) held for them

    // THE PEOPLE the machine has, by its ids.
    enum class WalkPhase : uint8_t { None, Routed, Straight };
    struct Person {
        int agent = -1;
        WalkPhase walk = WalkPhase::None;
        brain::WayId way = brain::kNoId;
        Vec2 target;
        std::vector<Vec2> steps;
        Vec2 rest;   // where the brain last left them standing (kept there while no walk is under way)
    };
    std::vector<Person> people;

    // THE DAY: the markers the resident goes round, and how long they stay at each.
    struct Stop {
        PlaceRef place;
        BrainTime dwell = 0;
    };
    std::vector<Stop> day;
    bool dayOn = false;
    int dayAt = -1;   // the stop they are at or going to (-1: not set out yet)
    brain::IntentKind intentKind = 0;

    CityBrainCounts counts;
    std::vector<brain::Transition> trace;

    // (made in start, in this order; the machine last)
    struct LabWorld;
    struct LabWalk;
    struct LabDrive;
    struct FrontPoints;
    std::unique_ptr<LabWorld> world;
    std::unique_ptr<brain::TripPlanner> planner;
    std::unique_ptr<FrontPoints> fronts;
    std::unique_ptr<brain::DrivingSkill> skill;
    std::unique_ptr<LabWalk> walk;
    std::unique_ptr<LabDrive> drive;
    std::unique_ptr<brain::Machine> machine;

    // ---- the sim's own things -----------------------------------------------------------------------------------------
    Agent& agentOf(AgentId who) { return sim.agents_[static_cast<std::size_t>(people[who].agent)]; }
    SimVehicle& vehicleOf(const CarRec& c) { return sim.vehicles_[static_cast<std::size_t>(c.vehicle)]; }
    CarRec* car(CarRef ref) {
        for (CarRec& c : cars)
            if (c.vehicle >= 0 && static_cast<uint32_t>(c.vehicle) == ref.id) return &c;
        return nullptr;
    }
    const CarRec* car(CarRef ref) const { return const_cast<Impl*>(this)->car(ref); }
    BrainTime now() const { return static_cast<BrainTime>(std::llround(sim.simSeconds_ * kMillis)); }

    // ---- places -------------------------------------------------------------------------------------------------------
    static bool isMarker(PlaceRef p) { return p.valid() && (p.id & kKindMask) == kMarkerTag; }
    static bool isPoint(PlaceRef p) { return p.valid() && (p.id & kKindMask) == kPointTag; }
    bool isSpace(PlaceRef p) const { return spaces && spaces->owns(p); }
    PlaceRef markerPlace(int index) const {
        PlaceRef p;
        if (index >= 0 && index < static_cast<int>(markers.size())) p.id = kMarkerTag | static_cast<uint32_t>(index);
        return p;
    }
    PlaceRef pointPlace(Vec2 at) {
        PlaceRef p;
        points.push_back(at);
        p.id = kPointTag | static_cast<uint32_t>(points.size() - 1);
        return p;
    }
    bool positionOf(PlaceRef place, Vec2& out) const {
        if (isMarker(place)) {
            if (place.id >= markers.size()) return false;
            out = markers[place.id].stand;
            return true;
        }
        if (isSpace(place)) return spaces->positionOf(place, out);
        if (isPoint(place) && (place.id & ~kKindMask) < points.size()) {
            out = points[place.id & ~kKindMask];
            return true;
        }
        return false;
    }
    std::string placeName(PlaceRef place) const {
        if (isMarker(place) && place.id < markers.size()) return markers[place.id].name;
        engine::KerbSpace space;
        if (isSpace(place) && spaces->spaceOf(place, space)) return "space " + model->kerbSpaceName(space);
        Vec2 at;
        if (positionOf(place, at)) return "the point " + fmt("%.1f", at.x) + " " + fmt("%.1f", at.y);
        return "place " + std::to_string(place.id);
    }
    // Where a car in a space stands and faces, and which way its kerb lies (unit, across the road).
    bool spacePose(PlaceRef place, engine::KerbSpacePose& pose, Vec2& kerbward, engine::KerbSpace* spaceOut = nullptr) const {
        engine::KerbSpace space;
        if (!isSpace(place) || !spaces->spaceOf(place, space) || !model->kerbSpacePose(space, pose)) return false;
        const engine::KerbLanePlace lane = model->kerbLaneBeside(space);
        kerbward = lane.valid() ? unit(pose.at - lane.at, engine::rightOf(pose.facing)) : engine::rightOf(pose.facing);
        if (spaceOut) *spaceOut = space;
        return true;
    }
    // Where a person stands beside a car in a space, on the kerb side: to get in, and after getting out.
    bool doorPoint(PlaceRef place, Vec2& out) const {
        engine::KerbSpacePose pose;
        Vec2 kerbward;
        engine::KerbSpace space;
        if (!spacePose(place, pose, kerbward, &space)) return false;
        out = pose.at + kerbward * (space.width * Real(0.5) + kDoorStandOff);
        return true;
    }
    // Where a walk to `place` ends.
    bool walkTarget(PlaceRef place, Vec2& out) const { return doorPoint(place, out) || positionOf(place, out); }
    // The kerb spaces near a place, nearest first on foot.
    std::vector<PlaceRef> spacesFor(PlaceRef door) const {
        std::vector<PlaceRef> out;
        engine::RoadFrontage frontage;
        Vec2 at;
        if (isMarker(door) && door.id < markers.size()) frontage = markers[door.id].frontage;
        else if (positionOf(door, at)) frontage = model->frontageAt(at);
        if (!frontage.street.valid()) return out;
        for (const engine::KerbSpaceNear& found : model->kerbSpacesNear(frontage, kSpaceReach, kSpacesLookedAt)) {
            const PlaceRef place = spaces->placeOf(found.space);
            if (place.valid()) out.push_back(place);
        }
        return out;
    }
    // Could `car`, driven by `who` (kNoAgent: by nobody yet), be left in `space` this moment?
    bool spaceFree(PlaceRef space, CarRef carRef, AgentId who) const {
        const std::map<uint32_t, uint32_t>::const_iterator in = occupant.find(space.id);
        if (in != occupant.end() && in->second != carRef.id) return false;
        for (const std::pair<const AgentId, uint32_t>& h : held)
            if (h.second == space.id && h.first != who) return false;
        return true;
    }

    // ---- getting in and out -------------------------------------------------------------------------------------------
    bool getIn(AgentId who, CarRec& c);
    void getOut(AgentId who, CarRec& c);
    void writeCarPose(CarRec& c, Vec2 pos, Vec2 heading);
    void haltWalker(AgentId who);
    void tickWalks(Real dt);
    void tickCars(Real dt);
    void tickDay();
    bool laneClear(const driving::PathPoint& front, Real length) const;
    void slideIn(CarRec& c, PlaceRef space);
};

// ---- The kerb lane beside a place, as the point a car's FRONT is at ---------------------------------------------------
// RoadPlaces answers the lane point level with the middle of a space; a car standing beside the space has its middle
// there, so its front is half its length further on. Held to the lane path: the whole body on it, and its front no
// nearer the end than kKerbLaneEndMargin.
struct CityBrain::Impl::FrontPoints : brain::KerbLanes {
    explicit FrontPoints(Impl& i) : im(i) {}
    Impl& im;
    Real length = 0;   // the resident's car's class's length
    bool kerbLane(PlaceRef place, driving::PathPoint& out) const override {
        driving::PathPoint mid;
        if (!im.spaces->kerbLane(place, mid) || !im.d->roads() || mid.path.index >= im.d->roads()->paths.size()) return false;
        const Real pathLength = im.d->roads()->paths[mid.path.index].length;
        const Real lowest = length, highest = pathLength - kKerbLaneEndMargin;
        if (engine::strictlyPast(lowest, highest, engine::kSamePoint)) return false;   // the lane is shorter than the car
        out.path = mid.path;
        out.along = std::clamp(mid.along + length * Real(0.5), lowest, highest);
        return true;
    }
};

// ---- The world's answers ---------------------------------------------------------------------------------------------
struct CityBrain::Impl::LabWorld : brain::World {
    explicit LabWorld(Impl& i) : im(i) {}
    Impl& im;

    BrainTime now() const override { return im.now(); }
    bool positionOf(PlaceRef place, Vec2& out) const override { return im.positionOf(place, out); }
    bool bodyPosition(AgentId who, Vec2& out) const override {
        if (who >= im.people.size()) return false;
        out = im.agentOf(who).pos;
        return true;
    }
    bool carStanding(CarRef ref, PlaceRef& out) const override {
        const CarRec* c = im.car(ref);
        if (!c || !c->standing.valid()) return false;
        out = c->standing;
        return true;
    }
    bool carFreeFor(CarRef ref, AgentId who) const override {
        const CarRec* c = im.car(ref);
        return c && (c->heldBy == brain::kNoAgent || c->heldBy == who);
    }
    bool claimCar(CarRef ref, AgentId who) override {
        CarRec* c = im.car(ref);
        if (!c || (c->heldBy != brain::kNoAgent && c->heldBy != who)) return false;
        c->heldBy = who;
        return true;
    }
    void releaseCar(CarRef ref, AgentId who) override {
        CarRec* c = im.car(ref);
        if (!c || c->heldBy != who) return;
        im.getOut(who, *c);
        c->heldBy = brain::kNoAgent;
        c->tripWanted = false;
        if (c->phase == CarPhase::WaitingForRoom) c->phase = CarPhase::Standing;
    }
    void leaveCar(CarRef ref, AgentId who) override {
        CarRec* c = im.car(ref);
        if (!c || c->heldBy != who) return;
        // on the road it stays where it is, placed and undriven, and stands at a place of its own
        if (c->onRoad) c->standing = im.pointPlace(im.vehicleOf(*c).pos);
        im.getOut(who, *c);
        c->heldBy = brain::kNoAgent;
        c->tripWanted = false;
        if (c->phase != CarPhase::EasingIn) c->phase = CarPhase::Standing;
    }
    void parkCar(CarRef ref, AgentId who, PlaceRef space) override {
        CarRec* c = im.car(ref);
        if (!c) return;
        c->standing = space;
        im.occupant[space.id] = ref.id;
        im.getOut(who, *c);
        if (c->heldBy == who) c->heldBy = brain::kNoAgent;
        im.held.erase(who);
    }
    bool spaceNear(PlaceRef door, CarRef ref, PlaceRef& out) const override {
        for (const PlaceRef space : im.spacesFor(door))
            if (im.spaceFree(space, ref, brain::kNoAgent)) {
                out = space;
                return true;
            }
        return false;
    }
    bool takeSpace(PlaceRef space, CarRef ref) override {
        CarRec* c = im.car(ref);
        if (!c || !im.isSpace(space) || !im.spaceFree(space, ref, c->heldBy)) return false;
        for (std::map<uint32_t, uint32_t>::iterator it = im.occupant.begin(); it != im.occupant.end();)
            it = it->second == ref.id ? im.occupant.erase(it) : std::next(it);
        im.occupant[space.id] = ref.id;
        im.slideIn(*c, space);
        return true;
    }
    void holdSpace(PlaceRef space, AgentId who) override {
        if (im.isSpace(space)) im.held[who] = space.id;
    }
    void dropHold(AgentId who) override { im.held.erase(who); }
    bool nextSpace(PlaceRef door, CarRef ref, AgentId who, PlaceRef taken, PlaceRef& out) const override {
        for (const PlaceRef space : im.spacesFor(door))
            if (space != taken && im.spaceFree(space, ref, who)) {
                out = space;
                ++im.counts.nextSpaces;
                return true;
            }
        return false;
    }
};

// ---- Walking: the sim's own -------------------------------------------------------------------------------------------
struct CityBrain::Impl::LabWalk : brain::WalkSkill {
    explicit LabWalk(Impl& i) : im(i) {}
    Impl& im;

    bool walk(AgentId who, brain::WayId way, Vec2, PlaceRef to, Real) override {
        if (who >= im.people.size() || way >= im.travel.modes.size()) return false;
        Person& p = im.people[who];
        Agent& a = im.agentOf(who);
        Vec2 target;
        if (a.mode != Agent::Mode::Pedestrian || !im.walkTarget(to, target)) return false;
        const TravelMode& mode = im.travel.modes[way];
        // at the way's speed, and on its board where the way is named as one of the things a walker can ride
        a.directedPace = static_cast<float>(mode.speed / (kWalkSpeed * std::max(Real(0.1), static_cast<Real>(a.speedFactor))));
        a.directedRide = Ride::None;
        for (const Ride r : {Ride::Skateboard, Ride::Scooter})
            if (mode.name == rideName(r)) a.directedRide = r;
        a.indoors = false;
        p.way = way;
        p.target = target;
        p.steps.clear();
        if (engine::atOrBefore((target - a.pos).length(), kBoardReach, engine::kSamePoint)) {
            p.walk = WalkPhase::Straight;   // a few steps: off the pavements and straight, as the trip lab's
        } else if (im.sim.sendAgentTo(p.agent, target)) {
            p.walk = WalkPhase::Routed;
        } else {
            a.directedPace = 0;
            a.directedRide = Ride::None;
            return false;
        }
        ++im.counts.walksBegun;
        return true;
    }
    void cancel(AgentId who) override {
        if (who < im.people.size()) im.haltWalker(who);
    }
};

void CityBrain::Impl::haltWalker(AgentId who) {
    Person& p = people[who];
    p.walk = WalkPhase::None;
    p.steps.clear();
    Agent& a = agentOf(who);
    a.directedPace = 0;
    a.directedRide = Ride::None;
    if (a.mode != Agent::Mode::Pedestrian) return;
    a.moving = false;
    a.speed = 0;
    a.route = engine::Route{};
    a.leg = 0;
    a.distOnLeg = 0;
    a.stopAtDist = -1;
    a.state = Agent::State::Resting;
    p.rest = a.pos;
}

void CityBrain::Impl::tickWalks(Real dt) {
    for (AgentId who = 0; who < people.size(); ++who) {
        Person& p = people[who];
        Agent& a = agentOf(who);
        if (p.walk == WalkPhase::None) {
            // STANDING WHERE THE BRAIN LEFT THEM. The sim places its whole population again whenever its clock is set
            // (the viewer opening the level at its hour, a `daynight` jump: CitySim::seedFromSchedule), and that would
            // carry the resident off to wherever a day they do not keep has them. They are the brain's to move.
            if (a.mode == Agent::Mode::Pedestrian &&
                (a.moving || a.indoors || !engine::approxEqual((a.pos - p.rest).length(), 0, engine::kSamePoint))) {
                a.pos = p.rest;
                a.tickFromPos = p.rest;
                a.moving = false;
                a.indoors = false;
                a.speed = 0;
                a.route = engine::Route{};
                a.wakeAt = -1;
                a.state = Agent::State::Resting;
                sim.grid_.place(p.agent, p.rest);
            }
            continue;
        }
        const brain::StateRow& state = table.states[machine->agent(who).state];
        bool arrived = false, blocked = false;
        if (p.walk == WalkPhase::Routed) {
            if (a.moving) continue;
            // the pavements have brought them as near as they go: the last steps are walked straight
            const Real off = (p.target - a.pos).length();
            if (engine::atOrBefore(off, kWalkArrivedWithin, engine::kSamePoint)) arrived = true;
            else if (engine::atOrBefore(off, kBoardReach, engine::kSamePoint)) p.walk = WalkPhase::Straight;
            else blocked = true;
        }
        if (p.walk == WalkPhase::Straight && !arrived && !blocked) {
            if (p.steps.empty()) p.steps = stepsOffThePaths(sim, p.agent, a.pos, p.target);
            if (p.steps.empty() || sim.stepAgentToward(p.agent, p.steps.front(), travel.modes[p.way].speed, dt)) {
                if (!p.steps.empty()) p.steps.erase(p.steps.begin());
                arrived = p.steps.empty();
            }
        }
        if (!arrived && !blocked) continue;
        haltWalker(who);
        if (arrived) {
            ++counts.walksArrived;
            machine->post(who, state.clock);        // the event the walking state's skill ends it by
        } else {
            ++counts.walksBlocked;
            LOG_WARN << "[brain] person " << who << ": the pavements end " << fmt("%.1f m", (p.target - a.pos).length())
                     << " from where the walk ends; reported as blocked";
            machine->post(who, state.raises[0]);    // ...and the one it says it cannot be done by
        }
    }
}

// ---- Driving: the brain's DrivingSkill, with a real car round it -------------------------------------------------------
struct CityBrain::Impl::LabDrive : brain::DriveSkill {
    explicit LabDrive(Impl& i) : im(i) {}
    Impl& im;

    // The person who has the car gets in. The car stays in its space until its trip begins.
    bool pullOut(CarRef ref, PlaceRef space) override {
        CarRec* c = im.car(ref);
        if (!c) return false;
        if (c->heldBy != brain::kNoAgent && !c->aboard && !im.getIn(c->heldBy, *c)) return false;
        if (c->onRoad) return true;   // left in a lane by an earlier trip: it sets off from there
        driving::PathPoint front;
        return im.fronts->kerbLane(space, front);
    }
    bool beginTrip(CarRef ref, AgentId person, PlaceRef fromSpace, PlaceRef toSpace, Real) override {
        CarRec* c = im.car(ref);
        driving::PathPoint to;
        if (!c || !im.fronts->kerbLane(toSpace, to)) return false;
        c->tripPerson = person;
        c->tripFrom = fromSpace;
        c->tripTo = toSpace;
        if (c->onRoad && c->phase != CarPhase::EasingOut) {
            c->phase = CarPhase::OnRoad;
            c->tripWanted = false;
            if (!im.skill->beginTrip(ref, person, fromSpace, toSpace, 0)) return false;
            ++im.counts.tripsBegun;
            return true;
        }
        // in its space: it is placed in the kerb lane when the lane is clear, eased out, and the trip begins then
        c->tripWanted = true;
        if (c->phase == CarPhase::Standing || c->phase == CarPhase::EasingIn) c->phase = CarPhase::WaitingForRoom;
        return true;
    }
    void endTrip(CarRef ref) override {
        CarRec* c = im.car(ref);
        if (!c) return;
        c->tripWanted = false;
        if (c->phase == CarPhase::WaitingForRoom) c->phase = CarPhase::Standing;
        im.skill->endTrip(ref);
    }
    // The car stands in `space` from now: off the road if it was still on it, and exactly where a car in the space
    // stands (the slide in, if one was under way, is over).
    void park(CarRef ref, PlaceRef space) override {
        CarRec* c = im.car(ref);
        if (!c) return;
        if (c->onRoad) {
            im.skill->endTrip(ref);
            im.skill->park(ref, space);
            c->onRoad = im.d->car(driving::CarId(ref.id)) != nullptr;
        }
        engine::KerbSpacePose pose;
        Vec2 kerbward;
        if (im.spacePose(space, pose, kerbward)) im.writeCarPose(*c, pose.at, pose.facing);
        c->phase = CarPhase::Standing;
        c->tripWanted = false;
    }
};

bool CityBrain::Impl::getIn(AgentId who, CarRec& c) {
    const int agent = people[who].agent;
    if (!sim.boardVehicle(agent, c.vehicle)) return false;
    // a guest of the hand-over: the old model keeps away, and on the road the car's place is copied into the agent
    sim.drivingHost_.seatGuest(agent, driving::CarId(static_cast<uint32_t>(c.vehicle)));
    sim.grid_.remove(agent);
    c.aboard = true;
    ++counts.boarded;
    return true;
}

void CityBrain::Impl::getOut(AgentId who, CarRec& c) {
    if (!c.aboard || who >= people.size()) return;
    const int agent = people[who].agent;
    Agent& a = agentOf(who);
    SimVehicle& v = vehicleOf(c);
    sim.drivingHost_.unseatGuest(agent);
    // the car is left exactly where it stands; the person is out on the kerb side
    a.pos = v.pos;
    a.heading = v.heading;
    a.moving = false;
    sim.alightVehicle(agent);
    Vec2 door;
    if (!c.onRoad && doorPoint(c.standing, door)) {
        a.pos = door;
        a.tickFromPos = door;
        sim.grid_.place(agent, door);
    }
    c.aboard = false;
    people[who].rest = a.pos;
    ++counts.alighted;
}

void CityBrain::Impl::writeCarPose(CarRec& c, Vec2 pos, Vec2 heading) {
    SimVehicle& v = vehicleOf(c);
    v.pos = pos;
    v.heading = heading;
    if (c.aboard && c.heldBy != brain::kNoAgent) {
        Agent& a = agentOf(c.heldBy);
        a.tickFromPos = a.pos;
        a.tickFromHeading = a.heading;
        a.pos = pos;
        a.heading = heading;
    } else {
        sim.parkedGrid_.place(c.vehicle, pos);
    }
}

bool CityBrain::Impl::laneClear(const driving::PathPoint& front, Real length) const {
    const Real pathLength = d->roads()->paths[front.path.index].length;
    const Real from = std::max(Real(0), front.along - length - kPullOutRoomBehind);
    const Real to = std::min(pathLength, front.along + kPullOutRoomAhead);
    return driving::carsOnStretch(d->pathLists(), front.path, from, to).empty();
}

// The space is the car's: it leaves the driving and is drawn sliding in from where it stands in the lane.
void CityBrain::Impl::slideIn(CarRec& c, PlaceRef space) {
    engine::KerbSpacePose pose;
    Vec2 kerbward;
    if (!spacePose(space, pose, kerbward)) return;
    const driving::CarId id(static_cast<uint32_t>(c.vehicle));
    if (c.onRoad) {
        CarRef ref;
        ref.id = id.index;
        skill->endTrip(ref);   // (harmless: the trip ended when the driving reported "parked")
        d->removeCar(id);
        c.onRoad = d->car(id) != nullptr;
    }
    const SimVehicle& v = vehicleOf(c);
    c.easeFromPos = v.pos;
    c.easeFromHeading = v.heading;
    c.easeToPos = pose.at;
    c.easeToHeading = pose.facing;
    c.easeT = 0;
    c.phase = CarPhase::EasingIn;
    c.standing = space;
    ++counts.slidIn;
}

void CityBrain::Impl::tickCars(Real dt) {
    for (CarRec& c : cars) {
        const driving::CarId id(static_cast<uint32_t>(c.vehicle));
        CarRef ref;
        ref.id = id.index;
        if (c.phase == CarPhase::WaitingForRoom) {
            driving::PathPoint front;
            const Real length = d->vehicleClasses().at(c.vehicleClass).length;
            if (!fronts->kerbLane(c.standing, front)) continue;
            if (!laneClear(front, length)) {
                ++counts.waitedForRoom;
            } else if (d->placeCar(id, c.vehicleClass, front)) {
                c.onRoad = true;
                occupant.erase(c.standing.id);
                const SimVehicle& v = vehicleOf(c);
                c.easeFromPos = v.pos;
                c.easeFromHeading = v.heading;
                // where the driving has it: the middle of its body and the way it points (as the hand-over copies it)
                const driving::CarPose pose = driving::carPose(*d->roads(), d->vehicleClasses(), d->ruleSet(), *d->car(id));
                const Vec3 mid = (pose.front + pose.rear) * 0.5;
                c.easeToPos = Vec2(mid.x, mid.z);
                c.easeToHeading = unit(Vec2(pose.body.x, pose.body.z), v.heading);
                c.easeT = 0;
                c.phase = CarPhase::EasingOut;
                ++counts.pulledOut;
            }
        }
        if (c.phase == CarPhase::EasingOut || c.phase == CarPhase::EasingIn) {
            c.easeT = std::min(kKerbEaseSeconds, c.easeT + dt);
            const Real s = engine::smoothstep01(c.easeT / kKerbEaseSeconds);
            writeCarPose(c, c.easeFromPos + (c.easeToPos - c.easeFromPos) * s,
                         unit(c.easeFromHeading + (c.easeToHeading - c.easeFromHeading) * s, c.easeToHeading));
            if (engine::atOrPast(c.easeT, kKerbEaseSeconds, engine::kSamePoint)) {
                if (c.phase == CarPhase::EasingIn) {
                    c.phase = CarPhase::Standing;
                } else {
                    c.phase = CarPhase::OnRoad;
                    if (c.tripWanted) {
                        c.tripWanted = false;
                        // (no route: the driving queues its own "no route" for the person, which the skill passes on)
                        if (skill->beginTrip(ref, c.tripPerson, c.tripFrom, c.tripTo, 0)) ++counts.tripsBegun;
                    }
                }
            }
        } else if (c.aboard && !c.onRoad) {
            // sitting in the car in its space: the person is where the car is
            const SimVehicle& v = vehicleOf(c);
            writeCarPose(c, v.pos, v.heading);
            Agent& a = agentOf(c.heldBy);
            a.moving = false;
            a.speed = 0;
        }
    }
}

// THE DAY TABLE: once the person has been where they wanted to be for that stop's minutes, they want to be at the next.
void CityBrain::Impl::tickDay() {
    if (!dayOn || day.empty() || people.empty()) return;
    const brain::AgentBrain& a = machine->agent(0);
    if (a.travelling() || a.state != table.intents[a.intentKind].arrive) return;
    const std::size_t at = dayAt < 0 ? day.size() - 1 : static_cast<std::size_t>(dayAt);
    if (now() - a.stateEntered < day[at].dwell) return;
    dayAt = static_cast<int>((at + 1) % day.size());
    machine->setIntent(0, intentKind, day[static_cast<std::size_t>(dayAt)].place);
}

// ---- CityBrain ----------------------------------------------------------------------------------------------------------

CityBrain::CityBrain(CitySim& sim) : impl_(std::make_unique<Impl>(sim)) {}
CityBrain::~CityBrain() = default;

bool CityBrain::start(const std::vector<engine::CityMarker>& markerSpecs, const engine::CityResident& resident,
                      const std::vector<int>& classOfSlot, std::string* report) {
    Impl& im = *impl_;
    CitySim& sim = im.sim;
    const auto refuse = [&](const std::string& why) {
        if (report) *report = why;
        return false;
    };
    if (!sim.drivingV2_ || !sim.drivingV2_->roads() || !sim.drivingV2_->hasRules() || !sim.drivingHost_.active())
        return refuse("the new driving does not run on this level (citysim.driving \"v2\" with a rule set)");
    if (!sim.nav_) return refuse("the city has no street map");
    im.d = &*sim.drivingV2_;
    im.model = im.d->model();

    // THE DATA: the ways of travelling and the states
#ifdef RT_ENABLE_SCRIPTING
    {
        const std::string code = engine::loadScriptCode("travel.lua", "");
        engine::ScriptVM vm;
        std::string err;
        if (code.empty() || !vm.doString(code, &err) || !engine::loadTravelCatalog(vm, im.travel, &err))
            return refuse("travel.lua: " + (code.empty() ? std::string("not found") : err));
    }
    brain::BrainSpec spec;
    {
        const std::string code = engine::loadScriptCode("brain.lua", "");
        engine::ScriptVM vm;
        std::string err;
        if (code.empty() || !vm.doString(code, &err) || !brain::loadBrainSpec(vm, spec, &err))
            return refuse("brain.lua: " + (code.empty() ? std::string("not found") : err));
    }
    std::string err;
    brain::registerBehaviours(im.behaviours);
    if (!brain::compileBrainTable(spec, im.behaviours, im.table, &err) || !brain::bindWays(im.travel, im.table, &err))
        return refuse(err);
#else
    return refuse("the brain's states and the ways of travelling are Lua files: this build has no scripting");
#endif

    // THE MARKERS, each resolved against the road model
    std::string left;
    for (const engine::CityMarker& spec : markerSpecs) {
        BrainMarker m;
        m.name = spec.name;
        m.stated = spec.stated;
        if (spec.stated) {
            for (std::size_t s = 0; s < im.model->streets.size(); ++s)
                if (im.model->streets[s].id == spec.street)
                    m.frontage = im.model->frontageOn(engine::ModelStreetId(static_cast<uint32_t>(s)), spec.station, spec.dir);
        } else {
            m.frontage = im.model->frontageAt(Vec2(spec.x, spec.z));
        }
        if (!m.frontage.valid() || !m.frontage.street.valid() || !im.model->sidewalkBeside(m.frontage, m.stand)) {
            left += (left.empty() ? "" : ", ") + spec.name;
            continue;
        }
        im.markers.push_back(m);
    }
    if (!left.empty())
        LOG_WARN << "[brain] markers left out (no such street, or no sidewalk beside it): " << left;
    im.spaces = std::make_unique<brain::RoadPlaces>(*im.model, kSpaceTag);
    std::string line = std::to_string(im.markers.size()) + " markers";
    if (!resident.present) {
        if (report) *report = line + ", no resident";
        return true;
    }

    // THE RESIDENT: the first walker the level made
    const int home = markerNamed(resident.home);
    if (home < 0) return refuse("resident.home \"" + resident.home + "\" is not one of the level's markers");
    int agent = -1;
    for (std::size_t i = 0; i < sim.agents_.size() && agent < 0; ++i) {
        const Agent& a = sim.agents_[i];
        if (a.mode == Agent::Mode::Pedestrian && !a.released && !a.playerControlled) agent = static_cast<int>(i);
    }
    if (agent < 0) return refuse("the level has no walker to be the resident (citysim.pedestrians must be at least 1)");
    const int intent = im.table.intent(resident.intent);
    if (intent < 0) return refuse("resident.intent \"" + resident.intent + "\" is not an intent of brain.lua");
    im.intentKind = static_cast<brain::IntentKind>(intent);
    uint8_t traits = 0;
    for (const std::string& name : resident.traits) {
        const int t = im.travel.trait(name);
        if (t < 0) return refuse("resident.traits: \"" + name + "\" is not a trait of travel.lua");
        traits = static_cast<uint8_t>(traits | (1u << t));
    }
    const PlaceRef homePlace = im.markerPlace(home);

    im.world = std::make_unique<Impl::LabWorld>(im);
    im.planner = std::make_unique<brain::TripPlanner>(*sim.nav_, im.travel, *im.world);
    im.fronts = std::make_unique<Impl::FrontPoints>(im);

    // THEIR CAR, standing in the kerb space nearest home on foot
    CarRef carRef;
    if (resident.car) {
        const std::vector<PlaceRef> near = im.spacesFor(homePlace);
        if (near.empty()) return refuse("no kerb parking space within " + fmt("%.0f m", kSpaceReach) + " on foot of the home marker");
        engine::KerbSpacePose pose;
        Vec2 kerbward;
        if (!im.spacePose(near.front(), pose, kerbward)) return refuse("the home marker's nearest kerb space has no place on the road");
        const int slot = sim.ambientSlotFor(static_cast<int>(sim.vehicles_.size()));
        const VehicleBody& body = sim.fleetBody(slot);
        const int vehicle = sim.addStationedVehicle(body.type, pose.at, pose.facing, -1e30);
        if (vehicle < 0) return refuse("the level's fleet has no body for the resident's car");
        SimVehicle& v = sim.vehicles_[static_cast<std::size_t>(vehicle)];
        v.slot = slot;   // drawn as this slot parked and driven (CitySim::ambientSlotFor of its index)
        v.length = body.length;
        v.width = body.width;
        v.height = body.height;
        Impl::CarRec c;
        c.vehicle = vehicle;
        const int cls = slot >= 0 && slot < static_cast<int>(classOfSlot.size()) ? classOfSlot[static_cast<std::size_t>(slot)] : -1;
        c.vehicleClass = cls >= 0 && cls < static_cast<int>(im.d->vehicleClasses().size()) ? static_cast<uint32_t>(cls) : 0u;
        c.standing = near.front();
        im.cars.push_back(c);
        im.occupant[c.standing.id] = static_cast<uint32_t>(vehicle);
        carRef.id = static_cast<uint32_t>(vehicle);
        im.fronts->length = im.d->vehicleClasses().at(c.vehicleClass).length;
        im.skill = std::make_unique<brain::DrivingSkill>(*im.d, *im.fronts, c.vehicleClass);
        line += "; the resident's car (vehicle " + std::to_string(vehicle) + ", a " + im.d->vehicleClasses().at(c.vehicleClass).name +
                ") stands in " + im.placeName(c.standing);
    } else {
        im.skill = std::make_unique<brain::DrivingSkill>(*im.d, *im.fronts, 0);
    }
    if (!im.skill->bindEvents(im.table, &err)) return refuse(err);
    im.walk = std::make_unique<Impl::LabWalk>(im);
    im.drive = std::make_unique<Impl::LabDrive>(im);
    im.machine = std::make_unique<brain::Machine>(im.table, *im.world, *im.planner, im.walk.get(), im.drive.get());
    im.machine->traceTo(&im.trace);

    // the person, standing at home, the brain's to move from now on
    {
        Agent& a = sim.agents_[static_cast<std::size_t>(agent)];
        sim.setAgentDirected(agent, true);
        sim.pinAgent(agent, true);
        a.pos = im.markers[static_cast<std::size_t>(home)].stand;
        a.tickFromPos = a.pos;
        a.heading = unit(im.markers[static_cast<std::size_t>(home)].frontage.on - a.pos, a.heading);
        a.tickFromHeading = a.heading;
        a.indoors = false;
        a.moving = false;
        a.speed = 0;
        a.route = engine::Route{};
        a.wakeAt = -1;
        a.state = Agent::State::Resting;
        a.car = carRef.valid() ? static_cast<int>(carRef.id) : -1;
        sim.grid_.place(agent, a.pos);
    }
    Impl::Person person;
    person.agent = agent;
    person.rest = im.markers[static_cast<std::size_t>(home)].stand;
    im.people.push_back(person);
    const AgentId who = im.machine->addAgent(homePlace, im.intentKind, traits, carRef, brain::Detail::Near);
    im.trace.push_back(brain::Transition{who, im.machine->agent(who).state, im.now()});

    // THEIR DAY
    for (const engine::CityResident::Stop& stop : resident.day) {
        const int at = markerNamed(stop.at);
        if (at < 0) {
            LOG_WARN << "[brain] resident.day: \"" << stop.at << "\" is not one of the level's markers -- left out";
            continue;
        }
        Impl::Stop s;
        s.place = im.markerPlace(at);
        s.dwell = static_cast<BrainTime>(std::llround(stop.minutes * kSecondsPerMinute * kMillis));
        im.day.push_back(s);
    }
    im.dayOn = !im.day.empty();
    line += "; the resident is the sim's agent " + std::to_string(agent) + ", at home at \"" + resident.home + "\"" +
            (im.day.empty() ? std::string(", no day table") : ", a day of " + std::to_string(im.day.size()) + " stops");
    if (report) *report = line;
    return true;
}

void CityBrain::tick(Real dt) {
    Impl& im = *impl_;
    if (!im.machine) return;
    // the driving's reports for the resident's car, taken from the hand-over (which drains the driving's events)
    const std::vector<driving::Event> events = im.sim.drivingHost_.takeGuestEvents();
    for (const driving::Event& e : events)
        if (e.kind == driving::EventKind::Parked) ++im.counts.parkedReports;
    im.skill->report(*im.machine, events);
    im.tickWalks(dt);
    im.machine->tick();
    im.tickCars(dt);
    im.tickDay();
}

const std::vector<BrainMarker>& CityBrain::markers() const { return impl_->markers; }

int CityBrain::markerNamed(const std::string& name) const {
    for (std::size_t i = 0; i < impl_->markers.size(); ++i)
        if (impl_->markers[i].name == name) return static_cast<int>(i);
    return -1;
}

bool CityBrain::hasResident() const { return impl_->machine != nullptr && !impl_->people.empty(); }
int CityBrain::residentAgent() const { return hasResident() ? impl_->people[0].agent : -1; }
int CityBrain::residentVehicle() const { return impl_->cars.empty() ? -1 : impl_->cars[0].vehicle; }
const std::vector<brain::Transition>& CityBrain::trace() const { return impl_->trace; }
const brain::Machine* CityBrain::machine() const { return impl_->machine.get(); }
const CityBrainCounts& CityBrain::counts() const { return impl_->counts; }
brain::BrainTime CityBrain::now() const { return impl_->now(); }
bool CityBrain::dayOn() const { return impl_->dayOn; }
void CityBrain::setDay(bool on) { impl_->dayOn = on && !impl_->day.empty(); }

std::string CityBrain::stateName() const {
    return hasResident() ? impl_->table.states[impl_->machine->agent(0).state].name : std::string();
}

std::string CityBrain::planText() const {
    if (!hasResident()) return "";
    const Impl& im = *impl_;
    const brain::AgentBrain& a = im.machine->agent(0);
    std::string s;
    for (int i = 0; i < a.legCount; ++i) {
        PlaceRef end;
        end.id = a.legEnd[i];
        const brain::WayId way = a.legWay(i);
        s += (i ? ", " : "") + (way < im.table.ways.size() ? im.table.ways[way].name : std::string("?")) + ">" + im.placeName(end);
    }
    return s;
}

std::string CityBrain::traceText() const {
    const Impl& im = *impl_;
    std::string s;
    for (const brain::Transition& t : im.trace)
        s += (s.empty() ? "" : " ") + im.table.states[t.state].name + "@" + std::to_string(t.at);
    return s;
}

std::string CityBrain::describe() const {
    const Impl& im = *impl_;
    if (!hasResident()) return std::to_string(im.markers.size()) + " markers, no resident with the agent brain on this level";
    const brain::AgentBrain& a = im.machine->agent(0);
    const Agent& body = im.sim.agents_[static_cast<std::size_t>(im.people[0].agent)];
    std::string s = im.machine->describe(0);
    // the places it names, in words
    std::vector<uint32_t> named;
    named.push_back(a.intentTarget);
    for (int i = 0; i < a.legCount; ++i) named.push_back(a.legEnd[i]);
    std::sort(named.begin(), named.end());
    named.erase(std::unique(named.begin(), named.end()), named.end());
    s += " | places:";
    for (const uint32_t id : named) {
        PlaceRef p;
        p.id = id;
        s += " " + std::to_string(id) + " = " + im.placeName(p) + ";";
    }
    s += " | the person (sim agent " + std::to_string(im.people[0].agent) + ") is at " + fmt("%.1f", body.pos.x) + " " +
         fmt("%.1f", body.pos.y) + (body.mode == Agent::Mode::Driver ? " in the car" : " on foot");
    if (!im.cars.empty()) {
        const Impl::CarRec& c = im.cars[0];
        static const char* const kPhase[] = {"standing", "waiting in its space for the kerb lane to be clear",
                                             "sliding out of its space", "on the road", "sliding into its space"};
        s += " | their car (vehicle " + std::to_string(c.vehicle) + ") is " + kPhase[static_cast<int>(c.phase)] +
             (c.phase == Impl::CarPhase::OnRoad && !c.aboard ? " (left there)" : "") + ", " +
             (c.onRoad ? "last in " : "in ") + im.placeName(c.standing);
        if (const driving::Car* onRoad = im.d->car(driving::CarId(static_cast<uint32_t>(c.vehicle))))
            s += ", " + fmt("%.1f m/s", onRoad->speed);
    }
    const std::map<AgentId, uint32_t>::const_iterator h = im.held.find(0);
    if (h != im.held.end()) {
        PlaceRef p;
        p.id = h->second;
        s += " | held for them: " + im.placeName(p);
    }
    s += std::string(" | day table ") + (im.day.empty() ? "none" : im.dayOn ? "on" : "off (brain day on)");
    uint8_t bit = 0;
    std::string has;
    for (const std::string& name : im.travel.traits) {
        if ((a.traits >> bit) & 1u) has += (has.empty() ? "" : ", ") + name;
        ++bit;
    }
    s += " | traits: " + (has.empty() ? std::string("none") : has);
    if (im.machine->counts().errors > 0) s += std::string(" | last error: ") + im.machine->lastError();
    if (!im.planner->lastReason().empty()) s += " | planned because: " + im.planner->lastReason();
    return s;
}

std::vector<BrainPlanLeg> CityBrain::planLegs() const {
    std::vector<BrainPlanLeg> out;
    if (!hasResident()) return out;
    const Impl& im = *impl_;
    const brain::AgentBrain& a = im.machine->agent(0);
    Vec2 from(brain::fromCentimetres(a.originX), brain::fromCentimetres(a.originZ));
    for (int i = 0; i < a.legCount; ++i) {
        BrainPlanLeg leg;
        PlaceRef end;
        end.id = a.legEnd[i];
        leg.from = from;
        if (!im.walkTarget(end, leg.to)) leg.to = from;
        const brain::WayId way = a.legWay(i);
        if (way < im.travel.modes.size()) {
            leg.colour = im.travel.modes[way].colour;
            leg.way = im.travel.modes[way].name;
        }
        leg.underWay = i == a.legNow;
        leg.done = i < a.legNow;
        out.push_back(leg);
        from = leg.to;
    }
    return out;
}

bool CityBrain::goTo(const std::string& marker, std::string* why) {
    Impl& im = *impl_;
    if (!hasResident()) {
        if (why) *why = "this level has no resident with the agent brain";
        return false;
    }
    const int at = markerNamed(marker);
    if (at < 0) {
        std::string names;
        for (const BrainMarker& m : im.markers) names += (names.empty() ? "" : ", ") + m.name;
        if (why) *why = "no marker \"" + marker + "\" (the markers: " + names + ")";
        return false;
    }
    im.dayOn = false;
    im.machine->setIntent(0, im.intentKind, im.markerPlace(at));
    return true;
}

bool CityBrain::setTrait(const std::string& name, bool on, std::string* why) {
    Impl& im = *impl_;
    const int t = im.travel.trait(name);
    if (!hasResident() || t < 0) {
        std::string names;
        for (const std::string& n : im.travel.traits) names += (names.empty() ? "" : ", ") + n;
        if (why) *why = !hasResident() ? "this level has no resident with the agent brain"
                                       : "no trait \"" + name + "\" in travel.lua (the traits: " + names + ")";
        return false;
    }
    im.machine->setTraits(0, static_cast<uint8_t>(on ? im.machine->agent(0).traits | (1u << t)
                                                    : im.machine->agent(0).traits & ~(1u << t)));
    return true;
}

engine::KerbSpace CityBrain::carSpace() const {
    engine::KerbSpace space;
    const Impl& im = *impl_;
    if (!im.cars.empty() && !im.cars[0].onRoad && im.isSpace(im.cars[0].standing)) im.spaces->spaceOf(im.cars[0].standing, space);
    return space;
}

std::string CityBrain::carSpaceName() const {
    const engine::KerbSpace space = carSpace();
    return space.valid() ? impl_->model->kerbSpaceName(space) : std::string();
}

engine::KerbSpace CityBrain::heldSpace() const {
    engine::KerbSpace space;
    const std::map<AgentId, uint32_t>::const_iterator h = impl_->held.find(0);
    if (h == impl_->held.end()) return space;
    PlaceRef p;
    p.id = h->second;
    impl_->spaces->spaceOf(p, space);
    return space;
}

void CityBrain::occupyForTest(const engine::KerbSpace& space) {
    const PlaceRef place = impl_->spaces->placeOf(space);
    if (place.valid()) impl_->occupant[place.id] = kTestCar;
}

// ---- The city sim's two calls (declared in city_sim.h) -----------------------------------------------------------------

bool CitySim::startBrain(const std::vector<engine::CityMarker>& markers, const engine::CityResident& resident,
                         const std::vector<int>& classOfSlot, std::string* report) {
    std::shared_ptr<CityBrain> made = std::make_shared<CityBrain>(*this);
    if (!made->start(markers, resident, classOfSlot, report)) return false;
    brain_ = std::move(made);
    return true;
}

void CitySim::tickBrain(Real dt) { brain_->tick(dt); }

}  // namespace citysim
