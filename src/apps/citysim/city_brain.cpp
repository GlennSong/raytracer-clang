#include "city_brain.h"

#include "city_car.h"
#include "city_sim.h"
#include "city_trip.h"   // stepsOffThePaths: a walker's last steps off the pavements
#include "brain/brain_driving.h"
#include "brain/brain_planner.h"
#include "brain/brain_states.h"
#include "../../engine/components.h"   // CityMarker, CityResident, CityHousehold
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
#include <cstdlib>
#include <map>

namespace citysim {

using brain::AgentId;
using brain::BrainTime;
using brain::CarRef;
using brain::PlaceRef;
using engine::Vec3;

namespace {

// HOW A PLACE'S HANDLE SAYS WHAT IT IS: the top byte. A marker is its place in the level's list (the low 16 bits) and
// whose spot at it is meant (the next 8: each person stands on a spot of their own, city_brain.h kStandApart); a kerb
// space is a handle of brain::RoadPlaces; a point is where a car was left in a lane.
constexpr uint32_t kKindMask = 0xFF000000u;
constexpr uint32_t kMarkerTag = 0x00000000u;
constexpr uint32_t kSpaceTag = 0x01000000u;
constexpr uint32_t kPointTag = 0x02000000u;
constexpr uint32_t kMarkerIndexMask = 0x0000FFFFu;
constexpr uint32_t kMarkerSpotShift = 16;
constexpr uint32_t kMarkerSpotMask = 0xFFu;
constexpr std::size_t kMostResidents = kMarkerSpotMask + 1;   // one spot each
// The car a test puts in a space (occupyForTest): no car of the households.
constexpr uint32_t kTestCar = brain::kMaxHandle;
// WHO HAS A CAR when it is nobody the brain owns: the player, or a walker a director took over (an OUTSIDER).
constexpr AgentId kOutsider = brain::kNoAgent - 1;
constexpr Real kMillis = 1000.0;
constexpr Real kSecondsPerMinute = 60.0;
// A car an outsider left at a point is put into the kerb lane beside it when the lane is empty from this far behind
// its rear to this far ahead of its front (a body's own room; nobody is waited for: it either fits or it does not).
constexpr Real kLeftRoom = 0.5;
// The events this class posts itself, by the names the data gives them (brain.lua `events`).
const char* const kVehicleGone = "vehicle_gone";
const char* const kPossessed = "possessed";
const char* const kReleased = "released";

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

// ---- Everything the residents need, in one place ---------------------------------------------------------------------

struct CityBrain::Impl {
    explicit Impl(CitySim& s) : sim(s) {}

    CitySim& sim;
    engine::World* ecs = nullptr;   // the engine's entities, the households' cars among them (ADR-0444; city_car.h)
    const engine::RoadModel* model = nullptr;
    driving::Driving* d = nullptr;
    TravelCatalog travel;
    brain::BehaviourSet behaviours;
    brain::BrainTable table;
    std::vector<BrainMarker> markers;
    std::unique_ptr<brain::RoadPlaces> spaces;
    std::vector<Vec2> points;   // where a car was left in a lane, by the low bits of a point place
    brain::EventId vehicleGone = brain::kNoId, possessed = brain::kNoId, released = brain::kNoId;

    // THE CARS, each one entity of the World (city_car.h): its pose, body, household and driver, and while the driving
    // has it, its CityCarOnLane. A car's CarRef and its id in the driving are both its number (CityCarBody::id). What
    // is kept here is the brain's own table about it: where it stands, who has it, and what it is doing between its
    // space and the lane.
    enum class CarPhase : uint8_t {
        Standing,         // in a kerb space, or left standing in a lane
        WaitingForRoom,   // in its space, its driver aboard, waiting for the kerb lane to be clear
        EasingOut,        // placed in the kerb lane; drawn sliding out of the space
        OnRoad,           // the driving's
        EasingIn,         // out of the driving; drawn sliding into its space
    };
    struct CarRec {
        uint32_t id = 0;                      // its number (CityCarBody::id)
        engine::Entity entity;
        int household = -1;
        PlaceRef standing;                    // where it stands, or last stood (not valid: an outsider has it)
        AgentId heldBy = brain::kNoAgent;     // a resident, kOutsider, or nobody
        int outsiderAgent = -1;               // the sim agent in it when an outsider has it (-1: the player themselves)
        bool lost = false;                    // an outsider has it and it is away (the player's own car)
        bool stranded = false;                // left where no lane is beside it: nobody can drive it from there
        bool aboard = false;                  // its holder, a resident, is in it
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

    // THE HOUSEHOLDS: who lives together, and the cars they share (by CarRef id).
    struct Household {
        std::string name;
        std::vector<AgentId> members;
        std::vector<uint32_t> cars;
    };
    std::vector<Household> households;

    // THE PEOPLE the machine has, by its ids.
    enum class WalkPhase : uint8_t { None, Routed, Straight };
    // One stop of a day: a marker, and how long they stay.
    struct Stop {
        PlaceRef place;
        BrainTime dwell = 0;
    };
    struct Person {
        int agent = -1;
        std::string name;
        int household = -1;
        WalkPhase walk = WalkPhase::None;
        brain::WayId way = brain::kNoId;
        Vec2 target;
        std::vector<Vec2> steps;
        // THEIR DAY: the markers they go round, and how long they stay at each.
        std::vector<Stop> day;
        bool dayOn = false;
        int dayAt = -1;   // the stop they are at or going to (-1: not set out yet)
        brain::IntentKind intentKind = 0;
        bool heldByDirector = false;          // a director moves them (the brain's `possessed` state)
        std::string reason;                   // why their last plan was chosen, or why there was none
        std::vector<brain::Transition> trace; // every state they entered
    };
    std::vector<Person> people;

    CityBrainCounts counts;
    std::vector<brain::Transition> traceAll;   // the machine's, shared out to each person's own after every tick

    // (made in start, in this order; the machine last)
    struct LabWorld;
    struct LabWalk;
    struct LabDrive;
    struct FrontPoints;
    struct NotingPlanner;
    std::unique_ptr<LabWorld> world;
    std::unique_ptr<NotingPlanner> planner;
    std::unique_ptr<FrontPoints> fronts;
    std::unique_ptr<brain::DrivingSkill> skill;
    std::unique_ptr<brain::LaneRoom> room;   // whether a lane is clear to pull out into
    std::unique_ptr<LabWalk> walk;
    std::unique_ptr<LabDrive> drive;
    std::unique_ptr<brain::Machine> machine;

    // ---- the sim's own things -----------------------------------------------------------------------------------------
    Agent& agentOf(AgentId who) { return sim.agents_[static_cast<std::size_t>(people[who].agent)]; }
    const Agent& agentOf(AgentId who) const { return sim.agents_[static_cast<std::size_t>(people[who].agent)]; }
    CarRec* car(CarRef ref) {
        for (CarRec& c : cars)
            if (c.id == ref.id) return &c;
        return nullptr;
    }
    const CarRec* car(CarRef ref) const { return const_cast<Impl*>(this)->car(ref); }
    CarRec* carNumbered(int number) {
        CarRef ref;
        if (number >= 0) ref.id = static_cast<uint32_t>(number);
        return ref.valid() ? car(ref) : nullptr;
    }
    const CarRec* carNumbered(int number) const { return const_cast<Impl*>(this)->carNumbered(number); }
    // ---- the car's own components (city_car.h): every question about where it is or what it is asks them -----------
    engine::Transform& poseOf(const CarRec& c) const { return *ecs->get<engine::Transform>(c.entity); }
    const CityCarBody& bodyOf(const CarRec& c) const { return *ecs->get<CityCarBody>(c.entity); }
    CityCarLink& linkOf(const CarRec& c) const { return *ecs->get<CityCarLink>(c.entity); }
    Vec2 carPos(const CarRec& c) const { return carPosition(poseOf(c)); }
    Vec2 carFacing(const CarRec& c) const { return carHeading(poseOf(c)); }
    driving::CarId idOf(const CarRec& c) const { return driving::CarId(c.id); }
    // Is it on the driving's road?
    bool onRoad(const CarRec& c) const { return d->car(idOf(c)) != nullptr; }
    // ITS LANE COMPONENT FOLLOWS THE DRIVING: added when the driving has just been given the car, taken away when the
    // driving no longer has it. Called after every placeCar and removeCar.
    void syncLane(const CarRec& c) {
        const bool has = ecs->has<CityCarOnLane>(c.entity);
        if (onRoad(c) && !has) {
            CityCarOnLane lane;
            lane.car = idOf(c);
            lane.roadYFrom = lane.roadY = poseOf(c).position.y;
            ecs->add<CityCarOnLane>(c.entity, lane);
        } else if (!onRoad(c) && has) {
            ecs->remove<CityCarOnLane>(c.entity);
        }
    }
    // WHERE A PERSON IS: on foot, their own place; in a household's car, the car's.
    Vec2 personPosition(const Agent& a) const {
        const CarRec* in = a.inCar >= 0 ? carNumbered(a.inCar) : nullptr;
        return in ? carPos(*in) : a.pos;
    }
    bool isPerson(int who) const { return who >= 0 && static_cast<std::size_t>(who) < people.size(); }
    int personOfAgent(int agent) const {
        for (std::size_t i = 0; i < people.size(); ++i)
            if (people[i].agent == agent) return static_cast<int>(i);
        return -1;
    }
    BrainTime now() const { return static_cast<BrainTime>(std::llround(sim.simSeconds_ * kMillis)); }
    Real lengthOf(const CarRec& c) const { return d->vehicleClasses().at(bodyOf(c).vehicleClass).length; }

    // ---- places -------------------------------------------------------------------------------------------------------
    static bool isMarker(PlaceRef p) { return p.valid() && (p.id & kKindMask) == kMarkerTag; }
    static bool isPoint(PlaceRef p) { return p.valid() && (p.id & kKindMask) == kPointTag; }
    static uint32_t markerIndex(PlaceRef p) { return p.id & kMarkerIndexMask; }
    static uint32_t markerSpot(PlaceRef p) { return (p.id >> kMarkerSpotShift) & kMarkerSpotMask; }
    bool isSpace(PlaceRef p) const { return spaces && spaces->owns(p); }
    // The marker `index` as the place `who` wants to be at: their own spot at it.
    PlaceRef markerPlace(int index, AgentId who) const {
        PlaceRef p;
        if (index >= 0 && index < static_cast<int>(markers.size()) && who < kMostResidents)
            p.id = kMarkerTag | (static_cast<uint32_t>(who) << kMarkerSpotShift) | static_cast<uint32_t>(index);
        return p;
    }
    // WHERE THE PERSON WITH SPOT NUMBER `spot` STANDS AT A MARKER: the first on the marker's own spot, the rest
    // kStandApart apart along the sidewalk, to one side and the other in turn.
    Vec2 spotAt(const BrainMarker& m, uint32_t spot) const {
        const Vec2 along = engine::perp(unit(m.stand - m.frontage.on, Vec2(1, 0)));
        const Real steps = static_cast<Real>((spot + 1) / 2) * (spot % 2 == 1 ? Real(1) : Real(-1));
        return m.stand + along * (steps * kStandApart);
    }
    PlaceRef pointPlace(Vec2 at) {
        PlaceRef p;
        points.push_back(at);
        p.id = kPointTag | static_cast<uint32_t>(points.size() - 1);
        return p;
    }
    bool positionOf(PlaceRef place, Vec2& out) const {
        if (isMarker(place)) {
            if (markerIndex(place) >= markers.size()) return false;
            out = spotAt(markers[markerIndex(place)], markerSpot(place));
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
        if (isMarker(place) && markerIndex(place) < markers.size()) return markers[markerIndex(place)].name;
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
    // THE KERB SIDE OF A POINT ON THE ROAD: the middle of the sidewalk beside the road nearest it, level with it. Where
    // a person stands to get into a car left in a lane, and where one put out of a car is stood. False: no sidewalk.
    bool kerbSideOf(Vec2 at, Vec2& out) const {
        const engine::RoadFrontage frontage = model->frontageAt(at);
        return frontage.valid() && model->sidewalkBeside(frontage, out);
    }
    // Where a walk to `place` ends.
    bool walkTarget(PlaceRef place, Vec2& out) const {
        if (doorPoint(place, out)) return true;
        Vec2 at;
        if (!positionOf(place, at)) return false;
        if (!isPoint(place) || !kerbSideOf(at, out)) out = at;
        return true;
    }
    // The kerb spaces near a place, nearest first on foot.
    std::vector<PlaceRef> spacesFor(PlaceRef door) const {
        std::vector<PlaceRef> out;
        engine::RoadFrontage frontage;
        Vec2 at;
        if (isMarker(door) && markerIndex(door) < markers.size()) frontage = markers[markerIndex(door)].frontage;
        else if (positionOf(door, at)) frontage = model->frontageAt(at);
        if (!frontage.street.valid()) return out;
        for (const engine::KerbSpaceNear& found : model->kerbSpacesNear(frontage, kSpaceReach, kSpacesLookedAt)) {
            const PlaceRef place = spaces->placeOf(found.space);
            if (place.valid()) out.push_back(place);
        }
        return out;
    }
    // Could `car`, driven by `who` (kNoAgent: by nobody yet), be left in `space` this moment? No other car stands in
    // it, and nobody else holds it.
    bool spaceFree(PlaceRef space, CarRef carRef, AgentId who) const {
        const std::map<uint32_t, uint32_t>::const_iterator in = occupant.find(space.id);
        if (in != occupant.end() && in->second != carRef.id) return false;
        for (const std::pair<const AgentId, uint32_t>& h : held)
            if (h.second == space.id && h.first != who) return false;
        return true;
    }
    void leaveEverySpace(const CarRec& c) {
        for (std::map<uint32_t, uint32_t>::iterator it = occupant.begin(); it != occupant.end();)
            it = it->second == c.id ? occupant.erase(it) : std::next(it);
    }
    // THE HOUSEHOLD IS TOLD ITS CAR HAS CHANGED (it is free, it has moved, it is gone): everyone but `except`. Only a
    // person standing with no way there acts on it (brain.lua `things_changed`).
    void tellHousehold(const CarRec& c, AgentId except) {
        if (!machine || c.household < 0) return;
        for (const AgentId m : households[static_cast<std::size_t>(c.household)].members)
            if (m != except) machine->notifyChanged(m);
    }

    // ---- getting in and out -------------------------------------------------------------------------------------------
    bool getIn(AgentId who, CarRec& c);
    void getOut(AgentId who, CarRec& c);
    // Anybody into the car's driver's seat (they are in it from now: not drawn on foot), and out of it again, stood at
    // `out`.
    void seat(int agent, const CarRec& c);
    void unseat(int agent, const CarRec& c, Vec2 out);
    // Where a person getting out of the car stands: on the kerb side (beside its space, or beside the road where it
    // is), or, failing that, a step to its side clear of the lanes.
    Vec2 outOf(const CarRec& c) const;
    void writeCarPose(CarRec& c, Vec2 pos, Vec2 heading);
    void haltWalker(AgentId who);
    void tickWalks(Real dt);
    void tickCars(Real dt);
    void tickDay(AgentId who);
    void slideIn(CarRec& c, PlaceRef space);
    void shareOutTrace();
    // ---- somebody the brain does not own ------------------------------------------------------------------------------
    void outsiderTook(CarRec& c, int agent, bool lost);
    void outsiderLeft(CarRec& c);
    std::string carLine(const CarRec& c) const;
};

// ---- The kerb lane beside a place, as the point a car's FRONT is at ---------------------------------------------------
// RoadPlaces answers the lane point level with the middle of a space; a car standing beside the space has its middle
// there, so its front is half its length further on. Held to the lane path: the whole body on it, and its front no
// nearer the end than kKerbLaneEndMargin. `length` is the car's being asked about: whoever asks sets it first (cars of
// one household may be of different classes).
struct CityBrain::Impl::FrontPoints : brain::KerbLanes {
    explicit FrontPoints(Impl& i) : im(i) {}
    Impl& im;
    Real length = 0;
    FrontPoints& of(const CarRec& c) {
        length = im.lengthOf(c);
        return *this;
    }
    bool frontAt(const driving::PathPoint& mid, driving::PathPoint& out) const {
        if (!im.d->roads() || mid.path.index >= im.d->roads()->paths.size()) return false;
        const Real pathLength = im.d->roads()->paths[mid.path.index].length;
        const Real lowest = length, highest = pathLength - kKerbLaneEndMargin;
        if (engine::strictlyPast(lowest, highest, engine::kSamePoint)) return false;   // the lane is shorter than the car
        out.path = mid.path;
        out.along = std::clamp(mid.along + length * Real(0.5), lowest, highest);
        return true;
    }
    bool kerbLane(PlaceRef place, driving::PathPoint& out) const override {
        driving::PathPoint mid;
        return im.spaces->kerbLane(place, mid) && frontAt(mid, out);
    }
};

// ---- The planner, with each person's reason kept ------------------------------------------------------------------------
// THE trip planner (brain_planner.h), asked as the machine asks it; what it said about each person's last plan is kept
// under that person, so `brain? <who>` can say why (one planner serves everybody, and its own last reason is whoever
// planned last).
struct CityBrain::Impl::NotingPlanner : brain::Planner {
    NotingPlanner(Impl& i, const engine::NavGraph& nav, const TravelCatalog& ways, const brain::World& world)
        : im(i), inner(nav, ways, world) {}
    Impl& im;
    brain::TripPlanner inner;
    bool plan(const brain::PlanAsk& ask, brain::PlanLegs& out) override {
        const bool ok = inner.plan(ask, out);
        if (ask.agent < im.people.size()) im.people[ask.agent].reason = ok ? inner.lastReason() : inner.lastError();
        return ok;
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
        out = im.personPosition(im.agentOf(who));   // (in a car: where the car is)
        return true;
    }
    // WHICH OF THEIR HOUSEHOLD'S CARS THEY WOULD TAKE NOW: the free one standing nearest them; when none is free, the
    // one they had in mind (so the plan can say "in use"), or the household's first.
    CarRef carOffered(AgentId who, CarRef has, Vec2 from) const override {
        if (who >= im.people.size() || im.people[who].household < 0) return has;
        const Household& h = im.households[static_cast<std::size_t>(im.people[who].household)];
        CarRef best;
        Real bestD2 = 0;
        for (const uint32_t id : h.cars) {
            CarRef ref;
            ref.id = id;
            PlaceRef at;
            Vec2 p;
            if (!carFreeFor(ref, who) || !carStanding(ref, at) || !im.positionOf(at, p)) continue;
            const Real d2 = (p - from).lengthSquared();
            if (!best.valid() || d2 < bestD2) {
                best = ref;
                bestD2 = d2;
            }
        }
        if (best.valid()) return best;
        if (has.valid() || h.cars.empty()) return has;
        best.id = h.cars.front();
        return best;
    }
    bool carStanding(CarRef ref, PlaceRef& out) const override {
        const CarRec* c = im.car(ref);
        // (a car an outsider has stands nowhere a resident could take it from; nor does one left where no lane is)
        if (!c || !c->standing.valid() || c->heldBy == kOutsider || c->stranded) return false;
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
        im.tellHousehold(*c, who);
    }
    void leaveCar(CarRef ref, AgentId who) override {
        CarRec* c = im.car(ref);
        if (!c || c->heldBy != who) return;
        // on the road it stays where it is, placed and undriven, and stands at a place of its own
        if (im.onRoad(*c)) c->standing = im.pointPlace(im.carPos(*c));
        im.getOut(who, *c);
        c->heldBy = brain::kNoAgent;
        c->tripWanted = false;
        if (c->phase != CarPhase::EasingIn) c->phase = CarPhase::Standing;
        im.tellHousehold(*c, who);
    }
    void parkCar(CarRef ref, AgentId who, PlaceRef space) override {
        CarRec* c = im.car(ref);
        if (!c) return;
        c->standing = space;
        im.occupant[space.id] = ref.id;
        im.getOut(who, *c);
        if (c->heldBy == who) c->heldBy = brain::kNoAgent;
        im.held.erase(who);
        im.tellHousehold(*c, who);
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
        im.leaveEverySpace(*c);
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
        if (a.mode != Agent::Mode::Pedestrian || a.inCar >= 0 || !im.walkTarget(to, target)) return false;
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
        if (who >= im.people.size()) return;
        Person& p = im.people[who];
        if (p.heldByDirector) {
            // a director moves them now: the brain's walk is forgotten, and the body left as the director has it
            p.walk = WalkPhase::None;
            p.steps.clear();
            return;
        }
        im.haltWalker(who);
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
}

void CityBrain::Impl::tickWalks(Real dt) {
    for (AgentId who = 0; who < people.size(); ++who) {
        Person& p = people[who];
        Agent& a = agentOf(who);
        // (a person not walking is left as they are: nothing else moves them, since the sim's day table does not place
        // an agent the brain owns -- Agent::brainOwned, ADR-0442)
        if (p.walk == WalkPhase::None || p.heldByDirector) continue;
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
        const Real short_ = (p.target - a.pos).length();
        haltWalker(who);
        if (arrived) {
            ++counts.walksArrived;
            machine->post(who, state.clock);        // the event the walking state's skill ends it by
        } else {
            ++counts.walksBlocked;
            LOG_WARN << "[brain] " << p.name << ": the pavements end " << fmt("%.1f m", short_)
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
        if (!c || c->heldBy == kOutsider) return false;
        if (c->heldBy != brain::kNoAgent && !c->aboard && !im.getIn(c->heldBy, *c)) return false;
        if (im.onRoad(*c)) return true;   // left in a lane by an earlier trip: it sets off from there
        driving::PathPoint front;
        return im.fronts->of(*c).kerbLane(space, front);
    }
    bool beginTrip(CarRef ref, AgentId person, PlaceRef fromSpace, PlaceRef toSpace, Real) override {
        CarRec* c = im.car(ref);
        driving::PathPoint to;
        if (!c || c->heldBy == kOutsider || !im.fronts->of(*c).kerbLane(toSpace, to)) return false;
        c->tripPerson = person;
        c->tripFrom = fromSpace;
        c->tripTo = toSpace;
        if (im.onRoad(*c) && c->phase != CarPhase::EasingOut) {
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
        if (!c || c->heldBy == kOutsider) return;
        c->tripWanted = false;
        if (c->phase == CarPhase::WaitingForRoom) c->phase = CarPhase::Standing;
        im.skill->endTrip(ref);
    }
    // The car stands in `space` from now: off the road if it was still on it, and exactly where a car in the space
    // stands (the slide in, if one was under way, is over).
    void park(CarRef ref, PlaceRef space) override {
        CarRec* c = im.car(ref);
        if (!c || c->heldBy == kOutsider) return;
        if (im.onRoad(*c)) {
            im.skill->endTrip(ref);
            im.fronts->of(*c);
            im.skill->park(ref, space);
            im.syncLane(*c);
        }
        engine::KerbSpacePose pose;
        Vec2 kerbward;
        if (im.spacePose(space, pose, kerbward)) im.writeCarPose(*c, pose.at, pose.facing);
        c->phase = CarPhase::Standing;
        c->tripWanted = false;
    }
    // The trip now ends beside `toSpace`. A car that has not left its space yet sets off for there when it does; one
    // still sliding out has its trip begun to there when the slide ends; one on the road has its trip's end changed.
    bool changeDestination(CarRef ref, PlaceRef toSpace) override {
        CarRec* c = im.car(ref);
        driving::PathPoint to;
        if (!c || c->heldBy == kOutsider || !im.fronts->of(*c).kerbLane(toSpace, to)) return false;
        if (c->tripWanted) {
            c->tripTo = toSpace;
            ++im.counts.redirected;
            return true;
        }
        if (!im.onRoad(*c) || !im.skill->changeDestination(ref, toSpace)) return false;
        c->tripTo = toSpace;
        ++im.counts.redirected;
        return true;
    }
    bool waitingToSetOff(CarRef ref) const override {
        const CarRec* c = im.car(ref);
        return c && c->phase == CarPhase::WaitingForRoom;
    }
};

// THE PERSON WHO HAS THE CAR GETS IN: into its driver's seat, which nobody else is in.
bool CityBrain::Impl::getIn(AgentId who, CarRec& c) {
    const Agent& a = agentOf(who);
    if (a.mode != Agent::Mode::Pedestrian || a.inCar >= 0 || a.vehicle >= 0 || linkOf(c).driver >= 0) return false;
    // NO WALK GOES ON IN THE CAR. A walk of no length (the person already beside the car) is ended by the machine on
    // the spot while the walk skill still has it; stepped on from inside the car, it "arrived" seconds later and its
    // report ended whatever state the person was in by then (pulling_out, 3.6 s into its two minutes). With the person
    // a driver of a sim vehicle (before ADR-0444) the sim refused to step them, which hid it.
    haltWalker(who);
    seat(people[who].agent, c);
    c.aboard = true;
    ++counts.boarded;
    return true;
}

// ...AND GETS OUT, leaving the car exactly where it stands: out on the kerb side.
void CityBrain::Impl::getOut(AgentId who, CarRec& c) {
    if (!c.aboard || who >= people.size()) return;
    unseat(people[who].agent, c, outOf(c));
    c.aboard = false;
    ++counts.alighted;
}

// IN THE DRIVER'S SEAT: the person is in the car from now (Agent::inCar), not drawn on foot, and not one of the
// walkers anybody senses; where they are is the car's pose. Their own place is set to where they got in.
void CityBrain::Impl::seat(int agent, const CarRec& c) {
    Agent& a = sim.agents_[static_cast<std::size_t>(agent)];
    a.inCar = static_cast<int>(c.id);
    a.pos = carPos(c);
    a.heading = carFacing(c);
    a.tickFromPos = a.pos;   // (a placement, not a motion)
    a.tickFromHeading = a.heading;
    a.indoors = false;
    a.moving = false;
    a.speed = 0;
    a.route = engine::Route{};
    a.leg = 0;
    a.distOnLeg = 0;
    a.directedPace = 0;
    a.directedRide = Ride::None;
    a.state = Agent::State::Waiting;
    linkOf(c).driver = agent;
    sim.grid_.remove(agent);
}

void CityBrain::Impl::unseat(int agent, const CarRec& c, Vec2 out) {
    Agent& a = sim.agents_[static_cast<std::size_t>(agent)];
    a.inCar = -1;
    a.pos = out;
    a.heading = carFacing(c);
    a.tickFromPos = out;
    a.tickFromHeading = a.heading;
    a.moving = false;
    a.speed = 0;
    a.route = engine::Route{};
    a.leg = 0;
    a.distOnLeg = 0;
    a.state = Agent::State::Resting;
    if (linkOf(c).driver == agent) linkOf(c).driver = -1;
    sim.grid_.place(agent, out);
}

Vec2 CityBrain::Impl::outOf(const CarRec& c) const {
    Vec2 out;
    const Vec2 at = carPos(c);
    // beside its space; or, left in a lane (or nowhere a space is), on the sidewalk beside the road there
    if (!onRoad(c) && doorPoint(c.standing, out)) return out;
    if (kerbSideOf(at, out)) return out;
    // (no sidewalk near: a step to the car's side, clear of the lanes, as the sim stands anyone getting out)
    const Vec2 h = carFacing(c);
    return sim.pushPoseClearOfLanes(at + Vec2(h.y, -h.x) * 2.0, 1.0);
}

// THE CAR IS AT `pos`, POINTING ALONG `heading` (drawn sliding between its space and the lane, or put down). Its height
// is left as it was: off the road the drawing stands it on the ground under it.
void CityBrain::Impl::writeCarPose(CarRec& c, Vec2 pos, Vec2 heading) {
    engine::Transform& t = poseOf(c);
    setCarPose(t, pos, heading, t.position.y);
}

// The space is the car's: it leaves the driving and is drawn sliding in from where it stands in the lane.
void CityBrain::Impl::slideIn(CarRec& c, PlaceRef space) {
    engine::KerbSpacePose pose;
    Vec2 kerbward;
    if (!spacePose(space, pose, kerbward)) return;
    const driving::CarId id = idOf(c);
    if (onRoad(c)) {
        CarRef ref;
        ref.id = id.index;
        skill->endTrip(ref);   // (harmless: the trip ended when the driving reported "parked")
        d->removeCar(id);
        syncLane(c);
    }
    c.easeFromPos = carPos(c);
    c.easeFromHeading = carFacing(c);
    c.easeToPos = pose.at;
    c.easeToHeading = pose.facing;
    c.easeT = 0;
    c.phase = CarPhase::EasingIn;
    c.standing = space;
    ++counts.slidIn;
}

void CityBrain::Impl::tickCars(Real dt) {
    for (CarRec& c : cars) {
        if (c.heldBy == kOutsider) continue;   // somebody else's to move
        const driving::CarId id = idOf(c);
        CarRef ref;
        ref.id = id.index;
        if (c.phase == CarPhase::WaitingForRoom) {
            driving::PathPoint front;
            const Real length = lengthOf(c);
            if (!fronts->of(c).kerbLane(c.standing, front)) continue;
            if (!room->clear(front, length, kPullOutRoomBehind, kPullOutRoomAhead)) {
                ++counts.waitedForRoom;
            } else if (d->placeCar(id, bodyOf(c).vehicleClass, front)) {
                syncLane(c);
                occupant.erase(c.standing.id);
                c.easeFromPos = carPos(c);
                c.easeFromHeading = carFacing(c);
                // where the driving has it: the middle of its body and the way it points (as the hand-over copies it)
                const driving::CarPose pose = driving::carPose(*d->roads(), d->vehicleClasses(), d->ruleSet(), *d->car(id));
                const Vec3 mid = (pose.front + pose.rear) * 0.5;
                c.easeToPos = Vec2(mid.x, mid.z);
                c.easeToHeading = unit(Vec2(pose.body.x, pose.body.z), c.easeFromHeading);
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
                        fronts->of(c);
                        // (no route: the driving queues its own "no route" for the person, which the skill passes on)
                        if (skill->beginTrip(ref, c.tripPerson, c.tripFrom, c.tripTo, 0)) ++counts.tripsBegun;
                    }
                }
            }
        }
    }
}

// A DAY TABLE: once the person has been where they wanted to be for that stop's minutes, they want to be at the next.
void CityBrain::Impl::tickDay(AgentId who) {
    Person& p = people[who];
    if (!p.dayOn || p.day.empty()) return;
    const brain::AgentBrain& a = machine->agent(who);
    if (a.travelling() || a.state != table.intents[a.intentKind].arrive) return;
    const std::size_t at = p.dayAt < 0 ? p.day.size() - 1 : static_cast<std::size_t>(p.dayAt);
    if (now() - a.stateEntered < p.day[at].dwell) return;
    p.dayAt = static_cast<int>((at + 1) % p.day.size());
    machine->setIntent(who, p.intentKind, p.day[static_cast<std::size_t>(p.dayAt)].place);
}

void CityBrain::Impl::shareOutTrace() {
    for (const brain::Transition& t : traceAll)
        if (t.agent < people.size()) people[t.agent].trace.push_back(t);
    traceAll.clear();
}

// ---- Somebody the brain does not own, and a household's car (ADR-0443) --------------------------------------------------

// AN OUTSIDER HAS THE CAR FROM NOW: it leaves every table. No space is its, it is off the driving's road, no trip is
// wanted of it, and nobody of the household can take it or plan with it until it is left again. `lost`: the player has
// it as a physical car of their own, so it is not drawn here (CityCarLink::away) until it is handed back.
void CityBrain::Impl::outsiderTook(CarRec& c, int agent, bool lost) {
    const driving::CarId id = idOf(c);
    if (onRoad(c)) {
        CarRef ref;
        ref.id = id.index;
        skill->endTrip(ref);
        d->removeCar(id);
        syncLane(c);
    }
    leaveEverySpace(c);
    c.heldBy = kOutsider;
    c.outsiderAgent = agent;
    c.lost = lost;
    linkOf(c).away = lost;
    c.stranded = false;
    c.aboard = false;
    c.phase = CarPhase::Standing;
    c.tripWanted = false;
    c.standing = PlaceRef();
    tellHousehold(c, brain::kNoAgent);
}

// THE OUTSIDER HAS LEFT IT where it now stands. In a kerb space when its middle is within kLeftInSpaceWithin of where
// a car in one stands and no other car is in that space. Otherwise at a point: on the road (no farther than
// kLeftOnRoadWithin beyond the kerb line) it is put into the kerb lane beside that point, so the driving knows it is
// there and a resident can drive it away; anywhere else it is stranded, and nobody plans with it.
void CityBrain::Impl::outsiderLeft(CarRec& c) {
    const Vec2 at = carPos(c);
    const driving::CarId id = idOf(c);
    CarRef ref;
    ref.id = id.index;
    c.heldBy = brain::kNoAgent;
    c.outsiderAgent = -1;
    c.lost = false;
    linkOf(c).away = false;
    c.aboard = false;
    c.phase = CarPhase::Standing;
    c.tripWanted = false;
    c.stranded = false;
    const engine::KerbSpaceNear near = model->nearestKerbSpace(at, kLeftInSpaceWithin);
    const PlaceRef space = near.space.valid() ? spaces->placeOf(near.space) : PlaceRef();
    const std::map<uint32_t, uint32_t>::const_iterator in = space.valid() ? occupant.find(space.id) : occupant.end();
    if (space.valid() && (in == occupant.end() || in->second == ref.id)) {
        c.standing = space;
        occupant[space.id] = ref.id;
    } else {
        c.standing = pointPlace(at);
        const engine::RoadFrontage frontage = model->frontageAt(at);
        const engine::KerbLanePlace lane = frontage.valid() && frontage.street.valid() ? model->kerbLaneBeside(frontage)
                                                                                         : engine::KerbLanePlace();
        const bool onTheRoad = frontage.valid() && lane.valid() &&
                               engine::atOrBefore(frontage.distance, frontage.section->carriagewayHalfWidth() + kLeftOnRoadWithin,
                                                  engine::kSamePoint);
        driving::PathPoint mid, front;
        mid.path = lane.path;
        mid.along = lane.along;
        if (!onRoad(c) && onTheRoad && fronts->of(c).frontAt(mid, front) && room->clear(front, lengthOf(c), kLeftRoom, kLeftRoom))
            d->placeCar(id, bodyOf(c).vehicleClass, front);
        syncLane(c);
        c.stranded = !onRoad(c);
    }
    ++counts.leftAgain;
    LOG_INFO << "[brain] " << carLine(c);
    tellHousehold(c, brain::kNoAgent);
}

std::string CityBrain::Impl::carLine(const CarRec& c) const {
    static const char* const kPhase[] = {"standing", "waiting in its space for the kerb lane to be clear",
                                         "sliding out of its space", "on the road", "sliding into its space"};
    std::string s = "the car of household " +
                    (c.household >= 0 ? households[static_cast<std::size_t>(c.household)].name : std::string("?")) + " (car " +
                    std::to_string(c.id) + ") ";
    if (c.heldBy == kOutsider) {
        s += c.outsiderAgent >= 0 ? "has been taken by somebody else (sim agent " + std::to_string(c.outsiderAgent) + ")"
                                  : std::string("has been taken by the player");
        return s + ": it is not free, and stands nowhere a resident could take it from";
    }
    s += std::string("is ") + kPhase[static_cast<int>(c.phase)] +
         (c.phase == CarPhase::OnRoad && !c.aboard ? " (left there)" : "") + ", " +
         (onRoad(c) && isSpace(c.standing) ? "last in " : isPoint(c.standing) ? "left at " : "in ") + placeName(c.standing);
    if (c.stranded) s += ", off the road: nobody can drive it from there";
    if (c.heldBy < people.size()) s += ", " + people[c.heldBy].name + " has it";
    else s += ", free";
    if (const driving::Car* placed = d->car(idOf(c))) s += ", " + fmt("%.1f m/s", placed->speed);
    return s;
}

// ---- CityBrain ----------------------------------------------------------------------------------------------------------

CityBrain::CityBrain(CitySim& sim) : impl_(std::make_unique<Impl>(sim)) {}
CityBrain::~CityBrain() = default;

bool CityBrain::start(engine::World& world, const std::vector<engine::CityMarker>& markerSpecs,
                      const std::vector<engine::CityResident>& residents, const std::vector<engine::CityHousehold>& householdSpecs,
                      const std::vector<int>& classOfSlot, std::string* report) {
    Impl& im = *impl_;
    CitySim& sim = im.sim;
    const auto refuse = [&](const std::string& why) {
        if (report) *report = why;
        for (const Impl::CarRec& c : im.cars) world.destroy(c.entity);   // (nothing is left behind)
        im.cars.clear();
        return false;
    };
    if (!sim.drivingV2_ || !sim.drivingV2_->roads() || !sim.drivingV2_->hasRules() || !sim.drivingHost_.active())
        return refuse("the new driving does not run on this level (citysim.driving \"v2\" with a rule set)");
    if (!sim.nav_) return refuse("the city has no street map");
    im.d = &*sim.drivingV2_;
    im.model = im.d->model();
    im.ecs = &world;
    {
        // the cars of a brain made before (the city built again on the same world) go: their numbers and their people
        // are the old city's
        std::vector<engine::Entity> old;
        world.each<CityCarBody>([&](engine::Entity e, CityCarBody&) { old.push_back(e); });
        for (const engine::Entity e : old) world.destroy(e);
    }

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
    // the events this class posts itself
    const auto eventNamed = [&](const char* name, brain::EventId& into) {
        const int e = im.table.event(name);
        if (e >= 0) into = static_cast<brain::EventId>(e);
        return e >= 0;
    };
    if (!eventNamed(kVehicleGone, im.vehicleGone) || !eventNamed(kPossessed, im.possessed) || !eventNamed(kReleased, im.released))
        return refuse(im.table.file + ": events lacks one of `" + kVehicleGone + "`, `" + kPossessed + "`, `" + kReleased +
                      "`, which the city's side of the brain posts");

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
        {
            // the post: beside the ring, half-way between away from the road and along the street (city_brain.h)
            const Vec2 away = unit(m.stand - m.frontage.on, Vec2(1, 0));
            m.post = m.stand + unit(away + engine::perp(away), away) * kMarkerPostOffset;
        }
        im.markers.push_back(m);
    }
    if (!left.empty())
        LOG_WARN << "[brain] markers left out (no such street, or no sidewalk beside it): " << left;
    im.spaces = std::make_unique<brain::RoadPlaces>(*im.model, kSpaceTag);
    std::string line = std::to_string(im.markers.size()) + " markers";
    if (residents.empty()) {
        if (report) *report = line + ", no resident";
        return true;
    }
    if (residents.size() > kMostResidents)
        return refuse("the level states " + std::to_string(residents.size()) + " residents; a marker has a spot for " +
                      std::to_string(kMostResidents));

    // THE RESIDENTS, checked before anything is made: each a walker the level made, in the order it made them
    std::vector<int> agents;
    for (std::size_t i = 0; i < sim.agents_.size() && agents.size() < residents.size(); ++i) {
        const Agent& a = sim.agents_[i];
        if (a.mode == Agent::Mode::Pedestrian && !a.released && !a.playerControlled) agents.push_back(static_cast<int>(i));
    }
    if (agents.size() < residents.size())
        return refuse("the level states " + std::to_string(residents.size()) + " residents and made " + std::to_string(agents.size()) +
                      " walkers (citysim.pedestrians must be at least the number of residents)");
    struct Made {
        int home = -1;
        brain::IntentKind intent = 0;
        uint8_t traits = 0;
    };
    std::vector<Made> made;
    for (std::size_t r = 0; r < residents.size(); ++r) {
        const engine::CityResident& resident = residents[r];
        const std::string at = "resident \"" + resident.name + "\"";
        for (std::size_t other = 0; other < r; ++other)
            if (residents[other].name == resident.name) return refuse(at + " is listed twice");
        Made m;
        m.home = markerNamed(resident.home);
        if (m.home < 0) return refuse(at + ": home \"" + resident.home + "\" is not one of the level's markers");
        const int intent = im.table.intent(resident.intent);
        if (intent < 0) return refuse(at + ": intent \"" + resident.intent + "\" is not an intent of brain.lua");
        m.intent = static_cast<brain::IntentKind>(intent);
        for (const std::string& name : resident.traits) {
            const int t = im.travel.trait(name);
            if (t < 0) return refuse(at + ": trait \"" + name + "\" is not a trait of travel.lua");
            m.traits = static_cast<uint8_t>(m.traits | (1u << t));
        }
        made.push_back(m);
        Impl::Person person;
        person.agent = agents[r];
        person.name = resident.name;
        person.intentKind = m.intent;
        im.people.push_back(person);
    }

    im.world = std::make_unique<Impl::LabWorld>(im);
    im.planner = std::make_unique<Impl::NotingPlanner>(im, *sim.nav_, im.travel, *im.world);
    im.fronts = std::make_unique<Impl::FrontPoints>(im);
    im.room = std::make_unique<brain::LaneRoom>(*im.d);

    // THE HOUSEHOLDS AND THEIR CARS, each car standing in the free kerb space nearest (on foot) the marker it states
    for (const engine::CityHousehold& hs : householdSpecs) {
        Impl::Household h;
        h.name = hs.name;
        const int index = static_cast<int>(im.households.size());
        for (const std::string& member : hs.members) {
            const int who = residentNamed(member);
            if (who < 0 || member != im.people[static_cast<std::size_t>(who)].name)
                return refuse("household \"" + hs.name + "\": member \"" + member + "\" is not one of the level's residents");
            if (im.people[static_cast<std::size_t>(who)].household >= 0)
                return refuse("household \"" + hs.name + "\": \"" + member + "\" is already in another household");
            im.people[static_cast<std::size_t>(who)].household = index;
            h.members.push_back(static_cast<AgentId>(who));
        }
        for (const engine::CityHousehold::Car& cs : hs.cars) {
            const std::string at = "household \"" + hs.name + "\": a car near \"" + cs.near + "\"";
            const int marker = markerNamed(cs.near);
            if (marker < 0) return refuse(at + ": that is not one of the level's markers");
            PlaceRef chosen;
            for (const PlaceRef space : im.spacesFor(im.markerPlace(marker, 0))) {
                if (im.occupant.count(space.id) > 0) continue;
                if (!cs.space.empty() && im.placeName(space) != "space " + cs.space) continue;
                chosen = space;
                break;
            }
            engine::KerbSpacePose pose;
            Vec2 kerbward;
            if (!chosen.valid() || !im.spacePose(chosen, pose, kerbward))
                return refuse(at + (cs.space.empty() ? std::string() : " in space \"" + cs.space + "\"") +
                              ": no such free kerb parking space within " + fmt("%.0f m", kSpaceReach) + " on foot of the marker");
            // THE CAR IS ONE ENTITY (city_car.h), numbered on from the sim's last vehicle row, and drawn as the fleet
            // slot the sim would deal a vehicle of that number (CitySim::ambientSlotFor)
            Impl::CarRec c;
            c.id = static_cast<uint32_t>(sim.vehicles_.size() + im.cars.size());            c.household = index;
            const int slot = sim.ambientSlotFor(static_cast<int>(c.id));
            const VehicleBody& body = sim.fleetBody(slot);
            const int cls = slot >= 0 && slot < static_cast<int>(classOfSlot.size()) ? classOfSlot[static_cast<std::size_t>(slot)] : -1;
            CityCarBody carBody;
            carBody.id = c.id;
            carBody.slot = slot;
            carBody.type = body.type;
            carBody.length = body.length;
            carBody.width = body.width;
            carBody.height = body.height;
            carBody.vehicleClass = cls >= 0 && cls < static_cast<int>(im.d->vehicleClasses().size()) ? static_cast<uint32_t>(cls) : 0u;
            CityCarLink link;
            link.household = index;
            engine::Transform where;
            setCarPose(where, pose.at, pose.facing, 0);
            c.entity = world.create();
            world.add<engine::Transform>(c.entity, where);
            world.add<engine::PrevTransform>(c.entity, engine::PrevTransform{where});
            world.add<CityCarBody>(c.entity, carBody);
            world.add<CityCarLink>(c.entity, link);
            c.standing = chosen;
            im.cars.push_back(c);
            im.occupant[chosen.id] = c.id;
            h.cars.push_back(c.id);
            line += "; household " + hs.name + "'s car (car " + std::to_string(c.id) + ", a " +
                    im.d->vehicleClasses().at(carBody.vehicleClass).name + ") stands in " + im.placeName(chosen);
        }
        im.households.push_back(h);
    }
    im.skill = std::make_unique<brain::DrivingSkill>(*im.d, *im.fronts, im.cars.empty() ? 0u : im.bodyOf(im.cars.front()).vehicleClass);
    // the hand-over copies the driving's pose into each car entity while the driving has it, and begins each tick
    sim.drivingHost_.useCarEntities(&world);
    if (!im.skill->bindEvents(im.table, &err)) return refuse(err);
    im.walk = std::make_unique<Impl::LabWalk>(im);
    im.drive = std::make_unique<Impl::LabDrive>(im);
    im.machine = std::make_unique<brain::Machine>(im.table, *im.world, *im.planner, im.walk.get(), im.drive.get());
    im.machine->traceTo(&im.traceAll);

    // THE PEOPLE, each standing on their own spot at their home marker, the brain's to move from now on
    for (std::size_t r = 0; r < residents.size(); ++r) {
        const engine::CityResident& resident = residents[r];
        Impl::Person& person = im.people[r];
        const AgentId id = static_cast<AgentId>(r);
        const BrainMarker& homeMarker = im.markers[static_cast<std::size_t>(made[r].home)];
        const PlaceRef homePlace = im.markerPlace(made[r].home, id);
        CarRef carRef;
        if (person.household >= 0 && !im.households[static_cast<std::size_t>(person.household)].cars.empty())
            carRef.id = im.households[static_cast<std::size_t>(person.household)].cars.front();
        {
            Agent& a = sim.agents_[static_cast<std::size_t>(person.agent)];
            sim.setAgentDirected(person.agent, true);
            a.brainOwned = true;   // the sim's day table never places them (CitySim::placeFromSchedule)
            sim.pinAgent(person.agent, true);
            a.pos = im.spotAt(homeMarker, static_cast<uint32_t>(r));
            a.tickFromPos = a.pos;
            a.heading = unit(homeMarker.frontage.on - homeMarker.stand, a.heading);
            a.tickFromHeading = a.heading;
            a.indoors = false;
            a.moving = false;
            a.speed = 0;
            a.route = engine::Route{};
            a.wakeAt = -1;
            a.state = Agent::State::Resting;
            a.car = -1;   // (the household's cars are entities, not the sim's vehicles: ADR-0444)
            sim.grid_.place(person.agent, a.pos);
        }
        const AgentId who = im.machine->addAgent(homePlace, person.intentKind, made[r].traits, carRef, brain::Detail::Near);
        person.trace.push_back(brain::Transition{who, im.machine->agent(who).state, im.now()});
        // THEIR DAY
        for (const engine::CityResident::Stop& stop : resident.day) {
            const int at = markerNamed(stop.at);
            if (at < 0) {
                LOG_WARN << "[brain] resident " << resident.name << ": day stop \"" << stop.at
                         << "\" is not one of the level's markers -- left out";
                continue;
            }
            Impl::Stop s;
            s.place = im.markerPlace(at, id);
            s.dwell = static_cast<BrainTime>(std::llround(stop.minutes * kSecondsPerMinute * kMillis));
            person.day.push_back(s);
        }
        person.dayOn = !person.day.empty();
        line += "; " + resident.name + " is the sim's agent " + std::to_string(person.agent) + ", at home at \"" + resident.home + "\"" +
                (person.household >= 0 ? " in household " + im.households[static_cast<std::size_t>(person.household)].name
                                       : std::string(" in no household")) +
                (person.day.empty() ? std::string(", no day table") : ", a day of " + std::to_string(person.day.size()) + " stops");
    }
    if (report) *report = line;
    return true;
}

void CityBrain::tick(Real dt) {
    Impl& im = *impl_;
    if (!im.machine) return;
    // the driving's reports for the households' cars, taken from the hand-over (which drains the driving's events)
    const std::vector<driving::Event> events = im.sim.drivingHost_.takeGuestEvents();
    for (const driving::Event& e : events)
        if (e.kind == driving::EventKind::Parked) ++im.counts.parkedReports;
    im.skill->report(*im.machine, events);
    im.tickWalks(dt);
    im.machine->tick();
    im.tickCars(dt);
    for (AgentId who = 0; who < im.people.size(); ++who) im.tickDay(who);
    im.shareOutTrace();
}

const std::vector<BrainMarker>& CityBrain::markers() const { return impl_->markers; }

int CityBrain::markerNamed(const std::string& name) const {
    for (std::size_t i = 0; i < impl_->markers.size(); ++i)
        if (impl_->markers[i].name == name) return static_cast<int>(i);
    return -1;
}

int CityBrain::residentCount() const { return impl_->machine ? static_cast<int>(impl_->people.size()) : 0; }

int CityBrain::residentNamed(const std::string& nameOrNumber) const {
    const Impl& im = *impl_;
    for (std::size_t i = 0; i < im.people.size(); ++i)
        if (im.people[i].name == nameOrNumber) return static_cast<int>(i);
    if (nameOrNumber.empty() || nameOrNumber.find_first_not_of("0123456789") != std::string::npos) return -1;
    const long n = std::strtol(nameOrNumber.c_str(), nullptr, 10);
    return n >= 0 && n < static_cast<long>(im.people.size()) ? static_cast<int>(n) : -1;
}

std::string CityBrain::residentName(int who) const {
    return impl_->isPerson(who) ? impl_->people[static_cast<std::size_t>(who)].name : std::string();
}

std::string CityBrain::householdName(int who) const {
    const Impl& im = *impl_;
    if (!im.isPerson(who) || im.people[static_cast<std::size_t>(who)].household < 0) return "";
    return im.households[static_cast<std::size_t>(im.people[static_cast<std::size_t>(who)].household)].name;
}

std::vector<int> CityBrain::householdOf(int who) const {
    const Impl& im = *impl_;
    std::vector<int> out;
    if (!im.isPerson(who)) return out;
    const int h = im.people[static_cast<std::size_t>(who)].household;
    if (h < 0) return {who};
    for (const AgentId m : im.households[static_cast<std::size_t>(h)].members) out.push_back(static_cast<int>(m));
    return out;
}

int CityBrain::residentAgent(int who) const {
    return residentCount() > 0 && impl_->isPerson(who) ? impl_->people[static_cast<std::size_t>(who)].agent : -1;
}
int CityBrain::residentOfAgent(int agent) const { return impl_->machine ? impl_->personOfAgent(agent) : -1; }

int CityBrain::residentCar(int who) const {
    const Impl& im = *impl_;
    if (residentCount() == 0 || !im.isPerson(who)) return -1;
    CarRef ref;
    ref.id = im.machine->agent(static_cast<AgentId>(who)).car;
    const Impl::CarRec* c = im.car(ref);
    return c ? static_cast<int>(c->id) : -1;
}

std::vector<int> CityBrain::cars() const {
    std::vector<int> out;
    for (const Impl::CarRec& c : impl_->cars) out.push_back(static_cast<int>(c.id));
    return out;
}

engine::Entity CityBrain::carEntity(int car) const {
    const Impl::CarRec* c = impl_->carNumbered(car);
    return c ? c->entity : engine::Entity{};
}

bool CityBrain::carPose(int car, Vec2& pos, Vec2& heading) const {
    const Impl::CarRec* c = impl_->carNumbered(car);
    if (!c || impl_->linkOf(*c).away) return false;
    pos = impl_->carPos(*c);
    heading = impl_->carFacing(*c);
    return true;
}

Vec2 CityBrain::personPosition(int who) const {
    return impl_->isPerson(who) ? impl_->personPosition(impl_->agentOf(static_cast<AgentId>(who))) : Vec2(0, 0);
}

bool CityBrain::inCar(int who) const { return impl_->isPerson(who) && impl_->agentOf(static_cast<AgentId>(who)).inCar >= 0; }

std::vector<int> CityBrain::membersOfCar(int car) const {
    const Impl& im = *impl_;
    std::vector<int> out;
    const Impl::CarRec* c = im.carNumbered(car);
    if (!c || c->household < 0) return out;
    for (const AgentId m : im.households[static_cast<std::size_t>(c->household)].members) out.push_back(static_cast<int>(m));
    return out;
}

CarHolder CityBrain::carHolder(int car) const {
    const Impl::CarRec* c = impl_->carNumbered(car);
    if (!c || c->heldBy == brain::kNoAgent) return CarHolder::Nobody;
    return c->heldBy == kOutsider ? CarHolder::Outsider : CarHolder::Resident;
}

std::string CityBrain::describeCar(int car) const {
    const Impl::CarRec* c = impl_->carNumbered(car);
    return c ? impl_->carLine(*c) : std::string("no household's car is car ") + std::to_string(car);
}

const std::vector<brain::Transition>& CityBrain::trace(int who) const {
    static const std::vector<brain::Transition> kNone;
    return impl_->isPerson(who) ? impl_->people[static_cast<std::size_t>(who)].trace : kNone;
}
const brain::Machine* CityBrain::machine() const { return impl_->machine.get(); }
const CityBrainCounts& CityBrain::counts() const { return impl_->counts; }
brain::BrainTime CityBrain::now() const { return impl_->now(); }
bool CityBrain::dayOn(int who) const { return impl_->isPerson(who) && impl_->people[static_cast<std::size_t>(who)].dayOn; }
void CityBrain::setDay(bool on) {
    for (int who = 0; who < static_cast<int>(impl_->people.size()); ++who) setDay(who, on);
}
void CityBrain::setDay(int who, bool on) {
    if (!impl_->isPerson(who)) return;
    Impl::Person& p = impl_->people[static_cast<std::size_t>(who)];
    p.dayOn = on && !p.day.empty();
}

std::string CityBrain::stateName(int who) const {
    return residentCount() > 0 && impl_->isPerson(who) ? impl_->table.states[impl_->machine->agent(static_cast<AgentId>(who)).state].name
                                                       : std::string();
}

std::string CityBrain::stateLabel(int who) const {
    return residentCount() > 0 && impl_->isPerson(who) ? impl_->table.states[impl_->machine->agent(static_cast<AgentId>(who)).state].label
                                                       : std::string();
}

std::string CityBrain::planReason(int who) const {
    return impl_->isPerson(who) ? impl_->people[static_cast<std::size_t>(who)].reason : std::string();
}

int CityBrain::wantedMarker(int who) const {
    const Impl& im = *impl_;
    if (residentCount() == 0 || !im.isPerson(who)) return -1;
    PlaceRef target;
    target.id = im.machine->agent(static_cast<AgentId>(who)).intentTarget;
    return Impl::isMarker(target) && Impl::markerIndex(target) < im.markers.size() ? static_cast<int>(Impl::markerIndex(target)) : -1;
}

Vec2 CityBrain::standSpot(int who, int marker) const {
    const Impl& im = *impl_;
    if (marker < 0 || marker >= static_cast<int>(im.markers.size()) || who < 0) return Vec2(0, 0);
    return im.spotAt(im.markers[static_cast<std::size_t>(marker)], static_cast<uint32_t>(who));
}

std::string CityBrain::standingAt(int who) const {
    if (residentCount() == 0 || !impl_->isPerson(who)) return "";
    const Impl& im = *impl_;
    const Agent& body = im.agentOf(static_cast<AgentId>(who));
    if (body.mode != Agent::Mode::Pedestrian || body.inCar >= 0) return "";
    for (const BrainMarker& m : im.markers)
        if (engine::atOrBefore((body.pos - im.spotAt(m, static_cast<uint32_t>(who))).length(), kWalkArrivedWithin, engine::kSamePoint))
            return m.name;
    return "";
}

std::string CityBrain::planText(int who) const {
    if (residentCount() == 0 || !impl_->isPerson(who)) return "";
    const Impl& im = *impl_;
    const brain::AgentBrain& a = im.machine->agent(static_cast<AgentId>(who));
    std::string s;
    for (int i = 0; i < a.legCount; ++i) {
        PlaceRef end;
        end.id = a.legEnd[i];
        const brain::WayId way = a.legWay(i);
        s += (i ? ", " : "") + (way < im.table.ways.size() ? im.table.ways[way].name : std::string("?")) + ">" + im.placeName(end);
    }
    return s;
}

std::string CityBrain::traceText(int who) const {
    const Impl& im = *impl_;
    std::string s;
    for (const brain::Transition& t : trace(who))
        s += (s.empty() ? "" : " ") + im.table.states[t.state].name + "@" + std::to_string(t.at);
    return s;
}

std::string CityBrain::describe(int who) const {
    const Impl& im = *impl_;
    if (residentCount() == 0) return std::to_string(im.markers.size()) + " markers, no resident with the agent brain on this level";
    if (!im.isPerson(who)) return "no resident " + std::to_string(who);
    const AgentId id = static_cast<AgentId>(who);
    const Impl::Person& person = im.people[id];
    const brain::AgentBrain& a = im.machine->agent(id);
    const Agent& body = im.agentOf(id);
    std::string s = person.name + (person.household >= 0 ? " of household " + im.households[static_cast<std::size_t>(person.household)].name
                                                         : std::string()) +
                    " = " + im.machine->describe(id);
    // the places it names, in words
    std::vector<uint32_t> named;
    named.push_back(a.intentTarget);
    for (int i = 0; i < a.legCount; ++i) named.push_back(a.legEnd[i]);
    std::sort(named.begin(), named.end());
    named.erase(std::unique(named.begin(), named.end()), named.end());
    s += " | places:";
    for (const uint32_t place : named) {
        PlaceRef p;
        p.id = place;
        s += " " + std::to_string(place) + " = " + im.placeName(p) + ";";
    }
    const Vec2 where = im.personPosition(body);
    s += " | the person (sim agent " + std::to_string(person.agent) + ") is at " + fmt("%.1f", where.x) + " " +
         fmt("%.1f", where.y) + (body.inCar >= 0 ? " in the car" : " on foot") +
         (person.heldByDirector ? ", moved by a director" : "");
    if (person.household >= 0)
        for (const uint32_t carId : im.households[static_cast<std::size_t>(person.household)].cars) {
            CarRef ref;
            ref.id = carId;
            if (const Impl::CarRec* c = im.car(ref)) s += " | " + im.carLine(*c);
        }
    else
        s += " | no household, no car";
    const std::map<AgentId, uint32_t>::const_iterator h = im.held.find(id);
    if (h != im.held.end()) {
        PlaceRef p;
        p.id = h->second;
        s += " | held for them: " + im.placeName(p);
    }
    s += std::string(" | day table ") + (person.day.empty() ? "none" : person.dayOn ? "on" : "off (brain day on)");
    uint8_t bit = 0;
    std::string has;
    for (const std::string& name : im.travel.traits) {
        if ((a.traits >> bit) & 1u) has += (has.empty() ? "" : ", ") + name;
        ++bit;
    }
    s += " | traits: " + (has.empty() ? std::string("none") : has);
    if (im.machine->counts().errors > 0) s += std::string(" | last error: ") + im.machine->lastError();
    if (!person.reason.empty()) s += " | planned because: " + person.reason;
    return s;
}

std::vector<BrainPlanLeg> CityBrain::planLegs(int who) const {
    std::vector<BrainPlanLeg> out;
    if (residentCount() == 0 || !impl_->isPerson(who)) return out;
    const Impl& im = *impl_;
    const brain::AgentBrain& a = im.machine->agent(static_cast<AgentId>(who));
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

bool CityBrain::goTo(int who, const std::string& marker, std::string* why) {
    Impl& im = *impl_;
    if (residentCount() == 0 || !im.isPerson(who)) {
        if (why) *why = residentCount() == 0 ? "this level has no resident with the agent brain" : "no such resident";
        return false;
    }
    const int at = markerNamed(marker);
    if (at < 0) {
        std::string names;
        for (const BrainMarker& m : im.markers) names += (names.empty() ? "" : ", ") + m.name;
        if (why) *why = "no marker \"" + marker + "\" (the markers: " + names + ")";
        return false;
    }
    Impl::Person& p = im.people[static_cast<std::size_t>(who)];
    p.dayOn = false;
    im.machine->setIntent(static_cast<AgentId>(who), p.intentKind, im.markerPlace(at, static_cast<AgentId>(who)));
    return true;
}

bool CityBrain::setTrait(int who, const std::string& name, bool on, std::string* why) {
    Impl& im = *impl_;
    const int t = im.travel.trait(name);
    if (residentCount() == 0 || !im.isPerson(who) || t < 0) {
        std::string names;
        for (const std::string& n : im.travel.traits) names += (names.empty() ? "" : ", ") + n;
        if (why) *why = residentCount() == 0 ? "this level has no resident with the agent brain"
                        : !im.isPerson(who) ? "no such resident"
                                            : "no trait \"" + name + "\" in travel.lua (the traits: " + names + ")";
        return false;
    }
    const AgentId id = static_cast<AgentId>(who);
    im.machine->setTraits(id, static_cast<uint8_t>(on ? im.machine->agent(id).traits | (1u << t) : im.machine->agent(id).traits & ~(1u << t)));
    return true;
}

engine::KerbSpace CityBrain::carSpace(int who) const {
    engine::KerbSpace space;
    const Impl& im = *impl_;
    const Impl::CarRec* c = im.carNumbered(residentCar(who));
    if (c && !im.onRoad(*c) && c->heldBy != kOutsider && im.isSpace(c->standing)) im.spaces->spaceOf(c->standing, space);
    return space;
}

std::string CityBrain::carSpaceName(int who) const {
    const engine::KerbSpace space = carSpace(who);
    return space.valid() ? impl_->model->kerbSpaceName(space) : std::string();
}

engine::KerbSpace CityBrain::heldSpace(int who) const {
    engine::KerbSpace space;
    if (!impl_->isPerson(who)) return space;
    const std::map<AgentId, uint32_t>::const_iterator h = impl_->held.find(static_cast<AgentId>(who));
    if (h == impl_->held.end()) return space;
    PlaceRef p;
    p.id = h->second;
    impl_->spaces->spaceOf(p, space);
    return space;
}

bool CityBrain::holdsSound(std::string* why) const {
    const Impl& im = *impl_;
    const auto fault = [&](const std::string& what) {
        if (why) *why = what;
        return false;
    };
    for (std::map<AgentId, uint32_t>::const_iterator h = im.held.begin(); h != im.held.end(); ++h) {
        PlaceRef space;
        space.id = h->second;
        if (h->first >= im.people.size()) return fault("somebody who is no resident holds " + im.placeName(space));
        const std::string& name = im.people[h->first].name;
        for (std::map<AgentId, uint32_t>::const_iterator other = std::next(h); other != im.held.end(); ++other)
            if (other->second == h->second)
                return fault(name + " and " + im.people[other->first].name + " both hold " + im.placeName(space));
        // the hold belongs to a leg of their plan that is still to be driven and ends in that space
        const brain::AgentBrain& a = im.machine->agent(h->first);
        bool legEndsThere = false;
        for (int i = a.legNow; i < a.legCount; ++i)
            legEndsThere = legEndsThere || (a.legHandle(i) != brain::kNoHandle && a.legEnd[i] == h->second);
        if (!legEndsThere) return fault(name + " holds " + im.placeName(space) + ", where no leg left in their plan ends a drive");
    }
    return true;
}

void CityBrain::occupyForTest(const engine::KerbSpace& space) {
    const PlaceRef place = impl_->spaces->placeOf(space);
    if (place.valid()) impl_->occupant[place.id] = kTestCar;
}

// ---- Somebody the brain does not own (ADR-0443) ---------------------------------------------------------------------------

int CityBrain::nearestFreeCar(Vec2 p, Real radius) const {
    const Impl& im = *impl_;
    int best = -1;
    Real bestD = radius;
    for (const Impl::CarRec& c : im.cars) {
        const CityCarLink& link = im.linkOf(c);
        if (link.driver >= 0 || link.away) continue;
        const Real d = (im.carPos(c) - p).length();
        if (d > bestD) continue;
        bestD = d;
        best = static_cast<int>(c.id);
    }
    return best;
}

bool CityBrain::board(int agent, int car) {
    Impl& im = *impl_;
    Impl::CarRec* c = im.machine ? im.carNumbered(car) : nullptr;
    if (!c || agent < 0 || agent >= static_cast<int>(im.sim.agents_.size())) return false;
    const Agent& a = im.sim.agents_[static_cast<std::size_t>(agent)];
    if (a.mode != Agent::Mode::Pedestrian || a.inCar >= 0 || a.vehicle >= 0) return false;
    // (nobody gets into a car with somebody at its wheel; and one of the brain's own people gets in by their plan,
    // unless a director has them)
    if (im.linkOf(*c).driver >= 0 || im.linkOf(*c).away || c->heldBy == kOutsider) return false;
    const int who = im.personOfAgent(agent);
    if (who >= 0 && !im.people[static_cast<std::size_t>(who)].heldByDirector) return false;
    LOG_INFO << "[brain] household car (car " << car << ") taken where it stood by sim agent " << agent;
    im.seat(agent, *c);
    im.outsiderTook(*c, agent, false);
    ++im.counts.takenStanding;
    return true;
}

bool CityBrain::alight(int agent) {
    Impl& im = *impl_;
    if (!im.machine || agent < 0 || agent >= static_cast<int>(im.sim.agents_.size())) return false;
    Impl::CarRec* c = im.carNumbered(im.sim.agents_[static_cast<std::size_t>(agent)].inCar);
    if (!c || c->heldBy != kOutsider || c->outsiderAgent != agent) return false;
    im.unseat(agent, *c, im.outOf(*c));
    im.outsiderLeft(*c);
    return true;
}

bool CityBrain::takenByPlayer(int car) {
    Impl& im = *impl_;
    Impl::CarRec* c = im.machine ? im.carNumbered(car) : nullptr;
    if (!c || c->heldBy == kOutsider) return false;
    const int who = c->aboard && c->heldBy < im.people.size() ? static_cast<int>(c->heldBy) : -1;
    if (who >= 0) {
        // THE DRIVER IS PUT OUT ON THE KERB SIDE of where the car is, and told the car is gone; what they do next is
        // their state's row to say
        const AgentId id = static_cast<AgentId>(who);
        Vec2 out;
        if (!im.kerbSideOf(im.carPos(*c), out)) out = im.outOf(*c);
        im.unseat(im.people[id].agent, *c, out);
        c->aboard = false;
        ++im.counts.putOut;
        im.machine->post(id, im.vehicleGone);
    }
    LOG_INFO << "[brain] household car (car " << car << ") taken by the player"
             << (who >= 0 ? " with " + im.people[static_cast<std::size_t>(who)].name + " at its wheel" : std::string());
    im.outsiderTook(*c, -1, true);
    ++im.counts.takenDriven;
    return true;
}

bool CityBrain::carLeftAt(int car, Vec2 pos, Vec2 heading) {
    Impl& im = *impl_;
    Impl::CarRec* c = im.machine ? im.carNumbered(car) : nullptr;
    if (!c || c->heldBy != kOutsider) return false;
    engine::Transform& t = im.poseOf(*c);
    setCarPose(t, pos, unit(heading, carHeading(t)), t.position.y);
    im.outsiderLeft(*c);
    return true;
}

bool CityBrain::directorHolds(int agent, bool on) {
    Impl& im = *impl_;
    const int who = im.machine ? im.personOfAgent(agent) : -1;
    if (who < 0) return false;
    const AgentId id = static_cast<AgentId>(who);
    Impl::Person& p = im.people[id];
    if (on == p.heldByDirector) return true;
    p.heldByDirector = on;
    if (on) {
        // the brain's own walk stops being stepped at once (the director may send them somewhere this very frame)
        p.walk = Impl::WalkPhase::None;
        p.steps.clear();
        Agent& a = im.agentOf(id);
        a.directedPace = 0;
        a.directedRide = Ride::None;
        ++im.counts.held;
        im.machine->post(id, im.possessed);
    } else {
        // let go: still the brain's (the possession channel has just handed them back to a day they do not keep)
        im.sim.setAgentDirected(agent, true);
        // out of whatever car the director had them in: one of the sim's, or a household's
        if (im.agentOf(id).vehicle >= 0) im.sim.alightVehicle(agent);
        if (im.agentOf(id).inCar >= 0) alight(agent);
        im.haltWalker(id);
        ++im.counts.letGo;
        im.machine->post(id, im.released);
    }
    return true;
}

// ---- The city sim's calls (declared in city_sim.h) ---------------------------------------------------------------------

bool CitySim::startBrain(engine::World& world, const std::vector<engine::CityMarker>& markers,
                         const std::vector<engine::CityResident>& residents, const std::vector<engine::CityHousehold>& households,
                         const std::vector<int>& classOfSlot, std::string* report) {
    std::shared_ptr<CityBrain> made = std::make_shared<CityBrain>(*this);
    if (!made->start(world, markers, residents, households, classOfSlot, report)) return false;
    brain_ = std::move(made);
    return true;
}

void CitySim::tickBrain(Real dt) { brain_->tick(dt); }
void CitySim::brainNoteHeld(int agent, bool on) {
    if (brain_) brain_->directorHolds(agent, on);
}

}  // namespace citysim
