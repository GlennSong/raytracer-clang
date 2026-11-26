#include "city_brain.h"

#include "brain_people.h"   // placeMarkerSpec: a place's marker (ADR-0491)
#include "places.h"

#include "city_car.h"
#include "city_driving.h"   // busService: a stop's wait, for the timetable (ADR-0484)
#include "city_sim.h"
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
#include <chrono>
#include <limits>
#include <deque>
#include <map>
#include <unordered_map>

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
// A BUS STOP (ADR-0484): its line (the next 12 bits up) and its place in the line (the low 12)
constexpr uint32_t kStopTag = 0x03000000u;
constexpr uint32_t kStopLineShift = 12;
constexpr uint32_t kStopIndexMask = 0x00000FFFu;
constexpr uint32_t kMarkerIndexMask = 0x0000FFFFu;
constexpr uint32_t kMarkerSpotShift = 16;
constexpr uint32_t kMarkerSpotMask = 0xFFu;
constexpr std::size_t kMostResidents = kMarkerSpotMask + 1;   // the spots one marker has (one person each)
static_assert(kMostResidents == kMarkerSpots, "city_brain.h kMarkerSpots is the spots a marker has");
// The first this many residents stand on the spot of their own number at every marker (Impl::takeSpot).
constexpr uint32_t kSpotsByNumber = 8;
// The car a test puts in a space (occupyForTest): no car of the households.
constexpr uint32_t kTestCar = brain::kMaxHandle;
// WHO HAS A CAR when it is nobody the brain owns: the player, or a walker a director took over (an OUTSIDER).
constexpr AgentId kOutsider = brain::kNoAgent - 1;
constexpr Real kMillis = 1000.0;
// A ROLE'S DAY WITH NOWHERE TO GO, OR NO WAY THERE, tries again after this many hours of the sim's clock (ADR-0491):
// the person stays in the state the table falls back to (home, work) for a quarter of an hour, not a tick.
constexpr double kDayRetryHours = 0.25;
// ...and no more than this many trips are begun in one tick (each is planned on the next): when a clock window opens
// for hundreds at once (river_town's start, 288 people: a 129 ms tick), the rest set out over the next ticks, as the
// sim's own day spends its departures (CitySim::kDeparturesPerStep).
constexpr int kDaySetOutsPerTick = 8;
// HOW MUCH ATTENTION A PERSON GETS (ADR-0493; one-brain plan, step 4): near the player (the sim's tier centre) they are
// NEAR -- skills move their body every tick; farther they are FAR -- no skill, their state ends by the brain's clock
// and they are visited only when it is due or an event comes. Raised inside the level's pedPromoteRadius, lowered
// beyond its pedDemoteRadius (the gap between is hysteresis). Each person's distance is looked at once every this
// many ticks, a share of them each tick.
constexpr uint32_t kDetailTurns = 30;
constexpr Real kSecondsPerMinute = 60.0;
// A car an outsider left at a point is put into the kerb lane beside it when the lane is empty from this far behind
// its rear to this far ahead of its front (a body's own room; nobody is waited for: it either fits or it does not).
constexpr Real kLeftRoom = 0.5;
// The events this class posts itself, by the names the data gives them (brain.lua `events`).
const char* const kVehicleGone = "vehicle_gone";
const char* const kPossessed = "possessed";
const char* const kReleased = "released";

// A FEW STEPS OFF THE PATHS for walker `who`, as a person takes them: out to the pavement on `from`'s side of its
// street, along to the pavement level with `to`, and in -- not straight along a front wall or across a corner. The
// pavement is the walkers' own (the verge they keep). The points to walk by, the next first, ending at `to`. (The
// one-agent trip lab's once; the walk skill's alone since that lab was deleted with the old driving, ADR-0459.)
std::vector<engine::Vec2> stepsOffThePaths(const CitySim& sim, int who, engine::Vec2 from, engine::Vec2 to) {
    (void)who;
    const engine::RoadModel* model = sim.roads();
    // (the sidewalk on p's side of the street it fronts: RoadModel::frontageAt, sidewalkBeside)
    auto pavement = [&](engine::Vec2 p) {
        if (!model || !model->built) return p;
        const engine::RoadFrontage f = model->frontageAt(p);
        engine::Vec2 out;
        return f.valid() && model->sidewalkBeside(f, out) ? out : p;
    };
    std::vector<engine::Vec2> steps;
    for (const engine::Vec2 p : {pavement(from), pavement(to), to})
        if ((p - (steps.empty() ? from : steps.back())).length() > 0.5) steps.push_back(p);
    if (steps.empty() || (steps.back() - to).length() > 1e-6) steps.push_back(to);
    return steps;
}

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

std::vector<engine::KerbSpace> kerbSpacesOffered(const engine::RoadModel& model, const engine::RoadFrontage& f,
                                                 const std::function<bool(const engine::KerbSpace&)>& keep) {
    std::vector<engine::KerbSpace> out;
    if (!f.street.valid()) return out;
    engine::KerbSpacePose pose;
    for (const engine::KerbSpaceNear& found : model.kerbSpacesNear(f, kSpaceReach, kSpacesLookedAt))
        if (model.kerbSpacePose(found.space, pose) && (!keep || keep(found.space))) out.push_back(found.space);
    return out;
}

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
    // the intents a day's trip to a seat, a group or a park wants (brain.lua; -1: the table has none, ADR-0512)
    int spotIntent = -1, groupIntent = -1, outsideIntent = -1;
    std::vector<BrainMarker> markers;
    std::unique_ptr<brain::RoadPlaces> spaces;
    std::vector<Vec2> points;   // where a car was left in a lane, by the low bits of a point place
    brain::EventId vehicleGone = brain::kNoId, possessed = brain::kNoId, released = brain::kNoId;

    // THE CARS, each one entity of the World (city_car.h), and everything about a car is on it (ADR-0455): its pose,
    // body, household, driver and whether the player has it, while the driving has it its CityCarOnLane, and the
    // brain's own keeping of it (CityCarKept: where it stands, who has it, what it is doing between its space and the
    // lane). A car's CarRef and its id in the driving are both its number (CityCarBody::id). Kept here are only the
    // entities, in the order they were made (the order every walk over the cars takes), and the first one's number:
    // the household cars are numbered in a row (start), so the car of a number is found at once (car), never by a walk.
    using CarPhase = CityCarPhase;
    using Car = engine::Entity;
    std::vector<Car> cars;
    uint32_t firstCar = 0;
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
        bool drives = true;   // false: their household's cars are not theirs to take (CityResident::drives)
        WalkPhase walk = WalkPhase::None;
        brain::WayId way = brain::kNoId;
        Vec2 target;
        std::vector<Vec2> steps;
        // THEIR DAY: the markers they go round, and how long they stay at each.
        std::vector<Stop> day;
        bool dayOn = false;
        int dayAt = -1;   // the stop they are at or going to (-1: not set out yet)
        // ...or THEIR ROLE'S DAY (ADR-0491): their agent's goal state on the sim's clock, the marker its trip goes to
        // (-1: not set out), and the venue it is for (-1: none)
        bool roleDay = false;
        int homeMarker = -1;
        int atMarker = -1;    // the marker their day last brought them to (home at the start)
        int dayMarker = -1;
        int dayVenue = -1;
        double dayRetryAt = 0;   // (the sim's total hours) after a NoRoute, nothing fires before this
        // ASLEEP IN A REST (ADR-0496): nothing can happen in their day before dayWakeAt (the sim's total hours), so it is
        // not read until then; daySleptAt is when they went under (their time in the state is caught up on waking)
        bool dayAsleep = false;
        double dayWakeAt = 0, daySleptAt = 0;
        // THE ATTENTION THEY GET (ADR-0493): the level they are stepped at, and when a far person is next due
        brain::Detail level = brain::Detail::Near;
        bool unseen = false;   // in a state they are out of sight in (showBody)
        bool waitingToSetOut = false;   // in setOutQueue
        uint64_t setOutTick = 0;        // the brain tick their day last began a trip (not read again that tick)
        BrainTime farWalkSince = brain::kNever;   // the walk whose route the crowd draws them along (farWalkDrawn)
        // the places (marker indices) they have found no way to, with every way they had to try ruled out (tickChoice)
        std::vector<uint32_t> noWayTo;
        brain::IntentKind intentKind = 0;
        bool heldByDirector = false;          // a director moves them (the brain's `possessed` state)
        std::string reason;                   // why their last plan was chosen, or why there was none
        std::vector<brain::Transition> trace; // every state they entered
    };
    std::vector<Person> people;

    CityBrainCounts counts;
    CityBrainCost cost;
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
    struct LabRide;
    std::unique_ptr<LabRide> ride;
    std::unique_ptr<LabDrive> drive;
    std::unique_ptr<brain::Machine> machine;

    // ---- the sim's own things -----------------------------------------------------------------------------------------
    Agent& agentOf(AgentId who) { return sim.agents_[static_cast<std::size_t>(people[who].agent)]; }
    const Agent& agentOf(AgentId who) const { return sim.agents_[static_cast<std::size_t>(people[who].agent)]; }
    // The household car numbered `ref` (not valid: none, or the car a test put in a space).
    Car car(CarRef ref) const {
        if (!ref.valid() || ref.id < firstCar || ref.id - firstCar >= cars.size()) return Car{};
        return cars[ref.id - firstCar];
    }
    Car carNumbered(int number) const {
        CarRef ref;
        if (number >= 0) ref.id = static_cast<uint32_t>(number);
        return car(ref);
    }
    // ---- the car's own components (city_car.h): every question about the car asks them -----------------------------
    engine::Transform& poseOf(Car c) const { return *ecs->get<engine::Transform>(c); }
    const CityCarBody& bodyOf(Car c) const { return *ecs->get<CityCarBody>(c); }
    CityCarLink& linkOf(Car c) const { return *ecs->get<CityCarLink>(c); }
    CityCarKept& keptOf(Car c) const { return *ecs->get<CityCarKept>(c); }
    uint32_t numberOf(Car c) const { return bodyOf(c).id; }
    Vec2 carPos(Car c) const { return carPosition(poseOf(c)); }
    Vec2 carFacing(Car c) const { return carHeading(poseOf(c)); }
    driving::CarId idOf(Car c) const { return driving::CarId(numberOf(c)); }
    // Its holder, a resident, is in it: at its wheel.
    bool aboard(Car c) const {
        const AgentId h = keptOf(c).heldBy;
        return h < people.size() && linkOf(c).driver >= 0 && linkOf(c).driver == people[h].agent;
    }
    // Is it on the driving's road?
    bool onRoad(Car c) const { return d->car(idOf(c)) != nullptr; }
    // HOW FAR FROM `at` THE DRIVING HAS THE CAR'S MIDDLE (it is on the road): where a car is handed between its kerb
    // space and the driving, the two should agree (CityBrainCounts::worstKerbHandOver).
    Real awayFromDriving(Car c, Vec2 at) const {
        const driving::Car* on = d->car(idOf(c));
        if (!on) return 0;
        const driving::CarPose pose = driving::carPose(*d->roads(), d->vehicleClasses(), d->ruleSet(), *on);
        const Vec3 mid = (pose.front + pose.rear) * 0.5;
        return (Vec2(mid.x, mid.z) - at).length();
    }
    // ITS LANE COMPONENT FOLLOWS THE DRIVING: added when the driving has just been given the car, taken away when the
    // driving no longer has it. Called after every placeCar and removeCar.
    void syncLane(Car c) {
        const bool has = ecs->has<CityCarOnLane>(c);
        if (onRoad(c) && !has) {
            CityCarOnLane lane;
            lane.car = idOf(c);
            lane.roadYFrom = lane.roadY = poseOf(c).position.y;
            ecs->add<CityCarOnLane>(c, lane);
        } else if (!onRoad(c) && has) {
            ecs->remove<CityCarOnLane>(c);
        }
    }
    // A PERSON IN A CAR IS WHERE THE CAR IS (#296, ADR-0445): whoever sits at a car's wheel has their own place and
    // heading set to the car's. One way: the car is the authority and nothing is written back. Called at the end of
    // every tick (after the driving and the slides between space and lane) and when they get in.
    void followCars() {
        for (const Car c : cars) {
            const int agent = linkOf(c).driver;
            // (a car of the near tier is a Vehicle its driver still drives, its Transform physics': followed too;
            // ADR-0448. One the player has is not the household's to follow.)
            if (agent < 0 || linkOf(c).away) continue;
            Agent& a = sim.agents_[static_cast<std::size_t>(agent)];
            a.pos = carPos(c);
            a.heading = carFacing(c);
        }
    }
    // THE BUS AND RAIL LINES AS THE PLANNER PRICES THEM (ADR-0484; step 4f): the sim's (CitySim::transit). A stop handle
    // names a line of it and a stop on that line.
    const TransitTimetable& transit() const { return sim.transit_; }
    bool stopKnown(PlaceRef p) const {
        return isStop(p) && stopLine(p) < transit().lines.size() && stopIndex(p) < transit().lines[stopLine(p)].stops.size();
    }
    static bool isStop(PlaceRef p) { return p.valid() && (p.id & kKindMask) == kStopTag; }
    static PlaceRef stopRef(uint32_t line, uint32_t stop) {
        PlaceRef p;
        p.id = kStopTag | (line << kStopLineShift) | (stop & kStopIndexMask);
        return p;
    }
    static uint32_t stopLine(PlaceRef p) { return (p.id & ~kKindMask) >> kStopLineShift; }
    static uint32_t stopIndex(PlaceRef p) { return p.id & kStopIndexMask; }
    // THE RIDERS' REPORTS (ADR-0484): the sim's riders (city_buses.h) say who of the brain's people got on and off; each
    // is posted to them as `boarded` or `alighted`
    void tickRides();
    bool isPerson(int who) const { return who >= 0 && static_cast<std::size_t>(who) < people.size(); }
    // WHO IS WHO, indexed (the island's 100k+ people: a search of everyone per question was 10^10 at the start)
    std::unordered_map<std::string, int> personByName;
    std::vector<int> personByAgent;   // the sim agent's person (-1: none)
    int personOfAgent(int agent) const {
        return agent >= 0 && static_cast<std::size_t>(agent) < personByAgent.size() ? personByAgent[static_cast<std::size_t>(agent)] : -1;
    }
    BrainTime now() const { return static_cast<BrainTime>(std::llround(sim.simSeconds_ * kMillis)); }
    Real lengthOf(Car c) const { return d->vehicleClasses().at(bodyOf(c).vehicleClass).length; }

    // ---- places -------------------------------------------------------------------------------------------------------
    static bool isMarker(PlaceRef p) { return p.valid() && (p.id & kKindMask) == kMarkerTag; }
    static bool isPoint(PlaceRef p) { return p.valid() && (p.id & kKindMask) == kPointTag; }
    static uint32_t markerIndex(PlaceRef p) { return p.id & kMarkerIndexMask; }
    static uint32_t markerSpot(PlaceRef p) { return (p.id >> kMarkerSpotShift) & kMarkerSpotMask; }
    bool isSpace(PlaceRef p) const { return spaces && spaces->owns(p); }
    // EACH PERSON'S SPOT AT A MARKER (ADR-0446), given once per person and marker, the first time they use it (the
    // level's residents in order, each their home and then their day's stops; later, whoever is sent there). The
    // first kSpotsByNumber residents take the spot of their own number, as ADR-0443 laid them out (the first resident
    // on the marker's own spot, the second kStandApart one way, the third the other way); everyone else, or one whose
    // numbered spot is taken, the lowest spot nobody has. (Every resident's own number put the two-hundredth 90 m along
    // the street from their marker.)
    std::map<std::pair<int, AgentId>, uint32_t> spotOf;
    std::vector<std::vector<uint8_t>> spotsTaken;   // per marker, per spot number
    std::vector<uint32_t> spotsShared;              // per marker, the people given a spot someone else has
    // THE MARKERS BY NAME, and the city's places (a role's day makes a place's marker the first time it goes there)
    std::unordered_map<std::string, int> markerByName;
    const PlaceMap* places = nullptr;
    // A MARKER from a level's statement, resolved against the road model: its index (the one there already by that
    // name), or -1 when no street or no sidewalk is beside it.
    int addMarker(const engine::CityMarker& spec);
    int markerForPlace(PlaceId id);
    int markerForNode(int node);
    // Their spot there: the one given, or (none given yet) the one they would be given now. MORE PEOPLE THAN SPOTS (a
    // tower's residents at its door, ADR-0493): everyone past the last free spot shares one, in turn (spotsShared counts
    // them). They are indoors there, out of sight; only when two are outside at once do they stand on one spot.
    uint32_t spotFor(int index, AgentId who) const {
        const std::map<std::pair<int, AgentId>, uint32_t>::const_iterator it = spotOf.find({index, who});
        if (it != spotOf.end()) return it->second;
        const bool known = index >= 0 && static_cast<std::size_t>(index) < spotsTaken.size();
        static const std::vector<uint8_t> kNone;
        const std::vector<uint8_t>& taken = known ? spotsTaken[static_cast<std::size_t>(index)] : kNone;
        const auto isFree = [&](uint32_t s) { return s >= taken.size() || !taken[s]; };
        uint32_t spot = who < kSpotsByNumber && isFree(who) ? static_cast<uint32_t>(who) : 0;
        while (spot < kMostResidents && !isFree(spot)) ++spot;
        if (spot < kMostResidents) return spot;
        const uint32_t shared = known && static_cast<std::size_t>(index) < spotsShared.size() ? spotsShared[static_cast<std::size_t>(index)] : 0;
        return shared % static_cast<uint32_t>(kMostResidents);
    }
    void takeSpot(int index, AgentId who) {
        if (index < 0 || index >= static_cast<int>(markers.size()) || spotOf.count({index, who}) > 0) return;
        const uint32_t spot = spotFor(index, who);
        if (spotsTaken.size() < markers.size()) spotsTaken.resize(markers.size());
        if (spotsShared.size() < markers.size()) spotsShared.resize(markers.size(), 0);
        std::vector<uint8_t>& taken = spotsTaken[static_cast<std::size_t>(index)];
        if (spot >= taken.size()) taken.resize(spot + 1, 0);
        if (taken[spot]) ++spotsShared[static_cast<std::size_t>(index)];
        taken[spot] = 1;
        spotOf[{index, who}] = spot;
    }
    // The marker `index` as the place `who` wants to be at: their own spot at it (takeSpot first). Not valid: no such
    // marker, or more people use it than a handle has spots for.
    PlaceRef markerPlace(int index, AgentId who) const {
        PlaceRef p;
        const uint32_t spot = spotFor(index, who);
        if (index >= 0 && index < static_cast<int>(markers.size()) && spot < kMostResidents)
            p.id = kMarkerTag | (spot << kMarkerSpotShift) | static_cast<uint32_t>(index);
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
        if (stopKnown(place)) {
            out = transit().lines[stopLine(place)].stops[stopIndex(place)].wait;
            return true;
        }
        return false;
    }
    std::string placeName(PlaceRef place) const {
        if (isMarker(place) && markerIndex(place) < markers.size()) return markers[markerIndex(place)].name;
        if (stopKnown(place)) {
            const TransitTimetable::Line& tl = transit().lines[stopLine(place)];
            const std::size_t src = static_cast<std::size_t>(std::max(0, tl.source));
            if (tl.network == "bus" && model && src < model->busLines.size() && stopIndex(place) < model->busLines[src].stops.size())
                return "stop " + model->busLines[src].stops[stopIndex(place)].name + " (line " + model->busLines[src].id + ")";
            if (tl.network == "rail" && src < sim.rail_.lines().size() && stopIndex(place) < sim.rail_.lines()[src].stops.size())
                return "station " + sim.rail_.lines()[src].stops[stopIndex(place)].name + " (rail line " + sim.rail_.lines()[src].name + ")";
        }
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
    // A KERB SPACE A CAR OF CLASS `cls` CAN DRIVE AWAY FROM AND BACK TO (ADR-0446): the kerb lane beside it lies in
    // the largest part of the road the class can drive (driving/trip_places.h: every lane of a part reaches every
    // other). A space beside a road that just ends at the lab's open edge, or one the class's routes cannot get out
    // of, is one a car parked in is stuck in, or a trip from it has no route and the car is left standing in the lane.
    struct ClassParts {
        driving::TripPlaces places;
        uint32_t main = driving::kNoTripPart;
    };
    mutable std::map<uint32_t, ClassParts> partsOfClass;
    bool drivable(PlaceRef space, uint32_t cls) const {
        if (!drivableSpacesOnly) return true;
        std::map<uint32_t, ClassParts>::iterator it = partsOfClass.find(cls);
        if (it == partsOfClass.end()) {
            ClassParts cp;
            cp.places = driving::findTripPlaces(*d->roads(), d->routeClass(cls), kKerbLaneEndMargin, kSmallestTripPart);
            Real most = 0;
            for (uint32_t p = 0; p < cp.places.parts.size(); ++p)
                if (cp.places.parts[p].total > most) {
                    most = cp.places.parts[p].total;
                    cp.main = p;
                }
            it = partsOfClass.emplace(cls, std::move(cp)).first;
        }
        driving::PathPoint lane;
        return spaces->kerbLane(space, lane) && it->second.places.partOf(lane.path) == it->second.main;
    }
    // THE LEVEL ASKS FOR IT (citysim.drivableSpacesOnly; brain_crowd does): without it every kerb space is offered, as
    // brain_lab was built and tested (one of its spaces, by `far`, is outside the largest part).
    bool drivableSpacesOnly = false;
    // The driving class a car is driven as (class 0 for a car the brain does not have).
    uint32_t classOf(CarRef ref) const {
        const Car c = car(ref);
        return c.valid() ? bodyOf(c).vehicleClass : 0u;
    }
    // The kerb spaces near a place a car of class `cls` can drive from and to, nearest first on foot.
    std::vector<PlaceRef> spacesFor(PlaceRef door, uint32_t cls) const {
        std::vector<PlaceRef> out;
        engine::RoadFrontage frontage;
        Vec2 at;
        if (isMarker(door) && markerIndex(door) < markers.size()) frontage = markers[markerIndex(door)].frontage;
        else if (positionOf(door, at)) frontage = model->frontageAt(at);
        if (!frontage.street.valid()) return out;
        for (const engine::KerbSpace& space : kerbSpacesOffered(*model, frontage, [&](const engine::KerbSpace& s) {
                 const PlaceRef place = spaces->placeOf(s);
                 return place.valid() && drivable(place, cls);
             }))
            out.push_back(spaces->placeOf(space));
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
    void leaveEverySpace(Car c) {
        const uint32_t number = numberOf(c);
        for (std::map<uint32_t, uint32_t>::iterator it = occupant.begin(); it != occupant.end();)
            it = it->second == number ? occupant.erase(it) : std::next(it);
    }
    // THE HOUSEHOLD IS TOLD ITS CAR HAS CHANGED (it is free, it has moved, it is gone): everyone but `except`. Only a
    // person standing with no way there acts on it (brain.lua `things_changed`).
    void tellHousehold(Car c, AgentId except) {
        const int household = linkOf(c).household;
        if (!machine || household < 0) return;
        for (const AgentId m : households[static_cast<std::size_t>(household)].members)
            if (m != except) machine->notifyChanged(m);
    }

    // ---- getting in and out -------------------------------------------------------------------------------------------
    bool getIn(AgentId who, Car c);
    void getOut(AgentId who, Car c);
    // Anybody into the car's driver's seat (they are in it from now: not drawn on foot), and out of it again, stood at
    // `out`.
    void seat(int agent, Car c);
    void unseat(int agent, Car c, Vec2 out);
    // Where a person getting out of the car stands: on the kerb side (beside its space, or beside the road where it
    // is), or, failing that, a step to its side clear of the lanes.
    Vec2 outOf(Car c) const;
    void writeCarPose(Car c, Vec2 pos, Vec2 heading);
    void standAt(Car c, Vec2 pos, Vec2 heading);
    void haltWalker(AgentId who);
    void tickWalks(Real dt);
    void tickCars();
    void tickDay(AgentId who);
    void tickRoleDay(AgentId who, Real dtHours);
    std::string traceTail(AgentId who) const {
        std::string s;
        const std::vector<brain::Transition>& tr = people[who].trace;
        for (std::size_t i = tr.size() > 8 ? tr.size() - 8 : 0; i < tr.size(); ++i)
            s += table.states[tr[i].state].name + "@" + std::to_string(tr[i].at) + " ";
        return s;
    }
    // THE MACHINE, stepped by attention (ADR-0493): events answered; then near people every tick, far people when due
    void tickMachine();
    void setLevel(AgentId who, brain::Detail level);
    // THE SCANS' COMPACT COPIES, one per person (ADR-0496: a scan of the island's 115k Person records cost about 5 ms a
    // tick): whether they are far, when a far one is next due, and the sim's total hours before which their day need
    // not be read (asleep in a rest: their wake; a far person on their way: never, until the brain visits them)
    std::vector<uint8_t> farOf;
    std::vector<BrainTime> dueOf;
    std::vector<double> dayGateOf;
    std::vector<AgentId> changedNow;
    std::deque<AgentId> setOutQueue;
    // ...and those of them near the player, served first (ADR-0514): the streets round the player fill in seconds at
    // load, not after the whole island's morning has gone out
    std::deque<AgentId> nearSetOutQueue;
    uint64_t tickNumber = 0;   // people whose day has a trip to begin, waiting for a turn (first come first served)
    brain::Detail detailWanted(AgentId who) const;
    void placeFarBody(AgentId who, bool justLowered = false);
    void farWalkDrawn(AgentId who);
    void raiseBody(AgentId who);
    Vec2 farWalkPoint(const Agent& a) const;
    void showBody(AgentId who);
    uint32_t detailTurn = 0;
    int daySetOutsThisTick = 0;
    void setOut(AgentId who, const GoalState& s);
    bool fireDay(AgentId who, GoalEvent e);
    void tickChoice(AgentId who);
    void parkInSpace(Car c, PlaceRef space);
    void towAway(Car c);
    // THE FAR CAR TIER (ADR-0516): the car leaves its space (and the road) while the clock drives it; it stands again
    // in the free kerb space nearest `at` (true), or, none near, at `at`, stranded (false)
    void sendAway(Car c);
    bool standInSpaceNear(Car c, Vec2 at);
    // `who` is far and drives a car that is away: they stay far until it can be put on their route (step 3)
    bool drivingAway(AgentId who) const;
    void shareOutTrace();
    // ---- somebody the brain does not own ------------------------------------------------------------------------------
    void outsiderTook(Car c, bool lost);
    void outsiderLeft(Car c);
    std::string carLine(Car c) const;
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
    FrontPoints& of(Car c) {
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
    Real kerbOffset(PlaceRef place) const override {
        driving::PathPoint mid;
        Vec2 at;
        return im.spaces->kerbLane(place, mid) && im.spaces->positionOf(place, at) ? im.d->offsetBeside(mid, at) : Real(0);
    }
};

// ---- The planner, with each person's reason kept ------------------------------------------------------------------------
// THE trip planner (brain_planner.h), asked as the machine asks it; what it said about each person's last plan is kept
// under that person, so `brain? <who>` can say why (one planner serves everybody, and its own last reason is whoever
// planned last).
struct CityBrain::Impl::NotingPlanner : brain::Planner {
    NotingPlanner(Impl& i, const TripNetwork& net, const TravelCatalog& ways, const brain::World& world)
        : im(i), inner(net, ways, world) {}
    Impl& im;
    brain::TripPlanner inner;
    bool plan(const brain::PlanAsk& ask, brain::PlanLegs& out) override {
        const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        const bool ok = inner.plan(ask, out);
        const double planMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        im.cost.planMs += planMs;
        im.cost.worstPlanMs = std::max(im.cost.worstPlanMs, planMs);
        ++im.cost.plans;
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
    uint32_t carClass(CarRef car) const override { return im.classOf(car); }
    bool stopPlace(uint32_t line, uint32_t stop, PlaceRef& out) const override {
        if (line >= im.transit().lines.size() || stop >= im.transit().lines[line].stops.size()) return false;
        out = Impl::stopRef(line, stop);
        return true;
    }
    BrainTime rideWait(uint32_t line, PlaceRef board) const override {
        if (!im.stopKnown(board) || Impl::stopLine(board) != line) return 0;
        const TransitTimetable::Line& tl = im.transit().lines[line];
        const Real s = 0.5 * tl.headwaySeconds + tl.stops[Impl::stopIndex(board)].accessSeconds;
        return static_cast<BrainTime>(std::llround(s * 1000.0));
    }
    bool carOnLane(CarRef ref, engine::PathPoint& out) const override {
        const Car c = im.car(ref);
        const driving::Car* dc = c.valid() ? im.d->car(im.idOf(c)) : nullptr;
        if (!dc) return false;
        out.path = dc->path;
        out.along = dc->along;
        return true;
    }
    bool bodyPosition(AgentId who, Vec2& out) const override {
        if (who >= im.people.size()) return false;
        out = im.agentOf(who).pos;   // (in a car: where the car is, Impl::followCars)
        return true;
    }
    // WHICH OF THEIR HOUSEHOLD'S CARS THEY WOULD TAKE NOW: the free one standing nearest them; when none is free, the
    // one they had in mind (so the plan can say "in use"), or the household's first.
    CarRef carOffered(AgentId who, CarRef has, Vec2 from) const override {
        if (who >= im.people.size() || im.people[who].household < 0) return has;
        if (!im.people[who].drives) return CarRef{};   // (a child, a student: no car is theirs)
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
        const Car c = im.car(ref);
        if (!c.valid()) return false;
        const CityCarKept& k = im.keptOf(c);
        // (a car an outsider has stands nowhere a resident could take it from; nor does one left where no lane is, nor
        // one away with its far driver)
        if (!k.standing.valid() || k.heldBy == kOutsider || k.stranded || k.phase == CarPhase::Away) return false;
        out = k.standing;
        return true;
    }
    bool carFreeFor(CarRef ref, AgentId who) const override {
        const Car c = im.car(ref);
        return c.valid() && (im.keptOf(c).heldBy == brain::kNoAgent || im.keptOf(c).heldBy == who);
    }
    bool claimCar(CarRef ref, AgentId who) override {
        const Car c = im.car(ref);
        if (!c.valid()) return false;
        CityCarKept& k = im.keptOf(c);
        if (k.heldBy != brain::kNoAgent && k.heldBy != who) return false;
        k.heldBy = who;
        return true;
    }
    void releaseCar(CarRef ref, AgentId who) override {
        const Car c = im.car(ref);
        if (!c.valid() || im.keptOf(c).heldBy != who) return;
        im.getOut(who, c);
        CityCarKept& k = im.keptOf(c);
        k.heldBy = brain::kNoAgent;
        k.tripWanted = false;
        if (k.phase == CarPhase::WaitingForRoom) k.phase = CarPhase::Standing;
        im.tellHousehold(c, who);
    }
    void leaveCar(CarRef ref, AgentId who) override {
        const Car c = im.car(ref);
        if (!c.valid() || im.keptOf(c).heldBy != who) return;
        // on the road it stays where it is, placed and undriven, and stands at a place of its own
        if (im.onRoad(c)) im.keptOf(c).standing = im.pointPlace(im.carPos(c));
        // AWAY WITH ITS FAR DRIVER (ADR-0516), who gives the drive up: it stands in the free kerb space nearest where the
        // clock has them (asked before the state is left), or, none near, there, stranded
        if (im.keptOf(c).phase == CarPhase::Away) {
            im.standInSpaceNear(c, im.machine->whereIs(who, im.now()));
            ++im.counts.awayLeft;
        }
        im.getOut(who, c);
        CityCarKept& k = im.keptOf(c);
        k.heldBy = brain::kNoAgent;
        k.tripWanted = false;
        k.phase = CarPhase::Standing;
        im.held.erase(who);   // (a space they were driving to is free again)
        im.tellHousehold(c, who);
    }
    void carDrivenByClock(CarRef ref, AgentId who) override {
        const Car c = im.car(ref);
        if (c.valid() && im.keptOf(c).heldBy == who && im.keptOf(c).phase != CarPhase::Away) im.sendAway(c);
    }
    void parkCar(CarRef ref, AgentId who, PlaceRef space) override {
        const Car c = im.car(ref);
        if (!c.valid()) return;
        im.keptOf(c).standing = space;
        im.occupant[space.id] = ref.id;
        im.getOut(who, c);
        if (im.keptOf(c).heldBy == who) im.keptOf(c).heldBy = brain::kNoAgent;
        im.held.erase(who);
        im.tellHousehold(c, who);
    }
    bool spaceNear(PlaceRef door, CarRef ref, PlaceRef& out) const override {
        for (const PlaceRef space : im.spacesFor(door, im.classOf(ref)))
            if (im.spaceFree(space, ref, brain::kNoAgent)) {
                out = space;
                return true;
            }
        return false;
    }
    bool takeSpace(PlaceRef space, CarRef ref) override {
        const Car c = im.car(ref);
        if (!c.valid() || !im.isSpace(space) || !im.spaceFree(space, ref, im.keptOf(c).heldBy)) return false;
        im.leaveEverySpace(c);
        im.occupant[space.id] = ref.id;
        im.parkInSpace(c, space);
        return true;
    }
    void holdSpace(PlaceRef space, AgentId who) override {
        if (im.isSpace(space)) im.held[who] = space.id;
    }
    void dropHold(AgentId who) override { im.held.erase(who); }
    bool nextSpace(PlaceRef door, CarRef ref, AgentId who, PlaceRef taken, PlaceRef& out) const override {
        for (const PlaceRef space : im.spacesFor(door, im.classOf(ref)))
            if (space != taken && im.spaceFree(space, ref, who)) {
                out = space;
                ++im.counts.nextSpaces;
                return true;
            }
        return false;
    }
};

// ---- Walking: the sim's own -------------------------------------------------------------------------------------------
// ---- Riding: the brain's RideSkill, on the buses of the road file's lines (ADR-0484): the sim's riders carry them -------
struct CityBrain::Impl::LabRide : brain::RideSkill {
    explicit LabRide(Impl& i) : im(i) {}
    Impl& im;

    // (a line of the transit timetable: a bus line, carried by the sim's bus riders, or a rail line, by its train riders)
    const TransitTimetable::Line* lineOf(uint32_t line) const {
        return line < im.transit().lines.size() ? &im.transit().lines[line] : nullptr;
    }
    bool waitFor(AgentId who, uint32_t line, PlaceRef board, PlaceRef alight) override {
        const TransitTimetable::Line* tl = lineOf(line);
        if (who >= im.people.size() || !tl || tl->source < 0 || !Impl::isStop(board) || !Impl::isStop(alight) ||
            Impl::stopLine(board) != line || Impl::stopLine(alight) != line)
            return false;
        const int agent = im.people[who].agent;
        if (tl->network == "rail") {
            // in at the station's way in (the walk there ended at it): the train riders take them from the street
            if (!im.sim.trainRiders_.wait(agent, tl->source, static_cast<int>(Impl::stopIndex(board)), static_cast<int>(Impl::stopIndex(alight))))
                return false;
            ++im.counts.trainWaits;
            return true;
        }
        if (!im.sim.buses_.wait(agent, static_cast<uint32_t>(tl->source), Impl::stopIndex(board), Impl::stopIndex(alight))) return false;
        im.agentOf(who).indoors = false;   // (standing at the stop: the walk there left them on its waiting place)
        ++im.counts.busWaits;
        return true;
    }
    bool aboard(AgentId who, uint32_t line, PlaceRef alight) override {
        const TransitTimetable::Line* tl = lineOf(line);
        if (who >= im.people.size() || !tl || tl->source < 0 || !Impl::isStop(alight) || Impl::stopLine(alight) != line) return false;
        const int agent = im.people[who].agent;
        if (tl->network == "rail") {
            if (!im.sim.trainRiders_.aboard(agent, tl->source, static_cast<int>(Impl::stopIndex(alight)))) return false;
        } else if (!im.sim.buses_.aboard(agent, static_cast<uint32_t>(tl->source), Impl::stopIndex(alight))) {
            return false;
        }
        im.agentOf(who).indoors = true;
        return true;
    }
    void cancel(AgentId who) override {
        if (who >= im.people.size()) return;
        im.sim.buses_.cancel(im.people[who].agent, im.sim.agents_);
        im.sim.trainRiders_.cancel(im.people[who].agent, im.sim.agents_);
    }
};

void CityBrain::Impl::tickRides() {
    if (sim.brainRideEvents_.empty()) return;
    const brain::EventId boarded = static_cast<brain::EventId>(table.event("boarded"));
    const brain::EventId alighted = static_cast<brain::EventId>(table.event("alighted"));
    for (const RideEvent& e : sim.brainRideEvents_) {
        const int who = personOfAgent(e.agent);
        if (who < 0) continue;
        if (!e.boarded) ++(e.train ? counts.trainRides : counts.busRides);
        machine->post(static_cast<AgentId>(who), e.boarded ? boarded : alighted);
    }
    sim.brainRideEvents_.clear();
}

struct CityBrain::Impl::LabWalk : brain::WalkSkill {
    explicit LabWalk(Impl& i) : im(i) {}
    Impl& im;

    bool walk(AgentId who, brain::WayId way, Vec2, PlaceRef to, Real) override {
        if (who >= im.people.size() || way >= im.travel.modes.size()) return false;
        Person& p = im.people[who];
        Agent& a = im.agentOf(who);
        Vec2 target;
        if (a.inCar >= 0 || !im.walkTarget(to, target)) return false;
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
        // (each end joins the walks from its point: where the person stands, and the place's stand, itself put on its
        // sidewalk from the marker's stated frontage, sidewalkBeside. Joined from the marker's frontage instead (W3, tried
        // and measured), a walk could end on the far kerb or past where its sidewalk stops, and the last steps, walked
        // straight, crossed the carriageway: the near crowd's cars hit walkers 27 times, against 1.)
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
    a.moving = false;
    a.speed = 0;
    a.route = engine::roads::WalkRoute{};
    a.leg = 0;
    a.distOnLeg = 0;
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
        if (state.skill != "walk") {
            // A WALK LEFT RUNNING when they are no longer walking (seen on the island: 26 in two minutes, #338); its report
            // would be an event their state does not have. Not reported; the first few are logged to find the cause.
            ++counts.staleWalks;
            if (counts.staleWalks <= 5)
                LOG_WARN << "[brain] " << p.name << ": a walk finished while in " << state.name << " (" << (p.level == brain::Detail::Far ? "far" : "near")
                         << "); the states before: " << traceTail(who);
            continue;
        }
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
        const Car c = im.car(ref);
        if (!c.valid() || im.keptOf(c).heldBy == kOutsider) return false;
        const AgentId holder = im.keptOf(c).heldBy;
        if (holder != brain::kNoAgent && !im.aboard(c) && !im.getIn(holder, c)) return false;
        if (im.onRoad(c)) return true;   // left in a lane by an earlier trip: it sets off from there
        driving::PathPoint front;
        return im.fronts->of(c).kerbLane(space, front);
    }
    bool beginTrip(CarRef ref, AgentId person, PlaceRef fromSpace, PlaceRef toSpace, Real) override {
        const Car c = im.car(ref);
        driving::PathPoint to;
        if (!c.valid() || im.keptOf(c).heldBy == kOutsider || !im.fronts->of(c).kerbLane(toSpace, to)) return false;
        CityCarKept& k = im.keptOf(c);
        k.tripPerson = person;
        k.tripFrom = fromSpace;
        k.tripTo = toSpace;
        // (the space it drives to is held for its driver from now, so nobody else pulls into it first)
        if (im.isSpace(toSpace) && person != brain::kNoAgent) im.held[person] = toSpace.id;
        if (im.onRoad(c)) {
            k.phase = CarPhase::OnRoad;
            k.tripWanted = false;
            if (!im.skill->beginTrip(ref, person, fromSpace, toSpace, 0)) return false;
            ++im.counts.tripsBegun;
            return true;
        }
        // in its space: it is placed beside the kerb lane when the lane is clear, and the trip begins then (it drives
        // out of the space on the driving's curve)
        k.tripWanted = true;
        if (k.phase == CarPhase::Standing) k.phase = CarPhase::WaitingForRoom;
        return true;
    }
    void endTrip(CarRef ref) override {
        const Car c = im.car(ref);
        if (!c.valid() || im.keptOf(c).heldBy == kOutsider) return;
        CityCarKept& k = im.keptOf(c);
        k.tripWanted = false;
        if (k.phase == CarPhase::WaitingForRoom) k.phase = CarPhase::Standing;
        im.skill->endTrip(ref);
    }
    // The car stands in `space` from now: off the road if it was still on it, and exactly where a car in the space
    // stands (the slide in, if one was under way, is over).
    void park(CarRef ref, PlaceRef space) override {
        const Car c = im.car(ref);
        if (!c.valid() || im.keptOf(c).heldBy == kOutsider) return;
        if (im.onRoad(c)) {
            im.skill->endTrip(ref);
            im.fronts->of(c);
            im.skill->park(ref, space);
            im.syncLane(c);
        }
        engine::KerbSpacePose pose;
        Vec2 kerbward;
        if (im.spacePose(space, pose, kerbward)) im.writeCarPose(c, pose.at, pose.facing);
        CityCarKept& k = im.keptOf(c);
        k.phase = CarPhase::Standing;
        k.tripWanted = false;
    }
    // The trip now ends beside `toSpace`. A car that has not left its space yet sets off for there when it does; one
    // still sliding out has its trip begun to there when the slide ends; one on the road has its trip's end changed.
    bool changeDestination(CarRef ref, PlaceRef toSpace) override {
        const Car c = im.car(ref);
        driving::PathPoint to;
        if (!c.valid() || im.keptOf(c).heldBy == kOutsider || !im.fronts->of(c).kerbLane(toSpace, to)) return false;
        CityCarKept& k = im.keptOf(c);
        if (k.tripWanted) {
            k.tripTo = toSpace;
            ++im.counts.redirected;
            return true;
        }
        if (!im.onRoad(c) || !im.skill->changeDestination(ref, toSpace)) return false;
        k.tripTo = toSpace;
        if (im.isSpace(toSpace) && k.tripPerson != brain::kNoAgent) im.held[k.tripPerson] = toSpace.id;
        ++im.counts.redirected;
        return true;
    }
    bool waitingToSetOff(CarRef ref) const override {
        const Car c = im.car(ref);
        return c.valid() && im.keptOf(c).phase == CarPhase::WaitingForRoom;
    }
};

// THE PERSON WHO HAS THE CAR GETS IN: into its driver's seat, which nobody else is in.
bool CityBrain::Impl::getIn(AgentId who, Car c) {
    const Agent& a = agentOf(who);
    if (a.inCar >= 0 || linkOf(c).driver >= 0) return false;
    // NO WALK GOES ON IN THE CAR. A walk of no length (the person already beside the car) is ended by the machine on
    // the spot while the walk skill still has it; stepped on from inside the car, it "arrived" seconds later and its
    // report ended whatever state the person was in by then (pulling_out, 3.6 s into its two minutes). With the person
    // a driver of a sim vehicle (before ADR-0444) the sim refused to step them, which hid it.
    haltWalker(who);
    seat(people[who].agent, c);   // (aboard from now: its holder's agent is at its wheel)
    ++counts.boarded;
    return true;
}

// ...AND GETS OUT, leaving the car exactly where it stands: out on the kerb side.
void CityBrain::Impl::getOut(AgentId who, Car c) {
    if (!aboard(c) || who >= people.size()) return;
    unseat(people[who].agent, c, outOf(c));
    ++counts.alighted;
}

// IN THE DRIVER'S SEAT: the person is in the car from now (Agent::inCar), not drawn on foot, and not one of the
// walkers anybody senses; their own place follows the car's from now (followCars).
void CityBrain::Impl::seat(int agent, Car c) {
    Agent& a = sim.agents_[static_cast<std::size_t>(agent)];
    a.inCar = static_cast<int>(numberOf(c));
    a.pos = carPos(c);
    a.heading = carFacing(c);
    a.tickFromPos = a.pos;   // (a placement, not a motion)
    a.tickFromHeading = a.heading;
    a.indoors = false;
    a.moving = false;
    a.speed = 0;
    a.route = engine::roads::WalkRoute{};
    a.leg = 0;
    a.distOnLeg = 0;
    a.directedPace = 0;
    a.directedRide = Ride::None;
    a.state = Agent::State::Waiting;
    linkOf(c).driver = agent;
    sim.grid_.remove(agent);
}

void CityBrain::Impl::unseat(int agent, Car c, Vec2 out) {
    Agent& a = sim.agents_[static_cast<std::size_t>(agent)];
    a.inCar = -1;
    a.pos = out;
    a.heading = carFacing(c);
    a.tickFromPos = out;
    a.tickFromHeading = a.heading;
    a.moving = false;
    a.speed = 0;
    a.route = engine::roads::WalkRoute{};
    a.leg = 0;
    a.distOnLeg = 0;
    a.state = Agent::State::Resting;
    if (linkOf(c).driver == agent) linkOf(c).driver = -1;
    sim.grid_.place(agent, out);
}

Vec2 CityBrain::Impl::outOf(Car c) const {
    Vec2 out;
    const Vec2 at = carPos(c);
    // beside its space; or, left in a lane (or nowhere a space is), on the sidewalk beside the road there
    if (!onRoad(c) && doorPoint(keptOf(c).standing, out)) return out;
    if (kerbSideOf(at, out)) return out;
    // (no sidewalk near: a step to the car's side, clear of the lanes, as the sim stands anyone getting out)
    const Vec2 h = carFacing(c);
    return sim.pushPoseClearOfLanes(at + Vec2(h.y, -h.x) * 2.0, 1.0);
}

// THE CAR IS AT `pos`, POINTING ALONG `heading` (drawn sliding between its space and the lane, or put down). Its height
// is left as it was: off the road the drawing stands it on the ground under it.
void CityBrain::Impl::writeCarPose(Car c, Vec2 pos, Vec2 heading) {
    engine::Transform& t = poseOf(c);
    setCarPose(t, pos, heading, t.position.y);
    keptOf(c).spaceFront = driving::PathPoint();   // (moved by hand: not where the driving left it)
}

// ...PUT DOWN THERE: drawn from there this tick too (its PrevTransform with it), not slid there from where it was.
void CityBrain::Impl::standAt(Car c, Vec2 pos, Vec2 heading) {
    writeCarPose(c, pos, heading);
    if (engine::PrevTransform* prev = ecs->get<engine::PrevTransform>(c)) prev->value = poseOf(c);
}

// The space is the car's: it leaves the driving where the driving left it. Its trip ended there: the driving drove it
// in on its curve across (Driving::beginTrip's pullInOffset) and it came to rest in the space, its front within the
// arrival distance of the space's own place; it stays exactly there, and pulls out from there (CityCarKept::spaceFront).
// A car not in the space -- a trip begun too near it to pull in ended in the lane (the driving's
// KerbMoveCounts::stoppedInLane), or one off the road -- is put where a car in the space stands, and counted.
void CityBrain::Impl::parkInSpace(Car c, PlaceRef space) {
    engine::KerbSpacePose pose;
    Vec2 kerbward;
    if (!spacePose(space, pose, kerbward)) return;
    CityCarKept& kept = keptOf(c);
    if (kept.phase == CarPhase::Away) {
        // A FAR DRIVE ENDS (ADR-0516): the car the clock drove stands in the space, where a car in it stands, and is
        // drawn again from there (no slide from where it last stood)
        standAt(c, pose.at, pose.facing);
        kept.phase = CarPhase::Standing;
        kept.standing = space;
        ++counts.farParked;
        ++counts.parkedInSpace;
        return;
    }
    const driving::CarId id = idOf(c);
    driving::PathPoint front;
    bool pulledIn = false;
    if (const driving::Car* on = d->car(id)) {
        pulledIn = std::fabs(on->offset - fronts->of(c).kerbOffset(space)) <= kInSpaceWithin;
        front.path = on->path;
        front.along = on->along;
        CarRef ref;
        ref.id = id.index;
        skill->endTrip(ref);   // (harmless: the trip ended when the driving reported "parked")
        d->removeCar(id);
        syncLane(c);
    }
    if (pulledIn) {
        keptOf(c).spaceFront = front;
    } else {
        counts.worstKerbHandOver = std::max(counts.worstKerbHandOver, (carPos(c) - pose.at).length());
        ++counts.putInSpace;
        writeCarPose(c, pose.at, pose.facing);
    }
    CityCarKept& k = keptOf(c);
    k.phase = CarPhase::Standing;
    k.standing = space;
    ++counts.parkedInSpace;
}

// A WRECK CLEARED (sitting 7; two-drivers-meeting-design.md section 8): the driving has taken it off the road; it is
// towed to the nearest free kerb space to where it stood (a tow truck later), and its household is told. Nobody holds
// it: its driver got out at the crash and went on without it. No free space near: it stands where it was, stranded.
void CityBrain::Impl::towAway(Car c) {
    syncLane(c);
    CityCarKept& k = keptOf(c);
    k.phase = CarPhase::Standing;
    k.tripWanted = false;
    k.spaceFront = driving::PathPoint();
    if (standInSpaceNear(c, carPos(c))) ++counts.towed;
    tellHousehold(c, brain::kNoAgent);
}

// THE CAR STANDS IN THE FREE KERB SPACE NEAREST `at` (a wreck towed; a car away with its far driver, who gave the drive
// up): where a car in that space stands. None near: it stands at `at`, stranded (nobody can drive it from there). Its
// phase is Standing either way.
bool CityBrain::Impl::standInSpaceNear(Car c, Vec2 at) {
    CityCarKept& k = keptOf(c);
    k.phase = CarPhase::Standing;
    const uint32_t number = numberOf(c);
    CarRef ref;
    ref.id = number;
    for (const PlaceRef space : spaces->spacesNear(at, kSpaceReach, kSpacesLookedAt)) {
        if (!spaceFree(space, ref, brain::kNoAgent)) continue;
        engine::KerbSpacePose pose;
        Vec2 kerbward;
        if (!spacePose(space, pose, kerbward)) continue;
        leaveEverySpace(c);
        occupant[space.id] = number;
        standAt(c, pose.at, pose.facing);
        k.standing = space;
        k.stranded = false;
        return true;
    }
    standAt(c, at, carFacing(c));
    k.standing = pointPlace(at);
    k.stranded = true;
    return false;
}

// AWAY (ADR-0516): its far driver has set off and the clock drives it. It leaves its kerb space (free for another car
// from now) and, left in a lane by an earlier trip, the road; nothing draws it or sees it until it stands again.
void CityBrain::Impl::sendAway(Car c) {
    const driving::CarId id = idOf(c);
    if (d && d->car(id)) {
        CarRef ref;
        ref.id = id.index;
        skill->endTrip(ref);
        d->removeCar(id);
        syncLane(c);
    }
    leaveEverySpace(c);
    CityCarKept& k = keptOf(c);
    k.phase = CarPhase::Away;
    k.tripWanted = false;
    k.stranded = false;
    k.spaceFront = driving::PathPoint();
    ++counts.drivenByClock;
}

bool CityBrain::Impl::drivingAway(AgentId who) const {
    const brain::AgentBrain& b = machine->agent(who);
    CarRef ref;
    ref.id = b.car;
    const Car c = car(ref);
    return c.valid() && keptOf(c).heldBy == who && keptOf(c).phase == CarPhase::Away;
}

void CityBrain::Impl::tickCars() {
    for (const Car c : cars) {
        CityCarKept& k = keptOf(c);
        if (k.heldBy == kOutsider) continue;   // somebody else's to move
        const driving::CarId id = idOf(c);
        CarRef ref;
        ref.id = id.index;
        if (k.phase == CarPhase::WaitingForRoom) {
            driving::PathPoint front;
            const Real length = lengthOf(c);
            if (k.spaceFront.path.valid()) front = k.spaceFront;   // (where the driving left it, pulling in)
            else if (!fronts->of(c).kerbLane(k.standing, front)) continue;
            if (!room->clear(front, length, kPullOutRoomBehind, kPullOutRoomAhead)) {
                ++counts.waitedForRoom;
            } else if (d->placeCarInSpace(id, bodyOf(c).vehicleClass, front, fronts->of(c).kerbOffset(k.standing))) {
                // placed standing in its space beside the kerb lane: its trip begins at once, and it drives out on
                // the driving's curve across
                counts.worstKerbHandOver = std::max(counts.worstKerbHandOver, awayFromDriving(c, carPos(c)));
                const driving::PathPoint keptFront = k.spaceFront;
                k.spaceFront = driving::PathPoint();
                syncLane(c);
                occupant.erase(k.standing.id);
                k.phase = CarPhase::OnRoad;
                ++counts.pulledOut;
                if (k.tripWanted) {
                    k.tripWanted = false;
                    fronts->of(c);
                    // (no route: the driving queues its own "no route" for the person, which the skill passes on)
                    if (skill->beginTrip(ref, k.tripPerson, k.tripFrom, k.tripTo, 0)) {
                        ++counts.tripsBegun;
                    } else {
                        // REFUSED BEFORE IT MOVED: back into its space, off the road -- left placed, it stood half in its
                        // space and listed in the kerb lane for good, and the cars behind waited for room behind it
                        // (river_town, #326: a car whose space began 1.5 m past junction 11)
                        d->removeCar(id);
                        syncLane(c);
                        occupant[k.standing.id] = numberOf(c);
                        k.spaceFront = keptFront;
                        k.phase = CarPhase::Standing;
                        ++counts.refusedInSpace;
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

// A MARKER, resolved against the road model (a level's, or one a role's day makes for a place).
int CityBrain::Impl::addMarker(const engine::CityMarker& spec) {
    const std::unordered_map<std::string, int>::const_iterator known = markerByName.find(spec.name);
    if (known != markerByName.end()) return known->second;
    if (markers.size() > kMarkerIndexMask) return -1;
    BrainMarker m;
    m.name = spec.name;
    m.kind = spec.kind;
    m.place = placeOfMarkerName(spec.name);
    m.stated = spec.stated;
    if (spec.stated) {
        for (std::size_t st = 0; st < model->streets.size(); ++st)
            if (model->streets[st].id == spec.street)
                m.frontage = model->frontageOn(engine::ModelStreetId(static_cast<uint32_t>(st)), spec.station, spec.dir);
    } else {
        m.frontage = model->frontageAt(Vec2(spec.x, spec.z));
    }
    if (!m.frontage.valid() || !m.frontage.street.valid() || !model->sidewalkBeside(m.frontage, m.stand)) return -1;
    // the post: beside the ring, half-way between away from the road and along the street (city_brain.h)
    const Vec2 away = unit(m.stand - m.frontage.on, Vec2(1, 0));
    m.post = m.stand + unit(away + engine::perp(away), away) * kMarkerPostOffset;
    markers.push_back(m);
    markerByName.emplace(m.name, static_cast<int>(markers.size() - 1));
    return static_cast<int>(markers.size() - 1);
}

int CityBrain::Impl::markerForPlace(PlaceId id) {
    if (!places || id == kNoPlace || static_cast<std::size_t>(id) >= places->places().size()) return -1;
    return addMarker(placeMarkerSpec((*places)[id], id, *model));
}

// A point of the sim's walking network (a corner, a path in a park): the frontage nearest it.
int CityBrain::Impl::markerForNode(int node) {
    if (node < 0 || node >= sim.walkPoints_.nodeCount()) return -1;
    engine::CityMarker m;
    m.name = "node_" + std::to_string(node);
    const Vec2 at = sim.walkPoints_.nodes[static_cast<std::size_t>(node)];
    m.x = at.x;
    m.z = at.y;
    return addMarker(m);
}

// THE LEVEL A PERSON SHOULD RUN AT (ADR-0493). Near: within the promote radius of the player, or someone the brain cannot
// yet run far -- anyone in a car (a near drive is not yet lowered), or held by a director. Far: beyond the demote radius,
// or a far driver whose car is away (ADR-0516: a car is not yet put part-way along its route). Between: as they are.
brain::Detail CityBrain::Impl::detailWanted(AgentId who) const {
    const Person& p = people[who];
    if (p.heldByDirector || sim.agents_[static_cast<std::size_t>(p.agent)].inCar >= 0) return brain::Detail::Near;
    if (p.level == brain::Detail::Far && drivingAway(who)) return brain::Detail::Far;
    // (a far walker the crowd draws is where the crowd has them, on their route: judged by the straight line between
    // the leg's ends instead, a walk whose line passes inside the ring and whose route does not raised and lowered its
    // walker over and over)
    const Agent& body = sim.agents_[static_cast<std::size_t>(p.agent)];
    const Vec2 at = p.level != brain::Detail::Far ? body.pos : body.farWalk ? farWalkPoint(body) : machine->whereIs(who, now());
    const Real away = (at - sim.tierCenter()).length();
    if (away < sim.pedPromoteRadius) return brain::Detail::Near;
    if (away > sim.pedDemoteRadius) return brain::Detail::Far;
    return p.level;
}

// A FAR PERSON'S BODY: where the brain says they are. On a leg they are not drawn (the sim's `indoors`: a far traveller
// is out of sight); at a place they stand at it, as a near person would.
// A FAR PERSON'S WALK, DRAWN (ADR-0495): when a far person's walking leg begins (or they are lowered on one), their
// agent is given the walk route from where they are to the leg's end, at the speed that arrives when the brain's clock
// says, from now: the GPU crowd moves and draws them along it (Agent::farWalk). Off it again when the leg ends or they
// are raised. Their body stays hidden; the sim does not move them.
void CityBrain::Impl::farWalkDrawn(AgentId who) {
    Person& p = people[who];
    Agent& a = agentOf(who);
    const brain::AgentBrain& b = machine->agent(who);
    const bool walking = b.travelling() && table.states[b.state].skill == "walk";
    if (!walking) {
        if (a.farWalk) {
            a.farWalk = false;
            a.route = engine::roads::WalkRoute{};
            sim.crowdChanged(p.agent);
        }
        return;
    }
    if (a.farWalk && p.farWalkSince == b.stateEntered) return;   // (this walk's route is given already)
    p.farWalkSince = b.stateEntered;
    Vec2 end;
    engine::WalkPoint from, to;
    engine::roads::WalkRoute r;
    if (machine->legEnd(b, end) && sim.walkJoin(a.pos, from) && sim.walkJoin(end, to))
        r = engine::roads::findWalkRoute(*sim.roads(), sim.walker_, from, to);
    const Real left = b.stateDue == brain::kNever || b.stateDue <= now() ? Real(0) : static_cast<Real>(b.stateDue - now()) / kMillis;
    if (!r.found || r.steps.empty() || !(r.metres > 0) || !(left > 0)) {
        if (a.farWalk) sim.crowdChanged(p.agent);
        a.farWalk = false;
        a.route = engine::roads::WalkRoute{};
        return;
    }
    a.route = std::move(r);
    a.leg = 0;
    a.distOnLeg = 0;
    a.speed = a.route.metres / left;
    a.vLastTick = sim.seconds();
    a.farWalk = true;
    sim.crowdChanged(p.agent);
}

// RAISED: the body where the brain has them -- along the route the crowd was drawing them on, else where the clock puts
// them -- and seen, and off the crowd.
void CityBrain::Impl::raiseBody(AgentId who) {
    Agent& a = agentOf(who);
    if (a.farWalk) {
        const Vec2 at = farWalkPoint(a);
        a.farWalk = false;
        a.route = engine::roads::WalkRoute{};
        a.speed = 0;
        sim.crowdChanged(people[who].agent);
        a.pos = at;
        a.tickFromPos = at;
        sim.grid_.place(people[who].agent, a.pos);
        a.moving = false;
    } else {
        placeFarBody(who);
        if (a.farWalk) {   // (they are near from now: their skill takes up the walk, not the crowd)
            a.farWalk = false;
            a.route = engine::roads::WalkRoute{};
            a.speed = 0;
            sim.crowdChanged(people[who].agent);
        }
    }
    a.indoors = false;
}

// Where a far walker drawn by the crowd is now: along their route at their speed since it began (farWalkDrawn).
Vec2 CityBrain::Impl::farWalkPoint(const Agent& a) const {
    Real go = std::max(Real(0), static_cast<Real>(sim.seconds() - a.vLastTick)) * a.speed;
    const engine::LaneGraph& G = sim.roads()->laneLevel;
    Vec2 last = a.pos;
    for (const engine::roads::WalkStep& s : a.route.steps) {
        const Real along = std::fabs(s.to - s.from);
        if (go <= along) return G.pointAt(s.walk, s.from + (s.forward ? go : -go));
        go -= along;
        last = G.pointAt(s.walk, s.to);
    }
    return last;
}

void CityBrain::Impl::placeFarBody(AgentId who, bool justLowered) {
    Person& p = people[who];
    Agent& a = agentOf(who);
    const brain::AgentBrain& b = machine->agent(who);
    // (on a walk the crowd is drawing already, the body is left where the walk began: the crowd moves them; just
    // lowered part-way along a walk, they are where their near walk had got to, and the crowd goes on from there)
    const bool sameWalk = a.farWalk && p.farWalkSince == b.stateEntered && b.travelling();
    const bool walkingOn = justLowered && b.travelling() && table.states[b.state].skill == "walk";
    if (!sameWalk && !walkingOn && a.seatPhase == 0) {   // (one on a seat or in a group stays there, ADR-0512)
        const Vec2 at = machine->whereIs(who, now());
        if ((at - a.pos).length() > engine::kSamePoint) {
            a.pos = at;
            a.tickFromPos = at;
            sim.grid_.place(p.agent, a.pos);
        }
        a.moving = false;
        a.speed = 0;
    }
    a.indoors = b.travelling() || !table.states[b.state].seen;
    farWalkDrawn(who);
}

// A STATE THE PERSON IS OUT OF SIGHT IN (brain.lua `seen = false`: indoors): their body is not drawn while in it, and is
// again once they leave it (a walk, a far traveller's own rule, sets it as it goes on).
void CityBrain::Impl::showBody(AgentId who) {
    Person& p = people[who];
    const bool seen = table.states[machine->agent(who).state].seen;
    if (!seen) {
        agentOf(who).indoors = true;
        p.unseen = true;
    } else if (p.unseen) {
        p.unseen = false;
        if (!(p.level == brain::Detail::Far && machine->agent(who).travelling())) agentOf(who).indoors = false;
    }
}

void CityBrain::Impl::tickMachine() {
    const std::vector<AgentId>& answered = machine->answerEvents();
    // (an event can change a state -- an arrival indoors -- with no step to follow it: the body shown or hidden now; a
    // near person arrived indoors and stepped no more stood on their spot at the marker, on river_town in the road)
    for (const AgentId who : answered)
        if (who < people.size() && !farOf[who]) showBody(who);
    // A NEAR PERSON WITH NOTHING TO DO THIS TICK is not stepped (ADR-0496): their state has no work of its own each tick
    // and its clock is not due (Machine::step would only run that clock). The island's 7,000 near people cost 7.5 ms.
    const BrainTime t = now();
    const auto nearBusy = [&](AgentId who) {
        const brain::AgentBrain& b = machine->agent(who);
        if (table.states[b.state].fns.step) return true;
        const BrainTime due = machine->nextAttention(who);
        return due != brain::kNever && due <= t;
    };
    // HEADLESS (no player, no tier centre): everybody near, as before attention
    if (!sim.hasTierCenter()) {
        for (AgentId who = 0; who < people.size(); ++who) {
            if (!nearBusy(who)) continue;
            machine->step(who, brain::Detail::Near);
            showBody(who);
        }
        return;
    }
    // (the people an event was answered for: a far one is visited now, and their day read)
    for (const AgentId who : answered)
        if (who < people.size() && farOf[who]) {
            dueOf[who] = t;
            dayGateOf[who] = 0;
        }
    // ONE SHARE OF THE PEOPLE has their distance looked at this tick (every kDetailTurns-th, in turn), and is raised or
    // lowered; the scan below passes over whoever was (steppedNow)
    const uint32_t turn = detailTurn++ % kDetailTurns;
    changedNow.clear();
    for (AgentId who = turn; who < people.size(); who += kDetailTurns) {
        Person& p = people[who];
        const brain::Detail level = detailWanted(who);
        if (level == p.level) continue;
        if (level == brain::Detail::Near) {
            raiseBody(who);   // (raised: their body where the clock has them, then the skill takes up the leg)
            ++counts.raised;
        } else {
            ++counts.lowered;
        }
        machine->step(who, level);
        setLevel(who, level);
        if (level == brain::Detail::Far) {
            placeFarBody(who, true);
            dueOf[who] = machine->nextAttention(who);
        }
        showBody(who);
        changedNow.push_back(who);
    }
    std::size_t next = 0;   // (changedNow is in person order)
    // THE SCAN, over the compact arrays: near people with something to do this tick, far people whose clock is due
    for (AgentId who = 0; who < people.size(); ++who) {
        if (next < changedNow.size() && changedNow[next] == who) {
            ++next;
            continue;
        }
        if (!farOf[who]) {
            if (!nearBusy(who)) continue;
            machine->step(who, brain::Detail::Near);
            showBody(who);
            continue;
        }
        if (dueOf[who] == brain::kNever || dueOf[who] > t) continue;
        machine->step(who, brain::Detail::Far);
        dayGateOf[who] = 0;
        ++counts.farVisits;
        placeFarBody(who);
        dueOf[who] = machine->nextAttention(who);
        showBody(who);
    }
}

// The level a person runs at, and its compact copy for the scans (farOf); their day is read again at once.
void CityBrain::Impl::setLevel(AgentId who, brain::Detail level) {
    people[who].level = level;
    farOf[who] = level == brain::Detail::Far ? 1 : 0;
    dayGateOf[who] = 0;
}

// THEIR ROLE'S DAY (ADR-0491): the sim's own table for their role (roles.lua, buildDayTable), run here on the sim's
// clock with the sim's own reading of it (CitySim::restEvents: the work window, the evening, the stop's length) and the
// sim's own choosers (choosePickedNode: lunch, an outing, a campus break, an activity, the errand). A GoTo state is a
// marker they want to be at: Home is their home's, Work their job's place, a pick its venue's place or the frontage
// nearest its point. Arriving there is the state's Arrived; the brain finding no way there is its NoRoute.
// They do not take a pick's seat or join its group (#334): the pick's hold is let go and they stand at the marker.
void CityBrain::Impl::tickRoleDay(AgentId who, Real dtHours) {
    Person& p = people[who];
    if (!p.roleDay || !p.dayOn || p.heldByDirector) return;
    Agent& a = agentOf(who);
    const GoalTable& t = sim.tableFor(a);
    if (a.goal < 0 || a.goal >= t.stateCount()) return;
    const GoalState& s = t.state(a.goal);
    const brain::AgentBrain& b = machine->agent(who);
    if (s.action == GoalAction::GoTo) {
        ++cost.goToReads;
        // (a far person on their way: nothing changes for them but when the brain visits them; their day is not read
        // until then -- tickMachine opens the gate)
        struct GateAfter {
            CityBrain::Impl& im;
            AgentId who;
            ~GateAfter() {
                const Person& q = im.people[who];
                if (im.farOf[who] && q.dayMarker >= 0) im.dayGateOf[who] = std::numeric_limits<double>::infinity();
            }
        } gate{*this, who};
        if (p.dayMarker < 0) {
            // (no more than kDaySetOutsPerTick a tick: the rest wait their turn, first come first served, their day not
            // read meanwhile -- the island's 115k all due out at once were each read every tick, 5 ms of it)
            if (daySetOutsThisTick >= kDaySetOutsPerTick) {
                if (!p.waitingToSetOut) {
                    p.waitingToSetOut = true;
                    (p.level == brain::Detail::Far ? setOutQueue : nearSetOutQueue).push_back(who);
                }
                dayGateOf[who] = std::numeric_limits<double>::infinity();
                return;
            }
            ++daySetOutsThisTick;
            p.waitingToSetOut = false;
            const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
            setOut(who, s);
            const double setOutMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            cost.setOutMs += setOutMs;
            cost.worstSetOutMs = std::max(cost.worstSetOutMs, setOutMs);
            return;
        }
        const PlaceRef wanted = markerPlace(p.dayMarker, who);
        if (!b.travelling() && b.state == table.intents[b.intentKind].arrive && b.intentTarget == wanted.id) {
            ++counts.dayArrivals;
            if (p.dayVenue >= 0 && p.dayVenue < static_cast<int>(sim.venues_.size()) &&
                !sim.venues_[static_cast<std::size_t>(p.dayVenue)].openAt(sim.clockHours_))
                ++counts.dayArrivedShut;
            if (s.target == GoalTarget::Work && a.workPlace != kNoPlace) {
                ++counts.dayToWork;
                if (!sim.inWorkWindow(a)) ++counts.dayToWorkOffShift;
            }
            // HOW LONG THEY STAY: a visit's own length (CitySim::visitHours), a pause on a walk round the block, else
            // the state's
            a.restDwell = p.dayVenue >= 0 || a.tripActivity >= 0 ? static_cast<float>(sim.visitHours(a))
                          : s.target == GoalTarget::Outing    ? static_cast<float>(sim.jitterHours(a, 0.01, 0.04))
                                                              : 0.0f;
            a.atActivity = a.tripActivity;
            a.tripActivity = -1;
            a.outingTo = -1;
            p.atMarker = p.dayMarker;
            fireDay(who, GoalEvent::Arrived);
            // ON THE PICK'S SEAT, TRACK OR GROUP (ADR-0512): near, they walk onto it; far, they are put on it
            if (sim.beginSpotUse(a, p.level == brain::Detail::Far)) ++counts.daySpots;
        } else if (b.state == table.noPlan) {
            ++counts.dayNoWay;
            a.outingTo = -1;
            a.tripVenue = -1;
            fireDay(who, GoalEvent::NoRoute);
            // THE DAY HAS GIVEN UP ON IT: they want to be where they are again (else the brain goes on wanting a place
            // there is no way to, planning again every minute: 25 of river_town's 5115 did, 782 plans in 30 minutes)
            if (p.atMarker >= 0) {
                const PlaceRef here = markerPlace(p.atMarker, who);
                if (here.valid()) machine->setIntent(who, p.intentKind, here);
            }
        }
        return;
    }
    // ASLEEP: nothing in their day before their wake (the island's 100k resting people read the clock sixty times a
    // second otherwise); on waking, their time in the state is caught up
    const double total = sim.clockTotalHours_;
    if (p.dayAsleep) {
        if (total < p.dayWakeAt) {
            dayGateOf[who] = p.dayWakeAt;   // (woken early, by an event: back to the gate)
            return;
        }
        a.goalHours += total - p.daySleptAt;
        p.dayAsleep = false;
    } else {
        a.goalHours += dtHours;
    }
    if (total < p.dayRetryAt) return;
    // A GROUP'S PLAYER (ADR-0512): the group says when it ends (CitySim::stepSessions); read again in a game minute
    if (a.session >= 0) {
        dayGateOf[who] = total + 1.0 / 60.0;
        return;
    }
    ++cost.restReads;
    GoalEvent events[CitySim::kRestEvents];
    const int n = sim.restEvents(a, s, events);
    for (int e = 0; e < n; ++e)
        if (fireDay(who, events[e])) return;
    // NOTHING FIRED, AND NOTHING CAN UNTIL A KNOWN TIME (CitySim::hoursUntilRestEvent, the sim's own reckoning). With the
    // clock standing (a held sky) they sleep too: nothing they wait for can come while it stands, and they wake when it
    // runs or jumps past their hour. (They were read every tick instead: on the island with the sky held, 100k rests a
    // tick, the brain 4x dearer.)
    const Real until = sim.hoursUntilRestEvent(a, s);
    p.dayAsleep = true;
    p.daySleptAt = total;
    p.dayWakeAt = until >= CitySim::kNoRestEvent ? std::numeric_limits<double>::infinity() : total + until;
    dayGateOf[who] = p.dayWakeAt;
}

// A GoTo STATE BEGINS: where it goes, as a marker, and they want to be there.
void CityBrain::Impl::setOut(AgentId who, const GoalState& s) {
    Person& p = people[who];
    Agent& a = agentOf(who);
    int marker = -1;
    p.dayVenue = -1;
    sim.releaseSeat(a);   // (off the seat or out of the group they were at: ADR-0512)
    const int origin = sim.walkPoints_.empty() ? -1 : sim.walkPoints_.nearestNode(a.pos);
    const int picked = origin >= 0 ? sim.choosePickedNode(a, origin, s) : -1;
    bool outdoors = false;   // the pick is out of doors: a park, a seat, a track, a group's area
    if (picked >= 0) {
        // (the pick's seat or place in a group, if it held one, is theirs from now: they go to it, ADR-0512)
        const int v = a.tripVenue;
        outdoors = a.tripSeat >= 0 || a.session >= 0 ||
                   !(v >= 0 && v < static_cast<int>(sim.venues_.size()) && placeIsIndoors(sim.venues_[static_cast<std::size_t>(v)].type));
        if (v >= 0 && v < static_cast<int>(sim.venues_.size()) && sim.venues_[static_cast<std::size_t>(v)].place != kNoPlace) {
            marker = markerForPlace(sim.venues_[static_cast<std::size_t>(v)].place);
            p.dayVenue = v;
        }
        if (marker < 0) marker = markerForNode(picked);
    } else if (s.target == GoalTarget::Home) {
        marker = p.homeMarker;
    } else if (s.target == GoalTarget::Work) {
        marker = a.workPlace != kNoPlace ? markerForPlace(a.workPlace) : markerForNode(a.work);   // (a day out's park)
    } else if (s.target == GoalTarget::Shop) {
        marker = a.shopPlace != kNoPlace ? markerForPlace(a.shopPlace) : markerForNode(a.shop);
    }
    if (marker < 0) {
        ++counts.dayNowhere;
        a.outingTo = -1;
        a.tripVenue = -1;
        sim.releaseSeat(a);
        fireDay(who, GoalEvent::NoRoute);
        return;
    }
    takeSpot(marker, who);
    const PlaceRef to = markerPlace(marker, who);
    if (!to.valid()) {   // (more people use it than it has spots)
        ++counts.dayNowhere;
        fireDay(who, GoalEvent::NoRoute);
        return;
    }
    ++counts.daySetOut;
    // (their new intent is answered at the next tick: read again this tick they would seem to have arrived already)
    p.setOutTick = tickNumber;
    p.dayMarker = marker;
    a.activity = s.activity;
    // WHAT THEY WANT AT IT (ADR-0512): on the pick's seat or track (use_spot), in its group (join_group), outside at a
    // park (be_outside_at); else their own intent, indoors at a place
    brain::IntentKind kind = p.intentKind;
    if (a.session >= 0 && groupIntent >= 0) kind = static_cast<brain::IntentKind>(groupIntent);
    else if (a.tripSeat >= 0 && spotIntent >= 0) kind = static_cast<brain::IntentKind>(spotIntent);
    else if (outdoors && outsideIntent >= 0) kind = static_cast<brain::IntentKind>(outsideIntent);
    machine->setIntent(who, kind, to);
}

// The event moves their goal state, when their table has a row for it.
bool CityBrain::Impl::fireDay(AgentId who, GoalEvent e) {
    Agent& a = agentOf(who);
    const GoalTable& t = sim.tableFor(a);
    const int next = t.onEvent(a.goal, e);
    if (next < 0) return false;
    a.goal = next;
    a.goalHours = 0;
    a.activity = t.state(next).activity;
    people[who].dayMarker = -1;
    people[who].dayAsleep = false;
    dayGateOf[who] = 0;
    // nowhere to go, or no way there: the day tries again no sooner than kDayRetryHours from now
    if (e == GoalEvent::NoRoute) people[who].dayRetryAt = sim.clockTotalHours_ + kDayRetryHours;
    return true;
}

// ANOTHER PLACE OF THE SAME KIND (ADR-0446). A person who has no way to the place they want after a way of getting there
// was found not to work (a leg of it could not be done, so it is ruled out: the brain's `ruledOut`) judges that place
// out of reach: they remember it, and choose the nearest place of the same kind (engine::CityMarker::kind, a data
// name) they have not found out of reach, in a straight line from where they stand (ties: the lower marker number), and
// want to be there instead; their day goes there from now on in place of the old one. Not a time limit and not a count
// of tries: the judgement is made the moment the plan fails that way, once per place. A place of no kind, or no other
// place of its kind left: nothing changes, and they wait and try again as before (brain.lua `no_way`).
void CityBrain::Impl::tickChoice(AgentId who) {
    const brain::AgentBrain& a = machine->agent(who);
    if (a.state != table.noPlan || a.ruledOut == 0) return;
    PlaceRef wanted;
    wanted.id = a.intentTarget;
    if (!isMarker(wanted) || markerIndex(wanted) >= markers.size()) return;
    const uint32_t from = markerIndex(wanted);
    const std::string& kind = markers[from].kind;
    if (kind.empty()) return;
    Person& p = people[who];
    if (std::find(p.noWayTo.begin(), p.noWayTo.end(), from) == p.noWayTo.end()) p.noWayTo.push_back(from);
    const Vec2 here = agentOf(who).pos;
    int best = -1;
    Real bestAway = 0;
    for (uint32_t m = 0; m < markers.size(); ++m) {
        if (markers[m].kind != kind || std::find(p.noWayTo.begin(), p.noWayTo.end(), m) != p.noWayTo.end()) continue;
        const Real away = (markers[m].stand - here).length();
        if (best >= 0 && !engine::strictlyPast(bestAway, away, engine::kSamePoint)) continue;
        best = static_cast<int>(m);
        bestAway = away;
    }
    if (best < 0) return;
    takeSpot(best, who);
    const PlaceRef to = markerPlace(best, who);
    if (!to.valid()) return;
    for (Stop& s : p.day)
        if (s.place.id == wanted.id) s.place = to;
    LOG_INFO << "[brain] " << p.name << ": no way to " << markers[from].name << ", so " << markers[static_cast<std::size_t>(best)].name
             << " instead (the nearest " << kind << " not found out of reach, " << fmt("%.0f m", bestAway) << ")";
    ++counts.otherPlaces;
    machine->setIntent(who, p.intentKind, to);
}

void CityBrain::Impl::shareOutTrace() {
    for (const brain::Transition& t : traceAll)
        if (t.agent < people.size()) people[t.agent].trace.push_back(t);
    traceAll.clear();
}

// ---- Somebody the brain does not own, and a household's car (ADR-0443) --------------------------------------------------

// AN OUTSIDER HAS THE CAR FROM NOW: it leaves every table. No space is its, it is off the driving's road, no trip is
// wanted of it, and nobody of the household can take it or plan with it until it is left again. `lost`: the player has
// it as a physical car of their own, so it is not drawn here (CityCarLink::away) until it is handed back. Otherwise
// the outsider is the sim agent at its wheel (CityCarLink::driver, seated first).
void CityBrain::Impl::outsiderTook(Car c, bool lost) {
    const driving::CarId id = idOf(c);
    if (onRoad(c)) {
        CarRef ref;
        ref.id = id.index;
        skill->endTrip(ref);
        d->removeCar(id);
        syncLane(c);
    }
    leaveEverySpace(c);
    linkOf(c).away = lost;
    CityCarKept& k = keptOf(c);
    k.heldBy = kOutsider;
    k.stranded = false;
    k.phase = CarPhase::Standing;
    k.tripWanted = false;
    k.standing = PlaceRef();
    tellHousehold(c, brain::kNoAgent);
}

// THE OUTSIDER HAS LEFT IT where it now stands. In a kerb space when its middle is within kLeftInSpaceWithin of where
// a car in one stands and no other car is in that space. Otherwise at a point: on the road (no farther than
// kLeftOnRoadWithin beyond the kerb line) it is put into the kerb lane beside that point, so the driving knows it is
// there and a resident can drive it away; anywhere else it is stranded, and nobody plans with it.
void CityBrain::Impl::outsiderLeft(Car c) {
    const Vec2 at = carPos(c);
    const driving::CarId id = idOf(c);
    CarRef ref;
    ref.id = id.index;
    linkOf(c).away = false;
    CityCarKept& k = keptOf(c);
    k.heldBy = brain::kNoAgent;
    k.phase = CarPhase::Standing;
    k.tripWanted = false;
    k.stranded = false;
    const engine::KerbSpaceNear near = model->nearestKerbSpace(at, kLeftInSpaceWithin);
    const PlaceRef space = near.space.valid() ? spaces->placeOf(near.space) : PlaceRef();
    const std::map<uint32_t, uint32_t>::const_iterator in = space.valid() ? occupant.find(space.id) : occupant.end();
    if (space.valid() && (in == occupant.end() || in->second == ref.id)) {
        k.standing = space;
        occupant[space.id] = ref.id;
    } else {
        k.standing = pointPlace(at);
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
        k.stranded = !onRoad(c);
    }
    ++counts.leftAgain;
    LOG_INFO << "[brain] " << carLine(c);
    tellHousehold(c, brain::kNoAgent);
}

std::string CityBrain::Impl::carLine(Car c) const {
    static const char* const kPhase[] = {"standing", "waiting in its space for the kerb lane to be clear", "on the road",
                                         "away, its far driver's drive run by the clock"};
    const CityCarLink& link = linkOf(c);
    const CityCarKept& k = keptOf(c);
    std::string s = "the car of household " +
                    (link.household >= 0 ? households[static_cast<std::size_t>(link.household)].name : std::string("?")) +
                    " (car " + std::to_string(numberOf(c)) + ") ";
    if (k.heldBy == kOutsider) {
        s += link.driver >= 0 ? "has been taken by somebody else (sim agent " + std::to_string(link.driver) + ")"
                              : std::string("has been taken by the player");
        return s + ": it is not free, and stands nowhere a resident could take it from";
    }
    s += std::string("is ") + kPhase[static_cast<int>(k.phase)] +
         (k.phase == CarPhase::OnRoad && !aboard(c) ? " (left there)" : "") + ", " +
         ((onRoad(c) || k.phase == CarPhase::Away) && isSpace(k.standing) ? "last in " : isPoint(k.standing) ? "left at " : "in ") + placeName(k.standing);
    if (k.stranded) s += ", off the road: nobody can drive it from there";
    if (k.heldBy < people.size()) s += ", " + people[k.heldBy].name + " has it";
    else s += ", free";
    if (const driving::Car* placed = d->car(idOf(c))) s += ", " + fmt("%.1f m/s", placed->speed);
    return s;
}

// ---- CityBrain ----------------------------------------------------------------------------------------------------------

CityBrain::CityBrain(CitySim& sim) : impl_(std::make_unique<Impl>(sim)) {}
CityBrain::~CityBrain() = default;

void CityBrain::setPlaces(const PlaceMap* places) { impl_->places = places; }

bool CityBrain::start(engine::World& world, const std::vector<engine::CityMarker>& markerSpecs,
                      const std::vector<engine::CityResident>& residents, const std::vector<engine::CityHousehold>& householdSpecs,
                      const std::vector<int>& agentOf, const std::vector<int>& classOfSlot, std::string* report,
                      bool drivableSpacesOnly) {
    Impl& im = *impl_;
    CitySim& sim = im.sim;
    im.drivableSpacesOnly = drivableSpacesOnly;
    const auto refuse = [&](const std::string& why) {
        if (report) *report = why;
        for (const Impl::Car c : im.cars) world.destroy(c);   // (nothing is left behind)
        im.cars.clear();
        return false;
    };
    // WITH NO DRIVING (ADR-0494: a level that names no citysim.drivingRules -- the island, the metros), the brain's
    // people walk and ride, and no household has a car: there is nothing to drive one on. The old sim's people had none
    // there either (ADR-0461).
    const bool driven = sim.drivingV2_ && sim.drivingV2_->roads() && sim.drivingV2_->hasRules() && sim.drivingHost_.active();
    if (!sim.roads() || !sim.roads()->built || sim.walker_.open.empty()) return refuse("the city's road model has no walks");
    im.d = driven ? &*sim.drivingV2_ : nullptr;
    im.model = driven ? im.d->model() : sim.roads();
    im.ecs = &world;
    {
        // the cars of a brain made before (the city built again on the same world) go: their numbers and their people
        // are the old city's (the cars that drive round for ever are the hand-over's, made for this city already)
        std::vector<engine::Entity> old;
        world.each<CityCarBody>([&](engine::Entity e, CityCarBody&) {
            if (!world.has<CityCarRoams>(e)) old.push_back(e);
        });
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
    im.spotIntent = im.table.intent("use_spot");
    im.groupIntent = im.table.intent("join_group");
    im.outsideIntent = im.table.intent("be_outside_at");
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
    for (const engine::CityMarker& spec : markerSpecs)
        if (im.addMarker(spec) < 0) left += (left.empty() ? "" : ", ") + spec.name;
    if (!left.empty())
        LOG_WARN << "[brain] markers left out (no such street, or no sidewalk beside it): " << left;
    im.spaces = std::make_unique<brain::RoadPlaces>(*im.model, kSpaceTag);
    std::string line = std::to_string(im.markers.size()) + " markers";
    if (residents.empty()) {
        if (report) *report = line + ", no resident";
        return true;
    }
    // EACH PERSON'S SPOT AT EVERY MARKER THEY USE, in the level's order: their home, then their day's stops
    for (std::size_t r = 0; r < residents.size(); ++r) {
        im.takeSpot(markerNamed(residents[r].home), static_cast<AgentId>(r));
        for (const engine::CityResident::Stop& stop : residents[r].day) im.takeSpot(markerNamed(stop.at), static_cast<AgentId>(r));
    }

    // THE RESIDENTS, checked before anything is made: each the sim's agent who is them (the city's own households,
    // ADR-0490), or a walker the level made, in the order it made them (hand-written residents)
    std::vector<int> agents;
    if (!agentOf.empty()) {
        if (agentOf.size() != residents.size())
            return refuse(std::to_string(residents.size()) + " residents and " + std::to_string(agentOf.size()) + " agents for them");
        std::vector<bool> taken(sim.agents_.size(), false);
        for (const int i : agentOf) {
            if (i < 0 || i >= static_cast<int>(sim.agents_.size()) || taken[static_cast<std::size_t>(i)])
                return refuse("resident's agent " + std::to_string(i) + " is not one of the sim's, or is two residents");
            const Agent& a = sim.agents_[static_cast<std::size_t>(i)];
            if (a.released || a.playerControlled) return refuse("resident's agent " + std::to_string(i) + " is somebody else's");
            taken[static_cast<std::size_t>(i)] = true;
            agents.push_back(i);
        }
    } else {
        for (std::size_t i = 0; i < sim.agents_.size() && agents.size() < residents.size(); ++i) {
            const Agent& a = sim.agents_[i];
            if (!a.released && !a.playerControlled) agents.push_back(static_cast<int>(i));
        }
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
        if (!im.personByName.emplace(resident.name, static_cast<int>(r)).second) return refuse(at + " is listed twice");
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
        person.drives = resident.drives;
        person.roleDay = resident.roleDay;
        person.homeMarker = m.home;
        person.atMarker = m.home;
        person.intentKind = m.intent;
        if (person.agent >= 0) {
            if (im.personByAgent.size() <= static_cast<std::size_t>(person.agent)) im.personByAgent.resize(static_cast<std::size_t>(person.agent) + 1, -1);
            im.personByAgent[static_cast<std::size_t>(person.agent)] = static_cast<int>(im.people.size());
        }
        im.people.push_back(person);
    }

    im.world = std::make_unique<Impl::LabWorld>(im);
    im.planner = std::make_unique<Impl::NotingPlanner>(im, TripNetwork{sim.roads(), &sim.walker_, im.d, &sim.transit_},
                                                       im.travel, *im.world);
    im.fronts = std::make_unique<Impl::FrontPoints>(im);
    if (im.d) im.room = std::make_unique<brain::LaneRoom>(*im.d);

    // THE HOUSEHOLDS AND THEIR CARS, each car standing in the free kerb space nearest (on foot) the marker it states
    std::size_t carsNotMade = 0;   // (no driving: none is)
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
        if (!im.d && !hs.cars.empty()) carsNotMade += hs.cars.size();
        for (const engine::CityHousehold::Car& cs : im.d ? hs.cars : std::vector<engine::CityHousehold::Car>{}) {
            const std::string at = "household \"" + hs.name + "\": a car near \"" + cs.near + "\"";
            const int marker = markerNamed(cs.near);
            if (marker < 0) return refuse(at + ": that is not one of the level's markers");
            // THE CAR IS ONE ENTITY (city_car.h), numbered on from the last car number in use, and drawn as the fleet
            // slot the sim would deal a vehicle of that number (CitySim::ambientSlotFor)
            // (in a row: the brain finds a car from its number by where it is in im.cars)
            const uint32_t number = sim.nextCarNumber(world);
            if (im.cars.empty()) im.firstCar = number;
            if (number != im.firstCar + im.cars.size())
                return refuse(at + ": car number " + std::to_string(number) + " does not follow the household cars before it");
            const int slot = sim.ambientSlotFor(static_cast<int>(number));
            const VehicleBody& body = sim.fleetBody(slot);
            const int cls = slot >= 0 && slot < static_cast<int>(classOfSlot.size()) ? classOfSlot[static_cast<std::size_t>(slot)] : -1;
            const uint32_t drivenAs = cls >= 0 && cls < static_cast<int>(im.d->vehicleClasses().size()) ? static_cast<uint32_t>(cls) : 0u;
            // ...standing in the free kerb space nearest the marker on foot that it can drive from
            PlaceRef chosen;
            for (const PlaceRef space : im.spacesFor(im.markerPlace(marker, 0), drivenAs)) {
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
            CityCarBody carBody;
            carBody.id = number;
            carBody.slot = slot;
            carBody.type = body.type;
            carBody.length = body.length;
            carBody.width = body.width;
            carBody.height = body.height;
            carBody.vehicleClass = drivenAs;
            CityCarLink link;
            link.household = index;
            CityCarKept kept;
            kept.standing = chosen;
            engine::Transform where;
            setCarPose(where, pose.at, pose.facing, 0);
            const Impl::Car c = world.create();
            world.add<engine::Transform>(c, where);
            world.add<engine::PrevTransform>(c, engine::PrevTransform{where});
            world.add<CityCarBody>(c, carBody);
            world.add<CityCarLink>(c, link);
            world.add<CityCarKept>(c, kept);
            im.cars.push_back(c);
            im.occupant[chosen.id] = number;
            h.cars.push_back(number);
            line += "; household " + hs.name + "'s car (car " + std::to_string(number) + ", a " +
                    im.d->vehicleClasses().at(carBody.vehicleClass).name + ") stands in " + im.placeName(chosen);
        }
        im.households.push_back(h);
    }
    if (carsNotMade > 0) line += "; no driving on this level: " + std::to_string(carsNotMade) + " household cars not made";
    if (im.d) im.skill = std::make_unique<brain::DrivingSkill>(*im.d, *im.fronts, im.cars.empty() ? 0u : im.bodyOf(im.cars.front()).vehicleClass);
    // (the hand-over copies the driving's pose into each car entity while the driving has it: it has the world since
    // the driving began, startDrivingV2)
    if (im.skill && !im.skill->bindEvents(im.table, &err)) return refuse(err);
    im.walk = std::make_unique<Impl::LabWalk>(im);
    if (im.d) im.drive = std::make_unique<Impl::LabDrive>(im);
    im.machine = std::make_unique<brain::Machine>(im.table, *im.world, *im.planner, im.walk.get(), im.drive.get());
    im.ride = std::make_unique<Impl::LabRide>(im);
    im.farOf.assign(residents.size(), 0);
    im.dueOf.assign(residents.size(), brain::kNever);
    im.dayGateOf.assign(residents.size(), 0.0);

    im.machine->setRide(im.ride.get());
    im.machine->traceTo(&im.traceAll);

    // THE PEOPLE, each standing on their own spot at their home marker, the brain's to move from now on
    for (std::size_t r = 0; r < residents.size(); ++r) {
        const engine::CityResident& resident = residents[r];
        Impl::Person& person = im.people[r];
        const AgentId id = static_cast<AgentId>(r);
        const BrainMarker& homeMarker = im.markers[static_cast<std::size_t>(made[r].home)];
        const PlaceRef homePlace = im.markerPlace(made[r].home, id);
        CarRef carRef;
        if (person.drives && person.household >= 0 && !im.households[static_cast<std::size_t>(person.household)].cars.empty())
            carRef.id = im.households[static_cast<std::size_t>(person.household)].cars.front();
        {
            Agent& a = sim.agents_[static_cast<std::size_t>(person.agent)];
            sim.setAgentDirected(person.agent, true);
            a.brainOwned = true;
            a.pos = im.spotAt(homeMarker, im.spotFor(made[r].home, id));
            a.tickFromPos = a.pos;
            a.heading = unit(homeMarker.frontage.on - homeMarker.stand, a.heading);
            a.tickFromHeading = a.heading;
            a.indoors = false;
            a.moving = false;
            a.speed = 0;
            a.route = engine::roads::WalkRoute{};
            a.state = Agent::State::Resting;
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
        person.dayOn = !person.day.empty() || person.roleDay;
        if (person.roleDay) {
            // THEIR ROLE'S DAY from the start of it, at home (whatever the sim's day placed them at, ADR-0491); the
            // sim's goal layer never runs it (they are directed), the brain does (tickRoleDay)
            Agent& a = sim.agents_[static_cast<std::size_t>(person.agent)];
            sim.releaseSeat(a);
            a.goal = sim.tableFor(a).entry();
            a.goalHours = 0;
            a.restDwell = 0;
            a.outingTo = -1;
            a.tripVenue = -1;
            a.tripActivity = -1;
            a.activity = Agent::Activity::AtHome;
        }
        line += "; " + resident.name + " is the sim's agent " + std::to_string(person.agent) + ", at home at \"" + resident.home + "\"" +
                (person.household >= 0 ? " in household " + im.households[static_cast<std::size_t>(person.household)].name
                                       : std::string(" in no household")) +
                (person.roleDay ? std::string(", their role's day")
                 : person.day.empty() ? std::string(", no day table") : ", a day of " + std::to_string(person.day.size()) + " stops");
    }
    // EACH BODY SHOWN OR HIDDEN BY THE STATE IT BEGINS IN (indoors at home: hidden). It was done by each tick's step of
    // everybody; since ADR-0496 a person at home with nothing to do is never stepped, and stood on their spot all day.
    for (AgentId who = 0; who < im.people.size(); ++who) im.showBody(who);
    if (report) *report = line;
    return true;
}

void CityBrain::tick(Real dt) {
    Impl& im = *impl_;
    if (!im.machine) return;
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    im.followCars();   // (the driving has moved the cars on the road: their people with them, before anyone plans)
    // the driving's reports for the households' cars, taken from the hand-over (which drains the driving's events)
    const std::vector<driving::Event> events = im.d ? im.sim.drivingHost_.takeGuestEvents() : std::vector<driving::Event>{};
    for (const driving::Event& e : events) {
        if (e.kind == driving::EventKind::Parked) ++im.counts.parkedReports;
        if (e.kind == driving::EventKind::WreckCleared) {   // (sitting 7: the wreck is towed to a kerb space near)
            CarRef ref;
            ref.id = e.car.index;
            if (const Impl::Car c = im.car(ref); c.valid()) im.towAway(c);
        }
    }
    if (im.skill) im.skill->report(*im.machine, events);
    im.tickRides();
    using Clock = std::chrono::steady_clock;
    const auto msSince = [](Clock::time_point at) { return std::chrono::duration<double, std::milli>(Clock::now() - at).count(); };
    Clock::time_point part = Clock::now();
    im.tickWalks(dt);
    im.cost.walksMs += msSince(part);
    part = Clock::now();
    im.tickMachine();
    im.cost.machineMs += msSince(part);
    im.tickCars();
    im.followCars();   // (...and the cars sliding between a space and the lane)
    const Real dtHours = dt * im.sim.hoursPerSecond();
    im.daySetOutsThisTick = 0;
    ++im.tickNumber;
    part = Clock::now();
    const double total = im.sim.clockTotalHours_;
    // THE TRIPS WAITING THEIR TURN go first, as many as a tick begins: the near player's, then the rest
    while ((!im.nearSetOutQueue.empty() || !im.setOutQueue.empty()) && im.daySetOutsThisTick < kDaySetOutsPerTick) {
        std::deque<AgentId>& q = !im.nearSetOutQueue.empty() ? im.nearSetOutQueue : im.setOutQueue;
        const AgentId who = q.front();
        q.pop_front();
        im.people[who].waitingToSetOut = false;
        im.dayGateOf[who] = 0;
        im.tickRoleDay(who, 0);   // (its time in a rest is not running: it is on its way out)
    }
    for (AgentId who = 0; who < im.people.size(); ++who) {
        if (im.people[who].setOutTick == im.tickNumber) continue;   // (set out by the queue this tick)
        // (not to be read before its gate: asleep in a rest, or far on their way, ADR-0496 -- the island's 100k)
        if (total < im.dayGateOf[who]) continue;
        ++im.cost.dayReads;
        im.tickChoice(who);
        im.tickDay(who);
        im.tickRoleDay(who, dtHours);
    }
    im.cost.daysMs += msSince(part);
    im.shareOutTrace();
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    ++im.cost.ticks;
    im.cost.ms += ms;
    im.cost.worstMs = std::max(im.cost.worstMs, ms);
}

const std::vector<BrainMarker>& CityBrain::markers() const { return impl_->markers; }

std::string CityBrain::attentionText() const {
    const Impl& im = *impl_;
    if (!im.machine) return "no brain";
    int far = 0, held = 0, car = 0, inRing = 0, between = 0;
    const Vec2 c = im.sim.tierCenter();
    for (AgentId who = 0; who < im.people.size(); ++who) {
        const Impl::Person& p = im.people[who];
        if (p.level == brain::Detail::Far) { ++far; continue; }
        const Agent& a = im.sim.agents_[static_cast<std::size_t>(p.agent)];
        if (p.heldByDirector) ++held;
        else if (a.inCar >= 0) ++car;
        else if ((a.pos - c).length() < im.sim.pedPromoteRadius) ++inRing;
        else ++between;
    }
    return std::to_string(far) + " far; near: " + std::to_string(inRing) + " within " + fmt("%.0f m", im.sim.pedPromoteRadius) +
           ", " + std::to_string(between) + " not yet lowered, " + std::to_string(car) + " in a car, " +
           std::to_string(held) + " held by a director";
}

std::string CityBrainCost::describe() const {
    const double n = static_cast<double>(std::max<uint64_t>(1, ticks));
    return fmt("%.3f ms a tick", ms / n) + " (worst " + fmt("%.3f", worstMs) + "): walks " + fmt("%.3f", walksMs / n) +
           ", machine " + fmt("%.3f", machineMs / n) + " (planning " + fmt("%.3f", planMs / n) + ", " +
           fmt("%.0f plans", static_cast<double>(plans)) + ", the worst " + fmt("%.1f ms", worstPlanMs) + "), days " + fmt("%.3f", daysMs / n) + " (setting out " +
           fmt("%.3f", setOutMs / n) + "; " + fmt("%.0f", static_cast<double>(dayReads) / n) + " days read a tick, " +
           fmt("%.0f", static_cast<double>(goToReads) / n) + " on their way, " + fmt("%.0f", static_cast<double>(restReads) / n) +
           " rests read; the worst setting out " + fmt("%.1f ms", worstSetOutMs) + "), the rest " + fmt("%.3f", (ms - walksMs - machineMs - daysMs) / n);
}

void CityBrain::insidePlaces(std::unordered_map<PlaceId, PlaceInside>& out) const {
    out.clear();
    const Impl& im = *impl_;
    for (std::size_t who = 0; who < im.people.size(); ++who) {
        const Impl::Person& p = im.people[who];
        if (p.atMarker < 0 || p.atMarker >= static_cast<int>(im.markers.size())) continue;
        const PlaceId place = im.markers[static_cast<std::size_t>(p.atMarker)].place;
        if (place == kNoPlace) continue;
        const brain::AgentBrain& b = im.machine->agent(static_cast<AgentId>(who));
        if (b.travelling() || im.table.states[b.state].seen) continue;   // (on the way, or outside it: not in)
        const Agent& a = im.sim.agents_[static_cast<std::size_t>(p.agent)];
        PlaceInside& in = out[place];
        ++in.people;
        in.staff += a.workPlace == place && a.activity == Agent::Activity::AtWork;
    }
}

int CityBrain::farCount() const {
    int n = 0;
    for (const Impl::Person& p : impl_->people) n += p.level == brain::Detail::Far ? 1 : 0;
    return n;
}

bool CityBrain::residentFar(int who) const {
    return who >= 0 && static_cast<std::size_t>(who) < impl_->people.size() &&
           impl_->people[static_cast<std::size_t>(who)].level == brain::Detail::Far;
}

int CityBrain::markerNamed(const std::string& name) const {
    const std::unordered_map<std::string, int>::const_iterator it = impl_->markerByName.find(name);
    return it == impl_->markerByName.end() ? -1 : it->second;
}

int CityBrain::residentCount() const { return impl_->machine ? static_cast<int>(impl_->people.size()) : 0; }

int CityBrain::residentNamed(const std::string& nameOrNumber) const {
    const Impl& im = *impl_;
    const std::unordered_map<std::string, int>::const_iterator it = im.personByName.find(nameOrNumber);
    if (it != im.personByName.end()) return it->second;
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
    return im.car(ref).valid() ? static_cast<int>(ref.id) : -1;
}

std::vector<int> CityBrain::cars() const {
    std::vector<int> out;
    for (const Impl::Car c : impl_->cars) out.push_back(static_cast<int>(impl_->numberOf(c)));
    return out;
}

engine::Entity CityBrain::carEntity(int car) const { return impl_->carNumbered(car); }

bool CityBrain::carPose(int car, Vec2& pos, Vec2& heading) const {
    const Impl::Car c = impl_->carNumbered(car);
    if (!c.valid() || impl_->linkOf(c).away) return false;
    pos = impl_->carPos(c);
    heading = impl_->carFacing(c);
    return true;
}

bool CityBrain::inCar(int who) const { return impl_->isPerson(who) && impl_->agentOf(static_cast<AgentId>(who)).inCar >= 0; }

std::vector<int> CityBrain::membersOfCar(int car) const {
    const Impl& im = *impl_;
    std::vector<int> out;
    const Impl::Car c = im.carNumbered(car);
    if (!c.valid() || im.linkOf(c).household < 0) return out;
    for (const AgentId m : im.households[static_cast<std::size_t>(im.linkOf(c).household)].members)
        out.push_back(static_cast<int>(m));
    return out;
}

CarHolder CityBrain::carHolder(int car) const {
    const Impl::Car c = impl_->carNumbered(car);
    if (!c.valid() || impl_->keptOf(c).heldBy == brain::kNoAgent) return CarHolder::Nobody;
    return impl_->keptOf(c).heldBy == kOutsider ? CarHolder::Outsider : CarHolder::Resident;
}

std::string CityBrain::describeCar(int car) const {
    const Impl::Car c = impl_->carNumbered(car);
    return c.valid() ? impl_->carLine(c) : std::string("no household's car is car ") + std::to_string(car);
}

const std::vector<brain::Transition>& CityBrain::trace(int who) const {
    static const std::vector<brain::Transition> kNone;
    return impl_->isPerson(who) ? impl_->people[static_cast<std::size_t>(who)].trace : kNone;
}
const brain::Machine* CityBrain::machine() const { return impl_->machine.get(); }
const CityBrainCounts& CityBrain::counts() const { return impl_->counts; }
brain::PlanRoadCounts CityBrain::planRoadCounts() const {
    return impl_->planner ? impl_->planner->inner.roadCounts() : brain::PlanRoadCounts();
}
const CityBrainCost& CityBrain::cost() const { return impl_->cost; }
brain::BrainTime CityBrain::now() const { return impl_->now(); }
bool CityBrain::dayOn(int who) const { return impl_->isPerson(who) && impl_->people[static_cast<std::size_t>(who)].dayOn; }
void CityBrain::setDay(bool on) {
    for (int who = 0; who < static_cast<int>(impl_->people.size()); ++who) setDay(who, on);
}
void CityBrain::setDay(int who, bool on) {
    if (!impl_->isPerson(who)) return;
    Impl::Person& p = impl_->people[static_cast<std::size_t>(who)];
    p.dayOn = on && (p.roleDay || !p.day.empty());
    if (static_cast<std::size_t>(who) < impl_->dayGateOf.size()) impl_->dayGateOf[static_cast<std::size_t>(who)] = 0;
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
    return im.spotAt(im.markers[static_cast<std::size_t>(marker)], im.spotFor(marker, static_cast<AgentId>(who)));
}

std::string CityBrain::standingAt(int who) const {
    if (residentCount() == 0 || !impl_->isPerson(who)) return "";
    const Impl& im = *impl_;
    const Agent& body = im.agentOf(static_cast<AgentId>(who));
    if (body.inCar >= 0) return "";
    for (std::size_t i = 0; i < im.markers.size(); ++i) {
        const BrainMarker& m = im.markers[i];
        if (engine::atOrBefore((body.pos - im.spotAt(m, im.spotFor(static_cast<int>(i), static_cast<AgentId>(who)))).length(),
                               kWalkArrivedWithin, engine::kSamePoint))
            return m.name;
    }
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
    const Vec2 where = body.pos;   // (in a car: the car's place, followCars)
    s += " | the person (sim agent " + std::to_string(person.agent) + ") is at " + fmt("%.1f", where.x) + " " +
         fmt("%.1f", where.y) + (body.inCar >= 0 ? " in the car" : " on foot") +
         (person.heldByDirector ? ", moved by a director" : "");
    if (person.household >= 0)
        for (const uint32_t carId : im.households[static_cast<std::size_t>(person.household)].cars) {
            CarRef ref;
            ref.id = carId;
            if (const Impl::Car c = im.car(ref); c.valid()) s += " | " + im.carLine(c);
        }
    else
        s += " | no household, no car";
    const std::map<AgentId, uint32_t>::const_iterator h = im.held.find(id);
    if (h != im.held.end()) {
        PlaceRef p;
        p.id = h->second;
        s += " | held for them: " + im.placeName(p);
    }
    if (person.roleDay) {
        // their role's day: the goal state they are in (ADR-0491)
        const Agent& agent = impl_->sim.agents_[static_cast<std::size_t>(person.agent)];
        const GoalTable& t = impl_->sim.tableFor(agent);
        s += std::string(" | role day ") + (person.dayOn ? "on" : "off (brain day on)") + ", " +
             (agent.goal >= 0 && agent.goal < t.stateCount() ? t.state(agent.goal).name : std::string("?"));
    } else {
        s += std::string(" | day table ") + (person.day.empty() ? "none" : person.dayOn ? "on" : "off (brain day on)");
    }
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
    im.takeSpot(at, static_cast<AgentId>(who));
    const PlaceRef place = im.markerPlace(at, static_cast<AgentId>(who));
    if (!place.valid()) {
        if (why) *why = "marker \"" + marker + "\" has no spot left";
        return false;
    }
    im.machine->setIntent(static_cast<AgentId>(who), p.intentKind, place);
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
    const Impl::Car c = im.carNumbered(residentCar(who));
    if (!c.valid()) return space;
    const CityCarKept& k = im.keptOf(c);
    if (!im.onRoad(c) && k.heldBy != kOutsider && im.isSpace(k.standing)) im.spaces->spaceOf(k.standing, space);
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
    for (const Impl::Car c : im.cars) {
        const CityCarLink& link = im.linkOf(c);
        if (link.driver >= 0 || link.away) continue;
        const Real d = (im.carPos(c) - p).length();
        if (d > bestD) continue;
        bestD = d;
        best = static_cast<int>(im.numberOf(c));
    }
    return best;
}

bool CityBrain::board(int agent, int car) {
    Impl& im = *impl_;
    const Impl::Car c = im.machine ? im.carNumbered(car) : Impl::Car{};
    if (!c.valid() || agent < 0 || agent >= static_cast<int>(im.sim.agents_.size())) return false;
    const Agent& a = im.sim.agents_[static_cast<std::size_t>(agent)];
    if (a.inCar >= 0) return false;
    // (nobody gets into a car with somebody at its wheel; and one of the brain's own people gets in by their plan,
    // unless a director has them)
    if (im.linkOf(c).driver >= 0 || im.linkOf(c).away || im.keptOf(c).heldBy == kOutsider) return false;
    const int who = im.personOfAgent(agent);
    if (who >= 0 && !im.people[static_cast<std::size_t>(who)].heldByDirector) return false;
    LOG_INFO << "[brain] household car (car " << car << ") taken where it stood by sim agent " << agent;
    im.seat(agent, c);
    im.outsiderTook(c, false);
    ++im.counts.takenStanding;
    return true;
}

bool CityBrain::alight(int agent) {
    Impl& im = *impl_;
    if (!im.machine || agent < 0 || agent >= static_cast<int>(im.sim.agents_.size())) return false;
    const Impl::Car c = im.carNumbered(im.sim.agents_[static_cast<std::size_t>(agent)].inCar);
    // (an outsider in the car is at its wheel)
    if (!c.valid() || im.keptOf(c).heldBy != kOutsider || im.linkOf(c).driver != agent) return false;
    im.unseat(agent, c, im.outOf(c));
    im.outsiderLeft(c);
    return true;
}

bool CityBrain::takenByPlayer(int car) {
    Impl& im = *impl_;
    const Impl::Car c = im.machine ? im.carNumbered(car) : Impl::Car{};
    if (!c.valid() || im.keptOf(c).heldBy == kOutsider) return false;
    const int who = im.aboard(c) ? static_cast<int>(im.keptOf(c).heldBy) : -1;
    if (who >= 0) {
        // THE DRIVER IS PUT OUT ON THE KERB SIDE of where the car is, and told the car is gone; what they do next is
        // their state's row to say
        const AgentId id = static_cast<AgentId>(who);
        Vec2 out;
        if (!im.kerbSideOf(im.carPos(c), out)) out = im.outOf(c);
        im.unseat(im.people[id].agent, c, out);
        ++im.counts.putOut;
        im.machine->post(id, im.vehicleGone);
    }
    LOG_INFO << "[brain] household car (car " << car << ") taken by the player"
             << (who >= 0 ? " with " + im.people[static_cast<std::size_t>(who)].name + " at its wheel" : std::string());
    im.outsiderTook(c, true);
    ++im.counts.takenDriven;
    return true;
}

bool CityBrain::carLeftAt(int car, Vec2 pos, Vec2 heading) {
    Impl& im = *impl_;
    const Impl::Car c = im.machine ? im.carNumbered(car) : Impl::Car{};
    // (a car the player took with the key is a physical Vehicle on this entity, VehicleSystem's to move: the near tier
    // retires the Vehicle first, then gives it back, CitySim::playerLeftCar)
    if (!c.valid() || im.keptOf(c).heldBy != kOutsider || im.ecs->has<engine::Vehicle>(c)) return false;
    engine::Transform& t = im.poseOf(c);
    setCarPose(t, pos, unit(heading, carHeading(t)), t.position.y);
    im.outsiderLeft(c);
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
        // out of the household's car the director had them in
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
                         const std::vector<int>& agentOf, const std::vector<int>& classOfSlot, std::string* report,
                         bool drivableSpacesOnly, const PlaceMap* places) {
    std::shared_ptr<CityBrain> made = std::make_shared<CityBrain>(*this);
    made->setPlaces(places);
    if (!made->start(world, markers, residents, households, agentOf, classOfSlot, report, drivableSpacesOnly)) return false;
    brain_ = std::move(made);
    return true;
}

void CitySim::tickBrain(Real dt) { brain_->tick(dt); }
void CitySim::brainNoteHeld(int agent, bool on) {
    if (brain_) brain_->directorHolds(agent, on);
}

}  // namespace citysim
