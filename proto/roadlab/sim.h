#ifndef ROADLAB_SIM_H
#define ROADLAB_SIM_H

// The simulator exists in this prototype for one reason: to prove the data model
// is enough. A road system that renders beautifully but cannot answer "who has
// right of way here" or "which lane must I be in" has deferred the hard half of
// the problem.
//
// Nothing here adds new road data. Vehicles are (lane node, distance along it),
// which is the same (s, t) frame the shader uses. Lane-change legality reads the
// MARKING — the same field that draws the line. Right of way reads the junction
// conflict table — the same table the signal phases were coloured from. Where a
// lane ends, the taper the renderer draws is the taper the driver must merge
// within. One source, three consumers.

#include "network.h"
#include "junction.h"
#include "tessellate.h"
#include <vector>

namespace roadlab {

struct Vehicle {
    int id = 0;
    int lane = -1;          // lane graph node index
    double travelled = 0;   // metres along that node, in travel order
    double speed = 0;       // m/s
    double length = 4.6, width = 1.9;
    double desiredFactor = 1.0;
    double lateral = 0;     // offset from the lane centre, for lane changes
    int changingTo = -1;    // lane node being moved into
    double changeTime = 0;
    // The trip. `route` is a path of lane nodes from a Dijkstra over the lane
    // graph; `routePos` is where along it the vehicle currently is. Keeping the
    // route at LANE resolution rather than road resolution is what makes "you
    // must be in one of these two lanes by the time you get there" answerable.
    std::vector<int> route;
    size_t routePos = 0;
    int targetRoad = -1;    // the road this trip ends on
    double routeUrgency = 0;   // 0 = on-route, 1 = must change lanes now
    // The reachability query behind routeDemand is a graph search, and a driver
    // does not re-ask it ten times a second. Cached and refreshed on a timer,
    // with the distance carried forward by how far they have actually driven.
    double routeCheckIn = 0;
    int cachedDemandRoad = -1;
    double cachedDemandDist = 1e9;
    Vec3 color{0.6, 0.6, 0.6};
    bool active = true;
    double waited = 0;      // seconds spent below 0.5 m/s
};

struct ParkedCar {
    Vec3 pos;
    double yaw = 0;
    Vec3 color{0.5, 0.5, 0.5};
};

struct ParkingSlot {
    int road = -1;
    double s = 0, t = 0;
    double length = 5.6;
    bool occupied = false;
};

// The footway network. Sidewalk strips are edges; the links between them are
// either a CORNER (walking round an intersection on the footway) or a CROSSING
// (stepping off the kerb onto the zebra the paint generator drew). The crossings
// come from junctionCrosswalks() — the same function that produces the paint —
// so a pedestrian can only cross where a crossing is actually painted.
struct WalkNode {
    int road = -1;
    int side = 1;           // +1 left footway, -1 right
    bool atEnd = true;      // which end of the road this node sits at
    int junction = -1;
    Vec3 pos{0, 0, 0};
};

struct WalkLink {
    int a = -1, b = -1;
    bool crossing = false;  // false = corner / continuation, true = carriageway
    int road = -1;          // the road being crossed
    double s = 0;           // station of the crossing
    int junction = -1;
    double length = 4.0;
};

struct Pedestrian {
    int node = -1;          // the footway end being walked toward
    int road = -1;
    int side = 1;
    double s = 0;
    double dir = 1;         // +1 with s
    double speed = 1.35;
    // Crossing state. A pedestrian either walks a footway or occupies a
    // crossing; vehicles read the second case and give way.
    int pendingLink = -1;   // a crossing decided on but not yet safe to start
    int crossingLink = -1;
    double crossProgress = 0;
    double waiting = 0;     // seconds spent at a kerb
    bool active = true;
    Vec3 color{0.7, 0.6, 0.5};
};

struct SimStats {
    int vehicles = 0;
    int moving = 0;
    int stopped = 0;
    double meanSpeedKph = 0;
    double meanWait = 0;
    int laneChanges = 0;
    int mandatoryChanges = 0;
    int completedTrips = 0;
    int replans = 0;
    int offRoute = 0;       // vehicles currently in a lane that cannot serve their route
    int pedestrians = 0;
    int pedsCrossing = 0;
    int pedsWaiting = 0;
    int crossingsMade = 0;
    double meanPedWait = 0;
    int parkedCars = 0;
};

struct SimParams {
    double idmMaxAccel = 1.6;      // m/s^2
    double idmComfortDecel = 2.2;
    double idmMinGap = 2.2;        // m
    double idmHeadway = 1.35;      // s
    double lateralAccelLimit = 2.6;   // m/s^2 through curves
    double laneChangeDuration = 2.6;  // s
    double yieldHorizon = 4.0;     // s of approaching traffic that blocks a yield
    double routeLookahead = 400.0; // how far ahead a driver plans their lane
    int routeMaxDepth = 8;         // lane-graph hops searched for "can I still get there"
    bool routing = true;
    double pedCrossGap = 4.5;      // seconds of clear road a pedestrian wants
    double pedMaxWait = 25.0;      // after which they take a smaller gap
    bool vehiclesYieldToPeds = true;
    double spawnRespawn = true;    // recycle vehicles that run out of road
    uint32_t seed = 3;
};

class Simulation {
public:
    Simulation(const Network& net, const SimParams& params = {});

    void seedVehicles(int count);
    void seedPedestrians(int count);
    void seedParking(double occupancy = 0.55);

    void step(double dt);
    void run(double seconds, double dt = 0.1);

    double time() const { return time_; }
    const std::vector<Vehicle>& vehicles() const { return vehicles_; }
    const std::vector<Pedestrian>& pedestrians() const { return peds_; }
    const std::vector<ParkingSlot>& parking() const { return slots_; }
    const std::vector<ParkedCar>& parkedCars() const { return parked_; }
    const std::vector<WalkNode>& walkNodes() const { return walkNodes_; }
    const std::vector<WalkLink>& walkLinks() const { return walkLinks_; }
    // Would a pedestrian step onto this crossing right now? Public because it is
    // the thing worth asserting about: at a signalised junction the answer must
    // agree with the phase table the drivers are obeying.
    bool pedestrianMayCross(int linkIndex, double patience = 0.0) const;
    SimStats stats() const;

    // World pose of a vehicle, including its lane-change lateral offset.
    bool vehiclePose(const Vehicle& v, Vec3& pos, double& yaw) const;
    bool pedestrianPose(const Pedestrian& p, Vec3& pos, double& yaw) const;

private:
    const Network& net_;
    SimParams params_;
    Rng rng_;
    double time_ = 0;
    int nextId_ = 1;
    int laneChanges_ = 0;
    int mandatory_ = 0;
    int completed_ = 0;
    int replans_ = 0;

    std::vector<Vehicle> vehicles_;
    std::vector<Pedestrian> peds_;
    std::vector<WalkNode> walkNodes_;
    std::vector<WalkLink> walkLinks_;
    std::vector<std::vector<int>> walkLinksOf_;
    // Crossings currently occupied, so a vehicle can be told to give way. Keyed
    // by (road, station) because that is how a driver perceives it.
    struct ActiveCrossing {
        int road;
        double s;
    };
    std::vector<ActiveCrossing> occupiedCrossings_;
    double pedWaitTotal_ = 0;
    int crossingsMade_ = 0;
    std::vector<ParkingSlot> slots_;
    std::vector<ParkedCar> parked_;

    // Per lane node, the vehicles on it ordered by distance travelled. Rebuilt
    // each step; this ordering is what makes leader/follower queries O(1) and is
    // the reason a lane is stored as an ordered list rather than a soup.
    std::vector<std::vector<int>> laneOccupants_;

    // connector road id -> (junction, connection index)
    std::vector<std::pair<int, int>> connectorOf_;
    // Per junction, per connection: is a vehicle on it, and how soon is the
    // nearest approaching vehicle.
    struct JunctionState {
        std::vector<char> occupied;
        std::vector<double> arrival;
    };
    std::vector<JunctionState> junctionState_;

    void rebuildOccupancy();
    void updateJunctionState();
    double laneLength(int node) const;
    double roadStationOf(const Vehicle& v) const;
    double desiredSpeedFor(const Vehicle& v) const;
    double gapToLeader(const Vehicle& v, double& leaderSpeed, double horizon = 140.0) const;
    double gapBehind(int laneNode, double travelled, double& followerSpeed) const;
    bool blockedByJunction(const Vehicle& v, double& distance) const;
    bool blockedByCrossing(const Vehicle& v, double& distance) const;
    void buildWalkGraph();
    // Is it safe to step off the kerb? Signalised crossings read the junction's
    // own phase table; everything else is gap acceptance against real vehicles.
    bool crossingClear(const WalkLink& link, double patience) const;
    int findWalkNode(int road, int side, bool atEnd) const;
    void stepPedestrians(double dt);
    int chooseSuccessor(Vehicle& v);
    // Dijkstra over the lane graph by travel time. Returns a lane-node path to a
    // destination chosen from what is actually reachable.
    std::vector<int> planRoute(int fromNode);
    void assignRoute(Vehicle& v);
    // The next road on the route that the vehicle's current lane cannot reach,
    // and how far away it is. -1 when the current lane still serves the plan.
    int routeDemand(const Vehicle& v, double& distance) const;
    void considerLaneChange(Vehicle& v);
    void respawn(Vehicle& v);
    int randomStartLane();
};

// Cars, parked cars and pedestrians as boxes, so a render can show the sim.
void tessellateAgents(const Simulation& sim, Mesh& out);

}  // namespace roadlab

#endif
