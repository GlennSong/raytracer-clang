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
    int targetRoad = -1;    // where this trip is heading
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

struct Pedestrian {
    int road = -1;
    int side = 1;           // +1 left footway, -1 right
    double s = 0;
    double dir = 1;         // +1 with s
    double speed = 1.35;
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
    int completedTrips = 0;
    int pedestrians = 0;
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
    int completed_ = 0;

    std::vector<Vehicle> vehicles_;
    std::vector<Pedestrian> peds_;
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
    int chooseSuccessor(int node);
    void considerLaneChange(Vehicle& v);
    void respawn(Vehicle& v);
    int randomStartLane();
};

// Cars, parked cars and pedestrians as boxes, so a render can show the sim.
void tessellateAgents(const Simulation& sim, Mesh& out);

}  // namespace roadlab

#endif
