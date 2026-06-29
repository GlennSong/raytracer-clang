#ifndef RAYTRACER_ENGINE_AI_AGENT_SIM_H
#define RAYTRACER_ENGINE_AI_AGENT_SIM_H

#include "nav_graph.h"
#include "pathfind.h"
#include <cstdint>
#include <vector>

namespace engine {

enum class AgentKind : uint8_t { Car, Pedestrian };

// Car-following speed cap (ADR-0058): given an agent's free-flow target speed and
// the centre-to-centre gap to the agent ahead on its lane, return the speed it may
// safely travel. Full stop at/under `minGap` (bumper distance), linear ramp up to
// `slowZone`, free above it. Pure + deterministic, so it is unit-tested directly.
Real carFollowingCap(Real freeSpeed, Real gap, Real minGap, Real slowZone);

// What an agent is doing right now. The daily loop is
// AtHome -> Commuting -> AtWork -> Returning -> AtHome (ADR-0058).
enum class AgentActivity : uint8_t { AtHome, Commuting, AtWork, Returning };

// One simulated inhabitant. Lives between a `home` node and a `work` node, with a
// daily schedule; while travelling it follows a Route at a class-appropriate
// (cars) or walking (pedestrians) speed. `pos`/`heading` are the cached world
// pose, refreshed every step — what a renderer or ECS bridge reads.
struct Agent {
    AgentKind kind = AgentKind::Car;
    int home = 0;                       // NavGraph node
    int work = 0;                       // NavGraph node
    AgentActivity activity = AgentActivity::AtHome;

    // Daily schedule, in-world hours [0,24): leave home for work at `departWork`,
    // leave work for home at `departHome` (per-agent jitter baked in at build).
    Real departWork = 8.0;
    Real departHome = 17.0;

    // Current trip.
    Route route;
    int leg = 0;                        // index into route.links
    Real distOnLeg = 0;                 // metres travelled along the current link
    int lane = 0;                       // chosen lane within the link (cars)
    Real speed = 0;                     // current speed (m/s)

    Vec2 pos;                           // cached world position (XZ)
    Real elevation = 0;                 // height above ground (bridge deck while on an overpass)
    Vec2 heading{1, 0};                 // cached unit heading
    bool moving = false;
};

// A deterministic agent simulation (ADR-0002 sim discipline): for a given seed
// and dt sequence it reproduces identical trajectories. Owns the agents and an
// in-world clock; holds a NavGraph by reference (must outlive the sim).
class AgentSim {
public:
    // Populate `carCount` cars + `pedCount` pedestrians over `graph`, each with a
    // random home/work pair and jittered schedule, seeded by `seed`.
    void build(const NavGraph& graph, int carCount, int pedCount, uint32_t seed);

    // Advance by `dt` real seconds. `hoursPerSecond` maps real time to the
    // in-world clock (so a full day passes in minutes); the clock wraps at 24h.
    void step(Real dt, Real hoursPerSecond = 0.05);

    Real timeOfDay() const { return clockHours; }
    const std::vector<Agent>& agents() const { return agentList; }
    const NavGraph* graph() const { return nav; }

private:
    void startTrip(Agent& a, int originNode, int goalNode);
    void advance(Agent& a, Real dt, Real gap);
    void computeGaps();                // per-lane leader gaps for moving agents
    void refreshPose(Agent& a);
    // Resting pose at a node, OFF the centreline: cars at the curb, pedestrians on
    // the sidewalk — so parked agents don't sit in the middle of the road.
    Vec2 idlePose(int node, AgentKind kind) const;
    uint32_t nextRandom();
    Real randomUnit();

    const NavGraph* nav = nullptr;
    std::vector<Agent> agentList;
    std::vector<Real> gaps_;           // centre-to-centre gap to each agent's leader
    Real clockHours = 6.0;             // sim starts at 06:00, before the commute
    uint32_t rngState = 1;
};

}  // namespace engine

#endif
