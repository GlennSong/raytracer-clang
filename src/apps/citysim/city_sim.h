#ifndef RAYTRACER_APPS_CITYSIM_CITY_SIM_H
#define RAYTRACER_APPS_CITYSIM_CITY_SIM_H

#include "../../engine/ai/nav_graph.h"
#include "../../engine/ai/pathfind.h"
#include "traffic_signal.h"
#include <cstdint>
#include <vector>

namespace citysim {

using engine::Real;   // the engine's scalar (double); used throughout the sim

// The agent-based city simulation (ADR-0059). An Agent is a brain (data): it
// either WALKS (a pedestrian) or POSSESSES and drives a SimVehicle. A car has no
// agency of its own — a SimVehicle with no driver is inert (parked). The player
// is modelled as an agent too (playerControlled = its brain is host input); the
// AI brain perceives and obeys traffic rules. This is the deterministic core
// (kinematic motion along the NavGraph); Jolt physics + instanced models are the
// device-verified skin layered on top in a later phase.

struct Agent {
    enum class Mode : uint8_t { Pedestrian, Driver };
    enum class Activity : uint8_t { AtHome, Commuting, AtWork, Returning };
    // Reactive behaviour state (ADR-0060): what the agent is doing right now,
    // decided from what it can SEE this step. Rendered by the debug widgets.
    enum class State : uint8_t { Resting, Walking, Avoiding, Waiting };

    Mode mode = Mode::Driver;
    State state = State::Resting;
    bool playerControlled = false;   // brain = host input; the sim won't auto-drive it
    int vehicle = -1;                // possessed SimVehicle index (Driver only; -1 = none)

    // Daily schedule (hours, 0..24), per-agent jittered.
    int home = 0, work = 0;
    Real departWork = 8.0, departHome = 17.0;
    Activity activity = Activity::AtHome;

    // Current trip along the lane graph.
    engine::Route route;
    int leg = 0;
    Real distOnLeg = 0, speed = 0, elevation = 0;
    int lane = 0;

    // Imperfect perception (ADR-0059): each step the agent perceives obstacles
    // with probability `reliability`; otherwise it misses (a fault). `brain` is a
    // per-agent deterministic RNG so faults reproduce from the sim seed.
    Real reliability = 1.0;
    uint32_t brain = 1;

    // Cached world pose (XZ + a height for bridges); what a renderer reads.
    engine::Vec2 pos;
    engine::Vec2 heading{1, 0};
    bool moving = false;
};

// A drivable car. Kinematic in the sim core; its pose tracks its driver. Inert
// when `driver < 0`.
struct SimVehicle {
    Real length = 4.2;
    Real width = 1.8;
    int driver = -1;             // agent index, or -1 when parked/unpossessed
    engine::Vec2 pos;            // cached pose (mirrors the driver while possessed)
    engine::Vec2 heading{1, 0};
};

class CitySim {
public:
    // `driverCount` agents that each own (possess) a car + `pedCount` walking
    // agents, over `graph`, seeded by `seed`. Deterministic (ADR-0002).
    void build(const engine::NavGraph& graph, int driverCount, int pedCount,
               uint32_t seed);

    // Advance by `dt` seconds; `hoursPerSecond` maps real time to the wrapping
    // in-world clock. Signals advance with it.
    void step(Real dt, Real hoursPerSecond = 0.05);

    Real timeOfDay() const { return clockHours_; }
    const std::vector<Agent>& agents() const { return agents_; }
    const std::vector<SimVehicle>& vehicles() const { return vehicles_; }
    const engine::NavGraph* graph() const { return nav_; }
    SignalController& signals() { return signals_; }

    // Mark an agent as host-driven (the player): the sim won't run its AI brain.
    void setPlayerControlled(int agentIndex, bool on) {
        if (agentIndex >= 0 && agentIndex < static_cast<int>(agents_.size()))
            agents_[agentIndex].playerControlled = on;
    }

    // Set every agent's perception reliability (1 = perfect; <1 = makes faults).
    void setPerceptionReliability(Real r) {
        for (Agent& a : agents_) a.reliability = r;
    }
    long faults() const { return faultCount_; }   // perception misses so far

    // World-space (XZ) points that cars must yield to in addition to the sim's own
    // pedestrians — chiefly the live player (on foot or in a car), injected by the
    // host each step so AI cars brake for and hold short of the player.
    void setExternalObstacles(std::vector<engine::Vec2> obstacles) {
        externalObstacles_ = std::move(obstacles);
    }

private:
    void startTrip(Agent& a, int origin, int goal);
    void advance(Agent& a, Real dt, Real gap);
    void computeGaps();
    Real brainUnit(Agent& a);   // per-agent deterministic roll for faults
    void refreshPose(Agent& a);
    void steer(Agent& a, Real dt);   // rate-limited heading (bounded turn radius)
    engine::Vec2 idlePose(int node, Agent::Mode mode) const;
    uint32_t rnd();
    Real rndUnit();

    const engine::NavGraph* nav_ = nullptr;
    std::vector<Agent> agents_;
    std::vector<SimVehicle> vehicles_;
    std::vector<Real> gaps_;
    std::vector<engine::Vec2> positions_;   // per-step snapshot of ped + player pos (cars yield to these)
    std::vector<engine::Vec2> externalObstacles_;   // host-injected (the live player)
    SignalController signals_;
    long faultCount_ = 0;
    Real clockHours_ = 6.0;
    uint32_t rng_ = 1;
};

}  // namespace citysim

#endif
