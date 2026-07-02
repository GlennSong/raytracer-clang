#ifndef RAYTRACER_APPS_CITYSIM_CITY_SIM_H
#define RAYTRACER_APPS_CITYSIM_CITY_SIM_H

#include "../../engine/ai/agent_memory.h"
#include "../../engine/ai/nav_graph.h"
#include "../../engine/ai/pathfind.h"
#include "traffic_signal.h"
#include <cstdint>
#include <utility>
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
    // decided from what it can SEE this step. Rendered by the debug widgets. The
    // first four are shared / pedestrian; the rest are the driver FSM (Cruising →
    // Following → Yielding → Turning, with Waiting for a held red). `Count` sizes
    // the render-side per-state arrays — keep it last.
    enum class State : uint8_t {
        Resting, Walking, Avoiding, Waiting,   // ped + shared
        Cruising, Following, Yielding, Turning, // driver FSM
        Count
    };

    Mode mode = Mode::Driver;
    State state = State::Resting;
    bool playerControlled = false;   // brain = host input; the sim won't auto-drive it
    bool released = false;           // ejected by the player (ADR-0061): the sim stops
                                     // driving this agent's ghost so it can't fight the
                                     // now player-driven physical car
    int vehicle = -1;                // possessed SimVehicle index (Driver only; -1 = none)

    // Daily schedule (hours, 0..24), per-agent jittered.
    int home = 0, work = 0;
    Real departWork = 8.0, departHome = 17.0;
    Activity activity = Activity::AtHome;

    // Think cadence (ADR-0061): agents DECIDE on a slow clock and COMMIT — the
    // reactive scan (what do I see, which way do I lean) runs only when
    // thinkTimer expires (staggered per agent), and the committed decision
    // (`leanTarget`, the FSM state) holds between thinks while every tick still
    // INTEGRATES toward it. Per-tick re-deciding is what made walkers oscillate:
    // a new answer every 100 ms reads as wigging out, a held answer as intent.
    Real thinkTimer = 0;
    Real leanTarget = 0;   // committed sideways lean (m); lateralOffset chases it

    // Current trip along the lane graph.
    engine::Route route;
    int leg = 0;
    int trips = 0;   // trips started so far — the ADR-0061 bridge rebuilds its
                     // pursuit path when this changes (a new route = a new path)
    Real distOnLeg = 0, speed = 0, elevation = 0;
    int lane = 0;
    // Tether (ADR-0061): while set, this planner ghost may not LEAD `tetherAnchor`
    // (its physical car) by more than `tetherLead` metres — it waits instead. The
    // host feeds the car's real position each step, so the plan can never outrun
    // the physics (a collision, a hill, a slow start no longer strands the car).
    bool tethered = false;
    engine::Vec2 tetherAnchor;
    Real tetherLead = 10.0;
    // Continuous sideways lean off the path (ADR-0060), rate-limited, so a walker
    // steers smoothly around what it sees instead of popping. Decays back to 0.
    Real lateralOffset = 0;

    // Imperfect perception (ADR-0059): each step the agent perceives obstacles
    // with probability `reliability`; otherwise it misses (a fault). `brain` is a
    // per-agent deterministic RNG so faults reproduce from the sim seed.
    Real reliability = 1.0;
    uint32_t brain = 1;
    // Working memory (ADR-0062: sense -> REMEMBER -> predict -> decide -> act).
    // Sightings become tracks with velocity estimates; a body that leaves the
    // cone persists a few seconds, extrapolated along where it was heading
    // (object permanence), so the agent acts on a continuous world instead of a
    // per-tick snapshot — and can anticipate a crossing before it happens.
    engine::AgentMemory memory;
    // Personality (ADR-0061): this agent's personal pace as a fraction of the
    // nominal speed — a timid driver holds ~0.85x the limit, a pushy one ~1.15x,
    // and walkers stroll or stride. Derived from `brain`'s bits at build (no rng
    // draw, so seeded scenarios are unchanged); traffic stops moving in lockstep.
    Real speedFactor = 1.0;

    // Cached world pose (XZ + a height for bridges); what a renderer reads.
    engine::Vec2 pos;
    engine::Vec2 heading{1, 0};
    bool moving = false;
};

// The kind of body a vehicle wears (ADR-0060 Phase 4: one composable vehicle,
// varied by a Body component). Dimensions come from the shared fleet table below,
// so the sim (following distance, colliders) and the renderer (mesh, lift) agree.
enum class VehicleType : uint8_t { Sedan, Hatchback, SUV, Pickup, Van, BoxTruck };

// A body's physical dimensions (metres) + its type. `length` is the travel axis;
// car-following keeps cars a bumper apart from THIS, so a longer truck naturally
// holds (and is held at) a larger gap.
struct VehicleBody {
    Real length = 4.2, width = 1.8, height = 1.3;
    VehicleType type = VehicleType::Sedan;
};

// The fleet: a fixed, deterministic set of body slots. A driver takes slot
// (vehicleIndex % size), so its body is stable and the renderer can mirror it.
int vehicleFleetSize();
const VehicleBody& vehicleFleetBody(int slot);   // slot wraps into [0, size)

// A drivable car. Kinematic in the sim core; its pose tracks its driver. Inert
// when `driver < 0`. Its body (type + dimensions) comes from the fleet table.
struct SimVehicle {
    Real length = 4.2;
    Real width = 1.8;
    Real height = 1.3;
    VehicleType type = VehicleType::Sedan;
    int driver = -1;             // agent index, or -1 when parked/unpossessed
    engine::Vec2 pos;            // cached pose (mirrors the driver while possessed)
    engine::Vec2 heading{1, 0};
};

// A body some agent might sense this step (ADR-0062): its plan position, its
// elevation (bridges — the 2.5D sensor gates on height), and a STABLE id so an
// observer's memory can track it across steps. Sim agents use their agent index;
// host-injected external obstacles use -(1+k) for the k-th injected point.
struct SensedGhost {
    engine::Vec2 pos;
    Real elevation = 0;
    int id = -1;
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
    Real seconds() const { return simSeconds_; }   // monotonic sim clock (memory time base)
    const std::vector<Agent>& agents() const { return agents_; }
    const std::vector<SimVehicle>& vehicles() const { return vehicles_; }
    const engine::NavGraph* graph() const { return nav_; }
    SignalController& signals() { return signals_; }

    // Mark an agent as host-driven (the player): the sim won't run its AI brain.
    void setPlayerControlled(int agentIndex, bool on) {
        if (agentIndex >= 0 && agentIndex < static_cast<int>(agents_.size()))
            agents_[agentIndex].playerControlled = on;
    }

    // Release an agent whose car the player commandeered (ADR-0061): the sim parks
    // its ghost and stops advancing it, so the brain no longer fights the physical
    // car the player now drives. Idempotent; -1/out-of-range is a no-op.
    void releaseDriver(int agentIndex) {
        if (agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size())) return;
        Agent& a = agents_[agentIndex];
        a.released = true;
        a.tethered = false;
        a.moving = false;
        a.speed = 0;
    }

    // Tether a planner ghost to its physical car (ADR-0061): the ghost holds
    // whenever it is more than `maxLead` metres from `anchor` (the car's real
    // position, re-fed each step). Determinism holds for an identical call
    // sequence — the host owns whatever nondeterminism it feeds in.
    void setAgentTether(int agentIndex, engine::Vec2 anchor, Real maxLead) {
        if (agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size())) return;
        Agent& a = agents_[agentIndex];
        a.tethered = true;
        a.tetherAnchor = anchor;
        a.tetherLead = maxLead;
    }
    void clearAgentTether(int agentIndex) {
        if (agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size())) return;
        agents_[agentIndex].tethered = false;
    }

    // Sample agent `agentIndex`'s current route as a polyline of lane-centre
    // (driver) / sidewalk (pedestrian) points, ~`step` metres apart — the pursuit
    // path the ADR-0061 bridge follows from the car's real pose. Empty when the
    // agent has no active route.
    std::vector<engine::Vec2> lanePath(int agentIndex, Real step = 3.0) const;

    // Is `pos` inside a junction box (within a junction node's widest incident
    // half-width + `margin`)? The bridge uses this for don't-block-the-box (a
    // physical car never idles inside one) and for spawn placement.
    bool nearJunction(engine::Vec2 pos, Real margin = 0.0) const;

    // Set every agent's perception reliability (1 = perfect; <1 = makes faults).
    void setPerceptionReliability(Real r) {
        for (Agent& a : agents_) a.reliability = r;
    }

    // How often an agent re-DECIDES its reactive behaviour (seconds). Between
    // thinks it commits to the last decision and just acts on it. Default 0.35 s.
    void setThinkPeriod(Real seconds) { thinkPeriod_ = seconds > 0.05 ? seconds : 0.05; }
    Real thinkPeriod() const { return thinkPeriod_; }
    long faults() const { return faultCount_; }   // perception misses so far

    // World-space (XZ) points that cars must yield to in addition to the sim's own
    // pedestrians — chiefly the live player (on foot or in a car), injected by the
    // host each step so AI cars brake for and hold short of the player.
    void setExternalObstacles(std::vector<engine::Vec2> obstacles) {
        externalObstacles_ = std::move(obstacles);
    }

    // World-space (XZ) static obstacles pedestrians steer around and never stand
    // inside — chiefly the signal poles on the sidewalks (injected once by the host
    // from the render bridge). Cars ignore these (poles sit off the carriageway).
    void setStaticObstacles(std::vector<engine::Vec2> obstacles) {
        staticObstacles_ = std::move(obstacles);
    }

private:
    void startTrip(Agent& a, int origin, int goal);
    void advance(Agent& a, Real dt, Real gap, Real minGap);
    void computeGaps();
    Real vehicleLength(int agentIndex) const;      // body length, or a ped's footprint
    Real pairMinGap(int follower, int leader) const;   // bumper-to-bumper follow gap
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
    std::vector<Real> minGaps_;   // per-agent follow gap to ITS leader (length-aware)
    std::vector<SensedGhost> sensed_;   // per-step snapshot of bodies cars/peds may SEE
                                        // (peds + players + external obstacles)
    std::vector<engine::Vec2> externalObstacles_;   // host-injected (the live player)
    std::vector<engine::Vec2> staticObstacles_;     // host-injected, static (signal poles)
    std::vector<std::pair<engine::Vec2, Real>> junctions_;   // centre + box radius
    SignalController signals_;
    long faultCount_ = 0;
    Real clockHours_ = 6.0;
    Real simSeconds_ = 0;   // seconds since build — the time base memory decays on
    Real thinkPeriod_ = 0.35;   // reactive re-decide cadence (s), staggered per agent
    uint32_t rng_ = 1;
};

}  // namespace citysim

#endif
