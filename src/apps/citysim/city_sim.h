#ifndef RAYTRACER_APPS_CITYSIM_CITY_SIM_H
#define RAYTRACER_APPS_CITYSIM_CITY_SIM_H

#include "../../engine/ai/agent_memory.h"
#include "../../engine/ai/nav_graph.h"
#include "../../engine/ai/pathfind.h"
#include "agent_grid.h"
#include "agent_id.h"
#include "city_goals.h"
#include "city_bus.h"
#include "city_dispatch.h"
#include "city_transit.h"
#include "places.h"
#include "relationships.h"
#include "traffic_signal.h"
#include <cstdint>
#include <utility>
#include <vector>

namespace citysim {

using engine::Real;   // the engine's scalar (double); used throughout the sim

// How high one grade-separation LAYER sits above the one below (m). A legacy
// bridge link carries `layer > 0` instead of an absolute deck elevation, so
// everything that places geometry on such a link — the sim's agent height, the
// render bridge's decals, parked cars and bay outlines — must lift by the same
// amount. Shared so those readings cannot drift apart.
constexpr Real kLayerClearance = 5.8;

// The agent-based city simulation (ADR-0060). An Agent is a brain (data): it
// either WALKS (a pedestrian) or POSSESSES and drives a SimVehicle. A car has no
// agency of its own — a SimVehicle with no driver is inert (parked). The player
// is modelled as an agent too (playerControlled = its brain is host input); the
// AI brain perceives and obeys traffic rules. This is the deterministic core
// (kinematic motion along the NavGraph); Jolt physics + instanced models are the
// device-verified skin layered on top in a later phase.

struct Agent {
    enum class Mode : uint8_t { Pedestrian, Driver };
    // The GOAL layer's outward label (ADR-0064): what the agent's day currently
    // reads as. The value comes from the agent's goal STATE in the archetype's
    // GoalTable (city_goals.h) — kept as a field so tests/renderers keep their
    // historical `Agent::Activity` reads while the table drives the behaviour.
    using Activity = citysim::Activity;
    // Reactive behaviour state (ADR-0061): what the agent is doing right now,
    // decided from what it can SEE this step. Rendered by the debug widgets. The
    // first four are shared / pedestrian; the rest are the driver FSM (Cruising →
    // Following → Yielding → Turning, with Waiting for a held red). `Count` sizes
    // the render-side per-state arrays — keep it last.
    enum class State : uint8_t {
        Resting, Walking, Avoiding, Waiting,   // ped + shared
        Cruising, Following, Yielding, Turning, // driver FSM
        Count
    };

    // Simulation tier (P4 three-tier traffic). K (Kinematic) is the full agent
    // as it always ran: sensing, FSM, signals, render bake, kinematic proxy.
    // V (Virtual) is a persistent FAR agent: full identity + exact route state
    // (link, lane, param, speed) but no rendering, no physics proxy, no
    // sensing/FSM and no live signal queries — it traverses its real A* route
    // on a coarse ~1 Hz tick at a modelled speed (class speed shaped by fixed
    // average junction delays), so promotion finds it where it should be.
    // P (the physical Jolt tier) stays what it was: possessTier's overlay on K.
    // Everything is K until a level opts in (CitySim::tieringEnabled) AND a
    // player position is fed (setTierCenter) — headless sims are unchanged.
    // D (Dormant) is the tier BELOW V: not simulated at all, not even coarsely.
    // A V agent still walks its route link by link once a second — cheap, but
    // not free, and there are thousands of them. A D agent costs nothing: when
    // the player comes near it is reconstructed from its own schedule
    // (scheduleSnapshot) and handed to V.
    //
    // Its CAR is untouched by any of this. Dormancy strips the agent, never the
    // vehicle — a parked car keeps its pose, its bay, its instance and its
    // proxy, which is what lets it still be seen, collided with and stolen
    // while its owner does not exist.
    enum class Tier : uint8_t { D, V, K };

    // Stable identity (Living City, ADR-0066 Phase 1). Assigned once at build in
    // agent order and never reused for a different agent this run, so relationship
    // tables / per-agent memory / job assignment key on THIS, not the array slot.
    // Distinct from the array index by design (indices churn on rebuild/recycle).
    AgentId uid = kNoAgent;

    // What KIND of life this agent leads (Living City, ADR-0066 Phase 4). A role
    // is not new machinery — it just flavours the existing home↔work schedule:
    // a Commuter keeps office hours; a Shopkeeper opens their shop before it opens
    // and closes it after; a Stroller has no job and spends the day at a park.
    // Assigned in assignPlaces from the agent's workplace + its own brain bits.
    enum class Role : uint8_t { Commuter, Shopkeeper, Stroller, Count };
    Role role = Role::Commuter;

    // HOW THIS AGENT IS MOVING RIGHT NOW. Nearly everything that reads `mode`
    // is asking exactly this — which route space to plan in, whether to sample a
    // lane or a sidewalk, whether to draw a car or a person, whether to grow a
    // Jolt character or a vehicle proxy — so flipping it is how an agent parks
    // and walks away. It is NOT identity: see `archetype`.
    Mode mode = Mode::Driver;
    // WHAT THIS AGENT IS, for the whole run. Only the goal-table lookup reads
    // this. Keeping it separate is what stops an agent hopping to a different
    // daily schedule the moment it gets out of its car.
    Mode archetype = Mode::Driver;
    State state = State::Resting;
    Tier tier = Tier::K;
    // Sim-seconds at which a DORMANT agent stopped being simulated. Its state is
    // reconstructed from its schedule on waking, so this is only telemetry.
    Real dormantSince = 0;
    // "Is this agent too far away to have a body?" — no drawn instance, no
    // physics proxy, nothing for anyone to sense or brake for.
    //
    // Ask through this rather than comparing to a specific tier. Nearly every
    // site in the sim wants exactly this question, and writing it as
    // `tier == Tier::V` silently means "the ONE far tier", so adding another
    // one below V turns each of those into a wrong answer with no compile
    // error. The tier PASS itself still compares explicitly — it is the thing
    // that decides which tier an agent belongs in.
    bool far() const { return tier != Tier::K; }
    // V-tier bookkeeping: sim-seconds at this agent's last coarse tick (its
    // next tick advances by the difference — no wall clock), and the remaining
    // modelled junction dwell it is currently "waiting out" mid-route.
    Real vLastTick = 0;
    Real vHold = 0;
    // SCHEDULED WAKE. An agent resting at home or at work has nothing to decide
    // until its own day says otherwise, so instead of re-reading the clock every
    // tick it names the sim-second it next needs thinking about and drops out of
    // the active list entirely until then. `wakeAt` < 0 means awake; `sleptAt`
    // is the UNWRAPPED in-world hour it went under (clockTotalHours), so the
    // dwell clock is caught up exactly on waking whatever the rate did
    // meanwhile — the sky's loop can be retuned or held under a sleeper.
    //
    // This is why the schedule had to be fixed first: a population that all
    // departs in the same 90 minutes has almost nobody asleep at any moment.
    Real wakeAt = -1;
    Real sleptAt = 0;
    bool playerControlled = false;   // brain = host input; the sim won't auto-drive it
    bool released = false;           // ejected by the player (ADR-0062): the sim stops
                                     // driving this agent's ghost so it can't fight the
                                     // now player-driven physical car
    int vehicle = -1;                // the car being DRIVEN right now; -1 while on foot
    // The car this agent OWNS, for the whole run (-1 = carless — it walks, and
    // later takes transit). Survives parking: `vehicle` goes to -1 when the
    // agent gets out, but `car` still says which vehicle is sitting in that bay
    // waiting for it. That is what lets a car persist as an object in the world,
    // be stolen, and be missed by its owner.
    int car = -1;

    // Where this agent lives and works (Living City, ADR-0066 Phase 3): the
    // PlaceMap UIDs assigned at build, or kNoPlace when the city authored none.
    // `home`/`work` NODES below are derived from these places' entrances, so the
    // existing commute machinery routes the agent to REAL buildings.
    Vec2 shopDoor;
    PlaceId homePlace = kNoPlace;
    PlaceId workPlace = kNoPlace;
    PlaceId shopPlace = kNoPlace;
    // The assigned places' DOORSTEPS (entrance nudged toward the building), set
    // by assignPlaces alongside the place ids. A resting walker stands here —
    // semantically indoors — instead of at the road-corner idle pose, which
    // could sit inside the traffic corridor and freeze every car that sensed it.
    engine::Vec2 homeDoor, workDoor;

    // Daily schedule (hours, 0..24), per-agent jittered.
    int home = 0, work = 0;
    // An errand stop near home (GoalTarget::Shop). -1 = this agent has none,
    // in which case the shopping leg fails NoRoute straight to ReturnHome.
    int shop = -1;
    // departWork is the TARGET ARRIVAL at work (Glenn, 2026-09-17: "the agent
    // would have to leave home earlier to get to work at 8"). The actual
    // departure is derived: arrival minus the trip, via departWorkHour().
    Real departWork = 8.0, departHome = 17.0;
    // What this agent's own commute COSTS, in seconds. Rate-free on purpose:
    // assignPlaces runs before the clock rate is set, and the sky changes the
    // rate at runtime, so an hour baked at assignment would be wrong twice.
    Real commuteSeconds = 0;
    Activity activity = Activity::AtHome;
    // Goal layer (ADR-0064): the agent's current state in its archetype's
    // GoalTable, plus the in-world hours spent resting in it (feeds the
    // table's optional DwellDone event). `activity` mirrors the state's label.
    int goal = 0;
    Real goalHours = 0;
    int restNode = -1;     // the node this agent last arrived at (wander departs from it)
    int arrivedLink = -1;  // the link it arrived ALONG (wander avoids U-turning back up it)
    // Fender-bender state (device: cars must not ghost through each other). When
    // two cars' bodies meet on a closing course both freeze here for a moment —
    // a visible crash-and-recover, staggered per driver so the tangle unwinds.
    // A car re-crashing over and over in place (a wedged wreck neither party can
    // steer or reverse out of) eventually ESCAPES: past a few consecutive
    // freezes it ignores car contact until it has driven clear of the spot.
    Real crashTimer = 0;
    int crashCount = 0;              // consecutive freezes at this wreck
    engine::Vec2 crashAnchor;        // where the pile-up started
    // Gridlock escape (ADR-0066 device fix): seconds this car has been pinned at
    // a junction hold (yieldAtLine with ~zero speed). Past a few seconds, box
    // occupancy stops counting occupants that are THEMSELVES stalled — a real
    // driver inches through a gridlocked box — which breaks the circular wait a
    // knot of overlapping junctions can otherwise form. Staggered per agent.
    Real holdTimer = 0;
    // A BUS AT ITS STOP: seconds it still stands there with the doors open
    // (Glenn, 2026-09-18: "they should stop and give people time to get on and
    // off"). The next trip is already routed; the bus just does not move until
    // this runs out. Both tiers honour it (advance, vAdvance).
    Real busDwell = 0;
    // PULLING OUT OF A SPACE: the car's drawn position eases from where it was
    // parked into its lane over `pullLen` metres of driving (pullS so far),
    // instead of appearing in the lane in one frame -- which read as "jump
    // back or teleport, then pivot" (Glenn, 2026-09-18; measured: 5-10 m
    // single-step jumps on departure). pullLen 0 = not pulling out.
    engine::Vec2 pullOffset{0, 0};    // parked position minus lane start
    // The turn from the lane heading back to the parked one, SIGNED and fixed
    // at departure (radians): the drawn yaw is the sim's plus this, fading.
    // Taking the shortest turn afresh each frame flipped direction whenever
    // the sim heading swung past the opposite of the parked one (a 136-degree
    // pivot mid-merge).
    Real pullYawOffset = 0;
    Real pullLen = 0;
    Real pullS = 0;

    // Think cadence (ADR-0062): agents DECIDE on a slow clock and COMMIT — the
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
    int parkedBay = -1;   // curbside bay held while resting (roads-v2.1 R6b)
    // PARKING NEAR THE DESTINATION: the bay reserved for the trip under way
    // (the route ends AT it, not at the corner), and the node the trip is
    // really for -- a car parked a street away still puts its driver at the
    // right door.
    int targetBay = -1;
    int tripGoal = -1;
    int trips = 0;   // trips started so far — the ADR-0062 bridge rebuilds its
                     // pursuit path when this changes (a new route = a new path)
    Real distOnLeg = 0, speed = 0, elevation = 0;
    // Carriageway grade (dY/m of travel) on the current link — the render
    // pitches an ELEVATED car from this, since it cannot sample the deck the
    // way it samples the ground (device: "adhere ... with all 4 wheels").
    Real grade = 0;
    // Absolute deck height (corridor decks/ramps): when set (> -1e29) the
    // render places the car at deckY directly instead of ground+elevation —
    // ground varies BETWEEN chain nodes on hilly terrain, which made deck
    // traffic hover/sink by the difference (device: "still hovering").
    Real deckY = -1e30;
    int lane = 0;
    // Fractional lane position CHASING `lane` (device: "see the cars change
    // lanes ... signal with their turn signals"): a lane change is laneF
    // easing toward the new integer lane over a couple of seconds; the render
    // reads the gap to blink the correct indicator. laneTimer paces the next
    // discretionary change.
    Real laneF = 0;
    Real laneTimer = 5.0;
    // Tether (ADR-0062): while set, this planner ghost may not LEAD `tetherAnchor`
    // (its physical car) by more than `tetherLead` metres — it waits instead. The
    // host feeds the car's real position each step, so the plan can never outrun
    // the physics (a collision, a hill, a slow start no longer strands the car).
    bool tethered = false;
    engine::Vec2 tetherAnchor;
    Real tetherLead = 10.0;
    // Continuous sideways lean off the path (ADR-0061), rate-limited, so a walker
    // steers smoothly around what it sees instead of popping. Decays back to 0.
    Real lateralOffset = 0;

    // Imperfect perception (ADR-0060): each step the agent perceives obstacles
    // with probability `reliability`; otherwise it misses (a fault). `brain` is a
    // per-agent deterministic RNG so faults reproduce from the sim seed.
    Real reliability = 1.0;
    uint32_t brain = 1;
    // This agent's own stream for TRIP decisions (lane choice, wander goal),
    // separate from `brain`'s fault stream. Per-agent so a decision depends only
    // on how many decisions THIS agent has made — the property that lets an
    // agent be advanced out of turn (waking from its schedule) without shifting
    // everybody else's choices. Seeded from `brain` by hash at build.
    uint32_t tripRng = 1;
    // Working memory (ADR-0063: sense -> REMEMBER -> predict -> decide -> act).
    // Sightings become tracks with velocity estimates; a body that leaves the
    // cone persists a few seconds, extrapolated along where it was heading
    // (object permanence), so the agent acts on a continuous world instead of a
    // per-tick snapshot — and can anticipate a crossing before it happens.
    engine::AgentMemory memory;
    // Personality (ADR-0062): this agent's personal pace as a fraction of the
    // nominal speed — a timid driver holds ~0.85x the limit, a pushy one ~1.15x,
    // and walkers stroll or stride. Derived from `brain`'s bits at build (no rng
    // draw, so seeded scenarios are unchanged); traffic stops moving in lockstep.
    Real speedFactor = 1.0;

    // Cached world pose (XZ + a height for bridges); what a renderer reads.
    engine::Vec2 pos;
    engine::Vec2 heading{1, 0};
    bool moving = false;
};

// The kind of body a vehicle wears (ADR-0061 Phase 4: one composable vehicle,
// varied by a Body component). Dimensions come from the shared fleet table below,
// so the sim (following distance, colliders) and the renderer (mesh, lift) agree.
enum class VehicleType : uint8_t { Sedan, Hatchback, SUV, Pickup, Van, BoxTruck, Bus };

// A body's physical dimensions (metres) + its type. `length` is the travel axis;
// car-following keeps cars a bumper apart from THIS, so a longer truck naturally
// holds (and is held at) a larger gap.
struct VehicleBody {
    Real length = 4.2, width = 1.8, height = 1.3;
    VehicleType type = VehicleType::Sedan;
};

// The BUILT-IN fleet: a fixed, deterministic set of body slots. A driver takes
// slot (vehicleIndex % size), so its body is stable and the renderer can mirror
// it.
//
// This is the FALLBACK, not the source of truth. The catalogue lives in
// vehicle_classes.lua and reaches the sim through CitySim::setFleet — one set of
// numbers per vehicle, shared by the drawn body, the follow gaps and the physics
// chassis. These values are what a build with no scripting (or a level with no
// vehicle recipes) falls back to; they are deliberately close to, but not
// authoritative for, the shipped classes.
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
    // Parked OFF-STREET -- in a garage or driveway, not on the road: not drawn,
    // not a body anything collides with. Where a car goes when there is no
    // marked bay or clear kerb near its owner's destination, rather than onto
    // a heap at the corner.
    bool offStreet = false;
};

// A body some agent might sense this step (ADR-0063): its plan position, its
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

    // Adopt a fleet catalogue (vehicle_classes.lua, via the level's vehicle
    // recipes). Call BEFORE build(): drivers take their body at build time, so a
    // later call would leave the cars already on the road at the old sizes. An
    // empty table restores the built-in one. Slots wrap exactly as the built-in
    // table's do.
    void setFleet(std::vector<VehicleBody> fleet) { fleet_ = std::move(fleet); }
    int fleetSize() const {
        return fleet_.empty() ? vehicleFleetSize()
                              : static_cast<int>(fleet_.size());
    }
    // AMBIENT traffic and kerbside parking skip TRANSIT bodies. A bus is 11.4 m
    // and belongs to a route, not to a parking bay -- the moment it entered the
    // ordinary rotation it broke 692 parking-band checks, because one car in
    // thirteen became a bus and no bay is that long. The sim and the renderer
    // both go through this, so a body and its mesh can never disagree.
    int ambientSlotFor(int i) const {
        const int n = fleetSize();
        if (n <= 0) return 0;
        int usable = 0;
        for (int s2 = 0; s2 < n; ++s2)
            if (fleetBody(s2).type != VehicleType::Bus) ++usable;
        if (usable <= 0) return ((i % n) + n) % n;   // a fleet of only buses
        int want = ((i % usable) + usable) % usable;
        for (int s2 = 0; s2 < n; ++s2) {
            if (fleetBody(s2).type == VehicleType::Bus) continue;
            if (want-- == 0) return s2;
        }
        return 0;
    }
    const VehicleBody& fleetBody(int slot) const {
        if (fleet_.empty()) return vehicleFleetBody(slot);
        const int n = static_cast<int>(fleet_.size());
        return fleet_[((slot % n) + n) % n];
    }

    // Advance by `dt` seconds; `hoursPerSecond` maps real time to the wrapping
    // in-world clock. Signals advance with it.
    void step(Real dt, Real hoursPerSecond = 0.05);

    // Median home->work travel time in SIM-SECONDS, measured over a sample in
    // assignPlaces. Multiply by the host's hoursPerSecond to get what a commute
    // costs in in-world HOURS — the number that says whether a day is livable
    // at this world's scale. 0 before assignPlaces runs.
    Real commuteSecondsMedian() const { return commuteSecondsMedian_; }
    // The same measure SPLIT BY MODE. A city can be perfectly livable for its
    // drivers and impossible for everyone on foot — a kilometre-scale walk is
    // hours of world time — and a blended median hides exactly that. 0 when no
    // agent of that kind has a routable commute.
    Real commuteSecondsDrive() const { return commuteSecondsDrive_; }
    Real commuteSecondsWalk() const { return commuteSecondsWalk_; }

    // WHERE AN AGENT'S DAY SAYS IT SHOULD BE at in-world hour `clock`.
    //
    // Pure: no rng, no shared state, and the clock is a PARAMETER rather than
    // the sim's own — so it can be asked about any hour, tested in isolation,
    // and reused to place an agent that has not been simulated at all. That is
    // what lets a city be seeded at its start hour instead of stepped up to it,
    // and (later) lets an agent sleep through hours of world time and be
    // reconstructed on demand when the player comes near.
    struct Snapshot {
        enum class Where : uint8_t { AtHome, AtWork, ToWork, ToHome };
        Where where = Where::AtHome;
        // For the travelling cases: how long the agent has been under way, in
        // sim-seconds. Zero when it is at either end.
        Real elapsedSeconds = 0;
    };
    Snapshot scheduleSnapshot(const Agent& a, Real clock) const;

    // Tell the sim how fast its clock runs, BEFORE seeding it.
    //
    // step() supplies this per call, but seedFromSchedule happens before any
    // step, and scheduleSnapshot needs the rate to turn "how long a commute
    // takes in seconds" into "how much of the day it eats". Seeding without it
    // used the header default (0.05) against a level authored at 0.004, which
    // made the travel window 12.5x too wide and launched roughly three quarters
    // of the population onto the road at load.
    void setHoursPerSecond(Real h) { hoursPerSecond_ = h; }
    Real hoursPerSecond() const { return hoursPerSecond_; }

    // Place the whole population where `clock` says it should be, and set the
    // sim clock to it. Replaces stepping the city up to its start hour: opening
    // at 22:00 is the same cost as opening at 08:00. Call after build() and
    // assignPlaces(); the host then steps normally from there.
    void seedFromSchedule(Real clock);

    // Dormancy telemetry: how many agents were put to sleep / rebuilt so far,
    // and how many are dormant right now. A healthy far field is mostly
    // dormant with a trickle of wakes, not a churn of both.
    int dormancies() const { return dormancies_; }
    int wakes() const { return wakes_; }
    int dormantAgents() const {
        int n = 0;
        for (const Agent& a : agents_) if (a.tier == Agent::Tier::D) ++n;
        return n;
    }

    // How many agents the scheduled wake skipped on the last step — the ones
    // waiting out their day at a door and costing nothing. Rises at night and
    // midday, falls through both rush hours.
    int sleepingAgents() const { return sleeping_; }
    // The hour this agent must LEAVE to arrive at departWork, at the current
    // clock rate. Wraps midnight: a long commute on a fast clock departs the
    // previous evening.
    Real departWorkHour(const Agent& a) const {
        const Real travel = a.commuteSeconds * (hoursPerSecond_ > 0 ? hoursPerSecond_ : 0.0);
        Real h = std::fmod(a.departWork - travel, 24.0);
        if (h < 0) h += 24.0;
        return h;
    }
    // PHASE TIMINGS (print-only, 2026-09-17). Where the milliseconds actually
    // go per step. Capping the sensing radius query changed nothing at 50k,
    // which killed two successive guesses about the hot path -- so measure it
    // rather than read the code and conclude. Microseconds, accumulated since
    // the last resetPhaseTimes().
    struct PhaseTimes {
        double rehash = 0, tierPass = 0, activeList = 0, goals = 0,
               sensedBuild = 0, gaps = 0, advance = 0, total = 0;
        // advance split five ways: the movement loop, the car pair check,
        // the two WHOLE-POPULATION loops, the 6-pass overlap relaxation,
        // and the tail.
        double advMove = 0, advPairs = 0, advPop = 0, advSolver = 0;
        int steps = 0;
    };
    const PhaseTimes& phaseTimes() const { return phase_; }
    void resetPhaseTimes() { phase_ = PhaseTimes{}; }

    Real timeOfDay() const { return clockHours_; }
    Real seconds() const { return simSeconds_; }   // monotonic sim clock (memory time base)
    const std::vector<Agent>& agents() const { return agents_; }
    const std::vector<SimVehicle>& vehicles() const { return vehicles_; }

    // Curbside parallel-parking bays (roads-v2.1 R6b, plan 4d phase 1):
    // marked bays mid-link on at-grade Local streets, seeded ~half full with
    // scenery cars at build; an arriving driver claims a free bay on its
    // arrival link instead of the old grass verge, and departure frees it.
    // occupant: -1 free, kBayScenery build-time filler, else the agent id.
    static constexpr int kBayScenery = -2;
    struct ParkingBay {
        engine::Vec2 pos, heading;   // bay centre + facing (along the link)
        int link = -1;               // directed link whose right curb it hugs
        engine::Real station = 0;    // metres from the link's `from` node
        // The Parking BAND this bay sits in (NavLink::parkWidth). The painted
        // stall is drawn to it, so the markings can never lap onto the kerb.
        engine::Real width = 0;
        int occupant = -1;
    };
    const std::vector<ParkingBay>& parkingBays() const { return bays_; }
    // Effective lane spacing for a link: a parked-up street loses its curb
    // strips from the DRIVABLE width (band-model semantics, sim-side). The
    // render bridge lines its lane arrows up with the same spacing.
    engine::Real laneSpacingFor(int li) const;
    const engine::NavGraph* graph() const { return nav_; }
    SignalController& signals() { return signals_; }

    // --- three-tier traffic (P4) --------------------------------------------
    // Public knobs with the approved defaults; a level/test opts in by setting
    // tieringEnabled and feeding the player position each fixed step. (The
    // level_loader JSON pass-through is a one-line hookup owned by the level
    // pipeline; the sim only exposes the fields.) Without BOTH the switch and
    // a centre, every agent stays K and behaviour is bit-identical to before.
    bool tieringEnabled = false;
    Real carPromoteRadius = 500.0;   // V->K inside this range of the player...
    Real carDemoteRadius = 620.0;    // ...K->V beyond this (hysteresis)
    Real pedPromoteRadius = 280.0;
    Real pedDemoteRadius = 350.0;
    // How often a FAR agent refreshes, in SIM SECONDS. The bucket count is
    // derived from this and the tick length (capped by vTickDivisor), so the
    // far tier keeps its ~1 Hz refresh whether the sim runs at 60 Hz or 15.
    Real vRefreshSeconds = 1.0;
    int vTickDivisor = 60;           // V bucket count: uid % divisor, one bucket
                                     // per fixed frame (~1 Hz per agent at 60 Hz)

    // DORMANCY (the tier below V). Off by default: with the switch off nothing
    // ever enters Tier::D and behaviour is bit-identical to the V/K sim.
    bool dormancyEnabled = false;
    // V->D beyond this, D->V inside `dormantResumeRadius` (hysteresis).
    //
    // THIS MUST EXCEED THE RADIUS AT WHICH PARKED CARS ARE DRAWN (the render
    // bridge's sceneryRadius, 450 m by default). A dormant agent's position is
    // reconstructed from its schedule when it wakes, so if it went dormant
    // while its car was still on screen, hours of world time could pass and the
    // wake would move it — teleporting a car the player is looking at. Keeping
    // dormancy strictly further out than anything drawn makes that
    // unobservable. The host asserts the relation at build.
    Real dormantRadius = 1200.0;
    Real dormantResumeRadius = 1000.0;

    // The tier bubble's centre (the player), fed by the host each fixed step.
    // No centre = no player = everything stays K (headless sims/tests).
    // TICK CADENCE (P8.2e). The sim owns how often it advances — the caller
    // only hands it elapsed time. `seconds` 0 = every call (historical, and
    // what every existing level/test relies on); 1/30 = tick at 30 Hz,
    // advancing by everything banked since the last tick. This used to live in
    // the render bridge, which meant a system's cadence was decided one layer
    // above the state it governs; the V tier's own bucket phase then quietly
    // disagreed with it (see docs/piedmont-p8-report.md).
    void setTickPeriod(Real seconds) { tickPeriodSeconds_ = seconds; }
    Real tickPeriod() const { return tickPeriodSeconds_; }
    // Sim seconds banked since the last tick — what a renderer extrapolates
    // poses by so traffic stays smooth at rates below the frame rate.
    Real secondsSinceTick() const { return sinceTick_; }
    // The sim's time of day. Two runs can only be compared at the SAME sim
    // hour: the workload (who is commuting, who is parked, and therefore who
    // sits in the player's bubble) swings enormously across the day, and a
    // slower run reaches a given hour later in wall time.
    Real clockHours() const { return clockHours_; }

    void setTierCenter(engine::Vec2 c) { tierCenter_ = c; haveTierCenter_ = true; }
    void clearTierCenter() { haveTierCenter_ = false; }
    // The bubble centre the render bridge also distance-culls scenery against.
    engine::Vec2 tierCenter() const { return tierCenter_; }
    bool hasTierCenter() const { return haveTierCenter_; }

    // Lifetime tier-transition counters (test gates: conservation, engagement).
    long tierPromotions() const { return promotions_; }
    long tierDemotions() const { return demotions_; }

    // Agent indices whose grid cell falls within `radius` of `pos`: a SUPERSET
    // by cell coverage, sorted ascending — callers apply exact predicates (the
    // physics bridge's possess ring uses this instead of a full agent scan).
    // Positions are exact for V agents and at most a step's drift stale for K.
    void queryAgentsNear(engine::Vec2 pos, Real radius,
                         std::vector<int>& out) const {
        grid_.query(pos, radius, out);
    }

    // Mark an agent as host-driven (the player): the sim won't run its AI brain.
    void setPlayerControlled(int agentIndex, bool on) {
        if (agentIndex >= 0 && agentIndex < static_cast<int>(agents_.size()))
            agents_[agentIndex].playerControlled = on;
    }

    // Release an agent whose car the player commandeered (ADR-0062): the sim parks
    // its ghost and stops advancing it, so the brain no longer fights the physical
    // car the player now drives. Idempotent; -1/out-of-range is a no-op.
    void releaseDriver(int agentIndex) {
        if (agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size())) return;
        Agent& a = agents_[agentIndex];
        a.released = true;
        a.tethered = false;
        a.moving = false;
        a.speed = 0;
        a.crashTimer = 0;   // released agents skip the sim's passes — stale wreck
        a.crashCount = 0;   // state would never decay and poison a future resume
    }

    // Tether a planner ghost to its physical car (ADR-0062): the ghost holds
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

    // Send an agent to a world position (possession, ADR-0079): route from
    // where they are to the nearest node and walk/drive there — startTrip with
    // snapped endpoints, in the same public index-based idiom as the tether
    // setters. Returns false when out of range or the destination doesn't
    // route (the agent keeps its current plan).
    bool sendAgentTo(int agentIndex, engine::Vec2 dest) {
        if (!nav_ || agentIndex < 0 ||
            agentIndex >= static_cast<int>(agents_.size()))
            return false;
        Agent& a = agents_[agentIndex];
        const int from = nav_->nearestNode(a.pos);
        const int to = nav_->nearestNode(dest);
        if (from < 0 || to < 0 || from == to) return false;
        startTrip(a, from, to, /*fromRest=*/!a.moving);
        return a.route.valid();
    }

    // Sample agent `agentIndex`'s current route as a polyline of lane-centre
    // (driver) / sidewalk (pedestrian) points, ~`step` metres apart — the pursuit
    // path the ADR-0062 bridge follows from the car's real pose. Empty when the
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

    // WANDER mode (ADR-0063, the agent lab): agents ignore the daily schedule and
    // start a fresh trip to a random reachable node the moment they arrive — so a
    // one-car lab level has its car lapping the circuit continuously instead of
    // parking until the evening commute. Deterministic (draws from the sim rng).
    // Since ADR-0064 this INSTALLS the built-in wander goal tables (replacing
    // whatever tables are set — call setGoalTables after it, not before).
    void setWander(bool on);
    bool wander() const { return wander_; }

    // The goal layer as data (ADR-0064): replace the per-archetype goal tables
    // (states + transitions over the C++ action vocabulary — city_goals.h).
    // Load-time only by design: call at level load (e.g. from tables authored
    // in agents.lua); every per-tick transition then runs in C++ from the
    // table. Agents are remapped onto the new table by their activity label
    // (first state with a matching label, else the entry state); a rebuild or
    // setWander() resets back to the built-ins.
    void setGoalTables(GoalTable pedestrian, GoalTable driver);

    // CARRYING PEOPLE (city_transit.h). RideBook is the bare relation; these
    // apply the sim-side POLICY that goes with it. Boarding clears `moving`,
    // and that one flag is all it takes: the pedestrian overlap solver, the
    // sensed_ scan and advance() ALREADY gate on it, so a passenger drops out
    // of all three without any of them learning what a ride is.
    bool boardRide(int passenger, int driver);
    void alightRide(int passenger, int atNode = -1);   // atNode: where they step off
    const RideBook& rides() const { return rides_; }
    bool riding(int i) const { return rides_.driverOf(i) >= 0; }

    // HAILING (city_dispatch.h). `hail` queues a walker for a ride; a free
    // taxi picks the cheapest reachable one up in the goal pass. Exposed so a
    // host, a test or a future bus policy can drive it without reaching into
    // the sim's internals.
    bool hail(int passenger, int pickup, int dropoff);
    // Hailed, or already promised a cab: either way the agent is WAITING and
    // must not be re-dispatched on foot by the goal pass.
    bool awaitingRide(int i) const {
        // A BUS RIDER STANDING AT A STOP counts too: the same hazard as the
        // taxi, one vehicle later. Without this the goal pass would relaunch
        // their trip on foot every tick and they would walk off before the bus
        // came. (A rider still WALKING to the stop is `moving`, so the GoTo
        // branch leaves them alone anyway.)
        if (const BusTrip* bt = buses_.tripOf(i)) { if (!bt->aboard) return true; }
        return dispatch_.isWaiting(i) || dispatch_.driverFor(i) >= 0;
    }
    const Dispatch& dispatch() const { return dispatch_; }
    // Mark an agent as a cab: it runs the taxi table instead of its archetype's.
    void setTaxi(int i, bool on);
    // What fraction of the DRIVERS work as cabs, and how willing a walker is to
    // hail one. Both persist across a rebuild, like the wander and tier knobs:
    // the host owns them.
    void setTaxiFraction(Real f);
    // BUSES. `routes` loops of `stopsPerRoute` stops are derived from the nav
    // graph (city_bus.h), and `busCount` drivers are put on them. `maxWalk`
    // bounds how far a rider will walk to a stop -- and so, in practice, how
    // often a bus is worth taking at all.
    void setBuses(int routes, int stopsPerRoute, int busCount, Real maxWalk);
    // The SERVICE DAY. Outside [startHour, endHour) buses finish up, deadhead
    // to the yard and sit there. endHour <= startHour means around the clock,
    // which is the old behaviour and stays the default.
    void setBusService(Real startHour, Real endHour) {
        busServiceStart_ = startHour;
        busServiceEnd_ = endHour;
    }
    bool busesInService() const {
        if (busServiceEnd_ <= busServiceStart_) return true;   // 24 hour service
        return clockHours_ >= busServiceStart_ && clockHours_ < busServiceEnd_;
    }
    const BusNetwork& buses() const { return buses_; }
    bool isBus(int i) const;
    // 1 while a departing car is still drawn at its parking space, easing to 0
    // once it has merged into its lane (see Agent::pullOffset).
    static Real pullOutWeight(const Agent& a);
    // A bus's route and the index of the stop it is heading for; -1 if the
    // agent is not a bus.
    int busRouteOf(int i) const {
        return isBus(i) ? busRoute_[static_cast<std::size_t>(i)] : -1;
    }
    int busNextStopOf(int i) const {
        return isBus(i) ? busStop_[static_cast<std::size_t>(i)] : -1;
    }
    // Legs a bus could not route and skipped. Evidence, not decoration: the
    // stall this counts is what stopped anyone boarding.
    long busSkippedLegs() const { return busSkippedLegs_; }
    // Which gate stops a walker even ASKING for a bus.
    struct BusGate { long thinkPed = 0, thinkDrv = 0, gotoPed = 0, gotoDrv = 0,
                     departures = 0, notPed = 0, noNet = 0, isBus = 0,
                     already = 0, noTarget = 0, asked = 0; };
    const BusGate& busGate() const { return busGate_; }
    long busStopsServed() const { return busStopsServed_; }
    // Discriminates the two ways a waiting rider is never carried: the bus
    // never OFFERED (attempts 0 -> it has not come round since they arrived)
    // or it offered and RideBook refused (attempts > 0, refused > 0).
    long busBoardAttempts() const { return busBoardAttempts_; }
    long busBoardRefused() const { return busBoardRefused_; }
    // `chance` is the odds a walker facing a trip at least `minMetres` long
    // hails instead of setting off on foot. 0 disables hailing entirely.
    void setHailPolicy(Real chance, Real minMetres) {
        hailChance_ = chance;
        hailMinMetres_ = minMetres;
    }
    bool isTaxi(int i) const;
    // Keyed by ARCHETYPE (see goalsFor): pass an agent's `archetype`, not its
    // current `mode`.
    const GoalTable& goalTable(Agent::Mode archetype) const {
        return archetype == Agent::Mode::Driver ? goalDriver_ : goalPed_;
    }

    // Make agents LIVE in the city (ADR-0066 Phase 3). Assign each agent a home
    // (a Home place) and a job (a routable Shop/Office/Civic place), pin its
    // home/work commute NODES to those places' sidewalk entrances, and seed the
    // surface-level relationship table (same workplace → coworker, same home →
    // neighbor). Deterministic (draws from each agent's `brain`, not the rng, so
    // the build stream is unchanged). Call AFTER build()/setWander with the same
    // graph; a no-op when `places` has no homes. `graph` must be the built one.
    void assignPlaces(const PlaceMap& places, const engine::NavGraph& graph);
    const RelationshipTable& relationships() const { return relationships_; }

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
    // fromRest: the trip starts from a PARKED pose (not chained mid-motion) —
    // a rest departure whose origin is a junction skips past the box so the car
    // never materializes among crossing traffic.
    void startTrip(Agent& a, int origin, int goal, bool fromRest = true);
    bool startWanderTrip(Agent& a, int from, bool fromRest = true);
    // Put a car owner back in its own car before a departure (see .cpp).
    void remountOwnedCar(Agent& a);
    // Goal layer (ADR-0064). goalThink is step()'s pass 1 for one agent: run
    // the agent's current goal state — retry a GoTo departure, or (Rest) emit
    // this tick's events and take the first table row that fires.
    void goalThink(Agent& a, Real dtHours);
    int indexOf(const Agent& a) const {
        return static_cast<int>(&a - agents_.data());
    }
    enum class GoalFire { NoRow, Blocked, Fired };
    GoalFire tryGoalEvent(Agent& a, GoalEvent event);
    // Execute the agent's (GoTo) goal state: start the trip toward its target.
    // False when no trip launched (then a NoRoute row, if any, has been taken).
    bool startGoalTrip(Agent& a, int origin, bool fromRest);
    int departNode(const Agent& a) const;   // where a rest departure starts from
    // The node a GoTo state aims at, or -1 when it has no fixed one (Random
    // picks inside startWanderTrip; Fare/Drop come from the Dispatch).
    int goalNodeFor(const Agent& a, GoalTarget target) const;
    void applyTaxiFraction();
    // Selects by ARCHETYPE (what the agent is), never by `mode` (how it happens
    // to be moving). An agent that parks and walks to a door must keep running
    // the same day — if this read `mode`, getting out of the car would swap it
    // onto the pedestrian schedule mid-commute.
    GoalTable& goalsFor(Agent::Mode archetype) {
        return archetype == Agent::Mode::Driver ? goalDriver_ : goalPed_;
    }
    // A CAB runs the taxi table instead of its archetype's -- the one place a
    // per-AGENT table beats a per-archetype one, because being a taxi is a job,
    // not a species. Everything else still goes through goalsFor.
    GoalTable& tableFor(const Agent& a) {
        const int i = indexOf(a);
        if (isBus(i)) return busTable_;
        return isTaxi(i) ? taxiTable_ : goalsFor(a.archetype);
    }
    const GoalTable& tableFor(const Agent& a) const {
        const int i = indexOf(a);
        if (isBus(i)) return busTable_;
        return isTaxi(i) ? taxiTable_
                         : (a.archetype == Agent::Mode::Driver ? goalDriver_ : goalPed_);
    }
    void installGoalTables(GoalTable pedestrian, GoalTable driver);
    bool launchClear(const Agent& a, int node) const;   // no moving car near the spawn
    void seatBusAt(int idx, int node, int queued = 0);   // a bus at rest on a stop
    // The nearest FREE bay to `target` within `maxDist`, claimed for agent
    // `self` (-1 if none). The one allocator for a car at load and on arrival.
    int claimBayNear(engine::Vec2 target, int self, Real maxDist);
    void releaseBays(Agent& a);          // free both the held and the reserved bay
    std::vector<int> nearestFreeBays(engine::Vec2 target, Real maxDist, int k) const;
    bool parksInBays(const Agent& a) const;   // a private car that parks (not a bus/cab/wanderer)
    void advance(Agent& a, Real dt, Real gap, Real minGap);
    // advance()'s junction verdict: the speed target after the signal brake and
    // the box-occupancy / turn-yield scan, plus the effective stop line (distance
    // along the current leg) the hard clamp in advance() holds at.
    struct JunctionGate {
        Real cap = 0;               // speed target after junction/signal/yield caps
        Real stopLinePos = 0;       // effective stop line on the current leg
        bool yieldAtLine = false;   // a TURNING car holding for box/oncoming traffic
    };
    JunctionGate junctionSpeedCap(const Agent& a, int li, Real target) const;
    Real senseAhead(Agent& a);   // perception/memory/TTC: distance to a body ahead
    void arriveOrChain(Agent& a, Real vArrive);   // arrival: chain, park, or rest
    void labelDriverState(Agent& a, Real seenAhead, Real gap, int legCount) const;
    void computeGaps();
    void computeCarWedge();   // fills carAheadGap_/carAheadSpeed_ (S7 senses)
    // --- three-tier traffic (P4) ---
    // Promote/demote against the bubble (fixed update, agent-index order:
    // demotions swept first, then promotions from a grid query — deterministic).
    void tierPass(Real hoursPerSecond);
    // Put one agent where its schedule says it is; shared by load-time seeding
    // and by waking a dormant agent, so both place by identical rules.
    void placeFromSchedule(int idx);
    void wakeDormant(int idx);        // D -> V, rebuilt from the schedule
    // One coarse V tick for agent `i`: goal layer at accumulated hours, then
    // vAdvance over the accumulated seconds; re-hashes the agent in the grid.
    // Promotion runs this as its catch-up so the K handoff pose is exact.
    // The actual advance. step() is the cadence gate in front of it.
    void stepTick(Real dt, Real hoursPerSecond);
    void tickV(int i, Real hoursPerSecond);
    // Advance a V agent along its REAL route at the modelled speed: link class
    // speed shaped by fixed average junction delays (vHold). No sensing, no
    // FSM, no live signal state — hasSignal() is static topology, not a query.
    void vAdvance(Agent& a, Real dt);

public:
    // Car-contact events since build — the roads-v2 S7 soak gate reads these
    // and ratchets them down slice by slice. `fastCrashEvents` is the class
    // the plan's "no pile-ups" gate is about: both bodies above walking pace
    // at contact. The remainder are slow junction-mouth brushes — kinematic
    // lane ribbons overlapping at low speed, arbitrated by the fender-bender
    // freeze; their fix is junction path geometry, not more braking rules.
    int crashEvents() const { return crashEvents_; }
    int fastCrashEvents() const { return fastCrashEvents_; }

private:
    // Per-node junction box radius: the widest incident half-width. Nonzero at
    // PLAIN nodes too (their road's half-width) — launchClear reads it at
    // arbitrary departure nodes; 0 only where a node has no out-links.
    Real junctionRadius(int node) const;
    Real vehicleLength(int agentIndex) const;      // body length, or a ped's footprint
    Real pairMinGap(int follower, int leader) const;   // bumper-to-bumper follow gap
    void measureCommute(const engine::NavGraph& graph);   // median commute (see .cpp)
    Real brainUnit(Agent& a);   // per-agent deterministic roll for faults
    uint32_t tripRnd(Agent& a); // per-agent stream for TRIP decisions (see .cpp)
    void refreshPose(Agent& a);
    void steer(Agent& a, Real dt);   // rate-limited heading (bounded turn radius)

    engine::Vec2 idlePose(int node, Agent::Mode mode, uint32_t brain) const;
    // Push a parked/idle car pose OUT of every at-grade carriageway. A knot
    // of short links can put one link's verge INSIDE a neighbour's lanes; a
    // car resting there dams the junction until its driver departs (three
    // drivers pinned for minutes behind one verge-parked car — surfaced by
    // the density round's street-fronting places).
    engine::Vec2 pushPoseClearOfLanes(engine::Vec2 p, engine::Real margin) const;
    uint32_t rnd();
    Real rndUnit();

    const engine::NavGraph* nav_ = nullptr;
    std::vector<Agent> agents_;
    std::vector<SimVehicle> vehicles_;
    // Adopted fleet catalogue; empty = use the built-in table (see setFleet).
    std::vector<VehicleBody> fleet_;
    std::vector<ParkingBay> bays_;
    // Each link's reverse twin (-1 if one-way), and a per-link route cost
    // scale kept at 1 except while a bay departure prices its own twin.
    std::vector<int> twinOf_;
    std::vector<Real> departScale_;
    std::vector<std::vector<int>> baysOnLink_;   // link -> bay indices
    std::vector<char> bayNarrowed_;   // link (or its reverse) carries bays
    std::vector<Real> gaps_;
    std::vector<Real> minGaps_;   // per-agent follow gap to ITS leader (length-aware)
    std::vector<Real> leaderSpeeds_;   // leader's speed where gaps_ < INF (IDM dv)
    // Car-vs-car vision wedge (roads-v2 S7 slice 3): per car, the nearest CAR
    // body in its forward corridor — the sense the lane-keyed gap logic
    // structurally lacks (merge convergence, wrecks on another link, bodies
    // inside the junction box). Net bumper gap + that body's along-my-heading
    // speed; INF/0 when clear.
    std::vector<Real> carAheadGap_;
    std::vector<Real> carAheadSpeed_;
    int crashEvents_ = 0;              // total fender-bender contacts (soak gate)
    int fastCrashEvents_ = 0;          // contacts with both bodies > 2 m/s
    std::vector<SensedGhost> sensed_;   // per-step snapshot of bodies cars/peds may SEE
                                        // (peds + players + external obstacles)
    std::vector<int> sensedIndex_;      // per agent: its slot in sensed_, -1 = absent
                                        // (lets grid candidates map back to ghosts)
    std::vector<engine::Vec2> externalObstacles_;   // host-injected (the live player)
    std::vector<engine::Vec2> staticObstacles_;     // host-injected, static (signal poles)
    std::vector<std::pair<engine::Vec2, Real>> junctions_;   // centre + box radius
    std::vector<Real> nodeBoxRadius_;   // per node: widest incident half-width
    // P4.1 spatial index. grid_ holds every agent (K re-hashed each step —
    // effectively free, place() early-outs on an unchanged cell; V only on its
    // coarse tick, when its position actually moves). junctionGrid_ is a static
    // bake of junctions_ so nearJunction() stops scanning the whole list.
    AgentGrid grid_;
    // Spatial index of PARKED cars, keyed by vehicle index. A driven car is
    // found through its driver in `grid_`; a parked one has no driver and sits
    // far from its owner, so it needs its own. Entries for driven cars go stale
    // harmlessly — every consumer filters on `driver < 0`.
    AgentGrid parkedGrid_;
    AgentGrid junctionGrid_;
    Real maxJunctionRadius_ = 0;
    mutable std::vector<int> parkedScratch_;  // parkedGrid_ candidates (kept
                                              // separate so a vehicle query can
                                              // nest inside an agent query)
    mutable std::vector<int> queryScratch_;   // shared candidate buffer (queries
    std::vector<int> pairScratch_;   // car-vs-car pair query (stepTick)
    RideBook rides_;                 // who is riding with whom (city_transit.h)
    Dispatch dispatch_;              // who wants a ride (city_dispatch.h)
    std::vector<uint8_t> taxi_;      // per-agent: runs the taxi table
    GoalTable taxiTable_;
    BusNetwork buses_;
    std::vector<int> busRoute_, busStop_;   // per agent; -1 = not a bus
    GoalTable busTable_;
    Real busMaxWalk_ = 0;
    Real busServiceStart_ = 0, busServiceEnd_ = 0;   // 0/0 = around the clock
    bool busServiceWas_ = true;                      // edge detect for the events
    long busSkippedLegs_ = 0;
    mutable BusGate busGate_;
    long busStopsServed_ = 0;
    long busBoardAttempts_ = 0, busBoardRefused_ = 0;
    Real taxiFraction_ = 0;
    Real hailChance_ = 0;
    Real hailMinMetres_ = 400;
                                              // never nest across a live iteration)
    std::vector<int> tierScratch_;            // tierPass promotion candidates
    std::vector<int> dormantScratch_;         // tierPass wake candidates
    int dormancies_ = 0;
    int wakes_ = 0;
    SignalController signals_;
    // Per-archetype goal tables (ADR-0064): what each agent's day IS. Built-in
    // defaults mirror the historical schedule/wander control flow bit-exactly;
    // scripting builds may replace them at load via setGoalTables.
    GoalTable goalPed_ = defaultScheduleGoals();
    GoalTable goalDriver_ = defaultScheduleGoals();
    RelationshipTable relationships_;   // surface-level social graph (ADR-0066)
    long faultCount_ = 0;
    Real commuteSecondsMedian_ = 0;
    Real commuteSecondsDrive_ = 0;
    Real commuteSecondsWalk_ = 0;
    // Last step's clock rate, so a sleeping agent can convert "in-world hours
    // until my next event" into a sim-second wake time.
    Real hoursPerSecond_ = 0.05;
    int sleeping_ = 0;   // agents skipped this step by the scheduled wake
    PhaseTimes phase_;   // print-only instrumentation
    Real clockHours_ = 6.0;
    // The clock UNWRAPPED (never reset): the time base for dwell accounting
    // across sleeps, immune to rate changes and holds. heldSeconds_ counts
    // sim-seconds spent at rate 0 so sleepers' wake times can be shifted
    // past a hold on release (rerateSleep).
    Real clockTotalHours_ = 0;
    Real heldSeconds_ = 0;
    Real lastLiveRate_ = 0;   // the rate sleepers' wake times were last valid at
    void rerateSleep(Real newRate);
    Real simSeconds_ = 0;   // seconds since build — the time base memory decays on
    Real thinkPeriod_ = 0.35;   // reactive re-decide cadence (s), staggered per agent
    bool wander_ = false;       // lab mode: perpetual random trips, no schedule
    // Three-tier traffic state (P4): the fed bubble centre, the fixed-frame
    // counter the V buckets key off (frameIndex_ % vTickDivisor — no wall
    // clock), and the lifetime transition counters the tests gate on.
    // Cadence state (see setTickPeriod): time banked toward the next tick, and
    // time elapsed since the last one.
    Real tickPeriodSeconds_ = 0.0;
    Real tickAccum_ = 0.0;
    Real sinceTick_ = 0.0;
    engine::Vec2 tierCenter_;
    // Indices of the agents that need this step's full passes (everything but
    // the far/V tier), resolved once per step in step(). Ascending, so the
    // passes keep their original iteration order.
    std::vector<int> active_;
    bool haveTierCenter_ = false;
    uint64_t frameIndex_ = 0;
    long promotions_ = 0;
    long demotions_ = 0;
    uint32_t rng_ = 1;
    // Hands out agent UIDs (ADR-0066): reset at build, then one per agent in
    // order. Separate from rng_ so identity allocation never perturbs the sim's
    // deterministic draw stream.
    AgentIdAllocator ids_;
};

}  // namespace citysim

#endif
