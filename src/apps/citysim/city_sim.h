#ifndef RAYTRACER_APPS_CITYSIM_CITY_SIM_H
#define RAYTRACER_APPS_CITYSIM_CITY_SIM_H

#include <algorithm>
#include "../../engine/ai/agent_memory.h"
#include "../../engine/ai/nav_graph.h"
#include "../../engine/ai/pathfind.h"
#include "agent_grid.h"
#include "agent_id.h"
#include "activities.h"
#include "city_goals.h"
#include "city_roles.h"   // every resident's day comes from its role (RoleCatalog, buildDayTable)
#include "city_bus.h"
#include "city_dispatch.h"
#include "city_transit.h"
#include "places.h"
#include "relationships.h"
#include "traffic_signal.h"
#include <cstdint>
#include <utility>
#include <queue>
#include <unordered_map>
#include <vector>

namespace citysim {

using engine::Real;   // the engine's scalar (double); used throughout the sim

// kLayerClearance: nav_graph.h (the graph resolves a link's height above the ground with it).
using engine::kLayerClearance;

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
    // A Student (campus milestone 4) lives in the university's residence hall and
    // spends the day between classes in the teaching hall and breaks on campus.
    enum class Role : uint8_t { Commuter, Shopkeeper, Stroller, Student, Count };
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
    // CONTINUITY (the 100k plan; Glenn: "I'd definitely want it so I could follow someone around the city ... I
    // don't want it so that if you turn around and turn back that person disappears"):
    //   rememberUntil -- sim seconds: someone near you lately stays FAR (its exact trip) instead of going dormant
    //                    (rebuilt from its schedule), so it is where it should be when you come back;
    //   pinned        -- followed: simulated in full however far it goes.
    Real rememberUntil = 0;
    bool pinned = false;
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
    bool playerControlled = false;
    // THE PHYSICAL TIER'S BODY, as the sim's followers must see it (#23): metres the possessed car's real
    // chassis trails this ghost along its heading (0 when not possessed, or not behind). Car-following and
    // the vision wedge measure a leader to its BODY -- gaps planned to the ghost put the car behind's
    // kinematic box into a body still a length back, and an immovable box ramming a chassis is the climb.
    Real bodyLag = 0;
    // DIRECTED (ADR-0091): a director owns this agent's plan — the possession
    // channel now, a model later. The goal layer leaves it entirely alone (no
    // schedule, no chained trip on arrival) while the stepper keeps moving it
    // along whatever route the director gave. This is NOT `released`, which
    // stops the agent being stepped at all, nor `playerControlled`, which hands
    // the body to the host: a directed agent is still the sim's to move.
    // WHERE ON THE LAST LEG THE ERRAND ACTUALLY ENDS, in metres along it, or
    // -1 for "at the node" (every ordinary trip). A route is a list of LINKS,
    // so arrival used to mean reaching the end of the last one — which on a
    // 40 m link leaves an agent 20 m from the door, or the car, it was sent to.
    Real stopAtDist = -1;
    bool directed = false;   // brain = host input; the sim won't auto-drive it
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
    float departWork = 8.0f, departHome = 17.0f;
    // What this agent's own commute COSTS, in seconds. Rate-free on purpose:
    // assignPlaces runs before the clock rate is set, and the sky changes the
    // rate at runtime, so an hour baked at assignment would be wrong twice.
    float commuteSeconds = 0;
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
    // INDOORS (Glenn, 2026-09-19: "agents who are at work or home should ...
    // go inside the building"). Resting at a door -- home, work, a shop, a
    // cafe -- means inside: not drawn, not a body on the pavement. Cleared the
    // moment a trip starts. Resting OUTSIDE (a bus stop, a park, a pause on a
    // walk) keeps this false and the walker stands where it is.
    bool indoors = false;
    // This rest's own length in in-world hours, when the stop sets it (a
    // coffee outlasts a look in a shop window); 0 = the goal state's dwell.
    float restDwell = 0;
    // The venue (CitySim::venues_) this trip is heading for, -1 for none.
    int tripVenue = -1;
    // A SEAT this trip is for (CitySim::seats_, reserved by this agent), -1 for none; and where the agent is in
    // using it: 0 not, 1 walking from the path to it, 2 sitting, 3 walking back to `seatBack` (the path).
    int tripSeat = -1;
    uint8_t seatPhase = 0;
    engine::Vec2 seatBack{0, 0};
    float loopS = 0;   // running a loop (a Jog spot): how far round it (m)
    int tripActivity = -1;   // the catalog activity this trip is for (CitySim::catalog_), -1 none
    int atActivity = -1;     // ...and the one it arrived for, while it stays (telemetry: `activities?`)
    int session = -1;        // the group activity session it is in (CitySim::sessions_), -1 none
    int sessionRole = -1;    // its role in it (the definition's roles)
    engine::Vec2 roamTarget{0, 0};   // where it is running to on the area
    // Where the current outing/lunch trip is going, kept until arrival: a leg
    // by BUS sets the walker down at a stop and the trip resumes from there,
    // and it must resume to the same place, not re-pick one.
    int outingTo = -1;
    // A bus that pulled up short of its stop node (busStandBack): the leg it
    // stands on, so its next trip starts from there. -1 otherwise.
    int busStoodLeg = -1;
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
    float busDwell = 0;
    // PULLING OUT OF A SPACE: the car's drawn position eases from where it was
    // parked into its lane over `pullLen` metres of driving (pullS so far),
    // instead of appearing in the lane in one frame -- which read as "jump
    // back or teleport, then pivot" (Glenn, 2026-09-18; measured: 5-10 m
    // single-step jumps on departure). pullLen 0 = not pulling out.
    // Where the agent stood when the current sim tick began: its body is DRAWN
    // interpolated from here to `pos` across the tick (one tick behind the
    // sim), not extrapolated past it. Reset wherever the sim PLACES an agent
    // rather than moving it, so a placement is never drawn as a slide.
    engine::Vec2 tickFromPos{0, 0};
    engine::Vec2 tickFromHeading{0, 1};
    float tickFromPullS = 0;   // pullS at the tick's start (the easing interpolates too)
    engine::Vec2 pullOffset{0, 0};    // parked position minus lane start
    // The turn from the lane heading back to the parked one, SIGNED and fixed
    // at departure (radians): the drawn yaw is the sim's plus this, fading.
    // Taking the shortest turn afresh each frame flipped direction whenever
    // the sim heading swung past the opposite of the parked one (a 136-degree
    // pivot mid-merge).
    float pullYawOffset = 0;
    float pullLen = 0;
    float pullS = 0;

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
    float grade = 0;
    // Absolute deck height (corridor decks/ramps): when set (> -1e29) the
    // render places the car at deckY directly instead of ground+elevation —
    // ground varies BETWEEN chain nodes on hilly terrain, which made deck
    // traffic hover/sink by the difference (device: "still hovering").
    float deckY = -1e30;
    int lane = 0;
    // Fractional lane position CHASING `lane` (device: "see the cars change
    // lanes ... signal with their turn signals"): a lane change is laneF
    // easing toward the new integer lane over a couple of seconds; the render
    // reads the gap to blink the correct indicator. laneTimer paces the next
    // discretionary change.
    float laneF = 0;
    float laneVel = 0;   // d(laneF)/dt, lanes/s: the glide is a damped spring, so it eases in and out (S-curve)
    float laneTimer = 5.0;
    // Tether (ADR-0062): while set, this planner ghost may not LEAD `tetherAnchor`
    // (its physical car) by more than `tetherLead` metres — it waits instead. The
    // host feeds the car's real position each step, so the plan can never outrun
    // the physics (a collision, a hill, a slow start no longer strands the car).
    bool tethered = false;
    engine::Vec2 tetherAnchor;
    float tetherLead = 10.0;
    // Continuous sideways lean off the path (ADR-0061), rate-limited, so a walker
    // steers smoothly around what it sees instead of popping. Decays back to 0.
    Real lateralOffset = 0;
    // Where the CROWD has put it off its path (the overlap floor's push, KEPT): re-applied each tick, so a packed
    // doorway is spaced out once and stays spaced, rather than snapped back onto the path line and shoved apart
    // afresh every tick (1.5-3 m hops at 13 m/s, the speed watchdog's "avoiding" walkers). Eases back to the path.
    engine::Vec2 crowdOffset{0, 0};

    // Imperfect perception (ADR-0060): each step the agent perceives obstacles
    // with probability `reliability`; otherwise it misses (a fault). `brain` is a
    // per-agent deterministic RNG so faults reproduce from the sim seed.
    float reliability = 1.0;
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
    float speedFactor = 1.0;

    // Cached world pose (XZ + a height for bridges); what a renderer reads.
    engine::Vec2 pos;
    engine::Vec2 heading{1, 0};
    // A driver's PATH direction at `pos` (refreshPose): the tangent of the
    // corner curve through a junction turn, else its link's direction. steer()
    // turns the body toward this, so the nose follows the arc it is driving.
    engine::Vec2 pathDir{0, 0};
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
// How far off its final street a director's destination may sit and still be
// walked to exactly. Beyond this it is not really "along that street" and the
// agent arrives at the node instead.
constexpr Real kErrandOffLink = 15.0;

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
    // A test lab can put the bus in the ambient rotation (citysim.ambientBus) to drive it over every
    // road with the rest -- a wandering driver never parks, so the bay problem above does not arise.
    bool ambientBus = false;
    int ambientSlotFor(int i) const {
        const int n = fleetSize();
        if (n <= 0) return 0;
        if (ambientBus) return ((i % n) + n) % n;
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
        // ...and the night out (eveningPlan): on the way to tonight's place, there, on the way home from it
        enum class Where : uint8_t { AtHome, AtWork, ToWork, ToHome, ToEvening, AtEvening, FromEvening };
        Where where = Where::AtHome;
        int venue = -1;   // the evening's: an index into venues_ (eveningVenue)
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
    // A NIGHT OUT (Glenn, 2026-10-06: "we would want restaurants and clubs and such for a night life"): whether
    // tonight -- noon to noon -- is one of this agent's nights out, and its evening if so: out between 18:30 and 21:00,
    // home two to five and a half hours later (by 02:30 at the latest). Its own bits and the night's number: a
    // different crowd every night, the same one for the same night. The share is the role's (ResidentRole::nightOutShare).
    bool eveningPlan(const Agent& a, Real& start, Real& end) const;
    // WHERE TONIGHT, for an agent nobody is simulating (a sleeper, a city seeded at 21:00): one of the six nearest
    // restaurants, bars or clubs to home, in its own town, within 1.2 km and open when its evening starts -- which
    // one, from its own bits and the night's number (the same place all evening). -1: nowhere (a night in).
    int eveningVenue(const Agent& a, Real eveningStart) const;
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
    // Every K agent (the simulated-in-full, drawn tier), ascending: what a drawer walks instead of agents()
    const std::vector<int>& nearAgents() const { return kIdx_; }
    // THE GPU CROWD'S FEED: the agents whose far trip may have changed since the last take (a tier change, a far
    // tick, a wake) -- a drawer re-reads just those. `all` set: re-read everyone (the list overflowed, untaken).
    void takeCrowdChanges(std::vector<int>& out, bool& all) {
        out.swap(crowdDirty_);
        crowdDirty_.clear();
        all = crowdDirtyAll_;
        crowdDirtyAll_ = false;
    }
    // Every V agent (simulated coarsely, not drawn in full), ascending: the drawers' mid-range band
    const std::vector<int>& farAgents() const { return vIdx_; }
    // A far agent's drawn pose: its last coarse pose run on along its heading at its speed for the time since that
    // tick (capped, and never past its link's end), so a far crowd moves rather than hopping once a second. False
    // for an agent with nothing to show: not far, indoors at rest, riding, released.
    bool farDrawPose(int agentIndex, engine::Vec2& pos, engine::Vec2& heading) const;
    // Vehicles hashed near (c, r), ascending -- a superset: every car is placed where it last parked (a driven one's
    // entry is stale; the caller skips cars with a driver)
    void parkedNear(engine::Vec2 c, Real r, std::vector<int>& out) const { parkedGrid_.query(c, r, out); }
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
    // THE NEAR TARGET (the 100k plan, pushing past it): with nearTarget > 0 the K ring's radii are scaled (35%..100%)
    // to hold about that many agents simulated in full round the player -- denser cities, a smaller ring; the GPU
    // crowd shows everyone past it. 0: the radii as set.
    int nearTarget = 0;
    Real nearScale() const { return nearScale_; }
    // ...and the FAR ring (V): with farTarget > 0 the dormant / resume radii scale (20%..100%) to hold about that many
    // agents in the coarse tier -- at 5x density a 1.2 km circle held 105,000 of them, ticked once a second. The GPU
    // crowd draws dormant travellers moving, so a smaller far ring is not an emptier view.
    int farTarget = 0;
    Real farScale() const { return farScale_; }
    long resumedTrips() const { return resumedTrips_; }
    long dormantEvents() const { return dormantEventsRun_; }   // schedule events run for sleeping agents
    double dormantEventMs() const { return dormantEventMs_; }   // ...and the wall time they took, in all
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
    // The drawn junction MOUTH sits at the widest arm's half-width PLUS the
    // sidewalk band (the road mesher trims every street body there and paints
    // the zebra just past it). Set before build(); 0 keeps the carriageway-only
    // box (headless sims with no drawn roads).
    void setJunctionPad(Real metres) { junctionPad_ = metres < 0 ? 0 : metres; }
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

    // TAKE THE BUS (ADR-0091). The same choice the goal layer makes at every
    // trip, offered to a director: plan a ride from where the agent stands to
    // `dest`, register it as waiting, and walk it to the boarding stop —
    // walking to the stop IS the trip. From there the sim does the rest, and
    // it already knows how: arriveOrChain stands a rider at its stop, the bus
    // picks up everyone waiting for its route, and sets them down at theirs.
    // False when no route helps, which for a short hop is the common answer
    // and means "just walk".
    bool sendAgentByBus(int agentIndex, engine::Vec2 dest) {
        if (!nav_ || agentIndex < 0 ||
            agentIndex >= static_cast<int>(agents_.size()))
            return false;
        Agent& a = agents_[static_cast<std::size_t>(agentIndex)];
        if (a.mode != Agent::Mode::Pedestrian || buses_.empty() ||
            isBus(agentIndex) || isTaxi(agentIndex) || buses_.tripOf(agentIndex))
            return false;
        const int origin = nav_->nearestNode(a.pos);
        const int to = nav_->nearestNode(dest);
        if (origin < 0 || to < 0 || origin == to) return false;
        BusTrip bt = buses_.planTrip(nav_->nodes[static_cast<std::size_t>(origin)],
                                     nav_->nodes[static_cast<std::size_t>(to)],
                                     busMaxWalk_);
        if (!bt.valid()) return false;
        const int stopNode =
            buses_.route(bt.route).stops[static_cast<std::size_t>(bt.fromStop)].node;
        // PROVE THE WALK BEFORE TAKING IT. startTrip's no-path branch PARKS the
        // agent — clears its route and moves it to an idle pose at the origin —
        // so calling it speculatively means every REFUSED ride still shunts the
        // agent about. Fourteen refusals in half a minute is a person flickering
        // around the street, which is exactly what Glenn watched happen. Same
        // rule as sendAgentTo: nothing is mutated until the answer is yes.
        if (stopNode != origin &&
            !engine::findRoute(*nav_, origin, stopNode, /*pedestrian=*/true).valid())
            return false;
        if (!buses_.waitFor(agentIndex, bt)) return false;
        a.wakeAt = -1;
        if (stopNode != origin) {
            startTrip(a, origin, stopNode, /*fromRest=*/!a.moving);
            if (!a.moving) { buses_.stopWaiting(agentIndex); return false; }
            a.state = Agent::State::Walking;
        }
        a.activity = Activity::Outing;
        return true;
    }

    // GO INSIDE (ADR-0091). A place is somewhere people are hidden from the
    // street while they are in it — pedVisible() is false for a still agent
    // that is indoors, so its body is reaped and it is simply "in there" until
    // it comes out. The director decides it has arrived; the engine has no
    // opinion about doors.
    void setAgentIndoors(int agentIndex, bool inside) {
        if (agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size()))
            return;
        Agent& a = agents_[static_cast<std::size_t>(agentIndex)];
        if (a.mode != Agent::Mode::Pedestrian) return;   // not while driving
        a.indoors = inside;
        if (inside) {
            a.moving = false;
            a.speed = 0;
            a.route = engine::Route{};
            a.leg = 0;
            a.distOnLeg = 0;
            a.state = Agent::State::Resting;
            a.tethered = false;
        }
    }

    // GET IN AND DRIVE (ADR-0091). A possessed pedestrian walks up to a car
    // and takes it. Everything else follows from `mode`, because every pass
    // already reads it: pedVisible() goes false so the walker system reaps its
    // body, the vehicle bridge draws the car instead, findRoute switches from
    // the pavement to the road graph, and the stepper advances it as traffic.
    // The agent's pose BECOMES the car's — you are where the car is once you
    // are in it — so the planner ghost and the thing on screen stay one object.
    // Refuses a car that already has a driver. The caller owns the question of
    // how far away is too far to reach.
    bool boardVehicle(int agentIndex, int vehicleIndex) {
        if (agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size()))
            return false;
        if (vehicleIndex < 0 || vehicleIndex >= static_cast<int>(vehicles_.size()))
            return false;
        Agent& a = agents_[static_cast<std::size_t>(agentIndex)];
        SimVehicle& v = vehicles_[static_cast<std::size_t>(vehicleIndex)];
        if (v.driver >= 0 || a.vehicle >= 0) return false;
        a.pos = v.pos;
        a.heading = v.heading;
        a.mode = Agent::Mode::Driver;
        a.vehicle = vehicleIndex;
        v.driver = agentIndex;
        v.offStreet = false;              // on the street the moment it is driven
        a.indoors = false;
        a.tethered = false;               // the walker body it had is gone
        a.moving = false;
        a.speed = 0;
        a.route = engine::Route{};
        a.leg = 0;
        a.distOnLeg = 0;
        a.state = Agent::State::Waiting;
        a.tickFromPos = a.pos;            // a placement, not a motion
        a.tickFromHeading = a.heading;
        grid_.place(agentIndex, a.pos);
        return true;
    }

    // Get out again, on the kerb beside the car, leaving it where it stands.
    bool alightVehicle(int agentIndex) {
        if (agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size()))
            return false;
        Agent& a = agents_[static_cast<std::size_t>(agentIndex)];
        if (a.vehicle < 0 || a.vehicle >= static_cast<int>(vehicles_.size()))
            return false;
        SimVehicle& v = vehicles_[static_cast<std::size_t>(a.vehicle)];
        v.pos = a.pos;
        v.heading = a.heading;
        v.driver = -1;
        parkedGrid_.place(a.vehicle, v.pos);
        a.vehicle = -1;
        a.mode = Agent::Mode::Pedestrian;
        a.moving = false;
        a.speed = 0;
        a.route = engine::Route{};
        a.leg = 0;
        a.distOnLeg = 0;
        a.state = Agent::State::Resting;
        // Step out onto the pavement rather than into the lane you parked in.
        const engine::Vec2 side(a.heading.y, -a.heading.x);
        a.pos = pushPoseClearOfLanes(engine::Vec2(a.pos.x + side.x * 2.0,
                                                  a.pos.y + side.y * 2.0), 1.0);
        a.tickFromPos = a.pos;
        grid_.place(agentIndex, a.pos);
        return true;
    }

    // The nearest car with nobody in it within `radius` of `p` — what a
    // director means by "that one". Skips off-street cars (in a garage, not
    // drawn, not reachable on foot). -1 when there is none.
    int nearestFreeVehicle(engine::Vec2 p, Real radius) const {
        int best = -1;
        Real bestD2 = radius * radius;
        for (std::size_t i = 0; i < vehicles_.size(); ++i) {
            const SimVehicle& v = vehicles_[i];
            if (v.driver >= 0 || v.offStreet) continue;
            const Real dx = v.pos.x - p.x, dy = v.pos.y - p.y;
            const Real d2 = dx * dx + dy * dy;
            if (d2 <= bestD2) { bestD2 = d2; best = static_cast<int>(i); }
        }
        return best;
    }

    // Hand an agent's PLAN to a director (ADR-0091). Idempotent; -1 or an
    // out-of-range index is a no-op. Turning it off returns the agent to its
    // schedule from wherever it is standing.
    void setAgentDirected(int agentIndex, bool on) {
        if (agentIndex >= 0 && agentIndex < static_cast<int>(agents_.size()))
            agents_[static_cast<std::size_t>(agentIndex)].directed = on;
    }
    bool agentDirected(int agentIndex) const {
        return agentIndex >= 0 && agentIndex < static_cast<int>(agents_.size()) &&
               agents_[static_cast<std::size_t>(agentIndex)].directed;
    }

    // The physical tier reports how far a possessed car's body trails its ghost (Agent::bodyLag).
    void setAgentBodyLag(int agentIndex, Real metres) {
        if (agentIndex >= 0 && agentIndex < static_cast<int>(agents_.size()))
            agents_[static_cast<std::size_t>(agentIndex)].bodyLag = metres;
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
    // Follow an agent: keep it in the full sim wherever it goes (and back off when unpinned).
    void pinAgent(int agentIndex, bool on);
    int pinnedCount() const { return static_cast<int>(pinned_.size()); }
    Real rememberSeconds = 600.0;   // how long someone near you stays exact once you have gone (0: off)
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
        Agent& a = agents_[static_cast<std::size_t>(agentIndex)];
        // WHERE THE ERRAND STARTS. startTrip PLACES an agent on its new route's
        // first leg, so routing from the node nearest its position teleports it
        // — and a drawn walker's planner ghost is leashed to its physical body
        // (ADR-0062, 5 m), so the stepper then skips it for ever: speed 0,
        // state Waiting, a perfectly valid route, and a possessed pedestrian
        // that never moves again. Measured: a 40 m errand moved the ghost 290 m.
        // An agent already under way therefore finishes the leg it stands on and
        // routes from the node AHEAD of it, and the leg is prepended below.
        const int legLink =
            (a.moving && a.route.valid() &&
             a.leg < static_cast<int>(a.route.links.size()))
                ? a.route.links[static_cast<std::size_t>(a.leg)] : -1;
        const int from = legLink >= 0
                             ? nav_->links[static_cast<std::size_t>(legLink)].to
                             : nav_->nearestNode(a.pos);
        if (from < 0) return false;
        // A DESTINATION THAT ROUTES. The single nearest node to `dest` can have
        // no path from here at all — 394 -> 86 in metro are 32 m apart with no
        // route for a pedestrian OR a car (different components of the graph),
        // and every walk_to that snapped to such a node was refused. Try the
        // nearest few and take the first that routes.
        engine::Route route;
        int to = -1;
        int finalLink = -1;
        // ARRIVE ALONG THE STREET THE DESTINATION IS ON. A door, a parked car
        // and a bus stop all belong to a LINK, not to a junction, so routing to
        // the nearest node leaves the agent wherever that junction happens to
        // be — measured at 23 m from a civic entrance it had been sent to.
        // Routing to the near end of the destination's own link and appending
        // that link makes the last street it walks the one it was sent to, and
        // the stop-short below then lands it beside the address.
        if (a.mode == Agent::Mode::Pedestrian) {
            const int dl = nav_->nearestLink(dest);
            if (dl >= 0) {
                const engine::NavLink& L = nav_->links[static_cast<std::size_t>(dl)];
                if (L.from != from) {
                    engine::Route r = engine::findRoute(*nav_, from, L.from, true);
                    if (r.valid()) { route = std::move(r); to = L.from; finalLink = dl; }
                } else if (L.to != from) {
                    engine::Route r = engine::findRoute(*nav_, from, L.to, true);
                    if (r.valid()) { route = std::move(r); to = L.to; }
                }
            }
        }
        if (to < 0) for (int cand : nearestNodesTo(dest, 8)) {
            if (cand == from) continue;
            engine::Route r = engine::findRoute(*nav_, from, cand,
                                                a.mode == Agent::Mode::Pedestrian);
            if (r.valid()) { route = std::move(r); to = cand; break; }
        }
        // NOTHING has been mutated yet, so a refusal really does leave the agent
        // its day. startTrip's no-path branch parks the agent, clears its route
        // and keeps the caller's goal — validating first is what makes this
        // function's contract ("the agent keeps its current plan") true.
        if (to < 0) return false;
        const Real keepDist = a.distOnLeg;
        // WAKE THEM. A sleeping agent is not in the active list at all, so the
        // stepper never reaches it: the errand sets moving, a valid route and a
        // destination, and the agent stands in its doorway until its OWN alarm
        // goes off — measured, with `stepped=0` and the ghost sitting 0.4 m
        // from its body, so neither the leash nor the route was to blame. A
        // director's errand overrides the day, and that includes the lie-in.
        a.wakeAt = -1;
        startTrip(a, from, to, /*fromRest=*/!a.moving);
        if (!a.route.valid()) return false;
        if (finalLink >= 0 &&
            (a.route.links.empty() ||
             a.route.links.back() != finalLink))
            a.route.links.push_back(finalLink);
        if (legLink >= 0) {
            // Finish the leg it is standing on (the same idiom as a bus pulling
            // away from where it actually stopped): prepend that link and put
            // the pose back, so the errand changes the PLAN and never the pose.
            a.route.links.insert(a.route.links.begin(), legLink);
            a.leg = 0;
            a.distOnLeg = keepDist;
            refreshPose(a);
        }
        // END AT THE POINT, not at the node. Project the destination onto the
        // last link of the route: if it lies alongside it, that is where the
        // agent stops. A destination that is not beside its final street (a
        // door round the corner) keeps the old behaviour and arrives at the
        // node, which is the honest answer rather than a wrong one.
        a.stopAtDist = -1;
        if (!a.route.links.empty()) {
            const engine::NavLink& last =
                nav_->links[static_cast<std::size_t>(a.route.links.back())];
            const engine::Vec2 A = nav_->nodes[static_cast<std::size_t>(last.from)];
            const engine::Vec2 B = nav_->nodes[static_cast<std::size_t>(last.to)];
            const engine::Vec2 AB(B.x - A.x, B.y - A.y);
            const Real len2 = AB.x * AB.x + AB.y * AB.y;
            if (len2 > 1e-6) {
                Real t = ((dest.x - A.x) * AB.x + (dest.y - A.y) * AB.y) / len2;
                t = std::max(Real(0), std::min(Real(1), t));
                const engine::Vec2 foot(A.x + AB.x * t, A.y + AB.y * t);
                const Real off = std::sqrt((dest.x - foot.x) * (dest.x - foot.x) +
                                           (dest.y - foot.y) * (dest.y - foot.y));
                if (off <= kErrandOffLink)
                    a.stopAtDist = t * last.length;
            }
        }
        // A DIRECTOR'S ERRAND OVERRIDES THE DAY. startTrip sets the trip
        // (moving, indoors, route) but not the reactive state the stepper reads,
        // so an agent sent from rest kept state=Resting and stood still holding
        // a valid route. Its activity has to move too: left at AtHome the goal
        // layer parks it again on its own schedule.
        a.state = a.mode == Agent::Mode::Driver ? Agent::State::Cruising
                                                : Agent::State::Walking;
        a.activity = Activity::Outing;
        return true;
    }

    // The `k` nav nodes nearest `p`, nearest first. Linear scan + partial sort:
    // a director's command, not a per-frame path (nearestNode is a scan too).
    std::vector<int> nearestNodesTo(engine::Vec2 p, int k) const {
        std::vector<int> idx;
        if (!nav_) return idx;
        const int n = nav_->nodeCount();
        idx.resize(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) idx[static_cast<std::size_t>(i)] = i;
        k = std::min(k, n);
        auto d2 = [&](int i) {
            return (nav_->nodes[static_cast<std::size_t>(i)] - p).lengthSquared();
        };
        std::partial_sort(idx.begin(), idx.begin() + k, idx.end(),
                          [&](int A, int B) { return d2(A) < d2(B); });
        idx.resize(static_cast<std::size_t>(k));
        return idx;
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
    // Which agent is driving the thing carrying `i` — a bus, a cab, anything.
    // -1 when it is on its own feet. `riding` answers whether; this answers
    // what, which is the question a director actually has.
    int carrierOf(int i) const { return rides_.driverOf(i); }

    // Why is this agent standing still? The question splits in two: advance()
    // was never called (a guard above the call site), or it ran and something
    // inside clamped the motion to zero. These three answer it.
    // How many agents the ADR-0062 leash held this tick, across the whole city.
    // A city where this sits above zero for minutes has people stranded in it —
    // the ghost cannot advance and the body cannot catch up — and that is the
    // one number that says so without possessing anybody.
    int tetherHeld() const { return tetherHeld_; }

    bool steppedLastTick(int i) const {
        return i >= 0 && i < static_cast<int>(advancedLast_.size()) &&
               advancedLast_[static_cast<std::size_t>(i)] != 0;
    }
    Real gapOf(int i) const {
        return i >= 0 && i < static_cast<int>(gaps_.size())
                   ? gaps_[static_cast<std::size_t>(i)] : Real(-1);
    }
    Real minGapOf(int i) const {
        return i >= 0 && i < static_cast<int>(minGaps_.size())
                   ? minGaps_[static_cast<std::size_t>(i)] : Real(-1);
    }

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
    // Is this pedestrian ON THE STREET to be drawn (and given a body)? A
    // walker, someone waiting at a stop or pausing outside -- not someone
    // indoors, riding, or far. The walker system and the renderer both ask.
    bool pedVisible(int i) const;
    // THE ROLES (city_roles.h): every resident's day is its role's, built by buildDayTable from the catalog
    // (roles.lua at level load; defaultRoleCatalog otherwise). Re-seats every agent on its new table.
    void setRoleCatalog(RoleCatalog c);
    const RoleCatalog& roles() const { return roles_; }
    // One role's table replaced outright (a level script's own day for it); false when the role is unknown.
    bool setRoleTable(const std::string& role, GoalTable t);
    // Places people go out to (everything but homes), for outings and lunch.
    struct Venue {
        PlaceType type = PlaceType::Shop;
        int node = -1;
        engine::Vec2 door;
        Real openHour = 0, closeHour = 24;
        uint8_t campus = 0;   // Place::campus: 1 teaching hall, 2 library, 3 residence, 4 quad, 5 field
        bool openAt(Real h) const {
            if (openHour == closeHour) return true;
            if (openHour < closeHour) return h >= openHour && h < closeHour;
            return h >= openHour || h < closeHour;
        }
    };
    const std::vector<Venue>& venues() const { return venues_; }
    // How many agents assignPlaces made students (0 without a campus with a residence and a teaching hall).
    int studentCount() const { return students_; }
    void setActivityCatalog(ActivityCatalog c) { catalog_ = std::move(c); }
    const ActivityCatalog& activityCatalog() const { return catalog_; }
    // Debug: agent i's follow gap and its minimum (computeGaps), +inf with no leader.
    Real debugGap(int i) const { return i >= 0 && i < static_cast<int>(gaps_.size()) ? gaps_[static_cast<std::size_t>(i)] : -1; }
    Real debugMinGap(int i) const { return i >= 0 && i < static_cast<int>(minGaps_.size()) ? minGaps_[static_cast<std::size_t>(i)] : -1; }
    const engine::NavGraph& nav() const { static const engine::NavGraph kEmpty; return nav_ ? *nav_ : kEmpty; }
    // ACTIVITY SPOTS (activities.h; the seats were the first kind -- the furniture library, M5; Glenn: benches you can
    // sit on, and so can everyone else): one per place a body does something, its point, the way it faces, its
    // height (a seat's hip), the path node it is reached from (setSpots finds it), what it is (kind), where it is
    // (tags: campus, park, sports -- set once the places are known) and who holds it.
    struct ActivitySpot {
        engine::Vec2 pos, face{0, 1};
        Real hip = 0.45;
        int node = -1;
        int occupant = -1;
        SpotKind kind = SpotKind::Sit;
        uint32_t tags = 0;
        int loop = -1;     // a Jog spot: the loop it runs (setLoops) and where on it the run starts (m)
        Real loopS = 0;
    };
    // LOOPS to run (a track): closed polylines. A Jog spot names one; a jogger runs it from the spot's place.
    void setLoops(std::vector<std::vector<engine::Vec2>> loops);
    int loopCount() const { return static_cast<int>(loops_.size()); }
    Real loopLength(int loop) const;
    engine::Vec2 loopPoint(int loop, Real s, engine::Vec2* tangent = nullptr) const;
    // The spot agent i is USING right now (phase 2: sitting, standing, running its loop), else nullptr.
    const ActivitySpot* usingSpot(int agentIndex) const;
    // AREAS a group activity happens on (a pitch): kind, tags, centre, long axis (unit), half length and width, and
    // the node it is reached from (setAreas finds it, within 150 m).
    struct ActivityArea {
        std::string kind;
        uint32_t tags = 0;
        engine::Vec2 center, axis{1, 0};
        Real halfL = 0, halfW = 0;
        int node = -1;
    };
    void setAreas(std::vector<ActivityArea> areas);
    const std::vector<ActivityArea>& areas() const { return areas_; }
    // A SESSION: one group activity running on an area -- gathering its players, then running (zones swapped at
    // half time), then ending (its players walk off). Dead once its last player has gone.
    struct Session {
        enum class State : uint8_t { Gathering, Running, Ending, Dead };
        int def = -1, area = -1;
        State state = State::Gathering;
        double gatherUntil = 0, startedAt = 0, endAt = 0;   // clockTotalHours_
        bool swapped = false;
        std::vector<int> members, roles;   // agent index, its role
        // where on its area it is (a ring's middle, a pair's midpoint; the area's centre for a roaming game) and its
        // own axis (the pair's line)
        engine::Vec2 center{0, 0}, axis{1, 0};
        Formation formation = Formation::Roam;   // the definition's
        int roleCount(int role) const;
    };
    const std::vector<Session>& sessions() const { return sessions_; }
    using SeatSpot = ActivitySpot;   // the name the seats grew up with
    void setSpots(std::vector<ActivitySpot> spots);
    void setSeats(std::vector<SeatSpot> seats) { setSpots(std::move(seats)); }
    const std::vector<ActivitySpot>& spots() const { return seats_; }
    const std::vector<SeatSpot>& seats() const { return seats_; }
    // A free spot answering `q` from `here`, reserved for agent `a`; -1 none. The one picker every goal goes through.
    int pickSpot(Agent& a, engine::Vec2 here, const ActivityQuery& q);
    // The agent's seat while it is SITTING on it (seatPhase 2 on a Sit or Lie spot), else nullptr: the renderer and
    // the walker system draw a seated body there. A jogger is not seated: it is drawn on the move where it runs.
    const SeatSpot* seatedOn(int agentIndex) const;
    // HOW A RESTING BODY IS HELD, for whoever draws it: on a seat (a bench, a chair, a fountain's rim: `hip` is world
    // Y), or in its place in a group on the grass -- sitting on the ground or lying there (`hip` 0: the drawer puts
    // it on the ground under `pos`). False for a body on its feet (walking, standing, running about).
    struct RestPose {
        enum class Kind : uint8_t { Seat, SitGround, Lie };
        Kind kind = Kind::Seat;
        engine::Vec2 pos, face{0, 1};
        Real hip = 0;
    };
    bool restPose(int agentIndex, RestPose& out) const;
    // EVERYTHING ABOUT ONE AGENT, on one line (the `who?` and `speeders?` debug commands): tier, where, how fast,
    // what it is doing (trip, seat, activity, session), its route leg and link, its rest, its leash.
    std::string describeAgent(int agentIndex) const;
    // THE CENSUS (Glenn: "Ridership feels sparse ... more people outside"): who is out right now, whatever their tier
    // -- walkers outside (moving, or standing out of doors) and moving, drivers moving, riders aboard, people waiting
    // at stops -- in all and by role. O(agents): ask it now and then, not every frame.
    struct Census {
        int agents = 0, walkers = 0, drivers = 0;
        int walkersOutside = 0, walkersMoving = 0, driversMoving = 0, riding = 0, waiting = 0;
        static constexpr int kRoles = 16;   // by role id (roles().roles: commuter, shopkeeper, stroller, student, the staff...)
        int byRole[kRoles] = {}, outsideByRole[kRoles] = {};
        int atWorkByRole[kRoles] = {};   // resting in a state wearing AtWork (on shift, in class)
        int outing = 0;                   // wearing Outing: at or on the way to a stop, by day or by night
        int busesMoving = 0, cabsMoving = 0;   // the service fleet, outside every count above
    };
    Census census() const;
    // A group member's place in its session (formations other than Roam) and the way it faces there.
    engine::Vec2 memberPlace(const Session& s, int agentIndex, engine::Vec2* face) const;
    // 1 while a departing car is still drawn at its parking space, easing to 0
    // once it has merged into its lane (see Agent::pullOffset).
    // `pullS`: how far into the pull the DRAWN car is (interpolated through
    // the sim tick by the renderer, like its position).
    static Real pullOutWeight(const Agent& a, Real pullS);
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

    // Make agents LIVE in the city (ADR-0066 Phase 3). Assign each agent a home
    // (a Home place) and a job (a routable Shop/Office/Civic place), pin its
    // home/work commute NODES to those places' sidewalk entrances, and seed the
    // surface-level relationship table (same workplace → coworker, same home →
    // neighbor). Deterministic (draws from each agent's `brain`, not the rng, so
    // the build stream is unchanged). Call AFTER build()/setWander with the same
    // graph; a no-op when `places` has no homes. `graph` must be the built one.
    void assignPlaces(const PlaceMap& places, const engine::NavGraph& graph);
    // The university's students (campus milestone 4): walkers moved into the residence hall, with class
    // in the teaching hall and the library for study. Part of assignPlaces (and so of its cache).
    void assignStudents(const PlaceMap& places, const engine::NavGraph& graph);
    // THE POPULATION CACHE (Glenn: "could we assign the job and home for each agent offline and save/load
    // that information?"). What assignPlaces decides for each agent -- home, job, errand stop, role, hours,
    // commute, starting pose -- plus its commute statistics, written to `<dir>/<key>.pop` keyed by a hash
    // of EVERYTHING it reads (the nav graph, the places, each agent's state before assignment, the bus
    // network, the shares, the format). A later load with the same inputs reads it instead of deciding
    // again. "" (the default) = no cache. RT_POPULATION_VERIFY=1 decides anyway and reports any field that
    // differs from the cache.
    void setPopulationCacheDir(std::string dir) { populationCacheDir_ = std::move(dir); }
    struct PopulationCacheReport { bool used = false, hit = false, saved = false; uint64_t key = 0; int mismatches = -1; };
    const PopulationCacheReport& populationCache() const { return popCache_; }
    // Share of drivers assignPlaces gives a CROSS-TOWN job (1.2 km+ away); the rest
    // take the nearest of their sampled jobs. Set before assignPlaces.
    void setLongCommuteShare(Real share) { longCommuteShare_ = share; }
    void setBusCommuteShare(Real share) { busCommuteShare_ = share; }
    // HOW FAR A DAY REACHES (Glenn: "a small percent wants a long distance trip between cities across the island.
    // Maybe some people go to nearby cities and most within their city"). Of the agents whose job is SAMPLED
    // (drivers, bus commuters), these shares look in the nearest towns / any town farther; the rest stay in their
    // own. A town is a street network (BusNetwork::networkOf) with jobs; a level of one town is unaffected.
    void setTripReach(Real nearbyShare, Real farShare) { nearbyTripShare_ = nearbyShare; farTripShare_ = farShare; }
    // What the last assignPlaces did with the drivers' jobs (for the load log).
    struct CommuteStats {
        int crossTownDrivers = 0, driversWithJobs = 0; Real meanDriverCommute = 0; int busCommuters = 0, busCommuteTried = 0;
        int towns = 0, ownTown = 0, nearbyTown = 0, farTown = 0;   // where everyone's work is, from their home's town
        int driverOwn = 0, driverNearby = 0, driverFar = 0;          // ...the drivers' alone
    };
    const CommuteStats& commuteStats() const { return commuteStats_; }
    const RelationshipTable& relationships() const { return relationships_; }

    // How often an agent re-DECIDES its reactive behaviour (seconds). Between
    // thinks it commits to the last decision and just acts on it. Default 0.35 s.
    void setThinkPeriod(Real seconds) { thinkPeriod_ = seconds > 0.05 ? seconds : 0.05; }
    Real thinkPeriod() const { return thinkPeriod_; }
    long faults() const { return faultCount_; }   // perception misses so far

    // World-space (XZ) points that cars must yield to in addition to the sim's own
    // pedestrians — chiefly the live player (on foot or in a car), injected by the
    // host each step so AI cars brake for and hold short of the player.
    // `halfLengths` (parallel, optional): > 0 marks a VEHICLE that long (half, m) -- the player's car, a taken car.
    // A car is not a person: it is seen down the lane far enough to stop from speed, and held short of by both
    // bodies' lengths, not by a person's clearance measured to its centre (Glenn, 2026-10-02: "If my car stops the
    // car behind me just rams me").
    void setExternalObstacles(std::vector<engine::Vec2> obstacles, std::vector<Real> halfLengths = {}) {
        externalObstacles_ = std::move(obstacles);
        externalHalf_ = std::move(halfLengths);
        externalHalf_.resize(externalObstacles_.size(), Real(0));
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
    // DEPARTURES PER STEP: each departure routes, and a clock window can send thousands at once --
    // measured on island_8_saltwood, 2313 wander routes in ONE step (357 ms), the hitch Glenn felt
    // driving the pass. Past the budget a departure reports Blocked and retries next tick, as a
    // driver waiting for launch clearance does. Buses are exempt.
    static constexpr int kDeparturesPerStep = 200;
    int departuresThisStep_ = 0;
    // Execute the agent's (GoTo) goal state: start the trip toward its target.
    // False when no trip launched (then a NoRoute row, if any, has been taken).
    bool startGoalTrip(Agent& a, int origin, bool fromRest);
    int departNode(const Agent& a) const;   // where a rest departure starts from
    // The node a GoTo state aims at, or -1 when it has no fixed one (Random
    // picks inside startWanderTrip; Fare/Drop come from the Dispatch).
    int goalNodeFor(const Agent& a, GoalTarget target) const;
    void applyTaxiFraction();
    // WHAT AN AGENT'S DAY IS: its ROLE's table (setRoleCatalog) -- by what the agent is (role, archetype), never
    // by `mode` (how it happens to be moving: a driver who parks and walks to a door keeps its day). A bus or a cab
    // runs its service table; the agent lab's wanderers their perpetual loop (goalPed_ / goalDriver_).
    const GoalTable& tableFor(const Agent& a) const {
        const int i = indexOf(a);
        if (isBus(i)) return busTable_;
        if (isTaxi(i)) return taxiTable_;
        if (wander_) return a.archetype == Agent::Mode::Driver ? goalDriver_ : goalPed_;
        return roleTables_[static_cast<std::size_t>(dayRoleOf(a))];
    }
    GoalTable& tableFor(const Agent& a) { return const_cast<GoalTable&>(static_cast<const CitySim&>(*this).tableFor(a)); }
    // The role whose day an agent runs: its own -- except a stroller with no day out to take (one who drives, or
    // a city with nowhere to go), whose park is its "work": the commuter's day (the historical park-as-destination).
    int dayRoleOf(const Agent& a) const {
        const int r = static_cast<int>(a.role);
        if (r < 0 || r >= static_cast<int>(roleTables_.size())) return 0;
        if (a.role == Agent::Role::Stroller && (a.archetype != Agent::Mode::Pedestrian || venues_.empty())) return 0;
        return r;
    }
    bool outingStroller(const Agent& a) const { return !wander_ && a.role == Agent::Role::Stroller && dayRoleOf(a) != 0; }
    int pickOuting(Agent& a, int origin);   // GoalTarget::Outing -> a node (sets tripVenue)
    int pickCampusBreak(Agent& a, int origin);   // GoalTarget::Campus -> a node (tripVenue / tripSeat)
    // Free seats near the campus (quad benches, bleachers): the seats within reach of a quad or field venue.
    int pickCampusSeat(Agent& a, engine::Vec2 here);
    int pickActivity(Agent& a, int origin, const GoalState& s);   // GoalTarget::Activity -> the spot's node (tripSeat)
    // Tags each spot by where it stands, once the places are known (assignPlaces): campus (near a quad or a
    // field), sports (near a field).
    void tagSpots();
    // THE CATALOG (activities.h): what there is to go and do, and the menus that choose. pickFromMenu chooses an
    // activity and its site -- a place, a spot or a street corner -- and returns the node to walk to (tripVenue /
    // tripSeat / tripActivity set), -1 for nothing.
    int pickFromMenu(Agent& a, int origin, const Menu& m);
    int tryActivity(Agent& a, int origin, int def, int prevVenue, bool countOnly, int* count);
    std::string venueKind(const Venue& v) const;
    std::string siteKindOf(const Agent& a) const;   // the kind of site the agent's trip is for (arrival's dwell)
    int pickLunch(Agent& a, int origin);    // GoalTarget::Lunch  -> a node, or -1
    engine::Vec2 freeStandingSpot(const Agent& a, engine::Vec2 want, engine::Vec2 along) const;
    void installGoalTables(GoalTable pedestrian, GoalTable driver);
    void rebuildRoleTables();   // every role's day, from roles_
    void reseatOnTables();      // each agent onto its table's state wearing its label (else the entry)
    bool launchClear(const Agent& a, int node) const;   // no moving car near the spawn
    void seatBusAt(int idx, int node, int queued = 0);   // a bus at rest on a stop
    // The nearest FREE bay to `target` within `maxDist`, claimed for agent
    // `self` (-1 if none). The one allocator for a car at load and on arrival.
    int claimBayNear(engine::Vec2 target, int self, Real maxDist);
    void releaseBays(Agent& a);          // free both the held and the reserved bay
    Real busDistanceToStop(const Agent& a) const;
    Real busStandBack(const Agent& a) const;   // route metres short of its stop node
    // WHERE A RIDER WAITS for route r at stop s: the kerb beside where the bus's doors stand -- back up the street the
    // bus arrives by (busStandBackAt), out past the carriageway onto the pavement: the same point the stop's pole and
    // bench stand at (buildBusStopProps). The stop NODE is usually a junction's middle, and riders who waited where
    // their walk ended stood in it (Glenn: "a lot of npcs standing around in the middle of the street"). False when
    // the route does not say how it arrives.
    bool stopWaitSpot(int route, int stop, engine::Vec2& at, engine::Vec2& along) const;
public:
    // The same stand-back for a bus arriving at `inLink`'s end node, measured from that node back along the
    // link to the bus's CENTRE -- where the stop's furniture belongs (#36). Walks back through plain nodes
    // like busStandBack; the bus length is the fleet's.
    Real busStandBackAt(int inLink) const;
private:
    std::vector<int> nearestFreeBays(engine::Vec2 target, Real maxDist, int k) const;
    bool parksInBays(const Agent& a) const;   // a private car that parks (not a bus/cab/wanderer)
    void advance(Agent& a, Real dt, Real gap, Real minGap);
    // advance()'s junction verdict: the speed target after the signal brake and
    // the box-occupancy / turn-yield scan, plus where the stop line is -- in
    // ROUTE metres from the car, because the line need not be on the current
    // leg (see junctionAhead).
    struct JunctionGate {
        Real cap = 0;               // speed target after junction/signal/yield caps
        bool yieldAtLine = false;   // holding for box/oncoming traffic/exit room
        int node = -1;              // the junction ahead (-1: none within reach)
        int approachLink = -1;      // the route link that enters it (its signal)
        Real distToNode = 0;        // route metres to the node
        Real distToLine = 0;        // route metres to the stop line (< 0: past it)
    };
    // The signal a car faces at an approach, for ITS movement there (read off its route): during a
    // lead arrow its left may be green while going straight is red (ADR-0109).
    Move moveFor(const Agent& a, int approachLink) const;
    SignalState signalFor(const Agent& a, int approachLink) const;
    JunctionGate junctionSpeedCap(const Agent& a, int li, Real target) const;
    // The next junction on the route within `horizon` route metres: the leg
    // whose link enters it, the distance to its node, and the length of the
    // APPROACH (back to the previous junction), which is where the stop line
    // may sit. horizon 0 looks at the current link only.
    struct JunctionAhead {
        int leg = -1;
        int node = -1;
        Real toNode = 0;
        Real approach = 0;
    };
    JunctionAhead junctionAhead(const Agent& a, Real horizon) const;
    Real stopLineBack(const Agent& a, const JunctionAhead& ja) const;   // line, metres short of ja.node
    Real senseAhead(Agent& a);   // perception/memory/TTC: distance to a body ahead
    // An injected body's half length when it is a vehicle (sensed id -(1+k)), else 0
    Real externalHalfOf(int id) const {
        const int k = -id - 1;
        return id < 0 && k < static_cast<int>(externalHalf_.size()) ? externalHalf_[static_cast<std::size_t>(k)] : Real(0);
    }
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
    bool clearPromotion(int i);   // seat a promoted car behind live traffic
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
    // Per-node junction box radius: the widest incident half-width (+ the
    // junction pad at street intersections: the drawn mouth). Nonzero at
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
    Real longCommuteShare_ = 0;   // setLongCommuteShare
    Real busCommuteShare_ = 0;    // setBusCommuteShare
    Real nearbyTripShare_ = 0.15, farTripShare_ = 0.05;   // setTripReach
    CommuteStats commuteStats_;
    std::vector<Agent> agents_;
    std::vector<SimVehicle> vehicles_;
    // Adopted fleet catalogue; empty = use the built-in table (see setFleet).
    std::vector<VehicleBody> fleet_;
    std::vector<ParkingBay> bays_;
    // Each link's reverse twin (-1 if one-way), and a per-link route cost
    // scale kept at 1 except while a bay departure prices its own twin.
    std::vector<int> twinOf_;
    std::vector<Real> departScale_;
    // True while startWanderTrip has priced the U-turn in departScale_: startTrip's own searches
    // must see the same prices, or the trip it builds could U-turn after all.
    bool wanderPriced_ = false;
    std::vector<std::vector<int>> baysOnLink_;   // link -> bay indices
    std::vector<char> bayNarrowed_;   // link (or its reverse) carries bays
    std::vector<uint8_t> advancedLast_;   // did advance() step agent i last tick
    int tetherHeld_ = 0;                 // agents the leash held last tick
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
    std::vector<int> sensedSet_;        // ...the agents given a slot this tick (cleared next tick)
    std::vector<uint8_t> advancedFlags_;   // per agent: advance() stepped it this tick
    std::vector<int> advancedSet_;         // ...which ones (cleared next tick)
                                        // (lets grid candidates map back to ghosts)
    std::vector<engine::Vec2> externalObstacles_;   // host-injected (the live player)
    std::vector<Real> externalHalf_;                // parallel: a vehicle's half length, 0 = a person
    std::vector<engine::Vec2> staticObstacles_;     // host-injected, static (signal poles)
    std::vector<std::pair<engine::Vec2, Real>> junctions_;   // centre + box radius
    std::vector<Real> nodeBoxRadius_;   // per node: widest incident half-width (+ pad)
    Real junctionPad_ = 0;              // + this at street intersections (see setJunctionPad)
    // P4.1 spatial index. grid_ holds every agent (K re-hashed each step —
    // effectively free, place() early-outs on an unchanged cell; V only on its
    // coarse tick, when its position actually moves). junctionGrid_ is a static
    // bake of junctions_ so nearJunction() stops scanning the whole list.
    AgentGrid grid_;
    // ...and the same cells holding ONE tier each (setTier keeps them): the wake check asks for dormant agents in a
    // kilometre and the promote check for far ones in 500 m -- from grid_ that was everyone there (~5,000 at 100k),
    // each loaded and sorted to read its tier
    AgentGrid vGrid_, dGrid_;
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
    std::string populationCacheDir_;      // setPopulationCacheDir
    PopulationCacheReport popCache_;
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
    // THE ROLES and each one's day (setRoleCatalog); the wanderers' tables (the agent lab) beside them.
    RoleCatalog roles_ = defaultRoleCatalog();
    std::vector<GoalTable> roleTables_ = [] {
        std::vector<GoalTable> v;
        for (const ResidentRole& r : defaultRoleCatalog().roles) v.push_back(buildDayTable(r.day));
        return v;
    }();
    GoalTable goalPed_ = wanderGoals(false);
    ActivityCatalog catalog_ = defaultActivityCatalog();
    int students_ = 0;
    std::vector<Venue> venues_;
    // the night's venues (restaurants, bars, clubs) by 250 m cell, for eveningVenue (indexNightVenues)
    std::unordered_map<int64_t, std::vector<int>> nightVenues_;
    void indexNightVenues();
    std::vector<SeatSpot> seats_;
    struct Loop { std::vector<engine::Vec2> pts; std::vector<Real> cum; Real length = 0; };
    std::vector<Loop> loops_;
    std::vector<ActivityArea> areas_;
    std::vector<Session> sessions_;
    int joinSession(Agent& a, int def, engine::Vec2 here, Real distLo, Real distHi, int nearest);
    void leaveSession(Agent& a);
    bool clearOfWalks(engine::Vec2 p, Real r) const;
    int joinSettled(Agent& a, int def, const std::vector<std::pair<Real, int>>& cand,
                    const std::function<int(const Session&)>& freeRole);
    void stepSessions();
    engine::Vec2 zonePoint(const Session& s, int role, uint32_t bits) const;   // a point in that role's zone
    void releaseSeat(Agent& a);
    int pickSeat(Agent& a, engine::Vec2 here);   // a free seat to walk to, reserved; -1 none
    void stepSeats(Real dt);                     // the walk off the path to a seat and back
    GoalTable goalDriver_ = wanderGoals(true);
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
    std::vector<engine::Vec2> crowdBase_;   // scratch: each walker's pose before the crowd offset (Agent::crowdOffset)
    std::vector<uint8_t> crowdHas_;
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
    // THE TIER LISTS (the 100k plan, stage 1): every K agent and every V agent, ascending, kept by setTier. The
    // per-tick passes walk these, never the whole population -- a dormant agent (93% of 100k) is touched only when
    // the bubble calls it back. Ascending order keeps every pass's order, and so its result, as it was.
    std::vector<int> kIdx_, vIdx_, tierScan_;
    std::vector<int> pinned_;   // the followed agents (pinAgent)
    std::vector<int> seatScan_;   // stepSeats' K + V, ascending
    Real nearScale_ = 1.0, farScale_ = 1.0;
    long resumedTrips_ = 0;   // wakes that resumed a trip (wakeDormant) rather than rebuilt from the schedule
    // THE SLEEPERS' DAY (Glenn: the island "feels like a graveyard"; the census: a dormant agent ran no schedule, so
    // no new trip ever started across 90% of the island). A dormant agent's next schedule event -- its next departure,
    // or its arrival when mid-trip -- sits in a min-heap on the game clock; when it comes due the agent is placed by
    // its schedule (placeFromSchedule: a trip begun, or indoors at the far end) without waking, the GPU crowd draws the
    // trip, and its next event goes in. Stale entries (it woke since) are skipped when popped.
    struct DormantEvent { double at; int agent; bool operator>(const DormantEvent& o) const { return at > o.at || (at == o.at && agent > o.agent); } };
    std::priority_queue<DormantEvent, std::vector<DormantEvent>, std::greater<DormantEvent>> dormantHeap_;
    std::vector<double> dormantEventAt_;   // per agent: its live event's time (-1 none)
    long dormantEventsRun_ = 0;
    double dormantEventMs_ = 0;
    void scheduleDormantEvent(int i);
    void runDormantEvents();
    std::vector<int> crowdDirty_;  // takeCrowdChanges
    bool crowdDirtyAll_ = true;
    void crowdChanged(int i) {
        if (crowdDirtyAll_) return;
        if (crowdDirty_.size() > agents_.size()) { crowdDirtyAll_ = true; crowdDirty_.clear(); return; }
        crowdDirty_.push_back(i);
    }
    bool rehashAll_ = true;   // a bulk move (build, seedFromSchedule): re-place EVERYONE in the grid next tick
    void setTier(int agentIndex, Agent::Tier t);
    void rebuildTierLists();
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
