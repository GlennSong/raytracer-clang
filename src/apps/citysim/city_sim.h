#ifndef RAYTRACER_APPS_CITYSIM_CITY_SIM_H
#define RAYTRACER_APPS_CITYSIM_CITY_SIM_H

#include <algorithm>
#include "../../proc_rng.h"
#include "../../engine/ai/agent_memory.h"
#include "../../engine/ai/nav_graph.h"
#include "../../engine/ai/pathfind.h"
#include "agent_grid.h"
#include "agent_id.h"
#include "activities.h"
#include "city_goals.h"
#include "city_roles.h"   // every resident's day comes from its role (RoleCatalog, buildDayTable)
#include "city_rail_riders.h"
#include "places.h"
#include "place_kinds.h"   // what kind each place is, and its tags (places.lua)
#include "households.h"     // ages, life stages and households (people.lua)
#include "relationships.h"
#include "../driving/driving.h"   // the driving (#234; a level that names citysim.drivingRules)
#include "city_driving.h"         // ...and the city sim's side of it: the drivers handed over (#255)
#include "city_poses.h"           // where the city's bodies are each fixed step (ADR-0454)
#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <queue>
#include <map>
#include <unordered_map>
#include <vector>

namespace engine {
struct CityMarker;     // components.h: a named point beside a street (ADR-0441)
struct CityResident;   // ...a resident with the agent brain
struct CityHousehold;  // ...and the residents who share a household's cars (ADR-0443)
class World;           // world.h: the entities, the households' cars among them (ADR-0444)
}  // namespace engine

namespace citysim {

class CityBrain;      // city_brain.h: the resident with the agent brain (ADR-0441)

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
    // Its id in the role catalog (city_roles.h RoleId): a built-in, or a staff role assignStaff gave it.
    RoleId role = 0;
    // WHO THEY ARE (households.h, people.lua): their household in the sim's plan (-1: none -- a bus or cab driver, or
    // more people than the housing holds), their age and their life stage (an index into the people catalog's
    // stages; -1 with no household). Set in assignPlaces; time is frozen, so never again.
    int32_t household = -1;
    uint8_t age = 0;
    int8_t stage = -1;
    // AN EVENING JOB (roles.lua side_job; plan step 6): the venue they work at on shift nights (-1: none) and the
    // shift's hours. Set by assignSideJobs; a shift night is their evening (eveningPlan).
    int32_t sideVenue = -1;
    float sideStart = 0, sideEnd = 0;

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
    // A DIRECTOR'S PACE ON FOOT (ADR-0441: the agent brain's walk skill, city_brain.h): while above 0, this walker moves
    // at that many times the walking speed and is drawn on `directedRide` (a board, a scooter, or nothing), whatever
    // activity its trip is for. 0: the trip's own (CitySim::ridePace, rideOf).
    float directedPace = 0;
    Ride directedRide = Ride::None;
    // OWNED BY THE AGENT BRAIN (ADR-0442; city_brain.h): where this person is, on foot or in a car, is the brain's to
    // say and nobody else's. The sim's day table never places them: CitySim::placeFromSchedule, the one function every
    // placing pass goes through (the seeding at build and at a clock jump, a dormant person's next event, a wake),
    // leaves them as they are. It goes with `directed` (the goal layer leaves their plan alone), which by itself still
    // lets a city with a day schedule place the person again.
    bool brainOwned = false;
    // IN A HOUSEHOLD'S CAR (ADR-0444; city_car.h): the number of the brain's car this person sits in (-1: none). Such a
    // car is an entity, not one of `vehicles`, so this person stays a walker (`mode` Pedestrian, `vehicle` -1) who is
    // not drawn on foot (pedVisible). The car is the authority on where they are: `pos` and `heading` follow its pose
    // every tick, one way (CityBrain, ADR-0445), so a reader of `pos` need not know about `inCar`.
    int inCar = -1;
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
    // Where the current outing/lunch trip is going, kept until arrival: a trip resumed part-way (a train journey's
    // street leg) resumes to the same place, not a newly picked one.
    int outingTo = -1;
    engine::Vec2 crashAnchor;        // where the pile-up started
    // Gridlock escape (ADR-0066 device fix): seconds this car has been pinned at
    // a junction hold (yieldAtLine with ~zero speed). Past a few seconds, box
    // occupancy stops counting occupants that are THEMSELVES stalled — a real
    // driver inches through a gridlocked box — which breaks the circular wait a
    // knot of overlapping junctions can otherwise form. Staggered per agent.
    Real holdTimer = 0;
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
    // AT A CROSSWALK (ADR-0452): the road model's crosswalk this walker waits at the kerb end of (`onCrosswalk` false) or
    // is crossing (true), as of its last step; -1: neither. `crossWaited`: seconds it has waited there to cross.
    // `crossingKind`: how the crossing it is on began (CrossingKind), for the counts.
    int crosswalkAt = -1;
    bool onCrosswalk = false;
    bool aboutToCross = false;   // waiting at a kerb with no lights, plainly about to cross (drivers give way)
    int8_t crossFromEnd = 0;     // the kerb end of `crosswalkAt` they step off from
    uint8_t crossingKind = 0;
    Real crossWaited = 0;
    // Continuous sideways lean off the path (ADR-0061), rate-limited, so a walker
    // steers smoothly around what it sees instead of popping. Decays back to 0.
    Real lateralOffset = 0;
    // Where the CROWD has put it off its path (the overlap floor's push, KEPT): re-applied each tick, so a packed
    // doorway is spaced out once and stays spaced, rather than snapped back onto the path line and shoved apart
    // afresh every tick (1.5-3 m hops at 13 m/s, the speed watchdog's "avoiding" walkers). Eases back to the path.
    engine::Vec2 crowdOffset{0, 0};

    // Imperfect perception (ADR-0060): each step the agent perceives obstacles
    // with probability `reliability`; otherwise it misses (a fault), drawn from
    // `faults`, its own stream so faults reproduce from the sim seed. `brain` is
    // the agent's FIXED identity bits (pace, schedule, job, park, pose): read, never
    // drawn from -- it was once the fault stream too, so every fault an agent
    // rolled changed who it was (ADR-0265).
    float reliability = 1.0;
    uint32_t brain = 1;
    engine::ProcRng faults{1};
    // This agent's own stream for TRIP decisions (lane choice, wander goal),
    // separate from `brain`'s fault stream. Per-agent so a decision depends only
    // on how many decisions THIS agent has made — the property that lets an
    // agent be advanced out of turn (waking from its schedule) without shifting
    // everybody else's choices. Seeded from `brain` at build.
    engine::ProcRng tripRng{1};
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
// FireEngine: a fire station's (2026-10-13), stationed in its bays -- never dealt to ordinary traffic, like the bus.
enum class VehicleType : uint8_t { Sedan, Hatchback, SUV, Pickup, Van, BoxTruck, Bus, FireEngine, Taxi };
// A SERVICE body is never dealt to ambient traffic or a kerb bay: the fire engine is its station's; the bus and the
// taxi are the services' (their old driving went with ADR-0460; the bodies stay in the fleet for the new driving's)
inline bool serviceVehicle(VehicleType t) {
    return t == VehicleType::Bus || t == VehicleType::FireEngine || t == VehicleType::Taxi;
}

// A body's physical dimensions (metres) + its type. `length` is the travel axis;
// car-following keeps cars a bumper apart from THIS, so a longer truck naturally
// holds (and is held at) a larger gap.
struct VehicleBody {
    Real length = 4.2, width = 1.8, height = 1.3;
    VehicleType type = VehicleType::Sedan;
    // What this car can pull away at (m/s^2, 0 to 10 m/s, PhysicsWorld::launchAccel on its own handling -- item 23):
    // the brains plan its launches under it, so a bus pulls away like a bus and a physical one keeps its ghost.
    // 0: not measured (nothing measures it since ADR-0461), so the stock kCarAccel.
    Real launch = 0;
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
// The built-in fleet as the new driving model's classes (#248), one per slot ("fleet_slot_<n>"): length and width from
// the body; it has no wheels, so the wheelbase and steering are vehicle_class.h's kUnknown* and the rest its defaults.
driving::VehicleClasses builtInDrivingClasses();

// A drivable car. Kinematic in the sim core; its pose tracks its driver. Inert
// when `driver < 0`. Its body (type + dimensions) comes from the fleet table.
// How far off its final street a director's destination may sit and still be
// walked to exactly. Beyond this it is not really "along that street" and the
// agent arrives at the node instead.
constexpr Real kErrandOffLink = 15.0;
// How close you have to be to open a car door (possession's `board`, a trip's getting in). Measured rather than
// guessed: a walker sent to a parked car stops on the PAVEMENT, and the car is parked on the far side of the
// carriageway — 8.3 m off the street's centreline one way, the footway the other, about 20 m apart on a metro street
// with nothing wrong. Reaching a car you can see across the road is a person crossing it, so the reach is a street's
// width. Getting to the STREET is still the director's problem, which is the part that matters.
constexpr Real kBoardReach = 25.0;
// How many of the nodes nearest a walker sendAgentTo looks at for one a walkable link leaves (ADR-0446).
constexpr int kWalkStartNodes = 8;
// A walker's speed (m/s) before its own pace and what it rides on (Agent::speedFactor, CitySim::ridePace).
constexpr Real kWalkSpeed = 1.4;

struct SimVehicle {
    Real length = 4.2;
    Real width = 1.8;
    Real height = 1.3;
    VehicleType type = VehicleType::Sedan;
    int slot = -1;               // its fleet slot (fleetBody): the body it was given, read for its launch
    int driver = -1;             // agent index, or -1 when parked/unpossessed
    engine::Vec2 pos;            // cached pose (mirrors the driver while possessed)
    engine::Vec2 heading{1, 0};
    // Parked OFF-STREET -- in a garage or driveway, not on the road: not drawn,
    // not a body anything collides with. Where a car goes when there is no
    // marked bay or clear kerb near its owner's destination, rather than onto
    // a heap at the corner.
    bool offStreet = false;
    // Parked UP A GARAGE (a garage bay): the deck's height, world y; below -1e29 on the ground.
    engine::Real parkedY = -1e30;
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
    // `pedCount` agents on foot over `graph`, seeded by `seed`. Deterministic (ADR-0002). Nobody is built driving: the
    // cars are the driving's car entities (city_car.h; ADR-0461, ADR-0462).
    void build(const engine::NavGraph& graph, int pedCount, uint32_t seed);

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
            if (!serviceVehicle(fleetBody(s2).type)) ++usable;
        if (usable <= 0) return ((i % n) + n) % n;   // a fleet of only buses
        int want = ((i % usable) + usable) % usable;
        for (int s2 = 0; s2 < n; ++s2) {
            if (serviceVehicle(fleetBody(s2).type)) continue;
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
    void step(Real dt, Real hoursPerSecond = 1.0 / 3600.0);   // (real time by default: DayNightCycle::dayMinutes)

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
    // WHO IS ON SHIFT (storefronts stage 3, the OPEN signs): per place (by PlaceId), how many of the people who work
    // there are at work now -- a simulated agent by its own state, a dormant one by its schedule. `out` is resized to
    // `placeCount`. `only` (by PlaceId, non-zero = wanted) limits the count to those places: a dormant worker's
    // schedule costs a snapshot, and the island's few hundred thousand of them took ~180 ms a call.
    // Is this worker at work now? A simulated agent by its own state, a dormant one by its schedule (a snapshot,
    // ~0.75 us): the one test staffOnShift makes, for callers that spread it over frames.
    bool atWorkNow(const Agent& a) const {
        return a.tier == Agent::Tier::D ? scheduleSnapshot(a, clockHours_).where == Snapshot::Where::AtWork
                                        : a.activity == Agent::Activity::AtWork;
    }
    void staffOnShift(std::vector<uint16_t>& out, int placeCount, std::vector<uint16_t>* hired = nullptr,
                      const std::vector<uint8_t>* only = nullptr, PlaceId watch = kNoPlace,
                      std::vector<int>* watchers = nullptr) const;   // + who works at `watch` (opensigns? telemetry)

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
    // Whether tonight (noon to noon) is one of agent a's evening shifts (its role's side_job `nights`), and the shift's
    // hours if so.
    bool sideShiftTonight(const Agent& a, Real& start, Real& end) const;
    int sideJobs() const { return sideJobs_; }   // agents holding an evening job
    // ON A BOARD (activities.lua `ride`: a child skating round the block): what walker i rides on the trip it is on --
    // Ride::None on foot -- and the pace it rides at, times its walking speed (1 on foot).
    Ride rideOf(int agentIndex) const;
    Real ridePace(const Agent& a) const;
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
    const std::vector<SimVehicle>& vehicles() const { return vehicles_; }

    // Curbside parallel-parking bays (roads-v2.1 R6b, plan 4d phase 1):
    // marked bays mid-link on at-grade Local streets, seeded ~half full with
    // scenery cars at build; an arriving driver claims a free bay on its
    // arrival link instead of the old grass verge, and departure frees it.
    // occupant: -1 free, kBayScenery build-time filler, else the agent id.
    // THESE ARE THE OLD DRIVING'S OWN BAYS, laid at build by its own rule (a bay every 7 m along a chain of links, 22 m
    // back from each end) and parked in by its cars only. They are NOT the road model's kerb parking spaces
    // (engine::RoadModel::kerbRuns, ADR-0439: stated once, kept clear of each arm's stop line), which the new driving
    // parks in and the paint draws on a level it drives. The two lists differ (the traffic lab: 1294 bays, 1098
    // spaces); this one goes with the old driving.
    static constexpr int kBayScenery = -2;
    struct ParkingBay {
        engine::Vec2 pos, heading;   // bay centre + facing (along the link)
        int link = -1;               // directed link whose right curb it hugs
        engine::Real station = 0;    // metres from the link's `from` node
        // The Parking BAND this bay sits in (NavLink::parkWidth). The painted
        // stall is drawn to it, so the markings can never lap onto the kerb.
        engine::Real width = 0;
        int occupant = -1;
        // A GARAGE STALL: its deck's height (world y; below -1e29 a kerbside bay) and its garage. (No garage stall is
        // laid since ADR-0462: their stalls came from the level, and nothing hands them in now.)
        engine::Real y = -1e30;
        int garage = -1;
    };
    engine::Real bayReach2(const ParkingBay& b, const engine::Vec2& target) const;
    // Effective lane spacing for a link: a parked-up street loses its curb
    // strips from the DRIVABLE width (band-model semantics, sim-side). The
    // render bridge lines its lane arrows up with the same spacing.
    engine::Real laneSpacingFor(int li) const;
    int walkerFile(const Agent& a) const;       // which file across the pavement a walker keeps (0 = by the kerb)
    engine::Real walkerVerge(const Agent& a) const;   // ...as its distance in from the kerb
    const engine::NavGraph* graph() const { return nav_; }
    // RT_TRAFFIC_DEBUG: why a walker last stood still (1 kerb gap, 2 signal line, 3 a car body, 4 its leader) and
    // its follow gap -- the "spawn point" heaps. A CAR's (#218, the lab's mid-turn freezes): 10 a crash hold, 11 a red,
    // 12 a crossing car in the box, 13 oncoming (a turn), 14 no exit room, 15 its lane leader, 16 a body in the wedge,
    // 17 a person seen, 18 its tether, 19 a speed cap (bend, junction pace)
    int holdWhyOf(int i) const { return i >= 0 && i < static_cast<int>(holdWhy_.size()) ? holdWhy_[static_cast<std::size_t>(i)] : 0; }
    void holdWhy(const Agent& a, uint8_t why) {
        const std::size_t i = static_cast<std::size_t>(&a - agents_.data());
        if (holdWhy_.size() != agents_.size()) holdWhy_.assign(agents_.size(), 0);
        if (i < holdWhy_.size()) holdWhy_[i] = why;
    }
    std::vector<uint8_t> holdWhy_;
    std::vector<int> solverFrom_, solverNbr_;   // the overlap solver's neighbours, gathered once a tick (advance)

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
    long dormantWaypoints() const { return dormantWaypoints_; }   // sleepers under way moved on (through traffic)
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

    // TAKE THE TRAIN (city_rail_riders.h): plan the journey to `dest` in another town, register it, and walk the
    // agent to the station's way in; the sim takes it from there. False when no train helps (same town, or a town
    // without a station) or the station cannot be walked to.
    bool sendAgentByTrain(int agentIndex, engine::Vec2 dest) {
        if (!nav_ || !rail_.active() || agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size())) return false;
        Agent& a = agents_[static_cast<std::size_t>(agentIndex)];
        if (a.mode != Agent::Mode::Pedestrian || rail_.tripOf(agentIndex)) return false;
        const int origin = nav_->nearestNode(a.pos);
        const int to = nav_->nearestNode(dest);
        if (origin < 0 || to < 0 || origin == to) return false;
        RailTrip t;
        if (!rail_.plan(origin, nav_->nodes[static_cast<std::size_t>(origin)], to, nav_->nodes[static_cast<std::size_t>(to)], t))
            return false;
        const int acc = rail_.stations()[static_cast<std::size_t>(t.from)].accessNode;
        if (acc != origin && !engine::findRoute(*nav_, origin, acc, /*pedestrian=*/true).valid()) return false;
        rail_.start(agentIndex, t);
        ++rail_.stats.planned;
        a.wakeAt = -1;
        a.activity = Activity::Outing;
        if (acc == origin) { enterStation(a, agentIndex); return true; }
        startTrip(a, origin, acc, /*fromRest=*/!a.moving);
        if (!a.moving) { rail_.forget(agentIndex); return false; }
        a.state = Agent::State::Walking;
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

    // A FEW STEPS OFF THE PATHS (ADR-0415 as built: the trip lab's person from their front door to their car, from
    // the car to a door beside it). A walker at rest, not under way, is moved straight toward `to` at `speed` for
    // `dt` seconds, facing the way it goes; true once it is there. The paths are a walker's only way about otherwise:
    // a walk of twenty metres along its own pavement to a car behind it was routed across the street, along the far
    // pavement and back. The caller keeps it short (within kBoardReach) and clear of the carriageway.
    bool stepAgentToward(int agentIndex, engine::Vec2 to, Real speed, Real dt) {
        if (agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size())) return false;
        Agent& a = agents_[static_cast<std::size_t>(agentIndex)];
        if (a.mode != Agent::Mode::Pedestrian || a.moving) return false;
        a.indoors = false;
        a.state = Agent::State::Walking;
        const engine::Vec2 d = to - a.pos;
        const Real L = d.length(), step = speed * dt;
        if (L <= step) {
            a.pos = to;
            a.speed = 0;
            a.state = Agent::State::Resting;
        } else {
            a.heading = d * (1.0 / L);
            a.pos = a.pos + a.heading * step;
            a.speed = speed;
        }
        grid_.place(agentIndex, a.pos);
        return L <= step;
    }

    // Get out again, on the kerb beside the car, leaving it where it stands.
    bool alightVehicle(int agentIndex) {
        if (agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size()))
            return false;
        Agent& a = agents_[static_cast<std::size_t>(agentIndex)];
        if (a.vehicle < 0 || a.vehicle >= static_cast<int>(vehicles_.size()))
            return false;
        SimVehicle& v = vehicles_[static_cast<std::size_t>(a.vehicle)];
        // INTO ITS SPACE. A drive sent to a destination (sendAgentTo) was routed to a free bay near it, reserved for
        // this car. When that drive has ENDED -- arrived along the bay's own street, the same test an ordinary arrival
        // parks by (arriveOrChain) -- the car is left IN the bay, not in the lane, and its driver gets out beside it
        // (at a garage, at its door on the street: the stall is up a deck). Not by distance: a bay whose station lies
        // inside a junction's box is one a car cannot stop at, and it came to rest 21 m on. Mid-drive, or anywhere
        // else, the reservation is let go and the car stays where it stands.
        const int self = agentIndex;
        engine::Vec2 carPos = a.pos, carHeading = a.heading;
        if (a.targetBay >= 0 && a.targetBay < static_cast<int>(bays_.size()) &&
            bays_[static_cast<std::size_t>(a.targetBay)].occupant == self) {
            const ParkingBay& b = bays_[static_cast<std::size_t>(a.targetBay)];
            const bool driveEnded = !a.moving && a.route.valid() && a.route.links.back() == b.link &&
                                    a.leg >= static_cast<int>(a.route.links.size());
            if (driveEnded) {
                a.parkedBay = a.targetBay;
                carPos = b.pos;
                carHeading = b.heading;
                v.parkedY = b.y;
                // out of a kerb space beside it; at a garage, out at its door on the street (the stall is up a deck)
                if (b.garage < 0) {
                    a.pos = b.pos;
                    a.heading = b.heading;
                }
            } else {
                bays_[static_cast<std::size_t>(a.targetBay)].occupant = -1;
            }
        }
        a.targetBay = -1;
        v.pos = carPos;
        v.heading = carHeading;
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

    // Mark an agent as host-driven (the player): the sim won't run its AI brain.
    void setPlayerControlled(int agentIndex, bool on) {
        if (agentIndex >= 0 && agentIndex < static_cast<int>(agents_.size()))
            agents_[agentIndex].playerControlled = on;
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
        int legLink =
            (a.moving && a.route.valid() &&
             a.leg < static_cast<int>(a.route.links.size()))
                ? a.route.links[static_cast<std::size_t>(a.leg)] : -1;
        Real keepDist = a.distOnLeg;
        const bool onFoot = a.mode == Agent::Mode::Pedestrian;
        int from = legLink >= 0 ? nav_->links[static_cast<std::size_t>(legLink)].to : nav_->nearestNode(a.pos);
        // A WALKER LEAVES FROM A NODE PEOPLE WALK: the nearest node can be a ramp's or a freeway's, none of whose links
        // is walkable (NavLink::walkable), and a walk from it never routes -- measured on brain_crowd: people at a place
        // beside an arterial, 10 m from a ramp's node, could not walk anywhere and stood for good (ADR-0446). The
        // nearest few nodes are looked at for the first a walkable link leaves; none: the nearest, as before.
        if (legLink < 0 && onFoot && from >= 0) {
            const auto walked = [&](int n) {
                for (const int l : nav_->outLinks[static_cast<std::size_t>(n)])
                    if (nav_->links[static_cast<std::size_t>(l)].walkable) return true;
                return false;
            };
            if (!walked(from))
                for (const int n : nav_->nearestNodes(a.pos, kWalkStartNodes))
                    if (walked(n)) {
                        from = n;
                        break;
                    }
        }
        // A CAR STILL IN ITS BAY (ADR-0415 as built) leaves from the bay's own street: routed from the node AHEAD of
        // the bay, startTrip pulls it out at the bay's station. From the nearest node, which can be the one behind
        // it, the drive began back there -- the car drawn reversing out of its space up the street.
        if (legLink < 0 && !onFoot && !a.moving && a.parkedBay >= 0 && a.parkedBay < static_cast<int>(bays_.size()) &&
            (bays_[static_cast<std::size_t>(a.parkedBay)].pos - a.pos).length() < 3.0)
            from = nav_->links[static_cast<std::size_t>(bays_[static_cast<std::size_t>(a.parkedBay)].link)].to;
        if (from < 0) return false;
        // A DESTINATION THAT ROUTES. The single nearest node to `dest` can have
        // no path from here at all — 394 -> 86 in metro are 32 m apart with no
        // route for a pedestrian OR a car (different components of the graph),
        // and every walk_to that snapped to such a node was refused. Try the
        // nearest few and take the first that routes.
        // ARRIVE ALONG THE STREET THE DESTINATION IS ON. A door, a parked car
        // and a bus stop all belong to a LINK, not to a junction, so routing to
        // the nearest node leaves the agent wherever that junction happens to
        // be — measured at 23 m from a civic entrance it had been sent to.
        // Routing to the near end of the destination's own link and appending
        // that link makes the last street it walks the one it was sent to, and
        // the stop-short below then lands it beside the address.
        // ...ON THE DESTINATION'S SIDE OF IT. A street is two links, one each way, and a walker walks a link on
        // its kerb side (rightOf its direction); the nearest link is either of the pair, so half the errands
        // ended on the far pavement, a carriageway from the door (measured: 24.9 m from a civic entrance).
        auto sideLink = [&](int l, engine::Vec2 p) {
            if (l >= 0 && static_cast<std::size_t>(l) < twinOf_.size() && twinOf_[static_cast<std::size_t>(l)] >= 0 &&
                !nav_->links[static_cast<std::size_t>(l)].footpath &&
                dot(p - nav_->nodes[static_cast<std::size_t>(nav_->links[static_cast<std::size_t>(l)].from)],
                    engine::rightOf(nav_->direction(l))) < 0)
                return twinOf_[static_cast<std::size_t>(l)];
            return l;
        };
        auto stationOn = [&](int l, engine::Vec2 p) {   // metres along link l level with p
            const engine::NavLink& L = nav_->links[static_cast<std::size_t>(l)];
            const engine::Vec2 A = nav_->nodes[static_cast<std::size_t>(L.from)];
            return std::clamp(dot(p - A, nav_->direction(l)), Real(0), L.length);
        };
        const int dl = onFoot ? sideLink(nav_->nearestLink(dest), dest) : -1;
        auto routeFrom = [&](int fromNode, engine::Route& route, int& to, int& finalLink) {
            route = engine::Route{};
            to = -1;
            finalLink = -1;
            if (dl >= 0) {
                const engine::NavLink& L = nav_->links[static_cast<std::size_t>(dl)];
                if (L.from != fromNode) {
                    engine::Route r = engine::findRoute(*nav_, fromNode, L.from, true);
                    if (r.valid()) { route = std::move(r); to = L.from; finalLink = dl; }
                } else if (L.to != fromNode) {
                    engine::Route r = engine::findRoute(*nav_, fromNode, L.to, true);
                    if (r.valid()) { route = std::move(r); to = L.to; }
                }
            }
            if (to < 0) for (int cand : nearestNodesTo(dest, 8)) {
                if (cand == fromNode) continue;
                engine::Route r = engine::findRoute(*nav_, fromNode, cand, onFoot);
                if (r.valid()) { route = std::move(r); to = cand; break; }
            }
            return to >= 0;
        };
        engine::Route route;
        int to = -1;
        int finalLink = -1;
        bool alongOwnLink = false;   // the whole walk is along the link they stand beside
        if (legLink < 0 && onFoot && !a.moving) {
            // A WALKER AT REST (out of a door, out of a car) sets off along the pavement it stands on, from where it
            // stands (ADR-0415 as built). Routed from the nearest node, startTrip put it AT that node -- 19.5 m from
            // a front door in one jump -- and it then walked back past where it had been. Of the street's two links
            // it takes the one whose pavement it is on, ahead, or the other (across the street) back the other way:
            // whichever is the shorter walk.
            const int own = sideLink(nav_->nearestLink(a.pos), a.pos);
            Real best = std::numeric_limits<Real>::infinity();
            for (int k = 0; k < 2 && own >= 0; ++k) {
                const int c = k == 0 ? own
                              : (static_cast<std::size_t>(own) < twinOf_.size() && !nav_->links[static_cast<std::size_t>(own)].footpath)
                                  ? twinOf_[static_cast<std::size_t>(own)] : -1;
                if (c < 0) continue;
                const Real Lc = nav_->links[static_cast<std::size_t>(c)].length;
                const Real at = stationOn(c, a.pos);
                if (c == dl && stationOn(dl, dest) >= at) {   // the door is further along this very pavement
                    const Real cost = stationOn(dl, dest) - at;
                    if (cost < best) { best = cost; legLink = c; keepDist = at; alongOwnLink = true; }
                    continue;
                }
                engine::Route r;
                int t = -1, fl = -1;
                if (!routeFrom(nav_->links[static_cast<std::size_t>(c)].to, r, t, fl)) continue;
                const Real cost = (Lc - at) + r.length(*nav_) + (fl >= 0 ? stationOn(fl, dest) : Real(0));
                if (cost < best) {
                    best = cost; legLink = c; keepDist = at; alongOwnLink = false;
                    route = std::move(r); to = t; finalLink = fl;
                    from = nav_->links[static_cast<std::size_t>(c)].to;
                }
            }
        }
        if (!alongOwnLink && to < 0) routeFrom(from, route, to, finalLink);
        // NOTHING has been mutated yet, so a refusal really does leave the agent
        // its day. startTrip's no-path branch parks the agent, clears its route
        // and keeps the caller's goal — validating first is what makes this
        // function's contract ("the agent keeps its current plan") true.
        if (!alongOwnLink && to < 0) return false;
        if (alongOwnLink) {   // (the route is that one link: startTrip lays it, from its start)
            from = nav_->links[static_cast<std::size_t>(legLink)].from;
            to = nav_->links[static_cast<std::size_t>(legLink)].to;
            finalLink = -1;
        }
        // WAKE THEM. A sleeping agent is not in the active list at all, so the
        // stepper never reaches it: the errand sets moving, a valid route and a
        // destination, and the agent stands in its doorway until its OWN alarm
        // goes off — measured, with `stepped=0` and the ghost sitting 0.4 m
        // from its body, so neither the leash nor the route was to blame. A
        // director's errand overrides the day, and that includes the lie-in.
        a.wakeAt = -1;
        startTrip(a, from, to, /*fromRest=*/!a.moving);
        if (!a.route.valid()) return false;
        if (alongOwnLink) {
            a.route.links.assign(1, legLink);
            a.leg = 0;
            a.distOnLeg = keepDist;
            refreshPose(a);
            a.tickFromPos = a.pos;
            legLink = -1;   // (laid, not prepended)
        }
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
            a.tickFromPos = a.pos;   // (startTrip set the tick's start at the node: drawn from there, it slid back)
        }
        // END AT THE POINT, not at the node. Project the destination onto the
        // last link of the route: if it lies alongside it, that is where the
        // agent stops. A destination that is not beside its final street (a
        // door round the corner) keeps the old behaviour and arrives at the
        // node, which is the honest answer rather than a wrong one.
        // A DRIVE TO A RESERVED BAY (startTrip found one near the goal) ends AT the bay: the stepper stops it at the
        // bay's station, and a stop-short projected from the door could pull up short of the space.
        a.stopAtDist = -1;
        if (!a.route.links.empty() && a.targetBay < 0) {
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
        return nav_ ? nav_->nearestNodes(p, k) : std::vector<int>{};
    }

    // Sample agent `agentIndex`'s current route as a polyline of lane-centre
    // (driver) / sidewalk (pedestrian) points, ~`step` metres apart — the pursuit
    // path the ADR-0062 bridge follows from the car's real pose. Empty when the
    // agent has no active route.
    std::vector<engine::Vec2> lanePath(int agentIndex, Real step = 3.0) const;
    // THE WAY AN AGENT WILL BE POSED (ADR-0415, the trip lab's line): a copy of agent `agentIndex` moved along its
    // route `step` metres at a time, from where it is to where the route ends -- its stop-short point, or the station
    // of the bay it is driving to -- each copy placed by refreshPose, the one rule that puts a walker on its pavement
    // and a car in its lane, round the corners and through the junction turns, as the stepper places the real one.
    // With `route`, the copy is set off from that route's start instead, as `mode` (a leg not yet begun: a car in
    // lane 0). `visit` sees each copy in turn (the first where the walk begins). Nothing in the sim changes.
    void walkPoses(int agentIndex, Real step, const std::function<void(const Agent&)>& visit,
                   const engine::Route* route = nullptr, Agent::Mode mode = Agent::Mode::Pedestrian) const;

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
    // NO DAY SCHEDULE (citysim.schedule = false; the trip lab, ADR-0415): everyone is seeded at home -- a car owner's
    // car parked outside, the person indoors -- and held there for a director (`directed`), going only where they are
    // sent. Set before seedFromSchedule.
    void setDaySchedule(bool on) { daySchedule_ = on; }
    bool daySchedule() const { return daySchedule_; }
    // THE DRIVING (#234; a level that names citysim.drivingRules): on, the sim holds a driving::Driving and ticks it once
    // per sim tick. Until startDrivingV2 begins it, it ticks an empty world. Off drops it.
    void useDrivingV2(bool on) { if (on) drivingV2_.emplace(); else drivingV2_.reset(); }
    // ...and the kinds of vehicle it knows (#248: one class per fleet class, read with the fleet). No-op under v1.
    void setDrivingVehicleClasses(driving::VehicleClasses c) { if (drivingV2_) drivingV2_->setVehicleClasses(std::move(c)); }
    // ...and the road it drives on: the level's road model (road_model_cache.h; its lane level is where the cars are),
    // held here for as long as the driving reads it, with the one line that says what it holds and where it came from
    // (the load's log, `lanegraph?`). The driving is made again on the new road, keeping its vehicle classes; it has no
    // cars yet, so nothing else is lost. No-op under v1.
    void setDrivingRoads(std::shared_ptr<const engine::RoadModel> roads, std::string report);
    const std::string& drivingRoadsReport() const { return drivingRoadsReport_; }
    // The new model, read-only; null when the level drives v1.
    const driving::Driving* drivingV2() const { return drivingV2_ ? &*drivingV2_ : nullptr; }
    // ...and to place, send and carry cars of its own in (the near tier, city_near.h; ADR-0447). Null under v1.
    driving::Driving* drivingV2Mutable() { return drivingV2_ ? &*drivingV2_ : nullptr; }
    // WRITTEN BEFORE EACH TICK OF THE NEW MODEL (ADR-0447): where each car physics carries really is (the near tier's
    // Driving::carriedAt). Empty: nothing. One hook; a second setter replaces the first.
    void setBeforeDriving(std::function<void(driving::Driving&)> hook) { beforeDriving_ = std::move(hook); }
    // THE NEW MODEL BEGINS (#255, ADR-0453; city_driving.h): reads the rule set `rulesFile` and gives it to the model,
    // then makes `cars` cars that drive round for ever (the level's citysim.cars; city_car.h CityCarRoams), entities of
    // `world` with nobody in them, each numbered by nextCarNumber, drawn as the fleet slot the sim deals that number
    // (ambientSlotFor) and driven as that slot's driving class (`classOfSlot`: per slot, an index into the model's
    // vehicle classes, or -1 for the first); and the hand-over puts them down and sends them on trips from then on.
    // False, with the reason in `report`, when there is no new model or the rule set is refused; then nothing is made.
    // On success `report` says what was done. Call once, after build.
    bool startDrivingV2(engine::World& world, const std::string& rulesFile, int cars, const std::vector<int>& classOfSlot,
                        std::string* report);
    // THE NEXT FREE CAR NUMBER: past every row of the vehicle list and every car entity of `world` (city_car.h).
    uint32_t nextCarNumber(engine::World& world) const;
    // The hand-over's side of it: counts, costs, the measurements (`driving?`).
    const DrivingHost& drivingHost() const { return drivingHost_; }
    // WHERE THE CITY'S BODIES ARE EACH FIXED STEP (city_poses.h, ADR-0454): the car entities' CityCarPose, the walkers'
    // poses, and the ground and the drawn models' body facts they stand on and are fitted with.
    CityPoses& poses() { return poses_; }
    const CityPoses& poses() const { return poses_; }
    // THE ONE WAY THE PLAYER TAKES A CAR ENTITY (city_car.h; ADR-0455), with the key or by sitting in a car of the near
    // tier: whoever's it is, a lane car or a Jolt car. A household's car is told to the brain (CityBrain::takenByPlayer:
    // off the driving's road, its driver put out on the kerb side, the household told); one that drives round for ever
    // is told to the hand-over (DrivingHost::takenByPlayer: its trip ended, off the road, never sent again). Either way
    // CityCarLink::away says from then that the player has it. False: no such car, the player has it already, it is
    // nobody's to take (a test bed's car), or its owner refused (an outsider is in it). Defined in city_car.cpp.
    bool playerTakesCar(engine::World& world, engine::Entity car);
    // ...AND THE ONE WAY IT IS GIVEN BACK (#293), once it is no Jolt car any more (the near tier retires it when the
    // player has left it and it has stopped): the household has it again where it stands (CityBrain::carLeftAt). False:
    // the player does not have it, or it is not a household's (a car that drove round for ever is not given back yet).
    bool playerLeftCar(engine::World& world, engine::Entity car);
    // THE RESIDENTS WITH THE AGENT BRAIN (ADR-0441, ADR-0443; city_brain.h, where these are defined): the level's
    // markers, its residents and the households whose cars they share, driven by the brain library over this sim's
    // walking and the new driving. Call once, after startDrivingV2 has succeeded. False, with the reason in `report`,
    // when it cannot be set up; then the level has no such residents. `classOfSlot` as startDrivingV2's. The
    // households' cars are entities of `world` (ADR-0444; city_car.h). brain(): null when there is none.
    bool startBrain(engine::World& world, const std::vector<engine::CityMarker>& markers,
                    const std::vector<engine::CityResident>& residents, const std::vector<engine::CityHousehold>& households,
                    const std::vector<int>& classOfSlot, std::string* report, bool drivableSpacesOnly = false);
    CityBrain* brain() const { return brain_.get(); }
    // A DIRECTOR TAKES ONE OF THE BRAIN'S PEOPLE OVER, OR LETS THEM GO (ADR-0443; the possession channel,
    // city_possess.cpp): the brain's `possessed` state holds them (their intent kept, nothing moving them on) until
    // `on` is false, when they plan again from where they stand. Nothing happens for an agent the brain does not own.
    void brainNoteHeld(int agent, bool on);
    // Does this agent drive one trip straight into the next (the level's wander mode, or a roaming role's day)?
    // The chained-arrival corner, the final-leg turn yield and the no-parking rules ask this, not wander_.
    bool roams(const Agent& a) const {
        return wander_ || (a.role < roles_.roles.size() && roles_.roles[a.role].day.roam);
    }

    // The goal layer as data (ADR-0064): replace the per-archetype goal tables
    // (states + transitions over the C++ action vocabulary — city_goals.h).
    // Load-time only by design: call at level load (e.g. from tables authored
    // in agents.lua); every per-tick transition then runs in C++ from the
    // table. Agents are remapped onto the new table by their activity label
    // (first state with a matching label, else the entry state); a rebuild or
    // setWander() resets back to the built-ins.
    void setGoalTables(GoalTable pedestrian, GoalTable driver);

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

    // ON A TRAIN JOURNEY PAST THE STREET: in a station, on a platform, aboard, on the way out. The goal pass leaves
    // them alone (it would relaunch their trip on foot every tick).
    bool onTrainJourney(int i) const {
        const RailTrip* rt = rail_.tripOf(i);
        return rt && rt->phase != RailTrip::Phase::ToStation;
    }
    // TRAINS (city_rail_riders.h): the host's running RailSim, read each tick. Between two towns with a station
    // each, walkers go by train. Kept across a rebuild (the stations are re-pinned to the new streets).
    void setRail(const RailSim* rail);
    const RailRiders& railRiders() const { return rail_; }
    // Is this pedestrian ON THE STREET to be drawn (and given a body)? A
    // walker, someone pausing outside -- not someone indoors, in a car, or far. The walker system and the renderer both ask.
    bool pedVisible(int i) const;
    // THE ROLES (city_roles.h): every resident's day is its role's, built by buildDayTable from the catalog
    // (roles.lua at level load; defaultRoleCatalog otherwise). Re-seats every agent on its new table.
    void setRoleCatalog(RoleCatalog c);
    const RoleCatalog& roles() const { return roles_; }
    // THE PLACE KINDS (place_kinds.h): places.lua at level load, defaultPlaceKindCatalog otherwise. The PlaceMap's kind
    // indexes and tag bits are this catalog's: set it before the places are given their kinds.
    void setPlaceKinds(PlaceKindCatalog c) { placeKinds_ = std::move(c); }
    const PlaceKindCatalog& placeKinds() const { return placeKinds_; }
    // WHO LIVES HERE (people.lua): ages, life stages and household types; assignPlaces fills the dwellings with them.
    void setPeople(PeopleCatalog c) { people_ = std::move(c); }
    const PeopleCatalog& people() const { return people_; }
    // HOW TALL agent i is drawn against a grown adult (people.lua band heights, by age): 1 for anyone the plan did
    // not make (a bus or cab driver)
    Real bodyScale(int agentIndex) const {
        if (agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size())) return 1.0;
        const Agent& a = agents_[static_cast<std::size_t>(agentIndex)];
        return a.stage < 0 ? 1.0 : static_cast<Real>(people_.heightAt(a.age));
    }
    // The households assignPlaces housed the agents in (Agent::household indexes it), and what it came to.
    const HouseholdPlan& households() const { return households_; }
    std::string populationLine() const { return describePlan(people_, households_); }
    // Agents assignPlaces could give no household (more people than the housing holds, an authored count), drivers
    // of buses and cabs aside: they take a home by floor area, as everyone did before households.
    int unhousedCount() const { return unhoused_; }
    // One role's table replaced outright (a level script's own day for it); false when the role is unknown.
    bool setRoleTable(const std::string& role, GoalTable t);
    // Places people go out to (everything but homes), for outings and lunch.
    struct Venue {
        PlaceType type = PlaceType::Shop;
        int node = -1;
        engine::Vec2 door;
        Real openHour = 0, closeHour = 24;
        int16_t kind = -1;    // Place::kind (placeKinds()), and its tags
        uint32_t tags = 0;
        PlaceId place = kNoPlace;
        // THE HOURS SOMEBODY WORKS THERE (bit h: someone hired there is at work at h:30; indexVenueStaffing). A shop,
        // cafe, restaurant, bar, club or supermarket nobody is on shift at is CLOSED to customers, whatever its hours
        // say (Glenn's OPEN signs, made true). All bits set: not gated (a park, a civic hall, a place nobody was hired at).
        uint32_t staffed = 0xFFFFFFu;
        bool openAt(Real h) const {
            const int hr = static_cast<int>(std::floor(std::fmod(h + 24.0, 24.0))) % 24;
            if (!(staffed >> hr & 1u)) return false;
            if (openHour == closeHour) return true;
            if (openHour < closeHour) return h >= openHour && h < closeHour;
            return h >= openHour || h < closeHour;
        }
    };
    const std::vector<Venue>& venues() const { return venues_; }
    // How many agents assignPlaces made students (0 without a campus with a residence and a teaching hall).
    int studentCount() const { return students_; }
    // Does this agent's role spend the day OUT, not at work (roles.lua `commute = false`: a child, someone at home,
    // the retired)?
    bool daysOut(const Agent& a) const {
        return a.role < roles_.roles.size() && !roles_.roles[a.role].day.commute;
    }
    // A student: a resident of a hall's beds (people.lua's stage with beds) with a class to go to.
    bool isStudent(const Agent& a) const {
        return a.stage >= 0 && people_.stages[static_cast<std::size_t>(a.stage)].beds && a.workPlace != kNoPlace;
    }
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
        bool raised = false;   // Lie on furniture (a sun lounger): `hip` is its bed's height, not the ground
    };
    bool restPose(int agentIndex, RestPose& out) const;
    // EVERYTHING ABOUT ONE AGENT, on one line (the `who?` and `speeders?` debug commands): tier, where, how fast,
    // what it is doing (trip, seat, activity, session), its route leg and link, its rest, its leash.
    std::string describeAgent(int agentIndex) const;
    // THE CENSUS (Glenn: "Ridership feels sparse ... more people outside"): who is out right now, whatever their tier
    // -- walkers outside (moving, or standing out of doors) and moving, people on a train journey
    // past the street (`waiting`) -- in all and by role. O(agents): ask it now and then, not every frame.
    struct Census {
        int agents = 0, walkers = 0;
        int walkersOutside = 0, walkersMoving = 0, waiting = 0;
        static constexpr int kRoles = 16;   // by role id (roles().roles)
        int byRole[kRoles] = {}, outsideByRole[kRoles] = {};
        int atWorkByRole[kRoles] = {};   // resting in a state wearing AtWork (on shift, in class)
        int outing = 0;                   // wearing Outing: at or on the way to a stop, by day or by night
        // THE CARS WITH NOBODY IN THEM (city_car.h CityCarRoams; ADR-0453): no person stands for them, so they are
        // outside every count above: how many there are, how many the driving has on its road, and of those on a trip
        int emptyCars = 0, emptyCarsOnRoad = 0, emptyCarsDriven = 0;
    };
    // ...with the cars of `world` counted too (null: the people alone).
    Census census(engine::World* world = nullptr) const;
    // EVERY PERSON ON FOOT OUTDOORS, as a thing for the new driving's drivers to look for along their way (ADR-0451):
    // where they are, how wide (kWalkerHalfAcross) and how they move. A driver finds the ones in its way by itself, so
    // the pavement needs no telling apart from the road here. Then the things the host wrote (setExternalThings).
    void thingsForDriving(std::vector<driving::RoadThing>& out) const;
    // CROSSWALKS (ADR-0452). The crosswalks are the road model's (road_model.h ModelCrosswalk) and so is which a walk
    // round a junction crosses (RoadModel::crosswalksOnWalk, asked with the arms the walker's links arrive and leave by:
    // RoadModel::armReachedAt). A person on foot WAITS AT THE CROSSWALK'S KERB END (where its route comes level with it,
    // a step back) until they go: under lights, on WALK (a rare few, by a seeded trait, on DON'T WALK when the gap is a
    // jaywalker's); elsewhere, when no car that is not already slowing or stopped would reach the crosswalk inside the
    // gap they accept (a seeded trait: people.lua `crossing`). Once started they cross. Nobody arbitrates: a driver gives
    // way by its own rule to people on the crosswalk and waiting at it (driving/things.h).
    enum class CrossingKind : uint8_t { Lights, Signs, Marked, Unmarked, AgainstTheLights };
    static constexpr uint8_t kCrossingKindCount = 5;
    static const char* crossingKindWord(CrossingKind k);
    // THE NEXT CROSSING ON A WALKER'S WAY at the junction ahead (found: false: none; `governed`: the junction ahead has
    // the model's crosswalks, so they, not the old signals, rule its walkers there).
    struct CrossingAhead {
        bool governed = false, found = false;
        uint32_t crosswalk = 0;
        int fromEnd = 0;
        Real toHold = 0;   // route metres to where they wait: level with the kerb end they step off from, a step back
        Real toFar = 0;    // ...to where they are across: level with the far kerb end
    };
    CrossingAhead crossingAhead(const Agent& a) const;
    // Whether a person at crosswalk `crosswalk` would step off now (the gap they accept, against the cars they see),
    // `bigger`: a jaywalker's gap; `waited`: how long they have waited (a longer wait makes them bolder).
    // `why` (optional, out): what holds them, in words (empty when they go).
    bool gapToCross(const Agent& a, uint32_t crosswalk, Real waited, bool bigger, std::string* why = nullptr) const;
    // One of the rare few who cross against the lights (people.lua crossing.jaywalk_share, a hash of who they are).
    bool jaywalks(const Agent& a) const;
    // THE COUNTS: agent-ticks waiting at kerbs; crossings begun, by kind. (Both hesitating -- a driver standing for a
    // person who waits at the kerb -- is the driving's count: Driving::ThingCounts::crosswalkStandOffs.)
    struct WalkStats {
        uint64_t waitingTicks = 0;
        std::array<uint64_t, kCrossingKindCount> crossings{};
    };
    // A walker held this near (m of route) the place they wait at a crosswalk is waiting at its kerb.
    static constexpr Real kCrossingAtKerb = 3.0;
    // ...and, slower than this (m/s), stands at the crosswalk's kerb end (ModelCrosswalk::kerbWait).
    static constexpr Real kCrossingStanding = 0.05;
    // How far behind a crosswalk's kerb end, on the pavement, a person waits (m): a step.
    static constexpr Real kCrossingKerbStep = 0.6;
    // The most (m) a waiting walker steps aside from its route's line to stand at the kerb end (a safety bound: it is a
    // few metres where the route passes the crosswalk).
    static constexpr Real kCrossingStepAside = 10.0;
    const WalkStats& walkStats() const { return walkStats_; }
    // People waiting at a kerb right now, and crossing right now.
    int walkersWaitingAtKerbs() const;
    int walkersOnCrosswalks() const;
    static constexpr Real kWalkerHalfAcross = 0.3;   // m: half a person's shoulders
    // A junction's box radius (the stop lines and a walker's wait are measured from it).
    Real junctionBox(int node) const { return junctionRadius(node); }
    // A group member's place in its session (formations other than Roam) and the way it faces there.
    engine::Vec2 memberPlace(const Session& s, int agentIndex, engine::Vec2* face) const;
    // 1 while a departing car is still drawn at its parking space, easing to 0
    // once it has merged into its lane (see Agent::pullOffset).
    // `pullS`: how far into the pull the DRAWN car is (interpolated through
    // the sim tick by the renderer, like its position).
    static Real pullOutWeight(const Agent& a, Real pullS);

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
    // of EVERYTHING it reads (the nav graph, the places, each agent's state before assignment, the shares, the
    // format). A later load with the same inputs reads it instead of deciding
    // again. "" (the default) = no cache. RT_POPULATION_VERIFY=1 decides anyway and reports any field that
    // differs from the cache.
    void setPopulationCacheDir(std::string dir) { populationCacheDir_ = std::move(dir); }
    struct PopulationCacheReport { bool used = false, hit = false, saved = false; uint64_t key = 0; int mismatches = -1; };
    const PopulationCacheReport& populationCache() const { return popCache_; }
    // Share of drivers assignPlaces gives a CROSS-TOWN job (1.2 km+ away); the rest
    // take the nearest of their sampled jobs. Set before assignPlaces.
    void setLongCommuteShare(Real share) { longCommuteShare_ = share; }
    // What the last assignPlaces did with the drivers' jobs (for the load log).
    struct CommuteStats {
        int crossTownDrivers = 0, driversWithJobs = 0; Real meanDriverCommute = 0;
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
    // THE THINGS ON THE ROAD THE WORLD OUTSIDE THE SIM KNOWS (ADR-0451): the player on foot, a car the player has, a
    // body set down -- written each step by the host; the new driving's drivers look for them along their way, with
    // the sim's own people on foot (walkersOnRoad).
    void setExternalThings(std::vector<driving::RoadThing> things) { externalThings_ = std::move(things); }
    void setExternalObstacles(std::vector<engine::Vec2> obstacles, std::vector<Real> halfLengths = {}) {
        externalObstacles_ = std::move(obstacles);
        externalHalf_ = std::move(halfLengths);
        externalHalf_.resize(externalObstacles_.size(), Real(0));
    }
    // THE PHYSICAL TIER'S BODIES OFF THEIR GHOSTS (#23; CI's packed junction, 2026-10-22): a possessed car's chassis
    // that a crash left metres from its ghost is where the car IS, and the sim's drivers never saw it -- a passing car's
    // kinematic box drove into it and laid it on its side. The bridge reports each such body (with the agent it
    // belongs to, which does not brake for itself), and drivers hold short of it as of the player's car.
    void setBodyObstacles(std::vector<engine::Vec2> at, std::vector<Real> halfLengths, std::vector<int> owners) {
        bodyObstacles_ = std::move(at);
        bodyHalf_ = std::move(halfLengths);
        bodyOwner_ = std::move(owners);
        bodyHalf_.resize(bodyObstacles_.size(), Real(2.15));
        bodyOwner_.resize(bodyObstacles_.size(), -1);
    }
    std::size_t bodyObstacleCount() const { return bodyObstacles_.size(); }

    // World-space (XZ) static obstacles pedestrians steer around and never stand
    // inside — chiefly the signal poles on the sidewalks (injected once by the host
    // from the render bridge). Cars ignore these (poles sit off the carriageway).
    // STATIC OBSTACLES a walker steers round and is never inside: the signal poles (setStaticObstacles), and the
    // street's furniture -- a bus stop's pole and bench, a cafe's tables, a bench (Glenn: "the npcs probably need to
    // recognize vehicles and furniture as obstacles to avoid"), each with its own clearance (addStaticObstacles).
    // Indexed by 8 m cell: a walker reads only the cells round it.
    void setStaticObstacles(std::vector<engine::Vec2> poles);
    void addStaticObstacles(const std::vector<engine::Vec2>& at, Real clearance);
    std::size_t staticObstacleCount() const { return staticObstacles_.size(); }

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
        if (wander_) return a.archetype == Agent::Mode::Driver ? goalDriver_ : goalPed_;
        return roleTables_[static_cast<std::size_t>(dayRoleOf(a))];
    }
    GoalTable& tableFor(const Agent& a) { return const_cast<GoalTable&>(static_cast<const CitySim&>(*this).tableFor(a)); }
    // The role whose day an agent runs: its own -- except a day out with none to take (one who drives, or a city
    // with nowhere to go), whose park is its "work": the catch-all job's day (the historical park-as-destination).
    int dayRoleOf(const Agent& a) const {
        const int r = static_cast<int>(a.role);
        if (r < 0 || r >= static_cast<int>(roleTables_.size())) return catchAllRole_;
        if (daysOut(a) && (a.archetype != Agent::Mode::Pedestrian || venues_.empty())) return catchAllRole_;
        return r;
    }
    bool outingStroller(const Agent& a) const { return !wander_ && daysOut(a) && dayRoleOf(a) == static_cast<int>(a.role); }
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
    // A walker on a STREET's pavement (not a walk, not a deck), clear of the junctions at either end where its
    // crossings start: its road is on its left, and it does not step into it to get round anyone.
    bool onStreet(const Agent& a) const {
        if (!nav_ || a.mode != Agent::Mode::Pedestrian || a.leg < 0 || a.leg >= static_cast<int>(a.route.links.size()))
            return false;
        const engine::NavLink& L = nav_->links[static_cast<std::size_t>(a.route.links[static_cast<std::size_t>(a.leg)])];
        if (L.footpath || L.layer != 0 || L.elevAbsolute) return false;
        return a.distOnLeg > junctionRadius(L.from) + 1.0 && a.distOnLeg < L.length - junctionRadius(L.to) - 1.0;
    }
    void reseatOnTables();      // each agent onto its table's state wearing its label (else the entry)
    bool launchClear(const Agent& a, int node) const;   // no moving car near the spawn
    // The nearest FREE bay to `target` within `maxDist`, claimed for agent
    // `self` (-1 if none). The one allocator for a car at load and on arrival.
    int claimBayNear(engine::Vec2 target, int self, Real maxDist);
    void releaseBays(Agent& a);          // free both the held and the reserved bay
    std::vector<int> nearestFreeBays(engine::Vec2 target, Real maxDist, int k, bool garages = true) const;
    bool fitsAGarage(int agentIdx) const;   // its car is a car (not a truck): under 5.6 m
    bool parksInBays(const Agent& a) const;   // a private car that parks (not a wanderer)
    void advance(Agent& a, Real dt, Real gap, Real minGap);
    // advance()'s junction verdict: the speed target after the signal brake and
    // the box-occupancy / turn-yield scan, plus where the stop line is -- in
    // ROUTE metres from the car, because the line need not be on the current
    // leg (see junctionAhead).
    struct JunctionGate {
        Real cap = 0;               // speed target after junction/signal/yield caps
        bool yieldAtLine = false;   // holding for box/oncoming traffic/exit room
        uint8_t why = 0;            // ...which: 1 a crossing car in the box, 2 oncoming (a turn), 3 no exit room
        int node = -1;              // the junction ahead (-1: none within reach)
        int approachLink = -1;      // the route link that enters it (its signal)
        Real distToNode = 0;        // route metres to the node
        Real distToLine = 0;        // route metres to the stop line (< 0: past it)
        // A WALKER AT THE MODEL'S CROSSWALKS (ADR-0452): they rule it here (`crossing.governed`: not the old signals),
        // the crossing ahead, and whether it holds them at its kerb end now (distToLine is then its toHold)
        CrossingAhead crossing;
        bool walkHold = false;
        CrossingKind kind = CrossingKind::Unmarked;   // how they would cross it, going now
    };
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
        if (id >= 0) return 0;
        if (k < static_cast<int>(externalHalf_.size())) return externalHalf_[static_cast<std::size_t>(k)];
        const int b = k - static_cast<int>(externalHalf_.size());   // the bridge's bodies follow the host's points
        return b < static_cast<int>(bodyHalf_.size()) ? bodyHalf_[static_cast<std::size_t>(b)] : Real(0);
    }
    // The agent an injected body belongs to (a possessed car's own chassis), else -1
    int externalOwnerOf(int id) const {
        const int b = -id - 1 - static_cast<int>(externalHalf_.size());
        return id < 0 && b >= 0 && b < static_cast<int>(bodyOwner_.size()) ? bodyOwner_[static_cast<std::size_t>(b)] : -1;
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
    void railPass(Real dt);                        // walks in and out, boarding, riding (city_rail_riders.h)
    void enterStation(Agent& a, int self);
    void tickV(int i, Real hoursPerSecond);
    bool clearPromotion(int i);   // seat a promoted car behind live traffic
    // Advance a V agent along its REAL route at the modelled speed: link class
    // speed shaped by fixed average junction delays (vHold). No sensing, no
    // FSM, no signal state (ADR-0456: the old clock is gone; every junction crossed costs kVJunctionDelay).
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
    void refreshPose(Agent& a) const;   // (const: it places `a` and reads the sim, so walkPoses can pose copies)
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
    std::vector<driving::RoadThing> externalThings_;   // host-injected things on the road (setExternalThings)
    WalkStats walkStats_;                              // crosswalks (ADR-0452)
    std::vector<driving::RoadThing> thingsScratch_;
    std::vector<Real> externalHalf_;                // parallel: a vehicle's half length, 0 = a person
    std::vector<engine::Vec2> bodyObstacles_;       // the physical tier's chassis off their ghosts (setBodyObstacles)
    std::vector<Real> bodyHalf_;
    std::vector<int> bodyOwner_;
    struct StaticObstacle { engine::Vec2 p; Real clearance; };
    std::vector<StaticObstacle> staticObstacles_;     // host-injected, static (setStaticObstacles / addStaticObstacles)
    std::unordered_map<int64_t, std::vector<int>> obstacleCells_;   // by 8 m cell
    Real obstacleMaxClearance_ = 0;
    template <typename F> void forObstaclesNear(engine::Vec2 p, Real r, F&& f) const {
        if (staticObstacles_.empty()) return;
        constexpr Real kCell = 8.0;
        const int64_t x0 = static_cast<int64_t>(std::floor((p.x - r) / kCell)), x1 = static_cast<int64_t>(std::floor((p.x + r) / kCell));
        const int64_t z0 = static_cast<int64_t>(std::floor((p.y - r) / kCell)), z1 = static_cast<int64_t>(std::floor((p.y + r) / kCell));
        for (int64_t x = x0; x <= x1; ++x)
            for (int64_t z = z0; z <= z1; ++z) {
                auto it = obstacleCells_.find((x << 32) ^ (z & 0xffffffff));
                if (it == obstacleCells_.end()) continue;
                for (int k : it->second) f(staticObstacles_[static_cast<std::size_t>(k)]);
            }
    }
    std::vector<std::pair<engine::Vec2, Real>> junctions_;   // centre + box radius
    std::vector<Real> nodeBoxRadius_;   // per node: widest incident half-width (+ pad)
    Real junctionPad_ = 0;              // + this at street intersections (see setJunctionPad)
    // P4.1 spatial index. grid_ holds every agent (K re-hashed each step —
    // effectively free, place() early-outs on an unchanged cell; V only on its
    // coarse tick, when its position actually moves). junctionGrid_ is a static
    // bake of junctions_ so nearJunction() stops scanning the whole list.
    AgentGrid grid_;
    AgentGrid carGrid_;               // the K tier's cars only, rebuilt each tick (stepTick: the walkers' car checks)
    std::vector<int> carGridIds_;
    AgentGrid pedGrid_;               // the K tier's walkers out on the street, rebuilt each tick (the overlap solver)
    std::vector<int> pedGridIds_;
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
    RailRiders rail_;
    const RailSim* railSrc_ = nullptr;
    std::string populationCacheDir_;      // setPopulationCacheDir
    PopulationCacheReport popCache_;
                                              // never nest across a live iteration)
    std::vector<int> tierScratch_;            // tierPass promotion candidates
    std::vector<int> dormantScratch_;         // tierPass wake candidates
    int dormancies_ = 0;
    int wakes_ = 0;
    // THE ROLES and each one's day (setRoleCatalog); the wanderers' tables (the agent lab) beside them.
    RoleCatalog roles_ = defaultRoleCatalog();
    PlaceKindCatalog placeKinds_ = defaultPlaceKindCatalog();
    PeopleCatalog people_ = defaultPeopleCatalog();
    HouseholdPlan households_;
    int unhoused_ = 0;
    uint32_t seed_ = 1;   // build()'s: the households' order
    int catchAllRole_ = 0;   // roles_.catchAllJobRole(), kept with the role tables
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
    // THE EVENING JOBS (roles.lua side_job): each holder of a role with one, by its share, takes one of the four nearest
    // venues of its kinds open through its shift, within reach of home. Before indexVenueStaffing: their shifts staff it.
    void assignSideJobs();
    int sideJobs_ = 0;
    // Venue::staffed from the hired staff's work windows (their schedules: at work from departHome + the commute to
    // their work departure). Counts of the result in venueStaffing_.
    void indexVenueStaffing();
public:
    struct VenueStaffing { int gated = 0, neverStaffed = 0, staffedNow = 0; };
    VenueStaffing venueStaffing(Real clock) const;
private:
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
    Real hoursPerSecond_ = 1.0 / 3600.0;   // (real time)
    Real staffingRate_ = 0;   // the clock rate Venue::staffed was worked out at (indexVenueStaffing)
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
    bool daySchedule_ = true;   // false: everyone at home, held for a director (setDaySchedule)
    std::optional<driving::Driving> drivingV2_;   // the new driving model, when the level asks for it (useDrivingV2)
    // Its road (setDrivingRoads): shared, so its address stays put while the model holds a pointer to it.
    std::shared_ptr<const engine::RoadModel> drivingRoads_;
    std::string drivingRoadsReport_ = "no lane graph yet";
    DrivingHost drivingHost_;   // the hand-over to the new model (startDrivingV2); inactive on a v1 level
    CityPoses poses_;           // where the bodies are each fixed step (poses())
    std::function<void(driving::Driving&)> beforeDriving_;   // setBeforeDriving
    // The resident with the agent brain and everything it needs (startBrain; city_brain.h). It reads and moves this
    // sim's agents directly, as the hand-over above does, so it is a friend. Null: none.
    friend class CityBrain;
    std::shared_ptr<CityBrain> brain_;
    void tickBrain(Real dt);    // one tick of it, after the driving's (defined in city_brain.cpp)
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
    // through traffic: a sleeper under way is moved on every few seconds (a waypoint event) so the wake finds it
    static constexpr Real kDormantWaypointSeconds = 4.0;
    std::vector<double> dormantArriveAt_;   // per agent: when its trip ends (its arrival event), -1 none
    long dormantWaypoints_ = 0;
    void dormantGlide(Agent& a, Real seconds);
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
    engine::ProcRng rng_{1};
    // Hands out agent UIDs (ADR-0066): reset at build, then one per agent in
    // order. Separate from rng_ so identity allocation never perturbs the sim's
    // deterministic draw stream.
    AgentIdAllocator ids_;
};

}  // namespace citysim

#endif
