#ifndef RAYTRACER_APPS_CITYSIM_CITY_RENDER_H
#define RAYTRACER_APPS_CITYSIM_CITY_RENDER_H

#include "../../engine/system.h"
#include "../../engine/ai/nav_graph.h"
#include "../../engine/components.h"   // engine::AuthoredPlace (level-authored places)
#include "city_meshes.h"   // buildPersonMesh, materials (was declared here)
#include "city_sim.h"
#include "places.h"        // PlaceMap (ADR-0066)
#include <functional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace citysim {

// The ECS render bridge for the agent-based city simulation (ADR-0060 Phase 6).
// It builds a NavGraph from the level's RoadEntity entities, runs a deterministic
// CitySim of driver+pedestrian agents over it, and bakes their poses into
// InstanceGroups so RenderSystem draws the whole city as a few instanced
// batches: one for cars, one for pedestrians, and one per signal state (red /
// yellow / green) whose emissive lenses light up to show each stoplight phase.
//
// This is the APPLICATION layer (apps/citysim). It depends on the core engine
// (World, components, AssetManager) but the core never depends on it. The
// reusable primitives it stands on — NavGraph, A*, perception — live in core.
//
// build()/step() take a World directly (like TrafficSystem) so the spawn +
// pose-bake logic is unit-tested headless; mesh upload is the only part that
// needs the AssetManager (skipped, with null mesh handles, when absent).
struct CityRenderParams {
    int cars = 40;          // -1 = by density (carsPerLaneKm over the nav)
    int pedestrians = 40;   // -1 = by density (pedsPerKm over the sidewalks)
    Real carsPerLaneKm = 10.0;
    Real pedsPerKm = 6.0;
    Real longCommuteShare = 0.0;   // CitySimConfig::longCommuteShare
    Real busCommuteShare = 0.0;    // CitySimConfig::busCommuteShare
    // Ceiling on each density-derived count (CitySimConfig::maxAmbient). The
    // bridge used to clamp at a literal 400, which halved metro v2: its own
    // network asks for 769 cars and 788 walkers.
    int maxAmbient = 400;
    int nearTarget = 0;                    // CitySimConfig::nearTarget
    uint32_t seed = 1;
    Real hoursPerSecond = 0.05;            // sim-clock hours advanced per real second
    // The in-world hour the level OPENS at. The population is placed directly
    // from its schedules at this hour (CitySim::seedFromSchedule), so any hour
    // costs the same: 22:00 is as cheap as 08:00. 10.5 keeps the historical
    // feel of the retired warm-up, which landed around mid-morning.
    Real startHour = 10.5;
    Real perceptionReliability = 0.97;     // <1 -> agents occasionally err (ADR-0060)
    engine::Vec3 carSize{1.8, 1.3, 4.2};   // matches the player sedan (W,H,L); +Z = travel
    engine::Vec3 pedSize{0.5, 1.8, 0.5};
    Real signalLensSize = 0.28;            // lit emissive lens: a plate this wide, over the head's own lamp (m)
    // How far from the player a PARKED scenery car still draws (m). A city-wide
    // instance group cannot be partially frustum-culled, so without this every
    // parked car in the city is submitted every frame, in both the colour and
    // shadow passes. 0 = no cull (the old behaviour).
    Real sceneryRadius = 450.0;
    // Physical-walker budget; see CityWalkerSystem. 0 or negative = unbounded
    // (the historical behaviour, kept so a measurement can A/B against it).
    int maxWalkerBodies = 200;
    // How far pedestrians promote into the drawn/simulated near tier and fall
    // back out (hysteresis). What is DRAWN is bounded by this, not by the
    // population: ~350 agents stood near the player at every hour on metro v2
    // whatever the density, because past the demote radius an agent has no
    // render membership at all. Drawn count scales with AREA, so 280 -> 400 is
    // ~2x, and near-tier agents are the expensive ones.
    Real pedPromoteRadius = 280.0;
    Real pedDemoteRadius = 350.0;
    // Cabs on the street, and how readily a walker hails one (0 = off).
    Real taxiFraction = 0.0;
    Real hailChance = 0.0;
    Real hailMinMetres = 400.0;
    int busRoutes = 0;
    int busStops = 8;
    int buses = 0;
    Real busMaxWalk = 120.0;
    Real busServiceStart = 5.5, busServiceEnd = 23.5;   // CitySimConfig: with a depot, the buses' day
    // Cars handed to real physics near the player (CitySimConfig::physicalCars).
    // 0 = none, the shipping default; the physics soaks opt in.
    int physicalCars = 0;
    // SIM RATE (P8.2d). Traffic is not physics: agents follow lanes, so a
    // bigger dt costs nothing but precision. 0 or >= the fixed rate keeps the
    // historical every-step tick (and every existing test/gate bit-identical);
    // 30 halves the sim's cost outright. Poses are EXTRAPOLATED along heading
    // between ticks, so the renderer and the kinematic proxies still see
    // smooth 60 Hz motion. `adaptiveRate` lets the sim dip FURTHER (never
    // below ~7.5 Hz) while the clock is behind — catching up by simulating
    // coarsely instead of by running ever more expensive steps, which is the
    // spiral that pins the frame at the step cap.
    // MEASURED CAVEAT (P8.2d): enabling this on piedmont REGRESSED the frame
    // (16 -> 9 fps). The far tier's coarse tick is `frameIndex_ % vTickDivisor`
    // — a count of SIM TICKS, documented as "~1 Hz at the 60 Hz step" — so
    // halving the sim rate also halves how often distant agents refresh their
    // position. They then read stale near the bubble edge and the tier pass
    // over-promotes: K/P went 950 -> 2650, and K agents are the ones that get
    // DRAWN (instanced draws 25 -> 60, GPU 32 -> 86 ms). Express the V budget
    // in sim SECONDS before turning this on.
    Real localHz = 0.0;
    bool adaptiveRate = true;
    bool debugWidgets = false;             // draw each agent's footprint + trajectory
    bool wander = false;                   // perpetual random trips (the agent lab)
    bool ambientBus = false;               // ambient rotation includes the bus body (wheel_lab)
    // Scripted goal tables (ADR-0064): the SOURCE of an agents.lua-style script
    // whose archetype tables replace the sim's built-ins at build. Loaded from
    // the level's citysim block; used only in scripting builds; "" = built-ins.
    std::string agentScript;
    // Data-driven fleet bodies (ADR-0065): the SOURCE of a vehicles.lua-style
    // script whose `vehicle.fleet` recipes build the instanced car meshes at
    // build. Loaded from the level's citysim block; used only in scripting
    // builds. Absent or empty means this level draws NO cars — vehicles are
    // optional content, and there is no built-in substitute.
    std::string vehicleScript;
    std::string fleet;   // a named fleet of that script (selectFleet); "" = its default
    // Three-tier traffic (P4): opt this level into the V/K bubble — far agents
    // become persistent coarse-tick "virtual" travellers (no render, no proxy,
    // no sensing) promoted back to full kinematic agents near the player. Off
    // by default so every existing level/test runs bit-identically. (The
    // level_loader "tiered" JSON knob is the pending one-line hookup.)
    bool tieredAgents = false;
    // P5 dormancy (see components.h). Needs tieredAgents to do anything.
    bool dormantAgents = false;
};

// The Dear ImGui window this system appends its "Living City" section into. It
// MUST match engine DebugOverlaySystem's ImGui::Begin(...) title: ImGui merges
// same-titled Begin() calls within a frame into ONE window, so the city section
// stacks inside the existing Debug panel instead of floating separately. This
// string is the ONLY coupling between the engine's debug panel and the city app
// — deliberately a shared literal, not a code dependency (which would reinstate
// the engine→app arrow the citysim library extraction removed).
inline constexpr const char* kDebugWindowTitle = "Debug";

class CityRenderSystem : public engine::System {
public:
    const CityRenderParams& params() const { return params_; }
    explicit CityRenderSystem(const CityRenderParams& params = {})
        : params_(params), debugWidgets_(params.debugWidgets) {}

    void onStart(engine::FrameContext& ctx) override;
    void update(engine::FrameContext& ctx) override;   // per-frame: debug-widget toggle
    void fixedUpdate(engine::FrameContext& ctx) override;
    // The "Living City" debug section (ADR-0066). Appends into the shared Debug
    // ImGui window (see kDebugWindowTitle) — per-layer widget toggles + a
    // selected-agent inspector. Guarded by RT_ENABLE_IMGUI + a null-context
    // check + "a city is loaded", so it draws only when a living city is present.
    void render(engine::FrameContext& ctx) override;

    // Debug widgets (rings / vision cones / navgraph) show/hide state. The `j`
    // action flips it in update(); this setter lets another system force it (the
    // spectate camera turns it on so the followed agent shows its ring + cone).
    void setDebugWidgets(bool on) { debugWidgets_ = on; }
    bool debugWidgets() const { return debugWidgets_; }

    // Rebuild-road-graph button (device: "a button to rebuild the road graph on
    // the terrain"). The panel sets a request; the host (ArenaState) polls this
    // each frame and, when true, reseeds the road recipe on disk and reloads the
    // level — so roads, terrain conform, and buildings all regrow consistently
    // from the new graph. Returns true once, then clears.
    bool consumeRebuildRoadsRequest() {
        bool r = rebuildRoadsRequested_;
        rebuildRoadsRequested_ = false;
        return r;
    }

    // Agent `agentId`'s current WORLD pose for spectating: the real external body
    // pose when one is reported (cars/peds owned by the physics bridges), else the
    // sim ghost (pos + groundAt + elevation). `outPos` is the body ORIGIN on the
    // ground (y = terrain + elevation); `outHeading` is its XZ heading. Returns
    // false for an out-of-range / released / unreported (no external body) agent.
    bool agentWorldPose(int agentId, engine::Vec3& outPos,
                        engine::Vec2& outHeading) const;

    // When a CityVehicleSystem owns the NPC cars as real physics Vehicles (ADR-0062),
    // this render bridge must NOT also draw them as instanced kinematic boxes (that
    // would double every car). Call before build(): it skips creating/baking the car
    // instance groups; peds, signals, and crosswalks stay owned here. The CitySim
    // still runs as the PLANNER (its ghosts drive the AgentDriver commands).
    void setCarsExternallyOwned(bool on) { carsExternallyOwned_ = on; }
    bool carsExternallyOwned() const { return carsExternallyOwned_; }

    // Same handoff for PEDESTRIANS (ADR-0062): when a CityWalkerSystem owns them
    // as physics characters, this bridge stops baking the instanced ped boxes
    // (they'd draw twice) — the CitySim keeps planning; walkers follow bodies.
    void setPedsExternallyOwned(bool on) { pedsExternallyOwned_ = on; }
    bool pedsExternallyOwned() const { return pedsExternallyOwned_; }

    // The physical car poses (ADR-0062), fed each step by the vehicle bridge so
    // the per-agent DEBUG WIDGETS ring the REAL car — not the planner ghost, which
    // legitimately runs ahead/behind (an empty ring on the ground is the ghost).
    // Only used when cars are externally owned; drivers without a reported pose
    // (released to the player, dead entity) draw no widget.
    struct ExternalAgentPose {
        int agentId = -1;
        engine::Vec2 pos;
        engine::Vec2 heading{1, 0};
        // Where the agent is TRYING to go right now (pursuit lookahead point /
        // planner ghost) — the debug arrow points here, visualising intent.
        engine::Vec2 target;
        // Optional BODY-truth state for the ring colour (an Agent::State value):
        // e.g. a walker physically blocked or knocked down shows red even while
        // its planner ghost thinks it's walking. -1 = use the ghost's state.
        int stateOverride = -1;
    };
    void setExternalCarPoses(std::vector<ExternalAgentPose> poses) {
        externalCarPoses_ = std::move(poses);
    }
    void setExternalPedPoses(std::vector<ExternalAgentPose> poses) {
        externalPedPoses_ = std::move(poses);
    }

    // --- testable core (no FrameContext) -----------------------------------
    // Build the NavGraph from every RoadEntity in `world`, seed the CitySim, and
    // create the instance-group entities. `assets` may be null (tests): then the
    // groups carry a null MeshHandle and only the transforms are populated.
    // No-op (returns false) if the world holds no navigable roads.
    // `ground` (optional) is the terrain sampler for worlds WITHOUT a CDLOD
    // terrain config — loader-less test worlds on a slope pass theirs here
    // (roads no longer store one); a level's TerrainLodConfig still wins.
    bool build(engine::World& world, engine::AssetManager* assets,
               std::function<double(double, double)> ground = nullptr);

    // Advance the sim by `dt` seconds and re-bake every InstanceGroup.
    void step(engine::World& world, Real dt);

    // THE WORLD CLOCK — lockstep with the day/night cycle (device: "I'd like
    // to keep the simulation in lock step with day night"). Before build: the
    // hour the city opens at. After: the rate its schedules run at, and a
    // JUMP — the sky landed more than a few in-world minutes from the sim's
    // clock — re-places the whole population from its schedules at the new
    // hour, exactly as the level opens (`daynight 22` IS "open the city at
    // 22:00"; promoted/possessed cars are skipped, as at build). Rate 0 (a
    // held sky) holds the clock while the city keeps moving. Returns true
    // when the population was re-seeded (the host re-bakes the groups).
    bool setWorldClock(Real hour, Real hoursPerSecond);

    bool built() const { return built_; }
    const CitySim& sim() const { return sim_; }
    // The player is aboard a sim vehicle (CityPlayerTransitSystem): stop
    // feeding their body to the sim as an obstacle, or the vehicle they are
    // riding brakes for them.
    void setPlayerRidingAgent(int agent) {
        playerRidingAgent_ = agent;
        playerRiding_ = agent >= 0;
    }
    int playerRidingAgent() const { return playerRidingAgent_; }
    // Where a passenger STANDS on this bus, in world space: the aisle by the
    // middle door. False when the agent is not drawn as a see-into vehicle.
    bool busStandingSpot(int agent, engine::Vec3* world) const;
    // A PASSENGER'S FRAME (Glenn: "Should the player be able to move freely
    // around the bus interior and find a seat?"). The bus's drawn pose -- the
    // exact matrix its body is drawn with this step -- and its saloon layout
    // in that body frame: seat hip points, door floor points, floor height.
    // False when the agent is not a see-into vehicle.
    bool busFrame(int agent, engine::Mat4* pose) const;
    // A CAR'S SEATS (Glenn, 2026-10-02: "a way to detect which part of the car you get into so you can be a
    // passenger (backseat, frontseat passenger) or the driver ... we'd have to be pointing at the seat that we
    // want"): an ambient car's drawn pose and its seat hips in the body frame -- [0] the driver's, [1] the front
    // passenger's (the driver's mirrored), then the two rear seats when the body is long enough to have them.
    // False for a bus, a parked or released agent, or a body that publishes no driver seat.
    bool carSeatsOf(int agent, engine::Mat4* pose, std::vector<engine::Vec3>* seats) const;
    const std::vector<engine::Vec3>& busSeats() const;
    const std::vector<engine::Vec3>& busDoors() const;
    engine::Real busFloorY() const;   // body-local floor height
    // True if an NPC rider is drawn in `seat` on this bus right now.
    bool busSeatTaken(int agent, int seat) const;
    static int riderSeat(int agent, int j, int n);
    void setPlayerSeat(int agent, int seat) { playerSeatAgent_ = agent; playerSeat_ = seat; }
    // R5 physical tier (roads-v2.1): agents whose RENDER truth is a Jolt
    // vehicle body. CityPhysicsSystem drives the bodies from the sim's own
    // plan and writes each pose here; syncGroups bakes it instead of the
    // kinematic agentPose, and flags the instance so its kinematic proxy
    // box parks instead of tracking (a box inside the dynamic chassis would
    // fight it).
    void setAgentPhysPose(int agent, const engine::Mat4& pose) {
        physPose_[agent] = pose;
    }
    void clearAgentPhysPose(int agent) { physPose_.erase(agent); }
    // Bake-order agent id per car instance (parallel to each carGroup's
    // transforms), so the physics bridge can park the kinematic proxy of a
    // possessed agent THE SAME TICK it acquires the body (a bake-time flag
    // would lag one tick and the box would kick the fresh chassis).
    const std::vector<std::vector<int>>& carAgentIds() const {
        return carAgentIds_;
    }
    // Same contract for the (single) pedestrian group, added for P4: the
    // physics bridge's incremental proxy diff keys ped boxes by agent uid, so
    // it needs each baked instance's agent — with V-tier walkers unbaked, the
    // instance list is no longer "all pedestrians in agent order".
    const std::vector<std::vector<int>>& pedAgentIds() const {
        return pedAgentIds_;
    }
    Real groundHeightAt(Real x, Real z) const { return groundAt(x, z); }
    // Where an agent is DRAWN this step: one tick behind, interpolated through the sim's tick (the same rule the
    // crowd's instances follow -- agentPose). Between ticks the raw position holds still and then hops; at a slowed
    // tick (the adaptive rate, down to 7.5 Hz) a walker posed from it read as flickering at superspeed.
    engine::Vec2 drawnAgentPos(int agentIndex, engine::Vec2* heading = nullptr) const;
    // Is anyone in this driver's car? Only while it is on a trip (moving; a bus or a cab on duty; the car you ride)
    bool carOccupied(int agentIndex) const;
    int loadMultiplier() const { return loadMul_; }
    // The stop-bar + lane-arrow paint mesh (R6c), rebuilt from the graph;
    // public so the gate can assert paint never leaves the carriageway.
    engine::RenderMesh buildRoadMarkings() const;
    CitySim& simMutable() { return sim_; }   // ADR-0062 bridge: release ejected drivers
    const PlaceMap& places() const { return places_; }
    const engine::NavGraph& nav() const { return nav_; }
    // Cars are split across several instance groups, one per body/colour variant
    // (an InstanceGroup shares one mesh, so variety needs multiple groups).
    const std::vector<engine::Entity>& carGroups() const { return carGroups_; }
    engine::Entity carGroup() const { return carGroups_.empty() ? engine::Entity{} : carGroups_[0]; }
    engine::Entity pedGroup() const { return pedGroup_; }
    engine::Entity signalGroup(SignalState s) const { return signalGroups_[static_cast<int>(s)]; }
    engine::Entity signalPostGroup() const { return signalPostGroup_; }
    engine::Entity crosswalkGroup() const { return crosswalkGroup_; }
    // Car lamp instance groups (ADR-0065 follow-up): emissive headlights (white),
    // brake lights (red), and turn signals (amber), rebuilt each step from each
    // drawn car's carLampState — the same shape as the signal lenses (one group
    // per lamp kind; a lit lamp pushes an emissive box at its body marker's world
    // pose). Exposed so headless tests read the baked transforms (mirrors
    // signalGroup()).
    engine::Entity headlightGroup() const { return headlightGroup_; }
    engine::Entity brakeLightGroup() const { return brakeLightGroup_; }
    engine::Entity turnSignalGroup() const { return turnSignalGroup_; }
    const std::vector<engine::Vec2>& crosswalkCenters() const { return crosswalkCenters_; }
    // The painted curbside bay outlines (R6b). Exposed so the height gate (#25)
    // can check the paint lands on the asphalt instead of floating over it.
    engine::Entity parkBayGroup() const { return parkBayGroup_; }
    // Debug widgets (ADR-0061): per-agent ground footprint coloured by behaviour
    // state, and a forward trajectory arrow. Empty unless params.debugWidgets.
    engine::Entity footprintGroup(Agent::State s) const { return footprintGroups_[static_cast<int>(s)]; }
    engine::Entity forwardGroup() const { return forwardGroup_; }
    // Debug NAVGRAPH view: ground strips along every link's lane centrelines +
    // a small ring at each junction node. Static data (depends only on nav_),
    // baked once at build; shown/hidden with the same HUD toggle as the rings.
    engine::Entity navLinkGroup() const { return navLinkGroup_; }
    engine::Entity navNodeGroup() const { return navNodeGroup_; }
    // Debug VISION-CONE view: one ground wedge per MOVING agent, sized to its
    // mode's sensing cone (drivers 18 m / 0.45 rad, walkers 4.5 m / 1.2 rad),
    // at the same widget pose the rings use (real bodies when external).
    engine::Entity visionGroup(Agent::Mode m) const {
        return visionGroups_[static_cast<int>(m)];
    }
    // Half-extents of a car / pedestrian box, for a physics collider that tracks
    // the drawn instance.
    engine::Vec3 carHalfExtent() const {
        return engine::Vec3(params_.carSize.x * 0.5, params_.carSize.y * 0.5,
                            params_.carSize.z * 0.5);
    }
    // Per-group collider half-extents, one entry per carGroups() entry (a group is
    // one fleet slot, so all its cars share a size). A van/box-truck collider is
    // bigger than a sedan's; the physics system sizes each group's boxes from this.
    std::vector<engine::Vec3> carGroupHalfExtents() const;
    engine::Vec3 pedHalfExtent() const {
        return engine::Vec3(params_.pedSize.x * 0.5, params_.pedSize.y * 0.5,
                            params_.pedSize.z * 0.5);
    }

    // A retained lamp attachment marker (ADR-0065 follow-up): a lamp name
    // ("headlight_l/r", "taillight_l/r") and its body-local position (+Z forward).
    // Kept independent of the scripting-only engine::Attachment type so the
    // non-scripting (Makefile) build stores markers too.
    struct LampMarker {
        std::string name;
        engine::Vec3 pos;
    };

    // One wheel of a fleet car, in body-local space (+Z forward, x>0 right).
    // Same independence rationale as LampMarker: a mirror of the scripting-only
    // engine::WheelPlacement, so the non-scripting build still compiles.
    struct CarWheel {
        engine::Vec3 pos;
        Real radius = 0.3;
        Real width = 0.2;
        bool steered = false;
        bool driven = true;
        bool handBrake = false;
    };

    // The fleet slot's car WITHOUT its wheels, for a car the player commandeers
    // (ADR-0062): it becomes a real Vehicle and grows physics wheels, so drawing
    // the baked ones too would put it on eight. Invalid when the level's fleet
    // recipes publish no separate wheelset (or scripting is off) — the caller
    // then draws no car for that slot.
    engine::MeshHandle carChassisMesh(int slot) const {
        if (carChassis_.empty()) return engine::MeshHandle{};
        return carChassis_[((slot % static_cast<int>(carChassis_.size())) +
                            static_cast<int>(carChassis_.size())) %
                           static_cast<int>(carChassis_.size())];
    }
    // That car's GLASS and CABIN, for a commandeered car: the chassis is the shell alone (no glass, no cabin, and the
    // shell's underside culls from inside), so without these the player sat in an empty frame over the road. The
    // cabin is invalid for a see-into body (bus, convertible), whose cabin is part of its chassis.
    engine::MeshHandle carGlassMesh(int slot) const { return slotOf(carGlassMesh_, slot); }
    engine::MeshHandle carCabinMesh(int slot) const { return slotOf(carCabinMesh_, slot); }
    // That car's lamp markers, for a promoted car's lenses. VehicleSystem falls
    // back to four lenses at GUESSED chassis corners when a Vehicle carries no
    // markers — which sat harmlessly on top of the old box car's baked lamp
    // boxes, but floats clear of a real generated body's inset housings and
    // draws every taillight twice. Never empty (a synthesized default set backs
    // the built-in fleet).
    const std::vector<LampMarker>& carLamps(int slot) const {
        static const std::vector<LampMarker> kNone;
        if (carLights_.empty()) return kNone;
        return carLights_[((slot % static_cast<int>(carLights_.size())) +
                           static_cast<int>(carLights_.size())) %
                          static_cast<int>(carLights_.size())];
    }
    // That car's wheels as data, for its Jolt config. Empty => no layout was
    // published; the caller derives one as before.
    const std::vector<CarWheel>& carWheels(int slot) const {
        static const std::vector<CarWheel> kNone;
        if (carWheels_.empty()) return kNone;
        return carWheels_[((slot % static_cast<int>(carWheels_.size())) +
                           static_cast<int>(carWheels_.size())) %
                          static_cast<int>(carWheels_.size())];
    }

private:
    static engine::MeshHandle slotOf(const std::vector<engine::MeshHandle>& v, int slot) {
        if (v.empty()) return engine::MeshHandle{};
        const int n = static_cast<int>(v.size());
        return v[static_cast<std::size_t>(((slot % n) + n) % n)];
    }
    // How many car variants were actually BUILT — the Lua fleet's length when a
    // level ships recipes, the built-in table's size otherwise. Agent-to-group
    // mapping must wrap by this and not by the C++ colour table, or a fleet of
    // any other length indexes off the end of its own groups. 1 when there are
    // no groups, so the modulo stays defined; the callers' index guards do the
    // rest.
    int drawVariantCount() const {
        return carGroups_.empty() ? 1 : static_cast<int>(carGroups_.size());
    }

    void syncGroups(engine::World& world);
    void syncCarLamps(engine::World& world);   // bake the emissive lamp instances
    int carTurnDir(const Agent& a) const;      // route-bend turn side (-1/0/+1)
    // Box sized by a.mode (car/ped). `agentIdx` >= 0 enables the per-agent
    // TILT LOW-PASS for cars (roads-v2.1 4b): the road-plane basis is re-fit
    // from terrain samples every tick, and raw refits vibrate the body on
    // noisy/quantised ground (drive feedback C1: "cars vibrate").
    engine::Mat4 agentPose(const Agent& a, int agentIdx = -1) const;
    // Where a signalled approach's pole stands and which way its head/arm point
    // (matches the city's street_kit placement, scaled to road width).
    struct SignalSite {
        engine::Vec3 base;   // pole foot (world)
        engine::Vec3 face;   // unit XZ: head facing (toward oncoming traffic)
        engine::Vec3 side;   // unit XZ: arm reach (toward the road centre)
        Real yaw;            // facing yaw for the assembly
    };
    SignalSite signalSite(int link) const;
    engine::Mat4 signalPostPose(int link) const;            // the pole assembly
    engine::Mat4 signalLensPose(int link, SignalState s) const;  // lit lens at active slot
    Real groundAt(Real x, Real z) const;
    // The height of the DRIVING SURFACE on `link` at `station` metres along it —
    // the same reading the sim gives a car driving that spot (Agent::deckY), so
    // anything placed against the road (a parked car, a painted bay outline)
    // lands on the deck the mesher actually built rather than on the raw
    // terrain beside it (#25). Falls back to the ground for an unknown link.
    Real deckYAt(int link, Real station, engine::Vec2 p) const;
    // A deck car's height: the drawn deck it is on (RoadDeckField::heightNear, within 3 m of its link's
    // lerped height `refY`, so never the street under it or the freeway over it), else `refY` itself.
    // The lerp between nav ends cuts under a ramp's vertical curves; the drawn profile does not (#35).
    Real deckSurfaceNear(Real x, Real z, Real refY) const;
    // The fleet slot agent `ai` is DRAWN with (the ambient rotation, the bus override): the one mesh,
    // lamps and wheel set that car wears. -1 when there is no drawable slot.
    int drawSlotFor(int ai) const;
    // The drawn road surface under (px, pz) for car `a`, `along` metres ahead of its centre: on a deck, the
    // deck it is on (deckSurfaceNear from its link's height); on the ground, the road or ground at ITS level
    // (the terrain + its layer lift as the reference), so an overpass or a stacked junction pad above never
    // answers for a street car (wheel_lab: groundAt's top-pad rule read the freeway 8 m up).
    Real carSurfaceAt(const Agent& a, Real px, Real pz, Real along) const;

    CityRenderParams params_;
    engine::NavGraph nav_;
    bool playerRiding_ = false;
    int playerRidingAgent_ = -1;
    // Bus-stop furniture entities, kept so a rebuild can remove the old set
    // rather than stacking a second pole on every stop.
    std::vector<engine::Entity> busStopProps_;
    int busVariant_ = -1;   // fleet slot that draws as a bus (-1 = none)
    Real busPrintAcc_ = 0;  // RT_PRINT_BUSES cadence
    CitySim sim_;
    std::vector<engine::Entity> carGroups_;   // one per car variant (body + colour)
    std::unordered_map<int, engine::Mat4> physPose_;      // R5: agent -> body pose
    std::vector<std::vector<int>> carAgentIds_;           // parallel to bakes
    // (The one-shot SceneryCar bake that used to live here is gone: parked cars
    // are real SimVehicles now, drawn straight off the vehicle list in
    // syncGroups. A cached pose would be wrong rather than stale, because a
    // parked car moves as soon as its owner drives it away.)
    // Set per fixed step: bake the render poses only on the frame's LAST step
    // (see fixedUpdate). True by default so a direct step() call still bakes.
    bool bakeThisStep_ = true;
    // Adaptive-dip state. The CADENCE itself lives in CitySim (setTickPeriod);
    // this is only the load multiplier the bridge derives from the frame's
    // fixed-step count and scales that period by.
    int loadMul_ = 1;
    int loadStreak_ = 0;
    std::vector<std::vector<int>> pedAgentIds_;           // ditto, ped group (P4)
    engine::Entity pedGroup_;
    engine::Entity pedSeatedGroup_;   // the drawn crowd's people sitting on benches and chairs (M5)
    engine::Entity pedGroundGroup_;   // ... sitting on the grass (a picnic)
    engine::Entity pedLieGroup_;      // ... lying on it (in the sun)
    // THE MID-RANGE BAND (the 100k plan's M band; Glenn: "increase the number of people throughout the island"):
    // far (V) agents within farCarRadius / farPedRadius, drawn from CitySim::farDrawPose as cheap stand-ins -- a
    // two-box car per body slot, the box person -- in groups of their own (no physics proxy, no lamps)
    std::vector<engine::Entity> farCarGroups_;
    engine::Entity farPedGroup_;
    // (far band ground heights: in the agent's draw cache below)
    int farDrawn_ = 0;
    // THE GPU CROWD'S FEED (crowd.comp; when the renderer has it, it replaces the CPU band above): one record per far
    // traveller (V or D, on a trip), its remaining route in an append-only arena compacted when the garbage
    // outweighs the live. Each frame re-reads only the agents the sim says changed (CitySim::takeCrowdChanges).
    void feedGpuCrowd(engine::FrameContext& ctx);
    bool crowdInit_ = false, gpuCrowd_ = false;
    std::vector<engine::Renderer::CrowdAgent> crowdRec_;
    std::vector<int> crowdSlot_, crowdOwner_, crowdFree_, crowdChanges_;
    // per slot: the route its arena span was laid for (size, first and last link) and the leg the span starts at --
    // the same route next tick reuses the span (only its distance and clock change)
    struct CrowdSpan { uint32_t size = 0; int first = -1, last = -1, baseLeg = 0; };
    std::vector<CrowdSpan> crowdSpan_;
    std::vector<uint32_t> crowdArena_, crowdDirtySlots_;
    std::size_t crowdGarbage_ = 0;
    std::vector<float> crowdLinkLen_;   // per link: its length as the shader measures it (xz, end to end)
    Real farCarDistance = 4000.0, farPedDistance = 1500.0;
    int crowdLive_ = 0, crowdDrawnSeen_ = 0;
    long lastDormantEvents_ = 0;      // the stats line's sleepers' events since the last line
    double lastDormantMs_ = 0;
    Real farCarRadius = 900.0, farPedRadius = 700.0;
    // PEOPLE INSIDE (campus M4): a streamed interior shows the agents the sim has indoors there -- sitting, and
    // standing (a lecturer) or lying (asleep) on the standing body. stepIndoors refreshes them once a second.
    engine::Entity indoorSitGroup_, indoorBodyGroup_;
    void stepIndoors(engine::World& world, Real dt);
    Real indoorAcc_ = 1.0;
    // record index -> whose people are drawn inside: the building's own place (kNoPlace: none) and each furnished shop
    // unit's (Interactables::units; kNoPlace: no place found -- its pieces go with the building's)
    struct BuildingPeople { PlaceId whole = kNoPlace; std::vector<PlaceId> unit; };
    std::unordered_map<int, BuildingPeople> buildingPlace_;
    std::vector<std::array<double, 5>> footpaths_;          // the parks' and the quad's walks (CitySimConfig)
    std::vector<std::vector<std::array<double, 2>>> jogLoops_;   // the tracks to run (CitySimConfig)
    std::vector<engine::CitySimConfig::ActivityAreaSpec> activityAreas_;   // the pitches, the lawns (CitySimConfig)
    std::vector<std::array<double, 5>> sitSpots_;
    std::vector<engine::CitySimConfig::GarageSpec> garages_;
    // THE OPEN SIGNS: each authored place's PlaceId (kNoPlace: skipped), and who is on shift where (dealOpenSigns)
    std::vector<PlaceId> placeOfAuthored_;
    std::vector<uint16_t> onShift_;
    PlaceId signWatch_ = kNoPlace;       // the nearest lit sign's place: opensigns? lists its staff
    std::vector<uint8_t> signedPlace_;   // places with an OPEN sign (by PlaceId): the only ones staffOnShift counts
    void dealOpenSigns(engine::World& world, engine::Settings& settings, engine::Vec3 cam);
    int parkingTick_ = 0;   // (parking? telemetry every 60 frames)   // the drivable parking garages (CitySimConfig)   // seats that are not furniture: a fountain's rim (CitySimConfig)
    std::unordered_map<int, std::vector<uint32_t>> indoorHeld_;   // record index -> spots our occupants hold, per piece
    int indoorDrawn_ = 0;
public:
    int indoorDrawn() const { return indoorDrawn_; }
private:
    engine::Entity signalGroups_[3];   // lit lens, indexed by SignalState (Green/Yellow/Red)
    engine::Entity signalPostGroup_;   // the static pole+arm+head assemblies
    engine::Entity parkBayGroup_;      // curbside bay outline markings (R6b)
    engine::Entity roadMarkGroup_;     // stop bars + lane-turn arrows (R6c)
public:
    // Did this bridge draw its own stop bars and arrows? Not over a builder that painted them (#39).
    bool drewRoadMarkings() const { return roadMarkGroup_.valid(); }
private:
    engine::Entity crosswalkGroup_;    // baked zebra decals at junction mouths
    // Car lamps (ADR-0065 follow-up): one emissive instance group per lamp kind.
    engine::Entity headlightGroup_{};  // white, forward
    engine::Entity brakeLightGroup_{}; // red, rear
    engine::Entity turnSignalGroup_{}; // amber, blinks (render clock)
    // Retained lamp markers, one list per car variant (indexed like carGroups_):
    // the Lua fleet's headlight_l/r + taillight_l/r markers in body-local space,
    // or a synthesized default set for C++ fallback cars. An empty entry => that
    // variant draws no lamps.
    std::vector<std::vector<LampMarker>> carLights_;
    // Wheel-less bodies + wheel layouts, one per car variant (indexed like
    // carGroups_). Populated only from Lua fleet recipes that publish a separate
    // wheelset; see carChassisMesh() / carWheels().
    std::vector<engine::MeshHandle> carChassis_;
    std::vector<std::vector<CarWheel>> carWheels_;
    // SEE-INTO VEHICLES (the bus; Glenn: "we should see the npcs sitting on
    // the bus. There should be a driver"). Per variant, indexed like
    // carGroups_: a CLEAR glass group (invalid Entity when the slot's glass is
    // merged dark into its body, i.e. every ordinary car), and the seat hip
    // points people are drawn on. The people themselves are instanced: one
    // driver group, three rider groups for a little variety of dress.
    std::vector<engine::Entity> carGlassGroups_;
    // FLEET V2 GLASS. Per slot: the same glass OPAQUE with a reflective glass material (traffic at large),
    // the cabin apart from the shell, and whether the slot is always seen into (the bus). An ordinary car
    // near the player draws clear glass + cabin + driver instead of the opaque glass (the near swap).
    std::vector<engine::Entity> carGlassOpaqueGroups_;
    std::vector<engine::Entity> carInteriorGroups_;
    std::vector<engine::MeshHandle> carGlassMesh_, carCabinMesh_;
    std::vector<char> carSeeInto_;
    // The agents drawn see-through this bake (nearest the player, with hysteresis).
    std::unordered_set<int> nearSwap_;
    std::vector<std::vector<engine::Vec3>> carSeats_;
    std::vector<std::vector<engine::Vec3>> carDoors_;
    std::vector<engine::Vec3> carDriverSeat_;
    std::vector<char> carHasDriver_;
    engine::Entity busDriverGroup_{};
    engine::Entity busRiderGroups_[3] = {};
    // The seat the PLAYER holds on the bus they ride (-1 none): NPC riders
    // are dealt round it, so nobody is drawn sitting in the player's lap.
    int playerSeatAgent_ = -1, playerSeat_ = -1;
    // The matrix each see-into vehicle was DRAWN with at the last bake, by
    // agent. A passenger is placed through exactly this -- recomputing the
    // pose skipped the drawn body's tilt filter, and on a slope the rider
    // wobbled against the bus by the difference.
    std::unordered_map<int, engine::Mat4> busDrawnPose_;
    // Parked cars that survived the distance cull on the last bake — how many
    // car bodies the frame actually pays for. This was computed and discarded,
    // which left the car share of the frame a guess with a 2.7x spread.
    int parkedDrawn_ = 0;
    // THE DRAW CACHE (bytes per agent; Glenn: "Why do you keep putting off this work?"): what the drawers keep per
    // agent -- its last car pose, its tilt low-pass, its speed for the brake-light test, its far-band ground height
    // -- for the agents actually DRAWN (the near and mid-range ~2,000), not five arrays sized for all 100,000
    // (~230 bytes each). Entries not touched for a while are swept.
    struct PoseCache { Real x = 0, z = 0, hx = 0, hz = 0; int slot = -1; bool valid = false; engine::Mat4 m; };
    struct AgentDrawCache {
        PoseCache pose;
        engine::Vec3 smoothUp{0, 0, 0};
        Real prevSpeed = 0;
        bool fresh = true;              // prevSpeed not yet seeded (the first look sees no deceleration)
        Real farY = 0;
        engine::Vec2 farYAt{1e30, 1e30};
        uint32_t used = 0;
    };
    mutable std::unordered_map<int, AgentDrawCache> drawCache_;
    mutable uint32_t drawBake_ = 0;
    AgentDrawCache& drawCache(int ai) const {
        AgentDrawCache& c = drawCache_[ai];
        c.used = drawBake_;
        return c;
    }
    Real bakeDt_ = 1.0 / 60.0;                     // last step's dt (filter gain)   // last step's per-agent speed (brake decel)
    // debug ground rings, one per Agent::State (indexed by it)
    engine::Entity footprintGroups_[static_cast<int>(Agent::State::Count)]{};
    engine::Entity forwardGroup_{};          // debug forward-trajectory arrows
    engine::Entity navLinkGroup_{};          // debug navgraph lane strips (static bake)
    engine::Entity navNodeGroup_{};          // debug junction-node rings (static bake)
    engine::Entity visionGroups_[2]{};       // debug sensing wedges, indexed by Agent::Mode
    engine::Entity blockGroup_{};            // debug city-BLOCK outlines (static bake)
    engine::Entity lotGroup_{};              // debug LOT outlines (static bake)
    engine::Entity colliderStripGroup_{};    // debug collider-prism rim outlines
    engine::Entity colliderPostGroup_{};     // debug collider-prism corner posts
    std::vector<engine::Mat4> navLinkBake_;  // cached navgraph transforms (built once)
    std::vector<engine::Mat4> navNodeBake_;
    std::vector<engine::Mat4> blockBake_;    // cached block-outline transforms
    std::vector<engine::Mat4> lotBake_;      // cached lot-outline transforms
    engine::AssetManager* assets_ = nullptr; // for bakes made after build() (bakePlanOutlines)
    // THE BUSES' DESTINATION SIGNS (engine::busSignImages): one group per label (each route's, then NOT IN SERVICE),
    // made on the first frame the renderer is at hand (the atlas is a texture); filled per step in syncCarLamps
    std::vector<engine::Entity> busSignGroups_;
    std::vector<int> busSignRow_;   // per route: its label's row
    int busSignOff_ = -1;           // the NOT IN SERVICE row
    int busSignsDrawn_ = 0;         // signs pushed this step (depots?)
    engine::Vec3 busSignAt_{0, 0, 0};   // ...the last one's world position
    bool busSignsTried_ = false;
    engine::Mat4 busSignLocal_;     // the board on the bus, in its body's frame
    void makeBusSigns(engine::FrameContext& ctx);
    int busSignRowFor(int agent) const;
    bool planBaked_ = false;                 // the plan outlines bake on first show
    void bakePlanOutlines(engine::World& world);
    std::vector<engine::Mat4> colliderStripBake_;  // prism rims (base + top loops)
    std::vector<engine::Mat4> colliderPostBake_;   // prism vertical corner posts
    std::vector<int> signalLinks_;     // approach links that carry a signal (cached)
    std::vector<uint8_t> siteShared_;  // by link: its signal shares an earlier one's pole and head (draws no lens)
    std::vector<int> postLinkOf_;      // per signal-post instance: the link whose state its head shows

    std::vector<int> parkedScratch_;   // the vehicles near the player (syncGroups)
    // Each signal's three lens poses (red, amber, green), parallel to signalLinks_: the poles never move, and
    // working them out every bake (a ground sample apiece) was 10.6 ms of the island's 13 ms sync for 4,727 signals
    std::vector<std::array<engine::Mat4, 3>> signalLensCache_;
    // BUILD-TIME furniture adoption (device: the city places poles, the sim
    // reacts): per-link SignalSite overrides published by the level loader's
    // StreetFurniture component. Empty when the level placed none — then this
    // system computes its own sites as before.
    std::vector<SignalSite> siteByLink_;
    std::vector<uint8_t> siteHasLink_;
    std::vector<engine::Vec2> crosswalkCenters_;   // one per junction approach (centre of band)
    std::function<double(double, double)> heightAt_;   // terrain drape (may be null)
    Real roadLift_ = 0.0;
    // The decks the level's road meshes rode (RoadDeck components, copied at
    // build): groundAt answers from these wherever a road passes, so every
    // placed car/paint/lens stands on the drawn asphalt rather than on the
    // terrain carved 0.22 m under it.
    std::vector<engine::RoadDeckField> decks_;
    // Sidewalk band width from the widest net look — signalSite's kerb
    // back-off must clear the junction pad, which spans out to
    // carriageway/2 + THIS (mirrors StreetFurnitureParams::sidewalkWidth).
    Real sidewalk_ = 3.5;
    bool built_ = false;
    bool debugWidgets_ = false;   // master runtime toggle (init from params; J flips it)
    // Per-layer refinements of debugWidgets_ (ADR-0066 panel): when the master is
    // on, each of these gates one widget layer independently from the Living City
    // ImGui section. Default on, so the J key alone behaves exactly as before.
    bool showAgentWidgets_ = true;   // per-agent state rings + intent arrows
    bool showVisionCones_ = true;    // per-agent sensing wedges
    bool showNavGraph_ = true;       // lane strips + junction-node rings
    bool showPlan_ = true;           // city-plan outlines: blocks + lots (L key)
    bool showColliders_ = false;     // building physics prisms (rims + posts)
    bool rebuildRoadsRequested_ = false;   // panel button → host reseeds + reloads
    int  inspectAgent_ = -1;         // agent selected in the inspector (-1 = none)
    // Places (ADR-0066): the level-authored destinations, and the routable
    // PlaceMap built from them at build() (each entrance snapped to the sidewalk).
    // Drawn as projected ImGui labels + markers when showPlaces_ (needs no mesh).
    std::vector<engine::AuthoredPlace> authoredPlaces_;
    PlaceMap places_;
    bool showPlaces_ = true;         // draw place markers + labels (ImGui overlay)
    // The SUN's elevation, staged each frame from the render view so the bake
    // (which runs without a FrameContext) can light traffic on the same truth
    // the player's car and the street lamps use. Defaults to daylight.
    Real solarElevation_ = 1.0;
    // Whether a render view has actually staged the sun. Headless sims, the
    // offline tracer and any host without the lighting pipeline never do, and
    // there the SIM CLOCK remains the only truth there is — so the clock stays
    // the documented fallback rather than every such context driving in the
    // dark with its headlights off.
    bool solarStaged_ = false;
    // THE WORLD CLOCK from a staged day/night cycle (lighting.clockHours),
    // via setWorldClock: the latest sky hour before build is where the city
    // opens; after build the rate is refreshed every step and a jump
    // re-seeds. -1 / 0 until a cycle is staged; the level's own citysim
    // clock stands where none ever is.
    Real worldClockHour_ = -1.0;
    Real worldClockRate_ = 0.0;
    void adoptWorldClock(engine::FrameContext& ctx);
    bool carsExternallyOwned_ = false;   // ADR-0062: a CityVehicleSystem owns the cars
    bool pedsExternallyOwned_ = false;   // ADR-0062: a CityWalkerSystem owns the peds
    std::vector<ExternalAgentPose> externalCarPoses_;   // real car poses for widgets
    std::vector<ExternalAgentPose> externalPedPoses_;   // real walker poses for widgets
};

}  // namespace citysim

#endif
