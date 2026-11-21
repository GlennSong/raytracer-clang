#ifndef RAYTRACER_APPS_CITYSIM_CITY_RENDER_H
#define RAYTRACER_APPS_CITYSIM_CITY_RENDER_H

#include "../../engine/system.h"
#include "../../engine/components.h"   // engine::AuthoredPlace (level-authored places)
#include "city_car.h"      // CityCarBody, CityCarLink (entityCarDrawKey)
#include "city_meshes.h"   // buildPersonMesh, materials (was declared here)
#include "city_sim.h"
#include "places.h"        // PlaceMap (ADR-0066)
#include "../../engine/control/city_requests.h"
#include "../../engine/control/control_commands.h"
#include "../../engine/player/seat_geometry.h"   // SeatGeometry (a rider astride)
#include <array>
#include <functional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace citysim {

// The ECS render bridge for the agent-based city simulation (ADR-0060 Phase 6).
// It hands the level's road model to a deterministic
// CitySim of pedestrian agents over its walks, and bakes their poses into
// InstanceGroups so RenderSystem draws the whole city as a few instanced
// batches: one for cars, one for pedestrians, and one per signal state (red /
// yellow / green) whose emissive lenses light up to show each stoplight phase.
//
// This is the APPLICATION layer (apps/citysim). It depends on the core engine
// (World, components, AssetManager) but the core never depends on it. The
// reusable primitives it stands on — the road model, its router, perception — live in core.
//
// build()/step() take a World directly (like TrafficSystem) so the spawn +
// pose-bake logic is unit-tested headless; mesh upload is the only part that
// needs the AssetManager (skipped, with null mesh handles, when absent).
struct CityRenderParams {
    int cars = 40;          // -1 = by density (carsPerLaneKm over the nav)
    int pedestrians = 40;   // -1 = by density (pedsPerKm over the sidewalks)
    Real carsPerLaneKm = 10.0;
    Real pedsPerKm = 6.0;
    // Ceiling on each density-derived count (CitySimConfig::maxAmbient). The
    // bridge used to clamp at a literal 400, which halved metro v2: its own
    // network asks for 769 cars and 788 walkers.
    int maxAmbient = 400;
    int nearTarget = 0;                    // CitySimConfig::nearTarget
    int farTarget = 0;                     // CitySimConfig::farTarget
    uint32_t seed = 1;
    Real hoursPerSecond = 1.0 / 3600.0;    // sim-clock hours advanced per real second (real time: DayNightCycle)
    // The in-world hour the level OPENS at. The population is placed directly
    // from its schedules at this hour (CitySim::seedFromSchedule), so any hour
    // costs the same: 22:00 is as cheap as 08:00. 10.5 keeps the historical
    // feel of the retired warm-up, which landed around mid-morning.
    Real startHour = 10.5;
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
    // the rule set the driving drives by (CitySimConfig::drivingRules); empty: no driving, no traffic (ADR-0461)
    std::string drivingRules;
    std::string levelPath;         // the level file (CitySimConfig::levelPath): the driving builds its lane graph from it
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
    bool schedule = true;                  // false: everyone waits at home to be sent (CitySim::setDaySchedule)
    std::vector<engine::CityMarker> markers;   // named points beside streets (CitySimConfig::markers, ADR-0441)
    std::vector<engine::CityResident> residents;     // the people with the agent brain (CitySimConfig::residents; city_brain.h)
    std::vector<engine::CityHousehold> households;   // ...and the households whose cars they share (ADR-0443)
    std::string crowd;   // a crowd made from the roads: crowd.lua's file (CitySimConfig::crowd; city_crowd.h, ADR-0446)
    bool drivableSpacesOnly = false;   // kerb spaces a car can drive from and back to only (CitySimConfig, ADR-0446)
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
    // pose when one is reported (walkers owned by the physics bridge), else the
    // sim ghost (pos + groundAt + elevation). `outPos` is the body ORIGIN on the
    // ground (y = terrain + elevation); `outHeading` is its XZ heading. Returns
    // false for an out-of-range / released / unreported (no external body) agent.
    bool agentWorldPose(int agentId, engine::Vec3& outPos,
                        engine::Vec2& outHeading) const;

    // A HANDOFF FOR PEDESTRIANS (ADR-0062): when a CityWalkerSystem owns them
    // as physics characters, this bridge stops baking the instanced ped boxes
    // (they'd draw twice) — the CitySim keeps planning; walkers follow bodies.
    void setPedsExternallyOwned(bool on) { pedsExternallyOwned_ = on; }
    bool pedsExternallyOwned() const { return pedsExternallyOwned_; }

    // The physical walker poses (ADR-0062), fed each step by the walker bridge so
    // the per-agent DEBUG WIDGETS ring the REAL body — not the planner ghost, which
    // legitimately runs ahead/behind (an empty ring on the ground is the ghost).
    // Only used when walkers are externally owned; one without a reported pose draws no widget.
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
    void setExternalPedPoses(std::vector<ExternalAgentPose> poses) {
        externalPedPoses_ = std::move(poses);
    }

    // --- testable core (no FrameContext) -----------------------------------
    // Hand the sim the level's road model (engine::LevelRoadModel), seed the CitySim, and
    // create the instance-group entities. `assets` may be null (tests): then the
    // groups carry a null MeshHandle and only the transforms are populated.
    // No-op (returns false) if the world holds no built road model with walks.
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
    // THE TYRES AGAINST THE ROAD (the new model's drawing check, #255): the car entity `car` (city_car.h) as the
    // four-wheel fit puts it now (without the tilt filter's memory: a pure read), and for each of its wheels the height of the wheel's
    // lowest point above the drawn road surface under it (metres; + floats, - sinks), wheel by wheel; and, when asked,
    // the pose it is drawn with and each wheel's lowest point. False when the driving does not have the car on its road
    // (no lane component) or its model has no wheels.
    bool wheelGaps(engine::World& world, engine::Entity car, std::vector<Real>& gaps, engine::Mat4* pose = nullptr,
                   std::vector<engine::Vec3>* bottoms = nullptr) const;
    // A CAR'S SEATS (Glenn, 2026-10-02: "a way to detect which part of the car you get into so you can be a
    // passenger (backseat, frontseat passenger) or the driver ... we'd have to be pointing at the seat that we
    // want"): a car entity's pose where its body is this step (city_car.h CityCarPose, ADR-0445) and its seat hips in
    // the body frame -- [0] the driver's, [1] the front passenger's (the driver's mirrored), then the two rear seats
    // when the body has a back bench. False: it has no such pose (the player has it, physics carries it, or no step
    // yet), or its model has no driver's seat.
    bool carEntitySeatsOf(engine::World& world, engine::Entity car, engine::Mat4* pose,
                          std::vector<engine::Vec3>* seats) const;
    // The model (variant) a household car is drawn as: its fleet slot, and its kind's (a bus, a cab, a fire engine).
    int carEntityVariant(int slot, VehicleType type) const { return parkedVariant(slot, type); }
    // The key of each car instance this bake (parallel to each carGroup's transforms): the car entity's driver's
    // agent index while somebody sits at its wheel, else entityCarKey (entityCarDrawKey).
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
    // What agent `i` stands on at (x, z): the ground, or -- waiting for a train -- its platform's top (the platform is
    // not ground: a walker posed on the ground under it stood inside the embankment, out of sight)
    Real floorAt(int i, Real x, Real z) const {
        if (const RailTrip* t = sim_.railRiders().tripOf(i))
            if (t->phase == RailTrip::Phase::Waiting && (Vec2(x, z) - t->spot).length() < 12.0) return t->top;
        return groundAt(x, z);
    }
    // Where an agent is DRAWN this step: one tick behind, interpolated through the sim's tick (the same rule the
    // crowd's instances follow -- agentPose). Between ticks the raw position holds still and then hops; at a slowed
    // tick (the adaptive rate, down to 7.5 Hz) a walker posed from it read as flickering at superspeed.
    engine::Vec2 drawnAgentPos(int agentIndex, engine::Vec2* heading = nullptr) const;
    // Is anyone in this car entity's driver's seat (the agent at its wheel)? While they are in it (a household's car
    // with its person in it), or under way.
    bool carOccupied(int agentIndex) const;
    // Is it dark by the sim's clock (in-world hours)? What the cars' headlights follow where no render view has staged
    // the sun (headless): on before dawn and after dusk.
    static bool clockIsDark(Real clockHours);
    int loadMultiplier() const { return loadMul_; }
    CitySim& simMutable() { return sim_; }   // the bridges: the take-a-car path, the near tier
    // A household car's key in carAgentIds_ while nobody is at its wheel: below every parked row's (-2 - row), as the
    // car's number is past every row (city_car.h). Public for the physics bridge, which keys its boxes by it.
    static int entityCarKey(uint32_t id) { return -2 - static_cast<int>(id); }
    // The bit a car entity's drawn instance carries in its name (InstanceGroup::ids; its number shifted by 8 beside it),
    // so a reader of the car groups can tell those instances from the sim's own cars.
    static constexpr uint64_t kCarEntityInstance = uint64_t(1) << 61;
    // ...and with somebody at its wheel, that driver's agent index (as syncCarEntities draws it)
    static int entityCarDrawKey(const CityCarBody& body, const CityCarLink& link) {
        return link.driver >= 0 ? link.driver : entityCarKey(body.id);
    }
    const PlaceMap& places() const { return places_; }
    // The height things drawn at `p` stand on: the ground.
    Real surfaceY(engine::Vec2 p) const;
    // Cars are split across several instance groups, one per body/colour variant
    // (an InstanceGroup shares one mesh, so variety needs multiple groups).
    const std::vector<engine::Entity>& carGroups() const { return carGroups_; }
    int taxiVariant() const { return taxiVariant_; }   // the slot the cabs draw as (-1: the fleet has none)
    engine::Entity carGroup() const { return carGroups_.empty() ? engine::Entity{} : carGroups_[0]; }
    engine::Entity pedGroup() const { return pedGroup_; }
    engine::Entity signalGroup(LensColour c) const { return signalGroups_[static_cast<int>(c)]; }
    engine::Entity signalPostGroup() const { return signalPostGroup_; }
    // THE DRAWN SIGNAL HEADS (ADR-0456), one per instance of the post group, in its order: the model arm each is
    // attached to (city, junction, arm: StreetFurniture::Signal), the new driving's lane junction there (not valid:
    // the head is dark: the new driving does not drive here), and what it shows now (driving::headAt), as lit in the
    // post instance's bottom row (headFaceNumber) and the arrow lens groups (signalGroup).
    struct SignalHead {
        int link = -1;                        // the street-map link it was planted along
        int city = -1, junction = -1, arm = -1;
        engine::Mat4 arrowLens[2];            // its arrow lens lit for a far turn [0] or a near turn [1]
    };
    std::size_t signalHeadCount() const { return signalHeads_.size(); }
    const SignalHead& signalHead(std::size_t k) const { return signalHeads_[k]; }
    engine::JunctionId signalHeadJunction(std::size_t k) const {
        return k < headJunctions_.size() ? headJunctions_[k] : engine::JunctionId();
    }
    driving::HeadShows signalHeadShows(std::size_t head) const;
    static int headFaceNumber(driving::Lamp face);
    engine::Entity crosswalkGroup() const { return crosswalkGroup_; }
    // How many drawn cars carry any of these lit-lamp bits this bake (engine::kLampMask*): the lamps are the body's
    // own lenses, lit by a mask on its instance (ADR-0342). For the headless tests.
    std::size_t carsLit(engine::World& world, uint32_t bits) const;
    // The painted curbside bay outlines (R6b). Exposed so the height gate (#25)
    // can check the paint lands on the asphalt instead of floating over it.
    engine::Entity parkBayGroup() const { return parkBayGroup_; }
    // Debug widgets (ADR-0061): per-agent ground footprint coloured by behaviour
    // state, and a forward trajectory arrow. Empty unless params.debugWidgets.
    engine::Entity footprintGroup(Agent::State s) const { return footprintGroups_[static_cast<int>(s)]; }
    engine::Entity forwardGroup() const { return forwardGroup_; }
    // Debug WALK-POINT view (the "nav" layer): a short strip and a small ring at each of the sim's walk points. Static
    // data, baked once at build; shown/hidden with the same HUD toggle as the rings.
    engine::Entity navLinkGroup() const { return navLinkGroup_; }
    engine::Entity navNodeGroup() const { return navNodeGroup_; }
    // Debug VISION-CONE view: one ground wedge per MOVING walker, sized to its
    // sensing cone (4.5 m / 1.2 rad), at the same widget pose the rings use
    // (real bodies when external).
    engine::Entity visionGroup() const { return visionGroup_; }
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

    // (One wheel of a fleet car, CarWheel, is the sim's: city_poses.h.)

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
    // That car's driver's hip point (the fleet recipe's `driver_seat`, its interior's own seat), for a commandeered
    // car's driver: false when the slot publishes none.
    bool carDriverSeat(int slot, engine::Vec3& out) const {
        if (carDriverSeat_.empty()) return false;
        const int n = static_cast<int>(carDriverSeat_.size());
        const std::size_t v = static_cast<std::size_t>(((slot % n) + n) % n);
        if (v >= carHasDriver_.size() || !carHasDriver_[v]) return false;
        out = carDriverSeat_[v];
        return true;
    }
    // Nothing round that car's driver (a convertible): a crash throws them (ADR-0252).
    bool carOpenSeat(int slot) const {
        if (carOpenSeat_.empty()) return false;
        const int n = static_cast<int>(carOpenSeat_.size());
        return carOpenSeat_[static_cast<std::size_t>(((slot % n) + n) % n)] != 0;
    }
    engine::MeshHandle carCabinMesh(int slot) const { return slotOf(carCabinMesh_, slot); }
    float carGlassOpacity(int slot) const {
        return slot >= 0 && static_cast<std::size_t>(slot) < carGlassOpacity_.size() ? carGlassOpacity_[static_cast<std::size_t>(slot)] : 0.30f;
    }
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
    // THAT CAR's HANDLING (item 23, #117): the slot's Jolt config -- its kit's or class's drivable handling on
    // its drawn wheels, the fleet's `handling` read by the one handling reader -- for a physical-tier car and a
    // commandeered one alike. Null when the fleet published none for it (no vehicles script).
    // THE SLOT AS THE PLAYER'S CAR (ADR-0355): its drivable recipe ("kit_suv") and paint; false when it has none
    bool carDrivable(int slot, std::string& recipe, engine::Vec3& paint) const {
        if (carDrivable_.empty()) return false;
        const int n = static_cast<int>(carDrivable_.size());
        const std::size_t i = static_cast<std::size_t>(((slot % n) + n) % n);
        recipe = carDrivable_[i];
        paint = i < carPaint_.size() ? carPaint_[i] : engine::Vec3(0.5, 0.5, 0.5);
        return !recipe.empty();
    }
    const engine::PhysicsWorld::VehicleConfig* carConfig(int slot) const {
        if (carConfigs_.empty()) return nullptr;
        const int n = static_cast<int>(carConfigs_.size());
        const engine::PhysicsWorld::VehicleConfig& c = carConfigs_[static_cast<std::size_t>(((slot % n) + n) % n)];
        return c.wheels.empty() ? nullptr : &c;
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
    int drawVariantCount() const { return sim_.poses().models.count(); }   // (carGroups_.size(), set at build)

    void syncGroups(engine::World& world);
    // A moving instance's name across bakes (InstanceGroup::ids): the agent's uid, and which of its parts (0 the body;
    // glass, cabin, lamps, a rider... their own small numbers)
    static uint64_t movingId(const Agent& a, uint32_t part) { return (static_cast<uint64_t>(a.uid) << 8) | part; }
    void syncCarLamps(engine::World& world);   // bake the emissive lamp instances
    // A walker's drawn pose (the sim's walker pose, city_poses.h).
    engine::Mat4 agentPose(const Agent& a) const;
    // Where a placed signal pole stands and which way its head/arm point (the loader's StreetFurniture).
    struct SignalSite {
        engine::Vec3 base;   // pole foot (world)
        engine::Vec3 face;   // unit XZ: head facing (toward oncoming traffic)
        engine::Vec3 side;   // unit XZ: arm reach (toward the road centre)
        Real yaw;            // facing yaw for the assembly
    };
    static engine::Mat4 signalLensPose(const SignalSite& site);   // a lens level with the head's green lamp
    // Ties each head to the new driving's lane level (tieSignalHeads, when the driving or its model changes).
    void tieSignalHeads(engine::World& world);
    Real groundAt(Real x, Real z) const;
    // A deck car's height: the drawn deck it is on (RoadDeckField::heightNear, within 3 m of its link's
    // lerped height `refY`, so never the street under it or the freeway over it), else `refY` itself.
    // The lerp between nav ends cuts under a ramp's vertical curves; the drawn profile does not (#35).
    Real deckSurfaceNear(Real x, Real z, Real refY) const;

    CityRenderParams params_;
    // EVERY BUS BODY (Glenn, 2026-11-06: two saloons, "one where it's all seats and one where they're standing room"):
    // the fleet's slots whose class is a bus ("bus", "bus_standing")
    std::vector<int> busVariants_;
    CitySim sim_;
    // THE DRAWN MODELS' BODY FACTS AND THE GROUND are the sim's (city_poses.h, ADR-0454): this host loads them at build
    // and draws from them, by these names
    int& busVariant_ = sim_.poses().models.bus;     // fleet slot that draws as a bus (-1 = none): the first of busVariants_
    int& fireVariant_ = sim_.poses().models.fire;   // ...and as a fire engine (a fire station's, stationed in its bays)
    int& taxiVariant_ = sim_.poses().models.taxi;   // ...and as a taxi (a household's or a lab's car of that class)
    std::vector<std::vector<CarWheel>>& carWheels_ = sim_.poses().models.wheels;   // each model's wheels
    std::function<double(double, double)>& heightAt_ = sim_.poses().ground.terrain;   // terrain drape (may be null)
    std::vector<engine::RoadDeckField>& decks_ = sim_.poses().ground.decks;          // the drawn road decks
    Real& roadLift_ = sim_.poses().ground.roadLift;
    Real& sidewalk_ = sim_.poses().ground.sidewalk;
    std::vector<engine::Entity> carGroups_;   // one per car variant (body + colour)
    // THE MID-RANGE BODIES (vehicles.lua `mid`): per variant, the cheaper whole car drawn from its `mid_from` out --
    // the same instances as carGroups_ (copied once the bake is done, lamp masks and all), the near group stopping
    // where this one starts. Invalid where the recipe has none.
    std::vector<engine::Entity> carMidGroups_;
    void copyMidBodies(engine::World& world);
    std::vector<std::vector<int>> carAgentIds_;           // parallel to bakes
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
    std::array<engine::Entity, engine::kLiePoses> pedLieGroups_;   // ... lying on it (in the sun), one per pose
    // THE BEACH'S THINGS (Glenn, 2026-11-02: "a towel under them and an umbrella"): a towel under everyone lying out
    // on the ground, an umbrella over each group on the sand -- the furniture kit's pieces, one set of groups (one per
    // finish) per colour, their transforms rewritten each sync
    std::array<std::vector<engine::Entity>, 8> towelGroups_, umbrellaGroups_;
    engine::Entity crowdBoardGroups_[2];   // the crowd's and the far band's riders' boards (skateboard, scooter)
    void syncBeachThings(engine::World& world);
    // THE MID-RANGE BAND (the 100k plan's M band; Glenn: "increase the number of people throughout the island"):
    // far (V) agents within farPedRadius, drawn from CitySim::farDrawPose as cheap stand-ins -- the box person -- in a
    // group of its own (no physics proxy)
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
    std::vector<uint32_t> crowdArena_, crowdDirtySlots_;
    std::size_t crowdGarbage_ = 0;
    // THE SEGMENTS THE GPU CROWD WALKS (#306 piece 6 W3): every walk of the road model's lane level, point to point, each
    // way -- a walk's first forward segment is crowdWalkSeg_[walk], its reverse crowdRevBase_ on from it -- and each
    // one's length as the shader measures it (xz, end to end). A traveller's span is laid from where it is, at each
    // refresh, over the segments of the steps it has left.
    std::vector<uint32_t> crowdWalkSeg_;
    uint32_t crowdRevBase_ = 0;
    std::vector<float> crowdLinkLen_;
    Real farPedDistance = 1500.0;
    int crowdLive_ = 0, crowdDrawnSeen_ = 0;
    long lastDormantEvents_ = 0;      // the stats line's sleepers' events since the last line
    double lastDormantMs_ = 0;
    Real farPedRadius = 700.0;
    // PEOPLE INSIDE (campus M4): a streamed interior shows the agents the sim has indoors there -- sitting, and
    // standing (a lecturer) or lying (asleep) on the standing body. stepIndoors refreshes them once a second.
    engine::Entity indoorSitGroup_, indoorBodyGroup_;
    void stepIndoors(engine::World& world, Real dt);
    Real indoorAcc_ = 1.0;
    // record index -> whose people are drawn inside: the building's own place (kNoPlace: none) and each furnished shop
    // unit's (Interactables::units; kNoPlace: no place found -- its pieces go with the building's)
    // `unit` is by the index of the interior's shop units, so it holds only for the interior it was made for: a tower
    // streams its interior in storey windows, and the ground floor's (shops) and the 40th's (none) differ -- `units`
    // says which one this is (their count and floor heights), and a different one is matched afresh (#114: a cached
    // list of N units indexed a 1-entry table when the player swung from the street to the top floors)
    struct BuildingPeople { PlaceId whole = kNoPlace; std::vector<PlaceId> unit; std::vector<double> units; };
    std::unordered_map<int, BuildingPeople> buildingPlace_;
    std::vector<std::vector<std::array<double, 2>>> jogLoops_;   // the tracks to run (CitySimConfig)
    std::vector<engine::CitySimConfig::ActivityAreaSpec> activityAreas_;   // the pitches, the lawns (CitySimConfig)
    std::vector<std::array<double, 5>> sitSpots_;
    // THE OPEN SIGNS: each authored place's PlaceId (kNoPlace: skipped), and who is on shift where (dealOpenSigns)
    std::vector<PlaceId> placeOfAuthored_;
    std::vector<uint16_t> onShift_;
    PlaceId signWatch_ = kNoPlace;       // the nearest lit sign's place: opensigns? lists its staff
    std::vector<uint8_t> signedPlace_;   // places with an OPEN sign (by PlaceId): the only ones staffOnShift counts
    std::vector<int> signedStaff_;       // ...and the agents who work at one (listed per population: places are set at build)
    std::vector<uint8_t> signedAtWork_;  // ...each one at work, refreshed a thirtieth a frame from shiftCursor_
    std::size_t signedStaffOf_ = 0, shiftCursor_ = 0;
    void listSignedStaff();
    void dealOpenSigns(engine::World& world, engine::Vec3 cam);
    int parkingTick_ = 0;   // (traffic? telemetry every 60 frames, the open signs every 30)
    std::unordered_map<int, std::vector<uint32_t>> indoorHeld_;   // record index -> spots our occupants hold, per piece
    int indoorDrawn_ = 0;
public:
    int indoorDrawn() const { return indoorDrawn_; }
private:
    engine::EventBus::Subscription onSimHour_;   // SimHourQuery: `daynight?` asks the sim's own hour
    engine::EventBus::Subscription onPin_, onFindPlace_, onLayer_, onJitter_;
    std::vector<engine::CityLayerRequest> layerRequests_;   // overlay <layer> and the web panel, applied in update
    std::vector<engine::JitterRequest> jitterRequests_;
    std::vector<engine::PinRequest> pinRequests_;
    std::string findPlace_;            // a FindPlaceRequest's name, until update answers it
    std::string groundQuery_;          // `ground? x z [r]`, until update answers it
    std::string groundResult_ = "none";
    engine::ControlAnswers answers_;   // traffic? opensigns? findplace? pin? jitter?
    engine::ControlCommands::Binding answerGround_;
    engine::Entity signalGroups_[kLensColourCount];   // the lit arrow lenses, by LensColour
    engine::Entity signalPostGroup_;   // the static pole+arm+head assemblies
    engine::Entity parkBayGroup_;      // curbside bay outline markings (R6b)
    engine::Entity crosswalkGroup_;    // baked zebra decals at junction mouths
    // Retained lamp markers, one list per car variant (indexed like carGroups_):
    // the Lua fleet's headlight_l/r + taillight_l/r markers in body-local space,
    // or a synthesized default set for C++ fallback cars. An empty entry => that
    // variant draws no lamps.
    std::vector<std::vector<LampMarker>> carLights_;
    // Wheel-less bodies + wheel layouts, one per car variant (indexed like
    // carGroups_). Populated only from Lua fleet recipes that publish a separate
    // wheelset; see carChassisMesh() / carWheels().
    std::vector<engine::MeshHandle> carChassis_;   // (the wheel layouts: carWheels_, the sim's models, above)
    // ...and each slot's handling (carConfig), read with the catalogue: no geometry, so a headless sim has it.
    std::vector<engine::PhysicsWorld::VehicleConfig> carConfigs_;
    // per slot: the menu's recipe for it and its paint (loadFleetDrivable), what a taken car is built as
    std::vector<std::string> carDrivable_;
    std::vector<engine::Vec3> carPaint_;
    // ...and its seat (the handling's `seat`): a seat with BARS, not a wheel, is ridden astride, so the slot draws
    // its own rider (carRiderGroups_) posed by the player's seat rule instead of the car driver
    std::vector<engine::SeatGeometry> carSeatGeom_;
    // SEE-INTO VEHICLES (the bus; Glenn: "we should see the npcs sitting on
    // the bus. There should be a driver"). Per variant, indexed like
    // carGroups_: a CLEAR glass group (invalid Entity when the slot's glass is
    // merged dark into its body, i.e. every ordinary car), and the driver's
    // seat. The driver is instanced: one group.
    std::vector<engine::Entity> carGlassGroups_;
    // FLEET V2 GLASS. Per slot: the same glass OPAQUE with a reflective glass material (traffic at large),
    // the cabin apart from the shell, and whether the slot is always seen into (the bus). An ordinary car
    // near the player draws clear glass + cabin + driver instead of the opaque glass (the near swap).
    std::vector<engine::Entity> carGlassOpaqueGroups_;
    std::vector<engine::Entity> carInteriorGroups_;
    std::vector<engine::MeshHandle> carGlassMesh_, carCabinMesh_;
    std::vector<char> carSeeInto_;
    std::vector<float> carGlassOpacity_;
    std::vector<char> carOpenSeat_;   // nothing round the driver (a convertible), by slot
    // The agents drawn see-through this bake (nearest the player, with hysteresis).
    std::unordered_set<int> nearSwap_;
    std::vector<engine::Vec3> carDriverSeat_;
    std::vector<engine::Vec3> carRearHip_;   // per slot: the back bench's hip point (its middle), where carHasRear_
    std::vector<char> carHasRear_;
    std::vector<char> carHasDriver_;
    engine::Entity busDriverGroup_{};
    std::vector<engine::Entity> carRiderGroups_;   // per slot: its rider astride (invalid: a car's driver, if any)
    // THE DRAW CACHE (bytes per agent; Glenn: "Why do you keep putting off this work?"): what the drawers keep per
    // drawn thing -- a car entity's speed for the brake-light test (by entityCarKey), a far walker's ground height --
    // for the things actually DRAWN, not arrays sized for all 100,000. Entries not touched for a while are swept.
    // (An old-sim driver's four-wheel fit memory went with the old driving: ADR-0461.)
    struct AgentDrawCache {
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
    Real bakeDt_ = engine::kDefaultFixedStep;                     // last step's dt (filter gain)   // last step's per-agent speed (brake decel)
    // (The four-wheel fit, its tilt low-pass, a car entity's and a walker's pose are the sim's: city_poses.h.)
    // THE NEW DRIVING'S CARS, WHICH ARE ENTITIES (ADR-0444, ADR-0453; city_car.h), into the car groups beside the sim's
    // own: each car's body instance where its CityCarPose is, and the key it is listed under in carAgentIds_ (its
    // driver's agent index while somebody sits at its wheel, else kEntityCarKey(id)). The lamps for them are
    // syncCarLamps's.
    void syncCarEntities(engine::World& world, std::vector<engine::InstanceGroup*>& cars);
    // The variant a car of fleet slot `slot` and body `type` is drawn as when nobody is driving it (the sim's models:
    // CityCarModels::variantOf).
    int parkedVariant(int slot, VehicleType type) const { return sim_.poses().models.variantOf(slot, type); }
    // The seats of a car drawn as variant `v`: the driver's first, the front passenger's, the back bench's.
    bool seatsOfVariant(int v, std::vector<engine::Vec3>& seats) const;
    // debug ground rings, one per Agent::State (indexed by it)
    engine::Entity footprintGroups_[static_cast<int>(Agent::State::Count)]{};
    engine::Entity forwardGroup_{};          // debug forward-trajectory arrows
    engine::Entity navLinkGroup_{};          // debug navgraph lane strips (static bake)
    engine::Entity navNodeGroup_{};          // debug junction-node rings (static bake)
    engine::Entity visionGroup_{};           // debug sensing wedges (the walkers')
    engine::Entity blockGroup_{};            // debug city-BLOCK outlines (static bake)
    engine::Entity lotGroup_{};              // debug LOT outlines (static bake)
    engine::Entity colliderStripGroup_{};    // debug collider-prism rim outlines
    engine::Entity colliderPostGroup_{};     // debug collider-prism corner posts
    std::vector<engine::Mat4> navLinkBake_;  // cached navgraph transforms (built once)
    std::vector<engine::Mat4> navNodeBake_;
    std::vector<engine::Mat4> blockBake_;    // cached block-outline transforms
    std::vector<engine::Mat4> lotBake_;      // cached lot-outline transforms
    engine::AssetManager* assets_ = nullptr; // for bakes made after build() (bakePlanOutlines)
    bool planBaked_ = false;                 // the plan outlines bake on first show
    void bakePlanOutlines(engine::World& world);
    std::vector<engine::Mat4> colliderStripBake_;  // prism rims (base + top loops)
    std::vector<engine::Mat4> colliderPostBake_;   // prism vertical corner posts
    // THE PLACED SIGNAL POLES (the loader's StreetFurniture, adopted at build): each pole's site, in the loader's
    // order, the walkers' obstacles; and the heads, one per post instance, with the lane junction of the new
    // driving each is tied to (tieSignalHeads; headsModel_: the model they were asked of). The poles never move: their
    // lens poses are worked out once (a ground sample apiece every bake was 10.6 ms of the island's 13 ms sync).
    std::vector<SignalSite> signalSites_;
    std::vector<SignalHead> signalHeads_;
    const engine::RoadModel* headsModel_ = nullptr;
    std::vector<engine::JunctionId> headJunctions_;
    // (the terrain, the road lift, the decks the level's road meshes rode and the sidewalk band are the sim's ground,
    // city_poses.h CityGround: heightAt_, roadLift_, decks_, sidewalk_ above. groundAt answers from the decks
    // wherever a road passes, so every placed car/paint/lens stands on the drawn asphalt rather than on the terrain
    // carved 0.22 m under it.)
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
    // THE SHAKE, MEASURED (`jitter [agent|-1] [frames]`, then `jitter?`): every drawn frame,
    // where a vehicle is drawn (as agentPose places it, without its side effects) and where the camera is; then how
    // evenly each moved -- Glenn: "buses and cars sometimes shake. Their position is not stable. Trains actually
    // appear very stable in contrast". -1: the nearest moving vehicle.
    void measureJitter(engine::FrameContext& ctx);
    struct JitterSample { engine::Vec3 at, cam; engine::Real dt; engine::Vec3 inVehicle; };
    std::vector<JitterSample> jitter_;
    int jitterAgent_ = -1, jitterFrames_ = 0;
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
    // TRAFFIC'S LAMPS AS LIGHTS (ADR-0362): each lit drawn car's head and tail lamps from its own markers, gathered by
    // syncCarLamps, published on the light board's traffic channel by update()
    std::vector<engine::LightCandidate> trafficLights_;
    // THE WORLD CLOCK from a staged day/night cycle (lighting.clockHours),
    // via setWorldClock: the latest sky hour before build is where the city
    // opens; after build the rate is refreshed every step and a jump
    // re-seeds. -1 / 0 until a cycle is staged; the level's own citysim
    // clock stands where none ever is.
    Real worldClockHour_ = -1.0;
    Real worldClockRate_ = 0.0;
    void adoptWorldClock(engine::FrameContext& ctx);
    bool pedsExternallyOwned_ = false;   // ADR-0062: a CityWalkerSystem owns the peds
    std::vector<ExternalAgentPose> externalPedPoses_;   // real walker poses for widgets
};

}  // namespace citysim

#endif
