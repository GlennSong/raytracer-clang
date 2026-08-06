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
#include <vector>

namespace citysim {

// The ECS render bridge for the agent-based city simulation (ADR-0060 Phase 6).
// It builds a NavGraph from the level's RoadNet entities, runs a deterministic
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
    Real signalLensSize = 0.34;            // lit emissive lens cube edge (m)
    // How far from the player a PARKED scenery car still draws (m). A city-wide
    // instance group cannot be partially frustum-culled, so without this every
    // parked car in the city is submitted every frame, in both the colour and
    // shadow passes. 0 = no cull (the old behaviour).
    Real sceneryRadius = 450.0;
    // Physical-walker budget; see CityWalkerSystem. 0 or negative = unbounded
    // (the historical behaviour, kept so a measurement can A/B against it).
    int maxWalkerBodies = 200;
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
    // Build the NavGraph from every RoadNet in `world`, seed the CitySim, and
    // create the instance-group entities. `assets` may be null (tests): then the
    // groups carry a null MeshHandle and only the transforms are populated.
    // No-op (returns false) if the world holds no navigable roads.
    bool build(engine::World& world, engine::AssetManager* assets);

    // Advance the sim by `dt` seconds and re-bake every InstanceGroup.
    void step(engine::World& world, Real dt);

    bool built() const { return built_; }
    const CitySim& sim() const { return sim_; }
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
    // The stop-bar + lane-arrow paint mesh (R6c), rebuilt from the graph;
    // public so the gate can assert paint never leaves the carriageway.
    engine::RenderMesh buildRoadMarkings() const;
    CitySim& simMutable() { return sim_; }   // ADR-0062 bridge: release ejected drivers
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

    CityRenderParams params_;
    engine::NavGraph nav_;
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
    engine::Entity signalGroups_[3];   // lit lens, indexed by SignalState (Green/Yellow/Red)
    engine::Entity signalPostGroup_;   // the static pole+arm+head assemblies
    engine::Entity parkBayGroup_;      // curbside bay outline markings (R6b)
    engine::Entity roadMarkGroup_;     // stop bars + lane-turn arrows (R6c)
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
    // Parked cars that survived the distance cull on the last bake — how many
    // car bodies the frame actually pays for. This was computed and discarded,
    // which left the car share of the frame a guess with a 2.7x spread.
    int parkedDrawn_ = 0;
    std::vector<Real> prevCarSpeed_;
    mutable std::vector<engine::Vec3> smoothUp_;   // per-agent tilt low-pass
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
    std::vector<engine::Mat4> colliderStripBake_;  // prism rims (base + top loops)
    std::vector<engine::Mat4> colliderPostBake_;   // prism vertical corner posts
    std::vector<int> signalLinks_;     // approach links that carry a signal (cached)
    // BUILD-TIME furniture adoption (device: the city places poles, the sim
    // reacts): per-link SignalSite overrides published by the level loader's
    // StreetFurniture component. Empty when the level placed none — then this
    // system computes its own sites as before.
    std::vector<SignalSite> siteByLink_;
    std::vector<uint8_t> siteHasLink_;
    std::vector<engine::Vec2> crosswalkCenters_;   // one per junction approach (centre of band)
    std::function<double(double, double)> heightAt_;   // terrain drape (may be null)
    Real roadLift_ = 0.0;
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
    bool carsExternallyOwned_ = false;   // ADR-0062: a CityVehicleSystem owns the cars
    bool pedsExternallyOwned_ = false;   // ADR-0062: a CityWalkerSystem owns the peds
    std::vector<ExternalAgentPose> externalCarPoses_;   // real car poses for widgets
    std::vector<ExternalAgentPose> externalPedPoses_;   // real walker poses for widgets
};

}  // namespace citysim

#endif
