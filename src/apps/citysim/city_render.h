#ifndef RAYTRACER_APPS_CITYSIM_CITY_RENDER_H
#define RAYTRACER_APPS_CITYSIM_CITY_RENDER_H

#include "../../engine/system.h"
#include "../../engine/ai/nav_graph.h"
#include "city_sim.h"
#include <functional>

namespace citysim {

// Build the fleet body mesh for slot `slot` (wraps into the fleet) — the same
// vertex-coloured car the instanced renderer uses, so a PROMOTED physical
// Vehicle (ADR-0061: the player commandeers an ambient car) looks identical to
// the instanced car it replaces. `withWheels` false for that promotion —
// VehicleSystem gives a real Vehicle physics wheels. x=width, y=height,
// z=length (travel +Z).
engine::RenderMesh fleetCarMesh(int slot, bool withWheels = true);

// The ECS render bridge for the agent-based city simulation (ADR-0059 Phase 6).
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
    int cars = 40;
    int pedestrians = 40;
    uint32_t seed = 1;
    Real hoursPerSecond = 0.05;            // sim-clock hours advanced per real second
    Real perceptionReliability = 0.97;     // <1 -> agents occasionally err (ADR-0059)
    engine::Vec3 carSize{1.8, 1.3, 4.2};   // matches the player sedan (W,H,L); +Z = travel
    engine::Vec3 pedSize{0.5, 1.8, 0.5};
    Real signalLensSize = 0.34;            // lit emissive lens cube edge (m)
    bool debugWidgets = false;             // draw each agent's footprint + trajectory
};

class CityRenderSystem : public engine::System {
public:
    explicit CityRenderSystem(const CityRenderParams& params = {})
        : params_(params), debugWidgets_(params.debugWidgets) {}

    void onStart(engine::FrameContext& ctx) override;
    void update(engine::FrameContext& ctx) override;   // per-frame: debug-widget toggle
    void fixedUpdate(engine::FrameContext& ctx) override;

    // When a CityVehicleSystem owns the NPC cars as real physics Vehicles (ADR-0061),
    // this render bridge must NOT also draw them as instanced kinematic boxes (that
    // would double every car). Call before build(): it skips creating/baking the car
    // instance groups; peds, signals, and crosswalks stay owned here. The CitySim
    // still runs as the PLANNER (its ghosts drive the AgentDriver commands).
    void setCarsExternallyOwned(bool on) { carsExternallyOwned_ = on; }
    bool carsExternallyOwned() const { return carsExternallyOwned_; }

    // Same handoff for PEDESTRIANS (ADR-0061): when a CityWalkerSystem owns them
    // as physics characters, this bridge stops baking the instanced ped boxes
    // (they'd draw twice) — the CitySim keeps planning; walkers follow bodies.
    void setPedsExternallyOwned(bool on) { pedsExternallyOwned_ = on; }
    bool pedsExternallyOwned() const { return pedsExternallyOwned_; }

    // The physical car poses (ADR-0061), fed each step by the vehicle bridge so
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
    CitySim& simMutable() { return sim_; }   // ADR-0061 bridge: release ejected drivers
    const engine::NavGraph& nav() const { return nav_; }
    // Cars are split across several instance groups, one per body/colour variant
    // (an InstanceGroup shares one mesh, so variety needs multiple groups).
    const std::vector<engine::Entity>& carGroups() const { return carGroups_; }
    engine::Entity carGroup() const { return carGroups_.empty() ? engine::Entity{} : carGroups_[0]; }
    engine::Entity pedGroup() const { return pedGroup_; }
    engine::Entity signalGroup(SignalState s) const { return signalGroups_[static_cast<int>(s)]; }
    engine::Entity signalPostGroup() const { return signalPostGroup_; }
    engine::Entity crosswalkGroup() const { return crosswalkGroup_; }
    const std::vector<engine::Vec2>& crosswalkCenters() const { return crosswalkCenters_; }
    // Debug widgets (ADR-0060): per-agent ground footprint coloured by behaviour
    // state, and a forward trajectory arrow. Empty unless params.debugWidgets.
    engine::Entity footprintGroup(Agent::State s) const { return footprintGroups_[static_cast<int>(s)]; }
    engine::Entity forwardGroup() const { return forwardGroup_; }
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

private:
    void syncGroups(engine::World& world);
    engine::Mat4 agentPose(const Agent& a) const;   // box sized by a.mode (car/ped)
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
    engine::Entity pedGroup_;
    engine::Entity signalGroups_[3];   // lit lens, indexed by SignalState (Green/Yellow/Red)
    engine::Entity signalPostGroup_;   // the static pole+arm+head assemblies
    engine::Entity crosswalkGroup_;    // baked zebra decals at junction mouths
    // debug ground rings, one per Agent::State (indexed by it)
    engine::Entity footprintGroups_[static_cast<int>(Agent::State::Count)]{};
    engine::Entity forwardGroup_{};          // debug forward-trajectory arrows
    std::vector<int> signalLinks_;     // approach links that carry a signal (cached)
    std::vector<engine::Vec2> crosswalkCenters_;   // one per junction approach (centre of band)
    std::function<double(double, double)> heightAt_;   // terrain drape (may be null)
    Real roadLift_ = 0.0;
    bool built_ = false;
    bool debugWidgets_ = false;   // runtime toggle (init from params; key flips it)
    bool carsExternallyOwned_ = false;   // ADR-0061: a CityVehicleSystem owns the cars
    bool pedsExternallyOwned_ = false;   // ADR-0061: a CityWalkerSystem owns the peds
    std::vector<ExternalAgentPose> externalCarPoses_;   // real car poses for widgets
    std::vector<ExternalAgentPose> externalPedPoses_;   // real walker poses for widgets
};

}  // namespace citysim

#endif
