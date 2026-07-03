#ifndef RAYTRACER_ENGINE_TRAFFIC_SYSTEM_H
#define RAYTRACER_ENGINE_TRAFFIC_SYSTEM_H

#include "../system.h"
#include "../ai/agent_sim.h"
#include "../ai/nav_graph.h"
#include <functional>

namespace engine {

// The ECS bridge that brings the navigation foundation (ADR-0059) to life: it
// builds a NavGraph from the level's RoadNet entities, runs a deterministic
// AgentSim of cars + pedestrians over it, and bakes their poses into two
// InstanceGroups (one per kind) so RenderSystem draws the whole crowd as two
// instanced batches. The sim is rebuilt lazily on the first fixedUpdate after a
// level loads (so the RoadNets exist) and stepped every fixed step thereafter.
//
// build()/step() take a World directly (like PhysicsSystem/MotionSystem) so the
// spawn + pose-sync logic is unit-tested headless; mesh upload is the only part
// that needs the AssetManager (skipped, with null mesh handles, when absent).
struct TrafficParams {
    int cars = 40;
    int pedestrians = 40;
    uint32_t seed = 1;
    Real hoursPerSecond = 0.05;   // sim-clock hours advanced per real second
    Vec3 carSize{2.0, 1.4, 4.2};  // x = width, y = height, z = length (travel axis)
    Vec3 pedSize{0.5, 1.8, 0.5};
};

class TrafficSystem : public System {
public:
    explicit TrafficSystem(const TrafficParams& params = {}) : params_(params) {}

    void fixedUpdate(FrameContext& ctx) override;

    // --- testable core (no FrameContext) -----------------------------------
    // Build the NavGraph from every RoadNet in `world`, seed the AgentSim, and
    // create the two InstanceGroup entities. `assets` may be null (tests): then
    // the groups carry a null MeshHandle and only the transforms are populated.
    // No-op (returns false) if the world holds no navigable roads.
    bool build(World& world, AssetManager* assets);

    // Advance the sim by `dt` seconds and re-bake the InstanceGroup transforms.
    void step(World& world, Real dt);

    bool built() const { return built_; }
    const AgentSim& sim() const { return sim_; }
    const NavGraph& nav() const { return nav_; }
    Entity carGroup() const { return carGroup_; }
    Entity pedGroup() const { return pedGroup_; }

private:
    void syncGroups(World& world);
    Mat4 poseOf(const Agent& a) const;

    TrafficParams params_;
    NavGraph nav_;
    AgentSim sim_;
    Entity carGroup_;
    Entity pedGroup_;
    std::function<double(double, double)> heightAt_;   // terrain drape (may be null)
    Real roadLift_ = 0.0;
    bool built_ = false;
};

}  // namespace engine

#endif
