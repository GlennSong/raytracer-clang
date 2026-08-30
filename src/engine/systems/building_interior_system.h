#ifndef RAYTRACER_ENGINE_SYSTEMS_BUILDING_INTERIOR_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEMS_BUILDING_INTERIOR_SYSTEM_H

#include "../system.h"
#include "../asset_manager.h"
#include "../physics/physics_world.h"
#include "../procgen/city/building_records.h"
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine {

class PhysicsSystem;
class World;

// Streams building INTERIORS (ADR-0080 Phase 2). A building's inside exists
// only while someone is there or arriving: the system reads the CityBuildings
// records (plan + doors + the ~250 B BuildingParams regen key), decides
// "inside" (XZ in the plan, y in the prism band) and "approaching" (within
// APPROACH_M in FRONT of a door — the dot with the door normal is what makes
// walking PAST a shop not build it), and grows the interior deterministically
// with growInterior — floors, inner walls, the stairwell — plus one Jolt mesh
// body for the walkable parts. The approach trigger means floors exist before
// the threshold is crossed. Release is hysteretic (RELEASE_M from every door)
// and GPU frees are rate-limited (VulkanRenderer::removeMesh waits on the
// device — never more than one free per FREE_EVERY steps, never on a step
// that built). The terrain collider window (terrain_lod_system.cpp) is the
// pattern: bodies owned directly, one build per fixed step, the record the
// player is inside exempt from the budget.
class BuildingInteriorSystem : public System {
public:
    explicit BuildingInteriorSystem(PhysicsSystem* physics = nullptr)
        : physics_(physics) {}

    void fixedUpdate(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    // Headless step for tests: the same logic as fixedUpdate with explicit
    // dependencies (no renderer, no clock).
    void step(World& world, PhysicsWorld* phys, AssetManager& assets,
              const Vec3& player);

    std::size_t residentCount() const { return resident_.size(); }

    static constexpr double APPROACH_M = 6.0;   // build when this near a door
    static constexpr double RELEASE_M = 40.0;   // free when this far from all
    static constexpr std::size_t MAX_RESIDENT = 5;
    static constexpr std::uint64_t FREE_EVERY = 120;   // steps between GPU frees

private:
    struct Resident {
        std::vector<Entity> entities;
        std::vector<MeshHandle> meshes;
        PhysicsBodyId body = INVALID_PHYSICS_BODY;
    };
    void build(World& world, PhysicsWorld* phys, AssetManager& assets,
               const BuildingRecord& r, std::size_t key);
    // Destroys entities and the body at once; queues the GPU meshes for the
    // rate-limited free.
    void release(World& world, PhysicsWorld* phys, std::size_t key);

    PhysicsSystem* physics_ = nullptr;
    std::unordered_map<std::size_t, Resident> resident_;   // key: record index
    std::vector<MeshHandle> freeQueue_;
    std::uint64_t stepCount_ = 0;
    std::uint64_t lastFree_ = 0;
};

}  // namespace engine

#endif
