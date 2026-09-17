#ifndef RAYTRACER_ENGINE_SYSTEMS_BUILDING_INTERIOR_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEMS_BUILDING_INTERIOR_SYSTEM_H

#include "../system.h"
#include "../asset_manager.h"
#include "../physics/physics_world.h"
#include "../procgen/city/building_records.h"
#include "../procgen/city/core_plan.h"   // the storey window rides the core (M5)
#include <array>
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
    // dependencies (no clock; renderer nullable — without it, surface maps
    // stay unbound and parts shade flat).
    void step(World& world, PhysicsWorld* phys, AssetManager& assets,
              Renderer* renderer, const Vec3& player);

    std::size_t residentCount() const { return resident_.size(); }

    static constexpr double APPROACH_M = 6.0;   // build when this near a door
    static constexpr double RELEASE_M = 40.0;   // free when this far from all
    static constexpr std::size_t MAX_RESIDENT = 5;
    static constexpr std::uint64_t FREE_EVERY = 120;   // steps between GPU frees
    // The storey WINDOW (skyscrapers v2 M5): a building of up to WHOLE_UP_TO
    // storeys is grown whole; a taller one streams [f - WINDOW_BELOW,
    // f + WINDOW_ABOVE) around the player's storey f (the lobby's first
    // three storeys while they are still outside), regrown when f nears an
    // edge of the window — except while riding a cab, when the arrival is
    // the moment the window catches up.
    static constexpr int WHOLE_UP_TO = 8;
    static constexpr int WINDOW_BELOW = 2;
    static constexpr int WINDOW_ABOVE = 3;
    // The storey a walker at `y` (capsule centre) stands in, 0 = ground.
    static int storeyOf(const BuildingRecord& r, Real y, Real feetDrop = 0.7);
    // The window a resident holds ([k0, k1); k1 < 0 = whole); k1 = 0 when absent.
    void residentWindow(std::size_t key, int& k0, int& k1) const;

private:
    struct Resident {
        std::vector<Entity> entities;
        std::vector<MeshHandle> meshes;
        PhysicsBodyId body = INVALID_PHYSICS_BODY;
        int k0 = 0, k1 = -1;   // the storey window grown
        CorePlan core;         // lazily derived (the hoistway test while riding)
        bool coreKnown = false;
    };
    void build(World& world, PhysicsWorld* phys, AssetManager& assets,
               Renderer* renderer, const BuildingRecord& r, std::size_t key,
               int k0, int k1);
    // Destroys entities and the body at once; queues the GPU meshes for the
    // rate-limited free.
    void release(World& world, PhysicsWorld* phys, std::size_t key);

    PhysicsSystem* physics_ = nullptr;
    std::unordered_map<std::size_t, Resident> resident_;   // key: record index
    // Baked surface texture sets (WoodSiding, Concrete, ...), one per surface
    // id, session-lived like the loader's SurfaceTexCache. Without this bind
    // the streamed parts fell back to the in-shader procedural: the "second
    // floor texture is black / no bump / 45-degree diagonal" report.
    std::unordered_map<int, std::array<TextureHandle, 4>> surfTex_;
    std::vector<MeshHandle> freeQueue_;
    std::uint64_t stepCount_ = 0;
    std::uint64_t lastFree_ = 0;
    Real lastPlayerY_ = 0;
    bool havePlayerY_ = false;
};

}  // namespace engine

#endif
