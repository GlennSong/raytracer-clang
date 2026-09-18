#ifndef RAYTRACER_ENGINE_SYSTEMS_ELEVATOR_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEMS_ELEVATOR_SYSTEM_H

#include "../system.h"
#include "../asset_manager.h"
#include "../physics/physics_world.h"
#include "../procgen/city/building_records.h"
#include "../procgen/city/core_plan.h"
#include <cstddef>
#include <map>
#include <unordered_map>
#include <vector>

namespace engine {

class PhysicsSystem;
class World;

// ELEVATORS (skyscrapers v2 M5/M6). A building with a core (core_plan.h) has
// a bank of hoistways; this system runs one CAB per hoistway of the building
// the player is in, as a REAL kinematic body — a floor, two sides, a back and
// a lit ceiling moved with moveKinematic, so the walker rides it through the
// ground-velocity transfer in PhysicsWorld::moveCharacter — plus the sliding
// HOISTWAY DOORS on the storeys around the player (kinematic leaves that
// close the shaft wherever the cab is not).
//
// Verbs: in front of a hoistway door, `elevator_call` (E) summons the
// nearest idle cab to that storey; inside the cab, `elevator_floor_up` /
// `elevator_floor_down` (Up / Down) pick a storey and `elevator_call` goes.
// The cab opens on arrival, dwells, closes; a trapezoidal profile (ACCEL to
// SPEED) between storeys. The floor panel is an ImGui strip (render()).
//
// One bank at a time (the building the player is in); everything is freed
// once they are RELEASE_M from the core. Headless-safe: no physics, no
// bodies; no renderer, still runs.
class ElevatorSystem : public System {
public:
    explicit ElevatorSystem(PhysicsSystem* physics = nullptr) : physics_(physics) {}

    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;        // input edges (frame-rate)
    void fixedUpdate(FrameContext& ctx) override;   // cabs, doors, bodies
    void render(FrameContext& ctx) override;        // the floor panel
    void onStop(FrameContext& ctx) override;

    static constexpr Real CALL_M = 2.5;      // in front of a hoistway door
    static constexpr Real RELEASE_M = 60.0;  // from the core: the bank is freed
    static constexpr Real SPEED = 3.0;       // m/s cruise
    static constexpr Real ACCEL = 1.5;       // m/s^2
    static constexpr Real DOOR_S = 1.2;      // leaf travel, seconds
    static constexpr Real DWELL_S = 6.0;     // doors open on arrival
    static constexpr int LEAF_WINDOW = 3;    // storeys of doors kept around the player
    // The cab must clear the door it is entered through (core_plan's kDoorH =
    // human::DOOR_HEIGHT = 2.7) with a little header, and still fit inside a
    // 3.2 m storey once its floor and ceiling slabs are counted.
    //
    // WIDTH is set by the door pocket, not by taste: a leaf is kLeafW = 0.55
    // and slides a full leaf clear of a 1.1 m opening, so the front wall
    // beside the opening must be at least 0.55 wide to swallow it --
    // CAB_W/2 - 0.55 >= 0.55, hence 2.2. At the old 1.8 the pocket was 0.35
    // and each leaf drove 0.2 m THROUGH the side wall every time the doors
    // opened, which is what Glenn saw: "the elevator door looks weird from
    // the inside". 2.2 + two 0.05 walls = 2.3, inside core_plan's 2.4 shaft.
    static constexpr Real CAB_W = 2.2, CAB_D = 1.6, CAB_H = 2.9;

    // What the HUD shows.
    struct Status {
        bool inCab = false;
        bool atDoor = false;
        bool moving = false;
        int floor = 0;      // the cab's storey (0 = ground)
        int selected = 0;   // the storey picked on the panel
        int floors = 0;     // storeys in the building
    };
    const Status& status() const { return status_; }
    std::size_t bankCount() const { return banks_.size(); }

    // Headless step for tests: the same logic as fixedUpdate with explicit
    // dependencies. `call` / `floorDelta` are this step's verbs.
    // `feetDrop` is how far the player's FEET sit below the position passed
    // in: halfHeight + radius of their capsule. It was a 0.7 constant, which
    // is the lab capsule (0.4 + 0.3); metro_v2_test ships 0.8 + 0.3 = 1.1, so
    // the storey came out a step high and in-cab detection was unreliable —
    // which reset the selected floor every tick and made the go button reopen
    // the doors instead of moving (Glenn, 2026-09-16).
    void step(World& world, PhysicsWorld* phys, AssetManager& assets, const Vec3& player, Real dt,
              bool call, int floorDelta, Real feetDrop = 0.7);
    // The cab's floor height (world) in bank `record`, hoistway `i`; NaN when absent.
    Real cabY(std::size_t record, std::size_t i) const;

private:
    enum class CabState { Idle, Opening, Open, Closing, Moving };
    struct Cab {
        Real y = 0;          // the cab floor's top, world
        Real vel = 0;
        int floor = 0;
        int target = 0;
        CabState state = CabState::Idle;
        Real doorT = 0;      // 0 closed .. 1 open
        Real dwell = 0;
        std::vector<PhysicsBodyId> bodies;
        std::vector<Entity> entities;
        std::vector<MeshHandle> meshes;
        std::vector<Vec3> local;   // part centres in the hoistway frame: (u, above the floor, v)
        // The cab's own DOORS (Glenn's walk, 2026-09-14: "after the door
        // closes it disappears and I can see the non-interior as we go up"):
        // two leaves on the cab front, index into `local`, sliding with the
        // hoistway leaves' doorT so a rider in a moving cab sees the cab.
        int doorLeft = -1, doorRight = -1;
    };
    struct Leaf {
        Entity entity;
        MeshHandle mesh;
        PhysicsBodyId body = INVALID_PHYSICS_BODY;
        int storey = 0, hoistway = 0;
        bool left = false;
    };
    struct Bank {
        CorePlan core;
        std::vector<Real> storeyY;   // slab tops, world
        std::vector<Cab> cabs;
        std::map<long long, Leaf> leaves;
        int selected = 0;
    };
    Bank& ensureBank(World& world, PhysicsWorld* phys, AssetManager& assets, const BuildingRecord& r,
                     std::size_t key);
    void releaseBank(World& world, PhysicsWorld* phys, AssetManager& assets, std::size_t key);
    void placeCab(World& world, PhysicsWorld* phys, const Bank& b, Cab& cab, const CoreShaft& hw, Real dt);
    void syncLeaves(World& world, PhysicsWorld* phys, AssetManager& assets, Bank& b, int playerStorey,
                    Real dt);
    static int storeyOf(const Bank& b, Real y, Real feetDrop = 0.7);

    PhysicsSystem* physics_ = nullptr;
    std::unordered_map<std::size_t, Bank> banks_;   // key: record index
    Status status_;
    bool callEdge_ = false;
    int floorDelta_ = 0;
};

}  // namespace engine

#endif
