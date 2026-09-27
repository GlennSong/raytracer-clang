#ifndef RAYTRACER_ENGINE_NATURE_COLLIDER_SYSTEM_H
#define RAYTRACER_ENGINE_NATURE_COLLIDER_SYSTEM_H

#include "../system.h"
#include "../physics/physics_world.h"   // PhysicsBodyId

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace engine {

class PhysicsSystem;

// THE NATURE'S COLLIDERS (#55, Glenn: "collision detection for the rocks and trees"). A forest is half a
// million trees and rocks -- far too many bodies -- so the loader stores them as plain shapes, binned on a
// 32 m grid (NatureColliders), and this keeps static bodies only in the cells around the player (or the
// car it drives), prefetched along its velocity, added nearest first under a per-step budget, and dropped
// as it leaves. Trunks are standing capsules, rocks oriented boxes.
struct NatureCollider {
    Vec3 centre;
    Quat orientation;
    Vec3 half;            // box half extents; a capsule: x radius, y half height (cylinder part)
    bool capsule = false;
};
struct NatureColliders {
    std::vector<NatureCollider> shapes;
    std::unordered_map<int64_t, std::vector<int>> cells;
    static constexpr double kCell = 32.0;
    static int64_t key(int i, int j) { return (static_cast<int64_t>(i) << 32) ^ static_cast<uint32_t>(j); }
    void add(const NatureCollider& c) {
        const int i = static_cast<int>(std::floor(c.centre.x / kCell)), j = static_cast<int>(std::floor(c.centre.z / kCell));
        cells[key(i, j)].push_back(static_cast<int>(shapes.size()));
        shapes.push_back(c);
    }
};

class NatureColliderSystem : public System {
public:
    explicit NatureColliderSystem(PhysicsSystem& physics) : physics_(physics) {}
    void fixedUpdate(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    PhysicsSystem& physics_;
    std::unordered_map<int64_t, std::vector<PhysicsBodyId>> live_;   // cell -> its bodies
};

}  // namespace engine

#endif
