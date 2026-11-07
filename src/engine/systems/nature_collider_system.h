#ifndef RAYTRACER_ENGINE_NATURE_COLLIDER_SYSTEM_H
#define RAYTRACER_ENGINE_NATURE_COLLIDER_SYSTEM_H

#include "../system.h"
#include "../physics/physics_world.h"   // PhysicsBodyId
#include "../control/control_commands.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine {

class PhysicsSystem;

// THE NATURE'S COLLIDERS (#55, Glenn: "collision detection for the rocks and trees"). A forest is half a
// million trees and rocks -- far too many bodies -- so the loader stores them as plain shapes, binned on a
// 32 m grid (NatureColliders), and this keeps static bodies only in the cells around the player (or the
// car it drives), prefetched along its velocity, added nearest first under a per-step budget, and dropped
// as it leaves. Trunks are standing capsules, rocks oriented boxes.
//
// STREET PIECES (ADR-0354): the kerb's trees and lamp posts stream in the same way, and so do the pieces a car knocks
// loose -- bins, racks, news boxes, hydrants (street_props.lua). A shape with a `piece` is breakable: its body lets a
// car through at the kind's breaking speed, and this hides the piece's drawn instance and drops a loose body in its
// place (a hydrant's stub then sprays). A broken piece never streams back.
struct NatureCollider {
    Vec3 centre;
    Quat orientation;
    Vec3 half;            // box half extents; a capsule: x radius, y half height (cylinder part)
    bool capsule = false;
    int piece = -1;       // NatureColliders::pieces, or -1: it stops whatever hits it
};
struct NatureColliders {
    struct PieceKind {
        MeshHandle mesh;          // centred on its box (the loose body's)
        RenderMaterial material;
        Real breaksAt = 2.0;
        Real mass = 30;           // kg, loose
        std::string spray;        // the stub's effect once it is knocked off (empty: none)
        Real sprayRate = 0, sprayUp = 0;
    };
    struct Piece {
        int kind = 0;
        Entity group;             // the InstanceGroup drawing it...
        std::size_t instance = 0; // ...and which of its instances
        bool broken = false;
    };
    std::vector<PieceKind> pieceKinds;
    std::vector<Piece> pieces;
    Real knockPush = 0.6, knockUp = 1.0, knockUpPerSpeed = 0.05;   // how a piece leaves (street_props.lua `knock`)
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
    void onStart(FrameContext& ctx) override;
    void fixedUpdate(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    void breakPieces(FrameContext& ctx, NatureColliders& nc);
    PhysicsSystem& physics_;
    std::unordered_map<int64_t, std::vector<PhysicsBodyId>> live_;   // cell -> its bodies
    std::unordered_map<PhysicsBodyId, int> shapeOf_;                 // a live body -> its shape
    EventBus::Subscription onCollision_;
    ControlCommands::Binding knockCmd_;   // `pieces knock`: the live piece nearest the player, as a car would
    bool knockNearest_ = false;
    struct Knock {
        PhysicsBodyId a = INVALID_PHYSICS_BODY, b = INVALID_PHYSICS_BODY;
        Vec3 normal;
        Real speed = 0;
    };
    std::vector<Knock> knocks_;
    struct LooseKick { Entity e; Vec3 velocity; Real mass = 30; };
    std::vector<LooseKick> looseKick_;   // loose pieces waiting for their body: their weight and their push
    std::vector<Entity> loose_;                        // every loose piece, oldest first
    struct Spray { Entity e; Real left = 0; };
    std::vector<Spray> sprays_;
};

}  // namespace engine

#endif
