#ifndef RAYTRACER_ENGINE_PLAYER_BODY_SYSTEM_H
#define RAYTRACER_ENGINE_PLAYER_BODY_SYSTEM_H

#include "../system.h"
#include "../block_body.h"
#include "../body_draw.h"

namespace engine {

struct PlayerPose;
struct Transform;

// THE PLAYER'S BODY POSED, in third person (Glenn, 2026-10-24: "3rd person might be fun as well"): the jointed
// body of engine/block_body.h (drawn through body_draw.h: blocks and skinned limbs) placed for what the body is
// doing -- against the wall reaching for the holds, prone in a crawl, under the canopy with the hands on the toggles, spread in the wingsuit, hanging off the rope -- and, when
// the body goes limp, wherever the ragdoll's bodies are. Walking and falling stay the walker mesh and its walk
// cycle (citysim's CityPlayerBodySystem, which stands down while this draws).
class PlayerBodySystem : public System {
public:
    void fixedUpdate(FrameContext& ctx) override;   // (with the capsule: drawn between ticks like it)
    // AT THE WHEEL (ADR-0252): sat in the vehicle as it is DRAWN this frame (between its steps), so the body never
    // lags the seat by a step -- VehicleSystem moves the car after this system's fixed step
    void render(FrameContext& ctx) override;

private:
    Entity parts_[BlockBody::kParts];
    BodyMeshes meshes_;   // the limbs' skinned tubes, by fold
    bool made_ = false;
    bool shown_ = false;
    bool driving_ = false;   // the body is in the driver's seat: render() places it
    void place(World& world, int part, const Vec3& at, const Quat& q, const Vec3& size);
    Vec3 facing_{0, 0, 1};   // held while there is nothing better to face
    Real clock_ = 0;
    // THE BOARD under the feet (skateboard or scooter), drawn in either view
    Entity board_[2];
    bool boardsMade_ = false;
    void drawBoards(FrameContext& ctx, const PlayerPose* p, const Transform* body, Real lift);
};

}  // namespace engine

#endif
