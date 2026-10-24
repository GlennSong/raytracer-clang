#ifndef RAYTRACER_ENGINE_PLAYER_ARMS_SYSTEM_H
#define RAYTRACER_ENGINE_PLAYER_ARMS_SYSTEM_H

#include "../system.h"

namespace engine {

// THE PLAYER'S BLOCK ARMS in first person (Glenn, 2026-10-24: "It might be funny to see my block arms as I move
// and thus when I pull myself up I'd see the arms do that"). Two sleeves and two hands, boxes in the player's
// own outfit, drawn from the shoulders to where the hands are: swinging with the stride, pumping on a run, out
// for balance in the air, crawling through the water, and HELD where they grip -- the ledge being pulled up onto,
// the wall being climbed, a ladder's rails; up on a parachute's toggles, spread in a wingsuit -- and the gun when it's drawn (slot 2: right hand on the grip, left
// under the barrel; assets/scripts/gun.lua puts the gun away while the hands are busy). Reads PlayerPose (engine/player_pose.h); runs after every camera
// writer so the hands sit on this frame's view.
class PlayerArmsSystem : public System {
public:
    void update(FrameContext& ctx) override;

private:
    // sleeve L, sleeve R, hand L, hand R, wing L, wing R (the wingsuit's), the canopy, its four lines
    static constexpr int kParts = 11;
    Entity parts_[kParts];
    Vec3 handWorld_[2];    // where the hands were drawn this frame (the chute's lines run to them)
    bool made_ = false;
    bool shown_ = false;
    Vec3 local_[2];        // each hand in camera space (right, up, forward), eased
    bool haveLocal_ = false;
    bool canopyShown_ = false;
    int slot_ = 1;         // the tool in hand, tracked like the gun script does: 1 bare hands, 2 the gun
};

}  // namespace engine

#endif
