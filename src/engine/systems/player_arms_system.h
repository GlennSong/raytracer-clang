#ifndef RAYTRACER_ENGINE_PLAYER_ARMS_SYSTEM_H
#define RAYTRACER_ENGINE_PLAYER_ARMS_SYSTEM_H

#include "../system.h"
#include "../player_pose.h"
#include "../body_draw.h"

namespace engine {

// THE PLAYER'S BLOCK ARMS in first person (Glenn, 2026-10-24: "It might be funny to see my block arms as I move
// and thus when I pull myself up I'd see the arms do that"). The body's own arms (ADR-0254: the jointed body's
// skinned arm tubes, the elbow bent by two-bone IK) and two hands, in the player's own outfit, reaching from the
// shoulders to where the hands are: swinging with the stride, pumping on a run, out
// for balance in the air, crawling through the water, and HELD where they grip -- the ledge being pulled up onto,
// the wall being climbed, a ladder's rails; up on a parachute's toggles, spread in a wingsuit -- and the gun when it's drawn (slot 2: right hand on the grip, left
// under the barrel; assets/scripts/gun.lua puts the gun away while the hands are busy). Reads PlayerPose (engine/player_pose.h); runs after every camera
// writer so the hands sit on this frame's view.
class PlayerArmsSystem : public System {
public:
    void update(FrameContext& ctx) override;

private:
    // sleeve L, sleeve R, hand L, hand R, wing L, wing R (the wingsuit's), the canopy, its four lines, the hook,
    // the watch's case and its strap
    static constexpr int kParts = 14;
    Entity parts_[kParts];
    Entity rope_[PlayerPose::kRopePoints - 1];   // the rope's links, one block each
    bool ropeShown_ = false;
    Vec3 handWorld_[2];    // where the hands were drawn this frame (the chute's lines run to them)
    bool made_ = false;
    bool shown_ = false;
    Vec3 local_[2];        // each hand in camera space (right, up, forward), eased
    bool haveLocal_ = false;
    bool canopyShown_ = false;
    bool watchShown_ = false;
    BodyMeshes arms_;   // the body's arm tubes, by fold (body_draw.h; shared with PlayerBodySystem's by key)
};

}  // namespace engine

#endif
