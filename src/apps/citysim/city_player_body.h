#ifndef RAYTRACER_APPS_CITYSIM_CITY_PLAYER_BODY_H
#define RAYTRACER_APPS_CITYSIM_CITY_PLAYER_BODY_H

#include "../../engine/system.h"
#include "../../engine/components.h"              // engine::Transform (prevPose_)
#include "../../engine/procgen/city/polygon.h"   // engine::Vec2 (facing_)
#include "../../renderer/renderer.h"              // engine::MeshHandle

#include <vector>

namespace citysim {

// The player IS a person: while the on-foot camera is in third person
// ("playerThirdPerson" in Settings, toggled by PlayerSystem on V), the player
// entity renders as the SAME articulated person the city walkers use
// (buildPersonMesh) in the fixed, distinctive playerOutfit(), its walk cycle
// driven by the body's real horizontal speed through the shared walkPoseIndex —
// so the player and the crowd stride identically. The character CAPSULE stays
// the collider; only the visual changes, the mesh's 1.8 m envelope scaled to
// the capsule's real height so the feet meet the ground. In FIRST person the
// Renderable is removed — no head in the camera. Knockdown does not apply to
// the player; only the stand + walk poses are drawn.
//
// Decoupled from PlayerSystem via the settings flag (no engine-system
// dependency); registered with the physics systems, since only there does the
// player capsule move. The pose logic is headless-tested (walkPoseIndex and
// the outfit table in tests/test_city_render.cpp); the system itself needs the
// viewer and is UNVERIFIED on device.
class CityPlayerBodySystem : public engine::System {
public:
    void fixedUpdate(engine::FrameContext& ctx) override;
    void onStop(engine::FrameContext& ctx) override;

private:
    engine::MeshHandle poseMesh(engine::FrameContext& ctx, int pose,
                                engine::Real scale);

    std::vector<engine::MeshHandle> poseMeshes_;   // player pose set, lazy
    bool bodyOn_ = false;                // we added the player's Renderable
    engine::Real stride_ = 0;            // walk-cycle phase (advances with speed)
    engine::Vec2 facing_{0, 1};          // last real travel direction (render yaw)
    engine::Vec3 lastPos_{0, 0, 0};      // capsule position last tick (speed src)
    engine::Transform prevPose_;         // last drawn pose (render interpolation)
};

}  // namespace citysim

#endif
