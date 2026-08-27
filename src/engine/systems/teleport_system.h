#ifndef RAYTRACER_ENGINE_TELEPORT_SYSTEM_H
#define RAYTRACER_ENGINE_TELEPORT_SYSTEM_H

#include "../system.h"
#include "../camera/fly_camera_controller.h"
#include "teleport_pose.h"

#include <string>
#include <vector>

namespace engine {

class PhysicsSystem;

// TELEPORT (device: "a debug option that could transport the player to an
// x, y, z position ... copy and paste it into imgui and jump to that
// location ... copy and paste coordinates out of imgui and give them to
// you"). One pose format everywhere — `x y z pitch yaw`, the same numbers
// `camera?`/`where?` print and `camera`/`teleport` accept on the socket —
// so a location moves between Glenn, the panel, the log and this session
// by paste. The panel (Debug → Teleport) shows the pose here with copy
// buttons, takes a pasted pose (tolerant of commas, parens, `x=` labels,
// the camera? reply; 2 numbers = x z on the ground), moves the PLAYER
// (the physics character, camera re-attached) or just the fly camera,
// keeps named bookmarks in settings, and lists the city map's conflict
// places with Go buttons.
struct TeleportBookmark {
    std::string name;
    Vec3 position;
    double pitch = 0, yaw = 0;
};

class TeleportSystem : public System {
public:
    // `physics` may be null (the editor): then only the camera moves.
    TeleportSystem(FlyCameraController& camera, PhysicsSystem* physics)
        : camera_(camera), physics_(physics) {}

    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    // Move the player there (or the camera when there is no player), with
    // the camera re-attached; returns what happened, for the log/socket.
    std::string teleport(FrameContext& ctx, const TeleportPose& pose, bool playerToo);
    // The current viewpoint as a TeleportPose (from the render view).
    static TeleportPose poseHere(const FrameContext& ctx);

private:
    FlyCameraController& camera_;
    PhysicsSystem* physics_;
    char pasteBuf_[256] = {};
    char nameBuf_[64] = {};
    std::string lastResult_;
    std::vector<TeleportBookmark> bookmarks_;
    bool groundAt(FrameContext& ctx, double x, double z, double& y) const;
    void loadBookmarks(FrameContext& ctx);
    void saveBookmarks(FrameContext& ctx);
};

}  // namespace engine

#endif
