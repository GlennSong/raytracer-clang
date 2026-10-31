#ifndef RAYTRACER_ENGINE_TELEPORT_POSE_H
#define RAYTRACER_ENGINE_TELEPORT_POSE_H

#include "../../rt_math.h"
#include "../properties.h"

#include <string>

namespace engine {

// THE POSE FORMAT (device: "copy and paste it into imgui and jump to that
// location ... copy and paste coordinates out of imgui and give them to
// you"): `x y z pitch yaw` (metres, degrees, yaw 0 = north/-z) — the line
// the Teleport panel copies and pastes, `where?` prints, `teleport` takes.
// Pure, engine-core (no physics), so the tests and the socket share it.
struct TeleportPose {
    Vec3 position;
    double pitch = 0, yaw = 0;
    bool hasLook = false;    // pitch/yaw given
    bool groundY = false;    // y not given: stand on the ground at (x, z)
};

// Parse "x y z [pitch yaw]" / "x z" from free text: every number in it, in
// order (commas, parens, `x=` labels and the camera? reply all read).
// 2 → x z (groundY), 3 → x y z, 4 → + pitch, 5+ → + yaw. Returns the count
// of numbers consumed (0 = nothing usable).
int parseTeleportPose(const std::string& text, TeleportPose& out);

// TeleportRequest (ADR-0253): move the player there (the fly camera when there is none). TeleportSystem hears it
// and answers `teleport?` with where it landed. The socket's `teleport`, the camera's fast travel on re-attach,
// and Lua (events.publish("TeleportRequest", {position = {x, y, z}, ground = true})) publish it.
struct TeleportRequest {
    TeleportPose pose;
};
inline void describeProperties(TeleportRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("position").units("m"), r.pose.position);
    v.field(FieldMeta("pitch").units("deg"), r.pose.pitch);
    v.field(FieldMeta("yaw").units("deg"), r.pose.yaw);
    v.field(FieldMeta("look"), r.pose.hasLook);      // pitch and yaw given
    v.field(FieldMeta("ground"), r.pose.groundY);    // stand on the ground at position's x z
}
// TeleportPanelRequest: unfold the Debug window's Teleport section once (a probe can shoot the panel; a header
// cannot be clicked from outside). The socket's `teleport open`; TeleportSystem hears it.
struct TeleportPanelRequest {};
inline void describeProperties(TeleportPanelRequest&, PropertyVisitor&) {}
// The pose as the paste format ("%.2f %.2f %.2f %.2f %.2f").
std::string formatTeleportPose(const Vec3& position, double pitch, double yaw);

}  // namespace engine

#endif
