#ifndef RAYTRACER_ENGINE_TELEPORT_POSE_H
#define RAYTRACER_ENGINE_TELEPORT_POSE_H

#include "../../rt_math.h"

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
// The pose as the paste format ("%.2f %.2f %.2f %.2f %.2f").
std::string formatTeleportPose(const Vec3& position, double pitch, double yaw);

}  // namespace engine

#endif
