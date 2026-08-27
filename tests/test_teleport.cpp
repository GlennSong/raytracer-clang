#include "test_framework.h"

#include "../src/engine/systems/teleport_pose.h"

#include <cmath>

using namespace engine;

// THE POSE FORMAT (device: "copy and paste it into imgui and jump to that
// location ... copy and paste coordinates out of imgui and give them to
// you"). One format, read tolerantly: plain numbers, commas, parens,
// labels, the socket's own camera? reply; two numbers mean "on the ground".

TEST_CASE(teleport_pose_reads_every_way_a_location_gets_pasted) {
    TeleportPose p;
    CHECK(parseTeleportPose("-123.0 4.08 -289.0 -24.3 161.1", p) == 5);
    CHECK_APPROX(p.position.x, -123.0, 1e-9);
    CHECK_APPROX(p.position.y, 4.08, 1e-9);
    CHECK_APPROX(p.position.z, -289.0, 1e-9);
    CHECK_APPROX(p.pitch, -24.3, 1e-9);
    CHECK_APPROX(p.yaw, 161.1, 1e-9);
    CHECK(p.hasLook && !p.groundY);
    // The camera? reply, verbatim.
    CHECK(parseTeleportPose("ok eye=-123.00,4.08,-289.00 pitch=-24.30 yaw=161.10 frame=6", p) == 5);
    CHECK_APPROX(p.position.z, -289.0, 1e-9);
    CHECK_APPROX(p.yaw, 161.1, 1e-9);
    // A census place: "(-887.4, -111.9)" → x z on the ground.
    CHECK(parseTeleportPose("(-887.4, -111.9)", p) == 2);
    CHECK(p.groundY && !p.hasLook);
    CHECK_APPROX(p.position.x, -887.4, 1e-9);
    CHECK_APPROX(p.position.z, -111.9, 1e-9);
    // Three numbers: a point, no look.
    CHECK(parseTeleportPose("x=10, y=20, z=30", p) == 3);
    CHECK(!p.groundY && !p.hasLook);
    CHECK_APPROX(p.position.y, 20.0, 1e-9);
    // Not a pose.
    CHECK(parseTeleportPose("hello", p) == 0);
    CHECK(parseTeleportPose("42", p) == 0);
    // Round trip through the paste format.
    const std::string s = formatTeleportPose(Vec3(1.5, 2.25, -3.0), -12.5, 270.0);
    CHECK(s == "1.50 2.25 -3.00 -12.50 270.00");
    CHECK(parseTeleportPose(s, p) == 5);
    CHECK_APPROX(p.position.y, 2.25, 1e-9);
    CHECK_APPROX(p.yaw, 270.0, 1e-9);
}
