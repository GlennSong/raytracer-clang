#include "test_framework.h"

#include "../src/apps/citysim/city_spectate.h"
#include "../src/apps/citysim/city_sim.h"

#include <cmath>
#include <vector>

using namespace engine;
using namespace citysim;

namespace {

// A bare Agent with just the fields the pure spectate helpers read. The sim
// builds full agents; here we only need mode / released / playerControlled /
// heading, so hand-rolling keeps the tests independent of a NavGraph.
Agent makeAgent(Agent::Mode mode = Agent::Mode::Driver, bool released = false,
                bool playerControlled = false) {
    Agent a;
    a.mode = mode;
    a.released = released;
    a.playerControlled = playerControlled;
    return a;
}

}  // namespace

TEST_CASE(spectatable_skips_released_and_player_agents) {
    CHECK(isSpectatable(makeAgent()));
    CHECK(!isSpectatable(makeAgent(Agent::Mode::Driver, /*released=*/true)));
    CHECK(!isSpectatable(
        makeAgent(Agent::Mode::Pedestrian, /*released=*/false, /*player=*/true)));
    // A walking (non-released, non-player) agent is spectatable too.
    CHECK(isSpectatable(makeAgent(Agent::Mode::Pedestrian)));
}

TEST_CASE(next_spectatable_cycles_and_wraps) {
    std::vector<Agent> agents = { makeAgent(), makeAgent(), makeAgent() };
    // Forward wraps 0 -> 1 -> 2 -> 0.
    CHECK(nextSpectatable(agents, 0, +1) == 1);
    CHECK(nextSpectatable(agents, 1, +1) == 2);
    CHECK(nextSpectatable(agents, 2, +1) == 0);
    // Backward wraps 0 -> 2 -> 1 -> 0.
    CHECK(nextSpectatable(agents, 0, -1) == 2);
    CHECK(nextSpectatable(agents, 2, -1) == 1);
    CHECK(nextSpectatable(agents, 1, -1) == 0);
}

TEST_CASE(next_spectatable_skips_released_agents) {
    std::vector<Agent> agents = {
        makeAgent(),                                   // 0 ok
        makeAgent(Agent::Mode::Driver, /*rel=*/true),  // 1 released
        makeAgent(Agent::Mode::Driver, /*rel=*/true),  // 2 released
        makeAgent(),                                   // 3 ok
    };
    // From 0 forward, skip 1 and 2, land on 3.
    CHECK(nextSpectatable(agents, 0, +1) == 3);
    // From 3 forward, wrap past 1/2 back to 0.
    CHECK(nextSpectatable(agents, 3, +1) == 0);
    // From 3 backward, skip 2/1, land on 0.
    CHECK(nextSpectatable(agents, 3, -1) == 0);
    // A player-controlled ghost is skipped just like a released one.
    agents[3].playerControlled = true;
    CHECK(nextSpectatable(agents, 0, +1) == 0);   // only 0 remains -> stays put
}

TEST_CASE(next_spectatable_edge_cases) {
    // Empty city: no target, in either direction.
    std::vector<Agent> none;
    CHECK(nextSpectatable(none, -1, +1) == -1);
    CHECK(nextSpectatable(none, 0, -1) == -1);

    // One spectatable agent: any cycle returns it (never -1, never a wrap-off).
    std::vector<Agent> one = { makeAgent() };
    CHECK(nextSpectatable(one, -1, +1) == 0);   // nothing selected yet
    CHECK(nextSpectatable(one, 0, +1) == 0);    // cycling is a no-op
    CHECK(nextSpectatable(one, 0, -1) == 0);

    // Every agent unspectatable: -1 regardless of the starting index.
    std::vector<Agent> allReleased = {
        makeAgent(Agent::Mode::Driver, /*rel=*/true),
        makeAgent(Agent::Mode::Driver, /*rel=*/true),
    };
    CHECK(nextSpectatable(allReleased, -1, +1) == -1);
    CHECK(nextSpectatable(allReleased, 0, +1) == -1);
    CHECK(nextSpectatable(allReleased, 1, -1) == -1);

    // Fresh selection (current = -1) begins at index 0 forward, last backward.
    std::vector<Agent> three = { makeAgent(), makeAgent(), makeAgent() };
    CHECK(nextSpectatable(three, -1, +1) == 0);
    CHECK(nextSpectatable(three, -1, -1) == 2);
    // An out-of-range current is treated like "nothing selected".
    CHECK(nextSpectatable(three, 99, +1) == 0);
}

TEST_CASE(clamp_spectate_index_handles_stale_selection) {
    // Empty city clamps to "no selection".
    CHECK(clampSpectateIndex(0, 0) == -1);
    CHECK(clampSpectateIndex(5, 0) == -1);
    // In-range indices pass through.
    CHECK(clampSpectateIndex(0, 3) == 0);
    CHECK(clampSpectateIndex(2, 3) == 2);
    // A selection that ran off the end (the count shrank) clamps to the last.
    CHECK(clampSpectateIndex(3, 3) == 2);
    CHECK(clampSpectateIndex(100, 3) == 2);
    // A negative selection stays "no selection".
    CHECK(clampSpectateIndex(-1, 3) == -1);
}

TEST_CASE(spectate_heading_degrees_matches_flycamera_convention) {
    // The chase camera sits BEHIND the agent looking along its heading, so the
    // returned FlyCamera yaw is the direction the agent MOVES (yaw = atan2(fx,
    // -fz) with forward = (heading.x, 0, heading.y)) — the exact convention
    // camera_system.cpp derives from a vehicle's forward vector.
    //
    // Heading +Z (.y = +1): FlyCamera yaw 0 faces -Z, so moving +Z is yaw 180
    // (camera looks +Z from behind).
    CHECK_APPROX(std::fabs(spectateHeadingDegrees(Vec2(0, 1))), 180.0, 1e-9);
    // Heading -Z (.y = -1): that IS the yaw-0 facing.
    CHECK_APPROX(spectateHeadingDegrees(Vec2(0, -1)), 0.0, 1e-9);
    // Heading +X (.x = +1) yaws +90 (atan2(1, 0)).
    CHECK_APPROX(spectateHeadingDegrees(Vec2(1, 0)), 90.0, 1e-9);
    // Heading -X (.x = -1) yaws -90.
    CHECK_APPROX(spectateHeadingDegrees(Vec2(-1, 0)), -90.0, 1e-9);
    // A diagonal toward +X and +Z: atan2(1, -1) = 135 degrees.
    CHECK_APPROX(spectateHeadingDegrees(Vec2(1, 1)), 135.0, 1e-9);

    // Round-trip: the yaw must place the camera's forward (FollowCameraController
    // convention) along the agent's heading direction. Check a couple of headings
    // against forward = (sin(yaw), _, -cos(yaw)) at pitch 0.
    for (Vec2 h : { Vec2(0, 1), Vec2(1, 0), Vec2(-1, 0), Vec2(0, -1) }) {
        Real yaw = degreesToRadians(spectateHeadingDegrees(h));
        CHECK_APPROX(std::sin(yaw), h.x, 1e-9);    // forward.x
        CHECK_APPROX(-std::cos(yaw), h.y, 1e-9);   // forward.z (= world z = heading.y)
    }
}
