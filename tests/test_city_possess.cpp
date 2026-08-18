// The PURE half of possession (ADR-0079): the control channel's possess.cmd
// grammar, the status line, and the speed policy. The impure system
// (city_possess.cpp) is a thin shell around these — and is exercised live over
// the channel plus by the Jolt drive-integration case in the physics suite.
#include "test_framework.h"

#include "../src/apps/citysim/city_possess_logic.h"

#include <cmath>

using namespace citysim;

TEST_CASE(possess_cmd_parses_car_with_and_without_position) {
    PossessCmd bare = parsePossessCmd("car");
    CHECK(bare.kind == PossessCmd::Kind::Car);
    CHECK(!bare.hasPos);

    PossessCmd at = parsePossessCmd("car -280.5 150");
    CHECK(at.kind == PossessCmd::Kind::Car);
    CHECK(at.hasPos);
    CHECK(std::fabs(at.x + 280.5) < 1e-9);
    CHECK(std::fabs(at.z - 150.0) < 1e-9);
}

TEST_CASE(possess_cmd_parses_destinations_and_rejects_partial_ones) {
    PossessCmd go = parsePossessCmd("driveto 300 -420");
    CHECK(go.kind == PossessCmd::Kind::DriveTo);
    CHECK(go.hasPos);
    CHECK(std::fabs(go.x - 300.0) < 1e-9);
    CHECK(std::fabs(go.z + 420.0) < 1e-9);

    CHECK(parsePossessCmd("walkto 10 20").kind == PossessCmd::Kind::WalkTo);

    // A destination without both coordinates is INVALID, not a silent no-op —
    // the status line carries the reason back to the sender.
    PossessCmd half = parsePossessCmd("driveto 300");
    CHECK(half.kind == PossessCmd::Kind::Invalid);
    CHECK(!half.error.empty());
}

TEST_CASE(possess_cmd_parses_stop_release_none_and_unknown) {
    CHECK(parsePossessCmd("stop").kind == PossessCmd::Kind::Stop);
    CHECK(parsePossessCmd("release").kind == PossessCmd::Kind::Release);
    CHECK(parsePossessCmd("").kind == PossessCmd::Kind::None);
    PossessCmd bad = parsePossessCmd("teleport 1 2");
    CHECK(bad.kind == PossessCmd::Kind::Invalid);
    CHECK(!bad.error.empty());
}

TEST_CASE(possess_status_line_carries_kind_state_pose_and_remaining) {
    const std::string s = formatPossessStatus(
        "car", PossessState::Driving, Vec3(-281.2, 6.0, 149.8), 8.4, 412.0);
    CHECK(s.find("car") == 0);
    CHECK(s.find("state=driving") != std::string::npos);
    CHECK(s.find("pos=-281.2,6.0,149.8") != std::string::npos);
    CHECK(s.find("speed=8.4") != std::string::npos);
    CHECK(s.find("remaining=412") != std::string::npos);
}

TEST_CASE(possess_state_names_are_distinct) {
    // The reading agent branches on these; two states sharing a name would
    // make stuck look like driving.
    const PossessState all[] = {
        PossessState::None,    PossessState::Idle,   PossessState::Driving,
        PossessState::Walking, PossessState::Arrived, PossessState::Stuck,
        PossessState::NoRoute,
    };
    for (std::size_t a = 0; a < 7; ++a)
        for (std::size_t b = a + 1; b < 7; ++b)
            CHECK(std::string(possessStateName(all[a])) !=
                  possessStateName(all[b]));
}

TEST_CASE(possess_cruise_speed_caps_fast_roads_keeps_slow_ones) {
    CHECK(std::fabs(possessCruiseSpeed(28.0) - kPossessCruiseCap) < 1e-9);
    CHECK(std::fabs(possessCruiseSpeed(8.0) - 8.0) < 1e-9);
}

TEST_CASE(possess_speed_at_station_selects_the_current_segment) {
    // Polyline arcs 0..30..60; first half Local (8), second half Arterial (16,
    // capped stays 14? no — 16 > 14 caps to 14).
    const std::vector<Real> arcs = {0, 30, 60};
    const std::vector<Real> speeds = {8.0, 16.0, 16.0};
    CHECK(std::fabs(speedAtStation(arcs, speeds, 5.0) - 8.0) < 1e-9);
    CHECK(std::fabs(speedAtStation(arcs, speeds, 45.0) - 14.0) < 1e-9);
    // Past the end: the last entry answers (the follower is at the tail).
    CHECK(std::fabs(speedAtStation(arcs, speeds, 500.0) - 14.0) < 1e-9);
    // Empty polyline: zero, never a read off the end.
    CHECK(speedAtStation({}, {}, 10.0) == 0.0);
}
