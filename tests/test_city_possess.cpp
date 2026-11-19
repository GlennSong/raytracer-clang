// The PURE half of possession (ADR-0079): the control channel's possess.cmd
// grammar and the status line. The impure system (city_possess.cpp) is a thin
// shell around these, exercised live over the channel. (Its own car driver and
// the speed policy it drove by went with the old driving, ADR-0459.)
#include "test_framework.h"

#include "../src/apps/citysim/city_possess_logic.h"

#include <cmath>

using namespace citysim;

TEST_CASE(possess_cmd_parses_walker_with_and_without_position) {
    PossessCmd bare = parsePossessCmd("walker");
    CHECK(bare.kind == PossessCmd::Kind::Walker);
    CHECK(!bare.hasPos);

    PossessCmd at = parsePossessCmd("walker -280.5 150");
    CHECK(at.kind == PossessCmd::Kind::Walker);
    CHECK(at.hasPos);
    CHECK(std::fabs(at.x + 280.5) < 1e-9);
    CHECK(std::fabs(at.z - 150.0) < 1e-9);

    // possession's own car, its drive and the bus ride are gone: said so, not taken silently
    for (const char* gone : {"car", "driveto 300 -420", "ride 1 2"}) {
        const PossessCmd c = parsePossessCmd(gone);
        CHECK(c.kind == PossessCmd::Kind::Invalid);
        CHECK(!c.error.empty());
    }
}

TEST_CASE(possess_cmd_parses_destinations_and_rejects_partial_ones) {
    PossessCmd go = parsePossessCmd("walkto 300 -420");
    CHECK(go.kind == PossessCmd::Kind::WalkTo);
    CHECK(go.hasPos);
    CHECK(std::fabs(go.x - 300.0) < 1e-9);
    CHECK(std::fabs(go.z + 420.0) < 1e-9);

    // A destination without both coordinates is INVALID, not a silent no-op —
    // the status line carries the reason back to the sender.
    PossessCmd half = parsePossessCmd("walkto 300");
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

// `direct on|off` hands the agent's PLAN to the director and gives it back
// (ADR-0091). A bare or misspelled switch must not silently mean "on": that
// would hold the goal layer off an agent nobody meant to direct, and it would
// stand in the street for the rest of the run.
TEST_CASE(possess_cmd_parses_direct_and_requires_an_explicit_switch) {
    PossessCmd on = parsePossessCmd("direct on");
    CHECK(on.kind == PossessCmd::Kind::Direct);
    CHECK(on.on);

    PossessCmd off = parsePossessCmd("direct off");
    CHECK(off.kind == PossessCmd::Kind::Direct);
    CHECK(!off.on);

    for (const char* bad : {"direct", "direct yes", "direct 1", "direct ON"}) {
        PossessCmd c = parsePossessCmd(bad);
        CHECK(c.kind == PossessCmd::Kind::Invalid);
        CHECK(!c.error.empty());
    }
}

TEST_CASE(possess_status_line_carries_kind_state_pose_and_remaining) {
    const std::string s = formatPossessStatus(
        "walker", PossessState::Walking, Vec3(-281.2, 6.0, 149.8), 1.4, 412.0);
    CHECK(s.find("walker") == 0);
    CHECK(s.find("state=walking") != std::string::npos);
    CHECK(s.find("pos=-281.2,6.0,149.8") != std::string::npos);
    CHECK(s.find("speed=1.4") != std::string::npos);
    CHECK(s.find("remaining=412") != std::string::npos);
}

TEST_CASE(possess_state_names_are_distinct) {
    // The reading agent branches on these; two states sharing a name would
    // make no-route look like arrived.
    const PossessState all[] = {
        PossessState::None, PossessState::Idle, PossessState::Walking, PossessState::Arrived, PossessState::NoRoute,
    };
    const std::size_t n = sizeof(all) / sizeof(all[0]);
    for (std::size_t a = 0; a < n; ++a)
        for (std::size_t b = a + 1; b < n; ++b)
            CHECK(std::string(possessStateName(all[a])) != possessStateName(all[b]));
}
