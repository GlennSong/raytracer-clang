// ONE ROAD MODULE, SEVERAL BUILDERS (Glenn, 2026-09-20: "I'd like to take what's
// in lanelab and make it a new road building module and allow us to switch
// between using the old one and the new one ... call one or the other up via lua
// or some common C++ interface").
//
// The interface is here, and since the lattice retired (2026-10-14) one builder sits
// behind it: lanes. Selection stays DATA -- the road block's "builder" -- so a new
// builder can join without a rebuild of every level.
#include "test_framework.h"

#include "../src/engine/procgen/city/roads/road_builder.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>
#include <vector>

using namespace engine;

TEST_CASE(road_builders_are_chosen_by_data_not_by_a_build_flag) {
    const std::vector<std::string> names = roads::roadBuilderNames();
    // ONE BUILDER: the lattice retired (2026-10-14); lanes is the default and the only one.
    CHECK(names.size() == 1u);
    CHECK(std::find(names.begin(), names.end(), "lanes") != names.end());
    CHECK(roads::roadBuilderName(nlohmann::json::object()) == "lanes");
    CHECK(roads::roadBuilderName(nlohmann::json{{"builder", "lanes"}}) == "lanes");
    CHECK(roads::roadBuilder("lanes") != nullptr);
    CHECK(roads::roadBuilder("lattice") == nullptr);
    // A name this build does not have is a miss, not a crash...
    CHECK(roads::roadBuilder("no-such-builder") == nullptr);
    // ...and a level that asks for one still gets the builder there is.
    CHECK(std::string(roads::roadBuilderFor(nlohmann::json{{"builder", "no-such-builder"}}).name()) == "lanes");
}

