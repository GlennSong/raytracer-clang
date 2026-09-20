// ONE ROAD MODULE, SEVERAL BUILDERS (Glenn, 2026-09-20: "I'd like to take what's
// in lanelab and make it a new road building module and allow us to switch
// between using the old one and the new one ... call one or the other up via lua
// or some common C++ interface").
//
// The interface is here; the lattice sits behind it and must still produce
// exactly what the loader used to build by calling the mesher itself, because
// every shipped level is built that way. Selection is DATA — the road block's
// "builder" — so a level, a recipe or a script can choose without a rebuild.
#include "test_framework.h"

#include "../src/engine/procgen/city/roads/road_builder.h"
#include "../src/engine/procgen/deprecated/roads/road_net_mesh.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>
#include <vector>

using namespace engine;

namespace {

// A small city block of streets on a slope, so the carve has something to do.
RoadEntity blockOfStreets() {
    RoadEntity road;
    RoadGraph& g = road.graph;
    for (int j = 0; j < 3; ++j)
        for (int i = 0; i < 3; ++i) g.nodes.push_back(RoadNode{Vec2(i * 60.0, j * 60.0)});
    auto at = [](int i, int j) { return j * 3 + i; };
    for (int j = 0; j < 3; ++j)
        for (int i = 0; i + 1 < 3; ++i) {
            g.addEdge(at(i, j), at(i + 1, j), 11.0);
            g.addEdge(at(j, i), at(j, i + 1), 8.0);
        }
    road.look.sidewalk = 3.0;
    return road;
}

double slope(double x, double z) { return 0.03 * x + 0.015 * z; }

}  // namespace

TEST_CASE(road_builders_are_chosen_by_data_not_by_a_build_flag) {
    const std::vector<std::string> names = roads::roadBuilderNames();
    CHECK(std::find(names.begin(), names.end(), "lattice") != names.end());

    // No "builder" key: the lattice, which is what every shipped level means.
    CHECK(roads::roadBuilderName(nlohmann::json::object()) == "lattice");
    CHECK(roads::roadBuilderName(nlohmann::json{{"builder", "lanes"}}) == "lanes");
    CHECK(roads::roadBuilder("lattice") != nullptr);
    CHECK(std::string(roads::roadBuilder("lattice")->name()) == "lattice");

    // A name this build does not have is a miss, not a crash...
    CHECK(roads::roadBuilder("no-such-builder") == nullptr);
    // ...and a level that asks for one still gets a road.
    CHECK(std::string(roads::roadBuilderFor(nlohmann::json{{"builder", "no-such-builder"}}).name()) ==
          "lattice");
}

TEST_CASE(the_lattice_builder_builds_exactly_what_the_loader_built_before) {
    const RoadEntity road = blockOfStreets();
    const RoadGroundFn ground = slope;

    roads::RoadBuildInput in;
    in.road = &road;
    in.ground = ground;
    const roads::RoadBuilder& b = roads::roadBuilderFor(nlohmann::json::object());

    // The surface, the deck and the kerb band: the builder's three, against the
    // three calls the loader used to make itself.
    CurbBandAudit bands;
    RoadDeckField deck;
    const RenderMesh direct = buildRoadNetMesh(road, ground, &bands, &deck);
    const roads::RoadProducts p = b.build(in);
    CHECK(p.mesh.vertices.size() == direct.vertices.size());
    CHECK(p.mesh.indices.size() == direct.indices.size());
    CHECK(!p.mesh.vertices.empty());
    CHECK(p.bands.loops.size() == bands.loops.size());
    CHECK(!p.bands.loops.empty());
    CHECK(p.deck.spines.size() == deck.spines.size());
    CHECK(!p.deck.empty());
    for (std::size_t i = 0; i < p.mesh.vertices.size(); i += 97)
        CHECK((p.mesh.vertices[i].position - direct.vertices[i].position).length() < 1e-9);

    // The carve the terrain pre-pass asks for. The lattice edits the level's
    // ground; it never hands one back.
    const roads::GroundPlan gp = b.ground(in);
    CHECK(gp.carve.size() == roadNetConformRegions(road, ground).size());
    CHECK(!gp.carve.empty());
    CHECK(!gp.replaces());
    CHECK(gp.replace == nullptr);

    // The centrelines everything routes on.
    const RoadGraph nav = b.navGraph(in);
    const RoadGraph navDirect = navRoadGraph(road, ground);
    CHECK(nav.nodes.size() == navDirect.nodes.size());
    CHECK(nav.edges.size() == navDirect.edges.size());
    CHECK(nav.edges.size() >= road.graph.edges.size());
}
