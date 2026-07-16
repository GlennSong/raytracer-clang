#include "test_framework.h"

#include "../src/engine/procgen/city/road_rules.h"
#include "../src/engine/procgen/city/road_net.h"
#include <nlohmann/json.hpp>
#include <cmath>

using namespace engine;

namespace {

// Count lanes of a given type in a cross-section.
int countType(const CrossSection& xs, LaneType t) {
    int n = 0;
    for (const LaneSpec& l : xs) if (l.type == t) ++n;
    return n;
}

}  // namespace

// A local street: sidewalk, one lane each way, sidewalk. No median, no shoulder.
TEST_CASE(cross_section_local) {
    const CrossSection xs = defaultDesign().crossSectionFor(RoadClass::Local);
    CHECK(drivingLaneCount(xs) == 2);
    CHECK(countType(xs, LaneType::Sidewalk) == 2);        // both sides walked
    CHECK(countType(xs, LaneType::Median) == 0);          // undivided
    CHECK(countType(xs, LaneType::Shoulder) == 0);        // surface street
    CHECK(xs.front().type == LaneType::Sidewalk);         // ordered left..right
    CHECK(xs.back().type == LaneType::Sidewalk);
    // carriageway = 2 lanes; total adds the two sidewalks.
    CHECK_APPROX(carriagewayWidth(xs), 2 * 3.2, 1e-9);
    CHECK(totalWidth(xs) > carriagewayWidth(xs));         // sidewalks add width
}

// An arterial: 4 lanes, 2 each way, wide sidewalks. Undivided in the current
// ruleset (dividedMedian=false), so no median lane — the cross-section must
// reflect the rule faithfully, not impose one. (A divided arterial variant is a
// rules/data change under the data-driven kit, not baked here.)
TEST_CASE(cross_section_arterial) {
    const ClassRules r = defaultDesign().forClass(RoadClass::Arterial);
    const CrossSection xs = defaultDesign().crossSectionFor(RoadClass::Arterial);
    CHECK(drivingLaneCount(xs) == 4);
    CHECK(countType(xs, LaneType::Median) == (r.dividedMedian ? 1 : 0));
    CHECK(countType(xs, LaneType::Sidewalk) == 2);        // wide walks both sides
    int fwd = 0, back = 0;
    for (const LaneSpec& l : xs) {
        if (l.type == LaneType::Driving && l.dir == LaneDir::Fwd) ++fwd;
        if (l.type == LaneType::Driving && l.dir == LaneDir::Back) ++back;
    }
    CHECK(fwd == 2);
    CHECK(back == 2);
}

// A freeway: paved shoulders (not sidewalks), a barrier median, six lanes.
TEST_CASE(cross_section_freeway) {
    const CrossSection xs = defaultDesign().crossSectionFor(RoadClass::Freeway);
    CHECK(drivingLaneCount(xs) == 6);
    CHECK(countType(xs, LaneType::Sidewalk) == 0);        // no pedestrians on a freeway
    CHECK(countType(xs, LaneType::Shoulder) == 2);        // both outer shoulders
    CHECK(countType(xs, LaneType::Median) == 1);
}

// A ramp: one-way single lane with asymmetric shoulders, no median, no sidewalk.
TEST_CASE(cross_section_ramp) {
    const CrossSection xs = defaultDesign().crossSectionFor(RoadClass::Ramp);
    CHECK(drivingLaneCount(xs) == 1);
    CHECK(countType(xs, LaneType::Median) == 0);
    CHECK(countType(xs, LaneType::Sidewalk) == 0);
    // exactly one travel lane, and it flows forward.
    int fwd = 0;
    for (const LaneSpec& l : xs) if (l.type == LaneType::Driving && l.dir == LaneDir::Fwd) ++fwd;
    CHECK(fwd == 1);
    // outer shoulder wider than inner (AASHTO asymmetric section).
    CHECK(xs.front().type == LaneType::Shoulder);
    CHECK(xs.back().type == LaneType::Shoulder);
    CHECK(xs.back().width > xs.front().width);
}

// P1 unification: a GENERATED district must carry real road classes end-to-end
// (generator -> planarize/capDegree/cleanup -> netGraph), not collapse every
// surface edge to Local. Regression for the netGraph RoadClass::Local hardcode.
TEST_CASE(generated_net_carries_classes) {
    RoadNet net;
    nlohmann::json recipe = {
        {"kind", "district"}, {"radius", 160.0}, {"arterials", 3},
        {"block_size", 40.0}, {"seed", 3},
    };
    applyGenerateRecipe(net, recipe);
    CHECK(!net.edges.empty());
    // the district generator distinguishes arterials from local streets.
    CHECK(net.edgeClasses.size() == net.edges.size());   // parallel array kept in sync
    int arterials = 0, locals = 0;
    for (RoadClass c : net.edgeClasses) {
        if (c == RoadClass::Arterial) ++arterials;
        if (c == RoadClass::Local) ++locals;
    }
    CHECK(arterials > 0);                                 // arterials survived the pipeline
    CHECK(locals > 0);                                    // and so did local streets

    // And the class reaches the meshing graph (netGraph no longer hardcodes Local).
    const RoadGraph g = navRoadGraph(net);
    int nonLocal = 0;
    for (const RoadEdge& e : g.edges) if (e.klass != RoadClass::Local) ++nonLocal;
    CHECK(nonLocal > 0);
}
