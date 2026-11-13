#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_MODEL_COMPARE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_MODEL_COMPARE_H

// THE ROAD MODEL AGAINST THE OLD FORMS (docs/road-model-design.md step 2's test, ADR-0417): for one built road network,
// where the one model (road_model.h) and each of today's forms agree about the same junctions, streets and lanes, and
// where the old forms disagree with EACH OTHER -- the evidence for moving readers to the model, one at a time.
//
// The old forms compared:
//   - the NAV GRAPH the city sim routes on: the road twin (road_twin.h, the travel-lane widths), its sampled road graph
//     (navRoadGraph) and the directed graph built from it (buildNavGraph, with its own merging of junction "knots": nodes
//     a few metres apart made one);
//   - the PAINT's junction boxes (deck_mesh.h junctionSpans, kept in the build's Result);
//   - the LANE GRAPH the new driving drives on (lane_graph_build.h buildLaneGraph), with its junctions and lane paths;
//   - the LANE MATCH (apps/citysim/lane_map.h): which nav links the builder's lanes were matched to.
//
// How a junction of one form is "the same" as one of another, the one measured rule here (this is a comparison, so it
// measures; nothing it finds goes into the model): within kCompareReach of the model junction's centre or of any of its
// arms' points. A form's junction near two model junctions holds them as one ("knots them"); a model junction near two of
// a form's holds them as one the other way. Streets are compared where their middle is: the nav link running along it
// there (within kCompareLinkReach, heading within about 25 degrees), its lanes each way, one-way, class, and the length
// of the nav's chain of links from junction node to junction node.
//
// This file is compiled into lanes_tool and lanes_tests only: it reads the city sim's lane match.

#include "engine/procgen/city/roads/road_model.h"

#include <string>

namespace engine {
namespace roads::lanes {

struct Result;

constexpr double kCompareReach = 12.0;      // metres: a junction of another form is this one's (kJunctionMergeGap)
constexpr double kCompareLinkReach = 3.0;   // metres: a nav link runs along a street's middle

// A junction compared both ways.
struct JunctionAgreement {
    uint32_t model = 0, other = 0;          // how many each side holds
    uint32_t matched = 0;                   // model junctions with exactly one of the other's, holding only them
    uint32_t onlyModel = 0, onlyOther = 0;  // with none on the other side
    uint32_t otherKnots = 0;                // of the other's: near two model junctions or more (it holds them as one)
    uint32_t modelKnots = 0;                // of the model's: near two of the other's or more (the other splits it)
};

struct RoadModelComparison {
    JunctionAgreement nav, paint, laneGraph;
    uint32_t navRampNodes = 0;              // the nav's junction nodes that are freeway or ramp gores
    uint32_t modelTwoArm = 0;               // model junctions of two arms (a street carrying on: no junction to the others)
    // streets against nav links
    uint32_t streetsCompared = 0, streetsWithoutLink = 0, laneCountDiffers = 0, oneWayDiffers = 0, classDiffers = 0, lengthDiffers = 0;
    // lanes against the lane graph's lane paths
    uint32_t modelRoadLanes = 0, lanesWithPaths = 0, lanesWithoutPaths = 0, sectionsOutOfOrder = 0;
    // the lane match
    int linksDrivable = 0, linksMatched = 0, linksCountDiffers = 0, linksPartial = 0, linksNone = 0;
    uint32_t streetsWithUnmatchedLink = 0;
    // the old forms against each other, at the model's junctions and streets
    uint32_t junctionsFormsDisagree = 0, streetsFormsDisagree = 0;
    std::string text;                       // the whole report, in words
};

// Compares `model` (assembled from `r`) with the old forms built from `r`.
RoadModelComparison compareRoadModel(const RoadModel& model, const Result& r);

}  // namespace roads::lanes
}  // namespace engine

#endif
