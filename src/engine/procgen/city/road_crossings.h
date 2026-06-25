#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_CROSSINGS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_CROSSINGS_H

#include "polygon.h"   // Vec2
#include <vector>

namespace engine {

// The crossing RESOLVER (unified-road-plan task 5): the connective tissue that turns a loose pile
// of road centerlines — whatever the generators (grid / radial / tensor) and the editable road
// emit — into ONE graph where roads that meet actually SHARE a node, instead of one mesh laid over
// another. Roads on the same grade get an at-grade node and weld (the weld engine unions their
// ribbons); roads on different grades get a grade-separated node (the upper bridges, no weld) that
// a later pass can splice with ramps. Pure 2-D + headless: it only rewrites centerlines + nodes.

// One road centerline as the generators hand it over (before resolving).
struct GraphSpine {
    std::vector<Vec2> points;
    double halfWidth = 4.0;
    int    layer = 0;          // grade level (0 = ground); crossings between layers are stacked
    int    klass = 0;          // RoadClass tag, carried through untouched
};

// A resolved junction: where incident roads meet. `gradeSeparated` is true when the incident
// spines span more than one layer (a fly-over), false for a plain at-grade intersection.
struct CrossNode {
    Vec2 pos;
    std::vector<int> spines;   // indices into ResolvedGraph::spines that touch this node
    bool gradeSeparated = false;
};

struct ResolvedGraph {
    std::vector<GraphSpine> spines;   // the input spines with crossing vertices spliced in
    std::vector<CrossNode>  nodes;
};

// Resolve every crossing among `in`: find where centerlines cross (proper interior crossings and
// T-touches where one road's endpoint lands on another), splice the exact crossing point into both
// spines so same-grade roads share a vertex (the weld then fuses them with no notch), and report a
// node per crossing tagged at-grade vs grade-separated. `weldTol` merges crossings that fall on
// the same spot into one node. Spine widths/layers/classes pass through unchanged.
ResolvedGraph resolveCrossings(const std::vector<GraphSpine>& in, double weldTol = 0.5);

}  // namespace engine

#endif
