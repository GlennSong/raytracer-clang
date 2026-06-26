#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_DISTRICT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_DISTRICT_H

#include "polygon.h"
#include "road_network.h"   // RoadGraph
#include <vector>
#include <cstdint>

namespace engine {

// A dense city district grown by SUBDIVISION (the structured-infill half of the plan). An irregular
// footprint is split by a few major ARTERIALS into sectors, and each sector is recursively bisected
// along its oriented bounding box until block-sized — the cut lines are local STREETS, the leftover
// pieces are BLOCKS. A grid emerges where a sector is rectangular; sectors at different angles read
// as different districts. This is where road DENSITY and structure come from (space colonization
// alone only makes a sparse tree).
struct DistrictParams {
    Vec2 center{0, 0};
    double radius = 130;       // city footprint radius (m)
    int    arterials = 3;      // major roads splitting the footprint into sectors
    double blockSize = 34;     // target block edge (m) — smaller = denser
    double irregular = 0.22;   // footprint irregularity (0 = circle)
    double jitter = 0.16;      // street-position randomization (fraction of extent)
    double arteryWidth = 6;    // carriageway width tagged on arterial edges
    double streetWidth = 3;    // carriageway width tagged on local edges
    std::uint32_t seed = 1;
};

struct DistrictNet {
    // One PLANARIZED graph: arterials and streets split at every crossing and share junction nodes,
    // so the network is connected (not a pile of independent segments). Edges carry width + class
    // (RoadClass::Arterial vs Local) so the caller can still style arterials and streets apart.
    RoadGraph          graph;
    std::vector<Poly2> blocks;   // final block polygons (for lots/buildings later)
};

DistrictNet buildDistrict(const DistrictParams& p);

}  // namespace engine
#endif
