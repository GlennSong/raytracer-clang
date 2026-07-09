#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_METRO_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_METRO_H

#include "polygon.h"
#include "road_network.h"   // RoadGraph
#include "buildability.h"   // HeightSampler, BuildabilityConfig (terrain-aware layout)
#include <cstdint>

namespace engine {

// A whole-metro road generator (the "kind":"metro" recipe). Unlike buildDistrict
// — which subdivides ONE footprint — this grows a network of ORGANIC ARTERIALS
// between a handful of hotspots by multi-source space colonization, stitches the
// separately-grown trees into one graph, and closes a few loops so the arterials
// actually enclose blocks (a tree encloses none). Each enclosed block (a face of
// the arterial graph, via extractBlocks) is then filled by the SAME OBB
// subdivision buildDistrict uses — so local streets are cuts of the arterial-
// bounded block and come off the arterials naturally — or by concentric
// rings+spokes for a radial hub. The result is one connected, planarized graph
// that flows through the normal planarize -> weld -> conform pipeline.
struct MetroParams {
    Vec2   center{0, 0};
    double radius      = 700.0;   // footprint half-extent (m)
    int    hotspots    = 6;       // number of hubs (>=2); one central, rest ringed
    double blockSize   = 70.0;    // target block edge for the street subdivision (m)
    double arteryWidth = 13.0;    // width tagged on arterial edges
    double streetWidth = 7.0;     // width tagged on local streets
    bool   ringRoad    = false;   // add a deliberate ring road around the core
    std::uint32_t seed = 5u;

    // Terrain-aware layout (optional). When `ground` is set, hotspots, arterial
    // growth and blocks are gated on the buildability of the ground: the city
    // hugs buildable land and avoids water / steep mountain, instead of marching
    // over them. Unset `ground` = the old terrain-blind 2D layout.
    HeightSampler      ground;    // terrain height sampler (null = no gating)
    BuildabilityConfig build;     // slope/water thresholds for the gate
};

// Grow the metro and return its planarized RoadGraph (arterials tagged
// RoadClass::Arterial, local streets RoadClass::Local).
RoadGraph buildMetro(const MetroParams& p);

}  // namespace engine

#endif
