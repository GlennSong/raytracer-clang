#ifndef RAYTRACER_ENGINE_PROCGEN_LANELAB_ROAD_TWIN_H
#define RAYTRACER_ENGINE_PROCGEN_LANELAB_ROAD_TWIN_H

// The lab network as the engine's own RoadEntity (ADR-0083 integration seam), so everything
// downstream that walks RoadEntity — the lot pass, the city map, street furniture — sees
// lanelab's streets without a second pipeline. Nodes at each spine's corners (Douglas-Peucker 0.25 m,
// runs capped at `nodeSpacing`), edges with the class and paved width, deck heights as absolute node
// elevations on freeway and ramp edges, the carriageway centrelines as plan.freewayPlans
// (the lot pass's right-of-way band). The STREET subgraph is planarised: every crossing and
// every T within 0.5 m becomes a shared node, which is what extractBlocks needs to close a
// block face. Freeways and ramps are grade-separated from streets: neither split nor splitting.

#include "engine/procgen/city/roads/road_entity.h"
#include "engine/procgen/lanelab/lanelab.h"

namespace engine {
namespace lanelab {

// `forLots`: the twin the BLOCK/LOT pass should read. The pass excludes freeway and ramp class from
// block faces (right: no frontage on a freeway), so a face with a ramp or a carriageway inside it is
// parcelled straight across the pavement (metro: 66 of 100 block feet on built pavement, every one over
// 300 m² across a ramp). In this mode every deck run is emitted as a street-class edge — a face
// boundary — and planarised with the streets; its earthwork width still keeps buildings off. The
// class-faithful twin (default) is what the unified road graph, the nav and the map read.
RoadEntity roadTwin(const Result& r, double nodeSpacing = 60.0, bool forLots = false);

}  // namespace lanelab
}  // namespace engine

#endif
