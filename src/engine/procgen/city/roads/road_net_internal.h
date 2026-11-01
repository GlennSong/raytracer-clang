#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_NET_INTERNAL_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_NET_INTERNAL_H

// The road entity's own internals. Not a public interface: the sampled/constrained
// graph is published as navRoadGraph.

#include "road_entity.h"

namespace engine::roadnet {

// The curvature cap: half-width + sidewalk + margin of the WIDEST edge.
double netMinTurnRadius(const RoadEntity& road);
// The entity's control graph sampled to polylines (Hermite through tangents).
RoadGraph sampleNetGraph(const RoadEntity& road, double minTurnRadius = 0.0);
// Dead ends that stop inside another road become T junctions (logged once).
void joinDanglingEndsLogged(RoadGraph& g, const RoadEntity& road);
// Sampled, dangling ends joined, classified: what navRoadGraph returns.
RoadGraph constrainedGraph(const RoadEntity& road, const RoadGroundFn& heightAt);

}  // namespace engine::roadnet

#endif
