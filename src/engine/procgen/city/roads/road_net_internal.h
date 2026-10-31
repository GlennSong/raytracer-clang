#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_NET_INTERNAL_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_NET_INTERNAL_H

// The road entity's own internals. Not a public interface: the sampled/constrained
// graph is published as navRoadGraph.

#include "road_entity.h"
#include "../road_rules.h"   // DesignRules (the per-class grade table)

namespace engine::roadnet {

// Profile slope limit (rise/run) for chain-height smoothing — the value the
// deleted weld used (WeldSolidParams::maxGrade); mesh and terrain carve both
// grade to it, so they stay in agreement.
inline constexpr double kRoadMaxGrade = 0.08;   // the single-grade FALLBACK
// The per-class grade table (freeway 5%, arterial 8%, collector 10%, local
// 12%, ramp 6%) the profile solve reads when RoadLook::perClassGrade is set.
inline const DesignRules kDesignRules{};

// The curvature cap: half-width + sidewalk + margin of the WIDEST edge.
double netMinTurnRadius(const RoadEntity& road);
// The entity's control graph sampled to polylines (Hermite through tangents).
RoadGraph sampleNetGraph(const RoadEntity& road, double minTurnRadius = 0.0);
// Dead ends that stop inside another road become T junctions (logged once).
void joinDanglingEndsLogged(RoadGraph& g, const RoadEntity& road);
// Sampled + constraints + semantic classification: what navRoadGraph returns.
RoadGraph constrainedGraph(const RoadEntity& road, const RoadGroundFn& heightAt);

}  // namespace engine::roadnet

#endif
