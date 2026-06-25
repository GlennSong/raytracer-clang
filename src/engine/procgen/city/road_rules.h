#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_RULES_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_RULES_H

#include "road_network.h"   // RoadClass

namespace engine {

// The road DESIGN RULES, in one place (ADR-0052/0053). Before this, the numbers that govern a
// road network were scattered: min arm angle / max degree in road_constraints' RoadRules; the
// bridge clearance, deck thickness and ramp grade hardcoded inline in buildLayeredRoadNetMesh;
// min curve radius and max grade passed as loose function arguments. This is the single source
// so "how acutely a road may curve", "when a junction becomes a roundabout", and "how steep a
// ramp may climb onto an overpass" live together and stay consistent — and so a road's CLASS
// drives its geometry rather than a magic constant at each call site.
//
// Per-class cross-section + geometry limits.
struct ClassRules {
    double minRadius;       // tightest centre-line curve (m) — a design-speed proxy
    double maxGrade;        // steepest longitudinal slope (rise/run)
    double laneWidth;       // nominal lane width (m)
    int    lanes;           // default lane count (both directions)
    bool   dividedMedian;   // directions separated by a median (a freeway is; a street isn't)
    double fullWidth() const { return lanes * laneWidth; }
};

// Network-wide policy + the per-class table.
struct DesignRules {
    // --- junction policy (the constraint pass, ADR-0052) ---
    double minArmAngle = 0.52;    // two arms closer than this (rad ~30 deg) can't share a flat junction
    int    maxArmsAtGrade = 4;    // a node with more arms than this is promoted to a roundabout

    // --- grade separation (the layered build / bridges, ADR-0054) ---
    double clearance = 5.0;       // vertical gap a bridge leaves over the road beneath it (m)
    double deckThickness = 0.8;   // bridge-deck slab thickness (m)
    double rampGrade = 0.06;      // grade a bridge approach / ramp climbs at (rise/run)

    ClassRules forClass(RoadClass c) const;     // the per-class row
    double minRadius(RoadClass c) const { return forClass(c).minRadius; }
    double maxGrade(RoadClass c) const { return forClass(c).maxGrade; }
};

// The shipped ruleset (sensible real-world-ish defaults). One global table the builders read.
const DesignRules& defaultDesign();

}  // namespace engine

#endif
