#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_RULES_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_RULES_H

#include "road_network.h"   // RoadClass
#include <cstdint>
#include <vector>

namespace engine {

// --- Cross-section (lane list) model -----------------------------------------
// A road's paved+walked width is not one scalar: it is an ORDERED list of lanes,
// left to right across the road (osm2streets `lane_specs_ltr`). This is the single
// source of carriageway width, lane count, median, and where each stripe goes —
// and it unifies the surface streets with the freeway's LaneSchedule (alignment.h).
// A CrossSection is authored per road CLASS (the data-driven template, below) and,
// later, overridable per edge; the mesher strokes exactly these lanes.
enum class LaneType : uint8_t {
    Sidewalk,   // raised pedestrian way (not carriageway)
    Shoulder,   // paved shoulder / verge (freeway, ramp)
    Parking,    // on-street parking lane
    Bike,       // bike lane / cycle track
    Driving,    // general travel lane
    Turn,       // dedicated turn bay
    Median,     // divider between opposing directions (flush, raised, or barrier)
};
enum class LaneDir : uint8_t { Fwd, Back, Both, None };  // None: sidewalk/median/parking

struct LaneSpec {
    LaneType type = LaneType::Driving;
    LaneDir  dir  = LaneDir::Fwd;
    double   width = 3.5;
};

// Lanes ordered LEFT to RIGHT across the road (facing along the edge a->b).
using CrossSection = std::vector<LaneSpec>;

// Paved carriageway width (everything except sidewalks) — what the graph strokes.
double carriagewayWidth(const CrossSection& xs);
// Full right-of-way width, including sidewalks.
double totalWidth(const CrossSection& xs);
// Count of Driving lanes (both directions) — the single `lanesAt()`-style query
// so painted dividers, junction dashes, and nav lanes never disagree.
int    drivingLaneCount(const CrossSection& xs);

// THE single lane-count query keyed on road CLASS (via its default cross-section).
// Markings, junction dashes, and nav all call this instead of re-deriving lanes
// from width with three different constants (road_mesh.cpp:969/1563 laneWidth 3.5,
// nav_graph.cpp:209 params) — so lane k is the same lane everywhere. Callers that
// need PER-DIRECTION lanes on a two-way road pass `perDirection` (half, min 1).
int    lanesForClass(RoadClass c, bool perDirection = false);

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
    double sidewalk;        // per-side sidewalk width (m); 0 = none (freeway / ramp)
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

    // The class's default cross-section (the data-driven lane-list template):
    // a local is [SW, drive, drive, SW]; an arterial adds parking + a median; a
    // freeway is shoulders + N lanes + barrier median; a ramp is one lane with
    // asymmetric shoulders. Built from the ClassRules row so widths/lane-count
    // stay consistent with minRadius/maxGrade. (Per-edge overrides come later.)
    CrossSection crossSectionFor(RoadClass c) const;
};

// The shipped ruleset (sensible real-world-ish defaults). One global table the builders read.
const DesignRules& defaultDesign();

}  // namespace engine

#endif
