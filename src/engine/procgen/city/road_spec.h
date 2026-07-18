#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_SPEC_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_SPEC_H

// Roads v2 data model (plan: nested-bouncing-turing.md Part 1). A road stops
// being "class + width" and becomes a SPEC: an ordered list of BANDS across the
// road, left to right. Venice Blvd, a one-way alley, a dirt road, and a 3-lane
// freeway are all the same structure with different bands. Direction lives PER
// LANE (the only correct home — RoadEdge::oneWay is deprecated by this); the
// mesher's cross-section, the markings, and the sim's lane frames are all
// generated from this one source, so they cannot disagree.

#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

enum class BandKind : uint8_t {
    Travel,     // a drivable lane
    Turn,       // a turn lane (drivable; decal arrows near junctions)
    Parking,    // parallel parking strip (drivable at crawl; cars may occupy)
    Shoulder,   // paved/soft shoulder (drivable in emergencies; no markings)
    Median,     // painted or raised centre separation (not drivable)
    Bike,       // bike lane (not car-drivable)
    Curb,       // the kerb: a raised lip; carries paint (red/green/none)
    Sidewalk,   // raised pedestrian band
    Barrier,    // parapet / guard wall (freeways, bridges)
};

enum class BandSurface : uint8_t { Asphalt, Concrete, Dirt, Gravel };

// Curb paint / marking style ids (kept small and explicit).
enum : uint8_t { kPaintNone = 0, kPaintRed = 1, kPaintGreen = 2, kPaintYellow = 3,
                 kPaintWhite = 4, kPaintBlue = 5 };

struct RoadBand {
    BandKind    kind = BandKind::Travel;
    float       width = 3.6f;          // metres
    int8_t      dir = 0;               // +1 forward (a->b), -1 backward, 0 n/a
    BandSurface surface = BandSurface::Asphalt;
    uint8_t     paint = kPaintNone;    // curb colour / marking style
};

struct RoadSpec {
    std::string name;                  // preset name it came from ("" = custom)
    std::vector<RoadBand> bands;       // ordered left -> right
    float designSpeed = 0;             // >0: the engineering pass runs (fwy/ramp)

    // --- derived helpers (the mesher/sim read these, never raw width fields) ---
    double totalWidth() const;               // sum of all band widths
    double carriagewayWidth() const;         // sum of drivable bands (Travel/Turn/Parking/Shoulder)
    double carriagewayHalfWidth() const { return carriagewayWidth() * 0.5; }
    int    laneCount() const;                // Travel + Turn bands
    bool   isOneWay() const;                 // all directed lanes same dir
    bool   hasSidewalk(int side) const;      // side: -1 = left end, +1 = right end
    // Lateral span [x0,x1) of band i, measured from the LEFT edge of the spec
    // (so centreline = totalWidth()/2). The lane-frame queries build on this.
    void   bandSpan(int i, double& x0, double& x1) const;
};

// Named presets — defaults, not the truth; levels may author full band lists.
// "local" 2-lane street + sidewalks | "arterial" 4-lane + sidewalks |
// "freeway3" 3 lanes/side + median + barriers, no sidewalks |
// "ramp1" one directed lane + shoulders + barriers | "dirt" country road |
// "alley" one one-way lane, no sidewalk.
RoadSpec roadSpecPreset(const std::string& name);

// COMPAT SHIM (transition only; removed with the legacy fields at cleanup):
// synthesize a spec from the legacy edge description so old graphs/JSON load
// unchanged. `width` = legacy carriageway width; sidewalk from the net's look.
RoadSpec roadSpecFromLegacy(double width, bool oneWay, double sidewalkWidth,
                            double curbHeight);

// JSON: either a preset name ("freeway3") or an object with a "bands" array
// (each {kind, width, dir?, surface?, paint?}). Round-trips.
RoadSpec roadSpecFromJson(const nlohmann::json& j);
nlohmann::json roadSpecToJson(const RoadSpec& spec);

}  // namespace engine

#endif
