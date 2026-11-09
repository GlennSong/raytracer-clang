#ifndef RAYTRACER_ENGINE_LIGHTS_PROBE_SITES_H
#define RAYTRACER_ENGINE_LIGHTS_PROBE_SITES_H

// WHERE THE REFLECTION PROBES STAND (ADR-0374; renderer-realism phase 3b, stage B). A probe sees what a street sees:
// so they stand along the streets, one every `spacing` metres of centreline at a little over a bus's height, each
// with a box (the parallax volume, ADR-0373) that reaches past the kerbs to the frontages either side. Those sites
// are the level's; the renderer holds 64 at a time, the nearest, and a probe keeps its slot while it stays near
// enough, so walking down a street re-captures only the probes it passes. Pure: the tests hold both.

#include "../../rt_math.h"
#include "../procgen/city/polygon.h"   // Vec2
#include <functional>
#include <vector>

namespace engine {

struct RoadGraph;

struct ProbeSite {
    Vec3 position;
    Vec3 boxMin, boxMax;
    bool indoor = false;   // a storey's (ADR-0380): only indoor surfaces read it, and they read only these
};

// The rules (lights.lua `probes`; these are the defaults a missing field takes).
struct ProbeRules {
    double spacing = 40.0;   // m of centreline per probe
    double height = 5.0;     // m above the carriageway
    double side = 30.0;      // m the box reaches past the road's half width, either side
    double below = 1.0;      // m the box reaches under the carriageway (its floor is the street: what the probe saw) ...
    double above = 45.0;     // ... and over it
    double merge = 20.0;     // a site this near another (and within 6 m in height) is the same site: junctions
    double hysteresis = 15.0;   // m past the nearest set's edge a resident probe may drift before it gives its slot up
    // INDOORS (ADR-0380): the storey the camera stands on, one probe every `indoorSpacing` m across its footprint at
    // `indoorHeight` over its floor, each box its cell from floor to ceiling
    double indoorSpacing = 8.0, indoorHeight = 1.6;
};

// Ground height at (x, z), for nodes whose elevation is above the ground
using ProbeGroundFn = std::function<double(double, double)>;

// Sites along every edge of `g` (the level's sampled graph, LevelRoadGraph, so curves are followed): an edge of length L
// takes max(1, round(L / spacing)) evenly spaced, merged at junctions.
void placeProbeSites(const RoadGraph& g, const ProbeGroundFn& ground, const ProbeRules& rules, std::vector<ProbeSite>& out);

// A storey's sites (ADR-0380): its footprint as an oriented box (centre, the two axes, the half extents), its floor and
// ceiling; a grid of cells `spacing` across (at least one), a probe in each at `height` over the floor, its box the
// cell's (as an axis-aligned box round the turned square, a little over) from the floor to the ceiling. Marked indoor.
void placeStoreyProbeSites(const Vec2& centre, const Vec2 axis[2], const Real half[2], double floorY, double ceilY,
                           double spacing, double height, std::vector<ProbeSite>& out);

// Which sites hold the renderer's `resident.size()` slots (slot -> site index, -1 empty). The nearest that many are
// wanted; a resident site leaves only once it is `hysteresis` past the farthest wanted one, and the freed slots go
// to the nearest wanted sites not yet in. Returns true when a slot changed.
bool pickResidentProbes(const std::vector<ProbeSite>& sites, const Vec3& eye, double hysteresis, std::vector<int>& resident);

}  // namespace engine

#endif
