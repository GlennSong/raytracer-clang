#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_PLAN_LAND_SHAPE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_PLAN_LAND_SHAPE_H

// A CITY'S SHAPE FROM ITS LAND (ADR-0106). Glenn, 2026-09-25: "For a city shape is there any way we
// could find the contours of that region or build a shape that better fits the contours and build a
// city within that shape instead of floodfilling a circle each time?"
//
// The city GROWS from its heart over buildable ground, cheapest ground first: every step costs its
// length, more the steeper it is and the higher it climbs above the heart. It stops at its target
// area. What it covers is bounded by a line of equal cost -- up a flat valley it reaches far, against
// the foothills it stops short, along a coastal plain it runs as a strip -- smoothed into the city's
// limits.
//
// Inside, the DEPTH field (distance to those limits) lays the city out: the deepest ground is the
// downtown grid; midtown's boulevard and the outskirts' ring streets are contours of depth, so they
// run parallel to the coast and the foothills; the streets between them run straight down the depth,
// from the heart to the edge.

#include "../polygon.h"            // Vec2
#include "../../terrain_field.h"   // HeightField

#include <functional>
#include <vector>

namespace engine {
namespace plan {

struct LandShapeParams {
    double cell = 10.0;            // field grid (m)
    double targetArea = 4e6;       // m^2 the city grows to
    double slopeWeight = 25.0;     // cost per metre x (1 + slopeWeight * slope)
    double riseWeight = 0.02;      // ...+ riseWeight * metres above the heart
    double smooth = 40.0;          // limits smoothed at this radius (m): no one-cell fingers
    // THE WATERFRONT PULLS: growth is cheaper near the sea (below seaLevel), by up to coastPull,
    // fading over coastReach -- a city runs down to its shore rather than along the plain behind it
    double seaLevel = -1e9;        // off
    double coastPull = 0.6, coastReach = 400.0;
    // WATER INSIDE THE CITY (a river, a lake): part of the footprint -- the city spans it -- but an
    // EDGE to the depth field, so the streets run along its banks and stop at a riverside street;
    // what crosses it is a bridge, placed on purpose (city_plan). Glenn: "the city blocks don't
    // follow the contours of the river ... what I've observed in actual cities."
    std::function<bool(const Vec2&)> water;
};

struct LandShape {
    Vec2 origin;                   // cell (0, 0)
    double cell = 10.0;
    int n = 0;                     // cells a side
    std::vector<char> in;          // the footprint
    std::vector<float> depth;      // m to the footprint's edge or its water (negative beyond them)
    std::vector<char> water;       // the footprint's water (params.water)
    std::vector<std::vector<Vec2>> limits;   // the footprint's outline(s), closed
    Vec2 heart;                    // the deepest point
    double area = 0.0, maxDepth = 0.0;

    bool inside(const Vec2& p) const;
    double depthAt(const Vec2& p) const;      // bilinear
    Vec2 gradient(const Vec2& p) const;       // of depth, unit (toward the heart); zero on a flat
    // the depth `level` contour: closed loops and (where it meets the grid's edge) open runs, resampled
    // every `step` m, smallest first dropped below `minLength`
    // `smooth` m: a moving average along it, so a street on a ragged edge drives as a curve
    std::vector<std::vector<Vec2>> contour(double level, double step = 15.0, double minLength = 200.0, double smooth = 0.0) const;
    bool isWater(const Vec2& p) const;
    // the depth above which `share` of the footprint's area lies
    double depthHolding(double share) const;
};

// Grow over the square `half` round `seed`; `buildable` says where the city may stand at all (dry,
// not too steep: the land mask), `ground` prices the growth.
LandShape growLandShape(const HeightField& ground, const std::function<bool(const Vec2&)>& buildable,
                        const Vec2& seed, double half, const LandShapeParams& p = {});

// A moving average `window` m wide along a polyline sampled every `step` m (ends held on an open one).
std::vector<Vec2> smoothPolyline(const std::vector<Vec2>& pts, bool closed, double window, double step);

// Follow depth DOWN from `from` (away from the heart) until it falls to `stopDepth`: a street
// crossing the contours square to them. `step` m a point.
std::vector<Vec2> descendDepth(const LandShape& s, const Vec2& from, double stopDepth, double step = 15.0);

}  // namespace plan
}  // namespace engine

#endif
