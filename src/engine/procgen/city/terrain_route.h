#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_TERRAIN_ROUTE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_TERRAIN_ROUTE_H

// A ROAD UP A MOUNTAIN: the cheapest path between two points over a heightfield that a road of a
// given grade can drive -- following the contours where it can, climbing in switchbacks where the
// slope is steeper than the road may be. After Galin et al. 2010, "Procedural Generation of Roads":
// A* on a grid whose moves reach up to three cells in 32 directions (so a route can take a gentle
// diagonal across a slope, not only the steep way up it), each move priced by its length, its grade
// against the road's limit, and how sharply it turns from the move before. The state is (cell,
// incoming direction), so the turn is priced exactly; a hairpin is three or more turning moves.
//
// Plan- or world-space (x, y) in, whatever the height function takes; the caller builds the road
// (profile, cut and fill) from the polyline. Engine vocabulary: a mountain road, a pass, a trail,
// a rail line (with a railway's grade) -- anything that must climb within a limit.

#include "polygon.h"            // Vec2
#include "../terrain_field.h"   // HeightField

#include <functional>
#include <vector>

namespace engine {

struct TerrainRouteParams {
    double cell = 8.0;             // grid step (m)
    double maxGrade = 0.08;        // the road's grade: above this each move pays steeply...
    double hardGrade = 0.14;       // ...and above this it may not go at all
    double gradeWeight = 400.0;    // cost multiplier on (grade - maxGrade)^2 per metre
    double flatWeight = 2.0;       // a mild preference for flatter ground at any grade (x grade per metre)
    double turnWeight = 6.0;       // cost per radian^2 of turn between consecutive moves (x cell)
    // cost per (change of grade)^2 between consecutive moves, per metre: ground whose slope along the
    // route changes smoothly is ground a road's smoothed profile can HUG -- over jagged ground the
    // profile bridges the dips (decks on piers) and cuts the humps
    double bendWeight = 0.0;       // off by default: on metro's jagged range it traded switchbacks for grade (ADR-0103)
    double maxTurnDeg = 60.0;      // sharper than this between two moves is not allowed
    double margin = 500.0;         // the search box: the endpoints' bounds grown by this (m)
    // optional: where the road may not go (water, a city block); true = blocked
    std::function<bool(double x, double y)> blocked;
};

struct TerrainRoute {
    std::vector<Vec2> points;      // start .. goal, simplified; empty when there is no way
    double length = 0.0;           // m, along the points
    double climb = 0.0;            // m, total ascent along it
    double worstGrade = 0.0;       // the steepest simplified segment
    long expanded = 0;             // search states expanded (diagnostics)
};

TerrainRoute routeOnTerrain(const HeightField& height, const Vec2& from, const Vec2& to,
                            const TerrainRouteParams& p = {});

// TIGHTEN a routed road (string-pulling): wherever a straight line between two of its points is
// drivable -- within `maxGrade` end to end, the ground never more than `maxCutFill` off that line,
// no longer than `maxStraight`, nothing `blocked` -- the detour between them goes. Glenn: "a really
// tight hairpin. We should smooth out spikes like that." A spike (up a spur and back), a zigzag on
// flat ground, a waypoint forced off the natural line: all drivable straight, all removed. A real
// switchback stays -- the straight line up the slope is too steep.
struct TightenParams {
    double maxGrade = 0.08, maxCutFill = 8.0, maxStraight = 900.0, sample = 10.0;
    std::function<bool(double x, double y)> blocked;
};
std::vector<Vec2> tightenRoute(const HeightField& height, const std::vector<Vec2>& pts, const TightenParams& p = {});

// ROUND its corners: resampled every `step` m and averaged over `radius` m either side, so no bend is
// tighter than a car takes at the road's speed (a mountain road's hairpin ~15 m, a freeway's ~300 m).
// The ends stay where they are.
std::vector<Vec2> roundRoute(const std::vector<Vec2>& pts, double radius, double step = 10.0);

// Length, climb and steepest grade (over `window` m) of a polyline on the ground.
TerrainRoute measureRoute(const HeightField& height, const std::vector<Vec2>& pts, double window = 50.0);

}  // namespace engine

#endif
