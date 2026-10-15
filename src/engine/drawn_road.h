#ifndef RAYTRACER_ENGINE_DRAWN_ROAD_H
#define RAYTRACER_ENGINE_DRAWN_ROAD_H

#include "procgen/city/polygon.h"   // Vec2

#include <array>
#include <unordered_map>
#include <vector>

namespace engine {

class World;

// THE ROAD AS IT WAS ACTUALLY DRAWN: every collider triangle on the ROADS render
// layer — asphalt, shoulders, kerbs, medians, sidewalks and junction pads as
// built, for either road builder — in plan, bucketed on an 8 m grid so a query
// touches a few dozen triangles.
//
// The deck field (RoadDeckField) answers "how far inside the driving surface",
// which is right for traffic and too narrow for planting: its half-width is
// lanes plus shoulder, so a junction flare, a median or a widened intersection
// mouth reads as clear. Tree placement and the level gate that checks it
// (city_furniture_never_stands_in_a_road) both ask THIS, so they cannot
// disagree about where the road is.
struct DrawnRoad {
    struct Tri { Vec2 a, b, c; };
    std::vector<Tri> tris;
    // THE HEIGHTS, kept only when asked (gatherDrawnRoad's withHeights): the corners' y per triangle, for topAt.
    // Plan-only users (the sealed-ground grass mask lives through play) don't pay for them.
    std::vector<std::array<float, 3>> heights;
    std::unordered_map<long long, std::vector<int>> cells;
    static constexpr double kCell = 8.0;

    void add(const Vec2& a, const Vec2& b, const Vec2& c);
    void add(const Vec3& a, const Vec3& b, const Vec3& c);   // with heights
    // The highest drawn surface at (x, z) at or below `ceiling` -- the sidewalk a street tree's pit is cut in, not
    // the viaduct over it. False where nothing is drawn (walls have no area in plan) or nothing has heights.
    bool topAt(double x, double z, double* y, double ceiling = 1e30) const;
    // Does anything drawn stand OVER a thing at (x, z) whose foot is at footY and top at topY, within `radius` in plan
    // -- a deck or a track a tree would grow up through (Glenn, 2026-10-15: "a tree going through the elevated train
    // deck")? A surface counts from half a metre over the foot (not the pavement it stands on) to the top. Heights
    // required (gatherDrawnRoad(world, true)).
    bool overhead(double x, double z, double footY, double topY, double radius) const;
    // Is (x, z) on a drawn road triangle?
    bool covers(double x, double z) const;
    // ...or within `margin` metres of one (a trunk radius, a canopy allowance).
    bool near(double x, double z, double margin) const;
    bool empty() const { return tris.empty(); }
};

// Gather the drawn road from the world as loaded so far.
DrawnRoad gatherDrawnRoad(World& world, bool withHeights = false);

}  // namespace engine

#endif
