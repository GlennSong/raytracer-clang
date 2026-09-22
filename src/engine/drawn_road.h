#ifndef RAYTRACER_ENGINE_DRAWN_ROAD_H
#define RAYTRACER_ENGINE_DRAWN_ROAD_H

#include "procgen/city/polygon.h"   // Vec2

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
    std::unordered_map<long long, std::vector<int>> cells;
    static constexpr double kCell = 8.0;

    void add(const Vec2& a, const Vec2& b, const Vec2& c);
    // Is (x, z) on a drawn road triangle?
    bool covers(double x, double z) const;
    // ...or within `margin` metres of one (a trunk radius, a canopy allowance).
    bool near(double x, double z, double margin) const;
    bool empty() const { return tris.empty(); }
};

// Gather the drawn road from the world as loaded so far.
DrawnRoad gatherDrawnRoad(World& world);

}  // namespace engine

#endif
