#ifndef RAYTRACER_ENGINE_PROCGEN_TRAILS_H
#define RAYTRACER_ENGINE_PROCGEN_TRAILS_H

// HIKING TRAILS ON THE GROUND (ADR-0134, Glenn: "Dirt paths for hiking trails ... Trails should hopefully
// just generate some texturing"). The terrain block's "trails" -- polylines [[x, z], ...] the island
// planner routed (island_world.h planTrails) -- indexed by segment bins, so the ground cover can ask
// "how far to the nearest path" per sample: dirt within a metre, trees, rocks and grass kept off.
// No ground is moved; a trail is texture and clearance.

#include "city/polygon.h"   // Vec2

#include <nlohmann/json.hpp>

#include <vector>

namespace engine {

class TrailNetwork {
public:
    explicit TrailNetwork(const std::vector<std::vector<Vec2>>& lines);
    static TrailNetwork fromJson(const nlohmann::json& trails);
    // the distance (m) from (x, z) to the nearest trail's centre line, capped at maxD
    double distance(double x, double z, double maxD = 8.0) const;
    bool empty() const { return segs_.empty(); }
    std::size_t segments() const { return segs_.size(); }

private:
    struct Seg { Vec2 a, b; };
    std::vector<Seg> segs_;
    double bin_ = 32.0, x0_ = 0, z0_ = 0;
    int nx_ = 0, nz_ = 0;
    std::vector<std::vector<int>> bins_;
};

}  // namespace engine

#endif
