#ifndef RAYTRACER_ENGINE_PROCGEN_LANELAB_DECK_HEIGHT_H
#define RAYTRACER_ENGINE_PROCGEN_LANELAB_DECK_HEIGHT_H

// Height fields over the plane, one per lane (ADR-0083). own(): the lane's parent profile
// projected onto the spine (continuous, never nearest-station). deck(): the same, except
// within blendLen of a higher-ranked lane sharing its level, where the NEAREST partner's
// blended deck takes over smoothly and exactly at the seam — recursive through the rank
// chain, so a connector, a local lane and an arterial lane agree wherever they touch.

#include "engine/procgen/lanelab/lane_expand.h"
#include "engine/procgen/lanelab/polyline_ops.h"
#include "engine/procgen/lanelab/geom2d.h"
#include <memory>
#include <vector>

namespace engine {
namespace lanelab {

class DeckHeight {
public:
    DeckHeight(const RoadLabGraph& g, const LaneSet& L);
    void setPartners(std::vector<std::vector<int>> partners);   // per lane, higher rank first
    // The real footprints (after closing): distances are measured to their boundaries, so a
    // point on a gore fillet or a cap is at distance 0 on both sides of the seam.
    void setFootprints(const std::vector<std::vector<Ring>>& outers, const std::vector<std::vector<Ring>>& holes);
    const std::vector<std::vector<int>>& partners() const { return partners_; }

    double own(int lane, const Vec2& p) const;
    double deck(int lane, const Vec2& p) const;
    // A road's envelope (sidewalks, shoulders) blends like its lanes do: the union of their partners.
    double ownRoad(int edge, const Vec2& p) const;
    double deckRoad(int edge, const Vec2& p) const;
    // Distance from p to the lane's footprint: 0 inside, else the distance to its boundary.
    double distanceToLane(int lane, const Vec2& p, double radius) const;
    // The lane of one of `roads` whose footprint is nearest p (within radius), or -1.
    int nearestLane(const std::vector<int>& roads, const Vec2& p, double radius = 60.0) const;
    // A layer (sidewalk, shoulder, median) follows the PAVEMENT it borders: the nearest lane's deck. The
    // old rule — the nearest road SPINE's profile — gave a corner sidewalk beside a wide arterial the
    // height of the narrow climbing local next to it, a metre proud, kerb faces showing (Glenn's raised
    // slab in the 4-way). Falls back to the nearest spine when no lane is within reach.
    double layerHeight(const std::vector<int>& roads, const Vec2& p) const;

private:
    const RoadLabGraph& g_;
    const LaneSet& L_;
    std::vector<std::vector<int>> partners_;
    std::vector<std::vector<int>> roadPartners_;
    std::vector<std::unique_ptr<SegmentGrid>> grids_;                   // centrelines (fallback)
    struct Boundary { std::vector<SegmentGrid> outers, holes; std::vector<Ring> outerRings, holeRings; std::vector<Box2> outerBoxes; };
    std::vector<Boundary> boundaries_;
    double blendTo(double z, const std::vector<int>& partners, const Vec2& p) const;
};

}  // namespace lanelab
}  // namespace engine

#endif
