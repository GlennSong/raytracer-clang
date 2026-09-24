#ifndef RAYTRACER_ENGINE_CITY_PLAN_SCENE_H
#define RAYTRACER_ENGINE_CITY_PLAN_SCENE_H

// A PLAN, BUILT: the lanes builder's scene from a CityPlan (ADR-0092 step 3).
//
// The plan is the design — a street graph with classes, a freeway with
// interchanges, and blocks with uses. The lanes builder takes a scene: classes,
// and edges whose paths are polylines (plus ramps attached to a carriageway).
// This is the one conversion between them, so what gets built is what was
// scored: every street edge of the plan becomes a chain between junctions, the
// freeway becomes its two one-way carriageways, and each interchange becomes a
// pair of ramps onto the arterial it crosses.

#include "city_plan.h"
#include "../roads/lanes/interchange.h"

#include <nlohmann/json.hpp>

namespace engine {
namespace plan {

struct SceneOptions {
    bool ramps = true;              // interchange ramps (off: the freeway is a wall)
    Real medianGap = 4.0;           // clear air between the two carriageways' decks (m): more than the
                                    // pavement's `closing`, so each is its own deck with its own parapet
    Real clearance = 8.0;           // the deck's height over a street it crosses
    int diamondsPerRoute = 6;       // interchanges per freeway route
    Real interchangeSpacing = 650;  // least distance between them along the route
};

// The scene, ready for `lanes_tool build` or a level's road entity.
nlohmann::json planToLanesScene(const CityPlan& plan, const SceneOptions& opt = {});

// The ground the scene is built on — the brief's relief over its world — as the scene's terrain
// block, and as the height field the lanes builder will make from it. The plan reads it too,
// to keep a town above the sea.
nlohmann::json sceneTerrain(const Brief& b);
HeightField sceneGround(const Brief& b);

// THE FREEWAY AS BOTH SIDES SEE IT. The plan clears streets out of the way of what the scene
// will build, so the two must build it from the same numbers; these are those numbers.
struct FreewaySection {
    Real laneW = 3.75;
    int lanes = 4;              // per carriageway
    Real carriageHalf = 10;     // a carriageway's half-width, shoulders included
    Real carriage = 12;         // centreline -> a carriageway's centreline
    Real edgeReach = 22;        // centreline -> the outer edge of the pavement
};
FreewaySection freewaySection(const Brief& b, Real medianGap = SceneOptions{}.medianGap);
roads::lanes::SystemOptions systemOptions();

// The ring as open chains — an edge cannot close on itself — split 400 m past each place an
// expressway crosses it, inside that system interchange's own stretch, where no diamond could
// stand anyway (a diamond cannot span a seam).
std::vector<std::vector<Vec2>> ringChains(const CityPlan& plan);
// An expressway's FREEWAY: from the map edge to where it ends just inside the ring, past its
// interchange's loops. Inside that it is a one-way boulevard pair to midtown's boulevard.
std::vector<Vec2> spurFreeway(const CityPlan& plan, std::size_t spur);
// Where each expressway meets the ring: which chain, and the four ramps. Carriageway ids are
// fw<k>_{a,b} for ring chain k and fw<chains + j>_{a,b} for expressway j.
struct SystemAt {
    std::size_t chain = 0, spur = 0;
    roads::lanes::SystemResult r;
};
std::vector<SystemAt> systemInterchanges(const CityPlan& plan, const std::vector<std::vector<Vec2>>& chains);

}  // namespace plan
}  // namespace engine

#endif
