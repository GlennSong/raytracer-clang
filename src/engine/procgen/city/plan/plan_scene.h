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

#include <nlohmann/json.hpp>

namespace engine {
namespace plan {

struct SceneOptions {
    bool ramps = true;              // interchange ramps (off: the freeway is a wall)
    Real carriagewayGap = 7.0;      // centre to each carriageway's centreline (m)
    Real clearance = 8.0;           // the deck's height over a street it crosses
    int diamondsPerRoute = 6;       // interchanges per freeway route
    Real interchangeSpacing = 650;  // least distance between them along the route
};

// The scene, ready for `lanes_tool build` or a level's road entity.
nlohmann::json planToLanesScene(const CityPlan& plan, const SceneOptions& opt = {});

}  // namespace plan
}  // namespace engine

#endif
