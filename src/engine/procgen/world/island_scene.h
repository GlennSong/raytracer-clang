#ifndef RAYTRACER_ENGINE_PROCGEN_WORLD_ISLAND_SCENE_H
#define RAYTRACER_ENGINE_PROCGEN_WORLD_ISLAND_SCENE_H

// THE ISLAND IN 3D (ADR-0112): one lanes scene for the whole island -- the level loader wires a
// level's terrain, lots, deck, nav and road graph from ONE lanes city, so every place and every
// island road goes into the same scene, as metro_planned's city, towns and loop do:
//   * every place's streets, from its plan (planToLanesScene), ids prefixed per place;
//   * the island FREEWAY as two carriageways offset from its route (12 m each side), split into two
//     chains each at the point farthest from any ramp (a closed edge cannot close on itself), at
//     grade -- raised by FLOORS only where a road passes under it (the clearance its diamonds were
//     sized for) and where it bridges a river;
//   * its RAMPS as diamondRamps wrote them, their anchors re-pointed to the chain holding their gore;
//   * the PASS and the MOUNTAIN road (class "rural") and the LINKS (class "collector"), bridged over
//     rivers, ending in the places' streets;
//   * one terrain grid over the island's land, at 20 m, on the island's own (eroded) ground.

#include "island_world.h"

#include <nlohmann/json.hpp>
#include <vector>

namespace engine {

struct IslandSceneOptions {
    double carriage = 12.0;       // freeway centreline -> a carriageway's centreline
    double underClearance = 8.2;  // deck over the road under it (under_clearance 5 + slab + structure 1.6: the builder asks 7.8)
    double bridgeOverWater = 6.0; // deck over a river's water
    double gridRes = 20.0;        // the scene's terrain grid (m)
    // A WINDOW (half > 0): only what lies in the square (centre, half) -- places' edges that reach
    // into it, island roads and the freeway clipped to it, ramps whose roads survive -- over a grid
    // of just that square. For iterating on one part of the island in a build of a minute or two;
    // the first step toward building the island in tiles.
    Vec2 windowCentre{0, 0};
    double windowHalf = 0.0;
};

// `placeScenes[k]`: site k's planToLanesScene output (empty json: none).
nlohmann::json islandLanesScene(const IslandWorld& w, const std::vector<nlohmann::json>& placeScenes, const IslandSceneOptions& o = {});

}  // namespace engine

#endif
