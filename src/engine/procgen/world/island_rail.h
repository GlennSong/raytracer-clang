#ifndef RAYTRACER_ENGINE_PROCGEN_WORLD_ISLAND_RAIL_H
#define RAYTRACER_ENGINE_PROCGEN_WORLD_ISLAND_RAIL_H

// THE ISLAND LOOP (Glenn, 2026-10-10: "it makes sense if it follows the highway in a loop and there's a station.
// There should be trains going in each direction"; 2026-10-11: "go ahead with the island loop").
//
// A double track (class "rail": one track each way, ADR-0153) that follows the island freeway round the island,
// beside it on the side away from the island's middle -- the towns' side -- and a station at every town:
//   * the line is the freeway carriageway on that side, offset outward (45 m clears every ramp there by 26 m on
//     island 8, never touches water, and crosses 10 roads -- which the builder separates: a track is never met
//     at grade until there are level crossings);
//   * the freeway's WATER floors carry over to the track, so it bridges each river where the freeway does;
//   * a station per town: the point of the line nearest the town's hub where a platform fits on the straight,
//     clear of road crossings and bridges -- a two-sided platform, one per track;
//   * the loop is cut between stations into edges (a closed edge cannot close on itself), each holding its
//     station's platform.
// It works on a lanes graph already written (the island's), so the line can be added without re-growing the island.

#include "../city/polygon.h"   // Vec2

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace engine {

struct RailTown {
    std::string name;
    Vec2 at;   // the town's hub (plan x, z)
};

struct IslandRailOptions {
    double offset = 45.0;          // carriageway centreline -> the track pair's centreline, outward
    double spacing = 10.0;         // the line's point spacing (m)
    double platformLength = 140.0; // a station's platform (an intercity train is ~95 m)
    double platformWidth = 4.5;
    double straightRadius = 900.0; // the tightest curve a platform may stand on
    double crossingClear = 150.0;  // no road crossing (a bridge, with its approach) this near a platform's ends
    double maxTownDistance = 2500.0;   // a town farther than this from the line gets no station
    std::string idPrefix = "rail_loop";
    // THE LINE'S CLASS on the island: the built-in rail (2.5%, fill only) rode 28 of 37 km of hills on viaducts.
    // A main line that cuts through what it cannot climb, at a mountain main line's grade.
    double gMax = 0.035;
    double balance = 0.5;
    // THE WAY IN (Glenn, 2026-10-11: "make it accessible"): per station a footway from the nearest street on the
    // nearer side, along the track beside the platforms, then over both tracks as a footbridge (stairs down to each
    // platform are the builder's). The leg along the track is the ramp's length: ~7 m up at the footway's 12 %.
    bool ways = true;
    double wayOffset = 14.0;       // the leg along the track, off the track pair's centreline
    double wayLeg = 60.0;          // its length
    double wayHold = 11.0;         // the footbridge held flat this far either side of the line (the platforms' backs)
    double wayReach = 10.05;       // the footbridge's far end, past the line (over the far platform's back edge)
    double wayMaxStreet = 400.0;   // no street nearer than this: no way in
};

struct IslandRailStation {
    std::string name;
    Vec2 at;            // the platform's middle
    double station = 0; // along the loop (m)
    double fromTown = 0;
    std::string street;     // the street its way in leaves from ("" -- none)
    double way = 0;         // the way's length
};

struct IslandRailResult {
    std::vector<nlohmann::json> edges;   // lanes-graph edges, class "rail", in running order, joined end to start
    std::vector<nlohmann::json> ways;    // class "footway": each station's way in
    nlohmann::json railClass;            // the graph's "rail" class settings (merged over the built-in)
    std::vector<IslandRailStation> stations;
    std::vector<std::string> skipped;    // towns with no station, and why
    double length = 0;
    int crossings = 0;                   // roads the line crosses (each a grade separation)
    int floors = 0;                      // water floors carried over from the freeway
};

// `graph`: a lanes graph with the freeway as class "freeway" edges whose ends join into closed chains (island
// scenes write two: one per carriageway). The line follows the chain farther out.
IslandRailResult islandRailLoop(const nlohmann::json& graph, const std::vector<RailTown>& towns, const IslandRailOptions& o = {});

}  // namespace engine

#endif
