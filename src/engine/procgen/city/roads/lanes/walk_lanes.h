#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_WALK_LANES_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_WALK_LANES_H

// THE WALKS THE BUILDER LAYS (ADR-0465, #306 task 4 piece 2): where people on foot walk, laid by the road build into the
// same centre lines as the car lanes (road_model_parts.h LaneCurves) and taken into the road model's built layer
// (RoadModel::walks, ModelCorner::walk). Nothing reads them yet but the tests; piece 3 lays them as lane-level paths.
//
// Words used here:
//   - a SIDEWALK LANE is the middle line of one kerb's sidewalk along a road: laid as a car lane is (the builder's own
//     frame of the road, lane_expand.h ribbonLane), the carriageway's half width and half the sidewalk out from the
//     spine toward that kerb, along the whole road. Its `dir` is the kerb it runs beside (+1: the kerb of the traffic
//     running along the spine, -1 the other), its kind the path users' word for it (assets/data/path_users.json);
//   - a FOOTWAY's lane is the lane expand lays for a footway road (its class's `footway`), of kind "footway";
//   - a CORNER WALK is the walk round a junction's kerb from one arm to the next round its ring (road_model.h
//     ModelCorner): the line round the DRAWN sidewalk (Pavement::sidewalk, and footways' own paving) kWalkFromKerb in
//     from its kerb, between the two arms' KERB ENDS. It is read off the drawn pavement, never a chord: the drawn walk is
//     shrunk by kWalkFromKerb, and the walk is the shorter way along that shrunk edge between the points of it nearest
//     the two kerb ends. An arm's kerb end is its crosswalk's end on the corner's kerb (where a person steps off), or with
//     no crosswalk the carriageway's edge level with its stated box edge (a footway arm: the footway's middle there).
//     The same for a compact junction and for the pairs of a spread junction's arms that meet it at different points,
//     which the road file cannot state (road_model.h CornerGap::ApartAtSpread): the drawn pavement between them says
//     where the kerb runs. Where no drawn pavement joins the two kerb ends, no walk is laid, and the gap says why (never
//     guessed).

#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/road_model_parts.h"

#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

// A corner walk keeps this far in from the drawn sidewalk's edges (m): off the kerb, and off the back of the pavement.
constexpr double kWalkFromKerb = 0.5;
// How far round a corner's drawn pavement the window reaches beyond its two kerb ends (m): the drawn walk is cut to this
// box before it is shrunk.
constexpr double kCornerWindow = 30.0;
// A SEAM IN THE DRAWN SIDEWALK no wider than this (m) is walked over: two roads' sidewalks drawn up to one another (each
// cut at its own end, at its own angle, onto the millimetre grid) meet with a hairline between them, which the mesh does
// not show. Measured on traffic_lab and river_town: closing 5 cm joins 34 and 13 corners that hairlines split; 20 cm and
// 50 cm join only 3-7 more, and close real gaps (a narrow median between two carriageways' sidewalks) as well.
constexpr double kSeamBridge = 0.05;

// (One corner walk, or why a ring pair has none, is lanes.h WalkCorner: the build's Result holds them.)

// LAYS THE WALKS of a build (`r` after its pavement, heights and lane lookup): every road's sidewalk lanes, each ring
// pair's corner walk, appended to the build's centre lines and listed in r.lanes.walks; the corners and the gaps in
// r.walkCorners. Called once, by build().
void layWalks(Result& r);

}  // namespace roads::lanes
}  // namespace engine

#endif
