#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_VERTICAL_PROFILE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_VERTICAL_PROFILE_H

// Elevation lives on the spine: z(s) per edge. Through roads smooth the terrain under a
// grade limit (designed at 80 % of the class limit so junction corrections have headroom);
// roads that meet at a node or cross at grade agree there; ramps hold their hosts' heights
// until departure / from the gore and smoothstep between.

#include "engine/procgen/city/roads/lanes/road_graph_spec.h"
#include "engine/procgen/terrain_field.h"

namespace engine {
namespace roads::lanes {

constexpr double kDesignGrade = 0.8;

void throughProfile(EdgeSpec& e, const HeightField& terrain, const RoadClassSpec& c);
// The profile a through road of this class would get along an arbitrary polyline (for planning estimates).
std::vector<double> profileAlong(const std::vector<Vec2>& xy, const HeightField& terrain, double window, double gMax);
// Coincident endpoints take the mean; an endpoint on another road's interior takes the host's
// height (unless they differ by >= maxDz: that is a viaduct, not a node). Linear end-to-end
// correction. Returns the largest mismatch found before correction.
double nodeConsistency(RoadLabGraph& g, double tol, double maxDz);
struct NodeMismatchWhere { Vec2 at; std::string a, b; bool atEnd = true; };
double nodeMismatch(const RoadLabGraph& g, double tol, double maxDz, NodeMismatchWhere* where = nullptr);
// Through roads crossing at grade: the higher-ranked road is authoritative, the other takes a
// smooth bump sized so it never spends more than the design headroom. Returns the worst mismatch.
double crossingConsistency(RoadLabGraph& g, double maxDz, double rampMaxDz, double radius = 100.0);   // rampMaxDz: a street meets a ramp only this close; higher is structure

// WHERE the roads meet does not change while their HEIGHTS are solved (only z moves), and neither does
// which of the two gives way (rank, then order). So the meeting points -- every pair's crossings and
// ends under the other's band, the shared nodes left out -- are found ONCE and every round of the agree
// loop re-levels just those. Finding them is an all-pairs pass over the edges' polylines: on the island
// (3,866 edges, a 34 km freeway in four chains) it cost 23 minutes when every round repeated it.
// `tee`: one STREET ending on another (neither a freeway nor a ramp) -- a junction, always levelled, never
// read as a grade separation however far apart the two start (a river bridge's end meeting its riverside
// street 2.8 m off was lifted to overpass clearance, and the agree loop diverged to 10 m)
// sep: with Rules::freewaySeparates, a FREEWAY's centreline crossing another road's (not a ramp) -- always a grade separation,
// never levelled, whatever the two heights (island 8's pass rose to the freeway and met it at grade).
// `declared`: the graph planned this crossing (CrossingSpec); then `sep` is its kind (a bridge, `hi` over) and `level` an
// at-grade meeting the heights are made to agree at however far apart they stand -- no kind read off the heights
struct CrossingMeet { std::size_t hi = 0, lo = 0; Vec2 p; double sHi = 0, sLo = 0; bool ramp = false, tee = false, sep = false, level = false, declared = false; };
// THE CROSSINGS A GRAPH THAT DECLARES THEM DID NOT (ADR-0215): every meet of two roads away from a shared node -- a
// crossing, or an end on the other's interior -- with no planned kind, "a / b at (x, y)". Empty for a graph that
// declares none (its kinds are inferred, as they always were).
std::vector<std::string> undeclaredCrossings(const RoadLabGraph& g, const std::vector<CrossingMeet>& meets);
// crossingConsistency's worst level-crossing mismatch, measured only (a designed graph is checked, not corrected)
double crossingMismatch(const RoadLabGraph& g, const std::vector<CrossingMeet>& meets, double maxDz, double rampMaxDz, const CrossingMeet** worstAt = nullptr);
// Two roads that may never meet at grade where their centrelines cross: a freeway and any road but a ramp.
bool mustSeparate(const EdgeSpec& a, const EdgeSpec& b);
// A TRACK crossing anything (a road, a ramp, the freeway) is a grade separation, whatever the scene's rules: there are
// no level crossings yet (the island loop, 2026-10-11). Two of the graph's crossings that must be structure.
bool separates(const RoadLabGraph& g, const EdgeSpec& a, const EdgeSpec& b);
bool anyTrack(const RoadLabGraph& g, const EdgeSpec& a, const EdgeSpec& b);   // a track is structure where a road ends under it too
std::vector<CrossingMeet> crossingMeets(const RoadLabGraph& g);
// crossingConsistency over the cached meets: the same pairs, in the same order, the same result.
double crossingConsistency(RoadLabGraph& g, const std::vector<CrossingMeet>& meets, double maxDz, double rampMaxDz, double radius = 100.0);
void applyFloors(EdgeSpec& e, const RoadClassSpec& c);   // re-impose a through road's floor holds after other passes moved it
// A LINE IN SECTIONS (a city's light rail, cut between stations): each track section's holds, carried onto the section
// joined to it end to start (as chainFloors), one joint on. A lift's approach ran out over the joint on its own section
// only, and the next section met it 4 m lower (Saltwood). Recompute whenever a track's floorPts change.
void chainTrackFloors(RoadLabGraph& g, double tol);
void rampProfile(RoadLabGraph& g, EdgeSpec& e, const HeightField& terrain);
double projectZ(const EdgeSpec& e, const Vec2& p);
double maxGrade(const EdgeSpec& e);

}  // namespace roads::lanes
}  // namespace engine

#endif
