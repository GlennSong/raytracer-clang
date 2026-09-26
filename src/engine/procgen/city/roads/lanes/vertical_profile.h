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
struct CrossingMeet { std::size_t hi = 0, lo = 0; Vec2 p; double sHi = 0, sLo = 0; bool ramp = false, tee = false, sep = false; };
// Two roads that may never meet at grade where their centrelines cross: a freeway and any road but a ramp.
bool mustSeparate(const EdgeSpec& a, const EdgeSpec& b);
std::vector<CrossingMeet> crossingMeets(const RoadLabGraph& g);
// crossingConsistency over the cached meets: the same pairs, in the same order, the same result.
double crossingConsistency(RoadLabGraph& g, const std::vector<CrossingMeet>& meets, double maxDz, double rampMaxDz, double radius = 100.0);
void applyFloors(EdgeSpec& e, const RoadClassSpec& c);   // re-impose a through road's floor holds after other passes moved it
void rampProfile(RoadLabGraph& g, EdgeSpec& e, const HeightField& terrain);
double projectZ(const EdgeSpec& e, const Vec2& p);
double maxGrade(const EdgeSpec& e);

}  // namespace roads::lanes
}  // namespace engine

#endif
