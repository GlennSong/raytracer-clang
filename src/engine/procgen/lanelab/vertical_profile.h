#ifndef RAYTRACER_ENGINE_PROCGEN_LANELAB_VERTICAL_PROFILE_H
#define RAYTRACER_ENGINE_PROCGEN_LANELAB_VERTICAL_PROFILE_H

// Elevation lives on the spine: z(s) per edge. Through roads smooth the terrain under a
// grade limit (designed at 80 % of the class limit so junction corrections have headroom);
// roads that meet at a node or cross at grade agree there; ramps hold their hosts' heights
// until departure / from the gore and smoothstep between.

#include "engine/procgen/lanelab/road_graph_spec.h"
#include "engine/procgen/terrain_field.h"

namespace engine {
namespace lanelab {

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
void applyFloors(EdgeSpec& e, const RoadClassSpec& c);   // re-impose a through road's floor holds after other passes moved it
void rampProfile(RoadLabGraph& g, EdgeSpec& e, const HeightField& terrain);
double projectZ(const EdgeSpec& e, const Vec2& p);
double maxGrade(const EdgeSpec& e);

}  // namespace lanelab
}  // namespace engine

#endif
