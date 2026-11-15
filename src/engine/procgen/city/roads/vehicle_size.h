#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_VEHICLE_SIZE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_VEHICLE_SIZE_H

// THE VEHICLES A ROAD IS CHECKED AGAINST (ADR-0435). The lane level opens each path through a junction to vehicle TAGS
// (a name a vehicle carries: "car", "bus"), checking it against one body per tag. This header is the one home of that
// body and of how it follows from the vehicle data, so the roads (which read the level's fleet themselves:
// lanes/road_model_cache.h levelVehicles) and the driving library (apps/driving/vehicle_class.h) derive the same way.
//
// Units: metres; angles in degrees where a name says so.

#include "rt_math.h"

#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

// A VEHICLE TAG and its body: the tightest radius its path may have, its width and its length. A connector tighter than
// the radius is closed to it. The length is nose to tail of the one rigid body that follows the path
// (engine::bodyOnPlan): through a bend that body lies on the chord between its bumpers and cuts inside the line, so it
// is swept at its length both where the builder checks a turn stays on the paved road and where it finds which paths'
// bodies can touch (swept_body.h). Zero: a body with no length, swept as a point on the line.
struct VehicleSize {
    std::string tag;
    double turningRadius = 0;
    double width = 0;
    double length = 0;
};

// ONE MORE VEHICLE CARRYING `tag`, into the list `sizes` (kept sorted by tag name, so the list and the road checked
// against it do not depend on the order vehicles are named in). A vehicle may use a path open to any one of its tags, so
// each tag stands for the most demanding vehicle that carries it: the LARGEST tightest-turn radius, the widest body,
// the longest (the body that cuts furthest inside a bend, ADR-0412). Then a connector open to a tag is one every vehicle
// with that tag can drive.
void addVehicleSize(std::vector<VehicleSize>& sizes, const std::string& tag, double turningRadius, double width,
                    double length);

// ONE WHEEL, as far as the wheelbase cares: how far along the body it sits (+ toward the nose) and whether it steers.
struct WheelSpot {
    Real along = 0;
    bool steered = false;
};

// THE WHEELBASE from a wheel layout: the middle of the steered wheels to the middle of the unsteered ones (a tandem's
// two back axles count as one, at their middle). False when there are no steered or no unsteered wheels.
bool wheelbaseFromWheels(const std::vector<WheelSpot>& wheels, Real& wheelbase);

// THE TIGHTEST TURN, from the wheelbase and the largest steering angle: the radius at the middle of the back axle is
// wheelbase / tan(angle) (the simple "bicycle" picture of a car). Zero when the angle is not between 0 and 90 degrees.
Real tightestTurnRadius(Real wheelbase, Real maxSteerDegrees);

}  // namespace roads::lanes
}  // namespace engine

#endif
