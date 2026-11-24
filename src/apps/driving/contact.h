#ifndef RAYTRACER_APPS_DRIVING_CONTACT_H
#define RAYTRACER_APPS_DRIVING_CONTACT_H

// BODY AGAINST BODY ON THE PLAN (issue #283; sitting 7: contact and crashes). THE ONE place that asks which car bodies
// touch: the measurements table counts it (measure.h, the bodies-touching row), and the driving's crash step acts on it
// (driving.h: a contact, a seeded roll, a wreck). Every pair of car bodies -- a rectangle on the plan, its middle
// halfway between the bumpers carPose gives, pointing from the rear bumper to the front -- is tested for overlap, with a
// sweep along x so each is tested only against those whose boxes it reaches into. It knows nothing about paths.
//
// Same result every run: bodies are swept in order of their lowest x, then car id; contacts come out lower car first.
//
// Units: metres.

#include "apps/driving/traffic_state.h"   // CarId, CarPose

#include <cstddef>
#include <vector>

namespace driving {

// A CAR'S BODY ON THE PLAN.
struct PlanBody {
    CarId car;
    double x = 0, z = 0;            // its middle
    double ux = 1, uz = 0;          // unit, from its rear to its front
    double halfLength = 0, halfWidth = 0;
    double height = 0;              // the road's height under its middle
};
// The body of a car at `pose`, `length` long and `width` wide (the host's drawing centres the vehicle between the
// bumpers and points it from the rear bumper to the front: citysim DrivingHost::mirror). The lane graph's own body rule
// (engine::bodyOnPlan), the one the road builder sweeps along each path for its crossing points (ADR-0412).
PlanBody planBody(CarId car, const CarPose& pose, double length, double width);
// How far a body reaches along a unit direction (dx, dz) from its middle.
double reachAlong(const PlanBody& b, double dx, double dz);
// HOW DEEP TWO BODIES OVERLAP on the plan: the shortest push that would part them (metres; separating axes over the
// two rectangles), or zero when they do not overlap.
double bodyOverlapDepth(const PlanBody& a, const PlanBody& b);
// Two bodies whose roads differ in height by more than this are on different decks (a street under a bridge), not
// touching: the road builder's same-level figure (Rules::sameLevelDz), which the crossing points use too.
constexpr double kTouchSameLevelMetres = 1.0;
// Overlaps shallower than this (metres) are rounding, not touching: a millimetre.
constexpr double kTouchToleranceMetres = 1e-3;

// THE RULE SET'S NUMBERS for what a contact does (driving.h CrashCounts; two-drivers-meeting-design.md section 8): the
// chance that a contact between two cars is a crash (the rest go on), and how long a wreck stands before it is cleared.
constexpr const char* kNumberCrashChance = "crash_chance";
constexpr const char* kNumberWreckSeconds = "wreck_seconds";

// TWO BODIES TOUCHING: indices into the bodies given (`a` the lower car id), how deep, and the point between them.
struct BodyContact {
    std::size_t a = 0, b = 0;
    double depth = 0;
    double x = 0, z = 0;
};
// EVERY PAIR OF `bodies` THAT TOUCHES (deeper than kTouchToleranceMetres, at one level), in (car a, car b) order.
// `sweep` is working memory kept by the caller.
void findContacts(const std::vector<PlanBody>& bodies, std::vector<std::size_t>& sweep, std::vector<BodyContact>& out);

}  // namespace driving

#endif
