#ifndef RAYTRACER_APPS_DRIVING_VEHICLE_CLASS_H
#define RAYTRACER_APPS_DRIVING_VEHICLE_CLASS_H

// WHAT THE DRIVING MODEL KNOWS ABOUT EACH KIND OF VEHICLE (driving-model-design.md 8 and 5.4; issue #248): how long and
// wide it is, how tightly it turns, how hard it speeds up and brakes, how fast it can go, and which lanes it may use
// (by tag name). One of these is a "vehicle class"; the host hands the driving model a table of them.
//
// Where the numbers come from (AGENTS.md "Where each number lives"):
//   - DERIVED from the vehicle data that already exists: length and width from the body, wheelbase from the wheel
//     layout, tightest turning radius from the wheelbase and the largest steering angle. The functions below do the
//     deriving, so every host derives the same way.
//   - AUTHORED once, in the vehicle data's `driving` block (assets/scripts/vehicles.lua), for what cannot be derived:
//     speeding up, braking, top speed, tags. The reader is scripting/vehicle_class_lua.h.
// The defaults below are what a field holds when the data gives nothing (a build with no scripting); they are not a
// second copy of any vehicle's numbers.
//
// Nothing here knows a kind of vehicle by name: class names and tag names are data.
//
// Units: metres, seconds, radians unless a name says degrees.

#include "rt_math.h"

#include <cstddef>
#include <string>
#include <vector>

namespace driving {

using engine::Real;

// THE DEFAULTS for the authored fields, used when the vehicle data gives none (a build with no scripting). Plausible
// for an ordinary car, not good enough to cite; a vehicle that matters has its own numbers in its `driving` block.
constexpr Real kUnauthoredSpeedUp = 1.5;       // m/s2, comfortable speeding-up
constexpr Real kUnauthoredBrake = 2.0;         // m/s2, comfortable braking
constexpr Real kUnauthoredHardestBrake = 7.0;  // m/s2, hardest braking
constexpr Real kUnauthoredTopSpeed = 40.0;     // m/s (144 km/h)

// THE DEFAULTS for the derived fields, used ONLY when the data has no wheels to derive them from (the city sim's
// built-in fallback fleet, a box per slot). The wheelbase is taken as this share of the length (vehicle_classes.lua
// bounds it to 0.50-0.70 for real cars), and the largest steering angle is the physics rig's default.
constexpr Real kUnknownWheelbaseShareOfLength = 0.6;
constexpr Real kUnknownMaxSteerDegrees = 30.0;

// THE SANITY BOUND on steering: no road vehicle's wheels turn further than this from straight ahead (real cars stop
// near 35-45 degrees). A turning radius tighter than this angle would give, for the class's wheelbase, is impossible.
constexpr Real kSteepestPossibleSteerDegrees = 60.0;

// ONE KIND OF VEHICLE, as the driving model sees it.
struct VehicleClass {
    std::string name;            // the vehicle data's class name; unique within a table

    // Derived from the body and the wheels.
    Real length = 0;             // nose to tail
    Real width = 0;              // side to side
    Real wheelbase = 0;          // the middle of the front (steered) axle to the middle of the back axle(s)
    // The tightest circle it can drive, as the radius traced by the MIDDLE OF THE BACK AXLE (the point that follows
    // the path most simply: it never slides sideways). The front and the corners swing wider.
    Real tightestTurnRadius = 0;

    // Authored in the vehicle data's `driving` block.
    Real speedUp = kUnauthoredSpeedUp;              // m/s2, comfortable speeding-up (IDM's a)
    Real brake = kUnauthoredBrake;                  // m/s2, comfortable braking (IDM's b)
    Real hardestBrake = kUnauthoredHardestBrake;    // m/s2, hardest braking it can do (Krauss's b_max)
    Real topSpeed = kUnauthoredTopSpeed;            // m/s
    // Tag NAMES, sorted and without repeats, matched against a lane's allowed tags (design 5.4).
    std::vector<std::string> tags;

    // A TRAILER it pulls, when it pulls one (nothing reads these yet). Zero length: no trailer.
    Real trailerLength = 0;      // the trailer's own nose to tail
    // Where the trailer hangs on: metres ahead of the middle of the back axle (negative: behind it, a tow ball).
    Real hitchAheadOfBackAxle = 0;

    bool hasTrailer() const { return trailerLength > 0; }
    bool hasTag(const std::string& tag) const;   // binary search over the sorted tags
};

// ONE WHEEL, as far as the wheelbase cares: how far along the body it sits (metres, + toward the nose) and whether it
// steers.
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

// A CLASS FROM WHAT THE VEHICLE DATA STATES: its name, its body's length and width, its wheelbase and largest
// steering angle. The authored fields keep their defaults until the `driving` block is read over them.
VehicleClass deriveVehicleClass(const std::string& name, Real length, Real width, Real wheelbase,
                                Real maxSteerDegrees);

// THE TABLE the driving model is given. Classes keep the order they were added in (index = that order); a name finds
// its class by a search over the names (the table is read at load, never in a hot loop).
class VehicleClasses {
public:
    // Add a class at the end; its index.
    std::size_t add(VehicleClass c);
    std::size_t size() const { return classes_.size(); }
    bool empty() const { return classes_.empty(); }
    const VehicleClass& at(std::size_t index) const { return classes_[index]; }
    // The class of that name, or null.
    const VehicleClass* find(const std::string& name) const;
    // Its index, or -1.
    int indexOf(const std::string& name) const;
    const std::vector<VehicleClass>& all() const { return classes_; }

private:
    std::vector<VehicleClass> classes_;
};

// WHAT EACH VEHICLE TAG ASKS OF A PATH THROUGH A JUNCTION. The road builder opens each connector to vehicle TAGS (a
// lane's `open_to` names tags, design 5.4), checking it against one turning radius and one width per tag. A class may
// use a path open to any one of its tags (route.h mayUse), so each tag must stand for the most demanding class that
// carries it: its turning radius is the LARGEST tightest-turn radius among them, its width the widest body. Then a
// connector open to a tag is one every class with that tag can drive. (Radii and widths are the classes' own, derived
// from the body and the wheels: deriveVehicleClass.)
struct TagSize {
    std::string tag;
    Real turningRadius = 0;
    Real width = 0;
};
// Every tag any class carries, sorted by name, so the list (and the road checked against it) does not depend on the
// order the classes were added in.
std::vector<TagSize> tagSizes(const VehicleClasses& classes);

// WHAT IS WRONG with a class, in plain words, one line each (empty when nothing is). Checks: a name; positive length,
// width and wheelbase, the wheelbase no longer than the vehicle; a turning radius no tighter than the steepest possible
// steer allows for the wheelbase; positive speeding-up, braking and top speed; hardest braking at least the
// comfortable; a trailer's length not negative.
std::vector<std::string> validate(const VehicleClass& c);
// ...and over a whole table: every class's problems (each line names its class), and two classes with one name.
std::vector<std::string> validate(const VehicleClasses& table);

}  // namespace driving

#endif
