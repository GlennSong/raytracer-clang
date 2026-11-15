#include "engine/procgen/city/roads/vehicle_size.h"

#include <algorithm>
#include <cmath>

namespace engine {
namespace roads::lanes {

namespace {

constexpr Real kRightAngleDegrees = 90.0;
constexpr Real kHalf = 0.5;

// The middle of the spread of `along` over the wheels that do (or do not) steer; false when there are none.
bool middleOf(const std::vector<WheelSpot>& wheels, bool steered, Real& middle) {
    bool any = false;
    Real lo = 0, hi = 0;
    for (const WheelSpot& w : wheels) {
        if (w.steered != steered) continue;
        if (!any) {
            lo = hi = w.along;
            any = true;
        } else {
            lo = std::min(lo, w.along);
            hi = std::max(hi, w.along);
        }
    }
    if (any) middle = kHalf * (lo + hi);
    return any;
}

}  // namespace

void addVehicleSize(std::vector<VehicleSize>& sizes, const std::string& tag, double turningRadius, double width,
                    double length) {
    std::vector<VehicleSize>::iterator it = std::lower_bound(
        sizes.begin(), sizes.end(), tag, [](const VehicleSize& t, const std::string& name) { return t.tag < name; });
    if (it == sizes.end() || it->tag != tag) {
        VehicleSize t;
        t.tag = tag;
        t.turningRadius = turningRadius;
        t.width = width;
        t.length = length;
        sizes.insert(it, t);
        return;
    }
    it->turningRadius = std::max(it->turningRadius, turningRadius);
    it->width = std::max(it->width, width);
    it->length = std::max(it->length, length);
}

bool wheelbaseFromWheels(const std::vector<WheelSpot>& wheels, Real& wheelbase) {
    Real front = 0, back = 0;
    if (!middleOf(wheels, true, front) || !middleOf(wheels, false, back)) return false;
    wheelbase = std::fabs(front - back);
    return true;
}

Real tightestTurnRadius(Real wheelbase, Real maxSteerDegrees) {
    if (!(maxSteerDegrees > 0) || !(maxSteerDegrees < kRightAngleDegrees)) return 0;
    // (the arithmetic as it has always been written here, to the last bit: the radius is part of the road model's key)
    const Real pi = std::acos(Real(-1));
    constexpr Real kHalfTurnDegrees = 180.0;
    return wheelbase / std::tan(maxSteerDegrees * pi / kHalfTurnDegrees);
}

}  // namespace roads::lanes
}  // namespace engine
