#include "apps/driving/vehicle_class.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>

namespace driving {

namespace {

const Real kPi = std::acos(Real(-1));
constexpr Real kHalfTurnDegrees = 180.0;
constexpr Real kRightAngleDegrees = 90.0;
constexpr Real kHalf = 0.5;

Real radians(Real degrees) { return degrees * kPi / kHalfTurnDegrees; }

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

std::string number(Real v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.3f", v);
    return buf;
}

}  // namespace

bool VehicleClass::hasTag(const std::string& tag) const { return std::binary_search(tags.begin(), tags.end(), tag); }

bool wheelbaseFromWheels(const std::vector<WheelSpot>& wheels, Real& wheelbase) {
    Real front = 0, back = 0;
    if (!middleOf(wheels, true, front) || !middleOf(wheels, false, back)) return false;
    wheelbase = std::fabs(front - back);
    return true;
}

Real tightestTurnRadius(Real wheelbase, Real maxSteerDegrees) {
    if (!(maxSteerDegrees > 0) || !(maxSteerDegrees < kRightAngleDegrees)) return 0;
    return wheelbase / std::tan(radians(maxSteerDegrees));
}

VehicleClass deriveVehicleClass(const std::string& name, Real length, Real width, Real wheelbase,
                                Real maxSteerDegrees) {
    VehicleClass c;
    c.name = name;
    c.length = length;
    c.width = width;
    c.wheelbase = wheelbase;
    c.tightestTurnRadius = tightestTurnRadius(wheelbase, maxSteerDegrees);
    return c;
}

std::size_t VehicleClasses::add(VehicleClass c) {
    classes_.push_back(std::move(c));
    return classes_.size() - 1;
}

int VehicleClasses::indexOf(const std::string& name) const {
    for (std::size_t i = 0; i < classes_.size(); ++i)
        if (classes_[i].name == name) return static_cast<int>(i);
    return -1;
}

const VehicleClass* VehicleClasses::find(const std::string& name) const {
    const int i = indexOf(name);
    return i < 0 ? nullptr : &classes_[static_cast<std::size_t>(i)];
}

std::vector<TagSize> tagSizes(const VehicleClasses& classes) {
    std::vector<TagSize> out;
    for (const VehicleClass& c : classes.all())
        for (const std::string& tag : c.tags) {
            std::vector<TagSize>::iterator it = std::lower_bound(
                out.begin(), out.end(), tag, [](const TagSize& t, const std::string& name) { return t.tag < name; });
            if (it == out.end() || it->tag != tag) {
                TagSize t;
                t.tag = tag;
                t.turningRadius = c.tightestTurnRadius;
                t.width = c.width;
                t.length = c.length;
                out.insert(it, t);
                continue;
            }
            it->turningRadius = std::max(it->turningRadius, c.tightestTurnRadius);
            it->width = std::max(it->width, c.width);
            it->length = std::max(it->length, c.length);
        }
    return out;
}

std::vector<std::string> validate(const VehicleClass& c) {
    std::vector<std::string> out;
    const std::string who = c.name.empty() ? std::string("a vehicle class") : c.name;
    if (c.name.empty()) out.push_back("a vehicle class has no name");
    if (!(c.length > 0)) out.push_back(who + ": its length " + number(c.length) + " m is not positive");
    if (!(c.width > 0)) out.push_back(who + ": its width " + number(c.width) + " m is not positive");
    if (!(c.wheelbase > 0)) out.push_back(who + ": its wheelbase " + number(c.wheelbase) + " m is not positive");
    else if (c.wheelbase > c.length)
        out.push_back(who + ": its wheelbase " + number(c.wheelbase) + " m is longer than the vehicle (" +
                      number(c.length) + " m)");
    const Real tightest = tightestTurnRadius(c.wheelbase, kSteepestPossibleSteerDegrees);
    if (!(c.tightestTurnRadius > 0))
        out.push_back(who + ": its tightest turning radius " + number(c.tightestTurnRadius) + " m is not positive");
    else if (c.wheelbase > 0 && c.tightestTurnRadius < tightest)
        out.push_back(who + ": its tightest turning radius " + number(c.tightestTurnRadius) +
                      " m is tighter than any steering can give a " + number(c.wheelbase) + " m wheelbase (" +
                      number(tightest) + " m)");
    if (!(c.speedUp > 0)) out.push_back(who + ": its speeding-up " + number(c.speedUp) + " m/s2 is not positive");
    if (!(c.brake > 0)) out.push_back(who + ": its comfortable braking " + number(c.brake) + " m/s2 is not positive");
    if (!(c.hardestBrake >= c.brake))
        out.push_back(who + ": its hardest braking " + number(c.hardestBrake) + " m/s2 is weaker than its comfortable " +
                      "braking " + number(c.brake) + " m/s2");
    if (!(c.topSpeed > 0)) out.push_back(who + ": its top speed " + number(c.topSpeed) + " m/s is not positive");
    if (c.trailerLength < 0)
        out.push_back(who + ": its trailer's length " + number(c.trailerLength) + " m is negative");
    return out;
}

std::vector<std::string> validate(const VehicleClasses& table) {
    std::vector<std::string> out;
    for (std::size_t i = 0; i < table.size(); ++i) {
        const VehicleClass& c = table.at(i);
        std::vector<std::string> own = validate(c);
        out.insert(out.end(), own.begin(), own.end());
        for (std::size_t j = 0; j < i; ++j)
            if (!c.name.empty() && table.at(j).name == c.name) {
                out.push_back("two vehicle classes are named \"" + c.name + "\" (number " + std::to_string(j) +
                              " and number " + std::to_string(i) + ")");
                break;
            }
    }
    return out;
}

}  // namespace driving
