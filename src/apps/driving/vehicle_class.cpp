#include "apps/driving/vehicle_class.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>

namespace driving {

namespace {

std::string number(Real v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.3f", v);
    return buf;
}

}  // namespace

bool VehicleClass::hasTag(const std::string& tag) const { return std::binary_search(tags.begin(), tags.end(), tag); }

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
        for (const std::string& tag : c.tags)
            engine::roads::lanes::addVehicleSize(out, tag, c.tightestTurnRadius, c.width, c.length);
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
