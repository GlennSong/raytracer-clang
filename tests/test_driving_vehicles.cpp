// The driving library's vehicle classes (#248): a class is derived from a body and its wheels, looked up by index and
// by name, and every kind of problem `validate` knows is reported; the `driving` block reader reads every field and
// refuses a bad block whole. Classes here are made up; the real fleet is checked in run_tests (test_vehicle_body.cpp),
// which can load vehicles.lua.

#include "test_framework.h"

#include "apps/driving/driving.h"
#include "apps/driving/vehicle_class.h"

#include <cmath>
#include <string>
#include <vector>

using namespace driving;

namespace {

constexpr Real kLength = 4.6;
constexpr Real kWidth = 1.8;
constexpr Real kWheelbase = 2.7;
constexpr Real kSteerDegrees = 30;
constexpr Real kFrontAxle = 1.35;
constexpr Real kBackAxle = -1.35;
constexpr Real kDegToRad = 3.14159265358979323846 / 180.0;

// A class that validates: an ordinary car.
VehicleClass goodCar(const std::string& name) {
    VehicleClass c = deriveVehicleClass(name, kLength, kWidth, kWheelbase, kSteerDegrees);
    c.tags = {"car"};
    return c;
}

// True when one of `problems` contains `words`.
bool says(const std::vector<std::string>& problems, const std::string& words) {
    for (const std::string& p : problems)
        if (p.find(words) != std::string::npos) return true;
    return false;
}

}  // namespace

TEST_CASE(driving_vehicle_class_is_derived_from_body_and_wheels) {
    // Four wheels at two axles: the wheelbase is axle to axle.
    std::vector<WheelSpot> wheels = {{kFrontAxle, true}, {kFrontAxle, true}, {kBackAxle, false}, {kBackAxle, false}};
    Real wb = 0;
    CHECK(wheelbaseFromWheels(wheels, wb));
    CHECK_APPROX(wb, kFrontAxle - kBackAxle, 1e-12);
    // A tandem's two back axles count as one, at their middle.
    const Real tandemA = -3.0, tandemB = -4.3;
    std::vector<WheelSpot> tandem = {{kFrontAxle, true}, {kFrontAxle, true}, {tandemA, false}, {tandemA, false},
                                     {tandemB, false}, {tandemB, false}};
    CHECK(wheelbaseFromWheels(tandem, wb));
    CHECK_APPROX(wb, kFrontAxle - 0.5 * (tandemA + tandemB), 1e-12);
    // No steered wheels (or no fixed ones): nothing to measure.
    std::vector<WheelSpot> noSteer = {{kFrontAxle, false}, {kBackAxle, false}};
    CHECK(!wheelbaseFromWheels(noSteer, wb));
    CHECK(!wheelbaseFromWheels(std::vector<WheelSpot>(), wb));

    // The tightest turn, at the middle of the back axle: wheelbase / tan(steer).
    CHECK_APPROX(tightestTurnRadius(kWheelbase, kSteerDegrees), kWheelbase / std::tan(kSteerDegrees * kDegToRad), 1e-12);
    CHECK(tightestTurnRadius(kWheelbase, 0) == 0);
    CHECK(tightestTurnRadius(kWheelbase, 90) == 0);

    const VehicleClass c = deriveVehicleClass("runabout", kLength, kWidth, kWheelbase, kSteerDegrees);
    CHECK(c.name == "runabout");
    CHECK(c.length == kLength && c.width == kWidth && c.wheelbase == kWheelbase);
    CHECK_APPROX(c.tightestTurnRadius, tightestTurnRadius(kWheelbase, kSteerDegrees), 1e-12);
    // The authored fields are at the named defaults until the data is read over them.
    CHECK(c.speedUp == kUnauthoredSpeedUp && c.brake == kUnauthoredBrake);
    CHECK(c.hardestBrake == kUnauthoredHardestBrake && c.topSpeed == kUnauthoredTopSpeed);
    CHECK(c.tags.empty() && !c.hasTrailer());
    CHECK(validate(c).empty());
}

TEST_CASE(driving_vehicle_classes_found_by_index_and_name) {
    VehicleClasses table;
    CHECK(table.empty());
    CHECK(table.add(goodCar("runabout")) == 0u);
    VehicleClass lorry = goodCar("lorry");
    lorry.tags = {"goods", "truck"};
    CHECK(table.add(lorry) == 1u);
    CHECK(table.size() == 2u);
    CHECK(table.at(0).name == "runabout" && table.at(1).name == "lorry");   // kept in the order added
    CHECK(table.indexOf("lorry") == 1);
    CHECK(table.indexOf("nothing") == -1);
    CHECK(table.find("nothing") == nullptr);
    const VehicleClass* found = table.find("lorry");
    CHECK(found != nullptr);
    if (found) {
        CHECK(found->hasTag("truck") && found->hasTag("goods") && !found->hasTag("car"));
    }
    CHECK(validate(table).empty());

    // The driving model holds the table it is given.
    Driving d;
    CHECK(d.vehicleClasses().empty());
    d.setVehicleClasses(table);
    CHECK(d.vehicleClasses().size() == 2u);
    CHECK(d.vehicleClasses().find("runabout") != nullptr);
}

// WHAT EACH TAG ASKS OF A CONNECTOR: the widest turning circle and the widest body among the classes carrying it, every
// tag once, sorted by name whatever order the classes came in.
TEST_CASE(driving_tag_sizes_take_the_most_demanding_class_per_tag) {
    VehicleClasses table;
    VehicleClass van = deriveVehicleClass("van", 5.5, 2.1, 3.4, kSteerDegrees);
    van.tags = {"goods", "van"};
    VehicleClass truck = deriveVehicleClass("truck", 8.0, 2.5, 4.8, kSteerDegrees);
    truck.tags = {"goods", "truck"};
    VehicleClass cab = goodCar("cab");
    cab.tags = {"car", "taxi"};
    table.add(van);
    table.add(truck);
    table.add(goodCar("runabout"));
    table.add(cab);
    const std::vector<TagSize> sizes = tagSizes(table);
    CHECK(sizes.size() == 5u);
    const char* order[] = {"car", "goods", "taxi", "truck", "van"};
    for (std::size_t i = 0; i < sizes.size() && i < 5; ++i) CHECK(sizes[i].tag == order[i]);
    if (sizes.size() != 5u) return;
    CHECK(sizes[0].turningRadius == goodCar("x").tightestTurnRadius && sizes[0].width == kWidth);
    // goods: the truck's circle and body, wider and longer than the van's (the length sweeps bends, ADR-0412)
    CHECK(sizes[1].turningRadius == truck.tightestTurnRadius && sizes[1].width == truck.width);
    CHECK(sizes[1].length == truck.length && sizes[1].length > van.length);
    CHECK(sizes[1].turningRadius > van.tightestTurnRadius);
    CHECK(sizes[4].turningRadius == van.tightestTurnRadius && sizes[4].width == van.width && sizes[4].length == van.length);
    // every class fits every one of its own tags
    for (const VehicleClass& c : table.all())
        for (const TagSize& t : sizes)
            if (c.hasTag(t.tag)) CHECK(c.tightestTurnRadius <= t.turningRadius && c.width <= t.width && c.length <= t.length);
    CHECK(tagSizes(VehicleClasses()).empty());
}

TEST_CASE(driving_vehicle_class_validate_names_each_problem) {
    {
        VehicleClass c = goodCar("");
        CHECK(says(validate(c), "has no name"));
    }
    {
        VehicleClass c = goodCar("flat");
        c.length = 0;
        CHECK(says(validate(c), "flat: its length 0.000 m is not positive"));
    }
    {
        VehicleClass c = goodCar("thin");
        c.width = -1;
        CHECK(says(validate(c), "thin: its width"));
    }
    {
        VehicleClass c = deriveVehicleClass("axleless", kLength, kWidth, 0, kSteerDegrees);
        CHECK(says(validate(c), "axleless: its wheelbase 0.000 m is not positive"));
    }
    {
        VehicleClass c = deriveVehicleClass("stretched", kLength, kWidth, kLength + 1, kSteerDegrees);
        CHECK(says(validate(c), "longer than the vehicle"));
    }
    {
        // Steering past the steepest possible: a turn tighter than any road vehicle with this wheelbase can make.
        VehicleClass c = deriveVehicleClass("spinner", kLength, kWidth, kWheelbase, 80);
        CHECK(says(validate(c), "spinner: its tightest turning radius"));
        CHECK(says(validate(c), "tighter than any steering can give"));
        // ...and exactly at the bound is still allowed.
        CHECK(validate(deriveVehicleClass("sharp", kLength, kWidth, kWheelbase, kSteepestPossibleSteerDegrees)).empty());
    }
    {
        VehicleClass c = goodCar("straight");
        c.tightestTurnRadius = 0;
        CHECK(says(validate(c), "straight: its tightest turning radius 0.000 m is not positive"));
    }
    {
        VehicleClass c = goodCar("soft");
        c.brake = 3;
        c.hardestBrake = 2;
        CHECK(says(validate(c), "soft: its hardest braking 2.000 m/s2 is weaker than its comfortable braking 3.000 m/s2"));
    }
    {
        VehicleClass c = goodCar("stuck");
        c.speedUp = 0;
        c.topSpeed = 0;
        c.brake = 0;
        const std::vector<std::string> p = validate(c);
        CHECK(says(p, "its speeding-up"));
        CHECK(says(p, "its top speed"));
        CHECK(says(p, "its comfortable braking"));
    }
    {
        VehicleClass c = goodCar("towing");
        c.trailerLength = -2;
        CHECK(says(validate(c), "its trailer's length"));
    }
    {
        VehicleClasses table;
        table.add(goodCar("twin"));
        table.add(goodCar("other"));
        table.add(goodCar("twin"));
        const std::vector<std::string> p = validate(table);
        CHECK(p.size() == 1u);
        CHECK(says(p, "two vehicle classes are named \"twin\" (number 0 and number 2)"));
    }
}

#ifdef RT_ENABLE_SCRIPTING

#include "apps/driving/scripting/vehicle_class_lua.h"
#include "engine/scripting/lua_state.h"
#include "engine/scripting/script_vm.h"

namespace {

// Runs `chunk` (which sets a global `block`) and reads that block into `out` over `start`.
bool readBlock(const std::string& chunk, VehicleClass& out, std::string& err) {
    engine::ScriptVM vm;
    std::string runErr;
    if (!vm.doString(chunk, &runErr)) {
        err = runErr;
        return false;
    }
    lua_State* L = engine::luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "block");
    const bool ok = readVehicleClassBlock(L, -1, "block", out, &err);
    lua_pop(L, 1);
    CHECK(lua_gettop(L) == base);
    return ok;
}

}  // namespace

TEST_CASE(driving_vehicle_block_reads_every_field) {
    VehicleClass c = goodCar("rig");
    std::string err;
    const bool ok = readBlock(R"LUA(block = { speed_up = 0.8, brake = 1.5, hardest_brake = 5.5, top_speed = 25,
                                    tags = { "truck", "goods", "truck" },
                                    trailer = { length = 13.6, hitch_ahead = 0.4 } })LUA",
                              c, err);
    if (!ok) std::printf("    %s\n", err.c_str());
    CHECK(ok);
    CHECK(c.speedUp == 0.8 && c.brake == 1.5 && c.hardestBrake == 5.5 && c.topSpeed == 25);
    CHECK(c.tags.size() == 2u && c.tags[0] == "goods" && c.tags[1] == "truck");   // sorted, repeats dropped
    CHECK(c.hasTrailer() && c.trailerLength == 13.6 && c.hitchAheadOfBackAxle == 0.4);
    // The sizes are not the block's: they stay as derived.
    CHECK(c.length == kLength && c.wheelbase == kWheelbase);
    CHECK(validate(c).empty());
}

TEST_CASE(driving_vehicle_block_refused_whole_with_every_fault) {
    VehicleClass c = goodCar("rig");
    const VehicleClass before = c;
    std::string err;
    CHECK(!readBlock(R"LUA(block = { speed_up = 0, brake = "hard", top_speed = 30, tags = { "car", 3 },
                                     breaks = 2, trailer = { length = 5 } })LUA",
                     c, err));
    std::printf("    %s\n", err.c_str());
    CHECK(err.find("block.speed_up is 0, expected a number above zero") != std::string::npos);
    CHECK(err.find("block.brake is missing or not a number") != std::string::npos);
    CHECK(err.find("block.hardest_brake is missing or not a number") != std::string::npos);
    CHECK(err.find("block.tags[2] is not a tag name") != std::string::npos);
    CHECK(err.find("block has the key \"breaks\"") != std::string::npos);
    CHECK(err.find("block.trailer.hitch_ahead is missing") != std::string::npos);
    // Untouched on a fault.
    CHECK(c.speedUp == before.speedUp && c.tags == before.tags && c.topSpeed == before.topSpeed);
    CHECK(!readBlock("block = 5", c, err));
    CHECK(err.find("block is missing or not a table") != std::string::npos);
}

#endif
