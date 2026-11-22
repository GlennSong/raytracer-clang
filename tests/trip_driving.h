// A DRIVING TO PRICE DRIVES BY (trip_plan.h TripNetwork::driving): the driving over a test scene's road model with one
// vehicle class, a car tagged "car" (the tag a lane level opens its lanes to cars by), and no rules (a route is found
// without them). Its router prices a leg on the roads as the brain's planner does on a level. The scenes post no
// limits and there are no rules to give one, so the car's top speed is a local street's (classSpeed, 8 m/s): a drive
// on their local streets is timed at it, as the tests' hand-worked numbers take it.
#ifndef RT_TESTS_TRIP_DRIVING_H
#define RT_TESTS_TRIP_DRIVING_H

#include "../src/apps/driving/driving.h"
#include "../src/apps/driving/vehicle_class.h"
#include "../src/engine/procgen/city/road_network.h"   // classSpeed

#include <memory>

namespace citytest {

inline std::unique_ptr<driving::Driving> tripDriving(const engine::RoadModel& model) {
    std::unique_ptr<driving::Driving> d = std::make_unique<driving::Driving>(&model);
    driving::VehicleClasses classes;
    driving::VehicleClass car = driving::deriveVehicleClass("car", 4.5, 1.8, 2.7, 30);
    car.tags = {"car"};
    car.topSpeed = engine::classSpeed(engine::RoadClass::Local);
    classes.add(car);
    d->setVehicleClasses(classes);
    return d;
}

}  // namespace citytest

#endif
