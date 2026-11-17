#include "city_car.h"

#include <cmath>

namespace citysim {

Vec2 carPosition(const engine::Transform& t) { return Vec2(t.position.x, t.position.z); }

Vec2 carHeading(const engine::Transform& t) {
    const engine::Vec3 forward = t.orientation.rotate(engine::Vec3(0, 0, 1));
    return Vec2(forward.x, forward.z);
}

void setCarPose(engine::Transform& t, Vec2 pos, Vec2 heading, Real y) {
    t.position = engine::Vec3(pos.x, y, pos.y);
    if (heading.length() > engine::kSamePoint)
        t.orientation = engine::Quat::fromAxisAngle(engine::Vec3(0, 1, 0), std::atan2(heading.x, heading.y));
}

void carTickBegins(engine::World& world) {
    world.each<CityCarBody, engine::Transform, engine::PrevTransform>(
        [](engine::Entity, CityCarBody&, engine::Transform& t, engine::PrevTransform& prev) { prev.value = t; });
}

}  // namespace citysim
