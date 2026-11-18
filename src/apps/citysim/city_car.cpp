#include "city_car.h"

#include "city_brain.h"   // CityBrain::takenByPlayer, carLeftAt
#include "city_sim.h"

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
        [&](engine::Entity e, CityCarBody&, engine::Transform& t, engine::PrevTransform& prev) {
            if (!world.has<engine::Vehicle>(e)) prev.value = t;
        });
}

bool carNowhere(engine::World& world, engine::Entity e) {
    return world.has<CityCarRoams>(e) && !world.has<CityCarOnLane>(e) && !world.has<engine::Vehicle>(e);
}

// ---- The player takes a car, and gives it back (ADR-0455): one way each, whoever's the car is ----------------------

bool CitySim::playerTakesCar(engine::World& world, engine::Entity car) {
    const CityCarBody* body = world.get<CityCarBody>(car);
    const CityCarLink* link = world.get<CityCarLink>(car);
    if (!body || !link || link->away) return false;
    // (each owner says yes or no and sets CityCarLink::away when it lets the car go: the one record of it)
    if (link->household >= 0) return brain_ && brain_->takenByPlayer(static_cast<int>(body->id));
    if (world.has<CityCarRoams>(car)) return drivingV2_ && drivingHost_.takenByPlayer(*drivingV2_, car);
    return false;
}

bool CitySim::playerLeftCar(engine::World& world, engine::Entity car) {
    const CityCarBody* body = world.get<CityCarBody>(car);
    const CityCarLink* link = world.get<CityCarLink>(car);
    const engine::Transform* t = world.get<engine::Transform>(car);
    if (!body || !link || !t || !link->away || link->household < 0 || !brain_) return false;
    return brain_->carLeftAt(static_cast<int>(body->id), carPosition(*t), carHeading(*t));
}

engine::Entity carEntityNumbered(engine::World& world, uint32_t id) {
    engine::Entity found;
    world.each<CityCarBody>([&](engine::Entity e, CityCarBody& body) {
        if (body.id == id) found = e;
    });
    return found;
}

void carsNear(const CitySim& sim, engine::World& world, Vec2 p, Real radius, std::vector<CarSeen>& out) {
    const Real r2 = radius * radius;
    const auto within = [&](Vec2 at) { return radius < 0 || (at - p).lengthSquared() <= r2; };
    const std::vector<Agent>& agents = sim.agents();
    const std::vector<SimVehicle>& vehicles = sim.vehicles();
    for (const int ni : sim.nearAgents()) {
        const Agent& a = agents[static_cast<std::size_t>(ni)];
        if (a.mode != Agent::Mode::Driver || a.released || a.far() || !within(a.pos)) continue;
        CarSeen c;
        c.pos = a.pos;
        c.heading = a.heading;
        c.speed = a.speed;
        c.moving = a.moving;
        if (a.vehicle >= 0 && a.vehicle < static_cast<int>(vehicles.size())) {
            c.length = vehicles[static_cast<std::size_t>(a.vehicle)].length;
            c.width = vehicles[static_cast<std::size_t>(a.vehicle)].width;
        }
        c.agent = ni;
        c.driver = ni;
        out.push_back(c);
    }
    world.each<CityCarBody, CityCarLink, engine::Transform>(
        [&](engine::Entity e, CityCarBody& body, CityCarLink& link, engine::Transform& t) {
            if (link.away || world.has<engine::Vehicle>(e) || carNowhere(world, e)) return;
            const Vec2 at = carPosition(t);
            if (!within(at)) return;
            CarSeen c;
            c.pos = at;
            c.heading = carHeading(t);
            const CityCarOnLane* lane = world.get<CityCarOnLane>(e);
            c.onRoad = lane != nullptr;
            if (lane) {
                c.speed = lane->speed;
                c.moving = lane->driven;
            }
            c.length = body.length;
            c.width = body.width;
            c.entity = e;
            c.id = body.id;
            c.driver = link.driver;
            out.push_back(c);
        });
}

}  // namespace citysim
