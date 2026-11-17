#include "city_near.h"

#include "../../engine/asset_manager.h"
#include "../../engine/components.h"
#include "../../engine/physics/physics_world.h"
#include "../../engine/systems/physics_system.h"
#include "../../engine/world.h"
#include "../../log.h"
#include "city_render.h"

#ifdef RT_ENABLE_SCRIPTING
#include "../../engine/script_assets.h"
#include "../../engine/scripting/procgen_bindings.h"
#include "../../engine/scripting/script_modules.h"
#include "../../engine/scripting/script_vm.h"
#include "../../engine/scripting/vehicle_spec.h"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace citysim {

using engine::Entity;
using engine::Real;
using engine::Vec2;
using engine::Vec3;

namespace {

constexpr Real kDegrees = 57.29577951308232;
const char* const kOwner = "CityNearSystem";
// pursuitCommand's own guards, off: the driving's driver already slows for the way's end and for the turns
constexpr Real kNoEndBrake = 1e9;
constexpr Real kNoTurnSlowdown = 0.0;

Vec2 flat(const Vec3& v) { return Vec2(v.x, v.z); }

// The centre line of `paths`, end to end (a point shared by two paths once).
std::vector<Vec2> lineOf(const engine::LaneGraph& roads, const std::vector<engine::PathId>& paths) {
    std::vector<Vec2> out;
    for (const engine::PathId p : paths) {
        if (p.index >= roads.paths.size()) continue;
        const engine::Path& P = roads.paths[p.index];
        for (uint32_t k = P.curve.first; k < P.curve.end(); ++k)
            if (out.empty() || (roads.points[k] - out.back()).length() > engine::kSamePoint) out.push_back(roads.points[k]);
    }
    return out;
}

}  // namespace

// ---- The controller ---------------------------------------------------------------------------------------------------

engine::DriverCommand nearCommand(CityNearCar& n, const engine::LaneGraph& roads, const driving::Driving& d, Vec2 middle,
                                  Vec2 forward, Real speed, Real dt) {
    engine::DriverCommand cmd;
    const driving::Car* car = d.car(n.car);
    const driving::CarIntent* want = d.intent(n.car);
    if (!car || !want) return cmd;   // (nothing decided yet: stand)

    // THE LINE: the path before its front's (its middle may still be on it), then its way from its front
    std::vector<engine::PathId> line;
    if (car->trailCount > 0) line.push_back(car->trail[0]);
    for (const engine::PathId p : d.wayFromFront(n.car)) line.push_back(p);
    if (line != n.line) {
        n.line = line;
        n.follower.setPath(lineOf(roads, line));
    }
    n.follower.update(middle);

    // THE SPEED: what the driver wants, and no more than a braking curve that ends where it means to be at rest, the
    // distance left corrected for how far it has gone since the driver last worked it out
    if (d.tickCount() != n.tickSeen) {
        n.tickSeen = d.tickCount();
        n.sinceTick = 0;
    }
    n.sinceTick += std::max(Real(0), speed) * dt;
    // (the speed the driver wants a lag ahead: its next tick's alone, from a standstill, reads as "stay stopped")
    Real wanted = std::max(Real(0), want->speed + want->acceleration * kNearLag);
    Real accel = want->acceleration;
    if (want->hasRest) {
        const Real left = want->restIn - n.sinceTick;
        const Real b = kStopCurveShare * n.brake;
        const Real curve = std::sqrt(2 * b * std::max(Real(0), left));
        if (curve < wanted) {
            wanted = curve;
            // the deceleration that ends at the place from here
            accel = left > engine::kSamePoint ? -speed * speed / (2 * left) : -n.brake;
        }
        if (left < kStopHoldWithin && speed < kStopHoldBelow) wanted = 0;
    }
    cmd = engine::pursuitCommand(n.follower, middle, forward, wanted, kNearLookMin + kNearLookGain * std::max(Real(0), speed),
                                 kNoEndBrake, kNoTurnSlowdown);
    // THE PEDAL ITS MASS AND ENGINE NEED for the acceleration wanted (what its engine and brakes were measured to do)
    if (accel > 0 && n.launch > 0) cmd.feedForward = std::min(Real(1), accel / n.launch);
    else if (accel < 0 && n.brake > 0) cmd.feedForward = -std::min(Real(1), -accel / n.brake);
    return cmd;
}

// ---- The spec a slot's car is built from ------------------------------------------------------------------------------

bool slotVehicleSpec(const CityRenderSystem& city, int slot, engine::VehicleSpec& spec, std::string* recipeOut,
                     std::string* why) {
#ifdef RT_ENABLE_SCRIPTING
    std::string recipe;
    Vec3 paint;
    if (!city.carDrivable(slot, recipe, paint)) {
        if (why) *why = "fleet slot " + std::to_string(slot) + " has no drivable recipe";
        return false;
    }
    const std::string lib = engine::loadScriptCode("vehicles.lua", "");
    engine::ScriptVM vm;
    engine::openProcgenLibrary(vm);
    engine::openModuleLoader(vm, engine::makeModuleSource(""));
    std::string err;
    char chunk[256];
    std::snprintf(chunk, sizeof chunk, "return vehicle.%s(seed, { color = { %.4f, %.4f, %.4f } })", recipe.c_str(), paint.x,
                  paint.y, paint.z);
    if (lib.empty() || !vm.doString(lib, &err) || !engine::loadVehicleSpec(vm, chunk, 1u, spec, &err)) {
        if (why) *why = "fleet slot " + std::to_string(slot) + "'s drivable " + recipe + ": " + (lib.empty() ? "no vehicles.lua" : err);
        return false;
    }
    if (recipeOut) *recipeOut = recipe;
    return true;
#else
    (void)city;
    (void)slot;
    (void)spec;
    (void)recipeOut;
    if (why) *why = "a build without scripting has no drivable recipes";
    return false;
#endif
}

// ---- The system -------------------------------------------------------------------------------------------------------

void CityNearSystem::onStart(engine::FrameContext& ctx) {
    world_ = &ctx.world;
    assets_ = &ctx.assets;
    city_.simMutable().setBeforeDriving([this](driving::Driving& d) { writePoses(d); });
    if (!ctx.commands) return;
    ctx.commands->add("near", "near route|queue|clear",
                      "The near tier's test bed in the viewer (ADR-0447): `route` puts a physics car at the start of the "
                      "test's route, `queue` a lane car with a physics car behind it; they set out when the route's light "
                      "turns red. `clear` takes the lab's cars away. `near?` says where they are.",
                      [this](const engine::ControlCommand& c) -> std::string {
                          if (!world_) return "err no world yet";
                          const std::string what = c.args.empty() ? std::string() : c.args[0];
                          if (what == "clear") {
                              clearLab(*world_);
                              return "ok the lab's cars are gone";
                          }
                          if (what != "route" && what != "queue") return "err usage: near route|queue|clear";
                          std::string why;
                          if (!stageLab(*world_, what == "queue", &why)) return "err " + why;
                          char buf[200];
                          std::snprintf(buf, sizeof buf, "ok staged on lane %u, round connector %u (tightest bend %.1f m) "
                                        "into lane %u; it sets out when the light turns red",
                                        stagedRoute_.approach.index, stagedRoute_.turn.index, stagedRoute_.turnRadius,
                                        stagedRoute_.exit.index);
                          return buf;
                      },
                      kOwner);
    ctx.commands->add("near?", "near?",
                      "The near tier's physics cars (ADR-0447): each one's car number, where it is, its speed, its front's "
                      "offset from its lane's centre, and the switches, poses written and time spent so far.",
                      [this](const engine::ControlCommand&) -> std::string {
                          if (!world_) return "err no world yet";
                          const driving::Driving* d = city_.sim().drivingV2();
                          std::string out = "ok";
                          char buf[256];
                          world_->each<CityNearCar, engine::Transform>([&](Entity, CityNearCar& n, engine::Transform& t) {
                              const driving::Car* c = d ? d->car(n.car) : nullptr;
                              std::snprintf(buf, sizeof buf, " | car %u at %.2f %.2f %.2f, %.2f m/s, offset %.2f m, path %u%s",
                                            n.car.index, t.position.x, t.position.y, t.position.z, c ? c->speed : 0.0,
                                            c ? c->offset : 0.0, c ? c->path.index : 0u, c && c->driven ? "" : ", arrived");
                              out += buf;
                          });
                          std::snprintf(buf, sizeof buf, " | into physics %u, into the lane %u, refused %u, poses written %llu, "
                                        "off the road %llu, %.3f ms a step",
                                        counts_.intoPhysics, counts_.intoLane, counts_.refused,
                                        static_cast<unsigned long long>(counts_.written),
                                        static_cast<unsigned long long>(counts_.offRoad),
                                        counts_.controlMs / static_cast<double>(std::max<uint64_t>(1, counts_.controlSteps)));
                          out += buf;
                          return out;
                      },
                      kOwner);
}

void CityNearSystem::onStop(engine::FrameContext& ctx) {
    city_.simMutable().setBeforeDriving(nullptr);
    if (ctx.commands) ctx.commands->removeOwnedBy(kOwner);
}

void CityNearSystem::writePoses(driving::Driving& d) {
    if (!world_) return;
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    engine::PhysicsWorld& pw = physics_.physicsWorld();
    bool any = false;
    world_->each<CityNearCar, engine::Vehicle>([&](Entity, CityNearCar& n, engine::Vehicle& v) {
        if (v.vehicleId == engine::PhysicsWorld::INVALID_VEHICLE) return;
        const driving::Car* c = d.car(n.car);
        if (!c || !c->driven) return;   // (arrived: it stands where it parked, on no trip to be written onto)
        any = true;
        const Vec3 at = pw.vehiclePosition(v.vehicleId);
        const Vec3 fwd3 = pw.vehicleOrientation(v.vehicleId).rotate(Vec3(0, 0, 1));
        const Vec2 forward = engine::normalize(flat(fwd3));
        const Vec3 vel = pw.vehicleVelocity(v.vehicleId);
        const Real speed = vel.x * fwd3.x + vel.y * fwd3.y + vel.z * fwd3.z;
        if (d.carriedAt(n.car, flat(at) + forward * n.halfLength, speed)) ++counts_.written;
        else ++counts_.offRoad;
    });
    if (any) counts_.controlMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void CityNearSystem::fixedUpdate(engine::FrameContext& ctx) {
    world_ = &ctx.world;
    assets_ = &ctx.assets;
    const driving::Driving* d = city_.sim().drivingV2();
    if (!d || !d->roads()) return;
    if (!staged_.empty()) sendStaged(ctx.world, ctx.clock.fixedStep());
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    engine::PhysicsWorld& pw = physics_.physicsWorld();
    bool any = false;
    ctx.world.each<CityNearCar, engine::Vehicle, engine::AgentDriver>(
        [&](Entity, CityNearCar& n, engine::Vehicle& v, engine::AgentDriver& ad) {
            if (v.vehicleId == engine::PhysicsWorld::INVALID_VEHICLE) return;
            any = true;
            const Vec3 at = pw.vehiclePosition(v.vehicleId);
            const Vec3 fwd3 = pw.vehicleOrientation(v.vehicleId).rotate(Vec3(0, 0, 1));
            const Vec3 vel = pw.vehicleVelocity(v.vehicleId);
            const Real speed = vel.x * fwd3.x + vel.y * fwd3.y + vel.z * fwd3.z;
            ad.command = nearCommand(n, *d->roads(), *d, flat(at), engine::normalize(flat(fwd3)), speed,
                                     ctx.clock.fixedStep());
        });
    if (!any) return;
    counts_.controlMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    ++counts_.controlSteps;
}

bool CityNearSystem::toPhysics(engine::World& world, engine::AssetManager& assets, Entity e, Real step, std::string* why) {
    const auto refuse = [&](const std::string& reason) {
        ++counts_.refused;
        if (why) *why = reason;
        return false;
    };
    driving::Driving* d = city_.simMutable().drivingV2Mutable();
    const CityCarBody* body = world.get<CityCarBody>(e);
    const CityCarOnLane* lane = world.get<CityCarOnLane>(e);
    const engine::Transform* t = world.get<engine::Transform>(e);
    if (!d || !body || !lane || !t) return refuse("not a car the driving has on its lanes");
    if (world.has<engine::Vehicle>(e)) return refuse("it is a physics car already");
    const driving::Car* c = d->car(lane->car);
    if (!c || !c->driven) return refuse("the driving has it on no trip");
    if (c->vehicleClass >= d->vehicleClasses().size()) return refuse("its vehicle class is unknown");
    engine::VehicleSpec spec;
    std::string err;
    if (!slotVehicleSpec(city_, body->slot, spec, nullptr, &err)) return refuse(err);
    std::map<int, SlotCar>::iterator measured = slotCars_.find(body->slot);
    if (measured == slotCars_.end()) {
        SlotCar s;
        s.restHeight = engine::PhysicsWorld::restHeight(spec.config);
        s.launch = engine::PhysicsWorld::launchAccel(spec.config);
        s.brake = engine::PhysicsWorld::brakeDecel(spec.config);
        measured = slotCars_.emplace(body->slot, s).first;
    }
    if (!(measured->second.restHeight > 0)) return refuse("its Jolt car could not be stood on its springs");

    // WHERE IT IS DRAWN: its middle and heading, on the lane graph's height under it, standing on its springs; a step
    // of its speed on, where the lane car would be when physics first moves it
    const Vec2 heading = carHeading(*t);
    const Real speed = c->speed;
    const Vec2 middle = carPosition(*t) + heading * (speed * step);
    const Vec3 at(middle.x, lane->roadY + measured->second.restHeight, middle.y);
    engine::makeVehicle(world, assets, spec, e, at, std::atan2(heading.x, heading.y) * kDegrees);
    engine::Vehicle* v = world.get<engine::Vehicle>(e);
    if (!v) return refuse("the engine made no Vehicle");
    v->startMoving = true;
    v->startVelocity = Vec3(heading.x * speed, 0, heading.y * speed);
    engine::AgentDriver ad;
    ad.assistSteer = true;
    ad.command = engine::DriverCommand::toward(middle, heading, speed);
    world.add<engine::AgentDriver>(e, ad);
    CityNearCar n;
    n.car = lane->car;
    n.halfLength = d->vehicleClasses().at(c->vehicleClass).length * 0.5;
    n.launch = measured->second.launch;
    n.brake = measured->second.brake;
    const driving::CarId id = lane->car;
    world.remove<CityCarOnLane>(e);   // (the driving no longer draws it: physics does)
    world.add<CityNearCar>(e, n);
    d->carry(id, true, kNearLag);
    ++counts_.intoPhysics;
    return true;
}

bool CityNearSystem::toLane(engine::World& world, Entity e, std::string* why) {
    const auto refuse = [&](const std::string& reason) {
        ++counts_.refused;
        if (why) *why = reason;
        return false;
    };
    driving::Driving* d = city_.simMutable().drivingV2Mutable();
    const CityNearCar* n = world.get<CityNearCar>(e);
    const engine::Vehicle* v = world.get<engine::Vehicle>(e);
    if (!d || !n || !v || v->vehicleId == engine::PhysicsWorld::INVALID_VEHICLE) return refuse("not a physics car of the near tier");
    const driving::Car* c = d->car(n->car);
    if (!c || !d->carried(n->car)) return refuse("the driving does not have it carried");
    if (v->engineDead) return refuse("it is a wreck");
    engine::PhysicsWorld& pw = physics_.physicsWorld();
    const engine::Quat q = pw.vehicleOrientation(v->vehicleId);
    const Vec3 up = q.rotate(Vec3(0, 1, 0));
    if (up.y < std::cos(kBackUprightDegrees / kDegrees)) return refuse("it is not upright");
    if (std::fabs(c->offset) > kBackOffset) return refuse("its front is " + std::to_string(c->offset) + " m off its lane's centre");
    const driving::CarPose pose = driving::carPose(*d->roads(), d->vehicleClasses(), d->ruleSet(), *c);
    const Vec2 lanes = engine::normalize(Vec2(pose.travel.x, pose.travel.z));
    const Vec2 forward = engine::normalize(flat(q.rotate(Vec3(0, 0, 1))));
    const Real align = std::acos(std::clamp(lanes.x * forward.x + lanes.y * forward.y, Real(-1), Real(1))) * kDegrees;
    if (align > kBackAlignDegrees) return refuse("it points " + std::to_string(align) + " degrees off its lane");

    // ONTO THE LANE: at the nearest point along it (no offset), keeping the speed it was last written with
    const driving::CarId id = n->car;
    d->carry(id, false);
    engine::VehicleRetire retire;
    retire.dropRenderable = true;   // (the city draws it among its cars again)
    world.add<engine::VehicleRetire>(e, retire);
    world.remove<CityNearCar>(e);
    CityCarOnLane lane;
    lane.car = id;
    lane.speed = c->speed;
    lane.driven = c->driven;
    world.add<CityCarOnLane>(e, lane);
    ++counts_.intoLane;
    return true;
}

bool CityNearSystem::stageLab(engine::World& world, bool queue, std::string* why) {
    const driving::Driving* d = city_.sim().drivingV2();
    if (!d) {
        if (why) *why = "the level has no new driving";
        return false;
    }
    clearLab(world);
    const std::vector<LabRoute> routes = labRoutes(*d, kLabRunUp, kLabRunOn);
    if (routes.empty()) {
        if (why) *why = "no lane into a light is long enough for the test's route";
        return false;
    }
    stagedRoute_ = routes.front();
    int slot = -1;
    uint32_t cls = 0;
    world.each<CityCarBody>([&](Entity, CityCarBody& b) {
        if (slot < 0) {
            slot = b.slot;
            cls = b.vehicleClass;
        }
    });
    if (slot < 0) slot = 0;
    driving::PathPoint first = stagedRoute_.start, firstTo = stagedRoute_.to;
    if (queue) first.along += kLabQueueBehind;
    const Entity lead = labCar(world, nextLabId_++, slot, cls, first, driving::PathPoint(), why);
    if (!lead.valid()) return false;
    lab_.push_back(lead);
    staged_.push_back(Staged{lead, firstTo, !queue});
    if (queue) {
        driving::PathPoint followTo = stagedRoute_.to;
        followTo.along -= kLabQueueBehind;
        const Entity follow = labCar(world, nextLabId_++, slot, cls, stagedRoute_.start, driving::PathPoint(), why);
        if (!follow.valid()) return false;
        lab_.push_back(follow);
        staged_.push_back(Staged{follow, followTo, true});
    }
    lastLamp_ = d->lamp(stagedRoute_.turn);
    return true;
}

void CityNearSystem::sendStaged(engine::World& world, Real step) {
    driving::Driving* d = city_.simMutable().drivingV2Mutable();
    if (!d || !assets_) return;
    const driving::Lamp now = d->lamp(stagedRoute_.turn);
    const bool turnedRed = now == driving::Lamp::Red && lastLamp_ != driving::Lamp::Red;
    lastLamp_ = now;
    if (!turnedRed) return;
    for (const Staged& s : staged_) {
        CityCarOnLane* lane = world.get<CityCarOnLane>(s.entity);
        if (!lane || !d->beginTrip(lane->car, kLabPerson, s.to).valid()) continue;
        lane->driven = true;
        std::string why;
        if (s.physics && !toPhysics(world, *assets_, s.entity, step, &why)) LOG_WARN << "[near] into physics: " << why;
    }
    staged_.clear();
}

void CityNearSystem::clearLab(engine::World& world) {
    driving::Driving* d = city_.simMutable().drivingV2Mutable();
    for (const Entity e : lab_) {
        if (!world.alive(e)) continue;
        const CityCarBody* b = world.get<CityCarBody>(e);
        if (d && b) {
            const driving::CarId car(b->id);
            const driving::TripId trip = d->tripOf(car);
            if (trip.valid()) d->endTrip(trip);
            d->carry(car, false);
            d->removeCar(car);
        }
        if (world.has<engine::Vehicle>(e)) {
            // (VehicleSystem takes the Jolt car away at its next step; the entity is left bare)
            world.add<engine::VehicleRetire>(e, engine::VehicleRetire{true});
            world.remove<CityCarBody>(e);
            world.remove<CityNearCar>(e);
            continue;
        }
        world.destroy(e);
    }
    lab_.clear();
    staged_.clear();
}

std::vector<LabRoute> labRoutes(const driving::Driving& d, Real runUp, Real runOn) {
    std::vector<LabRoute> out;
    const engine::LaneGraph* roads = d.roads();
    if (!roads || !d.lightsRun()) return out;
    for (uint32_t p = 0; p < roads->paths.size(); ++p) {
        const engine::Path& P = roads->paths[p];
        if (P.kind != engine::PathKind::Connector || !P.from.valid() || P.next.count == 0) continue;
        if (d.lamp(engine::PathId(p)) == driving::Lamp::Dark) continue;
        // (a turn its light holds at red some time in the cycle: a turn allowed on red never makes a car stop at the line)
        bool red = false;
        for (Real t = 0; t < d.lightTiming().cycle && !red; t += 1)
            red = driving::lampAt(*roads, d.rightOfWay(), engine::PathId(p), t) == driving::Lamp::Red;
        if (!red) continue;
        const engine::PathId exit = roads->nextPaths[P.next.first];
        if (P.from.index >= roads->paths.size() || exit.index >= roads->paths.size()) continue;
        if (roads->paths[P.from.index].length < runUp + engine::kSamePoint || roads->paths[exit.index].length < runOn) continue;
        LabRoute r;
        r.approach = P.from;
        r.turn = engine::PathId(p);
        r.exit = exit;
        r.turnRadius = P.tightestRadius;
        r.start.path = P.from;
        r.start.along = roads->paths[P.from.index].length - runUp;
        r.to.path = exit;
        r.to.along = runOn;
        out.push_back(r);
    }
    std::sort(out.begin(), out.end(), [](const LabRoute& a, const LabRoute& b) {
        return a.turnRadius != b.turnRadius ? a.turnRadius < b.turnRadius : a.turn.index < b.turn.index;
    });
    return out;
}

Entity CityNearSystem::labCar(engine::World& world, uint32_t id, int slot, uint32_t cls, const driving::PathPoint& front,
                              const driving::PathPoint& to, std::string* why) {
    driving::Driving* d = city_.simMutable().drivingV2Mutable();
    if (!d || cls >= d->vehicleClasses().size()) {
        if (why) *why = "the level has no new driving, or no such vehicle class";
        return Entity{};
    }
    const VehicleBody& fleet = city_.sim().fleetBody(slot);
    const driving::CarId car(id);
    if (!d->placeCar(car, cls, front, {}, why)) return Entity{};
    if (to.path.valid() && !d->beginTrip(car, kLabPerson, to).valid()) {
        d->removeCar(car);
        if (why) *why = "no way from there to the place it was sent to";
        return Entity{};
    }
    CityCarBody body;
    body.id = id;
    body.slot = slot;
    body.type = fleet.type;
    body.length = fleet.length;
    body.width = fleet.width;
    body.height = fleet.height;
    body.vehicleClass = cls;
    const driving::CarPose pose = driving::carPose(*d->roads(), d->vehicleClasses(), d->ruleSet(), *d->car(car));
    const Vec3 mid = (pose.front + pose.rear) * 0.5;
    engine::Transform where;
    setCarPose(where, Vec2(mid.x, mid.z), Vec2(pose.body.x, pose.body.z), mid.y);
    const Entity e = world.create();
    world.add<engine::Transform>(e, where);
    world.add<engine::PrevTransform>(e, engine::PrevTransform{where});
    world.add<CityCarBody>(e, body);
    world.add<CityCarLink>(e, CityCarLink{});
    CityCarOnLane lane;
    lane.car = car;
    lane.roadYFrom = lane.roadY = mid.y;
    lane.driven = to.path.valid();
    world.add<CityCarOnLane>(e, lane);
    return e;
}

}  // namespace citysim
