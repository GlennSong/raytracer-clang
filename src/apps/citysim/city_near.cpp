#include "city_near.h"

#include "../../engine/asset_manager.h"
#include "../../engine/components.h"
#include "../../engine/physics/physics_world.h"
#include "../../engine/systems/physics_system.h"
#include "../../engine/world.h"
#include "../../engine/audio/audio_engine.h"
#include "../../engine/audio/sfx.h"   // horn: the honk of a near car (ADR-0451)
#include "../../log.h"
#include "city_brain.h"
#include "city_physics.h"
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

// The centre line of `paths`, end to end (a point shared by two paths once), moved `offset` to the side (+ toward the
// kerb side, the driver's left: the left of a heading (x, z) is (z, -x)) where a driver goes round something.
std::vector<Vec2> lineOf(const engine::LaneGraph& roads, const std::vector<engine::PathId>& paths, Real offset = 0) {
    std::vector<Vec2> out;
    for (const engine::PathId p : paths) {
        if (p.index >= roads.paths.size()) continue;
        const engine::Path& P = roads.paths[p.index];
        for (uint32_t k = P.curve.first; k < P.curve.end(); ++k)
            if (out.empty() || (roads.points[k] - out.back()).length() > engine::kSamePoint) out.push_back(roads.points[k]);
    }
    if (offset != 0 && out.size() >= 2) {
        std::vector<Vec2> moved(out.size());
        for (std::size_t k = 0; k < out.size(); ++k) {
            const Vec2 dir = engine::normalize(k + 1 < out.size() ? out[k + 1] - out[k] : out[k] - out[k - 1]);
            moved[k] = out[k] + Vec2(dir.y, -dir.x) * offset;
        }
        return moved;
    }
    return out;
}

// WHAT IS IN A NEAR CAR'S WAY, AS PHYSICS SEES IT (ADR-0451): a box as wide as the car (its middle's height, thin along
// the way) swept along `line` from `from` metres (its front bumper) for `reach` metres, moved `offset` to the side, in
// steps of kSweepStep; the first thing met (PhysicsWorld::sweepBox: anything that moves, and characters; never the
// fixed world). `distance` is along the line from `from`.
constexpr Real kSweepStep = 4.0;          // m: the line is swept as straight pieces this long
constexpr Real kSweepHalfHeight = 0.6;    // m: half the height of the swept box (a person's knees to a car's roof)
constexpr Real kSweepHalfDepth = 0.05;    // m: half its depth along the way (the sweep's motion covers the rest)
// HOW FAR A NEAR CAR LOOKS: as far as it takes to stop from its speed (its own measured braking) after a reaction, and
// stand off, and a little more; never less than kSeeLeast
constexpr Real kSeeLeast = 35.0;          // m (at a line: past a junction box and a car length beyond it)
constexpr Real kSeeLeastBrake = 1.0;      // m/s2 (a car not yet measured)
constexpr Real kSeeReaction = 1.0;        // s
constexpr Real kSeeBeyond = 5.0;          // m
// LOOKING FOR ROOM TO GO ROUND: only once nearly stopped behind it; offsets tried in steps of kPassStep, up to
// kPassTries of them each way; past a thing up to kPassThingLength long; looking kPassOncomingLook further where the
// offset takes it over into the oncoming lane
constexpr Real kLookForRoomBelow = 2.0;   // m/s
constexpr Real kPassStep = 0.5;           // m
constexpr int kPassTries = 12;
constexpr Real kPassThingLength = 4.0;    // m
constexpr Real kPassOncomingLook = 30.0;  // m
// SOMETHING COMING INTO MY WAY (as a lane car judges it: things.h kComingWiden, kComingMargin): the wider look covers
// kComingLook seconds of my road
constexpr Real kComingLook = 3.0;         // s
using driving::kComingWiden;
using driving::kComingMargin;
using driving::kComingSlowest;
// A character met (it has no body): its number as a thing
constexpr uint32_t kCharacterThing = 0xFFFF0000u;
bool sweepAlong(const engine::PhysicsWorld& pw, const std::vector<Vec2>& pts, const std::vector<Real>& cum, Real from,
                Real reach, Real offset, Real halfWidth, Real height, const std::vector<engine::PhysicsBodyId>& ignore,
                Real& distance, engine::PhysicsWorld::SweepHit& hit, Vec2& dirAt, int* casts) {
    if (pts.size() < 2 || cum.size() != pts.size()) return false;
    const Real end = std::min(cum.back(), from + reach);
    for (Real s = from; s < end; s += kSweepStep) {
        const Real s1 = std::min(end, s + kSweepStep);
        const Vec2 a = engine::pointAtStation(pts.data(), cum.data(), pts.size(), s);
        const Vec2 b = engine::pointAtStation(pts.data(), cum.data(), pts.size(), s1);
        const Vec2 ab = b - a;
        const Real len = ab.length();
        if (!(len > engine::kSamePoint)) continue;
        const Vec2 dir = ab * (1 / len);
        const Vec2 side = Vec2(dir.y, -dir.x) * offset;
        const Vec3 start(a.x + side.x, height, a.y + side.y);
        const engine::Quat turn = engine::Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(dir.x, dir.y));
        if (casts) ++*casts;
        if (pw.sweepBox(Vec3(halfWidth, kSweepHalfHeight, kSweepHalfDepth), start, turn, Vec3(ab.x, 0, ab.y), ignore, hit)) {
            distance = s - from + hit.fraction * len;
            dirAt = dir;
            return true;
        }
    }
    return false;
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
    const Real offset = d.passOffset(n.car);   // (going round something: ADR-0451)
    if (line != n.line || offset != n.lineOffset) {
        n.line = line;
        n.lineOffset = offset;
        n.follower.setPath(lineOf(roads, line, offset));
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

std::unique_ptr<engine::ScriptVM> openVehiclesLibrary(std::string* why) {
#ifdef RT_ENABLE_SCRIPTING
    std::unique_ptr<engine::ScriptVM> vm = std::make_unique<engine::ScriptVM>();
    engine::openProcgenLibrary(*vm);
    engine::openModuleLoader(*vm, engine::makeModuleSource(""));
    const std::string lib = engine::loadScriptCode("vehicles.lua", "");
    std::string err;
    if (lib.empty() || !vm->doString(lib, &err)) {
        if (why) *why = lib.empty() ? "no vehicles.lua" : "vehicles.lua: " + err;
        return nullptr;
    }
    return vm;
#else
    if (why) *why = "a build without scripting has no vehicles.lua";
    return nullptr;
#endif
}

bool slotVehicleSpec(const CityRenderSystem& city, int slot, engine::VehicleSpec& spec, std::string* recipeOut,
                     std::string* why, engine::ScriptVM* vehicles) {
#ifdef RT_ENABLE_SCRIPTING
    std::string recipe;
    Vec3 paint;
    if (!city.carDrivable(slot, recipe, paint)) {
        if (why) *why = "fleet slot " + std::to_string(slot) + " has no drivable recipe";
        return false;
    }
    std::unique_ptr<engine::ScriptVM> own;
    if (!vehicles) {
        own = openVehiclesLibrary(why);
        if (!own) return false;
        vehicles = own.get();
    }
    std::string err;
    char chunk[256];
    std::snprintf(chunk, sizeof chunk, "return vehicle.%s(seed, { color = { %.4f, %.4f, %.4f } })", recipe.c_str(), paint.x,
                  paint.y, paint.z);
    if (!engine::loadVehicleSpec(*vehicles, chunk, 1u, spec, &err)) {
        if (why) *why = "fleet slot " + std::to_string(slot) + "'s drivable " + recipe + ": " + err;
        return false;
    }
    if (recipeOut) *recipeOut = recipe;
    return true;
#else
    (void)city;
    (void)slot;
    (void)spec;
    (void)recipeOut;
    (void)vehicles;
    if (why) *why = "a build without scripting has no drivable recipes";
    return false;
#endif
}

// ---- The system -------------------------------------------------------------------------------------------------------

void CityNearSystem::onStart(engine::FrameContext& ctx) {
    world_ = &ctx.world;
    assets_ = &ctx.assets;
    city_.simMutable().setBeforeDriving([this](driving::Driving& d) { writePoses(d); });
    // TOUCHES BETWEEN TWO OF THE CITY'S JOLT CARS (ADR-0448): counted, the first few kept
    onCollision_ = ctx.events.listen<engine::Collision>([this](const engine::Collision& col) {
        if (col.persisted || !world_) return;
        uint32_t a = 0, b = 0;
        int found = 0;
        bool aIsCar = false;
        engine::PhysicsWorld& pw = physics_.physicsWorld();
        world_->each<CityCarBody, engine::Vehicle>([&](Entity, CityCarBody& body, engine::Vehicle& v) {
            if (v.vehicleId == engine::PhysicsWorld::INVALID_VEHICLE) return;
            const engine::PhysicsBodyId id = pw.vehicleBody(v.vehicleId);
            if (id == col.bodyA) {
                a = body.id;
                aIsCar = true;
                ++found;
            } else if (id == col.bodyB) {
                b = body.id;
                ++found;
            }
        });
        if (found == 0) return;
        if (found == 1) {
            // ONE city Jolt car's chassis against something else (a drawn car's kinematic box, a pole, the player, a
            // kerb): its wheels meet the road by ray, so a chassis touch is a hit
            ++counts_.otherHits;
            NearContact hit{aIsCar ? a : b, 0, col.position, col.speed};
            const engine::PhysicsBodyId other = aIsCar ? col.bodyB : col.bodyA;
            hit.boxAt = pw.bodyPosition(other);
            hit.otherEntity = aIsCar ? col.b : col.a;
            if (boxes_)
                for (const CityPhysicsSystem::ProxyBox& box : boxes_->carProxyBoxes())
                    if (box.id == other) {
                        hit.carBox = true;
                        hit.boxKey = box.key;
                        hit.boxHalf = box.he;
                    }
            // (a drawn car's box keyed below zero has nobody at its wheel: a parked car, ADR-0450)
            if (hit.carBox && hit.boxKey < 0) ++counts_.parkedHits;
            else if (hit.carBox) ++counts_.drivenBoxHits;
            else if (boxes_ && boxes_->isPedestrianBox(other)) ++counts_.pedestrianHits;
            if (counts_.otherHitList.size() < kContactsKept) counts_.otherHitList.push_back(hit);
            return;
        }
        ++counts_.contacts;
        if (counts_.contactList.size() < kContactsKept) counts_.contactList.push_back(NearContact{a, b, col.position, col.speed});
    });
    if (!ctx.commands) return;
    ctx.commands->add("near", "near route|queue|clear|marks [off]|key",
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
                          if (what == "key") {
                              pressEnter_ = true;
                              return std::string("ok the key to get in or out, pressed on the next frame (VehicleSystem's "
                                                 "own: the nearest car within its reach, no looking at it needed)");
                          }
                          if (what == "ahead" && c.args.size() == 3) {
                              // (a place on a near car's line, so many metres ahead of its front: where the probe puts
                              // the player in its way, ADR-0451)
                              const uint32_t want = static_cast<uint32_t>(std::atoi(c.args[1].c_str()));
                              const Real metres = static_cast<Real>(std::atof(c.args[2].c_str()));
                              std::string out = "err no near car " + c.args[1] + " with a line";
                              world_->each<CityNearCar>([&](Entity, CityNearCar& n) {
                                  if (n.car.index != want || !n.follower.valid()) return;
                                  const std::vector<Vec2>& pts = n.follower.points();
                                  const Vec2 p = engine::pointAtStation(pts.data(), n.follower.arcs().data(), pts.size(),
                                                                        n.follower.station() + n.halfLength + metres);
                                  char b[96];
                                  std::snprintf(b, sizeof b, "ok %.2f %.2f", p.x, p.y);
                                  out = b;
                              });
                              return out;
                          }
                          if (what == "marks") {
                              marks_ = c.args.size() < 2 || c.args[1] != "off";
                              return marks_ ? "ok a cyan post over each car of the near tier" : "ok no posts";
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
    // WHAT IS IN DRIVERS' WAYS (ADR-0451): every driven car whose driver sees something in its way, nearest first by
    // how far ahead it is: lane or near, where the car is, its speed, and what it sees
    ctx.commands->add("things?", "things?",
                      "Every driven car that sees something in its way (ADR-0451): `lane` or `near`, its number, where it "
                      "is, its speed, how far ahead the thing is and how long it has stood; the horns, swerves and other "
                      "ways so far.",
                      [this](const engine::ControlCommand&) -> std::string {
                          if (!world_) return "err no world yet";
                          const driving::Driving* d = city_.sim().drivingV2();
                          if (!d || !d->roads()) return "err no driving";
                          std::string out = "ok";
                          char buf[256];
                          for (const driving::Car& c : d->cars()) {
                              if (!c.driven) continue;
                              const driving::ThingAhead* t = d->thingAhead(c.id);
                              if (!t || !t->found) continue;
                              const driving::CarPose pose = driving::carPose(*d->roads(), d->vehicleClasses(), d->ruleSet(), c);
                              std::snprintf(buf, sizeof buf, " | %s car %u at %.1f %.1f, %.2f m/s, a thing %.1f m ahead%s",
                                            d->carried(c.id) ? "near" : "lane", c.id.index, pose.front.x, pose.front.z,
                                            c.speed, t->distance, t->moves < driving::kThingStillBelow ? ", standing" : ", moving");
                              out += buf;
                          }
                          const driving::Driving::ThingCounts& tc = d->thingCounts();
                          std::snprintf(buf, sizeof buf, " | horns %llu, swerves %llu, other ways %llu",
                                        static_cast<unsigned long long>(tc.horns), static_cast<unsigned long long>(tc.swerves),
                                        static_cast<unsigned long long>(tc.anotherWays));
                          out += buf;
                          return out;
                      },
                      kOwner);
    // AT THE CROSSWALKS (ADR-0452): people waiting at a kerb end or crossing, with the crosswalk (the road model's), its
    // control and its walk signal; drivers giving way at a crosswalk; cars on a green arrow; the counts
    ctx.commands->add("crosswalks?", "crosswalks?",
                      "People at the road model's crosswalks (ADR-0452): each one waiting at a kerb end or crossing, where, "
                      "which crosswalk, its control and walk signal; each driver giving way at one; each car on a connector "
                      "whose head shows a green arrow; and the counts so far.",
                      [this](const engine::ControlCommand&) -> std::string {
                          if (!world_) return "err no world yet";
                          const citysim::CitySim& sim = city_.sim();
                          const driving::Driving* d = sim.drivingV2();
                          const engine::RoadModel* m = d ? d->model() : nullptr;
                          if (!d || !m || !d->roads()) return "err no driving";
                          std::string out = "ok";
                          char buf[256];
                          for (std::size_t i = 0; i < sim.agents().size(); ++i) {
                              const citysim::Agent& a = sim.agents()[i];
                              if (a.mode == citysim::Agent::Mode::Driver || !a.moving || a.crosswalkAt < 0) continue;
                              const engine::ModelCrosswalk* c = m->crosswalk(static_cast<uint32_t>(a.crosswalkAt));
                              if (!c) continue;
                              const driving::WalkSignal ws = driving::walkAtCrosswalk(d->rightOfWay(), static_cast<uint32_t>(a.crosswalkAt),
                                                                                      d->clockSeconds());
                              std::snprintf(buf, sizeof buf, " | walker %zu %s crosswalk %d at %.1f %.1f (%s, %s, junction %s)", i,
                                            a.onCrosswalk ? "on" : "waiting at", a.crosswalkAt, a.pos.x, a.pos.y,
                                            engine::crosswalkControlWord(c->control), driving::walkWord(ws),
                                            m->junctions[c->junction.index].id.c_str());
                              out += buf;
                          }
                          for (const driving::Car& c : d->cars()) {
                              if (!c.driven) continue;
                              const driving::CarPose pose = driving::carPose(*d->roads(), d->vehicleClasses(), d->ruleSet(), c);
                              const uint32_t cw = d->crosswalkGivingWayAt(c.id);
                              if (cw != engine::kNoIndex) {
                                  std::snprintf(buf, sizeof buf, " | car %u gives way at crosswalk %u, at %.1f %.1f, %.2f m/s",
                                                c.id.index, cw, pose.front.x, pose.front.z, c.speed);
                                  out += buf;
                              }
                              if (d->roads()->paths[c.path.index].kind == engine::PathKind::Connector &&
                                  driving::lampShownAt(*d->roads(), d->rightOfWay(), c.path, d->clockSeconds()) == driving::Lamp::GreenArrow) {
                                  std::snprintf(buf, sizeof buf, " | car %u turns on a green arrow, at %.1f %.1f, %.2f m/s", c.id.index,
                                                pose.front.x, pose.front.z, c.speed);
                                  out += buf;
                              }
                          }
                          const citysim::CitySim::WalkStats& ws = sim.walkStats();
                          const driving::Driving::ThingCounts& tc = d->thingCounts();
                          std::snprintf(buf, sizeof buf, " | crossings %llu on walk, %llu at signs, %llu marked, %llu unmarked, %llu against "
                                        "the lights; drivers gave way %llu, %llu, %llu",
                                        static_cast<unsigned long long>(ws.crossings[0]), static_cast<unsigned long long>(ws.crossings[1]),
                                        static_cast<unsigned long long>(ws.crossings[2]), static_cast<unsigned long long>(ws.crossings[3]),
                                        static_cast<unsigned long long>(ws.crossings[4]),
                                        static_cast<unsigned long long>(tc.crosswalkGiveWays[0]),
                                        static_cast<unsigned long long>(tc.crosswalkGiveWays[1]),
                                        static_cast<unsigned long long>(tc.crosswalkGiveWays[2]));
                          out += buf;
                          return out;
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
                          std::snprintf(buf, sizeof buf, " | near %d (over the cap %d), taken %u, given back %u, contacts %u",
                                        counts_.nearNow, counts_.overCapNow, counts_.taken, counts_.givenBack, counts_.contacts);
                          out += buf;
                          if (d) {
                              const driving::Driving::ThingCounts& tc = d->thingCounts();
                              std::snprintf(buf, sizeof buf,
                                            " | in the way: horns %llu (near cars' %u), swerves %llu, other ways %llu, hits on "
                                            "people %u, on parked cars %u",
                                            static_cast<unsigned long long>(tc.horns), counts_.hornsPlayed,
                                            static_cast<unsigned long long>(tc.swerves),
                                            static_cast<unsigned long long>(tc.anotherWays), counts_.pedestrianHits,
                                            counts_.parkedHits);
                              out += buf;
                          }
                          for (const NearContact& k : counts_.contactList) {
                              std::snprintf(buf, sizeof buf, " | touch: car %u and car %u at %.1f %.1f %.1f, closing at %.2f m/s",
                                            k.a, k.b, k.at.x, k.at.y, k.at.z, k.speed);
                              out += buf;
                          }
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

// THE NEAR TIER (ADR-0448; design section 4), each fixed step before the controller.
void CityNearSystem::tierStep(engine::FrameContext& ctx) {
    engine::World& world = ctx.world;
    driving::Driving* d = city_.simMutable().drivingV2Mutable();
    CityBrain* brain = city_.simMutable().brain();
    if (!d) return;
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    engine::PhysicsWorld& pw = physics_.physicsWorld();

    // 1. CARS THAT LEAVE THE TIER BY THEMSELVES: the player sat in one (VehicleSystem's key: only the seat changes, the
    // household loses it, its driver put out), or the brain let one go from the driving (it parked: it slides into its
    // space as a car the city draws, so its Jolt car goes)
    std::vector<Entity> nears;
    world.each<CityNearCar, engine::Vehicle>([&](Entity e, CityNearCar&, engine::Vehicle&) { nears.push_back(e); });
    for (const Entity e : nears) {
        const CityNearCar* n = world.get<CityNearCar>(e);
        const engine::Vehicle* v = world.get<engine::Vehicle>(e);
        const CityCarBody* body = world.get<CityCarBody>(e);
        if (v->driver.valid()) {
            const driving::CarId id = n->car;
            if (d->carried(id)) d->carry(id, false);
            world.remove<CityNearCar>(e);
            if (brain && body) brain->takenByPlayer(static_cast<int>(body->id));
            if (world.has<engine::AgentDriver>(e)) world.remove<engine::AgentDriver>(e);
            ++counts_.taken;
            continue;
        }
        const driving::Car* c = d->car(n->car);
        if (!c || !d->carried(n->car)) {
            world.add<engine::VehicleRetire>(e, engine::VehicleRetire{true});
            world.remove<CityNearCar>(e);
        }
    }
    // 2. GIVEN BACK (#293): a household car the player had (taken near or with the key), left: once it has stopped its
    // Jolt car goes, then the household has it again where it stands (CityBrain::carLeftAt: in a kerb space beside it,
    // or where it stands, on the road or not)
    world.each<CityCarBody, CityCarLink, engine::Vehicle>([&](Entity e, CityCarBody&, CityCarLink& link, engine::Vehicle&) {
        if (link.away && std::find(taken_.begin(), taken_.end(), e) == taken_.end()) taken_.push_back(e);
    });
    for (std::size_t i = 0; i < taken_.size();) {
        const Entity e = taken_[i];
        const engine::Vehicle* v = world.alive(e) ? world.get<engine::Vehicle>(e) : nullptr;
        const CityCarBody* body = world.alive(e) ? world.get<CityCarBody>(e) : nullptr;
        if (!body) {
            taken_.erase(taken_.begin() + static_cast<std::ptrdiff_t>(i));
            continue;
        }
        if (v) {
            if (!v->driver.valid() && v->vehicleId != engine::PhysicsWorld::INVALID_VEHICLE &&
                pw.vehicleVelocity(v->vehicleId).length() < kGiveBackBelow && !world.has<engine::VehicleRetire>(e))
                world.add<engine::VehicleRetire>(e, engine::VehicleRetire{true});
            ++i;
            continue;
        }
        const engine::Transform* t = world.get<engine::Transform>(e);
        if (brain && t && brain->carLeftAt(static_cast<int>(body->id), carPosition(*t), carHeading(*t))) ++counts_.givenBack;
        taken_.erase(taken_.begin() + static_cast<std::ptrdiff_t>(i));
    }

    // 3. THE TIER: around the player (in a car, the car)
    int nearNow = 0;
    world.each<CityNearCar>([&](Entity, CityNearCar&) { ++nearNow; });
    counts_.nearNow = nearNow;
    if (rules_.cap <= 0) return;
    Vec2 centre;
    bool found = false;
    world.each<engine::Transform, engine::ControlledBy>([&](Entity p, engine::Transform& t, engine::ControlledBy&) {
        if (found) return;
        found = true;
        centre = flat(t.position);
        if (const engine::InVehicle* iv = world.get<engine::InVehicle>(p))
            if (const engine::Transform* ct = world.alive(iv->vehicle) ? world.get<engine::Transform>(iv->vehicle) : nullptr)
                centre = flat(ct->position);
    });
    if (!found) return;
    int switches = 0;
    // OUT, the farthest first: beyond `leave`, back to the lane when it may (else it stays, over the cap if it must)
    std::vector<std::pair<Real, Entity>> out;
    world.each<CityNearCar, engine::Transform>([&](Entity e, CityNearCar&, engine::Transform& t) {
        const Real away = (flat(t.position) - centre).length();
        if (away > rules_.leave) out.push_back({away, e});
    });
    std::sort(out.begin(), out.end(), [](const std::pair<Real, Entity>& a, const std::pair<Real, Entity>& b) {
        return a.first > b.first;
    });
    for (const std::pair<Real, Entity>& o : out) {
        if (switches >= rules_.perStep) break;
        if (toLane(world, o.second)) {
            ++switches;
            --nearNow;
        }
    }
    // IN, the nearest first: driven lane cars within `enter`, up to the cap
    std::vector<std::pair<Real, Entity>> in;
    world.each<CityCarBody, CityCarLink, CityCarOnLane, engine::Transform>(
        [&](Entity e, CityCarBody&, CityCarLink& link, CityCarOnLane& lane, engine::Transform& t) {
            if (link.away || !lane.driven || world.has<engine::Vehicle>(e)) return;
            const Real away = (flat(t.position) - centre).length();
            if (away < rules_.enter) in.push_back({away, e});
        });
    std::sort(in.begin(), in.end(), [](const std::pair<Real, Entity>& a, const std::pair<Real, Entity>& b) {
        return a.first < b.first;
    });
    for (const std::pair<Real, Entity>& c : in) {
        if (switches >= rules_.perStep || nearNow >= rules_.cap) break;
        if (toPhysics(world, ctx.assets, c.second, ctx.clock.fixedStep())) {
            ++switches;
            ++nearNow;
        }
    }
    counts_.nearNow = nearNow;
    counts_.overCapNow = std::max(0, nearNow - rules_.cap);
    counts_.mostOverCap = std::max(counts_.mostOverCap, counts_.overCapNow);
    counts_.mostThisStep = std::max(counts_.mostThisStep, switches);
    counts_.tierMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    ++counts_.tierSteps;
}

void CityNearSystem::update(engine::FrameContext& ctx) {
    if (!pressEnter_) return;
    pressEnter_ = false;
    ctx.actions.injectPress("enter_vehicle");
}

void CityNearSystem::render(engine::FrameContext& ctx) {
    if (!marks_) return;
    const Vec3 kNearMark(0.1, 0.9, 1.0);   // cyan
    ctx.world.each<CityNearCar, engine::Transform>([&](Entity, CityNearCar&, engine::Transform& t) {
        ctx.debug.line(t.position, t.position + Vec3(0, kNearPostHeight, 0), kNearMark);
        ctx.debug.sphere(t.position + Vec3(0, kNearPostHeight, 0), 0.4, kNearMark);
    });
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
    // THE CARS THE DRIVING ALREADY SEES (ADR-0451): the near cars and the lane cars' boxes, passed over by the look ahead
    // (a car the driving does not drive -- parked, sliding into its space, the player's -- is a thing like any other)
    if (d.thingStep()) {
        known_.clear();
        world_->each<CityNearCar, engine::Vehicle>([&](Entity, CityNearCar&, engine::Vehicle& v) {
            if (v.vehicleId != engine::PhysicsWorld::INVALID_VEHICLE) known_.push_back(pw.vehicleBody(v.vehicleId));
        });
        if (boxes_)
            world_->each<CityCarOnLane>([&](Entity e, CityCarOnLane&) {
                const engine::PhysicsBodyId box = boxes_->entityCarBox(*world_, e);
                if (box != engine::INVALID_PHYSICS_BODY) known_.push_back(box);
            });
        std::sort(known_.begin(), known_.end());
    }
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
        if (d.thingStep()) seeAhead(d, n, v, at, speed);
    });
    if (any) counts_.controlMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// (a honk: the horn's loop-clean period repeated, faded in and out so it neither clicks on nor off)
constexpr int kHonkPeriods = 3;
constexpr Real kHonkAttack = 0.03, kHonkRelease = 0.09;   // s
constexpr Real kHonkHeard = 80.0;                          // m: how far it carries

void CityNearSystem::soundHorns(engine::FrameContext& ctx, const driving::Driving& d) {
    for (const driving::CarId id : d.hornsLastTick()) {
        ctx.world.each<CityNearCar, engine::Transform>([&](Entity, CityNearCar& n, engine::Transform& t) {
            if (n.car != id) return;
            ++counts_.hornsPlayed;
            LOG_INFO << "[near] horn: car " << id.index << " at " << t.position.x << " " << t.position.z;
            if (!ctx.audio.ready()) return;
            if (!hornClip_.valid()) {
                const std::vector<float> period = engine::sfx::horn(ctx.audio.sampleRate(), id.index + 1);
                std::vector<float> pcm;
                for (int k = 0; k < kHonkPeriods; ++k) pcm.insert(pcm.end(), period.begin(), period.end());
                const float rate = static_cast<float>(ctx.audio.sampleRate());
                const std::size_t in = static_cast<std::size_t>(kHonkAttack * rate), out = static_cast<std::size_t>(kHonkRelease * rate);
                for (std::size_t s = 0; s < pcm.size(); ++s) {
                    const float a = s < in ? static_cast<float>(s) / static_cast<float>(in) : 1.0f;
                    const float r = s + out > pcm.size() ? static_cast<float>(pcm.size() - s) / static_cast<float>(out) : 1.0f;
                    pcm[s] *= std::min(a, r);
                }
                hornClip_ = ctx.audio.createClip(pcm.data(), pcm.size(), 1, ctx.audio.sampleRate());
            }
            if (hornClip_.valid()) ctx.audio.playAt(hornClip_, t.position, kHonkHeard, engine::AudioPlayParams{});
        });
    }
}

void CityNearSystem::seeAhead(driving::Driving& d, const CityNearCar& n, const engine::Vehicle& v, const Vec3& at,
                              Real speed) {
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    const engine::PhysicsWorld& pw = physics_.physicsWorld();
    driving::ThingAhead seen;
    const driving::Car* c = d.car(n.car);
    const std::vector<Vec2>& pts = n.follower.points();
    const std::vector<Real>& cum = n.follower.arcs();
    if (c && n.follower.valid()) {
        const Real halfWidth = v.config.chassisHalfExtent.x;
        const Real from = n.follower.station() + n.halfLength;
        const Real v0 = std::max(Real(0), speed);
        Real reach = std::max(kSeeLeast, v0 * v0 / (2 * std::max(n.brake, kSeeLeastBrake)) + v0 * kSeeReaction +
                                             d.thingStandOff() + kSeeBeyond);
        // ...and through the next junction's box to a car's length and a stand-off beyond it: room to stand there is
        // judged before going in (things.h thing_stop), at the line
        const std::vector<engine::PathId> way = d.wayFromFront(n.car);
        Real upTo = -c->along;
        for (const engine::PathId p : way) {
            upTo += d.roads()->paths[p.index].length;
            if (d.roads()->paths[p.index].kind != engine::PathKind::Connector) continue;
            reach = std::max(reach, upTo + 2 * n.halfLength + d.thingStandOff() + kSeeBeyond);
            break;
        }
        Real distance = 0;
        engine::PhysicsWorld::SweepHit hit;
        Vec2 dir;
        int casts = 0;
        bool met = sweepAlong(pw, pts, cum, from, reach, 0, halfWidth, at.y, known_, distance, hit, dir, &casts);
        // ...and SOMETHING ABOUT TO STEP INTO IT: a wider sweep over the road I cover in a few seconds; of all it meets,
        // a thing moving toward my line that reaches my side of it before I reach it (and a little after) is in my way
        {
            const Real near = std::min(met ? distance : reach, std::max(kSeeLeast * Real(0.5), v0 * kComingLook));
            const Real end = std::min(cum.back(), from + near);
            std::vector<engine::PhysicsWorld::SweepHit> all;
            bool coming = false;
            for (Real s = from; s < end && !coming; s += kSweepStep) {
                const Real s1 = std::min(end, s + kSweepStep);
                const Vec2 a = engine::pointAtStation(pts.data(), cum.data(), pts.size(), s);
                const Vec2 b = engine::pointAtStation(pts.data(), cum.data(), pts.size(), s1);
                const Real len = (b - a).length();
                if (!(len > engine::kSamePoint)) continue;
                const Vec2 d2 = (b - a) * (1 / len);
                all.clear();
                ++casts;
                pw.sweepBoxAll(Vec3(halfWidth + kComingWiden, kSweepHalfHeight, kSweepHalfDepth), Vec3(a.x, at.y, a.y),
                               engine::Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(d2.x, d2.y)), Vec3(b.x - a.x, 0, b.y - a.y),
                               known_, all);
                for (const engine::PhysicsWorld::SweepHit& h2 : all) {
                    const Real dist = s - from + h2.fraction * len;
                    const Vec2 on2 = a + d2 * (h2.fraction * len);
                    const Real side = (h2.point.x - on2.x) * d2.y - (h2.point.z - on2.y) * d2.x;
                    const Real across = h2.velocity.x * d2.y - h2.velocity.z * d2.x;   // (+ toward the kerb side)
                    const Real gap = std::fabs(side) - halfWidth;
                    if (!(side * across < 0) || !(gap > 0)) continue;
                    if (gap / std::fabs(across) > dist / std::max(v0, kComingSlowest) + kComingMargin) continue;
                    met = coming = true;
                    distance = dist;
                    hit = h2;
                    dir = d2;
                    break;
                }
            }
        }
        if (met) {
            seen.found = true;
            seen.id = hit.character ? kCharacterThing : static_cast<uint32_t>(hit.body);
            seen.distance = distance;
            seen.speed = hit.velocity.x * dir.x + hit.velocity.z * dir.y;
            seen.moves = Vec2(hit.velocity.x, hit.velocity.z).length();
            const Vec2 on = engine::pointAtStation(pts.data(), cum.data(), pts.size(), from + distance);
            seen.side = (hit.point.x - on.x) * dir.y - (hit.point.z - on.y) * dir.x + n.lineOffset;
            // the path it stands on: along the way from my front
            Real rest = c->along + distance;
            for (const engine::PathId p : way) {
                seen.path = p;
                const Real len = d.roads()->paths[p.index].length;
                if (rest <= len) break;
                rest -= len;
            }
            // ROOM TO GO ROUND IT, as physics finds it: the nearest sideways offset (either way, in half-metre steps) along
            // which a box my width and a clearance either side meets nothing from my front to well past it -- in my lane,
            // or one lane over to the offside (oncoming traffic included, looked for further ahead), never past the kerb
            if (v0 < kLookForRoomBelow && c->path.index < d.roads()->paths.size()) {
                const Real lane = d.roads()->paths[c->path.index].width * Real(0.5);
                const Real clearance = d.passClearance();
                const std::vector<engine::PhysicsBodyId> self{pw.vehicleBody(v.vehicleId)};
                for (int k = 1; k <= kPassTries && !seen.roomToPass; ++k)
                    for (const Real sign : {Real(-1), Real(1)}) {
                        const Real o = sign * kPassStep * static_cast<Real>(k);
                        const bool kerbward = o > 0;
                        if (kerbward ? o > lane - halfWidth : -o > lane - halfWidth + 2 * lane) continue;
                        const bool oncoming = -o > lane - halfWidth;
                        const Real stretch = distance + 2 * n.halfLength + 2 * clearance + kPassThingLength +
                                             (oncoming ? kPassOncomingLook : Real(0));
                        Real d2 = 0;
                        engine::PhysicsWorld::SweepHit h2;
                        Vec2 dir2;
                        if (sweepAlong(pw, pts, cum, from, stretch, o - n.lineOffset, halfWidth + clearance, at.y, self, d2, h2,
                                       dir2, &casts))
                            continue;
                        seen.roomToPass = true;
                        seen.passOffset = o;
                        break;
                    }
            }
        }
        counts_.sweepCasts += static_cast<uint64_t>(casts);
        ++counts_.sweeps;
    }
    d.seeAhead(n.car, seen);
    counts_.sweepMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void CityNearSystem::fixedUpdate(engine::FrameContext& ctx) {
    world_ = &ctx.world;
    assets_ = &ctx.assets;
    const driving::Driving* d = city_.sim().drivingV2();
    if (!d || !d->roads()) return;
    if (!rulesRead_) {
        rulesRead_ = true;
        ctx.world.each<engine::CitySimConfig>([&](Entity, engine::CitySimConfig& c) { rules_ = c.near; });
    }
    if (!measured_ && city_.built() && rules_.cap > 0) measureSlots(ctx.world);
    if (!staged_.empty()) sendStaged(ctx.world, ctx.clock.fixedStep());
    tierStep(ctx);
    soundHorns(ctx, *d);
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

void CityNearSystem::refuse(const std::string& reason, uint32_t car) {
    ++counts_.refused;
    ++counts_.refusedBy[reason];
    std::map<uint32_t, std::string>::iterator last = lastRefusal_.find(car);
    if (car != kNoCarNumber && last != lastRefusal_.end() && last->second == reason) return;
    lastRefusal_[car] = reason;
    ++counts_.refusedCars;
    ++counts_.refusedCarsBy[reason];
}

const CityNearSystem::SlotCar& CityNearSystem::slotCar(int slot) {
    std::map<int, SlotCar>::iterator found = slotCars_.find(slot);
    if (found != slotCars_.end()) return found->second;
    SlotCar s;
    s.spec = std::make_shared<engine::VehicleSpec>();
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    if (!vehiclesVm_) vehiclesVm_ = openVehiclesLibrary(&s.why);
    if (vehiclesVm_ && slotVehicleSpec(city_, slot, *s.spec, nullptr, &s.why, vehiclesVm_.get())) {
        const std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        const engine::PhysicsWorld::CarMeasures m = engine::PhysicsWorld::measureCar(s.spec->config);
        s.restHeight = m.restHeight;
        s.launch = m.launch;
        s.brake = m.brake;
        if (!(s.restHeight > 0)) s.why = "its Jolt car could not be stood on its springs";
        LOG_INFO << "[near] fleet slot " << slot << ": its spec built in "
                 << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms, its Jolt car measured in "
                 << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count() << " ms";
    }
    return slotCars_.emplace(slot, s).first->second;
}

// EVERY FLEET SLOT THE LEVEL'S CARS USE, measured as the level loads (the first step the city is built): building a
// slot's spec and measuring its Jolt car took 1.2 s at the first switch (ADR-0447).
void CityNearSystem::measureSlots(engine::World& world) {
    measured_ = true;
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    std::vector<int> slots;
    world.each<CityCarBody>([&](Entity, CityCarBody& b) {
        if (std::find(slots.begin(), slots.end(), b.slot) == slots.end()) slots.push_back(b.slot);
    });
    for (const int s : slots) slotCar(s);
    counts_.slotsMeasured = static_cast<int>(slots.size());
    counts_.measureMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (!slots.empty())
        LOG_INFO << "[near] " << slots.size() << " fleet slots measured for the near tier in " << counts_.measureMs << " ms";
}

bool CityNearSystem::toPhysics(engine::World& world, engine::AssetManager& assets, Entity e, Real step, std::string* why) {
    const auto refuse = [&](const std::string& reason, const std::string& detail = std::string()) {
        const CityCarBody* refused = world.get<CityCarBody>(e);
        this->refuse(reason, refused ? refused->id : kNoCarNumber);
        if (why) *why = detail.empty() ? reason : reason + ": " + detail;
        return false;
    };
    driving::Driving* d = city_.simMutable().drivingV2Mutable();
    const CityCarBody* body = world.get<CityCarBody>(e);
    const CityCarOnLane* lane = world.get<CityCarOnLane>(e);
    const CityCarPose* pose = world.get<CityCarPose>(e);
    if (!d || !body || !lane || !pose) return refuse("not a car the driving has on its lanes");
    if (world.has<engine::Vehicle>(e)) return refuse("it is a physics car already");
    const driving::Car* c = d->car(lane->car);
    if (!c || !c->driven) return refuse("the driving has it on no trip");
    if (c->vehicleClass >= d->vehicleClasses().size()) return refuse("its vehicle class is unknown");
    const SlotCar& slotted = slotCar(body->slot);
    if (!slotted.why.empty() || !slotted.spec) return refuse("its slot has no Jolt car", slotted.why);
    const engine::VehicleSpec& spec = *slotted.spec;
    const SlotCar* measuredCar = &slotted;

    // WHERE ITS BODY IS THIS STEP (city_car.h CityCarPose, what its box and its drawing are at): its middle and heading,
    // on the road it stands by, standing on its springs; a step of its speed on, where the lane car would be when
    // physics first moves it
    const Vec2 heading = engine::normalize(Vec2(pose->body.m[0][2], pose->body.m[2][2]));
    const Real speed = c->speed;
    const Vec2 middle = Vec2(pose->body.m[0][3], pose->body.m[2][3]) + heading * (speed * step);
    const Vec3 at(middle.x, pose->road + measuredCar->restHeight, middle.y);
    engine::makeVehicle(world, assets, spec, e, at, std::atan2(heading.x, heading.y) * kDegrees);
    // its kinematic box goes now: the chassis is made at the end of this step and steps first in the next, before the
    // boxes are synced again (ADR-0449)
    if (boxes_) boxes_->dropEntityCarBox(world, e);
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
    n.launch = measuredCar->launch;
    n.brake = measuredCar->brake;
    const driving::CarId id = lane->car;
    world.remove<CityCarOnLane>(e);   // (the driving no longer draws it: physics does)
    world.add<CityNearCar>(e, n);
    d->carry(id, true, kNearLag);
    ++counts_.intoPhysics;
    lastRefusal_.erase(body->id);
    return true;
}

bool CityNearSystem::toLane(engine::World& world, Entity e, std::string* why) {
    const auto refuse = [&](const std::string& reason, const std::string& detail = std::string()) {
        const CityCarBody* refused = world.get<CityCarBody>(e);
        this->refuse(reason, refused ? refused->id : kNoCarNumber);
        if (why) *why = detail.empty() ? reason : reason + ": " + detail;
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
    if (std::fabs(c->offset) > kBackOffset) return refuse("off its lane's centre", std::to_string(c->offset) + " m");
    const driving::CarPose pose = driving::carPose(*d->roads(), d->vehicleClasses(), d->ruleSet(), *c);
    const Vec2 lanes = engine::normalize(Vec2(pose.travel.x, pose.travel.z));
    const Vec2 forward = engine::normalize(flat(q.rotate(Vec3(0, 0, 1))));
    const Real align = std::acos(std::clamp(lanes.x * forward.x + lanes.y * forward.y, Real(-1), Real(1))) * kDegrees;
    if (align > kBackAlignDegrees) return refuse("not pointing along its lane", std::to_string(align) + " degrees");

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
    if (const CityCarBody* body = world.get<CityCarBody>(e)) lastRefusal_.erase(body->id);
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
    city_.simMutable().poses().poseCarNow(world, e, city_.sim());   // (its body's pose now, not at the next step: ADR-0454)
    return e;
}

}  // namespace citysim
