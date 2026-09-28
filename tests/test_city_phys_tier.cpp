// THE PHYSICAL-TIER SOAK (roads-v2.1 R5, plan §5c gate): a real signalled
// city grid, the CitySim's kinematic brains, and the possession tier
// promoting the moving drivers nearest the player to Jolt wheeled vehicles
// that chase the sim's ghosts. Gates, in Glenn's terms: the tier actually
// engages (and respects its budget), no car ever flips, the physical car
// never visibly rubber-bands away from its ghost, and walking away releases
// every body.
#include "test_framework.h"

#include "../src/apps/citysim/city_physics.h"
#include "../src/apps/citysim/city_render.h"
#include "../src/engine/components.h"
#include "../src/engine/procgen/city/roads/road_entity.h"
#include "../src/engine/systems/physics_system.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/world.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

using namespace engine;
using namespace citysim;

namespace {

// 4x4 junction grid, 80 m pitch: the interior nodes are degree-4 signalled
// crossings, enough network for two dozen wandering drivers.
RoadEntity cityGrid() {
    RoadEntity net;
    net.look.defaultWidth = 10.0;
    net.look.sidewalk = 2.5;
    const int N = 4;
    const Real pitch = 80;
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i)
            net.graph.nodes.push_back(RoadNode{Vec2(i * pitch, j * pitch)});
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i) {
            if (i + 1 < N)
                net.graph.addEdge(j * N + i, j * N + i + 1, net.look.defaultWidth);
            if (j + 1 < N)
                net.graph.addEdge(j * N + i, (j + 1) * N + i, net.look.defaultWidth);
        }
    return net;
}

// The shipped vehicle catalogue. Cars are CONTENT: a city with no recipes draws
// none at all (the built-in box fleet is gone), so a soak that expects cars to
// collide with has to say which cars — exactly as a level does.
std::string readAsset(const std::string& name) {
    std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/" + name);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// A stub mesh backend: car bodies must actually be built for the fleet to
// exist, and building needs somewhere to upload to. No GPU.
struct StubUploader : engine::MeshUploader {
    uint32_t next = 1;
    engine::MeshHandle uploadMesh(const engine::RenderMesh&) override {
        return engine::MeshHandle{next++, 1};
    }
    void removeMesh(engine::MeshHandle) override {}
    engine::BoundingSphere getMeshBounds(engine::MeshHandle) const override { return {}; }
};

Real upDot(const Quat& q) { return q.rotate(Vec3(0, 1, 0)).y; }

}  // namespace

TEST_CASE(city_phys_tier_soak) {
    World world;
    world.add<RoadEntity>(world.create(), cityGrid());

    CityRenderParams params;
    params.cars = 40;   // busy near the player (24 had drifted to 5 within the ring)
    params.pedestrians = 0;
    params.seed = 7;
    params.wander = true;   // perpetual trips: drivers keep moving all soak
    params.physicalCars = 12;   // this soak exercises the physical tier
    CityRenderSystem city(params);
    CHECK(city.build(world, nullptr));

    // The player stands mid-grid; the tier centres its ring on this body.
    Entity player = world.create();
    Transform pt;
    pt.position = Vec3(120, 0.9, 120);
    world.add<Transform>(player, pt);
    world.add<CharacterController>(player, CharacterController{});

    PhysicsSystem physics;
    physics.initialize();
    // Flat ground plane under the whole grid (roads sit at y ~= 0).
    physics.physicsWorld().addBox(Vec3(600, 1, 600), Vec3(120, -1, 120),
                                  Quat::identity(), BodyMotion::Static);
    CityPhysicsSystem bridge(city, physics);

    // What stands near the measured trap point (163,152)? Poles are the
    // only static bodies besides the ground.
    if (InstanceGroup* posts = world.get<InstanceGroup>(city.signalPostGroup()))
        for (const Mat4& m : posts->transforms) {
            const Real px = m.m[0][3], pz = m.m[2][3];
            if (px > 150 && px < 175 && pz > 140 && pz < 170)
                std::printf("    [pole] (%.1f, %.1f)\n", px, pz);
        }

    const Real dt = 1.0 / 60.0;
    std::size_t maxPossessed = 0;
    int maxInRing = 0;
    long ticksShort = 0;   // ticks the tier held fewer than the ring offered (beyond one acquisition's lag)
    Real minUp = 1.0, worstDiv = 0.0;
    long ticksHeld = 0, ticksAbove3 = 0;
    Real worstExcursion = 0;                  // longest continuous d>3 spell (s)
    std::unordered_map<int, Real> spell;      // per-agent running d>3 spell
    std::unordered_map<int, Real> age;   // possessed agent -> seconds held
    for (int i = 0; i < 60 * 60; ++i) {
        city.step(world, dt);
        bridge.step(world, dt);
        physics.step(world, dt);

        PhysicsWorld& pw = physics.physicsWorld();
        std::unordered_map<int, Real> next;
        for (const auto& p : bridge.possessed()) {
            const Real a = age.count(p.agent) ? age[p.agent] + dt : 0.0;
            next[p.agent] = a;
            minUp = std::min(minUp, upDot(pw.vehicleOrientation(p.vid)));
            // Divergence body vs ghost, after a 2 s settle (the body spawns
            // at the ghost then has to catch its moving frame).
            if (a > 2.0) {
                const Vec3 bp = pw.vehiclePosition(p.vid);
                const Agent& ag = city.sim().agents()[p.agent];
                const Vec2 g = ag.pos;
                const Real dx = bp.x - g.x, dz = bp.z - g.y;
                const Real d = std::sqrt(dx * dx + dz * dz);

                worstDiv = std::max(worstDiv, d);
                ++ticksHeld;
                if (d > 3.0) ++ticksAbove3;
                // A spell counts SERIOUS divergence (> 6 m): a pin against
                // furniture/queued boxes must die fast (the snap fuses).
                // 3-4.5 m stop-line offsets and corner trails live below
                // this bar and are invisible (nothing rendered references
                // the ghost).
                if (d > 6.0) {
                    Real& e = spell[p.agent];
                    e += dt;
                    worstExcursion = std::max(worstExcursion, e);
                } else {
                    spell[p.agent] = 0;
                }
            }
        }
        age.swap(next);
        maxPossessed = std::max(maxPossessed, bridge.possessed().size());
        // The tier possesses every ELIGIBLE car in its ring, up to its budget: count the moving, near-tier
        // drivers inside the 90 m ring (the 1.3x drop band keeps a few more held past it).
        int inRing = 0;
        for (const Agent& a : city.sim().agents()) {
            if (a.mode != Agent::Mode::Driver || !a.moving || a.released || a.far()) continue;
            const Real dx = a.pos.x - 120, dz = a.pos.y - 120;
            if (dx * dx + dz * dz < 90 * 90) ++inRing;
        }
        maxInRing = std::max(maxInRing, inRing);
        if (static_cast<int>(bridge.possessed().size()) < std::min(inRing, 12) - 1) ++ticksShort;
    }

    // Walk the player out of the city: every body must be released.
    world.get<Transform>(player)->position = Vec3(5000, 0.9, 5000);
    for (int i = 0; i < 60 * 3; ++i) {
        city.step(world, dt);
        bridge.step(world, dt);
        physics.step(world, dt);
    }

    const Real fracAbove3 =
        ticksHeld > 0 ? Real(ticksAbove3) / Real(ticksHeld) : 0.0;
    std::printf(
        "    [tier] maxPossessed=%zu minUp=%.3f worstDiv=%.2f "
        "fracAbove3=%.3f worstSpell=%.1fs snaps=%d released=%zu\n",
        maxPossessed, minUp, worstDiv, fracAbove3, worstExcursion,
        bridge.snapCount(), bridge.possessed().size());
    std::printf("    [tier] eligible in the ring at most %d; ticks holding fewer than offered: %ld\n", maxInRing, ticksShort);
    // The tier engages on a busy grid. This used to be only `maxPossessed >= 6` with 24 cars -- a claim about
    // TRAFFIC (how many cars wander within 90 m of the player), which drifted to 5 as the drivers' routing
    // changed while the tier kept possessing every car it was offered. Now: the grid is busy (40 cars), and the
    // tier holds what its ring offers, up to its budget.
    CHECK(maxInRing >= 6);             // the scenario really is busy near the player
    CHECK(maxPossessed >= 6);          // the tier engages
    CHECK(ticksShort < 60);            // and keeps up with its ring (under a second short, all soak)
    CHECK(maxPossessed <= 12);         // and respects its budget
    CHECK(minUp > 0.5);                // zero flips (plan gate)
    // "No visible rubber-band", measured against what a PLAYER can see —
    // nothing rendered references the invisible ghost, so the visual truths
    // are: no flips (above), serious divergence (> 6 m: a pin, a lost body)
    // dies within seconds via the fuses, the absolute error stays fuse-
    // bounded, and teleport rescues stay rare. The sim-truth lag share
    // (fracAbove3: stop-line offsets <= 4.5 m, corner trails) is reported
    // and loosely bounded; tightening it below ~0.1 needs V2 path-
    // feedforward (drive the ghost's ROUTE, not the ghost).
    CHECK(fracAbove3 < 0.25);
    CHECK(worstExcursion < 4.0);
    CHECK(worstDiv < 12.5);
    // Observed 8/min bare, 13/min once curbside parking rows line every
    // link of this stress grid (parked kinematic boxes ~0.9 m off the lane
    // give corner-exit tracking error something to brush; each rescue is a
    // ~1.5 s pin). Metropolis locals are sparser than this fixture.
    CHECK(bridge.snapCount() <= 16);
    CHECK(bridge.possessed().size() == 0);   // out of range: all released

    physics.shutdown();
}

// P4: the possess tier fed by V promotions. With tiering ON and the bubble
// shrunk below this small grid's span, most drivers live as far (V) agents;
// the K set around the player — including cars promoted INTO it as they
// drive up — is what the possess ring draws from, the render bake shows, and
// the incremental proxy diff (uid-keyed add/remove/move) tracks. Gates: the
// physical tier still engages and respects its budget, no flips, drawn cars +
// far cars always account for every driver, and walking away releases all.
TEST_CASE(city_phys_tier_with_tiering) {
    World world;
    world.add<RoadEntity>(world.create(), cityGrid());

    CityRenderParams params;
    params.cars = 24;
    params.pedestrians = 8;
    params.seed = 7;
    params.wander = true;
    params.physicalCars = 12;   // this soak exercises the physical tier
    CityRenderSystem city(params);
    CHECK(city.build(world, nullptr));
    CitySim& sim = city.simMutable();
    sim.tieringEnabled = true;
    sim.carPromoteRadius = 100.0;   // the grid spans ~340 m corner to corner,
    sim.carDemoteRadius = 130.0;    // so a mid-grid player K-bubbles a corner
    sim.pedPromoteRadius = 100.0;   // of it and the rest of the town runs V
    sim.pedDemoteRadius = 130.0;

    Entity player = world.create();
    Transform pt;
    pt.position = Vec3(120, 0.9, 120);
    world.add<Transform>(player, pt);
    world.add<CharacterController>(player, CharacterController{});

    PhysicsSystem physics;
    physics.initialize();
    physics.physicsWorld().addBox(Vec3(600, 1, 600), Vec3(120, -1, 120),
                                  Quat::identity(), BodyMotion::Static);
    CityPhysicsSystem bridge(city, physics);

    const Real dt = 1.0 / 60.0;
    std::size_t maxPossessed = 0;
    Real minUp = 1.0;
    for (int i = 0; i < 60 * 20; ++i) {
        city.step(world, dt);
        bridge.step(world, dt);
        physics.step(world, dt);
        maxPossessed = std::max(maxPossessed, bridge.possessed().size());
        for (const auto& p : bridge.possessed())
            minUp = std::min(
                minUp, upDot(physics.physicsWorld().vehicleOrientation(p.vid)));
        if (i % 120 == 0) {
            // Conservation across the tier seam: every driver is either a
            // drawn K car or a far V agent — never both, never neither.
            int vDrivers = 0, kDrivers = 0;
            for (const Agent& a : city.sim().agents()) {
                if (a.mode != Agent::Mode::Driver) continue;
                (a.tier == Agent::Tier::V ? vDrivers : kDrivers)++;
            }
            CHECK(vDrivers + kDrivers == 24);
        }
    }
    std::printf("    [tier+V] maxPossessed=%zu minUp=%.3f promos=%ld demos=%ld "
                "snaps=%d\n",
                maxPossessed, minUp, sim.tierPromotions(), sim.tierDemotions(),
                bridge.snapCount());
    CHECK(sim.tierDemotions() > 0);   // the bubble engaged (town went far)
    CHECK(maxPossessed >= 1);         // ...and K cars near the player possess
    CHECK(maxPossessed <= 12);
    CHECK(minUp > 0.5);               // no flips among the physical bodies

    // Walk the player out: bodies release; the whole town demotes to V and
    // the proxy diff strips every box without a rebuild storm.
    world.get<Transform>(player)->position = Vec3(5000, 0.9, 5000);
    for (int i = 0; i < 60 * 5; ++i) {
        city.step(world, dt);
        bridge.step(world, dt);
        physics.step(world, dt);
    }
    CHECK(bridge.possessed().size() == 0);

    physics.shutdown();
}

// GATE for #26 ("I can walk through cars"). A drawn ambient car the player can
// walk through is one with no SOLID body where it is drawn: either its kinematic
// proxy box is missing, or the box parked itself below the world (which the tier
// does on purpose for possessed cars, whose dynamic chassis takes over), or the
// pool drifted out of step with the instances it is meant to track. This soak
// asserts the invariant directly, every tick: every drawn car is backed by
// SOMETHING solid at its own position — a proxy box or a possessed chassis —
// and then physically walks the player into one to prove it blocks.
TEST_CASE(city_every_drawn_car_is_solid) {
    World world;
    world.add<RoadEntity>(world.create(), cityGrid());

    CityRenderParams params;
    params.cars = 24;
    params.pedestrians = 0;
    params.seed = 7;
    params.wander = true;
    params.physicalCars = 12;   // this soak exercises the physical tier
    params.vehicleScript = readAsset("vehicles.lua");
    CHECK(!params.vehicleScript.empty());
    CityRenderSystem city(params);
    StubUploader uploader;
    engine::AssetManager assets(uploader);
    CHECK(city.build(world, &assets));

    Entity player = world.create();
    Transform pt;
    pt.position = Vec3(120, 0.9, 120);
    world.add<Transform>(player, pt);
    world.add<CharacterController>(player, CharacterController{});

    PhysicsSystem physics;
    physics.initialize();
    physics.physicsWorld().addBox(Vec3(600, 1, 600), Vec3(120, -1, 120),
                                  Quat::identity(), BodyMotion::Static);
    physics.createBodies(world);   // makes the player's real capsule
    CityPhysicsSystem bridge(city, physics);
    PhysicsWorld& pw = physics.physicsWorld();

    const Real dt = 1.0 / 60.0;
    long worstUncovered = 0;      // drawn cars with nothing solid at them
    Real worstGap = 0;            // how far the nearest solid body was (m)
    long checkedCars = 0;
    for (int i = 0; i < 60 * 30; ++i) {
        city.step(world, dt);
        bridge.step(world, dt);
        physics.step(world, dt);

        // Where is something SOLID? Every proxy box, plus every possessed
        // chassis (whose proxy deliberately parks out of the way).
        std::vector<Vec3> solid;
        for (engine::PhysicsBodyId id : bridge.carProxyBodies())
            solid.push_back(pw.bodyPosition(id));
        for (const auto& p : bridge.possessed())
            solid.push_back(pw.vehiclePosition(p.vid));

        long uncovered = 0;
        for (std::size_t gi = 0; gi < city.carGroups().size(); ++gi) {
            InstanceGroup* g = world.get<InstanceGroup>(city.carGroups()[gi]);
            if (!g) continue;
            for (std::size_t ii = 0; ii < g->transforms.size(); ++ii) {
                const Mat4& m = g->transforms[ii];
                ++checkedCars;
                const Vec3 c(m.m[0][3], m.m[1][3], m.m[2][3]);
                Real best = 1e30;
                for (const Vec3& s : solid)
                    best = std::min(best, (s - c).length());
                // A possessed chassis trails its drawn pose by a metre or two
                // only when the render pose is the ghost's; here the drawn pose
                // IS the body pose, so the tolerance only has to absorb the
                // chassis origin vs box centre offset.
                if (best > 1.5) {
                    ++uncovered;
                    const int aid = gi < city.carAgentIds().size() &&
                                            ii < city.carAgentIds()[gi].size()
                                        ? city.carAgentIds()[gi][ii]
                                        : -2;
                    bool poss = false;
                    for (const auto& pp : bridge.possessed())
                        if (pp.agent == aid) poss = true;
                    // Only printed when the invariant breaks: which car, where
                    // it was drawn, and whether the tier owned it — enough to
                    // tell a missing proxy from a possessed car drawn off its
                    // chassis without re-instrumenting.
                    Vec3 chassis(0, 0, 0);
                    for (const auto& pp : bridge.possessed())
                        if (pp.agent == aid) chassis = pw.vehiclePosition(pp.vid);
                    std::printf("    [hole] tick=%d agent=%d possessed=%d "
                                "gap=%.2f drawn=(%.1f,%.1f) chassis=(%.1f,%.1f)\n",
                                i, aid, poss ? 1 : 0, (double)best, (double)c.x,
                                (double)c.z, (double)chassis.x, (double)chassis.z);
                }
                worstGap = std::max(worstGap, std::min(best, Real(50)));
            }
        }
        worstUncovered = std::max(worstUncovered, uncovered);
    }
    std::printf("    [solid] checked=%ld worstUncovered=%ld worstGap=%.2f\n",
                checkedCars, worstUncovered, worstGap);
    CHECK(checkedCars > 0);
    CHECK(worstUncovered == 0);   // no drawn car is ever a hologram

    // And the invariant is not vacuous: walk the player straight at a drawn car
    // and it must not pass through. Pick a car, stand 4 m in front of its nose,
    // and push toward it for two seconds.
    Vec3 target(0, 0, 0);
    bool haveTarget = false;
    for (Entity ge : city.carGroups()) {
        InstanceGroup* g = world.get<InstanceGroup>(ge);
        if (!g || g->transforms.empty()) continue;
        const Mat4& m = g->transforms[0];
        target = Vec3(m.m[0][3], m.m[1][3], m.m[2][3]);
        haveTarget = true;
        break;
    }
    CHECK(haveTarget);

    CharacterController* cc = world.get<CharacterController>(player);
    CHECK(cc && cc->characterId != engine::INVALID_CHARACTER);
    const Vec3 approach(1, 0, 0);
    const Vec3 start = target - approach * 4.0 + Vec3(0, 0.9 - target.y, 0);
    pw.setCharacterPosition(cc->characterId, Vec3(start.x, 0.9, start.z));

    Real closest = 1e30;
    bool passedThrough = false;
    for (int i = 0; i < 120; ++i) {
        // Freeze the city so the target car stays put under the walk test —
        // this leg is about the proxy being solid, not about chasing traffic.
        bridge.step(world, dt);
        pw.moveCharacter(cc->characterId, approach * 3.0, dt);
        physics.step(world, dt);
        const Vec3 pp = pw.characterPosition(cc->characterId);
        closest = std::min(closest, std::fabs(pp.x - target.x));
        // Past the car's far side = walked through it.
        if (pp.x > target.x + 1.0) passedThrough = true;
    }
    std::printf("    [walk] closestApproach=%.2f m passedThrough=%d\n",
                closest, passedThrough ? 1 : 0);
    CHECK(!passedThrough);

    physics.shutdown();
}

// WHAT THE PLAYER SEES OF TRAFFIC (Glenn, 2026-09-18: "When the AI agent cars
// are driving they're floating off the road. The wheels seem sunk into the
// body", "the bus is sunk into the ground", "the vehicle will start to turn but
// then jump back or teleport and then do a sudden pivot on a dime", and "the
// wheels have to be on the road"). Measured on the DRAWN transforms, with each
// slot's own wheel layout -- the thing on screen, not the sim's idea of it:
//   wheels: every wheel's lowest point against the road (flat, y = 0 here)
//   jumps:  a drawn car moving further in one step than its speed allows
//   pivots: its heading swinging faster than a car can turn at that speed
//   backs:  a step that moves it backwards while it is driving forwards
namespace {
struct TrafficLook {
    long wheelSamples = 0;
    Real wheelMin = 1e9, wheelMax = -1e9, wheelAbsSum = 0;
    long steps = 0, jumps = 0, pivots = 0, backs = 0, reversingOut = 0;
    Real worstJump = 0;
};
TrafficLook watchTraffic(bool physicalTier) {
    World world;
    world.add<RoadEntity>(world.create(), cityGrid());
    CityRenderParams params;
    params.cars = 24;
    params.pedestrians = 0;
    params.seed = 7;
    params.wander = true;
    params.physicalCars = physicalTier ? 12 : 0;
    params.vehicleScript = readAsset("vehicles.lua");
    CityRenderSystem city(params);
    StubUploader uploader;
    engine::AssetManager assets(uploader);
    CHECK(city.build(world, &assets));
    Entity player = world.create();
    Transform pt;
    pt.position = Vec3(120, 0.9, 120);
    world.add<Transform>(player, pt);
    world.add<CharacterController>(player, CharacterController{});
    PhysicsSystem physics;
    physics.initialize();
    physics.physicsWorld().addBox(Vec3(600, 1, 600), Vec3(120, -1, 120),
                                  Quat::identity(), BodyMotion::Static);
    CityPhysicsSystem bridge(city, physics);

    TrafficLook t;
    const Real dt = 1.0 / 60.0;
    struct Last { Vec3 p; Real yaw; };
    std::unordered_map<int, Last> last;
    for (int i = 0; i < 60 * 40; ++i) {
        city.step(world, dt);
        bridge.step(world, dt);   // proxies run either way, as in the game
        physics.step(world, dt);
        if (i < 60 * 3) continue;   // let the tier settle
        std::unordered_map<int, Last> now;
        const auto& ids = city.carAgentIds();
        for (std::size_t v = 0; v < city.carGroups().size(); ++v) {
            InstanceGroup* g = world.get<InstanceGroup>(city.carGroups()[v]);
            if (!g || v >= ids.size()) continue;
            const auto& wheels = city.carWheels(static_cast<int>(v));
            for (std::size_t k = 0; k < g->transforms.size() && k < ids[v].size(); ++k) {
                const int ai = ids[v][k];
                if (ai < 0) continue;
                const Agent& a = city.sim().agents()[static_cast<std::size_t>(ai)];
                const Mat4& m = g->transforms[k];
                for (const auto& w : wheels) {
                    const Vec3 bottom = m.transformPoint(w.pos - Vec3(0, w.radius, 0));
                    t.wheelMin = std::min(t.wheelMin, bottom.y);
                    t.wheelMax = std::max(t.wheelMax, bottom.y);
                    t.wheelAbsSum += std::fabs(bottom.y);
                    ++t.wheelSamples;
                }
                const Vec3 p(m.m[0][3], m.m[1][3], m.m[2][3]);
                const Real yaw = std::atan2(m.m[0][2], m.m[2][2]);
                now[ai] = {p, yaw};
                const auto it = last.find(ai);
                if (it == last.end() || !a.moving) continue;
                ++t.steps;
                const Vec3 d = p - it->second.p;
                const Real disp = std::sqrt(d.x * d.x + d.z * d.z);
                const Real allowed = std::max(a.speed, Real(2)) * dt * 2.0 + 0.05;
                if (disp > allowed) {
                    ++t.jumps;
                    t.worstJump = std::max(t.worstJump, disp);
                }
                Real dy = yaw - it->second.yaw;
                while (dy > PI) dy -= 2 * PI;
                while (dy < -PI) dy += 2 * PI;
                // A car's yaw rate is bounded by speed / min turn radius (~5 m)
                // plus a margin; "on a dime" is turning with no speed to turn.
                if (std::fabs(dy) / dt > a.speed / 4.0 + 0.8) ++t.pivots;
                const Real fwdMove = d.x * std::sin(yaw) + d.z * std::cos(yaw);
                // Moving against its drawn nose while still merging out of a
                // space is REVERSING OUT (a car parked facing the other way
                // turning round); anywhere else it is a bug.
                if (a.speed > 1.0 && fwdMove < -0.02) {
                    if (a.pullLen > 0) ++t.reversingOut;
                    else ++t.backs;
                }
            }
        }
        last.swap(now);
    }
    return t;
}
void printLook(const char* label, const TrafficLook& t) {
    std::printf("    [look] %-9s wheels: %ld samples, lowest %+.2f m, highest %+.2f m, "
                "mean |gap| %.3f m | %ld steps: %ld jumps (worst %.2f m), %ld pivots, "
                "%ld backwards (+%ld reversing out of a space)\n",
                label, t.wheelSamples, t.wheelMin, t.wheelMax,
                t.wheelSamples ? t.wheelAbsSum / t.wheelSamples : 0.0, t.steps, t.jumps,
                t.worstJump, t.pivots, t.backs, t.reversingOut);
}
}  // namespace

TEST_CASE(city_drawn_traffic_rolls_on_its_wheels) {
    // Both, printed side by side: the physical tier is opt-in now
    // (CitySimConfig::physicalCars), and this is the measurement that made it
    // so -- measured 2026-09-18 it drew wheels from 0.57 m into the road to
    // 0.36 m above it, with 26 pivots and snaps of up to 12 m, while the sim's
    // own motion put every wheel on the deck.
    const TrafficLook phys = watchTraffic(true);
    const TrafficLook kin = watchTraffic(false);
    printLook("physical", phys);
    printLook("kinematic", kin);
    // THE SHIPPING DEFAULT. Flat ground: every wheel of every car at one
    // height, on the road deck (the deck stands a few cm proud of y = 0).
    CHECK(kin.wheelSamples > 10000);
    CHECK(kin.wheelMax - kin.wheelMin < 0.05);
    CHECK(kin.wheelMin > -0.02 && kin.wheelMax < 0.15);
    // Continuous motion: no pivots, no teleports (a pull-out used to redraw
    // the car in its lane 5-10 m away in one frame), nothing driving
    // backwards except a car turning round as it leaves a space.
    CHECK(kin.pivots == 0);
    CHECK(kin.worstJump < 0.5);
    CHECK(kin.backs <= 5);
}

// EVERY LIT LAMP IS ON ITS OWN CAR'S NOSE OR TAIL (Glenn, 2026-09-18: "when I'm
// in the bus the tail lights show up inside of the bus"). The lamp pass chose a
// car's markers by vehicle % fleet while the body was drawn from its own slot,
// so a bus wore a sedan's lamps -- tail lights 2.3 m behind its centre, inside
// the saloon -- and every other car another model's. Checked on the drawn
// instances: each lit lens, in the frame of the nearest drawn body, sits at
// that body's own end (|z| ~ half its length).
TEST_CASE(city_lamps_sit_on_their_own_car_s_ends) {
    World world;
    world.add<RoadEntity>(world.create(), cityGrid());
    CityRenderParams params;
    params.cars = 24;
    params.pedestrians = 0;
    params.seed = 7;
    params.wander = true;
    params.busRoutes = 1;
    params.busStops = 6;
    params.buses = 2;
    params.busMaxWalk = 200;
    params.vehicleScript = readAsset("vehicles.lua");
    CityRenderSystem city(params);
    StubUploader uploader;
    engine::AssetManager assets(uploader);
    CHECK(city.build(world, &assets));
    const std::vector<Vec3> he = city.carGroupHalfExtents();
    long lamps = 0, misplaced = 0;
    Real worst = 0;
    for (int i = 0; i < 60 * 40; ++i) {
        city.step(world, 1.0 / 60.0);
        if (i % 15) continue;
        for (Entity lg : {city.brakeLightGroup(), city.turnSignalGroup(), city.headlightGroup()}) {
            const InstanceGroup* g = world.get<InstanceGroup>(lg);
            if (!g) continue;
            for (const Mat4& lm : g->transforms) {
                const Vec3 lp(lm.m[0][3], lm.m[1][3], lm.m[2][3]);
                // The body this lens belongs to: one whose box (a little
                // grown) contains it, at an end. "Nearest centre" misassigns
                // a bus's tail light (5.7 m out) to the car queued behind it.
                Real off = 1e9;
                for (std::size_t v = 0; v < city.carGroups().size(); ++v) {
                    const InstanceGroup* cg = world.get<InstanceGroup>(city.carGroups()[v]);
                    if (!cg || v >= he.size()) continue;
                    for (const Mat4& cm : cg->transforms) {
                        const Vec3 c(cm.m[0][3], cm.m[1][3], cm.m[2][3]);
                        if ((c - lp).length() > he[v].z + 2.0) continue;
                        const Vec3 local = cm.inverse().transformPoint(lp);
                        if (std::fabs(local.x) > he[v].x + 0.3 ||
                            std::fabs(local.y) > he[v].y + 0.5 ||
                            std::fabs(local.z) > he[v].z + 0.3)
                            continue;
                        off = std::min(off, std::fabs(std::fabs(local.z) - he[v].z));
                    }
                }
                ++lamps;
                worst = std::max(worst, std::min(off, Real(99)));
                if (off > 0.25) ++misplaced;
            }
        }
    }
    std::printf("    [lamps] %ld lit lamps checked; %ld not at their own car's end (worst %.2f m)\n",
                lamps, misplaced, worst);
    CHECK(lamps > 50);
    CHECK(misplaced == 0);
}

// #23 JUNCTION PILE-UPS (QA: "cars grinding against each other in packed intersections ... piled upon other
// cars ... tipped over ... then somehow the cars become non-physics objects and they can go through each
// other"). An OVERSATURATED grid -- four times the soak's traffic -- with the player at an interior crossing,
// so the physical tier's bodies meet queues at every mouth. Measured per possessed body per tick:
//   tip    -- up vector's y (a car on its side reads ~0)
//   climb  -- chassis centre height above its resting height on the flat ground (on another car's roof: > 1 m)
//   ghost  -- the deepest 2D footprint overlap with any other car, body or kinematic box ("go through each other")
namespace {
struct Foot { Vec2 c, ax, az; Vec2 he; };   // an oriented footprint: centre, unit axes, half extents
Foot footOf(const Vec3& p, const Quat& q, Real hx, Real hz) {
    const Vec3 x = q.rotate(Vec3(1, 0, 0)), z = q.rotate(Vec3(0, 0, 1));
    auto unit = [](Real a, Real b) { const Real l = std::sqrt(a * a + b * b); return l > 1e-6 ? Vec2(a / l, b / l) : Vec2(1, 0); };
    return Foot{Vec2(p.x, p.z), unit(x.x, x.z), unit(z.x, z.z), Vec2(hx, hz)};
}
// Separating-axis penetration depth of two oriented rectangles (0 when apart).
Real footOverlap(const Foot& a, const Foot& b) {
    Real depth = 1e30;
    const Vec2 d = b.c - a.c;
    for (const Vec2& n : {a.ax, a.az, b.ax, b.az}) {
        auto r = [&](const Foot& f) { return f.he.x * std::fabs(f.ax.x * n.x + f.ax.y * n.y) + f.he.y * std::fabs(f.az.x * n.x + f.az.y * n.y); };
        const Real gap = r(a) + r(b) - std::fabs(d.x * n.x + d.y * n.y);
        if (gap <= 0) return 0;
        depth = std::min(depth, gap);
    }
    return depth;
}
}  // namespace

TEST_CASE(city_phys_tier_packed_junction_does_not_pile_up) {
    World world;
    world.add<RoadEntity>(world.create(), cityGrid());
    CityRenderParams params;
    params.cars = 160;
    params.pedestrians = 0;
    params.seed = 11;
    params.wander = true;
    params.physicalCars = 12;
    params.vehicleScript = readAsset("vehicles.lua");   // the drawn fleet: without it there are no kinematic cars to meet
    if (const char* s = std::getenv("RT_PILEUP_SEED")) params.seed = static_cast<uint32_t>(std::atoi(s));
    CityRenderSystem city(params);
    StubUploader uploader;
    engine::AssetManager assets(uploader);
    CHECK(city.build(world, &assets));

    Entity player = world.create();
    Transform pt;
    pt.position = Vec3(80, 0.9, 80);   // an interior signalled crossing
    world.add<Transform>(player, pt);
    world.add<CharacterController>(player, CharacterController{});

    PhysicsSystem physics;
    physics.initialize();
    physics.physicsWorld().addBox(Vec3(600, 1, 600), Vec3(120, -1, 120), Quat::identity(), BodyMotion::Static);
    CityPhysicsSystem bridge(city, physics);
    PhysicsWorld& pw = physics.physicsWorld();

    const Real dt = 1.0 / 60.0;
    const int ticks = 60 * 90;
    Real minUp = 1.0, restY = 0, worstClimb = 0, worstGhost = 0;
    int restN = 0;
    long bodyTicks = 0, tipTicks = 0, climbTicks = 0, ghostTicks = 0;
    std::unordered_map<int, Real> age;
    std::unordered_map<int, Vec3> lastPos;          // body positions last tick: a jump is a snap
    std::unordered_map<int, char> inEpisode;        // body currently > 0.3 m into something
    std::unordered_map<int, std::pair<int, Real>> episodeStart;   // class, depth
    Real worstBoxBox = 0; long boxBoxPairs = 0, boxSamples = 0, boxSeen = 0;
    Real worstDiv = 0, worstSpell = 0; std::unordered_map<int, Real> spell;   // body vs its ghost
    int episodes[4] = {0, 0, 0, 0}; Real episodeDepth[4] = {0, 0, 0, 0};   // spawn, snap, box appeared, contact
    std::unordered_map<int, int> classOf;
    for (int i = 0; i < ticks; ++i) {
        city.step(world, dt);
        bridge.step(world, dt);
        physics.step(world, dt);
        // every other car's footprint: the other bodies, and the kinematic boxes that are not parked below the world
        std::vector<std::pair<int, Foot>> feet;   // (agent or -1, footprint)
        std::vector<std::pair<int, Foot>> traffic;   // kinematic boxes of MOVING drivers (not parked cars), with the agent
        std::unordered_map<long long, int> byUid;
        for (std::size_t q = 0; q < city.sim().agents().size(); ++q) byUid[static_cast<long long>(city.sim().agents()[q].uid)] = static_cast<int>(q);
        for (const auto& p : bridge.possessed())
            feet.push_back({p.agent, footOf(pw.vehiclePosition(p.vid), pw.vehicleOrientation(p.vid), 0.92, 2.15)});
        for (const auto& box : bridge.carProxyBoxes()) {
            const Vec3 bp = pw.bodyPosition(box.id);
            if (bp.y < -100) continue;
            feet.push_back({-1, footOf(bp, pw.bodyOrientation(box.id), box.he.x, box.he.z)});
            if (auto u = byUid.find(box.key); u != byUid.end()) {
                const Agent& ag = city.sim().agents()[u->second];
                if (ag.mode == Agent::Mode::Driver && ag.moving) traffic.push_back({u->second, feet.back().second});
            }
        }
        // Kinematic cars inside each other (the sim's own wrecks; boxes don't collide with boxes), near the player.
        if (i % 30 == 0) {
            std::vector<Foot> boxes; std::vector<int> boxAgent;
            // On the carriageway only (the grid's roads run along multiples of 80 m, 10 m wide): this synthetic
            // grid has no lots, so cars at home all park on one spot beside the road -- stacked, but not traffic.
            auto onRoad = [](Real v) { const Real m = std::fmod(std::fabs(v) + 40.0, 80.0) - 40.0; return std::fabs(m) < 5.0; };
            for (const auto& [ag, f] : traffic)
                if ((f.c - Vec2(80, 80)).length() < 90 && (onRoad(f.c.x) || onRoad(f.c.y))) { boxes.push_back(f); boxAgent.push_back(ag); }
            for (std::size_t x = 0; x < boxes.size(); ++x)
                for (std::size_t y = x + 1; y < boxes.size(); ++y)
                    if ((boxes[x].c - boxes[y].c).length() < 6) {
                        const Real o = footOverlap(boxes[x], boxes[y]);
                        worstBoxBox = std::max(worstBoxBox, o);
                        if (o > 0.3) ++boxBoxPairs;
                        static const bool logBox = std::getenv("RT_PILEUP_BOXLOG") != nullptr;
                        if (logBox && o > 0.8) {
                            const Real dp = std::fabs(boxes[x].az.x * boxes[y].az.x + boxes[x].az.y * boxes[y].az.y);
                            const Agent& A = city.sim().agents()[boxAgent[x]]; const Agent& B = city.sim().agents()[boxAgent[y]];
                            std::printf("      crashCount %d/%d crashTimer %.1f/%.1f speed %.1f/%.1f |", A.crashCount, B.crashCount, A.crashTimer, B.crashTimer, A.speed, B.speed);
                            std::printf("      box-box t=%.1f (%.1f,%.1f) len %.1f w %.1f | (%.1f,%.1f) len %.1f w %.1f | |cos| %.2f centres %.1f m overlap %.2f\n",
                                        i * dt, boxes[x].c.x, boxes[x].c.y, 2 * boxes[x].he.y, 2 * boxes[x].he.x, boxes[y].c.x, boxes[y].c.y,
                                        2 * boxes[y].he.y, 2 * boxes[y].he.x, dp, (boxes[x].c - boxes[y].c).length(), o);
                        }
                    }
            ++boxSamples; boxSeen += static_cast<long>(boxes.size());
        }
        std::unordered_map<int, Real> next;
        for (const auto& p : bridge.possessed()) {
            const Real a = age.count(p.agent) ? age[p.agent] + dt : 0.0;
            next[p.agent] = a;
            if (a < 1.0) continue;   // spawn settle
            const Vec3 bp = pw.vehiclePosition(p.vid);
            const Real up = upDot(pw.vehicleOrientation(p.vid));
            ++bodyTicks;
            minUp = std::min(minUp, up);
            if (up < 0.8) ++tipTicks;
            if (up > 0.98 && restN < 2000) { restY += bp.y; ++restN; }
            const Real rest = restN ? restY / restN : bp.y;
            const Real climb = bp.y - rest;
            worstClimb = std::max(worstClimb, climb);
            if (climb > 0.6) ++climbTicks;
            { const Real dv = (city.sim().agents()[p.agent].pos - Vec2(bp.x, bp.z)).length();
              worstDiv = std::max(worstDiv, dv);
              Real& sp = spell[p.agent]; sp = dv > 6.0 ? sp + dt : 0; worstSpell = std::max(worstSpell, sp); }
            const Foot me = footOf(bp, pw.vehicleOrientation(p.vid), 0.92, 2.15);
            Real deepest = 0; int partner = -2; Vec2 partnerC;
            for (const auto& [ag, f] : feet) if (ag != p.agent) { const Real o = footOverlap(me, f); if (o > deepest) { deepest = o; partner = ag; partnerC = f.c; } }
            static const bool logDeep = std::getenv("RT_PILEUP_LOG") != nullptr;
            if (logDeep && deepest > 0.8) {
                const auto& ags = city.sim().agents();
                int near = -1; Real nd = 1e9;   // a kinematic box: whose ghost is it?
                if (partner == -1) for (std::size_t k = 0; k < ags.size(); ++k) { const Real dd = (ags[k].pos - partnerC).length(); if (dd < nd) { nd = dd; near = static_cast<int>(k); } }
                const Agent& me2 = ags[p.agent];
                { const Vec3 fz = pw.vehicleOrientation(p.vid).rotate(Vec3(0, 0, 1)); std::printf("      yaw %.0f y %.2f up %.3f |", std::atan2(fz.x, fz.z) * 57.3, bp.y, up); }
                if (partner >= 0) for (const auto& q : bridge.possessed()) if (q.agent == partner) { const Vec3 fz = pw.vehicleOrientation(q.vid).rotate(Vec3(0, 0, 1)); std::printf(" other yaw %.0f y %.2f |", std::atan2(fz.x, fz.z) * 57.3, pw.vehiclePosition(q.vid).y); }
                std::printf("      t=%.2f a%d body(%.1f,%.1f) ghost gap %.1f v=%.1f stuck %.2f crash %.2f | %s %d (%.1f,%.1f)%s v=%.1f crash %.2f | overlap %.2f\n",
                            i * dt, p.agent, bp.x, bp.z, (me2.pos - Vec2(bp.x, bp.z)).length(), me2.speed, p.stuckTime, me2.crashTimer,
                            partner == -1 ? "box" : "body", partner == -1 ? near : partner, partnerC.x, partnerC.y,
                            partner == -1 ? (" ghost-off " + std::to_string(nd).substr(0, 4)).c_str() : "",
                            (partner == -1 ? near : partner) >= 0 ? ags[partner == -1 ? near : partner].speed : 0.0,
                            (partner == -1 ? near : partner) >= 0 ? ags[partner == -1 ? near : partner].crashTimer : 0.0, deepest);
            }
            worstGhost = std::max(worstGhost, deepest);
            if (deepest > 0.3) ++ghostTicks;
        }
        // Episodes: how each deep overlap BEGAN. Spawn = a body younger than a tick's worth of acquisition
        // (either party), snap = a body jumped > 2 m this tick, contact = everything else (driven into it).
        for (const auto& p : bridge.possessed()) {
            const Vec3 bp = pw.vehiclePosition(p.vid);
            const Foot me = footOf(bp, pw.vehicleOrientation(p.vid), 0.92, 2.15);
            Real deepest = 0; int partner = -2;
            for (const auto& [ag, f] : feet) if (ag != p.agent) { const Real o = footOverlap(me, f); if (o > deepest) { deepest = o; partner = ag; } }
            const bool jumped = lastPos.count(p.agent) && (Vec2(bp.x, bp.z) - Vec2(lastPos[p.agent].x, lastPos[p.agent].z)).length() > 2.0;
            const bool partnerJumped = partner >= 0 && lastPos.count(partner) && [&]() { for (const auto& q : bridge.possessed()) if (q.agent == partner) { const Vec3 qp = pw.vehiclePosition(q.vid); return (Vec2(qp.x, qp.z) - Vec2(lastPos[partner].x, lastPos[partner].z)).length() > 2.0; } return false; }();
            const bool young = !lastPos.count(p.agent) || (partner >= 0 && !lastPos.count(partner));
            if (deepest > 0.3 && !inEpisode[p.agent]) {
                const int cls = young ? 0 : (jumped || partnerJumped) ? 1 : 3;
                ++episodes[cls]; classOf[p.agent] = cls; inEpisode[p.agent] = 1;
                if (cls == 3) {
                    const Agent& g = city.sim().agents()[p.agent];
                    std::printf("      contact episode t=%.1f a%d body (%.1f,%.1f,%.2f) ghost (%.1f,%.1f) v %.1f pull %.1f/%.1f crash %.1f | partner %s\n",
                                i * dt, p.agent, bp.x, bp.z, bp.y, g.pos.x, g.pos.y, g.speed, g.pullS, g.pullLen, g.crashTimer, partner >= 0 ? "body" : "box");
                }
            }
            if (deepest > 0.3) { int c = classOf[p.agent]; episodeDepth[c] = std::max(episodeDepth[c], deepest); }
            if (deepest < 0.1) inEpisode[p.agent] = 0;
        }
        lastPos.clear();
        for (const auto& p : bridge.possessed()) lastPos[p.agent] = pw.vehiclePosition(p.vid);
        age.swap(next);
    }
    const Real per = bodyTicks > 0 ? 1.0 / Real(bodyTicks) : 0.0;
    std::printf("    [pileup] seed %u: body-ticks %ld; minUp %.3f, tipped %.4f; worst climb %.2f m, climbing %.4f; "
                "worst overlap %.2f m, overlapping >0.3 m %.4f; snaps %d; sim crash events %d\n",
                params.seed, bodyTicks, minUp, tipTicks * per, worstClimb, climbTicks * per, worstGhost, ghostTicks * per,
                bridge.snapCount(), city.sim().crashEvents());
    std::printf("    [pileup] episodes (> 0.3 m) by how they began: spawn %d (deepest %.2f), snap %d (%.2f), contact %d (%.2f)\n",
                episodes[0], episodeDepth[0], episodes[1], episodeDepth[1], episodes[3], episodeDepth[3]);
    std::printf("    [pileup] snaps held back because the landing spot was taken: %d; body-to-ghost worst %.1f m, longest > 6 m %.1f s\n",
                bridge.snapsHeld(), worstDiv, worstSpell);
    std::printf("    [pileup] kinematic cars inside each other (within 90 m, twice a second): worst %.2f m, %.2f pairs > 0.3 m per sample (%.0f boxes per sample)\n",
                worstBoxBox, boxSamples ? Real(boxBoxPairs) / boxSamples : 0.0, boxSamples ? Real(boxSeen) / boxSamples : 0.0);
    CHECK(bodyTicks > 60 * 60 * 4);   // the tier really engaged at the crossing
    // Rates, not worst cases: one car in a 90 s run can still end up on its side (a turning body clipped at a
    // T-junction), and single seeds swing. Measured over seeds 11-18, drawn fleet loaded:
    //   before (#23 as filed)  body-ticks overlapping > 0.3 m up to 0.092, climbing up to 0.092, tipped up to 0.090
    //                          (every deep overlap began with a SNAP dropping a chassis into a car already there)
    //   after                  <= 0.0014, <= 0.0016, <= 0.0016
    // The kinematic (sim-only) overlaps are reported, not gated: what is left there is the sim's escape valve
    // (cars wedged through 5+ freezes pass through each other so the junction clears) -- a design call.
    CHECK(ghostTicks * per < 0.005);   // cars inside other cars
    CHECK(climbTicks * per < 0.005);   // cars on other cars' roofs
    CHECK(tipTicks * per < 0.005);     // cars on their sides
    physics.shutdown();
}
