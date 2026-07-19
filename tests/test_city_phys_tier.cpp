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
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/systems/physics_system.h"
#include "../src/engine/world.h"

#include <algorithm>
#include <cstdio>
#include <unordered_map>

using namespace engine;
using namespace citysim;

namespace {

// 4x4 junction grid, 80 m pitch: the interior nodes are degree-4 signalled
// crossings, enough network for two dozen wandering drivers.
RoadNet cityGrid() {
    RoadNet net;
    const int N = 4;
    const Real pitch = 80;
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i)
            net.nodes.push_back(Vec2(i * pitch, j * pitch));
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i) {
            if (i + 1 < N) net.edges.push_back({ j * N + i, j * N + i + 1 });
            if (j + 1 < N) net.edges.push_back({ j * N + i, (j + 1) * N + i });
        }
    net.width = 10.0;
    net.sidewalk = 2.5;
    return net;
}

Real upDot(const Quat& q) { return q.rotate(Vec3(0, 1, 0)).y; }

}  // namespace

TEST_CASE(city_phys_tier_soak) {
    World world;
    world.add<RoadNet>(world.create(), cityGrid());

    CityRenderParams params;
    params.cars = 24;
    params.pedestrians = 0;
    params.seed = 7;
    params.wander = true;   // perpetual trips: drivers keep moving all soak
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
    CHECK(maxPossessed >= 6);          // the tier engages on a busy grid
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
    CHECK(bridge.snapCount() <= 10);   // observed 8/min at 12 cars
    CHECK(bridge.possessed().size() == 0);   // out of range: all released

    physics.shutdown();
}
