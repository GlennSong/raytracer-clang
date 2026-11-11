// #24: CARS FALL THROUGH THE ROAD. The physical tier (Jolt wheeled cars chasing the sim's ghosts, ADR-0057) on a
// real lane-built city with grades, ramps and decks (metro_v2_test), the player's ring moved from one ramp or deck to
// the next (aimed at a busy street, the cars never left the ground: 0 car-ticks on a structure). Every tick, every
// physical car: its centre over the highest collider surface
// under it (no higher than its centre + 1 m, so a deck overhead is not the road it is on), and its fall speed.
// Gates: no car sinks into the road it is on (zero submarine ticks), none free-falls (a sustained drop no ramp
// makes), none goes under the level's lowest surface (the kill box).
#include "test_framework.h"

#include "../src/apps/citysim/city_physics.h"
#include "../src/apps/citysim/city_render.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/level_loader.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/systems/physics_system.h"
#include "../src/engine/world.h"
#include "../src/renderer/renderer.h"
#include "collider_grid.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

using namespace engine;
using namespace citysim;
using rt_test::ColliderGrid;

namespace {

std::string deckSoakLevel() {
    const char* only = std::getenv("RT_CITYSIM_LEVEL");
    const std::string name = only && *only ? std::string(only) : std::string("metro_v2_test");
    return std::string(RT_SOURCE_DIR) + "/assets/levels/" + name + ".json";
}

}  // namespace

TEST_CASE(city_physical_cars_never_fall_through_the_road) {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader{*renderer};
    AssetManager assets{uploader};
    World world;
    RenderView view;
    const bool loaded = LevelLoader::load(deckSoakLevel(), world, *renderer, view, assets, /*editorMode=*/false);
    CHECK(loaded);
    if (!loaded) return;

    // The tier's budget and perpetual trips (the level ships with the tier off: its viewer turns it on near the player).
    world.each<CitySimConfig>([](Entity, CitySimConfig& c) { c.physicalCars = 12; c.wander = true; });

    ColliderGrid surf;
    world.each<Transform, MeshCollider>([&](Entity, Transform& t, MeshCollider& mc) { surf.add(mc, t.position); });
    CHECK(!surf.tris.empty());

    // THE STRUCTURES: every point of a road chain that is a bridge (an authored deck or a layer above grade,
    // UnionSpine's own definition), in 8 m cells -- where #24's cars went under and the player's ring is aimed.
    struct DeckPt { Vec2 p; double y, hw; };
    std::vector<DeckPt> deckPts;
    std::unordered_map<long long, std::vector<int>> deckCells;
    world.each<RoadDeck>([&](Entity, RoadDeck& d) {
        for (const UnionSpine& sp : d.field.spines) {
            if (sp.use != UnionSpine::Use::Road || !(sp.authoredDeck || sp.layer > 0) || sp.yAbs.size() != sp.points.size()) continue;
            for (std::size_t i = 0; i < sp.points.size(); ++i) {
                deckCells[ColliderGrid::key(static_cast<int>(std::floor(sp.points[i].x / 8.0)), static_cast<int>(std::floor(sp.points[i].y / 8.0)))]
                    .push_back(static_cast<int>(deckPts.size()));
                deckPts.push_back({sp.points[i], sp.yAbs[i], sp.halfWidth});
            }
        }
    });
    CHECK(!deckPts.empty());
    if (deckPts.empty()) return;   // (the hops below pick a point modulo its count)
    auto onStructure = [&](const Vec3& c, double road) {
        const int cx = static_cast<int>(std::floor(c.x / 8.0)), cz = static_cast<int>(std::floor(c.z / 8.0));
        for (int dz = -1; dz <= 1; ++dz)
            for (int dx = -1; dx <= 1; ++dx) {
                const auto it = deckCells.find(ColliderGrid::key(cx + dx, cz + dz));
                if (it == deckCells.end()) continue;
                for (int k : it->second) {
                    const DeckPt& d = deckPts[k];
                    const double ex = c.x - d.p.x, ez = c.z - d.p.y;
                    if (ex * ex + ez * ez < (d.hw + 4.0) * (d.hw + 4.0) && std::fabs(road - d.y) < 1.0) return true;
                }
            }
        return false;
    };

    CityRenderSystem city;
    CHECK(city.build(world, &assets, nullptr));

    // The player: a bare marker the tier centres on (the level's own body would be stepped by its controller).
    world.each<ControlledBy>([&](Entity e, ControlledBy&) { world.remove<ControlledBy>(e); });
    const Entity player = world.create();
    world.add<Transform>(player, Transform{});
    world.add<ControlledBy>(player, ControlledBy{});

    PhysicsSystem physics;
    physics.initialize(nullptr);
    physics.createBodies(world);
    CityPhysicsSystem bridge(city, physics);

    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    const Real dt = 1.0 / 60.0;
    const int kHops = std::getenv("RT_DECK_SOAK_HOPS") ? std::atoi(std::getenv("RT_DECK_SOAK_HOPS")) : 6;
    const int kHopTicks = 60 * 20;   // 20 s at each spot
    long held = 0, sunk = 0, falling = 0, underWorld = 0, noSurface = 0;
    double worstSink = 0, worstFall = 0, minOver = 1e9;
    long onDeck = 0;   // car-ticks over a structure (another surface under the road they are on)
    Vec3 worstAt(0, 0, 0);
    std::unordered_map<int, double> fallTime;   // per vehicle: seconds dropping faster than any ramp
    std::unordered_map<int, double> age;
    std::size_t maxPossessed = 0;
    uint32_t pick = 12345;
    for (int hop = 0; hop < kHops; ++hop) {
        // The next spot: a structure point, by a fixed sequence (the same spots every run).
        pick = pick * 1664525u + 1013904223u;
        const DeckPt& at = deckPts[(pick >> 8) % deckPts.size()];
        world.get<Transform>(player)->position = Vec3(at.p.x, at.y, at.p.y);
        for (int i = 0; i < kHopTicks; ++i) {
            city.step(world, dt);
            bridge.step(world, dt);
            physics.step(world, dt);
            PhysicsWorld& pw = physics.physicsWorld();
            std::unordered_map<int, double> nextAge;
            for (const CityPhysicsSystem::Possessed& p : bridge.possessed()) {
                const double a = age.count(p.agent) ? age[p.agent] + dt : 0.0;
                nextAge[p.agent] = a;
                if (a < 1.0) continue;   // put down: settling onto its springs
                ++held;
                const Vec3 bp = pw.vehiclePosition(p.vid);
                const Vec3 vel = pw.vehicleVelocity(p.vid);
                if (bp.y < surf.lowest - 1.0) ++underWorld;
                double top = 0;
                if (!surf.surfaceAt(bp.x, bp.z, bp.y + 1.0, top)) { ++noSurface; continue; }
                // The centre over the road: spawnAbove when resting on its springs; under the road it is on means sunk.
                const double over = bp.y - top;
                minOver = std::min(minOver, over);
                if (onStructure(bp, top)) ++onDeck;
                if (over < 0.0) {
                    ++sunk;
                    if (-over > worstSink) { worstSink = -over; worstAt = bp; }
                }
                double& ft = fallTime[p.vid];
                ft = vel.y < -6.0 ? ft + dt : 0.0;   // the steepest ramp at 30 m/s drops ~3 m/s
                if (ft > 0.25) ++falling;
                worstFall = std::max(worstFall, ft);
            }
            age.swap(nextAge);
            maxPossessed = std::max(maxPossessed, bridge.possessed().size());
        }
    }
    std::printf("    [deck-soak] %d spots x 20 s: up to %zu physical cars, %ld car-ticks held; sunk %ld (worst %.2f m at (%.1f, %.1f, %.1f)), "
                "free-falling %ld (longest %.2f s), under the world %ld, no surface %ld; snaps %d; lowest centre over its road %.2f m; on a deck %ld; %.0f s\n",
                kHops, maxPossessed, held, sunk, worstSink, worstAt.x, worstAt.y, worstAt.z, falling, worstFall, underWorld, noSurface,
                bridge.snapCount(), minOver, onDeck, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    CHECK(maxPossessed >= 4);   // the tier engaged
    CHECK(held > 0);
    CHECK(onDeck > 0);   // and drove the structures, the question
    CHECK(sunk == 0);
    CHECK(falling == 0);
    CHECK(underWorld == 0);
}
