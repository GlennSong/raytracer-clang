// The building prism collider (ADR-0080): the walk-through gate. A character
// walks from the street THROUGH the door notch onto the floor cap and stands
// on the floor it can SEE (baseY + 0.05, not the pad half a metre below);
// walking at the wall beside the door stops outside; from inside, a wall
// edge with no door never lets it out. This is the test the extraction from
// level_loader exists for — the inline prism emission could never be walked.
#include "test_framework.h"
#include "../src/engine/physics/physics_world.h"
#include "../src/engine/procgen/city/building_collider.h"
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/shape_grammar.h"

#include <cmath>
#include <vector>

using namespace engine;

namespace {

// Mirrors PlayerSystem::fixedUpdate (same helper as test_physics.cpp).
void walk(PhysicsWorld& world, CharacterId c, const Vec3& vel, int frames) {
    for (int i = 0; i < frames; i++) world.moveCharacter(c, vel, 1.0 / 60.0);
}

// The lab: flat ground with its top at y=0; a 20 x 15 m building with plinth
// 0.45 (wall base baseY=0.45, drawn floor 0.5), one 2.0 x 2.7 door centred
// on the z=0 edge facing -Z (the street side). Door spans x in [9, 11].
void buildLab(PhysicsWorld& world) {
    world.addBox(Vec3(60, 0.5, 60), Vec3(0, -0.5, 0), Quat::identity(),
                 BodyMotion::Static);   // half-extents FIRST, then position
    const Poly2 plan = {{0, 0}, {20, 0}, {20, 15}, {0, 15}};
    const std::vector<DoorSpec> doors = {{Vec2(10, 0), Vec2(0, -1), 2.0, 2.7}};
    std::vector<Vec3> V;
    std::vector<uint32_t> I;
    appendBuildingPrism(V, I, plan, /*base*/ -0.05, /*top*/ 12.0,
                        /*floorY*/ 0.5, doors, /*plinth*/ 0.45);
    mirrorTriangles(I);
    world.addMesh(V, I, Vec3(), 0.85);
}

}  // namespace

TEST_CASE(character_walks_through_the_door_onto_the_floor) {
    PhysicsWorld world;
    world.initialize();
    buildLab(world);
    // Player-sized capsule (halfHeight 0.8, radius 0.3): centre rests 1.1
    // above whatever it stands on. Start 4 m outside the door.
    CharacterId c = world.addCharacter(0.8, 0.3, Vec3(10, 1.2, -4));
    CHECK(c != INVALID_CHARACTER);
    world.optimizeBroadPhase();
    walk(world, c, Vec3(), 60);   // settle on the street
    CHECK_APPROX(world.characterPosition(c).y, 1.1, 0.1);
    // Stroll straight through the doorway (threshold step, then the floor).
    walk(world, c, Vec3(0, 0, 1.6), 420);
    const Vec3 p = world.characterPosition(c);
    CHECK(p.z > 1.5);                 // inside, past the wall plane
    CHECK_APPROX(p.y, 1.6, 0.15);     // standing ON the drawn floor (0.5+1.1)
    world.shutdown();
}

TEST_CASE(walls_beside_the_door_still_block) {
    PhysicsWorld world;
    world.initialize();
    buildLab(world);
    for (const Real x : {Real(7.0), Real(13.0)}) {
        CharacterId c = world.addCharacter(0.8, 0.3, Vec3(x, 1.2, -4));
        world.optimizeBroadPhase();
        walk(world, c, Vec3(), 60);
        walk(world, c, Vec3(0, 0, 1.6), 420);
        const Vec3 p = world.characterPosition(c);
        CHECK(p.z < -0.2);            // stopped at the facade, outside
        CHECK(p.y < 1.4);             // and did NOT climb onto anything
    }
    world.shutdown();
}

TEST_CASE(no_exit_through_a_wall_from_inside) {
    PhysicsWorld world;
    world.initialize();
    buildLab(world);
    // Spawn ON the floor inside, walk at the rear (z=15) edge — no door there.
    CharacterId c = world.addCharacter(0.8, 0.3, Vec3(10, 1.7, 10));
    world.optimizeBroadPhase();
    walk(world, c, Vec3(), 60);
    CHECK_APPROX(world.characterPosition(c).y, 1.6, 0.15);  // on the floor cap
    walk(world, c, Vec3(0, 0, 1.6), 420);
    // The capsule (radius 0.3) rests with its centre a radius short of the
    // wall plane: ~14.7. Anything past the plane means it escaped.
    CHECK(world.characterPosition(c).z < 14.9);   // still inside
    // And at a side wall, for a second bearing (wall at x=20 -> rest ~19.7).
    walk(world, c, Vec3(1.6, 0, 0), 420);
    CHECK(world.characterPosition(c).x < 19.9);
    world.shutdown();
}

// THE TIERED COLLIDER (architectural toolkit, issue #105, step 1): a setback tower collides tier by tier, each
// deck capped where the grammar draws it, the top deck cut through over the core's stairs; a plain tower is
// one tier capped at its roof deck (5 cm over the last ceiling), not at its parapet's top.
TEST_CASE(tiered_collider_caps_every_deck_and_opens_over_the_stairs) {
    BuildingParams p;
    p.floors = 12;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 5;
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    {
        const std::vector<ColliderTier> one = colliderTiers(plan, p);
        CHECK(one.size() == 1);
        const std::vector<StoreyPlan> sps = storeyPlans(plan, p);
        CHECK(std::fabs(one.front().y1 - (sps.back().y0 + sps.back().h + 0.05)) < 1e-9);
        CHECK(std::fabs(one.front().parapet - p.parapet) < 1e-9);
    }
    BuildingParams s = p;
    s.setbackFloors = 4;
    s.setbackEvery = 2.0;
    const std::vector<ColliderTier> tiers = colliderTiers(plan, s);
    CHECK(tiers.size() >= 2);
    for (std::size_t i = 1; i < tiers.size(); ++i) {
        CHECK(tiers[i].y0 > tiers[i - 1].y0);
        CHECK(std::fabs(tiers[i].y0 - tiers[i - 1].y1) < 1e-9);
        CHECK(area(tiers[i].plan) < area(tiers[i - 1].plan));
    }
    // The collider: a vertex on every deck, and the top deck's cap misses the stair shafts' centres.
    const CorePlan core = coreFor(plan, s, entranceEdgeFor(plan, s));
    CHECK(core.valid);
    std::vector<Vec3> V;
    std::vector<uint32_t> I;
    const Real baseY = 3.0;
    CHECK(appendBuildingTiers(V, I, plan, s, baseY, baseY - 0.2, {}, coreRoofHoles(core)));
    for (const ColliderTier& t : tiers) {
        bool onDeck = false;
        for (const Vec3& v : V) if (std::fabs(v.y - (baseY + t.y1)) < 1e-6) onDeck = true;
        CHECK(onDeck);
    }
    const Real topY = baseY + tiers.back().y1;
    for (const CoreStair& st : core.stairs) {
        const Vec2 c = st.shaft.frame.toWorld({st.shaft.width * 0.5, st.shaft.depth * 0.5});
        bool covered = false;
        for (std::size_t i = 0; i + 2 < I.size(); i += 3) {
            const Vec3& a = V[I[i]]; const Vec3& b = V[I[i + 1]]; const Vec3& d = V[I[i + 2]];
            if (std::fabs(a.y - topY) > 1e-6 || std::fabs(b.y - topY) > 1e-6 || std::fabs(d.y - topY) > 1e-6) continue;
            const Poly2 tri = {{a.x, a.z}, {b.x, b.z}, {d.x, d.z}};
            if (pointInPolygon(tri, c)) covered = true;
        }
        CHECK(!covered);
    }
    // A pitched roof has no tiers: the caller keeps the one prism.
    BuildingParams g = p;
    g.roofStyle = BuildingParams::RoofStyle::Gable;
    CHECK(colliderTiers(plan, g).empty());
}
