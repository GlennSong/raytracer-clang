// The streamed interior (ADR-0080 Phase 2): deterministic geometry, a stair a
// real character can climb — around the well, up the next flight — and the
// build-on-approach / release-on-leave lifecycle. Walked, not asserted.
#include "test_framework.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/physics/physics_world.h"
#include "../src/engine/procgen/city/building_records.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/systems/building_interior_system.h"
#include "../src/engine/systems/door_system.h"
#include "../src/engine/world.h"

#include <cmath>
#include <vector>

using namespace engine;

namespace {

struct StubUploader : MeshUploader {
    int uploads = 0;
    int removes = 0;
    uint32_t next = 1;
    MeshHandle uploadMesh(const RenderMesh&) override {
        uploads++;
        return MeshHandle{next++, 1};
    }
    void removeMesh(MeshHandle) override { removes++; }
    BoundingSphere getMeshBounds(MeshHandle) const override { return {}; }
};

void walk(PhysicsWorld& world, CharacterId c, const Vec3& vel, int frames) {
    for (int i = 0; i < frames; i++) world.moveCharacter(c, vel, 1.0 / 60.0);
}

BuildingParams interiorParams() {
    BuildingParams p;
    p.floors = 2;            // ground (4.5) + two upper storeys (3.2 each)
    p.groundRetail = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 77;
    return p;
}
// Entrance lands on the +Z edge (faceDir default), so the stair takes the
// z=0 edge: foot near (6.1, 1.05), climbing +X, run ~6.7 for the ground
// storey.
const Poly2 kPlan = {{0, 0}, {20, 0}, {20, 15}, {0, 15}};

}  // namespace

TEST_CASE(interior_is_deterministic_and_stays_inside_the_plan) {
    BuildingParams p = interiorParams();
    RenderMesh col1, col2;
    const BuildingMesh a = growInterior(kPlan, p, 0.0, &col1);
    const BuildingMesh b = growInterior(kPlan, p, 0.0, &col2);
    std::size_t va = 0, vb = 0;
    for (const RenderMesh& m : a.parts) va += m.vertices.size();
    for (const RenderMesh& m : b.parts) vb += m.vertices.size();
    CHECK(va > 0);
    CHECK(va == vb);
    CHECK(col1.vertices.size() == col2.vertices.size());
    // The streamed wood part carries AUTHORED, non-constant UVs (the lobby
    // overlay had this gate; the streamed slabs did not, and a flat-grey
    // second floor slipped through as a single clamped sample).
    for (const RenderMesh& m : a.parts) {
        if (m.materialIndex != static_cast<int>(PartId::InteriorFloor))
            continue;
        float minU = 1e9f, maxU = -1e9f, minV = 1e9f, maxV = -1e9f;
        for (const Vertex& v : m.vertices) {
            minU = std::min(minU, v.u); maxU = std::max(maxU, v.u);
            minV = std::min(minV, v.v); maxV = std::max(maxV, v.v);
        }
        std::printf("    [uv-spread] floor u [%f, %f] v [%f, %f]\n",
                    minU, maxU, minV, maxV);
        CHECK(maxU - minU > 1.0f);
        CHECK(maxV - minV > 1.0f);
    }
    // Everything stays inside the plan bbox (inner shell, stair, slabs).
    for (const RenderMesh& m : a.parts)
        for (const Vertex& v : m.vertices) {
            CHECK(v.position.x > -0.01);
            CHECK(v.position.x < 20.01);
            CHECK(v.position.z > -0.01);
            CHECK(v.position.z < 15.01);
        }
    // One slab level per storey above ground: 4.55 and 7.75.
    for (const Real level : {Real(4.55), Real(7.75)}) {
        bool found = false;
        for (const RenderMesh& m : a.parts)
            for (const Vertex& v : m.vertices)
                if (std::fabs(v.position.y - level) < 1e-6) found = true;
        CHECK(found);
    }
}

TEST_CASE(character_climbs_the_stair_around_the_well_to_the_top) {
    BuildingParams p = interiorParams();
    const InteriorLayout il =
        interiorLayout(kPlan, p, entranceEdgeFor(kPlan, p));
    CHECK(il.hasStair);
    RenderMesh collider;
    growInterior(kPlan, p, 0.0, &collider);
    CHECK(!collider.indices.empty());

    PhysicsWorld world;
    world.initialize();
    world.addBox(Vec3(60, 0.5, 60), Vec3(0, -0.5, 0), Quat::identity(),
                 BodyMotion::Static);   // the ground floor stand-in
    std::vector<Vec3> verts;
    verts.reserve(collider.vertices.size());
    for (const Vertex& v : collider.vertices) verts.push_back(v.position);
    std::vector<uint32_t> idx = collider.indices;
    const std::size_t oneSided = idx.size();
    for (std::size_t i = 0; i + 2 < oneSided; i += 3) {
        idx.push_back(idx[i]);
        idx.push_back(idx[i + 2]);
        idx.push_back(idx[i + 1]);
    }
    world.addMesh(verts, idx, Vec3(), 0.85);

    // All targets derive from the layout: foot, run and arrival move with
    // the stair pitch, so re-tuning the pitch never breaks the gate.
    const Real sz = il.stairFoot.y;
    const Real sx0 = il.stairFoot.x;              // flight foot
    const Real sxT = sx0 + il.run + 1.2;          // past the top tread
    CharacterId c = world.addCharacter(0.8, 0.3, Vec3(sx0 - 1.6, 1.2, sz));
    world.optimizeBroadPhase();
    walk(world, c, Vec3(), 60);
    CHECK_APPROX(world.characterPosition(c).y, 1.1, 0.15);
    // Steer toward a waypoint instead of dead reckoning: stairs cut walking
    // speed roughly in half (measured by the climb probe below), so blind
    // fixed-frame legs overshoot on floors and stop short on flights.
    auto walkTo = [&](Real tx, Real tz, int maxFrames) {
        for (int i = 0; i < maxFrames; ++i) {
            const Vec3 p = world.characterPosition(c);
            const Real dx = tx - p.x, dz = tz - p.z;
            const Real len = std::sqrt(dx * dx + dz * dz);
            if (len < 0.15) break;
            world.moveCharacter(
                c, Vec3(dx / len * 1.2, 0, dz / len * 1.2), 1.0 / 60.0);
        }
    };

    // Up flight 1 (ground -> storey 1 at 4.55), stopping just past the top.
    walkTo(sxT, sz, 1200);
    Vec3 pos = world.characterPosition(c);
    std::printf("    [climb] flight1 end x=%.2f y=%.2f z=%.2f\n", pos.x,
                pos.y, pos.z);
    CHECK(pos.x > sxT - 0.7);
    CHECK_APPROX(pos.y, 4.55 + 1.1, 0.25);

    // Around the well on the storey-1 floor (the hole spans x 8.2..12.9,
    // z 0.4..1.7): sidestep clear of it, walk back along it, return to the
    // stair line at the flight-2 foot. Never leaves the floor.
    walkTo(sxT, sz + 2.2, 400);
    walkTo(sx0 - 0.3, sz + 2.2, 1000);
    walkTo(sx0 - 0.3, sz, 400);
    pos = world.characterPosition(c);
    std::printf("    [climb] around-well x=%.2f y=%.2f z=%.2f\n", pos.x,
                pos.y, pos.z);
    CHECK(pos.x < sx0 + 0.4);
    CHECK_APPROX(pos.y, 4.55 + 1.1, 0.25);    // still ON the floor, not down

    // Up flight 2 (storey 1 -> storey 2 at 7.75).
    walkTo(sxT, sz, 1200);
    pos = world.characterPosition(c);
    std::printf("    [climb] flight2 end x=%.2f y=%.2f z=%.2f\n", pos.x,
                pos.y, pos.z);
    CHECK(pos.x > sxT - 0.7);
    CHECK_APPROX(pos.y, 7.75 + 1.1, 0.25);

    // And back down flight 2 to the storey-1 floor.
    walkTo(sx0 - 1.2, sz, 1400);
    pos = world.characterPosition(c);
    std::printf("    [climb] descent x=%.2f y=%.2f z=%.2f\n", pos.x, pos.y,
                pos.z);
    CHECK_APPROX(pos.y, 4.55 + 1.1, 0.35);
    world.shutdown();
}

TEST_CASE(interior_system_builds_on_approach_and_releases_on_leave) {
    World world;
    PhysicsWorld phys;
    phys.initialize();
    StubUploader uploader;
    AssetManager assets(uploader);

    CityBuildings cb;
    BuildingRecord r;
    r.plan = kPlan;
    r.baseY = 0;
    r.groundY = -0.45;
    r.height = 12.0;
    r.params = interiorParams();
    r.doors.push_back({Vec2(10, 15), Vec2(0, 1), 2.0, 2.7});
    r.enterable = true;
    r.recipe = "lab";
    r.type = "civic";
    r.district = "test";
    cb.records.push_back(r);
    cb.buildIndex();
    world.add<CityBuildings>(world.create(), std::move(cb));

    BuildingInteriorSystem sys(nullptr);
    const int baselineBodies = phys.bodyCount();

    // Far outside: nothing builds.
    sys.step(world, &phys, assets, nullptr, Vec3(10, 1, 80));
    CHECK(sys.residentCount() == 0);

    // Walking along the street BESIDE the building, out of the door's
    // 6 m reach: never builds. (Directly in FRONT of the door a passer-by
    // DOES build — that is the approach trigger working as designed.)
    sys.step(world, &phys, assets, nullptr, Vec3(1, 1, 16.5));
    CHECK(sys.residentCount() == 0);

    // At the door, in front: resident within one step.
    sys.step(world, &phys, assets, nullptr, Vec3(10, 1, 17.5));
    CHECK(sys.residentCount() == 1);
    CHECK(phys.bodyCount() == baselineBodies + 1);
    CHECK(uploader.uploads > 0);

    // Inside: stays.
    sys.step(world, &phys, assets, nullptr, Vec3(10, 1, 8));
    CHECK(sys.residentCount() == 1);

    // 45+ m from every door: released at once (body), mesh free rate-limited.
    sys.step(world, &phys, assets, nullptr, Vec3(10, 1, 65));
    CHECK(sys.residentCount() == 0);
    CHECK(phys.bodyCount() == baselineBodies);
    CHECK(uploader.removes == 0);   // queued, not yet freed
    for (int i = 0; i < 130; ++i)
        sys.step(world, &phys, assets, nullptr, Vec3(10, 1, 65));
    CHECK(uploader.removes > 0);    // the rate limiter let one through
    phys.shutdown();
}


TEST_CASE(jolt_mesh_cost_scales_sanely) {
    // ADR-0080 2c: the unowned number — what a MeshShape build costs at
    // interior sizes. Sanity gate is generous (< 50 us/tri); the printed
    // us/tri is the measurement. If an interior's addMesh ever exceeds ~4 ms
    // the build moves to ctx.jobs (the plan's stated threshold).
    PhysicsWorld world;
    world.initialize();
    for (const int quads : {117, 1167, 11667}) {
        std::vector<Vec3> verts;
        std::vector<uint32_t> idx;
        verts.reserve(static_cast<std::size_t>(quads) * 4);
        const int side = std::max(1, static_cast<int>(std::sqrt(quads)));
        int q = 0;
        for (int gz = 0; gz < side && q < quads; ++gz)
            for (int gx = 0; gx < side * 2 && q < quads; ++gx, ++q) {
                const uint32_t s0 = static_cast<uint32_t>(verts.size());
                const double x = gx * 1.0, z = gz * 1.0,
                             y = 0.02 * ((gx * 7 + gz * 13) % 11);
                verts.push_back(Vec3(x, y, z));
                verts.push_back(Vec3(x + 1, y, z));
                verts.push_back(Vec3(x + 1, y, z + 1));
                verts.push_back(Vec3(x, y, z + 1));
                idx.insert(idx.end(), {s0, s0 + 1, s0 + 2, s0, s0 + 2, s0 + 3});
            }
        const std::size_t tris = idx.size() / 3;
        const auto t0 = std::chrono::steady_clock::now();
        PhysicsBodyId id = world.addMesh(verts, idx, Vec3(), 0.5);
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0)
                              .count();
        std::printf("    [jolt-mesh] tris=%zu addMesh=%.3f ms (%.2f us/tri)\n",
                    tris, ms, ms * 1000.0 / static_cast<double>(tris));
        CHECK(id != INVALID_PHYSICS_BODY);
        CHECK(ms * 1000.0 / static_cast<double>(tris) < 50.0);
        world.removeBody(id);
    }
    world.shutdown();
}


TEST_CASE(door_leaf_swings_away_from_the_mover) {
    // Glenn's rule (ADR-0080): the leaf gets OUT OF THE WAY. Door at the
    // origin, outward normal +Z. -1 = swings inside, +1 = swings outside.
    const Vec2 foot(0, 0), n(0, 1);
    // Walking IN from the street (moving -Z): the leaf leads them inward.
    CHECK(doorSwingSign(Vec2(0, 1.5), Vec2(0, -1.2), foot, n) < 0);
    // Walking OUT from the lobby (moving +Z): it leads them outward.
    CHECK(doorSwingSign(Vec2(0, -1.5), Vec2(0, 1.2), foot, n) > 0);
    // Standing still OUTSIDE: it swings to the side they are not on.
    CHECK(doorSwingSign(Vec2(0, 1.0), Vec2(0, 0), foot, n) < 0);
    // Standing still INSIDE: likewise, away from them.
    CHECK(doorSwingSign(Vec2(0, -1.0), Vec2(0, 0), foot, n) > 0);
    // Sidling along the wall (tangential velocity): position decides.
    CHECK(doorSwingSign(Vec2(1.0, 0.8), Vec2(1.2, 0), foot, n) < 0);
}
