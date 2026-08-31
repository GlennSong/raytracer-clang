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
    const PartId floorPart = floorFinishPartFor(interiorParams());
    for (const RenderMesh& m : a.parts) {
        if (m.materialIndex != static_cast<int>(floorPart)) continue;
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

TEST_CASE(floor_finishes_are_five_distinct_looks) {
    // Device: "all of the flooring in all of the buildings are the same"
    // and "what are some other floor materials we could create?" -- the
    // finish classes must be MATERIALLY distinct (albedo and/or surface),
    // not tones of one look, and the stone/marble/carpet classes must
    // actually be present so the bakes are reachable.
    BuildingParams p = interiorParams();
    RenderMaterial m[5];
    for (uint32_t k = 0; k < 5; ++k) {
        p.seed = (p.seed & ~uint32_t(0x1C0)) | (k << 6);
        m[k] = floorFinishFor(p);
    }
    for (int a = 0; a < 5; ++a)
        for (int b = a + 1; b < 5; ++b) {
            const bool albedoDiffers =
                std::fabs(m[a].albedo.x - m[b].albedo.x) > 0.05 ||
                std::fabs(m[a].albedo.y - m[b].albedo.y) > 0.05 ||
                std::fabs(m[a].albedo.z - m[b].albedo.z) > 0.05;
            const bool surfaceDiffers = m[a].surface() != m[b].surface();
            CHECK(albedoDiffers || surfaceDiffers);
        }
    bool tile = false, marble = false, carpet = false;
    for (int k = 0; k < 5; ++k) {
        if (m[k].surface() == RenderMaterial::Surface::Concrete) tile = true;
        if (m[k].surface() == RenderMaterial::Surface::Marble) marble = true;
        if (m[k].surface() == RenderMaterial::Surface::Carpet) carpet = true;
    }
    CHECK(tile);
    CHECK(marble);
    CHECK(carpet);
}

TEST_CASE(interior_paint_varies_and_stays_bright) {
    // Device: "not all just white walls" -- then, of the first palette,
    // "most buildings I entered they were still white": variety must be
    // READABLE, not just numerically distinct. At most ONE palette entry
    // may sit within a white-band (all channels > 0.78 and near-equal),
    // every channel stays >= 0.45, and all eight entries differ.
    BuildingParams p = interiorParams();
    int distinct = 0, whites = 0;
    Vec3 seen[8];
    for (uint32_t k = 0; k < 8; ++k) {
        p.seed = (p.seed & ~uint32_t(0x3800)) | (k << 11);
        const Vec3 c = interiorPaintFor(p);
        CHECK(c.x > 0.45);
        CHECK(c.y > 0.45);
        CHECK(c.z > 0.45);
        const Real mx = std::max(c.x, std::max(c.y, c.z));
        const Real mn = std::min(c.x, std::min(c.y, c.z));
        if (mn > 0.78 && mx - mn < 0.1) ++whites;
        bool fresh = true;
        for (int j = 0; j < distinct; ++j)
            if (std::fabs(seen[j].x - c.x) < 0.02 &&
                std::fabs(seen[j].y - c.y) < 0.02 &&
                std::fabs(seen[j].z - c.z) < 0.02)
                fresh = false;
        if (fresh) seen[distinct++] = c;
    }
    std::printf("    [paint] %d distinct paints, %d white-band\n",
                distinct, whites);
    CHECK(distinct >= 6);
    CHECK(whites <= 1);
}

TEST_CASE(stairs_pick_their_own_finish) {
    // Device: "could we apply different materials to the stairs for
    // different looks" -- the stair finish is an INDEPENDENT axis: a seed
    // exists where the stair differs from the floor (stone stair, wood
    // floor), and the matched-set class follows the floor exactly.
    BuildingParams p = interiorParams();
    // Floor class 0 (walnut wood), stair class 3 (stone).
    p.seed = (p.seed & ~uint32_t(0x1C0)) & ~uint32_t(0x600);
    p.seed |= (3u << 9);
    CHECK(floorFinishFor(p).surface() ==
          RenderMaterial::Surface::WoodSiding);
    CHECK(stairFinishFor(p).surface() ==
          RenderMaterial::Surface::Concrete);
    CHECK(stairFinishPartFor(p) == PartId::InteriorFloorTile);
    // Stair class 0 = matched set: identical material to the floor.
    p.seed &= ~uint32_t(0x600);
    const RenderMaterial f = floorFinishFor(p), st = stairFinishFor(p);
    CHECK(st.surface() == f.surface());
    CHECK(std::fabs(st.albedo.x - f.albedo.x) < 1e-9);
    CHECK(stairFinishPartFor(p) == floorFinishPartFor(p));
}

TEST_CASE(side_bay_edge_keeps_its_interior_wall) {
    // Device: "an interior wall was missing... completely see through to
    // the outside world." The attached-garage (sideBays) edge continued
    // past the inner-shell emission -- from inside, no wall at all. The
    // sealing pane replaces a RICHER shell (fewer tris than a windowed
    // wall), so a total-count census reads backwards; the honest gate is
    // COVERAGE: every edge of the rect plan must have Interior-part
    // triangles in its inset band. The broken bay edge had ZERO.
    BuildingParams p = interiorParams();
    p.floors = 1;
    p.sideBays = 1;
    const BuildingMesh bm =
        growPlanBuilding(kPlan, p, 0.0, FacadeDetail::Full);
    const Real gh = p.groundHeight;
    struct Band { int axis; Real lo, hi; };   // axis 0 = x, 1 = z
    const Band bands[4] = {{0, 0.35, 0.9}, {0, 19.1, 19.65},
                           {1, 0.35, 0.9}, {1, 14.1, 14.65}};
    int found[4] = {0, 0, 0, 0};
    for (const RenderMesh& m : bm.parts) {
        if (m.materialIndex != static_cast<int>(PartId::Interior)) continue;
        for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
            bool in[4] = {true, true, true, true};
            for (int k = 0; k < 3 && (in[0] || in[1] || in[2] || in[3]);
                 ++k) {
                const Vec3& v = m.vertices[m.indices[i + k]].position;
                if (v.y > gh + 0.6) { in[0] = in[1] = in[2] = in[3] = false; }
                for (int b = 0; b < 4; ++b) {
                    const Real c = bands[b].axis == 0 ? v.x : v.z;
                    if (c < bands[b].lo || c > bands[b].hi) in[b] = false;
                }
            }
            for (int b = 0; b < 4; ++b)
                if (in[b]) ++found[b];
        }
    }
    std::printf("    [side-bay] inset-band tris %d %d %d %d\n", found[0],
                found[1], found[2], found[3]);
    for (int b = 0; b < 4; ++b) CHECK(found[b] >= 2);
}

TEST_CASE(character_climbs_the_stair_around_the_well_to_the_top) {
    BuildingParams p = interiorParams();
    const InteriorLayout il =
        interiorLayout(kPlan, p, entranceEdgeFor(kPlan, p));
    CHECK(il.hasStair);
    {   // Layout probe: pin the numbers the climb runs against.
        Real wx0 = 1e9, wx1 = -1e9, wz0 = 1e9, wz1 = -1e9;
        for (const Vec2& v : il.well) {
            wx0 = std::min(wx0, v.x); wx1 = std::max(wx1, v.x);
            wz0 = std::min(wz0, v.y); wz1 = std::max(wz1, v.y);
        }
        std::printf("    [layout] foot=(%.2f,%.2f) dir=(%.2f,%.2f) "
                    "run=%.2f tread=%.2f well x[%.2f,%.2f] z[%.2f,%.2f]\n",
                    il.stairFoot.x, il.stairFoot.y, il.stairDir.x,
                    il.stairDir.y, il.run, il.tread, wx0, wx1, wz0, wz1);
    }
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

    // Up flight 2 (storey 1 -> storey 2 at 7.75). Chunked, with probe
    // prints: when a pitch change stalls the climb, the stall point is
    // what names the obstacle.
    for (int leg = 0; leg < 8; ++leg) {
        walkTo(sxT, sz, 150);
        const Vec3 lp = world.characterPosition(c);
        std::printf("    [climb] f2 leg %d x=%.2f y=%.2f z=%.2f\n", leg,
                    lp.x, lp.y, lp.z);
    }
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
