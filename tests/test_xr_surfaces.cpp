#include "test_framework.h"

#include "../src/engine/xr/xr_surfaces.h"

#include <set>

using namespace engine;

// Room-surface bookkeeping (engine/xr/xr_surfaces.h), tested with a fake mesh
// uploader. The ARKit providers that feed this run only on a physical headset
// — the simulator supports neither plane detection nor scene reconstruction —
// so the lifecycle logic (the part that can leak GPU meshes or lose surfaces)
// is proven here, on any host, against a counting stand-in.

namespace {

// Counting MeshOps: mints sequential tokens and records what is live. The
// invariant every test leans on: at any pause, live() is exactly the meshes a
// renderer would be holding — creates minus destroys, no double-frees.
struct FakeMeshes {
    uint64_t next = 1;
    std::set<uint64_t> live;
    int created = 0, destroyed = 0;
    bool doubleFree = false;

    XrSurfaceLedger::MeshOps ops() {
        return {
            [this](const XrSurfaceUpdate&) {
                created++;
                live.insert(next);
                return next++;
            },
            [this](uint64_t token) {
                destroyed++;
                if (!live.erase(token)) doubleFree = true;
            },
        };
    }
};

XrSurfaceUpdate makeUpdate(uint64_t id, XrSurfaceUpdate::Op op,
                           XrSurfaceClass cls = XrSurfaceClass::Mesh,
                           int triangles = 2) {
    XrSurfaceUpdate u;
    u.anchorId = id;
    u.op = op;
    u.cls = cls;
    for (int t = 0; t < triangles; t++) {
        const uint32_t base = static_cast<uint32_t>(u.positions.size());
        u.positions.push_back(Vec3(0, 0, 0));
        u.positions.push_back(Vec3(1, 0, 0));
        u.positions.push_back(Vec3(0, 0, 1));
        u.indices.push_back(base);
        u.indices.push_back(base + 1);
        u.indices.push_back(base + 2);
    }
    return u;
}

}  // namespace

// --- Ledger lifecycle ----------------------------------------------------

TEST_CASE(added_anchor_creates_exactly_one_mesh) {
    FakeMeshes meshes;
    XrSurfaceLedger ledger;
    ledger.apply({makeUpdate(7, XrSurfaceUpdate::Op::Added,
                             XrSurfaceClass::Floor)}, meshes.ops());

    CHECK(meshes.created == 1);
    CHECK(meshes.live.size() == 1);
    CHECK(ledger.surfaces().size() == 1);
    CHECK(ledger.surfaces().at(7).cls == XrSurfaceClass::Floor);
    CHECK(ledger.surfaces().at(7).meshToken != 0);
}

TEST_CASE(updated_anchor_replaces_its_mesh_without_leaking) {
    // The mutation this exists to catch: uploadMesh has no update-in-place, so
    // every anchor refresh is a new mesh — and forgetting to destroy the old
    // one leaks GPU memory continuously while the user looks around the room.
    FakeMeshes meshes;
    XrSurfaceLedger ledger;
    ledger.apply({makeUpdate(7, XrSurfaceUpdate::Op::Added)}, meshes.ops());
    const uint64_t first = ledger.surfaces().at(7).meshToken;

    for (int round = 0; round < 10; round++)
        ledger.apply({makeUpdate(7, XrSurfaceUpdate::Op::Updated)}, meshes.ops());

    CHECK(ledger.surfaces().size() == 1);
    CHECK(meshes.live.size() == 1);                       // never two at once
    CHECK(ledger.surfaces().at(7).meshToken != first);    // and it is the new one
    CHECK(meshes.created == 11);
    CHECK(meshes.destroyed == 10);
    CHECK(!meshes.doubleFree);
}

TEST_CASE(removed_anchor_destroys_its_mesh) {
    FakeMeshes meshes;
    XrSurfaceLedger ledger;
    ledger.apply({makeUpdate(7, XrSurfaceUpdate::Op::Added)}, meshes.ops());
    ledger.apply({makeUpdate(7, XrSurfaceUpdate::Op::Removed)}, meshes.ops());

    CHECK(ledger.surfaces().empty());
    CHECK(meshes.live.empty());
    CHECK(!meshes.doubleFree);
}

TEST_CASE(runtime_rough_edges_are_tolerated) {
    // ARKit's callback queue is not a tidy protocol. Three real orderings:
    // an Updated whose Added it overtook, a Removed for an id never seen, and
    // a Removed delivered twice. None of them may crash, leak, or double-free.
    FakeMeshes meshes;
    XrSurfaceLedger ledger;

    ledger.apply({makeUpdate(9, XrSurfaceUpdate::Op::Updated)}, meshes.ops());
    CHECK(ledger.surfaces().size() == 1);     // Updated-first behaves as Added

    ledger.apply({makeUpdate(42, XrSurfaceUpdate::Op::Removed)}, meshes.ops());
    CHECK(ledger.surfaces().size() == 1);     // unknown remove is a no-op

    ledger.apply({makeUpdate(9, XrSurfaceUpdate::Op::Removed)}, meshes.ops());
    ledger.apply({makeUpdate(9, XrSurfaceUpdate::Op::Removed)}, meshes.ops());
    CHECK(ledger.surfaces().empty());
    CHECK(meshes.live.empty());
    CHECK(!meshes.doubleFree);
}

TEST_CASE(a_mixed_stream_leaves_exactly_the_live_set) {
    FakeMeshes meshes;
    XrSurfaceLedger ledger;
    std::vector<XrSurfaceUpdate> stream;
    stream.push_back(makeUpdate(1, XrSurfaceUpdate::Op::Added, XrSurfaceClass::Floor));
    stream.push_back(makeUpdate(2, XrSurfaceUpdate::Op::Added, XrSurfaceClass::Wall));
    stream.push_back(makeUpdate(3, XrSurfaceUpdate::Op::Added, XrSurfaceClass::Mesh, 40));
    stream.push_back(makeUpdate(2, XrSurfaceUpdate::Op::Updated, XrSurfaceClass::Wall));
    stream.push_back(makeUpdate(1, XrSurfaceUpdate::Op::Removed));
    stream.push_back(makeUpdate(4, XrSurfaceUpdate::Op::Added, XrSurfaceClass::Table));
    ledger.apply(stream, meshes.ops());

    CHECK(ledger.surfaces().size() == 3);     // 2, 3, 4
    CHECK(meshes.live.size() == 3);
    CHECK(ledger.surfaces().count(1) == 0);
    CHECK(!meshes.doubleFree);
}

TEST_CASE(clear_releases_every_mesh_for_session_restart) {
    FakeMeshes meshes;
    XrSurfaceLedger ledger;
    for (uint64_t id = 1; id <= 5; id++)
        ledger.apply({makeUpdate(id, XrSurfaceUpdate::Op::Added)}, meshes.ops());
    ledger.clear(meshes.ops());

    CHECK(ledger.surfaces().empty());
    CHECK(meshes.live.empty());
    CHECK(!meshes.doubleFree);
}

TEST_CASE(geometryless_updates_carry_no_mesh) {
    // A plane can arrive before the runtime has meshed it. Track the surface,
    // mint no token — the create callback must not run on empty geometry.
    FakeMeshes meshes;
    XrSurfaceLedger ledger;
    ledger.apply({makeUpdate(7, XrSurfaceUpdate::Op::Added,
                             XrSurfaceClass::Wall, 0)}, meshes.ops());

    CHECK(ledger.surfaces().size() == 1);
    CHECK(ledger.surfaces().at(7).meshToken == 0);
    CHECK(meshes.created == 0);
    ledger.apply({makeUpdate(7, XrSurfaceUpdate::Op::Removed)}, meshes.ops());
    CHECK(meshes.destroyed == 0);
    CHECK(!meshes.doubleFree);
}

// --- Census (the numeric readout) ----------------------------------------

TEST_CASE(census_counts_classes_triangles_and_the_lowest_floor) {
    FakeMeshes meshes;
    XrSurfaceLedger ledger;
    auto floorAt = [](uint64_t id, Real y) {
        XrSurfaceUpdate u = makeUpdate(id, XrSurfaceUpdate::Op::Added,
                                       XrSurfaceClass::Floor);
        u.originFromAnchor = Mat4::translate(0, y, 0);
        return u;
    };
    // Two floors (the real floor and a rug shelf), one wall, one 40-tri chunk.
    // The REAL floor gets the lower anchor id deliberately: the ledger's map
    // iterates by id, so a census that lazily took the LAST floor seen would
    // land on the shelf — ordering the fixture this way is what makes that
    // mutation fail instead of coincidentally passing.
    ledger.apply({floorAt(1, -0.02), floorAt(2, 0.4),
                  makeUpdate(3, XrSurfaceUpdate::Op::Added, XrSurfaceClass::Wall),
                  makeUpdate(4, XrSurfaceUpdate::Op::Added, XrSurfaceClass::Mesh, 40)},
                 meshes.ops());

    const auto c = ledger.census();
    CHECK(c.total == 4);
    CHECK(c.countByClass[static_cast<int>(XrSurfaceClass::Floor)] == 2);
    CHECK(c.countByClass[static_cast<int>(XrSurfaceClass::Wall)] == 1);
    CHECK(c.countByClass[static_cast<int>(XrSurfaceClass::Mesh)] == 1);
    CHECK(c.triangles == 2 + 2 + 2 + 40);
    CHECK(c.floorValid);
    CHECK_APPROX(c.floorY, -0.02, 1e-9);      // the LOWEST floor, not the last
}

TEST_CASE(census_reports_no_floor_until_one_exists) {
    XrSurfaceLedger ledger;
    CHECK(!ledger.census().floorValid);
}

// --- Helpers --------------------------------------------------------------

TEST_CASE(every_surface_class_has_a_name_and_a_color) {
    // Total mappings: a class added to the enum without a name/color would
    // otherwise fall through to defaults silently.
    std::set<std::string> names;
    for (int i = 0; i < XR_SURFACE_CLASS_COUNT; i++) {
        const auto cls = static_cast<XrSurfaceClass>(i);
        names.insert(xrSurfaceClassName(cls));
        const Vec3 c = xrSurfaceClassColor(cls);
        CHECK(c.x >= 0 && c.x <= 1 && c.y >= 0 && c.y <= 1 && c.z >= 0 && c.z <= 1);
    }
    CHECK(names.size() == static_cast<size_t>(XR_SURFACE_CLASS_COUNT));
}

TEST_CASE(world_transform_scales_geometry_not_just_translation) {
    // A giant's room: at scale 25 a point 1 m up in anchor space must land
    // 25 world units up — vertices scale WITH the transform, which is exactly
    // where this differs from the rigid eye-pose scaling (translation only).
    const Mat4 anchor = Mat4::translate(2, 0, 0);
    const Mat4 world = xrSurfaceWorldTransform(Vec3(100, 5, 0), 25.0, anchor);

    const Vec3 origin = world.transformPoint(Vec3(0, 0, 0));
    CHECK_APPROX(origin.x, 100.0 + 25.0 * 2.0, 1e-9);
    CHECK_APPROX(origin.y, 5.0, 1e-9);

    const Vec3 up = world.transformPoint(Vec3(0, 1, 0));
    CHECK_APPROX(up.y - origin.y, 25.0, 1e-9);
}

TEST_CASE(boundary_edges_trace_the_outline_of_a_quad) {
    // Two triangles sharing a diagonal: the boundary is the four rim edges;
    // the shared diagonal (1,2) must not appear.
    const std::vector<uint32_t> quad = {0, 1, 2, 1, 3, 2};
    auto boundary = xrSurfaceBoundaryEdges(quad);

    CHECK(boundary.size() == 4);
    for (const auto& edge : boundary)
        CHECK(!(edge.first == 1 && edge.second == 2));
}

TEST_CASE(boundary_of_a_lone_triangle_is_all_three_edges) {
    auto boundary = xrSurfaceBoundaryEdges({0, 1, 2});
    CHECK(boundary.size() == 3);
}

// --- Extent (the dimensions readout) --------------------------------------

TEST_CASE(extent_measures_a_plane_in_anchor_space) {
    // A 2.4 x 0 x 1.5 m tabletop: extent.x/z are the printed dimensions.
    std::vector<Vec3> quad = {Vec3(-1.2, 0, -0.75), Vec3(1.2, 0, -0.75),
                              Vec3(1.2, 0, 0.75), Vec3(-1.2, 0, 0.75)};
    const auto extent = xrSurfaceExtent(quad);
    CHECK(extent.valid);
    CHECK_APPROX(extent.size().x, 2.4, 1e-9);
    CHECK_APPROX(extent.size().z, 1.5, 1e-9);
    CHECK(!xrSurfaceExtent({}).valid);
}

// --- Placement raycast ----------------------------------------------------

namespace {
// A unit quad in the XZ plane at y=0 (two triangles), ARKit plane style.
struct QuadMesh {
    std::vector<Vec3> positions = {Vec3(-0.5, 0, -0.5), Vec3(0.5, 0, -0.5),
                                   Vec3(0.5, 0, 0.5), Vec3(-0.5, 0, 0.5)};
    std::vector<uint32_t> indices = {0, 1, 2, 0, 2, 3};
};
}  // namespace

TEST_CASE(raycast_hits_a_plane_from_above_at_the_right_distance) {
    QuadMesh quad;
    Real t = 0;
    CHECK(xrRaycastTriangles(Vec3(0.25, 2, 0.25), Vec3(0, -1, 0),
                             quad.positions, quad.indices, t));
    CHECK_APPROX(t, 2.0, 1e-9);
}

TEST_CASE(raycast_is_two_sided) {
    // From BELOW: same quad, winding now faces away. A one-sided test would
    // let a placement ray fall through every table ARKit happened to wind
    // face-down — the reason the header promises two-sided.
    QuadMesh quad;
    Real t = 0;
    CHECK(xrRaycastTriangles(Vec3(0, -3, 0), Vec3(0, 1, 0),
                             quad.positions, quad.indices, t));
    CHECK_APPROX(t, 3.0, 1e-9);
}

TEST_CASE(raycast_misses_beside_and_behind) {
    QuadMesh quad;
    Real t = 0;
    // Beside the quad.
    CHECK(!xrRaycastTriangles(Vec3(2, 1, 2), Vec3(0, -1, 0),
                              quad.positions, quad.indices, t));
    // Pointing away: the surface is behind the ray, and hits at negative t
    // must not count — that would let you place objects behind your head.
    CHECK(!xrRaycastTriangles(Vec3(0, 1, 0), Vec3(0, 1, 0),
                              quad.positions, quad.indices, t));
}

TEST_CASE(raycast_returns_the_nearest_of_stacked_surfaces) {
    // Two parallel quads, shelf over floor: gaze from above must land on the
    // shelf, not tunnel to the floor behind it.
    QuadMesh quad;
    std::vector<Vec3> positions = quad.positions;      // y = 0 (floor)
    for (const Vec3& p : quad.positions)
        positions.push_back(Vec3(p.x, 1.0, p.z));      // y = 1 (shelf)
    std::vector<uint32_t> indices = quad.indices;
    for (uint32_t i : quad.indices) indices.push_back(i + 4);

    Real t = 0;
    CHECK(xrRaycastTriangles(Vec3(0, 3, 0), Vec3(0, -1, 0),
                             positions, indices, t));
    CHECK_APPROX(t, 2.0, 1e-9);                        // shelf at y=1, not floor
}

TEST_CASE(raycast_skips_degenerate_triangles) {
    // A zero-area triangle in front of a real one: skipped, not hit, not fatal.
    std::vector<Vec3> positions = {Vec3(0, 1, 0), Vec3(0, 1, 0), Vec3(0, 1, 0),
                                   Vec3(-1, 0, -1), Vec3(1, 0, -1), Vec3(0, 0, 1)};
    std::vector<uint32_t> indices = {0, 1, 2, 3, 4, 5};
    Real t = 0;
    CHECK(xrRaycastTriangles(Vec3(0, 2, 0), Vec3(0, -1, 0), positions, indices, t));
    CHECK_APPROX(t, 2.0, 1e-9);                        // the real one, at y=0
}

TEST_CASE(a_world_ray_round_trips_through_the_anchor_transform) {
    // The composition placement actually runs: world gaze ray -> anchor space
    // via the inverse world transform, raycast in real metres, point back out
    // to world. At scale 2 with a shifted base, a hit 10 real cm off the
    // anchor's centre must come back to the matching world position.
    QuadMesh quad;
    const Vec3 base(10, 3, -5);
    const Real scale = 2.0;
    const Mat4 anchor = Mat4::translate(1, 0.8, 2);    // a table anchor
    const Mat4 world = xrSurfaceWorldTransform(base, scale, anchor);
    const Mat4 inv = world.inverse();

    // Aim straight down over the point 0.1 m across the table in anchor space.
    const Vec3 targetWorld = world.transformPoint(Vec3(0.1, 0, 0.1));
    const Vec3 originWorld = targetWorld + Vec3(0, 4, 0);

    Vec3 o = inv.transformPoint(originWorld);
    Vec3 d = inv.transformDirection(Vec3(0, -1, 0));
    const Real dLen = d.length();
    d /= dLen;
    Real t = 0;
    CHECK(xrRaycastTriangles(o, d, quad.positions, quad.indices, t));

    const Vec3 hitAnchor = o + d * t;
    CHECK_APPROX(hitAnchor.x, 0.1, 1e-9);
    CHECK_APPROX(hitAnchor.y, 0.0, 1e-9);
    CHECK_APPROX(hitAnchor.z, 0.1, 1e-9);
    const Vec3 hitWorld = world.transformPoint(hitAnchor);
    CHECK_APPROX(hitWorld.x, targetWorld.x, 1e-9);
    CHECK_APPROX(hitWorld.y, targetWorld.y, 1e-9);
    CHECK_APPROX(hitWorld.z, targetWorld.z, 1e-9);
    // And t is REAL metres (anchor units), not world units: 4 world units of
    // drop at scale 2 is 2 real metres.
    CHECK_APPROX(t, 2.0, 1e-9);
}

// --- Store ----------------------------------------------------------------

TEST_CASE(store_drains_in_arrival_order_and_empties) {
    XrSurfaceStore store;
    store.push(makeUpdate(1, XrSurfaceUpdate::Op::Added));
    store.push(makeUpdate(1, XrSurfaceUpdate::Op::Updated));
    store.push(makeUpdate(1, XrSurfaceUpdate::Op::Removed));

    std::vector<XrSurfaceUpdate> drained;
    store.drain(drained);
    CHECK(drained.size() == 3);
    CHECK(drained[0].op == XrSurfaceUpdate::Op::Added);
    CHECK(drained[2].op == XrSurfaceUpdate::Op::Removed);

    drained.clear();
    store.drain(drained);
    CHECK(drained.empty());
}

// --- XrColliderPolicy: when room colliders get (re)built ------------------
//
// The physics side (Jolt MeshShape cooking) runs only in the CMake build;
// what the policy owns — and what these tests pin — is the SCHEDULE: first
// build immediate, refreshes throttled per anchor, removal forgetting, and
// the invalidate-all wave after an origin/scale move.

TEST_CASE(collider_policy_first_build_is_immediate) {
    XrColliderPolicy policy(1.0);
    policy.noteUpdate(1, 0.0);
    auto due = policy.drainDue(0.0);
    CHECK(due.size() == 1);
    CHECK(due[0] == 1);
    CHECK(policy.pendingCount() == 0);
}

TEST_CASE(collider_policy_throttles_update_storms) {
    XrColliderPolicy policy(1.0);
    policy.noteUpdate(1, 0.0);
    CHECK(policy.drainDue(0.0).size() == 1);

    // A storm of refinements inside the interval: all deferred, none lost.
    policy.noteUpdate(1, 0.1);
    policy.noteUpdate(1, 0.3);
    policy.noteUpdate(1, 0.5);
    CHECK(policy.drainDue(0.5).empty());
    CHECK(policy.pendingCount() == 1);

    // Past the interval the ONE coalesced rebuild fires.
    auto due = policy.drainDue(1.05);
    CHECK(due.size() == 1);
    CHECK(policy.pendingCount() == 0);
    // ...and nothing further without a new update.
    CHECK(policy.drainDue(5.0).empty());
}

TEST_CASE(collider_policy_new_anchor_unaffected_by_others_throttle) {
    XrColliderPolicy policy(1.0);
    policy.noteUpdate(1, 0.0);
    CHECK(policy.drainDue(0.0).size() == 1);

    policy.noteUpdate(1, 0.2);   // throttled refresh
    policy.noteUpdate(2, 0.2);   // brand-new anchor: a hole in the floor
    auto due = policy.drainDue(0.2);
    CHECK(due.size() == 1);
    CHECK(due[0] == 2);
}

TEST_CASE(collider_policy_removed_anchor_is_forgotten) {
    XrColliderPolicy policy(1.0);
    policy.noteUpdate(1, 0.0);
    policy.noteRemoved(1);
    CHECK(policy.drainDue(0.0).empty());
    CHECK(policy.pendingCount() == 0);

    // Re-added later: a fresh anchor again, immediate build.
    policy.noteUpdate(1, 3.0);
    CHECK(policy.drainDue(3.0).size() == 1);
}

TEST_CASE(collider_policy_invalidate_all_staggers_by_last_build) {
    XrColliderPolicy policy(1.0);
    policy.noteUpdate(1, 0.0);
    policy.noteUpdate(2, 0.0);
    CHECK(policy.drainDue(0.0).size() == 2);

    // Anchor 2 refreshes at t=1.2; both then invalidated at t=1.5 (origin
    // moved). Anchor 1's last build is old — it rebuilds now; anchor 2 waits
    // out its own interval. The wave staggers instead of spiking.
    policy.noteUpdate(2, 1.2);
    CHECK(policy.drainDue(1.2).size() == 1);
    policy.invalidateAll();
    auto due = policy.drainDue(1.5);
    CHECK(due.size() == 1);
    CHECK(due[0] == 1);
    CHECK(policy.pendingCount() == 1);
    CHECK(policy.drainDue(2.3).size() == 1);
    CHECK(policy.pendingCount() == 0);
}

TEST_CASE(collider_policy_drain_cap_spreads_a_burst) {
    // A whole room arriving at once must not cook in one frame: capped
    // drains return a slice and KEEP the rest dirty for later calls.
    XrColliderPolicy policy(1.0);
    for (uint64_t id = 1; id <= 5; id++) policy.noteUpdate(id, 0.0);
    CHECK(policy.drainDue(0.0, 2).size() == 2);
    CHECK(policy.pendingCount() == 3);
    CHECK(policy.drainDue(0.1, 2).size() == 2);
    CHECK(policy.drainDue(0.2, 2).size() == 1);
    CHECK(policy.pendingCount() == 0);
    CHECK(policy.drainDue(0.3, 2).empty());
}
