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
