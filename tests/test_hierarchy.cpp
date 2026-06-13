#include "test_framework.h"

#include "../src/engine/components.h"
#include "../src/engine/world.h"

using namespace engine;  // namespace migration (ADR-0015)

// Document hierarchy: stable ids, parent composition, cycle guards — the
// engine foundation under the editor's grouping/parenting (docs/NEXT_STEPS).

namespace {
Entity makeDoc(World& world, uint32_t id, uint32_t parentId, const Vec3& pos,
               const Vec3& scale = Vec3(1, 1, 1)) {
    Entity e = world.create();
    Transform t;
    t.position = pos;
    t.scale = scale;
    world.add<Transform>(e, t);
    SourceSpec s;
    s.id = id;
    s.parentId = parentId;
    world.add<SourceSpec>(e, s);
    return e;
}
}  // namespace

TEST_CASE(document_ids_allocate_and_resolve) {
    World world;
    makeDoc(world, 5, 0, Vec3());
    Entity b = makeDoc(world, 0, 0, Vec3());   // unassigned
    Entity c = makeDoc(world, 0, 0, Vec3());

    CHECK(maxDocumentId(world) == 5);
    CHECK(nextDocumentId(world) == 6);

    assignMissingDocumentIds(world);
    // The two id-0 specs pick up 6 and 7 (max+1 upward, iteration order).
    CHECK(world.get<SourceSpec>(b)->id >= 6);
    CHECK(world.get<SourceSpec>(c)->id >= 6);
    CHECK(world.get<SourceSpec>(b)->id != world.get<SourceSpec>(c)->id);

    CHECK(findByDocumentId(world, 5).valid());
    CHECK(!findByDocumentId(world, 0).valid());     // id 0 never matches
    CHECK(!findByDocumentId(world, 999).valid());
}

TEST_CASE(world_matrix_composes_parent_translation_and_scale) {
    World world;
    makeDoc(world, 1, 0, Vec3(10, 0, 0), Vec3(2, 2, 2));   // parent
    Entity child = makeDoc(world, 2, 1, Vec3(1, 5, 0));    // child of 1

    // Child origin in world: parent scales the child's local offset, then
    // translates. (1,5,0)*2 + (10,0,0) = (12,10,0).
    Mat4 wm = worldMatrix(world, child);
    Vec3 origin = wm.transformPoint(Vec3(0, 0, 0));
    CHECK_APPROX(origin.x, 12.0, 1e-9);
    CHECK_APPROX(origin.y, 10.0, 1e-9);
    CHECK_APPROX(origin.z, 0.0, 1e-9);

    // Unparented entity: world == local.
    Entity lone = makeDoc(world, 3, 0, Vec3(4, 4, 4));
    Vec3 p = worldMatrix(world, lone).transformPoint(Vec3(0, 0, 0));
    CHECK_APPROX(p.x, 4.0, 1e-9);
}

TEST_CASE(world_matrix_handles_grandparent_chain_and_broken_links) {
    World world;
    makeDoc(world, 1, 0, Vec3(10, 0, 0));
    makeDoc(world, 2, 1, Vec3(0, 10, 0));
    Entity gc = makeDoc(world, 3, 2, Vec3(0, 0, 10));

    Vec3 p = worldMatrix(world, gc).transformPoint(Vec3(0, 0, 0));
    CHECK_APPROX(p.x, 10.0, 1e-9);
    CHECK_APPROX(p.y, 10.0, 1e-9);
    CHECK_APPROX(p.z, 10.0, 1e-9);

    // A dangling parent id (parent deleted, hand-edited level) terminates the
    // walk at the entity's own local transform rather than crashing.
    Entity orphan = makeDoc(world, 4, 999, Vec3(7, 0, 0));
    Vec3 op = worldMatrix(world, orphan).transformPoint(Vec3(0, 0, 0));
    CHECK_APPROX(op.x, 7.0, 1e-9);
}

TEST_CASE(reparent_cycle_guard) {
    World world;
    Entity a = makeDoc(world, 1, 0, Vec3());      // root
    Entity b = makeDoc(world, 2, 1, Vec3());      // child of a
    Entity c = makeDoc(world, 3, 2, Vec3());      // child of b

    // Parenting a under c (its own descendant) would close a loop.
    CHECK(wouldCreateCycle(world, a, c));
    CHECK(wouldCreateCycle(world, a, b));
    CHECK(wouldCreateCycle(world, b, b));   // self
    // Parenting c under a fresh root is fine.
    Entity d = makeDoc(world, 4, 0, Vec3());
    CHECK(!wouldCreateCycle(world, c, d));
    CHECK(!wouldCreateCycle(world, d, c));
}

TEST_CASE(transform_from_matrix_round_trips) {
    Transform t;
    t.position = Vec3(3, -2, 5);
    t.scale = Vec3(2, 0.5, 1.5);
    t.orientation = Quat::fromAxisAngle(normalize(Vec3(0.2, 1, -0.3)), 0.9);

    Transform back = transformFromMatrix(t.matrix());
    CHECK_APPROX(back.position.x, 3.0, 1e-9);
    CHECK_APPROX(back.position.y, -2.0, 1e-9);
    CHECK_APPROX(back.scale.x, 2.0, 1e-9);
    CHECK_APPROX(back.scale.y, 0.5, 1e-9);
    CHECK_APPROX(back.scale.z, 1.5, 1e-9);
    // Orientation: compare action on a probe vector.
    Vec3 v(0.4, -0.6, 0.7);
    Vec3 a = t.orientation.rotate(v), b = back.orientation.rotate(v);
    CHECK_APPROX(a.x, b.x, 1e-9);
    CHECK_APPROX(a.y, b.y, 1e-9);
    CHECK_APPROX(a.z, b.z, 1e-9);
}

TEST_CASE(flatten_bakes_world_transform) {
    // The runtime flatten (level_loader, PLAY mode) is exactly
    // transformFromMatrix(worldMatrix(e)). Verify the composite for a child.
    World world;
    makeDoc(world, 1, 0, Vec3(10, 0, 0), Vec3(2, 2, 2));
    Entity child = makeDoc(world, 2, 1, Vec3(1, 0, 0));

    Transform flat = transformFromMatrix(worldMatrix(world, child));
    CHECK_APPROX(flat.position.x, 12.0, 1e-9);   // 1*2 + 10
    CHECK_APPROX(flat.scale.x, 2.0, 1e-9);        // parent scale propagates
}
