#include "test_framework.h"

#include "../src/engine/world.h"

using namespace engine;  // namespace migration (ADR-0014)

// Local component types so these tests exercise the registry without pulling in
// the renderer-coupled engine components.
namespace {
struct Position {
    int x, y;
};
struct Tag {
    int id;
};
}

TEST_CASE(world_create_destroy_alive) {
    World world;
    Entity a = world.create();
    Entity b = world.create();
    CHECK(world.alive(a));
    CHECK(world.alive(b));
    CHECK(world.entityCount() == 2);

    world.destroy(a);
    CHECK(!world.alive(a));
    CHECK(world.alive(b));
    CHECK(world.entityCount() == 1);

    // Destroying a stale handle is a no-op.
    world.destroy(a);
    CHECK(world.entityCount() == 1);
}

TEST_CASE(world_recycled_entity_is_distinct) {
    World world;
    Entity a = world.create();
    world.destroy(a);
    Entity b = world.create();

    // Reused slot, bumped generation: the old handle must read as dead.
    CHECK(!world.alive(a));
    CHECK(world.alive(b));
}

TEST_CASE(world_add_get_has_remove) {
    World world;
    Entity e = world.create();

    CHECK(!world.has<Position>(e));
    world.add<Position>(e, Position{3, 4});
    CHECK(world.has<Position>(e));

    Position* p = world.get<Position>(e);
    CHECK(p != nullptr);
    if (p) {
        CHECK(p->x == 3);
        CHECK(p->y == 4);
    }

    world.remove<Position>(e);
    CHECK(!world.has<Position>(e));
    CHECK(world.get<Position>(e) == nullptr);
}

TEST_CASE(world_destroy_strips_components) {
    World world;
    Entity e = world.create();
    world.add<Position>(e, Position{1, 1});
    world.destroy(e);

    // A recycled index must start with no components from its prior life.
    Entity reused = world.create();
    CHECK(reused.index == e.index);
    CHECK(!world.has<Position>(reused));
}

TEST_CASE(world_each_single_component) {
    World world;
    for (int i = 0; i < 3; i++) {
        Entity e = world.create();
        world.add<Position>(e, Position{i, i});
    }

    int count = 0;
    int sumX = 0;
    world.each<Position>([&](Entity, Position& p) {
        count++;
        sumX += p.x;
    });
    CHECK(count == 3);
    CHECK(sumX == 0 + 1 + 2);
}

TEST_CASE(world_each_intersects_components) {
    World world;
    Entity both = world.create();
    world.add<Position>(both, Position{5, 5});
    world.add<Tag>(both, Tag{1});

    Entity onlyPos = world.create();
    world.add<Position>(onlyPos, Position{9, 9});

    // each<Position, Tag> visits only entities that have BOTH.
    int visited = 0;
    world.each<Position, Tag>([&](Entity e, Position& p, Tag& t) {
        visited++;
        CHECK(e == both);
        CHECK(p.x == 5);
        CHECK(t.id == 1);
    });
    CHECK(visited == 1);
}

TEST_CASE(world_each_mutation_persists) {
    World world;
    Entity e = world.create();
    world.add<Position>(e, Position{0, 0});

    world.each<Position>([](Entity, Position& p) { p.x = 42; });

    Position* p = world.get<Position>(e);
    if (p) CHECK(p->x == 42);
}
