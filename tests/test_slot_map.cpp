#include "test_framework.h"

#include "../src/slot_map.h"

namespace {
struct Thing {
    int value;
};
}

TEST_CASE(handle_null_by_default) {
    Handle<Thing> handle;
    CHECK(!handle.valid());
    CHECK(handle.generation == 0);
}

TEST_CASE(slotmap_insert_get) {
    SlotMap<Thing> map;
    auto handle = map.insert(Thing{42});
    CHECK(handle.valid());
    CHECK(map.size() == 1);
    CHECK(map.contains(handle));

    Thing* thing = map.get(handle);
    CHECK(thing != nullptr);
    if (thing) CHECK(thing->value == 42);
}

TEST_CASE(slotmap_erase_invalidates) {
    SlotMap<Thing> map;
    auto handle = map.insert(Thing{7});
    CHECK(map.erase(handle));
    CHECK(map.size() == 0);
    CHECK(!map.contains(handle));
    CHECK(map.get(handle) == nullptr);

    // Erasing an already-freed handle is a no-op, not a crash.
    CHECK(!map.erase(handle));
}

TEST_CASE(slotmap_recycles_with_bumped_generation) {
    SlotMap<Thing> map;
    auto first = map.insert(Thing{1});
    map.erase(first);
    auto second = map.insert(Thing{2});

    // Slot index is reused, but the stale handle must not alias the new value.
    CHECK(second.index == first.index);
    CHECK(second.generation != first.generation);
    CHECK(!map.contains(first));
    CHECK(map.contains(second));

    Thing* viaStale = map.get(first);
    CHECK(viaStale == nullptr);
    Thing* viaFresh = map.get(second);
    if (viaFresh) CHECK(viaFresh->value == 2);
}

TEST_CASE(slotmap_multiple_live_handles) {
    SlotMap<Thing> map;
    auto a = map.insert(Thing{10});
    auto b = map.insert(Thing{20});
    auto c = map.insert(Thing{30});
    CHECK(map.size() == 3);

    map.erase(b);
    CHECK(map.size() == 2);
    CHECK(map.contains(a));
    CHECK(!map.contains(b));
    CHECK(map.contains(c));

    int sum = 0;
    map.forEach([&](Handle<Thing>, Thing& thing) { sum += thing.value; });
    CHECK(sum == 40);
}

TEST_CASE(slotmap_handle_at) {
    SlotMap<Thing> map;
    auto handle = map.insert(Thing{99});
    auto recovered = map.handleAt(handle.index);
    CHECK(recovered == handle);

    map.erase(handle);
    auto empty = map.handleAt(handle.index);
    CHECK(!empty.valid());
}

TEST_CASE(slotmap_clear_empties_and_staleness_holds) {
    SlotMap<Thing> map;
    auto a = map.insert(Thing{1});
    auto b = map.insert(Thing{2});
    CHECK(map.size() == 2);

    map.clear();
    CHECK(map.size() == 0);
    // Handles from before the clear must not alias new storage.
    CHECK(!map.contains(a));
    CHECK(!map.contains(b));
    CHECK(map.get(a) == nullptr);

    // The map is reusable after clear().
    auto fresh = map.insert(Thing{3});
    CHECK(map.contains(fresh));
    CHECK(map.size() == 1);
}

// Mirrors the renderer's usage (ADR-0007): a SlotMap keyed by a distinct Tag so
// its HandleType is the engine-facing handle (e.g. MeshHandle = Handle<MeshTag>).
TEST_CASE(slotmap_custom_tag_handle_type) {
    struct ResourceTag {};
    SlotMap<Thing, ResourceTag> resources;
    Handle<ResourceTag> handle = resources.insert(Thing{5});
    CHECK(handle.valid());
    CHECK(resources.contains(handle));

    resources.erase(handle);
    auto reused = resources.insert(Thing{6});
    CHECK(reused.index == handle.index);
    CHECK(!resources.contains(handle));   // stale handle detected
    CHECK(resources.contains(reused));
}
