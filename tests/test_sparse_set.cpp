#include "test_framework.h"

#include "../src/engine/sparse_set.h"

#include <algorithm>

TEST_CASE(sparse_set_insert_get) {
    SparseSet<int> set;
    set.insert(5, 50);
    set.insert(2, 20);

    CHECK(set.size() == 2);
    CHECK(set.containsIndex(5));
    CHECK(set.containsIndex(2));
    CHECK(!set.containsIndex(0));

    int* a = set.get(5);
    int* b = set.get(2);
    CHECK(a != nullptr && b != nullptr);
    if (a) CHECK(*a == 50);
    if (b) CHECK(*b == 20);
    CHECK(set.get(99) == nullptr);
}

TEST_CASE(sparse_set_insert_overwrites) {
    SparseSet<int> set;
    set.insert(3, 30);
    set.insert(3, 33);
    CHECK(set.size() == 1);
    int* v = set.get(3);
    if (v) CHECK(*v == 33);
}

TEST_CASE(sparse_set_remove_swap_and_pop) {
    SparseSet<int> set;
    set.insert(1, 11);
    set.insert(2, 22);
    set.insert(3, 33);

    // Removing a middle element must keep the others intact (swap-and-pop moves
    // the last element into the hole).
    set.removeIndex(1);
    CHECK(set.size() == 2);
    CHECK(!set.containsIndex(1));
    CHECK(set.containsIndex(2));
    CHECK(set.containsIndex(3));

    int* two = set.get(2);
    int* three = set.get(3);
    if (two) CHECK(*two == 22);
    if (three) CHECK(*three == 33);

    // Removing a non-existent index is a no-op.
    set.removeIndex(42);
    CHECK(set.size() == 2);
}

TEST_CASE(sparse_set_dense_iteration) {
    SparseSet<int> set;
    set.insert(4, 100);
    set.insert(7, 200);
    set.insert(1, 300);

    int sum = 0;
    for (uint32_t index : set.entityIndices()) {
        int* v = set.get(index);
        if (v) sum += *v;
    }
    CHECK(sum == 600);
    CHECK(set.entityIndices().size() == 3);
}
