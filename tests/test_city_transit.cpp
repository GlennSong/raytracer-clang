#include "../src/apps/citysim/city_transit.h"
#include "test_framework.h"

using namespace citysim;

TEST_CASE(ride_book_boards_and_alights) {
    RideBook b;
    CHECK(b.empty());
    CHECK(b.board(7, 3));
    CHECK(b.driverOf(7) == 3);
    CHECK(b.load(3) == 1);
    CHECK(b.driverOf(3) == -1);        // the driver is not a passenger
    b.alight(7);
    CHECK(b.driverOf(7) == -1);
    CHECK(b.load(3) == 0);
    CHECK(b.empty());
}

TEST_CASE(ride_book_refuses_impossible_rides) {
    RideBook b;
    CHECK(!b.board(4, 4));             // nobody is their own fare
    CHECK(!b.board(-1, 2));
    CHECK(b.board(5, 2));
    CHECK(!b.board(5, 9));             // already aboard
    CHECK(!b.board(9, 5));             // a rider cannot drive
    CHECK(b.load(2) == 1);
}

TEST_CASE(ride_book_carries_several_and_sets_them_all_down) {
    RideBook b;
    CHECK(b.board(10, 1));
    CHECK(b.board(11, 1));
    CHECK(b.board(12, 1));
    CHECK(b.load(1) == 3);
    CHECK(b.rideCount() == 3u);
    b.alightAll(1);
    CHECK(b.load(1) == 0);
    CHECK(b.empty());
}

TEST_CASE(ride_book_traversal_is_deterministic) {
    // Hash order must never reach the sim: rides() is ascending by passenger.
    RideBook b;
    for (int p : {40, 9, 22, 3, 31}) CHECK(b.board(p, 100 + p % 3));
    const auto r = b.rides();
    CHECK(r.size() == 5u);
    for (std::size_t i = 1; i < r.size(); ++i) CHECK(r[i - 1].first < r[i].first);
}
