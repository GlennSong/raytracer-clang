#include "../src/apps/citysim/city_dispatch.h"
#include "test_framework.h"

using namespace citysim;

TEST_CASE(dispatch_queues_a_hail_and_assigns_it) {
    Dispatch d;
    CHECK(d.empty());
    CHECK(d.hail(5, 100, 200));
    CHECK(d.waiting() == 1);
    CHECK(d.driverFor(5) == -1);            // nobody coming yet

    const int took = d.assign(9, [](int) { return 1.0; });
    CHECK(took == 5);
    CHECK(d.waiting() == 0);
    CHECK(d.assigned() == 1);
    CHECK(d.driverFor(5) == 9);
    const Fare* f = d.fareOf(9);
    CHECK(f != nullptr);
    CHECK(f->passenger == 5);
    CHECK(f->pickup == 100);
    CHECK(f->dropoff == 200);

    d.complete(9);
    CHECK(d.fareOf(9) == nullptr);
    CHECK(d.driverFor(5) == -1);
    CHECK(d.empty());
}

TEST_CASE(dispatch_refuses_impossible_hails) {
    Dispatch d;
    CHECK(!d.hail(-1, 10, 20));             // no such passenger
    CHECK(!d.hail(3, -1, 20));              // nowhere to be picked up
    CHECK(!d.hail(3, 10, -1));              // nowhere to go
    CHECK(d.hail(3, 10, 20));
    CHECK(!d.hail(3, 11, 21));              // already waiting: one hail each
    CHECK(d.waiting() == 1);

    CHECK(d.assign(4, [](int) { return 1.0; }) == 3);
    CHECK(!d.hail(3, 12, 22));              // already has a car coming
    // A driver already carrying a fare does not take another.
    CHECK(d.hail(8, 30, 40));
    CHECK(d.assign(4, [](int) { return 1.0; }) == -1);
}

TEST_CASE(dispatch_takes_the_cheapest_and_breaks_ties_by_index) {
    Dispatch d;
    CHECK(d.hail(11, 500, 900));
    CHECK(d.hail(2, 700, 900));
    CHECK(d.hail(7, 600, 900));
    // Cost keyed on the PICKUP node: 600 is nearest.
    const int near = d.assign(1, [](int pickup) {
        return pickup == 600 ? 1.0 : pickup == 700 ? 5.0 : 9.0;
    });
    CHECK(near == 7);

    // Both remaining cost the same -> the LOWER passenger index wins, so the
    // result cannot depend on hash order (ADR-0002).
    const int tie = d.assign(2, [](int) { return 3.0; });
    CHECK(tie == 2);
    CHECK(d.waiting() == 1);
    CHECK(d.waitingPassengers().size() == 1);
    CHECK(d.waitingPassengers()[0] == 11);
}

TEST_CASE(dispatch_skips_unreachable_fares) {
    Dispatch d;
    CHECK(d.hail(4, 10, 20));
    CHECK(d.hail(6, 30, 40));
    // A negative cost means "no route" — that fare is not taken, and the
    // driver still gets the other one rather than coming away empty.
    const int took = d.assign(1, [](int pickup) {
        return pickup == 10 ? -1.0 : 2.0;
    });
    CHECK(took == 6);
    CHECK(d.waiting() == 1);

    // Everything unreachable: nobody is assigned, and the queue is untouched.
    CHECK(d.assign(2, [](int) { return -1.0; }) == -1);
    CHECK(d.waiting() == 1);
    CHECK(d.driverFor(4) == -1);
}

TEST_CASE(dispatch_cancels_only_while_still_waiting) {
    Dispatch d;
    CHECK(d.hail(3, 10, 20));
    CHECK(d.hail(5, 30, 40));
    d.cancel(3);
    CHECK(d.waiting() == 1);
    d.cancel(99);                           // never hailed: harmless
    CHECK(d.waiting() == 1);

    CHECK(d.assign(1, [](int) { return 1.0; }) == 5);
    d.cancel(5);                            // already picked up: not a cancel
    CHECK(d.fareOf(1) != nullptr);
    CHECK(d.driverFor(5) == 1);

    d.clear();
    CHECK(d.empty());
    CHECK(d.fareOf(1) == nullptr);
}

TEST_CASE(dispatch_waiting_list_is_ascending_whatever_the_hail_order) {
    Dispatch d;
    for (int p : {40, 3, 17, 1, 28}) CHECK(d.hail(p, 100 + p, 900));
    const std::vector<int> w = d.waitingPassengers();
    CHECK(w.size() == 5);
    for (std::size_t i = 1; i < w.size(); ++i) CHECK(w[i - 1] < w[i]);
    CHECK(w[0] == 1);
    CHECK(w[4] == 40);
}
