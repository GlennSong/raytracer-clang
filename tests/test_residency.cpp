#include "test_framework.h"
#include "../src/engine/residency.h"
#include "../src/job_system.h"
#include <atomic>
#include <chrono>
#include <thread>
using namespace engine;

// The residency service (ADR-0095): what is resident follows the camera, with hysteresis; an
// outer tier lets go inside its inner edge; a jump loads everything in range at once; a budget
// defers the rest but never the nearest; two-phase items prepare on workers and commit later.

namespace {
Residency::Item item(double x, int* loads, int* unloads, double within = 100, double beyond = 150) {
    Residency::Item it;
    it.client = "test";
    it.center = Vec3(x, 0, 0);
    it.radius = 0;
    it.loadWithin = within;
    it.dropBeyond = beyond;
    it.load = [loads]() { ++*loads; return std::size_t(10); };
    it.unload = [unloads]() { ++*unloads; };
    return it;
}
}  // namespace

TEST_CASE(residency_follows_the_camera_with_hysteresis) {
    Residency r;
    int loads = 0, unloads = 0;
    r.add(item(0, &loads, &unloads));
    r.update(Vec3(0, 0, 0), 0);                  // first update: in range, loads
    CHECK(loads == 1 && r.stats().at("test").resident == 1);
    r.update(Vec3(120, 0, 0), 0);                // past loadWithin, inside dropBeyond: kept
    CHECK(unloads == 0);
    r.update(Vec3(160, 0, 0), 0);                // past dropBeyond: let go
    CHECK(unloads == 1 && r.stats().at("test").resident == 0);
    r.update(Vec3(120, 0, 0), 0);                // back inside dropBeyond but not loadWithin: stays out
    CHECK(loads == 1);
}

TEST_CASE(residency_outer_tier_lets_go_inside_its_inner_edge) {
    Residency r;
    int loads = 0, unloads = 0;
    Residency::Item it = item(0, &loads, &unloads, 500, 600);
    it.dropWithin = 200;                          // a facade tier: not drawn up close
    r.add(it);
    r.update(Vec3(300, 0, 0), 0);
    CHECK(loads == 1);
    r.update(Vec3(150, 0, 0), 0);                 // walked in past the inner edge (a jump-free step)
    CHECK(unloads == 1);
}

TEST_CASE(residency_budget_defers_all_but_the_nearest_unless_it_is_a_jump) {
    Residency r;
    int loads = 0, unloads = 0;
    for (int k = 0; k < 5; ++k) {
        Residency::Item it = item(k * 10.0, &loads, &unloads, 1000, 1100);
        it.load = [&loads]() { ++loads; std::this_thread::sleep_for(std::chrono::milliseconds(3)); return std::size_t(1); };
        r.add(it);
    }
    r.update(Vec3(0, 0, 0), 0);                   // first update is a jump: everything
    CHECK(loads == 5);
    Residency r2;
    loads = 0;
    for (int k = 0; k < 5; ++k) {
        Residency::Item it = item(k * 10.0, &loads, &unloads, 50, 1100);
        it.load = [&loads]() { ++loads; std::this_thread::sleep_for(std::chrono::milliseconds(3)); return std::size_t(1); };
        r2.add(it);
    }
    r2.update(Vec3(-1000, 0, 0), 0);              // far away: nothing
    r2.update(Vec3(-900, 0, 0), 1.0);             // still nothing in range
    for (int step = 1; step <= 9; ++step) r2.update(Vec3(-900 + step * 100.0, 0, 0), 1.0);   // walk in, 100 m steps
    // all in range now, but a 1 ms budget loads one a frame -- the nearest first
    CHECK(loads >= 1 && loads < 5);
    for (int f = 0; f < 5; ++f) r2.update(Vec3(0, 0, 0), 1.0);
    CHECK(loads == 5);
}

TEST_CASE(residency_two_phase_prepares_off_thread_and_commits_later) {
    JobSystem jobs(2);
    Residency r;
    std::atomic<int> prepared{0};
    int committed = 0, unloads = 0, loads = 0;
    Residency::Item it = item(0, &loads, &unloads, 100, 150);
    it.load = nullptr;
    it.prepare = [&prepared]() { ++prepared; return std::shared_ptr<void>(std::make_shared<int>(7)); };
    it.commit = [&committed](std::shared_ptr<void> p) { committed += *static_cast<int*>(p.get()); return std::size_t(3); };
    r.add(it);
    r.update(Vec3(1000, 0, 0), 4.0, &jobs);       // first update (a jump) far away: nothing wanted
    r.update(Vec3(900, 0, 0), 4.0, &jobs);
    for (int step = 1; step <= 9; ++step) r.update(Vec3(900 - step * 100.0, 0, 0), 4.0, &jobs);   // arrive, no jump
    // prepared on a worker; committed by a later update
    for (int f = 0; f < 200 && committed == 0; ++f) { std::this_thread::sleep_for(std::chrono::milliseconds(2)); r.update(Vec3(0, 0, 0), 4.0, &jobs); }
    CHECK(prepared == 1 && committed == 7);
    CHECK(r.stats().at("test").commits == 1);
}
