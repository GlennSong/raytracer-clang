// The Intelligent Driver Model (roads-v2 plan §3.1) — the sim's anti-pile-up
// backbone. These prove the LAW's safety properties headlessly: a follower
// approaching a standing leader stops short without contact, a platoon behind
// a braking leader never collides, and free road relaxes to the desired speed.
#include "test_framework.h"

#include "../src/engine/ai/idm.h"
#include <limits>
#include <vector>

using namespace engine;

TEST_CASE(idm_free_road_relaxes_to_desired_speed) {
    Real v = 0;
    const Real inf = std::numeric_limits<Real>::infinity();
    for (int i = 0; i < 4000; ++i) v = idmStep(v, 14.0, inf, 0.0, 0.05, 8.0);
    CHECK(v > 13.5);
    CHECK(v < 14.01);                 // never overshoots the limit
}

TEST_CASE(idm_stops_short_of_a_standing_leader) {
    // Follower at 14 m/s, standing leader 80 m ahead (net gap). It must come
    // to rest without the gap ever reaching zero.
    Real v = 14.0, s = 80.0;
    Real minS = s;
    for (int i = 0; i < 8000; ++i) {
        const Real dt = 0.05;
        v = idmStep(v, 14.0, s, v - 0.0, dt, 8.0);
        s -= v * dt;
        minS = std::min(minS, s);
    }
    CHECK(v < 0.05);                  // came to rest
    CHECK(minS > 0.3);                // and never touched the leader
}

TEST_CASE(idm_platoon_never_collides_when_the_leader_brakes) {
    // Ten cars at speed, 12 m net gaps; the leader brakes to a dead stop.
    // No pair may ever close below a bumper's width — the pile-up test.
    const int N = 10;
    std::vector<Real> pos(N), vel(N, 13.0);
    for (int i = 0; i < N; ++i) pos[i] = -i * 16.0;    // car 0 leads
    Real worst = 1e9;
    for (int stepI = 0; stepI < 6000; ++stepI) {
        const Real dt = 0.05;
        vel[0] = std::max(Real(0), vel[0] - 6.0 * dt);   // leader slams to 0
        pos[0] += vel[0] * dt;
        for (int i = 1; i < N; ++i) {
            const Real net = pos[i - 1] - pos[i] - 4.0;  // 4 m of car body
            vel[i] = idmStep(vel[i], 13.0, net, vel[i] - vel[i - 1], dt, 8.0);
            pos[i] += vel[i] * dt;
            worst = std::min(worst, net);
        }
    }
    CHECK(worst > 0.2);               // no pair ever collided
    for (int i = 1; i < N; ++i) CHECK(vel[i] < 0.1);   // all came to rest
}
