#include "../src/apps/citysim/rail_sim.h"
#include "test_framework.h"

#include <algorithm>
#include <cmath>

using namespace engine;
using namespace citysim;

// TRAINS on their own: tracks as the road builder hands them on (rail_track.h), joined into lines, run around the
// clock. No world, no meshes.

namespace {

// An oval of two tracks, one each way, cut into four pieces per track the way four graph edges would leave it:
// straights 800 m at z = +-200 (+-2 for each track), semicircles of radius 200 (+-2).
std::vector<RailTrack> ovalTracks() {
    std::vector<RailTrack> out;
    for (int dir = -1; dir <= 1; dir += 2) {
        const double r = 200.0 + dir * 2.0;
        auto piece = [&](const char* id, auto&& fn, double len, std::vector<RailStop> stops) {
            RailTrack t;
            t.edge = id;
            t.cls = "rail";
            t.dir = dir;
            const int n = static_cast<int>(std::ceil(len / 2.0)) + 1;
            for (int k = 0; k < n; ++k) t.pts.push_back(fn(static_cast<double>(k) / (n - 1)));
            t.stops = std::move(stops);
            return t;
        };
        const double pi = 3.14159265358979;
        // counter-clockwise for dir +1, clockwise for -1: build ccw and reverse
        std::vector<RailTrack> ring;
        ring.push_back(piece("south", [&](double u) { return Vec3(-400 + 800 * u, 0, -r); }, 800, {{"Central", 330, 470, -1, 1.05}}));
        ring.push_back(piece("east", [&](double u) { const double a = -pi / 2 + pi * u; return Vec3(400 + r * std::cos(a), 0, r * std::sin(a)); }, pi * r, {}));
        ring.push_back(piece("north", [&](double u) { return Vec3(400 - 800 * u, 0, r); }, 800, {{"Harbour", 330, 470, -1, 1.05}}));
        ring.push_back(piece("west", [&](double u) { const double a = pi / 2 + pi * u; return Vec3(-400 + r * std::cos(a), 0, r * std::sin(a)); }, pi * r, {}));
        if (dir < 0) {
            for (RailTrack& t : ring) {
                std::reverse(t.pts.begin(), t.pts.end());
                const double len = 800;
                for (RailStop& s : t.stops) { const double a = len - s.s1, b = len - s.s0; s.s0 = a; s.s1 = b; }
            }
            std::reverse(ring.begin(), ring.end());
        }
        out.insert(out.end(), ring.begin(), ring.end());
    }
    return out;
}

}  // namespace

TEST_CASE(rail_tracks_join_into_one_loop_each_way) {
    RailSim sim;
    sim.build(ovalTracks(), trainKindNamed("intercity"), 2);
    CHECK(sim.lines().size() == 2);
    for (const RailLine& L : sim.lines()) {
        CHECK(L.loop);
        CHECK(L.length > 2800 && L.length < 2900);
        CHECK(L.stops.size() == 2);
        // a curve allows less than the line speed, a straight the line speed
        CHECK(L.speedLimit(400.0) < 20.0 || L.speedLimit(1200.0) < 20.0);
    }
    CHECK(sim.trains().size() == 4);
}

TEST_CASE(trains_stop_at_every_platform_with_their_doors_open_there) {
    RailSim sim;
    sim.build(ovalTracks(), trainKindNamed("intercity"), 2);
    std::vector<int> stopsSeen(sim.trains().size(), 0);
    std::vector<char> wasDwelling(sim.trains().size(), 0);
    int doorsOpenWhileMoving = 0;
    for (int step = 0; step < 30 * 60 * 12; ++step) {   // twelve minutes at 30 Hz (a lap and its two dwells is ~4)
        sim.step(1.0 / 30.0);
        for (std::size_t i = 0; i < sim.trains().size(); ++i) {
            const Train& t = sim.trains()[i];
            if (t.dwelling && !wasDwelling[i]) {
                ++stopsSeen[i];
                const RailStop& st = sim.lines()[static_cast<std::size_t>(t.line)].stops[static_cast<std::size_t>(t.nextStop)];
                CHECK(t.s <= st.s1 && t.s >= st.s0 + t.kind.length() - 5.0);   // the whole train beside the platform
            }
            wasDwelling[i] = t.dwelling;
            if (t.doorSide != 0 && t.v > 0) ++doorsOpenWhileMoving;
        }
    }
    CHECK(doorsOpenWhileMoving == 0);
    for (int n : stopsSeen) CHECK(n >= 4);   // round the loop more than once: both stations, again
}

TEST_CASE(carriages_couple_end_to_end_on_the_curve) {
    // the gangway hand-over relies on it: one car's rear and the next car's front are the same place
    RailSim sim;
    sim.build(ovalTracks(), trainKindNamed("intercity"), 1);
    double worst = 0;
    for (int step = 0; step < 30 * 120; ++step) {
        sim.step(1.0 / 30.0);
        if (step % 15) continue;
        for (int c = 1; c + 1 < sim.carCount(0); ++c) {
            const CarPose a = sim.carPose(0, c, 0.5), b = sim.carPose(0, c + 1, 0.5);
            const double gap = sim.trains()[0].kind.gap;
            const Vec3 rearA = a.m.transformPoint(Vec3(0, 0, static_cast<Real>(-a.length / 2 - gap / 2)));
            const Vec3 frontB = b.m.transformPoint(Vec3(0, 0, static_cast<Real>(b.length / 2 + gap / 2)));
            worst = std::max(worst, static_cast<double>((rearA - frontB).length()));
        }
    }
    CHECK(worst < 0.35);   // a 24 m car's chord on a 200 m curve: the bodies' ends barely part
}
