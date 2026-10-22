#include "../src/apps/citysim/rail_sim.h"
#include "../src/apps/citysim/rail_board.h"
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
        ring.push_back(piece("south", [&](double u) { return Vec3(-400 + 800 * u, 0, -r); }, 800, {{"Central", 330, 470, -1, 1.05, false, {}, {}}}));
        ring.push_back(piece("east", [&](double u) { const double a = -pi / 2 + pi * u; return Vec3(400 + r * std::cos(a), 0, r * std::sin(a)); }, pi * r, {}));
        ring.push_back(piece("north", [&](double u) { return Vec3(400 - 800 * u, 0, r); }, 800, {{"Harbour", 330, 470, -1, 1.05, false, {}, {}}}));
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

TEST_CASE(each_line_runs_the_stock_its_track_class_asks_for) {
    // the oval as a main line, and a copy of it 1 km off as a city's light rail: two classes, never joined
    std::vector<RailTrack> tracks = ovalTracks();
    const std::size_t n = tracks.size();
    for (std::size_t i = 0; i < n; ++i) {
        RailTrack t = tracks[i];
        t.cls = "light_rail";
        t.edge = "lr_" + t.edge;
        for (Vec3& p : t.pts) p.z += 1000.0;
        tracks.push_back(t);
    }
    RailSim sim;
    sim.build(tracks, [](const std::string& cls) { return trainKindNamed(cls == "light_rail" ? "light_rail" : "intercity"); },
              [](double, const TrainKind& k) { return k.name == "light_rail" ? 3 : 2; });
    CHECK(sim.lines().size() == 4);   // two classes x two directions
    int intercity = 0, light = 0;
    for (const Train& t : sim.trains()) {
        const RailLine& L = sim.lines()[static_cast<std::size_t>(t.line)];
        if (L.cls == "light_rail") { CHECK(t.kind.name == "light_rail"); ++light; }
        else { CHECK(t.kind.name == "intercity"); ++intercity; }
    }
    CHECK(intercity == 4);
    CHECK(light == 6);
}

// THE PLATFORM BOARD (rail_board.h; Glenn, 2026-10-12: "a similar thing to the buses when you're on a station
// platform"): standing on Central's platform finds it (not beside the track, not under it); the board lists both
// lines calling there, this platform's first, each toward the other station; and the minutes it gives come true --
// the next train stands at the platform within a third of the time it said.
TEST_CASE(a_platform_board_finds_its_platform_and_its_minutes_come_true) {
    RailSim sim;
    sim.build(ovalTracks(), trainKindNamed("intercity"), 2);
    for (int i = 0; i < 30 * 40; ++i) sim.step(1.0 / 30.0);   // trains on their way
    const RailLine& L = sim.lines()[0];
    int k = -1;
    for (std::size_t j = 0; j < L.stops.size(); ++j) if (L.stops[j].name == "Central") k = static_cast<int>(j);
    CHECK(k >= 0);
    if (k < 0) return;
    const RailStop& st = L.stops[static_cast<std::size_t>(k)];
    // a point on the platform: the middle of its span, 3 m out on its side, a body's height over its top
    const double sm = 0.5 * (st.s0 + st.s1);
    const Vec3 a = L.at(sm), b = L.at(sm + 1.0);
    double fx = b.x - a.x, fz = b.z - a.z; const double fl = std::hypot(fx, fz); fx /= fl; fz /= fl;
    const Vec3 out(fz * st.side, 0, -fx * st.side);
    int line = -1, stop = -1;
    CHECK(platformAt(sim, a + out * (kPlatformGap + 2.0) + Vec3(0, static_cast<Real>(st.top + 0.9), 0), line, stop));
    CHECK(line == 0 && stop == k);
    CHECK(!platformAt(sim, a + out * (kPlatformGap + 8.0) + Vec3(0, static_cast<Real>(st.top + 0.9), 0), line, stop));   // past its back
    CHECK(!platformAt(sim, a + out * (kPlatformGap + 2.0) + Vec3(0, -6, 0), line, stop));            // under it
    const PlatformBoard board = platformBoard(sim, 0, k);
    CHECK(board.station == "Central");
    CHECK(board.departures.size() == 4);   // two lines, two trains each
    if (board.departures.empty()) return;
    CHECK(board.departures.front().thisPlatform && board.departures.front().line == 0);
    for (const PlatformBoard::Departure& d : board.departures) CHECK(d.toward == "Harbour");
    // the soonest on this platform's line: how long it really takes
    const PlatformBoard::Departure next = board.departures.front();
    CHECK(next.eta > 0);
    double t = 0;
    while (t < next.eta * 2 + 120) {
        const Train& T = sim.trains()[static_cast<std::size_t>(next.train)];
        if (T.dwelling && T.nextStop == k) break;
        sim.step(1.0 / 30.0);
        t += 1.0 / 30.0;
    }
    std::printf("    [board] said %.0f s, took %.0f s\n", next.eta, t);
    CHECK(std::fabs(t - next.eta) < next.eta / 3 + 20);
    CHECK(trainEta(sim, next.train, k) == 0);
}
