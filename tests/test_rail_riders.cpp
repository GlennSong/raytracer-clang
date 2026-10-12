#include "../src/apps/citysim/city_sim.h"
#include "../src/apps/citysim/rail_sim.h"
#include "test_framework.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace engine;
using namespace citysim;

// PEOPLE TAKING THE TRAIN (city_rail_riders.h): two towns no street joins, an oval of track past both with a station
// at each.

namespace {

// two grid towns, 1.3 km apart centre to centre, merged into one graph with nothing between them
NavGraph twoTowns() {
    RoadGraph g;
    for (int k = -1; k <= 1; k += 2) {
        GridRoadParams p;
        p.center = Vec2(0, 650.0 * k);
        p.extent = 400;
        p.cellSize = 120;
        p.dropout = 0;
        p.seed = 4 + static_cast<uint32_t>(k + 1);
        const RoadGraph t = gridRoads(p);
        std::vector<int> map;
        for (const RoadNode& n : t.nodes) map.push_back(g.addNode(n.pos));
        for (const RoadEdge& e : t.edges) g.addEdge(map[static_cast<std::size_t>(e.a)], map[static_cast<std::size_t>(e.b)], e.width, e.klass);
    }
    return buildNavGraph(g);
}

// rail_sim's test oval (two tracks, one each way; straights at z = +-200), its stations given ways in at the near
// edge of each town
std::vector<RailTrack> ovalWithWaysIn(const NavGraph& nav) {
    auto nearestNode = [&](Vec2 p) {
        Vec2 best = nav.nodes.front();
        for (const Vec2& n : nav.nodes)
            if (std::hypot(n.x - p.x, n.y - p.y) < std::hypot(best.x - p.x, best.y - p.y)) best = n;
        return best;
    };
    const Vec2 south = nearestNode(Vec2(0, -250)), north = nearestNode(Vec2(0, 250));
    std::vector<RailTrack> out;
    const double pi = 3.14159265358979;
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
        RailStop c{"Central", 330, 470, -1, 1.05};
        c.hasAccess = true; c.access = south;
        RailStop h{"Harbour", 330, 470, -1, 1.05};
        h.hasAccess = true; h.access = north;
        std::vector<RailTrack> ring;
        ring.push_back(piece("south", [&](double u) { return Vec3(-400 + 800 * u, 0, -r); }, 800, {c}));
        ring.push_back(piece("east", [&](double u) { const double a = -pi / 2 + pi * u; return Vec3(400 + r * std::cos(a), 0, r * std::sin(a)); }, pi * r, {}));
        ring.push_back(piece("north", [&](double u) { return Vec3(400 - 800 * u, 0, r); }, 800, {h}));
        ring.push_back(piece("west", [&](double u) { const double a = pi / 2 + pi * u; return Vec3(-400 + r * std::cos(a), 0, r * std::sin(a)); }, pi * r, {}));
        if (dir < 0) {
            for (RailTrack& t : ring) {
                std::reverse(t.pts.begin(), t.pts.end());
                for (RailStop& s : t.stops) { const double a = 800 - s.s1, b = 800 - s.s0; s.s0 = a; s.s1 = b; }
            }
            std::reverse(ring.begin(), ring.end());
        }
        out.insert(out.end(), ring.begin(), ring.end());
    }
    return out;
}

}  // namespace

TEST_CASE(rail_riders_pin_each_station_to_its_town) {
    const NavGraph nav = twoTowns();
    RailSim rail;
    rail.build(ovalWithWaysIn(nav), trainKindNamed("intercity"), 2);
    RailRiders rr;
    rr.build(&rail, nav);
    CHECK(rr.active());
    CHECK(rr.stations().size() == 2);
    if (rr.stations().size() != 2) return;
    const RailStation& c = rr.stations()[0].name == "Central" ? rr.stations()[0] : rr.stations()[1];
    const RailStation& h = rr.stations()[0].name == "Central" ? rr.stations()[1] : rr.stations()[0];
    CHECK(c.accessNode >= 0 && h.accessNode >= 0);
    CHECK(c.network >= 0 && h.network >= 0 && c.network != h.network);   // two towns, two networks
    // both lines stop at both
    for (int s : c.stopOnLine) CHECK(s >= 0);
    // a trip from deep in one town to deep in the other goes by train, from the near station to the far one
    int a = -1, b = -1;
    for (int i = 0; i < nav.nodeCount(); ++i) {
        if (nav.nodes[static_cast<std::size_t>(i)].y < -900 && a < 0 && rr.networkOf(i) == c.network) a = i;
        if (nav.nodes[static_cast<std::size_t>(i)].y > 900 && b < 0 && rr.networkOf(i) == h.network) b = i;
    }
    RailTrip t;
    Real secs = 0;
    CHECK(a >= 0 && b >= 0);
    CHECK(rr.plan(a, nav.nodes[static_cast<std::size_t>(a)], b, nav.nodes[static_cast<std::size_t>(b)], t, &secs));
    CHECK(rr.stations()[static_cast<std::size_t>(t.from)].name == "Central");
    CHECK(rr.stations()[static_cast<std::size_t>(t.to)].name == "Harbour");
    CHECK(secs > 0 && secs < 600);
    // within one town: no train
    CHECK(!rr.plan(a, nav.nodes[static_cast<std::size_t>(a)], c.accessNode, c.access, t));
    // a platform spot stands on the platform: beside the track, at its height
    Vec2 pos, face;
    Real top = 0;
    CHECK(rr.platformSpot(0, 0, 7, pos, top, face));
    Real off = 1e9;
    for (const Vec3& q : rail.lines()[0].pts) off = std::min<Real>(off, std::hypot(q.x - pos.x, q.z - pos.y));
    CHECK(off > kPlatformGap + 0.7 && off < kPlatformGap + 2.4);   // in the band, clear of the edge and the stair
    CHECK(std::fabs(top - 1.05) < 1e-9);
}

TEST_CASE(people_sent_to_the_other_town_go_by_train_and_arrive) {
    const NavGraph nav = twoTowns();
    RailSim rail;
    rail.build(ovalWithWaysIn(nav), trainKindNamed("intercity"), 2);
    CitySim sim;
    sim.build(nav, 0, 120, 17);
    sim.setRail(&rail);
    CHECK(sim.railRiders().active());

    const RailRiders& rr = sim.railRiders();
    std::vector<int> startNet(sim.agents().size(), -1);
    for (std::size_t i = 0; i < sim.agents().size(); ++i) {
        const Vec2 p = sim.agents()[i].pos;
        startNet[i] = p.y < 0 ? 0 : 1;
    }
    // a dozen from each town to the middle of the other; what happens after is the sim's own
    int sent = 0;
    for (std::size_t i = 0; i < sim.agents().size() && sent < 24; ++i)
        if (sim.sendAgentByTrain(static_cast<int>(i), Vec2(0, startNet[i] == 0 ? 650.0 : -650.0))) ++sent;
    std::printf("    [rail] sent %d by train\n", sent);
    CHECK(sent >= 12);
    // within a town, no train
    CHECK(!sim.sendAgentByTrain(23, sim.agents()[23].pos + Vec2(150, 0)));
    long waitedOnPlatform = 0, rodeFar = 0, arrivedOtherTown = 0;
    std::vector<char> seenWait(sim.agents().size(), 0), seenRide(sim.agents().size(), 0), seenArrive(sim.agents().size(), 0);
    std::vector<Vec2> boardedAt(sim.agents().size(), Vec2(0, 0));
    for (int tick = 0; tick < 60000; ++tick) {
        rail.step(0.05);
        sim.step(0.05, 0.05);
        for (std::size_t i = 0; i < sim.agents().size(); ++i) {
            const int ai = static_cast<int>(i);
            const RailTrip* t = rr.tripOf(ai);
            const Vec2 p = sim.agents()[i].pos;
            if (t && t->phase == RailTrip::Phase::Waiting && !seenWait[i]) {
                seenWait[i] = 1; ++waitedOnPlatform;
                CHECK(sim.pedVisible(ai));   // seen on the platform
            }
            if (t && t->phase == RailTrip::Phase::Aboard) {
                CHECK(!sim.pedVisible(ai));  // not on foot while aboard
                if (!seenRide[i]) { seenRide[i] = 1; boardedAt[i] = p; }
                else if (std::hypot(p.x - boardedAt[i].x, p.y - boardedAt[i].y) > 300 && seenRide[i] == 1) { seenRide[i] = 2; ++rodeFar; }
            }
            // off the train and out on the other town's street
            if (!t && seenRide[i] && !seenArrive[i] && (p.y < 0 ? 0 : 1) != startNet[i] && std::fabs(p.y) > 240) {
                seenArrive[i] = 1; ++arrivedOtherTown;
            }
        }
    }
    const RailRiders::Stats& st = rr.stats;
    std::printf("    [rail] planned %ld entered %ld boarded %ld set down %ld gave up %ld | waited %ld rode >300 m %ld "
                "arrived %ld\n", st.planned, st.entered, st.boarded, st.setDown, st.gaveUp, waitedOnPlatform, rodeFar,
                arrivedOtherTown);
    CHECK(st.planned > 0);          // trips into the other town chose the train
    CHECK(waitedOnPlatform > 0);    // walked to the station and in
    CHECK(st.boarded > 0);          // a train stopped and took them
    CHECK(rodeFar > 0);             // and carried them, not just flagged them
    CHECK(st.setDown > 0);          // and let them off
    CHECK(arrivedOtherTown > 0);    // in the other town, on its streets
}
