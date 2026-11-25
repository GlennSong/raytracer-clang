#include "../src/apps/citysim/city_sim.h"
#include "../src/apps/citysim/rail_sim.h"
#include "../src/apps/citysim/station_entrance.h"
#include "test_framework.h"
#include "model_scene.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace engine;
using namespace citysim;

// PEOPLE TAKING THE TRAIN (city_rail_riders.h): two towns no street joins, an oval of track past both with a station
// at each.

namespace {

// two grid towns, 1.3 km apart centre to centre, merged into one road file with nothing between them, built as a level's
// roads are and walked on the street map made from the model (model_scene.h, #306 piece 6 W2)
const citytest::ModelScene& twoTowns() {
    static RoadGraph g;
    if (!g.nodes.empty()) return citytest::graphScene("two_towns", g);
    for (int k = -1; k <= 1; k += 2) {
        citytest::GridCityParams p;
        p.center = Vec2(0, 650.0 * k);
        p.extent = 400;
        p.cellSize = 120;
        p.dropout = 0;
        p.seed = 4 + static_cast<uint32_t>(k + 1);
        const RoadGraph t = citytest::gridCity(p);
        std::vector<int> map;
        for (const RoadNode& n : t.nodes) map.push_back(g.addNode(n.pos));
        for (const RoadEdge& e : t.edges) g.addEdge(map[static_cast<std::size_t>(e.a)], map[static_cast<std::size_t>(e.b)], e.width, e.klass);
    }
    return citytest::graphScene("two_towns", g);
}

// rail_sim's test oval (two tracks, one each way; straights at z = +-200), its stations given ways in at the near
// edge of each town
std::vector<RailTrack> ovalWithWaysIn(const WalkPointMap& nav) {
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
        RailStop c{"Central", 330, 470, -1, 1.05, false, {}, {}};
        c.hasAccess = true; c.access = south;
        RailStop h{"Harbour", 330, 470, -1, 1.05, false, {}, {}};
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
    CitySim host;   // (the two towns' walk points: the sim sets them out on the model's walks)
    twoTowns().build(host, 1, 1);
    const WalkPointMap& nav = host.walkPoints();
    RailSim rail;
    rail.build(ovalWithWaysIn(nav), trainKindNamed("intercity"), 2);
    TrainRiders rr;
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
    Real secs = 0;
    CHECK(a >= 0 && b >= 0);
    CHECK(rr.doorToDoor(a, nav.nodes[static_cast<std::size_t>(a)], b, nav.nodes[static_cast<std::size_t>(b)], &secs));
    CHECK(secs > 0 && secs < 1800);   // door to door: the walks at both ends, the ways in and out, the wait, the ride
    // within one town: no train
    CHECK(!rr.doorToDoor(a, nav.nodes[static_cast<std::size_t>(a)], c.accessNode, c.access));
    // a platform spot stands on the platform: beside the track, at its height
    Vec2 pos, face;
    Real top = 0;
    CHECK(rr.platformSpot(0, 0, 7, pos, top, face));
    Real off = 1e9;
    for (const Vec3& q : rail.lines()[0].pts) off = std::min<Real>(off, std::hypot(q.x - pos.x, q.z - pos.y));
    CHECK(off > kPlatformGap + 0.7 && off < kPlatformGap + 2.4);   // in the band, clear of the edge and the stair
    CHECK(std::fabs(top - 1.05) < 1e-9);
}

// A RIDE, CARRIED (the one-brain plan step 4f): the rail lines priced in the transit timetable -- each stop waited for at
// its station's way in, the way in its access, a headway from the line's trains -- and one person carried from the
// Central way in to the Harbour one: in along the footway (not drawn), standing on the platform at its height, aboard the
// first train of the line to stand there with its doors open, off at Harbour and out on the street at its way in. The
// two reports come in order: boarded, then got off.
TEST_CASE(train_riders_carry_a_rider_between_two_ways_in) {
    CitySim host;
    twoTowns().build(host, 1, 1);
    const WalkPointMap& nav = host.walkPoints();
    RailSim rail;
    rail.build(ovalWithWaysIn(nav), trainKindNamed("intercity"), 2);
    TrainRiders rr;
    rr.build(&rail, nav);
    CHECK(rr.active());
    if (!rr.active()) return;
    TransitTimetable tt;
    rr.addTimetable(tt);
    CHECK(tt.lines.size() == 2u);   // one each way round the oval
    if (tt.lines.size() != 2u) return;
    for (const TransitTimetable::Line& l : tt.lines) {
        CHECK(l.network == "rail" && l.loop && l.stops.size() == 2u && l.headwaySeconds > 60);
        for (const TransitTimetable::Stop& st : l.stops) CHECK(st.accessSeconds >= 20 && st.toNextSeconds > 30);
    }
    // the line and its stops: Central (way in at the south town) to Harbour
    const int line = tt.lines[0].source;
    int board = -1, alight = -1;
    const std::vector<RailStop>& stops = rail.lines()[static_cast<std::size_t>(line)].stops;
    for (std::size_t k = 0; k < stops.size(); ++k) (stops[k].name == "Central" ? board : alight) = static_cast<int>(k);
    CHECK(board >= 0 && alight >= 0);
    std::vector<Agent> agents(3);
    const int who = 2;
    CHECK(rr.wait(who, line, board, alight));
    CHECK(rr.riding(who) && rr.rideOf(who)->phase == TrainRide::Phase::Entering);
    std::vector<RideEvent> events;
    bool sawPlatform = false, hidden = false;
    Real platformTop = -1;
    for (int t = 0; t < 60 * 60 * 10 && rr.riding(who); ++t) {   // 10 minutes at 60 Hz at most
        rail.step(1.0 / 60.0);
        for (const RideEvent& e : rr.tick(1.0 / 60.0, agents)) events.push_back(e);
        const TrainRide* r = rr.rideOf(who);
        if (r && r->phase == TrainRide::Phase::Entering) hidden = hidden || agents[who].indoors;
        if (r && r->phase == TrainRide::Phase::Waiting) { sawPlatform = !agents[who].indoors; platformTop = r->top; }
    }
    CHECK(hidden && sawPlatform && std::fabs(platformTop - 1.05) < 1e-9);
    CHECK(!rr.riding(who));
    CHECK(events.size() == 2u);
    if (events.size() == 2u) CHECK(events[0].agent == who && events[0].boarded && events[0].train && !events[1].boarded);
    const RailStation& harbour = rr.stations()[0].name == "Harbour" ? rr.stations()[0] : rr.stations()[1];
    CHECK((agents[who].pos - harbour.access).length() < 1e-9 && !agents[who].indoors);
    CHECK(rr.counts().waits == 1u && rr.counts().boardings == 1u && rr.counts().alightings == 1u);
}

// THE STATION ENTRANCE (Glenn, 2026-10-13: station buildings). A pavilion over a station's way in: the roof 3.6 m
// over the ground with the name board on it facing the street, the walls and machines solid, and a line of ticket
// gates across the way up with passages a walker fits through -- on a slope, every piece standing on its own ground.
// (Moved from test_city_bus.cpp with ADR-0460.)
TEST_CASE(a_station_entrance_stands_over_its_way_in) {
    citysim::StationEntrance e;
    e.name = "Saltwood";
    e.cls = "rail";
    e.foot = Vec3(100, 5, 200);
    e.along = Vec3(0, 0, 1);
    auto slope = [](double, double z) { return 5.0 + (z - 200.0) * 0.05; };   // rising 5% toward the platforms
    const citysim::EntranceMeshes m = citysim::stationEntranceMeshes(e, slope);
    CHECK(!m.body.indices.empty() && !m.glass.indices.empty() && !m.collider.indices.empty());
    // the roof over the highest column's ground (z 7: 5.35) by 3.6 m; nothing in the collider above head height but
    // the columns
    double top = -1e9, colTop = -1e9;
    for (const engine::Vertex& v : m.body.vertices) top = std::max(top, static_cast<double>(v.position.y));
    for (const engine::Vertex& v : m.collider.vertices) colTop = std::max(colTop, static_cast<double>(v.position.y));
    CHECK(top > 5.35 + 3.6 && top < 5.35 + 4.2);
    CHECK(std::fabs(colTop - (5.35 + 3.6)) < 0.05);
    // the gate line (z 4.4-5.9): the collider's boxes (24 vertices each), and the runs of floor across the line at
    // knee height that a 0.6 m capsule fits through
    struct Box { Vec3 lo, hi; };
    std::vector<Box> boxes;
    for (std::size_t i = 0; i + 24 <= m.collider.vertices.size(); i += 24) {
        Box b{Vec3(1e9, 1e9, 1e9), Vec3(-1e9, -1e9, -1e9)};
        for (std::size_t k = i; k < i + 24; ++k) {
            const Vec3 q = m.collider.vertices[k].position;
            b.lo = Vec3(std::min(b.lo.x, q.x), std::min(b.lo.y, q.y), std::min(b.lo.z, q.z));
            b.hi = Vec3(std::max(b.hi.x, q.x), std::max(b.hi.y, q.y), std::max(b.hi.z, q.z));
        }
        boxes.push_back(b);
    }
    const double zg = 205.1, yg = slope(100, zg) + 0.6;
    int passages = 0;
    bool wasBlocked = true;
    for (double x = 96.2; x < 103.8; x += 0.05) {
        bool blocked = false;
        for (const Box& b : boxes)
            blocked = blocked || (x + 0.3 > b.lo.x && x - 0.3 < b.hi.x && zg > b.lo.z && zg < b.hi.z && yg > b.lo.y && yg < b.hi.y);
        if (!blocked && wasBlocked) ++passages;
        wasBlocked = blocked;
    }
    CHECK(passages >= 4);
    // EVERY FACE OUTWARD (Glenn: "the geometry for the turnstyles ... have their normals inverted"): the engine's front
    // face is clockwise seen from the normal's side, so each triangle (a, b, c) has cross(c - a, b - a) along its normal
    int inward = 0, tris = 0;
    for (const engine::RenderMesh* mm : {&m.body, &m.glass}) {
        for (std::size_t i = 0; i + 2 < mm->indices.size(); i += 3) {
            const engine::Vertex& a = mm->vertices[mm->indices[i]];
            const Vec3 b = mm->vertices[mm->indices[i + 1]].position, c = mm->vertices[mm->indices[i + 2]].position;
            ++tris;
            if (engine::dot(engine::cross(c - a.position, b - a.position), a.normal) < 0) ++inward;
        }
    }
    CHECK(tris > 100);
    CHECK(inward == 0);
    // the board on the roof, facing back toward the street (-along)
    CHECK(m.board.m[2][2] < -0.99);
    CHECK(m.board.m[1][3] > 5.0 + 3.6 - 1.7);
}
