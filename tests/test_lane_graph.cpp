// The lane graph's types, save and load, structural check and text dump (issue #236, driving-model-design.md 5.2).

#include "test_framework.h"

#include "../src/engine/bundle/bundle.h"
#include "../src/engine/procgen/city/roads/lane_graph.h"

#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace engine;

namespace {

// THE TEST CROSSROADS: two two-way roads crossing at one junction, one lane each way on each road. The four arms are
// numbered east 0, north 1, west 2, south 3; each has an arriving lane (path 2 * arm) and a leaving lane (path
// 2 * arm + 1), each in its own section (the same number). The west arm's arriving lane has a lane for leaving cars at
// its kerb (path 8, in section 4 with path 4). Paths 9 to 20 are the twelve connectors of a four-way where every turn
// is allowed: from each arriving lane into the three other arms' leaving lanes. Arms are numbered anticlockwise and
// cars keep to the kerb side as in the UK and Japan, so arriving from arm a, arm a + 3 is the near turn, a + 2
// straight on, a + 1 the far turn.
constexpr int kArms = 4;
constexpr Real kLaneWidth = 3.0;
constexpr Real kKerbLaneWidth = 2.4;
constexpr Real kJunctionHalf = 5.0;    // where the arms meet the junction
constexpr Real kArmEnd = 45.0;         // where they start
constexpr Real kBoxHalf = 6.0;
constexpr Real kLaneSpeed = 13.4;      // metres per second (30 mph)
constexpr Real kTurnSpeed = 6.7;
constexpr Real kNearRadius = 6.0;
constexpr Real kFarRadius = 9.0;
constexpr uint32_t kFirstConnector = 9;
constexpr uint32_t kKerbLane = 8;

Vec2 armDir(int a) {
    const Vec2 dirs[kArms] = {Vec2(1, 0), Vec2(0, 1), Vec2(-1, 0), Vec2(0, -1)};
    return dirs[a];
}
// Each lane lies to the kerb side of the road's middle: an arriving lane (heading -d) at (d.y, -d.x) from it, a
// leaving one (heading d) at (-d.y, d.x).
Vec2 arriveSide(int a) { return Vec2(armDir(a).y, -armDir(a).x); }
Vec2 leaveSide(int a) { return Vec2(-armDir(a).y, armDir(a).x); }
uint32_t connectorId(int from, int to) {
    const int k = (to - from + kArms) % kArms - 1;   // 0: arm + 1, 1: arm + 2, 2: arm + 3
    return kFirstConnector + static_cast<uint32_t>(from * (kArms - 1) + k);
}
Turn turnOf(int from, int to) {
    const int k = (to - from + kArms) % kArms;
    return k == 1 ? Turn::FarTurn : k == 2 ? Turn::Straight : Turn::NearTurn;
}

LaneGraph makeCrossroads() {
    LaneGraph g;
    g.laneTagNames = {"through", "turn", "parking"};
    g.vehicleTagNames = {"car", "bus", "bike"};
    g.regulationKindNames = {"stop", "speed_limit"};
    g.controlNames = {"all_way_stop"};
    g.roadClassNames = {"street"};
    const TagMask through = TagMask(1) << LaneGraph::nameIndex(g.laneTagNames, "through");
    const TagMask turn = TagMask(1) << LaneGraph::nameIndex(g.laneTagNames, "turn");
    const TagMask kerbLane = TagMask(1) << LaneGraph::nameIndex(g.laneTagNames, "parking");
    const TagMask car = TagMask(1) << LaneGraph::nameIndex(g.vehicleTagNames, "car");
    const TagMask everyone = car | (TagMask(1) << LaneGraph::nameIndex(g.vehicleTagNames, "bus")) |
                             (TagMask(1) << LaneGraph::nameIndex(g.vehicleTagNames, "bike"));

    auto lane = [&](const Vec2& from, const Vec2& to, Real width, TagMask tags, TagMask who, uint32_t section,
                    uint32_t index) {
        Path p;
        p.kind = PathKind::Lane;
        p.laneTags = tags;
        p.vehicleTags = who;
        p.curve = g.addCurve({from, to}, {0, 0});
        p.length = g.along[p.curve.end() - 1];
        p.width = width;
        p.tightestRadius = kNoBend;
        p.speedLimit = kLaneSpeed;
        p.section = SectionId(section);
        p.indexInSection = index;
        g.paths.push_back(p);
    };
    // Lanes 0..7, then the kerb lane 8 beside lane 4.
    for (int a = 0; a < kArms; ++a) {
        const Vec2 d = armDir(a), in = arriveSide(a) * (kLaneWidth / 2), out = leaveSide(a) * (kLaneWidth / 2);
        lane(d * kArmEnd + in, d * kJunctionHalf + in, kLaneWidth, through, everyone, static_cast<uint32_t>(2 * a), 0);
        lane(d * kJunctionHalf + out, d * kArmEnd + out, kLaneWidth, through, everyone, static_cast<uint32_t>(2 * a + 1), 0);
    }
    {
        const Vec2 d = armDir(2), kerb = arriveSide(2) * (kLaneWidth + kKerbLaneWidth / 2);
        lane(d * kArmEnd + kerb, d * kJunctionHalf + kerb, kKerbLaneWidth, kerbLane, car, 4, 1);
    }
    // Sections: one lane each, except section 4 (lane 4 offside, lane 8 kerb, a broken line between).
    for (uint32_t s = 0; s < 2 * kArms; ++s) {
        Section sec;
        sec.roadClass = 0;
        sec.lanes.first = static_cast<uint32_t>(g.sectionLanes.size());
        sec.lines.first = static_cast<uint32_t>(g.laneLines.size());
        g.sectionLanes.push_back(PathId(s));
        if (s == 4) {
            g.sectionLanes.push_back(PathId(kKerbLane));
            LaneLine broken;
            broken.crossFromOffside = true;
            broken.crossFromKerbside = true;
            g.laneLines.push_back(broken);
        }
        sec.lanes.count = static_cast<uint32_t>(g.sectionLanes.size()) - sec.lanes.first;
        sec.lines.count = static_cast<uint32_t>(g.laneLines.size()) - sec.lines.first;
        g.sections.push_back(sec);
    }
    // Connectors 9..20, in id order: from each arm, into arm + 1, + 2, + 3.
    for (int a = 0; a < kArms; ++a) {
        for (int k = 1; k < kArms; ++k) {
            const int b = (a + k) % kArms;
            const Vec2 from = armDir(a) * kJunctionHalf + arriveSide(a) * (kLaneWidth / 2);
            const Vec2 to = armDir(b) * kJunctionHalf + leaveSide(b) * (kLaneWidth / 2);
            Path p;
            p.kind = PathKind::Connector;
            p.turn = turnOf(a, b);
            p.laneTags = p.turn == Turn::Straight ? through : turn;
            p.vehicleTags = everyone;
            p.curve = g.addCurve({from, to}, {0, 0});
            p.length = g.along[p.curve.end() - 1];
            p.width = kLaneWidth;
            p.tightestRadius = p.turn == Turn::Straight ? kNoBend : p.turn == Turn::NearTurn ? kNearRadius : kFarRadius;
            p.speedLimit = kTurnSpeed;
            p.junction = JunctionId(0);
            p.from = PathId(static_cast<uint32_t>(2 * a));
            if (p.turn == Turn::FarTurn) p.waitInsideAt = p.length / 2;
            g.paths.push_back(p);
        }
    }
    // What follows each path: an arriving lane, its three connectors; a connector, its leaving lane.
    for (uint32_t i = 0; i < g.paths.size(); ++i) {
        Path& p = g.paths[i];
        p.next.first = static_cast<uint32_t>(g.nextPaths.size());
        if (p.kind == PathKind::Lane && i < kKerbLane && i % 2 == 0) {
            const int a = static_cast<int>(i / 2);
            for (int k = 1; k < kArms; ++k) g.nextPaths.push_back(PathId(connectorId(a, (a + k) % kArms)));
        } else if (p.kind == PathKind::Connector) {
            const int b = static_cast<int>(i - kFirstConnector) / (kArms - 1);
            const int k = static_cast<int>(i - kFirstConnector) % (kArms - 1) + 1;
            g.nextPaths.push_back(PathId(static_cast<uint32_t>(2 * ((b + k) % kArms) + 1)));
        }
        p.next.count = static_cast<uint32_t>(g.nextPaths.size()) - p.next.first;
    }
    // Two regulations on lane 0: a speed limit from its start, a stop line near its end.
    Regulation limit;
    limit.kind = LaneGraph::nameIndex(g.regulationKindNames, "speed_limit");
    limit.path = PathId(0);
    limit.at = 0;
    limit.value = kLaneSpeed;
    Regulation stopLine;
    stopLine.kind = LaneGraph::nameIndex(g.regulationKindNames, "stop");
    stopLine.path = PathId(0);
    stopLine.at = 39;
    g.regulations = {limit, stopLine};
    g.paths[0].regulations = Range{0, 2};

    // The junction, its crosswalk over the east arm, its crossing points.
    Junction j;
    j.control = 0;
    j.box = g.addPlanPoints({Vec2(-kBoxHalf, -kBoxHalf), Vec2(kBoxHalf, -kBoxHalf), Vec2(kBoxHalf, kBoxHalf),
                             Vec2(-kBoxHalf, kBoxHalf)});
    j.connectors.first = static_cast<uint32_t>(g.junctionConnectors.size());
    for (uint32_t c = kFirstConnector; c < g.paths.size(); ++c) g.junctionConnectors.push_back(PathId(c));
    j.connectors.count = static_cast<uint32_t>(g.paths.size()) - kFirstConnector;
    // The lanes arriving at it (each arm's lane in, and the kerb lane beside the west arm's) and leaving it.
    j.arriving.first = static_cast<uint32_t>(g.junctionLanes.size());
    for (uint32_t p : {0u, 2u, 4u, 6u, kKerbLane}) g.junctionLanes.push_back(PathId(p));
    j.arriving.count = static_cast<uint32_t>(g.junctionLanes.size()) - j.arriving.first;
    j.leaving.first = static_cast<uint32_t>(g.junctionLanes.size());
    for (uint32_t p : {1u, 3u, 5u, 7u}) g.junctionLanes.push_back(PathId(p));
    j.leaving.count = static_cast<uint32_t>(g.junctionLanes.size()) - j.leaving.first;
    Crosswalk walk;
    walk.outline = g.addPlanPoints({Vec2(6, -3), Vec2(9, -3), Vec2(9, 3), Vec2(6, 3)});
    walk.junction = JunctionId(0);
    walk.walkLink = 17;
    g.crosswalks.push_back(walk);
    j.crosswalks = Range{0, 1};

    auto meet = [&](uint32_t a, OtherSide side, uint32_t other, Meeting m, Real aFrom, Real aTo, Real oFrom, Real oTo) {
        CrossingPoint c;
        c.a = PathId(a);
        c.otherSide = side;
        c.other = other;
        c.meeting = m;
        c.junction = JunctionId(0);
        c.aFrom = aFrom;
        c.aTo = aTo;
        c.otherFrom = oFrom;
        c.otherTo = oTo;
        g.crossings.push_back(c);
    };
    auto len = [&](uint32_t p) { return g.paths[p].length; };
    const uint32_t eastToWest = connectorId(0, 2), eastToNorth = connectorId(0, 1), eastToSouth = connectorId(0, 3);
    const uint32_t northToSouth = connectorId(1, 3), northToEast = connectorId(1, 0);
    const uint32_t westToEast = connectorId(2, 0), westToNorth = connectorId(2, 1);
    const uint32_t southToWest = connectorId(3, 2);
    // (sorted by first path, then other side, then other)
    meet(0, OtherSide::Crosswalk, 0, Meeting::Cross, 36, 39.5, 0, 3);
    meet(1, OtherSide::Crosswalk, 0, Meeting::Cross, 0.5, 4, 3, 6);
    meet(eastToNorth, OtherSide::Path, eastToWest, Meeting::Split, 0, 2, 0, 2);
    meet(eastToNorth, OtherSide::Path, eastToSouth, Meeting::Split, 0, 1.5, 0, 1.5);
    meet(eastToWest, OtherSide::Path, northToSouth, Meeting::Cross, 3, 6.5, 3.5, 7);
    meet(eastToWest, OtherSide::Path, southToWest, Meeting::Join, len(eastToWest) - 2, len(eastToWest),
         len(southToWest) - 2, len(southToWest));
    meet(northToEast, OtherSide::Path, westToEast, Meeting::Join, len(northToEast) - 2, len(northToEast),
         len(westToEast) - 2, len(westToEast));
    meet(westToEast, OtherSide::Path, westToNorth, Meeting::Split, 0, 2, 0, 2);
    j.crossings = Range{0, static_cast<uint32_t>(g.crossings.size())};
    g.junctions.push_back(j);

    // Three spaces along the kerb lane.
    for (int s = 0; s < 3; ++s) {
        CarSpace sp;
        sp.lane = PathId(kKerbLane);
        sp.from = 2 + 6 * s;
        sp.to = sp.from + 6;
        g.carSpaces.push_back(sp);
    }
    return g;
}

std::vector<uint8_t> save(const LaneGraph& g) {
    bundle::BinWriter w;
    putLaneGraph(w, g);
    return w.bytes;
}
bool load(const std::vector<uint8_t>& bytes, LaneGraph& g, std::string& err) {
    bundle::BinReader r(bytes.data(), bytes.size());
    return getLaneGraph(r, g, &err);
}
bool hasProblem(const std::vector<std::string>& problems, const std::string& words) {
    for (const std::string& p : problems)
        if (p.find(words) != std::string::npos) return true;
    std::printf("    expected a problem containing \"%s\"; got %zu:\n", words.c_str(), problems.size());
    for (const std::string& p : problems) std::printf("      %s\n", p.c_str());
    return false;
}

}  // namespace

TEST_CASE(lane_graph_crossroads_is_sound) {
    const LaneGraph g = makeCrossroads();
    CHECK(g.paths.size() == 21);
    CHECK(g.junctions[0].connectors.count == 12);
    const std::vector<std::string> problems = validate(g);
    for (const std::string& p : problems) std::printf("    problem: %s\n", p.c_str());
    CHECK(problems.empty());
    std::printf("    one path %zu bytes, one section %zu, one crossing point %zu\n", sizeof(Path), sizeof(Section),
                sizeof(CrossingPoint));
}

TEST_CASE(lane_graph_saves_and_loads_field_for_field) {
    const LaneGraph g = makeCrossroads();
    const std::vector<uint8_t> bytes = save(g);
    CHECK(bytes == save(g));   // the same bytes every time
    LaneGraph back;
    std::string err;
    CHECK(load(bytes, back, err));
    CHECK(err.empty());
    CHECK(back == g);
    CHECK(dumpText(back) == dumpText(g));

    // And through a real bundle section.
    bundle::BundleWriter w;
    w.openMemory();
    CHECK(writeLaneGraph(w, "city/e0/roads/lane_graph", g));
    nlohmann::json manifest = nlohmann::json::object();
    CHECK(w.finish(manifest, &err));
    std::unique_ptr<bundle::Bundle> b = bundle::Bundle::fromMemory(w.memory(), &err);
    CHECK(b != nullptr);
    if (b) {
        LaneGraph viaBundle;
        CHECK(readLaneGraph(*b, "city/e0/roads/lane_graph", viaBundle, &err));
        CHECK(viaBundle == g);
        CHECK(!readLaneGraph(*b, "city/e0/roads/missing", viaBundle, &err));
        CHECK(err.find("no section") != std::string::npos);
    }
}

TEST_CASE(lane_graph_text_dump_reads_by_hand) {
    const LaneGraph g = makeCrossroads();
    CHECK(dumpSection(g, SectionId(4)) ==
          "section 4: street, lanes offside to kerb: lane 4, lane 8\n"
          "  line between lane 4 and lane 8: broken, may be crossed either way\n");
    CHECK(dumpPath(g, PathId(connectorId(0, 1))) ==
          "connector 9: far turn, from lane 0 into lane 3, 9.2 m, tightest radius 9.0 m, 3.0 m wide, junction 0, "
          "may wait inside at 4.6 m, tags turn, open to car, bus, bike, speed limit 6.7 m/s\n"
          "  curve: (5.0, -1.5, 0.0) (-1.5, 5.0, 0.0)\n");
    CHECK(dumpCrossing(g, CrossingId(4)) ==
          "crossing point 4, junction 0: connector 10 crosses connector 13; on connector 10 from 3.0 to 6.5 m, "
          "on connector 13 from 3.5 to 7.0 m\n");
    CHECK(dumpCrossing(g, CrossingId(0)) ==
          "crossing point 0, junction 0: lane 0 crosses crosswalk 0; on lane 0 from 36.0 to 39.5 m, "
          "on crosswalk 0 from 0.0 to 3.0 m\n");
    const std::string all = dumpText(g);
    CHECK(all.find("lane graph: 21 paths (9 lanes, 12 connectors), 8 sections, 1 junctions, 8 crossing points, "
                   "1 crosswalks, 0 tracks, 2 regulations, 3 car spaces\n") == 0);
    CHECK(all.find("  regulation 1: stop at 39.0 m, value 0.0\n") != std::string::npos);
    CHECK(all.find("  arriving lanes: 0 2 4 6 8\n  leaving lanes: 1 3 5 7\n") != std::string::npos);
    CHECK(all.find("car space 2: lane 8 from 14.0 to 20.0 m\n") != std::string::npos);
    CHECK(all == dumpText(makeCrossroads()));   // the same text every run
}

TEST_CASE(lane_graph_validate_names_each_fault) {
    const LaneGraph good = makeCrossroads();
    {   // an index out of range
        LaneGraph g = good;
        g.nextPaths[g.paths[2].next.first] = PathId(999);
        CHECK(hasProblem(validate(g), "lane 2: next path 999 is out of range (21 paths)"));
    }
    {   // a lane and its section disagree about where it lies
        LaneGraph g = good;
        g.paths[kKerbLane].indexInSection = 0;
        const std::vector<std::string> p = validate(g);
        CHECK(hasProblem(p, "lane 8 says it is lane 0 of section 4, but the section does not have it there"));
        CHECK(hasProblem(p, "section 4 lists lane 8 as its lane 1, but that lane says it is lane 0 of section 4"));
    }
    {   // a connector whose from-lane doesn't list it as next
        LaneGraph g = good;
        g.paths[connectorId(1, 2)].from = PathId(4);
        CHECK(hasProblem(validate(g), "connector 12 leaves lane 4, but that lane does not list it as next"));
    }
    {   // regulations out of order along their path
        LaneGraph g = good;
        g.regulations[0].at = 39.5;
        CHECK(hasProblem(validate(g), "lane 0: its regulations are out of order (regulation 1 at 39.00 m after 39.50 m)"));
    }
    {   // a crossing stretch running backwards
        LaneGraph g = good;
        std::swap(g.crossings[4].aFrom, g.crossings[4].aTo);
        CHECK(hasProblem(validate(g), "crossing point 4: its stretch on connector 10 runs backwards (from 6.50 to 3.00 m)"));
    }
    {   // a junction that doesn't list one of its connectors
        LaneGraph g = good;
        g.junctions[0].connectors.count -= 1;
        CHECK(hasProblem(validate(g), "connector 20 says it runs through junction 0, but the junction does not list it"));
    }
    {   // a tag past the end of its name table
        LaneGraph g = good;
        g.paths[3].vehicleTags |= TagMask(1) << 5;
        CHECK(hasProblem(validate(g), "lane 3: vehicle tags 0x27 name tags past the 3 vehicle tag names"));
    }
    {   // a crossing pair stored the wrong way round
        LaneGraph g = good;
        std::swap(g.crossings[4].a.index, g.crossings[4].other);
        CHECK(hasProblem(validate(g), "crossing point 4: its paths must be stored lower id first (13, 10)"));
    }
    {   // a junction listing as arriving a lane that ends far from it (the east arm's leaving lane), and a connector
        LaneGraph g = good;
        g.junctionLanes[g.junctions[0].arriving.first] = PathId(1);
        g.junctionLanes[g.junctions[0].leaving.first] = PathId(kFirstConnector);
        const std::vector<std::string> p = validate(g);
        CHECK(hasProblem(p, "junction 0: arriving lane 1 ends at (45.0, 1.5), 39.00 m from its outline (at most 1.50)"));
        CHECK(hasProblem(p, "junction 0 lists connector 9 as leaving, not a lane"));
    }
    {   // its lanes out of id order
        LaneGraph g = good;
        std::swap(g.junctionLanes[g.junctions[0].leaving.first], g.junctionLanes[g.junctions[0].leaving.first + 1]);
        CHECK(hasProblem(validate(g), "junction 0: its leaving lanes are not in id order (1 after 3)"));
    }
}

TEST_CASE(lane_graph_refuses_a_bad_file) {
    const LaneGraph g = makeCrossroads();
    const std::vector<uint8_t> bytes = save(g);
    LaneGraph out;
    std::string err;

    // Cut short anywhere: refused with a message, never a crash.
    for (std::size_t n = 0; n < bytes.size(); ++n) {
        const std::vector<uint8_t> cut(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(n));
        err.clear();
        if (load(cut, out, err)) {
            CHECK(!"a truncated lane graph loaded");
            break;
        }
        if (err.empty()) {
            CHECK(!"a truncated lane graph was refused without a message");
            break;
        }
        CHECK(out.paths.empty());
    }
    {
        const std::vector<uint8_t> cut(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(bytes.size() / 2));
        CHECK(!load(cut, out, err));
        std::printf("    half a file: %s\n", err.c_str());
    }
    {   // a foreign section
        std::vector<uint8_t> bad = bytes;
        bad[0] = 'X';
        CHECK(!load(bad, out, err));
        CHECK(err.find("not a lane graph") != std::string::npos);
    }
    {   // a well-formed file of a broken graph
        LaneGraph broken = g;
        broken.sectionLanes[0] = PathId(999);
        CHECK(!load(save(broken), out, err));
        CHECK(err.find("section 0: its lane 0 is path 999, out of range") != std::string::npos);
        std::printf("    a broken graph: %s\n", err.c_str());
    }
    // Every byte flipped in turn: each copy either loads as a sound graph or is refused with a message.
    int refused = 0;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        std::vector<uint8_t> bad = bytes;
        bad[i] = static_cast<uint8_t>(~bad[i]);
        err.clear();
        if (load(bad, out, err)) {
            CHECK(validate(out).empty());
        } else {
            ++refused;
            CHECK(!err.empty());
        }
    }
    std::printf("    %d of %zu single-byte corruptions refused (the rest changed a number, not the structure)\n", refused,
                bytes.size());
    CHECK(refused > 0);
}

namespace {
// A bare graph for reachableParts (only what it reads): `sectionSizes` sections of lanes in path order, the lines in
// each crossable as `line` says, and the next lists.
LaneGraph reachGraph(const std::vector<uint32_t>& sectionSizes, const LaneLine& line, const std::vector<std::vector<uint32_t>>& next) {
    LaneGraph g;
    for (uint32_t s = 0; s < sectionSizes.size(); ++s) {
        Section sec;
        sec.lanes = Range{static_cast<uint32_t>(g.sectionLanes.size()), sectionSizes[s]};
        sec.lines = Range{static_cast<uint32_t>(g.laneLines.size()), sectionSizes[s] - 1};
        for (uint32_t k = 0; k < sectionSizes[s]; ++k) {
            Path p;
            p.section = SectionId(s);
            p.indexInSection = k;
            g.sectionLanes.push_back(PathId(static_cast<uint32_t>(g.paths.size())));
            g.paths.push_back(p);
            if (k + 1 < sectionSizes[s]) g.laneLines.push_back(line);
        }
        g.sections.push_back(sec);
    }
    for (std::size_t p = 0; p < g.paths.size() && p < next.size(); ++p) {
        g.paths[p].next = Range{static_cast<uint32_t>(g.nextPaths.size()), static_cast<uint32_t>(next[p].size())};
        for (uint32_t q : next[p]) g.nextPaths.push_back(PathId(q));
    }
    return g;
}
}  // namespace

// WHERE A CAR CAN GO ROUND (#238): the parts of the graph whose paths all reach one another, following next lists and
// the lane changes the lines allow.
TEST_CASE(lane_graph_reachable_parts_follow_next_and_lane_changes) {
    // the crossroads: every arm leads in and out once, no way round: each path a part of its own, but the west arm's
    // arriving lane and the lane at its kerb, which a car may move between either way
    const ReachableParts cross = reachableParts(makeCrossroads());
    CHECK(cross.sizes.size() == 20);
    CHECK(cross.partOf[4] == cross.partOf[kKerbLane] && cross.largest == cross.partOf[4] && cross.sizes[cross.largest] == 2);
    // a loop: lane 0 may move over into lane 1 beside it, 1 leads into 2, 2 back into 0; lane 3 leads into the loop and
    // nothing leads into it; lane 4 is led into from the loop and leads nowhere
    LaneLine oneWay;
    oneWay.crossFromOffside = true;
    const LaneGraph loop = reachGraph({2, 1, 1, 1}, oneWay, {{}, {2}, {0, 4}, {0}, {}});
    const ReachableParts parts = reachableParts(loop);
    CHECK(parts.sizes.size() == 3);
    CHECK(parts.partOf[0] == 0 && parts.partOf[1] == 0 && parts.partOf[2] == 0);
    CHECK(parts.sizes[parts.largest] == 3 && parts.largest == 0);
    CHECK(parts.partOf[3] == 1 && parts.partOf[4] == 2);   // numbered by their lowest path
    CHECK(stepsFrom(loop, PathId(0)).size() == 1 && stepsFrom(loop, PathId(1)).size() == 1);   // 1 may not move back into 0
    // the line solid: lane 0 cannot move over into lane 1, so nothing goes round: five parts of one path
    const ReachableParts solid = reachableParts(reachGraph({2, 1, 1, 1}, LaneLine(), {{}, {2}, {0, 4}, {0}, {}}));
    CHECK(solid.sizes.size() == 5);
    // the same answer every run
    CHECK(reachableParts(loop).partOf == parts.partOf);
}
