#include "driving_test_roads.h"

#include "apps/driving/traffic_state.h"   // kerbwardPerpSign

#include <algorithm>
#include <cmath>

namespace driving_test {

namespace {

using engine::LaneGraph;
using engine::Path;
using engine::PathId;
using engine::PathKind;
using engine::Range;
using engine::SectionId;
using engine::Vec2;

constexpr Real kPi = 3.14159265358979323846;
// How many straight pieces a quarter-arc turn is drawn with.
constexpr uint32_t kArcPieces = 8;
// The ordinary car's measurements (testCarClass).
constexpr Real kCarLength = 4.5, kCarWidth = 1.8, kCarWheelbase = 2.7, kCarSteerDegrees = 30;

Heading headingOf(const Vec2& v) {
    Heading best = Heading::East;
    Real bestDot = -2;
    for (uint8_t i = 0; i < kHeadingCount; ++i) {
        const Heading h = static_cast<Heading>(i);
        const Real d = engine::dot(v, headingVector(h));
        if (d > bestDot) {
            bestDot = d;
            best = h;
        }
    }
    return best;
}

// Which way the kerb lies from a road running `h`.
Vec2 kerbward(Heading h, driving::DrivesOn drivesOn) {
    return engine::perp(headingVector(h)) * driving::kerbwardPerpSign(drivesOn);
}

// How far lane k's centre line lies kerbward of the road's middle line.
Real laneOffset(const TestRoadSpec& spec, uint32_t k) { return (Real(k) + Real(0.5)) * spec.laneWidth; }

// A path with the spec's width, limit and tags, its curve and length from `pts`.
Path makePath(LaneGraph& g, const TestRoadSpec& spec, PathKind kind, const std::vector<Vec2>& pts) {
    Path p;
    p.kind = kind;
    p.vehicleTags = engine::TagMask(1);   // the test vehicle tag, name 0
    p.curve = g.addCurve(pts, std::vector<Real>(pts.size(), 0));
    p.length = g.along[p.curve.end() - 1];
    p.width = spec.laneWidth;
    p.speedLimit = spec.speedLimit;
    return p;
}

// Adds a section of `lanes` lanes, lane k running from start + kerb * offset(k) to end + kerb * offset(k).
SectionId addSection(LaneGraph& g, const TestRoadSpec& spec, const Vec2& start, const Vec2& end, const Vec2& kerb) {
    const SectionId id(static_cast<uint32_t>(g.sections.size()));
    engine::Section s;
    s.roadClass = 0;
    s.lanes.first = static_cast<uint32_t>(g.sectionLanes.size());
    s.lanes.count = spec.lanesEachWay;
    s.lines.first = static_cast<uint32_t>(g.laneLines.size());
    s.lines.count = spec.lanesEachWay - 1;
    for (uint32_t k = 0; k < spec.lanesEachWay; ++k) {
        const Vec2 side = kerb * laneOffset(spec, k);
        Path lane = makePath(g, spec, PathKind::Lane, {start + side, end + side});
        lane.section = id;
        lane.indexInSection = k;
        g.sectionLanes.push_back(PathId(static_cast<uint32_t>(g.paths.size())));
        g.paths.push_back(lane);
    }
    for (uint32_t k = 0; k + 1 < spec.lanesEachWay; ++k) g.laneLines.push_back(spec.line);
    g.sections.push_back(s);
    return id;
}

// The names every test graph carries.
void addNames(LaneGraph& g) {
    g.vehicleTagNames = {kTestVehicleTag};
    g.roadClassNames = {kTestRoadClass};
    g.controlNames = {kTestControl};
}

// Writes every path's next list from `nexts` (each already in id order).
void writeNextLists(LaneGraph& g, const std::vector<std::vector<PathId>>& nexts) {
    for (std::size_t i = 0; i < g.paths.size(); ++i) {
        g.paths[i].next.first = static_cast<uint32_t>(g.nextPaths.size());
        g.paths[i].next.count = static_cast<uint32_t>(nexts[i].size());
        g.nextPaths.insert(g.nextPaths.end(), nexts[i].begin(), nexts[i].end());
    }
}

}  // namespace

// ---- headings ------------------------------------------------------------------------------------------------------

engine::Vec2 headingVector(Heading h) {
    switch (h) {
    case Heading::East: return Vec2(1, 0);
    case Heading::North: return Vec2(0, 1);
    case Heading::West: return Vec2(-1, 0);
    case Heading::South: return Vec2(0, -1);
    }
    return Vec2(1, 0);
}

Heading opposite(Heading h) { return static_cast<Heading>((static_cast<uint8_t>(h) + 2) % kHeadingCount); }

Heading nearTurnFrom(Heading h, driving::DrivesOn drivesOn) { return headingOf(kerbward(h, drivesOn)); }
Heading farTurnFrom(Heading h, driving::DrivesOn drivesOn) { return opposite(nearTurnFrom(h, drivesOn)); }

// ---- names ---------------------------------------------------------------------------------------------------------

engine::PathId TestRoads::laneOf(SectionId s, uint32_t k) const {
    if (!s.valid() || s.index >= graph.sections.size()) return PathId();
    const engine::Section& sec = graph.sections[s.index];
    return k < sec.lanes.count ? graph.sectionLanes[sec.lanes.first + k] : PathId();
}

engine::JunctionId TestRoads::junction(uint32_t row, uint32_t col) const {
    if (row >= rows || col >= cols) return engine::JunctionId();
    return engine::JunctionId(row * cols + col);
}

SectionId TestRoads::sectionLeaving(uint32_t row, uint32_t col, Heading h) const {
    if (row >= rows || col >= cols) return SectionId();
    return leavingSections[(row * cols + col) * kHeadingCount + static_cast<uint8_t>(h)];
}

engine::PathId TestRoads::laneLeaving(uint32_t row, uint32_t col, Heading h, uint32_t k) const {
    return laneOf(sectionLeaving(row, col, h), k);
}

SectionId TestRoads::sectionArriving(uint32_t row, uint32_t col, Heading h) const {
    // The road arriving heading h leaves the junction one step the other way.
    const Vec2 back = headingVector(opposite(h));
    const long r = static_cast<long>(row) + static_cast<long>(back.y);
    const long c = static_cast<long>(col) + static_cast<long>(back.x);
    if (r < 0 || c < 0) return SectionId();
    return sectionLeaving(static_cast<uint32_t>(r), static_cast<uint32_t>(c), h);
}

engine::PathId TestRoads::laneArriving(uint32_t row, uint32_t col, Heading h, uint32_t k) const {
    return laneOf(sectionArriving(row, col, h), k);
}

engine::PathId TestRoads::connector(uint32_t row, uint32_t col, Heading in, uint32_t kIn, Heading out,
                                    uint32_t kOut) const {
    const engine::JunctionId j = junction(row, col);
    const PathId a = laneArriving(row, col, in, kIn);
    const PathId b = laneLeaving(row, col, out, kOut);
    if (!j.valid() || !a.valid() || !b.valid()) return PathId();
    const engine::Junction& jn = graph.junctions[j.index];
    for (uint32_t i = jn.connectors.first; i < jn.connectors.end(); ++i) {
        const PathId c = graph.junctionConnectors[i];
        const Path& p = graph.paths[c.index];
        if (p.from == a && p.next.count == 1 && graph.nextPaths[p.next.first] == b) return c;
    }
    return PathId();
}

SectionId TestRoads::straightSection(uint32_t i, bool eastward) const {
    const std::size_t at = static_cast<std::size_t>(i) * 2 + (eastward ? 0 : 1);
    return at < straightSections.size() ? straightSections[at] : SectionId();
}

engine::PathId TestRoads::straightLane(uint32_t i, bool eastward, uint32_t k) const {
    return laneOf(straightSection(i, eastward), k);
}

engine::PathId TestRoads::ringLane(uint32_t i, uint32_t k) const {
    return i < ringSections.size() ? laneOf(ringSections[i], k) : engine::PathId();
}

// ---- builders ------------------------------------------------------------------------------------------------------

TestRoads straightRoad(uint32_t sections, const TestRoadSpec& spec) {
    return straightRoadOf(std::vector<Real>(sections, spec.roadLength), spec);
}

TestRoads straightRoadOf(const std::vector<Real>& sectionLengths, const TestRoadSpec& spec) {
    TestRoads t;
    t.spec = spec;
    const uint32_t sections = static_cast<uint32_t>(sectionLengths.size());
    t.sectionCount = sections;
    LaneGraph& g = t.graph;
    addNames(g);
    const Vec2 east = headingVector(Heading::East);
    Real x = 0;
    for (uint32_t i = 0; i < sections; ++i) {
        const Vec2 west0 = east * x;
        x += sectionLengths[i];
        const Vec2 east0 = east * x;
        t.straightSections.push_back(addSection(g, spec, west0, east0, kerbward(Heading::East, spec.drivesOn)));
        t.straightSections.push_back(addSection(g, spec, east0, west0, kerbward(Heading::West, spec.drivesOn)));
    }
    // Each lane leads into the same lane of the next section its way.
    std::vector<std::vector<PathId>> nexts(g.paths.size());
    for (uint32_t i = 0; i + 1 < sections; ++i) {
        for (uint32_t k = 0; k < spec.lanesEachWay; ++k) {
            nexts[t.straightLane(i, true, k).index].push_back(t.straightLane(i + 1, true, k));
            nexts[t.straightLane(i + 1, false, k).index].push_back(t.straightLane(i, false, k));
        }
    }
    writeNextLists(g, nexts);
    return t;
}

TestRoads ringRoad(uint32_t sides, const TestRoadSpec& spec) {
    TestRoads t;
    t.spec = spec;
    LaneGraph& g = t.graph;
    addNames(g);
    // The polygon's corners round a circle whose chord between neighbours is roadLength.
    const Real radius = spec.roadLength / (2 * std::sin(kPi / Real(sides)));
    std::vector<Vec2> corners;
    for (uint32_t i = 0; i < sides; ++i) {
        const Real a = 2 * kPi * Real(i) / Real(sides);
        corners.push_back(Vec2(std::cos(a), std::sin(a)) * radius);
    }
    for (uint32_t i = 0; i < sides; ++i) {
        const Vec2 start = corners[i];
        const Vec2 end = corners[(i + 1) % sides];
        const Vec2 kerb = engine::perp(engine::normalize(end - start)) * driving::kerbwardPerpSign(spec.drivesOn);
        t.ringSections.push_back(addSection(g, spec, start, end, kerb));
    }
    std::vector<std::vector<PathId>> nexts(g.paths.size());
    for (uint32_t i = 0; i < sides; ++i)
        for (uint32_t k = 0; k < spec.lanesEachWay; ++k)
            nexts[t.ringLane(i, k).index].push_back(t.ringLane((i + 1) % sides, k));
    writeNextLists(g, nexts);
    return t;
}

TestRoads gridRoads(uint32_t rows, uint32_t cols, const TestRoadSpec& spec) {
    TestRoads t;
    t.spec = spec;
    t.rows = rows;
    t.cols = cols;
    LaneGraph& g = t.graph;
    addNames(g);
    const uint32_t lanes = spec.lanesEachWay;
    // Half the junction box's side: the road's half width (every lane each way) and the kerb corner.
    const Real halfBox = Real(lanes) * spec.laneWidth + spec.cornerRadius;
    const Real spacing = spec.roadLength + 2 * halfBox;
    auto centre = [&](uint32_t r, uint32_t c) { return Vec2(Real(c) * spacing, Real(r) * spacing); };
    // The junction one step from (r, c) heading h, when it is on the grid.
    auto step = [&](uint32_t r, uint32_t c, Heading h, uint32_t& nr, uint32_t& nc) {
        const Vec2 d = headingVector(h);
        const long rr = static_cast<long>(r) + static_cast<long>(d.y);
        const long cc = static_cast<long>(c) + static_cast<long>(d.x);
        if (rr < 0 || cc < 0 || rr >= static_cast<long>(rows) || cc >= static_cast<long>(cols)) return false;
        nr = static_cast<uint32_t>(rr);
        nc = static_cast<uint32_t>(cc);
        return true;
    };

    // The roads: one section each way between neighbouring junctions, from box edge to box edge.
    t.leavingSections.assign(static_cast<std::size_t>(rows) * cols * kHeadingCount, SectionId());
    for (uint32_t r = 0; r < rows; ++r)
        for (uint32_t c = 0; c < cols; ++c)
            for (uint8_t hi = 0; hi < kHeadingCount; ++hi) {
                const Heading h = static_cast<Heading>(hi);
                uint32_t nr = 0, nc = 0;
                if (!step(r, c, h, nr, nc)) continue;
                const Vec2 d = headingVector(h);
                t.leavingSections[(r * cols + c) * kHeadingCount + hi] =
                    addSection(g, spec, centre(r, c) + d * halfBox, centre(nr, nc) - d * halfBox, kerbward(h, spec.drivesOn));
            }

    // The junctions and their connectors, junction by junction (so each junction's connectors are in id order).
    std::vector<std::pair<PathId, PathId>> joins;   // connector -> the lane it leads into, in connector order
    for (uint32_t r = 0; r < rows; ++r) {
        for (uint32_t c = 0; c < cols; ++c) {
            const engine::JunctionId jid(static_cast<uint32_t>(g.junctions.size()));
            engine::Junction jn;
            jn.control = 0;
            jn.connectors.first = static_cast<uint32_t>(g.junctionConnectors.size());
            const Vec2 o = centre(r, c);
            jn.box = g.addPlanPoints({o + Vec2(-halfBox, -halfBox), o + Vec2(halfBox, -halfBox),
                                      o + Vec2(halfBox, halfBox), o + Vec2(-halfBox, halfBox)});
            for (uint8_t ii = 0; ii < kHeadingCount; ++ii) {
                const Heading in = static_cast<Heading>(ii);
                if (!t.sectionArriving(r, c, in).valid()) continue;
                for (uint8_t oi = 0; oi < kHeadingCount; ++oi) {
                    const Heading out = static_cast<Heading>(oi);
                    if (out == opposite(in) || !t.sectionLeaving(r, c, out).valid()) continue;
                    engine::Turn turn = engine::Turn::Straight;
                    uint32_t firstLane = 0, lastLane = lanes;   // which lanes take this movement, lane k into lane k
                    if (out == nearTurnFrom(in, spec.drivesOn)) {
                        turn = engine::Turn::NearTurn;
                        firstLane = lanes - 1;
                    } else if (out != in) {
                        turn = engine::Turn::FarTurn;
                        lastLane = 1;
                    }
                    for (uint32_t k = firstLane; k < lastLane; ++k) {
                        const PathId a = t.laneArriving(r, c, in, k);
                        const PathId b = t.laneLeaving(r, c, out, k);
                        const Vec2 pIn = g.points[g.paths[a.index].curve.end() - 1];
                        const Vec2 pOut = g.points[g.paths[b.index].curve.first];
                        std::vector<Vec2> pts;
                        Real radius = engine::kNoBend;
                        if (turn == engine::Turn::Straight) {
                            pts = {pIn, pOut};
                        } else {
                            // A quarter arc: it leaves pIn heading `in` and reaches pOut heading `out`, round a centre
                            // `radius` from both.
                            const Vec2 dIn = headingVector(in), dOut = headingVector(out);
                            radius = engine::dot(pOut - pIn, dIn);
                            const Vec2 middle = pIn + dOut * radius;
                            pts.push_back(pIn);
                            for (uint32_t i = 1; i < kArcPieces; ++i) {
                                const Real angle = (Real(i) / Real(kArcPieces)) * (kPi / 2);
                                pts.push_back(middle - dOut * (radius * std::cos(angle)) + dIn * (radius * std::sin(angle)));
                            }
                            pts.push_back(pOut);
                        }
                        Path conn = makePath(g, spec, PathKind::Connector, pts);
                        conn.turn = turn;
                        conn.junction = jid;
                        conn.from = a;
                        conn.tightestRadius = radius;
                        const PathId cid(static_cast<uint32_t>(g.paths.size()));
                        g.paths.push_back(conn);
                        g.junctionConnectors.push_back(cid);
                        joins.emplace_back(cid, b);
                    }
                }
            }
            jn.connectors.count = static_cast<uint32_t>(g.junctionConnectors.size()) - jn.connectors.first;
            g.junctions.push_back(jn);
        }
    }

    // Each lane leads into the connectors leaving its end; each connector into its lane.
    std::vector<std::vector<PathId>> nexts(g.paths.size());
    for (const std::pair<PathId, PathId>& j : joins) {
        nexts[g.paths[j.first.index].from.index].push_back(j.first);
        nexts[j.first.index].push_back(j.second);
    }
    writeNextLists(g, nexts);
    return t;
}

driving::VehicleClass testCarClass(const std::vector<std::string>& tags) {
    driving::VehicleClass c =
        driving::deriveVehicleClass("test_car", kCarLength, kCarWidth, kCarWheelbase, kCarSteerDegrees);
    c.tags = tags;
    std::sort(c.tags.begin(), c.tags.end());
    c.tags.erase(std::unique(c.tags.begin(), c.tags.end()), c.tags.end());
    return c;
}

}  // namespace driving_test
