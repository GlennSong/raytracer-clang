#include "engine/procgen/city/roads/lane_graph.h"

#include "engine/bundle/bundle.h"
#include "engine/procgen/city/polyline.h"   // stations: each point's distance along a polyline

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace engine {

namespace {

// The section's magic and the version this build writes and reads (2: junctions list their arriving and leaving lanes;
// 3: each path lists its crossing points; 4: a regulation covers a stretch, from and to, issue #240; 5: each path's bend
// profile, issue #281).
constexpr char kMagic[4] = {'L', 'G', 'R', 'F'};
constexpr uint32_t kVersion = 5;
// A path's centre line has at least this many points (a start and an end).
constexpr uint32_t kMinCurvePoints = 2;
// A closed outline (a junction's box, a crosswalk) has at least this many corners; a track's line two points.
constexpr uint32_t kMinOutlinePoints = 3;
constexpr uint32_t kMinLinePoints = 2;
// How far a path's stored length may differ from the distance along its last point, and how far past a path's end a
// stretch on it may reach, before it counts as a fault (rounding in whoever built it).
constexpr Real kLengthTolerance = 1e-3;
// How many problems a refused load spells out in its message (the rest are counted).
constexpr std::size_t kProblemsInMessage = 8;

// printf into a std::string (the C locale's decimal point: the same text on every machine).
#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 1, 2)))
#endif
std::string fmt(const char* f, ...) {
    va_list args;
    va_start(args, f);
    va_list copy;
    va_copy(copy, args);
    const int n = std::vsnprintf(nullptr, 0, f, copy);
    va_end(copy);
    std::string s(n > 0 ? static_cast<std::size_t>(n) : 0, '\0');
    if (n > 0) std::vsnprintf(&s[0], s.size() + 1, f, args);
    va_end(args);
    return s;
}

bool inRange(const Range& r, std::size_t size) { return r.first <= size && r.count <= size - r.first; }
// The end of a range clamped to its array, so the dump never reads outside one even on a broken graph.
uint32_t safeEnd(const Range& r, std::size_t size) {
    return static_cast<uint32_t>(std::min<std::size_t>(size, static_cast<std::size_t>(r.first) + r.count));
}
TagMask maskOfFirst(std::size_t names) { return names >= kMaxTagNames ? ~TagMask(0) : (TagMask(1) << names) - 1; }

const char* pathKindWord(PathKind k) { return k == PathKind::Connector ? "connector" : "lane"; }
std::string pathName(const LaneGraph& g, uint32_t i) {
    if (i >= g.paths.size()) return fmt("path %u", i);
    return fmt("%s %u", pathKindWord(g.paths[i].kind), i);
}
const char* turnWord(Turn t) {
    switch (t) {
    case Turn::None: return "no turn";
    case Turn::Straight: return "straight";
    case Turn::NearTurn: return "near turn";
    case Turn::FarTurn: return "far turn";
    case Turn::UTurn: return "U-turn";
    }
    return "unknown turn";
}
const char* meetingWord(Meeting m) {
    switch (m) {
    case Meeting::Cross: return "crosses";
    case Meeting::Join: return "joins";
    case Meeting::Split: return "splits from";
    }
    return "meets";
}
std::string otherName(const LaneGraph& g, const CrossingPoint& c) {
    switch (c.otherSide) {
    case OtherSide::Path: return pathName(g, c.other);
    case OtherSide::Crosswalk: return fmt("crosswalk %u", c.other);
    case OtherSide::Track: return fmt("track %u", c.other);
    }
    return fmt("unknown %u", c.other);
}
std::string nameOf(const std::vector<std::string>& table, NameIndex i) {
    if (i == kNoName) return "none";
    return i < table.size() ? table[i] : fmt("unknown name %u", i);
}
std::string namesOf(const std::vector<std::string>& table, TagMask m) {
    std::string s;
    for (uint32_t b = 0; b < kMaxTagNames; ++b) {
        if (!(m & (TagMask(1) << b))) continue;
        if (!s.empty()) s += ", ";
        s += b < table.size() ? table[b] : fmt("unknown tag %u", b);
    }
    return s.empty() ? "nothing" : s;
}
std::string metres(Real v) { return std::isinf(v) ? std::string("no bend") : fmt("%.1f m", v); }
std::string planPointsText(const LaneGraph& g, const Range& r) {
    std::string s;
    for (uint32_t i = r.first; i < safeEnd(r, g.planPoints.size()); ++i)
        s += fmt("%s(%.1f, %.1f)", s.empty() ? "" : " ", g.planPoints[i].x, g.planPoints[i].y);
    return s;
}

// ---- save and load ---------------------------------------------------------------------------------------------

using bundle::BinReader;
using bundle::BinWriter;

void putItem(BinWriter& w, const std::string& s) { w.putStr(s); }
void putItem(BinWriter& w, const Vec2& v) { w.put(v.x); w.put(v.y); }
void putItem(BinWriter& w, Real v) { w.put(v); }
void putItem(BinWriter& w, const Range& r) { w.put(r.first); w.put(r.count); }
template <typename Tag> void putItem(BinWriter& w, const GraphIndex<Tag>& id) { w.put(id.index); }
void putItem(BinWriter& w, const LaneLine& l) {
    w.put(static_cast<uint8_t>(l.crossFromOffside));
    w.put(static_cast<uint8_t>(l.crossFromKerbside));
}
void putItem(BinWriter& w, const Path& p) {
    w.put(static_cast<uint8_t>(p.kind));
    w.put(static_cast<uint8_t>(p.turn));
    w.put(p.laneTags);
    w.put(p.vehicleTags);
    putItem(w, p.curve);
    w.put(p.length);
    w.put(p.width);
    w.put(p.tightestRadius);
    putItem(w, p.bends);
    w.put(p.speedLimit);
    putItem(w, p.section);
    w.put(p.indexInSection);
    putItem(w, p.junction);
    putItem(w, p.from);
    w.put(p.waitInsideAt);
    putItem(w, p.next);
    putItem(w, p.regulations);
    putItem(w, p.crossings);
}
void putItem(BinWriter& w, const Section& s) {
    putItem(w, s.lanes);
    putItem(w, s.lines);
    w.put(s.roadClass);
}
void putItem(BinWriter& w, const Junction& j) {
    putItem(w, j.connectors);
    putItem(w, j.crossings);
    putItem(w, j.crosswalks);
    w.put(j.control);
    putItem(w, j.box);
    putItem(w, j.arriving);
    putItem(w, j.leaving);
}
void putItem(BinWriter& w, const CrossingPoint& c) {
    putItem(w, c.a);
    w.put(static_cast<uint8_t>(c.otherSide));
    w.put(static_cast<uint8_t>(c.meeting));
    w.put(c.other);
    putItem(w, c.junction);
    w.put(c.aFrom);
    w.put(c.aTo);
    w.put(c.otherFrom);
    w.put(c.otherTo);
}
void putItem(BinWriter& w, const Crosswalk& c) {
    putItem(w, c.outline);
    putItem(w, c.junction);
    w.put(c.walkLink);
}
void putItem(BinWriter& w, const Track& t) {
    putItem(w, t.line);
    w.put(t.railTrack);
}
void putItem(BinWriter& w, const Regulation& r) {
    w.put(r.kind);
    putItem(w, r.path);
    w.put(r.from);
    w.put(r.to);
    w.put(r.value);
}
void putItem(BinWriter& w, const CarSpace& s) {
    putItem(w, s.lane);
    w.put(s.from);
    w.put(s.to);
}
void putItem(BinWriter& w, const BendPiece& b) {
    w.put(b.from);
    w.put(b.radius);
}

bool getItem(BinReader& r, std::string& s) { return r.getStr(s); }
bool getItem(BinReader& r, Vec2& v) { return r.get(v.x) && r.get(v.y); }
bool getItem(BinReader& r, Real& v) { return r.get(v); }
bool getItem(BinReader& r, Range& x) { return r.get(x.first) && r.get(x.count); }
template <typename Tag> bool getItem(BinReader& r, GraphIndex<Tag>& id) { return r.get(id.index); }
template <typename E> bool getEnum(BinReader& r, E& e) {
    uint8_t v = 0;
    if (!r.get(v)) return false;
    e = static_cast<E>(v);   // (an unknown value is kept, and validate() names it)
    return true;
}
bool getItem(BinReader& r, LaneLine& l) {
    uint8_t a = 0, b = 0;
    if (!r.get(a) || !r.get(b)) return false;
    l.crossFromOffside = a != 0;
    l.crossFromKerbside = b != 0;
    return true;
}
bool getItem(BinReader& r, Path& p) {
    return getEnum(r, p.kind) && getEnum(r, p.turn) && r.get(p.laneTags) && r.get(p.vehicleTags) &&
           getItem(r, p.curve) && r.get(p.length) && r.get(p.width) && r.get(p.tightestRadius) &&
           getItem(r, p.bends) && r.get(p.speedLimit) && getItem(r, p.section) && r.get(p.indexInSection) && getItem(r, p.junction) &&
           getItem(r, p.from) && r.get(p.waitInsideAt) && getItem(r, p.next) && getItem(r, p.regulations) &&
           getItem(r, p.crossings);
}
bool getItem(BinReader& r, Section& s) { return getItem(r, s.lanes) && getItem(r, s.lines) && r.get(s.roadClass); }
bool getItem(BinReader& r, Junction& j) {
    return getItem(r, j.connectors) && getItem(r, j.crossings) && getItem(r, j.crosswalks) && r.get(j.control) &&
           getItem(r, j.box) && getItem(r, j.arriving) && getItem(r, j.leaving);
}
bool getItem(BinReader& r, CrossingPoint& c) {
    return getItem(r, c.a) && getEnum(r, c.otherSide) && getEnum(r, c.meeting) && r.get(c.other) &&
           getItem(r, c.junction) && r.get(c.aFrom) && r.get(c.aTo) && r.get(c.otherFrom) && r.get(c.otherTo);
}
bool getItem(BinReader& r, Crosswalk& c) { return getItem(r, c.outline) && getItem(r, c.junction) && r.get(c.walkLink); }
bool getItem(BinReader& r, Track& t) { return getItem(r, t.line) && r.get(t.railTrack); }
bool getItem(BinReader& r, Regulation& x) {
    return r.get(x.kind) && getItem(r, x.path) && r.get(x.from) && r.get(x.to) && r.get(x.value);
}
bool getItem(BinReader& r, CarSpace& s) { return getItem(r, s.lane) && r.get(s.from) && r.get(s.to); }
bool getItem(BinReader& r, BendPiece& b) { return r.get(b.from) && r.get(b.radius); }

// The fewest bytes one element of a list takes on disk (a default one, written): how a count read from a damaged file
// is caught before it asks for more memory than the file could fill.
template <typename T> std::size_t leastBytes() {
    BinWriter w;
    putItem(w, T());
    return w.bytes.size();
}

template <typename T> void putList(BinWriter& w, const std::vector<T>& v) {
    w.put<uint64_t>(static_cast<uint64_t>(v.size()));
    for (const T& x : v) putItem(w, x);
}
template <typename T> bool getList(BinReader& r, std::vector<T>& v, const char* what, std::string* err) {
    uint64_t n = 0;
    if (!r.get(n)) {
        if (err) *err = fmt("lane graph: the section ends before the count of %s", what);
        return false;
    }
    const std::size_t least = leastBytes<T>();
    if (n > r.remaining() / least) {
        if (err)
            *err = fmt("lane graph: the section says %llu %s, more than its last %zu bytes can hold",
                       static_cast<unsigned long long>(n), what, r.remaining());
        return false;
    }
    v.resize(static_cast<std::size_t>(n));
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (!getItem(r, v[i])) {
            if (err) *err = fmt("lane graph: the section ends inside %s, at entry %zu of %zu", what, i, v.size());
            return false;
        }
    }
    return true;
}

}  // namespace

// ---- building ---------------------------------------------------------------------------------------------------

NameIndex LaneGraph::nameIndex(const std::vector<std::string>& table, const std::string& name) {
    for (std::size_t i = 0; i < table.size(); ++i)
        if (table[i] == name) return static_cast<NameIndex>(i);
    return kNoName;
}

Range LaneGraph::addCurve(const std::vector<Vec2>& xy, const std::vector<Real>& z) {
    Range r;
    r.first = static_cast<uint32_t>(points.size());
    r.count = static_cast<uint32_t>(xy.size());
    const std::vector<Real> s = stations(xy);
    points.insert(points.end(), xy.begin(), xy.end());
    along.insert(along.end(), s.begin(), s.end());
    for (std::size_t i = 0; i < xy.size(); ++i) heights.push_back(i < z.size() ? z[i] : Real(0));
    return r;
}

namespace {
// The curve range of a path, or an empty one for an unknown path or a range off the shared arrays.
Range curveOf(const LaneGraph& g, PathId path) {
    if (!path.valid() || path.index >= g.paths.size()) return Range{};
    const Range r = g.paths[path.index].curve;
    if (r.end() > g.points.size() || r.end() > g.along.size() || r.end() > g.heights.size()) return Range{};
    return r;
}
}  // namespace

Vec2 LaneGraph::pointAt(PathId path, Real distanceAlong) const {
    const Range r = curveOf(*this, path);
    if (r.count == 0) return Vec2();
    return pointAtStation(points.data() + r.first, along.data() + r.first, r.count, distanceAlong);
}

Real LaneGraph::heightAt(PathId path, Real distanceAlong) const {
    const Range r = curveOf(*this, path);
    if (r.count == 0) return 0;
    return valueAtStation(along.data() + r.first, heights.data() + r.first, r.count, distanceAlong);
}

Vec2 LaneGraph::directionAt(PathId path, Real distanceAlong) const {
    const Vec2 kNoHeading(1, 0);
    const Range r = curveOf(*this, path);
    if (r.count == 0) return kNoHeading;
    const Vec2 d = chordAtStation(points.data() + r.first, along.data() + r.first, r.count, distanceAlong);
    if (d.length() > kShortestStationSpan) return normalize(d);
    const Vec2 whole = points[r.end() - 1] - points[r.first];
    return whole.length() > kShortestStationSpan ? normalize(whole) : kNoHeading;
}

Real LaneGraph::gradeAt(PathId path, Real distanceAlong) const {
    const Range r = curveOf(*this, path);
    if (r.count == 0) return 0;
    const Real* s = along.data() + r.first;
    const Real before = std::max(distanceAlong - kTangentHalfSpan, s[0]);
    const Real after = std::min(distanceAlong + kTangentHalfSpan, s[r.count - 1]);
    const Real run = chordAtStation(points.data() + r.first, s, r.count, distanceAlong).length();
    if (run <= kShortestStationSpan) return 0;
    const Real* h = heights.data() + r.first;
    return (valueAtStation(s, h, r.count, after) - valueAtStation(s, h, r.count, before)) / run;
}

Range LaneGraph::addPlanPoints(const std::vector<Vec2>& pts) {
    Range r;
    r.first = static_cast<uint32_t>(planPoints.size());
    r.count = static_cast<uint32_t>(pts.size());
    planPoints.insert(planPoints.end(), pts.begin(), pts.end());
    return r;
}

Range LaneGraph::addBends(const std::vector<BendPiece>& pieces) {
    Range r;
    r.first = static_cast<uint32_t>(bendPieces.size());
    r.count = static_cast<uint32_t>(pieces.size());
    bendPieces.insert(bendPieces.end(), pieces.begin(), pieces.end());
    return r;
}

namespace {
// A path's own bend pieces, or an empty range for an unknown path or one whose range runs off the shared array (then
// it reads as the whole path at its tightest radius).
Range bendsOf(const LaneGraph& g, PathId path) {
    if (!path.valid() || path.index >= g.paths.size()) return Range{};
    const Range r = g.paths[path.index].bends;
    return inRange(r, g.bendPieces.size()) ? r : Range{};
}
}  // namespace

uint32_t LaneGraph::bendCount(PathId path) const {
    if (!path.valid() || path.index >= paths.size()) return 0;
    const Range r = bendsOf(*this, path);
    return r.count > 0 ? r.count : 1;
}

BendSpan LaneGraph::bendSpan(PathId path, uint32_t i) const {
    BendSpan s;
    if (!path.valid() || path.index >= paths.size()) return s;
    const Path& p = paths[path.index];
    const Range r = bendsOf(*this, path);
    if (r.count == 0) {
        if (i == 0) {
            s.to = p.length;
            s.radius = p.tightestRadius;
        } else {
            s.from = s.to = p.length;
        }
        return s;
    }
    if (i >= r.count) {
        s.from = s.to = p.length;
        return s;
    }
    const BendPiece& b = bendPieces[r.first + i];
    s.from = b.from;
    s.to = i + 1 < r.count ? bendPieces[r.first + i + 1].from : p.length;
    s.radius = b.radius;
    return s;
}

Real LaneGraph::radiusAt(PathId path, Real distanceAlong) const {
    if (!path.valid() || path.index >= paths.size()) return kNoBend;
    const Range r = bendsOf(*this, path);
    if (r.count == 0) return paths[path.index].tightestRadius;
    // the last piece that begins at or before the place (the first when the place is before every piece)
    const BendPiece* first = bendPieces.data() + r.first;
    const BendPiece* last = first + r.count;
    const BendPiece* after = std::upper_bound(first, last, distanceAlong,
                                              [](Real d, const BendPiece& b) { return d < b.from; });
    return after == first ? first->radius : (after - 1)->radius;
}

BendSpan LaneGraph::tightestBend(PathId path) const {
    BendSpan best;
    const uint32_t n = bendCount(path);
    for (uint32_t i = 0; i < n; ++i) {
        const BendSpan s = bendSpan(path, i);
        if (i == 0 || s.radius < best.radius) best = s;
    }
    return best;
}

void indexCrossings(LaneGraph& g) {
    // junction by junction, those on no junction (kNoIndex) last; then (a, other side, other, aFrom), and the rest of
    // the record so equal keys still sort one way
    std::stable_sort(g.crossings.begin(), g.crossings.end(), [](const CrossingPoint& x, const CrossingPoint& y) {
        if (x.junction.index != y.junction.index) return x.junction.index < y.junction.index;
        if (x.a != y.a) return x.a < y.a;
        if (x.otherSide != y.otherSide) return static_cast<uint8_t>(x.otherSide) < static_cast<uint8_t>(y.otherSide);
        if (x.other != y.other) return x.other < y.other;
        if (x.aFrom != y.aFrom) return x.aFrom < y.aFrom;
        return x.otherFrom < y.otherFrom;
    });
    std::size_t k = 0;
    for (uint32_t j = 0; j < g.junctions.size(); ++j) {
        while (k < g.crossings.size() && g.crossings[k].junction.index < j) ++k;
        Range& r = g.junctions[j].crossings;
        r.first = static_cast<uint32_t>(k);
        while (k < g.crossings.size() && g.crossings[k].junction.index == j) ++k;
        r.count = static_cast<uint32_t>(k) - r.first;
    }
    // each path's list: (path, where the stretch on it begins, record), sorted
    struct Entry { uint32_t path; Real start; uint32_t crossing; };
    std::vector<Entry> entries;
    for (uint32_t c = 0; c < g.crossings.size(); ++c) {
        const CrossingPoint& x = g.crossings[c];
        entries.push_back(Entry{x.a.index, x.aFrom, c});
        if (x.otherSide == OtherSide::Path) entries.push_back(Entry{x.other, x.otherFrom, c});
    }
    std::sort(entries.begin(), entries.end(), [](const Entry& x, const Entry& y) {
        if (x.path != y.path) return x.path < y.path;
        if (x.start != y.start) return x.start < y.start;
        return x.crossing < y.crossing;
    });
    g.pathCrossings.clear();
    std::size_t e = 0;
    for (uint32_t p = 0; p < g.paths.size(); ++p) {
        Range& r = g.paths[p].crossings;
        r.first = static_cast<uint32_t>(g.pathCrossings.size());
        while (e < entries.size() && entries[e].path < p) ++e;   // (a path out of range: skipped, validate names it)
        while (e < entries.size() && entries[e].path == p) g.pathCrossings.push_back(CrossingId(entries[e++].crossing));
        r.count = static_cast<uint32_t>(g.pathCrossings.size()) - r.first;
    }
}

// ---- equality ---------------------------------------------------------------------------------------------------

namespace {
bool sameVec2(const Vec2& a, const Vec2& b) { return a.x == b.x && a.y == b.y; }
bool sameVec2s(const std::vector<Vec2>& a, const std::vector<Vec2>& b) {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), sameVec2);
}
}  // namespace

bool operator==(const Path& a, const Path& b) {
    return a.kind == b.kind && a.turn == b.turn && a.laneTags == b.laneTags && a.vehicleTags == b.vehicleTags &&
           a.curve == b.curve && a.length == b.length && a.width == b.width && a.tightestRadius == b.tightestRadius &&
           a.bends == b.bends && a.speedLimit == b.speedLimit && a.section == b.section && a.indexInSection == b.indexInSection &&
           a.junction == b.junction && a.from == b.from && a.waitInsideAt == b.waitInsideAt && a.next == b.next &&
           a.regulations == b.regulations && a.crossings == b.crossings;
}
bool operator==(const Section& a, const Section& b) {
    return a.lanes == b.lanes && a.lines == b.lines && a.roadClass == b.roadClass;
}
bool operator==(const CrossingPoint& a, const CrossingPoint& b) {
    return a.a == b.a && a.otherSide == b.otherSide && a.meeting == b.meeting && a.other == b.other &&
           a.junction == b.junction && a.aFrom == b.aFrom && a.aTo == b.aTo && a.otherFrom == b.otherFrom &&
           a.otherTo == b.otherTo;
}
bool operator==(const Crosswalk& a, const Crosswalk& b) {
    return a.outline == b.outline && a.junction == b.junction && a.walkLink == b.walkLink;
}
bool operator==(const Track& a, const Track& b) { return a.line == b.line && a.railTrack == b.railTrack; }
bool operator==(const Junction& a, const Junction& b) {
    return a.connectors == b.connectors && a.crossings == b.crossings && a.crosswalks == b.crosswalks &&
           a.control == b.control && a.box == b.box && a.arriving == b.arriving && a.leaving == b.leaving;
}
bool operator==(const Regulation& a, const Regulation& b) {
    return a.kind == b.kind && a.path == b.path && a.from == b.from && a.to == b.to && a.value == b.value;
}
bool operator==(const CarSpace& a, const CarSpace& b) { return a.lane == b.lane && a.from == b.from && a.to == b.to; }
namespace {
bool sameBends(const std::vector<BendPiece>& a, const std::vector<BendPiece>& b) {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](const BendPiece& x, const BendPiece& y) {
               return x.from == y.from && x.radius == y.radius;
           });
}
}  // namespace
bool operator==(const LaneGraph& a, const LaneGraph& b) {
    return a.laneTagNames == b.laneTagNames && a.vehicleTagNames == b.vehicleTagNames &&
           a.regulationKindNames == b.regulationKindNames && a.controlNames == b.controlNames &&
           a.roadClassNames == b.roadClassNames && a.paths == b.paths && a.sections == b.sections &&
           a.junctions == b.junctions && a.crossings == b.crossings && a.crosswalks == b.crosswalks &&
           a.tracks == b.tracks && a.regulations == b.regulations && a.carSpaces == b.carSpaces &&
           sameVec2s(a.points, b.points) && a.along == b.along && a.heights == b.heights &&
           a.nextPaths == b.nextPaths && a.sectionLanes == b.sectionLanes && a.laneLines == b.laneLines &&
           a.junctionConnectors == b.junctionConnectors && a.junctionLanes == b.junctionLanes &&
           a.pathCrossings == b.pathCrossings && sameVec2s(a.planPoints, b.planPoints) &&
           sameBends(a.bendPieces, b.bendPieces);
}

// ---- validate ---------------------------------------------------------------------------------------------------

std::vector<std::string> validate(const LaneGraph& g) {
    std::vector<std::string> out;
    const std::size_t nPaths = g.paths.size();

    if (g.along.size() != g.points.size() || g.heights.size() != g.points.size())
        out.push_back(fmt("the curve arrays disagree: %zu points, %zu distances along, %zu heights", g.points.size(),
                          g.along.size(), g.heights.size()));
    const std::size_t curvePoints = std::min({g.points.size(), g.along.size(), g.heights.size()});
    if (g.laneTagNames.size() > kMaxTagNames)
        out.push_back(fmt("%zu lane tag names, more than the %u a tag set can hold", g.laneTagNames.size(), kMaxTagNames));
    if (g.vehicleTagNames.size() > kMaxTagNames)
        out.push_back(fmt("%zu vehicle tag names, more than the %u a tag set can hold", g.vehicleTagNames.size(), kMaxTagNames));

    // Whether `id` is in path `p`'s next list.
    auto listsAsNext = [&](uint32_t p, uint32_t id) {
        const Path& q = g.paths[p];
        if (!inRange(q.next, g.nextPaths.size())) return false;
        for (uint32_t k = q.next.first; k < q.next.end(); ++k)
            if (g.nextPaths[k].index == id) return true;
        return false;
    };
    // A stretch [from, to] on path p: a problem when it runs backwards or off the path.
    auto checkStretch = [&](const std::string& who, uint32_t p, Real from, Real to) {
        if (!(from <= to)) out.push_back(fmt("%s: its stretch on %s runs backwards (from %.2f to %.2f m)", who.c_str(),
                                             pathName(g, p).c_str(), from, to));
        else if (p < nPaths && (from < 0 || to > g.paths[p].length + kLengthTolerance))
            out.push_back(fmt("%s: its stretch on %s (%.2f to %.2f m) runs off the path (%.2f m long)", who.c_str(),
                              pathName(g, p).c_str(), from, to, g.paths[p].length));
    };

    // Paths.
    for (uint32_t i = 0; i < nPaths; ++i) {
        const Path& p = g.paths[i];
        const std::string who = pathName(g, i);
        if (static_cast<uint8_t>(p.kind) >= kPathKindCount) {
            out.push_back(fmt("path %u: unknown kind %u", i, static_cast<unsigned>(p.kind)));
            continue;
        }
        if (static_cast<uint8_t>(p.turn) >= kTurnCount)
            out.push_back(fmt("%s: unknown turn %u", who.c_str(), static_cast<unsigned>(p.turn)));
        if (p.laneTags & ~maskOfFirst(g.laneTagNames.size()))
            out.push_back(fmt("%s: lane tags 0x%x name tags past the %zu lane tag names", who.c_str(), p.laneTags,
                              g.laneTagNames.size()));
        if (p.vehicleTags & ~maskOfFirst(g.vehicleTagNames.size()))
            out.push_back(fmt("%s: vehicle tags 0x%x name tags past the %zu vehicle tag names", who.c_str(), p.vehicleTags,
                              g.vehicleTagNames.size()));
        if (!(p.length > 0) || std::isinf(p.length)) out.push_back(fmt("%s: length %.2f m", who.c_str(), p.length));
        if (!(p.width > 0) || std::isinf(p.width)) out.push_back(fmt("%s: width %.2f m", who.c_str(), p.width));
        if (!(p.tightestRadius > 0)) out.push_back(fmt("%s: tightest radius %.2f m", who.c_str(), p.tightestRadius));
        // Its bend profile: pieces from 0, rising, each on the path, each radius above zero (kNoBend for straight).
        if (!inRange(p.bends, g.bendPieces.size())) {
            out.push_back(fmt("%s: its bend profile (%u to %u) is out of range (%zu pieces)", who.c_str(), p.bends.first,
                              p.bends.end(), g.bendPieces.size()));
        } else if (p.bends.count > 0) {
            if (g.bendPieces[p.bends.first].from != 0)
                out.push_back(fmt("%s: its bend profile starts %.2f m along, not at 0", who.c_str(),
                                  g.bendPieces[p.bends.first].from));
            for (uint32_t k = p.bends.first; k < p.bends.end(); ++k) {
                const BendPiece& b = g.bendPieces[k];
                if (!(b.radius > 0)) out.push_back(fmt("%s: bend piece %u has radius %.2f m", who.c_str(), k - p.bends.first, b.radius));
                if (k > p.bends.first && !(b.from > g.bendPieces[k - 1].from))
                    out.push_back(fmt("%s: its bend pieces do not rise (piece %u at %.2f m)", who.c_str(), k - p.bends.first, b.from));
                if (!(b.from < p.length + kLengthTolerance))
                    out.push_back(fmt("%s: bend piece %u begins at %.2f m, past its end (%.2f m)", who.c_str(),
                                      k - p.bends.first, b.from, p.length));
            }
        }
        if (!(p.speedLimit >= 0) || std::isinf(p.speedLimit))
            out.push_back(fmt("%s: speed limit %.2f m/s", who.c_str(), p.speedLimit));

        // Its centre line.
        if (!inRange(p.curve, curvePoints)) {
            out.push_back(fmt("%s: its curve (points %u to %u) is out of range (%zu points)", who.c_str(), p.curve.first,
                              p.curve.end(), curvePoints));
        } else if (p.curve.count < kMinCurvePoints) {
            out.push_back(fmt("%s: its curve has %u points, fewer than %u", who.c_str(), p.curve.count, kMinCurvePoints));
        } else {
            if (g.along[p.curve.first] != 0)
                out.push_back(fmt("%s: its curve starts %.2f m along, not at 0", who.c_str(), g.along[p.curve.first]));
            for (uint32_t k = p.curve.first + 1; k < p.curve.end(); ++k)
                if (!(g.along[k] >= g.along[k - 1])) {
                    out.push_back(fmt("%s: its curve's distances along go backwards at point %u", who.c_str(), k - p.curve.first));
                    break;
                }
            const Real last = g.along[p.curve.end() - 1];
            if (!(std::fabs(last - p.length) <= kLengthTolerance))
                out.push_back(fmt("%s: its length %.2f m is not its curve's %.2f m", who.c_str(), p.length, last));
        }

        // What follows it.
        if (!inRange(p.next, g.nextPaths.size())) {
            out.push_back(fmt("%s: its next list (%u to %u) is out of range (%zu entries)", who.c_str(), p.next.first,
                              p.next.end(), g.nextPaths.size()));
        } else {
            for (uint32_t k = p.next.first; k < p.next.end(); ++k) {
                const uint32_t n = g.nextPaths[k].index;
                if (n >= nPaths) {
                    out.push_back(fmt("%s: next path %u is out of range (%zu paths)", who.c_str(), n, nPaths));
                    continue;
                }
                if (k > p.next.first && !(g.nextPaths[k - 1] < g.nextPaths[k]))
                    out.push_back(fmt("%s: its next list is not in id order (%u after %u)", who.c_str(), n,
                                      g.nextPaths[k - 1].index));
                if (p.kind == PathKind::Lane && g.paths[n].kind == PathKind::Connector && g.paths[n].from.index != i)
                    out.push_back(fmt("%s lists %s as next, but that connector leaves %s", who.c_str(), pathName(g, n).c_str(),
                                      pathName(g, g.paths[n].from.index).c_str()));
            }
        }

        // Its regulations.
        if (!inRange(p.regulations, g.regulations.size())) {
            out.push_back(fmt("%s: its regulations (%u to %u) are out of range (%zu regulations)", who.c_str(),
                              p.regulations.first, p.regulations.end(), g.regulations.size()));
        } else {
            for (uint32_t k = p.regulations.first; k < p.regulations.end(); ++k) {
                if (g.regulations[k].path.index != i)
                    out.push_back(fmt("%s: regulation %u in its list stands on %s", who.c_str(), k,
                                      pathName(g, g.regulations[k].path.index).c_str()));
                if (k > p.regulations.first &&
                    !(g.regulations[k].from > g.regulations[k - 1].from ||
                      (g.regulations[k].from == g.regulations[k - 1].from && g.regulations[k].to >= g.regulations[k - 1].to)))
                    out.push_back(fmt("%s: its regulations are out of order (regulation %u from %.2f m after %.2f m)",
                                      who.c_str(), k, g.regulations[k].from, g.regulations[k - 1].from));
            }
        }

        if (p.kind == PathKind::Lane) {
            if (p.junction.valid()) out.push_back(fmt("%s: a lane names junction %u", who.c_str(), p.junction.index));
            if (p.from.valid()) out.push_back(fmt("%s: a lane names a from-lane (%u)", who.c_str(), p.from.index));
            if (p.turn != Turn::None) out.push_back(fmt("%s: a lane has a turn (%s)", who.c_str(), turnWord(p.turn)));
            if (p.waitInsideAt != kNoWaitInside) out.push_back(fmt("%s: a lane has a place to wait inside a junction", who.c_str()));
            if (!p.section.valid() || p.section.index >= g.sections.size()) {
                out.push_back(fmt("%s: its section %u is out of range (%zu sections)", who.c_str(), p.section.index,
                                  g.sections.size()));
            } else {
                const Section& s = g.sections[p.section.index];
                if (inRange(s.lanes, g.sectionLanes.size()) &&
                    (p.indexInSection >= s.lanes.count || g.sectionLanes[s.lanes.first + p.indexInSection].index != i))
                    out.push_back(fmt("%s says it is lane %u of section %u, but the section does not have it there",
                                      who.c_str(), p.indexInSection, p.section.index));
            }
        } else {
            if (p.section.valid()) out.push_back(fmt("%s: a connector names section %u", who.c_str(), p.section.index));
            if (p.turn == Turn::None) out.push_back(fmt("%s: a connector with no turn", who.c_str()));
            if (p.waitInsideAt != kNoWaitInside && !(p.waitInsideAt >= 0 && p.waitInsideAt <= p.length))
                out.push_back(fmt("%s: its place to wait inside (%.2f m) is off the path", who.c_str(), p.waitInsideAt));
            if (!p.junction.valid() || p.junction.index >= g.junctions.size()) {
                out.push_back(fmt("%s: its junction %u is out of range (%zu junctions)", who.c_str(), p.junction.index,
                                  g.junctions.size()));
            } else {
                const Junction& j = g.junctions[p.junction.index];
                if (inRange(j.connectors, g.junctionConnectors.size()) &&
                    !std::binary_search(g.junctionConnectors.begin() + j.connectors.first,
                                        g.junctionConnectors.begin() + j.connectors.end(), PathId(i)))
                    out.push_back(fmt("%s says it runs through junction %u, but the junction does not list it", who.c_str(),
                                      p.junction.index));
            }
            if (p.from.index >= nPaths || g.paths[p.from.index].kind != PathKind::Lane) {
                out.push_back(fmt("%s: its from-lane %u is not a lane", who.c_str(), p.from.index));
            } else if (!listsAsNext(p.from.index, i)) {
                out.push_back(fmt("%s leaves %s, but that lane does not list it as next", who.c_str(),
                                  pathName(g, p.from.index).c_str()));
            }
            if (inRange(p.next, g.nextPaths.size()) &&
                (p.next.count != 1 || g.nextPaths[p.next.first].index >= nPaths ||
                 g.paths[g.nextPaths[p.next.first].index].kind != PathKind::Lane))
                out.push_back(fmt("%s: it must lead into exactly one lane (it lists %u paths)", who.c_str(), p.next.count));
        }
    }

    // Sections.
    for (uint32_t s = 0; s < g.sections.size(); ++s) {
        const Section& sec = g.sections[s];
        if (sec.roadClass >= g.roadClassNames.size())
            out.push_back(fmt("section %u: road class %u is out of range (%zu names)", s, sec.roadClass, g.roadClassNames.size()));
        if (!inRange(sec.lanes, g.sectionLanes.size())) {
            out.push_back(fmt("section %u: its lanes (%u to %u) are out of range (%zu entries)", s, sec.lanes.first,
                              sec.lanes.end(), g.sectionLanes.size()));
            continue;
        }
        if (sec.lanes.count == 0) out.push_back(fmt("section %u has no lanes", s));
        if (!inRange(sec.lines, g.laneLines.size()))
            out.push_back(fmt("section %u: its lines (%u to %u) are out of range (%zu entries)", s, sec.lines.first,
                              sec.lines.end(), g.laneLines.size()));
        else if (sec.lanes.count > 0 && sec.lines.count != sec.lanes.count - 1)
            out.push_back(fmt("section %u: %u lanes need %u lines between them, it has %u", s, sec.lanes.count,
                              sec.lanes.count - 1, sec.lines.count));
        for (uint32_t k = 0; k < sec.lanes.count; ++k) {
            const uint32_t l = g.sectionLanes[sec.lanes.first + k].index;
            if (l >= nPaths) {
                out.push_back(fmt("section %u: its lane %u is path %u, out of range (%zu paths)", s, k, l, nPaths));
            } else if (g.paths[l].kind != PathKind::Lane) {
                out.push_back(fmt("section %u: its lane %u is %s", s, k, pathName(g, l).c_str()));
            } else if (g.paths[l].section.index != s || g.paths[l].indexInSection != k) {
                out.push_back(fmt("section %u lists %s as its lane %u, but that lane says it is lane %u of section %u", s,
                                  pathName(g, l).c_str(), k, g.paths[l].indexInSection, g.paths[l].section.index));
            }
        }
    }

    // Junctions.
    for (uint32_t j = 0; j < g.junctions.size(); ++j) {
        const Junction& jn = g.junctions[j];
        if (jn.control >= g.controlNames.size())
            out.push_back(fmt("junction %u: control %u is out of range (%zu names)", j, jn.control, g.controlNames.size()));
        if (!inRange(jn.box, g.planPoints.size()))
            out.push_back(fmt("junction %u: its box (%u to %u) is out of range (%zu plan points)", j, jn.box.first,
                              jn.box.end(), g.planPoints.size()));
        else if (jn.box.count < kMinOutlinePoints)
            out.push_back(fmt("junction %u: its box has %u corners", j, jn.box.count));
        if (!inRange(jn.connectors, g.junctionConnectors.size())) {
            out.push_back(fmt("junction %u: its connectors (%u to %u) are out of range (%zu entries)", j,
                              jn.connectors.first, jn.connectors.end(), g.junctionConnectors.size()));
        } else {
            for (uint32_t k = jn.connectors.first; k < jn.connectors.end(); ++k) {
                const uint32_t c = g.junctionConnectors[k].index;
                if (c >= nPaths || g.paths[c].kind != PathKind::Connector)
                    out.push_back(fmt("junction %u lists %s as a connector", j, pathName(g, c).c_str()));
                else if (g.paths[c].junction.index != j)
                    out.push_back(fmt("junction %u lists %s, but that connector says it runs through junction %u", j,
                                      pathName(g, c).c_str(), g.paths[c].junction.index));
                if (k > jn.connectors.first && !(g.junctionConnectors[k - 1] < g.junctionConnectors[k]))
                    out.push_back(fmt("junction %u: its connectors are not in id order (%u after %u)", j, c,
                                      g.junctionConnectors[k - 1].index));
            }
        }
        if (!inRange(jn.crossings, g.crossings.size())) {
            out.push_back(fmt("junction %u: its crossing points (%u to %u) are out of range (%zu)", j, jn.crossings.first,
                              jn.crossings.end(), g.crossings.size()));
        } else {
            for (uint32_t k = jn.crossings.first; k < jn.crossings.end(); ++k)
                if (g.crossings[k].junction.index != j)
                    out.push_back(fmt("junction %u lists crossing point %u, which says it lies in junction %u", j, k,
                                      g.crossings[k].junction.index));
        }
        if (!inRange(jn.crosswalks, g.crosswalks.size())) {
            out.push_back(fmt("junction %u: its crosswalks (%u to %u) are out of range (%zu)", j, jn.crosswalks.first,
                              jn.crosswalks.end(), g.crosswalks.size()));
        } else {
            for (uint32_t k = jn.crosswalks.first; k < jn.crosswalks.end(); ++k)
                if (g.crosswalks[k].junction.index != j)
                    out.push_back(fmt("junction %u lists crosswalk %u, which says it lies in junction %u", j, k,
                                      g.crosswalks[k].junction.index));
        }
        // Its arriving and leaving lanes: lanes, in id order, ending (arriving) or beginning (leaving) at its outline.
        std::vector<Vec2> outline;
        if (inRange(jn.box, g.planPoints.size()))
            outline.assign(g.planPoints.begin() + jn.box.first, g.planPoints.begin() + jn.box.end());
        for (int side = 0; side < 2; ++side) {
            const bool arriving = side == 0;
            const Range& r = arriving ? jn.arriving : jn.leaving;
            const char* how = arriving ? "arriving" : "leaving";
            if (!inRange(r, g.junctionLanes.size())) {
                out.push_back(fmt("junction %u: its %s lanes (%u to %u) are out of range (%zu entries)", j, how, r.first,
                                  r.end(), g.junctionLanes.size()));
                continue;
            }
            for (uint32_t k = r.first; k < r.end(); ++k) {
                const uint32_t p = g.junctionLanes[k].index;
                if (p >= nPaths || g.paths[p].kind != PathKind::Lane) {
                    out.push_back(fmt("junction %u lists %s as %s, not a lane", j, pathName(g, p).c_str(), how));
                    continue;
                }
                if (k > r.first && !(g.junctionLanes[k - 1] < g.junctionLanes[k]))
                    out.push_back(fmt("junction %u: its %s lanes are not in id order (%u after %u)", j, how, p,
                                      g.junctionLanes[k - 1].index));
                if (outline.size() < kMinOutlinePoints) continue;
                const Real at = arriving ? g.paths[p].length : 0;
                const Vec2 end = g.pointAt(PathId(p), at);
                const Real off = distToPolyline(end, outline, true);
                if (!(off <= kJunctionEndReach))
                    out.push_back(fmt("junction %u: %s lane %u %s at (%.1f, %.1f), %.2f m from its outline (at most %.2f)",
                                      j, how, p, arriving ? "ends" : "begins", end.x, end.y, off, kJunctionEndReach));
            }
        }
    }
    // Whether entry k of a junction-by-junction array lies in junction j's range of it.
    auto insideJunction = [&](uint32_t j, uint32_t k, Range Junction::*which) {
        const Range& r = g.junctions[j].*which;
        return k >= r.first && k < r.end();
    };

    // Crossing points. What leads into each path (the paths listing it as next), for the splits: two paths split where
    // one path leads into both.
    std::vector<std::vector<uint32_t>> ledFrom;
    if (!g.crossings.empty()) {
        ledFrom.resize(nPaths);
        for (uint32_t i = 0; i < nPaths; ++i)
            if (inRange(g.paths[i].next, g.nextPaths.size()))
                for (uint32_t k = g.paths[i].next.first; k < g.paths[i].next.end(); ++k)
                    if (g.nextPaths[k].index < nPaths) ledFrom[g.nextPaths[k].index].push_back(i);
    }
    // Whether some path is in both lists (each in id order, as next lists and ledFrom are).
    auto shareOne = [](const PathId* a, std::size_t na, const PathId* b, std::size_t nb) {
        for (std::size_t i = 0, j = 0; i < na && j < nb;) {
            if (a[i] == b[j]) return true;
            if (a[i] < b[j]) ++i;
            else ++j;
        }
        return false;
    };
    auto nextOf = [&](uint32_t p, std::size_t& n) -> const PathId* {
        const Range r = g.paths[p].next;
        n = inRange(r, g.nextPaths.size()) ? r.count : 0;
        return g.nextPaths.data() + (n ? r.first : 0);
    };
    for (uint32_t c = 0; c < g.crossings.size(); ++c) {
        const CrossingPoint& x = g.crossings[c];
        const std::string who = fmt("crossing point %u", c);
        if (x.a.index >= nPaths) {
            out.push_back(fmt("%s: path %u is out of range (%zu paths)", who.c_str(), x.a.index, nPaths));
            continue;
        }
        if (static_cast<uint8_t>(x.meeting) >= kMeetingCount)
            out.push_back(fmt("%s: unknown kind of meeting %u", who.c_str(), static_cast<unsigned>(x.meeting)));
        checkStretch(who, x.a.index, x.aFrom, x.aTo);
        switch (x.otherSide) {
        case OtherSide::Path:
            if (x.other >= nPaths) {
                out.push_back(fmt("%s: other path %u is out of range (%zu paths)", who.c_str(), x.other, nPaths));
            } else {
                if (!(x.a.index < x.other))
                    out.push_back(fmt("%s: its paths must be stored lower id first (%u, %u)", who.c_str(), x.a.index, x.other));
                checkStretch(who, x.other, x.otherFrom, x.otherTo);
                const Path& pa = g.paths[x.a.index];
                const Path& pb = g.paths[x.other];
                const std::string an = pathName(g, x.a.index), bn = pathName(g, x.other);
                // filed under the junction both run through, or under none when they do not run through one
                const bool oneJunction = pa.kind == PathKind::Connector && pb.kind == PathKind::Connector &&
                                         pa.junction.valid() && pa.junction == pb.junction;
                if (x.junction.valid() && !(oneJunction && pa.junction == x.junction))
                    out.push_back(fmt("%s is filed under junction %u, but %s and %s are not both its connectors", who.c_str(),
                                      x.junction.index, an.c_str(), bn.c_str()));
                if (!x.junction.valid() && oneJunction)
                    out.push_back(fmt("%s is filed under no junction, but %s and %s are both connectors of junction %u",
                                      who.c_str(), an.c_str(), bn.c_str(), pa.junction.index));
                if (x.meeting == Meeting::Split) {
                    if (x.aFrom != 0 || x.otherFrom != 0)
                        out.push_back(fmt("%s: a split, but its stretches start %.2f m along %s and %.2f m along %s, not at "
                                          "their starts", who.c_str(), x.aFrom, an.c_str(), x.otherFrom, bn.c_str()));
                    const std::vector<uint32_t>& fa = ledFrom[x.a.index];
                    const std::vector<uint32_t>& fb = ledFrom[x.other];
                    bool shared = false;
                    for (std::size_t i = 0, j = 0; i < fa.size() && j < fb.size() && !shared;) {
                        shared = fa[i] == fb[j];
                        if (fa[i] < fb[j]) ++i;
                        else ++j;
                    }
                    if (!shared)
                        out.push_back(fmt("%s: a split, but no path leads into both %s and %s", who.c_str(), an.c_str(), bn.c_str()));
                }
                if (x.meeting == Meeting::Join) {
                    if (std::fabs(x.aTo - pa.length) > kLengthTolerance || std::fabs(x.otherTo - pb.length) > kLengthTolerance)
                        out.push_back(fmt("%s: a join, but its stretches end %.2f m along %s (%.2f m long) and %.2f m along "
                                          "%s (%.2f m long), not at their ends", who.c_str(), x.aTo, an.c_str(), pa.length,
                                          x.otherTo, bn.c_str(), pb.length));
                    std::size_t na = 0, nb = 0;
                    const PathId* qa = nextOf(x.a.index, na);
                    const PathId* qb = nextOf(x.other, nb);
                    if (!shareOne(qa, na, qb, nb))
                        out.push_back(fmt("%s: a join, but %s and %s lead into no path in common", who.c_str(), an.c_str(), bn.c_str()));
                }
            }
            break;
        case OtherSide::Crosswalk:
        case OtherSide::Track: {
            const bool walk = x.otherSide == OtherSide::Crosswalk;
            const std::size_t n = walk ? g.crosswalks.size() : g.tracks.size();
            if (x.other >= n)
                out.push_back(fmt("%s: %s %u is out of range (%zu)", who.c_str(), walk ? "crosswalk" : "track", x.other, n));
            else if (walk && x.junction != g.crosswalks[x.other].junction)
                out.push_back(fmt("%s is filed under %s, but its crosswalk lies in %s", who.c_str(),
                                  x.junction.valid() ? fmt("junction %u", x.junction.index).c_str() : "no junction",
                                  g.crosswalks[x.other].junction.valid()
                                      ? fmt("junction %u", g.crosswalks[x.other].junction.index).c_str() : "none"));
            if (x.meeting != Meeting::Cross)
                out.push_back(fmt("%s: a %s can only be crossed, not %s", who.c_str(), walk ? "crosswalk" : "track",
                                  meetingWord(x.meeting)));
            if (!(x.otherFrom <= x.otherTo) || x.otherFrom < 0)
                out.push_back(fmt("%s: its stretch on %s runs backwards (from %.2f to %.2f m)", who.c_str(),
                                  otherName(g, x).c_str(), x.otherFrom, x.otherTo));
            break;
        }
        default:
            out.push_back(fmt("%s: unknown other side %u", who.c_str(), static_cast<unsigned>(x.otherSide)));
        }
        if (x.junction.valid()) {
            if (x.junction.index >= g.junctions.size())
                out.push_back(fmt("%s: junction %u is out of range (%zu junctions)", who.c_str(), x.junction.index,
                                  g.junctions.size()));
            else if (!insideJunction(x.junction.index, c, &Junction::crossings))
                out.push_back(fmt("%s says it lies in junction %u, but the junction's list does not hold it", who.c_str(),
                                  x.junction.index));
        }
        // In order within its junction: by (a, other side, other, aFrom).
        if (c > 0 && g.crossings[c - 1].junction == x.junction) {
            const CrossingPoint& y = g.crossings[c - 1];
            const bool ordered =
                y.a.index != x.a.index ? y.a.index < x.a.index
                : y.otherSide != x.otherSide ? static_cast<uint8_t>(y.otherSide) < static_cast<uint8_t>(x.otherSide)
                : y.other != x.other ? y.other < x.other
                : y.aFrom <= x.aFrom;
            if (!ordered) out.push_back(fmt("%s is out of order after crossing point %u", who.c_str(), c - 1));
            // one meeting recorded once: two records of one pair never overlap on both
            if (y.a == x.a && y.otherSide == x.otherSide && y.other == x.other && x.aFrom <= y.aTo && y.aFrom <= x.aTo &&
                x.otherFrom <= y.otherTo && y.otherFrom <= x.otherTo)
                out.push_back(fmt("%s and crossing point %u record one meeting of %s and %s twice (their stretches overlap on "
                                  "both)", who.c_str(), c - 1, pathName(g, x.a.index).c_str(), otherName(g, x).c_str()));
        }
        // those on no junction after every junction's own
        if (c > 0 && !g.crossings[c - 1].junction.valid() && x.junction.valid())
            out.push_back(fmt("%s (junction %u) comes after crossing point %u, which lies on no junction", who.c_str(),
                              x.junction.index, c - 1));
    }
    // Each path's list of its crossing points: in range, each naming the path, by where its stretch on the path begins
    // (then id); and together they hold each record once for each path it names (a list holds no record twice, and
    // names only its own path, so equal counts mean every record is in the list of each of its paths).
    {
        std::size_t named = 0, listed = 0;
        for (const CrossingPoint& x : g.crossings) named += x.otherSide == OtherSide::Path ? 2 : 1;
        for (uint32_t i = 0; i < nPaths; ++i) {
            const Range& r = g.paths[i].crossings;
            const std::string who = pathName(g, i);
            if (!inRange(r, g.pathCrossings.size())) {
                out.push_back(fmt("%s: its crossing points (%u to %u) are out of range (%zu entries)", who.c_str(), r.first,
                                  r.end(), g.pathCrossings.size()));
                continue;
            }
            listed += r.count;
            Real lastStart = 0;
            uint32_t lastId = kNoIndex;
            for (uint32_t k = r.first; k < r.end(); ++k) {
                const uint32_t c = g.pathCrossings[k].index;
                if (c >= g.crossings.size()) {
                    out.push_back(fmt("%s lists crossing point %u, out of range (%zu)", who.c_str(), c, g.crossings.size()));
                    lastId = kNoIndex;
                    continue;
                }
                const CrossingPoint& x = g.crossings[c];
                const bool asA = x.a.index == i, asOther = x.otherSide == OtherSide::Path && x.other == i;
                if (!asA && !asOther) {
                    out.push_back(fmt("%s lists crossing point %u, which is not on it", who.c_str(), c));
                    lastId = kNoIndex;
                    continue;
                }
                const Real start = asA ? x.aFrom : x.otherFrom;
                if (lastId != kNoIndex && !(lastStart < start || (lastStart == start && lastId < c)))
                    out.push_back(fmt("%s: its crossing points are out of order (%u, from %.2f m, after %u, from %.2f m)",
                                      who.c_str(), c, start, lastId, lastStart));
                lastStart = start;
                lastId = c;
            }
        }
        if (listed != named)
            out.push_back(fmt("the paths' crossing point lists hold %zu entries, but the crossing points name paths %zu times",
                              listed, named));
    }

    // Crosswalks and tracks.
    for (uint32_t w = 0; w < g.crosswalks.size(); ++w) {
        const Crosswalk& cw = g.crosswalks[w];
        if (!inRange(cw.outline, g.planPoints.size()) || cw.outline.count < kMinOutlinePoints)
            out.push_back(fmt("crosswalk %u: its outline (%u points from %u) is out of range or too small (%zu plan points)",
                              w, cw.outline.count, cw.outline.first, g.planPoints.size()));
        if (cw.junction.valid()) {
            if (cw.junction.index >= g.junctions.size())
                out.push_back(fmt("crosswalk %u: junction %u is out of range (%zu junctions)", w, cw.junction.index,
                                  g.junctions.size()));
            else if (!insideJunction(cw.junction.index, w, &Junction::crosswalks))
                out.push_back(fmt("crosswalk %u says it lies in junction %u, but the junction's list does not hold it", w,
                                  cw.junction.index));
        }
    }
    for (uint32_t t = 0; t < g.tracks.size(); ++t)
        if (!inRange(g.tracks[t].line, g.planPoints.size()) || g.tracks[t].line.count < kMinLinePoints)
            out.push_back(fmt("track %u: its line (%u points from %u) is out of range or too short (%zu plan points)", t,
                              g.tracks[t].line.count, g.tracks[t].line.first, g.planPoints.size()));

    // Regulations: each in its path's list (so the lists cover the array once), kinds known, on the path.
    for (uint32_t r = 0; r < g.regulations.size(); ++r) {
        const Regulation& reg = g.regulations[r];
        if (reg.kind >= g.regulationKindNames.size())
            out.push_back(fmt("regulation %u: kind %u is out of range (%zu names)", r, reg.kind, g.regulationKindNames.size()));
        if (reg.path.index >= nPaths) {
            out.push_back(fmt("regulation %u: path %u is out of range (%zu paths)", r, reg.path.index, nPaths));
            continue;
        }
        const Path& p = g.paths[reg.path.index];
        if (r < p.regulations.first || r >= p.regulations.end())
            out.push_back(fmt("regulation %u stands on %s, but that path's list does not hold it", r,
                              pathName(g, reg.path.index).c_str()));
        if (!(reg.from >= 0 && reg.from <= p.length + kLengthTolerance) || !(reg.to >= 0 && reg.to <= p.length + kLengthTolerance))
            out.push_back(fmt("regulation %u: from %.2f to %.2f m, off %s (%.2f m long)", r, reg.from, reg.to,
                              pathName(g, reg.path.index).c_str(), p.length));
        else if (reg.to < reg.from)
            out.push_back(fmt("regulation %u: runs backwards on %s (from %.2f to %.2f m)", r,
                              pathName(g, reg.path.index).c_str(), reg.from, reg.to));
    }

    // Car spaces.
    for (uint32_t s = 0; s < g.carSpaces.size(); ++s) {
        const CarSpace& sp = g.carSpaces[s];
        const std::string who = fmt("car space %u", s);
        if (sp.lane.index >= nPaths || g.paths[sp.lane.index].kind != PathKind::Lane) {
            out.push_back(fmt("%s: %s is not a lane", who.c_str(), pathName(g, sp.lane.index).c_str()));
            continue;
        }
        checkStretch(who, sp.lane.index, sp.from, sp.to);
        if (s > 0) {
            const CarSpace& y = g.carSpaces[s - 1];
            if (y.lane.index > sp.lane.index || (y.lane == sp.lane && y.from > sp.from))
                out.push_back(fmt("%s is out of order after car space %u", who.c_str(), s - 1));
        }
    }
    return out;
}

// ---- where a car can go -----------------------------------------------------------------------------------------

std::vector<PathId> stepsFrom(const LaneGraph& g, PathId from) { return stepsFrom(g, from, ReachRules()); }

std::vector<PathId> stepsFrom(const LaneGraph& g, PathId from, const ReachRules& rules) {
    std::vector<PathId> out;
    if (!from.valid() || from.index >= g.paths.size()) return out;
    const auto isOpen = [&](PathId q) { return rules.open.empty() || (q.index < rules.open.size() && rules.open[q.index] != 0); };
    if (!isOpen(from)) return out;
    const Path& p = g.paths[from.index];
    if (inRange(p.next, g.nextPaths.size()))
        for (uint32_t k = p.next.first; k < p.next.end(); ++k)
            if (g.nextPaths[k].index < g.paths.size() && isOpen(g.nextPaths[k])) out.push_back(g.nextPaths[k]);
    if (!rules.laneChanges || p.kind != PathKind::Lane || p.section.index >= g.sections.size()) return out;
    const Section& s = g.sections[p.section.index];
    if (!inRange(s.lanes, g.sectionLanes.size()) || !inRange(s.lines, g.laneLines.size())) return out;
    const uint32_t k = p.indexInSection;
    // the line on its offside (between lane k - 1 and k), crossed from the kerb side; the one on its kerb side (between
    // lane k and k + 1), crossed from the offside
    if (k > 0 && k - 1 < s.lines.count && g.laneLines[s.lines.first + k - 1].crossFromKerbside &&
        isOpen(g.sectionLanes[s.lanes.first + k - 1]))
        out.push_back(g.sectionLanes[s.lanes.first + k - 1]);
    if (k + 1 < s.lanes.count && k < s.lines.count && g.laneLines[s.lines.first + k].crossFromOffside &&
        isOpen(g.sectionLanes[s.lanes.first + k + 1]))
        out.push_back(g.sectionLanes[s.lanes.first + k + 1]);
    return out;
}

ReachableParts reachableParts(const LaneGraph& g) { return reachableParts(g, ReachRules()); }

ReachableParts reachableParts(const LaneGraph& g, const ReachRules& rules) {
    // Tarjan's walk, kept on an explicit stack (an island's lanes would overflow the call stack): each path gets the
    // order it was first reached in and the lowest such order it can get back to; a path that can get back to no path
    // reached before it closes a part, everything above it on the stack.
    const uint32_t n = static_cast<uint32_t>(g.paths.size());
    const uint32_t kUnseen = kNoIndex;
    std::vector<uint32_t> order(n, kUnseen), low(n, 0), tarjanPart(n, kUnseen);
    std::vector<char> onStack(n, 0);
    std::vector<uint32_t> stack;
    struct Frame { uint32_t path; std::vector<PathId> steps; std::size_t next; };
    std::vector<Frame> walk;
    uint32_t reached = 0, parts = 0;
    for (uint32_t root = 0; root < n; ++root) {
        if (order[root] != kUnseen) continue;
        walk.push_back(Frame{root, stepsFrom(g, PathId(root), rules), 0});
        order[root] = low[root] = reached++;
        stack.push_back(root);
        onStack[root] = 1;
        while (!walk.empty()) {
            Frame& f = walk.back();
            if (f.next < f.steps.size()) {
                const uint32_t q = f.steps[f.next++].index;
                if (order[q] == kUnseen) {
                    order[q] = low[q] = reached++;
                    stack.push_back(q);
                    onStack[q] = 1;
                    walk.push_back(Frame{q, stepsFrom(g, PathId(q), rules), 0});
                } else if (onStack[q]) {
                    low[f.path] = std::min(low[f.path], order[q]);
                }
                continue;
            }
            const uint32_t p = f.path;
            walk.pop_back();
            if (!walk.empty()) low[walk.back().path] = std::min(low[walk.back().path], low[p]);
            if (low[p] != order[p]) continue;
            for (;;) {
                const uint32_t q = stack.back();
                stack.pop_back();
                onStack[q] = 0;
                tarjanPart[q] = parts;
                if (q == p) break;
            }
            ++parts;
        }
    }
    // number the parts by their lowest path
    ReachableParts out;
    out.partOf.assign(n, 0);
    std::vector<uint32_t> number(parts, kUnseen);
    for (uint32_t p = 0; p < n; ++p) {
        uint32_t& id = number[tarjanPart[p]];
        if (id == kUnseen) {
            id = static_cast<uint32_t>(out.sizes.size());
            out.sizes.push_back(0);
        }
        out.partOf[p] = id;
        ++out.sizes[id];
    }
    for (uint32_t i = 0; i < out.sizes.size(); ++i)
        if (out.sizes[i] > out.sizes[out.largest]) out.largest = i;
    return out;
}

// ---- text dump --------------------------------------------------------------------------------------------------

std::string dumpSection(const LaneGraph& g, SectionId id) {
    if (id.index >= g.sections.size()) return fmt("section %u: out of range\n", id.index);
    const Section& s = g.sections[id.index];
    std::string out = fmt("section %u: %s, lanes offside to kerb:", id.index, nameOf(g.roadClassNames, s.roadClass).c_str());
    const uint32_t laneEnd = safeEnd(s.lanes, g.sectionLanes.size());
    for (uint32_t k = s.lanes.first; k < laneEnd; ++k)
        out += fmt("%s %s", k == s.lanes.first ? "" : ",", pathName(g, g.sectionLanes[k].index).c_str());
    out += "\n";
    const uint32_t lineEnd = safeEnd(s.lines, g.laneLines.size());
    for (uint32_t k = s.lines.first; k < lineEnd; ++k) {
        const uint32_t i = k - s.lines.first;   // between lane i and lane i + 1
        const std::string a = s.lanes.first + i < laneEnd ? pathName(g, g.sectionLanes[s.lanes.first + i].index) : "?";
        const std::string b = s.lanes.first + i + 1 < laneEnd ? pathName(g, g.sectionLanes[s.lanes.first + i + 1].index) : "?";
        const LaneLine& l = g.laneLines[k];
        std::string how;
        if (l.crossFromOffside && l.crossFromKerbside) how = "broken, may be crossed either way";
        else if (l.crossFromOffside) how = "may be crossed only from " + a;
        else if (l.crossFromKerbside) how = "may be crossed only from " + b;
        else how = "solid, may not be crossed";
        out += fmt("  line between %s and %s: %s\n", a.c_str(), b.c_str(), how.c_str());
    }
    return out;
}

std::string dumpPath(const LaneGraph& g, PathId id) {
    if (id.index >= g.paths.size()) return fmt("path %u: out of range\n", id.index);
    const Path& p = g.paths[id.index];
    std::string out = pathName(g, id.index) + ": ";
    const std::string nexts = [&] {
        std::string s;
        for (uint32_t k = p.next.first; k < safeEnd(p.next, g.nextPaths.size()); ++k)
            s += (s.empty() ? "" : ", ") + pathName(g, g.nextPaths[k].index);
        return s.empty() ? std::string("nothing") : s;
    }();
    if (p.kind == PathKind::Lane) {
        uint32_t laneCount = 0;
        if (p.section.index < g.sections.size()) laneCount = g.sections[p.section.index].lanes.count;
        const char* where = laneCount == 1 ? "offside and kerb"
                            : p.indexInSection == 0 ? "offside"
                            : p.indexInSection + 1 == laneCount ? "kerb"
                            : "between";
        out += fmt("section %u, lane %u of %u (%s), %s, %.1f m wide, tightest radius %s, tags %s, open to %s, "
                   "speed limit %.1f m/s, next: %s\n",
                   p.section.index, p.indexInSection, laneCount, where, metres(p.length).c_str(), p.width,
                   metres(p.tightestRadius).c_str(), namesOf(g.laneTagNames, p.laneTags).c_str(),
                   namesOf(g.vehicleTagNames, p.vehicleTags).c_str(), p.speedLimit, nexts.c_str());
    } else {
        const std::string wait = p.waitInsideAt == kNoWaitInside ? std::string("no place to wait inside")
                                                                  : fmt("may wait inside at %.1f m", p.waitInsideAt);
        out += fmt("%s, from %s into %s, %s, tightest radius %s, %.1f m wide, junction %u, %s, tags %s, open to %s, "
                   "speed limit %.1f m/s\n",
                   turnWord(p.turn), pathName(g, p.from.index).c_str(), nexts.c_str(), metres(p.length).c_str(),
                   metres(p.tightestRadius).c_str(), p.width, p.junction.index, wait.c_str(),
                   namesOf(g.laneTagNames, p.laneTags).c_str(), namesOf(g.vehicleTagNames, p.vehicleTags).c_str(),
                   p.speedLimit);
    }
    out += "  curve:";
    const std::size_t curveEnd = std::min({g.points.size(), g.along.size(), g.heights.size(),
                                           static_cast<std::size_t>(safeEnd(p.curve, g.points.size()))});
    for (std::size_t k = p.curve.first; k < curveEnd; ++k)
        out += fmt(" (%.1f, %.1f, %.1f)", g.points[k].x, g.points[k].y, g.heights[k]);
    out += "\n";
    // Its bend profile (issue #281): the tightest stretch and where, then every piece (where it begins, its radius).
    if (p.bends.count == 0) {
        out += "  bends: none measured; the whole path at its tightest radius (" + metres(p.tightestRadius) + ")\n";
    } else {
        const BendSpan t = g.tightestBend(id);
        out += fmt("  bends: the tightest %s, from %.1f to %.1f m along; pieces:", metres(t.radius).c_str(), t.from, t.to);
        for (uint32_t k = p.bends.first; k < safeEnd(p.bends, g.bendPieces.size()); ++k)
            out += fmt("%s %.1f m %s", k == p.bends.first ? "" : ",", g.bendPieces[k].from,
                       metres(g.bendPieces[k].radius).c_str());
        out += "\n";
    }
    for (uint32_t k = p.regulations.first; k < safeEnd(p.regulations, g.regulations.size()); ++k) {
        const Regulation& r = g.regulations[k];
        out += r.to == r.from ? fmt("  regulation %u: %s at %.1f m, value %.1f\n", k,
                                    nameOf(g.regulationKindNames, r.kind).c_str(), r.from, r.value)
                              : fmt("  regulation %u: %s from %.1f to %.1f m, value %.1f\n", k,
                                    nameOf(g.regulationKindNames, r.kind).c_str(), r.from, r.to, r.value);
    }
    if (p.crossings.count > 0) {
        out += "  crossing points along it:";
        for (uint32_t k = p.crossings.first; k < safeEnd(p.crossings, g.pathCrossings.size()); ++k) {
            const uint32_t c = g.pathCrossings[k].index;
            const char* sep = k == p.crossings.first ? "" : ",";
            if (c >= g.crossings.size()) {
                out += fmt("%s %u (out of range)", sep, c);
                continue;
            }
            const CrossingPoint& x = g.crossings[c];
            out += fmt("%s %u from %.1f m", sep, c, x.a == id ? x.aFrom : x.otherFrom);
        }
        out += "\n";
    }
    return out;
}

std::string dumpCrossing(const LaneGraph& g, CrossingId id) {
    if (id.index >= g.crossings.size()) return fmt("crossing point %u: out of range\n", id.index);
    const CrossingPoint& c = g.crossings[id.index];
    // where it is filed: its junction; two paths of no one junction, the junctions they run through; else on the road
    auto junctionOf = [&](uint32_t p) {
        if (p >= g.paths.size() || g.paths[p].kind != PathKind::Connector) return std::string("a lane");
        return fmt("junction %u", g.paths[p].junction.index);
    };
    const std::string where = c.junction.valid() ? fmt("junction %u", c.junction.index)
                              : c.otherSide == OtherSide::Path
                                  ? "between " + junctionOf(c.a.index) + " and " + junctionOf(c.other)
                                  : std::string("on the road");
    const std::string a = pathName(g, c.a.index), b = otherName(g, c);
    return fmt("crossing point %u, %s: %s %s %s; on %s from %.1f to %.1f m, on %s from %.1f to %.1f m\n", id.index,
               where.c_str(), a.c_str(), meetingWord(c.meeting), b.c_str(), a.c_str(), c.aFrom, c.aTo, b.c_str(),
               c.otherFrom, c.otherTo);
}

std::string dumpText(const LaneGraph& g) {
    uint32_t lanes = 0;
    for (const Path& p : g.paths) lanes += p.kind == PathKind::Lane ? 1 : 0;
    std::string out = fmt("lane graph: %zu paths (%u lanes, %zu connectors), %zu sections, %zu junctions, "
                          "%zu crossing points, %zu crosswalks, %zu tracks, %zu regulations, %zu car spaces\n",
                          g.paths.size(), lanes, g.paths.size() - lanes, g.sections.size(), g.junctions.size(),
                          g.crossings.size(), g.crosswalks.size(), g.tracks.size(), g.regulations.size(),
                          g.carSpaces.size());
    auto table = [&](const char* title, const std::vector<std::string>& names) {
        out += title;
        out += ":";
        for (std::size_t i = 0; i < names.size(); ++i) out += fmt("%s %zu %s", i ? "," : "", i, names[i].c_str());
        out += "\n";
    };
    table("lane tags", g.laneTagNames);
    table("vehicle tags", g.vehicleTagNames);
    table("regulation kinds", g.regulationKindNames);
    table("controls", g.controlNames);
    table("road classes", g.roadClassNames);
    for (uint32_t i = 0; i < g.sections.size(); ++i) out += dumpSection(g, SectionId(i));
    for (uint32_t i = 0; i < g.paths.size(); ++i) out += dumpPath(g, PathId(i));
    for (uint32_t j = 0; j < g.junctions.size(); ++j) {
        const Junction& jn = g.junctions[j];
        out += fmt("junction %u: control %s, crossing points %u to %u, crosswalks %u to %u\n", j,
                   nameOf(g.controlNames, jn.control).c_str(), jn.crossings.first, jn.crossings.end(),
                   jn.crosswalks.first, jn.crosswalks.end());
        out += "  connectors:";
        for (uint32_t k = jn.connectors.first; k < safeEnd(jn.connectors, g.junctionConnectors.size()); ++k)
            out += fmt(" %u", g.junctionConnectors[k].index);
        out += "\n  box: " + planPointsText(g, jn.box) + "\n";
        out += "  arriving lanes:";
        for (uint32_t k = jn.arriving.first; k < safeEnd(jn.arriving, g.junctionLanes.size()); ++k)
            out += fmt(" %u", g.junctionLanes[k].index);
        // what each arriving lane faces as it reaches the junction: the regulations standing at its end (a stop sign)
        std::string facing;
        for (uint32_t k = jn.arriving.first; k < safeEnd(jn.arriving, g.junctionLanes.size()); ++k) {
            const uint32_t lane = g.junctionLanes[k].index;
            if (lane >= g.paths.size()) continue;
            const Path& lp = g.paths[lane];
            for (uint32_t r = lp.regulations.first; r < safeEnd(lp.regulations, g.regulations.size()); ++r)
                if (g.regulations[r].from >= lp.length - kJunctionEndReach)
                    facing += fmt("%s lane %u %s", facing.empty() ? "" : ",", lane,
                                  nameOf(g.regulationKindNames, g.regulations[r].kind).c_str());
        }
        if (!facing.empty()) out += "\n  at the arriving lanes' ends:" + facing;
        out += "\n  leaving lanes:";
        for (uint32_t k = jn.leaving.first; k < safeEnd(jn.leaving, g.junctionLanes.size()); ++k)
            out += fmt(" %u", g.junctionLanes[k].index);
        out += "\n";
    }
    for (uint32_t c = 0; c < g.crossings.size(); ++c) out += dumpCrossing(g, CrossingId(c));
    for (uint32_t w = 0; w < g.crosswalks.size(); ++w) {
        const Crosswalk& cw = g.crosswalks[w];
        const std::string where = cw.junction.valid() ? fmt("junction %u", cw.junction.index) : std::string("on the road");
        const std::string link = cw.walkLink == kNoIndex ? std::string("no walkers' link") : fmt("walkers' link %u", cw.walkLink);
        out += fmt("crosswalk %u, %s: %s, outline %s\n", w, where.c_str(), link.c_str(), planPointsText(g, cw.outline).c_str());
    }
    for (uint32_t t = 0; t < g.tracks.size(); ++t)
        out += fmt("track %u: rail track %u, line %s\n", t, g.tracks[t].railTrack, planPointsText(g, g.tracks[t].line).c_str());
    for (uint32_t s = 0; s < g.carSpaces.size(); ++s) {
        const CarSpace& sp = g.carSpaces[s];
        out += fmt("car space %u: %s from %.1f to %.1f m\n", s, pathName(g, sp.lane.index).c_str(), sp.from, sp.to);
    }
    return out;
}

// ---- save and load ----------------------------------------------------------------------------------------------

void putLaneGraph(bundle::BinWriter& w, const LaneGraph& g) {
    w.magic(kMagic, kVersion);
    putList(w, g.laneTagNames);
    putList(w, g.vehicleTagNames);
    putList(w, g.regulationKindNames);
    putList(w, g.controlNames);
    putList(w, g.roadClassNames);
    putList(w, g.paths);
    putList(w, g.sections);
    putList(w, g.junctions);
    putList(w, g.crossings);
    putList(w, g.crosswalks);
    putList(w, g.tracks);
    putList(w, g.regulations);
    putList(w, g.carSpaces);
    putList(w, g.points);
    putList(w, g.along);
    putList(w, g.heights);
    putList(w, g.nextPaths);
    putList(w, g.sectionLanes);
    putList(w, g.laneLines);
    putList(w, g.junctionConnectors);
    putList(w, g.junctionLanes);
    putList(w, g.pathCrossings);
    putList(w, g.planPoints);
    putList(w, g.bendPieces);
}

bool getLaneGraph(bundle::BinReader& r, LaneGraph& g, std::string* err) {
    g = LaneGraph();
    uint32_t version = 0;
    if (!r.magic(kMagic, &version)) {
        if (err) *err = "lane graph: not a lane graph section (its first four bytes are not LGRF)";
        return false;
    }
    if (version != kVersion) {
        if (err) *err = fmt("lane graph: version %u, this build reads version %u", version, kVersion);
        return false;
    }
    LaneGraph in;
    const bool ok = getList(r, in.laneTagNames, "lane tag names", err) &&
                    getList(r, in.vehicleTagNames, "vehicle tag names", err) &&
                    getList(r, in.regulationKindNames, "regulation kind names", err) &&
                    getList(r, in.controlNames, "control names", err) &&
                    getList(r, in.roadClassNames, "road class names", err) && getList(r, in.paths, "paths", err) &&
                    getList(r, in.sections, "sections", err) && getList(r, in.junctions, "junctions", err) &&
                    getList(r, in.crossings, "crossing points", err) && getList(r, in.crosswalks, "crosswalks", err) &&
                    getList(r, in.tracks, "tracks", err) && getList(r, in.regulations, "regulations", err) &&
                    getList(r, in.carSpaces, "car spaces", err) && getList(r, in.points, "curve points", err) &&
                    getList(r, in.along, "curve distances along", err) && getList(r, in.heights, "curve heights", err) &&
                    getList(r, in.nextPaths, "next paths", err) && getList(r, in.sectionLanes, "section lanes", err) &&
                    getList(r, in.laneLines, "lane lines", err) &&
                    getList(r, in.junctionConnectors, "junction connectors", err) &&
                    getList(r, in.junctionLanes, "junction lanes", err) &&
                    getList(r, in.pathCrossings, "path crossing points", err) &&
                    getList(r, in.planPoints, "plan points", err) && getList(r, in.bendPieces, "bend pieces", err);
    if (!ok) return false;
    if (r.remaining() != 0) {
        if (err) *err = fmt("lane graph: %zu stray bytes after the last list", r.remaining());
        return false;
    }
    const std::vector<std::string> problems = validate(in);
    if (!problems.empty()) {
        if (err) {
            *err = fmt("lane graph refused, %zu problems:", problems.size());
            for (std::size_t i = 0; i < problems.size() && i < kProblemsInMessage; ++i) *err += "\n  " + problems[i];
            if (problems.size() > kProblemsInMessage) *err += fmt("\n  and %zu more", problems.size() - kProblemsInMessage);
        }
        return false;
    }
    g = std::move(in);
    return true;
}

bool writeLaneGraph(bundle::BundleWriter& out, const std::string& section, const LaneGraph& g) {
    bundle::BinWriter w;
    putLaneGraph(w, g);
    return out.add(section, w.bytes);
}

bool readLaneGraph(const bundle::Bundle& in, const std::string& section, LaneGraph& g, std::string* err) {
    const bundle::Bundle::View v = in.section(section);
    if (v.empty()) {
        g = LaneGraph();
        if (err) *err = fmt("lane graph: the bundle has no section %s", section.c_str());
        return false;
    }
    bundle::BinReader r(v.data, v.size);
    return getLaneGraph(r, g, err);
}

}  // namespace engine
