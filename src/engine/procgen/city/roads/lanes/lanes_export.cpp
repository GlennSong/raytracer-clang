#include "engine/procgen/city/roads/lanes/lanes_export.h"
#include "engine/procgen/city/roads/lanes/vertical_profile.h"

#include "engine/glb_export.h"
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iomanip>

namespace engine {
namespace roads::lanes {

bool writeGlb(const std::vector<NamedMesh>& meshes, const std::string& path, std::string* error) {
    // The engine's writer (glb_export.h) does the glTF; it also keeps tangents and a varying vertex colour,
    // which the original lanelab-only writer dropped.
    std::vector<engine::GlbEntry> entries;
    for (const NamedMesh& nm : meshes) { engine::GlbEntry e; e.name = nm.name; e.mesh = &nm.mesh; e.color = nm.color; entries.push_back(std::move(e)); }
    return engine::writeGlb(entries, path, nlohmann::json::object(), error);
}

bool writePlanSvg(const Result& r, const std::string& path) {
    const RoadLabGraph& g = r.graph; std::array<double, 4> b = g.bounds(); double W = b[1] - b[0], Hh = b[3] - b[2], scale = 1600.0 / std::max(W, Hh);
    std::ofstream f(path); if (!f) return false;
    auto X = [&](double x) { return (x - b[0]) * scale; }; auto Y = [&](double y) { return (b[3] - y) * scale; };
    f << "<svg xmlns='http://www.w3.org/2000/svg' width='" << W * scale << "' height='" << Hh * scale << "' viewBox='0 0 " << W * scale << " " << Hh * scale << "'>\n";
    f << "<rect width='100%' height='100%' fill='#8fa36b'/>\n";
    auto poly = [&](const PolySet& set, const char* fill, const char* stroke, double sw) {
        for (const Polygon2& p : set) {
            f << "<path fill='" << fill << "' fill-rule='evenodd' stroke='" << stroke << "' stroke-width='" << sw << "' d='";
            auto ring = [&](const Ring& rg) { for (size_t i = 0; i < rg.size(); ++i) f << (i ? " L" : "M") << X(rg[i].x) << " " << Y(rg[i].y); f << " Z"; };
            ring(p.outer); for (const Ring& h : p.holes) ring(h); f << "'/>\n";
        }
    };
    poly(r.pavement.sidewalk, "#d9d4c7", "#6d6a63", 0.5); poly(r.pavement.shoulder, "#5f5d5a", "none", 0); poly(r.pavement.median, "#7f9a5c", "none", 0);
    std::map<std::string, const char*> fc = {{"freeway", "#2b2b30"}, {"ramp", "#38383e"}, {"arterial", "#3a3a3c"}, {"local", "#454547"}};
    for (size_t li = 0; li < r.lanes.lanes.size(); ++li) { auto it = fc.find(r.lanes.lanes[li].cls); poly(r.pavement.footprints[li], it == fc.end() ? "#3a3a3c" : it->second, "none", 0); }
    poly(r.pavement.surface, "none", "#111", 0.6);
    for (const EdgeSpec* e : g.paved()) {
        std::string d; bool open = false;
        for (size_t i = 0; i < e->xy.size(); ++i) {
            bool onb = e->z[i] - e->t[i] > g.rules.bridgeH;
            if (onb) { d += (open ? " L" : "M") + std::to_string(X(e->xy[i].x)) + " " + std::to_string(Y(e->xy[i].y)); open = true; } else open = false;
        }
        if (!d.empty()) f << "<path fill='none' stroke='#f2d16b' stroke-width='" << 2.4 * scale << "' stroke-opacity='0.8' d='" << d << "'/>\n";
    }
    for (const Pier& p : r.piers) f << "<rect x='" << X(p.xy.x) - 1.3 * scale << "' y='" << Y(p.xy.y) - 1.3 * scale << "' width='" << 2.6 * scale << "' height='" << 2.6 * scale << "' fill='#f2d16b' stroke='#000' stroke-width='0.3'/>\n";
    f << "</svg>\n"; return true;
}

bool writeStatsJson(const Result& r, const std::string& path) {
    nlohmann::json j; j["name"] = r.graph.name; j["lanes"] = r.lanes.lanes.size(); j["seconds"] = r.seconds; j["timings"] = r.timings;
    for (const EdgeSpec& e : r.graph.edges) {
        if (e.laneCount() == 0) continue; nlohmann::json ej; ej["class"] = e.cls; ej["length"] = e.length(); ej["max_grade"] = maxGrade(e); ej["bridge_len"] = r.bridgeLen.at(e.id);
        if (e.ramp.valid) ej["ramp"] = {{"depart_s", e.ramp.departS}, {"gore_s", e.ramp.touchS}, {"climb", e.ramp.climb}, {"free_len", e.ramp.freeLen}, {"required_len", e.ramp.requiredLen}, {"ok", e.ramp.ok}, {"anchors", e.anchorLanes}};
        j["edges"][e.id] = ej;
    }
    for (const Surface& s : r.pavement.decks) j["decks"].push_back({{"area", s.area}, {"verts", s.verts.size()}, {"tris", s.tris.size()}, {"non_manifold", s.nonManifold}, {"cracks", s.cracks}, {"boundary_odd", s.boundaryOdd}});
    j["layers"] = {{"surface", setArea(r.pavement.surface)}, {"shoulder", setArea(r.pavement.shoulder)}, {"sidewalk", setArea(r.pavement.sidewalk)}, {"median", setArea(r.pavement.median)}};
    j["islands"] = r.pavement.islands; j["enclosed_blocks"] = r.pavement.enclosedBlocks; j["node_mismatch_after"] = r.nodeMismatchAfter;
    if (r.hasTerrain) j["terrain"] = {{"cut_m3", r.conform.cutM3}, {"fill_m3", r.conform.fillM3}, {"above", r.conform.above}, {"samples", r.conform.samples}, {"max_excess", r.conform.maxExcess}};
    for (const Check& c : invariants(r)) j["invariants"].push_back({{"name", c.name}, {"ok", c.ok}, {"detail", c.detail}});
    // cameras: one per anchored gore, 45 m back along the ramp, 6 m up, looking at the gore (engine world x, height, z)
    j["cameras"] = nlohmann::json::array();
    for (const EdgeSpec& e : r.graph.edges) {
        for (int k = 0; k < 2; ++k) {
            int gi = e.anchorIdx ? e.anchorIdx[k] : -1; if (gi < 0 || !e.ramp.valid) continue;
            size_t g = static_cast<size_t>(gi); double back = e.s[g] - 45.0; size_t bi = g; while (bi > 0 && e.s[bi] > back) --bi;
            Vec2 eye = e.xy[bi], tgt = e.xy[g]; double ze = e.z[bi] + 6.0, zt = e.z[g] + 1.0;
            j["cameras"].push_back({{"name", e.id + (k == 0 ? "_from_gore" : "_to_gore")}, {"eye", {eye.x, ze, eye.y}}, {"target", {tgt.x, zt, tgt.y}}});
        }
    }
    std::ofstream f(path); if (!f) return false; f << j.dump(1); return true;
}

namespace {
// The lane graph plan's look, in plan metres (1 unit = 1 m) unless named otherwise.
constexpr double kGraphSvgPixelsPerMetre = 6.0;   // how large the page opens; zoom for more
constexpr double kGraphSvgMargin = 20.0;
constexpr double kGraphSvgLaneStroke = 0.35;
constexpr double kGraphSvgArrowLength = 1.6, kGraphSvgArrowHalfWidth = 0.7;
constexpr double kGraphSvgLabelSize = 1.1, kGraphSvgJunctionLabelSize = 2.0;
constexpr double kGraphSvgLabelOffset = 0.6;      // a lane's id sits this far to the side of its centre line
constexpr double kGraphSvgTickStroke = 0.18, kGraphSvgOutlineStroke = 0.3, kGraphSvgEndStroke = 0.25;
constexpr double kGraphSvgEndMark = 0.7;          // the arriving circle's radius, the leaving square's half side
// Section colours, far apart in hue; a section takes every kGraphSvgColourStride-th one so neighbours differ.
const char* const kGraphSvgColours[] = {"#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e",
                                        "#17becf", "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22"};
constexpr std::size_t kGraphSvgColourCount = sizeof(kGraphSvgColours) / sizeof(kGraphSvgColours[0]);
constexpr std::size_t kGraphSvgColourStride = 3;
// Connectors: thinner and dashed, so they read apart from the lanes, in a colour per turn (indexed by Turn).
constexpr double kGraphSvgConnectorStroke = 0.22;
constexpr double kGraphSvgConnectorArrowLength = 0.9, kGraphSvgConnectorArrowHalfWidth = 0.4;
const char* const kGraphSvgConnectorDash = "0.8 0.35";
const char* const kGraphSvgTurnColours[] = {"#000000", "#1565c0", "#2e7d32", "#e65100", "#6a1b9a"};
const char* const kGraphSvgTurnNames[] = {"", "straight on", "near turn", "far turn", "U-turn"};
// Refused: junction outlines and crosses in this red; a cross's half size; the key's text size and line spacing.
const char* const kGraphSvgRefused = "#d50000";
constexpr double kGraphSvgRefusedCross = 1.2, kGraphSvgRefusedStroke = 0.35;
constexpr double kGraphSvgKeySize = 3.0, kGraphSvgKeyLine = 4.0;
constexpr double kGraphSvgCloseUpKeyScale = 0.3;   // the key on a close-up of one junction, against the whole plan's
// Crossing points, by Meeting (cross, join, split): the colour; the stretch's width and opacity; the mark's size.
const char* const kGraphSvgMeetingColours[] = {"#ad1457", "#00838f", "#f9a825"};
const char* const kGraphSvgMeetingNames[] = {"cross", "join", "split"};
constexpr double kGraphSvgStretchStroke = 1.1, kGraphSvgStretchOpacity = 0.35, kGraphSvgMeetingMark = 0.45;
}  // namespace

bool writeLaneGraphSvg(const LaneGraph& g, const std::string& path, const LaneGraphSvgMarks& marks) {
    double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
    for (const std::vector<Vec2>* set : {&g.points, &g.planPoints})
        for (const Vec2& p : *set) { x0 = std::min(x0, p.x); y0 = std::min(y0, p.y); x1 = std::max(x1, p.x); y1 = std::max(y1, p.y); }
    if (x0 > x1) { x0 = y0 = 0; x1 = y1 = 1; }
    if (marks.cropped) { x0 = marks.cropLo.x; y0 = marks.cropLo.y; x1 = marks.cropHi.x; y1 = marks.cropHi.y; }
    // Whether any of these points falls in the picture (always, uncropped): what lies wholly outside a window is not drawn.
    auto shown = [&](const Vec2* pts, uint32_t count) {
        if (!marks.cropped) return true;
        for (uint32_t k = 0; k < count; ++k)
            if (pts[k].x >= x0 - kGraphSvgMargin && pts[k].x <= x1 + kGraphSvgMargin && pts[k].y >= y0 - kGraphSvgMargin &&
                pts[k].y <= y1 + kGraphSvgMargin)
                return true;
        return false;
    };
    auto pathShown = [&](uint32_t p) {
        const Range& c = g.paths[p].curve;
        return c.end() <= g.points.size() && shown(g.points.data() + c.first, c.count);
    };
    std::ofstream f(path); if (!f) return false;
    f << std::fixed << std::setprecision(2);
    const double W = x1 - x0 + 2 * kGraphSvgMargin, Hh = y1 - y0 + 2 * kGraphSvgMargin;
    auto X = [&](double x) { return x - x0 + kGraphSvgMargin; };
    auto Y = [&](double y) { return y1 - y + kGraphSvgMargin; };   // plan y up, as the builder's plan SVG
    f << "<svg xmlns='http://www.w3.org/2000/svg' width='" << W * kGraphSvgPixelsPerMetre << "' height='" << Hh * kGraphSvgPixelsPerMetre
      << "' viewBox='0 0 " << W << " " << Hh << "' font-family='sans-serif'>\n";
    f << "<!-- plan point (x, y) is drawn at (x - " << x0 - kGraphSvgMargin << ", " << y1 + kGraphSvgMargin << " - y) -->\n";
    f << "<rect width='100%' height='100%' fill='#fbfaf7'/>\n";
    // junction outlines, with their ids (refused ones in red)
    f << "<g id='junctions' fill='#f3d9a4' fill-opacity='0.55' stroke='#b07a1e' stroke-width='" << kGraphSvgOutlineStroke << "'>\n";
    for (uint32_t j = 0; j < g.junctions.size(); ++j) {
        const Range& b = g.junctions[j].box;
        if (b.end() > g.planPoints.size() || b.count == 0 || !shown(g.planPoints.data() + b.first, b.count)) continue;
        const bool refused = std::find(marks.refusedJunctions.begin(), marks.refusedJunctions.end(), j) != marks.refusedJunctions.end();
        f << "<polygon" << (refused ? std::string(" fill='") + kGraphSvgRefused + "' stroke='" + kGraphSvgRefused + "'" : std::string()) << " points='";
        Vec2 c;
        for (uint32_t k = b.first; k < b.end(); ++k) { f << X(g.planPoints[k].x) << "," << Y(g.planPoints[k].y) << " "; c += g.planPoints[k]; }
        c = c / static_cast<double>(b.count);
        f << "'/>\n<text x='" << X(c.x) << "' y='" << Y(c.y) << "' font-size='" << kGraphSvgJunctionLabelSize
          << "' fill='#7a4f0a' stroke='none' text-anchor='middle'>J" << j << "</text>\n";
    }
    f << "</g>\n";
    // lane paths: centre line, arrowhead, id
    f << "<g id='lanes' fill='none' stroke-width='" << kGraphSvgLaneStroke << "'>\n";
    for (uint32_t p = 0; p < g.paths.size(); ++p) {
        const Path& path = g.paths[p];
        if (path.curve.end() > g.points.size() || path.kind != PathKind::Lane || !pathShown(p)) continue;
        const char* colour = kGraphSvgColours[(static_cast<std::size_t>(path.section.index) * kGraphSvgColourStride) % kGraphSvgColourCount];
        f << "<polyline stroke='" << colour << "' points='";
        for (uint32_t k = path.curve.first; k < path.curve.end(); ++k) f << X(g.points[k].x) << "," << Y(g.points[k].y) << " ";
        f << "'/>\n";
        const Vec2 tip = g.pointAt(PathId(p), path.length), d = g.directionAt(PathId(p), path.length), n = perp(d);
        const Vec2 b0 = tip - d * kGraphSvgArrowLength + n * kGraphSvgArrowHalfWidth, b1 = tip - d * kGraphSvgArrowLength - n * kGraphSvgArrowHalfWidth;
        f << "<polygon fill='" << colour << "' stroke='none' points='" << X(tip.x) << "," << Y(tip.y) << " " << X(b0.x) << "," << Y(b0.y)
          << " " << X(b1.x) << "," << Y(b1.y) << "'/>\n";
        const Vec2 mid = g.pointAt(PathId(p), path.length / 2), label = mid + perp(g.directionAt(PathId(p), path.length / 2)) * kGraphSvgLabelOffset;
        f << "<text x='" << X(label.x) << "' y='" << Y(label.y) << "' font-size='" << kGraphSvgLabelSize << "' fill='" << colour
          << "' stroke='none' text-anchor='middle' dominant-baseline='middle'>" << p << "</text>\n";
    }
    f << "</g>\n";
    // where each section starts: a tick across its lanes
    f << "<g id='section-starts' stroke='#333' stroke-width='" << kGraphSvgTickStroke << "'>\n";
    for (const Section& s : g.sections) {
        if (s.lanes.count == 0 || s.lanes.end() > g.sectionLanes.size()) continue;
        const PathId first = g.sectionLanes[s.lanes.first], last = g.sectionLanes[s.lanes.end() - 1];
        const Vec2 n = perp(g.directionAt(first, 0));
        const Vec2 a = g.pointAt(first, 0) + n * (g.paths[first.index].width / 2), b = g.pointAt(last, 0) - n * (g.paths[last.index].width / 2);
        // (the offside lane's far edge to the kerb lane's far edge, whichever side the normal points)
        const Vec2 a2 = g.pointAt(first, 0) - n * (g.paths[first.index].width / 2), b2 = g.pointAt(last, 0) + n * (g.paths[last.index].width / 2);
        const bool wide = distance(a, b) >= distance(a2, b2);
        const Vec2 from = wide ? a : a2, to = wide ? b : b2;
        f << "<line x1='" << X(from.x) << "' y1='" << Y(from.y) << "' x2='" << X(to.x) << "' y2='" << Y(to.y) << "'/>\n";
    }
    f << "</g>\n";
    // arriving ends (circles) and leaving starts (squares)
    f << "<g id='junction-ends' fill='none' stroke-width='" << kGraphSvgEndStroke << "'>\n";
    for (const Junction& jn : g.junctions) {
        for (uint32_t k = jn.arriving.first; k < jn.arriving.end() && k < g.junctionLanes.size(); ++k) {
            const PathId p = g.junctionLanes[k];
            if (p.index >= g.paths.size()) continue;
            const Vec2 q = g.pointAt(p, g.paths[p.index].length);
            f << "<circle cx='" << X(q.x) << "' cy='" << Y(q.y) << "' r='" << kGraphSvgEndMark << "' stroke='#c62828'/>\n";
        }
        for (uint32_t k = jn.leaving.first; k < jn.leaving.end() && k < g.junctionLanes.size(); ++k) {
            const PathId p = g.junctionLanes[k];
            if (p.index >= g.paths.size()) continue;
            const Vec2 q = g.pointAt(p, 0);
            f << "<rect x='" << X(q.x) - kGraphSvgEndMark << "' y='" << Y(q.y) - kGraphSvgEndMark << "' width='" << 2 * kGraphSvgEndMark
              << "' height='" << 2 * kGraphSvgEndMark << "' stroke='#2e7d32'/>\n";
        }
    }
    f << "</g>\n";
    // crossing points: each stretch wide and pale on its path, under the connectors
    // (the points of a path from `from` to `to` metres along it: its ends there, and its own points between)
    auto stretchPoints = [&](PathId p, double from, double to) {
        std::vector<Vec2> pts{g.pointAt(p, from)};
        const Range& c = g.paths[p.index].curve;
        for (uint32_t k = c.first; k < c.end() && k < g.along.size(); ++k)
            if (g.along[k] > from && g.along[k] < to) pts.push_back(g.points[k]);
        pts.push_back(g.pointAt(p, to));
        return pts;
    };
    auto meetingColour = [&](const CrossingPoint& x) {
        const std::size_t m = static_cast<std::size_t>(x.meeting);
        return m < kMeetingCount ? kGraphSvgMeetingColours[m] : kGraphSvgRefused;
    };
    std::vector<char> crossingShown(g.crossings.size(), 0);
    f << "<g id='crossing-stretches' fill='none' stroke-width='" << kGraphSvgStretchStroke << "' stroke-opacity='"
      << kGraphSvgStretchOpacity << "' stroke-linecap='round'>\n";
    for (uint32_t c = 0; c < g.crossings.size(); ++c) {
        const CrossingPoint& x = g.crossings[c];
        if (x.a.index >= g.paths.size() || !pathShown(x.a.index)) continue;
        crossingShown[c] = 1;
        std::vector<std::vector<Vec2>> lines{stretchPoints(x.a, x.aFrom, x.aTo)};
        if (x.otherPath().valid() && x.other < g.paths.size()) lines.push_back(stretchPoints(x.otherPath(), x.otherFrom, x.otherTo));
        for (const std::vector<Vec2>& line : lines) {
            f << "<polyline stroke='" << meetingColour(x) << "' points='";
            for (const Vec2& q : line) f << X(q.x) << "," << Y(q.y) << " ";
            f << "'/>\n";
        }
    }
    f << "</g>\n";
    // connectors, coloured by turn, each with a small arrowhead at its end
    f << "<g id='connectors' fill='none' stroke-width='" << kGraphSvgConnectorStroke << "' stroke-dasharray='" << kGraphSvgConnectorDash << "'>\n";
    for (uint32_t p = 0; p < g.paths.size(); ++p) {
        const Path& path = g.paths[p];
        if (path.curve.end() > g.points.size() || path.kind != PathKind::Connector || !pathShown(p)) continue;
        const std::size_t turn = static_cast<std::size_t>(path.turn) < kTurnCount ? static_cast<std::size_t>(path.turn) : 0;
        const char* colour = kGraphSvgTurnColours[turn];
        f << "<polyline stroke='" << colour << "' points='";
        for (uint32_t k = path.curve.first; k < path.curve.end(); ++k) f << X(g.points[k].x) << "," << Y(g.points[k].y) << " ";
        f << "'/>\n";
        const Vec2 tip = g.pointAt(PathId(p), path.length), d = g.directionAt(PathId(p), path.length), n = perp(d);
        const Vec2 b0 = tip - d * kGraphSvgConnectorArrowLength + n * kGraphSvgConnectorArrowHalfWidth;
        const Vec2 b1 = tip - d * kGraphSvgConnectorArrowLength - n * kGraphSvgConnectorArrowHalfWidth;
        f << "<polygon fill='" << colour << "' stroke='none' points='" << X(tip.x) << "," << Y(tip.y) << " " << X(b0.x) << "," << Y(b0.y)
          << " " << X(b1.x) << "," << Y(b1.y) << "'/>\n";
    }
    f << "</g>\n";
    // refused turns: a red cross where each is
    f << "<g id='refused-turns' stroke='" << kGraphSvgRefused << "' stroke-width='" << kGraphSvgRefusedStroke << "'>\n";
    for (const Vec2& q : marks.refusedTurns) {
        const double c = kGraphSvgRefusedCross;
        f << "<path d='M" << X(q.x) - c << " " << Y(q.y) - c << " L" << X(q.x) + c << " " << Y(q.y) + c << " M" << X(q.x) - c << " "
          << Y(q.y) + c << " L" << X(q.x) + c << " " << Y(q.y) - c << "'/>\n";
    }
    f << "</g>\n";
    // crossing points: a mark at the middle of the stretch on the first path, with its id (a circle: filed under a
    // junction; a square: across junctions, or out on a road)
    f << "<g id='crossing-points' stroke='none'>\n";
    for (uint32_t c = 0; c < g.crossings.size(); ++c) {
        if (!crossingShown[c]) continue;
        const CrossingPoint& x = g.crossings[c];
        const Vec2 q = g.pointAt(x.a, (x.aFrom + x.aTo) / 2);
        const double r = kGraphSvgMeetingMark;
        if (x.junction.valid())
            f << "<circle cx='" << X(q.x) << "' cy='" << Y(q.y) << "' r='" << r << "' fill='" << meetingColour(x) << "'>";
        else
            f << "<rect x='" << X(q.x) - r << "' y='" << Y(q.y) - r << "' width='" << 2 * r << "' height='" << 2 * r << "' fill='"
              << meetingColour(x) << "'>";
        f << "<title>crossing point " << c << "</title>" << (x.junction.valid() ? "</circle>\n" : "</rect>\n");
    }
    f << "</g>\n";
    // the key, in the corner at the picture's origin (smaller on a close-up, so it does not cover the junction)
    const double keyScale = marks.cropped ? kGraphSvgCloseUpKeyScale : 1.0;
    const double keyLine = kGraphSvgKeyLine * keyScale;
    f << "<g id='key' font-size='" << kGraphSvgKeySize * keyScale << "'>\n";
    double keyY = keyLine;
    for (std::size_t t = 1; t < kTurnCount; ++t, keyY += keyLine)
        f << "<text x='" << keyLine << "' y='" << keyY << "' fill='" << kGraphSvgTurnColours[t] << "'>connector: "
          << kGraphSvgTurnNames[t] << "</text>\n";
    for (std::size_t m = 0; m < kMeetingCount; ++m, keyY += keyLine)
        f << "<text x='" << keyLine << "' y='" << keyY << "' fill='" << kGraphSvgMeetingColours[m]
          << "'>crossing point: " << kGraphSvgMeetingNames[m] << " (circle: in a junction; square: across junctions)</text>\n";
    f << "<text x='" << keyLine << "' y='" << keyY << "' fill='" << kGraphSvgRefused << "'>refused: junction (red), turn (cross)</text>\n";
    f << "</g>\n</svg>\n";
    return static_cast<bool>(f);
}

}  // namespace roads::lanes
}  // namespace engine
