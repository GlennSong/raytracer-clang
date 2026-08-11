#include "odr.h"

#include <cstdio>
#include <fstream>
#include <sstream>

namespace roadlab {

const char* odrLaneType(StripKind kind) {
    switch (kind) {
        case StripKind::Travel:
        case StripKind::Turn:
            return "driving";
        case StripKind::Bus:
            return "bus";
        case StripKind::Bike:
            return "biking";
        case StripKind::Parking:
            return "parking";
        case StripKind::Shoulder:
            return "shoulder";
        case StripKind::Median:
        case StripKind::Apron:
            return "median";
        case StripKind::Curb:
            return "curb";
        case StripKind::Gutter:
            return "border";
        case StripKind::Verge:
            return "restricted";
        case StripKind::Sidewalk:
            return "sidewalk";
        case StripKind::Barrier:
            return "border";
        case StripKind::Slope:
            return "none";
    }
    return "none";
}

const char* odrRoadMarkType(MarkStyle style) {
    switch (style) {
        case MarkStyle::None: return "none";
        case MarkStyle::Solid: return "solid";
        case MarkStyle::Dashed: return "broken";
        case MarkStyle::WideDashed: return "broken";
        case MarkStyle::Double: return "solid solid";
        case MarkStyle::SolidDashed: return "solid broken";
        case MarkStyle::DashedSolid: return "broken solid";
        case MarkStyle::Botts: return "botts dots";
        // OpenDRIVE has no hatching or chevron roadMark; those are areas, and a
        // faithful export would emit them as <object> surfaces. Declaring them
        // "none" is honest about the loss rather than pretending they are lines.
        case MarkStyle::Hatch:
        case MarkStyle::Chevron:
            return "none";
    }
    return "none";
}

const char* odrRoadMarkColor(PaintColor color) {
    switch (color) {
        case PaintColor::White: return "standard";
        case PaintColor::Yellow: return "yellow";
        case PaintColor::Red: return "red";
        case PaintColor::Blue: return "blue";
        case PaintColor::Green: return "green";
    }
    return "standard";
}

namespace {

struct Writer {
    std::ostringstream out;
    int indent = 0;
    int precision = 10;

    void line(const std::string& text) {
        for (int i = 0; i < indent; ++i) out << "    ";
        out << text << "\n";
    }
    void open(const std::string& text) {
        line(text);
        ++indent;
    }
    void close(const std::string& text) {
        --indent;
        line(text);
    }
    std::string num(double v) const {
        if (std::fabs(v) < 1e-12) v = 0.0;   // no "-0"
        char buf[64];
        std::snprintf(buf, sizeof buf, "%.*g", precision, v);
        return buf;
    }
    std::string attr(const char* key, double v) const {
        return std::string(key) + "=\"" + num(v) + "\"";
    }
    std::string attr(const char* key, int v) const {
        return std::string(key) + "=\"" + std::to_string(v) + "\"";
    }
    std::string attr(const char* key, const std::string& v) const {
        std::string esc;
        for (char c : v) {
            switch (c) {
                case '&': esc += "&amp;"; break;
                case '<': esc += "&lt;"; break;
                case '>': esc += "&gt;"; break;
                case '"': esc += "&quot;"; break;
                default: esc += c;
            }
        }
        return std::string(key) + "=\"" + esc + "\"";
    }
    std::string poly(const Poly3& p) const {
        return attr("a", p.a) + " " + attr("b", p.b) + " " + attr("c", p.c) + " " +
               attr("d", p.d);
    }
};

// Emit one piecewise-cubic profile clipped to [begin, end] and re-based so the
// exported road starts at s = 0. This is where Poly3::shifted earns its keep:
// a piece that starts before the window has to be measured from the window.
void writeProfile(Writer& w, const Profile1D& profile, double begin, double end,
                  const char* element, const char* extra = "") {
    if (profile.pieces.empty()) return;
    for (size_t i = 0; i < profile.pieces.size(); ++i) {
        double pieceStart = profile.pieces[i].s0;
        double pieceEnd = (i + 1 < profile.pieces.size()) ? profile.pieces[i + 1].s0 : 1e300;
        double a = std::max(pieceStart, begin);
        double b = std::min(pieceEnd, end);
        if (b - a < 1e-6) continue;
        Poly3 p = profile.pieces[i].poly.shifted(a - pieceStart);
        w.line(std::string("<") + element + " " + w.attr("s", a - begin) + " " + w.poly(p) +
               extra + "/>");
    }
}

void writeLink(Writer& w, const Network& net, const Road& r) {
    auto describe = [&](const RoadLink& link, const char* tag) {
        if (link.type == LinkType::None || link.id < 0) return;
        if (link.type == LinkType::Junction) {
            w.line(std::string("<") + tag + " " + w.attr("elementType", std::string("junction")) +
                   " " + w.attr("elementId", link.id) + "/>");
        } else {
            w.line(std::string("<") + tag + " " + w.attr("elementType", std::string("road")) +
                   " " + w.attr("elementId", link.id) + " " +
                   w.attr("contactPoint", std::string(link.toStart ? "start" : "end")) + "/>");
        }
    };
    // A ramp's merge is an ExtraLaneLink rather than a road link; at road level
    // OpenDRIVE still wants to see the successor, so surface it here.
    RoadLink pred = r.pred, succ = r.succ;
    for (const ExtraLaneLink& el : net.extraLinks) {
        if (el.fromRoad == r.id && succ.type == LinkType::None) {
            succ.type = LinkType::Road;
            succ.id = el.toRoad;
            succ.toStart = el.toAtStart;
        }
        if (el.toRoad == r.id && pred.type == LinkType::None && el.toAtStart) {
            pred.type = LinkType::Road;
            pred.id = el.fromRoad;
            pred.toStart = !el.fromAtEnd;
        }
    }
    if (pred.type == LinkType::None && succ.type == LinkType::None) return;
    w.open("<link>");
    describe(pred, "predecessor");
    describe(succ, "successor");
    w.close("</link>");
}

void writePlanView(Writer& w, const Road& r) {
    double begin = r.begin(), end = r.end();
    w.open("<planView>");
    bool any = false;
    for (const GeomPrim& g : r.spine.prims()) {
        double a = std::max(g.s0, begin);
        double b = std::min(g.s0 + g.length, end);
        if (b - a < 1e-6) continue;
        // Clipping a primitive means re-deriving its start pose, and for a
        // clothoid also its start and end curvature — the parameters are only
        // meaningful relative to where the piece now begins.
        Frame f = r.spine.frameAt(a);
        w.open("<geometry " + w.attr("s", a - begin) + " " + w.attr("x", f.planPos.x) + " " +
               w.attr("y", f.planPos.y) + " " + w.attr("hdg", f.heading) + " " +
               w.attr("length", b - a) + ">");
        switch (g.kind) {
            case GeomKind::Line:
                w.line("<line/>");
                break;
            case GeomKind::Arc:
                w.line("<arc " + w.attr("curvature", g.curv0) + "/>");
                break;
            case GeomKind::Spiral: {
                double c = (g.curv1 - g.curv0) / g.length;
                w.line("<spiral " + w.attr("curvStart", g.curv0 + c * (a - g.s0)) + " " +
                       w.attr("curvEnd", g.curv0 + c * (b - g.s0)) + "/>");
                break;
            }
        }
        w.close("</geometry>");
        any = true;
    }
    if (!any) {
        // A road whose whole window was trimmed away still needs one geometry,
        // or the file is invalid.
        Frame f = r.spine.frameAt(begin);
        w.open("<geometry " + w.attr("s", 0.0) + " " + w.attr("x", f.planPos.x) + " " +
               w.attr("y", f.planPos.y) + " " + w.attr("hdg", f.heading) + " " +
               w.attr("length", std::max(0.01, end - begin)) + ">");
        w.line("<line/>");
        w.close("</geometry>");
    }
    w.close("</planView>");
}

void writeLane(Writer& w, const LaneSection& sec, const Strip& st, double clipS) {
    w.open("<lane " + w.attr("id", st.id) + " " +
           w.attr("type", std::string(odrLaneType(st.kind))) + " " +
           w.attr("level", std::string("false")) + ">");
    if (st.predecessor != kNoLane || st.successor != kNoLane) {
        w.open("<link>");
        if (st.predecessor != kNoLane)
            w.line("<predecessor " + w.attr("id", st.predecessor) + "/>");
        if (st.successor != kNoLane) w.line("<successor " + w.attr("id", st.successor) + "/>");
        w.close("</link>");
    }
    // Widths are polynomials in the offset from the lane section's start, so a
    // section clipped by a junction trim needs its widths re-based too.
    Poly3 width = st.width.shifted(clipS - sec.s0);
    w.line("<width " + w.attr("sOffset", 0.0) + " " + w.poly(width) + "/>");
    const Marking& m = st.outerMark;
    w.line("<roadMark " + w.attr("sOffset", 0.0) + " " +
           w.attr("type", std::string(odrRoadMarkType(m.style))) + " " +
           w.attr("weight", std::string(m.width > 0.18f ? "bold" : "standard")) + " " +
           w.attr("color", std::string(odrRoadMarkColor(m.color))) + " " +
           w.attr("width", double(m.width)) + " " +
           w.attr("laneChange",
                  std::string(m.crossableFromLeft() && m.crossableFromRight()
                                  ? "both"
                                  : (m.crossableFromLeft() ? "decrease"
                                                           : (m.crossableFromRight() ? "increase"
                                                                                     : "none")))) +
           "/>");
    if (st.speedLimit > 0.5f) {
        w.line("<speed " + w.attr("sOffset", 0.0) + " " + w.attr("max", double(st.speedLimit)) +
               " " + w.attr("unit", std::string("km/h")) + "/>");
    }
    w.close("</lane>");
}

void writeLanes(Writer& w, const Road& r) {
    double begin = r.begin(), end = r.end();
    w.open("<lanes>");
    writeProfile(w, r.xs.laneOffset, begin, end, "laneOffset");
    for (size_t i = 0; i < r.xs.sections.size(); ++i) {
        const LaneSection& sec = r.xs.sections[i];
        double a = std::max(sec.s0, begin);
        double b = std::min(sec.s0 + sec.length, end);
        if (b - a < 1e-6) continue;
        w.open("<laneSection " + w.attr("s", a - begin) + ">");
        if (!sec.left.empty()) {
            w.open("<left>");
            // OpenDRIVE wants left lanes in descending id order, outermost first.
            for (size_t k = sec.left.size(); k-- > 0;)
                writeLane(w, sec, sec.left[k], a);
            w.close("</left>");
        }
        w.open("<center>");
        {
            w.open("<lane " + w.attr("id", 0) + " " + w.attr("type", std::string("none")) + " " +
                   w.attr("level", std::string("false")) + ">");
            const Marking& m = sec.centerMark;
            w.line("<roadMark " + w.attr("sOffset", 0.0) + " " +
                   w.attr("type", std::string(odrRoadMarkType(m.style))) + " " +
                   w.attr("weight", std::string("standard")) + " " +
                   w.attr("color", std::string(odrRoadMarkColor(m.color))) + " " +
                   w.attr("width", double(m.width)) + "/>");
            w.close("</lane>");
        }
        w.close("</center>");
        if (!sec.right.empty()) {
            w.open("<right>");
            for (size_t k = 0; k < sec.right.size(); ++k)
                writeLane(w, sec, sec.right[k], a);
            w.close("</right>");
        }
        w.close("</laneSection>");
    }
    w.close("</lanes>");
}

}  // namespace

std::string openDriveString(const Network& net, const OdrOptions& opt) {
    Writer w;
    w.precision = opt.precision;

    Vec2 lo{1e300, 1e300}, hi{-1e300, -1e300};
    double minZ = 1e300, maxZ = -1e300;
    for (const Road& r : net.roads()) {
        Vec2 a, b;
        r.spine.planBounds(a, b);
        lo.x = std::min(lo.x, a.x);
        lo.y = std::min(lo.y, a.y);
        hi.x = std::max(hi.x, b.x);
        hi.y = std::max(hi.y, b.y);
        for (double s = r.begin(); s <= r.end(); s += 10.0) {
            double y = r.surfacePoint(s, 0).y;
            minZ = std::min(minZ, y);
            maxZ = std::max(maxZ, y);
        }
    }
    if (net.roads().empty()) {
        lo = {0, 0};
        hi = {0, 0};
        minZ = maxZ = 0;
    }

    w.line("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
    w.open("<OpenDRIVE>");
    {
        std::string header = "<header " + w.attr("revMajor", 1) + " " + w.attr("revMinor", 7) +
                             " " + w.attr("name", opt.name) + " " +
                             w.attr("version", std::string("1.00")) + " ";
        if (!opt.date.empty()) header += w.attr("date", opt.date) + " ";
        header += w.attr("north", hi.y) + " " + w.attr("south", lo.y) + " " +
                  w.attr("east", hi.x) + " " + w.attr("west", lo.x) + " " +
                  w.attr("vendor", std::string("roadlab"));
        w.open(header + ">");
        w.line("<!-- roadlab: y-up XZ plan exports directly to OpenDRIVE's z-up XY; "
               "both rotate the first plan axis toward the second and both put +t left. -->");
        w.close("</header>");
    }

    for (const Road& r : net.roads()) {
        if (!opt.includeConnectors && r.kind == RoadKind::Connector) continue;
        if (r.activeLength() < 1e-3) continue;
        w.open("<road " + w.attr("name", r.name) + " " + w.attr("length", r.activeLength()) +
               " " + w.attr("id", r.id) + " " + w.attr("junction", r.junctionId) + " " +
               w.attr("rule", std::string("RHT")) + ">");
        writeLink(w, net, r);
        {
            // The design speed is a property of the whole road, and it is the
            // same number the geometry was checked against.
            w.open("<type " + w.attr("s", 0.0) + " " +
                   w.attr("type",
                          std::string(r.designSpeed >= 90 ? "motorway"
                                                          : (r.designSpeed >= 55 ? "rural"
                                                                                 : "town"))) +
                   ">");
            w.line("<speed " + w.attr("max", r.designSpeed) + " " +
                   w.attr("unit", std::string("km/h")) + "/>");
            w.close("</type>");
        }
        writePlanView(w, r);
        w.open("<elevationProfile>");
        writeProfile(w, r.spine.elevationConst(), r.begin(), r.end(), "elevation");
        w.close("</elevationProfile>");
        w.open("<lateralProfile>");
        writeProfile(w, r.spine.superelevationConst(), r.begin(), r.end(), "superelevation");
        w.close("</lateralProfile>");
        writeLanes(w, r);
        w.close("</road>");
    }

    for (const Junction& j : net.junctions()) {
        if (j.connections.empty()) continue;
        w.open("<junction " + w.attr("id", j.id) + " " + w.attr("name", j.name) + ">");
        for (size_t ci = 0; ci < j.connections.size(); ++ci) {
            const Connection& c = j.connections[ci];
            if (c.connectorRoad < 0) continue;
            const Road& conn = net.road(c.connectorRoad);
            // The connector's own lane: a generated connector carries exactly one.
            int connLane = 0;
            int sec = conn.xs.sectionIndexAt(conn.begin() + 1e-3);
            for (const Strip& st : conn.xs.sections[size_t(std::max(0, sec))].right)
                if (st.isLane()) connLane = st.id;
            w.open("<connection " + w.attr("id", int(ci)) + " " +
                   w.attr("incomingRoad", c.from.road) + " " +
                   w.attr("connectingRoad", c.connectorRoad) + " " +
                   w.attr("contactPoint", std::string("start")) + ">");
            w.line("<laneLink " + w.attr("from", c.from.lane) + " " + w.attr("to", connLane) +
                   "/>");
            w.close("</connection>");
        }
        w.close("</junction>");
    }

    w.close("</OpenDRIVE>");
    return w.out.str();
}

bool writeOpenDrive(const Network& net, const std::string& path, std::string& error,
                    const OdrOptions& opt) {
    std::ofstream out(path);
    if (!out) {
        error = "cannot write " + path;
        return false;
    }
    out << openDriveString(net, opt);
    return true;
}

}  // namespace roadlab
