#include "odr.h"

#include <cstdio>
#include <fstream>
#include <map>
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

// --- the export plan ------------------------------------------------------
//
// The file is NOT a one-to-one dump of the network. OpenDRIVE allows a road at
// most one predecessor and one successor, and requires a <junction> wherever the
// connection is not one-to-one. Our ramps are deliberately modelled internally
// as a lane continuing into another lane (which is what a merge physically is,
// and it avoids inventing a junction with no pad) — but at a diverge that leaves
// the mainline with two successors, which is exactly the case the format
// forbids.
//
// So export builds a PLAN first: a list of road windows, some of which are short
// stubs carved off the front or back of a real road to serve as the connecting
// roads of a synthesised junction. Nothing new is invented geometrically — a
// stub is another window over a spine we already have, the same trick junction
// trimming and road splitting already use.

namespace {

struct OdrLink {
    LinkType type = LinkType::None;
    int id = -1;
    // Which end of the TARGET we attach to. Mirrors RoadLink::toStart.
    bool toStart = true;
};

struct OdrRoad {
    int id = -1;
    int source = -1;      // index into net.roads()
    double s0 = 0, s1 = 0;
    int junction = -1;
    std::string name;
    OdrLink pred, succ;
    // Cross-road lane links, used at the road's first and last lane sections
    // where OpenDRIVE reads <lane><link> as referring to the linked ROAD's lanes
    // rather than to the neighbouring lane section.
    std::map<int, int> predLaneLinks;
    std::map<int, int> succLaneLinks;

    double length() const { return std::max(0.0, s1 - s0); }
};

struct OdrConnection {
    int incoming = -1;
    int connecting = -1;
    bool contactStart = true;
    std::vector<std::pair<int, int>> laneLinks;
};

struct OdrJunctionOut {
    int id = -1;
    std::string name;
    std::vector<OdrConnection> conns;
};

struct ExportPlan {
    std::vector<OdrRoad> roads;
    std::vector<OdrJunctionOut> junctions;

    OdrRoad* find(int id) {
        for (OdrRoad& r : roads)
            if (r.id == id) return &r;
        return nullptr;
    }
};

// How lanes pair up across a plain road link, matching what the lane graph does:
// both sides are ordered right-to-left in the travel frame, so the pairing is a
// zip and an unpaired lane is a lane that ends.
std::vector<std::pair<int, int>> defaultLanePairs(const Network& net, int roadA, bool aAtEnd,
                                                  int roadB, bool bAtStart) {
    if (roadA < 0 || roadB < 0) return {};
    // The same positional pairing the lane graph uses, so the exported laneLinks
    // and the simulator's topology cannot disagree.
    return pairLanesAcross(net.road(roadA), aAtEnd, net.road(roadB), bAtStart);
}

// One branch of a not-one-to-one connection.
struct Branch {
    int road = -1;
    bool contactAtEnd = false;   // where the branch attaches to the shared point
    std::vector<std::pair<int, int>> laneLinks;
};

// Carve a stub off `target` at the end nearest the junction, and wire it up as a
// connecting road between `hub` and `target`.
bool makeStub(ExportPlan& plan, int& nextRoadId, int hubRoad, bool hubAtEnd, const Branch& branch,
              int junctionId, bool incomingIsHub, OdrConnection& connOut) {
    OdrRoad* tr = plan.find(branch.road);
    OdrRoad* hub = plan.find(hubRoad);
    if (!tr || !hub) return false;
    double len = tr->length();
    if (len < 4.0) return false;
    double L = std::min(15.0, len * 0.4);

    OdrRoad stub;
    stub.id = nextRoadId++;
    stub.source = tr->source;
    stub.junction = junctionId;
    stub.name = tr->name + ".stub";

    if (!branch.contactAtEnd) {
        // The branch is entered at its start, so the stub is its leading slice.
        stub.s0 = tr->s0;
        stub.s1 = tr->s0 + L;
        tr->s0 += L;
        stub.pred = {LinkType::Road, hubRoad, !hubAtEnd};
        stub.succ = {LinkType::Road, tr->id, true};
        tr->pred = {LinkType::Road, stub.id, false};
    } else {
        // Entered at its end: the stub is the trailing slice, and the geometric
        // roles swap because OpenDRIVE links are about ends, not direction.
        stub.s1 = tr->s1;
        stub.s0 = tr->s1 - L;
        tr->s1 -= L;
        stub.succ = {LinkType::Road, hubRoad, !hubAtEnd};
        stub.pred = {LinkType::Road, tr->id, false};
        tr->succ = {LinkType::Road, stub.id, true};
    }

    // contactPoint is where the INCOMING road meets the connecting road. At a
    // diverge the incoming road is the hub; at a merge it is the branch, and the
    // two sit at opposite ends of the stub — which is why this is not simply a
    // property of how the stub was carved.
    connOut.contactStart = incomingIsHub ? !branch.contactAtEnd : branch.contactAtEnd;

    // The stub is a slice of the target, so its lanes ARE the target's lanes;
    // the link onward is the identity.
    for (const auto& pr : branch.laneLinks) {
        if (!branch.contactAtEnd)
            stub.succLaneLinks[pr.second] = pr.second;
        else
            stub.predLaneLinks[pr.second] = pr.second;
    }
    connOut.incoming = incomingIsHub ? hubRoad : branch.road;
    connOut.connecting = stub.id;
    connOut.laneLinks = branch.laneLinks;
    plan.roads.push_back(stub);
    return true;
}

// Replace an ambiguous connection at one road end with a junction whose
// connecting roads are stubs of the branches.
void synthesiseJunction(ExportPlan& plan, const Network& net, int& nextRoadId, int& nextJunctionId,
                        int hubRoad, bool hubAtEnd, const std::vector<Branch>& branches,
                        bool incomingIsHub, const char* kind) {
    OdrRoad* hub = plan.find(hubRoad);
    if (!hub || branches.size() < 2) return;
    OdrJunctionOut j;
    j.id = nextJunctionId;
    j.name = std::string(kind) + "-" + (hub->name.empty() ? std::to_string(hubRoad) : hub->name);

    std::vector<OdrConnection> conns;
    for (const Branch& b : branches) {
        OdrConnection c;
        if (makeStub(plan, nextRoadId, hubRoad, hubAtEnd, b, j.id, incomingIsHub, c))
            conns.push_back(c);
    }
    if (conns.size() < 2) return;   // could not carve stubs; leave it alone

    // Re-find: makeStub pushed onto plan.roads and may have reallocated.
    hub = plan.find(hubRoad);
    if (!hub) return;
    if (hubAtEnd)
        hub->succ = {LinkType::Junction, j.id, true};
    else
        hub->pred = {LinkType::Junction, j.id, true};
    j.conns = conns;
    plan.junctions.push_back(j);
    ++nextJunctionId;
    (void)net;
}

ExportPlan buildPlan(const Network& net, const OdrOptions& opt) {
    ExportPlan plan;
    int nextRoadId = 0;
    for (const Road& r : net.roads()) {
        if (!opt.includeConnectors && r.kind == RoadKind::Connector) continue;
        if (r.activeLength() < 1e-3) continue;
        OdrRoad pr;
        pr.id = r.id;
        pr.source = r.id;
        pr.s0 = r.begin();
        pr.s1 = r.end();
        pr.junction = r.junctionId;
        pr.name = r.name;
        pr.pred = {r.pred.type, r.pred.id, r.pred.toStart};
        pr.succ = {r.succ.type, r.succ.id, r.succ.toStart};
        plan.roads.push_back(pr);
        nextRoadId = std::max(nextRoadId, r.id + 1);
    }
    int nextJunctionId = net.junctionCount();

    // Real junctions. A connecting road must carry its own links to the incoming
    // and outgoing roads, not just be named by the <connection>.
    for (const Junction& jn : net.junctions()) {
        if (jn.connections.empty()) continue;
        OdrJunctionOut j;
        j.id = jn.id;
        j.name = jn.name;
        for (const Connection& c : jn.connections) {
            if (c.connectorRoad < 0) continue;
            OdrRoad* conn = plan.find(c.connectorRoad);
            if (!conn) continue;
            const Road& cr = net.road(c.connectorRoad);
            int connLane = 0;
            int sec = cr.xs.sectionIndexAt(cr.begin() + 1e-3);
            for (const Strip& st : cr.xs.sections[size_t(std::max(0, sec))].right)
                if (st.isLane()) connLane = st.id;

            const JunctionArm& fromArm = jn.arms[size_t(c.fromArm)];
            const JunctionArm& toArm = jn.arms[size_t(c.toArm)];
            conn->pred = {LinkType::Road, c.from.road, !fromArm.atEnd};
            conn->succ = {LinkType::Road, c.to.road, !toArm.atEnd};
            conn->predLaneLinks[connLane] = c.from.lane;
            conn->succLaneLinks[connLane] = c.to.lane;

            OdrConnection oc;
            oc.incoming = c.from.road;
            oc.connecting = c.connectorRoad;
            oc.contactStart = true;
            oc.laneLinks.push_back({c.from.lane, connLane});
            j.conns.push_back(oc);
        }
        if (!j.conns.empty()) plan.junctions.push_back(j);
    }

    // Ramps. An ExtraLaneLink is one-to-one until it collides with an existing
    // road link — and when it does, that end needs a junction.
    for (const ExtraLaneLink& el : net.extraLinks) {
        OdrRoad* from = plan.find(el.fromRoad);
        OdrRoad* to = plan.find(el.toRoad);
        if (!from || !to) continue;

        OdrLink fromEndLink = el.fromAtEnd ? from->succ : from->pred;
        OdrLink toEndLink = el.toAtStart ? to->pred : to->succ;

        if (fromEndLink.type == LinkType::Road && fromEndLink.id >= 0) {
            // DIVERGE: the mainline already goes somewhere, and now the ramp
            // leaves from the same place.
            std::vector<Branch> branches;
            Branch through;
            through.road = fromEndLink.id;
            through.contactAtEnd = !fromEndLink.toStart;
            through.laneLinks = defaultLanePairs(net, el.fromRoad, el.fromAtEnd, fromEndLink.id,
                                                 fromEndLink.toStart);
            Branch exitBranch;
            exitBranch.road = el.toRoad;
            exitBranch.contactAtEnd = !el.toAtStart;
            exitBranch.laneLinks = {{el.fromLane, el.toLane}};
            branches.push_back(through);
            branches.push_back(exitBranch);
            synthesiseJunction(plan, net, nextRoadId, nextJunctionId, el.fromRoad, el.fromAtEnd,
                               branches, /*incomingIsHub=*/true, "diverge");
        } else if (toEndLink.type == LinkType::Road && toEndLink.id >= 0) {
            // MERGE: something already arrives here, and now the ramp does too.
            // The hub is the DOWNSTREAM road, and the branches are its feeders.
            std::vector<Branch> branches;
            Branch mainline;
            mainline.road = toEndLink.id;
            mainline.contactAtEnd = !toEndLink.toStart;
            mainline.laneLinks =
                defaultLanePairs(net, toEndLink.id, !toEndLink.toStart, el.toRoad, el.toAtStart);
            Branch ramp;
            ramp.road = el.fromRoad;
            ramp.contactAtEnd = el.fromAtEnd;
            ramp.laneLinks = {{el.fromLane, el.toLane}};
            branches.push_back(mainline);
            branches.push_back(ramp);
            // The hub is the DOWNSTREAM road and the branches feed it, so the
            // <connection> names the feeder as the incoming road.
            synthesiseJunction(plan, net, nextRoadId, nextJunctionId, el.toRoad, !el.toAtStart,
                               branches, /*incomingIsHub=*/false, "merge");
        } else {
            // Genuinely one-to-one: a plain road link says it exactly.
            if (el.fromAtEnd)
                from->succ = {LinkType::Road, el.toRoad, el.toAtStart};
            else
                from->pred = {LinkType::Road, el.toRoad, el.toAtStart};
            if (el.toAtStart)
                to->pred = {LinkType::Road, el.fromRoad, !el.fromAtEnd};
            else
                to->succ = {LinkType::Road, el.fromRoad, !el.fromAtEnd};
            if (el.fromAtEnd)
                from->succLaneLinks[el.fromLane] = el.toLane;
            else
                from->predLaneLinks[el.fromLane] = el.toLane;
            if (el.toAtStart)
                to->predLaneLinks[el.toLane] = el.fromLane;
            else
                to->succLaneLinks[el.toLane] = el.fromLane;
        }
    }
    return plan;
}

// --- emission -------------------------------------------------------------

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
                  const char* element) {
    if (profile.pieces.empty()) return;
    for (size_t i = 0; i < profile.pieces.size(); ++i) {
        double pieceStart = profile.pieces[i].s0;
        double pieceEnd = (i + 1 < profile.pieces.size()) ? profile.pieces[i + 1].s0 : 1e300;
        double a = std::max(pieceStart, begin);
        double b = std::min(pieceEnd, end);
        if (b - a < 1e-6) continue;
        Poly3 p = profile.pieces[i].poly.shifted(a - pieceStart);
        w.line(std::string("<") + element + " " + w.attr("s", a - begin) + " " + w.poly(p) + "/>");
    }
}

void writeLink(Writer& w, const OdrRoad& pr) {
    auto describe = [&](const OdrLink& link, const char* tag) {
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
    if (pr.pred.type == LinkType::None && pr.succ.type == LinkType::None) return;
    w.open("<link>");
    describe(pr.pred, "predecessor");
    describe(pr.succ, "successor");
    w.close("</link>");
}

void writePlanView(Writer& w, const Road& r, double begin, double end) {
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
        Frame f = r.spine.frameAt(begin);
        w.open("<geometry " + w.attr("s", 0.0) + " " + w.attr("x", f.planPos.x) + " " +
               w.attr("y", f.planPos.y) + " " + w.attr("hdg", f.heading) + " " +
               w.attr("length", std::max(0.01, end - begin)) + ">");
        w.line("<line/>");
        w.close("</geometry>");
    }
    w.close("</planView>");
}

void writeLane(Writer& w, const OdrRoad& pr, const LaneSection& sec, const Strip& st, double clipS,
               bool firstSection, bool lastSection) {
    w.open("<lane " + w.attr("id", st.id) + " " +
           w.attr("type", std::string(odrLaneType(st.kind))) + " " +
           w.attr("level", std::string("false")) + ">");

    // At the first and last lane sections OpenDRIVE reads <lane><link> as
    // referring to the LINKED ROAD's lanes; in between it means the neighbouring
    // lane section. Those two slots never collide, because linkSections only
    // fills the interior ones.
    int predLane = st.predecessor;
    int succLane = st.successor;
    if (firstSection) {
        auto it = pr.predLaneLinks.find(st.id);
        if (it != pr.predLaneLinks.end()) predLane = it->second;
    }
    if (lastSection) {
        auto it = pr.succLaneLinks.find(st.id);
        if (it != pr.succLaneLinks.end()) succLane = it->second;
    }
    if (predLane != kNoLane || succLane != kNoLane) {
        w.open("<link>");
        if (predLane != kNoLane) w.line("<predecessor " + w.attr("id", predLane) + "/>");
        if (succLane != kNoLane) w.line("<successor " + w.attr("id", succLane) + "/>");
        w.close("</link>");
    }
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
                                  : (m.crossableFromLeft()
                                         ? "decrease"
                                         : (m.crossableFromRight() ? "increase" : "none")))) +
           "/>");
    if (st.speedLimit > 0.5f) {
        w.line("<speed " + w.attr("sOffset", 0.0) + " " + w.attr("max", double(st.speedLimit)) +
               " " + w.attr("unit", std::string("km/h")) + "/>");
    }
    w.close("</lane>");
}

void writeLanes(Writer& w, const OdrRoad& pr, const Road& r) {
    double begin = pr.s0, end = pr.s1;
    w.open("<lanes>");
    writeProfile(w, r.xs.laneOffset, begin, end, "laneOffset");

    // Which sections actually fall inside the window, so "first" and "last" mean
    // first and last EXPORTED, not first and last in the source road.
    std::vector<size_t> live;
    for (size_t i = 0; i < r.xs.sections.size(); ++i) {
        double a = std::max(r.xs.sections[i].s0, begin);
        double b = std::min(r.xs.sections[i].s0 + r.xs.sections[i].length, end);
        if (b - a >= 1e-6) live.push_back(i);
    }
    for (size_t k = 0; k < live.size(); ++k) {
        const LaneSection& sec = r.xs.sections[live[k]];
        double a = std::max(sec.s0, begin);
        bool firstSection = (k == 0);
        bool lastSection = (k + 1 == live.size());
        w.open("<laneSection " + w.attr("s", a - begin) + ">");
        if (!sec.left.empty()) {
            w.open("<left>");
            // OpenDRIVE wants left lanes in descending id order, outermost first.
            for (size_t i = sec.left.size(); i-- > 0;)
                writeLane(w, pr, sec, sec.left[i], a, firstSection, lastSection);
            w.close("</left>");
        }
        w.open("<center>");
        {
            const Marking& m = sec.centerMark;
            w.open("<lane " + w.attr("id", 0) + " " + w.attr("type", std::string("none")) + " " +
                   w.attr("level", std::string("false")) + ">");
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
            for (size_t i = 0; i < sec.right.size(); ++i)
                writeLane(w, pr, sec, sec.right[i], a, firstSection, lastSection);
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
    ExportPlan plan = buildPlan(net, opt);

    Vec2 lo{1e300, 1e300}, hi{-1e300, -1e300};
    for (const Road& r : net.roads()) {
        Vec2 a, b;
        r.spine.planBounds(a, b);
        lo.x = std::min(lo.x, a.x);
        lo.y = std::min(lo.y, a.y);
        hi.x = std::max(hi.x, b.x);
        hi.y = std::max(hi.y, b.y);
    }
    if (net.roads().empty()) {
        lo = {0, 0};
        hi = {0, 0};
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

    for (const OdrRoad& pr : plan.roads) {
        if (pr.length() < 1e-3) continue;
        const Road& r = net.road(pr.source);
        w.open("<road " + w.attr("name", pr.name) + " " + w.attr("length", pr.length()) + " " +
               w.attr("id", pr.id) + " " + w.attr("junction", pr.junction) + " " +
               w.attr("rule", std::string("RHT")) + ">");
        writeLink(w, pr);
        {
            w.open("<type " + w.attr("s", 0.0) + " " +
                   w.attr("type", std::string(r.designSpeed >= 90
                                                  ? "motorway"
                                                  : (r.designSpeed >= 55 ? "rural" : "town"))) +
                   ">");
            w.line("<speed " + w.attr("max", r.designSpeed) + " " +
                   w.attr("unit", std::string("km/h")) + "/>");
            w.close("</type>");
        }
        writePlanView(w, r, pr.s0, pr.s1);
        w.open("<elevationProfile>");
        writeProfile(w, r.spine.elevationConst(), pr.s0, pr.s1, "elevation");
        w.close("</elevationProfile>");
        w.open("<lateralProfile>");
        writeProfile(w, r.spine.superelevationConst(), pr.s0, pr.s1, "superelevation");
        w.close("</lateralProfile>");
        writeLanes(w, pr, r);
        w.close("</road>");
    }

    for (const OdrJunctionOut& j : plan.junctions) {
        if (j.conns.empty()) continue;
        w.open("<junction " + w.attr("id", j.id) + " " + w.attr("name", j.name) + ">");
        for (size_t ci = 0; ci < j.conns.size(); ++ci) {
            const OdrConnection& c = j.conns[ci];
            w.open("<connection " + w.attr("id", int(ci)) + " " +
                   w.attr("incomingRoad", c.incoming) + " " +
                   w.attr("connectingRoad", c.connecting) + " " +
                   w.attr("contactPoint", std::string(c.contactStart ? "start" : "end")) + ">");
            for (const auto& ll : c.laneLinks)
                w.line("<laneLink " + w.attr("from", ll.first) + " " + w.attr("to", ll.second) +
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
