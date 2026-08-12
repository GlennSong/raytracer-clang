#include "odr.h"

#include <cstdio>
#include <fstream>
#include <map>
#include "rl_xml.h"
#include <sstream>

namespace roadlab {

const char* odrLaneType(StripKind kind, int dir) {
    switch (kind) {
        case StripKind::Turn:
            // A centre turn lane with no direction of its own is a TWLTL, and
            // that is exactly what OpenDRIVE means by "bidirectional".
            return dir == 0 ? "bidirectional" : "driving";
        case StripKind::Travel:
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

// One <signal> on an approach: what the arm faces as it enters the junction.
struct OdrSignal {
    int roadId = -1;
    double s = 0;
    double t = 0;
    bool forward = true;   // orientation "+" (with s) or "-"
    std::string name;
    std::string type;
    std::string subtype = "-1";
    bool dynamic = false;
};

struct OdrJunctionOut {
    int id = -1;
    std::string name;
    std::vector<OdrConnection> conns;
};

struct ExportPlan {
    std::vector<OdrRoad> roads;
    std::vector<OdrJunctionOut> junctions;
    std::vector<OdrSignal> signals;

    OdrRoad* find(int id) {
        for (OdrRoad& r : roads)
            if (r.id == id) return &r;
        return nullptr;
    }
};

const char* roadKindName(RoadKind k) {
    switch (k) {
        case RoadKind::Normal: return "normal";
        case RoadKind::Ramp: return "ramp";
        case RoadKind::Connector: return "connector";
        case RoadKind::RoundaboutRing: return "ring";
    }
    return "normal";
}

double maxArmPriority(const Junction& j) {
    double m = 0;
    for (const JunctionArm& a : j.arms) m = std::max(m, a.priority);
    return m;
}

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

        if (!opt.includeObjects) continue;
        // How each approach is controlled. The `type` is the standard code, so
        // any reader knows what the sign is; the `name` carries our exact enum
        // so a round trip does not have to reverse-engineer it from the code.
        for (const JunctionArm& arm : jn.arms) {
            OdrRoad* ar = plan.find(arm.road);
            if (!ar) continue;
            OdrSignal sig;
            sig.roadId = arm.road;
            // Stop line: at the arm's contact, re-based into the exported window.
            sig.s = clampd(arm.sContact - ar->s0, 0.0, ar->length());
            // Beside the approach lanes, on the driver's right.
            sig.t = arm.atEnd ? arm.rightExtent - 0.6 : -arm.leftExtent + 0.6;
            sig.forward = arm.atEnd;
            sig.name = std::string("roadlab:") + controlName(jn.control);
            switch (jn.control) {
                case JunctionControl::Signalized:
                    sig.type = "1000001";   // OpenDRIVE-country traffic light
                    sig.dynamic = true;
                    break;
                case JunctionControl::AllWayStop:
                    sig.type = "206";
                    break;
                case JunctionControl::PriorityStop:
                    // Only the minor arms actually carry a sign.
                    sig.type = arm.priority < maxArmPriority(jn) - 1e-6 ? "206" : "306";
                    break;
                case JunctionControl::Yield:
                case JunctionControl::RoundaboutEntry:
                    sig.type = "205";
                    break;
                case JunctionControl::Merge:
                case JunctionControl::Uncontrolled:
                    sig.type = "";   // nothing stands there, but the name still says so
                    break;
            }
            plan.signals.push_back(sig);
        }
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
           w.attr("type", std::string(odrLaneType(st.kind, st.dir))) + " " +
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
    // The surface material and the kerb height are ours, not OpenDRIVE's, but
    // both have a slot in the schema that means the right thing, so they survive
    // a round trip instead of coming back as flat asphalt.
    w.line("<material " + w.attr("sOffset", 0.0) + " " +
           w.attr("surface", std::string(surfaceName(st.surface))) + " " +
           w.attr("friction", st.surface == SurfaceKind::Grass ? 0.4 : 0.8) + "/>");
    if (st.speedLimit > 0.5f) {
        w.line("<speed " + w.attr("sOffset", 0.0) + " " + w.attr("max", double(st.speedLimit)) +
               " " + w.attr("unit", std::string("km/h")) + "/>");
    }
    if (st.height > 0.001f) {
        w.line("<height " + w.attr("sOffset", 0.0) + " " + w.attr("inner", double(st.height)) +
               " " + w.attr("outer", double(st.height)) + "/>");
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
        bool anySignal = false;
        for (const OdrSignal& sg : plan.signals) {
            if (sg.roadId != pr.id) continue;
            if (!anySignal) w.open("<signals>");
            anySignal = true;
            w.line("<signal " + w.attr("s", sg.s) + " " + w.attr("t", sg.t) + " " +
                   w.attr("id", int(&sg - plan.signals.data())) + " " + w.attr("name", sg.name) +
                   " " + w.attr("dynamic", std::string(sg.dynamic ? "yes" : "no")) + " " +
                   w.attr("orientation", std::string(sg.forward ? "+" : "-")) + " " +
                   w.attr("zOffset", 2.2) + " " +
                   w.attr("country", std::string("OpenDRIVE")) + " " + w.attr("type", sg.type) +
                   " " + w.attr("subtype", sg.subtype) + " " +
                   w.attr("hOffset", 0.0) + "/>");
        }
        if (anySignal) w.close("</signals>");
        // A ramp and a circulatory carriageway are ordinary roads to OpenDRIVE,
        // but the difference decides who yields to whom, so it goes in the
        // format's sanctioned extension slot rather than being thrown away.
        if (pr.junction < 0 && r.kind != RoadKind::Normal) {
            w.open("<userData>");
            w.line("<roadlab " + w.attr("kind", std::string(roadKindName(r.kind))) + "/>");
            w.close("</userData>");
        }
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


// --- import ---------------------------------------------------------------

StripKind stripKindFromOdr(const std::string& t) {
    if (t == "driving") return StripKind::Travel;
    if (t == "bidirectional") return StripKind::Turn;
    if (t == "bus" || t == "hov" || t == "taxi") return StripKind::Bus;
    if (t == "biking") return StripKind::Bike;
    if (t == "parking") return StripKind::Parking;
    if (t == "shoulder" || t == "stop") return StripKind::Shoulder;
    if (t == "median") return StripKind::Median;
    if (t == "curb") return StripKind::Curb;
    if (t == "border") return StripKind::Gutter;
    if (t == "restricted") return StripKind::Verge;
    if (t == "sidewalk" || t == "walking") return StripKind::Sidewalk;
    // "none", "special1..3", "roadWorks", "entry", "exit", "offRamp", "onRamp",
    // "connectingRamp" all end up somewhere reasonable rather than dropped.
    if (t == "entry" || t == "exit" || t == "offRamp" || t == "onRamp" ||
        t == "connectingRamp")
        return StripKind::Travel;
    return StripKind::Slope;
}

MarkStyle markStyleFromOdr(const std::string& t) {
    if (t == "solid") return MarkStyle::Solid;
    if (t == "broken") return MarkStyle::Dashed;
    if (t == "solid solid") return MarkStyle::Double;
    if (t == "solid broken") return MarkStyle::SolidDashed;
    if (t == "broken solid") return MarkStyle::DashedSolid;
    if (t == "broken broken") return MarkStyle::Dashed;
    if (t == "botts dots") return MarkStyle::Botts;
    return MarkStyle::None;
}

JunctionControl junctionControlFromOdr(const std::string& s) {
    if (s == "yield") return JunctionControl::Yield;
    if (s == "priority-stop") return JunctionControl::PriorityStop;
    if (s == "all-way-stop") return JunctionControl::AllWayStop;
    if (s == "signalized") return JunctionControl::Signalized;
    if (s == "roundabout") return JunctionControl::RoundaboutEntry;
    if (s == "merge") return JunctionControl::Merge;
    return JunctionControl::Uncontrolled;
}

SurfaceKind surfaceKindFromOdr(const std::string& s) {
    if (s == "concrete") return SurfaceKind::Concrete;
    if (s == "gravel") return SurfaceKind::Gravel;
    if (s == "dirt" || s == "soil") return SurfaceKind::Dirt;
    if (s == "grass") return SurfaceKind::Grass;
    if (s == "brick" || s == "cobble" || s == "pavingStone") return SurfaceKind::Brick;
    if (s == "steel") return SurfaceKind::Steel;
    return SurfaceKind::Asphalt;
}

// What a lane made of, when the file does not say. Files written by other tools
// almost never carry <material>, and defaulting everything to asphalt turns
// verges and medians into pavement.
SurfaceKind defaultSurfaceFor(StripKind kind) {
    switch (kind) {
        case StripKind::Verge:
        case StripKind::Slope:
        case StripKind::Median:
            return SurfaceKind::Grass;
        case StripKind::Curb:
        case StripKind::Gutter:
        case StripKind::Sidewalk:
        case StripKind::Barrier:
        case StripKind::Apron:
            return SurfaceKind::Concrete;
        default:
            return SurfaceKind::Asphalt;
    }
}

PaintColor paintColorFromOdr(const std::string& c) {
    if (c == "yellow") return PaintColor::Yellow;
    if (c == "red") return PaintColor::Red;
    if (c == "blue") return PaintColor::Blue;
    if (c == "green") return PaintColor::Green;
    return PaintColor::White;   // "standard" and "white"
}

namespace {

Poly3 polyFrom(const XmlNode& n) {
    return Poly3{n.attrD("a"), n.attrD("b"), n.attrD("c"), n.attrD("d")};
}

void readProfile(const XmlNode* parent, const char* element, Profile1D& out) {
    out.clear();
    if (!parent) return;
    for (const XmlNode* e : parent->all(element)) out.add(e->attrD("s"), polyFrom(*e));
}

// poly3 and paramPoly3 have no primitive in this model. Rather than drop them or
// pretend they are straight, sample the curve and emit a chain of ANCHORED short
// segments: each carries its own absolute pose, so the approximation cannot drift
// even over a long record.
void approximateCurve(Spine& sp, const XmlNode& geom, const XmlNode& shape, bool parametric,
                      OdrImportReport& rep) {
    double x = geom.attrD("x"), y = geom.attrD("y"), hdg = geom.attrD("hdg");
    double len = geom.attrD("length");
    if (len <= 1e-6) return;
    double c = std::cos(hdg), s = std::sin(hdg);

    auto localAt = [&](double p, double& lu, double& lv) {
        if (parametric) {
            lu = shape.attrD("aU") + p * (shape.attrD("bU") +
                                          p * (shape.attrD("cU") + p * shape.attrD("dU")));
            lv = shape.attrD("aV") + p * (shape.attrD("bV") +
                                          p * (shape.attrD("cV") + p * shape.attrD("dV")));
        } else {
            lu = p;
            lv = shape.attrD("a") + p * (shape.attrD("b") +
                                         p * (shape.attrD("c") + p * shape.attrD("d")));
        }
    };
    // pRange="normalized" means the parameter runs 0..1; "arcLength" means 0..len.
    bool normalized = parametric && shape.attrS("pRange", "normalized") != "arcLength";
    double pMax = normalized ? 1.0 : len;

    int steps = std::max(4, int(len / 1.0));
    steps = std::min(steps, 2000);
    Vec2 prev{x, y};
    double prevP = 0;
    for (int i = 1; i <= steps; ++i) {
        double p = pMax * double(i) / double(steps);
        double lu = 0, lv = 0;
        localAt(p, lu, lv);
        Vec2 world{x + c * lu - s * lv, y + s * lu + c * lv};
        Vec2 d = world - prev;
        double segLen = length(d);
        if (segLen > 1e-7)
            sp.addAnchored(GeomKind::Line, prev, headingOf(d), segLen, 0.0, 0.0);
        prev = world;
        prevP = p;
    }
    (void)prevP;
    ++rep.approximatedGeometry;
}

}  // namespace

bool openDriveFromString(const std::string& doc, Network& out, std::string& error,
                         OdrImportReport* reportOut) {
    XmlNode root;
    if (!xmlParse(doc, root, error)) return false;
    if (root.name != "OpenDRIVE") {
        error = "not an OpenDRIVE document (root is <" + root.name + ">)";
        return false;
    }
    OdrImportReport rep;
    if (const XmlNode* hdr = root.child("header")) rep.name = hdr->attrS("name");

    // What the approaches are controlled by, gathered while the roads are read
    // and matched to junction arms afterwards.
    struct ApproachSignal {
        int road = -1;
        double s = 0;
        bool forward = true;
        JunctionControl control = JunctionControl::Uncontrolled;
        bool explicitControl = false;   // the file named it, rather than us inferring
    };
    std::vector<ApproachSignal> approaches;

    // The file's ids need not be dense or zero-based, so build a mapping and
    // remap every reference through it.
    std::map<int, int> roadIdMap;
    std::map<int, int> junctionIdMap;
    std::vector<const XmlNode*> roadNodes = root.all("road");
    std::vector<const XmlNode*> junctionNodes = root.all("junction");
    for (size_t i = 0; i < roadNodes.size(); ++i) roadIdMap[roadNodes[i]->attrI("id", int(i))] = int(i);
    for (size_t i = 0; i < junctionNodes.size(); ++i)
        junctionIdMap[junctionNodes[i]->attrI("id", int(i))] = int(i);
    auto mapRoad = [&](int fileId) {
        auto it = roadIdMap.find(fileId);
        return it == roadIdMap.end() ? -1 : it->second;
    };
    auto mapJunction = [&](int fileId) {
        auto it = junctionIdMap.find(fileId);
        return it == junctionIdMap.end() ? -1 : it->second;
    };

    for (const XmlNode* rn : roadNodes) {
        Road r;
        r.name = rn->attrS("name", "road");
        int junctionFileId = rn->attrI("junction", -1);
        r.junctionId = junctionFileId >= 0 ? mapJunction(junctionFileId) : -1;
        r.kind = r.junctionId >= 0 ? RoadKind::Connector : RoadKind::Normal;
        if (const XmlNode* ud = rn->child("userData")) {
            if (const XmlNode* mine = ud->child("roadlab")) {
                std::string k = mine->attrS("kind");
                if (k == "ramp") r.kind = RoadKind::Ramp;
                else if (k == "ring") r.kind = RoadKind::RoundaboutRing;
                else if (k == "connector") r.kind = RoadKind::Connector;
            }
        }

        // --- plan view ---
        const XmlNode* pv = rn->child("planView");
        Vec2 chained{0, 0};
        double chainedHdg = 0;
        bool first = true;
        if (pv) {
            for (const XmlNode* g : pv->all("geometry")) {
                double gx = g->attrD("x"), gy = g->attrD("y"), ghdg = g->attrD("hdg");
                double glen = g->attrD("length");
                if (glen <= 1e-9) continue;
                if (!first) {
                    double drift = length(Vec2{gx, gy} - chained);
                    rep.maxGeometryDrift = std::max(rep.maxGeometryDrift, drift);
                }
                first = false;
                if (const XmlNode* line = g->child("line")) {
                    (void)line;
                    r.spine.addAnchored(GeomKind::Line, {gx, gy}, ghdg, glen, 0.0, 0.0);
                } else if (const XmlNode* arc = g->child("arc")) {
                    double k = arc->attrD("curvature");
                    r.spine.addAnchored(std::fabs(k) < 1e-9 ? GeomKind::Line : GeomKind::Arc,
                                        {gx, gy}, ghdg, glen, k, k);
                } else if (const XmlNode* sp = g->child("spiral")) {
                    r.spine.addAnchored(GeomKind::Spiral, {gx, gy}, ghdg, glen,
                                        sp->attrD("curvStart"), sp->attrD("curvEnd"));
                } else if (const XmlNode* pp = g->child("paramPoly3")) {
                    approximateCurve(r.spine, *g, *pp, true, rep);
                } else if (const XmlNode* p3 = g->child("poly3")) {
                    approximateCurve(r.spine, *g, *p3, false, rep);
                } else {
                    r.spine.addAnchored(GeomKind::Line, {gx, gy}, ghdg, glen, 0.0, 0.0);
                    ++rep.approximatedGeometry;
                }
                // Where this record ENDS, for the drift check on the next one.
                r.spine.finalize();
                Frame f = r.spine.frameAt(r.spine.length());
                chained = f.planPos;
                chainedHdg = f.heading;
            }
        }
        (void)chainedHdg;
        if (r.spine.prims().empty()) {
            r.spine.setStart({0, 0}, 0.0);
            r.spine.addLine(std::max(0.1, rn->attrD("length", 1.0)));
        }
        r.spine.setCrossfall(0.0);   // the file's own geometry already has it
        readProfile(rn->child("elevationProfile"), "elevation", r.spine.elevation());
        readProfile(rn->child("lateralProfile"), "superelevation", r.spine.superelevation());
        r.spine.finalize();

        // --- links ---
        auto readLink = [&](const XmlNode* n, RoadLink& link) {
            if (!n) return;
            std::string type = n->attrS("elementType");
            int id = n->attrI("elementId", -1);
            if (type == "junction") {
                int mapped = mapJunction(id);
                if (mapped >= 0) link = RoadLink{LinkType::Junction, mapped, true, {}};
            } else if (type == "road") {
                int mapped = mapRoad(id);
                if (mapped >= 0)
                    link = RoadLink{LinkType::Road, mapped, n->attrS("contactPoint", "start") ==
                                                                 "start", {}};
            }
        };
        if (const XmlNode* lk = rn->child("link")) {
            readLink(lk->child("predecessor"), r.pred);
            readLink(lk->child("successor"), r.succ);
        }
        if (const XmlNode* ty = rn->child("type")) {
            if (const XmlNode* sp = ty->child("speed")) {
                double v = sp->attrD("max", 0.0);
                std::string unit = sp->attrS("unit", "km/h");
                if (unit == "m/s") v *= 3.6;
                if (unit == "mph") v *= 1.609344;
                if (v > 1.0) r.designSpeed = v;
            }
        }

        // --- lanes ---
        const XmlNode* lanes = rn->child("lanes");
        if (lanes) {
            readProfile(lanes, "laneOffset", r.xs.laneOffset);
            for (const XmlNode* ls : lanes->all("laneSection")) {
                LaneSection sec;
                sec.s0 = ls->attrD("s");
                auto readSide = [&](const char* sideName, std::vector<Strip>& stack) {
                    const XmlNode* side = ls->child(sideName);
                    if (!side) return;
                    std::vector<const XmlNode*> ln = side->all("lane");
                    // Our stacks run outward from the reference line; OpenDRIVE
                    // lists left lanes outermost-first, so that side is reversed.
                    if (std::string(sideName) == "left")
                        std::reverse(ln.begin(), ln.end());
                    for (const XmlNode* l : ln) {
                        Strip st;
                        st.id = l->attrI("id", 0);
                        st.kind = stripKindFromOdr(l->attrS("type", "none"));
                        std::vector<const XmlNode*> widths = l->all("width");
                        if (widths.empty()) {
                            st.width = Poly3::constant(0.0);
                        } else {
                            st.width = polyFrom(*widths.front());
                            if (widths.size() > 1)
                                rep.extraWidthRecords += int(widths.size()) - 1;
                        }
                        if (const XmlNode* rm = l->child("roadMark")) {
                            st.outerMark.style = markStyleFromOdr(rm->attrS("type", "none"));
                            st.outerMark.color = paintColorFromOdr(rm->attrS("color", "standard"));
                            double w = rm->attrD("width", 0.12);
                            if (w > 0.01) st.outerMark.width = float(w);
                        }
                        if (const XmlNode* sp = l->child("speed")) {
                            double v = sp->attrD("max", 0.0);
                            std::string unit = sp->attrS("unit", "km/h");
                            if (unit == "m/s") v *= 3.6;
                            if (unit == "mph") v *= 1.609344;
                            if (v > 1.0) st.speedLimit = float(v);
                        }
                        // Direction is not stored: in right-hand traffic the
                        // negative-id lanes run with s and the positive ones
                        // against, which is this model's own convention too.
                        bool drives = st.kind == StripKind::Travel || st.kind == StripKind::Bus;
                        st.dir = drives ? (st.id < 0 ? int8_t(1) : int8_t(-1)) : int8_t(0);
                        // A "bidirectional" lane belongs to neither direction; a
                        // turn lane spelled "driving" belongs to its own side.
                        if (st.kind == StripKind::Turn && l->attrS("type") != "bidirectional")
                            st.dir = st.id < 0 ? int8_t(1) : int8_t(-1);
                        if (st.kind == StripKind::Sidewalk) st.access = kAccessPed;
                        st.surface = defaultSurfaceFor(st.kind);
                        if (const XmlNode* mat = l->child("material"))
                            st.surface = surfaceKindFromOdr(mat->attrS("surface", "asphalt"));
                        if (st.kind == StripKind::Sidewalk || st.kind == StripKind::Curb)
                            st.height = 0.15f;
                        if (const XmlNode* h = l->child("height"))
                            st.height = float(std::max(h->attrD("inner"), h->attrD("outer")));
                        // At an interior boundary <link> names the neighbouring
                        // lane section and finalize() will recompute it; at the
                        // two outer boundaries it names the linked ROAD's lane,
                        // which nothing else can recover.
                        if (const XmlNode* lk = l->child("link")) {
                            if (const XmlNode* p = lk->child("predecessor"))
                                st.predecessor = p->attrI("id", kNoLane);
                            if (const XmlNode* sc = lk->child("successor"))
                                st.successor = sc->attrI("id", kNoLane);
                        }
                        stack.push_back(st);
                        ++rep.lanes;
                    }
                };
                readSide("right", sec.right);
                readSide("left", sec.left);
                if (const XmlNode* centre = ls->child("center")) {
                    if (const XmlNode* cl = centre->child("lane")) {
                        if (const XmlNode* rm = cl->child("roadMark")) {
                            sec.centerMark.style = markStyleFromOdr(rm->attrS("type", "none"));
                            sec.centerMark.color =
                                paintColorFromOdr(rm->attrS("color", "standard"));
                            double w = rm->attrD("width", 0.12);
                            if (w > 0.01) sec.centerMark.width = float(w);
                        }
                    }
                }
                sec.assignIds();
                r.xs.sections.push_back(sec);
                ++rep.laneSections;
            }
        }
        if (r.xs.sections.empty()) {
            LaneSection sec = roadPreset("street2").section;
            sec.s0 = 0;
            r.xs.sections.push_back(sec);
        }
        r.xs.finalize(r.spine.length());

        if (const XmlNode* sigs = rn->child("signals")) {
            for (const XmlNode* sg : sigs->all("signal")) {
                ApproachSignal as;
                as.road = rep.roads;   // roads are appended in document order
                as.s = sg->attrD("s");
                as.forward = sg->attrS("orientation", "+") != "-";
                std::string nm = sg->attrS("name");
                std::string type = sg->attrS("type");
                if (nm.rfind("roadlab:", 0) == 0) {
                    as.control = junctionControlFromOdr(nm.substr(8));
                    as.explicitControl = true;
                } else if (type.rfind("1000", 0) == 0 || sg->attrS("dynamic", "no") == "yes") {
                    as.control = JunctionControl::Signalized;   // a light of some country
                } else if (type == "206") {
                    as.control = JunctionControl::AllWayStop;
                } else if (type == "205") {
                    as.control = JunctionControl::Yield;
                } else {
                    continue;   // a speed-limit or warning sign says nothing about control
                }
                approaches.push_back(as);
            }
        }

        out.addRoad(std::move(r));
        ++rep.roads;
    }

    // --- junctions ---
    for (const XmlNode* jn : junctionNodes) {
        Junction j;
        j.name = jn->attrS("name", "junction");
        j.imported = true;
        j.control = JunctionControl::Uncontrolled;
        std::vector<std::pair<int, bool>> armSeen;   // (road, atEnd)
        auto armIndex = [&](int road, bool atEnd) {
            for (size_t i = 0; i < armSeen.size(); ++i)
                if (armSeen[i].first == road && armSeen[i].second == atEnd) return int(i);
            armSeen.push_back({road, atEnd});
            JunctionArm arm;
            arm.road = road;
            arm.atEnd = atEnd;
            j.arms.push_back(arm);
            return int(armSeen.size()) - 1;
        };

        for (const XmlNode* cn : jn->all("connection")) {
            int incoming = mapRoad(cn->attrI("incomingRoad", -1));
            int connecting = mapRoad(cn->attrI("connectingRoad", -1));
            if (incoming < 0 || connecting < 0) continue;
            const Road& conn = out.road(connecting);
            // Which end of each arm the junction holds comes from the CONNECTING
            // road's own links, not from the arm's. The arm may reach the
            // junction through an intermediate road (an exporter that carves
            // stubs writes exactly that), in which case the arm's own link names
            // the stub and says nothing about the junction; the connector always
            // names both of its neighbours with a contactPoint.
            int outgoing = -1;
            bool incAtEnd = false, outAtEnd = false;
            bool haveIncoming = false;
            bool outIsConnSucc = true;   // which end of the connector faces out
            for (const RoadLink* lk : {&conn.pred, &conn.succ}) {
                if (lk->type != LinkType::Road || lk->id < 0) continue;
                if (lk->id == incoming && !haveIncoming) {
                    haveIncoming = true;
                    incAtEnd = !lk->toStart;
                } else {
                    outgoing = lk->id;
                    outAtEnd = !lk->toStart;
                    outIsConnSucc = (lk == &conn.succ);
                }
            }
            const Road& inc = out.road(incoming);
            if (!haveIncoming) {
                // The connector does not name the incoming road — fall back to
                // whichever of its ends carries a junction link.
                incAtEnd = inc.succ.type == LinkType::Junction || inc.pred.type != LinkType::Junction;
            }
            if (outgoing < 0) continue;

            Connection c;
            c.connectorRoad = connecting;
            c.fromArm = armIndex(incoming, incAtEnd);
            c.toArm = armIndex(outgoing, outAtEnd);
            int fromLane = 0, toLane = 0;
            for (const XmlNode* ll : cn->all("laneLink")) {
                fromLane = ll->attrI("from", 0);
                toLane = ll->attrI("to", 0);
                break;   // one connector road carries one movement here
            }
            int incSection = inc.xs.sectionIndexAt(incAtEnd ? std::max(inc.begin(), inc.end() - 1e-3)
                                                            : inc.begin() + 1e-3);
            const Road& outRoad = out.road(outgoing);
            int outSection = outRoad.xs.sectionIndexAt(
                outAtEnd ? std::max(outRoad.begin(), outRoad.end() - 1e-3)
                         : outRoad.begin() + 1e-3);
            c.from = {incoming, incSection, fromLane};
            // <laneLink to="..."> names a lane on the CONNECTING road, not on the
            // outgoing one. Which lane the movement actually lands in is on that
            // connector lane's own link, at whichever end faces outward.
            int connLane = toLane;
            int farLane = connLane;
            {
                double at = outIsConnSucc ? std::max(conn.begin(), conn.end() - 1e-3)
                                          : conn.begin() + 1e-3;
                int cs = conn.xs.sectionIndexAt(at);
                const LaneSection& cse = conn.xs.sections[size_t(std::max(0, cs))];
                const Strip* st = cse.strip(connLane);
                if (st) {
                    int far = outIsConnSucc ? st->successor : st->predecessor;
                    if (far != kNoLane) farLane = far;
                }
            }
            c.to = {outgoing, outSection, farLane};
            j.connections.push_back(c);
        }
        if (j.arms.empty()) continue;
        // How the junction is controlled comes from the signs standing on its
        // approaches — which is where the information lives in OpenDRIVE, and
        // also where it lives in the world.
        for (const ApproachSignal& as : approaches) {
            bool onAnArm = false;
            for (const JunctionArm& arm : j.arms)
                if (arm.road == as.road && arm.atEnd == as.forward) onAnArm = true;
            if (!onAnArm) continue;
            if (as.explicitControl) {
                j.control = as.control;
                break;
            }
            if (j.control == JunctionControl::Uncontrolled) j.control = as.control;
        }
        out.addJunction(j);
        ++rep.junctions;
    }

    if (rep.approximatedGeometry > 0)
        rep.notes.push_back(std::to_string(rep.approximatedGeometry) +
                            " polynomial geometry record(s) sampled into segments");
    if (rep.extraWidthRecords > 0)
        rep.notes.push_back(std::to_string(rep.extraWidthRecords) +
                            " extra <width> record(s) ignored; only the first per lane is used");
    if (rep.maxGeometryDrift > 0.05)
        rep.notes.push_back("geometry records disagree by up to " +
                            std::to_string(rep.maxGeometryDrift) +
                            " m; each is anchored to its own pose");
    if (reportOut) *reportOut = rep;
    return true;
}

bool readOpenDrive(const std::string& path, Network& out, std::string& error,
                   OdrImportReport* report) {
    std::ifstream in(path);
    if (!in) {
        error = "cannot open " + path;
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return openDriveFromString(ss.str(), out, error, report);
}

}  // namespace roadlab
