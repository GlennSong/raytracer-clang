#include "engine/procgen/city/roads/lanes/road_layout.h"

#include "engine/bundle/bundle.h"                                 // fnv1aStr: laneLevelKey

#include "engine/procgen/city/roads/lanes/control_plan.h"        // findJunctions: the nodes and their grouping, once
#include "engine/procgen/city/roads/lanes/lane_expand.h"         // expand: ramps composed as the builder composes them
#include "engine/procgen/city/roads/lanes/lane_graph_connect.h"  // kStraightOnDegrees: the one "straight on"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/road_design.h"         // planCrossings: the planner's crossing policy
#include "rt_math.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <set>
#include <tuple>

namespace engine {
namespace roads::lanes {

namespace {

std::vector<CrossingSpec> crossingsFromJson(const nlohmann::json& list) {
    std::vector<CrossingSpec> out;
    for (const nlohmann::json& c : list) {
        CrossingSpec x;
        x.a = c.at("a").get<std::string>();
        x.b = c.at("b").get<std::string>();
        x.kind = c.value("kind", std::string("at_grade"));
        x.at = Vec2(c.at("at").at(0).get<double>(), c.at("at").at(1).get<double>());
        out.push_back(x);
    }
    return out;
}

// A spread junction's points (ModelNode, ADR-0425), as the file writes them.
nlohmann::json nodesJson(const std::vector<ModelNode>& nodes) {
    nlohmann::json list = nlohmann::json::array();
    for (const ModelNode& n : nodes) {
        nlohmann::json points = nlohmann::json::array();
        for (const ModelNodePoint& p : n.points) points.push_back({{"road", p.road}, {"at", p.station}});
        list.push_back({{"at", {written(n.at.x), written(n.at.y)}}, {"points", points}});
    }
    return list;
}

// A street as the file writes it; the junctions and streets it names by id are places in `layout`'s lists.
nlohmann::json streetJson(const ModelStreet& st, const RoadLayout& layout) {
    nlohmann::json ends = nlohmann::json::array();
    for (const ModelJunctionId j : st.junctions) ends.push_back(j.valid() ? nlohmann::json(layout.junctions[j.index].id) : nlohmann::json());
    nlohmann::json s = {{"id", st.id}, {"road", st.road}, {"from", st.from}, {"to", st.to}, {"junctions", ends}};
    if (st.carriesOn[0].valid() || st.carriesOn[1].valid()) {
        nlohmann::json on = nlohmann::json::array();
        for (const ModelStreetId c : st.carriesOn) on.push_back(c.valid() ? nlohmann::json(layout.streets[c.index].id) : nlohmann::json());
        s["carries_on"] = on;
    }
    if (!st.name.empty()) s["name"] = st.name;
    return s;
}

nlohmann::json dividedJson(const std::vector<DividedPair>& divided) {
    nlohmann::json list = nlohmann::json::array();
    for (const DividedPair& p : divided) list.push_back({p[0], p[1]});
    return list;
}

// ---- a JSON text read where it lies: just enough to find members and their bytes, never re-written -------------------
constexpr std::size_t kNone = std::string::npos;

struct Member {
    std::string key;
    std::size_t lead = 0;         // just after the '{' or ',' before it
    std::size_t comma = kNone;    // the ',' before it (none for the first)
    std::size_t keyBegin = 0, valueBegin = 0, valueEnd = 0;
};

class Text {
public:
    explicit Text(const std::string& s) : s_(s) {}
    std::size_t ws(std::size_t i) const {
        while (i < s_.size() && (s_[i] == ' ' || s_[i] == '\n' || s_[i] == '\r' || s_[i] == '\t')) ++i;
        return i;
    }
    // One past the end of the value starting at (or after whitespace from) i; kNone when the text is not well formed.
    std::size_t end(std::size_t i) const {
        i = ws(i);
        if (i >= s_.size()) return kNone;
        const char c = s_[i];
        if (c == '"') {
            for (++i; i < s_.size(); ++i) {
                if (s_[i] == '\\') ++i;
                else if (s_[i] == '"') return i + 1;
            }
            return kNone;
        }
        if (c == '{' || c == '[') {
            int depth = 0;
            while (i < s_.size()) {
                const char d = s_[i];
                if (d == '"') {
                    i = end(i);
                    if (i == kNone) return kNone;
                    continue;
                }
                if (d == '{' || d == '[') ++depth;
                else if ((d == '}' || d == ']') && --depth == 0) return i + 1;
                ++i;
            }
            return kNone;
        }
        while (i < s_.size() && s_[i] != ',' && s_[i] != '}' && s_[i] != ']' && s_[i] != ' ' && s_[i] != '\n' && s_[i] != '\r' && s_[i] != '\t') ++i;
        return i;
    }
    bool isObject(std::size_t i) const { return i < s_.size() && s_[i] == '{'; }
    bool isArray(std::size_t i) const { return i < s_.size() && s_[i] == '['; }
    // The members of the object opening at `open`; false when it is not well formed.
    bool members(std::size_t open, std::vector<Member>& out) const {
        out.clear();
        if (!isObject(open)) return false;
        std::size_t i = open + 1, comma = kNone;
        for (;;) {
            const std::size_t k = ws(i);
            if (k >= s_.size()) return false;
            if (s_[k] == '}' && out.empty() && comma == kNone) return true;
            if (s_[k] != '"') return false;
            Member m;
            m.lead = i;
            m.comma = comma;
            m.keyBegin = k;
            const std::size_t keyEnd = end(k);
            if (keyEnd == kNone) return false;
            m.key = s_.substr(k + 1, keyEnd - k - 2);
            const std::size_t colon = ws(keyEnd);
            if (colon >= s_.size() || s_[colon] != ':') return false;
            m.valueBegin = ws(colon + 1);
            m.valueEnd = end(m.valueBegin);
            if (m.valueEnd == kNone) return false;
            out.push_back(m);
            const std::size_t next = ws(m.valueEnd);
            if (next >= s_.size()) return false;
            if (s_[next] == '}') return true;
            if (s_[next] != ',') return false;
            comma = next;
            i = next + 1;
        }
    }
    // Where each element of the array opening at `open` begins; false when it is not well formed.
    bool elements(std::size_t open, std::vector<std::size_t>& out) const {
        out.clear();
        if (!isArray(open)) return false;
        std::size_t i = open + 1;
        for (;;) {
            const std::size_t k = ws(i);
            if (k >= s_.size()) return false;
            if (s_[k] == ']' && out.empty()) return true;
            const std::size_t e = end(k);
            if (e == kNone) return false;
            out.push_back(k);
            const std::size_t next = ws(e);
            if (next >= s_.size()) return false;
            if (s_[next] == ']') return true;
            if (s_[next] != ',') return false;
            i = next + 1;
        }
    }
    const std::string& str() const { return s_; }

private:
    const std::string& s_;
};

// The bytes a member takes with it: the comma before it, or (the first of its object) up to the next member.
std::pair<std::size_t, std::size_t> removal(const std::vector<Member>& ms, std::size_t k) {
    const Member& m = ms[k];
    if (m.comma != kNone) return {m.comma, m.valueEnd};
    if (k + 1 < ms.size()) return {m.keyBegin, ms[k + 1].keyBegin};
    return {m.keyBegin, m.valueEnd};
}

const Member* memberNamed(const std::vector<Member>& ms, const std::string& key) {
    for (const Member& m : ms)
        if (m.key == key) return &m;
    return nullptr;
}

// The whitespace that leads a member (between the ',' or '{' before it and its key): what a member inserted after it
// is led by, so it lines up.
std::string leadOf(const std::string& s, const Member& m) { return s.substr(m.lead, m.keyBegin - m.lead); }

}  // namespace

nlohmann::json junctionJson(const ModelJunction& pj, const std::vector<ModelStreet>& streets) {
    nlohmann::json j;
    j["id"] = pj.id;
    j["at"] = {pj.at.x, pj.at.y};
    if (!pj.control.empty()) j["control"] = pj.control;
    if (!pj.why.empty()) j["why"] = pj.why;
    if (pj.hand) j["hand"] = true;
    nlohmann::json arms = nlohmann::json::array();
    for (const ModelArm& a : pj.arms) {
        nlohmann::json aj;
        aj["road"] = a.road;
        aj["at"] = a.station;
        aj["spine"] = a.spineIn ? "in" : "out";
        if (!a.regulations.empty()) aj["regulations"] = a.regulations;
        if (a.street.valid()) aj["street"] = streets[a.street.index].id;
        if (stated(a.heading)) aj["heading"] = a.heading;
        arms.push_back(aj);
    }
    j["arms"] = arms;
    if (!pj.inside.empty()) {
        nlohmann::json inside = nlohmann::json::array();
        for (const ModelStretch& st : pj.inside) inside.push_back({{"road", st.road}, {"from", st.from}, {"to", st.to}});
        j["inside"] = inside;
    }
    if (!pj.through.empty()) j["through"] = pj.through;
    if (!pj.nodes.empty()) j["nodes"] = nodesJson(pj.nodes);
    return j;
}

std::vector<DividedPair> dividedFromIds(const RoadLabGraph& g, std::vector<std::string>* unpaired) {
    std::vector<DividedPair> out;
    auto endsWith = [](const std::string& s, const std::string& tail) {
        return s.size() > tail.size() && s.compare(s.size() - tail.size(), tail.size(), tail) == 0;
    };
    for (const EdgeSpec& e : g.edges) {
        const bool in = endsWith(e.id, "_in"), outward = endsWith(e.id, "_out");
        if (!in && !outward) continue;
        const std::string name = e.id.substr(0, e.id.size() - (in ? 3 : 4));
        const EdgeSpec* other = g.find(name + (in ? "_out" : "_in"));
        std::string why;
        if (!other) why = "no road " + name + (in ? "_out" : "_in");
        else if (other->cls != e.cls) why = "its partner " + other->id + " is of another class (" + other->cls + ")";
        else if (!e.lanes.oneWay() || !other->lanes.oneWay()) why = "it or its partner is not one-way";
        if (!why.empty()) {
            if (unpaired) unpaired->push_back("road " + e.id + ": no divided pair (" + why + ")");
            continue;
        }
        if (in) out.push_back(DividedPair{e.id, other->id});
    }
    return out;
}

namespace {
// The graph's divided roads: as it states them, else (a file made before they were stated) found from its ids.
std::vector<DividedPair> dividedOf(const RoadLabGraph& graph, LayoutSource source, std::vector<std::string>* unpaired = nullptr) {
    return graph.listsDivided || source == LayoutSource::Planner ? graph.divided : dividedFromIds(graph, unpaired);
}

// HOW THE LAYOUT FINDS A ROAD FILE'S JUNCTIONS: the one set-up, so the layout's junctions (layoutJunctionFacts) and the
// controls planned onto them (planLayoutControls) are found the same way and each control lands on a junction the
// layout wrote.
//   - two roads meeting away from a shared end: as the graph declares them, else as the planner's crossing policy has
//     them (planCrossings: the planner's own, which city_plan's design step writes into the scene as its crossings);
//   - the divided roads: as the graph states them, else found from its ids;
//   - a planner's file joins road ends that are one written place (kWrittenStation); an older file by its storage rule.
// Holds the lists `how` points into, so it is made in place and never copied.
struct LayoutFinding {
    std::vector<CrossingSpec> crossings;
    std::vector<DividedPair> divided;
    JunctionFinding how;
    LayoutFinding(const RoadLabGraph& graph, LayoutSource source, std::vector<CarryOn>* carryOns)
        : crossings(graph.declaresCrossings ? graph.crossings : crossingsFromJson(planCrossings(graph))),
          divided(dividedOf(graph, source)) {
        how.endTolerance = source == LayoutSource::Planner ? kWrittenStation : -1.0;
        how.atGrade = &crossings;
        how.divided = &divided;
        how.carryOns = carryOns;
    }
    LayoutFinding(const LayoutFinding&) = delete;
    LayoutFinding& operator=(const LayoutFinding&) = delete;
};
}  // namespace

std::vector<JunctionFacts> layoutJunctionFacts(const RoadLabGraph& graph, LayoutSource source, std::vector<CarryOn>* carryOns) {
    const LayoutFinding finding(graph, source, carryOns);
    return findJunctions(graph, ControlPolicy(), finding.how);
}

ControlPlan planLayoutControls(const RoadLabGraph& graph, LayoutSource source, const ControlPolicy& policy) {
    const LayoutFinding finding(graph, source, nullptr);
    return planControls(graph, policy, finding.how);
}

RoadLayout deriveLayout(const RoadLabGraph& graph, LayoutSource source) {
    RoadLayout out;
    const ComposedRoads composed(graph);   // the ramps composed from their anchors: the builder's stations
    const RoadLabGraph& g = composed.graph();
    out.divided = dividedOf(graph, source, &out.notes);
    std::vector<CarryOn> carryOns;   // where a street only carries on (no junction): its two streets name each other
    // ---- the junctions: the graph's own list as it states them, or found ----------------------------------------------
    if (graph.listsJunctions) {
        out.junctions = graph.junctions;
        out.keptList = true;
        // (the places a street carries on, as the control plan finds its nodes: city_plan controls wrote the list so)
        JunctionFinding how;
        how.carryOns = &carryOns;
        (void)findJunctions(graph, ControlPolicy(), how);
    } else {
        for (const JunctionFacts& f : layoutJunctionFacts(graph, source, &carryOns)) out.junctions.push_back(junctionOf(f));
    }
    // ---- each arm's heading; the arms of a junction found here put in ring order (armRing: the one place); the through
    // pairs. (A list the file states is kept as it is, marked or not: `lanes_tool layout --arm-order --write` puts an
    // unmarked one in ring order and marks it.)
    out.armsInRing = !out.keptList || graph.armsInRing;
    for (ModelJunction& pj : out.junctions) {
        for (ModelArm& a : pj.arms) {
            const EdgeSpec* e = g.find(a.road);
            if (!e || e->xy.size() < 2) {
                out.notes.push_back("junction " + pj.id + ": arm on road " + a.road + " has no spine to measure");
                continue;
            }
            if (!stated(a.heading)) a.heading = degreesOf(armDirection(*e, a.station, a.spineIn));
        }
        // a junction found here carries the through pairs its facts state (control_plan.h findJunctions: the one rule),
        // following its arms into their ring; a list the file states and has no pair written for gets them from the same
        // rule, over its headings as written
        if (!out.keptList) {
            orderArmsRound(pj, composed, &out.notes);
            for (std::array<uint32_t, 2>& t : pj.through)
                if (t[0] > t[1]) std::swap(t[0], t[1]);
            std::sort(pj.through.begin(), pj.through.end());
        } else if (pj.through.empty()) {
            std::vector<Vec2> dirs;
            for (const ModelArm& a : pj.arms) dirs.push_back(stated(a.heading) ? directionOf(a.heading) : Vec2(1, 0));
            pj.through = throughPairs(dirs);
        }
    }
    // ---- the streets: each paved road cut at its arms, the stretches inside a junction left out ------------------------
    struct Cut {
        double s;
        int junction;
        bool spineIn;
    };
    std::map<std::string, std::vector<Cut>> cutsOf;
    std::map<std::string, std::vector<std::pair<double, double>>> insideOf;
    for (std::size_t j = 0; j < out.junctions.size(); ++j) {
        for (const ModelArm& a : out.junctions[j].arms) cutsOf[a.road].push_back(Cut{a.station, static_cast<int>(j), a.spineIn});
        for (const ModelStretch& st : out.junctions[j].inside) insideOf[st.road].emplace_back(st.from, st.to);
    }
    for (const EdgeSpec& e : g.edges) {
        if (e.laneCount() == 0 || e.xy.size() < 2 || e.s.empty()) continue;
        const double length = written(e.s.back());
        std::vector<double> at{0.0, length};
        for (const Cut& c : cutsOf[e.id]) at.push_back(c.s);
        for (const std::pair<double, double>& in : insideOf[e.id]) {
            at.push_back(in.first);
            at.push_back(in.second);
        }
        std::sort(at.begin(), at.end());
        at.erase(std::unique(at.begin(), at.end(), [](double a, double b) { return sameWrittenStation(a, b); }), at.end());
        std::vector<ModelStreet> mine;
        for (std::size_t k = 0; k + 1 < at.size(); ++k) {
            const double from = std::max(0.0, at[k]), to = std::min(length, at[k + 1]);
            if (to - from < kShortestStreet) continue;
            const double mid = 0.5 * (from + to);
            bool inside = false;
            for (const std::pair<double, double>& in : insideOf[e.id]) inside = inside || (mid >= in.first && mid <= in.second);
            if (inside) continue;
            ModelStreet st;
            st.road = e.id;
            st.from = from;
            st.to = to;
            for (const Cut& c : cutsOf[e.id]) {
                if (!c.spineIn && sameWrittenStation(c.s, from)) st.junctions[0] = ModelJunctionId(static_cast<uint32_t>(c.junction));
                if (c.spineIn && sameWrittenStation(c.s, to)) st.junctions[1] = ModelJunctionId(static_cast<uint32_t>(c.junction));
            }
            mine.push_back(st);
        }
        for (std::size_t k = 0; k < mine.size(); ++k) {
            mine[k].id = mine.size() == 1 ? e.id : e.id + "/" + std::to_string(k);
            out.streets.push_back(mine[k]);
        }
    }
    std::map<std::string, std::vector<std::size_t>> streetsOf;
    for (std::size_t i = 0; i < out.streets.size(); ++i) streetsOf[out.streets[i].road].push_back(i);
    // ---- where a street only carries on: no junction, and its two streets name each other (ADR-0419) ------------------
    for (const CarryOn& c : carryOns) {
        std::array<int, 2> st{-1, -1}, end{0, 0};
        for (std::size_t k = 0; k < 2; ++k) {
            end[k] = c.stations[k] < kArmReachesPast ? 0 : 1;   // the road's start, or its end
            for (std::size_t i : streetsOf[c.roads[k]]) {
                const ModelStreet& s = out.streets[i];
                if (end[k] == 0 ? sameWrittenStation(s.from, 0.0) : atOrPast(s.to, written(c.stations[k]), kWrittenStation)) st[k] = static_cast<int>(i);
            }
        }
        if (st[0] < 0 || st[1] < 0) {
            out.notes.push_back("roads " + c.roads[0] + " and " + c.roads[1] + " carry on into each other where no street ends");
            continue;
        }
        // A STREET'S END IS AT A JUNCTION OR CARRIES ON, NEVER BOTH (ADR-0435). Where the file's own list states a junction
        // at this meeting (a control plan that gives carry-ons a control makes each a junction of two arms), the junction
        // says how the street runs on (its through pair), and every reader asks the junction first (the road model,
        // lanes/lane_level.cpp). The grouping above still reports the meeting as a carry-on; stating it too wrote a
        // second answer nothing read into the file (traffic_lab: 40 street ends).
        if (out.streets[static_cast<std::size_t>(st[0])].junctions[static_cast<std::size_t>(end[0])].valid() ||
            out.streets[static_cast<std::size_t>(st[1])].junctions[static_cast<std::size_t>(end[1])].valid())
            continue;
        out.streets[static_cast<std::size_t>(st[0])].carriesOn[static_cast<std::size_t>(end[0])] = ModelStreetId(static_cast<uint32_t>(st[1]));
        out.streets[static_cast<std::size_t>(st[1])].carriesOn[static_cast<std::size_t>(end[1])] = ModelStreetId(static_cast<uint32_t>(st[0]));
    }
    // ---- each arm's street ---------------------------------------------------------------------------------------------
    for (ModelJunction& pj : out.junctions)
        for (ModelArm& a : pj.arms) {
            a.street = ModelStreetId();
            for (std::size_t i : streetsOf[a.road]) {
                const ModelStreet& st = out.streets[i];
                if (sameWrittenStation(a.spineIn ? st.to : st.from, a.station)) a.street = ModelStreetId(static_cast<uint32_t>(i));
            }
            if (!a.street.valid()) {
                char buf[256];
                std::snprintf(buf, sizeof buf, "junction %s: its arm on road %s (spine %s at %.2f m) is no street's end", pj.id.c_str(),
                              a.road.c_str(), a.spineIn ? "in" : "out", a.station);
                out.notes.push_back(buf);
            }
        }
    return out;
}

std::string describeLayout(const RoadLayout& layout) {
    std::map<std::size_t, uint32_t> byArms;
    uint32_t through = 0;
    for (const ModelJunction& pj : layout.junctions) {
        ++byArms[pj.arms.size()];
        through += static_cast<uint32_t>(pj.through.size());
    }
    uint32_t deadEnds = 0, bothJunctions = 0, carryOnEnds = 0;
    for (const ModelStreet& st : layout.streets) {
        deadEnds += (st.junctions[0].valid() ? 0u : 1u) + (st.junctions[1].valid() ? 0u : 1u);
        bothJunctions += st.junctions[0].valid() && st.junctions[1].valid() ? 1u : 0u;
        carryOnEnds += (st.carriesOn[0].valid() ? 1u : 0u) + (st.carriesOn[1].valid() ? 1u : 0u);
    }
    char buf[384];
    std::snprintf(buf, sizeof buf, "%zu junctions (%s), %zu streets (%u with a junction at both ends, %u ends at no junction, %u of those "
                  "carrying on into another street), %u through pairs, %zu divided roads",
                  layout.junctions.size(), layout.keptList ? "the graph's own list, kept" : "found", layout.streets.size(), bothJunctions,
                  deadEnds, carryOnEnds, through, layout.divided.size());
    std::string out = buf;
    for (const DividedPair& p : layout.divided) out += "\n  divided road: " + p[0] + " and " + p[1];
    out += "\n  junctions by arms:";
    for (const std::pair<const std::size_t, uint32_t>& b : byArms) out += " " + std::to_string(b.second) + " with " + std::to_string(b.first) + ";";
    out += "\n";
    for (const std::string& n : layout.notes) out += "  note: " + n + "\n";
    return out;
}

void writeLayout(nlohmann::json& scene, const RoadLayout& layout) {
    nlohmann::json junctions = nlohmann::json::array(), streets = nlohmann::json::array();
    for (const ModelJunction& pj : layout.junctions) junctions.push_back(junctionJson(pj, layout.streets));
    for (const ModelStreet& st : layout.streets) streets.push_back(streetJson(st, layout));
    scene["junctions"] = junctions;
    if (layout.armsInRing) scene["arms_in_ring"] = true;
    else scene.erase("arms_in_ring");
    scene["streets"] = streets;
    if (!layout.divided.empty()) scene["divided"] = dividedJson(layout.divided);
}

// The text with the cut members taken out, each with its comma.
static std::string cutText(const std::string& text, std::vector<std::pair<std::size_t, std::size_t>> cut) {
    if (cut.empty()) return text;
    std::sort(cut.begin(), cut.end());
    std::string out;
    out.reserve(text.size());
    std::size_t at = 0;
    for (const std::pair<std::size_t, std::size_t>& c : cut) {
        if (c.first < at) continue;   // (never: the members cut do not overlap)
        out.append(text, at, c.first - at);
        at = c.second;
    }
    out.append(text, at, kNone);
    return out;
}

std::string keyedRoadFileText(const std::string& text) {
    const Text t(text);
    std::vector<Member> top;
    if (!t.members(t.ws(0), top)) return text;
    std::vector<std::pair<std::size_t, std::size_t>> cut;
    // The streets, the divided pairs, and the whole junctions list with its controls and signs (ADR-0421) are cut from
    // these bytes, with the lane level's rules (ADR-0426) and the arms' ring mark (ADR-0427): the meshes, deck and road
    // graph come from the edges, and the lane graph keys what it reads itself (laneLevelKey). What the PAINT reads of
    // them (controls, signs, streets, the arms' order, the lane rules: ADR-0431) keys the city beside these bytes, not in
    // them: deck_mesh.h paintFactsKey, hashed by the city producer.
    // Since step 9 (ADR-0428) the paint draws each stated junction's arms (deck_mesh.h junctionAreas), so what it reads of
    // them -- each junction's arms by road, station and way, and its stretches inside -- is keyed, after the cut text, in
    // an order of its own (the arms' list order, their controls, signs, names and points move no paint).
    for (std::size_t k = 0; k < top.size(); ++k)
        if (top[k].key == "streets" || top[k].key == "divided" || top[k].key == "junctions" || top[k].key == "lane_rules" ||
            top[k].key == "arms_in_ring" || top[k].key == "bus_lines")
            cut.push_back(removal(top, k));
    std::string out = cutText(text, cut);
    if (const Member* listed = memberNamed(top, "junctions")) {
        const nlohmann::json list = nlohmann::json::parse(text.substr(listed->valueBegin, listed->valueEnd - listed->valueBegin), nullptr, false);
        if (list.is_array()) {
            nlohmann::json painted = nlohmann::json::array();
            for (const nlohmann::json& j : list) {
                if (!j.is_object()) continue;
                std::vector<std::string> arms;
                for (const nlohmann::json& a : j.value("arms", nlohmann::json::array()))
                    if (a.is_object()) arms.push_back(nlohmann::json::array({a.value("road", ""), a.value("at", 0.0), a.value("spine", "")}).dump());
                std::sort(arms.begin(), arms.end());
                painted.push_back(nlohmann::json::array({arms, j.value("inside", nlohmann::json::array())}));
            }
            out += "\n/* the stated junctions the paint draws */ " + painted.dump();
        }
    }
    return out;
}

uint64_t laneLevelKey(const nlohmann::json& spec) {
    // the layout, which the lane level reads by id (its streets and carry-ons, the divided pairs, the junctions' arms,
    // through pairs, controls and signs; the mark that its arms are listed in their ring, ADR-0427) and the lane level's
    // rules (ADR-0426): what keyedRoadFileText leaves out. A
    // junction's points (ADR-0425) are the street map's: the lane level does not read them, so they key nothing.
    if (!spec.is_object()) return 0;
    std::string text;
    for (const char* key : {"junctions", "arms_in_ring", "streets", "divided", "lane_rules"}) {
        if (!spec.contains(key)) continue;
        nlohmann::json member = spec[key];
        if (std::string(key) == "junctions" && member.is_array())
            for (nlohmann::json& j : member)
                if (j.is_object()) j.erase("nodes");
        text += std::string(key) + "|" + member.dump() + "|";
    }
    return text.empty() ? 0 : bundle::fnv1aStr(text);
}

std::string insertBusLinesText(const std::string& text, const nlohmann::json& lines, std::string* err) {
    auto fail = [&](const std::string& why) {
        if (err) *err = why;
        return std::string();
    };
    const Text t(text);
    std::vector<Member> top;
    if (!t.members(t.ws(0), top) || top.empty()) return fail("the file is not a JSON object with members");
    // one line to a bus line, inside an array that opens and closes on lines of its own
    std::string value = "[";
    for (std::size_t i = 0; i < lines.size(); ++i) value += (i ? ",\n    " : "\n    ") + lines[i].dump();
    value += lines.empty() ? "]" : "\n  ]";
    std::string out = text;
    if (const Member* had = memberNamed(top, "bus_lines")) {
        out.replace(had->valueBegin, had->valueEnd - had->valueBegin, value);
    } else {
        // WHERE THE LAYOUT WRITER LEAVES IT: the layout written again takes "streets" and "divided" out and puts them
        // back after the file's last member, and the lane rules are written just before them (insertLaneRulesText), so
        // the lines go before all three -- after the member before "lane_rules" (or "streets", or "divided"); a file
        // with none of them: after its last member
        std::size_t after = top.size() - 1;
        for (std::size_t k = 0; k < top.size(); ++k)
            if (top[k].key == "lane_rules" || top[k].key == "streets" || top[k].key == "divided") {
                if (k > 0) after = k - 1;
                break;
            }
        const Member& before = top[after];
        out.insert(before.valueEnd, "," + leadOf(text, before) + "\"bus_lines\": " + value);
    }
    if (keyedRoadFileText(out) != keyedRoadFileText(text)) return fail("writing them would change what the bake key sees");
    return out;
}

uint64_t busLinesKey(const nlohmann::json& spec) {
    if (!spec.is_object() || !spec.contains("bus_lines")) return 0;
    return bundle::fnv1aStr("bus_lines|" + spec["bus_lines"].dump());
}

std::string insertLaneRulesText(const std::string& text, double solidBeforeStopLine, std::string* err) {
    auto fail = [&](const std::string& why) {
        if (err) *err = why;
        return std::string();
    };
    const Text t(text);
    std::vector<Member> top;
    if (!t.members(t.ws(0), top) || top.empty()) return fail("the file is not a JSON object with members");
    if (memberNamed(top, "lane_rules")) return fail("the file already states its \"lane_rules\"");
    // WHERE THE LAYOUT WRITER LEAVES IT (ADR-0437): the layout written again takes "streets" and "divided" out and puts
    // them back after the file's last member (insertLayoutText), so the lane rules end up before them. Written there
    // the first time -- after the member before "streets" (or "divided" where there is no "streets") -- the next
    // `layout --write` moves nothing. A file with neither: after its last member.
    std::size_t after = top.size() - 1;
    for (std::size_t k = 0; k < top.size(); ++k)
        if (top[k].key == "streets" || top[k].key == "divided") {
            if (k > 0) after = k - 1;
            break;
        }
    const Member& before = top[after];
    const std::string value = nlohmann::json{{"solid_before_stop_line", solidBeforeStopLine}}.dump();
    std::string out = text;
    out.insert(before.valueEnd, "," + leadOf(text, before) + "\"lane_rules\": " + value);
    if (keyedRoadFileText(out) != keyedRoadFileText(text)) return fail("writing it would change what the bake key sees");
    return out;
}

std::string stripLayoutText(const std::string& text, bool keepNodes) {
    const Text t(text);
    const std::size_t open = t.ws(0);
    std::vector<Member> top;
    if (!t.members(open, top)) return text;
    std::vector<std::pair<std::size_t, std::size_t>> cut;
    std::vector<Member> jm, am;
    std::vector<std::size_t> elems, arms;
    bool keepsJunctions = false;   // (the arms' ring mark goes with a junctions list cut whole: ADR-0427)
    for (std::size_t k = 0; k < top.size(); ++k) {
        const Member& m = top[k];
        if (m.key == "streets" || m.key == "divided") cut.push_back(removal(top, k));
        if (m.key != "junctions" || !t.elements(m.valueBegin, elems)) continue;
        bool controls = false;
        for (std::size_t e : elems)
            if (t.members(e, jm) && memberNamed(jm, "control")) controls = true;
        if (!controls) {
            cut.push_back(removal(top, k));
            continue;
        }
        keepsJunctions = true;
        for (std::size_t e : elems) {
            if (!t.members(e, jm)) continue;
            for (std::size_t q = 0; q < jm.size(); ++q) {
                if (jm[q].key == "through" || (jm[q].key == "nodes" && !keepNodes)) cut.push_back(removal(jm, q));
                if (jm[q].key != "arms" || !t.elements(jm[q].valueBegin, arms)) continue;
                for (std::size_t a : arms) {
                    if (!t.members(a, am)) continue;
                    for (std::size_t r = 0; r < am.size(); ++r)
                        if (am[r].key == "street" || am[r].key == "heading") cut.push_back(removal(am, r));
                }
            }
        }
    }
    if (!keepsJunctions)
        for (std::size_t k = 0; k < top.size(); ++k)
            if (top[k].key == "arms_in_ring") cut.push_back(removal(top, k));
    return cutText(text, cut);
}

std::string insertLayoutText(const std::string& text, const RoadLayout& layout, std::string* err) {
    auto fail = [&](const std::string& why) {
        if (err) *err = why;
        return std::string();
    };
    const Text t(text);
    std::vector<Member> top;
    if (!t.members(t.ws(0), top) || top.empty()) return fail("the file is not a JSON object with members");
    if (memberNamed(top, "streets")) return fail("the file already has a \"streets\" list (strip it first)");
    if (memberNamed(top, "divided")) return fail("the file already has a \"divided\" list (strip it first)");
    const Member& last = top.back();
    const std::string lead = leadOf(text, last);
    const std::size_t nl = lead.rfind('\n');
    const std::string indent = nl == std::string::npos ? std::string() : lead.substr(nl + 1);
    // a value laid out as the file lays out its own: the file's indent per level (nlohmann's dump at that width, each line
    // after the first moved in by the member's own indent), or on one line in a file written on one line
    auto valueText = [&](const nlohmann::json& v) {
        if (nl == std::string::npos) return v.dump();
        std::string d = v.dump(std::max<int>(1, static_cast<int>(indent.size())));
        std::string o;
        o.reserve(d.size() + d.size() / 8);
        for (char c : d) {
            o += c;
            if (c == '\n') o += indent;
        }
        return o;
    };
    std::vector<std::pair<std::size_t, std::string>> ins;
    std::string tail;
    if (const Member* jl = memberNamed(top, "junctions")) {
        // the file's own list: the layout must be that list, junction for junction and arm for arm
        if (!layout.keptList) return fail("the file has a \"junctions\" list the layout did not keep");
        std::vector<std::size_t> elems, arms;
        std::vector<Member> jm, am;
        if (!t.elements(jl->valueBegin, elems) || elems.size() != layout.junctions.size())
            return fail("the file's \"junctions\" list and the layout differ in length");
        for (std::size_t i = 0; i < elems.size(); ++i) {
            const ModelJunction& pj = layout.junctions[i];
            if (!t.members(elems[i], jm) || jm.empty()) return fail("junction " + std::to_string(i) + " is not an object");
            const Member* id = memberNamed(jm, "id");
            if (!id || text.substr(id->valueBegin, id->valueEnd - id->valueBegin) != nlohmann::json(pj.id).dump())
                return fail("junction " + std::to_string(i) + " is not " + pj.id + " in the file");
            const Member* armList = memberNamed(jm, "arms");
            if (!armList || !t.elements(armList->valueBegin, arms) || arms.size() != pj.arms.size())
                return fail("junction " + pj.id + ": its arms differ from the file's");
            for (std::size_t a = 0; a < arms.size(); ++a) {
                if (!t.members(arms[a], am) || am.empty()) return fail("junction " + pj.id + ": arm " + std::to_string(a) + " is not an object");
                std::string add;
                const std::string armLead = leadOf(text, am.back());
                if (!memberNamed(am, "street") && pj.arms[a].street.valid())
                    add += "," + armLead + "\"street\": " + nlohmann::json(layout.streets[pj.arms[a].street.index].id).dump();
                if (!memberNamed(am, "heading") && stated(pj.arms[a].heading)) add += "," + armLead + "\"heading\": " + nlohmann::json(pj.arms[a].heading).dump();
                if (!add.empty()) ins.emplace_back(am.back().valueEnd, add);
            }
            if (!memberNamed(jm, "through") && !pj.through.empty()) {
                // after the junction's last member, but before its nodes where the file kept them (a junction is written
                // "..., through, nodes": written back in that order, a file that has not changed comes out byte for byte)
                const Member& after = jm.back().key == "nodes" && jm.size() > 1 ? jm[jm.size() - 2] : jm.back();
                ins.emplace_back(after.valueEnd, "," + leadOf(text, jm.back()) + "\"through\": " + nlohmann::json(pj.through).dump());
            }
        }
    } else {
        nlohmann::json junctions = nlohmann::json::array();
        for (const ModelJunction& pj : layout.junctions) junctions.push_back(junctionJson(pj, layout.streets));
        tail += "," + lead + "\"junctions\": " + valueText(junctions);
        if (layout.armsInRing) tail += "," + lead + "\"arms_in_ring\": true";
    }
    nlohmann::json streets = nlohmann::json::array();
    for (const ModelStreet& st : layout.streets) streets.push_back(streetJson(st, layout));
    tail += "," + lead + "\"streets\": " + valueText(streets);
    if (!layout.divided.empty()) tail += "," + lead + "\"divided\": " + valueText(dividedJson(layout.divided));
    ins.emplace_back(last.valueEnd, tail);
    std::stable_sort(ins.begin(), ins.end(), [](const std::pair<std::size_t, std::string>& a, const std::pair<std::size_t, std::string>& b) {
        return a.first < b.first;
    });
    std::string out;
    out.reserve(text.size() + 1024);
    std::size_t at = 0;
    for (const std::pair<std::size_t, std::string>& i : ins) {
        out.append(text, at, i.first - at);
        out += i.second;
        at = i.first;
    }
    out.append(text, at, kNone);
    // (compared with the junctions' nodes out of both: `text` may have kept them, stripLayoutText's keepNodes)
    if (stripLayoutText(out) != stripLayoutText(text)) return fail("the layout, written in, does not strip back to the file it was written into");
    return out;
}

// ---- the streets' names (ADR-0420) ------------------------------------------------------------------------------------

std::vector<std::vector<std::size_t>> streetRuns(const std::vector<ModelJunction>& junctions, const std::vector<ModelStreet>& streets,
                                                 const std::vector<DividedPair>& divided, const std::vector<char>& named) {
    std::map<std::string, std::vector<std::size_t>> ofRoad;
    for (std::size_t i = 0; i < streets.size(); ++i) ofRoad[streets[i].road].push_back(i);
    std::vector<std::size_t> parent(streets.size());
    for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = i;
    std::function<std::size_t(std::size_t)> root = [&](std::size_t i) { return parent[i] == i ? i : parent[i] = root(parent[i]); };
    auto isNamed = [&](std::size_t i) { return i < named.size() && named[i]; };
    auto unite = [&](std::size_t a, std::size_t b) {
        if (!isNamed(a) || !isNamed(b)) return;
        a = root(a);
        b = root(b);
        if (a != b) parent[std::max(a, b)] = std::min(a, b);
    };
    for (const ModelJunction& j : junctions)
        for (const std::array<uint32_t, 2>& t : j.through) {
            if (t[0] >= j.arms.size() || t[1] >= j.arms.size()) continue;
            const ModelStreetId a = j.arms[t[0]].street, b = j.arms[t[1]].street;
            if (a.valid() && b.valid() && a.index < streets.size() && b.index < streets.size()) unite(a.index, b.index);
        }
    for (std::size_t i = 0; i < streets.size(); ++i)
        for (const ModelStreetId c : streets[i].carriesOn)
            if (c.valid() && c.index < streets.size()) unite(i, c.index);
    for (const DividedPair& p : divided) {
        std::vector<std::size_t> both;
        for (const std::string& road : p) {
            const std::map<std::string, std::vector<std::size_t>>::const_iterator it = ofRoad.find(road);
            if (it != ofRoad.end()) both.insert(both.end(), it->second.begin(), it->second.end());
        }
        for (std::size_t k = 1; k < both.size(); ++k) unite(both.front(), both[k]);
    }
    std::vector<std::vector<std::size_t>> runs;
    std::map<std::size_t, std::size_t> runOfRoot;
    for (std::size_t i = 0; i < streets.size(); ++i) {
        if (!isNamed(i)) continue;
        const std::size_t r = root(i);
        const std::map<std::size_t, std::size_t>::const_iterator it = runOfRoot.find(r);
        if (it == runOfRoot.end()) {
            runOfRoot[r] = runs.size();
            runs.push_back({i});
        } else {
            runs[it->second].push_back(i);
        }
    }
    return runs;
}

std::vector<char> namedStreets(const RoadLabGraph& g, const std::vector<ModelStreet>& streets, const StreetNameBook& book) {
    std::vector<char> out(streets.size(), 0);
    for (std::size_t i = 0; i < streets.size(); ++i) {
        const EdgeSpec* e = g.find(streets[i].road);
        if (!e || e->laneCount() == 0) continue;
        const RoadClassSpec& c = g.cls(*e);
        out[i] = book.unnamed.count(e->cls) || !c.carriesCars() ? 0 : 1;
    }
    return out;
}

std::vector<NamedStretch> namedStretches(const std::vector<ModelStreet>& streets, const std::string& roadPrefix) {
    std::vector<NamedStretch> out;
    for (const ModelStreet& s : streets)
        if (!s.name.empty()) out.push_back(NamedStretch{roadPrefix + s.road, s.from, s.to, s.name});
    return out;
}

NamingReport nameStreetRuns(std::vector<ModelStreet>& streets, const std::vector<ModelJunction>& junctions,
                            const std::vector<DividedPair>& divided, const RoadLabGraph& g, const StreetNameBook& book,
                            const std::vector<NamedStretch>& keep) {
    NamingReport rep;
    const std::vector<char> named = namedStreets(g, streets, book);
    const std::vector<std::vector<std::size_t>> runs = streetRuns(junctions, streets, divided, named);
    std::map<std::string, std::vector<const NamedStretch*>> keptOn;
    std::set<std::string> taken;
    for (const NamedStretch& k : keep) {
        keptOn[k.road].push_back(&k);
        taken.insert(k.name);
    }
    std::map<std::string, std::string> partner;
    for (const DividedPair& p : divided) {
        partner[p[0]] = p[1];
        partner[p[1]] = p[0];
    }
    auto travelWidth = [&](const std::string& road) {
        const EdgeSpec* e = g.find(road);
        return e ? e->lanes.travelWidth() : 0.0;
    };
    std::vector<std::string> runName(runs.size());
    std::vector<RunToName> toName;
    std::vector<std::size_t> toNameRun;
    for (std::size_t r = 0; r < runs.size(); ++r) {
        RunToName shape;
        double longest = -1, longestKept = -1;
        for (std::size_t i : runs[r]) {
            const ModelStreet& s = streets[i];
            const double len = s.to - s.from;
            shape.length += len;
            double w = travelWidth(s.road);
            const std::map<std::string, std::string>::const_iterator pt = partner.find(s.road);
            if (pt != partner.end()) w += travelWidth(pt->second);
            shape.width = std::max(shape.width, w);
            if (len > longest) {
                longest = len;
                const EdgeSpec* e = g.find(s.road);
                shape.roadClass = e ? e->cls : std::string();
            }
            const double mid = 0.5 * (s.from + s.to);
            const std::map<std::string, std::vector<const NamedStretch*>>::const_iterator kt = keptOn.find(s.road);
            if (kt != keptOn.end())
                for (const NamedStretch* k : kt->second)
                    if (atOrPast(mid, k->from, kWrittenStation) && atOrBefore(mid, k->to, kWrittenStation) && len > longestKept) {
                        longestKept = len;
                        runName[r] = k->name;
                    }
        }
        if (runName[r].empty()) {
            toName.push_back(shape);
            toNameRun.push_back(r);
        } else {
            ++rep.kept;
        }
    }
    const std::vector<std::string> fresh = nameRuns(toName, book, 0, taken);
    for (std::size_t k = 0; k < fresh.size(); ++k) runName[toNameRun[k]] = fresh[k];
    rep.generated = static_cast<uint32_t>(fresh.size());
    rep.runs = static_cast<uint32_t>(runs.size());
    // a street whose own stretch the file names keeps that name, even where a run now joins it to a street named
    // otherwise (ADR-0477: a junction stated as two can make a through pair of two differently named streets); the
    // run's name goes to the streets the file names not
    auto ownName = [&](const ModelStreet& s) {
        const double mid = 0.5 * (s.from + s.to);
        const std::map<std::string, std::vector<const NamedStretch*>>::const_iterator kt = keptOn.find(s.road);
        if (kt != keptOn.end())
            for (const NamedStretch* k : kt->second)
                if (atOrPast(mid, k->from, kWrittenStation) && atOrBefore(mid, k->to, kWrittenStation)) return k->name;
        return std::string();
    };
    std::vector<std::string> own(streets.size());
    for (std::size_t i = 0; i < streets.size(); ++i) own[i] = ownName(streets[i]);
    for (ModelStreet& s : streets) s.name.clear();
    for (std::size_t r = 0; r < runs.size(); ++r)
        for (std::size_t i : runs[r]) {
            streets[i].name = own[i].empty() ? runName[r] : own[i];
            ++rep.streets;
        }
    return rep;
}

NamingReport nameLayout(RoadLayout& layout, const RoadLabGraph& g, const std::vector<NamedStretch>& keep) {
    return nameStreetRuns(layout.streets, layout.junctions, layout.divided, g, streetNameBook(), keep);
}

// ONE MEMBER WRITTEN INTO EACH OBJECT OF A TOP-LEVEL LIST, in place: `member` with the value given for the object's id
// (in list order: `values` holds one entry per object, its id and the value, null for none), after its last member,
// unless it has the member already. Nothing else moves by a byte.
static std::string insertListMembersText(const std::string& text, const char* list, const char* member,
                                         const std::vector<std::pair<std::string, nlohmann::json>>& values, std::string* err) {
    auto fail = [&](const std::string& why) {
        if (err) *err = why;
        return std::string();
    };
    const Text t(text);
    std::vector<Member> top, sm;
    if (!t.members(t.ws(0), top)) return fail("the file is not a JSON object with members");
    const Member* listed = memberNamed(top, list);
    std::vector<std::size_t> elems;
    if (!listed || !t.elements(listed->valueBegin, elems)) return fail(std::string("the file has no \"") + list + "\" list");
    if (elems.size() != values.size()) return fail(std::string("the file's \"") + list + "\" list and the values given differ in length");
    std::vector<std::pair<std::size_t, std::string>> ins;
    for (std::size_t i = 0; i < elems.size(); ++i) {
        if (!t.members(elems[i], sm) || sm.empty()) return fail(std::string(list) + " " + std::to_string(i) + " is not an object");
        const Member* id = memberNamed(sm, "id");
        if (!id || text.substr(id->valueBegin, id->valueEnd - id->valueBegin) != nlohmann::json(values[i].first).dump())
            return fail(std::string(list) + " " + std::to_string(i) + " is not " + values[i].first + " in the file");
        if (memberNamed(sm, member) || values[i].second.is_null()) continue;
        ins.emplace_back(sm.back().valueEnd, "," + leadOf(text, sm.back()) + "\"" + member + "\": " + values[i].second.dump());
    }
    std::string out;
    out.reserve(text.size() + ins.size() * 32);
    std::size_t at = 0;
    for (const std::pair<std::size_t, std::string>& i : ins) {
        out.append(text, at, i.first - at);
        out += i.second;
        at = i.first;
    }
    out.append(text, at, kNone);
    if (keyedRoadFileText(out) != keyedRoadFileText(text)) return fail(std::string("the ") + member + "s, written in, change what the bake key hashes");
    return out;
}

std::string insertStreetNamesText(const std::string& text, const std::vector<ModelStreet>& streets, std::string* err) {
    std::vector<std::pair<std::string, nlohmann::json>> values;
    for (const ModelStreet& s : streets) values.emplace_back(s.id, s.name.empty() ? nlohmann::json() : nlohmann::json(s.name));
    return insertListMembersText(text, "streets", "name", values, err);
}

std::string insertJunctionControlsText(const std::string& text, const std::vector<std::pair<std::string, std::string>>& controls,
                                       std::string* err) {
    std::vector<std::pair<std::string, nlohmann::json>> values;
    for (const std::pair<std::string, std::string>& c : controls)
        values.emplace_back(c.first, c.second.empty() ? nlohmann::json() : nlohmann::json(c.second));
    return insertListMembersText(text, "junctions", "control", values, err);
}

// ---- a spread junction's points (ADR-0425) ------------------------------------------------------------------------------

std::vector<JunctionNodes> junctionNodes(const RoadLabGraph& g, LayoutSource source, std::vector<std::string>* notes) {
    std::vector<JunctionNodes> out;
    auto note = [&](const std::string& n) {
        if (notes) notes->push_back(n);
    };
    // a point (road, station) lies on a junction when an arm of it stands there or one of its inside stretches runs past it
    // (both written to the centimetre, as the nodes are: sameWrittenStation, the one answer)
    auto holds = [&](const ModelJunction& pj, const ModelNodePoint& p) {
        for (const ModelArm& a : pj.arms)
            if (a.road == p.road && sameWrittenStation(a.station, p.station)) return true;
        for (const ModelStretch& st : pj.inside)
            if (st.road == p.road && atOrPast(p.station, st.from, kWrittenStation) && atOrBefore(p.station, st.to, kWrittenStation)) return true;
        return false;
    };
    std::map<std::string, std::size_t> slot;   // junction id -> its entry in `out`
    for (const ModelJunction& pj : g.junctions)
        if (!pj.inside.empty()) {
            slot[pj.id] = out.size();
            out.push_back(JunctionNodes{pj.id, {}});
        }
    // the nodes as the layout finds them (control_plan.h findJunctions: the same road ends at one point, ramps' ends on a
    // street's side, crossings at grade) and the places where a street only carries on (a file whose junctions were grouped
    // before ADR-0419 holds some inside a junction), each placed in the one junction of the file that holds all its points
    std::vector<ModelNode> candidates;
    std::vector<CarryOn> carryOns;
    for (const JunctionFacts& f : layoutJunctionFacts(g, source, &carryOns))
        for (std::size_t n = 0; n < f.nodes.size() && n < f.nodePoints.size(); ++n)
            candidates.push_back(ModelNode{f.nodes[n], f.nodePoints[n]});
    for (const CarryOn& c : carryOns)
        candidates.push_back(ModelNode{c.at, {ModelNodePoint{c.roads[0], written(c.stations[0])}, ModelNodePoint{c.roads[1], written(c.stations[1])}}});
    auto samePoint = [&](const ModelNodePoint& a, const ModelNodePoint& b) { return a.road == b.road && sameWrittenStation(a.station, b.station); };
    for (const ModelNode& cand : candidates) {
        std::vector<const ModelJunction*> in;
        for (const ModelJunction& pj : g.junctions) {
            bool all = true;
            for (const ModelNodePoint& p : cand.points) all = all && holds(pj, p);
            if (all) in.push_back(&pj);
        }
        if (in.size() != 1) {
            // (a carry-on outside every junction is no point of one; a node no junction holds is a fault of the file's list)
            if (cand.points.size() == 2 && in.empty()) {
                bool isCarryOn = false;
                for (const CarryOn& c : carryOns) isCarryOn = isCarryOn || (c.roads[0] == cand.points[0].road && sameWrittenStation(written(c.stations[0]), cand.points[0].station));
                if (isCarryOn) continue;
            }
            char where[64];
            std::snprintf(where, sizeof where, "(%.2f, %.2f)", cand.at.x, cand.at.y);
            std::string pts;
            for (const ModelNodePoint& p : cand.points) pts += " " + p.road + "@" + std::to_string(p.station);
            note(std::string("the node at ") + where + " (" + pts + " ) " + (in.empty() ? "lies on no junction of the file" : "lies on several junctions of the file"));
            continue;
        }
        const std::map<std::string, std::size_t>::const_iterator it = slot.find(in.front()->id);
        if (it == slot.end()) continue;   // a junction at one point: its `at`
        // one node of the junction already holding one of its points (a crossing named where road ends already meet, found
        // twice) takes the rest of them: they are one place
        std::vector<ModelNode>& nodes = out[it->second].nodes;
        ModelNode* into = nullptr;
        for (ModelNode& n : nodes)
            for (const ModelNodePoint& p : n.points)
                for (const ModelNodePoint& q : cand.points)
                    if (!into && samePoint(p, q)) into = &n;
        if (!into) {
            nodes.push_back(ModelNode{Vec2(written(cand.at.x), written(cand.at.y)), {}});
            into = &nodes.back();
        }
        for (const ModelNodePoint& q : cand.points) {
            bool have = false;
            for (const ModelNodePoint& p : into->points) have = have || samePoint(p, q);
            if (!have) into->points.push_back(q);
        }
    }
    // (and two nodes a later one joined, through a point each shares with it, are one)
    for (JunctionNodes& jn : out)
        for (bool merged = true; merged;) {
            merged = false;
            for (std::size_t a = 0; a < jn.nodes.size() && !merged; ++a)
                for (std::size_t b = a + 1; b < jn.nodes.size() && !merged; ++b) {
                    bool shares = false;
                    for (const ModelNodePoint& p : jn.nodes[a].points)
                        for (const ModelNodePoint& q : jn.nodes[b].points) shares = shares || samePoint(p, q);
                    if (!shares) continue;
                    for (const ModelNodePoint& q : jn.nodes[b].points) {
                        bool have = false;
                        for (const ModelNodePoint& p : jn.nodes[a].points) have = have || samePoint(p, q);
                        if (!have) jn.nodes[a].points.push_back(q);
                    }
                    jn.nodes.erase(jn.nodes.begin() + static_cast<std::ptrdiff_t>(b));
                    merged = true;
                }
        }
    // every arm and every inside stretch's two ends stand at one of the nodes found
    for (const JunctionNodes& jn : out) {
        const ModelJunction* pj = nullptr;
        for (const ModelJunction& j : g.junctions)
            if (j.id == jn.junction) pj = &j;
        auto atNodes = [&](const std::string& road, double s) {
            int k = 0;
            for (const ModelNode& n : jn.nodes)
                for (const ModelNodePoint& p : n.points)
                    if (p.road == road && sameWrittenStation(p.station, s)) {
                        ++k;
                        break;
                    }
            return k;
        };
        for (const ModelArm& a : pj->arms)
            if (atNodes(a.road, a.station) != 1) {
                char buf[192];
                std::snprintf(buf, sizeof buf, "junction %s: its arm on %s at %.2f stands at %d of its nodes", jn.junction.c_str(), a.road.c_str(), a.station,
                              atNodes(a.road, a.station));
                note(buf);
            }
        for (const ModelStretch& st : pj->inside)
            for (double s : {st.from, st.to})
                if (atNodes(st.road, s) < 1) {
                    char buf[192];
                    std::snprintf(buf, sizeof buf, "junction %s: its stretch of %s inside it ends at %.2f, at none of its nodes", jn.junction.c_str(),
                                  st.road.c_str(), s);
                    note(buf);
                }
    }
    return out;
}

std::string insertJunctionNodesText(const std::string& text, const RoadLabGraph& g, const std::vector<JunctionNodes>& nodes, std::string* err) {
    std::map<std::string, const JunctionNodes*> of;
    for (const JunctionNodes& jn : nodes) of[jn.junction] = &jn;
    std::vector<std::pair<std::string, nlohmann::json>> values;
    for (const ModelJunction& pj : g.junctions) {
        const std::map<std::string, const JunctionNodes*>::const_iterator it = of.find(pj.id);
        values.emplace_back(pj.id, it == of.end() || it->second->nodes.empty() ? nlohmann::json() : nodesJson(it->second->nodes));
    }
    return insertListMembersText(text, "junctions", "nodes", values, err);
}

// ---- the order of a junction's arms round it (ADR-0427; road_layout.h) --------------------------------------------------

ComposedRoads::ComposedRoads(const RoadLabGraph& asRead) : g_(asRead) { (void)expand(g_); }

std::vector<std::size_t> armRing(const ModelJunction& pj, const ComposedRoads& roads, std::vector<std::string>* notes) {
    const RoadLabGraph& g = roads.graph();
    auto note = [&](const std::string& s) {
        if (notes) notes->push_back("junction " + pj.id + ": " + s);
    };
    const std::size_t n = pj.arms.size();
    std::vector<Vec2> mouth(n), way(n);
    std::vector<const EdgeSpec*> edge(n, nullptr);
    for (std::size_t i = 0; i < n; ++i) {
        const ModelArm& a = pj.arms[i];
        edge[i] = g.find(a.road);
        if (!edge[i] || edge[i]->xy.size() < 2) {
            mouth[i] = pj.at;
            way[i] = stated(a.heading) ? directionOf(a.heading) : Vec2(1, 0);
            note("arm " + std::to_string(i) + " (" + a.road + ") has no spine: measured by its " + (stated(a.heading) ? "stated heading" : "list place alone (east)"));
            continue;
        }
        mouth[i] = pointAt(edge[i]->xy, edge[i]->s, a.station);
        way[i] = stated(a.heading) ? directionOf(a.heading) : armDirection(*edge[i], a.station, a.spineIn);
    }
    double spread = 0, reach = 0;
    for (std::size_t i = 0; i < n; ++i) {
        reach = std::max(reach, (mouth[i] - pj.at).length());
        for (std::size_t k = i + 1; k < n; ++k) spread = std::max(spread, (mouth[i] - mouth[k]).length());
    }
    std::vector<double> key(n);
    for (std::size_t i = 0; i < n; ++i) {
        key[i] = degreesOf(way[i]);
        if (!(spread > kArmRingSpreadMouths) || !edge[i] || edge[i]->xy.size() < 2) continue;
        // spread: where the arm's road leaves the circle holding every mouth, walking its spine outward from the mouth
        const EdgeSpec& e = *edge[i];
        const ModelArm& a = pj.arms[i];
        const double R = reach + kArmRingSpreadMargin;
        const std::size_t m = e.xy.size();
        std::vector<Vec2> out{mouth[i]};
        if (a.spineIn) {
            for (std::size_t k = m; k-- > 0;)
                if (e.s[k] < a.station) out.push_back(e.xy[k]);
        } else {
            for (std::size_t k = 0; k < m; ++k)
                if (e.s[k] > a.station) out.push_back(e.xy[k]);
        }
        if (out.size() < 2) {
            note("arm " + std::to_string(i) + " (" + a.road + ") has no spine beyond its mouth: measured by its heading");
            continue;
        }
        bool left = false;
        Vec2 exitAt = out.back();
        for (std::size_t k = 0; k + 1 < out.size(); ++k) {
            const Vec2 p = out[k], q = out[k + 1];
            if ((q - pj.at).length() < R) continue;
            const Vec2 d = q - p, f = p - pj.at;
            const double A = dot(d, d), B = 2 * dot(f, d), C = dot(f, f) - R * R;
            const double disc = B * B - 4 * A * C;
            const double t = A > kArmRingNoLength2 && disc >= 0 ? std::clamp((-B + std::sqrt(disc)) / (2 * A), 0.0, 1.0) : 1.0;
            exitAt = p + d * t;
            left = true;
            break;
        }
        if (!left)
            note("arm " + std::to_string(i) + " (" + a.road + ") never leaves the circle of " + std::to_string(R) +
                 " m round the junction: measured by its road's far end");
        if ((exitAt - pj.at).length() > kArmRingAtMiddle) key[i] = degreesOf(exitAt - pj.at);
        else note("arm " + std::to_string(i) + " (" + a.road + ") leaves at the junction's middle: measured by its heading");
    }
    // by the measure alone (a strict order); arms measuring exactly the same keep their listed order, and are said
    std::vector<std::size_t> order(n);
    for (std::size_t i = 0; i < n; ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](std::size_t x, std::size_t y) { return key[x] < key[y]; });
    for (std::size_t k = 0; k + 1 < n; ++k)
        if (key[order[k]] == key[order[k + 1]])
            note("arms " + std::to_string(order[k]) + " (" + pj.arms[order[k]].road + ") and " + std::to_string(order[k + 1]) + " (" +
                 pj.arms[order[k + 1]].road + ") measure the same direction: kept in their listed order");
    return order;
}

void orderArmsRound(ModelJunction& pj, const ComposedRoads& roads, std::vector<std::string>* notes) {
    const std::vector<std::size_t> order = armRing(pj, roads, notes);
    std::vector<uint32_t> newIndex(pj.arms.size(), kNoIndex);
    std::vector<ModelArm> arms;
    for (std::size_t k = 0; k < order.size(); ++k) {
        newIndex[order[k]] = static_cast<uint32_t>(k);
        arms.push_back(pj.arms[order[k]]);
    }
    pj.arms.swap(arms);
    for (std::array<uint32_t, 2>& t : pj.through)
        for (uint32_t& a : t)
            if (a < newIndex.size()) a = newIndex[a];
}

bool ringDiffers(const ModelJunction& pj, const ComposedRoads& roads, std::vector<std::string>* notes) {
    const std::vector<std::size_t> order = armRing(pj, roads, notes);
    if (order.empty()) return false;
    const std::size_t start = static_cast<std::size_t>(std::find(order.begin(), order.end(), std::size_t(0)) - order.begin());
    for (std::size_t k = 0; k < order.size(); ++k)
        if (order[(start + k) % order.size()] != k) return true;
    return false;
}

ArmRingCheck checkArmRings(const RoadLabGraph& asRead) {
    ArmRingCheck c;
    const ComposedRoads roads(asRead);
    for (const ModelJunction& pj : asRead.junctions)
        if (ringDiffers(pj, roads, &c.notes)) c.differ.push_back(pj.id);
    return c;
}

std::string insertArmOrderText(const std::string& text, const RoadLabGraph& g, std::vector<std::string>* changed, std::string* err,
                               std::vector<std::string>* notes) {
    auto fail = [&](const std::string& why) {
        if (err) *err = why;
        return std::string();
    };
    const ComposedRoads roads(g);
    const Text t(text);
    std::vector<Member> top, jm;
    if (!t.members(t.ws(0), top)) return fail("the file is not a JSON object with members");
    const Member* listed = memberNamed(top, "junctions");
    std::vector<std::size_t> elems;
    if (!listed || !t.elements(listed->valueBegin, elems)) return fail("the file has no \"junctions\" list");
    if (elems.size() != g.junctions.size()) return fail("the file's \"junctions\" list and the graph read from it differ in length");
    std::vector<std::pair<std::pair<std::size_t, std::size_t>, std::string>> edits;   // (from, to) replaced by the text
    for (std::size_t j = 0; j < elems.size(); ++j) {
        const ModelJunction& pj = g.junctions[j];
        if (!t.members(elems[j], jm)) return fail("junction " + std::to_string(j) + " is not an object");
        const Member* id = memberNamed(jm, "id");
        if (!id || text.substr(id->valueBegin, id->valueEnd - id->valueBegin) != nlohmann::json(pj.id).dump())
            return fail("junction " + std::to_string(j) + " is not " + pj.id + " in the file");
        const std::vector<std::size_t> order = armRing(pj, roads, notes);
        bool same = true;
        for (std::size_t k = 0; k < order.size(); ++k) same = same && order[k] == k;
        if (same) continue;
        if (changed) changed->push_back(pj.id);
        const Member* arms = memberNamed(jm, "arms");
        std::vector<std::size_t> ae;
        if (!arms || !t.elements(arms->valueBegin, ae) || ae.size() != pj.arms.size())
            return fail("junction " + pj.id + ": its \"arms\" list does not fit the graph read from the file");
        // each arm's text moves whole into the slot of the arm it now follows
        for (std::size_t k = 0; k < ae.size(); ++k) {
            const std::size_t from = ae[order[k]];
            const std::string armText = text.substr(from, t.end(from) - from);
            edits.push_back({{ae[k], t.end(ae[k])}, armText});
        }
        const Member* through = memberNamed(jm, "through");
        if (through) {
            std::vector<uint32_t> newIndex(pj.arms.size(), kNoIndex);
            for (std::size_t k = 0; k < order.size(); ++k) newIndex[order[k]] = static_cast<uint32_t>(k);
            nlohmann::json pairs = nlohmann::json::array();
            for (const std::array<uint32_t, 2>& p : pj.through)
                pairs.push_back({p[0] < newIndex.size() ? newIndex[p[0]] : p[0], p[1] < newIndex.size() ? newIndex[p[1]] : p[1]});
            edits.push_back({{through->valueBegin, through->valueEnd}, pairs.dump()});
        }
    }
    // the file's word that its arms are so listed, beside the junctions list (true where it already states it)
    if (const Member* mark = memberNamed(top, "arms_in_ring")) edits.push_back({{mark->valueBegin, mark->valueEnd}, "true"});
    else edits.push_back({{listed->valueEnd, listed->valueEnd}, "," + leadOf(text, *listed) + "\"arms_in_ring\": true"});
    std::sort(edits.begin(), edits.end());
    std::string out;
    out.reserve(text.size());
    std::size_t at = 0;
    for (const std::pair<std::pair<std::size_t, std::size_t>, std::string>& e : edits) {
        out.append(text, at, e.first.first - at);
        out += e.second;
        at = e.first.second;
    }
    out.append(text, at, kNone);
    if (keyedRoadFileText(out) != keyedRoadFileText(text)) return fail("the arms, put in ring order, change what the bake key hashes");
    return out;
}

std::string insertArmText(const std::string& text, const RoadLabGraph& g, const std::string& junction, const std::string& road,
                          std::string* added, std::string* err) {
    auto fail = [&](const std::string& why) {
        if (err) *err = why;
        return std::string();
    };
    const ComposedRoads roads(g);
    const RoadLabGraph& cg = roads.graph();
    std::size_t j = 0;
    while (j < g.junctions.size() && g.junctions[j].id != junction) ++j;
    if (j == g.junctions.size()) return fail("the file lists no junction " + junction);
    ModelJunction pj = g.junctions[j];
    const EdgeSpec* e = cg.find(road);
    if (!e || e->xy.size() < 2) return fail("the file has no road " + road + " with a centre line");
    // the road's end at the junction: at its point, or at one of its points where it is spread
    std::vector<Vec2> places{pj.at};
    for (const ModelNode& n : pj.nodes) places.push_back(n.at);
    double nearest = std::numeric_limits<double>::infinity();
    int end = 0;
    for (int k = 0; k < 2; ++k)
        for (const Vec2& p : places) {
            const double d = distance(k ? e->xy.back() : e->xy.front(), p);
            if (d < nearest) {
                nearest = d;
                end = k;
            }
        }
    if (nearest > cg.rules.endpointTol) {
        char b[256];
        std::snprintf(b, sizeof b, "road %s ends %.2f m from junction %s's nearest point, more than the endpoint tolerance (%.2f m): "
                      "its centre line must reach the junction first", road.c_str(), nearest, junction.c_str(), cg.rules.endpointTol);
        return fail(b);
    }
    ModelArm arm;
    arm.road = road;
    arm.spineIn = end == 1;
    arm.station = end == 1 ? written(e->s.back()) : 0.0;
    arm.heading = degreesOf(armDirection(*e, arm.station, arm.spineIn));
    for (const ModelArm& a : pj.arms)
        if (a.road == road && a.spineIn == arm.spineIn) return fail("junction " + junction + " already states that end of " + road + " as an arm");
    const std::size_t fresh = pj.arms.size();
    pj.arms.push_back(arm);
    // its place in the ring: the stated arms keep their order, the new one goes where the measure puts it
    const std::vector<std::size_t> order = armRing(pj, roads);
    std::size_t place = 0;
    std::vector<uint32_t> newIndex(pj.arms.size(), kNoIndex);
    for (std::size_t k = 0, next = 0; k < order.size(); ++k) {
        newIndex[order[k]] = static_cast<uint32_t>(k);
        if (order[k] == fresh) {
            place = k;
            continue;
        }
        if (order[k] != next++) return fail("junction " + junction + "'s stated arms are not in the measure's ring order (" + kArmOrderCommand + ")");
    }
    // the through pairs: the one rule over the arms' headings in ring order; the stated pairs must be among them
    std::vector<Vec2> dirs(pj.arms.size());
    for (std::size_t k = 0; k < pj.arms.size(); ++k) {
        const ModelArm& a = pj.arms[k];
        const EdgeSpec* ae = cg.find(a.road);
        dirs[newIndex[k]] = stated(a.heading) ? directionOf(a.heading) : ae ? armDirection(*ae, a.station, a.spineIn) : Vec2(1, 0);
    }
    const ThroughPairs rule = throughPairs(dirs);
    auto ruleHas = [&](uint32_t a, uint32_t b) {
        for (const std::array<uint32_t, 2>& p : rule)
            if ((p[0] == a && p[1] == b) || (p[0] == b && p[1] == a)) return true;
        return false;
    };
    ThroughPairs through;
    for (const std::array<uint32_t, 2>& p : pj.through) {
        if (p[0] >= fresh || p[1] >= fresh) return fail("junction " + junction + " states a through pair naming no arm");
        through.push_back({newIndex[p[0]], newIndex[p[1]]});
        if (!ruleHas(through.back()[0], through.back()[1]))
            return fail("junction " + junction + "'s stated through pair " + std::to_string(p[0]) + "-" + std::to_string(p[1]) +
                        " is not one the rule gives over the arms' headings");
    }
    std::string pairsAdded;
    for (const std::array<uint32_t, 2>& p : rule)
        if (p[0] == place || p[1] == place) {
            through.push_back(p);
            pairsAdded += " " + std::to_string(p[0]) + "-" + std::to_string(p[1]);
        }
    // the text: the arm's own text at its place in the junction's "arms", and its "through" written again
    const Text t(text);
    std::vector<Member> top, jm, am;
    std::vector<std::size_t> elems, arms;
    const Member* listed = t.members(t.ws(0), top) ? memberNamed(top, "junctions") : nullptr;
    if (!listed || !t.elements(listed->valueBegin, elems) || elems.size() != g.junctions.size())
        return fail("the file's \"junctions\" list does not fit the graph read from it");
    if (!t.members(elems[j], jm)) return fail("junction " + junction + " is not an object in the file");
    const Member* armList = memberNamed(jm, "arms");
    if (!armList || !t.elements(armList->valueBegin, arms) || arms.size() != fresh || arms.empty() || !t.members(arms.front(), am) || am.empty())
        return fail("junction " + junction + ": its \"arms\" list does not fit the graph read from the file");
    // laid out as its neighbours are: the whitespace before an arm, and its members one indent step further in
    std::size_t leadStart = arms.front();
    while (leadStart > 0 && (text[leadStart - 1] == ' ' || text[leadStart - 1] == '\n' || text[leadStart - 1] == '\r' || text[leadStart - 1] == '\t')) --leadStart;
    const std::string elemLead = text.substr(leadStart, arms.front() - leadStart);
    const std::string elemIndent = elemLead.substr(elemLead.rfind('\n') == std::string::npos ? 0 : elemLead.rfind('\n') + 1);
    const std::string memberLead = leadOf(text, am.front());
    const std::size_t nl = memberLead.rfind('\n');
    const int step = nl == std::string::npos ? -1 : static_cast<int>(memberLead.size() - nl - 1) - static_cast<int>(elemIndent.size());
    const nlohmann::json armJson = {{"road", arm.road}, {"at", arm.station}, {"spine", arm.spineIn ? "in" : "out"}};
    std::string armText;
    for (char c : armJson.dump(step)) {
        armText += c;
        if (c == '\n') armText += elemIndent;
    }
    std::vector<std::pair<std::pair<std::size_t, std::size_t>, std::string>> edits;
    if (place < arms.size()) edits.push_back({{arms[place], arms[place]}, armText + "," + elemLead});
    else edits.push_back({{t.end(arms.back()), t.end(arms.back())}, "," + elemLead + armText});
    if (const Member* tm = memberNamed(jm, "through")) edits.push_back({{tm->valueBegin, tm->valueEnd}, nlohmann::json(through).dump()});
    else {
        const Member& after = jm.back().key == "nodes" && jm.size() > 1 ? jm[jm.size() - 2] : jm.back();
        edits.push_back({{after.valueEnd, after.valueEnd}, "," + leadOf(text, jm.back()) + "\"through\": " + nlohmann::json(through).dump()});
    }
    std::sort(edits.begin(), edits.end());
    std::string out;
    std::size_t at = 0;
    for (const std::pair<std::pair<std::size_t, std::size_t>, std::string>& ed : edits) {
        out.append(text, at, ed.first.first - at);
        out += ed.second;
        at = ed.first.second;
    }
    out.append(text, at, kNone);
    if (added) {
        char b[256];
        std::snprintf(b, sizeof b, "junction %s: the %s of %s (station %.2f, heading %.2f) is arm %zu of its %zu; through pairs added:%s",
                      junction.c_str(), arm.spineIn ? "end" : "start", road.c_str(), arm.station, arm.heading, place, pj.arms.size(),
                      pairsAdded.empty() ? " none" : pairsAdded.c_str());
        *added = b;
    }
    return out;
}

std::string insertSplitJunctionText(const std::string& text, const RoadLabGraph& g, const std::string& junction,
                                    const std::vector<std::string>& movingRoads, std::string* added, std::string* err) {
    auto fail = [&](const std::string& why) {
        if (err) *err = why;
        return std::string();
    };
    std::size_t j = 0;
    while (j < g.junctions.size() && g.junctions[j].id != junction) ++j;
    if (j == g.junctions.size()) return fail("the file lists no junction " + junction);
    const ModelJunction& J = g.junctions[j];
    const std::string newId = "j" + std::to_string(g.junctions.size());
    for (const ModelJunction& o : g.junctions)
        if (o.id == newId) return fail("the file already lists a junction " + newId);
    // the stated points: a point holding any of the moving roads goes to the new junction, the rest stay
    std::vector<ModelNode> keep, moved;
    for (const ModelNode& n : J.nodes) {
        bool moves = false;
        for (const ModelNodePoint& p : n.points)
            moves = moves || std::find(movingRoads.begin(), movingRoads.end(), p.road) != movingRoads.end();
        (moves ? moved : keep).push_back(n);
    }
    if (keep.empty() || moved.empty())
        return fail("junction " + junction + "'s stated points do not split in two by those roads (" + std::to_string(keep.size()) + " stay, " +
                    std::to_string(moved.size()) + " move)");
    // each part made as the grouping makes a junction (control_plan.h junctionOverNodes), its arms in their ring, its
    // through pairs by the one rule; the control, its reason and each arm's signs as the file states them for the whole
    const ComposedRoads roads(g);
    auto part = [&](const std::vector<ModelNode>& nodes, const std::string& id, ModelJunction& pj) {
        std::string why;
        const JunctionFacts f = junctionOverNodes(g, nodes, ControlPolicy(), &why);
        if (!why.empty()) return why;
        if (f.arms.size() < 3) return "part " + id + " would have " + std::to_string(f.arms.size()) + " arms: no junction";
        pj = junctionOf(f);
        pj.id = id;
        pj.control = J.control;
        pj.why = J.why;
        pj.hand = J.hand;
        pj.nodes = nodes.size() > 1 ? nodes : std::vector<ModelNode>();
        if (nodes.size() == 1) pj.at = nodes.front().at;
        for (ModelArm& a : pj.arms)
            for (const ModelArm& o : J.arms)
                if (o.road == a.road && o.spineIn == a.spineIn && sameWrittenStation(o.station, a.station)) a.regulations = o.regulations;
        orderArmsRound(pj, roads);
        return std::string();
    };
    ModelJunction pk, pm;
    std::string why = part(keep, J.id, pk);
    if (why.empty()) why = part(moved, newId, pm);
    if (!why.empty()) return fail("junction " + junction + ": " + why);
    // the text: the junction's element written again as the part that stays, the new part after the list's last element
    const Text t(text);
    std::vector<Member> top, am;
    std::vector<std::size_t> elems;
    const Member* listed = t.members(t.ws(0), top) ? memberNamed(top, "junctions") : nullptr;
    if (!listed || !t.elements(listed->valueBegin, elems) || elems.size() != g.junctions.size())
        return fail("the file's \"junctions\" list does not fit the graph read from it");
    if (!t.members(elems[j], am) || am.empty()) return fail("junction " + junction + " is not an object in the file");
    std::size_t leadStart = elems[j];
    while (leadStart > 0 && (text[leadStart - 1] == ' ' || text[leadStart - 1] == '\n' || text[leadStart - 1] == '\r' || text[leadStart - 1] == '\t')) --leadStart;
    const std::string elemLead = text.substr(leadStart, elems[j] - leadStart);
    const std::string elemIndent = elemLead.substr(elemLead.rfind('\n') == std::string::npos ? 0 : elemLead.rfind('\n') + 1);
    const std::string memberLead = leadOf(text, am.front());
    const std::size_t nl = memberLead.rfind('\n');
    const int step = nl == std::string::npos ? -1 : static_cast<int>(memberLead.size() - nl - 1) - static_cast<int>(elemIndent.size());
    // laid out as the layout writer leaves a junction: its members in order, then "through" and "nodes" on one line each
    auto elementText = [&](const ModelJunction& pj) {
        ModelJunction bare = pj;
        for (ModelArm& a : bare.arms) a.heading = kNotStated;
        nlohmann::json body = junctionJson(bare, {});
        body.erase("through");
        body.erase("nodes");
        std::string s;
        for (char c : body.dump(step)) {
            s += c;
            if (c == '\n') s += elemIndent;
        }
        std::string tail;
        if (!pj.through.empty()) tail += "," + memberLead + "\"through\": " + nlohmann::json(pj.through).dump();
        if (!pj.nodes.empty()) tail += "," + memberLead + "\"nodes\": " + nodesJson(pj.nodes).dump();
        const std::size_t close = s.rfind(nl == std::string::npos ? "}" : "\n");
        return s.substr(0, close) + tail + s.substr(close);
    };
    const std::size_t lastEnd = t.end(elems.back());
    std::string out = text.substr(0, elems[j]) + elementText(pk) + text.substr(t.end(elems[j]), lastEnd - t.end(elems[j])) + "," + elemLead +
                      elementText(pm) + text.substr(lastEnd);
    if (added) {
        std::string words;
        for (const ModelJunction* pj : {&pk, &pm}) {
            words += (words.empty() ? "" : "; ") + pj->id + ":";
            for (const ModelArm& a : pj->arms) words += " " + a.road + (a.spineIn ? "(in)" : "(out)");
            words += ", through";
            for (const std::array<uint32_t, 2>& p : pj->through) words += " " + std::to_string(p[0]) + "-" + std::to_string(p[1]);
            for (const ModelStretch& s : pj->inside) words += ", inside " + s.road;
        }
        *added = words;
    }
    return out;
}

std::string setEdgeMemberText(const std::string& text, const std::string& edge, const std::vector<std::string>& keyPath,
                              const nlohmann::json& value, std::string* err) {
    auto fail = [&](const std::string& why) {
        if (err) *err = why;
        return std::string();
    };
    if (keyPath.empty()) return fail("no member named");
    const Text t(text);
    std::vector<Member> top, ms;
    std::vector<std::size_t> elems;
    if (!t.members(t.ws(0), top) || top.empty()) return fail("the file is not a JSON object with members");
    const Member* edges = memberNamed(top, "edges");
    if (!edges || !t.elements(edges->valueBegin, elems)) return fail("the file has no \"edges\" list");
    const std::string id = nlohmann::json(edge).dump();
    std::size_t open = kNone;
    for (std::size_t e : elems) {
        if (!t.members(e, ms)) continue;
        const Member* m = memberNamed(ms, "id");
        if (m && text.compare(m->valueBegin, m->valueEnd - m->valueBegin, id) == 0) {
            open = e;
            break;
        }
    }
    if (open == kNone) return fail("the file has no edge " + edge);
    const Member* m = nullptr;
    for (std::size_t k = 0; k < keyPath.size(); ++k) {
        if (!t.members(open, ms) || !(m = memberNamed(ms, keyPath[k]))) return fail("edge " + edge + " has no member " + keyPath[k]);
        open = m->valueBegin;
    }
    // laid out as the file lays out its own: one indent step is the top level's indent, each line after the first moved in
    // by the member's own indent (on one line in a file written on one line)
    const std::string lead = leadOf(text, *m), topLead = leadOf(text, top.front());
    const std::size_t nl = lead.rfind('\n'), topNl = topLead.rfind('\n');
    std::string valueText;
    if (nl == std::string::npos || topNl == std::string::npos) valueText = value.dump();
    else {
        const std::string indent = lead.substr(nl + 1);
        for (char c : value.dump(std::max<int>(1, static_cast<int>(topLead.size() - topNl - 1)))) {
            valueText += c;
            if (c == '\n') valueText += indent;
        }
    }
    return text.substr(0, m->valueBegin) + valueText + text.substr(m->valueEnd);
}

}  // namespace roads::lanes
}  // namespace engine
