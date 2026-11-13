#include "engine/procgen/city/roads/lanes/road_layout.h"

#include "engine/procgen/city/roads/lanes/control_plan.h"        // findJunctions: the nodes and their grouping, once
#include "engine/procgen/city/roads/lanes/lane_expand.h"         // expand: ramps composed as the builder composes them
#include "engine/procgen/city/roads/lanes/lane_graph_connect.h"  // kStraightOnDegrees: the one "straight on"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/road_design.h"         // planCrossings: the planner's crossing policy
#include "rt_math.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <tuple>

namespace engine {
namespace roads::lanes {

namespace {

// Stations, points and headings are written to the centimetre (or the hundredth of a degree), so a file reads cleanly
// and the same roads write the same bytes.
double written(double v) { return std::round(v * 100.0) / 100.0; }

// Two stations written from one number are equal; this only absorbs the last bit of a sum.
constexpr double kSameStation = 1e-6;

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

// The way an arm leaves its junction: from its station, kArmHeadingReach along the road away from the junction.
Vec2 armDirection(const EdgeSpec& e, double at, bool spineIn) {
    const double length = e.s.back();
    const double far = spineIn ? std::max(0.0, at - kArmHeadingReach) : std::min(length, at + kArmHeadingReach);
    Vec2 d = pointAt(e.xy, e.s, far) - pointAt(e.xy, e.s, at);
    if (d.length() < 1e-9) {
        const Vec2 t = tangentAtStation(e.xy, e.s, at);
        d = spineIn ? t * -1.0 : t;
    }
    return d.length() < 1e-9 ? Vec2(1, 0) : normalize(d);
}

double degreesOf(const Vec2& d) {
    double a = written(std::atan2(d.y, d.x) * RAD_TO_DEG);
    if (a < 0) a += 360.0;
    if (a >= 360.0) a -= 360.0;
    return a;
}

Vec2 directionOf(double degrees) { return Vec2(std::cos(degrees * DEG_TO_RAD), std::sin(degrees * DEG_TO_RAD)); }

// The pairs of arms that are one street carrying on through (see the top of road_layout.h).
std::vector<std::array<int, 2>> throughPairs(const std::vector<Vec2>& dirs) {
    std::vector<std::array<int, 2>> out;
    if (dirs.size() == 2) return {{0, 1}};
    const double headOn = -std::cos(kStraightOnDegrees * DEG_TO_RAD);
    std::vector<std::tuple<double, int, int>> pairs;
    for (std::size_t i = 0; i < dirs.size(); ++i)
        for (std::size_t k = i + 1; k < dirs.size(); ++k) {
            const double d = dot(dirs[i], dirs[k]);
            if (d < headOn) pairs.emplace_back(d, static_cast<int>(i), static_cast<int>(k));
        }
    std::sort(pairs.begin(), pairs.end());
    std::vector<char> used(dirs.size(), 0);
    for (const std::tuple<double, int, int>& p : pairs) {
        const int a = std::get<1>(p), b = std::get<2>(p);
        if (used[static_cast<std::size_t>(a)] || used[static_cast<std::size_t>(b)]) continue;
        used[static_cast<std::size_t>(a)] = used[static_cast<std::size_t>(b)] = 1;
        out.push_back({a, b});
    }
    std::sort(out.begin(), out.end());
    return out;
}

nlohmann::json junctionJson(const PlannedJunction& pj) {
    nlohmann::json j;
    j["id"] = pj.id;
    j["at"] = {pj.at.x, pj.at.y};
    if (!pj.control.empty()) j["control"] = pj.control;
    if (!pj.why.empty()) j["why"] = pj.why;
    if (pj.hand) j["hand"] = true;
    nlohmann::json arms = nlohmann::json::array();
    for (const PlannedArm& a : pj.arms) {
        nlohmann::json aj;
        aj["road"] = a.road;
        aj["at"] = a.at;
        aj["spine"] = a.spineIn ? "in" : "out";
        if (!a.regulations.empty()) aj["regulations"] = a.regulations;
        if (!a.street.empty()) aj["street"] = a.street;
        if (a.hasHeading) aj["heading"] = a.heading;
        arms.push_back(aj);
    }
    j["arms"] = arms;
    if (!pj.inside.empty()) {
        nlohmann::json inside = nlohmann::json::array();
        for (const PlannedStretch& st : pj.inside) inside.push_back({{"road", st.road}, {"from", st.from}, {"to", st.to}});
        j["inside"] = inside;
    }
    if (!pj.through.empty()) j["through"] = pj.through;
    return j;
}

nlohmann::json streetJson(const PlannedStreet& st) {
    nlohmann::json ends = nlohmann::json::array();
    for (const std::string& j : st.junctions) ends.push_back(j.empty() ? nlohmann::json() : nlohmann::json(j));
    return {{"id", st.id}, {"road", st.road}, {"from", st.from}, {"to", st.to}, {"junctions", ends}};
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

RoadLayout deriveLayout(const RoadLabGraph& graph, LayoutSource source) {
    RoadLayout out;
    RoadLabGraph g = graph;
    (void)expand(g);   // the ramps composed from their anchors: the builder's stations
    // ---- the junctions: the graph's own list as it states them, or found ----------------------------------------------
    if (graph.listsJunctions) {
        out.junctions = graph.junctions;
        out.keptList = true;
    } else {
        // two roads meeting away from a shared end: as the graph declares them, else as the planner's crossing policy has
        // them (planCrossings: the planner's own, which city_plan's design step writes into the scene as its crossings)
        const std::vector<CrossingSpec> crossings = graph.declaresCrossings ? graph.crossings : crossingsFromJson(planCrossings(graph));
        JunctionFinding how;
        how.endTolerance = source == LayoutSource::Planner ? kPlannerSameEnd : -1.0;
        how.atGrade = &crossings;
        for (const JunctionFacts& f : findJunctions(graph, ControlPolicy(), how)) {
            PlannedJunction pj;
            pj.id = f.id;
            pj.at = Vec2(written(f.at.x), written(f.at.y));
            for (const ArmFacts& a : f.arms) {
                PlannedArm arm;
                arm.road = a.road;
                arm.at = written(a.at);
                arm.spineIn = a.spineIn;
                pj.arms.push_back(arm);
            }
            pj.inside = f.inside;
            out.junctions.push_back(pj);
        }
    }
    // ---- each arm's heading; the arms in order round a junction found here; the through pairs -------------------------
    for (PlannedJunction& pj : out.junctions) {
        std::vector<Vec2> dirs;
        for (PlannedArm& a : pj.arms) {
            const EdgeSpec* e = g.find(a.road);
            if (!e || e->xy.size() < 2) {
                dirs.push_back(Vec2(1, 0));
                out.notes.push_back("junction " + pj.id + ": arm on road " + a.road + " has no spine to measure");
                continue;
            }
            if (!a.hasHeading) {
                a.hasHeading = true;
                a.heading = degreesOf(armDirection(*e, a.at, a.spineIn));
            }
            dirs.push_back(directionOf(a.heading));
        }
        if (!out.keptList) {
            std::vector<std::size_t> order(pj.arms.size());
            for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
            std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return pj.arms[a].heading < pj.arms[b].heading; });
            std::vector<PlannedArm> arms;
            std::vector<Vec2> d;
            for (std::size_t i : order) {
                arms.push_back(pj.arms[i]);
                d.push_back(dirs[i]);
            }
            pj.arms.swap(arms);
            dirs.swap(d);
        }
        if (pj.through.empty()) pj.through = throughPairs(dirs);
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
        for (const PlannedArm& a : out.junctions[j].arms) cutsOf[a.road].push_back(Cut{a.at, static_cast<int>(j), a.spineIn});
        for (const PlannedStretch& st : out.junctions[j].inside) insideOf[st.road].emplace_back(st.from, st.to);
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
        at.erase(std::unique(at.begin(), at.end(), [](double a, double b) { return std::fabs(a - b) < kSameStation; }), at.end());
        std::vector<PlannedStreet> mine;
        for (std::size_t k = 0; k + 1 < at.size(); ++k) {
            const double from = std::max(0.0, at[k]), to = std::min(length, at[k + 1]);
            if (to - from < kShortestStreet) continue;
            const double mid = 0.5 * (from + to);
            bool inside = false;
            for (const std::pair<double, double>& in : insideOf[e.id]) inside = inside || (mid >= in.first && mid <= in.second);
            if (inside) continue;
            PlannedStreet st;
            st.road = e.id;
            st.from = from;
            st.to = to;
            for (const Cut& c : cutsOf[e.id]) {
                if (!c.spineIn && std::fabs(c.s - from) < kSameStation) st.junctions[0] = out.junctions[static_cast<std::size_t>(c.junction)].id;
                if (c.spineIn && std::fabs(c.s - to) < kSameStation) st.junctions[1] = out.junctions[static_cast<std::size_t>(c.junction)].id;
            }
            mine.push_back(st);
        }
        for (std::size_t k = 0; k < mine.size(); ++k) {
            mine[k].id = mine.size() == 1 ? e.id : e.id + "/" + std::to_string(k);
            out.streets.push_back(mine[k]);
        }
    }
    // ---- each arm's street ---------------------------------------------------------------------------------------------
    std::map<std::string, std::vector<std::size_t>> streetsOf;
    for (std::size_t i = 0; i < out.streets.size(); ++i) streetsOf[out.streets[i].road].push_back(i);
    for (PlannedJunction& pj : out.junctions)
        for (PlannedArm& a : pj.arms) {
            a.street.clear();
            for (std::size_t i : streetsOf[a.road]) {
                const PlannedStreet& st = out.streets[i];
                if (std::fabs((a.spineIn ? st.to : st.from) - a.at) < kSameStation) a.street = st.id;
            }
            if (a.street.empty()) {
                char buf[256];
                std::snprintf(buf, sizeof buf, "junction %s: its arm on road %s (spine %s at %.2f m) is no street's end", pj.id.c_str(),
                              a.road.c_str(), a.spineIn ? "in" : "out", a.at);
                out.notes.push_back(buf);
            }
        }
    return out;
}

std::string describeLayout(const RoadLayout& layout) {
    std::map<std::size_t, uint32_t> byArms;
    uint32_t through = 0;
    for (const PlannedJunction& pj : layout.junctions) {
        ++byArms[pj.arms.size()];
        through += static_cast<uint32_t>(pj.through.size());
    }
    uint32_t deadEnds = 0, bothJunctions = 0;
    for (const PlannedStreet& st : layout.streets) {
        deadEnds += (st.junctions[0].empty() ? 1u : 0u) + (st.junctions[1].empty() ? 1u : 0u);
        bothJunctions += !st.junctions[0].empty() && !st.junctions[1].empty() ? 1u : 0u;
    }
    char buf[256];
    std::snprintf(buf, sizeof buf, "%zu junctions (%s), %zu streets (%u with a junction at both ends, %u ends at no junction), %u through pairs",
                  layout.junctions.size(), layout.keptList ? "the graph's own list, kept" : "found", layout.streets.size(), bothJunctions,
                  deadEnds, through);
    std::string out = buf;
    out += "\n  junctions by arms:";
    for (const std::pair<const std::size_t, uint32_t>& b : byArms) out += " " + std::to_string(b.second) + " with " + std::to_string(b.first) + ";";
    out += "\n";
    for (const std::string& n : layout.notes) out += "  note: " + n + "\n";
    return out;
}

void writeLayout(nlohmann::json& scene, const RoadLayout& layout) {
    nlohmann::json junctions = nlohmann::json::array(), streets = nlohmann::json::array();
    for (const PlannedJunction& pj : layout.junctions) junctions.push_back(junctionJson(pj));
    for (const PlannedStreet& st : layout.streets) streets.push_back(streetJson(st));
    scene["junctions"] = junctions;
    scene["streets"] = streets;
}

std::string stripLayoutText(const std::string& text) {
    const Text t(text);
    const std::size_t open = t.ws(0);
    std::vector<Member> top;
    if (!t.members(open, top)) return text;
    std::vector<std::pair<std::size_t, std::size_t>> cut;
    std::vector<Member> jm, am;
    std::vector<std::size_t> elems, arms;
    for (std::size_t k = 0; k < top.size(); ++k) {
        const Member& m = top[k];
        if (m.key == "streets") cut.push_back(removal(top, k));
        if (m.key != "junctions" || !t.elements(m.valueBegin, elems)) continue;
        bool controls = false;
        for (std::size_t e : elems)
            if (t.members(e, jm) && memberNamed(jm, "control")) controls = true;
        if (!controls) {
            cut.push_back(removal(top, k));
            continue;
        }
        for (std::size_t e : elems) {
            if (!t.members(e, jm)) continue;
            for (std::size_t q = 0; q < jm.size(); ++q) {
                if (jm[q].key == "through") cut.push_back(removal(jm, q));
                if (jm[q].key != "arms" || !t.elements(jm[q].valueBegin, arms)) continue;
                for (std::size_t a : arms) {
                    if (!t.members(a, am)) continue;
                    for (std::size_t r = 0; r < am.size(); ++r)
                        if (am[r].key == "street" || am[r].key == "heading") cut.push_back(removal(am, r));
                }
            }
        }
    }
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

std::string insertLayoutText(const std::string& text, const RoadLayout& layout, std::string* err) {
    auto fail = [&](const std::string& why) {
        if (err) *err = why;
        return std::string();
    };
    const Text t(text);
    std::vector<Member> top;
    if (!t.members(t.ws(0), top) || top.empty()) return fail("the file is not a JSON object with members");
    if (memberNamed(top, "streets")) return fail("the file already has a \"streets\" list (strip it first)");
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
            const PlannedJunction& pj = layout.junctions[i];
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
                if (!memberNamed(am, "street") && !pj.arms[a].street.empty()) add += "," + armLead + "\"street\": " + nlohmann::json(pj.arms[a].street).dump();
                if (!memberNamed(am, "heading") && pj.arms[a].hasHeading) add += "," + armLead + "\"heading\": " + nlohmann::json(pj.arms[a].heading).dump();
                if (!add.empty()) ins.emplace_back(am.back().valueEnd, add);
            }
            if (!memberNamed(jm, "through") && !pj.through.empty())
                ins.emplace_back(jm.back().valueEnd, "," + leadOf(text, jm.back()) + "\"through\": " + nlohmann::json(pj.through).dump());
        }
    } else {
        nlohmann::json junctions = nlohmann::json::array();
        for (const PlannedJunction& pj : layout.junctions) junctions.push_back(junctionJson(pj));
        tail += "," + lead + "\"junctions\": " + valueText(junctions);
    }
    nlohmann::json streets = nlohmann::json::array();
    for (const PlannedStreet& st : layout.streets) streets.push_back(streetJson(st));
    tail += "," + lead + "\"streets\": " + valueText(streets);
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
    if (stripLayoutText(out) != text) return fail("the layout, written in, does not strip back to the file it was written into");
    return out;
}

}  // namespace roads::lanes
}  // namespace engine
