#include "engine/procgen/city/roads/lanes/control_plan.h"

#include "engine/procgen/city/roads/lanes/lane_expand.h"          // expand: ramps composed as the builder composes them
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"     // kRampEndReach: how far a ramp runs on inside a street
#include "engine/procgen/city/roads/lanes/lane_graph_connect.h"   // kStraightOnDegrees: the one "straight on"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/road_layout.h"          // armRing: the arms written in their ring (ADR-0427)
#include "rt_math.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <tuple>

namespace engine {
namespace roads::lanes {

namespace {

// Stations and points are written to the road graph to the centimetre, so the file reads cleanly and a re-plan of the
// same roads writes the same bytes.
constexpr double kWrittenPrecision = 100.0;

std::string fmt(const char* f, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}

// A vector with no length to speak of: no direction can be read from it.
constexpr double kNoDirection = 1e-9;

bool carsDrive(const RoadLabGraph& g, const EdgeSpec& e) {
    const RoadClassSpec& c = g.cls(e);
    return c.carriesCars() && e.laneCount() > 0 && e.xy.size() >= 2 && !e.s.empty() && e.s.back() > 0;
}

// A ramp's end that is anchored to the side of a street by name alone (no gore on a host lane): where the ramp ends on
// that street. `end` 0 is the ramp's start, 1 its end.
const RampAnchor* streetEnd(const EdgeSpec& e, int end) {
    const RampAnchor& a = end == 0 ? e.from : e.to;
    return !a.set && !a.edge.empty() ? &a : nullptr;
}

// One road at one node: the road (index) and the station along its spine.
struct NodePoint {
    int edge = -1;
    double s = 0;
};
struct Node {
    Vec2 at;
    std::vector<NodePoint> points;
    bool rampEnd = false;
    int ramp = -1;   // the ramp whose free end it is
    bool crossing = false;   // two roads crossing at grade (the caller's), not road ends meeting
};

int findRoot(std::vector<int>& parent, int i) {
    while (parent[static_cast<std::size_t>(i)] != i) {
        parent[static_cast<std::size_t>(i)] = parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(i)])];
        i = parent[static_cast<std::size_t>(i)];
    }
    return i;
}
void unite(std::vector<int>& parent, int a, int b) {
    a = findRoot(parent, a);
    b = findRoot(parent, b);
    if (a != b) parent[static_cast<std::size_t>(std::max(a, b))] = std::min(a, b);
}

bool listed(const std::vector<std::string>& names, const std::string& name) {
    return std::find(names.begin(), names.end(), name) != names.end();
}

void count(std::vector<std::pair<std::string, uint32_t>>& tally, const std::string& name) {
    for (std::pair<std::string, uint32_t>& t : tally)
        if (t.first == name) {
            ++t.second;
            return;
        }
    tally.emplace_back(name, 1);
}

}  // namespace

double written(double v) { return std::round(v * kWrittenPrecision) / kWrittenPrecision; }

Vec2 armDirection(const EdgeSpec& e, double at, bool spineIn) {
    const double length = e.s.back();
    const double far = spineIn ? std::max(0.0, at - kArmHeadingReach) : std::min(length, at + kArmHeadingReach);
    Vec2 d = pointAt(e.xy, e.s, far) - pointAt(e.xy, e.s, at);
    if (d.length() < kNoDirection) {
        const Vec2 t = tangentAtStation(e.xy, e.s, at);
        d = spineIn ? t * -1.0 : t;
    }
    return d.length() < kNoDirection ? Vec2(1, 0) : normalize(d);
}

double degreesOf(const Vec2& d) {
    double a = written(std::atan2(d.y, d.x) * RAD_TO_DEG);
    if (a < 0) a += 360.0;
    if (a >= 360.0) a -= 360.0;
    return a;
}

ThroughPairs throughPairs(const std::vector<Vec2>& dirs) {
    ThroughPairs out;
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
        out.push_back({static_cast<uint32_t>(a), static_cast<uint32_t>(b)});
    }
    std::sort(out.begin(), out.end());
    return out;
}

namespace {

// THE LIMIT A ROAD POSTS: its own where a person set it, else what the policy posts on its class (0: none).
double postedLimitOn(const EdgeSpec& e, const ControlPolicy& policy) {
    return e.speedLimitHand ? e.speedLimit : policy.postedLimit(e.cls);
}

// One arm of road `e` at station `at`, as the policy sees it: the lanes arriving and leaving by it follow from the way
// its spine runs there.
ArmFacts armFacts(const EdgeSpec& e, double at, bool spineIn, const ControlPolicy& policy) {
    ArmFacts a;
    a.road = e.id;
    a.roadClass = e.cls;
    a.at = at;
    a.spineIn = spineIn;
    a.arriving = spineIn ? e.lanes.fwd : e.lanes.back;
    a.leaving = spineIn ? e.lanes.back : e.lanes.fwd;
    a.limit = postedLimitOn(e, policy);
    return a;
}

// The through pairs of a junction just found, as the one rule states them from its arms' headings as written (the
// pairs the layout writes into the file for it). `g`: the graph with its ramps composed.
ThroughPairs statedThrough(const RoadLabGraph& g, const std::vector<ArmFacts>& arms) {
    std::vector<Vec2> dirs;
    for (const ArmFacts& a : arms) {
        const EdgeSpec* e = g.find(a.road);
        dirs.push_back(e && e->xy.size() >= 2 ? directionOf(degreesOf(armDirection(*e, written(a.at), a.spineIn))) : Vec2(1, 0));
    }
    return throughPairs(dirs);
}

// THE ARMS OF A JUNCTION OVER THESE POINTS: `stations` is each road's stations at the junction's points (road index ->
// stations). A road has an arm before its lowest station there (its spine runs in) and one after its highest (its spine
// runs out), each where the road reaches more than kArmReachesPast beyond. `ramps`: the roads that are ramps ending
// here. `inside` (optional) gets the stretches of road lying between the points. `g`: the graph with its ramps composed.
std::vector<ArmFacts> armsOver(const RoadLabGraph& g, const std::map<int, std::vector<double>>& stations,
                               const std::set<int>& ramps, const ControlPolicy& policy, std::vector<ModelStretch>* inside) {
    std::vector<ArmFacts> arms;
    for (const std::pair<const int, std::vector<double>>& kv : stations) {
        const EdgeSpec& e = g.edges[static_cast<std::size_t>(kv.first)];
        const double lo = *std::min_element(kv.second.begin(), kv.second.end());
        const double hi = *std::max_element(kv.second.begin(), kv.second.end());
        if (inside && hi - lo > kArmReachesPast) inside->push_back(ModelStretch{e.id, written(lo), written(hi)});
        for (int side = 0; side < 2; ++side) {
            // side 0: the road before the junction (its spine runs in); side 1: after it (its spine runs out)
            const bool in = side == 0;
            if (in ? lo <= kArmReachesPast : hi >= e.s.back() - kArmReachesPast) continue;
            ArmFacts a = armFacts(e, in ? lo : hi, in, policy);
            a.ramp = ramps.count(kv.first) > 0;
            arms.push_back(a);
        }
    }
    return arms;
}

}  // namespace

bool FactRange::holds(bool hasValue, double value) const {
    if (!set) return true;
    return hasValue && value >= lo && value <= hi;
}

uint32_t ControlPolicy::classRank(const std::string& roadClass) const {
    for (std::size_t i = 0; i < classOrder.size(); ++i)
        if (classOrder[i] == roadClass) return static_cast<uint32_t>(i);
    return static_cast<uint32_t>(classOrder.size());
}

double ControlPolicy::postedLimit(const std::string& roadClass) const {
    for (const std::pair<std::string, double>& l : speedLimits)
        if (l.first == roadClass) return l.second;
    return 0.0;
}

void settleRoles(JunctionFacts& j, const ControlPolicy& policy) {
    std::vector<std::size_t> app;   // the approaches, in arm order
    for (std::size_t a = 0; a < j.arms.size(); ++a) {
        j.arms[a].major = false;
        if (j.arms[a].arriving > 0) app.push_back(a);
    }
    j.approaches = static_cast<int>(app.size());
    j.hasMinor = false;
    j.majorClass.clear();
    j.minorClass.clear();
    j.majorLanes = j.minorLanes = 0;
    j.majorLimit = j.minorLimit = 0;
    if (app.empty()) return;
    uint32_t best = std::numeric_limits<uint32_t>::max(), worst = 0;
    for (std::size_t a : app) {
        best = std::min(best, policy.classRank(j.arms[a].roadClass));
        worst = std::max(worst, policy.classRank(j.arms[a].roadClass));
    }
    if (best != worst) {
        for (std::size_t a : app) j.arms[a].major = policy.classRank(j.arms[a].roadClass) == best;
    } else {
        // Every approach on one class: the one road running straight through, if there is exactly one, is the major road
        // -- a pair of arms the junction STATES as one road carrying on (`through`: read, never measured here), at least
        // one of them arriving (a ONE-WAY road runs through on one approach and one way out: a frontage road, ADR-0413),
        // with an approach besides it.
        std::vector<std::pair<std::size_t, std::size_t>> through;
        for (const std::array<uint32_t, 2>& t : j.through)
            if (t[0] < j.arms.size() && t[1] < j.arms.size() && (j.arms[t[0]].arriving > 0 || j.arms[t[1]].arriving > 0))
                through.emplace_back(t[0], t[1]);
        bool besides = false;
        for (std::size_t a : app) besides = besides || (through.size() == 1 && a != through[0].first && a != through[0].second);
        const bool oneThrough = through.size() == 1 && besides;
        for (std::size_t a : app)
            j.arms[a].major = !oneThrough || a == through[0].first || a == through[0].second;
    }
    for (std::size_t a : app) j.hasMinor = j.hasMinor || !j.arms[a].major;
    // The minor road's facts come from the minor approaches; with none, from every approach (a crossroads of equals:
    // its "minor road" is the same class as its major one).
    uint32_t minorRank = 0;
    bool anyMinor = false;
    for (std::size_t a : app) {
        const ArmFacts& arm = j.arms[a];
        if (arm.major) {
            j.majorClass = arm.roadClass;
            j.majorLanes = std::max(j.majorLanes, arm.arriving);
            j.majorLimit = std::max(j.majorLimit, arm.limit);
        }
        if (!arm.major || !j.hasMinor) {
            const uint32_t r = policy.classRank(arm.roadClass);
            if (!anyMinor || r > minorRank) {
                minorRank = r;
                j.minorClass = arm.roadClass;
            }
            anyMinor = true;
            j.minorLanes = std::max(j.minorLanes, arm.arriving);
            j.minorLimit = std::max(j.minorLimit, arm.limit);
        }
    }
}

std::string describeFacts(const JunctionFacts& j) {
    auto road = [](const std::string& cls, int lanes, double limit) {
        return fmt("%s (%d lane%s, %s)", cls.c_str(), lanes, lanes == 1 ? "" : "s",
                   limit > 0 ? fmt("%.1f m/s", limit).c_str() : "no limit posted");
    };
    std::string out = fmt("%zu arms (%d approaches)%s: major %s", j.arms.size(), j.approaches,
                          j.rampEnd ? ", a ramp's end" : "", road(j.majorClass, j.majorLanes, j.majorLimit).c_str());
    out += j.hasMinor ? ", minor " + road(j.minorClass, j.minorLanes, j.minorLimit)
                      : ", no minor road (the least of them " + road(j.minorClass, j.minorLanes, j.minorLimit) + ")";
    return out;
}

ControlChoice chooseControl(const JunctionFacts& j, const ControlPolicy& policy, const std::vector<Decided>& before) {
    ControlChoice out;
    for (std::size_t r = 0; r < policy.rows.size(); ++r) {
        const PolicyRow& row = policy.rows[r];
        if (!row.arms.holds(true, static_cast<double>(j.arms.size()))) continue;
        if (!row.approaches.holds(true, static_cast<double>(j.approaches))) continue;
        if (!row.major.empty() && (j.approaches == 0 || !listed(row.major, j.majorClass))) continue;
        if (!row.minor.empty() && (j.approaches == 0 || !listed(row.minor, j.minorClass))) continue;
        if (row.hasMinor != YesNo::Any && (row.hasMinor == YesNo::Yes) != j.hasMinor) continue;
        if (row.rampEnd != YesNo::Any && (row.rampEnd == YesNo::Yes) != j.rampEnd) continue;
        if (!row.majorLanes.holds(j.approaches > 0, j.majorLanes)) continue;
        if (!row.minorLanes.holds(j.approaches > 0, j.minorLanes)) continue;
        if (!row.majorLimit.holds(j.majorLimit > 0, j.majorLimit)) continue;
        if (!row.minorLimit.holds(j.minorLimit > 0, j.minorLimit)) continue;
        if (row.apart > 0) {
            bool near = false;
            for (const Decided& d : before) near = near || (d.control == row.control && distance(d.at, j.at) < row.apart);
            if (near) continue;
        }
        out.row = static_cast<int>(r);
        out.control = row.control;
        out.why = row.name;
        return out;
    }
    return out;
}

std::vector<JunctionFacts> findJunctions(const RoadLabGraph& graph, const ControlPolicy& policy, const JunctionFinding& how) {
    // The ramps as the builder lays them: their spines composed from their anchors (lane_expand.h expand, the same
    // call the builder makes), so a ramp's stations here are the builder's.
    RoadLabGraph g = graph;
    expand(g);
    const double tol = how.endTolerance >= 0 ? how.endTolerance : g.rules.endpointTol;
    // ---- the nodes: road ends at one point (ends within the graph's endpoint tolerance), in road order --------------
    std::vector<Node> nodes;
    std::map<std::pair<long long, long long>, std::vector<int>> grid;   // cell -> nodes with an end in it
    auto cellOf = [&](const Vec2& p) {
        return std::make_pair(static_cast<long long>(std::floor(p.x / (2 * tol))), static_cast<long long>(std::floor(p.y / (2 * tol))));
    };
    for (std::size_t ei = 0; ei < g.edges.size(); ++ei) {
        const EdgeSpec& e = g.edges[ei];
        if (!carsDrive(g, e)) continue;
        for (int end = 0; end < 2; ++end) {
            if (streetEnd(e, end)) continue;   // a ramp's end on a street's side: its own node, below
            const Vec2 p = end == 0 ? e.xy.front() : e.xy.back();
            const std::pair<long long, long long> c = cellOf(p);
            int found = -1;
            for (long long dx = -1; dx <= 1 && found < 0; ++dx)
                for (long long dy = -1; dy <= 1 && found < 0; ++dy) {
                    const std::map<std::pair<long long, long long>, std::vector<int>>::const_iterator it =
                        grid.find(std::make_pair(c.first + dx, c.second + dy));
                    if (it == grid.end()) continue;
                    for (int n : it->second)
                        if (distance(nodes[static_cast<std::size_t>(n)].at, p) <= tol) {
                            found = n;
                            break;
                        }
                }
            if (found < 0) {
                found = static_cast<int>(nodes.size());
                Node n;
                n.at = p;
                nodes.push_back(n);
                grid[c].push_back(found);
            }
            nodes[static_cast<std::size_t>(found)].points.push_back(NodePoint{static_cast<int>(ei), end == 0 ? 0.0 : e.s.back()});
        }
    }
    // a road end alone is a dead end, not a junction
    nodes.erase(std::remove_if(nodes.begin(), nodes.end(), [](const Node& n) { return n.points.size() < 2; }), nodes.end());
    // ramps ending on a street's side: a ramp may run on inside the street's lanes for up to kRampEndReach before it
    // ends (that stretch lies inside the street's paved road), so the junction is where the ramp comes out of the street's
    // lanes: the ramp's stretch from there to its end lies inside it, and the street meets it there
    for (std::size_t ei = 0; ei < g.edges.size(); ++ei) {
        const EdgeSpec& e = g.edges[ei];
        if (!carsDrive(g, e)) continue;
        for (int end = 0; end < 2; ++end) {
            const RampAnchor* a = streetEnd(e, end);
            if (!a) continue;
            const EdgeSpec* street = g.find(a->edge);
            if (!street || !carsDrive(g, *street)) continue;
            const double freeEnd = end == 0 ? 0.0 : e.s.back();
            double out = freeEnd;   // where the ramp comes out of the street's lanes
            for (std::size_t k = 0; k < e.xy.size(); ++k) {
                const std::size_t i = end == 0 ? k : e.xy.size() - 1 - k;
                if (std::fabs(e.s[i] - freeEnd) > kRampEndReach) break;
                out = e.s[i];
                if (project(street->xy, street->s, e.xy[i]).distance > g.hw(*street)) break;
            }
            Node node;
            node.rampEnd = true;
            node.ramp = static_cast<int>(ei);
            const double sMeet = project(street->xy, street->s, pointAt(e.xy, e.s, out)).station;
            node.at = pointAt(street->xy, street->s, sMeet);
            node.points.push_back(NodePoint{static_cast<int>(ei), freeEnd});
            node.points.push_back(NodePoint{static_cast<int>(ei), out});
            node.points.push_back(NodePoint{g.index.at(a->edge), sMeet});
            nodes.push_back(node);
        }
    }
    // two roads crossing at grade, as the caller names them: a node of both, at each one's station there (a ramp's ends
    // are its own nodes above, so a crossing naming a ramp is not one of these)
    if (how.atGrade)
        for (const CrossingSpec& c : *how.atGrade) {
            if (c.kind != "at_grade") continue;
            const EdgeSpec* a = g.find(c.a);
            const EdgeSpec* b = g.find(c.b);
            if (!a || !b || a == b || a->isRamp() || b->isRamp() || !carsDrive(g, *a) || !carsDrive(g, *b)) continue;
            Node node;
            node.at = c.at;
            node.crossing = true;
            node.points.push_back(NodePoint{g.index.at(c.a), project(a->xy, a->s, c.at).station});
            node.points.push_back(NodePoint{g.index.at(c.b), project(b->xy, b->s, c.at).station});
            nodes.push_back(node);
        }

    // ---- a node where a street only carries on (two arms) is no junction (ADR-0419) -----------------------------------
    auto armsAt = [&](const Node& n) {
        int arms = 0;
        for (const NodePoint& p : n.points) {
            const EdgeSpec& e = g.edges[static_cast<std::size_t>(p.edge)];
            arms += (p.s > kArmReachesPast ? 1 : 0) + (p.s < e.s.back() - kArmReachesPast ? 1 : 0);
        }
        return arms;
    };
    {
        std::vector<Node> kept;
        for (Node& n : nodes) {
            if (armsAt(n) >= 3) {
                kept.push_back(std::move(n));
                continue;
            }
            // (a crossing the caller names where both roads END is the place their road ends already meet: that node holds
            // them, and a crossing of two arms is nothing more)
            if (how.carryOns && !n.crossing && n.points.size() == 2) {
                CarryOn c;
                c.at = n.at;
                for (std::size_t k = 0; k < 2; ++k) {
                    c.roads[k] = g.edges[static_cast<std::size_t>(n.points[k].edge)].id;
                    c.stations[k] = n.points[k].s;
                }
                how.carryOns->push_back(c);
            }
        }
        nodes.swap(kept);
    }

    // ---- nodes with no room for a car between them, and a divided road's two carriageways: ONE junction (ADR-0419) ----
    // (control_plan.h, above findJunctions: the rule.) Along road `via`, a node's stop line stands back from it as the one
    // stop-line rule says (road_model_parts.h stopLineBack), asked of the node as the junction it would be alone: its
    // arms (armsOver, the way a group's arms are made below) and their through pairs as the one rule states them
    // (statedThrough), so the same street carrying on is not a road crossing it.
    struct NodeArms {
        std::vector<ArmFacts> arms;
        ThroughPairs through;
        std::vector<ArmMouth> mouths;
    };
    std::vector<NodeArms> nodeArms(nodes.size());
    for (std::size_t n = 0; n < nodes.size(); ++n) {
        NodeArms& na = nodeArms[n];
        std::map<int, std::vector<double>> stations;
        for (const NodePoint& p : nodes[n].points) stations[p.edge].push_back(p.s);
        na.arms = armsOver(g, stations, {}, policy, nullptr);
        na.through = statedThrough(g, na.arms);
        for (const ArmFacts& a : na.arms) {
            const int edge = g.index.at(a.road);
            const EdgeSpec& e = g.edges[static_cast<std::size_t>(edge)];
            na.mouths.push_back(ArmMouth{edge, pointAt(e.xy, e.s, a.at), g.hw(e)});
        }
    }
    // Metres from node n back along road `via` to its stop line on the side its arm of `via` with this `spineIn` leaves
    // by; below zero when the node has no such arm (the road ends at the node on that side).
    auto nodeStopLineBack = [&](int n, int via, bool spineIn) {
        const NodeArms& na = nodeArms[static_cast<std::size_t>(n)];
        for (std::size_t k = 0; k < na.arms.size(); ++k)
            if (na.mouths[k].road == via && na.arms[k].spineIn == spineIn) return stopLineBack(na.mouths, na.through, k);
        return -1.0;
    };
    // each carriageway of a divided road -> its partner
    std::map<int, int> partner;
    for (const DividedPair& p : how.divided ? *how.divided : g.divided) {
        const EdgeSpec* a = g.find(p[0]);
        const EdgeSpec* b = g.find(p[1]);
        if (!a || !b) continue;
        partner[g.index.at(p[0])] = g.index.at(p[1]);
        partner[g.index.at(p[1])] = g.index.at(p[0]);
    }
    // the two nodes hold the two carriageways of one divided road, neither of them the road they lie along
    auto carriagewaysMeet = [&](const Node& a, const Node& b, int via) {
        for (const NodePoint& p : a.points) {
            const std::map<int, int>::const_iterator it = partner.find(p.edge);
            if (p.edge == via || it == partner.end() || it->second == via) continue;
            for (const NodePoint& q : b.points)
                if (q.edge == it->second) return true;
        }
        return false;
    };
    std::vector<int> parent(nodes.size());
    std::iota(parent.begin(), parent.end(), 0);
    std::map<int, std::vector<std::pair<double, int>>> onRoad;   // road -> (station, node), sorted below
    for (std::size_t n = 0; n < nodes.size(); ++n)
        for (const NodePoint& p : nodes[n].points) onRoad[p.edge].emplace_back(p.s, static_cast<int>(n));
    for (std::pair<const int, std::vector<std::pair<double, int>>>& kv : onRoad) {
        std::sort(kv.second.begin(), kv.second.end());
        for (std::size_t k = 0; k + 1 < kv.second.size(); ++k) {
            const int a = kv.second[k].second, b = kv.second[k + 1].second;
            if (a == b) continue;
            const Node& na = nodes[static_cast<std::size_t>(a)];
            const Node& nb = nodes[static_cast<std::size_t>(b)];
            // the stretch between the two stop lines: a's on its arm leaving up the road, b's on its arm arriving down it.
            // (A node with no such arm has the road ending within kArmReachesPast of it: no room at all.)
            const double backA = nodeStopLineBack(a, kv.first, false), backB = nodeStopLineBack(b, kv.first, true);
            const bool noRoom = backA < 0 || backB < 0 ||
                                kv.second[k + 1].first - kv.second[k].first - backA - backB < g.rules.storage;
            if (noRoom || carriagewaysMeet(na, nb, kv.first) || carriagewaysMeet(nb, na, kv.first)) unite(parent, a, b);
        }
    }

    // ---- each group a junction: its arms, and their facts -------------------------------------------------------------
    std::vector<JunctionFacts> out;
    std::vector<int> idOfRoot(nodes.size(), -1);
    std::vector<std::vector<int>> members;
    for (std::size_t n = 0; n < nodes.size(); ++n) {
        const int r = findRoot(parent, static_cast<int>(n));
        if (idOfRoot[static_cast<std::size_t>(r)] < 0) {
            idOfRoot[static_cast<std::size_t>(r)] = static_cast<int>(members.size());
            members.emplace_back();
        }
        members[static_cast<std::size_t>(idOfRoot[static_cast<std::size_t>(r)])].push_back(static_cast<int>(n));
    }
    for (std::size_t m = 0; m < members.size(); ++m) {
        JunctionFacts j;
        j.id = "j" + std::to_string(m);
        std::map<int, std::vector<double>> stations;   // road -> its stations at this junction's nodes
        std::set<int> ramps;
        for (int n : members[m]) {
            const Node& node = nodes[static_cast<std::size_t>(n)];
            j.at += node.at;
            j.nodes.push_back(node.at);
            j.nodePoints.emplace_back();
            for (const NodePoint& p : node.points)
                j.nodePoints.back().push_back(ModelNodePoint{g.edges[static_cast<std::size_t>(p.edge)].id, written(p.s)});
            j.rampEnd = j.rampEnd || node.rampEnd;
            if (node.ramp >= 0) ramps.insert(node.ramp);
            for (const NodePoint& p : node.points) stations[p.edge].push_back(p.s);
        }
        j.at = j.at / static_cast<double>(members[m].size());
        j.arms = armsOver(g, stations, ramps, policy, &j.inside);
        j.through = statedThrough(g, j.arms);
        settleRoles(j, policy);
        out.push_back(j);
    }
    return out;
}

ModelJunction junctionOf(const JunctionFacts& f) {
    ModelJunction pj;
    pj.id = f.id;
    pj.at = Vec2(written(f.at.x), written(f.at.y));
    for (const ArmFacts& a : f.arms) {
        ModelArm arm;
        arm.road = a.road;
        arm.station = written(a.at);
        arm.spineIn = a.spineIn;
        pj.arms.push_back(arm);
    }
    pj.through = f.through;
    pj.inside = f.inside;
    // a junction over several points says which roads meet at each (road_model_parts.h ModelNode, ADR-0425); one over a
    // single point needs not
    if (!f.inside.empty())
        for (std::size_t n = 0; n < f.nodes.size() && n < f.nodePoints.size(); ++n)
            pj.nodes.push_back(ModelNode{Vec2(written(f.nodes[n].x), written(f.nodes[n].y)), f.nodePoints[n]});
    return pj;
}

ControlPlan planControls(const RoadLabGraph& g, const ControlPolicy& policy, const JunctionFinding& finding) {
    ControlPlan plan;
    std::vector<CarryOn> carryOns;
    JunctionFinding how = finding;
    how.carryOns = &carryOns;
    std::vector<JunctionFacts> all = findJunctions(g, policy, how);
    // the places where a street only carries on: what the policy gives a place of two arms, the same everywhere
    plan.carryOns = static_cast<uint32_t>(carryOns.size());
    {
        RoadLabGraph ge = g;
        expand(ge);   // (the ramps' spines composed, as findJunctions composes them)
        std::set<std::string> controls;
        for (const CarryOn& c : carryOns) {
            JunctionFacts j;
            j.id = "carry-on";
            j.at = c.at;
            for (std::size_t k = 0; k < 2; ++k) {
                const EdgeSpec* e = ge.find(c.roads[k]);
                if (!e || e->xy.size() < 2) continue;
                // (the road ends here: its spine runs in)
                j.arms.push_back(armFacts(*e, c.stations[k], c.stations[k] > kArmReachesPast, policy));
            }
            j.through = statedThrough(ge, j.arms);
            settleRoles(j, policy);
            const ControlChoice choice = chooseControl(j, policy, {});
            if (choice.row >= 0) controls.insert(choice.control);
        }
        if (controls.size() == 1) plan.carryOnControl = *controls.begin();
        else if (controls.size() > 1)
            plan.problems.push_back(fmt("the places where a street only carries on are given %zu different controls by the policy; none is "
                                        "written", controls.size()));
    }
    // hand-written junctions are kept; a planned junction sharing an arm's road end with one is left to it
    std::set<std::pair<std::string, bool>> handArms;
    std::vector<Decided> before;
    for (const ModelJunction& h : g.junctions) {
        if (!h.hand) continue;
        plan.junctions.push_back(h);
        ++plan.handKept;
        for (const ModelArm& a : h.arms) handArms.insert(std::make_pair(a.road, a.spineIn));
        before.push_back(Decided{h.at, h.control});
    }
    std::vector<JunctionFacts> todo;
    for (const JunctionFacts& j : all) {
        bool taken = false;
        for (const ArmFacts& a : j.arms) taken = taken || handArms.count(std::make_pair(a.road, a.spineIn)) > 0;
        if (!taken) todo.push_back(j);
    }
    // WHICH ARMS ARE ONE ROAD CARRYING ON is the road file's to state. A junction the file already lists with its layout
    // (the same id, the same arms, every arm's heading written) states its through pairs: they are read as stated, on the
    // arms as found here, and the roles settled from them. (One the file does not list keeps what throughPairs states
    // for it: the pairs the layout then writes.)
    for (JunctionFacts& j : todo) {
        const ModelJunction* stated = nullptr;
        for (const ModelJunction& l : g.junctions)
            if (l.id == j.id) stated = &l;
        if (!stated || stated->arms.size() != j.arms.size()) continue;
        std::vector<uint32_t> found(stated->arms.size(), kNoIndex);   // a listed arm -> the arm found here
        bool same = true;
        for (std::size_t k = 0; k < stated->arms.size() && same; ++k) {
            const ModelArm& a = stated->arms[k];
            for (std::size_t n = 0; n < j.arms.size(); ++n)
                if (j.arms[n].road == a.road && j.arms[n].spineIn == a.spineIn && sameWrittenStation(written(j.arms[n].at), a.station))
                    found[k] = static_cast<uint32_t>(n);
            same = found[k] != kNoIndex && engine::stated(a.heading);
        }
        if (!same) continue;
        j.through.clear();
        for (const std::array<uint32_t, 2>& t : stated->through)
            if (t[0] < found.size() && t[1] < found.size()) j.through.push_back({found[t[0]], found[t[1]]});
        settleRoles(j, policy);
    }
    // decided most important first: the major road's class, then the minor road's, then more lanes, then id order
    std::vector<std::size_t> order(todo.size());
    std::iota(order.begin(), order.end(), 0);
    auto key = [&](const JunctionFacts& j) {
        const uint32_t major = j.approaches > 0 ? policy.classRank(j.majorClass) : std::numeric_limits<uint32_t>::max();
        const uint32_t minor = j.hasMinor ? policy.classRank(j.minorClass) : major;
        return std::make_tuple(major, minor, -j.majorLanes, -j.minorLanes);
    };
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return key(todo[a]) < key(todo[b]); });
    std::vector<ControlChoice> choice(todo.size());
    for (std::size_t i : order) {
        choice[i] = chooseControl(todo[i], policy, before);
        if (choice[i].row < 0) {
            plan.problems.push_back("junction " + todo[i].id + " at " + fmt("(%.1f, %.1f)", todo[i].at.x, todo[i].at.y) +
                                    ": no row of the placement policy holds for it (" + describeFacts(todo[i]) +
                                    "); it is left out of the plan");
            continue;
        }
        before.push_back(Decided{todo[i].at, choice[i].control});
    }
    for (const PolicyRow& row : policy.rows) plan.byRow.emplace_back(row.name, 0);
    for (std::size_t i = 0; i < todo.size(); ++i) {
        if (choice[i].row < 0) continue;
        const JunctionFacts& j = todo[i];
        const PolicyRow& row = policy.rows[static_cast<std::size_t>(choice[i].row)];
        ModelJunction pj = junctionOf(j);
        pj.control = choice[i].control;
        pj.why = choice[i].why;
        for (std::size_t a = 0; a < j.arms.size(); ++a) {
            if (j.arms[a].arriving > 0) pj.arms[a].regulations = j.arms[a].major ? row.majorSigns : row.minorSigns;
            for (const std::string& kind : pj.arms[a].regulations) count(plan.signs, kind);
        }
        plan.junctions.push_back(pj);
        plan.facts.push_back(j);
        count(plan.byControl, pj.control);
        ++plan.byRow[static_cast<std::size_t>(choice[i].row)].second;
    }
    for (const ModelJunction& h : g.junctions)
        if (h.hand) count(plan.byControl, h.control);
    std::sort(plan.byControl.begin(), plan.byControl.end());
    std::sort(plan.signs.begin(), plan.signs.end());
    for (const EdgeSpec& e : g.edges) {
        if (!carsDrive(g, e)) continue;
        plan.limits.emplace_back(e.id, postedLimitOn(e, policy));
    }
    return plan;
}

void writeControlPlan(nlohmann::json& scene, const ControlPlan& plan, bool junctionsOnly) {
    // WHAT THE SCENE STATES OF A JUNCTION ALREADY LISTED is kept where the same junction is planned again: by its id, and
    // each arm by its road, station and way. The listed junctions are read into the model's own types (the road file's
    // one reader) and each junction is written by the one writer (road_layout.h junctionJson), so whatever an arm states
    // is carried whole: nothing here names the members to keep.
    std::vector<ModelJunction> listedJunctions;
    std::vector<ModelStreet> listedStreets;
    if (scene.contains("junctions") && scene["junctions"].is_array() && !scene["junctions"].empty()) {
        RoadLabGraph asListed = RoadLabGraph::fromJson(scene);
        listedJunctions = std::move(asListed.junctions);
        listedStreets = std::move(asListed.streets);
    }
    std::map<std::string, const ModelJunction*> listed;
    for (const ModelJunction& j : listedJunctions) listed[j.id] = &j;
    // the scene's roads composed, to measure a ring on (made once, when a junction needs it); the scene's word that its
    // listed arms are in their ring
    const bool markedRing = scene.value("arms_in_ring", false);
    std::unique_ptr<ComposedRoads> composed;
    auto roads = [&]() -> const ComposedRoads& {
        if (!composed) {
            nlohmann::json bare = scene;
            for (const char* key : {"junctions", "streets", "divided", "arms_in_ring"}) bare.erase(key);
            composed = std::make_unique<ComposedRoads>(RoadLabGraph::fromJson(bare));
        }
        return *composed;
    };
    nlohmann::json list = nlohmann::json::array();
    for (const ModelJunction& pj : plan.junctions) {
        // the junction as written: the plan's id, place, control, stretches inside and points; its arms and through
        // pairs set below
        ModelJunction j = pj;
        j.arms.clear();
        j.through.clear();
        const std::map<std::string, const ModelJunction*>::const_iterator at = listed.find(pj.id);
        const ModelJunction* was = at != listed.end() ? at->second : nullptr;
        const bool wasListed = was != nullptr;
        std::vector<ModelArm> arms;
        std::vector<int> oldIndex;   // each arm written: the listed arm it is (-1: none)
        for (const ModelArm& a : pj.arms) {
            // the planned arm: where it is and the signs the plan stands on it; what else the scene states of it (its
            // street, its heading) only where it is a listed arm
            ModelArm arm;
            int found = -1;
            if (wasListed)
                for (std::size_t k = 0; k < was->arms.size() && found < 0; ++k) {
                    const ModelArm& old = was->arms[k];
                    if (old.road == a.road && old.spineIn == a.spineIn && sameWrittenStation(old.station, a.station)) {
                        found = static_cast<int>(k);
                        arm = old;
                    }
                }
            arm.road = a.road;
            arm.station = a.station;
            arm.spineIn = a.spineIn;
            arm.regulations = a.regulations;
            oldIndex.push_back(found);
            arms.push_back(arm);
        }
        // THE ARMS IN THEIR RING (ADR-0427). Where the scene marks its list as in ring order ("arms_in_ring") and every
        // planned arm is one listed arm with none left over, the list's own order is kept (a planner's stated ring stands);
        // otherwise the arms go in the order the one measure gives (road_layout.h armRing, on the scene's roads composed).
        // Either way each listed through pair whose two arms were both kept follows its arms (ADR-0424: written in another
        // order without renumbering, a through pair named two other arms).
        const std::size_t listedArms = wasListed ? was->arms.size() : 0;
        std::vector<int> planOfListed(listedArms, -1);   // a listed arm -> the planned arm it is
        bool permutation = wasListed && oldIndex.size() == listedArms;
        for (std::size_t n = 0; n < oldIndex.size(); ++n) {
            if (oldIndex[n] < 0 || planOfListed[static_cast<std::size_t>(oldIndex[n])] >= 0) { permutation = false; continue; }
            planOfListed[static_cast<std::size_t>(oldIndex[n])] = static_cast<int>(n);
        }
        std::vector<std::size_t> slots;   // each place in the written list -> the planned arm written there
        if (permutation && markedRing)
            for (std::size_t k = 0; k < listedArms; ++k) slots.push_back(static_cast<std::size_t>(planOfListed[k]));
        else
            slots = armRing(pj, roads());
        std::vector<int> placeOf(pj.arms.size(), -1);   // a planned arm -> its place in the written list
        for (std::size_t k = 0; k < slots.size(); ++k) {
            placeOf[slots[k]] = static_cast<int>(k);
            j.arms.push_back(arms[slots[k]]);
        }
        // (a junction the scene does not list is written with no through pairs here: the layout states them)
        if (wasListed)
            for (const std::array<uint32_t, 2>& pair : was->through) {
                if (pair[0] >= listedArms || pair[1] >= listedArms) continue;
                const int n0 = planOfListed[pair[0]], n1 = planOfListed[pair[1]];
                if (n0 < 0 || n1 < 0) continue;
                j.through.push_back({static_cast<uint32_t>(placeOf[static_cast<std::size_t>(n0)]),
                                     static_cast<uint32_t>(placeOf[static_cast<std::size_t>(n1)])});
            }
        // THE JUNCTION'S POINTS (ADR-0425): the plan's where it has them, else the listed junction's (a list written
        // before the plan, its stretches inside the same)
        if (j.nodes.empty() && wasListed && !j.inside.empty() && j.inside.size() == was->inside.size()) {
            bool sameInside = true;
            for (std::size_t k = 0; k < j.inside.size() && sameInside; ++k)
                sameInside = j.inside[k].road == was->inside[k].road && sameWrittenStation(j.inside[k].from, was->inside[k].from) &&
                             sameWrittenStation(j.inside[k].to, was->inside[k].to);
            if (sameInside) j.nodes = was->nodes;
        }
        list.push_back(junctionJson(j, listedStreets));
    }
    scene["junctions"] = list;
    scene["arms_in_ring"] = true;   // every junction's arms just written in their ring (ADR-0427)
    if (junctionsOnly) return;
    if (!plan.carryOnControl.empty()) scene["carry_on_control"] = plan.carryOnControl;
    else scene.erase("carry_on_control");
    std::map<std::string, double> limitOf(plan.limits.begin(), plan.limits.end());
    for (nlohmann::json& e : scene["edges"]) {
        const std::map<std::string, double>::const_iterator it = limitOf.find(e.at("id").get<std::string>());
        if (it == limitOf.end()) continue;
        if (it->second > 0) e["speed_limit"] = it->second;
        else e.erase("speed_limit");
    }
}

std::string describeControlPlan(const ControlPlan& plan) {
    std::string out = fmt("%zu junctions planned (%u written by hand and kept):", plan.junctions.size(), plan.handKept);
    for (std::size_t i = 0; i < plan.byControl.size(); ++i)
        out += fmt("%s %u %s", i ? "," : "", plan.byControl[i].second, plan.byControl[i].first.c_str());
    out += "\n  by the policy's rows:";
    for (std::size_t i = 0; i < plan.byRow.size(); ++i)
        out += fmt("%s %u \"%s\"", i ? "," : "", plan.byRow[i].second, plan.byRow[i].first.c_str());
    out += fmt("\n  places where a street only carries on (no junction): %u; the control the builder's own junction there takes: %s",
               plan.carryOns, plan.carryOnControl.empty() ? "none stated" : plan.carryOnControl.c_str());
    out += "\n  regulations stood where approaches arrive:";
    if (plan.signs.empty()) out += " none";
    for (std::size_t i = 0; i < plan.signs.size(); ++i)
        out += fmt("%s %u approaches %s", i ? "," : "", plan.signs[i].second, plan.signs[i].first.c_str());
    std::map<double, uint32_t> limits;
    for (const std::pair<std::string, double>& l : plan.limits) ++limits[l.second];
    out += "\n  roads by posted limit:";
    for (const std::pair<const double, uint32_t>& l : limits)
        out += l.first > 0 ? fmt(" %u at %.2f m/s;", l.second, l.first) : fmt(" %u with none posted;", l.second);
    out += "\n";
    for (const std::string& p : plan.problems) out += "  problem: " + p + "\n";
    return out;
}

}  // namespace roads::lanes
}  // namespace engine
