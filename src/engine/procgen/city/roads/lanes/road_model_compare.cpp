#include "engine/procgen/city/roads/lanes/road_model_compare.h"

#include "apps/citysim/city_sim.h"                            // the kerbside bays the city sim lays
#include "apps/citysim/lane_map.h"                            // the lane match the city sim makes
#include "engine/ai/nav_graph.h"
#include "engine/components.h"                                // TrafficLane: the lanes as the loader publishes them
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"
#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/road_twin.h"
#include "engine/procgen/city/roads/road_entity.h"            // navRoadGraph
#include "rt_math.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <map>
#include <memory>
#include <set>

namespace engine {
namespace roads::lanes {

namespace {

std::string fmt(const char* f, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}

// The worst examples of one finding: the largest `key` first, up to kExamples of them.
constexpr std::size_t kExamples = 8;
struct Examples {
    std::vector<std::pair<double, std::string>> all;
    void add(double key, const std::string& text) { all.emplace_back(key, text); }
    std::string text(const std::string& indent = "    ") const {
        std::vector<std::pair<double, std::string>> v = all;
        std::stable_sort(v.begin(), v.end(), [](const std::pair<double, std::string>& a, const std::pair<double, std::string>& b) { return a.first > b.first; });
        std::string out;
        for (std::size_t i = 0; i < v.size() && i < kExamples; ++i) out += indent + v[i].second + "\n";
        if (v.size() > kExamples) out += indent + fmt("... and %zu more\n", v.size() - kExamples);
        return out;
    }
};

double secondsSince(const std::chrono::steady_clock::time_point& t) { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count(); }

// Which of the other form's junctions lie near each model junction, and the reverse.
struct Pairing {
    std::vector<std::vector<uint32_t>> ofModel, ofOther;
};
Pairing pairJunctions(const std::vector<std::vector<Vec2>>& anchors, const std::vector<Vec2>& others) {
    Pairing p;
    p.ofModel.resize(anchors.size());
    p.ofOther.resize(others.size());
    const double cell = 2 * kCompareReach;
    std::map<std::pair<long long, long long>, std::vector<uint32_t>> grid;
    for (std::size_t o = 0; o < others.size(); ++o)
        grid[std::make_pair(static_cast<long long>(std::floor(others[o].x / cell)), static_cast<long long>(std::floor(others[o].y / cell)))].push_back(static_cast<uint32_t>(o));
    for (std::size_t j = 0; j < anchors.size(); ++j) {
        std::set<uint32_t> found;
        for (const Vec2& a : anchors[j]) {
            const long long cx = static_cast<long long>(std::floor(a.x / cell)), cy = static_cast<long long>(std::floor(a.y / cell));
            for (long long dx = -1; dx <= 1; ++dx)
                for (long long dy = -1; dy <= 1; ++dy) {
                    const std::map<std::pair<long long, long long>, std::vector<uint32_t>>::const_iterator it = grid.find(std::make_pair(cx + dx, cy + dy));
                    if (it == grid.end()) continue;
                    for (uint32_t o : it->second)
                        if (distance(others[o], a) <= kCompareReach) found.insert(o);
                }
        }
        for (uint32_t o : found) {
            p.ofModel[j].push_back(o);
            p.ofOther[o].push_back(static_cast<uint32_t>(j));
        }
    }
    return p;
}

JunctionAgreement tally(const Pairing& p) {
    JunctionAgreement a;
    a.model = static_cast<uint32_t>(p.ofModel.size());
    a.other = static_cast<uint32_t>(p.ofOther.size());
    for (const std::vector<uint32_t>& m : p.ofModel) {
        if (m.empty()) ++a.onlyModel;
        if (m.size() >= 2) ++a.modelKnots;
        if (m.size() == 1 && p.ofOther[m.front()].size() == 1) ++a.matched;
    }
    for (const std::vector<uint32_t>& o : p.ofOther) {
        if (o.empty()) ++a.onlyOther;
        if (o.size() >= 2) ++a.otherKnots;
    }
    return a;
}

std::string agreementText(const char* other, const JunctionAgreement& a) {
    return fmt("  model %u, %s %u: %u matched one to one; %u only in the model; %u only in the %s; %u of the %s's hold two model "
               "junctions or more as one; %u model junctions the %s splits into two or more\n",
               a.model, other, a.other, a.matched, a.onlyModel, a.onlyOther, other, a.otherKnots, other, a.modelKnots, other);
}

std::string names(const RoadModel& m, const std::vector<uint32_t>& js) {
    std::string out;
    for (uint32_t j : js) out += (out.empty() ? "" : ", ") + m.junctions[j].id;
    return out;
}

const char* className(RoadClass k) { return roadClassName(k); }

}  // namespace

RoadModelComparison compareRoadModel(const RoadModel& m, const Result& r) {
    RoadModelComparison out;
    const RoadLabGraph& g = r.graph;
    std::string& T = out.text;
    T = fmt("THE ROAD MODEL OF %s AGAINST THE OLD FORMS\n\n", m.name.c_str());
    T += fmt("How \"the same junction\" is decided: another form's junction within %.0f m of the model junction's centre or of one of its "
             "arms' points (where the arm meets it). How a street is compared: the nav link along its middle (within %.0f m, heading "
             "within 25 degrees).\n",
             kCompareReach, kCompareLinkReach);

    // ---- the model junctions' anchor points ------------------------------------------------------------------------------
    std::vector<std::vector<Vec2>> anchors(m.junctions.size());
    for (std::size_t j = 0; j < m.junctions.size(); ++j) {
        const ModelJunction& J = m.junctions[j];
        anchors[j].push_back(J.at);
        for (const ModelArm& a : J.arms) {
            const EdgeSpec* e = g.find(a.road);
            if (!e || e->xy.size() < 2) continue;
            anchors[j].push_back(pointAt(e->xy, e->s, a.station));
            // and kCompareReach out along the arm: a junction of roads meeting at a shallow angle is drawn where their lanes
            // part, further out along them than a square one
            const double out = a.spineIn ? std::max(0.0, a.station - kCompareReach) : std::min(e->s.back(), a.station + kCompareReach);
            anchors[j].push_back(pointAt(e->xy, e->s, out));
        }
        if (J.arms.size() == 2) ++out.modelTwoArm;
    }

    // ---- the nav graph, as the city sim builds it ------------------------------------------------------------------------
    const auto tNav = std::chrono::steady_clock::now();
    HeightField ground;
    if (r.hasTerrain) {
        std::shared_ptr<HeightGrid> grid = std::make_shared<HeightGrid>(r.terrain);
        ground = [grid](double x, double z) { return grid->sample(x, z); };
    }
    const RoadEntity twin = roadTwin(r, 60.0, false, /*forNav=*/true);
    const RoadGraph navRoads = navRoadGraph(twin, ground ? ground : HeightField());
    NavGraph nav = buildNavGraph(navRoads);   // (its lanes set from the road file below, as the sim's are: ADR-0418)
    std::vector<std::vector<int>> incident(nav.nodes.size());
    for (int l = 0; l < nav.linkCount(); ++l) {
        incident[static_cast<std::size_t>(nav.links[static_cast<std::size_t>(l)].from)].push_back(l);
        incident[static_cast<std::size_t>(nav.links[static_cast<std::size_t>(l)].to)].push_back(l);
    }
    std::vector<int> navJ;
    std::vector<Vec2> navAt;
    std::vector<char> navRamp;
    for (int n = 0; n < nav.nodeCount(); ++n) {
        if (!nav.isJunction(n)) continue;
        bool ramp = false;
        for (int l : incident[static_cast<std::size_t>(n)]) {
            const RoadClass k = nav.links[static_cast<std::size_t>(l)].klass;
            ramp = ramp || k == RoadClass::Freeway || k == RoadClass::Ramp;
        }
        navJ.push_back(n);
        navAt.push_back(nav.nodes[static_cast<std::size_t>(n)]);
        navRamp.push_back(ramp ? 1 : 0);
        out.navRampNodes += ramp ? 1u : 0u;
    }
    const double navSeconds = secondsSince(tNav);
    const Pairing pn = pairJunctions(anchors, navAt);
    out.nav = tally(pn);

    // ---- the paint's boxes (kept by the build), as the model's assembly attributed them -------------------------------
    struct Box { int edge; JunctionSpan span; Vec2 mid; std::vector<uint32_t> heldBy; };
    std::vector<Box> boxes;
    for (const std::pair<const int, std::vector<JunctionSpan>>& kv : r.junctionBoxes) {
        const EdgeSpec& e = g.edges[static_cast<std::size_t>(kv.first)];
        for (const JunctionSpan& sp : kv.second) boxes.push_back(Box{kv.first, sp, pointAt(e.xy, e.s, 0.5 * (sp.s0 + sp.s1)), {}});
    }
    std::map<std::tuple<std::string, double, double>, std::size_t> boxIndex;
    for (std::size_t b = 0; b < boxes.size(); ++b) boxIndex[std::make_tuple(g.edges[static_cast<std::size_t>(boxes[b].edge)].id, boxes[b].span.s0, boxes[b].span.s1)] = b;
    std::vector<std::vector<uint32_t>> boxesOf(m.junctions.size());
    for (std::size_t j = 0; j < m.junctions.size(); ++j)
        for (const ModelStretch& st : m.junctions[j].boxes) {
            const std::map<std::tuple<std::string, double, double>, std::size_t>::const_iterator it = boxIndex.find(std::make_tuple(st.road, st.from, st.to));
            if (it == boxIndex.end()) continue;
            boxesOf[j].push_back(static_cast<uint32_t>(it->second));
            boxes[it->second].heldBy.push_back(static_cast<uint32_t>(j));
        }
    {
        JunctionAgreement& a = out.paint;
        a.model = static_cast<uint32_t>(m.junctions.size());
        // the paint has no junctions, only a box along each street: its "junctions" are its boxes, grouped where their middles
        // lie within kCompareReach of each other (a divided street's two, a crossroads' two or four)
        std::vector<int> group(boxes.size(), -1);
        int groups = 0;
        for (std::size_t b = 0; b < boxes.size(); ++b) {
            if (group[b] >= 0) continue;
            std::vector<std::size_t> todo{b};
            group[b] = groups;
            while (!todo.empty()) {
                const std::size_t c = todo.back();
                todo.pop_back();
                for (std::size_t d = 0; d < boxes.size(); ++d)
                    if (group[d] < 0 && distance(boxes[c].mid, boxes[d].mid) <= 2 * kCompareReach) { group[d] = groups; todo.push_back(d); }
            }
            ++groups;
        }
        a.other = static_cast<uint32_t>(groups);
        std::vector<std::set<uint32_t>> junctionsOfGroup(static_cast<std::size_t>(groups));
        for (std::size_t b = 0; b < boxes.size(); ++b)
            for (uint32_t j : boxes[b].heldBy) junctionsOfGroup[static_cast<std::size_t>(group[b])].insert(j);
        std::vector<std::set<int>> groupsOfJunction(m.junctions.size());
        for (std::size_t j = 0; j < m.junctions.size(); ++j)
            for (uint32_t b : boxesOf[j]) groupsOfJunction[j].insert(group[b]);
        for (std::size_t j = 0; j < m.junctions.size(); ++j) {
            if (groupsOfJunction[j].empty()) ++a.onlyModel;
            if (groupsOfJunction[j].size() >= 2) ++a.modelKnots;
            if (groupsOfJunction[j].size() == 1 && junctionsOfGroup[static_cast<std::size_t>(*groupsOfJunction[j].begin())].size() == 1) ++a.matched;
        }
        for (const std::set<uint32_t>& js : junctionsOfGroup) {
            if (js.empty()) ++a.onlyOther;
            if (js.size() >= 2) ++a.otherKnots;
        }
    }

    // ---- the lane graph ------------------------------------------------------------------------------------------------------
    const auto tLg = std::chrono::steady_clock::now();
    LaneGraphOptions lgo;
    {
        VehicleSize car, longVehicle;   // lanes_tool lanegraph's two test vehicles
        car.tag = "car";
        car.turningRadius = 5.0;
        car.width = 1.9;
        longVehicle.tag = "long_vehicle";
        longVehicle.turningRadius = 10.0;
        longVehicle.width = 2.6;
        lgo.vehicles = {car, longVehicle};
    }
    const LaneGraphBuild lg = buildLaneGraph(r, lgo);
    const double lgSeconds = secondsSince(tLg);
    std::vector<Vec2> lgAt;
    for (const JunctionNote& n : lg.junctionNotes) lgAt.push_back(n.at);
    const Pairing pl = pairJunctions(anchors, lgAt);
    out.laneGraph = tally(pl);

    // ---- junctions ----------------------------------------------------------------------------------------------------------
    T += fmt("\nJUNCTIONS. The model holds %zu (%u of them two arms: a street carrying on, which no other form calls a junction).\n",
             m.junctions.size(), out.modelTwoArm);
    T += "\n1. Against the nav graph's junction nodes (nodes where three or more roads meet, after its own knot merging):\n";
    T += agreementText("nav", out.nav);
    {
        Examples onlyModel, onlyNavStreet, onlyNavRamp, navKnots, modelSplit;
        uint32_t onlyNavRampCount = 0, onlyModelTwoArm = 0;
        for (std::size_t j = 0; j < m.junctions.size(); ++j) {
            const ModelJunction& J = m.junctions[j];
            if (pn.ofModel[j].empty()) {
                if (J.arms.size() == 2) ++onlyModelTwoArm;
                else onlyModel.add(static_cast<double>(J.arms.size()), fmt("%s at (%.1f, %.1f), %zu arms", J.id.c_str(), J.at.x, J.at.y, J.arms.size()));
            }
            if (pn.ofModel[j].size() >= 2) {
                double spread = 0;
                for (uint32_t a : pn.ofModel[j]) for (uint32_t b : pn.ofModel[j]) spread = std::max(spread, distance(navAt[a], navAt[b]));
                modelSplit.add(spread, fmt("%s at (%.1f, %.1f): %zu nav junction nodes, %.1f m apart at most", J.id.c_str(), J.at.x, J.at.y, pn.ofModel[j].size(), spread));
            }
        }
        for (std::size_t o = 0; o < navAt.size(); ++o) {
            const int n = navJ[o];
            if (pn.ofOther[o].empty()) {
                if (navRamp[o]) {
                    ++onlyNavRampCount;
                    onlyNavRamp.add(static_cast<double>(incident[static_cast<std::size_t>(n)].size()), fmt("nav node %d at (%.1f, %.1f), %zu links", n, navAt[o].x, navAt[o].y, incident[static_cast<std::size_t>(n)].size()));
                } else {
                    onlyNavStreet.add(static_cast<double>(incident[static_cast<std::size_t>(n)].size()), fmt("nav node %d at (%.1f, %.1f), %zu links, knot spread %.1f m", n, navAt[o].x, navAt[o].y,
                                                                                                           incident[static_cast<std::size_t>(n)].size(), nav.nodeSpread.empty() ? 0.0 : nav.nodeSpread[static_cast<std::size_t>(n)]));
                }
            }
            if (pn.ofOther[o].size() >= 2) {
                double apart = 0;
                for (uint32_t a : pn.ofOther[o]) for (uint32_t b : pn.ofOther[o]) apart = std::max(apart, distance(m.junctions[a].at, m.junctions[b].at));
                navKnots.add(apart, fmt("nav node %d at (%.1f, %.1f) holds model junctions %s (%.1f m apart at most)", n, navAt[o].x, navAt[o].y,
                                        names(m, pn.ofOther[o]).c_str(), apart));
            }
        }
        T += fmt("  of those only in the model, %u are two-arm junctions; the rest:\n", onlyModelTwoArm) + onlyModel.text();
        T += fmt("  only in the nav, at streets (%zu):\n", onlyNavStreet.all.size()) + onlyNavStreet.text();
        T += fmt("  only in the nav, at freeway or ramp gores (%u; the model holds a ramp's anchored end as a ramp, not a junction):\n", onlyNavRampCount) + onlyNavRamp.text();
        T += "  nav nodes that hold two model junctions or more as one (the nav's knots):\n" + navKnots.text();
        T += "  model junctions the nav splits into several nodes:\n" + modelSplit.text();
    }
    T += "\n2. Against the paint's junction boxes (a box along each street; boxes whose middles lie within 24 m of each other are one painted junction here):\n";
    T += agreementText("paint", out.paint);
    {
        Examples noBox, unheld, shared;
        for (std::size_t j = 0; j < m.junctions.size(); ++j)
            if (boxesOf[j].empty() && m.junctions[j].arms.size() >= 3)
                noBox.add(static_cast<double>(m.junctions[j].arms.size()), fmt("%s at (%.1f, %.1f), %zu arms", m.junctions[j].id.c_str(), m.junctions[j].at.x, m.junctions[j].at.y, m.junctions[j].arms.size()));
        for (const Box& b : boxes) {
            const std::string& road = g.edges[static_cast<std::size_t>(b.edge)].id;
            if (b.heldBy.empty()) unheld.add(b.span.s1 - b.span.s0, fmt("box on %s, stations %.1f to %.1f, at (%.1f, %.1f)", road.c_str(), b.span.s0, b.span.s1, b.mid.x, b.mid.y));
            if (b.heldBy.size() >= 2) shared.add(static_cast<double>(b.heldBy.size()), fmt("box on %s at (%.1f, %.1f) holds model junctions %s", road.c_str(), b.mid.x, b.mid.y, names(m, b.heldBy).c_str()));
        }
        T += fmt("  model junctions of three arms or more with no box (%zu):\n", noBox.all.size()) + noBox.text();
        T += fmt("  boxes no model junction holds (%zu):\n", unheld.all.size()) + unheld.text();
        T += fmt("  boxes holding two model junctions or more (%zu):\n", shared.all.size()) + shared.text();
    }
    T += "\n3. Against the lane graph's junctions (built from the paint's boxes, its carry-ons and its nodes):\n";
    T += agreementText("lane graph", out.laneGraph);
    {
        Examples onlyLg, lgKnots, modelSplit, onlyModel;
        std::map<int, uint32_t> onlyByOrigin;
        static const char* kOrigin[] = {"box", "carry-on", "node"};
        for (std::size_t o = 0; o < lgAt.size(); ++o) {
            const JunctionNote& n = lg.junctionNotes[o];
            if (pl.ofOther[o].empty()) {
                ++onlyByOrigin[static_cast<int>(n.origin)];
                onlyLg.add(static_cast<double>(n.turnsStated), fmt("lane graph junction %zu (%s) at (%.1f, %.1f), roads %s", o, kOrigin[static_cast<int>(n.origin)], n.at.x, n.at.y, n.roads.c_str()));
            }
            if (pl.ofOther[o].size() >= 2) {
                double apart = 0;
                for (uint32_t a : pl.ofOther[o]) for (uint32_t b : pl.ofOther[o]) apart = std::max(apart, distance(m.junctions[a].at, m.junctions[b].at));
                lgKnots.add(apart, fmt("lane graph junction %zu at (%.1f, %.1f) holds model junctions %s (%.1f m apart at most)", o, n.at.x, n.at.y, names(m, pl.ofOther[o]).c_str(), apart));
            }
        }
        for (std::size_t j = 0; j < m.junctions.size(); ++j) {
            if (pl.ofModel[j].size() >= 2) modelSplit.add(static_cast<double>(pl.ofModel[j].size()), fmt("%s at (%.1f, %.1f): %zu lane graph junctions", m.junctions[j].id.c_str(), m.junctions[j].at.x, m.junctions[j].at.y, pl.ofModel[j].size()));
            if (pl.ofModel[j].empty() && m.junctions[j].arms.size() >= 3) onlyModel.add(static_cast<double>(m.junctions[j].arms.size()), fmt("%s at (%.1f, %.1f), %zu arms", m.junctions[j].id.c_str(), m.junctions[j].at.x, m.junctions[j].at.y, m.junctions[j].arms.size()));
        }
        std::string origins;
        for (const std::pair<const int, uint32_t>& b : onlyByOrigin) origins += fmt("%s%u %s", origins.empty() ? "" : ", ", b.second, kOrigin[b.first]);
        T += fmt("  only in the lane graph, by what it was made from: %s\n", origins.empty() ? "none" : origins.c_str()) + onlyLg.text();
        T += fmt("  model junctions of three arms or more with no lane graph junction (%zu):\n", onlyModel.all.size()) + onlyModel.text();
        T += "  lane graph junctions that hold two model junctions or more as one:\n" + lgKnots.text();
        T += "  model junctions the lane graph splits into several:\n" + modelSplit.text();
    }

    // ---- streets against nav links ------------------------------------------------------------------------------------------
    const double cosHeading = std::cos(25.0 * DEG_TO_RAD);
    auto linkAlong = [&](const Vec2& p, const Vec2& t, bool forward) {
        int best = -1;
        double bd = kCompareLinkReach;
        for (int l = 0; l < nav.streetLinkCount(); ++l) {
            const NavLink& L = nav.links[static_cast<std::size_t>(l)];
            const Vec2 a = nav.nodes[static_cast<std::size_t>(L.from)], b = nav.nodes[static_cast<std::size_t>(L.to)];
            const Vec2 ab = b - a;
            const double l2 = dot(ab, ab);
            if (l2 < 1e-12) continue;
            const double d = dot(normalize(ab), t);
            if (forward ? d < cosHeading : d > -cosHeading) continue;
            const double u = std::clamp(dot(p - a, ab) / l2, 0.0, 1.0);
            const double dist = (p - (a + ab * u)).length();
            if (dist < bd) { bd = dist; best = l; }
        }
        return best;
    };
    // the nav's chain of links through `link`, from junction node (or dead end) to junction node: its length
    auto chainLength = [&](int link) {
        double len = nav.links[static_cast<std::size_t>(link)].length;
        for (int way = 0; way < 2; ++way) {
            int cur = link;
            for (int guard = 0; guard < 100000; ++guard) {
                const NavLink& c = nav.links[static_cast<std::size_t>(cur)];
                const int n = way == 0 ? c.to : c.from;
                if (nav.isJunction(n)) break;
                int next = -1, count = 0;
                for (int l : incident[static_cast<std::size_t>(n)]) {
                    const NavLink& o = nav.links[static_cast<std::size_t>(l)];
                    if (l == cur) continue;
                    if (way == 0 ? o.from != n : o.to != n) continue;                 // leaving n onward (or arriving at it, backwards)
                    if ((way == 0 ? o.to : o.from) == (way == 0 ? c.from : c.to)) continue;   // the same road the other way
                    next = l;
                    ++count;
                }
                if (count != 1 || next == link) break;
                cur = next;
                len += nav.links[static_cast<std::size_t>(cur)].length;
            }
        }
        return len;
    };
    std::vector<engine::TrafficLane> traffic;
    for (const Lane& l : r.lanes.lanes) {
        engine::TrafficLane t;
        t.xy = l.xy;
        t.w = l.w;
        t.parent = l.parent;
        t.dir = l.dir;
        t.kind = l.kind;
        traffic.push_back(std::move(t));
    }
    // each road's centre line and stated lanes, as the loader publishes them (engine::LevelLanes::roads, ::stated)
    std::vector<std::vector<Vec2>> spines;
    std::vector<engine::StatedLanes> statedLanes;
    for (const EdgeSpec& e : g.edges) {
        spines.push_back(e.xy);
        statedLanes.push_back({e.lanes.fwd, e.lanes.back});
    }
    citysim::LaneMap lm;
    lm.build(nav, traffic, spines, statedLanes);
    out.linksStated = lm.stateLanes(nav);   // THE NAV'S LANES FROM THE ROAD FILE, as CityRenderSystem sets them
    out.linksDrivable = lm.drivable;
    out.linksMatched = lm.matched;
    out.linksCountDiffers = lm.countDiffers;
    out.linksPartial = lm.partial;
    out.linksNone = lm.none;
    // the lane graph's sections, by road and way: (from, to, lanes)
    std::map<std::pair<int, int>, std::vector<std::tuple<double, double, uint32_t>>> sectionsOf;
    for (std::size_t s = 0; s < lg.sectionSources.size() && s < lg.graph.sections.size(); ++s) {
        const SectionSource& src = lg.sectionSources[s];
        sectionsOf[std::make_pair(src.edge, src.dir)].emplace_back(src.from, src.to, lg.graph.sections[s].lanes.count);
    }
    Examples noLink, lanesDiffer, oneWay, cls, length, unmatched, formsDisagree;
    for (const ModelStreet& S : m.streets) {
        if (S.section.track || S.section.footway || S.section.lanesForward + S.section.lanesBackward == 0) continue;
        const EdgeSpec& e = g.edges[S.edge];
        if (e.xy.size() < 2) continue;
        ++out.streetsCompared;
        const double mid = 0.5 * (S.from + S.to);
        const Vec2 p = pointAt(e.xy, e.s, mid), t = tangentAtStation(e.xy, e.s, mid);
        const int fwd = S.section.lanesForward > 0 ? linkAlong(p, t, true) : -1;
        const int back = linkAlong(p, t, false);
        const int any = fwd >= 0 ? fwd : back;
        if (fwd < 0 && back < 0) {
            ++out.streetsWithoutLink;
            noLink.add(S.to - S.from, fmt("%s (%s, %.0f m) at (%.1f, %.1f)", S.id.c_str(), S.section.roadClass.c_str(), S.to - S.from, p.x, p.y));
            continue;
        }
        bool differs = false;
        std::string what;
        if (fwd >= 0 && nav.links[static_cast<std::size_t>(fwd)].lanes != S.section.lanesForward) { differs = true; what += fmt(" along: nav %d, model %d;", nav.links[static_cast<std::size_t>(fwd)].lanes, S.section.lanesForward); }
        if (S.section.lanesForward > 0 && fwd < 0) { differs = true; what += fmt(" along: no nav link, model %d;", S.section.lanesForward); }
        if (back >= 0 && nav.links[static_cast<std::size_t>(back)].lanes != S.section.lanesBackward) { differs = true; what += fmt(" against: nav %d, model %d;", nav.links[static_cast<std::size_t>(back)].lanes, S.section.lanesBackward); }
        if (S.section.lanesBackward > 0 && back < 0) { differs = true; what += fmt(" against: no nav link, model %d;", S.section.lanesBackward); }
        if (differs) {
            ++out.laneCountDiffers;
            lanesDiffer.add(S.to - S.from, fmt("%s (%s) at (%.1f, %.1f):%s", S.id.c_str(), S.section.roadClass.c_str(), p.x, p.y, what.c_str()));
        }
        const bool navOneWay = (fwd >= 0) != (back >= 0) || nav.links[static_cast<std::size_t>(any)].oneWay;
        if (navOneWay != S.oneWay) {
            ++out.oneWayDiffers;
            oneWay.add(S.to - S.from, fmt("%s at (%.1f, %.1f): model %s, nav %s", S.id.c_str(), p.x, p.y, S.oneWay ? "one-way" : "two-way", navOneWay ? "one-way" : "two-way"));
        }
        if (nav.links[static_cast<std::size_t>(any)].klass != classOf(e)) {
            ++out.classDiffers;
            cls.add(S.to - S.from, fmt("%s at (%.1f, %.1f): model %s (as %s), nav %s", S.id.c_str(), p.x, p.y, S.section.roadClass.c_str(), className(classOf(e)),
                                       className(nav.links[static_cast<std::size_t>(any)].klass)));
        }
        const double chain = chainLength(any), own = S.to - S.from;
        if (std::fabs(chain - own) > std::max(5.0, 0.1 * own)) {
            ++out.lengthDiffers;
            length.add(std::fabs(chain - own), fmt("%s at (%.1f, %.1f): model %.0f m from %s to %s, nav chain %.0f m", S.id.c_str(), p.x, p.y, own,
                                                   S.junctions[0].valid() ? m.junctions[S.junctions[0].index].id.c_str() : "(none)",
                                                   S.junctions[1].valid() ? m.junctions[S.junctions[1].index].id.c_str() : "(none)", chain));
        }
        if ((fwd >= 0 && !lm.has(fwd)) || (back >= 0 && !lm.has(back))) {
            ++out.streetsWithUnmatchedLink;
            unmatched.add(own, fmt("%s (%s) at (%.1f, %.1f): link %s%s", S.id.c_str(), S.section.roadClass.c_str(), p.x, p.y,
                                   fwd >= 0 && !lm.has(fwd) ? "along it" : "", back >= 0 && !lm.has(back) ? " against it" : ""));
        }
        // the old forms against each other: the nav link's lanes along the street against the lane graph's section there
        if (fwd >= 0) {
            const std::map<std::pair<int, int>, std::vector<std::tuple<double, double, uint32_t>>>::const_iterator it = sectionsOf.find(std::make_pair(static_cast<int>(S.edge), 1));
            if (it != sectionsOf.end())
                for (const std::tuple<double, double, uint32_t>& sec : it->second)
                    if (mid >= std::get<0>(sec) && mid <= std::get<1>(sec) && static_cast<int>(std::get<2>(sec)) != nav.links[static_cast<std::size_t>(fwd)].lanes) {
                        ++out.streetsFormsDisagree;
                        formsDisagree.add(own, fmt("%s at (%.1f, %.1f): along it the nav drives %d lanes, the lane graph's section there has %u (the model's class: %d)",
                                                   S.id.c_str(), p.x, p.y, nav.links[static_cast<std::size_t>(fwd)].lanes, std::get<2>(sec), S.section.lanesForward));
                    }
        }
    }
    T += fmt("\nSTREETS. %u of the model's streets that cars drive compared with the nav link along their middle.\n", out.streetsCompared);
    T += fmt("  no nav link along it: %u\n", out.streetsWithoutLink) + noLink.text();
    T += fmt("  lanes each way differ: %u\n", out.laneCountDiffers) + lanesDiffer.text();
    T += fmt("  one-way differs: %u\n", out.oneWayDiffers) + oneWay.text();
    T += fmt("  class differs: %u\n", out.classDiffers) + cls.text();
    T += fmt("  length differs (the nav's chain of links between its junction nodes against the street, by more than 5 m and 10%%): %u\n", out.lengthDiffers) + length.text();
    T += fmt("\nTHE LANE MATCH (the city sim's: which nav links the builder's lanes were found under): %d drivable links, %d matched, %d with "
             "another lane count, %d where a lane starts or ends along the link, %d with no lane found\n",
             lm.drivable, lm.matched, lm.countDiffers, lm.partial, lm.none);
    T += fmt("  model streets whose nav link was not matched: %u\n", out.streetsWithUnmatchedLink) + unmatched.text();

    // ---- the lane count and the kerb parking, by road class (ADR-0418) ------------------------------------------------------
    {
        // the sim's kerbside bays, laid on this nav as the city sim lays them (no agents)
        citysim::CitySim sim;
        sim.build(nav, 0, 0, 5);
        struct Row {
            int links = 0, differ = 0, matched = 0, otherCount = 0, partial = 0, none = 0, band = 0, bays = 0;
            std::map<int, int> navLanes;   // the nav's lanes -> links
        };
        std::map<std::string, Row> rows;
        std::map<std::string, int> noRoad;   // links with no road found under them, by the nav's class
        std::vector<int> baysOn(static_cast<std::size_t>(nav.linkCount()), 0);
        for (const citysim::CitySim::ParkingBay& b : sim.parkingBays())
            if (b.garage < 0 && b.link >= 0 && b.link < nav.linkCount()) ++baysOn[static_cast<std::size_t>(b.link)];
        for (int l = 0; l < nav.linkCount(); ++l) {
            const NavLink& L = nav.links[static_cast<std::size_t>(l)];
            if (L.footpath) continue;
            const int road = lm.roadOf(l);
            if (road < 0 || static_cast<std::size_t>(road) >= g.edges.size()) {
                ++noRoad[className(L.klass)];
                out.baysOffStatedParking += static_cast<uint32_t>(baysOn[static_cast<std::size_t>(l)]);
                continue;
            }
            const EdgeSpec& e = g.edges[static_cast<std::size_t>(road)];
            const double parking = kerbParking(g, e);
            Row& row = rows[fmt("%-10s %d/%d, kerb parking %.1f m", e.cls.c_str(), e.lanes.fwd, e.lanes.back, parking)];
            ++row.links;
            ++row.navLanes[L.lanes];
            const int statedThisWay = lm.wayOf(l) > 0 ? e.lanes.fwd : e.lanes.back;
            row.differ += L.lanes != statedThisWay ? 1 : 0;
            switch (lm.outcomeOf(l)) {
                case citysim::LaneMap::Outcome::Matched: ++row.matched; break;
                case citysim::LaneMap::Outcome::CountDiffers: ++row.otherCount; break;
                case citysim::LaneMap::Outcome::Partial: ++row.partial; break;
                default: ++row.none; break;
            }
            row.band += L.parkWidth > 0 ? 1 : 0;
            const int bays = baysOn[static_cast<std::size_t>(l)];
            row.bays += bays;
            // a bay is only ever in a band its road states: none where it states no parking, and the band's width
            if (bays > 0 && (parking <= 0 || std::fabs(L.parkWidth - parking) > 1e-6)) out.baysOffStatedParking += static_cast<uint32_t>(bays);
        }
        T += fmt("\nLANES EACH WAY AND KERB PARKING BY ROAD CLASS (each drivable nav link, by the road the lane match finds under it; the "
                 "road file's lanes along/against its spine, and its kerb parking as built): %d of %d links carry their road's stated lanes\n",
                 out.linksStated, lm.drivable);
        T += fmt("  %-44s %6s %-18s %7s %8s %8s %8s %6s %6s %6s\n", "class, lanes along/against, kerb parking", "links", "nav lanes (links)",
                 "differ", "matched", "other n", "partial", "none", "band", "bays");
        for (const std::pair<const std::string, Row>& kv : rows) {
            std::string lanes;
            for (const std::pair<const int, int>& n : kv.second.navLanes) lanes += fmt("%s%d: %d", lanes.empty() ? "" : ", ", n.first, n.second);
            const Row& r = kv.second;
            T += fmt("  %-44s %6d %-18s %7d %8d %8d %8d %6d %6d %6d\n", kv.first.c_str(), r.links, lanes.c_str(), r.differ, r.matched, r.otherCount,
                     r.partial, r.none, r.band, r.bays);
        }
        std::string nr;
        for (const std::pair<const std::string, int>& n : noRoad) nr += fmt("%s%d %s", nr.empty() ? "" : ", ", n.second, n.first.c_str());
        T += fmt("  links with no lane at their middle (no road found; they keep their class's lanes): %s\n", nr.empty() ? "none" : nr.c_str());
        T += fmt("  (differ: links whose nav lanes are not their road's stated lanes that way; matched / other n / partial / none: the lane "
                 "match's outcome; band: links with a kerb parking band; bays: the sim's kerbside bays)\n");
        T += fmt("  the sim's kerbside bays: %zu; on a road that states no kerb parking, or off its stated band: %u\n", sim.parkingBays().size(),
                 out.baysOffStatedParking);
    }

    // ---- lanes against the lane graph's lane paths ------------------------------------------------------------------------------
    std::vector<uint32_t> pathsOf(r.lanes.lanes.size(), 0);
    for (std::size_t p = 0; p < lg.sources.size() && p < lg.graph.paths.size(); ++p)
        if (lg.graph.paths[p].kind == PathKind::Lane && lg.sources[p].lane >= 0 && static_cast<std::size_t>(lg.sources[p].lane) < pathsOf.size()) ++pathsOf[static_cast<std::size_t>(lg.sources[p].lane)];
    Examples noPaths, outOfOrder;
    for (std::size_t li = 0; li < m.lanes.size() && li < r.lanes.lanes.size(); ++li) {
        const ModelLane& L = m.lanes[li];
        if (L.edge == kNoIndex) continue;
        const RoadClassSpec& c = g.cls(g.edges[L.edge]);
        if (c.track || c.footway) continue;
        ++out.modelRoadLanes;
        if (pathsOf[li] > 0) ++out.lanesWithPaths;
        else {
            ++out.lanesWithoutPaths;
            noPaths.add(L.spineTo - L.spineFrom, fmt("%s (%s, slot %d, %.0f m)", L.id.c_str(), L.kind.c_str(), L.slot, L.spineTo - L.spineFrom));
        }
    }
    uint32_t sectionsOfTwoRoads = 0;
    for (std::size_t s = 0; s < lg.graph.sections.size(); ++s) {
        const Section& sec = lg.graph.sections[s];
        std::map<uint32_t, int> lastOf;   // per road, the slot of its lane met last (a ramp's lane beside its host is its own road's)
        bool ok = true;
        std::string slots;
        for (uint32_t k = sec.lanes.first; k < sec.lanes.end(); ++k) {
            const PathId p = lg.graph.sectionLanes[k];
            const int li = p.index < lg.sources.size() ? lg.sources[p.index].lane : -1;
            if (li < 0 || static_cast<std::size_t>(li) >= m.lanes.size()) continue;
            const ModelLane& L = m.lanes[static_cast<std::size_t>(li)];
            slots += fmt("%s%s:%d", slots.empty() ? "" : ",", L.id.c_str(), L.slot);
            const std::map<uint32_t, int>::const_iterator was = lastOf.find(L.edge);
            if (was != lastOf.end() && L.slot <= was->second) ok = false;
            lastOf[L.edge] = L.slot;
        }
        if (lastOf.size() > 1) ++sectionsOfTwoRoads;
        if (!ok) {
            ++out.sectionsOutOfOrder;
            const SectionSource& src = lg.sectionSources[s];
            outOfOrder.add(static_cast<double>(sec.lanes.count), fmt("section %zu on %s (way %+d, %.0f to %.0f): model slots offside to kerb %s", s,
                                                                     src.edge >= 0 ? g.edges[static_cast<std::size_t>(src.edge)].id.c_str() : "?", src.dir, src.from, src.to, slots.c_str()));
        }
    }
    T += fmt("\nLANES. %u of the model's lanes on roads cars drive: %u made into lane-graph lane paths, %u into none.\n", out.modelRoadLanes,
             out.lanesWithPaths, out.lanesWithoutPaths) + noPaths.text();
    T += fmt("  lane-graph sections whose lanes of one road, offside to kerb, are not in the model's slot order: %u\n", out.sectionsOutOfOrder) + outOfOrder.text();
    T += fmt("  lane-graph sections holding lanes of two roads or more side by side (a ramp's lane of its own beside its host): %u\n", sectionsOfTwoRoads);
    T += fmt("  (of the lanes made into no lane path: a lane on a road lying wholly inside a junction is on no street in the model either: %u of them)\n",
             [&] {
                 uint32_t n = 0;
                 for (std::size_t li = 0; li < m.lanes.size() && li < pathsOf.size(); ++li)
                     if (m.lanes[li].edge != kNoIndex && pathsOf[li] == 0 && m.lanes[li].streets.empty()) ++n;
                 return n;
             }());

    // ---- where the old forms disagree with each other ---------------------------------------------------------------------
    {
        std::map<std::string, uint32_t> combos;
        Examples ex;
        for (std::size_t j = 0; j < m.junctions.size(); ++j) {
            const ModelJunction& J = m.junctions[j];
            if (J.arms.size() < 3) continue;
            const bool navHas = !pn.ofModel[j].empty(), paintHas = !boxesOf[j].empty(), lgHas = !pl.ofModel[j].empty();
            const std::string key = std::string("nav ") + (navHas ? "yes" : "no") + ", paint " + (paintHas ? "yes" : "no") + ", lane graph " + (lgHas ? "yes" : "no");
            ++combos[key];
            const bool navKnot = navHas && pn.ofOther[pn.ofModel[j].front()].size() >= 2;
            const bool lgKnot = lgHas && pl.ofOther[pl.ofModel[j].front()].size() >= 2;
            if (navHas != paintHas || navHas != lgHas || navKnot != lgKnot || pn.ofModel[j].size() != pl.ofModel[j].size()) {
                ++out.junctionsFormsDisagree;
                ex.add(static_cast<double>(J.arms.size()), fmt("%s at (%.1f, %.1f), %zu arms: %s; nav nodes %zu%s, lane graph junctions %zu%s", J.id.c_str(), J.at.x, J.at.y,
                                                               J.arms.size(), key.c_str(), pn.ofModel[j].size(), navKnot ? " (one knotted with another model junction)" : "",
                                                               pl.ofModel[j].size(), lgKnot ? " (one holding another model junction)" : ""));
            }
        }
        T += "\nWHERE THE OLD FORMS DISAGREE WITH EACH OTHER (the evidence). At the model's junctions of three arms or more, which forms have a junction there:\n";
        for (const std::pair<const std::string, uint32_t>& c : combos) T += fmt("  %-40s %u\n", c.first.c_str(), c.second);
        T += fmt("  junctions where the nav, the paint and the lane graph do not all say the same (one has it and another not, or one knots it with a "
                 "neighbour and another does not, or they split it differently): %u\n", out.junctionsFormsDisagree) + ex.text();
        T += fmt("  streets where the nav's lanes along it and the lane graph's section there differ: %u\n", out.streetsFormsDisagree) + formsDisagree.text();
    }

    // ---- what the model does not state yet ---------------------------------------------------------------------------------
    T += "\nWHAT THE MODEL DOES NOT STATE YET (left empty):\n";
    for (const std::string& s : m.notStated) T += "  - " + s + "\n";
    T += fmt("\nWHERE THE BUILD DOES NOT MATCH WHAT WAS STATED (the model's own problems): %zu\n", m.problems.size());
    {
        Examples ex;
        for (std::size_t i = 0; i < m.problems.size(); ++i) ex.add(-static_cast<double>(i), m.problems[i]);
        T += ex.text();
    }
    T += fmt("\n(nav graph %.1f s, lane graph %.1f s)\n", navSeconds, lgSeconds);
    return out;
}

}  // namespace roads::lanes
}  // namespace engine
