// lanes_tool — the headless host for the lanes road builder (ADR-0083, ADR-0089).
//   lanes_tool build <graph.json> [--out <dir>]     build, print the summary and invariants, write <name>.glb/.svg/stats.json
//                      [--probe x y r] [--quick] [--holes] [--walls]   heights and steps around a point; --quick skips the invariant sweep and the exports; --holes lists unpaved holes; --walls lists parapets that end on drivable road
//   lanes_tool lanegraph <graph.json> [--out <dir>] [--vehicle tag:radius:width[:length] ...] [--junction J ...]   the lane graph
//                      (#237, #238, #239; built from the road model since ADR-0426): lane_graph.txt (its dump), problems.txt,
//                      model_problems.txt (the road model's own), corner_faults.txt, reach.txt (how much a car can get
//                      round, and why not the rest) and lane_graph.svg (lanes by section, junction outlines, connectors by
//                      turn, refused turns as red crosses -- at the corner's middle where the turn fits no vehicle, at the
//                      junction's centre where no lane path takes it -- crossing points by kind); with --junction,
//                      junction_J.svg and junction_J.txt close up
//   lanes_tool terrain <graph.json> <out.json> [step]   the scene's natural ground on a grid (for a map)
//   lanes_tool interchange-demo <out.json>   write the system-interchange test scene: a curved elevated ring over a radial
//                      expressway at ground level, its four ramps, and the expressway's inner leg becoming a boulevard
//   lanes_tool layout <graph.json> [--write] [--fresh] [--names] [--nodes] [--lane-rules M]   the road file's junctions and
//                      streets with ids (ADR-0417); --write puts them into the file in place (the bake key does not see them);
//                      --names names the streets' runs once, from the planner's generator, keeping names the file has
//                      (ADR-0420); --nodes the points of each junction spread over several, once, as the layout finds them
//                      (ADR-0425); --lane-rules M the lane level's rules, the solid stretch before a stop line M metres
//                      (ADR-0426)
//   lanes_tool streetmap <graph.json> [--route x0 z0 x1 z1]   the street map agents route on, built from the file's road
//                      model alone, in words; --route: a car's route between two points, street by street (ADR-0425)
//   lanes_tool model <graph.json> [--out <dir>]   build the roads and assemble THE ROAD MODEL: model.txt, report.txt, model.svg
//   lanes_tool regions-lots <level.json> [--out <dir>]   the level's lots grown again in memory from its bundle (run with
//                      RT_BUNDLE_WRITE=0), dumped beside the stored ones (lots_grown.txt, lots_stored.txt). Writes nothing else.
//   lanes_tool junctions <graph.json> [--top N]   the junctions the file's grouping makes (the control plan's where the file
//                      plans controls, else the layout's), each with its nodes and its size (the most any two nodes lie
//                      apart), the biggest first (ADR-0418)
#include "engine/procgen/city/roads/lanes/control_plan.h"   // junctions: findJunctions, the one grouping
#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/lanes/road_design.h"
#include "engine/procgen/city/roads/lanes/road_layout.h"   // layout: the road file's junctions and streets (ADR-0417)
#include "engine/procgen/city/roads/road_model.h"   // model: the one road model (ADR-0417)
#include "engine/procgen/city/roads/street_map.h"   // streetmap: the graph agents route on, from the model (ADR-0425)
#include "engine/ai/pathfind.h"                     // streetmap --route
#include "engine/procgen/city/roads/lanes/block_audit.h"
#include "engine/procgen/city/roads/lanes/city_producer.h"   // regions-lots: the level's city bundle and its products
#include "engine/procgen/city/roads/lanes/lots_producer.h"   // regions-lots: the lot pass grown again in memory
#include "engine/procgen/city/lot_cache.h"                   // regions-lots: the lots the bundle stores
#include "engine/procgen/city/polygon.h"
#include "engine/procgen/city/roads/lanes/geom2d.h"
#include <set>
#include <map>
#include <array>
#include <tuple>
#include <algorithm>
#include <cmath>
#include "engine/procgen/city/roads/lanes/lanes_export.h"
#include "engine/procgen/city/roads/lane_turns.h"            // turnKey
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"
#include "engine/procgen/city/roads/lanes/interchange.h"
#include "engine/procgen/city/roads/lanes/terrain_recipe.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>

// Whether writing `out` over `text` (a road file's bytes) keeps the city's bake key: the bytes it hashes
// (keyedRoadFileText) and what the paint reads of the layout (deck_mesh.h paintFactsKey, ADR-0431) are both unchanged.
static bool bakeKeyUnchanged(const std::string& out, const std::string& text) {
    using namespace engine::roads::lanes;
    if (keyedRoadFileText(out) != keyedRoadFileText(text)) return false;
    const nlohmann::json a = nlohmann::json::parse(out, nullptr, false), b = nlohmann::json::parse(text, nullptr, false);
    return paintFactsKey(a) == paintFactsKey(b);
}
#include <string>
#include <sys/stat.h>

using namespace engine;
using namespace engine::roads::lanes;

namespace {

// The --out folder, made with every folder above it (`out/lanelab/x` when `out/lanelab` is not there yet). Says so
// and returns false when it cannot be made: a plain mkdir made only the last folder, failed silently, and the tool
// went on to print "wrote ..." for files it had not written.
bool makeOutDir(const std::string& dir) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (!ec && std::filesystem::is_directory(dir, ec)) return true;
    std::fprintf(stderr, "lanes: cannot make the output folder %s%s%s\n", dir.c_str(), ec ? ": " : "", ec ? ec.message().c_str() : "");
    return false;
}

// THE SYSTEM-INTERCHANGE SCENE, the city planner's geometry in miniature: a ring of radius 1000
// held 8 m up, crossed square by a radial expressway at ground level, with the ring's frontage
// roads cut back round the crossing. Outside, the expressway climbs back to 8 m to bridge a
// street; inside, it ends 150 m in and its carriageways carry on as a one-way boulevard pair to
// a boulevard 220 m inside the ring — midtown's, in the city.
nlohmann::json interchangeDemo() {
    using nlohmann::json;
    auto pts = [](const std::vector<Vec2>& xy) { json a = json::array(); for (const Vec2& p : xy) a.push_back({std::round(p.x * 100) / 100, std::round(p.y * 100) / 100}); return a; };
    auto arc = [](double r, double a0, double a1) {   // the ring: centre (0, -1000), clockwise with increasing station
        std::vector<Vec2> out; const int n = static_cast<int>(std::fabs(a1 - a0) * r / 8.0);
        for (int i = 0; i <= n; ++i) { const double a = a0 + (a1 - a0) * i / n; out.push_back(Vec2(r * std::cos(a), -1000.0 + r * std::sin(a))); }
        return out;
    };
    const double lanes = 4, laneW = 3.75, carriageHalf = lanes * laneW / 2 + 2.5, carriage = carriageHalf + 2.0;   // a 4 m median
    json scene;
    scene["name"] = "interchange_demo";
    scene["terrain"] = {{"type", "flat"}, {"bounds", {-900.0, 900.0, -600.0, 900.0}}, {"res", 3.0}};
    // the road class table's rows (ADR-0276), the freeway at this demo's lane width
    roads::lanes::RoadClassSpec fw = roads::lanes::roadClass("freeway");
    fw.lanes.w = laneW;
    scene["classes"] = {{"freeway", roads::lanes::classJson(fw)},
                        {"ramp", roads::lanes::classJson(roads::lanes::roadClass("ramp"))},
                        {"boulevard", roads::lanes::classJson(roads::lanes::roadClass("boulevard"))},
                        {"arterial", roads::lanes::classJson(roads::lanes::roadClass("arterial"))},
                        {"collector", roads::lanes::classJson(roads::lanes::roadClass("collector"))}};
    scene["rules"] = {{"closing", 3.0}};
    json edges = json::array();
    // the ring, 8 m up the whole way
    const double a0 = M_PI / 2 + 0.75, a1 = M_PI / 2 - 0.75;
    std::vector<Vec2> ra = arc(1000.0 - carriage, a0, a1), rb = arc(1000.0 + carriage, a1, a0);
    json ringFloor = json::array(); for (const Vec2& p : arc(1000.0, a0, a1)) ringFloor.push_back({p.x, p.y, 8.0, 8.0});
    edges.push_back({{"id", "ring_a"}, {"class", "freeway"}, {"floor", ringFloor}, {"path", {{"points", pts(ra)}}}});
    edges.push_back({{"id", "ring_b"}, {"class", "freeway"}, {"floor", ringFloor}, {"path", {{"points", pts(rb)}}}});
    // the expressway, inward along -y: at ground through the interchange, 8 m up beyond 420 m out
    std::vector<Vec2> stem; for (double y = 800; y >= -150; y -= 8) stem.push_back(Vec2(0, y));
    json stemFloor = json::array(); for (double y = 800; y >= 420; y -= 30) stemFloor.push_back({0.0, y, 8.0, 15.0});
    std::vector<Vec2> sa, sb; for (const Vec2& p : stem) { sa.push_back(p + Vec2(-carriage, 0)); sb.push_back(p + Vec2(carriage, 0)); }
    std::reverse(sb.begin(), sb.end());
    edges.push_back({{"id", "stem_a"}, {"class", "freeway"}, {"floor", stemFloor}, {"path", {{"points", pts(sa)}}}});
    edges.push_back({{"id", "stem_b"}, {"class", "freeway"}, {"floor", stemFloor}, {"path", {{"points", pts(sb)}}}});
    // inside: the carriageways carry on as a one-way boulevard pair that splays to meet
    // midtown's boulevard at two Ts 40 m apart. (Closing the pair into one two-way road puts two
    // edges' lanes on one centreline at the node, and they overlap there by construction.)
    const double tee = 20.0;
    edges.push_back({{"id", "blvd_in"}, {"class", "boulevard"}, {"path", {{"type", "bezier"}, {"points", pts({Vec2(-carriage, -150), Vec2(-carriage, -180), Vec2(-tee, -190), Vec2(-tee, -220)})}}}});
    edges.push_back({{"id", "blvd_out"}, {"class", "boulevard"}, {"path", {{"type", "bezier"}, {"points", pts({Vec2(tee, -220), Vec2(tee, -190), Vec2(carriage, -180), Vec2(carriage, -150)})}}}});
    edges.push_back({{"id", "mid_w"}, {"class", "arterial"}, {"path", {{"points", pts({Vec2(-400, -220), Vec2(-tee, -220)})}}}});
    edges.push_back({{"id", "mid_c"}, {"class", "arterial"}, {"path", {{"points", pts({Vec2(-tee, -220), Vec2(tee, -220)})}}}});
    edges.push_back({{"id", "mid_e"}, {"class", "arterial"}, {"path", {{"points", pts({Vec2(tee, -220), Vec2(400, -220)})}}}});
    // the frontage roads, 50 m either side of the ring, cut back from the crossing
    for (double side : {-1.0, 1.0}) {
        const double r = 1000.0 + side * 50.0, gap = 330.0 / r;
        edges.push_back({{"id", side < 0 ? "front_in_w" : "front_out_w"}, {"class", "collector"}, {"path", {{"points", pts(arc(r, a0, M_PI / 2 + gap))}}}});
        edges.push_back({{"id", side < 0 ? "front_in_e" : "front_out_e"}, {"class", "collector"}, {"path", {{"points", pts(arc(r, M_PI / 2 - gap, a1))}}}});
    }
    // a street the expressway bridges, outside
    edges.push_back({{"id", "street_out"}, {"class", "collector"}, {"path", {{"points", pts({Vec2(-600, 600), Vec2(600, 600)})}}}});

    SystemRoad ring; ring.route = arc(1000.0, a0, a1); ring.aId = "ring_a"; ring.bId = "ring_b";
    ring.carriage = carriage; ring.edgeReach = carriage + carriageHalf; ring.lanes = 4; ring.laneW = laneW;
    SystemRoad rad; rad.route = stem; rad.aId = "stem_a"; rad.bId = "stem_b";
    rad.carriage = carriage; rad.edgeReach = carriage + carriageHalf; rad.lanes = 4; rad.laneW = laneW;
    SystemOptions so; so.idPrefix = "x0";
    const SystemResult sr = systemInterchange(ring, rad, -1.0, so);   // the outer leg runs back up the stem's stations
    if (!sr.built) std::fprintf(stderr, "interchange-demo: no interchange: %s\n", sr.why.c_str());
    for (const json& r : sr.ramps) edges.push_back(r);
    scene["edges"] = edges;
    return scene;
}

// EVERY LOT IN WORDS (regions-lots): its kind, recipe, block, site and size, its lot line, its plan and every door with
// where it faces, to the tenth of a millimetre -- two grows of the same city compare byte for byte.
std::string lotDump(const std::vector<LotBuilding>& lots) {
    std::string out;
    char buf[512];
    auto ring = [&](const char* what, const std::vector<Vec2>& pts) {
        out += what;
        for (const Vec2& p : pts) {
            std::snprintf(buf, sizeof buf, " (%.4f, %.4f)", p.x, p.y);
            out += buf;
        }
        out += "\n";
    };
    for (std::size_t i = 0; i < lots.size(); ++i) {
        const LotBuilding& b = lots[i];
        std::snprintf(buf, sizeof buf, "lot %zu: %s %s block %d district %s site (%.4f, %.4f) yaw %.5f size %.4f x %.4f x %.4f base %.4f ground %.4f units %zu\n",
                      i, b.type.c_str(), b.recipe.c_str(), b.block, b.district.c_str(), b.site.x, b.site.y, b.yaw, b.width, b.depth, b.height, b.baseY,
                      b.groundY, b.units.size());
        out += buf;
        ring("  lot line:", b.lot);
        ring("  plan:", b.plan);
        for (const BuildingUnit& u : b.units)
            for (const DoorSpec& d : u.doors) {
                std::snprintf(buf, sizeof buf, "  door at (%.4f, %.4f) facing (%.5f, %.5f) %.3f x %.3f%s%s\n", d.foot.x, d.foot.y, d.normal.x, d.normal.y,
                              d.width, d.height, d.back ? " back" : "", d.open ? " open" : "");
                out += buf;
            }
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc >= 4 && std::strcmp(argv[1], "terrain") == 0) {
        // the scene's natural ground (before any road conforms it), every `step` m, as JSON
        std::ifstream f(argv[2]); nlohmann::json spec; f >> spec;
        const RoadLabGraph g = RoadLabGraph::fromJson(spec);
        const std::array<double, 4> b = g.terrain.bounds;
        const HeightField h = makeTerrain(g.terrain, b);
        const double step = argc >= 5 ? std::atof(argv[4]) : 20.0;
        nlohmann::json out = {{"x0", b[0]}, {"y0", b[2]}, {"step", step}};
        std::vector<std::vector<double>> rows;
        for (double y = b[2]; y <= b[3]; y += step) { rows.emplace_back(); for (double x = b[0]; x <= b[1]; x += step) rows.back().push_back(h(x, y)); }
        out["z"] = rows;
        std::ofstream(argv[3]) << out.dump() << "\n";
        std::printf("wrote %s: %zu x %zu\n", argv[3], rows.empty() ? 0 : rows[0].size(), rows.size());
        return 0;
    }
    if (argc >= 3 && std::strcmp(argv[1], "landings") == 0) {
        // THE RAMPS' LANDINGS (#282): each ramp end on a street checked as build() checks it, without the build -- the
        // lanes composed, no heights, so a planned crossing's clearance is not measured here. Every graph named.
        int faults = 0, planned = 0;
        for (int i = 2; i < argc; ++i) {
            try {
                RoadLabGraph g = RoadLabGraph::load(argv[i]);
                (void)expand(g);
                int ramps = 0;
                for (const EdgeSpec& e : g.edges) ramps += e.isRamp() ? 1 : 0;
                const std::vector<RampLandingFault> fs = checkRampLandings(g);
                std::printf("%s: %d ramps, %zu ends land wrongly\n", argv[i], ramps, fs.size());
                for (const RampLandingFault& f : fs) { std::printf("  %s\n", describe(f).c_str()); planned += f.declared ? 1 : 0; }
                faults += static_cast<int>(fs.size());
                // and the frontage roads the ramps land on, as planned (ADR-0413): always planned, so always refused
                int frontage = 0;
                for (const EdgeSpec& e : g.edges) frontage += e.frontageOf.empty() ? 0 : 1;
                const std::vector<FrontageFault> ffs = checkFrontageRoads(g);
                if (frontage > 0) std::printf("%s: %d frontage road pieces, %zu run wrongly\n", argv[i], frontage, ffs.size());
                for (const FrontageFault& f : ffs) std::printf("  %s\n", describe(f).c_str());
                faults += static_cast<int>(ffs.size());
                planned += static_cast<int>(ffs.size());
            } catch (const std::exception& ex) { std::fprintf(stderr, "lanes: %s: %s\n", argv[i], ex.what()); }
        }
        std::printf("%d ramp ends land wrongly (or frontage roads run wrongly) in all, %d of them planned (build() refuses those)\n", faults, planned);
        return 0;
    }
    if (argc >= 3 && std::strcmp(argv[1], "junctions") == 0) {
        // THE JUNCTIONS AS THE FILE'S GROUPING MAKES THEM (control_plan.h findJunctions): the control plan's where the file
        // plans controls (city_plan controls finds them so), else the layout's (lanes_tool layout --fresh); each with its
        // nodes and its size, the biggest first. One line per junction: "junction <id> at <x> <y> nodes <n> size <m> roads
        // <a,b,..> points <x,y> ..." (a script reads them to draw the close-ups).
        std::size_t top = 20;
        for (int i = 3; i + 1 < argc; ++i)
            if (std::strcmp(argv[i], "--top") == 0) top = static_cast<std::size_t>(std::atoi(argv[i + 1]));
        std::vector<JunctionFacts> js;
        bool controls = false;
        try {
            const RoadLabGraph g = RoadLabGraph::load(argv[2]);
            controls = g.declaresJunctions;
            js = controls ? findJunctions(g, ControlPolicy()) : layoutJunctionFacts(g, LayoutSource::Inferred);
        } catch (const std::exception& ex) { std::fprintf(stderr, "lanes_tool: %s: %s\n", argv[2], ex.what()); return 1; }
        auto sizeOf = [](const JunctionFacts& j) {
            double s = 0;
            for (const Vec2& a : j.nodes)
                for (const Vec2& b : j.nodes) s = std::max(s, (a - b).length());
            return s;
        };
        std::vector<std::size_t> order(js.size());
        for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
        std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return sizeOf(js[a]) > sizeOf(js[b]); });
        std::size_t multi = 0;
        for (const JunctionFacts& j : js) multi += j.nodes.size() > 1 ? 1 : 0;
        std::printf("%s: %zu junctions (%s's grouping), %zu of them of two nodes or more\n", argv[2], js.size(),
                    controls ? "the control plan" : "the layout", multi);
        for (std::size_t k = 0; k < order.size() && k < top; ++k) {
            const JunctionFacts& j = js[order[k]];
            std::set<std::string> roads;
            for (const ArmFacts& a : j.arms) roads.insert(a.road);
            for (const ModelStretch& st : j.inside) roads.insert(st.road);
            std::string rs, pts;
            for (const std::string& r : roads) rs += (rs.empty() ? "" : ",") + r;
            for (const Vec2& p : j.nodes) {
                char b[64];
                std::snprintf(b, sizeof b, " %.1f,%.1f", p.x, p.y);
                pts += b;
            }
            std::printf("junction %s at %.1f %.1f nodes %zu size %.1f arms %zu roads %s points%s\n", j.id.c_str(), j.at.x, j.at.y,
                        j.nodes.size(), sizeOf(j), j.arms.size(), rs.c_str(), pts.c_str());
        }
        return 0;
    }
    if (argc >= 3 && std::strcmp(argv[1], "layout") == 0) {
        // THE ROAD FILE'S LAYOUT (road_layout.h, ADR-0417): its junctions and streets with ids, derived ONCE from what the
        // file holds today (LayoutSource::Inferred: ends within the endpoint tolerance, the planner's crossing policy where
        // the file declares no crossings), printed; --write puts it into the file in place, nothing else moving by a byte,
        // for a person to read and correct. A layout the file already has is kept (--fresh derives it again); a control
        // plan's junctions list is kept as stated and given its streets.
        // --names (ADR-0420): the streets' names, ONCE, from the planner's generator (street_names.h) over the runs the file
        // states (through pairs, carries_on, divided roads), written into its streets in place for a person to read and
        // edit; a name the file already has is kept. A layout written again keeps the names the file had.
        // --nodes (ADR-0425): the points of each junction the layout spread over several (which roads meet at each), ONCE,
        // as the layout finds them, written into the file's own junctions in place for a person to read: what the street
        // map joins a spread junction's arms and inside stretches by.
        // --lane-rules M (ADR-0426): the lane level's rules, ONCE, written into the file for a person to read and edit: how far
        // back from a stop line the lines between lanes are solid (M metres). Nothing else moves; the bake key does not see it.
        // --arm-order (ADR-0427): THE CHECK -- each listed junction whose stated ring (its arms' list order) differs from the
        // one measure (road_layout.h armRing, on the roads composed), and the measure's notes; exits 3 when any differs or the
        // file is not marked "arms_in_ring". With --write: each listed junction's arms put in ring order by the measure,
        // their texts moved whole, the through pairs renumbered, and the file marked; nothing else moves.
        bool write = false, fresh = false, names = false, nodes = false, armOrder = false;
        double laneRules = -1;
        for (int i = 3; i < argc; ++i) {
            if (std::strcmp(argv[i], "--arm-order") == 0) armOrder = true;
            if (std::strcmp(argv[i], "--write") == 0) write = true;
            if (std::strcmp(argv[i], "--fresh") == 0) fresh = true;
            if (std::strcmp(argv[i], "--names") == 0) names = true;
            if (std::strcmp(argv[i], "--nodes") == 0) nodes = true;
            if (std::strcmp(argv[i], "--lane-rules") == 0 && i + 1 < argc) laneRules = std::atof(argv[i + 1]);
        }
        std::string text;
        {
            std::ifstream f(argv[2], std::ios::binary);
            if (!f) { std::fprintf(stderr, "lanes_tool: cannot read %s\n", argv[2]); return 1; }
            std::ostringstream ss;
            ss << f.rdbuf();
            text = ss.str();
        }
        if (armOrder) {
            // THE RING CHECKED (never a gate: a marked file is trusted at load), or with --write put in ring order and marked
            const std::string dir = std::filesystem::path(argv[2]).parent_path().string();
            RoadLabGraph stated;
            try {
                stated = RoadLabGraph::fromJson(nlohmann::json::parse(text), dir.empty() ? "." : dir);
            } catch (const std::exception& ex) {
                std::fprintf(stderr, "lanes_tool: %s: %s\n", argv[2], ex.what());
                return 1;
            }
            if (!write) {
                const ArmRingCheck c = checkArmRings(stated);
                for (const std::string& n : c.notes) std::printf("  note: %s\n", n.c_str());
                std::printf("%s: %s; %zu of %zu junctions' stated rings differ from the measure%s", argv[2],
                            stated.armsInRing ? "marked \"arms_in_ring\"" : "NOT marked \"arms_in_ring\"", c.differ.size(),
                            stated.junctions.size(), c.differ.empty() ? "" : ":");
                for (const std::string& id : c.differ) std::printf(" %s", id.c_str());
                std::printf("\n");
                return c.differ.empty() && stated.armsInRing ? 0 : 3;
            }
            std::string err;
            std::vector<std::string> changed, notes;
            const std::string out = insertArmOrderText(text, stated, &changed, &err, &notes);
            if (out.empty()) { std::fprintf(stderr, "lanes_tool: %s: %s\n", argv[2], err.c_str()); return 1; }
            for (const std::string& n : notes) std::printf("  note: %s\n", n.c_str());
            std::printf("%s: %zu junctions' arms put in ring order:", argv[2], changed.size());
            for (const std::string& id : changed) std::printf(" %s", id.c_str());
            std::printf("\n");
            try {
                const RoadLabGraph again = RoadLabGraph::fromJson(nlohmann::json::parse(out), dir.empty() ? "." : dir);
                const ArmRingCheck c = checkArmRings(again);
                if (!again.armsInRing || !c.differ.empty()) {
                    std::fprintf(stderr, "lanes_tool: %s: read back, %s; not written\n", argv[2],
                                 again.armsInRing ? ("junction " + c.differ.front() + " still differs from the measure").c_str() : "not marked");
                    return 1;
                }
            } catch (const std::exception& ex) {
                std::fprintf(stderr, "lanes_tool: %s: would not read back (%s); not written\n", argv[2], ex.what());
                return 1;
            }
            std::ofstream(argv[2], std::ios::binary) << out;
            std::printf("wrote the ring order and \"arms_in_ring\": true into %s (the bytes the bake keys are unchanged: %s)\n", argv[2],
                        bakeKeyUnchanged(out, text) ? "yes" : "NO");
            return 0;
        }
        if (laneRules >= 0) {
            std::string err;
            const std::string out = insertLaneRulesText(text, laneRules, &err);
            if (out.empty()) { std::fprintf(stderr, "lanes_tool: %s: %s\n", argv[2], err.c_str()); return 1; }
            const std::string dir = std::filesystem::path(argv[2]).parent_path().string();
            try { (void)RoadLabGraph::fromJson(nlohmann::json::parse(out), dir.empty() ? "." : dir); }
            catch (const std::exception& ex) { std::fprintf(stderr, "lanes_tool: %s: would not read back (%s); not written\n", argv[2], ex.what()); return 1; }
            std::printf("%s: lane rules: the lines between lanes solid %.1f m before a stop line%s\n", argv[2], laneRules,
                        write ? "" : " (not written: --write)");
            if (!write) return 0;
            std::ofstream(argv[2], std::ios::binary) << out;
            std::printf("wrote \"lane_rules\" into %s (%zu bytes more; the bytes the bake keys are unchanged: %s)\n", argv[2],
                        out.size() - text.size(), bakeKeyUnchanged(out, text) ? "yes" : "NO");
            return 0;
        }
        const std::string base = stripLayoutText(text);
        const std::string dir = std::filesystem::path(argv[2]).parent_path().string();
        RoadLayout layout;
        std::vector<NamedStretch> keep;
        try {
            const RoadLabGraph stated = RoadLabGraph::fromJson(nlohmann::json::parse(text), dir.empty() ? "." : dir);
            keep = namedStretches(stated.streets);
            if (nodes) {
                std::vector<std::string> notes;
                const std::vector<JunctionNodes> found = junctionNodes(stated, LayoutSource::Inferred, &notes);
                std::size_t spread = 0, points = 0, listed = 0;
                for (const JunctionNodes& jn : found) {
                    spread += 1;
                    points += jn.nodes.size();
                }
                for (const ModelJunction& pj : stated.junctions) listed += pj.nodes.empty() ? 0 : 1;
                std::printf("%s: %zu junctions over several points, %zu points found for them (%zu junctions list theirs already)\n", argv[2],
                            spread, points, listed);
                for (const std::string& n : notes) std::printf("  note: %s\n", n.c_str());
                if (!write) return 0;
                // (a node no junction of the file holds is reported and left out; an arm or an inside stretch at no node, or
                // at two, is a node list that does not fit the file: refused)
                for (const std::string& n : notes)
                    if (n.rfind("junction ", 0) == 0) { std::fprintf(stderr, "lanes_tool: %s: not written: the nodes do not fit the file's junctions (notes above)\n", argv[2]); return 1; }
                std::string err;
                const std::string out = insertJunctionNodesText(text, stated, found, &err);
                if (out.empty()) { std::fprintf(stderr, "lanes_tool: %s: not written: %s\n", argv[2], err.c_str()); return 1; }
                try { (void)RoadLabGraph::fromJson(nlohmann::json::parse(out), dir.empty() ? "." : dir); }
                catch (const std::exception& ex) { std::fprintf(stderr, "lanes_tool: %s: would not read back (%s); not written\n", argv[2], ex.what()); return 1; }
                std::ofstream(argv[2], std::ios::binary) << out;
                std::printf("wrote the nodes into %s (%zu bytes more; the bytes the bake keys are unchanged: %s; the lane graph's key is unchanged: %s)\n",
                            argv[2], out.size() - text.size(), bakeKeyUnchanged(out, text) ? "yes" : "NO",
                            laneLevelKey(nlohmann::json::parse(out)) == laneLevelKey(nlohmann::json::parse(text)) ? "yes" : "NO");
                return 0;
            }
            if (names && !fresh && stated.listsStreets) {
                // the file's own streets, as they are: only their names are added
                std::vector<ModelStreet> streets = stated.streets;
                const NamingReport rep = nameStreetRuns(streets, stated.junctions, stated.divided, stated, streetNameBook(), keep);
                std::printf("%s: %u runs named over %u streets (%u kept, %u new)\n", argv[2], rep.runs, rep.streets, rep.kept, rep.generated);
                if (!write) return 0;
                std::string err;
                const std::string out = insertStreetNamesText(text, streets, &err);
                if (out.empty()) { std::fprintf(stderr, "lanes_tool: %s: not written: %s\n", argv[2], err.c_str()); return 1; }
                (void)RoadLabGraph::fromJson(nlohmann::json::parse(out), dir.empty() ? "." : dir);
                std::ofstream(argv[2], std::ios::binary) << out;
                std::printf("wrote the names into %s (%zu bytes more; the bytes the bake keys are unchanged: %s)\n", argv[2], out.size() - text.size(),
                            stripLayoutText(out) == base ? "yes" : "NO");
                return 0;
            }
            const nlohmann::json spec = nlohmann::json::parse(fresh ? base : text);
            const RoadLabGraph g = RoadLabGraph::fromJson(spec, dir.empty() ? "." : dir);
            layout = deriveLayout(g, LayoutSource::Inferred);
            if (names || !keep.empty()) {
                const NamingReport rep = nameLayout(layout, g, keep);
                std::printf("%s: names: %u runs (%u kept, %u new)\n", argv[2], rep.runs, rep.kept, rep.generated);
            }
        } catch (const std::exception& ex) { std::fprintf(stderr, "lanes_tool: %s: %s\n", argv[2], ex.what()); return 1; }
        std::printf("%s: %s", argv[2], describeLayout(layout).c_str());
        if (!write) return 0;
        std::string err;
        const std::string out = insertLayoutText(base, layout, &err);
        if (out.empty()) { std::fprintf(stderr, "lanes_tool: %s: not written: %s\n", argv[2], err.c_str()); return 1; }
        try { (void)RoadLabGraph::fromJson(nlohmann::json::parse(out), dir.empty() ? "." : dir); }
        catch (const std::exception& ex) { std::fprintf(stderr, "lanes_tool: %s: the file written would not read back (%s); not written\n", argv[2], ex.what()); return 1; }
        {
            std::ofstream f(argv[2], std::ios::binary);
            f << out;
        }
        std::printf("wrote the layout into %s (%zu bytes more; the bytes the bake keys are unchanged: %s)\n", argv[2], out.size() - base.size(),
                    stripLayoutText(out) == base ? "yes" : "NO");
        return 0;
    }
    if (argc >= 3 && std::strcmp(argv[1], "streetmap") == 0) {
        // THE STREET MAP (street_map.h, ADR-0425): the graph agents route on, built from the road file's planned layer alone
        // (no build: no deck heights), in words -- its counts and every place the stated facts leave unjoined
        try {
            RoadLabGraph g = RoadLabGraph::load(argv[2]);
            (void)expand(g);   // the ramps composed, as the loader reads them
            const RoadModel model = plannedRoadModel(g);
            NavGraph map;
            StreetMapReport rep;
            const bool ok = appendStreetMap(map, model, 0, nullptr, &rep);
            std::printf("%s: %s\n", argv[2], describeStreetMap(rep).c_str());
            // --route x0 z0 x1 z1: the car's route between the nodes nearest two points, street by street, with its time
            for (int i = 3; i + 4 < argc; ++i) {
                if (std::strcmp(argv[i], "--route") != 0) continue;
                const int a = map.nearestNode(Vec2(std::atof(argv[i + 1]), std::atof(argv[i + 2])));
                const int b = map.nearestNode(Vec2(std::atof(argv[i + 3]), std::atof(argv[i + 4])));
                const Route r = findRoute(map, a, b, false);
                std::printf("route %d -> %d: %zu links, %.1f m, %.1f s\n", a, b, r.links.size(), r.length(map), routeSeconds(map, r, false));
                for (std::size_t k = 0; k < r.links.size(); ++k) {
                    const NavLink& L = map.links[static_cast<std::size_t>(r.links[k])];
                    const Vec2 p = map.nodes[static_cast<std::size_t>(L.from)];
                    std::printf("  %s %s (%.0f, %.0f) %.0f m, %.1f s%s\n",
                                L.street >= 0 ? model.streets[static_cast<std::size_t>(L.street)].id.c_str() : "inside",
                                L.junction >= 0 ? model.junctions[static_cast<std::size_t>(L.junction)].id.c_str() : "", p.x, p.y, L.length,
                                linkTravelSeconds(map, k > 0 ? r.links[k - 1] : -1, r.links[k], b, false), map.isJunction(L.to) ? " (junction)" : "");
                }
            }
            return ok ? 0 : 1;
        } catch (const std::exception& e) { std::fprintf(stderr, "lanes_tool: %s\n", e.what()); return 1; }
    }
    if (argc >= 3 && std::strcmp(argv[1], "model") == 0) {
        // THE ROAD MODEL (road_model.h, ADR-0417): the roads built, the model assembled from the build; `model` writes its
        // dump (model.txt), its report (report.txt: what it holds, what is not stated, where the build does not match) and
        // a map (model.svg).
        std::string outDir = ".";
        for (int i = 3; i + 1 < argc; ++i)
            if (std::strcmp(argv[i], "--out") == 0) outDir = argv[i + 1];
        if (!makeOutDir(outDir)) return 1;
        const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        std::unique_ptr<Result> rp;
        try { rp = build(RoadLabGraph::load(argv[2])); } catch (const std::exception& e) { std::fprintf(stderr, "lanes: %s\n", e.what()); return 1; }
        const double buildSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        const RoadModel model = assembleRoadModel(*rp, LaneGraphOptions());   // (no vehicles named: `lanegraph` is the lane level's command)
        const double modelSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
        std::printf("%s: built in %.1f s, the model assembled in %.2f s: %zu junctions, %zu streets, %zu lanes, %zu connections, %zu problems\n", argv[2],
                    buildSeconds, modelSeconds, model.junctions.size(), model.streets.size(), model.lanes.size(), model.connections.size(),
                    model.problems.size());
        std::ofstream(outDir + "/model.txt") << dumpRoadModel(model);
        std::ofstream(outDir + "/report.txt") << reportRoadModel(model);
        std::ofstream(outDir + "/model.svg") << roadModelSvg(model);
        std::printf("wrote %s/model.txt, report.txt, model.svg\n", outDir.c_str());
        return 0;
    }
    if (argc >= 3 && std::strcmp(argv[1], "regions-lots") == 0) {
        // THE LOTS GROWN AGAIN (moves nothing, saves nothing but the files named here): the
        // level's city bundle as it is (run with RT_BUNDLE_WRITE=0: a bundle that is not current is baked in memory, never
        // written), its lots grown again in memory by the existing lot pass from the bundle's own city products, every lot
        // dumped (lots_grown.txt) beside the lots the bundle stores (lots_stored.txt)
        std::string outDir = ".";
        for (int i = 3; i + 1 < argc; ++i)
            if (std::strcmp(argv[i], "--out") == 0) outDir = argv[i + 1];
        if (!makeOutDir(outDir)) return 1;
        bundle::LevelInputs in;
        std::string err;
        if (!bundle::loadLevelInputs(argv[2], in, &err)) { std::fprintf(stderr, "lanes_tool: %s: %s\n", argv[2], err.c_str()); return 1; }
        const std::vector<CityEntity> ents = cityEntities(in.level);
        if (ents.empty()) { std::fprintf(stderr, "lanes_tool: %s has no lane-built city\n", argv[2]); return 1; }
        std::string status;
        const std::shared_ptr<bundle::Bundle> b = levelCityBundle(in, &status);
        std::printf("bundle: %s\n", status.c_str());
        if (!b) return 1;
        CityProducts p;
        if (!readCityProducts(*b, ents.back().ordinal, p, &err)) { std::fprintf(stderr, "lanes_tool: %s\n", err.c_str()); return 1; }
        NetLotResult stored;
        if (!lotcache::readLotResult(*b, kLotsSectionPrefix, stored, &err, false)) { std::fprintf(stderr, "lanes_tool: stored lots: %s\n", err.c_str()); return 1; }
        const LotsCityInputs city = lotsCityInputs(p);   // the lots producer's inputs, from the same sections it reads
        const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        const NetLotResult grown = growLotsForLevel(in, city);
        std::printf("grew %zu lots in memory in %.1f s (the bundle stores %zu)\n", grown.lots.size(),
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), stored.lots.size());
        const std::string dumpGrown = lotDump(grown.lots), dumpStored = lotDump(stored.lots);
        std::ofstream(outDir + "/lots_grown.txt") << dumpGrown;
        std::ofstream(outDir + "/lots_stored.txt") << dumpStored;
        std::printf("the lots grown in memory and the lots the bundle stores dump %s\n", dumpGrown == dumpStored ? "byte-identical" : "DIFFERENTLY");
        std::printf("wrote %s/lots_grown.txt, lots_stored.txt\n", outDir.c_str());
        return 0;
    }
    if (argc == 3 && std::strcmp(argv[1], "interchange-demo") == 0) {
        std::ofstream(argv[2]) << interchangeDemo().dump(1) << "\n";
        std::printf("wrote %s\n", argv[2]);
        return 0;
    }
    if (argc >= 4 && std::strcmp(argv[1], "design") == 0) {
        // THE DESIGN ALONE (ADR-0215): the graph with every road's planned profile written in, no meshing -- built
        // again, the builder takes those heights as given and checks them
        const auto t0 = std::chrono::steady_clock::now();
        nlohmann::json spec;
        { std::ifstream f(argv[2]); if (!f) { std::fprintf(stderr, "lanes_tool: cannot read %s\n", argv[2]); return 1; } f >> spec; }
        const std::string dir = std::filesystem::path(argv[2]).parent_path().string();
        // --plan-crossings: every crossing given its planned kind by the planner's policy first (planCrossings)
        for (int i = 4; i < argc; ++i)
            if (std::strcmp(argv[i], "--plan-crossings") == 0) {
                spec["crossings"] = planCrossings(RoadLabGraph::fromJson(spec, dir.empty() ? "." : dir));
                std::printf("planned %zu crossings\n", spec["crossings"].size());
            }
        DesignReport rep;
        const nlohmann::json out = designedGraph(spec, dir.empty() ? "." : dir, &rep);
        std::ofstream(argv[3]) << out.dump(1) << "\n";
        std::printf("designed %s -> %s in %.1f s: node mismatch %.1f cm (was %.1f), %d crossings lifted, %zu not in the plan\n", argv[2], argv[3],
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), 100 * rep.nodeMismatchAfter, 100 * rep.nodeMismatchBefore,
                    rep.lifted, rep.undeclared.size());
        for (std::size_t i = 0; i < rep.undeclared.size() && (i < 20 || std::getenv("LANES_ALL_UNDECLARED")); ++i) std::printf("  not in the plan: %s\n", rep.undeclared[i].c_str());
        return 0;
    }
    if (argc >= 3 && std::strcmp(argv[1], "lanegraph") == 0) {
        // THE LANE GRAPH (#237, #238): the built lanes cut into sections, the junctions and the connectors through them;
        // its text dump, its problems, how much of it a car can get round, and a plan SVG in --out.
        // --vehicle tag:radius:width[:length] (again for each): a vehicle tag every lane is open to, the tightest radius
        // its path may have, its width and its body's length (metres; none: swept as a point). None given: the tool's two
        // test vehicles below.
        // --junction J (again for each): also junction_J.svg, the plan close up round junction J, and junction_J.txt, its
        // connectors and every crossing point on them.
        std::string outDir = ".";
        LaneGraphOptions lgo;
        std::vector<uint32_t> closeUps;
        for (int i = 3; i + 1 < argc; ++i) {
            if (std::strcmp(argv[i], "--out") == 0) outDir = argv[i + 1];
            if (std::strcmp(argv[i], "--junction") == 0) closeUps.push_back(static_cast<uint32_t>(std::atoi(argv[i + 1])));
            if (std::strcmp(argv[i], "--vehicle") == 0) {
                const std::string spec = argv[i + 1];
                const std::size_t a = spec.find(':'), b = a == std::string::npos ? a : spec.find(':', a + 1);
                if (b == std::string::npos) { std::fprintf(stderr, "lanes: --vehicle wants tag:radius:width[:length], not %s\n", spec.c_str()); return 2; }
                const std::size_t c = spec.find(':', b + 1);   // the body's length, when given (#284 item 9)
                VehicleSize v;
                v.tag = spec.substr(0, a);
                v.turningRadius = std::atof(spec.substr(a + 1, b - a - 1).c_str());
                v.width = std::atof(spec.substr(b + 1, c == std::string::npos ? std::string::npos : c - b - 1).c_str());
                if (c != std::string::npos) v.length = std::atof(spec.substr(c + 1).c_str());
                lgo.vehicles.push_back(v);
            }
        }
        // The tool's test vehicles: a car (about a 5 m turning radius, 1.9 m wide) and a long vehicle (a bus or a
        // lorry: about 10 m, 2.6 m). Names and sizes for looking at the lab's roads; the driving model's own vehicles
        // are its data, passed in by whoever builds the graph for it.
        if (lgo.vehicles.empty()) {
            VehicleSize car, longVehicle;
            car.tag = "car";
            car.turningRadius = 5.0;
            car.width = 1.9;
            longVehicle.tag = "long_vehicle";
            longVehicle.turningRadius = 10.0;
            longVehicle.width = 2.6;
            lgo.vehicles = {car, longVehicle};
        }
        if (!makeOutDir(outDir)) return 1;
        const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        std::unique_ptr<Result> rp;
        try { rp = build(RoadLabGraph::load(argv[2])); } catch (const std::exception& e) { std::fprintf(stderr, "lanes: %s\n", e.what()); return 1; }
        const double buildMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        // THE ROAD MODEL of this build, its lane level with it (ADR-0429), and what building the lane level found
        LaneGraphBuild lg;
        const RoadModel model = assembleRoadModel(*rp, lgo, &lg);
        if (!lg.error.empty()) { std::fprintf(stderr, "lanes: %s\n", lg.error.c_str()); return 1; }
        const LaneGraph& G = model.laneLevel;
        uint32_t lanes = 0;
        for (const Path& p : G.paths) lanes += p.kind == PathKind::Lane ? 1 : 0;
        uint32_t full = 0, partly = 0, refused = 0, noTurns = 0, fitsNone = 0, noPath = 0;
        for (const JunctionNote& n : lg.junctionNotes) {
            if (n.turnsStated == 0) ++noTurns;
            else if (n.turnsRefused == 0) ++full;
            else if (n.turnsRefused < n.turnsStated) ++partly;
            else ++refused;
        }
        for (const Refusal& ref : lg.refusals) ++(ref.why == RefusalWhy::NoLanePath ? noPath : fitsNone);
        std::printf("%s: %u lane paths, %zu connectors (%u straight on, %u near turns, %u far turns) of %zu stated turns, %u fitting no "
                    "vehicle, %u with no lane path to take them; %zu sections, %zu junctions (%u with every stated turn built, %u some, "
                    "%u none, %u with no turns), %zu problems; largest part a car can get round: %.1f%% of the lane paths\n",
                    rp->graph.name.c_str(), lanes, G.paths.size() - lanes, lg.connectorsByTurn[static_cast<int>(Turn::Straight)],
                    lg.connectorsByTurn[static_cast<int>(Turn::NearTurn)], lg.connectorsByTurn[static_cast<int>(Turn::FarTurn)],
                    model.turns.size(), fitsNone, noPath, G.sections.size(), G.junctions.size(), full, partly, refused, noTurns,
                    lg.problems.size(), 100.0 * lg.reachableShare);
        {
            // THE ROAD MODEL'S OWN PROBLEMS (its one home: where the paint and the stated junctions disagree, the stated
            // turns it could not settle -- an arriving lane that takes no turn, an arm with no heading), in model_problems.txt
            std::ofstream f(outDir + "/model_problems.txt");
            for (const std::string& p : model.problems) f << p << "\n";
            uint32_t inJunctions = 0, other = 0;
            for (const LaneGap& gp : lg.laneGaps) ++(gp.junction != kNoIndex ? inJunctions : other);
            std::printf("  the road model's own problems: %zu (model_problems.txt); lane stretches with no lane path: %u inside a "
                        "junction, %u for another reason (each in the problems); %u lanes cars drive made into no lane path\n",
                        model.problems.size(), inJunctions, other, lg.drivenLanesLeftOut);
        }
        std::printf("  lane level %.0f ms (%.0f of it the connectors), the build before it %.0f ms", lg.milliseconds,
                    lg.connectMilliseconds, buildMs);
        for (const char* stage : {"lanes", "profiles", "footprints", "surfaces"})
            if (rp->timings.count(stage)) std::printf(", %s %.0f", stage, 1000 * rp->timings.at(stage));
        std::printf("\n  left out: %u connector lanes, %u track/footway lanes; %u lanes tidied; joined as stated: %u where a "
                    "street carries on, %u where a lane leaves its host, %u where one merges into it, %u of them with a gap\n",
                    lg.connectorsLeftOut, lg.otherLanesLeftOut, lg.tidiedLanes, lg.carryOnJoins, lg.divergeJoins, lg.mergeJoins, lg.joinsWithGap);
        {
            // THE CORNER FAULTS (ADR-0426): the stated turns whose drawn corner does not fit a class, by class; one line per
            // turn in corner_faults.txt (the work list for step 9)
            std::map<std::string, uint32_t> byTag;
            std::map<uint32_t, std::string> perTurn;
            for (const CornerFault& f : lg.cornerFaults) {
                ++byTag[f.tag.empty() ? std::string("every class") : f.tag];
                char b[96];
                std::snprintf(b, sizeof b, " (%.1f, %.1f)", f.at.x, f.at.y);
                perTurn[f.turn] += " [" + (f.tag.empty() ? std::string("every class") : f.tag) + ": " + f.why + b + "]";
            }
            std::printf("  corner faults: %zu over %zu stated turns;", lg.cornerFaults.size(), perTurn.size());
            for (const std::pair<const std::string, uint32_t>& t : byTag) std::printf(" %s %u;", t.first.c_str(), t.second);
            std::printf("\n");
            std::ofstream f(outDir + "/corner_faults.txt");
            for (const std::pair<const uint32_t, std::string>& t : perTurn) {
                const ModelTurn& mt = model.turns[t.first];
                f << model.junctions[mt.junction.index].id << " arm " << mt.fromArm << " -> " << mt.toArm << " " << turnKey(mt.turn) << " "
                  << model.lanes[mt.from.index].id << " -> " << model.lanes[mt.to.index].id << ":" << t.second << "\n";
            }
        }
        for (std::size_t i = 0; i < G.vehicleTagNames.size() && i < lg.closedTo.size(); ++i)
            std::printf("  %u connectors closed to %s (too tight for it, or it would leave the paved road)\n", lg.closedTo[i],
                        G.vehicleTagNames[i].c_str());
        for (const std::pair<std::string, std::vector<uint32_t>>& o : lg.outside)
            std::printf("  outside the largest part: %zu lanes %s\n", o.second.size(), o.first.c_str());
        // EACH VEHICLE TAG'S REACH (ADR-0426): the share of the lane paths in the largest part a vehicle of the tag can get
        // round and back, the paths it may use only (its tag, no bend tighter than it turns), with the lane changes the lines
        // allow and with next paths alone (what a car can do while the driving changes no lanes); and the freeway's lane
        // paths in that part (next paths alone)
        for (const VehicleSize& v : lgo.vehicles) {
            const NameIndex bit = LaneGraph::nameIndex(G.vehicleTagNames, v.tag);
            double share[2] = {0, 0};
            uint32_t freeway = 0, freewayAll = 0;
            for (int changes = 1; changes >= 0; --changes) {
                ReachRules rr;
                rr.laneChanges = changes != 0;
                rr.open.assign(G.paths.size(), 0);
                for (uint32_t p = 0; p < G.paths.size(); ++p)
                    rr.open[p] = bit < kMaxTagNames && (G.paths[p].vehicleTags & (TagMask(1) << bit)) && G.paths[p].tightestRadius >= v.turningRadius;
                const ReachableParts parts = reachableParts(G, rr);
                uint32_t in = 0;
                freeway = freewayAll = 0;
                for (uint32_t p = 0; p < G.paths.size(); ++p) {
                    if (G.paths[p].kind != PathKind::Lane) continue;
                    const bool inLargest = !parts.sizes.empty() && parts.partOf[p] == parts.largest && rr.open[p];
                    in += inLargest ? 1 : 0;
                    const bool fw = G.roadClassNames[G.sections[G.paths[p].section.index].roadClass] == "freeway";
                    freewayAll += fw ? 1 : 0;
                    freeway += fw && inLargest ? 1 : 0;
                }
                share[changes] = lanes ? 100.0 * in / lanes : 0.0;
            }
            std::printf("  %s: %.1f%% of the lane paths in the largest part it can get round with lane changes, %.1f%% without; freeway "
                        "lane paths in it without: %u of %u\n", v.tag.c_str(), share[1], share[0], freeway, freewayAll);
        }
        {
            const CrossingReport& cr = lg.crossings;
            std::printf("  crossing points: %zu (%u crosses, %u joins, %u splits between two paths; %u with crosswalks); of "
                        "the path pairs, %u reach across two junctions and %u are a connector and a lane; the busiest "
                        "junction %u has %u; %u junctions with two or more connectors have none; %u far turns have a "
                        "place to wait inside; long bodies: %u records of their own, %u made longer; %.0f ms\n",
                        G.crossings.size(), cr.byMeeting[static_cast<int>(Meeting::Cross)],
                        cr.byMeeting[static_cast<int>(Meeting::Join)], cr.byMeeting[static_cast<int>(Meeting::Split)],
                        cr.withCrosswalks, cr.betweenJunctions, cr.withLanes, cr.busiestJunction, cr.busiestCount,
                        cr.junctionsWithout, cr.waitInsidePoints, cr.onlyForLength, cr.widenedForLength, cr.milliseconds);
        }
        {
            // the stated controls and signs, carried through (#240)
            const ControlPlanReport& cp = lg.controlPlan;
            std::string controls, kinds;
            for (const std::pair<std::string, uint32_t>& c : cp.byControl)
                controls += (controls.empty() ? "" : ", ") + std::to_string(c.second) + " " + c.first;
            for (const std::pair<std::string, uint32_t>& k : cp.regulations)
                kinds += (kinds.empty() ? "" : ", ") + std::to_string(k.second) + " " + k.first;
            std::printf("  controls: %u stated junctions; lane-level junctions: %s; at arriving lanes' ends: %s; posted limits on %u lanes "
                        "and %u connectors\n", cp.planned, controls.c_str(), kinds.empty() ? "nothing" : kinds.c_str(), cp.lanesPosted,
                        cp.connectorsPosted);
        }
        {
            // THE BENDS BY ROAD CLASS (#281): each path's tightest measured bend (its bend profile's tightest piece), lanes
            // by their section's class and connectors by the class of the lane they lead into, so a glance says whether
            // the numbers look like the roads (a freeway ring in the hundreds of metres, a street corner a few metres).
            struct Group { std::vector<double> radii; uint32_t bent = 0; double least = kNoBend; uint32_t leastPath = 0; double leastAt = 0; };
            std::map<std::pair<int, std::string>, Group> groups;   // (lanes 0 / connectors 1, class) -> its paths
            for (uint32_t p = 0; p < G.paths.size(); ++p) {
                const Path& path = G.paths[p];
                uint32_t lane = p;
                if (path.kind == PathKind::Connector && path.next.count > 0) lane = G.nextPaths[path.next.first].index;
                const SectionId sec = G.paths[lane].section;
                const std::string cls = sec.valid() && G.sections[sec.index].roadClass < G.roadClassNames.size()
                                            ? G.roadClassNames[G.sections[sec.index].roadClass] : std::string("?");
                Group& gr = groups[{path.kind == PathKind::Lane ? 0 : 1, cls}];
                const BendSpan t = G.tightestBend(PathId(p));
                gr.radii.push_back(t.radius);
                if (std::isfinite(t.radius)) ++gr.bent;
                if (t.radius < gr.least) { gr.least = t.radius; gr.leastPath = p; gr.leastAt = t.from; }
            }
            auto radius = [](double r) {
                if (!std::isfinite(r)) return std::string("no bend");
                char b[32];
                std::snprintf(b, sizeof b, "%.1f m", r);
                return std::string(b);
            };
            std::printf("  bends by road class (each path's tightest measured bend; kinks: %u in lanes, the worst %.1f deg; %u in "
                        "connectors, the worst %.1f deg; %u where paths join, the worst %.1f deg; bend profiles %zu pieces, "
                        "%zu bytes):\n", lg.laneKinks, lg.worstLaneKink, lg.connectorKinks, lg.worstConnectorKink,
                        lg.joinKinks, lg.worstJoinKink, G.bendPieces.size(), lg.bendPieceBytes);
            std::printf("    %-28s %6s %6s %10s %12s %10s  %s\n", "paths", "count", "bent", "median", "lowest 10%", "minimum", "where the minimum is");
            for (std::pair<const std::pair<int, std::string>, Group>& kv : groups) {
                std::vector<double>& rs = kv.second.radii;
                std::sort(rs.begin(), rs.end());
                const double median = rs[rs.size() / 2], tenth = rs[rs.size() / 10];
                const std::string who = (kv.first.first == 0 ? "lanes, " : "connectors into ") + kv.first.second;
                char where[96] = "-";
                if (std::isfinite(kv.second.least))
                    std::snprintf(where, sizeof where, "%s %u from %.1f m along",
                                  G.paths[kv.second.leastPath].kind == PathKind::Lane ? "lane" : "connector",
                                  kv.second.leastPath, kv.second.leastAt);
                std::printf("    %-28s %6zu %6u %10s %12s %10s  %s\n", who.c_str(), rs.size(), kv.second.bent, radius(median).c_str(),
                            radius(tenth).c_str(), radius(kv.second.least).c_str(), where);
            }
        }
        const std::string dumpPath = outDir + "/lane_graph.txt", problemsPath = outDir + "/problems.txt", svgPath = outDir + "/lane_graph.svg";
        const std::string reachPath = outDir + "/reach.txt";
        { std::ofstream f(dumpPath); f << dumpText(G); if (!f) { std::fprintf(stderr, "lanes: cannot write %s\n", dumpPath.c_str()); return 1; } }
        { std::ofstream f(problemsPath); for (const std::string& p : lg.problems) f << p << "\n"; }
        {
            std::ofstream f(reachPath);
            f << "largest part a car can get round: " << lg.reach.sizes[lg.reach.largest] << " paths (" << 100.0 * lg.reachableShare
              << "% of the lane paths)\n";
            for (const std::pair<std::string, std::vector<uint32_t>>& o : lg.outside) {
                f << o.second.size() << " lanes " << o.first << ":";
                for (uint32_t p : o.second) f << " " << p;
                f << "\n";
            }
        }
        for (std::size_t i = 0; i < lg.problems.size() && i < 10; ++i) std::printf("  problem: %s\n", lg.problems[i].c_str());
        LaneGraphSvgMarks marks;
        for (const Refusal& r : lg.refusals) marks.refusedTurns.push_back(r.at);
        if (!writeLaneGraphSvg(G, svgPath, marks)) { std::fprintf(stderr, "lanes: cannot write %s\n", svgPath.c_str()); return 1; }
        std::printf("wrote %s, %s, %s, %s\n", dumpPath.c_str(), problemsPath.c_str(), reachPath.c_str(), svgPath.c_str());
        for (uint32_t j : closeUps) {
            if (j >= G.junctions.size()) { std::fprintf(stderr, "lanes: no junction %u (%zu junctions)\n", j, G.junctions.size()); return 2; }
            const Junction& jn = G.junctions[j];
            // the window: the junction's outline and its connectors, and a margin round them
            constexpr double kCloseUpMargin = 8.0;
            std::vector<Vec2> pts(G.planPoints.begin() + jn.box.first, G.planPoints.begin() + jn.box.end());
            for (uint32_t k = jn.connectors.first; k < jn.connectors.end(); ++k) {
                const Range& c = G.paths[G.junctionConnectors[k].index].curve;
                pts.insert(pts.end(), G.points.begin() + c.first, G.points.begin() + c.end());
            }
            LaneGraphSvgMarks close = marks;
            close.cropped = true;
            bounds(pts, close.cropLo, close.cropHi);
            close.cropLo = close.cropLo - Vec2(kCloseUpMargin, kCloseUpMargin);
            close.cropHi = close.cropHi + Vec2(kCloseUpMargin, kCloseUpMargin);
            const std::string base = outDir + "/junction_" + std::to_string(j);
            if (!writeLaneGraphSvg(G, base + ".svg", close)) { std::fprintf(stderr, "lanes: cannot write %s.svg\n", base.c_str()); return 1; }
            // the junction itself (its control, the signs its arriving lanes face), the planned junctions it came from (what
            // the road graph's control plan says and why), then its connectors, then every crossing point on one of them
            std::ofstream f(base + ".txt");
            {
                // the dump's lines for this junction: its header and the indented lines under it
                std::istringstream all(dumpText(G));
                const std::string head = "junction " + std::to_string(j) + ":";
                bool in = false;
                for (std::string line; std::getline(all, line);) {
                    if (line.compare(0, head.size(), head) == 0) in = true;
                    else if (in && line.compare(0, 2, "  ") != 0) break;
                    if (in) f << line << "\n";
                }
            }
            if (jn.modelJunction < rp->graph.junctions.size()) {
                // the stated junction it is (the road model's junction of the same index: the road file's list)
                const ModelJunction& p = rp->graph.junctions[jn.modelJunction];
                f << "stated as " << p.id << " at (" << p.at.x << ", " << p.at.y << "): " << p.control << ", \"" << p.why << "\"\n";
                for (const ModelArm& a : p.arms) {
                    f << "  arm: road " << a.road << ", its spine running " << (a.spineIn ? "in" : "out") << " at " << a.station << " m";
                    for (const std::string& r : a.regulations) f << ", " << r;
                    f << "\n";
                }
            }
            std::vector<uint32_t> records;
            for (uint32_t k = jn.connectors.first; k < jn.connectors.end(); ++k) {
                const PathId c = G.junctionConnectors[k];
                f << engine::dumpPath(G, c);
                for (uint32_t m = G.paths[c.index].crossings.first; m < G.paths[c.index].crossings.end(); ++m)
                    records.push_back(G.pathCrossings[m].index);
            }
            std::sort(records.begin(), records.end());
            records.erase(std::unique(records.begin(), records.end()), records.end());
            for (uint32_t c : records) f << dumpCrossing(G, CrossingId(c));
            std::printf("wrote %s.svg, %s.txt (%u connectors, %zu crossing points)\n", base.c_str(), base.c_str(),
                        jn.connectors.count, records.size());
        }
        return 0;
    }
    if (argc < 3 || (std::strcmp(argv[1], "build") != 0 && std::strcmp(argv[1], "blocks") != 0)) {
        std::fprintf(stderr, "usage: lanes_tool build <graph.json> [--out <dir>]\n"
                             "       lanes_tool lanegraph <graph.json> [--out <dir>] [--vehicle tag:radius:width[:length] ...] [--junction J ...]   (the lane graph: dump, problems, reach, plan SVG; junction close-ups)\n"
                             "       lanes_tool design <graph.json> <out.json> [--plan-crossings]   (the graph with every road's planned profile; built again, taken as given)\n"
                             "       lanes_tool landings <graph.json> ...   (each ramp end on a street: on the carriageway going its way? #282)\n"
                             "       lanes_tool layout <graph.json> [--write] [--fresh] [--names]   (the road file's junctions and streets with ids; --write puts them in the file, ADR-0417; --names their names, ADR-0420)\n"
                             "       lanes_tool model <graph.json> [--out <dir>]   (the road model: model.txt, report.txt, model.svg)\n"
                             "       lanes_tool regions-lots <level.json> [--out <dir>]   (the level's lots grown in memory, dumped beside the stored ones)\n"
                             "                    [--ramp-width M] [--ramp-shoulder M]   ramp carriageway (default 4.5 m lane, 2.5 m shoulder)\n");
        return 2;
    }
    std::string out; for (int i = 3; i + 1 < argc; ++i) if (std::strcmp(argv[i], "--out") == 0) out = argv[i + 1];
    std::string debugLane; for (int i = 3; i + 1 < argc; ++i) if (std::strcmp(argv[i], "--debug-lane") == 0) debugLane = argv[i + 1];
    bool probe = false; double probeX = 0, probeY = 0, probeR = 30; for (int i = 3; i + 3 < argc; ++i) if (std::strcmp(argv[i], "--probe") == 0) { probe = true; probeX = std::atof(argv[i + 1]); probeY = std::atof(argv[i + 2]); probeR = std::atof(argv[i + 3]); }
    bool quick = false; for (int i = 3; i < argc; ++i) if (std::strcmp(argv[i], "--quick") == 0) quick = true;
    bool holes = false; for (int i = 3; i < argc; ++i) if (std::strcmp(argv[i], "--holes") == 0) holes = true;
    bool walls = false; for (int i = 3; i < argc; ++i) if (std::strcmp(argv[i], "--walls") == 0) walls = true;
    if (std::strcmp(argv[1], "blocks") == 0) {
        // blocks <graph.json> [--citysim level.json] [--out dir]: the engine's block/lot pass on the pavement's holes, audited and drawn
        using namespace engine::roads::lanes;
        std::string out = ".", cs; for (int i = 3; i + 1 < argc; ++i) { if (std::strcmp(argv[i], "--out") == 0) out = argv[i + 1]; if (std::strcmp(argv[i], "--citysim") == 0) cs = argv[i + 1]; }
        nlohmann::json citysim = nlohmann::json::object();
        if (!cs.empty()) { std::ifstream f(cs); nlohmann::json lvl; f >> lvl; citysim = lvl.value("citysim", nlohmann::json::object()); }
        std::unique_ptr<Result> r = build(RoadLabGraph::load(argv[2]));
        {   // where are the holes? (debug for the scene-block path)
            auto census = [](const PolySet& s, const char* name) { size_t holes = 0, big = 0; for (const Polygon2& p : s) for (const Ring& h : p.holes) { ++holes; if (std::fabs(signedArea(h)) >= 2000.0) ++big; } std::printf("  %s: %zu polygons, %zu holes (%zu >= 2000 m2)\n", name, s.size(), holes, big); };
            census(r->pavement.surface, "surface"); census(r->pavement.shoulder, "shoulder"); census(unionSets(r->pavement.surface, r->pavement.shoulder), "union");
        }
        BlockAudit a = auditBlocks(*r, citysim);
        std::printf("scene-hole blocks: ");
        std::printf("%s", summary(a).c_str());
        if (!makeOutDir(out)) return 1;
        const std::string svg = out + "/blocks.svg"; writeBlocksSvg(*r, a, svg); std::printf("wrote %s\n", svg.c_str());
        {   // --at x y r: what the lot pass put near a point — lots, building part meshes, LOD1 boxes (set LANELAB_AUDIT_PARTS=1 to grow meshes)
            bool at = false; double ax = 0, ay = 0, ar = 30; for (int i = 3; i + 3 < argc; ++i) if (std::strcmp(argv[i], "--at") == 0) { at = true; ax = std::atof(argv[i + 1]); ay = std::atof(argv[i + 2]); ar = std::atof(argv[i + 3]); }
            if (at) {
                std::printf("--at (%.0f, %.0f) r %.0f\n", ax, ay, ar);
                for (const engine::LotBuilding& lb : a.grown.lots) {
                    if (std::hypot(lb.site.x - ax, lb.site.y - ay) > ar) continue; double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300; for (const engine::Vec2& q : lb.plan) { x0 = std::min(x0, q.x); x1 = std::max(x1, q.x); y0 = std::min(y0, q.y); y1 = std::max(y1, q.y); }
                    std::printf("  lot %-6s site (%.0f, %.0f) plan [%.0f %.0f]-[%.0f %.0f] %.1fx%.1f groundY %.2f\n", lb.type.c_str(), lb.site.x, lb.site.y, x0, y0, x1, y1, lb.width, lb.depth, lb.groundY);
                }
                auto meshes = [&](const std::vector<engine::RenderMesh>& ms, const char* what) {
                    for (size_t i = 0; i < ms.size(); ++i) {
                        const engine::RenderMesh& m = ms[i]; if (m.vertices.empty()) continue; double x0 = 1e300, y0 = 1e300, z0 = 1e300, x1 = -1e300, y1 = -1e300, z1 = -1e300;
                        for (const auto& v : m.vertices) { x0 = std::min(x0, static_cast<double>(v.position.x)); x1 = std::max(x1, static_cast<double>(v.position.x)); y0 = std::min(y0, static_cast<double>(v.position.y)); y1 = std::max(y1, static_cast<double>(v.position.y)); z0 = std::min(z0, static_cast<double>(v.position.z)); z1 = std::max(z1, static_cast<double>(v.position.z)); }
                        const double cx = 0.5 * (x0 + x1), cz = 0.5 * (z0 + z1); if (std::hypot(cx - ax, cz - ay) > ar + 0.5 * std::hypot(x1 - x0, z1 - z0)) continue;
                        std::printf("  %s #%zu: xz [%.0f %.0f]-[%.0f %.0f] (%.1fx%.1f) y %.2f..%.2f, %zu verts\n", what, i, x0, z0, x1, z1, x1 - x0, z1 - z0, y0, y1, m.vertices.size());
                    }
                };
                meshes(a.grown.parts, "part"); meshes(a.grown.flatParts, "lod1");
            }
        }
        return (a.broken || !a.padConflicts.empty()) ? 2 : 0;
    }
    std::unique_ptr<Result> rp;
    try { rp = build(RoadLabGraph::load(argv[2])); } catch (const std::exception& e) { std::fprintf(stderr, "lanes: %s\n", e.what()); return 1; }
    Result& r = *rp;
    std::printf("%s", summary(r).c_str());
    bool ok = true;
    if (!quick) for (const Check& c : invariants(r)) { std::printf("%s  %s: %s\n", c.ok ? "PASS" : "FAIL", c.name.c_str(), c.detail.c_str()); ok = ok && c.ok; }
    if (walls) {   // parapet runs whose END CAP stands on drivable pavement: a wall a car runs into
        using namespace engine::roads::lanes;
        ParapetCensus census;
        const std::vector<ParapetRun> runs = parapetRuns(r, &census);
        struct Bad { engine::Vec2 at; double len; std::string lane; };
        std::vector<Bad> bad; double total = 0;
        for (const ParapetRun& run : runs) {
            double len = 0; for (size_t i = 0; i + 1 < run.pts.size(); ++i) len += distance(run.pts[i], run.pts[i + 1]);
            total += len;
            const std::string lane = run.lane >= 0 ? r.lanes.lanes[static_cast<size_t>(run.lane)].id : std::string("?");
            if (run.capOnPavement[0]) bad.push_back({run.pts.front(), len, lane});
            if (run.capOnPavement[1]) bad.push_back({run.pts.back(), len, lane});
        }
        // Which side of a boundary edge does the wall go on? sweepWall offsets along (d.y, -d.x). Sample each
        // run's midpoints and ask what is paved at the wall's own position: a wall standing on a travel lane is
        // in the road, one standing off the lanes is on the shoulder where it belongs.
        const std::vector<Box2> laneBox = laneBoxes(r.pavement.footprints); const LaneGrid laneGrid(laneBox, r.pavement.footprints);
        auto pavedAt = [&](const engine::Vec2& q, double z) {
            for (int ojI : laneGrid.at(q)) {
                const size_t oj = static_cast<size_t>(ojI); const Box2& bb = laneBox[oj];
                if (q.x < bb.minX || q.x > bb.maxX || q.y < bb.minY || q.y > bb.maxY || !contains(r.pavement.footprints[oj], q)) continue;
                if (std::fabs(r.heights->deck(ojI, q) - z) < 1.0) return true;
            }
            return false;
        };
        // What does the wall stand on, and what lies between it and the lane it protects? A barrier belongs at the
        // outer edge of its carriageway's shoulder; one standing on bare ground with a gap of nothing behind it
        // belongs to no carriageway at all.
        int onLane = 0, onShoulder = 0, onMedian = 0, onNothing = 0, onDeckEdge = 0; std::vector<engine::Vec2> nowhereAt;
        int gapPaved = 0, gapBare = 0;
        for (const ParapetRun& run : runs) {
            for (size_t i = 0; i + 1 < run.pts.size(); ++i) {
                engine::Vec2 d = run.pts[i + 1] - run.pts[i]; const double l = d.length(); if (l < 1e-6) continue; d = d / l;
                const engine::Vec2 nrm(d.y, -d.x), mid = (run.pts[i] + run.pts[i + 1]) * 0.5;
                const engine::Vec2 wall = mid + nrm * ((run.offOuter[i] + run.offInner[i]) * 0.5);
                if (run.offOuter[i] < 0.2) ++onDeckEdge;                       // set inside the deck edge: a bridge parapet
                else if (pavedAt(wall, run.z[i])) ++onLane;
                else if (contains(r.pavement.shoulder, wall)) ++onShoulder;
                else if (contains(r.pavement.median, wall)) ++onMedian;
                else { ++onNothing; if (nowhereAt.size() < 6) nowhereAt.push_back(wall); }
                const engine::Vec2 gap = mid + nrm * (run.offInner[i] * 0.5);   // halfway between the deck edge and the wall
                if (pavedAt(gap, run.z[i]) || contains(r.pavement.shoulder, gap) || contains(r.pavement.median, gap)) ++gapPaved; else ++gapBare;
            }
        }
        // how much of each kind of edge carries a barrier, and what the barriers are, per road class
        std::printf("--walls: edge metres built / seen:");
        for (size_t k = 0; k < static_cast<size_t>(EdgeRole::Count_); ++k)
            std::printf(" %s %.0f/%.0f", edgeRoleName(static_cast<EdgeRole>(k)), census.built[k], census.metres[k]);
        std::printf("\n");
        std::map<std::string, std::array<double, 3>> byClass;   // class -> metres of none/wall/guardrail
        for (const ParapetRun& run : runs) {
            if (run.lane < 0 || r.lanes.lanes[static_cast<size_t>(run.lane)].parent < 0) continue;
            double len = 0; for (size_t i = 0; i + 1 < run.pts.size(); ++i) len += distance(run.pts[i], run.pts[i + 1]);
            const EdgeSpec& e = r.graph.edges[static_cast<size_t>(r.lanes.lanes[static_cast<size_t>(run.lane)].parent)];
            byClass[e.isRamp() ? std::string("ramp") : e.cls][static_cast<size_t>(run.kind)] += len;
        }
        for (const auto& [cls, m] : byClass) std::printf("         %-10s wall %.0f m, guardrail %.0f m\n", cls.c_str(), m[1], m[2]);
        std::printf("--walls: the wall stands on: deck edge %d, shoulder %d, median %d, LANE %d, BARE GROUND %d\n", onDeckEdge, onShoulder, onMedian, onLane, onNothing);
        std::printf("         between the deck edge and the wall: paved %d, bare %d\n", gapPaved, gapBare);
        for (const engine::Vec2& q : nowhereAt) std::printf("    on bare ground at (%7.1f, %7.1f)\n", q.x, q.y);
        if (probe && r.hasTerrain) {   // conformed ground standing OVER a deck near the probe: the green wedges
            const engine::Vec2 P(probeX, probeY); const HeightGrid& G = r.terrain;
            int over = 0; double worst = 0; engine::Vec2 worstAt; std::string worstLane;
            const int i0 = std::max(0, static_cast<int>((P.x - probeR - G.x0) / G.res)), i1 = std::min(G.nx - 1, static_cast<int>((P.x + probeR - G.x0) / G.res) + 1);
            const int j0 = std::max(0, static_cast<int>((P.y - probeR - G.y0) / G.res)), j1 = std::min(G.ny - 1, static_cast<int>((P.y + probeR - G.y0) / G.res) + 1);
            for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
                const engine::Vec2 q(G.x0 + i * G.res, G.y0 + j * G.res); if (distance(q, P) > probeR) continue;
                const double zg = G.at(i, j);
                for (size_t li = 0; li < r.lanes.lanes.size(); ++li) {
                    const Lane& l = r.lanes.lanes[li]; if (l.xy.size() < 2) continue;
                    if (project(l.xy, l.s, q).distance > l.w / 2 + 1.0) continue;
                    const double excess = zg - r.heights->deck(static_cast<int>(li), q);
                    if (excess > 0.01) { ++over; if (excess > worst) { worst = excess; worstAt = q; worstLane = l.id; } }
                }
            }
            std::printf("         ground over a deck near the probe: %d grid nodes, worst %.2f m at (%.1f, %.1f) over %s\n",
                        over, worst, worstAt.x, worstAt.y, worstLane.c_str());
        }
        {   // PAINT rails: lane paint is swept along l.left / l.right. A jump in those polylines draws a
            // strip straight across the carriageway, which reads as a pale slab lying on the road.
            int jumps = 0; double worst = 0; std::vector<engine::Vec2> jumpAt;
            for (const Lane& l : r.lanes.lanes) {
                for (const std::vector<engine::Vec2>* E : {&l.left, &l.right}) {
                    for (size_t i = 0; i + 1 < E->size(); ++i) {
                        const double d = distance((*E)[i], (*E)[i + 1]);
                        if (d < 5.0) continue;
                        ++jumps; if (d > worst) worst = d;
                        if (jumpAt.size() < 8) jumpAt.push_back((*E)[i]);
                    }
                }
            }
            std::printf("         paint rails: %d jumps over 5 m in lane edge polylines, worst %.1f m\n", jumps, worst);
            for (const engine::Vec2& q : jumpAt) std::printf("           jump at (%7.1f, %7.1f)\n", q.x, q.y);
        }
        if (probe) {   // the OTHER layers near the probe point: median slabs and shoulder slivers show as
            // green tops with concrete sides, and read as slabs lying across the road
            const engine::Vec2 P(probeX, probeY);
            auto listLayer = [&](const PolySet& ps, const char* what) {
                int shown = 0;
                for (const Polygon2& pg : ps) {
                    if (pg.outer.size() < 3) continue;
                    engine::Vec2 c; for (const engine::Vec2& q : pg.outer) c += q; c = c / static_cast<double>(pg.outer.size());
                    if (distance(c, P) > probeR || shown++ >= 10) continue;
                    double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
                    for (const engine::Vec2& q : pg.outer) { x0 = std::min(x0, q.x); x1 = std::max(x1, q.x); y0 = std::min(y0, q.y); y1 = std::max(y1, q.y); }
                    std::printf("           %-9s %7.0f m2 at (%7.1f,%7.1f)  %3zu pts  bbox %.0f x %.0f m\n",
                                what, std::fabs(signedArea(pg.outer)), c.x, c.y, pg.outer.size(), x1 - x0, y1 - y0);
                }
            };
            std::printf("         layers within %.0f m of (%.1f, %.1f):\n", probeR, probeX, probeY);
            listLayer(r.pavement.median, "median"); listLayer(r.pavement.shoulder, "shoulder");
        }
        if (probe) {   // every run near the probe point, with its geometry: what is actually standing there
            const engine::Vec2 P(probeX, probeY);
            std::printf("         runs within %.0f m of (%.1f, %.1f):\n", probeR, probeX, probeY);
            int shownRuns = 0;
            for (const ParapetRun& run : runs) {
                double dmin = 1e300; for (const engine::Vec2& q : run.pts) dmin = std::min(dmin, distance(q, P));
                if (dmin > probeR || shownRuns++ >= 14) continue;
                double len = 0; for (size_t i = 0; i + 1 < run.pts.size(); ++i) len += distance(run.pts[i], run.pts[i + 1]);
                const std::string lane = run.lane >= 0 ? r.lanes.lanes[static_cast<size_t>(run.lane)].id : std::string("?");
                // how far the run wanders sideways from its own lane: a run crossing the road shows up here
                double maxLat = 0;
                if (run.lane >= 0) { const Lane& l = r.lanes.lanes[static_cast<size_t>(run.lane)];
                    for (const engine::Vec2& q : run.pts) maxLat = std::max(maxLat, project(l.xy, l.s, q).distance); }
                std::printf("           %-14s %-9s h %.2f  %5.0f m  %3zu pts  (%.0f,%.0f)->(%.0f,%.0f)  max %.1f m off its lane\n",
                            lane.c_str(), run.kind == BarrierKind::Guardrail ? "guardrail" : "wall", run.height, len, run.pts.size(),
                            run.pts.front().x, run.pts.front().y, run.pts.back().x, run.pts.back().y, maxLat);
            }
        }
        std::sort(bad.begin(), bad.end(), [](const Bad& a, const Bad& b) { return a.len > b.len; });
        std::printf("--walls: %zu parapet runs, %.0f m total; %zu ends stand on same-level pavement\n", runs.size(), total, bad.size());
        {   // folds: a run whose direction reverses draws a bowtie across the road
            int folded = 0; double foldedM = 0; std::vector<engine::Vec2> foldAt;
            for (const ParapetRun& run : runs) {
                bool fold = false; double len = 0;
                for (size_t i = 0; i + 2 < run.pts.size(); ++i) {
                    const engine::Vec2 d0 = run.pts[i + 1] - run.pts[i], d1 = run.pts[i + 2] - run.pts[i + 1];
                    const double l0 = d0.length(), l1 = d1.length(); len += l0;
                    if (l0 < 1e-9 || l1 < 1e-9) continue;
                    if (dot(d0 / l0, d1 / l1) < -0.2) { if (!fold && foldAt.size() < 8) foldAt.push_back(run.pts[i + 1]); fold = true; }
                }
                if (fold) { ++folded; foldedM += len; }
            }
            std::printf("         %d runs fold back on themselves (%.0f m)\n", folded, foldedM);
            for (const engine::Vec2& q : foldAt) std::printf("           fold at (%7.1f, %7.1f)\n", q.x, q.y);
        }
        {   // what every freeway/ramp deck edge FACES, and how much of it carries a barrier today
            double allM = 0, allB = 0;
            for (size_t i = 0; i < static_cast<size_t>(EdgeRole::Count_); ++i) { allM += census.metres[i]; allB += census.built[i]; }
            std::printf("         edge roles (freeway + ramp only), metres of edge and how much is walled today:\n");
            for (size_t i = 0; i < static_cast<size_t>(EdgeRole::Count_); ++i) {
                const double m = census.metres[i], b = census.built[i];
                if (m < 0.5) continue;
                std::printf("           %-10s %8.0f m  walled %7.0f m (%3.0f %%)\n", edgeRoleName(static_cast<EdgeRole>(i)), m, b, m > 0 ? 100.0 * b / m : 0.0);
            }
            std::printf("           %-10s %8.0f m  walled %7.0f m (%3.0f %%)\n", "all", allM, allB, allM > 0 ? 100.0 * allB / allM : 0.0);
        }
        for (size_t i = 0; i < bad.size() && i < 20; ++i) std::printf("  cap at (%7.1f, %7.1f)  run %5.0f m  lane %s\n", bad[i].at.x, bad[i].at.y, bad[i].len, bad[i].lane.c_str());
    }
    if (holes) {   // every hole in the PAVED surface, by what it lies next to: a gore wedge inside the road corridor reads as a missing segment
        using namespace engine::roads::lanes;
        struct H { double area, opened; engine::Vec2 c; std::string near; double d; };
        std::vector<H> hs;
        for (const Polygon2& sp : r.pavement.surface) for (const Ring& h : sp.holes) {
            const double a = std::fabs(signedArea(h)); if (a < 1.0 || a >= 2000.0) continue;   // >= 2000 m2 is a city block
            engine::Vec2 c; for (const engine::Vec2& q : h) c += q; c = c / static_cast<double>(h.size());
            // what actually covers the ground here: the median layer carries the kerbed grass slabs (islands and gores)
            H e; e.area = a; e.opened = contains(r.pavement.median, c) ? 0.0 : 1.0; e.c = c; e.d = 1e300;
            for (const EdgeSpec& ed : r.graph.edges) { if (ed.xy.size() < 2) continue; const double d = project(ed.xy, ed.s, c).distance; if (d < e.d) { e.d = d; e.near = ed.id + " (" + ed.cls + ")"; } }
            hs.push_back(std::move(e));
        }
        std::sort(hs.begin(), hs.end(), [](const H& a, const H& b) { return a.area > b.area; });
        int kerbed = 0, bare = 0; double bareArea = 0;
        for (const H& e : hs) { if (e.opened > 1.0) { ++bare; bareArea += e.area; } else ++kerbed; }
        std::printf("--holes: %zu holes under 2000 m2 in the paved surface: %d covered by a kerbed slab, %d UNCOVERED (%.0f m2 of bare ground inside the pavement)\n", hs.size(), kerbed, bare, bareArea);
        int shown = 0;
        for (const H& e : hs) {
            if (shown++ >= 25) continue;
            std::printf("  %7.0f m2 at (%7.1f, %7.1f)  %5.1f m from %-22s %s\n", e.area, e.c.x, e.c.y, e.d, e.near.c_str(), e.opened > 1.0 ? "UNCOVERED" : "kerbed slab");
        }
    }
    if (probe) {   // heights around a point: every road's profile, every lane's deck height, and the deck triangles by owning road
        using namespace engine::roads::lanes;
        const engine::Vec2 P(probeX, probeY); std::printf("--probe (%.1f, %.1f) r %.0f\n", probeX, probeY, probeR);
        for (const EdgeSpec& e : r.graph.edges) {
            if (e.z.size() < 2) continue; const Projection pr = project(e.xy, e.s, P); if (pr.distance > probeR) continue;
            std::printf("  %-12s %-9s %.1f m off, station %.1f of %.1f, z at -15..+15:", e.id.c_str(), e.cls.c_str(), pr.distance, pr.station, e.s.back());
            for (double d = -15; d <= 15; d += 5) { const double st = pr.station + d; if (st < 0 || st > e.s.back()) { std::printf("   ---"); continue; } std::printf(" %6.2f", interp(e.s, e.z, st)); }
            std::printf("  (ends %.2f / %.2f)\n", e.z.front(), e.z.back());
        }
        for (size_t li = 0; li < r.lanes.lanes.size(); ++li) {
            const Lane& l = r.lanes.lanes[li]; if (l.isConnector()) continue; const Projection pr = project(l.xy, l.s, P); if (pr.distance > probeR) continue;
            std::printf("  lane %-10s %.1f m off: own %.2f deck %.2f at P; partners:", l.id.c_str(), pr.distance, r.heights->own(static_cast<int>(li), P), r.heights->deck(static_cast<int>(li), P));
            for (int q : r.pavement.partners[li]) std::printf(" %s", r.lanes.lanes[static_cast<size_t>(q)].id.c_str()); std::printf("\n");
        }
        std::map<std::string, std::array<double, 3>> byRoad;   // owner road -> min z, max z, count
        for (const Surface& s : r.pavement.decks) for (size_t ti = 0; ti < s.tris.size(); ++ti) {
            const auto& tr = s.tris[ti]; const engine::Vec2 c = (s.verts[static_cast<size_t>(tr[0])].xy + s.verts[static_cast<size_t>(tr[1])].xy + s.verts[static_cast<size_t>(tr[2])].xy) / 3.0;
            if (std::hypot(c.x - P.x, c.y - P.y) > probeR) continue; const int o = s.triOwner[ti]; const Lane& l = r.lanes.lanes[static_cast<size_t>(o)];
            const std::string road = l.parent >= 0 ? r.graph.edges[static_cast<size_t>(l.parent)].id : l.id; auto& a = byRoad[road];
            const double z = (s.verts[static_cast<size_t>(tr[0])].z + s.verts[static_cast<size_t>(tr[1])].z + s.verts[static_cast<size_t>(tr[2])].z) / 3.0;
            if (a[2] == 0) { a[0] = a[1] = z; } a[0] = std::min(a[0], z); a[1] = std::max(a[1], z); a[2] += 1;
        }
        for (const auto& kv : byRoad) std::printf("  deck triangles owned by %-10s %5.0f, z %.2f .. %.2f\n", kv.first.c_str(), kv.second[2], kv.second[0], kv.second[1]);
        for (const Pier& pr : r.piers) if (std::hypot(pr.xy.x - P.x, pr.xy.y - P.y) <= probeR) std::printf("  pier of %s at (%.1f, %.1f) z %.2f .. %.2f (%.1f m tall)\n", r.graph.edges[static_cast<size_t>(pr.edge)].id.c_str(), pr.xy.x, pr.xy.y, pr.z0, pr.z1, pr.z1 - pr.z0);
        if (r.hasTerrain && r.terrain.inside(P.x, P.y)) std::printf("  conformed ground at P: %.2f\n", r.terrain.sample(P.x, P.y));
        { std::vector<SurfaceStep> steps = surfaceSteps(r); std::sort(steps.begin(), steps.end(), [](const SurfaceStep& x, const SurfaceStep& y) { return std::fabs(x.dz) > std::fabs(y.dz); }); int n = 0;
          for (const SurfaceStep& st : steps) {
              if (std::hypot(st.at.x - P.x, st.at.y - P.y) > probeR) continue; if (n++ >= 40) continue;
              std::printf("  step %s %s %.2f m, %.1f m long at (%.1f, %.1f)", st.a.c_str(), st.b.empty() ? "over ground" : ("vs " + st.b).c_str(), st.dz, st.length, st.at.x, st.at.y);
              if (st.b.empty() && r.hasTerrain && r.terrain.inside(st.at.x, st.at.y)) {   // what conform did at the nearest grid node
                  const int i = static_cast<int>(std::lround((st.at.x - r.terrain.x0) / r.terrain.res)), j = static_cast<int>(std::lround((st.at.y - r.terrain.y0) / r.terrain.res));
                  if (i >= 0 && i < r.terrain.nx && j >= 0 && j < r.terrain.ny) { const size_t nn = static_cast<size_t>(j) * r.terrain.nx + i;
                      std::printf("  [ground %.2f; nearest node owner %s at %.1f m, node z %.2f]", r.terrain.sample(st.at.x, st.at.y), nn < r.conform.nodeOwner.size() ? r.conform.nodeOwner[nn].c_str() : "?", nn < r.conform.nodeDist.size() ? r.conform.nodeDist[nn] : -1.0, r.terrain.at(i, j)); }
              }
              std::printf("\n");
          }
          std::printf("  %d step edges within %.0f m\n", n, probeR); }
        for (const auto& kv : r.pavement.pairs) {
            const Lane& a = r.lanes.lanes[static_cast<size_t>(kv.first.first)]; const Lane& b = r.lanes.lanes[static_cast<size_t>(kv.first.second)];
            if (kv.second.separatedArea < 1 || std::hypot(kv.second.minAt.x - P.x, kv.second.minAt.y - P.y) > probeR) continue;
            std::printf("  separated pair %s|%s: same-level %.0f m2, separated %.0f m2, dz max %.2f, min gap %.2f at (%.0f, %.0f)\n", a.id.c_str(), b.id.c_str(), kv.second.sameLevelArea, kv.second.separatedArea, kv.second.dz, kv.second.minDz, kv.second.minAt.x, kv.second.minAt.y);
        }
    }
    if (!out.empty() && !quick) {
        if (!makeOutDir(out)) return 1;
        std::string base = out + "/" + r.graph.name; std::string err;
        std::vector<NamedMesh> meshes = buildMeshes(r); size_t tris = 0; for (const NamedMesh& m : meshes) tris += m.mesh.indices.size() / 3;
    if (!debugLane.empty()) {   // the lane's footprint (raw rings), its spine and every deck triangle within its bbox, by owner
        using namespace engine::roads::lanes;
        const int li = r.lanes.find(debugLane);
        if (li < 0) std::printf("--debug-lane: no lane %s\n", debugLane.c_str());
        else {
            const Lane& l = r.lanes.lanes[static_cast<size_t>(li)]; const PolySet& fp = r.pavement.footprints[static_cast<size_t>(li)];
            Box2 b = bounds(fp[0]); for (const Polygon2& pg : fp) { Box2 c = bounds(pg); b.minX = std::min(b.minX, c.minX); b.minY = std::min(b.minY, c.minY); b.maxX = std::max(b.maxX, c.maxX); b.maxY = std::max(b.maxY, c.maxY); }
            const double m = 12; std::ofstream f(out + "/lane_" + debugLane + ".svg");
            f << "<svg xmlns='http://www.w3.org/2000/svg' viewBox='" << b.minX - m << " " << -(b.maxY + m) << " " << (b.maxX - b.minX) + 2 * m << " " << (b.maxY - b.minY) + 2 * m << "'>\n<g transform='scale(1,-1)'>\n";
            f << "<rect x='" << b.minX - m << "' y='" << b.minY - m << "' width='" << (b.maxX - b.minX) + 2 * m << "' height='" << (b.maxY - b.minY) + 2 * m << "' fill='#f4f4ee'/>\n";
            int inBox = 0, owned = 0;
            for (const Surface& s : r.pavement.decks) for (size_t ti = 0; ti < s.tris.size(); ++ti) {
                const auto& tr = s.tris[ti]; const engine::Vec2& A = s.verts[static_cast<size_t>(tr[0])].xy; const engine::Vec2& B = s.verts[static_cast<size_t>(tr[1])].xy; const engine::Vec2& C = s.verts[static_cast<size_t>(tr[2])].xy;
                const engine::Vec2 c = (A + B + C) / 3.0; if (c.x < b.minX - m || c.x > b.maxX + m || c.y < b.minY - m || c.y > b.maxY + m) continue;
                ++inBox; const bool mine = s.triOwner[ti] == li; owned += mine;
                f << "<polygon points='" << A.x << "," << A.y << " " << B.x << "," << B.y << " " << C.x << "," << C.y << "' fill='" << (mine ? "#f2a33c" : "#c9c9c9") << "' fill-opacity='0.8' stroke='#666' stroke-width='0.05'/>\n";
            }
            for (const Polygon2& pg : fp) { f << "<polygon fill='none' stroke='#c0392b' stroke-width='0.3' points='"; for (const engine::Vec2& q : pg.outer) f << q.x << "," << q.y << " "; f << "'/>\n"; for (const Ring& h : pg.holes) { f << "<polygon fill='none' stroke='#8e44ad' stroke-width='0.3' points='"; for (const engine::Vec2& q : h) f << q.x << "," << q.y << " "; f << "'/>\n"; } }
            f << "<polyline fill='none' stroke='#1f4e9c' stroke-width='0.25' stroke-dasharray='1 0.6' points='"; for (const engine::Vec2& q : l.xy) f << q.x << "," << q.y << " "; f << "'/>\n</g>\n</svg>\n";
            std::printf("--debug-lane %s: %d deck triangles in its bbox, %d owned by it; footprint polygons %zu, ring vertices %zu; wrote %s\n", debugLane.c_str(), inBox, owned, fp.size(), fp.empty() ? size_t(0) : fp[0].outer.size(), (out + "/lane_" + debugLane + ".svg").c_str());
        }
    }
        bool wrote = true;
        if (!writeGlb(meshes, base + ".glb", &err)) { std::fprintf(stderr, "lanes: %s\n", err.c_str()); wrote = false; }
        if (!writePlanSvg(r, base + ".svg")) { std::fprintf(stderr, "lanes: cannot write %s.svg\n", base.c_str()); wrote = false; }
        if (!writeStatsJson(r, out + "/stats.json")) { std::fprintf(stderr, "lanes: cannot write %s/stats.json\n", out.c_str()); wrote = false; }
        if (!wrote) return 1;
        std::printf("wrote %s.glb (%zu meshes, %zu triangles), %s.svg, %s/stats.json\n", base.c_str(), meshes.size(), tris, base.c_str(), out.c_str());
    }
    return ok ? 0 : 1;
}
