// Junction-geometry census — the PLANNING-phase view of the acute-angle problem.
//
// The curb probe measures what the mesher had to draw. This one measures what
// the planner handed it: how tight the junctions are, whether there is room to
// realign them, and how sharp the bends are between them. Device reasoning:
// "the sight lines are important and also taking sharp turns around acute
// angles means your car will tip over. So it's a no-go from a design point of
// view" — so the gate belongs on the graph, not on the corner geometry.
//
// Reports, for a shipped level:
//   * junctions by degree, and the TIGHTEST-GAP distribution
//   * how many junctions violate a range of candidate thresholds (sizing the work)
//   * REALIGN FEASIBILITY: how much straight run each tight arm has to bend in
//   * BENDS: minimum curve radius along each chain, and what that means at speed
//   * a dry run of the resolution ladder (realign / roundabout / stagger)
//
// Usage:
//     ./build/junction_angle_probe [level.json] [--target 60] [--csv out.csv]
#include "../../src/engine/procgen/city/road_net.h"
#include "../../src/engine/procgen/city/road_network.h"
#include "../../src/engine/procgen/city/polygon.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using namespace engine;
using json = nlohmann::json;

static const double PI_ = 3.14159265358979323846;
static double deg(double r) { return r * 180.0 / PI_; }

namespace {

struct Junction {
    int    node = -1;
    Vec2   pos;
    int    degree = 0;
    double tightest = 360.0;   // smallest gap between adjacent arms (deg)
    // The two arms forming that gap, and how much straight run each has before
    // the next real junction — the room a realignment has to work in.
    double roomA = 0, roomB = 0;
    double armLenA = 0, armLenB = 0;
};

// Neighbours of every node. PARALLEL edges (two edges joining the same pair)
// would otherwise read as extra arms and inflate the degree — so count each
// neighbour once, and report how many duplicates were seen.
std::vector<std::vector<int>> adjacency(const RoadGraph& g, int* dupesOut) {
    std::vector<std::vector<int>> adj(g.nodes.size());
    for (const RoadEdge& e : g.edges) {
        if (e.a == e.b) continue;
        adj[e.a].push_back(e.b);
        adj[e.b].push_back(e.a);
    }
    int dupes = 0;
    for (auto& v : adj) {
        std::sort(v.begin(), v.end());
        const std::size_t before = v.size();
        v.erase(std::unique(v.begin(), v.end()), v.end());
        dupes += static_cast<int>(before - v.size());
    }
    if (dupesOut) *dupesOut = dupes / 2;
    return adj;
}

// Walk from `v` toward `n` while the nodes are plain through-nodes, summing
// length — the distance to the next REAL junction (or dead end). This is the
// budget a bend has: you cannot realign an approach that is 12 m long.
double runToNextJunction(const RoadGraph& g, const std::vector<std::vector<int>>& adj,
                         int v, int n, double* firstLeg) {
    double run = (g.nodes[n].pos - g.nodes[v].pos).length();
    if (firstLeg) *firstLeg = run;
    int prev = v, cur = n;
    for (int hop = 0; hop < 4096; ++hop) {
        if (adj[cur].size() != 2) break;                 // a junction or a dead end
        const int nxt = adj[cur][0] == prev ? adj[cur][1] : adj[cur][0];
        run += (g.nodes[nxt].pos - g.nodes[cur].pos).length();
        prev = cur;
        cur = nxt;
    }
    return run;
}

// Circumradius of three points — +inf for a straight run. The tightest of these
// along a chain is what a car actually feels.
double circumradius(const Vec2& a, const Vec2& b, const Vec2& c) {
    const double ab = (b - a).length(), bc = (c - b).length(), ca = (a - c).length();
    const double area2 = std::fabs(cross(b - a, c - a));
    if (area2 < 1e-9) return 1e30;
    return (ab * bc * ca) / (2.0 * area2);
}

}  // namespace

int main(int argc, char** argv) {
    std::string levelPath = "assets/levels/metro_v2_test.json";
    std::string csvPath;
    double target = 60.0;
    bool control = false;   // measure the planner's own graph, not the sampled one
    double minArm = -1.0;   // >=0 overrides the recipe's min_arm_angle_deg (0 = pass off)
    double runIn = -1.0;    // >=0 overrides realign_run_in
    double dissolve = -1.0; // >=0 overrides dissolve_acute_deg (the CULL threshold)
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--target" && i + 1 < argc) target = std::atof(argv[++i]);
        else if (a == "--csv" && i + 1 < argc) csvPath = argv[++i];
        else if (a == "--control") control = true;
        else if (a == "--minarm" && i + 1 < argc) minArm = std::atof(argv[++i]);
        else if (a == "--runin" && i + 1 < argc) runIn = std::atof(argv[++i]);
        else if (a == "--dissolve" && i + 1 < argc) dissolve = std::atof(argv[++i]);
        else if (a.size() && a[0] != '-') levelPath = a;
    }

    std::ifstream f(levelPath);
    if (!f.is_open()) { std::fprintf(stderr, "cannot open %s\n", levelPath.c_str()); return 1; }
    json root;
    f >> root;
    json roadBlock;
    for (const json& e : root.value("entities", json::array()))
        if (e.contains("road")) { roadBlock = e["road"]; break; }
    if (roadBlock.is_null()) { std::fprintf(stderr, "no road entity\n"); return 1; }

    RoadEntity net = roadNetFromJson(roadBlock);
    if (roadBlock.contains("generate")) {
        json gen = roadBlock["generate"];
        if (minArm >= 0.0) gen["min_arm_angle_deg"] = minArm;   // A/B the realign pass
        if (runIn >= 0.0) gen["realign_run_in"] = runIn;
        if (dissolve >= 0.0) gen["dissolve_acute_deg"] = dissolve;
        applyGenerateRecipe(net, gen, nullptr);
    }

    // Two different graphs, and the difference matters: `--control` is what the
    // PLANNER produced (what a graph-level fix owns), while the default is the
    // sampled + constrained graph the mesher and the driver see. Passes on the
    // mesh path can re-node and merge, so a junction can be clean in one and not
    // the other.
    const RoadGraph g = control ? net.graph : roadNetFullGraph(net, nullptr);
    std::printf("# graph: %s\n", control ? "CONTROL (planner output)" : "SAMPLED (what the mesher sees)");
    int dupeEdges = 0;
    const auto adj = adjacency(g, &dupeEdges);

    std::printf("# JUNCTION ANGLE PROBE — %s\n", levelPath.c_str());
    std::printf("# %zu nodes, %zu edges", g.nodes.size(), g.edges.size());
    if (dupeEdges) std::printf("  (%d PARALLEL edge pairs)", dupeEdges);
    std::printf("\n");

    // ---------------------------------------------------------- junctions
    std::vector<Junction> js;
    std::map<int, int> byDegree;
    for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v) {
        const int d = static_cast<int>(adj[v].size());
        if (d < 3) continue;
        ++byDegree[d];
        struct Arm { double ang; int nbr; };
        std::vector<Arm> arms;
        arms.reserve(d);
        for (int n : adj[v]) {
            const Vec2 dv = g.nodes[n].pos - g.nodes[v].pos;
            if (dv.lengthSquared() < 1e-12) continue;
            arms.push_back({std::atan2(dv.y, dv.x), n});
        }
        if (arms.size() < 3) continue;
        std::sort(arms.begin(), arms.end(),
                  [](const Arm& x, const Arm& y) { return x.ang < y.ang; });
        Junction J;
        J.node = v;
        J.pos = g.nodes[v].pos;
        J.degree = static_cast<int>(arms.size());
        int tightK = 0;
        for (std::size_t k = 0; k < arms.size(); ++k) {
            double gap = arms[(k + 1) % arms.size()].ang - arms[k].ang;
            if (gap <= 0) gap += 2.0 * PI_;
            if (deg(gap) < J.tightest) { J.tightest = deg(gap); tightK = static_cast<int>(k); }
        }
        const int nA = arms[tightK].nbr;
        const int nB = arms[(tightK + 1) % arms.size()].nbr;
        J.roomA = runToNextJunction(g, adj, v, nA, &J.armLenA);
        J.roomB = runToNextJunction(g, adj, v, nB, &J.armLenB);
        js.push_back(J);
    }

    std::printf("\n## JUNCTIONS: %zu  (edges %zu — watch this when culling)\n",
                js.size(), g.edges.size());
    std::printf("  by degree:");
    for (const auto& kv : byDegree) std::printf("  %d-way x%d", kv.first, kv.second);
    std::printf("\n");

    // tightest-gap histogram, 10 deg buckets
    int hist[19] = {0};
    for (const Junction& J : js) hist[std::min(18, static_cast<int>(J.tightest / 10.0))]++;
    std::printf("\n## TIGHTEST GAP PER JUNCTION\n");
    for (int b = 0; b < 19; ++b) {
        if (!hist[b]) continue;
        std::printf("  %3d-%3d deg : %4d  ", b * 10, b * 10 + 10, hist[b]);
        for (int k = 0; k < hist[b]; k += 2) std::printf("#");
        std::printf("\n");
    }

    std::printf("\n## HOW MUCH WORK, BY CANDIDATE THRESHOLD\n");
    std::printf("  %-10s %-10s %-10s\n", "threshold", "violating", "share");
    for (double t : {50.0, 55.0, 60.0, 65.0, 70.0, 75.0}) {
        int n = 0;
        for (const Junction& J : js) if (J.tightest < t) ++n;
        std::printf("  %-10.0f %-10d %-10.0f%%\n", t, n, 100.0 * n / std::max<std::size_t>(1, js.size()));
    }

    // ---------------------------------------------------- rim vs core
    // Device, reading the city map: "a lot of these areas lie on the outer
    // boundary." Tightest gap against distance from the city's centre — if the
    // rim really is worse, the fix belongs where the outskirts are GROWN, not in
    // a global angle floor.
    if (!js.empty()) {
        Vec2 c(0, 0);
        for (const Junction& J : js) c += J.pos;
        c = c * (1.0 / static_cast<double>(js.size()));
        double rMax = 0;
        for (const Junction& J : js) rMax = std::max(rMax, (double)(J.pos - c).length());
        const int kRings = 5;
        std::vector<int> n(kRings, 0), tight(kRings, 0);
        std::vector<double> sum(kRings, 0.0);
        for (const Junction& J : js) {
            const double f = rMax > 1e-6 ? (J.pos - c).length() / rMax : 0.0;
            const int r = std::min(kRings - 1, static_cast<int>(f * kRings));
            ++n[r];
            sum[r] += J.tightest;
            if (J.tightest < target) ++tight[r];
        }
        std::printf("\n## RIM vs CORE (centre %.0f, %.0f; outer radius %.0f m)\n",
                    (double)c.x, (double)c.y, rMax);
        std::printf("  %-14s %-6s %-11s %-14s\n", "ring", "n", "mean tight", "under target");
        for (int r = 0; r < kRings; ++r) {
            if (!n[r]) continue;
            std::printf("  %3d-%3d%% out   %-6d %-11.1f %d (%.0f%%)\n",
                        r * 100 / kRings, (r + 1) * 100 / kRings, n[r], sum[r] / n[r],
                        tight[r], 100.0 * tight[r] / n[r]);
        }
    }

    // ------------------------------------------------- realign feasibility
    // A bend needs straight run to work in. Rule of thumb used here: opening a
    // gap by `d` degrees over a run of L metres asks the approach to shift
    // laterally by about L*sin(d) — so the shorter the arm, the harsher the
    // bend. Report the room actually available on the tight pair.
    std::printf("\n## REALIGN FEASIBILITY (violating at %.0f deg)\n", target);
    int nViol = 0, roomy = 0, tightRoom = 0, stubby = 0;
    for (const Junction& J : js) {
        if (J.tightest >= target) continue;
        ++nViol;
        const double room = std::min(J.roomA, J.roomB);
        if (room >= 60.0) ++roomy;
        else if (room >= 25.0) ++tightRoom;
        else ++stubby;
    }
    std::printf("  violating                       : %d\n", nViol);
    std::printf("  both arms >= 60 m of run        : %d  (comfortable bend)\n", roomy);
    std::printf("  25-60 m                         : %d  (tight but workable)\n", tightRoom);
    std::printf("  under 25 m                      : %d  (no room — roundabout or stagger)\n", stubby);

    // ------------------------------------------------------------- bends
    // Minimum curve radius anywhere on a through-run. This is the tipping
    // number: lateral acceleration is v^2/R, so at 50 km/h (13.9 m/s) a 0.4 g
    // limit wants R >= ~49 m.
    double worstR = 1e30;
    Vec2 worstAt(0, 0);
    int under30 = 0, under50 = 0, under80 = 0, samples = 0;
    for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v) {
        if (adj[v].size() != 2) continue;
        const double R = circumradius(g.nodes[adj[v][0]].pos, g.nodes[v].pos, g.nodes[adj[v][1]].pos);
        ++samples;
        if (R < 30.0) ++under30;
        if (R < 50.0) ++under50;
        if (R < 80.0) ++under80;
        if (R < worstR) { worstR = R; worstAt = g.nodes[v].pos; }
    }
    std::printf("\n## BENDS (through-nodes: %d)\n", samples);
    std::printf("  radius under 30 m               : %d  (%.1f%%)  — unpleasant at any speed\n",
                under30, 100.0 * under30 / std::max(1, samples));
    std::printf("  radius under 50 m               : %d  (%.1f%%)  — 0.4 g at 50 km/h\n",
                under50, 100.0 * under50 / std::max(1, samples));
    std::printf("  radius under 80 m               : %d  (%.1f%%)\n",
                under80, 100.0 * under80 / std::max(1, samples));
    std::printf("  tightest                        : %.1f m at (%.1f, %.1f)\n",
                worstR, (double)worstAt.x, (double)worstAt.y);

    // -------------------------------------------------- ladder dry run
    // What the policy WOULD do, with no geometry changed: realign where there is
    // room, otherwise roundabout (peer classes, space) or stagger.
    std::printf("\n## LADDER DRY RUN (target %.0f deg)\n", target);
    int wRealign = 0, wRing = 0, wStagger = 0;
    for (const Junction& J : js) {
        if (J.tightest >= target) continue;
        const double room = std::min(J.roomA, J.roomB);
        if (room >= 25.0) ++wRealign;
        else if (J.degree >= 4) ++wRing;
        else ++wStagger;
    }
    std::printf("  realign (bend the approach)     : %d\n", wRealign);
    std::printf("  roundabout (no room, 4+ arms)   : %d\n", wRing);
    std::printf("  stagger (no room, 3 arms)       : %d\n", wStagger);
    std::printf("  -> roundabouts would be %.0f%% of junctions before any budget cap\n",
                100.0 * wRing / std::max<std::size_t>(1, js.size()));

    std::printf("\n## WORST 12 (aim a camera at x/z)\n");
    std::sort(js.begin(), js.end(),
              [](const Junction& a, const Junction& b) { return a.tightest < b.tightest; });
    std::printf("  %-10s %-10s %-5s %-9s %-9s %-9s\n", "x", "z", "deg", "tightest", "roomA", "roomB");
    for (int i = 0; i < 12 && i < static_cast<int>(js.size()); ++i) {
        const Junction& J = js[i];
        std::printf("  %-10.2f %-10.2f %-5d %-9.1f %-9.1f %-9.1f\n",
                    (double)J.pos.x, (double)J.pos.y, J.degree, J.tightest, J.roomA, J.roomB);
    }

    if (!csvPath.empty()) {
        std::ofstream out(csvPath);
        out << "x,z,degree,tightestDeg,roomA,roomB,armLenA,armLenB\n";
        for (const Junction& J : js)
            out << J.pos.x << ',' << J.pos.y << ',' << J.degree << ',' << J.tightest << ','
                << J.roomA << ',' << J.roomB << ',' << J.armLenA << ',' << J.armLenB << '\n';
        std::printf("\nwrote %s\n", csvPath.c_str());
    }
    return 0;
}
