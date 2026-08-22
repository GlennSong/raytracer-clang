// Curb-weld diagnostic — does the curb/sidewalk BAND actually weld at junctions?
//
// Device report: "there are places where it simply fails and we have overlapping
// curbs or they're not properly dovetailed and rounded." This probe takes a
// shipped level, grows its road network exactly as the loader does, meshes it
// with the real mesher, and then measures the band the mesher actually emitted
// against three things a curb return must be:
//
//   ROUNDED   — at a junction corner the kerb turns through an ARC of about
//               look.cornerRadius, not a single mitre vertex. Measured on the
//               mesher's own band loops (CurbBandAudit): per corner, the total
//               turn, the arc length it is spread over, the implied radius, and
//               the worst single-vertex turn.
//   SINGLE    — no two pieces of sidewalk stack on the same ground. Measured by
//               rasterizing the emitted band-top triangles in a disc around each
//               junction and counting cells covered more than once.
//   CONTINUOUS— the band exists everywhere the asphalt boundary runs, except
//               where a non-street mouth (a ramp) deliberately gaps it.
//               Measured by walking the boundary loop and sampling outward for
//               band-top coverage.
//
// Every finding carries world (x, z) so a camera can be aimed at it:
//     ./build/editor_app   (or the viewer MCP set_camera)
//
// Usage:
//     ./build/curb_weld_probe [level.json] [--top N] [--radius M] [--csv out.csv]
// Default level: assets/levels/metro_v2_test.json
#include "../../src/engine/procgen/city/road_net.h"
#include "../../src/engine/procgen/city/road_network.h"
#include "../../src/engine/procgen/city/polygon.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace engine;
using json = nlohmann::json;

static const double PI_ = 3.14159265358979323846;

// ---------------------------------------------------------------- mesh access
// The band's two colours, straight from sweepCurbSidewalkBand.
static const Vec3 kWalkColor(0.62, 0.62, 0.60);
static const Vec3 kCurbColor(0.48, 0.48, 0.47);

static bool sameColor(const Vec3& a, const Vec3& b) {
    return std::fabs(a.x - b.x) < 0.02 && std::fabs(a.y - b.y) < 0.02 &&
           std::fabs(a.z - b.z) < 0.02;
}

struct Tri {
    Vec2 a, b, c;          // XZ
    double ya, yb, yc;     // world Y at each corner
    double minx, minz, maxx, maxz;
    bool walkTop;          // sidewalk slab top (up-facing, walk colour)
    bool curbFace;         // the kerb lip (walk-adjacent, near-vertical)
};

// STRICT interior: a sample sitting exactly on a shared edge would otherwise be
// counted by both of the quad's triangles and read as an overlap that is not
// there (the arena's 60-deg star proved it — 3.5 m2 of "overlap" with zero
// excess triangle area). Points on an edge are dropped from both, which
// under-counts by a hairline and never invents a fold.
static bool inTri(const Vec2& p, const Vec2& a, const Vec2& b, const Vec2& c) {
    const double d1 = cross(b - a, p - a), d2 = cross(c - b, p - b), d3 = cross(a - c, p - c);
    const double eps = 1e-9;
    return (d1 > eps && d2 > eps && d3 > eps) || (d1 < -eps && d2 < -eps && d3 < -eps);
}

// The carriageway/pad surface: dark, up-facing. The kerb line is supposed to sit
// exactly on this surface's edge — round the kerb line without rounding the
// asphalt and the two come apart, leaving bare ground between road and kerb.
static std::vector<Tri> asphaltTris(const RenderMesh& m) {
    std::vector<Tri> out;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vertex& va = m.vertices[m.indices[i]];
        const Vertex& vb = m.vertices[m.indices[i + 1]];
        const Vertex& vc = m.vertices[m.indices[i + 2]];
        auto dark = [](const Vec3& c) { return c.x < 0.30 && c.y < 0.30 && c.z < 0.30; };
        if (!dark(va.color) || !dark(vb.color) || !dark(vc.color)) continue;
        if ((va.normal.y + vb.normal.y + vc.normal.y) / 3.0 < 0.5) continue;
        Tri t;
        t.a = Vec2(va.position.x, va.position.z);
        t.b = Vec2(vb.position.x, vb.position.z);
        t.c = Vec2(vc.position.x, vc.position.z);
        t.ya = va.position.y; t.yb = vb.position.y; t.yc = vc.position.y;
        t.minx = std::min({t.a.x, t.b.x, t.c.x}); t.maxx = std::max({t.a.x, t.b.x, t.c.x});
        t.minz = std::min({t.a.y, t.b.y, t.c.y}); t.maxz = std::max({t.a.y, t.b.y, t.c.y});
        t.walkTop = true;   // reuse the flag: "this is the surface I asked for"
        t.curbFace = false;
        out.push_back(t);
    }
    return out;
}

static std::vector<Tri> bandTris(const RenderMesh& m) {
    std::vector<Tri> out;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vertex& va = m.vertices[m.indices[i]];
        const Vertex& vb = m.vertices[m.indices[i + 1]];
        const Vertex& vc = m.vertices[m.indices[i + 2]];
        // ANY vertex, not all three: MeshBuilder may share a vertex between the
        // slab top and the kerb face, and requiring all three dropped real band
        // triangles — which then read as "band missing" (they were not).
        const bool walk = sameColor(va.color, kWalkColor) || sameColor(vb.color, kWalkColor) ||
                          sameColor(vc.color, kWalkColor);
        const bool kerb = sameColor(va.color, kCurbColor) && sameColor(vb.color, kCurbColor) &&
                          sameColor(vc.color, kCurbColor);
        if (!walk && !kerb) continue;
        const double ny = (va.normal.y + vb.normal.y + vc.normal.y) / 3.0;
        Tri t;
        t.a = Vec2(va.position.x, va.position.z);
        t.b = Vec2(vb.position.x, vb.position.z);
        t.c = Vec2(vc.position.x, vc.position.z);
        t.ya = va.position.y; t.yb = vb.position.y; t.yc = vc.position.y;
        t.minx = std::min({t.a.x, t.b.x, t.c.x}); t.maxx = std::max({t.a.x, t.b.x, t.c.x});
        t.minz = std::min({t.a.y, t.b.y, t.c.y}); t.maxz = std::max({t.a.y, t.b.y, t.c.y});
        t.walkTop = walk && ny > 0.5;
        t.curbFace = kerb && std::fabs(ny) < 0.5;
        if (!t.walkTop && !t.curbFace) continue;
        out.push_back(t);
    }
    return out;
}

// Uniform grid over triangle AABBs, so a junction disc pulls only its neighbours.
struct TriGrid {
    double cell = 4.0;
    std::unordered_map<long long, std::vector<int>> buckets;
    const std::vector<Tri>* tris = nullptr;

    static long long key(int cx, int cz) {
        return (static_cast<long long>(cx) << 32) ^ (static_cast<long long>(cz) & 0xffffffffLL);
    }
    void build(const std::vector<Tri>& t) {
        tris = &t;
        for (int i = 0; i < static_cast<int>(t.size()); ++i)
            for (int cx = (int)std::floor(t[i].minx / cell); cx <= (int)std::floor(t[i].maxx / cell); ++cx)
                for (int cz = (int)std::floor(t[i].minz / cell); cz <= (int)std::floor(t[i].maxz / cell); ++cz)
                    buckets[key(cx, cz)].push_back(i);
    }
    // Every triangle index whose AABB touches the disc (p, r).
    std::vector<int> near(const Vec2& p, double r) const {
        std::vector<int> out;
        for (int cx = (int)std::floor((p.x - r) / cell); cx <= (int)std::floor((p.x + r) / cell); ++cx)
            for (int cz = (int)std::floor((p.y - r) / cell); cz <= (int)std::floor((p.y + r) / cell); ++cz) {
                auto it = buckets.find(key(cx, cz));
                if (it == buckets.end()) continue;
                out.insert(out.end(), it->second.begin(), it->second.end());
            }
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
        return out;
    }
};

// ------------------------------------------------------------- corner metrics
// One turn of the kerb line: a maximal run of same-signed turning vertices.
struct Corner {
    Vec2 at;                 // where the turn happens (mid-run)
    double totalTurnDeg = 0; // how far the kerb swings through the run
    double arcLen = 0;       // metres of kerb the swing is spread over
    double effRadius = 0;    // arcLen / totalTurn(rad) — the radius it reads as
    double maxVertTurnDeg = 0;   // worst single-vertex turn (a mitre spikes here)
    int    verts = 0;
};

static double turnAt(const Poly2& L, int i) {
    const int m = static_cast<int>(L.size());
    const Vec2 e0 = L[i] - L[(i + m - 1) % m];
    const Vec2 e1 = L[(i + 1) % m] - L[i];
    const double l0 = e0.length(), l1 = e1.length();
    if (l0 < 1e-9 || l1 < 1e-9) return 0.0;
    double c = dot(e0, e1) / (l0 * l1);
    c = std::max(-1.0, std::min(1.0, c));
    const double ang = std::acos(c) * 180.0 / PI_;
    return cross(e0, e1) >= 0 ? ang : -ang;   // signed: + = left turn
}

// Corners of `L` whose apex lies within `r` of `p`.
static std::vector<Corner> cornersNear(const Poly2& L, const Vec2& p, double r) {
    std::vector<Corner> out;
    const int m = static_cast<int>(L.size());
    if (m < 4) return out;
    std::vector<char> used(m, 0);
    for (int i = 0; i < m; ++i) {
        if (used[i]) continue;
        const double t0 = turnAt(L, i);
        if (std::fabs(t0) < 4.0) continue;            // straight enough to ignore
        // Grow the run while the kerb keeps turning the SAME way without a
        // straight stretch between — that whole run is one corner, whether the
        // mesher spread it over an arc or dumped it on one vertex.
        int lo = i, hi = i;
        const double sgn = t0 > 0 ? 1.0 : -1.0;
        while (true) {
            const int nxt = (hi + 1) % m;
            const double t = turnAt(L, nxt);
            if (t * sgn <= 0 || std::fabs(t) < 4.0 || nxt == lo) break;
            hi = nxt;
            if (hi == i) break;
        }
        Corner c;
        double sum = 0, arc = 0, worst = 0;
        Vec2 acc(0, 0);
        int n = 0;
        for (int k = lo;; k = (k + 1) % m) {
            const double t = turnAt(L, k);
            sum += std::fabs(t);
            worst = std::max(worst, std::fabs(t));
            acc += L[k];
            ++n;
            used[k] = 1;
            if (k != lo) arc += (L[k] - L[(k + m - 1) % m]).length();
            if (k == hi) break;
        }
        c.at = acc * (1.0 / std::max(1, n));
        if ((c.at - p).length() > r) continue;
        c.totalTurnDeg = sum;
        c.arcLen = arc;
        c.maxVertTurnDeg = worst;
        c.verts = n;
        c.effRadius = sum > 1e-6 ? arc / (sum * PI_ / 180.0) : 1e9;
        out.push_back(c);
    }
    return out;
}

// ------------------------------------------------------- band reach (spikes)
// The band should be `sidewalkWidth` wide, everywhere. A mitred corner blows the
// outer offset out to sidewalkWidth / max(0.2, cos(half-angle)) — up to 5x — and
// that spike is what crosses the junction and lands on the neighbouring kerb.
// Measured off the EMITTED mesh: the farthest any band-top vertex sits from the
// kerb line it belongs to.
struct SegGrid {
    double cell = 8.0;
    std::unordered_map<long long, std::vector<std::pair<Vec2, Vec2>>> buckets;
    static long long key(int cx, int cz) {
        return (static_cast<long long>(cx) << 32) ^ (static_cast<long long>(cz) & 0xffffffffLL);
    }
    void addLoops(const std::vector<Poly2>& loops) {
        for (const Poly2& L : loops)
            for (std::size_t i = 0; i < L.size(); ++i) {
                const Vec2 a = L[i], b = L[(i + 1) % L.size()];
                const double lo_x = std::min(a.x, b.x), hi_x = std::max(a.x, b.x);
                const double lo_z = std::min(a.y, b.y), hi_z = std::max(a.y, b.y);
                for (int cx = (int)std::floor(lo_x / cell); cx <= (int)std::floor(hi_x / cell); ++cx)
                    for (int cz = (int)std::floor(lo_z / cell); cz <= (int)std::floor(hi_z / cell); ++cz)
                        buckets[key(cx, cz)].push_back({a, b});
            }
    }
    // Distance from p to the nearest kerb segment (searching outward by rings).
    double dist(const Vec2& p) const {
        double best = 1e30;
        for (int ring = 0; ring <= 4 && best > ring * cell; ++ring) {
            const int cx0 = (int)std::floor(p.x / cell), cz0 = (int)std::floor(p.y / cell);
            for (int cx = cx0 - ring; cx <= cx0 + ring; ++cx)
                for (int cz = cz0 - ring; cz <= cz0 + ring; ++cz) {
                    if (ring > 0 && std::abs(cx - cx0) != ring && std::abs(cz - cz0) != ring) continue;
                    auto it = buckets.find(key(cx, cz));
                    if (it == buckets.end()) continue;
                    for (const auto& sg : it->second) {
                        const Vec2 ab = sg.second - sg.first;
                        const double l2 = ab.lengthSquared();
                        double t = l2 > 1e-12 ? dot(p - sg.first, ab) / l2 : 0.0;
                        t = t < 0 ? 0 : (t > 1 ? 1 : t);
                        best = std::min(best, (sg.first + ab * t - p).length());
                    }
                }
        }
        return best;
    }
};

// ------------------------------------------------------------------- findings
struct JunctionReport {
    Vec2 pos;
    int degree = 0;
    // rounding
    int corners = 0;
    int sharpCorners = 0;        // a single vertex eats > kSharpTurn of the swing
    double worstVertTurnDeg = 0;
    double minEffRadius = 1e9;
    // overlap
    double overlapArea = 0;      // m^2 of band-top covered more than once
    int    maxStack = 0;
    double overlapDropM = 0;     // worst Y difference between stacked band tops
    // continuity
    double kerbLen = 0;          // kerb metres inspected near this junction
    double holeLen = 0;          // ...with NO band beside it at all
    double narrowLen = 0;        // ...with a band under half the nominal width
    double minWidth = 1e9;
    double detachedLen = 0;      // kerb metres with no asphalt on the road side
    // spikes
    double maxReach = 0;         // farthest a band vertex sits from the kerb line
    Vec2   reachAt{0, 0};
    // rounding measured on the de-spurred kerb line
    int cleanCorners = 0, cleanRounded = 0;
    double score() const {       // rank: overlap dominates, then gaps, then sharpness
        return overlapArea * 3.0 + holeLen * 2.0 + narrowLen * 0.5 + sharpCorners * 2.0;
    }
};

static const double kSharpTurn = 30.0;   // deg absorbed by ONE vertex = a mitre


// ------------------------------------------------------------------- arena
// A synthetic N-arm star, so the failure has a minimal reproduction that does
// not depend on a 2 km level: one node at the origin, `arms` spokes of length
// `len`, the first at 0 deg and the rest spread by `spreadDeg`.
static RoadEntity star(int arms, double spreadDeg, double width) {
    RoadEntity net;
    net.look.defaultWidth = width;
    net.look.sidewalk = 3.5;
    net.look.curb = 0.16;
    net.look.cornerRadius = 3.0;
    net.look.autoRoundabout = false;
    RoadNode c; c.pos = Vec2(0, 0);
    net.graph.nodes.push_back(c);
    for (int i = 0; i < arms; ++i) {
        const double a = (i == 0 ? 0.0 : spreadDeg * i) * PI_ / 180.0;
        RoadNode n; n.pos = Vec2(std::cos(a) * 60.0, std::sin(a) * 60.0);
        net.graph.nodes.push_back(n);
        RoadEdge e; e.a = 0; e.b = static_cast<int>(net.graph.nodes.size()) - 1;
        e.width = static_cast<Real>(width);
        net.graph.edges.push_back(e);
    }
    return net;
}

static void runArena() {
    std::printf("# ARENA — synthetic star junctions (flat, no level).\n");
    std::printf("# want: reach == sidewalk (3.50), a rounded return (effR ~ cornerRadius 3.00),\n");
    std::printf("#       no double-covered band.\n");
    std::printf("# triArea vs covered: if the band never folded, they would match.\n");
    std::printf("%-6s %-8s %-8s %-9s %-8s %-8s %-7s %-9s %-9s\n",
                "arms", "spread", "reach", "worstTurn", "corners", "rounded", "ovl",
                "triArea", "covered");
    struct Case { int arms; double spread; };
    const std::vector<Case> cases = {
        {3, 90.0}, {3, 120.0}, {3, 60.0}, {3, 45.0}, {3, 30.0},
        {4, 90.0}, {4, 75.0}, {4, 60.0}, {4, 45.0}, {5, 72.0},
    };
    for (const Case& cs : cases) {
        RoadEntity net = star(cs.arms, cs.spread, 12.0);
        CurbBandAudit A;
        const RenderMesh m = buildRoadNetMesh(net, nullptr, &A);
        if (A.loops.empty()) { std::printf("%-6d %-8.0f  (no band)\n", cs.arms, cs.spread); continue; }
        std::vector<Tri> tris = bandTris(m);
        SegGrid kerbs; kerbs.addLoops(A.loops);
        double reach = 0, worstTurn = 0;
        int corners = 0, rounded = 0;
        for (const Tri& t : tris) {
            if (!t.walkTop) continue;
            for (const Vec2& v : {t.a, t.b, t.c}) reach = std::max(reach, kerbs.dist(v));
        }
        for (const Poly2& L : A.loops)
            for (const Corner& c : cornersNear(L, Vec2(0, 0), 40.0)) {
                if (c.totalTurnDeg < 25.0) continue;
                ++corners;
                worstTurn = std::max(worstTurn, c.maxVertTurnDeg);
                if (c.effRadius > net.look.cornerRadius * 0.5) ++rounded;
            }
        // overlap over the whole star, 0.25 m cells
        double ovl = 0, covered = 0, triArea = 0;
        // triArea over EVERY band-top triangle, covered over the whole bbox they
        // occupy — the same patch of ground, so the two are comparable: a band
        // that never folded covers exactly the area of its triangles.
        double bx0 = 1e30, bx1 = -1e30, bz0 = 1e30, bz1 = -1e30;
        for (const Tri& t : tris)
            if (t.walkTop) {
                triArea += std::fabs(cross(t.b - t.a, t.c - t.a)) * 0.5;
                bx0 = std::min(bx0, t.minx); bx1 = std::max(bx1, t.maxx);
                bz0 = std::min(bz0, t.minz); bz1 = std::max(bz1, t.maxz);
            }
        TriGrid g; g.build(tris);
        for (double x = bx0 - 0.5; x < bx1 + 0.5; x += 0.25)
            for (double z = bz0 - 0.5; z < bz1 + 0.5; z += 0.25) {
                const Vec2 p(x + 0.125, z + 0.125);
                int stack = 0;
                for (int ti : g.near(p, 0.3)) {
                    const Tri& t = tris[ti];
                    if (!t.walkTop) continue;
                    if (p.x < t.minx || p.x > t.maxx || p.y < t.minz || p.y > t.maxz) continue;
                    if (inTri(p, t.a, t.b, t.c)) ++stack;
                }
                if (stack >= 1) covered += 0.0625;
                if (stack >= 2) ovl += 0.0625;
            }
        std::printf("%-6d %-8.0f %-8.2f %-9.1f %-8d %-8d %-7.1f %-9.1f %-9.1f\n",
                    cs.arms, cs.spread, reach, worstTurn, corners, rounded, ovl,
                    triArea, covered);
    }
}

int main(int argc, char** argv) {
    std::string levelPath = "assets/levels/metro_v2_test.json";
    int topN = 20;
    double discR = 18.0;
    std::string csvPath, svgPath;
    bool haveFocus = false;
    Vec2 focus(0, 0);
    double focusR = 30.0;
    double cornerOverride = -1.0;   // >= 0 overrides look.cornerRadius
    double walkOverride = -1.0;     // >= 0 overrides look.sidewalk
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--focus" && i + 3 < argc) {
            focus = Vec2(std::atof(argv[i + 1]), std::atof(argv[i + 2]));
            focusR = std::atof(argv[i + 3]);
            i += 3; haveFocus = true;
        }
        else if (a == "--svg" && i + 1 < argc) svgPath = argv[++i];
        else if (a == "--top" && i + 1 < argc) topN = std::atoi(argv[++i]);
        else if (a == "--radius" && i + 1 < argc) discR = std::atof(argv[++i]);
        else if (a == "--csv" && i + 1 < argc) csvPath = argv[++i];
        else if (a == "--corner" && i + 1 < argc) cornerOverride = std::atof(argv[++i]);
        else if (a == "--sidewalk" && i + 1 < argc) walkOverride = std::atof(argv[++i]);
        else if (a == "--arena") { runArena(); return 0; }
        else if (a.size() && a[0] != '-') levelPath = a;
    }

    std::ifstream f(levelPath);
    if (!f.is_open()) { std::fprintf(stderr, "cannot open %s\n", levelPath.c_str()); return 1; }
    json root;
    f >> root;

    // The road entity, built the way the loader builds it: JSON look + recipe.
    json roadBlock;
    for (const json& e : root.value("entities", json::array()))
        if (e.contains("road")) { roadBlock = e["road"]; break; }
    if (roadBlock.is_null()) { std::fprintf(stderr, "no road entity in %s\n", levelPath.c_str()); return 1; }

    RoadEntity net = roadNetFromJson(roadBlock);
    if (roadBlock.contains("generate"))
        applyGenerateRecipe(net, roadBlock["generate"], nullptr);   // flat: XZ topology is height-free

    if (cornerOverride >= 0.0) net.look.cornerRadius = cornerOverride;
    if (walkOverride >= 0.0) net.look.sidewalk = walkOverride;
    CurbBandAudit audit;
    const RenderMesh mesh = buildRoadNetMesh(net, nullptr, &audit);

    std::vector<Tri> tris = bandTris(mesh);
    TriGrid grid;
    grid.build(tris);
    std::vector<Tri> road = asphaltTris(mesh);
    TriGrid roadGrid;
    roadGrid.build(road);

    std::size_t nWalk = 0, nCurb = 0;
    for (const Tri& t : tris) { nWalk += t.walkTop; nCurb += t.curbFace; }

    std::printf("# CURB WELD PROBE — %s\n", levelPath.c_str());
    std::printf("# look: sidewalk %.2f m, curb %.2f m, cornerRadius %.2f m\n",
                net.look.sidewalk, net.look.curb, net.look.cornerRadius);
    std::printf("# graph: %zu control nodes, %zu edges | band: %zu loops, %zu mouth gaps\n",
                net.graph.nodes.size(), net.graph.edges.size(), audit.loops.size(),
                audit.mouthGaps.size());
    std::printf("# mesh: %zu tris total, %zu band-top, %zu kerb-face | junctions (deg>=3): %zu\n",
                mesh.indices.size() / 3, nWalk, nCurb, audit.junctions.size());
    if (audit.loops.empty() || audit.junctions.empty()) {
        std::fprintf(stderr, "no band or no junctions — nothing to measure\n");
        return 2;
    }

    // Loop vertices in a grid too, so a junction pulls only its own kerb line.
    std::unordered_map<long long, std::vector<std::pair<int, int>>> loopGrid;   // cell -> (loop, vert)
    const double lcell = 8.0;
    auto lkey = [&](const Vec2& p) {
        return (static_cast<long long>((long long)std::floor(p.x / lcell)) << 32) ^
               (static_cast<long long>((long long)std::floor(p.y / lcell)) & 0xffffffffLL);
    };
    for (int li = 0; li < static_cast<int>(audit.loops.size()); ++li)
        for (int vi = 0; vi < static_cast<int>(audit.loops[li].size()); ++vi)
            loopGrid[lkey(audit.loops[li][vi])].push_back({li, vi});

    auto loopsNear = [&](const Vec2& p, double r) {
        std::vector<int> out;
        for (int cx = (int)std::floor((p.x - r) / lcell); cx <= (int)std::floor((p.x + r) / lcell); ++cx)
            for (int cz = (int)std::floor((p.y - r) / lcell); cz <= (int)std::floor((p.y + r) / lcell); ++cz) {
                auto it = loopGrid.find((static_cast<long long>(cx) << 32) ^
                                        (static_cast<long long>(cz) & 0xffffffffLL));
                if (it == loopGrid.end()) continue;
                for (const auto& lv : it->second) out.push_back(lv.first);
            }
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
        return out;
    };

    auto inMouthGap = [&](const Vec2& p) {
        for (const auto& g : audit.mouthGaps) {
            const Vec2 ab = g.second - g.first;
            const double l2 = ab.lengthSquared();
            double t = l2 > 1e-12 ? dot(p - g.first, ab) / l2 : 0.0;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            if ((g.first + ab * t - p).length() < 0.8) return true;
        }
        return false;
    };

    // ------------------------------------------------------------------ focus
    // White-box dump of ONE spot: which loops run through it, how they wind, and
    // an SVG so the geometry can be looked at instead of argued about.
    if (haveFocus) {
        std::printf("\n## FOCUS (%.2f, %.2f) r=%.1f\n", focus.x, focus.y, focusR);
        const std::vector<int> ls = loopsNear(focus, focusR);
        std::printf("  %zu loops pass within %.1f m\n", ls.size(), focusR);
        std::printf("  %-6s %-8s %-11s %-9s %-8s %-10s\n",
                    "loop", "verts", "signedArea", "vertsHere", "minTurn", "maxTurn");
        for (int li : ls) {
            const Poly2& L = audit.loops[li];
            int here = 0; double mn = 1e9, mx = -1e9;
            for (int i = 0; i < static_cast<int>(L.size()); ++i)
                if ((L[i] - focus).length() <= focusR) {
                    ++here;
                    const double t = std::fabs(turnAt(L, i));
                    mn = std::min(mn, t); mx = std::max(mx, t);
                }
            std::printf("  %-6d %-8zu %-11.1f %-9d %-8.1f %-10.1f\n",
                        li, L.size(), (double)signedArea(L), here,
                        here ? mn : 0.0, here ? mx : 0.0);
        }
        // The kerb line itself, vertex by vertex: does the junction pad's own
        // rounded corner (junctionPatch's fillet) survive the union + snap into
        // the loop the band is swept along?
        for (int li : ls) {
            const Poly2& L = audit.loops[li];
            std::printf("  -- loop %d, vertices within 14 m (x, z, edge-len, turn)\n", li);
            for (int i = 0; i < static_cast<int>(L.size()); ++i) {
                if ((L[i] - focus).length() > 14.0) continue;
                const Vec2 prev = L[(i + L.size() - 1) % L.size()];
                std::printf("     %9.3f %9.3f   %6.3f  %7.1f\n", (double)L[i].x, (double)L[i].y,
                            (double)(L[i] - prev).length(), turnAt(L, i));
            }
        }
        if (!svgPath.empty()) {
            std::ofstream sv(svgPath);
            const double S = 900.0 / (2 * focusR);
            auto X = [&](double wx) { return (wx - (focus.x - focusR)) * S; };
            auto Y = [&](double wz) { return (wz - (focus.y - focusR)) * S; };
            sv << "<svg xmlns='http://www.w3.org/2000/svg' width='900' height='900' "
                  "viewBox='0 0 900 900'><rect width='900' height='900' fill='#111'/>\n";
            // every band triangle in view: band tops blue, kerb faces orange
            for (const Tri& t : tris) {
                if (t.maxx < focus.x - focusR || t.minx > focus.x + focusR ||
                    t.maxz < focus.y - focusR || t.minz > focus.y + focusR) continue;
                const char* fill = t.walkTop ? "#3a6ea5" : "#a55a2a";
                sv << "<polygon points='" << X(t.a.x) << ',' << Y(t.a.y) << ' '
                   << X(t.b.x) << ',' << Y(t.b.y) << ' ' << X(t.c.x) << ',' << Y(t.c.y)
                   << "' fill='" << fill << "' fill-opacity='0.35' stroke='"
                   << (t.walkTop ? "#7fb8ff" : "#e09a5a") << "' stroke-width='0.4'/>\n";
            }
            // the mesher's band loops, one hue each
            const char* hues[] = {"#ff4d4d", "#4dff88", "#ffd24d", "#c04dff",
                                  "#4dd2ff", "#ff8ad2", "#9dff4d", "#ff9a4d"};
            int hi = 0;
            for (int li : ls) {
                const Poly2& L = audit.loops[li];
                const char* col = hues[hi++ % 8];
                for (int i = 0; i < static_cast<int>(L.size()); ++i) {
                    const Vec2 a = L[i], b = L[(i + 1) % L.size()];
                    if ((a - focus).length() > focusR * 1.6 && (b - focus).length() > focusR * 1.6)
                        continue;
                    sv << "<line x1='" << X(a.x) << "' y1='" << Y(a.y) << "' x2='" << X(b.x)
                       << "' y2='" << Y(b.y) << "' stroke='" << col << "' stroke-width='1.6'/>\n";
                    sv << "<circle cx='" << X(a.x) << "' cy='" << Y(a.y)
                       << "' r='2' fill='" << col << "'/>\n";
                }
            }
            for (std::size_t j = 0; j < audit.junctions.size(); ++j) {
                const Vec2& p = audit.junctions[j];
                if ((p - focus).length() > focusR) continue;
                sv << "<circle cx='" << X(p.x) << "' cy='" << Y(p.y)
                   << "' r='5' fill='none' stroke='#fff' stroke-width='2'/>\n";
            }
            sv << "</svg>\n";
            std::printf("  wrote %s (blue = sidewalk top, orange = kerb face, "
                        "coloured lines = band loops)\n", svgPath.c_str());
        }
    }

    // A HAIRLINE SPUR: a sub-5 cm edge across which the kerb reverses. The band
    // snap-rounds loops to 1 cm and then drops points closer than 0.5 cm — so a
    // snap can leave two points exactly 1 cm apart (the quantum), above the
    // dedupe epsilon, and the pair survives as a reversing spur. Count them, and
    // build a cleaned copy so the rounding metric can be run with and without.
    int spurs = 0;
    double spurWorstTurn = 0;
    std::vector<Poly2> clean;
    clean.reserve(audit.loops.size());
    for (const Poly2& L : audit.loops) {
        for (int i = 0; i < static_cast<int>(L.size()); ++i) {
            const double elen = (L[i] - L[(i + L.size() - 1) % L.size()]).length();
            const double t = std::fabs(turnAt(L, i));
            if (elen < 0.05 && t > 90.0) { ++spurs; spurWorstTurn = std::max(spurWorstTurn, t); }
        }
        Poly2 c;
        for (const Vec2& p : L)
            if (c.empty() || (p - c.back()).length() > 0.05) c.push_back(p);
        while (c.size() > 3 && (c.front() - c.back()).length() <= 0.05) c.pop_back();
        if (c.size() >= 3) clean.push_back(std::move(c));
    }

    std::vector<std::pair<Vec2, Vec2>> gapSamples;   // (kerb point, uncovered probe point)

    // WHICH SIDE is each loop's band on? Deriving it from the winding is not
    // safe — polygonUnion's loops do not all wind the same way, and a wrong
    // guess reports a perfectly good kerb as missing its band. Decide it by
    // evidence: sample both sides along the loop and take the side the mesher
    // actually put band on. (+1 = the right normal, -1 = the left.)
    std::vector<double> loopSide(audit.loops.size(), 1.0);
    {
        TriGrid probeGrid;
        probeGrid.build(tris);
        for (std::size_t li = 0; li < audit.loops.size(); ++li) {
            const Poly2& L = audit.loops[li];
            int right = 0, left = 0;
            const int stride = std::max(1, static_cast<int>(L.size()) / 24);
            for (std::size_t i = 0; i < L.size(); i += stride) {
                const Vec2 a = L[i], b = L[(i + 1) % L.size()];
                const Vec2 e = b - a;
                const double len = e.length();
                if (len < 1e-6) continue;
                const Vec2 n(e.y / len, -e.x / len);
                const Vec2 mid = a + e * 0.5;
                for (int sgn = -1; sgn <= 1; sgn += 2) {
                    const Vec2 q = mid + n * (sgn * audit.sidewalkWidth * 0.5);
                    for (int ti : probeGrid.near(q, 0.4)) {
                        const Tri& t = tris[ti];
                        if (!t.walkTop) continue;
                        if (q.x < t.minx || q.x > t.maxx || q.y < t.minz || q.y > t.maxz) continue;
                        if (inTri(q, t.a, t.b, t.c)) { (sgn > 0 ? right : left)++; break; }
                    }
                }
            }
            loopSide[li] = (left > right) ? -1.0 : 1.0;
        }
    }

    SegGrid kerbs;
    kerbs.addLoops(audit.loops);

    // ------------------------------------------------------------- per junction
    std::vector<JunctionReport> reps;
    reps.reserve(audit.junctions.size());
    const double raster = 0.25;

    for (std::size_t ji = 0; ji < audit.junctions.size(); ++ji) {
        JunctionReport R;
        R.pos = audit.junctions[ji];
        R.degree = audit.junctionDegree[ji];

        // -- rounding, on the CLEANED kerb line: what the corner geometry would
        //    read as if the hairline spurs were not in the way.
        for (const Poly2& L : clean) {
            for (const Corner& c : cornersNear(L, R.pos, discR)) {
                if (c.totalTurnDeg < 25.0) continue;
                ++R.cleanCorners;
                if (c.effRadius > 1.5) ++R.cleanRounded;
            }
        }
        // -- rounding: every kerb corner inside the disc, as the band gets it
        for (int li : loopsNear(R.pos, discR)) {
            for (const Corner& c : cornersNear(audit.loops[li], R.pos, discR)) {
                ++R.corners;
                if (c.maxVertTurnDeg > kSharpTurn) ++R.sharpCorners;
                R.worstVertTurnDeg = std::max(R.worstVertTurnDeg, c.maxVertTurnDeg);
                if (c.totalTurnDeg > 25.0)     // only real returns, not kinks
                    R.minEffRadius = std::min(R.minEffRadius, c.effRadius);
            }
        }

        // -- overlap: rasterize band tops in the disc, count multi-cover cells
        const std::vector<int> cand = grid.near(R.pos, discR);
        const int steps = static_cast<int>(discR / raster);
        for (int ix = -steps; ix <= steps; ++ix) {
            for (int iz = -steps; iz <= steps; ++iz) {
                const Vec2 p(R.pos.x + (ix + 0.5) * raster, R.pos.y + (iz + 0.5) * raster);
                if ((p - R.pos).length() > discR) continue;
                int stack = 0;
                double ylo = 1e30, yhi = -1e30;
                for (int ti : cand) {
                    const Tri& t = tris[ti];
                    if (!t.walkTop) continue;
                    if (p.x < t.minx || p.x > t.maxx || p.y < t.minz || p.y > t.maxz) continue;
                    if (!inTri(p, t.a, t.b, t.c)) continue;
                    ++stack;
                    const double y = (t.ya + t.yb + t.yc) / 3.0;
                    ylo = std::min(ylo, y); yhi = std::max(yhi, y);
                }
                if (stack >= 2) {
                    R.overlapArea += raster * raster;
                    R.maxStack = std::max(R.maxStack, stack);
                    R.overlapDropM = std::max(R.overlapDropM, yhi - ylo);
                }
            }
        }

        // -- spikes: how far does the emitted band get from the kerb line?
        for (int ti : cand) {
            const Tri& t = tris[ti];
            if (!t.walkTop) continue;
            for (const Vec2& v : {t.a, t.b, t.c}) {
                if ((v - R.pos).length() > discR) continue;
                const double d = kerbs.dist(v);
                if (d > R.maxReach) { R.maxReach = d; R.reachAt = v; }
            }
        }

        // -- continuity: walk the kerb line, sample toward the band side
        for (int li : loopsNear(R.pos, discR)) {
            const Poly2& L = audit.loops[li];
            const double side = loopSide[li];
            const int m = static_cast<int>(L.size());
            for (int i = 0; i < m; ++i) {
                const Vec2 a = L[i], b = L[(i + 1) % m];
                const Vec2 e = b - a;
                const double len = e.length();
                if (len < 1e-6) continue;
                const Vec2 outward = Vec2(e.y, -e.x) * (side / len);   // the side the band is on
                const int ns = std::max(1, static_cast<int>(len / 0.5));
                for (int s = 0; s < ns; ++s) {
                    const Vec2 p = a + e * ((s + 0.5) / ns);
                    if ((p - R.pos).length() > discR) continue;
                    if (inMouthGap(p)) continue;
                    // March outward and find how wide the band actually is here.
                    // A HOLE (nothing at all beside the kerb) and a NARROWING
                    // (band present but pinched) are different defects: the trim
                    // trades the second for the first on purpose, so they must
                    // be counted apart.
                    double width = 0.0;
                    for (int k = 1; k <= 14; ++k) {
                        const double dist = k * (audit.sidewalkWidth / 14.0);
                        const Vec2 q = p + outward * dist;
                        bool covered = false;
                        for (int ti : cand) {
                            const Tri& t = tris[ti];
                            if (!t.walkTop) continue;
                            if (q.x < t.minx || q.x > t.maxx || q.y < t.minz ||
                                q.y > t.maxz) continue;
                            if (inTri(q, t.a, t.b, t.c)) { covered = true; break; }
                        }
                        if (!covered) break;
                        width = dist;
                    }
                    // Is the asphalt still right there on the other side of the
                    // kerb? 0.3 m in, because the kerb line IS the asphalt edge.
                    const Vec2 inward = p - outward * 0.3;
                    bool onRoad = false;
                    for (int ti : roadGrid.near(inward, 0.4)) {
                        const Tri& t = road[ti];
                        if (inward.x < t.minx || inward.x > t.maxx || inward.y < t.minz ||
                            inward.y > t.maxz) continue;
                        if (inTri(inward, t.a, t.b, t.c)) { onRoad = true; break; }
                    }
                    const double seg = len / ns;
                    if (!onRoad) R.detachedLen += seg;
                    R.kerbLen += seg;
                    if (width < 0.3) R.holeLen += seg;
                    else if (width < audit.sidewalkWidth * 0.5) R.narrowLen += seg;
                    R.minWidth = std::min(R.minWidth, width);
                }
            }
        }
        reps.push_back(R);
    }

    // ---------------------------------------------------------------- summary
    int nSharp = 0, nOverlap = 0, nGap = 0, nRounded = 0, nSpike = 0;
    double totOverlap = 0, totGap = 0, worstTurn = 0, worstReach = 0;
    std::vector<double> reaches;
    for (const JunctionReport& R : reps) {
        if (R.sharpCorners > 0) ++nSharp;
        if (R.overlapArea > 0.5) ++nOverlap;
        if (R.holeLen > 1.0) ++nGap;
        if (R.minEffRadius < 1e8 && R.minEffRadius > net.look.cornerRadius * 0.5) ++nRounded;
        if (R.maxReach > audit.sidewalkWidth * 1.5) ++nSpike;
        totOverlap += R.overlapArea;
        totGap += R.holeLen;
        worstTurn = std::max(worstTurn, R.worstVertTurnDeg);
        worstReach = std::max(worstReach, R.maxReach);
        reaches.push_back(R.maxReach);
    }
    std::sort(reaches.begin(), reaches.end());
    std::printf("\n## CENSUS over %zu junctions\n", reps.size());
    std::printf("  mitred (a corner where ONE vertex eats > %.0f deg) : %d  (%.1f%%)\n",
                kSharpTurn, nSharp, 100.0 * nSharp / reps.size());
    std::printf("  kerb returns actually rounded to >= %.1f m         : %d  (%.1f%%)\n",
                net.look.cornerRadius * 0.5, nRounded, 100.0 * nRounded / reps.size());
    std::printf("  overlapping band (> 0.5 m2 double-covered)         : %d  (%.1f%%), %.1f m2 total\n",
                nOverlap, 100.0 * nOverlap / reps.size(), totOverlap);
    double totKerb = 0, totNarrow = 0;
    for (const JunctionReport& R : reps) { totKerb += R.kerbLen; totNarrow += R.narrowLen; }
    std::printf("  kerb with NO band beside it (a hole)              : %.1f m of %.1f m (%.1f%%)\n",
                totGap, totKerb, totKerb > 0 ? 100.0 * totGap / totKerb : 0.0);
    std::printf("  kerb whose band is under half width (narrowed)    : %.1f m (%.1f%%)\n",
                totNarrow, totKerb > 0 ? 100.0 * totNarrow / totKerb : 0.0);
    double totDetached = 0;
    for (const JunctionReport& R : reps) totDetached += R.detachedLen;
    std::printf("  kerb with NO asphalt on its road side (detached)   : %.1f m (%.1f%%)\n",
                totDetached, totKerb > 0 ? 100.0 * totDetached / totKerb : 0.0);
    std::printf("  band SPIKES past %.1f m (1.5x the %.1f m walk)      : %d  (%.1f%%)\n",
                audit.sidewalkWidth * 1.5, audit.sidewalkWidth, nSpike,
                100.0 * nSpike / reps.size());
    std::printf("  worst single-vertex kerb turn                      : %.1f deg\n", worstTurn);
    std::printf("  band reach from the kerb  median %.2f m / worst %.2f m (want %.2f m)\n",
                reaches[reaches.size() / 2], worstReach, audit.sidewalkWidth);
    int cc = 0, cr = 0;
    for (const JunctionReport& R : reps) { cc += R.cleanCorners; cr += R.cleanRounded; }
    if (!gapSamples.empty()) {
        std::printf("\n## UNCOVERED SAMPLES (kerb point -> probe point, first %zu)\n",
                    gapSamples.size());
        for (const auto& gs : gapSamples)
            std::printf("  kerb (%9.3f, %9.3f)  probe (%9.3f, %9.3f)\n",
                        (double)gs.first.x, (double)gs.first.y,
                        (double)gs.second.x, (double)gs.second.y);
    }
    std::printf("\n## SPURS vs GEOMETRY\n");
    std::printf("  hairline spurs (< 5 cm edge, kerb reverses > 90 deg) : %d over %zu loops"
                " (worst turn %.1f deg)\n", spurs, audit.loops.size(), spurWorstTurn);
    std::printf("  junction corners on the DE-SPURRED kerb line         : %d, of which"
                " rounded (arc radius > 1.5 m): %d (%.1f%%)\n",
                cc, cr, cc ? 100.0 * cr / cc : 0.0);

    // Where do the kerb-line turns actually sit? The band's snap/de-spike pass
    // drops a vertex only when dot(e0,e1) < -0.985 (a turn steeper than ~170
    // deg). If the mass of junction turns sits JUST under that cut, the filter
    // is the reason the hairpins survive into the band.
    {
        int hist[10] = {0};
        int total = 0;
        for (const Poly2& L : audit.loops)
            for (int i = 0; i < static_cast<int>(L.size()); ++i) {
                const double t = std::fabs(turnAt(L, i));
                if (t < 4.0) continue;
                ++total;
                ++hist[std::min(9, static_cast<int>(t / 18.0))];
            }
        std::printf("\n## KERB-LINE TURN DISTRIBUTION (%d turning vertices over %zu loops)\n",
                    total, audit.loops.size());
        for (int b = 0; b < 10; ++b)
            if (hist[b])
                std::printf("  %3d-%3d deg : %6d  %s\n", b * 18, (b + 1) * 18, hist[b],
                            b == 9 ? "<- the de-spike filter cuts at ~170 deg" : "");
    }

    std::sort(reps.begin(), reps.end(),
              [](const JunctionReport& a, const JunctionReport& b) { return a.score() > b.score(); });

    std::printf("\n## WORST %d JUNCTIONS (aim a camera at x/z)\n", topN);
    std::printf("%-10s %-10s %-4s %-7s %-6s %-9s %-7s %-6s %-7s %-7s\n",
                "x", "z", "deg", "corners", "sharp", "worstTurn", "reach", "ovl", "stack", "gap");
    for (int i = 0; i < topN && i < static_cast<int>(reps.size()); ++i) {
        const JunctionReport& R = reps[i];
        std::printf("%-10.2f %-10.2f %-4d %-7d %-6d %-9.1f %-7.2f %-6.1f %-7d %-7.1f\n",
                    R.pos.x, R.pos.y, R.degree, R.corners, R.sharpCorners, R.worstVertTurnDeg,
                    R.maxReach, R.overlapArea, R.maxStack, R.holeLen);
    }

    if (!csvPath.empty()) {
        std::ofstream out(csvPath);
        out << "x,z,degree,corners,sharpCorners,worstVertTurnDeg,minEffRadius,"
               "overlapArea,maxStack,overlapDropM,holeLen,narrowLen,maxReach\n";
        for (const JunctionReport& R : reps)
            out << R.pos.x << ',' << R.pos.y << ',' << R.degree << ',' << R.corners << ','
                << R.sharpCorners << ',' << R.worstVertTurnDeg << ','
                << (R.minEffRadius > 1e8 ? -1.0 : R.minEffRadius) << ',' << R.overlapArea << ','
                << R.maxStack << ',' << R.overlapDropM << ',' << R.holeLen << ','
                << R.narrowLen << ',' << R.maxReach << '\n';
        std::printf("\nwrote %s\n", csvPath.c_str());
    }
    return 0;
}
