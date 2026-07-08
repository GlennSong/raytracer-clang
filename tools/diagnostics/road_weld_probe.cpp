// Road-junction weld diagnostic.
//   arena : synthetic N-arm junctions across angles/grades -> which cases weld
//   city  : sweep every junction of the living_city net -> rank the worst welds
//
// Metric per junction: build the local arm ribbons + junction pad exactly as the
// mesher does, boolean-union them (the CURB boundary), then rasterize a disc and
// measure GAP = area the curb encloses that the actual deck mesh does NOT cover
// (the terrain-through-the-junction failure). Also reports the min approach angle
// and how fragmented polygonUnion's output was (a robustness proxy).
#include "../../src/engine/procgen/city/road_net.h"
#include "../../src/engine/procgen/city/road_network.h"
#include "../../src/engine/procgen/city/road_offset.h"
#include "../../src/engine/procgen/city/polygon.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>
using namespace engine;

static const double TAU_PI = 3.14159265358979323846;

// ---- point-in-triangle (XZ) for mesh coverage ------------------------------
static bool inTri(const Vec2& p, const Vec2& a, const Vec2& b, const Vec2& c) {
    double d1 = cross(b - a, p - a), d2 = cross(c - b, p - b), d3 = cross(a - c, p - c);
    bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(neg && pos);
}

// Top-facing deck triangles of a road mesh, as XZ triangles (with a tiny AABB).
struct Tri { Vec2 a, b, c; double minx, minz, maxx, maxz; };
static std::vector<Tri> topTris(const RenderMesh& m) {
    std::vector<Tri> out;
    for (size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vertex& va = m.vertices[m.indices[i]];
        const Vertex& vb = m.vertices[m.indices[i + 1]];
        const Vertex& vc = m.vertices[m.indices[i + 2]];
        // top surface only (deck + sidewalk tops face up); skip walls/undersides
        double ny = (va.normal.y + vb.normal.y + vc.normal.y) / 3.0;
        if (ny < 0.5) continue;
        Tri t{Vec2(va.position.x, va.position.z), Vec2(vb.position.x, vb.position.z),
              Vec2(vc.position.x, vc.position.z), 0, 0, 0, 0};
        t.minx = std::min({t.a.x, t.b.x, t.c.x}); t.maxx = std::max({t.a.x, t.b.x, t.c.x});
        t.minz = std::min({t.a.y, t.b.y, t.c.y}); t.maxz = std::max({t.a.y, t.b.y, t.c.y});
        out.push_back(t);
    }
    return out;
}

struct JunctionInfo {
    int node; Vec2 pos; int degree; double minAngleDeg;
    double gapArea;      // m^2 of curb-enclosed ground the deck misses
    double curbArea;     // m^2 the curb encloses (near the node)
    double gapFrac;      // gapArea / curbArea
    int    unionLoops;   // polygonUnion outer-loop count for arms+pad (1 = clean)
    double padRadius;
    double gapCx, gapCz, gapMinR, gapMaxR;
};

// Build the local junction model (arm stubs + pad), union them, and measure the
// uncovered area against the real mesh's top triangles.
static JunctionInfo measure(const RoadGraph& g, int v,
                            const std::vector<std::vector<std::pair<int,double>>>& adj,
                            const std::vector<Tri>& tris, double sidewalk) {
    JunctionInfo J{}; J.node = v; J.pos = g.nodes[v].pos; J.degree = (int)adj[v].size();
    // arm dir + width + length-to-next-node (clip the stub so it can't overshoot
    // into a neighbouring junction, which would fake a gap in the local model).
    struct Arm { Vec2 dir; double width; double len; };
    std::vector<Arm> arms;
    double widest = 0;
    for (auto& [nb, w] : adj[v]) {
        Vec2 d = g.nodes[nb].pos - g.nodes[v].pos;
        double L = d.length();
        if (L < 1e-6) continue;
        arms.push_back({normalize(d), w, L});
        widest = std::max(widest, w);
    }
    if (arms.size() < 2) { J.minAngleDeg = 180; return J; }
    // min angle between adjacent arms (sorted by bearing)
    std::sort(arms.begin(), arms.end(), [](const Arm& a, const Arm& b){
        return std::atan2(a.dir.y, a.dir.x) < std::atan2(b.dir.y, b.dir.x); });
    double minAng = 360;
    for (size_t i = 0; i < arms.size(); ++i) {
        Vec2 a = arms[i].dir, b = arms[(i + 1) % arms.size()].dir;
        double ang = std::acos(std::max(-1.0, std::min(1.0, dot(a, b)))) * 180 / TAU_PI;
        minAng = std::min(minAng, ang);
    }
    J.minAngleDeg = minAng;
    J.padRadius = widest * 0.5 * 1.02;

    // Local ribbons: a straight stub per arm, clipped to 85% of the distance to
    // the next node, plus the junction pad disc — the union the mesher forms.
    double reach = J.padRadius + 0.5 * widest + 8.0;
    std::vector<Poly2> polys;
    for (const Arm& a : arms) {
        double L = std::min(reach, a.len * 0.85);
        if (L < J.padRadius + 0.5) L = J.padRadius + 0.5;   // always clear the pad
        std::vector<Vec2> cl = { g.nodes[v].pos - a.dir * 1.0, g.nodes[v].pos + a.dir * L };
        Poly2 rb = ribbonOutline(cl, a.width * 0.5, 4.0);
        if (rb.size() >= 3) polys.push_back(rb);
    }
    Poly2 pad;
    for (int k = 0; k < 24; ++k) {
        double t = 2 * TAU_PI * k / 24;
        pad.push_back(g.nodes[v].pos + Vec2(std::cos(t), std::sin(t)) * J.padRadius);
    }
    polys.push_back(pad);
    std::vector<Poly2> loops = polygonUnion(polys);
    // outer loops (CCW, positive area) vs holes
    std::vector<Poly2> outers;
    for (const Poly2& L : loops) if (signedArea(L) > 0.5) outers.push_back(L);
    J.unionLoops = (int)outers.size();

    // Rasterize a disc around the node; a cell counts if it's inside the curb
    // (any outer loop) and not inside a smaller reversed loop.
    double R = reach * 0.75;
    double cell = 0.20, cell2 = cell * cell;
    int span = (int)std::ceil(R / cell);
    double curbArea = 0, gapArea = 0;
    double gcx = 0, gcz = 0, gMinR = 1e30, gMaxR = 0; int ng = 0;
    // local triangle subset (AABB overlap with the disc) for speed
    std::vector<const Tri*> local;
    for (const Tri& t : tris)
        if (t.maxx >= J.pos.x - R && t.minx <= J.pos.x + R &&
            t.maxz >= J.pos.y - R && t.minz <= J.pos.y + R) local.push_back(&t);
    for (int iz = -span; iz <= span; ++iz)
        for (int ix = -span; ix <= span; ++ix) {
            Vec2 q(J.pos.x + ix * cell, J.pos.y + iz * cell);
            double rq = (q - J.pos).length();
            if (rq > R) continue;
            bool inCurb = false;
            for (const Poly2& L : outers) if (pointInPolygon(L, q)) { inCurb = true; break; }
            if (!inCurb) continue;
            curbArea += cell2;
            bool covered = false;
            for (const Tri* t : local)
                if (q.x >= t->minx && q.x <= t->maxx && q.y >= t->minz && q.y <= t->maxz &&
                    inTri(q, t->a, t->b, t->c)) { covered = true; break; }
            if (!covered) {
                gapArea += cell2;
                gcx += q.x; gcz += q.y; ++ng;
                gMinR = std::min(gMinR, rq); gMaxR = std::max(gMaxR, rq);
            }
        }
    (void)sidewalk;
    J.curbArea = curbArea; J.gapArea = gapArea;
    J.gapFrac = curbArea > 1e-6 ? gapArea / curbArea : 0;
    J.gapCx = ng ? gcx / ng : J.pos.x; J.gapCz = ng ? gcz / ng : J.pos.y;
    J.gapMinR = ng ? gMinR : 0; J.gapMaxR = ng ? gMaxR : 0;
    return J;
}

static std::vector<std::vector<std::pair<int,double>>> adjacency(const RoadGraph& g) {
    std::vector<std::vector<std::pair<int,double>>> adj(g.nodes.size());
    for (const RoadEdge& e : g.edges) {
        if (e.a < 0 || e.b < 0) continue;
        adj[e.a].push_back({e.b, (double)e.width});
        adj[e.b].push_back({e.a, (double)e.width});
    }
    return adj;
}

// ---- arena: synthetic junctions -------------------------------------------
static RoadNet star(int arms, double firstGap, double width) {
    // one centre node + `arms` spokes; the first two spokes are `firstGap` apart,
    // the rest spread evenly around the remaining circle.
    RoadNet net; net.width = width; net.sidewalk = 3.5;
    net.nodes.push_back(Vec2(0, 0));
    std::vector<double> bear;
    bear.push_back(0.0);
    bear.push_back(firstGap * TAU_PI / 180.0);
    double rest = 2 * TAU_PI - bear[1];
    for (int k = 2; k < arms; ++k) bear.push_back(bear[1] + rest * (k - 1) / (arms - 1));
    for (int k = 0; k < arms; ++k) {
        double a = bear[k];
        net.nodes.push_back(Vec2(std::cos(a), std::sin(a)) * 60.0);
        net.edges.push_back({0, k + 1});
    }
    return net;
}

static void runArena() {
    printf("# ARENA — synthetic junctions (flat). gapFrac = fraction of curb-enclosed ground the deck misses.\n");
    printf("%-6s %-10s %-9s %-8s %-8s %-7s\n", "arms", "minAngle", "gapFrac", "gapArea", "loops", "verdict");
    struct Case { int arms; double gap; };
    std::vector<Case> cases;
    for (int deg = 15; deg <= 175; deg += 10) cases.push_back({2, (double)deg});   // a bend
    for (double deg : {20.0, 30.0, 45.0, 60.0, 90.0}) cases.push_back({3, deg});   // T / Y
    for (double deg : {30.0, 45.0, 60.0, 90.0}) cases.push_back({4, deg});          // + / X
    for (double deg : {45.0, 60.0}) cases.push_back({5, deg});
    for (const Case& c : cases) {
        RoadNet net = star(c.arms, c.gap, 7.0);
        RenderMesh mesh = buildRoadNetMesh(net);
        RoadGraph g = navRoadGraph(net);
        auto adj = adjacency(g);
        std::vector<Tri> tris = topTris(mesh);
        // the centre node is the one nearest origin with degree == arms
        int best = -1; double bd = 1e30;
        for (int v = 0; v < (int)g.nodes.size(); ++v)
            if ((int)adj[v].size() >= 3) {
                double d = g.nodes[v].pos.length();
                if (d < bd) { bd = d; best = v; }
            }
        if (best < 0) { printf("%-6d %-10.0f  (no junction formed)\n", c.arms, c.gap); continue; }
        JunctionInfo J = measure(g, best, adj, tris, net.sidewalk);
        const char* verdict = J.gapFrac > 0.06 ? "GAP" : (J.gapFrac > 0.02 ? "marginal" : "ok");
        printf("%-6d %-10.1f %-9.3f %-8.2f %-8d %-7s\n",
               c.arms, J.minAngleDeg, J.gapFrac, J.gapArea, J.unionLoops, verdict);
    }
}

// ---- city: sweep the living_city net --------------------------------------
static void runCity() {
    RoadNet net; net.width = 7.0; net.sidewalk = 3.5; net.curb = 0.16;
    net.cornerRadius = 3.0; net.lift = 0.08;
    nlohmann::json gen = {
        {"kind","district"},{"radius",170},{"arterials",3},{"artery_width",13},
        {"street_width",7},{"block_size",82},{"curviness",0.22},{"seed",5}};
    applyGenerateRecipe(net, gen);
    RenderMesh mesh = buildRoadNetMesh(net);
    RoadGraph g = navRoadGraph(net);
    auto adj = adjacency(g);
    std::vector<Tri> tris = topTris(mesh);
    printf("# CITY — living_city junctions. %zu graph nodes, %zu top-tris.\n",
           g.nodes.size(), tris.size());
    std::vector<JunctionInfo> js;
    for (int v = 0; v < (int)g.nodes.size(); ++v) {
        if ((int)adj[v].size() < 3) continue;
        js.push_back(measure(g, v, adj, tris, net.sidewalk));
    }
    std::sort(js.begin(), js.end(), [](const JunctionInfo& a, const JunctionInfo& b){
        return a.gapArea > b.gapArea; });
    // summary
    int nGap = 0, nFrag = 0, nAcute = 0; double totGap = 0;
    for (const JunctionInfo& J : js) {
        if (J.gapFrac > 0.02) ++nGap;
        if (J.unionLoops != 1) ++nFrag;
        if (J.minAngleDeg < 35) ++nAcute;
        totGap += J.gapArea;
    }
    printf("# junctions=%zu  gappy(>2%%)=%d  union-fragmented=%d  acute(<35deg)=%d  total-gap=%.1f m^2\n",
           js.size(), nGap, nFrag, nAcute, totGap);
    printf("%-5s %-9s %-9s %-8s %-8s %-6s %-16s\n",
           "node", "minAngle", "gapFrac", "gapArea", "curbA", "loops", "pos");
    for (size_t i = 0; i < js.size() && i < 25; ++i) {
        const JunctionInfo& J = js[i];
        printf("%-5d %-9.1f %-9.3f %-8.2f %-8.1f %-6d (%.0f,%.0f) gap@(%.0f,%.0f) r=%.1f..%.1f\n",
               J.node, J.minAngleDeg, J.gapFrac, J.gapArea, J.curbArea, J.unionLoops,
               J.pos.x, J.pos.y, J.gapCx, J.gapCz, J.gapMinR, J.gapMaxR);
    }
    // correlation: gapFrac vs angle buckets
    printf("# gap rate by min-approach-angle bucket:\n");
    int loB[6] = {0,20,30,40,60,90}, hiB[6] = {20,30,40,60,90,181};
    for (int b = 0; b < 6; ++b) {
        int n = 0, gap = 0; double avg = 0;
        for (const JunctionInfo& J : js)
            if (J.minAngleDeg >= loB[b] && J.minAngleDeg < hiB[b]) {
                ++n; avg += J.gapFrac; if (J.gapFrac > 0.02) ++gap;
            }
        if (n) printf("  %3d-%3d deg: n=%-3d  gappy=%-3d  avgGapFrac=%.3f\n",
                      loB[b], hiB[b], n, gap, avg / n);
    }
    // SVG map
    FILE* f = fopen(getenv("SVG") ? getenv("SVG") : "/dev/null", "w");
    if (f) {
        double S = 2.0, off = 420;
        fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='840' height='840' style='background:#0c1016'>\n");
        for (const RoadEdge& e : g.edges)
            fprintf(f, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='#26323f' stroke-width='1'/>\n",
                    g.nodes[e.a].pos.x*S+off, g.nodes[e.a].pos.y*S+off,
                    g.nodes[e.b].pos.x*S+off, g.nodes[e.b].pos.y*S+off);
        for (const JunctionInfo& J : js) {
            const char* col = J.gapFrac > 0.06 ? "#ff2a2a" : (J.gapFrac > 0.02 ? "#ff9500" : "#2ec26e");
            double r = 2.5 + std::min(6.0, J.gapArea);
            fprintf(f, "<circle cx='%.1f' cy='%.1f' r='%.1f' fill='%s' fill-opacity='0.8'/>\n",
                    J.pos.x*S+off, J.pos.y*S+off, r, col);
        }
        fprintf(f, "</svg>\n");
        fclose(f);
    }
}

// White-box view of one region: draw the ACTUAL deck top-triangles (filled), the
// road centrelines, and mark grid cells the deck fails to cover in red — the
// ground truth for whether a reported gap is real mesh uncoverage.
static void runFocus(double cx, double cz, double half) {
    RoadNet net; net.width = 7.0; net.sidewalk = 3.5; net.curb = 0.16;
    net.cornerRadius = 3.0; net.lift = 0.08;
    nlohmann::json gen = {
        {"kind","district"},{"radius",170},{"arterials",3},{"artery_width",13},
        {"street_width",7},{"block_size",82},{"curviness",0.22},{"seed",5}};
    applyGenerateRecipe(net, gen);
    RenderMesh mesh = buildRoadNetMesh(net);
    RoadGraph g = navRoadGraph(net);
    std::vector<Tri> tris = topTris(mesh);
    const char* out = getenv("SVG") ? getenv("SVG") : "/dev/stdout";
    FILE* f = fopen(out, "w");
    double W = 900, S = W / (2 * half);
    auto X = [&](double x){ return (x - (cx - half)) * S; };
    auto Y = [&](double z){ return (z - (cz - half)) * S; };
    fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='%.0f' height='%.0f' style='background:#0c1016'>\n", W, W);
    // deck top-triangles (grey fill) — the actual paved surface
    for (const Tri& t : tris) {
        if (t.maxx < cx-half || t.minx > cx+half || t.maxz < cz-half || t.minz > cz+half) continue;
        fprintf(f, "<polygon points='%.1f,%.1f %.1f,%.1f %.1f,%.1f' fill='#3a4653' fill-opacity='0.9' stroke='#4a5a6a' stroke-width='0.3'/>\n",
                X(t.a.x),Y(t.a.y), X(t.b.x),Y(t.b.y), X(t.c.x),Y(t.c.y));
    }
    // centrelines + node ids
    for (const RoadEdge& e : g.edges)
        fprintf(f, "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke='#ffd54a' stroke-width='0.8' stroke-opacity='0.6'/>\n",
                X(g.nodes[e.a].pos.x),Y(g.nodes[e.a].pos.y), X(g.nodes[e.b].pos.x),Y(g.nodes[e.b].pos.y));
    for (int v = 0; v < (int)g.nodes.size(); ++v) {
        Vec2 p = g.nodes[v].pos;
        if (p.x < cx-half || p.x > cx+half || p.y < cz-half || p.y > cz+half) continue;
        fprintf(f, "<circle cx='%.1f' cy='%.1f' r='2' fill='#8ec5ff'/><text x='%.1f' y='%.1f' fill='#8ec5ff' font-size='10'>%d</text>\n",
                X(p.x),Y(p.y), X(p.x)+3,Y(p.y)-3, v);
    }
    // uncovered grid cells (red) — terrain would show here
    double cell = 0.25; int gaps = 0;
    for (double z = cz-half; z <= cz+half; z += cell)
        for (double x = cx-half; x <= cx+half; x += cell) {
            bool covered = false;
            for (const Tri& t : tris)
                if (x>=t.minx&&x<=t.maxx&&z>=t.minz&&z<=t.maxz&&inTri(Vec2(x,z),t.a,t.b,t.c)){covered=true;break;}
            if (!covered) continue;  // outside the paved area entirely — not a gap
        }
    // to mark true interior gaps we need "expected paved": use within-halfwidth of
    // a centreline or within a pad. Mark cells that are roadway-expected but bare.
    std::vector<std::vector<std::pair<int,double>>> adj = adjacency(g);
    for (double z = cz-half; z <= cz+half; z += cell)
        for (double x = cx-half; x <= cx+half; x += cell) {
            Vec2 q(x,z);
            // expected = within halfwidth of some edge segment, or within a pad
            bool expect = false;
            for (const RoadEdge& e : g.edges) {
                Vec2 a=g.nodes[e.a].pos,b=g.nodes[e.b].pos,ab=b-a; double L2=ab.lengthSquared();
                double t=L2<1e-9?0:std::max(0.0,std::min(1.0,dot(q-a,ab)/L2));
                if ((q-(a+ab*t)).length() < e.width*0.5) { expect=true; break; }
            }
            if (!expect)
                for (int v=0; v<(int)g.nodes.size() && !expect; ++v)
                    if ((int)adj[v].size()>=3) {
                        double wm=0; for (auto&pr:adj[v]) wm=std::max(wm,pr.second);
                        if ((q-g.nodes[v].pos).length() < wm*0.5*1.02) expect=true;
                    }
            if (!expect) continue;
            bool covered=false;
            for (const Tri& t : tris)
                if (x>=t.minx&&x<=t.maxx&&z>=t.minz&&z<=t.maxz&&inTri(q,t.a,t.b,t.c)){covered=true;break;}
            if (!covered) { fprintf(f,"<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='#ff2a2a'/>\n",
                                    X(x)-1,Y(z)-1,2.0,2.0); ++gaps; }
        }
    fprintf(f, "</svg>\n");
    fclose(f);
    fprintf(stderr, "focus (%.0f,%.0f)+-%.0f: %d expected-but-bare cells\n", cx, cz, half, gaps);
}

int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "city";
    if (mode == "arena") runArena();
    else if (mode == "focus")
        runFocus(argc>2?atof(argv[2]):103, argc>3?atof(argv[3]):63, argc>4?atof(argv[4]):22);
    else runCity();
    return 0;
}
