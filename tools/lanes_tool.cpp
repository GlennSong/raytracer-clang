// lanes_tool — the headless host for the lanes road builder (ADR-0083, ADR-0089).
//   lanes_tool build <graph.json> [--out <dir>]     build, print the summary and invariants, write <name>.glb/.svg/stats.json
//                      [--probe x y r] [--quick] [--holes] [--walls]   heights and steps around a point; --quick skips the invariant sweep and the exports; --holes lists unpaved holes; --walls lists parapets that end on drivable road
#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/lanes/road_twin.h"
#include "engine/procgen/city/roads/lanes/block_audit.h"
#include "engine/procgen/city/road_network.h"   // extractBlocks: the lot pass's face walk
#include "engine/procgen/city/polygon.h"
#include "engine/procgen/city/roads/lanes/geom2d.h"
#include <set>
#include <tuple>
#include <algorithm>
#include <cmath>
#include "engine/procgen/city/roads/lanes/lanes_export.h"
#include "engine/procgen/city/roads/lanes/level_import.h"
#include <fstream>
#include <nlohmann/json.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/stat.h>

using namespace engine;
using namespace engine::roads::lanes;

int main(int argc, char** argv) {
    if (argc < 3 || (std::strcmp(argv[1], "build") != 0 && std::strcmp(argv[1], "from-level") != 0 && std::strcmp(argv[1], "twin") != 0 && std::strcmp(argv[1], "blocks") != 0)) {
        std::fprintf(stderr, "usage: lanes_tool build <graph.json> [--out <dir>]\n"
                             "       lanes_tool from-level <level.json> --out <dir> [--diamonds N] [--no-freeway]   (writes <dir>/<name>_lanelab.json + terrain grid)\n"
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
        // blocks <graph.json> [--citysim level.json] [--out dir]: the engine's block/lot pass on the twin, audited and drawn
        using namespace engine::roads::lanes;
        std::string out = ".", cs; for (int i = 3; i + 1 < argc; ++i) { if (std::strcmp(argv[i], "--out") == 0) out = argv[i + 1]; if (std::strcmp(argv[i], "--citysim") == 0) cs = argv[i + 1]; }
        nlohmann::json citysim = nlohmann::json::object();
        if (!cs.empty()) { std::ifstream f(cs); nlohmann::json lvl; f >> lvl; citysim = lvl.value("citysim", nlohmann::json::object()); }
        bool graphBlocks = false; for (int i = 3; i < argc; ++i) if (std::strcmp(argv[i], "--graph-blocks") == 0) graphBlocks = true;
        std::unique_ptr<Result> r = build(RoadLabGraph::load(argv[2]));
        {   // where are the holes? (debug for the scene-block path)
            auto census = [](const PolySet& s, const char* name) { size_t holes = 0, big = 0; for (const Polygon2& p : s) for (const Ring& h : p.holes) { ++holes; if (std::fabs(ringArea(h)) >= 2000.0) ++big; } std::printf("  %s: %zu polygons, %zu holes (%zu >= 2000 m2)\n", name, s.size(), holes, big); };
            census(r->pavement.surface, "surface"); census(r->pavement.shoulder, "shoulder"); census(unionSets(r->pavement.surface, r->pavement.shoulder), "union");
        }
        BlockAudit a = auditBlocks(*r, citysim, !graphBlocks);
        std::printf("%s blocks: ", graphBlocks ? "graph-face" : "scene-hole");
        std::printf("%s", summary(a).c_str());
        mkdir(out.c_str(), 0755); const std::string svg = out + "/blocks.svg"; writeBlocksSvg(*r, a, svg); std::printf("wrote %s\n", svg.c_str());
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
    if (std::strcmp(argv[1], "twin") == 0) {
        // twin <graph.json>: the RoadEntity twin's degree census and the block faces the lot pass would walk
        using namespace engine::roads::lanes;
        std::unique_ptr<Result> r = build(RoadLabGraph::load(argv[2]));
        bool forLots = false; for (int i = 3; i < argc; ++i) if (std::strcmp(argv[i], "--lot-twin") == 0) forLots = true;
        engine::RoadEntity twin = roadTwin(*r, 60.0, forLots);   // default: what the block/lot pass sees today; --lot-twin: the parked variant
        std::vector<int> deg(twin.graph.nodes.size(), 0);
        for (const engine::RoadEdge& e : twin.graph.edges) { ++deg[static_cast<size_t>(e.a)]; ++deg[static_cast<size_t>(e.b)]; }
        int d1 = 0, d3 = 0, d4 = 0; for (int d : deg) { d1 += d == 1; d3 += d == 3; d4 += d >= 4; }
        std::printf("twin: %zu nodes, %zu edges; degree-1 %d, T %d, X %d\n", twin.graph.nodes.size(), twin.graph.edges.size(), d1, d3, d4);
        for (int i = 3; i + 3 < argc; ++i) if (std::strcmp(argv[i], "--nodes") == 0) {   // --nodes x y r: the twin's nodes near a point
            const engine::Vec2 c(std::atof(argv[i + 1]), std::atof(argv[i + 2])); const double rr = std::atof(argv[i + 3]);
            for (size_t n = 0; n < twin.graph.nodes.size(); ++n) { const auto& p = twin.graph.nodes[n].pos; if ((p - c).length() > rr) continue;
                std::printf("  node %zu at (%.2f, %.2f) degree %d:", n, p.x, p.y, deg[n]);
                for (const engine::RoadEdge& e : twin.graph.edges) if (e.a == static_cast<int>(n) || e.b == static_cast<int>(n)) { const auto& o = twin.graph.nodes[static_cast<size_t>(e.a == static_cast<int>(n) ? e.b : e.a)].pos; std::printf(" ->(%.1f,%.1f) c%d w%.1f%s", o.x, o.y, static_cast<int>(e.klass), e.width, e.baked ? " baked" : ""); }
                std::printf("\n"); }
        }
        engine::RoadGraph rg;   // the street subgraph exactly as growLotBuildingsOnNets builds it
        for (const engine::RoadNode& n : twin.graph.nodes) rg.nodes.push_back({n.pos});
        for (const engine::RoadEdge& e : twin.graph.edges) {
            if (e.baked || e.klass == engine::RoadClass::Freeway || e.klass == engine::RoadClass::Ramp) continue;
            rg.edges.push_back(engine::RoadEdge{e.a, e.b, e.width, engine::RoadClass::Local, 0});
        }
        std::vector<engine::Poly2> faces = engine::extractBlocks(rg);
        std::printf("street subgraph: %zu edges -> %zu faces\n", rg.edges.size(), faces.size());
        {   // spurs: a face vertex where the boundary turns straight back (a dead end poking into the block)
            int spurFaces = 0, spurs = 0;
            for (const auto& f : faces) { int n = 0; for (size_t i = 0; i < f.size(); ++i) { const auto& a = f[(i + f.size() - 1) % f.size()]; const auto& b = f[i]; const auto& c = f[(i + 1) % f.size()]; engine::Vec2 u1 = b - a, u2 = c - b; double l1 = u1.length(), l2 = u2.length(); if (l1 < 1e-6 || l2 < 1e-6) continue; if (dot(u1, u2) / (l1 * l2) < -0.985) ++n; } if (n) { ++spurFaces; spurs += n; } }
            int dead = 0; for (int d : deg) dead += d == 1;
            std::printf("spurs: %d faces carry %d dead-end spurs (twin dead ends: %d)\n", spurFaces, spurs, dead);
        }
        std::vector<double> areas; for (const auto& f : faces) { double a = 0; for (size_t i = 0; i < f.size(); ++i) { const auto& p = f[i]; const auto& q = f[(i + 1) % f.size()]; a += p.x * q.y - q.x * p.y; } areas.push_back(a / 2); }
        std::sort(areas.begin(), areas.end());
        std::printf("face areas (m2):"); for (size_t i = 0; i < areas.size() && i < 40; ++i) std::printf(" %.0f", areas[i]); std::printf("\n");
        // the lot pass's first step on each face: inset by the road margin (sidewalk). Collinear vertices are the suspect.
        double margin = 5.0; for (int i = 3; i + 1 < argc; ++i) if (std::strcmp(argv[i], "--margin") == 0) margin = std::atof(argv[i + 1]);
        size_t ok = 0, dead = 0; double worst = 1e300; size_t maxVerts = 0;
        for (const auto& f : faces) {
            maxVerts = std::max(maxVerts, f.size());
            engine::Poly2 foot = engine::inset(f, margin); double a = 0;
            for (size_t i = 0; i < foot.size(); ++i) { const auto& p = foot[i]; const auto& q = foot[(i + 1) % foot.size()]; a += p.x * q.y - q.x * p.y; }
            a = std::fabs(a / 2);
            if (foot.size() >= 3 && a > 135 && std::isfinite(a)) ++ok;
            else {   // a dead face: its size, vertex count, centroid and shortest edge (the inset's reversal trigger)
                ++dead; double fa = 0, cx = 0, cy = 0, shortest = 1e300;
                for (size_t i = 0; i < f.size(); ++i) { const auto& p = f[i]; const auto& q = f[(i + 1) % f.size()]; fa += p.x * q.y - q.x * p.y; cx += p.x; cy += p.y; shortest = std::min(shortest, std::hypot(q.x - p.x, q.y - p.y)); }
                std::printf("  dead face: %.0f m2, %zu vertices, shortest edge %.1f m, at (%.0f, %.0f)\n", std::fabs(fa / 2), f.size(), shortest, cx / f.size(), cy / f.size());
            }
            worst = std::min(worst, a);
        }
        std::printf("inset(face, %.1f): %zu ok, %zu dead (smallest area %.1f, most vertices on a face %zu)\n", margin, ok, dead, worst, maxVerts);
        // BROKEN BLOCKS: the lot pass's block foot (face inset by the margin) against the BUILT paved surface —
        // every lane's asphalt, freeways and ramps included. A foot that overlaps pavement is a block the
        // parceller will fill across a road. Reported with the roads it crosses.
        {
            const PolySet& paved = r->pavement.surface; std::vector<std::tuple<double, engine::Vec2, double, std::string>> bad;
            // the lot pass's second step, reproduced: densify the foot every 8 m and push every vertex clear of the
            // sampled ribbons (twin edge width/2 + sidewalk clearance), four rounds — city_lots.cpp pushPolyClearOfRoads
            const double roadClear = margin + 0.6;
            auto pushClear = [&](engine::Poly2 poly) {
                engine::Poly2 dense;
                for (size_t i = 0; i < poly.size(); ++i) { const auto& a = poly[i]; const auto& b = poly[(i + 1) % poly.size()]; dense.push_back(a); const int div = static_cast<int>((b - a).length() / 8.0); for (int k = 1; k <= div; ++k) dense.push_back(a + (b - a) * (static_cast<double>(k) / (div + 1))); }
                for (auto& v : dense) for (int guard = 0; guard < 4; ++guard) {
                    double worstNeed = 0; engine::Vec2 away;
                    for (const engine::RoadEdge& e : twin.graph.edges) {
                        const auto& a = twin.graph.nodes[static_cast<size_t>(e.a)].pos; const auto& b = twin.graph.nodes[static_cast<size_t>(e.b)].pos;
                        engine::Vec2 ab = b - a; double l2 = ab.lengthSquared(); double t = l2 > 1e-12 ? std::max(0.0, std::min(1.0, dot(v - a, ab) / l2)) : 0.0;
                        engine::Vec2 q = a + ab * t; double d = (v - q).length(); double need = e.width * 0.5 + roadClear - d;
                        if (need > worstNeed) { worstNeed = need; away = d > 1e-6 ? (v - q) * (1.0 / d) : engine::Vec2(0, 1); }
                    }
                    if (worstNeed <= 0.01) break; v = v + away * worstNeed;
                }
                return dense;
            };
            for (const auto& f : faces) {
                engine::Poly2 foot = engine::inset(f, margin); if (foot.size() < 3) continue;
                foot = pushClear(foot); if (foot.size() < 3 || std::fabs(engine::area(foot)) < 135.0) continue;
                Ring ring(foot.begin(), foot.end()); PolySet fs = fromRing(ring); double a = setArea(intersectSets(fs, paved));
                if (a <= 1.0) continue;
                std::set<std::string> roads;
                for (size_t li = 0; li < r->lanes.lanes.size(); ++li) {
                    const Lane& l = r->lanes.lanes[li]; if (l.parent < 0 || r->pavement.footprints[li].empty()) continue;
                    if (setArea(intersectSets(fs, r->pavement.footprints[li])) > 0.5) roads.insert(r->graph.edges[static_cast<size_t>(l.parent)].id + "(" + r->graph.edges[static_cast<size_t>(l.parent)].cls + ")");
                }
                std::string rs; for (const auto& x : roads) rs += (rs.empty() ? "" : " ") + x;
                engine::Vec2 c; for (const auto& q : foot) c += q; c = c / static_cast<double>(foot.size());
                bad.emplace_back(a, c, std::fabs(engine::area(foot)), rs);
            }
            std::sort(bad.begin(), bad.end(), [](const auto& x, const auto& y) { return std::get<0>(x) > std::get<0>(y); });
            double tot = 0; for (const auto& b : bad) tot += std::get<0>(b);
            std::printf("BROKEN BLOCKS: %zu of %zu block feet overlap built pavement by > 1 m2 (total %.0f m2)\n", bad.size(), faces.size(), tot);
            for (size_t i = 0; i < bad.size() && i < 12; ++i) std::printf("  %7.0f m2 of a %6.0f m2 block at (%.0f, %.0f) across: %s\n", std::get<0>(bad[i]), std::get<2>(bad[i]), std::get<1>(bad[i]).x, std::get<1>(bad[i]).y, std::get<3>(bad[i]).c_str());
        }
        // --at x y: the face that contains the point, and every twin edge with a node inside that face
        // (a street the face SHOULD have been split by), plus an SVG of the twin around it.
        for (int i = 3; i + 2 < argc; ++i) if (std::strcmp(argv[i], "--at") == 0) {
            const engine::Vec2 at(std::atof(argv[i + 1]), std::atof(argv[i + 2]));
            auto inside = [](const engine::Poly2& f, const engine::Vec2& q) { bool in = false; for (size_t a = 0, b = f.size() - 1; a < f.size(); b = a++) { if ((f[a].y > q.y) != (f[b].y > q.y) && q.x < (f[b].x - f[a].x) * (q.y - f[a].y) / (f[b].y - f[a].y) + f[a].x) in = !in; } return in; };
            int hit = -1; for (size_t f = 0; f < faces.size(); ++f) if (inside(faces[f], at)) { hit = static_cast<int>(f); break; }
            if (hit < 0) { std::printf("--at (%.0f, %.0f): no face contains the point\n", at.x, at.y); continue; }
            const engine::Poly2& F = faces[static_cast<size_t>(hit)]; double ar = 0; for (size_t k = 0; k < F.size(); ++k) { const auto& a = F[k]; const auto& b = F[(k + 1) % F.size()]; ar += a.x * b.y - b.x * a.y; }
            std::printf("--at (%.0f, %.0f): face %d, %zu vertices, %.0f m2\n", at.x, at.y, hit, F.size(), std::fabs(ar / 2));
            std::map<std::string, int> interior;   // class:baked -> edges with a node strictly inside the face
            int nIn = 0;
            for (const engine::RoadEdge& e : twin.graph.edges) {
                const engine::Vec2 m = (twin.graph.nodes[static_cast<size_t>(e.a)].pos + twin.graph.nodes[static_cast<size_t>(e.b)].pos) * 0.5;
                if (!inside(F, m)) continue; ++nIn;
                std::string key = std::to_string(static_cast<int>(e.klass)) + (e.baked ? ":baked" : ":live"); ++interior[key];
            }
            std::printf("  twin edges whose midpoint lies inside this face: %d\n", nIn);
            // the nearest twin sub-edge of each kind to the point itself: is a street boundary right here, or only a ramp?
            struct Near { double d = 1e300; int klass = -1; bool baked = false; } nearLive, nearBaked, nearRamp;
            for (const engine::RoadEdge& e : twin.graph.edges) {
                const auto& a = twin.graph.nodes[static_cast<size_t>(e.a)].pos; const auto& b = twin.graph.nodes[static_cast<size_t>(e.b)].pos;
                engine::Vec2 ab = b - a; double l2 = ab.lengthSquared(); double t = l2 > 1e-9 ? std::max(0.0, std::min(1.0, dot(at - a, ab) / l2)) : 0.0;
                double d = (at - (a + ab * t)).length();
                Near& n = (e.klass == engine::RoadClass::Ramp || e.klass == engine::RoadClass::Freeway) ? nearRamp : (e.baked ? nearBaked : nearLive);
                if (d < n.d) { n.d = d; n.klass = static_cast<int>(e.klass); n.baked = e.baked; }
            }
            std::printf("  nearest live street sub-edge %.1f m (class %d); nearest baked %.1f m; nearest ramp/freeway %.1f m\n", nearLive.d, nearLive.klass, nearBaked.d, nearRamp.d);
            for (const auto& kv : interior) std::printf("    class %s -> %d\n", kv.first.c_str(), kv.second);
            // the SVG: 250 m around the point
            std::ofstream svg(std::string(argv[2]) + ".twin_at.svg"); const double R = 250;
            svg << "<svg xmlns='http://www.w3.org/2000/svg' viewBox='" << at.x - R << " " << at.y - R << " " << 2 * R << " " << 2 * R << "'>\n<rect x='" << at.x - R << "' y='" << at.y - R << "' width='" << 2 * R << "' height='" << 2 * R << "' fill='#f4f4ee'/>\n";
            svg << "<polygon fill='#cfe3ff' fill-opacity='0.6' stroke='#2255aa' stroke-width='1' points='"; for (const auto& q : F) svg << q.x << "," << q.y << " "; svg << "'/>\n";
            std::vector<int> deg(twin.graph.nodes.size(), 0); for (const engine::RoadEdge& e : twin.graph.edges) { ++deg[static_cast<size_t>(e.a)]; ++deg[static_cast<size_t>(e.b)]; }
            for (const engine::RoadEdge& e : twin.graph.edges) {
                const auto& a = twin.graph.nodes[static_cast<size_t>(e.a)].pos; const auto& b = twin.graph.nodes[static_cast<size_t>(e.b)].pos;
                const char* col = e.klass == engine::RoadClass::Freeway ? "#c33" : e.klass == engine::RoadClass::Ramp ? "#e80" : e.baked ? "#999" : "#223";
                svg << "<line x1='" << a.x << "' y1='" << a.y << "' x2='" << b.x << "' y2='" << b.y << "' stroke='" << col << "' stroke-width='" << (e.baked ? 1.0 : 1.6) << "'/>\n";
            }
            for (size_t n = 0; n < deg.size(); ++n) if (deg[n] != 2) { const auto& q = twin.graph.nodes[n].pos; svg << "<circle cx='" << q.x << "' cy='" << q.y << "' r='2.5' fill='" << (deg[n] == 1 ? "#c00" : deg[n] == 3 ? "#0a0" : "#00c") << "'/>\n"; }
            svg << "<circle cx='" << at.x << "' cy='" << at.y << "' r='6' fill='none' stroke='#f0f' stroke-width='2'/>\n</svg>\n";
            std::printf("  wrote %s.twin_at.svg\n", argv[2]);
        }
        return 0;
    }
    if (std::strcmp(argv[1], "from-level") == 0) {
        if (out.empty()) { std::fprintf(stderr, "from-level needs --out <dir>\n"); return 2; }
        ImportOptions o; for (int i = 3; i < argc; ++i) { if (std::strcmp(argv[i], "--no-freeway") == 0) o.freewayLoop = false; if (std::strcmp(argv[i], "--diamonds") == 0 && i + 1 < argc) o.diamonds = std::atoi(argv[++i]);
            if (std::strcmp(argv[i], "--ramp-width") == 0 && i + 1 < argc) o.rampLaneW = std::atof(argv[++i]);
            if (std::strcmp(argv[i], "--ramp-shoulder") == 0 && i + 1 < argc) o.rampShoulder = std::atof(argv[++i]); }
        mkdir(out.c_str(), 0755); ImportReport rep;
        try {
            nlohmann::json graph = graphFromLevel(argv[2], out, o, rep); std::string path = out + "/" + graph["name"].get<std::string>() + ".json";
            std::ofstream f(path); f << graph.dump(1); std::printf("%s\nwrote %s (%zu edges)\n", rep.notes.c_str(), path.c_str(), graph["edges"].size());
        } catch (const std::exception& e) { std::fprintf(stderr, "lanes from-level: %s\n", e.what()); return 1; }
        return 0;
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
                                what, std::fabs(ringArea(pg.outer)), c.x, c.y, pg.outer.size(), x1 - x0, y1 - y0);
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
            const double a = std::fabs(ringArea(h)); if (a < 1.0 || a >= 2000.0) continue;   // >= 2000 m2 is a city block
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
        mkdir(out.c_str(), 0755); std::string base = out + "/" + r.graph.name; std::string err;
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
        if (!writeGlb(meshes, base + ".glb", &err)) std::fprintf(stderr, "lanes: %s\n", err.c_str());
        writePlanSvg(r, base + ".svg"); writeStatsJson(r, out + "/stats.json");
        std::printf("wrote %s.glb (%zu meshes, %zu triangles), %s.svg, %s/stats.json\n", base.c_str(), meshes.size(), tris, base.c_str(), out.c_str());
    }
    return ok ? 0 : 1;
}
