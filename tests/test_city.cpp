#include <limits>
#include <map>
#include <cstdio>
#include <string>
#include "test_framework.h"

#include "../src/engine/procgen/city/polygon.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/parcel.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/road_mesh.h"
#include "../src/engine/procgen/city/road_net.h"   // buildRoadNetLattice (the ONE mesher)
#include "../src/engine/procgen/city/city_lots.h"   // LotBuilding, appendLotMassBox
#include "../src/engine/procgen/city/site_plan.h"   // offsetPolygonEdges (corner closure test)
#include "../src/engine/mesh_builder.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <set>

using namespace engine;  // namespace migration (ADR-0015)

namespace {
bool triContainsXZ(const RenderMesh& m, std::size_t i, double px, double pz) {
    const auto& A = m.vertices[m.indices[i]].position;
    const auto& B = m.vertices[m.indices[i+1]].position;
    const auto& C = m.vertices[m.indices[i+2]].position;
    double area2 = (B.x-A.x)*(C.z-A.z) - (C.x-A.x)*(B.z-A.z);
    if (std::fabs(area2) < 1e-7) return false;     // a degenerate tri contains no area
    double d1 = (px-B.x)*(A.z-B.z) - (A.x-B.x)*(pz-B.z);
    double d2 = (px-C.x)*(B.z-C.z) - (B.x-C.x)*(pz-C.z);
    double d3 = (px-A.x)*(C.z-A.z) - (C.x-A.x)*(pz-A.z);
    bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(neg && pos);                       // same side of all edges
}
bool meshCoversXZ(const RenderMesh& m, double px, double pz) {
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3)
        if (triContainsXZ(m, i, px, pz)) return true;
    return false;
}
double meshAreaXZ(const RenderMesh& m) {
    double area = 0;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const auto& A = m.vertices[m.indices[i]].position;
        const auto& B = m.vertices[m.indices[i+1]].position;
        const auto& C = m.vertices[m.indices[i+2]].position;
        area += 0.5 * std::fabs((B.x-A.x)*(C.z-A.z) - (C.x-A.x)*(B.z-A.z));
    }
    return area;
}
double distToSeg(double px, double pz, Vec2 a, Vec2 b) {
    double dx = b.x-a.x, dz = b.y-a.y, l2 = dx*dx + dz*dz;
    double t = l2 < 1e-12 ? 0 : std::max(0.0, std::min(1.0, ((px-a.x)*dx + (pz-a.y)*dz)/l2));
    double cx = a.x + dx*t, cz = a.y + dz*t;
    return std::sqrt((px-cx)*(px-cx) + (pz-cz)*(pz-cz));
}
}  // namespace

TEST_CASE(stroke_ribbon_is_exact_and_never_strays) {
    // ADR-0048 back-to-basics: stroking a centerline into a flat ribbon.
    // (1) A straight constant-width line is an exact rectangle: area == length*width.
    std::vector<Vec2> line = { {0,0}, {100,0} };
    RenderMesh sm = strokeRibbon(line, {4.0}, 0.0, Vec3(0.1,0.1,0.1), false);
    CHECK(sm.indices.size() % 3 == 0);
    CHECK_APPROX(meshAreaXZ(sm), 100.0 * 8.0, 1e-6);     // 2*halfWidth * length

    // (2) A hairpin tighter than the width must not fold: every emitted vertex stays
    // within the half-width of the centerline (a stray/folded triangle would not).
    std::vector<Vec2> hair = { {-60,0}, {0,0}, {-60,8} };   // ~170-degree reversal
    double w = 6.0;
    RenderMesh hm = strokeRibbon(hair, {w}, 0.0, Vec3(0.1,0.1,0.1), false);
    CHECK(!hm.vertices.empty());
    double worst = 0;
    for (const Vertex& v : hm.vertices) {
        double d = std::min(distToSeg(v.position.x, v.position.z, hair[0], hair[1]),
                            distToSeg(v.position.x, v.position.z, hair[1], hair[2]));
        worst = std::max(worst, d);
    }
    CHECK(worst <= w + 1e-3);                             // nothing strays outside the ribbon
}

TEST_CASE(stroke_ribbon_closed_ring_has_a_hole) {
    // A stroked circle is an annulus, not a disc: its centre is uncovered and its
    // area matches a ring (~ circumference * width), far below the disc area.
    std::vector<Vec2> circ;
    const int N = 48; const double R = 30.0, w = 5.0;     // w = half-width
    for (int i = 0; i < N; ++i) {
        double t = 2.0 * 3.14159265358979 * i / N;
        circ.push_back(Vec2(R * std::cos(t), R * std::sin(t)));
    }
    RenderMesh m = strokeRibbon(circ, {w}, 0.0, Vec3(0.1,0.1,0.1), /*closed=*/true);
    CHECK(!m.vertices.empty());
    CHECK(!meshCoversXZ(m, 0.0, 0.0));                    // the hole: centre uncovered
    CHECK(meshCoversXZ(m, R, 0.0));                       // the ring itself is covered
    double ring = 2.0 * 3.14159265 * R * (2.0 * w);       // ~ circumference * full width
    CHECK(meshAreaXZ(m) < ring * 1.4);                    // a ring, nowhere near a disc
    CHECK(meshAreaXZ(m) > ring * 0.7);
}

TEST_CASE(stroke_ribbon_variable_width_tapers) {
    // Per-point widths give a trapezoid: area == length * average full width.
    std::vector<Vec2> line = { {0,0}, {100,0} };
    RenderMesh m = strokeRibbon(line, {6.0, 1.0}, 0.0, Vec3(0.1,0.1,0.1), false);
    CHECK_APPROX(meshAreaXZ(m), 100.0 * (12.0 + 2.0) / 2.0, 1e-6);   // (2*6 + 2*1)/2 * len
}

TEST_CASE(stroke_ribbon_every_triangle_faces_up) {
    // Whatever the curve, the flat ribbon is single-sided up — no flipped winding.
    std::vector<Vec2> wig = { {0,0}, {20,10}, {40,-10}, {60,8}, {62,-2}, {30,-20} };
    RenderMesh m = strokeRibbon(wig, {4.0}, 0.0, Vec3(0.1,0.1,0.1), false);
    CHECK(!m.vertices.empty());
    for (const Vertex& v : m.vertices) CHECK(v.normal.y > 0.5);
}

TEST_CASE(stroke_ribbon_hairpin_has_a_round_cap) {
    // A ~180-degree apex gets a semicircular turning cap from the round join: a point
    // just past the apex vertex is covered by the cap, but nothing beyond the radius.
    std::vector<Vec2> hair = { {-60,0}, {0,0}, {-60,6} };   // apex at the origin vertex
    const double w = 6.0;
    RenderMesh m = strokeRibbon(hair, {w}, 0.0, Vec3(0.1,0.1,0.1), false);
    CHECK(meshCoversXZ(m, 5.0, 0.0));      // on the cap, past where the legs end (x=0)
    CHECK(!meshCoversXZ(m, 8.0, 0.0));     // but the cap stops at the half-width radius
}

TEST_CASE(stroke_network_intersection_is_a_clean_junction) {
    // Two crossing roads (an X): planarize splits them at the crossing into a 4-way
    // node, and the mesh fills that intersection (centre covered) without any geometry
    // straying past the arms — a proper junction, not two ribbons overlapping loose.
    RoadGraph g;
    int a = g.addNode(Vec2(-40,0)), b = g.addNode(Vec2(40,0));
    int c = g.addNode(Vec2(0,-40)), d = g.addNode(Vec2(0,40));
    g.addEdge(a, b, 10); g.addEdge(c, d, 10);
    RoadGraph pg = planarize(g);
    CHECK(pg.nodes.size() == 5u);                       // the crossing inserted a node
    RenderMesh m = buildRoadNetLattice(pg, nullptr, nullptr, 0.0, 0.15, false);
    CHECK(meshCoversXZ(m, 2.0, 2.0));                   // the intersection is filled
    // Nothing strays past an arm end (length 40) plus its half-width (5) — the far
    // ribbon corner sits at sqrt(40^2+5^2) ~ 40.3, so bound by the arm reach + width.
    double worst = 0;
    for (const Vertex& v : m.vertices)
        worst = std::max(worst, std::sqrt(double(v.position.x*v.position.x +
                                                 v.position.z*v.position.z)));
    CHECK(worst <= 40.0 + 5.0 + 1e-3);
}

TEST_CASE(radial_roads_make_rings_and_spokes) {
    // ADR-0044: concentric rings sampled from true ARCS (bounded chord error) +
    // avenues radiating from a central roundabout. No centre node, so the avenues
    // meet the inner ring instead of converging to a spike; rings are smooth.
    RadialParams rp;
    rp.extent = 280; rp.ringSpacing = 70; rp.spokes = 8; rp.seed = 5;
    RoadGraph g = radialRoads(rp);
    CHECK(g.edges.size() > 0);

    const int rings = 4;                          // round(280 / 70)
    // Arc sampling gives many nodes per ring (a smooth circle), more than a coarse
    // polygon would — comfortably above spokes per ring.
    CHECK(g.nodes.size() > static_cast<std::size_t>(rings * rp.spokes * 3));

    Real minR = 1e9, maxR = 0;
    for (const RoadNode& n : g.nodes) {
        minR = std::min(minR, n.pos.length());
        maxR = std::max(maxR, n.pos.length());
    }
    CHECK(minR > rp.ringSpacing * 0.7);           // nothing inside the roundabout
    CHECK(maxR <= rp.extent + rp.ringSpacing * 0.2);

    // Sampled ring nodes sit ON their circle (true arc, not a chord midpoint that
    // cuts the corner): every node is within chord-error slack of some ring radius.
    int offCircle = 0;
    for (const RoadNode& n : g.nodes) {
        Real r = n.pos.length();
        bool onRing = false;
        for (int k = 1; k <= rings; ++k)
            if (std::abs(r - k * rp.ringSpacing) < 1.0) onRing = true;
        if (!onRing) offCircle++;
    }
    CHECK(offCircle == 0);

    std::vector<Poly2> blocks = extractBlocks(planarize(g));
    CHECK(blocks.size() > 8);
}

TEST_CASE(tensor_roads_blend_radial_core_into_grid_rim) {
    // ADR-0044: the tensor field is radial in the core and a grid at the rim.
    // We verify both regimes from one generator: near the centre the two road
    // families run radial/tangential, far out they align to the grid axes.
    TensorRoadParams tp;
    tp.extent = 260; tp.spacing = 70; tp.step = 8;
    tp.radialDecay = 110; tp.gridAngle = 0; tp.seed = 5;
    RoadGraph g = tensorRoads(tp);
    CHECK(g.edges.size() > 0);
    CHECK(g.nodes.size() > 0);

    // Everything stays inside the requested region.
    for (const RoadNode& n : g.nodes) {
        CHECK(std::abs(n.pos.x) <= tp.extent + 1.0);
        CHECK(std::abs(n.pos.y) <= tp.extent + 1.0);
    }

    // Classify each edge by where it sits: in the core, the field is radial, so
    // an edge should run either ~radially or ~tangentially. Count how many do.
    int coreEdges = 0, coreAligned = 0, rimEdges = 0, rimAxisAligned = 0;
    for (const RoadEdge& e : g.edges) {
        Vec2 a = g.nodes[e.a].pos, b = g.nodes[e.b].pos;
        Vec2 mid = (a + b) * 0.5;
        Vec2 dir = normalize(b - a);
        if (dir.lengthSquared() < 1e-9) continue;
        Real r = mid.length();
        if (r > 1e-3 && r < tp.radialDecay * 0.6) {
            ++coreEdges;
            Vec2 radial = normalize(mid);
            Real along = std::abs(dot(dir, radial));     // 1 = radial, 0 = tangential
            if (along > 0.85 || along < 0.15) ++coreAligned;
        } else if (r > tp.radialDecay * 1.4) {
            ++rimEdges;
            Real ax = std::max(std::abs(dir.x), std::abs(dir.y));
            if (ax > 0.92) ++rimAxisAligned;             // aligned to the grid axes
        }
    }
    CHECK(coreEdges > 0);
    CHECK(rimEdges > 0);
    // The blend is approximate (the field rotates continuously), so we only ask
    // that the dominant character holds for most edges in each regime.
    CHECK(coreAligned * 2 > coreEdges);                  // >50% radial/tangential
    CHECK(rimAxisAligned * 2 > rimEdges);                // >50% grid-aligned

    // The streamlines actually enclose blocks once planarised.
    std::vector<Poly2> blocks = extractBlocks(planarize(g));
    CHECK(blocks.size() > 4);
}

TEST_CASE(tensor_roads_are_deterministic) {
    TensorRoadParams tp; tp.extent = 200; tp.spacing = 60; tp.seed = 11;
    RoadGraph a = tensorRoads(tp), b = tensorRoads(tp);
    CHECK(a.nodes.size() == b.nodes.size());
    CHECK(a.edges.size() == b.edges.size());
}

namespace {
// Worst grade along an edge over the given heightfield (matches pruneSteepEdges).
Real edgeGrade(const RoadGraph& g, const RoadEdge& e, const HeightField& h) {
    Vec2 a = g.nodes[e.a].pos, b = g.nodes[e.b].pos;
    Real len = (b - a).length();
    return len < 1e-4 ? 0 : std::fabs(h(b.x, b.y) - h(a.x, a.y)) / len;
}
// Number of connected components among the nodes that appear in some edge.
int edgeComponents(const RoadGraph& g) {
    std::vector<int> p(g.nodes.size());
    for (std::size_t i = 0; i < p.size(); ++i) p[i] = static_cast<int>(i);
    std::function<int(int)> find = [&](int x) { return p[x] == x ? x : p[x] = find(p[x]); };
    std::vector<char> used(g.nodes.size(), 0);
    for (const RoadEdge& e : g.edges) { p[find(e.a)] = find(e.b); used[e.a] = used[e.b] = 1; }
    std::set<int> roots;
    for (std::size_t i = 0; i < g.nodes.size(); ++i) if (used[i]) roots.insert(find(static_cast<int>(i)));
    return static_cast<int>(roots.size());
}
}  // namespace

TEST_CASE(prune_steep_edges_keeps_the_network_connected) {
    // ADR-0046: on a hillside tilted 0.4 in x, a 3x3 lattice's x-edges climb at
    // grade 0.4 (>0.12) while its z-edges are flat. The prune must NOT fragment the
    // graph: it keeps every gentle (z) edge and adds back only the minimal steep (x)
    // bridges needed to join the three vertical lines — so the result is still one
    // connected network, with the steep streets thinned to the unavoidable few.
    HeightField slope = [](double x, double) { return 0.4 * x; };
    RoadGraph g;
    int id[3][3];
    for (int j = 0; j < 3; ++j)
        for (int i = 0; i < 3; ++i) id[i][j] = g.addNode(Vec2(i * 100.0, j * 100.0));
    for (int j = 0; j < 3; ++j)
        for (int i = 0; i < 3; ++i) {
            if (i < 2) g.addEdge(id[i][j], id[i + 1][j], 8, RoadClass::Local);  // steep (x)
            if (j < 2) g.addEdge(id[i][j], id[i][j + 1], 8, RoadClass::Local);  // flat  (z)
        }
    CHECK(g.edges.size() == 12u);
    CHECK(edgeComponents(g) == 1);

    RoadGraph pruned = pruneSteepEdges(g, slope, 0.12);
    CHECK(edgeComponents(pruned) == 1);                 // still one coherent network
    int gentle = 0, steep = 0;
    for (const RoadEdge& e : pruned.edges)
        (edgeGrade(pruned, e, slope) <= 0.12 ? gentle : steep)++;
    CHECK(gentle == 6);                                 // every walkable street kept
    CHECK(steep == 2);                                  // only the 2 bridges to link 3 lines
}

TEST_CASE(prune_steep_edges_keeps_arterials_and_never_orphans) {
    // The skeleton stays whole: an Arterial is kept even when steep, and a node's
    // last edge is kept rather than orphan it. A lone steep local spur off a node
    // that has no other edge must survive.
    HeightField slope = [](double x, double) { return 0.5 * x; };
    RoadGraph g;
    int a = g.addNode(Vec2(0, 0)), b = g.addNode(Vec2(100, 0)), c = g.addNode(Vec2(200, 0));
    g.addEdge(a, b, 16, RoadClass::Arterial);           // steep but structural
    g.addEdge(b, c, 8, RoadClass::Local);               // steep, but c has no other edge
    RoadGraph pruned = pruneSteepEdges(g, slope, 0.12);
    CHECK(pruned.edges.size() == 2u);                   // both kept (skeleton + no orphan)
}

TEST_CASE(connect_components_heals_a_split_network) {
    // ADR-0046: two separate street fragments must be stitched into one connected
    // graph by the shortest bridge between them (the coherence backstop after a
    // contour-coupled trace leaves non-crossing streamlines).
    RoadGraph g;
    int a = g.addNode(Vec2(0, 0)),  b = g.addNode(Vec2(50, 0));    // fragment 1
    int c = g.addNode(Vec2(0, 200)), d = g.addNode(Vec2(50, 200)); // fragment 2, far off
    int e = g.addNode(Vec2(500, 0)), f = g.addNode(Vec2(500, 50)); // fragment 3
    g.addEdge(a, b, 8); g.addEdge(c, d, 8); g.addEdge(e, f, 8);
    CHECK(edgeComponents(g) == 3);

    RoadGraph joined = connectComponents(g);
    CHECK(edgeComponents(joined) == 1);                 // one coherent network
    CHECK(joined.edges.size() == 5u);                   // 3 original + 2 connectors
}

TEST_CASE(tensor_field_follows_contours_on_a_slope) {
    // ADR-0046: with terrain coupling the avenues bend to follow the hillside, so
    // the major (Collector) family runs far gentler than the terrain-blind layout
    // that marches the same field straight across the slope.
    HeightField slope = [](double x, double) { return 0.4 * x; };
    auto meanCollectorGrade = [&](const RoadGraph& g) {
        Real sum = 0; int n = 0;
        for (const RoadEdge& e : g.edges)
            if (e.klass == RoadClass::Collector) { sum += edgeGrade(g, e, slope); ++n; }
        return n ? sum / n : Real(0);
    };

    TensorRoadParams blind; blind.extent = 200; blind.spacing = 60; blind.seed = 7;
    RoadGraph gBlind = tensorRoads(blind);

    TensorRoadParams aware = blind;
    aware.terrain = &slope; aware.slopeAlign = 2.0; aware.maxGrade = 0.12;
    RoadGraph gAware = tensorRoads(aware);

    Real blindGrade = meanCollectorGrade(gBlind);
    Real awareGrade = meanCollectorGrade(gAware);
    CHECK(awareGrade < blindGrade);          // avenues bent toward the contours
    CHECK(awareGrade < 0.12);                // and they actually read as walkable
}

namespace {
Poly2 square(Real s) {
    return {{0, 0}, {s, 0}, {s, s}, {0, s}};   // CCW
}
bool hasPart(const BuildingMesh& bm, PartId id) {
    for (const RenderMesh& p : bm.parts)
        if (p.materialIndex == static_cast<int>(id)) return true;
    return false;
}
}  // namespace

// --- polygon ----------------------------------------------------------------

TEST_CASE(polygon_area_centroid_and_winding) {
    Poly2 sq = square(10);
    CHECK_APPROX(area(sq), 100.0, 1e-9);
    CHECK(isCCW(sq));
    Vec2 c = centroid(sq);
    CHECK_APPROX(c.x, 5.0, 1e-9);
    CHECK_APPROX(c.y, 5.0, 1e-9);
    // A CW ring has negative signed area; ensureCCW flips it.
    Poly2 cw = {{0, 0}, {0, 10}, {10, 10}, {10, 0}};
    CHECK(signedArea(cw) < 0);
    ensureCCW(cw);
    CHECK(isCCW(cw));
}

TEST_CASE(polygon_point_in_polygon) {
    Poly2 sq = square(10);
    CHECK(pointInPolygon(sq, {5, 5}));
    CHECK(!pointInPolygon(sq, {15, 5}));
    CHECK(!pointInPolygon(sq, {-1, 5}));
}

TEST_CASE(polygon_convex_hull) {
    Poly2 pts = {{0, 0}, {10, 0}, {10, 10}, {0, 10}, {5, 5}, {3, 7}};  // +2 interior
    Poly2 hull = convexHull(pts);
    CHECK(hull.size() == 4);                 // interior points dropped
    CHECK_APPROX(area(hull), 100.0, 1e-9);
}

TEST_CASE(polygon_obb_of_axis_aligned_rect) {
    Poly2 rect = {{0, 0}, {20, 0}, {20, 6}, {0, 6}};
    OBB2 obb = orientedBoundingBox(rect);
    CHECK_APPROX(obb.center.x, 10.0, 1e-6);
    CHECK_APPROX(obb.center.y, 3.0, 1e-6);
    // Long axis half-extent ~10, short ~3.
    Real lo = obb.half[obb.longAxis()], sh = obb.half[1 - obb.longAxis()];
    CHECK_APPROX(lo, 10.0, 1e-6);
    CHECK_APPROX(sh, 3.0, 1e-6);
}

TEST_CASE(polygon_inset_shrinks_area) {
    Poly2 sq = square(20);
    Poly2 in = inset(sq, 2.0);
    CHECK(in.size() == 4);
    // Inset of a 20x20 square by 2 -> 16x16.
    CHECK_APPROX(area(in), 256.0, 1e-6);
    // Over-inset past the medial axis collapses to empty.
    CHECK(inset(sq, 11.0).empty());
}

TEST_CASE(polygon_split_by_line) {
    Poly2 sq = square(10);
    Poly2 left, right;
    splitByLine(sq, {5, 5}, {0, 1}, left, right);   // vertical line x=5
    CHECK(left.size() >= 3);
    CHECK(right.size() >= 3);
    CHECK_APPROX(area(left) + area(right), 100.0, 1e-6);
    CHECK_APPROX(area(left), 50.0, 1e-6);
}

// --- shape grammar (Phase 0) ------------------------------------------------

TEST_CASE(grammar_grows_a_multipart_building) {
    BuildingParams p; p.floors = 5; p.seed = 3;
    Scope s = scopeFromFootprint(square(18), 0.0, 20.0);
    BuildingMesh bm = growBuilding(s, p);
    CHECK(bm.parts.size() >= 3);                 // wall + glass + roof at least
    CHECK(hasPart(bm, PartId::Wall));
    CHECK(hasPart(bm, PartId::Glass));
    // 5 floors * 3.2 + 4.5 ground + 1.1 parapet = 21.6 m.
    CHECK_APPROX(bm.height, 5 * 3.2 + 4.5 + 1.1, 1e-6);
    CHECK(!bm.proxy.vertices.empty());           // coarse LOD proxy emitted
    CHECK(!bm.attaches.empty());
}

// The DISTANT city is made of these mass boxes (past detailDistance the facades
// are dropped and a render cell becomes a handful of them), so their normals
// decide how the whole skyline shades. This shipped INVERTED: the side normal
// used the LEFT normal (-d.z, 0, d.x) of a CCW box, pointing into the box, so
// every far building was lit inside-out. It hid from the `facing` debug view
// because emitQuad winds geometry to agree with the normal it is handed — the
// near wall gets culled and you see the far wall with its flipped normal aimed
// back at the camera, which reads as front-facing. Only a geometric test catches
// it: on a closed convex box every face must point away from the centre.
TEST_CASE(city_lot_mass_box_faces_outward) {
    for (Real yaw : {Real(0), Real(0.7), Real(2.9), Real(-1.3)}) {
        LotBuilding lot;
        lot.site = Vec2(12, -30);
        lot.width = 18; lot.depth = 10; lot.height = 24;
        lot.yaw = yaw; lot.baseY = 3.0;
        RenderMesh m;
        appendLotMassBox(m, lot, Vec3(0.6, 0.5, 0.4), Vec3(0.2, 0.2, 0.22));
        CHECK(!m.vertices.empty());

        const Vec3 c(lot.site.x, lot.baseY + lot.height * 0.5, lot.site.y);
        int inward = 0, shadeInward = 0, tris = 0;
        for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
            const Vec3& a = m.vertices[m.indices[i]].position;
            const Vec3& b = m.vertices[m.indices[i + 1]].position;
            const Vec3& d = m.vertices[m.indices[i + 2]].position;
            // The engine winds front faces CLOCKWISE: outward = cross(c-a, b-a).
            Vec3 gn = cross(d - a, b - a);
            if (gn.length() < 1e-6f) continue;
            gn = normalize(gn);
            ++tris;
            const Vec3 mid = (a + b + d) * (1.0f / 3.0f);
            if (dot(gn, mid - c) < 0.0) ++inward;                     // winding
            if (dot(m.vertices[m.indices[i]].normal, mid - c) < 0.0)  // shading
                ++shadeInward;
        }
        CHECK(tris >= 10);        // 4 walls + a cap, two triangles each
        CHECK(inward == 0);
        CHECK(shadeInward == 0);
    }
}

// The HLOD proxy is what the DISTANT city is made of, so a wrong-facing proxy
// shows up as inside-out buildings on the horizon. It is the one place a
// building's mass is built from orientedBoundingBox axes (axis[1] = perp(axis[0]))
// rather than by extruding the plan — a basis whose handedness is easy to get
// wrong — so pin it: a proxy is a closed convex box, therefore EVERY triangle's
// geometric normal must point away from the box centroid, and the shading normal
// must agree with it (the viewer culls by winding; the path tracer is two-sided
// and would silently tolerate a flip).
TEST_CASE(grammar_hlod_proxy_faces_outward) {
    for (int seed : {3, 11, 29}) {
        BuildingParams p; p.floors = 5; p.seed = seed;
        Scope s = scopeFromFootprint(square(18), 0.0, 20.0);
        BuildingMesh bm = growBuilding(s, p);
        const RenderMesh& m = bm.proxy;
        CHECK(!m.vertices.empty());
        CHECK(m.indices.size() >= 3);

        Vec3 c(0, 0, 0);
        for (const Vertex& v : m.vertices) c = c + v.position;
        c = c * (1.0f / static_cast<float>(m.vertices.size()));

        int inward = 0, disagree = 0, tris = 0;
        for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
            const Vec3& a = m.vertices[m.indices[i]].position;
            const Vec3& b = m.vertices[m.indices[i + 1]].position;
            const Vec3& d = m.vertices[m.indices[i + 2]].position;
            // The engine winds front faces CLOCKWISE: outward = cross(c-a, b-a).
            Vec3 gn = cross(d - a, b - a);
            if (gn.length() < 1e-6f) continue;      // degenerate: not a facing
            gn = normalize(gn);
            ++tris;
            const Vec3 mid = (a + b + d) * (1.0f / 3.0f);
            if (dot(gn, mid - c) < 0.0) ++inward;   // winding faces the interior
            // the baked shading normal must agree with the winding
            if (dot(gn, m.vertices[m.indices[i]].normal) < 0.0) ++disagree;
        }
        CHECK(tris > 0);
        CHECK(inward == 0);
        CHECK(disagree == 0);
    }
}

TEST_CASE(grammar_wall_part_picks_procedural_material) {
    Scope s = scopeFromFootprint(square(18), 0.0, 16.0);
    // A facade routes its wall geometry to its chosen procedural part (here
    // brick), not the flat Wall part.
    BuildingParams brick; brick.floors = 4; brick.seed = 7; brick.wallPart = PartId::Brick;
    BuildingMesh bm = growBuilding(s, brick);
    CHECK(hasPart(bm, PartId::Brick));
    CHECK(!hasPart(bm, PartId::Wall));

    // The default keeps the flat Wall material.
    BuildingParams plain; plain.floors = 4; plain.seed = 7;
    BuildingMesh pm = growBuilding(s, plain);
    CHECK(hasPart(pm, PartId::Wall));
    CHECK(!hasPart(pm, PartId::Brick));

    // Each facade part carries its Surface id (world-space procedural material);
    // the flat Wall carries none.
    using S = RenderMaterial::Surface;
    Vec3 c(0.5, 0.25, 0.18);
    CHECK(materialFor(PartId::Brick, c).surface() == S::Brick);
    CHECK(materialFor(PartId::Concrete, c).surface() == S::Concrete);
    CHECK(materialFor(PartId::Stucco, c).surface() == S::Stucco);
    CHECK(materialFor(PartId::Metal, c).surface() == S::CorrugatedMetal);
    CHECK(materialFor(PartId::Wall, c).surface() == S::None);
}

TEST_CASE(grammar_walkable_ground_punches_a_door) {
    BuildingParams p; p.floors = 3; p.walkableGround = true; p.seed = 1;
    Scope s = scopeFromFootprint(square(16), 0.0, 12.0);
    BuildingMesh bm = growBuilding(s, p);
    CHECK(hasPart(bm, PartId::Door));            // a real entrance opening
    bool entrance = false;
    for (const AttachPoint& a : bm.attaches) if (a.tag == "entrance") entrance = true;
    CHECK(entrance);
}

TEST_CASE(grammar_taller_with_more_floors_and_deterministic) {
    Scope s = scopeFromFootprint(square(16), 0.0, 12.0);
    BuildingParams a; a.floors = 4; a.seed = 7;
    BuildingParams b; b.floors = 12; b.seed = 7;
    CHECK(growBuilding(s, b).height > growBuilding(s, a).height);
    // Same seed + params -> identical geometry.
    BuildingMesh m1 = growBuilding(s, a);
    BuildingMesh m2 = growBuilding(s, a);
    CHECK(m1.parts.size() == m2.parts.size());
    CHECK(m1.merged().vertices.size() == m2.merged().vertices.size());
}

// --- parcels (Phase 1) ------------------------------------------------------

TEST_CASE(parcel_subdivides_into_lots_conserving_area) {
    Poly2 block = square(100);                   // 10,000 m^2
    ParcelParams pp; pp.targetArea = 420; pp.seed = 5;
    std::vector<Lot> lots = subdivideBlock(block, pp);
    CHECK(lots.size() > 5);
    Real total = 0;
    int courts = 0;
    for (const Lot& l : lots) {
        total += l.area;
        CHECK(l.area > 0);
        CHECK_APPROX(l.frontage.length(), 1.0, 1e-6);
        if (l.court) ++courts;
    }
    // Frontage-first parceling (v2 step 10): lots ring the block + one
    // interior court, with a single alley gap — so they NEVER overlap
    // (total <= block area) and cover most of it, but no longer partition it
    // exactly. A deep square block yields exactly one court.
    CHECK(total <= 10000.0 + 1.0);               // no overlap / no area created
    CHECK(total >= 10000.0 * 0.85);              // most of the block is parcelled
    CHECK(courts == 1);                          // the reachable interior court
}

TEST_CASE(parcel_is_deterministic) {
    Poly2 block = square(80);
    ParcelParams pp; pp.seed = 9;
    CHECK(subdivideBlock(block, pp).size() == subdivideBlock(block, pp).size());
}

// --- road network (Phase 2) -------------------------------------------------

TEST_CASE(road_grid_blocks_equal_cells) {
    GridRoadParams gp; gp.extent = 200; gp.cellSize = 100; gp.jitter = 0; gp.seed = 1;
    RoadGraph g = gridRoads(gp);
    CHECK(g.nodes.size() == 25);                 // 5x5 node grid
    RoadGraph pg = planarize(g);
    std::vector<Poly2> blocks = extractBlocks(pg);
    CHECK(blocks.size() == 16);                  // 4x4 cells
    Real total = 0;
    for (const Poly2& b : blocks) { total += area(b); CHECK(isCCW(b)); }
    CHECK_APPROX(total, 400.0 * 400.0, 1.0);     // faces tile the region exactly
}

TEST_CASE(road_planarize_splits_a_crossing) {
    // Two crossing segments forming an X: planarize must insert the centre node
    // and split both edges into 4.
    RoadGraph g;
    int a = g.addNode({-10, 0}), b = g.addNode({10, 0});
    int c = g.addNode({0, -10}), d = g.addNode({0, 10});
    g.addEdge(a, b); g.addEdge(c, d);
    RoadGraph pg = planarize(g);
    CHECK(pg.nodes.size() == 5);                 // 4 ends + 1 crossing
    CHECK(pg.edges.size() == 4);                 // each segment split in two
}

// --- Floorplan buildings (building-grammar-plan.md P3) ----------------------

TEST_CASE(plan_building_extrudes_an_l_plan) {
    // An L-shaped plan grows an L-shaped building: geometry reaches into BOTH
    // wings, the winding convention holds (it culls correctly in the viewer),
    // and the build is deterministic.
    Poly2 L = {{-12, -10}, {12, -10}, {12, 2}, {2, 2}, {2, 10}, {-12, 10}};
    BuildingParams p;
    p.floors = 3;
    p.seed = 4;
    p.wallPart = PartId::Brick;
    p.quoins = true;
    BuildingMesh bm = growPlanBuilding(L, p);
    CHECK(!bm.parts.empty());
    CHECK(bm.height > 6.0);
    RenderMesh m = bm.merged();
    CHECK(!m.vertices.empty());
    // Coverage of both wings: some wall vertex deep in the east wing (x > 8,
    // z < 0) AND some in the south wing (z > 6, x < 0).
    bool east = false, south = false;
    for (const Vertex& v : m.vertices) {
        if (v.position.x > 8 && v.position.z < 0) east = true;
        if (v.position.z > 6 && v.position.x < 0) south = true;
    }
    CHECK(east);
    CHECK(south);
    int bad = 0;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vertex& a = m.vertices[m.indices[i]];
        const Vertex& b = m.vertices[m.indices[i + 1]];
        const Vertex& c = m.vertices[m.indices[i + 2]];
        if (dot(cross(b.position - a.position, c.position - a.position), a.normal) > 1e-6)
            ++bad;
    }
    CHECK(bad == 0);
    BuildingMesh bm2 = growPlanBuilding(L, p);
    CHECK(bm2.merged().vertices.size() == m.vertices.size());
}

TEST_CASE(plan_building_setbacks_stack_tiers) {
    // setbackFloors/setbackEvery shrink the plan per tier (base/shaft/capital):
    // wall vertices in the TOP tier sit strictly inside the base footprint.
    Poly2 sq = {{-10, -10}, {10, -10}, {10, 10}, {-10, 10}};
    BuildingParams p;
    p.floors = 9;
    p.setbackFloors = 3;
    p.setbackEvery = 1.5;
    p.seed = 2;
    p.groundHeight = 4.0;
    p.floorHeight = 3.0;
    BuildingMesh bm = growPlanBuilding(sq, p);
    RenderMesh m = bm.merged();
    // Top tier spans the last three floors: y in [4 + 6*3, 4 + 9*3].
    Real topY0 = 4.0 + 6 * 3.0 + 0.5;
    Real maxR = 0;
    for (const Vertex& v : m.vertices) {
        if (v.position.y < topY0 || v.position.y > 4.0 + 9 * 3.0) continue;
        maxR = std::max(maxR, std::max(std::fabs(v.position.x), std::fabs(v.position.z)));
    }
    CHECK(maxR > 1.0);            // the top tier exists
    CHECK(maxR < 10.0 - 1.0);     // and is set back inside the base (2 tiers in)
}

TEST_CASE(plan_building_grows_pitched_roofs) {
    // Gable: sloped roof faces (normals tilted, not flat-up) + gable-end wall
    // triangles rising above the eaves; Hip: sloped ends instead of walls.
    Poly2 sq = {{-8, -6}, {8, -6}, {8, 6}, {-8, 6}};
    BuildingParams p;
    p.floors = 2;
    p.seed = 3;
    p.roofStyle = BuildingParams::RoofStyle::Gable;
    p.roofPitch = 0.6;
    BuildingMesh gable = growPlanBuilding(sq, p);
    int sloped = 0;
    Real maxY = 0;
    RenderMesh gm = gable.merged();
    for (std::size_t i = 0; i + 2 < gm.indices.size(); i += 3) {
        const Vec3& n = gm.vertices[gm.indices[i]].normal;
        if (n.y > 0.25 && n.y < 0.93) ++sloped;
    }
    for (const Vertex& v : gm.vertices) maxY = std::max(maxY, (Real)v.position.y);
    CHECK(sloped >= 2);                     // the two roof slopes exist
    CHECK(maxY > gable.height - 0.1);       // height includes the ridge
    p.roofStyle = BuildingParams::RoofStyle::Hip;
    BuildingMesh hip = growPlanBuilding(sq, p);
    int hipSloped = 0;
    RenderMesh hm = hip.merged();
    for (std::size_t i = 0; i + 2 < hm.indices.size(); i += 3) {
        const Vec3& n = hm.vertices[hm.indices[i]].normal;
        if (n.y > 0.25 && n.y < 0.93) ++hipSloped;
    }
    CHECK(hipSloped > sloped);              // hips slope the ends too

    // SOFFIT (device: "you can see through the bottom of them if they
    // overhang"): the roof volume is CLOSED underneath — down-facing faces at
    // the eave plane reach past the wall line to the overhang's outer edge.
    auto soffitReach = [](const RenderMesh& m, Real wallHalfDepth) {
        Real reach = 0;
        for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
            const Vec3& n = m.vertices[m.indices[i]].normal;
            if (n.y > -0.99) continue;                       // not a soffit face
            for (int k = 0; k < 3; ++k) {
                const Vec3& q = m.vertices[m.indices[i + k]].position;
                if (q.y < 1.0) continue;                     // ground slab, not eaves
                reach = std::max(reach, std::fabs((Real)q.z));
            }
        }
        return reach - wallHalfDepth;   // > 0: covers the overhang
    };
    CHECK(soffitReach(gm, 6.0) > 0.2);
    CHECK(soffitReach(hm, 6.0) > 0.2);
}

TEST_CASE(plan_building_prow_tiers_stay_bounded) {
    // A rounded flatiron PROW with aggressive setback tiers: the tier insets
    // used to explode at the near-parallel arc chords — the line intersection
    // flies off and the ring grows spikes (device: "one of the triangle
    // skyscrapers went haywire when building the top"). Every emitted vertex
    // must stay within the ground plan's bounds (+ cornice/eave slack).
    Poly2 prow = {{-10, -7}, {6, -6}};
    for (int k = 0; k <= 4; ++k) {   // the arc nose: near-parallel chords
        Real t = k / 4.0, a = -0.6 + t * 1.9;
        prow.push_back(Vec2(6 + 3.0 * std::cos(a), -2 + 3.0 * std::sin(a)));
    }
    prow.push_back({-9, 7});
    BuildingParams p;
    p.floors = 12;
    p.setbackFloors = 3;
    p.setbackEvery = 1.5;
    p.curtainWall = true;
    p.seed = 4;
    BuildingMesh bm = growPlanBuilding(prow, p);
    Vec2 lo, hi;
    bounds(prow, lo, hi);
    RenderMesh m = bm.merged();
    for (const Vertex& v : m.vertices) {
        CHECK(v.position.x > lo.x - 2.0);
        CHECK(v.position.x < hi.x + 2.0);
        CHECK(v.position.z > lo.y - 2.0);
        CHECK(v.position.z < hi.y + 2.0);
    }
}

// The MASS STACK (skyscrapers v2, ADR-0086 point 5): the 1916 New York
// envelope stacks a street wall, a first setback, later steps and a shaft;
// the storey stack follows it floor by floor; Envelope::None reproduces the
// uniform setback offsets exactly.
TEST_CASE(street_wall_setback_envelope_stacks_a_base_steps_and_a_shaft) {
    BuildingParams p;
    p.floors = 30;
    p.envelope = BuildingParams::Envelope::StreetWallSetback;
    p.baseFloors = 5;
    p.setback1 = 6.0;
    p.stepFloors = 8;
    p.stepDepth = 2.0;
    p.towerFrac = 0.35;
    p.towerFloor = 21;
    const Poly2 plan = {{0, 0}, {48, 0}, {48, 36}, {0, 36}};
    const std::vector<MassTier> tiers = massStack(plan, p);
    CHECK(tiers.size() >= 4u);   // base, the first setback, a step, the shaft
    for (std::size_t i = 1; i < tiers.size(); ++i) {
        CHECK(tiers[i].floor0 > tiers[i - 1].floor0);
        CHECK(area(tiers[i].plan) < area(tiers[i - 1].plan));
        CHECK(tiers[i].plan.size() == 4u);   // rectilinear on a rectangular plan
        for (const Vec2& v : tiers[i].plan) CHECK(pointInPolygon(tiers[i - 1].plan, v));
    }
    CHECK(tiers[1].floor0 == 5);
    const Real shaft = area(tiers.back().plan);
    CHECK(shaft > 0.25 * area(plan) && shaft < 0.45 * area(plan));
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, p);
    CHECK(storeys.size() == 31u);          // the ground storey + 30 floors
    CHECK(storeys[5].tier == 0);           // floor 4 still wears the street wall
    CHECK(storeys[6].tier == 1);           // floor 5 wears the first setback
    CHECK(storeys.back().tier == static_cast<int>(tiers.size()) - 1);
    BuildingMesh bm = growPlanBuilding(plan, p);
    CHECK(!bm.parts.empty());
    CHECK(bm.height > 30 * p.floorHeight);
    // Envelope::None: the uniform offsets, tier by tier as before.
    BuildingParams q;
    q.floors = 12;
    q.setbackFloors = 4;
    q.setbackEvery = 1.5;
    const std::vector<StoreyPlan> s2 = storeyPlans(plan, q);
    CHECK(s2.back().tier == 2);
    CHECK(s2[5].tier == 1);                // floor 4: the first uniform setback
}

// NIGHT LIGHTING (skyscrapers v2 M4): a lit pane's vertex colour is its tint,
// picked per window from the building's palette — cool whites behind a
// curtain wall, warm ones elsewhere — and the lit-glass material carries the
// emissive-vertex-tint flag so the shaders apply it to the emission only.
TEST_CASE(lit_panes_carry_a_tint_palette) {
    CHECK((materialFor(PartId::GlassLit, Vec3(1, 1, 1)).flags &
           RenderMaterial::FLAG_EMISSIVE_VERTEX_TINT) != 0u);
    CHECK((materialFor(PartId::Glass, Vec3(1, 1, 1)).flags &
           RenderMaterial::FLAG_EMISSIVE_VERTEX_TINT) == 0u);
    auto tints = [](bool curtain) {
        BuildingParams p;
        p.floors = 8;
        p.curtainWall = curtain;
        p.seed = 5;
        const Poly2 plan = {{0, 0}, {30, 0}, {30, 20}, {0, 20}};
        BuildingMesh bm = growPlanBuilding(plan, p);
        std::map<std::string, int> counts;
        // BuildingMesh::parts is packed (one entry per non-empty part, materialIndex = the PartId).
        const RenderMesh* lit = nullptr;
        for (const RenderMesh& part : bm.parts)
            if (part.materialIndex == static_cast<int>(PartId::GlassLit)) lit = &part;
        CHECK(lit != nullptr);
        if (!lit) return counts;
        for (const Vertex& v : lit->vertices) {
            char key[64];
            std::snprintf(key, sizeof key, "%.2f,%.2f,%.2f", v.color.x, v.color.y, v.color.z);
            counts[key] += 1;
        }
        return counts;
    };
    const auto office = tints(true), homes = tints(false);
    CHECK(office.size() >= 2u);   // a palette, not one colour
    CHECK(homes.size() >= 2u);
    auto top = [](const std::map<std::string, int>& m) {
        std::string best; int n = -1;
        for (const auto& kv : m) if (kv.second > n) { n = kv.second; best = kv.first; }
        return best;
    };
    CHECK(top(office) != top(homes));   // offices read cool, homes warm
    // The tint is a per-window choice hashed from the pane's anchor: deterministic.
    const auto again = tints(true);
    CHECK(again == office);
}

// NIGHT DRESSING (skyscrapers v2 M4, step 2): a tall tower wears a crown band
// under its coping and red aviation beacons at its roof corners, all as lit
// glass with a per-building tint; a short house wears none of it.
TEST_CASE(tall_towers_wear_a_crown_and_beacons) {
    auto litAbove = [](int floors, bool curtain, Real minY, int& red) {
        BuildingParams p;
        p.floors = floors;
        p.curtainWall = curtain;
        p.seed = 9;
        const Poly2 plan = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
        BuildingMesh bm = growPlanBuilding(plan, p);
        int n = 0;
        red = 0;
        for (const RenderMesh& part : bm.parts) {
            const bool lit = part.materialIndex == static_cast<int>(PartId::GlassLit);
            const bool beacon = part.materialIndex == static_cast<int>(PartId::Beacon);
            if (!lit && !beacon) continue;
            for (const Vertex& v : part.vertices) {
                if (v.position.y < minY) continue;
                if (lit) ++n;
                // The roof-corner lamps live in the FLASHING part, red-tinted.
                if (beacon && v.color.x > 0.9f && v.color.y < 0.2f) ++red;
            }
        }
        return n;
    };
    int red = 0;
    // 30 floors of 3.2 m + a 4.5 m ground storey: the roof is over 61 m, so
    // beacons and (for a curtain wall) a crown or signage sit above it.
    const int tall = litAbove(30, true, 4.5 + 30 * 3.2 - 0.1, red);
    CHECK(tall > 0);
    CHECK(red >= 4 * 24);   // four corner beacons, six faces of four vertices each
    {   // ...each wearing a translucent halo bulb in its own part.
        BuildingParams p;
        p.floors = 30;
        p.curtainWall = true;
        p.seed = 9;
        const BuildingMesh bm = growPlanBuilding({{0, 0}, {40, 0}, {40, 40}, {0, 40}}, p);
        std::size_t haloTris = 0;
        for (const RenderMesh& part : bm.parts)
            if (part.materialIndex == static_cast<int>(PartId::BeaconGlow)) haloTris += part.indices.size() / 3;
        CHECK(haloTris >= 4 * 128);
        CHECK(materialFor(PartId::BeaconGlow, Vec3(1, 1, 1)).opacity < 1.0f);
        // ...and leaves the runtime a "beacon" attach point per lamp, at the
        // lamp: above the roof, inside the plan.
        int beacons = 0;
        for (const AttachPoint& ap : bm.attaches) {
            if (ap.tag != "beacon") continue;
            ++beacons;
            CHECK(ap.position.y > 4.5 + 30 * 3.2);
            CHECK(ap.position.x > 0 && ap.position.x < 40 && ap.position.z > 0 && ap.position.z < 40);
        }
        CHECK(beacons >= 4);
        // The far (Flat) mesh carries the housing but not the spheres.
        const BuildingMesh flat = growPlanBuilding({{0, 0}, {40, 0}, {40, 40}, {0, 40}}, p, 0.0, FacadeDetail::Flat);
        std::size_t flatHalo = 0, flatLamp = 0;
        for (const RenderMesh& part : flat.parts) {
            if (part.materialIndex == static_cast<int>(PartId::BeaconGlow)) flatHalo += part.indices.size();
            if (part.materialIndex == static_cast<int>(PartId::Beacon)) flatLamp += part.indices.size();
        }
        CHECK(flatHalo == 0);
        CHECK(flatLamp > 0);
    }
    int redLow = 0;
    const int low = litAbove(4, false, 4.5 + 4 * 3.2 - 0.1, redLow);
    CHECK(low == 0);
    CHECK(redLow == 0);
}

TEST_CASE(curtain_wall_interior_skins_meet_at_the_corners) {
    // Glenn (2026-09-14): "the skyscraper interiors don't have corners so you
    // can see through the walls" — the painted skin behind a curtain wall
    // was one inset quad per edge, leaving an inset-wide slot at every
    // corner. Now every skin runs to the INSET polygon's corner, so each
    // corner is shared by two skins on every storey, the ground included.
    const Poly2 plan = {{0, 0}, {30, 0}, {30, 24}, {0, 24}};
    BuildingParams p;
    p.floors = 6;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.core = 1;   // no core: the four facade skins are the whole room
    p.seed = 3;
    const Real inset = std::max(p.wallThickness, Real(0.55));
    auto cornersShared = [&](const BuildingMesh& bm, Real y0, const Poly2& storeyPlan) {
        const Poly2 inner = offsetPolygonEdges(storeyPlan, std::vector<Real>(storeyPlan.size(), -inset));
        int shared = 0;
        for (const Vec2& c : inner) {
            int hits = 0;
            for (const RenderMesh& part : bm.parts) {
                if (part.materialIndex != static_cast<int>(PartId::Interior)) continue;
                for (const Vertex& v : part.vertices)
                    if (std::fabs(v.position.y - y0) < 1e-4 && std::fabs(v.position.x - c.x) < 1e-3 &&
                        std::fabs(v.position.z - c.y) < 1e-3)
                        ++hits;
            }
            if (hits >= 2) ++shared;
        }
        return shared;
    };
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, p);
    const BuildingMesh upper = growInterior(plan, p, 0.0);
    CHECK(cornersShared(upper, storeys[2].y0, storeys[2].plan) == 4);
    const BuildingMesh ground = growPlanBuilding(plan, p, 0.0, FacadeDetail::Full);
    CHECK(cornersShared(ground, 0.0, plan) == 4);   // the entrance wall and the blank stair wall mitre too
}

TEST_CASE(distant_mass_box_carries_window_cell_uvs) {
    // The far tier's lit windows: the mass box's wall UVs count window cells
    // (3.2 m per cell, eight cells per texture repeat), the roof cap samples
    // the map's dark first cell.
    LotBuilding lb;
    lb.site = Vec2(10, 20);
    lb.width = 32.0;   // ten cells along
    lb.depth = 16.0;   // five cells across
    lb.height = 51.2;  // sixteen cells up
    lb.baseY = 0;
    lb.yaw = 0;
    RenderMesh m;
    appendLotMassBox(m, lb, Vec3(0.5, 0.5, 0.5), Vec3(0.2, 0.2, 0.2), std::numeric_limits<Real>::quiet_NaN());
    CHECK(m.vertices.size() == 20);
    float maxU = 0, maxV = 0;
    for (std::size_t i = 0; i < 16; ++i) { maxU = std::max(maxU, m.vertices[i].u); maxV = std::max(maxV, m.vertices[i].v); }
    CHECK(std::fabs(maxU - 32.0f / (3.2f * 8)) < 1e-4f);
    CHECK(std::fabs(maxV - 51.2f / (3.2f * 8)) < 1e-4f);
    for (std::size_t i = 16; i < 20; ++i) CHECK(m.vertices[i].u < 0.1f && m.vertices[i].v < 0.1f);
}

TEST_CASE(curtain_wall_interiors_look_out_through_clear_panes) {
    // Looking out (skyscrapers v2): the facade's panes are front-face only,
    // and a curtain-wall storey's inner face is a glass part above a painted
    // spandrel band — the interior system draws that part clear.
    CHECK(materialFor(PartId::Glass, Vec3(1, 1, 1)).flags & RenderMaterial::FLAG_FRONT_ONLY);
    CHECK(materialFor(PartId::GlassLit, Vec3(1, 1, 1)).flags & RenderMaterial::FLAG_FRONT_ONLY);
    const Poly2 plan = {{0, 0}, {30, 0}, {30, 24}, {0, 24}};
    BuildingParams p;
    p.floors = 6;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.core = 1;
    p.seed = 3;
    const BuildingMesh bm = growInterior(plan, p, 0.0);
    std::size_t glassVerts = 0, paintVerts = 0;
    for (const RenderMesh& part : bm.parts) {
        if (part.materialIndex == static_cast<int>(PartId::Glass)) glassVerts += part.vertices.size();
        if (part.materialIndex == static_cast<int>(PartId::Interior)) paintVerts += part.vertices.size();
    }
    CHECK(glassVerts >= 4 * 4 * 6);   // four edges, six upper storeys, a quad each
    CHECK(paintVerts > 0);
}
