#include "test_framework.h"

#include "../src/engine/procgen/city/polygon.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/parcel.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/road_mesh.h"
#include "../src/engine/procgen/city/city.h"
#include "../src/engine/mesh_builder.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <set>

using namespace engine;  // namespace migration (ADR-0015)

TEST_CASE(road_mesh_builds_junctions_and_ribbons) {
    // ADR-0044: the road surface trims ribbons back at junctions and fills the
    // gap with a pad, so a 4-way crossing produces real geometry (not overlap).
    RoadGraph g;
    int c = g.addNode(Vec2(0, 0));
    int e = g.addNode(Vec2(40, 0)), w = g.addNode(Vec2(-40, 0));
    int n = g.addNode(Vec2(0, 40)), s = g.addNode(Vec2(0, -40));
    g.addEdge(c, e, 10); g.addEdge(c, w, 10);
    g.addEdge(c, n, 10); g.addEdge(c, s, 10);

    RoadMeshParams p;
    RenderMesh m = buildRoadMesh(g, p);
    CHECK(!m.vertices.empty());
    CHECK(m.indices.size() % 3 == 0);
    // The degree-4 hub adds a junction pad (8 ring verts -> 8 fan triangles) on
    // top of the four ribbons, so there is real intersection geometry.
    CHECK(m.indices.size() / 3 >= 8u + 4u * 2u);

    // No road geometry strays beyond the arms' reach (sanity on the trim).
    double maxR = 0;
    for (const Vertex& v : m.vertices)
        maxR = std::max(maxR, std::sqrt(double(v.position.x * v.position.x +
                                             v.position.z * v.position.z)));
    CHECK(maxR <= 41.0);
}

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
    RenderMesh m = buildRoadMesh(pg, RoadMeshParams{});
    CHECK(meshCoversXZ(m, 2.0, 2.0));                   // the intersection is filled
    // Nothing strays past an arm end (length 40) plus its half-width (5) — the far
    // ribbon corner sits at sqrt(40^2+5^2) ~ 40.3, so bound by the arm reach + width.
    double worst = 0;
    for (const Vertex& v : m.vertices)
        worst = std::max(worst, std::sqrt(double(v.position.x*v.position.x +
                                                 v.position.z*v.position.z)));
    CHECK(worst <= 40.0 + 5.0 + 1e-3);
}

TEST_CASE(union_ribbons_merges_crossing_without_overlap) {
    // ADR-0048: two perpendicular strips. The union is non-overlapping by
    // construction (disjoint grid cells), so its SUMMED triangle area is the true
    // union area — it covers the crossing, yet is strictly less than the two strokes'
    // summed area, which double-counts the overlap.
    UnionSpine h; h.halfWidth = 5; h.points = { {-50,0}, {50,0} };
    UnionSpine v; v.halfWidth = 5; v.points = { {0,-50}, {0,50} };
    Vec3 c(0.1, 0.1, 0.1);
    RenderMesh u = unionRibbons({ h, v }, 0.5, 0.0, c);
    CHECK(meshCoversXZ(u, 1.0, 1.0));        // the crossing is filled
    CHECK(meshCoversXZ(u, 40.0, 0.3));       // one arm
    CHECK(meshCoversXZ(u, 0.3, 40.0));       // the other arm
    CHECK(!meshCoversXZ(u, 40.0, 40.0));     // the empty quadrant stays empty

    // Method-consistent non-overlap proof: union-of-both < (union-of-each summed),
    // by the crossing overlap that was MERGED rather than double-counted. The bodies
    // overlap in a ~10x10 = ~100 square at the centre.
    double uBoth = meshAreaXZ(u);
    double uH = meshAreaXZ(unionRibbons({ h }, 0.5, 0.0, c));
    double uV = meshAreaXZ(unionRibbons({ v }, 0.5, 0.0, c));
    double overlap = (uH + uV) - uBoth;
    CHECK(overlap > 80.0);                   // the crossing was merged (~100)
    CHECK(overlap < 130.0);                  // and it's only the crossing, nothing more
}

TEST_CASE(union_ribbons_keeps_a_ring_hole) {
    // A closed ring unions to an annulus: the centre stays uncovered.
    std::vector<Vec2> circ;
    const double R = 30.0;
    for (int i = 0; i < 48; ++i) {
        double t = 2.0 * 3.14159265 * i / 48;
        circ.push_back(Vec2(R*std::cos(t), R*std::sin(t)));
    }
    UnionSpine ring; ring.halfWidth = 5; ring.closed = true; ring.points = circ;
    RenderMesh u = unionRibbons({ ring }, 0.6, 0.0, Vec3(0.1,0.1,0.1));
    CHECK(!u.vertices.empty());
    CHECK(!meshCoversXZ(u, 0.0, 0.0));       // hole preserved
    CHECK(meshCoversXZ(u, R, 0.3));          // ring covered
}

TEST_CASE(union_roadbed_adds_a_sidewalk_band_and_drapes) {
    // ADR-0048: a straight road. The roadbed covers the carriageway, a sidewalk band
    // just outside it (0 <= sdf < sidewalkWidth), nothing past it; and with a height
    // field every vertex is seated on the terrain.
    UnionSpine s; s.halfWidth = 5; s.points = { {-40,0}, {40,0} };
    RoadbedParams p; p.cell = 0.5; p.sidewalkWidth = 2.0; p.curbHeight = 0.2; p.lift = 0.05;
    p.heightAt = [](double x, double) { return 0.01 * x; };     // a gentle ramp
    RenderMesh m = unionRoadbed({ s }, p);
    CHECK(meshCoversXZ(m, 0.0, 0.0));        // carriageway (sdf = -5)
    CHECK(meshCoversXZ(m, 0.0, 6.0));        // sidewalk band (dist 6 -> sdf 1)
    CHECK(!meshCoversXZ(m, 0.0, 8.0));       // beyond the sidewalk (sdf 3 > 2)

    // Draped: every vertex sits on terrain(=0.01x over ~[-47,47]) + lift (+curb on the
    // walk), so the y-range is the terrain range, not a flat plane.
    double minY = 1e9, maxY = -1e9;
    for (const Vertex& v : m.vertices) {
        minY = std::min(minY, static_cast<double>(v.position.y));
        maxY = std::max(maxY, static_cast<double>(v.position.y));
    }
    CHECK(minY > -0.6);
    CHECK(maxY < 0.8);
    CHECK(maxY - minY > 0.5);                // it follows the ramp, not flat
}

TEST_CASE(union_roadbed_from_graph_converts_edges) {
    // ADR-0048: the RoadGraph overload turns each edge into a spine, so a generated
    // network (an X crossing here) roadbeds into one merged surface covering the join.
    RoadGraph g;
    int a = g.addNode(Vec2(-30,0)), b = g.addNode(Vec2(30,0));
    int c = g.addNode(Vec2(0,-30)), d = g.addNode(Vec2(0,30));
    g.addEdge(a, b, 10); g.addEdge(c, d, 10);
    CHECK(graphToSpines(g).size() == 2u);                // one spine per edge
    RoadbedParams p; p.cell = 0.6; p.sidewalkWidth = 2.0;
    RenderMesh m = unionRoadbed(g, p);
    CHECK(!m.vertices.empty());
    CHECK(meshCoversXZ(m, 1.0, 1.0));                    // the crossing is merged
    CHECK(meshCoversXZ(m, 25.0, 0.3));                   // an arm
}

TEST_CASE(road_mesh_hairpin_builds_a_turning_pad) {
    // ADR-0048: a sharp degree-2 reversal (legs nearly parallel = ~160-degree
    // deflection) can't be a simple bend — the widened ribbon folds. With hairpin
    // handling on, the apex is built as a junction-style turning pad, so it pulls the
    // ribbons back and emits more geometry than the plain folded bend.
    RoadGraph g;
    int A = g.addNode(Vec2(60, 6)), V = g.addNode(Vec2(0, 0)), B = g.addNode(Vec2(60, -6));
    g.addEdge(A, V, 10); g.addEdge(V, B, 10);

    RoadMeshParams plain;                       // hairpinDeflection = 0 -> simple bend
    RenderMesh mb = buildRoadMesh(g, plain);
    RoadMeshParams pad; pad.hairpinDeflection = 1.0;   // ~57deg+; this 160deg turn qualifies
    RenderMesh mp = buildRoadMesh(g, pad);

    CHECK(mp.indices.size() % 3 == 0);
    CHECK(!mp.vertices.empty());
    // The pad path is distinct from the simple bend: it pulls both ribbons back (a
    // setback) and fans a turning bulb, so the geometry differs from the folded bend.
    CHECK(mp.indices.size() != mb.indices.size());
}

TEST_CASE(road_mesh_sidewalks_raise_a_kerb) {
    // ADR-0044 cross-section: sidewalks add a raised skirt — a slab top standing
    // `curb` above the carriageway and an outer face dropping to the ground.
    RoadGraph g;
    int c = g.addNode(Vec2(0, 0));
    int e = g.addNode(Vec2(40, 0)), w = g.addNode(Vec2(-40, 0));
    int n = g.addNode(Vec2(0, 40)), s = g.addNode(Vec2(0, -40));
    g.addEdge(c, e, 10); g.addEdge(c, w, 10);
    g.addEdge(c, n, 10); g.addEdge(c, s, 10);

    RoadMeshParams flat;                       // no sidewalks (default width 0)
    RenderMesh bare = buildRoadMesh(g, flat);

    RoadMeshParams p;
    p.sidewalkWidth = 2.5;
    p.curbHeight = 0.15;
    RenderMesh m = buildRoadMesh(g, p);

    CHECK(m.indices.size() > bare.indices.size());   // the skirt added geometry

    // Flat ground (no heightAt): the road sits at `lift`, slab tops a curb above
    // it, and the outer face drops to ground (y = 0, below the road).
    double minY = 1e9, maxY = -1e9;
    for (const Vertex& v : m.vertices) {
        minY = std::min(minY, double(v.position.y));
        maxY = std::max(maxY, double(v.position.y));
    }
    CHECK_APPROX(maxY, flat.lift + p.curbHeight, 1e-6);   // raised exactly one curb
    CHECK(minY < flat.lift - 0.1);                        // outer face dropped to ground
}

TEST_CASE(road_mesh_plaza_opens_a_hub) {
    // ADR-0044: a many-armed hub with a plaza trims every arm back to the plaza
    // radius, so the junction fills a clean disc instead of a cramped fan.
    const double TAU = 6.283185307179586;
    RoadGraph g;
    int c = g.addNode(Vec2(0, 0));
    const int arms = 8;
    for (int i = 0; i < arms; ++i) {
        double a = TAU * i / arms;
        int e = g.addNode(Vec2(std::cos(a) * 60.0, std::sin(a) * 60.0));
        g.addEdge(c, e, 10);
    }
    // The plaza's signature is the cleared gap around the centre: the pad fans
    // from the node out to the trim radius, and the ribbons only start there, so
    // the nearest road vertex to the hub sits at ~plazaRadius (bigger than the
    // tight natural trim).
    auto hubGap = [](const RenderMesh& m) {
        double minR = 1e9;
        for (const Vertex& v : m.vertices) {
            double d = std::sqrt(double(v.position.x * v.position.x +
                                        v.position.z * v.position.z));
            if (d > 1.0) minR = std::min(minR, d);   // skip the pad-centre fan apex
        }
        return minR;
    };
    RoadMeshParams plain;
    RoadMeshParams p;
    p.plazaRadius = 20.0;
    p.plazaMinArms = 6;
    CHECK(hubGap(buildRoadMesh(g, p)) > 15.0);          // pad opened to the plaza
    CHECK(hubGap(buildRoadMesh(g, p)) > hubGap(buildRoadMesh(g, plain)));
}

TEST_CASE(road_mesh_lane_markings_paint_lines) {
    // ADR-0044: a marked carriageway carries a double-yellow centreline, dashed
    // white dividers, and solid edge lines, raised just above the asphalt — and
    // the lane count grows with the road width.
    RoadGraph g;
    int a = g.addNode(Vec2(-50, 0)), b = g.addNode(Vec2(50, 0));
    g.addEdge(a, b, 14);                       // wide enough for dividers each side

    RoadMeshParams plain;
    RenderMesh bare = buildRoadMesh(g, plain);

    RoadMeshParams p;
    p.laneMarkings = true;
    RenderMesh m = buildRoadMesh(g, p);
    CHECK(m.indices.size() > bare.indices.size());   // stripes added geometry

    bool sawYellow = false, sawWhite = false;
    double maxY = -1e9;
    for (const Vertex& v : m.vertices) {
        maxY = std::max(maxY, double(v.position.y));
        if (v.color.x > 0.7 && v.color.y > 0.6 && v.color.z < 0.3) sawYellow = true;
        if (v.color.x > 0.8 && v.color.y > 0.8 && v.color.z > 0.78) sawWhite = true;
    }
    CHECK(sawYellow);                                 // double-yellow centreline
    CHECK(sawWhite);                                  // edge lines + dividers
    CHECK_APPROX(maxY, plain.lift + p.markLift, 1e-6);  // stripes sit a hair proud
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

// --- city (Phase 3) ---------------------------------------------------------

TEST_CASE(city_generates_deterministically) {
    CityParams cp; cp.extent = 300; cp.cellSize = 100; cp.seed = 42;
    CityModel a = generateCity(cp);
    CityModel b = generateCity(cp);
    CHECK(a.buildings.size() == b.buildings.size());
    CHECK(a.blockCount == b.blockCount);
    CHECK(a.buildings.size() > 0);
    CHECK(!a.parts.empty());
}

TEST_CASE(city_highrise_is_taller_than_residential) {
    CityParams cp; cp.extent = 400; cp.cellSize = 95; cp.seed = 7;
    CityModel m = generateCity(cp);
    Real maxHigh = 0, maxResidential = 0;
    bool industrial = false;
    for (const CityBuilding& b : m.buildings) {
        if (b.district == District::HighRise) maxHigh = std::max(maxHigh, b.height);
        if (b.district == District::Residential) maxResidential = std::max(maxResidential, b.height);
        if (b.district == District::Industrial) industrial = true;
    }
    CHECK(maxHigh > maxResidential);
    CHECK(maxHigh > 40.0);                         // high-rise towers exist
    CHECK(industrial);                             // the industrial zone is populated
}

TEST_CASE(city_drapes_on_terrain_foundations_track_ground) {
    // City Arena (ADR-0038 §6): with a ground sampler, foundations sit on the
    // terrain (a ramp -> a spread of base elevations), there is no flat ground
    // plane, and street/park trees are scattered.
    CityParams cp; cp.extent = 280; cp.cellSize = 95; cp.seed = 4;
    cp.groundAt = [](const Vec2& p) { return 0.5 * p.x; };   // linear ramp
    CityModel m = generateCity(cp);
    CHECK(m.ground.vertices.empty());          // terrain is the ground
    CHECK(m.treeCount > 0);
    CHECK(!m.props.vertices.empty());

    Real lo = 1e30, hi = -1e30;
    for (const CityBuilding& b : m.buildings) {
        lo = std::min(lo, b.baseY); hi = std::max(hi, b.baseY);
        // The building sits on its block's flat grade (max ground over the whole
        // block), so it tracks the ramp within a block half-width (~50 m).
        CHECK(std::fabs(b.baseY - 0.5 * b.site.x) < 60.0);
    }
    CHECK(hi - lo > 50.0);                      // buildings span the ramp
}

TEST_CASE(city_on_terrain_emits_flatten_footprints) {
    // On terrain, the city hands back cut/fill footprints so the loader can grade
    // the terrain flat under the roads and blocks. A flat city emits none.
    CityParams flat; flat.extent = 200; flat.cellSize = 95; flat.seed = 5;
    CHECK(generateCity(flat).flatten.empty());

    CityParams cp; cp.extent = 200; cp.cellSize = 95; cp.seed = 5;
    cp.groundAt = [](const Vec2& p) { return 4.0 * std::sin(p.x * 0.01); };
    CityModel m = generateCity(cp);
    CHECK(!m.flatten.empty());

    // Each footprint is a sane polygon with a tight AABB, and its target plane
    // sits within the terrain's height range (it grades to the road/block grade,
    // not to some wild value).
    for (const TerrainFlatten& f : m.flatten) {
        CHECK(f.polygon.size() >= 3);
        CHECK(f.maxX >= f.minX);
        CHECK(f.maxZ >= f.minZ);
        double cx = (f.minX + f.maxX) * 0.5, cz = (f.minZ + f.maxZ) * 0.5;
        CHECK(std::fabs(f.planeY(cx, cz)) < 10.0);   // within +/- the 4 m relief + curb
    }
}

TEST_CASE(city_flat_keeps_ground_plane_and_trees) {
    CityParams cp; cp.extent = 200; cp.cellSize = 95; cp.seed = 2;
    CityModel m = generateCity(cp);            // no sampler -> flat
    CHECK(!m.ground.vertices.empty());         // flat city gets a ground plane
    CHECK(m.treeCount > 0);
    // Flat ground: every building sits on the block grade, the curb +0.12 lift.
    for (const CityBuilding& b : m.buildings) CHECK_APPROX(b.baseY, 0.12, 1e-6);
}

TEST_CASE(city_winding_matches_engine_convention) {
    // MeshBuilder::box renders correctly under the viewer's back-face culling: its
    // triangles wind so the geometric normal opposes the shading normal
    // (geo·normal <= 0). The city's buildings/roads/pavement MUST follow the same
    // convention or they render inside-out in the viewer — and the offline tracer
    // is two-sided, so this test is the only place a flipped winding is caught.
    auto wrong = [](const RenderMesh& m) {
        int bad = 0;
        for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
            const Vertex& a = m.vertices[m.indices[i]];
            const Vertex& b = m.vertices[m.indices[i + 1]];
            const Vertex& c = m.vertices[m.indices[i + 2]];
            if (dot(cross(b.position - a.position, c.position - a.position), a.normal) > 1e-6) ++bad;
        }
        return bad;
    };
    CHECK(wrong(MeshBuilder::box(Vec3(2, 2, 2))) == 0);     // the convention itself
    CityParams cp; cp.extent = 160; cp.cellSize = 85; cp.seed = 11;
    cp.groundAt = [](const Vec2& p) { return 6.0 * std::sin(p.x * 0.01); };
    CityModel m = generateCity(cp);
    for (const RenderMesh& part : m.parts) CHECK(wrong(part) == 0);  // buildings
    CHECK(wrong(m.roads) == 0);
    CHECK(wrong(m.pavement) == 0);
    // Curved + tiered masses too.
    Scope s = scopeFromFootprint({{-12, -10}, {12, -10}, {12, 10}, {-12, 10}}, 0, 10);
    BuildingParams cyl; cyl.shape = BuildingShape::Cylinder; cyl.floors = 8;
    BuildingParams pag; pag.shape = BuildingShape::Pagoda; pag.tiers = 5;
    CHECK(wrong(growBuilding(s, cyl).merged()) == 0);
    CHECK(wrong(growBuilding(s, pag).merged()) == 0);
}

TEST_CASE(city_has_street_furniture) {
    // Lamp posts + crosswalks are added regardless of tree scatter. Lamps and
    // signals are instanced (ADR-0041), so they live in instanceGroups, not the
    // baked props mesh — and at least one group carries placements.
    CityParams cp; cp.extent = 200; cp.cellSize = 95; cp.seed = 5;
    cp.scatterTrees = false;
    CityModel m = generateCity(cp);
    std::size_t instances = 0;
    for (const CityInstanceGroup& g : m.instanceGroups) instances += g.transforms.size();
    CHECK(instances > 0);                  // lamp posts / signals, without any trees
    CHECK(!m.roads.vertices.empty());      // roads + lane lines + crosswalks
}

TEST_CASE(city_buildings_have_valid_box_colliders) {
    CityParams cp; cp.extent = 200; cp.cellSize = 95; cp.seed = 5;
    cp.groundAt = [](const Vec2& p) { return 0.3 * p.x; };   // sloped, to vary baseY
    CityModel m = generateCity(cp);
    CHECK(!m.buildings.empty());
    for (const CityBuilding& b : m.buildings) {
        CHECK(b.boxHalf.x > 0.1 && b.boxHalf.y > 0.1 && b.boxHalf.z > 0.1);
        // Box centre sits half the height above the foundation.
        CHECK_APPROX(b.boxCenter.y, b.baseY + b.height * 0.5, 1e-6);
        // Box centre (the OBB centre) is near the footprint site. Not exact:
        // scopeFromFootprint anchors its shrink-fit at the lot OBB's centre,
        // which on an L/wedge off-cut can sit several metres from the polygon
        // centroid — 8 m still catches a real misplacement (roads are further).
        CHECK(distance(Vec2(b.boxCenter.x, b.boxCenter.z), b.site) < 8.0);
    }
}

TEST_CASE(city_hlod_proxy_is_far_cheaper_than_detail) {
    CityParams cp; cp.extent = 300; cp.cellSize = 100; cp.seed = 11;
    CityModel m = generateCity(cp);
    std::size_t detailTris = 0;
    for (const RenderMesh& p : m.parts) detailTris += p.indices.size() / 3;
    std::size_t proxyTris = m.hlodProxy.indices.size() / 3;
    CHECK(proxyTris > 0);
    CHECK(proxyTris == m.buildings.size() * 12);   // one 12-tri box per building
    // The whole-city HLOD is an order of magnitude lighter than the detail.
    CHECK(proxyTris * 10 < detailTris);
}

TEST_CASE(city_parks_leave_blocks_empty) {
    CityParams cp; cp.extent = 400; cp.cellSize = 95; cp.parkFraction = 0.3; cp.seed = 3;
    CityModel m = generateCity(cp);
    // With 30% parks, fewer buildings than a no-park run of the same seed.
    CityParams cp2 = cp; cp2.parkFraction = 0.0;
    CityModel m2 = generateCity(cp2);
    CHECK(m.buildings.size() < m2.buildings.size());
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
