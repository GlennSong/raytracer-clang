#include "test_framework.h"

#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/road_mesh.h"
#include <cmath>

using namespace engine;

namespace {

int degreeOf(const RoadGraph& g, int v) {
    int d = 0;
    for (const RoadEdge& e : g.edges) if (e.a == v || e.b == v) ++d;
    return d;
}

int maxDegree(const RoadGraph& g) {
    int mx = 0;
    for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v) mx = std::max(mx, degreeOf(g, v));
    return mx;
}

// A plain "+" crossing: a horizontal edge and a vertical edge that cross at the origin,
// each on `layerH` / `layerV`.
RoadGraph plusCrossing(int layerH, int layerV) {
    RoadGraph g;
    g.nodes = { {Vec2(-20, 0)}, {Vec2(20, 0)}, {Vec2(0, -20)}, {Vec2(0, 20)} };
    g.edges.push_back({0, 1, 8, RoadClass::Local, layerH});
    g.edges.push_back({2, 3, 8, RoadClass::Local, layerV});
    return g;
}

}  // namespace

// Same layer: the crossing becomes a real intersection — one shared node, degree 4.
TEST_CASE(layered_same_layer_makes_an_intersection) {
    RoadGraph g = plusCrossing(0, 0);
    CHECK(gradeSeparationCount(g) == 0);
    RoadGraph out = planarizeLayered(g);
    CHECK(maxDegree(out) == 4);                 // the four arms meet at the split node
    CHECK(out.edges.size() == 4);               // each edge cut in two
}

// Different layers: the crossing is an overpass — no shared node, both edges intact.
TEST_CASE(layered_different_layer_makes_an_overpass) {
    RoadGraph g = plusCrossing(0, 1);
    CHECK(gradeSeparationCount(g) == 1);
    RoadGraph out = planarizeLayered(g);
    CHECK(out.edges.size() == 2);               // neither edge was split
    CHECK(maxDegree(out) <= 2);                 // nothing meets: they pass over/under
    // The two layers are preserved on the output.
    int onGround = 0, above = 0;
    for (const RoadEdge& e : out.edges) (e.layer == 0 ? onGround : above)++;
    CHECK(onGround == 1);
    CHECK(above == 1);
}

// Default behaviour (everything on layer 0) matches the old planarize: a "+" intersects.
TEST_CASE(layered_defaults_to_at_grade) {
    RoadGraph g;
    g.nodes = { {Vec2(-20, 0)}, {Vec2(20, 0)}, {Vec2(0, -20)}, {Vec2(0, 20)} };
    g.addEdge(0, 1);            // layer defaults to 0
    g.addEdge(2, 3);
    CHECK(gradeSeparationCount(g) == 0);
    CHECK(planarizeLayered(g).edges.size() == planarize(g).edges.size());
}

// A freeway (layer 1) flying over a surface grid (layer 0): the grid still planarizes into
// its own intersections, while the freeway crosses every street as a separation.
TEST_CASE(layered_freeway_over_a_grid) {
    RoadGraph g;
    // Two parallel ground streets (vertical), crossed by one ground street (horizontal)...
    g.nodes = { {Vec2(-10, -30)}, {Vec2(-10, 30)},   // street A (x=-10)
                {Vec2(10, -30)},  {Vec2(10, 30)},    // street B (x=10)
                {Vec2(-30, 0)},   {Vec2(30, 0)},     // cross street (y=0)
                {Vec2(-30, 15)},  {Vec2(30, 15)} };  // the freeway (y=15), layer 1
    g.edges.push_back({0, 1, 8, RoadClass::Local, 0});
    g.edges.push_back({2, 3, 8, RoadClass::Local, 0});
    g.edges.push_back({4, 5, 8, RoadClass::Local, 0});
    g.edges.push_back({6, 7, 16, RoadClass::Arterial, 1});
    // The freeway crosses both vertical streets above grade -> 2 separations.
    CHECK(gradeSeparationCount(g) == 2);
    RoadGraph out = planarizeLayered(g);
    // The ground cross street splits both vertical streets (2 intersections, degree 4 each);
    // the freeway is untouched by them.
    int deg4 = 0;
    for (int v = 0; v < static_cast<int>(out.nodes.size()); ++v) if (degreeOf(out, v) == 4) ++deg4;
    CHECK(deg4 == 2);
}

// --- the clearance solver (ADR-0054): lift a road onto a deck that clears a crossing ---

namespace {
// `n` evenly spaced samples over [0, span], flat ground at 0, requiring `clear` height at
// the sample nearest the middle (the crossing).
void crossingProfile(int n, double span, double clear, std::vector<double>& s,
                     std::vector<double>& minH, int& mid) {
    s.resize(n); minH.assign(n, 0.0);
    for (int i = 0; i < n; ++i) s[i] = span * i / (n - 1);
    mid = n / 2;
    minH[mid] = clear;
}
}  // namespace

// The deck clears the obstacle and never breaks the grade limit; it never dips below ground.
TEST_CASE(clearance_meets_height_within_grade) {
    std::vector<double> s, minH; int mid;
    crossingProfile(41, 200.0, 6.0, s, minH, mid);   // 5 m spacing
    const double g = 0.06;
    std::vector<double> y = clearanceProfile(s, minH, g);
    for (int i = 0; i < (int)y.size(); ++i) CHECK(y[i] >= minH[i] - 1e-9);   // dominates
    for (int i = 1; i < (int)y.size(); ++i)
        CHECK(std::fabs(y[i] - y[i - 1]) <= g * (s[i] - s[i - 1]) + 1e-9);   // within grade
    CHECK(std::fabs(y[mid] - 6.0) < 1e-9);                                   // peak = clearance
}

// With room to ramp (100 m each side at 6% needs exactly 100 m), the deck returns to ground.
TEST_CASE(clearance_returns_to_ground_with_room) {
    std::vector<double> s, minH; int mid;
    crossingProfile(41, 200.0, 6.0, s, minH, mid);
    std::vector<double> y = clearanceProfile(s, minH, 0.06);
    CHECK(std::fabs(y.front()) < 1e-6);
    CHECK(std::fabs(y.back()) < 1e-6);
}

// Too short to drop back at grade -> the ends ride high (honest: the ramp needs more room).
TEST_CASE(clearance_ends_ride_high_when_cramped) {
    std::vector<double> s, minH; int mid;
    crossingProfile(21, 100.0, 6.0, s, minH, mid);   // only 50 m each side
    std::vector<double> y = clearanceProfile(s, minH, 0.06);
    CHECK(std::fabs(y.front() - 3.0) < 1e-6);         // 6 - 0.06*50 = 3 m still up at the end
    CHECK(std::fabs(y.back() - 3.0) < 1e-6);
}

// No raised constraint -> the profile just hugs the ground.
TEST_CASE(clearance_hugs_ground_when_unconstrained) {
    std::vector<double> s(11), minH(11, 0.0);
    for (int i = 0; i < 11; ++i) s[i] = i * 4.0;
    std::vector<double> y = clearanceProfile(s, minH, 0.08);
    for (double v : y) CHECK(std::fabs(v) < 1e-12);
}

// --- the bridge deck rides the clearance profile (ADR-0054) ---

// A straight road over a mid crossing: the deck top peaks at the clearance height and the
// slab gives the mesh `thickness` of vertical extent below the deck.
TEST_CASE(bridge_deck_rides_the_clearance_profile) {
    std::vector<double> s, minH; int mid;
    crossingProfile(41, 200.0, 6.0, s, minH, mid);
    std::vector<double> y = clearanceProfile(s, minH, 0.06);

    std::vector<Vec2> center(s.size());
    for (std::size_t i = 0; i < s.size(); ++i) center[i] = Vec2(s[i] - 100.0, 0.0);  // along x
    RenderMesh deck = bridgeDeck(center, y, 5.0, Vec3(0.1, 0.1, 0.1), 0.6);

    CHECK(!deck.vertices.empty());
    CHECK(deck.indices.size() % 3 == 0);

    double maxY = -1e30, minY = 1e30;
    for (const Vertex& v : deck.vertices) {
        maxY = std::max(maxY, (double)v.position.y);
        minY = std::min(minY, (double)v.position.y);
    }
    CHECK(std::fabs(maxY - 6.0) < 1e-6);          // deck top peaks at the clearance
    CHECK(std::fabs(minY - (0.0 - 0.6)) < 1e-6);  // fascia drops a slab thickness at the ends
}

// A degenerate input (mismatched lengths) yields an empty mesh, not a crash.
TEST_CASE(bridge_deck_rejects_bad_input) {
    std::vector<Vec2> center = { Vec2(0, 0), Vec2(10, 0) };
    std::vector<double> y = { 1.0 };              // wrong length
    CHECK(bridgeDeck(center, y, 4.0, Vec3(0, 0, 0)).vertices.empty());
}

// --- end-to-end layered road (ADR-0051/0054): edge layers + a lifted bridge deck ---

#include "../src/engine/procgen/city/road_net.h"
#include <nlohmann/json.hpp>

// edge_layers survives a JSON round-trip.
TEST_CASE(road_net_edge_layers_roundtrip) {
    RoadNet net;
    net.nodes = { Vec2(-100, 0), Vec2(100, 0), Vec2(0, -100), Vec2(0, 100) };
    net.edges = { {0, 1}, {2, 3} };
    net.edgeLayers = { 0, 1 };
    RoadNet back = roadNetFromJson(roadNetToJson(net));
    CHECK(back.edgeLayers.size() == 2);
    CHECK(back.edgeLayers[0] == 0);
    CHECK(back.edgeLayers[1] == 1);
}

// A ground road crossed by a layer-1 road builds a deck that rises well above grade.
TEST_CASE(layered_net_lifts_a_bridge_deck) {
    RoadNet net;
    net.nodes = { Vec2(-130, 0), Vec2(130, 0), Vec2(0, -130), Vec2(0, 130) };
    net.edges = { {0, 1}, {2, 3} };
    net.edgeLayers = { 0, 1 };
    net.width = 14.0; net.markings = false; net.crosswalks = false;
    RenderMesh m = buildRoadNetMesh(net);
    CHECK(!m.vertices.empty());
    double maxY = -1e30;
    for (const Vertex& v : m.vertices) maxY = std::max(maxY, (double)v.position.y);
    CHECK(maxY > 4.0);     // the bridge cleared the ground road (≈ lift + clearance + slab)
}

// With both roads on layer 0 they intersect at grade — no lifted deck.
TEST_CASE(same_layer_net_stays_flat) {
    RoadNet net;
    net.nodes = { Vec2(-130, 0), Vec2(130, 0), Vec2(0, -130), Vec2(0, 130) };
    net.edges = { {0, 1}, {2, 3} };
    net.edgeLayers = { 0, 0 };
    net.width = 14.0;
    RenderMesh m = buildRoadNetMesh(net);
    double maxY = -1e30;
    for (const Vertex& v : m.vertices) maxY = std::max(maxY, (double)v.position.y);
    CHECK(maxY < 2.0);     // everything sits near grade
}

// --- connected networks + a viaduct over several streets ---

namespace {
// Connected-component count over a graph's edges (isolated nodes ignored).
int componentCount(const RoadGraph& g) {
    int n = (int)g.nodes.size();
    std::vector<std::vector<int>> adj(n);
    for (const RoadEdge& e : g.edges) { adj[e.a].push_back(e.b); adj[e.b].push_back(e.a); }
    std::vector<char> seen(n, 0);
    int comps = 0;
    for (int s = 0; s < n; ++s) {
        if (seen[s] || adj[s].empty()) continue;
        ++comps;
        std::vector<int> st = { s }; seen[s] = 1;
        while (!st.empty()) {
            int v = st.back(); st.pop_back();
            for (int u : adj[v]) if (!seen[u]) { seen[u] = 1; st.push_back(u); }
        }
    }
    return comps;
}

// A 3x3 lattice of streets (connected) on layer 0, plus one elevated highway (layer 1).
RoadGraph gridPlusHighway() {
    RoadGraph g;
    int id[3][3];
    double xs[3] = { -80, 0, 80 }, zs[3] = { -80, 0, 80 };
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) id[i][j] = g.addNode(Vec2(xs[i], zs[j]));
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        if (j < 2) g.addEdge(id[i][j], id[i][j + 1], 13, RoadClass::Local);    // along z
        if (i < 2) g.addEdge(id[i][j], id[i + 1][j], 13, RoadClass::Local);    // along x
    }
    int h0 = g.addNode(Vec2(-150, 40)), h1 = g.addNode(Vec2(150, 40));
    g.edges.push_back({ h0, h1, 24, RoadClass::Arterial, 1 });                 // the viaduct
    return g;
}
}  // namespace

// The street grid is one connected component; the elevated highway is a separate one (it
// crosses the grid as grade separations, neither fragmenting nor merging it).
TEST_CASE(grid_stays_connected_under_a_viaduct) {
    RoadGraph g = gridPlusHighway();
    RoadGraph out = planarizeLayered(g);
    CHECK(componentCount(out) == 2);             // grid + highway, kept apart by layer
    CHECK(gradeSeparationCount(g) == 3);         // the highway flies over 3 cross-streets
}

// The same network as an editable RoadNet meshes with a lifted viaduct over the grid.
TEST_CASE(grid_overpass_net_lifts_a_viaduct) {
    RoadNet net;
    double xs[3] = { -80, 0, 80 }, zs[3] = { -80, 0, 80 };
    int id[3][3], k = 0;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) { net.nodes.push_back(Vec2(xs[i], zs[j])); id[i][j] = k++; }
    std::vector<int> layers;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        if (j < 2) { net.edges.push_back({ id[i][j], id[i][j + 1] }); layers.push_back(0); }
        if (i < 2) { net.edges.push_back({ id[i][j], id[i + 1][j] }); layers.push_back(0); }
    }
    int h0 = (int)net.nodes.size(); net.nodes.push_back(Vec2(-150, 40));
    int h1 = (int)net.nodes.size(); net.nodes.push_back(Vec2(150, 40));
    net.edges.push_back({ h0, h1 }); layers.push_back(1);
    net.edgeWidths.assign(net.edges.size(), 0.0); net.edgeWidths.back() = 24.0;
    net.edgeLayers = layers;
    net.width = 13.0; net.markings = true;
    RenderMesh m = buildRoadNetMesh(net);
    CHECK(!m.vertices.empty());
    double maxY = -1e30;
    for (const Vertex& v : m.vertices) maxY = std::max(maxY, (double)v.position.y);
    CHECK(maxY > 4.0);                            // the viaduct cleared the grid
}

// Regression: a crossing that lands exactly on a bridge sample vertex (a 300 m straight, so
// the midpoint sample sits on the crossing) is still detected and lifted.
TEST_CASE(bridge_crossing_on_a_vertex_still_lifts) {
    RoadNet net;
    net.nodes = { Vec2(-150, 0), Vec2(150, 0), Vec2(0, -150), Vec2(0, 150) };
    net.edges = { {0, 1}, {2, 3} };
    net.edgeLayers = { 0, 1 };
    net.width = 16.0; net.markings = false;
    RenderMesh m = buildRoadNetMesh(net);
    double maxY = -1e30;
    for (const Vertex& v : m.vertices) maxY = std::max(maxY, (double)v.position.y);
    CHECK(maxY > 4.0);
}

// Bridge piers: a column under the elevated sample reaches from ground up toward the deck.
TEST_CASE(bridge_piers_stand_under_the_deck) {
    std::vector<Vec2> center = { Vec2(-10, 0), Vec2(0, 0), Vec2(10, 0) };
    std::vector<double> deckY = { 6.0, 6.0, 6.0 };
    RenderMesh piers = bridgePiers(center, deckY, { 1 }, 8.0, 3.0, 0.8, Vec3(0.3, 0.3, 0.3));
    CHECK(!piers.vertices.empty());
    double maxY = -1e30, minY = 1e30;
    for (const Vertex& v : piers.vertices) { maxY = std::max(maxY, (double)v.position.y); minY = std::min(minY, (double)v.position.y); }
    CHECK(std::fabs(minY) < 1e-6);                 // foot on the ground
    CHECK(std::fabs(maxY - (6.0 - 0.8)) < 1e-6);   // head at the deck underside
}

// Deck barriers: a parapet wall stands above the deck on both edges.
TEST_CASE(deck_barriers_stand_on_the_verges) {
    std::vector<Vec2> c = { Vec2(-10, 0), Vec2(0, 0), Vec2(10, 0) };
    std::vector<double> y = { 5.0, 5.0, 5.0 };
    RenderMesh b = deckBarriers(c, y, 4.0, 0.9, Vec3(0.5, 0.5, 0.5));
    CHECK(!b.vertices.empty());
    double maxY = -1e30, minY = 1e30, maxAbsZ = 0;
    for (const Vertex& v : b.vertices) {
        maxY = std::max(maxY, (double)v.position.y); minY = std::min(minY, (double)v.position.y);
        maxAbsZ = std::max(maxAbsZ, std::fabs((double)v.position.z));
    }
    CHECK(std::fabs(minY - 5.0) < 1e-6);            // foot on the deck
    CHECK(std::fabs(maxY - 5.9) < 1e-6);            // top a barrier-height above
    CHECK(std::fabs(maxAbsZ - 4.0) < 1e-6);         // on the verges (±halfWidth)
}

// Deck markings: a wider deck gets more lane-divider stripes than a narrow one.
TEST_CASE(deck_markings_scale_lanes_with_width) {
    std::vector<Vec2> c = { Vec2(-50, 0), Vec2(0, 0), Vec2(50, 0) };
    std::vector<double> y = { 0, 0, 0 };
    DeckMarkParams mp;
    std::size_t narrow = deckMarkings(c, y, 4.0, mp).indices.size();    // ~2 lanes
    std::size_t wide   = deckMarkings(c, y, 12.0, mp).indices.size();   // ~7 lanes
    CHECK(narrow > 0);
    CHECK(wide > narrow);
}

// A tapering deck (per-point half-width) narrows along its length — the merge-gore primitive.
TEST_CASE(deck_tapers_with_per_point_width) {
    std::vector<Vec2> c;                                  // straight run along x
    std::vector<double> y, hw;
    for (int i = 0; i <= 10; ++i) { c.push_back(Vec2(i * 5.0, 0)); y.push_back(0.0); hw.push_back(0.5 + 0.45 * i); }
    RenderMesh d = bridgeDeck(c, y, hw, Vec3(0.1, 0.1, 0.1), 0.4);
    CHECK(!d.vertices.empty());
    // Vertices near the narrow end (x≈0) hug the centerline; near the wide end (x≈50) spread out.
    double zNarrow = 0, zWide = 0;
    for (const Vertex& v : d.vertices) {
        if (v.position.x < 3.0)  zNarrow = std::max(zNarrow, std::fabs((double)v.position.z));
        if (v.position.x > 47.0) zWide   = std::max(zWide,   std::fabs((double)v.position.z));
    }
    CHECK(std::fabs(zNarrow - 0.5) < 1e-6);              // half-width 0.5 at the nose
    CHECK(std::fabs(zWide - 5.0) < 1e-6);                // half-width 5.0 at the wide end
    CHECK(zWide > zNarrow);
}

// --- the design-rules registry (ADR-0052): one class-keyed source for road parameters ---
#include "../src/engine/procgen/city/road_rules.h"

TEST_CASE(design_rules_rank_classes_sensibly) {
    const DesignRules& d = defaultDesign();
    CHECK(d.minRadius(RoadClass::Freeway) > d.minRadius(RoadClass::Local));   // freeway curves gently
    CHECK(d.maxGrade(RoadClass::Freeway) < d.maxGrade(RoadClass::Local));     // and climbs gently
    CHECK(d.forClass(RoadClass::Freeway).dividedMedian);                      // freeway is divided
    CHECK(!d.forClass(RoadClass::Local).dividedMedian);
    CHECK(d.forClass(RoadClass::Ramp).lanes == 1);                            // a ramp is one lane
    CHECK(d.forClass(RoadClass::Freeway).fullWidth() > d.forClass(RoadClass::Local).fullWidth());
    CHECK(d.clearance > 4.0 && d.rampGrade > 0.0);                            // grade-sep numbers present
}

// The unified weld assembles ribbons into a solid mesh: a cross of two ribbons has area ~144.
TEST_CASE(weld_ribbons_makes_a_solid_cross) {
    std::vector<UnionSpine> sp;
    UnionSpine a; a.halfWidth = 2.0; a.points = { Vec2(-10, 0), Vec2(10, 0) }; sp.push_back(a);
    UnionSpine b; b.halfWidth = 2.0; b.points = { Vec2(0, -10), Vec2(0, 10) }; sp.push_back(b);
    RenderMesh m = weldRibbons(sp, 0.0, Vec3(0.1, 0.1, 0.1));
    CHECK(!m.vertices.empty());
    CHECK(m.indices.size() % 3 == 0);
    double area = 0;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vec3& A = m.vertices[m.indices[i]].position;
        const Vec3& B = m.vertices[m.indices[i + 1]].position;
        const Vec3& C = m.vertices[m.indices[i + 2]].position;
        area += std::fabs((B.x - A.x) * (C.z - A.z) - (C.x - A.x) * (B.z - A.z)) * 0.5;
    }
    CHECK(std::fabs(area - 144.0) < 0.5);            // welded cross area
}

// A "#" of four ribbons crossing THROUGH each other welds into a solid surface with its central
// block OPEN (the enclosed cell bridged out and triangulated away). Two H ribbons (y=±5), two V
// ribbons (x=±5), each 4 wide spanning [-15,15]: filled area = 4*120 - 4*16 = 416, central 6x6
// hole excluded. The triangulated mesh area is the filled 416 (the hole carries no triangles).
TEST_CASE(weld_ribbons_opens_a_block_hole) {
    auto mk = [](Vec2 a, Vec2 b) { UnionSpine s; s.halfWidth = 2.0; s.points = { a, b }; return s; };
    std::vector<UnionSpine> sp = {
        mk(Vec2(-15, -5), Vec2(15, -5)), mk(Vec2(-15, 5), Vec2(15, 5)),
        mk(Vec2(-5, -15), Vec2(-5, 15)), mk(Vec2(5, -15), Vec2(5, 15)) };
    RenderMesh m = weldRibbons(sp, 0.0, Vec3(0.1, 0.1, 0.1));
    double area = 0;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vec3& A = m.vertices[m.indices[i]].position;
        const Vec3& B = m.vertices[m.indices[i + 1]].position;
        const Vec3& C = m.vertices[m.indices[i + 2]].position;
        area += std::fabs((B.x - A.x) * (C.z - A.z) - (C.x - A.x) * (B.z - A.z)) * 0.5;
    }
    CHECK(std::fabs(area - 416.0) < 2.0);
}

// A corner radius fillets the welded outline at a junction: the cross's four reflex notches (the
// curb returns where perpendicular arms meet) round off, smoothing the sharp inner corners with
// arcs. The change is small and the arcs add geometry.
TEST_CASE(weld_corner_radius_fillets_junction) {
    auto mk = [](Vec2 a, Vec2 b) { UnionSpine s; s.halfWidth = 2.0; s.points = { a, b }; return s; };
    std::vector<UnionSpine> sp = { mk(Vec2(-10, 0), Vec2(10, 0)), mk(Vec2(0, -10), Vec2(0, 10)) };
    auto meshArea = [](const RenderMesh& m) {
        double a = 0;
        for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
            const Vec3& A = m.vertices[m.indices[i]].position;
            const Vec3& B = m.vertices[m.indices[i + 1]].position;
            const Vec3& C = m.vertices[m.indices[i + 2]].position;
            a += std::fabs((B.x - A.x) * (C.z - A.z) - (C.x - A.x) * (B.z - A.z)) * 0.5;
        }
        return a;
    };
    RenderMesh sharp = weldRibbons(sp, 0.0, Vec3(0.1, 0.1, 0.1), 0.0);
    RenderMesh round = weldRibbons(sp, 0.0, Vec3(0.1, 0.1, 0.1), 1.5);
    double as = meshArea(sharp), ar = meshArea(round);
    CHECK(std::fabs(as - 144.0) < 0.5);                 // sharp cross is exact
    CHECK(std::fabs(ar - as) < 6.0);                    // fillets nudge the area only a little
    CHECK(round.indices.size() > sharp.indices.size()); // arcs add triangles
}

// The solid extrude wraps the welded outline into a closed slab: a deck up top, an underside,
// and vertical side walls — so it has real thickness instead of a one-sided ribbon. A flat cross
// keeps the 144 deck area, mirrors it on the bottom, and the walls span the slab's full depth.
TEST_CASE(weld_solid_is_a_closed_slab) {
    auto mk = [](Vec2 a, Vec2 b) { UnionSpine s; s.halfWidth = 2.0; s.points = { a, b }; return s; };
    std::vector<UnionSpine> sp = { mk(Vec2(-10, 0), Vec2(10, 0)), mk(Vec2(0, -10), Vec2(0, 10)) };
    WeldSolidParams p;
    p.topY = 1.0; p.thickness = 0.5;
    RenderMesh m = weldSolid(sp, p);

    double deckY = -1e9, botY = 1e9;
    bool sawBottom = false, sawWall = false, sawStrip = false;
    for (const Vertex& v : m.vertices) {
        botY = std::min(botY, (double)v.position.y);
        if (v.normal.y > 0.9 && v.u < 0.5) deckY = std::max(deckY, (double)v.position.y); // plain deck
        else if (v.normal.y < -0.9) sawBottom = true;
        else if (std::fabs(v.normal.y) < 0.1) sawWall = true;   // horizontal-facing side wall
        if (v.u > 0.5) sawStrip = true;                          // painted marking strip
    }
    CHECK(sawBottom && sawWall && sawStrip);                     // underside, walls, and paint strip
    CHECK(std::fabs(deckY - 1.0) < 1e-6);                       // plain deck at topY
    CHECK(std::fabs(botY - 0.5) < 1e-6);                        // underside thickness below

    // The plain (unpainted) deck is laid as per-spine strips that overlap at the junction, so the
    // cross's two 20x4 = 80 strips sum to 160 (the 16 overlap counted by both); the marking strips
    // ride above and aren't part of this count.
    double deckArea = 0;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vertex& A = m.vertices[m.indices[i]];
        if (A.normal.y < 0.9 || A.u > 0.5) continue;            // only the plain deck
        const Vec3& a = A.position, &b = m.vertices[m.indices[i + 1]].position,
                    &c = m.vertices[m.indices[i + 2]].position;
        deckArea += std::fabs((b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z)) * 0.5;
    }
    CHECK(std::fabs(deckArea - 160.0) < 0.5);
}

// With a terrain function the deck rides a smoothed profile: a road over a single sharp bump
// follows the rise but irons the peak down, so the deck height stays between flat and raw.
TEST_CASE(weld_solid_smooths_a_terrain_bump) {
    auto mk = [](Vec2 a, Vec2 b) { UnionSpine s; s.halfWidth = 2.0; s.points = {};
        for (int i = 0; i <= 20; ++i) s.points.push_back(a + (b - a) * (i / 20.0)); return s; };
    std::vector<UnionSpine> sp = { mk(Vec2(-50, 0), Vec2(50, 0)) };
    WeldSolidParams p;
    p.thickness = 0.5; p.maxGrade = 0.08;
    p.heightAt = [](double x, double) { return std::fabs(x) < 4.0 ? 10.0 : 0.0; };  // a spike at x=0
    RenderMesh m = weldSolid(sp, p);
    double peak = -1e9;
    for (const Vertex& v : m.vertices) if (v.normal.y > 0.9) peak = std::max(peak, (double)v.position.y);
    CHECK(peak < 10.0);     // the sharp 10m spike is ironed down by the grade limit
    CHECK(peak > 0.5);      // but the road still rises toward it (not flattened away)
}

// Lane paint rides road-aligned strips with road-local UV for the RoadMarkings surface: the strip
// rails are exactly u=1 and u=3 (so the centerline u=2 spans across), v tracks arc-length to ~the
// road length, and the strip is finely subdivided (many quads). The plain welded deck and the side
// walls carry u=0 so no paint lands there.
TEST_CASE(weld_solid_bakes_road_local_uv) {
    auto mk = [](Vec2 a, Vec2 b) { UnionSpine s; s.halfWidth = 2.0; s.points = {};
        for (int i = 0; i <= 20; ++i) s.points.push_back(a + (b - a) * (i / 20.0)); return s; };
    std::vector<UnionSpine> sp = { mk(Vec2(-50, 0), Vec2(50, 0)) };   // a 100 m straight road
    RenderMesh m = weldSolid(sp, WeldSolidParams{});
    double maxV = 0, minStripU = 9, maxStripU = -9;
    int stripVerts = 0;
    bool allPaintFacesUp = true;
    for (const Vertex& v : m.vertices) {
        if (v.u > 0.5) {                                             // a painted strip vertex
            ++stripVerts;
            minStripU = std::min(minStripU, (double)v.u);
            maxStripU = std::max(maxStripU, (double)v.u);
            maxV = std::max(maxV, (double)v.v);
            if (v.normal.y < 0.9) allPaintFacesUp = false;           // paint only on the up-facing strip
        }
    }
    CHECK(std::fabs(minStripU - 1.0) < 1e-5);   // left rail exactly u=1
    CHECK(std::fabs(maxStripU - 3.0) < 1e-5);   // right rail exactly u=3 (centerline u=2 spans)
    CHECK(stripVerts > 60);                      // finely subdivided (100 m / ~3 m steps, 6 v/quad)
    CHECK(allPaintFacesUp);                      // walls/underside carry no paint UV
    CHECK(maxV > 90.0);                          // v tracks arc-length toward the 100 m length
}

// Junction pads close the wedge at OFF-SQUARE junctions (device: "a break in the
// geometry ... at a t-junction where the angles aren't square"). Every chain ends
// square to its OWN direction at a junction node, so when the through-road BENDS
// there, the two arm caps disagree by the bend angle and a knife-wedge of ground
// shows between them. WeldSolidParams::padCenters lays a disc per junction that
// must cover the wedge with up-facing deck.
TEST_CASE(weld_solid_junction_pad_fills_skewed_tee) {
    // A through road that bends ~20 degrees at the junction, plus a side arm: the
    // three chains all END at the node (0,0), each with a square cap.
    auto mk = [](Vec2 a, Vec2 b) {
        UnionSpine s; s.halfWidth = 3.5;
        for (int i = 0; i <= 10; ++i) s.points.push_back(a + (b - a) * (i / 10.0));
        return s;
    };
    Vec2 node(0, 0);
    std::vector<UnionSpine> sp = {
        mk(Vec2(-40, 0), node),                       // west arm
        mk(node, Vec2(40, 10)),                       // east arm, bent up ~14 deg
        mk(node, Vec2(5, 40)),                        // side arm, north
    };
    auto upCovered = [](const RenderMesh& m, const Vec2& p) {
        for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
            const Vertex& A = m.vertices[m.indices[i]];
            const Vertex& B = m.vertices[m.indices[i + 1]];
            const Vertex& C = m.vertices[m.indices[i + 2]];
            if (A.normal.y < 0.5 && B.normal.y < 0.5) continue;
            auto s = [&](Vec2 a, Vec2 b) {
                return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
            };
            Vec2 a(A.position.x, A.position.z), b(B.position.x, B.position.z),
                 c(C.position.x, C.position.z);
            double d1 = s(a, b), d2 = s(b, c), d3 = s(c, a);
            bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
            if (!(neg && pos)) return true;
        }
        return false;
    };
    // The wedge sits on the OUTSIDE of the through-road bend (below the node
    // here): the 14-degree sector between the west arm's cap plane (x = 0) and
    // the east arm's cap plane (0.970x + 0.243y = 0), within the arms' 3.5 m
    // half-width of the node — no arm's ribbon covers it.
    std::vector<Vec2> wedge = { Vec2(0.15, -3.0), Vec2(0.35, -3.2), Vec2(0.6, -3.3) };

    WeldSolidParams bare;                       // without pads: the wedge is open
    RenderMesh holey = weldSolid(sp, bare);
    int uncoveredBare = 0;
    for (const Vec2& p : wedge) if (!upCovered(holey, p)) ++uncoveredBare;
    CHECK(uncoveredBare > 0);                   // the bug reproduces without pads

    WeldSolidParams padded;                     // with the junction pad: covered
    padded.padCenters = { node };
    padded.padRadii = { 3.5 * 1.02 };
    RenderMesh solid = weldSolid(sp, padded);
    for (const Vec2& p : wedge) CHECK(upCovered(solid, p));
}

// Multilane roads (device: "do we still have multilane roads?"): a 13 m arterial
// (4 lanes) bakes dashed lane-DIVIDER strips at u = 4 between same-direction
// lanes, with v carrying raw arc-length for the dash phase. A 7 m street (2
// lanes, centre boundary only) bakes none — the double yellow is the only divide.
TEST_CASE(weld_solid_bakes_lane_dividers_on_arterials) {
    auto mk = [](Vec2 a, Vec2 b, double hw) {
        UnionSpine s; s.halfWidth = hw;
        for (int i = 0; i <= 20; ++i) s.points.push_back(a + (b - a) * (i / 20.0));
        return s;
    };
    RenderMesh wide = weldSolid({mk(Vec2(-60, 0), Vec2(60, 0), 6.5)}, WeldSolidParams{});
    int dividerVerts = 0;
    float maxV = 0;
    for (const Vertex& v : wide.vertices)
        if (v.u > 3.5f) {
            ++dividerVerts;
            maxV = std::max(maxV, v.v);
            CHECK(v.normal.y > 0.9);              // paint faces up
        }
    CHECK(dividerVerts > 0);                      // dividers exist on the arterial
    CHECK(maxV > 100.0f);                         // v tracks raw arc-length (120 m road)

    RenderMesh narrow = weldSolid({mk(Vec2(-60, 0), Vec2(60, 0), 3.5)}, WeldSolidParams{});
    for (const Vertex& v : narrow.vertices) CHECK(v.u <= 3.5f);   // streets: none
}
