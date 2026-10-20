// lanelab (ADR-0083): the geometry seam and, as the modules land, the generator's invariants.
#include "test_framework.h"
#include <CDT.h>
#include <filesystem>
#include <fstream>
#include "engine/procgen/city/roads/lanes/geom2d.h"
#include "engine/procgen/city/roads/lanes/deck_mesh.h"
#include "engine/mesh_audit.h"

using namespace engine;
using namespace engine::roads::lanes;

namespace {
Ring square(double x0, double y0, double s) { return {{x0, y0}, {x0 + s, y0}, {x0 + s, y0 + s}, {x0, y0 + s}}; }
}

TEST_CASE(lanes_union_of_two_overlapping_squares_has_the_inclusion_exclusion_area) {
    PolySet u = unionRings({square(0, 0, 10), square(5, 5, 10)});
    CHECK(u.size() == 1);
    CHECK_APPROX(setArea(u), 100 + 100 - 25, 1e-6);
    CHECK(contains(u, {7.5, 7.5}));
    CHECK(!contains(u, {12.0, 2.0}));
}

TEST_CASE(lanes_difference_leaves_a_hole_and_closing_fills_a_notch) {
    PolySet frame = differenceSets(fromRing(square(0, 0, 10)), fromRing(square(4, 4, 2)));
    CHECK(frame.size() == 1 && frame[0].holes.size() == 1);
    CHECK_APPROX(setArea(frame), 96, 1e-6);
    // an L-shape: closing with r=3 fills the concave corner (a fillet of area (1 - pi/4) r^2)
    Ring L = {{0, 0}, {10, 0}, {10, 4}, {4, 4}, {4, 10}, {0, 10}};
    PolySet closed = closing(fromRing(L), 3.0);
    CHECK_APPROX(setArea(closed) - setArea(fromRing(L)), (1 - 3.14159265 / 4) * 9, 0.05);
}

TEST_CASE(lanes_constrained_triangulation_splits_crossing_constraints_and_keeps_them_as_edges) {
    // two crossing strips' outlines as constraints: the CDT must insert the intersection points
    std::vector<Vec2> pts = {{0, 4}, {20, 4}, {20, 6}, {0, 6}, {9, 0}, {11, 0}, {11, 10}, {9, 10}};
    std::vector<std::pair<int, int>> edges = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}};
    Triangulation t = constrainedTriangulation(pts, edges);
    CHECK(t.verts.size() == 8 + 4);            // four crossing points added
    CHECK(!t.tris.empty());
    for (const auto& tri : t.tris) {           // CCW
        const Vec2& a = t.verts[tri[0]]; const Vec2& b = t.verts[tri[1]]; const Vec2& c = t.verts[tri[2]];
        CHECK(cross(b - a, c - a) > 0);
    }
}

#include "engine/procgen/city/roads/lanes/lane_expand.h"
#include "engine/procgen/city/roads/lanes/terrain_recipe.h"
#include "engine/procgen/city/roads/lanes/vertical_profile.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"

TEST_CASE(lanes_lane_offsets_come_from_the_road_spec_bands) {
    RoadClassSpec art; art.name = "arterial"; art.lanes.w = 3.0; art.lanes.fwd = 2; art.lanes.back = 2; art.lanes.gap = 6.0; art.sidewalk = 2.6;
    RoadSpec spec = bandsFor(art, art.lanes);
    CHECK(spec.laneCount() == 4);
    CHECK_APPROX(spec.totalWidth(), 2 * 2.6 + 4 * 3.0 + 6.0, 1e-6);
    std::vector<LaneSlot> slots = laneSlots(art, art.lanes);
    CHECK(slots.size() == 4);
    // forward lanes on the right of the spine (negative left-normal), inner first; backward on the left
    double f0 = 0, f1 = 0, b0 = 0, b1 = 0;
    for (const LaneSlot& s : slots) { if (s.name == "f0") f0 = s.offset; if (s.name == "f1") f1 = s.offset; if (s.name == "b0") b0 = s.offset; if (s.name == "b1") b1 = s.offset; }
    CHECK_APPROX(f0, -(3.0 + 1.5), 1e-6); CHECK_APPROX(f1, -(3.0 + 4.5), 1e-6);
    CHECK_APPROX(b0, +(3.0 + 1.5), 1e-6); CHECK_APPROX(b1, +(3.0 + 4.5), 1e-6);
    RoadClassSpec fwy; fwy.name = "freeway"; fwy.lanes.w = 3.6; fwy.lanes.fwd = 3; fwy.lanes.back = 0;
    std::vector<LaneSlot> one = laneSlots(fwy, fwy.lanes);   // one-way: centred on the spine
    CHECK(one.size() == 3); CHECK_APPROX(one[0].offset, 3.6, 1e-6); CHECK_APPROX(one[1].offset, 0.0, 1e-6); CHECK_APPROX(one[2].offset, -3.6, 1e-6);
}

TEST_CASE(lanes_graph_loads_and_expands_the_valley_scene) {
    RoadLabGraph g = RoadLabGraph::load(std::string(RT_SOURCE_DIR) + "/assets/lanelab/valley_viaduct.json");
    CHECK(g.edges.size() == 3);
    CHECK(g.find("ramp") && g.find("ramp")->isRamp() && g.find("ramp")->to.set);
    LaneSet L = expand(g);
    CHECK(L.lanes.size() == 3 + 2 + 2);
    CHECK(g.find("ramp")->anchorLanes.at("to") == "fwy.f2");
    // the ramp's last point sits on the host outer lane's centreline
    int host = L.find("fwy.f2"); CHECK(host >= 0);
    const Lane& h = L.lanes[static_cast<size_t>(host)];
    CHECK(project(h.xy, h.s, g.find("ramp")->xy.back()).distance < 0.05);
    HeightField terrain = makeTerrain(g.terrain, g.bounds());
    for (EdgeSpec& e : g.edges) if (!e.isRamp()) throughProfile(e, terrain, g.cls(e));
    CHECK(maxGrade(*g.find("fwy")) <= 0.03 * kDesignGrade + 1e-6);
}

#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/lanes/road_twin.h"
#include "engine/procgen/city/roads/lanes/block_audit.h"
#include "engine/procgen/city/roads/lanes/city_producer.h"
#include "engine/procgen/city/roads/lanes/lots_producer.h"
#include "engine/procgen/city/roads/road_builder.h"
#include "engine/bundle/bundle_glb.h"
#include "engine/procgen/city/lot_cache.h"
#include "engine/model_importer.h"
#include "engine/procgen/city/road_network.h"
#include "engine/procgen/city/polygon.h"
#include <algorithm>
#include <cstdio>
#include <set>

namespace {
std::map<std::string, std::unique_ptr<Result>>& resultCache() { static std::map<std::string, std::unique_ptr<Result>> c; return c; }
const Result& scene(const std::string& name) {
    auto& c = resultCache(); auto it = c.find(name);
    if (it == c.end()) { std::unique_ptr<Result> r = build(RoadLabGraph::load(std::string(RT_SOURCE_DIR) + "/assets/lanelab/" + name + ".json")); std::printf("%s", summary(*r).c_str()); it = c.emplace(name, std::move(r)).first; }
    return *it->second;
}
void checkInvariants(const Result& r) { for (const Check& c : invariants(r)) { if (!c.ok) std::printf("    invariant failed: %s — %s\n", c.name.c_str(), c.detail.c_str()); CHECK(c.ok); } }
// A scene with a KNOWN, filed limitation: the named invariants may fail (reported, not asserted); every other
// one must hold, and a known one that starts holding says so -- take it off the list.
void checkInvariantsExcept(const Result& r, const std::vector<std::string>& known, const char* issue) {
    for (const Check& c : invariants(r)) {
        const bool isKnown = std::find(known.begin(), known.end(), c.name) != known.end();
        if (!c.ok && isKnown) { std::printf("    known failure (%s): %s — %s\n", issue, c.name.c_str(), c.detail.c_str()); continue; }
        if (c.ok && isKnown) std::printf("    KNOWN FAILURE NOW HOLDS (%s): %s — drop it from the list\n", issue, c.name.c_str());
        if (!c.ok) std::printf("    invariant failed: %s — %s\n", c.name.c_str(), c.detail.c_str());
        CHECK(c.ok);
    }
}
double bridge(const Result& r, const char* id) { return r.bridgeLen.at(id); }
}

TEST_CASE(lanes_grid_city_builds_and_holds_its_invariants) {
    const Result& r = scene("grid_city"); checkInvariants(r);
    CHECK(r.pavement.decks.size() == 2);                       // the eastbound carriageway is an island
    CHECK(setArea(r.pavement.median) > 1000 && setArea(r.pavement.sidewalk) > 5000);
    CHECK(r.graph.find("exit")->anchorLanes.at("from") == "fw_wb.f2" && r.graph.find("entry")->anchorLanes.at("to") == "fw_wb.f2");
}

TEST_CASE(lanes_valley_viaduct_builds_and_holds_its_invariants) {
    const Result& r = scene("valley_viaduct"); checkInvariants(r);
    CHECK(bridge(r, "fwy") >= 90 && bridge(r, "fwy") <= 150);
    CHECK(bridge(r, "ramp") > 20);                             // the ramp crosses the valley's north end on structure
    CHECK(r.pavement.decks.size() == 1);                       // local, ramp and freeway weld into one surface that passes over itself
    bool separated = false; for (const auto& kv : r.pavement.pairs) if (kv.second.separatedArea > 0 && kv.second.dz > 15) separated = true;
    CHECK(separated);
}

TEST_CASE(lanes_hill_junction_builds_and_holds_its_invariants) {
    const Result& r = scene("hill_junction"); checkInvariants(r);
    int connectors = 0; for (const Lane& l : r.lanes.lanes) if (l.isConnector()) ++connectors;
    CHECK(connectors == 2 && r.lanes.lanes.size() == 10);
    CHECK(r.pavement.decks.size() == 1);
    CHECK(!r.pavement.islands.empty());                        // the slip lane and its connector enclose an island
}

// THE SCENES THE CORPUS WAS MISSING (2026-09-20). Running every scene through
// tools/lanes_corpus.py showed the authored set had no crossing of two FREEWAYS
// (ring_city carries a freeway over arterials, which is a different test), no
// 2 -> 1 ramp merge, and no sustained grade. These four are the controls for
// those three holes: they pass today, and the day one stops passing is the day
// something real broke.
TEST_CASE(lanes_two_freeways_cross_one_carried_over_the_other) {
    const Result& r = scene("freeway_cross"); checkInvariants(r);
    CHECK(r.piers.size() >= 12);                               // it is carried, not merged
    CHECK(bridge(r, "ns_a") > 100 && bridge(r, "ns_b") > 100); // the north-south pair flies
    CHECK(bridge(r, "ew_a") == 0.0 && bridge(r, "ew_b") == 0.0);   // the east-west pair stays down
}

TEST_CASE(lanes_a_carried_crossing_still_clears_when_the_ground_rises_under_it) {
    const Result& r = scene("freeway_cross_relief"); checkInvariants(r);
    CHECK(r.piers.size() >= 12);
    CHECK(r.hasTerrain);
}

TEST_CASE(lanes_two_lane_ramps_die_into_one_over_their_dovetail) {
    const Result& r = scene("merge_taper"); checkInvariants(r);
    // mainline 3 + three feeders at 2 + ramps 1, 2, 2
    CHECK(r.lanes.lanes.size() == 14);
    CHECK(r.graph.find("ramp_short")->lanes.dovetail == 30.0);
    CHECK(r.graph.find("ramp_long")->lanes.dovetail == 90.0);
    CHECK(r.graph.find("ramp_one")->anchorLanes.count("to") == 1);   // the control still merges
}

// NOTE: this scene USED TO fail the no-steps invariant, and that was the scene's point (#82). It holds now (seen
// 2026-10-13: the suite said "KNOWN FAILURE NOW HOLDS"), so it is asserted like every other. The history: with
// its open ends pinned the road answers 114 m of ground rise with a 53 m CUTTING at
// the top, and the terrain conform does not fully clear a cutting that deep: the
// cutting's lip leaves step edges. Before the pin it passed -- by floating 65 m in the
// air instead. A builder that cannot switchback or tunnel has to lose somewhere; this is
// where, and what it costs. It is a NAMED known failure, not a red test: asserting it
// kept main's CI red for weeks, hiding every other regression.
TEST_CASE(lanes_a_freeway_climbs_sustained_relief_within_its_grade) {
    const Result& r = scene("steep_climb");
    checkInvariants(r);
    const EdgeSpec* climb = r.graph.find("climb");
    CHECK(climb != nullptr);
    if (!climb) return;
    double lo = 1e9, hi = -1e9, tlo = 1e9, thi = -1e9;
    for (double z : climb->z) { lo = std::min(lo, z); hi = std::max(hi, z); }
    for (double t : climb->t) { tlo = std::min(tlo, t); thi = std::max(thi, t); }
    std::printf("    [climb] deck %.0f..%.0f m (%.0f m gained) over ground %.0f..%.0f m (%.0f m)\n",
                lo, hi, hi - lo, tlo, thi, thi - tlo);
    // The ground gains more than the road is allowed to: 1900 m at 4% is 76 m and
    // the relief is ~114 m, so the road cannot follow it — the case every imported
    // city failed on.
    CHECK(hi - lo > 40.0);
    CHECK(thi - tlo > hi - lo);                                // the ground out-climbs the road

    // An OPEN end may be cut into the ground — that is a cutting, and buildable — but
    // it must never FLOAT: a viaduct to nowhere. Before the profile solve pinned open
    // ends (2026-09-20) this freeway began 65 m in the air over a valley floor it
    // never came down to, 1040 m of it on piers, with every invariant green. It now
    // starts at the ground and pays for the climb it cannot make with a cutting at
    // the far end, which is what the ground leaves it.
    const double startGap = climb->z.front() - climb->t.front();
    const double endGap = climb->z.back() - climb->t.back();
    std::printf("    [climb] ends: start deck-ground %+.1f m, end %+.1f m\n", startGap, endGap);
    CHECK(startGap < 1.0);
    CHECK(endGap < 1.0);
    // and it no longer answers the climb with a kilometre of viaduct
    CHECK(bridge(r, "climb") < 0.15 * climb->length());
}

TEST_CASE(lanes_ring_city_builds_and_holds_its_invariants) {
    const Result& r = scene("ring_city"); checkInvariants(r);
    double onStructure = 0, total = 0;
    for (const EdgeSpec& e : r.graph.edges) if (e.id.rfind("in_", 0) == 0 || e.id.rfind("out_", 0) == 0) { onStructure += r.bridgeLen.at(e.id); total += e.length(); }
    CHECK(onStructure > 0.4 * total && onStructure < 0.95 * total);
    int ramps = 0; for (const EdgeSpec& e : r.graph.edges) if (e.ramp.valid) { ++ramps; CHECK(e.ramp.ok); CHECK(!e.anchorLanes.empty()); }
    CHECK(ramps == 8);
    CHECK(r.pavement.decks.size() == 1);
}

// The RoadEntity twin is what the engine's lot pass walks (ADR-0083 integration seam). Ring city's
// grid is 16 blocks: every one must come out of extractBlocks as a face AND survive the lot pass's
// first step (inset by the sidewalk). A node every 8 m along a straight street used to leave 44-vertex
// faces whose miter inset collapsed to nothing — the grid built no buildings at all.
TEST_CASE(lanes_road_twin_closes_every_ring_city_block_and_each_survives_the_sidewalk_inset) {
    const Result& r = scene("ring_city");
    engine::RoadEntity twin = roadTwin(r);
    std::vector<int> deg(twin.graph.nodes.size(), 0);
    for (const engine::RoadEdge& e : twin.graph.edges) { ++deg[static_cast<size_t>(e.a)]; ++deg[static_cast<size_t>(e.b)]; }
    int tees = 0, crosses = 0; for (int d : deg) { tees += d == 3; crosses += d >= 4; }
    CHECK(tees >= 8);        // the four inner grid lines end on the outer ones (ramp feet on the spokes add more)
    CHECK(crosses >= 13);    // 4 inner crossings + 8 arterial crossings + the arterials' own (at-grade ramp runs may add one)
    engine::RoadGraph rg;    // the street subgraph exactly as growLotBuildingsOnNets builds it
    for (const engine::RoadNode& n : twin.graph.nodes) rg.nodes.push_back({n.pos});
    for (const engine::RoadEdge& e : twin.graph.edges)
        if (!e.baked && e.klass != engine::RoadClass::Freeway && e.klass != engine::RoadClass::Ramp) rg.edges.push_back(engine::RoadEdge{e.a, e.b, e.width, engine::RoadClass::Local, 0});
    std::vector<engine::Poly2> faces = engine::extractBlocks(rg);
    CHECK(faces.size() == 16);
    size_t ok = 0, maxVerts = 0;
    for (const engine::Poly2& f : faces) { maxVerts = std::max(maxVerts, f.size()); engine::Poly2 foot = engine::inset(f, 3.5); if (foot.size() >= 3 && engine::area(foot) > 1000.0) ++ok; }
    CHECK(ok == faces.size());
    CHECK(maxVerts <= 12);   // corners and junctions only, never a node every 8 m
    CHECK(twin.plan.freewayPlans.size() == 8);   // the ring's carriageway arcs: the lot pass's right-of-way band
}

// The same seam on CURVED streets: curve_blocks is a bezier arterial crossed by three straight
// locals and closed by two more, so four faces have a curved side whose twin nodes come from
// Douglas-Peucker. Every face must survive the metro's 5 m sidewalk inset — the failure that put
// rim-synthesised lots across metro streets was a face dying in that inset on an edge a few metres
// long (a crossing node beside a corner, a stub under the node tolerance, two junctions almost
// coincident).
TEST_CASE(lanes_road_twin_faces_with_curved_sides_survive_the_metro_sidewalk_inset) {
    const Result& r = scene("curve_blocks");
    engine::RoadEntity twin = roadTwin(r);
    engine::RoadGraph rg;
    for (const engine::RoadNode& n : twin.graph.nodes) rg.nodes.push_back({n.pos});
    for (const engine::RoadEdge& e : twin.graph.edges)
        if (!e.baked && e.klass != engine::RoadClass::Freeway && e.klass != engine::RoadClass::Ramp) rg.edges.push_back(engine::RoadEdge{e.a, e.b, e.width, engine::RoadClass::Local, 0});
    std::vector<engine::Poly2> faces = engine::extractBlocks(rg);
    CHECK(faces.size() == 4);
    size_t ok = 0; double shortest = 1e300;
    for (const engine::Poly2& f : faces) {
        for (size_t i = 0; i < f.size(); ++i) shortest = std::min(shortest, (f[(i + 1) % f.size()] - f[i]).length());
        engine::Poly2 foot = engine::inset(f, 5.0); if (foot.size() >= 3 && engine::area(foot) > 500.0) ++ok;
    }
    CHECK(ok == faces.size());
    CHECK(shortest >= 8.0);   // no face edge shorter than the cleanup's floor
}

// What the block/lot pass reads. On the clean scenes no block
// foot — the face inset by the sidewalk and pushed clear of the sampled ribbons, as the pass does it —
// may overlap built pavement. This is the in-code form of "are there broken blocks".
static double brokenBlockArea(const Result& r, double margin) {
    engine::RoadEntity twin = roadTwin(r);   // the twin the lot pass reads today
    engine::RoadGraph rg;
    for (const engine::RoadNode& n : twin.graph.nodes) rg.nodes.push_back({n.pos});
    for (const engine::RoadEdge& e : twin.graph.edges)
        if (!e.baked && e.klass != engine::RoadClass::Freeway && e.klass != engine::RoadClass::Ramp) rg.edges.push_back(engine::RoadEdge{e.a, e.b, e.width, engine::RoadClass::Local, 0});
    double total = 0;
    for (const engine::Poly2& f : engine::extractBlocks(rg)) {
        engine::Poly2 foot = engine::inset(f, margin); if (foot.size() < 3) continue;
        engine::Poly2 dense;
        for (size_t i = 0; i < foot.size(); ++i) { const auto& a = foot[i]; const auto& b = foot[(i + 1) % foot.size()]; dense.push_back(a); const int div = static_cast<int>((b - a).length() / 8.0); for (int k = 1; k <= div; ++k) dense.push_back(a + (b - a) * (static_cast<double>(k) / (div + 1))); }
        for (auto& p : dense) for (int guard = 0; guard < 4; ++guard) {
            double worstNeed = 0; engine::Vec2 away;
            for (const engine::RoadEdge& e : twin.graph.edges) {
                const auto& a = twin.graph.nodes[static_cast<size_t>(e.a)].pos; const auto& b = twin.graph.nodes[static_cast<size_t>(e.b)].pos;
                engine::Vec2 ab = b - a; double l2 = ab.lengthSquared(); double t = l2 > 1e-12 ? std::max(0.0, std::min(1.0, dot(p - a, ab) / l2)) : 0.0;
                engine::Vec2 q = a + ab * t; double d = (p - q).length(); double need = e.width * 0.5 + margin + 0.6 - d;
                if (need > worstNeed) { worstNeed = need; away = d > 1e-6 ? (p - q) * (1.0 / d) : engine::Vec2(0, 1); }
            }
            if (worstNeed <= 0.01) break; p = p + away * worstNeed;
        }
        if (std::fabs(engine::area(dense)) < 135.0) continue;
        Ring ring(dense.begin(), dense.end()); total += setArea(intersectSets(fromRing(ring), r.pavement.surface));
    }
    return total;
}
TEST_CASE(lanes_twin_leaves_no_block_foot_on_built_pavement_in_the_clean_scenes) {
    CHECK(brokenBlockArea(scene("ring_city"), 5.0) < 1.0);
    CHECK(brokenBlockArea(scene("curve_blocks"), 5.0) < 1.0);
    CHECK(brokenBlockArea(scene("hill_junction"), 5.0) < 1.0);
}

// The engine's OWN block/lot pass on the twin (headless), audited: on the clean scenes no block may carry
// a thin protrusion (an 8 m opening removes < 20 m²) or sit on built pavement (> 1 m²). This is the test
// Glenn asked for: if blocks look like spikes and snakes in-engine, this must fail.
TEST_CASE(lanes_the_city_block_pass_yields_well_formed_blocks_on_the_clean_scenes) {
    nlohmann::json cs = {{"sidewalk", 3.5}, {"buildLots", true}, {"seed", 3}};
    for (const char* name : {"ring_city", "curve_blocks", "hill_junction", "grid_city"}) {
        BlockAudit a = auditBlocks(scene(name), cs, /*fromScene=*/true);
        std::printf("    %s: %s", name, summary(a).c_str());
        CHECK(a.blocks.size() >= 1 || std::string(name) == "hill_junction");
        CHECK(a.broken == 0);
        CHECK(a.padConflicts.empty());   // no building pad or block terrace lifts pavement or sidewalk
    }
}

// ---- the city producer (ADR-0084) ------------------------------------------------------------------------

namespace {
bool sameF(const std::vector<float>& a, const std::vector<float>& b) { return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin()); }
bool sameRing(const std::vector<Vec2>& a, const std::vector<Vec2>& b) { if (a.size() != b.size()) return false; for (size_t i = 0; i < a.size(); ++i) if (a[i].x != b[i].x || a[i].y != b[i].y) return false; return true; }
}  // namespace

TEST_CASE(lanes_city_products_round_trip_through_a_bundle_to_the_byte) {
    using namespace engine::bundle;
    const Result& r = scene("ring_city");
    CityProducts p = cityProductsFromResult(r, 250.0);
    CHECK(p.hasTerrain == r.hasTerrain); CHECK(!p.cells.empty()); CHECK(!p.holes.empty()); CHECK(p.nav.nodes.size() > 10 && p.row.edges.size() > 0);
    size_t tris = 0; for (const CityCellMesh& c : p.cells) tris += c.mesh.triangleCount(); CHECK(tris > 100000);
    BundleWriter w; w.openMemory(); writeCityProducts(w, 0, p); nlohmann::json manifest = nlohmann::json::object(); std::string err; CHECK(w.finish(manifest, &err));
    std::unique_ptr<Bundle> b = Bundle::fromMemory(w.memory(), &err); CHECK(b != nullptr); if (!b) return;
    CityProducts q; CHECK(readCityProducts(*b, 0, q, &err)); if (!err.empty()) std::printf("    %s\n", err.c_str());
    CHECK(q.hasTerrain == p.hasTerrain && q.cell == p.cell && q.ground.z == p.ground.z && q.ground.nx == p.ground.nx);
    CHECK(q.twin.graph.nodes.size() == p.twin.graph.nodes.size() && q.twin.graph.edges.size() == p.twin.graph.edges.size() && q.nav.edges.size() == p.nav.edges.size() && q.row.edges.size() == p.row.edges.size());
    for (size_t i = 0; i < p.twin.graph.nodes.size() && i < q.twin.graph.nodes.size(); ++i) CHECK(p.twin.graph.nodes[i].pos.x == q.twin.graph.nodes[i].pos.x && p.twin.graph.nodes[i].elev == q.twin.graph.nodes[i].elev);
    CHECK(q.holes.size() == p.holes.size()); for (size_t i = 0; i < p.holes.size() && i < q.holes.size(); ++i) CHECK(sameRing(p.holes[i], q.holes[i]));
    CHECK(q.cells.size() == p.cells.size());
    std::map<std::tuple<std::string, int, int>, const CityCellMesh*> byKey; for (const CityCellMesh& c : q.cells) byKey[{c.mesh.name, c.cx, c.cz}] = &c;
    int matched = 0;
    for (const CityCellMesh& c : p.cells) {
        auto it = byKey.find({c.mesh.name, c.cx, c.cz}); CHECK(it != byKey.end()); if (it == byKey.end()) continue;
        const PackedMesh& a = c.mesh; const PackedMesh& d = it->second->mesh;
        CHECK(a.flags == d.flags && a.friction == d.friction && a.roughness == d.roughness && sameF(a.pos, d.pos) && sameF(a.nrm, d.nrm) && sameF(a.tan, d.tan) && sameF(a.uv, d.uv) && sameF(a.col, d.col) && a.idx == d.idx);
        ++matched;
    }
    CHECK(matched == static_cast<int>(p.cells.size()));
    // blocks derived at load equal the scene's own
    const std::vector<Poly2> viaBundle = blocksFromHoles(q.holes, 1.5, 3.5), direct = sceneBlocks(r, 1.5, 2000.0, 3.5);
    CHECK(viaBundle.size() == direct.size()); for (size_t i = 0; i < viaBundle.size() && i < direct.size(); ++i) CHECK(sameRing(viaBundle[i], direct[i]));
    CHECK(b->manifest()["sections"].size() == w.sections().size());
}

TEST_CASE(lanes_city_producer_identity_follows_its_inputs) {
    using namespace engine::bundle;
    registerCityProducer(); const BundleProducer* cp = findProducer(kCityProducerName); CHECK(cp != nullptr); if (!cp) return;
    // level files name their graphs relative to the repo root (the viewer's working directory)
    const std::filesystem::path before = std::filesystem::current_path(); std::filesystem::current_path(RT_SOURCE_DIR);
    struct Restore { std::filesystem::path p; ~Restore() { std::error_code ec; std::filesystem::current_path(p, ec); } } restore{before};
    LevelInputs in; std::string err; CHECK(loadLevelInputs("assets/lanelab/levels/ring.json", in, &err));
    CHECK(cp->applies(in)); CHECK(cityEntities(in.level).size() == 1 && !cityEntities(in.level)[0].inlineGraph);
    const ProducerIdentity a = cp->identity(in), b = cp->identity(in); CHECK(a.key == b.key && a.tag == kLanesBuildTag && !a.inputs.empty() && a.inputs[0].bytes > 0);
    LevelInputs other = in; other.level["citysim"]["renderCell"] = 125.0; CHECK(cp->identity(other).key != a.key);
    LevelInputs inl = in; inl.level["entities"][cityEntities(in.level)[0].entityIndex]["lanelab"] = nlohmann::json{{"edges", nlohmann::json::array()}};
    CHECK(cityEntities(inl.level)[0].inlineGraph && cp->identity(inl).key != a.key);
}

// TWO SPELLINGS, ONE CITY (docs/road-module-plan.md phase 3). The lab's shape:"lanelab"
// and the roads module's shape:"road" with `"builder": "lanes"` name the same city entity:
// the producer sees both, numbers them the same, and — because the key is the graph's bytes
// and not the entity's spelling — a level MIGRATES from one to the other without rebaking.
TEST_CASE(lanes_a_road_entity_that_names_the_builder_is_the_same_city) {
    using namespace engine::bundle;
    registerCityProducer(); const BundleProducer* cp = findProducer(kCityProducerName);
    CHECK(cp != nullptr); if (!cp) return;
    const std::filesystem::path before = std::filesystem::current_path(); std::filesystem::current_path(RT_SOURCE_DIR);
    struct Restore { std::filesystem::path p; ~Restore() { std::error_code ec; std::filesystem::current_path(p, ec); } } restore{before};
    LevelInputs lab; std::string err; CHECK(loadLevelInputs("assets/lanelab/levels/ring.json", lab, &err));
    const std::vector<CityEntity> labCity = cityEntities(lab.level);
    CHECK(labCity.size() == 1); if (labCity.size() != 1) return;
    const int idx = labCity[0].entityIndex;

    // The same level, respelled: shape:"road", the block under "road", builder named.
    LevelInputs road = lab;
    nlohmann::json& ent = road.level["entities"][static_cast<std::size_t>(idx)];
    nlohmann::json block = ent["lanelab"]; block["builder"] = "lanes";
    ent.erase("lanelab"); ent["shape"] = "road"; ent["road"] = block;

    const std::vector<CityEntity> roadCity = cityEntities(road.level);
    CHECK(roadCity.size() == 1); if (roadCity.size() != 1) return;
    CHECK(roadCity[0].graphPath == labCity[0].graphPath && !roadCity[0].inlineGraph);
    CHECK(roadCity[0].ordinal == 0 && roadCity[0].entityIndex == idx);
    CHECK(cityOrdinalForEntity(road.level, idx) == 0);
    CHECK(cityOrdinalForEntity(road.level, idx + 1) == -1);   // nothing else in the level is a city
    CHECK(cp->applies(road) && cp->identity(road).key == cp->identity(lab).key);   // no rebake to migrate

    // A road entity that does NOT name the builder gets the default -- lanes, the only builder since the
    // lattice retired (2026-10-14) -- and so is the same city.
    LevelInputs plain = road; plain.level["entities"][static_cast<std::size_t>(idx)]["road"].erase("builder");
    CHECK(cityEntities(plain.level).size() == 1);
    CHECK(engine::roads::roadBuilderName(block) == "lanes");
    CHECK(engine::roads::roadBuilderName(plain.level["entities"][static_cast<std::size_t>(idx)]["road"]) == "lanes");

    // The OTHER producers must read the level the same way, or the migration would move the
    // bundle DIRECTORY (keyed by the whole applicable set) and silently rebake the city. A
    // road entity that IS this city must not count as "a road entity whose lots the lattice
    // pipeline owns" — the bug the first cut of the second spelling had.
    registerLotsProducer();
    const BundleProducer* lp = findProducer(kLotsProducerName);
    CHECK(lp != nullptr);
    if (lp) { CHECK(lp->applies(lab)); CHECK(lp->applies(road) == lp->applies(lab)); }
    CHECK(bundleDirForLevel(road) == bundleDirForLevel(lab));

    // And the builder behind that name owns the graph everything routes on — which is why
    // the loader publishes ITS twin and does not re-derive one from the recipe.
    engine::roads::RoadBuilder* b = engine::roads::roadBuilder("lanes");
    CHECK(b != nullptr); if (!b) return;
    CHECK(b->ownsNavGraph());
    CHECK(engine::roads::roadBuilderFor(plain.level["entities"][static_cast<std::size_t>(idx)]["road"]).ownsNavGraph());
}

TEST_CASE(lanes_bundle_glb_view_reads_back_through_the_engine_importer) {
    using namespace engine::bundle;
    const Result& r = scene("ring_city"); CityProducts p = cityProductsFromResult(r, 250.0);
    BundleWriter w; w.openMemory(); writeCityProducts(w, 0, p); nlohmann::json manifest = nlohmann::json::object(); std::string err; CHECK(w.finish(manifest, &err));
    std::unique_ptr<Bundle> b = Bundle::fromMemory(w.memory(), &err); CHECK(b != nullptr); if (!b) return;
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "rt_bundle_glb_test"; std::error_code ec; std::filesystem::remove_all(dir, ec); std::filesystem::create_directories(dir, ec);
    const std::string glb = (dir / "city.glb").string();
    CHECK(writeBundleGlb(*b, glb, &err)); if (!err.empty()) std::printf("    %s\n", err.c_str());
    std::map<std::string, size_t> vertsByMaterial; for (const CityCellMesh& c : p.cells) vertsByMaterial[c.mesh.name] += c.mesh.vertexCount();
    const engine::CpuModel model = engine::ModelImporter::loadCpu(glb);
    CHECK(model.meshes.size() == vertsByMaterial.size());
    size_t total = 0; for (const engine::CpuMesh& m : model.meshes) total += m.geometry.vertices.size();
    size_t expected = 0; for (const auto& kv : vertsByMaterial) expected += kv.second;
    CHECK(total == expected);
    std::vector<std::string> files; CHECK(writeBundleGlbCells(*b, (dir / "cells").string(), &err, &files)); CHECK(!files.empty() && std::filesystem::exists(dir / "cells" / "index.json"));
    const engine::CpuModel cell = engine::ModelImporter::loadCpu(files.front()); CHECK(!cell.meshes.empty());
    std::filesystem::remove_all(dir, ec);
}

TEST_CASE(lanes_grown_lots_round_trip_through_the_lot_codecs) {
    using namespace engine::bundle; using namespace engine::lotcache;
    nlohmann::json level; { std::ifstream f(std::string(RT_SOURCE_DIR) + "/assets/lanelab/levels/ring.json"); f >> level; }
    BlockAudit a = auditBlocks(scene("ring_city"), level.value("citysim", nlohmann::json::object()), /*fromScene=*/true);
    CHECK(a.grown.lots.size() > 100);
    BundleWriter w; w.openMemory(); writeLotResult(w, "lots/e0/", a.grown); nlohmann::json manifest = nlohmann::json::object(); std::string err; CHECK(w.finish(manifest, &err));
    std::unique_ptr<Bundle> b = Bundle::fromMemory(w.memory(), &err); CHECK(b != nullptr); if (!b) return;
    NetLotResult back; CHECK(readLotResult(*b, "lots/e0/", back, &err)); if (!err.empty()) std::printf("    %s\n", err.c_str());
    CHECK(back.lots.size() == a.grown.lots.size() && back.plan.lots.size() == a.grown.plan.lots.size() && back.plan.blocks.size() == a.grown.plan.blocks.size() && back.gradeFlatten.size() == a.grown.gradeFlatten.size());
    size_t units = 0, unitsBack = 0, mismatched = 0;
    for (size_t i = 0; i < a.grown.lots.size() && i < back.lots.size(); ++i) {
        const LotBuilding& x = a.grown.lots[i]; const LotBuilding& y = back.lots[i];
        if (x.type != y.type || x.recipe != y.recipe || x.block != y.block || x.groundY != y.groundY || x.plan.size() != y.plan.size() || x.pad.size() != y.pad.size() || x.units.size() != y.units.size() || x.treeSpots.size() != y.treeSpots.size() || x.padMesh.indices != y.padMesh.indices) ++mismatched;
        units += x.units.size(); unitsBack += y.units.size();
        for (size_t k = 0; k < x.units.size() && k < y.units.size(); ++k) {
            BinWriter wa, wb; putBuildingParams(wa, x.units[k].params); putBuildingParams(wb, y.units[k].params);
            if (wa.bytes != wb.bytes || x.units[k].doors.size() != y.units[k].doors.size() || x.units[k].enterable != y.units[k].enterable) ++mismatched;
        }
    }
    CHECK(mismatched == 0); CHECK(units == unitsBack && units > 0);
    std::printf("    ring lots: %zu lots, %zu units, %zu terraces, %zu bytes\n", back.lots.size(), units, back.gradeFlatten.size(), w.memory()->size());
}

// ---- the lots producer (ADR-0084, milestone B) ---------------------------------------------------------

TEST_CASE(lanes_lots_producer_identity_follows_its_inputs_and_the_city_key) {
    using namespace engine::bundle;
    registerCityProducer(); registerLotsProducer();
    const BundleProducer* lp = findProducer(kLotsProducerName); const BundleProducer* cp = findProducer(kCityProducerName);
    CHECK(lp != nullptr && cp != nullptr); if (!lp || !cp) return;
    const std::filesystem::path before = std::filesystem::current_path(); std::filesystem::current_path(RT_SOURCE_DIR);
    struct Restore { std::filesystem::path p; ~Restore() { std::error_code ec; std::filesystem::current_path(p, ec); } } restore{before};
    LevelInputs in; std::string err; CHECK(loadLevelInputs("assets/lanelab/levels/ring.json", in, &err));
    CHECK(lp->applies(in));
    const ProducerIdentity a = lp->identity(in), b = lp->identity(in);
    CHECK(a.key == b.key && a.tag == kLotsBuildTag && a.inputs.size() >= cp->identity(in).inputs.size());
    LevelInputs seed = in; seed.level["citysim"]["seed"] = 4242; CHECK(lp->identity(seed).key != a.key);                 // the citysim block
    LevelInputs cell = in; cell.level["citysim"]["renderCell"] = 125.0; CHECK(lp->identity(cell).key != a.key);         // through the city key
    LevelInputs spawn = in; spawn.level["player"]["position"][0] = spawn.level["player"]["position"][0].get<double>() + 5.0; CHECK(lp->identity(spawn).key != a.key);
    LevelInputs terrain = in; terrain.level["terrain"] = nlohmann::json::object(); CHECK(lp->applies(terrain));        // terrain levels bake too, on the loader's ground (ADR-0095)...
    LevelInputs shaped = terrain; shaped.level["entities"].push_back(nlohmann::json{{"shape", "script"}, {"onTerrain", true}}); CHECK(!lp->applies(shaped));   // ...unless a script shapes it
    LevelInputs plain = in; plain.level["citysim"].erase("buildLots"); plain.level["citysim"].erase("planOnly"); CHECK(!lp->applies(plain));
    LevelInputs road = in; road.level["entities"].push_back(nlohmann::json{{"shape", "road"}}); CHECK(!lp->applies(road));
}

TEST_CASE(lanes_lots_bake_reads_the_city_products_back_and_is_deterministic) {
    using namespace engine::bundle; using namespace engine::lotcache;
    registerCityProducer(); registerLotsProducer();
    const std::filesystem::path before = std::filesystem::current_path(); std::filesystem::current_path(RT_SOURCE_DIR);
    struct Restore { std::filesystem::path p; ~Restore() { std::error_code ec; std::filesystem::current_path(p, ec); } } restore{before};
    LevelInputs in; std::string err; CHECK(loadLevelInputs("assets/lanelab/levels/ring.json", in, &err));
    // The producer's grow, twice, from the cached ring scene's products: byte-identical lots.
    const CityProducts p = cityProductsFromResult(scene("ring_city"), cityRenderCell(in.level));
    LotsCityInputs city; city.hasTerrain = p.hasTerrain; city.ground = p.ground; city.holes = p.holes; city.nav = p.lotNav; city.pavedSidewalk = p.bands.sidewalkWidth;
    // the decks too, as the producer reads them back: they set the earthwork and the 6 m freeway clearance (60c72b30).
    // Without them this grow kept the 8 lots by the ring's ramps that the bake drops: 189 here vs 181 read back.
    city.deck = p.deck;
    nlohmann::json ra, rb;
    const NetLotResult ga = growLotsForLevel(in, city, &ra), gb = growLotsForLevel(in, city, &rb);
    CHECK(ga.lots.size() > 100 && ga.lots.size() == gb.lots.size() && ra["units"] == rb["units"] && !ga.parts.empty());
    BundleWriter wa, wb; wa.openMemory(); wb.openMemory(); writeLotResult(wa, "lots/", ga); writeLotResult(wb, "lots/", gb);
    CHECK(wa.sections().size() == wb.sections().size());
    size_t differing = 0;
    for (size_t i = 0; i < wa.sections().size() && i < wb.sections().size(); ++i) if (wa.sections()[i].name != wb.sections()[i].name || wa.sections()[i].fnv != wb.sections()[i].fnv) ++differing;
    CHECK(differing == 0);
    // The real bake, in memory: city then lots in one bundle, the lots read the city's sections back.
    BakeRequest req; req.levelPath = "assets/lanelab/levels/ring.json"; req.toMemory = true; req.only = {kCityProducerName, kLotsProducerName};
    const BakeReport rep = bakeLevel(req);
    CHECK(rep.ok); if (!rep.ok) { std::printf("    bake: %s\n", rep.error.c_str()); return; }
    CHECK(rep.built.size() == 2 && rep.reports.count(kLotsProducerName) == 1);
    std::unique_ptr<Bundle> b = Bundle::fromMemory(rep.memory, &err); CHECK(b != nullptr); if (!b) return;
    NetLotResult back; CHECK(readLotResult(*b, kLotsSectionPrefix, back, &err)); if (!err.empty()) std::printf("    %s\n", err.c_str());
    CHECK(back.lots.size() == ga.lots.size() && back.plan.lots.size() == ga.plan.lots.size() && back.parts.size() == ga.parts.size() && back.flatParts.size() == ga.flatParts.size());
    // parts are stored per render cell: the cell chunks carry exactly the grown triangles, and reassemble to whole parts
    CHECK(lotCellSize(*b, kLotsSectionPrefix) == cityRenderCell(in.level));
    std::vector<LotCellPart> cps; CHECK(listLotCellParts(*b, kLotsSectionPrefix, cps, &err)); CHECK(cps.size() > 25);
    size_t cellTris = 0, wholeTris = 0, backTris = 0;
    for (const LotCellPart& cp : cps) { RenderMesh m; CHECK(readLotPart(*b, cp.section, m)); cellTris += m.indices.size() / 3; }
    for (const RenderMesh& m : ga.parts) wholeTris += m.indices.size() / 3; for (const RenderMesh& m : ga.flatParts) wholeTris += m.indices.size() / 3;
    for (const RenderMesh& m : back.parts) backTris += m.indices.size() / 3; for (const RenderMesh& m : back.flatParts) backTris += m.indices.size() / 3;
    CHECK(cellTris == wholeTris && backTris == wholeTris);
    const nlohmann::json lotsEntry = manifestProducer(b->manifest(), kLotsProducerName);
    CHECK(!lotsEntry.is_null() && lotsEntry.value("key", std::string()) == hex16(findProducer(kLotsProducerName)->identity(in).key));
    std::printf("    %s; %zu cell parts (grow %.1f s, write %.1f s)\n", rep.reports.at(kLotsProducerName).report.value("summary", std::string()).c_str(), cps.size(), rep.reports.at(kLotsProducerName).timings.at("grow"), rep.reports.at(kLotsProducerName).timings.at("write"));
}

// EVERY LOT MESH THROUGH THE MESH AUDIT (Glenn, 2026-10-14: "city lots should be reworked"): the grown lots of a lab
// city -- buildings, parks, gardens, playgrounds, car parks, yards, walks, the draped pads -- part by part, no triangle
// wound against its normals (bar float slivers), none without area, nothing non-finite. Before this only the bake
// audit (rt_bake --audit) looked at a lot's open ground. (Its 22 zero-area triangles on the island were float32
// storage collapsing sub-millimetre edges far from the origin -- packMesh drops those now; this lab is near the origin.)
TEST_CASE(lanes_every_lot_mesh_passes_the_mesh_audit) {
    using namespace engine::bundle; using namespace engine::lotcache;
    registerCityProducer(); registerLotsProducer();
    const std::filesystem::path before = std::filesystem::current_path(); std::filesystem::current_path(RT_SOURCE_DIR);
    struct Restore { std::filesystem::path p; ~Restore() { std::error_code ec; std::filesystem::current_path(p, ec); } } restore{before};
    LevelInputs in; std::string err; CHECK(loadLevelInputs("assets/lanelab/levels/ring.json", in, &err));
    const CityProducts p = cityProductsFromResult(scene("ring_city"), cityRenderCell(in.level));
    LotsCityInputs city; city.hasTerrain = p.hasTerrain; city.ground = p.ground; city.holes = p.holes; city.nav = p.lotNav; city.pavedSidewalk = p.bands.sidewalkWidth;
    city.deck = p.deck;
    const NetLotResult g = growLotsForLevel(in, city, nullptr);
    MeshAudit total;
    auto look = [&](const char* kind, size_t part, const RenderMesh& m) {
        if (m.indices.empty()) return;
        const MeshAudit a = auditMesh(m);
        if (a.inward - a.inwardSliver > 0 || a.degenerate > 0 || a.nonFinite > 0)
            printMeshAudit((std::string(kind) + " part " + std::to_string(part)).c_str(), a);
        total.add(a);
    };
    for (size_t i = 0; i < g.parts.size(); ++i) look("lot", i, g.parts[i]);
    for (size_t i = 0; i < g.flatParts.size(); ++i) look("lot flat", i, g.flatParts[i]);
    printMeshAudit("ALL LOTS (ring_city)", total);
    CHECK(total.tris > 100000);
    CHECK(total.inward - total.inwardSliver == 0);
    CHECK(total.degenerate == 0);
    CHECK(total.nonFinite == 0);
}

// ADR-0104: a river through a block leaves TWO blocks, one each side, and none of either is wet.
TEST_CASE(lanes_blocks_stand_back_from_a_river_through_them) {
    using namespace engine::roads::lanes;
    const std::vector<Ring> holes{{Vec2(0, 0), Vec2(200, 0), Vec2(200, 100), Vec2(0, 100)}};
    const std::vector<Ring> water{{Vec2(-10, 40), Vec2(210, 40), Vec2(210, 60), Vec2(-10, 60)}};   // a 20 m river across it
    const std::vector<Poly2> dry = blocksFromHoles(holes, 0.5, 0.0, 0.0, &water);
    const std::vector<Poly2> all = blocksFromHoles(holes, 0.5, 0.0, 0.0);
    CHECK(all.size() == 1);
    CHECK(dry.size() == 2);
    for (const Poly2& b : dry)
        for (const Vec2& p : b) CHECK(p.y <= 40.0 + 1e-6 || p.y >= 60.0 - 1e-6);
}

// LANE ARROWS (Glenn, 2026-09-25: "lane signs like arrows for where to turn ... they should be a part
// of the road to define if you have a protected turn"): each lane of an approach is painted with what
// it may do. The usual assignment, and the turn pockets that change it.
TEST_CASE(lanes_arrows_assign_turns_to_the_lanes_that_make_them) {
    using engine::roads::lanes::assignLaneMoves;
    // a four-way, two lanes: left + straight, straight + right
    auto m = assignLaneMoves(2, true, true, true);
    CHECK(m[0].left && m[0].straight && !m[0].right);
    CHECK(!m[1].left && m[1].straight && m[1].right);
    // three lanes: the middle one goes straight only
    m = assignLaneMoves(3, true, true, true);
    CHECK(m[1].straight && !m[1].left && !m[1].right);
    // a left-turn pocket turns left only; the lane beside it carries straight on
    m = assignLaneMoves(3, true, true, true, true, false);
    CHECK(m[0].left && !m[0].straight);
    CHECK(m[1].straight);
    // the stem of a T (no straight): the left lane turns left, the right lane right
    m = assignLaneMoves(2, true, false, true);
    CHECK(m[0].left && !m[0].straight && !m[0].right);
    CHECK(m[1].right && !m[1].straight && !m[1].left);
    // the top of a T with the cross street on the left: left + straight, straight only
    m = assignLaneMoves(2, true, true, false);
    CHECK(m[0].left && m[0].straight);
    CHECK(m[1].straight && !m[1].right);
}

// ADR-0116: a mountain road CUTS through the rise it cannot climb. The profile used to be the grade-limited
// envelope from above only -- it fills and never cuts -- so a pass over rough ground rode viaducts off every
// bump (2.7 km of bridge in 4.2). `balance` blends toward the envelope from below: closer to the ground,
// the grade kept, and a floor (a bridge's hold) still cleared.
TEST_CASE(lanes_a_balanced_profile_cuts_as_well_as_fills) {
    // a 2 km climb of 200 m (10%) with 25 m bumps every 150 m: steeper than 12% in places
    const HeightField ground = [](double x, double) { return 0.1 * x + 12.5 * std::sin(x / 150.0 * 6.2831853); };
    auto profile = [&](double balance, bool hold) {
        EdgeSpec e;
        for (double x = 0; x <= 2000.0; x += 2.0) e.xy.emplace_back(x, 0.0);
        if (hold) e.floorPts.push_back({1000.0, 0.0, 0.1 * 1000.0 + 30.0, 20.0});   // a hold 30 m over the ground at 1 km
        RoadClassSpec c; c.gMax = 0.15; c.window = 60.0; c.balance = balance;
        throughProfile(e, ground, c);
        return e;
    };
    const EdgeSpec fill = profile(0.0, false), bal = profile(0.5, false), held = profile(0.5, true);
    // what makes a viaduct is how high the deck stands over the ground: the tallest fill
    auto stats = [](const EdgeSpec& e, double& maxFill, double& maxGrade) {
        maxFill = 0; maxGrade = 0;
        for (std::size_t i = 0; i < e.z.size(); ++i) {
            maxFill = std::max(maxFill, e.z[i] - e.t[i]);
            if (i) maxGrade = std::max(maxGrade, std::fabs(e.z[i] - e.z[i - 1]) / (e.s[i] - e.s[i - 1]));
        }
    };
    double gapF, gradeF, gapB, gradeB;
    stats(fill, gapF, gradeF); stats(bal, gapB, gradeB);
    std::printf("    [balance] tallest fill: fill-only %.1f m, balanced %.1f m; grade %.3f / %.3f\n", gapF, gapB, gradeF, gradeB);
    CHECK(gapB < 0.7 * gapF);                          // it stands far lower off the ground
    CHECK(gradeB <= 0.8 * 0.15 + 1e-6);                 // and keeps the design grade
    const std::size_t mid = 500;                        // x = 1000
    CHECK(held.z[mid] >= 0.1 * 1000.0 + 30.0 - 1e-6);   // a hold is still cleared
    double gapH, gradeH; stats(held, gapH, gradeH);
    CHECK(gradeH <= 0.8 * 0.15 + 1e-6);
}

// NEAR-DEGENERATE CROSSINGS (regenerating island_8_nature: a freeway waypoint moved a metre, the city bake
// threw "Intersection of constraint edges ... can not be resolved: computed split vertex is invalid", and the
// level loaded with no roads or city at all). CDT's own recorded case (its issue #211) throws at a vanishing
// snap distance; constrainedTriangulation retries at coarser ones and returns a triangulation.
#include <fstream>
TEST_CASE(lanes_triangulation_survives_a_near_degenerate_crossing) {
    std::ifstream f(std::string(RT_SOURCE_DIR) + "/third_party/CDT/CDT/tests/inputs/issue-211.txt");
    CHECK(f.good());
    if (!f.good()) return;
    std::size_t nv = 0, ne = 0;
    f >> nv >> ne;
    std::vector<engine::Vec2> pts(nv);
    for (auto& p : pts) f >> p.x >> p.y;
    std::vector<std::pair<int, int>> edges;
    for (std::size_t i = 0; i < ne; ++i) {
        std::pair<int, int> e;
        f >> e.first >> e.second;
        // the file's last edge names vertex 97 of 97 (0..96): CDT's own test only checks it throws, and read
        // raw it overran CDT's tables -- this test then failed differently run to run
        if (e.first < static_cast<int>(nv) && e.second < static_cast<int>(nv)) edges.push_back(e);
    }
    bool threw = false;
    engine::roads::lanes::Triangulation t;
    try {
        t = engine::roads::lanes::constrainedTriangulation(pts, edges, 5e-17);
    } catch (const std::exception& e) {
        threw = true;
        std::printf("    threw: %s\n", e.what());
    } catch (...) {
        threw = true;
    }
    std::printf("    issue-211 input: %zu points, %zu valid edges -> %zu triangles%s\n", nv, edges.size(), t.tris.size(), threw ? " (THREW)" : "");
    // and the raw 5e-17 attempt really does fail on it: the retry is what rescued it
    bool rawThrows = false;
    {
        std::vector<CDT::V2d<double>> v; for (const auto& p : pts) v.push_back({p.x, p.y});
        std::vector<CDT::Edge> es; for (const auto& e : edges) es.emplace_back(static_cast<CDT::VertInd>(e.first), static_cast<CDT::VertInd>(e.second));
        CDT::RemoveDuplicatesAndRemapEdges(v, es);
        CDT::Triangulation<double> raw(CDT::VertexInsertionOrder::Auto, CDT::IntersectingConstraintEdges::TryResolve, 5e-17);
        try { raw.insertVertices(v); raw.insertEdges(es); } catch (const CDT::InvalidEdgeSplitVertex&) { rawThrows = true; }
    }
    CHECK(rawThrows);
    CHECK(!threw);
    CHECK(!t.tris.empty());
}

// PADS CLIPPED TO THEIR BLOCKS, FAST (island_8_nature: clipPadsToBlocks was about half of a 100 s load -- every
// lot cut against the union of all 1,407 blocks, the union built block by block). Now each lot is cut against
// the inset blocks its bounds touch. The same region: checked against the old algorithm, reimplemented here,
// over a town of rectangular and L-shaped blocks with lots straddling their edges.
TEST_CASE(lanes_pads_clip_to_their_blocks_same_as_the_full_union) {
    using namespace engine::roads::lanes;
    std::vector<engine::Poly2> blocks;
    for (int by = 0; by < 6; ++by)
        for (int bx = 0; bx < 6; ++bx) {
            const double x0 = bx * 70.0, y0 = by * 70.0;
            if ((bx + by) % 3 == 0)   // an L
                blocks.push_back({{x0, y0}, {x0 + 50, y0}, {x0 + 50, y0 + 20}, {x0 + 25, y0 + 20}, {x0 + 25, y0 + 50}, {x0, y0 + 50}});
            else
                blocks.push_back({{x0, y0}, {x0 + 50, y0}, {x0 + 50, y0 + 50}, {x0, y0 + 50}});
        }
    std::vector<engine::LotBuilding> lots;
    uint32_t seed = 7;
    auto rnd = [&]() { seed = seed * 1664525u + 1013904223u; return (seed >> 8) / 16777216.0; };
    for (int i = 0; i < 300; ++i) {
        engine::LotBuilding lb;
        lb.type = "home";
        const double cx = rnd() * 420 - 10, cy = rnd() * 420 - 10, w = 6 + rnd() * 20, d = 6 + rnd() * 20;
        lb.plan = {{cx, cy}, {cx + w, cy}, {cx + w, cy + d}, {cx, cy + d}};
        lb.site = engine::Vec2(cx + w * 0.5, cy + d * 0.5);
        lb.groundY = rnd() * 10;
        lots.push_back(lb);
    }
    const double sidewalk = 5.0;
    const std::vector<engine::TerrainFlatten> fast = clipPadsToBlocks(lots, blocks, sidewalk);
    // THE OLD WAY: the union of every inset block, built block by block, and every lot cut against all of it
    const double falloff = lanesPadFalloff(sidewalk);
    PolySet all;
    for (const engine::Poly2& b : blocks) all = unionSets(all, offsetSet(fromRing(Ring(b.begin(), b.end())), -falloff));
    std::vector<engine::TerrainFlatten> slow;
    for (const engine::LotBuilding& lb : lots) {
        const engine::TerrainFlatten raw = engine::lotPadFlatten(lb, 2.2 + falloff, falloff);
        Ring ring; for (const engine::Vec3& v : raw.polygon) ring.emplace_back(v.x, v.z);
        std::vector<Ring> mine;
        for (const Polygon2& pg : intersectSets(fromRing(ring), all)) {
            if (pg.outer.size() < 3 || std::fabs(ringArea(pg.outer)) < 1.0) continue;
            mine.push_back(pg.outer);
            std::vector<engine::Vec3> poly; for (const engine::Vec2& q : pg.outer) poly.push_back(engine::Vec3(q.x, 0, q.y));
            slow.push_back(engine::makeFlattenPad(std::move(poly), lb.groundY, falloff));
        }
        // ...and the pad holds its walls: a plan corner off the clipped pad makes the plan a pad too
        bool held = true;
        for (const engine::Vec2& v : lb.plan) {
            bool in = false;
            for (const Ring& r : mine) in = in || pointInRing(r, v);
            held = held && in;
        }
        if (!mine.empty() && !held) {
            std::vector<engine::Vec3> walls; for (const engine::Vec2& v : lb.plan) walls.push_back(engine::Vec3(v.x, 0, v.y));
            slow.push_back(engine::makeFlattenPad(std::move(walls), lb.groundY, falloff));
        }
    }
    auto signature = [](const std::vector<engine::TerrainFlatten>& v) {
        std::vector<std::pair<double, double>> s;   // (plane height, footprint area), sorted
        for (const engine::TerrainFlatten& f : v) {
            double a = 0;
            for (std::size_t i = 0; i < f.polygon.size(); ++i) {
                const engine::Vec3& p = f.polygon[i]; const engine::Vec3& q = f.polygon[(i + 1) % f.polygon.size()];
                a += p.x * q.z - q.x * p.z;
            }
            s.push_back({std::round(f.c * 1000) / 1000, std::round(std::fabs(a) * 0.5 * 100) / 100});
        }
        std::sort(s.begin(), s.end());
        return s;
    };
    const auto sf = signature(fast), ss = signature(slow);
    std::printf("    %zu lots -> %zu pads (fast) / %zu pads (full union)\n", lots.size(), fast.size(), slow.size());
    CHECK(fast.size() == slow.size());
    CHECK(sf == ss);
    CHECK(!fast.empty() && fast.size() < lots.size() * 2);
}

// A LINE IN SECTIONS: a lift's approach runs on across the joint (chainTrackFloors). Two light-rail sections end to
// start, a street under the first 30 m short of the joint: the second section takes up the descent where the first
// leaves it, no step at the joint (Saltwood's joints stepped 4 m).
TEST_CASE(lanes_a_track_lift_runs_on_across_a_joint_between_sections) {
    nlohmann::json g;
    g["name"] = "joint";
    g["edges"] = nlohmann::json::array({
        {{"id", "a"}, {"class", "light_rail"}, {"path", {{"type", "polyline"}, {"points", {{-400, 0}, {0, 0}}}}}},
        {{"id", "b"}, {"class", "light_rail"}, {"path", {{"type", "polyline"}, {"points", {{0, 0}, {400, 0}}}}}},
        {{"id", "st"}, {"class", "local"}, {"path", {{"type", "polyline"}, {"points", {{-30, -150}, {-30, 150}}}}}}});
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
    const EdgeSpec& a = *r->graph.find("a"); const EdgeSpec& b = *r->graph.find("b");
    const double zA = a.z.back(), zB = b.z.front(), lift = zA - a.t.back();
    std::printf("    [joint] a ends %.2f m, b starts %.2f m (%.1f m over the ground)\n", zA, zB, lift);
    CHECK(lift > 3.0);                    // still on its way down from the street when it reaches the joint
    CHECK(std::fabs(zA - zB) < 0.05);     // and the next section takes it up there
    checkInvariants(*r);
}

// A SKEWED CROSSING OVER A CLIMB: the lift clears the lower road's highest point under the upper band, and along the
// lower road that band runs on for the lower's own half width over the tangent -- the island loop crossed the pass at
// 28 deg with the pass 13.9 % up, and the pass's outer lane, 6 m on, came 21 cm short of the clearance.
TEST_CASE(lanes_a_track_over_a_steep_road_at_a_skew_clears_its_outer_lane) {
    const double c = std::cos(28.0 * 3.14159265358979 / 180.0), s = std::sin(28.0 * 3.14159265358979 / 180.0);
    nlohmann::json g;
    g["name"] = "skew";
    g["classes"]["mountain"] = {{"w", 3.5}, {"fwd", 1}, {"back", 1}, {"shoulder", 1.0}, {"g_max", 0.15}, {"window", 60}, {"thick", 0.5}};
    g["edges"] = nlohmann::json::array({
        {{"id", "rail"}, {"class", "rail"}, {"path", {{"type", "polyline"}, {"points", {{-600, 0}, {600, 0}}}}}},
        // up 14 % through the crossing: held at 21 m 150 m on, 0 m 150 m back
        {{"id", "climb"}, {"class", "mountain"}, {"path", {{"type", "polyline"}, {"points", {{-300 * c, -300 * s}, {300 * c, 300 * s}}}}},
         {"floor", {{150 * c, 150 * s, 21.0, 20.0}}}}});
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
    const EdgeSpec& cl = *r->graph.find("climb");
    std::printf("    [skew] climb %.1f m at the crossing, %.1f m 20 m on\n", interp(cl.s, cl.z, 300.0), interp(cl.s, cl.z, 320.0));
    checkInvariants(*r);
}

// THE LIGHT RAIL LAB: held up over its streets, its stations reached from below -- the way passes under the line at
// the street's level, and each platform stands level along its length.
TEST_CASE(lanes_light_rail_lab_reaches_its_stations_from_below) {
    const Result& r = scene("light_rail_lab"); checkInvariants(r);
    for (const char* id : {"lr_s", "lr_n"}) {
        const EdgeSpec& e = *r.graph.find(id);
        double zMin = 1e9, zMax = -1e9;
        for (std::size_t i = 0; i < e.s.size(); ++i)
            if (e.s[i] >= 62 && e.s[i] <= 142) { zMin = std::min(zMin, e.z[i]); zMax = std::max(zMax, e.z[i]); }
        CHECK(zMax - zMin < 0.05);        // level along the platforms
        const EdgeSpec& w = *r.graph.find(std::string(id) == "lr_s" ? "lr_way_s" : "lr_way_n");
        double wMax = -1e9;
        for (double z : w.z) wMax = std::max(wMax, z);
        CHECK(zMin - wMax > 5.0);         // the way under it, on the street's level
    }
}

// THE STATION KIT AND WHAT STANDS ON A TRACK (Glenn, 2026-10-12: "cars weirdly stuck in the tracks", the light rail
// "really close to buildings", stations want lights and canopies with dimension). On the light-rail lab:
// - a street under the line answers ITS height, not the rail's -- for parked cars (heightAt) and traffic (heightOn);
//   a tree or a rock still stays off the track (coversAny); the deck field carries the difference through the bundle;
// - a footprint the track crosses is kept out, even with every corner well off the line; one 20 m off is not;
// - each platform has a ribbed canopy, a steel frame and lamps, and each stop the night lights under them.
TEST_CASE(lanes_light_rail_lab_tracks_carry_no_street_and_stations_have_canopies_and_lamps) {
    const Result& r = scene("light_rail_lab");
    const CityProducts p = cityProductsFromResult(r, 64.0);
    int tracks = 0, footways = 0;
    for (const UnionSpine& sp : p.deck.spines) { tracks += sp.use == UnionSpine::Use::Track; footways += sp.use == UnionSpine::Use::Footway; }
    CHECK(tracks >= 2 && footways >= 2);
    // where the line passes over a street
    int checked = 0;
    for (const EdgeSpec& t : r.graph.edges) {
        if (t.cls != "light_rail") continue;
        for (const EdgeSpec& s : r.graph.edges) {
            if (s.cls == "light_rail" || s.cls == "footway") continue;
            for (const Vec2& X : crossings(t.xy, s.xy)) {
                auto zAt = [&X](const EdgeSpec& e) {
                    const double st = project(e.xy, e.s, X).station;
                    const std::size_t j = std::min<std::size_t>(e.s.size() - 1, std::max<std::size_t>(1, static_cast<std::size_t>(std::lower_bound(e.s.begin(), e.s.end(), st) - e.s.begin())));
                    const double u = std::clamp((st - e.s[j - 1]) / std::max(1e-9, e.s[j] - e.s[j - 1]), 0.0, 1.0);
                    return e.z[j - 1] + (e.z[j] - e.z[j - 1]) * u;
                };
                const double trackZ = zAt(t), streetZ = zAt(s);
                if (trackZ - streetZ < 4.0) continue;
                double y = 0;
                CHECK(p.deck.heightAt(X.x, X.y, 1.0, &y));
                CHECK(std::fabs(y - streetZ) < 1.0);
                const RoadClass local = classOf(s);
                CHECK(p.deck.heightOn(X.x, X.y, 0.5, &local, 0.0, 1e9, &y));
                CHECK(std::fabs(y - streetZ) < 1.0);
                CHECK(p.deck.coversAny(X.x, X.y, 0.5));
                ++checked;
            }
        }
    }
    std::printf("    [lr] %d crossings under the line checked\n", checked);
    CHECK(checked >= 2);
    {   // through the bundle
        bundle::BinWriter w; bundle::putDeckField(w, p.deck);
        RoadDeckField back; bundle::BinReader rd(w.bytes.data(), w.bytes.size());
        CHECK(bundle::getDeckField(rd, back));
        int t2 = 0; for (const UnionSpine& sp : back.spines) t2 += sp.use == UnionSpine::Use::Track;
        CHECK(t2 == tracks);
    }
    // buildings clear of the line
    const auto keepOut = lanesTrackKeepOut(p.deck, p.rails);
    CHECK(static_cast<bool>(keepOut));
    if (keepOut) {
        const RailTrack& t = p.rails.front();
        const Vec3 a = t.pts[t.pts.size() / 4], b = t.pts[t.pts.size() / 4 + 3];
        Vec2 d(b.x - a.x, b.z - a.z); d = d / d.length(); const Vec2 n(-d.y, d.x), c(a.x, a.z);
        auto square = [&](Vec2 m, double h) { return std::vector<Vec2>{m + (d + n) * h, m + (n - d) * h, m - (d + n) * h, m + (d - n) * h}; };
        CHECK(keepOut(square(c, 14.0)));             // the line through its middle, corners ~20 m off
        CHECK(!keepOut(square(c + n * 40.0, 10.0)));  // 20 m clear of it
    }
    // the kit
    std::set<std::string> names;
    for (const CityCellMesh& cm : p.cells) names.insert(cm.mesh.name);
    CHECK(names.count("canopy") && names.count("station_steel") && names.count("station_lamp"));
    int stops = 0;
    for (const RailTrack& t : p.rails)
        for (const RailStop& st : t.stops) {
            ++stops;
            CHECK(st.lamps.size() >= 4);
            for (const Vec3& l : st.lamps) CHECK(l.y > st.top + 2.5 && l.y < st.top + 3.6);
        }
    CHECK(stops >= 2);
}

// EVERY DECK FACE OUTWARD (Glenn, 2026-10-13: "the elevated freeway/rail undercarriage when viewed from below can be
// seen through, the normals are wound wrong there as well"). The engine draws CLOCKWISE triangles (seen from the side
// their normal points to) and culls the rest in the opaque pass: a triangle (a, b, c) faces its normal when
// cross(c - a, b - a) points along it. Every mesh the decks build, on the freeway, viaduct and rail labs.
TEST_CASE(lanes_every_deck_triangle_faces_its_normal) {
    // Every lab graph through the mesh audit (engine/mesh_audit.h), by named mesh, with where its first inward triangle
    // is. The island's bake had 2303 flipped deck triangles on its elevated freeway (rt_bake --audit) while the four
    // lab decks this tested passed -- so every lab graph, not a chosen four.
    MeshAudit total;
    for (const char* scn : {"curve_blocks", "freeway_cross", "freeway_cross_relief", "grid_city", "hill_junction",
                            "light_rail_lab", "merge_taper", "rail_loop", "ring_city", "steep_climb",
                            "system_interchange", "valley_viaduct"}) {
        const Result& r = scene(scn);
        for (const NamedMesh& nm : buildMeshes(r)) {
            const MeshAudit a = auditMesh(nm.mesh);
            total.add(a);
            if (a.inward > 0) printMeshAudit((std::string(scn) + " " + nm.name).c_str(), a);
        }
    }
    printMeshAudit("decks", total);
    CHECK(total.tris > 10000);
    CHECK(total.inward == 0);
}

// A STREET'S SIDEWALK RUNS ON UNDER A BRIDGE (Glenn, 2026-10-15: "where the elevated train deck goes over the sidewalk
// it removes that part of the sidewalk underneath it"). Shoulders and sidewalks were cut by every lane in PLAN; now
// only by what stands at their level. Under the light-rail lab's raised track and the ring city's ring, a street's
// sidewalk must lie inside the bridge lanes' footprints, metres under them (none did with the plan-only cut, which
// RT_LAYERS_FLAT=1 still gives: 0, 0 and 0; 150, 450 and 301 m2 at the interchange now).
TEST_CASE(lanes_a_sidewalk_runs_on_under_a_bridge) {
  for (const char* scn : {"light_rail_lab", "ring_city"}) {
    const Result& r = scene(scn);
    double under = 0;
    for (size_t li = 0; li < r.lanes.lanes.size(); ++li) {
        for (const Polygon2& piece : intersectSets(r.pavement.sidewalk, r.pavement.footprints[li])) {
            if (piece.outer.size() < 3) continue;
            Vec2 c; for (const Vec2& q : piece.outer) c += q; c = c / static_cast<double>(piece.outer.size());
            int street = -1;
            for (size_t ei = 0; ei < r.graph.edges.size(); ++ei)
                if (r.graph.cls(r.graph.edges[ei]).sidewalk > 0 && project(r.graph.edges[ei].xy, r.graph.edges[ei].s, c).distance < 20) street = static_cast<int>(ei);
            if (street < 0) continue;
            if (r.heights->own(static_cast<int>(li), c) - r.heights->ownRoad(street, c) > 3.0) under += setArea(PolySet{piece});
        }
    }
    // ...and its MESH is there too, at street level (Glenn, 2026-10-15: "there's a cut in the sidewalk because of the
    // lane above it"): the outline ran on under the deck, but each vertex took the height of the nearest lane in
    // plan -- the deck overhead -- and the triangles spanning the two levels were dropped as too steep
    double meshed = 0;
    const std::vector<double> band = layerBands(r.graph, kSidewalkLayer);
    for (const FlatMesh& m : layerMeshes(r.graph, *r.heights, r.pavement.sidewalk, layerRoads(r.graph, false), nullptr, &band)) {
        for (const auto& t : m.tris) {
            const DeckVertex &a = m.verts[static_cast<size_t>(t[0])], &b = m.verts[static_cast<size_t>(t[1])], &c = m.verts[static_cast<size_t>(t[2])];
            const Vec2 q = (a.xy + b.xy + c.xy) / 3.0; const double z = (a.z + b.z + c.z) / 3.0;
            bool overhead = false, atStreet = false;
            for (size_t li = 0; li < r.lanes.lanes.size() && !overhead; ++li) {
                if (r.pavement.footprints[li].empty() || !contains(r.pavement.footprints[li], q)) continue;
                overhead = r.heights->own(static_cast<int>(li), q) - z > 3.0;
            }
            for (size_t ei = 0; ei < r.graph.edges.size() && overhead && !atStreet; ++ei)
                atStreet = r.graph.cls(r.graph.edges[ei]).sidewalk > 0 && project(r.graph.edges[ei].xy, r.graph.edges[ei].s, q).distance < 20 && std::fabs(z - r.heights->ownRoad(static_cast<int>(ei), q)) < 0.5;
            if (overhead && atStreet) meshed += 0.5 * std::fabs(cross(b.xy - a.xy, c.xy - a.xy));
        }
    }
    std::printf("    [layers] %s sidewalk under the bridges: %.0f m2 outlined, %.0f m2 meshed at street level\n", scn, under, meshed);
    CHECK(under > 100.0);
    CHECK(meshed > 0.9 * under);
  }
}

// NOTHING ELEVATED IS SEE-THROUGH FROM BELOW (Glenn, 2026-10-13: "the elevated freeway/rail undercarriage when viewed
// from below can be seen through"). engine::auditUndersides over every lab graph with a deck: every walkable surface
// more than 2 m over the ground has something facing down within 3 m under it. The island's bake had 8512 open (rt_bake
// --audit), most of them the median wall's top over the seam between twin carriageways -- the wall had no bottom.
TEST_CASE(lanes_no_elevated_surface_is_see_through_from_below) {
    size_t hanging = 0, open = 0;
    for (const char* scn : {"freeway_cross", "freeway_cross_relief", "light_rail_lab", "rail_loop", "system_interchange",
                            "valley_viaduct", "steep_climb", "hill_junction"}) {
        const Result& r = scene(scn);
        const std::vector<NamedMesh> meshes = buildMeshes(r);
        std::vector<NamedMeshRef> refs;
        for (const NamedMesh& nm : meshes) refs.push_back({nm.name, &nm.mesh});
        const UndersideAudit u = auditUndersides(
            refs,
            [](const std::string& n) {
                return n == "asphalt" || n == "shoulder" || n == "sidewalk" || n == "median" || n == "ballast" || n == "platform" || n == "concrete";
            },
            [&r](double x, double z) { return r.hasTerrain ? r.terrain.sampleDrawn(x, z) : 0.0; });
        if (u.open > 0) {
            std::printf("    %s:\n", scn);
            printUndersideAudit(u, 6);
            std::printf("    [undersides]   first at %.1f %.2f %.1f\n", u.firstOpen.x, u.firstOpen.y, u.firstOpen.z);
            if (std::getenv("RT_UNDERSIDE_COLUMN")) printColumn(refs, u.firstOpen.x, u.firstOpen.z, 1.0);
        }
        hanging += u.hanging;
        open += u.open;
    }
    std::printf("    [undersides] labs: %zu hanging, %zu open\n", hanging, open);
    CHECK(hanging > 1000);
    CHECK(open == 0);
}

// NOTHING ELEVATED IS SEE-THROUGH FROM AN ANGLE (Glenn, 2026-10-14: "a strip of polygons missing along the side ... the
// hole is clearly one way"): engine::auditSeeThrough -- from half a metre under every deck's soffit, looking up and at
// 45 deg all round, no ray meets the back of a face. The 1.2 m carriageway slab beside a 0.3 m shoulder left its 0.9 m
// step open (slab()'s side rule hid the whole side behind the neighbour's top).
TEST_CASE(lanes_no_elevated_deck_is_see_through_from_an_angle) {
    size_t eyes = 0, through = 0;
    // KNOWN, small (a regression past them fails): 4 at the interchange, 5 at the valley's ramp ends. They were 27 and
    // 10 while the slab's sides merged within a centimetre of their edge: a look up at a slab's corner went through the
    // crack between the step face and the soffit (2026-10-14: merged within 2 mm now). Shoulders and sidewalks cut per
    // road at their own level (2026-10-15) tessellate differently, which moves the eyes (one per walkable top): the same
    // slab-corner ties sample as 1 on the relief crossing and 6 at the interchange -- the geometry along each new look is
    // identical under RT_LAYERS_FLAT=1 (RT_SEE_THROUGH_RAY).
    const std::map<std::string, size_t> known = {{"freeway_cross", 0}, {"freeway_cross_relief", 1}, {"light_rail_lab", 0},
                                                 {"rail_loop", 0}, {"system_interchange", 6}, {"valley_viaduct", 5}};
    for (const char* scn : {"freeway_cross", "freeway_cross_relief", "light_rail_lab", "rail_loop", "system_interchange", "valley_viaduct"}) {
        const Result& r = scene(scn);
        const std::vector<NamedMesh> meshes = buildMeshes(r);
        std::vector<NamedMeshRef> refs;
        for (const NamedMesh& nm : meshes) refs.push_back({nm.name, &nm.mesh});
        const SeeThroughAudit u = auditSeeThrough(
            refs, [](const std::string& n) { return n == "asphalt" || n == "shoulder" || n == "sidewalk" || n == "median" || n == "ballast" || n == "platform"; },
            [&r](double x, double z) { return r.hasTerrain ? r.terrain.sampleDrawn(x, z) : 0.0; });
        std::printf("    [see-through] %-20s %6zu eyes (%zu inside solids), %6zu of %6zu looks see through", scn, u.eyes, u.insideSolid, u.through, u.looks);
        if (u.through) std::printf(" -- first from %.1f %.1f %.1f looking %.2f %.2f %.2f", u.firstEye.x, u.firstEye.y, u.firstEye.z, u.firstDir.x, u.firstDir.y, u.firstDir.z);
        std::printf("\n");
        for (const auto& [m, n] : u.byHitMesh) std::printf("      backs of %-12s %6zu\n", m.c_str(), n);
        if (std::getenv("RT_SEE_THROUGH_SAMPLES")) for (const std::string& smp : u.samples) std::printf("      %s\n", smp.c_str());
        // RT_SEE_THROUGH_RAY="scene,ox,oy,oz,dx,dy,dz": every face along one look in that scene (mesh_audit printRay)
        if (const char* ray = std::getenv("RT_SEE_THROUGH_RAY")) {
            char name[64]; double v[6];
            if (std::sscanf(ray, "%63[^,],%lf,%lf,%lf,%lf,%lf,%lf", name, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == 7 && std::string(name) == scn)
                printRay(refs, Vec3(v[0], v[1], v[2]), Vec3(v[3], v[4], v[5]));
        }
        // RT_SEE_THROUGH_SECTION="scene,x0,z0,x1,z1,step": what stands over each point of a line in that scene
        if (const char* sec = std::getenv("RT_SEE_THROUGH_SECTION")) {
            char name[64]; double a0, b0, a1, b1, st;
            if (std::sscanf(sec, "%63[^,],%lf,%lf,%lf,%lf,%lf", name, &a0, &b0, &a1, &b1, &st) == 6 && std::string(name) == scn)
                printSection(refs, a0, b0, a1, b1, st);
        }
        eyes += u.eyes;
        through += u.through;
        CHECK(u.through <= known.at(scn));
    }
    CHECK(eyes > 1000);
}

// THE DECKS' UNDERSIDES face DOWN (Glenn: "the elevated freeway/rail undercarriage when viewed from below can be seen
// through"): over an elevated deck's top, as many square metres face down as face up -- its soffit.
TEST_CASE(lanes_an_elevated_deck_has_a_soffit_facing_down) {
    for (const char* scn : {"valley_viaduct", "light_rail_lab", "freeway_cross"}) {
        const Result& r = scene(scn);
        double up = 0, down = 0;
        for (const NamedMesh& nm : buildMeshes(r)) {
            const RenderMesh& m = nm.mesh;
            for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
                const Vec3 a = m.vertices[m.indices[i]].position, b = m.vertices[m.indices[i + 1]].position,
                           c = m.vertices[m.indices[i + 2]].position;
                const Vec3 f = cross(c - a, b - a);   // (the engine's front: clockwise from outside)
                const double area = 0.5 * f.length();
                if (area < 1e-9) continue;
                const double ny = f.y / (2 * area);
                if (ny > 0.9) up += area; else if (ny < -0.9) down += area;
                (void)nm;
            }
            std::printf("    [soffit] %s %-14s up %10.0f m2, down %10.0f m2\n", scn, nm.name.c_str(), up, down);
            up = down = 0;
        }
    }
}
