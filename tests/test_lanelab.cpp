// lanelab (ADR-0083): the geometry seam and, as the modules land, the generator's invariants.
#include "test_framework.h"
#include <filesystem>
#include <fstream>
#include "engine/procgen/lanelab/geom2d.h"

using namespace engine;
using namespace engine::lanelab;

namespace {
Ring square(double x0, double y0, double s) { return {{x0, y0}, {x0 + s, y0}, {x0 + s, y0 + s}, {x0, y0 + s}}; }
}

TEST_CASE(lanelab_union_of_two_overlapping_squares_has_the_inclusion_exclusion_area) {
    PolySet u = unionRings({square(0, 0, 10), square(5, 5, 10)});
    CHECK(u.size() == 1);
    CHECK_APPROX(setArea(u), 100 + 100 - 25, 1e-6);
    CHECK(contains(u, {7.5, 7.5}));
    CHECK(!contains(u, {12.0, 2.0}));
}

TEST_CASE(lanelab_difference_leaves_a_hole_and_closing_fills_a_notch) {
    PolySet frame = differenceSets(fromRing(square(0, 0, 10)), fromRing(square(4, 4, 2)));
    CHECK(frame.size() == 1 && frame[0].holes.size() == 1);
    CHECK_APPROX(setArea(frame), 96, 1e-6);
    // an L-shape: closing with r=3 fills the concave corner (a fillet of area (1 - pi/4) r^2)
    Ring L = {{0, 0}, {10, 0}, {10, 4}, {4, 4}, {4, 10}, {0, 10}};
    PolySet closed = closing(fromRing(L), 3.0);
    CHECK_APPROX(setArea(closed) - setArea(fromRing(L)), (1 - 3.14159265 / 4) * 9, 0.05);
}

TEST_CASE(lanelab_constrained_triangulation_splits_crossing_constraints_and_keeps_them_as_edges) {
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

#include "engine/procgen/lanelab/lane_expand.h"
#include "engine/procgen/lanelab/terrain_recipe.h"
#include "engine/procgen/lanelab/vertical_profile.h"
#include "engine/procgen/lanelab/polyline_ops.h"

TEST_CASE(lanelab_lane_offsets_come_from_the_road_spec_bands) {
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

TEST_CASE(lanelab_graph_loads_and_expands_the_valley_scene) {
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

#include "engine/procgen/lanelab/lanelab.h"
#include "engine/procgen/lanelab/road_twin.h"
#include "engine/procgen/lanelab/block_audit.h"
#include "engine/procgen/lanelab/city_producer.h"
#include "engine/procgen/lanelab/lots_producer.h"
#include "engine/bundle/bundle_glb.h"
#include "engine/procgen/city/lot_cache.h"
#include "engine/model_importer.h"
#include "engine/procgen/city/road_network.h"
#include "engine/procgen/city/polygon.h"
#include <cstdio>

namespace {
std::map<std::string, std::unique_ptr<Result>>& resultCache() { static std::map<std::string, std::unique_ptr<Result>> c; return c; }
const Result& scene(const std::string& name) {
    auto& c = resultCache(); auto it = c.find(name);
    if (it == c.end()) { std::unique_ptr<Result> r = build(RoadLabGraph::load(std::string(RT_SOURCE_DIR) + "/assets/lanelab/" + name + ".json")); std::printf("%s", summary(*r).c_str()); it = c.emplace(name, std::move(r)).first; }
    return *it->second;
}
void checkInvariants(const Result& r) { for (const Check& c : invariants(r)) { if (!c.ok) std::printf("    invariant failed: %s — %s\n", c.name.c_str(), c.detail.c_str()); CHECK(c.ok); } }
double bridge(const Result& r, const char* id) { return r.bridgeLen.at(id); }
}

TEST_CASE(lanelab_grid_city_builds_and_holds_its_invariants) {
    const Result& r = scene("grid_city"); checkInvariants(r);
    CHECK(r.pavement.decks.size() == 2);                       // the eastbound carriageway is an island
    CHECK(setArea(r.pavement.median) > 1000 && setArea(r.pavement.sidewalk) > 5000);
    CHECK(r.graph.find("exit")->anchorLanes.at("from") == "fw_wb.f2" && r.graph.find("entry")->anchorLanes.at("to") == "fw_wb.f2");
}

TEST_CASE(lanelab_valley_viaduct_builds_and_holds_its_invariants) {
    const Result& r = scene("valley_viaduct"); checkInvariants(r);
    CHECK(bridge(r, "fwy") >= 90 && bridge(r, "fwy") <= 150);
    CHECK(bridge(r, "ramp") > 20);                             // the ramp crosses the valley's north end on structure
    CHECK(r.pavement.decks.size() == 1);                       // local, ramp and freeway weld into one surface that passes over itself
    bool separated = false; for (const auto& kv : r.pavement.pairs) if (kv.second.separatedArea > 0 && kv.second.dz > 15) separated = true;
    CHECK(separated);
}

TEST_CASE(lanelab_hill_junction_builds_and_holds_its_invariants) {
    const Result& r = scene("hill_junction"); checkInvariants(r);
    int connectors = 0; for (const Lane& l : r.lanes.lanes) if (l.isConnector()) ++connectors;
    CHECK(connectors == 2 && r.lanes.lanes.size() == 10);
    CHECK(r.pavement.decks.size() == 1);
    CHECK(!r.pavement.islands.empty());                        // the slip lane and its connector enclose an island
}

TEST_CASE(lanelab_ring_city_builds_and_holds_its_invariants) {
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
TEST_CASE(lanelab_road_twin_closes_every_ring_city_block_and_each_survives_the_sidewalk_inset) {
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
TEST_CASE(lanelab_road_twin_faces_with_curved_sides_survive_the_metro_sidewalk_inset) {
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
TEST_CASE(lanelab_twin_leaves_no_block_foot_on_built_pavement_in_the_clean_scenes) {
    CHECK(brokenBlockArea(scene("ring_city"), 5.0) < 1.0);
    CHECK(brokenBlockArea(scene("curve_blocks"), 5.0) < 1.0);
    CHECK(brokenBlockArea(scene("hill_junction"), 5.0) < 1.0);
}

// The engine's OWN block/lot pass on the twin (headless), audited: on the clean scenes no block may carry
// a thin protrusion (an 8 m opening removes < 20 m²) or sit on built pavement (> 1 m²). This is the test
// Glenn asked for: if blocks look like spikes and snakes in-engine, this must fail.
TEST_CASE(lanelab_the_city_block_pass_yields_well_formed_blocks_on_the_clean_scenes) {
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

TEST_CASE(lanelab_city_products_round_trip_through_a_bundle_to_the_byte) {
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

TEST_CASE(lanelab_city_producer_identity_follows_its_inputs) {
    using namespace engine::bundle;
    registerCityProducer(); const BundleProducer* cp = findProducer(kCityProducerName); CHECK(cp != nullptr); if (!cp) return;
    // level files name their graphs relative to the repo root (the viewer's working directory)
    const std::filesystem::path before = std::filesystem::current_path(); std::filesystem::current_path(RT_SOURCE_DIR);
    struct Restore { std::filesystem::path p; ~Restore() { std::error_code ec; std::filesystem::current_path(p, ec); } } restore{before};
    LevelInputs in; std::string err; CHECK(loadLevelInputs("assets/lanelab/levels/ring.json", in, &err));
    CHECK(cp->applies(in)); CHECK(cityEntities(in.level).size() == 1 && !cityEntities(in.level)[0].inlineGraph);
    const ProducerIdentity a = cp->identity(in), b = cp->identity(in); CHECK(a.key == b.key && a.tag == kLanelabBuildTag && !a.inputs.empty() && a.inputs[0].bytes > 0);
    LevelInputs other = in; other.level["citysim"]["renderCell"] = 125.0; CHECK(cp->identity(other).key != a.key);
    LevelInputs inl = in; inl.level["entities"][cityEntities(in.level)[0].entityIndex]["lanelab"] = nlohmann::json{{"edges", nlohmann::json::array()}};
    CHECK(cityEntities(inl.level)[0].inlineGraph && cp->identity(inl).key != a.key);
}

TEST_CASE(lanelab_bundle_glb_view_reads_back_through_the_engine_importer) {
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

TEST_CASE(lanelab_grown_lots_round_trip_through_the_lot_codecs) {
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

TEST_CASE(lanelab_lots_producer_identity_follows_its_inputs_and_the_city_key) {
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
    LevelInputs terrain = in; terrain.level["terrain"] = nlohmann::json::object(); CHECK(!lp->applies(terrain));       // terrain levels grow in the pre-pass
    LevelInputs plain = in; plain.level["citysim"].erase("buildLots"); plain.level["citysim"].erase("planOnly"); CHECK(!lp->applies(plain));
    LevelInputs road = in; road.level["entities"].push_back(nlohmann::json{{"shape", "road"}}); CHECK(!lp->applies(road));
}

TEST_CASE(lanelab_lots_bake_reads_the_city_products_back_and_is_deterministic) {
    using namespace engine::bundle; using namespace engine::lotcache;
    registerCityProducer(); registerLotsProducer();
    const std::filesystem::path before = std::filesystem::current_path(); std::filesystem::current_path(RT_SOURCE_DIR);
    struct Restore { std::filesystem::path p; ~Restore() { std::error_code ec; std::filesystem::current_path(p, ec); } } restore{before};
    LevelInputs in; std::string err; CHECK(loadLevelInputs("assets/lanelab/levels/ring.json", in, &err));
    // The producer's grow, twice, from the cached ring scene's products: byte-identical lots.
    const CityProducts p = cityProductsFromResult(scene("ring_city"), cityRenderCell(in.level));
    LotsCityInputs city; city.hasTerrain = p.hasTerrain; city.ground = p.ground; city.holes = p.holes;
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
