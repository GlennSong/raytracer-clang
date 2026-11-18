// lanelab (ADR-0083): the geometry seam and, as the modules land, the generator's invariants.
#include "test_framework.h"
#include <CDT.h>
#include <filesystem>
#include <fstream>
#include "engine/procgen/city/roads/lanes/geom2d.h"
#include "engine/procgen/city/roads/lanes/deck_mesh.h"
#include "engine/mesh_audit.h"
#include "engine/mesh_builder.h"
#include <array>
#include <cmath>
#include <functional>
#include "engine/vertex_pack.h"

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
#include "engine/level_params.h"
#include "engine/procgen/terrain.h"
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
    CHECK(project(h.xy(), h.s(), g.find("ramp")->xy.back()).distance < 0.05);
    HeightField terrain = makeTerrain(g.terrain, g.bounds());
    for (EdgeSpec& e : g.edges) if (!e.isRamp()) throughProfile(e, terrain, g.cls(e));
    CHECK(maxGrade(*g.find("fwy")) <= 0.03 * kDesignGrade + 1e-6);
}

#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/lanes/block_audit.h"
#include "engine/procgen/city/roads/lanes/city_producer.h"
#include "engine/procgen/city/roads/lanes/lots_producer.h"
#include "engine/procgen/city/roads/road_builder.h"
#include "engine/bundle/bundle_glb.h"
#include "engine/procgen/city/lot_cache.h"
#include "engine/model_importer.h"
#include "engine/procgen/city/road_network.h"
#include "engine/procgen/city/polygon.h"
#include "apps/citysim/city_sim.h"
#include "engine/ai/nav_graph.h"
#include "engine/procgen/city/roads/road_model.h"   // plannedRoadModel: the street map's source (ADR-0425)
#include "engine/procgen/city/roads/street_map.h"   // appendStreetMap
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

// THE REGRESSION THAT WOULD HAVE CAUGHT "half on the sidewalk" (parking round), on the road the city DRIVES:
// the street map a lane-built city's sim runs on (ADR-0425: built from the road model at load, each street's kerb
// parking band from its stated cross-section, streetSection). Every bay the sim mints lives inside the
// Parking band of its own road -- never a bare travel lane, the stall between the last travel lane and the kerb,
// the widest car in the fleet fitting inside the band. (It ran on the old metro grower's generate recipe until
// that retired, ADR-0267, then on the road twin's nav until ADR-0425.) On a planned city: the lab scenes' streets carry
// two travel lanes in 12 m and have no room for a band, while a planned city's local street is one 6 m lane each way,
// parked along. (river_town: its road file states its spread junctions' points, which a street map needs.)
TEST_CASE(lanes_parking_bays_live_inside_the_road_s_parking_band) {
    std::size_t totalBays = 0;
    for (const char* name : {"planned/river_town_lanelab"}) {
        RoadLabGraph g = RoadLabGraph::load(std::string(RT_SOURCE_DIR) + "/assets/lanelab/" + name + ".json");
        (void)expand(g);   // the ramps composed, as the loader reads them
        engine::NavGraph nav;
        engine::StreetMapReport rep;
        CHECK(engine::appendStreetMap(nav, engine::plannedRoadModel(g), 0, nullptr, &rep));
        citysim::CitySim sim;
        sim.build(nav, 0, 0, 5);
        const auto& bays = sim.parkingBays();
        engine::Real widestCar = 0;
        for (int i = 0; i < citysim::vehicleFleetSize(); ++i) widestCar = std::max(widestCar, citysim::vehicleFleetBody(i).width);
        engine::Real worstOut = 0, worstIn = 0;
        for (const citysim::CitySim::ParkingBay& b : bays) {
            const engine::NavLink& L = nav.links[b.link];
            CHECK(L.parkWidth > 0);                       // never a bare travel lane
            CHECK_APPROX(b.width, L.parkWidth, 1e-9);
            const engine::Vec2 dir = nav.direction(b.link);
            const engine::Vec2 rel = b.pos - nav.nodes[L.from];
            const engine::Real lat = rel.x * dir.y - rel.y * dir.x;   // + = right of travel
            const engine::Real edge = L.width * 0.5;                  // kerb line
            const engine::Real inner = edge - L.parkWidth;            // travel-lane boundary
            CHECK_APPROX(lat, (inner + edge) * 0.5, 1e-6);           // the bay centre IS the band centre
            worstOut = std::max(worstOut, lat + b.width * 0.5 - edge);
            worstIn = std::max(worstIn, inner - (lat - b.width * 0.5));
            CHECK(lat + widestCar * 0.5 <= edge + 1e-9);             // a real car parked there fits the band too
            CHECK(lat - widestCar * 0.5 >= inner - 1e-9);
        }
        std::printf("[parking] %s: %zu bays; widest fleet car %.2f m; worst overhang: kerb %.4f m, travel lane %.4f m\n",
                    name, bays.size(), widestCar, worstOut, worstIn);
        CHECK(!bays.empty());      // the streets really are parked-along
        CHECK(worstOut <= 1e-9);   // NOT half on the sidewalk
        CHECK(worstIn <= 1e-9);    // NOT half in the travel lane
        totalBays += bays.size();
    }
    std::printf("[parking] %zu bays\n", totalBays);
    CHECK(totalBays >= 2000u);   // river_town's street map: 2114 (ADR-0425): a city parked along, not a handful of bays
}

// The engine's OWN block/lot pass on the pavement's holes (headless), audited: on the clean scenes no block may carry
// a thin protrusion (an 8 m opening removes < 20 m²) or sit on built pavement (> 1 m²). This is the test
// Glenn asked for: if blocks look like spikes and snakes in-engine, this must fail.
TEST_CASE(lanes_the_city_block_pass_yields_well_formed_blocks_on_the_clean_scenes) {
    nlohmann::json cs = {{"sidewalk", 3.5}, {"buildLots", true}, {"seed", 3}};
    for (const char* name : {"ring_city", "curve_blocks", "hill_junction", "grid_city"}) {
        BlockAudit a = auditBlocks(scene(name), cs);
        std::printf("    %s: %s", name, summary(a).c_str());
        CHECK(a.blocks.size() >= 1 || std::string(name) == "hill_junction");
        CHECK(a.broken == 0);
        CHECK(a.padConflicts.empty());   // no building pad or block terrace lifts pavement or sidewalk
    }
}

// Lots on a planned city with a freeway, grown the loader's way (the pavement's holes). Two drive-feedback gates
// that ran on the retired graph-face lot pass until ADR-0268:
//   E1/B4 "parks don't sit on their lots and explode into the street including fences": no park or green pad
//     lies on built pavement (over 1 m2);
//   #19 "orphaned houses along the elevated freeway": no lot stands closer to the freeway than to any street
//     while beyond the reach a street lot could front (60 m).
TEST_CASE(lanes_planned_city_lots_keep_parks_off_the_pavement_and_front_streets_not_the_freeway) {
    const Result& r = scene("planned/metro_v2_test_lanelab");
    BlockAudit a = auditBlocks(r, {{"sidewalk", 4.0}, {"buildLots", true}, {"seed", 5}});
    // the build's roads by class: the freeway carriageways and ramps, and every street a building may front
    // (tracks and footways are neither)
    std::vector<std::vector<Vec2>> streets, freeway;
    for (const EdgeSpec& e : r.graph.edges) {
        if (e.xy.size() < 2) continue;
        const RoadClassSpec& spec = r.graph.cls(e);
        if (spec.track || spec.footway) continue;
        const engine::RoadClass k = classOf(r.graph, e);
        (k == engine::RoadClass::Freeway || k == engine::RoadClass::Ramp ? freeway : streets).push_back(e.xy);
    }
    CHECK(!freeway.empty());   // the case is about a freeway: the planned city must carry one
    auto nearest = [](const std::vector<std::vector<Vec2>>& lines, const Vec2& p) {
        double best = 1e30;
        for (const std::vector<Vec2>& line : lines)
            for (size_t i = 0; i + 1 < line.size(); ++i) {
                const Vec2 A = line[i], B = line[i + 1], ab = B - A;
                const double l2 = ab.lengthSquared(); const double t = l2 > 1e-9 ? std::max(0.0, std::min(1.0, dot(p - A, ab) / l2)) : 0.0;
                best = std::min(best, (p - (A + ab * t)).length());
            }
        return best;
    };
    int parks = 0, parksOnPavement = 0, lots = 0, orphans = 0;
    for (const engine::LotBuilding& lb : a.grown.lots) {
        if (lb.recipe.rfind("underfreeway", 0) == 0) continue;   // in the right-of-way by design
        if (lb.type == "park" || lb.type == "green") {
            ++parks;
            if (lb.pad.size() >= 3 && setArea(intersectSets(fromRing(Ring(lb.pad.begin(), lb.pad.end())), r.pavement.surface)) > 1.0) ++parksOnPavement;
            continue;
        }
        ++lots;
        const double toStreet = nearest(streets, lb.site), toFreeway = nearest(freeway, lb.site);
        if (toStreet > 60.0 && toFreeway < toStreet) {
            if (orphans < 8) std::printf("    [lots]   fronts only the freeway: %s/%s at (%.0f, %.0f) block %d, %.0f m to a street, %.0f m to the freeway\n", lb.type.c_str(), lb.recipe.c_str(), lb.site.x, lb.site.y, lb.block, toStreet, toFreeway);
            ++orphans;
        }
    }
    std::printf("    [lots] %d parks (%d on pavement), %d lots (%d fronting only the freeway)\n", parks, parksOnPavement, lots, orphans);
    CHECK(parks > 0);
    CHECK(parksOnPavement == 0);
    CHECK(lots > 0);
    CHECK(orphans == 0);
}

// ---- the city producer (ADR-0084) ------------------------------------------------------------------------

namespace {
bool sameF(const std::vector<float>& a, const std::vector<float>& b) { return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin()); }
// MESH v2 (ADR-0224): tangents and tints as the GPU reads them (the oct snorm16 pair's direction, unorm8), each triangle
// the same, wound the same, though the index codec may start it at another corner
bool sameAsUploaded(const engine::bundle::PackedMesh& a, const engine::bundle::PackedMesh& d) {
    if (a.tan.size() != d.tan.size() || a.col.size() != d.col.size() || a.idx.size() != d.idx.size()) return false;
    for (size_t i = 0; i + 2 < a.tan.size(); i += 3) {
        int16_t qa[2], qd[2];
        engine::octEncode(a.tan[i], a.tan[i + 1], a.tan[i + 2], qa); engine::octEncode(d.tan[i], d.tan[i + 1], d.tan[i + 2], qd);
        double ax, ay, az, dx, dy, dz;   // the direction the GPU decodes (two codes on the fold edge decode alike)
        engine::octDecode(qa, ax, ay, az); engine::octDecode(qd, dx, dy, dz);
        if (std::abs(ax - dx) > 1e-12 || std::abs(ay - dy) > 1e-12 || std::abs(az - dz) > 1e-12) return false;
    }
    for (size_t i = 0; i < a.col.size(); ++i) if (engine::unorm8(a.col[i]) != engine::unorm8(d.col[i])) return false;
    for (size_t t = 0; t + 2 < a.idx.size(); t += 3) {
        bool one = false;
        for (int k = 0; k < 3 && !one; ++k)
            one = d.idx[t] == a.idx[t + k] && d.idx[t + 1] == a.idx[t + (k + 1) % 3] && d.idx[t + 2] == a.idx[t + (k + 2) % 3];
        if (!one) return false;
    }
    return true;
}
bool sameRing(const std::vector<Vec2>& a, const std::vector<Vec2>& b) { if (a.size() != b.size()) return false; for (size_t i = 0; i < a.size(); ++i) if (a[i].x != b[i].x || a[i].y != b[i].y) return false; return true; }
}  // namespace

TEST_CASE(lanes_city_products_round_trip_through_a_bundle) {   // to the byte, but for the meshes' GPU-only channels
    using namespace engine::bundle;
    const Result& r = scene("ring_city");
    CityProducts p = cityProductsFromResult(r, 250.0);
    CHECK(p.hasTerrain == r.hasTerrain); CHECK(!p.cells.empty()); CHECK(!p.holes.empty());
    size_t tris = 0; for (const CityCellMesh& c : p.cells) tris += c.mesh.triangleCount(); CHECK(tris > 100000);
    BundleWriter w; w.openMemory(); writeCityProducts(w, 0, p); nlohmann::json manifest = nlohmann::json::object(); std::string err; CHECK(w.finish(manifest, &err));
    std::unique_ptr<Bundle> b = Bundle::fromMemory(w.memory(), &err); CHECK(b != nullptr); if (!b) return;
    CityProducts q; CHECK(readCityProducts(*b, 0, q, &err)); if (!err.empty()) std::printf("    %s\n", err.c_str());
    CHECK(q.hasTerrain == p.hasTerrain && q.cell == p.cell && q.ground.z == p.ground.z && q.ground.nx == p.ground.nx);
    CHECK(q.holes.size() == p.holes.size()); for (size_t i = 0; i < p.holes.size() && i < q.holes.size(); ++i) CHECK(sameRing(p.holes[i], q.holes[i]));
    CHECK(q.cells.size() == p.cells.size());
    std::map<std::tuple<std::string, int, int>, const CityCellMesh*> byKey; for (const CityCellMesh& c : q.cells) byKey[{c.mesh.name, c.cx, c.cz}] = &c;
    int matched = 0;
    for (const CityCellMesh& c : p.cells) {
        auto it = byKey.find({c.mesh.name, c.cx, c.cz}); CHECK(it != byKey.end()); if (it == byKey.end()) continue;
        const PackedMesh& a = c.mesh; const PackedMesh& d = it->second->mesh;
        CHECK(a.flags == d.flags && a.friction == d.friction && a.roughness == d.roughness && sameF(a.pos, d.pos) && sameF(a.nrm, d.nrm) && sameF(a.uv, d.uv) && sameAsUploaded(a, d));
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
    CHECK(b->pavesCity());
    CHECK(engine::roads::roadBuilderFor(plain.level["entities"][static_cast<std::size_t>(idx)]["road"]).pavesCity());
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
    BlockAudit a = auditBlocks(scene("ring_city"), level.value("citysim", nlohmann::json::object()));
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
    LotsCityInputs city; city.hasTerrain = p.hasTerrain; city.ground = p.ground; city.holes = p.holes; city.pavedSidewalk = p.bands.sidewalkWidth;
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
    LotsCityInputs city; city.hasTerrain = p.hasTerrain; city.ground = p.ground; city.holes = p.holes; city.pavedSidewalk = p.bands.sidewalkWidth;
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

// PAINT SITS FLUSH ON HILLS (#244). Road paint is thin flat geometry laid on the road's triangles. Lifting only its
// corners onto the road left each flat paint triangle bridging the folds of the road under it: sunk into the road over
// a crest, floating over a dip. MeshBuilder::drapeOnMesh cuts it along the road's triangle edges so every piece lies
// on one road triangle. Three road surfaces that fold under the paint -- a crest, a curve whose bank grows round it,
// a ramp that sags into its climb while its cross-slope turns over -- and on each the gap between paint and road,
// measured at every paint triangle's corners, edge middles, middle and three points inside, must be the lift to a
// millimetre. The same paint with only its corners lifted must NOT pass, or the scene has no fold to find.
namespace {
using RoadTris = std::vector<std::array<Vec3, 3>>;
// A road sheet from a lattice of points at(i, j), i along the road, j across it: two triangles a cell, the diagonal
// alternating cell to cell as a deck's triangulation does.
RoadTris roadSheet(int along, int across, const std::function<Vec3(int, int)>& at) {
    RoadTris out;
    for (int i = 0; i + 1 < along; ++i)
        for (int j = 0; j + 1 < across; ++j) {
            const Vec3 a = at(i, j), b = at(i + 1, j), c = at(i + 1, j + 1), d = at(i, j + 1);
            if ((i + j) % 2 == 0) { out.push_back({a, b, c}); out.push_back({a, c, d}); }
            else { out.push_back({a, b, d}); out.push_back({b, c, d}); }
        }
    return out;
}
// The road's height at plan point q (the triangle under it nearest `near`); NaN off the road.
double roadHeightAt(const RoadTris& road, const Vec2& q, double near) {
    constexpr double kOnEdge = 1e-9;
    double best = std::nan("");
    for (const std::array<Vec3, 3>& t : road) {
        const Vec2 a(t[0].x, t[0].z), b(t[1].x, t[1].z), c(t[2].x, t[2].z);
        const double det = cross(b - a, c - a);
        if (std::fabs(det) < 1e-12) continue;
        const double u = cross(q - a, c - a) / det, v = cross(b - a, q - a) / det;
        if (u < -kOnEdge || v < -kOnEdge || u + v > 1 + kOnEdge) continue;
        const double z = t[0].y + u * (t[1].y - t[0].y) + v * (t[2].y - t[0].y);
        if (std::isnan(best) || std::fabs(z - near) < std::fabs(best - near)) best = z;
    }
    return best;
}
// Paint stripes along plan polylines, `halfWidth` each side, every corner ON the road (as the old fit left it).
RenderMesh paintOn(const RoadTris& road, const std::vector<std::vector<Vec2>>& lines, double halfWidth, double near) {
    RenderMesh paint;
    const Vec3 white(0.9, 0.9, 0.88);
    for (const std::vector<Vec2>& line : lines)
        for (std::size_t k = 0; k + 1 < line.size(); ++k) {
            const Vec2 d = normalize(line[k + 1] - line[k]), side(-d.y * halfWidth, d.x * halfWidth);
            const Vec2 q[4] = {line[k] - side, line[k + 1] - side, line[k + 1] + side, line[k] + side};
            Vec3 p[4];
            for (int i = 0; i < 4; ++i) p[i] = Vec3(q[i].x, roadHeightAt(road, q[i], near), q[i].y);
            MeshBuilder::emitQuad(paint, p[0], p[1], p[2], p[3], Vec3(0, 1, 0), white);
        }
    return paint;
}
struct PaintGap { double worst = 0; double paintArea = 0; int offRoad = 0; int facingDown = 0; };
// The worst |paint - road - lift| over the paint, sampled through every triangle, and the paint's plan area.
PaintGap paintGap(const RenderMesh& paint, const RoadTris& road, double lift) {
    PaintGap g;
    const double w[][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0.5, 0.5, 0}, {0, 0.5, 0.5}, {0.5, 0, 0.5},
                           {1.0 / 3, 1.0 / 3, 1.0 / 3}, {0.7, 0.15, 0.15}, {0.15, 0.7, 0.15}, {0.15, 0.15, 0.7}};
    for (std::size_t t = 0; t + 2 < paint.indices.size(); t += 3) {
        const Vertex& A = paint.vertices[paint.indices[t]];
        const Vertex& B = paint.vertices[paint.indices[t + 1]];
        const Vertex& C = paint.vertices[paint.indices[t + 2]];
        const Vec3 front = cross(C.position - A.position, B.position - A.position);
        if (front.y <= 0 || A.normal.y <= 0) ++g.facingDown;
        g.paintArea += 0.5 * std::fabs(front.y);
        for (const double* s : w) {
            const Vec3 p = A.position * s[0] + B.position * s[1] + C.position * s[2];
            const double h = roadHeightAt(road, Vec2(p.x, p.z), p.y);
            if (std::isnan(h)) { ++g.offRoad; continue; }
            g.worst = std::max(g.worst, std::fabs(p.y - h - lift));
        }
    }
    return g;
}
}  // namespace

TEST_CASE(lanes_paint_lies_flush_on_a_crest_a_banked_curve_and_a_ramp) {
    constexpr double kMm = 0.001;
    struct Scene { const char* name; RoadTris road; std::vector<std::vector<Vec2>> lines; double near; };
    std::vector<Scene> scenes;
    {   // (a) A CREST: 80 m over a 20 m summit, 2.5 m cells; three lane lines and a stop bar across the top. A bridge
        // deck crosses 6 m over the summit: the paint must stay on the street under it.
        Scene s{"crest", roadSheet(33, 5, [](int i, int j) { const double x = 2.5 * i; return Vec3(x, 20.0 - 0.004 * (x - 40.0) * (x - 40.0), -5.0 + 2.5 * j); }), {}, 20.0};
        for (double z : {-3.5, 0.0, 3.5}) { std::vector<Vec2> l; for (double x = 1.0; x <= 79.0; x += 6.0) l.push_back(Vec2(x, z)); s.lines.push_back(l); }
        s.lines.push_back({Vec2(41.3, -4.6), Vec2(41.3, 4.6)});
        const RoadTris bridge = roadSheet(2, 2, [](int i, int j) { return Vec3(36.0 + 8.0 * i, 26.0, -30.0 + 60.0 * j); });
        RoadTris both = s.road; both.insert(both.end(), bridge.begin(), bridge.end());
        s.road = both;
        scenes.push_back(s);
    }
    {   // (b) A BANKED CURVE: 60 m radius through a radian, climbing 3%, its bank growing from level to 10% round it
        Scene s{"banked curve", roadSheet(51, 5, [](int i, int j) {
            const double th = 0.02 * i, off = -5.0 + 2.5 * j, r = 60.0 + off;
            return Vec3(r * std::cos(th), 0.03 * 60.0 * th + 0.10 * th * off, r * std::sin(th));
        }), {}, 0.0};
        for (double off : {-3.5, 0.0, 3.5}) {
            std::vector<Vec2> l;
            for (double th = 0.01; th <= 0.99; th += 0.1) l.push_back(Vec2((60.0 + off) * std::cos(th), (60.0 + off) * std::sin(th)));
            s.lines.push_back(l);
        }
        scenes.push_back(s);
    }
    {   // (c) A RAMP: level, then sagging into an 8% climb, its cross-slope turning from -3% to +3% along it
        Scene s{"ramp", roadSheet(34, 5, [](int i, int j) {
            const double x = 3.0 * i, z = -4.0 + 2.0 * j, rise = x < 30.0 ? 0.0 : x < 80.0 ? 0.0008 * (x - 30.0) * (x - 30.0) : 2.0 + 0.08 * (x - 80.0);
            return Vec3(x, rise + (-0.03 + 0.06 * x / 99.0) * z, z);
        }), {}, 0.0};
        for (double z : {-3.0, 0.0, 3.0}) { std::vector<Vec2> l; for (double x = 0.5; x <= 98.5; x += 7.0) l.push_back(Vec2(x, z)); s.lines.push_back(l); }
        scenes.push_back(s);
    }
    {   // A ROAD THAT DOES NOT FOLD (an even 5% grade, 2% cross-fall): its paint keeps its triangles
        const RoadTris road = roadSheet(34, 5, [](int i, int j) { const double x = 3.0 * i, z = -4.0 + 2.0 * j; return Vec3(x, 0.05 * x + 0.02 * z, z); });
        std::vector<std::vector<Vec2>> lines;
        for (double z : {-3.0, 0.0, 3.0}) { std::vector<Vec2> l; for (double x = 0.5; x <= 98.5; x += 7.0) l.push_back(Vec2(x, z)); lines.push_back(l); }
        const RenderMesh flat = paintOn(road, lines, 0.08, 0.0);
        const RenderMesh draped = MeshBuilder::drapeOnMesh(flat, road, kPaintLift, kPaintReach, kPaintFlush);
        CHECK(draped.indices.size() == flat.indices.size());
        CHECK(paintGap(draped, road, kPaintLift).worst <= kMm);
    }
    for (const Scene& s : scenes) {
        const RenderMesh flat = paintOn(s.road, s.lines, 0.08, s.near);
        // the corners-only fit: every corner on the road and lifted, the triangles between them flat
        RenderMesh cornersOnly = flat;
        for (Vertex& v : cornersOnly.vertices) v.position.y += kPaintLift;
        const PaintGap before = paintGap(cornersOnly, s.road, kPaintLift);
        const RenderMesh draped = MeshBuilder::drapeOnMesh(flat, s.road, kPaintLift, kPaintReach, kPaintFlush);
        const PaintGap after = paintGap(draped, s.road, kPaintLift);
        std::printf("  %s: paint %zu -> %zu triangles; worst gap off the lift: corners only %.1f mm, draped %.4f mm; area %.3f -> %.3f m2\n",
                    s.name, flat.indices.size() / 3, draped.indices.size() / 3, before.worst * 1000.0, after.worst * 1000.0,
                    before.paintArea, after.paintArea);
        CHECK(before.worst > kMm);                                       // the scene has folds under its paint
        CHECK(after.worst <= kMm);                                       // ...and the drape follows every one of them
        CHECK(after.offRoad == 0 && after.facingDown == 0);
        CHECK(std::fabs(after.paintArea - before.paintArea) < 1e-6 * before.paintArea + 1e-6);   // nothing trimmed, nothing twice
        for (const Vertex& v : draped.vertices) CHECK_APPROX(v.color.x, 0.9, 1e-9);
    }
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
            if (pg.outer.size() < 3 || std::fabs(signedArea(pg.outer)) < 1.0) continue;
            mine.push_back(pg.outer);
            std::vector<engine::Vec3> poly; for (const engine::Vec2& q : pg.outer) poly.push_back(engine::Vec3(q.x, 0, q.y));
            slow.push_back(engine::makeFlattenPad(std::move(poly), lb.groundY, falloff));
        }
        // ...and the pad holds its walls: a plan corner off the clipped pad makes the plan a pad too
        bool held = true;
        for (const engine::Vec2& v : lb.plan) {
            bool in = false;
            for (const Ring& r : mine) in = in || pointInPolygon(r, v);
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

// CROSSINGS AS PLANNED (ADR-0215, Glenn: "neither system is guessing"): a graph that declares its crossings gets the
// kind it planned, not one read off the heights. Two local streets cross at (0, 0); "a" is held 5 m up there.
namespace {
nlohmann::json crossingGraph(const nlohmann::json& crossings, bool holdA) {
    nlohmann::json g;
    g["name"] = "planned_crossing";
    nlohmann::json a = {{"id", "a"}, {"class", "local"}, {"path", {{"type", "polyline"}, {"points", {{-300, 0}, {300, 0}}}}}};
    if (holdA) a["floor"] = {{0.0, 0.0, 5.0, 20.0}};
    g["edges"] = nlohmann::json::array({a, {{"id", "b"}, {"class", "local"}, {"path", {{"type", "polyline"}, {"points", {{0, -300}, {0, 300}}}}}}});
    if (!crossings.is_null()) g["crossings"] = crossings;
    return g;
}
double zAt(const Result& r, const char* id, double s) { const EdgeSpec& e = *r.graph.find(id); return interp(e.s, e.z, s); }
const Check* checkNamed(const std::vector<Check>& cs, const std::string& n) { for (const Check& c : cs) if (c.name == n) return &c; return nullptr; }
}

TEST_CASE(lanes_a_planned_level_crossing_is_levelled_however_far_apart_it_stood) {
    // unplanned, 5 m apart reads as a bridge (>= bridge_h): the streets are left apart
    const std::unique_ptr<Result> guessed = build(RoadLabGraph::fromJson(crossingGraph(nullptr, true)));
    const double dzGuessed = std::fabs(zAt(*guessed, "a", 300.0) - zAt(*guessed, "b", 300.0));
    // planned at grade: they meet
    const std::unique_ptr<Result> planned = build(RoadLabGraph::fromJson(crossingGraph(
        nlohmann::json::array({{{"a", "a"}, {"b", "b"}, {"at", {0, 0}}, {"kind", "at_grade"}}}), true)));
    const double dzPlanned = std::fabs(zAt(*planned, "a", 300.0) - zAt(*planned, "b", 300.0));
    std::printf("    [planned level] dz unplanned %.2f m, planned %.3f m\n", dzGuessed, dzPlanned);
    CHECK(dzGuessed > 4.0);
    CHECK(dzPlanned < 0.05);
    const std::vector<Check> checks = invariants(*planned);
    const Check* asPlanned = checkNamed(checks, "crossings are as planned");
    CHECK(asPlanned != nullptr && asPlanned->ok);
}

TEST_CASE(lanes_a_planned_bridge_clears_a_road_it_stood_level_with) {
    const std::unique_ptr<Result> guessed = build(RoadLabGraph::fromJson(crossingGraph(nullptr, false)));
    const std::unique_ptr<Result> planned = build(RoadLabGraph::fromJson(crossingGraph(
        nlohmann::json::array({{{"a", "a"}, {"b", "b"}, {"at", {0, 0}}, {"kind", "bridge"}, {"over", "b"}}}), false)));
    const double dzGuessed = zAt(*guessed, "b", 300.0) - zAt(*guessed, "a", 300.0);
    const double dzPlanned = zAt(*planned, "b", 300.0) - zAt(*planned, "a", 300.0);
    std::printf("    [planned bridge] b over a: unplanned %.2f m, planned %.2f m\n", dzGuessed, dzPlanned);
    CHECK(std::fabs(dzGuessed) < 0.05);   // flat ground, no plan: a level crossing
    const Rules R;
    CHECK(dzPlanned >= R.underClearance + 0.5 + R.structureDepth - 0.05);   // b over a, its slab and girders clear
}

TEST_CASE(lanes_a_crossing_the_plan_left_out_is_reported_with_its_place) {
    // the graph declares its crossings, but the list is empty: (0, 0) is found and named
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(crossingGraph(nlohmann::json::array(), false)));
    const std::vector<Check> checks = invariants(*r);
    const Check* c = checkNamed(checks, "crossings are as planned");
    CHECK(c != nullptr);
    if (!c) return;
    std::printf("    [unplanned] %s\n", c->detail.c_str());
    CHECK(!c->ok);
    CHECK(c->detail.find("1 of the crossings found") != std::string::npos && c->detail.find("a / b at (0, 0)") != std::string::npos);
}

// PLANNED ELEVATIONS (ADR-0215): designedGraph writes every road's profile into the graph; built again, the builder
// takes the heights as given -- the same decks as building it straight -- and checks them, never re-designs them.
TEST_CASE(lanes_a_designed_graph_builds_the_heights_it_was_designed_with) {
    nlohmann::json spec;
    { std::ifstream f(std::string(RT_SOURCE_DIR) + "/assets/lanelab/freeway_cross_relief.json"); f >> spec; }
    const std::unique_ptr<Result> straight = build(RoadLabGraph::fromJson(spec, std::string(RT_SOURCE_DIR) + "/assets/lanelab"));
    const nlohmann::json designed = designedGraph(spec, std::string(RT_SOURCE_DIR) + "/assets/lanelab");
    const std::unique_ptr<Result> planned = build(RoadLabGraph::fromJson(designed, std::string(RT_SOURCE_DIR) + "/assets/lanelab"));
    CHECK(planned->graph.designed() && !straight->graph.designed());
    double worst = 0;
    for (const EdgeSpec& e : straight->graph.edges) {
        const EdgeSpec* o = planned->graph.find(e.id);
        CHECK(o != nullptr && o->z.size() == e.z.size());
        if (!o || o->z.size() != e.z.size()) continue;
        for (std::size_t i = 0; i < e.z.size(); ++i) worst = std::max(worst, std::fabs(o->z[i] - e.z[i]));
    }
    std::printf("    [designed] %zu edges, worst height difference %.6f m\n", straight->graph.edges.size(), worst);
    CHECK(worst == 0.0);
    CHECK(planned->pavement.decks.size() == straight->pavement.decks.size());
}

TEST_CASE(lanes_a_planned_profile_is_built_as_given_and_its_faults_are_named) {
    nlohmann::json designed = designedGraph(crossingGraph(nullptr, false));
    // a 3 m hump in "a" over the crossing at its middle: the builder keeps it, and the checks say what is wrong
    for (nlohmann::json& e : designed["edges"])
        if (e["id"] == "a") {
            std::vector<double> z = e["profile"]["z"].get<std::vector<double>>();
            for (std::size_t i = 0; i < z.size(); ++i) if (std::fabs(static_cast<double>(i) - 300.0) <= 20.0) z[i] += 3.0;
            e["profile"]["z"] = z;
        }
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(designed));
    const double hump = zAt(*r, "a", 300.0) - zAt(*r, "a", 250.0);
    std::printf("    [as given] a's hump %.2f m at the crossing\n", hump);
    CHECK(hump > 2.9);   // not re-designed away
    const std::vector<Check> checks = invariants(*r);
    const Check* junctions = checkNamed(checks, "junction heights consistent");
    const Check* grades = checkNamed(checks, "grades within class limits");
    CHECK(junctions != nullptr && !junctions->ok);   // a and b no longer meet at the crossing
    CHECK(grades != nullptr && !grades->ok);         // and a climbs 3 m in a metre
    if (junctions) std::printf("    [as given] %s\n", junctions->detail.c_str());
    CHECK(junctions != nullptr && junctions->detail.find("at (0, 0)") != std::string::npos);   // named where
}

// THE PLANNER'S CROSSING POLICY (ADR-0215 step 3): each meeting of two roads gets its kind from what the two are --
// a freeway or a track is crossed on a bridge, a footway crosses on one but meets the street it starts from at grade,
// two streets meet at grade -- and designing the graph names each bridge's upper road.
TEST_CASE(lanes_the_crossing_policy_plans_each_meeting_by_what_meets) {
    nlohmann::json g;
    g["name"] = "policy";
    g["rules"] = {{"freeway_separates", true}};
    auto line = [](const char* id, const char* cls, std::vector<std::array<double, 2>> pts) {
        nlohmann::json p = nlohmann::json::array();
        for (const auto& q : pts) p.push_back({q[0], q[1]});
        return nlohmann::json{{"id", id}, {"class", cls}, {"path", {{"type", "polyline"}, {"points", p}}}};
    };
    g["edges"] = nlohmann::json::array({
        line("fwy", "freeway", {{-600, 0}, {600, 0}}),
        line("main", "arterial", {{0, -400}, {0, 400}}),                 // crosses the freeway
        line("side", "local", {{-300, 200}, {0, 200}}),                  // ends on main's interior: a tee
        line("rail", "rail", {{-400, -200}, {400, -200}}),               // crosses main
        line("way", "footway", {{0, -150}, {150, -150}, {150, -260}})});  // starts on main, crosses the rail
    const nlohmann::json planned = planCrossings(RoadLabGraph::fromJson(g));
    std::map<std::string, std::string> kind;
    for (const auto& c : planned) {
        std::string a = c["a"], b = c["b"];
        if (b < a) std::swap(a, b);
        kind[a + "/" + b] = c["kind"];
        std::printf("    [policy] %s / %s at (%.0f, %.0f): %s\n", a.c_str(), b.c_str(), c["at"][0].get<double>(), c["at"][1].get<double>(), c["kind"].get<std::string>().c_str());
    }
    CHECK(kind["fwy/main"] == "bridge");
    CHECK(kind["main/side"] == "at_grade");
    CHECK(kind["main/rail"] == "bridge");
    CHECK(kind["main/way"] == "at_grade");   // the footway's way in starts at the street
    CHECK(kind["rail/way"] == "bridge");     // and crosses the track on a footbridge
    // designed: every bridge names its upper road, and nothing is left out of the plan
    g["crossings"] = planned;
    DesignReport rep;
    const nlohmann::json designed = designedGraph(g, ".", &rep);
    CHECK(rep.undeclared.empty());
    for (const auto& c : designed["crossings"])
        if (c["kind"] == "bridge") CHECK(!c.value("over", std::string()).empty());
}

// A FREEWAY AT GRADE WEARS ITS GUARDRAILS (2026-10-21, Glenn: "freeways don't have railguards anymore. Neither do
// the onramps"): left to the class defaults, a freeway on flat ground carries a guardrail down both outer edges;
// the drop rule the island's open roads author (a rail only where the ground 5 m out falls 2.5 m) builds none.
TEST_CASE(lanes_a_freeway_at_grade_wears_its_guardrails) {
    auto railMetres = [](const nlohmann::json& g) {
        const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
        ParapetCensus census;
        double rail = 0;
        for (const ParapetRun& run : parapetRuns(*r, &census))
            if (run.kind == BarrierKind::Guardrail)
                for (size_t i = 0; i + 1 < run.pts.size(); ++i) rail += distance(run.pts[i], run.pts[i + 1]);
        std::printf("    [rails] at grade %.0f of %.0f m built, guardrail %.0f m\n", census.built[static_cast<size_t>(EdgeRole::AtGrade)],
                    census.metres[static_cast<size_t>(EdgeRole::AtGrade)], rail);
        return rail;
    };
    nlohmann::json g;
    g["name"] = "rails";
    g["edges"] = nlohmann::json::array({{{"id", "fwy"}, {"class", "freeway"}, {"path", {{"type", "polyline"}, {"points", {{-300, 0}, {300, 0}}}}}}});
    CHECK(railMetres(g) > 2 * 600 * 0.9);   // both sides, nearly end to end
    g["classes"]["freeway"]["edges"]["at_grade"] = {{"kind", "guardrail"}, {"min_drop", 2.5}};
    CHECK(railMetres(g) < 1.0);             // flat ground: nowhere to fall
}

// A CITY BLOCK STAYS BEHIND ITS SIDEWALK (2026-10-21, Glenn: "there are green patches so the road is not conforming
// right"): a block is the hole in the drawn pavement, inset a little and simplified -- and the simplified outline may
// not cut back across the sidewalk on the outside of a bend, or every pad and its feather laid to it lifts the ground
// over the sidewalk.
TEST_CASE(lanes_a_city_block_stays_behind_its_sidewalk) {
    for (const char* name : {"curve_blocks", "ring_city", "grid_city"}) {
        const Result& r = scene(name);
        const PolySet paved = unionSets(unionSets(r.pavement.surface, r.pavement.shoulder), r.pavement.sidewalk);
        const std::vector<Ring> holes = pavementHoles(r, 2000.0);
        const std::vector<Poly2> blocks = blocksFromHoles(holes, 1.5, kBlockMarginBehindSidewalk, kMinBlockWidth);
        PolySet all;
        for (const Poly2& b : blocks) { for (Polygon2& pg : fromRing(b)) all.push_back(std::move(pg)); }
        const double over = setArea(intersectSets(unionSets(all, PolySet{}), paved)), area = setArea(unionSets(all, PolySet{}));
        // ...and a curve stays a few long frontages: clipping the chords to the hole gave every curve back as
        // ~1 m pieces no lot could front (9% of the island's buildings).
        double perim = 0, shortRun = 0;
        for (const Poly2& b : blocks)
            for (size_t i = 0; i < b.size(); ++i) {
                const double L = std::hypot(b[(i + 1) % b.size()].x - b[i].x, b[(i + 1) % b.size()].y - b[i].y);
                perim += L; if (L < 4.0) shortRun += L;
            }
        std::printf("    [blocks] %s: %zu blocks, %.0f m2, %.1f m2 of it over the drawn pavement, %.1f%% of the outline in pieces under 4 m\n",
                    name, blocks.size(), area, over, 100.0 * shortRun / std::max(perim, 1.0));
        CHECK(over < 1.0);
        CHECK(shortRun < 0.03 * perim);
    }
}

// THE ROADS KEEP THEIR GROUND, THE HILLS GET THEIRS (ADR-0221, Glenn: "the hills aren't smooth even after all this
// work to increase the terrain detail"): the road build conforms its own copy of the ground at its grid's cell (20 m
// on the island), and the terrain drawn from that copy lost everything finer. The ground a lane city stands on is the
// build's grid in a corridor along the roads (the nodes it changed, or a deck reaches) and the natural ground's full
// detail away from them.
TEST_CASE(lanes_the_road_grid_keeps_a_corridor_and_the_hills_their_detail) {
    nlohmann::json root;
    root["terrain"] = {{"seed", 3}, {"heightScale", 40.0}, {"noiseScale", 0.02}, {"octaves", 6}};
    const TerrainParams tp = readTerrainParams(root["terrain"]);
    const Noise nz(3);
    auto natural = [&](double x, double z) { return terrainHeight(tp, nz, x, z); };
    auto grid = std::make_shared<HeightGrid>();
    grid->x0 = 0; grid->y0 = 0; grid->res = 20; grid->nx = 31; grid->ny = 31;
    grid->z.resize(static_cast<std::size_t>(grid->nx) * grid->ny);
    for (int j = 0; j < grid->ny; ++j)
        for (int i = 0; i < grid->nx; ++i) grid->at(i, j) = natural(i * 20.0, j * 20.0);
    grid->at(15, 15) += 3.0;   // a road's fill at one node
    const auto base = laneErodedBase(root, grid, nullptr);
    CHECK(std::fabs((*base)(300, 300) - grid->at(15, 15)) < 1e-3);           // the road's node: the grid exactly
    CHECK(std::fabs((*base)(305.3, 296.1) - grid->sample(305.3, 296.1)) < 1e-3);   // in its corridor: the build's ground
    double lost = 0, kept = 0;   // far from the road (and the edge band): the natural ground, not the grid's blur
    for (double z = 61.3; z < 160; z += 7.1)
        for (double x = 62.9; x < 160; x += 6.7) {
            lost = std::max(lost, std::fabs(grid->sample(x, z) - natural(x, z)));
            kept = std::max(kept, std::fabs((*base)(x, z) - natural(x, z)));
        }
    std::printf("    [road grid] away from the road: the grid alone was off the natural ground by up to %.3f m, now %.5f m\n", lost, kept);
    CHECK(lost > 0.05);   // the terrain has detail finer than the grid...
    CHECK(kept < 1e-3);   // ...and keeps it
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
                const RoadClass local = classOf(r.graph, s);
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
    // identical under RT_LAYERS_FLAT=1 (RT_SEE_THROUGH_RAY). #142's along-the-road partner pull (2026-11-10) lifts a
    // low deck at (-38, 70) 0.3 m, which moves the eyes again: one more slab-corner tie there, 7 (the same look,
    // RT_SEE_THROUGH_RAY with the direction normalised, meets the soffit's front in both builds).
    const std::map<std::string, size_t> known = {{"freeway_cross", 0}, {"freeway_cross_relief", 1}, {"light_rail_lab", 0},
                                                 {"rail_loop", 0}, {"system_interchange", 7}, {"valley_viaduct", 5}};
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

// #218 (measured 2026-11-10): what recomputing every city's lanes at LOAD costs -- expand() on each planned graph, the
// builder's own first step (the sim would read the same lanes the bake was built from). Print-only; RT_EXPAND_TIMING.
TEST_CASE(lanes_print_expand_cost_per_city) {
    if (!std::getenv("RT_EXPAND_TIMING")) return;
    for (const char* name : {"traffic_lab", "metro_v2_test", "river_town", "metro_planned", "island_8_nature_lanelab"}) {
        std::string path = std::string(RT_SOURCE_DIR) + "/assets/lanelab/planned/" + name;
        if (path.size() < 5 || path.substr(path.size() - 5) != ".json")
            path += std::string(name).find("_lanelab") == std::string::npos ? "_lanelab.json" : ".json";
        std::ifstream in(path);
        if (!in) { std::printf("    [expand] %s: no graph\n", name); continue; }
        const nlohmann::json j = nlohmann::json::parse(in);
        RoadLabGraph g = RoadLabGraph::fromJson(j);
        const auto t0 = std::chrono::steady_clock::now();
        const LaneSet L = expand(g);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::size_t pts = 0;
        for (const Lane& l : L.lanes) pts += l.xy().size();
        std::printf("    [expand] %-26s %5zu edges -> %6zu lanes, %8zu points: %.1f ms\n", name, g.edges.size(), L.lanes.size(), pts, ms);
    }
}

// ---- THE LANE GRAPH (#237), ITS CONNECTORS (#238): SINCE ADR-0426 A TABLE BUILT FROM THE ROAD MODEL -----------------------
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"
#include "engine/procgen/city/roads/lanes/lane_graph_connect.h"
#include "engine/procgen/city/roads/lane_turns.h"   // the stated turns (the hand-made junctions below)
#include "lane_level_scene.h"                       // a lab scene with the lane level's facts stated in memory
#include "engine/procgen/city/roads/lanes/lane_level.h"   // drivenLanePoints: the line a lane is driven along
#include "engine/procgen/city/roads/road_model.h"   // the traffic lab's road model's own problems
#include "engine/procgen/city/polyline.h"           // segmentsCross: two lanes' centre lines crossing
#include <map>
#include <set>

namespace {
// The vehicle tags the tests open every lane to (tests' names, not kinds of vehicle), with the sizes the connectors are
// checked against: a car-sized one and a long one (turning radius and width, metres).
constexpr const char* kLaneGraphTestTag = "test_any";
constexpr const char* kLaneGraphLongTag = "test_long";
constexpr double kTestCarRadius = 5.0, kTestCarWidth = 1.9;
constexpr double kTestLongRadius = 10.0, kTestLongWidth = 2.6;
// A builder lane's paths may overlap by this much (metres): rounding.
constexpr double kLaneCoverTolerance = 0.01;
// A section's lanes start (and end) within this of the section's start along its road's spine (metres).
constexpr double kSectionShareTolerance = 0.05;
// Two neighbours in a section stand no farther apart than their half widths and this (metres).
constexpr double kNeighbourSlack = 0.5;
// Two paths that meet do so within this (metres) and this (degrees): the builder writes far finer.
constexpr double kTestJoinDistance = 0.01;
constexpr double kTestJoinDegrees = 1.0;
// Two directions' cosines this close are the same direction (rounding).
constexpr double kCosineRounding = 1e-6;
// A join the lane level makes between two lanes is reported when their lines miss by more than this (lane_level.cpp).
constexpr double kTestJoinReported = 1e-3;

// THE SCENE A LANE GRAPH TEST READS: the lab scene with the lane level's facts stated in memory (its layout, controls and
// lane rules: lane_level_scene.h), built once.
const Result& laneSceneOf(const std::string& name) {
    static std::map<std::string, std::unique_ptr<Result>> cache;
    std::map<std::string, std::unique_ptr<Result>>::iterator it = cache.find(name);
    if (it == cache.end())
        it = cache.emplace(name, lane_level_test::buildWithLaneLevelFacts(std::string(RT_SOURCE_DIR) + "/assets/lanelab/" + name + ".json")).first;
    return *it->second;
}

// THE ROAD MODEL of a scene, its lane level with it (RoadModel::laneLevel), and what building the lane level reported:
// assembled once, through the one entry point.
struct SceneRoads {
    RoadModel model;
    LaneGraphBuild report;
};
const SceneRoads& sceneRoadsOf(const std::string& name) {
    static std::map<std::string, SceneRoads> cache;
    std::map<std::string, SceneRoads>::iterator it = cache.find(name);
    if (it == cache.end()) {
        LaneGraphOptions o;
        VehicleSize car, longVehicle;
        car.tag = kLaneGraphTestTag;
        car.turningRadius = kTestCarRadius;
        car.width = kTestCarWidth;
        longVehicle.tag = kLaneGraphLongTag;
        longVehicle.turningRadius = kTestLongRadius;
        longVehicle.width = kTestLongWidth;
        o.vehicles = {car, longVehicle};
        it = cache.emplace(name, SceneRoads()).first;
        it->second.model = assembleRoadModel(laneSceneOf(name), o, &it->second.report);
        if (!it->second.report.error.empty()) std::printf("      [lane graph] %s: %s\n", name.c_str(), it->second.report.error.c_str());
    }
    return it->second;
}
const LaneGraphBuild& laneGraphOf(const std::string& name) { return sceneRoadsOf(name).report; }
// THE ROAD MODEL of a scene (the planned and built layers the lane level is built from, and the lane level itself).
const RoadModel& modelOf(const std::string& name) { return sceneRoadsOf(name).model; }
// How far the points stray from ONE LINE ACROSS THE ROAD through `spineAt` (the line through it and the farthest point;
// all within a few centimetres of it when the points are the starts, or the ends, of a section's lanes).
double offCrossLine(const Vec2& spineAt, const std::vector<Vec2>& pts) {
    Vec2 far = spineAt;
    for (const Vec2& p : pts)
        if (distance(p, spineAt) > distance(far, spineAt)) far = p;
    double worst = 0;
    if (distance(far, spineAt) < 0.5) {
        for (const Vec2& p : pts) worst = std::max(worst, distance(p, spineAt));
        return worst;
    }
    const Vec2 across = normalize(far - spineAt);
    for (const Vec2& p : pts) worst = std::max(worst, std::fabs(cross(across, p - spineAt)));
    return worst;
}

// WHAT A SCENE IS KNOWN TO HOLD that a lane graph check would otherwise call a fault, each WRITTEN HERE from looking at
// the scene (never a number the builder reports): a new one, or one gone, fails the check.
struct LaneGraphKnown {
    std::set<std::string> drivenLeftOut;           // lanes cars drive made into no lane path at all, by builder lane id
    std::map<std::string, std::string> otherGaps;  // a lane's stretch with no lane path outside every junction: lane id -> words of its reason
    std::set<std::string> insideRoads;             // roads the road file states inside a spread junction (their lanes: no lane path)
    std::map<std::string, uint32_t> inside;        // model junction id -> lane paths passing inside its outline
    // joins between two different lanes missing by more than 1 cm or turning more than a degree: "from>into" -> (gap in
    // metres, turn in degrees), held to kKnownJoinGap and kKnownJoinDegrees
    std::map<std::string, std::pair<double, double>> joins;
    // lane paths of two roads crossing at one level outside every junction's stated extent: "a x b @ (x, y)" (builder lane
    // ids in id order, the place to the metre)
    std::set<std::string> crossings;
    std::map<std::string, double> tidied;          // lanes whose fold the lane level cuts out: lane id -> most metres it may cut
    // junction gaps that are NOT that junction's stated stretch (a stop line where neither the paint nor ADR-0419's rule
    // puts it): "lane id @ junction id"
    std::set<std::string> junctionGapsOff;
    uint32_t carryOnJoins = 0, divergeJoins = 0, mergeJoins = 0;   // the joins the model states, made (written from the scene)
    // the stated merges marked on their Join crossing points: "joining lane>host lane" (builder lane ids)
    std::set<std::string> merges;
    // the scene's problem lines by kind (problemKind) and count; `pinProblems` false: pinned by a test of its own
    std::map<std::string, uint32_t> problems;
    bool pinProblems = true;
};
// THE KIND OF A LANE-LEVEL PROBLEM LINE, by its words (for pinning a scene's problems by kind and count).
std::string problemKind(const std::string& p) {
    if (p.find(" fits no vehicle: ") != std::string::npos) return "fits no vehicle";
    if (p.find("; tidied (") != std::string::npos) return "tidied";
    if (p.find("crossing points: ") == 0) return "crossing points";
    if (p.find("kink: ") == 0) return "kink";
    if (p.find(" apart at (") != std::string::npos && p.find("): joined") != std::string::npos) return "join with a gap";
    if (p.find("its outline crosses itself") != std::string::npos) return "outline";
    if (p.find("states no junction there") != std::string::npos) return "no junction stated";
    if (p.find("no room between its stop lines") != std::string::npos) return "no room";
    if (p.find("has no lane path to") != std::string::npos) return "turn with no lane path";
    return "other";
}
constexpr double kKnownJoinGap = 0.002, kKnownJoinDegrees = 0.05;
// A lane path is a slice of its built lane: its ends stand where the built lane is at its stretch's ends, and its length is
// the stretch's, within this (metres: the slice drops points nearer than a millimetre to a cut, and the graph stores its
// points as written; a centimetre is ten times either).
constexpr double kSliceTolerance = 0.01;
// A junction's stretch along a lane runs from the street's end to the arm's stop line as the paint or ADR-0419's rule
// puts it, within this (metres: the road file's stations are written to the centimetre and a hole of a centimetre or two
// between two streets' stations goes with the junction beside it; projecting a lane point onto its road's spine adds
// millimetres on a bend).
constexpr double kJunctionStretchTolerance = 0.05;
// A crossing of two lanes the model joins is theirs to make only this near the place they join (metres).
constexpr double kNearTheirJoin = 5.0;

// EACH JUNCTION'S STATED EXTENT along each road (edge index -> stretches (from, to, model junction)): every arm's stretch
// from its station (the street's end there) to its stop line (ModelArm::stopLine), and every stretch the road file states
// inside the junction.
std::map<int, std::vector<std::tuple<double, double, uint32_t>>> junctionExtents(const Result& r, const RoadModel& m) {
    std::map<int, std::vector<std::tuple<double, double, uint32_t>>> out;
    for (uint32_t j = 0; j < m.junctions.size(); ++j) {
        for (uint32_t a = 0; a < m.junctions[j].arms.size(); ++a) {
            const ModelArm& arm = m.junctions[j].arms[a];
            const std::map<std::string, int>::const_iterator e = r.graph.index.find(arm.road);
            const double stop = arm.stopLine;
            if (e == r.graph.index.end() || !stated(stop)) continue;
            out[e->second].emplace_back(std::min(arm.station, stop), std::max(arm.station, stop), j);
        }
        for (const ModelStretch& st : m.junctions[j].inside) {
            const std::map<std::string, int>::const_iterator e = r.graph.index.find(st.road);
            if (e != r.graph.index.end()) out[e->second].emplace_back(st.from, st.to, j);
        }
    }
    return out;
}

// THE CHECKS EVERY SCENE'S LANE GRAPH MUST PASS (#237's "done when" 1 to 3).
void checkLaneGraph(const std::string& name, const LaneGraphKnown& known = LaneGraphKnown()) {
    const Result& r = laneSceneOf(name);
    const LaneGraphBuild& lg = laneGraphOf(name);
    const LaneGraph& G = modelOf(name).laneLevel;
    CHECK(lg.error.empty());
    uint32_t lanePaths = 0;
    for (const Path& p : G.paths) lanePaths += p.kind == PathKind::Lane ? 1 : 0;
    std::printf("    [lane graph] %s: %u lane paths, %zu sections, %zu junctions, %zu problems, %.0f ms; %u connector lanes left "
                "out; joined as stated: %u carrying on, %u leaving a host, %u merging into one, %u with a gap\n",
                name.c_str(), lanePaths, G.sections.size(), G.junctions.size(), lg.problems.size(), lg.milliseconds,
                lg.connectorsLeftOut, lg.carryOnJoins, lg.divergeJoins, lg.mergeJoins, lg.joinsWithGap);
    for (std::size_t i = 0; i < lg.problems.size() && i < 4; ++i) std::printf("      problem: %s\n", problemLine(lg.problems[i]).c_str());

    // 1. sound
    const std::vector<std::string> faults = validate(G);
    for (std::size_t i = 0; i < faults.size() && i < 8; ++i) std::printf("      fault: %s\n", faults[i].c_str());
    CHECK(faults.empty());
    CHECK(lg.sources.size() == G.paths.size() && lg.sectionSources.size() == G.sections.size());
    if (lg.sources.size() != G.paths.size() || lg.sectionSources.size() != G.sections.size()) return;

    // 2. every lane path a stretch of one builder lane on one of its streets (the model's ids: its street and its lane),
    // the stretches of one lane never overlapping and within its length as the graph reads it (a fold tidied out of it)
    const LaneSet& L = r.lanes;
    std::vector<std::vector<uint32_t>> ofLane(L.lanes.size());
    const RoadModel& m = modelOf(name);   // (the streets a lane runs along are the model's lane's: the one place they are held)
    CHECK(m.lanes.size() == L.lanes.size());
    if (m.lanes.size() != L.lanes.size()) return;
    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        if (G.paths[p].kind != PathKind::Lane) continue;
        CHECK(G.paths[p].modelLane == static_cast<uint32_t>(lg.sources[p].lane) && G.paths[p].street != kNoIndex);
        bool onIt = false;
        for (const ModelLaneSpan& sp : m.lanes[static_cast<std::size_t>(lg.sources[p].lane)].streets)
            onIt = onIt || sp.street.index == G.paths[p].street;
        CHECK(onIt);
        ofLane[static_cast<std::size_t>(lg.sources[p].lane)].push_back(p);
    }
    // EVERY LANE CARS DRIVE COVERED, END TO END, by its lane paths and its gaps, each CHECKED AGAINST THE BUILT LANE and
    // the stated junctions, not the lane level's own numbers:
    //   - a lane left out entirely is a connector, a track or a footway, or a lane the scene names (known.drivenLeftOut);
    //   - its length as the graph reads it is the built lane's own, but for the lanes the scene names as tidied (a fold
    //     cut out: known.tidied, each with the most it may cut);
    //   - each lane path is a slice of the built lane: its ends where the built lane is at its stretch's ends, its length
    //     the stretch's (kSliceTolerance; on a tidied lane, within the amount cut);
    //   - a gap "inside a junction" is that junction's stated stretch on the lane's road (from the street's end to the
    //     arm's stop line as the paint or ADR-0419's rule puts it, or a stretch the file states inside it), or a whole lane
    //     of a road the scene names as stated inside a spread junction; any other gap is one the scene names.
    std::vector<std::vector<const LaneGap*>> gapsOf(L.lanes.size());
    for (const LaneGap& gp : lg.laneGaps) {
        CHECK(gp.lane < L.lanes.size());
        if (gp.lane < L.lanes.size()) gapsOf[gp.lane].push_back(&gp);
    }
    std::map<int, std::vector<std::tuple<double, double, uint32_t>>> extents = junctionExtents(r, m);
    // the built lane the way cars drive it, and its stations: on the line it is driven along (ADR-0450: a lane the kerb
    // parking band overlaps is driven along the middle of the rest of it)
    std::vector<std::vector<Vec2>> builtXy(L.lanes.size());
    std::vector<std::vector<double>> builtS(L.lanes.size());
    for (std::size_t li = 0; li < L.lanes.size(); ++li) {
        builtXy[li] = li < m.lanes.size() && m.lanes[li].curve.count == L.lanes[li].xy().size()
                          ? drivenLanePoints(r.graph, m, static_cast<uint32_t>(li))
                          : L.lanes[li].xy().copied();
        if (L.lanes[li].dir < 0) std::reverse(builtXy[li].begin(), builtXy[li].end());
        builtS[li] = stations(builtXy[li]);
    }
    // ...and each of its points' station along its road's spine, as the builder laid it (the model's record of the build:
    // RoadModel::spineStations, a lane's points laid one beside each spine sample), so a point `along` metres reads as a
    // station without projecting (projecting a point 9 m off a bending spine moves it by decimetres)
    std::vector<std::vector<double>> builtStation(L.lanes.size());
    for (std::size_t li = 0; li < L.lanes.size() && li < m.lanes.size(); ++li) {
        const ModelLane& ml = m.lanes[li];
        if (ml.curve.count != L.lanes[li].xy().size()) continue;
        for (uint32_t k = 0; k < ml.curve.count; ++k) builtStation[li].push_back(m.spineStations[ml.curve.first + k]);
        if (L.lanes[li].dir < 0) std::reverse(builtStation[li].begin(), builtStation[li].end());
    }
    auto stationAlong = [&](std::size_t li, double along) {
        const std::vector<double>& s = builtS[li];
        const std::vector<double>& st = builtStation[li];
        if (st.size() != s.size() || s.empty()) return kNotStated;
        const std::size_t k = static_cast<std::size_t>(std::upper_bound(s.begin(), s.end(), along) - s.begin());
        if (k == 0) return st.front();
        if (k >= s.size()) return st.back();
        const double t = s[k] > s[k - 1] ? (along - s[k - 1]) / (s[k] - s[k - 1]) : 0.0;
        return st[k - 1] + (st[k] - st[k - 1]) * t;
    };
    auto builtPoint = [&](std::size_t li, double along) { return pointAt(builtXy[li], builtS[li], along); };
    int badSlices = 0;
    for (std::size_t li = 0; li < L.lanes.size(); ++li) {
        if (lg.leftOut[li] || builtS[li].empty()) continue;
        const double builtLength = builtS[li].back();
        const std::map<std::string, double>::const_iterator t = known.tidied.find(L.lanes[li].id);
        const double cut = t == known.tidied.end() ? 0.0 : t->second;
        const bool lengthOk = lg.laneLengths[li] <= builtLength + kSliceTolerance && lg.laneLengths[li] >= builtLength - cut - kSliceTolerance &&
                              (cut > 0 || std::fabs(lg.laneLengths[li] - builtLength) <= kSliceTolerance);
        if (!lengthOk && badSlices++ < 6)
            std::printf("      lane %s: the graph reads it %.3f m long, the built lane is %.3f m (may cut %.2f)\n", L.lanes[li].id.c_str(),
                        lg.laneLengths[li], builtLength, cut);
        CHECK(lengthOk);
        if (t != known.tidied.end() && !(lg.laneLengths[li] < builtLength - kSliceTolerance))
            std::printf("      lane %s is named as tidied, and is not: take it off the list\n", L.lanes[li].id.c_str());
        if (t != known.tidied.end()) CHECK(lg.laneLengths[li] < builtLength - kSliceTolerance);   // (a named tidy that no longer happens: off the list)
        for (uint32_t p : ofLane[li]) {
            const double from = lg.sources[p].from, to = lg.sources[p].to;
            const double off0 = distance(G.pointAt(PathId(p), 0), builtPoint(li, from)), off1 = distance(G.pointAt(PathId(p), G.paths[p].length), builtPoint(li, to));
            const double offLength = std::fabs(G.paths[p].length - (to - from));
            const bool ok = cut > 0 ? distToPolyline(G.pointAt(PathId(p), 0), builtXy[li], false) <= cut + kSliceTolerance &&
                                          distToPolyline(G.pointAt(PathId(p), G.paths[p].length), builtXy[li], false) <= cut + kSliceTolerance &&
                                          offLength <= cut + kSliceTolerance
                                    : off0 <= kSliceTolerance && off1 <= kSliceTolerance && offLength <= kSliceTolerance;
            if (!ok && badSlices++ < 6)
                std::printf("      lane path %u of %s (%.3f to %.3f m): its ends %.3f / %.3f m off the built lane there, its length %.3f m off\n", p,
                            L.lanes[li].id.c_str(), from, to, off0, off1, offLength);
            CHECK(ok);
        }
    }
    int badCover = 0, badJunctionGaps = 0;
    std::set<std::string> badJunctionGapLanes;
    std::set<std::string> leftOutDriven;
    std::map<std::string, std::string> otherGaps;
    for (std::size_t li = 0; li < L.lanes.size(); ++li) {
        const Lane& l = L.lanes[li];
        const bool trackOrFootway = l.road() >= 0 && (r.graph.cls(r.graph.edges[static_cast<std::size_t>(l.road())]).track ||
                                                      r.graph.cls(r.graph.edges[static_cast<std::size_t>(l.road())]).footway);
        if (lg.leftOut[li]) {
            CHECK(ofLane[li].empty() && gapsOf[li].empty());
            if (!l.isConnector() && !trackOrFootway) leftOutDriven.insert(l.id);
            continue;
        }
        CHECK(!l.isConnector() && !trackOrFootway);
        const std::string road = l.road() >= 0 ? r.graph.edges[static_cast<std::size_t>(l.road())].id : std::string();
        for (const LaneGap* gp : gapsOf[li]) {
            if (gp->junction == kNoIndex) {
                otherGaps[l.id] = gp->why;
                continue;
            }
            const bool whole = gp->from <= kLaneCoverTolerance && gp->to >= lg.laneLengths[li] - kLaneCoverTolerance;
            if (whole && ofLane[li].empty()) {
                if (!known.insideRoads.count(road) && badJunctionGaps++ < 4)
                    std::printf("      lane %s: wholly \"inside junction %s\", but its road is not one the scene states inside one\n",
                                l.id.c_str(), r.graph.junctions[gp->junction].id.c_str());
                CHECK(known.insideRoads.count(road) == 1u);
                continue;
            }
            // the junction's STATED extent, worked out here (not the lane level's outline, which is drawn through its own
            // lane ends): the gap, read on the BUILT lane and projected onto its road's spine, runs from the street's end
            // to the arm's stop line exactly (an arm of that junction on this road: junctionExtents), or lies on a stretch
            // the road file states inside the junction. A lane whose fold was cut out (known.tidied) may be off by up to
            // the amount cut.
            const double s0 = stationAlong(li, gp->from), s1 = stationAlong(li, gp->to);
            const double lo = std::min(s0, s1), hi = std::max(s0, s1);
            const double tol = kJunctionStretchTolerance + (known.tidied.count(l.id) ? known.tidied.at(l.id) : 0.0);
            bool within = false;
            for (const std::tuple<double, double, uint32_t>& x : extents[l.road()]) {
                if (std::get<2>(x) != gp->junction) continue;
                const bool armStretch = std::fabs(lo - std::get<0>(x)) <= tol && std::fabs(hi - std::get<1>(x)) <= tol;
                const bool onIt = lo >= std::get<0>(x) - tol && hi <= std::get<1>(x) + tol;
                bool inside = false;   // (an inside stretch may hold a shorter gap; an arm's stretch must be matched end to end)
                for (const ModelStretch& st : r.graph.junctions[gp->junction].inside)
                    inside = inside || (st.road == road && std::fabs(st.from - std::get<0>(x)) < 1e-9 && std::fabs(st.to - std::get<1>(x)) < 1e-9);
                within = within || armStretch || (inside && onIt);
            }
            if (!within) badJunctionGapLanes.insert(l.id + " @ " + r.graph.junctions[gp->junction].id);
            if (!within && known.junctionGapsOff.count(l.id + " @ " + r.graph.junctions[gp->junction].id)) continue;
            if (!within && badJunctionGaps++ < 8) {
                std::printf("      lane %s: %.2f to %.2f m along it (stations %.2f to %.2f of %s), said to be inside junction %s, is not "
                            "that junction's stated stretch:", l.id.c_str(), gp->from, gp->to, lo, hi, road.c_str(),
                            r.graph.junctions[gp->junction].id.c_str());
                for (const std::tuple<double, double, uint32_t>& x : extents[l.road()])
                    if (std::get<2>(x) == gp->junction) std::printf(" %.2f to %.2f;", std::get<0>(x), std::get<1>(x));
                std::printf("\n");
            }
            CHECK(within);
        }
        std::vector<std::pair<double, double>> cover;
        for (uint32_t p : ofLane[li]) cover.emplace_back(lg.sources[p].from, lg.sources[p].to);
        for (const LaneGap* gp : gapsOf[li]) cover.emplace_back(gp->from, gp->to);
        std::sort(cover.begin(), cover.end());
        double reached = 0;
        bool hole = false, overlap = false;
        for (const std::pair<double, double>& c : cover) {
            hole = hole || c.first > reached + kLaneCoverTolerance;
            overlap = overlap || c.first < reached - kLaneCoverTolerance;
            reached = c.second;
        }
        hole = hole || std::fabs(reached - lg.laneLengths[li]) > kLaneCoverTolerance;
        if ((hole || overlap) && badCover++ < 4)
            std::printf("      lane %s: its paths and gaps %s its %.3f m (reaching %.3f m)\n", L.lanes[li].id.c_str(),
                        overlap ? "overlap along" : "do not cover", lg.laneLengths[li], reached);
        CHECK(builtS[li].empty() || lg.laneLengths[li] <= builtS[li].back() + kLaneCoverTolerance);   // (the line driven)
    }
    CHECK(badCover == 0);
    CHECK(badJunctionGaps == 0);
    for (const std::string& k : known.junctionGapsOff)
        if (!badJunctionGapLanes.count(k)) std::printf("      known junction gap %s is now its junction's stated stretch: take it off the list\n", k.c_str());
    CHECK(badJunctionGapLanes == known.junctionGapsOff);
    for (const std::string& id : leftOutDriven)
        if (!known.drivenLeftOut.count(id)) std::printf("      lane %s: made into no lane path (not known)\n", id.c_str());
    CHECK(leftOutDriven == known.drivenLeftOut);
    std::set<std::string> otherGapLanes, knownGapLanes;
    for (const std::pair<const std::string, std::string>& g : otherGaps) {
        otherGapLanes.insert(g.first);
        const std::map<std::string, std::string>::const_iterator k = known.otherGaps.find(g.first);
        if (k == known.otherGaps.end()) std::printf("      lane %s: a stretch with no lane path outside every junction (not known): %s\n", g.first.c_str(), g.second.c_str());
        else CHECK(g.second.find(k->second) != std::string::npos);
    }
    for (const std::pair<const std::string, std::string>& k : known.otherGaps) knownGapLanes.insert(k.first);
    CHECK(otherGapLanes == knownGapLanes);
    // each lane path in exactly one section, and no connector in any
    std::vector<int> inSections(G.paths.size(), 0);
    for (const PathId& p : G.sectionLanes) ++inSections[p.index];
    for (uint32_t p = 0; p < G.paths.size(); ++p) CHECK(inSections[p] == (G.paths[p].kind == PathKind::Lane ? 1 : 0));
    // a section's lanes start together and end together (on one line across the road, through its spine at the section's
    // start and end); neighbours side by side; a short section reported
    int badShare = 0, badNeighbours = 0;
    for (uint32_t s = 0; s < G.sections.size(); ++s) {
        const Section& sec = G.sections[s];
        const SectionSource& ss = lg.sectionSources[s];
        const EdgeSpec& e = r.graph.edges[static_cast<std::size_t>(ss.edge)];
        std::vector<Vec2> starts, ends;
        for (uint32_t k = sec.lanes.first; k < sec.lanes.end(); ++k) {
            starts.push_back(G.pointAt(G.sectionLanes[k], 0));
            ends.push_back(G.pointAt(G.sectionLanes[k], G.paths[G.sectionLanes[k].index].length));
        }
        auto spineAt = [&](double u) { return pointAt(e.xy, e.s, ss.dir > 0 ? u : e.s.back() - u); };
        const double offStart = offCrossLine(spineAt(ss.from), starts), offEnd = offCrossLine(spineAt(ss.to), ends);
        // (a lane whose fold was tidied out of it, on the inside of a sharp corner, may have its cut inside the fold: its
        // point there is where the fold was cut, not on the line across)
        bool tidied = false;
        for (uint32_t k = sec.lanes.first; k < sec.lanes.end(); ++k) {
            const std::size_t li = static_cast<std::size_t>(lg.sources[G.sectionLanes[k].index].lane);
            tidied = tidied || lg.laneLengths[li] < L.lanes[li].s().back() - kLaneCoverTolerance;
        }
        if ((offStart > kSectionShareTolerance || offEnd > kSectionShareTolerance) && tidied)
            std::printf("      section %u on %s: its lanes' starts stray %.3f m and ends %.3f m from a line across the road, a lane "
                        "of it tidied (a fold cut out)\n", s, e.id.c_str(), offStart, offEnd);
        else if (offStart > kSectionShareTolerance || offEnd > kSectionShareTolerance)
            if (badShare++ < 4) std::printf("      section %u on %s: its lanes' starts stray %.3f m and ends %.3f m from a line across the road\n", s,
                                            e.id.c_str(), offStart, offEnd);
        for (uint32_t k = sec.lanes.first; k < sec.lanes.end(); ++k) {
            const PathId p = G.sectionLanes[k];
            if (k + 1 < sec.lanes.end()) {
                const PathId q = G.sectionLanes[k + 1];
                const double gap = distance(G.pointAt(p, G.paths[p.index].length / 2), G.pointAt(q, G.paths[q.index].length / 2));
                if (gap > (G.paths[p.index].width + G.paths[q.index].width) / 2 + kNeighbourSlack)
                    if (badNeighbours++ < 4) std::printf("      section %u: lanes %u and %u stand %.2f m apart\n", s, p.index, q.index, gap);
            }
        }
    }
    CHECK(badShare == 0);
    CHECK(badNeighbours == 0);
    // where one stretch of a builder lane follows the next, they join within 1 cm and 1 degree -- or they turn no more
    // than the builder's lane itself turns there (a cut on a bend tighter than about 29 m reads the two sides'
    // directions half a metre apart, kTangentHalfSpan, and a lane kinked at a sample turns at it whether cut there or
    // not): the cut adds no kink. Where two DIFFERENT lanes are joined as the model states (a street carrying on, a lane
    // leaving or merging into its host), they meet within the same 1 cm and 1 degree, but for the scene's known joins,
    // named by their builder lanes ("from>into", known.joins), each held to its written gap and turn. (Those over 1 mm
    // apart are also each a problem line with the gap, joinsWithGap; one turning under kKinkDegrees is in no problem line.)
    // (Lanes and connectors: checkConnectors.)
    int badJoins = 0, onBends = 0;
    uint32_t gapsBetweenLanes = 0;
    std::set<std::string> joinsFound, joinsKnown;
    for (const std::pair<const std::string, std::pair<double, double>>& k : known.joins) joinsKnown.insert(k.first);
    const double joinCos = std::cos(kTestJoinDegrees * 3.14159265358979 / 180.0);
    for (uint32_t p = 0; p < G.paths.size(); ++p)
        for (uint32_t k = G.paths[p].next.first; k < G.paths[p].next.end(); ++k) {
            const PathId q = G.nextPaths[k];
            if (G.paths[p].kind != PathKind::Lane || G.paths[q.index].kind != PathKind::Lane) continue;
            if (lg.sources[p].lane != lg.sources[q.index].lane) {
                const double gap = distance(G.pointAt(PathId(p), G.paths[p].length), G.pointAt(q, 0));
                const double cosine = dot(G.directionAt(PathId(p), G.paths[p].length), G.directionAt(q, 0));
                gapsBetweenLanes += gap > kTestJoinReported ? 1 : 0;
                if (gap > kTestJoinDistance || cosine < joinCos) {
                    const std::string key = L.lanes[static_cast<std::size_t>(lg.sources[p].lane)].id + ">" +
                                            L.lanes[static_cast<std::size_t>(lg.sources[q.index].lane)].id;
                    joinsFound.insert(key);
                    const double degrees = std::acos(std::clamp(cosine, -1.0, 1.0)) * 180.0 / 3.14159265358979;
                    const std::map<std::string, std::pair<double, double>>::const_iterator k = known.joins.find(key);
                    const bool asKnown = k != known.joins.end() && std::fabs(gap - k->second.first) <= kKnownJoinGap &&
                                         std::fabs(degrees - k->second.second) <= kKnownJoinDegrees;
                    if (!asKnown)
                        std::printf("      lane %s: %.4f m apart, %.2f degrees (%s)\n", key.c_str(), gap, degrees,
                                    k == known.joins.end() ? "not known" : "known at another gap or turn");
                    CHECK(asKnown || k == known.joins.end());
                }
                continue;
            }
            const double gap = distance(G.pointAt(PathId(p), G.paths[p].length), G.pointAt(q, 0));
            const double cosine = dot(G.directionAt(PathId(p), G.paths[p].length), G.directionAt(q, 0));
            double allowed = joinCos;
            if (cosine < joinCos) {
                const Lane& l = L.lanes[static_cast<std::size_t>(lg.sources[p].lane)];
                std::vector<Vec2> xy = l.xy().copied();
                if (l.dir < 0) std::reverse(xy.begin(), xy.end());
                const std::vector<double> s = stations(xy);
                const double t = lg.sources[p].to;
                const Vec2 before = normalize(pointAt(xy, s, t) - pointAt(xy, s, t - kTangentHalfSpan));
                const Vec2 after = normalize(pointAt(xy, s, t + kTangentHalfSpan) - pointAt(xy, s, t));
                allowed = std::min(joinCos, dot(before, after) - kCosineRounding);
                ++onBends;
            }
            if (gap > kTestJoinDistance || cosine < allowed)
                if (badJoins++ < 4) std::printf("      lane %u -> lane %u: %.4f m apart, cos %.6f (allowed %.6f)\n", p, q.index, gap, cosine, allowed);
        }
    if (onBends) std::printf("      (%d cuts on bends tighter than a degree per half metre: held to the builder lane's own turn)\n", onBends);
    CHECK(badJoins == 0);
    CHECK(gapsBetweenLanes == lg.joinsWithGap);
    for (const std::string& k : joinsKnown)
        if (!joinsFound.count(k)) std::printf("      known join %s now meets within 1 cm and 1 degree: take it off the list\n", k.c_str());
    CHECK(joinsFound == joinsKnown);
    // EVERY JOIN THE MODEL STATES IS MADE, as many as written from the scene (a stated carry-on, diverge or merge not made fails)
    if (lg.carryOnJoins != known.carryOnJoins || lg.divergeJoins != known.divergeJoins || lg.mergeJoins != known.mergeJoins)
        std::printf("      joins made: %u carrying on, %u leaving a host, %u merging (known: %u, %u, %u)\n", lg.carryOnJoins, lg.divergeJoins,
                    lg.mergeJoins, known.carryOnJoins, known.divergeJoins, known.mergeJoins);
    CHECK(lg.carryOnJoins == known.carryOnJoins && lg.divergeJoins == known.divergeJoins && lg.mergeJoins == known.mergeJoins);
    // EACH STATED MERGE NAMED ON ITS JOIN, and no other crossing point naming one; the model says which of the two
    // paths joins, and so gives way (RoadModel::mergingAt, ADR-0436): the set of "joining lane>host lane" over the
    // records that name a merge is the scene's, written here
    {
        std::set<std::string> marked;
        for (const CrossingPoint& x : G.crossings) {
            if (x.modelConnection == kNoIndex) continue;
            CHECK(x.meeting == Meeting::Join && x.otherSide == OtherSide::Path);
            const PathId merging = m.mergingAt(x);
            CHECK(merging.valid() && (merging == x.a || merging == x.otherPath()));
            if (!merging.valid()) continue;
            const PathId other = merging == x.a ? x.otherPath() : x.a;
            if (!other.valid() || other.index >= G.paths.size()) continue;
            const std::string key = L.lanes[G.paths[merging.index].modelLane].id + ">" + L.lanes[G.paths[other.index].modelLane].id;
            if (marked.insert(key).second && !known.merges.count(key)) std::printf("      stated merge %s marked (not known)\n", key.c_str());
        }
        for (const std::string& k : known.merges)
            if (!marked.count(k)) std::printf("      stated merge %s not marked on any join\n", k.c_str());
        CHECK(marked == known.merges);
    }
    // ITS PROBLEM LINES BY KIND AND COUNT, as written for the scene
    if (known.pinProblems) {
        std::map<std::string, uint32_t> byKind;
        for (const std::string& p : problemLines(lg.problems)) {
            ++byKind[problemKind(p)];
            if (problemKind(p) == "other") std::printf("      problem: %s\n", p.c_str());
        }
        if (byKind != known.problems)
            for (const std::pair<const std::string, uint32_t>& k : byKind) std::printf("      %u problems: %s\n", k.second, k.first.c_str());
        CHECK(byKind == known.problems);
    }

    // 3. junctions: one per stated junction its lanes reach, each naming it; its arriving and leaving lanes' ends on its
    // outline; NO LANE PATH INSIDE ONE (a point of a lane path inside a junction's outline, farther than kJunctionEndReach
    // from it, at the level of the junction's lane ends -- within the builder's same-level figure). known.inside: the
    // model junctions (by id) a scene is known to have lane paths inside, each with exactly how many.
    std::map<std::string, uint32_t> insideFound;
    const std::map<std::string, uint32_t>& insideKnown = known.inside;
    for (uint32_t j = 0; j < G.junctions.size(); ++j) {
        const Junction& jn = G.junctions[j];
        CHECK(jn.modelJunction < r.graph.junctions.size() && (j == 0 || G.junctions[j - 1].modelJunction < jn.modelJunction));
        const std::vector<Vec2> outline(G.planPoints.begin() + jn.box.first, G.planPoints.begin() + jn.box.end());
        std::vector<double> levels;
        for (uint32_t k = jn.arriving.first; k < jn.arriving.end(); ++k) {
            const PathId p = G.junctionLanes[k];
            CHECK(distToPolyline(G.pointAt(p, G.paths[p.index].length), outline, true) <= kJunctionEndReach);
            levels.push_back(G.heightAt(p, G.paths[p.index].length));
        }
        for (uint32_t k = jn.leaving.first; k < jn.leaving.end(); ++k) {
            const PathId p = G.junctionLanes[k];
            CHECK(distToPolyline(G.pointAt(p, 0), outline, true) <= kJunctionEndReach);
            levels.push_back(G.heightAt(p, 0));
        }
        if (outline.size() < 3 || levels.empty()) continue;
        Box2 box{outline[0].x, outline[0].y, outline[0].x, outline[0].y};
        for (const Vec2& q : outline) {
            box.minX = std::min(box.minX, q.x), box.minY = std::min(box.minY, q.y);
            box.maxX = std::max(box.maxX, q.x), box.maxY = std::max(box.maxY, q.y);
        }
        for (uint32_t p = 0; p < G.paths.size(); ++p) {
            if (G.paths[p].kind != PathKind::Lane) continue;
            for (uint32_t k = G.paths[p].curve.first; k < G.paths[p].curve.end(); ++k) {
                const Vec2 q = G.points[k];
                if (q.x < box.minX || q.x > box.maxX || q.y < box.minY || q.y > box.maxY) continue;
                if (!pointInPolygon(outline, q) || distToPolyline(q, outline, true) <= kJunctionEndReach) continue;
                double nearest = 1e300;
                for (double z : levels) nearest = std::min(nearest, std::fabs(G.heights[k] - z));
                if (nearest > r.graph.rules.sameLevelDz) continue;
                const std::string jid = r.graph.junctions[jn.modelJunction].id;
                if (insideFound[jid]++ < 2 && !insideKnown.count(jid))
                    std::printf("      lane path %u passes inside junction %s at (%.1f, %.1f)\n", p, jid.c_str(), q.x, q.y);
                break;
            }
        }
    }
    for (const std::pair<const std::string, uint32_t>& f : insideFound)
        if (!insideKnown.count(f.first) || insideKnown.at(f.first) != f.second)
            std::printf("      junction %s: %u lane paths inside it (known: %u)\n", f.first.c_str(), f.second,
                        insideKnown.count(f.first) ? insideKnown.at(f.first) : 0u);
    CHECK(insideFound == insideKnown);

    // 4. NO TWO ROADS' LANES CROSS AT ONE LEVEL OUTSIDE EVERY JUNCTION (cars would pass through each other mid-block):
    // measured here, not read from the crossing points (which pair only lanes sharing a path before or after): every two
    // lane paths of different roads (the built lanes' roads) whose centre lines cross, within the road file's same-level
    // figure of each other there, at a point on neither road's stated junction stretch (junctionExtents: the street's end to
    // the arm's stop line, or a stretch stated inside a junction) -- but for two lanes the model states join (one's path
    // leads into the other's: a street carrying on, a ramp leaving or joining its host) within kNearTheirJoin of where they
    // join, the join itself held above. known.crossings names the scene's, "a x b @ (x, y)" (builder lane ids, in id order;
    // the place to the metre). (Two centre lines lying along each other, collinear, are not found here: an open gap.)
    {
        std::map<std::pair<int, int>, std::vector<Vec2>> joinsOf;   // two builder lanes -> where one's path leads into the other's
        for (uint32_t p = 0; p < G.paths.size(); ++p)
            for (uint32_t k = G.paths[p].next.first; k < G.paths[p].next.end(); ++k) {
                const int a = lg.sources[p].lane, b = lg.sources[G.nextPaths[k].index].lane;
                if (a >= 0 && b >= 0 && a != b) joinsOf[std::make_pair(std::min(a, b), std::max(a, b))].push_back(G.pointAt(G.nextPaths[k], 0));
            }
        auto onJunctionStretch = [&](int edge, const Vec2& q) {
            const EdgeSpec& e = r.graph.edges[static_cast<std::size_t>(edge)];
            const double s = project(e.xy, e.s, q).station;
            for (const std::tuple<double, double, uint32_t>& x : extents[edge])
                if (s >= std::get<0>(x) - kJunctionStretchTolerance && s <= std::get<1>(x) + kJunctionStretchTolerance) return true;
            return false;
        };
        struct Seg {
            uint32_t path;
            Vec2 a, b;
            double za, zb;
        };
        constexpr double kCell = 20.0;
        std::map<std::pair<long, long>, std::vector<Seg>> grid;
        auto roadOfPath = [&](uint32_t p) { return L.lanes[static_cast<std::size_t>(lg.sources[p].lane)].road(); };
        for (uint32_t p = 0; p < G.paths.size(); ++p) {
            if (G.paths[p].kind != PathKind::Lane) continue;
            for (uint32_t k = G.paths[p].curve.first; k + 1 < G.paths[p].curve.end(); ++k) {
                const Seg s{p, G.points[k], G.points[k + 1], G.heights[k], G.heights[k + 1]};
                const long x0 = static_cast<long>(std::floor(std::min(s.a.x, s.b.x) / kCell)), x1 = static_cast<long>(std::floor(std::max(s.a.x, s.b.x) / kCell));
                const long y0 = static_cast<long>(std::floor(std::min(s.a.y, s.b.y) / kCell)), y1 = static_cast<long>(std::floor(std::max(s.a.y, s.b.y) / kCell));
                for (long x = x0; x <= x1; ++x)
                    for (long y = y0; y <= y1; ++y) grid[std::make_pair(x, y)].push_back(s);
            }
        }
        std::set<std::string> crossingsFound;
        for (const std::pair<const std::pair<long, long>, std::vector<Seg>>& cell : grid)
            for (std::size_t i = 0; i < cell.second.size(); ++i)
                for (std::size_t j = i + 1; j < cell.second.size(); ++j) {
                    const Seg& s = cell.second[i];
                    const Seg& t = cell.second[j];
                    if (roadOfPath(s.path) == roadOfPath(t.path)) continue;
                    const int la = lg.sources[s.path].lane, lb = lg.sources[t.path].lane;
                    Vec2 hit;
                    if (!segmentsCross(s.a, s.b, t.a, t.b, hit)) continue;
                    const double fs = distance(s.a, s.b) > 0 ? distance(s.a, hit) / distance(s.a, s.b) : 0;
                    const double ft = distance(t.a, t.b) > 0 ? distance(t.a, hit) / distance(t.a, t.b) : 0;
                    if (std::fabs((s.za + (s.zb - s.za) * fs) - (t.za + (t.zb - t.za) * ft)) > r.graph.rules.sameLevelDz) continue;
                    if (onJunctionStretch(roadOfPath(s.path), hit) || onJunctionStretch(roadOfPath(t.path), hit)) continue;
                    bool nearTheirJoin = false;
                    const std::map<std::pair<int, int>, std::vector<Vec2>>::const_iterator jn = joinsOf.find(std::make_pair(std::min(la, lb), std::max(la, lb)));
                    if (jn != joinsOf.end())
                        for (const Vec2& at : jn->second) nearTheirJoin = nearTheirJoin || distance(at, hit) <= kNearTheirJoin;
                    if (nearTheirJoin) continue;
                    std::string a = L.lanes[static_cast<std::size_t>(la)].id, b = L.lanes[static_cast<std::size_t>(lb)].id;
                    if (b < a) std::swap(a, b);
                    char place[64];
                    std::snprintf(place, sizeof place, " @ (%.0f, %.0f)", hit.x, hit.y);
                    const std::string key = a + " x " + b + place;
                    if (crossingsFound.insert(key).second && !known.crossings.count(key))
                        std::printf("      lanes %s cross at one level outside every junction (not known)\n", key.c_str());
                }
        for (const std::string& k : known.crossings)
            if (!crossingsFound.count(k)) std::printf("      lanes %s known to cross: not found\n", k.c_str());
        CHECK(crossingsFound == known.crossings);
    }
}

// The direction a path's own centre line runs at its start (or end): its first (last) piece.
Vec2 endPieceDirection(const LaneGraph& G, PathId p, bool atStart) {
    const Range c = G.paths[p.index].curve;
    return atStart ? normalize(G.points[c.first + 1] - G.points[c.first]) : normalize(G.points[c.end() - 1] - G.points[c.end() - 2]);
}

// THE CONNECTORS' CHECKS (#238's "done when"): every stated turn made or refused, once; the ends meet their lanes; no
// connector tighter than a vehicle allowed on it, nor taking one off the paved road; connectors sharing a lane share
// that end bit for bit.
void checkConnectors(const std::string& name) {
    const Result& r = laneSceneOf(name);
    const LaneGraphBuild& lg = laneGraphOf(name);
    const LaneGraph& G = modelOf(name).laneLevel;
    uint32_t connectors = 0;
    for (const Path& p : G.paths) connectors += p.kind == PathKind::Connector ? 1 : 0;
    std::printf("    [connectors] %s: %u connectors (%u straight on, %u near, %u far), %zu turns refused, largest part a car can "
                "get round %.1f%% of the lanes, connectors %.0f ms\n", name.c_str(), connectors,
                lg.connectorsByTurn[static_cast<int>(Turn::Straight)], lg.connectorsByTurn[static_cast<int>(Turn::NearTurn)],
                lg.connectorsByTurn[static_cast<int>(Turn::FarTurn)], lg.refusals.size(), 100 * lg.reachableShare,
                lg.connectMilliseconds);
    CHECK(validate(G).empty());
    CHECK(lg.junctionNotes.size() == G.junctions.size());
    if (lg.junctionNotes.size() != G.junctions.size()) return;

    // every stated turn: one connector, or one refusal (a line in the problems): its corner fits no vehicle (a corner fault
    // for every class), or there is no lane path to take it
    std::vector<uint32_t> refusedAt(G.junctions.size(), 0);
    for (const Refusal& ref : lg.refusals) ++refusedAt[ref.junction];
    // the turns the ROAD MODEL states at each junction (not the lane level's count of them)
    std::map<uint32_t, uint32_t> modelTurnsAt;
    for (const ModelTurn& t : modelOf(name).turns) ++modelTurnsAt[t.junction.index];
    int badCounts = 0;
    for (uint32_t j = 0; j < G.junctions.size(); ++j) {
        const JunctionNote& n = lg.junctionNotes[j];
        const Junction& jn = G.junctions[j];
        bool counted = n.turnsStated == (modelTurnsAt.count(jn.modelJunction) ? modelTurnsAt.at(jn.modelJunction) : 0u) &&
                       n.turnsStated == n.turnsBuilt + n.turnsRefused && jn.connectors.count == n.turnsBuilt;
        counted = counted && refusedAt[j] == n.turnsRefused;
        // one connector per (from, to)
        std::set<std::pair<uint32_t, uint32_t>> pairs;
        for (uint32_t k = jn.connectors.first; k < jn.connectors.end(); ++k) {
            const Path& c = G.paths[G.junctionConnectors[k].index];
            counted = counted && pairs.insert(std::make_pair(c.from.index, G.nextPaths[c.next.first].index)).second;
        }
        if (!counted && badCounts++ < 4)
            std::printf("      junction %u: %u turns stated, %u built, %u refused (%u refusals listed), %u connectors\n", j,
                        n.turnsStated, n.turnsBuilt, n.turnsRefused, refusedAt[j], jn.connectors.count);
    }
    CHECK(badCounts == 0);
    std::size_t problemFitsNone = 0, problemNoPath = 0, refusedFitsNone = 0, refusedNoPath = 0;
    // (by the problems' stated kinds, road_problem.h, not by their words)
    for (const RoadProblem& p : lg.problems) {
        problemFitsNone += p.kind == RoadProblemKind::TurnFitsNoVehicle ? 1 : 0;
        problemNoPath += p.kind == RoadProblemKind::TurnNoLanePath ? 1 : 0;
    }
    for (const Refusal& ref : lg.refusals) {
        ++(ref.why == RefusalWhy::NoLanePath ? refusedNoPath : refusedFitsNone);
        CHECK(ref.modelTurn != kNoIndex);
        CHECK(ref.why == RefusalWhy::NoLanePath ? !(ref.from.valid() && ref.to.valid()) : ref.from.valid() && ref.to.valid());
    }
    CHECK(problemFitsNone == refusedFitsNone && problemNoPath == refusedNoPath);

    // the ends: in position within 1 cm and direction within 1 degree of their lanes' (the lane's direction as the graph
    // reads it; the connector's, its own end piece); the tightest bend no tighter than any vehicle allowed on it; swept at
    // the widest of them, on the paved road
    const LaneLookup look(r.pavement);
    const double joinCos = std::cos(kTestJoinDegrees * 3.14159265358979 / 180.0);
    auto paved = [&](const Vec2& p, double z) {
        for (int li : look.grid.at(p)) {
            const Box2& b = look.box[static_cast<std::size_t>(li)];
            if (p.x < b.minX || p.x > b.maxX || p.y < b.minY || p.y > b.maxY || !look.footprint[static_cast<std::size_t>(li)].contains(p)) continue;
            if (std::fabs(r.heights->deck(li, p) - z) <= r.graph.rules.sameLevelDz) return true;
        }
        return false;
    };
    const double radiusOf[2] = {kTestCarRadius, kTestLongRadius}, widthOf[2] = {kTestCarWidth, kTestLongWidth};
    const NameIndex tagOf[2] = {LaneGraph::nameIndex(G.vehicleTagNames, kLaneGraphTestTag), LaneGraph::nameIndex(G.vehicleTagNames, kLaneGraphLongTag)};
    int badEnds = 0, badRadius = 0, offRoad = 0;
    for (uint32_t c = 0; c < G.paths.size(); ++c) {
        const Path& p = G.paths[c];
        if (p.kind != PathKind::Connector) continue;
        const PathId from = p.from, into = G.nextPaths[p.next.first];
        const double gapIn = distance(G.pointAt(from, G.paths[from.index].length), G.pointAt(PathId(c), 0));
        const double gapOut = distance(G.pointAt(PathId(c), p.length), G.pointAt(into, 0));
        const double cosIn = dot(G.directionAt(from, G.paths[from.index].length), endPieceDirection(G, PathId(c), true));
        const double cosOut = dot(endPieceDirection(G, PathId(c), false), G.directionAt(into, 0));
        const double dzIn = std::fabs(G.heightAt(from, G.paths[from.index].length) - G.heightAt(PathId(c), 0));
        const double dzOut = std::fabs(G.heightAt(PathId(c), p.length) - G.heightAt(into, 0));
        if (gapIn > kTestJoinDistance || gapOut > kTestJoinDistance || cosIn < joinCos || cosOut < joinCos || dzIn > kTestJoinDistance ||
            dzOut > kTestJoinDistance)
            if (badEnds++ < 4) std::printf("      connector %u: ends %.4f / %.4f m off, cos %.6f / %.6f, heights %.4f / %.4f m off\n", c,
                                           gapIn, gapOut, cosIn, cosOut, dzIn, dzOut);
        double widest = -1;
        for (int v = 0; v < 2; ++v) {
            if (tagOf[v] == kNoName || !(p.vehicleTags & (TagMask(1) << tagOf[v]))) continue;
            if (p.tightestRadius < radiusOf[v] - 1e-6 && badRadius++ < 4)
                std::printf("      connector %u: tightest bend %.2f m, open to a vehicle that needs %.2f m\n", c, p.tightestRadius, radiusOf[v]);
            widest = std::max(widest, widthOf[v]);
        }
        CHECK(widest >= 0);   // open to someone, or it would have been refused
        // (at the vehicle's own width and the builder's margin either side, every kSweepStep along, as the builder samples
        // it: a crack between the lanes' footprints narrower than a sample's step can pass between two samples, along the
        // line or across it, here as there -- the lab's one-way frontage roads put a hairline one, under 0.1 m wide and
        // paved either side of it, exactly half a car's width inside a near turn's corner: ADR-0413)
        const int samples = std::max(1, static_cast<int>(std::ceil(p.length / kSweepStep)));
        for (int k = 0; k <= samples && widest >= 0; ++k) {
            const double s = p.length * k / samples;
            const Vec2 at = G.pointAt(PathId(c), s), side = perp(G.directionAt(PathId(c), s)) * (widest / 2 + kSweepMargin);
            if (!paved(at + side, G.heightAt(PathId(c), s)) || !paved(at - side, G.heightAt(PathId(c), s))) {
                if (offRoad++ < 4)
                    std::printf("      connector %u (%s, %.1f m, tightest %.1f m) swept %.1f m wide leaves the paved road %.2f m along it, "
                                "at (%.2f, %.2f)\n", c, modelOf(name).turnOf(PathId(c)) == Turn::Straight ? "straight on" : "a turn", p.length,
                                p.tightestRadius, widest, s, at.x, at.y);
                break;
            }
        }
    }
    CHECK(badEnds == 0);
    CHECK(badRadius == 0);
    CHECK(offRoad == 0);

    // connectors leaving one lane start with the same piece, and those into one lane end with the same piece, bit for bit
    std::map<uint32_t, uint32_t> firstFrom, firstInto;
    int unshared = 0;
    for (uint32_t c = 0; c < G.paths.size(); ++c) {
        const Path& p = G.paths[c];
        if (p.kind != PathKind::Connector) continue;
        const uint32_t into = G.nextPaths[p.next.first].index;
        const std::map<uint32_t, uint32_t>::iterator a = firstFrom.emplace(p.from.index, c).first;
        const std::map<uint32_t, uint32_t>::iterator b = firstInto.emplace(into, c).first;
        const Range ra = G.paths[a->second].curve, rb = G.paths[b->second].curve;
        const bool sameStart = G.points[ra.first].x == G.points[p.curve.first].x && G.points[ra.first].y == G.points[p.curve.first].y &&
                               G.points[ra.first + 1].x == G.points[p.curve.first + 1].x && G.points[ra.first + 1].y == G.points[p.curve.first + 1].y &&
                               G.heights[ra.first] == G.heights[p.curve.first];
        const bool sameEnd = G.points[rb.end() - 1].x == G.points[p.curve.end() - 1].x && G.points[rb.end() - 1].y == G.points[p.curve.end() - 1].y &&
                             G.points[rb.end() - 2].x == G.points[p.curve.end() - 2].x && G.points[rb.end() - 2].y == G.points[p.curve.end() - 2].y &&
                             G.heights[rb.end() - 1] == G.heights[p.curve.end() - 1];
        if ((!sameStart || !sameEnd) && unshared++ < 4) std::printf("      connector %u does not share its %s exactly\n", c, sameStart ? "end" : "start");
    }
    CHECK(unshared == 0);
}
}  // namespace

TEST_CASE(lanes_connectors_of_grid_city_meet_their_lanes_and_fit_their_vehicles) { checkConnectors("grid_city"); }
TEST_CASE(lanes_connectors_of_hill_junction_meet_their_lanes_and_fit_their_vehicles) { checkConnectors("hill_junction"); }
TEST_CASE(lanes_connectors_of_ring_city_meet_their_lanes_and_fit_their_vehicles) { checkConnectors("ring_city"); }
TEST_CASE(lanes_connectors_of_valley_viaduct_meet_their_lanes_and_fit_their_vehicles) { checkConnectors("valley_viaduct"); }
TEST_CASE(lanes_connectors_of_the_traffic_lab_meet_their_lanes_and_fit_their_vehicles) { checkConnectors("planned/traffic_lab_lanelab"); }

// THE TRAFFIC LAB IS ROUTABLE (#238): a car on almost any lane can get to almost any other -- the largest set of lanes
// that can all reach one another holds at least 90% of them -- and no lane end is half-joined.
// (Its known problems by count, with the tests' two vehicles: a change to any is a change to look at, then to write here.)
// (step 9, ADR-0428: the stop lines moved to the stated rule -- fits none 82 -> 77, crossing lines 17 -> 29, kinks 65 -> 63)
// (ADR-0450: kerb lanes driven clear of the parking band, 1.25 m nearer the centre line and 3.5 m wide -- fits none
// 76 -> 75, crossing lines 29 -> 16 (narrower paths beside each other touch less), tidied folds 13 -> 11)
constexpr uint32_t kTrafficLabFitsNone = 75,   // 77 before ADR-0432: the carriageway half width is exact now, not read back off float lane bands
                    kTrafficLabCrossingLines = 16, kTrafficLabKinks = 63, kTrafficLabTidied = 11;
TEST_CASE(lanes_the_traffic_lab_lane_graph_can_be_driven_round) {
    const LaneGraphBuild& lg = laneGraphOf("planned/traffic_lab_lanelab");
    std::printf("    traffic lab: %.1f%% of the lanes in the largest part a car can get round\n", 100 * lg.reachableShare);
    for (const std::pair<std::string, std::vector<uint32_t>>& o : lg.outside) std::printf("      %zu lanes %s\n", o.second.size(), o.first.c_str());
    CHECK(lg.reachableShare >= 0.9);
    // EVERY RAMP IS A WAY ON AND OFF (ADR-0426): each ramp's lane leaves its host's lane and leads into the road it joins
    const RoadModel& lab = modelOf("planned/traffic_lab_lanelab");
    const LaneGraph& G = lab.laneLevel;
    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        const Path& path = G.paths[p];
        if (path.kind != PathKind::Lane || lab.roadClassOf(PathId(p)) != "ramp") continue;
        CHECK(path.next.count > 0);
    }
    CHECK(lg.divergeJoins == 8u && lg.mergeJoins == 8u);
    // NO LANE END HALF-JOINED (the old `unjoined.empty()`): the lane level joins only what the model states, so a half
    // join is a stated join whose lines miss -- exactly one here, the exit d1_0_a_off leaving fw1_a's kerb lane 0.011 m off
    CHECK(lg.joinsWithGap == 1u);
    // (NO TWO ROADS' LANES CROSSING AT ONE LEVEL OUTSIDE EVERY JUNCTION, the old `crossingsOutside == 0`, is measured from
    // the lanes' own centre lines in checkLaneGraph, part 4: lanes_lane_graph_of_the_traffic_lab_is_sound_and_covers_every_lane.)
    // EVERY PROBLEM IS ONE WE KNOW, BY COUNT (and by id where it names a place to fix), so a new one fails: a stated turn
    // that fits no vehicle (a corner fault), a fold tidied, a crossing point to look at (lane_graph_crossings.h), a kink in a
    // line (#281: the road builder's to fix, reported, not smoothed), the one stated join whose lines miss, a junction whose
    // arms' stop lines overlap so its outline crosses itself (j0 and j22: corners for step 9), and a street end that meets
    // another road where the road file states no junction (the dead-end spokes at the ring: the planner's to state)
    std::map<std::string, uint32_t> byKind;
    std::set<std::string> outlines, unstated, gapJoins;
    for (const std::string& p : problemLines(lg.problems)) {
        std::string kind = "unknown";
        if (p.find(" fits no vehicle: ") != std::string::npos) kind = "fits no vehicle";
        else if (p.find("; tidied (") != std::string::npos) kind = "tidied";
        else if (p.find("crossing points: ") == 0) kind = "crossing points";
        else if (p.find("kink: ") == 0 && p.find(" at (") != std::string::npos) kind = "kink";
        else if (p.find(" apart at (") != std::string::npos && p.find("): joined") != std::string::npos) {
            kind = "join with a gap";
            gapJoins.insert(p.substr(0, p.find(", their lines")));
        } else if (p.find("its outline crosses itself") != std::string::npos) {
            kind = "outline";
            outlines.insert(p.substr(std::string("junction ").size(), p.find(" at (") - std::string("junction ").size()));
        } else if (p.find("states no junction there") != std::string::npos) {
            kind = "no junction stated";
            unstated.insert(p.substr(std::string("street ").size(), p.find(':') - std::string("street ").size()));
        }
        if (kind == "unknown") std::printf("      problem: %s\n", p.c_str());
        ++byKind[kind];
    }
    const std::map<std::string, uint32_t> knownKinds = {{"fits no vehicle", kTrafficLabFitsNone}, {"tidied", kTrafficLabTidied}, {"crossing points", kTrafficLabCrossingLines},
                                                        {"kink", kTrafficLabKinks}, {"join with a gap", 1}, {"outline", 5}, {"no junction stated", 17}};
    for (const std::pair<const std::string, uint32_t>& k : byKind) std::printf("      %u problems: %s\n", k.second, k.first.c_str());
    CHECK(byKind == knownKinds);
    // (j4, j19, j93, j96 since step 9, and j22 no longer, ADR-0428: two arms' stop lines, stated by the rule, cross where
    // they meet at a sharp angle)
    CHECK(outlines == std::set<std::string>({"j0", "j4", "j19", "j93", "j96"}));
    CHECK(gapJoins == std::set<std::string>({"lane d1_0_a_off.f0 leaves lane fw1_a.f3"}));
    CHECK(unstated == std::set<std::string>({"s208", "s283", "s288", "s290", "s292", "s295", "s297", "s299", "s301", "s303", "s308",
                                             "s326", "s328", "s330", "s332", "s335", "s337"}));
}

// THE TRAFFIC LAB'S ROAD MODEL'S OWN PROBLEMS, BY KIND AND COUNT (written from its model_problems.txt): 8 sides taken as
// far turns where an arm with no through pair stated stands halfway round (j7, j18, j82); and NONE of: an arriving lane
// no turn-table row covers, anything else. (Since ADR-0428 the paint draws the stated junctions: no paint box can
// disagree with them, so those problem lines are gone.)
TEST_CASE(lanes_the_traffic_lab_road_model_problems_are_known) {
    const RoadModel& m = modelOf("planned/traffic_lab_lanelab");
    std::map<std::string, uint32_t> byKind;
    std::set<std::string> halfway;
    for (const std::string& p : m.problems) {
        std::string kind = "other";
        if (p.find(" takes no turn: ") != std::string::npos) kind = "arriving lane taking no turn";
        else if (p.find(" stands halfway round from arm ") != std::string::npos) {
            kind = "halfway, taken as far";
            // named by the arms' roads (their places in the list are the stated ring's, ADR-0427)
            unsigned a = 0, b = 0;
            char id[32] = {0};
            if (std::sscanf(p.c_str(), "junction %31[^:]: arm %u stands halfway round from arm %u", id, &a, &b) == 3)
                for (const ModelJunction& J : m.junctions)
                    if (J.id == id && a < J.arms.size() && b < J.arms.size())
                        halfway.insert(J.id + ": " + J.arms[a].road + " halfway from " + J.arms[b].road);
        }
        if (kind == "other") std::printf("      model problem: %s\n", p.c_str());
        ++byKind[kind];
    }
    for (const std::pair<const std::string, uint32_t>& k : byKind) std::printf("      %u model problems: %s\n", k.second, k.first.c_str());
    CHECK(byKind == (std::map<std::string, uint32_t>{{"halfway, taken as far", 8}}));
    CHECK(halfway == (std::set<std::string>{"j7: s203 halfway from s16", "j7: s230 halfway from s13", "j18: s219 halfway from s28",
                                            "j18: s28 halfway from s219", "j18: s23 halfway from s31", "j18: s31 halfway from s23",
                                            "j82: s141 halfway from s224", "j82: s224 halfway from s141"}));
}

// THE TRAFFIC LAB'S BENDS LOOK LIKE ITS ROADS (#281): every path's bend profile measured as a vehicle feels it. The
// lanes of the class with the highest posted limit (the lab's freeway ring, about 290 m round) bend in the hundreds of
// metres, not the 27.5 m the old point-to-point reading gave; each lane's tightest radius is its profile's tightest; a
// profile is small; and every kink counted is listed in the problems with its place.
TEST_CASE(lanes_the_traffic_lab_bends_look_like_its_roads) {
    const LaneGraphBuild& lg = laneGraphOf("planned/traffic_lab_lanelab");
    const RoadModel& lab = modelOf("planned/traffic_lab_lanelab");
    const LaneGraph& G = lab.laneLevel;
    double fastest = 0;
    for (uint32_t i = 0; i < G.paths.size(); ++i)
        if (G.paths[i].kind == PathKind::Lane) fastest = std::max(fastest, lab.postedLimit(PathId(i)));
    std::vector<double> radii;
    uint32_t mismatched = 0;
    for (uint32_t i = 0; i < G.paths.size(); ++i) {
        const Path& p = G.paths[i];
        if (p.kind != PathKind::Lane) continue;
        if (G.tightestBend(PathId(i)).radius != p.tightestRadius) ++mismatched;
        if (lab.postedLimit(PathId(i)) == fastest) radii.push_back(G.tightestBend(PathId(i)).radius);
    }
    std::sort(radii.begin(), radii.end());
    CHECK(!radii.empty());
    const double median = radii.empty() ? 0 : radii[radii.size() / 2];
    std::printf("    the fastest lanes (%.1f m/s): %zu, median tightest bend %.1f m; %zu bend pieces (%zu bytes); kinks: %u in "
                "lanes (worst %.1f deg), %u in connectors (worst %.1f deg), %u where paths join\n",
                fastest, radii.size(), median, G.bendPieces.size(), lg.bendPieceBytes, lg.laneKinks, lg.worstLaneKink,
                lg.connectorKinks, lg.worstConnectorKink, lg.joinKinks);
    CHECK(median > 200 && median < kStraightEnoughRadius);
    CHECK(mismatched == 0u);
    CHECK(G.bendPieces.size() < 4 * G.paths.size());   // a handful of pieces a path at most, on the whole
    uint32_t listed = 0;
    for (const RoadProblem& p : lg.problems)
        listed += p.kind == RoadProblemKind::LaneKink || p.kind == RoadProblemKind::ConnectorKink || p.kind == RoadProblemKind::JoinKink ? 1 : 0;
    CHECK(listed == lg.laneKinks + lg.connectorKinks + lg.joinKinks);
}

// ---- a hand-made junction (the crossing-point tests below) ----------------------------------------------------------------
namespace {
// A hand-made junction's legs: roads arriving from the given compass bearings (degrees, the way they point away from the
// junction's middle), `lanes` lanes each way, 3.5 m wide, ends 10 m out. Lane ids: road r's arriving lane k is
// r * 100 + k, its leaving lane k r * 100 + 50 + k (k from the centre line to the kerb: its slot).
constexpr double kHandLaneWidth = 3.5, kHandReach = 10.0, kHandDegrees = 3.14159265358979 / 180.0;
struct HandLeg {
    Vec2 heading;                  // the way its traffic runs at the junction (a unit vector)
    std::vector<uint32_t> lanes;   // lane ids, centre-line side to kerb
    std::vector<Vec2> ends;        // where each lane ends (arriving) or begins (leaving), in the same order
};
void handJunction(const std::vector<double>& bearings, const std::vector<uint32_t>& lanesEachWay, double kerbPerp, bool mirrored,
                  std::vector<HandLeg>& arriving, std::vector<HandLeg>& leaving) {
    arriving.clear();
    leaving.clear();
    for (std::size_t r = 0; r < bearings.size(); ++r) {
        Vec2 out(std::cos(bearings[r] * kHandDegrees), std::sin(bearings[r] * kHandDegrees));
        if (mirrored) out.y = -out.y;
        HandLeg in, away;
        in.heading = out * -1.0;
        away.heading = out;
        for (uint32_t k = 0; k < lanesEachWay[r]; ++k) {
            const double offset = (k + 0.5) * kHandLaneWidth;   // kerbward of the middle line
            in.lanes.push_back(static_cast<uint32_t>(r * 100 + k));
            in.ends.push_back(out * kHandReach + perp(in.heading) * (kerbPerp * offset));
            away.lanes.push_back(static_cast<uint32_t>(r * 100 + 50 + k));
            away.ends.push_back(out * kHandReach + perp(away.heading) * (kerbPerp * offset));
        }
        arriving.push_back(in);
        leaving.push_back(away);
    }
}
// THE TURNS OF A HAND-MADE JUNCTION, as the road model states a junction's (lane_turns.h, ADR-0426): its arms listed in
// their ring, counter-clockwise (ADR-0427: the hand scene is the writer of its own ring -- by heading here, every arm
// meeting at one point), the roads whose bearings lie within 30 degrees of opposite stated as through pairs (the nearest
// first, each road in one pair), its lanes by slot. `kerbPerp` and `mirrored` as handJunction's (a mirrored world keeps
// its kerb on the other side, and its ring runs the other way through the roads). The turns come back road by road (the
// road the turn leaves, the table's row, the road it takes), so a junction and its mirror list them alike.
std::vector<StatedTurn> handTurns(const std::vector<double>& bearings, const std::vector<uint32_t>& lanes, double kerbPerp,
                                  bool mirrored) {
    std::vector<TurnArm> byRoad(bearings.size());
    for (std::size_t r = 0; r < bearings.size(); ++r) {
        byRoad[r].heading = mirrored ? std::fmod(360.0 - bearings[r], 360.0) : bearings[r];
        byRoad[r].hasHeading = true;
        for (uint32_t k = 0; k < lanes[r]; ++k) {
            byRoad[r].arriving.push_back(TurnArmLane{static_cast<uint32_t>(r * 100 + k), "through", static_cast<int>(k)});
            byRoad[r].leaving.push_back(TurnArmLane{static_cast<uint32_t>(r * 100 + 50 + k), "through", static_cast<int>(k)});
        }
    }
    std::vector<uint32_t> roadOf(bearings.size()), armOf(bearings.size());   // the ring: arm -> road, road -> arm
    for (uint32_t r = 0; r < bearings.size(); ++r) roadOf[r] = r;
    std::stable_sort(roadOf.begin(), roadOf.end(), [&](uint32_t x, uint32_t y) { return byRoad[x].heading < byRoad[y].heading; });
    std::vector<TurnArm> arms;
    for (uint32_t a = 0; a < roadOf.size(); ++a) {
        armOf[roadOf[a]] = a;
        arms.push_back(byRoad[roadOf[a]]);
    }
    std::vector<std::pair<double, std::array<uint32_t, 2>>> candidates;
    for (uint32_t a = 0; a < bearings.size(); ++a)
        for (uint32_t b = a + 1; b < bearings.size(); ++b) {
            const double off = std::fabs(std::fabs(std::fmod(bearings[b] - bearings[a] + 540.0, 360.0) - 180.0));
            if (off < 30.0) candidates.push_back({off, {a, b}});
        }
    std::sort(candidates.begin(), candidates.end(), [](const std::pair<double, std::array<uint32_t, 2>>& x,
                                                       const std::pair<double, std::array<uint32_t, 2>>& y) { return x.first < y.first; });
    std::vector<std::array<uint32_t, 2>> pairs;
    std::vector<char> paired(bearings.size(), 0);
    for (const std::pair<double, std::array<uint32_t, 2>>& c : candidates) {
        if (paired[c.second[0]] || paired[c.second[1]]) continue;
        paired[c.second[0]] = paired[c.second[1]] = 1;
        pairs.push_back({armOf[c.second[0]], armOf[c.second[1]]});
    }
    // which way round the kerb lies, for this junction's kerb sign (as kerbSideCounterClockwise reads the engine's: a
    // negative sign puts the kerb counter-clockwise from the arm; the mirrored world's headings are its own)
    std::vector<StatedTurn> turns = turnsAtJunction(arms, pairs, "unmarked", laneTurnRules(), nullptr, kerbPerp < 0);
    for (StatedTurn& t : turns) t.fromArm = roadOf[t.fromArm], t.toArm = roadOf[t.toArm];
    std::stable_sort(turns.begin(), turns.end(), [](const StatedTurn& x, const StatedTurn& y) {
        return std::make_tuple(x.fromArm, x.row, x.toArm) < std::make_tuple(y.fromArm, y.row, y.toArm);
    });
    return turns;
}
}  // namespace

TEST_CASE(lanes_connector_shape_is_straight_arc_straight_with_the_largest_arc) {
    const Vec2 from(0, -8), fromDir(0, 1), to(6, 0), toDir(1, 0);   // lines cross at (0, 0): 8 m and 6 m away
    const ConnectorShape s = connectorShape(from, fromDir, to, toDir);
    CHECK(s.ok && s.singleArc);
    CHECK_APPROX(s.tightestRadius, 6.0 - kConnectorLeadIn, 1e-9);   // the shorter reach, less the lead-in, at 90 degrees
    CHECK(s.points.front().x == from.x && s.points.front().y == from.y);
    CHECK(s.points.back().x == to.x && s.points.back().y == to.y);
    CHECK(dot(normalize(s.points[1] - s.points[0]), fromDir) > 1 - 1e-12);
    CHECK(dot(normalize(s.points.back() - s.points[s.points.size() - 2]), toDir) > 1 - 1e-12);
    // every point lies on the lead-in, the straight before the arc, or the circle round (R, -R) from the cross
    const double R = s.tightestRadius;
    for (const Vec2& p : s.points) {
        const bool onStraights = (std::fabs(p.x) < 1e-9 && p.y <= -R + 1e-9) || (std::fabs(p.y) < 1e-9 && p.x >= R - 1e-9);
        CHECK(onStraights || std::fabs(distance(p, Vec2(R, -R)) - R) < 1e-9);
    }
    // asked for no more than 4 m, it bends at 4 m
    CHECK_APPROX(connectorShape(from, fromDir, to, toDir, 4.0).tightestRadius, 4.0, 1e-9);
    // a jog sideways along one direction: two arcs
    const ConnectorShape jog = connectorShape(Vec2(0, 0), Vec2(1, 0), Vec2(10, 1), Vec2(1, 0));
    CHECK(jog.ok && !jog.singleArc && jog.tightestRadius > 1 && std::isfinite(jog.tightestRadius));
    // straight on along one line: no bend
    const ConnectorShape line = connectorShape(Vec2(0, 0), Vec2(1, 0), Vec2(10, 0), Vec2(1, 0));
    CHECK(line.ok && line.tightestRadius == kNoBend);
}

// WHAT EACH SCENE IS KNOWN TO HOLD (LaneGraphKnown), written from looking at it:
//   - no scene has a lane cars drive made into no lane path, nor a stretch of one with no lane path outside every
//     junction; the traffic lab's 36 roads stated inside its spread junctions (the file's junctions[].inside) are wholly
//     inside them;
//   - joins between two different lanes missing by more than 1 cm or turning by more than a degree, by builder lane
//     ("from>into"), with their gap (m) and turn (degrees): grid_city's corner streets carry on round a right angle (the
//     layout states each bend as a carry-on: the two lanes' ends 2.5 m apart at 90 degrees) and its boulevard's and local
//     streets' carry-ons at its east end; ring_city's inner and outer rings' carry-ons, 2 cm and a third of a degree apart
//     at their joints, and its grid's corner carry-ons; the traffic lab's ramps' ends joined end to end to a street at a
//     degree or two, and the exit d1_0_a_off leaving fw1_a's kerb lane 0.011 m off. Of these values only the right-angle
//     corners' are worked out by hand (two lanes 1.75 m off their roads' centre lines, ending at the corner: sqrt(2) x
//     1.75 = 2.4749 m apart, at 90 degrees); every other gap and turn here is a snapshot of the built lanes, read off a run;
//   - lane paths inside a junction's outline: the traffic lab's j93, a spread junction whose outline, drawn round its
//     arms' stop lines, takes in one lane path;
//   - lanes of two roads crossing at one level outside every junction (faults of the scenes' road files, for step 9 or the
//     scene's author): grid_city's boulevard's second lane (blvd.b1) across the local street's lane at their carry-on
//     corner, the corner a 100-degree turn with the two lanes 2.5 to 5 m apart; ring_city's two ramps in its north-east
//     quarter, the east interchange's off-ramp e_in_off and the north one's on-ramp n_in_on, crossing each other at grade
//     at (253.7, 222.7) with no junction or structure stated; the traffic lab's j0 and j84, each with two arms (s33 and s36;
//     s147 and s149) meeting at one point at a sharp angle, so their lanes cross some 20 m before the arms' stop lines
//     (the outline is drawn at the stop lines, so the crossing lies outside it).
LaneGraphKnown gridCityKnown() {
    LaneGraphKnown k;
    k.crossings = {"blvd.b1 x loc_e.b0 @ (638, 25)"};
    k.carryOnJoins = 11;   // its 6 corner bends and the boulevard's and local streets' 5 carry-on ends (the layout's)
    k.divergeJoins = 1;    // the exit ramp
    k.mergeJoins = 1;      // the entry ramp
    k.merges = {"entry.f0>fw_wb.f2"};   // the file: entry's `to` anchor on fw_wb, side right: its outer lane, f2 of three
    // (the one "other": blvd.f1 carrying on into loc_e, which has no lane in its slot)
    // (fits no vehicle 1 -> 2 at step 9, ADR-0428: the stop lines moved to the stated rule, one more corner fault)
    k.problems = {{"crossing points", 28}, {"fits no vehicle", 2}, {"join with a gap", 11}, {"kink", 11}, {"other", 1}};
    k.joins = {{"v0.b0>h0.f0", {2.4749, 90.0}}, {"v0.f0>h4.f0", {2.4749, 90.0}}, {"v5.b0>h0.b0", {2.4749, 90.0}},
               {"h0.b0>v0.f0", {2.4749, 90.0}}, {"h0.f0>v5.f0", {2.4749, 90.0}}, {"h4.b0>v0.b0", {2.4749, 90.0}},
               {"blvd.f0>loc_e.f0", {2.4885, 99.65}}, {"blvd.f1>loc_e.f0", {5.0944, 99.65}}, {"loc_s.f0>loc_e.b0", {1.6978, 58.04}},
               {"loc_e.b0>blvd.b0", {2.4885, 99.65}}, {"loc_e.f0>loc_s.b0", {1.6978, 58.04}}};
    return k;
}
LaneGraphKnown ringCityKnown() {
    LaneGraphKnown k;
    for (const char* j : {"in_NE.f0>in_ES.f0", "in_NE.f2>in_ES.f2", "in_ES.f0>in_SW.f0", "in_ES.f2>in_SW.f2", "in_SW.f0>in_WN.f0",
                          "in_SW.f2>in_WN.f2", "in_WN.f0>in_NE.f0", "in_WN.f2>in_NE.f2"})
        k.joins[j] = {0.0211, 0.34};
    for (const char* j : {"out_EN.f0>out_NW.f0", "out_EN.f2>out_NW.f2", "out_NW.f0>out_WS.f0", "out_NW.f2>out_WS.f2", "out_WS.f0>out_SE.f0",
                          "out_WS.f2>out_SE.f2", "out_SE.f0>out_EN.f0", "out_SE.f2>out_EN.f2"})
        k.joins[j] = {0.0201, 0.32};
    for (const char* j : {"v0.b0>h0.f0", "v0.f0>h3.f0", "h0.b0>v0.f0", "h0.f0>v3.f0", "v3.b0>h0.b0", "v3.f0>h3.b0", "h3.b0>v0.b0", "h3.f0>v3.b0"})
        k.joins[j] = {2.4749, 90.0};
    k.crossings = {"e_in_off.f0 x n_in_on.f0 @ (254, 223)"};
    k.carryOnJoins = 32;   // the inner and outer rings' 3 lanes each at their 4 joints (24) and the grid's 8 corner lanes
    k.divergeJoins = 4;    // its 4 exit ramps
    k.mergeJoins = 4;      // its 4 entry ramps
    // the file: each *_on ramp's `to` anchor on its ring carriageway, side right: the ring's outer lane, f2 of three
    k.merges = {"n_in_on.f0>in_NE.f2", "e_in_on.f0>in_ES.f2", "e_out_on.f0>out_EN.f2", "n_out_on.f0>out_NW.f2"};
    k.problems = {{"crossing points", 42}, {"fits no vehicle", 4}, {"join with a gap", 28}, {"kink", 8}};
    return k;
}
LaneGraphKnown hillJunctionKnown() {
    LaneGraphKnown k;
    k.divergeJoins = 2;   // its two turn pockets, each swept out of its host lane
    k.problems = {{"crossing points", 4}};
    return k;
}
LaneGraphKnown valleyViaductKnown() {
    LaneGraphKnown k;
    k.mergeJoins = 2;   // its ramp's two lanes: the second into the first, the first into its host
    // the file: ramp (two lanes, "dovetail") anchored `to` fwy, side right: fwy's outer lane f2
    k.merges = {"ramp.f1>ramp.f0", "ramp.f0>fwy.f2"};
    k.problems = {{"fits no vehicle", 1}};
    return k;
}
LaneGraphKnown trafficLabKnown() {
    LaneGraphKnown k;
    // lanes of two roads crossing before their stop lines: at j0, s33's and s36's (two arms meeting at one point at a sharp
    // angle; two of the four found before step 9 now lie inside the junction, its stop lines stated by the rule, ADR-0428),
    // at j84 s147's and s149's, and at j22's approach s57's and s60's
    // (moved 2 m with the kerb lanes of s36, s60 and s149 since they are driven clear of the parking band, ADR-0450: s36's
    // two kerb lanes now meet s33's outer lane, where its b0 met f1 and f2)
    k.crossings = {"s33.f2 x s36.b0 @ (-221, -309)", "s33.f2 x s36.f0 @ (-225, -306)", "s57.f0 x s60.b2 @ (-219, 309)",
                   "s147.b2 x s149.f0 @ (210, -309)"};
    // the folds cut out (each lane's "tidied" problem line gives the metres; written here rounded up). (s61.b0 and s65.f0
    // no longer fold: kerb lanes driven 1.25 m nearer their spine, ADR-0450)
    k.tidied = {{"s2.f2", 2.4}, {"s30.f2", 1.7}, {"s90.f2", 1.3}, {"s182.f0", 1.5}, {"s189.f2", 1.5},
                {"s193.b1", 3.1}, {"s212.b2", 2.4}, {"s212.b1", 0.4}, {"s212.f1", 0.6}, {"s212.f2", 2.3}, {"s215.b2", 0.6}};
    k.insideRoads = {"s0", "s1", "s2", "s4", "s26", "s27", "s30", "s34", "s115", "s117", "s145", "s146", "s168", "s169", "s170", "s171", "s172",
                     "s173", "s174", "s176", "s189", "s190", "s191", "s195", "s198", "s200", "s201", "s202", "s204", "s206", "s207", "s211",
                     "s214", "s216", "s217", "s220"};
    // (none: j93's one lane path inside it was its outline folding back over its north-west arm, the legs once laid
    // round by the circle walk; laid round the stated ring, ADR-0427, the outline no longer folds)
    k.inside = {};
    // its 8 one-lane ramps (the file's d*_*_off and d*_*_on), each anchored at both ends: each leaves one host (8
    // diverges) and merges into another (8 merges) -- an off-ramp leaves a freeway carriageway and merges into a frontage
    // road or street, an on-ramp leaves a street and merges into a freeway carriageway's kerb lane (f3 of four)
    k.divergeJoins = 8;
    k.mergeJoins = 8;
    k.merges = {"d0_0_a_off.f0>s311.f0", "d0_0_b_off.f0>s260.f0", "d1_0_a_off.f0>s343.f0", "d1_0_b_off.f0>s242.f0",
                "d0_0_a_on.f0>fw0_a.f3", "d0_0_b_on.f0>fw0_b.f3", "d1_0_a_on.f0>fw1_a.f3", "d1_0_b_on.f0>fw1_b.f3"};
    k.pinProblems = false;   // (pinned by lanes_the_traffic_lab_lane_graph_can_be_driven_round)
    k.joins = {{"s257.f0>d0_0_b_on.f0", {0.0, 1.07}}, {"s277.f0>d1_0_a_on.f0", {0.0002, 1.44}}, {"s315.f0>d0_0_a_on.f0", {0.0001, 1.86}},
               {"fw1_a.f3>d1_0_a_off.f0", {0.0109, 0.57}}, {"d0_0_a_off.f0>s311.f0", {0.0, 1.07}}, {"d0_0_b_off.f0>s260.f0", {0.0, 1.07}},
               {"d1_0_a_off.f0>s343.f0", {0.0, 1.07}}, {"d1_0_b_off.f0>s242.f0", {0.0, 1.07}}};
    return k;
}
TEST_CASE(lanes_lane_graph_of_grid_city_is_sound_and_covers_every_lane) { checkLaneGraph("grid_city", gridCityKnown()); }
TEST_CASE(lanes_lane_graph_of_hill_junction_is_sound_and_covers_every_lane) { checkLaneGraph("hill_junction", hillJunctionKnown()); }
TEST_CASE(lanes_lane_graph_of_ring_city_is_sound_and_covers_every_lane) { checkLaneGraph("ring_city", ringCityKnown()); }
TEST_CASE(lanes_lane_graph_of_valley_viaduct_is_sound_and_covers_every_lane) { checkLaneGraph("valley_viaduct", valleyViaductKnown()); }
TEST_CASE(lanes_lane_graph_of_the_traffic_lab_is_sound_and_covers_every_lane) {
    checkLaneGraph("planned/traffic_lab_lanelab", trafficLabKnown());
}

// One grid_city junction by hand: the two arterials (two lanes each way) crossing at (160, 120). Four approaches of two
// lanes each arrive and four leave; an arriving lane of v2 ends on the paint's stop bar.
TEST_CASE(lanes_lane_graph_a_grid_city_crossing_checked_by_hand) {
    const Result& r = laneSceneOf("grid_city");
    const LaneGraphBuild& lg = laneGraphOf("grid_city");
    const LaneGraph& G = modelOf("grid_city").laneLevel;
    const int v2 = r.graph.index.at("v2"), h2 = r.graph.index.at("h2");
    const Vec2 crossing(160, 120);
    int found = -1;
    for (uint32_t j = 0; j < G.junctions.size(); ++j) {
        const Range b = G.junctions[j].box;
        const std::vector<Vec2> outline(G.planPoints.begin() + b.first, G.planPoints.begin() + b.end());
        if (pointInPolygon(outline, crossing)) found = static_cast<int>(j);
    }
    CHECK(found >= 0);
    if (found < 0) return;
    const Junction& jn = G.junctions[static_cast<std::size_t>(found)];
    CHECK(jn.arriving.count == 8);
    CHECK(jn.leaving.count == 8);
    auto roadOf = [&](PathId p) { return r.lanes.lanes[static_cast<std::size_t>(lg.sources[p.index].lane)].road(); };
    int fromV2 = 0, fromH2 = 0;
    for (uint32_t k = jn.arriving.first; k < jn.arriving.end(); ++k) {
        fromV2 += roadOf(G.junctionLanes[k]) == v2 ? 1 : 0;
        fromH2 += roadOf(G.junctionLanes[k]) == h2 ? 1 : 0;
    }
    CHECK(fromV2 == 4 && fromH2 == 4);
    // v2's arm arriving there along its spine, as the stated junction gives it (junctionAreas: the one rule, ADR-0428): its
    // stop line stands back from its mouth by h2's half width and kStopLineBack; a lane arriving up v2 ends on it
    const EdgeSpec& e = r.graph.edges[static_cast<std::size_t>(v2)];
    const EdgeSpec& cross = r.graph.edges[static_cast<std::size_t>(h2)];
    const double at = project(e.xy, e.s, crossing).station;
    const ArmStop* arm = nullptr;
    for (const JunctionArea& A : junctionAreas(r.graph))
        for (const ArmStop& as : A.arms)
            if (as.edge == v2 && as.spineIn && approxEqual(as.station, at, 1.0)) arm = &as;
    CHECK(arm != nullptr);
    if (!arm) return;
    const double stopBar = arm->station - (r.graph.hw(cross) + kStopLineBack);
    CHECK(approxEqual(arm->stopLine, stopBar, kWrittenStation));
    int onBar = 0;
    for (uint32_t k = jn.arriving.first; k < jn.arriving.end(); ++k) {
        const PathId p = G.junctionLanes[k];
        const Lane& l = r.lanes.lanes[static_cast<std::size_t>(lg.sources[p.index].lane)];
        if (l.road() != v2 || l.dir < 0) continue;
        const Vec2 end = G.pointAt(p, G.paths[p.index].length);
        const Projection pr = project(e.xy, e.s, end);
        std::printf("    v2 lane %s arrives at %.3f m along v2; the stop bar stands at %.3f m, %.2f m off the spine\n", l.id.c_str(),
                    pr.station, stopBar, pr.distance);
        CHECK(std::fabs(pr.station - stopBar) < kTestJoinDistance);
        CHECK(pr.distance < r.graph.hw(e));   // across the road, within its lanes: where the bar is painted
        ++onBar;
    }
    CHECK(onBar == 2);
}

// Offside to kerb: in a section of two lanes each way, lane 0 is the one nearer the road's centre line, both ways.
TEST_CASE(lanes_lane_graph_lists_a_two_lane_section_offside_to_kerb) {
    const Result& r = laneSceneOf("grid_city");
    const LaneGraphBuild& lg = laneGraphOf("grid_city");
    const LaneGraph& G = modelOf("grid_city").laneLevel;
    const int v2 = r.graph.index.at("v2");
    const EdgeSpec& e = r.graph.edges[static_cast<std::size_t>(v2)];
    int checked[2] = {0, 0};
    for (uint32_t s = 0; s < G.sections.size(); ++s) {
        const SectionSource& ss = lg.sectionSources[s];
        if (ss.edge != v2 || G.sections[s].lanes.count != 2) continue;
        const PathId offside = G.sectionLanes[G.sections[s].lanes.first], kerb = G.sectionLanes[G.sections[s].lanes.first + 1];
        const double dOff = project(e.xy, e.s, G.pointAt(offside, G.paths[offside.index].length / 2)).distance;
        const double dKerb = project(e.xy, e.s, G.pointAt(kerb, G.paths[kerb.index].length / 2)).distance;
        CHECK(dOff < dKerb);
        // and both run the section's way: along the spine for +1
        const Vec2 along = tangentAtStation(e.xy, e.s, project(e.xy, e.s, G.pointAt(offside, 0)).station);
        CHECK(dot(G.directionAt(offside, 0), along) * ss.dir > 0);
        ++checked[ss.dir > 0 ? 0 : 1];
    }
    std::printf("    v2: %d two-lane sections along its spine and %d against it, offside lane nearer the centre in each\n", checked[0], checked[1]);
    CHECK(checked[0] > 0 && checked[1] > 0);
}

// ---- A LEVEL'S BUILT ROAD MODEL (ITS LANE LEVEL WITH IT) FROM ITS CACHE (road_model_cache.h) ----------------------------
#include "engine/procgen/city/roads/lanes/road_model_cache.h"

namespace {
// A level holding one lane-built city, the lanelab graph at `graphPath`, as the loader reads it.
engine::bundle::LevelInputs cacheTestLevel(const std::string& graphPath) {
    engine::bundle::LevelInputs in;
    in.levelPath = "lane_graph_cache_test.json";   // named only; the key reads the graph file, not the level file
    in.levelDir = ".";
    in.level = {{"entities", nlohmann::json::array({{{"shape", "lanelab"}, {"lanelab", {{"graph", graphPath}}}}})}};
    return in;
}
LaneGraphOptions cacheTestVehicles() {
    LaneGraphOptions o;
    VehicleSize car, longVehicle;
    car.tag = kLaneGraphTestTag;
    car.turningRadius = kTestCarRadius;
    car.width = kTestCarWidth;
    longVehicle.tag = kLaneGraphLongTag;
    longVehicle.turningRadius = kTestLongRadius;
    longVehicle.width = kTestLongWidth;
    o.vehicles = {car, longVehicle};
    return o;
}
std::vector<char> cacheFileBytes(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return std::vector<char>(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}
void putCacheFile(const std::string& path, const std::vector<char>& bytes) {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}
}  // namespace

// BUILT ONCE, THEN READ: the first load builds the roads and the graph and saves it; the next reads the file with no road
// build and gets the very same graph; a lost file is built again to the same bytes; a damaged or cut-short file is
// refused, built again and written over; no cache folder means built every time and nothing written.
TEST_CASE(lanes_a_level_lane_graph_is_built_once_then_read_from_its_cache) {
    namespace fs = std::filesystem;
    const fs::path dirPath = fs::temp_directory_path() / "rt_lanes_lane_graph_cache";
    std::error_code ec;
    fs::remove_all(dirPath, ec);
    const std::string dir = dirPath.string();
    // the hill junction with the lane level's facts stated, as a road file of its own (lane_level_scene.h)
    fs::create_directories(dirPath, ec);
    const std::string graphPath = (dirPath / "hill_junction.json").string();
    nlohmann::json withFacts;
    {
        std::ifstream f(std::string(RT_SOURCE_DIR) + "/assets/lanelab/hill_junction.json");
        nlohmann::json g;
        f >> g;
        withFacts = lane_level_test::withLaneLevelFacts(g);
        std::ofstream(graphPath) << withFacts.dump(1);
    }
    const engine::bundle::LevelInputs in = cacheTestLevel(graphPath);
    const LaneGraphOptions opts = cacheTestVehicles();

    const LevelBuiltRoads first = obtainLevelRoadModel(in, dir, &opts);
    std::printf("    first load: %s\n", describeLevelRoadModel(first).c_str());
    CHECK(first.model != nullptr);
    if (!first.model) return;
    CHECK(first.model->built && !first.model->lanes.empty());   // the whole built model, not the lane level alone
    CHECK(!first.fromCache && first.saved && first.roadSeconds > 0);
    CHECK(levelRoadsKey(in) != 0);
    CHECK(first.notes.empty());
    CHECK(first.summary.connectors > 0);
    // the very graph the lane graph tests build from the same roads
    CHECK(first.model->laneLevel == modelOf("hill_junction").laneLevel);
    CHECK(first.summary == summarizeLaneGraph(modelOf("hill_junction").laneLevel, laneGraphOf("hill_junction")));
    // THE KEY MOVES with what the lane level reads beyond the roads: the file's lane rules (another file, another key)
    // and the turn table's bytes (its hash), each on its own; the key is the one roadModelCacheKey makes
    {
        const uint64_t facts = laneLevelKey(withFacts);
        const uint64_t kerb = kerbParkingRules().hash;
        CHECK(first.key == roadModelCacheKey(levelRoadsKey(in), facts, roadModelTablesHash(laneTurnRules().hash, kerb), opts));
        // the table's hash is of its file's bytes: the shipped file's, and another text one byte longer (a comment
        // edited) hashes otherwise, and keys another file
        std::ifstream tf(std::string(RT_SOURCE_DIR) + "/assets/data/lane_turns.json", std::ios::binary);
        const std::string table((std::istreambuf_iterator<char>(tf)), std::istreambuf_iterator<char>());
        CHECK(!table.empty() && laneTurnRules().hash == laneTurnTableHash(table));
        const uint64_t edited = laneTurnTableHash(table + " ");
        CHECK(edited != laneTurnRules().hash);
        CHECK(roadModelCacheKey(levelRoadsKey(in), facts, roadModelTablesHash(edited, kerb), opts) != first.key);
        // ...and with the kerb parking rules' bytes (ADR-0439): the saved model holds the spaces laid by them
        std::ifstream kf(std::string(RT_SOURCE_DIR) + "/assets/data/kerb_parking.json", std::ios::binary);
        const std::string kerbText((std::istreambuf_iterator<char>(kf)), std::istreambuf_iterator<char>());
        CHECK(!kerbText.empty() && kerb == bundle::fnv1aStr(kerbText));
        CHECK(roadModelCacheKey(levelRoadsKey(in), facts, roadModelTablesHash(laneTurnRules().hash, bundle::fnv1aStr(kerbText + " ")), opts) !=
              first.key);
        CHECK(roadModelTablesHash(1, 2) != roadModelTablesHash(2, 1));   // (which table changed is told apart)
        nlohmann::json otherRules = withFacts;
        otherRules["lane_rules"]["solid_before_stop_line"] = 25.0;
        CHECK(laneLevelKey(otherRules) != facts);
        const std::string otherPath = (dirPath / "hill_junction_rules.json").string();
        std::ofstream(otherPath) << otherRules.dump(1);
        const LevelBuiltRoads otherKey = obtainLevelRoadModel(cacheTestLevel(otherPath), std::string(), &opts);
        CHECK(otherKey.key != first.key);
    }
    const std::vector<char> bytes = cacheFileBytes(first.path);
    CHECK(!bytes.empty());

    const LevelBuiltRoads second = obtainLevelRoadModel(in, dir, &opts);
    std::printf("    second load: %s\n", describeLevelRoadModel(second).c_str());
    CHECK(second.fromCache && !second.saved && second.roadSeconds == 0);
    CHECK(second.model != nullptr && *second.model == *first.model);   // the whole model, its lane level in it
    CHECK(second.summary == first.summary);
    CHECK(dumpText(second.model->laneLevel) == dumpText(first.model->laneLevel));

    // lost: built again, to the same bytes
    fs::remove(first.path, ec);
    const LevelBuiltRoads rebuilt = obtainLevelRoadModel(in, dir, &opts);
    CHECK(!rebuilt.fromCache && rebuilt.saved);
    CHECK(cacheFileBytes(first.path) == bytes);

    // damaged in the middle of the graph, or cut short: refused (a note says why), built again, written over
    for (int damage = 0; damage < 2; ++damage) {
        std::vector<char> bad = bytes;
        if (damage == 0) bad[bad.size() / 2] = static_cast<char>(~bad[bad.size() / 2]);
        else bad.resize(bad.size() / 2);
        putCacheFile(first.path, bad);
        const LevelBuiltRoads again = obtainLevelRoadModel(in, dir, &opts);
        CHECK(!again.fromCache && again.saved);
        CHECK(again.model != nullptr && *again.model == *first.model);
        CHECK(again.notes.size() == 1 && again.notes[0].find("refused") != std::string::npos);
        if (!again.notes.empty()) std::printf("    %s\n", again.notes[0].c_str());
        CHECK(cacheFileBytes(first.path) == bytes);
        CHECK(obtainLevelRoadModel(in, dir, &opts).fromCache);
    }

    // other vehicles: another key, another file; the first stays
    LaneGraphOptions fewer = opts;
    fewer.vehicles.pop_back();
    const LevelBuiltRoads other = obtainLevelRoadModel(in, dir, &fewer);
    CHECK(other.key != first.key && other.path != first.path && !other.fromCache);
    CHECK(fs::exists(first.path, ec));

    // no cache folder (RT_NOCACHE): built, nothing written
    const LevelBuiltRoads uncached = obtainLevelRoadModel(in, std::string(), &opts);
    CHECK(uncached.model != nullptr && !uncached.fromCache && !uncached.saved && uncached.path.empty());
    CHECK(*uncached.model == *first.model);

    // a level with no lane-built city: no graph, and why
    engine::bundle::LevelInputs empty;
    empty.level = {{"entities", nlohmann::json::array()}};
    const LevelBuiltRoads none = obtainLevelRoadModel(empty, dir, &opts);
    CHECK(none.model == nullptr && !none.error.empty());
    fs::remove_all(dirPath, ec);
}

// ---- WHERE PATHS CROSS, JOIN AND SPLIT (#239, lane_graph_crossings.h) ---------------------------------------------------
#include "engine/procgen/city/roads/lanes/lane_graph_crossings.h"

namespace {
// The crossing tests measure with the lane graph tests' car and long vehicle (the long one, 2.6 m, is the widest: two
// paths open to it touch closer than 2.6 + kCrossingMargin = 3.1 m), at the road builder's default same-level figure.
CrossingOptions crossingTestOptions() {
    CrossingOptions o;
    o.vehicles = cacheTestVehicles().vehicles;
    o.clearance = Rules().sameLevelDz;
    return o;
}
double testReach() { return kTestLongWidth + kCrossingMargin; }
constexpr double kHandLaneLength = 30.0;
constexpr double kMiniLaneLength = 20.0;
constexpr TagMask kBothTestVehicles = 3;

// Adds a path open to both test vehicles, with an empty next list beside it.
uint32_t addTestPath(LaneGraph& g, std::vector<std::vector<PathId>>& next, PathKind kind, const std::vector<Vec2>& pts,
                     const std::vector<Real>& z) {
    Path p;
    p.kind = kind;
    p.vehicleTags = kBothTestVehicles;
    p.curve = g.addCurve(pts, z);
    p.length = g.along[p.curve.end() - 1];
    p.width = kHandLaneWidth;
    g.paths.push_back(p);
    next.emplace_back();
    return static_cast<uint32_t>(g.paths.size() - 1);
}
// Puts a lane in a section of its own.
void ownSection(LaneGraph& g, uint32_t lane) {
    Section s;
    s.lanes = Range{static_cast<uint32_t>(g.sectionLanes.size()), 1};
    s.lines = Range{static_cast<uint32_t>(g.laneLines.size()), 0};
    g.paths[lane].section = SectionId(static_cast<uint32_t>(g.sections.size()));
    g.sectionLanes.push_back(PathId(lane));
    g.sections.push_back(s);
}
LaneGraph emptyTestGraph() {
    LaneGraph g;
    g.vehicleTagNames = {kLaneGraphTestTag, kLaneGraphLongTag};
    return g;
}
// Writes the next lists and the one junction: `connectors` through it, `arriving` and `leaving` lanes, its outline round
// `ends`.
void finishTestGraph(LaneGraph& g, std::vector<std::vector<PathId>>& next, std::vector<PathId> arriving, std::vector<PathId> leaving,
                     const std::vector<Vec2>& ends) {
    for (uint32_t p = 0; p < g.paths.size(); ++p) {
        std::sort(next[p].begin(), next[p].end());
        g.paths[p].next = Range{static_cast<uint32_t>(g.nextPaths.size()), static_cast<uint32_t>(next[p].size())};
        g.nextPaths.insert(g.nextPaths.end(), next[p].begin(), next[p].end());
    }
    Junction jn;
    jn.connectors = Range{0, static_cast<uint32_t>(g.junctionConnectors.size())};
    jn.box = g.addPlanPoints(convexHull(ends));
    std::sort(arriving.begin(), arriving.end());
    std::sort(leaving.begin(), leaving.end());
    jn.arriving = Range{static_cast<uint32_t>(g.junctionLanes.size()), static_cast<uint32_t>(arriving.size())};
    g.junctionLanes.insert(g.junctionLanes.end(), arriving.begin(), arriving.end());
    jn.leaving = Range{static_cast<uint32_t>(g.junctionLanes.size()), static_cast<uint32_t>(leaving.size())};
    g.junctionLanes.insert(g.junctionLanes.end(), leaving.begin(), leaving.end());
    g.junctions.push_back(jn);
}

// A HAND-MADE JUNCTION AS A LANE GRAPH: handJunction's legs, each lane kHandLaneLength long straight out from the
// junction, a section per leg (offside to kerb), a connector for every turn the road model would state there (handTurns:
// lane_turns.h; connectorShape's, as the builder makes them), the next lists, and the one junction, its outline round the
// lane ends. `pathOf` maps a leg's lane id (road * 100 + k, + 50 leaving) to its path.
struct HandGraph {
    LaneGraph g;
    std::map<uint32_t, uint32_t> pathOf;
    std::map<uint32_t, Turn> turnOf;   // each connector's turn, as handTurns stated it (the table holds no turn kind)
};
HandGraph handGraph(const std::vector<double>& bearings, const std::vector<uint32_t>& lanes, double kerbPerp = -1,
                    bool mirrored = false) {
    std::vector<HandLeg> arriving, leaving;
    handJunction(bearings, lanes, kerbPerp, mirrored, arriving, leaving);
    HandGraph h;
    LaneGraph& g = h.g = emptyTestGraph();
    std::vector<std::vector<PathId>> next;
    std::vector<PathId> in, out;
    std::vector<Vec2> ends;
    for (int side = 0; side < 2; ++side)
        for (const HandLeg& leg : side == 0 ? arriving : leaving) {
            Section sec;
            sec.lanes.first = static_cast<uint32_t>(g.sectionLanes.size());
            sec.lines.first = static_cast<uint32_t>(g.laneLines.size());
            for (std::size_t k = 0; k < leg.lanes.size(); ++k) {
                const Vec2 e = leg.ends[k];
                const uint32_t p = side == 0 ? addTestPath(g, next, PathKind::Lane, {e - leg.heading * kHandLaneLength, e}, {0, 0})
                                             : addTestPath(g, next, PathKind::Lane, {e, e + leg.heading * kHandLaneLength}, {0, 0});
                g.paths[p].section = SectionId(static_cast<uint32_t>(g.sections.size()));
                g.paths[p].indexInSection = static_cast<uint32_t>(k);
                g.sectionLanes.push_back(PathId(p));
                if (k > 0) g.laneLines.push_back(LaneLine());
                h.pathOf[leg.lanes[k]] = p;
                (side == 0 ? in : out).push_back(PathId(p));
                ends.push_back(e);
            }
            sec.lanes.count = static_cast<uint32_t>(g.sectionLanes.size()) - sec.lanes.first;
            sec.lines.count = static_cast<uint32_t>(g.laneLines.size()) - sec.lines.first;
            g.sections.push_back(sec);
        }
    for (const StatedTurn& t : handTurns(bearings, lanes, kerbPerp, mirrored)) {
        const PathId a(h.pathOf[t.from]), b(h.pathOf[t.to]);
        const ConnectorShape s = connectorShape(g.pointAt(a, g.paths[a.index].length), g.directionAt(a, g.paths[a.index].length),
                                                g.pointAt(b, 0), g.directionAt(b, 0));
        CHECK(s.ok);
        const uint32_t c = addTestPath(g, next, PathKind::Connector, s.points, std::vector<Real>(s.points.size(), 0));
        h.turnOf[c] = t.turn;
        g.paths[c].tightestRadius = s.tightestRadius;
        g.paths[c].junction = JunctionId(0);
        g.paths[c].from = a;
        next[a.index].push_back(PathId(c));
        next[c].push_back(b);
        g.junctionConnectors.push_back(PathId(c));
    }
    finishTestGraph(g, next, in, out, ends);
    return h;
}

// A MINIATURE JUNCTION OF STRAIGHT CONNECTORS, for the cases whose answer is a formula: each turn a straight connector
// from `from` to `to` at height z. Turns that start at one point share the lane into it (kMiniLaneLength straight
// behind the first such turn), turns that end at one point share the lane out of it (as far straight on).
struct MiniTurn {
    Vec2 from, to;
    double z = 0;
};
LaneGraph miniJunction(const std::vector<MiniTurn>& turns) {
    LaneGraph g = emptyTestGraph();
    std::vector<std::vector<PathId>> next;
    std::vector<Vec2> starts, finishes, ends;
    std::vector<uint32_t> laneIn, laneOut;
    auto same = [](const Vec2& a, const Vec2& b) { return a.x == b.x && a.y == b.y; };
    for (const MiniTurn& t : turns) {
        const Vec2 d = normalize(t.to - t.from);
        if (std::find_if(starts.begin(), starts.end(), [&](const Vec2& s) { return same(s, t.from); }) == starts.end()) {
            starts.push_back(t.from);
            laneIn.push_back(addTestPath(g, next, PathKind::Lane, {t.from - d * kMiniLaneLength, t.from}, {t.z, t.z}));
            ownSection(g, laneIn.back());
            ends.push_back(t.from);
        }
        if (std::find_if(finishes.begin(), finishes.end(), [&](const Vec2& s) { return same(s, t.to); }) == finishes.end()) {
            finishes.push_back(t.to);
            laneOut.push_back(addTestPath(g, next, PathKind::Lane, {t.to, t.to + d * kMiniLaneLength}, {t.z, t.z}));
            ownSection(g, laneOut.back());
            ends.push_back(t.to);
        }
    }
    std::vector<PathId> in, out;
    for (uint32_t p : laneIn) in.push_back(PathId(p));
    for (uint32_t p : laneOut) out.push_back(PathId(p));
    for (const MiniTurn& t : turns) {
        const std::size_t si = static_cast<std::size_t>(std::find_if(starts.begin(), starts.end(), [&](const Vec2& s) { return same(s, t.from); }) - starts.begin());
        const std::size_t fi = static_cast<std::size_t>(std::find_if(finishes.begin(), finishes.end(), [&](const Vec2& s) { return same(s, t.to); }) - finishes.begin());
        const uint32_t c = addTestPath(g, next, PathKind::Connector, {t.from, t.to}, {t.z, t.z});
        g.paths[c].junction = JunctionId(0);
        g.paths[c].from = PathId(laneIn[si]);
        next[laneIn[si]].push_back(PathId(c));
        next[c].push_back(PathId(laneOut[fi]));
        g.junctionConnectors.push_back(PathId(c));
    }
    finishTestGraph(g, next, in, out, ends);
    return g;
}

int countMeeting(const LaneGraph& g, Meeting m) {
    int n = 0;
    for (const CrossingPoint& x : g.crossings) n += x.otherSide == OtherSide::Path && x.meeting == m ? 1 : 0;
    return n;
}
// The one record of paths a and b (a < b), or null when there is not exactly one.
const CrossingPoint* recordOf(const LaneGraph& g, uint32_t a, uint32_t b) {
    const CrossingPoint* found = nullptr;
    int n = 0;
    for (const CrossingPoint& x : g.crossings)
        if (x.otherSide == OtherSide::Path && x.a.index == a && x.other == b) {
            found = &x;
            ++n;
        }
    return n == 1 ? found : nullptr;
}
// A stretch end found by sampling against the true one: never short of it (it must cover where bodies could touch, to
// rounding), and no more than two samples beyond it (one for the sampling, one for the widening).
bool coversFrom(double found, double truth) { return found <= truth + 0.01 && found >= truth - 2 * kCrossingStep; }
bool coversTo(double found, double truth) { return found >= truth - 0.01 && found <= truth + 2 * kCrossingStep; }

// HOW MANY PAIRS OF CONNECTORS OF ONE JUNCTION CROSS, centre line through centre line at one level, where no record of
// the pair holds the crossing on both (design 5.3: every crossing between two connectors at the same level is in the
// table). Prints the first few.
int uncoveredCrossings(const LaneGraph& G, double clearance) {
    int missing = 0;
    for (const Junction& jn : G.junctions)
        for (uint32_t i = jn.connectors.first; i < jn.connectors.end(); ++i)
            for (uint32_t j = i + 1; j < jn.connectors.end(); ++j) {
                const uint32_t a = G.junctionConnectors[i].index, b = G.junctionConnectors[j].index;
                const Range ra = G.paths[a].curve, rb = G.paths[b].curve;
                for (uint32_t k = ra.first + 1; k < ra.end(); ++k)
                    for (uint32_t m = rb.first + 1; m < rb.end(); ++m) {
                        Vec2 at;
                        if (!segmentsCross(G.points[k - 1], G.points[k], G.points[m - 1], G.points[m], at)) continue;
                        const double sa = G.along[k - 1] + distance(G.points[k - 1], at), sb = G.along[m - 1] + distance(G.points[m - 1], at);
                        if (std::fabs(G.heightAt(PathId(a), sa) - G.heightAt(PathId(b), sb)) > clearance) continue;
                        bool held = false;
                        for (uint32_t q = G.paths[a].crossings.first; q < G.paths[a].crossings.end() && !held; ++q) {
                            const CrossingPoint& x = G.crossings[G.pathCrossings[q].index];
                            const bool aFirst = x.a.index == a;
                            if (x.otherSide != OtherSide::Path || (aFirst ? x.other : x.a.index) != b) continue;
                            const double af = aFirst ? x.aFrom : x.otherFrom, at2 = aFirst ? x.aTo : x.otherTo;
                            const double bf = aFirst ? x.otherFrom : x.aFrom, bt = aFirst ? x.otherTo : x.aTo;
                            held = sa >= af && sa <= at2 && sb >= bf && sb <= bt;
                        }
                        if (!held && missing++ < 4)
                            std::printf("      connectors %u and %u cross at (%.2f, %.2f), %.2f and %.2f m along: no record holds it\n", a, b,
                                        at.x, at.y, sa, sb);
                    }
            }
    return missing;
}
// Pairs of connectors leaving one lane (and entering one lane): the splits (and joins) there must be.
void sharedEndPairs(const LaneGraph& G, int& splits, int& joins) {
    std::vector<int> leaving(G.paths.size(), 0), entering(G.paths.size(), 0);
    for (const Path& p : G.paths)
        if (p.kind == PathKind::Connector) {
            ++leaving[p.from.index];
            ++entering[G.nextPaths[p.next.first].index];
        }
    splits = joins = 0;
    for (std::size_t p = 0; p < G.paths.size(); ++p) {
        splits += leaving[p] * (leaving[p] - 1) / 2;
        joins += entering[p] * (entering[p] - 1) / 2;
    }
}
}  // namespace

// THE TEXTBOOK FOUR-WAY: one lane each way, every turn allowed (twelve connectors). The textbook counts 16 places where
// paths cross, 8 where they merge and 8 where they part -- PLACES: the three paths leaving one lane part at two places.
// The table holds PAIRS, as the junction plan needs (who gives way to whom is a question about two paths): every two of
// the three paths leaving a lane split, 3 pairs a lane, 12; likewise 12 joins; and the 16 crosses, each its own pair.
TEST_CASE(lanes_crossing_points_four_way_one_lane_each_way) {
    HandGraph h = handGraph({0, 90, 180, 270}, {1, 1, 1, 1});
    const CrossingReport rep = findCrossingPoints(h.g, crossingTestOptions());
    const std::vector<std::string> faults = validate(h.g);
    for (const std::string& f : faults) std::printf("      fault: %s\n", f.c_str());
    CHECK(faults.empty());
    std::printf("    four-way: %d crosses, %d joins, %d splits\n", countMeeting(h.g, Meeting::Cross), countMeeting(h.g, Meeting::Join),
                countMeeting(h.g, Meeting::Split));
    CHECK(countMeeting(h.g, Meeting::Cross) == 16);
    CHECK(countMeeting(h.g, Meeting::Join) == 12);
    CHECK(countMeeting(h.g, Meeting::Split) == 12);
    CHECK(rep.byMeeting[static_cast<int>(Meeting::Cross)] == 16 && rep.withLanes == 0 && rep.betweenJunctions == 0);
    CHECK(rep.problems.empty());
    CHECK(h.g.junctions[0].crossings.count == 40);
    // which cross: a near turn nothing; a straight on the two straights across it and the two far turns across it; a far
    // turn the two straights across its way and the far turns from the roads either side (not the one opposite: the two
    // pass clear of each other)
    for (uint32_t c = h.g.junctionConnectors.front().index; c < h.g.paths.size(); ++c) {
        int crosses = 0;
        for (uint32_t k = h.g.paths[c].crossings.first; k < h.g.paths[c].crossings.end(); ++k)
            crosses += h.g.crossings[h.g.pathCrossings[k].index].meeting == Meeting::Cross ? 1 : 0;
        CHECK(crosses == (h.turnOf.at(c) == Turn::NearTurn ? 0 : 4));
    }
    CHECK(uncoveredCrossings(h.g, crossingTestOptions().clearance) == 0);
}

// THE TEXTBOOK T: one lane each way: 3 crosses, 3 joins, 3 splits (two paths leave each lane, two enter each).
TEST_CASE(lanes_crossing_points_t_junction_one_lane_each_way) {
    HandGraph h = handGraph({0, 180, 270}, {1, 1, 1});
    findCrossingPoints(h.g, crossingTestOptions());
    CHECK(validate(h.g).empty());
    std::printf("    T: %d crosses, %d joins, %d splits\n", countMeeting(h.g, Meeting::Cross), countMeeting(h.g, Meeting::Join),
                countMeeting(h.g, Meeting::Split));
    CHECK(countMeeting(h.g, Meeting::Cross) == 3);
    CHECK(countMeeting(h.g, Meeting::Join) == 3);
    CHECK(countMeeting(h.g, Meeting::Split) == 3);
    CHECK(uncoveredCrossings(h.g, crossingTestOptions().clearance) == 0);
}

// A PLAIN CROSS: two straight paths at right angles, 20 m long, crossing at their middles. Bodies touch where the centre
// lines are closer than 3.1 m: 3.1 m either side of the middle on each.
TEST_CASE(lanes_crossing_points_plain_cross_by_geometry) {
    LaneGraph g = miniJunction({{Vec2(-10, 0), Vec2(10, 0)}, {Vec2(0, -10), Vec2(0, 10)}});
    const CrossingReport rep = findCrossingPoints(g, crossingTestOptions());
    CHECK(validate(g).empty());
    CHECK(g.crossings.size() == 1);   // and nothing between a connector and the lanes before and after it
    const uint32_t a = g.junctionConnectors[0].index, b = g.junctionConnectors[1].index;
    const CrossingPoint* x = recordOf(g, a, b);
    CHECK(x != nullptr);
    if (!x) return;
    CHECK(x->meeting == Meeting::Cross && x->junction == JunctionId(0));
    const double r = testReach();
    std::printf("    plain cross: on the one %.2f to %.2f m, on the other %.2f to %.2f m (bodies touch from %.2f to %.2f)\n", x->aFrom,
                x->aTo, x->otherFrom, x->otherTo, 10 - r, 10 + r);
    CHECK(coversFrom(x->aFrom, 10 - r) && coversTo(x->aTo, 10 + r));
    CHECK(coversFrom(x->otherFrom, 10 - r) && coversTo(x->otherTo, 10 + r));
    CHECK(rep.problems.empty());
}

// A SKEWED CROSSING: the other path at 30 degrees. A body at distance d along from the crossing is d sin 30 from the
// other centre line, so they touch for 3.1 / sin 30 = 6.2 m either side: a long stretch, and right (the brief: don't
// shorten it). And a skewed four-way built as the builder builds one: sound, every crossing of centre lines in the table.
TEST_CASE(lanes_crossing_points_skewed_crossing_is_long) {
    const double s30 = std::sin(30 * kHandDegrees), c30 = std::cos(30 * kHandDegrees);
    LaneGraph g = miniJunction({{Vec2(-10, 0), Vec2(10, 0)}, {Vec2(-10 * c30, -10 * s30), Vec2(10 * c30, 10 * s30)}});
    findCrossingPoints(g, crossingTestOptions());
    CHECK(validate(g).empty());
    const CrossingPoint* x = recordOf(g, g.junctionConnectors[0].index, g.junctionConnectors[1].index);
    CHECK(x != nullptr);
    if (!x) return;
    const double half = testReach() / s30;
    std::printf("    skewed at 30 degrees: %.2f to %.2f m on the one (bodies touch from %.2f to %.2f)\n", x->aFrom, x->aTo, 10 - half, 10 + half);
    CHECK(coversFrom(x->aFrom, 10 - half) && coversTo(x->aTo, 10 + half));
    CHECK(coversFrom(x->otherFrom, 10 - half) && coversTo(x->otherTo, 10 + half));

    HandGraph h = handGraph({0, 60, 180, 240}, {2, 2, 2, 2});
    const CrossingReport rep = findCrossingPoints(h.g, crossingTestOptions());
    CHECK(validate(h.g).empty());
    std::printf("    skewed four-way, two lanes each way: %d crosses, %d joins, %d splits\n", countMeeting(h.g, Meeting::Cross),
                countMeeting(h.g, Meeting::Join), countMeeting(h.g, Meeting::Split));
    CHECK(uncoveredCrossings(h.g, crossingTestOptions().clearance) == 0);
    int splits = 0, joins = 0;
    sharedEndPairs(h.g, splits, joins);
    CHECK(countMeeting(h.g, Meeting::Split) == splits && countMeeting(h.g, Meeting::Join) == joins);
    // two turn lanes side by side (none here: two lanes each way turn from one lane each) never touch all the way round
    CHECK(rep.problems.empty());
}

// TWO TURNS INTO ONE LANE, and ONE LANE SPLITTING TWO WAYS: two straight paths 10 m long meeting at one end at 40 degrees.
// A body u from the shared end is u sin 40 from the other centre line: they touch from 3.1 / sin 40 = 4.82 m before the
// end -- to the very end of both (a join), or from the very start to 4.82 m along (a split).
TEST_CASE(lanes_crossing_points_join_and_split_by_geometry) {
    const double s20 = std::sin(20 * kHandDegrees), c20 = std::cos(20 * kHandDegrees);
    const double reach = testReach() / std::sin(40 * kHandDegrees);
    const Vec2 o(0, 0), p(-10 * c20, 10 * s20), q(-10 * c20, -10 * s20);
    {
        LaneGraph g = miniJunction({{p, o}, {q, o}});
        findCrossingPoints(g, crossingTestOptions());
        CHECK(validate(g).empty());
        CHECK(g.crossings.size() == 1);
        const CrossingPoint* x = recordOf(g, g.junctionConnectors[0].index, g.junctionConnectors[1].index);
        CHECK(x != nullptr);
        if (!x) return;
        std::printf("    join: %.2f to %.2f m and %.2f to %.2f m (bodies touch from %.2f m)\n", x->aFrom, x->aTo, x->otherFrom, x->otherTo, 10 - reach);
        CHECK(x->meeting == Meeting::Join);
        CHECK(x->aTo == g.paths[x->a.index].length && x->otherTo == g.paths[x->other].length);   // to the end of both, exactly
        CHECK(coversFrom(x->aFrom, 10 - reach) && coversFrom(x->otherFrom, 10 - reach));
    }
    {
        LaneGraph g = miniJunction({{o, p * -1.0}, {o, q * -1.0}});
        findCrossingPoints(g, crossingTestOptions());
        CHECK(validate(g).empty());
        CHECK(g.crossings.size() == 1);
        const CrossingPoint* x = recordOf(g, g.junctionConnectors[0].index, g.junctionConnectors[1].index);
        CHECK(x != nullptr);
        if (!x) return;
        std::printf("    split: %.2f to %.2f m and %.2f to %.2f m (bodies clear at %.2f m)\n", x->aFrom, x->aTo, x->otherFrom, x->otherTo, reach);
        CHECK(x->meeting == Meeting::Split);
        CHECK(x->aFrom == 0 && x->otherFrom == 0);
        CHECK(coversTo(x->aTo, reach) && coversTo(x->otherTo, reach));
    }
}

// A BRIDGE OVER A STREET is not a crossing: the plain cross with one path 6 m up gives none; 0.8 m up (within the road
// builder's same-level figure, 1 m) it is still a crossing.
TEST_CASE(lanes_crossing_points_none_between_levels) {
    LaneGraph bridge = miniJunction({{Vec2(-10, 0), Vec2(10, 0), 0.0}, {Vec2(0, -10), Vec2(0, 10), 6.0}});
    findCrossingPoints(bridge, crossingTestOptions());
    CHECK(validate(bridge).empty());
    CHECK(bridge.crossings.empty());
    LaneGraph step = miniJunction({{Vec2(-10, 0), Vec2(10, 0), 0.0}, {Vec2(0, -10), Vec2(0, 10), 0.8}});
    findCrossingPoints(step, crossingTestOptions());
    CHECK(step.crossings.size() == 1);
}

// A CROSSWALK over the east road of the four-way, 3 m wide and 10 m long across both lanes: one record for each path a
// body on which could be on it -- the lane in and the lane out, and no connector (they start 2 m from it, beyond a car's
// half width and the margin) -- each with the stretch of the walk the body covers.
TEST_CASE(lanes_crossing_points_a_crosswalk_gives_one_record_per_path_over_it) {
    HandGraph h = handGraph({0, 90, 180, 270}, {1, 1, 1, 1});
    Crosswalk walk;
    walk.outline = h.g.addPlanPoints({Vec2(12, -5), Vec2(15, -5), Vec2(15, 5), Vec2(12, 5)});
    walk.junction = JunctionId(0);
    h.g.crosswalks.push_back(walk);
    h.g.junctions[0].crosswalks = Range{0, 1};
    const CrossingReport rep = findCrossingPoints(h.g, crossingTestOptions());
    const std::vector<std::string> faults = validate(h.g);
    for (const std::string& f : faults) std::printf("      fault: %s\n", f.c_str());
    CHECK(faults.empty());
    CHECK(rep.withCrosswalks == 2);
    const double half = kTestLongWidth / 2 + kCrossingMargin;
    for (const CrossingPoint& x : h.g.crossings) {
        if (x.otherSide != OtherSide::Crosswalk) continue;
        const uint32_t laneIn = h.pathOf[0], laneOut = h.pathOf[50];
        CHECK(x.a.index == laneIn || x.a.index == laneOut);
        CHECK(x.junction == JunctionId(0) && x.other == 0);
        const Vec2 mid = h.g.pointAt(x.a, (x.aFrom + x.aTo) / 2);
        // along the walk from its first edge (y = -5); the lane's centre line is at mid.y
        CHECK(std::fabs(x.otherFrom - (mid.y + 5 - half)) < 1e-6 && std::fabs(x.otherTo - (mid.y + 5 + half)) < 1e-6);
        // on the lane: where it runs from x = 12 - half to 15 + half
        const double sNear = x.a.index == laneIn ? 40 - (15 + half) : (12 - half) - 10;
        const double sFar = x.a.index == laneIn ? 40 - (12 - half) : (15 + half) - 10;
        std::printf("    crosswalk: %s %.2f to %.2f m (body on it from %.2f to %.2f), the walk %.2f to %.2f m\n",
                    x.a.index == laneIn ? "the lane in" : "the lane out", x.aFrom, x.aTo, sNear, sFar, x.otherFrom, x.otherTo);
        CHECK(coversFrom(x.aFrom, sNear) && coversTo(x.aTo, sFar));
    }
}

// MIRRORED, THE MIRRORED TABLE: a junction and its mirror image (traffic keeping to the other side) give the same turns,
// so the same paths in the same order -- and the same crossing points, record for record.
TEST_CASE(lanes_crossing_points_of_a_mirrored_junction_are_mirrored) {
    for (const std::vector<double>& bearings : {std::vector<double>{0, 90, 180, 270}, std::vector<double>{0, 60, 180, 240},
                                                std::vector<double>{0, 90, 160, 215, 270}, std::vector<double>{0, 180, 270}}) {
        const std::vector<uint32_t> lanes(bearings.size(), 2);
        HandGraph a = handGraph(bearings, lanes, -1, false), b = handGraph(bearings, lanes, 1, true);
        findCrossingPoints(a.g, crossingTestOptions());
        findCrossingPoints(b.g, crossingTestOptions());
        CHECK(validate(a.g).empty() && validate(b.g).empty());
        CHECK(a.g.crossings.size() == b.g.crossings.size() && !a.g.crossings.empty());
        int differ = 0;
        for (std::size_t i = 0; i < a.g.crossings.size() && i < b.g.crossings.size(); ++i) {
            const CrossingPoint& x = a.g.crossings[i];
            const CrossingPoint& y = b.g.crossings[i];
            const bool same = x.a == y.a && x.other == y.other && x.meeting == y.meeting && x.junction == y.junction &&
                              std::fabs(x.aFrom - y.aFrom) <= kCrossingStep + 1e-9 && std::fabs(x.aTo - y.aTo) <= kCrossingStep + 1e-9 &&
                              std::fabs(x.otherFrom - y.otherFrom) <= kCrossingStep + 1e-9 &&
                              std::fabs(x.otherTo - y.otherTo) <= kCrossingStep + 1e-9;
            if (!same && differ++ < 4) std::printf("      record %zu differs in the mirror\n", i);
        }
        CHECK(differ == 0);
        // and in either, two turn lanes side by side are not touching all the way round
        CHECK(uncoveredCrossings(a.g, crossingTestOptions().clearance) == 0);
    }
}

// ---- the scenes' and the lab's crossing points --------------------------------------------------------------------------
namespace {
// THE CHECKS EVERY SCENE'S CROSSING POINTS MUST PASS: every crossing of two connectors' centre lines at one level in the
// table; a split for every two connectors leaving one lane and a join for every two entering one; every junction with
// connectors that share a lane has records; the same table when found again.
void checkCrossings(const std::string& name) {
    const Result& r = laneSceneOf(name);
    const LaneGraphBuild& lg = laneGraphOf(name);
    const LaneGraph& G = modelOf(name).laneLevel;
    const CrossingReport& cr = lg.crossings;
    std::printf("    [crossing points] %s: %zu (%u crosses, %u joins, %u splits; %u reaching across junctions, %u with a lane); "
                "busiest junction %u with %u; %u junctions of two or more connectors with none; %zu problems; %.0f ms\n",
                name.c_str(), G.crossings.size(), cr.byMeeting[0], cr.byMeeting[1], cr.byMeeting[2], cr.betweenJunctions,
                cr.withLanes, cr.busiestJunction, cr.busiestCount, cr.junctionsWithout, cr.problems.size(), cr.milliseconds);
    for (std::size_t i = 0; i < cr.problems.size() && i < 4; ++i) std::printf("      %s\n", problemLine(cr.problems[i]).c_str());
    CHECK(validate(G).empty());
    CHECK(uncoveredCrossings(G, r.graph.rules.sameLevelDz) == 0);
    int splits = 0, joins = 0, foundSplits = 0, foundJoins = 0;
    sharedEndPairs(G, splits, joins);
    for (const CrossingPoint& x : G.crossings) {
        if (x.otherSide != OtherSide::Path || G.paths[x.a.index].kind != PathKind::Connector ||
            G.paths[x.other].kind != PathKind::Connector)
            continue;
        foundSplits += x.meeting == Meeting::Split ? 1 : 0;
        foundJoins += x.meeting == Meeting::Join ? 1 : 0;
    }
    CHECK(foundSplits == splits);
    CHECK(foundJoins == joins);
    // a junction with none: its connectors each leave a lane of their own and enter a lane of their own, and pass clear of
    // one another (a stated junction of two arms carrying on, its lanes side by side; a corner where one road bends into
    // another, its near turn inside its far turn)
    int without = 0, carryOns = 0;
    for (uint32_t j = 0; j < G.junctions.size(); ++j) {
        const Junction& jn = G.junctions[j];
        if (jn.crossings.count > 0 || jn.connectors.count < 2) continue;
        ++without;
        carryOns += r.graph.junctions[jn.modelJunction].arms.size() == 2 ? 1 : 0;
        std::vector<uint32_t> froms, intos;
        for (uint32_t k = jn.connectors.first; k < jn.connectors.end(); ++k) {
            const Path& c = G.paths[G.junctionConnectors[k].index];
            froms.push_back(c.from.index);
            intos.push_back(G.nextPaths[c.next.first].index);
        }
        std::sort(froms.begin(), froms.end());
        std::sort(intos.begin(), intos.end());
        CHECK(std::adjacent_find(froms.begin(), froms.end()) == froms.end());
        CHECK(std::adjacent_find(intos.begin(), intos.end()) == intos.end());
    }
    std::printf("      %d junctions with two or more connectors and no crossing point, %d of them carry-ons\n", without, carryOns);
    // found again: the same table, byte for byte (the stated merges, which the lane level names on its Joins after they
    // are found, CrossingPoint::modelConnection, are not the crossings code's: left out of the comparison)
    LaneGraph found = G;
    for (CrossingPoint& x : found.crossings) x.modelConnection = kNoIndex;
    LaneGraph again = G;
    CrossingOptions co = crossingTestOptions();
    co.clearance = r.graph.rules.sameLevelDz;
    findCrossingPoints(again, co);
    bundle::BinWriter w1, w2;
    putLaneGraph(w1, found);
    putLaneGraph(w2, again);
    CHECK(w1.bytes == w2.bytes);
}
}  // namespace

TEST_CASE(lanes_crossing_points_of_grid_city) { checkCrossings("grid_city"); }
TEST_CASE(lanes_crossing_points_of_ring_city) { checkCrossings("ring_city"); }
TEST_CASE(lanes_crossing_points_of_hill_junction) { checkCrossings("hill_junction"); }
TEST_CASE(lanes_crossing_points_of_the_traffic_lab) { checkCrossings("planned/traffic_lab_lanelab"); }

// ---- LONG BODIES (review #284 item 9, ADR-0412: swept_body.h) ----------------------------------------------------------
#include "engine/procgen/city/roads/lanes/swept_body.h"

namespace {
// A quarter circle of radius `r` round the origin, from (r, 0) heading +y to (0, r) heading -x, a point every degree.
std::vector<Vec2> quarterArc(double r) {
    std::vector<Vec2> pts;
    for (int d = 0; d <= 90; ++d) pts.push_back(Vec2(r * std::cos(d * kHandDegrees), r * std::sin(d * kHandDegrees)));
    return pts;
}
// Two turns side by side, round the origin at radii `inner` and `outer`, each from a straight lane of its own into one
// of its own (kMiniLaneLength each), one junction.
LaneGraph twoTurnsSideBySide(double inner, double outer) {
    LaneGraph g = emptyTestGraph();
    std::vector<std::vector<PathId>> next;
    std::vector<PathId> in, out;
    std::vector<Vec2> ends;
    for (const double r : {inner, outer}) {
        const std::vector<Vec2> arc = quarterArc(r);
        const uint32_t a = addTestPath(g, next, PathKind::Lane, {Vec2(r, -kMiniLaneLength), arc.front()}, {0, 0});
        const uint32_t b = addTestPath(g, next, PathKind::Lane, {arc.back(), Vec2(-kMiniLaneLength, r)}, {0, 0});
        ownSection(g, a);
        ownSection(g, b);
        const uint32_t c = addTestPath(g, next, PathKind::Connector, arc, std::vector<Real>(arc.size(), 0));
        g.paths[c].junction = JunctionId(0);
        g.paths[c].from = PathId(a);
        g.paths[c].tightestRadius = r;
        next[a].push_back(PathId(c));
        next[c].push_back(PathId(b));
        g.junctionConnectors.push_back(PathId(c));
        in.push_back(PathId(a));
        out.push_back(PathId(b));
        ends.push_back(arc.front());
        ends.push_back(arc.back());
    }
    finishTestGraph(g, next, in, out, ends);
    return g;
}
}  // namespace

TEST_CASE(lanes_a_long_body_cuts_inside_a_bend_by_its_chord) {
    // The one body rule (lane_graph.h bodyOnPlan): bumpers on the line, the body on the chord. Round a 10 m circle a 12 m
    // body's middle lies R (1 - cos(L / 2R)) = 1.75 m inside the line; a body of no length, and any body on a straight,
    // sweeps the line's own point.
    const double r = 10, length = 12;
    const std::vector<Vec2> arc = quarterArc(r);
    const BodyLine line = makeBodyLine({Vec2(r, -kMiniLaneLength), arc.front()}, arc, {arc.back(), Vec2(-kMiniLaneLength, r)},
                                       length);
    const double middle = line.length / 2;
    std::vector<Vec2> spots;
    sweptPoints(line, middle, length, spots);
    double deepest = 0;
    for (const Vec2& q : spots) deepest = std::max(deepest, r - q.length());
    const double expected = r * (1 - std::cos(length / (2 * r)));
    std::printf("    a 12 m body round a 10 m bend: %zu swept points, the deepest %.3f m inside the line (the chord: %.3f m)\n",
                spots.size(), deepest, expected);
    CHECK(std::fabs(deepest - expected) < 0.02);
    const PlanRectangle body = bodyAt(line, middle + length / 2, length, 2.5);
    CHECK(std::fabs((r - body.middle.length()) - expected) < 0.02);
    CHECK(std::fabs(body.halfLength - length / 2) < 1e-12 && std::fabs(body.halfWidth - 1.25) < 1e-12);
    spots.clear();
    sweptPoints(line, middle, 0, spots);
    CHECK(spots.size() == 1u && std::fabs(spots[0].length() - r) < 0.01);
    spots.clear();
    sweptPoints(line, -kMiniLaneLength + 1, length, spots);   // well back on the straight lane: the line's own point
    CHECK(spots.size() == 1u);
}

TEST_CASE(lanes_crossing_points_reach_where_a_long_body_swings_beside_another_turn) {
    // Two turns side by side, 3.5 m apart round 10 m and 13.5 m: their centre lines never come within the long test
    // vehicle's touching distance (2.6 m wide, so 3.1 m), so swept as points they share no record. Swept at 12 m the outer
    // body's middle cuts about 1.3 m inside its line, toward the inner turn: the two turns get records, crosses, and on
    // the outer turn none starts at its start -- a body standing at its line, still straight on the lane behind, is clear;
    // only once its front has come round far enough does its middle reach the inner turn. (The inner turn's bodies swing
    // away from the outer, so they touch the outer bus near their own ends: two meetings, one near each.) With the car
    // tag 4.5 m long (its swing within the margin: short) every record concerns the long tag on at least one side, and
    // none concerns two cars: their own bodies never touch here.
    LaneGraph points = twoTurnsSideBySide(10, 13.5), swept = points;
    CrossingOptions o = crossingTestOptions();
    CHECK(findCrossingPoints(points, o).byMeeting[static_cast<int>(Meeting::Cross)] == 0u);
    for (VehicleSize& v : o.vehicles) v.length = v.tag == kLaneGraphLongTag ? 12 : 4.5;
    const CrossingReport rep = findCrossingPoints(swept, o);
    const TagMask carTag = TagMask(1) << LaneGraph::nameIndex(swept.vehicleTagNames, kLaneGraphTestTag);
    const uint32_t innerTurn = 2, outerTurn = 5;   // (addTestPath order: lane, lane, connector for each radius)
    CHECK(swept.paths[innerTurn].kind == PathKind::Connector && swept.paths[outerTurn].kind == PathKind::Connector);
    int between = 0;
    for (uint32_t k = 0; k < swept.crossings.size(); ++k) {
        const CrossingPoint& x = swept.crossings[k];
        if (x.otherSide != OtherSide::Path || x.a.index != innerTurn || x.other != outerTurn) continue;
        ++between;
        std::printf("    %s", dumpCrossing(swept, CrossingId(k)).c_str());
        CHECK(x.meeting == Meeting::Cross);
        CHECK(x.otherFrom > 0 && x.otherTo <= swept.paths[outerTurn].length);
        CHECK(x.aFrom >= 0 && x.aTo > x.aFrom && x.aTo <= swept.paths[innerTurn].length);
        CHECK(!(x.concerns(x.a, carTag) && x.concerns(x.otherPath(), carTag)));   // never two cars
    }
    std::printf("    two turns 3.5 m apart, a 12 m body: %d records between them; %u found only for length, %u made longer\n",
                between, rep.onlyForLength, rep.widenedForLength);
    CHECK(between >= 1);
    CHECK(rep.onlyForLength >= 1u);
    CHECK(validate(swept).empty());
}

// ---- a ramp's end on a street: on the carriageway going its way (#282, ADR-0411) ------------------------------------
#include "engine/procgen/city/roads/lanes/interchange.h"
namespace {
// ONE STREET, ONE RAMP: a two-way collector along x (its forward lanes run +x, on the spine's right in plan: -y) and a
// ramp that ends on it, by the end given (an anchor object, or the street's bare id) along the path given.
nlohmann::json oneRampScene(const nlohmann::json& to, const std::vector<std::array<double, 2>>& path, const nlohmann::json& landing,
                            const nlohmann::json& floor = nullptr) {
    nlohmann::json g;
    g["name"] = "one_ramp";
    nlohmann::json pts = nlohmann::json::array();
    for (const std::array<double, 2>& p : path) pts.push_back({p[0], p[1]});
    nlohmann::json ramp = {{"id", "r"}, {"class", "ramp"}, {"to", to}, {"path", {{"type", "polyline"}, {"points", pts}}}};
    if (!landing.is_null()) ramp["landing"] = landing;
    if (!floor.is_null()) ramp["floor"] = floor;
    g["edges"] = nlohmann::json::array({{{"id", "st"}, {"class", "collector"}, {"path", {{"type", "polyline"}, {"points", {{0, 0}, {800, 0}}}}}}, ramp});
    return g;
}
// running +x alongside the street's forward (-y) side, from the south-west
const std::vector<std::array<double, 2>> kAlongsideSouth = {{40, -60}, {100, -40}, {160, -22}, {200, -14}, {240, -10}};
nlohmann::json kerbAnchor(const char* side) { return {{"edge", "st"}, {"at", {250, 0}}, {"side", side}, {"aux", 30}, {"taper", 20}}; }
bool refuses(const nlohmann::json& g, const char* naming) {
    try { (void)build(RoadLabGraph::fromJson(g)); } catch (const std::runtime_error& e) {
        std::printf("    refused: %s\n", e.what());
        return std::string(e.what()).find(naming) != std::string::npos && std::string(e.what()).find("ramp r:") != std::string::npos;
    }
    return false;
}
}  // namespace

// A RAMP THAT JOINS THE CARRIAGEWAY GOING ITS WAY is built and passes: anchored beside the kerb lane of the street's
// forward lanes, arriving the way they run, its landing planned so.
TEST_CASE(lanes_a_ramp_merging_into_the_carriageway_going_its_way_is_built) {
    const nlohmann::json landing = {{"street", "st"}, {"joins", "merge"}, {"carriageway", "forward"}, {"half", "near"}, {"crossing", "none"}};
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(oneRampScene(kerbAnchor("kerb"), kAlongsideSouth, landing)));
    CHECK(checkRampLandings(r->graph).empty());
    CHECK(r->rampLandings.empty());
    CHECK(r->graph.find("r")->anchorLanes.at("to") == "st.f1");   // the forward lanes' kerb lane
    checkInvariants(*r);
}

// THE BUILDER REFUSES A RAMP THAT LANDS AGAINST THE TRAFFIC, naming it: the same ramp anchored beside the street's
// backward lanes (which run -x, against it), and one ending along the street's centre line (the traffic lab's
// touch-downs before #282). Planned, each is refused; planned before the rule (no "landing"), each only warns.
TEST_CASE(lanes_the_builder_refuses_a_ramp_landing_on_the_carriageway_coming_the_other_way) {
    const nlohmann::json against = {{"street", "st"}, {"joins", "merge"}, {"carriageway", "backward"}, {"half", "far"}, {"crossing", "none"}};
    CHECK(refuses(oneRampScene(kerbAnchor("back-kerb"), kAlongsideSouth, against), "runs against it"));
    // ...and it is the geometry that is judged, not the words: planned for the forward lanes but anchored beside the other
    const nlohmann::json forwardWords = {{"street", "st"}, {"joins", "merge"}, {"carriageway", "forward"}, {"half", "near"}, {"crossing", "none"}};
    CHECK(refuses(oneRampScene(kerbAnchor("back-kerb"), kAlongsideSouth, forwardWords), "anchored beside its backward lanes"));
    // along the centre line, by the street's bare id: the old touch-down
    const std::vector<std::array<double, 2>> onCentre = {{40, -60}, {100, -40}, {160, -15}, {200, -3}, {230, 0}, {260, 0}};
    CHECK(refuses(oneRampScene("st", onCentre, {{"street", "st"}, {"joins", "junction"}}), "centre line"));
    // not planned: built, and warned
    const std::unique_ptr<Result> old = build(RoadLabGraph::fromJson(oneRampScene("st", onCentre, nullptr)));
    CHECK(old->rampLandings.size() == 1);
    if (!old->rampLandings.empty()) {
        std::printf("    warned: %s\n", describe(old->rampLandings[0]).c_str());
        CHECK(old->rampLandings[0].ramp == "r" && !old->rampLandings[0].declared);
    }
    // a bare end meeting the street at an angle is an arm of a junction, not judged here
    const std::vector<std::array<double, 2>> square = {{250, -150}, {250, -80}, {250, 0}};
    CHECK(checkRampLandings(RoadLabGraph::fromJson(oneRampScene("st", square, nullptr))).empty());
}

// A RAMP THAT CROSSES OVER THE STREET TO REACH THE FAR CARRIAGEWAY is built when it holds its planned clearance there
// (its floor point), and refused when it does not: coming from the north, over the street, down on the south side,
// into the forward lanes' kerb lane.
TEST_CASE(lanes_a_ramp_over_the_street_to_its_far_carriageway_is_checked_against_its_planned_clearance) {
    const std::vector<std::array<double, 2>> over = {{60, 200}, {150, 120}, {230, 30}, {260, 0}, {300, -40}, {380, -50}, {460, -30}, {520, -14}};
    const double clear = Rules().underClearance + std::max(roadClass("ramp").thick, roadClass("collector").thick) + Rules().structureDepth;
    const nlohmann::json landing = {{"street", "st"}, {"joins", "merge"}, {"carriageway", "forward"}, {"half", "far"},
                                    {"crossing", {{"kind", "over"}, {"street", "st"}, {"at", {260, 0}}, {"clearance", clear}}}};
    const nlohmann::json anchor = {{"edge", "st"}, {"at", {530, 0}}, {"side", "kerb"}, {"aux", 30}, {"taper", 20}};
    const nlohmann::json floor = nlohmann::json::array({{260, 0, clear, 25}});
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(oneRampScene(anchor, over, landing, floor)));
    const EdgeSpec& e = *r->graph.find("r");
    std::printf("    over the street: %.2f m (planned %.2f m)\n", projectZ(e, Vec2(260, 0)) - projectZ(*r->graph.find("st"), Vec2(260, 0)), clear);
    CHECK(checkRampLandings(r->graph).empty());
    checkInvariants(*r);
    // the same ramp with no floor point to hold it up: it crosses the street at grade, and is refused
    CHECK(refuses(oneRampScene(anchor, over, landing), "passes over st"));
}

// THE PLANNER LANDS EVERY RAMP ON THE CARRIAGEWAY GOING ITS WAY (#282): a straight freeway between two frontage roads
// 50 m off, crossed by one street. The band beside the deck meets the crossing street too near the frontage junction for
// a four-way, so each ramp touches down on a frontage road -- on its far carriageway (the near one runs the other way),
// crossing over the frontage road at the builder's clearance first. A side street meeting the far side where the
// ramp must come down leaves no room: the diamond is refused, never landed on the centre line.
namespace {
std::vector<RampStreet> frontageStreets(bool sideStreet) {
    std::vector<RampStreet> s = {
        {"x_s", {{1000, -250}, {1000, -50}}, false, "collector"}, {"x_m", {{1000, -50}, {1000, 50}}, false, "collector"},
        {"x_n", {{1000, 50}, {1000, 250}}, false, "collector"},
        {"f_s1", {{0, -50}, {1000, -50}}, false, "collector"}, {"f_s2", {{1000, -50}, {2000, -50}}, false, "collector"},
        {"f_n1", {{0, 50}, {1000, 50}}, false, "collector"}, {"f_n2", {{1000, 50}, {2000, 50}}, false, "collector"}};
    if (sideStreet) {   // a local street meeting the south frontage road from the far side, 100 m short of the crossing
        s[3].xy = {{0, -50}, {900, -50}};
        s.push_back({"f_s1b", {{900, -50}, {1000, -50}}, false, "collector"});
        s.push_back({"side", {{900, -50}, {900, -250}}, false, "local"});
    }
    return s;
}
DiamondResult frontageDiamond(const std::vector<RampStreet>& streets) {
    std::vector<Vec2> route;
    for (double x = 0; x <= 2000.0; x += 10.0) route.push_back(Vec2(x, 0));
    DiamondOptions o;
    o.solidBeforeStopLine = 20.0;   // this scene's lane rule: an off-ramp's merge lane ends 20 m before the stop line ahead
    o.aId = "fw_a"; o.bId = "fw_b";
    const RoadClassSpec& fw = roadClass("freeway");   // the carriageways below: the class table's, 7 m off the route
    o.carriage = 7.0;
    o.freewayLanes = fw.lanes.fwd;
    o.freewayLaneW = fw.lanes.w;
    o.edgeReach = o.carriage + fw.lanes.fwd * fw.lanes.w / 2 + fw.shoulder;
    o.rampHalf = 4.5 / 2 + 2.5;
    o.gRamp = roadClass("ramp").gMax;
    o.decel = 70; o.taperOff = 50; o.aux = 90; o.taperOn = 60;
    o.deck = [](double) { return 8.4; };
    return diamondRamps(route, streets, o);
}
}  // namespace

TEST_CASE(lanes_the_planner_lands_frontage_ramps_on_the_far_carriageway_over_the_road) {
    const DiamondResult dr = frontageDiamond(frontageStreets(false));
    CHECK(dr.built == 1);
    CHECK(dr.ramps.size() == 4);
    const double clear = Rules().underClearance + std::max(roadClass("ramp").thick, roadClass("collector").thick) + Rules().structureDepth;
    for (const nlohmann::json& e : dr.ramps) {
        const bool off = e["id"].get<std::string>().find("_off") != std::string::npos;
        const nlohmann::json& street = off ? e["to"] : e["from"];
        const nlohmann::json& L = e["landing"];
        std::printf("    %s: %s %s's %s carriageway (%s half), over %s at %.1f m; floor %s\n", e["id"].get<std::string>().c_str(),
                    L["joins"].get<std::string>().c_str(), L["street"].get<std::string>().c_str(), L["carriageway"].get<std::string>().c_str(),
                    L["half"].get<std::string>().c_str(), L["crossing"]["street"].get<std::string>().c_str(), L["crossing"]["clearance"].get<double>(),
                    e["floor"].dump().c_str());
        CHECK(street.is_object() && street["edge"] == L["street"]);   // anchored beside a lane, not the bare street
        CHECK(L["joins"] == "merge" && L["half"] == "far" && L["crossing"]["kind"] == "over");
        CHECK(L["crossing"]["clearance"].get<double>() >= clear - 1e-6);
        CHECK(e["floor"][0][2].get<double>() >= clear - 1e-6);
        // south frontage roads run +x along their spine: a's traffic (+x) takes their forward lanes; b's (-x) the north
        // roads' backward lanes
        const bool a = e["id"].get<std::string>().rfind("d0_0_a", 0) == 0;
        CHECK(street["side"] == (a ? "kerb" : "back-kerb"));
        CHECK(L["street"].get<std::string>().rfind(a ? "f_s" : "f_n", 0) == 0);
    }
    // as built: every ramp lands as planned (one street's worth of the scene is enough for the builder to check)
    nlohmann::json g;
    g["name"] = "frontage_diamond";
    g["edges"] = nlohmann::json::array();
    std::vector<Vec2> a, b;
    for (double x = 0; x <= 2000.0; x += 10.0) { a.push_back(Vec2(x, -7)); b.push_back(Vec2(2000 - x, 7)); }
    auto line = [](const std::vector<Vec2>& v) { nlohmann::json p = nlohmann::json::array(); for (const Vec2& q : v) p.push_back({q.x, q.y}); return p; };
    nlohmann::json floor = nlohmann::json::array();
    for (double x = 0; x <= 2000.0; x += 30.0) floor.push_back({x, 0.0, 8.4, 15.0});
    g["edges"].push_back({{"id", "fw_a"}, {"class", "freeway"}, {"floor", floor}, {"path", {{"type", "polyline"}, {"points", line(a)}}}});
    g["edges"].push_back({{"id", "fw_b"}, {"class", "freeway"}, {"floor", floor}, {"path", {{"type", "polyline"}, {"points", line(b)}}}});
    for (const RampStreet& s : frontageStreets(false))
        g["edges"].push_back({{"id", s.id}, {"class", s.cls}, {"path", {{"type", "polyline"}, {"points", line(s.xy)}}}});
    for (const nlohmann::json& e : dr.ramps) g["edges"].push_back(e);
    if (const char* outPath = std::getenv("RT_LANES_SCENE_OUT")) std::ofstream(outPath) << g.dump(1) << "\n";   // for lanes_tool to draw
    RoadLabGraph rg = RoadLabGraph::fromJson(g);
    (void)expand(rg);
    for (const RampLandingFault& f : checkRampLandings(rg)) std::printf("    fault: %s\n", describe(f).c_str());
    CHECK(checkRampLandings(rg).empty());
}

TEST_CASE(lanes_the_planner_refuses_a_diamond_with_no_room_to_land_rather_than_landing_on_the_centre_line) {
    const DiamondResult dr = frontageDiamond(frontageStreets(true));
    std::printf("    built %d, refused for want of a landing %d\n", dr.built, dr.rejectedLanding);
    CHECK(dr.built == 0);
    CHECK(dr.rejectedLanding >= 1);   // (the crossing, met at a vertex of the route, may be found twice)
    CHECK(dr.ramps.empty());
}

// ---- one-way frontage roads: a ramp joins the lane nearest the freeway at grade (ADR-0413) ---------------------------
namespace {
// oneRampScene's street made one-way: its two lanes both running +x (lanes centred: the offside lane, f0, on the +y side)
nlohmann::json oneWayRampScene(const nlohmann::json& to, const std::vector<std::array<double, 2>>& path, const nlohmann::json& landing) {
    nlohmann::json g = oneRampScene(to, path, landing);
    g["edges"][0]["lanes"] = {{"fwd", 2}, {"back", 0}};
    return g;
}
// running +x alongside the street's +y side, from the north-west
const std::vector<std::array<double, 2>> kAlongsideNorth = {{40, 60}, {100, 40}, {160, 22}, {200, 14}, {240, 10}};
// running -x alongside the +y side, from the north-east: against a road running +x
const std::vector<std::array<double, 2>> kAgainstNorth = {{460, 60}, {400, 40}, {340, 22}, {300, 14}, {260, 10}};
nlohmann::json offsideAnchor() { return {{"edge", "st"}, {"at", {250, 0}}, {"side", "offside"}, {"aux", 30}, {"taper", 20}}; }
}  // namespace

// A RAMP MERGING INTO A ONE-WAY ROAD going its way, beside the road's lane nearest it (its offside lane: the ramp comes
// from that side), is built and passes; the same ramp running the other way is refused; and a ramp beside a TWO-WAY
// road's offside lane is refused, its lane of its own standing in the carriageway coming the other way.
TEST_CASE(lanes_a_ramp_merging_beside_a_one_way_roads_lane_nearest_it_is_built_and_one_against_it_refused) {
    const nlohmann::json near = {{"street", "st"}, {"joins", "merge"}, {"carriageway", "forward"}, {"half", "near"}, {"crossing", "none"}};
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(oneWayRampScene(offsideAnchor(), kAlongsideNorth, near)));
    for (const RampLandingFault& f : checkRampLandings(r->graph)) std::printf("    fault: %s\n", describe(f).c_str());
    CHECK(checkRampLandings(r->graph).empty());
    CHECK(r->graph.find("r")->anchorLanes.at("to") == "st.f0");   // the one-way road's offside lane
    checkInvariants(*r);
    // going the other way along the one-way road: refused
    CHECK(refuses(oneWayRampScene(offsideAnchor(), kAgainstNorth, near), "runs against it"));
    // beside a two-way road's offside lane: refused
    CHECK(refuses(oneRampScene(offsideAnchor(), kAlongsideNorth, near), "offside lane"));
    // and a ramp planned to cross nothing that crosses the street at grade to its lane: refused (the near half of a
    // two-way road beside the ramp coming from the north is the backward carriageway; its forward kerb lane is beyond)
    CHECK(refuses(oneRampScene(kerbAnchor("kerb"), kAlongsideNorth, near), "at grade"));
}

// A FRONTAGE ROAD AS PLANNED is checked: one-way, running the way the carriageway it is planned beside runs. Planned
// beside a carriageway running the other way, or with lanes both ways, the graph is refused, naming the road.
TEST_CASE(lanes_the_builder_refuses_a_frontage_road_that_does_not_run_the_freeways_way) {
    auto scene = [](bool runsWithIt, bool oneWay) {
        nlohmann::json g;
        g["name"] = "frontage";
        nlohmann::json fr = {{"id", "fr"}, {"class", "collector"}, {"frontage_of", "fw"},
                             {"path", {{"type", "polyline"}, {"points", runsWithIt ? nlohmann::json{{0, -50}, {600, -50}} : nlohmann::json{{600, -50}, {0, -50}}}}}};
        if (oneWay) fr["lanes"] = {{"fwd", 2}, {"back", 0}};
        g["edges"] = nlohmann::json::array({{{"id", "fw"}, {"class", "freeway"}, {"path", {{"type", "polyline"}, {"points", {{0, 0}, {600, 0}}}}}}, fr});
        return g;
    };
    CHECK(checkFrontageRoads(RoadLabGraph::fromJson(scene(true, true))).empty());
    auto refused = [](const nlohmann::json& g, const char* naming) {
        try { (void)build(RoadLabGraph::fromJson(g)); } catch (const std::runtime_error& e) {
            std::printf("    refused: %s\n", e.what());
            return std::string(e.what()).find("frontage road fr") != std::string::npos && std::string(e.what()).find(naming) != std::string::npos;
        }
        return false;
    };
    CHECK(refused(scene(false, true), "run against it"));
    CHECK(refused(scene(true, false), "lanes both ways"));
}

// THE PLANNER LANDS A DIAMOND BETWEEN ONE-WAY FRONTAGE ROADS AT GRADE (ADR-0413): the freeway of the test above, its
// frontage roads one-way the way the carriageway beside each runs (south, a's side: +x; north, b's: -x). All four ramps
// come down beside their frontage road and join or leave its lane nearest the freeway -- the offside lane -- on the near
// half, crossing nothing (no floor point, no crossing). As built, every ramp lands as planned and every frontage road
// runs as planned. With a local street cutting the south road 100 m short of the crossing, where a's off-ramp needs its
// lane of its own: a street that may go (spare) is closed and the two pieces are one road for the ramp; one that may not
// leaves no room, and the diamond is refused.
namespace {
std::vector<RampStreet> oneWayFrontageStreets(bool sideStreet, bool spare) {
    std::vector<RampStreet> s = {
        {"x_s", {{1000.0, -250.0}, {1000.0, -50.0}}, false, "collector"}, {"x_m", {{1000.0, -50.0}, {1000.0, 50.0}}, false, "collector"},
        {"x_n", {{1000.0, 50.0}, {1000.0, 250.0}}, false, "collector"},
        {"f_s1", {{0.0, -50.0}, {sideStreet ? 900.0 : 1000.0, -50.0}}, false, "collector", 2}, {"f_s2", {{1000.0, -50.0}, {2000.0, -50.0}}, false, "collector", 2},
        {"f_n1", {{1000.0, 50.0}, {0.0, 50.0}}, false, "collector", 2}, {"f_n2", {{2000.0, 50.0}, {1000.0, 50.0}}, false, "collector", 2}};
    if (sideStreet) {
        s.push_back({"f_s1b", {{900.0, -50.0}, {1000.0, -50.0}}, false, "collector", 2});
        s.push_back({"side", {{900.0, -50.0}, {900.0, -250.0}}, false, "local", 0, spare});
    }
    return s;
}
// the scene as plan_scene writes it: the freeway's carriageways, the streets (closed ones left out, joined pieces one
// edge under the first's id), the frontage roads one-way with whose frontage road each is, and the ramps
nlohmann::json oneWayFrontageScene(const std::vector<RampStreet>& streets, const DiamondResult& dr) {
    auto line = [](const std::vector<Vec2>& v) { nlohmann::json p = nlohmann::json::array(); for (const Vec2& q : v) p.push_back({q.x, q.y}); return p; };
    nlohmann::json g;
    g["name"] = "one_way_frontage_diamond";
    g["edges"] = nlohmann::json::array();
    std::vector<Vec2> a, b;
    for (double x = 0; x <= 2000.0; x += 10.0) { a.push_back(Vec2(x, -7)); b.push_back(Vec2(2000 - x, 7)); }
    nlohmann::json floor = nlohmann::json::array();
    for (double x = 0; x <= 2000.0; x += 30.0) floor.push_back({x, 0.0, 8.4, 15.0});
    g["edges"].push_back({{"id", "fw_a"}, {"class", "freeway"}, {"floor", floor}, {"path", {{"type", "polyline"}, {"points", line(a)}}}});
    g["edges"].push_back({{"id", "fw_b"}, {"class", "freeway"}, {"floor", floor}, {"path", {{"type", "polyline"}, {"points", line(b)}}}});
    for (const RampStreet& s : streets) {
        if (std::find(dr.closed.begin(), dr.closed.end(), s.id) != dr.closed.end()) continue;
        std::vector<Vec2> xy = s.xy;
        bool absorbed = false;
        for (const std::vector<std::string>& run : dr.joined) {
            if (run.front() == s.id)
                for (std::size_t i = 1; i < run.size(); ++i)
                    for (const RampStreet& t : streets) if (t.id == run[i]) xy.insert(xy.end(), t.xy.begin() + 1, t.xy.end());
            absorbed = absorbed || std::find(run.begin() + 1, run.end(), s.id) != run.end();
        }
        if (absorbed) continue;
        nlohmann::json e = {{"id", s.id}, {"class", s.cls}, {"path", {{"type", "polyline"}, {"points", line(xy)}}}};
        if (s.oneWayLanes > 0) {
            e["lanes"] = {{"fwd", s.oneWayLanes}, {"back", 0}};
            e["frontage_of"] = xy.front().y < 0 ? "fw_a" : "fw_b";
        }
        g["edges"].push_back(e);
    }
    for (const nlohmann::json& e : dr.ramps) g["edges"].push_back(e);
    return g;
}
}  // namespace

TEST_CASE(lanes_the_planner_lands_a_diamond_between_one_way_frontage_roads_at_grade_on_the_lane_nearest_the_freeway) {
    const std::vector<RampStreet> streets = oneWayFrontageStreets(false, false);
    const DiamondResult dr = frontageDiamond(streets);
    CHECK(dr.built == 1);
    CHECK(dr.ramps.size() == 4);
    CHECK(dr.closed.empty());
    for (const nlohmann::json& e : dr.ramps) {
        const bool off = e["id"].get<std::string>().find("_off") != std::string::npos;
        const nlohmann::json& street = off ? e["to"] : e["from"];
        const nlohmann::json& L = e["landing"];
        std::printf("    %s: %s %s's %s carriageway (%s half) beside its %s lane, crossing %s; floor %s\n", e["id"].get<std::string>().c_str(),
                    L["joins"].get<std::string>().c_str(), L["street"].get<std::string>().c_str(), L["carriageway"].get<std::string>().c_str(),
                    L["half"].get<std::string>().c_str(), street["side"].get<std::string>().c_str(), L["crossing"].dump().c_str(),
                    e.contains("floor") ? e["floor"].dump().c_str() : "none");
        CHECK(street.is_object() && street["edge"] == L["street"]);
        CHECK(L["joins"] == "merge" && L["half"] == "near" && L["carriageway"] == "forward" && L["crossing"] == "none");
        CHECK(!e.contains("floor"));   // nothing to hold it over
        CHECK(street["side"] == "offside");
        const bool a = e["id"].get<std::string>().rfind("d0_0_a", 0) == 0;
        CHECK(L["street"].get<std::string>().rfind(a ? "f_s" : "f_n", 0) == 0);
    }
    // as built: the ramps land as planned, crossing nothing, and the frontage roads run as planned
    const nlohmann::json g = oneWayFrontageScene(streets, dr);
    if (const char* outPath = std::getenv("RT_LANES_SCENE_OUT")) std::ofstream(outPath) << g.dump(1) << "\n";   // for lanes_tool to draw
    RoadLabGraph rg = RoadLabGraph::fromJson(g);
    (void)expand(rg);
    for (const RampLandingFault& f : checkRampLandings(rg)) std::printf("    fault: %s\n", describe(f).c_str());
    CHECK(checkRampLandings(rg).empty());
    CHECK(checkFrontageRoads(rg).empty());
    for (const nlohmann::json& e : dr.ramps) {   // its own path never meets its frontage road's line
        const EdgeSpec& ramp = *rg.find(e["id"].get<std::string>());
        CHECK(crossings(ramp.xy, rg.find(e["landing"]["street"].get<std::string>())->xy).empty());
    }

    // a side street 100 m short of the crossing, which may go: closed, and the south road's two pieces one road
    const std::vector<RampStreet> withSpare = oneWayFrontageStreets(true, true);
    const DiamondResult ds = frontageDiamond(withSpare);
    std::printf("    with a spare side street: built %d, closed %zu, joined %zu\n", ds.built, ds.closed.size(), ds.joined.size());
    CHECK(ds.built == 1);
    CHECK(ds.closed == std::vector<std::string>{"side"});
    CHECK(ds.joined.size() == 1 && ds.joined[0] == (std::vector<std::string>{"f_s1", "f_s1b"}));
    RoadLabGraph rs = RoadLabGraph::fromJson(oneWayFrontageScene(withSpare, ds));
    (void)expand(rs);
    for (const RampLandingFault& f : checkRampLandings(rs)) std::printf("    fault: %s\n", describe(f).c_str());
    CHECK(checkRampLandings(rs).empty());
    CHECK(rs.find("d0_0_a_off")->landing.street == "f_s1");
    // ...and one that may not: no room, the diamond refused
    const DiamondResult dn = frontageDiamond(oneWayFrontageStreets(true, false));
    std::printf("    with a side street that may not go: built %d, refused for want of a landing %d\n", dn.built, dn.rejectedLanding);
    CHECK(dn.built == 0);
    CHECK(dn.rejectedLanding >= 1);
}

// WHERE THE SIDEWALK STARTS (issue #292): outside the shoulder, at the carriageway's edge (RoadLabGraph::
// sidewalkOuterHalfWidth), the one answer the pavement, the bands and the ground share. The scene: one straight road,
// 200 m along y = 0, two 3.5 m lanes (half width 3.5 m), a 2.5 m shoulder and a 2 m sidewalk. Its shoulder runs from 3.5
// to 6 m off the centre line and its sidewalk from 6 to 8 m, each side: 2 x 2 m x 200 m = 800 m2 of sidewalk. (The
// pavement once laid the sidewalk from the lanes' edge, 3.5 to 5.5 m: wholly under the shoulder, so none was drawn.)
TEST_CASE(lanes_the_sidewalk_lies_outside_the_shoulder) {
    nlohmann::json g;
    g["name"] = "sidewalk_outside_shoulder";
    g["classes"]["verge_street"] = {{"w", 3.5}, {"fwd", 1}, {"back", 1}, {"shoulder", 2.5}, {"sidewalk", 2.0},
                                    {"g_max", 0.09}, {"window", 40}, {"thick", 0.5}};
    g["edges"] = nlohmann::json::array(
        {{{"id", "v"}, {"class", "verge_street"}, {"path", {{"type", "polyline"}, {"points", {{-100, 0}, {100, 0}}}}}}});
    const std::unique_ptr<Result> r = build(RoadLabGraph::fromJson(g));
    const EdgeSpec& e = *r->graph.find("v");
    CHECK(approxEqual(r->graph.hw(e), 3.5, kSamePoint));
    CHECK(approxEqual(r->graph.carriagewayHalfWidth(e), 6.0, kSamePoint));
    CHECK(approxEqual(r->graph.sidewalkOuterHalfWidth(e), 8.0, kSamePoint));
    for (const double side : {1.0, -1.0}) {
        CHECK(contains(r->pavement.sidewalk, Vec2(0, side * 7.0)));     // the middle of the sidewalk
        CHECK(!contains(r->pavement.sidewalk, Vec2(0, side * 4.75)));   // the middle of the shoulder
        CHECK(contains(r->pavement.shoulder, Vec2(0, side * 4.75)));
        CHECK(!contains(r->pavement.sidewalk, Vec2(0, side * 8.5)));    // beyond the sidewalk
    }
    const double area = setArea(r->pavement.sidewalk);
    std::printf("    sidewalk %.1f m2 (two 2 m strips of 200 m: 800)\n", area);
    CHECK(area > 780.0 && area < 840.0);
}
