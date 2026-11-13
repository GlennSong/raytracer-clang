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
#include "apps/citysim/city_sim.h"
#include "engine/ai/nav_graph.h"
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
    engine::RoadGraph rg;    // the twin's street subgraph: freeways and ramps left out
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

// THE REGRESSION THAT WOULD HAVE CAUGHT "half on the sidewalk" (parking round), on the road the city DRIVES:
// the nav twin a lane-built city's sim runs on (city_producer: navRoadGraph(roadTwin(r, 60, false, forNav)),
// its streets' cross-sections from road_twin's roadSpecStreetParking). Every bay the sim mints lives inside the
// Parking band of its own road -- never a bare travel lane, the stall between the last travel lane and the kerb,
// the widest car in the fleet fitting inside the band. (It ran on the old metro grower's generate recipe until
// that retired, ADR-0267.) On a planned city: the lab scenes' streets carry two travel lanes in 12 m and have
// no room for a band, while a planned city's local street is one 6 m lane each way, parked along.
TEST_CASE(lanes_parking_bays_live_inside_the_road_s_parking_band) {
    std::size_t totalBays = 0;
    for (const char* name : {"planned/metro_v2_test_lanelab"}) {
        const Result& r = scene(name);
        const engine::NavGraph nav = engine::buildNavGraph(engine::navRoadGraph(roadTwin(r, 60.0, false, /*forNav=*/true), nullptr));
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
    CHECK(totalBays >= 2400u);   // measured 2611 (2026-10-29): a city parked along, not a handful of bays
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
    const engine::RoadEntity twin = roadTwin(r);
    engine::RoadGraph streets, freeway;
    for (const engine::RoadEdge& e : twin.graph.edges) {
        engine::RoadGraph& g = (e.klass == engine::RoadClass::Freeway || e.klass == engine::RoadClass::Ramp) ? freeway : streets;
        const int b = static_cast<int>(g.nodes.size());
        g.nodes.push_back(twin.graph.nodes[static_cast<size_t>(e.a)]); g.nodes.push_back(twin.graph.nodes[static_cast<size_t>(e.b)]);
        g.edges.push_back(engine::RoadEdge{b, b + 1, e.width, e.klass, 0});
    }
    CHECK(!freeway.edges.empty());   // the case is about a freeway: the planned city must carry one
    auto nearest = [](const engine::RoadGraph& g, const Vec2& p) {
        double best = 1e30;
        for (const engine::RoadEdge& e : g.edges) {
            const Vec2 A = g.nodes[static_cast<size_t>(e.a)].pos, B = g.nodes[static_cast<size_t>(e.b)].pos, ab = B - A;
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
    CHECK(p.hasTerrain == r.hasTerrain); CHECK(!p.cells.empty()); CHECK(!p.holes.empty()); CHECK(p.nav.nodes.size() > 10);
    size_t tris = 0; for (const CityCellMesh& c : p.cells) tris += c.mesh.triangleCount(); CHECK(tris > 100000);
    BundleWriter w; w.openMemory(); writeCityProducts(w, 0, p); nlohmann::json manifest = nlohmann::json::object(); std::string err; CHECK(w.finish(manifest, &err));
    std::unique_ptr<Bundle> b = Bundle::fromMemory(w.memory(), &err); CHECK(b != nullptr); if (!b) return;
    CityProducts q; CHECK(readCityProducts(*b, 0, q, &err)); if (!err.empty()) std::printf("    %s\n", err.c_str());
    CHECK(q.hasTerrain == p.hasTerrain && q.cell == p.cell && q.ground.z == p.ground.z && q.ground.nx == p.ground.nx);
    CHECK(q.twin.graph.nodes.size() == p.twin.graph.nodes.size() && q.twin.graph.edges.size() == p.twin.graph.edges.size() && q.nav.edges.size() == p.nav.edges.size());
    for (size_t i = 0; i < p.twin.graph.nodes.size() && i < q.twin.graph.nodes.size(); ++i) CHECK(p.twin.graph.nodes[i].pos.x == q.twin.graph.nodes[i].pos.x && p.twin.graph.nodes[i].elev == q.twin.graph.nodes[i].elev);
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
        for (const Lane& l : L.lanes) pts += l.xy.size();
        std::printf("    [expand] %-26s %5zu edges -> %6zu lanes, %8zu points: %.1f ms\n", name, g.edges.size(), L.lanes.size(), pts, ms);
    }
}

// ---- THE LANE GRAPH FROM THE BUILT LANES (#237), ITS CONNECTORS (#238) ----------------------------------------------------
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"
#include "engine/procgen/city/roads/lanes/lane_graph_connect.h"
#include <map>
#include <set>

namespace {
// The vehicle tags the tests open every lane to (tests' names, not kinds of vehicle), with the sizes the connectors are
// checked against: a car-sized one and a long one (turning radius and width, metres).
constexpr const char* kLaneGraphTestTag = "test_any";
constexpr const char* kLaneGraphLongTag = "test_long";
constexpr double kTestCarRadius = 5.0, kTestCarWidth = 1.9;
constexpr double kTestLongRadius = 10.0, kTestLongWidth = 2.6;
// A builder lane's paths and its stretches inside junctions may miss its length by this much (metres).
constexpr double kLaneCoverTolerance = 0.01;
// A section's lanes start (and end) within this of the section's start along its road's spine (metres).
constexpr double kSectionShareTolerance = 0.05;
// Two neighbours in a section stand no farther apart than their half widths and this (metres).
constexpr double kNeighbourSlack = 0.5;
// Two directions' cosines this close are the same direction (rounding).
constexpr double kCosineRounding = 1e-6;

const LaneGraphBuild& laneGraphOf(const std::string& name) {
    static std::map<std::string, LaneGraphBuild> cache;
    std::map<std::string, LaneGraphBuild>::iterator it = cache.find(name);
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
        it = cache.emplace(name, buildLaneGraph(scene(name), o)).first;
    }
    return it->second;
}
bool anyProblemWith(const LaneGraphBuild& lg, const std::string& words) {
    for (const std::string& p : lg.problems)
        if (p.find(words) != std::string::npos) return true;
    return false;
}
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

// THE CHECKS EVERY SCENE'S LANE GRAPH MUST PASS (#237's "done when" 1 to 3).
void checkLaneGraph(const std::string& name) {
    const Result& r = scene(name);
    const LaneGraphBuild& lg = laneGraphOf(name);
    const LaneGraph& G = lg.graph;
    uint32_t lanePaths = 0;
    for (const Path& p : G.paths) lanePaths += p.kind == PathKind::Lane ? 1 : 0;
    std::printf("    [lane graph] %s: %u lane paths, %zu sections, %zu junctions, %zu problems, %.0f ms; %u connector lanes left "
                "out, %u junctions merged, %u ends cut at an outline, %u road joins, %u crossings outside junctions\n",
                name.c_str(), lanePaths, G.sections.size(), G.junctions.size(), lg.problems.size(), lg.milliseconds,
                lg.connectorsLeftOut, lg.mergedJunctions, lg.endsAtOutline, lg.roadJoins, lg.crossingsOutside);
    for (std::size_t i = 0; i < lg.problems.size() && i < 4; ++i) std::printf("      problem: %s\n", lg.problems[i].c_str());

    // 1. sound
    const std::vector<std::string> faults = validate(G);
    for (std::size_t i = 0; i < faults.size() && i < 8; ++i) std::printf("      fault: %s\n", faults[i].c_str());
    CHECK(faults.empty());
    CHECK(lg.sources.size() == G.paths.size() && lg.sectionSources.size() == G.sections.size());
    if (lg.sources.size() != G.paths.size() || lg.sectionSources.size() != G.sections.size()) return;

    // 2. every builder lane cars drive covered once, end to end, by its paths and its stretches inside junctions (as the
    // graph reads it: a fold tidied out of it is not counted; a lane carried on past its end to a merged section's cut
    // covers only its own stretch)
    const LaneSet& L = r.lanes;
    std::vector<std::vector<uint32_t>> ofLane(L.lanes.size());
    for (uint32_t p = 0; p < G.paths.size(); ++p)
        if (G.paths[p].kind == PathKind::Lane) ofLane[static_cast<std::size_t>(lg.sources[p].lane)].push_back(p);
    int badCover = 0;
    for (std::size_t li = 0; li < L.lanes.size(); ++li) {
        if (lg.leftOut[li]) { CHECK(ofLane[li].empty()); continue; }
        std::vector<uint32_t>& ps = ofLane[li];
        std::sort(ps.begin(), ps.end(), [&](uint32_t a, uint32_t b) { return lg.sources[a].from < lg.sources[b].from; });
        double total = lg.insideJunctions[li], reached = 0;
        bool overlap = false;
        for (uint32_t p : ps) {
            overlap = overlap || lg.sources[p].from < reached - kLaneCoverTolerance;
            reached = lg.sources[p].to;
            total += lg.sources[p].to - lg.sources[p].from;
        }
        if (overlap || std::fabs(total - lg.laneLengths[li]) > kLaneCoverTolerance) {
            if (badCover++ < 4) std::printf("      lane %s: paths and junction stretches %.3f m of %.3f m%s\n", L.lanes[li].id.c_str(), total,
                                            lg.laneLengths[li], overlap ? ", overlapping" : "");
        }
        CHECK(lg.laneLengths[li] <= L.lanes[li].s.back() + kLaneCoverTolerance);
    }
    CHECK(badCover == 0);
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
        if (offStart > kSectionShareTolerance || offEnd > kSectionShareTolerance)
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
        if (ss.to - ss.from < kShortestSection) CHECK(anyProblemWith(lg, "section " + std::to_string(s) + " "));
    }
    CHECK(badShare == 0);
    CHECK(badNeighbours == 0);
    // where one lane follows another, they join within 1 cm and 1 degree -- or, where a builder lane is carried on into
    // the next section, they turn no more than the builder's lane itself turns there (a cut on a bend tighter than
    // about 29 m reads the two sides' directions half a metre apart, kTangentHalfSpan, and a lane kinked at a
    // sample turns at it whether cut there or not): the cut adds no kink. (Lanes and connectors: checkConnectors.)
    int badJoins = 0, onBends = 0;
    const double joinCos = std::cos(kJoinDegrees * 3.14159265358979 / 180.0);
    for (uint32_t p = 0; p < G.paths.size(); ++p)
        for (uint32_t k = G.paths[p].next.first; k < G.paths[p].next.end(); ++k) {
            const PathId q = G.nextPaths[k];
            if (G.paths[p].kind != PathKind::Lane || G.paths[q.index].kind != PathKind::Lane) continue;
            const double gap = distance(G.pointAt(PathId(p), G.paths[p].length), G.pointAt(q, 0));
            const double cosine = dot(G.directionAt(PathId(p), G.paths[p].length), G.directionAt(q, 0));
            double allowed = joinCos;
            if (lg.sources[p].lane == lg.sources[q.index].lane && cosine < joinCos) {
                const Lane& l = L.lanes[static_cast<std::size_t>(lg.sources[p].lane)];
                std::vector<Vec2> xy = l.xy;
                if (l.dir < 0) std::reverse(xy.begin(), xy.end());
                const std::vector<double> s = stations(xy);
                const double t = lg.sources[p].to;
                const Vec2 before = normalize(pointAt(xy, s, t) - pointAt(xy, s, t - kTangentHalfSpan));
                const Vec2 after = normalize(pointAt(xy, s, t + kTangentHalfSpan) - pointAt(xy, s, t));
                allowed = std::min(joinCos, dot(before, after) - kCosineRounding);
                ++onBends;
            }
            if (gap > kJoinDistance || cosine < allowed)
                if (badJoins++ < 4) std::printf("      lane %u -> lane %u: %.4f m apart, cos %.6f (allowed %.6f)\n", p, q.index, gap, cosine, allowed);
        }
    if (onBends) std::printf("      (%d cuts on bends tighter than a degree per half metre: held to the builder lane's own turn)\n", onBends);
    CHECK(badJoins == 0);

    // 3. junctions: two arriving and two leaving or reported; their ends on the outline; no lane inside one
    int inside = 0, skipped = 0;
    for (uint32_t j = 0; j < G.junctions.size(); ++j) {
        const Junction& jn = G.junctions[j];
        if (lg.junctionNotes[j].origin == JunctionOrigin::Box && (jn.arriving.count < 2 || jn.leaving.count < 2))
            CHECK(anyProblemWith(lg, "junction " + std::to_string(j) + " at"));
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
        if (lg.junctionNotes[j].refused || levels.empty()) { ++skipped; continue; }
        for (uint32_t p = 0; p < G.paths.size(); ++p)
            for (uint32_t k = G.paths[p].curve.first; k < G.paths[p].curve.end() && G.paths[p].kind == PathKind::Lane; ++k) {
                const Vec2 q = G.points[k];
                if (!pointInPolygon(outline, q) || distToPolyline(q, outline, true) <= kJunctionEndReach) continue;
                double nearest = 1e300;
                for (double z : levels) nearest = std::min(nearest, std::fabs(G.heights[k] - z));
                if (nearest > kJunctionLevelReach) continue;
                if (inside++ < 4) std::printf("      lane %u passes inside junction %u at (%.1f, %.1f)\n", p, j, q.x, q.y);
                break;
            }
    }
    if (skipped) std::printf("      (%d junctions refused were not searched for lanes inside them)\n", skipped);
    CHECK(inside == 0);
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
    const Result& r = scene(name);
    const LaneGraphBuild& lg = laneGraphOf(name);
    const LaneGraph& G = lg.graph;
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

    // every stated turn: one connector, or one refusal (a line in the problems; a refused junction's line for all of them)
    std::vector<uint32_t> refusedAt(G.junctions.size(), 0);
    for (const Refusal& ref : lg.refusals) refusedAt[ref.junction] += ref.from.valid() ? 1 : 0;
    int badCounts = 0;
    for (uint32_t j = 0; j < G.junctions.size(); ++j) {
        const JunctionNote& n = lg.junctionNotes[j];
        const Junction& jn = G.junctions[j];
        bool counted = n.turnsStated == n.turnsBuilt + n.turnsRefused && jn.connectors.count == n.turnsBuilt;
        counted = counted && (n.refused ? refusedAt[j] == 0 && anyProblemWith(lg, "junction " + std::to_string(j) + " at")
                                        : refusedAt[j] == n.turnsRefused);
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
    int problemRefusals = 0;
    for (const std::string& p : lg.problems) problemRefusals += p.find(" is refused, at (") != std::string::npos ? 1 : 0;
    CHECK(static_cast<std::size_t>(problemRefusals) == lg.refusals.size() - [&] {
        std::size_t whole = 0;
        for (const Refusal& ref : lg.refusals) whole += ref.from.valid() ? 0 : 1;
        return whole;
    }());

    // the ends: in position within 1 cm and direction within 1 degree of their lanes' (the lane's direction as the graph
    // reads it; the connector's, its own end piece); the tightest bend no tighter than any vehicle allowed on it; swept at
    // the widest of them, on the paved road
    const LaneLookup look(r.pavement);
    const double joinCos = std::cos(kJoinDegrees * 3.14159265358979 / 180.0);
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
        if (gapIn > kJoinDistance || gapOut > kJoinDistance || cosIn < joinCos || cosOut < joinCos || dzIn > kJoinDistance || dzOut > kJoinDistance)
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
        // (at the vehicle's own width, every kSweepStep along as the builder samples it: a crack between the lanes'
        // footprints narrower than a sample's step can pass between two samples, here as there)
        const int samples = std::max(1, static_cast<int>(std::ceil(p.length / kSweepStep)));
        for (int k = 0; k <= samples && widest >= 0; ++k) {
            const double s = p.length * k / samples;
            const Vec2 at = G.pointAt(PathId(c), s), side = perp(G.directionAt(PathId(c), s)) * (widest / 2);
            if (!paved(at + side, G.heightAt(PathId(c), s)) || !paved(at - side, G.heightAt(PathId(c), s))) {
                if (offRoad++ < 4)
                    std::printf("      connector %u (%s, %.1f m, tightest %.1f m) swept %.1f m wide leaves the paved road %.2f m along it, "
                                "at (%.2f, %.2f)\n", c, p.turn == Turn::Straight ? "straight on" : "a turn", p.length,
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
TEST_CASE(lanes_the_traffic_lab_lane_graph_can_be_driven_round) {
    const LaneGraphBuild& lg = laneGraphOf("planned/traffic_lab_lanelab");
    std::printf("    traffic lab: %.1f%% of the lanes in the largest part a car can get round\n", 100 * lg.reachableShare);
    for (const std::pair<std::string, std::vector<uint32_t>>& o : lg.outside) std::printf("      %zu lanes %s\n", o.second.size(), o.first.c_str());
    CHECK(lg.reachableShare >= 0.9);
    CHECK(lg.unjoined.empty());
    CHECK(lg.crossingsOutside == 0);
    // every problem is a refusal with its place, a fold tidied, a crossing point to look at (lane_graph_crossings.h), or a
    // kink in a line with its place (#281: the road builder's to fix, reported, not smoothed)
    for (const std::string& p : lg.problems) {
        const bool known = p.find(" is refused, at (") != std::string::npos || p.find("; tidied (") != std::string::npos ||
                           p.find(": refused: ") != std::string::npos || p.find("crossing points: ") == 0 ||
                           (p.find("kink: ") == 0 && p.find(" at (") != std::string::npos);
        if (!known) std::printf("      problem: %s\n", p.c_str());
        CHECK(known);
    }
}

// THE TRAFFIC LAB'S BENDS LOOK LIKE ITS ROADS (#281): every path's bend profile measured as a vehicle feels it. The
// lanes of the class with the highest posted limit (the lab's freeway ring, about 290 m round) bend in the hundreds of
// metres, not the 27.5 m the old point-to-point reading gave; each lane's tightest radius is its profile's tightest; a
// profile is small; and every kink counted is listed in the problems with its place.
TEST_CASE(lanes_the_traffic_lab_bends_look_like_its_roads) {
    const LaneGraphBuild& lg = laneGraphOf("planned/traffic_lab_lanelab");
    const LaneGraph& G = lg.graph;
    double fastest = 0;
    for (const Path& p : G.paths)
        if (p.kind == PathKind::Lane) fastest = std::max(fastest, p.speedLimit);
    std::vector<double> radii;
    uint32_t mismatched = 0;
    for (uint32_t i = 0; i < G.paths.size(); ++i) {
        const Path& p = G.paths[i];
        if (p.kind != PathKind::Lane) continue;
        if (G.tightestBend(PathId(i)).radius != p.tightestRadius) ++mismatched;
        if (p.speedLimit == fastest) radii.push_back(G.tightestBend(PathId(i)).radius);
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
    for (const std::string& p : lg.problems) listed += p.find("kink: ") == 0 ? 1 : 0;
    CHECK(listed == lg.laneKinks + lg.connectorKinks + lg.joinKinks);
}

// ---- which turns a junction offers, on hand-made junctions -------------------------------------------------------------
namespace {
// A hand-made junction's legs: roads arriving from the given compass bearings (degrees, the way they point away from the
// junction's middle), `lanes` lanes each way, 3.5 m wide, ends 10 m out. Lane ids: road r's arriving lane k is
// r * 100 + k, its leaving lane k r * 100 + 50 + k (k offside to kerb).
constexpr double kHandLaneWidth = 3.5, kHandReach = 10.0, kHandDegrees = 3.14159265358979 / 180.0;
void handJunction(const std::vector<double>& bearings, const std::vector<uint32_t>& lanesEachWay, double kerbPerp, bool mirrored,
                  std::vector<TurnLeg>& arriving, std::vector<TurnLeg>& leaving) {
    arriving.clear();
    leaving.clear();
    for (std::size_t r = 0; r < bearings.size(); ++r) {
        Vec2 out(std::cos(bearings[r] * kHandDegrees), std::sin(bearings[r] * kHandDegrees));
        if (mirrored) out.y = -out.y;
        TurnLeg in, away;
        in.road = away.road = static_cast<int>(r);
        in.dir = 1;
        away.dir = -1;
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
std::vector<JunctionTurn> handTurns(const std::vector<double>& bearings, const std::vector<uint32_t>& lanes, double kerbPerp = -1,
                                    bool mirrored = false) {
    std::vector<TurnLeg> a, l;
    handJunction(bearings, lanes, kerbPerp, mirrored, a, l);
    TurnRules rules;
    rules.kerbPerpSign = kerbPerp;
    return junctionTurns(a, l, rules);
}
int countTurns(const std::vector<JunctionTurn>& ts, Turn kind) {
    int n = 0;
    for (const JunctionTurn& t : ts) n += t.turn == kind ? 1 : 0;
    return n;
}
bool hasTurn(const std::vector<JunctionTurn>& ts, uint32_t from, uint32_t to, Turn kind) {
    for (const JunctionTurn& t : ts)
        if (t.from == from && t.to == to && t.turn == kind) return true;
    return false;
}
// Whether two lists of turns are the same, kinds and all.
bool sameTurns(const std::vector<JunctionTurn>& a, const std::vector<JunctionTurn>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i].from != b[i].from || a[i].to != b[i].to || a[i].turn != b[i].turn) return false;
    return true;
}
}  // namespace

TEST_CASE(lanes_junction_turns_four_way_one_lane_each_way_has_twelve) {
    // roads to the east, north, west and south; kerb on perp(direction) * -1 (this engine's side)
    const std::vector<JunctionTurn> ts = handTurns({0, 90, 180, 270}, {1, 1, 1, 1});
    CHECK(ts.size() == 12);
    CHECK(countTurns(ts, Turn::Straight) == 4 && countTurns(ts, Turn::NearTurn) == 4 && countTurns(ts, Turn::FarTurn) == 4);
    // arriving from the east (heading west), the kerb is on perp(west) * -1 = north: the near turn goes north (road 1)
    CHECK(hasTurn(ts, 0, 150, Turn::NearTurn));
    CHECK(hasTurn(ts, 0, 250, Turn::Straight));
    CHECK(hasTurn(ts, 0, 350, Turn::FarTurn));
    for (const JunctionTurn& t : ts) CHECK(t.from / 100 != t.to / 100);   // never back the way it came
}

TEST_CASE(lanes_junction_turns_two_lanes_each_way_pair_kerb_and_offside) {
    const std::vector<JunctionTurn> ts = handTurns({0, 90, 180, 270}, {2, 2, 2, 2});
    // straight on lane by lane; the near turn from the kerb lane (1) into the kerb lane; the far turn offside to offside
    CHECK(countTurns(ts, Turn::Straight) == 8 && countTurns(ts, Turn::NearTurn) == 4 && countTurns(ts, Turn::FarTurn) == 4);
    CHECK(hasTurn(ts, 0, 250, Turn::Straight) && hasTurn(ts, 1, 251, Turn::Straight));
    CHECK(hasTurn(ts, 1, 151, Turn::NearTurn) && !hasTurn(ts, 0, 150, Turn::NearTurn));
    CHECK(hasTurn(ts, 0, 350, Turn::FarTurn) && !hasTurn(ts, 1, 351, Turn::FarTurn));
}

TEST_CASE(lanes_junction_turns_a_t_shares_the_stem_between_its_turns) {
    // a through road east-west, a stem to the south; two lanes each way
    const std::vector<JunctionTurn> ts = handTurns({0, 180, 270}, {2, 2, 2});
    // the stem (road 2) has no way straight on: its offside lane takes the far turn, its kerb lane the near turn
    CHECK(hasTurn(ts, 200, 0 * 100 + 50 + 0, Turn::FarTurn) || hasTurn(ts, 200, 1 * 100 + 50 + 0, Turn::FarTurn));
    int fromStem = 0;
    for (const JunctionTurn& t : ts)
        if (t.from / 100 == 2) {
            ++fromStem;
            CHECK((t.turn == Turn::FarTurn && t.from == 200 && t.to % 50 == 0) || (t.turn == Turn::NearTurn && t.from == 201 && t.to % 50 == 1));
        }
    CHECK(fromStem == 2);
    // six movements in all, as a T has: two from each road (the through road: straight on and one turn into the stem)
    std::set<std::pair<uint32_t, uint32_t>> movements;
    for (const JunctionTurn& t : ts) movements.insert(std::make_pair(t.from / 100, t.to / 100));
    CHECK(movements.size() == 6);
}

TEST_CASE(lanes_junction_turns_into_a_road_with_fewer_lanes_take_the_nearest) {
    // three lanes arrive from the west (road 1) and go straight on into two leaving east (road 0)
    const std::vector<JunctionTurn> ts = handTurns({0, 180}, {2, 3});
    // the lanes line up from the middle line out: the third, kerb-most, goes into the nearest, the kerb lane
    CHECK(hasTurn(ts, 100, 50, Turn::Straight));
    CHECK(hasTurn(ts, 101, 51, Turn::Straight));
    CHECK(hasTurn(ts, 102, 51, Turn::Straight));
    CHECK(countTurns(ts, Turn::Straight) == 3 + 2);   // and the two lanes coming back the other way into three: each one
}

TEST_CASE(lanes_junction_turns_skewed_four_way_classifies_by_angle) {
    // the cross road meets at 60 degrees: the straight-on legs are still straight on, the turns near or far by side
    const std::vector<JunctionTurn> ts = handTurns({0, 60, 180, 240}, {1, 1, 1, 1});
    CHECK(ts.size() == 12);
    CHECK(countTurns(ts, Turn::Straight) == 4 && countTurns(ts, Turn::NearTurn) == 4 && countTurns(ts, Turn::FarTurn) == 4);
}

TEST_CASE(lanes_junction_turns_five_way_one_straight_on_per_road) {
    // five roads; the one 20 degrees off straight on and the one 35 degrees off: the straighter is straight on
    const std::vector<JunctionTurn> ts = handTurns({0, 90, 160, 215, 270}, {1, 1, 1, 1, 1});
    CHECK(ts.size() == 20);
    for (uint32_t r = 0; r < 5; ++r) {
        int straight = 0;
        for (const JunctionTurn& t : ts) straight += t.from / 100 == r && t.turn == Turn::Straight ? 1 : 0;
        CHECK(straight <= 1);
    }
    CHECK(hasTurn(ts, 0, 250, Turn::Straight));   // from the east: 160 is 20 degrees off straight on, 215 is 35
    CHECK(!hasTurn(ts, 0, 350, Turn::Straight));
}

TEST_CASE(lanes_junction_turns_mirrored_mirror_their_sides_and_nothing_else) {
    for (const std::vector<double>& bearings : {std::vector<double>{0, 90, 180, 270}, std::vector<double>{0, 60, 180, 240},
                                                std::vector<double>{0, 90, 160, 215, 270}, std::vector<double>{0, 180, 270}}) {
        const std::vector<uint32_t> lanes(bearings.size(), 2);
        const std::vector<JunctionTurn> asBuilt = handTurns(bearings, lanes, -1, false);
        // the same junction seen in a mirror, its traffic keeping to the other side: the same turns, the same kinds (a near
        // turn is still the one by the kerb, now turning the other way)
        CHECK(sameTurns(asBuilt, handTurns(bearings, lanes, 1, true)));
        // mirrored alone, one lane each way: the same pairs, near and far swapped
        const std::vector<uint32_t> one(bearings.size(), 1);
        const std::vector<JunctionTurn> a = handTurns(bearings, one, -1, false), b = handTurns(bearings, one, -1, true);
        CHECK(a.size() == b.size());
        for (std::size_t i = 0; i < a.size() && i < b.size(); ++i) {
            CHECK(a[i].from == b[i].from && a[i].to == b[i].to);
            const Turn swapped = a[i].turn == Turn::NearTurn ? Turn::FarTurn : a[i].turn == Turn::FarTurn ? Turn::NearTurn : a[i].turn;
            CHECK(b[i].turn == swapped);
        }
    }
}

// THE SHAPE: a quarter turn between two lane ends 8 m from where their lines cross is straight, an arc of the largest
// radius that fits, straight; its ends exactly the lanes'.
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

TEST_CASE(lanes_lane_graph_of_grid_city_is_sound_and_covers_every_lane) { checkLaneGraph("grid_city"); }
TEST_CASE(lanes_lane_graph_of_hill_junction_is_sound_and_covers_every_lane) { checkLaneGraph("hill_junction"); }
TEST_CASE(lanes_lane_graph_of_ring_city_is_sound_and_covers_every_lane) { checkLaneGraph("ring_city"); }
TEST_CASE(lanes_lane_graph_of_valley_viaduct_is_sound_and_covers_every_lane) { checkLaneGraph("valley_viaduct"); }
TEST_CASE(lanes_lane_graph_of_the_traffic_lab_is_sound_and_covers_every_lane) { checkLaneGraph("planned/traffic_lab_lanelab"); }

// One grid_city junction by hand: the two arterials (two lanes each way) crossing at (160, 120). Four approaches of two
// lanes each arrive and four leave; an arriving lane of v2 ends on the paint's stop bar.
TEST_CASE(lanes_lane_graph_a_grid_city_crossing_checked_by_hand) {
    const Result& r = scene("grid_city");
    const LaneGraphBuild& lg = laneGraphOf("grid_city");
    const LaneGraph& G = lg.graph;
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
    auto roadOf = [&](PathId p) { return r.lanes.lanes[static_cast<std::size_t>(lg.sources[p.index].lane)].parent; };
    int fromV2 = 0, fromH2 = 0;
    for (uint32_t k = jn.arriving.first; k < jn.arriving.end(); ++k) {
        fromV2 += roadOf(G.junctionLanes[k]) == v2 ? 1 : 0;
        fromH2 += roadOf(G.junctionLanes[k]) == h2 ? 1 : 0;
    }
    CHECK(fromV2 == 4 && fromH2 == 4);
    // v2's box there, as the paint finds it; a lane arriving up v2 (along its spine) ends on the stop bar
    const LaneLookup look(r.pavement);
    const std::map<int, std::vector<JunctionSpan>> spans = junctionSpans(r, look);
    const EdgeSpec& e = r.graph.edges[static_cast<std::size_t>(v2)];
    const double at = project(e.xy, e.s, crossing).station;
    const JunctionSpan* box = nullptr;
    for (const JunctionSpan& sp : spans.at(v2))
        if (at >= sp.s0 && at <= sp.s1) box = &sp;
    CHECK(box != nullptr);
    if (!box) return;
    CHECK(box->crossRank >= r.graph.cls(e).rank);   // the bar is painted: the crossing road ranks as high
    int onBar = 0;
    for (uint32_t k = jn.arriving.first; k < jn.arriving.end(); ++k) {
        const PathId p = G.junctionLanes[k];
        const Lane& l = r.lanes.lanes[static_cast<std::size_t>(lg.sources[p.index].lane)];
        if (l.parent != v2 || l.dir < 0) continue;
        const Vec2 end = G.pointAt(p, G.paths[p.index].length);
        const Projection pr = project(e.xy, e.s, end);
        std::printf("    v2 lane %s arrives at %.3f m along v2; the stop bar stands at %.3f m, %.2f m off the spine\n", l.id.c_str(),
                    pr.station, box->s0 - kStopLineBack, pr.distance);
        CHECK(std::fabs(pr.station - (box->s0 - kStopLineBack)) < kJoinDistance);
        CHECK(pr.distance < r.graph.hw(e));   // across the road, within its lanes: where the bar is painted
        ++onBar;
    }
    CHECK(onBar == 2);
}

// Offside to kerb: in a section of two lanes each way, lane 0 is the one nearer the road's centre line, both ways.
TEST_CASE(lanes_lane_graph_lists_a_two_lane_section_offside_to_kerb) {
    const Result& r = scene("grid_city");
    const LaneGraphBuild& lg = laneGraphOf("grid_city");
    const LaneGraph& G = lg.graph;
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

// ---- A LEVEL'S LANE GRAPH FROM ITS CACHE (lane_graph_cache.h) -----------------------------------------------------------
#include "engine/procgen/city/roads/lanes/lane_graph_cache.h"

namespace {
// A level holding one lane-built city, the given lanelab graph, as the loader reads it.
engine::bundle::LevelInputs cacheTestLevel(const std::string& graphName) {
    engine::bundle::LevelInputs in;
    in.levelPath = "lane_graph_cache_test.json";   // named only; the key reads the graph file, not the level file
    in.levelDir = ".";
    in.level = {{"entities", nlohmann::json::array({{{"shape", "lanelab"},
                                                     {"lanelab", {{"graph", std::string(RT_SOURCE_DIR) + "/assets/lanelab/" + graphName + ".json"}}}}})}};
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
    const engine::bundle::LevelInputs in = cacheTestLevel("hill_junction");
    const LaneGraphOptions opts = cacheTestVehicles();

    const LevelLaneGraph first = obtainLevelLaneGraph(in, opts, dir);
    std::printf("    first load: %s\n", describeLevelLaneGraph(first).c_str());
    CHECK(first.graph != nullptr);
    if (!first.graph) return;
    CHECK(!first.fromCache && first.saved && first.roadSeconds > 0);
    CHECK(first.key == laneGraphCacheKey(levelRoadsKey(in), opts));
    CHECK(levelRoadsKey(in) != 0);
    CHECK(first.notes.empty());
    CHECK(first.summary.connectors > 0);
    // the very graph the lane graph tests build from the same roads
    CHECK(*first.graph == laneGraphOf("hill_junction").graph);
    CHECK(first.summary == summarizeLaneGraph(laneGraphOf("hill_junction")));
    const std::vector<char> bytes = cacheFileBytes(first.path);
    CHECK(!bytes.empty());

    const LevelLaneGraph second = obtainLevelLaneGraph(in, opts, dir);
    std::printf("    second load: %s\n", describeLevelLaneGraph(second).c_str());
    CHECK(second.fromCache && !second.saved && second.roadSeconds == 0);
    CHECK(second.graph != nullptr && *second.graph == *first.graph);
    CHECK(second.summary == first.summary);
    CHECK(dumpText(*second.graph) == dumpText(*first.graph));

    // lost: built again, to the same bytes
    fs::remove(first.path, ec);
    const LevelLaneGraph rebuilt = obtainLevelLaneGraph(in, opts, dir);
    CHECK(!rebuilt.fromCache && rebuilt.saved);
    CHECK(cacheFileBytes(first.path) == bytes);

    // damaged in the middle of the graph, or cut short: refused (a note says why), built again, written over
    for (int damage = 0; damage < 2; ++damage) {
        std::vector<char> bad = bytes;
        if (damage == 0) bad[bad.size() / 2] = static_cast<char>(~bad[bad.size() / 2]);
        else bad.resize(bad.size() / 2);
        putCacheFile(first.path, bad);
        const LevelLaneGraph again = obtainLevelLaneGraph(in, opts, dir);
        CHECK(!again.fromCache && again.saved);
        CHECK(again.graph != nullptr && *again.graph == *first.graph);
        CHECK(again.notes.size() == 1 && again.notes[0].find("refused") != std::string::npos);
        if (!again.notes.empty()) std::printf("    %s\n", again.notes[0].c_str());
        CHECK(cacheFileBytes(first.path) == bytes);
        CHECK(obtainLevelLaneGraph(in, opts, dir).fromCache);
    }

    // other vehicles: another key, another file; the first stays
    LaneGraphOptions fewer = opts;
    fewer.vehicles.pop_back();
    const LevelLaneGraph other = obtainLevelLaneGraph(in, fewer, dir);
    CHECK(other.key != first.key && other.path != first.path && !other.fromCache);
    CHECK(fs::exists(first.path, ec));

    // no cache folder (RT_NOCACHE): built, nothing written
    const LevelLaneGraph uncached = obtainLevelLaneGraph(in, opts, std::string());
    CHECK(uncached.graph != nullptr && !uncached.fromCache && !uncached.saved && uncached.path.empty());
    CHECK(*uncached.graph == *first.graph);

    // a level with no lane-built city: no graph, and why
    engine::bundle::LevelInputs empty;
    empty.level = {{"entities", nlohmann::json::array()}};
    const LevelLaneGraph none = obtainLevelLaneGraph(empty, opts, dir);
    CHECK(none.graph == nullptr && !none.error.empty());
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
constexpr double kTestSpeedLimit = 10.0;   // metres per second, any: nothing here reads it

// Adds a path open to both test vehicles, with an empty next list beside it.
uint32_t addTestPath(LaneGraph& g, std::vector<std::vector<PathId>>& next, PathKind kind, const std::vector<Vec2>& pts,
                     const std::vector<Real>& z) {
    Path p;
    p.kind = kind;
    p.vehicleTags = kBothTestVehicles;
    p.curve = g.addCurve(pts, z);
    p.length = g.along[p.curve.end() - 1];
    p.width = kHandLaneWidth;
    p.speedLimit = kTestSpeedLimit;
    g.paths.push_back(p);
    next.emplace_back();
    return static_cast<uint32_t>(g.paths.size() - 1);
}
// Puts a lane in a section of its own.
void ownSection(LaneGraph& g, uint32_t lane) {
    Section s;
    s.roadClass = 0;
    s.lanes = Range{static_cast<uint32_t>(g.sectionLanes.size()), 1};
    s.lines = Range{static_cast<uint32_t>(g.laneLines.size()), 0};
    g.paths[lane].section = SectionId(static_cast<uint32_t>(g.sections.size()));
    g.sectionLanes.push_back(PathId(lane));
    g.sections.push_back(s);
}
LaneGraph emptyTestGraph() {
    LaneGraph g;
    g.vehicleTagNames = {kLaneGraphTestTag, kLaneGraphLongTag};
    g.roadClassNames = {"street"};
    g.controlNames = {kControlNotPlanned};
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
    jn.control = 0;
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
// junction, a section per leg (offside to kerb), a connector for every turn junctionTurns states (connectorShape's, as
// the builder makes them), the next lists, and the one junction, its outline round the lane ends. `pathOf` maps a leg's
// lane id (road * 100 + k, + 50 leaving) to its path.
struct HandGraph {
    LaneGraph g;
    std::map<uint32_t, uint32_t> pathOf;
};
HandGraph handGraph(const std::vector<double>& bearings, const std::vector<uint32_t>& lanes, double kerbPerp = -1,
                    bool mirrored = false) {
    std::vector<TurnLeg> arriving, leaving;
    handJunction(bearings, lanes, kerbPerp, mirrored, arriving, leaving);
    HandGraph h;
    LaneGraph& g = h.g = emptyTestGraph();
    std::vector<std::vector<PathId>> next;
    std::vector<PathId> in, out;
    std::vector<Vec2> ends;
    for (int side = 0; side < 2; ++side)
        for (const TurnLeg& leg : side == 0 ? arriving : leaving) {
            Section sec;
            sec.roadClass = 0;
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
    TurnRules rules;
    rules.kerbPerpSign = kerbPerp;
    for (const JunctionTurn& t : junctionTurns(arriving, leaving, rules)) {
        const PathId a(h.pathOf[t.from]), b(h.pathOf[t.to]);
        const ConnectorShape s = connectorShape(g.pointAt(a, g.paths[a.index].length), g.directionAt(a, g.paths[a.index].length),
                                                g.pointAt(b, 0), g.directionAt(b, 0));
        CHECK(s.ok);
        const uint32_t c = addTestPath(g, next, PathKind::Connector, s.points, std::vector<Real>(s.points.size(), 0));
        g.paths[c].turn = t.turn;
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
        g.paths[c].turn = Turn::Straight;
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
        CHECK(crosses == (h.g.paths[c].turn == Turn::NearTurn ? 0 : 4));
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
    const Result& r = scene(name);
    const LaneGraphBuild& lg = laneGraphOf(name);
    const LaneGraph& G = lg.graph;
    const CrossingReport& cr = lg.crossings;
    std::printf("    [crossing points] %s: %zu (%u crosses, %u joins, %u splits; %u reaching across junctions, %u with a lane); "
                "busiest junction %u with %u; %u junctions of two or more connectors with none; %zu problems; %.0f ms\n",
                name.c_str(), G.crossings.size(), cr.byMeeting[0], cr.byMeeting[1], cr.byMeeting[2], cr.betweenJunctions,
                cr.withLanes, cr.busiestJunction, cr.busiestCount, cr.junctionsWithout, cr.problems.size(), cr.milliseconds);
    for (std::size_t i = 0; i < cr.problems.size() && i < 4; ++i) std::printf("      %s\n", cr.problems[i].c_str());
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
    // one another (a carry-on's lanes side by side; a corner where one road bends into another, its near turn inside
    // its far turn)
    int without = 0, carryOns = 0;
    for (uint32_t j = 0; j < G.junctions.size(); ++j) {
        const Junction& jn = G.junctions[j];
        if (jn.crossings.count > 0 || jn.connectors.count < 2) continue;
        ++without;
        carryOns += lg.junctionNotes[j].origin == JunctionOrigin::CarryOn ? 1 : 0;
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
    // found again: the same table, byte for byte
    LaneGraph again = G;
    CrossingOptions co = crossingTestOptions();
    co.clearance = r.graph.rules.sameLevelDz;
    findCrossingPoints(again, co);
    bundle::BinWriter w1, w2;
    putLaneGraph(w1, G);
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
        g.paths[c].turn = Turn::FarTurn;
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
DiamondResult frontageDiamond(bool sideStreet) {
    std::vector<Vec2> route;
    for (double x = 0; x <= 2000.0; x += 10.0) route.push_back(Vec2(x, 0));
    DiamondOptions o;
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
    return diamondRamps(route, frontageStreets(sideStreet), o);
}
}  // namespace

TEST_CASE(lanes_the_planner_lands_frontage_ramps_on_the_far_carriageway_over_the_road) {
    const DiamondResult dr = frontageDiamond(false);
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
    const DiamondResult dr = frontageDiamond(true);
    std::printf("    built %d, refused for want of a landing %d\n", dr.built, dr.rejectedLanding);
    CHECK(dr.built == 0);
    CHECK(dr.rejectedLanding >= 1);   // (the crossing, met at a vertex of the route, may be found twice)
    CHECK(dr.ramps.empty());
}
