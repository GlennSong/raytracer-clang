// THE SECTION AND THE FACE (architectural toolkit, issue #105, foundation B, step 2a): lines only, and the proof
// that nothing downstream can tell -- a lot's section tessellates to the lot corner for corner, the n-gon is the
// drum's circle, the offset is the grammar's offsetPlan, a flat face is the plan edge's rectangle, and a building
// grown from a section hashes to the building grown from its plan.
#include "test_framework.h"
#include "../src/engine/procgen/city/face.h"
#include "../src/engine/procgen/city/section.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/architect.h"
#include "../src/engine/procgen/city/space_graph.h"
#include "../src/engine/mesh_audit.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

using namespace engine;

namespace {

bool samePoly(const Poly2& a, const Poly2& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i].x != b[i].x || a[i].y != b[i].y) return false;   // bit for bit
    return true;
}

uint64_t hashOf(const BuildingMesh& bm) {
    uint64_t h = 1469598103934665603ull;
    for (const RenderMesh& part : bm.parts)
        for (const Vertex& vx : part.vertices) {
            const double q[3] = {vx.position.x, vx.position.y, vx.position.z};
            for (double d : q) {
                h ^= static_cast<uint64_t>(static_cast<int64_t>(std::llround(d * 4096.0)));
                h *= 1099511628211ull;
            }
        }
    return h;
}

}  // namespace

TEST_CASE(section_from_lot_tessellates_to_the_lot) {
    const Poly2 rect = {{-12, -10}, {12, -10}, {12, 10}, {-12, 10}};
    const Poly2 lPlan = {{-14, -12}, {14, -12}, {14, 0}, {0, 0}, {0, 12}, {-14, 12}};
    for (const Poly2& p : {rect, lPlan}) {
        const Section s = Section::fromLot(p);
        CHECK(s.outer.size() == p.size());
        CHECK(samePoly(s.tessellate(), p));
        for (std::size_t i = 0; i < s.outer.size(); ++i) {
            CHECK(s.outer[i].kind == Edge::Kind::Line);
            CHECK(s.outer[i].b.x == s.outer[(i + 1) % s.outer.size()].a.x && s.outer[i].b.y == s.outer[(i + 1) % s.outer.size()].a.y);
        }
    }
    // A clockwise lot comes out CCW, as every grower insists.
    Poly2 cw = rect;
    std::reverse(cw.begin(), cw.end());
    CHECK(isCCW(Section::fromLot(cw).tessellate()));
}

TEST_CASE(section_rect_and_ngon_are_the_grammars_own) {
    CHECK(samePoly(Section::rect(24, 20).tessellate(), Poly2{{-12, -10}, {12, -10}, {12, 10}, {-12, 10}}));
    // The drum's circle as city_lots draws it: sixteen corners from angle 0, counter-clockwise.
    const Vec2 cc(3.5, -2.0);
    const Real rad = 11.0;
    Poly2 circ;
    for (int k = 0; k < 16; ++k) {
        const Real a2 = 2.0 * 3.14159265358979323846 * k / 16;
        circ.push_back(cc + Vec2(std::cos(a2), std::sin(a2)) * rad);
    }
    CHECK(samePoly(Section::ngon(16, rad, cc).tessellate(), circ));
}

TEST_CASE(section_offset_is_the_grammars_offset_plan) {
    const Poly2 lPlan = {{-14, -12}, {14, -12}, {14, 0}, {0, 0}, {0, 12}, {-14, 12}};
    for (const Real d : {Real(0.55), Real(2.0), Real(-0.75)})
        CHECK(samePoly(Section::fromLot(lPlan).offset(d).tessellate(), offsetPlanPublic(lPlan, d)));
    const Section holed = Section::fromLot(lPlan).withHole(Section::rect(4, 4));
    CHECK(holed.holes.size() == 1);
    CHECK(holed.holesTessellated().front().size() == 4);
    CHECK(holed.offset(1.0).holes.size() == 1);
}

TEST_CASE(flat_face_is_the_plan_edges_rectangle) {
    const Poly2 plan = {{-12, -10}, {12, -10}, {12, 10}, {-12, 10}};
    const Face f = Face::ofEdge(plan, 1, 4.5, 3.2);   // the +x edge, (12,-10) -> (12,10)
    CHECK(f.kind == Face::Kind::Flat);
    CHECK_APPROX(f.width, 20.0, 1e-12);
    CHECK_APPROX(f.height, 3.2, 1e-12);
    const Vec3 bl = f.at(0, 0), tr = f.at(f.width, f.height);
    CHECK_APPROX(bl.x, 12.0, 1e-12); CHECK_APPROX(bl.y, 4.5, 1e-12); CHECK_APPROX(bl.z, -10.0, 1e-12);
    CHECK_APPROX(tr.x, 12.0, 1e-12); CHECK_APPROX(tr.y, 7.7, 1e-12); CHECK_APPROX(tr.z, 10.0, 1e-12);
    CHECK_APPROX(f.normal(1, 1).x, 1.0, 1e-12);   // outward: to the right of a->b on a CCW plan
    CHECK_APPROX(f.normal(1, 1).z, 0.0, 1e-12);
}

TEST_CASE(a_section_grows_what_its_plan_grows) {
    const Poly2 lPlan = {{-14, -12}, {14, -12}, {14, 0}, {0, 0}, {0, 12}, {-14, 12}};
    BuildingParams p;
    p.floors = 9;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 5;
    p.setbackFloors = 4;
    p.setbackEvery = 1.5;
    const uint64_t fromPlan = hashOf(growPlanBuilding(lPlan, p, 1.5));
    const uint64_t fromSection = hashOf(growSectionBuilding(Section::fromLot(lPlan), p, 1.5));
    CHECK(fromPlan == fromSection);
    CHECK(hashOf(growSectionBuilding(Section::rect(24, 20), p, 1.5)) == hashOf(growPlanBuilding(Section::rect(24, 20).tessellate(), p, 1.5)));
}

TEST_CASE(rounded_corners_are_arcs_tessellated_within_tolerance) {
    const Section s = Section::rect(24, 20).rounded(5.0);
    int lines = 0, arcs = 0;
    for (const Edge& e : s.outer) (e.kind == Edge::Kind::Arc ? arcs : lines)++;
    CHECK(lines == 4);
    CHECK(arcs == 4);
    for (const Edge& e : s.outer) if (e.kind == Edge::Kind::Arc) CHECK_APPROX(e.length(), 5.0 * 3.14159265358979323846 * 0.5, 1e-6);
    const Real tol = 0.25;
    const Tessellation t = s.tessellation(tol);
    CHECK(t.points.size() == t.edgeOf.size());
    CHECK(t.points.size() > 8);
    CHECK(isCCW(t.points));
    // Every point on the arcs' circles, inside the rectangle, and every arc chord within the tolerance.
    const Vec2 centres[4] = {{-7, -5}, {7, -5}, {7, 5}, {-7, 5}};
    for (std::size_t i = 0; i < t.points.size(); ++i) {
        const Vec2& p = t.points[i];
        CHECK(std::fabs(p.x) <= 12.0 + 1e-9 && std::fabs(p.y) <= 10.0 + 1e-9);
        if (s.outer[static_cast<std::size_t>(t.edgeOf[i])].kind != Edge::Kind::Arc) continue;
        Real best = 1e9;
        for (const Vec2& c : centres) best = std::min(best, std::fabs((p - c).length() - 5.0));
        CHECK(best < 1e-6);
        CHECK(std::fabs(p.x) >= 7.0 - 1e-9 && std::fabs(p.y) >= 5.0 - 1e-9);   // in the corner, not curling inward
        const Vec2& q = t.points[(i + 1) % t.points.size()];
        if (t.edgeOf[(i + 1) % t.points.size()] == t.edgeOf[i]) {
            const Real chord = (q - p).length();
            const Real sag = 5.0 - std::sqrt(std::max(Real(0), 25.0 - chord * chord * 0.25));
            CHECK(sag <= tol + 1e-6);
        }
    }
    // The arcs' ends are corners of the polygon (the tangent points), so a facade knows where an arc starts.
    bool tangentKept = false;
    for (const Vec2& p : t.points) if (std::fabs(p.x - 7.0) < 1e-9 && std::fabs(p.y + 10.0) < 1e-9) tangentKept = true;
    CHECK(tangentKept);
}

TEST_CASE(a_stadium_is_two_semicircles) {
    const Section s = Section::rect(30, 16).rounded(8.0);   // half the depth: the short edges vanish
    if (std::getenv("RT_SECTION_DEBUG")) {
        for (const Edge& e : s.outer) std::printf("    %s a(%.4f %.4f) b(%.4f %.4f) bulge %.4f len %.4f\n", e.kind == Edge::Kind::Arc ? "arc " : "line", e.a.x, e.a.y, e.b.x, e.b.y, e.bulge, e.length());
        for (const Vec2& q : s.tessellate(0.1)) std::printf("      (%.3f %.3f)\n", q.x, q.y);
    }
    int lines = 0, arcs = 0;
    for (const Edge& e : s.outer) (e.kind == Edge::Kind::Arc ? arcs : lines)++;
    CHECK(lines == 2);
    CHECK(arcs == 4);
    const Poly2 p = s.tessellate(0.1);
    CHECK(isCCW(p));
    Real minX = 1e9, maxX = -1e9;
    for (const Vec2& q : p) { minX = std::min(minX, q.x); maxX = std::max(maxX, q.x); }
    CHECK_APPROX(minX, -15.0, 1e-6);
    CHECK_APPROX(maxX, 15.0, 1e-6);
    CHECK_APPROX(area(p), 14.0 * 16.0 + 3.14159265358979323846 * 64.0, 5.0);   // short by the chords' slack only
}

TEST_CASE(a_section_moves_into_a_site_frame) {
    const Section s = Section::rect(10, 4).rounded(1.0);
    const Vec2 o(100, 50), u(0, 1), v(-1, 0);   // a quarter turn
    const Poly2 a = s.tessellate(), b = s.transformed(o, u, v).tessellate();
    CHECK(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        const Vec2 w = o + u * a[i].x + v * a[i].y;
        CHECK_APPROX(w.x, b[i].x, 1e-9);
        CHECK_APPROX(w.y, b[i].y, 1e-9);
    }
    CHECK(isCCW(b));
}

// THE FIRST SECTION BUILDING (Glenn, 2026-10-15: "one of these new buildings using cells working with a furnished
// interior and an exterior that fits in the lot"): the rounded tower recipe on the rounded plan the lot pass fits to
// a lot like the lab's -- every cell reached from the street up to the roof, the upper floors furnished, the
// exterior and the interior clean through the mesh audit.
TEST_CASE(rounded_tower_is_furnished_and_walkable_base_to_roof) {
    BuildingRecipe rc;
    CHECK(architectRecipeByName("rounded_tower", 7, 0.25, 40, 2400, rc));
    CHECK(rc.massing == BuildingRecipe::Massing::Section);
    CHECK(rc.section.kind == BuildingRecipe::SectionSpec::Kind::Rounded);
    BuildingParams p = rc.params;
    p.openDoorway = true;
    const Section sec = Section::rect(56, 40).rounded(std::min(rc.section.radius, Real(12.0)));
    const Poly2 plan = sec.tessellate();
    CHECK(plan.size() > 8);
    CHECK(isCCW(plan));
    int arcs = 0;
    for (const Edge& e : sec.outer) if (e.kind == Edge::Kind::Arc) ++arcs;
    CHECK(arcs == 4);
    // The gate.
    const SpaceGraph g = spaceGraphOf(plan, p, 0.0);
    CHECK(g.modelled);
    CHECK(g.roof >= 0);
    const std::vector<int> bad = unreachableCells(g);
    std::printf("    [rounded] %d storeys, %zu cells, %zu portals, %zu unreachable, %zu-gon plan, %d floors\n", g.storeys, g.cells.size(),
                g.portals.size(), bad.size(), plan.size(), p.floors);
    CHECK(bad.empty());
    // Furnished: the lobby's shops and the office floors above.
    const BuildingMesh lobby = growInterior(plan, p, 0.0, nullptr, 0, 1);
    const BuildingMesh floor5 = growInterior(plan, p, 0.0, nullptr, 5, 6);
    std::printf("    [rounded] furniture: lobby %zu pieces (%zu shop units), storey 5 %zu pieces\n", lobby.furniture.size(), lobby.units.size(),
                floor5.furniture.size());
    CHECK(lobby.furniture.size() > 20);
    CHECK(floor5.furniture.size() > 20);
    // Clean meshes: the exterior and the interior through the audit.
    MeshAudit ext, in;
    for (const RenderMesh& part : growPlanBuilding(plan, p, 0.0).parts) ext.add(auditMesh(part));
    for (const RenderMesh& part : growInterior(plan, p, 0.0).parts) in.add(auditMesh(part));
    std::printf("    [rounded] audit: exterior %ld tris, %ld inward, %ld degenerate; interior %ld tris, %ld inward, %ld degenerate\n",
                ext.tris, ext.inward, ext.degenerate, in.tris, in.inward, in.degenerate);
    CHECK(ext.tris > 1000);
    CHECK(ext.inward == 0);
    CHECK(ext.degenerate == 0);
    CHECK(in.inward == 0);
    CHECK(in.degenerate == 0);
    // The roof's contents in the graph: the HVAC units as holes in the deck, the helipad a cell of its own with a
    // ramp portal from the roof, the gate still green.
    const BuildingMesh ext2 = growPlanBuilding(plan, p, 0.0);
    int hvac = 0, pads = 0, ramps = 0;
    for (const RoofItem& ri : ext2.roof) {
        if (ri.kind == RoofItem::Kind::Hvac) ++hvac;
        if (ri.kind == RoofItem::Kind::Helipad) ++pads;
        if (ri.kind == RoofItem::Kind::Ramp) ++ramps;
    }
    const SpaceGraph gr = spaceGraphOf(plan, p, 0.0, &ext2.roof);
    int padCells = 0, rampPortals = 0;
    for (const Cell& c : gr.cells) if (c.kind == CellKind::Pad) ++padCells;
    for (const Portal& po : gr.portals) if (po.kind == Portal::Kind::Ramp) ++rampPortals;
    std::printf("    [rounded] roof: %d hvac, %d pad(s), %d ramp(s); graph: %d pad cells, %d ramp portals, %zu unreachable\n", hvac, pads, ramps,
                padCells, rampPortals, unreachableCells(gr).size());
    CHECK(pads == 1);
    CHECK(ramps == 1);
    CHECK(padCells == 1);
    CHECK(rampPortals == 1);
    CHECK(unreachableCells(gr).empty());
}
