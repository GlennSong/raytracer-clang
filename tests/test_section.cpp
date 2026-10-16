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
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/furniture.h"
#include "../src/engine/procgen/city/room_plan.h"

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

// THE CORNER OFFICE IS THE WHOLE ROUNDED SECTION (Glenn, 2026-10-15): on a rounded plan the ring lays one band along
// each arc's run of chords -- rooms that follow the curve, their glass the chords, their door on the inner curve --
// instead of a sliver office per chord; each has a rectangle inside it for the furniture; the floor stays walkable
// and the building passes the gate.
TEST_CASE(curved_corner_office_is_one_walled_room) {
    const Poly2 plan = Section::rect(56, 40).rounded(6.0).tessellate();
    BuildingParams p;
    p.floors = 20;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 5;
    const std::vector<StoreyPlan> sps = storeyPlans(plan, p);
    const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
    CHECK(core.valid);
    const RoomPlan rp = roomPlan(sps[5].plan, p, core, static_cast<std::size_t>(-1), interiorWallInset(p), 5);
    int curved = 0, framed = 0, curvedDoors = 0;
    for (const Room& r : rp.rooms) {
        if (r.rect.size() <= 4) continue;
        ++curved;
        if (r.frame.size() == 4) ++framed;
        for (std::size_t fi = 0; fi < r.frame.size(); ++fi) {
            const Vec2& q = r.frame[fi];
            const bool in = pointInPolygon(r.rect, q + (centroid(r.rect) - q) * 0.01);
            if (!in) {
                std::printf("    [curved] room %zu-gon frame corner %zu (%.2f, %.2f) outside; frame", r.rect.size(), fi, q.x, q.y);
                for (const Vec2& f : r.frame) std::printf(" (%.2f,%.2f)", f.x, f.y);
                std::printf("; rect");
                for (const Vec2& v : r.rect) std::printf(" (%.2f,%.2f)", v.x, v.y);
                std::printf("\n");
            }
            CHECK(in);
        }
        // a door on a wall that lies on this room's boundary
        for (const RoomWall& w : rp.walls) {
            if (w.doorAt < 0) continue;
            const Vec2 d = w.a + (w.b - w.a) * w.doorAt;
            Real best = 1e9;
            for (std::size_t i = 0; i < r.rect.size(); ++i) {
                const Vec2 a = r.rect[i], b = r.rect[(i + 1) % r.rect.size()];
                const Vec2 ab = b - a;
                const Real t = std::max(Real(0), std::min(Real(1), dot(d - a, ab) / std::max(Real(1e-9), ab.lengthSquared())));
                best = std::min(best, (d - (a + ab * t)).length());
            }
            if (best < 0.05) { ++curvedDoors; break; }
        }
    }
    std::printf("    [curved] %zu rooms, %d curved, %d framed, %d with a door on their curve\n", rp.rooms.size(), curved, framed, curvedDoors);
    CHECK(curved >= 4);
    CHECK(framed == curved);
    CHECK(curvedDoors == curved);
    CHECK(floorIsWalkable(rp, sps[5].plan, core.frame.toWorld({core.length * 0.5, -1.1})));
    // Furnished inside the curve.
    std::vector<PlacedPiece> placed;
    emitFurniture(placed, nullptr, rp, 20.0, p.seed);
    int inCurved = 0;
    for (const Room& r : rp.rooms) {
        if (r.rect.size() <= 4) continue;
        for (const PlacedPiece& pc : placed) {
            const Vec2 at(pc.xform.m[0][3], pc.xform.m[2][3]);
            if (pointInPolygon(r.rect, at)) { ++inCurved; break; }
        }
    }
    std::printf("    [curved] %d of %d curved rooms furnished\n", inCurved, curved);
    CHECK(inCurved == curved);
    const SpaceGraph g = spaceGraphOf(plan, p, 0.0);
    CHECK(unreachableCells(g).empty());
}

TEST_CASE(section_notch_and_chamfer_are_exact_surgery) {
    // A bite out of a rectangle's first edge: an L of six corners, its area the rectangle's less the bite's.
    const Section L = Section::rect(24, 20).notch(0, 0.0, 8.0, 6.0);
    const Poly2 lp = L.tessellate();
    CHECK(lp.size() == 7);   // (the bite starts at the corner: five new corners replace one edge's end, one line left)
    CHECK(isCCW(lp));
    CHECK_APPROX(area(lp), 24.0 * 20.0 - 8.0 * 6.0, 1e-9);
    const Section C = Section::rect(24, 20).chamfer(2, 3.0);
    const Poly2 cp = C.tessellate();
    CHECK(cp.size() == 5);
    CHECK_APPROX(area(cp), 24.0 * 20.0 - 0.5 * 3.0 * 3.0, 1e-9);
}

TEST_CASE(section_subtract_is_the_notch_within_a_millimetre) {
    // Clipper2 on a millimetre grid: a corner rectangle cut away gives the same L as the exact notch.
    const Section bite = Section::rect(8, 6, Vec2(-12 + 4, -10 + 3));
    const Section L = Section::rect(24, 20).subtract(bite);
    const Poly2 lp = L.tessellate();
    CHECK(lp.size() == 6);
    CHECK(isCCW(lp));
    CHECK_APPROX(area(lp), 24.0 * 20.0 - 8.0 * 6.0, 0.05);
    CHECK(L.holes.empty());
    // A hole stays a hole.
    const Section lightwell = Section::rect(24, 20).subtract(Section::rect(6, 6));
    CHECK(lightwell.holes.size() == 1);
    CHECK_APPROX(area(lightwell.tessellate()), 24.0 * 20.0, 0.05);
    CHECK_APPROX(area(lightwell.holesTessellated().front()), 36.0, 0.05);
}

TEST_CASE(section_boolean_retags_arcs) {
    // A rounded slab minus a bite in one straight side: Clipper hands back chords; the four corners are arcs again.
    const Section rounded = Section::rect(40, 24).rounded(5.0);
    const Section cut = rounded.subtract(Section::rect(6, 4, Vec2(0, -12 + 2)));
    int arcs = 0, lines = 0;
    for (const Edge& e : cut.outer) (e.kind == Edge::Kind::Arc ? arcs : lines)++;
    std::printf("    [retag] %d arcs, %d lines after the cut (%zu chords in)\n", arcs, lines, rounded.tessellate(0.1).size());
    // the ring is closed and in order: every edge's end is the next edge's start
    for (std::size_t i = 0; i < cut.outer.size(); ++i) {
        const Edge& e = cut.outer[i];
        const Edge& f = cut.outer[(i + 1) % cut.outer.size()];
        CHECK((e.b - f.a).length() < 1e-6);
    }
    CHECK(lines == 8);
    CHECK(arcs == 4);
    for (const Edge& e : cut.outer)
        if (e.kind == Edge::Kind::Arc) {
            std::printf("    [retag] arc a(%.3f %.3f) b(%.3f %.3f) chord %.3f bulge %.4f len %.4f (true sag %.4f)\n", e.a.x, e.a.y, e.b.x, e.b.y,
                        (e.b - e.a).length(), e.bulge, e.length(), 5.0 - std::sqrt(25.0 - 12.5));
            CHECK_APPROX(e.length(), 5.0 * 3.14159265358979323846 * 0.5, 0.05);
        }
    CHECK_APPROX(area(cut.tessellate(0.05)), area(rounded.tessellate(0.05)) - 24.0, 0.5);
    // ...and a plain rectangle's corners are not arcs.
    CHECK(Section::retagArcs(Section::rect(24, 20).tessellate()).outer.size() == 4);
    for (const Edge& e : Section::retagArcs(Section::rect(24, 20).tessellate()).outer) CHECK(e.kind == Edge::Kind::Line);
}

TEST_CASE(tessellate_for_bays_gives_one_even_bay_per_chord) {
    const Section s = Section::rect(56, 40).rounded(8.0);
    const Poly2 p = s.tessellateForBays(2.0);   // (a 2 m bay: six chords a quarter where the tolerance asks four)
    const Poly2 q = s.tessellate();
    CHECK(p.size() > q.size());   // more chords than the tolerance alone asks
    // every chord of the arcs about a bay long
    const Tessellation t = s.tessellation();
    (void)t;
    int chords = 0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Real L = (p[(i + 1) % p.size()] - p[i]).length();
        if (L > 20.0) continue;   // a straight side
        ++chords;
        CHECK(L > 2.0 * 0.7 && L < 2.0 * 1.3);
    }
    CHECK(chords >= 20);
}

TEST_CASE(a_glass_dome_caps_the_lens_and_a_diagrid_crosses_its_glass) {
    // top = 10 (emitGlassDome): the top outline lofted to a point in glass, the building that much taller, with the
    // deck under it still the roof the stairs reach; diagrid = 2: the facade's verticals replaced by diagonals.
    const Poly2 plan = Section::ngon(16, 14.0).tessellate();
    BuildingParams p;
    p.floors = 12;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 31;
    p.top = 1;
    const BuildingMesh plain = growPlanBuilding(plan, p, 0.0);
    p.top = 10;
    const BuildingMesh domed = growPlanBuilding(plan, p, 0.0);
    CHECK(domed.height > plain.height + 3.0);                   // the rise
    CHECK(domed.height < plain.height + 14.0 * 0.8 + 1.0);      // and no more than the profile gives
    auto partVerts = [](const BuildingMesh& bm, PartId id) {
        std::size_t n = 0;
        for (const RenderMesh& part : bm.parts) if (part.materialIndex == static_cast<int>(id)) n += part.vertices.size();
        return n;
    };
    CHECK(partVerts(domed, PartId::Glass) > partVerts(plain, PartId::Glass));   // the dome is glass
    // the deck under the dome is still walked: the graph's roof cell stands
    const SpaceGraph g = spaceGraphOf(plan, p, 0.0);
    CHECK(g.roof >= 0);
    CHECK(unreachableCells(g).empty());
    p.diagrid = 2;
    const BuildingMesh grid = growPlanBuilding(plan, p, 0.0);
    CHECK(partVerts(grid, PartId::Detail) != partVerts(domed, PartId::Detail));   // a different lattice
    std::printf("    [dome] plain %.1f m, domed %.1f m; glass %zu -> %zu vertices; detail %zu -> %zu with the diagrid\n", plain.height, domed.height,
                partVerts(plain, PartId::Glass), partVerts(domed, PartId::Glass), partVerts(domed, PartId::Detail), partVerts(grid, PartId::Detail));
}

TEST_CASE(every_glass_cap_rises_over_the_deck_the_stairs_reach) {
    // tops 11-14 (emitGlassCap, mass_stack.h Cap::Kind): the spire, the vault, the aperture and the flared lid --
    // each lofted in glass over the top outline, the building that much taller, the deck under it still the roof
    // the stairs reach. The spire is the tallest, the flared lid the lowest.
    const Poly2 plan = {{-12, -9}, {12, -9}, {12, 9}, {-12, 9}};
    BuildingParams p;
    p.floors = 12;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 31;
    p.top = 9;
    const BuildingMesh bare = growPlanBuilding(plan, p, 0.0);
    auto partVerts = [](const BuildingMesh& bm, PartId id) {
        std::size_t n = 0;
        for (const RenderMesh& part : bm.parts) if (part.materialIndex == static_cast<int>(id)) n += part.vertices.size();
        return n;
    };
    Real rise[15] = {};
    for (uint8_t top = 11; top <= 14; ++top) {
        p.top = top;
        const BuildingMesh capped = growPlanBuilding(plan, p, 0.0);
        rise[top] = capped.height - bare.height;
        CHECK(rise[top] > 2.0);
        CHECK(partVerts(capped, PartId::Glass) > partVerts(bare, PartId::Glass));
        CHECK(partVerts(capped, PartId::Detail) > partVerts(bare, PartId::Detail));   // the ribs
        const SpaceGraph g = spaceGraphOf(plan, p, 0.0);
        CHECK(g.roof >= 0);
        CHECK(unreachableCells(g).empty());
    }
    CHECK(rise[11] > rise[12] && rise[11] > rise[13] && rise[11] > rise[14]);   // the spire stands tallest
    CHECK(rise[14] < rise[12]);                                                   // the flared lid is low
    CHECK(rise[13] < rise[12]);                                                   // the aperture stops short of a dome
    std::printf("    [caps] spire %.1f, vault %.1f, aperture %.1f, flared %.1f m over the bare deck\n", rise[11], rise[12], rise[13], rise[14]);
}

